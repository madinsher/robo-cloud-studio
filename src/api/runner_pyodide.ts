/**
 * Pyodide runner for station Python programs (contract I5, browser, no server needed).
 *
 * The drop-in API (python/robodk + the legacy python/robolink module) is written into Pyodide's file system and its
 * in-process transport (robodk.robolink._TRANSPORT) calls executeRpc on the page's Robolink synchronously, so
 * `from robolink import *; RDK = Robolink()` drives the live station without any socket. While a script runs,
 * rdk.exec.syncDepth > 0: blocking moves, WaitMove/WaitFinished and time.sleep()/pause() advance the station clock
 * synchronously (virtual time), so polling scripts make progress although the script runs on the page's thread.
 * Calls that can only complete asynchronously in the browser (a Python post processor in MakeProgram) raise with a
 * clear message; run such scripts on the studio server (CPython runner) instead.
 *
 * Pyodide (~10 MB) is loaded from jsDelivr on the first script call, not when the runner is installed.
 */
import robolinkSrc from '../../python/robodk/robolink.py?raw';
import officialSrc from '../../python/robodk/_official.py?raw';
import robomathSrc from '../../python/robodk/robomath.py?raw';
import robodialogsSrc from '../../python/robodk/robodialogs.py?raw';
import robofileioSrc from '../../python/robodk/robofileio.py?raw';
import robolinkutilsSrc from '../../python/robodk/robolinkutils.py?raw';
import robodkInitSrc from '../../python/robodk/__init__.py?raw';
import legacyRobolinkSrc from '../../python/robolink/__init__.py?raw';
import { executeRpc, RpcRequest } from './rpc';
import { pythonSource } from './exec';
import { registerScriptRunner } from '../core/motion/runtime';
import type { ScriptContext, ScriptRunner } from './exec';
import type { Robolink } from './robolink';

export const PYODIDE_URL = 'https://cdn.jsdelivr.net/pyodide/v0.26.4/full/pyodide.js';

/** Python files of the drop-in, by path inside the Pyodide file system. */
export const DROPIN_FILES: Record<string, string> = {
  '/studio/robodk/__init__.py': robodkInitSrc,
  '/studio/robodk/robolink.py': robolinkSrc,
  '/studio/robodk/_official.py': officialSrc,
  '/studio/robodk/robomath.py': robomathSrc,
  '/studio/robodk/robodialogs.py': robodialogsSrc,
  '/studio/robodk/robofileio.py': robofileioSrc,
  '/studio/robodk/robolinkutils.py': robolinkutilsSrc,
  '/studio/robolink/__init__.py': legacyRobolinkSrc,
};

/** Bootstrap run once per interpreter: transport, virtual-time sleep, quit()/exit() that do not close stdin. */
export const BOOTSTRAP = `
import sys, time, json, builtins
if '/studio' not in sys.path:
    sys.path.insert(0, '/studio')
import studio_host
import robodk.robolink as _rl
_rl._TRANSPORT = lambda text: str(studio_host.transport(text))
_rl.DEFAULT_URL = 'inprocess://studio'
def _sleep(seconds):
    studio_host.sleep(float(seconds))
time.sleep = _sleep
def _quit(code=None):
    raise SystemExit(code)
builtins.quit = _quit
builtins.exit = _quit
import robolink
def __studio_run(code, file, argv):
    import traceback
    sys.argv = [file] + [str(a) for a in argv]
    g = {'__name__': '__main__', '__file__': file, '__builtins__': builtins}
    status, err = 0, ''
    try:
        exec(compile(code, file, 'exec'), g)
    except SystemExit as e:
        c = e.code
        status = 0 if c is None else (c if isinstance(c, int) else 1)
        if not isinstance(c, (int, type(None))):
            err = str(c)
    except BaseException:
        status, err = 1, traceback.format_exc()
    finally:
        for v in list(g.values()):
            if isinstance(v, _rl.Robolink):
                try:
                    v.Disconnect()
                except Exception:
                    pass
        g.clear()
    return json.dumps([status, err])
`;

export interface PyodideRunnerOptions {
  rdk: Robolink;
  /** Browser app (for __app__ calls); optional. */
  app?: any;
  onOutput?: (name: string, line: string, stream: 'stdout' | 'stderr') => void;
  /** Custom loader (tests): returns a Pyodide-like object. */
  load?: () => Promise<any>;
}

export interface PyodideRunner extends ScriptRunner {
  /** Outputs of finished runs, most recent last. */
  history: Array<{ name: string; args: string[]; code: number; output: string }>;
  /** true once Pyodide is loaded. */
  loaded(): boolean;
}

/** Synchronous in-process transport: one JSON-RPC request -> response JSON, waits complete in virtual time. */
export function inProcessTransport(rdk: Robolink, app?: any): (text: string) => string {
  return (text: string) => {
    let req: RpcRequest;
    try { req = JSON.parse(text); } catch { return JSON.stringify({ id: null, error: 'invalid JSON' }); }
    rdk.exec.syncDepth++;
    try {
      const res = executeRpc(rdk, req, { app });
      if (res.result && typeof res.result.then === 'function') {
        res.result.catch(() => {});
        return JSON.stringify({ id: req.id, error: `${req.method} completes asynchronously in the browser and cannot be awaited by a Pyodide script (run the script on the studio server)` });
      }
      return JSON.stringify(res);
    } finally { rdk.exec.syncDepth--; }
  };
}

async function loadFromCdn(): Promise<any> {
  const w = globalThis as any;
  if (!w.loadPyodide) {
    if (typeof document === 'undefined') throw new Error('Pyodide needs a browser (no document)');
    await new Promise<void>((resolve, reject) => { const s = document.createElement('script'); s.src = PYODIDE_URL; s.onload = () => resolve(); s.onerror = () => reject(new Error('Failed to load Pyodide')); document.head.appendChild(s); });
  }
  return w.loadPyodide({ indexURL: PYODIDE_URL.replace(/pyodide\.js$/, '') });
}

export function createPyodideRunner(opts: PyodideRunnerOptions): PyodideRunner {
  let pyPromise: Promise<any> | null = null;
  let current: { name: string; out: string } | null = null;
  const history: PyodideRunner['history'] = [];
  const emit = (s: string, stream: 'stdout' | 'stderr') => {
    if (!current) return;
    current.out += s + '\n';
    if (opts.onOutput && s.trim()) opts.onOutput(current.name, s, stream);
  };
  const init = () => (pyPromise ??= (async () => {
    const py = await (opts.load ?? loadFromCdn)();
    py.FS.mkdirTree('/studio/robodk');
    py.FS.mkdirTree('/studio/robolink');
    for (const [p, src] of Object.entries(DROPIN_FILES)) py.FS.writeFile(p, src);
    py.registerJsModule('studio_host', {
      transport: inProcessTransport(opts.rdk, opts.app),
      sleep: (s: number) => { opts.rdk.exec.syncDepth++; try { opts.rdk.exec.advance(Math.max(0, Number(s) || 0)); } finally { opts.rdk.exec.syncDepth--; } },
    });
    py.setStdout?.({ batched: (s: string) => emit(s, 'stdout') });
    py.setStderr?.({ batched: (s: string) => emit(s, 'stderr') });
    await py.runPythonAsync(BOOTSTRAP);
    return py;
  })());
  // scripts run one at a time on the page's thread
  let chain: Promise<unknown> = Promise.resolve();
  const runner = ((ctx: ScriptContext) => {
    const code = pythonSource(ctx);
    if (code == null) return Promise.resolve(false as any);
    const job = chain.then(async () => {
      const py = await init();
      current = { name: ctx.name, out: '' };
      py.globals.set('__studio_code', code);
      py.globals.set('__studio_file', `${ctx.name.replace(/[^\w.-]+/g, '_') || 'script'}.py`);
      py.globals.set('__studio_argv', JSON.stringify(ctx.args));
      const [status, err] = JSON.parse(String(await py.runPythonAsync('__studio_run(__studio_code, __studio_file, json.loads(__studio_argv))'))) as [number, string];
      const out = current.out + (err ? `\n${err}` : '');
      current = null;
      history.push({ name: ctx.name, args: ctx.args, code: status, output: out });
      if (history.length > 200) history.shift();
      if (status !== 0) {
        const lines = out.trim().split(/\r?\n/);
        throw new Error(`${ctx.name}${ctx.args.length ? `(${ctx.args.join(',')})` : ''} exited with ${status}: ${[...lines].reverse().find((l) => /Error|Exception/.test(l)) ?? lines[lines.length - 1] ?? ''}`);
      }
    });
    chain = job.catch(() => {});
    return job;
  }) as PyodideRunner;
  runner.history = history;
  runner.loaded = () => !!pyPromise;
  return runner;
}

/**
 * Install the Pyodide runner for a page's Robolink: API RunCode/RunProgram of PROGRAM_PYTHON items and program calls
 * (through the station runtime, when one is attached to rdk.exec). Returns the runner.
 */
export function installPyodideRunner(rdk: Robolink, opts: Omit<PyodideRunnerOptions, 'rdk'> & { global?: boolean } = {}): PyodideRunner & { uninstall(): void } {
  const runner = createPyodideRunner({ ...opts, rdk }) as PyodideRunner & { uninstall(): void };
  rdk.exec.setScriptRunner(runner);
  (rdk.exec as any).scriptRunnerKind = 'pyodide';
  // programs played by the app's StationRuntime (not through rdk.exec) reach the runner through the global registry
  const off = opts.global === false ? () => {} : registerScriptRunner(runner as any);
  runner.uninstall = () => { off(); if ((rdk.exec as any).scriptRunnerKind === 'pyodide') { rdk.exec.setScriptRunner(null); (rdk.exec as any).scriptRunnerKind = null; } };
  return runner;
}
