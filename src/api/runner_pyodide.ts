/**
 * Pyodide runner for station Python programs (contract I5, browser, no server needed).
 *
 * The drop-in API (python/robodk + the legacy python/robolink module) is written into Pyodide's file system and its
 * in-process transport (robodk.robolink._TRANSPORT) calls executeRpc on the page's Robolink synchronously, so
 * `from robolink import *; RDK = Robolink()` drives the live station without any socket.
 *
 * Two execution modes:
 *  - cooperative (default where the browser has WebAssembly JSPI, e.g. Chrome/Edge 137+): every script is a coroutine
 *    of the one interpreter (pyodide.ffi.run_sync). pause()/time.sleep() wait in real time (scaled by the simulation
 *    speed) and blocking API calls (moves, WaitMove, WaitFinished) wait for the station's realtime clock, both without
 *    blocking the page; scripts run concurrently like RoboDK's script processes, so RoboDK "background" scripts with
 *    `while True:` loops (conveyor, sensor and gripper simulations of Library 06.c-06.f) run beside the programs.
 *    A script whose calling run stops (Stop, Reset, station closed) is ended at its next API call or pause.
 *  - synchronous (no JSPI): rdk.exec.syncDepth > 0 while a script runs, so blocking moves, waits and pause() advance
 *    the station clock synchronously (virtual time). One script at a time; a script that keeps the page busy longer
 *    than `syncBudgetS` is stopped with a clear message instead of freezing the tab.
 * Calls that can only complete asynchronously in the browser (a Python post processor in MakeProgram) raise with a
 * clear message in the synchronous mode; run such scripts on the studio server (CPython runner) instead.
 *
 * Pyodide (~10 MB) is loaded from jsDelivr when a station with Python programs is opened (or on the first script
 * call); while it loads, the live clock of runs waiting for a script stands still (StationRuntime.holding()).
 */
import robolinkSrc from '../../python/robodk/robolink.py?raw';
import officialSrc from '../../python/robodk/_official.py?raw';
import robomathSrc from '../../python/robodk/robomath.py?raw';
import robodialogsSrc from '../../python/robodk/robodialogs.py?raw';
import robofileioSrc from '../../python/robodk/robofileio.py?raw';
import robolinkutilsSrc from '../../python/robodk/robolinkutils.py?raw';
import robodkInitSrc from '../../python/robodk/__init__.py?raw';
import legacyRobolinkSrc from '../../python/robolink/__init__.py?raw';
import threadsSrc from '../../python/robodk/_threads_pyodide.py?raw';
import { executeRpc, encodeValue, RpcRequest } from './rpc';
import { pythonSource } from './exec';
import { registerScriptRunner } from '../core/motion/runtime';
import { ItemType } from '../core/items/item';
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
  '/studio/robodk/_threads_pyodide.py': threadsSrc,
};

/**
 * Bootstrap run once per interpreter: transport, sleep, per-script output and stopwatch, quit()/exit() that do not
 * close stdin. Host calls carry the id of the calling script (found on the Python stack: concurrent coroutines each
 * have their own), and may answer with a promise, awaited through run_sync (cooperative mode only).
 */
export const BOOTSTRAP = `
import sys, time, json, builtins
if '/studio' not in sys.path:
    sys.path.insert(0, '/studio')
import studio_host
import robodk.robolink as _rl
try:
    from pyodide.ffi import run_sync as _run_sync
except Exception:
    _run_sync = None
class StudioScriptStopped(BaseException):
    """Raised inside a station script whose calling program was stopped (Stop / Reset / station closed)."""
_ABORT = '\\x00abort:'
def _studio_rid():
    f = sys._getframe(1)
    while f is not None:
        if f.f_code is __studio_run.__code__:
            return f.f_locals.get('run_id', 0)
        f = f.f_back
    return 0
def _studio_wait(r):
    if r is not None and hasattr(r, 'then'):
        r = _run_sync(r)
    if isinstance(r, str) and r.startswith(_ABORT):
        raise StudioScriptStopped(r[len(_ABORT):])
    return r
_rl._TRANSPORT = lambda text: str(_studio_wait(studio_host.transport(text, _studio_rid())))
_rl.DEFAULT_URL = 'inprocess://studio'
def _sleep(seconds):
    _studio_wait(studio_host.sleep(float(seconds), _studio_rid()))
time.sleep = _sleep
class _StudioOut(object):
    def __init__(self, stream):
        self.stream = stream
    def write(self, s):
        if s:
            studio_host.out(_studio_rid(), str(s), self.stream)
        return len(s)
    def flush(self):
        pass
    def isatty(self):
        return False
sys.stdout, sys.stderr = _StudioOut('stdout'), _StudioOut('stderr')
def _quit(code=None):
    raise SystemExit(code)
builtins.quit = _quit
builtins.exit = _quit
import robolink
import robodk.robomath as _rm
# tic()/toc() per script (RoboDK runs every script in its own process: one stopwatch each)
_tics = {}
def _tic():
    _tics[_studio_rid()] = time.time()
def _toc():
    t0 = _tics.get(_studio_rid())
    if t0 is None:
        print("Toc: start time not set")
        return -1
    return time.time() - t0
for _m in list(sys.modules.values()):
    if getattr(_m, '__name__', '').split('.')[0] in ('robodk', 'robolink') and getattr(_m, 'tic', None) is _rm.tic:
        _m.tic, _m.toc = _tic, _toc
def __studio_run(code, file, argv, run_id=0):
    import traceback
    sys.argv = [file] + [str(a) for a in argv]
    g = {'__name__': '__main__', '__file__': file, '__builtins__': builtins}
    status, err, stopped = 0, '', 0
    try:
        exec(compile(code, file, 'exec'), g)
    except SystemExit as e:
        c = e.code
        status = 0 if c is None else (c if isinstance(c, int) else 1)
        if not isinstance(c, (int, type(None))):
            err = str(c)
    except StudioScriptStopped as e:
        status, err, stopped = (0, '', 1) if not str(e) else (1, str(e), 0)
    except BaseException:
        status, err = 1, traceback.format_exc()
    finally:
        for v in list(g.values()):
            if isinstance(v, _rl.Robolink):
                try:
                    v.Disconnect()
                except BaseException:
                    pass
        g.clear()
        _tics.pop(run_id, None)
    return json.dumps([status, err, stopped])
def __studio_job(run_id):
    job = json.loads(str(studio_host.job(run_id)))
    return __studio_run(job['code'], job['file'], job['argv'], run_id)
# threading.Thread as coroutines of this interpreter (JSPI): python/robodk/_threads_pyodide.py (Library 08.a)
try:
    import robodk._threads_pyodide as _studio_threads
    _studio_threads.install(globals())
except Exception:
    pass
`;

export interface PyodideRunnerOptions {
  rdk: Robolink;
  /** Browser app (for __app__ calls, the simulation speed and preloading when a station opens); optional. */
  app?: any;
  onOutput?: (name: string, line: string, stream: 'stdout' | 'stderr') => void;
  /** Custom loader (tests): returns a Pyodide-like object. */
  load?: () => Promise<any>;
  /**
   * Cooperative mode (concurrent coroutines, real-time waits that do not block the page). Default: on when the loaded
   * Pyodide reports WebAssembly JSPI support (pyodide.ffi.run_sync works), else the synchronous virtual-time mode.
   */
  cooperative?: boolean;
  /** Synchronous mode: a script busy on the page thread longer than this (wall seconds) is stopped (default 30). */
  syncBudgetS?: number;
}

export interface PyodideRunner extends ScriptRunner {
  /** Outputs of finished runs, most recent last. */
  history: Array<{ name: string; args: string[]; code: number; output: string; stopped?: boolean }>;
  /** true once Pyodide loading has started. */
  loaded(): boolean;
  /** true while Pyodide is downloading / starting (the station clock of runs waiting for a script holds). */
  loading(): boolean;
  /** Start loading Pyodide now (a station with Python programs was opened). */
  preload(): Promise<void>;
  /** Scripts running now (names). */
  running(): string[];
  /** true when scripts run as cooperative coroutines (JSPI). Known once Pyodide is loaded. */
  cooperative(): boolean;
}

/** Marker a host call answers with when the calling script must stop (Python raises StudioScriptStopped). */
const ABORT = '\x00abort:';

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

/**
 * Cooperative in-process transport: the call runs at once with realtime waits (rdk.exec.asyncDepth > 0); a call that
 * completes later (blocking move, WaitMove, WaitFinished, Pause, MakeProgram with a Python post) answers with a promise
 * of the response JSON, which the script awaits through run_sync while the page keeps running.
 */
export function cooperativeTransport(rdk: Robolink, app?: any): (text: string) => string | Promise<string> {
  return (text: string) => {
    let req: RpcRequest;
    try { req = JSON.parse(text); } catch { return JSON.stringify({ id: null, error: 'invalid JSON' }); }
    let res;
    rdk.exec.asyncDepth++;
    try { res = executeRpc(rdk, req, { app }); } finally { rdk.exec.asyncDepth--; }
    if (res.result && typeof res.result.then === 'function') {
      return Promise.resolve(res.result).then((v) => JSON.stringify({ id: req.id, result: encodeValue(v) }), (e) => JSON.stringify({ id: req.id, error: String(e?.message ?? e) }));
    }
    return JSON.stringify(res);
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

/** WebAssembly JSPI support of a loaded Pyodide (run_sync works). */
function jspiOf(py: any): boolean {
  const m = py?._module ?? py?._api?._module;
  return !!(m && m.jspiSupported === true);
}

interface ScriptRun {
  id: number; name: string; args: string[]; out: string; partial: Record<string, string>;
  abort?: { aborted: boolean }; yielded: boolean;
  /** Time of the script's first host call (the synchronous-mode budget runs from there). */
  t0: number; code: string; file: string;
}

export function createPyodideRunner(opts: PyodideRunnerOptions): PyodideRunner {
  const rdk = opts.rdk;
  let pyPromise: Promise<any> | null = null;
  let ready = false;
  let coop = false;
  const history: PyodideRunner['history'] = [];
  const runs = new Map<number, ScriptRun>();
  let seq = 0;
  let last: ScriptRun | null = null;
  const budgetMs = (opts.syncBudgetS ?? 30) * 1000;
  const now = () => (typeof performance !== 'undefined' ? performance.now() : Date.now());
  const runOf = (rid: unknown) => runs.get(Number(rid)) ?? last;
  const emitLine = (run: ScriptRun | null, s: string, stream: 'stdout' | 'stderr') => {
    if (!run) return;
    run.out += s + '\n';
    if (opts.onOutput && s.trim()) opts.onOutput(run.name, s, stream);
  };
  // print() output arrives in pieces: complete lines are reported per script
  const out = (rid: unknown, s: string, stream: 'stdout' | 'stderr') => {
    const run = runOf(rid);
    if (!run) return;
    const parts = ((run.partial[stream] ?? '') + String(s)).split('\n');
    run.partial[stream] = parts.pop() ?? '';
    for (const line of parts) emitLine(run, line, stream);
  };
  const flush = (run: ScriptRun) => { for (const k of ['stdout', 'stderr'] as const) if (run.partial[k]) { emitLine(run, run.partial[k], k); run.partial[k] = ''; } };
  /** The page's simulation speed applies to scripts (RDK.SimulationSpeed(), pause() durations, the API ticker). */
  const syncSpeed = () => {
    const s = Number(opts.app?.simSpeed);
    if (Number.isFinite(s) && s > 0 && rdk.simSpeed !== s) { rdk.simSpeed = s; rdk.exec.speed = s; }
  };
  /** Synchronous mode: stop a script that keeps the page thread busy (an endless background loop would freeze the tab). */
  const overBudget = (run: ScriptRun | null): string | null =>
    run && !coop && now() - (run.t0 ||= now()) > budgetMs
      ? `${ABORT}${run.name} was stopped after ${Math.round(budgetMs / 1000)} s: it keeps running (a background loop), and this browser cannot run station scripts beside the simulation (no WebAssembly JSPI). Use Chrome or Edge 137+, or the studio server.`
      : null;
  const stopNow = () => (coop ? new Promise<string>((r) => setTimeout(() => r(ABORT), 0)) : ABORT);
  const syncTransport = inProcessTransport(rdk, opts.app);
  const asyncTransport = cooperativeTransport(rdk, opts.app);
  const host = {
    transport: (text: string, rid?: number) => {
      const run = runOf(rid);
      if (run?.abort?.aborted) return stopNow();
      const over = overBudget(run);
      if (over) return over;
      syncSpeed();
      if (!coop) return syncTransport(text);
      const r = asyncTransport(text);
      if (typeof r !== 'string' && run) run.yielded = true;
      return r;
    },
    sleep: (s: number, rid?: number) => {
      const run = runOf(rid);
      if (run?.abort?.aborted) return stopNow();
      const over = overBudget(run);
      if (over) return over;
      syncSpeed();
      const sec = Math.max(0, Number(s) || 0);
      if (!coop) { rdk.exec.syncDepth++; try { rdk.exec.advance(sec); } finally { rdk.exec.syncDepth--; } return null; }
      // real time scaled by the simulation speed (RoboDK scripts sleep in real time: the same at 1x), always yielding
      if (run) run.yielded = true;
      const ms = (sec * 1000) / Math.max(1e-3, rdk.exec.speed || 1);
      return new Promise((r) => setTimeout(() => r(run?.abort?.aborted ? ABORT : null), ms));
    },
    out,
    job: (rid: number) => { const run = runs.get(Number(rid)); return JSON.stringify(run ? { code: run.code, file: run.file, argv: run.args } : { code: '', file: 'script.py', argv: [] }); },
  };
  const init = () => (pyPromise ??= (async () => {
    const py = await (opts.load ?? loadFromCdn)();
    py.FS.mkdirTree('/studio/robodk');
    py.FS.mkdirTree('/studio/robolink');
    for (const [p, src] of Object.entries(DROPIN_FILES)) py.FS.writeFile(p, src);
    py.registerJsModule('studio_host', host);
    py.setStdout?.({ batched: (s: string) => emitLine(last, s, 'stdout') });
    py.setStderr?.({ batched: (s: string) => emitLine(last, s, 'stderr') });
    await py.runPythonAsync(BOOTSTRAP);
    coop = opts.cooperative ?? jspiOf(py);
    ready = true;
    return py;
  })().catch((e) => {
    // a failed download must not leave the runner "loading" (the clock of waiting runs would hold for good): the
    // next script call tries again
    pyPromise = null;
    throw e;
  }));
  const exec1 = async (ctx: ScriptContext, code: string): Promise<void> => {
    const py = await init();
    const run: ScriptRun = { id: ++seq, name: ctx.name, args: ctx.args, out: '', partial: {}, abort: (ctx as any).abort, yielded: false, t0: 0, code, file: `${ctx.name.replace(/[^\w.-]+/g, '_') || 'script'}.py` };
    runs.set(run.id, run);
    last = run;
    let res: string;
    try { res = String(await py.runPythonAsync(`__studio_job(${run.id})`)); } finally { runs.delete(run.id); if (last === run) last = [...runs.values()].pop() ?? null; flush(run); }
    const [status, err, stopped] = JSON.parse(res) as [number, string, number?];
    // a script that answered without waiting took no simulated time: its caller resumes at the call
    if (!run.yielded) (ctx as any).elapsed = 0;
    const output = run.out + (err ? `\n${err}` : '');
    history.push({ name: ctx.name, args: ctx.args, code: status, output, ...(stopped ? { stopped: true } : {}) });
    if (history.length > 200) history.shift();
    if (status !== 0) {
      const lines = output.trim().split(/\r?\n/);
      throw new Error(`${ctx.name}${ctx.args.length ? `(${ctx.args.join(',')})` : ''} exited with ${status}: ${[...lines].reverse().find((l) => /Error|Exception|stopped/.test(l)) ?? lines[lines.length - 1] ?? ''}`);
    }
  };
  // synchronous mode: scripts run one at a time on the page's thread; cooperative mode: concurrently
  let chain: Promise<unknown> = Promise.resolve();
  const runner = ((ctx: ScriptContext) => {
    const code = pythonSource(ctx);
    if (code == null) return Promise.resolve(false as any);
    return init().then(() => {
      if (coop) return exec1(ctx, code);
      const j = chain.then(() => exec1(ctx, code));
      chain = j.catch(() => {});
      return j;
    });
  }) as PyodideRunner;
  runner.history = history;
  runner.loaded = () => !!pyPromise;
  runner.loading = () => !!pyPromise && !ready;
  runner.preload = () => init().then(() => undefined);
  runner.running = () => [...runs.values()].map((r) => r.name);
  runner.cooperative = () => coop;
  return runner;
}

/**
 * Install the Pyodide runner for a page's Robolink: API RunCode/RunProgram of PROGRAM_PYTHON items and program calls
 * (through the station runtime, when one is attached to rdk.exec). With an app, Pyodide starts loading as soon as a
 * station with Python programs is shown, so the first script call does not wait for the download. Returns the runner.
 */
export function installPyodideRunner(rdk: Robolink, opts: Omit<PyodideRunnerOptions, 'rdk'> & { global?: boolean; preload?: boolean } = {}): PyodideRunner & { uninstall(): void } {
  const runner = createPyodideRunner({ ...opts, rdk }) as PyodideRunner & { uninstall(): void };
  rdk.exec.setScriptRunner(runner);
  (rdk.exec as any).scriptRunnerKind = 'pyodide';
  // programs played by the app's StationRuntime (not through rdk.exec) reach the runner through the global registry
  const off = opts.global === false ? () => {} : registerScriptRunner(runner as any);
  const app = opts.app;
  let offed = false;
  let offStation = () => {};
  if (app?.events?.on && opts.preload !== false) {
    const check = () => {
      if (runner.loaded()) return;
      let has = false;
      try { for (const it of app.station?.walk?.() ?? []) if (it.type === ItemType.PROGRAM_PYTHON) { has = true; break; } } catch { has = false; }
      if (has) setTimeout(() => { if (!offed) runner.preload().catch((e) => app.log?.(`Python (Pyodide) could not be loaded: ${e?.message ?? e}`, 'warn')); }, 200);
    };
    const r = app.events.on('stationChanged', check);
    offStation = typeof r === 'function' ? r : () => app.events.off?.('stationChanged', check);
  }
  runner.uninstall = () => { offed = true; off(); offStation(); if ((rdk.exec as any).scriptRunnerKind === 'pyodide') { rdk.exec.setScriptRunner(null); (rdk.exec as any).scriptRunnerKind = null; } };
  return runner;
}
