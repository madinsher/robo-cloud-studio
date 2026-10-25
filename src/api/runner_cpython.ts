/**
 * CPython runner for station Python programs (contract I5, Node only: studio server, headless tests).
 *
 * A PROGRAM_PYTHON item (params.pythonCode = source text, from the RoboDK import) or a program call such as
 * "SprayOn(1)" runs as `python -u <name>.py 1` with the drop-in (python/robodk + legacy python/robolink) first on
 * PYTHONPATH and STUDIO_URL pointing at the studio API, so `from robolink import *; RDK = Robolink()` talks to the same
 * station. Arguments arrive in sys.argv like in RoboDK. The returned promise resolves when the script exits with 0.
 */
import { spawn, spawnSync, ChildProcess } from 'node:child_process';
import { mkdtempSync, writeFileSync, existsSync, rmSync } from 'node:fs';
import { tmpdir } from 'node:os';
import path from 'node:path';
import { ItemType } from '../core/items/item';
import type { ScriptContext, ScriptRunner } from './exec';

export interface CPythonRunnerOptions {
  /** Studio API URL the scripts connect to (ws://host:port). */
  url: string | (() => string);
  /** Python executable (default: STUDIO_PYTHON, else `py -3` on Windows / python3). */
  python?: string;
  pythonArgs?: string[];
  /** Directory holding the drop-in packages (default: <cwd>/python). */
  dropIn?: string;
  /** Working directory of the scripts (default: a temporary directory per run). */
  cwd?: string | ((ctx: ScriptContext) => string | undefined);
  /** Kill a script after this many seconds (0 = never). */
  timeoutS?: number;
  onOutput?: (name: string, line: string, stream: 'stdout' | 'stderr') => void;
  env?: Record<string, string>;
}

export interface CPythonRunner extends ScriptRunner {
  stopAll(): void;
  running(): number;
  /** Outputs of finished runs (name, exit code, stdout+stderr), most recent last. */
  history: Array<{ name: string; args: string[]; code: number | null; output: string; seconds: number }>;
}

let cachedPython: { cmd: string; args: string[] } | null = null;
/** Locate a usable Python 3 interpreter (STUDIO_PYTHON, py -3, python, python3). */
export function findPython(): { cmd: string; args: string[] } {
  if (cachedPython) return cachedPython;
  const cands: Array<{ cmd: string; args: string[] }> = [];
  if (process.env.STUDIO_PYTHON) cands.push({ cmd: process.env.STUDIO_PYTHON, args: [] });
  if (process.platform === 'win32') cands.push({ cmd: 'py', args: ['-3'] }, { cmd: 'python', args: [] });
  else cands.push({ cmd: 'python3', args: [] }, { cmd: 'python', args: [] });
  for (const c of cands) {
    try {
      const r = spawnSync(c.cmd, [...c.args, '-c', 'import sys; print(sys.version_info[0]); print(sys.executable)'], { encoding: 'utf8', timeout: 20000, windowsHide: true });
      const [major, exe] = r.stdout.trim().split(/\r?\n/);
      // use the interpreter itself (the Windows `py` launcher adds seconds per start)
      if (r.status === 0 && major === '3') return (cachedPython = exe && existsSync(exe) ? { cmd: exe, args: [] } : c);
    } catch { /* try next */ }
  }
  throw new Error('No Python 3 interpreter found (set STUDIO_PYTHON)');
}

/** Source text of a station Python program. */
export function pythonSource(ctx: ScriptContext): string | null {
  const it = ctx.program?.type === ItemType.PROGRAM_PYTHON ? ctx.program : ctx.station.find(ctx.name, ItemType.PROGRAM_PYTHON);
  const p: any = it?.params ?? {};
  const code = p.pythonCode ?? p.source ?? p.code;
  return typeof code === 'string' && code.length ? code : null;
}

export function createCPythonRunner(opts: CPythonRunnerOptions): CPythonRunner {
  const procs = new Set<ChildProcess>();
  const history: CPythonRunner['history'] = [];
  const dropIn = opts.dropIn ?? path.join(process.cwd(), 'python');
  const runner = ((ctx: ScriptContext) => new Promise<void>((resolve, reject) => {
    const code = pythonSource(ctx);
    if (code == null) { reject(new Error(`No Python source for ${ctx.name}`)); return; }
    const py = opts.python ? { cmd: opts.python, args: opts.pythonArgs ?? [] } : findPython();
    const dir = mkdtempSync(path.join(tmpdir(), 'vbs-py-'));
    const file = path.join(dir, `${ctx.name.replace(/[^\w.-]+/g, '_') || 'script'}.py`);
    writeFileSync(file, code, 'utf8');
    const cwd = (typeof opts.cwd === 'function' ? opts.cwd(ctx) : opts.cwd) ?? dir;
    const url = typeof opts.url === 'function' ? opts.url() : opts.url;
    const env: Record<string, string> = { ...(process.env as any), ...opts.env, STUDIO_URL: url, PYTHONIOENCODING: 'utf-8', PYTHONUNBUFFERED: '1', PYTHONPATH: [dropIn, process.env.PYTHONPATH].filter(Boolean).join(path.delimiter) };
    const t0 = Date.now();
    const child = spawn(py.cmd, [...py.args, '-u', file, ...ctx.args], { cwd: existsSync(cwd) ? cwd : dir, env, windowsHide: true });
    procs.add(child);
    let out = '';
    const feed = (stream: 'stdout' | 'stderr') => (d: Buffer) => {
      const s = d.toString('utf8');
      out += s;
      if (out.length > 200000) out = out.slice(-100000);
      if (opts.onOutput) for (const line of s.split(/\r?\n/)) if (line.trim()) opts.onOutput(ctx.name, line, stream);
    };
    child.stdout.on('data', feed('stdout'));
    child.stderr.on('data', feed('stderr'));
    const timer = opts.timeoutS ? setTimeout(() => { out += `\n[killed after ${opts.timeoutS} s]`; child.kill(); }, opts.timeoutS * 1000) : null;
    child.on('error', (e) => { procs.delete(child); if (timer) clearTimeout(timer); reject(e); });
    child.on('close', (code) => {
      procs.delete(child);
      if (timer) clearTimeout(timer);
      history.push({ name: ctx.name, args: ctx.args, code, output: out, seconds: (Date.now() - t0) / 1000 });
      if (history.length > 200) history.shift();
      try { rmSync(dir, { recursive: true, force: true }); } catch { /* ignore */ }
      if (code === 0) resolve();
      else {
        const lines = out.trim().split(/\r?\n/);
        const err = [...lines].reverse().find((l) => /Error|Exception|killed/.test(l)) ?? lines[lines.length - 1] ?? '';
        reject(new Error(`${ctx.name}${ctx.args.length ? `(${ctx.args.join(',')})` : ''} exited with ${code}: ${err}`));
      }
    });
  })) as CPythonRunner;
  runner.stopAll = () => { for (const p of procs) p.kill(); procs.clear(); };
  runner.running = () => procs.size;
  runner.history = history;
  return runner;
}
