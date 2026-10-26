/**
 * CPython runner for station Python programs (contract I5, Node only: studio server, headless tests).
 *
 * A PROGRAM_PYTHON item (params.pythonCode = source text, from the RoboDK import) or a program call such as
 * "SprayOn(1)" runs with the drop-in (python/robodk + legacy python/robolink) first on PYTHONPATH and STUDIO_URL
 * pointing at the studio API, so `from robolink import *; RDK = Robolink()` talks to the same station. Arguments
 * arrive in sys.argv like in RoboDK. The returned promise resolves when the script exits with 0.
 *
 * mode 'worker' (default): a pool of persistent interpreters (python/robodk/_script_worker.py) runs one script each
 * at a time in a fresh __main__ (about 1 s saved per call: programs that call a script on every instruction stay
 * usable); concurrent scripts get separate workers. mode 'process': one `python -u <name>.py args` per run.
 * A call naming no station Python program resolves `false` (not handled: RoboDK ignores unknown calls in simulation).
 */
import { spawn, spawnSync, ChildProcess } from 'node:child_process';
import { mkdtempSync, writeFileSync, existsSync, rmSync } from 'node:fs';
import { tmpdir } from 'node:os';
import path from 'node:path';
import { pythonSource } from './exec';
import type { ScriptContext, ScriptRunner } from './exec';

export { pythonSource };

export interface CPythonRunnerOptions {
  /** Studio API URL the scripts connect to (ws://host:port). */
  url: string | (() => string);
  /** Python executable (default: STUDIO_PYTHON, else `py -3` on Windows / python3). */
  python?: string;
  pythonArgs?: string[];
  /** Directory holding the drop-in packages (default: <cwd>/python). */
  dropIn?: string;
  /** Working directory of the scripts (default: a temporary directory per runner). */
  cwd?: string | ((ctx: ScriptContext) => string | undefined);
  /** Kill a script after this many seconds (0 = never). */
  timeoutS?: number;
  onOutput?: (name: string, line: string, stream: 'stdout' | 'stderr') => void;
  env?: Record<string, string>;
  mode?: 'worker' | 'process';
  /** Idle workers kept alive (default 4). */
  maxIdle?: number;
  /** Unattended runs: tkinter is replaced by python/robodk/_headless_gui (mainloop presses the main button). Default: STUDIO_HEADLESS_GUI=1. */
  headlessGui?: boolean;
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

interface Worker { proc: ChildProcess; buf: string; ready: Promise<void>; job: Job | null; dead: boolean }
interface Job { id: number; ctx: ScriptContext; out: string; t0: number; resolve: (v: any) => void; reject: (e: Error) => void; timer: any }

const fileName = (name: string) => `${name.replace(/[^\w.-]+/g, '_') || 'script'}.py`;

export function createCPythonRunner(opts: CPythonRunnerOptions): CPythonRunner {
  const procs = new Set<ChildProcess>();
  const history: CPythonRunner['history'] = [];
  const dropIn = opts.dropIn ?? path.join(process.cwd(), 'python');
  const mode = opts.mode ?? (process.env.STUDIO_PY_RUNNER === 'process' ? 'process' : 'worker');
  const maxIdle = opts.maxIdle ?? 4;
  const headlessGui = opts.headlessGui ?? process.env.STUDIO_HEADLESS_GUI === '1';
  let workDir: string | null = null;
  const dir = () => (workDir ??= mkdtempSync(path.join(tmpdir(), 'vbs-py-')));
  const env = (): Record<string, string> => ({ ...(process.env as any), ...(headlessGui ? { MPLBACKEND: 'Agg' } : {}), ...opts.env, STUDIO_URL: url(), PYTHONIOENCODING: 'utf-8', PYTHONUNBUFFERED: '1', PYTHONPATH: [headlessGui ? path.join(dropIn, 'robodk', '_headless_gui') : '', dropIn, process.env.PYTHONPATH].filter(Boolean).join(path.delimiter) });
  const url = () => (typeof opts.url === 'function' ? opts.url() : opts.url);
  const py = () => (opts.python ? { cmd: opts.python, args: opts.pythonArgs ?? [] } : findPython());
  const cwdOf = (ctx: ScriptContext) => { const c = typeof opts.cwd === 'function' ? opts.cwd(ctx) : opts.cwd; return c && existsSync(c) ? c : dir(); };
  const emit = (ctx: ScriptContext, s: string, stream: 'stdout' | 'stderr') => { if (opts.onOutput) for (const line of s.split(/\r?\n/)) if (line.trim()) opts.onOutput(ctx.name, line, stream); };
  const record = (ctx: ScriptContext, code: number | null, out: string, t0: number) => {
    history.push({ name: ctx.name, args: ctx.args, code, output: out, seconds: (Date.now() - t0) / 1000 });
    if (history.length > 200) history.shift();
  };
  const failure = (ctx: ScriptContext, code: number | null, out: string) => {
    const lines = out.trim().split(/\r?\n/);
    const err = [...lines].reverse().find((l) => /Error|Exception|killed/.test(l)) ?? lines[lines.length - 1] ?? '';
    return new Error(`${ctx.name}${ctx.args.length ? `(${ctx.args.join(',')})` : ''} exited with ${code}: ${err}`);
  };

  // ---- process mode ------------------------------------------------------------------------------------------
  const runProcess = (ctx: ScriptContext, code: string) => new Promise<void>((resolve, reject) => {
    const p = py();
    const file = path.join(dir(), fileName(ctx.name));
    writeFileSync(file, code, 'utf8');
    const t0 = Date.now();
    const child = spawn(p.cmd, [...p.args, '-u', file, ...ctx.args], { cwd: cwdOf(ctx), env: env(), windowsHide: true });
    procs.add(child);
    let out = '';
    const feed = (stream: 'stdout' | 'stderr') => (d: Buffer) => { const s = d.toString('utf8'); out += s; if (out.length > 200000) out = out.slice(-100000); emit(ctx, s, stream); };
    child.stdout.on('data', feed('stdout'));
    child.stderr.on('data', feed('stderr'));
    const timer = opts.timeoutS ? setTimeout(() => { out += `\n[killed after ${opts.timeoutS} s]`; child.kill(); }, opts.timeoutS * 1000) : null;
    child.on('error', (e) => { procs.delete(child); if (timer) clearTimeout(timer); reject(e); });
    child.on('close', (c) => {
      procs.delete(child);
      if (timer) clearTimeout(timer);
      record(ctx, c, out, t0);
      if (c === 0) resolve(); else reject(failure(ctx, c, out));
    });
  });

  // ---- worker mode -------------------------------------------------------------------------------------------
  const idle: Worker[] = [];
  const busy = new Set<Worker>();
  let seq = 0;
  const spawnWorker = (): Worker => {
    const p = py();
    const proc = spawn(p.cmd, [...p.args, '-u', path.join(dropIn, 'robodk', '_script_worker.py')], { cwd: dir(), env: env(), windowsHide: true });
    procs.add(proc);
    let readyResolve!: () => void;
    const w: Worker = { proc, buf: '', ready: new Promise<void>((r) => (readyResolve = r)), job: null, dead: false };
    proc.stdout!.on('data', (d: Buffer) => {
      w.buf += d.toString('utf8');
      let i: number;
      while ((i = w.buf.indexOf('\n')) >= 0) {
        const line = w.buf.slice(0, i); w.buf = w.buf.slice(i + 1);
        let msg: any = null;
        try { msg = line.startsWith('{') ? JSON.parse(line) : null; } catch { msg = null; }
        const job = w.job;
        if (!msg || !msg.t) { if (job && line.trim()) { job.out += line + '\n'; emit(job.ctx, line, 'stdout'); } continue; }
        if (msg.t === 'ready') { readyResolve(); continue; }
        if (!job || msg.id !== job.id) continue;
        if (msg.t === 'out') { job.out += msg.s; if (job.out.length > 200000) job.out = job.out.slice(-100000); emit(job.ctx, msg.s, msg.stream === 'stderr' ? 'stderr' : 'stdout'); }
        else if (msg.t === 'done') {
          w.job = null;
          if (job.timer) clearTimeout(job.timer);
          record(job.ctx, msg.code, job.out, job.t0);
          release(w);
          if (msg.code === 0) job.resolve(undefined); else job.reject(failure(job.ctx, msg.code, job.out || msg.error || ''));
        }
      }
    });
    proc.stderr!.on('data', (d: Buffer) => { const s = d.toString('utf8'); if (w.job) { w.job.out += s; emit(w.job.ctx, s, 'stderr'); } });
    const die = (why: string) => {
      if (w.dead) return;
      w.dead = true;
      procs.delete(proc);
      busy.delete(w);
      const k = idle.indexOf(w); if (k >= 0) idle.splice(k, 1);
      readyResolve();
      const job = w.job; w.job = null;
      if (job) { if (job.timer) clearTimeout(job.timer); record(job.ctx, null, job.out + `\n${why}`, job.t0); job.reject(failure(job.ctx, null, job.out + `\n${why}`)); }
    };
    proc.on('exit', (c) => die(`[worker exited with ${c}]`));
    proc.on('error', (e) => die(`[worker error: ${e.message}]`));
    return w;
  };
  const release = (w: Worker) => {
    busy.delete(w);
    if (w.dead) return;
    if (idle.length >= maxIdle) { w.dead = true; procs.delete(w.proc); w.proc.kill(); return; }
    idle.push(w);
  };
  const runWorker = (ctx: ScriptContext, code: string) => new Promise<void>((resolve, reject) => {
    const w = idle.pop() ?? spawnWorker();
    busy.add(w);
    const job: Job = { id: ++seq, ctx, out: '', t0: Date.now(), resolve, reject, timer: null };
    w.job = job;
    if (opts.timeoutS) job.timer = setTimeout(() => { job.out += `\n[killed after ${opts.timeoutS} s]`; w.proc.kill(); }, opts.timeoutS * 1000);
    const file = path.join(cwdOf(ctx), fileName(ctx.name));
    w.ready.then(() => {
      if (w.dead || w.job !== job) return;
      w.proc.stdin!.write(JSON.stringify({ id: job.id, file, code, args: ctx.args, url: url(), cwd: cwdOf(ctx) }) + '\n');
    });
  });

  const runner = ((ctx: ScriptContext) => {
    const code = pythonSource(ctx);
    if (code == null) return Promise.resolve(false as any);
    return mode === 'process' ? runProcess(ctx, code) : runWorker(ctx, code);
  }) as CPythonRunner;
  runner.stopAll = () => {
    for (const p of procs) p.kill();
    procs.clear();
    idle.length = 0;
    busy.clear();
    if (workDir) { try { rmSync(workDir, { recursive: true, force: true }); } catch { /* ignore */ } workDir = null; }
  };
  runner.running = () => (mode === 'process' ? procs.size : busy.size);
  runner.history = history;
  return runner;
}
