/**
 * T10: the browser station-Python runner (src/api/runner_pyodide.ts) with its real bootstrap and the real drop-in,
 * executed by CPython standing in for Pyodide (no Pyodide in Node). The stand-in implements the Pyodide calls the
 * runner uses (FS, registerJsModule, setStdout/Err, globals, runPythonAsync) and forwards the synchronous
 * `studio_host` calls (transport, sleep) over pipes to this process, where they run against the station exactly as in
 * the browser: in-process JSON-RPC, waits and time.sleep() in virtual time.
 */
import { describe, it, expect } from 'vitest';
import { spawn } from 'node:child_process';
import { mkdtempSync, mkdirSync, writeFileSync } from 'node:fs';
import { tmpdir } from 'node:os';
import path from 'node:path';
import { Station, Item, ItemType } from '../src/core/items/item';
import { Robot } from '../src/core/items/robot';
import { Program } from '../src/core/items/program';
import { createRobotFromLibrary } from '../src/core/items/library';
import { Robolink, RobolinkItem } from '../src/api/robolink';
import { createPyodideRunner, installPyodideRunner, inProcessTransport } from '../src/api/runner_pyodide';
import { findPython } from '../src/api/runner_cpython';
import { StationRuntime } from '../src/core/motion/runtime';

let py: { cmd: string; args: string[] } | null = null;
try { py = findPython(); } catch { py = null; }

const DRIVER = String.raw`
import sys, json, types, os
ROOT = sys.argv[1]
_in, _out = sys.stdin, sys.__stdout__
def _call(kind, payload):
    _out.write(json.dumps({'k': kind, 'p': payload}) + '\n'); _out.flush()
    return json.loads(_in.readline())['r']
host = types.ModuleType('studio_host')
host.transport = lambda text, rid=0: _call('transport', [text, rid])
host.sleep = lambda s, rid=0: _call('sleep', [s, rid])
host.out = lambda rid, s, stream: _call('out', [rid, s, stream])
host.job = lambda rid: _call('job', rid)
sys.modules['studio_host'] = host
class _W(object):  # Pyodide's batched stdout delivers whole lines
    def __init__(self, k): self.k, self.buf = k, ''
    def write(self, t):
        self.buf += t
        while chr(10) in self.buf:
            line, self.buf = self.buf.split(chr(10), 1)
            _call('out', [self.k, line])
        return len(t)
    def flush(self): pass
sys.stdout, sys.stderr = _W('stdout'), _W('stderr')
cfg = json.loads(sys.__stdin__.readline())
exec(cfg['boot'].replace("'/studio'", repr(ROOT.replace('\\', '/') + '/studio')), globals())
for k, v in cfg['globals'].items():
    globals()[k] = v
_out.write(json.dumps({'k': 'result', 'p': eval(cfg['src'], globals())}) + '\n'); _out.flush()
`;

/** CPython-backed stand-in for the Pyodide object the runner drives. */
function cpythonPyodide() {
  const root = mkdtempSync(path.join(tmpdir(), 'vbs-pyo-'));
  const modules: Record<string, any> = {};
  const globals = new Map<string, any>();
  let boot = '';
  let stdout: any = null, stderr: any = null;
  writeFileSync(path.join(root, 'driver.py'), DRIVER);
  return {
    FS: { mkdirTree: (p: string) => mkdirSync(path.join(root, p), { recursive: true }), writeFile: (p: string, s: string) => writeFileSync(path.join(root, p), s) },
    registerJsModule: (name: string, obj: any) => { modules[name] = obj; },
    setStdout: (o: any) => { stdout = o; },
    setStderr: (o: any) => { stderr = o; },
    globals: { set: (k: string, v: any) => globals.set(k, v), get: (k: string) => globals.get(k) },
    runPythonAsync: (src: string) => {
      if (!boot) { boot = src; return Promise.resolve(undefined); }
      return new Promise((resolve, reject) => {
        const child = spawn(py!.cmd, [...py!.args, '-u', path.join(root, 'driver.py'), root], { windowsHide: true, env: { ...process.env, PYTHONIOENCODING: 'utf-8' } });
        let buf = '', err = '';
        child.stderr.on('data', (d) => (err += d));
        child.stdout.on('data', (d) => {
          buf += d;
          let i: number;
          while ((i = buf.indexOf('\n')) >= 0) {
            const msg = JSON.parse(buf.slice(0, i)); buf = buf.slice(i + 1);
            if (msg.k === 'result') { resolve(msg.p); child.stdin.end(); continue; }
            let r: any = null;
            const h = modules.studio_host;
            if (msg.k === 'transport') r = h.transport(msg.p[0], msg.p[1]);
            else if (msg.k === 'sleep') r = h.sleep(msg.p[0], msg.p[1]);
            else if (msg.k === 'out') { if (h.out) h.out(msg.p[0], msg.p[1], msg.p[2]); else (msg.p[2] === 'stderr' ? stderr : stdout)?.batched(String(msg.p[1]).replace(/\n$/, '')); }
            else if (msg.k === 'job') r = h.job(msg.p);
            // a promise answer (cooperative mode) blocks this script's process until it settles, like run_sync in the
            // browser, while this event loop (the page) keeps running
            Promise.resolve(r).then((v) => child.stdin.write(JSON.stringify({ r: v ?? null }) + '\n'));
          }
        });
        child.on('close', (c) => { if (c) reject(new Error(`python exited ${c}: ${err}`)); });
        child.stdin.write(JSON.stringify({ boot, src, globals: Object.fromEntries(globals) }) + '\n');
      });
    },
  };
}

function cell() {
  const st = new Station('Pyodide');
  const r = createRobotFromLibrary('UR10e');
  st.addChild(r);
  r.setJoints([0, -90, 90, -90, -90, 0]);
  return { st, r };
}
function script(st: Station, name: string, code: string): Item {
  const it = new Item(ItemType.PROGRAM_PYTHON, name);
  it.params.pythonCode = code;
  st.addChild(it);
  return it;
}

describe('in-process transport (browser bridge for Pyodide scripts)', () => {
  it('answers JSON-RPC synchronously and completes a blocking robot move in virtual time', () => {
    const { st, r } = cell();
    const rdk = new Robolink(st);
    rdk.exec.mode = 'realtime';            // the page's mode: waits must still complete synchronously inside a script
    const t = inProcessTransport(rdk);
    const item = JSON.parse(t(JSON.stringify({ id: 1, method: 'Item', params: ['', 2] }))).result;
    expect(item.$item).toBe(r.id);
    const res = JSON.parse(t(JSON.stringify({ id: 2, method: 'MoveJ', target: r.id, params: [[30, -90, 90, -90, -90, 0], true] })));
    expect(res.error).toBeUndefined();
    expect(r.joints()[0]).toBeCloseTo(30, 6);   // blocking move finished before the call returned
    expect(rdk.exec.busy(r)).toBe(false);
    const bad = JSON.parse(t(JSON.stringify({ id: 3, method: 'Joints', target: null, params: [] })));
    expect(bad.error).toMatch(/^InputError/);
  });
});

describe.skipIf(!py)('Pyodide runner (bootstrap + drop-in run by CPython in place of Pyodide)', { timeout: 120000 }, () => {
  it('runs a station script with sys.argv, legacy imports, quit(), output capture and virtual-time sleep', async () => {
    const { st, r } = cell();
    const rdk = new Robolink(st);
    rdk.exec.mode = 'realtime';
    const lines: string[] = [];
    const runner = createPyodideRunner({ rdk, load: async () => cpythonPyodide(), onOutput: (_n, l) => lines.push(l) });
    script(st, 'Gripper', `import sys
from robolink import *
from robodk import *
RDK = Robolink()
robot = RDK.Item('', ITEM_TYPE_ROBOT)
robot.setSpeed(100, 30)
robot.MoveJ([float(sys.argv[1]), -90, 90, -90, -90, 0], False)
busy = robot.Busy()
t0 = RDK.SimulationTime() if hasattr(RDK, 'SimulationTime') else 0
time.sleep(30)                      # virtual time: the move finishes meanwhile
RDK.setParam('GRIP', '%d|%d|%.1f' % (busy, robot.Busy(), robot.Joints().list()[0]))
print('moved to', robot.Joints().list()[0])
quit()
print('not reached')
`);
    await runner({ station: st, runtime: null as any, name: 'Gripper', args: ['45'], robot: null, program: null });
    expect(String(st.getParam('GRIP'))).toBe('1|0|45.0');
    expect(r.joints()[0]).toBeCloseTo(45, 6);
    expect(lines.some((l) => /moved to 45/.test(l))).toBe(true);
    expect(lines.some((l) => /not reached/.test(l))).toBe(false);
    expect(runner.history[0]).toMatchObject({ name: 'Gripper', args: ['45'], code: 0 });
  });

  it('a failing script rejects with its Python error; unknown calls are not handled (false)', async () => {
    const { st } = cell();
    const rdk = new Robolink(st);
    const runner = createPyodideRunner({ rdk, load: async () => cpythonPyodide() });
    script(st, 'Broken', `from robolink import *\nRDK = Robolink()\nRDK.Item('nope').Joints()\n`);
    await expect(runner({ station: st, runtime: null as any, name: 'Broken', args: [], robot: null, program: null })).rejects.toThrow(/InputError/);
    await expect(runner({ station: st, runtime: null as any, name: 'SetRPM', args: ['70'], robot: null, program: null })).resolves.toBe(false);
  });

  it('program calls such as SprayOn(1) run the station script through the station runtime (I5)', async () => {
    const { st, r } = cell();
    const rdk = new Robolink(st);
    const rt = new StationRuntime(st);
    rdk.exec.attach(rt as any, { mode: 'virtual' });
    const inst = installPyodideRunner(rdk, { load: async () => cpythonPyodide(), global: false });
    script(st, 'SprayOn', `import sys\nfrom robolink import *\nRDK = Robolink()\nRDK.setParam('SPRAY', sys.argv[1] if len(sys.argv) > 1 else 'none')\n`);
    const prog = new Program('Main');
    st.addChild(prog);
    prog.setRobot(r);
    prog.addInstruction({ kind: 'move', moveType: 'MoveJ', targetId: null, joints: [20, -90, 90, -90, -90, 0] } as any);
    prog.addInstruction({ kind: 'code', code: 'SprayOn(1)', asFunctionCall: true } as any);
    prog.addInstruction({ kind: 'move', moveType: 'MoveJ', targetId: null, joints: [40, -90, 90, -90, -90, 0] } as any);
    const h = rt.start(prog, { robot: r as Robot });
    for (let i = 0; i < 400 && (h.status === 'running' || h.status === 'queued'); i++) { rt.tick(0.05); await new Promise((res) => setTimeout(res, 5)); }
    expect(h.status).toBe('done');
    expect(String(st.getParam('SPRAY'))).toBe('1');
    expect(r.joints()[0]).toBeCloseTo(40, 6);
    expect(inst.history.map((x) => x.args)).toEqual([['1']]);
    inst.uninstall();
    expect(new RobolinkItem(rdk, prog).Valid()).toBe(true);
  });
});

const wait = (ms: number) => new Promise((r) => setTimeout(r, ms));
async function until(cond: () => boolean, ms: number): Promise<boolean> { const t0 = Date.now(); while (!cond() && Date.now() - t0 < ms) await wait(20); return cond(); }

// F1 (V1 critical): RoboDK background scripts (`while True: ... pause()`, Library 06.c-06.f) froze the browser tab: the
// runner ran them synchronously on the page thread and one at a time, so the first endless loop never returned.
describe.skipIf(!py)('Pyodide runner: background station scripts (cooperative mode, JSPI semantics)', { timeout: 120000 }, () => {
  const LOOP = `from robolink import *\nfrom robodk import *\nRDK = Robolink()\nn = 0\nprint('loop started')\nwhile True:\n    n += 1\n    RDK.setParam('N', n)\n    pause(0.02)\n`;

  it('a thread running an endless loop runs beside its program, outlives it like in RoboDK and ends on Stop', async () => {
    const { st, r } = cell();
    const rdk = new Robolink(st);
    const rt = new StationRuntime(st);
    rdk.exec.attach(rt as any, { mode: 'virtual' });
    const inst = installPyodideRunner(rdk, { load: async () => cpythonPyodide(), global: false, cooperative: true });
    const loop = script(st, 'Loop', LOOP);
    const prog = new Program('Main');
    st.addChild(prog);
    prog.setRobot(r);
    prog.addInstruction({ kind: 'thread', programId: loop.id, programName: 'Loop' } as any);
    prog.addInstruction({ kind: 'move', moveType: 'MoveJ', targetId: null, joints: [20, -90, 90, -90, -90, 0] } as any);
    prog.addInstruction({ kind: 'pause', timeMs: 1500 } as any);
    prog.addInstruction({ kind: 'move', moveType: 'MoveJ', targetId: null, joints: [40, -90, 90, -90, -90, 0] } as any);
    const h = rt.start(prog, { robot: r as Robot });
    // the page's animation loop: real-time ticks while the script loops concurrently
    const t0 = Date.now();
    while ((h.status === 'running' || h.status === 'queued') && Date.now() - t0 < 60000) { rt.tick(0.05); await wait(50); }
    expect(h.status).toBe('done');
    expect(r.joints()[0]).toBeCloseTo(40, 6);
    const n1 = Number(st.getParam('N'));
    expect(n1).toBeGreaterThan(5);                       // the loop made progress while the program ran
    expect(inst.running()).toEqual(['Loop']);            // and still runs (a thread outlives its program)
    rt.stop();                                           // Stop / Reset ends it at its next call
    expect(await until(() => inst.running().length === 0, 15000)).toBe(true);
    const last = inst.history.at(-1)!;
    expect(last).toMatchObject({ name: 'Loop', code: 0, stopped: true });
    expect(last.output).toContain('loop started');
    const n2 = Number(st.getParam('N'));
    await wait(300);
    expect(Number(st.getParam('N'))).toBe(n2);
    inst.uninstall();
  });

  it('two endless scripts and a blocking script run concurrently; a blocking API move plays in real time', async () => {
    const { st, r } = cell();
    const rdk = new Robolink(st);                         // browser setup: exec not attached, page clock 'virtual'
    const runner = createPyodideRunner({ rdk, load: async () => cpythonPyodide(), cooperative: true });
    script(st, 'LoopA', LOOP.replace(/'N'/g, "'A'"));
    script(st, 'LoopB', LOOP.replace(/'N'/g, "'B'"));
    script(st, 'Mover', `from robolink import *\nRDK = Robolink()\nrobot = RDK.Item('', ITEM_TYPE_ROBOT)\nrobot.setSpeed(-1, 60)\nrobot.MoveJ([60, -90, 90, -90, -90, 0])\nRDK.setParam('MOVED', '%.1f' % robot.Joints().list()[0])\n`);
    const abortA = { aborted: false }, abortB = { aborted: false };
    const a = runner({ station: st, runtime: null as any, name: 'LoopA', args: [], robot: null, program: null, abort: abortA } as any).catch((e) => e);
    const b = runner({ station: st, runtime: null as any, name: 'LoopB', args: [], robot: null, program: null, abort: abortB } as any).catch((e) => e);
    const seen: number[] = [];
    const iv = setInterval(() => seen.push(r.joints()[0]), 10);
    const t0 = Date.now();
    await runner({ station: st, runtime: null as any, name: 'Mover', args: [], robot: null, program: null });
    const took = Date.now() - t0;
    clearInterval(iv);
    expect(String(st.getParam('MOVED'))).toBe('60.0');
    // 60 deg at 60 deg/s (plus acceleration): about a second of real time, with intermediate joint values on the way
    expect(took).toBeGreaterThan(700);
    expect(seen.some((q) => q > 5 && q < 55)).toBe(true);
    expect(Number(st.getParam('A'))).toBeGreaterThan(3);
    expect(Number(st.getParam('B'))).toBeGreaterThan(3);
    abortA.aborted = abortB.aborted = true;
    await Promise.all([a, b]);
    expect(runner.running()).toEqual([]);
    expect(runner.history.filter((x) => x.stopped).map((x) => x.name).sort()).toEqual(['LoopA', 'LoopB']);
  });

  it('synchronous mode (no JSPI): an endless script is stopped after the page-thread budget instead of freezing', async () => {
    const { st } = cell();
    const rdk = new Robolink(st);
    const runner = createPyodideRunner({ rdk, load: async () => cpythonPyodide(), cooperative: false, syncBudgetS: 1 });
    script(st, 'Loop', LOOP);
    const t0 = Date.now();
    await expect(runner({ station: st, runtime: null as any, name: 'Loop', args: [], robot: null, program: null })).rejects.toThrow(/stopped after 1 s.*JSPI/);
    expect(Date.now() - t0).toBeLessThan(20000);
    expect(Number(st.getParam('N'))).toBeGreaterThan(1);
  });
});
