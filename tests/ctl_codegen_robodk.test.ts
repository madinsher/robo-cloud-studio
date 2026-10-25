/**
 * RoboDK station scripts generated from control models: they compile with RoboDK's Python 3.7 and, run against a
 * robolink stand-in with a virtual clock, reproduce the studio executor's firing / outcome sequence and timing.
 */
import { describe, it, expect } from 'vitest';
import * as fs from 'node:fs';
import * as os from 'node:os';
import * as path from 'node:path';
import { spawnSync } from 'node:child_process';
import { Station, ItemType } from '../src/core/items/item';
import { Program } from '../src/core/items/program';
import { createRobotFromLibrary } from '../src/core/items/library';
import { parsePetri, parseDes, TEMPLATES, parseBt } from '../src/ctl/dsl';
import { PetriExecutor, AutomatonExecutor } from '../src/ctl/exec';
import { FsmExecutor, parseFsm } from '../src/ctl/fsm';
import { ControlRuntime } from '../src/ctl/runtime';
import { simulatedBackend } from '../src/ctl/host';
import { makeStationWorld } from '../src/ctl/world';
import { robodkStationScript } from '../src/ctl/codegen/robodk';

const PY37 = 'C:/RoboDK/Python37/python.exe';
const HAVE_PY = fs.existsSync(PY37);

const STUB = `import json, os, time
ITEM_TYPE_STATION = 1
ITEM_TYPE_ROBOT = 2
ITEM_TYPE_TARGET = 6
ITEM_TYPE_PROGRAM = 8
CFG = json.load(open(os.environ['VBS_STUB']))


class Item(object):
    def __init__(self, rdk, name, typ):
        self.rdk, self.name, self.typ = rdk, name, typ

    def Valid(self):
        return (self.typ == ITEM_TYPE_PROGRAM and self.name in CFG['programs']) or (self.typ == ITEM_TYPE_ROBOT and self.name in CFG['robots'])

    def Name(self):
        return self.name

    def getLink(self, typ):
        return Item(self.rdk, CFG['programs'][self.name]['robot'], ITEM_TYPE_ROBOT)

    def RunProgram(self):
        self.rdk.started[self.name] = time.time()

    def Busy(self):
        s = self.rdk.started.get(self.name)
        return s is not None and time.time() < s + CFG['programs'][self.name]['duration'] - 1e-9

    def Stop(self):
        self.rdk.started.pop(self.name, None)


class Robolink(object):
    def __init__(self, *a, **k):
        self.started, self.params = {}, {}

    def Item(self, name, typ=-1):
        return Item(self, name, typ)

    def setParam(self, n, v):
        self.params[n] = str(v)

    def getParam(self, n):
        return self.params.get(n)
`;
const DRIVER = `import sys, runpy, time
sys.path.insert(0, sys.argv[2])
clock = [1000.0]
time.time = lambda: clock[0]
def _sleep(s):
    clock[0] += s
time.sleep = _sleep
script = sys.argv[1]
sys.argv = [script, sys.argv[3]]
runpy.run_path(script, run_name='__main__')
`;

/** Generate, py_compile and run the script against the stand-in; returns the TRACE lines and the RESULT line. */
function runScript(kind: 'des' | 'petri' | 'fsm' | 'bt', source: string, programs: Record<string, { robot: string; duration: number }>, seconds = 120) {
  const dir = fs.mkdtempSync(path.join(os.tmpdir(), 'ctl-rdk-'));
  fs.mkdirSync(path.join(dir, 'stub', 'robodk'), { recursive: true });
  fs.writeFileSync(path.join(dir, 'stub', 'robodk', '__init__.py'), '');
  fs.writeFileSync(path.join(dir, 'stub', 'robodk', 'robolink.py'), STUB);
  fs.writeFileSync(path.join(dir, 'cfg.json'), JSON.stringify({ programs, robots: [...new Set(Object.values(programs).map((p) => p.robot))] }));
  const f = robodkStationScript(kind, source, 'model');
  const script = path.join(dir, f.name); fs.writeFileSync(script, f.content);
  fs.writeFileSync(path.join(dir, 'driver.py'), DRIVER);
  const comp = spawnSync(PY37, ['-I', '-m', 'py_compile', script], { encoding: 'utf8', timeout: 60000 });
  const r = spawnSync(PY37, ['-I', path.join(dir, 'driver.py'), script, path.join(dir, 'stub'), String(seconds)], { encoding: 'utf8', timeout: 120000, env: { ...process.env, VBS_STUB: path.join(dir, 'cfg.json') } });
  fs.rmSync(dir, { recursive: true, force: true });
  const lines = r.stdout.split(/\r?\n/);
  return { compile: comp.status, stderr: r.stderr, trace: lines.filter((l) => l.startsWith('TRACE')).map((l) => { const [, t, kind2, label] = l.split('\t'); return { t: Number(t), kind: kind2, label }; }), result: lines.find((l) => l.startsWith('RESULT'))?.split('\t') ?? [] };
}

function tsWorld(programs: Record<string, { robot: string; duration: number }>) {
  const st = new Station('s'); const robots = new Map<string, ReturnType<typeof createRobotFromLibrary>>();
  for (const [name, p] of Object.entries(programs)) { let r = robots.get(p.robot); if (!r) { r = st.addChild(createRobotFromLibrary('UR5e', p.robot)); robots.set(p.robot, r); } const pr = st.addChild(new Program(name)); pr.setRobot(r); }
  const be = simulatedBackend(Object.fromEntries(Object.entries(programs).map(([n, p]) => [n, p.duration])));
  void ItemType;
  return { world: makeStationWorld({ station: st, programs: be }), be };
}

const CELL = { 'A1 Pick from buffer': { robot: 'M1', duration: 3 }, 'A2 Load machine': { robot: 'M1', duration: 4 }, 'A3 Retract': { robot: 'M1', duration: 1 }, 'B2 Unload AMR': { robot: 'M1', duration: 3.5 }, 'C1 Unload machine': { robot: 'M2', duration: 6 }, 'B1 Enter': { robot: 'AMR', duration: 2 }, 'B3 Leave': { robot: 'AMR', duration: 2 } };
const CELL_NET = `petri cell with monitor
place p1 tokens=1 kind=idle
place a1
place a2
place p2 tokens=1 kind=idle
place b1
place b2
place rM tokens=1 kind=resource
place rZ tokens=1 kind=resource
place V tokens=1 kind=monitor
place p3 tokens=1 kind=idle
transition t1a
transition t2a
transition t3a
transition t1b
transition t2b
transition t3b
transition t1c delay=2
arc p1, rM, V -> t1a -> a1
arc a1, rZ -> t2a -> a2, V
arc a2 -> t3a -> rM, rZ, p1
arc p2, rZ, V -> t1b -> b1
arc b1, rM -> t2b -> b2, V
arc b2 -> t3b -> rZ, rM, p2
arc p3 -> t1c -> p3
action t1a program="A1 Pick from buffer"
action t2a program="A2 Load machine"
action t3a program="A3 Retract"
action t1b program="B1 Enter"
action t2b program="B2 Unload AMR"
action t3b program="B3 Leave"
action t1c program="C1 Unload machine"`;

describe.skipIf(!HAVE_PY)('RoboDK station script = studio executor (Python 3.7, virtual clock)', () => {
  it('Petri cell with its GMEC monitor: same start / end sequence and times for 60 s', () => {
    const py = runScript('petri', CELL_NET, CELL, 60);
    expect(py.compile).toBe(0); expect(py.stderr).toBe('');
    const { world } = tsWorld(CELL); const x = new PetriExecutor({ doc: parsePetri(CELL_NET), world }); x.run(60, 0.05);
    const ts = x.trace.map((e) => ({ t: e.t, kind: e.phase, label: e.transition }));
    const pyT = py.trace.filter((e) => e.kind === 'start' || e.kind === 'end');
    expect(ts.length).toBeGreaterThan(30);
    const n = Math.min(pyT.length, ts.length); expect(n).toBeGreaterThanOrEqual(Math.max(pyT.length, ts.length) - 4); // the last tick may differ
    expect(pyT.slice(0, n).map((e) => `${e.kind}:${e.label}`)).toEqual(ts.slice(0, n).map((e) => `${e.kind}:${e.label}`));
    pyT.slice(0, n).forEach((e, i) => expect(Math.abs(e.t - ts[i].t)).toBeLessThan(0.11));
    // the programs RoboDK would run concurrently: M2 overlaps M1
    const progs = py.trace.filter((e) => e.kind === 'program'); expect(progs.find((e) => e.label === 'C1 Unload machine')!.t).toBeLessThan(0.1); expect(progs.find((e) => e.label === 'A1 Pick from buffer')!.t).toBeLessThan(0.1);
  });
  it('automaton with guards, a counter and fail outcomes: same event sequence', () => {
    const src = `des depal
automaton D
  initial Plan
  marked Done
  Plan -found-> Pick
  Plan -none_left-> Done
  Pick -picked-> Count
  Count -counted-> Plan
uncontrollable picked counted
blackboard remaining=3
guard found "remaining > 0"
guard none_left "remaining == 0"
action D.Pick program=PickBox done=picked
action D.Count set=remaining value="=remaining - 1" done=counted`;
    const progs = { PickBox: { robot: 'R', duration: 1.5 } };
    const py = runScript('des', src, progs, 30);
    expect(py.compile).toBe(0); expect(py.stderr).toBe('');
    const { world } = tsWorld(progs); const x = new AutomatonExecutor({ doc: parseDes(src), world }); x.run(30);
    expect(py.trace.filter((e) => e.kind === 'event').map((e) => e.label)).toEqual(x.trace.map((e) => e.event));
    expect(py.result[1]).toBe('done'); expect(x.verdict).toBe('done');
  });
  it('FSM template: same outcome sequence and end outcome', () => {
    const progs = { Home: { robot: 'R', duration: 1 }, Pick: { robot: 'R', duration: 2 }, Place: { robot: 'R', duration: 1.5 } };
    const py = runScript('fsm', TEMPLATES.fsm, progs, 60);
    expect(py.compile).toBe(0); expect(py.stderr).toBe('');
    const { world } = tsWorld(progs); const x = new FsmExecutor({ doc: parseFsm(TEMPLATES.fsm), world }); x.run(60);
    expect(py.trace.filter((e) => e.kind === 'outcome').map((e) => e.label)).toEqual(x.trace.map((e) => `${e.path}:${e.outcome}`));
    expect(py.result[4]).toBe('done');
    const pyEnd = py.trace.filter((e) => e.kind === 'outcome').at(-1)!.t; expect(Math.abs(pyEnd - x.trace.at(-1)!.t)).toBeLessThan(0.11);
  });
  it('behavior tree with a parallel and a timeout: same program starts', () => {
    const src = 'bt cell\nsequence root memory\n  parallel 2\n    action program program=Weld1\n    action program program=Weld2\n  timeout 3\n    action program program=Slow\n  action program program=Weld1';
    const progs = { Weld1: { robot: 'R1', duration: 2 }, Weld2: { robot: 'R2', duration: 4 }, Slow: { robot: 'R1', duration: 10 } };
    const py = runScript('bt', src, progs, 30);
    expect(py.compile).toBe(0); expect(py.stderr).toBe('');
    const { world, be } = tsWorld(progs); const rt = new ControlRuntime({ bt: parseBt(src), bindings: { ...world.host, step: (dt) => world.step!(dt) } }); rt.run(30, 0.05, () => rt.status !== 'running');
    expect(py.trace.filter((e) => e.kind === 'program').map((e) => e.label)).toEqual(be.log.filter((e) => e.event === 'start').map((e) => e.program));
    expect(rt.status).toBe('failure'); expect(py.result[1]).toBe('failed');
  });
});
