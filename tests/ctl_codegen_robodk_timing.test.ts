/**
 * RoboDK station scripts against a robolink stand-in that behaves like RoboDK 5.9 on the points that broke real runs:
 *  - Item.Busy() on a running program costs a round trip (~0.12 s wall, measured 107-175 ms in RoboDK);
 *  - the simulation runs at a speed (RoboDK starts at 5) with SimulationTime() / SimulationSpeed() / setSimulationSpeed();
 *  - a program's outputs (SetDO Ready, 1 at its end) are station parameters once it has finished.
 * The studio runs the same models: the BT's condition sees the signal its parallel branch just set, and a timer that
 * ends before a program wins the race for a shared token.
 */
import { describe, it, expect } from 'vitest';
import * as fs from 'node:fs';
import * as os from 'node:os';
import * as path from 'node:path';
import { spawnSync } from 'node:child_process';
import { robodkStationScript } from '../src/ctl/codegen/robodk';

const PY37 = 'C:/RoboDK/Python37/python.exe';
const HAVE_PY = fs.existsSync(PY37);

/** RoboDK-like stand-in: virtual wall clock (driver), simulation clock at a speed, slow Busy(), program outputs. */
const STUB = `import json, os, time
ITEM_TYPE_STATION = 1
ITEM_TYPE_ROBOT = 2
ITEM_TYPE_TARGET = 6
ITEM_TYPE_PROGRAM = 8
CFG = json.load(open(os.environ['VBS_STUB']))
LAT = CFG.get('latency', 0.12)


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
        self.rdk.started[self.name] = self.rdk.sim()
        self.rdk.log.append(('run', self.name, round(self.rdk.sim(), 3)))

    def Busy(self):
        s = self.rdk.started.get(self.name)
        if s is None:
            return False
        time.sleep(LAT)  # one robolink round trip while a program runs
        return self.rdk.sim() < s + CFG['programs'][self.name]['duration'] - 1e-9

    def Stop(self):
        self.rdk.started.pop(self.name, None)


class Robolink(object):
    def __init__(self, *a, **k):
        self.started, self.params, self.log = {}, {}, []
        self.speed, self.w0, self.s0 = CFG.get('speed', 5.0), time.time(), 0.0

    def sim(self):
        return self.s0 + (time.time() - self.w0) * self.speed

    def SimulationTime(self):
        return round(self.sim(), 3)

    def SimulationSpeed(self):
        return self.speed

    def setSimulationSpeed(self, s):
        self.s0, self.w0, self.speed = self.sim(), time.time(), s
        self.log.append(('speed', s))

    def Item(self, name, typ=-1):
        return Item(self, name, typ)

    def setParam(self, n, v):
        self.params[n] = str(v)

    def _params(self):
        out = dict(self.params)
        for p, c in CFG['programs'].items():
            s = self.started.get(p)
            if s is not None and self.sim() >= s + c['duration'] - 1e-9:
                for k, v in c.get('sets', {}).items():
                    out[k] = str(v)
        return out

    def getParam(self, n):
        v = self._params().get(n)
        if v is None:
            print('WARNING: Invalid/unknown parameter to get: %s' % n)  # what RoboDK 5.9's robolink prints
        return v

    def getParams(self):
        return [[k, v] for k, v in self._params().items()]
`;
const DRIVER = `import sys, runpy, time, atexit
sys.path.insert(0, sys.argv[2])
clock = [1000.0]
time.time = lambda: clock[0]
def _sleep(s):
    clock[0] += max(0.0, s)
time.sleep = _sleep
script = sys.argv[1]
sys.argv = [script] + sys.argv[3:]
from robodk import robolink
_init = robolink.Robolink.__init__
LINKS = []
def _keep(self, *a, **k):
    _init(self, *a, **k)
    LINKS.append(self)
robolink.Robolink.__init__ = _keep
try:
    runpy.run_path(script, run_name='__main__')
finally:
    for l in LINKS:
        print('LOG\\t' + repr(l.log))
`;

function runScript(kind: 'petri' | 'bt', source: string, cfg: Record<string, unknown>, args: string[]) {
  const dir = fs.mkdtempSync(path.join(os.tmpdir(), 'ctl-rdkt-'));
  fs.mkdirSync(path.join(dir, 'stub', 'robodk'), { recursive: true });
  fs.writeFileSync(path.join(dir, 'stub', 'robodk', '__init__.py'), '');
  fs.writeFileSync(path.join(dir, 'stub', 'robodk', 'robolink.py'), STUB);
  const programs = cfg.programs as Record<string, { robot: string }>;
  fs.writeFileSync(path.join(dir, 'cfg.json'), JSON.stringify({ ...cfg, robots: [...new Set(Object.values(programs).map((p) => p.robot))] }));
  const f = robodkStationScript(kind, source, 'model');
  const script = path.join(dir, f.name); fs.writeFileSync(script, f.content);
  fs.writeFileSync(path.join(dir, 'driver.py'), DRIVER);
  const r = spawnSync(PY37, ['-I', path.join(dir, 'driver.py'), script, path.join(dir, 'stub'), ...args], { encoding: 'utf8', timeout: 120000, env: { ...process.env, VBS_STUB: path.join(dir, 'cfg.json') } });
  fs.rmSync(dir, { recursive: true, force: true });
  const lines = r.stdout.split(/\r?\n/);
  return {
    stdout: lines,
    stderr: r.stderr,
    trace: lines.filter((l) => l.startsWith('TRACE')).map((l) => { const [, t, k, label] = l.split('\t'); return { t: Number(t), kind: k, label }; }),
    result: lines.find((l) => l.startsWith('RESULT'))?.split('\t') ?? [],
    log: lines.find((l) => l.startsWith('LOG')) ?? '',
  };
}

// the verifier's station: R1 = UR10e (A_Pick, A_Place), R2 = ABB IRB 1600 (B_Work ends with SetDO Ready, 1)
const PROGRAMS = { A_Pick: { robot: 'R1', duration: 2.3 }, A_Place: { robot: 'R1', duration: 2.6 }, B_Work: { robot: 'R2', duration: 6.6, sets: { Ready: 1 } } };

const BT60 = `bt Mission
sequence root memory
  parallel 2
    action program name=pickA seconds=1 program=A_Pick robot=R1 timeout=60
    action program name=workB seconds=1 program=B_Work robot=R2 timeout=60
  condition "Ready == 1"
  action program name=placeA seconds=1 program=A_Place robot=R1 timeout=60`;

const RACE = `petri Race between a robot program and a timer
place p0 tokens=1
place a1
place d1
place tok tokens=1
place a2
place d2
place xA
place xB
transition tF
transition tA
transition tD delay=2
transition tX
transition tY
arc p0 -> tF -> a1, d1
arc a1 -> tA -> a2
arc d1 -> tD -> d2
arc a2, tok -> tX -> xA
arc d2, tok -> tY -> xB
final xA d2
final xB a2
action tA program=A_Pick robot=R1
action tX program=A_Place robot=R1
action tY program=B_Work robot=R2`;

describe.skipIf(!HAVE_PY)('RoboDK station script timing and signals (RoboDK-like stand-in)', () => {
  it('BT: the condition after a parallel reads the signal the finished program set in the same step (studio: success)', () => {
    for (const speed of ['1', '5']) {
      const py = runScript('bt', BT60, { programs: PROGRAMS, speed: 5, latency: 0.12 }, ['120', speed]);
      expect(py.stderr).toBe('');
      expect(py.result[1], `speed ${speed}: ${py.result.join(' ')}`).toBe('done');
      expect(py.trace.filter((e) => e.kind === 'program').map((e) => e.label)).toEqual(['A_Pick', 'B_Work', 'A_Place']);
      expect(py.trace.find((e) => e.kind === 'tree')?.label).toBe('success');
    }
  });
  it('Petri race: the 2 s timer beats the 2.3 s program as in the studio, at the default speed and at RoboDK\'s speed 5', () => {
    for (const args of [['60'], ['60', '0']]) {
      const py = runScript('petri', RACE, { programs: PROGRAMS, speed: 5, latency: 0.12 }, args);
      expect(py.stderr).toBe('');
      const starts = py.trace.filter((e) => e.kind === 'start').map((e) => e.label);
      expect(starts, args.join(' ')).toContain('tY');
      expect(starts).not.toContain('tX');
      expect(py.trace.filter((e) => e.kind === 'program').map((e) => e.label)).toEqual(['A_Pick', 'B_Work']);
      expect(py.result[1]).toBe('done');
      // delays run on the simulation clock: tD ends at 2 s of simulation time, not after 40 slow steps
      const tD = py.trace.find((e) => e.kind === 'end' && e.label === 'tD')!;
      expect(tD.t).toBeGreaterThanOrEqual(2 - 1e-6); expect(tD.t).toBeLessThan(2.8);
    }
  });
  it('stdout: no "Invalid/unknown parameter" warning per tick for a signal no program has set yet (one getParams() per read)', () => {
    const GUARD = ['petri Guard on a robot output', 'place p0 tokens=1', 'place a1', 'place b1', 'place a2', 'place b2', 'place a3', 'place pE',
      'transition tF', 'transition tA', 'transition tB', 'transition tC guard="Ready == 1"', 'transition tJ',
      'arc p0 -> tF -> a1, b1', 'arc a1 -> tA -> a2', 'arc b1 -> tB -> b2', 'arc a2 -> tC -> a3', 'arc a3, b2 -> tJ -> pE',
      'action tA program=A_Pick robot=R1', 'action tB program=B_Work robot=R2', 'action tC program=A_Place robot=R1'].join('\n');
    for (const [kind, src] of [['petri', GUARD], ['bt', BT60]] as Array<['petri' | 'bt', string]>) {
      const py = runScript(kind, src, { programs: PROGRAMS, speed: 5, latency: 0.12 }, ['60']);
      expect(py.result[1], `${kind}: ${py.stdout.join('\n')}`).toBe('done');
      expect(py.trace.filter((e) => e.kind === 'program').map((e) => e.label)).toEqual(['A_Pick', 'B_Work', 'A_Place']);
      expect(py.stdout.filter((l) => /WARNING/.test(l)), kind).toEqual([]);
    }
  });
  it('sets the simulation speed to 1 by default (as the studio runs) and restores RoboDK\'s speed; 0 keeps it', () => {
    const a = runScript('petri', RACE, { programs: PROGRAMS, speed: 5, latency: 0.12 }, ['60']);
    expect(a.log).toContain("('speed', 1.0)"); expect(a.log).toContain("('speed', 5.0)");
    const b = runScript('petri', RACE, { programs: PROGRAMS, speed: 5, latency: 0.12 }, ['60', '0']);
    expect(b.log).not.toContain("'speed'");
  });
});
