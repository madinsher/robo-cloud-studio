/**
 * ROS 2 packages generated from control models, built and run in WSL Ubuntu-22.04 / ROS 2 Jazzy:
 *  - colcon build of every package (ament_python supervisor packages, ament_cmake BehaviorTree.CPP package);
 *  - end-to-end runs over DDS (supervisor + simulated skill server; BT.CPP runner; yasmin node) whose results match the
 *    studio executors with the same program durations;
 *  - the yasmin machine against a robolink stand-in with a virtual clock: same outcome trace as the FSM executor.
 * The BT.CPP XML is checked in TS (well-formed, node mapping) without WSL.
 */
import { describe, it, expect, beforeAll } from 'vitest';
import * as fs from 'node:fs';
import * as os from 'node:os';
import * as path from 'node:path';
import { spawnSync } from 'node:child_process';
import { Station } from '../src/core/items/item';
import { Program } from '../src/core/items/program';
import { createRobotFromLibrary } from '../src/core/items/library';
import { parsePetri, parseDes, TEMPLATES } from '../src/ctl/dsl';
import { PetriExecutor, AutomatonExecutor } from '../src/ctl/exec';
import { FsmExecutor, parseFsm } from '../src/ctl/fsm';
import { simulatedBackend } from '../src/ctl/host';
import { makeStationWorld } from '../src/ctl/world';
import { ros2Packages, btCppXml, yasminModel } from '../src/ctl/codegen/ros2';
import { parseXml, xmlAll } from '../src/ctl/codegen/common';

const WSL = 'C:/Windows/System32/wsl.exe';
const haveRos = (() => { if (process.platform !== 'win32' || !fs.existsSync(WSL)) return false; const r = spawnSync(WSL, ['-d', 'Ubuntu-22.04', '--', 'test', '-f', '/opt/ros/jazzy/setup.bash'], { timeout: 60000 }); return r.status === 0; })();

const NET = `petri Cell fork
place p1 tokens=1 kind=idle
place a1
place p2 tokens=1 kind=idle
place b1
place d1
place d2
transition t1a
transition t2a
transition t1b
transition t2b delay=0.5
arc p1 -> t1a -> a1
arc a1 -> t2a -> d1
arc p2 -> t1b -> b1
arc b1 -> t2b -> d2
final d1 d2
action t1a program="Weld left"
action t1b program="Weld right"`;
const BT = `bt Weld tree
sequence root memory
  parallel 2
    action program program="Weld left" timeout=30
    retry 1
      action program program="Weld right"
  condition "!estop && count < 5"
  action set count="=count + 1"
  action wait seconds=0.5
  timeout 5
    action goto zone=table
  action report`;
const DES = `des Two cells
automaton L
  initial I
  marked D
  I -startL-> W
  W -doneL-> D
automaton R
  initial I
  marked D
  I -startR-> W
  W -doneR-> D
uncontrollable doneL doneR
action L.W program="Weld left" done=doneL
action R.W program="Weld right" done=doneR`;
const DUR: Record<string, number> = { 'Weld left': 2, 'Weld right': 3, Home: 1, Pick: 1.5, Place: 1, goto_table: 1 };
const ROBOT: Record<string, string> = { 'Weld left': 'A', 'Weld right': 'B', Home: 'R', Pick: 'R', Place: 'R' };

/** The studio executors on stand-in programs with the same durations. */
function tsWorld() {
  const st = new Station('s'); const robots = new Map<string, ReturnType<typeof createRobotFromLibrary>>();
  for (const [name, rn] of Object.entries(ROBOT)) { let r = robots.get(rn); if (!r) { r = st.addChild(createRobotFromLibrary('UR5e', rn)); robots.set(rn, r); } st.addChild(new Program(name)).setRobot(r); }
  return makeStationWorld({ station: st, programs: simulatedBackend(DUR) });
}

describe('BehaviorTree.CPP v4 XML', () => {
  it('maps the studio node semantics: reactive vs memory sequences, retry attempts, parallel thresholds, scripts', () => {
    const { xml, skills, vars } = btCppXml(BT);
    const root = parseXml(xml); expect(root.name).toBe('root'); expect(root.attrs.BTCPP_format).toBe('4');
    expect(xmlAll(root, 'Sequence').length).toBe(1); expect(xmlAll(root, 'ReactiveSequence').length).toBe(0); // `memory` sequence
    expect(xmlAll(root, 'Parallel')[0].attrs).toMatchObject({ success_count: '2', failure_count: '1' });
    expect(xmlAll(root, 'RetryUntilSuccessful')[0].attrs.num_attempts).toBe('2'); // retry 1 = one more attempt
    expect(xmlAll(root, 'Timeout').map((t) => t.attrs.msec)).toEqual(['30000', '5000']);
    expect(xmlAll(root, 'ScriptCondition')[0].attrs.code).toBe('! estop && count < 5');
    expect(xmlAll(root, 'Program').map((p) => p.attrs.program)).toEqual(['Weld left', 'Weld right']);
    expect([...skills.keys()].sort()).toEqual(['Goto', 'Program']);
    expect(vars.get('count')).toBe('double'); expect(vars.get('estop')).toBe('bool');
    // a reactive (py_trees default) sequence → ReactiveSequence
    expect(btCppXml('bt r\nsequence root\n  action goto zone=a\n  action wait seconds=1').xml).toMatch(/<ReactiveSequence name="root">[\s\S]*<Sleep msec="1000"\/>/);
  });
  it('the yasmin data maps every outcome of every state (unmapped timeouts fall back to the fail branch)', () => {
    const y = yasminModel(TEMPLATES.fsm) as { states: Array<{ name: string; map: Record<string, string>; outcomes: string[]; machine?: { states: Array<{ name: string; map: Record<string, string>; outcomes: string[] }> } }>; outcomes: string[] };
    for (const s of [...y.states, ...y.states.flatMap((s) => s.machine?.states ?? [])]) for (const o of s.outcomes) expect(s.map[o]).toBeDefined();
    const pick = y.states.find((s) => s.name === 'Cycle')!.machine!.states.find((s) => s.name === 'Pick')!;
    expect(pick.map).toMatchObject({ ok: 'Place', fail: 'failed', timeout: 'failed', canceled: 'canceled' });
    expect(y.outcomes).toEqual(expect.arrayContaining(['done', 'aborted', 'canceled']));
  });
});

describe.skipIf(!haveRos)('ROS 2 packages: colcon build and runs in WSL (ROS 2 Jazzy)', () => {
  let out = ''; const sections = new Map<string, string>(); let tmp = '';
  beforeAll(() => {
    tmp = fs.realpathSync.native(fs.mkdtempSync(path.join(os.tmpdir(), 'ctl-ros-')));
    const write = (rel: string, content: string) => { const p = path.join(tmp, rel); fs.mkdirSync(path.dirname(p), { recursive: true }); fs.writeFileSync(p, content.replace(/\r\n/g, '\n')); };
    for (const [kind, src, name] of [['petri', NET, 'Cell fork'], ['fsm', TEMPLATES.fsm, 'Pick all boxes'], ['bt', BT, 'Weld tree'], ['des', DES, 'Two cells']] as const)
      for (const f of ros2Packages(kind, src, name, { durations: DUR, robots: ROBOT })) write(`src/${f.name}`, f.content);
    write('stub/robodk/__init__.py', '');
    write('stub/robodk/robolink.py', `import time
ITEM_TYPE_ROBOT, ITEM_TYPE_PROGRAM = 2, 8
DUR = ${JSON.stringify(DUR)}
ROB = ${JSON.stringify(ROBOT)}
class Item(object):
    def __init__(self, rdk, name, typ):
        self.rdk, self.name, self.typ = rdk, name, typ
    def Valid(self):
        return self.name in DUR or self.typ == ITEM_TYPE_ROBOT
    def Name(self):
        return self.name
    def getLink(self, typ):
        return Item(self.rdk, ROB.get(self.name, 'prog:' + self.name), ITEM_TYPE_ROBOT)
    def RunProgram(self):
        self.rdk.started[self.name] = time.time()
    def Busy(self):
        s = self.rdk.started.get(self.name)
        return s is not None and time.time() < s + DUR[self.name] - 1e-9
    def Stop(self):
        self.rdk.started.pop(self.name, None)
class Robolink(object):
    def __init__(self, *a, **k):
        self.started, self.params = {}, {}
    def Item(self, name, typ=-1):
        return Item(self, name, typ)
    def setParam(self, n, v):
        self.params[n] = v
    def getParam(self, n):
        return self.params.get(n)
`);
    write('yasmin_stub.py', `import sys, time
clock = [1000.0]
time.time = lambda: clock[0]
def _sleep(s):
    clock[0] += s
time.sleep = _sleep
sys.path.insert(0, sys.argv[1])
from robodk.robolink import Robolink
from pick_all_boxes_control.yasmin_fsm import build_fsm
from pick_all_boxes_control.model import MODEL
from yasmin import Blackboard
sm, ctx = build_fsm(Robolink(), MODEL)
print('RESULT\\t%s\\t%.2f' % (sm(Blackboard()), ctx.now()), flush=True)
`);
    const wsl = (p: string) => `/mnt/${p[0].toLowerCase()}${p.slice(2).replace(/\\/g, '/')}`;
    write('run.sh', `#!/bin/bash
SRC="$1"
source /opt/ros/jazzy/setup.bash
W=$(mktemp -d $HOME/t7ws_test.XXXXXX); cp -r "$SRC/src" $W/  # unique: the suite may run concurrently
cd $W
echo "=== build"; colcon build --event-handlers console_direct- 2>&1 | grep -E "Summary|Failed|error:|finished" ; echo "BUILD_EXIT=\${PIPESTATUS[0]}"
source install/setup.bash
echo "=== pycompile"; n=0; bad=0; for f in $(find src -name "*.py"); do python3 -m py_compile "$f" && n=$((n+1)) || bad=$((bad+1)); done; echo "PYCOMPILE ok=$n bad=$bad"
export ROS_DOMAIN_ID=$((RANDOM % 200 + 20))
# this WSL's UDP loopback drops rclcpp ↔ rclpy service traffic (rclpy ↔ rclpy works); shared memory is reliable
export FASTDDS_BUILTIN_TRANSPORTS=SHM
echo "=== petri"; timeout 60 ros2 launch cell_fork_control cell_fork_control.launch.py 2>&1 | grep -E "TRACE|RESULT|Traceback|Error" | sed 's/^\\[[a-z_]*-[0-9]*\\] //'
echo "=== des"; timeout 60 ros2 launch two_cells_control two_cells_control.launch.py 2>&1 | grep -E "TRACE|RESULT|Traceback|Error|not found|failed|WARN" | sed 's/^\\[[a-z_]*-[0-9]*\\] //'
echo "=== btcpp"; timeout 60 ros2 launch weld_tree_control weld_tree_control.launch.py engine:=btcpp 2>&1 | grep -E "RESULT|what|Error|ERROR|WARN" | sed 's/^\\[[a-z_]*-[0-9]*\\] //'
echo "=== yasmin_ros"; timeout 60 ros2 launch pick_all_boxes_control pick_all_boxes_control.launch.py engine:=yasmin 2>&1 | grep -E "TRACE|RESULT|Traceback|Error|not found|failed|WARN|warn" | sed 's/^\\[[a-z_]*-[0-9]*\\] //'
echo "=== yasmin_stub"; python3 "$SRC/yasmin_stub.py" "$SRC/stub" 2>&1 | grep -E "TRACE|RESULT|Error|Traceback|not found"
echo "=== end"
rm -rf $W
`);
    const r = spawnSync(WSL, ['-d', 'Ubuntu-22.04', '--', 'bash', `${wsl(tmp)}/run.sh`, wsl(tmp)], { encoding: 'utf8', timeout: 900000, env: { ...process.env, MSYS_NO_PATHCONV: '1' } });
    out = (r.stdout ?? '') + (r.stderr ?? '');
    fs.writeFileSync(path.join(os.tmpdir(), 'ctl_codegen_ros.log'), out); // full output of the WSL run (diagnostics)
    let cur = ''; for (const l of out.split(/\r?\n/)) { const m = /^=== (\w+)/.exec(l); if (m) { cur = m[1]; sections.set(cur, ''); } else if (cur) sections.set(cur, sections.get(cur)! + l + '\n'); }
    fs.rmSync(tmp, { recursive: true, force: true });
  }, 960000);
  const trace = (sec: string) => (sections.get(sec) ?? '').split('\n').filter((l) => l.startsWith('TRACE')).map((l) => { const [, t, kind, label] = l.split('\t'); return { t: Number(t), kind, label }; });
  const result = (sec: string) => (sections.get(sec) ?? '').split('\n').find((l) => l.startsWith('RESULT'))?.split('\t') ?? [];

  it('colcon builds all five packages (ament_python and the BehaviorTree.CPP ament_cmake package); every .py compiles', () => {
    expect(sections.get('build'), out).toMatch(/Summary: 5 packages finished/); expect(sections.get('build')).not.toMatch(/Failed/);
    expect(sections.get('pycompile')).toMatch(/PYCOMPILE ok=\d+ bad=0/); expect(Number(/ok=(\d+)/.exec(sections.get('pycompile') ?? '')?.[1])).toBeGreaterThanOrEqual(25);
  });
  it('Petri net over ROS (supervisor + skill server): same firing sequence and times as the studio executor', () => {
    const x = new PetriExecutor({ doc: parsePetri(NET), world: tsWorld() }); x.run(30, 0.05);
    const ros = trace('petri').filter((e) => e.kind === 'start' || e.kind === 'end'); const ts = x.trace.map((e) => ({ t: e.t, kind: e.phase, label: e.transition }));
    expect(ros.map((e) => `${e.kind}:${e.label}`), sections.get('petri')).toEqual(ts.map((e) => `${e.kind}:${e.label}`));
    ros.forEach((e, i) => expect(Math.abs(e.t - ts[i].t)).toBeLessThan(0.35)); // DDS service latency
    expect(result('petri')[1]).toBe('done'); expect(x.verdict).toBe('done');
    // both programs started together: robots run concurrently
    const progs = trace('petri').filter((e) => e.kind === 'program'); expect(progs.map((e) => e.label).sort()).toEqual(['Weld left', 'Weld right']); expect(Math.abs(progs[0].t - progs[1].t)).toBeLessThan(0.11);
  });
  it('automaton over ROS: same event sequence and verdict (both cells end in their marked state → done)', () => {
    const x = new AutomatonExecutor({ doc: parseDes(DES), world: tsWorld() }); x.run(30, 0.05);
    expect(trace('des').filter((e) => e.kind === 'event').map((e) => e.label), sections.get('des')).toEqual(x.trace.map((e) => e.event));
    expect(result('des')[1]).toBe(x.verdict); expect(x.verdict).toBe('done');
  });
  it('BehaviorTree.CPP runner with the skill stubs over ROS: SUCCESS after the parallel welds, the wait and the goto', () => {
    const r = result('btcpp'); expect(r[1], sections.get('btcpp')).toBe('SUCCESS');
    // parallel(2 s, 3 s) + 0.5 s + goto 1 s ≈ 4.5 s (+ tick and service latency)
    expect(Number(r[2])).toBeGreaterThan(4.4); expect(Number(r[2])).toBeLessThan(7);
  });
  it('yasmin FSM: over ROS it ends with outcome done; on a robolink stand-in (virtual clock) its outcome trace equals the FSM executor', () => {
    expect(result('yasmin_ros')[3], sections.get('yasmin_ros')).toBe('done');
    const x = new FsmExecutor({ doc: parseFsm(TEMPLATES.fsm), world: tsWorld() }); x.run(60, 0.05);
    const y = trace('yasmin_stub').filter((e) => e.kind === 'outcome');
    expect(y.map((e) => e.label), sections.get('yasmin_stub')).toEqual(x.trace.map((e) => `${e.path}:${e.outcome}`));
    y.forEach((e, i) => expect(Math.abs(e.t - x.trace[i].t)).toBeLessThan(0.21));
    expect(result('yasmin_stub')[1]).toBe(x.outcome); expect(x.outcome).toBe('done');
  });
});
