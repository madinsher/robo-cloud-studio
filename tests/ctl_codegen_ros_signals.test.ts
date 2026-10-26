/**
 * ROS 2 packages of models whose guards / conditions / decisions read a robot program's output (B_Work ends with
 * SetDO Ready, 1): the simulated skill server applies the station program's outputs and reports them with the service
 * response, so the supervisor node, the BehaviorTree.CPP runner and yasmin see the signal when the program is done,
 * as the studio executors do. Built and run in WSL Ubuntu-22.04 / ROS 2 Jazzy.
 */
import { describe, it, expect, beforeAll } from 'vitest';
import * as fs from 'node:fs';
import * as os from 'node:os';
import * as path from 'node:path';
import { spawnSync } from 'node:child_process';
import { ros2Packages } from '../src/ctl/codegen/ros2';

const WSL = 'C:/Windows/System32/wsl.exe';
const haveRos = (() => { if (process.platform !== 'win32' || !fs.existsSync(WSL)) return false; const r = spawnSync(WSL, ['-d', 'Ubuntu-22.04', '--', 'test', '-f', '/opt/ros/jazzy/setup.bash'], { timeout: 60000 }); return r.status === 0; })();

const DUR = { A_Pick: 2.3, B_Work: 6.6, A_Place: 2.6 };
const ROBOT = { A_Pick: 'R1', A_Place: 'R1', B_Work: 'R2' };
const OUT: Record<string, Array<[string, number]>> = { B_Work: [['Ready', 1]] };
const PETRI = `petri Guarded
place p0 tokens=1
place a1
place b1
place a2
place b2
place done
transition tF
transition tA
transition tB
transition tC guard="Ready == 1"
arc p0 -> tF -> a1, b1
arc a1 -> tA -> a2
arc b1 -> tB -> b2
arc a2 -> tC -> done
final done b2
action tA program=A_Pick robot=R1
action tB program=B_Work robot=R2
action tC program=A_Place robot=R1`;
const BT = `bt Mission
sequence root memory
  parallel 2
    action program name=pickA program=A_Pick robot=R1 timeout=60
    action program name=workB program=B_Work robot=R2 timeout=60
  condition "Ready == 1"
  action program name=placeA program=A_Place robot=R1 timeout=60`;
const FSM = `fsm Pick all boxes
outcomes done aborted
state PickA initial
  do program=A_Pick timeout=30
  fail -> aborted
  ok -> Work
state Work
  do program=B_Work
  ok -> Check
  fail -> aborted
state Check
  when "Ready == 1" -> yes
  otherwise -> no
  yes -> PlaceA
  no -> aborted
state PlaceA
  do program=A_Place
  ok -> done
  fail -> aborted`;

describe('ROS 2 packages: the skill server writes the station programs\' outputs', () => {
  it('model.py carries the outputs, the skill server reports them with the response', () => {
    const files = ros2Packages('petri', PETRI, 'Guarded', { durations: DUR, robots: ROBOT, outputs: OUT });
    const model = files.find((f) => f.name.endsWith('/model.py'))!.content;
    expect(model).toMatch(/OUTPUTS = json\.loads\(r'''\{"A_Pick":|OUTPUTS = json\.loads\(r'''\{"B_Work":\[\["Ready",1\]\]\}'''\)/);
    expect(files.find((f) => f.name.endsWith('/skill_server.py'))!.content).toMatch(/SIGNALS/);
    expect(files.find((f) => f.name.endsWith('/ros_link.py'))!.content).toMatch(/def apply_signals/);
  });
});

describe.skipIf(!haveRos)('ROS 2 packages with signal guards: colcon build and runs in WSL (ROS 2 Jazzy)', () => {
  const sections = new Map<string, string>(); let out = '';
  beforeAll(() => {
    const tmp = fs.realpathSync.native(fs.mkdtempSync(path.join(os.tmpdir(), 'ctl-rossig-')));
    const write = (rel: string, content: string) => { const p = path.join(tmp, rel); fs.mkdirSync(path.dirname(p), { recursive: true }); fs.writeFileSync(p, content.replace(/\r\n/g, '\n')); };
    for (const [kind, src, name] of [['petri', PETRI, 'Guarded'], ['bt', BT, 'Mission'], ['fsm', FSM, 'Pick all boxes']] as const)
      for (const f of ros2Packages(kind, src, name, { durations: DUR, robots: ROBOT, outputs: OUT })) write(`src/${f.name}`, f.content);
    const wsl = (p: string) => `/mnt/${p[0].toLowerCase()}${p.slice(2).replace(/\\/g, '/')}`;
    write('run.sh', `#!/bin/bash
SRC="$1"
source /opt/ros/jazzy/setup.bash
W=$(mktemp -d $HOME/f3ws_test.XXXXXX); cp -r "$SRC/src" $W/
cd $W
echo "=== build"; colcon build --event-handlers console_direct- 2>&1 | grep -E "Summary|Failed|error:|finished" ; echo "BUILD_EXIT=\${PIPESTATUS[0]}"
source install/setup.bash
export ROS_DOMAIN_ID=$((RANDOM % 200 + 20))
export FASTDDS_BUILTIN_TRANSPORTS=SHM
echo "=== petri"; timeout 60 ros2 launch guarded_control guarded_control.launch.py 2>&1 | grep -E "TRACE|RESULT|Traceback|Error" | sed 's/^\\[[a-z_]*-[0-9]*\\] //'
echo "=== bt"; timeout 60 ros2 launch mission_control mission_control.launch.py 2>&1 | grep -E "TRACE|RESULT|Traceback|Error" | sed 's/^\\[[a-z_]*-[0-9]*\\] //'
echo "=== btcpp"; timeout 60 ros2 launch mission_control mission_control.launch.py engine:=btcpp 2>&1 | grep -E "RESULT|what|Error|ERROR" | sed 's/^\\[[a-z_]*-[0-9]*\\] //'
echo "=== fsm"; timeout 60 ros2 launch pick_all_boxes_control pick_all_boxes_control.launch.py 2>&1 | grep -E "TRACE|RESULT|Traceback|Error" | sed 's/^\\[[a-z_]*-[0-9]*\\] //'
echo "=== yasmin"; timeout 60 ros2 launch pick_all_boxes_control pick_all_boxes_control.launch.py engine:=yasmin 2>&1 | grep -E "TRACE|RESULT|Traceback|Error" | sed 's/^\\[[a-z_]*-[0-9]*\\] //'
echo "=== end"
rm -rf $W
`);
    const r = spawnSync(WSL, ['-d', 'Ubuntu-22.04', '--', 'bash', `${wsl(tmp)}/run.sh`, wsl(tmp)], { encoding: 'utf8', timeout: 900000, env: { ...process.env, MSYS_NO_PATHCONV: '1' } });
    out = (r.stdout ?? '') + (r.stderr ?? '');
    fs.writeFileSync(path.join(os.tmpdir(), 'ctl_codegen_ros_signals.log'), out);
    let cur = ''; for (const l of out.split(/\r?\n/)) { const m = /^=== (\w+)/.exec(l); if (m) { cur = m[1]; sections.set(cur, ''); } else if (cur) sections.set(cur, sections.get(cur)! + l + '\n'); }
    fs.rmSync(tmp, { recursive: true, force: true });
  }, 960000);
  const result = (sec: string) => (sections.get(sec) ?? '').split('\n').find((l) => l.startsWith('RESULT'))?.split(/\t| /) ?? [];
  const trace = (sec: string) => (sections.get(sec) ?? '').split('\n').filter((l) => l.startsWith('TRACE')).map((l) => { const [, t, kind, label] = l.split('\t'); return { t: Number(t), kind, label }; });

  it('builds the four packages', () => { expect(sections.get('build'), out).toMatch(/Summary: 4 packages finished/); });
  it('Petri net: the guard Ready == 1 opens when B_Work has ended; done at {done b2} like the studio (≈ 9.6 s)', () => {
    const r = result('petri'); expect(r[1], sections.get('petri')).toBe('done');
    const starts = trace('petri').filter((e) => e.kind === 'start'); const tC = starts.find((e) => e.label === 'tC')!; const tBend = trace('petri').find((e) => e.kind === 'end' && e.label === 'tB')!;
    expect(tC.t).toBeGreaterThanOrEqual(tBend.t - 0.01); expect(Number(r[2])).toBeGreaterThan(8.5); expect(Number(r[2])).toBeLessThan(12);
  });
  it('BT (runtime and BehaviorTree.CPP): the condition after the parallel sees Ready = 1; A_Place runs; success', () => {
    expect(result('bt')[1], sections.get('bt')).toBe('done');
    expect(trace('bt').filter((e) => e.kind === 'program').map((e) => e.label)).toEqual(['A_Pick', 'B_Work', 'A_Place']);
    expect(result('btcpp')[1], sections.get('btcpp')).toBe('SUCCESS');
  });
  it('FSM (runtime and yasmin): Check → yes → PlaceA → outcome done', () => {
    expect(trace('fsm').map((e) => e.label), sections.get('fsm')).toContain('Pick_all_boxes/Check:yes');
    expect(result('fsm')[4] ?? result('fsm').at(-1)).toBe('done');
    expect(result('yasmin').at(-1), sections.get('yasmin')).toBe('done');
  });
});
