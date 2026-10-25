/**
 * Behavior-tree semantics (B9–B12, B21, B25) and supervisors under partial observation (B7, B8, B20).
 */
import { describe, it, expect } from 'vitest';
import * as fs from 'node:fs';
import * as os from 'node:os';
import * as path from 'node:path';
import { spawnSync } from 'node:child_process';
import { demos } from '../src/demos';
import { ItemType } from '../src/core/items/item';
import type { Robot } from '../src/core/items/robot';
import { parseBt, parseDes } from '../src/ctl/dsl';
import { btToDsl } from '../src/ctl/graphdoc';
import { ControlRuntime, simulatedHost, simulatedBindings } from '../src/ctl/runtime';
import { AutomatonExecutor } from '../src/ctl/exec';
import { SupervisorRuntime, supervisorPython } from '../src/ctl/des';
import { synthesizeSupervisor } from '../src/ctl/supervisor';
import { analyse } from '../src/ctl/analysis';
import { YOUBOT_DES_DSL, YOUBOT_BT_DSL } from '../src/ctl/examples_dsl';
import { YOUBOT_STATION_DES, YOUBOT_STATION_BT } from '../src/ctl/course_stations';
import { perRobotBackend } from '../src/ctl/host';
import { makeStationWorld } from '../src/ctl/world';

const PY37 = 'C:/RoboDK/Python37/python.exe';

describe('BT editor round trip and actions (B9, B11)', () => {
  it('program / signal arguments survive btToDsl → parseBt, and the program runs', () => {
    const doc = parseBt('bt w\nsequence root memory\n  action program program="Weld left"\n  action signal signal=lamp value=true');
    const legacy = parseBt('bt w\nsequence root memory\n  action program name="Weld left"'); // older editor output
    expect(legacy.root.children![0].args).toMatchObject({ program: 'Weld left' });
    const again = parseBt(btToDsl(doc));
    expect(again.root.children![0].args).toMatchObject({ program: 'Weld left' }); expect(again.root.children![1].args).toMatchObject({ signal: 'lamp', value: true });
    const host = simulatedHost({ programs: { 'Weld left': 1 } });
    const rt = new ControlRuntime({ bt: again, bindings: host }); rt.run(3, 0.1, () => rt.status === 'success');
    expect(rt.status).toBe('success'); expect(host.runs).toEqual(['Weld left']); expect(host.signals.lamp).toBe(true);
  });
  it('an unknown action name is an analysis error and fails at run time', () => {
    const src = 'bt typo\nsequence root memory\n  action gotoo zone=table\n  action report';
    const r = analyse('bt', src); expect(r.ok).toBe(false); expect(r.markdown).toMatch(/unknown action 'gotoo'/);
    const w = simulatedBindings(); const rt = new ControlRuntime({ bt: parseBt(src), bindings: w }); rt.run(1);
    expect(rt.status).toBe('failure'); expect(rt.bb.reported).toBeUndefined(); expect(rt.tree.unknown.has('gotoo')).toBe(true);
    expect(w.state.x).toBe(0);
  });
});

describe('BT parallel and halting (B10, B12)', () => {
  it('parallel 2 {wait 1; wait 2} succeeds at 2 s (finished children are not restarted)', () => {
    const rt = new ControlRuntime({ bt: parseBt('bt p\nparallel 2\n  action wait seconds=1\n  action wait seconds=2'), bindings: simulatedHost() });
    let t = 0; while (rt.status !== 'success' && t < 20) { rt.tick(0.1); t = rt.time; }
    expect(rt.status).toBe('success'); expect(rt.time).toBeGreaterThan(1.8); expect(rt.time).toBeLessThan(2.25);
  });
  it('the timeout of one branch halts only its own program: Long completes once', () => {
    const host = simulatedHost({ programs: { Long: 5, Short: 3 } });
    const rt = new ControlRuntime({ bt: parseBt('bt h\nparallel 1\n  action program program=Long\n  timeout 2\n    action program program=Short'), bindings: host });
    rt.run(10, 0.1, () => rt.status !== 'running');
    expect(rt.status).toBe('success'); expect(host.runs).toEqual(['Long', 'Short']);
    const longEnd = host.log.find((e) => e.program === 'Long' && e.event === 'end')!; expect(longEnd.t).toBeGreaterThan(4.85); expect(longEnd.t).toBeLessThan(5.25);
    expect(host.log.filter((e) => e.program === 'Long' && e.event === 'start').length).toBe(1);
  });
  it('analysis warns about a reactive sequence that re-executes finished actions (BT.CPP ReactiveSequence)', () => {
    const r = analyse('bt', 'bt r\nsequence root\n  action goto zone=table timeout=60\n  action wait seconds=1 timeout=2\n  action report timeout=1');
    expect(r.markdown).toMatch(/reactive sequence/);
  });
});

describe('BT on an arm-only station (B21)', () => {
  it('a tree of program actions drives both welding robots, the second branch waits for nothing but its own robot', () => {
    const st = demos.find((d) => d.id === 'welding')!.build(); const [r1, r2] = st.itemsOfType<Robot>(ItemType.ROBOT);
    const world = makeStationWorld({ station: st, programs: perRobotBackend(st) });
    const q1 = r1.joints(), q2 = r2.joints(); let both = 0;
    const rt = new ControlRuntime({ bt: parseBt('bt weld\nparallel 2\n  action program program="Weld left"\n  action program program="Weld right"'), bindings: { ...world.host, step: (dt) => world.step!(dt) } });
    for (let i = 0; i < 2000 && rt.status === 'running'; i++) { const a = r1.joints(), b = r2.joints(); rt.tick(0.05); if (a.some((v, k) => Math.abs(v - r1.joints()[k]) > 1e-6) && b.some((v, k) => Math.abs(v - r2.joints()[k]) > 1e-6)) both++; }
    expect(rt.status).toBe('success'); expect(both * 0.05).toBeGreaterThan(20); void q1; void q2;
  });
});

describe('Supervisors under partial observation (B7, B8)', () => {
  const sup = synthesizeSupervisor(YOUBOT_DES_DSL)!;
  it('the deployed table is observer-based, the analysis reports what runs', () => {
    expect(sup.table.observer).toBe(true); expect(sup.table.unobservable).toEqual(['g_slip']);
    expect(sup.table.states.every((s) => s.next.g_slip === undefined)).toBe(true);
    const r = analyse('des', YOUBOT_DES_DSL); const sec = r.sections.find((s) => s.title.startsWith('Partial observation'))!;
    expect(sec.level === 'error').toBe(!sup.implementable); expect(r.metrics['implementable under partial observation']).toBe(sup.implementable);
  });
  it('after the path to g_ok and an unobserved slip, g_close and o_giveup are accepted (TS runtime)', () => {
    const go = (rt: SupervisorRuntime) => { for (const e of ['b_go_table', 'b_arrive', 'a_reach', 'a_reached', 'v_detect', 'g_close', 'g_ok']) rt.observe(e); };
    const a = new SupervisorRuntime(sup.table); go(a); expect(a.observe('g_slip')).toBe(false); a.observe('g_close');
    const b = new SupervisorRuntime(sup.table); go(b); b.observe('o_giveup');
  });
  it.skipIf(!fs.existsSync(PY37))('the exported Python supervisor accepts the same events (Python 3.7)', () => {
    const dir = fs.mkdtempSync(path.join(os.tmpdir(), 'ctl-sup-'));
    fs.writeFileSync(path.join(dir, 'youbot_supervisor.py'), supervisorPython(sup.table, 'sup'));
    fs.writeFileSync(path.join(dir, 'check.py'), `import sys\nsys.path.insert(0, ${JSON.stringify(dir)})\nfrom youbot_supervisor import Supervisor\nfor tail in (['g_close'], ['o_giveup']):\n    s = Supervisor()\n    for e in ['b_go_table','b_arrive','a_reach','a_reached','v_detect','g_close','g_ok']: s.observe(e)\n    assert s.observe('g_slip') is False\n    for e in tail: s.observe(e)\nprint('OK')\n`);
    const r = spawnSync(PY37, ['-I', path.join(dir, 'check.py')], { encoding: 'utf8', timeout: 60000 });
    expect(r.stderr).toBe(''); expect(r.stdout.trim()).toBe('OK');
    fs.rmSync(dir, { recursive: true, force: true });
  });
  it('an unobservable event from the world never throws out of the executors', () => {
    const host = simulatedHost({ programs: { Pick: 1 } });
    const pending: string[] = [];
    const x = new AutomatonExecutor({ world: { host, forRobot: () => null, events: () => pending.splice(0) }, doc: parseDes(`des slip
automaton G
  initial O
  marked O
  O -close-> C
  C -ok-> D
  D -slip-> O
  D -put-> O
spec S
  initial a
  marked a
  a -close-> a
  a -put-> a
uncontrollable ok slip
unobservable slip
guard put "false"
action G.C program=Pick done=ok`), supervisor: synthesizeSupervisor(`des slip
automaton G
  initial O
  marked O
  O -close-> C
  C -ok-> D
  D -slip-> O
  D -put-> O
spec S
  initial a
  marked a
  a -close-> a
  a -put-> a
uncontrollable ok slip
unobservable slip`)!.table });
    x.run(1.5, 0.1); pending.push('slip');
    expect(() => x.run(3, 0.1)).not.toThrow();
    expect(x.trace.some((t) => t.event === 'slip')).toBe(true);
    // the BT mission: a slip after the grasp is ignored by the observer-based supervisor (no model mismatch, no exception)
    const w = simulatedBindings({ targets: 1 }, undefined, { speed: 5 });
    const bt = parseBt(YOUBOT_BT_DSL.replace('action stow event=a_stow done=a_stowed timeout=10\n        action goto zone=bin', 'action drop\n        action stow event=a_stow done=a_stowed timeout=10\n        action goto zone=bin'));
    const rt = new ControlRuntime({ bt, supervisor: synthesizeSupervisor(YOUBOT_DES_DSL)!.table, bindings: w });
    expect(() => rt.run(60)).not.toThrow();
  });
});

describe('Course-A mission over several objects (B20)', () => {
  it('targets = 2 under the cyclic plant: two deliveries, 0 supervisor mismatches, mission reported', () => {
    const sup = synthesizeSupervisor(YOUBOT_STATION_DES)!; expect(sup).not.toBeNull();
    const w = simulatedBindings({ targets: 2 }, undefined, { speed: 4 });
    const rt = new ControlRuntime({ bt: parseBt(YOUBOT_STATION_BT), supervisor: sup.table, bindings: w });
    rt.run(200, 0.1, (bb) => bb.reported === true);
    expect(rt.bb.reported).toBe(true); expect(rt.supervisorErrors).toBe(0); expect(rt.denied).toBe(0); expect(w.state.targets).toBe(0);
    expect(w.state.zone).toBe('home');
  });
});
