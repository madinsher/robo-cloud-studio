/**
 * Executable hierarchical FSM kind (yasmin-style): outcomes, nested machines, decision states over the blackboard,
 * entry / exit actions, timeouts, cancel, structural analysis, DSL round trip, run on a real station.
 */
import { describe, it, expect } from 'vitest';
import { demos } from '../src/demos';
import { ItemType } from '../src/core/items/item';
import type { Robot } from '../src/core/items/robot';
import { parseFsm, fsmToDsl, FsmExecutor, checkFsm } from '../src/ctl/fsm';
import { TEMPLATES, detectKind } from '../src/ctl/dsl';
import { analyse } from '../src/ctl/analysis';
import { simulatedHost } from '../src/ctl/runtime';
import { perRobotBackend } from '../src/ctl/host';
import { makeStationWorld } from '../src/ctl/world';
import type { ExecutorWorld } from '../src/ctl/exec';

const sim = (programs: Record<string, number>, fail: string[] = []) => { const host = simulatedHost({ programs, fail }); return { host, world: { host, forRobot: () => null, events: () => host.events!() } as ExecutorWorld }; };

describe('FSM document', () => {
  it('parses nested machines, decision states and outcome maps; writes back identically', () => {
    expect(detectKind(TEMPLATES.fsm)).toBe('fsm');
    const doc = parseFsm(TEMPLATES.fsm);
    expect(doc.root.outcomes).toEqual(['done', 'aborted']); expect(doc.blackboard).toEqual({ remaining: 3, picked: 0 });
    const cycle = doc.root.states.find((s) => s.name === 'Cycle')!; expect(cycle.kind).toBe('machine'); expect(cycle.machine!.outcomes).toEqual(['empty', 'failed']);
    const plan = cycle.machine!.states.find((s) => s.name === 'Plan')!; expect(plan.kind).toBe('decision'); expect(plan.when).toEqual([{ expr: 'remaining > 0', outcome: 'next' }]); expect(plan.otherwise).toBe('empty');
    expect(checkFsm(doc).filter((i) => i.level === 'error')).toEqual([]);
    const again = parseFsm(fsmToDsl(doc)); expect(again).toEqual(doc);
    const r = analyse('fsm', TEMPLATES.fsm); expect(r.ok).toBe(true); expect(r.metrics.machines).toBe(2);
  });
  it('reports unmapped outcomes and unknown targets as errors', () => {
    const r = analyse('fsm', 'fsm bad\noutcomes done\nstate A\n  do program=X\n  ok -> B\nstate B\n  ok -> nowhere');
    expect(r.ok).toBe(false); expect(r.markdown).toMatch(/outcome 'fail' is not mapped/); expect(r.markdown).toMatch(/no state or outcome nowhere/);
  });
});

describe('FSM executor', () => {
  it('runs the template: 3 pick/place cycles decided by the blackboard, then park → done', () => {
    const { host, world } = sim({ Home: 1, Pick: 2, Place: 1.5 });
    const x = new FsmExecutor({ doc: parseFsm(TEMPLATES.fsm), world }); x.run(60);
    expect(x.outcome).toBe('done'); expect(x.verdict).toBe('done');
    expect(host.runs).toEqual(['Home', 'Pick', 'Place', 'Pick', 'Place', 'Pick', 'Place', 'Home']);
    expect(x.bb.remaining).toBe(0); expect(x.bb.picked).toBe(3);
    // timing: Home 1 + 3 × (2 + 1.5) + Home 1 = 12.5 s (instantaneous decisions take no time)
    expect(x.time).toBeGreaterThan(12.4); expect(x.time).toBeLessThan(12.9);
    const outs = x.trace.map((t) => `${t.path.split('/').slice(1).join('/')}:${t.outcome}`);
    expect(outs.slice(0, 4)).toEqual(['Home:ok', 'Cycle/Plan:next', 'Cycle/Pick:ok', 'Cycle/Place:ok']);
    expect(outs).toContain('Cycle/Plan:empty'); expect(outs).toContain('Cycle:empty');
  });
  it('a failing skill takes the fail branch out of the nested machine (outcome propagation)', () => {
    const { host, world } = sim({ Home: 1, Pick: 2, Place: 1 }, ['Pick']);
    const x = new FsmExecutor({ doc: parseFsm(TEMPLATES.fsm), world }); x.run(30);
    expect(x.outcome).toBe('aborted'); expect(host.runs).toEqual(['Home']);
    expect(x.trace.map((t) => t.outcome)).toEqual(['ok', 'next', 'fail', 'failed']);
  });
  it('state timeout → timeout outcome (falls back to the fail map), machine timeout pre-empts the children, cancel halts the skill', () => {
    const { world } = sim({ Slow: 10, Fast: 1 });
    const x = new FsmExecutor({ world, doc: parseFsm('fsm t\noutcomes done late\nstate A\n  do program=Slow\n  timeout 2\n  ok -> done\n  fail -> late') });
    x.run(5); expect(x.outcome).toBe('late'); expect(x.time).toBeLessThan(2.3);
    const y = new FsmExecutor({ world, doc: parseFsm('fsm m\noutcomes done late\nmachine M\n  outcomes ok\n  timeout 3\n  state A\n    do program=Fast\n    ok -> B\n  state B\n    do program=Slow\n    ok -> ok\n  ok -> done\n  timeout -> late') });
    y.run(10); expect(y.outcome).toBe('late'); expect(y.time).toBeGreaterThan(2.9); expect(y.time).toBeLessThan(3.3);
    const { host: h2, world: w2 } = sim({ Slow: 10 });
    const z = new FsmExecutor({ world: w2, doc: parseFsm('fsm c\noutcomes done stopped\nstate A\n  do program=Slow\n  ok -> done\n  fail -> done\n  canceled -> stopped') });
    z.run(1); z.cancel(); z.run(1); expect(z.outcome).toBe('stopped'); expect(h2.playing).toBeNull();
  });
  it('entry / exit actions set the blackboard and signals; a decision waits for a sensor value', () => {
    const { host, world } = sim({});
    const x = new FsmExecutor({ world, doc: parseFsm('fsm io\noutcomes done\nstate Wait decision\n  entry signal=lamp value=1\n  exit signal=lamp value=0\n  when "part_present" -> go\n  go -> End\nstate End\n  entry set=finished value=true\n  ok -> done') });
    x.run(1); expect(x.where()).toMatch(/Wait$/); expect(host.signals.lamp).toBe(1);
    host.signals.part_present = true; x.run(0.5);
    expect(x.outcome).toBe('done'); expect(host.signals.lamp).toBe(0); expect(x.bb.finished).toBe(true);
  });
  it('runs on the welding station: both programs in sequence through real robots', () => {
    const st = demos.find((d) => d.id === 'welding')!.build(); const [r1, r2] = st.itemsOfType<Robot>(ItemType.ROBOT);
    const world = makeStationWorld({ station: st, programs: perRobotBackend(st) });
    const x = new FsmExecutor({ world, doc: parseFsm('fsm weld\noutcomes done failed\nstate Left initial\n  do program="Weld left" timeout=120\n  ok -> Right\n  fail -> failed\nstate Right\n  do program="Weld right" timeout=120\n  ok -> done\n  fail -> failed') });
    let leftLast = 0, rightFirst = Infinity; const q1 = r1.joints(), q2 = r2.joints(); let p1 = q1, p2 = q2;
    for (let i = 0; i < 3000 && x.status === 'running'; i++) { x.tick(0.05); if (r1.joints().some((v, k) => Math.abs(v - p1[k]) > 1e-6)) leftLast = x.time; if (r2.joints().some((v, k) => Math.abs(v - p2[k]) > 1e-6)) rightFirst = Math.min(rightFirst, x.time); p1 = r1.joints(); p2 = r2.joints(); }
    expect(x.outcome).toBe('done'); expect(rightFirst).toBeGreaterThanOrEqual(leftLast - 0.1); expect(leftLast).toBeGreaterThan(20);
  });
});
