import { describe, it, expect } from 'vitest';
import { parseDes, parsePetri } from '../src/ctl/dsl';
import { desToDsl, petriToDsl, setAction, actionOf } from '../src/ctl/graphdoc';
import { AutomatonExecutor, PetriExecutor, ExecutorWorld } from '../src/ctl/exec';
import { simulatedHost, simulatedBindings } from '../src/ctl/runtime';
import { analyse } from '../src/ctl/analysis';
import { DES, parallel, supcon, supervisorTable } from '../src/ctl/des';

const world = (host = simulatedHost({ programs: { 'Pick part': 2, 'Place part': 1.5, Weld: 3 } }), mobile?: ReturnType<typeof simulatedBindings>): ExecutorWorld & { host: ReturnType<typeof simulatedHost> } => ({ host, forRobot: (name) => (mobile && name === 'AMR' ? mobile : null), step: (dt) => mobile?.step?.(dt), events: () => [...(mobile?.events?.() ?? []), ...(host.events?.() ?? [])] });

const CELL_DES = `des Machine and robot
automaton M
  initial Idle
  marked Idle
  Idle -load-> Busy
  Busy -done-> Idle
automaton R
  initial Free
  marked Free
  Free -load-> Loading
  Loading -loaded-> Free
uncontrollable done loaded
action R.Loading program="Pick part" robot=UR done=loaded
action M.Busy wait=1 done=done`;

describe('Action bindings in the DSL', () => {
  it('round-trip through the writers, and the analysis lists them', () => {
    const doc = parseDes(CELL_DES);
    expect(doc.actions.length).toBe(2); expect(doc.actions[0]).toMatchObject({ target: 'R.Loading', kind: 'program', value: 'Pick part', robot: 'UR', done: 'loaded' });
    const again = parseDes(desToDsl(doc)); expect(again.actions).toEqual(doc.actions);
    setAction(doc.actions, 'M.Busy', { kind: 'signal', value: 'start', component: 'M1', args: { value: 1 } });
    expect(actionOf(parseDes(desToDsl(doc)).actions, 'M.Busy')).toMatchObject({ kind: 'signal', component: 'M1', args: { value: 1 } });
    const r = analyse('des', desToDsl(doc)); expect(r.error).toBeUndefined(); expect(r.sections.find((s) => s.title.startsWith('Bound actions'))!.lines.length).toBe(2);
    setAction(doc.actions, 'M.Nowhere', { kind: 'wait', value: '1', args: {} }); expect(analyse('des', desToDsl(doc)).sections.find((s) => s.title.startsWith('Bound actions'))!.level).toBe('warn');
    const p = parsePetri('petri x\nplace p tokens=1\ntransition t\narc p -> t\naction t program="Weld" robot=KUKA');
    expect(p.actions[0]).toMatchObject({ target: 't', kind: 'program', value: 'Weld', robot: 'KUKA' });
    expect(parsePetri(petriToDsl({ spec: p.spec, checks: p.checks, horizon: p.horizon, layout: p.layout, actions: p.actions })).actions).toEqual(p.actions);
  });
});

describe('Automaton executor (parallel state machines with entry actions)', () => {
  it('runs the machine / robot cycle: load → program on the robot → loaded → done → idle', () => {
    const w = world(); const x = new AutomatonExecutor({ doc: parseDes(CELL_DES), world: w });
    x.run(10, 0.1, (e) => e.fired >= 4); x.tick(0.1); // one more tick: the second program starts
    expect(x.trace.map((t) => t.event).slice(0, 4)).toEqual(['load', 'done', 'loaded', 'load']); // the machine's 1 s wait ends before the 2 s program
    expect(w.host.runs).toEqual(['Pick part', 'Pick part']);
    expect(x.mismatches).toBe(0); expect(x.failed).toBe(0);
    const t0 = x.trace[0].t; expect(x.trace[1].t - t0).toBeGreaterThanOrEqual(0.9); expect(x.trace[2].t - t0).toBeGreaterThanOrEqual(1.9); expect(x.trace[2].t - t0).toBeLessThan(2.4);
  });
  it('waits for the shared program simulator, respects the supervisor, and goes idle when nothing is enabled', () => {
    const src = `des Two robots one simulator
automaton A
  initial I
  marked I
  I -a_go-> W
  W -a_done-> I
automaton B
  initial I
  marked I
  I -b_go-> W
  W -b_done-> I
spec OneAtATime
  initial N
  marked N
  N -a_go-> A
  A -a_done-> N
  N -b_go-> B
  B -b_done-> N
uncontrollable a_done b_done
action A.W program="Pick part" robot=R1 done=a_done
action B.W program="Place part" robot=R2 done=b_done`;
    const doc = parseDes(src); const g = parallel(...doc.plant.map((a) => DES.fromSpec(a)));
    const sup = supervisorTable(supcon(g, doc.specs.map((a) => DES.fromSpec(a)), doc.uncontrollable).supervisor, doc.uncontrollable);
    const w = world(); const x = new AutomatonExecutor({ doc, world: w, supervisor: sup });
    x.run(12, 0.1, (e) => e.fired >= 4);
    expect(x.mismatches).toBe(0); expect(x.denied).toBeGreaterThan(0); // b_go denied while A works
    const evs = x.trace.map((t) => t.event); expect(evs[0]).toBe('a_go'); expect(evs[1]).toBe('a_done'); expect(evs[2]).toBe('b_go');
    // without the supervisor both start; the second program waits for the simulator instead of overlapping
    const w2 = world(); const y = new AutomatonExecutor({ doc: parseDes(src.replace(/spec OneAtATime[\s\S]*?uncontrollable/, 'uncontrollable')), world: w2 });
    y.run(12, 0.1, (e) => e.fired >= 4); y.tick(0.1);
    expect(w2.host.runs.slice(0, 2)).toEqual(['Pick part', 'Place part']); expect(y.mismatches).toBe(0); // B queued first for the simulator, so it runs before A's second cycle
    // a terminal automaton becomes idle
    const z = new AutomatonExecutor({ doc: parseDes('des one\nautomaton S\n  initial a\n  marked b\n  a -go-> b\naction S.a wait=0.3'), world: world() });
    z.run(5); expect(z.status).toBe('idle'); expect(z.where()).toBe('S=b');
  });
  it('drives a mobile robot with goto and reports world events into the automaton', () => {
    const mobile = simulatedBindings({}, undefined, { speed: 5 });
    const doc = parseDes(`des AMR shuttle
automaton Base
  initial H
  marked H
  H -go_table-> mT
  mT -arrive-> T
  T -go_home-> mH
  mH -arrive-> H
uncontrollable arrive
action Base.mT goto=table robot=AMR done=arrive
action Base.mH goto=home robot=AMR done=arrive`);
    const x = new AutomatonExecutor({ doc, world: world(undefined, mobile) });
    x.run(60, 0.1, (e) => e.fired >= 4);
    expect(x.trace.map((t) => t.event).slice(0, 4)).toEqual(['go_table', 'arrive', 'go_home', 'arrive']); expect(x.mismatches).toBe(0);
  });
});

describe('Petri executor (transitions as robot operations)', () => {
  it('fires transitions with real durations, shares the robot through a resource place, and detects the deadlock of the course cell', () => {
    const src = `petri Two-machine cell with one robot
place inA tokens=2
place inB tokens=2
place robot tokens=1 kind=resource
place doneA
place doneB
transition tA
transition tB
arc inA, robot -> tA -> doneA, robot
arc inB, robot -> tB -> doneB, robot
action tA program="Pick part" robot=UR
action tB program="Place part" robot=UR`;
    const p = parsePetri(src); const w = world();
    const x = new PetriExecutor({ doc: { spec: p.spec, checks: p.checks, horizon: p.horizon, layout: p.layout, actions: p.actions }, world: w });
    x.run(30);
    expect(x.status).toBe('idle'); expect(x.firings).toEqual({ tA: 2, tB: 2 }); expect(w.host.runs.length).toBe(4);
    expect(x.time).toBeGreaterThanOrEqual(6.9); // 2×2 s + 2×1.5 s sequential on one robot
    expect(x.where()).toBe('robot doneA×2 doneB×2');
    // the course deadlock: both processes hold one resource and wait for the other
    const dl = parsePetri(`petri deadlock
place p1 tokens=1
place a1
place p2 tokens=1
place b1
place rM tokens=1 kind=resource
place rZ tokens=1 kind=resource
transition t1a delay=1
transition t2a delay=1
transition t1b delay=1
transition t2b delay=1
arc p1, rM -> t1a -> a1
arc a1, rZ -> t2a -> p1, rM, rZ
arc p2, rZ -> t1b -> b1
arc b1, rM -> t2b -> p2, rM, rZ`);
    const y = new PetriExecutor({ doc: { spec: dl.spec, checks: [], horizon: 300, layout: {}, actions: [] }, world: world() });
    y.run(20); expect(y.status).toBe('idle'); expect(y.where()).toBe('a1 b1'); expect(y.log.at(-1)).toMatch(/deadlock/);
  });
});
