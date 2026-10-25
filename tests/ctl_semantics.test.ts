/**
 * Executor semantics on a real station (welding demo: ABB IRB 2600 + FANUC M-20iA from the library) driven headless
 * through the program backend with one ProgramSimulator per robot — the bugs B1–B6, B13–B18 of the ctl investigation.
 * Assertions are about robot motion, timing and markings, not status strings.
 */
import { describe, it, expect } from 'vitest';
import { demos } from '../src/demos';
import { ItemType, Station, Target } from '../src/core/items/item';
import type { Robot } from '../src/core/items/robot';
import { Program } from '../src/core/items/program';
import { ProgramSimulator } from '../src/core/motion/simulator';
import { transl, getPos, multiply } from '../src/core/math/pose';
import { parseDes, parsePetri } from '../src/ctl/dsl';
import { AutomatonExecutor, PetriExecutor, ExecutorWorld } from '../src/ctl/exec';
import { perRobotBackend, simulatedBackend, mapSignals } from '../src/ctl/host';
import { makeStationWorld } from '../src/ctl/world';
import { PetriNet } from '../src/ctl/petri';
import { simulatedHost } from '../src/ctl/runtime';

const welding = (): Station => demos.find((d) => d.id === 'welding')!.build();
const diff = (a: number[], b: number[]) => Math.max(...a.map((v, i) => Math.abs(v - b[i])));
const progDuration = (st: Station, name: string) => { const s = new ProgramSimulator(st); s.collisionOptions = { enabled: false }; s.compile(st.itemsOfType<Program>(ItemType.PROGRAM).find((p) => p.name === name)!); return s.duration; };

/** Drive an executor on a station, recording when each robot moved. */
function drive(x: { tick(dt: number): string; status: string }, robots: Robot[], seconds: number, dt = 0.05) {
  const prev = robots.map((r) => r.joints()); const moving: number[][] = robots.map(() => []); let t = 0;
  for (let i = 0; i < seconds / dt && x.status === 'running'; i++) {
    x.tick(dt); t += dt;
    robots.forEach((r, k) => { const q = r.joints(); if (diff(q, prev[k]) > 1e-6) moving[k].push(t); prev[k] = q; });
  }
  return { t, moving: moving.map((m) => (m.length ? { first: m[0], last: m[m.length - 1], n: m.length } : null)) };
}

function stationWorld(st: Station) {
  const programs = perRobotBackend(st, { signals: mapSignals(new Map()) });
  const starts: string[] = []; const start0 = programs.start.bind(programs);
  programs.start = (p, o) => { starts.push(p.name); return start0(p, o); };
  return { world: makeStationWorld({ station: st, programs }), starts, programs };
}

const FORK_JOIN = `petri Weld both
place start tokens=1
place pL
place pR
place dL
place dR
place done
transition fork
transition tL
transition tR
transition join
arc start -> fork -> pL, pR
arc pL -> tL -> dL
arc pR -> tR -> dR
arc dL, dR -> join -> done
action tL program="Weld left"
action tR program="Weld right"`;

describe('B1/B2: concurrent robot programs, each started once', () => {
  it('Petri fork/join on the welding cell: both robots weld at the same time, cycle ≈ the longer program', () => {
    const st = welding(); const robots = st.itemsOfType<Robot>(ItemType.ROBOT).slice(0, 2);
    const dL = progDuration(st, 'Weld left'), dR = progDuration(st, 'Weld right');
    const { world, starts } = stationWorld(st);
    const x = new PetriExecutor({ doc: parsePetri(FORK_JOIN), world });
    const res = drive(x, robots, 150);
    expect(x.where()).toBe('done'); expect(x.verdict).toBe('done');
    expect(starts.filter((s) => s === 'Weld left').length).toBe(1); expect(starts.filter((s) => s === 'Weld right').length).toBe(1);
    const [a, b] = res.moving; expect(a && b).toBeTruthy();
    const overlap = Math.min(a!.last, b!.last) - Math.max(a!.first, b!.first);
    expect(overlap).toBeGreaterThan(0.85 * Math.min(dL, dR));
    expect(x.time).toBeLessThanOrEqual(1.1 * Math.max(dL, dR) + 0.5);
    expect(x.time).toBeGreaterThanOrEqual(Math.max(dL, dR) - 0.1);
  });
  it('two automaton components on the two robots finish in one program time; two programs on one robot are serialised', () => {
    const st = welding(); const robots = st.itemsOfType<Robot>(ItemType.ROBOT).slice(0, 2);
    const dL = progDuration(st, 'Weld left'), dR = progDuration(st, 'Weld right');
    const { world } = stationWorld(st);
    const x = new AutomatonExecutor({ world, doc: parseDes(`des two welders
automaton L
  initial I
  marked D
  I -l_go-> W
  W -l_done-> D
automaton R
  initial I
  marked D
  I -r_go-> W
  W -r_done-> D
uncontrollable l_done r_done
action L.W program="Weld left" done=l_done
action R.W program="Weld right" done=r_done`) });
    const res = drive(x, robots, 150);
    expect(x.verdict).toBe('done'); expect(x.time).toBeLessThan(1.1 * Math.max(dL, dR) + 0.5);
    expect(Math.min(res.moving[0]!.last, res.moving[1]!.last) - Math.max(res.moving[0]!.first, res.moving[1]!.first)).toBeGreaterThan(0.85 * Math.min(dL, dR));
    // the same robot twice: the second program waits for the first (no overlap, total ≈ 2 × duration)
    const st2 = welding(); const r1 = st2.itemsOfType<Robot>(ItemType.ROBOT)[0];
    const p2 = st2.addChild(new Program('Weld left again')); p2.setRobot(r1); for (const ins of st2.itemsOfType<Program>(ItemType.PROGRAM).find((p) => p.name === 'Weld left')!.instructions()) p2.addInstruction(JSON.parse(JSON.stringify(ins.data)));
    const w2 = stationWorld(st2);
    const y = new PetriExecutor({ world: w2.world, doc: parsePetri(FORK_JOIN.replace('action tR program="Weld right"', 'action tR program="Weld left again"')) });
    drive(y, [r1], 200);
    expect(y.where()).toBe('done'); expect(y.time).toBeGreaterThan(2 * dL - 0.5);
    const tr = y.trace.filter((e) => e.transition === 'tL' || e.transition === 'tR');
    const endFirst = Math.min(...tr.filter((e) => e.phase === 'end').map((e) => e.t));
    expect(tr.filter((e) => e.phase === 'end').every((e) => e.t >= endFirst)).toBe(true);
    expect(Math.max(...tr.filter((e) => e.phase === 'end').map((e) => e.t)) - endFirst).toBeGreaterThan(dL - 0.5);
  });
});

describe('B3: a program that does not compile fails the operation', () => {
  it('Petri transition bound to an unreachable program: failed, tokens back, robot still, verdict failed', () => {
    const st = welding(); const r1 = st.itemsOfType<Robot>(ItemType.ROBOT)[0];
    const far = st.addChild(new Target('Far away')); far.setPose(transl(20000, 0, 0));
    const bad = st.addChild(new Program('Bad')); bad.setRobot(r1); bad.addMoveJ(far);
    const { world } = stationWorld(st); const q0 = r1.joints();
    const x = new PetriExecutor({ world, doc: parsePetri('petri bad\nplace s tokens=1\nplace d\ntransition tb\narc s -> tb -> d\naction tb program="Bad"') });
    x.run(10, 0.05);
    expect(x.failed).toBe(1); expect(x.where()).toBe('s'); expect(x.verdict).toBe('failed'); expect(diff(q0, r1.joints())).toBeLessThan(1e-9);
    expect(x.log.some((l) => /unreachable|cannot|fail/i.test(l))).toBe(true);
  });
  it('automaton: a failed action fires its fail= event (the failure branch runs), without fail= the component stops', () => {
    const host = simulatedHost({ programs: { Pick: 1, Recover: 1 }, fail: ['Pick'] });
    const w: ExecutorWorld = { host, forRobot: () => null, events: () => host.events!() };
    const x = new AutomatonExecutor({ world: w, doc: parseDes(`des pick with recovery
automaton R
  initial I
  marked I
  I -go-> P
  P -ok-> I
  P -miss-> F
  F -recovered-> I
uncontrollable ok miss recovered
guard go "tries < 1"
blackboard tries=0
action R.P program=Pick done=ok fail=miss
action R.F program=Recover done=recovered`) });
    x.run(10, 0.1, (e) => e.trace.length >= 3);
    expect(x.trace.map((t) => t.event).slice(0, 3)).toEqual(['go', 'miss', 'recovered']); expect(host.runs[0]).toBe('Recover');
    const y = new AutomatonExecutor({ world: { host: simulatedHost({ programs: { Pick: 1 }, fail: ['Pick'] }), forRobot: () => null }, doc: parseDes('des p\nautomaton R\n  initial I\n  marked I\n  I -go-> P\n  P -ok-> I\nuncontrollable ok\naction R.P program=Pick done=ok') });
    y.run(5); expect(y.verdict).toBe('failed'); expect(y.where()).toBe('R=P'); expect(y.fired).toBe(1);
  });
});

describe('B4/B5/B6: targets move their own robot, linear moves are straight, one action per robot', () => {
  it('a target linked to the M-20iA moves the M-20iA even without robot=', () => {
    const st = welding(); const [irb, fanuc] = st.itemsOfType<Robot>(ItemType.ROBOT);
    const t = st.itemsOfType<Target>(ItemType.TARGET).find((x) => x.name === 'Seam M-20iA R 3')!; t.robotId = fanuc.id;
    const q0 = irb.joints(), q1 = fanuc.joints();
    const { world } = stationWorld(st);
    const x = new PetriExecutor({ world, doc: parsePetri('petri m\nplace s tokens=1\nplace d\ntransition t\narc s -> t -> d\naction t target="Seam M-20iA R 3"') });
    x.run(30, 0.05);
    expect(x.verdict).toBe('done'); expect(diff(q0, irb.joints())).toBeLessThan(1e-9); expect(diff(q1, fanuc.joints())).toBeGreaterThan(1);
    const tcp = getPos(multiply(fanuc.poseAbs(), fanuc.solveFK(fanuc.joints()))), goal = getPos(t.poseAbs());
    expect(Math.hypot(tcp[0] - goal[0], tcp[1] - goal[1], tcp[2] - goal[2])).toBeLessThan(0.5);
  });
  it('linear=true keeps the TCP on the straight segment (< 1 mm)', () => {
    const st = welding(); const irb = st.itemsOfType<Robot>(ItemType.ROBOT)[0];
    const t1 = st.itemsOfType<Target>(ItemType.TARGET).find((x) => x.name === 'Seam IRB 2600 L 1')!, t6 = st.itemsOfType<Target>(ItemType.TARGET).find((x) => x.name === 'Seam IRB 2600 L 6')!;
    irb.setJoints(irb.jointsForTarget(t1)!);
    const a = getPos(multiply(irb.poseAbs(), irb.solveFK(irb.joints()))), b = getPos(t6.poseAbs());
    const { world } = stationWorld(st);
    const x = new PetriExecutor({ world, doc: parsePetri('petri l\nplace s tokens=1\nplace d\ntransition t\narc s -> t -> d\naction t target="Seam IRB 2600 L 6" robot="IRB 2600 L" linear=true') });
    let dev = 0;
    for (let i = 0; i < 400 && x.status === 'running'; i++) {
      x.tick(0.05);
      const p = getPos(multiply(irb.poseAbs(), irb.solveFK(irb.joints())));
      const ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]], ap = [p[0] - a[0], p[1] - a[1], p[2] - a[2]]; const L2 = ab[0] ** 2 + ab[1] ** 2 + ab[2] ** 2;
      const u = Math.max(0, Math.min(1, (ap[0] * ab[0] + ap[1] * ab[1] + ap[2] * ab[2]) / L2));
      dev = Math.max(dev, Math.hypot(ap[0] - u * ab[0], ap[1] - u * ab[1], ap[2] - u * ab[2]));
    }
    expect(x.verdict).toBe('done'); expect(Math.hypot(b[0] - a[0], b[1] - a[1], b[2] - a[2])).toBeGreaterThan(400); expect(dev).toBeLessThan(1);
  });
  it('two concurrent transitions moving the same robot are serialised, each reaches its target', () => {
    const st = welding(); const irb = st.itemsOfType<Robot>(ItemType.ROBOT)[0];
    const tg = (n: string) => st.itemsOfType<Target>(ItemType.TARGET).find((x) => x.name === n)!;
    const { world } = stationWorld(st);
    const x = new PetriExecutor({ world, doc: parsePetri(`petri same robot
place a tokens=1
place b tokens=1
place da
place db
transition mA
transition mB
arc a -> mA -> da
arc b -> mB -> db
action mA target="Seam IRB 2600 L 2" robot="IRB 2600 L"
action mB target="Seam IRB 2600 L 5" robot="IRB 2600 L"`) });
    const errAt: Record<string, number> = {};
    for (let i = 0; i < 600 && x.status === 'running'; i++) {
      const before = { ...x.firings }; x.tick(0.05);
      for (const id of ['mA', 'mB']) if ((x.firings[id] ?? 0) > (before[id] ?? 0)) { const p = getPos(multiply(irb.poseAbs(), irb.solveFK(irb.joints()))); const g = getPos(tg(id === 'mA' ? 'Seam IRB 2600 L 2' : 'Seam IRB 2600 L 5').poseAbs()); errAt[id] = Math.hypot(p[0] - g[0], p[1] - g[1], p[2] - g[2]); }
    }
    expect(x.verdict).toBe('done'); expect(errAt.mA).toBeLessThan(0.1); expect(errAt.mB).toBeLessThan(0.1);
    const ends = x.trace.filter((e) => e.phase === 'end'); expect(Math.abs(ends[0].t - ends[1].t)).toBeGreaterThan(0.3);
  });
});

describe('B13/B14: run-time inputs — signals, guards, sensor events', () => {
  it('a Petri transition guarded by a signal fires only after the signal rises; an input transition waits for its event', () => {
    const be = simulatedBackend({ Work: 2 });
    const st = new Station('io'); const p = st.addChild(new Program('Work')); void p;
    const world = makeStationWorld({ station: st, programs: be });
    const x = new PetriExecutor({ world, doc: parsePetri(`petri guarded
place in tokens=1
place mid
place out
transition t guard="part_present"
transition u on=go
arc in -> t -> mid
arc mid -> u -> out
sensor go "count >= 2"
action t program=Work`) });
    for (let i = 0; i < 30; i++) x.tick(0.1);
    expect(x.where()).toBe('in'); expect(x.status).toBe('running');
    be.signals.set('part_present', true);
    for (let i = 0; i < 40; i++) x.tick(0.1);
    expect(x.where()).toBe('mid'); expect(be.log.find((e) => e.event === 'start')!.t).toBeGreaterThanOrEqual(3 - 1e-9);
    be.signals.set('count', 1); x.tick(0.1); expect(x.where()).toBe('mid');
    be.signals.set('count', 2); x.tick(0.1); x.tick(0.1); expect(x.where()).toBe('out'); expect(x.verdict).toBe('done');
  });
  it('DES guarded decision over a counter: found ×3 then none_left (3 PickBox runs)', () => {
    const host = simulatedHost({ programs: { PickBox: 1 } });
    const x = new AutomatonExecutor({ world: { host, forRobot: () => null, events: () => host.events!() }, doc: parseDes(`des depalletize
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
action D.Count set=remaining value="=remaining - 1" done=counted`) });
    x.run(30);
    expect(x.trace.map((t) => t.event).filter((e) => e === 'found' || e === 'none_left')).toEqual(['found', 'found', 'found', 'none_left']);
    expect(host.runs).toEqual(['PickBox', 'PickBox', 'PickBox']); expect(x.verdict).toBe('done');
  });
});

describe('B16/B17/B18: timed semantics, capacities, deadlock verdict', () => {
  const noWorld = (): ExecutorWorld => ({ host: simulatedHost(), forRobot: () => null });
  it('a source transition with delay=1 is single-server: 3 tokens after 3.05 s', () => {
    const x = new PetriExecutor({ world: noWorld(), doc: parsePetri('petri gen\nplace out\ntransition g delay=1\narc g -> out') });
    x.run(3.05, 0.05); expect(x.marking[0]).toBe(3);
    const y = new PetriExecutor({ world: noWorld(), doc: parsePetri('petri gen\nplace out\ntransition g delay=1 servers=2\narc g -> out') });
    y.run(3.05, 0.05); expect(y.marking[0]).toBe(6);
  });
  it('rate= samples a positive exponential duration; immediate transitions pre-empt timed ones', () => {
    const x = new PetriExecutor({ world: noWorld(), seed: 7, doc: parsePetri('petri r\nplace a tokens=1\nplace b\ntransition t rate=0.5\narc a -> t -> b') });
    x.run(60, 0.05, (e) => e.marking[1] === 1); expect(x.marking[1]).toBe(1); expect(x.time).toBeGreaterThanOrEqual(0.1);
    const durations: number[] = []; for (let s = 1; s <= 200; s++) { const y = new PetriExecutor({ world: noWorld(), seed: s, doc: parsePetri('petri r\nplace a tokens=1\nplace b\ntransition t rate=0.5\narc a -> t -> b') }); y.run(100, 0.05, (e) => e.marking[1] === 1); durations.push(y.time); }
    const mean = durations.reduce((s, v) => s + v, 0) / durations.length; expect(mean).toBeGreaterThan(1.6); expect(mean).toBeLessThan(2.5);
    const z = new PetriExecutor({ world: noWorld(), doc: parsePetri('petri imm\nplace a tokens=1\nplace slow\nplace fast\ntransition t1 delay=1\ntransition t2 immediate\narc a -> t1 -> slow\narc a -> t2 -> fast') });
    z.run(2); expect(z.where()).toBe('fast');
  });
  it('capacity=1: the feeding transition is disabled while the place is full (analysis and execution)', () => {
    const src = 'petri cap\nplace src tokens=3\nplace buf capacity=1\nplace out\ntransition feed\ntransition take delay=1\narc src -> feed -> buf\narc buf -> take -> out';
    const net = new PetriNet(parsePetri(src).spec); const rg = net.reachability();
    expect(Math.max(...rg.markings.map((m) => m[1]))).toBe(1);
    const x = new PetriExecutor({ world: noWorld(), doc: parsePetri(src) }); let maxBuf = 0;
    for (let i = 0; i < 100 && x.status === 'running'; i++) { x.tick(0.1); maxBuf = Math.max(maxBuf, x.marking[1]); }
    expect(maxBuf).toBe(1); expect(x.marking[2]).toBe(3);
  });
  it('dead marking classification: final marking → done, any other → deadlock', () => {
    const fin = new PetriExecutor({ world: noWorld(), doc: parsePetri('petri f\nplace a tokens=1\nplace b\ntransition t\narc a -> t -> b\nfinal b') }); fin.run(2); expect(fin.verdict).toBe('done');
    const notFin = new PetriExecutor({ world: noWorld(), doc: parsePetri('petri f\nplace a tokens=1\nplace b\nplace c\ntransition t\narc a -> t -> b\nfinal c') }); notFin.run(2); expect(notFin.verdict).toBe('deadlock');
  });
});
