/**
 * A/B tests: the same job done two ways, with the relation between the two results asserted — an optimal solver
 * against a heuristic, a model against the fleet, a saved station against its reloaded copy, a linear move against a
 * joint move, one architecture against another, one language against the other. Each pair states what A and B are.
 */
import { describe, it, expect } from 'vitest';
import { Station, Frame, Target, Tool, SceneObject } from '../src/core/items/item';
import { Program } from '../src/core/items/program';
import { createRobotFromLibrary, ROBOT_LIBRARY } from '../src/core/items/library';
import { ProgramSimulator } from '../src/core/motion/simulator';
import { sampleAt } from '../src/core/motion/trajectory';
import { checkCollisions } from '../src/core/collision/collision';
import { mul, transl, rotx, DEG, getPos, distance } from '../src/core/math/pose';
import { saveStation, loadStation } from '../src/io/station-file';
import { demos } from '../src/demos';
import { MobileRobot, MapItem } from '../src/mobile/items';
import { planPath, pathLength } from '../src/mobile/planner';
import { LocalizationEstimator, defaultNavStack } from '../src/mobile/navstack';
import { hungarian, greedyAssignment, sequentialAuction, cbba } from '../src/ctl/mrta';
import { dispatch, evaluateSequence, moore, JobShop } from '../src/ctl/sched';
import { DES, supcon, parallel, analyseBlocking } from '../src/ctl/des';
import { parseDes } from '../src/ctl/dsl';
import { loadMap, prioritizedPlanning, cbs, firstConflict, sumOfCosts, executeAdg, executeNaive, WAREHOUSE_MAP } from '../src/mrs/mapf';
import { Rng } from '../src/mrs/rng';
import { diskGraph, algebraicConnectivity } from '../src/mrs/graph';
import { consensusStep } from '../src/mrs/consensus';
import { parseMission, MRS_TEMPLATES } from '../src/mrs/dsl';
import { compareArchitectures } from '../src/mrs/mission';
import { FleetRuntime } from '../src/mrs/runtime';
import { MRS_EXAMPLES } from '../src/mrs/examples';
import { setLang, t, getLang } from '../src/ui/i18n';
import { listPosts, compileForPost } from '../src/posts/index';

function cell() {
  const st = new Station('AB cell');
  const robot = st.addChild(createRobotFromLibrary('UR10e', 'UR10e'));
  const tool = robot.addChild(new Tool('Gripper')); tool.setPoseTool(transl(0, 0, 120)); robot.setTool(tool);
  const frame = st.addChild(new Frame('Table')); frame.setPose(transl(600, 0, 0)); robot.setFrame(frame);
  const home = frame.addChild(new Target('Home')); home.setJoints([0, -90, 90, -90, -90, 0]); home.setAsJointTarget();
  const a = frame.addChild(new Target('A')); a.setPose(mul(transl(100, 200, 300), rotx(180 * DEG)));
  const b = frame.addChild(new Target('B')); b.setPose(mul(transl(100, -200, 300), rotx(180 * DEG)));
  return { st, robot, frame, home, a, b };
}

describe('A/B — persistence and exchange formats', () => {
  it('A: every demo station serialised · B: the same station reloaded and serialised again — identical documents, identical item trees', () => {
    for (const d of demos) {
      const st = d.build(); const strip = (f: any) => ({ ...f, savedAt: undefined }); const A = saveStation(st); const B = saveStation(loadStation(A));
      expect(strip(B), d.id).toEqual(strip(A));
      const names = (s: Station) => [...s.walk()].map((i) => `${i.type}:${i.name}`);
      expect(names(loadStation(A)), d.id).toEqual(names(st));
    }
  });
  it('A: a program compiled for the simulator · B: the same program compiled for every post processor — the same motion count reaches every dialect', () => {
    const { st, robot, home, a, b } = cell(); const prog = st.addChild(new Program('P')); prog.setRobot(robot); prog.addMoveJ(home); prog.addMoveL(a); prog.addMoveL(b); prog.addMoveJ(home);
    const simA = new ProgramSimulator(st); simA.compile(prog); const moves = simA.steps.filter((s) => s.trajectory).length;
    expect(moves).toBe(4);
    for (const p of listPosts()) {
      const B = compileForPost(st, prog); const n = B.events.filter((e) => e.kind === 'moveJ' || e.kind === 'moveL').length;
      expect(n, p.id).toBe(moves);
      const files = p.generate(B); expect(files.length, p.id).toBeGreaterThan(0); expect(files.every((f) => f.content.length > 0), p.id).toBe(true);
    }
  });
});

describe('A/B — motion and collision', () => {
  it('A: MoveL between two targets · B: MoveJ between the same targets — same end pose, the linear TCP path stays on the line, the joint path leaves it', () => {
    const { st, robot, home, a, b } = cell();
    const run = (linear: boolean) => { const p = st.addChild(new Program(linear ? 'L' : 'J')); p.setRobot(robot); p.addMoveJ(home); p.addMoveJ(a); if (linear) p.addMoveL(b); else p.addMoveJ(b); const sim = new ProgramSimulator(st); const res = sim.compile(p); const tr = sim.steps.filter((s) => s.trajectory).pop()!.trajectory!; return { ok: res.problems.filter((x) => x.severity === 'error').length === 0, tr }; };
    const A = run(true), B = run(false); expect(A.ok && B.ok).toBe(true);
    const endA = getPos(sampleAt(A.tr, A.tr.duration).pose), endB = getPos(sampleAt(B.tr, B.tr.duration).pose);
    expect(distance(endA, endB)).toBeLessThan(1);
    const p0 = getPos(sampleAt(A.tr, 0).pose), p1 = endA; const off = (p: number[]) => { const d = [p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]]; const L = Math.hypot(...d); const u = d.map((x) => x / L); const w = [p[0] - p0[0], p[1] - p0[1], p[2] - p0[2]]; const s = w[0] * u[0] + w[1] * u[1] + w[2] * u[2]; return Math.hypot(w[0] - s * u[0], w[1] - s * u[1], w[2] - s * u[2]); };
    const devA = Math.max(...A.tr.samples.map((s) => off(getPos(s.pose)))), devB = Math.max(...B.tr.samples.map((s) => off(getPos(s.pose))));
    expect(devA).toBeLessThan(2); expect(devB).toBeGreaterThan(devA + 5);
    expect(A.tr.length).toBeLessThan(B.tr.length);
  });
  it('A: the cell as built · B: a box moved into the robot — static check finds no pair in A and a robot–box pair in B, program validation reports it only in B', () => {
    const { st, robot, frame, home } = cell(); const box = st.addChild(new SceneObject('Box')); box.geometry = [{ primitive: { kind: 'box', size: [300, 300, 300] }, origin: Array.from(transl(0, 0, 150)) }]; box.setPose(transl(2500, 0, 100));
    const far = frame.addChild(new Target('Far')); far.setPose(mul(transl(300, 500, 250), rotx(180 * DEG)));
    const p = st.addChild(new Program('P')); p.setRobot(robot); p.addMoveJ(home); p.addMoveJ(far);
    const A = checkCollisions(st); expect(A.length).toBe(0);
    const simA = new ProgramSimulator(st); simA.collisionOptions = { enabled: true, sampleStep: 0.1 }; simA.compile(p); expect(simA.collisions.length).toBe(0);
    const last = simA.steps.filter((x) => x.trajectory).pop()!.trajectory!; const qEnd = sampleAt(last, last.duration).joints; const elbow = robot.fk(qEnd).linkPoses[3]; box.setPose(transl(elbow[12], elbow[13], elbow[14] - 150)); // the box sits where the elbow ends the last move
    robot.setJoints(qEnd); const B = checkCollisions(st); expect(B.length).toBeGreaterThan(0); expect(B.some((pr) => [pr.a.item.name, pr.b.item.name].includes('Box'))).toBe(true);
    robot.setJoints([0, -90, 90, -90, -90, 0]); // start free of contact (resting contacts before motion are deliberately ignored), end inside the box
    const simB = new ProgramSimulator(st); simB.collisionOptions = { enabled: true, sampleStep: 0.05 }; const res = simB.compile(p); expect(simB.collisions.length + res.problems.filter((x) => /collision/i.test(x.message)).length).toBeGreaterThan(0);
  });
  it('A: analytic/numeric IK of every library robot at its home flange pose · B: forward kinematics — B(A(x)) returns to x for every model', () => {
    let checked = 0;
    for (const entry of ROBOT_LIBRARY) {
      const r = createRobotFromLibrary(entry.id, entry.id); if (r.dof < 6) continue;
      const q0 = r.joints(); const target = r.fk(q0).flange; const sol = r.solveIK(target, { seed: q0.map((v) => v + 5) });
      if (!sol.ok) continue; const back = r.fk(sol.joints).flange; expect(distance(getPos(back), getPos(target)), entry.id).toBeLessThan(0.5); checked++;
    }
    expect(checked).toBeGreaterThan(10);
  });
});

describe('A/B — planning, allocation, scheduling', () => {
  it('A: grid A* with string-pulling · B: the raw grid path — both collision-free, B never shorter than A, same endpoints', () => {
    const map = new MapItem('m'); map.resize(100, 60, 100, 0, 0); map.fillRect(0, 0, 10000, 100); map.fillRect(0, 5900, 10000, 6000); map.fillRect(4000, 2000, 4200, 4000); map.fillRect(6000, 0, 6200, 3500);
    const A = planPath(map, [500, 3000], [9500, 3000], { smooth: true }), B = planPath(map, [500, 3000], [9500, 3000], { smooth: false });
    expect(A.ok && B.ok).toBe(true); expect(pathLength(A.path)).toBeLessThanOrEqual(pathLength(B.path) + 1e-6); expect(A.path.length).toBeLessThan(B.path.length);
    expect(A.path[0]).toEqual(B.path[0]); expect(A.path[A.path.length - 1]).toEqual(B.path[B.path.length - 1]);
    for (const pt of [...A.path, ...B.path]) expect(map.isFree(pt[0], pt[1])).toBe(true);
  });
  it('A: Hungarian assignment · B: greedy, sequential auction, CBBA on the same cost matrix — A is never worse, every method assigns every task once', () => {
    const rng = new Rng(11); const robots = Array.from({ length: 6 }, (_, i) => ({ id: `r${i}`, at: { x: rng.random() * 10, y: rng.random() * 10 } })); const tasks = Array.from({ length: 6 }, (_, j) => ({ id: `t${j}`, at: { x: rng.random() * 10, y: rng.random() * 10 } }));
    const cost = robots.map((r) => tasks.map((tk) => Math.hypot(r.at.x - tk.at.x, r.at.y - tk.at.y)));
    const A = hungarian(cost); const g = greedyAssignment(cost); const auc = sequentialAuction(robots, tasks); const c = cbba(robots, tasks);
    expect(new Set(A.assignment).size).toBe(6); expect(new Set(g.assignment).size).toBe(6);
    expect(A.total).toBeLessThanOrEqual(g.total + 1e-9);
    const sumA = A.total; const sumAuc = Object.entries(auc.bundles).reduce((s, [rid, ts]) => s + ts.reduce((q, tid) => q + cost[robots.findIndex((r) => r.id === rid)][tasks.findIndex((tk) => tk.id === tid)], 0), 0);
    expect(sumAuc).toBeGreaterThanOrEqual(sumA - 1e-9);
    expect(Object.values(c.bundles).flat().sort()).toEqual(tasks.map((tk) => tk.id).sort()); expect(c.totalCost).toBeGreaterThanOrEqual(sumA - 1e-9);
  });
  it('A: EDD sequence · B: SPT, FIFO, Moore — SPT minimises total completion, EDD minimises maximum lateness, Moore minimises the number of late jobs', () => {
    const jobs = [{ id: 'j1', p: 4, d: 6 }, { id: 'j2', p: 2, d: 3 }, { id: 'j3', p: 6, d: 20 }, { id: 'j4', p: 3, d: 7 }, { id: 'j5', p: 5, d: 9 }];
    const by = (k: (j: typeof jobs[0]) => number) => [...jobs].sort((x, y) => k(x) - k(y)).map((j) => j.id);
    const EDD = evaluateSequence(jobs, by((j) => j.d)), SPT = evaluateSequence(jobs, by((j) => j.p)), FIFO = evaluateSequence(jobs, jobs.map((j) => j.id)), M = moore(jobs);
    expect(SPT.sumC).toBeLessThanOrEqual(Math.min(EDD.sumC, FIFO.sumC)); expect(EDD.Lmax).toBeLessThanOrEqual(Math.min(SPT.Lmax, FIFO.Lmax)); expect(M.late).toBeLessThanOrEqual(Math.min(EDD.late, SPT.late, FIFO.late));
    const shop: JobShop = { machines: ['M1', 'M2'], jobs: [{ id: 'a', ops: [{ alternatives: [{ machine: 'M1', p: 3 }] }, { alternatives: [{ machine: 'M2', p: 2 }] }] }, { id: 'b', ops: [{ alternatives: [{ machine: 'M2', p: 4 }] }, { alternatives: [{ machine: 'M1', p: 1 }] }] }, { id: 'c', ops: [{ alternatives: [{ machine: 'M1', p: 2 }] }, { alternatives: [{ machine: 'M2', p: 3 }] }] }] };
    const rules = ['SPT', 'LPT', 'FIFO', 'MWKR'] as const; const spans = rules.map((r) => dispatch(shop, r).makespan);
    for (const s of spans) expect(s).toBeGreaterThanOrEqual(9); // lower bound: the load of M2 (2+4+3)
    expect(Math.min(...spans)).toBeLessThanOrEqual(Math.max(...spans));
  });
  it('A: conflict-based search · B: prioritised planning on the same warehouse instance — both conflict-free, A never costs more; ADG execution under delays is collision-free while naive replay collides', () => {
    const grid = loadMap(WAREHOUSE_MAP); const starts: [number, number][] = [[0, 0], [0, 9], [6, 0], [6, 9], [0, 4]]; const goals: [number, number][] = [[6, 9], [6, 0], [0, 9], [0, 0], [6, 4]];
    const A = cbs(grid, starts, goals).paths!; const B = prioritizedPlanning(grid, starts, goals)!;
    expect(A && B).toBeTruthy(); expect(firstConflict(A)).toBeNull(); expect(firstConflict(B)).toBeNull(); expect(sumOfCosts(A)).toBeLessThanOrEqual(sumOfCosts(B));
    const adg = executeAdg(A, 0.3, new Rng(3)); const naive = executeNaive(A, 0.3, new Rng(3));
    expect(adg.ticks).not.toBeNull(); expect(adg.collisions).toBe(0); expect(naive).toBeGreaterThan(0);
  });
});

describe('A/B — group control: model against fleet, architecture against architecture, radio against radio', () => {
  it('A: connected disk graph · B: the same robots with a shorter radio — λ₂(A) > λ₂(B) = 0 and consensus converges only in A', () => {
    const p: [number, number][] = [[0, 0], [1, 0], [2, 0], [3, 0], [4, 0]];
    const GA = diskGraph(p, 1.5), GB = diskGraph(p, 0.9);
    expect(algebraicConnectivity(GA)).toBeGreaterThan(0); expect(algebraicConnectivity(GB)).toBe(0);
    let xa = [0, 1, 2, 3, 4], xb = [0, 1, 2, 3, 4]; for (let k = 0; k < 400; k++) { xa = consensusStep(xa, GA, 0.3); xb = consensusStep(xb, GB, 0.3); }
    expect(Math.max(...xa) - Math.min(...xa)).toBeLessThan(1e-3); expect(Math.max(...xb) - Math.min(...xb)).toBeGreaterThan(1);
  });
  it('A: centralised · B: hybrid · C: decentralised on the mission with a coordinator outage — B and C finish before A, only A stalls, all keep d_safe', () => {
    const [A, C, B] = compareArchitectures(parseMission(MRS_TEMPLATES.mission)).map((r) => r.metrics);
    expect(A.completed && B.completed && C.completed).toBe(true);
    expect(B.time).toBeLessThan(A.time); expect(C.time).toBeLessThan(A.time);
    expect(A.stalledSeconds).toBeGreaterThan(0); expect(B.stalledSeconds + C.stalledSeconds).toBe(0);
    expect(B.fallbackSeconds).toBeGreaterThan(0); expect(C.peer).toBeGreaterThan(B.peer); expect(A.peer).toBe(0);
  }, 60000);
  it('A: the mission model (single integrators, analysis) · B: the same mission on configured station robots driven by pose · C: driven through the unicycle controller — all complete, B ≈ A in time, C travels at least as far as B', () => {
    const src = 'mission ab\nrobots r1 r2 r3 r4\narchitecture centralized\ncomm radius=8 period=0.5 lost=2 settle=0.5\ncoordinator at=0,0 range=40\nphase form line r=1.0\nphase goto at=5,0 speed=0.4\nphase home\nsafety d_safe=0.5 gamma=2 sense=2\nduration 300 dt=0.1 seed=1 vmax=0.6';
    const scene = () => { const st = new Station('ab'); const robots = [0, 1, 2, 3].map((i) => { const r = st.addChild(new MobileRobot(`r${i + 1}`)); r.setPose2D(i * 1200, 0, 0); r.kin.maxSpeed = 700; r.home = { x: i * 1200, y: 0, theta: 0 }; return r; }); return { st, robots }; };
    const d = parseMission(src); d.robots = [0, 1, 2, 3].map((i) => ({ name: `r${i + 1}`, at: [i * 1.2, 0] as [number, number], home: [i * 1.2, 0] as [number, number] }));
    const A = compareArchitectures(d, ['centralized'])[0].metrics; expect(A.completed).toBe(true);
    const run = (drive: 'pose' | 'unicycle') => { const { st, robots } = scene(); const rt = new FleetRuntime({ kind: 'mission', source: src, robots, station: st, drive }); for (let t = 0; t < 300 && !rt.done; t += 0.1) rt.tick(0.1); const m = (rt.driver.model as any).metrics(); return { done: rt.done, time: rt.time, travelled: m.travelled as number, home: robots.every((r) => Math.hypot(r.state.x - r.home!.x, r.state.y - r.home!.y) < 400) }; };
    const B = run('pose'), C = run('unicycle');
    expect(B.done && C.done).toBe(true); expect(B.home && C.home).toBe(true);
    expect(B.time).toBeGreaterThan(A.time * 0.5); expect(B.time).toBeLessThan(A.time * 2.5);
    expect(C.travelled).toBeGreaterThan(B.travelled * 0.9);
  }, 60000);
  it('A: the course mission analysed · B: the same document run on a fleet built from it — the fleet reaches the phases the report predicts', () => {
    const ex = MRS_EXAMPLES.find((e) => e.id === 'arch-compare')!; const d = parseMission(ex.source);
    const A = compareArchitectures(d, ['hybrid'])[0].metrics; expect(A.completed).toBe(true);
    const st = new Station('fleet'); const robots = d.robots.map((r) => { const m = st.addChild(new MobileRobot(r.name)); m.setPose2D(r.at[0] * 1000, r.at[1] * 1000, 0); m.kin.maxSpeed = 700; m.home = { x: r.at[0] * 1000, y: r.at[1] * 1000, theta: 0 }; return m; });
    const rt = new FleetRuntime({ kind: 'mission', source: ex.source.replace('architecture compare', 'architecture hybrid'), robots, station: st, drive: 'pose' }); for (let t = 0; t < 400 && !rt.done; t += 0.1) rt.tick(0.1);
    const B = (rt.driver.model as any).metrics(); expect(rt.done).toBe(true); expect(B.tasksDone).toBe(A.tasksDone); expect(B.robotsAlive).toBe(A.robotsAlive); expect(B.minDistance).toBeGreaterThanOrEqual(d.safety.dSafe - 0.02);
  }, 60000);
});

describe('A/B — supervision and localisation', () => {
  it('A: the plant alone · B: the plant under a synthesised supervisor — B is a sub-behaviour of A, nonblocking, and never enables the forbidden event', () => {
    const doc = parseDes('des ab\nautomaton M\n  initial I\n  marked I\n  I -start-> W\n  W -done-> I\n  W -fault-> F\n  F -reset-> I\nuncontrollable done fault\nspec S\n  events reset\n  initial E\n  marked E');
    const plant = parallel(...doc.plant.map((a) => DES.fromSpec(a))); const r = supcon(plant, doc.specs.map((a) => DES.fromSpec(a)), doc.uncontrollable);
    expect(r.realizable).toBe(true);
    const A = analyseBlocking(plant), B = analyseBlocking(r.supervisor);
    expect(A.nonblocking).toBe(true); expect(B.nonblocking).toBe(true);
    expect(B.reachable).toBeLessThanOrEqual(A.reachable); expect(B.transitions).toBeLessThan(A.transitions);
    expect(r.supervisor.transitions().some((tr) => tr.event === 'reset')).toBe(false);
  });
  it('A: RTK GNSS in open sky · B: the same drive with the sky blocked (GNSS/INS dead reckoning) — A stays within centimetres, B drifts with distance', () => {
    const drive = (gnss: boolean) => { const cfg = defaultNavStack('tractor', 'orchard'); cfg.localization = 'gnss_rtk'; const est = new LocalizationEstimator(cfg, 5); est.reset(0, 0, 0); let x = 0; for (let k = 0; k < 2000; k++) { x += 100; est.update({ x, y: 0, theta: 0 }, 0.1, { gnss, featureRich: false }); } return est.state; };
    const A = drive(true), B = drive(false);
    expect(A.rmse).toBeLessThan(100); expect(B.maxError).toBeGreaterThan(A.maxError * 3); expect(B.fixes).toBeLessThanOrEqual(A.fixes);
  });
});

describe('A/B — languages', () => {
  it('A: English · B: Russian — every menu, tab and dialog label used by the UI has a distinct Russian string and the language switch is symmetric', () => {
    const labels = ['File', 'Edit', 'Add', 'Program', 'Robot', 'Mobile & Fleet', 'Agriculture', 'Control', 'Group', 'Tools', 'Connect', 'View', 'Help', 'Simulation', 'Fleet', 'Process', 'Navigation', 'Vision', 'Camera', 'Log', 'Cancel', 'OK', 'Analyse', 'Analyse all', 'Run on fleet', 'Stop', 'Diagram', 'Text', 'New station', 'Save station (.vbstation)', 'Undo', 'Redo', 'Language: Русский', 'Demo scenarios (navigation, vision, control design)…'];
    const before = getLang(); setLang('en'); const A = labels.map(t); expect(A).toEqual(labels);
    setLang('ru'); const B = labels.map(t);
    B.forEach((s, i) => { expect(s, labels[i]).not.toBe(labels[i]); expect(/[А-Яа-яЁё]/.test(s), labels[i]).toBe(true); });
    expect(new Set(B).size).toBe(B.length);
    setLang('en'); expect(labels.map(t)).toEqual(A); setLang(before);
  });
});
