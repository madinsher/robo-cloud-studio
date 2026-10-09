import { describe, it, expect } from 'vitest';
import { hungarian, greedyAssignment, bottleneckAssignment, sequentialAuction, cbba, priorityPlanning, cbs, buildTPG, simulateTPG, reservationDeadlockCheck, criticalSections, Graph, routeCost } from '../src/ctl/mrta';
import { spt, edd, moore, johnson, dispatch, lowerBounds, bestAssignmentBound, tabuSearch, branchAndBound, criticalPath, rightShift, scheduleDeviation, scheduleRobustness, oee, classify, cellJobShop, JobShop } from '../src/ctl/sched';
import { responseTimes, edfTest, blockingBounds, endToEndLatency, latencyBudget, wcetEstimate, qosRecommendation, placementAdvice, COURSE_TASKS, liuLaylandBound, RTTask } from '../src/ctl/realtime';
import { systemReliability, CELL_COMPONENTS, minimalCutSets, topEventProbability, ftaSensitivity, CONTACT_FTA, requiredPL, achievedPL, plMeets, separationDistance, admissibleSpeed, detectionThreshold, reliabilityWeibull, optimalReplacement, markovAvailability, fmeaTable, actionPriority, stopCategory, countsTowardsPL } from '../src/ctl/reliability';
import { parseSTL, robustness, formatSTL, falsify, pairwise, ruleOfThree, trialsForRuleOfThree, clopperPearsonLower, trialsForTarget, trialsForRate, simRealGap, acceptanceCheck, impactOfChange, traceabilitySummary, Signal } from '../src/ctl/vv';

describe('MRTA and MAPF (chapter 13)', () => {
  it('Hungarian beats greedy; course corridor matrix gives 81 s / makespan 55', () => {
    const c = [[55, 62, 70], [40, 12, 35], [48, 30, 14]];
    const h = hungarian(c); expect(h.total).toBe(81); expect(h.assignment).toEqual([0, 1, 2]);
    expect(bottleneckAssignment(c).makespan).toBe(55);
    const c2 = [[40, 41, 90], [39, 80, 85], [70, 75, 95]];
    expect(hungarian(c2).total).toBe(175); expect(greedyAssignment(c2).total).toBeGreaterThanOrEqual(175);
    let s = 5; const rnd = () => { s = (s * 1664525 + 1013904223) >>> 0; return s / 4294967296; };
    let worse = 0;
    for (let k = 0; k < 50; k++) { const m = Array.from({ length: 5 }, () => Array.from({ length: 5 }, () => Math.round(10 + 90 * rnd()))); const hh = hungarian(m).total, gg = greedyAssignment(m).total; expect(hh).toBeLessThanOrEqual(gg); if (gg > hh) worse++; }
    expect(worse).toBeGreaterThan(10);
    expect(hungarian([[1, Infinity], [Infinity, 1], [2, 2]]).assignment.filter((a) => a >= 0).length).toBe(2);
  });
  it('sequential auction and CBBA assign every task once and stay within 2× of the optimum-ish', () => {
    const robots = [{ id: 'r1', at: { x: 0, y: 0 } }, { id: 'r2', at: { x: 10, y: 0 } }, { id: 'r3', at: { x: 5, y: 8 } }];
    const tasks = Array.from({ length: 6 }, (_, i) => ({ id: `t${i}`, at: { x: (i * 3.7) % 10, y: (i * 2.3) % 8 }, duration: 1 }));
    const a = sequentialAuction(robots, tasks); const all = Object.values(a.bundles).flat(); expect(all.sort()).toEqual(tasks.map((t) => t.id).sort());
    const c = cbba(robots, tasks); expect(c.converged).toBe(true); expect(Object.values(c.bundles).flat().sort()).toEqual(tasks.map((t) => t.id).sort());
    expect(c.totalCost).toBeLessThan(2 * a.totalCost + 1);
    expect(routeCost(robots[0], [tasks[0]])).toBeGreaterThan(0);
    const c2 = cbba(robots.slice(0, 2), tasks, { neighbours: (id) => (id === 'r1' ? ['r2'] : ['r1']) }); expect(c2.converged).toBe(true);
  });
  it('priority planning fails on the corridor counterexample, CBS solves it, the TPG is acyclic and robust to delays', () => {
    const g: Graph = { nodes: ['a', 'c1', 'c2', 'b', 'p'], edges: [['a', 'c1'], ['c1', 'c2'], ['c2', 'b'], ['c1', 'p']] };
    const agents = [{ id: 'A', start: 'a', goal: 'b' }, { id: 'B', start: 'b', goal: 'a' }];
    const pp = priorityPlanning(g, agents, ['A', 'B']); expect(pp.failed).toEqual(['B']);
    const pp2 = priorityPlanning(g, agents, ['B', 'A']); expect(pp2.failed).toEqual([]);
    const r = cbs(g, agents); expect(r.found).toBe(true); expect(r.paths.A).toContain('p');
    const tpg = buildTPG(r.paths); expect(tpg.acyclic).toBe(true); expect(tpg.arcs.some((x) => x.kind === 'priority')).toBe(true);
    const sim = simulateTPG(tpg, { A: 3 }); expect(sim.finished).toBe(true); expect(sim.collisions).toBe(0);
    // a plan edited on the fly: both agents claim the corridor in opposite order → cycle
    const bad = buildTPG({ A: ['a', 'c1', 'c2', 'b'], B: ['b', 'c2', 'c1', 'a'] }); expect(bad.acyclic).toBe(false);
    expect(criticalSections(g, agents)).toEqual(expect.arrayContaining(['c1', 'c2']));
    const grid: Graph = { nodes: [], edges: [] };
    for (let x = 0; x < 4; x++) for (let y = 0; y < 4; y++) { grid.nodes.push(`${x},${y}`); if (x) grid.edges.push([`${x - 1},${y}`, `${x},${y}`]); if (y) grid.edges.push([`${x},${y - 1}`, `${x},${y}`]); }
    const many = cbs(grid, [{ id: '1', start: '0,0', goal: '3,3' }, { id: '2', start: '3,3', goal: '0,0' }, { id: '3', start: '0,3', goal: '3,0' }, { id: '4', start: '3,0', goal: '0,3' }]);
    expect(many.found).toBe(true); expect(many.sumOfCosts).toBeGreaterThanOrEqual(24);
  });
  it('section reservation deadlock check', () => {
    expect(reservationDeadlockCheck({ r1: ['s1', 'corridor', 's2'], r2: ['s2', 'corridor', 's1'] }).deadlockFree).toBe(false);
    expect(reservationDeadlockCheck({ r1: ['s1', 'corridor', 's2'], r2: ['s1', 'corridor', 's2'] }).deadlockFree).toBe(true);
  });
});

describe('Scheduling (chapter 14)', () => {
  it('SPT / EDD / Moore / Johnson on the course exercise', () => {
    const jobs = [7, 3, 9, 2, 5, 4].map((p, i) => ({ id: `j${i + 1}`, p, d: [10, 8, 25, 6, 20, 14][i] }));
    const a = spt(jobs), b = edd(jobs), m = moore(jobs);
    expect(a.sumC).toBeLessThanOrEqual(b.sumC); expect(b.Lmax).toBeLessThanOrEqual(a.Lmax); expect(m.late).toBeLessThanOrEqual(Math.min(a.late, b.late));
    expect(a.order).toEqual(['j4', 'j2', 'j6', 'j5', 'j1', 'j3']);
    expect(johnson([{ id: 'a', p1: 3, p2: 6 }, { id: 'b', p1: 5, p2: 2 }, { id: 'c', p1: 1, p2: 2 }]).makespan).toBe(12);
  });
  it('cell job shop: LB 44 s, dispatch and branch & bound give the optimal 54 s, critical path through S1', () => {
    const shop = cellJobShop();
    expect(bestAssignmentBound(shop).lb).toBe(44);
    const lb = lowerBounds(shop); expect(lb.lb).toBeGreaterThanOrEqual(34);
    const rules = (['SPT', 'LPT', 'EDD', 'MWKR', 'FIFO', 'CR', 'ATC'] as const).map((r) => dispatch(shop, r));
    expect(Math.min(...rules.map((s) => s.makespan))).toBe(54);
    const bb = branchAndBound(shop); expect(bb.optimal).toBe(true); expect(bb.schedule.makespan).toBe(54);
    const cp = criticalPath(bb.schedule); expect(cp.length).toBeGreaterThanOrEqual(3); expect(cp.some((o) => o.machine === 'S1' || o.machine === 'S2')).toBe(true);
    const ts = tabuSearch(shop, { iterations: 50 }); expect(ts.schedule.makespan).toBe(54);
    // third machine: makespan drops
    const shop3: JobShop = { machines: [...shop.machines, 'S3'], jobs: shop.jobs.map((j) => ({ ...j, ops: j.ops.map((o, k) => (k === 1 ? { alternatives: [...o.alternatives, { machine: 'S3', p: o.alternatives[0].p }] } : o)) })) };
    expect(branchAndBound(shop3).schedule.makespan).toBeLessThan(54);
  });
  it('rescheduling: right shift keeps the order, deviation and Monte Carlo robustness are reported; OEE example 0.759', () => {
    const shop = cellJobShop(); const s = dispatch(shop, 'MWKR');
    const shifted = rightShift(shop, s, 'S1', 10, 6); expect(shifted.makespan).toBeGreaterThanOrEqual(s.makespan);
    const dev = scheduleDeviation(s, shifted); expect(dev.moved).toBeGreaterThan(0);
    const rob = scheduleRobustness(shop, s, 0.15, 30); expect(rob.mean).toBeGreaterThan(50); expect(rob.sd).toBeGreaterThan(0);
    const o = oee({ shiftMin: 480, plannedBreaksMin: 30, downtimeMin: 45, nominalCycleS: 22, produced: 950, scrap: 18 });
    expect(o.oee).toBeCloseTo(0.759, 2); expect(o.dominantLoss).toBe('performance');
    expect(classify('J3', ['prec'], 'Cmax').complexity).toMatch(/NP-hard/); expect(classify('1', [], 'Lmax').method).toMatch(/EDD/);
  });
});

describe('Real time (chapter 15)', () => {
  it('course task set: U = 0.80, response times 1.2 / 11.6 / 43.8 / 93.8 ms, schedulable beyond the Liu–Layland bound', () => {
    const three = responseTimes(COURSE_TASKS.slice(0, 3)); expect(three.U).toBeCloseTo(0.65, 6); expect(three.llSufficient).toBe(true); expect(liuLaylandBound(3)).toBeCloseTo(0.78, 2);
    const r = responseTimes(COURSE_TASKS); expect(r.U).toBeCloseTo(0.8, 6); expect(r.llSufficient).toBe(false); expect(r.schedulable).toBe(true);
    expect(r.results.map((x) => Number(x.R.toFixed(1)))).toEqual([1.2, 11.6, 43.8, 93.8]);
    expect(edfTest(COURSE_TASKS).schedulable).toBe(true);
    const over = responseTimes([...COURSE_TASKS, { id: 'extra', C: 40, T: 80 }]); expect(over.schedulable).toBe(false); expect(edfTest([...COURSE_TASKS, { id: 'extra', C: 40, T: 80 }]).schedulable).toBe(false);
  });
  it('blocking bounds and end-to-end latency (313 ms async, 155 ms sync)', () => {
    const tasks: RTTask[] = [{ id: 'H', C: 1, T: 10, criticalSections: { bus: 0.5 } }, { id: 'M', C: 2, T: 20 }, { id: 'L', C: 3, T: 50, criticalSections: { bus: 2, log: 4 } }];
    expect(blockingBounds(tasks, 'pcp').H).toBe(2); expect(blockingBounds(tasks, 'pip').H).toBe(2); expect(blockingBounds(tasks, 'pcp').L).toBe(0);
    const chain = [{ id: 'camera', T: 33, R: 33 }, { id: 'image', T: 50, R: 93.8 }, { id: 'pose', T: 50, R: 12 }, { id: 'planner', T: 20, R: 15 }, { id: 'drive', T: 5, R: 1.2 }];
    expect(endToEndLatency(chain, 'async').latency).toBeCloseTo(313, 0); expect(endToEndLatency(chain, 'sync').latency).toBeCloseTo(155, 0);
    const b = latencyBudget(0.313, 0.15, 0.015); expect(b.errorM).toBeCloseTo(0.047, 3); expect(b.ok).toBe(false); expect(b.maxSpeedMps).toBeCloseTo(0.048, 3);
    expect(wcetEstimate([10, 12, 11, 30], 'firm').budget).toBe(39);
    expect(qosRecommendation('sensor_stream', 30).reliability).toBe('BEST_EFFORT'); expect(qosRecommendation('command').reliability).toBe('RELIABLE');
    expect(placementAdvice({ name: 'estop', closedLoop: true, safety: true, rateHz: 1000, heavy: false }).level).toBe('drive');
    expect(placementAdvice({ name: 'schedule', closedLoop: false, safety: false, rateHz: 0.1, heavy: true }).level).toBe('plant');
  });
});

describe('Reliability and safety (chapter 16)', () => {
  it('cell budget: λ = 15.8e-4, MTTF 633 h, A = 0.9937, machine S 31.6 %', () => {
    const r = systemReliability(CELL_COMPONENTS, 4);
    expect(r.lambda).toBeCloseTo(15.8e-4, 8); expect(r.mttf).toBeCloseTo(633, 0); expect(r.availability).toBeCloseTo(0.9937, 4);
    expect(r.contributions[0].id).toBe('S'); expect(r.contributions[0].share).toBeCloseTo(0.316, 3);
    const red = systemReliability(CELL_COMPONENTS.map((c) => (c.id === 'controller' ? { ...c, redundancy: 2 } : c)), 4); expect(red.mttf / r.mttf - 1).toBeLessThan(0.03);
    expect(reliabilityWeibull(6000, 2.5, 3000)).toBeCloseTo(Math.exp(-Math.pow(0.5, 2.5)), 6);
    const rep = optimalReplacement(6000, 2.5, 1, 10); expect(rep.interval).toBeLessThan(6000); expect(rep.costRate).toBeLessThan(rep.runToFailureRate);
    const mk = markovAvailability({ degrade: 1 / 2000, fail: 1 / 500, repairFromDegraded: 1 / 8, repairFromFailed: 1 / 24 }); expect(mk.availability).toBeGreaterThan(0.99);
  });
  it('FTA: three cut sets of order 4, P ≈ 1.1e-12, B dominates; removing the hardware chain drops to order 3', () => {
    const cs = minimalCutSets(CONTACT_FTA); expect(cs.length).toBe(3); expect(cs.every((c) => c.length === 4)).toBe(true);
    const p = topEventProbability(CONTACT_FTA); expect(p.rareEvent).toBeCloseTo(1.1e-12, 13); expect(p.minOrder).toBe(4); expect(p.dominant).toContain('B');
    const sens = ftaSensitivity(CONTACT_FTA); expect(sens.find((x) => x.event === 'B')!.gain).toBeGreaterThan(0.8); expect(sens.find((x) => x.event === 'A')!.gain).toBeLessThan(0.01); expect(sens.some((x) => x.event === 'D')).toBe(false);
    const noHw = JSON.parse(JSON.stringify(CONTACT_FTA)); noHw.children[1].children[1] = { kind: 'event', id: 'E', p: 1e-3 };
    expect(topEventProbability(noHw).rareEvent).toBeCloseTo(1.1e-5, 6); expect(topEventProbability(noHw).minOrder).toBe(3);
    const rows = fmeaTable([{ element: 'encoder', failureMode: 'no signal', systemEffect: 'uncontrolled motion', detection: 'convergence check', S: 9, O: 2, D: 3, measure: 'cat.1 stop' }, { element: 'camera', failureMode: 'no frames', systemEffect: 'no recognition', detection: 'deadline', S: 4, O: 3, D: 2, measure: 'wait mode' }]);
    expect(rows[0].element).toBe('encoder'); expect(rows[0].mandatory).toBe(true); expect(actionPriority(1, 1, 1)).toBe('L');
  });
  it('ISO 13849 PLr / PL, ISO/TS 15066 SSM distance 1.54 m and 0.31 m/s in a narrow aisle, FDIR threshold', () => {
    expect(requiredPL(2, 2, 2)).toBe('e'); expect(requiredPL(2, 1, 2)).toBe('d'); expect(requiredPL(1, 1, 1)).toBe('a');
    expect(achievedPL('3', 'high', 'medium').pl).toBe('d'); expect(plMeets('d', 'd')).toBe(true); expect(plMeets('c', 'd')).toBe(false); expect(achievedPL('1', 'low', 'none').pl).toBeNull();
    expect(countsTowardsPL('cbf_filter')).toBe(false); expect(countsTowardsPL('safety_scanner')).toBe(true);
    const s = separationDistance({ vRobot: 1.2, tReaction: 0.15, tStop: 0.35, Zd: 0.1, Zr: 0.05 }); expect(s.Sp).toBeCloseTo(1.54, 2);
    expect(admissibleSpeed({ distance: 1.0, tReaction: 0.15, decel: 1.5, Zd: 0.1, Zr: 0.05 })).toBeCloseTo(0.311, 2);
    const d = detectionThreshold({ checkRateHz: 100, maxFalseAlarmsPerDay: 1 }); expect(d.kSigma).toBeCloseTo(5.3, 1); expect(d.confirmSteps3Sigma).toBe(3); expect(d.delayMs).toBe(30);
    expect(stopCategory({ carriesLoad: true, collaborative: false }).category).toBe(1);
  });
});

describe('V&V and acceptance (chapter 17)', () => {
  const sig = (dist: number[]): Signal => ({ t: dist.map((_, i) => i), values: dist.map((d, i) => ({ dist: d, human: i > 2 })) });
  it('STL robustness is the margin; falsification finds the violating scenario that random search misses', () => {
    const f = parseSTL('G[0,60] (human -> dist >= 0.8)');
    expect(formatSTL(f)).toBe('G[0,60] ((human -> dist >= 0.8))');
    expect(robustness(f, sig([5, 5, 5, 1.0, 1.2, 3]))).toBeCloseTo(0.2, 9);
    expect(robustness(f, sig([5, 5, 5, 0.5, 1.2, 3]))).toBeCloseTo(-0.3, 9);
    expect(robustness(parseSTL('F[0,3] dist <= 1'), sig([5, 3, 0.9, 5]))).toBeCloseTo(0.1, 9);
    expect(robustness(parseSTL('(dist >= 2) U[0,5] (dist <= 1)'), sig([5, 3, 0.9, 5]))).toBeCloseTo(0.1, 9);
    // scenario: robot drives along x at 1 m/s and turns at t = 30; a human walks from (px, py) with speed v at heading a
    const simulate = (p: Record<string, number>): Signal => { const t: number[] = [], values: Record<string, number | boolean>[] = []; for (let k = 0; k <= 60; k++) { const rx = k < 30 ? k : 30, ry = k < 30 ? 0 : k - 30; const hx = p.px + p.v * Math.cos(p.a) * k, hy = p.py + p.v * Math.sin(p.a) * k; const dist = Math.hypot(rx - hx, ry - hy); t.push(k); values.push({ dist, human: dist < 3 }); } return { t, values }; };
    const r = falsify(f, { px: [0, 40], py: [3, 8], v: [0.5, 1.6], a: [-Math.PI, Math.PI] }, simulate, { budget: 300, seed: 11 });
    expect(r.minRobustness).toBeLessThanOrEqual(r.randomBaseline); expect(r.evaluations).toBeLessThanOrEqual(300);
    expect(r.falsified).toBe(true);
  });
  it('pairwise covering array covers every pair with far fewer tests than the full factorial', () => {
    const factors = { object: ['bolt', 'nut', 'washer', 'other'], light: ['dim', 'normal', 'bright'], position: ['p1', 'p2', 'p3', 'p4', 'p5'], neighbour: ['no', 'yes'], gripper: ['empty', 'holding'] };
    const rows = pairwise(factors); expect(rows.length).toBeLessThanOrEqual(30); expect(rows.length).toBeGreaterThanOrEqual(20);
    const names = Object.keys(factors);
    for (let i = 0; i < names.length; i++) for (let j = i + 1; j < names.length; j++) for (const a of factors[names[i] as keyof typeof factors]) for (const b of factors[names[j] as keyof typeof factors]) expect(rows.some((r) => r[names[i]] === a && r[names[j]] === b), `${names[i]}=${a} ${names[j]}=${b}`).toBe(true);
  });
  it('rule of three, Clopper–Pearson (97/100 → 0.927 < 0.95, ≈250 trials needed), sim-real gap, acceptance bound', () => {
    expect(ruleOfThree(100)).toBeCloseTo(0.03, 9); expect(trialsForRuleOfThree(0.05)).toBe(60);
    expect(clopperPearsonLower(97, 100)).toBeCloseTo(0.927, 2); expect(clopperPearsonLower(100, 100)).toBeCloseTo(0.9704, 3);
    expect(trialsForTarget(0.95, 3)).toBe(154); const n = trialsForRate(0.95, 0.03); expect(n).toBeGreaterThan(200); expect(n).toBeLessThan(350); // course: "about 250"; the exact Clopper–Pearson count at 3 % failures is a little above 300
    let s = 3; const rnd = () => { s = (s * 1664525 + 1013904223) >>> 0; return s / 4294967296; }; const g = () => { let a = 0; for (let i = 0; i < 12; i++) a += rnd(); return a - 6; };
    const gap = simRealGap(Array.from({ length: 20 }, () => 52.3 + 1.1 * g()), Array.from({ length: 20 }, () => 58.7 + 3.4 * g()));
    expect(gap.significant).toBe(true); expect(gap.biasPercent).toBeGreaterThan(8); expect(gap.spreadRatio).toBeGreaterThan(1.5);
    const acc = acceptanceCheck(Array.from({ length: 20 }, () => 11.4 + 1.8 * g()), 15, 'max'); expect(acc.passMean).toBe(true); expect(acc.bound95).toBeGreaterThan(14);
    const reqs = [{ id: 'R1', text: 'arm still while base moves', cls: 'safety' as const, formal: 'G !(base_mv & arm_mv)', verification: 'model checking + monitor', result: 'pass' as const, components: ['supervisor', 'base'] }, { id: 'R4', text: 'mission in 15 min', cls: 'performance' as const, verification: '20 runs', result: 'open' as const, components: ['planner'] }];
    expect(impactOfChange(reqs, 'base').map((r) => r.id)).toEqual(['R1']); expect(traceabilitySummary(reqs).unformalised).toEqual(['R4']);
  });
});
