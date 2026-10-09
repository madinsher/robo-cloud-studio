import { describe, it, expect } from 'vitest';
import { parseDomain, parseProblem, ground, plan, validatePlan, relaxedHeuristic, htnPlan, solveSTN, checkSTNU, scheduleGantt, executeWithReplanning, findAction, planText } from '../src/ctl/planning';
import { YOUBOT_PDDL_DOMAIN, YOUBOT_PDDL_PROBLEM, YOUBOT_HTN, CELL_PDDL_DOMAIN, CELL_PDDL_PROBLEM } from '../src/ctl/examples';
import { buildReachMap, reachIndexAt, basePlacements, blockingSet, rearrangementPlan, tampSolve, graspSuccessRate, allowedPoseError, planRobustness, TableScene } from '../src/ctl/tamp';
import { valueIteration, policyIteration, graspStrategyMDP, sensitivityScan, switchPoints, classificationPOMDP, beliefUpdate, expectedReward, lookaheadValue, bestTerminalAction, qmdp, alphaVectorBackup, decisionThreshold, Shield } from '../src/ctl/mdp';

describe('Task planning (chapter 10)', () => {
  const d = parseDomain(YOUBOT_PDDL_DOMAIN), p = parseProblem(YOUBOT_PDDL_PROBLEM);
  it('parses and grounds the course domain; GBFS/h_FF finds a valid plan, A*/h_max an optimal one', () => {
    const t = ground(d, p);
    expect(t.actions.length).toBeGreaterThan(20); expect(t.atoms.length).toBeGreaterThan(20);
    const g = plan(t, { search: 'gbfs', heuristic: 'hff' });
    expect(g.found).toBe(true); expect(validatePlan(t, g.plan).valid).toBe(true);
    expect(g.plan.filter((a) => a.schema.name === 'observe').length).toBe(3); // objects must be detected first
    const a = plan(t, { search: 'astar', heuristic: 'hmax', maxExpansions: 100000 });
    expect(a.found).toBe(true); expect(a.cost).toBeLessThanOrEqual(g.cost);
    expect(a.cost).toBe(12 + 9 + 3 + 8 + 9 + 7 + 6 + 7 + 9 + 3 + 8 + 9 + 7 + 6 + 7 + 11 + 3 + 8 + 11 + 7 + 6 + 7 + 12);
    expect(planText(a)).toMatch(/cost/);
    const h = relaxedHeuristic(t, t.init, 'hadd'); const hm = relaxedHeuristic(t, t.init, 'hmax');
    expect(h.h).toBeGreaterThan(hm.h); // h_add over-estimates shared subgoals, h_max under-estimates
  });
  it('HTN decomposition delivers an undetected object via the search method (recursion)', () => {
    const t = ground(d, p);
    const r = htnPlan(t, YOUBOT_HTN, [{ name: 'deliver', args: ['bolt1', 'bin-bolts'] }]);
    expect(r.found).toBe(true);
    expect(r.plan.map((a) => a.schema.name)).toEqual(['move', 'move', 'observe', 'pick', 'move', 'move', 'place']);
    expect(validatePlan({ ...t, goalPos: [t.atomIndex.get('in-bin(bolt1,bin-bolts)')!], goalNeg: [] }, r.plan).valid).toBe(true);
    expect(r.trace.some((l) => /M3/.test(l))).toBe(true);
  });
  it('durative cell domain plans and schedules a Gantt with the exclusive zone', () => {
    const cd = parseDomain(CELL_PDDL_DOMAIN), cp = parseProblem(CELL_PDDL_PROBLEM(2));
    const t = ground(cd, cp);
    const r = plan(t, { search: 'gbfs', heuristic: 'hff' });
    expect(r.found).toBe(true); expect(validatePlan(t, r.plan).valid).toBe(true);
    const g = scheduleGantt(r.plan);
    expect(g.makespan).toBeGreaterThanOrEqual(4 + 5 + 22 + 5 + 4);
    const loads = g.rows.filter((x) => /^(load|unload)/.test(x.action));
    for (const a of loads) for (const b of loads) if (a !== b) expect(a.start >= b.end || b.start >= a.end).toBe(true); // zone exclusive
  });
  it('STN consistency / bounds and STNU dynamic controllability (course §10.7.3)', () => {
    const stn = solveSTN([{ from: 'z', to: 'a', min: 0, max: 10 }, { from: 'a', to: 'b', min: 5, max: 8 }, { from: 'z', to: 'b', min: 0, max: 12 }]);
    expect(stn.consistent).toBe(true); expect(stn.latest.a).toBe(7); expect(stn.earliest.b).toBe(5);
    expect(solveSTN([{ from: 'z', to: 'a', min: 5, max: 6 }, { from: 'a', to: 'z', min: 0, max: 0 }]).consistent).toBe(false);
    const base = [{ from: 'z', to: 'start', min: 20, max: 20 }, { from: 'z', to: 'free', min: 30, max: 40, contingent: true }, { from: 'free', to: 'unload', min: 0, max: 5 }, { from: 'arrive', to: 'unload', min: 0, max: 100 }];
    const ok = checkSTNU([...base, { from: 'start', to: 'arrive', min: 8, max: 14, contingent: true }]);
    expect(ok.controllable).toBe(true);
    const bad = checkSTNU([...base, { from: 'start', to: 'arrive', min: 8, max: 16, contingent: true }]);
    expect(bad.controllable).toBe(false);
    const early = checkSTNU([{ from: 'z', to: 'start', min: 0, max: 20 }, ...base.slice(1), { from: 'start', to: 'arrive', min: 8, max: 16, contingent: true }]);
    expect(early.controllable).toBe(true); // leaving earlier restores controllability
  });
  it('plan–execute–replan recovers from an unreachable object', async () => {
    const t = ground(d, p);
    let failedOnce = false; const replans: string[] = [];
    const r = await executeWithReplanning(t, {
      execute: (a) => { if (a.name === 'pick bolt1 table1' && !failedOnce) { failedOnce = true; return false; } return true; },
      observe: (s) => { if (failedOnce && s.has(t.atomIndex.get('on(bolt2,table1)')!)) { const n = new Set(s); n.add(t.atomIndex.get('blocked(bolt1,bolt2)')!); return n; } return s; },
      onReplan: (why) => replans.push(why),
    });
    expect(r.success).toBe(true); expect(r.replans).toBe(1); expect(replans[0]).toMatch(/pick bolt1/);
    expect(r.executed.some((a) => a.name === 'pick bolt2 table1')).toBe(true);
    expect(findAction(t, 'move base corridor')).toBeDefined();
  });
});

describe('TAMP (chapter 11)', () => {
  it('reachability map and inverse base placement', () => {
    const map = buildReachMap({ links: [0.3, 0.25], limits: [[-2.5, 2.5], [-2.8, 2.8]] }, 0.05, 30000);
    expect(reachIndexAt(map, { x: 0.4, y: 0 })).toBeGreaterThan(0.1);
    expect(reachIndexAt(map, { x: 0.7, y: 0 })).toBe(0);
    const cands = basePlacements(map, { x: 2, y: 1 }, (b) => b.y < 1, { x: 0, y: 0 });
    expect(cands.length).toBeGreaterThan(0); expect(cands[0].base.y).toBeLessThan(1);
    expect(Math.hypot(cands[0].base.x - 2, cands[0].base.y - 1)).toBeLessThan(0.56);
  });
  it('blocking set, rearrangement with depth bound and robustness', () => {
    const scene: TableScene = { xMin: 0, xMax: 0.8, yMin: 0, yMax: 0.5, approachHalfWidth: 0.03, objects: [{ id: 'bolt1', x: 0.4, y: 0.25, r: 0.02 }, { id: 'nut2', x: 0.44, y: 0.26, r: 0.015 }, { id: 'nut3', x: 0.37, y: 0.22, r: 0.015 }, { id: 'far', x: 0.1, y: 0.1, r: 0.02 }] };
    expect(blockingSet(scene, 'bolt1').sort()).toEqual(['nut2', 'nut3']);
    const r = rearrangementPlan(scene, 'bolt1');
    expect(r).not.toBeNull(); expect(r!.moves).toBe(2); expect(r!.steps[r!.steps.length - 1]).toEqual({ kind: 'pick', object: 'bolt1' });
    const rob = planRobustness(scene, 'bolt1', 0.002, 20); expect(rob.successRate).toBeGreaterThan(0.5);
    expect(graspSuccessRate(5, { x: 10, y: 10 })).toBeGreaterThan(0.9); expect(graspSuccessRate(15, { x: 10, y: 10 })).toBeLessThan(0.5);
    const allowed = allowedPoseError({ x: 10, y: 10 }, 0.9); expect(allowed).toBeGreaterThan(3); expect(allowed).toBeLessThan(8);
  });
  it('skeleton + parameters loop feeds geometric failures back as (blocked …) facts', () => {
    const d = parseDomain(YOUBOT_PDDL_DOMAIN), p = parseProblem(YOUBOT_PDDL_PROBLEM);
    const scene: TableScene = { xMin: 0, xMax: 0.8, yMin: 0, yMax: 0.5, approachHalfWidth: 0.03, approach: { x: -1, y: 0 }, objects: [{ id: 'bolt1', x: 0.4, y: 0.25, r: 0.02 }, { id: 'bolt2', x: 0.44, y: 0.26, r: 0.015 }] }; // side grasp from +x: bolt2 sits in front of bolt1
    let checks = 0;
    const r = tampSolve(d, p, (a, state, task) => { if (a.schema.name !== 'pick') return null; checks++; const onTable = (id: string) => { const i = task.atomIndex.get(`on(${id},table1)`); return i !== undefined && state.has(i); }; const blockers = blockingSet(scene, a.args[0]).filter(onTable); return blockers.length ? blockers.map((b) => ({ name: 'blocked', args: [a.args[0], b] })) : null; });
    expect(r.found).toBe(true); expect(r.skeletons).toBe(2); expect(r.feedback[0]).toMatch(/pick bolt1 table1 infeasible → \(blocked bolt1 bolt2\)/);
    const order = r.plan.filter((a) => a.schema.name === 'pick').map((a) => a.args[0]);
    expect(order.indexOf('bolt2')).toBeLessThan(order.indexOf('bolt1')); // right order instead of a rearrangement
    expect(checks).toBeGreaterThan(0);
  });
});

describe('MDP / POMDP (chapter 12)', () => {
  it('grasp strategy MDP: refine first (12.943 s vs 15.287 s), switch point near p = 0.70', () => {
    const m = graspStrategyMDP();
    const vi = valueIteration(m), pi = policyIteration(m);
    expect(vi.values.s0).toBeCloseTo(12.943, 2); expect(vi.values.s1).toBeCloseTo(9.943, 2); expect(vi.values.s3).toBeCloseTo(16.193, 2);
    expect(vi.policy.s0).toBe('refine'); expect(pi.policy.s0).toBe('refine'); expect(pi.values.s0).toBeCloseTo(vi.values.s0, 4);
    expect(vi.qValues.s0.grasp).toBeCloseTo(15.287, 2);
    const scan = sensitivityScan((p) => graspStrategyMDP(p), 's0', [0.5, 0.9], 40);
    const sw = switchPoints(scan); expect(sw.length).toBe(1); expect(sw[0].p).toBeGreaterThan(0.68); expect(sw[0].p).toBeLessThan(0.72); expect(sw[0].to).toBe('grasp');
    const tScan = switchPoints(sensitivityScan((tr) => graspStrategyMDP(0.55, tr), 's0', [1, 8], 70)); expect(tScan[0].p).toBeGreaterThan(5.2); expect(tScan[0].p).toBeLessThan(5.5);
  });
  it('classification POMDP reproduces the belief update, the value of looking closer and the 0.943 threshold', () => {
    const m = classificationPOMDP();
    const b0 = { bolt: 0.4, nut: 0.4, other: 0.2 };
    const u = beliefUpdate(m, b0, 'look', 'obs_bolt');
    expect(u.pObs).toBeCloseTo(0.39, 3); expect(u.belief.bolt).toBeCloseTo(0.718, 3); expect(u.belief.nut).toBeCloseTo(0.205, 3);
    expect(expectedReward(m, u.belief, 'place_bolt')).toBeCloseTo(0.26, 1);
    const la = lookaheadValue(m, u.belief, 'look_closer', (b2) => bestTerminalAction(m, b2));
    expect(la.value).toBeCloseTo(11.07, 1);
    expect(la.branches.find((x) => x.o === 'obs_bolt')!.p).toBeCloseTo(0.685, 2);
    expect(decisionThreshold(20, 50, 4)).toBeCloseTo(0.943, 3); expect(decisionThreshold(20, 10, 4)).toBeCloseTo(0.867, 3); expect(decisionThreshold(20, 200, 4)).toBeCloseTo(0.982, 3);
    // QMDP never looks closer; α-vector backup with horizon 2 does
    expect(qmdp(m, u.belief).action).not.toBe('look_closer');
    const av = alphaVectorBackup(m, 2);
    expect(av.value(u.belief).action).toBe('look_closer');
    expect(av.value({ bolt: 0.99, nut: 0.005, other: 0.005 }).action).toBe('place_bolt');
  });
  it('shield overrides unsafe actions of a policy', () => {
    const sh = new Shield<string, string>((s) => (s === 'near_human' ? ['stop', 'slow'] : ['stop', 'slow', 'fast']), () => 'stop');
    expect(sh.apply('free', 'fast')).toBe('fast'); expect(sh.apply('near_human', 'fast')).toBe('stop'); expect(sh.overrides).toBe(1);
  });
});
