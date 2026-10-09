import { describe, it, expect } from 'vitest';
import { Rng, Vec2, dist } from '../src/mrs/rng';
import { completeGraph, pathGraph, ringGraph, laplacianSpectrum, vertexConnectivity, diameter, starGraph } from '../src/mrs/graph';
import { eventTriggeredConsensus, runConsensus, cooperativeTransport, potentialRepulsion, disagreement } from '../src/mrs/consensus';
import { acoTsp, fireflyOptimize, greyWolfOptimize, beeColonyOptimize, gaussianSource } from '../src/mrs/swarm';
import { vickreyAuction, contractNetMessages } from '../src/mrs/allocation';
import { Bimatrix, dominatedStrategies, pureNash, mixedNash2x2, paretoOptimal, bestResponseDynamics, fictitiousPlay, exactPotential, shapleyValue, inCore, isSuperadditive, replicatorDynamics } from '../src/mrs/games';
import { gridWorldFromMap, qLearning, optimalQ, greedyPath, independentQLearning } from '../src/mrs/learning';
import { geneticAlgorithm, rouletteTable, onePointCrossover, mutateBit, differentialEvolution, evolutionStrategy, BENCHMARKS } from '../src/mrs/evo';
import { wolframRun, wolframStats, lifeFromText, lifeRun, GLIDER, pheromoneStep } from '../src/mrs/ca';
import { fuzzyExample, fuzzyInfer, obstacleController, simulateFuzzyAvoidance } from '../src/mrs/fuzzy';
import { byzantineAgreement, byzantineBound, trustConsensus, degradation, commonLyapunov, dwellTimeSweep, switchedGrowth, expm2 } from '../src/mrs/resilience';

const close = (a: number, b: number, tol = 1e-9) => expect(Math.abs(a - b)).toBeLessThanOrEqual(tol);

describe('chapter 4 — topology comparison, event-triggered consensus, cooperative transport, potential fields', () => {
  it('table 4.1 and structural numbers', () => {
    expect(vertexConnectivity(starGraph(5))).toBe(1); expect(vertexConnectivity(ringGraph(5))).toBe(2); expect(vertexConnectivity(completeGraph(4))).toBe(3); expect(diameter(pathGraph(4))).toBe(3); expect(diameter(ringGraph(4))).toBe(2); expect(diameter(starGraph(4))).toBe(2); expect(diameter(completeGraph(4))).toBe(1);
    const lam = laplacianSpectrum(starGraph(5)); close(lam[1], 1, 1e-9); close(lam[4], 5, 1e-9);
  });
  it('event-triggered consensus converges with far fewer broadcasts than periodic messaging', () => {
    const A = ringGraph(8); const x0 = [3, -1, 4, 1, -5, 9, 2, -6]; const et = eventTriggeredConsensus(x0, A, 0.2, 300, 0.15, 0.01); const periodic = runConsensus(x0, A, 0.2, 300);
    expect(et.broadcasts).toBeLessThan(0.5 * et.periodicMessages); expect(disagreement(et.xs[300])).toBeLessThan(0.25); expect(disagreement(periodic[300])).toBeLessThan(1e-3);
  });
  it('cooperative transport: regimes of the PD law and convergence to the target', () => {
    const p0: Vec2[] = [[0, 0], [1, 0], [0.5, 1]]; const target: Vec2 = [5, 2];
    const o = cooperativeTransport(p0, target, { m: 1, kp: 4, kd: 2 }); expect(o.regime).toBe('oscillatory'); close(o.omega, Math.sqrt(16 - 4) / 2, 1e-12); expect(o.overshoot).toBeGreaterThan(0.05);
    const c = cooperativeTransport(p0, target, { m: 1, kp: 4, kd: 4 }); expect(c.regime).toBe('critical'); expect(c.overshoot).toBeLessThan(1e-3); expect(dist(c.xo[c.xo.length - 1], target)).toBeLessThan(0.02);
    expect(cooperativeTransport(p0, target, { m: 1, kp: 1, kd: 4 }).regime).toBe('overdamped');
  });
  it('potential fields push robots apart and away from obstacles', () => { const u = potentialRepulsion([[0, 0], [0.1, 0]], [[0, 0.1]], 0.01, 0.01, 5); expect(u[0][0]).toBeLessThan(0); expect(u[1][0]).toBeGreaterThan(0); expect(u[0][1]).toBeLessThan(0); });
});

describe('chapter 5 — ACO and the other metaheuristics', () => {
  it('ACO finds a tour at least as short as the nearest-neighbour heuristic on a 10-city ring', () => {
    const pts: Vec2[] = Array.from({ length: 10 }, (_, i) => [Math.cos((2 * Math.PI * i) / 10), Math.sin((2 * Math.PI * i) / 10)]); const shuffled = new Rng(3).shuffle(pts);
    const r = acoTsp(shuffled, new Rng(1), { iterations: 60 }); expect(r.tour.length).toBe(10); expect(new Set(r.tour).size).toBe(10); close(r.length, 10 * 2 * Math.sin(Math.PI / 10), 1e-6); expect(r.length).toBeLessThanOrEqual(r.greedyLength + 1e-9);
  });
  it('firefly, grey wolf and bee colony reach the Gaussian source', () => {
    const f = gaussianSource([0.7, -0.4], 0.5); const rng = new Rng(2); const x0 = (): Vec2[] => Array.from({ length: 15 }, () => [rng.uniform(-2, 2), rng.uniform(-2, 2)]);
    expect(dist(fireflyOptimize(f, x0(), 60, rng).best, [0.7, -0.4])).toBeLessThan(0.1); expect(dist(greyWolfOptimize(f, x0(), 80, rng).best, [0.7, -0.4])).toBeLessThan(0.1); expect(dist(beeColonyOptimize(f, x0(), 60, rng).best, [0.7, -0.4])).toBeLessThan(0.1);
  });
});

describe('chapter 9 — games', () => {
  const G: Bimatrix = { rows: ['Z1', 'Z2'], cols: ['Z1', 'Z2'], u: [[[4, 4], [10, 6]], [[6, 10], [2, 2]]] };
  it('§9.3.4 task-allocation game: two pure equilibria, mixed q = 0.8 with payoff 5.2; dominance, Pareto', () => {
    expect(pureNash(G)).toEqual([[0, 1], [1, 0]]); const m = mixedNash2x2(G)!; close(m.p, 0.8, 1e-9); close(m.q, 0.8, 1e-9); close(m.payoff[0], 5.2, 1e-9); close(m.payoff[1], 5.2, 1e-9);
    expect(dominatedStrategies(G)).toEqual({ row: [], col: [] }); expect(paretoOptimal(G)).toEqual([[0, 1], [1, 0]]);
    const pd: Bimatrix = { rows: ['C', 'D'], cols: ['C', 'D'], u: [[[3, 3], [0, 5]], [[5, 0], [1, 1]]] }; expect(dominatedStrategies(pd)).toEqual({ row: ['C'], col: ['C'] }); expect(pureNash(pd)).toEqual([[1, 1]]); expect(paretoOptimal(pd).some(([i, j]) => i === 0 && j === 0)).toBe(true);
  });
  it('best response converges in the coordination game and cycles in matching pennies; fictitious play frequencies; potential', () => {
    expect(bestResponseDynamics(G, [0, 0]).converged).toBe(true);
    const mp: Bimatrix = { rows: ['H', 'T'], cols: ['H', 'T'], u: [[[1, -1], [-1, 1]], [[-1, 1], [1, -1]]] }; expect(bestResponseDynamics(mp).converged).toBe(false); const fp = fictitiousPlay(mp, 2000); close(fp.rowFreq[0], 0.5, 0.05); close(fp.colFreq[0], 0.5, 0.05); expect(exactPotential(mp).isPotential).toBe(false);
    const cong: Bimatrix = { rows: ['A', 'B'], cols: ['A', 'B'], u: [[[-2, -2], [-1, -1]], [[-1, -1], [-2, -2]]] }; expect(exactPotential(cong).isPotential).toBe(true); expect(exactPotential(G).isPotential).toBe(true);
    const rd = replicatorDynamics([[3, 0], [5, 1]], [0.9, 0.1]); expect(rd[rd.length - 1][1]).toBeGreaterThan(0.99);
  });
  it('§9.5.3 Shapley value (26.67, 41.67, 51.67) lies in the core', () => {
    const v = { '0': 10, '1': 20, '2': 30, '0,1': 60, '0,2': 70, '1,2': 90, '0,1,2': 120 }; const phi = shapleyValue(v, 3); close(phi[0], 80 / 3, 1e-9); close(phi[1], 125 / 3, 1e-9); close(phi[2], 155 / 3, 1e-9);
    expect(inCore(v, 3, phi).inCore).toBe(true); expect(inCore(v, 3, [40, 40, 40]).blocking).toContain('1,2'); expect(isSuperadditive(v, 3)).toBe(true);
  });
  it('Vickrey auction: the winner pays the second-best bid; contract-net message count', () => {
    const a = vickreyAuction([[0, 0], [10, 0]], [[2, 0]], [20]); expect(a.assignment).toEqual({ 0: 0 }); close(a.payments[0], 12); expect(contractNetMessages(4, 6).total).toBe(24 + 24 + 6);
  });
});

describe('chapter 7 — Q-learning corridor (table 7.2) and independent multi-agent learning', () => {
  it('reproduces Q(s, right) after episodes 1–3 and the optimum 6.2 / 8 / 10', () => {
    const w = gridWorldFromMap('S..G'); const r = qLearning(w, { alpha: 0.5, gamma: 0.9, epsilon: 0, episodes: 3, forceAction: 1, snapshots: [1, 2, 3] });
    const q = (ep: number, s: number) => r.snapshots[ep - 1].Q[`0,${s}`][1];
    close(q(1, 0), -0.5); close(q(1, 1), -0.5); close(q(1, 2), 5); close(q(2, 0), -0.75); close(q(2, 1), 1.5); close(q(2, 2), 7.5); close(q(3, 0), -0.2); close(q(3, 1), 3.625); /* the course table prints 4.03; the update 1.5 + 0.5·(−1 + 0.9·7.5 − 1.5) gives 3.625 */ close(q(3, 2), 8.75);
    const opt = optimalQ(w, 0.9); close(opt['0,0'][1], 6.2, 1e-9); close(opt['0,1'][1], 8, 1e-9); close(opt['0,2'][1], 10, 1e-9);
    const long = qLearning(w, { alpha: 0.5, gamma: 0.9, epsilon: 0.1, episodes: 300, rng: new Rng(1) }); close(long.Q.get('0,0')![1], 6.2, 0.05); expect(greedyPath(w, long.Q).length).toBe(4);
  });
  it('IQL: two robots learn to reach their goals with fewer collisions', () => {
    const w = gridWorldFromMap('....\n....\n....'); const r = independentQLearning(w, [[0, 0], [2, 3]], [[2, 3], [0, 0]], { episodes: 600, rng: new Rng(2) });
    expect(r.collisionsLast).toBeLessThanOrEqual(r.collisionsFirst); expect(r.stepsLast).toBeLessThan(12);
  });
});

describe('chapter 12 — genetic algorithm example, DE and ES', () => {
  it('§12.2.6: roulette table, crossover offspring, mutation, generation statistics', () => {
    const pop = ['01101', '11000', '01000', '10011']; const f = (x: number) => x * x; const tbl = rouletteTable(pop, pop.map((b) => f(parseInt(b, 2))));
    close(tbl[0].f, 169); close(tbl[1].f, 576); close(tbl[0].p, 169 / 1170, 1e-12); close(tbl[1].expected, 1.97, 0.01); close(tbl[2].expected, 0.22, 0.01);
    expect(onePointCrossover('11000', '10011', 4)).toEqual(['11001', '10010']); expect(onePointCrossover('11000', '01101', 2)).toEqual(['11101', '01000']); expect(mutateBit('01000', 0)).toBe('11000');
    const gen1 = ['11001', '10010', '11101', '11000'].map((b) => f(parseInt(b, 2))); close(gen1.reduce((s, v) => s + v, 0), 2366); close(gen1.reduce((s, v) => s + v, 0) / 4, 591.5);
    const ga = geneticAlgorithm(f, { bits: 5, initial: pop, generations: 30, pm: 0.05, elitism: true, rng: new Rng(4) }); expect(ga.bestX).toBe(31); expect(ga.history[0].mean).toBe(292.5);
  });
  it('DE and ES minimise the benchmarks', () => {
    const de = differentialEvolution(BENCHMARKS.rastrigin.f, [-5.12, -5.12], [5.12, 5.12], { generations: 150, rng: new Rng(5) }); expect(de.value).toBeLessThan(0.05);
    const es = evolutionStrategy(BENCHMARKS.sphere.f, [3, -2], { generations: 80, rng: new Rng(6) }); expect(es.value).toBeLessThan(1e-3); expect(es.sigmaHistory[es.sigmaHistory.length - 1]).toBeLessThan(es.sigmaHistory[0]);
  });
});

describe('§4.6 — cellular automata; §13.4 — fuzzy control; chapter 16 / §13.5 — resilience and switching', () => {
  it('rule 90 is symmetric (Sierpiński), rule 30 is not; the glider has period 4 and shift (1, 1); pheromone evaporates', () => {
    const s90 = wolframStats(wolframRun(90, 31, 15)); expect(s90.symmetric).toBe(true); const s30 = wolframStats(wolframRun(30, 31, 15)); expect(s30.symmetric).toBe(false); expect(s30.distinctRows).toBe(16);
    expect(wolframRun(90, 7, 2)[2]).toEqual([0, 1, 0, 0, 0, 1, 0]);
    const g = lifeFromText(GLIDER); const grid = Array.from({ length: 10 }, () => Array(10).fill(0)); g.forEach((r, i) => r.forEach((v, j) => { grid[i][j] = v; })); const life = lifeRun(grid, 12); expect(life.period).toBe(4); expect(life.shift).toEqual([1, 1]); expect(life.population.every((p) => p === 5)).toBe(true);
    const ph = pheromoneStep([[0, 0, 0], [0, 0, 0], [0, 0, 0]], [[1, 1]]); expect(ph[1][1]).toBeGreaterThan(ph[0][1]); expect(ph[0][1]).toBeGreaterThan(0); expect(ph.flat().reduce((s, v) => s + v, 0)).toBeLessThan(1);
  });
  it('§13.4.2 example: rule 5 fires at 0.4, rule 2 at 0.3, the output is a compromise (slow-ish, turning left)', () => {
    const e = fuzzyExample(); const r5 = e.active.find((a) => a.rule === 5)!, r2 = e.active.find((a) => a.rule === 2)!; close(r5.strength, 0.4, 1e-9); close(r2.strength, 0.3, 1e-9);
    close(e.memberships.dL.far, 1); close(e.memberships.dF.near, 0.4, 1e-9); close(e.memberships.dF.medium, 0.3, 1e-9); close(e.memberships.dR.medium, 0.2, 1e-9); close(e.memberships.dR.far, 0.6667, 1e-3);
    expect(e.outputs.v).toBeGreaterThan(0.25); expect(e.outputs.v).toBeLessThan(0.5); expect(e.outputs.omega).toBeGreaterThan(0.3);
    const freeRun = fuzzyInfer(obstacleController(), { dL: 3, dF: 3, dR: 3 }).outputs; expect(freeRun.v).toBeGreaterThan(0.8); close(freeRun.omega, 0, 1e-6);
    const sim = simulateFuzzyAvoidance([[3, 0], [3, 0.4], [3, -0.4]], [6, 0]); expect(sim.reachedGoal).toBe(true); expect(sim.minObstacleDistance).toBeGreaterThan(0.4);
  });
  it('§16.8.2: four agents, one Byzantine — honest agents agree; n ≥ 3f + 1; trust isolates a liar; degradation; switching', () => {
    const r = byzantineAgreement([{ name: 'A', value: 'continue' }, { name: 'B', value: 'continue' }, { name: 'C', value: 'continue' }, { name: 'D', value: 'continue', lies: { A: 'continue', B: 'evacuate', C: 'evacuate' } }]);
    expect(r.agreementAmongHonest).toBe(true); expect(r.honestDecision).toBe('continue');
    const three = byzantineAgreement([{ name: 'A', value: 'continue' }, { name: 'B', value: 'evacuate' }, { name: 'C', value: 'continue', lies: { A: 'continue', B: 'evacuate' } }]); expect(three.ambiguous).toBeGreaterThan(0); expect(r.ambiguous).toBe(0); expect(byzantineBound(4, 1).tolerable).toBe(true); expect(byzantineBound(3, 1).tolerable).toBe(false); expect(byzantineBound(4, 1).messagesOM).toBe(6);
    const A = completeGraph(6); const tc = trustConsensus([0.1, 0.4, 0.2, 0.9, 0.5, 0], A, 0.1, 200, { 5: (k) => (k % 2 ? 100 : -100) }); const last = tc.xs[200].slice(0, 5); expect(Math.max(...last) - Math.min(...last)).toBeLessThan(0.05); expect(Math.max(...last)).toBeLessThan(1); expect(tc.finalTrustOfLiars).toBeLessThan(0.05);
    const deg = degradation(starGraph(5), [1, 0]); expect(deg[1].connected).toBe(true); expect(deg[2].connected).toBe(false);
    const A1: [[number, number], [number, number]] = [[-0.1, 1], [-10, -0.1]], A2: [[number, number], [number, number]] = [[-0.1, 10], [-1, -0.1]]; // two stable modes that destabilise under fast switching
    expect(commonLyapunov([A1, A2]).exists).toBe(false); const sw = dwellTimeSweep([A1, A2], 0.05, 3, 60); expect(sw.unstableFor.length).toBeGreaterThan(0); expect(sw.dwellTime).not.toBeNull(); expect(switchedGrowth([A1, A2], 0.3).stable).toBe(false); expect(switchedGrowth([A1, A2], 3).stable).toBe(true);
    const S: [[number, number], [number, number]] = [[-1, 0], [0, -2]]; expect(commonLyapunov([S, [[-2, 0.5], [0.5, -1]]]).exists).toBe(true); close(expm2(S, 1)[0][0], Math.exp(-1), 1e-9);
  });
});
