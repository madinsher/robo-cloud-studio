import { describe, it, expect } from 'vitest';
import { Rng, Vec2, dist } from '../src/mrs/rng';
import { pathGraph, ringGraph, starGraph, completeGraph, randomGeometricGraph, laplacian, laplacianSpectrum, algebraicConnectivity, maxConsensusStep, diskGraph, isConnected, metropolisWeights, graphFromEdges, robustness } from '../src/mrs/graph';
import { consensusStep, runConsensus, disagreement, convergenceFactor, formationVelocity, formationError, connectivityWeight, rendezvousVelocity, wmsrStep } from '../src/mrs/consensus';
import { simulate } from '../src/mrs/sim';
import { boidsAcceleration, boidsStep, polarization, vicsekStep, vicsekOrder, ringNeighborhood, psoVelocity, psoOptimize, repulsion, robotPsoSearch, gaussianSource, minPairDistance } from '../src/mrs/swarm';
import { greedyAssignment, optimalAssignment, bestInsertion, ssiAuction, routeLength, totalLength, bruteForceRouting, CBBAAgent, cbbaBuildBundle, cbbaConsensus, runCbba, isConflictFree } from '../src/mrs/allocation';
import { loadMap, spaceTimeAStar, prioritizedPlanning, firstConflict, cbs, sumOfCosts, actionDependencies, executeAdg, successors, freeCells, WAREHOUSE_MAP, Cell } from '../src/mrs/mapf';
import { gridPoints, voronoiLabels, massCentroids, coverageCost, lloydStep, gaussianDensity, limitedLloydStep, limitedCost, informationConsensus, centralizedEstimate, covarianceIntersection, inv2, add2, eig2, M2 } from '../src/mrs/coverage';
import { pairConstraint, safeVelocity, speedPolygon, goToGoal, unstuckNominal, rotate, staleConstraint, antipodal, runGroup, minPairDistance as minDist } from '../src/mrs/safety';

const close = (a: number, b: number, tol = 1e-9) => expect(Math.abs(a - b)).toBeLessThanOrEqual(tol);

describe('ПР1 — consensus, formation, connectivity, W-MSR (course acceptance tests)', () => {
  it('Laplacian of P3, zero row sums, λ₂ of K6 and C8, λ₂ = 0 iff disconnected', () => {
    expect(laplacian(pathGraph(3))).toEqual([[1, -1, 0], [-1, 2, -1], [0, -1, 1]]);
    const { A } = randomGeometricGraph(12, 0.45, new Rng(1)); for (const r of laplacian(A)) close(r.reduce((s, v) => s + v, 0), 0, 1e-12);
    close(algebraicConnectivity(completeGraph(6)), 6, 1e-9); close(algebraicConnectivity(ringGraph(8)), 2 - 2 * Math.cos((2 * Math.PI) / 8), 1e-9);
    const B = graphFromEdges(4, [[0, 1], [2, 3]]); close(algebraicConnectivity(B), 0, 1e-9); B[1][2] = B[2][1] = 1; expect(algebraicConnectivity(B)).toBeGreaterThan(1e-6);
  });
  it('§4.1.5 example: spectrum {0, 1, 3, 4}; table 4.1 λ₂ for n = 4', () => {
    const A = graphFromEdges(4, [[0, 1], [1, 2], [1, 3], [2, 3]]); const lam = laplacianSpectrum(A);
    [0, 1, 3, 4].forEach((v, i) => close(lam[i], v, 1e-9));
    close(algebraicConnectivity(pathGraph(4)), 2 - Math.SQRT2, 1e-9); close(algebraicConnectivity(ringGraph(4)), 2, 1e-9); close(algebraicConnectivity(starGraph(4)), 1, 1e-9); close(algebraicConnectivity(completeGraph(4)), 4, 1e-9);
  });
  it('max step of a star, convergence to the average with mean preservation, vector states, λ₂ rate bound', () => {
    close(maxConsensusStep(starGraph(5)), 0.25);
    const rng = new Rng(2); const { A } = randomGeometricGraph(10, 0.5, rng); const x0 = Array.from({ length: 10 }, () => rng.normal()); const xs = runConsensus(x0, A, 0.9 * maxConsensusStep(A), 400); const mean = x0.reduce((s, v) => s + v, 0) / 10;
    for (const v of xs[xs.length - 1]) close(v, mean, 1e-6); for (const x of xs) close(x.reduce((s, v) => s + v, 0) / 10, mean, 1e-10);
    const xv: Vec2[] = [[0, 1], [2, 3], [4, 5], [6, 7], [8, 9]]; const x1 = consensusStep(xv, ringGraph(5), 0.3); close(x1.reduce((s, v) => s + v[0], 0) / 5, 4); close(x1.reduce((s, v) => s + v[1], 0) / 5, 5);
    const P = pathGraph(6); const lam2 = algebraicConnectivity(P); const y0 = [1, -1, 2, 0.5, -2, 3]; const ys = runConsensus(y0, P, 0.4, 60); expect(disagreement(ys[60])).toBeLessThanOrEqual((1 - 0.4 * lam2) ** 60 * disagreement(y0) * (1 + 1e-9));
  });
  it('§4.3.4 example: P4, ε = 0.25, x(0) = (0,4,8,12) → the three iterations of the text, ρ = 0.854, ≈ 29 iterations for 1 %', () => {
    const xs = runConsensus([0, 4, 8, 12], pathGraph(4), 0.25, 3);
    expect(xs[1]).toEqual([1, 4, 8, 11]); expect(xs[2]).toEqual([1.75, 4.25, 7.75, 10.25]); expect(xs[3]).toEqual([2.375, 4.5, 7.5, 9.625]);
    const c = convergenceFactor(pathGraph(4), 0.25); close(c.rho, 0.8536, 1e-3); expect(c.iterationsFor1pc).toBe(30); close(c.epsOpt, 2 / (2 - Math.SQRT2 + 2 + Math.SQRT2), 1e-12);
  });
  it('formation assembles on a ring and keeps the centre of ξ', () => {
    const rng = new Rng(3); const n = 6; const offsets: Vec2[] = Array.from({ length: n }, (_, i) => [0.3 * Math.cos((2 * Math.PI * i) / n), 0.3 * Math.sin((2 * Math.PI * i) / n)]); const p0: Vec2[] = Array.from({ length: n }, () => [rng.uniform(), rng.uniform()]); const A = ringGraph(n);
    const { hist } = simulate(p0, (_t, p) => formationVelocity(p, A, offsets, 1), 600, { dt: 0.05 }); const pf = hist.p[hist.p.length - 1];
    expect(formationError(pf, offsets)).toBeLessThan(1e-3);
    const c0 = [0, 1].map((k) => p0.reduce((s, q, i) => s + q[k] - offsets[i][k], 0) / n); const cf = [0, 1].map((k) => pf.reduce((s, q, i) => s + q[k] - offsets[i][k], 0) / n); close(c0[0], cf[0], 1e-6); close(c0[1], cf[1], 1e-6);
  });
  it('connectivity weight and rendezvous that never breaks an initial edge', () => {
    expect(connectivityWeight(1, 1)).toBe(0); close(connectivityWeight(0, 1), 2); expect(connectivityWeight(0.99, 1)).toBeGreaterThan(1e3);
    const R = 1; const p0: Vec2[] = [[0, 0], [0.95, 0], [1.9, 0], [2.85, 0], [2.85, 0.95]]; const A0 = diskGraph(p0, R); expect(isConnected(A0)).toBe(true);
    const { hist } = simulate(p0, (_t, p) => rendezvousVelocity(p, A0, R, 0.5), 5000, { dt: 0.002, vmax: 0.5, recordEvery: 5 });
    for (const P of hist.p) for (let i = 0; i < 5; i++) for (let j = i + 1; j < 5; j++) if (A0[i][j]) expect(dist(P[i], P[j])).toBeLessThan(R);
    const pf = hist.p[hist.p.length - 1]; const c: Vec2 = [pf.reduce((s, q) => s + q[0], 0) / 5, pf.reduce((s, q) => s + q[1], 0) / 5]; expect(Math.max(...pf.map((q) => dist(q, c)))).toBeLessThan(0.1);
  });
  it('W-MSR filters one malicious agent on K7 (3-robust) while plain consensus is hijacked', () => {
    const A = completeGraph(7); let x = [0.1, 0.4, 0.2, 0.9, 0.5, 0.3, 0]; const normal = [0, 1, 2, 3, 4, 5]; const lo = 0, hi = 0.9;
    for (let k = 0; k < 300; k++) { x[6] = k % 2 ? 100 : -100; x = wmsrStep(x, A, 1, 0.1, [6]); }
    const vals = normal.map((i) => x[i]); expect(Math.max(...vals) - Math.min(...vals)).toBeLessThan(1e-3); const m = vals.reduce((s, v) => s + v, 0) / 6; expect(m).toBeGreaterThanOrEqual(lo - 1e-9); expect(m).toBeLessThanOrEqual(hi + 1e-9);
    let y = [0.1, 0.4, 0.2, 0.9, 0.5, 0.3, 0]; for (let k = 0; k < 300; k++) { y[6] = 100; y = consensusStep(y, A, 0.1); } expect(y.slice(0, 6).reduce((s, v) => s + v, 0) / 6).toBeGreaterThan(50);
    expect(robustness(completeGraph(7))).toBeGreaterThanOrEqual(3); expect(robustness(pathGraph(5))).toBe(1); expect(robustness(metropolisWeights(ringGraph(4)).map((r) => r.map((v) => (v > 0 ? 1 : 0))))).toBeGreaterThanOrEqual(1);
  });
});

describe('ПР2 — boids, Vicsek, PSO, robot swarm search', () => {
  it('Reynolds rules: isolated robot, the three components, a flock aligns without collisions', () => {
    expect(boidsAcceleration([[0, 0], [5, 5]], [[1, 0], [0, 1]], 0, { rSep: 0.2, rView: 1 })).toEqual([0, 0]);
    const p: Vec2[] = [[0, 0], [0.1, 0]], v: Vec2[] = [[0, 0], [1, 0]];
    const sep = boidsAcceleration(p, v, 0, { rSep: 0.5, rView: 1, wSep: 1, wAli: 0, wCoh: 0 }); close(sep[0], -10, 1e-9); close(sep[1], 0);
    const ali = boidsAcceleration(p, v, 0, { rSep: 0.5, rView: 1, wSep: 0, wAli: 1, wCoh: 0 }); close(ali[0], 1); const coh = boidsAcceleration(p, v, 0, { rSep: 0.5, rView: 1, wSep: 0, wAli: 0, wCoh: 1 }); close(coh[0], 0.1);
    const rng = new Rng(4); let P: Vec2[] = Array.from({ length: 20 }, () => [rng.uniform(), rng.uniform()]); let V: Vec2[] = Array.from({ length: 20 }, () => [rng.normal(0, 0.1), rng.normal(0, 0.1)]);
    for (let k = 0; k < 800; k++) ({ p: P, v: V } = boidsStep(P, V, 0.05, 0.2, 0.3, { rSep: 0.15, rView: 0.6 }));
    expect(polarization(V)).toBeGreaterThan(0.9); expect(minPairDistance(P)).toBeGreaterThan(0.1);
  });
  it('polarization, torus neighbours and the Vicsek phase transition', () => {
    close(polarization([[1, 0], [2, 0]]), 1); close(polarization([[1, 0], [-1, 0]]), 0); expect(polarization([[0, 0], [0, 0]])).toBe(0);
    const r = vicsekStep([[0.05, 0.5], [0.95, 0.5]], [0, Math.PI / 2], 0, 0.2, 0, 1, new Rng(0)); close(r.theta[0], Math.PI / 4, 1e-12); close(r.theta[1], Math.PI / 4, 1e-12);
    const rng = new Rng(5); const low = vicsekOrder(100, 5, 1, 0.3, 300, rng); const high = vicsekOrder(100, 5, 1, 6, 300, rng); expect(low).toBeGreaterThan(0.8); expect(high).toBeLessThan(0.3);
  });
  it('PSO velocity formula, vmax, finds the maximum; §5.6.4 one-dimensional example', () => {
    const x: Vec2[] = [[0, 0], [1, 0], [0, 1]], v: Vec2[] = [[0.1, 0], [0, 0], [0, 0]], pb: Vec2[] = [[0.5, 0], [1, 1], [0, 1]], pv = [1, 3, 2];
    const got = psoVelocity(x, v, pb, pv, ringNeighborhood(3, 1), new Rng(7), { w: 0.5, c1: 1, c2: 1 }); const r = new Rng(7); const r1 = x.map(() => [r.random(), r.random()]), r2 = x.map(() => [r.random(), r.random()]);
    for (let i = 0; i < 3; i++) for (let c = 0; c < 2; c++) close(got[i][c], 0.5 * v[i][c] + r1[i][c] * (pb[i][c] - x[i][c]) + r2[i][c] * (1 - x[i][c]), 1e-12);
    const vv = psoVelocity([[0, 0], [0, 0], [0, 0], [0, 0]], [[0, 0], [0, 0], [0, 0], [0, 0]], [[10, 10], [10, 10], [10, 10], [10, 10]], [1, 1, 1, 1], ringNeighborhood(4), new Rng(0), { vmax: 0.2 }); expect(Math.max(...vv.map((w) => Math.hypot(w[0], w[1])))).toBeLessThanOrEqual(0.2 + 1e-12);
    const rng = new Rng(8); const f = gaussianSource([0.7, -0.4], 0.5); const res = psoOptimize(f, Array.from({ length: 12 }, () => [rng.uniform(-2, 2), rng.uniform(-2, 2)] as Vec2), 150, rng); expect(dist(res.best, [0.7, -0.4])).toBeLessThan(0.02); for (let i = 1; i < res.history.length; i++) expect(res.history[i]).toBeGreaterThanOrEqual(res.history[i - 1]);
  });
  it('repulsion and the robot swarm reaches the source with a bounded step', () => {
    const r = repulsion([[0, 0], [0.05, 0], [5, 5]], 0.1); close(r[0][0], -0.05); close(r[1][0], 0.05); expect(r[2]).toEqual([0, 0]);
    const rng = new Rng(9); const src: Vec2 = [2, 1.5]; const f = gaussianSource(src, 0.8); const x0: Vec2[] = Array.from({ length: 8 }, () => [rng.uniform(0, 0.5), rng.uniform(0, 0.5)]);
    const res = robotPsoSearch(f, x0, 400, rng, { vmax: 0.03, dMin: 0.1, noiseStd: 0.005 });
    let maxStep = 0; for (let k = 1; k < res.traj.length; k++) for (let i = 0; i < 8; i++) maxStep = Math.max(maxStep, dist(res.traj[k][i], res.traj[k - 1][i])); expect(maxStep).toBeLessThanOrEqual(0.03 + 1e-9);
    const last = res.traj[res.traj.length - 1]; const c: Vec2 = [last.reduce((s, q) => s + q[0], 0) / 8, last.reduce((s, q) => s + q[1], 0) / 8]; expect(dist(c, src)).toBeLessThan(0.3);
  });
});

describe('ПР3 — allocation: greedy vs optimal, SSI, CBBA', () => {
  it('greedy loses 3.5× on the course matrix; rectangular greedy is valid', () => {
    const C = [[1, 2], [1.1, 10]]; const g = greedyAssignment(C); expect(g.assignment).toEqual({ 0: 0, 1: 1 }); close(g.cost, 11); close(optimalAssignment(C).cost, 3.1);
    const rng = new Rng(1); const R = Array.from({ length: 4 }, () => Array.from({ length: 7 }, () => rng.uniform())); const a = greedyAssignment(R); expect(Object.keys(a.assignment).length).toBe(4); expect(new Set(Object.values(a.assignment)).size).toBe(4); expect(a.cost).toBeGreaterThanOrEqual(optimalAssignment(R).cost - 1e-12);
  });
  it('best insertion, every task exactly once, SSI within 2× of the brute-force optimum', () => {
    const tasks: Vec2[] = [[1, 0], [3, 0], [2, 0]]; const b = bestInsertion([0, 0], [0, 1], tasks, 2); expect(b.pos).toBe(1); close(b.inc, 0);
    const rng = new Rng(2); const robots: Vec2[] = Array.from({ length: 3 }, () => [rng.uniform(0, 10), rng.uniform(0, 10)]); const T: Vec2[] = Array.from({ length: 9 }, () => [rng.uniform(0, 10), rng.uniform(0, 10)]);
    const s = ssiAuction(robots, T); expect(s.routes.flat().sort((x, y) => x - y)).toEqual([...Array(9).keys()]); expect(s.rounds).toBe(9); expect(s.bids).toBe(3 * 45);
    const rng3 = new Rng(3); const r2: Vec2[] = Array.from({ length: 2 }, () => [rng3.uniform(0, 10), rng3.uniform(0, 10)]); const t5: Vec2[] = Array.from({ length: 5 }, () => [rng3.uniform(0, 10), rng3.uniform(0, 10)]);
    expect(totalLength(r2, ssiAuction(r2, t5).routes, t5)).toBeLessThanOrEqual(2 * bruteForceRouting(r2, t5)); expect(routeLength([0, 0], [0, 1], tasks)).toBe(3);
  });
  it('bundle: takes the best tasks within capacity, respects a better known bid; consensus is conflict-free on complete and line graphs; release resets later tasks', () => {
    const tasks: Vec2[] = [[1, 0], [2, 0], [50, 0]]; const rewards = [10, 10, 10]; const ag = new CBBAAgent(0, [0, 0], 3, 2); cbbaBuildBundle(ag, tasks, rewards); expect(ag.bundle).toEqual([0, 1]); expect(ag.path).toEqual([0, 1]); close(ag.y[0], 10 * 0.95, 1e-12); expect(ag.z).toEqual([0, 0, -1]);
    const a1 = new CBBAAgent(0, [0, 0], 1, 1); a1.y[0] = 100; a1.z[0] = 3; cbbaBuildBundle(a1, [[1, 0]], [10]); expect(a1.bundle).toEqual([]);
    for (const line of [false, true]) for (let seed = 0; seed < 5; seed++) { const rng = new Rng(seed + 10); const st: Vec2[] = Array.from({ length: 4 }, () => [rng.uniform(0, 10), rng.uniform(0, 10)]); const T: Vec2[] = Array.from({ length: 10 }, () => [rng.uniform(0, 10), rng.uniform(0, 10)]); const R = T.map(() => rng.uniform(5, 15)); const A = line ? pathGraph(4) : completeGraph(4); const r = runCbba(st, T, R, A, { capacity: 3 }); expect(isConflictFree(r.agents), `seed ${seed} line=${line}`).toBe(true); if (!line) expect(r.agents.reduce((s, a) => s + a.path.length, 0)).toBe(10); expect(r.iterations).toBeLessThan(200); }
    const t2: Vec2[] = [[1, 0], [2, 0]]; const a0 = new CBBAAgent(0, [0, 0], 2, 2), b0 = new CBBAAgent(1, [0, 0], 2, 2); cbbaBuildBundle(a0, t2, [10, 10]); b0.y[0] = 99; b0.z[0] = 1; b0.bundle = [0]; b0.path = [0]; cbbaConsensus([a0, b0], completeGraph(2)); expect(a0.bundle).toEqual([]); expect(a0.z[1]).toBe(-1); expect(a0.z[0]).toBe(1);
  });
});

const validMoves = (g: boolean[][], p: Cell[]) => p.every((c, t) => t === 0 || successors(g, p[t - 1]).some((s) => s[0] === c[0] && s[1] === c[1]));
const randomInstance = (rng: Rng, g: boolean[][], n: number) => { const free = freeCells(g); const idx = rng.sample(free.length, 2 * n); return { starts: idx.slice(0, n).map((i) => free[i]), goals: idx.slice(n).map((i) => free[i]) }; };
describe('ПР4 — MAPF on the warehouse grid', () => {
  const CORRIDOR = loadMap('#.#\n...\n#.#');
  it('space-time A*: plain shortest path, vertex / edge constraints, a goal blocked later, unreachable', () => {
    const g = loadMap(WAREHOUSE_MAP); const p = spaceTimeAStar(g, [0, 0], [6, 9])!; expect(p.length - 1).toBe(15); expect(validMoves(g, p)).toBe(true);
    expect(spaceTimeAStar(loadMap('...'), [0, 0], [0, 2], [{ kind: 'v', cell: [0, 1], t: 1 }])).toEqual([[0, 0], [0, 0], [0, 1], [0, 2]]);
    expect(spaceTimeAStar(loadMap('..'), [0, 0], [0, 1], [{ kind: 'e', from: [0, 0], to: [0, 1], t: 1 }])).toEqual([[0, 0], [0, 0], [0, 1]]);
    expect(spaceTimeAStar(loadMap('...'), [0, 0], [0, 1], [{ kind: 'v', cell: [0, 1], t: 4 }])!.length - 1).toBe(5);
    expect(spaceTimeAStar(loadMap('.#.'), [0, 0], [0, 2])).toBeNull();
  });
  it('prioritized planning crosses the corridor (SOC 5) but is incomplete', () => {
    const paths = prioritizedPlanning(CORRIDOR, [[1, 0], [0, 1]], [[1, 2], [2, 1]])!; expect(firstConflict(paths)).toBeNull(); expect(sumOfCosts(paths)).toBe(5);
    expect(prioritizedPlanning(loadMap('....'), [[0, 1], [0, 0]], [[0, 2], [0, 3]], [0, 1])).toBeNull();
  });
  it('conflict detection: vertex, swap, parked agent, following allowed', () => {
    expect(firstConflict([[[0, 0], [0, 1]], [[0, 2], [0, 1]]])).toEqual({ kind: 'v', a: 0, b: 1, cell: [0, 1], t: 1 });
    expect(firstConflict([[[0, 0], [0, 1]], [[0, 1], [0, 0]]])).toEqual({ kind: 'e', a: 0, b: 1, u: [0, 0], w: [0, 1], t: 1 });
    expect(firstConflict([[[0, 0]], [[0, 2], [0, 1], [0, 0]]])).toEqual({ kind: 'v', a: 0, b: 1, cell: [0, 0], t: 2 });
    expect(firstConflict([[[0, 1], [0, 2]], [[0, 0], [0, 1]]])).toBeNull();
  });
  it('CBS solves the incomplete case and is never worse than the best priority order', () => {
    const r = cbs(loadMap('....\n.#..'), [[0, 1], [0, 0]], [[0, 2], [0, 3]]); expect(r.paths).not.toBeNull(); expect(firstConflict(r.paths!)).toBeNull();
    const rng = new Rng(11); const g = loadMap(WAREHOUSE_MAP);
    for (let k = 0; k < 4; k++) {
      const { starts, goals } = randomInstance(rng, g, 4); const { paths } = cbs(g, starts, goals); expect(paths).not.toBeNull(); expect(firstConflict(paths!)).toBeNull();
      paths!.forEach((p, i) => { expect(p[0]).toEqual(starts[i]); expect(p[p.length - 1]).toEqual(goals[i]); expect(validMoves(g, p)).toBe(true); });
      const perms = (xs: number[]): number[][] => (xs.length <= 1 ? [xs] : xs.flatMap((x, i) => perms([...xs.slice(0, i), ...xs.slice(i + 1)]).map((q) => [x, ...q])));
      const best = Math.min(...perms([0, 1, 2, 3]).map((o) => { const pp = prioritizedPlanning(g, starts, goals, o); return pp ? sumOfCosts(pp) : Infinity; })); expect(sumOfCosts(paths!)).toBeLessThanOrEqual(best);
    }
  });
  it('ADG: following dependency; delayed execution never collides and ends at the goals', () => {
    const { deps } = actionDependencies([[[0, 1], [0, 2]], [[0, 0], [0, 1]]]); expect([...deps.get('1,0')!]).toEqual(['0,0']); expect(deps.get('0,0')!.size).toBe(0);
    const rng = new Rng(12); const g = loadMap(WAREHOUSE_MAP);
    for (let k = 0; k < 5; k++) { const { starts, goals } = randomInstance(rng, g, 6); const { paths } = cbs(g, starts, goals); const ex = executeAdg(paths!, 0.3, rng); expect(ex.ticks).not.toBeNull(); expect(ex.collisions).toBe(0); expect(ex.log[ex.log.length - 1]).toEqual(paths!.map((p) => p[p.length - 1])); }
  });
});

describe('ПР5 — coverage and distributed estimation', () => {
  it('two robots split the square; empty cell keeps the robot; cost value; Lloyd is monotone and gathers at the hot spot', () => {
    const { q, dA } = gridPoints(0, 1, 0, 1, 0.1); const p: Vec2[] = [[0.25, 0.5], [0.75, 0.5]]; const lab = voronoiLabels(p, q); q.forEach((z, k) => { if (z[0] < 0.5) expect(lab[k]).toBe(0); if (z[0] > 0.5) expect(lab[k]).toBe(1); });
    const { M, C } = massCentroids(p, q, q.map(() => 1), dA, lab); close(M[0], 0.5, 1e-9); close(M[1], 0.5, 1e-9); close(C[0][0], 0.25, 1e-9); close(C[1][0], 0.75, 1e-9); close(C[0][1], 0.5, 1e-9);
    const e = massCentroids([[0, 0], [5, 5]], [[0, 0], [0.1, 0]], [1, 1], 1, voronoiLabels([[0, 0], [5, 5]], [[0, 0], [0.1, 0]])); expect(e.M[1]).toBe(0); expect(e.C[1]).toEqual([5, 5]);
    close(coverageCost([[0, 0]], [[1, 0], [0, 2]], [1, 0.5], 0.1), 0.1 * (1 + 0.5 * 4), 1e-12);
    const rng = new Rng(1); const G = gridPoints(0, 4, 0, 4, 0.1); const phi = gaussianDensity(G.q, [3, 1], 0.8); let P: Vec2[] = Array.from({ length: 6 }, () => [rng.uniform(0, 0.8), rng.uniform(0, 0.8)]); const costs = [coverageCost(P, G.q, phi, G.dA)];
    for (let k = 0; k < 60; k++) { P = lloydStep(P, G.q, phi, G.dA); costs.push(coverageCost(P, G.q, phi, G.dA)); }
    for (let k = 1; k < costs.length; k++) expect(costs[k]).toBeLessThanOrEqual(costs[k - 1] + 1e-9); expect(costs[costs.length - 1]).toBeLessThan(0.3 * costs[0]);
    const c: Vec2 = [P.reduce((s, z) => s + z[0], 0) / 6, P.reduce((s, z) => s + z[1], 0) / 6]; expect(dist(c, [3, 1])).toBeLessThan(1);
  });
  it('limited range: a far robot waits; the limited cost decreases and robots spread', () => {
    const { q, dA } = gridPoints(0, 1, 0, 1, 0.05); const p2 = limitedLloydStep([[5, 5], [0.5, 0.5]], q, q.map(() => 1), dA, 0.3); expect(p2[0]).toEqual([5, 5]);
    const rng = new Rng(2); const G = gridPoints(0, 5, 0, 5, 0.1); const phi = G.q.map(() => 1); let P: Vec2[] = Array.from({ length: 8 }, () => [rng.uniform(2, 3), rng.uniform(2, 3)]); const c0 = limitedCost(P, G.q, phi, G.dA, 0.8);
    for (let k = 0; k < 80; k++) P = limitedLloydStep(P, G.q, phi, G.dA, 0.8); expect(limitedCost(P, G.q, phi, G.dA, 0.8)).toBeLessThan(c0); expect(minPairDistance(P)).toBeGreaterThan(0.5);
  });
  it('information consensus converges to the centralised estimate; covariance intersection is consistent and picks the better estimate', () => {
    const rng = new Rng(3); const n = 8; const { A } = randomGeometricGraph(n, 0.55, rng); const W = metropolisWeights(A); const xt: Vec2 = [2, -1];
    const R: M2[] = Array.from({ length: n }, () => [[rng.uniform(0.01, 1), 0], [0, rng.uniform(0.01, 1)]]); const z: Vec2[] = R.map((Ri) => [xt[0] + rng.normal(0, Math.sqrt(Ri[0][0])), xt[1] + rng.normal(0, Math.sqrt(Ri[1][1]))]);
    const est = informationConsensus(z, R, W, 300); const { x } = centralizedEstimate(z, R); expect(est.length).toBe(301); for (const e of est[300]) { close(e[0], x[0], 1e-6); close(e[1], x[1], 1e-6); } est[0].forEach((e, i) => { close(e[0], z[i][0], 1e-9); close(e[1], z[i][1], 1e-9); });
    const ci = covarianceIntersection([0, 0], [[1, 0], [0, 4]], [1, 1], [[4, 0], [0, 1]]); close(ci.omega, 0.5, 1e-9); const naive = inv2(add2(inv2([[1, 0], [0, 4]]), inv2([[4, 0], [0, 1]]))); const diff: M2 = add2(ci.P, naive, 1, -1); expect(eig2(diff)[0]).toBeGreaterThanOrEqual(-1e-12); close(ci.x[0], 0.2, 1e-9); close(ci.x[1], 0.8, 1e-9);
    const c2 = covarianceIntersection([0, 0], [[0.1, 0], [0, 0.1]], [5, 5], [[10, 0], [0, 10]]); expect(c2.omega).toBe(1); close(c2.x[0], 0, 1e-9);
  });
});

describe('ПР6 — group safety with barrier functions', () => {
  it('pair constraint values; the QP filter (inactive, single active, corner, brute-force check)', () => {
    const c = pairConstraint([1, 0], [0, 0], 0.5, 2); expect(c.a).toEqual([2, 0]); close(c.b, -0.5 * 2 * (1 - 0.25));
    expect(safeVelocity([0.1, 0], [{ a: [1, 0], b: -1 }])).toEqual([0.1, 0]); const u1 = safeVelocity([-1, 1], [{ a: [1, 0], b: 0 }]); close(u1[0], 0); close(u1[1], 1);
    const u2 = safeVelocity([0, 0], [{ a: [1, 0], b: 0.5 }, { a: [0, 1], b: 0.5 }]); close(u2[0], 0.5); close(u2[1], 0.5);
    const rng = new Rng(1);
    for (let k = 0; k < 30; k++) {
      const cons = [...Array.from({ length: 4 }, () => ({ a: [rng.normal(), rng.normal()] as Vec2, b: rng.uniform(-1, 0) })), ...speedPolygon(1, 8)]; const un: Vec2 = [rng.normal() * 2, rng.normal() * 2]; const u = safeVelocity(un, cons);
      let best = Infinity; for (let i = 0; i <= 240; i++) for (let j = 0; j <= 240; j++) { const g: Vec2 = [-1.2 + (2.4 * i) / 240, -1.2 + (2.4 * j) / 240]; if (cons.every((cc) => cc.a[0] * g[0] + cc.a[1] * g[1] >= cc.b)) best = Math.min(best, (g[0] - un[0]) ** 2 + (g[1] - un[1]) ** 2); }
      expect((u[0] - un[0]) ** 2 + (u[1] - un[1]) ** 2).toBeLessThanOrEqual(best + 1e-9); expect(cons.every((cc) => cc.a[0] * u[0] + cc.a[1] * u[1] >= cc.b - 1e-7)).toBe(true);
    }
  });
  it('crossing is safe with the filter and collides without it', () => {
    const r = runGroup([[0, 0], [1, 0.05]], [[1, 0], [0, 0.05]], 1500); expect(r.dmin).toBeGreaterThanOrEqual(0.2 - 1e-3); expect(r.maxGoalError).toBeLessThan(0.05);
    let p: Vec2[] = [[0, 0], [1, 0]]; const goals: Vec2[] = [[1, 0], [0, 0]]; let dmin = Infinity; for (let k = 0; k < 600; k++) { const u = goToGoal(p, goals, 1, 0.3); p = p.map((q, i) => [q[0] + 0.02 * u[i][0], q[1] + 0.02 * u[i][1]] as Vec2); dmin = Math.min(dmin, minDist(p)); } expect(dmin).toBeLessThan(0.05);
  });
  it('hysteresis of the unstuck rule and the antipodal swap of 8 robots', () => {
    let r = unstuckNominal([[0, 0]], [[1, 0]], [[0, 0]], [0], 1, 0.3, { holdSteps: 3 }); expect(r.hold[0]).toBe(2); const e = rotate([0.3, 0], -Math.PI / 4); close(r.uNom[0][0], e[0], 1e-12); close(r.uNom[0][1], e[1], 1e-12);
    r = unstuckNominal([[0, 0]], [[1, 0]], [[0.3, 0]], r.hold, 1, 0.3, { holdSteps: 3 }); expect(r.hold[0]).toBe(1); close(r.uNom[0][0], e[0], 1e-12);
    const { p, goals } = antipodal(8, 1); const plain = runGroup(p, goals, 2500); const fixed = runGroup(p, goals, 2500, { unstuck: true });
    expect(fixed.dmin).toBeGreaterThanOrEqual(0.2 - 2e-3); expect(plain.maxGoalError).toBeGreaterThan(0.5); expect(fixed.maxGoalError).toBeLessThan(0.1);
  }, 30000);
  it('stale data: margin and worst-case term; evading an uncooperative neighbour known with 0.5 s delay', () => {
    const c = staleConstraint([1, 0], [0, 0], 1, 0.1, 0.5, 2); expect(c.a).toEqual([2, 0]); close(c.b, -2 * (1 - 0.36) + 2 * 0.1, 1e-12);
    let pi: Vec2 = [0, 0], pj: Vec2 = [1.2, 0]; const hist: Vec2[] = [pj]; let dmin = Infinity; const DT = 0.02;
    for (let k = 0; k < 1500; k++) { const lag = 25; const known = hist[Math.max(0, hist.length - 1 - lag)]; const age = Math.min(lag, hist.length - 1) * DT; const u = safeVelocity([0, 0], [...speedPolygon(0.3), staleConstraint(pi, known, age, 0.1, 0.3, 2)]); pi = [pi[0] + DT * u[0], pi[1] + DT * u[1]]; pj = [pj[0] - DT * 0.1, pj[1]]; hist.push(pj); dmin = Math.min(dmin, dist(pi, pj)); }
    expect(dmin).toBeGreaterThanOrEqual(0.3 - 1e-3);
  });
});
