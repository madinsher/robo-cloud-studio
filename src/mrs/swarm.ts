/**
 * Swarm behaviour and swarm intelligence (course chapter 5, practicum ПР2): Reynolds boids, the polarization order
 * parameter and the Vicsek phase transition, PSO with a local (lbest) topology and its robot version (speed limit,
 * repulsion, noisy measurements), ant colony optimisation for routing (TSP), firefly and grey-wolf optimisers.
 */
import { Rng, Vec2, sub, norm, dist, clampNorm } from './rng';

export interface BoidsParams { rSep: number; rView: number; wSep?: number; wAli?: number; wCoh?: number }
/** Acceleration of robot i by the three Reynolds rules (§5.3): separation Σ(pᵢ−pⱼ)/‖·‖² (closer than rSep), alignment v̄−vᵢ, cohesion p̄−pᵢ. */
export function boidsAcceleration(p: Vec2[], v: Vec2[], i: number, prm: BoidsParams): Vec2 {
  const wSep = prm.wSep ?? 0.05, wAli = prm.wAli ?? 5, wCoh = prm.wCoh ?? 0.1;
  let sx = 0, sy = 0, ax = 0, ay = 0, cx = 0, cy = 0, n = 0;
  for (let j = 0; j < p.length; j++) { if (j === i) continue; const d = dist(p[i], p[j]); if (d > prm.rView) continue; n++; ax += v[j][0]; ay += v[j][1]; cx += p[j][0]; cy += p[j][1]; if (d < prm.rSep && d > 1e-12) { sx += (p[i][0] - p[j][0]) / (d * d); sy += (p[i][1] - p[j][1]) / (d * d); } }
  if (!n) return [0, 0];
  return [wSep * sx + wAli * (ax / n - v[i][0]) + wCoh * (cx / n - p[i][0]), wSep * sy + wAli * (ay / n - v[i][1]) + wCoh * (cy / n - p[i][1])];
}
/** Synchronous flock step; the speed is kept in [vmin, vmax] (a flock has no standing members). */
export function boidsStep(p: Vec2[], v: Vec2[], dt: number, vmin: number, vmax: number, prm: BoidsParams): { p: Vec2[]; v: Vec2[] } {
  const acc = p.map((_, i) => boidsAcceleration(p, v, i, prm));
  const vn = v.map((vi, i) => { const w: Vec2 = [vi[0] + dt * acc[i][0], vi[1] + dt * acc[i][1]]; const s = norm(w) || 1e-12; const c = Math.min(Math.max(s, vmin), vmax); return [(w[0] * c) / s, (w[1] * c) / s] as Vec2; });
  return { p: p.map((q, i) => [q[0] + dt * vn[i][0], q[1] + dt * vn[i][1]] as Vec2), v: vn };
}
/** Order parameter φ = ‖Σvᵢ‖ / Σ‖vᵢ‖ ∈ [0, 1]. */
export function polarization(v: Vec2[]): number { let sx = 0, sy = 0, s = 0; for (const w of v) { sx += w[0]; sy += w[1]; s += norm(w); } return s > 0 ? Math.hypot(sx, sy) / s : 0; }
export function minPairDistance(p: Vec2[]): number { let m = Infinity; for (let i = 0; i < p.length; i++) for (let j = i + 1; j < p.length; j++) m = Math.min(m, dist(p[i], p[j])); return m; }

/** Vicsek (1995) step on a box×box torus: θᵢ⁺ = arg Σ_{j: dᵢⱼ≤r} e^{iθⱼ} + ξᵢ, ξ ~ U[−η/2, η/2]. */
export function vicsekStep(p: Vec2[], theta: number[], speed: number, radius: number, eta: number, box: number, rng: Rng): { p: Vec2[]; theta: number[] } {
  const n = p.length; const th = new Array<number>(n);
  const torus = (a: number) => { let d = a % box; if (d > box / 2) d -= box; if (d < -box / 2) d += box; return d; };
  for (let i = 0; i < n; i++) { let c = 0, s = 0; for (let j = 0; j < n; j++) { const dx = torus(p[i][0] - p[j][0]), dy = torus(p[i][1] - p[j][1]); if (dx * dx + dy * dy <= radius * radius) { c += Math.cos(theta[j]); s += Math.sin(theta[j]); } } th[i] = Math.atan2(s, c); }
  const noise = th.map(() => rng.uniform(-eta / 2, eta / 2)); const th2 = th.map((a, i) => a + noise[i]);
  const p2 = p.map((q, i) => [(((q[0] + speed * Math.cos(th2[i])) % box) + box) % box, (((q[1] + speed * Math.sin(th2[i])) % box) + box) % box] as Vec2);
  return { p: p2, theta: th2 };
}
/** Mean order parameter after the transient (second half of the run by default). */
export function vicsekOrder(n: number, box: number, radius: number, eta: number, steps: number, rng: Rng, speed = 0.03, burnIn?: number): number {
  let p: Vec2[] = Array.from({ length: n }, () => [rng.uniform(0, box), rng.uniform(0, box)]); let th = Array.from({ length: n }, () => rng.uniform(-Math.PI, Math.PI));
  const b = burnIn ?? Math.floor(steps / 2); let acc = 0, cnt = 0;
  for (let k = 0; k < steps; k++) { ({ p, theta: th } = vicsekStep(p, th, speed, radius, eta, box, rng)); if (k >= b) { acc += polarization(th.map((a) => [Math.cos(a), Math.sin(a)] as Vec2)); cnt++; } }
  return cnt ? acc / cnt : 0;
}

/** lbest ring neighbourhood: i−k … i+k (including i). */
export function ringNeighborhood(n: number, k = 1): number[][] { return Array.from({ length: n }, (_, i) => Array.from({ length: 2 * k + 1 }, (_, d) => (((i + d - k) % n) + n) % n)); }
export interface PsoParams { w?: number; c1?: number; c2?: number; vmax?: number }
/** PSO velocity for a maximisation with a local topology (§5.6.2): v⁺ = w v + c₁r₁(pbest − x) + c₂r₂(lbest − x); r₁ then r₂ drawn for all particles. */
export function psoVelocity(x: Vec2[], v: Vec2[], pbest: Vec2[], pbestVal: number[], neigh: number[][], rng: Rng, prm: PsoParams = {}): Vec2[] {
  const w = prm.w ?? 0.72, c1 = prm.c1 ?? 1.49, c2 = prm.c2 ?? 1.49; const n = x.length;
  const r1 = x.map(() => [rng.random(), rng.random()] as Vec2); const r2 = x.map(() => [rng.random(), rng.random()] as Vec2);
  const out: Vec2[] = [];
  for (let i = 0; i < n; i++) { let best = neigh[i][0]; for (const j of neigh[i]) if (pbestVal[j] > pbestVal[best]) best = j; const lb = pbest[best]; let vn: Vec2 = [w * v[i][0] + c1 * r1[i][0] * (pbest[i][0] - x[i][0]) + c2 * r2[i][0] * (lb[0] - x[i][0]), w * v[i][1] + c1 * r1[i][1] * (pbest[i][1] - x[i][1]) + c2 * r2[i][1] * (lb[1] - x[i][1])]; if (prm.vmax !== undefined) vn = clampNorm(vn, prm.vmax); out.push(vn); }
  return out;
}
export function updateBest(x: Vec2[], val: number[], pbest: Vec2[], pbestVal: number[]): { pbest: Vec2[]; pbestVal: number[] } { const pb = pbest.map((q) => [q[0], q[1]] as Vec2); const pv = pbestVal.slice(); for (let i = 0; i < x.length; i++) if (val[i] > pv[i]) { pv[i] = val[i]; pb[i] = [x[i][0], x[i][1]]; } return { pbest: pb, pbestVal: pv }; }
/** Classical PSO (particles without bodies): best point, value and the history of the best value. */
export function psoOptimize(field: (z: Vec2) => number, x0: Vec2[], steps: number, rng: Rng, k = 1, vmax?: number): { best: Vec2; value: number; history: number[] } {
  let x = x0.map((q) => [q[0], q[1]] as Vec2); let v: Vec2[] = x.map(() => [0, 0]); let pbest = x.map((q) => [q[0], q[1]] as Vec2); let pbestVal = x.map((q) => field(q)); const neigh = ringNeighborhood(x.length, k); const history = [Math.max(...pbestVal)];
  for (let s = 0; s < steps; s++) { v = psoVelocity(x, v, pbest, pbestVal, neigh, rng, { vmax }); x = x.map((q, i) => [q[0] + v[i][0], q[1] + v[i][1]] as Vec2); ({ pbest, pbestVal } = updateBest(x, x.map((q) => field(q)), pbest, pbestVal)); history.push(Math.max(...pbestVal)); }
  let b = 0; for (let i = 1; i < pbestVal.length; i++) if (pbestVal[i] > pbestVal[b]) b = i;
  return { best: pbest[b], value: pbestVal[b], history };
}
/** Repulsion of robots closer than dMin: Σⱼ gain·(dMin − d)·(xᵢ − xⱼ)/d. */
export function repulsion(x: Vec2[], dMin: number, gain = 1): Vec2[] {
  return x.map((xi, i) => { let ux = 0, uy = 0; for (let j = 0; j < x.length; j++) { if (j === i) continue; const d = dist(xi, x[j]); if (d < 1e-9 || d >= dMin) continue; ux += (gain * (dMin - d) * (xi[0] - x[j][0])) / d; uy += (gain * (dMin - d) * (xi[1] - x[j][1])) / d; } return [ux, uy] as Vec2; });
}
export interface RobotPsoOptions { vmax?: number; dMin?: number; k?: number; noiseStd?: number; forget?: number }
/** Swarm source seeking by robots: step ≤ vmax, repulsion, the field is measured at the real position with noise; optional forgetting of records. */
export function robotPsoSearch(field: (z: Vec2) => number, x0: Vec2[], steps: number, rng: Rng, o: RobotPsoOptions = {}): { traj: Vec2[][]; best: number[]; bestPoint: Vec2 } {
  const vmax = o.vmax ?? 0.05, dMin = o.dMin ?? 0.1, k = o.k ?? 1, noise = o.noiseStd ?? 0, forget = o.forget ?? 0;
  const measure = (z: Vec2) => field(z) + (noise > 0 ? rng.normal(0, noise) : 0);
  let x = x0.map((q) => [q[0], q[1]] as Vec2); let v: Vec2[] = x.map(() => [0, 0]); let pbest = x.map((q) => [q[0], q[1]] as Vec2); let pbestVal = x.map((q) => measure(q)); const neigh = ringNeighborhood(x.length, k);
  const argmax = () => { let b = 0; for (let i = 1; i < pbestVal.length; i++) if (pbestVal[i] > pbestVal[b]) b = i; return b; };
  const traj = [x.map((q) => [q[0], q[1]] as Vec2)]; const best = [field(pbest[argmax()])];
  for (let s = 0; s < steps; s++) {
    if (forget > 0) pbestVal = pbestVal.map((pv) => pv - forget * Math.abs(pv));
    v = psoVelocity(x, v, pbest, pbestVal, neigh, rng); const rep = repulsion(x, dMin);
    const step = v.map((vi, i) => clampNorm([vi[0] + rep[i][0], vi[1] + rep[i][1]] as Vec2, vmax));
    x = x.map((q, i) => [q[0] + step[i][0], q[1] + step[i][1]] as Vec2); v = step;
    ({ pbest, pbestVal } = updateBest(x, x.map((q) => measure(q)), pbest, pbestVal));
    traj.push(x.map((q) => [q[0], q[1]] as Vec2)); best.push(field(pbest[argmax()]));
  }
  return { traj, best, bestPoint: pbest[argmax()] };
}
/** Concentration field: a main Gaussian source plus optional distractors [(centre, sigma, amp)]. */
export function gaussianSource(center: Vec2, sigma = 0.3, amp = 1, distractors: Array<[Vec2, number, number]> = []): (z: Vec2) => number {
  return (z) => { let v = amp * Math.exp(-((z[0] - center[0]) ** 2 + (z[1] - center[1]) ** 2) / (2 * sigma * sigma)); for (const [c, s, a] of distractors) v += a * Math.exp(-((z[0] - c[0]) ** 2 + (z[1] - c[1]) ** 2) / (2 * s * s)); return v; };
}

/** Ant colony optimisation for a closed tour over points (§5.5): returns the best tour, its length and the history. */
export function acoTsp(points: Vec2[], rng: Rng, o: { ants?: number; iterations?: number; alpha?: number; beta?: number; rho?: number; Q?: number; tau0?: number } = {}): { tour: number[]; length: number; history: number[]; greedyLength: number } {
  const n = points.length; const ants = o.ants ?? n, iters = o.iterations ?? 50, alpha = o.alpha ?? 1, beta = o.beta ?? 3, rho = o.rho ?? 0.3, Q = o.Q ?? 1, tau0 = o.tau0 ?? 1;
  const d = points.map((a) => points.map((b) => dist(a, b))); const tau = points.map(() => Array(n).fill(tau0)); const eta = d.map((r) => r.map((x) => (x > 0 ? 1 / x : 0)));
  const tourLength = (t: number[]) => t.reduce((s, c, i) => s + d[c][t[(i + 1) % n]], 0);
  let bestTour: number[] = [...Array(n).keys()], bestLen = Infinity; const history: number[] = [];
  const greedyTour = (start: number) => { const left = new Set([...Array(n).keys()]); left.delete(start); const t = [start]; while (left.size) { let cur = t[t.length - 1], nb = -1; for (const j of left) if (nb < 0 || d[cur][j] < d[cur][nb]) nb = j; t.push(nb); left.delete(nb); } return t; };
  const greedyLength = Math.min(...[...Array(n).keys()].map((s) => tourLength(greedyTour(s))));
  for (let it = 0; it < iters; it++) {
    const tours: number[][] = [];
    for (let a = 0; a < ants; a++) {
      const start = rng.int(n); const left = new Set([...Array(n).keys()]); left.delete(start); const t = [start];
      while (left.size) { const cur = t[t.length - 1]; const cands = [...left]; const w = cands.map((j) => tau[cur][j] ** alpha * eta[cur][j] ** beta); const tot = w.reduce((s, x) => s + x, 0); let r = rng.random() * tot, pick = cands[cands.length - 1]; for (let k = 0; k < cands.length; k++) { r -= w[k]; if (r <= 0) { pick = cands[k]; break; } } t.push(pick); left.delete(pick); }
      tours.push(t); const L = tourLength(t); if (L < bestLen) { bestLen = L; bestTour = t; }
    }
    for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) tau[i][j] *= 1 - rho;
    for (const t of tours) { const L = tourLength(t); for (let i = 0; i < n; i++) { const a = t[i], b = t[(i + 1) % n]; tau[a][b] += Q / L; tau[b][a] += Q / L; } }
    history.push(bestLen);
  }
  return { tour: bestTour, length: bestLen, history, greedyLength };
}

/** Firefly algorithm (§5.8) maximising a 2-D field: dimmer fireflies move towards brighter ones with attractiveness β₀e^{−γr²}. */
export function fireflyOptimize(field: (z: Vec2) => number, x0: Vec2[], steps: number, rng: Rng, o: { beta0?: number; gamma?: number; alpha?: number } = {}): { best: Vec2; value: number; history: number[] } {
  const beta0 = o.beta0 ?? 1, gamma = o.gamma ?? 1, alpha0 = o.alpha ?? 0.2; const x = x0.map((q) => [q[0], q[1]] as Vec2); let f = x.map((q) => field(q)); const history: number[] = [Math.max(...f)];
  for (let s = 0; s < steps; s++) {
    const alpha = alpha0 * (1 - s / steps);
    for (let i = 0; i < x.length; i++) for (let j = 0; j < x.length; j++) if (f[j] > f[i]) { const r2 = (x[i][0] - x[j][0]) ** 2 + (x[i][1] - x[j][1]) ** 2; const b = beta0 * Math.exp(-gamma * r2); x[i] = [x[i][0] + b * (x[j][0] - x[i][0]) + alpha * (rng.random() - 0.5), x[i][1] + b * (x[j][1] - x[i][1]) + alpha * (rng.random() - 0.5)]; f[i] = field(x[i]); }
    f = x.map((q) => field(q)); history.push(Math.max(...f));
  }
  let b = 0; for (let i = 1; i < f.length; i++) if (f[i] > f[b]) b = i; return { best: x[b], value: f[b], history };
}
/** Grey wolf optimiser (§5.9) maximising a 2-D field: the pack follows the three best wolves α, β, δ with a shrinking coefficient a: 2 → 0. */
export function greyWolfOptimize(field: (z: Vec2) => number, x0: Vec2[], steps: number, rng: Rng): { best: Vec2; value: number; history: number[] } {
  const x = x0.map((q) => [q[0], q[1]] as Vec2); let f = x.map((q) => field(q)); const history: number[] = [Math.max(...f)];
  for (let s = 0; s < steps; s++) {
    const order = [...f.keys()].sort((i, j) => f[j] - f[i]); const leaders = [x[order[0]], x[order[1] ?? order[0]], x[order[2] ?? order[0]]]; const a = 2 - (2 * s) / steps;
    for (let i = 0; i < x.length; i++) { const pos: Vec2 = [0, 0]; for (const L of leaders) { for (let c = 0; c < 2; c++) { const A = 2 * a * rng.random() - a, C = 2 * rng.random(); const D = Math.abs(C * L[c] - x[i][c]); pos[c] += (L[c] - A * D) / 3; } } x[i] = pos; }
    f = x.map((q) => field(q)); history.push(Math.max(...f));
  }
  let b = 0; for (let i = 1; i < f.length; i++) if (f[i] > f[b]) b = i; return { best: x[b], value: f[b], history };
}
/** Bee colony (§5.7) maximising a 2-D field: scouts sample, the best sites are exploited by more bees in a shrinking patch. */
export function beeColonyOptimize(field: (z: Vec2) => number, x0: Vec2[], steps: number, rng: Rng, o: { elite?: number; patch?: number; recruits?: number } = {}): { best: Vec2; value: number; history: number[] } {
  let x = x0.map((q) => [q[0], q[1]] as Vec2); const elite = o.elite ?? Math.max(1, Math.floor(x.length / 3)), recruits = o.recruits ?? 3; let patch = o.patch ?? 0.5; let f = x.map((q) => field(q)); const history = [Math.max(...f)];
  const lo = [Math.min(...x.map((q) => q[0])), Math.min(...x.map((q) => q[1]))], hi = [Math.max(...x.map((q) => q[0])), Math.max(...x.map((q) => q[1]))];
  for (let s = 0; s < steps; s++) {
    const order = [...f.keys()].sort((i, j) => f[j] - f[i]); const nx: Vec2[] = [];
    for (let e = 0; e < Math.min(elite, x.length); e++) { const c = x[order[e]]; let best = c, bv = f[order[e]]; for (let r = 0; r < recruits; r++) { const z: Vec2 = [c[0] + patch * (rng.random() - 0.5), c[1] + patch * (rng.random() - 0.5)]; const v = field(z); if (v > bv) { bv = v; best = z; } } nx.push(best); }
    while (nx.length < x.length) nx.push([rng.uniform(lo[0], hi[0]), rng.uniform(lo[1], hi[1])]);
    x = nx; f = x.map((q) => field(q)); patch *= 0.95; history.push(Math.max(...f));
  }
  let b = 0; for (let i = 1; i < f.length; i++) if (f[i] > f[b]) b = i; return { best: x[b], value: f[b], history };
}
export { sub, norm };
