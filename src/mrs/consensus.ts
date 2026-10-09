/**
 * Consensus, formation and connectivity maintenance (course chapter 4, practicum ПР1) plus the resilient W-MSR
 * update (chapter 16 §16.8.3), event-triggered consensus (chapter 13 §13.5.2), cooperative transport (§4.5.2) and
 * potential-field collision avoidance (§4.5.3).
 */
import { Vec2, sub, norm, mean2 } from './rng';
import { Matrix, laplacian, laplacianSpectrum, neighbors, edges } from './graph';

/** One discrete step x⁺ = x − εLx for scalar states (N) or vector states (N × d). */
export function consensusStep(x: number[], A: Matrix, eps: number): number[];
export function consensusStep(x: Vec2[], A: Matrix, eps: number): Vec2[];
export function consensusStep(x: number[] | Vec2[], A: Matrix, eps: number): number[] | Vec2[] {
  const L = laplacian(A); const n = x.length;
  if (typeof x[0] === 'number') { const xs = x as number[]; return xs.map((xi, i) => { let s = 0; for (let j = 0; j < n; j++) s += L[i][j] * xs[j]; return xi - eps * s; }); }
  const xv = x as Vec2[]; return xv.map((xi, i) => { let sx = 0, sy = 0; for (let j = 0; j < n; j++) { sx += L[i][j] * xv[j][0]; sy += L[i][j] * xv[j][1]; } return [xi[0] - eps * sx, xi[1] - eps * sy] as Vec2; });
}
export function runConsensus(x0: number[], A: Matrix, eps: number, steps: number): number[][] { const xs = [x0.slice()]; for (let k = 0; k < steps; k++) xs.push(consensusStep(xs[xs.length - 1], A, eps)); return xs; }
/** Continuous protocol ẋ = −Lx integrated with Euler steps h (chapter 4 §4.3.1). */
export function runContinuousConsensus(x0: number[], A: Matrix, h: number, steps: number): number[][] { return runConsensus(x0, A, h, steps); }
/** Disagreement ‖x − x̄·1‖ (vector states: over all coordinates). */
export function disagreement(x: number[] | Vec2[]): number {
  if (!x.length) return 0;
  if (typeof x[0] === 'number') { const xs = x as number[]; const m = xs.reduce((s, v) => s + v, 0) / xs.length; return Math.sqrt(xs.reduce((s, v) => s + (v - m) ** 2, 0)); }
  const xv = x as Vec2[]; const m = mean2(xv); return Math.sqrt(xv.reduce((s, v) => s + (v[0] - m[0]) ** 2 + (v[1] - m[1]) ** 2, 0));
}
/** Convergence factor ρ = max_{i≥2} |1 − ελᵢ| of the discrete protocol and the optimal step ε* = 2/(λ₂ + λₙ). */
export function convergenceFactor(A: Matrix, eps: number): { rho: number; epsOpt: number; rhoOpt: number; iterationsFor1pc: number } {
  const lam = laplacianSpectrum(A); const rest = lam.slice(1); if (!rest.length) return { rho: 0, epsOpt: 0, rhoOpt: 0, iterationsFor1pc: 0 };
  const rho = Math.max(...rest.map((l) => Math.abs(1 - eps * l))); const epsOpt = 2 / (rest[0] + rest[rest.length - 1]); const rhoOpt = Math.max(...rest.map((l) => Math.abs(1 - epsOpt * l)));
  return { rho, epsOpt, rhoOpt, iterationsFor1pc: rho > 0 && rho < 1 ? Math.ceil(Math.log(0.01) / Math.log(rho)) : Infinity };
}

/** Formation by offsets: uᵢ = −k Σⱼ aᵢⱼ((pᵢ − δᵢ) − (pⱼ − δⱼ)) — consensus on ξ = p − δ, any connected graph. */
export function formationVelocity(p: Vec2[], A: Matrix, offsets: Vec2[], gain = 1): Vec2[] {
  const xi = p.map((q, i) => sub(q, offsets[i])); const L = laplacian(A); const n = p.length;
  return xi.map((_, i) => { let sx = 0, sy = 0; for (let j = 0; j < n; j++) { sx += L[i][j] * xi[j][0]; sy += L[i][j] * xi[j][1]; } return [-gain * sx, -gain * sy] as Vec2; });
}
/** RMS shape error: deviation of ξᵢ = pᵢ − δᵢ from their mean. */
export function formationError(p: Vec2[], offsets: Vec2[]): number { const xi = p.map((q, i) => sub(q, offsets[i])); const m = mean2(xi); return Math.sqrt(xi.reduce((s, v) => s + (v[0] - m[0]) ** 2 + (v[1] - m[1]) ** 2, 0) / xi.length); }
/** Regular polygon offsets of radius r (formation shapes `circle`, `line`, `wedge`, `grid`). */
export function formationOffsets(shape: string, n: number, r = 0.3): Vec2[] {
  if (shape === 'line') return Array.from({ length: n }, (_, i) => [(i - (n - 1) / 2) * r, 0] as Vec2);
  if (shape === 'wedge') return Array.from({ length: n }, (_, i) => { const k = Math.ceil(i / 2), s = i % 2 ? 1 : -1; return [-k * r, i ? s * k * r : 0] as Vec2; });
  if (shape === 'grid') { const c = Math.ceil(Math.sqrt(n)); return Array.from({ length: n }, (_, i) => [(i % c) * r, -Math.floor(i / c) * r] as Vec2); }
  return Array.from({ length: n }, (_, i) => { const a = (2 * Math.PI * i) / n; return [r * Math.cos(a), r * Math.sin(a)] as Vec2; });
}

/** Connectivity-preserving edge weight w(d) = (2R − d)/(R − d)² (Ji & Egerstedt 2007); 0 when d ≥ R. */
export function connectivityWeight(d: number, radius: number): number { if (d >= radius) return 0; return (2 * radius - d) / (radius - d) ** 2; }
/** Rendezvous over the edges of the INITIAL graph with the weights above: no initial edge is ever broken. */
export function rendezvousVelocity(p: Vec2[], Ainit: Matrix, radius: number, gain = 1): Vec2[] {
  const u: Vec2[] = p.map(() => [0, 0]);
  for (const [i, j] of edges(Ainit)) { const d = norm(sub(p[i], p[j])); const w = connectivityWeight(d, radius); u[i][0] -= gain * w * (p[i][0] - p[j][0]); u[i][1] -= gain * w * (p[i][1] - p[j][1]); u[j][0] -= gain * w * (p[j][0] - p[i][0]); u[j][1] -= gain * w * (p[j][1] - p[i][1]); }
  return u;
}

/** W-MSR step: drop up to F larger and F smaller neighbour values, then a consensus step over the rest. Malicious agents keep their values. */
export function wmsrStep(x: number[], A: Matrix, F: number, eps: number, malicious: number[] = []): number[] {
  const out = x.slice();
  for (let i = 0; i < x.length; i++) {
    if (malicious.includes(i)) continue;
    const vals = neighbors(A, i).map((j) => x[j]);
    const higher = vals.filter((v) => v > x[i]).sort((a, b) => b - a).slice(F); const lower = vals.filter((v) => v < x[i]).sort((a, b) => a - b).slice(F); const equal = vals.filter((v) => v === x[i]);
    let s = 0; for (const v of [...higher, ...lower, ...equal]) s += v - x[i];
    out[i] = x[i] + eps * s;
  }
  return out;
}

/** Event-triggered consensus (§13.5.2): agent i broadcasts x̂ᵢ only when |x̂ᵢ − xᵢ| exceeds σ|xᵢ| + ε; neighbours use the last broadcast values. */
export function eventTriggeredConsensus(x0: number[], A: Matrix, eps: number, steps: number, sigma = 0.1, absThreshold = 0): { xs: number[][]; broadcasts: number; periodicMessages: number; events: number[] } {
  const n = x0.length; let x = x0.slice(); const xhat = x0.slice(); const xs = [x.slice()]; let broadcasts = n; const events: number[] = [n]; const L = laplacian(A);
  for (let k = 0; k < steps; k++) {
    let ev = 0;
    for (let i = 0; i < n; i++) if (Math.abs(xhat[i] - x[i]) > sigma * Math.abs(x[i]) + absThreshold) { xhat[i] = x[i]; ev++; }
    broadcasts += ev; events.push(ev);
    x = x.map((xi, i) => { let s = 0; for (let j = 0; j < n; j++) s += L[i][j] * xhat[j]; return xi - eps * s; });
    xs.push(x.slice());
  }
  return { xs, broadcasts, periodicMessages: n * (steps + 1), events };
}

/** Cooperative transport (§4.5.2): n robots carry an object at the mean of the grasp points with a PD law on the object error. */
export function cooperativeTransport(p0: Vec2[], target: Vec2, opts: { m?: number; kp?: number; kd?: number; dt?: number; steps?: number } = {}): { t: number[]; xo: Vec2[]; regime: 'overdamped' | 'critical' | 'oscillatory'; omega: number; settlingTime: number; overshoot: number } {
  const m = opts.m ?? 1, kp = opts.kp ?? 4, kd = opts.kd ?? 2, dt = opts.dt ?? 0.01, steps = opts.steps ?? 2000;
  let p = p0.map((q) => [q[0], q[1]] as Vec2); let v: Vec2[] = p.map(() => [0, 0]); const t: number[] = []; const xo: Vec2[] = [];
  const disc = kd * kd - 4 * m * kp; const regime = disc > 1e-12 ? 'overdamped' : disc < -1e-12 ? 'oscillatory' : 'critical'; const omega = disc < 0 ? Math.sqrt(-disc) / (2 * m) : 0;
  const c0 = mean2(p); const d0 = norm(sub(c0, target)) || 1; const dir0: Vec2 = [(c0[0] - target[0]) / d0, (c0[1] - target[1]) / d0]; let settling = NaN, overshoot = 0;
  for (let k = 0; k <= steps; k++) {
    const c = mean2(p); const cv = mean2(v); t.push(k * dt); xo.push(c);
    const e = sub(c, target); const d = norm(e); if (d <= 0.02 * d0 && Number.isNaN(settling)) settling = k * dt; if (d > 0.02 * d0) settling = NaN;
    const along = e[0] * dir0[0] + e[1] * dir0[1]; if (along < 0) overshoot = Math.max(overshoot, -along / d0);
    const a: Vec2 = [(-kp * e[0] - kd * cv[0]) / m, (-kp * e[1] - kd * cv[1]) / m];
    v = v.map((vi) => [vi[0] + dt * a[0], vi[1] + dt * a[1]] as Vec2); p = p.map((q, i) => [q[0] + dt * v[i][0], q[1] + dt * v[i][1]] as Vec2);
  }
  return { t, xo, regime, omega, settlingTime: settling, overshoot };
}

/** Potential-field repulsion −∇U with U = Σ k_r/‖pᵢ − pⱼ‖ + Σ k_o/‖pᵢ − oₖ‖ (§4.5.3), saturated at umax. */
export function potentialRepulsion(p: Vec2[], obstacles: Vec2[], kr = 0.01, ko = 0.01, umax = 1): Vec2[] {
  return p.map((pi, i) => {
    let ux = 0, uy = 0;
    for (let j = 0; j < p.length; j++) { if (j === i) continue; const d = sub(pi, p[j]); const r = Math.max(norm(d), 1e-6); ux += (kr * d[0]) / r ** 3; uy += (kr * d[1]) / r ** 3; }
    for (const o of obstacles) { const d = sub(pi, o); const r = Math.max(norm(d), 1e-6); ux += (ko * d[0]) / r ** 3; uy += (ko * d[1]) / r ** 3; }
    const nrm = Math.hypot(ux, uy); return (nrm > umax ? [(ux * umax) / nrm, (uy * umax) / nrm] : [ux, uy]) as Vec2;
  });
}
