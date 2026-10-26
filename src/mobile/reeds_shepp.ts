/**
 * Reeds–Shepp (forward + reverse) and Dubins (forward only) shortest paths for car-like robots with a minimum turning
 * radius. Formulas follow Reeds & Shepp (1990) as implemented in OMPL's ReedsSheppStateSpace (all 48 words via the
 * time-flip / reflect / backwards symmetries) and Shkel & Lumelsky for Dubins. Every candidate is integrated and kept
 * only when it really ends at the goal, so a numerical corner case can never produce a wrong path.
 */
import type { Pose2 } from './footprint';

type Seg = 'L' | 'R' | 'S';
export interface CarPath { types: Seg[]; lengths: number[] /* unit-radius arc lengths, negative = reverse */ }

const PI = Math.PI, TWOPI = 2 * Math.PI, ZERO = 1e-10;
const mod2pi = (x: number) => { let v = x % TWOPI; if (v < -PI) v += TWOPI; else if (v > PI) v -= TWOPI; return v; };
const polar = (x: number, y: number): [number, number] => [Math.sqrt(x * x + y * y), Math.atan2(y, x)];
function tauOmega(u: number, v: number, xi: number, eta: number, phi: number): [number, number] {
  const delta = mod2pi(u - v), A = Math.sin(u) - Math.sin(delta), B = Math.cos(u) - Math.cos(delta) - 1;
  const t1 = Math.atan2(eta * A - xi * B, xi * A + eta * B), t2 = 2 * (Math.cos(delta) - Math.cos(v) - Math.cos(u)) + 3;
  const tau = t2 < 0 ? mod2pi(t1 + PI) : mod2pi(t1);
  return [tau, mod2pi(tau - u + v - phi)];
}
type F = (x: number, y: number, phi: number) => [number, number, number] | null;
const LpSpLp: F = (x, y, phi) => { const [u, t] = polar(x - Math.sin(phi), y - 1 + Math.cos(phi)); if (t >= -ZERO) { const v = mod2pi(phi - t); if (v >= -ZERO) return [t, u, v]; } return null; };
const LpSpRp: F = (x, y, phi) => {
  const [u1r, t1] = polar(x + Math.sin(phi), y - 1 - Math.cos(phi)); const u1 = u1r * u1r;
  if (u1 >= 4) { const u = Math.sqrt(u1 - 4), theta = Math.atan2(2, u), t = mod2pi(t1 + theta), v = mod2pi(t - phi); if (t >= -ZERO && v >= -ZERO) return [t, u, v]; }
  return null;
};
const LpRmL: F = (x, y, phi) => {
  const xi = x - Math.sin(phi), eta = y - 1 + Math.cos(phi); const [u1, theta] = polar(xi, eta);
  if (u1 <= 4) { const u = -2 * Math.asin(0.25 * u1), t = mod2pi(theta + 0.5 * u + PI), v = mod2pi(phi - t + u); if (t >= -ZERO && u <= ZERO) return [t, u, v]; }
  return null;
};
const LpRupLumRm: F = (x, y, phi) => {
  const xi = x + Math.sin(phi), eta = y - 1 - Math.cos(phi), rho = 0.25 * (2 + Math.sqrt(xi * xi + eta * eta));
  if (rho <= 1) { const u = Math.acos(rho); const [t, v] = tauOmega(u, -u, xi, eta, phi); if (t >= -ZERO && v <= ZERO) return [t, u, v]; }
  return null;
};
const LpRumLumRp: F = (x, y, phi) => {
  const xi = x + Math.sin(phi), eta = y - 1 - Math.cos(phi), rho = (20 - xi * xi - eta * eta) / 16;
  if (rho >= 0 && rho <= 1) { const u = -Math.acos(rho); if (u >= -0.5 * PI) { const [t, v] = tauOmega(u, u, xi, eta, phi); if (t >= -ZERO && v >= -ZERO) return [t, u, v]; } }
  return null;
};
const LpRmSmLm: F = (x, y, phi) => {
  const xi = x - Math.sin(phi), eta = y - 1 + Math.cos(phi); const [rho, theta] = polar(xi, eta);
  if (rho >= 2) { const r = Math.sqrt(rho * rho - 4), u = 2 - r, t = mod2pi(theta + Math.atan2(r, -2)), v = mod2pi(phi - 0.5 * PI - t); if (t >= -ZERO && u <= ZERO && v <= ZERO) return [t, u, v]; }
  return null;
};
const LpRmSmRm: F = (x, y, phi) => {
  const xi = x + Math.sin(phi), eta = y - 1 - Math.cos(phi); const [rho, theta] = polar(-eta, xi);
  if (rho >= 2) { const t = theta, u = 2 - rho, v = mod2pi(t + 0.5 * PI - phi); if (t >= -ZERO && u <= ZERO && v <= ZERO) return [t, u, v]; }
  return null;
};
const LpRmSLmRp: F = (x, y, phi) => {
  const xi = x + Math.sin(phi), eta = y - 1 - Math.cos(phi); const [rho] = polar(xi, eta);
  if (rho >= 2) { const u = 4 - Math.sqrt(rho * rho - 4); if (u <= ZERO) { const t = mod2pi(Math.atan2((4 - u) * xi - 2 * eta, -2 * xi + (u - 4) * eta)), v = mod2pi(t - phi); if (t >= -ZERO && v >= -ZERO) return [t, u, v]; } }
  return null;
};

const TYPES: Seg[][] = [
  ['L', 'R', 'L'], ['R', 'L', 'R'], ['L', 'R', 'L', 'R'], ['R', 'L', 'R', 'L'], ['L', 'R', 'S', 'L'], ['R', 'L', 'S', 'R'],
  ['L', 'S', 'R', 'L'], ['R', 'S', 'L', 'R'], ['L', 'R', 'S', 'R'], ['R', 'L', 'S', 'L'], ['R', 'S', 'R', 'L'], ['L', 'S', 'L', 'R'],
  ['L', 'S', 'R'], ['R', 'S', 'L'], ['L', 'S', 'L'], ['R', 'S', 'R'], ['L', 'R', 'S', 'L', 'R'], ['R', 'L', 'S', 'R', 'L'],
];

/** All Reeds–Shepp candidate words for a unit-radius goal (x, y, phi) relative to the start. */
export function reedsSheppCandidates(x: number, y: number, phi: number): CarPath[] {
  const out: CarPath[] = [];
  const add = (type: number, l: number[]) => out.push({ types: TYPES[type], lengths: l });
  const four = (f: F, tA: number, tB: number, mk: (r: [number, number, number], s: number) => number[], X = x, Y = y) => {
    let r = f(X, Y, phi); if (r) add(tA, mk(r, 1));
    r = f(-X, Y, -phi); if (r) add(tA, mk(r, -1));
    r = f(X, -Y, -phi); if (r) add(tB, mk(r, 1));
    r = f(-X, -Y, phi); if (r) add(tB, mk(r, -1));
  };
  const h = 0.5 * PI;
  // CSC
  four(LpSpLp, 14, 15, ([t, u, v], s) => [s * t, s * u, s * v]);
  four(LpSpRp, 12, 13, ([t, u, v], s) => [s * t, s * u, s * v]);
  // CCC (+ backwards)
  const xb = x * Math.cos(phi) + y * Math.sin(phi), yb = x * Math.sin(phi) - y * Math.cos(phi);
  four(LpRmL, 0, 1, ([t, u, v], s) => [s * t, s * u, s * v]);
  four(LpRmL, 0, 1, ([t, u, v], s) => [s * v, s * u, s * t], xb, yb);
  // CCCC
  four(LpRupLumRm, 2, 3, ([t, u, v], s) => [s * t, s * u, -s * u, s * v]);
  four(LpRumLumRp, 2, 3, ([t, u, v], s) => [s * t, s * u, s * u, s * v]);
  // CCSC (+ backwards)
  four(LpRmSmLm, 4, 5, ([t, u, v], s) => [s * t, -s * h, s * u, s * v]);
  four(LpRmSmRm, 8, 9, ([t, u, v], s) => [s * t, -s * h, s * u, s * v]);
  four(LpRmSmLm, 6, 7, ([t, u, v], s) => [s * v, s * u, -s * h, s * t], xb, yb);
  four(LpRmSmRm, 10, 11, ([t, u, v], s) => [s * v, s * u, -s * h, s * t], xb, yb);
  // CCSCC
  four(LpRmSLmRp, 16, 17, ([t, u, v], s) => [s * t, -s * h, s * u, -s * h, s * v]);
  return out;
}

/** Dubins (forward only) candidate words for a unit-radius goal relative to the start (heading 0). */
export function dubinsCandidates(x: number, y: number, phi: number): CarPath[] {
  const m = (a: number) => { const v = a % TWOPI; return v < 0 ? v + TWOPI : v; };
  const d = Math.hypot(x, y), th = Math.atan2(y, x);
  const a = m(-th), b = m(phi - th);
  const sa = Math.sin(a), sb = Math.sin(b), ca = Math.cos(a), cb = Math.cos(b), cab = Math.cos(a - b);
  const out: CarPath[] = [];
  let p2 = 2 + d * d - 2 * cab + 2 * d * (sa - sb);
  if (p2 >= 0) { const tmp = Math.atan2(cb - ca, d + sa - sb); out.push({ types: ['L', 'S', 'L'], lengths: [m(-a + tmp), Math.sqrt(p2), m(b - tmp)] }); }
  p2 = 2 + d * d - 2 * cab + 2 * d * (sb - sa);
  if (p2 >= 0) { const tmp = Math.atan2(ca - cb, d - sa + sb); out.push({ types: ['R', 'S', 'R'], lengths: [m(a - tmp), Math.sqrt(p2), m(-b + tmp)] }); }
  p2 = -2 + d * d + 2 * cab + 2 * d * (sa + sb);
  if (p2 >= 0) { const p = Math.sqrt(p2), tmp = Math.atan2(-ca - cb, d + sa + sb) - Math.atan2(-2, p); out.push({ types: ['L', 'S', 'R'], lengths: [m(-a + tmp), p, m(-m(b) + tmp)] }); }
  p2 = -2 + d * d + 2 * cab - 2 * d * (sa + sb);
  if (p2 >= 0) { const p = Math.sqrt(p2), tmp = Math.atan2(ca + cb, d - sa - sb) - Math.atan2(2, p); out.push({ types: ['R', 'S', 'L'], lengths: [m(a - tmp), p, m(b - tmp)] }); }
  let tmp = (6 - d * d + 2 * cab + 2 * d * (sa - sb)) / 8;
  if (Math.abs(tmp) <= 1) { const p = m(TWOPI - Math.acos(tmp)), t = m(a - Math.atan2(ca - cb, d - sa + sb) + p / 2); out.push({ types: ['R', 'L', 'R'], lengths: [t, p, m(a - b - t + p)] }); }
  tmp = (6 - d * d + 2 * cab + 2 * d * (-sa + sb)) / 8;
  if (Math.abs(tmp) <= 1) { const p = m(TWOPI - Math.acos(tmp)), t = m(-a - Math.atan2(ca - cb, d + sa - sb) + p / 2); out.push({ types: ['L', 'R', 'L'], lengths: [t, p, m(m(b) - a - t + p)] }); }
  return out;
}

/** Integrate a unit-radius word from (0,0,0); returns the end pose. */
export function integrateWord(p: CarPath, x = 0, y = 0, phi = 0): [number, number, number] {
  for (let i = 0; i < p.types.length; i++) {
    const v = p.lengths[i];
    switch (p.types[i]) {
      case 'L': x += Math.sin(phi + v) - Math.sin(phi); y += -Math.cos(phi + v) + Math.cos(phi); phi += v; break;
      case 'R': x += -Math.sin(phi - v) + Math.sin(phi); y += Math.cos(phi - v) - Math.cos(phi); phi -= v; break;
      case 'S': x += v * Math.cos(phi); y += v * Math.sin(phi); break;
    }
  }
  return [x, y, phi];
}

export interface CarPathResult {
  /** Pose path [x, y, headingDeg, dir] (dir 1 forward, -1 reverse), first point = start. */
  poses: number[][];
  /** Geometric length (mm) and weighted cost (reverse and direction switches penalised). */
  length: number;
  cost: number;
  word: CarPath;
}

/**
 * Shortest car path between two poses with turning radius R (mm). Reverse allowed → Reeds–Shepp, else Dubins.
 * Returns null only if no candidate verifies (numerically degenerate input).
 */
export function reedsSheppPath(start: Pose2, goal: Pose2, R: number, opts: { allowReverse?: boolean; step?: number; reverseCost?: number; switchCost?: number } = {}): CarPathResult | null {
  const th0 = (start.theta * PI) / 180, c = Math.cos(th0), s = Math.sin(th0);
  const dx = goal.x - start.x, dy = goal.y - start.y;
  const x = (c * dx + s * dy) / R, y = (-s * dx + c * dy) / R, phi = mod2pi(((goal.theta - start.theta) * PI) / 180);
  const cands = opts.allowReverse === false ? dubinsCandidates(x, y, phi) : reedsSheppCandidates(x, y, phi);
  const rc = opts.reverseCost ?? 1.5, sc = opts.switchCost ?? 0.3; // switch cost in units of R
  let best: CarPath | null = null, bestCost = Infinity, bestLen = Infinity;
  for (const w of cands) {
    if (w.lengths.some((l) => !Number.isFinite(l))) continue;
    const [ex, ey, ep] = integrateWord(w);
    if (Math.hypot(ex - x, ey - y) > 1e-5 * Math.max(1, Math.hypot(x, y)) + 1e-6 || Math.abs(mod2pi(ep - phi)) > 1e-5) continue;
    let len = 0, cost = 0, switches = 0, lastDir = 0;
    for (const l of w.lengths) {
      if (Math.abs(l) < 1e-9) continue;
      len += Math.abs(l); cost += Math.abs(l) * (l < 0 ? rc : 1);
      const d = Math.sign(l); if (lastDir && d !== lastDir) switches++; lastDir = d;
    }
    cost += switches * sc;
    if (cost < bestCost - 1e-9) { bestCost = cost; bestLen = len; best = w; }
  }
  if (!best) return null;
  return { poses: sampleWord(best, start, R, opts.step ?? 100), length: bestLen * R, cost: bestCost * R, word: best };
}

/** Sample a word as pose points in the world (every `step` mm, plus every segment end). */
export function sampleWord(w: CarPath, start: Pose2, R: number, step = 100): number[][] {
  const th0 = (start.theta * PI) / 180;
  const out: number[][] = [[start.x, start.y, start.theta, 1]];
  let x = 0, y = 0, phi = 0;
  const toWorld = (lx: number, ly: number, lp: number, dir: number) => [start.x + R * (Math.cos(th0) * lx - Math.sin(th0) * ly), start.y + R * (Math.sin(th0) * lx + Math.cos(th0) * ly), ((th0 + lp) * 180) / PI, dir];
  for (let i = 0; i < w.types.length; i++) {
    const l = w.lengths[i];
    if (Math.abs(l) < 1e-9) continue;
    const n = Math.max(1, Math.ceil((Math.abs(l) * R) / step));
    const one: CarPath = { types: [w.types[i]], lengths: [0] };
    for (let k = 1; k <= n; k++) {
      one.lengths[0] = (l * k) / n;
      const [ex, ey, ep] = integrateWord(one, x, y, phi);
      out.push(toWorld(ex, ey, ep, Math.sign(l)));
    }
    [x, y, phi] = integrateWord({ types: [w.types[i]], lengths: [l] }, x, y, phi);
  }
  // normalise headings to (-180, 180]
  for (const p of out) p[2] = ((((p[2] + 180) % 360) + 360) % 360) - 180;
  return out;
}
