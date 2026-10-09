/**
 * Fuzzy (Mamdani) control for a robot in the group (course §13.4): membership functions, the obstacle-avoidance
 * rule base of table 13.2 over three range sectors, min–max inference and centroid defuzzification. The worked
 * example of §13.4.2 (d_L = 2.0, d_F = 0.7, d_R = 1.6) is reproduced by `fuzzyExample`.
 */
export type MF = (x: number) => number;
export const trap = (a: number, b: number, c: number, d: number): MF => (x) => (x <= a || x >= d ? 0 : x < b ? (x - a) / (b - a) : x <= c ? 1 : (d - x) / (d - c));
export const tri = (a: number, b: number, c: number): MF => trap(a, b, b, c);
export interface FuzzyVar { name: string; terms: Record<string, MF>; range: [number, number] }
export interface FuzzyRule { if: Record<string, string>; then: Record<string, string> }
export interface FuzzyController { inputs: FuzzyVar[]; outputs: FuzzyVar[]; rules: FuzzyRule[] }

/** The distance variable of §13.4.2: near (core [0, 0.4], falls to 0.9), medium (triangle with its apex at 1.0 m, calibrated so that μ(0.7) = 0.3 and μ(1.6) = 0.2 as in the worked example), far (rise 1.2–1.8). */
export const distanceVar = (name: string): FuzzyVar => ({ name, range: [0, 3], terms: { near: trap(-1, 0, 0.4, 0.9), medium: tri(4 / 7, 1.0, 1.75), far: trap(1.2, 1.8, 10, 11) } });
export const speedVar: FuzzyVar = { name: 'v', range: [0, 1], terms: { stop: trap(-1, 0, 0, 0.15), small: tri(0.05, 0.25, 0.45), medium: tri(0.3, 0.5, 0.7), large: trap(0.6, 0.85, 1, 2) } };
export const turnVar: FuzzyVar = { name: 'omega', range: [-1, 1], terms: { hardRight: trap(-2, -1, -0.8, -0.5), right: tri(-0.7, -0.4, -0.1), straight: tri(-0.2, 0, 0.2), left: tri(0.1, 0.4, 0.7), hardLeft: trap(0.5, 0.8, 1, 2) } };
/** Table 13.2 (D far, С medium, Б near; П0 straight, Л left, Пр right, ВЛ hard left, ВПр hard right). */
export const OBSTACLE_RULES: FuzzyRule[] = [
  { if: { dL: 'far', dF: 'far', dR: 'far' }, then: { v: 'large', omega: 'straight' } },
  { if: { dL: 'far', dF: 'medium', dR: 'far' }, then: { v: 'medium', omega: 'straight' } },
  { if: { dL: 'near', dF: 'far', dR: 'far' }, then: { v: 'medium', omega: 'right' } },
  { if: { dL: 'far', dF: 'far', dR: 'near' }, then: { v: 'medium', omega: 'left' } },
  { if: { dL: 'far', dF: 'near', dR: 'far' }, then: { v: 'small', omega: 'hardLeft' } },
  { if: { dL: 'near', dF: 'near', dR: 'far' }, then: { v: 'small', omega: 'hardRight' } },
  { if: { dL: 'far', dF: 'near', dR: 'near' }, then: { v: 'small', omega: 'hardLeft' } },
  { if: { dL: 'near', dF: 'far', dR: 'near' }, then: { v: 'medium', omega: 'straight' } },
  { if: { dL: 'near', dF: 'near', dR: 'near' }, then: { v: 'stop', omega: 'hardLeft' } },
  { if: { dL: 'medium', dF: 'medium', dR: 'medium' }, then: { v: 'medium', omega: 'straight' } },
];
export const obstacleController = (): FuzzyController => ({ inputs: [distanceVar('dL'), distanceVar('dF'), distanceVar('dR')], outputs: [speedVar, turnVar], rules: OBSTACLE_RULES });

export function fuzzify(ctrl: FuzzyController, x: Record<string, number>): Record<string, Record<string, number>> { const out: Record<string, Record<string, number>> = {}; for (const v of ctrl.inputs) { out[v.name] = {}; for (const [t, mf] of Object.entries(v.terms)) out[v.name][t] = mf(x[v.name] ?? 0); } return out; }
/** Mamdani inference: rule strength = min of antecedents, aggregation = max, centroid defuzzification on a grid. */
export function fuzzyInfer(ctrl: FuzzyController, x: Record<string, number>, grid = 201): { outputs: Record<string, number>; strengths: number[]; memberships: Record<string, Record<string, number>> } {
  const mem = fuzzify(ctrl, x); const strengths = ctrl.rules.map((r) => Math.min(...Object.entries(r.if).map(([v, t]) => mem[v]?.[t] ?? 0)));
  const outputs: Record<string, number> = {};
  for (const ov of ctrl.outputs) { let num = 0, den = 0; for (let k = 0; k < grid; k++) { const y = ov.range[0] + ((ov.range[1] - ov.range[0]) * k) / (grid - 1); let mu = 0; ctrl.rules.forEach((r, i) => { const t = r.then[ov.name]; if (!t || strengths[i] <= 0) return; mu = Math.max(mu, Math.min(strengths[i], ov.terms[t](y))); }); num += y * mu; den += mu; } outputs[ov.name] = den > 0 ? num / den : 0; }
  return { outputs, strengths, memberships: mem };
}
/** The numeric example of §13.4.2. */
export function fuzzyExample(): ReturnType<typeof fuzzyInfer> & { active: Array<{ rule: number; strength: number }> } { const ctrl = obstacleController(); const r = fuzzyInfer(ctrl, { dL: 2.0, dF: 0.7, dR: 1.6 }); return { ...r, active: r.strengths.map((s, i) => ({ rule: i + 1, strength: s })).filter((a) => a.strength > 0) }; }
/** Fuzzy avoidance driven through a corridor with obstacles: robot heading θ, three sector ranges from point obstacles. */
export function simulateFuzzyAvoidance(obstacles: Array<[number, number]>, goal: [number, number], o: { start?: [number, number]; theta0?: number; steps?: number; dt?: number; vmax?: number; wmax?: number; sensorRange?: number } = {}): { path: Array<[number, number]>; minObstacleDistance: number; reachedGoal: boolean; steps: number } {
  const ctrl = obstacleController(); let p: [number, number] = o.start ?? [0, 0]; let th = o.theta0 ?? 0; const steps = o.steps ?? 600, dt = o.dt ?? 0.1, vmax = o.vmax ?? 0.5, wmax = o.wmax ?? 1.2, range = o.sensorRange ?? 3; const path: Array<[number, number]> = [p]; let dmin = Infinity, reached = false, n = 0;
  const sector = (lo: number, hi: number) => { let d = range; for (const ob of obstacles) { const dx = ob[0] - p[0], dy = ob[1] - p[1]; const r = Math.hypot(dx, dy); let a = Math.atan2(dy, dx) - th; a = Math.atan2(Math.sin(a), Math.cos(a)); if (a >= lo && a < hi) d = Math.min(d, r); } return d; };
  for (let k = 0; k < steps; k++) {
    n++; const dF = sector(-Math.PI / 6, Math.PI / 6), dL = sector(Math.PI / 6, Math.PI / 2), dR = sector(-Math.PI / 2, -Math.PI / 6); const out = fuzzyInfer(ctrl, { dL, dF, dR }).outputs;
    const gAng = Math.atan2(goal[1] - p[1], goal[0] - p[0]); let e = gAng - th; e = Math.atan2(Math.sin(e), Math.cos(e)); const free = Math.min(dF, dL, dR) >= 1.8; const w = free ? Math.max(-wmax, Math.min(wmax, 2 * e)) : out.omega * wmax + 0.3 * e; const v = (free ? 1 : out.v) * vmax;
    th += w * dt; p = [p[0] + v * Math.cos(th) * dt, p[1] + v * Math.sin(th) * dt]; path.push(p); for (const ob of obstacles) dmin = Math.min(dmin, Math.hypot(ob[0] - p[0], ob[1] - p[1])); if (Math.hypot(goal[0] - p[0], goal[1] - p[1]) < 0.3) { reached = true; break; }
  }
  return { path, minObstacleDistance: dmin, reachedGoal: reached, steps: n };
}
