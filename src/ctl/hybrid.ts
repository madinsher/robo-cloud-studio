/**
 * Hybrid control (course chapter 9): mode automata with invariants and guards over continuous state, chattering /
 * Zeno checks (hysteresis between opposite guards), the dwell-time bound τ_d > ln μ / (2λ) (Theorem 9.3), a runtime
 * mode machine with dwell-time enforcement, control barrier function (CBF) safety filters solved in closed form for
 * one constraint and by iterative projection for several, and the Simplex architecture switch.
 */
import { expr, Env } from './expr';

export interface HybridMode { id: string; invariant?: string; /** human-readable dynamics / controller description */ dynamics?: string; /** optional speed limit applied in this mode (m/s) */ vMax?: number }
export interface HybridTransition { from: string; to: string; guard: string; /** minimal time in `from` before this transition may fire (s) */ dwell?: number; reset?: string }
export interface HybridSpec { name: string; modes: HybridMode[]; initial: string; transitions: HybridTransition[]; /** state variables with default values */ vars?: Record<string, number | boolean> }

export interface HybridIssue { level: 'error' | 'warning' | 'info'; message: string; transitions?: string[] }

/** Detect threshold comparisons `v < c` / `v > c` etc. in a guard. */
function thresholds(guard: string): Array<{ v: string; op: string; c: number }> {
  const out: Array<{ v: string; op: string; c: number }> = [];
  const re = /([A-Za-z_][\w.]*)\s*(<=|>=|<|>)\s*(-?\d+(?:\.\d+)?)/g; let m: RegExpExecArray | null;
  while ((m = re.exec(guard))) out.push({ v: m[1], op: m[2], c: Number(m[3]) });
  const re2 = /(-?\d+(?:\.\d+)?)\s*(<=|>=|<|>)\s*([A-Za-z_][\w.]*)/g;
  while ((m = re2.exec(guard))) out.push({ v: m[3], op: m[2] === '<' ? '>' : m[2] === '<=' ? '>=' : m[2] === '>' ? '<' : '<=', c: Number(m[1]) });
  return out;
}

/** Structural checks: chattering (opposite transitions on the same variable without a hysteresis gap or dwell), unknown modes, invariants contradicted by entry guards. */
export function checkHybrid(spec: HybridSpec): HybridIssue[] {
  const out: HybridIssue[] = [];
  const ids = new Set(spec.modes.map((m) => m.id));
  if (!ids.has(spec.initial)) out.push({ level: 'error', message: `initial mode ${spec.initial} is not defined` });
  for (const t of spec.transitions) { if (!ids.has(t.from) || !ids.has(t.to)) out.push({ level: 'error', message: `transition ${t.from} → ${t.to} refers to an unknown mode` }); try { expr(t.guard); } catch (e) { out.push({ level: 'error', message: `guard "${t.guard}": ${(e as Error).message}` }); } }
  for (const a of spec.transitions) for (const b of spec.transitions) {
    if (a.from !== b.to || a.to !== b.from || a.from >= a.to) continue;
    const ta = thresholds(a.guard), tb = thresholds(b.guard);
    for (const x of ta) for (const y of tb) {
      if (x.v !== y.v) continue;
      const aLess = x.op.startsWith('<'), bLess = y.op.startsWith('<');
      if (aLess === bLess) continue; // same direction, no oscillation pair
      const lo = aLess ? x.c : y.c, hi = aLess ? y.c : x.c; // a: v < lo ; b: v > hi
      const gap = hi - lo;
      const dwell = Math.max(a.dwell ?? 0, b.dwell ?? 0);
      if (gap <= 0 && dwell <= 0) out.push({ level: 'error', message: `chattering: ${a.from} ⇄ ${a.to} switch on ${x.v} at ${lo} / ${hi} with no hysteresis and no dwell time — Zeno behaviour under sensor noise (§9.2.3)`, transitions: [`${a.from}→${a.to}`, `${b.from}→${b.to}`] });
      else if (gap <= 0) out.push({ level: 'warning', message: `${a.from} ⇄ ${a.to}: no hysteresis on ${x.v}; the dwell time ${dwell} s bounds the switching rate`, transitions: [`${a.from}→${a.to}`, `${b.from}→${b.to}`] });
      else out.push({ level: 'info', message: `${a.from} ⇄ ${a.to}: hysteresis ${gap} on ${x.v} (${lo} / ${hi})`, transitions: [`${a.from}→${a.to}`, `${b.from}→${b.to}`] });
    }
  }
  for (const m of spec.modes) if (!spec.transitions.some((t) => t.from === m.id) && spec.transitions.some((t) => t.to === m.id)) out.push({ level: 'info', message: `mode ${m.id} is absorbing (no outgoing transition)` });
  return out;
}

/** Theorem 9.3: minimal dwell time for stability of switching among subsystems with decay rate λ and Lyapunov jump μ. */
export function dwellTime(lambda: number, mu: number): number { return Math.log(Math.max(1, mu)) / (2 * lambda); }

/** Runtime mode machine: evaluates guards on a value environment, enforces dwell times, counts switches (chattering detector). */
export class ModeMachine {
  mode: string; enteredAt = 0; switches = 0; readonly log: Array<{ t: number; from: string; to: string }> = [];
  constructor(readonly spec: HybridSpec) { this.mode = spec.initial; }
  step(t: number, env: Env): string {
    for (const tr of this.spec.transitions) {
      if (tr.from !== this.mode) continue;
      if (t - this.enteredAt < (tr.dwell ?? 0)) continue;
      if (expr(tr.guard).bool({ ...env, t, mode: this.mode })) { this.log.push({ t, from: this.mode, to: tr.to }); this.mode = tr.to; this.enteredAt = t; this.switches++; break; }
    }
    return this.mode;
  }
  invariantHolds(env: Env): boolean { const m = this.spec.modes.find((x) => x.id === this.mode); return !m?.invariant || expr(m.invariant).bool({ ...env, mode: this.mode }); }
  current(): HybridMode { return this.spec.modes.find((x) => x.id === this.mode)!; }
  reset(): void { this.mode = this.spec.initial; this.enteredAt = 0; this.switches = 0; this.log.length = 0; }
}

/** Simulate the mode machine on a sampled signal; returns switch count and max switching frequency (chattering detector). */
export function simulateModes(spec: HybridSpec, samples: Array<{ t: number } & Env>): { modes: string[]; switches: number; maxSwitchesPerSecond: number; invariantViolations: number } {
  const mm = new ModeMachine(spec); const modes: string[] = []; let viol = 0;
  for (const s of samples) { const { t, ...env } = s; modes.push(mm.step(t, env)); if (!mm.invariantHolds(env)) viol++; }
  let maxRate = 0;
  for (let i = 0; i < mm.log.length; i++) { let j = i; while (j < mm.log.length && mm.log[j].t - mm.log[i].t <= 1) j++; maxRate = Math.max(maxRate, j - i); }
  return { modes, switches: mm.switches, maxSwitchesPerSecond: maxRate, invariantViolations: viol };
}

// ---------------------------------------------------------------------------------------------
// Control barrier functions
// ---------------------------------------------------------------------------------------------

export interface CBFConstraint { /** h(x) value */ h: number; /** L_f h(x) */ lf?: number; /** L_g h(x): gradient row multiplying u */ lg: number[]; /** class-K gain α(h) = gamma·h */ gamma?: number }

/**
 * Safety filter: u* = argmin ‖u − u_nom‖² s.t. L_f h + L_g h·u ≥ −γ h for every constraint; optional box bounds on u.
 * One constraint is solved in closed form (projection on a half-space); several by cyclic projections (Dykstra-like).
 */
export function cbfFilter(uNom: number[], constraints: CBFConstraint[], opts: { uMin?: number[]; uMax?: number[]; iterations?: number } = {}): { u: number[]; active: boolean[]; margin: number[] } {
  let u = [...uNom];
  const project = (c: CBFConstraint, v: number[]): { v: number[]; active: boolean; margin: number } => {
    const a = c.lg; const rhs = -(c.gamma ?? 1) * c.h - (c.lf ?? 0); // a·u ≥ rhs
    const dot = a.reduce((s, ai, i) => s + ai * v[i], 0); const nn = a.reduce((s, ai) => s + ai * ai, 0);
    if (dot >= rhs || nn === 0) return { v, active: false, margin: dot - rhs };
    const lam = (rhs - dot) / nn;
    return { v: v.map((vi, i) => vi + lam * a[i]), active: true, margin: 0 };
  };
  const clamp = (v: number[]) => v.map((vi, i) => Math.min(opts.uMax?.[i] ?? Infinity, Math.max(opts.uMin?.[i] ?? -Infinity, vi)));
  const iters = constraints.length > 1 ? (opts.iterations ?? 50) : 1;
  let active = constraints.map(() => false), margin = constraints.map(() => 0);
  for (let it = 0; it < iters; it++) {
    for (let k = 0; k < constraints.length; k++) { const r = project(constraints[k], u); u = r.v; active[k] = active[k] || r.active; margin[k] = r.margin; }
    u = clamp(u);
  }
  margin = constraints.map((c) => c.lg.reduce((s, ai, i) => s + ai * u[i], 0) + (c.lf ?? 0) + (c.gamma ?? 1) * c.h);
  return { u, active, margin };
}

/** Circular workspace h = R² − ‖p − c‖² for a single integrator ṗ = u (course §9.6.4). */
export function zoneConstraint(p: [number, number], c: [number, number], R: number, gamma: number): CBFConstraint { const d = [p[0] - c[0], p[1] - c[1]]; return { h: R * R - (d[0] * d[0] + d[1] * d[1]), lg: [-2 * d[0], -2 * d[1]], gamma }; }
/** Minimum distance to a (moving) human: h = ‖p − p_h‖² − d_min²; L_f h = −2(p − p_h)·v_h. */
export function distanceConstraint(p: [number, number], ph: [number, number], dMin: number, gamma: number, vh: [number, number] = [0, 0]): CBFConstraint { const d = [p[0] - ph[0], p[1] - ph[1]]; return { h: d[0] * d[0] + d[1] * d[1] - dMin * dMin, lf: -2 * (d[0] * vh[0] + d[1] * vh[1]), lg: [2 * d[0], 2 * d[1]], gamma }; }

/** Simplex architecture: use the advanced controller's command while the safety monitor accepts it, else the baseline. */
export function simplexSwitch<T>(advanced: T, baseline: T, safe: (u: T) => boolean): { u: T; source: 'advanced' | 'baseline' } { return safe(advanced) ? { u: advanced, source: 'advanced' } : { u: baseline, source: 'baseline' }; }

/** Course mobile-manipulator mode automaton (§9.2.2) with the SLOW / NORMAL hysteresis of §9.2.3. */
export const MOBILE_MANIPULATOR_MODES: HybridSpec = {
  name: 'Mobile manipulator modes',
  initial: 'TRANSPORT',
  vars: { dist_target: 5, dist_human: 10, v: 0, human: false, estop: false, ack: false, grasp_done: false, arm_stowed: true, target_lost: false },
  modes: [
    { id: 'TRANSPORT', invariant: 'arm_stowed && v <= 1.2', dynamics: 'unicycle base, arm fixed', vMax: 1.2 },
    { id: 'SLOW', invariant: 'arm_stowed && v <= 0.3', dynamics: 'reduced speed near a human', vMax: 0.3 },
    { id: 'APPROACH', invariant: 'v <= 0.15', dynamics: 'base position control, arm fixed', vMax: 0.15 },
    { id: 'MANIPULATE', invariant: 'v == 0', dynamics: 'base stopped, arm joint control', vMax: 0 },
    { id: 'STOP', invariant: 'true', dynamics: 'decelerate a_max, arm frozen', vMax: 0 },
  ],
  transitions: [
    { from: 'TRANSPORT', to: 'APPROACH', guard: 'dist_target <= 0.6' },
    { from: 'TRANSPORT', to: 'SLOW', guard: 'dist_human < 2.0', dwell: 0.4 },
    { from: 'SLOW', to: 'TRANSPORT', guard: 'dist_human > 2.5', dwell: 0.4 },
    { from: 'TRANSPORT', to: 'STOP', guard: 'human || estop' },
    { from: 'SLOW', to: 'STOP', guard: 'human || estop' },
    { from: 'APPROACH', to: 'MANIPULATE', guard: 'dist_target <= 0.02 && v <= 0.01' },
    { from: 'APPROACH', to: 'TRANSPORT', guard: 'target_lost' },
    { from: 'APPROACH', to: 'STOP', guard: 'human || estop' },
    { from: 'MANIPULATE', to: 'TRANSPORT', guard: 'grasp_done && arm_stowed' },
    { from: 'STOP', to: 'TRANSPORT', guard: '!human && !estop && ack' },
  ],
};
