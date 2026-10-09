/**
 * Group simulator on the plane (course practicum library `urrts/sim.py`): single integrators ṗᵢ = uᵢ with a speed
 * limit, a radio model (range + independent edge loss per step), robot failures and actuation noise. The control
 * law is a function (t, p, A, state) → u; the history records positions, commands and the number of links.
 */
import { Rng, Vec2, clampNorm } from './rng';
import { Matrix, diskGraph, edgeCount } from './graph';

export class CommModel {
  constructor(public radius = Infinity, public dropProb = 0, public rng: Rng = new Rng(0)) {}
  adjacency(p: Vec2[], alive?: boolean[]): Matrix {
    const A = diskGraph(p, this.radius, alive);
    if (this.dropProb > 0) for (let i = 0; i < A.length; i++) for (let j = i + 1; j < A.length; j++) if (A[i][j] > 0 && this.rng.random() < this.dropProb) { A[i][j] = A[j][i] = 0; }
    return A;
  }
}

export interface History { t: number[]; p: Vec2[][]; u: Vec2[][]; edges: number[] }
export type Controller = (t: number, p: Vec2[], A: Matrix, state: Record<string, unknown>) => Vec2[];

export interface SimOptions { dt?: number; comm?: CommModel; vmax?: number; failures?: Record<number, number[]>; noiseStd?: number; rng?: Rng; recordEvery?: number; state?: Record<string, unknown> }

export function simulate(p0: Vec2[], controller: Controller, steps: number, o: SimOptions = {}): { hist: History; alive: boolean[]; state: Record<string, unknown> } {
  const dt = o.dt ?? 0.05, rng = o.rng ?? new Rng(0), comm = o.comm ?? new CommModel(); const rec = o.recordEvery ?? 1;
  let p = p0.map((q) => [q[0], q[1]] as Vec2); const n = p.length; const alive = Array(n).fill(true); const state = o.state ?? {}; state.alive = alive;
  const hist: History = { t: [], p: [], u: [], edges: [] };
  for (let k = 0; k < steps; k++) {
    if (o.failures && o.failures[k]) for (const i of o.failures[k]) alive[i] = false;
    const A = comm.adjacency(p, alive);
    let u = controller(k * dt, p.map((q) => [q[0], q[1]] as Vec2), A, state);
    u = u.map((v, i) => { if (!alive[i]) return [0, 0] as Vec2; let w: Vec2 = o.vmax !== undefined && Number.isFinite(o.vmax) ? clampNorm(v, o.vmax) : v; if (o.noiseStd) w = [w[0] + rng.normal(0, o.noiseStd), w[1] + rng.normal(0, o.noiseStd)]; return w; });
    if (k % rec === 0) { hist.t.push(k * dt); hist.p.push(p.map((q) => [q[0], q[1]] as Vec2)); hist.u.push(u); hist.edges.push(edgeCount(A)); }
    p = p.map((q, i) => [q[0] + dt * u[i][0], q[1] + dt * u[i][1]] as Vec2);
  }
  hist.t.push(steps * dt); hist.p.push(p); hist.u.push(p.map(() => [0, 0] as Vec2)); hist.edges.push(edgeCount(comm.adjacency(p, alive)));
  return { hist, alive, state };
}

/** Desired point velocity (ux, uy) → (v, ω) of a differential drive through a point l ahead of the axle (§4.2.1). */
export function unicycleFromVelocity(theta: number, u: Vec2, l = 0.03): { v: number; w: number } { const c = Math.cos(theta), s = Math.sin(theta); return { v: c * u[0] + s * u[1], w: (-s * u[0] + c * u[1]) / l }; }
/** (v, ω) → wheel angular speeds of an e-puck-like robot, proportionally saturated. */
export function wheelSpeeds(v: number, w: number, wheelRadius = 0.0205, axle = 0.052, maxSpeed = 6.28): { left: number; right: number } {
  let left = (v - (w * axle) / 2) / wheelRadius, right = (v + (w * axle) / 2) / wheelRadius; const m = Math.max(Math.abs(left), Math.abs(right), 1e-12);
  if (m > maxSpeed) { left *= maxSpeed / m; right *= maxSpeed / m; } return { left, right };
}
