/** Small seeded random generator (mulberry32) so that every group-control experiment is reproducible. */
export class Rng {
  private s: number;
  constructor(seed = 0) { this.s = (seed >>> 0) || 0x9e3779b9; }
  /** Uniform in [0, 1). */
  random(): number { let t = (this.s += 0x6d2b79f5); t = Math.imul(t ^ (t >>> 15), t | 1); t ^= t + Math.imul(t ^ (t >>> 7), t | 61); return ((t ^ (t >>> 14)) >>> 0) / 4294967296; }
  uniform(a = 0, b = 1): number { return a + (b - a) * this.random(); }
  /** Standard normal (Box–Muller). */
  normal(mean = 0, std = 1): number { let u = 0; while (u === 0) u = this.random(); const v = this.random(); return mean + std * Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v); }
  int(n: number): number { return Math.floor(this.random() * n); }
  choice<T>(xs: T[]): T { return xs[this.int(xs.length)]; }
  shuffle<T>(xs: T[]): T[] { const a = xs.slice(); for (let i = a.length - 1; i > 0; i--) { const j = this.int(i + 1); [a[i], a[j]] = [a[j], a[i]]; } return a; }
  /** k distinct indices from 0..n-1. */
  sample(n: number, k: number): number[] { return this.shuffle([...Array(n).keys()]).slice(0, k); }
  /** Exponential inter-arrival time with the given rate. */
  exponential(rate: number): number { return -Math.log(1 - this.random()) / rate; }
}

export type Vec2 = [number, number];
export const add = (a: Vec2, b: Vec2): Vec2 => [a[0] + b[0], a[1] + b[1]];
export const sub = (a: Vec2, b: Vec2): Vec2 => [a[0] - b[0], a[1] - b[1]];
export const scale = (a: Vec2, k: number): Vec2 => [a[0] * k, a[1] * k];
export const dot = (a: Vec2, b: Vec2): number => a[0] * b[0] + a[1] * b[1];
export const norm = (a: Vec2): number => Math.hypot(a[0], a[1]);
export const dist = (a: Vec2, b: Vec2): number => Math.hypot(a[0] - b[0], a[1] - b[1]);
/** Limit the length of a vector to vmax. */
export const clampNorm = (a: Vec2, vmax: number): Vec2 => { const n = norm(a); return n > vmax && n > 0 ? scale(a, vmax / n) : a; };
export const mean2 = (ps: Vec2[]): Vec2 => { let x = 0, y = 0; for (const p of ps) { x += p[0]; y += p[1]; } return ps.length ? [x / ps.length, y / ps.length] : [0, 0]; };
