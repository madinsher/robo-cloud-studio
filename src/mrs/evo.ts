/**
 * Evolutionary computation and adaptation (course chapter 12): a binary genetic algorithm (the x² example of
 * §12.2.6 with roulette selection, one-point crossover and bit mutation), differential evolution and a (μ, λ)
 * evolution strategy with self-adaptive step, used to tune group-control parameters.
 */
import { Rng } from './rng';

export const bitsToInt = (b: string) => parseInt(b, 2);
export const intToBits = (x: number, n: number) => x.toString(2).padStart(n, '0');
export interface GAOptions { bits?: number; popSize?: number; generations?: number; pc?: number; pm?: number; elitism?: boolean; rng?: Rng; initial?: string[] }
export interface GAGeneration { gen: number; population: string[]; fitness: number[]; best: string; bestFitness: number; mean: number }
/** Roulette-wheel selection probabilities and expected copies (the numbers of §12.2.6). */
export function rouletteTable(pop: string[], fit: number[]): Array<{ individual: string; x: number; f: number; p: number; expected: number }> { const sum = fit.reduce((s, v) => s + v, 0) || 1; return pop.map((b, i) => ({ individual: b, x: bitsToInt(b), f: fit[i], p: fit[i] / sum, expected: (fit[i] / sum) * pop.length })); }
export function onePointCrossover(a: string, b: string, point: number): [string, string] { return [a.slice(0, point) + b.slice(point), b.slice(0, point) + a.slice(point)]; }
export function mutateBit(a: string, i: number): string { return a.slice(0, i) + (a[i] === '1' ? '0' : '1') + a.slice(i + 1); }
/** Binary GA maximising fitness(x) over integers coded on `bits` bits. */
export function geneticAlgorithm(fitness: (x: number) => number, o: GAOptions = {}): { history: GAGeneration[]; best: string; bestX: number; bestFitness: number } {
  const bits = o.bits ?? 5, popSize = o.popSize ?? 4, gens = o.generations ?? 10, pc = o.pc ?? 0.9, pm = o.pm ?? 0.02, rng = o.rng ?? new Rng(0);
  let pop = o.initial?.slice() ?? Array.from({ length: popSize }, () => intToBits(rng.int(1 << bits), bits)); const history: GAGeneration[] = [];
  const evalPop = (P: string[]) => P.map((b) => fitness(bitsToInt(b)));
  let fit = evalPop(pop); const record = (g: number) => { let bi = 0; for (let i = 1; i < pop.length; i++) if (fit[i] > fit[bi]) bi = i; history.push({ gen: g, population: pop.slice(), fitness: fit.slice(), best: pop[bi], bestFitness: fit[bi], mean: fit.reduce((s, v) => s + v, 0) / fit.length }); };
  record(0);
  for (let g = 1; g <= gens; g++) {
    const sum = fit.reduce((s, v) => s + v, 0) || 1; const pick = () => { let r = rng.random() * sum; for (let i = 0; i < pop.length; i++) { r -= fit[i]; if (r <= 0) return pop[i]; } return pop[pop.length - 1]; };
    const next: string[] = []; if (o.elitism) { let bi = 0; for (let i = 1; i < pop.length; i++) if (fit[i] > fit[bi]) bi = i; next.push(pop[bi]); }
    while (next.length < pop.length) { let a = pick(), b = pick(); if (rng.random() < pc) [a, b] = onePointCrossover(a, b, 1 + rng.int(bits - 1)); for (const c of [a, b]) { let m = c; for (let i = 0; i < bits; i++) if (rng.random() < pm) m = mutateBit(m, i); if (next.length < pop.length) next.push(m); } }
    pop = next; fit = evalPop(pop); record(g);
  }
  let bi = 0; for (let i = 1; i < pop.length; i++) if (fit[i] > fit[bi]) bi = i; const bestHist = history.reduce((b, h) => (h.bestFitness > b.bestFitness ? h : b), history[0]);
  return { history, best: bestHist.best, bestX: bitsToInt(bestHist.best), bestFitness: bestHist.bestFitness };
}
/** Differential evolution (DE/rand/1/bin) minimising f over a box. */
export function differentialEvolution(f: (x: number[]) => number, lo: number[], hi: number[], o: { popSize?: number; generations?: number; F?: number; CR?: number; rng?: Rng } = {}): { best: number[]; value: number; history: number[]; evaluations: number } {
  const d = lo.length, NP = o.popSize ?? 20, gens = o.generations ?? 100, F = o.F ?? 0.8, CR = o.CR ?? 0.9, rng = o.rng ?? new Rng(0); let evals = 0;
  let pop = Array.from({ length: NP }, () => lo.map((l, k) => rng.uniform(l, hi[k]))); let val = pop.map((x) => { evals++; return f(x); }); const history: number[] = [Math.min(...val)];
  for (let g = 0; g < gens; g++) {
    for (let i = 0; i < NP; i++) { const [a, b, c] = rng.sample(NP, 3); const jr = rng.int(d); const y = pop[i].map((xi, k) => (rng.random() < CR || k === jr ? Math.min(hi[k], Math.max(lo[k], pop[a][k] + F * (pop[b][k] - pop[c][k]))) : xi)); const fy = f(y); evals++; if (fy <= val[i]) { pop[i] = y; val[i] = fy; } }
    history.push(Math.min(...val));
  }
  let bi = 0; for (let i = 1; i < NP; i++) if (val[i] < val[bi]) bi = i; return { best: pop[bi], value: val[bi], history, evaluations: evals };
}
/** (μ, λ) evolution strategy with a self-adaptive log-normal step size, minimising f. */
export function evolutionStrategy(f: (x: number[]) => number, x0: number[], o: { mu?: number; lambda?: number; generations?: number; sigma0?: number; rng?: Rng; lo?: number[]; hi?: number[] } = {}): { best: number[]; value: number; history: number[]; sigmaHistory: number[] } {
  const mu = o.mu ?? 3, lambda = o.lambda ?? 12, gens = o.generations ?? 60, rng = o.rng ?? new Rng(0); const d = x0.length; const tau = 1 / Math.sqrt(2 * d);
  let parents = Array.from({ length: mu }, () => ({ x: x0.slice(), s: o.sigma0 ?? 0.3 })); const history: number[] = [f(x0)]; const sigmaHistory: number[] = [o.sigma0 ?? 0.3]; let best = { x: x0.slice(), v: f(x0) };
  for (let g = 0; g < gens; g++) {
    const kids = Array.from({ length: lambda }, () => { const p = rng.choice(parents); const s = p.s * Math.exp(tau * rng.normal()); const x = p.x.map((xi, k) => { let v = xi + s * rng.normal(); if (o.lo) v = Math.max(o.lo[k], v); if (o.hi) v = Math.min(o.hi[k], v); return v; }); return { x, s, v: f(x) }; }).sort((a, b) => a.v - b.v);
    parents = kids.slice(0, mu); if (kids[0].v < best.v) best = { x: kids[0].x, v: kids[0].v }; history.push(best.v); sigmaHistory.push(parents.reduce((s, p) => s + p.s, 0) / mu);
  }
  return { best: best.x, value: best.v, history, sigmaHistory };
}
/** Benchmark functions for the evolutionary kinds. */
export const BENCHMARKS: Record<string, { f: (x: number[]) => number; lo: number; hi: number; optimum: number }> = {
  sphere: { f: (x) => x.reduce((s, v) => s + v * v, 0), lo: -5, hi: 5, optimum: 0 },
  rastrigin: { f: (x) => 10 * x.length + x.reduce((s, v) => s + v * v - 10 * Math.cos(2 * Math.PI * v), 0), lo: -5.12, hi: 5.12, optimum: 0 },
  rosenbrock: { f: (x) => { let s = 0; for (let i = 0; i + 1 < x.length; i++) s += 100 * (x[i + 1] - x[i] * x[i]) ** 2 + (1 - x[i]) ** 2; return s; }, lo: -2, hi: 2, optimum: 0 },
};
