/**
 * Cellular automata as a model of spatially distributed dynamics (course §4.6): elementary Wolfram rules (rule 90
 * → Sierpiński triangle, rule 30 → chaos) and Conway's Game of Life (glider detection), plus a stigmergy-style
 * "pheromone" diffusion automaton used by swarm foraging.
 */
export function wolframStep(row: number[], rule: number): number[] { const n = row.length; return row.map((_, i) => { const l = row[(i - 1 + n) % n], c = row[i], r = row[(i + 1) % n]; return (rule >> ((l << 2) | (c << 1) | r)) & 1; }); }
export function wolframRun(rule: number, width: number, steps: number, init?: number[]): number[][] { let row = init ?? Array.from({ length: width }, (_, i) => (i === Math.floor(width / 2) ? 1 : 0)); const out = [row]; for (let t = 0; t < steps; t++) { row = wolframStep(row, rule); out.push(row); } return out; }
/** Fraction of live cells per generation and whether the pattern is symmetric (rule 90 is, rule 30 is not). */
export function wolframStats(space: number[][]): { density: number[]; symmetric: boolean; distinctRows: number } {
  const density = space.map((r) => r.reduce((s, v) => s + v, 0) / r.length); const symmetric = space.every((r) => r.every((v, i) => v === r[r.length - 1 - i]));
  return { density, symmetric, distinctRows: new Set(space.map((r) => r.join(''))).size };
}
export function lifeStep(g: number[][], torus = true): number[][] {
  const R = g.length, C = g[0].length;
  return g.map((row, r) => row.map((v, c) => { let n = 0; for (let dr = -1; dr <= 1; dr++) for (let dc = -1; dc <= 1; dc++) { if (!dr && !dc) continue; let rr = r + dr, cc = c + dc; if (torus) { rr = (rr + R) % R; cc = (cc + C) % C; } else if (rr < 0 || rr >= R || cc < 0 || cc >= C) continue; n += g[rr][cc]; } return v ? (n === 2 || n === 3 ? 1 : 0) : n === 3 ? 1 : 0; }));
}
export function lifeFromText(text: string): number[][] { return text.replace(/^\n+|\n+$/g, '').split('\n').map((r) => [...r.trim()].map((ch) => (ch === '#' || ch === 'O' || ch === '1' ? 1 : 0))); }
export const GLIDER = '.#.\n..#\n###';
/** Run Life; detects the period and the translation of a moving pattern (a glider: period 4, shift (1, 1)). */
export function lifeRun(g0: number[][], steps: number): { grids: number[][][]; population: number[]; period: number | null; shift: [number, number] | null } {
  let g = g0.map((r) => r.slice()); const grids = [g]; const population = [g.flat().reduce((s, v) => s + v, 0)];
  const sig = (m: number[][]) => { const cells: Array<[number, number]> = []; m.forEach((r, i) => r.forEach((v, j) => { if (v) cells.push([i, j]); })); if (!cells.length) return { key: '', r: 0, c: 0 }; const r0 = Math.min(...cells.map((x) => x[0])), c0 = Math.min(...cells.map((x) => x[1])); return { key: cells.map(([i, j]) => `${i - r0},${j - c0}`).sort().join(';'), r: r0, c: c0 }; };
  const s0 = sig(g); let period: number | null = null, shift: [number, number] | null = null;
  for (let t = 1; t <= steps; t++) { g = lifeStep(g); grids.push(g); population.push(g.flat().reduce((s, v) => s + v, 0)); if (period === null) { const s = sig(g); if (s.key === s0.key && s.key) { period = t; shift = [s.r - s0.r, s.c - s0.c]; } } }
  return { grids, population, period, shift };
}
/** Pheromone field (stigmergy, §5.5): deposit at robot cells, evaporate by rho, diffuse with weight kappa. */
export function pheromoneStep(field: number[][], deposits: Array<[number, number]>, rho = 0.05, kappa = 0.1, amount = 1): number[][] {
  const R = field.length, C = field[0].length; const f = field.map((r) => r.slice()); for (const [r, c] of deposits) if (r >= 0 && r < R && c >= 0 && c < C) f[r][c] += amount;
  return f.map((row, r) => row.map((v, c) => { let s = 0, n = 0; for (const [dr, dc] of [[1, 0], [-1, 0], [0, 1], [0, -1]]) { const rr = r + dr, cc = c + dc; if (rr >= 0 && rr < R && cc >= 0 && cc < C) { s += f[rr][cc]; n++; } } return (1 - rho) * ((1 - kappa) * v + (kappa * s) / Math.max(1, n)); }));
}
