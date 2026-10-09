/**
 * Reinforcement learning for robot groups (course chapter 7): tabular Q-learning / SARSA on a grid world (the
 * corridor example of §7.2.4), value iteration for the optimum, and independent multi-agent Q-learning (IQL) of two
 * robots that must not enter the same cell.
 */
import { Rng } from './rng';

export interface GridWorld { rows: number; cols: number; walls: Set<string>; goal: [number, number]; start: [number, number]; stepReward: number; goalReward: number; /** actions available: 'lr' (left/right) or 'four' */ actions: Array<[number, number]>; actionNames: string[] }
export function gridWorldFromMap(text: string, o: { stepReward?: number; goalReward?: number; corridor?: boolean } = {}): GridWorld {
  const rows = text.replace(/^\n+|\n+$/g, '').split('\n').map((r) => r.trim()); const walls = new Set<string>(); let goal: [number, number] = [0, 0], start: [number, number] = [0, 0];
  rows.forEach((r, i) => [...r].forEach((ch, j) => { if (ch === '#') walls.add(`${i},${j}`); if (ch === 'G') goal = [i, j]; if (ch === 'S') start = [i, j]; }));
  const corridor = o.corridor ?? rows.length === 1;
  return { rows: rows.length, cols: rows[0].length, walls, goal, start, stepReward: o.stepReward ?? -1, goalReward: o.goalReward ?? 10, actions: corridor ? [[0, -1], [0, 1]] : [[-1, 0], [0, 1], [1, 0], [0, -1]], actionNames: corridor ? ['L', 'R'] : ['U', 'R', 'D', 'L'] };
}
export const sKey = (s: [number, number]) => `${s[0]},${s[1]}`;
export function gwStep(w: GridWorld, s: [number, number], a: number): { s2: [number, number]; r: number; done: boolean } {
  const [dr, dc] = w.actions[a]; let r = s[0] + dr, c = s[1] + dc; if (r < 0 || r >= w.rows || c < 0 || c >= w.cols || w.walls.has(`${r},${c}`)) { r = s[0]; c = s[1]; }
  const done = r === w.goal[0] && c === w.goal[1]; return { s2: [r, c], r: done ? w.goalReward : w.stepReward, done };
}
export type QTable = Map<string, number[]>;
export const qOf = (Q: QTable, s: [number, number], nA: number): number[] => { const k = sKey(s); let q = Q.get(k); if (!q) { q = Array(nA).fill(0); Q.set(k, q); } return q; };
export interface RLOptions { alpha?: number; gamma?: number; epsilon?: number; episodes?: number; maxSteps?: number; sarsa?: boolean; /** deterministic greedy tie-break to the right (the textbook example) */ tieRight?: boolean; /** always take this action (the textbook example repeats the same path in every episode) */ forceAction?: number; rng?: Rng; snapshots?: number[] }
/** Tabular Q-learning (or SARSA); returns the table, per-episode returns and snapshots after selected episodes. */
export function qLearning(w: GridWorld, o: RLOptions = {}): { Q: QTable; returns: number[]; snapshots: Array<{ episode: number; Q: Record<string, number[]> }>; steps: number[] } {
  const alpha = o.alpha ?? 0.5, gamma = o.gamma ?? 0.9, eps = o.epsilon ?? 0, episodes = o.episodes ?? 100, maxSteps = o.maxSteps ?? 200, rng = o.rng ?? new Rng(0); const nA = w.actions.length; const Q: QTable = new Map(); const returns: number[] = []; const steps: number[] = []; const snapshots: Array<{ episode: number; Q: Record<string, number[]> }> = [];
  const greedy = (q: number[]) => { let b = o.tieRight ? nA - 1 : 0; for (let a = 0; a < nA; a++) if (q[a] > q[b] + 1e-12) b = a; return b; };
  const policy = (s: [number, number]) => (o.forceAction !== undefined ? o.forceAction : rng.random() < eps ? rng.int(nA) : greedy(qOf(Q, s, nA)));
  for (let ep = 1; ep <= episodes; ep++) {
    let s = w.start; let a = policy(s); let G = 0, n = 0;
    for (let t = 0; t < maxSteps; t++) {
      const { s2, r, done } = gwStep(w, s, a); G += r; n++; const q = qOf(Q, s, nA); const a2 = policy(s2); const target = done ? r : r + gamma * (o.sarsa ? qOf(Q, s2, nA)[a2] : Math.max(...qOf(Q, s2, nA)));
      q[a] += alpha * (target - q[a]); s = s2; a = a2; if (done) break;
    }
    returns.push(G); steps.push(n); if (o.snapshots?.includes(ep)) snapshots.push({ episode: ep, Q: Object.fromEntries([...Q].map(([k, v]) => [k, v.slice()])) });
  }
  return { Q, returns, snapshots, steps };
}
/** Optimal Q* by value iteration (deterministic transitions). */
export function optimalQ(w: GridWorld, gamma = 0.9): Record<string, number[]> {
  const states: Array<[number, number]> = []; for (let r = 0; r < w.rows; r++) for (let c = 0; c < w.cols; c++) if (!w.walls.has(`${r},${c}`) && !(r === w.goal[0] && c === w.goal[1])) states.push([r, c]);
  const V = new Map<string, number>(states.map((s) => [sKey(s), 0])); const Q: Record<string, number[]> = {};
  for (let it = 0; it < 1000; it++) { let delta = 0; for (const s of states) { const q = w.actions.map((_, a) => { const { s2, r, done } = gwStep(w, s, a); return r + (done ? 0 : gamma * (V.get(sKey(s2)) ?? 0)); }); Q[sKey(s)] = q; const v = Math.max(...q); delta = Math.max(delta, Math.abs(v - (V.get(sKey(s)) ?? 0))); V.set(sKey(s), v); } if (delta < 1e-12) break; }
  return Q;
}
export function greedyPath(w: GridWorld, Q: Record<string, number[]> | QTable, maxSteps = 100): Array<[number, number]> { let s = w.start; const path = [s]; for (let t = 0; t < maxSteps; t++) { const q = Q instanceof Map ? Q.get(sKey(s)) : Q[sKey(s)]; if (!q) break; let b = 0; for (let a = 0; a < q.length; a++) if (q[a] > q[b]) b = a; const { s2, done } = gwStep(w, s, b); s = s2; path.push(s); if (done) break; } return path; }

/** Independent Q-learning of two robots on a small grid: each must reach its own goal; entering the same cell costs a collision penalty. */
export function independentQLearning(w: GridWorld, starts: [[number, number], [number, number]], goals: [[number, number], [number, number]], o: { episodes?: number; alpha?: number; gamma?: number; epsilon?: number; collisionPenalty?: number; rng?: Rng; maxSteps?: number } = {}): { collisionsFirst: number; collisionsLast: number; stepsFirst: number; stepsLast: number; returns: [number[], number[]]; finalCollisionRate: number } {
  const episodes = o.episodes ?? 400, alpha = o.alpha ?? 0.3, gamma = o.gamma ?? 0.9, eps0 = o.epsilon ?? 0.2, pen = o.collisionPenalty ?? -5, rng = o.rng ?? new Rng(1), maxSteps = o.maxSteps ?? 40; const nA = w.actions.length;
  const Q: [QTable, QTable] = [new Map(), new Map()]; const returns: [number[], number[]] = [[], []]; const collisions: number[] = []; const stepsPer: number[] = [];
  const worlds = goals.map((g) => ({ ...w, goal: g })) as [GridWorld, GridWorld];
  for (let ep = 0; ep < episodes; ep++) {
    const eps = eps0 * (1 - ep / episodes); let s: [[number, number], [number, number]] = [starts[0], starts[1]]; const done = [false, false]; const G = [0, 0]; let col = 0, n = 0;
    for (let t = 0; t < maxSteps && !(done[0] && done[1]); t++) {
      n++;
      const acts = [0, 1].map((i) => { if (done[i]) return -1; const q = qOf(Q[i], s[i], nA); if (rng.random() < eps) return rng.int(nA); let b = 0; for (let a = 0; a < nA; a++) if (q[a] > q[b]) b = a; return b; });
      const next = [0, 1].map((i) => (done[i] ? { s2: s[i], r: 0, done: true } : gwStep(worlds[i], s[i], acts[i]))) as Array<{ s2: [number, number]; r: number; done: boolean }>;
      const clash = !done[0] && !done[1] && next[0].s2[0] === next[1].s2[0] && next[0].s2[1] === next[1].s2[1]; if (clash) col++;
      for (const i of [0, 1]) { if (done[i]) continue; const r = next[i].r + (clash ? pen : 0); const q = qOf(Q[i], s[i], nA); const target = next[i].done ? r : r + gamma * Math.max(...qOf(Q[i], next[i].s2, nA)); q[acts[i]] += alpha * (target - q[acts[i]]); G[i] += r; if (!clash) { s[i] = next[i].s2; done[i] = next[i].done; } }
    }
    returns[0].push(G[0]); returns[1].push(G[1]); collisions.push(col); stepsPer.push(n);
  }
  const k = Math.max(1, Math.floor(episodes / 10)); const avg = (xs: number[]) => xs.reduce((a, b) => a + b, 0) / xs.length;
  return { collisionsFirst: avg(collisions.slice(0, k)), collisionsLast: avg(collisions.slice(-k)), stepsFirst: avg(stepsPer.slice(0, k)), stepsLast: avg(stepsPer.slice(-k)), returns, finalCollisionRate: avg(collisions.slice(-k)) };
}
