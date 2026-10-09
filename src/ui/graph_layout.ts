/**
 * Graph layout shared by the report graph views and the graphical editors: BFS layering from the initial / root
 * nodes, left-to-right tree layout for behavior trees, Fruchterman–Reingold refinement for automata, Petri nets and
 * mode graphs. Pure (no DOM), so it can be tested and reused by the editors for "auto layout".
 */
import type { GraphView } from '../ctl/analysis';

export interface GraphLayout { pos: Map<string, [number, number]>; width: number; height: number; tree: boolean }
export const TREE_BOX = { w: 132, h: 20, col: 156, row: 25 };

/**
 * Lay out `g`. `fixed` pins nodes to given positions (the editor's saved layout); only the remaining nodes are
 * placed, around the pinned ones.
 */
export function layoutGraph(g: GraphView, minWidth = 320, fixed?: Map<string, [number, number]>): GraphLayout {
  const ids = g.nodes.map((n) => n.id); const idx = new Map(ids.map((id, i) => [id, i]));
  const out = new Map<string, string[]>(); const indeg = new Map<string, number>();
  for (const n of g.nodes) { out.set(n.id, []); indeg.set(n.id, 0); }
  for (const e of g.edges) { if (!idx.has(e.from) || !idx.has(e.to)) continue; out.get(e.from)!.push(e.to); if (e.from !== e.to) indeg.set(e.to, (indeg.get(e.to) ?? 0) + 1); }
  let roots = g.nodes.filter((n) => n.initial).map((n) => n.id);
  if (!roots.length) roots = g.nodes.filter((n) => (indeg.get(n.id) ?? 0) === 0).map((n) => n.id);
  if (!roots.length && g.nodes.length) roots = [g.nodes[0].id];
  const level = new Map<string, number>(); const queue = [...roots]; for (const r of roots) level.set(r, 0);
  while (queue.length) { const u = queue.shift()!; for (const v of out.get(u) ?? []) if (!level.has(v)) { level.set(v, level.get(u)! + 1); queue.push(v); } }
  for (const n of g.nodes) if (!level.has(n.id)) level.set(n.id, 0);
  const layers: string[][] = []; for (const n of g.nodes) { const l = level.get(n.id)!; (layers[l] ??= []).push(n.id); }
  const tree = g.kind === 'tree';
  const pos = new Map<string, [number, number]>();
  const rowH = 90; const maxPerRow = Math.max(1, ...layers.map((l) => l.length));
  const W = Math.max(minWidth, maxPerRow * 110);
  if (tree) {
    let slot = 0; const y = new Map<string, number>();
    const place = (u: string): number => { const ch = out.get(u) ?? []; if (!ch.length) { y.set(u, slot++ * TREE_BOX.row + 16); return y.get(u)!; } const ys = ch.map(place); const v = (ys[0] + ys[ys.length - 1]) / 2; y.set(u, v); return v; };
    for (const r of roots) place(r);
    for (const n of g.nodes) if (!y.has(n.id)) y.set(n.id, slot++ * TREE_BOX.row + 16);
    for (const n of g.nodes) pos.set(n.id, [TREE_BOX.w / 2 + 10 + level.get(n.id)! * TREE_BOX.col, y.get(n.id)!]);
    return { pos, width: TREE_BOX.w + 20 + (layers.length - 1) * TREE_BOX.col, height: slot * TREE_BOX.row + 12, tree };
  }
  layers.forEach((l, li) => l.forEach((id, i) => pos.set(id, [(W / (l.length + 1)) * (i + 1) + ((i * 7 + li * 3) % 11) - 5, 40 + li * rowH])));
  const pinned = new Set<string>();
  if (fixed) for (const [id, p] of fixed) if (pos.has(id)) { pos.set(id, [p[0], p[1]]); pinned.add(id); }
  const n = g.nodes.length;
  if (n > 2 && pinned.size < n) {
    // Fruchterman–Reingold refinement seeded by the layering (pinned nodes do not move)
    const P = ids.map((id) => [...pos.get(id)!] as [number, number]); const io = new Map(ids.map((id, i) => [id, i]));
    const E = g.edges.filter((e) => e.from !== e.to && io.has(e.from) && io.has(e.to)).map((e) => [io.get(e.from)!, io.get(e.to)!] as [number, number]);
    const k = n <= 8 ? 165 : 105; let temp = 60;
    for (let it = 0; it < 250; it++) {
      const F: Array<[number, number]> = P.map(() => [0, 0]);
      for (let i = 0; i < n; i++) for (let j = i + 1; j < n; j++) { let dx = P[j][0] - P[i][0], dy = P[j][1] - P[i][1]; let d = Math.hypot(dx, dy); if (d < 1) { dx = 1; dy = ((i + j) % 3) - 1; d = 1; } const f = (k * k) / d; F[i][0] -= (dx / d) * f; F[i][1] -= (dy / d) * f; F[j][0] += (dx / d) * f; F[j][1] += (dy / d) * f; }
      for (const [a, b] of E) { const dx = P[b][0] - P[a][0], dy = P[b][1] - P[a][1]; const d = Math.hypot(dx, dy) || 1; const f = (d * d) / k; F[a][0] += (dx / d) * f; F[a][1] += (dy / d) * f; F[b][0] -= (dx / d) * f; F[b][1] -= (dy / d) * f; }
      for (let i = 0; i < n; i++) { if (pinned.has(ids[i])) continue; const m = Math.hypot(F[i][0], F[i][1]) || 1; const step = Math.min(m, temp); P[i][0] += (F[i][0] / m) * step; P[i][1] += (F[i][1] / m) * step; }
      temp = Math.max(1, temp * 0.97);
    }
    if (pinned.size) { ids.forEach((id, i) => pos.set(id, [Math.max(40, P[i][0]), Math.max(40, P[i][1])])); }
    else { const xs = P.map((q) => q[0]), ys = P.map((q) => q[1]); const x0 = Math.min(...xs), y0 = Math.min(...ys); ids.forEach((id, i) => pos.set(id, [P[i][0] - x0 + 60, P[i][1] - y0 + 40])); }
  }
  const xs = [...pos.values()].map((q) => q[0]), ys = [...pos.values()].map((q) => q[1]);
  return { pos, width: Math.max(minWidth, (xs.length ? Math.max(...xs) : 0) + 60), height: (ys.length ? Math.max(...ys) : 0) + 60, tree };
}
