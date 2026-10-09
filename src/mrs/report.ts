/** Report helpers shared by the group-control analysers (sections, number formatting, plot builders). */
import type { ReportSection, PlotView, PlotSeries, GraphView } from '../ctl/analysis';
import type { Matrix } from './graph';
import type { Vec2 } from './rng';

export type Metrics = Record<string, number | string | boolean>;
export const f = (v: number, d = 3): string => (Number.isFinite(v) ? Number(v.toFixed(d)).toString() : String(v));
export const sec = (title: string, level: ReportSection['level'], lines: string[] = [], extra: Partial<ReportSection> = {}): ReportSection => ({ title, level, lines, ...extra });
export const lines = (title: string, series: PlotSeries[], xlabel = 'step', ylabel = '', extra: Partial<PlotView> = {}): PlotView => ({ kind: 'lines', title, xlabel, ylabel, series, ...extra });
export const paths = (title: string, series: PlotSeries[], extra: Partial<PlotView> = {}): PlotView => ({ kind: 'paths', title, series, equal: true, ...extra });
export const grid = (title: string, cells: number[][], cellStyle: PlotView['cellStyle'] = 'binary', extra: Partial<PlotView> = {}): PlotView => ({ kind: 'grid', title, cells, cellStyle, ...extra });
export const bars = (title: string, items: Array<{ label: string; value: number }>, ylabel = ''): PlotView => ({ kind: 'bars', title, bars: items, ylabel });
/** Series from a scalar history. */
export const seriesOf = (name: string, ys: number[], x0 = 0, dx = 1): PlotSeries => ({ name, points: ys.map((y, i) => [x0 + i * dx, y]) });
/** One series per robot from a position history (subsampled to ≤ maxPts points). */
export function trajectories(hist: Vec2[][], names: string[], maxPts = 400): PlotSeries[] {
  const n = hist[0]?.length ?? 0; const step = Math.max(1, Math.floor(hist.length / maxPts)); const out: PlotSeries[] = [];
  for (let i = 0; i < n; i++) { const pts: Array<[number, number]> = []; for (let k = 0; k < hist.length; k += step) pts.push([hist[k][i][0], hist[k][i][1]]); const last = hist[hist.length - 1][i]; if (pts[pts.length - 1][0] !== last[0] || pts[pts.length - 1][1] !== last[1]) pts.push([last[0], last[1]]); out.push({ name: names[i] ?? `r${i + 1}`, points: pts }); }
  return out;
}
/** Communication graph as a graph view (undirected; positions pinned when given). */
export function networkView(A: Matrix, names: string[], positions?: Vec2[], labels?: string[]): GraphView {
  const nodes = names.map((nm, i) => ({ id: nm, label: labels?.[i] ?? nm })); const edges: GraphView['edges'] = [];
  for (let i = 0; i < A.length; i++) for (let j = i + 1; j < A.length; j++) if (A[i][j] > 0) edges.push({ from: names[i], to: names[j], label: A[i][j] !== 1 ? f(A[i][j], 2) : undefined });
  const pos = positions ? Object.fromEntries(positions.map((p, i) => [names[i], [p[0], p[1]] as [number, number]])) : undefined;
  return { kind: 'network', nodes, edges, undirected: true, positions: pos };
}
export const mean = (xs: number[]): number => (xs.length ? xs.reduce((s, v) => s + v, 0) / xs.length : 0);
