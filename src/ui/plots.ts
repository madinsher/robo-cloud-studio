/**
 * SVG charts for analysis reports (PlotView): time series, planar trajectories with markers and obstacles,
 * cell grids (binary / heat / labels) and bars. Theme colours come from CSS variables; `standaloneSvg` in
 * control_ui.ts inlines them for export.
 */
import type { PlotView, PlotSeries } from '../ctl/analysis';

const NS = 'http://www.w3.org/2000/svg';
const PALETTE = ['#4dabf7', '#51cf66', '#fab005', '#ff6b6b', '#cc5de8', '#22b8cf', '#ff922b', '#94d82d', '#f06595', '#748ffc', '#20c997', '#e599f7'];
const el = (tag: string, attrs: Record<string, string | number> = {}, text?: string) => { const e = document.createElementNS(NS, tag); for (const [k, v] of Object.entries(attrs)) e.setAttribute(k, String(v)); if (text !== undefined) e.textContent = text; return e; };
const fmtTick = (v: number) => (Math.abs(v) >= 1000 ? v.toFixed(0) : Math.abs(v) >= 10 ? v.toFixed(1).replace(/\.0$/, '') : Math.abs(v) >= 1 ? v.toFixed(2).replace(/\.?0+$/, '') : v.toFixed(3).replace(/\.?0+$/, '') || '0');
function ticks(lo: number, hi: number, n = 5): number[] { if (!(hi > lo)) return [lo]; const raw = (hi - lo) / n; const p = 10 ** Math.floor(Math.log10(raw)); const m = raw / p; const step = (m >= 5 ? 10 : m >= 2 ? 5 : m >= 1 ? 2 : 1) * p; const out: number[] = []; for (let v = Math.ceil(lo / step) * step; v <= hi + 1e-9; v += step) out.push(Number(v.toFixed(10))); return out; }

export function renderPlot(p: PlotView): SVGSVGElement {
  if (p.kind === 'grid') return renderGrid(p);
  if (p.kind === 'bars') return renderBars(p);
  const W = p.width ?? 560, H = p.height ?? (p.kind === 'paths' ? 360 : 240); const m = { l: 52, r: 12, t: p.title ? 22 : 8, b: p.xlabel ? 34 : 22 };
  const series = (p.series ?? []).filter((s) => s.points.length); const pts = series.flatMap((s) => s.points); const mk = p.markers ?? []; const obs = p.obstacles ?? [];
  const xs = [...pts.map((q) => q[0]), ...mk.map((q) => q.x), ...obs.flatMap((o) => [o.x, o.x + o.w])], ys = [...pts.map((q) => q[1]), ...mk.map((q) => q.y), ...obs.flatMap((o) => [o.y, o.y + o.h])];
  let x0 = Math.min(...xs), x1 = Math.max(...xs), y0 = Math.min(...ys), y1 = Math.max(...ys); if (!Number.isFinite(x0)) { x0 = 0; x1 = 1; y0 = 0; y1 = 1; }
  if (p.ylog) { const pos = ys.filter((v) => v > 0); y0 = Math.min(...pos, 1e-12); y1 = Math.max(...pos, y0 * 10); }
  if (x1 - x0 < 1e-12) { x0 -= 0.5; x1 += 0.5; } if (y1 - y0 < 1e-12) { y0 -= 0.5; y1 += 0.5; }
  const padX = (x1 - x0) * 0.04, padY = p.ylog ? 0 : (y1 - y0) * 0.06; x0 -= padX; x1 += padX; if (!p.ylog) { y0 -= padY; y1 += padY; }
  const pw = W - m.l - m.r, ph = H - m.t - m.b;
  if (p.equal) { const sx = pw / (x1 - x0), sy = ph / (y1 - y0); const s = Math.min(sx, sy); const cx = (x0 + x1) / 2, cy = (y0 + y1) / 2; x0 = cx - pw / s / 2; x1 = cx + pw / s / 2; y0 = cy - ph / s / 2; y1 = cy + ph / s / 2; }
  const X = (v: number) => m.l + ((v - x0) / (x1 - x0)) * pw; const Y = (v: number) => (p.ylog ? m.t + ph - ((Math.log10(Math.max(v, y0)) - Math.log10(y0)) / (Math.log10(y1) - Math.log10(y0))) * ph : m.t + ph - ((v - y0) / (y1 - y0)) * ph);
  const svg = el('svg', { class: 'ctl-graph ctl-plot', viewBox: `0 0 ${W} ${H}`, width: W, height: H }) as SVGSVGElement;
  if (p.title) svg.appendChild(el('text', { x: m.l, y: 14, class: 'plot-title' }, p.title));
  // axes and grid
  const tx = ticks(x0, x1), ty = p.ylog ? ticks(Math.log10(y0), Math.log10(y1), 4).map((e) => 10 ** e) : ticks(y0, y1);
  for (const v of tx) { svg.appendChild(el('line', { x1: X(v), x2: X(v), y1: m.t, y2: m.t + ph, class: 'plot-grid' })); svg.appendChild(el('text', { x: X(v), y: m.t + ph + 12, 'text-anchor': 'middle', class: 'plot-tick' }, fmtTick(v))); }
  for (const v of ty) { svg.appendChild(el('line', { x1: m.l, x2: m.l + pw, y1: Y(v), y2: Y(v), class: 'plot-grid' })); svg.appendChild(el('text', { x: m.l - 4, y: Y(v) + 3, 'text-anchor': 'end', class: 'plot-tick' }, p.ylog ? v.toExponential(0) : fmtTick(v))); }
  svg.appendChild(el('rect', { x: m.l, y: m.t, width: pw, height: ph, class: 'plot-frame' }));
  if (p.xlabel) svg.appendChild(el('text', { x: m.l + pw / 2, y: H - 4, 'text-anchor': 'middle', class: 'plot-label' }, p.xlabel));
  if (p.ylabel) svg.appendChild(el('text', { x: 10, y: m.t + ph / 2, 'text-anchor': 'middle', class: 'plot-label', transform: `rotate(-90 10 ${m.t + ph / 2})` }, p.ylabel));
  const clip = el('clipPath', { id: 'plot-clip-' + Math.random().toString(36).slice(2, 8) }); clip.appendChild(el('rect', { x: m.l, y: m.t, width: pw, height: ph })); const defs = el('defs'); defs.appendChild(clip); svg.appendChild(defs);
  const g = el('g', { 'clip-path': `url(#${clip.id})` }); svg.appendChild(g);
  for (const o of obs) g.appendChild(el('rect', { x: X(o.x), y: Y(o.y + o.h), width: Math.max(1, X(o.x + o.w) - X(o.x)), height: Math.max(1, Y(o.y) - Y(o.y + o.h)), class: 'plot-obstacle' }));
  series.forEach((s, i) => { const c = s.color ?? PALETTE[i % PALETTE.length]; const d = s.points.map((q, k) => `${k ? 'L' : 'M'} ${X(q[0]).toFixed(1)} ${Y(q[1]).toFixed(1)}`).join(' '); g.appendChild(el('path', { d, fill: 'none', stroke: c, 'stroke-width': p.kind === 'paths' ? 1.6 : 1.5, 'stroke-dasharray': s.dashed ? '5 4' : '', class: 'plot-line' })); if (s.markers || s.points.length <= 12) for (const q of s.points) g.appendChild(el('circle', { cx: X(q[0]), cy: Y(q[1]), r: 2.5, fill: c })); if (p.kind === 'paths') { const q0 = s.points[0], q1 = s.points[s.points.length - 1]; g.appendChild(el('circle', { cx: X(q0[0]), cy: Y(q0[1]), r: 3, fill: 'none', stroke: c })); g.appendChild(el('circle', { cx: X(q1[0]), cy: Y(q1[1]), r: 4, fill: c })); } });
  for (const k of mk) { const x = X(k.x), y = Y(k.y); const kind = k.kind ?? 'goal'; if (kind === 'goal') g.appendChild(el('path', { d: `M ${x - 5} ${y - 5} L ${x + 5} ${y + 5} M ${x - 5} ${y + 5} L ${x + 5} ${y - 5}`, class: 'plot-marker goal' })); else if (kind === 'source') g.appendChild(el('path', { d: `M ${x} ${y - 7} L ${x + 6} ${y + 4} L ${x - 6} ${y + 4} Z`, class: 'plot-marker source' })); else if (kind === 'station' || kind === 'task') g.appendChild(el('rect', { x: x - 4, y: y - 4, width: 8, height: 8, class: `plot-marker ${kind}` })); else if (kind === 'obstacle') g.appendChild(el('circle', { cx: x, cy: y, r: 5, class: 'plot-marker obstacle' })); else g.appendChild(el('circle', { cx: x, cy: y, r: 4, class: `plot-marker ${kind}` })); if (k.label) g.appendChild(el('text', { x: x + 6, y: y - 5, class: 'plot-tick' }, k.label)); }
  // legend
  if (series.length > 1 && series.length <= 12 && p.kind !== 'paths') { series.forEach((s, i) => { const y = m.t + 12 + i * 13; svg.appendChild(el('line', { x1: W - m.r - 90, x2: W - m.r - 72, y1: y - 4, y2: y - 4, stroke: s.color ?? PALETTE[i % PALETTE.length], 'stroke-width': 2, 'stroke-dasharray': s.dashed ? '5 4' : '' })); svg.appendChild(el('text', { x: W - m.r - 68, y, class: 'plot-tick' }, s.name.length > 14 ? s.name.slice(0, 13) + '…' : s.name)); }); }
  svg.appendChild(el('title', {}, p.title ?? 'chart'));
  return svg;
}
function renderGrid(p: PlotView): SVGSVGElement {
  const cells = p.cells ?? [[0]]; const R = cells.length, C = Math.max(...cells.map((r) => r.length)); const size = Math.max(3, Math.min(14, Math.floor(540 / Math.max(C, 1)), Math.floor(360 / Math.max(R, 1)))); const W = C * size + 8, H = R * size + (p.title ? 26 : 8);
  const svg = el('svg', { class: 'ctl-graph ctl-plot', viewBox: `0 0 ${W} ${H}`, width: W, height: H }) as SVGSVGElement; const top = p.title ? 22 : 4; if (p.title) svg.appendChild(el('text', { x: 4, y: 14, class: 'plot-title' }, p.title));
  const flat = cells.flat(); const max = Math.max(...flat, 1e-9);
  cells.forEach((row, r) => row.forEach((v, c) => { let fill: string; if (p.cellStyle === 'labels') fill = v > 0 ? PALETTE[(v - 1) % PALETTE.length] : 'transparent'; else if (p.cellStyle === 'heat') { const t = Math.min(1, v / max); fill = `rgba(250, 176, 5, ${(0.08 + 0.92 * t).toFixed(3)})`; } else fill = v ? 'var(--text, #e6e9ef)' : 'transparent'; svg.appendChild(el('rect', { x: 4 + c * size, y: top + r * size, width: size - (size > 4 ? 1 : 0), height: size - (size > 4 ? 1 : 0), fill, class: 'plot-cell' })); }));
  return svg;
}
function renderBars(p: PlotView): SVGSVGElement {
  const items = p.bars ?? []; const W = p.width ?? Math.max(240, 60 * items.length + 60), H = p.height ?? 200; const m = { l: 44, r: 8, t: p.title ? 24 : 8, b: 28 }; const pw = W - m.l - m.r, ph = H - m.t - m.b; const max = Math.max(...items.map((i) => i.value), 1e-9), min = Math.min(0, ...items.map((i) => i.value)); const Y = (v: number) => m.t + ph - ((v - min) / (max - min)) * ph;
  const svg = el('svg', { class: 'ctl-graph ctl-plot', viewBox: `0 0 ${W} ${H}`, width: W, height: H }) as SVGSVGElement; if (p.title) svg.appendChild(el('text', { x: m.l, y: 14, class: 'plot-title' }, p.title));
  for (const v of ticks(min, max, 4)) { svg.appendChild(el('line', { x1: m.l, x2: m.l + pw, y1: Y(v), y2: Y(v), class: 'plot-grid' })); svg.appendChild(el('text', { x: m.l - 4, y: Y(v) + 3, 'text-anchor': 'end', class: 'plot-tick' }, fmtTick(v))); }
  const bw = pw / Math.max(items.length, 1); items.forEach((it, i) => { const x = m.l + i * bw + bw * 0.15; svg.appendChild(el('rect', { x, y: Math.min(Y(it.value), Y(0)), width: bw * 0.7, height: Math.abs(Y(0) - Y(it.value)), fill: PALETTE[i % PALETTE.length] })); svg.appendChild(el('text', { x: x + bw * 0.35, y: m.t + ph + 12, 'text-anchor': 'middle', class: 'plot-tick' }, it.label)); svg.appendChild(el('text', { x: x + bw * 0.35, y: Y(it.value) - 3, 'text-anchor': 'middle', class: 'plot-tick' }, fmtTick(it.value))); });
  return svg;
}

export type { PlotSeries };
