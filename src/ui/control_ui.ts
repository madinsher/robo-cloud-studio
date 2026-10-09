/**
 * Control tab: design the upper control level of a robot / a complex of robots with the course methods —
 * model documents (automata + supervisory control, Petri nets, behavior trees, statecharts, GR(1), mode automata,
 * PDDL, MDP/POMDP, task allocation, MAPF, scheduling, real-time, reliability, V&V), analysis reports with graph
 * views, and a mission runtime that executes a behavior tree on a station robot under a synthesised supervisor
 * with LTL3 monitors and a mode machine.
 */
import type { App } from '../app';
import { h, clear, dialog, toast, downloadText, contextMenu, MenuEntry, fmt } from './dom';
import { ItemType, Item } from '../core/items/item';
import { MobileRobot } from '../mobile/items';
import { FleetItem } from '../fleet/fleet';
import { ControlModelItem, controlModels, addControlModel, CONTROL_KINDS, ControlKind, GROUP_LABELS } from '../ctl/model';
import { isMrsKind, MrsKind } from '../mrs/model';
import { MRS_EXAMPLES, MRS_PARTS } from '../mrs/examples';
import { FleetRuntime, RUNNABLE_KINDS, buildWarehouseScene } from '../mrs/runtime';
import { parseWarehouse } from '../mrs/dsl';
import { renderPlot } from './plots';
import { TEMPLATES, parseBt, parseDes, parsePetri, parseHybrid, detectKind } from '../ctl/dsl';
import { analyse, Report, GraphView } from '../ctl/analysis';
import { COURSE_EXAMPLES } from '../ctl/examples_dsl';
import { ControlRuntime, stationBindings, hostBindings, WorldBindings } from '../ctl/runtime';
import { AutomatonExecutor, PetriExecutor, ExecutorWorld } from '../ctl/exec';
import { Robot } from '../core/items/robot';
import { Program } from '../core/items/program';
import { Target } from '../core/items/item';
import { ZoneItem } from '../mobile/items';
import { planMoveJ, planMoveL } from '../core/motion/trajectory';
import { DES, parallel, supcon, supervisorTable, supervisorPython, SupervisorTable } from '../ctl/des';
import { t } from './i18n';
import { layoutGraph, TREE_BOX } from './graph_layout';
import { buildGraphEditor, EditorKind, ActionCatalog } from './graph_editor';
import { buildBtEditor } from './bt_editor';

const LEVEL_ICON = { ok: '✅', warn: '⚠️', error: '❌', info: 'ℹ️' } as const;
const kindLabel = (k: ControlKind) => CONTROL_KINDS.find((x) => x.kind === k)?.label ?? k;
/** "(ch. 4)" for the control-design kinds, "(ПР1, MRS 4, 13, 16)" for the group-control kinds. */
const kindTag = (k: { chapter: string; practicum?: string; group: string }) => (k.group === 'multi-robot' ? `${k.practicum && k.practicum !== '—' ? k.practicum + ', ' : ''}${k.chapter}` : `ch. ${k.chapter}`);
const kv = (k: string, v: string) => h('div', { class: 'kv-row' }, h('span', { class: 'k' }, t(k)), h('span', { class: 'v' }, v));

/** Synthesise the supervisor table of a DES document (null when the specification is unrealisable). */
export function supervisorFromDes(source: string): SupervisorTable | null {
  const doc = parseDes(source);
  if (!doc.plant.length) return null;
  const g = parallel(...doc.plant.map((a) => DES.fromSpec(a)));
  const r = supcon(g, doc.specs.map((a) => DES.fromSpec(a)), doc.uncontrollable);
  return r.realizable ? supervisorTable(r.supervisor, doc.uncontrollable, doc.unobservable, doc.plant.length) : null;
}

/** Run the analysis of a model and store the verdict on the item. */
export function analyseModel(app: App, m: ControlModelItem): Report {
  const r = analyse(m.kind, m.source);
  m.lastReport = r.markdown; m.lastOk = r.ok; m.notify('report');
  app.log(`[control] ${m.name}: ${r.ok ? 'OK' : 'issues found'}${r.error ? ` — ${r.error}` : ''} (${r.durationMs} ms)`, r.ok ? 'info' : 'warn');
  return r;
}

/** Add a fresh model from a template and select it. */
export function newControlModel(app: App, kind: ControlKind, name?: string, source?: string): ControlModelItem {
  const m = app.cmd(() => addControlModel(app.station, kind, name ?? kindLabel(kind), source ?? TEMPLATES[kind]));
  app.select(m);
  openControl(app, m);
  return m;
}

export function openControl(app: App, m?: ControlModelItem): void {
  (app as any).bottom?.show?.('control');
  if (m) (app as any).controlPanel?.open?.(m);
}

export async function newModelDialog(app: App, defaultKind: ControlKind = 'des'): Promise<void> {
  const r = await dialog<{ kind: ControlKind; name: string }>(isMrsKind(defaultKind) ? 'New group-control model' : 'New control model', [
    { key: 'kind', label: 'Kind', type: 'select', value: defaultKind, options: CONTROL_KINDS.map((k) => ({ value: k.kind, label: `${k.label} (${kindTag(k)})` })) },
    { key: 'name', label: 'Name', type: 'text', value: '' },
  ], { okLabel: 'Create' });
  if (r) newControlModel(app, r.kind, r.name.trim() || undefined);
}

export async function courseExamplesDialog(app: App): Promise<void> {
  const r = await dialog<{ id: string }>('Course examples (A: mobile manipulator, B: production cell)', [
    { key: 'id', label: 'Example', type: 'select', value: COURSE_EXAMPLES[0].id, options: [{ value: '*', label: 'All examples' }, ...COURSE_EXAMPLES.map((e) => ({ value: e.id, label: `${e.name} — ch. ${e.chapter}` }))] },
  ], { okLabel: 'Add', width: 640 });
  if (!r) return;
  const list = r.id === '*' ? COURSE_EXAMPLES : COURSE_EXAMPLES.filter((e) => e.id === r.id);
  let last: ControlModelItem | null = null;
  app.cmd(() => { for (const e of list) last = addControlModel(app.station, e.kind, e.name, e.source); });
  if (last) { app.select(last); openControl(app, last); }
  toast(`${list.length} model(s) added`, 'ok');
}

/** Course examples of the group-control module (ПР1–ПР6, the warehouse homework, chapter examples). */
export async function groupExamplesDialog(app: App): Promise<void> {
  const r = await dialog<{ id: string }>('Group control — course examples (ПР1–ПР6, warehouse homework, chapters)', [
    { key: 'id', label: 'Example', type: 'select', value: MRS_EXAMPLES[0].id, options: [{ value: '*', label: 'All examples' }, ...MRS_PARTS.map((part) => ({ value: `part:${part}`, label: `All of ${part}` })), ...MRS_EXAMPLES.map((e) => ({ value: e.id, label: `${e.name} — ${e.kind}` }))] },
  ], { okLabel: 'Add', width: 720 });
  if (!r) return;
  const list = r.id === '*' ? MRS_EXAMPLES : r.id.startsWith('part:') ? MRS_EXAMPLES.filter((e) => e.part === r.id.slice(5)) : MRS_EXAMPLES.filter((e) => e.id === r.id);
  let last: ControlModelItem | null = null;
  app.cmd(() => { for (const e of list) last = addControlModel(app.station, e.kind, e.name, e.source); });
  if (last) { app.select(last); openControl(app, last); }
  toast(`${list.length} model(s) added`, 'ok');
}

/** Build the station scene (map, station zones, robots at their homes) of a warehouse model. */
export function buildWarehouseSceneFromModel(app: App, m: ControlModelItem): void {
  if (m.kind !== 'warehouse') return toast('Select a warehouse model (group control) first', 'warn');
  try { const d = parseWarehouse(m.source); const existing = app.station.itemsOfType<MobileRobot>(ItemType.MOBILE_ROBOT).filter((r) => d.cfg.robots.names.includes(r.name)); if (existing.length) return toast(`Robots ${existing.map((r) => r.name).join(', ')} already exist — remove them or rename the fleet`, 'warn'); const built = app.cmd(() => buildWarehouseScene(app.station, d.cfg, d.stationKinds)); app.select(built.robots[0]); toast(`Warehouse scene: ${built.zones.length} stations, ${built.robots.length} robots`, 'ok'); app.log(`[fleet] built the scene of "${m.name}": map ${d.cfg.warehouse.map.split('\n').length} rows, stations ${Object.keys(d.cfg.warehouse.stations).join(' ')}`); } catch (e) { toast(`Cannot build the scene: ${(e as Error).message}`, 'error', 6000); }
}

export function analyseAll(app: App): void {
  const ms = controlModels(app.station);
  if (!ms.length) return toast('No control models in the station', 'warn');
  let bad = 0;
  for (const m of ms) if (!analyseModel(app, m).ok) bad++;
  toast(bad ? `${bad} of ${ms.length} models report issues` : `${ms.length} models OK`, bad ? 'warn' : 'ok');
  (app as any).controlPanel?.render?.();
}

/** Properties section for a control model. */
export function controlSection(app: App, m: ControlModelItem): HTMLElement {
  const k = CONTROL_KINDS.find((x) => x.kind === m.kind);
  return h('details', { class: 'section', open: true }, h('summary', null, t('Control model')),
    h('div', { class: 'kv' }, kv('Kind', k?.label ?? m.kind), kv('Course chapter', k?.chapter ?? '—'), kv('Lines', String(m.source.split('\n').length)), kv('Last verdict', m.lastOk === null ? '—' : m.lastOk ? '✅ OK' : '❌ issues found')),
    h('div', { class: 'btn-row' },
      h('button', { class: 'btn primary', onClick: () => openControl(app, m) }, t('Open in Control tab')),
      h('button', { class: 'btn', onClick: () => { analyseModel(app, m); openControl(app, m); } }, t('Analyse')),
      h('button', { class: 'btn', onClick: () => downloadText(`${m.name}.md`, m.lastReport ?? analyseModel(app, m).markdown, 'text/markdown') }, t('Report (Markdown)'))));
}

// ---------------------------------------------------------------------------------------------
// Graph view (SVG)
// ---------------------------------------------------------------------------------------------

/** Layered layout: BFS from initial / root nodes, top to bottom. Returns an SVG element. */
export function renderGraph(g: GraphView, width = 320): SVGSVGElement {
  const NS = 'http://www.w3.org/2000/svg';
  const el = (tag: string, attrs: Record<string, string | number>, text?: string) => { const e = document.createElementNS(NS, tag); for (const [k, v] of Object.entries(attrs)) e.setAttribute(k, String(v)); if (text !== undefined) e.textContent = text; return e; };
  let fixed: Map<string, [number, number]> | undefined;
  if (g.kind === 'network' && !g.positions && g.nodes.length > 2) { // communication graphs without coordinates: nodes on a circle
    const n = g.nodes.length; const r = Math.max(60, 22 * n); fixed = new Map(g.nodes.map((nd, i) => [nd.id, [r + 40 + r * Math.cos((2 * Math.PI * i) / n - Math.PI / 2), r + 40 + r * Math.sin((2 * Math.PI * i) / n - Math.PI / 2)] as [number, number]]));
  }
  if (g.positions) { // pin the nodes to the model coordinates, scaled into the drawing
    const ps = Object.values(g.positions); const xs = ps.map((q) => q[0]), ys = ps.map((q) => q[1]); const x0 = Math.min(...xs), x1 = Math.max(...xs), y0 = Math.min(...ys), y1 = Math.max(...ys); const span = Math.max(x1 - x0, y1 - y0, 1e-9); const sc = Math.max(width, 320) / span; fixed = new Map(Object.entries(g.positions).map(([id, q]) => [id, [40 + (q[0] - x0) * sc, 40 + (y1 - q[1]) * sc] as [number, number]]));
  }
  const lay = layoutGraph(g, width, fixed); const { pos, tree } = lay; const Wf = lay.width, H = lay.height; const BOX_W = TREE_BOX.w, BOX_H = TREE_BOX.h;
  const svg = el('svg', { class: 'ctl-graph', viewBox: `0 0 ${Wf} ${H}`, width: Wf, height: H }) as SVGSVGElement;
  const defs = el('defs', {}); const marker = el('marker', { id: 'ctl-arrow', viewBox: '0 0 10 10', refX: 9, refY: 5, markerWidth: 7, markerHeight: 7, orient: 'auto-start-reverse' });
  marker.appendChild(el('path', { d: 'M 0 0 L 10 5 L 0 10 z', fill: 'currentColor' })); defs.appendChild(marker); svg.appendChild(defs);
  const R = tree ? 0 : 20;
  const seen = new Map<string, number>();
  for (const e of g.edges) {
    const a = pos.get(e.from), b = pos.get(e.to); if (!a || !b) continue;
    const key = e.from < e.to ? `${e.from}|${e.to}` : `${e.to}|${e.from}`; const k = seen.get(key) ?? 0; seen.set(key, k + 1);
    if (e.from === e.to) {
      const p = el('path', { d: `M ${a[0] - 8} ${a[1] - R} C ${a[0] - 30} ${a[1] - R - 45}, ${a[0] + 30} ${a[1] - R - 45}, ${a[0] + 8} ${a[1] - R}`, fill: 'none', stroke: 'currentColor', 'marker-end': 'url(#ctl-arrow)', class: 'ctl-edge' }); svg.appendChild(p);
      if (e.label) svg.appendChild(el('text', { x: a[0], y: a[1] - R - 36, class: 'ctl-elabel', 'text-anchor': 'middle' }, e.label));
      continue;
    }
    if (tree) { const sx = a[0] + BOX_W / 2, ex = b[0] - BOX_W / 2; svg.appendChild(el('path', { d: `M ${sx} ${a[1]} C ${sx + 30} ${a[1]}, ${ex - 30} ${b[1]}, ${ex} ${b[1]}`, fill: 'none', stroke: 'currentColor', class: 'ctl-edge' })); continue; }
    const dx = b[0] - a[0], dy = b[1] - a[1], d = Math.hypot(dx, dy) || 1; const ux = dx / d, uy = dy / d;
    const nx = -uy, ny = ux; const off = (k - (k > 0 ? 0.5 : 0)) * 24 * (e.from < e.to ? 1 : -1);
    const sx = a[0] + ux * R, sy = a[1] + uy * R, ex = b[0] - ux * R, ey = b[1] - uy * R;
    const mx = (sx + ex) / 2 + nx * off, my = (sy + ey) / 2 + ny * off;
    svg.appendChild(el('path', { d: off ? `M ${sx} ${sy} Q ${mx} ${my} ${ex} ${ey}` : `M ${sx} ${sy} L ${ex} ${ey}`, fill: 'none', stroke: 'currentColor', ...(g.undirected ? {} : { 'marker-end': 'url(#ctl-arrow)' }), class: 'ctl-edge' }));
    if (e.label) { const lx = sx + (ex - sx) * 0.38 + nx * (off * 0.6 + 7), ly = sy + (ey - sy) * 0.38 + ny * (off * 0.6 + 7); svg.appendChild(el('text', { x: lx, y: ly, class: 'ctl-elabel', 'text-anchor': 'middle' }, e.label.length > 26 ? e.label.slice(0, 25) + '…' : e.label)); }
  }
  for (const n of g.nodes) {
    const [x, y] = pos.get(n.id)!;
    const grp = el('g', { class: `ctl-node ${n.kind ?? ''} ${n.initial ? 'initial' : ''} ${n.marked ? 'marked' : ''}` });
    if (tree) {
      grp.appendChild(el('rect', { x: x - BOX_W / 2, y: y - BOX_H / 2, width: BOX_W, height: BOX_H, rx: n.kind === 'action' ? 3 : n.kind === 'condition' ? 10 : 1 }));
      grp.appendChild(el('text', { x, y: y + 4, 'text-anchor': 'middle', class: 'box' }, n.label.length > 22 ? n.label.slice(0, 21) + '…' : n.label));
    } else if (n.kind === 'transition') {
      grp.appendChild(el('rect', { x: x - 6, y: y - 18, width: 12, height: 36 }));
      grp.appendChild(el('text', { x, y: y + 32, 'text-anchor': 'middle' }, n.label.length > 22 ? n.label.slice(0, 22) + '…' : n.label));
    } else {
      grp.appendChild(el('circle', { cx: x, cy: y, r: R }));
      if (n.marked) grp.appendChild(el('circle', { cx: x, cy: y, r: R - 4, class: 'inner' }));
      if (n.initial) grp.appendChild(el('path', { d: `M ${x - R - 26} ${y} L ${x - R - 2} ${y}`, stroke: 'currentColor', 'marker-end': 'url(#ctl-arrow)' }));
      const tok = n.tokens ? (n.tokens <= 3 ? '●'.repeat(n.tokens) : `${n.tokens}`) : '';
      const short = n.label.length <= 5;
      grp.appendChild(el('text', { x, y: y + 4, 'text-anchor': 'middle', class: tok ? 'tok' : '' }, tok || (short ? n.label : n.id.length <= 5 ? n.id : n.id.slice(0, 4) + '…')));
      if (tok || !short) grp.appendChild(el('text', { x, y: y + R + 13, 'text-anchor': 'middle', class: 'sub' }, n.label.length > 24 ? n.label.slice(0, 24) + '…' : n.label));
    }
    grp.appendChild(el('title', {}, n.label));
    svg.appendChild(grp);
  }
  return svg;
}


/** Serialise a graph view as a self-contained SVG file (explicit colours instead of the theme variables). */
export function standaloneSvg(svg: SVGSVGElement): string {
  const c = svg.cloneNode(true) as SVGSVGElement;
  c.setAttribute('xmlns', 'http://www.w3.org/2000/svg');
  const style = document.createElementNS('http://www.w3.org/2000/svg', 'style');
  style.textContent = `svg { background: #2c323d; font-family: Inter, system-ui, sans-serif; font-size: 11px; color: #9aa4b2; }
.ctl-edge { stroke-width: 1.2; opacity: 0.85; } .ctl-elabel { fill: #e6e9ef; font-size: 10px; }
.ctl-node circle, .ctl-node rect { fill: #242932; stroke: #9aa4b2; stroke-width: 1.4; } .ctl-node.initial circle, .ctl-node.initial rect { stroke: #4dabf7; }
.ctl-node.marked circle.inner { fill: none; } .ctl-node.transition rect { fill: #9aa4b2; } .ctl-node.resource circle { stroke: #fab005; } .ctl-node.idle circle { stroke: #51cf66; }
.ctl-node.action rect { stroke: #51cf66; } .ctl-node.condition rect { stroke: #fab005; } .ctl-node.sequence rect, .ctl-node.fallback rect, .ctl-node.parallel rect { stroke: #4dabf7; }
.ctl-node text { fill: #e6e9ef; font-size: 11px; } .ctl-node text.sub { font-size: 10px; fill: #9aa4b2; } .ctl-node text.tok { font-size: 9px; } .ctl-node text.box { font-size: 10px; }
.plot-title { fill: #e6e9ef; font-size: 11px; font-weight: 600; } .plot-tick { fill: #9aa4b2; font-size: 9px; } .plot-label { fill: #9aa4b2; font-size: 10px; } .plot-grid { stroke: #3a414d; stroke-width: 0.6; } .plot-frame { fill: none; stroke: #9aa4b2; stroke-width: 0.8; opacity: 0.6; } .plot-obstacle { fill: #9aa4b2; opacity: 0.45; }
.plot-marker.goal { stroke: #ff6b6b; stroke-width: 1.6; } .plot-marker.source { fill: #fab005; } .plot-marker.station { fill: #4dabf7; } .plot-marker.task { fill: #51cf66; } .plot-marker.robot { fill: none; stroke: #e6e9ef; stroke-width: 1.2; } .plot-marker.start { fill: none; stroke: #51cf66; stroke-width: 1.4; } .plot-marker.obstacle { fill: #9aa4b2; }`;
  c.insertBefore(style, c.firstChild);
  const bg = document.createElementNS('http://www.w3.org/2000/svg', 'rect'); bg.setAttribute('width', '100%'); bg.setAttribute('height', '100%'); bg.setAttribute('fill', '#2c323d');
  c.insertBefore(bg, style.nextSibling);
  return new XMLSerializer().serializeToString(c);
}

/** Render an analysis report (sections, tables, graphs, metrics). */
export function renderReport(r: Report): HTMLElement {
  const wrap = h('div', { class: 'ctl-report' });
  wrap.appendChild(h('div', { class: `ctl-verdict ${r.ok ? 'ok' : 'bad'}` }, `${r.ok ? '✅' : '❌'} ${r.title} — ${r.ok ? t('OK') : t('issues found')} · ${r.durationMs} ms`));
  for (const s of r.sections) {
    const d = h('details', { class: `ctl-section lvl-${s.level}`, open: s.level !== 'info' || !!s.graph || !!s.plot }, h('summary', null, `${LEVEL_ICON[s.level]} ${s.title}`));
    if (s.lines.length) d.appendChild(h('ul', null, ...s.lines.map((l) => h('li', null, l))));
    if (s.table) d.appendChild(h('table', { class: 'grid small' }, h('thead', null, h('tr', null, ...s.table.head.map((c) => h('th', null, c)))), h('tbody', null, ...s.table.rows.map((row) => h('tr', null, ...row.map((c) => h('td', null, String(c))))))));
    if (s.graph) d.appendChild(h('div', { class: 'ctl-graph-wrap' }, renderGraph(s.graph) as unknown as HTMLElement));
    if (s.plot) d.appendChild(h('div', { class: 'ctl-graph-wrap' }, renderPlot(s.plot) as unknown as HTMLElement));
    wrap.appendChild(d);
  }
  const mk = Object.entries(r.metrics);
  if (mk.length) wrap.appendChild(h('details', { class: 'ctl-section lvl-info' }, h('summary', null, `📊 ${t('Metrics')}`), h('table', { class: 'grid small' }, h('tbody', null, ...mk.map(([k, v]) => h('tr', null, h('td', null, k), h('td', null, String(v))))))));
  return wrap;
}

// ---------------------------------------------------------------------------------------------
// Mission runtime on the station
// ---------------------------------------------------------------------------------------------

export interface MissionRun { rt: ControlRuntime; robot: MobileRobot; model: ControlModelItem; hook: (dt: number) => void; startedAt: number; done: boolean }

export function startMission(app: App, model: ControlModelItem, robot: MobileRobot, opts: { targets?: number; maxSeconds?: number } = {}): MissionRun {
  const bt = parseBt(model.source);
  const models = controlModels(app.station);
  const findModel = (kind: ControlKind, name?: string) => (name ? models.find((m) => m.kind === kind && m.name === name) : undefined) ?? models.find((m) => m.kind === kind);
  let supervisor: SupervisorTable | undefined; const desM = findModel('des', bt.supervisor);
  if (desM) { const s = supervisorFromDes(desM.source); if (s) { supervisor = s; app.log(`[control] supervisor from "${desM.name}": ${s.states.length} states`); } else toast(`Supervisor "${desM.name}" is unrealisable — running without it`, 'warn'); }
  const hm = findModel('hybrid', bt.modes); const modes = hm ? parseHybrid(hm.source) : undefined;
  const human = () => { const it = [...app.station.walk()].find((i: Item) => i !== robot && /human|operator|person|worker/i.test(i.name)); if (!it) return null; const p = it.pose(); return [p[12], p[13]] as [number, number]; };
  const w = stationBindings({ robot, station: app.station, humanPosition: human });
  const host = stationWorld(app).host; // program / move / signal actions of the station for the tree
  const inFleet = app.station.itemsOfType<FleetItem>(ItemType.FLEET).some((f) => f.robotIds.includes(robot.id));
  const rt = new ControlRuntime({ bt, supervisor, modes, bindings: { ...w, actions: { ...host.actions, ...w.actions }, halt: { ...host.halt, ...w.halt }, events: () => [...(w.events?.() ?? []), ...(host.events?.() ?? [])], step: inFleet ? undefined : (dt) => w.stepRobot(dt) }, log: (m) => app.log(`[${model.name}] ${m}`, /denied|VIOLATED|mismatch/.test(m) ? 'warn' : 'info') });
  rt.bb.targets = opts.targets ?? 1;
  const maxSeconds = opts.maxSeconds ?? 600;
  // A tree whose root returns success per cycle (the course mission) keeps being ticked until it reports; a tree
  // without a report action stops at its first success. Failures stop the run when they persist (~0.5 s) or come
  // from a supervisor denial.
  const hasReport = JSON.stringify(bt.root).includes('"report"');
  let failTicks = 0;
  const run: MissionRun = { rt, robot, model, startedAt: app.worldTime, done: false, hook: (dt) => {
    if (run.done) return;
    rt.tick(dt);
    if (modes) { if (typeof rt.bb.vmax === 'number') robot.params.vmaxOverride = rt.bb.vmax; else delete robot.params.vmaxOverride; }
    failTicks = rt.status === 'failure' ? failTicks + 1 : 0;
    const finished = rt.bb.reported === true || (rt.status === 'success' && !hasReport) || (rt.status === 'failure' && (failTicks * dt >= 0.5 || rt.bb.lastDenied !== undefined)) || rt.time >= maxSeconds;
    if (finished) {
      run.done = true; stopMission(app, run);
      const s = rt.summary();
      const msg = `${model.name} on ${robot.name}: ${rt.bb.reported ? 'mission reported' : rt.status} after ${s.time.toFixed(1)} s — denied ${s.denied}, violations ${s.violations}, model mismatches ${s.supervisorErrors}`;
      app.log(`[control] ${msg}`, s.violations || s.supervisorErrors ? 'warn' : 'info'); toast(msg, s.violations ? 'warn' : 'ok', 6000);
    }
  } };
  app.worldHooks.push(run.hook);
  app.startWorld();
  app.log(`[control] mission "${model.name}" started on ${robot.name}${supervisor ? ' under supervisor' : ''}${modes ? ' with mode automaton' : ''}`);
  return run;
}

export function stopMission(app: App, run: MissionRun): void {
  const i = app.worldHooks.indexOf(run.hook); if (i >= 0) app.worldHooks.splice(i, 1);
  run.done = true; delete run.robot.params.vmaxOverride;
  if (run.robot.state.path) { run.robot.state.path = null; run.robot.state.v = 0; run.robot.state.status = 'idle'; }
}

export function traceCsv(rt: ControlRuntime): string {
  const keys = [...new Set(rt.trace.values.flatMap((v) => Object.keys(v)))].filter((k) => rt.trace.values.some((v) => typeof v[k] === 'number' || typeof v[k] === 'boolean'));
  const rows = [['t', ...keys].join(',')];
  rt.trace.t.forEach((tt, i) => rows.push([tt.toFixed(2), ...keys.map((k) => { const v = rt.trace.values[i][k]; return typeof v === 'boolean' ? (v ? 1 : 0) : typeof v === 'number' ? Number(v.toFixed(4)) : ''; })].join(',')));
  return rows.join('\n');
}


// ---------------------------------------------------------------------------------------------
// Automata / Petri nets executed on the station (actions = programs, targets, zones, signals)
// ---------------------------------------------------------------------------------------------

/** Station artefacts the diagram inspector offers as actions. */
export function actionCatalog(app: App): ActionCatalog & { desModels: string[]; hybridModels: string[] } {
  const st = app.station;
  return {
    robots: [...st.itemsOfType<Robot>(ItemType.ROBOT).map((r) => ({ name: r.name, mobile: false })), ...st.itemsOfType<MobileRobot>(ItemType.MOBILE_ROBOT).map((r) => ({ name: r.name, mobile: true }))],
    programs: st.itemsOfType<Program>(ItemType.PROGRAM).map((p) => ({ name: p.name, robot: p.robot()?.name })),
    targets: st.itemsOfType<Target>(ItemType.TARGET).map((x) => x.name),
    zones: st.itemsOfType<ZoneItem>(ItemType.ZONE).map((z) => z.name),
    signals: [...app.processSim.signals.keys()],
    desModels: controlModels(st).filter((m) => m.kind === 'des').map((m) => m.name), hybridModels: controlModels(st).filter((m) => m.kind === 'hybrid').map((m) => m.name),
  };
}

/** The station as an executor world: host actions over the app, per-mobile-robot bindings, world stepping. */
export function stationWorld(app: App): ExecutorWorld & { stop(): void } {
  const host = hostBindings({
    station: app.station,
    runProgram: (p) => app.runProgram(p), programPlaying: () => app.sim.playing, stopProgram: () => app.stopProgram(),
    moveDuration: (robot, target, linear) => { const q1 = robot.jointsForTarget(target); if (!q1) return null; const tr = linear ? planMoveL(robot, robot.joints(), robot.solveFK(q1), robot.motion.speedLinear, robot.motion.accelLinear) : planMoveJ(robot, robot.joints(), q1, robot.motion.speedJoints, robot.motion.accelJoints); return tr.duration; },
    setSignal: (n, v) => app.processSim.setSignal(n, v),
  });
  const mobiles = new Map<string, WorldBindings & { stepRobot(dt: number): void }>();
  const inFleet = (r: MobileRobot) => app.station.itemsOfType<FleetItem>(ItemType.FLEET).some((f) => f.robotIds.includes(r.id));
  const forRobot = (name?: string) => {
    const r = app.station.itemsOfType<MobileRobot>(ItemType.MOBILE_ROBOT).find((m) => m.name === name || (!name && mobiles.size === 0)); if (!r) return null;
    let b = mobiles.get(r.id); if (!b) { b = stationBindings({ robot: r, station: app.station }); mobiles.set(r.id, b); } return b;
  };
  return {
    host, forRobot,
    step: (dt) => { for (const [id, b] of mobiles) { const r = app.station.findById(id) as MobileRobot | null; if (r && !inFleet(r)) b.stepRobot(dt); } },
    events: () => [...(host.events?.() ?? []), ...[...mobiles.values()].flatMap((b) => b.events?.() ?? [])],
    stop: () => { host.halt?.program?.({ time: 0, dt: 0, bb: {} }, {}); for (const [id] of mobiles) { const r = app.station.findById(id) as MobileRobot | null; if (r) { r.state.path = null; r.state.v = 0; r.state.status = 'idle'; } } },
  };
}

export interface ActiveRun { model: ControlModelItem; hook: (dt: number) => void; done: boolean; rt?: ControlRuntime; exec?: AutomatonExecutor | PetriExecutor; fleet?: FleetRuntime; stop(): void; status(): string; logLines(): string[] }

/** Run a group-control model (consensus / formation, swarm, coverage, safety, grid MAPF, warehouse fleet) on the mobile robots of the station. */
export function startFleetRun(app: App, model: ControlModelItem, opts: { maxSeconds?: number } = {}): ActiveRun {
  if (!isMrsKind(model.kind)) throw new Error('not a group-control model');
  if (!RUNNABLE_KINDS.includes(model.kind)) throw new Error(`${kindLabel(model.kind)} is analysed, not executed; runnable kinds: ${RUNNABLE_KINDS.join(', ')}`);
  const robots = app.station.itemsOfType<MobileRobot>(ItemType.MOBILE_ROBOT);
  const log = (m: string) => app.log(`[fleet ${model.name}] ${m}`, /FAULT|DOUBLE|CONFLICT|failed|only \d+ of/.test(m) ? 'warn' : 'info');
  // environment for the mode automaton / supervisor of a mission: the human position (an item named "Human", as in the mission runtime) and the e-stop flag
  const env = () => { const human = [...app.station.walk()].find((it: Item) => /human|operator|person|worker/i.test(it.name)) ?? null; const hp = human ? human.pose() : null; const rs = app.station.itemsOfType<MobileRobot>(ItemType.MOBILE_ROBOT); const dh = hp ? Math.min(...rs.map((r) => Math.hypot(hp[12] - r.state.x, hp[13] - r.state.y) / 1000), 99) : 99; return { dist_human: dh, human: dh < 2, estop: !!(app as any).estop, battery: rs.length ? Math.min(...rs.map((r) => r.batteryLevel())) : 1 }; };
  const rt = new FleetRuntime({ kind: model.kind as MrsKind, source: model.source, robots, station: app.station, log, maxSeconds: opts.maxSeconds, env, createRobots: (name, x, y) => app.cmd(() => { const r = app.station.addChild(new MobileRobot(name)); r.setPose2D(x, y, 0); return r; }) });
  const run: ActiveRun = {
    model, fleet: rt, done: false,
    hook: (dt) => { if (run.done) return; rt.tick(dt); if (rt.done) { run.done = true; const i = app.worldHooks.indexOf(run.hook); if (i >= 0) app.worldHooks.splice(i, 1); toast(rt.status(), 'ok', 6000); } },
    stop: () => { const i = app.worldHooks.indexOf(run.hook); if (i >= 0) app.worldHooks.splice(i, 1); run.done = true; rt.stop(); },
    status: () => rt.status(), logLines: () => rt.log,
  };
  app.worldHooks.push(run.hook); app.startWorld();
  app.log(`[fleet] "${model.name}" (${model.kind}) started on ${rt.robots.map((r) => r.name).join(', ')}`);
  return run;
}

/** Run an automaton or Petri-net model on the station until it is quiescent (or stopped). */
export function startModelRun(app: App, model: ControlModelItem, opts: { maxSeconds?: number } = {}): ActiveRun {
  const world = stationWorld(app);
  const log = (m: string) => app.log(`[${model.name}] ${m}`, /mismatch|failed|denied|deadlock/.test(m) ? 'warn' : 'info');
  let exec: AutomatonExecutor | PetriExecutor;
  if (model.kind === 'des') {
    const doc = parseDes(model.source);
    let supervisor; if (doc.specs.length) { supervisor = supervisorFromDes(model.source) ?? undefined; if (!supervisor) toast('Specifications are unrealisable — running the plant without a supervisor', 'warn'); }
    exec = new AutomatonExecutor({ doc, world, supervisor, log });
  } else if (model.kind === 'petri') {
    const p = parsePetri(model.source);
    exec = new PetriExecutor({ doc: { spec: p.spec, checks: p.checks, horizon: p.horizon, layout: p.layout, actions: p.actions }, world, log });
  } else throw new Error('only automata (des) and Petri nets (petri) can be executed here');
  const maxSeconds = opts.maxSeconds ?? 3600;
  const run: ActiveRun = {
    model, exec, done: false,
    hook: (dt) => {
      if (run.done) return;
      const st = exec.tick(dt);
      if (st !== 'running' || exec.time >= maxSeconds) {
        run.done = true; const i = app.worldHooks.indexOf(run.hook); if (i >= 0) app.worldHooks.splice(i, 1); world.stop();
        const s = exec.summary(); const msg = `${model.name}: ${s.status} after ${s.time.toFixed(1)} s — ${s.fired} firings, ${s.mismatches} mismatches, ${s.failed} failed actions · ${s.where}`;
        app.log(`[control] ${msg}`, s.mismatches || s.failed ? 'warn' : 'info'); toast(msg, s.mismatches || s.failed ? 'warn' : 'ok', 6000);
      }
    },
    stop: () => { const i = app.worldHooks.indexOf(run.hook); if (i >= 0) app.worldHooks.splice(i, 1); run.done = true; exec.stop(); world.stop(); },
    status: () => { const s = exec.summary(); return `${run.done ? '⏹' : '▶'} ${model.name}: ${s.status} · t=${fmt(s.time, 1)} s · ${s.fired} ${t('firings')} · ${t('mismatches')} ${s.mismatches}${s.denied ? ` · ${t('denied')} ${s.denied}` : ''} · ${s.where}${s.running.length ? ` · ${t('running')}: ${s.running.join('; ')}` : ''}`; },
    logLines: () => exec.log,
  };
  app.worldHooks.push(run.hook);
  app.startWorld();
  app.log(`[control] ${model.kind === 'des' ? 'automaton' : 'Petri net'} "${model.name}" started on the station (${model.kind === 'des' ? (exec as AutomatonExecutor).comps.length + ' components' : (exec as PetriExecutor).net.places.length + ' places'})`);
  return run;
}

// ---------------------------------------------------------------------------------------------
// Panel
// ---------------------------------------------------------------------------------------------

export function buildControlPanel(app: App): { el: HTMLElement; render: () => void; open: (m: ControlModelItem) => void } {
  let selected: ControlModelItem | null = null;
  let run: ActiveRun | null = null;
  let lastReport: Report | null = null;
  const list = h('div', { class: 'ctl-list' });
  const nameIn = h('input', { type: 'text', class: 'ctl-name', placeholder: 'Model name' }) as HTMLInputElement;
  const kindSel = h('select', { class: 'ctl-kind' }, ...CONTROL_KINDS.map((k) => h('option', { value: k.kind }, `${k.label} (${kindTag(k)})`))) as HTMLSelectElement;
  const editor = h('textarea', { class: 'ctl-editor', spellcheck: false, wrap: 'off', placeholder: t('Select a model on the left, or create one with New ▾ / Course examples ▾') }) as HTMLTextAreaElement;
  let view: 'text' | 'diagram' = 'text';
  // per-model undo / redo of the document text, shared by the text and diagram views (Ctrl+Z / Ctrl+Y in the diagrams, ↶ ↷ buttons)
  const history = new Map<string, { undo: string[]; redo: string[] }>();
  const histOf = (m: ControlModelItem) => { let hh = history.get(m.id); if (!hh) { hh = { undo: [], redo: [] }; history.set(m.id, hh); } return hh; };
  const commit = (src: string) => { if (!selected) return; const m = selected; if (m.source !== src) { const hh = histOf(m); hh.undo.push(m.source); if (hh.undo.length > 200) hh.undo.shift(); hh.redo.length = 0; } m.source = src; editor.value = src; m.notify('source'); updateHistoryBtns(); };
  const restore = (src: string) => { const m = selected!; m.source = src; editor.value = src; m.notify('source'); if (view === 'diagram') { try { loadDiagram(m.kind, src); } catch { setView('text'); } } updateHistoryBtns(); };
  const undoEdit = () => { if (!selected) return; const hh = histOf(selected); const prev = hh.undo.pop(); if (prev === undefined) return; hh.redo.push(selected.source); restore(prev); };
  const redoEdit = () => { if (!selected) return; const hh = histOf(selected); const next = hh.redo.pop(); if (next === undefined) return; hh.undo.push(selected.source); restore(next); };
  const graph = buildGraphEditor({ onChange: (src) => commit(src), catalog: () => actionCatalog(app) });
  graph.el.style.display = 'none';
  const btGraph = buildBtEditor({ onChange: (src) => commit(src), catalog: () => actionCatalog(app) });
  btGraph.el.style.display = 'none';
  const editorWrap = h('div', { class: 'ctl-editor-wrap' }, editor, graph.el, btGraph.el);
  const canDiagram = (k: string) => k === 'des' || k === 'petri' || k === 'bt';
  const loadDiagram = (kind: string, src: string) => { if (kind === 'bt') btGraph.load(src); else graph.load(kind as EditorKind, src); };
  const activeGraph = () => (selected?.kind === 'bt' ? btGraph : graph);
  const undoBtn = h('button', { class: 'btn small', title: t('Undo the last edit of this model (Ctrl+Z in the diagram)'), onClick: () => undoEdit() }, '↶');
  const redoBtn = h('button', { class: 'btn small', title: t('Redo (Ctrl+Y / Ctrl+Shift+Z in the diagram)'), onClick: () => redoEdit() }, '↷');
  const updateHistoryBtns = () => { const hh = selected ? histOf(selected) : null; undoBtn.disabled = !hh || !hh.undo.length; redoBtn.disabled = !hh || !hh.redo.length; };
  const viewBtns = { text: h('button', { class: 'btn small on', onClick: () => setView('text') }, t('Text')), diagram: h('button', { class: 'btn small', onClick: () => setView('diagram') }, t('Diagram')), expand: h('button', { class: 'btn small', title: t('Expand the editor over the model list and the report'), onClick: () => { el.classList.toggle('ge-expanded'); viewBtns.expand.classList.toggle('on', el.classList.contains('ge-expanded')); activeGraph().render(); } }, '⛶') };
  const setView = (v: 'text' | 'diagram') => {
    if (v === 'diagram') {
      if (!selected) return toast('Select a model first', 'warn');
      if (!canDiagram(selected.kind)) return toast('The diagram editor is available for automata (des), Petri nets (petri) and behavior trees (bt)', 'warn');
      try { loadDiagram(selected.kind, editor.value); } catch (e) { toast(`Cannot draw the document: ${(e as Error).message}`, 'error', 6000); return; }
    }
    view = v; editor.style.display = v === 'text' ? '' : 'none'; graph.el.style.display = v === 'diagram' && selected?.kind !== 'bt' ? '' : 'none'; btGraph.el.style.display = v === 'diagram' && selected?.kind === 'bt' ? '' : 'none'; editorWrap.classList.toggle('diagram', v === 'diagram'); if (v === 'text' && el.classList.contains('ge-expanded')) { el.classList.remove('ge-expanded'); viewBtns.expand.classList.remove('on'); }
    viewBtns.text.classList.toggle('on', v === 'text'); viewBtns.diagram.classList.toggle('on', v === 'diagram');
    if (v === 'diagram') activeGraph().render();
  };
  const report = h('div', { class: 'ctl-report-wrap' });
  const robotSel = h('select', { class: 'ctl-robot' }) as HTMLSelectElement;
  const targetsIn = h('input', { type: 'number', min: 0, step: 1, value: 1, class: 'ctl-targets', title: 'Objects to deliver (blackboard "targets")' }) as HTMLInputElement;
  const status = h('div', { class: 'ctl-status hint' });
  const runLog = h('div', { class: 'ctl-runlog' });

  const drop = (btn: HTMLElement, entries: () => MenuEntry[]) => { btn.addEventListener('click', () => { const r = btn.getBoundingClientRect(); contextMenu(r.left, r.bottom, entries()); }); return btn; };
  const groups = [...new Set(CONTROL_KINDS.map((k) => k.group))];
  const newBtn = drop(h('button', { class: 'btn primary small' }, t('New') + ' ▾'), () => groups.flatMap((g, i) => [...(i ? [{ separator: true }] : []), { label: GROUP_LABELS[g], disabled: true }, ...CONTROL_KINDS.filter((k) => k.group === g).map((k) => ({ label: `${k.label} (${kindTag(k)})`, action: () => newControlModel(app, k.kind) }))]));
  const exBtn = drop(h('button', { class: 'btn small' }, t('Course examples') + ' ▾'), () => [
    { label: t('Add all examples'), action: () => { app.cmd(() => { for (const e of COURSE_EXAMPLES) addControlModel(app.station, e.kind, e.name, e.source); }); render(); toast(`${COURSE_EXAMPLES.length} models added`, 'ok'); } },
    { separator: true },
    ...COURSE_EXAMPLES.map((e) => ({ label: e.name, action: () => { const m = app.cmd(() => addControlModel(app.station, e.kind, e.name, e.source)); open(m); doAnalyse(); } })),
  ]);
  const grpBtn = drop(h('button', { class: 'btn small', title: t('Course examples of the group-control module: practicum ПР1–ПР6, the warehouse homework and the chapter examples') }, t('Group examples') + ' ▾'), () => [
    { label: t('Add all group examples'), action: () => { app.cmd(() => { for (const e of MRS_EXAMPLES) addControlModel(app.station, e.kind, e.name, e.source); }); render(); toast(`${MRS_EXAMPLES.length} models added`, 'ok'); } },
    ...MRS_PARTS.flatMap((part) => [{ separator: true }, { label: part, disabled: true }, ...MRS_EXAMPLES.filter((e) => e.part === part).map((e) => ({ label: e.name, action: () => { const m = app.cmd(() => addControlModel(app.station, e.kind, e.name, e.source)); open(m); doAnalyse(); } }))]),
  ]);
  const sceneBtn = h('button', { class: 'btn small', title: t('Build the map, the station zones and the robots of the selected warehouse model'), onClick: () => { if (selected) buildWarehouseSceneFromModel(app, selected); renderRobots(); } }, t('Build scene'));
  const delBtn = h('button', { class: 'btn small', onClick: () => { if (!selected) return; const m = selected; app.deleteItems([m]); selected = null; render(); } }, t('Delete'));
  const analyseBtn = h('button', { class: 'btn primary', onClick: () => doAnalyse() }, t('Analyse'));
  const allBtn = h('button', { class: 'btn', onClick: () => analyseAll(app) }, t('Analyse all'));
  const detectBtn = h('button', { class: 'btn small', title: 'Detect the kind from the first line of the document', onClick: () => { const k = detectKind(editor.value); if (k && selected) { app.cmd(() => { selected!.kind = k; selected!.notify('kind'); }); kindSel.value = k; toast(`Kind: ${kindLabel(k)}`, 'info'); } else toast('Unknown document kind', 'warn'); } }, t('Detect kind'));
  const runBtn = h('button', { class: 'btn primary', onClick: () => doRun() }, '▶ ' + t('Run mission'));
  const runLabel = () => { runBtn.textContent = '▶ ' + t(selected && isMrsKind(selected.kind) ? 'Run on fleet' : selected && (selected.kind === 'des' || selected.kind === 'petri') ? 'Run on station' : 'Run mission'); sceneBtn.style.display = selected?.kind === 'warehouse' ? '' : 'none'; };
  const stopBtn = h('button', { class: 'btn', onClick: () => { if (run) { run.stop(); app.log('[control] run stopped'); } refreshStatus(); } }, '⏹ ' + t('Stop'));
  const exportBtn = drop(h('button', { class: 'btn' }, t('Export') + ' ▾'), () => {
    const m = selected; if (!m) return [{ label: t('No model selected'), disabled: true }];
    const e: MenuEntry[] = [
      { label: t('Report (Markdown)'), action: () => downloadText(`${m.name}.md`, (lastReport?.kind === m.kind && lastReport.title ? lastReport : analyseModel(app, m)).markdown, 'text/markdown') },
      { label: t('Document (.ctl.txt)'), action: () => downloadText(`${m.name}.ctl.txt`, m.source) },
      { label: t('Graph view (SVG)'), action: () => { const g = report.querySelector<SVGSVGElement>('svg.ctl-graph'); if (!g) return toast('Analyse a model that has a graph view first', 'warn'); downloadText(`${m.name}.svg`, standaloneSvg(g), 'image/svg+xml'); } },
    ];
    if (m.kind === 'des') e.push({ separator: true },
      { label: t('Supervisor table (JSON)'), action: () => { const s = supervisorFromDes(m.source); if (!s) return toast('Specification is unrealisable — no supervisor', 'warn'); downloadText(`${m.name}.supervisor.json`, JSON.stringify(s, null, 1), 'application/json'); } },
      { label: t('Supervisor runtime (Python)'), action: () => { const s = supervisorFromDes(m.source); if (!s) return toast('Specification is unrealisable — no supervisor', 'warn'); downloadText(`${m.name.replace(/\W+/g, '_').toLowerCase()}_supervisor.py`, supervisorPython(s, m.name.replace(/\W+/g, '_').toLowerCase())); } });
    if (run && run.model === m) { e.push({ separator: true }); if (run.rt) e.push({ label: t('Mission trace (CSV)'), action: () => downloadText(`${m.name}.trace.csv`, traceCsv(run!.rt!), 'text/csv') }); e.push({ label: t(run.fleet ? 'Fleet log' : 'Mission log'), action: () => downloadText(`${m.name}.log.txt`, run!.logLines().join('\n')) }); }
    return e;
  });

  const left = h('div', { class: 'ctl-left' }, h('div', { class: 'btn-row' }, newBtn, exBtn, grpBtn, delBtn), list);
  const right = h('div', { class: 'ctl-right' },
    h('div', { class: 'btn-row ctl-head' }, nameIn, kindSel, detectBtn, h('span', { class: 'ctl-view' }, viewBtns.text, viewBtns.diagram, viewBtns.expand, undoBtn, redoBtn), analyseBtn, allBtn, exportBtn),
    h('div', { class: 'ctl-split' }, editorWrap, report),
    h('div', { class: 'btn-row ctl-run' }, h('b', null, t('Mission runtime:')), h('span', null, t('Robot')), robotSel, h('span', null, t('targets')), targetsIn, runBtn, stopBtn, sceneBtn, status),
    runLog);
  const el = h('div', { class: 'ctl-panel' }, left, right);
  // Ctrl+Z / Ctrl+Y / Ctrl+Shift+Z while a diagram is visible (and no text field has the focus): document-level so that
  // the shortcut works right after a toolbar click, an inspector edit or a canvas click
  document.addEventListener('keydown', (e) => {
    if (view !== 'diagram' || !(e.ctrlKey || e.metaKey) || el.offsetParent === null) return;
    const a = document.activeElement as HTMLElement | null; const tag = a?.tagName ?? '';
    if (tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT' || a?.isContentEditable) return; // native undo inside fields
    if (!(a === document.body || el.contains(a))) return;
    const k = e.key.toLowerCase();
    if (k === 'z' && !e.shiftKey) { e.preventDefault(); undoEdit(); } else if (k === 'y' || (k === 'z' && e.shiftKey)) { e.preventDefault(); redoEdit(); }
  });

  nameIn.addEventListener('change', () => { if (selected && nameIn.value.trim()) { const m = selected; app.cmd(() => m.setName(nameIn.value.trim())); renderList(); } });
  kindSel.addEventListener('change', () => { if (selected) { const m = selected; app.cmd(() => { m.kind = kindSel.value as ControlKind; m.notify('kind'); }); renderList(); viewBtns.diagram.disabled = !canDiagram(m.kind); if (view === 'diagram') setView(canDiagram(m.kind) ? 'diagram' : 'text'); } });
  editor.addEventListener('input', () => { if (selected) { selected.source = editor.value; } });
  editor.addEventListener('change', () => { if (selected) commit(editor.value); });
  editor.addEventListener('keydown', (e) => { if (e.key === 'Tab') { e.preventDefault(); const s = editor.selectionStart; editor.setRangeText('  ', s, editor.selectionEnd, 'end'); editor.dispatchEvent(new Event('input')); } if ((e.ctrlKey || e.metaKey) && e.key === 'Enter') { e.preventDefault(); doAnalyse(); } });

  const renderList = () => {
    clear(list);
    const ms = controlModels(app.station);
    if (!ms.length) { list.appendChild(h('div', { class: 'hint' }, t('No control models. Use New ▾ or Course examples ▾, or Help › Demo scenarios (control design).'))); return; }
    for (const m of ms) {
      list.appendChild(h('div', { class: `ctl-item ${m === selected ? 'active' : ''}`, onClick: () => { open(m); app.select(m); } },
        h('span', { class: `ctl-dot ${m.lastOk === null ? '' : m.lastOk ? 'ok' : 'bad'}` }, m.lastOk === null ? '○' : m.lastOk ? '●' : '●'),
        h('span', { class: 'ctl-item-name' }, m.name),
        h('small', { class: 'badge' }, m.kind)));
    }
  };
  const renderRobots = () => {
    const cur = robotSel.value; clear(robotSel);
    for (const r of app.station.itemsOfType<MobileRobot>(ItemType.MOBILE_ROBOT)) robotSel.appendChild(h('option', { value: r.id }, r.name));
    if ([...robotSel.options].some((o) => o.value === cur)) robotSel.value = cur;
  };
  const open = (m: ControlModelItem) => {
    selected = m; nameIn.value = m.name; kindSel.value = m.kind; editor.value = m.source;
    viewBtns.diagram.disabled = !canDiagram(m.kind);
    if (view === 'diagram') { if (canDiagram(m.kind)) { try { loadDiagram(m.kind, m.source); graph.el.style.display = m.kind === 'bt' ? 'none' : ''; btGraph.el.style.display = m.kind === 'bt' ? '' : 'none'; } catch (e) { toast(`Cannot draw the document: ${(e as Error).message}`, 'warn'); setView('text'); } } else setView('text'); }
    clear(report); lastReport = null;
    if (m.lastReport) report.appendChild(h('div', { class: 'hint' }, t('Last verdict:') + ` ${m.lastOk ? '✅ OK' : '❌ issues found'} — ${t('press Analyse (Ctrl+Enter) for the full report')}`));
    renderList(); refreshStatus(); updateHistoryBtns();
    if (m.kind === 'bt') doAnalyse();
  };
  const doAnalyse = () => {
    if (!selected) return toast('Select or create a model first', 'warn');
    selected.source = editor.value;
    const r = analyseModel(app, selected); lastReport = r;
    clear(report); report.appendChild(renderReport(r));
    renderList();
  };
  const doRun = () => {
    if (!selected) return toast('Select a model first', 'warn');
    if (run && !run.done) run.stop();
    if (view === 'text') selected.source = editor.value;
    if (isMrsKind(selected.kind)) {
      try { run = startFleetRun(app, selected); shownDone = false; } catch (e) { toast(`Cannot start: ${(e as Error).message}`, 'error', 7000); return; }
      refreshStatus(); return;
    }
    if (selected.kind === 'des' || selected.kind === 'petri') {
      try { run = startModelRun(app, selected); shownDone = false; } catch (e) { toast(`Cannot start: ${(e as Error).message}`, 'error', 6000); return; }
      refreshStatus(); return;
    }
    if (selected.kind !== 'bt') return toast('Run needs a behavior tree (bt), an automaton (des) or a Petri net (petri) model', 'warn');
    const robot = app.station.findById(robotSel.value) as MobileRobot | null;
    if (!robot) return toast('Add a mobile robot first (Mobile & Fleet › Add mobile robot…)', 'warn');
    try { const m = startMission(app, selected, robot, { targets: Number(targetsIn.value) || 0 }); run = { model: m.model, hook: m.hook, get done() { return m.done; }, set done(v: boolean) { m.done = v; }, rt: m.rt, stop: () => stopMission(app, m), status: () => missionStatus(m), logLines: () => m.rt.log }; shownDone = false; } catch (e) { toast(`Cannot start: ${(e as Error).message}`, 'error', 6000); return; }
    refreshStatus();
  };
  const missionStatus = (m: MissionRun) => { const s = m.rt.summary(); return `${m.done ? '⏹' : '▶'} ${m.model.name} on ${m.robot.name}: ${s.status} · t=${fmt(s.time, 1)} s · ${s.ticks} ticks · ${t('plant')} ${s.plantState ?? '—'}${s.mode ? ` · ${t('mode')} ${s.mode}` : ''} · ${t('denied')} ${s.denied} · ${t('violations')} ${s.violations} · ${s.verdicts.map((v) => `${v.verdict} ${v.formula}`).join(' | ')}`; };
  const refreshStatus = () => {
    runLabel();
    if (!run) { status.textContent = t(selected && isMrsKind(selected.kind) ? (RUNNABLE_KINDS.includes(selected.kind) ? 'Run on fleet drives the mobile robots of the station with the group law of the model (mission under a chosen architecture, consensus / formation, swarm, coverage, safety filter, grid MAPF plan, warehouse fleet); a warehouse model builds its scene with Build scene.' : 'This group-control model is analysed only (report with charts); runnable kinds: mission, consensus, swarm, coverage, safety, gridmapf, warehouse.') : selected && (selected.kind === 'des' || selected.kind === 'petri') ? 'Run on station executes the automaton / net: entry actions and transition operations bound in the diagram inspector drive the robots, programs, targets and signals of the station.' : 'Zones named as in the tree (home, table, bin, dock) are used as goto targets; an item named "Human" is the human position.'); return; }
    status.textContent = run.status();
    clear(runLog);
    for (const l of run.logLines().slice(-6)) runLog.appendChild(h('div', { class: /denied|VIOLATED|mismatch|failed|deadlock/.test(l) ? 'log-warn' : 'log-info' }, l));
  };
  const render = () => {
    const ms = controlModels(app.station);
    if (selected && !ms.includes(selected)) selected = null;
    if (!selected && ms.length) { const sel = app.station.selection.find((i) => i instanceof ControlModelItem) as ControlModelItem | undefined; open(sel ?? ms[0]); }
    else if (selected) renderList();
    else { renderList(); nameIn.value = ''; editor.value = ''; clear(report); if (view === 'diagram') setView('text'); }
    renderRobots(); refreshStatus();
  };
  let shownDone = false;
  setInterval(() => { if (run && (!run.done || !shownDone) && el.offsetParent !== null) { refreshStatus(); shownDone = run.done; } }, 300);
  (app as any).controlPanel = { render, open: (m: ControlModelItem) => { open(m); }, current: () => selected, run: () => run, start: doRun, stop: () => { if (run) run.stop(); refreshStatus(); }, analyse: doAnalyse, setView, graph, btGraph, undo: undoEdit, redo: redoEdit, history: () => (selected ? histOf(selected) : null), graphSvgs: () => [...report.querySelectorAll<SVGSVGElement>('svg.ctl-graph')].map(standaloneSvg), buildScene: () => { if (selected) buildWarehouseSceneFromModel(app, selected); renderRobots(); } };
  return { el, render, open: (m) => { open(m); } };
}
