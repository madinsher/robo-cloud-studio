/**
 * Graphical editors for automata (DES documents) and Petri nets on top of the DSL. The diagram and the text are two
 * views of the same document: every edit regenerates the DSL (`desToDsl` / `petriToDsl`, positions in `layout`
 * lines), and switching back to the diagram re-parses the text.
 *
 * Interaction: Select — drag nodes, click to inspect; State / Place / Transition — click on the canvas to add;
 * Edge / Arc — click the source then the target; Delete — click a node or an edge; Delete key removes the selection,
 * Escape cancels; double-click on empty canvas adds a state / place.
 */
import { h, clear, toast, dialog } from './dom';
import { t } from './i18n';
import { parseDes, parsePetri, DesDoc, ActionBinding, ACTION_KINDS } from '../ctl/dsl';
import type { AutomatonSpec } from '../ctl/des';
import type { PlaceKind } from '../ctl/petri';
import type { GraphView } from '../ctl/analysis';
import { desToDsl, petriToDsl, PetriDoc, Layout, XY, automatonStates, automatonEdges, addState, removeState, renameState, setInitial, toggleMarked, setEdgeEvents, edgeEvents, setAlphabet, newAutomaton, uniqueName, addPlace, addPetriTransition, removePetriNode, renamePetriNode, setArc, removeArc, isPlace, petriIds, setAction, actionOf, retargetActions } from '../ctl/graphdoc';
import { layoutGraph } from './graph_layout';

export type EditorKind = 'des' | 'petri';
export type EditorMode = 'select' | 'node' | 'node2' | 'edge' | 'delete';
export type Selection = { type: 'node'; id: string } | { type: 'edge'; from: string; to: string } | null;
/** Station artefacts offered by the action panel (robots, programs, targets, zones, signals). */
export interface ActionCatalog { robots: Array<{ name: string; mobile: boolean }>; programs: Array<{ name: string; robot?: string }>; targets: string[]; zones: string[]; signals: string[] }
export interface GraphEditor {
  el: HTMLElement;
  /** Parse `source` of the given kind into the diagram (throws DslError on a syntax error). */
  load(kind: EditorKind, source: string): void;
  source(): string;
  kind(): EditorKind | null;
  render(): void;
  setMode(m: EditorMode): void;
  autoLayout(): void;
  select(sel: Selection): void;
  /** Add a node at canvas coordinates (state / place, or a Petri transition with `transition = true`). */
  addNodeAt(x: number, y: number, transition?: boolean): string | null;
  /** Connect two nodes (DES: with the given events; Petri: an arc). */
  connect(from: string, to: string, events?: string[]): void;
  moveNode(id: string, x: number, y: number): void;
  state(): { des: DesDoc | null; petri: PetriDoc | null; block: number; mode: EditorMode; selection: Selection };
  setBlock(i: number): void;
}

const NS = 'http://www.w3.org/2000/svg';
const R = 22;
const PLACE_KINDS: PlaceKind[] = ['idle', 'activity', 'resource', 'monitor', 'buffer', 'other'];
export const svgEl = (tag: string, attrs: Record<string, string | number | undefined>, text?: string) => { const e = document.createElementNS(NS, tag); for (const [k, v] of Object.entries(attrs)) if (v !== undefined) e.setAttribute(k, String(v)); if (text !== undefined) e.textContent = text; return e; };
export const field = (label: string, input: HTMLElement) => h('label', { class: 'ge-field' }, h('span', null, t(label)), input);
export const num = (v: number | undefined, onChange: (n: number | undefined) => void, step = 'any') => { const i = h('input', { type: 'number', step, value: v === undefined ? '' : String(v), class: 'ge-num' }) as HTMLInputElement; i.addEventListener('change', () => onChange(i.value === '' ? undefined : Number(i.value))); return i; };
export const text = (v: string, onChange: (s: string) => void, placeholder = '') => { const i = h('input', { type: 'text', value: v, placeholder, class: 'ge-text' }) as HTMLInputElement; i.addEventListener('change', () => onChange(i.value)); i.addEventListener('keydown', (e) => { if (e.key === 'Enter') i.blur(); }); return i; };
export const check = (label: string, v: boolean, onChange: (b: boolean) => void) => { const i = h('input', { type: 'checkbox' }) as HTMLInputElement; i.checked = v; i.addEventListener('change', () => onChange(i.checked)); return h('label', { class: 'ge-check' }, i, ' ', t(label)); };

export function buildGraphEditor(opts: { onChange: (source: string) => void; catalog?: () => ActionCatalog }): GraphEditor {
  let kind: EditorKind | null = null; let des: DesDoc | null = null; let petri: PetriDoc | null = null; let block = 0;
  let mode: EditorMode = 'select'; let sel: Selection = null; let arcFrom: string | null = null;
  let drag: { id: string; dx: number; dy: number; moved: boolean } | null = null;

  const bar = h('div', { class: 'btn-row ge-bar' });
  const canvas = h('div', { class: 'ge-canvas' });
  const inspector = h('div', { class: 'ge-inspector' });
  const svg = svgEl('svg', { class: 'ctl-graph ge-svg', tabindex: 0 }) as SVGSVGElement;
  canvas.appendChild(svg);
  const el = h('div', { class: 'ge-root' }, bar, h('div', { class: 'ge-body' }, canvas, inspector));

  // ---- document access -----------------------------------------------------------------------
  const blocks = (): Array<{ a: AutomatonSpec; spec: boolean }> => (des ? [...des.plant.map((a) => ({ a, spec: false })), ...des.specs.map((a) => ({ a, spec: true }))] : []);
  const cur = (): AutomatonSpec | null => blocks()[block]?.a ?? null;
  const layout = (): Layout => { if (kind === 'des') { const a = cur(); if (!a || !des) return {}; return ((des.layout ??= {})[a.name] ??= {}); } return petri ? (petri.layout ??= {}) : {}; };
  const view = (): GraphView => {
    if (kind === 'des') { const a = cur(); if (!a) return { kind: 'automaton', nodes: [], edges: [] }; return { kind: 'automaton', nodes: automatonStates(a).map((s) => ({ id: s, label: s, initial: s === a.initial, marked: a.marked.includes(s) })), edges: automatonEdges(a).map((e) => ({ from: e.from, to: e.to, label: e.events.join(',') })) }; }
    const s = petri!.spec;
    return { kind: 'petri', nodes: [...s.places.map((p) => ({ id: p.id, label: p.id, kind: p.kind ?? 'place', tokens: p.tokens ?? 0 })), ...s.transitions.map((tr) => ({ id: tr.id, label: tr.id, kind: 'transition' }))], edges: s.arcs.map((a) => ({ from: a.from, to: a.to, label: a.weight && a.weight > 1 ? String(a.weight) : a.inhibitor ? 'o' : undefined })) };
  };
  const ensureLayout = () => {
    const L = layout(); const g = view(); const missing = g.nodes.filter((n) => !L[n.id]); if (!missing.length) return;
    const fixed = new Map<string, XY>(Object.entries(L).filter(([id]) => g.nodes.some((n) => n.id === id)) as Array<[string, XY]>);
    const res = layoutGraph(g, 320, fixed.size ? fixed : undefined);
    for (const n of missing) L[n.id] = res.pos.get(n.id) ?? [80, 80];
  };
  const actions = (): ActionBinding[] => (kind === 'des' && des ? (des.actions ??= []) : petri ? (petri.actions ??= []) : []);
  const actionKey = (id: string) => (kind === 'des' ? `${cur()?.name}.${id}` : id);
  const hasAction = (id: string) => !!actionOf(actions(), actionKey(id)) || (kind === 'des' && actions().some((a) => a.target === id));
  const source = () => (kind === 'des' && des ? desToDsl(des) : kind === 'petri' && petri ? petriToDsl(petri) : '');
  const emit = () => opts.onChange(source());
  const say = (e: unknown) => toast((e as Error).message, 'warn');
  const guarded = (fn: () => void) => { try { fn(); emit(); render(); } catch (e) { say(e); render(); } };

  // ---- rendering -----------------------------------------------------------------------------
  const isSel = (s: Selection) => sel && JSON.stringify(sel) === JSON.stringify(s);
  function render() {
    renderBar(); renderInspector();
    clear(svg);
    if (!kind) { svg.setAttribute('width', '320'); svg.setAttribute('height', '120'); svg.appendChild(svgEl('text', { x: 12, y: 30, class: 'ctl-elabel' }, t('No document loaded'))); return; }
    ensureLayout();
    const g = view(); const L = layout();
    const xs = g.nodes.map((n) => L[n.id][0]), ys = g.nodes.map((n) => L[n.id][1]);
    const W = Math.max(canvas.clientWidth - 2, (xs.length ? Math.max(...xs) : 0) + 80), H = Math.max(canvas.clientHeight - 2, (ys.length ? Math.max(...ys) : 0) + 80);
    svg.setAttribute('width', String(W)); svg.setAttribute('height', String(H)); svg.setAttribute('viewBox', `0 0 ${W} ${H}`);
    const defs = svgEl('defs', {}); const marker = svgEl('marker', { id: 'ge-arrow', viewBox: '0 0 10 10', refX: 9, refY: 5, markerWidth: 7, markerHeight: 7, orient: 'auto-start-reverse' });
    marker.appendChild(svgEl('path', { d: 'M 0 0 L 10 5 L 0 10 z', fill: 'currentColor' })); defs.appendChild(marker); svg.appendChild(defs);
    svg.appendChild(svgEl('rect', { x: 0, y: 0, width: W, height: H, fill: 'transparent', class: 'ge-bg' }));
    const uc = new Set(des?.uncontrollable ?? []);
    const seen = new Map<string, number>();
    for (const e of g.edges) {
      const a = L[e.from], b = L[e.to]; if (!a || !b) continue;
      const key = e.from < e.to ? `${e.from}|${e.to}` : `${e.to}|${e.from}`; const k = seen.get(key) ?? 0; seen.set(key, k + 1);
      const cls = `ge-edge ctl-edge ${kind === 'des' && e.label && e.label.split(',').every((ev) => uc.has(ev)) ? 'uc' : ''} ${isSel({ type: 'edge', from: e.from, to: e.to }) ? 'ge-selected' : ''}`;
      let d: string; let lx: number, ly: number;
      if (e.from === e.to) { d = `M ${a[0] - 8} ${a[1] - R} C ${a[0] - 34} ${a[1] - R - 48}, ${a[0] + 34} ${a[1] - R - 48}, ${a[0] + 8} ${a[1] - R}`; lx = a[0]; ly = a[1] - R - 38; }
      else {
        const dx = b[0] - a[0], dy = b[1] - a[1], dist = Math.hypot(dx, dy) || 1; const ux = dx / dist, uy = dy / dist; const nx = -uy, ny = ux;
        const off = (k - (k > 0 ? 0.5 : 0)) * 26 * (e.from < e.to ? 1 : -1);
        const rb = kind === 'petri' && g.nodes.find((n) => n.id === e.to)?.kind === 'transition' ? 12 : R; const ra = kind === 'petri' && g.nodes.find((n) => n.id === e.from)?.kind === 'transition' ? 12 : R;
        const sx = a[0] + ux * ra, sy = a[1] + uy * ra, ex = b[0] - ux * rb, ey = b[1] - uy * rb; const mx = (sx + ex) / 2 + nx * off, my = (sy + ey) / 2 + ny * off;
        d = off ? `M ${sx} ${sy} Q ${mx} ${my} ${ex} ${ey}` : `M ${sx} ${sy} L ${ex} ${ey}`; lx = sx + (ex - sx) * 0.4 + nx * (off * 0.6 + 8); ly = sy + (ey - sy) * 0.4 + ny * (off * 0.6 + 8);
      }
      const grp = svgEl('g', { class: cls });
      grp.appendChild(svgEl('path', { d, fill: 'none', stroke: 'currentColor', 'marker-end': e.label === 'o' ? undefined : 'url(#ge-arrow)', class: 'ge-line' }));
      if (e.label === 'o') { const b2 = L[e.to]; grp.appendChild(svgEl('circle', { cx: b2[0] - (b2[0] - a[0]) / Math.hypot(b2[0] - a[0], b2[1] - a[1]) * 16, cy: b2[1] - (b2[1] - a[1]) / Math.hypot(b2[0] - a[0], b2[1] - a[1]) * 16, r: 4, fill: 'none', stroke: 'currentColor' })); }
      grp.appendChild(svgEl('path', { d, fill: 'none', class: 'ge-edge-hit' }));
      if (e.label && e.label !== 'o') grp.appendChild(svgEl('text', { x: lx, y: ly, class: 'ctl-elabel', 'text-anchor': 'middle' }, e.label.length > 30 ? e.label.slice(0, 29) + '…' : e.label));
      grp.addEventListener('pointerdown', (ev) => { ev.stopPropagation(); if (mode === 'delete') { removeEdge(e.from, e.to); return; } select({ type: 'edge', from: e.from, to: e.to }); });
      svg.appendChild(grp);
    }
    for (const n of g.nodes) {
      const [x, y] = L[n.id];
      const grp = svgEl('g', { class: `ctl-node ge-node ${n.kind ?? ''} ${n.initial ? 'initial' : ''} ${n.marked ? 'marked' : ''} ${isSel({ type: 'node', id: n.id }) ? 'ge-selected' : ''} ${arcFrom === n.id ? 'ge-arc-src' : ''}` });
      if (n.kind === 'transition') { grp.appendChild(svgEl('rect', { x: x - 7, y: y - 20, width: 14, height: 40 })); grp.appendChild(svgEl('text', { x, y: y + 34, 'text-anchor': 'middle' }, n.label)); }
      else {
        grp.appendChild(svgEl('circle', { cx: x, cy: y, r: R }));
        if (n.marked) grp.appendChild(svgEl('circle', { cx: x, cy: y, r: R - 4, class: 'inner' }));
        if (n.initial) grp.appendChild(svgEl('path', { d: `M ${x - R - 28} ${y} L ${x - R - 2} ${y}`, stroke: 'currentColor', 'marker-end': 'url(#ge-arrow)' }));
        const tok = n.tokens ? (n.tokens <= 4 ? '●'.repeat(n.tokens) : String(n.tokens)) : '';
        const short = n.label.length <= 5;
        grp.appendChild(svgEl('text', { x, y: y + 4, 'text-anchor': 'middle', class: tok ? 'tok' : '' }, tok || (short ? n.label : n.label.slice(0, 4) + '…')));
        if (tok || !short) grp.appendChild(svgEl('text', { x, y: y + R + 13, 'text-anchor': 'middle', class: 'sub' }, n.label));
      }
      if (hasAction(n.id) && (kind === 'petri' ? n.kind === 'transition' : true)) { grp.classList.add('has-action'); grp.appendChild(svgEl('text', { x: x + (n.kind === 'transition' ? 12 : R - 4), y: y - (n.kind === 'transition' ? 12 : R - 6), class: 'act' }, '▶')); }
      grp.appendChild(svgEl('title', {}, n.label + (hasAction(n.id) ? ` — ${(actionOf(actions(), actionKey(n.id)) ?? actions().find((a) => a.target === n.id))?.kind}` : '')));
      grp.addEventListener('pointerdown', (ev) => { ev.stopPropagation(); onNodeDown(n.id, ev as PointerEvent); });
      grp.addEventListener('dblclick', (ev) => { ev.stopPropagation(); select({ type: 'node', id: n.id }); (inspector.querySelector('input.ge-text') as HTMLInputElement | null)?.focus(); });
      svg.appendChild(grp);
    }
  }

  function renderBar() {
    clear(bar);
    if (!kind) return;
    const mb = (m: EditorMode, label: string, title: string) => h('button', { class: `btn small ${mode === m ? 'on' : ''}`, title: t(title), onClick: () => setMode(m) }, t(label));
    bar.append(mb('select', 'Select', 'Drag nodes, click to inspect'), mb('node', kind === 'des' ? '+ State' : '+ Place', 'Click on the canvas to add'));
    if (kind === 'petri') bar.append(mb('node2', '+ Transition', 'Click on the canvas to add a transition'));
    bar.append(mb('edge', kind === 'des' ? '+ Edge' : '+ Arc', 'Click the source node, then the target'), mb('delete', 'Delete', 'Click a node or an edge to remove it'));
    bar.append(h('button', { class: 'btn small', title: t('Recompute the positions of this diagram'), onClick: () => autoLayout() }, t('Auto layout')));
    if (kind === 'des' && des) {
      const list = blocks();
      const selB = h('select', { class: 'ge-block', onChange: (e: Event) => { setBlock(Number((e.target as HTMLSelectElement).value)); } }, ...list.map((b, i) => h('option', { value: i, selected: i === block }, `${b.spec ? 'spec' : 'automaton'} ${b.a.name}`))) as HTMLSelectElement;
      bar.append(h('span', { class: 'ge-sep' }), selB,
        h('button', { class: 'btn small', title: t('Add a plant component'), onClick: () => guarded(() => { const a = newAutomaton(uniqueName(list.map((b) => b.a.name), 'G')); des!.plant.push(a); block = des!.plant.length - 1; sel = null; }) }, t('+ Automaton')),
        h('button', { class: 'btn small', title: t('Add a specification automaton'), onClick: () => guarded(() => { const a = newAutomaton(uniqueName(list.map((b) => b.a.name), 'E')); des!.specs.push(a); block = des!.plant.length + des!.specs.length - 1; sel = null; }) }, t('+ Spec')),
        h('button', { class: 'btn small', title: t('Remove this automaton'), onClick: () => { const b = list[block]; if (!b || list.length <= 1) return toast('The document keeps at least one automaton', 'warn'); guarded(() => { if (b.spec) des!.specs = des!.specs.filter((x) => x !== b.a); else des!.plant = des!.plant.filter((x) => x !== b.a); delete des!.layout?.[b.a.name]; block = Math.max(0, block - 1); sel = null; }); } }, t('Remove')));
    }
    const hints: Record<EditorMode, string> = { select: 'Drag to move · click to inspect · double-click the canvas to add', node: 'Click on the canvas to add', node2: 'Click on the canvas to add a transition', edge: arcFrom ? `From ${arcFrom}: click the target node (Esc cancels)` : 'Click the source node', delete: 'Click a node or an edge to remove it' };
    bar.append(h('span', { class: 'hint ge-hint' }, t(hints[mode])));
  }

  function renderInspector() {
    clear(inspector);
    if (!kind) return;
    const title = (s: string) => h('div', { class: 'prop-title' }, t(s));
    if (!sel) {
      if (kind === 'des' && des) {
        const a = cur(); const d = des;
        inspector.append(title('Document'), field('Name', text(d.name, (v) => guarded(() => { d.name = v.trim() || d.name; }))),
          field('Uncontrollable', text(d.uncontrollable.join(' '), (v) => guarded(() => { d.uncontrollable = v.split(/[\s,]+/).filter(Boolean); }), 'events')),
          field('Unobservable', text(d.unobservable.join(' '), (v) => guarded(() => { d.unobservable = v.split(/[\s,]+/).filter(Boolean); }), 'events')),
          field('Faults', text(d.faults.join(' '), (v) => guarded(() => { d.faults = v.split(/[\s,]+/).filter(Boolean); }), 'events')));
        if (a) inspector.append(title(blocks()[block].spec ? 'Specification' : 'Automaton'),
          field('Name', text(a.name, (v) => guarded(() => { const nm = v.trim(); if (!nm || /\s/.test(nm)) throw new Error('a name without spaces'); if (blocks().some((b) => b.a !== a && b.a.name === nm)) throw new Error(`${nm} exists`); const L = d.layout?.[a.name]; if (d.layout && L) { delete d.layout[a.name]; d.layout[nm] = L; } a.name = nm; }))),
          field('Alphabet', text(a.events.join(' '), (v) => guarded(() => setAlphabet(a, v.split(/[\s,]+/))), 'events')),
          h('div', { class: 'hint' }, t('Events used by transitions are always in the alphabet; declare extra ones here (e.g. a spec that forbids an event).')));
        inspector.append(h('div', { class: 'hint' }, `${d.checks.length} check line(s) are kept as text.`));
      } else if (petri) {
        const p = petri;
        inspector.append(title('Petri net'), field('Name', text(p.spec.name, (v) => guarded(() => { p.spec.name = v.trim() || p.spec.name; }))), field('Horizon (s)', num(p.horizon, (v) => guarded(() => { p.horizon = v ?? 300; }))),
          h('div', { class: 'hint' }, `${p.spec.places.length} places · ${p.spec.transitions.length} transitions · ${p.spec.arcs.length} arcs · ${p.checks.length} check line(s) kept as text`));
      }
      return;
    }
    const del = h('button', { class: 'btn small', onClick: () => deleteSelection() }, t('Delete'));
    if (sel.type === 'node') {
      const id = sel.id;
      if (kind === 'des') {
        const a = cur()!;
        inspector.append(title('State'), field('Name', text(id, (v) => guarded(() => { const L = layout(); renameState(a, id, v.trim()); if (L[id]) { L[v.trim()] = L[id]; delete L[id]; } retargetActions(actions(), actionKey(id), actionKey(v.trim())); sel = { type: 'node', id: v.trim() }; }))),
          check('Initial', a.initial === id, () => guarded(() => setInitial(a, id))), check('Marked', a.marked.includes(id), (b) => guarded(() => toggleMarked(a, id, b))),
          h('div', { class: 'hint' }, `${a.transitions.filter((tr) => tr.from === id).length} outgoing · ${a.transitions.filter((tr) => tr.to === id).length} incoming`), del, actionPanel(actionKey(id), true));
      } else {
        const s = petri!.spec; const pl = s.places.find((x) => x.id === id); const tr = s.transitions.find((x) => x.id === id);
        const rename = (v: string) => guarded(() => { const L = layout(); const nm = v.trim(); renamePetriNode(s, id, nm); if (L[id]) { L[nm] = L[id]; delete L[id]; } retargetActions(actions(), id, nm); sel = { type: 'node', id: nm }; });
        if (pl) inspector.append(title('Place'), field('Name', text(id, rename)), field('Tokens', num(pl.tokens ?? 0, (v) => guarded(() => { pl.tokens = v ?? 0; }), '1')),
          field('Kind', (() => { const sl = h('select', { onChange: (e: Event) => guarded(() => { const v = (e.target as HTMLSelectElement).value; pl.kind = v ? (v as PlaceKind) : undefined; }) }, h('option', { value: '' }, '—'), ...PLACE_KINDS.map((k) => h('option', { value: k, selected: pl.kind === k }, k))); return sl; })()),
          field('Capacity', num(pl.capacity, (v) => guarded(() => { pl.capacity = v; }), '1')), field('Label', text(pl.label ?? '', (v) => guarded(() => { pl.label = v.trim() || undefined; }))), del);
        if (tr) inspector.append(title('Transition'), field('Name', text(id, rename)), field('Delay (s)', num(tr.delay, (v) => guarded(() => { tr.delay = v; }))), field('Rate (1/s)', num(tr.rate, (v) => guarded(() => { tr.rate = v; }))),
          check('Immediate', !!tr.immediate, (b) => guarded(() => { tr.immediate = b || undefined; })), field('Weight', num(tr.weight, (v) => guarded(() => { tr.weight = v; }))), field('Priority', num(tr.priority, (v) => guarded(() => { tr.priority = v; }), '1')), field('Label', text(tr.label ?? '', (v) => guarded(() => { tr.label = v.trim() || undefined; }))), del, actionPanel(id, false));
      }
    } else {
      const { from, to } = sel;
      if (kind === 'des') {
        const a = cur()!;
        inspector.append(title('Edge'), h('div', { class: 'hint' }, `${from} → ${to}`), field('Events', text(edgeEvents(a, from, to).join(', '), (v) => guarded(() => { setEdgeEvents(a, from, to, v.split(/[\s,]+/)); if (!edgeEvents(a, from, to).length) sel = null; }), 'e1, e2')),
          h('div', { class: 'hint' }, t('Uncontrollable events are drawn dashed; edit the list in the document properties.')), del);
      } else {
        const s = petri!.spec; const arc = s.arcs.find((x) => x.from === from && x.to === to)!;
        inspector.append(title('Arc'), h('div', { class: 'hint' }, `${from} → ${to}`), field('Weight', num(arc.weight ?? 1, (v) => guarded(() => setArc(s, from, to, { weight: v, inhibitor: arc.inhibitor })), '1')),
          isPlace(s, from) ? check('Inhibitor', !!arc.inhibitor, (b) => guarded(() => setArc(s, from, to, { weight: arc.weight, inhibitor: b }))) : h('span'),
          h('button', { class: 'btn small', title: t('Swap direction'), onClick: () => guarded(() => { removeArc(s, from, to); setArc(s, to, from, { weight: arc.weight }); sel = { type: 'edge', from: to, to: from }; }) }, t('Reverse')), del);
      }
    }
  }


  // ---- action panel: bind the studio's programming artefacts to a state / transition ----------------
  function actionPanel(target: string, desMode: boolean): HTMLElement {
    const list = actions(); const cur = actionOf(list, target);
    const cat = opts.catalog?.() ?? { robots: [], programs: [], targets: [], zones: [], signals: [] };
    const wrap = h('div', { class: 'ge-action' }, h('div', { class: 'prop-title' }, t(desMode ? 'Entry action (robot)' : 'Operation (robot)')));
    const defaults: Record<ActionBinding['kind'], string> = { program: cat.programs[0]?.name ?? '', target: cat.targets[0] ?? '', goto: cat.zones[0] ?? '', signal: cat.signals[0] ?? 'start', wait: '1', event: 'done', set: 'flag' };
    const kindSel = h('select', { onChange: (e: Event) => { const k = (e.target as HTMLSelectElement).value as ActionBinding['kind'] | ''; guarded(() => { if (!k) setAction(list, target, null); else setAction(list, target, { kind: k, value: defaults[k], robot: k === 'goto' ? cat.robots.find((r) => r.mobile)?.name : k === 'program' || k === 'target' ? (cur?.robot ?? cat.robots.find((r) => !r.mobile)?.name) : undefined, done: cur?.done, args: {} }); }); } },
      h('option', { value: '' }, t('none')), ...ACTION_KINDS.map((k) => h('option', { value: k, selected: cur?.kind === k }, t({ program: 'run program', target: 'move to target', goto: 'go to zone', signal: 'set signal', wait: 'wait', event: 'emit event', set: 'set blackboard' }[k]))));
    wrap.append(field('Action', kindSel));
    if (!cur) { wrap.append(h('div', { class: 'hint' }, t(desMode ? 'Runs when the state is entered; done= fires the event that leaves it.' : 'Runs when the transition fires; the output tokens appear when it finishes.'))); return wrap; }
    const upd = (fn: (a: ActionBinding) => void) => guarded(() => fn(cur));
    const selectOf = (values: string[], value: string, onChange: (v: string) => void) => { const opts2 = values.includes(value) || !value ? values : [value, ...values]; const sl = h('select', { onChange: (e: Event) => onChange((e.target as HTMLSelectElement).value) }, ...opts2.map((v) => h('option', { value: v, selected: v === value }, v))); return sl; };
    if (cur.kind === 'program' || cur.kind === 'target' || cur.kind === 'goto') {
      const robots = cat.robots.filter((r) => (cur.kind === 'goto' ? r.mobile : !r.mobile)).map((r) => r.name);
      wrap.append(field('Robot', robots.length ? selectOf(robots, cur.robot ?? '', (v) => upd((a) => { a.robot = v || undefined; })) : text(cur.robot ?? '', (v) => upd((a) => { a.robot = v.trim() || undefined; }), 'robot name')));
    }
    if (cur.kind === 'program') { const progs = cat.programs.filter((p) => !cur.robot || !p.robot || p.robot === cur.robot).map((p) => p.name); wrap.append(field('Program', progs.length ? selectOf(progs, cur.value, (v) => upd((a) => { a.value = v; })) : text(cur.value, (v) => upd((a) => { a.value = v.trim(); }), 'Program item name'))); }
    if (cur.kind === 'target') wrap.append(field('Target', cat.targets.length ? selectOf(cat.targets, cur.value, (v) => upd((a) => { a.value = v; })) : text(cur.value, (v) => upd((a) => { a.value = v.trim(); }), 'Target item name')), check('Linear (MoveL)', cur.args.linear === true, (b) => upd((a) => { if (b) a.args.linear = true; else delete a.args.linear; })));
    if (cur.kind === 'goto') wrap.append(field('Zone', cat.zones.length ? selectOf(cat.zones, cur.value, (v) => upd((a) => { a.value = v; })) : text(cur.value, (v) => upd((a) => { a.value = v.trim(); }), 'zone name')));
    if (cur.kind === 'signal') wrap.append(field('Signal', cat.signals.length ? selectOf(cat.signals, cur.value, (v) => upd((a) => { a.value = v; })) : text(cur.value, (v) => upd((a) => { a.value = v.trim(); }), 'signal name')), field('Value', text(String(cur.args.value ?? 'true'), (v) => upd((a) => { a.args.value = v === 'true' ? true : v === 'false' ? false : Number.isFinite(Number(v)) && v.trim() !== '' ? Number(v) : v; }))));
    if (cur.kind === 'wait') wrap.append(field('Seconds', num(Number(cur.value), (v) => upd((a) => { a.value = String(v ?? 1); }))));
    if (cur.kind === 'event') wrap.append(field('Event', text(cur.value, (v) => upd((a) => { a.value = v.trim(); }))));
    if (cur.kind === 'set') wrap.append(field('Key', text(cur.value, (v) => upd((a) => { a.value = v.trim(); }))), field('Value', text(String(cur.args.value ?? 'true'), (v) => upd((a) => { a.args.value = v; }))));
    if (desMode) { const a0 = cur; const evs = [...new Set(cur ? (cur.kind && (des ? [...(blocks().flatMap((b) => b.a.events))] : [])) : [])]; wrap.append(field('Done event (fired when finished)', evs.length ? selectOf(['', ...evs], a0.done ?? '', (v) => upd((a) => { a.done = v || undefined; })) : text(a0.done ?? '', (v) => upd((a) => { a.done = v.trim() || undefined; }), 'event'))); }
    return wrap;
  }

  // ---- interaction ---------------------------------------------------------------------------
  const pt = (ev: PointerEvent | MouseEvent): XY => { const r = svg.getBoundingClientRect(); return [ev.clientX - r.left, ev.clientY - r.top]; };
  function onNodeDown(id: string, ev: PointerEvent) {
    if (mode === 'delete') { deleteNode(id); return; }
    if (mode === 'edge') { if (!arcFrom) { arcFrom = id; render(); } else { const from = arcFrom; arcFrom = null; void connectInteractive(from, id); } return; }
    select({ type: 'node', id });
    const [x, y] = pt(ev); const p = layout()[id]; drag = { id, dx: p[0] - x, dy: p[1] - y, moved: false };
    svg.setPointerCapture(ev.pointerId);
  }
  svg.addEventListener('pointermove', (ev) => { if (!drag) return; const [x, y] = pt(ev); const L = layout(); L[drag.id] = [Math.max(30, Math.round(x + drag.dx)), Math.max(30, Math.round(y + drag.dy))]; drag.moved = true; render(); });
  svg.addEventListener('pointerup', () => { if (drag?.moved) emit(); drag = null; });
  svg.addEventListener('pointerdown', (ev) => {
    if (!kind) return; svg.focus();
    if (mode === 'node' || mode === 'node2') { const [x, y] = pt(ev); addNodeAt(x, y, mode === 'node2'); return; }
    if (mode === 'edge') { arcFrom = null; render(); return; }
    select(null);
  });
  svg.addEventListener('dblclick', (ev) => { if (kind && mode === 'select' && (ev.target as Element).classList.contains('ge-bg')) { const [x, y] = pt(ev); addNodeAt(x, y, false); } });
  svg.addEventListener('keydown', (ev) => { if (ev.key === 'Delete' || ev.key === 'Backspace') { ev.preventDefault(); deleteSelection(); } if (ev.key === 'Escape') { arcFrom = null; setMode('select'); } });

  async function connectInteractive(from: string, to: string) {
    if (kind === 'des') {
      const a = cur()!; const existing = edgeEvents(a, from, to);
      const r = await dialog<{ events: string }>(existing.length ? 'Edit edge events' : 'New edge', [{ key: 'events', label: `Events ${from} → ${to} (comma separated)`, type: 'text', value: existing.join(', ') }], { okLabel: 'Apply' });
      if (!r) { render(); return; }
      guarded(() => { setEdgeEvents(a, from, to, r.events.split(/[\s,]+/)); sel = edgeEvents(a, from, to).length ? { type: 'edge', from, to } : null; });
    } else guarded(() => { setArc(petri!.spec, from, to); sel = { type: 'edge', from, to }; });
  }
  function connect(from: string, to: string, events?: string[]) { guarded(() => { if (kind === 'des') setEdgeEvents(cur()!, from, to, [...edgeEvents(cur()!, from, to), ...(events ?? [])]); else setArc(petri!.spec, from, to); sel = { type: 'edge', from, to }; }); }
  function addNodeAt(x: number, y: number, transition = false): string | null {
    let id: string | null = null;
    guarded(() => { if (kind === 'des') id = addState(cur()!); else id = transition ? addPetriTransition(petri!.spec) : addPlace(petri!.spec); layout()[id] = [Math.round(x), Math.round(y)]; sel = { type: 'node', id }; });
    return id;
  }
  function moveNode(id: string, x: number, y: number) { layout()[id] = [x, y]; emit(); render(); }
  function deleteNode(id: string) { guarded(() => { const L = layout(); delete L[id]; retargetActions(actions(), actionKey(id), null); if (kind === 'des') removeState(cur()!, id); else removePetriNode(petri!.spec, id); if (sel?.type === 'node' && sel.id === id) sel = null; else if (sel?.type === 'edge' && (sel.from === id || sel.to === id)) sel = null; }); }
  function removeEdge(from: string, to: string) { guarded(() => { if (kind === 'des') setEdgeEvents(cur()!, from, to, []); else removeArc(petri!.spec, from, to); sel = null; }); }
  function deleteSelection() { if (!sel) return; if (sel.type === 'node') deleteNode(sel.id); else removeEdge(sel.from, sel.to); }
  function select(s: Selection) { sel = s; render(); }
  function setMode(m: EditorMode) { mode = m; arcFrom = null; canvas.classList.toggle('mode-add', m === 'node' || m === 'node2'); canvas.classList.toggle('mode-edge', m === 'edge'); canvas.classList.toggle('mode-delete', m === 'delete'); render(); }
  function autoLayout() { if (!kind) return; if (kind === 'des') { const a = cur(); if (a && des?.layout) delete des.layout[a.name]; } else if (petri) petri.layout = {}; ensureLayout(); emit(); render(); }
  function setBlock(i: number) { block = Math.max(0, Math.min(i, blocks().length - 1)); sel = null; arcFrom = null; render(); }
  function load(k: EditorKind, src: string) {
    const prevName = kind === 'des' ? cur()?.name : null;
    if (k === 'des') { des = parseDes(src); petri = null; if (!des.plant.length && !des.specs.length) des.plant.push(newAutomaton('G')); }
    else { const p = parsePetri(src); petri = { spec: p.spec, checks: p.checks, horizon: p.horizon, layout: p.layout, actions: p.actions }; des = null; }
    kind = k; sel = null; arcFrom = null;
    const idx = prevName ? blocks().findIndex((b) => b.a.name === prevName) : -1; block = idx >= 0 ? idx : 0;
    render();
  }
  window.addEventListener('resize', () => { if (kind && el.offsetParent !== null) render(); });
  return { el, load, source, kind: () => kind, render, setMode, autoLayout, select, addNodeAt, connect, moveNode, state: () => ({ des, petri, block, mode, selection: sel }), setBlock };
}
