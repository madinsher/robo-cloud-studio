/**
 * Graphical behavior-tree editor on top of the `bt` DSL: the tree is drawn left to right (root on the left, one row
 * per leaf), nodes are selected, added from a type menu, reordered, dragged onto another node to move them, and
 * edited in an inspector (type, name, memory, counts, action binding with its arguments — including the station's
 * programs, targets, zones and signals — timeout and skill contract, supervisor events). Every edit regenerates
 * the text with `btToDsl`; switching back to the diagram re-parses it.
 */
import { h, clear, toast, contextMenu, MenuEntry } from './dom';
import { t } from './i18n';
import { parseBt, tokenize, BtDoc } from '../ctl/dsl';
import type { BTNodeSpec, BTNodeType } from '../ctl/bt';
import type { GraphView } from '../ctl/analysis';
import { btToDsl, btWalk, btParentOf, btAddChild, btRemove, btMove, btReparent, btOutdent, btSetType, btLabel, isBtLeaf, isBtDecorator, BT_COMPOSITES, BT_DECORATORS, BT_LEAVES, BT_GLYPH, BT_ACTION_BINDINGS } from '../ctl/graphdoc';
import { layoutGraph, TREE_BOX } from './graph_layout';
import { svgEl, field, num, text, check, ActionCatalog } from './graph_editor';

export interface BtEditor {
  el: HTMLElement;
  load(source: string): void;
  source(): string;
  render(): void;
  select(node: BTNodeSpec | null): void;
  addChild(type: BTNodeType): BTNodeSpec | null;
  remove(): void;
  move(dir: -1 | 1): void;
  state(): { doc: BtDoc | null; selection: BTNodeSpec | null };
}

const qv = (v: unknown) => (typeof v === 'string' && /\s/.test(v) ? `"${v}"` : String(v));

export function buildBtEditor(opts: { onChange: (source: string) => void; catalog?: () => ActionCatalog & { desModels?: string[]; hybridModels?: string[] } }): BtEditor {
  let doc: BtDoc | null = null; let sel: BTNodeSpec | null = null;
  let drag: { node: BTNodeSpec; x0: number; y0: number; moved: boolean } | null = null; let hover: BTNodeSpec | null = null;
  const bar = h('div', { class: 'btn-row ge-bar' }); const canvas = h('div', { class: 'ge-canvas' }); const inspector = h('div', { class: 'ge-inspector' });
  const svg = svgEl('svg', { class: 'ctl-graph ge-svg bt-svg', tabindex: 0 }) as SVGSVGElement; canvas.appendChild(svg);
  const el = h('div', { class: 'ge-root' }, bar, h('div', { class: 'ge-body' }, canvas, inspector));
  const ids = new Map<BTNodeSpec, string>(); const byId = new Map<string, BTNodeSpec>();
  const source = () => (doc ? btToDsl(doc) : '');
  const emit = () => opts.onChange(source());
  const guarded = (fn: () => void) => { try { fn(); emit(); render(); } catch (e) { toast((e as Error).message, 'warn'); render(); } };

  const view = (): GraphView => {
    ids.clear(); byId.clear(); if (!doc) return { kind: 'tree', nodes: [], edges: [] };
    const nodes: GraphView['nodes'] = []; const edges: GraphView['edges'] = [];
    btWalk(doc.root).forEach(({ node, parent }, i) => { const id = `n${i}`; ids.set(node, id); byId.set(id, node); nodes.push({ id, label: `${BT_GLYPH[node.type]} ${btLabel(node)}`, kind: node.type }); if (parent) edges.push({ from: ids.get(parent)!, to: id }); });
    return { kind: 'tree', nodes, edges };
  };

  function render() {
    renderBar(); renderInspector(); clear(svg);
    if (!doc) { svg.setAttribute('width', '320'); svg.setAttribute('height', '80'); return; }
    const g = view(); const lay = layoutGraph(g, 320);
    const W = Math.max(canvas.clientWidth - 2, lay.width), H = Math.max(canvas.clientHeight - 2, lay.height);
    svg.setAttribute('width', String(W)); svg.setAttribute('height', String(H)); svg.setAttribute('viewBox', `0 0 ${W} ${H}`);
    svg.appendChild(svgEl('rect', { x: 0, y: 0, width: W, height: H, fill: 'transparent', class: 'ge-bg' }));
    for (const e of g.edges) { const a = lay.pos.get(e.from)!, b = lay.pos.get(e.to)!; const sx = a[0] + TREE_BOX.w / 2, ex = b[0] - TREE_BOX.w / 2; svg.appendChild(svgEl('path', { d: `M ${sx} ${a[1]} C ${sx + 30} ${a[1]}, ${ex - 30} ${b[1]}, ${ex} ${b[1]}`, fill: 'none', stroke: 'currentColor', class: 'ctl-edge' })); }
    for (const n of g.nodes) {
      const [x, y] = lay.pos.get(n.id)!; const spec = byId.get(n.id)!;
      const grp = svgEl('g', { class: `ctl-node ge-node bt-node ${n.kind} ${spec === sel ? 'ge-selected' : ''} ${spec === hover ? 'bt-drop' : ''} ${spec === drag?.node && drag.moved ? 'bt-dragging' : ''}`, 'data-id': n.id });
      grp.appendChild(svgEl('rect', { x: x - TREE_BOX.w / 2, y: y - TREE_BOX.h / 2, width: TREE_BOX.w, height: TREE_BOX.h, rx: spec.type === 'action' ? 3 : spec.type === 'condition' ? 10 : 1 }));
      grp.appendChild(svgEl('text', { x, y: y + 4, 'text-anchor': 'middle', class: 'box' }, n.label.length > 22 ? n.label.slice(0, 21) + '…' : n.label));
      grp.appendChild(svgEl('title', {}, `${spec.type}: ${btLabel(spec)}${spec.type === 'action' ? '\n' + Object.entries(spec.args ?? {}).map(([k, v]) => `${k}=${String(v)}`).join(' ') : ''}`));
      grp.addEventListener('pointerdown', (ev) => { ev.stopPropagation(); select(spec); drag = { node: spec, x0: ev.clientX, y0: ev.clientY, moved: false }; svg.setPointerCapture(ev.pointerId); });
      svg.appendChild(grp);
    }
  }
  svg.addEventListener('pointermove', (ev) => {
    if (!drag) return;
    if (!drag.moved && Math.hypot(ev.clientX - drag.x0, ev.clientY - drag.y0) < 6) return;
    drag.moved = true; svg.style.cursor = 'grabbing';
    const target = document.elementFromPoint(ev.clientX, ev.clientY)?.closest('.bt-node') as SVGGElement | null;
    const nh = target ? byId.get(target.getAttribute('data-id') ?? '') ?? null : null;
    if (nh !== hover) { hover = nh && nh !== drag.node ? nh : null; render(); }
  });
  svg.addEventListener('pointerup', () => {
    svg.style.cursor = '';
    if (drag?.moved && hover && doc) { const n = drag.node, tgt = hover; drag = null; hover = null; guarded(() => btReparent(doc!.root, n, tgt)); return; }
    drag = null; if (hover) { hover = null; render(); }
  });
  svg.addEventListener('pointerdown', () => { svg.focus(); select(null); });
  svg.addEventListener('keydown', (ev) => { if (ev.key === 'Delete' || ev.key === 'Backspace') { ev.preventDefault(); remove(); } if (ev.key === 'ArrowUp' && ev.altKey) move(-1); if (ev.key === 'ArrowDown' && ev.altKey) move(1); });

  const typeMenu = (onPick: (tp: BTNodeType) => void): MenuEntry[] => [
    ...BT_COMPOSITES.map((tp) => ({ label: `${BT_GLYPH[tp]} ${tp}`, action: () => onPick(tp) })), { separator: true },
    ...BT_DECORATORS.map((tp) => ({ label: `${BT_GLYPH[tp]} ${tp}`, action: () => onPick(tp) })), { separator: true },
    ...BT_LEAVES.map((tp) => ({ label: `${BT_GLYPH[tp]} ${tp}`, action: () => onPick(tp) })),
  ];
  const drop = (btn: HTMLElement, entries: () => MenuEntry[]) => { btn.addEventListener('click', () => { const r = btn.getBoundingClientRect(); contextMenu(r.left, r.bottom, entries()); }); return btn; };
  function renderBar() {
    clear(bar); if (!doc) return;
    const target = sel ?? doc.root; const canAdd = !isBtLeaf(target) && !(isBtDecorator(target) && (target.children?.length ?? 0) >= 1);
    bar.append(drop(h('button', { class: 'btn small primary', disabled: !canAdd, title: t('Add a child to the selected node (or to the root)') }, t('+ Child') + ' ▾'), () => typeMenu((tp) => addChild(tp))),
      drop(h('button', { class: 'btn small', disabled: !sel || sel === doc.root, title: t('Insert a sibling after the selected node') }, t('+ Sibling') + ' ▾'), () => typeMenu((tp) => { if (!sel || !doc) return; const p = btParentOf(doc.root, sel)!; guarded(() => { const n = btAddChild(p, tp, p.children!.indexOf(sel!) + 1); sel = n; }); })),
      h('button', { class: 'btn small', disabled: !sel || sel === doc.root, onClick: () => move(-1), title: 'Alt+↑' }, '↑'), h('button', { class: 'btn small', disabled: !sel || sel === doc.root, onClick: () => move(1), title: 'Alt+↓' }, '↓'),
      h('button', { class: 'btn small', disabled: !sel || sel === doc.root, onClick: () => { if (sel && doc) guarded(() => btOutdent(doc!.root, sel!)); }, title: t('Move out of the parent') }, '⇤'),
      h('button', { class: 'btn small', disabled: !sel || sel === doc.root, onClick: () => remove() }, t('Delete')),
      h('span', { class: 'hint ge-hint' }, t('Click to inspect · drag a node onto a composite to move it inside, onto a leaf to place it after · + Child adds under the selection')));
  }

  function renderInspector() {
    clear(inspector); if (!doc) return;
    const title = (s: string) => h('div', { class: 'prop-title' }, t(s));
    const cat = opts.catalog?.() ?? { robots: [], programs: [], targets: [], zones: [], signals: [] };
    const d = doc;
    if (!sel) {
      inspector.append(title('Behavior tree'), field('Name', text(d.name, (v) => guarded(() => { d.name = v.trim() || d.name; }))),
        field('Supervisor (des model)', selectOrText(cat.desModels ?? [], d.supervisor ?? '', (v) => guarded(() => { d.supervisor = v || undefined; }))),
        field('Mode automaton (hybrid model)', selectOrText(cat.hybridModels ?? [], d.modes ?? '', (v) => guarded(() => { d.modes = v || undefined; }))),
        field('Monitors (LTL, one per line)', area(d.monitors.join('\n'), (v) => guarded(() => { d.monitors = v.split('\n').map((x) => x.trim()).filter(Boolean); }))),
        field('Leaf models (leaf p= ticks=)', area(Object.entries(d.leafModels).map(([k, m]) => `${k}${m.p !== undefined ? ` p=${m.p}` : ''}${m.ticks !== undefined ? ` ticks=${m.ticks}` : ''}`).join('\n'), (v) => guarded(() => { d.leafModels = {}; for (const line of v.split('\n')) { const { words, opts: o } = tokenize(line.trim()); if (words[0]) d.leafModels[words[0]] = { p: o.p as number | undefined, ticks: o.ticks as number | undefined }; } }))),
        field('Outcomes (leaf=success,running)', text(Object.entries(d.outcomes).map(([k, v]) => `${k}=${v.join(',')}`).join(' '), (v) => guarded(() => { d.outcomes = {}; for (const [k, val] of Object.entries(tokenize(v).opts)) d.outcomes[k] = String(val).split(',') as BtDoc['outcomes'][string]; }))),
        h('div', { class: 'hint' }, `${btWalk(d.root).length} ${t('nodes')} · ${d.checks.length} check line(s) kept as text`));
      return;
    }
    const n = sel; const root = d.root;
    const typeSel = h('select', { onChange: (e: Event) => guarded(() => btSetType(n, (e.target as HTMLSelectElement).value as BTNodeType)) }, ...[...BT_COMPOSITES, ...BT_DECORATORS, ...BT_LEAVES].map((tp) => h('option', { value: tp, selected: n.type === tp }, `${BT_GLYPH[tp]} ${tp}`)));
    inspector.append(title(n === root ? 'Root node' : 'Node'), field('Type', typeSel));
    if (!isBtLeaf(n)) {
      inspector.append(field('Name', text(n.name ?? '', (v) => guarded(() => { n.name = v.trim() || n.type; }))));
      if (n.type === 'sequence' || n.type === 'fallback') inspector.append(check('Memory (resume the running child)', !!n.memory, (b) => guarded(() => { n.memory = b || undefined; })));
      if (n.type === 'parallel') inspector.append(field('Success threshold', num(n.threshold, (v) => guarded(() => { n.threshold = v; }), '1')));
      if (n.type === 'retry' || n.type === 'repeat') inspector.append(field('Count', num(n.count, (v) => guarded(() => { n.count = v ?? 1; }), '1')));
      if (n.type === 'timeout') inspector.append(field('Seconds', num(n.seconds, (v) => guarded(() => { n.seconds = v ?? 1; }))));
      inspector.append(h('div', { class: 'hint' }, `${n.children?.length ?? 0} ${t('children')}`));
    } else if (n.type === 'condition') {
      inspector.append(field('Expression or blackboard key', text(n.fn ?? '', (v) => guarded(() => { n.fn = v.trim() || 'true'; }), 'battery < 0.2')), field('Name', text(n.name ?? '', (v) => guarded(() => { n.name = v.trim() || undefined; }))));
    } else {
      const a = (n.args ??= {});
      inspector.append(field('Binding', selectOrText(BT_ACTION_BINDINGS, n.fn ?? '', (v) => guarded(() => { n.fn = v.trim() || 'wait'; }))));
      const argSel = (key: string, label: string, options: string[]) => field(label, selectOrText(options, String(a[key] ?? ''), (v) => guarded(() => { if (v) a[key] = v; else delete a[key]; })));
      if (n.fn === 'goto') inspector.append(argSel('zone', 'Zone', cat.zones));
      if (n.fn === 'program') inspector.append(argSel('name', 'Program', cat.programs.map((p) => p.name)), argSel('robot', 'Robot', cat.robots.filter((r) => !r.mobile).map((r) => r.name)));
      if (n.fn === 'move') inspector.append(argSel('target', 'Target', cat.targets), argSel('robot', 'Robot', cat.robots.filter((r) => !r.mobile).map((r) => r.name)));
      if (n.fn === 'signal') inspector.append(argSel('name', 'Signal', cat.signals), field('Value', text(String(a.value ?? 'true'), (v) => guarded(() => { a.value = v === 'true' ? true : v === 'false' ? false : Number.isFinite(Number(v)) && v.trim() !== '' ? Number(v) : v; }))));
      if (n.fn === 'wait') inspector.append(field('Seconds', num(Number(a.seconds ?? 1), (v) => guarded(() => { a.seconds = v ?? 1; }))));
      if (n.fn === 'event') inspector.append(field('Emit event', text(String(a.emit ?? ''), (v) => guarded(() => { a.emit = v.trim(); }))));
      if (n.fn === 'grasp') inspector.append(field('Success probability p', num(a.p === undefined ? undefined : Number(a.p), (v) => guarded(() => { if (v === undefined) delete a.p; else a.p = v; }))));
      inspector.append(title('Supervisor events'),
        field('event= (controllable, asked before start)', text(String(a.event ?? ''), (v) => guarded(() => { if (v.trim()) a.event = v.trim(); else delete a.event; }))),
        field('done= (observed on success)', text(String(a.done ?? ''), (v) => guarded(() => { if (v.trim()) a.done = v.trim(); else delete a.done; }))),
        field('arrive= / ok= / miss= / put= (world events)', text(['arrive', 'ok', 'miss', 'put'].filter((k) => a[k] !== undefined).map((k) => `${k}=${String(a[k])}`).join(' '), (v) => guarded(() => { for (const k of ['arrive', 'ok', 'miss', 'put']) delete a[k]; Object.assign(a, tokenize(v).opts); }))),
        title('Contract'),
        field('Timeout (s)', num(n.timeout, (v) => guarded(() => { n.timeout = v; }))), field('Precondition', text(n.pre ?? '', (v) => guarded(() => { n.pre = v.trim() || undefined; }), 'expression')), field('Postcondition', text(n.post ?? '', (v) => guarded(() => { n.post = v.trim() || undefined; }), 'held')),
        field('Other arguments (key=value …)', text(Object.entries(a).filter(([k]) => !['zone', 'name', 'robot', 'target', 'value', 'seconds', 'emit', 'p', 'event', 'done', 'arrive', 'ok', 'miss', 'put'].includes(k)).map(([k, v]) => `${k}=${qv(v)}`).join(' '), (v) => guarded(() => { const keep = ['zone', 'name', 'robot', 'target', 'value', 'seconds', 'emit', 'p', 'event', 'done', 'arrive', 'ok', 'miss', 'put']; for (const k of Object.keys(a)) if (!keep.includes(k)) delete a[k]; Object.assign(a, tokenize(v).opts); }))),
        field('Name (optional)', text(n.name ?? '', (v) => guarded(() => { n.name = v.trim() || undefined; }))));
    }
    if (n !== root) inspector.append(h('div', { class: 'btn-row' }, h('button', { class: 'btn small', onClick: () => move(-1) }, '↑'), h('button', { class: 'btn small', onClick: () => move(1) }, '↓'), h('button', { class: 'btn small', onClick: () => remove() }, t('Delete'))));
  }
  const area = (v: string, onChange: (s: string) => void) => { const a = h('textarea', { rows: 3, class: 'ge-text' }) as HTMLTextAreaElement; a.value = v; a.addEventListener('change', () => onChange(a.value)); return a; };
  const selectOrText = (options: string[], value: string, onChange: (v: string) => void) => { if (!options.length) return text(value, (v) => onChange(v.trim())); const list = ['', ...(options.includes(value) || !value ? options : [value, ...options])]; return h('select', { onChange: (e: Event) => onChange((e.target as HTMLSelectElement).value) }, ...list.map((o) => h('option', { value: o, selected: o === value }, o || '—'))); };

  function select(n: BTNodeSpec | null) { sel = n; render(); }
  function addChild(type: BTNodeType): BTNodeSpec | null { if (!doc) return null; const p = sel ?? doc.root; let out: BTNodeSpec | null = null; guarded(() => { out = btAddChild(p, type); sel = out; }); return out; }
  function remove() { if (!doc || !sel || sel === doc.root) return; const n = sel; guarded(() => { btRemove(doc!.root, n); sel = null; }); }
  function move(dir: -1 | 1) { if (!doc || !sel) return; const n = sel; guarded(() => btMove(doc!.root, n, dir)); }
  function load(src: string) { doc = parseBt(src); sel = null; hover = null; drag = null; render(); }
  window.addEventListener('resize', () => { if (doc && el.offsetParent !== null) render(); });
  return { el, load, source, render, select, addChild, remove, move, state: () => ({ doc, selection: sel }) };
}
