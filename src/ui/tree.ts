import { App } from '../app';
import { Item, ItemType, Frame, Target, Tool, SceneObject, Folder, Station } from '../core/items/item';
import { Robot } from '../core/items/robot';
import { Program, Instruction } from '../core/items/program';
import { MobileRobot } from '../mobile/items';
import { ControlModelItem } from '../ctl/model';
import { h, clear, icon, contextMenu, dialog } from './dom';
import { itemContextMenu } from './dialogs';
import { t } from './i18n';
import { VirtualList } from './vlist';

const ICON_OF: Partial<Record<ItemType, string>> = {
  [ItemType.STATION]: 'station', [ItemType.ROBOT]: 'robot', [ItemType.FRAME]: 'frame', [ItemType.TOOL]: 'tool', [ItemType.OBJECT]: 'object', [ItemType.TARGET]: 'target',
  [ItemType.PROGRAM]: 'program', [ItemType.INSTRUCTION]: 'instruction', [ItemType.FOLDER]: 'folder', [ItemType.CAMERA]: 'camera', [ItemType.MOBILE_ROBOT]: 'mobile',
  [ItemType.COMPONENT]: 'component', [ItemType.FLEET]: 'fleet', [ItemType.MAP]: 'map', [ItemType.FIELD]: 'field', [ItemType.MISSION]: 'mission', [ItemType.SENSOR]: 'sensor',
  [ItemType.ZONE]: 'zone', [ItemType.PATH]: 'path', [ItemType.NOTES]: 'notes', [ItemType.CROP_ROW]: 'row', [ItemType.CONTROL_MODEL]: 'control',
};
/** Change reasons that never alter what a tree row shows (they fire every frame during a simulation). */
const SILENT = /^(pose|joints|param:|limits$|chain$|geometry$|kin$|map$|plants$|rows$|camera$|color$|collisionMap$|behaviour$|home$|source$|report$)/;
const ROW_H = 22;

interface Row { item: Item; depth: number; hasChildren: boolean; collapsed: boolean }

/**
 * Station tree panel. The tree is flattened into rows and virtualised (only visible rows exist in the DOM).
 * Structural changes re-flatten; selection / badge changes repaint the visible window in place; joint and pose
 * updates (every simulation frame) are ignored. Listeners are re-bound per station and released on switch.
 */
export class TreePanel {
  el: HTMLElement;
  private collapsed = new Set<string>();
  private filter = '';
  private dragging: Item | null = null;
  private list!: HTMLElement;
  private vlist!: VirtualList<Row>;
  private byId = new Map<string, Item>();
  private rowOf = new Map<string, Row>();
  private offs: Array<() => void> = [];
  /** Number of full re-flattens (diagnostics / tests). */
  rebuilds = 0;

  constructor(readonly app: App) {
    this.el = h('div', { class: 'panel tree-panel' });
    this.buildShell();
    app.events.on('stationChanged', () => { this.bind(); this.render(); });
    app.events.on('activeProgram', () => this.scheduleRepaint());
    app.events.on('activeRobot', () => this.scheduleRepaint());
    this.bind();
    this.render();
    // badges that follow the simulation without item events (fleet robot status / battery): repainted twice a second
    // while the world or a program runs (only rows whose badge text changed are redrawn)
    const live = setInterval(() => { if (this.app.worldRunning || this.app.isPlaying()) this.vlist.repaint(); }, 500);
    (live as unknown as { unref?: () => void }).unref?.();
  }

  private bind() {
    for (const off of this.offs) off();
    const ev = this.app.station.events;
    this.offs = [
      ev.on('childAdded', () => this.scheduleRender()),
      ev.on('childRemoved', () => this.scheduleRender()),
      ev.on('selection', () => this.scheduleRepaint()),
      ev.on('changed', ({ what }) => { if (SILENT.test(what)) return; if (what === 'name' || what === 'batch') this.scheduleRender(); else this.scheduleRepaint(); }),
    ];
  }

  private timer: ReturnType<typeof setTimeout> | null = null;
  private full = false;
  private scheduleRender() { this.full = true; this.schedule(); }
  private scheduleRepaint() { this.schedule(); }
  private schedule() {
    if (this.timer) return;
    this.timer = setTimeout(() => { this.timer = null; if (this.full) this.render(); else this.vlist.repaint(); }, 30);
  }

  private buildShell() {
    clear(this.el);
    const head = h('div', { class: 'panel-head' },
      h('span', null, icon('tree'), ' ' + t('Station')),
      h('input', { class: 'tree-filter', placeholder: 'filter…', value: this.filter, onInput: (e: Event) => { this.filter = (e.target as HTMLInputElement).value.toLowerCase(); this.render(); } }));
    this.list = h('div', { class: 'tree-list' });
    this.el.append(head, this.list);
    this.vlist = new VirtualList<Row>(this.list, ROW_H, {
      key: (r) => r.item.id,
      sig: (r) => this.rowSig(r),
      render: (r) => this.renderRow(r),
    });
    this.wireEvents();
  }

  /** Re-flatten the tree and repaint the visible rows. */
  render(): void {
    this.full = false;
    this.rebuilds++;
    const rows: Row[] = [];
    this.byId.clear();
    this.rowOf.clear();
    const match = this.filter ? this.matchSet() : null;
    const walk = (item: Item, depth: number) => {
      if (match && !match.has(item)) return;
      this.byId.set(item.id, item);
      const hasChildren = item.children.length > 0 && !(item instanceof Program && item.children.every((c) => c instanceof Instruction));
      const collapsed = this.collapsed.has(item.id) || (!this.expandForced.has(item.id) && (item.type === ItemType.CROP_ROW || (item.type === ItemType.FIELD && !this.filter && item.children.length > 40)));
      const row = { item, depth, hasChildren, collapsed };
      this.rowOf.set(item.id, row);
      rows.push(row);
      if (collapsed) return;
      for (const c of item.children) if (!(item instanceof Program && c instanceof Instruction)) walk(c, depth + 1);
    };
    walk(this.app.station, 0);
    this.vlist.setItems(rows);
  }

  /** Items whose subtree contains a name matching the filter (one post-order pass). */
  private matchSet(): Set<Item> {
    const out = new Set<Item>();
    const visit = (it: Item): boolean => {
      let any = it.name.toLowerCase().includes(this.filter);
      for (const c of it.children) if (visit(c)) any = true;
      if (any) out.add(it);
      return any;
    };
    visit(this.app.station);
    return out;
  }

  private badgeOf(item: Item): [string, string] | null {
    if (item instanceof Robot) return [`${item.dof} DOF`, ''];
    if (item instanceof Program) { const n = item.instructions().length; const r = item.lastResult; return [`${n} ins${r ? ` · ${r.duration.toFixed(1)} s` : ''}`, r && !r.ok ? 'bad' : '']; }
    if (item instanceof Target) return item.isJointTarget ? ['J', ''] : null;
    if (item instanceof MobileRobot) return [`${item.state.status} ${(item.batteryLevel() * 100).toFixed(0)}%`, item.state.status];
    if (item instanceof Tool && item.parent instanceof Robot && item.parent.activeTool() === item) return ['active', ''];
    if (item instanceof Frame && this.app.activeRobot?.activeFrame() === item) return ['ref', ''];
    if (item instanceof ControlModelItem) return [`${item.kind}${item.lastOk === null ? '' : item.lastOk ? ' ✓' : ' ✗'}`, item.lastOk === false ? 'bad' : ''];
    return null;
  }

  private rowSig(r: Row): string {
    const it = r.item;
    const b = this.badgeOf(it);
    return `${r.depth}|${r.hasChildren ? 1 : 0}${r.collapsed ? 1 : 0}|${it.selected ? 1 : 0}${it.visible ? 1 : 0}${it === this.app.activeProgram || it === this.app.activeRobot ? 1 : 0}|${it.name}|${b ? b.join('/') : ''}`;
  }

  private renderRow(r: Row): HTMLElement {
    const item = r.item;
    const b = this.badgeOf(item);
    return h('div', { class: `tree-row ${item.selected ? 'selected' : ''} ${!item.visible ? 'hidden-item' : ''} ${item === this.app.activeProgram || item === this.app.activeRobot ? 'active' : ''}`, style: { paddingLeft: `${8 + r.depth * 14}px` }, draggable: item.type !== ItemType.STATION, dataset: { id: item.id } },
      h('span', { class: `twisty ${r.hasChildren ? '' : 'empty'}` }, r.hasChildren ? (r.collapsed ? '▸' : '▾') : ''),
      icon(ICON_OF[item.type] ?? 'object'),
      h('span', { class: 'tree-name' }, item.name, b ? h('small', { class: `badge ${b[1]}` }, b[0]) : null),
      item.type !== ItemType.STATION && item.type !== ItemType.INSTRUCTION ? h('span', { class: 'tree-eye', title: 'Toggle visibility' }, item.visible ? '👁' : '◌') : null);
  }

  private rowAt(e: Event): { row: HTMLElement; item: Item } | null {
    const row = (e.target as HTMLElement | null)?.closest?.('.tree-row') as HTMLElement | null;
    if (!row || !this.list.contains(row)) return null;
    const item = this.byId.get(row.dataset.id ?? '') ?? this.app.station.findById(row.dataset.id ?? '');
    return item ? { row, item } : null;
  }

  /** All row interaction is delegated to the list, so rows can be recycled freely. */
  private wireEvents() {
    const L = this.list;
    L.addEventListener('click', (e) => {
      const hit = this.rowAt(e); if (!hit) return;
      const tgt = e.target as HTMLElement;
      if (tgt.closest('.twisty')) {
        const id = hit.item.id;
        if (this.rowOf.get(id)?.collapsed) { this.collapsed.delete(id); this.expandForced.add(id); } else { this.collapsed.add(id); this.expandForced.delete(id); }
        this.render();
        return;
      }
      if (tgt.closest('.tree-eye')) { hit.item.setVisible(!hit.item.visible); return; }
      this.app.select(hit.item, (e as MouseEvent).ctrlKey || (e as MouseEvent).shiftKey);
    });
    L.addEventListener('dblclick', (e) => {
      const hit = this.rowAt(e); if (!hit) return;
      const item = hit.item;
      if ((e.target as HTMLElement).closest('.tree-name')) { void this.rename(item); return; }
      if (item instanceof Target) this.app.moveRobotTo(item); else if (item instanceof Program) this.app.runProgram(item); else this.app.renderer.focusItem(item);
    });
    L.addEventListener('contextmenu', (e) => {
      e.preventDefault();
      const hit = this.rowAt(e);
      if (!hit) { if (e.target === L || (e.target as HTMLElement).classList?.contains('vlist-pad')) contextMenu(e.clientX, e.clientY, itemContextMenu(this.app, this.app.station)); return; }
      if (!hit.item.selected) this.app.select(hit.item);
      contextMenu(e.clientX, e.clientY, itemContextMenu(this.app, hit.item));
    });
    L.addEventListener('dragstart', (e) => { const hit = this.rowAt(e); if (!hit) return; this.dragging = hit.item; e.dataTransfer?.setData('text/plain', hit.item.id); });
    L.addEventListener('dragover', (e) => { const hit = this.rowAt(e); if (!hit) return; const d = this.dragging; if (d && d !== hit.item && !d.isAncestorOf(hit.item)) { e.preventDefault(); hit.row.classList.add('drop'); } });
    L.addEventListener('dragleave', (e) => { this.rowAt(e)?.row.classList.remove('drop'); });
    L.addEventListener('drop', (e) => {
      const hit = this.rowAt(e); if (!hit) return;
      e.preventDefault(); hit.row.classList.remove('drop');
      const item = hit.item;
      const d = this.dragging; this.dragging = null;
      if (!d || d === item || d.isAncestorOf(item)) return;
      if (d instanceof Instruction && item instanceof Instruction && d.parent === item.parent) { this.app.cmd(() => item.parent!.addChild(d, item.parent!.children.indexOf(item))); return; }
      if (d instanceof Instruction) return;
      this.app.cmd(() => { if ((e as DragEvent).shiftKey) d.setParent(item); else d.setParentStatic(item); });
    });
  }
  /** Fields / crop rows the user expanded explicitly (they are collapsed by default). */
  private expandForced = new Set<string>();

  /** Scroll the row of `item` into view (selection from the 3D view). */
  reveal(item: Item): void {
    const n = this.vlist.length;
    for (let i = 0; i < n; i++) if (this.vlist.at(i)!.item === item) { this.vlist.scrollTo(i); return; }
  }

  async rename(item: Item): Promise<void> {
    const r = await dialog<{ name: string }>('Rename', [{ key: 'name', label: 'Name', type: 'text', value: item.name }]);
    if (r && r.name.trim()) this.app.cmd(() => item.setName(r.name.trim()));
  }
}

export { Folder, SceneObject, Station };
