import { App } from '../app';
import { Item, ItemType, Frame, Target, Tool, SceneObject, Folder } from '../core/items/item';
import { Robot } from '../core/items/robot';
import { Program, Instruction } from '../core/items/program';
import { MobileRobot } from '../mobile/items';
import { ControlModelItem } from '../ctl/model';
import { h, clear, icon, contextMenu, MenuEntry, dialog } from './dom';
import { itemContextMenu } from './dialogs';
import { t } from './i18n';

const ICON_OF: Partial<Record<ItemType, string>> = {
  [ItemType.STATION]: 'station', [ItemType.ROBOT]: 'robot', [ItemType.FRAME]: 'frame', [ItemType.TOOL]: 'tool', [ItemType.OBJECT]: 'object', [ItemType.TARGET]: 'target',
  [ItemType.PROGRAM]: 'program', [ItemType.INSTRUCTION]: 'instruction', [ItemType.FOLDER]: 'folder', [ItemType.CAMERA]: 'camera', [ItemType.MOBILE_ROBOT]: 'mobile',
  [ItemType.COMPONENT]: 'component', [ItemType.FLEET]: 'fleet', [ItemType.MAP]: 'map', [ItemType.FIELD]: 'field', [ItemType.MISSION]: 'mission', [ItemType.SENSOR]: 'sensor',
  [ItemType.ZONE]: 'zone', [ItemType.PATH]: 'path', [ItemType.NOTES]: 'notes', [ItemType.CROP_ROW]: 'row', [ItemType.CONTROL_MODEL]: 'control',
};

/** Station tree panel. */
export class TreePanel {
  el: HTMLElement;
  private collapsed = new Set<string>();
  private filter = '';
  private dragging: Item | null = null;

  constructor(readonly app: App) {
    this.el = h('div', { class: 'panel tree-panel' });
    app.events.on('stationChanged', () => this.render());
    app.station.events.on('changed', () => this.scheduleRender());
    const bind = () => {
      app.station.events.on('changed', () => this.scheduleRender());
      app.station.events.on('childAdded', () => this.scheduleRender());
      app.station.events.on('childRemoved', () => this.scheduleRender());
      app.station.events.on('selection', () => this.scheduleRender());
    };
    bind();
    app.events.on('stationChanged', bind);
    this.render();
  }

  private timer: number | null = null;
  private scheduleRender() {
    if (this.timer) return;
    this.timer = window.setTimeout(() => { this.timer = null; this.render(); }, 30);
  }

  render(): void {
    clear(this.el);
    const head = h('div', { class: 'panel-head' },
      h('span', null, icon('tree'), ' ' + t('Station')),
      h('input', { class: 'tree-filter', placeholder: 'filter…', value: this.filter, onInput: (e: Event) => { this.filter = (e.target as HTMLInputElement).value.toLowerCase(); this.render(); } }));
    this.el.appendChild(head);
    const list = h('div', { class: 'tree-list' });
    this.renderNode(this.app.station, list, 0);
    this.el.appendChild(list);
    list.addEventListener('contextmenu', (e) => { e.preventDefault(); if (e.target === list) contextMenu(e.clientX, e.clientY, itemContextMenu(this.app, this.app.station)); });
  }

  private matches(item: Item): boolean {
    if (!this.filter) return true;
    for (const it of item.walk()) if (it.name.toLowerCase().includes(this.filter)) return true;
    return false;
  }

  private renderNode(item: Item, list: HTMLElement, depth: number) {
    if (!this.matches(item)) return;
    const hasChildren = item.children.length > 0 && !(item instanceof Program && item.children.every((c) => c instanceof Instruction));
    const collapsed = this.collapsed.has(item.id) || (item.type === ItemType.CROP_ROW) || (item.type === ItemType.FIELD && !this.filter && item.children.length > 40);
    const row = h('div', { class: `tree-row ${item.selected ? 'selected' : ''} ${!item.visible ? 'hidden-item' : ''} ${item === this.app.activeProgram || item === this.app.activeRobot ? 'active' : ''}`, style: { paddingLeft: `${8 + depth * 14}px` }, draggable: item.type !== ItemType.STATION, dataset: { id: item.id } },
      h('span', { class: `twisty ${hasChildren ? '' : 'empty'}`, onClick: (e: MouseEvent) => { e.stopPropagation(); if (collapsed) this.collapsed.delete(item.id); else this.collapsed.add(item.id); this.render(); } }, hasChildren ? (collapsed ? '▸' : '▾') : ''),
      icon(ICON_OF[item.type] ?? 'object'),
      h('span', { class: 'tree-name', onDblClick: (e: MouseEvent) => { e.stopPropagation(); this.rename(item); } }, item.name, this.badge(item)),
      item.type !== ItemType.STATION && item.type !== ItemType.INSTRUCTION ? h('span', { class: 'tree-eye', title: 'Toggle visibility', onClick: (e: MouseEvent) => { e.stopPropagation(); item.setVisible(!item.visible); } }, item.visible ? '👁' : '◌') : null);
    row.addEventListener('click', (e) => { this.app.select(item, e.ctrlKey || e.shiftKey); });
    row.addEventListener('dblclick', () => { if (item instanceof Target) this.app.moveRobotTo(item); else if (item instanceof Program) this.app.runProgram(item); else this.app.renderer.focusItem(item); });
    row.addEventListener('contextmenu', (e) => { e.preventDefault(); e.stopPropagation(); if (!item.selected) this.app.select(item); contextMenu(e.clientX, e.clientY, itemContextMenu(this.app, item)); });
    row.addEventListener('dragstart', (e) => { this.dragging = item; e.dataTransfer?.setData('text/plain', item.id); });
    row.addEventListener('dragover', (e) => { if (this.dragging && this.dragging !== item && !this.dragging.isAncestorOf(item)) { e.preventDefault(); row.classList.add('drop'); } });
    row.addEventListener('dragleave', () => row.classList.remove('drop'));
    row.addEventListener('drop', (e) => {
      e.preventDefault(); row.classList.remove('drop');
      const d = this.dragging; this.dragging = null;
      if (!d || d === item || d.isAncestorOf(item)) return;
      if (d instanceof Instruction && item instanceof Instruction && d.parent === item.parent) { this.app.cmd(() => item.parent!.addChild(d, item.parent!.children.indexOf(item))); return; }
      if (d instanceof Instruction) return;
      this.app.cmd(() => { if (e.shiftKey) d.setParent(item); else d.setParentStatic(item); });
    });
    list.appendChild(row);
    if (!collapsed) {
      const kids = item instanceof Program ? item.children.filter((c) => !(c instanceof Instruction)) : item.children;
      for (const c of kids) this.renderNode(c, list, depth + 1);
    }
  }

  private badge(item: Item): Node | null {
    if (item instanceof Robot) return h('small', { class: 'badge' }, `${item.dof} DOF`);
    if (item instanceof Program) { const n = item.instructions().length; const r = item.lastResult; return h('small', { class: `badge ${r && !r.ok ? 'bad' : ''}` }, `${n} ins${r ? ` · ${r.duration.toFixed(1)} s` : ''}`); }
    if (item instanceof Target) return item.isJointTarget ? h('small', { class: 'badge' }, 'J') : null;
    if (item instanceof MobileRobot) return h('small', { class: `badge ${item.state.status}` }, `${item.state.status} ${(item.batteryLevel() * 100).toFixed(0)}%`);
    if (item instanceof Tool && item.parent instanceof Robot && item.parent.activeTool() === item) return h('small', { class: 'badge' }, 'active');
    if (item instanceof Frame && this.app.activeRobot?.activeFrame() === item) return h('small', { class: 'badge' }, 'ref');
    if (item instanceof ControlModelItem) return h('small', { class: `badge ${item.lastOk === false ? 'bad' : ''}` }, `${item.kind}${item.lastOk === null ? '' : item.lastOk ? ' ✓' : ' ✗'}`);
    return null;
  }

  async rename(item: Item): Promise<void> {
    const r = await dialog<{ name: string }>('Rename', [{ key: 'name', label: 'Name', type: 'text', value: item.name }]);
    if (r && r.name.trim()) this.app.cmd(() => item.setName(r.name.trim()));
  }
}

export { Folder, SceneObject };
