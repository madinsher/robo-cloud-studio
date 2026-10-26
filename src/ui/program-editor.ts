import { App } from '../app';
import { Program, Instruction, InstructionData } from '../core/items/program';
import { Item } from '../core/items/item';
import { VirtualList } from './vlist';
import { Target, ItemType, Frame, Tool } from '../core/items/item';
import { Robot } from '../core/items/robot';
import { h, clear, icon, contextMenu, fmt } from './dom';
import { exportDialog } from './dialogs';

/**
 * Program editor: instruction list of the active program with add/reorder/step controls. The list is virtualised
 * (programs of 5k+ instructions), rows are recycled by signature and joint/pose updates never rebuild it; the
 * running instruction and the first-error marker of the live run are repainted in place.
 */
export class ProgramEditor {
  el: HTMLElement;
  private activeIndex = -1;
  private errorIndex = -1;
  private head!: HTMLElement;
  private body!: HTMLElement;
  private foot: HTMLElement | null = null;
  private vlist: VirtualList<Instruction> | null = null;
  private offs: Array<() => void> = [];
  private program: Program | null = null;
  /** Full renders (diagnostics / tests). */
  rebuilds = 0;

  constructor(readonly app: App) {
    this.el = h('div', { class: 'program-editor' });
    app.events.on('activeProgram', () => this.render());
    app.events.on('stationChanged', () => { this.bind(); this.render(); });
    app.events.on('simulation', () => this.syncRun());
    app.events.on('preview', () => this.repaint(true));
    this.bind();
    this.render();
  }

  private bind() {
    for (const off of this.offs) off();
    const ev = this.app.station.events;
    const structural = ({ parent, child }: { parent: Item; child: Item }) => { if (parent === this.program || child instanceof Program || parent instanceof Program) this.scheduleRender(); };
    this.offs = [
      ev.on('childAdded', structural),
      ev.on('childRemoved', structural),
      ev.on('selection', () => this.scheduleRepaint()),
      ev.on('changed', ({ item, what }) => {
        if (what === 'pose' || what === 'joints' || what.startsWith('param:')) return;
        if (item instanceof Program && what === 'name') this.scheduleRender();
        else if (item.parent === this.program || item instanceof Target || item === this.program) this.scheduleRepaint();
      }),
    ];
  }

  private timer: ReturnType<typeof setTimeout> | null = null;
  private full = false;
  private scheduleRender() { this.full = true; this.schedule(); }
  private scheduleRepaint() { this.schedule(); }
  private schedule() { if (this.timer) return; this.timer = setTimeout(() => { this.timer = null; if (this.full) this.render(); else this.repaint(true); }, 30); }

  /** Running instruction / error marker of the live run for this program. */
  private syncRun() {
    const p = this.program;
    if (!p) return;
    const i = this.app.runningInstruction(p);
    const err = this.app.errorInstruction(p);
    if (i !== this.activeIndex || err !== this.errorIndex) {
      this.activeIndex = i; this.errorIndex = err;
      this.repaint(false);
      if (i >= 0) this.vlist?.scrollTo(i);
    }
  }

  private repaint(withFoot: boolean) {
    if (!this.vlist || this.program !== this.app.activeProgram) { this.render(); return; }
    const p = this.program!;
    const ins = p.instructions();
    if (ins.length !== this.vlist.length || ins.some((x, i) => this.vlist!.at(i) !== x)) this.vlist.setItems(ins);
    else this.vlist.repaint();
    if (withFoot) this.renderFoot();
  }

  render(): void {
    this.full = false;
    this.rebuilds++;
    clear(this.el);
    const p = this.app.activeProgram;
    this.program = p;
    this.activeIndex = p ? this.app.runningInstruction(p) : -1;
    this.errorIndex = p ? this.app.errorInstruction(p) : -1;
    const programs = this.app.station.itemsOfType<Program>(ItemType.PROGRAM);
    const sel = h('select', { class: 'prog-select', onChange: (e: Event) => { const id = (e.target as HTMLSelectElement).value; const pr = this.app.station.findById(id) as Program | null; this.app.setActiveProgram(pr); if (pr) this.app.select(pr); } }, h('option', { value: '' }, '(no program)'), ...programs.map((x) => h('option', { value: x.id, selected: x === p }, x.name)));
    this.head = h('div', { class: 'pe-head' }, icon('program'), sel,
      h('button', { class: 'btn small', title: 'New program', onClick: () => this.app.addProgram() }, '+ New'),
      p ? h('button', { class: 'btn small primary', onClick: () => this.app.runProgram(p) }, '▶ Run') : null,
      p ? h('button', { class: 'btn small', onClick: () => { this.app.validateProgram(p); this.render(); } }, 'Validate') : null,
      p ? h('button', { class: 'btn small', onClick: () => exportDialog(this.app, p) }, 'Export…') : null,
      p ? h('span', { class: 'pe-add' }, 'Add: ', ...this.addButtons(p)) : null);
    this.el.appendChild(this.head);
    this.vlist = null;
    this.foot = null;
    if (!p) { this.el.appendChild(h('div', { class: 'hint pad' }, 'Create a program (Program > New) or select one. Use "Teach" on a robot to record targets.')); return; }
    this.body = h('div', { class: 'ins-list' });
    this.el.appendChild(this.body);
    this.vlist = new VirtualList<Instruction>(this.body, 22, { key: (ins) => ins.id, sig: (ins, i) => this.rowSig(p, ins, i), render: (ins, i) => this.renderRow(p, ins, i) });
    this.wireEvents(p);
    this.vlist.setItems(p.instructions());
    this.foot = h('div', { class: 'pe-foot' });
    this.el.appendChild(this.foot);
    this.renderFoot();
  }

  private renderFoot() {
    const res = this.program?.lastResult;
    if (!this.foot) return;
    // RoboDK's cycle time (Item.Update()), plus the playback time when programmed speeds make it differ
    this.foot.textContent = res ? `Cycle time ${(res.updateTime ?? res.duration).toFixed(2)} s${res.updateTime !== undefined && Math.abs(res.updateTime - res.duration) > 0.005 ? ` (playback ${res.duration.toFixed(2)} s)` : ''} · TCP distance ${(res.distance / 1000).toFixed(2)} m · ${res.ok ? 'valid' : `${res.problems.filter((x) => x.severity === 'error').length} error(s)`}` : '';
    this.foot.style.display = res ? '' : 'none';
  }

  private problemOf(p: Program, ins: Instruction) {
    return this.app.problemsOf(p).get(ins.id) ?? null;
  }

  private rowSig(p: Program, ins: Instruction, i: number): string {
    const d = ins.data;
    const target = d.kind === 'move' && d.targetId ? this.app.station.findById(d.targetId) : null;
    const step = this.app.stepOf(ins);
    const pr = this.problemOf(p, ins);
    return `${i}|${ins.selected ? 1 : 0}${ins.enabled ? 1 : 0}|${i === this.activeIndex ? 1 : 0}${i === this.errorIndex ? 1 : 0}|${ins.name}|${target?.name ?? ''}|${step?.trajectory ? step.trajectory.duration.toFixed(2) : ''}|${pr ? pr.severity + pr.message : ''}|${d.kind === 'move' && d.joints ? d.joints.join(',') : ''}`;
  }

  private renderRow(p: Program, ins: Instruction, i: number): HTMLElement {
    const problem = this.problemOf(p, ins);
    const d = ins.data;
    const target = d.kind === 'move' && d.targetId ? (this.app.station.findById(d.targetId) as Target | null) : null;
    const step = this.app.stepOf(ins);
    return h('div', { class: `ins-row ${ins.selected ? 'selected' : ''} ${!ins.enabled ? 'disabled' : ''} ${problem ? problem.severity : ''} ${i === this.activeIndex ? 'running' : ''} ${i === this.errorIndex ? 'error error-at' : ''}`, draggable: true, dataset: { id: ins.id, index: String(i) } },
      h('span', { class: 'ins-idx' }, String(i + 1)),
      h('span', { class: 'ins-kind' }, kindLabel(d)),
      h('span', { class: 'ins-name' }, d.kind === 'move' ? (target ? target.name : d.joints ? `[${d.joints.map((v) => fmt(v, 0)).join(', ')}]` : 'pose') : ins.name.replace(/^(Set Speed|Pause|Event: |Code: |\/\/ |Show )/, '')),
      step && step.trajectory ? h('span', { class: 'ins-time' }, `${step.trajectory.duration.toFixed(2)} s`) : null,
      problem ? h('span', { class: 'ins-problem', title: problem.message }, '⚠ ', problem.message.slice(0, 40)) : null,
      h('span', { class: 'ins-actions' },
        h('button', { class: 'btn-icon', title: 'Move up', dataset: { act: 'up' } }, '↑'),
        h('button', { class: 'btn-icon', title: 'Move down', dataset: { act: 'down' } }, '↓'),
        h('button', { class: 'btn-icon', title: 'Delete', dataset: { act: 'del' } }, '✕')));
  }

  private hit(e: Event): { row: HTMLElement; ins: Instruction; i: number } | null {
    const row = (e.target as HTMLElement | null)?.closest?.('.ins-row') as HTMLElement | null;
    if (!row || !this.body.contains(row)) return null;
    const i = Number(row.dataset.index);
    const ins = this.vlist?.at(i);
    return ins && ins.id === row.dataset.id ? { row, ins, i } : null;
  }

  /** Row events are delegated to the list (rows are recycled by the virtual list). */
  private wireEvents(p: Program) {
    const B = this.body;
    B.addEventListener('click', (e) => {
      const x = this.hit(e); if (!x) return;
      const act = (e.target as HTMLElement).closest<HTMLElement>('[data-act]')?.dataset.act;
      if (act === 'up') return this.move(p, x.ins, -1);
      if (act === 'down') return this.move(p, x.ins, 1);
      if (act === 'del') { this.app.deleteItems([x.ins]); this.app.previewProgram(); return; }
      this.app.select(x.ins);
    });
    B.addEventListener('dblclick', (e) => {
      const x = this.hit(e); if (!x) return;
      const d = x.ins.data;
      const target = d.kind === 'move' && d.targetId ? (this.app.station.findById(d.targetId) as Target | null) : null;
      const robot = p.robot();
      const step = this.app.stepOf(x.ins);
      if (target && robot instanceof Robot) { this.app.setActiveRobot(robot); this.app.moveRobotTo(target, d.kind === 'move' && d.moveType === 'MoveL'); } else if (step) this.app.seekProgram(step.t1);
    });
    B.addEventListener('contextmenu', (e) => {
      const x = this.hit(e); if (!x) return;
      e.preventDefault();
      const { ins, i } = x;
      contextMenu((e as MouseEvent).clientX, (e as MouseEvent).clientY, [
        { label: 'Run from here', action: () => this.app.runProgram(p, { fromIndex: i }) },
        { label: ins.enabled ? 'Disable' : 'Enable', action: () => { this.app.cmd(() => { ins.enabled = !ins.enabled; }); this.app.previewProgram(); } },
        { label: 'Insert MoveJ (teach) after', action: () => this.insertTeach(p, p.children.indexOf(ins) + 1, 'MoveJ') },
        { label: 'Insert MoveL (teach) after', action: () => this.insertTeach(p, p.children.indexOf(ins) + 1, 'MoveL') },
        { separator: true },
        { label: 'Delete', action: () => { this.app.deleteItems([ins]); this.app.previewProgram(); } },
      ]);
    });
    B.addEventListener('dragstart', (e) => { const x = this.hit(e); if (x) (e as DragEvent).dataTransfer?.setData('text/plain', x.ins.id); });
    B.addEventListener('dragover', (e) => { const x = this.hit(e); if (!x) return; e.preventDefault(); x.row.classList.add('drop'); });
    B.addEventListener('dragleave', (e) => { this.hit(e)?.row.classList.remove('drop'); });
    B.addEventListener('drop', (e) => {
      const x = this.hit(e); if (!x) return;
      e.preventDefault(); x.row.classList.remove('drop');
      const id = (e as DragEvent).dataTransfer?.getData('text/plain');
      const src = id ? (this.app.station.findById(id) as Instruction | null) : null;
      if (src && src.parent === p && src !== x.ins) { this.app.cmd(() => p.addChild(src, p.children.indexOf(x.ins))); this.app.previewProgram(); }
    });
  }

  private addButtons(p: Program): HTMLElement[] {
    const add = (data: InstructionData) => { const sel = this.app.station.selection[0]; const idx = sel instanceof Instruction && sel.parent === p ? p.children.indexOf(sel) + 1 : undefined; this.app.cmd(() => p.addInstruction(data, idx)); this.app.previewProgram(); };
    const b = (label: string, fn: () => void, title?: string) => h('button', { class: 'btn small', title, onClick: fn }, label);
    return [
      b('MoveJ', () => this.insertTeach(p, undefined, 'MoveJ'), 'Teach current robot position as MoveJ'),
      b('MoveL', () => this.insertTeach(p, undefined, 'MoveL'), 'Teach current robot position as MoveL'),
      b('MoveC', () => { const ts = this.app.station.itemsOfType<Target>(ItemType.TARGET); if (ts.length >= 2) add({ kind: 'move', moveType: 'MoveC', targetId: ts[ts.length - 1].id, viaTargetId: ts[ts.length - 2].id }); }),
      b('Speed', () => add({ kind: 'speed', speedLinear: 500, speedJoints: 90 })),
      b('Round', () => add({ kind: 'rounding', radius: 10 })),
      b('Frame', () => { const f = this.app.station.itemsOfType<Frame>(ItemType.FRAME)[0]; add({ kind: 'frame', frameId: f?.id ?? null }); }),
      b('Tool', () => { const r = p.robot(); const t = r instanceof Robot ? r.activeTool() : null; add({ kind: 'tool', toolId: t?.id ?? null }); }),
      b('Pause', () => add({ kind: 'pause', timeMs: 1000 })),
      b('Set DO', () => add({ kind: 'io', io: 'DO_1', value: true, wait: false })),
      b('Wait DI', () => add({ kind: 'io', io: 'DI_1', value: true, wait: true })),
      b('Grip', () => add({ kind: 'event', action: 'gripper_close' })),
      b('Release', () => add({ kind: 'event', action: 'gripper_open' })),
      b('Code', () => add({ kind: 'code', code: 'MyRoutine', asFunctionCall: true })),
      b('Comment', () => add({ kind: 'print', message: 'comment', isComment: true })),
      b('Call', () => { const other = this.app.station.itemsOfType<Program>(ItemType.PROGRAM).find((x) => x !== p); add({ kind: 'call', programId: other?.id ?? null, programName: other?.name }); }),
      b('Navigate', () => add({ kind: 'mobile_move', x: 0, y: 0, speed: 1000 })),
      b('Signal', () => add({ kind: 'signal', signal: 'spray', value: true, wait: false })),
      b('Thread', () => { const other = this.app.station.itemsOfType<Program>(ItemType.PROGRAM).find((x) => x !== p); add({ kind: 'thread', programId: other?.id ?? null, programName: other?.name }); }),
      b('Wait', () => add({ kind: 'wait', what: 'signal', signal: 'DI_1', value: true, timeMs: 1000 })),
    ];
  }

  private insertTeach(p: Program, index: number | undefined, moveType: 'MoveJ' | 'MoveL') {
    const robotItem = p.robot();
    const robot = robotItem instanceof Robot ? robotItem : this.app.activeRobot;
    if (!robot) return this.app.log('Program has no robot', 'warn');
    this.app.setActiveRobot(robot);
    const frame = robot.activeFrame() ?? this.app.station;
    this.app.cmd(() => {
      const t = new Target(`Target ${this.app.station.itemsOfType(ItemType.TARGET).length + 1}`);
      frame.addChild(t);
      t.setPoseAbs(robot.poseTCPAbs());
      t.setJoints(robot.joints());
      t.robotId = robot.id;
      if (moveType === 'MoveJ') t.setAsJointTarget();
      const sel = this.app.station.selection[0];
      const idx = index ?? (sel instanceof Instruction && sel.parent === p ? p.children.indexOf(sel) + 1 : undefined);
      p.addInstruction({ kind: 'move', moveType, targetId: t.id }, idx);
    });
    this.app.previewProgram();
  }

  private move(p: Program, ins: Instruction, dir: number) {
    const i = p.children.indexOf(ins);
    const j = i + dir;
    if (j < 0 || j >= p.children.length) return;
    this.app.cmd(() => p.addChild(ins, j));
    this.app.previewProgram();
  }
}

function kindLabel(d: InstructionData): string {
  switch (d.kind) {
    case 'move': return d.moveType;
    case 'jointPath': return 'Path';
    case 'speed': return 'Speed';
    case 'frame': return 'Frame';
    case 'tool': return 'Tool';
    case 'pause': return 'Pause';
    case 'event': return d.action.replace('_', ' ');
    case 'code': return d.asFunctionCall ? 'Call' : 'Code';
    case 'print': return d.isComment ? '//' : 'Msg';
    case 'rounding': return 'Round';
    case 'io': return d.wait ? 'WaitDI' : 'SetDO';
    case 'call': return 'Call';
    case 'mobile_move': return 'Navigate';
    case 'mobile_follow': return 'Follow';
    case 'mission_task': return 'Task';
    case 'signal': return d.wait ? 'WaitSig' : 'SetSig';
    case 'loop': return 'Loop';
    case 'if': return 'If';
    case 'thread': return 'Thread';
    case 'wait': return 'Wait';
  }
}
export { Tool };
