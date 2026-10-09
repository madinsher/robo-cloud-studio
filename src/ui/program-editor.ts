import { App } from '../app';
import { Program, Instruction, InstructionData } from '../core/items/program';
import { Target, ItemType, Frame, Tool } from '../core/items/item';
import { Robot } from '../core/items/robot';
import { h, clear, icon, contextMenu, fmt } from './dom';
import { exportDialog } from './dialogs';

/** Program editor: instruction list of the active program with add/reorder/step controls. */
export class ProgramEditor {
  el: HTMLElement;
  private activeIndex = -1;

  constructor(readonly app: App) {
    this.el = h('div', { class: 'program-editor' });
    app.events.on('activeProgram', () => this.render());
    app.events.on('stationChanged', () => this.render());
    const bind = () => { app.station.events.on('changed', () => this.scheduleRender()); app.station.events.on('childAdded', () => this.scheduleRender()); app.station.events.on('childRemoved', () => this.scheduleRender()); app.station.events.on('selection', () => this.scheduleRender()); };
    bind();
    app.events.on('stationChanged', bind);
    const bindSim = () => app.sim.events.on('instruction', ({ instruction }) => { const p = this.app.activeProgram; if (!p) return; const i = p.instructions().indexOf(instruction); if (i !== this.activeIndex) { this.activeIndex = i; this.highlight(); } });
    bindSim();
    app.events.on('stationChanged', bindSim);
    app.events.on('simulation', ({ time }) => { const idx = this.app.sim.stepIndexAt(time); const step = this.app.sim.steps[idx]; const p = this.app.activeProgram; if (!p || !step) return; const i = p.instructions().indexOf(step.instruction); if (i !== this.activeIndex) { this.activeIndex = i; this.highlight(); } });
    this.render();
  }

  private timer: number | null = null;
  private scheduleRender() { if (this.timer) return; this.timer = window.setTimeout(() => { this.timer = null; this.render(); }, 30); }

  private highlight() {
    this.el.querySelectorAll('.ins-row').forEach((r, i) => r.classList.toggle('running', i === this.activeIndex));
  }

  render(): void {
    clear(this.el);
    const p = this.app.activeProgram;
    const programs = this.app.station.itemsOfType<Program>(ItemType.PROGRAM);
    const sel = h('select', { class: 'prog-select', onChange: (e: Event) => { const id = (e.target as HTMLSelectElement).value; const pr = this.app.station.findById(id) as Program | null; this.app.setActiveProgram(pr); if (pr) this.app.select(pr); } }, h('option', { value: '' }, '(no program)'), ...programs.map((x) => h('option', { value: x.id, selected: x === p }, x.name)));
    const head = h('div', { class: 'pe-head' }, icon('program'), sel,
      h('button', { class: 'btn small', title: 'New program', onClick: () => this.app.addProgram() }, '+ New'),
      p ? h('button', { class: 'btn small primary', onClick: () => this.app.runProgram(p) }, '▶ Run') : null,
      p ? h('button', { class: 'btn small', onClick: () => { this.app.previewProgram(); this.render(); } }, 'Validate') : null,
      p ? h('button', { class: 'btn small', onClick: () => exportDialog(this.app, p) }, 'Export…') : null,
      p ? h('span', { class: 'pe-add' }, 'Add: ', ...this.addButtons(p)) : null);
    this.el.appendChild(head);
    if (!p) { this.el.appendChild(h('div', { class: 'hint pad' }, 'Create a program (Program > New) or select one. Use "Teach" on a robot to record targets.')); return; }
    const res = p.lastResult;
    const list = h('div', { class: 'ins-list' });
    const robot = p.robot();
    p.instructions().forEach((ins, i) => {
      const problem = res?.problems.find((pr) => pr.instructionId === ins.id);
      const d = ins.data;
      const target = d.kind === 'move' && d.targetId ? (this.app.station.findById(d.targetId) as Target | null) : null;
      const step = this.app.sim.steps.find((s) => s.instruction === ins);
      const row = h('div', { class: `ins-row ${ins.selected ? 'selected' : ''} ${!ins.enabled ? 'disabled' : ''} ${problem ? problem.severity : ''} ${i === this.activeIndex ? 'running' : ''}`, draggable: true, dataset: { id: ins.id } },
        h('span', { class: 'ins-idx' }, String(i + 1)),
        h('span', { class: 'ins-kind' }, kindLabel(d)),
        h('span', { class: 'ins-name' }, d.kind === 'move' ? (target ? target.name : d.joints ? `[${d.joints.map((v) => fmt(v, 0)).join(', ')}]` : 'pose') : ins.name.replace(/^(Set Speed|Pause|Event: |Code: |\/\/ |Show )/, '')),
        step && step.trajectory ? h('span', { class: 'ins-time' }, `${step.trajectory.duration.toFixed(2)} s`) : null,
        problem ? h('span', { class: 'ins-problem', title: problem.message }, '⚠ ', problem.message.slice(0, 40)) : null,
        h('span', { class: 'ins-actions' },
          h('button', { class: 'btn-icon', title: 'Move up', onClick: (e: MouseEvent) => { e.stopPropagation(); this.move(p, ins, -1); } }, '↑'),
          h('button', { class: 'btn-icon', title: 'Move down', onClick: (e: MouseEvent) => { e.stopPropagation(); this.move(p, ins, 1); } }, '↓'),
          h('button', { class: 'btn-icon', title: 'Delete', onClick: (e: MouseEvent) => { e.stopPropagation(); this.app.deleteItems([ins]); this.app.previewProgram(); } }, '✕')));
      row.addEventListener('click', () => this.app.select(ins));
      row.addEventListener('dblclick', () => { if (target && robot instanceof Robot) { this.app.setActiveRobot(robot); this.app.moveRobotTo(target, d.kind === 'move' && d.moveType === 'MoveL'); } else if (step) this.app.seekProgram(step.t1); });
      row.addEventListener('contextmenu', (e) => { e.preventDefault(); contextMenu(e.clientX, e.clientY, [
        { label: 'Run from here', action: () => { this.app.runProgram(p); if (step) this.app.seekProgram(step.t0); } },
        { label: ins.enabled ? 'Disable' : 'Enable', action: () => { this.app.cmd(() => { ins.enabled = !ins.enabled; }); this.app.previewProgram(); } },
        { label: 'Insert MoveJ (teach) after', action: () => this.insertTeach(p, i + 1, 'MoveJ') },
        { label: 'Insert MoveL (teach) after', action: () => this.insertTeach(p, i + 1, 'MoveL') },
        { separator: true },
        { label: 'Delete', action: () => { this.app.deleteItems([ins]); this.app.previewProgram(); } },
      ]); });
      row.addEventListener('dragstart', (e) => e.dataTransfer?.setData('text/plain', ins.id));
      row.addEventListener('dragover', (e) => { e.preventDefault(); row.classList.add('drop'); });
      row.addEventListener('dragleave', () => row.classList.remove('drop'));
      row.addEventListener('drop', (e) => { e.preventDefault(); row.classList.remove('drop'); const id = e.dataTransfer?.getData('text/plain'); const src = id ? (this.app.station.findById(id) as Instruction | null) : null; if (src && src.parent === p && src !== ins) { this.app.cmd(() => p.addChild(src, p.children.indexOf(ins))); this.app.previewProgram(); } });
      list.appendChild(row);
    });
    this.el.appendChild(list);
    if (res) this.el.appendChild(h('div', { class: 'pe-foot' }, `Cycle time ${res.duration.toFixed(2)} s · TCP distance ${(res.distance / 1000).toFixed(2)} m · ${res.ok ? 'valid' : `${res.problems.filter((x) => x.severity === 'error').length} error(s)`}`));
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
