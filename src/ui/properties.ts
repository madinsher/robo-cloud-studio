import { App } from '../app';
import { Item, ItemType, Frame, Target, Tool, SceneObject, Camera as CameraItem } from '../core/items/item';
import { Robot } from '../core/items/robot';
import { Program, Instruction } from '../core/items/program';
import { MobileRobot, MapItem, ZoneItem } from '../mobile/items';
import { FleetItem } from '../fleet/fleet';
import { FieldItem, MissionItem, CropRow } from '../agri/items';
import { Component } from '../vc/component';
import { h, clear, formField, fmt, icon } from './dom';
import { poseToXyzrpw, xyzrpwToPose, poseToKuka, kukaToPose, poseToQuat, poseToUr, getPos, multiply, invert, Mat4 } from '../core/math/pose';
import { robotParametersDialog, runMissionPlan, exportDialog } from './dialogs';
import { navStackSection } from './navstack_ui';
import { visionSection } from './vision_ui';
import { controlSection } from './control_ui';
import { ControlModelItem } from '../ctl/model';
import { t } from './i18n';

type EulerMode = 'xyzrpw' | 'kuka' | 'ur' | 'abb';

/** Properties / inspector panel for the selected item. */
export class PropertiesPanel {
  el: HTMLElement;
  private euler: EulerMode = 'xyzrpw';
  private current: Item | null = null;
  private liveTimer: number | null = null;

  constructor(readonly app: App) {
    this.el = h('div', { class: 'panel props-panel' });
    const bind = () => {
      this.app.station.events.on('selection', () => this.render());
      this.app.station.events.on('changed', ({ item, what }) => { if (item === this.current && what !== 'pose' && what !== 'joints') this.scheduleRender(); else if (item === this.current) this.updateLive(); });
    };
    bind();
    app.events.on('stationChanged', () => { bind(); this.render(); });
    app.events.on('activeRobot', () => this.scheduleRender());
    this.render();
  }

  private timer: number | null = null;
  private scheduleRender() {
    if (this.timer) return;
    this.timer = window.setTimeout(() => { this.timer = null; this.render(); }, 30);
  }

  private liveFields: Array<() => void> = [];
  private updateLive() {
    if (this.liveTimer) return;
    this.liveTimer = window.setTimeout(() => { this.liveTimer = null; for (const f of this.liveFields) f(); }, 40);
  }

  render(): void {
    clear(this.el);
    this.liveFields = [];
    const item = this.app.station.selection[0] ?? null;
    this.current = item;
    this.el.appendChild(h('div', { class: 'panel-head' }, h('span', null, icon('gear'), ' ' + t('Properties'))));
    if (!item) { this.el.appendChild(h('div', { class: 'panel-body hint' }, t('Select an item in the tree or the 3D view.'))); return; }
    const body = h('div', { class: 'panel-body' });
    body.appendChild(h('div', { class: 'prop-title' }, h('b', null, item.name), h('small', null, ` ${item.typeName()}`)));
    if (item.type !== ItemType.STATION && item.type !== ItemType.INSTRUCTION && item.type !== ItemType.PROGRAM && item.type !== ItemType.CROP_ROW && item.type !== ItemType.CONTROL_MODEL) body.appendChild(this.poseEditor(item));
    if (item instanceof Robot) body.appendChild(this.robotEditor(item));
    else if (item instanceof Target) body.appendChild(this.targetEditor(item));
    else if (item instanceof Tool) body.appendChild(this.toolEditor(item));
    else if (item instanceof Program) body.appendChild(this.programEditor(item));
    else if (item instanceof Instruction) body.appendChild(this.instructionEditor(item));
    else if (item instanceof MobileRobot) body.appendChild(this.mobileEditor(item));
    else if (item instanceof FieldItem) body.appendChild(this.fieldEditor(item));
    else if (item instanceof CropRow) body.appendChild(this.rowEditor(item));
    else if (item instanceof MissionItem) body.appendChild(this.missionEditor(item));
    else if (item instanceof FleetItem) body.appendChild(this.fleetEditor(item));
    else if (item instanceof Component) body.appendChild(this.componentEditor(item));
    else if (item instanceof MapItem) body.appendChild(this.section('Map', h('div', { class: 'kv' }, kv('Size', `${item.width} × ${item.height} cells`), kv('Resolution', `${item.resolution} mm`), kv('Inflation', `${item.inflation} mm`))));
    else if (item instanceof ZoneItem) body.appendChild(this.section('Zone', formField({ key: 'kind', label: 'Kind', type: 'select', value: item.kind, options: ['work', 'nogo', 'charging', 'loading', 'unloading', 'parking', 'speed_limit', 'headland'].map((k) => ({ value: k, label: k })) }, (v) => this.app.cmd(() => { item.kind = v; item.notify('kind'); })).el));
    else if (item instanceof ControlModelItem) body.appendChild(controlSection(this.app, item));
    else if (item instanceof SceneObject) body.appendChild(this.objectEditor(item));
    else if (item instanceof CameraItem) body.appendChild(this.cameraEditor(item));
    // generic params
    const keys = Object.keys(item.params);
    if (keys.length) body.appendChild(this.section('Parameters', h('div', { class: 'kv' }, ...keys.map((k) => kv(k, typeof item.params[k] === 'object' ? JSON.stringify(item.params[k]).slice(0, 60) : String(item.params[k]))))));
    this.el.appendChild(body);
  }

  private section(title: string, ...content: (Node | null)[]): HTMLElement {
    return h('details', { class: 'section', open: true }, h('summary', null, t(title)), ...content);
  }

  // -- Pose ----------------------------------------------------------------
  private poseEditor(item: Item): HTMLElement {
    const wrap = h('div');
    const labelsOf = (m: EulerMode) => (m === 'kuka' ? ['X', 'Y', 'Z', 'A', 'B', 'C'] : m === 'ur' ? ['X', 'Y', 'Z', 'Rx', 'Ry', 'Rz'] : m === 'abb' ? ['X', 'Y', 'Z', 'q1', 'q2', 'q3', 'q4'] : ['X', 'Y', 'Z', 'Rx', 'Ry', 'Rz']);
    const values = (m: Mat4) => (this.euler === 'kuka' ? poseToKuka(m) : this.euler === 'ur' ? poseToUr(m) : this.euler === 'abb' ? [m[12], m[13], m[14], ...poseToQuat(m)] : poseToXyzrpw(m));
    const inputs: HTMLInputElement[] = [];
    const grid = h('div', { class: 'pose-grid' });
    const rebuild = () => {
      clear(grid);
      inputs.length = 0;
      const v = values(item.pose());
      labelsOf(this.euler).forEach((l, i) => {
        const inp = h('input', { type: 'number', step: 'any', value: fmt(v[i], 3) }) as HTMLInputElement;
        inp.addEventListener('change', () => {
          const vals = inputs.map((x) => parseFloat(x.value) || 0);
          const m = this.euler === 'kuka' ? kukaToPose(vals[0], vals[1], vals[2], vals[3], vals[4], vals[5]) : this.euler === 'ur' ? (() => { const { urToPose } = require_ur(); return urToPose(vals[0], vals[1], vals[2], vals[3], vals[4], vals[5]); })() : this.euler === 'abb' ? (() => { const { quatToPose } = require_ur(); return quatToPose(vals[0], vals[1], vals[2], [vals[3], vals[4], vals[5], vals[6]]); })() : xyzrpwToPose(vals[0], vals[1], vals[2], vals[3], vals[4], vals[5]);
          this.app.cmd(() => item.setPose(m));
        });
        inputs.push(inp);
        grid.appendChild(h('label', { class: 'pose-cell' }, h('span', null, l), inp));
      });
    };
    rebuild();
    this.liveFields.push(() => { const v = values(item.pose()); inputs.forEach((inp, i) => { if (document.activeElement !== inp) inp.value = fmt(v[i], 3); }); });
    const modeSel = formField({ key: 'euler', label: 'Orientation', type: 'select', value: this.euler, options: [{ value: 'xyzrpw', label: 'XYZ + Rx Ry Rz (RoboDK / Stäubli)' }, { value: 'kuka', label: 'XYZ + A B C (KUKA / Fanuc WPR)' }, { value: 'ur', label: 'XYZ + rotation vector (UR)' }, { value: 'abb', label: 'XYZ + quaternion (ABB)' }] }, (v) => { this.euler = v; rebuild(); });
    const parentName = item.parent && item.parent.type !== ItemType.STATION ? item.parent.name : 'Station';
    const absPos = getPos(item.poseAbs());
    const absLine = h('div', { class: 'hint' }, `abs: ${fmt(absPos[0], 1)}, ${fmt(absPos[1], 1)}, ${fmt(absPos[2], 1)} mm`);
    this.liveFields.push(() => { const p = getPos(item.poseAbs()); absLine.textContent = `abs: ${fmt(p[0], 1)}, ${fmt(p[1], 1)}, ${fmt(p[2], 1)} mm`; });
    wrap.appendChild(this.section(`Pose relative to ${parentName} (mm / deg)`, grid, modeSel.el, absLine));
    return wrap;
  }

  // -- Robot ---------------------------------------------------------------
  private robotEditor(r: Robot): HTMLElement {
    const wrap = h('div');
    const { lower, upper } = r.jointLimits();
    const q = r.joints();
    const sliders: HTMLInputElement[] = [];
    const nums: HTMLInputElement[] = [];
    const jointRows = r.jointNames().map((name, i) => {
      const slider = h('input', { type: 'range', min: lower[i], max: upper[i], step: 0.1, value: q[i] }) as HTMLInputElement;
      const num = h('input', { type: 'number', step: 0.1, value: fmt(q[i], 2), class: 'joint-num' }) as HTMLInputElement;
      const apply = (v: number) => { const qq = r.joints(); qq[i] = Math.max(lower[i], Math.min(upper[i], v)); r.setJoints(qq); };
      slider.addEventListener('input', () => { apply(parseFloat(slider.value)); num.value = fmt(parseFloat(slider.value), 2); });
      slider.addEventListener('change', () => this.app.snapshot());
      num.addEventListener('change', () => { apply(parseFloat(num.value)); slider.value = num.value; this.app.snapshot(); });
      sliders.push(slider); nums.push(num);
      return h('div', { class: 'joint-row' }, h('span', { class: 'joint-name' }, name), slider, num);
    });
    this.liveFields.push(() => { const qq = r.joints(); qq.forEach((v, i) => { if (sliders[i]) sliders[i].value = String(v); if (nums[i] && document.activeElement !== nums[i]) nums[i].value = fmt(v, 2); }); });
    const tcp = h('div', { class: 'kv' });
    const updTcp = () => { clear(tcp); const p = poseToXyzrpw(r.poseTCP()); tcp.append(kv('TCP in frame', `${fmt(p[0], 1)}, ${fmt(p[1], 1)}, ${fmt(p[2], 1)} | ${fmt(p[3], 1)}, ${fmt(p[4], 1)}, ${fmt(p[5], 1)}`), kv('Frame', r.activeFrame()?.name ?? 'Robot base'), kv('Tool', r.activeTool()?.name ?? 'Flange'), kv('Reach', `${fmt(r.reach, 0)} mm`), kv('Post', r.postProcessor)); };
    updTcp();
    this.liveFields.push(updTcp);
    const cartJog = this.cartesianJog(r);
    wrap.appendChild(this.section('Joints (deg / mm)', ...jointRows, h('div', { class: 'btn-row' }, h('button', { class: 'btn', onClick: () => this.app.cmd(() => r.setJoints(r.jointsHome())) }, 'Home'), h('button', { class: 'btn', onClick: () => this.app.addTarget() }, 'Teach target'), h('button', { class: 'btn', onClick: () => this.app.teach('MoveJ') }, '+ MoveJ'), h('button', { class: 'btn', onClick: () => this.app.teach('MoveL') }, '+ MoveL'))));
    wrap.appendChild(this.section('Cartesian jog (tool frame)', cartJog));
    wrap.appendChild(this.section('Status', tcp, h('div', { class: 'btn-row' }, h('button', { class: 'btn', onClick: () => robotParametersDialog(this.app, r) }, 'Parameters…'), h('button', { class: 'btn', onClick: () => this.app.setActiveRobot(r) }, 'Set active'))));
    // frame & tool selectors
    const frames = this.app.station.itemsOfType<Frame>(ItemType.FRAME);
    const frameSel = formField({ key: 'f', label: 'Active reference frame', type: 'select', value: r.activeFrameId ?? '', options: [{ value: '', label: '(robot base / parent)' }, ...frames.map((f) => ({ value: f.id, label: f.name }))] }, (v) => this.app.cmd(() => r.setFrame(v ? this.app.station.findById(v) : null)));
    const toolSel = formField({ key: 't', label: 'Active tool', type: 'select', value: r.activeToolId ?? r.activeTool()?.id ?? '', options: [{ value: '', label: '(flange)' }, ...r.tools().map((t) => ({ value: t.id, label: t.name }))] }, (v) => this.app.cmd(() => r.setTool(v ? (this.app.station.findById(v) as Tool) : null)));
    wrap.appendChild(this.section('Frames', frameSel.el, toolSel.el));
    return wrap;
  }

  private cartesianJog(r: Robot): HTMLElement {
    const stepIn = h('input', { type: 'number', value: 10, step: 1, class: 'joint-num' }) as HTMLInputElement;
    const rotIn = h('input', { type: 'number', value: 5, step: 1, class: 'joint-num' }) as HTMLInputElement;
    const jog = (axis: number, sign: number, rot: boolean) => {
      const step = (rot ? parseFloat(rotIn.value) : parseFloat(stepIn.value)) * sign;
      const tcp = r.solveFK();
      const d = rot ? [0, 0, 0, 0, 0, 0] : [0, 0, 0, 0, 0, 0];
      d[axis + (rot ? 3 : 0)] = step;
      const delta = xyzrpwToPose(d[0], d[1], d[2], d[3], d[4], d[5]);
      const target = multiply(tcp, delta); // tool-frame jog
      const res = r.solveIK(target, { seed: r.joints(), restarts: 1 });
      if (res.ok) { r.setJoints(res.joints); this.app.snapshot(); } else this.app.log('Jog target unreachable', 'warn');
    };
    const btn = (l: string, a: number, s: number, rot: boolean) => h('button', { class: 'btn jog', onClick: () => jog(a, s, rot) }, l);
    return h('div', null,
      h('div', { class: 'jog-grid' }, btn('-X', 0, -1, false), btn('+X', 0, 1, false), btn('-Y', 1, -1, false), btn('+Y', 1, 1, false), btn('-Z', 2, -1, false), btn('+Z', 2, 1, false)),
      h('div', { class: 'jog-grid' }, btn('-Rx', 0, -1, true), btn('+Rx', 0, 1, true), btn('-Ry', 1, -1, true), btn('+Ry', 1, 1, true), btn('-Rz', 2, -1, true), btn('+Rz', 2, 1, true)),
      h('div', { class: 'btn-row' }, h('label', null, 'step mm ', stepIn), h('label', null, 'deg ', rotIn)));
  }

  private targetEditor(t: Target): HTMLElement {
    const robot = this.app.activeRobot;
    const q = robot ? robot.jointsForTarget(t) : null;
    return this.section('Target',
      formField({ key: 'j', label: 'Joint target (use recorded joints)', type: 'checkbox', value: t.isJointTarget }, (v) => this.app.cmd(() => (v ? t.setAsJointTarget() : t.setAsCartesianTarget()))).el,
      h('div', { class: 'kv' }, kv('Recorded joints', t.joints ? t.joints.map((x) => fmt(x, 1)).join(', ') : '—'), kv('Reachable by active robot', robot ? (q ? 'yes' : 'NO') : 'no active robot')),
      h('div', { class: 'btn-row' }, h('button', { class: 'btn', onClick: () => this.app.moveRobotTo(t) }, 'MoveJ here'), h('button', { class: 'btn', onClick: () => this.app.moveRobotTo(t, true) }, 'MoveL here'), h('button', { class: 'btn', onClick: () => this.app.cmd(() => { if (!robot) return; t.setPoseAbs(robot.poseTCPAbs()); t.setJoints(robot.joints()); }) }, 'Teach')));
  }

  private toolEditor(t: Tool): HTMLElement {
    return this.section('Tool', h('div', { class: 'kv' }, kv('Kind', t.toolKind), kv('Gripper', t.closed ? 'closed' : 'open'), kv('Attached', String(t.attached.length))),
      formField({ key: 'kind', label: 'Tool kind', type: 'select', value: t.toolKind, options: ['generic', 'gripper', 'vacuum', 'sprayer', 'cutter', 'camera', 'welding', 'shears'].map((k) => ({ value: k, label: k })) }, (v) => this.app.cmd(() => { t.toolKind = v; })).el,
      t.parent instanceof Robot ? h('button', { class: 'btn', onClick: () => this.app.cmd(() => (t.parent as Robot).setTool(t)) }, 'Set as active tool') : null);
  }

  private programEditor(p: Program): HTMLElement {
    const robots = [...this.app.station.itemsOfType<Robot>(ItemType.ROBOT), ...this.app.station.itemsOfType<MobileRobot>(ItemType.MOBILE_ROBOT)];
    const res = p.lastResult;
    return this.section('Program',
      formField({ key: 'r', label: 'Robot', type: 'select', value: p.robotId ?? '', options: [{ value: '', label: '(none)' }, ...robots.map((r) => ({ value: r.id, label: r.name }))] }, (v) => this.app.cmd(() => p.setRobot(v ? this.app.station.findById(v) : null))).el,
      h('div', { class: 'kv' }, kv('Instructions', String(p.instructions().length)), kv('Cycle time', res ? `${res.duration.toFixed(2)} s` : '—'), kv('Distance', res ? `${(res.distance / 1000).toFixed(2)} m` : '—'), kv('Status', res ? (res.ok ? 'OK' : `${res.problems.length} problem(s)`) : 'not validated')),
      res && res.problems.length ? h('ul', { class: 'problems' }, ...res.problems.map((pr) => h('li', { class: pr.severity }, pr.message))) : null,
      h('div', { class: 'btn-row' }, h('button', { class: 'btn primary', onClick: () => this.app.runProgram(p) }, '▶ Run'), h('button', { class: 'btn', onClick: () => { this.app.setActiveProgram(p); this.app.previewProgram(); this.render(); } }, 'Validate'), h('button', { class: 'btn', onClick: () => exportDialog(this.app, p) }, 'Export…')));
  }

  private instructionEditor(ins: Instruction): HTMLElement {
    const d = ins.data;
    const fields: HTMLElement[] = [];
    const upd = () => { this.app.cmd(() => { ins.name = describe(ins); ins.notify('data'); }); this.app.previewProgram(); };
    const targets = this.app.station.itemsOfType<Target>(ItemType.TARGET);
    if (d.kind === 'move') {
      fields.push(formField({ key: 'mt', label: 'Move type', type: 'select', value: d.moveType, options: ['MoveJ', 'MoveL', 'MoveC'].map((m) => ({ value: m, label: m })) }, (v) => { d.moveType = v; upd(); }).el);
      fields.push(formField({ key: 't', label: 'Target', type: 'select', value: d.targetId ?? '', options: [{ value: '', label: '(inline)' }, ...targets.map((t) => ({ value: t.id, label: t.name }))] }, (v) => { d.targetId = v || null; upd(); }).el);
      if (d.moveType === 'MoveC') fields.push(formField({ key: 'v', label: 'Via target', type: 'select', value: d.viaTargetId ?? '', options: [{ value: '', label: '(none)' }, ...targets.map((t) => ({ value: t.id, label: t.name }))] }, (v) => { d.viaTargetId = v || null; upd(); }).el);
      fields.push(formField({ key: 's', label: 'Speed override (mm/s, empty = program)', type: 'number', value: d.speed ?? '' }, (v) => { d.speed = isNaN(v) ? undefined : v; upd(); }).el);
    } else if (d.kind === 'speed') {
      fields.push(formField({ key: 'sl', label: 'Linear speed (mm/s)', type: 'number', value: d.speedLinear ?? '' }, (v) => { d.speedLinear = isNaN(v) ? undefined : v; upd(); }).el);
      fields.push(formField({ key: 'sj', label: 'Joint speed (deg/s)', type: 'number', value: d.speedJoints ?? '' }, (v) => { d.speedJoints = isNaN(v) ? undefined : v; upd(); }).el);
    } else if (d.kind === 'pause') fields.push(formField({ key: 'p', label: 'Pause (ms, -1 = wait user)', type: 'number', value: d.timeMs }, (v) => { d.timeMs = v; upd(); }).el);
    else if (d.kind === 'rounding') fields.push(formField({ key: 'r', label: 'Rounding radius (mm, -1 = fine)', type: 'number', value: d.radius }, (v) => { d.radius = v; upd(); }).el);
    else if (d.kind === 'io') { fields.push(formField({ key: 'io', label: 'IO name', type: 'text', value: d.io }, (v) => { d.io = v; upd(); }).el); fields.push(formField({ key: 'v', label: 'Value', type: 'checkbox', value: !!d.value }, (v) => { d.value = v; upd(); }).el); fields.push(formField({ key: 'w', label: 'Wait for input (instead of set output)', type: 'checkbox', value: d.wait }, (v) => { d.wait = v; upd(); }).el); }
    else if (d.kind === 'code') { fields.push(formField({ key: 'c', label: 'Code / program name', type: 'textarea', value: d.code }, (v) => { d.code = v; upd(); }).el); fields.push(formField({ key: 'f', label: 'Call as function', type: 'checkbox', value: d.asFunctionCall }, (v) => { d.asFunctionCall = v; upd(); }).el); }
    else if (d.kind === 'print') { fields.push(formField({ key: 'm', label: 'Message', type: 'text', value: d.message }, (v) => { d.message = v; upd(); }).el); fields.push(formField({ key: 'c', label: 'Comment only', type: 'checkbox', value: d.isComment }, (v) => { d.isComment = v; upd(); }).el); }
    else if (d.kind === 'event') { const objs = this.app.station.itemsOfType<SceneObject>(ItemType.OBJECT); fields.push(formField({ key: 'a', label: 'Action', type: 'select', value: d.action, options: ['attach', 'detach', 'gripper_close', 'gripper_open', 'show', 'hide'].map((a) => ({ value: a, label: a })) }, (v) => { d.action = v; upd(); }).el); fields.push(formField({ key: 'o', label: 'Object', type: 'select', value: d.objectId ?? '', options: [{ value: '', label: '(nearest)' }, ...objs.map((o) => ({ value: o.id, label: o.name }))] }, (v) => { d.objectId = v || null; upd(); }).el); }
    else if (d.kind === 'mobile_move') { fields.push(formField({ key: 'x', label: 'X (mm)', type: 'number', value: d.x }, (v) => { d.x = v; upd(); }).el); fields.push(formField({ key: 'y', label: 'Y (mm)', type: 'number', value: d.y }, (v) => { d.y = v; upd(); }).el); fields.push(formField({ key: 's', label: 'Speed (mm/s)', type: 'number', value: d.speed ?? '' }, (v) => { d.speed = isNaN(v) ? undefined : v; upd(); }).el); }
    else if (d.kind === 'signal') { fields.push(formField({ key: 's', label: 'Signal', type: 'text', value: d.signal }, (v) => { d.signal = v; upd(); }).el); fields.push(formField({ key: 'v', label: 'Value', type: 'text', value: String(d.value) }, (v) => { d.value = v === 'true' ? true : v === 'false' ? false : isNaN(+v) ? v : +v; upd(); }).el); }
    else fields.push(h('pre', { class: 'code-preview small' }, JSON.stringify(d, null, 1)));
    fields.push(formField({ key: 'en', label: 'Enabled', type: 'checkbox', value: ins.enabled }, (v) => { ins.enabled = v; upd(); }).el);
    return this.section(`Instruction: ${ins.name}`, ...fields);
  }

  private mobileEditor(m: MobileRobot): HTMLElement {
    const st = h('div', { class: 'kv' });
    const upd = () => { clear(st); st.append(kv('Status', m.state.status), kv('Position', `${fmt(m.state.x / 1000, 2)}, ${fmt(m.state.y / 1000, 2)} m @ ${fmt(m.state.theta, 1)}°`), kv('Speed', `${fmt(m.state.v / 1000, 2)} m/s`), kv('Battery', `${fmt(m.batteryLevel() * 100, 0)} % (${fmt(m.battery.levelWh, 0)} Wh)`), kv('Odometer', `${fmt(m.state.odometer / 1000, 1)} m`), kv('Task', m.state.taskId ?? '—')); };
    upd();
    this.liveFields.push(upd);
    const k = m.kin;
    const num = (label: string, get: () => number, set: (v: number) => void, step = 1) => formField({ key: label, label, type: 'number', value: get(), step }, (v) => this.app.cmd(() => { set(v); m.notify('kin'); })).el;
    return h('div', null,
      this.section('State', st),
      this.section('Kinematics', formField({ key: 'd', label: 'Drive', type: 'select', value: k.drive, options: ['differential', 'ackermann', 'omni', 'tracked'].map((x) => ({ value: x, label: x })) }, (v) => this.app.cmd(() => { k.drive = v; })).el,
        num('Max speed (mm/s)', () => k.maxSpeed, (v) => (k.maxSpeed = v)), num('Max accel (mm/s²)', () => k.maxAccel, (v) => (k.maxAccel = v)), num('Max yaw rate (deg/s)', () => k.maxYawRate, (v) => (k.maxYawRate = v)), num('Wheel base (mm)', () => k.wheelBase, (v) => (k.wheelBase = v)), num('Min turn radius (mm)', () => k.minTurnRadius, (v) => (k.minTurnRadius = v)),
        num('Footprint length (mm)', () => k.footprint[0], (v) => (k.footprint[0] = v)), num('Footprint width (mm)', () => k.footprint[1], (v) => (k.footprint[1] = v))),
      this.section('Battery & capabilities', num('Capacity (Wh)', () => m.battery.capacityWh, (v) => (m.battery.capacityWh = v)), num('Low threshold (0-1)', () => m.battery.lowThreshold, (v) => (m.battery.lowThreshold = v), 0.05),
        formField({ key: 'cap', label: 'Capabilities (comma separated)', type: 'text', value: m.capabilities.join(', ') }, (v) => this.app.cmd(() => { m.capabilities = v.split(',').map((s: string) => s.trim()).filter(Boolean); })).el,
        formField({ key: 'ns', label: 'ROS 2 namespace', type: 'text', value: m.rosNamespace }, (v) => this.app.cmd(() => { m.rosNamespace = v; })).el),
      navStackSection(this.app, m, this.liveFields));
  }

  private fieldEditor(f: FieldItem): HTMLElement {
    const c = f.crop;
    const rows = f.rows();
    const plants = rows.reduce((s, r) => s + r.plants.length, 0);
    const ripe = rows.reduce((s, r) => s + r.ripeFruitCount(), 0);
    return this.section('Field', h('div', { class: 'kv' }, kv('Crop', `${c.crop} (${c.training})`), kv('Rows', String(rows.length)), kv('Plants', String(plants)), kv('Ripe fruit', String(ripe)), kv('Row spacing', `${c.rowSpacing} mm`), kv('Plant spacing', `${c.plantSpacing} mm`), kv('Area', `${(polygonAreaMm2(f.polygon) / 1e6 / 10000).toFixed(2)} ha`), kv('Geo origin', f.geoOrigin ? `${f.geoOrigin.lat.toFixed(6)}, ${f.geoOrigin.lon.toFixed(6)}` : '—')));
  }

  private rowEditor(r: CropRow): HTMLElement {
    return this.section('Crop row', h('div', { class: 'kv' }, kv('Length', `${(r.length() / 1000).toFixed(1)} m`), kv('Plants', String(r.plants.length)), kv('Ripe fruit', String(r.ripeFruitCount())), kv('Segment', r.segmentId)));
  }

  private missionEditor(m: MissionItem): HTMLElement {
    const fleet = m.fleetId ? (this.app.station.findById(m.fleetId) as FleetItem | null) : null;
    const tasks = fleet ? fleet.tasks.filter((t) => m.taskIds.includes(t.id)) : [];
    const prog = h('progress', { value: m.progress, max: 1, style: { width: '100%' } });
    this.liveFields.push(() => { prog.value = m.progress; });
    return this.section('Mission', h('div', { class: 'kv' }, kv('Type', m.missionType), kv('Status', m.status), kv('Rows', String(m.rowIds.length || 'all')), kv('Tasks', `${tasks.filter((t) => t.status === 'done').length} / ${tasks.length}`)), prog,
      h('div', { class: 'btn-row' }, h('button', { class: 'btn primary', onClick: () => runMissionPlan(this.app, m) }, 'Plan'), h('button', { class: 'btn', onClick: () => this.app.startWorld() }, '▶ Run world')));
  }

  private fleetEditor(f: FleetItem): HTMLElement {
    const fm = this.app.fleetManager(f);
    const k = fm.kpis();
    const list = h('div', { class: 'kv' });
    const upd = () => { const k = fm.kpis(); clear(list); list.append(kv('Robots', String(fm.robots().length)), kv('Tasks done / pending', `${k.tasksDone} / ${k.tasksPending}`), kv('Throughput', `${k.throughputPerHour.toFixed(1)} tasks/h`), kv('Utilisation', `${(k.utilization * 100).toFixed(0)} %`), kv('Distance', `${(k.distanceTravelled / 1000).toFixed(0)} m`), kv('Energy', `${k.energyUsedWh.toFixed(0)} Wh`), kv('Mean wait', `${k.meanWaitTime.toFixed(0)} s`)); };
    upd();
    this.liveFields.push(upd);
    void k;
    return this.section('Fleet', formField({ key: 'a', label: 'Allocation', type: 'select', value: f.allocation, options: [{ value: 'auction', label: 'Cost-based auction' }, { value: 'nearest', label: 'Nearest' }, { value: 'round_robin', label: 'Round robin' }] }, (v) => this.app.cmd(() => { f.allocation = v; })).el,
      formField({ key: 'c', label: 'Robots per row segment', type: 'number', value: f.segmentCapacity, min: 1 }, (v) => this.app.cmd(() => { f.segmentCapacity = v; })).el, list,
      h('div', { class: 'btn-row' }, h('button', { class: 'btn', onClick: () => this.app.cmd(() => { for (const m of this.app.station.itemsOfType<MobileRobot>(ItemType.MOBILE_ROBOT)) fm.addRobot(m); this.render(); }) }, 'Add all mobile robots')));
  }

  private componentEditor(c: Component): HTMLElement {
    const b = c.behaviour as any;
    const comps = this.app.station.itemsOfType<Component>(ItemType.COMPONENT).filter((x) => x !== c);
    const fields: HTMLElement[] = [h('div', { class: 'kv' }, kv('Behaviour', b.type), kv('Products held', String(c.products.length)), kv('Entered / exited', `${c.stats.entered} / ${b.type === 'sink' ? b.count : c.stats.exited}`))];
    if ('next' in b) fields.push(formField({ key: 'n', label: 'Output to', type: 'select', value: b.next ?? '', options: [{ value: '', label: '(none)' }, ...comps.map((x) => ({ value: x.id, label: x.name }))] }, (v) => this.app.cmd(() => { b.next = v || null; })).el);
    const num = (label: string, key: string, step = 1) => fields.push(formField({ key, label, type: 'number', value: b[key], step }, (v) => this.app.cmd(() => { b[key] = v; c.notify('behaviour'); })).el);
    if (b.type === 'conveyor') { num('Speed (mm/s)', 'speed'); num('Spacing (mm)', 'spacing'); }
    if (b.type === 'feeder') { num('Interval (s)', 'interval', 0.1); num('Limit (-1 = infinite)', 'limit'); }
    if (b.type === 'process') { num('Cycle time (s)', 'cycleTime', 0.1); num('Capacity', 'capacity'); num('MTBF (s, 0 = none)', 'mtbf'); num('MTTR (s)', 'mttr'); }
    if (b.type === 'buffer') num('Capacity', 'capacity');
    fields.push(formField({ key: 'en', label: 'Enabled', type: 'checkbox', value: b.enabled }, (v) => this.app.cmd(() => { b.enabled = v; })).el);
    return this.section('Component', ...fields);
  }

  private cameraEditor(c: CameraItem): HTMLElement {
    const num = (label: string, get: () => number, set: (v: number) => void, step = 1) => formField({ key: label, label, type: 'number', value: get(), step }, (v) => this.app.cmd(() => { set(v); c.notify('kind'); })).el;
    return h('div', null,
      this.section('Camera / sensor',
        formField({ key: 'kind', label: 'Kind', type: 'select', value: c.kind, options: [{ value: 'rgb', label: 'RGB (mono / stereo colour)' }, { value: 'depth', label: 'Depth (stereo / RGB-D / ToF)' }, { value: 'lidar2d', label: '2D LiDAR' }, { value: 'lidar3d', label: '3D LiDAR' }] }, (v) => this.app.cmd(() => { c.kind = v; c.notify('kind'); })).el,
        num('Field of view (deg)', () => c.fov, (v) => (c.fov = v)), num('Width (px)', () => c.width, (v) => (c.width = v)), num('Height (px)', () => c.height, (v) => (c.height = v)),
        num('Near (mm)', () => c.near, (v) => (c.near = v)), num('Far (mm)', () => c.far, (v) => (c.far = v), 100)),
      visionSection(this.app, c, this.liveFields));
  }

  private objectEditor(o: SceneObject): HTMLElement {
    return this.section('Object', h('div', { class: 'kv' }, kv('Geometry', o.geometry.map((g) => g.mesh ?? g.primitive?.kind).join(', ') || '—'), kv('Curves', String(o.curves.length)), kv('Mass', `${o.mass} kg`)),
      formField({ key: 'color', label: 'Colour', type: 'color', value: o.color ?? '#9aa3ad' }, (v) => this.app.cmd(() => { o.color = v; for (const g of o.geometry) g.color = v; o.notify('geometry'); })).el);
  }
}

function kv(k: string, v: string): HTMLElement {
  return h('div', { class: 'kv-row' }, h('span', { class: 'k' }, k), h('span', { class: 'v' }, v));
}
function describe(ins: Instruction): string {
  const { describeInstruction } = require_program();
  return describeInstruction(ins.data);
}
import * as programModule from '../core/items/program';
import * as poseModule from '../core/math/pose';
function require_program() { return programModule; }
function require_ur() { return poseModule; }
function polygonAreaMm2(poly: number[][]): number {
  let a = 0;
  for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) a += poly[j][0] * poly[i][1] - poly[i][0] * poly[j][1];
  return Math.abs(a) / 2;
}
export { invert };
