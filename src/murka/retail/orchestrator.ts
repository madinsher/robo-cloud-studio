/**
 * The retail orchestrators as studio control models: the yasmin state machines are an `fsm` DSL document
 * (missionFsmDsl), executed by the studio's FsmExecutor (src/ctl/fsm.ts); every state is a skill of the action library
 * bound here to the station simulation (navigation, pick / place, perception, head and arms), with the same blackboard
 * keys, retries, error continuation and cancel semantics.
 *  - flavor 'murom': murom_fsm/orchestrator.py + recovery.py of robot-retailer-murka(-v2) (_build_mission_sm /
 *    _build_depal_sm / _build_order_sm and the single tasks; skills `murom.<state>`);
 *  - flavor 'murka': murka_fsm/orchestrator.py of the MURKA_Simulation copy (only /murka/task/depalletize and
 *    /murka/task/collect_order: no perception, VERIFY_ITEM, held-object recovery, scan, dock or mission; skills
 *    `murka.<state>`; SKU instances chosen from the registry by name).
 */
import { FsmExecutor, parseFsm, FsmDoc } from '../../ctl/fsm';
import type { ExecutorWorld } from '../../ctl/exec';
import type { WorldBindings } from '../../ctl/runtime';
import type { BTNode, TickContext } from '../../ctl/bt';
import type { MuromSim } from './sim';
import type { Navigator, NavOutcome, P3 } from './nav';
import type { ManipServer, PickResult, PlaceResult } from './manip';
import type { RetailLayout } from './layout';
import { slotsByPrefix } from './layout';
import { Task, TaskRunner } from './tasks';
import { recognize, renderHeadFrame, scanTargets, Prototype, SlotView, Frame } from './perception';
import type { AssetStore } from '../../scene/assets';

export const BASKET_CAPACITY = 2;
export const MISSION_DEFAULTS = { scan: ['shelf_A1', 'shelf_A2', 'shelf_A3'], depal: ['pallet_2', 'box_blue', 'bulk_B3', 1] as [string, string, string, number], order: ['tea_box', 'juice_pack'] };
/** Task arguments of the MURKA_Simulation copy when none are given: its README quick start and scripts/e2e_test.py. */
export const COPY_DEFAULTS = { depal: ['pallet_1', 'box_red', 'bulk_B1', 1] as [string, string, string, number], order: ['tea_box', 'coffee_can'] };
/** Orchestrator family: murom_fsm (robot-retailer-murka v1 / v2) or murka_fsm (the MURKA_Simulation copy). */
export type FsmFlavor = 'murom' | 'murka';
const COPY_TASKS_MSG = (name: string) => `задачи '${name}' нет в оркестраторе MURKA_Simulation (murka_fsm): есть depalletize, order`;
export const PHASES: Record<string, [number, number]> = { SCAN: [0, 15], DEPAL: [15, 35], ORDER: [50, 40], DOCK: [90, 10] };
export const HEAD_AXIS_X = -0.076, CAMERA_Z = 1.575, HEAD_PAN_LIMIT = 1.6;
export const HEAD_TILT_LIMITS: [number, number] = [-0.9, 0.6];
export const SCAN_TILT = 0.31, VERIFY_MIN_CONF = 0.4, OBS_YAW = 0.8, OBS_PAN = 0.75;
export const OBS_ARM = [0.35, 0, 0, 0], TRAVEL_ARM = [0.3, 0, 0, -2.2];
export const ARM_JOINTS = ['shoulder_pitch_joint', 'shoulder_roll_joint', 'shoulder_yaw_joint', 'elbow_joint'];
/** Sim time a recognition call waits for fresh camera / range-finder frames. */
export const SCAN_FRAME_WAIT = 0.2;

const norm = (a: number) => Math.atan2(Math.sin(a), Math.cos(a));
const clamp = (v: number, lo: number, hi: number) => Math.max(lo, Math.min(hi, v));

export type NavSpec = ['slot', string, string] | ['target', string] | ['pose', P3] | ['pose_coarse', P3];
export type MissionTask =
  | { type: 'mission'; scan: string[]; depal: [string, string, string, number] | []; order: string[] }
  | { type: 'depalletize'; pallet_id: string; sku: string; target_shelf: string; count: number }
  | { type: 'collect_order'; skus: string[] }
  | { type: 'scan'; shelves: string[] }
  | { type: 'goto' | 'dock' | 'undock' | 'home'; nav: NavSpec };

/**
 * _parse_mission: mission / task by name with its arguments (`full scan=shelf_A1 depal=pallet_1,box_red,bulk_B1,1
 * order=tea_box`). The MURKA_Simulation copy (flavor 'murka') has the services /murka/task/depalletize (pallet sku
 * target_shelf count) and /murka/task/collect_order (SKUs) only.
 */
export function parseMission(name0: string, args0: string[], L: RetailLayout, flavor: FsmFlavor = 'murom'): MissionTask {
  const name = name0.trim().toLowerCase(); const args = args0.filter((a) => a !== '');
  if (flavor === 'murka') {
    if (name === 'depalletize' || name === 'depal') {
      const p: Array<string | number> = [...COPY_DEFAULTS.depal];
      args.slice(0, 4).forEach((a, i) => { p[i] = i === 3 ? parseInt(a, 10) : a; });
      return { type: 'depalletize', pallet_id: String(p[0]), sku: String(p[1]), target_shelf: String(p[2]), count: Math.max(1, Number(p[3]) || 1) };
    }
    if (name === 'order' || name === 'collect_order') return { type: 'collect_order', skus: args.length ? args : [...COPY_DEFAULTS.order] };
    throw new Error(COPY_TASKS_MSG(name));
  }
  if (name === 'full' || name === 'mission' || name === '') {
    const p: { scan: string[]; depal: Array<string | number>; order: string[] } = { scan: [...MISSION_DEFAULTS.scan], depal: [...MISSION_DEFAULTS.depal], order: [...MISSION_DEFAULTS.order] };
    for (const a of args) {
      const i = a.indexOf('='); const key = i < 0 ? a : a.slice(0, i); const vals = (i < 0 ? '' : a.slice(i + 1)).split(',').filter(Boolean);
      if (!(key in p)) throw new Error(`full: неизвестный параметр '${key}' (scan, depal, order)`);
      if (key === 'depal' && vals.length) { const d: Array<string | number> = [...MISSION_DEFAULTS.depal]; vals.slice(0, 4).forEach((v, k) => { d[k] = k === 3 ? parseInt(v, 10) : v; }); (p as Record<string, unknown>)[key] = d; }
      else (p as Record<string, unknown>)[key] = vals;
    }
    return { type: 'mission', scan: p.scan, depal: p.depal.length ? (p.depal as [string, string, string, number]) : [], order: p.order };
  }
  if (name === 'depalletize' || name === 'depal') {
    const p: Array<string | number> = [...MISSION_DEFAULTS.depal];
    args.slice(0, 4).forEach((a, i) => { p[i] = i === 3 ? parseInt(a, 10) : a; });
    return { type: 'depalletize', pallet_id: String(p[0]), sku: String(p[1]), target_shelf: String(p[2]), count: Math.max(1, Number(p[3])) };
  }
  if (name === 'order' || name === 'collect_order') return { type: 'collect_order', skus: args.length ? args : [...MISSION_DEFAULTS.order] };
  if (name === 'scan') return { type: 'scan', shelves: args.length ? args : [...MISSION_DEFAULTS.scan] };
  if (name === 'dock' || name === 'home') return { type: name, nav: ['target', name] };
  if (name === 'undock') { if (!L.dock) throw new Error('в locations.yaml нет станции зарядки'); const s = L.dock.standoff; return { type: 'undock', nav: ['pose_coarse', [s.x, s.y, s.yaw]] }; }
  if (name === 'goto') {
    if (args.length === 1) {
      const tid = args[0];
      if (tid === 'dock' || tid === 'home' || L.targets[tid]) return { type: 'goto', nav: ['target', tid] };
      const s = L.slots[tid]; if (!s) throw new Error(`неизвестная цель '${tid}'`);
      return { type: 'goto', nav: ['slot', tid, s.arm ?? 'left'] };
    }
    if (args.length === 3) { const [x, y, yaw] = args.map(Number); return { type: 'goto', nav: ['pose', [x, y, yaw]] }; }
    throw new Error('goto: нужна цель или x y yaw');
  }
  throw new Error(`неизвестная миссия '${name}' (full, depalletize, order, scan, goto, dock, undock, home)`);
}

// ------------------------------------------------------------------- FSM document (yasmin structure)
const st = (name: string, skill: string, outcomes: string[], tr: Record<string, string>, extra = '', ind = '', initial = false): string[] => [
  `${ind}state ${name}${initial ? ' initial' : ''}`, `${ind}  do skill=murom.${skill}${extra ? ` ${extra}` : ''} outcomes=${outcomes.join(',')}`, ...Object.entries(tr).map(([o, t]) => `${ind}  ${o} -> ${t}`)];
/** The same lines with the skills of the MURKA_Simulation copy (murka.<state>). */
const copySkills = (lines: string[]) => lines.map((l) => l.replace('do skill=murom.', 'do skill=murka.'));
/** _build_depal_sm (identical in murom_fsm and in the copy's murka_fsm; the planning skill differs). */
function depalStates(goHome: boolean, ind: string): string[] {
  return [
    ...st('PLAN', 'plan_depal', ['found', 'none_left', 'error'], { found: 'NAV_PICK', none_left: goHome ? 'NAV_HOME' : 'done', error: 'error' }, '', ind),
    ...st('NAV_PICK', 'nav', ['ok', 'fail'], { ok: 'PICK', fail: 'error' }, 'key=pick_nav', ind),
    ...st('PICK', 'pick', ['ok', 'fail'], { ok: 'NAV_PLACE', fail: 'error' }, '', ind),
    ...st('NAV_PLACE', 'nav', ['ok', 'fail'], { ok: 'PLACE', fail: 'error' }, 'key=place_nav', ind),
    ...st('PLACE', 'place', ['ok', 'fail'], { ok: 'PLAN', fail: 'error' }, '', ind),
    ...(goHome ? st('NAV_HOME', 'nav', ['ok', 'fail'], { ok: 'done', fail: 'done' }, 'key=home_nav', ind) : []),
  ];
}
/** _build_order_sm of the MURKA_Simulation copy: PLAN_ITEM found -> NAV_SHELF (no NAV_VERIFY / VERIFY_ITEM). */
function copyOrderStates(goHome: boolean, ind: string): string[] {
  return copySkills([
    ...st('PLAN_ITEM', 'plan_item', ['found', 'to_packing', 'finished', 'error'], { found: 'NAV_SHELF', to_packing: 'UNLOAD_PLAN', finished: goHome ? 'NAV_HOME' : 'done', error: 'error' }, '', ind),
    ...st('NAV_SHELF', 'nav', ['ok', 'fail'], { ok: 'PICK_ITEM', fail: 'error' }, 'key=pick_nav', ind),
    ...st('PICK_ITEM', 'pick', ['ok', 'fail', 'skip'], { ok: 'PLACE_BASKET', skip: 'PLAN_ITEM', fail: 'error' }, 'skip_on_fail=true', ind),
    ...st('PLACE_BASKET', 'place', ['ok', 'fail'], { ok: 'PLAN_ITEM', fail: 'error' }, '', ind),
    ...st('UNLOAD_PLAN', 'unload_plan', ['item', 'empty', 'error'], { item: 'NAV_UNLOAD', empty: 'PLAN_ITEM', error: 'error' }, '', ind),
    ...st('NAV_UNLOAD', 'nav', ['ok', 'fail'], { ok: 'PICK_BASKET', fail: 'error' }, 'key=unload_nav', ind),
    ...st('PICK_BASKET', 'pick', ['ok', 'fail'], { ok: 'PLACE_PACK', fail: 'error' }, '', ind),
    ...st('PLACE_PACK', 'place', ['ok', 'fail'], { ok: 'UNLOAD_PLAN', fail: 'error' }, '', ind),
    ...(goHome ? st('NAV_HOME', 'nav', ['ok', 'fail'], { ok: 'done', fail: 'done' }, 'key=home_nav', ind) : []),
  ]);
}
function orderStates(goHome: boolean, ind: string): string[] {
  return [
    ...st('PLAN_ITEM', 'plan_item', ['found', 'to_packing', 'finished', 'error'], { found: 'NAV_VERIFY', to_packing: 'UNLOAD_PLAN', finished: goHome ? 'NAV_HOME' : 'done', error: 'error' }, '', ind),
    ...st('NAV_VERIFY', 'nav_verify', ['ok', 'fail'], { ok: 'VERIFY_ITEM', fail: 'error' }, 'key=verify_nav', ind),
    ...st('VERIFY_ITEM', 'verify', ['ok', 'mismatch', 'fail'], { ok: 'NAV_SHELF', mismatch: 'PLAN_ITEM', fail: 'error' }, '', ind),
    ...st('NAV_SHELF', 'nav', ['ok', 'fail'], { ok: 'PICK_ITEM', fail: 'error' }, 'key=pick_nav', ind),
    ...st('PICK_ITEM', 'pick', ['ok', 'fail', 'skip'], { ok: 'PLACE_BASKET', skip: 'PLAN_ITEM', fail: 'error' }, 'skip_on_fail=true', ind),
    ...st('PLACE_BASKET', 'place', ['ok', 'fail'], { ok: 'PLAN_ITEM', fail: 'error' }, '', ind),
    ...st('UNLOAD_PLAN', 'unload_plan', ['item', 'empty', 'error'], { item: 'NAV_UNLOAD', empty: 'PLAN_ITEM', error: 'error' }, '', ind),
    ...st('NAV_UNLOAD', 'nav', ['ok', 'fail'], { ok: 'PICK_BASKET', fail: 'error' }, 'key=unload_nav', ind),
    ...st('PICK_BASKET', 'pick', ['ok', 'fail'], { ok: 'PLACE_PACK', fail: 'error' }, '', ind),
    ...st('PLACE_PACK', 'place', ['ok', 'fail'], { ok: 'UNLOAD_PLAN', fail: 'error' }, '', ind),
    ...(goHome ? st('NAV_HOME', 'nav', ['ok', 'fail'], { ok: 'done', fail: 'done' }, 'key=home_nav', ind) : []),
  ];
}
/** The FSM DSL of a task: same states / outcomes / transitions as the yasmin machines of orchestrator.py (murom_fsm, or
 *  murka_fsm of the MURKA_Simulation copy for flavor 'murka', header `fsm MURKA <task>`). */
export function missionFsmDsl(task: MissionTask, flavor: FsmFlavor = 'murom'): string {
  const out: string[] = [`fsm ${flavor === 'murka' ? 'MURKA' : 'MUROM'} ${task.type}`, 'outcomes done error'];
  if (flavor === 'murka') {
    const initial = (lines: string[]) => lines.map((l, i) => (i === 0 ? `${l} initial` : l));
    if (task.type === 'depalletize') out.push(...initial(copySkills(depalStates(true, ''))));
    else if (task.type === 'collect_order') out.push(...initial(copyOrderStates(true, '')));
    else throw new Error(COPY_TASKS_MSG(task.type));
    return out.join('\n');
  }
  if (task.type === 'mission') {
    const stages: Array<[string, string[]]> = [];
    if (task.scan.length) stages.push(['SCAN', st('SCAN', 'scan', ['ok', 'fail'], { ok: 'done', fail: 'error' }, '', '    ')]);
    if (task.depal.length) stages.push(['DEPAL', depalStates(false, '    ')]);
    if (task.order.length) stages.push(['ORDER', orderStates(false, '    ')]);
    stages.push(['DOCK', st('NAV_DOCK', 'nav', ['ok', 'fail'], { ok: 'done', fail: 'error' }, 'key=dock_nav', '    ')]);
    stages.forEach(([phase, inner], i) => {
      const nxt = i + 1 < stages.length ? `PH_${stages[i + 1][0]}` : 'FINISH';
      const params = phase === 'SCAN' ? ` scan=${task.scan.join(',')}` : phase === 'DEPAL' ? ` depal=${task.depal.join(',')}` : phase === 'ORDER' ? ` order=${task.order.join(',')}` : '';
      out.push(...st(`PH_${phase}`, 'phase', ['next'], { next: phase }, `phase=${phase}${params}`, '', i === 0));
      out.push(`machine ${phase}`, '  outcomes done error', ...inner.map((l) => l.slice(2)), `  done -> ${nxt}`, `  error -> ERR_${phase}`);
      out.push(...st(`ERR_${phase}`, 'record_error', ['next'], { next: nxt }, `phase=${phase}`));
    });
    out.push(...st('FINISH', 'finish', ['done', 'error'], { done: 'done', error: 'error' }));
    return out.join('\n');
  }
  if (task.type === 'depalletize') out.push(...depalStates(true, '').map((l, i) => (i === 0 ? `${l} initial` : l)));
  else if (task.type === 'collect_order') out.push(...orderStates(true, '').map((l, i) => (i === 0 ? `${l} initial` : l)));
  else if (task.type === 'scan') out.push(...st('SCAN', 'scan', ['ok', 'fail'], { ok: 'done', fail: 'error' }, '', '', true));
  else out.push(...st(task.type.toUpperCase(), 'nav', ['ok', 'fail'], { ok: 'done', fail: 'error' }, 'key=nav', '', true));
  return out.join('\n');
}

// ------------------------------------------------------------------- held objects (recovery.py)
type Hand = 'left' | 'right' | 'both';
export class HeldObjects {
  private byArm: Partial<Record<Hand, string>> = {};
  snapshot(): Partial<Record<Hand, string>> { return { ...this.byArm }; }
  private put(arm: Hand, name: string): void {
    const held: Partial<Record<Hand, string>> = {};
    for (const [a, n] of Object.entries(this.byArm) as Array<[Hand, string]>) if (n !== name && a !== arm) held[a] = n;
    if (arm === 'both') for (const k of Object.keys(held)) delete held[k as Hand]; else delete held.both;
    held[arm] = name; this.byArm = held;
  }
  onPick(r: PickResult): string {
    if (r.status !== 'succeeded' && r.status !== 'canceled') return '';
    if (!r.objectName || !['left', 'right', 'both'].includes(r.arm) || (r.status === 'succeeded' && !r.success)) return '';
    this.put(r.arm as Hand, r.objectName); return r.objectName;
  }
  onPlace(name: string, arm: string, r: PlaceResult): void { if (r.status === 'succeeded' && r.success) { const h: Partial<Record<Hand, string>> = {}; for (const [a, n] of Object.entries(this.byArm) as Array<[Hand, string]>) if (n !== name && a !== arm) h[a] = n; this.byArm = h; } }
  merged(objects: Record<string, { location: string }> | null): Partial<Record<Hand, string>> {
    if (!objects || !Object.keys(objects).length) return this.snapshot();
    const held: Partial<Record<Hand, string>> = {};
    for (const [name, o] of Object.entries(objects).sort((a, b) => (a[0] < b[0] ? -1 : 1))) { const loc = o.location ?? ''; if (loc.startsWith('carried_') && ['left', 'right', 'both'].includes(loc.slice(8))) held[loc.slice(8) as Hand] = name; }
    return held;
  }
}
export const describeHeld = (held: Partial<Record<Hand, string>>) => Object.entries(held).sort().map(([a, n]) => `${a === 'both' ? 'обе руки' : `рука ${a}`} — ${n}`).join(', ');
export function busyMessage(held: Partial<Record<Hand, string>>, need: string[], purpose: string): string {
  const busy = (Object.entries(held) as Array<[Hand, string]>).filter(([a]) => a === 'both' || need.includes(a)).sort();
  if (!busy.length) return '';
  const who = busy.map(([a, n]) => (a === 'both' ? `обе руки держат ${n}` : `рука ${a} держит ${n}`)).join('; ');
  const free = need.length > 1 ? 'нужны свободные руки' : `нужна свободная рука ${need[0]}`;
  return `${who} — товар не уложен после отмены или отказа и в корзину не убран; для ${purpose} ${free}`;
}

// ------------------------------------------------------------------- orchestrator
export interface ProgressEvent { t: number; taskType: string; state: string; phase: string; detail: string; percent: number }
export interface Detection { slotId: string; sku: string; confidence: number; position: [number, number, number]; width: number; height: number }
export interface OrchestratorOptions { verifyWithCamera?: boolean; assets?: AssetStore | null; onEvent?: (e: ProgressEvent) => void; dumpFrames?: boolean; catalog: Record<string, Prototype>; flavor?: FsmFlavor }
type BB = Record<string, any>;

export class RetailOrchestrator {
  readonly inventory = new Map<string, [string, number]>();
  readonly held = new HeldObjects();
  readonly progress: ProgressEvent[] = [];
  readonly states: Array<{ t: number; state: string }> = [];
  readonly scans: Array<{ t: number; target: string; views: SlotView[] }> = [];
  readonly dumps: Array<{ target: string; ids: string[]; frame: Frame }> = [];
  readonly navLog: Array<{ t0: number; t1: number; state: string; goal: string; outcome: NavOutcome }> = [];
  readonly opLog: Array<{ t0: number; t1: number; state: string; kind: 'pick' | 'place'; result: PickResult | PlaceResult }> = [];
  readonly log: string[] = [];
  readonly verifyWithCamera: boolean;
  private cancelFlag = false;
  task: MissionTask | null = null;
  /** The project's orchestrator family (station variant): its tasks and defaults. */
  readonly flavor: FsmFlavor;
  constructor(readonly sim: MuromSim, readonly nav: Navigator, readonly manip: ManipServer, readonly L: RetailLayout, readonly opts: OrchestratorOptions) { this.verifyWithCamera = opts.verifyWithCamera ?? true; this.flavor = opts.flavor ?? 'murom'; }
  private say(m: string): void { this.log.push(`[${this.sim.time.toFixed(2)}] ${m}`); }
  cancel(): void { this.cancelFlag = true; }
  get cancelRequested(): boolean { return this.cancelFlag; }
  get compactView(): boolean { return this.sim.h.hasWrist; }
  get cameraFwd(): number { return this.sim.h.hasWrist ? 0.11 : 0.086; }
  pose(): P3 { return this.sim.pose(); }
  objects(): Record<string, { sku: string; location: string; pos: [number, number, number] }> { return this.sim.registry(); }
  basketContents(arm = 'left'): string[] { return Object.entries(this.objects()).filter(([, o]) => o.location.startsWith('basket_') && (this.L.basket[o.location]?.arm ?? 'left') === arm).map(([n]) => n); }
  basketFull(arm: string): boolean { const occ = new Set(Object.values(this.objects()).map((o) => o.location)); return Object.values(this.L.basket).filter((b) => b.arm === arm).every((b) => occ.has(b.id)); }
  heldNow(): Partial<Record<Hand, string>> { return this.held.merged(this.objects()); }

  publishState(name: string, bb: BB): void {
    this.states.push({ t: this.sim.time, state: name }); bb.murom_state = name;
    const type = this.task?.type ?? '';
    const done = Number(bb.done_items ?? 0);
    let percent: number;
    if (type === 'mission' && bb.phase) { const [base, w] = PHASES[bb.phase] ?? [0, 0]; percent = Math.min(100, base + w * Math.min(1, done / Math.max(1, Number(bb.phase_total ?? 1)))); }
    else percent = Math.min(100, (100 * done) / Math.max(1, Number(bb.phase_total ?? 1)));
    const e: ProgressEvent = { t: this.sim.time, taskType: type, state: name, phase: String(bb.phase ?? ''), detail: String(bb.detail ?? ''), percent };
    this.progress.push(e); this.opts.onEvent?.(e);
  }

  // ------------------------------------------------------------- head / arms
  *moveHead(pan0: number, tilt0: number, duration = 1.2, wait = true): Task<boolean> {
    const pan = clamp(pan0, -HEAD_PAN_LIMIT, HEAD_PAN_LIMIT), tilt = clamp(tilt0, HEAD_TILT_LIMITS[0], HEAD_TILT_LIMITS[1]);
    this.sim.sendTraj(['head_pan_joint', 'head_tilt_joint'], [pan, tilt], duration);
    if (!wait) return true;
    const tEnd = this.sim.time + duration + 3.0;
    while (this.sim.time < tEnd) {
      if (Math.abs(this.sim.joint('head_pan_joint') - pan) < 0.03 && Math.abs(this.sim.joint('head_tilt_joint') - tilt) < 0.03) { yield 0.5; return true; }
      yield 0.05;
    }
    this.say(`голова не вышла в (${pan.toFixed(2)}, ${tilt.toFixed(2)})`);
    return false;
  }
  *lookAtSlot(slotId: string): Task<boolean> {
    const s = this.L.slots[slotId]; const kind = s.kind ?? 'gondola';
    const tx = s.pose.x, ty = s.pose.y, tz = s.pose.z + (kind === 'gondola' || kind === 'packing' ? 0.08 : 0.14);
    const [rx, ry, ryaw] = this.pose();
    const ax = rx + HEAD_AXIS_X * Math.cos(ryaw), ay = ry + HEAD_AXIS_X * Math.sin(ryaw);
    const pan = norm(Math.atan2(ty - ay, tx - ax) - ryaw);
    const d = Math.max(0.05, Math.hypot(tx - ax, ty - ay) - this.cameraFwd);
    return yield* this.moveHead(pan, Math.atan2(CAMERA_Z - tz, d));
  }
  *armsForView(observe: boolean): Task<void> {
    if (!this.compactView) return;
    const held = this.heldNow();
    const sides = ([['l', 'left'], ['r', 'right']] as Array<[string, Hand]>).filter(([, a]) => !held[a] && !held.both).map(([s]) => s);
    if (!sides.length) return;
    const q = observe ? OBS_ARM : TRAVEL_ARM; const names: string[] = [], targets: number[] = [];
    for (const s of sides) { names.push(...ARM_JOINTS.map((j) => `${s}_${j}`), `${s}_wrist_roll_joint`, `${s}_wrist_pitch_joint`); targets.push(...q, 0, 0); }
    this.sim.sendTraj(names, targets, 2.0);
    const tEnd = this.sim.time + 5.0;
    while (this.sim.time < tEnd) { if (names.every((n, i) => Math.abs(this.sim.joint(n) - targets[i]) < 0.1)) return; yield 0.05; }
    this.say(`руки не вышли в позу ${observe ? 'осмотра' : 'переноски'}`);
  }
  *restorePose(waitHead = true): Task<void> { yield* this.moveHead(0, 0, 1.2, waitHead); yield* this.armsForView(false); }

  // ------------------------------------------------------------- perception
  /** /murom/perception/scan: detections of the visible slots (null: no frames). */
  *scan(target: string): Task<Detection[] | null> {
    const ids = scanTargets(this.L, target);
    if (!ids.length) return null;
    yield SCAN_FRAME_WAIT;
    const cam = this.sim.h.camera; if (!cam) return null;
    const frame = renderHeadFrame(this.sim.station, cam, this.sim.h.depth, this.opts.assets ?? null);
    const views = recognize(this.L, ids, frame, this.opts.catalog);
    if (this.opts.dumpFrames) this.dumps.push({ target, ids, frame: { ...frame, render: undefined } as Frame });
    this.scans.push({ t: this.sim.time, target, views });
    const dets: Detection[] = [];
    for (const v of views) { if (!v.visible) continue; dets.push({ slotId: v.slotId, sku: v.sku, confidence: v.confidence, position: v.position, width: v.width, height: v.height }); this.inventory.set(v.slotId, [v.sku, v.confidence]); }
    this.say(`scan ${target}: видно слотов ${dets.length} из ${ids.length} (${[...new Set(dets.map((d) => d.sku).filter((s) => s && s !== 'unknown'))].sort().join(', ')})`);
    return dets;
  }
  orderCandidates(sku: string, bad: Set<string>): Array<[string, string, string]> {
    const objs = this.objects();
    const bySlot = new Map<string, string>(); for (const [n, o] of Object.entries(objs)) if (o.location.startsWith('shelf_') && !bySlot.has(o.location)) bySlot.set(o.location, n);
    const out: Array<[string, string, string]> = [];
    for (const [slotId, [seen, conf]] of [...this.inventory].sort((a, b) => (a[0] < b[0] ? -1 : 1))) if (seen === sku && conf >= 0.5 && bySlot.has(slotId) && !bad.has(slotId)) out.push([bySlot.get(slotId)!, slotId, 'камере']);
    const used = new Set(out.map((c) => c[1]));
    for (const [n, o] of Object.entries(objs)) {
      const loc = o.location;
      if (o.sku === sku && loc.startsWith('shelf_') && !used.has(loc) && !bad.has(loc)) { const seen = this.inventory.get(loc); if (seen && seen[0] !== sku && seen[0] !== 'unknown') continue; out.push([n, loc, 'реестру']); }
    }
    return out;
  }
  inventorySummary(): string {
    const counts: Record<string, number> = {}; let empty = 0, unknown = 0;
    for (const [sku] of this.inventory.values()) { if (sku === '') empty++; else if (sku === 'unknown') unknown++; else counts[sku] = (counts[sku] ?? 0) + 1; }
    const items = Object.entries(counts).sort().map(([k, v]) => `${k}×${v}`).join(', ') || 'товаров не найдено';
    return `${items}; пустых слотов ${empty}, не опознано ${unknown}`;
  }
  scanViews(shelf: string): P3[] {
    const slots = slotsByPrefix(this.L, `${shelf}_L0_S`); if (!slots.length) return [];
    const xs = slots.map((s) => this.L.slots[s].pose.x); const y = Math.min(...slots.map((s) => this.L.slots[s].standoff!.y));
    const centers: number[] = []; for (let i = 0; i + 1 < xs.length; i += 2) centers.push((xs[i] + xs[i + 1]) / 2);
    const yaw = this.compactView ? OBS_YAW : 0;
    return centers.map((c) => [c - HEAD_AXIS_X * Math.cos(yaw), y - HEAD_AXIS_X * Math.sin(yaw), yaw]);
  }

  // ------------------------------------------------------------- hands
  *releaseHeld(bb: BB, need: string[]): Task<void> {
    const held = this.heldNow();
    for (const arm of ['left', 'right'] as const) {
      const name = held[arm]; if (!need.includes(arm) || !name || this.cancelFlag) continue;
      if (this.basketFull(arm)) { this.say(`рука ${arm} держит ${name}, а слоты корзины для неё заняты — товар остаётся в руке`); continue; }
      bb.detail = `${name} из руки ${arm} — в корзину`; this.publishState('RELEASE_HELD', bb);
      const r = yield* this.manip.place(name, 'basket', arm, () => this.cancelFlag);
      this.held.onPlace(name, arm, r);
      this.say(r.success ? `RELEASE_HELD: ${name} из руки ${arm} уложен в корзину` : `RELEASE_HELD: ${name} из руки ${arm} в корзину не уложен: ${r.message}`);
    }
  }
  *freeHands(bb: BB, need: string[], purpose: string): Task<boolean> {
    yield* this.releaseHeld(bb, need);
    const msg = busyMessage(this.heldNow(), need, purpose);
    if (!msg) return true;
    this.say(msg); bb.errors = [...bb.errors, msg]; return false;
  }

  // ------------------------------------------------------------- states
  private navGoal(nav: NavSpec) {
    if (nav[0] === 'slot') return { targetId: nav[1], arm: nav[2] };
    if (nav[0] === 'target') return { targetId: nav[1] };
    return { pose: nav[1] as P3, skipFineAlign: nav[0] === 'pose_coarse' };
  }
  *stNav(name: string, bb: BB, key: string): Task<string> {
    const nav = bb[key] as NavSpec; const t0 = this.sim.time;
    const r = yield* this.nav.execute(this.navGoal(nav));
    this.navLog.push({ t0, t1: this.sim.time, state: name, goal: JSON.stringify(nav), outcome: r });
    if (!r.success) { this.say(`${name}: ${r.message}`); bb.errors = [...bb.errors, `${name}: ${r.message}`]; return 'fail'; }
    return 'ok';
  }
  *stPick(name: string, bb: BB, skipOnFail: boolean): Task<string> {
    const t0 = this.sim.time;
    const r = yield* this.manip.pick(bb.pick_object, bb.pick_arm, () => this.cancelFlag);
    this.opLog.push({ t0, t1: this.sim.time, state: name, kind: 'pick', result: r });
    const held = this.held.onPick(r);
    if (r.status !== 'succeeded' || !r.success) {
      let msg = r.message || 'нет ответа';
      if (held && !this.cancelFlag) { bb.held_object = held; bb.held_arm = r.arm; return 'ok'; }
      if (held) msg += `; ${held} остался в руке ${r.arm}`;
      this.say(`${name}: ${msg}`);
      if (skipOnFail && !this.cancelFlag) { bb.failed_items = [...bb.failed_items, [bb.pick_object, msg]]; return 'skip'; }
      bb.errors = [...bb.errors, `${name} ${bb.pick_object}: ${msg}`]; return 'fail';
    }
    bb.held_object = r.objectName; bb.held_arm = r.arm; return 'ok';
  }
  *stPlace(name: string, bb: BB): Task<string> {
    const t0 = this.sim.time;
    const r = yield* this.manip.place(bb.held_object, bb.place_location, bb.held_arm, () => this.cancelFlag);
    this.opLog.push({ t0, t1: this.sim.time, state: name, kind: 'place', result: r });
    this.held.onPlace(bb.held_object, bb.held_arm, r);
    if (r.status !== 'succeeded' || !r.success) { this.say(`${name}: ${r.message}`); bb.errors = [...bb.errors, `${name} ${bb.held_object}: ${r.message}`]; return 'fail'; }
    bb.done_items = Number(bb.done_items) + 1; return 'ok';
  }
  *stPlanDepal(bb: BB): Task<string> {
    if (bb.remaining_count <= 0) return 'none_left';
    if (!(yield* this.freeHands(bb, ['left', 'right'], 'депалетирования'))) return 'error';
    const pallet = bb.pallet_id, sku = bb.sku, shelf = bb.target_shelf;
    const objs = this.objects();
    const order = (loc: string): [number, number] => { const parts = loc.split('_'); const l = parts[parts.length - 2], s = parts[parts.length - 1]; const a = parseInt(l?.slice(1), 10), b = parseInt(s?.slice(1), 10); return Number.isFinite(a) && Number.isFinite(b) ? [-a, b] : [0, 99]; };
    const cands = Object.entries(objs).filter(([, o]) => o.sku === sku && o.location.startsWith(`${pallet}_L`));
    if (!cands.length) { if (Number(bb.done_items) === 0) { bb.errors = [...bb.errors, `на ${pallet} нет ${sku}`]; return 'error'; } return 'none_left'; }
    cands.sort((a, b) => { const x = order(a[1].location), y = order(b[1].location); return x[0] - y[0] || x[1] - y[1]; });
    const [objName, obj] = cands[0];
    const pickSlot = this.L.slots[obj.location];
    if (!pickSlot) { bb.errors = [...bb.errors, `слот ${obj.location} не описан`]; return 'error'; }
    const occupied = new Set(Object.values(objs).map((o) => o.location));
    let placeId: string | null = null;
    for (const sid of slotsByPrefix(this.L, `${shelf}_L`).filter((s) => !occupied.has(s))) {
      const [ok, why] = this.manip.checkPlace(objName, sid, 'both');
      if (ok) { placeId = sid; break; }
      this.say(`${sid}: ${why} — слот пропущен`);
    }
    if (!placeId) { bb.errors = [...bb.errors, `на ${shelf} нет достижимых свободных слотов`]; return 'error'; }
    const place = this.L.slots[placeId]; const arm = pickSlot.arm ?? 'both';
    if (place.arm !== arm && place.armAlt !== arm) { bb.errors = [...bb.errors, `${placeId}: нет подхода для руки ${arm}`]; return 'error'; }
    bb.pick_object = objName; bb.pick_arm = arm; bb.pick_nav = ['slot', obj.location, arm]; bb.place_location = placeId; bb.place_nav = ['slot', placeId, arm];
    bb.remaining_count = bb.remaining_count - 1; bb.detail = `${objName}: ${obj.location} -> ${placeId}`;
    this.say(`План: ${bb.detail} (рука ${arm})`);
    return 'found';
  }
  *stPlanItem(bb: BB): Task<string> {
    let basket = this.basketContents();
    if (basket.length >= BASKET_CAPACITY) return 'to_packing';
    if (!bb.remaining_skus.length) return basket.length ? 'to_packing' : 'finished';
    if (!(yield* this.freeHands(bb, ['left'], 'сборки заказа'))) return 'error';
    basket = this.basketContents();
    if (basket.length >= BASKET_CAPACITY) return 'to_packing';
    const remaining: string[] = [...bb.remaining_skus]; const bad = new Set<string>(bb.bad_slots);
    let cands: Array<[string, string, string]> = [];
    while (remaining.length) { cands = this.orderCandidates(remaining[0], bad); if (cands.length) break; this.say(`SKU ${remaining[0]} не найден на гондолах — пропуск`); bb.failed_items = [...bb.failed_items, [remaining[0], 'нет на полках']]; remaining.shift(); }
    if (!remaining.length) { bb.remaining_skus = remaining; return basket.length ? 'to_packing' : 'finished'; }
    const sku = remaining.shift()!; bb.remaining_skus = remaining;
    const [objName, location, source] = cands[0];
    const slot = this.L.slots[location];
    if (!slot) { bb.errors = [...bb.errors, `слот ${location} не описан`]; return 'error'; }
    bb.pick_object = objName; bb.pick_sku = sku; bb.pick_slot = location; bb.pick_arm = slot.arm; bb.pick_nav = ['slot', location, slot.arm];
    const so = slot.standoff ?? slot.approach!;
    let yaw = so.yaw;
    if (this.compactView) { const a = Math.atan2(slot.pose.y - so.y, slot.pose.x - so.x); yaw = a - Math.sign(norm(a - so.yaw) || 1) * OBS_PAN; }
    bb.verify_nav = ['pose', [so.x, so.y, yaw]]; bb.place_location = 'basket';
    bb.detail = `${sku} (${objName}) из ${location}`;
    this.say(`Заказ: беру ${bb.detail} — слот по ${source}`);
    return 'found';
  }
  *stUnloadPlan(bb: BB): Task<string> {
    const basket = this.basketContents(); if (!basket.length) return 'empty';
    if (!(yield* this.freeHands(bb, ['left'], 'выгрузки корзины'))) return 'error';
    const occupied = new Set(Object.values(this.objects()).map((o) => o.location));
    const free = slotsByPrefix(this.L, 'packing_S').filter((s) => !occupied.has(s));
    if (!free.length) { bb.errors = [...bb.errors, 'нет свободных слотов упаковки']; return 'error'; }
    const objName = [...basket].sort()[0];
    bb.pick_object = objName; bb.pick_arm = 'left'; bb.place_location = free[0]; bb.unload_nav = ['slot', free[0], 'left'];
    bb.detail = `выгрузка ${objName} -> ${free[0]}`;
    return 'item';
  }
  // ------------------------------------------------------------- states of the MURKA_Simulation copy (murka_fsm)
  /** basket_contents of the copy: every product in a basket_* slot. */
  basketAll(): string[] { return Object.entries(this.objects()).filter(([, o]) => o.location.startsWith('basket_')).map(([n]) => n); }
  /** PlanDepalStep of murka_fsm: no held-object recovery; no box of the SKU left on the pallet ends the task (none_left). */
  *stPlanDepalCopy(bb: BB): Task<string> {
    if (bb.remaining_count <= 0) return 'none_left';
    const pallet = bb.pallet_id, sku = bb.sku, shelf = bb.target_shelf;
    const objs = this.objects();
    const order = (loc: string): [number, number] => { const parts = loc.split('_'); if (parts.length < 3) return [0, 99]; const a = parseInt(parts[parts.length - 2].slice(1), 10), b = parseInt(parts[parts.length - 1].slice(1), 10); return Number.isFinite(a) && Number.isFinite(b) ? [-a, b] : [0, 99]; };
    const cands = Object.entries(objs).filter(([, o]) => o.sku === sku && o.location.startsWith(`${pallet}_L`));
    if (!cands.length) { this.say(`На ${pallet} не осталось ${sku}`); return 'none_left'; }
    cands.sort((a, b) => { const x = order(a[1].location), y = order(b[1].location); return x[0] - y[0] || x[1] - y[1]; });
    const [objName, obj] = cands[0];
    const pickSlot = this.L.slots[obj.location];
    if (!pickSlot) { bb.errors = [...bb.errors, `Слот ${obj.location} не описан`]; return 'error'; }
    const occupied = new Set(Object.values(objs).map((o) => o.location));
    const free = slotsByPrefix(this.L, `${shelf}_L`).filter((s) => !occupied.has(s));
    if (!free.length) { bb.errors = [...bb.errors, `На ${shelf} нет свободных слотов`]; return 'error'; }
    let placeId: string | null = null;
    for (const sid of free) {
      const [ok, why] = this.manip.checkPlace(objName, sid, 'both');
      if (ok) { placeId = sid; break; }
      this.say(`${sid}: ${why} — слот пропущен`);
    }
    if (!placeId) { bb.errors = [...bb.errors, `На ${shelf} нет свободных слотов, достижимых двуручной укладкой`]; return 'error'; }
    const place = this.L.slots[placeId]; const arm = pickSlot.arm ?? '';
    if (place.arm !== arm && place.armAlt !== arm) { bb.errors = [...bb.errors, `${placeId}: нет подхода для руки ${arm}`]; return 'error'; }
    bb.pick_object = objName; bb.pick_arm = arm; bb.pick_nav = ['slot', obj.location, arm]; bb.place_location = placeId; bb.place_nav = ['slot', placeId, arm];
    bb.remaining_count = bb.remaining_count - 1; bb.detail = `${objName}: ${obj.location} -> ${placeId}`;
    this.say(`План: ${bb.detail} (рука ${arm})`);
    return 'found';
  }
  /** PlanOrderItem of murka_fsm: the first instance of the SKU on a gondola by name (registry, no camera inventory). */
  *stPlanItemCopy(bb: BB): Task<string> {
    const basket = this.basketAll();
    if (basket.length >= BASKET_CAPACITY) return 'to_packing';
    if (!bb.remaining_skus.length) return basket.length ? 'to_packing' : 'finished';
    const objs = Object.entries(this.objects()).sort((a, b) => (a[0] < b[0] ? -1 : a[0] > b[0] ? 1 : 0));
    const remaining: string[] = [...bb.remaining_skus];
    let cands: typeof objs = [];
    while (remaining.length) {
      cands = objs.filter(([, o]) => o.sku === remaining[0] && o.location.startsWith('shelf_'));
      if (cands.length) break;
      this.say(`SKU ${remaining[0]} не найден на гондолах — пропуск`);
      bb.failed_items = [...bb.failed_items, [remaining[0], 'нет на полках']];
      remaining.shift();
    }
    if (!remaining.length) { bb.remaining_skus = remaining; return basket.length ? 'to_packing' : 'finished'; }
    const sku = remaining.shift()!; bb.remaining_skus = remaining;
    const [objName, obj] = cands[0];
    const slot = this.L.slots[obj.location];
    if (!slot) { bb.errors = [...bb.errors, `Слот ${obj.location} не описан`]; return 'error'; }
    bb.pick_object = objName; bb.pick_arm = slot.arm; bb.pick_nav = ['slot', obj.location, slot.arm]; bb.place_location = 'basket';
    bb.detail = `${sku} (${objName}) из ${obj.location}`;
    this.say(`Заказ: беру ${bb.detail}`);
    return 'found';
  }
  /** UnloadPlan of murka_fsm: the first basket product by name -> the first free packing slot. */
  *stUnloadPlanCopy(bb: BB): Task<string> {
    const basket = this.basketAll(); if (!basket.length) return 'empty';
    const occupied = new Set(Object.values(this.objects()).map((o) => o.location));
    const free = slotsByPrefix(this.L, 'packing_S').filter((s) => !occupied.has(s));
    if (!free.length) { bb.errors = [...bb.errors, 'Нет свободных слотов упаковки']; return 'error'; }
    const objName = [...basket].sort()[0];
    bb.pick_object = objName; bb.pick_arm = 'left'; bb.place_location = free[0]; bb.unload_nav = ['slot', free[0], 'left'];
    bb.detail = `выгрузка ${objName} -> ${free[0]}`;
    return 'item';
  }

  *stScan(bb: BB): Task<string> {
    const views: Array<[string, P3]> = []; for (const shelf of bb.scan_shelves as string[]) for (const p of this.scanViews(shelf)) views.push([shelf, p]);
    if (!views.length) { bb.errors = [...bb.errors, `нет гондол для осмотра: ${bb.scan_shelves}`]; return 'fail'; }
    bb.phase_total = views.length; let skipped = 0;
    yield* this.armsForView(true);
    try {      // head and arms go back to the travel pose on any exit (cancel, failure)
      for (let i = 0; i < views.length; i++) {
        if (this.cancelFlag) return 'fail';
        const [shelf, pose] = views[i];
        bb.detail = `осмотр ${shelf}, вид ${i + 1}/${views.length}`; this.publishState('NAV_VIEW', bb);
        const t0 = this.sim.time;
        const r = yield* this.nav.execute({ pose });
        this.navLog.push({ t0, t1: this.sim.time, state: 'NAV_VIEW', goal: JSON.stringify(pose), outcome: r });
        if (this.cancelFlag) return 'fail';
        if (!r.success) { this.say(`осмотр: не доехал до вида ${i + 1} (${r.message}) — пропуск`); skipped++; continue; }
        this.publishState('LOOK', bb);
        const ryaw = this.pose()[2];
        yield* this.moveHead(norm(Math.PI / 2 - ryaw), SCAN_TILT);
        this.publishState('RECOGNIZE', bb);
        let dets = yield* this.scan(shelf);
        if (dets === null && !this.cancelFlag) { this.say(`осмотр: распознавание вида ${i + 1} не удалось — повтор`); dets = yield* this.scan(shelf); }
        if (dets === null) { this.say(`осмотр: вид ${i + 1} (${shelf}) пропущен — распознавание не удалось дважды`); skipped++; continue; }
        bb.done_items = i + 1;
      }
    } finally {
      yield* this.restorePose();
    }
    let report = this.inventorySummary();
    if (skipped) report += `; пропущено видов ${skipped} из ${views.length}`;
    bb.detail = report; bb.scan_report = report; this.say(`Инвентаризация: ${report}`);
    return 'ok';
  }
  *stVerify(bb: BB): Task<string> {
    if (!this.verifyWithCamera) return 'ok';
    const slotId = bb.pick_slot as string, sku = bb.pick_sku as string;
    yield* this.armsForView(true);
    let dets: Detection[] | null = null;
    try {
      yield* this.lookAtSlot(slotId);
      if (this.cancelFlag) return 'fail';
      dets = yield* this.scan(slotId);
    } finally {
      yield* this.restorePose(false);
    }
    if (dets === null) { this.say('VERIFY_ITEM: распознавание не удалось — захват без проверки'); return 'ok'; }
    const det = dets.find((d) => d.slotId === slotId);
    const seen = det ? det.sku : 'не виден', conf = det ? det.confidence : 0;
    if (det && det.sku === sku && conf >= VERIFY_MIN_CONF) { this.say(`VERIFY_ITEM: в ${slotId} ${sku} (уверенность ${conf.toFixed(2)})`); return 'ok'; }
    this.say(`VERIFY_ITEM: в ${slotId} ожидался ${sku}, камера видит '${seen}' (${conf.toFixed(2)})`);
    const retries = { ...(bb.verify_retries as Record<string, number>) };
    bb.bad_slots = [...bb.bad_slots, slotId];
    if ((retries[sku] ?? 0) < 2) { retries[sku] = (retries[sku] ?? 0) + 1; bb.verify_retries = retries; bb.remaining_skus = [sku, ...bb.remaining_skus]; }
    else bb.failed_items = [...bb.failed_items, [sku, `камера не подтвердила товар (${seen})`]];
    return 'mismatch';
  }
  /** PhaseState setup; the stage parameters come from the phase skill's arguments (scan= / depal= / order=, written into
   *  the document by missionFsmDsl and editable in the Control tab), else from the task. */
  stPhase(bb: BB, phase: string, args: Record<string, unknown> = {}): string {
    bb.phase = phase; bb.done_items = 0; bb.phase_total = 1;
    const t = this.task; const list = (v: unknown) => String(v).split(',').map((x) => x.trim()).filter(Boolean);
    if (phase === 'SCAN') bb.scan_shelves = args.scan !== undefined ? list(args.scan) : t?.type === 'mission' ? [...t.scan] : [...MISSION_DEFAULTS.scan];
    if (phase === 'DEPAL') {
      const d = args.depal !== undefined ? list(args.depal) : t?.type === 'mission' && t.depal.length ? t.depal.map(String) : MISSION_DEFAULTS.depal.map(String);
      bb.pallet_id = d[0]; bb.sku = d[1]; bb.target_shelf = d[2]; bb.remaining_count = Math.max(1, parseInt(d[3] ?? '1', 10)); bb.phase_total = bb.remaining_count;
    }
    if (phase === 'ORDER') { bb.remaining_skus = args.order !== undefined ? list(args.order) : t?.type === 'mission' ? [...t.order] : [...MISSION_DEFAULTS.order]; bb.phase_total = bb.remaining_skus.length; }
    this.say(`=== Этап миссии ${phase} ===`);
    return 'next';
  }

  newBlackboard(task: MissionTask): BB {
    const bb: BB = { done_items: 0, phase_total: 1, failed_items: [], errors: [], phase_errors: [], bad_slots: [], verify_retries: {}, remaining_skus: [], remaining_count: 0, detail: '', home_nav: ['target', 'home'], dock_nav: ['target', 'dock'] };
    if (task.type === 'depalletize') { bb.pallet_id = task.pallet_id; bb.sku = task.sku; bb.target_shelf = task.target_shelf; bb.remaining_count = task.count; bb.phase_total = task.count; }
    else if (task.type === 'collect_order') { bb.remaining_skus = [...task.skus]; bb.phase_total = task.skus.length; }
    else if (task.type === 'scan') bb.scan_shelves = [...task.shelves];
    else if (task.type !== 'mission') bb.nav = task.nav;
    return bb;
  }

  /** The action library: one skill per state kind; every skill checks the cancel request first (MuromState.execute). */
  bindings(): WorldBindings {
    const runners = new Map<string, TaskRunner<string>>();
    const skill = (failOutcome: string, fn: (bb: BB, args: Record<string, unknown>, state: string) => Task<string>) => (ctx: TickContext, args: Record<string, unknown>, node: BTNode) => {
      const n = node as BTNode & { outcome?: string };
      let tr = runners.get(n.id);
      if (n.status !== 'running' || !tr) {
        const state = String((ctx.bb.fsm_state as string | undefined)?.split('/').pop() ?? '');
        const self = this;
        tr = new TaskRunner((function* () { if (self.cancelFlag) return failOutcome; if (!['murom.phase', 'murom.record_error'].includes(String(args.__skill))) self.publishState(state, ctx.bb); return yield* fn(ctx.bb, args, state); })(), this.sim);
        runners.set(n.id, tr);
      }
      tr.tick();
      if (!tr.done) return 'running';
      runners.delete(n.id);
      if (tr.error) { this.say(`skill error: ${(tr.error as Error).message}`); n.outcome = failOutcome; return 'failure'; }
      n.outcome = tr.value; return 'success';
    };
    const tag = (name: string, h: ReturnType<typeof skill>) => (ctx: TickContext, args: Record<string, unknown>, node: BTNode) => h(ctx, { ...args, __skill: name }, node);
    const self = this;
    const actions: WorldBindings['actions'] = {
      'murom.phase': tag('murom.phase', skill('next', function* (bb, a) { return self.stPhase(bb, String(a.phase), a); })),
      'murom.record_error': tag('murom.record_error', skill('next', function* (bb, a) { bb.phase_errors = [...bb.phase_errors, String(a.phase)]; self.say(`Этап ${a.phase} не выполнен — миссия продолжается`); return 'next'; })),
      'murom.finish': skill('error', function* (bb) { return bb.phase_errors.length ? 'error' : 'done'; }),
      'murom.nav': skill('fail', (bb, a, s) => this.stNav(s, bb, String(a.key))),
      'murom.nav_verify': skill('fail', function* (bb, a, s) { if (!self.verifyWithCamera) return 'ok'; return yield* self.stNav(s, bb, String(a.key)); }),
      'murom.pick': skill('fail', (bb, a, s) => this.stPick(s, bb, a.skip_on_fail === true || a.skip_on_fail === 'true')),
      'murom.place': skill('fail', (bb, _a, s) => this.stPlace(s, bb)),
      'murom.plan_depal': skill('error', (bb) => this.stPlanDepal(bb)),
      'murom.plan_item': skill('error', (bb) => this.stPlanItem(bb)),
      'murom.unload_plan': skill('error', (bb) => this.stUnloadPlan(bb)),
      'murom.scan': skill('fail', (bb) => this.stScan(bb)),
      'murom.verify': skill('fail', (bb) => this.stVerify(bb)),
      // murka_fsm of the MURKA_Simulation copy: NavState / PickState / PlaceState as above, its own planning states
      'murka.nav': skill('fail', (bb, a, s) => this.stNav(s, bb, String(a.key))),
      'murka.pick': skill('fail', (bb, a, s) => this.stPick(s, bb, a.skip_on_fail === true || a.skip_on_fail === 'true')),
      'murka.place': skill('fail', (bb, _a, s) => this.stPlace(s, bb)),
      'murka.plan_depal': skill('error', (bb) => this.stPlanDepalCopy(bb)),
      'murka.plan_item': skill('error', (bb) => this.stPlanItemCopy(bb)),
      'murka.unload_plan': skill('error', (bb) => this.stUnloadPlanCopy(bb)),
    };
    const sim = this.sim;
    return { sense: () => ({ battery: sim.battery.pct, docked: sim.battery.docked, x: sim.pose()[0], y: sim.pose()[1] }), actions, halt: {} };
  }

  /** Build the executor of a task: the FSM document + the action library on the simulation. */
  /** The executor of a task: its FSM document (or `dsl`, an edited document) + the action library on the simulation. */
  executor(task: MissionTask, dsl0?: string, log?: (m: string) => void): { exec: FsmExecutor; doc: FsmDoc; dsl: string } {
    this.task = task; this.cancelFlag = false;
    const dsl = dsl0 ?? missionFsmDsl(task, this.flavor); const doc = parseFsm(dsl);
    const host = this.bindings(); const sim = this.sim;
    const world: ExecutorWorld = { host, forRobot: () => null, step: (dt) => sim.step(dt) };
    const exec = new FsmExecutor({ doc, world, maxPerTick: 50, log });
    Object.assign(exec.bb, this.newBlackboard(task), doc.blackboard); // values declared in the document win
    return { exec, doc, dsl };
  }
  /** The task a document was made for (`fsm MUROM <type>` / `fsm MURKA <type>` header; defaults of that task). */
  static taskOfDsl(dsl: string, L: RetailLayout): MissionTask {
    const m = /^\s*fsm\s+(MUROM|MURKA)\s+(\w+)/m.exec(dsl); const type = m?.[2] ?? 'mission';
    const flavor: FsmFlavor = m?.[1] === 'MURKA' || (!m && /skill=murka\./.test(dsl)) ? 'murka' : 'murom';
    const name = type === 'mission' ? 'full' : type === 'collect_order' ? 'order' : type;
    try { return parseMission(name, [], L, flavor); } catch { return flavor === 'murka' ? parseMission('order', [], L, flavor) : parseMission('full', [], L); }
  }
}
