/**
 * Scripted-fleet player: TypeScript port of the MURKA agromarca `garden_supervisor/scripted_fleet.py` (v2), the
 * Webots supervisor that plays Mission B (waypoint windows with interp / tangent heading and linear / trapezoid
 * profiles, attachments of depot modules, time-bounded carries of crates, module reconfigurations with a lift-arc-landing
 * transfer, visible wheel / steering state). Same script JSON (`/garden_supervisor/commands`), same tick semantics:
 * one time line per robot, the last window with start_s <= t owns it, finished windows never write again, driven nodes
 * are seeded from the world once, carriers that the script does not drive are read live every tick.
 * Units like the original: metres, radians, seconds. `StationFleetWorld` binds it to an imported station (node names
 * resolve to items by name, then by DEF = name.upper() with '-' -> '_', as the Webots adapter does).
 */
import type { Item, Station } from '../../core/items/item';
import { Mat4, multiply, transl, rotz, invert } from '../../core/math/pose';

export type Pose4 = [number, number, number, number]; // x, y, z, yaw

export const WHEEL_RADIUS = 0.3388;
export const WHEELBASE = 2.19;
export const STEER_LIMIT = 0.3;
export const STEER_RATE = 1.5;
const WHEELED_PREFIXES = ['UTP-G'];
export const WHEEL_FIELDS = ['fl_wheel_rot', 'fr_wheel_rot', 'rl_wheel_rot', 'rr_wheel_rot'];
export const STEER_FIELDS = ['fl_steer_rot', 'fr_steer_rot'];
export const TURN_RATE_MAX = 0.5;
export const MIN_TURN_DIST = 0.3;
export const MIN_MOVE_FRAC = 0.4;
export const RAMP_MAX_S = 1.5;
export const TRANSFER_S = 1.8;
export const TRANSFER_ARC_M = 0.5;
export const CARRY_LIFT_M = 0.03;
export const CARRY_RAMP_S = 0.6;
const CARRY_GLIDE_SPEED = 1.0;
const CARRY_GLIDE_TURN = 1.0;
export const SLOT_TOL_M = 1.5;
export const MODULE_MOUNT_X = -0.95;
export const MODULE_MOUNT_Y = 0.0;
export const DECK_TOP_Z = 1.5065;
export const MOUNT_MARGIN_Z = 0.003;
export const MODULE_HEIGHTS: Record<string, number> = { module_sprayer: 0.35, module_harvester_arm: 0.55, module_scout_mast: 0.6 };
const YAW_MODES = ['interp', 'tangent'], PROFILES = ['linear', 'trapezoid'], DRIVES = ['forward', 'reverse', 'auto'];
const POSE_EPS = 1e-9;

/** Mount offset (x, y, z, yaw) of a depot module on the UTP-G rear deck pad (z = deck top + h/2 + 3 mm). */
export function moduleMountOffset(module: string): Pose4 {
  const h = MODULE_HEIGHTS[module];
  if (h === undefined) throw new Error(`no mount offset known for '${module}': pass an explicit 'offset' (x, y, z, yaw in the carrier frame)`);
  return [MODULE_MOUNT_X, MODULE_MOUNT_Y, Math.round((DECK_TOP_Z + 0.5 * h + MOUNT_MARGIN_Z) * 1e4) / 1e4, 0];
}

/** The world the fleet drives (the Webots Supervisor in the original). */
export interface FleetWorld {
  /** (x, y, z, yaw) of a node; throws when absent. */
  getPose(name: string): Pose4;
  setPose(name: string, x: number, y: number, z: number, yaw: number): void;
  hasNode?(name: string): boolean;
  resetPhysics?(name: string): void;
  /** Visible wheels / steering (SFRotation PROTO fields). */
  setFieldRotation?(name: string, field: string, rotation: number[]): void;
  trackPose?(name: string): void;
}

export interface WaypointJson { robot_id: string; start_s: number; end_s: number; target: { x: number; y: number; z?: number | null; yaw?: number | null }; yaw_mode?: string; drive?: string; profile?: string }
export interface FleetScript {
  yaw_mode?: string; profile?: string | null; turn_rate?: number;
  waypoints?: WaypointJson[];
  attachments?: Array<{ child: string; carrier: string; offset?: { x?: number; y?: number; z?: number; yaw?: number } | null; start_s?: number }>;
  carries?: Array<{ child: string; carrier: string; start_s: number; end_s: number; offset?: { x?: number; y?: number; yaw?: number } | null; lift_m?: number; ramp_s?: number }>;
  reconfigs?: Array<{ robot_id: string; start_s: number; depot_slot?: string; old_module?: string | null; new_module?: string | null; offset?: { x?: number; y?: number; z?: number; yaw?: number } | null; transfer_s?: number | null; slot_tol?: number }>;
  cmd_vel?: unknown[];
}

interface Waypoint { robot_id: string; start_s: number; end_s: number; x: number; y: number; z: number | null; yaw: number | null; yaw_mode: string | null; drive: string; profile: string | null }
interface Reconfig { robot_id: string; start_s: number; depot_slot: string; old_module: string | null; new_module: string | null; offset: Pose4 | null; transfer_s: number | null; slot_tol: number; fired: boolean }
interface Attachment { child: string; carrier: string; offset: Pose4; start_s: number }
interface Carry { child: string; carrier: string; start_s: number; end_s: number; offset: [number, number, number] | null; lift_m: number; ramp_s: number }

/** Wrap an angle difference into (-pi, pi] (Python modulo semantics). */
export function shortestAngle(delta: number): number {
  const m = 2 * Math.PI;
  const d = (((delta + Math.PI) % m) + m) % m - Math.PI;
  return d === -Math.PI ? Math.PI : d;
}
const compose = (b: Pose4, off: ArrayLike<number>): Pose4 => {
  const c = Math.cos(b[3]), s = Math.sin(b[3]);
  return [b[0] + c * off[0] - s * off[1], b[1] + s * off[0] + c * off[1], b[2] + off[2], b[3] + off[3]];
};
const relativePlanar = (b: Pose4, p: Pose4): [number, number, number] => {
  const dx = p[0] - b[0], dy = p[1] - b[1], c = Math.cos(-b[3]), s = Math.sin(-b[3]);
  return [c * dx - s * dy, s * dx + c * dy, shortestAngle(p[3] - b[3])];
};
const samePose = (a: Pose4, b: Pose4) => a.every((v, k) => Math.abs(v - b[k]) < POSE_EPS);
const smoothstep = (u: number) => { const v = u < 0 ? 0 : u > 1 ? 1 : u; return v * v * (3 - 2 * v); };
const isWheeled = (n: string) => WHEELED_PREFIXES.some((p) => n.startsWith(p));
const opt4 = (r: { x?: number; y?: number; z?: number; yaw?: number } | null | undefined): Pose4 | null => (r == null ? null : [+(r.x ?? 0), +(r.y ?? 0), +(r.z ?? 0), +(r.yaw ?? 0)]);
const check = (v: string | null | undefined, allowed: string[], what: string) => { if (v != null && !allowed.includes(v)) throw new Error(`${what} must be one of ${allowed.join('/')}, got '${v}'`); };

/** Pose-vs-time of one waypoint window (built when the window first owns its node). */
export class WindowPlan {
  readonly T: number; readonly x1: number; readonly y1: number; readonly z1: number;
  kind: 'tangent' | 'interp'; rateLimited = false; done = false; distance: number;
  TA = 0; TB = 0; TC = 0; dA = 0; dC = 0; heading: number; yawEnd: number; dyaw: number; ramp: number;
  constructor(readonly t0: number, t1: number, readonly p0: Pose4, target: [number, number, number | null, number | null], mode: string, readonly profile: string, drive: string, turnRate: number, minDist: number) {
    this.T = Math.max(1e-6, t1 - t0);
    const [x0, y0, z0, yaw0] = p0;
    const [tx, ty, tz, tyaw] = target;
    this.x1 = tx; this.y1 = ty; this.z1 = tz === null ? z0 : tz;
    const dx = tx - x0, dy = ty - y0;
    this.distance = Math.hypot(dx, dy);
    this.heading = yaw0;
    if (mode === 'tangent' && this.distance >= minDist) {
      this.kind = 'tangent';
      const fwd = Math.atan2(dy, dx);
      const head = drive === 'reverse' ? fwd + Math.PI : drive === 'auto' ? (Math.abs(shortestAngle(fwd - yaw0)) <= 0.5 * Math.PI + 1e-9 ? fwd : fwd + Math.PI) : fwd;
      this.dA = shortestAngle(head - yaw0);
      this.heading = yaw0 + this.dA;
      this.dC = tyaw !== null ? shortestAngle(tyaw - this.heading) : 0;
      let ta = (1.5 * Math.abs(this.dA)) / turnRate, tc = (1.5 * Math.abs(this.dC)) / turnRate;
      const budget = this.T * (1 - MIN_MOVE_FRAC);
      if (ta + tc > budget) { const k = budget / (ta + tc); ta *= k; tc *= k; this.rateLimited = true; }
      this.TA = ta; this.TC = tc; this.TB = this.T - ta - tc;
      this.yawEnd = this.heading + this.dC;
      this.dyaw = this.dA + this.dC;
    } else {
      this.kind = 'interp';
      const yaw1 = tyaw !== null ? tyaw : mode === 'interp' ? 0 : yaw0;
      this.dyaw = shortestAngle(yaw1 - yaw0);
      this.yawEnd = yaw0 + this.dyaw;
      this.TB = this.T;
    }
    this.ramp = profile === 'trapezoid' ? Math.min(RAMP_MAX_S, 0.25 * this.TB) : 0;
  }
  get peakSpeed(): number { return this.distance <= 0 ? 0 : this.distance / Math.max(1e-6, this.TB - this.ramp); }
  get peakTurnRate(): number {
    if (this.kind !== 'tangent') return Math.abs(this.dyaw) / this.T;
    return Math.max(this.TA > 0 ? (1.5 * Math.abs(this.dA)) / this.TA : 0, this.TC > 0 ? (1.5 * Math.abs(this.dC)) / this.TC : 0);
  }
  get endPose(): Pose4 { return [this.x1, this.y1, this.z1, this.yawEnd]; }
  private fraction(tau: number, length: number): number {
    if (tau <= 0) return 0;
    if (tau >= length) return 1;
    const rho = this.ramp;
    if (rho <= 1e-9) return tau / length;
    const v = 1 / (length - rho);
    if (tau < rho) return (0.5 * v * tau * tau) / rho;
    if (tau <= length - rho) return 0.5 * v * rho + v * (tau - rho);
    const rest = length - tau;
    return 1 - (0.5 * v * rest * rest) / rho;
  }
  poseAt(t: number): Pose4 {
    const [x0, y0, z0, yaw0] = this.p0;
    const tau = t - this.t0;
    if (tau <= 0) return this.p0;
    if (tau >= this.T) return this.endPose;
    if (this.kind === 'interp') { const s = this.fraction(tau, this.T); return [x0 + (this.x1 - x0) * s, y0 + (this.y1 - y0) * s, z0 + (this.z1 - z0) * s, yaw0 + this.dyaw * s]; }
    if (tau < this.TA) return [x0, y0, z0, yaw0 + this.dA * smoothstep(tau / this.TA)];
    const tb = tau - this.TA;
    if (tb < this.TB) { const s = this.fraction(tb, this.TB); return [x0 + (this.x1 - x0) * s, y0 + (this.y1 - y0) * s, z0 + (this.z1 - z0) * s, this.heading]; }
    const u = this.TC > 0 ? (tb - this.TB) / this.TC : 1;
    return [this.x1, this.y1, this.z1, this.heading + this.dC * smoothstep(u)];
  }
}

interface Transfer { t0: number; duration: number; start: Pose4; shelf: Pose4 | null; robot: string | null; offset: Pose4 | null }
export interface WheelState { phi: number; steer: number; w_phi: number; w_steer: number }
export interface PlanRow { robot_id: string; start_s: number; end_s: number; kind: string; distance_m: number; turn_a_s: number; drive_s: number; turn_c_s: number; peak_speed_mps: number; peak_turn_rate: number; rate_limited: boolean }

/** Plays back a script against a FleetWorld (see the module comment). */
export class ScriptedFleet {
  waypoints: Waypoint[] = []; reconfigs: Reconfig[] = []; attachments: Attachment[] = []; carries: Carry[] = [];
  depotShelfPose = new Map<string, [number, number, number]>();
  yawMode = 'interp'; profile: string | null = null; turnRate = TURN_RATE_MAX; minTurnDist = MIN_TURN_DIST; transferS = TRANSFER_S;
  errors: string[] = []; warnings: string[] = []; nSetPose = 0;
  log: (msg: string) => void = () => {};
  private timelines = new Map<string, Waypoint[]>(); private starts = new Map<string, number[]>();
  private driven = new Set<string>(); private wheeled: string[] = [];
  private plans = new Map<string, WindowPlan>(); private cmd = new Map<string, Pose4>(); private emitted = new Map<string, Pose4>();
  private shelfYaw = new Map<string, number>(); private attached = new Map<string, Attachment>(); private attApplied = new Set<string>();
  private xfers = new Map<string, Transfer>(); private fired = new Set<string>();
  private carryState = new Map<string, { done: boolean; captured?: [number, number, number]; target?: [number, number, number]; z0?: number; glide?: number }>();
  private wheelState = new Map<string, WheelState>(); private slotPose = new Map<string, Pose4 | null>();
  private tracked = new Set<string>(); private reported = new Set<string>(); private prevT: number | null = null;

  static fromScript(data: FleetScript): ScriptedFleet { const f = new ScriptedFleet(); f.loadScript(data); return f; }

  /** Replace the script, keeping what already happened (idempotent for an identical payload). */
  loadScript(data: FleetScript): void {
    const yawMode = String(data.yaw_mode ?? this.yawMode), profile = data.profile !== undefined ? data.profile : this.profile;
    check(yawMode, YAW_MODES, 'yaw_mode'); check(profile, PROFILES, 'profile');
    const waypoints = (data.waypoints ?? []).map((w): Waypoint => {
      const wp: Waypoint = { robot_id: String(w.robot_id), start_s: +w.start_s, end_s: +w.end_s, x: +w.target.x, y: +w.target.y, z: w.target.z == null ? null : +w.target.z, yaw: w.target.yaw == null ? null : +w.target.yaw, yaw_mode: w.yaw_mode ?? null, drive: String(w.drive ?? 'forward'), profile: w.profile ?? null };
      check(wp.yaw_mode, YAW_MODES, 'waypoint yaw_mode'); check(wp.profile, PROFILES, 'waypoint profile'); check(wp.drive, DRIVES, 'waypoint drive');
      return wp;
    });
    const reconfigs = (data.reconfigs ?? []).map((r): Reconfig => {
      const ev: Reconfig = { robot_id: String(r.robot_id), start_s: +r.start_s, depot_slot: String(r.depot_slot ?? ''), old_module: r.old_module ?? null, new_module: r.new_module ?? null, offset: opt4(r.offset), transfer_s: r.transfer_s == null ? null : +r.transfer_s, slot_tol: +(r.slot_tol ?? SLOT_TOL_M), fired: false };
      if (ev.new_module && !ev.offset) moduleMountOffset(ev.new_module); // fail loud for unknown modules
      return ev;
    });
    const attachments = (data.attachments ?? []).map((a): Attachment => ({ child: String(a.child), carrier: String(a.carrier), offset: opt4(a.offset) ?? moduleMountOffset(String(a.child)), start_s: +(a.start_s ?? 0) }));
    const carries = (data.carries ?? []).map((c): Carry => ({ child: String(c.child), carrier: String(c.carrier), start_s: +c.start_s, end_s: +c.end_s, offset: c.offset == null ? null : [+(c.offset.x ?? 0), +(c.offset.y ?? 0), +(c.offset.yaw ?? 0)], lift_m: +(c.lift_m ?? CARRY_LIFT_M), ramp_s: +(c.ramp_s ?? CARRY_RAMP_S) }));
    this.yawMode = yawMode; this.profile = profile ?? null; this.turnRate = +(data.turn_rate ?? this.turnRate);
    this.waypoints = waypoints; this.reconfigs = reconfigs; this.attachments = attachments; this.carries = carries;
    this.rebuild();
  }

  private rebuild(): void {
    const tl = new Map<string, Waypoint[]>();
    for (const wp of this.waypoints) { let l = tl.get(wp.robot_id); if (!l) tl.set(wp.robot_id, (l = [])); l.push(wp); }
    for (const l of tl.values()) l.sort((a, b) => a.start_s - b.start_s || a.end_s - b.end_s);
    this.timelines = tl;
    this.starts = new Map([...tl].map(([k, l]) => [k, l.map((w) => w.start_s)]));
    const driven = new Set<string>(tl.keys());
    for (const c of this.carries) driven.add(c.child);
    for (const a of this.attachments) driven.add(a.child);
    for (const ev of this.reconfigs) for (const m of [ev.old_module, ev.new_module]) if (m) driven.add(m);
    this.driven = driven;
    this.wheeled = [...tl.keys()].filter(isWheeled).sort();
    for (const ev of this.reconfigs) { const k = this.eventKey(ev); if (ev.fired) this.fired.add(k); else if (this.fired.has(k)) ev.fired = true; }
  }
  private eventKey(ev: Reconfig): string { return JSON.stringify([ev.robot_id, ev.start_s, ev.old_module, ev.new_module]); }

  /** Names of all nodes the script reads or writes. */
  requiredNodes(): string[] {
    const s = new Set<string>(this.timelines.keys());
    for (const a of this.attachments) { s.add(a.child); s.add(a.carrier); }
    for (const c of this.carries) { s.add(c.child); s.add(c.carrier); }
    for (const ev of this.reconfigs) { s.add(ev.robot_id); for (const m of [ev.old_module, ev.new_module]) if (m) s.add(m); }
    return [...s].sort();
  }
  validateWorld(world: FleetWorld): string[] {
    return this.requiredNodes().filter((n) => { if (world.hasNode) return !world.hasNode(n); try { world.getPose(n); return false; } catch { return true; } });
  }
  requireWorld(world: FleetWorld): void { const m = this.validateWorld(world); if (m.length) throw new Error(`nodes missing in the world: ${m.join(', ')}`); }

  private report(what: string, e: unknown): void {
    const msg = `${what}: ${(e as Error)?.message ?? String(e)}`;
    if (this.reported.has(msg)) return;
    this.reported.add(msg); this.errors.push(msg); this.log(`ERROR ${msg}`);
  }
  private warnOnce(key: string, msg: string): void { if (this.reported.has(key)) return; this.reported.add(key); this.warnings.push(msg); this.log(`WARNING ${msg}`); }

  /** Last pose the fleet commanded (or seeded) for a node. */
  commandedPose(name: string): Pose4 | undefined { return this.cmd.get(name); }
  private seed(world: FleetWorld, name: string): Pose4 {
    let p = this.cmd.get(name);
    if (!p) { const [x, y, z, yaw] = world.getPose(name); p = [+x, +y, +z, +yaw]; this.cmd.set(name, p); }
    return p;
  }
  private emit(world: FleetWorld, name: string, pose: Pose4): boolean {
    this.cmd.set(name, pose);
    const last = this.emitted.get(name);
    if (last && samePose(last, pose)) return false;
    world.setPose(name, pose[0], pose[1], pose[2], pose[3]);
    world.resetPhysics?.(name);
    this.emitted.set(name, pose);
    this.nSetPose++;
    return true;
  }
  private carrierPose(world: FleetWorld, name: string): Pose4 {
    if (this.driven.has(name)) return this.seed(world, name);
    if (world.trackPose && !this.tracked.has(name)) { this.tracked.add(name); world.trackPose(name); }
    const [x, y, z, yaw] = world.getPose(name);
    return [+x, +y, +z, +yaw];
  }
  /** Remember the shelf pose of depot modules (the first snapshot wins). */
  snapshotDepotModules(world: FleetWorld, names: string[]): void {
    const missing: string[] = [];
    for (const n of names) {
      if (this.depotShelfPose.has(n)) continue;
      let p: Pose4;
      try { p = world.getPose(n); } catch { missing.push(n); continue; }
      this.depotShelfPose.set(n, [p[0], p[1], p[2]]);
      this.shelfYaw.set(n, p[3]);
      if (!this.cmd.has(n)) this.cmd.set(n, [p[0], p[1], p[2], p[3]]);
    }
    if (missing.length) throw new Error(`nodes missing in the world: ${missing.join(', ')}`);
  }
  private shelf(name: string): Pose4 | null { const b = this.depotShelfPose.get(name); return b ? [b[0], b[1], b[2], this.shelfYaw.get(name) ?? 0] : null; }

  /** Advance the script to `simTime` seconds. */
  tick(world: FleetWorld, simTime: number): void {
    const t = +simTime;
    this.begin(world);
    const before = new Map<string, Pose4>();
    for (const n of this.wheeled) { const p = this.cmd.get(n); if (p) before.set(n, p); }
    this.tickWindows(world, t);
    this.tickReconfigs(world, t);
    this.tickAttachments(world, t);
    this.tickTransfers(world, t);
    this.tickCarries(world, t);
    this.tickWheels(world, t, before);
    this.prevT = t;
  }

  private begin(world: FleetWorld): void {
    const shelfNames = [...this.attachments.map((a) => a.child), ...this.reconfigs.flatMap((ev) => [ev.old_module, ev.new_module].filter((m): m is string => !!m))];
    for (const n of shelfNames) if (!this.depotShelfPose.has(n)) { try { this.snapshotDepotModules(world, [n]); } catch (e) { this.report(`shelf snapshot ${n}`, e); } }
    for (const n of [...this.driven].sort()) {
      if (this.emitted.has(n)) continue;
      try { this.emit(world, n, this.seed(world, n)); } catch (e) { this.report(`hold ${n}`, e); }
    }
  }

  private windowSettings(wp: Waypoint): [string, string] {
    const mode = wp.yaw_mode || this.yawMode;
    return [mode, wp.profile || this.profile || (mode === 'tangent' ? 'trapezoid' : 'linear')];
  }
  private makePlan(wp: Waypoint, p0: Pose4): WindowPlan {
    const [mode, profile] = this.windowSettings(wp);
    return new WindowPlan(wp.start_s, wp.end_s, p0, [wp.x, wp.y, wp.z, wp.yaw], mode, profile, wp.drive, this.turnRate, this.minTurnDist);
  }

  private tickWindows(world: FleetWorld, t: number): void {
    for (const [rid, lst] of this.timelines) {
      const st = this.starts.get(rid)!;
      let idx = -1;
      for (let i = 0; i < st.length && st[i] <= t; i++) idx = i; // bisect_right - 1 (starts are sorted)
      if (idx < 0) continue;
      const wp = lst[idx];
      try {
        const [mode, profile] = this.windowSettings(wp);
        const key = JSON.stringify([wp.robot_id, wp.start_s, wp.end_s, wp.x, wp.y, wp.z, wp.yaw, mode, profile, wp.drive]);
        let plan = this.plans.get(key);
        if (!plan) { plan = this.makePlan(wp, this.seed(world, rid)); this.plans.set(key, plan); }
        if (plan.done) continue; // a finished window let go (a carry / attachment may own the node now)
        this.emit(world, rid, plan.poseAt(t));
        if (t >= plan.t0 + plan.T) plan.done = true;
      } catch (e) { this.report(`window ${rid}@${wp.start_s.toFixed(3)}`, e); }
    }
  }

  private tickAttachments(world: FleetWorld, t: number): void {
    for (const a of this.attachments) {
      const key = JSON.stringify([a.child, a.carrier, a.start_s]);
      if (this.attApplied.has(key) || t < a.start_s) continue;
      this.attApplied.add(key);
      this.attached.set(a.child, a);
    }
    for (const [child, a] of [...this.attached]) {
      if (this.xfers.has(child)) continue;
      try { this.emit(world, child, compose(this.carrierPose(world, a.carrier), a.offset)); } catch (e) { this.report(`attachment ${child}->${a.carrier}`, e); }
    }
  }

  private tickReconfigs(world: FleetWorld, t: number): void {
    for (const ev of this.reconfigs) {
      if (ev.fired || t < ev.start_s) continue;
      try { this.fireReconfig(world, ev, t); } catch (e) { this.report(`reconfig ${ev.robot_id}@${ev.start_s.toFixed(3)}`, e); }
    }
  }
  private fireReconfig(world: FleetWorld, ev: Reconfig, t: number): void {
    const robot = this.carrierPose(world, ev.robot_id);
    this.checkSlot(world, ev, robot);
    const duration = ev.transfer_s ?? this.transferS;
    const plans: Array<[string, Transfer]> = [];
    if (ev.old_module) {
      const shelf = this.shelf(ev.old_module);
      if (!shelf) throw new Error(`nodes missing in the world: ${ev.old_module}`);
      plans.push([ev.old_module, { t0: t, duration: Math.max(1e-6, duration), start: this.seed(world, ev.old_module), shelf, robot: null, offset: null }]);
    }
    if (ev.new_module) plans.push([ev.new_module, { t0: t, duration: Math.max(1e-6, duration), start: this.seed(world, ev.new_module), shelf: null, robot: ev.robot_id, offset: ev.offset ?? moduleMountOffset(ev.new_module) }]);
    for (const [n, x] of plans) { this.attached.delete(n); this.xfers.set(n, x); }
    ev.fired = true;
    this.fired.add(this.eventKey(ev));
  }
  private checkSlot(world: FleetWorld, ev: Reconfig, robot: Pose4): void {
    if (!ev.depot_slot) return;
    if (!this.slotPose.has(ev.depot_slot)) { try { this.slotPose.set(ev.depot_slot, world.getPose(ev.depot_slot)); } catch { this.slotPose.set(ev.depot_slot, null); } }
    const slot = this.slotPose.get(ev.depot_slot);
    if (!slot) { this.warnOnce(`slot-missing:${ev.depot_slot}`, `reconfig of ${ev.robot_id}: depot slot '${ev.depot_slot}' is not in the world, distance not checked`); return; }
    const d = Math.hypot(robot[0] - slot[0], robot[1] - slot[1]);
    if (d > ev.slot_tol) this.warnOnce(`slot-far:${ev.robot_id}@${ev.start_s}`, `reconfig of ${ev.robot_id} at t=${ev.start_s.toFixed(1)}: robot is ${d.toFixed(2)} m from ${ev.depot_slot} (tolerance ${ev.slot_tol.toFixed(2)} m); swapping anyway`);
  }
  private tickTransfers(world: FleetWorld, t: number): void {
    for (const [name, xf] of [...this.xfers]) {
      try {
        const target = xf.robot !== null ? compose(this.carrierPose(world, xf.robot), xf.offset!) : xf.shelf!;
        const u = (t - xf.t0) / xf.duration;
        if (u >= 1) {
          this.xfers.delete(name);
          if (xf.robot !== null) this.attached.set(name, { child: name, carrier: xf.robot, offset: xf.offset!, start_s: xf.t0 });
          this.emit(world, name, target);
          continue;
        }
        const s = smoothstep(u), [sx, sy, sz, syaw] = xf.start;
        this.emit(world, name, [sx + (target[0] - sx) * s, sy + (target[1] - sy) * s, sz + (target[2] - sz) * s + TRANSFER_ARC_M * Math.sin(Math.PI * Math.max(0, u)), syaw + shortestAngle(target[3] - syaw) * s]);
      } catch (e) { this.report(`transfer ${name}`, e); }
    }
  }

  private tickCarries(world: FleetWorld, t: number): void {
    for (const c of this.carries) {
      const key = JSON.stringify([c.child, c.carrier, c.start_s, c.end_s]);
      const st = this.carryState.get(key);
      if (t < c.start_s || st?.done) continue;
      try { this.carryStep(world, t, c, key, st); } catch (e) { this.report(`carry ${c.child} by ${c.carrier}`, e); }
    }
  }
  private carryStep(world: FleetWorld, t: number, c: Carry, key: string, state: ReturnType<ScriptedFleet['carryState']['get']>): void {
    const span = Math.max(1e-6, c.end_s - c.start_s), ramp = Math.max(1e-6, Math.min(c.ramp_s, 0.5 * span));
    if (!state) {
      if (t >= c.end_s) { this.carryState.set(key, { done: true }); return; }
      const base = this.carrierPose(world, c.carrier), child = this.seed(world, c.child);
      const captured = relativePlanar(base, child), target = c.offset ?? captured;
      const dist = Math.hypot(target[0] - captured[0], target[1] - captured[1]), turn = Math.abs(shortestAngle(target[2] - captured[2]));
      const glide = Math.max(ramp, (1.5 * dist) / CARRY_GLIDE_SPEED, (1.5 * turn) / CARRY_GLIDE_TURN);
      state = { done: false, captured, target, z0: child[2], glide: Math.min(glide, span) };
      this.carryState.set(key, state);
    }
    const base = this.carrierPose(world, c.carrier), tt = Math.min(t, c.end_s);
    const rLift = Math.min(smoothstep((tt - c.start_s) / ramp), smoothstep((c.end_s - tt) / ramp));
    const rGlide = smoothstep((tt - c.start_s) / state.glide!);
    const cap = state.captured!, tgt = state.target!;
    const [x, y, , yaw] = compose(base, [cap[0] + (tgt[0] - cap[0]) * rGlide, cap[1] + (tgt[1] - cap[1]) * rGlide, 0, cap[2] + shortestAngle(tgt[2] - cap[2]) * rGlide]);
    this.emit(world, c.child, [x, y, state.z0! + c.lift_m * rLift, yaw]);
    if (t >= c.end_s) state.done = true;
  }

  private tickWheels(world: FleetWorld, t: number, before: Map<string, Pose4>): void {
    const dt = this.prevT === null ? 0 : Math.max(0, t - this.prevT);
    for (const name of this.wheeled) {
      const cur = this.cmd.get(name), old = before.get(name);
      if (!cur || !old) continue;
      let st = this.wheelState.get(name);
      if (!st) this.wheelState.set(name, (st = { phi: 0, steer: 0, w_phi: 0, w_steer: 0 }));
      try { this.advanceWheels(world, name, st, old, cur, dt); } catch (e) { this.report(`wheels ${name}`, e); }
    }
  }
  private advanceWheels(world: FleetWorld, name: string, st: WheelState, old: Pose4, cur: Pose4, dt: number): void {
    const dx = cur[0] - old[0], dy = cur[1] - old[1], dyaw = shortestAngle(cur[3] - old[3]);
    let ds = Math.hypot(dx, dy);
    if (ds > 0) { const mid = old[3] + 0.5 * dyaw; if (dx * Math.cos(mid) + dy * Math.sin(mid) < 0) ds = -ds; }
    st.phi += ds / WHEEL_RADIUS;
    let target = 0;
    if (ds !== 0 || Math.abs(dyaw) > 1e-12) { const sign = ds < 0 ? -1 : 1; target = Math.max(-STEER_LIMIT, Math.min(STEER_LIMIT, Math.atan2(WHEELBASE * dyaw * sign, Math.abs(ds)))); }
    const step = STEER_RATE * dt;
    st.steer += Math.max(-step, Math.min(step, target - st.steer));
    if (!world.setFieldRotation) return;
    if (Math.abs(st.phi - st.w_phi) > 1e-6) { for (const f of WHEEL_FIELDS) world.setFieldRotation(name, f, [0, 1, 0, st.phi]); st.w_phi = st.phi; }
    if (Math.abs(st.steer - st.w_steer) > 1e-6) { for (const f of STEER_FIELDS) world.setFieldRotation(name, f, [0, 0, 1, st.steer]); st.w_steer = st.steer; }
  }
  /** Accumulated visible-wheel angle and steering of a UTP-G (copy). */
  wheelStateOf(name: string): WheelState | undefined { const s = this.wheelState.get(name); return s ? { ...s } : undefined; }

  /** Chain every robot's windows offline from the seed poses and describe each one (rate_limited = window too short). */
  planReport(seed: Record<string, Pose4>): PlanRow[] {
    const rows: PlanRow[] = [];
    for (const rid of [...this.timelines.keys()].sort()) {
      let pose = seed[rid];
      for (const wp of this.timelines.get(rid)!) {
        const p = this.makePlan(wp, pose);
        rows.push({ robot_id: rid, start_s: wp.start_s, end_s: wp.end_s, kind: p.kind, distance_m: p.distance, turn_a_s: p.TA, drive_s: p.TB, turn_c_s: p.TC, peak_speed_mps: p.peakSpeed, peak_turn_rate: p.peakTurnRate, rate_limited: p.rateLimited });
        pose = p.endPose;
      }
    }
    return rows;
  }
}

/** FleetWorld over a studio station: node names -> items (name, then DEF), poses in metres on the items' absolute poses. */
export class StationFleetWorld implements FleetWorld {
  private cache = new Map<string, Item | null>();
  /** Wheel / steering field values written by the fleet (name.field -> axis-angle). */
  readonly fields = new Map<string, number[]>();
  constructor(readonly station: Station) {}
  item(name: string): Item | null {
    if (this.cache.has(name)) return this.cache.get(name)!;
    const def = name.toUpperCase().replace(/-/g, '_');
    let found: Item | null = null;
    for (const it of this.station.walk()) {
      if (it === this.station) continue;
      if (it.name === name) { found = it; break; }
      if (!found && it.params.def === def) found = it;
    }
    this.cache.set(name, found);
    return found;
  }
  hasNode(name: string): boolean { return !!this.item(name); }
  getPose(name: string): Pose4 {
    const it = this.item(name);
    if (!it) throw new Error(`no node DEF ${name.toUpperCase().replace(/-/g, '_')} (name '${name}') in the world`);
    const P = it.poseAbs();
    return [P[12] / 1000, P[13] / 1000, P[14] / 1000, Math.atan2(P[1], P[0])];
  }
  setPose(name: string, x: number, y: number, z: number, yaw: number): void {
    const it = this.item(name);
    if (!it) throw new Error(`no node '${name}' in the world`);
    const w = ((((yaw + Math.PI) % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI)) - Math.PI; // canonical angle like the Webots adapter
    const abs: Mat4 = multiply(transl(x * 1000, y * 1000, z * 1000), rotz(w));
    const parent = it.parent;
    it.setPose(parent && parent !== this.station ? multiply(invert(parent.poseAbs()), abs) : abs);
  }
  setFieldRotation(name: string, field: string, rotation: number[]): void { this.fields.set(`${name}.${field}`, [...rotation]); }
}

export interface FleetPlayback { t: number; ticks: number; errors: string[]; warnings: string[] }

/**
 * Run a script on a station like the Webots supervisor does (t = n * dt, n = 1, 2, ...), calling `onTick` after every
 * tick (collision checks, trace sampling). `depotModules` are snapshotted before the first tick (shelf poses).
 */
export function playScriptedFleet(station: Station, script: FleetScript | ScriptedFleet, opts: { dt?: number; tEnd: number; depotModules?: string[]; world?: FleetWorld; onTick?: (t: number, fleet: ScriptedFleet) => void }): FleetPlayback & { fleet: ScriptedFleet; world: FleetWorld } {
  const fleet = script instanceof ScriptedFleet ? script : ScriptedFleet.fromScript(script);
  const world = opts.world ?? new StationFleetWorld(station);
  if (opts.depotModules?.length) fleet.snapshotDepotModules(world, opts.depotModules);
  fleet.requireWorld(world);
  const dt = opts.dt ?? 0.016;
  let n = 0, t = 0;
  for (;;) {
    n++;
    t = n * dt;
    if (t > opts.tEnd) break;
    fleet.tick(world, t);
    opts.onTick?.(t, fleet);
  }
  return { t: (n - 1) * dt, ticks: n - 1, errors: fleet.errors, warnings: fleet.warnings, fleet, world };
}
