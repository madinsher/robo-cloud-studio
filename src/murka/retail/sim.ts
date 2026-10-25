/**
 * The robot-side simulation of the MURKA retail mission on the studio station — what murom_webots (driver +
 * supervisor plugin) and murom_navigation/battery_node provide in Webots, in the studio's own simulator:
 *  - base: differential drive integrated by the studio's drive model (src/mobile/controller.ts) from velocity commands
 *    (/cmd_vel with the 0.6 s watchdog) or from a planned pose path (Nav2 stand-in, see nav.ts), ground-truth pose;
 *  - joints: /murom/joint_trajectory semantics of murom_driver (linear interpolation from the current position, a new
 *    command replaces the trajectory of its joints, motor velocity limits), written into the studio arm / head Robots;
 *  - products: the supervisor registry (name, SKU, location id, pose), magnetic attach / detach (palm centre ≤ 0.18 m
 *    from the product top, two hands: middle of the palms ≤ 0.15 m from the box centre and palm spacing = box width
 *    ± 0.03 m), carried products follow the palm (or the two-palm basis), basket products ride with the robot;
 *  - battery_node: idle / drive / turn drain, 0.8 %/s charging within 6 cm (and 0.15 rad) of the dock pose;
 *  - an independent collision monitor: arm capsules, hand and carried load against the furniture colliders of the
 *    imported world (boundingObjects), and the base footprint against the same colliders.
 * Units: metres / radians in the API (like the MURKA code), the station stays in mm / degrees.
 */
import { Station, SceneObject, Item, GeometryRef } from '../../core/items/item';
import { Robot } from '../../core/items/robot';
import { MobileRobot } from '../../mobile/items';
import { Mat4, multiply, invert, fromArray, identity, transl, rotz } from '../../core/math/pose';
import { integrate, stepMobile } from '../../mobile/controller';
import { StationRuntime } from '../../core/motion/runtime';
import { ArmKinematics, V3, M3, mv3, rot, mm3, sdBox, R_UPPER_ARM, R_FOREARM, PALM_SURFACE } from './kinematics';
import type { RetailLayout } from './layout';
import type { MuromHandles, RetailStationResult } from './station';
import type { AssetStore } from '../../scene/assets';

export const ATTACH_MAX_DIST = 0.18, BOTH_MAX_DIST = 0.15, BOTH_WIDTH_TOL = 0.03, GRIP_CLEARANCE = 0.012;
export const CMD_VEL_TIMEOUT = 0.6;
/** Penetration tolerance of the collision monitor (MURKA test_clearance.py: 5 mm). */
export const COLLISION_TOL = 0.005;
const RAD = 180 / Math.PI, DEGR = Math.PI / 180;
const wrapPi = (a: number) => Math.atan2(Math.sin(a), Math.cos(a));

export interface RetailObject {
  name: string; sku: string; size: V3; item: SceneObject | null;
  location: string;
  /** world centre (m) and yaw (rad) */
  pos: V3; yaw: number;
  mode: 'static' | 'palm' | 'tip' | 'both' | 'basket';
  arm?: 'left' | 'right';
  /** palm-frame pose of the object (row-major R + t), basket: robot-frame (x, y, z, yaw), both: (a, b, c, yaw0, boxYaw0) */
  rel?: number[];
}

export interface CollisionEvent { t: number; part: string; item: string; depth: number }
interface Obb { item: string; T: Mat4; inv: Mat4; half: V3; center: V3; radius: number; lo: V3; hi: V3 }
type Aabb = [V3, V3];
const aabbOf = (pts: Array<{ p: V3; r: number }>): Aabb => { const lo: V3 = [Infinity, Infinity, Infinity], hi: V3 = [-Infinity, -Infinity, -Infinity]; for (const { p, r } of pts) for (let a = 0; a < 3; a++) { lo[a] = Math.min(lo[a], p[a] - r); hi[a] = Math.max(hi[a], p[a] + r); } return [lo, hi]; };
const overlaps = (b: Obb, a: Aabb, m = 0.01) => a[0][0] <= b.hi[0] + m && a[1][0] >= b.lo[0] - m && a[0][1] <= b.hi[1] + m && a[1][1] >= b.lo[1] - m && a[0][2] <= b.hi[2] + m && a[1][2] >= b.lo[2] - m;

interface JointSeg { from: number; to: number; t0: number; dur: number }

export class MuromSim {
  time = 0;
  readonly station: Station; readonly h: MuromHandles; readonly L: RetailLayout;
  readonly kin = { left: new ArmKinematics('left'), right: new ArmKinematics('right') };
  readonly objects = new Map<string, RetailObject>();
  readonly attached: { left: string | null; right: string | null; both: string | null } = { left: null, right: null, both: null };
  /** Battery (%) and docked flag (battery_node). */
  battery = { pct: 100, docked: false, idle: 0.01, drivePerM: 0.30, turnPerRad: 0.04, chargePerS: 0.8 };
  readonly collisions: CollisionEvent[] = [];
  readonly baseContacts: CollisionEvent[] = [];
  /** Largest penetration seen (m) per category. */
  maxArmPen = 0; maxBasePen = 0;
  /** Distance driven (m) and turned (rad). */
  odometer = 0; turned = 0;
  readonly log: string[] = [];
  private segs = new Map<string, JointSeg>();
  private cmd = { v: 0, w: 0, t: -1e9 };
  private pathMode: { speed: number; tol: number; headTol: number } | null = null;
  private furniture: Obb[] = [];
  private activeContacts = new Map<string, CollisionEvent>();
  private runtime: StationRuntime;
  /** Motor velocity limits (rad/s) by joint (PROTO maxVelocity). */
  private vmax = new Map<string, number>();

  /** Arm geometry of the collision monitor: capsule radii and hand points (wrist / palm link frames, m). */
  readonly armGeom: Record<'left' | 'right', { rUp: number; rFore: number; wrist: V3[]; palm: V3[]; source: 'mesh' | 'points' }>;
  constructor(r: RetailStationResult, opts: { batteryPct?: number; assets?: AssetStore | null } = {}) {
    this.station = r.station; this.h = r.robot; this.L = r.layout;
    this.armGeom = { left: armGeometry(this.h.left, this.kin.left, this.h.hasWrist, opts.assets ?? null), right: armGeometry(this.h.right, this.kin.right, this.h.hasWrist, opts.assets ?? null) };
    this.battery.pct = opts.batteryPct ?? 100;
    this.runtime = StationRuntime.of(this.station);
    for (const [n, [rb, i]] of this.h.joints) this.vmax.set(n, (rb.jointMaxVelocities()[i] ?? 171.9) * DEGR);
    // product registry from the station (spawned by loadRetailLayout from products.yaml)
    for (const inst of r.layout.instances) {
      const it = this.station.find(inst.name);
      const cat = r.layout.catalog[inst.sku];
      const item = it instanceof SceneObject ? it : null;
      const P = item ? item.poseAbs() : identity();
      this.objects.set(inst.name, { name: inst.name, sku: inst.sku, size: cat ? [...cat.size] : [0.1, 0.1, 0.1], item, location: inst.slot, pos: [P[12] / 1000, P[13] / 1000, P[14] / 1000], yaw: Math.atan2(P[1], P[0]), mode: 'static' });
    }
    this.collectFurniture();
    this.updateDocked();
  }

  // ------------------------------------------------------------------ state
  get base(): MobileRobot { return this.h.base; }
  /** Ground-truth pose (x, y, yaw) in the map (m, rad) — /murom/pose. */
  pose(): [number, number, number] { const s = this.base.state; return [s.x / 1000, s.y / 1000, s.theta * DEGR]; }
  /** z of base_link in the map (m). */
  get z0(): number { return this.base.poseAbs()[14] / 1000; }
  joint(name: string): number { const j = this.h.joints.get(name); return j ? j[0].joints()[j[1]] * DEGR : 0; }
  joints(names: string[]): number[] { return names.map((n) => this.joint(n)); }
  hasJoint(name: string): boolean { return this.h.joints.has(name); }
  private setJoint(name: string, v: number): void {
    const j = this.h.joints.get(name); if (!j) return;
    const q = j[0].joints().slice(); q[j[1]] = v * RAD; j[0].setJoints(q);
  }
  /** base_link point (m) -> map. */
  toMap(p: ArrayLike<number>): V3 { const [x, y, th] = this.pose(); const c = Math.cos(th), s = Math.sin(th); return [x + c * p[0] - s * p[1], y + s * p[0] + c * p[1], this.z0 + p[2]]; }
  /** map point -> base_link (m). */
  toLocal(p: ArrayLike<number>, pose = this.pose()): V3 { const [x, y, th] = pose; const dx = p[0] - x, dy = p[1] - y; return [Math.cos(th) * dx + Math.sin(th) * dy, -Math.sin(th) * dx + Math.cos(th) * dy, p[2] - this.z0]; }
  /** Arm joints [pitch, roll, yaw, elbow] and wrist [roll, pitch] (rad). */
  armState(arm: 'left' | 'right'): [number[], [number, number]] { const k = this.kin[arm]; const w = this.h.hasWrist ? this.joints(k.wristNames) : [0, 0]; return [this.joints(k.jointNames), [w[0], w[1]]]; }
  /** Palm frame in the map (rotation row-major, origin). */
  palmWorld(arm: 'left' | 'right'): [M3, V3] {
    const [q, w] = this.armState(arm); const [R, o] = this.kin[arm].palmFrame(q, w);
    const th = this.pose()[2];
    return [mm3(rot('z', th), R), this.toMap(o)];
  }
  /** Grip point in the map: v2 palm surface centre, classic forearm tip. */
  gripPoint(arm: 'left' | 'right'): V3 {
    const [q, w] = this.armState(arm); const k = this.kin[arm];
    return this.toMap(this.h.hasWrist ? k.palmPoint(q, w) : k.fk(q));
  }

  // ------------------------------------------------------------- commands
  /** /murom/joint_trajectory with one point: linear interpolation from the current positions over `duration`. */
  sendTraj(names: string[], targets: number[], duration: number): void {
    names.forEach((n, i) => { if (this.h.joints.has(n)) this.segs.set(n, { from: this.joint(n), to: targets[i], t0: this.time, dur: Math.max(1e-3, duration) }); });
  }
  /** Joints still moving along their commanded trajectory. */
  trajActive(names?: string[]): boolean { for (const [n, s] of this.segs) if ((!names || names.includes(n)) && this.time < s.t0 + s.dur) return true; return false; }
  /** /cmd_vel (m/s, rad/s) with the driver's watchdog. */
  cmdVel(v: number, w: number): void { this.pathMode = null; this.base.state.path = null; this.cmd = { v, w, t: this.time }; }
  stop(): void { this.cmdVel(0, 0); }
  /** Follow a pose path (mm, deg) with the studio path tracker (Nav2 stand-in). */
  followPath(path: number[][], o: { speed: number; tol: number; headTol?: number }): void {
    this.cmd = { v: 0, w: 0, t: -1e9 };
    this.base.state.path = path; this.base.state.pathIndex = 0; this.base.state.status = 'moving';
    this.pathMode = { speed: o.speed, tol: o.tol, headTol: o.headTol ?? 3 };
  }
  get pathActive(): boolean { return !!this.pathMode && !!this.base.state.path; }

  // ------------------------------------------------------------- products
  objWorld(o: RetailObject): Mat4 { return multiply(transl(o.pos[0] * 1000, o.pos[1] * 1000, o.pos[2] * 1000), rotz(o.yaw)); }
  private writeObject(o: RetailObject, R?: M3): void {
    if (!o.item) return;
    const M = R
      ? fromArray([R[0], R[3], R[6], 0, R[1], R[4], R[7], 0, R[2], R[5], R[8], 0, o.pos[0] * 1000, o.pos[1] * 1000, o.pos[2] * 1000, 1])
      : this.objWorld(o);
    o.item.setPoseAbs(M);
  }
  /** /murom/sim/attach. */
  attach(name: string, arm: 'left' | 'right' | 'both'): { success: boolean; message: string } {
    const o = this.objects.get(name);
    if (!o) return { success: false, message: `Объект ${name} не найден` };
    if (arm === 'both') {
      if (this.attached.left || this.attached.right || this.attached.both) return { success: false, message: `Руки заняты` };
      const tl = this.gripPoint('left'), tr = this.gripPoint('right');
      const mid: V3 = [(tl[0] + tr[0]) / 2, (tl[1] + tr[1]) / 2, (tl[2] + tr[2]) / 2];
      const dist = Math.hypot(mid[0] - o.pos[0], mid[1] - o.pos[1], mid[2] - o.pos[2]);
      const [lat, fwd] = bothBasis(tl, tr);
      if (this.h.hasWrist) {
        const c = Math.cos(o.yaw), s = Math.sin(o.yaw);
        // box width along the palm line: |lat · R| · half size, x2
        const width = 2 * (Math.abs(lat[0] * c + lat[1] * s) * o.size[0] / 2 + Math.abs(-lat[0] * s + lat[1] * c) * o.size[1] / 2);
        const sep = Math.hypot(tr[0] - tl[0], tr[1] - tl[1]);
        if (dist > BOTH_MAX_DIST || Math.abs(sep - width) > BOTH_WIDTH_TOL) return { success: false, message: `Ладони далеко от ${name}: середина-центр ${dist.toFixed(3)} м, между ладонями ${sep.toFixed(3)} при ширине ${width.toFixed(3)}` };
      } else if (dist > 0.35) return { success: false, message: `Кончики далеко от ${name}: середина-центр ${dist.toFixed(3)} м` };
      const rel = [o.pos[0] - mid[0], o.pos[1] - mid[1], o.pos[2] - mid[2]];
      o.rel = [rel[0] * lat[0] + rel[1] * lat[1], rel[0] * fwd[0] + rel[1] * fwd[1], rel[2], Math.atan2(fwd[1], fwd[0]), o.yaw];
      o.mode = 'both'; o.location = 'carried_both'; this.attached.both = name;
      return { success: true, message: `${name} захвачен ДВУМЯ руками (mid-dist ${dist.toFixed(3)})` };
    }
    if (this.attached[arm]) return { success: false, message: `Рука ${arm} уже держит ${this.attached[arm]}` };
    const tip = this.gripPoint(arm);
    const top: V3 = [o.pos[0], o.pos[1], o.pos[2] + o.size[2] / 2];
    const dist = Math.hypot(tip[0] - top[0], tip[1] - top[1], tip[2] - top[2]);
    if (dist > ATTACH_MAX_DIST) return { success: false, message: `${this.h.hasWrist ? 'Ладонь' : 'Кончик'} слишком далеко от ${name}: ${dist.toFixed(3)} м` };
    this.attached[arm] = name; o.location = `carried_${arm}`; o.arm = arm;
    if (this.h.hasWrist) {
      const [R, p] = this.palmWorld(arm);
      const Ro = rot('z', o.yaw);
      const Rt = [R[0], R[3], R[6], R[1], R[4], R[7], R[2], R[5], R[8]];
      const rr = mm3(Rt, Ro); const tt = mv3(Rt, [o.pos[0] - p[0], o.pos[1] - p[1], o.pos[2] - p[2]]);
      o.rel = [...rr, ...tt]; o.mode = 'palm';
    } else { o.mode = 'tip'; o.yaw = 0; }
    return { success: true, message: `${name} захвачен рукой ${arm} (dist ${dist.toFixed(3)})` };
  }
  /** /murom/sim/detach (place pose: position only, the yaw is kept). */
  detach(name: string, place: V3 | null, locationId: string): { success: boolean; message: string } {
    const o = this.objects.get(name);
    if (!o) return { success: false, message: `Объект ${name} не найден` };
    let arm: 'left' | 'right' | 'both' | null = null;
    if (this.attached.left === name) arm = 'left'; else if (this.attached.right === name) arm = 'right'; else if (this.attached.both === name) arm = 'both';
    if (!arm) return { success: false, message: `${name} не захвачен` };
    this.attached[arm] = null;
    const jump = place ? Math.hypot(place[0] - o.pos[0], place[1] - o.pos[1], place[2] - o.pos[2]) : 0;
    if (place) o.pos = [...place];
    o.location = locationId || 'unknown'; o.arm = undefined;
    if (o.location.startsWith('basket')) {
      const l = this.toLocal(o.pos);
      o.rel = [l[0], l[1], l[2], o.yaw - this.pose()[2]]; o.mode = 'basket';
    } else { o.mode = 'static'; o.rel = undefined; }
    this.writeObject(o);
    return { success: true, message: `${name} отпущен (${o.location}, скачок ${jump.toFixed(3)} м)` };
  }
  /** Registry (/murom/sim/objects): name -> {sku, location, pos}. */
  registry(): Record<string, { sku: string; location: string; pos: V3; yaw: number }> {
    const out: Record<string, { sku: string; location: string; pos: V3; yaw: number }> = {};
    for (const [n, o] of [...this.objects].sort((a, b) => (a[0] < b[0] ? -1 : 1))) out[n] = { sku: o.sku, location: o.location, pos: [...o.pos], yaw: o.yaw };
    return out;
  }

  // ------------------------------------------------------------------ step
  step(dt: number): void {
    // joints (murom_driver: linear interpolation, motor velocity limit)
    for (const [n, s] of this.segs) {
      const f = Math.min(1, (this.time + dt - s.t0) / s.dur);
      const want = s.from + (s.to - s.from) * f;
      const cur = this.joint(n); const lim = (this.vmax.get(n) ?? 3) * dt;
      this.setJoint(n, cur + Math.max(-lim, Math.min(lim, want - cur)));
      if (f >= 1 && Math.abs(want - this.joint(n)) < 1e-9) this.segs.delete(n);
    }
    // base
    const st = this.base.state;
    const x0 = st.x, y0 = st.y, th0 = st.theta;
    if (this.pathMode && st.path) {
      const moving = stepMobile(this.base, dt, { speed: this.pathMode.speed, goalTolerance: this.pathMode.tol, headingTolerance: this.pathMode.headTol });
      if (!moving) { this.pathMode = null; st.path = null; }
    } else {
      const live = this.time - this.cmd.t <= CMD_VEL_TIMEOUT;
      integrate(this.base, { v: live ? this.cmd.v * 1000 : 0, omega: live ? this.cmd.w * RAD : 0 }, dt);
    }
    const ds = Math.hypot(st.x - x0, st.y - y0) / 1000, dth = Math.abs(wrapPi((st.theta - th0) * DEGR));
    this.odometer += ds; this.turned += dth;
    this.time += dt;
    // carried / basket products
    this.updateObjects();
    // battery_node
    this.updateDocked();
    const b = this.battery;
    if (b.docked) b.pct = Math.min(100, b.pct + b.chargePerS * dt);
    else b.pct = Math.max(0, b.pct - (b.idle * dt + b.drivePerM * ds + b.turnPerRad * dth));
    this.base.battery.levelWh = (this.base.battery.capacityWh * b.pct) / 100;
    if (b.docked && st.status !== 'charging' && !this.pathMode) st.status = 'idle';
    // collision monitor
    this.monitor();
    this.runtime.tick(dt);
  }

  private updateDocked(): void {
    const d = this.L.dock; if (!d) { this.battery.docked = false; return; }
    const [x, y, th] = this.pose();
    this.battery.docked = Math.hypot(x - d.approach.x, y - d.approach.y) < d.radius && Math.abs(wrapPi(th - d.approach.yaw)) < 0.15;
  }

  private updateObjects(): void {
    for (const arm of ['left', 'right'] as const) {
      const n = this.attached[arm]; if (!n) continue;
      const o = this.objects.get(n)!;
      if (o.mode === 'palm' && o.rel) {
        const [R, p] = this.palmWorld(arm);
        const Ro = mm3(R, o.rel.slice(0, 9)); const t = mv3(R, o.rel.slice(9, 12));
        o.pos = [p[0] + t[0], p[1] + t[1], p[2] + t[2]]; o.yaw = Math.atan2(Ro[3], Ro[0]);
        this.writeObject(o, Ro);
      } else {
        const tip = this.gripPoint(arm);
        o.pos = [tip[0], tip[1], tip[2] - o.size[2] / 2 - GRIP_CLEARANCE];
        this.writeObject(o);
      }
    }
    if (this.attached.both) {
      const o = this.objects.get(this.attached.both)!; const [a, b, c, yaw0, boxYaw0] = o.rel!;
      const tl = this.gripPoint('left'), tr = this.gripPoint('right');
      const mid = [(tl[0] + tr[0]) / 2, (tl[1] + tr[1]) / 2, (tl[2] + tr[2]) / 2];
      const [lat, fwd] = bothBasis(tl, tr);
      o.pos = [mid[0] + a * lat[0] + b * fwd[0], mid[1] + a * lat[1] + b * fwd[1], mid[2] + c];
      o.yaw = boxYaw0 + Math.atan2(fwd[1], fwd[0]) - yaw0;
      this.writeObject(o);
    }
    const ryaw = this.pose()[2];
    for (const o of this.objects.values()) if (o.mode === 'basket' && o.rel) { o.pos = this.toMap(o.rel); o.yaw = ryaw + o.rel[3]; this.writeObject(o); }
  }

  // ------------------------------------------------------- collision monitor
  private collectFurniture(): void {
    const robotOf = (it: Item) => { for (let p: Item | null = it; p; p = p.parent) if (p instanceof Robot || p instanceof MobileRobot) return true; return false; };
    const products = new Set([...this.objects.values()].map((o) => o.item).filter(Boolean));
    for (const it of this.station.walk()) {
      if (!(it instanceof SceneObject) || products.has(it) || robotOf(it) || it.params.ground) continue;
      const refs = (it as unknown as { colliders?: GeometryRef[] }).colliders; if (!refs) continue;
      const P = it.poseAbs();
      for (const g of refs) {
        const pr = g.primitive; if (!pr || pr.kind === 'plane') continue;
        const T = multiply(P, g.origin ? fromArray(g.origin) : identity());
        const half: V3 = pr.kind === 'box' ? [pr.size[0] / 2000, pr.size[1] / 2000, pr.size[2] / 2000] : pr.kind === 'sphere' ? [pr.radius / 1000, pr.radius / 1000, pr.radius / 1000] : [pr.radius / 1000, pr.radius / 1000, pr.length / 2000];
        const Tm = fromArray(Array.from(T).map((v, i) => (i >= 12 && i < 15 ? v / 1000 : v)));
        const lo: V3 = [Infinity, Infinity, Infinity], hi: V3 = [-Infinity, -Infinity, -Infinity];
        for (let c = 0; c < 8; c++) { const q = [c & 1 ? half[0] : -half[0], c & 2 ? half[1] : -half[1], c & 4 ? half[2] : -half[2]]; for (let a = 0; a < 3; a++) { const w = Tm[a] * q[0] + Tm[4 + a] * q[1] + Tm[8 + a] * q[2] + Tm[12 + a]; lo[a] = Math.min(lo[a], w); hi[a] = Math.max(hi[a], w); } }
        this.furniture.push({ item: it.name, T: Tm, inv: invert(Tm), half, center: [Tm[12], Tm[13], Tm[14]], radius: Math.hypot(...half), lo, hi });
      }
    }
  }
  /** Signed distance (m) of a map point to a furniture box (< 0 inside). */
  private sdObb(b: Obb, p: V3): number {
    const I = b.inv; const q: V3 = [I[0] * p[0] + I[4] * p[1] + I[8] * p[2] + I[12], I[1] * p[0] + I[5] * p[1] + I[9] * p[2] + I[13], I[2] * p[0] + I[6] * p[1] + I[10] * p[2] + I[14]];
    return sdBox(q, [-b.half[0], -b.half[1], -b.half[2]], b.half);
  }
  private armLocal: Record<string, { key: string; pts: Array<{ p: V3; r: number; part: string }> }> = {};
  /** Points of an arm (capsule samples with radius, hand points) in the map. */
  armSamples(arm: 'left' | 'right'): Array<{ p: V3; r: number; part: string }> {
    const [q, w] = this.armState(arm); const k = this.kin[arm];
    const key = `${q.join(',')}|${w.join(',')}`;
    let c = this.armLocal[arm];
    if (!c || c.key !== key) {
      const [pYaw, pElb, tip, rf] = k.chain(q); const G = this.armGeom[arm];
      const pts: Array<{ p: V3; r: number; part: string }> = [];
      for (const [a, b, r, part] of [[pYaw, pElb, G.rUp, 'upper_arm'], [pElb, tip, G.rFore, 'forearm']] as Array<[V3, V3, number, string]>) { const n = Math.max(2, Math.ceil(Math.hypot(b[0] - a[0], b[1] - a[1], b[2] - a[2]) / 0.02) + 1); for (let i = 0; i < n; i++) { const t = i / (n - 1); pts.push({ p: [a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]), a[2] + t * (b[2] - a[2])], r, part: `${arm}_${part}` }); } }
      if (this.h.hasWrist) {
        if (G.source === 'mesh') {
          const rw = mm3(rf, rot('z', w[0])), rp = mm3(rw, rot('x', w[1]));
          const org = mv3(rf, [0, 0, -0.03]); const o: V3 = [tip[0] + org[0], tip[1] + org[1], tip[2] + org[2]];
          for (const v of G.wrist) { const d = mv3(rw, v); pts.push({ p: [tip[0] + d[0], tip[1] + d[1], tip[2] + d[2]], r: 0, part: `${arm}_hand` }); }
          for (const v of G.palm) { const d = mv3(rp, v); pts.push({ p: [o[0] + d[0], o[1] + d[1], o[2] + d[2]], r: 0, part: `${arm}_hand` }); }
        } else for (const p of k.handPoints(q, w)) pts.push({ p, r: 0, part: `${arm}_hand` });
      }
      c = this.armLocal[arm] = { key, pts };
    }
    const [x, y, th] = this.pose(); const cs = Math.cos(th), sn = Math.sin(th), z0 = this.z0;
    return c.pts.map(({ p, r, part }) => ({ p: [x + cs * p[0] - sn * p[1], y + sn * p[0] + cs * p[1], z0 + p[2]] as V3, r, part }));
  }
  private bodyCache: V3[] | null = null;
  /** Surface samples (base_link) of the MUROM boundingObjects: torso box and the two wheels. */
  bodySamples(): V3[] {
    if (this.bodyCache) return this.bodyCache;
    const out: V3[] = [];
    const [lo, hi] = [[-0.17, -0.19, 0.612], [0.06, 0.19, 1.492]];
    for (let i = 0; i <= 4; i++) for (let j = 0; j <= 4; j++) for (let k = 0; k <= 6; k++) {
      if (i > 0 && i < 4 && j > 0 && j < 4 && k > 0 && k < 6) continue;
      out.push([lo[0] + (hi[0] - lo[0]) * i / 4, lo[1] + (hi[1] - lo[1]) * j / 4, lo[2] + (hi[2] - lo[2]) * k / 6]);
    }
    for (const sy of [1, -1]) for (let a = 0; a < 16; a++) for (const dy of [-0.04, 0.04]) out.push([0.177 * Math.cos((2 * Math.PI * a) / 16), sy * 0.2205 + dy, 0.177 + 0.177 * Math.sin((2 * Math.PI * a) / 16)]);
    return (this.bodyCache = out);
  }
  private loadSamples(o: RetailObject): V3[] {
    const c = Math.cos(o.yaw), s = Math.sin(o.yaw); const out: V3[] = [];
    for (const x of [-1, 0, 1]) for (const y of [-1, 0, 1]) for (const z of [-1, 0, 1]) { if (!x && !y && !z) continue; const lx = x * o.size[0] / 2, ly = y * o.size[1] / 2; out.push([o.pos[0] + c * lx - s * ly, o.pos[1] + s * lx + c * ly, o.pos[2] + z * o.size[2] / 2]); }
    return out;
  }
  private record(list: CollisionEvent[], key: string, part: string, item: string, depth: number, seen: Set<string>): void {
    seen.add(key);
    const e = this.activeContacts.get(key);
    if (e) { e.depth = Math.max(e.depth, depth); return; }
    const ev = { t: this.time, part, item, depth };
    this.activeContacts.set(key, ev); list.push(ev);
    this.log.push(`[${this.time.toFixed(2)}] contact ${part} x ${item}: ${(depth * 1000).toFixed(1)} mm`);
  }
  private lastCheck = { key: '', armKey: '' };
  private monitor(): void {
    const [x, y, th] = this.pose();
    const poseKey = `${x.toFixed(4)},${y.toFixed(4)},${th.toFixed(4)}`;
    const armKey = `${poseKey}|${[...this.h.joints.keys()].map((n) => this.joint(n).toFixed(4)).join(',')}|${this.attached.left},${this.attached.right},${this.attached.both}`;
    if (armKey === this.lastCheck.armKey) return; // nothing moved: same contacts as the last check
    const seen = new Set<string>();
    // minTop: boxes lower than this are driven over (steps / pads below a third of the wheel radius)
    const check = (pts: Array<{ p: V3; r: number; part: string }>, list: CollisionEvent[], base: boolean, minTop = -Infinity) => {
      if (!pts.length) return;
      const box = aabbOf(pts);
      for (const b of this.furniture) {
        if (b.hi[2] < minTop || !overlaps(b, box)) continue;
        const pen: Record<string, number> = {};
        for (const s of pts) { const d = s.r - this.sdObb(b, s.p); if (d > (pen[s.part] ?? 0)) pen[s.part] = d; }
        for (const [part, d] of Object.entries(pen)) {
          if (base) this.maxBasePen = Math.max(this.maxBasePen, d); else this.maxArmPen = Math.max(this.maxArmPen, d);
          if (d > COLLISION_TOL) this.record(list, `${part}|${b.item}`, part, b.item, d, seen);
        }
      }
    };
    const armPts = [...this.armSamples('left'), ...this.armSamples('right')];
    for (const n of [this.attached.left, this.attached.right, this.attached.both]) if (n) { const o = this.objects.get(n)!; for (const p of this.loadSamples(o)) armPts.push({ p, r: 0, part: `load ${n}` }); }
    check(armPts, this.collisions, false);
    // robot body against the same colliders: the Webots boundingObjects of MUROM (torso box, wheel cylinders r 0.177 m);
    // the capsule shell has no collider in Webots either
    const c = Math.cos(th), s = Math.sin(th), z0 = this.z0;
    check(this.bodySamples().map(([lx, ly, z]) => ({ p: [x + c * lx - s * ly, y + s * lx + c * ly, z0 + z] as V3, r: 0, part: 'base' })), this.baseContacts, true, 0.06);
    for (const k of [...this.activeContacts.keys()]) if (!seen.has(k)) this.activeContacts.delete(k);
    this.lastCheck = { key: poseKey, armKey };
  }
}

/**
 * Collision geometry of an arm like MURKA's test_clearance.py: capsule radii = largest xy radius of the visual mesh
 * vertices of the upper arm / forearm links, hand = wrist and palm mesh vertices thinned to a 1 cm voxel grid. Without
 * loaded meshes: the radii and the conservative hand box points of arm_kinematics (R_UPPER_ARM, R_FOREARM, handPoints).
 */
export function armGeometry(arm: Robot, k: ArmKinematics, hasWrist: boolean, assets: AssetStore | null): { rUp: number; rFore: number; wrist: V3[]; palm: V3[]; source: 'mesh' | 'points' } {
  const linkPts = (suffix: string): V3[] => {
    const link = arm.chain.links.find((l) => l.name.endsWith(suffix)); if (!link || !assets) return [];
    const out: V3[] = [];
    for (const v of link.visuals) {
      if (!v.mesh) continue; const a = assets.get(v.mesh) ?? assets.resolve(v.mesh); const m = a?.mesh; if (!m) continue;
      const sc = (v.scale ?? [1, 1, 1]).map((x) => x / 1000); const O = v.origin ? fromArray(v.origin) : null;
      for (let i = 0; i < m.positions.length; i += 3) {
        let p: V3 = [m.positions[i] * sc[0], m.positions[i + 1] * sc[1], m.positions[i + 2] * sc[2]];
        if (O) p = [O[0] * p[0] + O[4] * p[1] + O[8] * p[2] + O[12] / 1000, O[1] * p[0] + O[5] * p[1] + O[9] * p[2] + O[13] / 1000, O[2] * p[0] + O[6] * p[1] + O[10] * p[2] + O[14] / 1000];
        out.push(p);
      }
    }
    return out;
  };
  const voxel = (P: V3[], step = 0.01): V3[] => { const seen = new Set<string>(); const out: V3[] = []; for (const p of P) { const key = `${Math.floor(p[0] / step)},${Math.floor(p[1] / step)},${Math.floor(p[2] / step)}`; if (!seen.has(key)) { seen.add(key); out.push(p); } } return out; };
  const up = linkPts('upper_arm'), fore = linkPts('forearm'), wrist = hasWrist ? linkPts('wrist') : [], palm = hasWrist ? linkPts('palm') : [];
  const rmax = (P: V3[], d: number) => (P.length ? Math.max(...P.map((p) => Math.hypot(p[0], p[1]))) : d);
  const ok = up.length > 0 && fore.length > 0 && (!hasWrist || (wrist.length > 0 && palm.length > 0));
  void k;
  // fallback radii: v2 = arm_kinematics R_UPPER_ARM / R_FOREARM; classic = max xy radius of the v1 STL link meshes
  const [ru, rf] = hasWrist ? [R_UPPER_ARM, R_FOREARM] : [0.0308, 0.0263];
  return ok ? { rUp: rmax(up, ru), rFore: rmax(fore, rf), wrist: voxel(wrist), palm: voxel(palm), source: 'mesh' } : { rUp: ru, rFore: rf, wrist: [], palm: [], source: 'points' };
}

/** Horizontal basis of a two-hand grasp: lat (left -> right) and fwd = z × lat. */
export function bothBasis(tl: ArrayLike<number>, tr: ArrayLike<number>): [[number, number], [number, number]] {
  const lx = tr[0] - tl[0], ly = tr[1] - tl[1]; const n = Math.hypot(lx, ly) || 1;
  const lat: [number, number] = [lx / n, ly / n];
  return [lat, [-lat[1], lat[0]]];
}

/** Palm surface centre offset in the palm frame (arm_kinematics PALM_SURFACE), for callers. */
export const palmSurface = (m: number): V3 => [0, -m * PALM_SURFACE[0], PALM_SURFACE[1]];
