/**
 * PickProduct / PlaceProduct / CheckPlace of murom_manipulation (pick_place_server.py), on the studio simulation.
 * The robot is already parked at the target (navigation). Pick: head pan -> [carry pose] -> [stage] -> pre-grasp ->
 * grasp -> settle -> magnetic attach -> lift -> [stage] -> carry. Place: [stage] -> pre-place -> place -> settle ->
 * detach (place pose = where the product actually is in the hand) -> retreat -> [stage] -> home. Two-hand boxes:
 * stage / pre / grasp at the near vertical edges with the palms inwards, carry, and the mirrored place with a retreat
 * of the palms towards the robot. Every IK point of the v2 robot is chosen by ArmKinematics.ikPalmSelect (palm down /
 * inwards, no arm / load penetration into the torso, the basket and the furniture boxes next to the slot, clean
 * joint-linear path from the neighbouring point); the classic robot uses the tip IK and the under-board scheme.
 * Joints move linearly between points (murom_driver) with the durations move_time_long 3.0 s / move_time_short 1.5 s.
 * Cancellation follows the original: before attach the arm leaves the furniture the way it came, after attach the
 * carry is completed, a failed place keeps the product in the hand.
 */
import { ArmKinematics, V3, Box, Load, BASKET_BOX, PEN_TOL, mv3, mm3, tr3, rot, sdBox } from './kinematics';
import type { MuromSim } from './sim';
import type { RetailLayout } from './layout';
import { Task } from './tasks';

export const GRIP_CLEARANCE = 0.012, PALM_GAP = 0.003;
export const DOWN: V3 = [0, 0, -1];
export const PRE_OFFSET = 0.10, BASKET_PRE_OLD = 0.08, IK_TOL = 0.020, JOINT_TOL = 0.09, SETTLE = 0.4, NORMAL_TOL_DEG = 3.0;
export const CEIL_MARGIN = 0.06, STAGE_DY = 0.25, BOARD_T = 0.03, FOREARM_CLEAR = 0.034, FACE_Y = 0.495, BASKET_PRE = 0.15;
export const BASKET_VIA: V3 = [-0.30, 0.30, 1.25];
export const CARRY_PALM: V3 = [0.05, 0.40, 1.25];
export const REPLAN_SHIFT = 0.005, REPLAN_YAW = 0.3 * Math.PI / 180;
export const BOTH_PRE_PICK = 0.12, BOTH_PRE_PLACE = 0.08, PALLET_STAGE = 0.20, BOTH_RETREAT = 0.20;
export const BOTH_CARRY: [number, number] = [0.26, 1.50];
export const HOME_V2 = [0.3, 0.0, 0.0, -2.2], HOME_CLASSIC = [0.0, 0.0, 0.0, -1.8];
export const MOVE_TIME_LONG = 3.0, MOVE_TIME_SHORT = 1.5;

export type Arm = 'left' | 'right';
export type GoalStatus = 'succeeded' | 'aborted' | 'canceled';
export interface PickResult { status: GoalStatus; success: boolean; message: string; objectName: string; arm: string }
export interface PlaceResult { status: GoalStatus; success: boolean; message: string }
type Geo = { kind: 'new' | 'legacy' | 'classic'; pre_z: number; stage: V3 | null; obst: Box[]; obst_final: Box[]; obst_carry: Box[]; basket?: boolean; via?: boolean; legacy?: Geo; furn?: Box[]; where?: string };
type Q = number[];
type Pair = [Q, Q];

class Cancelled extends Error {}

/** Arm operations and the server state that persists between goals (held loads, basket positions, check cache). */
export class ManipServer {
  readonly kin: Record<Arm, ArmKinematics>;
  /** arm -> [product, load box in the palm frame] */
  readonly held: Partial<Record<Arm, [string, Load]>> = {};
  heldBoth: { obj?: string; dims?: [number, number]; dyaw?: number } = {};
  readonly basketXy = new Map<string, [number, number]>();
  private checkCache = new Map<string, boolean>();
  readonly log: string[] = [];
  /** Operation statistics (IK selections, joint moves) for the report. */
  stats = { moves: 0, ikSelections: 0, attaches: 0, detaches: 0 };
  tLong = MOVE_TIME_LONG; tShort = MOVE_TIME_SHORT;
  /** Two-hand carry grip points (x, z) in base_link — BOTH_CARRY of the original unless a fix lowers it. */
  bothCarry: [number, number] = [...BOTH_CARRY];

  constructor(readonly sim: MuromSim, readonly L: RetailLayout) { this.kin = sim.kin; }
  private say(m: string): void { this.log.push(`[${this.sim.time.toFixed(2)}] ${m}`); }
  get hasWrist(): boolean { return this.sim.h.hasWrist; }
  /** z of base_link in the map (v2); 0 for the classic robot like the original. */
  get z0(): number { return this.hasWrist ? this.sim.z0 : 0; }
  private cur(arm: Arm): [Q, [number, number]] { return this.sim.armState(arm); }
  private wristFor(arm: Arm, q: Q, normal: V3 = DOWN): [number, number] { return this.kin[arm].wristForNormal(q, normal); }
  private rp(arm: Arm, q: Q): [Q, [number, number]] { return [q, this.wristFor(arm, q)]; }

  // ------------------------------------------------------------- motion
  private *waitJoints(names: string[], targets: number[], duration: number, extra = 5.0): Task<boolean> {
    yield duration;
    const deadline = this.sim.time + extra; let stable = 0;
    while (this.sim.time < deadline) {
      const ok = names.every((n, i) => Math.abs(this.sim.joint(n) - targets[i]) < JOINT_TOL);
      stable = ok ? stable + 1 : 0;
      if (stable >= 2) return true;
      yield 0.1;
    }
    this.say(`Суставы не пришли в цель: ${names.map((n, i) => `${n}=${(this.sim.joint(n) - targets[i]).toFixed(3)}`).join(' ')}`);
    return false;
  }
  private wristTargets(arm: Arm, q: Q, normal: V3): [string[], number[]] { if (!this.hasWrist) return [[], []]; return [[...this.kin[arm].wristNames], [...this.wristFor(arm, q, normal)]]; }
  *moveArm(arm: Arm, q: Q, duration: number, headPan: number | null = null, normal: V3 = DOWN): Task<boolean> {
    const names = [...this.kin[arm].jointNames]; const targets = [...q];
    const [wn, wt] = this.wristTargets(arm, q, normal); names.push(...wn); targets.push(...wt);
    if (headPan !== null) { names.push('head_pan_joint'); targets.push(headPan); }
    this.sim.sendTraj(names, targets, duration); this.stats.moves++;
    return yield* this.waitJoints(names, targets, duration);
  }
  *moveBoth(ql: Q, qr: Q, duration: number, headPan: number | null = null): Task<boolean> {
    const names = [...this.kin.left.jointNames, ...this.kin.right.jointNames]; const targets = [...ql, ...qr];
    for (const [arm, q, m] of [['left', ql, 1], ['right', qr, -1]] as Array<[Arm, Q, number]>) { const [wn, wt] = this.wristTargets(arm, q, [0, -m, 0]); names.push(...wn); targets.push(...wt); }
    if (headPan !== null) { names.push('head_pan_joint'); targets.push(headPan); }
    this.sim.sendTraj(names, targets, duration); this.stats.moves++;
    return yield* this.waitJoints(names, targets, duration);
  }
  private *go(arm: Arm | 'both', q: Q | Pair, duration: number, headPan: number | null = null): Task<boolean> {
    if (arm === 'both') { const [l, r] = q as Pair; return yield* this.moveBoth(l, r, duration, headPan); }
    return yield* this.moveArm(arm, q as Q, duration, headPan);
  }
  private *step(cancel: () => boolean, arm: Arm | 'both', reached: Array<Q | Pair>, q: Q | Pair, duration: number, headPan: number | null = null): Task<boolean> {
    if (cancel()) throw new Cancelled();
    reached.push(q);
    return yield* this.go(arm, q, duration, headPan);
  }
  private *retreat(arm: Arm | 'both', back: Array<Q | Pair>, final: Q | Pair | null = null): Task<void> {
    for (const q of back) yield* this.go(arm, q, this.tShort);
    if (final) yield* this.go(arm, final, this.tLong, 0.0);
  }
  homePose(): Q { return [...(this.hasWrist ? HOME_V2 : HOME_CLASSIC)]; }
  private home(arm: Arm | 'both'): Q | Pair { return arm === 'both' ? [this.homePose(), this.homePose()] : this.homePose(); }

  // ------------------------------------------------------------- IK helpers
  ikOrNone(arm: Arm, target: V3, o: { topDown?: boolean; ref?: [Q, ArrayLike<number>] | null; normal?: V3; obstacles?: Box[]; paths?: Array<[Q, ArrayLike<number>, (Load | null)?]> | null; load?: Load | null; point?: ArrayLike<number> | null; quiet?: boolean } = {}): Q | null {
    const normal = o.normal ?? DOWN;
    if (o.topDown && this.hasWrist) {
      const ref = o.ref ?? this.cur(arm);
      this.stats.ikSelections++;
      const sel = this.kin[arm].ikPalmSelect(target, normal, { ref, paths: o.paths, tol: IK_TOL, maxNormalDeg: NORMAL_TOL_DEG, obstacles: o.obstacles ?? [], load: o.load ?? null, point: o.point ?? null });
      if (!sel) { if (!o.quiet) this.say(`IK (ладонь) не сошлась для ${arm}: цель ${target.map((v) => v.toFixed(3)).join(', ')}`); return null; }
      if (sel.pathPen > 0 && !o.quiet) this.say(`IK (ладонь) для ${arm}: путь с наименьшим заходом ${(sel.pathPen * 1000).toFixed(0)} мм`);
      return sel.q;
    }
    const [q, err] = this.kin[arm].ik(target, o.ref ? [...o.ref[0]] : null);
    if (err > IK_TOL) { if (!o.quiet) this.say(`IK не сошлась для ${arm}: err ${(err * 1000).toFixed(0)} мм`); return null; }
    return q;
  }
  private ikBothPath(points: Array<[V3, V3]>, quiet = false): Pair[] | null {
    const out: Pair[] = [];
    const ref: Record<Arm, [Q, ArrayLike<number>]> = { left: this.cur('left'), right: this.cur('right') };
    for (const [tl, tr] of points) {
      const qs: Q[] = [];
      for (const [arm, t0, m] of [['left', tl, 1], ['right', tr, -1]] as Array<[Arm, V3, number]>) {
        const normal: V3 = [0, -m, 0];
        const t: V3 = this.hasWrist ? [t0[0] + 0.05, t0[1] - m * (0.005 - PALM_GAP), t0[2]] : t0;
        const q = this.ikOrNone(arm, t, { topDown: true, ref: ref[arm], normal, quiet });
        if (!q) return null;
        ref[arm] = [q, this.wristFor(arm, q, normal)];
        qs.push(q);
      }
      out.push([qs[0], qs[1]]);
    }
    return out;
  }
  private localDims(size: V3, yaw: number): [number, number] { const c = Math.abs(Math.cos(yaw)), s = Math.abs(Math.sin(yaw)); return c >= s ? [size[0], size[1]] : [size[1], size[0]]; }
  private bothTargets(lx: number, ly: number, topZ: number, dims: [number, number]): [V3, V3] {
    const gx = lx - dims[0] / 2, gy = dims[1] / 2 + 0.005, gz = topZ - 0.10;
    return [[gx, ly + gy, gz], [gx, ly - gy, gz]];
  }
  /** Load box in the palm frame (centre, half sizes, rotation) — from the grasp pose, else a generic box under the palm. */
  loadOf(arm: Arm, size: V3, qGrasp?: Q | null, center?: V3, yaw = 0, wrist?: [number, number]): Load {
    const half: V3 = [size[0] / 2, size[1] / 2, size[2] / 2];
    if (qGrasp) {
      const k = this.kin[arm];
      const [rp, org] = k.palmFrame(qGrasp, wrist ?? this.wristFor(arm, qGrasp));
      const rt = tr3(rp);
      return [mv3(rt, [center![0] - org[0], center![1] - org[1], center![2] - org[2]]), half, mm3(rt, rot('z', yaw))];
    }
    const m = arm === 'left' ? 1 : -1; const a = Math.max(size[0], size[1]) / 2;
    return [[0, -m * (0.016 + PALM_GAP + size[2] / 2), -0.054], [a, half[2], a]];
  }
  private pickTargets(name: string, size: V3): [number, number, number] {
    const o = this.sim.objects.get(name)!;
    let [lx, ly] = this.sim.toLocal(o.pos);
    const b = this.L.basket[o.location];
    if (b) [lx, ly] = this.basketXy.get(name) ?? [b.dx, b.dy];
    const graspZ = o.pos[2] - this.z0 + size[2] / 2 + (this.hasWrist ? PALM_GAP : GRIP_CLEARANCE);
    return [lx, ly, graspZ];
  }
  private legacyGeom(z: number, location: string, place: boolean): Geo {
    const pre = place && location.startsWith('basket') ? BASKET_PRE_OLD : PRE_OFFSET;
    return { kind: 'legacy', pre_z: z + pre, stage: null, obst: [], obst_final: [], obst_carry: [] };
  }
  approachGeom(arm: Arm, lx: number, ly: number, z: number, location: string, place = false): Geo {
    const legacy = this.legacyGeom(z, location, place);
    if (!this.hasWrist) return this.classicGeom(arm, lx, ly, z, location, legacy);
    const m = Math.sign(ly) || 1;
    const keep: Box = [[-1.0, Math.min(m * FACE_Y, m * 1.5), 0.0], [1.0, Math.max(m * FACE_Y, m * 1.5), 2.0]];
    const where = location.split('_').find((p) => p[0] === 'L' && /^\d+$/.test(p.slice(1))) ?? location;
    if (location.startsWith('basket')) {
      const basket: Box = [[...BASKET_BOX[0]], [...BASKET_BOX[1]]];
      return { kind: 'new', pre_z: z + BASKET_PRE, stage: [BASKET_VIA[0], m * BASKET_VIA[1], BASKET_VIA[2]], basket: true, via: !place, obst: [keep, basket], obst_final: [keep], obst_carry: [keep, basket], legacy, furn: [keep], where };
    }
    const slot = this.L.slots[location];
    const furn: Box[] = []; let preZ = z + PRE_OFFSET; let stage: V3 | null = null;
    const bz = this.z0;
    if (slot?.depthIn !== undefined) {
      const face = ly - m * slot.depthIn;
      const [y0, y1] = [face, face + m * 0.8].sort((a, b) => a - b);
      furn.push([[lx - 1.0, y0, 0.0], [lx + 1.0, y1, slot.pose.z - bz]]);
      if (slot.ceiling !== undefined) furn.push([[lx - 1.0, y0, slot.ceiling - bz], [lx + 1.0, y1, slot.ceiling - bz + BOARD_T]]);
    }
    if (slot?.ceiling !== undefined) { preZ = Math.max(z, Math.min(preZ, slot.ceiling - bz - CEIL_MARGIN)); stage = [lx, ly - m * STAGE_DY, preZ]; }
    return { kind: 'new', pre_z: preZ, stage, obst: furn, obst_final: furn, obst_carry: [...furn, keep], legacy, furn, where };
  }
  private classicGeom(arm: Arm, lx: number, ly: number, z: number, location: string, legacy: Geo): Geo {
    const slot = this.L.slots[location];
    if (!slot || slot.ceiling === undefined || slot.depthIn === undefined) return legacy;
    const m = Math.sign(ly) || 1;
    const face = ly - m * slot.depthIn;
    const [y0, y1] = [face, face + m * 0.8].sort((a, b) => a - b);
    const ceil = slot.ceiling - this.z0;
    const board: Box = [[lx - 1.0, y0, ceil], [lx + 1.0, y1, ceil + BOARD_T]];
    const k = this.kin[arm];
    const clearance = (q: Q) => { const [, pElb, tip] = k.chain(q); let mn = Infinity; for (let i = 0; i < 15; i++) { const t = i / 14; mn = Math.min(mn, sdBox([pElb[0] + t * (tip[0] - pElb[0]), pElb[1] + t * (tip[1] - pElb[1]), pElb[2] + t * (tip[2] - pElb[2])], board[0], board[1])); } return mn - FOREARM_CLEAR; };
    let preZ: number | null = null;
    for (let zz = Math.min(z + PRE_OFFSET, ceil); zz >= z - 1e-6; zz -= 0.005) {
      const [q, err] = k.ik([lx, ly, zz]);
      if (err <= IK_TOL && clearance(q) >= 0) { preZ = zz; break; }
    }
    if (preZ === null) { preZ = z; this.say(`${location}: предплечье у цели не проходит под доской — pre на высоте цели`); }
    return { kind: 'classic', pre_z: preZ, stage: [lx, ly - m * STAGE_DY, preZ], obst: [], obst_final: [], obst_carry: [], legacy, furn: [board], where: location };
  }
  carryPose(arm: Arm, ref: [Q, ArrayLike<number>] | null = null, obstacles: Box[] = [], load: Load | null = null): Q {
    const m = arm === 'left' ? 1 : -1;
    if (this.hasWrist) { const q = this.ikOrNone(arm, [CARRY_PALM[0], m * CARRY_PALM[1], CARRY_PALM[2]], { topDown: true, ref, obstacles, load }); if (q) return q; }
    return [0.0, m * 0.35, 0.0, -1.2];
  }
  private planPalm(arm: Arm, geo: Geo, final: V3, load: Load | null, loadOf: ((q: Q) => Load) | null): [Q | null, Q | null, Q, Q] | null {
    const stage = geo.stage, obst = geo.obst;
    const qVia = geo.via ? this.carryPose(arm, null, geo.obst_carry) : null;
    const cur = qVia ? this.rp(arm, qVia) : this.cur(arm);
    const qPre = this.ikOrNone(arm, [final[0], final[1], geo.pre_z], { topDown: true, ref: cur, obstacles: obst, load, paths: stage ? [] : null });
    if (!qPre) return null;
    const qFinal = this.ikOrNone(arm, final, { topDown: true, ref: this.rp(arm, qPre), obstacles: geo.obst_final, load });
    if (!qFinal) return null;
    let qStage: Q | null = null;
    if (stage) {
      const paths: Array<[Q, ArrayLike<number>, (Load | null)?]> = [[qPre, this.wristFor(arm, qPre), loadOf ? loadOf(qFinal) : load], cur];
      if (!geo.basket) paths.push([this.homePose(), this.wristFor(arm, this.homePose()), null]);
      qStage = this.ikOrNone(arm, stage, { topDown: true, ref: this.rp(arm, qPre), obstacles: obst, load, paths });
      if (!qStage) return null;
    }
    return [qVia, qStage, qPre, qFinal];
  }
  private planPen(arm: Arm, plan: Array<Q | null>, boxes: Box[], load: Load | null, loadOf: ((q: Q) => Load) | null): number {
    if (!boxes.length) return 0;
    const k = this.kin[arm];
    const pts: Array<[Q, ArrayLike<number>]> = [this.cur(arm), ...plan.filter((q): q is Q => !!q).map((q) => this.rp(arm, q))];
    const legs: Array<[[Q, ArrayLike<number>], [Q, ArrayLike<number>], Load | null]> = [];
    for (let i = 0; i + 1 < pts.length; i++) legs.push([pts[i], pts[i + 1], load]);
    if (loadOf) { const held = loadOf(plan[3]!); legs.push([pts[pts.length - 1], pts[pts.length - 2], held], [pts[pts.length - 2], this.rp(arm, this.carryPose(arm, pts[pts.length - 2] as [Q, ArrayLike<number>], [], held)), held]); }
    else legs.push([pts[pts.length - 1], pts[pts.length - 2], null], [pts[pts.length - 2], this.rp(arm, this.homePose()), null]);
    let out = 0;
    for (const [a, b, ld] of legs) out = Math.max(out, k.bodyPen(a[0], a[1], boxes, ld), k.bodyPen(b[0], b[1], boxes, ld), k.pathPen(a[0], a[1], b[0], b[1], boxes, ld));
    return out;
  }
  /** Returns [qVia, qStage, qPre, qFinal, geo], null (no IK) or a refusal message. */
  private planApproach(arm: Arm, geo: Geo, final: V3, load: Load | null = null, loadOf: ((q: Q) => Load) | null = null): [Q | null, Q | null, Q, Q, Geo] | null | string {
    const pre: V3 = [final[0], final[1], geo.pre_z];
    if (!this.hasWrist) {
      const qPre = this.ikOrNone(arm, pre), qFinal = this.ikOrNone(arm, final);
      if (!qPre || !qFinal) return null;
      let qStage: Q | null = null;
      if (geo.stage) { qStage = this.ikOrNone(arm, geo.stage, { ref: [qPre, [0, 0]] }); if (!qStage) this.say('ступень недостижима — подход без неё'); }
      return [null, qStage, qPre, qFinal, geo];
    }
    let plan = this.planPalm(arm, geo, final, load, loadOf);
    if (!plan && geo.legacy) {
      const furn = geo.furn ?? [], where = geo.where ?? '';
      geo = geo.legacy;
      plan = this.planPalm(arm, geo, final, load, loadOf);
      if (!plan) return null;
      const pen = this.planPen(arm, plan, furn, load, loadOf);
      if (pen > PEN_TOL) return `Цель вне досягаемости без касания мебели (${where})`;
      this.say(`Подход рукой ${arm}: откат на прежнюю схему (pre прямо над целью)`);
    }
    return plan ? [...plan, geo] : null;
  }

  // -------------------------------------------------------------------- PICK
  *pick(objName: string, armReq: string, cancel: () => boolean): Task<PickResult> {
    const sim = this.sim;
    const res = (status: GoalStatus, success: boolean, message: string, name = '', arm = ''): PickResult => ({ status, success, message, objectName: name, arm });
    if (cancel()) return res('canceled', false, 'отменено до начала захвата');
    const obj = sim.objects.get(objName);
    if (!obj) return res('aborted', false, `Объект ${objName} не найден`);
    const cat = this.L.catalog[obj.sku]; const size = obj.size;
    if (cat?.twoHand || armReq === 'both') return yield* this.pickBoth(objName, size, cancel);
    let arm: Arm | null = armReq === 'left' || armReq === 'right' ? armReq : null;
    let lastMsg = ''; const reached: Q[] = [];
    let qLift: Q | null = null, qStage: Q | null = null, geo: Geo | null = null;
    try {
      for (let attempt = 0; attempt < 2; attempt++) {
        if (reached.length) { yield* this.retreat(arm!, reached.slice(0, -1).reverse()); reached.length = 0; }
        if (attempt && cancel()) throw new Cancelled();
        let [lx, ly, graspZ] = this.pickTargets(objName, size);
        if (!arm) arm = ly > 0 ? 'left' : 'right';
        geo = this.approachGeom(arm, lx, ly, graspZ, obj.location);
        this.say(`PICK ${objName} (${obj.sku}) рукой ${arm} (поп. ${attempt + 1}): локально (${lx.toFixed(3)}, ${ly.toFixed(3)}, ${graspZ.toFixed(3)})`);
        const center: V3 = [lx, ly, obj.pos[2] - this.z0]; const yaw = obj.yaw - sim.pose()[2];
        const a = arm;
        const plan = this.planApproach(arm, geo, [lx, ly, graspZ], null, (q) => this.loadOf(a, size, q, center, yaw));
        if (!Array.isArray(plan)) { lastMsg = plan || 'Цель вне досягаемости (IK)'; continue; }
        const [qVia, qSt, qPre, qGrasp0, g2] = plan; geo = g2; qStage = qSt;
        let qGrasp = qGrasp0; qLift = qPre;
        let pose0 = sim.pose();
        const head = arm === 'left' ? 0.9 : -0.9;
        if (qVia && !(yield* this.step(cancel, arm, reached, qVia, this.tShort, head))) { lastMsg = 'не вышли в позу переноски'; continue; }
        if (qStage && !(yield* this.step(cancel, arm, reached, qStage, this.tLong, head))) { lastMsg = 'не вышли на ступень'; continue; }
        if (!(yield* this.step(cancel, arm, reached, qPre, qStage ? this.tShort : this.tLong, head))) { lastMsg = 'не вышли в pre-grasp'; continue; }
        if (this.hasWrist && !obj.location.startsWith('basket') && this.moved(pose0)) {
          [lx, ly, graspZ] = this.pickTargets(objName, size);
          const old = geo.kind === 'legacy';
          let g = this.approachGeom(arm, lx, ly, graspZ, obj.location); if (old) g = g.legacy!; geo = g;
          const c2: V3 = [lx, ly, obj.pos[2] - this.z0]; const y2 = obj.yaw - sim.pose()[2];
          const qg = this.ikOrNone(arm, [lx, ly, graspZ], { topDown: true, ref: this.rp(arm, qPre), obstacles: geo.obst_final });
          qLift = qg ? this.ikOrNone(arm, [lx, ly, geo.pre_z], { topDown: true, ref: this.rp(arm, qg), obstacles: geo.obst, load: this.loadOf(arm, size, qg, c2, y2) }) : null;
          if (!qg || !qLift) { lastMsg = 'Цель вне досягаемости после сдвига базы (IK)'; continue; }
          qGrasp = qg; pose0 = sim.pose();
          this.say(`PICK ${objName}: цель пересчитана в pre (база сдвинулась)`);
        }
        if (!(yield* this.step(cancel, arm, reached, qGrasp, this.tShort))) { lastMsg = 'не вышли в grasp'; continue; }
        yield SETTLE;
        if (cancel()) throw new Cancelled();
        const r = sim.attach(objName, arm); this.stats.attaches++;
        this.say(`attach ${objName}: ${r.message}`);
        if (r.success) {
          lastMsg = '';
          if (this.hasWrist) {
            const [cx, cy] = this.pickTargets(objName, size);
            const [qAct, wAct] = this.cur(arm);
            this.held[arm] = [objName, this.loadOf(arm, size, qAct, [cx, cy, obj.pos[2] - this.z0], obj.yaw - sim.pose()[2], wAct)];
          }
          this.basketXy.delete(objName);
          break;
        }
        lastMsg = `attach: ${r.message}`;
      }
    } catch (e) {
      if (!(e instanceof Cancelled)) throw e;
      return yield* this.failPick(cancel, arm ?? 'left', `отменено до захвата ${objName}`, reached);
    }
    if (lastMsg) return yield* this.failPick(cancel, arm ?? 'left', lastMsg, reached);
    yield* this.moveArm(arm!, qLift!, this.tShort);
    let last = qLift!;
    if (qStage) { yield* this.moveArm(arm!, qStage, this.tShort); last = qStage; }
    const held = this.held[arm!];
    const qCarry = this.carryPose(arm!, this.rp(arm!, last), geo!.obst_carry, held ? held[1] : null);
    yield* this.moveArm(arm!, qCarry, this.tLong, 0.0);
    if (cancel()) return res('canceled', false, `отменено после захвата: ${objName} в руке ${arm}`, objName, arm!);
    return res('succeeded', true, 'OK', objName, arm!);
  }
  private moved(pose0: [number, number, number]): boolean {
    const [x, y, th] = this.sim.pose(); const d = Math.atan2(Math.sin(th - pose0[2]), Math.cos(th - pose0[2]));
    return Math.hypot(x - pose0[0], y - pose0[1]) > REPLAN_SHIFT || Math.abs(d) > REPLAN_YAW;
  }
  private *failPick(cancel: () => boolean, arm: Arm | 'both', msg: string, reached: Array<Q | Pair>): Task<PickResult> {
    this.say(`PICK ${cancel() ? 'отменён' : 'fail'}: ${msg}`);
    yield* this.retreat(arm, reached.slice(0, -1).reverse(), this.home(arm));
    return { status: cancel() ? 'canceled' : 'aborted', success: false, message: cancel() && !msg.startsWith('отменено') ? `отменено (${msg})` : msg, objectName: '', arm: '' };
  }
  private *pickBoth(objName: string, size: V3, cancel: () => boolean): Task<PickResult> {
    const sim = this.sim; let lastMsg = ''; const reached: Pair[] = [];
    let pre: Pair | null = null; let dims: [number, number] = [size[0], size[1]]; let dyaw = 0;
    try {
      for (let attempt = 0; attempt < 2; attempt++) {
        if (reached.length) { yield* this.retreat('both', reached.slice(0, -1).reverse()); reached.length = 0; }
        if (attempt && cancel()) throw new Cancelled();
        const obj = sim.objects.get(objName)!;
        dyaw = obj.yaw - sim.pose()[2];
        dims = this.localDims(size, dyaw);
        const [lx, ly] = sim.toLocal(obj.pos);
        const topZ = obj.pos[2] - this.z0 + size[2] / 2;
        const [tl, tr] = this.bothTargets(lx, ly, topZ, dims);
        const preT: [V3, V3] = [[tl[0], tl[1], tl[2] + BOTH_PRE_PICK], [tr[0], tr[1], tr[2] + BOTH_PRE_PICK]];
        const stage: [V3, V3] = [[preT[0][0] - PALLET_STAGE, preT[0][1], preT[0][2]], [preT[1][0] - PALLET_STAGE, preT[1][1], preT[1][2]]];
        this.say(`PICK ${objName} ДВУМЯ руками (поп. ${attempt + 1}): центр локально (${lx.toFixed(3)}, ${ly.toFixed(3)})`);
        const path = this.ikBothPath([stage, preT, [tl, tr]]);
        if (!path) { lastMsg = 'Цель вне досягаемости (IK, both)'; continue; }
        const [iksStage, iksPre, iksGrasp] = path; pre = iksPre;
        if (!(yield* this.step(cancel, 'both', reached, iksStage, this.tLong, 0.0))) { lastMsg = 'не вышли на ступень (both)'; continue; }
        if (!(yield* this.step(cancel, 'both', reached, iksPre, this.tShort))) { lastMsg = 'не вышли в pre-grasp (both)'; continue; }
        if (!(yield* this.step(cancel, 'both', reached, iksGrasp, this.tShort))) { lastMsg = 'не вышли в grasp (both)'; continue; }
        yield SETTLE;
        if (cancel()) throw new Cancelled();
        const r = sim.attach(objName, 'both'); this.stats.attaches++;
        this.say(`attach ${objName}: ${r.message}`);
        if (r.success) { lastMsg = ''; break; }
        lastMsg = `attach: ${r.message}`;
      }
    } catch (e) {
      if (!(e instanceof Cancelled)) throw e;
      return yield* this.failPick(cancel, 'both', `отменено до захвата ${objName}`, reached);
    }
    if (lastMsg) return yield* this.failPick(cancel, 'both', lastMsg, reached);
    this.heldBoth = { obj: objName, dims, dyaw };
    yield* this.moveBoth(pre![0], pre![1], this.tShort);
    const gy = dims[1] / 2 + 0.005;
    const carry = this.ikBothPath([[[this.bothCarry[0], gy, this.bothCarry[1]], [this.bothCarry[0], -gy, this.bothCarry[1]]]]);
    if (carry) yield* this.moveBoth(carry[0][0], carry[0][1], this.tLong);
    if (cancel()) return { status: 'canceled', success: false, message: `отменено после захвата: ${objName} в обеих руках`, objectName: objName, arm: 'both' };
    return { status: 'succeeded', success: true, message: 'OK (both)', objectName: objName, arm: 'both' };
  }

  // ------------------------------------------------------------------- PLACE
  *place(objName: string, locationId0: string, armReq: string, cancel: () => boolean): Task<PlaceResult> {
    const sim = this.sim;
    const obj = sim.objects.get(objName);
    if (!obj) return { status: 'aborted', success: false, message: `Объект ${objName} не найден` };
    const size = obj.size;
    if (armReq === 'both' || obj.location === 'carried_both') return yield* this.placeBoth(objName, locationId0, size, cancel);
    const arm: Arm = armReq === 'left' || armReq === 'right' ? armReq : obj.location === 'carried_left' ? 'left' : 'right';
    if (cancel()) return { status: 'canceled', success: false, message: `отменено до начала укладки; ${objName} остаётся в руке ${arm}` };
    let locationId = locationId0; let lx: number, ly: number, floor: number;
    if (locationId === 'basket') {
      const slot = this.freeBasketSlot(arm);
      if (!slot) return { status: 'aborted', success: false, message: `Нет свободного слота корзины для руки ${arm}` };
      [lx, ly, floor] = [slot.dx, slot.dy, slot.dz]; locationId = slot.id;
    } else {
      const s = this.L.slots[locationId];
      if (!s) return { status: 'aborted', success: false, message: `Слот ${locationId} не найден` };
      [lx, ly] = sim.toLocal([s.pose.x, s.pose.y, 0]);
      floor = s.pose.z - this.z0;
    }
    const tip: V3 = [lx, ly, floor + size[2] + (this.hasWrist ? PALM_GAP : GRIP_CLEARANCE)];
    const restZ = floor + size[2] / 2 + 0.002;
    let geo = this.approachGeom(arm, tip[0], tip[1], tip[2], locationId, true);
    const h = this.held[arm];
    const heldLoad = this.hasWrist && h && h[0] === objName ? h[1] : null;
    const load = heldLoad ?? this.loadOf(arm, size);
    const plan = this.planApproach(arm, geo, tip, load);
    if (!Array.isArray(plan)) return { status: 'aborted', success: false, message: plan || 'Слот вне досягаемости (IK)' };
    const [, qStage, qPre, qPlace0, g2] = plan; geo = g2;
    let qPlace = qPlace0;
    if (heldLoad) {
      const k = this.kin[arm];
      const qItem = this.ikOrNone(arm, [lx, ly, restZ], { topDown: true, point: heldLoad[0], load, ref: this.rp(arm, qPlace), paths: [[qPre, k.wristForNormal(qPre, DOWN)]], obstacles: geo.obst_final });
      if (qItem) qPlace = qItem; else this.say(`PLACE ${objName}: товар в слот по позе в ладони — нет решения IK, цель ладони — номинал`);
    }
    this.say(`PLACE ${objName} -> ${locationId} рукой ${arm}`);
    const qStart = this.cur(arm)[0]; const reached: Q[] = [];
    try {
      if (qStage && !(yield* this.step(cancel, arm, reached, qStage, this.tLong))) return yield* this.failPlace(cancel, arm, 'не вышли на ступень', reached, qStart, objName);
      if (!(yield* this.step(cancel, arm, reached, qPre, qStage ? this.tShort : this.tLong))) return yield* this.failPlace(cancel, arm, 'не вышли в pre-place', reached, qStart, objName);
      if (!(yield* this.step(cancel, arm, reached, qPlace, this.tShort))) return yield* this.failPlace(cancel, arm, 'не вышли в place', reached, qStart, objName);
      yield SETTLE;
      if (cancel()) throw new Cancelled();
    } catch (e) {
      if (!(e instanceof Cancelled)) throw e;
      return yield* this.failPlace(cancel, arm, 'отменено до укладки', reached, qStart, objName);
    }
    let ix = lx, iy = ly;
    if (heldLoad) { const [qa, wa] = this.cur(arm); const c = this.kin[arm].palmPoint(qa, wa, heldLoad[0]); ix = c[0]; iy = c[1]; }
    const m = sim.toMap([ix, iy, 0]);
    const r = sim.detach(objName, [m[0], m[1], restZ + this.z0], locationId); this.stats.detaches++;
    this.say(`detach ${objName}: ${r.message}`);
    if (!r.success) return yield* this.failPlace(cancel, arm, `detach: ${r.message}`, reached, qStart, objName);
    delete this.held[arm];
    if (locationId.startsWith('basket')) this.basketXy.set(objName, [ix, iy]);
    yield* this.moveArm(arm, qPre, this.tShort);
    if (qStage) {
      yield* this.moveArm(arm, qStage, this.tShort);
      if (locationId.startsWith('basket')) yield* this.moveArm(arm, this.carryPose(arm, this.rp(arm, qStage), geo.obst_carry), this.tShort);
    }
    yield* this.moveArm(arm, this.homePose(), this.tLong);
    return { status: 'succeeded', success: true, message: `OK (${locationId})` };
  }
  private *failPlace(cancel: () => boolean, arm: Arm | 'both', msg0: string, reached: Array<Q | Pair>, start: Q | Pair | null, held = ''): Task<PlaceResult> {
    const msg = held ? `${msg0}; ${held} остаётся в ${arm === 'both' ? 'обеих руках' : `руке ${arm}`}` : msg0;
    this.say(`PLACE ${cancel() ? 'отменена' : 'fail'}: ${msg}`);
    if (reached.length) {
      const back = reached.slice(0, -1).reverse();
      if (held) yield* this.retreat(arm, back, start);
      else yield* this.retreat(arm, [...back, ...(start ? [start] : [])], this.home(arm));
    }
    return { status: cancel() ? 'canceled' : 'aborted', success: false, message: cancel() && !msg.startsWith('отменено') ? `отменено (${msg})` : msg };
  }
  private placeBothPoints(p: { x: number; y: number; z: number }, size: V3, dims: [number, number], pose?: [number, number, number]): Array<[V3, V3]> {
    const [lx, ly] = this.sim.toLocal([p.x, p.y, 0], pose);
    const [tl, tr] = this.bothTargets(lx, ly, p.z - this.z0 + size[2], dims);
    const pre: [V3, V3] = [[tl[0], tl[1], tl[2] + BOTH_PRE_PLACE], [tr[0], tr[1], tr[2] + BOTH_PRE_PLACE]];
    const back: [V3, V3] = [[pre[0][0] - BOTH_RETREAT, pre[0][1], pre[0][2]], [pre[1][0] - BOTH_RETREAT, pre[1][1], pre[1][2]]];
    return [pre, [tl, tr], back];
  }
  private *placeBoth(objName: string, locationId: string, size: V3, cancel: () => boolean): Task<PlaceResult> {
    const s = this.L.slots[locationId];
    if (!s) return { status: 'aborted', success: false, message: `Слот ${locationId} не найден` };
    if (cancel()) return { status: 'canceled', success: false, message: `отменено до начала укладки; ${objName} остаётся в обеих руках` };
    const held = this.heldBoth.obj === objName ? this.heldBoth : {};
    const dyaw = held.dyaw ?? -this.sim.pose()[2];
    const dims = held.dims ?? this.localDims(size, dyaw);
    const path = this.ikBothPath(this.placeBothPoints(s.pose, size, dims));
    if (!path) return { status: 'aborted', success: false, message: 'Слот вне досягаемости (IK, both)' };
    const [iksPre, iksPlace, iksBack] = path;
    this.say(`PLACE ${objName} -> ${locationId} ДВУМЯ руками`);
    const qStart: Pair = [this.cur('left')[0], this.cur('right')[0]]; const reached: Pair[] = [];
    try {
      if (!(yield* this.step(cancel, 'both', reached, iksPre, this.tLong))) return yield* this.failPlace(cancel, 'both', 'не вышли в pre-place (both)', reached, qStart, objName);
      if (!(yield* this.step(cancel, 'both', reached, iksPlace, this.tShort))) return yield* this.failPlace(cancel, 'both', 'не вышли в place (both)', reached, qStart, objName);
      yield SETTLE;
      if (cancel()) throw new Cancelled();
    } catch (e) {
      if (!(e instanceof Cancelled)) throw e;
      return yield* this.failPlace(cancel, 'both', 'отменено до укладки', reached, qStart, objName);
    }
    const r = this.sim.detach(objName, [s.pose.x, s.pose.y, s.pose.z + size[2] / 2 + 0.002], locationId); this.stats.detaches++;
    this.say(`detach ${objName}: ${r.message}`);
    if (!r.success) return yield* this.failPlace(cancel, 'both', `detach: ${r.message}`, reached, qStart, objName);
    this.heldBoth = {};
    yield* this.moveBoth(iksPre[0], iksPre[1], this.tShort);
    yield* this.moveBoth(iksBack[0], iksBack[1], this.tShort);
    yield* this.moveBoth(this.homePose(), this.homePose(), this.tLong);
    return { status: 'succeeded', success: true, message: `OK (${locationId}, both)` };
  }

  /** CheckPlace: is a two-hand place into the slot reachable from its approach pose (no motion). */
  checkPlace(objName: string, locationId: string, arm: string): [boolean, string] {
    const s = this.L.slots[locationId]; const o = this.sim.objects.get(objName);
    if (!s || !o) return [false, `нет слота ${locationId} или товара ${objName}`];
    const cat = this.L.catalog[o.sku];
    if (arm !== 'both' && !cat?.twoHand) return [true, 'одноручная укладка не проверяется'];
    if (!s.approach) return [false, `у слота ${locationId} нет подъезда`];
    const size = o.size, appr = s.approach;
    let dims: [number, number];
    if (this.heldBoth.obj === objName && this.heldBoth.dims) dims = this.heldBoth.dims;
    else { const src = this.L.slots[o.location]?.approach ?? appr; dims = this.localDims(size, o.yaw - src.yaw); }
    const points = this.placeBothPoints(s.pose, size, dims, [appr.x, appr.y, appr.yaw]);
    const key = `${this.hasWrist}|${points.flat().flat().map((v) => v.toFixed(3)).join(',')}`;
    if (!this.checkCache.has(key)) this.checkCache.set(key, this.ikBothPath(points, true) !== null);
    const ok = this.checkCache.get(key)!;
    return [ok, ok ? 'достижима' : 'двуручная укладка вне досягаемости IK'];
  }
  freeBasketSlot(arm: Arm): { id: string; dx: number; dy: number; dz: number } | null {
    const occupied = new Set([...this.sim.objects.values()].map((o) => o.location).filter((l) => l.startsWith('basket_')));
    for (const b of Object.values(this.L.basket)) if (b.arm === arm && !occupied.has(b.id)) return b;
    return null;
  }
}
