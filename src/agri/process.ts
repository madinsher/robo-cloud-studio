/**
 * Agricultural process models executed by the FleetManager while robots work their rows (registered at import, so
 * every FleetManager — including the one the app creates for a loaded station — runs them):
 * - harvest / thin: arm-side work-path orientation, base stops placed so the fruit are within the arm's real reach,
 *   picking time per fruit (the base stands still), fruit removed one by one at their pick time, bin capacity with
 *   unload trips; thinning removes unripe fruit above the target crop load and books no yield;
 * - spray / irrigate: dose per metre from the canopy volume (or litres per plant), tank / water capacity with refill
 *   trips, plants record the applied dose, spraying cures diseased plants;
 * - mow / weed: swath coverage (area worked, coverage of the field's inter-rows);
 * - prune / pollinate: workload per plant (time from canopy volume), plants stamped as done;
 * - scout: observations of plant health (noisy) stored on the mission, disease detection, optional follow-up tasks;
 * - transport: load at the location, deliver to `meta.dropoff`, unload.
 * Also provides the orchard alleys as traffic segments.
 */
import { Station, ItemType, Camera, Item } from '../core/items/item';
import { MobileRobot, ZoneItem } from '../mobile/items';
import { FleetManager, FleetTask, registerTaskProcess, registerSegmentProvider, TrafficSegment, WorkDirective, fleetPlugins } from '../fleet/fleet';
import { FieldItem, CropRow, MissionItem } from './items';
import { fruitWorldPositions, plantPosition } from './orchard';
import { armOf, armMount, armReach, armLift, setArmLift } from './harvest';
import { plantWorld } from './collision_world';
import { actionEnergy } from './equipment';
import { detectFruit } from './vision';
import { Robot } from '../core/items/robot';
import { poseFromZ, multiply, invert, getPos, transl } from '../core/math/pose';
import { ikAll } from '../core/kinematics/ik';
import { robotCollisionChecker } from '../core/collision/collision';

// -- helpers -------------------------------------------------------------------------------------------------------

function rowOf(fm: FleetManager, task: FleetTask): CropRow | null {
  const r = task.meta?.rowId ? fm.station.findById(task.meta.rowId) : null;
  return r instanceof CropRow ? r : null;
}
function missionOf(fm: FleetManager, task: FleetTask): MissionItem | null {
  const m = task.meta?.missionId ? fm.station.findById(task.meta.missionId) : null;
  return m instanceof MissionItem ? m : null;
}
/** Row direction in the station frame. */
function rowDirWorld(row: CropRow): [number, number] {
  const [dx, dy] = row.direction();
  const a = row.parent ? row.parent.poseAbs() : null;
  return a ? [a[0] * dx + a[4] * dy, a[1] * dx + a[5] * dy] : [dx, dy];
}
function workDir(task: FleetTask): [number, number] {
  const w = task.workPath!;
  const dx = w[w.length - 1][0] - w[0][0], dy = w[w.length - 1][1] - w[0][1], l = Math.hypot(dx, dy) || 1;
  return [dx / l, dy / l];
}
function settingsOf(fm: FleetManager, task: FleetTask): Record<string, any> { return missionOf(fm, task)?.settings ?? task.meta ?? {}; }
function rt(task: FleetTask): Record<string, any> { return (task.meta._proc ??= {}); }
/** Unloading / refill location: zone of the kind, else charging zone, else the robot's home. */
function serviceLocation(fm: FleetManager, robot: MobileRobot, kinds: string[]): [number, number] | null {
  const zones = fm.station.itemsOfType<ZoneItem>(ItemType.ZONE);
  for (const k of kinds) {
    const z = zones.find((q) => q.kind === k);
    if (z) { const [x, y] = z.centroid(); const a = z.poseAbs(); return [a[0] * x + a[4] * y + a[12], a[1] * x + a[5] * y + a[13]]; }
  }
  return robot.home ? [robot.home.x, robot.home.y] : null;
}
function resource(robot: MobileRobot, name: string, capacity: number, unit: string, full = true) {
  return (robot.resources[name] ??= { capacity, level: full ? capacity : 0, unit });
}

/** First camera mounted on the carrier (or on its arm). */
function cameraOf(robot: MobileRobot): Camera | null {
  for (const it of robot.walk() as Iterable<Item>) if (it instanceof Camera) return it;
  return null;
}

/** Orient a row work path so that the carrier's arm faces the row (fixes the arm facing the neighbour row). */
export function orientForArm(fm: FleetManager, task: FleetTask, robot: MobileRobot): void {
  const row = rowOf(fm, task);
  const arm = armOf(robot);
  if (!row || !arm || !task.workPath || task.workPath.length < 2 || !task.meta.side) return;
  const mount = armMount(arm);
  const w = workDir(task);
  if (Math.abs(mount.y) < 50) {
    // centred arm (turret) reaches both sides: no turnaround for the side; start from the nearer end (rail trolleys
    // cannot turn on a rail)
    const p = robot.state, a = task.workPath[0], b = task.workPath[task.workPath.length - 1];
    if (Math.hypot(b[0] - p.x, b[1] - p.y) < Math.hypot(a[0] - p.x, a[1] - p.y) - 1) {
      task.workPath = [...task.workPath].reverse();
      task.location = [task.workPath[0][0], task.workPath[0][1]];
      task.goalHeading = Math.atan2(task.workPath[1][1] - task.workPath[0][1], task.workPath[1][0] - task.workPath[0][0]) * 180 / Math.PI;
    }
    return;
  }
  const sA = mount.side;
  const d = rowDirWorld(row);
  // travelling along +d with the line on the row's side s, the row is on the robot's right for s = +1
  const want = -task.meta.side * sA; // +1: along the row direction
  if ((w[0] * d[0] + w[1] * d[1]) * want < 0) {
    task.workPath = [...task.workPath].reverse();
    task.location = [task.workPath[0][0], task.workPath[0][1]];
    task.goalHeading = Math.atan2(task.workPath[1][1] - task.workPath[0][1], task.workPath[1][0] - task.workPath[0][0]) * 180 / Math.PI;
  }
}

// -- harvest / thin --------------------------------------------------------------------------------------------------

interface Stop { s: number; fruit: number[][] }

/**
 * Place base stops along the work path so that the fruit of the row side are inside the arm's reach. An arm on a lift
 * (`mountArmOnLift`) reaches every height of its stroke: the vertical distance counts only beyond the lift range. With a
 * lift the fruit of a stop are picked in height order (up at one stop, down at the next) so the lift sweeps once.
 */
export function planStops(fm: FleetManager, task: FleetTask, robot: MobileRobot, select: (f: { ripe: number; picked: boolean }, plantIdx: number, fruitIdx: number) => boolean): { stops: Stop[]; unreachable: number } {
  const row = rowOf(fm, task)!;
  const arm = armOf(robot);
  const w = task.workPath!;
  const a = w[0], dir = workDir(task);
  const nrm = [-dir[1], dir[0]]; // robot left
  const mount = arm ? armMount(arm) : { x: 0, y: 0, z: robot.kin.footprint[2], side: 1 as const };
  const stroke = arm ? armLift(arm).stroke : 0;
  const reachSetting = settingsOf(fm, task).reach as number | undefined;
  const R = reachSetting ?? (arm ? armReach(arm) * 0.9 : 1200);
  const ground = row.parent ? row.parent.poseAbs()[14] : 0;
  const zBase = robot.poseAbs()[14] - ground + mount.z;
  const items: Array<{ s: number; half: number; ref: number[]; z: number }> = [];
  let unreachable = 0;
  const all = fruitWorldPositions(row, false);
  let k = 0;
  row.plants.forEach((pl, pi) => pl.fruit.forEach((f, fi) => {
    const p = all[k++];
    if (!select(f, pi, fi)) return;
    if (task.meta.side && p.side !== task.meta.side) return;
    const s = (p.p[0] - a[0]) * dir[0] + (p.p[1] - a[1]) * dir[1];
    const lat = (p.p[0] - a[0]) * nrm[0] + (p.p[1] - a[1]) * nrm[1];
    const dz0 = p.p[2] - ground - zBase;
    const dz = dz0 - Math.max(0, Math.min(stroke, dz0)); // the lift brings the arm base up to the fruit height
    // no arm on the carrier (abstract picker, planner warning "without reach limits"): every fruit of the side is
    // picked, grouped in stops of ± R along the row
    const h2 = !arm && reachSetting === undefined ? R * R : R * R - (lat - mount.y) * (lat - mount.y) - dz * dz;
    if (h2 <= 0 || Math.sign(lat - 0) !== Math.sign(mount.y || lat)) { unreachable++; return; }
    items.push({ s, half: Math.sqrt(h2), ref: [pi, fi], z: p.p[2] });
  }));
  items.sort((x, y) => x.s - y.s);
  const len = Math.hypot(w[w.length - 1][0] - a[0], w[w.length - 1][1] - a[1]);
  const stops: Stop[] = [];
  const done = new Uint8Array(items.length);
  for (let i = 0; i < items.length; i++) {
    if (done[i]) continue;
    const sArm = items[i].s + items[i].half * 0.9;
    // stops are reached while still on the work path (a stop at s = len was skipped: the work phase ends there)
    const s = Math.max(0, Math.min(len - 50, sArm - mount.x));
    const armAt = s + mount.x;
    const at: number[] = [];
    for (let j = i; j < items.length; j++) {
      if (items[j].s - armAt > R) break;
      if (!done[j] && Math.abs(items[j].s - armAt) <= items[j].half) { done[j] = 1; at.push(j); }
    }
    if (!done[i]) { done[i] = 1; unreachable++; }
    if (stroke > 0) { const up = stops.length % 2 === 0 ? 1 : -1; at.sort((x, y) => (items[x].z - items[y].z) * up); }
    if (at.length) stops.push({ s, fruit: at.map((j) => items[j].ref) });
  }
  return { stops, unreachable };
}

/**
 * Arm motion during fleet harvest / thinning, on by default: the arm (and its lift) moves to every fruit it picks — a
 * collision-free IK configuration per fruit (`armGoalForFruit`), reached at the fruit's pick time through a
 * pre-approach pose 250 mm back along the gripper axis (retract → transfer → approach, linear joint motion per leg,
 * timed by the joint speed limits: a pick waits for the arm, the base stands meanwhile); between the stops of a row the
 * arm stays at its retract pose, it returns to its travel pose at the end of the row and for unloading trips. A fruit
 * the arm cannot reach without collision at its stop stays on the plant (KPI `armUnreachable`). Mission setting
 * `animateArm: false` (or `enabled = false`) keeps the timing-only model (base stops and seconds per fruit, no joint
 * motion) for batch runs; `armCollisionCheck: false` skips the collision test of the pick configurations.
 */
export const harvestAnimation = { enabled: true };

export interface ArmGoal { q: number[]; h: number }
/** Arm motion: from `from` (at t0) through the legs, each linear in joint space and lift height, arriving at its t1. */
interface ArmAnim { home: ArmGoal; from: ArmGoal; t0: number; legs: Array<{ to: ArmGoal; t1: number }>; key: string; settled: boolean; at: ArmGoal | null; atPre: ArmGoal | null }
const armAnims = new WeakMap<Robot, ArmAnim>();
/** Pre-approach / retract distance along the gripper axis (mm). */
const BACKOFF = 250;
/** Lift speed (mm/s) used to time the motions. */
const LIFT_SPEED = 400;
/** Collision test per arm, rebuilt for every task (row). */
const armFree = new WeakMap<Robot, { taskId: string; free: ArmFreeTest }>();
function freeTestFor(fm: FleetManager, task: FleetTask, arm: Robot): ArmFreeTest {
  let e = armFree.get(arm);
  if (!e || e.taskId !== task.id) { e = { taskId: task.id, free: armFreeTest(fm.station, arm, animOf(fm, arm).home) }; armFree.set(arm, e); }
  return e.free;
}
const APPROACH_ELEVATIONS = [0, 20, -20, 40, -40, 60, -60, -80, 80];

function armNow(arm: Robot): ArmGoal { return { q: arm.joints(), h: armLift(arm).h }; }
function animOf(fm: FleetManager, arm: Robot): ArmAnim {
  let A = armAnims.get(arm);
  if (!A) { const g = armNow(arm); A = { home: g, from: g, t0: fm.time, legs: [], key: 'home', settled: true, at: null, atPre: null }; armAnims.set(arm, A); }
  return A;
}
/** Interpolated arm state of an animation at time t (completed legs are consumed). */
function armAt(A: ArmAnim, t: number): ArmGoal {
  while (A.legs.length && t >= A.legs[0].t1) { A.from = A.legs[0].to; A.t0 = A.legs[0].t1; A.legs.shift(); }
  const leg = A.legs[0];
  if (!leg) return A.from;
  const f = leg.t1 > A.t0 ? Math.min(1, Math.max(0, (t - A.t0) / (leg.t1 - A.t0))) : 1;
  return { q: A.from.q.map((q, i) => q + ((leg.to.q[i] ?? q) - q) * f), h: A.from.h + (leg.to.h - A.from.h) * f };
}
/** Time (s) a leg needs at the arm's joint speed limits and the lift speed. */
function legSeconds(arm: Robot, a: ArmGoal, b: ArmGoal): number {
  const v = arm.jointMaxVelocities();
  let c = Math.abs(b.h - a.h) / LIFT_SPEED;
  b.q.forEach((q, i) => { c = Math.max(c, Math.abs(q - (a.q[i] ?? q)) / (v[i] || 180)); });
  return c;
}
/**
 * Start a motion of the arm through the poses `via` (no-op while already heading to the same key). A pose that changes
 * the lift height is reached in two legs so the arm reconfigures with the lift high: going up the lift rises first and
 * the joints move after, going down the joints move first and the lift descends after (keeps the forearm away from the
 * deck and the carrier). The legs share the time window in proportion to the time each needs at the joint speed limits:
 * arriving at `arrive` (a pick time) — later when the joint / lift speed limits need more time: the return value is that
 * delay (s) — else taking max(minSeconds, 1.25 × the time needed).
 */
function armMove(fm: FleetManager, arm: Robot, key: string, via: ArmGoal[], arrive: number | null, minSeconds = 0): number {
  const A = animOf(fm, arm);
  if (A.key === key) return 0;
  const start = armAt(A, fm.time);
  const pts: ArmGoal[] = [];
  let prev = start;
  for (const g of via) {
    if (Math.abs(g.h - prev.h) > 1) pts.push(g.h > prev.h ? { q: prev.q, h: g.h } : { q: g.q, h: prev.h });
    pts.push(g);
    prev = g;
  }
  const need = pts.map((g, i) => Math.max(0.05, legSeconds(arm, i ? pts[i - 1] : start, g)));
  const total = need.reduce((a, b) => a + b, 0);
  const T = arrive !== null ? Math.max(arrive - fm.time, pts.length ? total : 0) : Math.max(minSeconds, total * 1.25);
  let t = fm.time;
  A.legs = pts.map((g, i) => ({ to: g, t1: i === pts.length - 1 ? fm.time + T : (t += (T * need[i]) / total) }));
  A.from = start; A.t0 = fm.time; A.key = key; A.settled = false;
  A.at = null; A.atPre = null; // the legs already retract from the fruit the arm was on
  return arrive !== null ? Math.max(0, fm.time + T - arrive) : 0;
}
/** Put the arm at a goal now (it is on the fruit at the pick time); `pre` = its retract pose. */
function armPlace(fm: FleetManager, arm: Robot, key: string, g: ArmGoal, pre: ArmGoal | null): void {
  const A = animOf(fm, arm);
  A.from = g; A.t0 = fm.time; A.legs = []; A.key = key; A.settled = false; A.at = g; A.atPre = pre;
}
/** Poses from the current state to a fruit: retract from the fruit the arm is on, the fruit's pre-approach, the fruit. */
function viaToFruit(A: ArmAnim, g: ArmGoal, pre: ArmGoal | null): ArmGoal[] {
  return [...(A.at && A.atPre ? [A.atPre] : []), ...(pre ? [pre] : []), g];
}
/** Poses back to the travel pose (retract first when on a fruit). */
function viaHome(A: ArmAnim): ArmGoal[] { return [...(A.at && A.atPre ? [A.atPre] : []), A.home]; }
/** Between two stops of a row the arm only retracts from the last fruit and stays ready (no swing to the travel pose). */
function viaReady(A: ArmAnim): ArmGoal[] { return A.at && A.atPre ? [A.atPre] : []; }
/** Apply the arm animation at the manager's clock (every step, also between work phases). */
function applyArm(arm: Robot, t: number): void {
  const A = armAnims.get(arm);
  if (!A || A.settled) return;
  const g = armAt(A, t);
  arm.setJoints(g.q);
  setArmLift(arm, g.h);
  if (!A.legs.length) A.settled = true;
}

/**
 * Pre-approach / retract configuration of a pick goal: the TCP pulled back `d` mm along the gripper axis (same lift
 * height), the IK solution nearest to the pick joints; null when it is out of reach, on another branch or colliding.
 */
export function armBackoff(arm: Robot, g: ArmGoal, d = BACKOFF, free?: ArmFreeTest): ArmGoal | null {
  const tcp = arm.solveFK(g.q);
  const toolInv = invert(arm.poseTool());
  const dist = (q: number[]) => Math.max(...q.map((v, i) => Math.abs(v - g.q[i])));
  // the full distance, else shorter (out of reach, wrist branch change or contact further back)
  for (const k of [1, 0.7, 0.45, 0.25]) {
    const target = multiply(tcp, transl(0, 0, -d * k));
    let q: number[] | null = null;
    if (arm.hasAnalyticIK()) q = ikAll(arm.chain, multiply(target, toolInv), { seed: g.q, turns: 'none' }).sort((a, b) => dist(a) - dist(b))[0] ?? null;
    else { const r = arm.solveIK(target, { seed: g.q, restarts: 0, maxIterations: 60 }); q = r.ok ? r.joints : null; }
    if (q && dist(q) <= 90 && (!free || free(q, g.h))) return { q, h: g.h };
  }
  return null;
}

/** Collision test of an arm configuration (joints + lift height): true when free. */
export type ArmFreeTest = (q: number[], h: number) => boolean;

/**
 * Collision test of arm configurations with the simulator's checker: the arm links and its tool against themselves, the
 * carrier (deck, lift column), the other robots and the station's objects; pairs already in contact at the travel pose `home`
 * (the base standing on its deck) are ignored. The arm is put back where it was after every test.
 */
export function armFreeTest(station: Station, arm: Robot, home: ArmGoal): ArmFreeTest {
  const checker = robotCollisionChecker(station, arm, {});
  const key = (c: { a: { item: Item; part: string }; b: { item: Item; part: string } }) => `${c.a.item.id}/${c.a.part}|${c.b.item.id}/${c.b.part}`;
  const put = (q: number[], h: number) => { arm.setJoints(q); setArmLift(arm, h); };
  const q0 = arm.joints(), h0 = armLift(arm).h;
  put(home.q, home.h);
  const base = new Set(checker.check(1).map(key));
  put(q0, h0);
  return (q, h) => {
    const cq = arm.joints(), ch = armLift(arm).h;
    put(q, h);
    const hit = checker.check(1).some((c) => !base.has(key(c)));
    put(cq, ch);
    return !hit;
  };
}

/**
 * Joints (and lift height) that put the TCP on a fruit at world position p: approach from the arm base (tool z
 * horizontal towards the fruit, else tilted up / down to ±80°, else across the carrier for fruit close to the arm
 * base); with a lift the base goes to the fruit height (else ±300 / 600 mm). Every IK solution of a candidate is
 * considered — the configuration family (front / elbow / wrist flags) of `ref` (the travel pose) first, then nearest to
 * the current joints (continuity) — and the first one `free` accepts wins; null when no candidate is reachable and
 * collision-free.
 */
export function armGoalForFruit(arm: Robot, p: [number, number, number], free?: ArmFreeTest, ref?: number[]): ArmGoal | null {
  const lift = armLift(arm);
  const carrier = arm.parentAttachAbs();
  const mount = arm.pose();
  let hs = [lift.h];
  if (lift.stroke > 0) {
    const inv = invert(carrier);
    const zl = inv[2] * p[0] + inv[6] * p[1] + inv[10] * p[2] + inv[14];
    const cl = (h: number) => Math.max(0, Math.min(lift.stroke, h));
    const h0 = cl(zl - lift.baseZ);
    hs = [...new Set([h0, cl(h0 - 300), cl(h0 + 300), cl(h0 - 600), cl(h0 + 600)])];
  }
  const seed = arm.joints();
  const analytic = arm.hasAnalyticIK();
  const toolInv = invert(arm.poseTool());
  const refCfg = ref && analytic ? arm.configFlags(ref) : null;
  // the turn of each joint nearest to the current joints, but never more than 270° from the travel pose (no wind-up
  // of the wrist over a row that would end in a full-turn unwinding)
  const { lower, upper } = arm.jointLimits();
  const allRevolute = arm.chain.joints.every((j) => j.type !== 'prismatic');
  const bounded = (q: number[]) => !ref || !allRevolute ? q : q.map((v, i) => {
    let x = v;
    while (x - ref[i] > 270 && x - 360 >= (lower[i] ?? -Infinity)) x -= 360;
    while (ref[i] - x > 270 && x + 360 <= (upper[i] ?? Infinity)) x += 360;
    return x;
  });
  const dist = (q: number[]) => {
    let d = q.reduce((a, v, i) => a + Math.abs(v - seed[i]) * (i < 3 ? 1.5 : 1), 0);
    if (refCfg) { const c = arm.configFlags(q); d += (c.front !== refCfg.front ? 4000 : 0) + (c.elbowUp !== refCfg.elbowUp ? 2000 : 0) + (c.wristFlip !== refCfg.wristFlip ? 1000 : 0); }
    return d;
  };
  // horizontal approach directions: from the arm base towards the fruit, else across the carrier towards the fruit side
  const cy: [number, number] = [carrier[4], carrier[5]];
  const sideSign = Math.sign((p[0] - carrier[12]) * cy[0] + (p[1] - carrier[13]) * cy[1]) || 1;
  for (const h of hs) {
    if (lift.stroke > 0) mount[14] = lift.baseZ + h;
    const base = multiply(carrier, mount);
    const bp = getPos(base);
    const dx = p[0] - bp[0], dy = p[1] - bp[1], dl = Math.hypot(dx, dy);
    const dirs: Array<[number, number]> = [[cy[0] * sideSign, cy[1] * sideSign]];
    if (dl > 1) dirs.unshift([dx / dl, dy / dl]);
    const toBase = invert(base);
    for (const hor of dirs) for (const e of APPROACH_ELEVATIONS) {
      const c = Math.cos(e * Math.PI / 180), sn = Math.sin(e * Math.PI / 180);
      const target = multiply(toBase, poseFromZ(p, [hor[0] * c, hor[1] * c, sn], [0, 0, -1]));
      let sols: number[][];
      if (analytic) sols = ikAll(arm.chain, multiply(target, toolInv), { seed, turns: 'none' }).map(bounded).sort((a, b) => dist(a) - dist(b));
      else { const r = arm.solveIK(target, { seed, restarts: 1, maxIterations: 60 }); sols = r.ok ? [r.joints] : []; }
      for (const q of sols) if (!free || free(q, h)) return { q, h };
    }
  }
  return null;
}
/** World position of fruit `ref` = [plant, fruit] of a row (same frame as fruitWorldPositions). */
function fruitWorld(row: CropRow, ref: number[]): [number, number, number] | null {
  const pl = row.plants[ref[0]];
  const f = pl?.fruit[ref[1]];
  if (!f) return null;
  const [dx, dy] = row.direction();
  const [bx, by] = plantPosition(row, pl);
  const lx = bx + dx * f.p[0] - dy * f.p[1], ly = by + dy * f.p[0] + dx * f.p[1];
  const abs = row.parent ? row.parent.poseAbs() : null;
  return abs ? [abs[0] * lx + abs[4] * ly + abs[12], abs[1] * lx + abs[5] * ly + abs[13], f.p[2] + abs[14]] : [lx, ly, f.p[2]];
}

// the arm keeps moving between work phases (back to its travel pose after the last pick of a row)
fleetPlugins.push((fm) => { fm.afterStep.push(() => { for (const r of fm.robots()) { const a = armOf(r); if (a) applyArm(a, fm.time); } }); });

function pickProcess(kind: 'harvest' | 'thin') {
  const animated = (fm: FleetManager, task: FleetTask, robot: MobileRobot): Robot | null => {
    const arm = armOf(robot);
    return arm && (settingsOf(fm, task).animateArm ?? harvestAnimation.enabled) ? arm : null;
  };
  return {
    onAssign: (fm: FleetManager, task: FleetTask, robot: MobileRobot) => orientForArm(fm, task, robot),
    onWorkStart: (fm: FleetManager, task: FleetTask, robot: MobileRobot) => {
      const row = rowOf(fm, task);
      if (!row) return;
      const st = settingsOf(fm, task);
      const p = rt(task);
      if (kind === 'harvest') {
        const r = planStops(fm, task, robot, (f) => f.ripe >= 0.5 && !f.picked);
        p.stops = r.stops; p.unreachable = r.unreachable;
      } else {
        // thinning: unripe fruit above the target load per plant
        const keep = st.keepFruitPerPlant ?? Math.round((row.parent as FieldItem | null)?.crop.fruitPerPlant ?? 50) * 0.6;
        const excess = new Set<string>();
        row.plants.forEach((pl, pi) => {
          const live = pl.fruit.map((f, fi) => ({ f, fi })).filter((x) => !x.f.picked);
          const n = live.length - keep;
          if (n <= 0) return;
          live.filter((x) => x.f.ripe < 0.5).sort((x, y) => x.f.ripe - y.f.ripe).slice(0, n).forEach((x) => excess.add(`${pi}:${x.fi}`));
        });
        const r = planStops(fm, task, robot, (_f, pi, fi) => excess.has(`${pi}:${fi}`));
        p.stops = r.stops; p.unreachable = r.unreachable;
      }
      p.stopIdx = 0; p.picks = [];
      task.meta.unreachable = (task.meta.unreachable ?? 0) + p.unreachable;
      task.meta.fruitSource = kind === 'harvest' && cameraOf(robot) ? 'vision' : 'ground_truth';
      fm.bookStat(kind, 'unreachable', p.unreachable);
    },
    onWork: (fm: FleetManager, task: FleetTask, robot: MobileRobot, s: number, _dt: number, holding: boolean): WorkDirective | void => {
      const row = rowOf(fm, task);
      if (!row) return;
      const p = rt(task);
      const st = settingsOf(fm, task);
      const spf = st.secondsPerFruit ?? (kind === 'thin' ? 2 : 6);
      const fruitKg = task.meta.fruitKg ?? 0.18;
      const arm = animated(fm, task, robot);
      const keyOf = (pk: { ref: number[] }) => `${task.id}:${pk.ref[0]}:${pk.ref[1]}`;
      // pick goal (collision-free IK configuration on the fruit) and its pre-approach / retract pose, solved once per fruit
      const goalOf = (pk: { ref: number[]; g?: ArmGoal | null; pre?: ArmGoal | null; bp?: number[] }): ArmGoal | null => {
        // solved for the base pose at that time: solved again when the base has moved since (still braking at the stop)
        const bp = [robot.state.x, robot.state.y, robot.state.theta];
        if (pk.g !== undefined && pk.bp && (Math.hypot(bp[0] - pk.bp[0], bp[1] - pk.bp[1]) > 3 || Math.abs(bp[2] - pk.bp[2]) > 0.2)) pk.g = undefined;
        if (pk.g === undefined) {
          pk.bp = bp;
          const fw = fruitWorld(row, pk.ref);
          const free = st.armCollisionCheck === false || !arm ? undefined : freeTestFor(fm, task, arm);
          pk.g = fw && arm ? armGoalForFruit(arm, fw, free, animOf(fm, arm).home.q) : null;
          pk.pre = pk.g && arm ? armBackoff(arm, pk.g, BACKOFF, free) : null;
        }
        return pk.g;
      };
      // fruit removed at their pick time (with the arm on the fruit when the arm is animated)
      let pickedNow = false;
      while (p.picks?.length && p.picks[0].t <= fm.time) {
        const pk = p.picks.shift();
        const f = row.plants[pk.ref[0]]?.fruit[pk.ref[1]];
        if (!f || f.picked) continue;
        if (arm) {
          const g = goalOf(pk);
          if (!g) { fm.bookStat(kind, 'armUnreachable', 1); task.meta.armUnreachable = (task.meta.armUnreachable ?? 0) + 1; continue; }
          armPlace(fm, arm, keyOf(pk), g, pk.pre ?? null);
          pickedNow = true;
        }
        f.picked = true;
        actionEnergy(fm, robot, 'pick');
        if (kind === 'thin') { (f as any).thinned = true; task.meta.removed = (task.meta.removed ?? 0) + 1; }
        if (kind === 'harvest') {
          fm.bookYield({ fruit: 1, kg: fruitKg });
          task.meta.harvestedKg = (task.meta.harvestedKg ?? 0) + fruitKg;
          task.meta.harvested = (task.meta.harvested ?? 0) + 1;
          const bin = resource(robot, 'bin', st.binCapacityKg ?? 400, 'kg', false);
          bin.level += fruitKg;
        } else fm.bookStat('thin', 'fruitRemoved', 1);
        row.notify('plants');
      }
      // arm motion towards the next scheduled fruit (reached 0.1 s before its pick time), else back to the travel pose
      if (arm) {
        const next = p.picks?.[0];
        const A = animOf(fm, arm);
        if (next && A.key !== keyOf(next) && Math.abs(robot.state.v) > 2) {
          // the base is still braking at the stop: the arm starts towards the fruit once it stands (solved for that pose)
        } else if (next && A.key !== keyOf(next)) {
          const g = goalOf(next);
          // an unreachable fruit: the arm waits where it is (the fruit stays on the plant at its pick time)
          const late = armMove(fm, arm, keyOf(next), g ? viaToFruit(A, g, next.pre ?? null) : [], Math.max(fm.time + 0.2, next.t - 0.1));
          // the arm cannot get there in time at its speed limits: this pick and the rest of the stop happen later
          if (late > 0) for (const q of p.picks) q.t += late;
        } else if (!next && A.key !== 'home' && A.key !== 'ready') {
          // more stops ahead on this row: stay ready at the retract pose while the base moves on; else travel pose
          if (p.stopIdx < (p.stops?.length ?? 0)) armMove(fm, arm, 'ready', viaReady(A), null, 0.5);
          else armMove(fm, arm, 'home', viaHome(A), null, 2.5);
        }
      }
      if (holding) return;
      // picks delayed by the arm's speed limits: the base keeps standing until the last pick of the stop (and for the
      // step of a pick: the gripper is on the fruit while the base stands)
      if (arm && p.picks?.length) return { hold: Math.max(0.05, p.picks[p.picks.length - 1].t - fm.time + 0.05) };
      if (pickedNow) return { hold: Math.max(0.05, _dt) };
      if (kind === 'harvest') {
        const bin = robot.resources.bin;
        if (bin && bin.level >= bin.capacity) {
          const loc = serviceLocation(fm, robot, ['unloading', 'loading', 'charging']);
          if (loc) {
            if (arm) armMove(fm, arm, 'home', viaHome(animOf(fm, arm)), null, 2.5); // travel pose for the unloading trip
            return { service: { kind: 'unload', location: loc, duration: st.unloadSeconds ?? 90 } };
          }
        }
      }
      const stops: Stop[] = p.stops ?? [];
      if (p.stopIdx < stops.length && s >= stops[p.stopIdx].s - 25) {
        const stop = stops[p.stopIdx++];
        const settle = st.settleSeconds ?? 2;
        let fruit = stop.fruit;
        // with a camera on the carrier the arm only picks what the camera detects at the stop (misses stay on the tree)
        const cam = kind === 'harvest' ? cameraOf(robot) : null;
        if (cam) {
          const seen = new Set(detectFruit(fm.station, cam, { onlyRipe: true, solids: [] }).map((d) => d.fruit));
          const vis = fruit.filter((ref) => seen.has(row.plants[ref[0]]?.fruit[ref[1]]));
          fm.bookStat('harvest', 'missedByVision', fruit.length - vis.length);
          fm.bookStat('harvest', 'detectedAtStops', vis.length);
          task.meta.missedByVision = (task.meta.missedByVision ?? 0) + fruit.length - vis.length;
          fruit = vis;
        }
        fruit.forEach((ref, i) => p.picks.push({ t: fm.time + settle + (i + 1) * spf, ref }));
        return { hold: settle + fruit.length * spf + 0.05 };
      }
    },
    onService: (fm: FleetManager, _task: FleetTask, robot: MobileRobot, k: string) => { if (k === 'unload' && robot.resources.bin) { fm.bookStat('harvest', 'unloadedKg', robot.resources.bin.level); robot.resources.bin.level = 0; fm.bookStat('harvest', 'unloads', 1); } },
    onDone: (fm: FleetManager, task: FleetTask, robot: MobileRobot) => {
      const arm = animated(fm, task, robot);
      if (arm) armMove(fm, arm, 'home', viaHome(animOf(fm, arm)), null, 2.5);
      const row = rowOf(fm, task); row?.notify('plants'); (row?.parent as FieldItem | null)?.notify('rows'); return {};
    },
  };
}
registerTaskProcess('harvest', pickProcess('harvest'));
registerTaskProcess('thin', pickProcess('thin'));

// -- spray / irrigate ------------------------------------------------------------------------------------------------

function plantsAlong(fm: FleetManager, task: FleetTask, s0: number, s1: number): Array<{ row: CropRow; idx: number }> {
  const row = rowOf(fm, task);
  if (!row || !task.workPath) return [];
  const a = task.workPath[0], dir = workDir(task);
  const out: Array<{ row: CropRow; idx: number }> = [];
  row.plants.forEach((pl, i) => { const [x, y] = plantWorld(row, pl); const s = (x - a[0]) * dir[0] + (y - a[1]) * dir[1]; if (s >= s0 && s < s1) out.push({ row, idx: i }); });
  return out;
}

function liquidProcess(kind: 'spray' | 'irrigate') {
  const res = kind === 'spray' ? 'tank' : 'water';
  return {
    onWorkStart: (_fm: FleetManager, task: FleetTask) => { rt(task).s = 0; },
    onWork: (fm: FleetManager, task: FleetTask, robot: MobileRobot, s: number): WorkDirective | void => {
      const row = rowOf(fm, task);
      const st = settingsOf(fm, task);
      const p = rt(task);
      const tank = resource(robot, res, st.tankL ?? (kind === 'spray' ? 600 : 1000), 'L');
      const s0 = p.s ?? 0;
      if (s <= s0) return;
      const crop = (row?.parent as FieldItem | null)?.crop;
      let litres = 0;
      const plants = plantsAlong(fm, task, s0, s);
      if (kind === 'spray') {
        // tree-row-volume dosing: litres per metre of row side = canopy height x width x dose per m³
        const lpm = st.doseLPerM ?? ((crop ? (crop.canopyHeight / 1000) * (crop.canopyWidth / 1000) : 2) * (st.doseLPerM3 ?? 0.07));
        litres = lpm * ((s - s0) / 1000);
        for (const { row: r, idx } of plants) { const pl = r.plants[idx] as any; pl.sprayedL = (pl.sprayedL ?? 0) + lpm * (crop ? crop.plantSpacing / 1000 : 1); if (pl.health < 0.7) { pl.health = Math.min(1, pl.health + 0.3); fm.bookStat('spray', 'plantsTreated', 1); } }
      } else {
        const lpp = st.litresPerPlant ?? 4;
        litres = lpp * plants.length;
        for (const { row: r, idx } of plants) { const pl = r.plants[idx] as any; pl.waterL = (pl.waterL ?? 0) + lpp; }
      }
      if (litres > tank.level) {
        // tank empty part-way: apply what is left, go refill, come back here
        fm.bookStat(kind, 'litres', tank.level);
        tank.level = 0;
        p.s = s;
        const loc = serviceLocation(fm, robot, ['loading', 'charging']);
        if (loc) return { service: { kind: 'refill', location: loc, duration: tank.capacity / (st.refillLps ?? 5) } };
        return;
      }
      tank.level -= litres;
      fm.bookStat(kind, 'litres', litres);
      actionEnergy(fm, robot, 'litre', litres);
      p.s = s;
    },
    onService: (fm: FleetManager, _task: FleetTask, robot: MobileRobot) => { const t = robot.resources[res]; if (t) t.level = t.capacity; fm.bookStat(kind, 'refills', 1); },
    onDone: (_fm: FleetManager, task: FleetTask) => ({ areaM2: task.workPath ? (pathLength2(task.workPath) / 1000) * (task.meta.swathM ?? 2) : 0 }),
  };
}
registerTaskProcess('spray', liquidProcess('spray'));
registerTaskProcess('irrigate', liquidProcess('irrigate'));

function pathLength2(w: number[][]) { let l = 0; for (let i = 1; i < w.length; i++) l += Math.hypot(w[i][0] - w[i - 1][0], w[i][1] - w[i - 1][1]); return l; }

// -- mow / weed ------------------------------------------------------------------------------------------------------

function coverageProcess(kind: 'mow' | 'weed') {
  return {
    onWorkStart: (_fm: FleetManager, task: FleetTask) => { rt(task).s = 0; },
    onWork: (fm: FleetManager, task: FleetTask, _robot: MobileRobot, s: number): WorkDirective | void => {
      const p = rt(task);
      if (s > (p.s ?? 0)) { const a = ((s - p.s) / 1000) * (task.meta.swathM ?? 1.5); fm.bookStat(kind, 'areaM2', a); fm.bookYield({ areaM2: a }); p.s = s; }
      if (kind === 'weed') {
        // weeding stops at each plant position for the tool cycle
        const st = settingsOf(fm, task);
        const spp = st.secondsPerPlant ?? 0;
        if (spp > 0) {
          const pl = plantsAlong(fm, task, p.last ?? -1, s);
          if (pl.length) { p.last = s; fm.bookStat('weed', 'plantsWeeded', pl.length); return { hold: spp * pl.length }; }
        }
      }
    },
    onDone: (fm: FleetManager, task: FleetTask) => {
      const row = rowOf(fm, task);
      const field = row?.parent as FieldItem | null;
      if (field) { const inter = field.rows().reduce((a, r) => a + (r.length() / 1000) * (field.crop.rowSpacing / 1000), 0); fm.bookStat(kind, 'fieldInterRowM2', 0); if (inter > 0) fm.processStats[kind].coverage = (fm.processStats[kind].areaM2 ?? 0) / inter; }
      return {};
    },
  };
}
registerTaskProcess('mow', coverageProcess('mow'));
registerTaskProcess('weed', coverageProcess('weed'));

// -- prune / pollinate -------------------------------------------------------------------------------------------------

function perPlantProcess(kind: 'prune' | 'pollinate') {
  return {
    onAssign: (fm: FleetManager, task: FleetTask, robot: MobileRobot) => orientForArm(fm, task, robot),
    onWorkStart: (_fm: FleetManager, task: FleetTask) => { rt(task).last = -1e9; rt(task).done = 0; },
    onWork: (fm: FleetManager, task: FleetTask, _robot: MobileRobot, s: number, _dt: number, holding: boolean): WorkDirective | void => {
      if (holding) return;
      const p = rt(task);
      const row = rowOf(fm, task);
      const crop = (row?.parent as FieldItem | null)?.crop;
      const st = settingsOf(fm, task);
      const plants = plantsAlong(fm, task, p.last, s + 1);
      if (!plants.length) return;
      p.last = s + 1;
      let t = 0;
      for (const { row: r, idx } of plants) {
        const pl = r.plants[idx] as any;
        if (kind === 'prune' && pl.prunedAt !== undefined) continue;
        // workload grows with the canopy volume (m³): ~25 s/m³ pruning, ~4 s/m³ pollination
        const vol = (pl.height / 1000) * Math.PI * (pl.width / 2000) ** 2;
        t += st.secondsPerPlant ?? vol * (kind === 'prune' ? 25 : 4);
        if (kind === 'prune') pl.prunedAt = fm.time; else pl.pollinatedAt = fm.time;
        fm.bookStat(kind, 'plants', 1);
        actionEnergy(fm, _robot, kind === 'prune' ? 'cut' : 'plant');
      }
      void crop;
      if (t > 0) { fm.bookStat(kind, 'seconds', t); return { hold: t }; }
    },
  };
}
registerTaskProcess('prune', perPlantProcess('prune'));
registerTaskProcess('pollinate', perPlantProcess('pollinate'));

// -- scout ----------------------------------------------------------------------------------------------------------------

export interface ScoutObservation { t: number; rowId: string; plant: number; health: number; diseased: boolean; robot: string }

let scoutSeed = 99;
const scoutRnd = () => { scoutSeed = (scoutSeed * 16807) % 2147483647; return scoutSeed / 2147483647; };

registerTaskProcess('scout', {
  onWorkStart: (_fm, task) => { rt(task).last = -1e9; },
  onWork: (fm, task, robot, s) => {
    const p = rt(task);
    const range = 1500;
    const plants = plantsAlong(fm, task, p.last, s + range);
    if (!plants.length) return;
    p.last = s + range;
    const mission = missionOf(fm, task);
    const st = settingsOf(fm, task);
    const thr = st.diseaseThreshold ?? 0.6;
    for (const { row, idx } of plants) {
      const pl = row.plants[idx];
      const noise = (scoutRnd() + scoutRnd() + scoutRnd() - 1.5) * 0.04;
      const h = Math.max(0, Math.min(1, pl.health + noise));
      const obs: ScoutObservation = { t: fm.time, rowId: row.id, plant: idx, health: +h.toFixed(3), diseased: h < thr, robot: robot.name };
      if (mission) mission.observations.push(obs);
      fm.bookStat('scout', 'observations', 1);
      actionEnergy(fm, robot, 'image');
      if (obs.diseased) fm.bookStat('scout', 'diseased', 1);
    }
  },
  onDone: (fm, task) => {
    const mission = missionOf(fm, task);
    const st = settingsOf(fm, task);
    if (!mission || !st.followUp) return {};
    // follow-up tasks: one per row with diseased plants (e.g. spray), created once
    const row = rowOf(fm, task);
    if (!row || !task.workPath) return {};
    const sick = mission.observations.filter((o) => o.rowId === row.id && o.diseased);
    const key = `${row.id}:${st.followUp}`;
    if (!sick.length || mission.followUps.includes(key)) return {};
    mission.followUps.push(key);
    fm.addTask({ type: st.followUp, location: [task.workPath[0][0], task.workPath[0][1]], workPath: task.workPath.map((q) => [...q]), workSpeed: st.followUpSpeed ?? 800, requiredCapabilities: [st.followUp], segments: task.segments, meta: { missionId: mission.id, rowId: row.id, side: task.meta.side, swathM: task.meta.swathM, followUpOf: task.id, diseasedPlants: sick.length } });
    return {};
  },
});

// -- transport -------------------------------------------------------------------------------------------------------------

registerTaskProcess('transport', {
  onWork: (fm, task, robot, _s, _dt, holding) => {
    if (holding || !task.meta.dropoff) return;
    const cargo = resource(robot, 'cargo', robot.payloadKg, 'kg', false);
    cargo.level = Math.min(cargo.capacity, task.payloadKg ?? 0);
    return { service: { kind: 'deliver', location: task.meta.dropoff as [number, number], duration: task.meta.unloadSeconds ?? 30 } };
  },
  onService: (fm, task, robot, k) => { if (k === 'deliver') { fm.bookStat('transport', 'deliveredKg', robot.resources.cargo?.level ?? task.payloadKg ?? 0); fm.bookStat('transport', 'deliveries', 1); if (robot.resources.cargo) robot.resources.cargo.level = 0; } },
});

// -- traffic segments: orchard alleys --------------------------------------------------------------------------------------

/** Alley polygons (station frame) between consecutive row lines, plus the outer alleys. Ids match mission task segments. */
export function alleySegments(station: Station): TrafficSegment[] {
  const out: TrafficSegment[] = [];
  for (const field of station.itemsOfType<FieldItem>(ItemType.FIELD)) {
    const rows = field.rows();
    if (!rows.length) continue;
    const [dx, dy] = rows[0].direction();
    const nx = -dy, ny = dx;
    const a = field.poseAbs();
    const toW = (s: number, o: number) => { const x = s * dx + o * nx, y = s * dy + o * ny; return [a[0] * x + a[4] * y + a[12], a[1] * x + a[5] * y + a[13]]; };
    const byIdx = new Map<number, { o: number; s0: number; s1: number }>();
    for (const r of rows) {
      const o = r.start[0] * nx + r.start[1] * ny;
      const s0 = Math.min(r.start[0] * dx + r.start[1] * dy, r.end[0] * dx + r.end[1] * dy), s1 = Math.max(r.start[0] * dx + r.start[1] * dy, r.end[0] * dx + r.end[1] * dy);
      const e = byIdx.get(r.index);
      byIdx.set(r.index, e ? { o, s0: Math.min(e.s0, s0), s1: Math.max(e.s1, s1) } : { o, s0, s1 });
    }
    const idx = [...byIdx.keys()].sort((p, q) => p - q);
    const sp = field.crop.rowSpacing;
    for (let i = -1; i < idx.length; i++) {
      const lo = i >= 0 ? byIdx.get(idx[i])! : null, hi = i + 1 < idx.length ? byIdx.get(idx[i + 1])! : null;
      const o0 = lo ? lo.o : hi!.o - sp, o1 = hi ? hi.o : lo!.o + sp;
      const s0 = Math.min(lo?.s0 ?? Infinity, hi?.s0 ?? Infinity), s1 = Math.max(lo?.s1 ?? -Infinity, hi?.s1 ?? -Infinity);
      const id = `${field.id}:alley${i >= 0 ? idx[i] : (hi ? idx[0] - 1 : -1)}`;
      out.push({ id, polygon: [toW(s0, o0), toW(s1, o0), toW(s1, o1), toW(s0, o1)], capacity: 1 });
    }
  }
  return out;
}
registerSegmentProvider('agri.alleys', alleySegments);

/** Lower the health of plants within `radius` of a point (disease outbreak that scouting can discover). */
export function infectPlants(station: Station, center: [number, number], radius: number, severity = 0.5): number {
  let n = 0;
  for (const row of station.itemsOfType<CropRow>(ItemType.CROP_ROW)) row.plants.forEach((pl) => {
    const [x, y] = plantWorld(row, pl);
    if (Math.hypot(x - center[0], y - center[1]) <= radius) { pl.health = Math.max(0, pl.health - severity); n++; }
  });
  return n;
}

fleetPlugins.push((fm) => { fm.logMsg(`agri process models active (${['harvest', 'thin', 'spray', 'irrigate', 'mow', 'weed', 'prune', 'pollinate', 'scout', 'transport'].join(', ')})`); });
