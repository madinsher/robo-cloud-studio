/**
 * Mobile robot drive models and path tracking.
 *
 * Drive models (`integrate`): differential (wheel-speed saturation), tracked / skid steer (track slip and ICR
 * expansion), omni (vx, vy, ω), legged (holonomic with a lateral speed limit), Ackermann bicycle (steering angle state
 * with angle and rate limits, reverse) and aerial (x, y, z, yaw with climb / hover). The state point is the item origin
 * (footprint centre). The reference point of the drive, the point without lateral velocity, is the origin for
 * differential / tracked / legged drives and the rear axle (`rearAxleOffset`, body x = kin.rearAxle, default
 * −wheelBase/2) for Ackermann: the bicycle model, the turning-radius limit, Reeds–Shepp joins, Hybrid A* arcs and the
 * path tracker all hold at the rear axle, and the origin follows as a rigid body (V3 F4: referencing the bicycle
 * model to the origin made the rear wheels slide sideways 8–19° in turns).
 *
 * Paths: plain [x, y] polylines or pose paths [x, y, headingDeg, dir] (dir 1 forward, -1 reverse, 0 rotate in place),
 * the format produced by the Hybrid-A* / Reeds–Shepp planners. `followPath` converts a polyline into a pose path the
 * drive can execute (turn-then-drive for differential/tracked/legged, Reeds–Shepp chains for Ackermann, constant
 * heading for holonomic drives). `stepMobile` tracks pose paths with a curvature feed-forward + lateral/heading
 * feedback law that works forward and in reverse, and stops at cusps.
 */
import { MobileRobot, MobileKinematics } from './items';
import { DEG, RAD } from '../core/math/pose';
import { pathLength, regulatedMinRadius } from './planner';
import { reedsSheppPath } from './reeds_shepp';
import { shiftPose, shiftPosePath } from './footprint';

export interface VelocityCommand {
  v: number; // mm/s, body x (signed)
  omega: number; // deg/s
  /** Body-frame lateral velocity (mm/s) — holonomic drives only. */
  vy?: number;
  /** Vertical velocity (mm/s) — aerial only. */
  vz?: number;
  /** Requested steering angle (deg) — Ackermann; derived from ω/v when absent. */
  steer?: number;
}

const wrap180 = (a: number) => ((((a + 180) % 360) + 360) % 360) - 180;
const clamp = (v: number, lo: number, hi: number) => Math.max(lo, Math.min(hi, v));

/** Effective minimum turning radius (mm) of a car-like robot (0 = can turn in place). */
export function minTurnRadius(k: MobileKinematics): number {
  if (k.drive !== 'ackermann') return k.minTurnRadius || 0;
  const fromSteer = k.wheelBase / Math.tan(Math.max(1, k.maxSteer) * DEG);
  return Math.max(k.minTurnRadius || 0, fromSteer);
}

/**
 * Body x (mm) of the drive's reference point — the point without lateral velocity that the curvature limit and path
 * tracking refer to: the rear axle of an Ackermann base (kin.rearAxle, default −wheelBase/2), 0 for the other drives.
 */
export function rearAxleOffset(k: MobileKinematics): number {
  if (k.drive !== 'ackermann') return 0;
  const a = k.rearAxle ?? -Math.max(0, k.wheelBase) / 2;
  return Number.isFinite(a) ? a : 0;
}

export function isHolonomic(k: MobileKinematics): boolean { return k.drive === 'omni' || k.drive === 'aerial'; }

/** Mechanical power draw (W) for the current state. */
export function powerDraw(robot: MobileRobot): number {
  const b = robot.battery, st = robot.state, k = robot.kin;
  const speed = Math.hypot(st.v, st.vy ?? 0) / 1000;
  let w = b.idleW + speed * b.drivePerMps * (b.driveFactor ?? 1) + (b.equipmentW ?? 0);
  if (k.drive === 'tracked') w += Math.abs(st.omega * DEG) * (k.track / 2000) * b.drivePerMps * (1 + (k.turnSlip ?? 0.5)); // skid-steer turning losses
  if (k.drive === 'aerial' && (st.z ?? 0) > 50) w += (b.hoverW ?? 400) + Math.max(0, st.vz ?? 0) / 1000 * (b.climbPerMps ?? 300);
  return w;
}

/** Integrate the robot state for dt seconds with the given command, respecting the drive model and its limits. */
export function integrate(robot: MobileRobot, cmd: VelocityCommand, dt: number): void {
  const k = robot.kin;
  const st = robot.state;
  const accel = (cur: number, target: number, a: number) => cur + clamp(target - cur, -a * dt, a * dt);
  const maxYaw = k.maxYawRate;
  let vx = st.v, vy = st.vy ?? 0, omega: number;
  /** Lateral body velocity of the origin when the reference point is not the origin (Ackermann rear axle). */
  let vyOrigin: number | null = null;
  switch (k.drive) {
    case 'differential':
    case 'tracked': {
      // wheel / track speed saturation preserves the commanded curvature
      const half = (k.track / 2) * (k.drive === 'tracked' ? 1 + (k.turnSlip ?? 0) : 1);
      let v = clamp(cmd.v, -k.maxReverseSpeed, k.maxSpeed);
      let w = clamp(cmd.omega, -maxYaw, maxYaw) * DEG;
      const wheel = Math.max(Math.abs(v - w * half), Math.abs(v + w * half));
      if (wheel > k.maxSpeed) { const s = k.maxSpeed / wheel; v *= s; w *= s; }
      if (k.minTurnRadius > 0 && Math.abs(v) > 1) w = clamp(w, -Math.abs(v) / k.minTurnRadius, Math.abs(v) / k.minTurnRadius);
      vx = accel(st.v, v, k.maxAccel);
      const wPrev = st.omega * DEG;
      w = wPrev + clamp(w - wPrev, -(k.maxAccel / Math.max(100, half)) * dt, (k.maxAccel / Math.max(100, half)) * dt);
      omega = w * RAD;
      vy = 0;
      if (k.drive === 'tracked') {
        // commanded track speeds -> actual body motion with longitudinal slip and lateral skid in turns
        const s = clamp(k.slip ?? 0, 0, 0.9);
        const vAct = vx * (1 - s);
        const skid = (k.turnSlip ?? 0) > 0 ? -0.05 * (k.turnSlip ?? 0) * vAct * Math.sin(clamp(w * 0.5, -1, 1)) : 0;
        st.v = vx; st.omega = omega; st.vy = skid;
        moveBody(robot, vAct, skid, omega, dt);
        return finish(robot, dt);
      }
      break;
    }
    case 'omni':
    case 'legged':
    case 'aerial': {
      const latMax = k.maxLateralSpeed ?? (k.drive === 'legged' ? 0.4 * k.maxSpeed : k.maxSpeed);
      let tx = clamp(cmd.v, -(k.drive === 'omni' || k.drive === 'aerial' ? k.maxSpeed : k.maxReverseSpeed), k.maxSpeed);
      let ty = clamp(cmd.vy ?? 0, -latMax, latMax);
      const n = Math.hypot(tx, ty);
      if (n > k.maxSpeed) { tx *= k.maxSpeed / n; ty *= k.maxSpeed / n; }
      vx = accel(st.v, tx, k.maxAccel);
      vy = accel(st.vy ?? 0, ty, k.maxAccel);
      omega = accel(st.omega, clamp(cmd.omega, -maxYaw, maxYaw), Math.max(90, maxYaw * 2));
      if (k.drive === 'aerial') {
        const climb = k.maxClimbRate ?? 1500;
        st.vz = accel(st.vz ?? 0, clamp(cmd.vz ?? 0, -climb, climb), Math.max(500, k.maxAccel));
        st.z = clamp((st.z ?? 0) + (st.vz ?? 0) * dt, 0, k.maxAltitude ?? 120000);
        if ((st.z ?? 0) <= 0 && (st.vz ?? 0) < 0) st.vz = 0;
        // no horizontal motion while landed
        if ((st.z ?? 0) < 50) { vx = 0; vy = 0; omega = 0; }
      }
      break;
    }
    case 'ackermann':
    default: {
      const L = Math.max(1, k.wheelBase);
      const rMin = minTurnRadius(k);
      const steerMax = Math.min(k.maxSteer, Math.atan(L / Math.max(1, rMin)) * RAD);
      vx = accel(st.v, clamp(cmd.v, -k.maxReverseSpeed, k.maxSpeed), k.maxAccel);
      let desired: number;
      if (cmd.steer !== undefined) desired = cmd.steer;
      else if (Math.abs(cmd.v) > 1) desired = Math.atan((cmd.omega * DEG * L) / cmd.v) * RAD;
      else desired = st.steer;
      desired = clamp(desired, -steerMax, steerMax);
      const rate = k.maxSteerRate ?? 60;
      st.steer = clamp(st.steer + clamp(desired - st.steer, -rate * dt, rate * dt), -steerMax, steerMax);
      omega = ((vx * Math.tan(st.steer * DEG)) / L) * RAD;
      if (Math.abs(omega) > maxYaw) omega = Math.sign(omega) * maxYaw; // yaw-rate limit (fast at low radius)
      vy = 0;
      // the bicycle model holds at the rear axle (body x = a): the rigid body turns about the ICR on the rear axle line,
      // so the origin moves with the lateral body velocity −a·ω (v is the rear-axle speed, the odometer counts it)
      vyOrigin = -rearAxleOffset(k) * omega * DEG;
      break;
    }
  }
  st.v = vx; st.vy = vy; st.omega = omega;
  if (vyOrigin !== null) moveBody(robot, vx, vyOrigin, omega, dt, Math.abs(vx));
  else moveBody(robot, vx, vy, omega, dt);
  finish(robot, dt);
}

/** Exact planar motion over dt with constant body velocities (arc integration); `odoSpeed` overrides the odometer rate. */
function moveBody(robot: MobileRobot, vx: number, vy: number, omegaDeg: number, dt: number, odoSpeed?: number): void {
  const st = robot.state;
  const w = omegaDeg * DEG, th = st.theta * DEG;
  let dxb: number, dyb: number;
  if (Math.abs(w * dt) < 1e-6) { dxb = vx * dt; dyb = vy * dt; }
  else {
    const s = Math.sin(w * dt), c = 1 - Math.cos(w * dt);
    dxb = (vx * s - vy * c) / w;
    dyb = (vx * c + vy * s) / w;
  }
  st.x += Math.cos(th) * dxb - Math.sin(th) * dyb;
  st.y += Math.sin(th) * dxb + Math.cos(th) * dyb;
  st.theta = wrap180(st.theta + omegaDeg * dt);
  st.odometer += (odoSpeed ?? Math.hypot(vx, vy)) * dt;
}

function finish(robot: MobileRobot, dt: number): void {
  robot.battery.levelWh = Math.max(0, robot.battery.levelWh - (powerDraw(robot) * dt) / 3600);
  robot.syncPoseFromState();
}

export interface PurePursuitOptions {
  /** Pose believed by the robot (localization estimate); the controller steers from it instead of the true state. */
  pose?: { x: number; y: number; theta: number };
  lookahead?: number; // mm
  speed?: number; // mm/s
  goalTolerance?: number; // mm
  slowdownDistance?: number; // mm
  allowReverse?: boolean;
  /** Heading tolerance (deg) for rotate-in-place legs and the final heading. */
  headingTolerance?: number;
}

/** Legacy pure pursuit on a 2-D polyline (kept for tools that drive a unicycle directly). Returns null when done. */
export function purePursuit(robot: MobileRobot, opts: PurePursuitOptions = {}): VelocityCommand | null {
  const st = opts.pose ? { ...robot.state, x: opts.pose.x, y: opts.pose.y, theta: opts.pose.theta } : robot.state;
  const path = st.path;
  if (!path || path.length === 0) return null;
  const L = opts.lookahead ?? Math.max(600, robot.kin.wheelBase * 1.2);
  const vmax = Math.min(opts.speed ?? robot.kin.maxSpeed, robot.kin.maxSpeed);
  const tol = opts.goalTolerance ?? 150;
  const goal = path[path.length - 1];
  const dGoal = Math.hypot(goal[0] - st.x, goal[1] - st.y);
  if (dGoal < tol) return null;
  let best = st.pathIndex, bd = Infinity;
  for (let i = st.pathIndex; i < path.length; i++) {
    const d = Math.hypot(path[i][0] - st.x, path[i][1] - st.y);
    if (d < bd) { bd = d; best = i; }
    if (d > bd + 2 * L) break;
  }
  robot.state.pathIndex = best;
  let target = goal;
  for (let i = best; i < path.length - 1; i++) {
    const pt = circleSegment([st.x, st.y], L, path[i], path[i + 1]);
    if (pt) { target = pt; break; }
  }
  if (Math.hypot(target[0] - st.x, target[1] - st.y) > L * 1.5 && dGoal > L) target = path[best];
  const th = st.theta * DEG;
  const dx = target[0] - st.x, dy = target[1] - st.y;
  const lx = Math.cos(th) * dx + Math.sin(th) * dy;
  const ly = -Math.sin(th) * dx + Math.cos(th) * dy;
  const ld = Math.hypot(lx, ly) || 1;
  const curvature = (2 * ly) / (ld * ld);
  const heading = Math.atan2(ly, lx);
  let v = vmax;
  const slow = opts.slowdownDistance ?? 1500;
  if (dGoal < slow) v = Math.max(150, vmax * (dGoal / slow));
  v *= Math.max(0.2, 1 - Math.abs(heading) / Math.PI);
  let omega = curvature * v * RAD;
  if (Math.abs(heading) > 60 * DEG && robot.kin.drive !== 'ackermann') { v = 0; omega = Math.sign(heading) * robot.kin.maxYawRate * 0.7; }
  return { v, omega };
}

function circleSegment(c: number[], r: number, a: number[], b: number[]): number[] | null {
  const dx = b[0] - a[0], dy = b[1] - a[1];
  const fx = a[0] - c[0], fy = a[1] - c[1];
  const A = dx * dx + dy * dy, B = 2 * (fx * dx + fy * dy), C = fx * fx + fy * fy - r * r;
  const disc = B * B - 4 * A * C;
  if (disc < 0 || A === 0) return null;
  const t2 = (-B + Math.sqrt(disc)) / (2 * A);
  if (t2 >= 0 && t2 <= 1) return [a[0] + t2 * dx, a[1] + t2 * dy];
  return null;
}

// ---------------------------------------------------------------------------------------------------------------
// Pose paths
// ---------------------------------------------------------------------------------------------------------------

export function isPosePath(path: number[][]): boolean { return path.length > 0 && path[0].length >= 4 && path[path.length - 1].length >= 4; }

/** Sample a straight segment as pose points every `step` mm. */
function straightPoses(a: number[], b: number[], th: number, dir: number, step = 200): number[][] {
  const len = Math.hypot(b[0] - a[0], b[1] - a[1]);
  const n = Math.max(1, Math.ceil(len / step));
  const out: number[][] = [];
  for (let i = 0; i <= n; i++) out.push([a[0] + ((b[0] - a[0]) * i) / n, a[1] + ((b[1] - a[1]) * i) / n, th, dir]);
  return out;
}

export interface FollowOptions {
  /** Final heading (deg) to reach at the goal (rotate in place / Reeds–Shepp approach). */
  goalHeading?: number;
  /** Allow reverse motion in the converted path (Ackermann goal behind: back up instead of turning around). */
  allowReverse?: boolean;
  /** Start pose (defaults to the robot state). */
  start?: { x: number; y: number; theta: number };
}

/**
 * Convert a 2-D polyline into a pose path executable by the robot's drive:
 * - differential / tracked / legged: rotate in place at every vertex that turns by more than 8°, then drive straight
 *   (no corner cutting, the footprint follows the planned corridor);
 * - Ackermann: Reeds–Shepp connections with the minimum turning radius between waypoint poses (headings along the
 *   segments, bisectors at interior vertices); the goal heading is free unless given (backing up when shorter);
 * - omni / aerial: the polyline with the current heading held (or the goal heading).
 */
export function toPosePath(robot: MobileRobot, path: number[][], opts: FollowOptions = {}): number[][] {
  if (isPosePath(path)) return path.map((p) => [p[0], p[1], p[2], p[3]]);
  const st = opts.start ?? robot.state;
  const k = robot.kin;
  const pts = path.map((p) => [p[0], p[1]]);
  if (!pts.length) return [];
  // drop leading points on top of the robot (within the arrival tolerance: never turn around to reach them)
  if (pts.length > 1 && Math.hypot(pts[0][0] - st.x, pts[0][1] - st.y) < 300) pts[0] = [st.x, st.y];
  else if (Math.hypot(pts[0][0] - st.x, pts[0][1] - st.y) > 1) pts.unshift([st.x, st.y]);
  else pts[0] = [st.x, st.y];
  const dedup: number[][] = [pts[0]];
  for (const p of pts.slice(1)) if (Math.hypot(p[0] - dedup[dedup.length - 1][0], p[1] - dedup[dedup.length - 1][1]) > 1) dedup.push(p);
  const out: number[][] = [];
  if (isHolonomic(k)) {
    const th = opts.goalHeading ?? st.theta;
    out.push([st.x, st.y, st.theta, 1]);
    for (let i = 1; i < dedup.length; i++) out.push(...straightPoses(dedup[i - 1], dedup[i], th, 1).slice(1));
    return out;
  }
  if (k.drive === 'ackermann') {
    // Reeds–Shepp joins are paths of the rear axle (where the radius limit holds): planned between the rear-axle poses
    // of the waypoint poses and returned as origin poses
    const ax = rearAxleOffset(k);
    const R = minTurnRadius(k) * 1.15;
    const segDir = (i: number) => Math.atan2(dedup[i + 1][1] - dedup[i][1], dedup[i + 1][0] - dedup[i][0]) * RAD;
    const poses: Array<{ x: number; y: number; theta: number; free?: boolean }> = [{ x: st.x, y: st.y, theta: st.theta }];
    for (let i = 1; i < dedup.length; i++) {
      const last = i === dedup.length - 1;
      let th: number;
      if (!last) { const a = segDir(i - 1), b = segDir(i); th = a + wrap180(b - a) / 2; }
      else th = opts.goalHeading ?? segDir(i - 1);
      poses.push({ x: dedup[i][0], y: dedup[i][1], theta: th, free: last && opts.goalHeading === undefined });
    }
    out.push([st.x, st.y, st.theta, 1]);
    for (let i = 1; i < poses.length; i++) {
      const a = poses[i - 1], b = poses[i];
      const cands = b.free ? [b.theta, wrap180(b.theta + 180)] : [b.theta];
      let best: number[][] | null = null, bestCost = Infinity;
      for (const th of cands) {
        const rs = reedsSheppPath(shiftPose({ x: a.x, y: a.y, theta: a.theta }, ax), shiftPose({ x: b.x, y: b.y, theta: th }, ax), R, { allowReverse: opts.allowReverse ?? true, step: 150 });
        if (rs && rs.cost < bestCost) { bestCost = rs.cost; best = rs.poses; }
      }
      if (best) out.push(...shiftPosePath(best.slice(1), -ax));
    }
    return out;
  }
  // differential / tracked / legged: turn then drive
  let heading = st.theta;
  out.push([st.x, st.y, heading, 1]);
  const turnTol = 8;
  for (let i = 1; i < dedup.length; i++) {
    const a = dedup[i - 1], b = dedup[i];
    let th = Math.atan2(b[1] - a[1], b[0] - a[0]) * RAD;
    let dir = 1;
    // short reverse moves (goal just behind) when allowed
    if (opts.allowReverse && Math.abs(wrap180(th - heading)) > 150 && Math.hypot(b[0] - a[0], b[1] - a[1]) < 3000) { th = wrap180(th + 180); dir = -1; }
    if (Math.abs(wrap180(th - heading)) > turnTol) out.push([a[0], a[1], th, 0]);
    out.push(...straightPoses(a, b, th, dir).slice(1));
    heading = th;
  }
  if (opts.goalHeading !== undefined && Math.abs(wrap180(opts.goalHeading - heading)) > 0.5) {
    const g = dedup[dedup.length - 1];
    out.push([g[0], g[1], opts.goalHeading, 0]);
  }
  return out;
}

/** Legs of a pose path: maximal runs of points reached with the same motion (dir of point i = motion from i-1 to i). */
export function pathLegs(path: number[][]): Array<{ a: number; b: number; dir: number }> {
  const legs: Array<{ a: number; b: number; dir: number }> = [];
  let a = 0;
  for (let i = 1; i < path.length; i++) {
    if (i + 1 < path.length && path[i + 1][3] === path[i][3]) continue;
    legs.push({ a, b: i, dir: path[i][3] });
    a = i;
  }
  return legs;
}

const legCache = new WeakMap<number[][], Array<{ a: number; b: number; dir: number }>>();
function legsOf(path: number[][]) {
  let l = legCache.get(path);
  if (!l) { l = pathLegs(path); legCache.set(path, l); }
  return l;
}

/** Assign a path to a robot and start following (polylines are converted for the drive, see toPosePath). */
export function followPath(robot: MobileRobot, path: number[][], opts: FollowOptions = {}): void {
  robot.state.path = toPosePath(robot, path, opts);
  robot.state.pathIndex = 0;
  robot.state.status = 'moving';
  if (robot.kin.drive === 'aerial' && robot.state.zTarget === undefined) robot.state.zTarget = robot.kin.cruiseAltitude ?? 5000;
}

interface TrackState { leg: number }
const trackStates = new WeakMap<MobileRobot, { path: number[][]; t: TrackState }>();
/** Rear-axle pose paths of the origin pose paths followed by Ackermann bases (cached per path and offset). */
const refPaths = new WeakMap<number[][], { a: number; p: number[][] }>();
function refPathOf(path: number[][], a: number): number[][] {
  if (!a) return path;
  const c = refPaths.get(path);
  if (c && c.a === a) return c.p;
  const p = shiftPosePath(path, a);
  refPaths.set(path, { a, p });
  return p;
}

/**
 * Pose-path tracker. Non-holonomic legs use the kinematic law (s = distance travelled, e_y lateral error left of the
 * reference, e_θ heading error, κ_p path curvature dθ/ds): κ = dir·κ_p − k1·e_y − dir·k2·e_θ, k1 = 1/Ld², k2 = 2/Ld
 * (critically damped), which is stable forward and in reverse; rotate legs turn in place; cusps stop the robot.
 * Holonomic drives track the path point-wise with a heading controller. Returns null when the path is complete.
 */
export function trackPath(robot: MobileRobot, opts: PurePursuitOptions = {}): VelocityCommand | null {
  const statePath = robot.state.path;
  if (!statePath || !statePath.length) return null;
  const k = robot.kin;
  // the tracking law runs at the drive's reference point: the rear axle of an Ackermann base follows the rear-axle
  // path of the (origin) pose path, so the curvature it is given is the one its bicycle model can drive
  const ax = rearAxleOffset(k);
  const path = refPathOf(statePath, ax);
  const pose = shiftPose(opts.pose ?? robot.state, ax);
  let ts = trackStates.get(robot);
  if (!ts || ts.path !== path) {
    ts = { path, t: { leg: 0 } };
    const legs = legsOf(path);
    // resume: leg containing the stored path index
    const idx = robot.state.pathIndex;
    for (let i = 0; i < legs.length; i++) if (idx >= legs[i].a && idx <= legs[i].b) { ts.t.leg = i; if (idx < legs[i].b) break; }
    trackStates.set(robot, ts);
  }
  const legs = legsOf(path);
  const vcap = Math.min(opts.speed ?? k.maxSpeed, k.maxSpeed);
  const goalTol = opts.goalTolerance ?? 60;
  const headTol = opts.headingTolerance ?? 1.5;
  const decel = Math.max(100, k.maxAccel * 0.8);
  for (let guard = 0; guard < legs.length + 2; guard++) {
    const leg = legs[ts.t.leg];
    if (!leg) return null;
    const last = ts.t.leg === legs.length - 1;
    if (isHolonomic(k)) return holonomicCommand(robot, path, pose, opts, vcap, goalTol, headTol);
    if (leg.dir === 0 && k.drive === 'ackermann') { robot.state.pathIndex = leg.b; if (last) return null; ts.t.leg++; continue; }
    if (leg.dir === 0) {
      const target = path[leg.b][2];
      const err = wrap180(target - pose.theta);
      if (Math.abs(err) <= headTol) {
        robot.state.pathIndex = leg.b;
        if (last) return null;
        ts.t.leg++;
        continue;
      }
      if (Math.abs(robot.state.v) > 30) return { v: 0, omega: 0 }; // stop before turning on the spot
      const wmax = k.maxYawRate * 0.8;
      // braking-aware yaw profile (no overshoot): ω = sqrt(2·α·|err|)
      const alpha = Math.max(30, (k.maxAccel / Math.max(100, k.track / 2)) * RAD * 0.5);
      const w = Math.min(wmax, Math.sqrt(2 * alpha * Math.abs(err)), Math.max(3, Math.abs(err) * 4));
      return { v: 0, omega: Math.sign(err) * w };
    }
    // locate the reference on the leg (monotonic search ahead of the stored index)
    let i0 = Math.max(leg.a, Math.min(robot.state.pathIndex, leg.b - 1));
    let bestI = i0, bestD = Infinity, bestT = 0;
    for (let i = i0; i < leg.b; i++) {
      const a = path[i], b = path[i + 1];
      const dx = b[0] - a[0], dy = b[1] - a[1];
      const l2 = dx * dx + dy * dy;
      const t = l2 > 1e-9 ? clamp(((pose.x - a[0]) * dx + (pose.y - a[1]) * dy) / l2, 0, 1) : 0;
      const d = Math.hypot(pose.x - a[0] - t * dx, pose.y - a[1] - t * dy);
      if (d < bestD - 1e-6) { bestD = d; bestI = i; bestT = t; }
      if (d > bestD + 3000) break;
    }
    robot.state.pathIndex = bestI;
    const a = path[bestI], b = path[bestI + 1];
    const segLen = Math.hypot(b[0] - a[0], b[1] - a[1]);
    // remaining distance along the leg
    let rem = segLen * (1 - bestT);
    for (let i = bestI + 1; i < leg.b; i++) rem += Math.hypot(path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1]);
    // passed the end of the leg (projection beyond the last point)?
    const end = path[leg.b], prevE = path[Math.max(leg.a, leg.b - 1)];
    const tdx = end[0] - prevE[0], tdy = end[1] - prevE[1], tl = Math.hypot(tdx, tdy) || 1;
    const along = ((pose.x - end[0]) * tdx + (pose.y - end[1]) * tdy) / tl; // >0 beyond the end
    // an Ackermann base cannot fix its heading at the goal: it drives its last leg to the end (a Reeds–Shepp approach
    // still curves there: stopping 80 mm early on a 1.5 m radius leaves 3° of heading error, V3 Navigate legs)
    const legTol = last ? (k.drive === 'ackermann' ? Math.min(goalTol, 15) : goalTol) : Math.min(goalTol, 80);
    if (rem < legTol || (bestI === leg.b - 1 && along > -legTol * 0.5)) {
      robot.state.pathIndex = leg.b;
      if (last) return null;
      ts.t.leg++;
      continue;
    }
    // reference pose and curvature
    const thA = a[2], thB = b[2];
    const thR = thA + wrap180(thB - thA) * bestT;
    const kp = segLen > 1 ? (wrap180(thB - thA) * DEG) / segLen : 0;
    const xr = a[0] + (b[0] - a[0]) * bestT, yr = a[1] + (b[1] - a[1]) * bestT;
    const cr = Math.cos(thR * DEG), sr = Math.sin(thR * DEG);
    const ey = -sr * (pose.x - xr) + cr * (pose.y - yr);
    const eth = wrap180(pose.theta - thR) * DEG;
    const dir = leg.dir;
    const Ld = Math.max(400, Math.min(1500, (k.drive === 'ackermann' ? 0.8 : 0.5) * Math.max(k.wheelBase, k.footprint[0] * 0.5)));
    // differential-like drives realign on the spot when far off heading
    if (k.drive !== 'ackermann' && Math.abs(eth) > 45 * DEG) {
      if (Math.abs(robot.state.v) > 30) return { v: 0, omega: 0 };
      return { v: 0, omega: -Math.sign(eth) * k.maxYawRate * 0.6 };
    }
    const eyc = clamp(ey, -Ld, Ld);
    // feed-forward curvature (signed for the motion direction); an Ackermann base previews the next curvature change
    // (Reeds–Shepp / Hybrid A* paths switch curvature instantly): its wheels need T = |Δδ|/rate to steer there, which
    // the speed law below makes a transition of at most TR mm (v ≤ TR/T at the change); the feed-forward switches
    // T/2 before the change, so the transition is centred on it
    let kff = dir * kp, previewing = false;
    const steerRate = Math.max(1, k.maxSteerRate ?? 60) * DEG;
    const TR = 100;
    if (k.drive === 'ackermann') {
      const L = k.wheelBase, dNowFF = Math.atan(L * dir * kp), vNow = Math.abs(robot.state.v);
      let s = segLen * (1 - bestT);
      for (let i = bestI + 1; i < leg.b && s < TR; i++) {
        const sl = Math.hypot(path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1]);
        const ki = sl > 1 ? (dir * wrap180(path[i + 1][2] - path[i][2]) * DEG) / sl : 0;
        if (Math.abs(Math.atan(L * ki) - dNowFF) > 2 * DEG) {
          const T = Math.abs(Math.atan(L * ki) - robot.state.steer * DEG) / steerRate;
          if (s <= 0.5 * T * Math.min(vNow, TR / Math.max(T, 1e-6))) { kff = ki; previewing = true; }
          break;
        }
        s += sl;
      }
    }
    const kappa = kff - eyc / (Ld * Ld) - dir * (2 / Ld) * eth;
    // speed: stop at the leg end (cusp / goal), slow in curvature, never below a creep speed
    const vlim = dir > 0 ? vcap : Math.min(vcap, k.maxReverseSpeed);
    let v = Math.min(vlim, Math.sqrt(2 * decel * Math.max(0, rem - legTol * 0.3)) + 20);
    const kAbs = Math.max(Math.abs(kappa), Math.abs(kp));
    if (kAbs > 1e-6) v = Math.min(v, Math.sqrt(1500 / kAbs)); // lateral acceleration ~1.5 m/s²
    // Nav2 regulated pure pursuit (navigation method of the robot's stack): v·r/r_min on curves tighter than r_min
    const rpp = regulatedMinRadius(robot);
    if (rpp !== null) {
      if (Math.abs(kp) > 1e-9 && 1 / Math.abs(kp) < rpp) v = Math.min(v, (vlim * (1 / Math.abs(kp))) / rpp);
      // anticipate tighter curves ahead (braking distance): v ≤ sqrt(v_curve² + 2·a·s)
      let s = segLen * (1 - bestT);
      for (let i = bestI + 1; i < leg.b && s < 4000; i++) {
        const sl = Math.hypot(path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1]);
        const ki = sl > 1 ? Math.abs(wrap180(path[i + 1][2] - path[i][2]) * DEG) / sl : 0;
        if (ki > 1e-9 && 1 / ki < rpp) v = Math.min(v, Math.sqrt(((vlim / ki) / rpp) ** 2 + 2 * decel * s));
        s += sl;
      }
    }
    if (opts.slowdownDistance !== undefined && last && rem < opts.slowdownDistance) v = Math.min(v, Math.max(150, vlim * (rem / opts.slowdownDistance)));
    let creep = 25;
    if (k.drive === 'ackermann') {
      // steering-rate-aware speed: a curvature change ahead needing a steering change Δδ is crossed in a transition
      // v·Δδ/rate long (centred on the change by the preview); it is kept ≤ TR (braking-distance aware:
      // v ≤ sqrt(v_change² + 2·a·(s − TR/2))), which leaves a heading error of about Δκ·TR/4 for the feedback to remove
      const rate = steerRate, L = k.wheelBase;
      const steerMax = Math.min(k.maxSteer, Math.atan(L / Math.max(1, minTurnRadius(k))) * RAD) * DEG;
      const dNow = Math.abs(clamp(Math.atan(L * kappa), -steerMax, steerMax) - robot.state.steer * DEG);
      let s = segLen * (1 - bestT);
      for (let i = bestI + 1; i < leg.b && s < 6000; i++) {
        const sl = Math.hypot(path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1]);
        const ki = sl > 1 ? (dir * wrap180(path[i + 1][2] - path[i][2]) * DEG) / sl : 0;
        const need = Math.abs(clamp(Math.atan(L * ki), -steerMax, steerMax) - robot.state.steer * DEG);
        if (need > 2 * DEG) v = Math.min(v, Math.sqrt(((TR * rate) / need) ** 2 + 2 * decel * Math.max(0, s - 0.5 * TR)));
        s += sl;
      }
      if (!previewing && dNow > 2 * DEG) {
        // the wheels lag the curvature needed now (leg start after a cusp, a disturbance): steer at standstill first
        // when starting, else slow down so the lag lasts at most TR
        creep = 0;
        if (dNow > 4 * DEG && Math.abs(robot.state.v) < 50) v = 0;
        else v = Math.min(v, (TR * rate) / dNow);
      }
    }
    v = Math.max(v, creep);
    // do not start a reverse leg while still rolling forward (and vice versa)
    if (robot.state.v * dir < -30) return { v: 0, omega: 0, steer: Math.atan(k.wheelBase * kappa) * RAD };
    const vs = dir * v;
    return { v: vs, omega: vs * kappa * RAD, steer: Math.atan(k.wheelBase * kappa) * RAD };
  }
  return null;
}

function holonomicCommand(robot: MobileRobot, path: number[][], pose: { x: number; y: number; theta: number }, opts: PurePursuitOptions, vcap: number, goalTol: number, headTol: number): VelocityCommand | null {
  const k = robot.kin, st = robot.state;
  const goal = path[path.length - 1];
  const goalTh = goal.length >= 3 ? goal[2] : pose.theta;
  // aerial: take off to the cruise altitude before moving, hold it while flying
  let vz = 0;
  if (k.drive === 'aerial') {
    const zt = st.zTarget ?? k.cruiseAltitude ?? 5000;
    const ez = zt - (st.z ?? 0);
    vz = clamp(ez * 1.5, -(k.maxClimbRate ?? 1500), k.maxClimbRate ?? 1500);
    if ((st.z ?? 0) < Math.min(zt * 0.8, zt - 300)) return { v: 0, vy: 0, omega: 0, vz };
  }
  const dGoal = Math.hypot(goal[0] - pose.x, goal[1] - pose.y);
  const eth = wrap180(goalTh - pose.theta);
  if (dGoal < goalTol && Math.abs(eth) < headTol) return null;
  // closest point and lookahead target
  let best = st.pathIndex, bd = Infinity;
  for (let i = st.pathIndex; i < path.length; i++) {
    const d = Math.hypot(path[i][0] - pose.x, path[i][1] - pose.y);
    if (d < bd) { bd = d; best = i; }
    if (d > bd + 3000) break;
  }
  st.pathIndex = best;
  const L = opts.lookahead ?? Math.max(300, k.footprint[0] * 0.5);
  let target = goal;
  for (let i = best; i < path.length - 1; i++) { const pt = circleSegment([pose.x, pose.y], L, path[i], path[i + 1]); if (pt) { target = pt; break; } }
  if (Math.hypot(target[0] - pose.x, target[1] - pose.y) > L * 1.5) target = path[Math.min(path.length - 1, best + 1)];
  const dx = target[0] - pose.x, dy = target[1] - pose.y, dn = Math.hypot(dx, dy) || 1;
  const decel = Math.max(100, k.maxAccel * 0.8);
  let v = Math.min(vcap, Math.sqrt(2 * decel * Math.max(0, dGoal - goalTol * 0.3)) + 15);
  if (dGoal < goalTol) v = 0;
  const wx = (dx / dn) * v, wy = (dy / dn) * v;
  const th = pose.theta * DEG;
  const bx = Math.cos(th) * wx + Math.sin(th) * wy, by = -Math.sin(th) * wx + Math.cos(th) * wy;
  const thTarget = path[Math.min(path.length - 1, best + 1)].length >= 3 ? path[Math.min(path.length - 1, best + 1)][2] : goalTh;
  const e = wrap180((dGoal < 1000 ? goalTh : thTarget) - pose.theta);
  const omega = clamp(e * 2, -k.maxYawRate, k.maxYawRate);
  return { v: bx, vy: by, omega, vz };
}

/** Step a mobile robot along its current path. Returns true when it is still moving. */
export function stepMobile(robot: MobileRobot, dt: number, opts: PurePursuitOptions = {}): boolean {
  if (robot.state.status === 'charging') {
    robot.battery.levelWh = Math.min(robot.battery.capacityWh, robot.battery.levelWh + (robot.battery.chargeW * dt) / 3600);
    return false;
  }
  if (!robot.state.path) {
    const st = robot.state;
    if (Math.abs(st.v) > 1 || Math.abs(st.vy ?? 0) > 1 || Math.abs(st.omega) > 0.5 || Math.abs(st.vz ?? 0) > 1) integrate(robot, { v: 0, omega: 0, vy: 0, vz: 0 }, dt);
    else robot.battery.levelWh = Math.max(0, robot.battery.levelWh - (powerDraw(robot) * dt) / 3600);
    return false;
  }
  // legacy 2-D paths written directly into the state are converted on first use
  if (!isPosePath(robot.state.path)) { const p = robot.state.path; robot.state.path = toPosePath(robot, p.slice(Math.max(0, robot.state.pathIndex))); robot.state.pathIndex = 0; }
  const cmd = trackPath(robot, opts);
  if (!cmd) {
    robot.state.path = null;
    robot.state.v = 0;
    robot.state.vy = 0;
    robot.state.omega = 0;
    if (robot.state.status === 'moving') robot.state.status = 'idle';
    robot.syncPoseFromState();
    return false;
  }
  integrate(robot, cmd, dt);
  return true;
}

/** Estimated travel time for a path (s): trapezoidal speed profile, reverse speed on reverse legs, rotations. */
export function estimateTravelTime(robot: MobileRobot, path: number[][], speed = robot.kin.maxSpeed): number {
  const k = robot.kin;
  const v = Math.max(1, Math.min(speed, k.maxSpeed));
  const seg = (len: number, vm: number) => {
    const tAcc = vm / k.maxAccel, dAcc = 0.5 * k.maxAccel * tAcc * tAcc;
    return 2 * dAcc > len ? 2 * Math.sqrt(len / k.maxAccel) : 2 * tAcc + (len - 2 * dAcc) / vm;
  };
  if (!isPosePath(path)) {
    const len = pathLength(path);
    const turns = isHolonomic(k) ? 0 : Math.max(0, path.length - 2) * (k.drive === 'ackermann' ? 2.5 : 1.5);
    return seg(len, v) + turns;
  }
  let t = 0;
  for (const leg of pathLegs(path)) {
    if (leg.dir === 0) { t += Math.abs(wrap180(path[leg.b][2] - path[leg.a][2])) / Math.max(1, k.maxYawRate * 0.6) + 0.5; continue; }
    let len = 0;
    for (let i = leg.a; i < leg.b; i++) len += Math.hypot(path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1]);
    t += seg(len, leg.dir > 0 ? v : Math.min(v, k.maxReverseSpeed));
  }
  return t;
}
