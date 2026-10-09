/** Mobile robot kinematic models, pure-pursuit path tracking and a fleet-level mobile simulator. */
import { MobileRobot } from './items';
import { DEG, RAD } from '../core/math/pose';
import { pathLength } from './planner';

export interface VelocityCommand {
  v: number; // mm/s
  omega: number; // deg/s
}

/** Integrate the robot state for dt seconds with the given command, respecting kinematic limits. */
export function integrate(robot: MobileRobot, cmd: VelocityCommand, dt: number): void {
  const k = robot.kin;
  const st = robot.state;
  const vTarget = Math.max(-k.maxReverseSpeed, Math.min(k.maxSpeed, cmd.v));
  const dv = vTarget - st.v;
  const maxDv = k.maxAccel * dt;
  st.v += Math.max(-maxDv, Math.min(maxDv, dv));
  let omega = Math.max(-k.maxYawRate, Math.min(k.maxYawRate, cmd.omega));
  if (k.drive === 'ackermann' || k.minTurnRadius > 0) {
    // limit curvature by min turning radius
    const rMin = k.minTurnRadius || k.wheelBase / Math.tan(k.maxSteer * DEG);
    const maxOmega = (Math.abs(st.v) / rMin) * RAD;
    omega = Math.max(-maxOmega, Math.min(maxOmega, omega));
    st.steer = Math.abs(st.v) > 1 ? Math.atan((omega * DEG * k.wheelBase) / st.v) * RAD : 0;
  }
  st.omega = omega;
  const th = st.theta * DEG;
  st.x += st.v * Math.cos(th) * dt;
  st.y += st.v * Math.sin(th) * dt;
  st.theta = ((st.theta + omega * dt + 540) % 360) - 180;
  st.odometer += Math.abs(st.v) * dt;
  // battery
  const draw = robot.battery.idleW + (Math.abs(st.v) / 1000) * robot.battery.drivePerMps;
  robot.battery.levelWh = Math.max(0, robot.battery.levelWh - (draw * dt) / 3600);
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
}

/** Compute a pure-pursuit velocity command for the robot's current path. Returns null when the path is done. */
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
  // advance path index to the closest segment ahead
  let best = st.pathIndex, bd = Infinity;
  for (let i = st.pathIndex; i < path.length; i++) {
    const d = Math.hypot(path[i][0] - st.x, path[i][1] - st.y);
    if (d < bd) { bd = d; best = i; }
    if (d > bd + 2 * L) break;
  }
  st.pathIndex = best;
  robot.state.pathIndex = best;
  // lookahead point
  let target = goal;
  for (let i = best; i < path.length - 1; i++) {
    const a = path[i], b = path[i + 1];
    const pt = circleSegment([st.x, st.y], L, a, b);
    if (pt) { target = pt; break; }
  }
  if (Math.hypot(target[0] - st.x, target[1] - st.y) > L * 1.5 && dGoal > L) {
    // far from path: head to the closest point first
    target = path[best];
  }
  const th = st.theta * DEG;
  const dx = target[0] - st.x, dy = target[1] - st.y;
  const lx = Math.cos(th) * dx + Math.sin(th) * dy;
  const ly = -Math.sin(th) * dx + Math.cos(th) * dy;
  const ld = Math.hypot(lx, ly) || 1;
  const curvature = (2 * ly) / (ld * ld); // 1/mm
  const heading = Math.atan2(ly, lx);
  let v = vmax;
  const slow = opts.slowdownDistance ?? 1500;
  if (dGoal < slow) v = Math.max(150, vmax * (dGoal / slow));
  // slow down in sharp turns
  v *= Math.max(0.2, 1 - Math.abs(heading) / Math.PI);
  let omega = curvature * v * RAD;
  if (Math.abs(heading) > 60 * DEG && robot.kin.drive !== 'ackermann') {
    // rotate in place for differential drives
    v = 0;
    omega = Math.sign(heading) * robot.kin.maxYawRate * 0.7;
  }
  return { v, omega };
}

function circleSegment(c: number[], r: number, a: number[], b: number[]): number[] | null {
  const dx = b[0] - a[0], dy = b[1] - a[1];
  const fx = a[0] - c[0], fy = a[1] - c[1];
  const A = dx * dx + dy * dy, B = 2 * (fx * dx + fy * dy), C = fx * fx + fy * fy - r * r;
  const disc = B * B - 4 * A * C;
  if (disc < 0 || A === 0) return null;
  const sq = Math.sqrt(disc);
  const t2 = (-B + sq) / (2 * A);
  if (t2 >= 0 && t2 <= 1) return [a[0] + t2 * dx, a[1] + t2 * dy];
  return null;
}

/** Assign a path to a robot and start following. */
export function followPath(robot: MobileRobot, path: number[][]): void {
  robot.state.path = path.map((p) => [p[0], p[1]]);
  robot.state.pathIndex = 0;
  robot.state.status = 'moving';
}

/** Step a mobile robot along its current path. Returns true when it is still moving. */
export function stepMobile(robot: MobileRobot, dt: number, opts: PurePursuitOptions = {}): boolean {
  if (robot.state.status === 'charging') {
    robot.battery.levelWh = Math.min(robot.battery.capacityWh, robot.battery.levelWh + (robot.battery.chargeW * dt) / 3600);
    return false;
  }
  if (!robot.state.path) {
    if (Math.abs(robot.state.v) > 1) integrate(robot, { v: 0, omega: 0 }, dt);
    else {
      robot.battery.levelWh = Math.max(0, robot.battery.levelWh - (robot.battery.idleW * dt) / 3600);
    }
    return false;
  }
  const cmd = purePursuit(robot, opts);
  if (!cmd) {
    // arrived: align heading with last segment if provided
    robot.state.path = null;
    robot.state.v = 0;
    robot.state.omega = 0;
    if (robot.state.status === 'moving') robot.state.status = 'idle';
    robot.syncPoseFromState();
    return false;
  }
  integrate(robot, cmd, dt);
  return true;
}

/** Estimated travel time for a path (s) with trapezoidal approximation. */
export function estimateTravelTime(robot: MobileRobot, path: number[][], speed = robot.kin.maxSpeed): number {
  const len = pathLength(path);
  const v = Math.min(speed, robot.kin.maxSpeed);
  const tAcc = v / robot.kin.maxAccel;
  const dAcc = 0.5 * robot.kin.maxAccel * tAcc * tAcc;
  // add a turn penalty per vertex
  const turns = Math.max(0, path.length - 2) * 1.5;
  if (2 * dAcc > len) return 2 * Math.sqrt(len / robot.kin.maxAccel) + turns;
  return 2 * tAcc + (len - 2 * dAcc) / v + turns;
}
