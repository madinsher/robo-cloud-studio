/**
 * Goal navigation for a mobile robot on the station's collision world (contract I8): plan a footprint- and
 * drive-aware pose path (Hybrid A* / Reeds–Shepp for Ackermann, footprint grid A* for differential / tracked /
 * legged / omni, straight flight for aerial), then simulate the drive model + path tracker on a scratch copy of the
 * robot to get the real trajectory (duration and pose at any time). Used by control-model `goto`, by the program
 * runtime for Navigate / Follow path instructions (see ./program_motion) and by precise docking.
 *
 * Poses are in the mobile robot's parent frame (x, y in mm, heading in degrees); planning happens in the station frame.
 */
import { Station, Item } from '../core/items/item';
import { MobileRobot } from './items';
import { followPath, stepMobile, toPosePath, estimateTravelTime, isPosePath } from './controller';
import { planForRobot } from './planner';
import { stationWorld } from './world';

export interface NavGoal {
  x: number; y: number;
  /** Final heading (deg); free when undefined. */
  heading?: number;
  /** Position tolerance (mm, default 100; 20 for precise docking). */
  tolerance?: number;
  /** Arrive driving backwards along the goal heading (undocking / reversing into a bay). */
  reverse?: boolean;
  /** Precision approach: straight final leg along the goal heading, ≤ 20 mm / 1° at the goal. */
  precise?: boolean;
  /** Speed cap (mm/s). */
  speed?: number;
}

export interface NavResult {
  ok: boolean;
  error?: string;
  /** Simulated duration (s) — never 0 for a real displacement. */
  duration: number;
  sample: (t: number) => { x: number; y: number; heading: number };
  /** Planned pose path (parent frame) and its length (mm). */
  path?: number[][];
  length?: number;
  /** Position / heading error at the end of the simulated drive (mm / deg). */
  finalError?: number;
  finalHeadingError?: number;
}

export interface NavOptions {
  /** Start pose (parent frame); default: the robot's state. */
  start?: { x: number; y: number; heading: number };
  /** Integration step (s, default 0.05). */
  dt?: number;
  /** Give up after this simulated time (s). */
  maxTime?: number;
  /** Clearance kept to obstacles while planning (mm, default 120). */
  margin?: number;
}

const wrap180 = (a: number) => ((((a + 180) % 360) + 360) % 360) - 180;

/** Planar transform between the robot's parent frame and the station frame. */
function frames(mobile: Item) {
  const p = mobile.parent;
  const a = p && p.parent ? p.poseAbs() : null; // the station itself is the identity
  const c = a ? a[0] : 1, s = a ? a[1] : 0, tx = a ? a[12] : 0, ty = a ? a[13] : 0;
  const rot = Math.atan2(s, c) * 180 / Math.PI;
  return {
    toWorld: (x: number, y: number, h = 0) => ({ x: c * x - s * y + tx, y: s * x + c * y + ty, h: h + rot }),
    toParent: (x: number, y: number, h = 0) => { const dx = x - tx, dy = y - ty; return { x: c * dx + s * dy, y: -s * dx + c * dy, h: h - rot }; },
  };
}

/** A scratch copy of the robot (kinematics, battery, footprint) that can be driven without touching the scene. */
function scratchOf(mobile: MobileRobot, x: number, y: number, theta: number): MobileRobot {
  const s = new MobileRobot(`${mobile.name} (plan)`);
  s.kin = JSON.parse(JSON.stringify(mobile.kin));
  s.battery = { ...mobile.battery };
  s.state = { ...s.state, x, y, theta, z: mobile.state.z, v: 0, omega: 0, steer: 0, vy: 0, vz: 0, status: 'idle', path: null, pathIndex: 0, taskId: null };
  return s;
}

/** Drive a world-frame pose path with the robot's drive model; returns the sampled trajectory (world frame). */
function simulate(robot: MobileRobot, path: number[][], o: { speed?: number; tol: number; headTol: number; dt: number; maxTime: number }) {
  followPath(robot, path);
  const T: number[] = [0], X: number[] = [robot.state.x], Y: number[] = [robot.state.y], H: number[] = [robot.state.theta];
  let t = 0;
  while (t < o.maxTime) {
    const moving = stepMobile(robot, o.dt, { speed: o.speed, goalTolerance: o.tol, headingTolerance: o.headTol });
    t += o.dt;
    T.push(t); X.push(robot.state.x); Y.push(robot.state.y); H.push(robot.state.theta);
    if (!moving) break;
  }
  return { T, X, Y, H, timedOut: t >= o.maxTime };
}

function sampler(tr: { T: number[]; X: number[]; Y: number[]; H: number[] }, toParent: (x: number, y: number, h?: number) => { x: number; y: number; h: number }) {
  const { T, X, Y, H } = tr;
  return (t: number) => {
    if (t <= 0) { const p = toParent(X[0], Y[0], H[0]); return { x: p.x, y: p.y, heading: p.h }; }
    const n = T.length - 1;
    if (t >= T[n]) { const p = toParent(X[n], Y[n], H[n]); return { x: p.x, y: p.y, heading: p.h }; }
    let lo = 0, hi = n;
    while (hi - lo > 1) { const m = (lo + hi) >> 1; if (T[m] <= t) lo = m; else hi = m; }
    const f = (t - T[lo]) / Math.max(1e-9, T[hi] - T[lo]);
    const p = toParent(X[lo] + (X[hi] - X[lo]) * f, Y[lo] + (Y[hi] - Y[lo]) * f, H[lo] + wrap180(H[hi] - H[lo]) * f);
    return { x: p.x, y: p.y, heading: p.h };
  };
}

function pathLen(p: number[][]): number { let l = 0; for (let i = 1; i < p.length; i++) l += Math.hypot(p[i][0] - p[i - 1][0], p[i][1] - p[i - 1][1]); return l; }

/** Execute a world-frame pose path on a scratch robot and package the result. */
function run(mobile: MobileRobot, scratch: MobileRobot, path: number[][], goal: { x: number; y: number; heading?: number } | null, o: { speed?: number; tol: number; headTol: number; dt: number; maxTime?: number }, f: ReturnType<typeof frames>): NavResult {
  const maxTime = o.maxTime ?? Math.max(60, estimateTravelTime(scratch, path, o.speed ?? scratch.kin.maxSpeed) * 4 + 60);
  const tr = simulate(scratch, path, { speed: o.speed, tol: o.tol * 0.8, headTol: o.headTol, dt: o.dt, maxTime });
  const n = tr.T.length - 1;
  const end = goal ?? { x: path[path.length - 1][0], y: path[path.length - 1][1], heading: undefined };
  const err = Math.hypot(tr.X[n] - end.x, tr.Y[n] - end.y);
  const herr = end.heading !== undefined ? Math.abs(wrap180(tr.H[n] - end.heading)) : 0;
  const ok = !tr.timedOut && err <= o.tol && herr <= Math.max(3, o.headTol * 2);
  void mobile;
  return {
    ok,
    error: ok ? undefined : tr.timedOut ? `Drive did not reach the goal within ${maxTime.toFixed(0)} s (error ${err.toFixed(0)} mm)` : `Stopped ${err.toFixed(0)} mm / ${herr.toFixed(1)}° from the goal`,
    duration: Math.max(o.dt, tr.T[n]),
    sample: sampler(tr, f.toParent),
    path: path.map((p) => { const q = f.toParent(p[0], p[1], p[2]); return [q.x, q.y, q.h, p[3]]; }),
    length: pathLen(path),
    finalError: err,
    finalHeadingError: herr,
  };
}

/**
 * Plan and simulate a drive to `goal` (contract I8). Uses the station's map and registered obstacles (crop trunks /
 * canopies), the robot's footprint and drive model. `reverse` / `precise` add a straight final leg along the goal
 * heading (driven backwards for `reverse`) so the robot docks within 20 mm and 1°.
 */
export function planAndFollow(station: Station, mobile: MobileRobot, goal: NavGoal, o: NavOptions = {}): NavResult {
  const f = frames(mobile);
  const s0 = o.start ?? { x: mobile.state.x, y: mobile.state.y, heading: mobile.state.theta };
  const sw = f.toWorld(s0.x, s0.y, s0.heading);
  const gw = f.toWorld(goal.x, goal.y, goal.heading ?? 0);
  const goalHeading = goal.heading !== undefined ? wrap180(gw.h) : undefined;
  const precise = !!goal.precise || !!goal.reverse;
  const tol = goal.tolerance ?? (precise ? 20 : 100);
  const headTol = precise ? 0.8 : 1.5;
  const dt = o.dt ?? 0.05;
  const scratch = scratchOf(mobile, sw.x, sw.y, wrap180(sw.h));
  const world = stationWorld(station);
  const empty = !world.map && !world.circles.length;
  const [L, W, H] = mobile.kin.footprint;
  const margin = o.margin ?? 120;
  const plan = (gx: number, gy: number, gh?: number): number[][] | { error: string } => {
    if (empty) return toPosePath(scratch, [[sw.x, sw.y], [gx, gy]], { goalHeading: gh, allowReverse: true });
    const r = planForRobot(scratch, { x: sw.x, y: sw.y, theta: scratch.state.theta }, { x: gx, y: gy }, { world, margin, goalHeading: gh });
    return r.ok ? r.path : { error: r.error ?? 'no path' };
  };
  let path: number[][] | null = null;
  let error = '';
  if (precise && goalHeading !== undefined && mobile.kin.drive !== 'aerial') {
    // pre-pose on the goal axis, then a straight docking leg (forward, or backwards when reversing in)
    const u = [Math.cos(goalHeading * Math.PI / 180), Math.sin(goalHeading * Math.PI / 180)];
    const D = Math.max(1.2 * L, 1500);
    const sgn = goal.reverse ? -1 : 1;
    const pre = [gw.x - u[0] * D * sgn, gw.y - u[1] * D * sgn];
    let legFree = true;
    if (!empty) for (let s = 0; s <= D; s += 100) { const x = pre[0] + u[0] * s * sgn, y = pre[1] + u[1] * s * sgn; if (world.clearance(x, y, goalHeading, L, W, H, margin * 0.5) < margin * 0.5) { legFree = false; break; } }
    if (legFree) {
      const head = Math.hypot(pre[0] - sw.x, pre[1] - sw.y) < 30 && Math.abs(wrap180(goalHeading - scratch.state.theta)) < 1 ? [[sw.x, sw.y, scratch.state.theta, 1]] : plan(pre[0], pre[1], goalHeading);
      if (Array.isArray(head)) {
        const n = Math.max(1, Math.ceil(D / 100));
        const leg: number[][] = [];
        for (let i = 1; i <= n; i++) leg.push([pre[0] + u[0] * D * sgn * (i / n), pre[1] + u[1] * D * sgn * (i / n), goalHeading, sgn]);
        path = [...head, ...leg];
      } else error = head.error;
    } else error = 'Docking leg along the goal heading is obstructed';
  }
  if (!path) {
    const p = plan(gw.x, gw.y, goalHeading);
    if (!Array.isArray(p)) return { ok: false, error: error ? `${error}; ${p.error}` : p.error, duration: 0, sample: () => ({ x: s0.x, y: s0.y, heading: s0.heading }) };
    path = p;
  }
  if (!isPosePath(path)) path = toPosePath(scratch, path, { goalHeading });
  return run(mobile, scratch, path, { x: gw.x, y: gw.y, heading: goalHeading }, { speed: goal.speed, tol, headTol, dt, maxTime: o.maxTime }, f);
}

/**
 * Follow a polyline (parent frame) with the robot's drive: converted to an executable pose path (turn-then-drive,
 * Reeds–Shepp joins for Ackermann, holonomic tracking for omni) and simulated.
 */
export function followPolyline(station: Station, mobile: MobileRobot, points: number[][], o: NavOptions & { speed?: number; tolerance?: number } = {}): NavResult {
  void station;
  const f = frames(mobile);
  const s0 = o.start ?? { x: mobile.state.x, y: mobile.state.y, heading: mobile.state.theta };
  const sw = f.toWorld(s0.x, s0.y, s0.heading);
  const scratch = scratchOf(mobile, sw.x, sw.y, wrap180(sw.h));
  const pts = points.map((p) => { const q = f.toWorld(p[0], p[1]); return [q.x, q.y]; });
  if (!pts.length) return { ok: false, error: 'Empty path', duration: 0, sample: () => ({ x: s0.x, y: s0.y, heading: s0.heading }) };
  const path = toPosePath(scratch, [[sw.x, sw.y], ...pts], { allowReverse: false });
  const last = pts[pts.length - 1];
  return run(mobile, scratch, path, { x: last[0], y: last[1] }, { speed: o.speed, tol: o.tolerance ?? 150, headTol: 1.5, dt: o.dt ?? 0.05, maxTime: o.maxTime }, f);
}
