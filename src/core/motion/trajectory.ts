import { Mat4, slerpPose, multiply, invert, transformPoint, poseFromZ } from '../math/pose';
import { Robot } from '../items/robot';
import { planSingle, densify, MotionSettings } from './planner';

export interface TrajectorySample {
  /** Time since motion start (s). */
  t: number;
  joints: number[];
  /** TCP pose relative to the robot base. */
  pose: Mat4;
}

export interface Trajectory {
  samples: TrajectorySample[];
  duration: number;
  /** Cartesian TCP length (mm). */
  length: number;
  ok: boolean;
  error?: string;
}

/**
 * Trapezoidal (or triangular) velocity profile position at time t for total distance d,
 * max velocity v, acceleration a. Returns [s(t), duration].
 */
export function trapezoid(d: number, v: number, a: number): { duration: number; s: (t: number) => number } {
  if (d <= 1e-9) return { duration: 0, s: () => 0 };
  const tAcc = v / a;
  const dAcc = 0.5 * a * tAcc * tAcc;
  if (2 * dAcc >= d) {
    // triangular
    const tPeak = Math.sqrt(d / a);
    const duration = 2 * tPeak;
    return {
      duration,
      s: (t: number) => {
        t = Math.max(0, Math.min(duration, t));
        if (t < tPeak) return 0.5 * a * t * t;
        const tr = duration - t;
        return d - 0.5 * a * tr * tr;
      },
    };
  }
  const tFlat = (d - 2 * dAcc) / v;
  const duration = 2 * tAcc + tFlat;
  return {
    duration,
    s: (t: number) => {
      t = Math.max(0, Math.min(duration, t));
      if (t < tAcc) return 0.5 * a * t * t;
      if (t < tAcc + tFlat) return dAcc + v * (t - tAcc);
      const tr = duration - t;
      return d - 0.5 * a * tr * tr;
    },
  };
}

const settingsOf = (speedLinear: number, speedJoints: number, accelLinear: number, accelJoints: number): MotionSettings => ({ speedLinear, speedJoints, accelLinear, accelJoints, rounding: -1 });

/** Joint-space (MoveJ) trajectory: synchronised profile on the slowest joint (per-joint velocity limits honoured). */
export function planMoveJ(robot: Robot, q0: number[], q1: number[], speedJoints: number, accelJoints: number, dt = 0.02, tool: Mat4 = robot.poseTool()): Trajectory {
  const m = planSingle(robot, q0, { data: { kind: 'move', moveType: 'MoveJ', targetId: null, joints: q1.slice(0, robot.dof) }, tool, settings: settingsOf(robot.motion.speedLinear, speedJoints, robot.motion.accelLinear, accelJoints) });
  const tr = densify(m.trajectory, dt);
  if (!robot.jointsValid(q1)) { tr.ok = false; tr.error = 'Target joints outside limits'; }
  return tr;
}

/** Cartesian linear (MoveL) trajectory (adaptive IK knots, densified every dt). Poses are TCP relative to the robot base. */
export function planMoveL(robot: Robot, q0: number[], p1: Mat4, speedLinear: number, accelLinear: number, dt = 0.02, speedJoints = 180, tool: Mat4 = robot.poseTool()): Trajectory {
  const m = planSingle(robot, q0, { data: { kind: 'move', moveType: 'MoveL', targetId: null, pose: Array.from(p1) }, tool, settings: settingsOf(speedLinear, speedJoints, accelLinear, speedJoints * 4) });
  return densify(m.trajectory, dt);
}

/** Circular (MoveC) trajectory through a via pose to a final pose (TCP poses relative to the robot base). */
export function planMoveC(robot: Robot, q0: number[], pVia: Mat4, p1: Mat4, speedLinear: number, accelLinear: number, dt = 0.02, tool: Mat4 = robot.poseTool()): Trajectory {
  const m = planSingle(robot, q0, { data: { kind: 'move', moveType: 'MoveC', targetId: null, pose: Array.from(p1), viaPose: Array.from(pVia) }, tool, settings: settingsOf(speedLinear, robot.motion.speedJoints, accelLinear, robot.motion.accelJoints) });
  return densify(m.trajectory, dt);
}

/** Sample a trajectory at time t (exact profile when the trajectory has an evaluator, else linear interpolation). */
export function sampleAt(tr: Trajectory, t: number): TrajectorySample {
  const ev = (tr as { eval?: (t: number) => TrajectorySample }).eval;
  if (typeof ev === 'function' && tr.samples.length) return ev.call(tr, t);
  const s = tr.samples;
  if (s.length === 0) throw new Error('empty trajectory');
  if (t <= 0) return s[0];
  if (t >= tr.duration) return s[s.length - 1];
  let lo = 0, hi = s.length - 1;
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1;
    if (s[mid].t <= t) lo = mid;
    else hi = mid;
  }
  const a = s[lo], b = s[hi];
  const f = b.t > a.t ? (t - a.t) / (b.t - a.t) : 0;
  return { t, joints: a.joints.map((v, i) => v + (b.joints[i] - v) * f), pose: slerpPose(a.pose, b.pose, f) };
}

export { transformPoint, multiply, invert, poseFromZ };
