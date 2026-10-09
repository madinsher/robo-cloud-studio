import { Mat4, slerpPose, getPos, distance, rotationAngle, multiply, invert, transformPoint, sub, add, scale, norm, cross, normalize, poseFromZ, RAD, dot } from '../math/pose';
import { Robot } from '../items/robot';

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

/** Joint-space (MoveJ) trajectory: synchronised trapezoidal profile on the slowest joint. */
export function planMoveJ(robot: Robot, q0: number[], q1: number[], speedJoints: number, accelJoints: number, dt = 0.02): Trajectory {
  const n = q0.length;
  const dq = q1.map((v, i) => v - q0[i]);
  const dmax = Math.max(...dq.map(Math.abs));
  const prof = trapezoid(dmax, speedJoints, accelJoints);
  const samples: TrajectorySample[] = [];
  let length = 0;
  let prevPos = getPos(robot.solveFK(q0));
  const steps = Math.max(1, Math.ceil(prof.duration / dt));
  for (let k = 0; k <= steps; k++) {
    const t = (k / steps) * prof.duration;
    const f = dmax > 1e-9 ? prof.s(t) / dmax : 1;
    const q = new Array(n);
    for (let i = 0; i < n; i++) q[i] = q0[i] + dq[i] * f;
    const pose = robot.solveFK(q);
    const p = getPos(pose);
    length += distance(p, prevPos);
    prevPos = p;
    samples.push({ t, joints: q, pose });
  }
  if (!robot.jointsValid(q1)) return { samples, duration: prof.duration, length, ok: false, error: 'Target joints outside limits' };
  return { samples, duration: prof.duration, length, ok: true };
}

/** Cartesian linear (MoveL) trajectory with IK at each sample. Poses are TCP relative to the robot base. */
export function planMoveL(robot: Robot, q0: number[], p1: Mat4, speedLinear: number, accelLinear: number, dt = 0.02, speedJoints = 180): Trajectory {
  const p0 = robot.solveFK(q0);
  const d = distance(getPos(p0), getPos(p1));
  const ang = rotationAngle(p0, p1);
  // duration bounded by linear speed and by a nominal rotational speed (deg/s) ~ speedJoints
  const profLin = trapezoid(d, speedLinear, accelLinear);
  const profRot = trapezoid(ang * RAD, speedJoints, speedJoints * 4);
  const duration = Math.max(profLin.duration, profRot.duration, 1e-6);
  const steps = Math.max(1, Math.ceil(duration / dt));
  const samples: TrajectorySample[] = [];
  let q = [...q0];
  let length = 0;
  let prevPos = getPos(p0);
  for (let k = 0; k <= steps; k++) {
    const t = (k / steps) * duration;
    // use the dominant profile normalised to [0,1]
    const f = profLin.duration >= profRot.duration ? (d > 1e-9 ? profLin.s((t / duration) * profLin.duration) / d : k / steps) : (ang > 1e-9 ? profRot.s((t / duration) * profRot.duration) / (ang * RAD) : k / steps);
    const pose = slerpPose(p0, p1, f);
    const r = robot.solveIK(pose, { seed: q, restarts: 2, maxIterations: 100, keepFirstSolution: true });
    if (!r.ok) {
      const near = r.posError < 1 && r.rotError < 0.01;
      return { samples, duration, length, ok: false, error: near ? `MoveL crosses a singularity at ${(f * 100).toFixed(1)}% (residual ${r.posError.toFixed(2)} mm) — start from a non-singular configuration or use MoveJ` : `MoveL unreachable at ${(f * 100).toFixed(1)}% (pos err ${r.posError.toFixed(2)} mm)` };
    }
    // detect joint jumps (singularity / configuration flip)
    const jump = Math.max(...r.joints.map((v, i) => Math.abs(v - q[i])));
    if (k > 0 && jump > 45) return { samples, duration, length, ok: false, error: `Joint jump of ${jump.toFixed(0)} deg (singularity) at ${(f * 100).toFixed(0)}%` };
    q = r.joints;
    const p = getPos(pose);
    length += distance(p, prevPos);
    prevPos = p;
    samples.push({ t, joints: q, pose });
  }
  return { samples, duration, length, ok: true };
}

/** Circular (MoveC) trajectory through a via pose to a final pose. */
export function planMoveC(robot: Robot, q0: number[], pVia: Mat4, p1: Mat4, speedLinear: number, accelLinear: number, dt = 0.02): Trajectory {
  const p0 = robot.solveFK(q0);
  const A = getPos(p0), B = getPos(pVia), C = getPos(p1);
  // circumcircle of A,B,C
  const a = sub(A, C), b = sub(B, C);
  const axb = cross(a, b);
  const denom = 2 * dot(axb, axb);
  if (denom < 1e-9) return planMoveL(robot, q0, p1, speedLinear, accelLinear, dt); // collinear
  const term = scale(cross(sub(scale(b, dot(a, a)), scale(a, dot(b, b))), axb), 1 / denom);
  const center = add(C, term);
  const radius = norm(term);
  const nrm = normalize(axb);
  const u = normalize(sub(A, center));
  const v = cross(nrm, u);
  const angleOf = (P: [number, number, number]) => {
    const r = sub(P, center);
    return Math.atan2(dot(r, v), dot(r, u));
  };
  let thB = angleOf(B), thC = angleOf(C);
  if (thB < 0) thB += 2 * Math.PI;
  if (thC < 0) thC += 2 * Math.PI;
  if (thB > thC) {
    // going the other way round
    thC -= 2 * Math.PI;
  }
  const arc = Math.abs(thC) * radius;
  const prof = trapezoid(arc, speedLinear, accelLinear);
  const duration = Math.max(prof.duration, 1e-6);
  const steps = Math.max(2, Math.ceil(duration / dt));
  const samples: TrajectorySample[] = [];
  let q = [...q0];
  let length = 0, prevPos = A;
  for (let k = 0; k <= steps; k++) {
    const t = (k / steps) * duration;
    const f = arc > 1e-9 ? prof.s(t) / arc : k / steps;
    const th = thC * f;
    const pos = add(center, add(scale(u, radius * Math.cos(th)), scale(v, radius * Math.sin(th))));
    const pose = slerpPose(p0, p1, f);
    pose[12] = pos[0]; pose[13] = pos[1]; pose[14] = pos[2];
    const r = robot.solveIK(pose, { seed: q, restarts: 2, maxIterations: 100, keepFirstSolution: true });
    if (!r.ok) return { samples, duration, length, ok: false, error: `MoveC unreachable at ${(f * 100).toFixed(0)}%` };
    q = r.joints;
    length += distance(pos, prevPos);
    prevPos = pos;
    samples.push({ t, joints: q, pose });
  }
  return { samples, duration, length, ok: true };
}

/** Sample a trajectory at time t (linear interpolation of joints). */
export function sampleAt(tr: Trajectory, t: number): TrajectorySample {
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
