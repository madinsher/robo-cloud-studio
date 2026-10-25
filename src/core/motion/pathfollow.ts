/**
 * Curve / point following (RoboDK "Curve follow project" / "Point follow project"):
 * generates a program from object curves or points with the tool Z along the surface normal
 * (or a fixed direction), approach/retract moves, and optional tool orientation freedom.
 * Used for welding seams, glue beads, spraying along canopies, pruning cuts, milling.
 */
import { Station, SceneObject, Frame, Target } from '../items/item';
import { Robot } from '../items/robot';
import { Program } from '../items/program';
import { Mat4, poseFromZ, multiply, invert, transl, mul, normalize, sub, cross, norm, Vec3, transformPoint, transformDir, rotz, DEG } from '../math/pose';
import { solveIKWithCarrier, applyCombined, carrierOf } from '../kinematics/combined';

export interface FollowOptions {
  /** Approach/retract distance along -Z of the tool (mm). */
  approach?: number;
  /** Spacing between generated points along the curve (mm); 0 = use curve vertices. */
  step?: number;
  /** Tool Z direction: 'normal' (curve normal estimate), 'down' (-Z world), 'custom'. */
  zMode?: 'normal' | 'down' | 'custom';
  customZ?: Vec3;
  /** Rotate the tool about its Z axis by this angle (deg) — e.g. torch lead angle. */
  spin?: number;
  /** Allow IK to ignore rotation about tool Z (symmetric tools). */
  freeToolZ?: boolean;
  /** Speed along the path (mm/s). */
  speed?: number;
  /** Use MoveL between points (true) or MoveJ (false). */
  linear?: boolean;
  /** Name of the created program. */
  name?: string;
  /** Digital output to switch on while following (e.g. Arc, Spray). */
  io?: string;
  /** Optimise the rotation about the tool Z axis per point (RoboDK "tool orientation optimisation"). */
  optimizeToolZ?: boolean;
  /** Candidate spin step (deg) for the optimisation. */
  spinStep?: number;
  /** Keep the elbow/wrist configuration of the start (default true). */
  preferredConfig?: boolean;
  /** Carrier mechanism (rail/turntable) moved jointly with the arm (RoboDK "optimise external axes"). */
  carrier?: Robot | null;
}

export interface FollowResult {
  program: Program;
  points: number;
  unreachable: number;
}

/** Resample a polyline at a fixed step (mm). */
export function resampleCurve(points: number[][], step: number): number[][] {
  if (step <= 0 || points.length < 2) return points;
  const out: number[][] = [points[0]];
  let carry = 0;
  for (let i = 1; i < points.length; i++) {
    const a = points[i - 1], b = points[i];
    const seg = Math.hypot(b[0] - a[0], b[1] - a[1], (b[2] ?? 0) - (a[2] ?? 0));
    let s = step - carry;
    while (s <= seg) {
      const t = s / seg;
      out.push([a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, (a[2] ?? 0) + ((b[2] ?? 0) - (a[2] ?? 0)) * t]);
      s += step;
    }
    carry = seg - (s - step);
  }
  const last = points[points.length - 1];
  const l = out[out.length - 1];
  if (Math.hypot(last[0] - l[0], last[1] - l[1], (last[2] ?? 0) - (l[2] ?? 0)) > 1e-6) out.push(last);
  return out;
}

/** Generate a follow program for a curve (points in the object's frame, optional normals). */
export function generateCurveFollow(station: Station, robot: Robot, object: SceneObject, curve: { points: number[][]; normals?: number[][] }, opts: FollowOptions = {}): FollowResult {
  const approach = opts.approach ?? 50;
  const pts = opts.step ? resampleCurve(curve.points, opts.step) : curve.points;
  const objAbs = object.poseAbs();
  const base = robot.poseAbs();
  const frame = station.addChild(new Frame(`${object.name} path`));
  frame.setPoseAbs(objAbs);
  robot.setFrame(frame);
  const prog = station.addChild(new Program(opts.name ?? `Follow ${object.name}`));
  prog.setRobot(robot);
  prog.setSpeed(opts.speed ?? 50, 60);
  prog.setRounding(2);
  const home = frame.addChild(new Target('Home'));
  home.setJoints(robot.joints());
  home.setAsJointTarget();
  prog.addMoveJ(home);
  let q = robot.joints();
  let unreachable = 0, n = 0;
  const zOf = (i: number): Vec3 => {
    if (opts.zMode === 'custom' && opts.customZ) return normalize(opts.customZ);
    if (opts.zMode === 'down' || !curve.normals?.[i]) {
      if (opts.zMode !== 'normal' || !curve.normals?.[i]) return [0, 0, -1];
    }
    // tool Z points INTO the surface: opposite of the surface normal
    const nrm = curve.normals![Math.min(i, curve.normals!.length - 1)];
    return normalize([-nrm[0], -nrm[1], -nrm[2]]) as Vec3;
  };
  const poses: Array<{ abs: Mat4; local: Mat4 }> = [];
  for (let i = 0; i < pts.length; i++) {
    const p = pts[i];
    const pw = transformPoint(objAbs, [p[0], p[1], p[2] ?? 0]);
    const zLocal = zOf(i);
    const zw = transformDir(objAbs, zLocal);
    // X hint along the path direction
    const nxt = pts[Math.min(i + 1, pts.length - 1)], prv = pts[Math.max(i - 1, 0)];
    const dirLocal = normalize(sub([nxt[0], nxt[1], nxt[2] ?? 0], [prv[0], prv[1], prv[2] ?? 0]));
    const dirW = norm(dirLocal) > 1e-9 ? transformDir(objAbs, dirLocal) : [1, 0, 0] as Vec3;
    let abs = poseFromZ(pw, zw, dirW);
    if (opts.spin) abs = mul(abs, rotz(opts.spin * DEG));
    poses.push({ abs, local: multiply(invert(objAbs), abs) });
  }
  const ikOpts = { freeToolZ: opts.freeToolZ ?? false, restarts: 1, maxIterations: 80 };
  const spinStep = opts.spinStep ?? 30;
  const spins = opts.optimizeToolZ ? Array.from({ length: Math.round(360 / spinStep) }, (_, i) => -180 + i * spinStep) : [0];
  const carrier = opts.carrier ?? null;
  const carrierBase = carrier ? carrier.poseAbs() : null;
  /** Absolute TCP pose reached by a combined carrier + robot solution (robot state restored afterwards). */
  const reachedWithCarrier = (sol: { carrierJoints: number[]; robotJoints: number[] }): Mat4 => {
    const qc = carrier!.joints(), qr = robot.joints();
    applyCombined(carrier!, robot, sol);
    const abs = robot.poseTCPAbs();
    applyCombined(carrier!, robot, { carrierJoints: qc, robotJoints: qr });
    return abs;
  };
  /** Solve IK for an absolute pose, optionally over spin candidates and with the carrier. Returns joints or null. */
  const solve = (absPose: Mat4, seed: number[], first: boolean): { q: number[]; abs: Mat4; carrierQ?: number[] } | null => {
    let best: { q: number[]; abs: Mat4; carrierQ?: number[]; cost: number } | null = null;
    for (const spin of spins) {
      const cand = spin ? mul(absPose, rotz(spin * DEG)) : absPose;
      if (carrier && carrierBase) {
        const sol = solveIKWithCarrier(carrier, robot, multiply(invert(carrierBase), cand), { seed: [...carrier.joints(), ...seed], restarts: first ? 2 : 0, maxIterations: first ? 200 : 100, freeToolZ: ikOpts.freeToolZ });
        if (!sol.ok) continue;
        const cost = sol.robotJoints.reduce((a, v, i) => a + Math.abs(v - seed[i]), 0) + sol.carrierJoints.reduce((a, v, i) => a + Math.abs(v - carrier.joints()[i]) * 0.05, 0);
        if (!best || cost < best.cost) best = { q: sol.robotJoints, abs: ikOpts.freeToolZ ? reachedWithCarrier(sol) : cand, carrierQ: sol.carrierJoints, cost };
      } else {
        const r = robot.solveIK(multiply(invert(base), cand), { seed, ...ikOpts, ...(first ? { restarts: 6, maxIterations: 200 } : {}), keepFirstSolution: !(opts.preferredConfig ?? true) });
        if (!r.ok) continue;
        const cost = r.joints.reduce((a, v, i) => a + Math.abs(v - seed[i]), 0);
        // tool-axis freedom: store the pose the robot reaches (optimised rotation about tool Z), like RoboDK
        if (!best || cost < best.cost) best = { q: r.joints, abs: ikOpts.freeToolZ ? multiply(base, robot.solveFK(r.joints)) : cand, cost };
      }
      if (!opts.optimizeToolZ) break;
    }
    return best;
  };
  // targets carry the carrier joints as synchronised external axes (6+N joints, RoboDK convention) so the simulator
  // moves the carrier with the arm; only when the carrier is the robot's synchronised axis (its mount)
  const syncCarrier = !!carrier && carrierOf(robot) === carrier;
  const withExt = (q: number[], carrierQ?: number[]): number[] => (syncCarrier && carrierQ ? [...q, ...carrierQ] : q);
  const applyCarrier = (carrierQ?: number[]) => { if (carrier && carrierQ) applyCombined(carrier, robot, { carrierJoints: carrierQ, robotJoints: robot.joints() }); };
  // tool-axis freedom: aim each target's X axis like the last reached pose (no needless rotation about the tool axis,
  // so the wrist does not wind up along closed curves and run into its joint limit)
  let lastReached: Mat4 | null = null;
  const aimed = (abs: Mat4): Mat4 => {
    if (!ikOpts.freeToolZ || !lastReached || opts.optimizeToolZ) return abs;
    const z: Vec3 = [abs[8], abs[9], abs[10]], lx: Vec3 = [lastReached[0], lastReached[1], lastReached[2]];
    return norm(cross(lx, z)) > 0.2 ? poseFromZ([abs[12], abs[13], abs[14]], z, lx) : abs;
  };
  for (let i = 0; i < poses.length; i++) {
    const abs = aimed(poses[i].abs);
    if (i === 0) {
      const app = mul(abs, transl(0, 0, -approach));
      const r0 = solve(app, q, true);
      if (r0) {
        applyCarrier(r0.carrierQ);
        const t = frame.addChild(new Target('Approach'));
        t.setPose(multiply(invert(frame.poseAbs()), r0.abs));
        t.setJoints(withExt(r0.q, r0.carrierQ));
        if (carrier) t.setParam('carrierJoints', r0.carrierQ ?? []);
        prog.addMoveJ(t);
        q = r0.q;
        lastReached = r0.abs;
      }
      if (opts.io) prog.setDO(opts.io, true);
    }
    const r = solve(abs, q, n === 0);
    if (!r) { unreachable++; continue; }
    applyCarrier(r.carrierQ);
    q = r.q;
    lastReached = r.abs;
    n++;
    const t = frame.addChild(new Target(`P${n}`));
    t.setPose(multiply(invert(frame.poseAbs()), r.abs));
    t.setJoints(withExt(r.q, r.carrierQ));
    if (carrier) t.setParam('carrierJoints', r.carrierQ ?? []);
    if (opts.linear === false) prog.addMoveJ(t); else prog.addMoveL(t);
  }
  if (opts.io) prog.setDO(opts.io, false);
  if (poses.length) {
    const last = poses[poses.length - 1];
    const rr = solve(mul(last.abs, transl(0, 0, -approach)), q, false);
    if (rr) { const t = frame.addChild(new Target('Retract')); t.setPose(multiply(invert(frame.poseAbs()), rr.abs)); t.setJoints(rr.q); prog.addMoveL(t); }
  }
  prog.addMoveJ(home);
  return { program: prog, points: n, unreachable };
}

/** Point follow: visit isolated points (drilling, planting, pollinating) with approach/retract on each. */
export function generatePointFollow(station: Station, robot: Robot, object: SceneObject, points: Array<{ point: number[]; normal?: number[] }>, opts: FollowOptions = {}): FollowResult {
  const approach = opts.approach ?? 50;
  const objAbs = object.poseAbs();
  const base = robot.poseAbs();
  const frame = station.addChild(new Frame(`${object.name} points`));
  frame.setPoseAbs(objAbs);
  robot.setFrame(frame);
  const prog = station.addChild(new Program(opts.name ?? `Points ${object.name}`));
  prog.setRobot(robot);
  prog.setSpeed(opts.speed ?? 200, 90);
  const home = frame.addChild(new Target('Home'));
  home.setJoints(robot.joints());
  home.setAsJointTarget();
  prog.addMoveJ(home);
  let q = robot.joints();
  let n = 0, unreachable = 0;
  for (const p of points) {
    const pw = transformPoint(objAbs, [p.point[0], p.point[1], p.point[2] ?? 0]);
    const z: Vec3 = p.normal ? normalize([-p.normal[0], -p.normal[1], -p.normal[2]]) : opts.customZ ?? [0, 0, -1];
    const abs = poseFromZ(pw, transformDir(objAbs, z));
    const inBase = multiply(invert(base), abs);
    const rA = robot.solveIK(mul(inBase, transl(0, 0, -approach)), { seed: q, freeToolZ: opts.freeToolZ ?? true, restarts: 1 });
    const rP = rA.ok ? robot.solveIK(inBase, { seed: rA.joints, freeToolZ: opts.freeToolZ ?? true, restarts: 0 }) : rA;
    if (!rA.ok || !rP.ok) { unreachable++; continue; }
    n++;
    const local = multiply(invert(objAbs), abs);
    // tool-axis freedom: store the poses the robot reaches (rotation about tool Z chosen by the solver)
    const free = opts.freeToolZ ?? true;
    const reached = (q: number[]) => multiply(invert(objAbs), multiply(base, robot.solveFK(q)));
    const tA = frame.addChild(new Target(`Approach ${n}`)); tA.setPose(free ? reached(rA.joints) : mul(local, transl(0, 0, -approach))); tA.setJoints(rA.joints);
    const tP = frame.addChild(new Target(`Point ${n}`)); tP.setPose(free ? reached(rP.joints) : local); tP.setJoints(rP.joints);
    prog.addMoveJ(tA);
    prog.addMoveL(tP);
    if (opts.io) { prog.setDO(opts.io, true); prog.pause(200); prog.setDO(opts.io, false); }
    prog.addMoveL(tA);
    q = rA.joints;
  }
  prog.addMoveJ(home);
  return { program: prog, points: n, unreachable };
}

export { cross };
