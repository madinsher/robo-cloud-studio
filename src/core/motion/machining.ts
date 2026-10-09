/**
 * Robot machining from NC programs (RoboDK "Robot machining project" / "3D printing project" / "Milling"):
 * turns G-code segments (rapids, cuts with feed rates, spindle/coolant state, extrusion) into a robot
 * program with approach/retract, per-segment speeds, spindle / extruder digital outputs and rapid moves
 * at a clearance height. Works for milling, cutting (laser, plasma, waterjet, knife), dispensing and FDM
 * printing; the path stays attached to the part object so it moves with it.
 */
import { Station, SceneObject, Frame, Target } from '../items/item';
import { Robot } from '../items/robot';
import { Program } from '../items/program';
import { Mat4, poseFromZ, multiply, invert, transl, mul, normalize, sub, norm, Vec3, transformPoint, transformDir, rotz, DEG } from '../math/pose';

/** A curve produced from NC code: polyline + metadata (stored on SceneObject.curves). */
export interface MachiningCurve {
  name: string;
  points: number[][];
  kind?: 'cut' | 'rapid';
  /** Feed rate (mm/min) for cutting segments. */
  feed?: number;
  spindle?: boolean;
  /** Extruding (3D printing) while moving. */
  extrude?: boolean;
  tool?: number;
  normals?: number[][];
}

export interface MachiningOptions {
  name?: string;
  /** Approach/retract distance along -Z of the tool at start/end (mm). */
  approach?: number;
  /** Clearance above the part for rapid moves that the NC file specifies in Z (kept) — used when a rapid is 2D only. */
  clearance?: number;
  /** Tool Z direction: -Z of the part frame (default, milling/printing) or along curve normals. */
  zMode?: 'down' | 'normal' | 'custom';
  customZ?: Vec3;
  /** Rotate tool about its Z (deg). */
  spin?: number;
  /** Let IK ignore rotation about the tool axis (symmetric tools: spindle, nozzle, laser). Default true. */
  freeToolZ?: boolean;
  /** Override feed for all cuts (mm/s). If unset, the NC feed (mm/min) is used. */
  cutSpeed?: number;
  /** Speed for rapid moves (mm/s). Default 250. */
  rapidSpeed?: number;
  /** Fallback when a cut has no feed (mm/s). Default 20. */
  defaultCutSpeed?: number;
  /** Digital output switched with the spindle / laser / extruder. */
  spindleIO?: string;
  extruderIO?: string;
  /** Emit rapids as MoveJ (default false = MoveL at rapid speed). */
  rapidAsJoint?: boolean;
  /** Maximum points per curve after resampling (0 = keep). */
  step?: number;
  /** Rounding radius for cutting moves (mm). Default 1. */
  rounding?: number;
}

export interface MachiningResult {
  program: Program;
  points: number;
  unreachable: number;
  cutLength: number;
  rapidLength: number;
  /** Estimated cycle time (s) from feeds and rapid speed. */
  estimatedTime: number;
  segments: number;
}

function polyLength(pts: number[][]): number {
  let l = 0;
  for (let i = 1; i < pts.length; i++) l += Math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1], (pts[i][2] ?? 0) - (pts[i - 1][2] ?? 0));
  return l;
}

function resample(points: number[][], step: number): number[][] {
  if (!step || points.length < 2) return points;
  const out: number[][] = [points[0]];
  let carry = 0;
  for (let i = 1; i < points.length; i++) {
    const a = points[i - 1], b = points[i];
    const d = Math.hypot(b[0] - a[0], b[1] - a[1], (b[2] ?? 0) - (a[2] ?? 0));
    let s = step - carry;
    while (s <= d) { const t = s / d; out.push([a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, (a[2] ?? 0) + ((b[2] ?? 0) - (a[2] ?? 0)) * t]); s += step; }
    carry = d - (s - step);
    if (i === points.length - 1) out.push(b);
  }
  return out;
}

/**
 * Generate a machining program for `robot` over the NC curves of `part`. Curves without `kind` are treated
 * as cuts (so plain curve objects work too).
 */
export function generateMachining(station: Station, robot: Robot, part: SceneObject, opts: MachiningOptions = {}): MachiningResult {
  const curves = (part.curves as MachiningCurve[]).filter((c) => c.points.length > 1);
  const approach = opts.approach ?? 50;
  const rapidSpeed = opts.rapidSpeed ?? 250;
  const defaultCut = opts.defaultCutSpeed ?? 20;
  const rounding = opts.rounding ?? 1;
  const objAbs = part.poseAbs();
  const base = robot.poseAbs();
  const frame = station.addChild(new Frame(`${part.name} NC`));
  frame.setPoseAbs(objAbs);
  robot.setFrame(frame);
  const prog = station.addChild(new Program(opts.name ?? `Machining ${part.name}`));
  prog.setRobot(robot);
  const home = frame.addChild(new Target('Home'));
  home.setJoints(robot.joints());
  home.setAsJointTarget();
  prog.addMoveJ(home);
  prog.setRounding(rounding);

  const zOf = (c: MachiningCurve, i: number): Vec3 => {
    if (opts.zMode === 'custom' && opts.customZ) return normalize(opts.customZ);
    if (opts.zMode === 'normal' && c.normals?.[i]) { const nrm = c.normals[Math.min(i, c.normals.length - 1)]; return normalize([-nrm[0], -nrm[1], -nrm[2]]) as Vec3; }
    return [0, 0, -1];
  };
  const ikOpts = { freeToolZ: opts.freeToolZ ?? true, restarts: 1, maxIterations: 80 };
  let q = robot.joints();
  let first = true;
  const solve = (abs: Mat4): number[] | null => {
    const r = robot.solveIK(multiply(invert(base), abs), { seed: q, ...ikOpts, ...(first ? { restarts: 6, maxIterations: 200 } : {}) });
    return r.ok ? r.joints : null;
  };
  const poseAt = (c: MachiningCurve, pts: number[][], i: number): Mat4 => {
    const p = pts[i];
    const pw = transformPoint(objAbs, [p[0], p[1], p[2] ?? 0]);
    const zw = transformDir(objAbs, zOf(c, i));
    const nxt = pts[Math.min(i + 1, pts.length - 1)], prv = pts[Math.max(i - 1, 0)];
    const dirLocal = normalize(sub([nxt[0], nxt[1], nxt[2] ?? 0], [prv[0], prv[1], prv[2] ?? 0]));
    const dirW = norm(dirLocal) > 1e-9 ? transformDir(objAbs, dirLocal) : ([1, 0, 0] as Vec3);
    let abs = poseFromZ(pw, zw, dirW);
    if (opts.spin) abs = mul(abs, rotz(opts.spin * DEG));
    return abs;
  };
  const addTarget = (name: string, abs: Mat4, joints: number[]): Target => {
    const t = frame.addChild(new Target(name));
    t.setPose(multiply(invert(frame.poseAbs()), abs));
    t.setJoints(joints);
    return t;
  };

  let points = 0, unreachable = 0, cutLength = 0, rapidLength = 0, time = 0, segments = 0;
  let spindleOn = false, extruderOn = false, lastSpeed = -1;
  const setSpeed = (v: number) => { if (Math.abs(v - lastSpeed) > 1e-6) { prog.setSpeed(v); lastSpeed = v; } };
  const setIO = (io: string | undefined, on: boolean, cur: boolean): boolean => { if (io && on !== cur) prog.setDO(io, on); return io ? on : cur; };

  for (let ci = 0; ci < curves.length; ci++) {
    const c = curves[ci];
    const kind = c.kind ?? 'cut';
    const pts = kind === 'cut' && opts.step ? resample(c.points, opts.step) : c.points;
    const len = polyLength(pts);
    const speed = kind === 'rapid' ? rapidSpeed : opts.cutSpeed ?? (c.feed ? c.feed / 60 : defaultCut);
    segments++;
    if (kind === 'cut') cutLength += len; else rapidLength += len;
    time += len / Math.max(1e-6, speed);
    // approach before the very first point
    if (first) {
      const a0 = mul(poseAt(c, pts, 0), transl(0, 0, -approach));
      const qa = solve(a0);
      if (qa) { q = qa; prog.addMoveJ(addTarget('Approach', a0, qa)); }
      first = false;
    }
    if (kind === 'cut') {
      prog.comment(`${c.name}${c.feed ? ` F${c.feed}` : ''}${c.tool !== undefined ? ` T${c.tool}` : ''}`);
      spindleOn = setIO(opts.spindleIO, c.spindle !== false, spindleOn);
      extruderOn = setIO(opts.extruderIO, !!c.extrude, extruderOn);
    } else {
      extruderOn = setIO(opts.extruderIO, false, extruderOn);
    }
    setSpeed(speed);
    const start = kind === 'rapid' ? 1 : 0; // rapid's first point is where we already are
    for (let i = start; i < pts.length; i++) {
      const abs = poseAt(c, pts, i);
      const qi = solve(abs);
      if (!qi) { unreachable++; continue; }
      q = qi;
      points++;
      const t = addTarget(kind === 'rapid' ? `R${points}` : `P${points}`, abs, qi);
      if (kind === 'rapid' && opts.rapidAsJoint) prog.addMoveJ(t); else prog.addMoveL(t);
    }
  }
  extruderOn = setIO(opts.extruderIO, false, extruderOn);
  spindleOn = setIO(opts.spindleIO, false, spindleOn);
  // retract from the last reached point
  if (points) {
    const lastAbs = robot.solveFK(q, robot.poseTool());
    const retract = mul(multiply(base, lastAbs), transl(0, 0, -approach));
    const qr = solve(retract);
    setSpeed(rapidSpeed);
    if (qr) { q = qr; prog.addMoveL(addTarget('Retract', retract, qr)); }
  }
  prog.addMoveJ(home);
  prog.setParam('machining', { cutLength, rapidLength, estimatedTime: time, segments, part: part.id });
  return { program: prog, points, unreachable, cutLength, rapidLength, estimatedTime: time, segments };
}
