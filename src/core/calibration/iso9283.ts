/**
 * ISO 9283 performance tests: pose accuracy & repeatability on the ISO cube diagonal plane, path accuracy,
 * plus the ballbar circular test used by RoboDK (`Popup_ISO9283_CubeProgram`, ballbar validation).
 */
import { Station, Frame, Target } from '../items/item';
import { Robot } from '../items/robot';
import { Program } from '../items/program';
import { Mat4, mul, transl, rotx, DEG, multiply, invert, getPos } from '../math/pose';

/** The 5 ISO 9283 test points (P1 centre, P2..P5 corners of the diagonal plane inset by 10 % of the side). */
export function isoCubePoints(side: number): Array<[number, number, number]> {
  const h = side / 2, k = side * 0.4; // (L - 0.1 L) / 2 = 0.45 L... ISO uses points at (L±0.1L)/2 : use 0.4 L half-distance
  return [[0, 0, 0], [-k, -k, -h * 0.8], [k, k, -h * 0.8], [k, -k, h * 0.8], [-k, k, h * 0.8]];
}

/** Create the ISO 9283 cube program: cycles through P1..P5 `repeat` times with the tool pointing down. */
export function createIsoCubeProgram(station: Station, robot: Robot, centerInBase: [number, number, number], side: number, repeat = 3, speed = 500): { program: Program; frame: Frame; targets: Target[] } {
  const frame = station.addChild(new Frame(`ISO9283 cube ${side} mm`));
  frame.setPoseAbs(multiply(robot.poseAbs(), transl(...centerInBase)));
  robot.setFrame(frame);
  const prog = station.addChild(new Program(`ISO9283 ${robot.name}`));
  prog.setRobot(robot);
  prog.setSpeed(speed, 90);
  const targets: Target[] = [];
  isoCubePoints(side).forEach((p, i) => {
    const t = frame.addChild(new Target(`P${i + 1}`));
    t.setPose(mul(transl(p[0], p[1], p[2]), rotx(180 * DEG)));
    targets.push(t);
  });
  for (let r = 0; r < repeat; r++) {
    for (const t of targets) { prog.addMoveJ(t); prog.pause(500); }
  }
  prog.addMoveJ(targets[0]);
  return { program: prog, frame, targets };
}

export interface PoseAccuracyStats {
  /** ISO 9283 pose accuracy AP (mean deviation from the commanded pose) per point, mm. */
  accuracy: number[];
  /** Pose repeatability RP = mean radius + 3σ per point, mm. */
  repeatability: number[];
  overall: { AP: number; RP: number };
}

/** Compute ISO 9283 AP/RP from commanded points and repeated measured positions (mm). */
export function poseAccuracy(commanded: Array<[number, number, number]>, measured: Array<Array<[number, number, number]>>): PoseAccuracyStats {
  const accuracy: number[] = [], repeatability: number[] = [];
  commanded.forEach((c, i) => {
    const ms = measured[i] ?? [];
    if (!ms.length) { accuracy.push(NaN); repeatability.push(NaN); return; }
    const mean: [number, number, number] = [0, 0, 0];
    for (const m of ms) { mean[0] += m[0] / ms.length; mean[1] += m[1] / ms.length; mean[2] += m[2] / ms.length; }
    accuracy.push(Math.hypot(mean[0] - c[0], mean[1] - c[1], mean[2] - c[2]));
    const l = ms.map((m) => Math.hypot(m[0] - mean[0], m[1] - mean[1], m[2] - mean[2]));
    const lbar = l.reduce((a, v) => a + v, 0) / l.length;
    const s = Math.sqrt(l.reduce((a, v) => a + (v - lbar) ** 2, 0) / Math.max(1, l.length - 1));
    repeatability.push(lbar + 3 * s);
  });
  const valid = (a: number[]) => a.filter((v) => !isNaN(v));
  return { accuracy, repeatability, overall: { AP: Math.max(0, ...valid(accuracy)), RP: Math.max(0, ...valid(repeatability)) } };
}

/** Path accuracy: mean/max distance of measured points to the commanded polyline (mm). */
export function pathAccuracy(commanded: number[][], measured: number[][]): { mean: number; max: number } {
  const distToSeg = (p: number[], a: number[], b: number[]) => {
    const ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    const l2 = ab[0] ** 2 + ab[1] ** 2 + ab[2] ** 2 || 1;
    const t = Math.max(0, Math.min(1, ((p[0] - a[0]) * ab[0] + (p[1] - a[1]) * ab[1] + (p[2] - a[2]) * ab[2]) / l2));
    return Math.hypot(p[0] - a[0] - ab[0] * t, p[1] - a[1] - ab[1] * t, p[2] - a[2] - ab[2] * t);
  };
  const d = measured.map((p) => { let best = Infinity; for (let i = 1; i < commanded.length; i++) best = Math.min(best, distToSeg(p, commanded[i - 1], commanded[i])); return best; });
  return { mean: d.reduce((a, v) => a + v, 0) / Math.max(1, d.length), max: Math.max(0, ...d) };
}

/** Ballbar test program: circle of given radius in a plane around a centre (tool pointing down). */
export function createBallbarProgram(station: Station, robot: Robot, centerInBase: [number, number, number], radius = 150, points = 36, speed = 100): { program: Program; frame: Frame } {
  const frame = station.addChild(new Frame(`Ballbar R${radius}`));
  frame.setPoseAbs(multiply(robot.poseAbs(), transl(...centerInBase)));
  robot.setFrame(frame);
  const prog = station.addChild(new Program(`Ballbar ${robot.name}`));
  prog.setRobot(robot);
  prog.setSpeed(speed, 60);
  prog.setRounding(1);
  let first: Target | null = null;
  for (let i = 0; i <= points; i++) {
    const a = (2 * Math.PI * i) / points;
    const t = frame.addChild(new Target(`C${i}`));
    t.setPose(mul(transl(radius * Math.cos(a), radius * Math.sin(a), 0), rotx(180 * DEG)));
    if (i === 0) { first = t; prog.addMoveJ(t); } else prog.addMoveL(t);
  }
  if (first) prog.addMoveJ(first);
  return { program: prog, frame };
}

/** Ballbar analysis: radial deviations of measured points from the nominal circle (mm). */
export function ballbarAnalysis(measured: Array<[number, number, number]>, center: [number, number, number], radius: number): { circularity: number; meanRadius: number; deviations: number[] } {
  const r = measured.map((p) => Math.hypot(p[0] - center[0], p[1] - center[1], p[2] - center[2]));
  const dev = r.map((v) => v - radius);
  return { circularity: Math.max(...dev) - Math.min(...dev), meanRadius: r.reduce((a, v) => a + v, 0) / r.length, deviations: dev };
}

/** Simulated measuring device (laser tracker / stereo camera): true TCP + Gaussian-ish noise. */
export function simulateTrackerMeasure(robot: Robot, noiseMm = 0.03, seedRef = { s: 1 }): [number, number, number] {
  const p = getPos(robot.poseTCPAbs());
  const rnd = () => { seedRef.s = (seedRef.s * 1103515245 + 12345) & 0x7fffffff; return (seedRef.s / 0x7fffffff - 0.5) * 2; };
  const g = () => (rnd() + rnd() + rnd()) / 3;
  return [p[0] + g() * noiseMm, p[1] + g() * noiseMm, p[2] + g() * noiseMm];
}

export { invert };
