import { describe, it, expect } from 'vitest';
import { calibrateTcpByPoint } from '../src/core/calibration/tcp';
import { calibrateFrame, CALIBRATE_FRAME_3P_P1_ORIGIN, CALIBRATE_TURNTABLE, CALIBRATE_FRAME_6P } from '../src/core/calibration/frame';
import { calibrateRobot, simulateMeasurements, applyCalibration } from '../src/core/calibration/robot';
import { createIsoCubeProgram, poseAccuracy, createBallbarProgram, ballbarAnalysis } from '../src/core/calibration/iso9283';
import { createRobotFromLibrary } from '../src/core/items/library';
import { Station } from '../src/core/items/item';
import { ProgramSimulator } from '../src/core/motion/simulator';
import { mul, transl, rotx, roty, rotz, DEG, invert, multiply, getPos, poseToXyzrpw } from '../src/core/math/pose';

describe('calibration', () => {
  it('TCP calibration recovers the tool offset from 4 poses touching one point', () => {
    const tcp = [12.5, -3, 180] as const;
    const point = [600, 100, 300];
    const flanges = [[0, 0, 0], [30, 0, 0], [0, 40, 0], [-25, 20, 60]].map(([a, b, c]) => {
      const R = mul(rotx(180 * DEG), rotx(a * DEG), roty(b * DEG), rotz(c * DEG));
      // flange such that flange * tcp = point:  t = point - R*tcp
      const rt = [R[0] * tcp[0] + R[4] * tcp[1] + R[8] * tcp[2], R[1] * tcp[0] + R[5] * tcp[1] + R[9] * tcp[2], R[2] * tcp[0] + R[6] * tcp[1] + R[10] * tcp[2]];
      const m = new Float64Array(R);
      m[12] = point[0] - rt[0]; m[13] = point[1] - rt[1]; m[14] = point[2] - rt[2];
      return m;
    });
    const res = calibrateTcpByPoint(flanges);
    expect(Math.abs(res.tcp[0] - tcp[0])).toBeLessThan(1e-6);
    expect(Math.abs(res.tcp[2] - tcp[2])).toBeLessThan(1e-6);
    expect(res.maxError).toBeLessThan(1e-6);
  });

  it('frame calibration from 3 points and turntable circle fit', () => {
    const f = calibrateFrame([[100, 0, 0], [200, 0, 0], [150, 50, 0]], CALIBRATE_FRAME_3P_P1_ORIGIN);
    expect(poseToXyzrpw(f.pose).slice(0, 3).map((v) => Math.round(v))).toEqual([100, 0, 0]);
    const six = calibrateFrame([[0, 0, 5], [100, 0, 5], [0, 100, 5], [10, 0, 5], [90, 0, 5], [0, 50, 5]], CALIBRATE_FRAME_6P);
    expect(Math.abs(six.pose[14] - 5)).toBeLessThan(1e-6);
    const pts: [number, number, number][] = [];
    for (let i = 0; i < 8; i++) { const a = (i / 8) * 2 * Math.PI; pts.push([500 + 200 * Math.cos(a), -100 + 200 * Math.sin(a), 50]); }
    const t = calibrateFrame(pts, CALIBRATE_TURNTABLE);
    expect(Math.abs((t.radius ?? 0) - 200)).toBeLessThan(1e-6);
    expect(Math.abs(t.pose[12] - 500)).toBeLessThan(1e-6);
    expect(Math.abs(Math.abs(t.axis![2]) - 1)).toBeLessThan(1e-9);
  });

  it('robot calibration identifies joint offsets from simulated tracker measurements', () => {
    const r = createRobotFromLibrary('UR5e');
    const realDh = r.chain.dh!.map((d, i) => ({ ...d, theta: d.theta + [0.3, -0.2, 0.15, 0.1, -0.25, 0.05][i] }));
    const joints: number[][] = [];
    let s = 3;
    const rnd = () => { s = (s * 1103515245 + 12345) & 0x7fffffff; return s / 0x7fffffff; };
    for (let i = 0; i < 30; i++) joints.push([rnd() * 180 - 90, -rnd() * 120 - 20, rnd() * 140 - 70, rnd() * 180 - 90, rnd() * 180 - 90, rnd() * 360 - 180]);
    const ms = simulateMeasurements(r, realDh, joints, 0.0, [0, 0, 100]);
    const res = calibrateRobot(r, ms, { jointOffsets: true, tool: [0, 0, 100] });
    expect(res.before.mean).toBeGreaterThan(1);
    expect(res.after.max).toBeLessThan(0.05);
    applyCalibration(r, res);
    expect(r.params.calibrated).toBe(true);
  });

  it('ISO 9283 cube and ballbar programs are valid; statistics computed', () => {
    const st = new Station();
    const r = st.addChild(createRobotFromLibrary('KUKA_KR16_R2010'));
    const { program, targets } = createIsoCubeProgram(st, r, [1100, 0, 700], 400, 2);
    const res = new ProgramSimulator(st).compile(program);
    expect(res.problems.filter((p) => p.severity === 'error')).toEqual([]);
    const commanded = targets.map((t) => getPos(t.poseAbs()) as [number, number, number]);
    const measured = commanded.map((c) => [0, 1, 2].map((k) => [c[0] + 0.1 * k, c[1] - 0.05 * k, c[2]] as [number, number, number]));
    const stats = poseAccuracy(commanded, measured);
    expect(stats.overall.AP).toBeGreaterThan(0.05);
    expect(stats.overall.RP).toBeGreaterThan(0);
    const bb = createBallbarProgram(st, r, [1100, 0, 700], 150, 12);
    const res2 = new ProgramSimulator(st).compile(bb.program);
    expect(res2.problems.filter((p) => p.severity === 'error')).toEqual([]);
    const a = ballbarAnalysis([[150, 0, 0], [0, 150.2, 0], [-149.8, 0, 0]], [0, 0, 0], 150);
    expect(a.circularity).toBeCloseTo(0.4, 5);
    void invert; void multiply;
  });
});
