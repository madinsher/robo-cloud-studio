import { describe, it, expect } from 'vitest';
import * as P from '../src/core/math/pose';

const close = (a: number[], b: number[], tol = 1e-6) => a.every((v, i) => Math.abs(v - b[i]) < tol);

describe('pose math', () => {
  it('multiply/invert are consistent', () => {
    const a = P.mul(P.transl(100, 20, -5), P.rotx(0.3), P.roty(-1.1), P.rotz(2));
    const inv = P.invert(a);
    expect(P.poseEquals(P.multiply(a, inv), P.identity(), 1e-9)).toBe(true);
    expect(P.poseEquals(P.invertGeneral(a), inv, 1e-9)).toBe(true);
  });
  it('xyzrpw roundtrip', () => {
    const v = [10, 20, 30, 15, -40, 120];
    const m = P.xyzrpwToPose(...(v as [number, number, number, number, number, number]));
    expect(close(P.poseToXyzrpw(m), v)).toBe(true);
  });
  it('KUKA abc roundtrip', () => {
    const v = [1, 2, 3, 170, -30, 45];
    const m = P.kukaToPose(...(v as [number, number, number, number, number, number]));
    expect(close(P.poseToKuka(m), v)).toBe(true);
  });
  it('Fanuc wpr roundtrip', () => {
    const v = [1, 2, 3, 10, 20, 30];
    const m = P.fanucToPose(...(v as [number, number, number, number, number, number]));
    expect(close(P.poseToFanuc(m), v)).toBe(true);
  });
  it('quaternion roundtrip', () => {
    const m = P.mul(P.rotx(0.7), P.roty(1.2), P.rotz(-2.5));
    const q = P.poseToQuat(m);
    const m2 = P.quatToPose(0, 0, 0, q);
    expect(P.poseEquals(m, m2, 1e-9)).toBe(true);
  });
  it('UR rotation vector roundtrip', () => {
    const m = P.mul(P.transl(5, 6, 7), P.rotx(0.7), P.roty(1.2), P.rotz(-2.5));
    const v = P.poseToUr(m);
    const m2 = P.urToPose(...(v as [number, number, number, number, number, number]));
    expect(P.poseEquals(m, m2, 1e-9)).toBe(true);
  });
  it('slerp endpoints', () => {
    const a = P.rotx(0.2), b = P.mul(P.transl(10, 0, 0), P.rotz(1));
    expect(P.poseEquals(P.slerpPose(a, b, 0), a, 1e-9)).toBe(true);
    expect(P.poseEquals(P.slerpPose(a, b, 1), b, 1e-9)).toBe(true);
  });
  it('poseFromZ builds orthonormal frame', () => {
    const m = P.poseFromZ([1, 2, 3], [0, 1, 1]);
    const x: P.Vec3 = [m[0], m[1], m[2]], y: P.Vec3 = [m[4], m[5], m[6]], z: P.Vec3 = [m[8], m[9], m[10]];
    expect(Math.abs(P.dot(x, y))).toBeLessThan(1e-9);
    expect(Math.abs(P.dot(x, z))).toBeLessThan(1e-9);
    expect(Math.abs(P.norm(z) - 1)).toBeLessThan(1e-9);
  });
});
