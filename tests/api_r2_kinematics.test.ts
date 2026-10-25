/**
 * R2 (T2 request to the API): RoboDK SolveFK / SolveIK / SolveIK_All / JointsConfig through the studio's RoboDK-semantics
 * kinematics. A robot with synchronised external axes (Library 02.f: IRB 6640 + 2-axis turntable) takes the 8-joint
 * vector in SolveFK like RoboDK; values are RoboDK 5.9.4's own (tests/fixtures/robodk_gt: 12 FK samples per robot).
 */
import { describe, it, expect } from 'vitest';
import { existsSync, readFileSync } from 'node:fs';
import { gunzipSync } from 'node:zlib';
import path from 'node:path';
import { importRdkBestEffort } from '../src/io/robodk/rdk_container';
import { AssetStore } from '../src/scene/assets';
import { Robolink } from '../src/api/robolink';

const FILE = 'C:/RoboDK/Library/Example-02.f-Robot Machining Spherical Part and Turntable - ABB.rdk';
const GT = path.join(__dirname, 'fixtures/robodk_gt/Example-02.f-Robot_Machining_Spherical_Part_and_Turntable_-_ABB.rdk_.json.gz');
const colMajor = (rows: number[]) => [0, 1, 2, 3].flatMap((c) => [0, 1, 2, 3].map((r) => rows[r * 4 + c]));

describe.skipIf(!existsSync(FILE) || !existsSync(GT))('API kinematics with RoboDK semantics (Library 02.f, synchronised turntable)', () => {
  it('SolveFK of the 8-joint vector = RoboDK; SolveIK / SolveIK_All return joints whose FK is the pose; JointsConfig flags', async () => {
    const gt = JSON.parse(gunzipSync(readFileSync(GT)).toString().replace(/\bNaN\b/g, 'null'));
    const g = gt.robots.find((r: any) => r.name === 'ABB IRB 6640-130/3.2');
    const b = readFileSync(FILE);
    const { station } = await importRdkBestEffort(b.buffer.slice(b.byteOffset, b.byteOffset + b.byteLength) as ArrayBuffer, '02.f', new AssetStore());
    const rdk = new Robolink(station);
    const robot = rdk.Item('ABB IRB 6640-130/3.2', 2);
    expect(g.fk.length).toBe(12);
    for (const s of g.fk) {
      expect(s.q.length).toBe(8);
      const m = robot.SolveFK(s.q).m;
      const want = colMajor(s.flange);
      expect(Math.hypot(m[12] - want[12], m[13] - want[13], m[14] - want[14]), `position for q=${s.q}`).toBeLessThan(0.01);
      for (const k of [0, 1, 2, 4, 5, 6, 8, 9, 10]) expect(Math.abs(m[k] - want[k]), `rotation ${k}`).toBeLessThan(1e-5);
    }
    // the arm alone (6 joints): IK of its own FK, closest to the seed
    const q6 = g.fk[0].q.slice(0, 6);
    const pose = robot.SolveFK(q6);
    const ik = robot.SolveIK(pose, q6);
    expect(ik.length).toBe(6);
    ik.forEach((v: number, i: number) => expect(Math.abs(v - q6[i]), `J${i + 1}`).toBeLessThan(1e-4));
    const all = robot.SolveIK_All(pose);
    expect(all.length).toBeGreaterThan(1);
    for (const q of all) { const p = robot.SolveFK(q).m, w = pose.m; expect(Math.hypot(p[12] - w[12], p[13] - w[13], p[14] - w[14])).toBeLessThan(1e-3); }
    expect(all.some((q: number[]) => q.every((v, i) => Math.abs(v - q6[i]) < 1e-4))).toBe(true);
    // RoboDK conf_RLF of each IK solution are distinct configurations
    const confs = new Set(all.map((q: number[]) => robot.JointsConfig(q).slice(0, 3).join('')));
    expect(confs.size).toBeGreaterThan(1);
  }, 120_000);
});
