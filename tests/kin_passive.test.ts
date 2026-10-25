// R1 (rdk-sim): passive counterbalance links of RoboDK robots (cylinder at a pivot of link 1, rod at a pivot of link 2,
// both aimed from one pivot to the other, Y along the J2 axis). Ground truth: RoboDK 5.9.4 ObjectLink(k).PoseAbs() of
// every link at 6 joint vectors (tests/fixtures/robodk_gt/passive/*.json, dump_linkposes.py).
import { describe, it, expect } from 'vitest';
import { existsSync, readFileSync } from 'node:fs';
import { importRdkBestEffort } from '../src/io/robodk/rdk_container';
import { AssetStore } from '../src/scene/assets';
import { Robot } from '../src/core/items/robot';
import { visualOrigin } from '../src/core/kinematics/chain';
import { multiply, fromRows } from '../src/core/math/pose';
import type { Mat4 } from '../src/core/math/pose';

const LIB = 'C:/RoboDK/Library';
const CASES: Array<[string, string, string]> = [
  ['kr90', 'Example-05.e-Manufacturing with 2 robots.rdk', 'KUKA KR 90 R3100 extra'],
  ['kr100', 'Example-03.d-Polishing Canadian Canoe - KUKA.rdk', 'KUKA KR 100 R3500 press'],
  ['abb6640', 'Example-05.c-Robot machining with a rail and turntable - ABB.rdk', 'ABB IRB 6640-130/3.2'],
  ['abb6700', 'ABB-IRB-6700-150-3-20.robot', 'ABB IRB 6700-150/3.20'],
  ['fanuc', 'Fanuc-R-2000iB-210F.robot', 'Fanuc R-2000iB/210F'],
];
const rows = (a: number[]): Mat4 => fromRows([a.slice(0, 4), a.slice(4, 8), a.slice(8, 12), a.slice(12, 16)]);
const errMm = (a: Mat4, b: Mat4) => Math.hypot(a[12] - b[12], a[13] - b[13], a[14] - b[14]);
const errRot = (a: Mat4, b: Mat4) => Math.max(...[0, 1, 2, 4, 5, 6, 8, 9, 10].map((i) => Math.abs(a[i] - b[i])));

describe('RoboDK passive links (counterbalance) follow J2 like RoboDK', () => {
  for (const [tag, file, name] of CASES) {
    it.skipIf(!existsSync(`${LIB}/${file}`))(`${name}: cylinder / rod frames equal RoboDK's link 7 / 8 poses at 6 joint vectors`, async () => {
      const gt = JSON.parse(readFileSync(`tests/fixtures/robodk_gt/passive/${tag}.json`, 'utf8'));
      const b = readFileSync(`${LIB}/${file}`);
      const { station } = await importRdkBestEffort(b.buffer.slice(b.byteOffset, b.byteOffset + b.byteLength), file, new AssetStore());
      const robot = [...station.walk()].find((i): i is Robot => i instanceof Robot && i.name === name)!;
      expect(robot).toBeTruthy();
      const passive = robot.chain.links.flatMap((l, i) => l.visuals.filter((v) => v.passive).map((v) => ({ v, link: i })));
      expect(new Set(passive.map((p) => p.v.meshLink))).toEqual(new Set([7, 8]));
      for (const s of gt.samples) {
        const fk = robot.fk(s.q.slice(0, robot.dof));
        const base = robot.poseAbs();
        // the regular links coincide (the meshes of links 0..6 sit in these frames)
        for (let k = 1; k <= 6; k++) expect(errMm(multiply(base, fk.linkPoses[k]), rows(s.links[k])), `${name} link ${k}`).toBeLessThan(0.01);
        for (const { v, link } of passive) {
          const got = multiply(multiply(base, fk.linkPoses[link]), visualOrigin(fk.linkPoses, link, v));
          const want = rows(s.links[v.meshLink!]);
          expect(errMm(got, want), `${name} passive ${v.meshLink} at q ${s.q}`).toBeLessThan(0.01);
          expect(errRot(got, want), `${name} passive ${v.meshLink} rotation`).toBeLessThan(1e-4);
        }
      }
    }, 120000);
  }
});
