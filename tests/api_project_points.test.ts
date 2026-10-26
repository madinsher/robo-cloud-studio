/**
 * T10: Robolink ProjectPoints against RoboDK ground truth. Example-07.c stores the result of RoboDK's own projection
 * (PROJECTION_ALONG_NORMAL_RECALC of a 25 mm grid along -Z of the Section frames onto the Inspection Part): the curve
 * objects under Section1/Section2 hold RoboDK's projected points [x,y,z,i,j,k]. Projecting the same grid points with
 * the studio must give the same positions and surface normals; grid points that missed the part (RoboDK dropped them)
 * come back unchanged.
 */
import { describe, it, expect } from 'vitest';
import { readFileSync, existsSync } from 'node:fs';
import { importRdkBestEffort } from '../src/io/robodk/rdk_container';
import { AssetStore } from '../src/scene/assets';
import { ItemType, SceneObject } from '../src/core/items/item';
import { Robolink, RobolinkItem, PROJECTION_NONE } from '../src/api/robolink';
import { multiply, invert, Mat4 } from '../src/core/math/pose';

const FILE = 'C:/RoboDK/Library/Example-07.c-Script Curve Follow Custom.rdk';
const tp = (m: Mat4, p: number[]) => [m[0] * p[0] + m[4] * p[1] + m[8] * p[2] + m[12], m[1] * p[0] + m[5] * p[1] + m[9] * p[2] + m[13], m[2] * p[0] + m[6] * p[1] + m[10] * p[2] + m[14]];
const td = (m: Mat4, p: number[]) => [m[0] * p[0] + m[4] * p[1] + m[8] * p[2], m[1] * p[0] + m[5] * p[1] + m[9] * p[2], m[2] * p[0] + m[6] * p[1] + m[10] * p[2]];

describe.skipIf(!existsSync(FILE))('ProjectPoints vs RoboDK (Example-07.c saved projections)', () => {
  it('reproduces RoboDK projected points and recalculated normals on the Inspection Part', async () => {
    const b = readFileSync(FILE);
    const assets = new AssetStore();
    const { station } = await importRdkBestEffort(b.buffer.slice(b.byteOffset, b.byteOffset + b.byteLength), 'ex07c', assets);
    const rdk = new Robolink(station);
    rdk.assets = assets;
    const part = station.find('Inspection Part', ItemType.OBJECT) as SceneObject;
    for (const sec of ['Section1', 'Section2']) {
      const fr = station.find(sec, ItemType.FRAME)!;
      const gt = (fr.children.find((c) => c instanceof SceneObject) as SceneObject).curves.flatMap((c) => c.points);
      expect(gt.length).toBeGreaterThan(100);
      const R = multiply(invert(part.poseAbs()), fr.poseAbs());     // Section frame -> part coordinates
      const Ri = invert(R);
      const grid = (x: number, y: number) => [...tp(R, [x, y, 0]), ...td(R, [0, 0, -1])];
      const out = new RobolinkItem(rdk, part).ProjectPoints(gt.map((p) => grid(p[0], p[1])));
      const angles: number[] = [];
      out.forEach((q, i) => {
        const s = tp(Ri, q), n = td(Ri, q.slice(3)), g = gt[i];
        expect(Math.hypot(s[0] - g[0], s[1] - g[1], s[2] - g[2]), `${sec} point ${i}`).toBeLessThan(0.01);
        const c = (n[0] * g[3] + n[1] * g[4] + n[2] * g[5]) / (Math.hypot(n[0], n[1], n[2]) * Math.hypot(g[3], g[4], g[5]));
        angles.push((Math.acos(Math.min(1, c)) * 180) / Math.PI);
      });
      angles.sort((a, b2) => a - b2);
      expect(angles[angles.length >> 1], `${sec} median normal error (deg)`).toBeLessThan(0.01);
      expect(angles[Math.floor(angles.length * 0.95)], `${sec} 95% normal error (deg)`).toBeLessThan(0.5);
      expect(angles[angles.length - 1], `${sec} max normal error (deg)`).toBeLessThan(2);
      // grid points RoboDK dropped (outside the part / through windows) do not hit the part and stay where they were
      const kept = new Set(gt.map((p) => `${Math.round(p[0])},${Math.round(p[1])}`));
      const missed: number[][] = [];
      for (let x = 0; x <= 300; x += 25) for (let y = 0; y <= 300; y += 25) if (!kept.has(`${x},${y}`)) missed.push(grid(x, y));
      expect(missed.length).toBeGreaterThan(10);
      new RobolinkItem(rdk, part).ProjectPoints(missed).forEach((q, i) => expect(Math.abs(tp(Ri, q)[2]), `missed ${i}`).toBeLessThan(1e-6));
      // PROJECTION_NONE returns the input
      expect(new RobolinkItem(rdk, part).ProjectPoints([grid(0, 0)], PROJECTION_NONE)[0]).toEqual(grid(0, 0));
    }
  }, 120000);
});
