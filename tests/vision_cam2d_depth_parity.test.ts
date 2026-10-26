/**
 * G3 (V6 "Smaller Cam2D parity gaps"): Cam2D DEPTH snapshots pixel for pixel against RoboDK 5.9.4.
 * Ground truth (tests/fixtures/vision_gt/robodk_cam2d_depth, written by RoboDK itself with gen_robodk_depth.py and
 * V6's gen_v6_three_plates.py):
 *  - two ramps (3000 x 3000 x 2 mm boxes tilted 20° about the camera y / x axis, 1 m below the camera): the depth of a
 *    pixel tells exactly where along u (v) RoboDK sampled it. RoboDK samples its depth at (−0.125, −0.375) px from the
 *    pixel centre (image coordinates, v down) at 640x480 / FOV 32 and at 320x240 / FOV 50 — sample 0 of the standard
 *    4x MSAA pattern — while its RGB snapshot is centre-sampled;
 *  - V6's three plates (RoboDK had 149 depth edge pixels the studio lacked: one column right of the green plate, one
 *    row below the red one with the depth of the plate's side face) and a single plate.
 */
import { it, expect, describe } from 'vitest';
import fs from 'node:fs';
import path from 'node:path';
import zlib from 'node:zlib';
import { Station, SceneObject } from '../src/core/items/item';
import { Robolink } from '../src/api/robolink';
import { mul, transl, rotx, roty, DEG } from '../src/core/math/pose';
import { decodeGrey32, decodePNG } from '../src/vision/png';

const FIX = path.resolve(__dirname, 'fixtures/vision_gt/robodk_cam2d_depth');
const gt = (name: string, far: number) => decodeGrey32(new Uint8Array(zlib.gunzipSync(fs.readFileSync(path.join(FIX, name)))), far);

function station() {
  const st = new Station();
  const RDK = new Robolink(st);
  const ref = RDK.AddFrame('Camera Reference');
  (ref.item as any).setPose(mul(transl(0, 0, 1000), rotx(180 * DEG)));
  const box = (name: string, size: [number, number, number], pose: Float64Array, color = '#808080') => {
    const o = st.addChild(new SceneObject(name));
    o.geometry = [{ primitive: { kind: 'box', size }, color }];
    o.color = color;
    o.setPose(pose);
    return o;
  };
  return { st, RDK, ref, box };
}

/** Sample position offsets along u (ramp about y) or v (ramp about x), from the depth of each pixel. */
function rampOffsets(depth: Float32Array, w: number, h: number, fovDeg: number, axis: 'u' | 'v'): { mean: number; std: number; n: number } {
  const f = h / 2 / Math.tan((fovDeg / 2) * DEG), c = axis === 'u' ? w / 2 : h / 2;
  const a = 20 * DEG, t = 1;
  let s = 0, s2 = 0, n = 0;
  for (let j = 0; j < h; j++) for (let i = 0; i < w; i++) {
    const Z = depth[j * w + i];
    if (!(Z > 0)) continue;
    const pos = c + (f * (Math.cos(a) - (1000 * Math.cos(a) - t) / Z)) / Math.sin(a);
    const off = pos - ((axis === 'u' ? i : j) + 0.5);
    s += off; s2 += off * off; n++;
  }
  const mean = s / n;
  return { mean, std: Math.sqrt(Math.max(0, s2 / n - mean * mean)), n };
}

describe('Cam2D DEPTH sampling = RoboDK 5.9.4', () => {
  it('ramps: the studio samples depth where RoboDK does, (−0.125, −0.375) px from the pixel centre, at any size / FOV', () => {
    const { RDK, ref, box } = station();
    const A = box('rampA', [3000, 3000, 2], roty(20 * DEG));
    const B = box('rampB', [3000, 3000, 2], rotx(20 * DEG));
    for (const [size, fov, suffix] of [['640x480', 32, ''], ['320x240', 50, '_small']] as const) {
      const cam = RDK.Cam2D_Add(ref, `FOV=${fov} FAR_LENGTH=3000 SIZE=${size} DEPTH`);
      for (const [name, axis] of [['rampA', 'u'], ['rampB', 'v']] as const) {
        A.setVisible(name === 'rampA'); B.setVisible(name === 'rampB');
        const studio = decodeGrey32(RDK.Cam2D_Snapshot('', cam, 'DEPTH') as Uint8Array, 3000);
        const robodk = gt(`depth_${name}${suffix}.grey32.gz`, 3000);
        expect([studio.width, studio.height]).toEqual([robodk.width, robodk.height]);
        const so = rampOffsets(studio.depth, studio.width, studio.height, fov, axis), ro = rampOffsets(robodk.depth, robodk.width, robodk.height, fov, axis);
        expect(so.n).toBe(studio.width * studio.height);
        expect(ro.n).toBe(so.n);
        // RoboDK: u −0.122 / −0.125, v −0.373 / −0.375 (std 0.008 / 0.003 px: its depth quantisation); studio exact
        expect(Math.abs(so.mean - ro.mean), `${name}${suffix} studio ${so.mean} robodk ${ro.mean}`).toBeLessThan(0.005);
        expect(so.std).toBeLessThan(1e-3);
        expect(so.mean).toBeCloseTo(axis === 'u' ? -0.125 : -0.375, 3);
      }
    }
  });

  it("V6's three plates: no missing depth edge pixel (was 149 of 11 878), same depth on every pixel, RGB still centre-sampled", () => {
    const { RDK, ref, box } = station();
    box('plate_red', [100, 100, 2], transl(120, 80, 99), '#ff0000');
    box('plate_green', [60, 60, 2], transl(-50, -200, 99), '#00ff00');
    box('plate_blue', [100, 100, 2], transl(-150, 100, -101), '#0000ff');
    const camd = RDK.Cam2D_Add(ref, 'FOV=32 FAR_LENGTH=1000 SIZE=640x480 DEPTH');
    const studio = decodeGrey32(RDK.Cam2D_Snapshot('', camd, 'DEPTH') as Uint8Array, 1000);
    const robodk = gt('v6_three_plates.grey32.gz', 1000);
    let onlyR = 0, onlyS = 0, both = 0, maxTop = 0, maxSide = 0, side = 0;
    for (let i = 0; i < studio.depth.length; i++) {
      const r = robodk.depth[i] > 0, s = studio.depth[i] > 0;
      if (r && !s) onlyR++; else if (s && !r) onlyS++;
      else if (r && s) {
        both++;
        const dz = Math.abs(robodk.depth[i] - studio.depth[i]);
        if (Math.abs(robodk.depth[i] - 900) < 0.1) maxTop = Math.max(maxTop, dz); else { side++; maxSide = Math.max(maxSide, dz); }
      }
    }
    expect({ onlyR, onlyS, both }).toEqual({ onlyR: 0, onlyS: 0, both: 11878 });
    // RoboDK's depth reads 900.0145 on the 900 mm top faces (its depth precision); the row below the red plate hits
    // the plate's side face, nearly parallel to the rays (32 mm of depth per pixel): RoboDK 900.755
    expect(maxTop).toBeLessThan(0.02);
    expect(side).toBe(93);
    expect(maxSide).toBeLessThan(0.1);
    // the RGB snapshot of RoboDK is centre-sampled: the same silhouettes as before (V6: 11 729 px, XOR 0)
    const cam = RDK.Cam2D_Add(ref, 'FOCAL_LENGHT=6 FOV=32 FAR_LENGHT=1000 SIZE=640x480 BG_COLOR=black');
    const rgb = decodePNG(RDK.Cam2D_Snapshot('', cam) as Uint8Array);
    const ref6 = decodePNG(new Uint8Array(fs.readFileSync(path.join(FIX, 'v6_three_plates_rgb.png'))));
    const lit = (p: { rgba: Uint8Array | Uint8ClampedArray }, i: number) => p.rgba[i * 4] + p.rgba[i * 4 + 1] + p.rgba[i * 4 + 2] > 30;
    let xor = 0, n = 0;
    for (let i = 0; i < 640 * 480; i++) { if (lit(rgb, i) !== lit(ref6, i)) xor++; if (lit(ref6, i)) n++; }
    expect(n).toBe(11729);
    expect(xor).toBe(0);
  });

  it('a single plate: depth coverage equal to RoboDK', () => {
    const { RDK, ref, box } = station();
    box('plate', [60, 60, 2], transl(-50, -200, 99));
    const cam = RDK.Cam2D_Add(ref, 'FOV=32 FAR_LENGTH=3000 SIZE=640x480 DEPTH');
    const studio = decodeGrey32(RDK.Cam2D_Snapshot('', cam, 'DEPTH') as Uint8Array, 3000);
    const robodk = gt('depth_plate.grey32.gz', 3000);
    let xor = 0, n = 0;
    for (let i = 0; i < studio.depth.length; i++) { const r = robodk.depth[i] > 0; if (r !== studio.depth[i] > 0) xor++; if (r) n++; }
    expect(n).toBeGreaterThan(3000);
    expect(xor).toBe(0);
  });
});
