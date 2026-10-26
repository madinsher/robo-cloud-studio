/**
 * V6 findings on the RoboDK camera API in the browser:
 *  - Cam2D_Snapshot(file) returned 0 in the browser (no file sink): the page's Robolink now writes into the session's
 *    in-memory file system (src/api/memory_fs.ts, File › Files saved by scripts) and returns 1 like RoboDK.
 *  - setColor / Recolor had no visible effect on AddShape / AddFile / imported parts (the shape colour won over the item
 *    colour in the 3D view and in Cam2D images): they now recolour the item's shapes (RoboDK: all shapes).
 *  - cameras are named "Camera 1", "Camera 2", ... and Cam2D_Settings answers in RoboDK's format.
 * RoboDK 5.9.4 ground truth: verify/V6/cam2d_robodk/result.json (snapshot_ret 1, red plate 8649 px, names, settings).
 */
import { describe, it, expect } from 'vitest';
import { Station, SceneObject, ItemType } from '../src/core/items/item';
import { Robolink, ITEM_TYPE_CAMERA } from '../src/api/robolink';
import { MemoryFileSystem } from '../src/api/memory_fs';
import { mul, transl, rotx, DEG } from '../src/core/math/pose';
import { decodePNG } from '../src/vision/png';
import { AssetStore } from '../src/scene/assets';

function scene() {
  const st = new Station('cam');
  const RDK = new Robolink(st);
  RDK.assets = new AssetStore();   // the page's asset store (AddShape / AddFile meshes)
  const ref = RDK.AddFrame('Camera Reference');
  (ref.item as any).setPose(mul(transl(0, 0, 1000), rotx(180 * DEG)));
  return { st, RDK, ref };
}
const redPixels = (png: Uint8Array) => {
  const img = decodePNG(png);
  let n = 0;
  for (let i = 0; i < img.width * img.height; i++) { const r = img.rgba[i * 4], g = img.rgba[i * 4 + 1], b = img.rgba[i * 4 + 2]; if (r > 150 && g < 80 && b < 80) n++; }
  return n;
};

describe('RoboDK camera API in the browser', () => {
  it('Cam2D_Snapshot(file) saves into the session file system and returns 1 (RoboDK), the file can be read back', () => {
    const { RDK, ref } = scene();
    const fs = new MemoryFileSystem();
    RDK.fileSystem = fs;
    const cam = RDK.Cam2D_Add(ref, 'FOV=40 SIZE=320x240');
    expect(RDK.Cam2D_Snapshot('C:/Temp/v6_snap.png', cam)).toBe(1);
    const png = fs.readFile('C:/Temp/v6_snap.png');
    expect([...png.slice(0, 8)]).toEqual([0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a]);
    expect(fs.exists('C:/Temp')).toBe(true);
    expect(fs.list().map((f) => f.path)).toEqual(['C:/Temp/v6_snap.png']);
    // 06.c SimulateCamera: PATH_OPENSTATION + '/sample_image.png'
    expect(RDK.Cam2D_Snapshot(`${RDK.getParam('PATH_OPENSTATION') || 'C:/Station'}/sample_image.png`, cam)).toBe(1);
  });

  it('setColor on an AddShape plate is drawn by the camera (shape colours follow the item colour)', () => {
    const { st, RDK, ref } = scene();
    const tri = [[-50, -50, 100], [50, -50, 100], [50, 50, 100], [-50, -50, 100], [50, 50, 100], [-50, 50, 100]];
    const plate = RDK.AddShape(tri.map((p) => [...p]) as any);
    const obj = plate.item as SceneObject;
    expect(obj.geometry.length).toBeGreaterThan(0);
    const cam = RDK.Cam2D_Add(ref, 'FOV=32 SIZE=640x480 BG_COLOR=black');
    expect(redPixels(RDK.Cam2D_Snapshot('', cam) as Uint8Array)).toBe(0);
    plate.setColor([1, 0, 0, 1]);
    expect(obj.geometry.every((g) => g.color === '#ff0000')).toBe(true);
    expect(plate.Color()).toEqual([1, 0, 0, 1]);
    expect(redPixels(RDK.Cam2D_Snapshot('', cam) as Uint8Array)).toBeGreaterThan(1000);
    // Recolor(to, from): only the shapes of `from` change
    obj.geometry.push({ primitive: { kind: 'box', size: [10, 10, 10] }, color: '#00ff00' });
    plate.Recolor([0, 0, 1], [0, 1, 0], 0.05);
    expect(obj.geometry.map((g) => g.color)).toEqual([...obj.geometry.slice(0, -1).map(() => '#ff0000'), '#0000ff']);
    void st;
  });

  it('cameras are named "Camera 1", "Camera 2", ... and Cam2D_Settings uses RoboDK\'s format', () => {
    const { RDK, ref } = scene();
    const c1 = RDK.Cam2D_Add(ref, 'FOCAL_LENGHT=6 FOV=32 FAR_LENGHT=1000 SIZE=640x480 BG_COLOR=black');
    RDK.Cam2D_Add(ref, 'FOV=32 SIZE=640x480');
    const c3 = RDK.Cam2D_Add(ref, 'FOV=32 FAR_LENGTH=1000 SIZE=640x480 DEPTH');
    expect(RDK.ItemList(ITEM_TYPE_CAMERA, true)).toEqual(['Camera 1', 'Camera 2', 'Camera 3']);
    expect(RDK.Item('Camera 1', ITEM_TYPE_CAMERA).Valid()).toBe(true);
    expect(RDK.Cam2D_Settings(c1)).toBe('FOV=32.000 PixelSize=7.168635 Size=640x480 SHOW_LIMITS FOCAL_LENGTH=6.00 FAR_LENGTH=1000.00 BG_COLOR=#000000 Persp WindowResize');
    expect(RDK.Cam2D_Settings(c3)).toBe('FOV=32.000 PixelSize=5.973862 Size=640x480 SHOW_LIMITS FOCAL_LENGTH=5.00 FAR_LENGTH=1000.00 BG_COLOR=#ffffff DEPTH Persp WindowResize');
    void ItemType;
  });
});
