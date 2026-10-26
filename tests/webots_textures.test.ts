/**
 * Webots appearance textures on every geometry: primitives (Box, Cylinder, Cone, Sphere, Plane), IndexedFaceSet and
 * ElevationGrid get a textured mesh (Webots-like texture coordinates, the image embedded in its material); a missing
 * image is reported, never dropped silently.
 */
import { describe, it, expect } from 'vitest';
import { importWebotsFiles } from '../src/io/webots';
import { importWebotsWorld } from '../src/io/webots';
import { primitiveMesh } from '../src/io/webots/geometry';
import { AssetStore } from '../src/scene/assets';
import { SceneObject } from '../src/core/items/item';
import { dirSource, webotsCacheFetcher, WEBOTS_HOME, HAS_WEBOTS_CACHE } from './webots_helpers';

// 1x1 PNG
const PNG = Uint8Array.from(atob('iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAYAAAAfFcSJAAAADUlEQVR42mP8z8BQDwAEhQGAhKmMIQAAAABJRU5ErkJggg=='), (c) => c.charCodeAt(0));
const look = (img: string) => `PBRAppearance { baseColorMap ImageTexture { url [ "${img}" ] } roughness 1 metalness 0 }`;
const world = (img: string) => `#VRML_SIM R2025a utf8
WorldInfo { }
Solid { name "box" children [ Shape { appearance ${look(img)} geometry Box { size 1 2 3 } } ] }
Solid { name "cyl" translation 2 0 0 children [ Shape { appearance ${look(img)} geometry Cylinder { radius 0.5 height 1 } } ] }
Solid { name "cone" translation 4 0 0 children [ Shape { appearance ${look(img)} geometry Cone { bottomRadius 0.5 height 1 } } ] }
Solid { name "ball" translation 6 0 0 children [ Shape { appearance ${look(img)} geometry Sphere { radius 0.5 } } ] }
Solid { name "plane" translation 8 0 0 children [ Shape { appearance ${look(img)} geometry Plane { size 2 1 } } ] }
Solid { name "ifs" translation 10 0 0 children [ Shape { appearance ${look(img)} geometry IndexedFaceSet { coord Coordinate { point [ 0 0 0, 1 0 0, 1 1 0, 0 1 0 ] } coordIndex [ 0 1 2 3 -1 ] } } ] }
Solid { name "grid" translation 12 0 0 children [ Shape { appearance ${look(img)} geometry ElevationGrid { xDimension 3 yDimension 3 xSpacing 0.5 ySpacing 0.5 height [ 0 0 0 0 1 0 0 0 0 ] } } ] }
`;
const enc = (s: string) => new TextEncoder().encode(s);

describe('Webots textures', () => {
  it('every textured geometry becomes a textured mesh carrying the image', async () => {
    const assets = new AssetStore();
    const { station, report } = await importWebotsFiles(new Map([['w/worlds/t.wbt', enc(world('tex.png'))], ['w/worlds/tex.png', PNG]]), assets);
    const objs = [...station.walk()].filter((i): i is SceneObject => i instanceof SceneObject);
    expect(objs.map((o) => o.name).sort()).toEqual(['ball', 'box', 'cone', 'cyl', 'grid', 'ifs', 'plane']);
    for (const o of objs) {
      expect(o.geometry.length, o.name).toBe(1);
      const g = o.geometry[0]; expect(g.mesh, o.name).toMatch(/#tex=/);
      const m = assets.get(g.mesh!)!.mesh!; expect(m.uvs, o.name).toBeDefined(); expect(m.materials?.[0].map?.data.length, o.name).toBe(PNG.length);
      // primitives keep their exact shape (collision, comparisons); the textured mesh is what is drawn
      if (!['ifs', 'grid'].includes(o.name)) expect(g.primitive?.kind, o.name).toBe({ box: 'box', cyl: 'cylinder', cone: 'cone', ball: 'sphere', plane: 'plane' }[o.name]);
      expect(g.color, o.name).toBeUndefined();
    }
    // box 1 x 2 x 3 m -> mm extents kept
    const box = objs.find((o) => o.name === 'box')!; const bm = assets.get(box.geometry[0].mesh!)!.mesh!;
    expect(bm.max.map((v, k) => (v - bm.min[k]) * box.geometry[0].scale![k])).toEqual([1000, 2000, 3000]);
    expect(report.some((l) => /image not found/.test(l))).toBe(false);
  });
  it('a missing texture image is reported and the base colour is kept', async () => {
    const { station, report } = await importWebotsFiles(new Map([['w/worlds/t.wbt', enc(world('missing.png'))]]), new AssetStore());
    expect(report.some((l) => /texture missing\.png on Box: image not found/.test(l))).toBe(true);
    const box = [...station.walk()].find((i) => i.name === 'box') as SceneObject; expect(box.geometry[0].primitive?.kind).toBe('box');
  });
  it('primitive meshes have outward faces and texture coordinates in [0, 1]', () => {
    for (const kind of ['box', 'cylinder', 'cone', 'sphere', 'plane'] as const) {
      const m = primitiveMesh(kind, { size: [1, 1, 1], radius: 0.5, height: 1 });
      expect(m.uvs!.every((v) => v >= -1e-9 && v <= 1 + 1e-9), kind).toBe(true);
      // outward: normal . centroid-of-triangle >= 0 (closed solids centred at the origin; the plane faces +Z)
      for (let i = 0; i < m.positions.length; i += 9) {
        const c = [0, 1, 2].map((a) => (m.positions[i + a] + m.positions[i + 3 + a] + m.positions[i + 6 + a]) / 3);
        const n = [m.normals[i], m.normals[i + 1], m.normals[i + 2]];
        expect(kind === 'plane' ? n[2] : n[0] * c[0] + n[1] * c[1] + n[2] * c[2], kind).toBeGreaterThanOrEqual(-1e-9);
      }
    }
  });
});

describe.skipIf(!HAS_WEBOTS_CACHE)('Webots sample: textured_shapes.wbt (local Webots cache)', () => {
  it('the textured cone, box, sphere, cylinder and plane keep their textures', () => {
    const assets = new AssetStore();
    const r = importWebotsWorld('samples/geometries/worlds/textured_shapes.wbt', dirSource(`${WEBOTS_HOME}/projects`), { assets, remote: webotsCacheFetcher() });
    const textured = [...r.station.walk()].filter((i): i is SceneObject => i instanceof SceneObject).flatMap((o) => o.geometry).filter((g) => g.mesh && /#tex=/.test(g.mesh) && assets.get(g.mesh)?.mesh?.materials?.[0]?.map);
    expect(textured.length, r.report.filter((l) => /texture/.test(l)).join('\n')).toBeGreaterThanOrEqual(5);
  }, 120000);
});
