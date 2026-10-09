import { describe, it, expect } from 'vitest';
import { Station, SceneObject } from '../src/core/items/item';
import { createRobotFromLibrary } from '../src/core/items/library';
import { checkCollisionsMapped, setCollisionPair, getCollisionMap, collisionLine, itemsCollide, MeshBVH } from '../src/core/collision/collision';
import { AssetStore } from '../src/scene/assets';
import { transl, identity } from '../src/core/math/pose';

function boxMesh(s: number): Float32Array {
  const h = s / 2;
  const v = [[-h, -h, -h], [h, -h, -h], [h, h, -h], [-h, h, -h], [-h, -h, h], [h, -h, h], [h, h, h], [-h, h, h]];
  const f = [[0, 1, 2], [0, 2, 3], [4, 6, 5], [4, 7, 6], [0, 4, 5], [0, 5, 1], [1, 5, 6], [1, 6, 2], [2, 6, 7], [2, 7, 3], [3, 7, 4], [3, 4, 0]];
  return new Float32Array(f.flatMap((t) => t.flatMap((i) => v[i])));
}

describe('collision map, ray casting and BVH', () => {
  it('collision map disables and re-enables pairs; ray hits objects', () => {
    const st = new Station();
    const a = st.addChild(new SceneObject('A'));
    a.geometry = [{ primitive: { kind: 'box', size: [200, 200, 200] }, origin: Array.from(identity()) }];
    const b = st.addChild(new SceneObject('B'));
    b.geometry = [{ primitive: { kind: 'box', size: [200, 200, 200] }, origin: Array.from(identity()) }];
    b.setPose(transl(150, 0, 0));
    expect(checkCollisionsMapped(st).length).toBe(1);
    setCollisionPair(st, a, b, -1, -1, false);
    expect(checkCollisionsMapped(st).length).toBe(0);
    setCollisionPair(st, a, b, -1, -1, true);
    expect(checkCollisionsMapped(st).length).toBe(1);
    getCollisionMap(st).active = false;
    expect(checkCollisionsMapped(st).length).toBe(0);
    getCollisionMap(st).active = true;
    const hit = collisionLine(st, [-1000, 0, 0], [1000, 0, 0]);
    expect(hit?.item).toBe(a);
    expect(hit?.point[0]).toBeCloseTo(-100, 3);
    expect(collisionLine(st, [0, 1000, 500], [0, -1000, 500])).toBeNull();
    expect(itemsCollide(a, b).length).toBe(1);
  });
  it('BVH mesh/mesh and ray tests', () => {
    const assets = new AssetStore();
    assets.registerMesh('cube', { positions: boxMesh(100), normals: new Float32Array(0), min: [-50, -50, -50], max: [50, 50, 50], triangles: 12 });
    const st = new Station();
    const a = st.addChild(new SceneObject('A'));
    a.geometry = [{ mesh: 'cube', origin: Array.from(identity()) }];
    const b = st.addChild(new SceneObject('B'));
    b.geometry = [{ mesh: 'cube', origin: Array.from(identity()) }];
    b.setPose(transl(0, 0, 130));
    // bounding boxes of rotated meshes may overlap while triangles do not: here boxes don't overlap either
    expect(itemsCollide(a, b, { assets, meshAccurate: true }).length).toBe(0);
    b.setPose(transl(60, 60, 60));
    expect(itemsCollide(a, b, { assets, meshAccurate: true }).length).toBe(1);
    const bvh = new MeshBVH(boxMesh(100), transl(0, 0, 0));
    expect(bvh.raycast([0, 0, 500], [0, 0, -1])).toBeCloseTo(450, 6);
    const r = st.addChild(createRobotFromLibrary('UR5e'));
    expect(collisionLine(st, [-2000, 0, 80], [2000, 0, 80])?.item).toBe(r);
  });
});
