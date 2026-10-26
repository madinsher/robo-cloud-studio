/**
 * T11: the .vbstation file is self-contained. Every mesh the tree references is embedded (source bytes or the
 * MeshData itself), unreferenced assets are left out, and a load into a fresh AssetStore (a fresh page) resolves
 * every reference to identical geometry. The RoboDK case is checked on a real Library station.
 */
import { describe, it, expect } from 'vitest';
import { existsSync, readFileSync } from 'node:fs';
import { Station, SceneObject, Tool } from '../src/core/items/item';
import { Robot } from '../src/core/items/robot';
import { AssetStore } from '../src/scene/assets';
import { saveStation, loadStation, referencedMeshIds, packValue, unpackValue } from '../src/io/station-file';
import { MeshData } from '../src/io/mesh/stl';
import { createRobotFromLibrary } from '../src/core/items/library';
import { identity } from '../src/core/math/pose';

function cube(scale = 1): MeshData {
  // 12 triangles of a unit cube (positions only matter here)
  const v = [[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0], [0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1]].map((p) => p.map((x) => x * scale));
  const f = [[0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7], [0, 1, 5], [0, 5, 4], [1, 2, 6], [1, 6, 5], [2, 3, 7], [2, 7, 6], [3, 0, 4], [3, 4, 7]];
  const positions = new Float32Array(f.flatMap((t) => t.flatMap((i) => v[i])));
  const normals = new Float32Array(positions.length).fill(0.5);
  return { positions, normals, min: [0, 0, 0], max: [scale, scale, scale], triangles: 12 };
}
function binarySTL(m: MeshData): Uint8Array {
  const n = m.triangles; const buf = new ArrayBuffer(84 + n * 50); const dv = new DataView(buf); dv.setUint32(80, n, true);
  for (let t = 0; t < n; t++) for (let k = 0; k < 9; k++) dv.setFloat32(84 + t * 50 + 12 + k * 4, m.positions[t * 9 + k], true);
  return new Uint8Array(buf);
}
const same = (a: Float32Array, b: Float32Array) => a.length === b.length && a.every((x, i) => x === b[i]);

describe('self-contained .vbstation', () => {
  it('embeds parsed-only meshes (RoboDK, glTF, CAD), scaled STL and robot link meshes; skips unreferenced assets; a fresh store resolves all', () => {
    const A = new AssetStore();
    const st = new Station('Cell');
    // RoboDK binary import style: registerMesh without source bytes
    A.registerMesh('rdk:part#0', cube(10), 'part link 0');
    const part = st.addChild(new SceneObject('Part')); part.geometry = [{ mesh: 'rdk:part#0', origin: Array.from(identity()) }];
    // glTF: parsed mesh + glb source (no synchronous re-parse) -> stored as MeshData
    const g = A.registerMesh('gripper.glb', cube(3), 'gripper.glb'); g.type = 'glb'; g.source = new Uint8Array([1, 2, 3]);
    const tool = new Tool('Gripper'); tool.geometry = [{ mesh: 'gripper.glb', origin: Array.from(identity()) }];
    // zip container STL in metres parsed with scale 1000
    A.registerRaw('zip/fixture.stl', 'stl', binarySTL(cube(0.5)), 'fixture.stl', 1000);
    const fix = st.addChild(new SceneObject('Fixture')); fix.geometry = [{ mesh: 'zip/fixture.stl', origin: Array.from(identity()) }];
    // URDF style reference by package URI resolved through the basename
    A.registerRaw('base_link.stl', 'stl', binarySTL(cube(2)), 'base_link.stl');
    const robot = st.addChild(createRobotFromLibrary('UR5e')) as Robot;
    robot.chain.links[0].visuals = [{ mesh: 'package://ur_description/meshes/base_link.stl', origin: identity() }];
    robot.addChild(tool); robot.setTool(tool);
    // stale asset from an earlier station
    A.registerMesh('stale', cube(99), 'stale');

    const refs = referencedMeshIds(st.serialize());
    expect([...refs].sort()).toEqual(['gripper.glb', 'package://ur_description/meshes/base_link.stl', 'rdk:part#0', 'zip/fixture.stl']);
    const file = saveStation(st, A);
    expect(Object.keys(file.assets).sort()).toEqual([...refs].sort());
    expect(file.assets['rdk:part#0'].type).toBe('mesh');
    expect(file.assets['gripper.glb'].type).toBe('mesh');
    expect(file.assets['zip/fixture.stl']).toMatchObject({ type: 'stl', scale: 1000 });

    // fresh page: new store, JSON text round trip
    const B = new AssetStore();
    const st2 = loadStation(JSON.parse(JSON.stringify(file)), B);
    for (const ref of refs) {
      const a = A.resolve(ref)!, b = B.resolve(ref);
      expect(b, ref).toBeTruthy();
      expect(same(a.mesh!.positions, b!.mesh!.positions), ref).toBe(true);
      expect(b!.mesh!.max).toEqual(a.mesh!.max);
    }
    expect(B.resolve('zip/fixture.stl')!.mesh!.max[0]).toBeCloseTo(500, 3); // the container scale survives the reload
    expect(B.get('stale')).toBeUndefined();
    expect((st2.find(robot.name) as Robot).chain.links[0].visuals[0].mesh).toBe('package://ur_description/meshes/base_link.stl');
  });

  it('packs typed arrays at any depth (MeshData extensions such as uvs / per-material groups / texture bytes)', () => {
    const v = { positions: new Float32Array([1.5, -2, 3]), groups: [{ start: 0, count: 3, color: '#fff' }], tex: { width: 2, data: new Uint8Array([9, 8, 7, 6]) }, idx: new Uint32Array([0, 1, 2]) };
    const back = unpackValue(JSON.parse(JSON.stringify(packValue(v)))) as typeof v;
    expect(back.positions).toBeInstanceOf(Float32Array); expect([...back.positions]).toEqual([1.5, -2, 3]);
    expect(back.tex.data).toBeInstanceOf(Uint8Array); expect([...back.tex.data]).toEqual([9, 8, 7, 6]);
    expect(back.idx).toBeInstanceOf(Uint32Array); expect(back.groups).toEqual(v.groups);
  });

  const RDK = 'C:/RoboDK/Library/Example-06.b-Pick and place 2 tables.rdk';
  it.skipIf(!existsSync(RDK))('RoboDK 06.b: every referenced mesh is saved and reloads with identical triangles in a fresh store', async () => {
    const { importRdkBestEffort } = await import('../src/io/robodk/rdk_container');
    const bytes = readFileSync(RDK);
    const A = new AssetStore();
    const { station } = await importRdkBestEffort(bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength) as ArrayBuffer, '06.b', A);
    const refs = [...referencedMeshIds(station.serialize())];
    const resolvable = refs.filter((r) => A.resolve(r));
    expect(resolvable.length).toBeGreaterThan(10); // robot links, tool, tables, balls
    expect(resolvable.reduce((n, r) => n + A.resolve(r)!.mesh!.triangles, 0)).toBeGreaterThan(10000);
    const json = JSON.stringify(saveStation(station, A));
    const B = new AssetStore();
    const st2 = loadStation(JSON.parse(json), B);
    const missing = refs.filter((r) => A.resolve(r) && !B.resolve(r));
    expect(missing).toEqual([]);
    const tri = (s: AssetStore) => resolvable.reduce((n, r) => n + s.resolve(r)!.mesh!.triangles, 0);
    expect(tri(B)).toBe(tri(A));
    expect([...st2.walk()].length).toBe([...station.walk()].length);
  }, 120000);
});
