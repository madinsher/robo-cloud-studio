/**
 * Mesh materials in the renderer's mesh creation (src/scene/three-geometry.ts): per-group file materials (OBJ+MTL,
 * COLLADA) when the geometry has no explicit colour, embedded base-colour textures with UVs, and the opacity of
 * robot link visuals (Webots transparency 1 ghost links stay invisible).
 */
import { describe, it, expect, beforeAll } from 'vitest';
import * as THREE from 'three';
import { buildGeometry, buildLinkVisuals, meshDataToGeometry } from '../src/scene/three-geometry';
import { AssetStore } from '../src/scene/assets';
import { parseOBJ } from '../src/io/mesh/obj';
import { identity } from '../src/core/math/pose';
import type { LinkVisual } from '../src/core/kinematics/chain';

const enc = (s: string) => new TextEncoder().encode(s);
const OBJ = 'mtllib m.mtl\nv 0 0 0\nv 1 0 0\nv 0 1 0\nv 1 1 0\nvt 0 0\nvt 1 0\nvt 0 1\nvt 1 1\nusemtl a\nf 1/1 2/2 3/3\nusemtl b\nf 2/2 4/4 3/3\n';
const MTL = 'newmtl a\nKd 1 0 0\nd 0.5\nnewmtl b\nKd 0 0 1\nmap_Kd t.png\n';

describe('renderer mesh materials', () => {
  let assets: AssetStore;
  beforeAll(() => {
    // minimal DOM for three's ImageLoader (the node test environment has no document)
    const g = globalThis as Record<string, unknown>;
    if (!g.document) g.document = { createElementNS: () => ({ style: {}, addEventListener() {}, removeEventListener() {}, setAttribute() {}, src: '' }) };
    (URL as unknown as { createObjectURL: (b: Blob) => string }).createObjectURL = () => 'blob:test';
    (URL as unknown as { revokeObjectURL: (u: string) => void }).revokeObjectURL = () => {};
    assets = new AssetStore();
    const files: Record<string, Uint8Array> = { 'm.mtl': enc(MTL), 't.png': new Uint8Array([137, 80, 78, 71]) };
    assets.registerMesh('two', parseOBJ(OBJ, 1, { readFile: (p) => files[p] }), 'two');
  });

  it('uses one material per OBJ group when the ref has no colour, with the MTL colour, opacity and texture', () => {
    const g = buildGeometry([{ mesh: 'two', origin: Array.from(identity()) }], assets);
    const mesh = g.children[0] as THREE.Mesh;
    expect(Array.isArray(mesh.material)).toBe(true);
    const [a, b] = mesh.material as THREE.MeshStandardMaterial[];
    expect(a.color.getHex()).toBe(0xff0000);
    expect(a.transparent && a.opacity).toBe(0.5);
    expect(b.color.b).toBe(1);
    expect(b.map).toBeInstanceOf(THREE.Texture);
    expect(mesh.geometry.groups.map((x) => [x.start, x.count, x.materialIndex])).toEqual([[0, 3, 0], [3, 3, 1]]);
    expect(mesh.geometry.getAttribute('uv').count).toBe(6);
  });

  it('an explicit colour overrides the file materials (Webots Mesh + appearance)', () => {
    const mesh = buildGeometry([{ mesh: 'two', color: '#00ff00' }], assets).children[0] as THREE.Mesh;
    expect(Array.isArray(mesh.material)).toBe(false);
    expect((mesh.material as THREE.MeshStandardMaterial).color.getHex()).toBe(0x00ff00);
  });

  it('untextured meshes are smoothed and keep their groups', () => {
    const m = parseOBJ('v 0 0 0\nv 1 0 0\nv 0 1 0\nv 1 1 0\nusemtl x\nf 1 2 3\nusemtl y\nf 2 4 3\n');
    const geo = meshDataToGeometry(m);
    expect(geo.index).not.toBeNull();
    expect(geo.groups).toHaveLength(2);
  });

  it('robot link visuals keep their opacity (transparency 1 -> invisible)', () => {
    const v: LinkVisual & { opacity?: number } = { primitive: { kind: 'box', size: [10, 10, 10] }, origin: identity(), opacity: 0 };
    const mat = (buildLinkVisuals([v], assets, '#ffffff').children[0] as THREE.Mesh).material as THREE.MeshStandardMaterial;
    expect(mat.transparent).toBe(true);
    expect(mat.opacity).toBe(0);
  });
});
