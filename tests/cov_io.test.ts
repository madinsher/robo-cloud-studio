import { describe, it, expect, vi, afterEach } from 'vitest';
import { gzipSync, deflateSync } from 'node:zlib';
import { zipSync, strToU8 } from 'fflate';
import * as THREE from 'three';
import { parseSTL, writeBinarySTL, writeSTL, MeshData } from '../src/io/mesh/stl';
import { parseOBJ } from '../src/io/mesh/obj';
import { parseDAE } from '../src/io/mesh/dae';
import { parseGLTF } from '../src/io/mesh/gltf';
import { importCad, isCadFile } from '../src/io/mesh/step';
import { saveStation, loadStation, bytesToBase64, base64ToBytes } from '../src/io/station-file';
import { importZipContainer } from '../src/io/containers/zip_import';
import { scanRdk, importRdkBestEffort } from '../src/io/robodk/rdk_container';
import { exportAnimatedGLB } from '../src/io/export/gltf_anim';
import { AssetStore } from '../src/scene/assets';
import { Station, SceneObject, Frame, Tool } from '../src/core/items/item';
import { Robot } from '../src/core/items/robot';
import { Program } from '../src/core/items/program';
import { ProgramSimulator } from '../src/core/motion/simulator';
import { createRobotFromLibrary } from '../src/core/items/library';
import { transl, mul, rotx, rotz, DEG, identity, getPos, toRows, fromRows } from '../src/core/math/pose';
import { demos } from '../src/demos';

// ---- occt-import-js is an optional 11 MB wasm dependency: mock it with a tiny fake reader ----
const fakeOcct = {
  calls: [] as string[],
  ReadStepFile(bytes: Uint8Array, params: any) { fakeOcct.calls.push('step'); return bytes.length ? okResult(params) : { success: false }; },
  ReadIgesFile(_b: Uint8Array, params: any) { fakeOcct.calls.push('iges'); return okResult(params); },
  ReadBrepFile(_b: Uint8Array, params: any) { fakeOcct.calls.push('brep'); return okResult(params); },
};
function okResult(params: any) {
  return {
    success: true,
    root: { name: 'assembly', children: [{ name: 'part A', children: [] }, { children: [] }] },
    meshes: [
      { name: 'cube face', color: [1, 0.5, 0], attributes: { position: { array: [0, 0, 0, 10, 0, 0, 10, 10, 0, 0, 10, 0] }, normal: { array: [0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1] } }, index: { array: [0, 1, 2, 0, 2, 3] } },
      { attributes: { position: { array: [0, 0, 0, 1, 0, 0, 0, 1, 0] } }, index: { array: [0, 1, 2] }, params },
    ],
  };
}
vi.mock('occt-import-js', () => ({ default: async (_opts: any) => fakeOcct }));

// A FileReader polyfill for three's GLTFExporter (Node has Blob but no FileReader).
class NodeFileReader {
  result: ArrayBuffer | string | null = null;
  onloadend: (() => void) | null = null;
  readAsArrayBuffer(blob: Blob) { blob.arrayBuffer().then((b) => { this.result = b; this.onloadend?.(); }); }
  readAsDataURL(blob: Blob) { blob.arrayBuffer().then((b) => { this.result = `data:${blob.type};base64,${Buffer.from(b).toString('base64')}`; this.onloadend?.(); }); }
}

const tri: MeshData = { positions: new Float32Array([0, 0, 0, 10, 0, 0, 0, 10, 0, 0, 0, 5, 10, 0, 5, 0, 10, 5]), normals: new Float32Array(18).fill(0), min: [0, 0, 0], max: [10, 10, 5], triangles: 2 };

async function makeGlb(scene: THREE.Object3D): Promise<ArrayBuffer> {
  const { GLTFExporter } = await import('three/addons/exporters/GLTFExporter.js');
  vi.stubGlobal('FileReader', NodeFileReader);
  try {
    return await new Promise<ArrayBuffer>((resolve, reject) => new GLTFExporter().parse(scene, (r) => resolve(r as ArrayBuffer), reject, { binary: true }));
  } finally {
    vi.unstubAllGlobals();
  }
}

afterEach(() => vi.unstubAllGlobals());

describe('STL', () => {
  it('parses ASCII STL with facet normals and scales', () => {
    const ascii = `solid tri\n facet normal 0 0 1\n  outer loop\n   vertex 0 0 0\n   vertex 1 0 0\n   vertex 0 1 0\n  endloop\n endfacet\nendsolid tri\n`;
    const m = parseSTL(new TextEncoder().encode(ascii).buffer as ArrayBuffer, 1000);
    expect(m.triangles).toBe(1);
    expect(Array.from(m.normals.slice(0, 3))).toEqual([0, 0, 1]);
    expect(m.max).toEqual([1000, 1000, 0]);
    expect(m.min).toEqual([0, 0, 0]);
  });
  it('writes binary STL that parses back with computed normals (both writers)', () => {
    const buf = writeBinarySTL(tri.positions, 'hdr');
    expect(buf.byteLength).toBe(84 + 2 * 50);
    const back = parseSTL(buf);
    expect(back.triangles).toBe(2);
    expect(Array.from(back.normals.slice(0, 3))).toEqual([0, 0, 1]);
    expect(back.max).toEqual([10, 10, 5]);
    const u8 = writeSTL(tri, (p) => [p[0] * 2, p[1] * 2, p[2] * 2]);
    const back2 = parseSTL(u8.buffer as ArrayBuffer);
    expect(back2.triangles).toBe(2);
    expect(back2.max).toEqual([20, 20, 10]);
    expect(new TextDecoder().decode(u8.slice(0, 18))).toBe('VerticalBot Studio');
  });
  it('detects binary files whose header starts with "solid"', () => {
    const buf = writeBinarySTL(tri.positions, 'solid but binary');
    expect(parseSTL(buf).triangles).toBe(2);
    // a degenerate triangle keeps a finite normal
    const flat = writeBinarySTL([0, 0, 0, 0, 0, 0, 0, 0, 0]);
    expect(Array.from(parseSTL(flat).normals.slice(0, 3))).toEqual([0, 0, 0]);
  });
});

describe('OBJ', () => {
  it('handles negative indices, missing normals and comments', () => {
    const obj = `# comment\n\nv 0 0 0\nv 1 0 0\nv 0 1 0\nf -3 -2 -1\nf 1/1 2/2 3/3\n`;
    const m = parseOBJ(obj, 10);
    expect(m.triangles).toBe(2);
    // face normal fallback: counter-clockwise in XY -> +Z
    expect(Array.from(m.normals.slice(0, 3))).toEqual([0, 0, 1]);
    expect(m.max).toEqual([10, 10, 0]);
  });
  it('uses vn by negative index and tolerates out-of-range vertices', () => {
    const obj = `v 0 0 0\nv 1 0 0\nv 0 1 0\nvn 1 0 0\nf 1//-1 2//-1 9//-1\n`;
    const m = parseOBJ(obj);
    expect(m.triangles).toBe(1);
    expect(Array.from(m.normals.slice(0, 3))).toEqual([1, 0, 0]);
    expect(Array.from(m.positions.slice(6, 9))).toEqual([0, 0, 0]);
  });
});

describe('COLLADA', () => {
  const src = (pos: string, prims: string, extra = '', asset = '<asset><unit meter="0.01"/><up_axis>Z_UP</up_axis></asset>') => `<?xml version="1.0"?>
<COLLADA>${asset}
<library_geometries><geometry id="g1"><mesh>
  <source id="pos"><float_array id="pa" count="12">${pos}</float_array><technique_common><accessor source="#pa" count="4" stride="3"/></technique_common></source>
  <source id="nor"><float_array id="na" count="3">0 0 1</float_array><technique_common><accessor source="#na" count="1" stride="3"/></technique_common></source>
  <vertices id="v"><input semantic="POSITION" source="#pos"/></vertices>
  ${prims}
</mesh></geometry></library_geometries>${extra}</COLLADA>`;
  const quad = '0 0 0 1 0 0 1 1 0 0 1 0';
  const inputs = '<input semantic="VERTEX" source="#v" offset="0"/><input semantic="NORMAL" source="#nor" offset="1"/>';

  it('reads polylist with unit scale and provided normals', () => {
    const m = parseDAE(src(quad, `<polylist count="1">${inputs}<vcount>4</vcount><p>0 0 1 0 2 0 3 0</p></polylist>`));
    expect(m.triangles).toBe(2);
    expect(m.max[0]).toBeCloseTo(10, 9); // 1 m * 0.01 unit -> 10 mm
    expect(Array.from(m.normals.slice(0, 3))).toEqual([0, 0, 1]);
    const raw = parseDAE(src(quad, `<polylist count="1">${inputs}<vcount>4</vcount><p>0 0 1 0 2 0 3 0</p></polylist>`), false);
    expect(raw.max[0]).toBeCloseTo(0.01, 9);
  });
  it('reads polygons, tristrips and trifans', () => {
    const p = parseDAE(src(quad, `<polygons count="1">${inputs}<p>0 0 1 0 2 0 3 0</p></polygons>`));
    expect(p.triangles).toBe(2);
    const s = parseDAE(src(quad, `<tristrips count="1"><input semantic="VERTEX" source="#v" offset="0"/><p>0 1 3 2</p></tristrips>`));
    expect(s.triangles).toBe(2);
    expect(s.normals.length).toBe(s.positions.length); // computed face normals
    const f = parseDAE(src(quad, `<trifans count="1"><input semantic="VERTEX" source="#v" offset="0"/><p>0 1 2 3</p></trifans>`));
    expect(f.triangles).toBe(2);
    expect(Array.from(f.normals.slice(0, 3)).map((v) => Math.round(v))).toEqual([0, 0, 1]);
  });
  it('applies scene node transforms (matrix/translate/rotate/scale), instance_node and Y_UP conversion', () => {
    const scene = `<library_nodes><node id="shared"><translate>0 0 1</translate><instance_geometry url="#g1"/></node></library_nodes>
      <library_visual_scenes><visual_scene id="vs"><node><matrix>1 0 0 5  0 1 0 0  0 0 1 0  0 0 0 1</matrix><scale>2 2 2</scale><rotate>0 0 1 90</rotate><rotate>1 0 0 0</rotate>
      <node><instance_node url="#shared"/></node></node></visual_scene></library_visual_scenes>`;
    const prims = `<triangles count="2">${inputs}<p>0 0 1 0 2 0 0 0 2 0 3 0</p></triangles>`;
    const m = parseDAE(src(quad, prims, scene, '<asset><unit meter="1"/><up_axis>Y_UP</up_axis></asset>'));
    expect(m.triangles).toBe(2);
    // Y_UP -> Z_UP: (x, y, z) -> (x, -z, y). Before conversion z = 2 (translate 1 scaled by 2) so y = -2000 mm
    expect(m.min[1]).toBeCloseTo(-2000, 6);
    expect(m.max[1]).toBeCloseTo(-2000, 6);
    // x: 5 + 2*rot90(x,y).x -> for vertex (1,1): x = 5 - 2 = 3 => min x 3000; max x 5000
    expect(m.min[0]).toBeCloseTo(3000, 6);
    expect(m.max[0]).toBeCloseTo(5000, 6);
  });
  it('skips primitives without a resolvable VERTEX input and geometries without mesh', () => {
    const m = parseDAE(`<COLLADA><library_geometries><geometry id="a"/><geometry id="b"><mesh><triangles><input semantic="VERTEX" source="#nope"/><p>0 1 2</p></triangles></mesh></geometry></library_geometries></COLLADA>`);
    expect(m.triangles).toBe(0);
  });
});

describe('glTF import', () => {
  it('flattens a GLB with nested transforms into mm, Z-up', async () => {
    const scene = new THREE.Scene();
    const parent = new THREE.Group();
    parent.position.set(1, 0, 0); // 1 m along X
    const mesh = new THREE.Mesh(new THREE.BoxGeometry(0.2, 0.2, 0.2));
    mesh.position.set(0, 0.5, 0); // 0.5 m up (Y-up in glTF)
    parent.add(mesh);
    scene.add(parent);
    const glb = await makeGlb(scene);
    const m = await parseGLTF(glb);
    expect(m.nodes).toBe(1);
    expect(m.triangles).toBe(12);
    // Y-up 0.5 m becomes Z-up 500 mm; box half size 100 mm
    expect(m.max[2]).toBeCloseTo(600, 3);
    expect(m.min[2]).toBeCloseTo(400, 3);
    expect(m.max[0]).toBeCloseTo(1100, 3);
    expect(m.normals.length).toBe(m.positions.length);
    const raw = await parseGLTF(glb, { scale: 1, yUpToZUp: false });
    expect(raw.max[1]).toBeCloseTo(0.6, 6);
  });
  it('keeps studio exports Z-up and handles empty scenes', async () => {
    const scene = new THREE.Scene();
    const wrap = new THREE.Group();
    wrap.name = 'VerticalBotStudio_mm_to_m';
    const mesh = new THREE.Mesh(new THREE.PlaneGeometry(1, 1));
    mesh.position.set(0, 0, 2);
    wrap.add(mesh);
    scene.add(wrap);
    const m = await parseGLTF(await makeGlb(scene), { scale: 1 });
    expect(m.max[2]).toBeCloseTo(2, 6);
    expect(m.triangles).toBe(2);
    const empty = await parseGLTF(await makeGlb(new THREE.Scene()));
    expect(empty.triangles).toBe(0);
    expect(empty.min).toEqual([0, 0, 0]);
  });
});

describe('STEP / IGES / BREP via (mocked) OpenCascade', () => {
  it('routes by extension, expands indexed meshes and decodes colours', async () => {
    const data = new Uint8Array([1, 2, 3]).buffer;
    const res = await importCad(data, 'part.STEP');
    expect(fakeOcct.calls.at(-1)).toBe('step');
    expect(res.names).toEqual(['assembly', 'part A']);
    expect(res.meshes.length).toBe(2);
    expect(res.meshes[0].name).toBe('cube face');
    expect(res.meshes[0].color).toBe('#ff8000');
    expect(res.meshes[0].mesh.triangles).toBe(2);
    expect(res.meshes[0].mesh.max).toEqual([10, 10, 0]);
    expect(Array.from(res.meshes[0].mesh.normals.slice(0, 3))).toEqual([0, 0, 1]);
    expect(res.meshes[1].name).toBe('solid 2');
    expect(res.meshes[1].color).toBeUndefined();
    expect((res.meshes[1] as any).mesh.triangles).toBe(1);
    await importCad(data, 'a.igs');
    expect(fakeOcct.calls.at(-1)).toBe('iges');
    await importCad(data, 'a.brep', { linearDeflection: 0.1, angularDeflection: 0.2 });
    expect(fakeOcct.calls.at(-1)).toBe('brep');
    await expect(importCad(new ArrayBuffer(0), 'empty.stp')).rejects.toThrow(/could not read/);
  });
  it('recognises CAD extensions', () => {
    expect(isCadFile('x.step')).toBe(true);
    expect(isCadFile('x.IGES')).toBe(true);
    expect(isCadFile('x.brp')).toBe(true);
    expect(isCadFile('x.stl')).toBe(false);
  });
});

describe('station file', () => {
  it('round-trips items and assets through base64', () => {
    const st = new Station('Cell');
    const f = st.addChild(new Frame('F'));
    f.setPose(transl(1, 2, 3));
    const obj = f.addChild(new SceneObject('Part'));
    obj.geometry = [{ mesh: 'part.stl', origin: Array.from(identity()), color: '#123456' }];
    const assets = new AssetStore();
    assets.registerRaw('part.stl', 'stl', new Uint8Array(writeBinarySTL(tri.positions)), 'part', 1, 'mm');
    assets.registerMesh('parsed-only', tri, 'nosource');
    const file = saveStation(st, assets);
    expect(file.format).toBe('vbstation');
    expect(Object.keys(file.assets)).toEqual(['part.stl']);
    expect(file.assets['part.stl'].units).toBe('mm');
    const json = JSON.parse(JSON.stringify(file));
    const assets2 = new AssetStore();
    const st2 = loadStation(json, assets2);
    expect(st2.name).toBe('Cell');
    expect(getPos(st2.find('F')!.pose())).toEqual([1, 2, 3]);
    expect(assets2.get('part.stl')?.mesh?.triangles).toBe(2);
    expect(assets2.get('part.stl')?.name).toBe('part');
    // a bare serialized tree is accepted too and no assets are required
    const st3 = loadStation(st.serialize());
    expect(st3.find('Part')).toBeTruthy();
    expect(saveStation(st).assets).toEqual({});
  });
  it('base64 helpers work with and without Buffer', () => {
    const bytes = new Uint8Array(70000).map((_, i) => i % 251);
    const b64 = bytesToBase64(bytes);
    expect(base64ToBytes(b64)).toEqual(bytes);
    vi.stubGlobal('Buffer', undefined);
    const b64b = bytesToBase64(bytes);
    expect(b64b).toBe(b64);
    expect(base64ToBytes(b64b)).toEqual(bytes);
  });
});

describe('zip container import', () => {
  it('imports meshes, metadata transforms and joints from a Visual Components style archive', async () => {
    const stl = new Uint8Array(writeBinarySTL(tri.positions));
    const obj = strToU8('v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n');
    const dae = strToU8(`<COLLADA><asset><unit meter="1"/></asset><library_geometries><geometry id="g"><mesh><source id="p"><float_array id="pa" count="9">0 0 0 1 0 0 0 1 0</float_array><technique_common><accessor source="#pa" count="3" stride="3"/></technique_common></source><vertices id="v"><input semantic="POSITION" source="#p"/></vertices><triangles count="1"><input semantic="VERTEX" source="#v" offset="0"/><p>0 1 2</p></triangles></mesh></geometry></library_geometries></COLLADA>`);
    const xml = strToU8(`<Component Name="Cell" Units="mm"><Node Name="Body" Matrix="1 0 0 0  0 1 0 0  0 0 1 0  100 200 300 1" Geometry="body.stl"/>
      <Joint Name="J1" Type="Rotational" Axis="0 0 1" Min="-90" Max="90"/><Axis Name="A2"/><Dof/></Component>`);
    const json = strToU8(JSON.stringify({ Name: 'Meta', nodes: [{ name: 'cover.dae', matrix: [1, 0, 0, 5, 0, 1, 0, 6, 0, 0, 1, 7, 0, 0, 0, 1] }] }));
    const glb = new Uint8Array(await makeGlb(new THREE.Scene().add(new THREE.Mesh(new THREE.BoxGeometry(1, 1, 1)))));
    const zip = zipSync({ 'geo/body.stl': stl, 'geo/lid.obj': obj, 'geo/cover.dae': dae, 'geo/box.glb': glb, 'component.xml': xml, 'lid.xml': strToU8('<Node Name="Lid" Position="10 20 30" Uri="lid.obj"/>'), 'meta.json': json, 'component.rsc': new Uint8Array([1, 2, 3]), 'bad.glb': new Uint8Array(10) });
    const assets = new AssetStore();
    const { folder, report } = await importZipContainer(zip, 'KUKA_gripper.vcmx', assets);
    expect(report.format).toBe('kuka-sim');
    expect(report.meshes.sort()).toEqual(['geo/body.stl', 'geo/box.glb', 'geo/cover.dae', 'geo/lid.obj']);
    expect(report.metadata).toEqual(expect.arrayContaining(['component.xml', 'meta.json']));
    expect(report.names).toEqual(expect.arrayContaining(['Cell', 'Body', 'Lid', 'J1']));
    expect(report.joints.map((j) => j.name)).toEqual(['J1', 'A2', 'joint3']);
    expect(report.joints[0]).toMatchObject({ type: 'Rotational', axis: '0 0 1', min: -90, max: 90 });
    expect(report.notes.some((n) => /rsc/.test(n))).toBe(true);
    expect(report.notes.some((n) => /joint\/axis/.test(n))).toBe(true);
    expect(report.notes.some((n) => n.startsWith('bad.glb'))).toBe(true);
    expect(folder.find('bad')).toBeNull();
    const body = folder.find('body') as SceneObject;
    expect(getPos(body.pose())).toEqual([100, 200, 300]);
    expect(body.bbox?.max).toEqual([10, 10, 5]);
    const lid = folder.find('lid') as SceneObject;
    expect(getPos(lid.pose())).toEqual([10, 20, 30]);
    const cover = folder.find('cover') as SceneObject;
    expect(getPos(cover.pose())).toEqual([5, 6, 7]);
    expect(folder.find('box')).toBeTruthy();
    expect(folder.name).toBe('KUKA_gripper');
    expect(assets.get('KUKA_gripper/geo/lid.obj')?.mesh?.triangles).toBe(1);
  });
  it('reports archives without meshes and detects metre units', async () => {
    const zip = zipSync({ 'a/readme.txt': strToU8('Units="m" nothing here'), 'thing.vcmx.xml': strToU8('<x Name="Thing"/>') });
    const { folder, report } = await importZipContainer(zip.buffer.slice(zip.byteOffset, zip.byteOffset + zip.byteLength) as ArrayBuffer, 'plain.zip', new AssetStore());
    expect(report.format).toBe('visual-components');
    expect(report.meshes).toEqual([]);
    expect(report.notes.some((n) => /No standard meshes/.test(n))).toBe(true);
    expect(folder.children.length).toBe(0);
  });
});

describe('RoboDK .rdk best-effort scan', () => {
  function payload(): Uint8Array {
    const parts: Uint8Array[] = [];
    const enc = new TextEncoder();
    const size = () => parts.reduce((a, p) => a + p.length, 0);
    const align = (n: number) => { const pad = (n - (size() % n)) % n; if (pad) parts.push(new Uint8Array(pad)); };
    const str = (s: string) => { parts.push(enc.encode(s)); parts.push(new Uint8Array([0])); };
    // the scanner walks poses in 8-byte steps and UTF-16 in 2-byte steps, so the payload keeps them aligned
    const doubles = (v: number[]) => { align(8); const b = new Uint8Array(v.length * 8); const dv = new DataView(b.buffer); v.forEach((x, i) => dv.setFloat64(i * 8, x, true)); parts.push(b); };
    str('Frame 2');
    str('Conveyor');
    // UTF-16LE name
    align(2);
    const u16 = new Uint8Array('Станция'.length * 2 + 2);
    [...'Станция'].forEach((ch, i) => { const c = ch.charCodeAt(0); u16[i * 2] = c & 0xff; u16[i * 2 + 1] = c >> 8; });
    parts.push(u16);
    // row-major pose with translation 100,200,300
    doubles([1, 0, 0, 100, 0, 1, 0, 200, 0, 0, 1, 300, 0, 0, 0, 1]);
    // column-major pose (rotz 90) with translation 5,6,7
    const m = mul(transl(5, 6, 7), rotz(90 * DEG));
    doubles(Array.from(m));
    // a non-orthonormal decoy that ends with 1
    doubles([2, 0, 0, 0, 0, 2, 0, 0, 0, 0, 2, 0, 0, 0, 0, 1]);
    // an embedded binary STL with 12 facets
    const pos: number[] = [];
    for (let i = 0; i < 12; i++) pos.push(i, 0, 0, i + 1, 0, 0, 0, 1, 0);
    parts.push(new Uint8Array(writeBinarySTL(pos)));
    const out = new Uint8Array(size());
    let o = 0;
    for (const p of parts) { out.set(p, o); o += p.length; }
    return out;
  }
  it('recovers strings, poses and meshes from a raw payload', async () => {
    const raw = payload();
    const { report } = await scanRdk(raw.buffer.slice(0) as ArrayBuffer);
    expect(report.compressed).toBe('none');
    expect(report.strings).toEqual(expect.arrayContaining(['Frame 2', 'Conveyor', 'Станция']));
    expect(report.poses).toBe(2);
    expect(report.meshes).toEqual([{ offset: expect.any(Number), triangles: 12 }]);
    const { station, report: rep2 } = await importRdkBestEffort(raw.buffer.slice(0) as ArrayBuffer, 'test.rdk');
    const rec = station.find('Recovered items')!;
    // names are paired with poses in order of appearance ("Frame 2" is kept: only bare "Frame3"-style names are dropped)
    expect(rec.children.map((c) => c.name)).toEqual(['Frame 2', 'Conveyor']);
    expect(getPos(rec.children[0].pose())).toEqual([100, 200, 300]);
    expect(getPos(rec.children[1].pose()).map((v) => Math.round(v))).toEqual([5, 6, 7]);
    expect(rep2.notes.some((n) => /Recovered 2 poses/.test(n))).toBe(true);
    expect((rep2 as any)._poses).toBeUndefined();
    expect((station.params.rdkReport as { poses: number }).poses).toBe(2);
  });
  it('inflates gzip, zlib and zlib-with-header payloads and flags zip containers', async () => {
    const raw = payload();
    const gz = gzipSync(raw);
    const r1 = await scanRdk(gz.buffer.slice(gz.byteOffset, gz.byteOffset + gz.byteLength) as ArrayBuffer);
    expect(r1.report.compressed).toBe('gzip');
    expect(r1.report.poses).toBe(2);
    const zl = deflateSync(raw);
    const r2 = await scanRdk(zl.buffer.slice(zl.byteOffset, zl.byteOffset + zl.byteLength) as ArrayBuffer);
    expect(r2.report.compressed).toBe('zlib');
    expect(r2.report.strings).toContain('Conveyor');
    const prefixed = new Uint8Array(16 + zl.length);
    prefixed.set([0x52, 0x44, 0x4b, 0x00, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12], 0);
    prefixed.set(zl, 16);
    const r3 = await scanRdk(prefixed.buffer as ArrayBuffer);
    expect(r3.report.compressed).toBe('zlib');
    expect(r3.report.notes[0]).toMatch(/zlib stream found at offset 16/);
    expect(r3.report.poses).toBe(2);
    const zip = zipSync({ 'a.txt': strToU8('hello world text') });
    const r4 = await scanRdk(zip.buffer.slice(zip.byteOffset, zip.byteOffset + zip.byteLength) as ArrayBuffer);
    expect(r4.report.compressed).toBe('zip');
    expect(r4.report.notes[0]).toMatch(/ZIP container/);
  });
});

describe('animated glTF export', () => {
  /** A stand-in for the WebGL renderer: item roots + robot link groups whose matrices follow the station. */
  function fakeRenderer(st: Station) {
    const scene = new THREE.Scene();
    const entries: Array<{ item: any; root: THREE.Group; links?: THREE.Group[]; flangeGroup?: THREE.Group }> = [];
    const toThree = (m: Float64Array) => new THREE.Matrix4().fromArray(Array.from(m));
    for (const it of st.walk()) {
      if (it === st) continue;
      const root = new THREE.Group();
      root.name = it.name;
      root.matrixAutoUpdate = false;
      root.userData.isFrame = it instanceof Frame;
      scene.add(root);
      const e: any = { item: it, root };
      if (it instanceof Robot) {
        e.links = it.chain.links.map((_, i) => { const g = new THREE.Group(); g.name = `link${i}`; g.matrixAutoUpdate = false; g.add(new THREE.Mesh(new THREE.BoxGeometry(0.1, 0.1, 0.1))); root.add(g); return g; });
        e.flangeGroup = new THREE.Group(); e.flangeGroup.matrixAutoUpdate = false; root.add(e.flangeGroup);
      } else if (it instanceof SceneObject) root.add(new THREE.Mesh(new THREE.BoxGeometry(100, 100, 100)));
      entries.push(e);
    }
    const helper = new THREE.AxesHelper(1);
    scene.add(helper);
    const ground = new THREE.Mesh(new THREE.PlaneGeometry(1, 1));
    ground.name = 'ground';
    scene.add(ground);
    const gizmo = new THREE.Group();
    gizmo.visible = true;
    return {
      scene, helper, ground, gizmo,
      transform: { getHelper: () => gizmo },
      animationTargets: () => entries,
      syncNow() {
        for (const e of entries) {
          e.root.matrix.copy(toThree(e.item instanceof Tool ? e.item.flangeAbs() : e.item.poseAbs()));
          if (e.item instanceof Robot && e.links) {
            const fk = e.item.fk();
            e.links.forEach((g: THREE.Group, i: number) => g.matrix.copy(toThree(fk.linkPoses[i] ?? fk.linkPoses[fk.linkPoses.length - 1])));
            e.flangeGroup!.matrix.copy(toThree(fk.flange));
          }
        }
      },
    };
  }
  it('samples the simulation into TRS keyframe tracks and restores the scene', async () => {
    const st = new Station('anim');
    const robot = st.addChild(createRobotFromLibrary('UR5e'));
    const tool = robot.addChild(new Tool('T'));
    tool.setPoseTool(transl(0, 0, 100));
    robot.setTool(tool);
    const part = st.addChild(new SceneObject('Part'));
    part.setPose(transl(500, 0, 0));
    st.addChild(new Frame('Ref'));
    const prog = st.addChild(new Program('Wave'));
    prog.setRobot(robot);
    prog.addMoveJ([0, -90, 90, -90, -90, 0]);
    prog.addMoveJ([30, -90, 90, -90, -90, 0]);
    const sim = new ProgramSimulator(st);
    const res = sim.compile(prog);
    expect(res.ok).toBe(true);
    expect(sim.duration).toBeGreaterThan(0);
    sim.seek(sim.duration / 2);
    const r = fakeRenderer(st);
    r.syncNow();
    const progress: number[] = [];
    vi.stubGlobal('FileReader', NodeFileReader);
    const out = await exportAnimatedGLB(r as any, sim, { fps: 10, name: 'clip', onProgress: (d) => progress.push(d) });
    expect(out.frames).toBe(Math.ceil(sim.duration * 10) + 1);
    expect(progress.length).toBe(out.frames);
    expect(out.duration).toBeCloseTo(sim.duration, 9);
    // the first link (base) never moves; the remaining links + flange do
    expect(out.tracks).toBeGreaterThanOrEqual(2);
    expect(out.glb.byteLength).toBeGreaterThan(1000);
    expect(new TextDecoder().decode(new Uint8Array(out.glb, 0, 4))).toBe('glTF');
    // scene restored: wrapper removed, helpers visible again, gizmo restored, time restored
    expect(r.scene.children.some((c) => c.name === 'VerticalBotStudio_mm_to_m')).toBe(false);
    expect(r.helper.visible).toBe(true);
    expect(r.ground.visible).toBe(true);
    expect(r.gizmo.visible).toBe(true);
    expect(sim.time).toBeCloseTo(sim.duration / 2, 9);
    // the GLB re-imports (studio export => stays Z-up) with all robot boxes
    const m = await parseGLTF(out.glb);
    expect(m.nodes).toBeGreaterThan(3);
    // the exported clip is readable by three
    const { GLTFLoader } = await import('three/addons/loaders/GLTFLoader.js');
    const gltf = await new Promise<any>((resolve, reject) => new GLTFLoader().parse(out.glb, '', resolve, reject));
    expect(gltf.animations.length).toBe(1);
    expect(gltf.animations[0].name).toBe('clip');
    expect(gltf.animations[0].tracks.length).toBe(out.tracks);
  });
  it('exports a static scene without animation tracks and honours hideHelpers=false', async () => {
    const st = demos.find((d) => d.id === 'pickplace')!.build();
    const prog = st.itemsOfType<Program>(8 as any)[0] ?? (st.find('PickPlace') as Program);
    const sim = new ProgramSimulator(st);
    sim.compile(new Program('empty'));
    const r = fakeRenderer(st);
    vi.stubGlobal('FileReader', NodeFileReader);
    const out = await exportAnimatedGLB(r as any, sim, { hideHelpers: false, scale: 1 });
    expect(out.tracks).toBe(0);
    expect(out.frames).toBe(2);
    expect(out.duration).toBe(0);
    void prog;
  });
});

export { toRows, fromRows, rotx };
