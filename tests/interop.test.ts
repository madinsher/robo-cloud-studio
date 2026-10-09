import { describe, it, expect } from 'vitest';
import { createRobotFromLibrary } from '../src/core/items/library';
import { Station, SceneObject, Tool } from '../src/core/items/item';
import { AssetStore } from '../src/scene/assets';
import { exportRobotURDF, exportStationURDF, packageZip } from '../src/io/urdf/urdf_export';
import { robotFromURDF } from '../src/io/urdf/urdf';
import { writeSTL, parseSTL } from '../src/io/mesh/stl';
import { flangePose, jointLimits } from '../src/core/kinematics/chain';
import { transl, mul, rotz, DEG, getPos } from '../src/core/math/pose';
import { importZipContainer } from '../src/io/containers/zip_import';
import { unzipSync, zipSync, strToU8 } from 'fflate';
import { postProcessRdkExport, isRdkExport } from '../src/io/robodk/rdk_import';
import { loadStation } from '../src/io/station-file';
import { ItemType } from '../src/core/items/item';

function cubeSTL(size: number): Uint8Array {
  const tris: number[][] = [];
  const s = size / 2;
  const faces = [[[-s,-s,-s],[s,-s,-s],[s,s,-s],[-s,s,-s]],[[-s,-s,s],[s,-s,s],[s,s,s],[-s,s,s]]];
  for (const f of faces) { tris.push([...f[0], ...f[1], ...f[2]], [...f[0], ...f[2], ...f[3]]); }
  const out = new Uint8Array(84 + tris.length * 50);
  new DataView(out.buffer).setUint32(80, tris.length, true);
  tris.forEach((t, i) => { const dv = new DataView(out.buffer, 84 + i * 50); t.forEach((v, k) => dv.setFloat32(12 + k * 4, v, true)); });
  return out;
}

describe('STL writer', () => {
  it('round-trips through the parser with a unit transform', () => {
    const src = parseSTL(cubeSTL(100).buffer as ArrayBuffer);
    const bytes = writeSTL(src, (p) => [p[0] / 1000, p[1] / 1000, p[2] / 1000]);
    const back = parseSTL(bytes.buffer as ArrayBuffer);
    expect(back.triangles).toBe(src.triangles);
    expect(back.max[0]).toBeCloseTo(0.05, 6);
  });
});

describe('URDF export', () => {
  it('exports a library robot and re-imports it with identical kinematics', () => {
    const r = createRobotFromLibrary('UR5e');
    const tool = new Tool('Gripper'); tool.setPoseTool(transl(0, 0, 120)); r.addChild(tool); r.setTool(tool);
    const res = exportRobotURDF(r);
    expect(res.urdf).toContain('<robot name="');
    expect(res.urdf).toContain('<joint name="Gripper_tcp_joint"');
    const back = robotFromURDF(res.urdf, {}, { tipLink: 'tool0' });
    expect(back.dof).toBe(6);
    expect(res.urdf).toContain('<link name="Gripper_tcp"/>');
    const q = [10, -70, 80, -100, -80, 20];
    const a = getPos(flangePose(r.chain, q)), b = getPos(flangePose(back.chain, q));
    for (let i = 0; i < 3; i++) expect(Math.abs(a[i] - b[i])).toBeLessThan(0.5);
    const lim = jointLimits(back.chain);
    expect(lim.lower[1]).toBeCloseTo(jointLimits(r.chain).lower[1], 3);
    const zip = packageZip(res);
    const files = Object.keys(unzipSync(zip));
    expect(files.some((f) => f.endsWith('package.xml'))).toBe(true);
    expect(files.some((f) => f.endsWith('.urdf'))).toBe(true);
  });
  it('exports a station with meshes and objects', () => {
    const st = new Station('Cell');
    const assets = new AssetStore();
    assets.registerRaw('box.stl', 'stl', cubeSTL(200), 'box.stl');
    const r = createRobotFromLibrary('KUKA_KR6_R900'); r.setPose(transl(1000, 0, 0)); st.addChild(r);
    const o = new SceneObject('Table'); o.geometry = [{ mesh: 'box.stl', origin: Array.from(transl(0, 0, 100)) }]; o.setPose(mul(transl(500, 200, 0), rotz(30 * DEG))); st.addChild(o);
    const res = exportStationURDF(st, assets);
    expect(res.warnings).toEqual([]);
    expect(Object.keys(res.files).some((f) => f.startsWith('meshes/') && f.endsWith('.stl'))).toBe(true);
    expect(res.urdf).toContain('<link name="Table">');
    expect(res.urdf).toMatch(/<joint name="KUKA_KR_6_R900_sixx_Agilus_base_joint" type="fixed">[\s\S]*xyz="1 0 0"/);
    // object mesh written in metres: 200 mm cube -> 0.1 m half size
    const stl = Object.entries(res.files).find(([f]) => f.startsWith('meshes/'))![1] as Uint8Array;
    const m = parseSTL(stl.buffer.slice(stl.byteOffset, stl.byteOffset + stl.byteLength) as ArrayBuffer);
    expect(m.max[0]).toBeCloseTo(0.1, 5);
  });
});

describe('zip container import (Visual Components style)', () => {
  it('extracts meshes, names, joints and transforms from a synthetic .vcmx', async () => {
    const xml = `<?xml version="1.0"?><Component Name="Conveyor 3m" Units="mm"><Node Name="Belt"><Geometry File="belt.stl"/><Matrix>1 0 0 0  0 1 0 0  0 0 1 0  100 200 300 1</Matrix></Node><Joint Name="Belt_Axis" Type="Translational" Axis="1 0 0" Min="0" Max="3000"/></Component>`;
    const zip = zipSync({ 'component.xml': strToU8(xml), 'geometry/belt.stl': cubeSTL(50), 'component.rsc': new Uint8Array([1, 2, 3]) });
    const assets = new AssetStore();
    const { folder, report } = await importZipContainer(zip, 'conveyor.vcmx', assets);
    expect(report.format).toBe('visual-components');
    expect(report.meshes).toEqual(['geometry/belt.stl']);
    expect(report.names).toContain('Conveyor 3m');
    expect(report.joints[0]).toMatchObject({ name: 'Belt_Axis', type: 'Translational', min: 0, max: 3000 });
    expect(folder.children.length).toBe(1);
    const p = getPos(folder.children[0].pose());
    expect(p).toEqual([100, 200, 300]);
    expect(report.notes.some((n) => /rsc/.test(n))).toBe(true);
  });
});

describe('RoboDK export import: machining projects and joint paths', () => {
  it('links machining projects, generates a curve-follow program and keeps joint paths', () => {
    const data = {
      format: 'vbstation', version: 1, assets: {},
      station: { id: 's', type: 1, name: 'RDK', pose: Array.from(transl(0, 0, 0)), visible: true, params: { source: 'rdk_export.py' }, children: [
        { id: 'r', type: 2, name: 'UR5e', pose: Array.from(transl(0, 0, 0)), visible: true, params: { rdk_type: 2, library_hint: 'UR5e' }, children: [] },
        { id: 'o', type: 5, name: 'Part', pose: Array.from(transl(500, 0, 0)), visible: true, params: {}, children: [], curves: [{ name: 'c1', points: [[0, 0, 200], [50, 0, 200], [100, 0, 200], [150, 0, 200]] }] },
        { id: 'm', type: 11, name: 'Curve follow', pose: Array.from(transl(0, 0, 0)), visible: true, params: { rdk_type: 11, robotName: 'UR5e', partName: 'Part' }, children: [] },
        { id: 'p', type: 8, name: 'Prog', pose: Array.from(transl(0, 0, 0)), visible: true, params: { rdk_type: 8, robotName: 'UR5e', instructions: [{ name: 'Set Speed (250.0 mm/s)', type: 3 }, { name: 'T1', type: 1, moveType: 1, isJointTarget: true, pose: Array.from(transl(400, 0, 400)), joints: [0, -90, 90, -90, -90, 0] }], jointsList: [[0, -90, 90, -90, -90, 0], [1, -90, 90, -90, -90, 0]] }, children: [] },
      ] },
    };
    const st = loadStation(data as any);
    expect(isRdkExport(st)).toBe(true);
    const rep = postProcessRdkExport(st);
    expect(rep.robots[0].source).toBe('library');
    expect(rep.machining[0]).toMatchObject({ name: 'Curve follow', part: 'Part', generated: true });
    expect(rep.programs[0].jointPath).toBe(2);
    const m = st.itemsOfType(ItemType.MACHINING)[0];
    expect(st.findById(String(m.getParam('programId')))).toBeTruthy();
    expect(st.itemsOfType(ItemType.PROGRAM).length).toBe(2);
  });
});
