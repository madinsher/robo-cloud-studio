/**
 * Acceptance tests on the real MURKA Webots material (webots.md §9). Skipped (visibly) when the MURKA folder is absent.
 * Ground truth: the world / PROTO files themselves (poses scanned independently of the importer), the investigator's
 * inventory (tests/fixtures/webots/murka_inventory.json: node-type counts per file) and template proof-of-concept counts.
 */
import { describe, it, expect, beforeAll } from 'vitest';
import fs from 'node:fs';
import path from 'node:path';
import { importWebotsWorld, buildWebotsProto, WebotsBuildResult, parseWorld, parseProtoFile, countTypesInText, ProtoRegistry } from '../src/io/webots';
import { parseNodes } from '../src/io/webots/vrml';
import { AssetStore } from '../src/scene/assets';
import { Item, SceneObject, Camera, Frame, GeometryRef } from '../src/core/items/item';
import { Robot } from '../src/core/items/robot';
import { MobileRobot } from '../src/mobile/items';
import { Mat4, Vec3, identity, multiply, invert, transformPoint, transformDir, fromArray, RAD } from '../src/core/math/pose';
import { HAS_MURKA, MURKA, RETAIL_V2, RETAIL_V1, dirSource, murkaSource, murkaWebotsFiles } from './webots_helpers';

const INVENTORY = JSON.parse(fs.readFileSync(path.resolve(__dirname, 'fixtures/webots/murka_inventory.json'), 'utf8')) as Record<string, { nodeTypes: Record<string, number>; urlCount: number }>;
const AGRO_ROOT = 'simulation_ws/src/agromarca/murca_garden_ws';
const colliders = (it: Item) => (it as unknown as { colliders?: GeometryRef[] }).colliders ?? [];
/** Report lines that are failures (as opposed to informative resolution notes). */
const errorsOf = (report: string[]) => report.filter((l) => /not found|cannot be resolved|unknown node type|not available offline|template|syntax|expected|recursive|not supported|skipped|failed|error/i.test(l) && !/resolved by name|texture coordinates|kinematics taken/i.test(l));

/** Split a MURKA-relative path into a source root (5 levels above the file, covering ../../../assets) and the path in it. */
function rooted(rel: string): { root: string; file: string } {
  const parts = rel.split('/');
  const k = Math.max(0, parts.length - 5);
  return { root: parts.slice(0, k).join('/'), file: parts.slice(k).join('/') };
}

/** Independent scan of the top-level nodes of a world: type, DEF, name, translation, rotation. */
function scanTopLevel(text: string): Array<{ type: string; def?: string; name?: string; translation?: number[]; rotation?: number[] }> {
  const out: Array<{ type: string; def?: string; name?: string; translation?: number[]; rotation?: number[] }> = [];
  let depth = 0, cur: (typeof out)[number] | null = null, line = '';
  const lines = text.split(/\r?\n/);
  for (const raw of lines) {
    const l = raw.replace(/"[^"]*"/g, (s) => s.replace(/[{}#]/g, '_')).replace(/#.*$/, '');
    const s = raw.trim();
    if (depth === 0) {
      const m = /^(?:DEF\s+(\S+)\s+)?([A-Z]\w*)\s*\{/.exec(s);
      if (m) { cur = { type: m[2], ...(m[1] ? { def: m[1] } : {}) }; out.push(cur); }
    } else if (depth === 1 && cur) {
      const f = /^(translation|rotation|name)\s+(.*)$/.exec(s);
      if (f) {
        if (f[1] === 'name') cur.name = /"([^"]*)"/.exec(f[2])![1];
        else cur[f[1] as 'translation' | 'rotation'] = f[2].replace(/#.*$/, '').trim().split(/\s+/).map(Number);
      }
    }
    for (const ch of l) { if (ch === '{') depth++; else if (ch === '}') depth--; }
    line = l;
  }
  void line;
  return out;
}

function expectPoseMatches(item: Item, t: number[] | undefined, r: number[] | undefined, tolMm = 1e-6) {
  const P = item.pose();
  const tt = t ?? [0, 0, 0];
  for (let k = 0; k < 3; k++) expect(Math.abs(P[12 + k] - tt[k] * 1000)).toBeLessThanOrEqual(tolMm);
  if (r && Math.abs(r[3]) > 1e-12) {
    const n = Math.hypot(r[0], r[1], r[2]);
    const ax: Vec3 = [r[0] / n, r[1] / n, r[2] / n];
    // rotation about the axis by the angle: a vector perpendicular to the axis turns by exactly that angle
    const perp: Vec3 = Math.abs(ax[2]) < 0.9 ? [0, 0, 1] : [1, 0, 0];
    const v = [perp[1] * ax[2] - perp[2] * ax[1], perp[2] * ax[0] - perp[0] * ax[2], perp[0] * ax[1] - perp[1] * ax[0]] as Vec3;
    const w = transformDir(P, v);
    const c = (v[0] * w[0] + v[1] * w[1] + v[2] * w[2]) / (v[0] ** 2 + v[1] ** 2 + v[2] ** 2);
    expect(Math.acos(Math.max(-1, Math.min(1, c)))).toBeCloseTo(Math.abs(((r[3] + Math.PI) % (2 * Math.PI)) - Math.PI), 5);
  }
}

/** Does a sphere (centre in station coordinates, mm) touch any collider of an item? */
function sphereHits(item: Item, c: Vec3, r: number): boolean {
  const P = item.poseAbs();
  for (const g of colliders(item)) {
    const p = transformPoint(invert(multiply(P, g.origin ? fromArray(g.origin) : identity())), c);
    const pr = g.primitive;
    if (!pr) continue;
    let d = Infinity;
    if (pr.kind === 'box') { const q = p.map((v, k) => Math.max(0, Math.abs(v) - pr.size[k] / 2)); d = Math.hypot(q[0], q[1], q[2]); }
    else if (pr.kind === 'cylinder') d = Math.hypot(Math.max(0, Math.hypot(p[0], p[1]) - pr.radius), Math.max(0, Math.abs(p[2]) - pr.length / 2));
    else if (pr.kind === 'sphere') d = Math.max(0, Math.hypot(p[0], p[1], p[2]) - pr.radius);
    if (d <= r) return true;
  }
  return false;
}

describe.skipIf(!HAS_MURKA)('MURKA Webots files (AT-W1 parse all)', () => {
  const files = murkaWebotsFiles();
  // 12 worlds + 27 PROTOs outside build/install copies (the report's 13 + 33 counted those copies too)
  it('finds the 12 worlds and 27 PROTO files of the inventory', () => {
    expect(files.filter((f) => f.endsWith('.wbt'))).toHaveLength(12);
    expect(files.filter((f) => f.endsWith('.proto'))).toHaveLength(27);
    expect(files).toHaveLength(Object.keys(INVENTORY).length);
    for (const f of files) expect(INVENTORY[f], f).toBeTruthy();
  });

  it('parses every file without errors and the node-type counts equal the inventory', () => {
    for (const f of files) {
      const text = fs.readFileSync(path.join(MURKA, f), 'utf8');
      let counted: Record<string, number>;
      if (f.endsWith('.wbt')) { const w = parseWorld(text, f); expect(w.nodes.length, f).toBeGreaterThan(3); counted = countTypesInText(text, f); }
      else { const p = parseProtoFile(text, f); expect(p.name, f).toBe(path.basename(f, '.proto')); counted = countTypesInText(p.body, f); }
      expect(counted, f).toEqual(INVENTORY[f].nodeTypes);
    }
  });

  it('imports every world without errors (meshes resolved, PROTOs expanded, standard PROTOs offline)', () => {
    for (const f of files.filter((x) => x.endsWith('.wbt') && !x.includes('trellis'))) {
      const { root, file } = rooted(f);
      const r = importWebotsWorld(file, dirSource(path.join(MURKA, root)), { assets: new AssetStore() });
      expect(errorsOf(r.report), f).toEqual([]);
      expect([...r.station.walk()].length, f).toBeGreaterThan(15);
    }
  }, 180000);

  it('expands every PROTO with its default fields into items', () => {
    for (const f of files.filter((x) => x.endsWith('.proto'))) {
      const { root, file } = rooted(f);
      const r = buildWebotsProto(file, dirSource(path.join(MURKA, root)), { loadMeshes: false });
      expect(errorsOf(r.report), f).toEqual([]);
      expect(r.station.children.length, f).toBe(1);
    }
  }, 120000);
});

describe.skipIf(!HAS_MURKA)('MURKA retail store v2 (AT-W2, AT-W5, AT-W6)', () => {
  let r: WebotsBuildResult;
  let assets: AssetStore;
  const worldText = HAS_MURKA ? fs.readFileSync(path.join(MURKA, RETAIL_V2, 'worlds/retail_store.wbt'), 'utf8') : '';
  beforeAll(() => {
    assets = new AssetStore();
    r = importWebotsWorld('worlds/retail_store.wbt', murkaSource(RETAIL_V2), { assets });
  }, 120000);

  it('every top-level object is present with the pose of the file', () => {
    const top = scanTopLevel(worldText).filter((n) => n.name && n.type !== 'Robot');
    expect(top.length).toBe(17); // 5 Solids, 6 shelves, 3 pallets, table, charging station, Murom
    for (const n of top) {
      const it = r.station.find(n.name!);
      expect(it, n.name).toBeTruthy();
      expectPoseMatches(it!, n.translation, n.rotation);
    }
    const names = r.station.children.map((c) => c.name);
    expect(names.filter((n) => /^(shelf_A[123]|bulk_B[123])$/.test(n)).sort()).toEqual(['bulk_B1', 'bulk_B2', 'bulk_B3', 'shelf_A1', 'shelf_A2', 'shelf_A3']);
    expect(names.filter((n) => n.startsWith('pallet_'))).toHaveLength(3);
    expect(names).toEqual(expect.arrayContaining(['floor', 'wall_north', 'wall_south', 'wall_east', 'wall_west', 'packing_table', 'charging_station']));
  });

  it('template PROTOs: shelf boards at the declared levels, counts equal the proof of concept', () => {
    const tops = (name: string) => (r.station.find(name) as SceneObject).geometry.filter((g) => g.primitive?.kind === 'box' && Math.abs((g.primitive as { size: number[] }).size[2] - 30) < 1e-9).map((g) => g.origin![14] + 15).sort((a, b) => a - b);
    expect((r.station.find('shelf_A1') as SceneObject).geometry).toHaveLength(7);
    expect((r.station.find('bulk_B1') as SceneObject).geometry).toHaveLength(6);
    tops('shelf_A1').forEach((z, i) => expect(Math.abs(z - [900, 1250, 1550][i])).toBeLessThan(1));
    tops('bulk_B1').forEach((z, i) => expect(Math.abs(z - [1150, 1650][i])).toBeLessThan(1));
    // expanded text of the instances, counted like template_poc.mjs
    const reg = new ProtoRegistry(murkaSource(RETAIL_V2));
    const w = parseWorld(worldText, 'worlds/retail_store.wbt');
    reg.registerScope('worlds/retail_store.wbt', w.externs);
    const counts = (name: string) => { const n = w.nodes.find((x) => x.fields.name?.k === 'str' && x.fields.name.v[0] === name)!; return countTypesInText(reg.expandedText(n)!); };
    expect(counts('shelf_A1')).toMatchObject({ Shape: 7, Pose: 7 });
    expect(counts('bulk_B3')).toMatchObject({ Shape: 6, Pose: 6 });
    expect(counts('charging_station')).toMatchObject({ Shape: 4, Pose: 4 });
    expect(counts('shelf_A1').Transform).toBeUndefined();
    // the charging station collider is the column only (the pad is a decal)
    expect(colliders(r.station.find('charging_station')!).map((g) => g.primitive)).toEqual([{ kind: 'box', size: [120, 600, 500] }]);
  });

  it('MUROM: differential base, 6+6 DOF arms and a 2 DOF head with Webots motor names, cameras and lidar', () => {
    const murom = r.station.find('murom') as MobileRobot;
    expect(murom).toBeInstanceOf(MobileRobot);
    expect(murom.state.x).toBeCloseTo(-7199, 6);
    expect(murom.state.y).toBeCloseTo(4500, 6);
    expect(Math.abs(Math.abs(murom.state.theta) - 180)).toBeLessThan(0.01);
    expect(murom.kin.drive).toBe('differential');
    expect(murom.kin.wheelRadius).toBeCloseTo(177, 9);
    expect(murom.kin.track).toBeCloseTo(441, 9);
    expect(murom.kin.maxSpeed).toBeCloseTo(20 * 177, 6);
    const robots = murom.childrenOfType<Robot>(2);
    const byDof = robots.map((x) => x.dof).sort();
    expect(byDof).toEqual([2, 6, 6]);
    const joints = [...(murom.params.wheelJoints as string[]), ...robots.flatMap((x) => x.jointNames())].sort();
    const motors = [...new Set([...fs.readFileSync(path.join(MURKA, RETAIL_V2, 'protos/Murom.proto'), 'utf8').matchAll(/RotationalMotor\s*\{\s*name\s+"([^"]+)"/g)].map((m) => m[1]))].sort();
    expect(motors).toHaveLength(16);
    expect(joints).toEqual(motors);
    const head = robots.find((x) => x.dof === 2)!;
    expect(head.jointNames()).toEqual(['head_pan_joint', 'head_tilt_joint']);
    head.setJoints([0, 0]);
    const cam = head.children.find((c) => c.name === 'head_camera') as Camera;
    expect(cam.kind).toBe('rgb');
    expect(cam.fovAxis).toBe('horizontal');
    expect(cam.fov).toBeCloseTo(1.2 * RAD, 9); // 68.75 deg horizontal
    expect([cam.width, cam.height]).toEqual([320, 240]);
    const view = transformDir(cam.poseAbs(), [0, 0, 1]);
    const fwd = transformDir(murom.poseAbs(), [1, 0, 0]);
    expect(view[0] * fwd[0] + view[1] * fwd[1] + view[2] * fwd[2]).toBeCloseTo(1, 9);
    const depth = head.children.find((c) => c.name === 'head_depth') as Camera;
    expect([depth.kind, depth.near, depth.far]).toEqual(['depth', 100, 5000]);
    const lidar = murom.children.find((c) => c.name === 'lidar') as Camera;
    expect(lidar.kind).toBe('lidar2d');
    expect(lidar.width).toBe(640);
    expect(lidar.fov).toBeCloseTo(4.189 * RAD, 9);
    expect([lidar.near, lidar.far]).toEqual([150, 8000]);
    expect(Array.from(lidar.pose()).slice(12, 15).map((v) => Math.round(v * 1e6) / 1e6)).toEqual([300, 0, 660]);
    expect(murom.sensors.map((s) => s.kind).sort()).toEqual(['camera', 'gnss', 'imu', 'imu', 'imu', 'lidar2d'].filter((k) => k !== 'camera'));
  });

  it('AT-W5: all Murom v2 mesh URLs resolve and load; the robot extent matches its 0.79 x 0.79 x 1.63 m envelope', () => {
    const urls = new Set([...fs.readFileSync(path.join(MURKA, RETAIL_V2, 'protos/Murom.proto'), 'utf8').matchAll(/url \[ "([^"]+\.obj)" \]/g)].map((m) => m[1]));
    expect(urls.size).toBe(110);
    for (const u of urls) {
      const a = assets.get(`protos/${u}`);
      expect(a?.mesh?.triangles, u).toBeGreaterThan(0);
    }
    expect(r.report.filter((l) => /not found/.test(l))).toEqual([]);
    const murom = r.station.find('murom') as MobileRobot;
    const [L, W, H] = murom.kin.footprint;
    expect(L).toBeGreaterThan(780); expect(L).toBeLessThan(860);
    expect(W).toBeGreaterThan(780); expect(W).toBeLessThan(820);
    expect(H).toBeGreaterThan(1630); expect(H).toBeLessThan(1700);
  });

  it('RetailProduct (IMPORTABLE, spawned by the supervisor) has no collider', () => {
    const p = buildWebotsProto('protos/RetailProduct.proto', murkaSource(RETAIL_V2), { loadMeshes: false });
    const so = p.station.children[0] as SceneObject;
    expect(so).toBeInstanceOf(SceneObject);
    expect(colliders(so)).toEqual([]);
  });
});

describe.skipIf(!HAS_MURKA)('MURKA retail v1 classic', () => {
  it('Murom v1: 12 motors -> 2 wheels + 4 + 4 DOF arms + 2 DOF head; STL meshes in their Pose wrappers', () => {
    const assets = new AssetStore();
    const r = importWebotsWorld('worlds/retail_store.wbt', murkaSource(RETAIL_V1), { assets });
    const murom = r.station.find('murom') as MobileRobot;
    expect(murom.childrenOfType<Robot>(2).map((x) => x.dof).sort()).toEqual([2, 4, 4]);
    expect((murom.params.wheelJoints as string[]).sort()).toEqual(['left_wheel_joint', 'right_wheel_joint']);
    const stl = [...assets.entries()].filter(([id]) => id.endsWith('.stl'));
    expect(stl.length).toBe(17);
    expect(errorsOf(r.report)).toEqual([]);
  }, 60000);
});

describe.skipIf(!HAS_MURKA)('MURKA orchard Mission B world (AT-W2, AT-W3, AT-W7)', () => {
  let r: WebotsBuildResult;
  let assets: AssetStore;
  const worldText = HAS_MURKA ? fs.readFileSync(path.join(MURKA, AGRO_ROOT, 'src/murca_webots/worlds/orchard_demo.wbt'), 'utf8') : '';
  beforeAll(() => {
    assets = new AssetStore();
    r = importWebotsWorld('src/murca_webots/worlds/orchard_demo.wbt', dirSource(path.join(MURKA, AGRO_ROOT)), { assets });
  }, 120000);

  it('every named top-level node is present with the pose of the file (trees, crates, depot, fences, robots)', () => {
    const top = scanTopLevel(worldText).filter((n) => n.name && !['garden_supervisor', 'capture_supervisor'].includes(n.name));
    expect(top.length).toBe(53);
    for (const n of top) {
      const it = r.station.find(n.name!);
      expect(it, n.name).toBeTruthy();
      expectPoseMatches(it!, n.translation, n.rotation);
    }
    const trees = top.filter((n) => /^apple_tree_/.test(n.name!)).length;
    expect(trees).toBe(25); // 7 + 6 + 6 + 6 in the file (the report's "26" is off by one)
    expect(r.station.children.filter((c) => /^apple_tree_s\d_\d\d$/.test(c.name))).toHaveLength(trees);
    expect((r.station.find('crate_work') as SceneObject).params.def).toBe('CRATE_WORK');
  });

  it('standard PROTOs offline: Floor 40 x 30 m with contactMaterial ground, background and light', () => {
    const floor = r.station.find('floor') as SceneObject;
    expect(floor.geometry[0].primitive).toEqual({ kind: 'plane', size: [40000, 30000] });
    expect(floor.params.contactMaterial).toBe('ground');
    expect(floor.params.ground).toBe(true);
    expect((r.station.settings.background as { texture: string }).texture).toBe('noon_cloudy_countryside');
    expect((r.station.settings.lights as unknown[]).length).toBe(1);
  });

  it('UTP-G: physical G-01 has Ackermann drive + 4 DOF arm; scripted G-02..04 keep the drive of the default variant', () => {
    const g1 = r.station.find('UTP-G-01') as MobileRobot;
    expect(g1.kin.drive).toBe('ackermann');
    expect(g1.kin.wheelBase).toBeCloseTo(2190, 6);
    expect(g1.kin.track).toBeCloseTo(1820, 6);
    expect(g1.kin.maxSteer).toBeCloseTo(0.3 * RAD, 9);
    expect(g1.kin.minTurnRadius).toBeCloseTo(2190 / Math.tan(0.3), 3);
    expect(g1.params.steerJoints).toEqual(['front_left_steer_motor', 'front_right_steer_motor']);
    const arm = g1.childrenOfType<Robot>(2)[0];
    expect(arm.jointNames()).toEqual(['arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'arm_joint_4']);
    expect(arm.chain.joints.map((j) => j.type)).toEqual(['revolute', 'revolute', 'revolute', 'prismatic']);
    expect(arm.jointLimits().upper[3]).toBeCloseTo(650, 9);
    for (const n of ['UTP-G-02', 'UTP-G-03', 'UTP-G-04']) {
      const g = r.station.find(n) as MobileRobot;
      expect(g.kin.drive, n).toBe('ackermann');
      expect(g.childrenOfType(2), n).toHaveLength(0);
      expect(g.params.kinematic, n).toBe(true);
    }
    // the UTP-G body mesh is in cm (scale 0.01 in the PROTO): scaled to mm by 10
    const body = g1.geometry.find((gr) => gr.mesh?.endsWith('vehicle/base_lod.stl'))!;
    expect(body.scale!.map((v) => Math.round(Math.abs(v) * 1e6) / 1e6)).toEqual([10, 10, 10]);
    const m = assets.get(body.mesh!)!.mesh!;
    expect(Math.max(m.max[0] - m.min[0], m.max[1] - m.min[1], m.max[2] - m.min[2]) * 10).toBeGreaterThan(2500);
    const cam = g1.children.find((c) => c.name === 'rgbd_camera') as Camera;
    expect([cam.width, cam.height, cam.fovAxis]).toEqual([640, 480, 'horizontal']);
  });

  it('template expansion counts equal the proof of concept (G-01 physical, G-02 scripted, A-01 scripted)', () => {
    const src = dirSource(path.join(MURKA, AGRO_ROOT));
    const reg = new ProtoRegistry(src);
    const wp = 'src/murca_webots/worlds/orchard_demo.wbt';
    const w = parseWorld(worldText, wp);
    reg.registerScope(wp, w.externs);
    const c = (def: string) => countTypesInText(reg.expandedText(w.nodes.find((n) => n.def === def)!)!);
    const g1 = c('UTP_G_01'), g2 = c('UTP_G_02'), a1 = c('UTP_A_01');
    expect([g1.Shape, (g1.HingeJoint ?? 0) + (g1.SliderJoint ?? 0), g1.Physics, g1.Transform]).toEqual([13, 10, 11, 43]);
    expect([g1.HingeJoint, g1.SliderJoint]).toEqual([9, 1]);
    expect([g2.Shape, g2.HingeJoint ?? 0, g2.Physics ?? 0, g2.Transform]).toEqual([3, 0, 0, 35]);
    expect([a1.Shape, a1.Transform, a1.Propeller ?? 0]).toEqual([3, 4, 0]);
  });

  it('AT-W7 colliders: trunk-only tree colliders, decals and pads never collide', () => {
    const tree = r.station.find('apple_tree_s1_01')!;
    const [x, y] = [-15000, 5000];
    expect(sphereHits(tree, [x + 400, y, 2000], 100)).toBe(false); // crown: no collider
    expect(sphereHits(tree, [x + 100, y, 500], 100)).toBe(true); // trunk
    expect(sphereHits(tree, [x + 300, y, 500], 100)).toBe(false);
    for (const n of ['depot_slot_1', 'depot_slot_2', 'charging_pad_1', 'charging_pad_2', 'path_horizontal', 'path_vertical', 'collection_point_1', 'module_sprayer']) expect(colliders(r.station.find(n)!), n).toEqual([]);
    expect(colliders(r.station.find('collection_point_2')!)).toHaveLength(1);
    // crate collider: mesh AABB box lifted by half its height
    const crate = r.station.find('crate_work')!;
    expect(colliders(crate).map((g) => g.primitive)).toEqual([{ kind: 'box', size: [760, 680, 450] }]);
    expect(sphereHits(crate, [-10800, -12500, 440], 5)).toBe(true);
    expect(sphereHits(crate, [-10800, -12500, 470], 5)).toBe(false);
    // UTP-G collision proxy: 7 boxes, the portal tunnel (|y| < 0.62 m, z < 0.62 m) is free
    const g1 = r.station.find('UTP-G-01') as MobileRobot;
    expect(colliders(g1).filter((g) => g.primitive?.kind === 'box').length).toBeGreaterThanOrEqual(7);
    const tunnel = transformPoint(g1.poseAbs(), [0, 0, 300]);
    expect(sphereHits(g1, tunnel, 100)).toBe(false);
  });

  it('the apple tree CadShape keeps its COLLADA materials and textures', () => {
    const tree = r.station.find('apple_tree_s1_01') as SceneObject;
    const ref = tree.geometry[0];
    expect(ref.color).toBeUndefined();
    const m = assets.get(ref.mesh!)!.mesh!;
    expect(m.triangles).toBeGreaterThan(18000);
    expect(m.materials!.filter((x) => x.map).map((x) => x.map!.name!.split('/').pop()).sort()).toEqual(['apples.png', 'branch.png', 'leafe.png']);
    expect(m.uvs!.length).toBe((m.positions.length / 3) * 2);
    expect(ref.scale!.map((v) => Math.round(v * 1e6) / 1e6)).toEqual([450, 450, 1000]);
  });
});

describe.skipIf(!HAS_MURKA)('MURKA legacy agro worlds (AT-W3, AT-W4)', () => {
  it('murca_garden.wbt: RectangleArena 40 x 30 m without walls', () => {
    const r = importWebotsWorld('src/murca_webots/worlds/murca_garden.wbt', dirSource(path.join(MURKA, AGRO_ROOT)), { assets: new AssetStore() });
    const arena = r.station.children.find((c) => c.params.webotsBuiltin === 'RectangleArena') as SceneObject;
    expect(arena.geometry.map((g) => g.primitive)).toEqual([{ kind: 'plane', size: [40000, 30000] }]);
    expect(colliders(arena)).toEqual([]);
    expect(errorsOf(r.report)).toEqual([]);
  });

  it('replication murca_apple_garden.wbt (R2023b, no EXTERNPROTO) resolves Floor and the backgrounds by name with warnings', () => {
    const r = importWebotsWorld('src/murca_webots/worlds/murca_apple_garden.wbt', dirSource(path.join(MURKA, 'simulation_ws/src/agromarca/article/replication')), { assets: new AssetStore() });
    for (const t of ['Floor', 'TexturedBackground', 'TexturedBackgroundLight']) expect(r.report.some((l) => l.startsWith(`${t}: no EXTERNPROTO`)), t).toBe(true);
    expect(r.station.children.some((c) => c.params.webotsBuiltin === 'Floor')).toBe(true);
    expect(r.station.settings.background).toBeTruthy();
    expect((r.station.settings.lights as unknown[]).length).toBeGreaterThanOrEqual(1);
    const text = fs.readFileSync(path.join(MURKA, 'simulation_ws/src/agromarca/article/replication/src/murca_webots/worlds/murca_apple_garden.wbt'), 'utf8');
    const fileTrees = scanTopLevel(text).filter((n) => n.name && /tree/i.test(n.name));
    expect(fileTrees.length).toBeGreaterThan(20);
    for (const n of fileTrees) expectPoseMatches(r.station.find(n.name!)!, n.translation, n.rotation);
  });
});

describe.skipIf(!HAS_MURKA)('MURKA orchard trellis stills (AT-W16)', () => {
  it('17 OBJ+MTL CadShapes load with their material colours; DEF/USE instancing and DYN_* groups are kept', () => {
    const assets = new AssetStore();
    const r = importWebotsWorld('src/murca_webots/worlds/orchard_trellis_stills.wbt', dirSource(path.join(MURKA, AGRO_ROOT)), { assets });
    expect(errorsOf(r.report)).toEqual([]);
    const objs = [...assets.entries()].filter(([id]) => id.endsWith('.obj'));
    expect(objs).toHaveLength(17);
    const multi = objs.filter(([, a]) => (a.mesh!.materials?.length ?? 0) > 1);
    expect(multi.length).toBeGreaterThan(0);
    for (const [id, a] of multi) {
      const colours = new Set(a.mesh!.materials!.filter((m) => m.color).map((m) => m.color!.map((v) => v.toFixed(3)).join(',')));
      expect(colours.size, id).toBeGreaterThanOrEqual(2);
    }
    const dyn = r.station.children.filter((c) => c instanceof Frame && /^DYN_/.test(c.name));
    expect(dyn.length).toBe(4);
    for (const d of dyn) expect(d.pose()[14]).toBeCloseTo(-1000000, 3);
  }, 180000);
});

describe.skipIf(!HAS_MURKA)('kinematic decomposition of the MURKA robots is consistent with the PROTO joint list', () => {
  it('UTP_G default variant: 10 joints = 2 steer + 4 wheels + 4 arm; every joint lands in exactly one place', () => {
    const p = buildWebotsProto('src/murca_webots/protos/UTP_G.proto', dirSource(path.join(MURKA, AGRO_ROOT)), { loadMeshes: false });
    const g = p.station.children[0] as MobileRobot;
    const all = [...(g.params.wheelJoints as string[]), ...(g.params.steerJoints as string[]), ...g.childrenOfType<Robot>(2).flatMap((x) => x.jointNames())];
    expect(new Set(all).size).toBe(10);
    expect(all.filter((n) => /wheel_motor$/.test(n))).toHaveLength(4);
    void parseNodes;
  });
});
