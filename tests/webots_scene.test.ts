/**
 * Webots world import on a committed synthetic fixture (tests/fixtures/webots/mini): station settings, Solids with
 * Transform scale / DEF-USE / nested Solids, boundingObject colliders, template PROTO expansion, the RectangleArena
 * built-in, meshes with materials and textures, and a differential robot decomposed into MobileRobot + Robots + cameras.
 * Expected values are computed by hand from the fixture text (metres -> mm, radians -> degrees).
 */
import { describe, it, expect, beforeAll } from 'vitest';
import { importWebotsWorld, importWebotsFiles, WebotsBuildResult, parseShots, applyShot, viewpointLook } from '../src/io/webots';
import { AssetStore } from '../src/scene/assets';
import { Item, SceneObject, Camera, Frame, Notes, GeometryRef } from '../src/core/items/item';
import { Robot } from '../src/core/items/robot';
import { MobileRobot } from '../src/mobile/items';
import { Mat4, multiply, transformDir, transformPoint, rotz, transl, getPos, RAD } from '../src/core/math/pose';
import { dirSource, FIXTURES } from './webots_helpers';
import fs from 'node:fs';
import path from 'node:path';

const colliders = (it: Item) => (it as unknown as { colliders?: GeometryRef[] }).colliders;
const byName = (root: Item, name: string) => { const f = root.find(name); if (!f) throw new Error(`no item ${name}`); return f; };
const pos = (o: number[] | Mat4 | undefined) => (o ? [o[12], o[13], o[14]] : [0, 0, 0]);
const close = (a: number[], b: number[], tol = 1e-6) => a.every((v, i) => Math.abs(v - b[i]) <= tol);

describe('Webots mini world', () => {
  let r: WebotsBuildResult;
  let assets: AssetStore;
  beforeAll(() => {
    assets = new AssetStore();
    r = importWebotsWorld('mini/worlds/mini.wbt', dirSource(FIXTURES), { assets });
  });

  it('maps WorldInfo, Viewpoint, background and lights to station settings', () => {
    const s = r.station;
    expect(s.name).toBe('Mini world');
    expect((s.settings.webots as any).basicTimeStep).toBe(16);
    expect((s.settings.webots as any).version).toBe('R2025a');
    const notes = s.children.find((c) => c instanceof Notes) as Notes;
    expect(notes.text).toBe('line one # not a comment\nline two');
    const vp = s.settings.viewpoint as any;
    expect(vp.position).toEqual([5000, 0, 2000]);
    expect(vp.follow).toBe('bot');
    expect((s.settings.background as any).texture).toBe('noon_cloudy_countryside');
    const lights = s.settings.lights as any[];
    expect(lights).toHaveLength(1);
    expect(lights[0].direction[2]).toBeCloseTo(-1, 9);
    expect(lights[0].intensity).toBe(1.5);
    // TexturedBackground has no EXTERNPROTO line: resolved by name with a visible warning
    expect(r.report.some((l) => l.startsWith('TexturedBackground: no EXTERNPROTO') && l.includes('built-in'))).toBe(true);
  });

  it('flattens Transform scale into the box size and keeps DEF/USE, nested Solids, colliders and mass', () => {
    const box = byName(r.station, 'box_a') as SceneObject;
    expect(pos(box.pose())).toEqual([1000, 500, 250]);
    const xAxis = transformDir(box.pose(), [1, 0, 0]);
    expect(close(xAxis, [0, 1, 0], 1e-12)).toBe(true);
    expect(box.geometry).toHaveLength(1);
    expect(box.geometry[0].primitive).toEqual({ kind: 'box', size: [200, 200, 300] });
    expect(pos(box.geometry[0].origin)).toEqual([0, 0, 100]);
    expect(box.geometry[0].color).toBe('#ff0000');
    expect(box.geometry[0].opacity).toBeCloseTo(0.75, 12);
    expect(box.mass).toBe(2.5);
    expect(box.params.def).toBe('BOX_A');
    const lid = box.children.find((c) => c.name === 'lid') as SceneObject;
    expect(pos(lid.pose())).toEqual([0, 0, 550]);
    expect(lid.geometry[0].primitive).toEqual({ kind: 'box', size: [100, 200, 300] }); // USE of the unscaled shape
    const c = colliders(box)!;
    expect(c.map((g) => g.primitive)).toEqual([{ kind: 'box', size: [200, 200, 300] }, { kind: 'sphere', radius: 50 }]);
    expect(pos(c[0].origin)).toEqual([0, 0, 100]);
    // a Solid without boundingObject has no collision geometry (Webots decal semantics)
    const decal = byName(r.station, 'decal');
    expect(colliders(decal)).toEqual([]);
  });

  it('expands the template PROTO per instance (for loop, if, enumerated string)', () => {
    const x = byName(r.station, 'shelf_x') as SceneObject;
    expect(x.params.proto).toBe('Shelf');
    expect(x.geometry).toHaveLength(4);
    const boards = x.geometry.filter((g) => g.primitive?.kind === 'box' && (g.primitive as any).size[2] === 20);
    const tops = boards.map((g) => pos(g.origin)[2] + 10);
    expect(tops.map((v) => Math.round(v * 1000) / 1000)).toEqual([400, 800, 1200]);
    expect(boards[0].color).toBe('#b3b3b3');
    expect(colliders(x)).toHaveLength(4); // boundingObject USE BODY (the Group defined in children)
    const y = byName(r.station, 'shelf_y') as SceneObject;
    expect(y.geometry).toHaveLength(1); // showBack FALSE
    expect(y.geometry[0].color).toBe('#996633'); // material "wood"
    expect(transformDir(y.pose(), [1, 0, 0])[0]).toBeCloseTo(-1, 4);
  });

  it('replaces the webots:// RectangleArena by the built-in equivalent: floor plane + 4 walls outside the floor', () => {
    const a = byName(r.station, 'rectangle arena') as SceneObject;
    expect(a.params.ground).toBe(true);
    expect(a.geometry[0].primitive).toEqual({ kind: 'plane', size: [4000, 3000] });
    const walls = colliders(a)!;
    expect(walls).toHaveLength(4); // the ground plane is not a collider
    const centers = walls.map((w) => pos(w.origin));
    expect(centers).toContainEqual([2050, 0, 250]);
    expect(centers).toContainEqual([0, -1550, 250]);
    expect(walls.map((w) => (w.primitive as any).size)).toContainEqual([4200, 100, 500]);
  });

  it('registers meshes in metres with mm scale, OBJ+MTL materials, COLLADA textures and IndexedFaceSet', () => {
    const m = byName(r.station, 'meshes') as SceneObject;
    const [stl, obj, dae, ifs] = m.geometry;
    expect(stl.mesh).toBe('mini/meshes/tri.stl');
    expect(stl.scale!.map((v) => Math.round(v * 1e6) / 1e6)).toEqual([10, 20, 30]);
    expect(close(transformDir(Float64Array.from(stl.origin!), [0, 1, 0]), [0, 0, 1], 1e-9)).toBe(true);
    expect(stl.color).toBe('#00ff00');
    expect(assets.get('mini/meshes/tri.stl')!.mesh!.max).toEqual([1, 1, 0]);
    // CadShape: no colour on the ref, the file materials apply
    expect(obj.color).toBeUndefined();
    const om = assets.get(obj.mesh!)!.mesh!;
    expect(om.materials!.map((x) => x.name)).toEqual(['red', 'blue']);
    expect(om.materials![0].color).toEqual([1, 0, 0]);
    expect(om.materials![0].opacity).toBe(0.5);
    expect(om.materials![1].emissive).toEqual([0, 0, 0.2]);
    expect(om.groups).toEqual([{ start: 0, count: 6, material: 0 }, { start: 6, count: 3, material: 1 }]);
    expect(om.uvs!.length).toBe(18);
    const dm = assets.get(dae.mesh!)!.mesh!;
    expect(dm.materials!.map((x) => x.name)).toEqual(['leaves', 'bark']);
    expect(dm.materials![0].map!.data.length).toBe(fs.statSync(path.join(FIXTURES, 'mini/meshes/tex.png')).size);
    expect(dm.materials![0].map!.mime).toBe('image/png');
    expect(dm.materials![1].color![1]).toBeCloseTo(0.8, 9);
    expect(dm.materials![1].opacity).toBeCloseTo(0.6, 9);
    expect(Array.from(dm.uvs!.slice(0, 6))).toEqual([0, 0, 1, 0, 1, 1]);
    expect(dm.min[2]).toBeCloseTo(0.1, 6); // node translate kept, metres
    const im = assets.get(ifs.mesh!)!.mesh!;
    expect(im.triangles).toBe(2);
    expect(im.max).toEqual([1, 1, 0]);
  });

  it('decomposes the robot into a differential MobileRobot, a lift and a pan/tilt head with the camera on its flange', () => {
    const bot = byName(r.station, 'bot') as MobileRobot;
    expect(bot).toBeInstanceOf(MobileRobot);
    expect(bot.kin.drive).toBe('differential');
    expect(bot.kin.track).toBeCloseTo(360, 9);
    expect(bot.kin.wheelRadius).toBeCloseTo(50, 9);
    expect(bot.kin.maxSpeed).toBeCloseTo(500, 9); // 10 rad/s * 50 mm
    expect(bot.state.theta).toBeCloseTo(0.5 * RAD, 9);
    expect(bot.params.wheelJoints).toEqual(['left_wheel_motor', 'right_wheel_motor']);
    expect(colliders(bot)!.length).toBe(4); // body box, two wheel cylinders, caster sphere
    const robots = bot.children.filter((c) => c instanceof Robot) as Robot[];
    expect(robots.map((x) => x.name).sort()).toEqual(['bot_head', 'bot_lift']);
    const lift = robots.find((x) => x.name === 'bot_lift')!;
    expect(lift.chain.joints[0].type).toBe('prismatic');
    expect(lift.jointLimits()).toEqual({ lower: [0], upper: [100] });
    expect(lift.joints()[0]).toBeCloseTo(20, 9);
    // the stored slider position is undone: zero pose 20 mm lower than the endPoint translation
    expect(pos(lift.chain.joints[0].origin).map((v) => Math.round(v * 1e6) / 1e6)).toEqual([-100, 0, 250]);
    const head = robots.find((x) => x.name === 'bot_head')!;
    expect(head.jointNames()).toEqual(['pan', 'tilt']);
    expect(head.joints()[0]).toBeCloseTo(0.5 * RAD, 9);
    expect(head.jointLimits().lower[1]).toBeCloseTo(-0.5 * RAD, 9);
    expect(head.jointLimits().upper[1]).toBeCloseTo(0.7 * RAD, 9);
    expect(head.chain.joints[0].maxVelocity).toBeCloseTo(2 * RAD, 9);
    // FK at the initial joints reproduces the stored neck pose (translation 0.1 0 0.25, rotation 0.5 rad about Z)
    const neck = head.fk(head.joints()).linkPoses[1];
    const want = multiply(transl(100, 0, 250), rotz(0.5));
    for (let i = 0; i < 16; i++) expect(neck[i]).toBeCloseTo(want[i], 9);
    // camera: on the head flange, Webots +X optical axis -> studio +Z
    const cam = head.children.find((c) => c instanceof Camera) as Camera;
    expect(cam.name).toBe('cam');
    expect(cam.fov).toBeCloseTo(1.2 * RAD, 9);
    expect(cam.fovAxis).toBe('horizontal');
    expect([cam.width, cam.height, cam.near, cam.far]).toEqual([320, 240, 50, 20000]);
    const view = transformDir(cam.poseAbs(), [0, 0, 1]);
    expect(close(view, [Math.cos(1), Math.sin(1), 0], 1e-9)).toBe(true); // mobile yaw 0.5 + pan 0.5
    const down = transformDir(cam.poseAbs(), [0, 1, 0]);
    expect(close(down, [0, 0, -1], 1e-9)).toBe(true); // image +Y points down
    const camPos = getPos(cam.poseAbs());
    const neckAbs = multiply(bot.poseAbs(), neck);
    expect(close(camPos, transformPoint(neckAbs, [50, 0, 70]), 1e-6)).toBe(true);
    // lidar on the base: Camera item + MobileRobot.sensors entry
    const lidar = bot.children.find((c) => c instanceof Camera && c.name === 'lidar') as Camera;
    expect(lidar.kind).toBe('lidar2d');
    expect([lidar.width, lidar.near, lidar.far]).toEqual([360, 100, 5000]);
    expect(bot.sensors.map((s) => s.kind).sort()).toEqual(['gnss', 'lidar2d']);
    // the passive flap is fixed and reported
    expect(r.report.some((l) => l.includes('without motor (flap)'))).toBe(true);
  });

  it('reports unknown node types and imports supervisor-only robots as frames', () => {
    expect(r.report.some((l) => l.startsWith('UnknownThing: unknown node type'))).toBe(true);
    const sup = byName(r.station, 'sup');
    expect(sup).toBeInstanceOf(Frame);
    expect(sup.params.supervisor).toBe(true);
    expect(sup.params.controller).toBe('<extern>');
  });
});

describe('importWebotsFiles (I7)', () => {
  const read = (p: string) => new Uint8Array(fs.readFileSync(path.join(FIXTURES, p)));
  const enc = (s: string) => new TextEncoder().encode(s);

  it('resolves a PROTO by name when the world has no EXTERNPROTO line and keeps importing after errors', async () => {
    const files = new Map<string, Uint8Array>([
      ['proj/protos/Shelf.proto', read('mini/protos/Shelf.proto')],
      ['proj/worlds/w.wbt', enc('#VRML_SIM R2023b utf8\nShelf { name "s1" levels [ 0.3 ] }\nMissingProto { }\nSolid { name "after" }\n')],
    ]);
    const { station, report } = await importWebotsFiles(files, new AssetStore());
    expect(station.find('s1')).toBeInstanceOf(SceneObject);
    expect((station.find('s1') as SceneObject).geometry).toHaveLength(2);
    expect(station.find('after')).toBeTruthy();
    expect(report.some((l) => l.includes('Shelf: no EXTERNPROTO') && l.includes('proj/protos/Shelf.proto'))).toBe(true);
    expect(report.some((l) => l.startsWith('MissingProto: unknown node type'))).toBe(true);
  });

  it('previews PROTO files when no world is given', async () => {
    const files = new Map<string, Uint8Array>([['DiffBot.proto', read('mini/protos/DiffBot.proto')]]);
    const { station } = await importWebotsFiles(files);
    const bot = station.find('diffbot');
    expect(bot).toBeInstanceOf(MobileRobot);
    expect((bot as MobileRobot).childrenOfType(2 /* ROBOT */).length).toBe(2);
  });

  it('throws a located syntax error for a broken world', async () => {
    const files = new Map<string, Uint8Array>([['w.wbt', enc('#VRML_SIM R2025a utf8\nSolid {\n  translation 0 0\n  children [ Shape { ]\n}\n')]]);
    await expect(importWebotsFiles(files)).rejects.toThrow(/w\.wbt:4/);
  });
});

describe('viewpoints and shots (synthetic)', () => {
  it('a Webots viewpoint looks along its +X with +Z up; shots park every DYN_ group but the selected one', async () => {
    const look = viewpointLook({ position: [1000, 0, 500], orientation: [0, 0, 1, Math.PI / 2], fieldOfView: Math.PI / 3 });
    expect(look.target.map((v) => Math.round(v))).toEqual([1000, 1000, 500]);
    expect(look.up.map((v) => Math.round(v * 1e9) / 1e9)).toEqual([0, 0, 1]);
    expect(look.fovDeg).toBeCloseTo(60, 12);
    const world = `#VRML_SIM R2025a utf8
Viewpoint { orientation 0 0 1 0 position -3 0 1 }
DEF DYN_A Transform { translation 1 2 -1000 children [ Shape { geometry Box { size 1 1 1 } } ] }
DEF DYN_B Transform { translation 3 4 -1000 children [ Shape { geometry Box { size 1 1 1 } } ] }
`;
    const { station } = await importWebotsFiles(new Map([['w.wbt', new TextEncoder().encode(world)]]), new AssetStore());
    const shots = parseShots({ shots: [{ key: 'b', position: [0, 0, 2], orientation: [0, 1, 0, 0.5], fov: 1.1 }] });
    const { shown, parked } = applyShot(station, shots[0]);
    expect(shown!.name).toBe('DYN_B');
    expect(Array.from(shown!.poseAbs()).slice(12, 15)).toEqual([3000, 4000, 0]);
    expect(parked.map((p) => [p.name, p.poseAbs()[14]])).toEqual([['DYN_A', -1000000]]);
    expect(station.settings.viewpoint).toMatchObject({ position: [0, 0, 2000], orientation: [0, 1, 0, 0.5], fieldOfView: 1.1, shot: 'b' });
    expect(() => parseShots({ shots: [{ key: 'x', position: [0, 0] }] })).toThrow(/position/);
  });
});

describe('pass-through and Solid-derived nodes (synthetic)', () => {
  it('Slot plugs its endPoint, Charger / Connector are Solids, a Fluid volume never collides, nested lights become station lights', async () => {
    const world = `#VRML_SIM R2025a utf8
Solid { name "cabinet" translation 1 0 0 children [
  Slot { type "door" endPoint Solid { name "door" translation 0 0.5 0 children [ Shape { geometry Box { size 0.1 1 2 } } ] boundingObject Box { size 0.1 1 2 } } }
  PointLight { location 0 0 2 intensity 3 }
] }
Charger { name "dock" translation 0 2 0 children [ Shape { geometry Cylinder { radius 0.2 height 0.1 } } ] boundingObject Cylinder { radius 0.2 height 0.1 } }
Fluid { name "pool" translation 5 0 -1 children [ Shape { geometry Box { size 4 4 2 } } ] boundingObject Box { size 4 4 2 } density 1025 }
`;
    const { station, report } = await importWebotsFiles(new Map([['w.wbt', new TextEncoder().encode(world)]]), new AssetStore());
    const door = station.find('door') as SceneObject;
    expect(door.parent!.name).toBe('cabinet');
    expect(Array.from(door.poseAbs()).slice(12, 15)).toEqual([1000, 500, 0]);
    expect(colliders(door)![0].primitive).toEqual({ kind: 'box', size: [100, 1000, 2000] });
    const dock = station.find('dock') as SceneObject;
    expect([dock.params.webotsType, colliders(dock)!.length]).toEqual(['Charger', 1]);
    const pool = station.find('pool') as SceneObject;
    expect([pool.params.fluid, pool.params.density, colliders(pool)]).toEqual([true, 1025, []]);
    expect(pool.geometry[0].primitive).toEqual({ kind: 'box', size: [4000, 4000, 2000] });
    expect(station.settings.lights).toEqual([{ type: 'PointLight', intensity: 3, color: [1, 1, 1], ambientIntensity: 0, castShadows: false, location: [1000, 0, 2000], attached: true }]);
    expect(report.filter((l) => /ignored|not supported/.test(l))).toEqual([]);
  });
});
