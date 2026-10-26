// R1 (rdk-sim): collision parity with RoboDK 5.9.4 on Library stations. Ground truth measured on RoboDK itself
// (scratch probes rdk_poses.py / rdk_collpairs.py / rdk_vprobe*.py, round 2): RDK.Collision(a, b) at given joints,
// pairs in contact on the first row of InstructionListJoints(COLLISION_ON), probe shapes at mesh vertices.
import { describe, it, expect } from 'vitest';
import { existsSync, readFileSync } from 'node:fs';
import { importRdkBestEffort } from '../src/io/robodk/rdk_container';
import { AssetStore } from '../src/scene/assets';
import { Robot } from '../src/core/items/robot';
import { Program } from '../src/core/items/program';
import { ItemType, SceneObject, Tool } from '../src/core/items/item';
import { checkCollisionsMapped, getCollisionMap } from '../src/core/collision/collision';
import { ProgramSimulator } from '../src/core/motion/simulator';

const LIB = 'C:/RoboDK/Library';
const has = (f: string) => existsSync(`${LIB}/${f}`);
async function load(f: string) {
  const b = readFileSync(`${LIB}/${f}`);
  const assets = new AssetStore();
  const { station } = await importRdkBestEffort(b.buffer.slice(b.byteOffset, b.byteOffset + b.byteLength), f, assets);
  getCollisionMap(station);
  return { station, assets };
}
const names = (pairs: ReturnType<typeof checkCollisionsMapped>) => pairs.map((p) => [p.a.item.name, p.b.item.name].sort().join(' x '));

describe('RoboDK collision parity (Library)', () => {
  const F01a = 'Example-01.a-Robot Painting - Universal Robots.rdk';
  it.skipIf(!has(F01a))('01.a: the paint gun touches Object Inspection at the RoboDK collision posture and clears it 5 deg higher (tool geometry at the inverse of its stored frame)', async () => {
    const { station, assets } = await load(F01a);
    const robot = [...station.walk()].find((i): i is Robot => i instanceof Robot)!;
    const q = [-219.536, -114.993, -76.946, -70.137, 127.814, 81.267]; // RoboDK: RDK.Collision(Paint gun, Object Inspection) = 1
    robot.setJoints(q);
    expect(names(checkCollisionsMapped(station, { assets: assets as any, meshAccurate: true }))).toContain('Object Inspection x Paint gun');
    robot.setJoints([q[0], q[1] + 5, ...q.slice(2)]); // RoboDK: 0
    expect(names(checkCollisionsMapped(station, { assets: assets as any, meshAccurate: true }))).not.toContain('Object Inspection x Paint gun');
    // and the collision-checked program reports it like RoboDK (PaintTop: collision at its first target)
    const sim = new ProgramSimulator(station);
    sim.collisionOptions = { enabled: true, assets: assets as any };
    robot.setJoints([-151.9858, -78.2225, -86.6417, -64.6022, 73.0619, 110.0317]);
    const r = sim.compile(station.itemsOfType<Program>(ItemType.PROGRAM).find((p) => p.name === 'PaintTop')!);
    expect(r.problems.find((p) => /^Collision/.test(p.message))?.message).toMatch(/Paint gun.*Object Inspection|Object Inspection.*Paint gun/);
  }, 180000);

  const F02i = 'Example-02.i-Wood engraving - Remote TCP - Motoman.rdk';
  it.skipIf(!has(F02i))('02.i: items in contact in the station as loaded are collisions on stations with RoboDK\'s map (Wood_Holder x Finish_Part1)', async () => {
    const { station, assets } = await load(F02i);
    const pairs = names(checkCollisionsMapped(station, { assets: assets as any, meshAccurate: true }));
    // RoboDK 5.9.4, start of every program: Wood_Holder x Finish_Part1, Wood_Holder x BALL END - 10MM, Finish_Part1 x BALL END - 10MM
    expect(pairs).toEqual(expect.arrayContaining(['Finish_Part1 x Wood_Holder']));
    const sim = new ProgramSimulator(station);
    sim.collisionOptions = { enabled: true, assets: assets as any };
    const r = sim.compile(station.itemsOfType<Program>(ItemType.PROGRAM).find((p) => p.name === 'Star')!);
    expect(r.problems.some((p) => p.severity === 'error' && /^Collision/.test(p.message))).toBe(true);
  }, 180000);

  const FJ = 'Curve-Following-with-JAKA-Zu5.rdk';
  it.skipIf(!has(FJ))('JAKA Zu5: the tool face on the flange face is a contact (RoboDK: collision at 0 mm, none 0.001 mm away)', async () => {
    const { station, assets } = await load(FJ);
    expect(names(checkCollisionsMapped(station, { assets: assets as any, meshAccurate: true }))).toContain('JAKA Zu5 x Tool');
    // move the tool geometry 0.01 mm away from the flange along the flange Z (RoboDK setGeometryPose: no collision)
    const tool = [...station.walk()].find((i): i is Tool => i instanceof Tool && i.name === 'Tool')!;
    const { multiply, transl } = await import('../src/core/math/pose');
    for (const g of tool.geometry) g.origin = Array.from(multiply(transl(0, 0, 0.01), Float64Array.from(g.origin as ArrayLike<number>))) as any;
    expect(names(checkCollisionsMapped(station, { assets: assets as any, meshAccurate: true }))).not.toContain('JAKA Zu5 x Tool');
  }, 180000);

  const F07c = 'Example-07.c-Script Curve Follow Custom.rdk';
  it.skipIf(!has(F07c))('07.c: a ray cast restricted to one object (ProjectPoints) reproduces RoboDK\'s projected grid point; the paint gun above the part blocks an unrestricted ray', async () => {
    const { station, assets } = await load(F07c);
    const { collisionLine } = await import('../src/core/collision/collision');
    const part = station.find('Inspection Part', ItemType.OBJECT) as SceneObject;
    const fr = station.find('Section1', ItemType.FRAME)!;
    const g = (fr.children.find((c) => c instanceof SceneObject) as SceneObject).curves.flatMap((c) => c.points)[0]; // RoboDK's projection
    const F = fr.poseAbs();
    const o: [number, number, number] = [F[0] * g[0] + F[4] * g[1] + F[12], F[1] * g[0] + F[5] * g[1] + F[13], F[2] * g[0] + F[6] * g[1] + F[14]];
    const far: [number, number, number] = [o[0] + F[8] * 1e4, o[1] + F[9] * 1e4, o[2] + F[10] * 1e4];
    const want = [F[0] * g[0] + F[4] * g[1] + F[8] * g[2] + F[12], F[1] * g[0] + F[5] * g[1] + F[9] * g[2] + F[13], F[2] * g[0] + F[6] * g[1] + F[10] * g[2] + F[14]];
    expect(collisionLine(station, o, far, { assets: assets as any, meshAccurate: true })?.item.name).toBe('Paint gun');
    const hit = collisionLine(station, o, far, { assets: assets as any, meshAccurate: true, only: part })!;
    expect(hit.item).toBe(part);
    expect(Math.hypot(hit.point[0] - want[0], hit.point[1] - want[1], hit.point[2] - want[2])).toBeLessThan(0.01);
  }, 180000);

  const F06d = 'Example-06.d-Pick and stamp.rdk';
  it.skipIf(!has(F06d))('06.d: the robot inside its cage is not in contact at load (RoboDK: no collision) — exact mesh tests within budget', async () => {
    const { station, assets } = await load(F06d);
    const t0 = performance.now();
    const pairs = names(checkCollisionsMapped(station, { assets: assets as any, meshAccurate: true }));
    expect(pairs.filter((p) => /Cage/.test(p))).toEqual([]);
    expect(performance.now() - t0).toBeLessThan(30000);
  }, 180000);

  const F06c = 'Example-06.c-Pick and place with 2D camera.rdk';
  it.skipIf(!has(F06c))('06.c: object / tool meshes sit at the inverse of their stored frame (pallet below its frame plane)', async () => {
    const { station } = await load(F06c);
    for (const it of station.walk()) {
      if (it instanceof SceneObject && it.name === 'Europallet') {
        expect(it.bbox!.max[2]).toBeLessThanOrEqual(0); // the parts are stacked on the frame plane: the pallet is below it
        expect(it.bbox!.min[2]).toBeCloseTo(-145, 0);
      }
      if (it instanceof Tool && it.name === 'GripperA') expect(Array.from(it.geometry[0].origin as ArrayLike<number>).slice(12, 15).map((v) => Math.round(v))).not.toEqual([92, 35, 30]);
    }
  }, 180000);
});
