import { describe, it, expect } from 'vitest';
import { readFileSync, existsSync } from 'node:fs';
import { importRdkBestEffort } from '../src/io/robodk/rdk_container';
import { AssetStore } from '../src/scene/assets';
import { Robot } from '../src/core/items/robot';
import { Program } from '../src/core/items/program';
import { SceneObject, Tool, Target, ItemType } from '../src/core/items/item';
import { getPos, invert, multiply } from '../src/core/math/pose';

const lib = 'C:/RoboDK/Library';
const file = (name: string) => `${lib}/${name}`;

/** Library file as an ArrayBuffer (tests that need one are skipped, visibly, when C:/RoboDK is absent). */
function load(name: string): ArrayBuffer {
  const buf = readFileSync(file(name));
  return buf.buffer.slice(buf.byteOffset, buf.byteOffset + buf.byteLength);
}
const has = (...names: string[]) => names.every((n) => existsSync(file(n)));

describe('RoboDK container import', () => {
  it.skipIf(!has('ABB-IRB-1600-6-1-45.robot'))('builds the ABB IRB 1600 from its .robot file', async () => {
    const data = load('ABB-IRB-1600-6-1-45.robot');
    const assets = new AssetStore();
    const { station, report } = await importRdkBestEffort(data, 'ABB-IRB-1600-6-1-45.robot', assets);
    expect(report.fidelity).toBe('full');
    const robot = station.itemsOfType<Robot>(ItemType.ROBOT)[0];
    expect(robot.name).toBe('ABB IRB 1600-6/1.45');
    expect(robot.dof).toBe(6);
    expect(robot.postProcessor).toBe('ABB_RAPID_IRC5');
    expect(robot.brand).toBe('ABB');
    const tcp = getPos(robot.solveFKFlange([0, 0, 0, 0, 0, 0]));
    expect(tcp[0]).toBeCloseTo(815, 0);
    expect(tcp[1]).toBeCloseTo(0, 0);
    expect(tcp[2]).toBeCloseTo(1186.5, 0);
    const limits = robot.jointLimits();
    expect(limits.lower.slice(0, 6)).toEqual([-180, -90, -245, -200, -115, -400]);
    expect(limits.upper.slice(0, 6)).toEqual([180, 120, 65, 200, 115, 400]);
    expect(robot.chain.links.some((l) => l.visuals.length > 0)).toBe(true);
    expect(assets.get(robot.chain.links.find((l) => l.visuals.length)?.visuals[0].mesh ?? '')?.mesh?.triangles).toBeGreaterThan(10);
  });

  it.skipIf(!has('Example-03.c-Painting curve - KUKA.rdk'))('opens a painting station with the robot, tool and program', async () => {
    const data = load('Example-03.c-Painting curve - KUKA.rdk');
    const assets = new AssetStore();
    const { station, report } = await importRdkBestEffort(data, 'painting.rdk', assets);
    // honest report: the curve-follow project keeps only its links and input file
    expect(report.fidelity).toBe('partial');
    expect(report.dropped).toEqual([expect.stringMatching(/^machining project Path1 settings: /)]);
    expect(station.name).toMatch(/painting/i);
    const robot = station.itemsOfType<Robot>(ItemType.ROBOT)[0];
    expect(robot).toBeTruthy();
    expect(robot.dof).toBe(6);
    expect(robot.name).toMatch(/KUKA/i);
    const tool = station.itemsOfType<Tool>(ItemType.TOOL).find((t) => /paint gun/i.test(t.name));
    expect(tool).toBeTruthy();
    expect(tool!.geometry.length).toBeGreaterThan(0);
    const program = station.find('Path1', ItemType.PROGRAM) as Program;
    expect(program).toBeTruthy();
    expect(program.robot()?.name).toBe(robot.name);
    const moves = program.instructions().filter((i) => i.data.kind === 'move');
    expect(moves.length).toBeGreaterThan(10);
    const first = moves[0].data as { moveType: string; targetId: string | null; joints?: number[] };
    expect(first.moveType).toBe('MoveJ');
    // curve-follow programs store inline moves (joints + pose in the instruction, no target item)
    expect(first.targetId).toBeNull();
    expect(first.joints?.[0]).toBeCloseTo(-35.08, 1);
    const speed = program.instructions().find((i) => i.data.kind === 'speed');
    expect(speed?.data).toMatchObject({ speedLinear: 1000 });
  });

  it.skipIf(!has('Denso-Cobotta.robot', 'Doosan-M1013.robot', 'ABB-IRB-120-3-0-6.robot'))('opens several other library robots without losing the kinematic table', async () => {
    const names = ['Denso-Cobotta.robot', 'Doosan-M1013.robot', 'ABB-IRB-120-3-0-6.robot'];
    for (const name of names) {
      const data = load(name);
      const { station, report } = await importRdkBestEffort(data, name, new AssetStore());
      const robot = station.itemsOfType<Robot>(ItemType.ROBOT)[0];
      expect(report.fidelity, name).toBe('full');
      expect(robot.dof, name).toBeGreaterThanOrEqual(4);
      expect(robot.name.length, name).toBeGreaterThan(2);
      const tcp = getPos(robot.solveFKFlange(robot.jointsHome()));
      expect(tcp.every((v) => Number.isFinite(v)), name).toBe(true);
    }
  });

  it.skipIf(!has('Example-01.a-Robot Painting - Universal Robots.rdk'))('keeps the UR painting station inside the view', async () => {
    const data = load('Example-01.a-Robot Painting - Universal Robots.rdk');
    const { station } = await importRdkBestEffort(data, 'ur-paint.rdk', new AssetStore());
    const positions = [...station.walk()].map((item) => getPos(item.poseAbs()));
    expect(positions.every((p) => p.every((v) => Number.isFinite(v) && Math.abs(v) < 1e6))).toBe(true);
    const spot = station.find('Top Paint 1', ItemType.TARGET) as Target;
    expect(spot).toBeTruthy();
    const local = getPos(spot.pose());
    expect(local[0]).toBeCloseTo(31.1, 0);
    expect(local[1]).toBeCloseTo(24.8, 0);
    expect(local[2]).toBeCloseTo(35.4, 0);
    const robot = station.find('UR10', ItemType.ROBOT) as Robot;
    expect(robot.joints().map((v) => Math.round(v * 100) / 100)).toEqual([-151.99, -78.22, -86.64, -64.6, 73.06, 110.03]);
    const gun = station.find('Paint gun', ItemType.TOOL) as Tool;
    expect(getPos(gun.pose()).map((v) => Math.round(v))).toEqual([50, 0, 450]);
    expect(station.find('MainProg', ItemType.PROGRAM)).toBeTruthy();
  });

  it.skipIf(!has('Tutorial-UR-Painting.rdk'))('loads Tutorial-UR-Painting with link meshes and program calls', async () => {
    const data = load('Tutorial-UR-Painting.rdk');
    const assets = new AssetStore();
    const { station } = await importRdkBestEffort(data, 'Tutorial-UR-Painting.rdk', assets);
    const robot = station.find('UR10', ItemType.ROBOT) as Robot;
    const triangles = robot.chain.links.reduce((n, link) => n + link.visuals.reduce((s, v) => s + (assets.get(v.mesh ?? '')?.mesh?.triangles ?? 0), 0), 0);
    expect(triangles).toBeGreaterThan(20000);
    expect(robot.chain.links.filter((l) => l.visuals.length > 0).length).toBe(robot.chain.links.length);
    const main = station.find('MainProg', ItemType.PROGRAM) as Program;
    const calls = main.instructions().filter((i) => i.data.kind === 'call').map((i) => (i.data as { programName?: string }).programName);
    expect(calls).toEqual(['ApproachMove', 'PaintTop', 'Retract']);
    const { ProgramSimulator } = await import('../src/core/motion/simulator');
    const sim = new ProgramSimulator(station);
    sim.collisionOptions = { enabled: true, selfOnly: true, assets, sampleStep: 0.25 };
    const res = sim.compile(main);
    expect(res.problems.filter((p) => p.severity === 'error').map((p) => p.message)).toEqual([]);
    expect(res.executed).toBeGreaterThan(calls.length);
  }, 60000);

  it.skipIf(!has('Example-06.c-Pick and place with 2D camera.rdk'))('lays out the 2D-camera pick-and-place cell', async () => {
    const data = load('Example-06.c-Pick and place with 2D camera.rdk');
    const assets = new AssetStore();
    const { station } = await importRdkBestEffort(data, 'camera.rdk', assets);
    expect(getPos(station.find('Part 1')!.pose()).map((v) => Math.round(v))).toEqual([75, 100, 75]);
    expect(getPos(station.find('Part 12')!.pose()).map((v) => Math.round(v))).toEqual([225, 500, 225]);
    const belt = station.find('Conveyor Belt') as Robot;
    // RoboDK keeps the conveyor's base offset in the kinematics (SolveFK), not in the item pose
    expect(getPos(belt.pose()).map((v) => Math.round(v))).toEqual([0, 0, 0]);
    const mount = multiply(belt.pose(), belt.chain.base!);
    expect(getPos(mount).map((v) => Math.round(v))).toEqual([440, 200, 0]);
    expect(mount[9]).toBeCloseTo(1, 2);
    expect(mount[2]).toBeCloseTo(-1, 2);
    // every triangle of the belt mesh link (16 870, counted by an independent walker of the file), merged per colour
    expect(belt.chain.links[0].visuals.reduce((n, v) => n + (assets.get(v.mesh ?? '')?.mesh?.triangles ?? 0), 0)).toBe(16870);
    const move = station.find('MoveConveyor') as Program;
    expect(move.robot()?.name).toBe('Conveyor Belt');
    const main = station.find('MainSimulation') as Program;
    // "Run MoveConveyor" starts the conveyor on a parallel thread; calls to station Python programs stay code calls (I5)
    expect(main.instructions().map((ins) => [ins.data.kind, ins.name])).toEqual([['code', 'Call PrepareSimulation'], ['code', 'Call SimulateCamera'], ['thread', 'Run MoveConveyor'], ['thread', 'Run PartsToConveyor'], ['code', 'Call PartsToPallet']]);
    const run = main.instructions()[2].data as { programId: string };
    expect(run.programId).toBe(move.id);
    const pallet = station.find('Europallet') as SceneObject;
    // the stored geometry frame is inverted (RoboDK 5.9.4 collides probe shapes with the mesh at the inverse, round 2):
    // the 145 mm pallet hangs below its frame, whose plane is the top the parts are stacked on
    expect(pallet.geometry[0].origin![14]).toBeCloseTo(-145, 0);
    expect(pallet.geometry[0].origin![6]).toBeCloseTo(1, 3);
    expect(pallet.bbox!.max[2]).toBeLessThanOrEqual(0);
    expect(pallet.bbox!.min[2]).toBeCloseTo(-145, 0);
  }, 60000);

  it.skipIf(!has('Example-06.f-Pick and Place CNC and Dual RobotiQ Gripper-UR10.rdk'))('plays the dual-gripper CNC cycle without forearm collisions', async () => {
    const data = load('Example-06.f-Pick and Place CNC and Dual RobotiQ Gripper-UR10.rdk');
    const assets = new AssetStore();
    const { station } = await importRdkBestEffort(data, 'cnc.rdk', assets);
    expect((station.find('OpenDoor1') as Program).robot()?.name).toBe('Mazak Lathe');
    expect((station.find('CloseDoor1') as Program).robot()?.name).toBe('Mazak Lathe');
    expect((station.find('OpenGripper1') as Program).robot()?.name).toBe('RobotiQ HandE Gripper 1');
    expect((station.find('OpenGripper2') as Program).robot()?.name).toBe('RobotiQ HandE Gripper 2');
    const lathe = station.find('Mazak Lathe') as Robot;
    expect(lathe.chain.joints[0].type).toBe('prismatic');
    expect(lathe.chain.links[0].visuals.length).toBeGreaterThan(0);
    expect(lathe.chain.links[1].visuals.length).toBeGreaterThan(0);
    const at = (q: number, link: number) => { lathe.setJoints([q]); return getPos(multiply(lathe.poseAbs(), lathe.fk().linkPoses[link])); };
    const doorClosed = at(0, 1);
    const doorOpen = at(700, 1);
    const bodyClosed = at(0, 0);
    const bodyOpen = at(700, 0);
    expect(doorOpen[0] - doorClosed[0]).toBeCloseTo(700, 0);
    expect(bodyOpen[0]).toBeCloseTo(bodyClosed[0], 0);
    expect(station.find('Part Done 1')!.visible).toBe(false);
    expect(station.find('Part 1')!.visible).toBe(true);
    const work = station.find('Work 1') as Program;
    expect(work.instructions().some((i) => i.data.kind === 'event' && i.data.action === 'show' && i.data.name === 'Part Done 1')).toBe(true);
    const grip = station.find('Gripper1PickInput') as Program;
    expect(grip.instructions().some((i) => i.data.kind === 'event' && i.data.action === 'attach' && i.data.name === 'Gripper1')).toBe(true);
    const jaw = station.find('RobotiQ HandE Gripper 1') as Robot;
    expect(jaw.dof).toBe(1);
    expect(jaw.chain.links[1].visuals.length).toBeGreaterThan(0);
    expect(jaw.chain.links[2].visuals.length).toBeGreaterThan(0);
    jaw.setJoints([50]);
    // fingers travel ±q/2 along the z axis of the gripper's base-offset frame (chain.base); the flange stays put
    const inBase = (m: Float64Array) => multiply(invert(jaw.chain.base!), m);
    const fingers = jaw.fk().linkPoses.map(inBase);
    expect(fingers[1][14]).toBeCloseTo(25, 6);
    expect(fingers[2][14]).toBeCloseTo(-25, 6);
    expect(fingers[2][10]).toBeCloseTo(-1, 6);
    const fl0 = jaw.solveFKFlange([0]), fl50 = jaw.solveFKFlange([50]);
    expect(Array.from(fl50).map((v, i) => v - fl0[i]).every((v) => Math.abs(v) < 1e-9)).toBe(true);
    const { ProgramSimulator } = await import('../src/core/motion/simulator');
    const sim = new ProgramSimulator(station);
    sim.collisionOptions = { enabled: true, selfOnly: true, assets, sampleStep: 0.5 };
    const res = sim.compile(station.find('MainProgram') as Program);
    // no collision of the arm (the forearm passing the lathe door); contacts of the gripper fingers with the part they
    // hold are collisions under RoboDK's map (round 2: RoboDK checks a held part against its holder, 02.i)
    expect(res.problems.filter((p) => p.severity === 'error' && /UR10e\/link/.test(p.message)).map((p) => p.message)).toEqual([]);
    const throughDoor = sim.steps.some((s) => s.robot?.name === 'UR10e' && !!s.trajectory?.samples.at(-1)?.joints.every((v) => Math.abs(v) < 1));
    expect(throughDoor).toBe(false);
  }, 90000);

  it.skipIf(!has('Example-09.c-PRM Automated Collision Avoidance - Comau.rdk'))('assembles the Comau Smart5 at the RoboDK link frames', async () => {
    const data = load('Example-09.c-PRM Automated Collision Avoidance - Comau.rdk');
    const { station } = await importRdkBestEffort(data, 'comau.rdk', new AssetStore());
    const robot = station.find('Comau Smart5 NJ 165-3.4 SH') as Robot;
    expect(robot.dof).toBe(6);
    expect(robot.chain.joints[0].axis[2]).toBe(-1);
    expect(robot.chain.joints[2].couple?.joint).toBe('J2');
    const at = (i: number) => getPos(multiply(robot.poseAbs(), robot.fk().linkPoses[i])).map((v) => Math.round(v));
    expect(at(2)).toEqual([607, 595, 500]);
    expect(at(3)).toEqual([742, 727, 1660]);
    expect(at(4)).toEqual([1743, 1706, 1227]);
    expect(at(6)).toEqual([1973, 1709, 1227]);
    for (let i = 0; i < robot.chain.links.length; i++) expect(robot.chain.links[i].visuals.length).toBeGreaterThan(0);
  }, 60000);

  it.skipIf(!has('paint_gun.tool', 'box.sld'))('reads a library tool and a solid', async () => {
    const gun = load('paint_gun.tool');
    const box = load('box.sld');
    const assets = new AssetStore();
    const tool = await importRdkBestEffort(gun, 'paint_gun.tool', assets);
    expect(tool.report.fidelity).toBe('full');
    const item = tool.station.itemsOfType<Tool>(ItemType.TOOL)[0];
    expect(item.name).toMatch(/paint gun/i);
    expect(item.geometry.length).toBeGreaterThan(0);
    const solid = await importRdkBestEffort(box, 'box.sld', assets);
    expect(solid.report.fidelity).toBe('full');
    const meshId = solid.station.children[0] instanceof Robot ? '' : (solid.station.children[0] as { geometry?: Array<{ mesh?: string }> }).geometry?.[0]?.mesh;
    const mesh = meshId ? assets.get(meshId)?.mesh : undefined;
    expect(mesh?.triangles).toBe(12);
    expect(mesh?.min.map((v) => Math.round(v))).toEqual([-50, -50, -50]);
    expect(mesh?.max.map((v) => Math.round(v))).toEqual([50, 50, 50]);
  });
});
