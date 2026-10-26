// T3 (sim): collision checking along program timelines (RoboDK parity): tools without geometry have no collider,
// the station collision map is honoured, other robots that move during the program are re-evaluated at every sample,
// imported mesh links are tested triangle-exactly (no box false positives, real self-collisions found).
import { describe, it, expect } from 'vitest';
import { Station, Target, Tool, SceneObject } from '../src/core/items/item';
import { Program } from '../src/core/items/program';
import { createRobotFromLibrary } from '../src/core/items/library';
import { ProgramSimulator } from '../src/core/motion/simulator';
import { robotCollisionChecker, checkRobotCollisions, setCollisionPair } from '../src/core/collision/collision';
import { AssetStore } from '../src/scene/assets';
import { buildMechanism, MAKE_ROBOT_1T } from '../src/core/kinematics/mechanism';
import { StationRuntime } from '../src/core/motion/runtime';
import { multiply, transl } from '../src/core/math/pose';

/** Closed box mesh (12 triangles) centred at the origin. */
function boxMesh(sx: number, sy: number, sz: number) {
  const x = sx / 2, y = sy / 2, z = sz / 2;
  const v = [[-x, -y, -z], [x, -y, -z], [x, y, -z], [-x, y, -z], [-x, -y, z], [x, -y, z], [x, y, z], [-x, y, z]];
  const f = [[0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7], [0, 1, 5], [0, 5, 4], [1, 2, 6], [1, 6, 5], [2, 3, 7], [2, 7, 6], [3, 0, 4], [3, 4, 7]];
  const positions = new Float32Array(f.flatMap((t) => t.flatMap((i) => v[i])));
  return { positions, normals: new Float32Array(positions.length), min: [-x, -y, -z] as [number, number, number], max: [x, y, z] as [number, number, number], triangles: 12 };
}

describe('collision along timelines', () => {
  it('a tool without geometry has no collider (welding torch at the part is not a collision)', () => {
    const st = new Station();
    const r = st.addChild(createRobotFromLibrary('UR10e'));
    const torch = r.addChild(new Tool('Torch')); torch.setPoseTool(transl(0, 0, 200)); r.setTool(torch);
    const part = st.addChild(new SceneObject('Part'));
    part.geometry = [{ primitive: { kind: 'box', size: [100, 100, 20] } }];
    part.setPose(multiply(r.poseAbs(), r.solveFK(r.joints(), transl(0, 0, 200)))); // TCP exactly on the part
    expect(checkRobotCollisions(st, r).filter((p) => p.a.item === part || p.b.item === part)).toEqual([]);
    // with geometry the torch body does collide
    torch.geometry = [{ primitive: { kind: 'cylinder', radius: 15, length: 200 }, origin: Array.from(transl(0, 0, 100)) }];
    expect(checkRobotCollisions(st, r).some((p) => p.a.item === part || p.b.item === part)).toBe(true);
  });

  it('the robot checker re-evaluates another robot that moves (no stale cache within one version)', () => {
    const st = new Station();
    const a = st.addChild(createRobotFromLibrary('UR10e', 'A'));
    const b = st.addChild(createRobotFromLibrary('UR10e', 'B'));
    b.setPose(transl(1400, 0, 0));
    a.setJoints([180, -90, 0, -90, 0, 0]); b.setJoints([0, -90, 0, -90, 0, 0]); // both arms up: apart
    const chk = robotCollisionChecker(st, a, {});
    expect(chk.check(1)).toEqual([]);
    // UR arms stretch along -X at q1 = 0: A (q1 = 180) reaches +X, B (q1 = 0) reaches back towards A
    b.setJoints([0, 0, 0, -90, 0, 0]);
    a.setJoints([180, 0, 0, -90, 0, 0]);
    const pairs = chk.check(1);
    expect(pairs.some((p) => (p.a.item === a && p.b.item === b) || (p.a.item === b && p.b.item === a))).toBe(true);
  });

  it('the station collision map disables a pair along the program timeline', () => {
    const st = new Station();
    const r = st.addChild(createRobotFromLibrary('UR10e'));
    r.setJoints([0, 0, 0, -90, 0, 0]); // arm horizontal along -X
    const post = st.addChild(new SceneObject('Post'));
    post.geometry = [{ primitive: { kind: 'box', size: [100, 100, 1000] } }];
    const t = st.addChild(new Target('Swing')); t.setJoints([180, 0, 0, -90, 0, 0]); t.setAsJointTarget();
    post.setPose(transl(0, -800, 500)); // the swing passes q1 = 90 deg, where the arm lies along -Y
    const p = st.addChild(new Program('P')); p.setRobot(r); p.addMoveJ(t);
    const sim = new ProgramSimulator(st);
    sim.collisionOptions = { enabled: true, sampleStep: 0.02 };
    const res = sim.compile(p);
    expect(res.problems.some((x) => x.severity === 'error' && /Collision: .*Post|Post.*×/.test(x.message))).toBe(true);
    expect(sim.errorTime!).toBeGreaterThan(0);
    expect(sim.errorTime!).toBeLessThan(res.duration);
    setCollisionPair(st, r, post, -1, -1, false);
    const res2 = new ProgramSimulator(st);
    res2.collisionOptions = { enabled: true, sampleStep: 0.02 };
    expect(res2.compile(p).ok).toBe(true);
  });

  it('an imported RoboDK collision map (entities + checked pairs) decides which pairs are checked', () => {
    const st = new Station();
    const r = st.addChild(createRobotFromLibrary('UR10e'));
    r.setJoints([0, -90, 0, -90, 0, 0]);
    const elbow = r.fk().linkPoses[3];
    const box = st.addChild(new SceneObject('Box')); box.geometry = [{ primitive: { kind: 'box', size: [300, 300, 300] } }];
    box.setPose(transl(elbow[12], elbow[13], elbow[14]));
    const other = st.addChild(new SceneObject('Other')); other.geometry = [{ primitive: { kind: 'box', size: [300, 300, 300] } }];
    other.setPose(transl(elbow[12], elbow[13], elbow[14] + 150));
    const entities = [...r.chain.links.map((_, k) => ({ id: r.id, name: r.name, link: k })), { id: box.id, name: 'Box' }, { id: other.id, name: 'Other' }];
    const iBox = entities.length - 2;
    // RoboDK checks only link 3 x Box (Other is not checked against anything)
    st.params.collisionMap = { entities, pairs: [[3, iBox]] } as any;
    const pairs = robotCollisionChecker(st, r, {}).check(1);
    expect(pairs.length).toBeGreaterThan(0);
    expect(pairs.every((p) => [p.a.item, p.b.item].includes(box) && [p.a.linkIndex, p.b.linkIndex].includes(3))).toBe(true);
    // a user override (setCollisionPair) wins over the imported map
    setCollisionPair(st, r, box, 3, -1, false);
    expect(robotCollisionChecker(st, r, {}).check(2)).toEqual([]);
  });

  it('SRDF disabled collision pairs (params.disabledCollisions on the imported root) are not checked', () => {
    const st = new Station();
    const r = st.addChild(createRobotFromLibrary('UR10e'));
    r.setJoints([0, -112, 176, -154, -90, 35]); // folded: the wrist touches the upper arm
    const self = () => checkRobotCollisions(st, r, { selfOnly: true }).filter((p) => p.a.item === r && p.b.item === r);
    const pairs = self();
    expect(pairs.length).toBeGreaterThan(0);
    r.params.disabledCollisions = pairs.map((p) => [p.a.part, p.b.part, 'Never']) as any;
    expect(self()).toEqual([]);
  });

  it('a part held by a tool is not a collision with the gripper fingers holding it, but still collides with other objects', () => {
    const st = new Station();
    const arm = st.addChild(createRobotFromLibrary('UR10e', 'Arm'));
    arm.setJoints([0, -90, 90, -90, -90, 0]);
    const fingers = arm.addChild(buildMechanism({ type: MAKE_ROBOT_1T, parameters: [50], name: 'Gripper' }));
    fingers.chain.links[1].visuals = [{ primitive: { kind: 'box', size: [30, 30, 120] }, origin: Array.from(transl(0, 0, 60)) } as any];
    const tool = arm.addChild(new Tool('Gripper TCP')); tool.setPoseTool(transl(0, 0, 100)); arm.setTool(tool);
    const part = st.addChild(new SceneObject('Part')); part.geometry = [{ primitive: { kind: 'box', size: [60, 60, 60] } }];
    part.setPoseAbs(tool.poseAbs());
    const rt = new StationRuntime(st); rt.attachObject(part, tool);
    const pairs = () => checkRobotCollisions(st, arm).filter((p) => p.a.item === part || p.b.item === part);
    expect(pairs()).toEqual([]);
    const wall = st.addChild(new SceneObject('Wall')); wall.geometry = [{ primitive: { kind: 'box', size: [40, 400, 400] } }];
    wall.setPoseAbs(tool.poseAbs());
    expect(pairs().some((p) => p.a.item === wall || p.b.item === wall)).toBe(true);
  });

  it('mesh links are tested triangle-exactly: a thin shell next to a link is no collision, a part through it is', () => {
    const st = new Station();
    const assets = new AssetStore();
    assets.registerMesh('plate.stl', boxMesh(400, 400, 4));
    const r = st.addChild(createRobotFromLibrary('UR10e'));
    r.setJoints([0, -90, 0, -90, 0, 0]);
    const elbow = r.fk().linkPoses[3];
    const plate = st.addChild(new SceneObject('Plate'));
    plate.geometry = [{ mesh: 'plate.stl' }];
    // the plate's bounding box (400 x 400 x 4) is 30 mm beside the forearm capsule
    plate.setPose(transl(elbow[12], elbow[13] + 260, elbow[14]));
    const chk = (o: SceneObject) => robotCollisionChecker(st, r, { assets }).check(Math.random()).filter((p) => p.a.item === o || p.b.item === o);
    expect(chk(plate)).toEqual([]);
    plate.setPose(transl(elbow[12], elbow[13], elbow[14]));
    expect(chk(plate).length).toBeGreaterThan(0);
  });
});
