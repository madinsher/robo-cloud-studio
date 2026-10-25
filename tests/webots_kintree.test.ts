/**
 * KinTree (I3) decomposition: drive detection (differential, Ackermann, caster, floating base), maximal serial chains
 * with nested mounting, mimic joints, tip links, sensors. The physical invariant checked everywhere: for random joint
 * values, every link pose of the decomposed items (MobileRobot / nested Robots) equals the forward kinematics of the
 * original tree computed independently here.
 */
import { describe, it, expect } from 'vitest';
import { buildStationItemsFromTree, KinTree, KinJoint, KinLink } from '../src/io/kintree';
import { Robot } from '../src/core/items/robot';
import { MobileRobot } from '../src/mobile/items';
import { Camera, SceneObject, Item } from '../src/core/items/item';
import { Mat4, identity, multiply, transl, rotAxis, rotx, DEG, RAD, transformDir } from '../src/core/math/pose';

const link = (name: string, extra: Partial<KinLink> = {}): KinLink => ({ name, visuals: [{ primitive: { kind: 'box', size: [10, 10, 10] } }], ...extra });
const rev = (name: string, parent: string, child: string, origin: Mat4, axis: [number, number, number], lower = -90, upper = 90, extra: Partial<KinJoint> = {}): KinJoint => ({ name, type: 'revolute', parent, child, origin, axis, lower, upper, ...extra });
const cont = (name: string, parent: string, child: string, origin: Mat4, axis: [number, number, number]): KinJoint => ({ name, type: 'continuous', parent, child, origin, axis, velocity: 600 });
const fixed = (name: string, parent: string, child: string, origin: Mat4): KinJoint => ({ name, type: 'fixed', parent, child, origin, axis: [0, 0, 1] });
const wheelLink = (name: string, r = 100) => link(name, { visuals: [], collisions: [{ primitive: { kind: 'cylinder', radius: r, length: 40 }, origin: Array.from(rotx(Math.PI / 2)) }] });

/** Reference FK of a tree: link poses relative to the root for joint values (mimics resolved from their master). */
function treeFK(tree: KinTree, q: Record<string, number>): Map<string, Mat4> {
  const out = new Map<string, Mat4>([[tree.root, identity()]]);
  const val = (j: KinJoint) => (j.mimic ? j.mimic.multiplier * (q[j.mimic.joint] ?? 0) + j.mimic.offset : q[j.name] ?? 0);
  const rec = (l: string) => {
    for (const j of tree.joints.filter((x) => x.parent === l)) {
      const v = val(j);
      const m = j.type === 'prismatic' ? transl(j.axis[0] * v, j.axis[1] * v, j.axis[2] * v) : j.type === 'fixed' ? identity() : rotAxis(j.axis, v * DEG);
      out.set(j.child, multiply(multiply(out.get(l)!, j.origin), m));
      rec(j.child);
    }
  };
  rec(tree.root);
  return out;
}

/** Link poses of the decomposed robots (absolute, items not in a station: root at identity). */
function itemLinkPoses(robots: Robot[]): Map<string, Mat4> {
  const out = new Map<string, Mat4>();
  for (const r of robots) {
    const fk = r.fk(r.joints());
    r.chain.links.forEach((l, i) => { if (i > 0 && !l.name.endsWith('__return')) out.set(l.name, multiply(r.poseAbs(), fk.linkPoses[i])); }); // __return: the way back from a side branch of a mechanism
  }
  return out;
}

function expectSamePose(a: Mat4, b: Mat4, tol = 1e-6) {
  for (let i = 0; i < 16; i++) expect(Math.abs(a[i] - b[i])).toBeLessThan(tol);
}

/** Set random joint values on the robots and compare with the tree FK. */
function checkFK(tree: KinTree, robots: Robot[], seed = 1) {
  let s = seed;
  const rnd = () => { s = (s * 16807) % 2147483647; return s / 2147483647; };
  const q: Record<string, number> = {};
  for (const r of robots) {
    const { lower, upper } = r.jointLimits();
    const v = lower.map((lo, k) => lo + (Math.min(upper[k], lo + 720) - lo) * rnd());
    r.setJoints(v);
    r.jointNames().forEach((n, k) => (q[n] = v[k]));
  }
  const ref = treeFK(tree, q);
  const got = itemLinkPoses(robots);
  expect(got.size).toBeGreaterThan(0);
  for (const [name, pose] of got) expectSamePose(pose, ref.get(name)!);
}

function humanoid(): KinTree {
  const links = ['base', 'lw', 'rw', 'caster', 'torso', 'chest', 'l1', 'l2', 'l3', 'r1', 'r2', 'r3', 'neck', 'head'].map((n) => (n === 'lw' || n === 'rw' ? wheelLink(n) : link(n)));
  links.find((l) => l.name === 'head')!.sensors = [{ kind: 'camera', name: 'eye', pose: transl(50, 0, 0), fov: 60, fovAxis: 'horizontal', width: 320, height: 240 }];
  links.find((l) => l.name === 'base')!.sensors = [{ kind: 'lidar2d', name: 'scan', pose: transl(200, 0, 300), fov: 240, width: 640, near: 150, far: 8000 }, { kind: 'imu', name: 'imu', pose: identity() }];
  return {
    name: 'hum', root: 'base', links,
    joints: [
      cont('lw_j', 'base', 'lw', transl(0, 250, 100), [0, 1, 0]),
      cont('rw_j', 'base', 'rw', transl(0, -250, 100), [0, 1, 0]),
      fixed('caster_j', 'base', 'caster', transl(-300, 0, 30)),
      fixed('torso_j', 'base', 'torso', transl(0, 0, 500)),
      rev('torso_yaw', 'torso', 'chest', transl(0, 0, 300), [0, 0, 1], -120, 120),
      rev('l_sh', 'chest', 'l1', transl(0, 200, 400), [0, 1, 0]), rev('l_el', 'l1', 'l2', transl(0, 0, -300), [0, 1, 0], 0, 150), rev('l_wr', 'l2', 'l3', transl(0, 0, -250), [1, 0, 0]),
      rev('r_sh', 'chest', 'r1', transl(0, -200, 400), [0, 1, 0]), rev('r_el', 'r1', 'r2', transl(0, 0, -300), [0, 1, 0], 0, 150), rev('r_wr', 'r2', 'r3', transl(0, 0, -250), [1, 0, 0]),
      rev('head_pan', 'chest', 'neck', transl(0, 0, 500), [0, 0, 1]), rev('head_tilt', 'neck', 'head', transl(0, 0, 80), [0, 1, 0], -45, 30),
    ],
  };
}

describe('buildStationItemsFromTree', () => {
  it('humanoid on wheels: differential base, torso chain with two arms and a head mounted on its flange', () => {
    const tree = humanoid();
    const res = buildStationItemsFromTree(tree);
    const mob = res.mobile!;
    expect(mob).toBeInstanceOf(MobileRobot);
    expect(res.root).toBe(mob);
    expect(mob.kin.drive).toBe('differential');
    expect(mob.kin.track).toBeCloseTo(500, 9);
    expect(mob.kin.wheelRadius).toBeCloseTo(100, 9);
    expect(mob.kin.maxSpeed).toBeCloseTo(600 * DEG * 100, 9);
    expect(mob.kin.maxYawRate).toBeCloseTo((2 * mob.kin.maxSpeed / 500) * RAD, 9);
    expect(res.robots.map((r) => `${r.name}:${r.dof}`)).toEqual(['hum_torso_yaw:1', 'hum_left_arm:3', 'hum_right_arm:3', 'hum_head:2']);
    const torso = res.robots[0];
    expect(torso.parent).toBe(mob);
    for (const r of res.robots.slice(1)) expect(r.parent).toBe(torso);
    // base geometry: base + caster + torso + both wheels (folded at q = 0)
    expect(mob.geometry.length).toBe(3);
    expect(((mob as unknown) as { colliders: unknown[] }).colliders.length).toBe(2);
    checkFK(tree, res.robots, 7);
    checkFK(tree, res.robots, 99);
    // camera on the head flange follows the head joints; lidar + imu go to the mobile sensors
    const eye = res.cameras.find((c) => c.name === 'eye')!;
    expect(eye.parent).toBe(res.robots[3]);
    expect(eye.fovAxis).toBe('horizontal');
    const head = res.robots[3];
    head.setJoints([30, -20]);
    const ref = treeFK(tree, { torso_yaw: torso.joints()[0], head_pan: 30, head_tilt: -20 });
    expectSamePose(eye.poseAbs(), multiply(ref.get('head')!, transl(50, 0, 0)));
    expect(mob.sensors.map((s) => s.kind)).toEqual(['lidar2d', 'imu']);
    expect(mob.sensors[0].range).toBe(8000);
    expect(res.cameras.find((c) => c.name === 'scan')!.parent).toBe(mob);
  });

  it('Ackermann: steered front wheels give wheel base, track, max steer and the minimum turning radius', () => {
    const links = ['base', 'fl_s', 'fr_s', 'fl', 'fr', 'rl', 'rr'].map((n) => (n.length === 2 ? wheelLink(n, 300) : link(n)));
    const tree: KinTree = {
      name: 'car', root: 'base', links,
      joints: [
        rev('fl_steer', 'base', 'fl_s', transl(1100, 900, 300), [0, 0, 1], -20, 20), cont('fl_w', 'fl_s', 'fl', identity(), [0, 1, 0]),
        rev('fr_steer', 'base', 'fr_s', transl(1100, -900, 300), [0, 0, 1], -25, 25), cont('fr_w', 'fr_s', 'fr', identity(), [0, 1, 0]),
        cont('rl_w', 'base', 'rl', transl(-1100, 910, 300), [0, 1, 0]), cont('rr_w', 'base', 'rr', transl(-1100, -910, 300), [0, 1, 0]),
      ],
    };
    const res = buildStationItemsFromTree(tree);
    const k = res.mobile!.kin;
    expect(k.drive).toBe('ackermann');
    expect(k.wheelBase).toBeCloseTo(2200, 9);
    expect(k.track).toBeCloseTo(1820, 9);
    expect(k.maxSteer).toBe(20);
    expect(k.minTurnRadius).toBeCloseTo(2200 / Math.tan(20 * DEG), 6);
    expect(k.wheelRadius).toBe(300);
    expect(res.robots).toHaveLength(0);
    expect(res.mobile!.params.steerJoints).toEqual(['fl_steer', 'fr_steer']);
  });

  it('a raised continuous joint (head tilt) and a free swivel are not drive wheels', () => {
    const tree = humanoid();
    tree.joints.find((j) => j.name === 'head_tilt')!.type = 'continuous';
    tree.links.push(link('sw'), wheelLink('cw', 40));
    tree.joints.push(cont('swivel', 'base', 'sw', transl(-300, 0, 40), [0, 0, 1]), cont('cw_j', 'sw', 'cw', transl(-30, 0, 0), [0, 1, 0]));
    tree.joints.find((j) => j.name === 'swivel')!.velocity = undefined;
    const res = buildStationItemsFromTree(tree);
    expect(res.mobile!.params.wheelJoints).toEqual(['lw_j', 'rw_j']);
    expect(res.report.some((l) => l.includes('cw_j under the free swivel swivel is a caster'))).toBe(true);
    expect(res.robots.find((r) => r.name === 'hum_head')!.jointNames()).toEqual(['head_pan', 'head_tilt']);
  });

  it('mimic joints: serial mimics stay in the chain, side-branch mimics are drawn static and reported', () => {
    const tree: KinTree = {
      name: 'grip', root: 'world', links: ['world', 'base', 'a1', 'a2', 'a3', 'fl', 'fr', 'tool0'].map((n) => link(n)),
      joints: [
        fixed('mount', 'world', 'base', transl(0, 0, 100)),
        rev('j1', 'base', 'a1', transl(0, 0, 200), [0, 0, 1]),
        rev('j2', 'a1', 'a2', transl(0, 0, 300), [0, 1, 0]),
        rev('j3', 'a2', 'a3', transl(0, 0, 300), [0, 1, 0], -90, 90, { mimic: { joint: 'j2', multiplier: -1, offset: 0 } }),
        fixed('tool', 'a3', 'tool0', transl(0, 0, 50)),
        { name: 'finger_l', type: 'prismatic', parent: 'tool0', child: 'fl', origin: transl(0, 20, 0), axis: [0, 1, 0], lower: 0, upper: 40 },
        { name: 'finger_r', type: 'prismatic', parent: 'tool0', child: 'fr', origin: transl(0, -20, 0), axis: [0, -1, 0], lower: 0, upper: 40, mimic: { joint: 'finger_l', multiplier: 1, offset: 0 } },
      ],
    };
    const res = buildStationItemsFromTree(tree);
    expect(res.robots).toHaveLength(1);
    const r = res.robots[0];
    expect(res.root).toBe(r); // single chain on a fixed base: the robot is the root, base visuals in links[0]
    expect(r.chain.links[0].visuals.length).toBe(2); // world + base links
    expect(r.jointNames()).toEqual(['j1', 'j2', 'finger_l']);
    expect(r.chain.joints.map((j) => j.name)).toEqual(['j1', 'j2', 'j3', 'finger_l']);
    expect(r.chain.joints[2].mimic).toEqual({ joint: 'j2', multiplier: -1, offset: 0 });
    expect(r.pose()[14]).toBe(100); // robot base = parent link of j1
    expect(res.report.some((l) => l.includes('mimic side branch finger_r'))).toBe(true);
    // the static finger stays visible on the last link
    expect(r.chain.links[3].visuals.length).toBe(3); // tool0... folded: fl link + tool0 + fr (static)
    checkFK(tree, [r], 3);
  });

  it('tip links cut a chain; the following joints become a robot mounted on the flange', () => {
    const tree: KinTree = {
      name: 'arm', root: 'base', links: ['base', 'l1', 'l2', 'tool0', 'g1'].map((n) => link(n)),
      joints: [rev('j1', 'base', 'l1', transl(0, 0, 100), [0, 0, 1]), rev('j2', 'l1', 'l2', transl(0, 0, 400), [0, 1, 0]), fixed('t', 'l2', 'tool0', transl(0, 0, 120)), rev('g', 'tool0', 'g1', transl(10, 0, 30), [1, 0, 0])],
    };
    const res = buildStationItemsFromTree(tree, { tipLinks: ['tool0'] });
    expect(res.robots.map((r) => r.dof)).toEqual([2, 1]);
    const [arm, grip] = res.robots;
    expect(arm.chain.flange[14]).toBe(120);
    expect(grip.parent).toBe(arm);
    // the gripper robot's base is tool0 (= the arm flange); its first joint origin carries the offset
    expect(Array.from(grip.pose())).toEqual(Array.from(identity()));
    expect(Array.from(grip.chain.joints[0].origin).slice(12, 15)).toEqual([10, 0, 30]);
    checkFK(tree, res.robots, 5);
  });

  it('floating base without wheels gives an omni MobileRobot; drive "none" keeps wheels as joints; "kinematic" forces a platform', () => {
    const tree: KinTree = {
      name: 'drone', root: 'world', links: [{ name: 'world', visuals: [] }, link('body'), link('g1')],
      joints: [{ name: 'free', type: 'floating', parent: 'world', child: 'body', origin: transl(0, 0, 1000), axis: [0, 0, 1] }, rev('gimbal', 'body', 'g1', transl(0, 0, -50), [0, 1, 0])],
    };
    const res = buildStationItemsFromTree(tree);
    expect(res.mobile!.kin.drive).toBe('omni');
    expect(res.mobile!.pose()[14]).toBe(1000);
    expect(res.robots[0].parent).toBe(res.mobile);
    const none = buildStationItemsFromTree(humanoid(), { drive: 'none' });
    expect(none.mobile).toBeUndefined();
    expect(none.root).toBeInstanceOf(SceneObject);
    expect(none.robots.map((r) => r.name)).toContain('hum_lw_j');
    const box: KinTree = { name: 'cart', root: 'b', links: [link('b')], joints: [] };
    const kin = buildStationItemsFromTree(box, { drive: 'kinematic' });
    expect(kin.mobile!.kin.drive).toBe('omni');
    const forced = buildStationItemsFromTree(box, { drive: { drive: 'ackermann', wheelBase: 2000, maxSteer: 15 } });
    expect(forced.mobile!.kin.drive).toBe('ackermann');
    expect(forced.mobile!.kin.wheelBase).toBe(2000);
  });

  it('a mimic of a joint in another branch makes one mechanism (the mimic follows its master); sensors on intermediate links are reported', () => {
    const tree: KinTree = {
      name: 't', root: 'b', links: ['b', 'c', 'x1', 'x2', 'z1', 'y1'].map((n) => link(n)),
      joints: [rev('yaw', 'b', 'c', transl(0, 0, 100), [0, 0, 1]), rev('x1', 'c', 'x1', transl(0, 100, 0), [0, 1, 0]), rev('x2', 'x1', 'x2', transl(0, 0, 100), [0, 1, 0]), rev('z1', 'c', 'z1', transl(0, -100, 0), [0, 1, 0]), rev('y1', 'z1', 'y1', transl(0, 0, 100), [0, 1, 0], -90, 90, { mimic: { joint: 'x1', multiplier: 1, offset: 0 } })],
    };
    tree.links.find((l) => l.name === 'x1')!.sensors = [{ kind: 'camera', name: 'mid', pose: identity() }];
    const res = buildStationItemsFromTree(tree);
    expect(res.report.some((l) => l.includes('imported as independent'))).toBe(false);
    expect(res.report.some((l) => /with cross-chain mimics imported as one chain of 4 DOF/.test(l))).toBe(true);
    expect(res.robots).toHaveLength(1); expect(res.robots[0].jointNames().sort()).toEqual(['x1', 'x2', 'yaw', 'z1']);
    expect(res.report.some((l) => l.includes('sensor mid sits on intermediate link x1'))).toBe(true);
    const cam = res.cameras[0] as Camera;
    const x = res.robots.find((r) => r.jointNames().includes('x2'))!;
    expect(cam.parent).toBe(x);
    // at the import configuration the camera is where the tree puts it
    const ref = treeFK(tree, Object.fromEntries(res.robots.flatMap((r) => r.jointNames().map((n, k) => [n, r.joints()[k]]))));
    expectSamePose(cam.poseAbs(), ref.get('x1')!);
    expect(transformDir(cam.poseAbs(), [0, 0, 1])).toBeTruthy();
    void (cam as Item);
    checkFK(tree, res.robots, 7); // every link of the mechanism at random joint values, y1 following x1 on its own branch
  });
});
