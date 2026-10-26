/**
 * T5 acceptance: URDF kinematics vs ROS. tests/fixtures/ros_gt/fk/<case>.json holds the urdf_parser_py tree FK of every
 * link (q = 0 and q = 0.3 rad / 0.05 m on every movable joint, mimic honoured) for the ROS-expanded URDFs in
 * tests/fixtures/ros_gt/xacro. The studio's URDF model must reproduce every link pose (< 0.1 mm, rotation < 1e-6),
 * the station items built through the kinematic tree must reproduce every chain link, and no movable joint may be lost.
 */
import { describe, it, expect } from 'vitest';
import { readdirSync, existsSync } from 'node:fs';
import { parseURDF, urdfLinkPoses, importURDF, chainFromURDF, URDFError, URDFModel } from '../src/io/urdf/urdf';
import { Robot } from '../src/core/items/robot';
import { MobileRobot } from '../src/mobile/items';
import { multiply, Mat4, RAD } from '../src/core/math/pose';
import { FIX, read, join } from './ros_gt_util';

const fkFiles = readdirSync(join(FIX, 'fk')).filter((f) => f.endsWith('.json')).sort();
type GT = { zero: Record<string, number[][]>; q: Record<string, number[][]> };
const gtOf = (f: string): GT => Object.values(JSON.parse(read(join(FIX, 'fk', f))))[0] as GT;
const qConfig = (m: URDFModel) => Object.fromEntries(m.joints.filter((j) => ['revolute', 'continuous', 'prismatic'].includes(j.type)).map((j) => [j.name, j.type === 'prismatic' ? 50 : 0.3 * RAD]));
/** Values of every joint (mimic joints from their master) — robots that import a cross-chain mimic as independent get its value. */
const qFull = (m: URDFModel, q: Record<string, number>) => Object.fromEntries(m.joints.map((j) => [j.name, j.mimic ? j.mimic.multiplier * (q[j.mimic.joint] ?? 0) + j.mimic.offset : q[j.name] ?? 0]));

function poseErr(T: Mat4, G: number[][]): { pos: number; rot: number } {
  const pos = Math.hypot(T[12] - G[0][3] * 1000, T[13] - G[1][3] * 1000, T[14] - G[2][3] * 1000);
  let rot = 0;
  for (let r = 0; r < 3; r++) for (let c = 0; c < 3; c++) rot = Math.max(rot, Math.abs(T[c * 4 + r] - G[r][c]));
  return { pos, rot };
}

describe('URDF tree FK == urdf_parser_py for every link', () => {
  it('has FK ground truth for the MURKA, realsense, UR, TurtleBot and vertical_bot robots', () => {
    expect(fkFiles.length).toBeGreaterThanOrEqual(25);
  });
  for (const f of fkFiles) {
    const name = f.replace(/\.json$/, '');
    const checkErr = join(FIX, 'fk', `${name}.check_err`);
    it(`${name}`, () => {
      const text = read(join(FIX, 'xacro', `${name}.urdf`));
      if (existsSync(checkErr)) {
        // check_urdf (urdfdom) rejects this model -> so does the studio, with the same reason
        const why = /Error:\s+(.*?)\.?\s*$/m.exec(read(checkErr))![1];
        expect(() => parseURDF(text)).toThrow(URDFError);
        expect(() => parseURDF(text)).toThrow(why);
        return;
      }
      const m = parseURDF(text);
      const gt = gtOf(f);
      expect(m.links.size).toBe(Object.keys(gt.zero).length);
      for (const [mode, q] of [['zero', {}], ['q', qConfig(m)]] as const) {
        const poses = urdfLinkPoses(m, q);
        for (const [link, G] of Object.entries(gt[mode])) {
          const T = poses.get(link);
          expect(T, `${mode} ${link}`).toBeDefined();
          const e = poseErr(T!, G);
          expect(e.pos, `${mode} ${link} position (mm)`).toBeLessThan(0.1);
          expect(e.rot, `${mode} ${link} rotation`).toBeLessThan(1e-6);
        }
      }
    });
  }
});

describe('tree import keeps every movable joint and the chain kinematics', () => {
  for (const f of fkFiles) {
    const name = f.replace(/\.json$/, '');
    if (existsSync(join(FIX, 'fk', `${name}.check_err`))) continue;
    it(`${name}: station items reproduce the ROS link poses`, () => {
      const res = importURDF(read(join(FIX, 'xacro', `${name}.urdf`)));
      const m = res.model;
      const gt = gtOf(f);
      // every independent movable joint is a robot joint or a drive wheel / steering joint of the mobile base
      const robotJoints = new Set(res.robots.flatMap((r) => r.chain.joints.map((j) => j.name)));
      const mobileJoints = new Set([...((res.mobile?.params.wheelJoints as string[]) ?? []), ...((res.mobile?.params.steerJoints as string[]) ?? [])]);
      for (const j of m.joints.filter((x) => ['revolute', 'continuous', 'prismatic'].includes(x.type))) {
        const kept = robotJoints.has(j.name) || mobileJoints.has(j.name) || (!!j.mimic && res.report.some((r) => r.includes(j.name)));
        expect(kept, `movable joint ${j.name} kept`).toBe(true);
      }
      // chain link poses at q = 0 and at the GT q (only links whose root path is made of chain joints and fixed joints;
      // q skipped for a robot when the GT value lies outside its joint limits)
      const parentJoint = new Map(m.joints.map((j) => [j.child, j]));
      const pathOk = (link: string) => { for (let pj = parentJoint.get(link); pj; pj = parentJoint.get(pj.parent)) if (pj.type !== 'fixed' && !robotJoints.has(pj.name)) return false; return true; };
      let checked = 0;
      for (const [mode, q] of [['zero', qFull(m, {})], ['q', qFull(m, qConfig(m))]] as const) {
        for (const r of res.robots) {
          const act = r.chain.joints.filter((j) => j.type !== 'fixed' && !j.mimic);
          if (act.some((j) => (q[j.name] ?? 0) < j.lower - 1e-9 || (q[j.name] ?? 0) > j.upper + 1e-9)) continue;
          r.setJoints(act.map((j) => q[j.name] ?? 0));
          const fk = r.fk(r.joints());
          r.chain.links.forEach((l, k) => {
            if (k === 0 || !gt[mode][l.name] || !pathOk(l.name)) return;
            const T = multiply(r.poseAbs(), fk.linkPoses[k]);
            const e = poseErr(T, gt[mode][l.name]);
            expect(e.pos, `${mode} ${r.name}/${l.name}`).toBeLessThan(0.1);
            expect(e.rot, `${mode} ${r.name}/${l.name}`).toBeLessThan(1e-6);
            checked++;
          });
        }
      }
      if (res.robots.length) expect(checked).toBeGreaterThan(0);
    });
  }
});

describe('tree robots decomposed like the ROS model', () => {
  const imp = (n: string) => importURDF(read(join(FIX, 'xacro', `${n}.urdf`)));
  it('MUROM: all 16 movable joints by URDF name (2 arms, head, 2 wheels on a differential base)', () => {
    const res = imp('murom');
    const names = res.robots.flatMap((r) => r.jointNames());
    for (const j of ['head_pan_joint', 'head_tilt_joint', 'r_elbow_joint', 'l_elbow_joint', 'r_wrist_pitch_joint', 'l_shoulder_pitch_joint']) expect(names).toContain(j);
    expect(res.mobile).toBeInstanceOf(MobileRobot);
    expect(res.mobile!.kin.drive).toBe('differential');
    expect((res.mobile!.params.wheelJoints as string[]).sort()).toEqual(['left_wheel_joint', 'right_wheel_joint']);
    const movable = res.model.joints.filter((j) => j.type !== 'fixed' && !j.mimic).length;
    expect(names.length + 2).toBe(movable);
  });
  it('TurtleBot3 / TurtleBot4: differential base with both wheels, no fake arm', () => {
    for (const n of ['turtlebot3_waffle', 'tb4']) {
      const res = imp(n);
      expect(res.mobile?.kin.drive).toBe('differential');
      expect((res.mobile!.params.wheelJoints as string[]).length).toBe(2);
      expect(res.mobile!.kin.track).toBeGreaterThan(100);
    }
  });
  it('UR5e + Robotiq 3F: the arm ends at tool0 (6 DOF), the gripper fingers are their own chains', () => {
    const res = imp('ur5e_gripper');
    const arm = res.robots.find((r) => r.jointNames().includes('shoulder_pan_joint'))!;
    expect(arm.jointNames()).toEqual(['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']);
    const gt = gtOf('ur5e_gripper.json').zero.tool0;
    const e = poseErr(multiply(arm.poseAbs(), arm.fk(arm.joints()).flange), gt);
    expect(e.pos).toBeLessThan(0.1);
    expect(res.robots.length).toBeGreaterThan(1);
  });
  it('MURCA UTP-G: steering and wheel joints kept; no arm even with use_manipulator:=true, exactly like ROS', () => {
    // murca_robot.urdf.xacro tests `${use_manipulator == 'true'}` but xacro types the property as bool True, so ROS
    // never emits the arm (a bug in the MURKA file that the studio reproduces faithfully)
    const res = imp('murca_robot_manip');
    const names = res.robots.flatMap((r) => r.jointNames());
    for (const j of ['utp_g_01/front_left_steering_joint', 'utp_g_01/front_right_steering_joint', 'utp_g_01/back_right_wheel_joint', 'utp_g_01/back_left_wheel_joint']) expect(names).toContain(j);
    expect(names.some((n) => n.includes('arm_joint'))).toBe(false);
  });
  it('mimic side branches are drawn (second finger), floating joints are not actuated, duplicates rejected', () => {
    const urdf = `<robot name="g"><link name="world"/><link name="base"><visual><geometry><box size="0.1 0.1 0.1"/></geometry></visual></link><link name="palm"><visual><geometry><box size="0.1 0.1 0.1"/></geometry></visual></link>
      <link name="f1"><visual><geometry><box size="0.01 0.01 0.05"/></geometry></visual></link><link name="f2"><visual><geometry><box size="0.01 0.01 0.05"/></geometry></visual></link><link name="tip"><visual><geometry><sphere radius="0.01"/></geometry></visual></link>
      <joint name="float" type="floating"><parent link="world"/><child link="base"/></joint>
      <joint name="j1" type="revolute"><parent link="base"/><child link="palm"/><axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint>
      <joint name="fj1" type="prismatic"><parent link="palm"/><child link="f1"/><origin xyz="0 0.02 0.05"/><axis xyz="0 1 0"/><limit lower="0" upper="0.04" effort="1" velocity="1"/></joint>
      <joint name="fj2" type="prismatic"><parent link="palm"/><child link="f2"/><origin xyz="0 -0.02 0.05"/><axis xyz="0 -1 0"/><limit lower="0" upper="0.04" effort="1" velocity="1"/><mimic joint="fj1"/></joint>
      <joint name="tipj" type="fixed"><parent link="f1"/><child link="tip"/><origin xyz="0 0 0.05"/></joint></robot>`;
    const { chain } = chainFromURDF(parseURDF(urdf));
    expect(chain.dof).toBe(2); // j1 + fj1 (floating joint is not a DOF)
    expect(chain.links.flatMap((l) => l.visuals).length).toBe(5); // base, palm, f1, f2 (mimic branch), tip
    const res = importURDF(urdf);
    expect(res.robots.flatMap((r) => r.jointNames())).toEqual(['j1', 'fj1']);
    expect(() => parseURDF('<robot name="d"><link name="a"/><link name="a"/></robot>')).toThrow(/link 'a' is not unique/);
    expect(() => parseURDF('<robot name="d"><link name="a"/><joint name="j" type="fixed"><parent link="a"/><child link="ghost"/></joint></robot>')).toThrow(/child link 'ghost'/);
    expect(() => parseURDF('<robot name="d"><link name="a"/><link name="b"/></robot>')).toThrow(/Two root links/);
  });
  it('named materials defined inside a visual are reusable; continuous joints carry no limits into the tree', () => {
    const m = parseURDF('<robot name="m"><link name="a"><visual><geometry><box size="1 1 1"/></geometry><material name="blue"><color rgba="0 0 1 1"/></material></visual></link><link name="b"><visual><geometry><box size="1 1 1"/></geometry><material name="blue"/></visual></link><joint name="c" type="continuous"><parent link="a"/><child link="b"/><axis xyz="0 0 1"/></joint></robot>');
    expect(m.links.get('b')!.visuals[0].color).toBe('#0000ff');
    const res = importURDF('<robot name="m"><link name="a"/><link name="b"/><joint name="c" type="continuous"><parent link="a"/><child link="b"/><axis xyz="0 0 1"/></joint></robot>');
    expect(res.tree.joints[0].lower).toBeUndefined();
    expect(res.robots[0]).toBeInstanceOf(Robot);
  });
});
