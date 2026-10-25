import { describe, it, expect } from 'vitest';
import { existsSync } from 'node:fs';
import { parseURDF, chainFromURDF, robotFromURDF, importURDF } from '../src/io/urdf/urdf';
import { processXacro } from '../src/io/urdf/xacro';
import { forwardKinematics } from '../src/core/kinematics/chain';
import { VBOT, HAS_VBOT, read, diskResolver, join } from './ros_gt_util';

// The vertical_bot_ros packages live next to this repository (C:/vertical_bot_ros); the former tests resolved a wrong
// path and returned early, so they could never fail. They are skipped visibly when the packages are absent.
const VR = `${VBOT}/vertical_robot_model/urdf/vertical_robot.urdf`;
const PAL = `${VBOT}/palletizer_model_pkg/urdf/palletizer_model.urdf.xacro`;

describe('URDF / xacro import', () => {
  it.skipIf(!existsSync(VR))('vertical_robot.urdf: 4 actuated joints and all 7 visuals survive the import', () => {
    const model = parseURDF(read(VR));
    expect(model.name).toBe('vertical_robot');
    expect(model.joints.filter((j) => j.type !== 'fixed' && !j.mimic).length).toBe(4);
    expect(model.meshes.some((m) => m.includes('base.stl'))).toBe(true);
    const visuals = [...model.links.values()].reduce((n, l) => n + l.visuals.length, 0);
    expect(visuals).toBe(7);
    // serial-chain view keeps every visual (mimic branches folded static)
    const { chain } = chainFromURDF(model);
    expect(chain.links.reduce((n, l) => n + l.visuals.length, 0)).toBe(7);
    expect(forwardKinematics(chain, new Array(chain.dof).fill(0)).flange.length).toBe(16);
    // tree import: both carriages and both lifts are drivable by their URDF names
    const res = importURDF(read(VR));
    expect(res.robots.flatMap((r) => r.jointNames()).sort()).toEqual(['joint_1', 'joint_1_1', 'joint_2', 'joint_2_1']);
  });

  it.skipIf(!HAS_VBOT || !existsSync(PAL))('palletizer xacro with includes: all 12 visuals and every joint', () => {
    const dir = `${VBOT}/palletizer_model_pkg`;
    const opts = { filename: 'package://palletizer_model_pkg/urdf/palletizer_model.urdf.xacro', packages: { palletizer_model_pkg: dir }, resolveInclude: diskResolver(join(dir, 'urdf'), { palletizer_model_pkg: dir }) };
    const model = parseURDF(read(PAL), opts);
    expect([...model.links.values()].reduce((n, l) => n + l.visuals.length, 0)).toBe(12);
    const robot = robotFromURDF(read(PAL), opts);
    expect(robot.dof).toBeGreaterThanOrEqual(3);
    expect(robot.chain.links.reduce((n, l) => n + l.visuals.length, 0)).toBe(12);
    const res = importURDF(read(PAL), opts);
    const kept = new Set(res.robots.flatMap((r) => r.chain.joints.map((j) => j.name)));
    for (const j of ['pal_joint0', 'pal_joint1', 'pal_joint2', 'pal_joint3']) expect(kept.has(j)).toBe(true);
  });

  it('xacro properties, math and macros expand with ROS typing', () => {
    const src = `<robot name="t" xmlns:xacro="http://ros.org/wiki/xacro">
      <xacro:property name="w" value="0.5"/>
      <xacro:macro name="leg" params="name x">
        <link name="\${name}"><visual><geometry><box size="\${w} \${w*2} \${x}"/></geometry></visual></link>
      </xacro:macro>
      <xacro:leg name="a" x="1"/>
      <xacro:leg name="b" x="\${pi/2}"/>
      <xacro:if value="\${w > 0.1}"><link name="c"/></xacro:if>
    </robot>`;
    const out = processXacro(src);
    expect(out).toContain('size="0.5 1.0 1"'); // 0.5*2 is the float 1.0, "1" stays the int 1 (ROS xacro output)
    expect(out).toContain('size="0.5 1.0 1.5707963267948966"');
    expect(out).toContain('name="b"');
    expect(out).toContain('name="c"');
    expect(out).not.toContain('xacro:');
  });
});
