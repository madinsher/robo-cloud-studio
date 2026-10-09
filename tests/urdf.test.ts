import { describe, it, expect } from 'vitest';
import { readFileSync, existsSync } from 'node:fs';
import { parseURDF, chainFromURDF, robotFromURDF } from '../src/io/urdf/urdf';
import { processXacro } from '../src/io/urdf/xacro';
import { forwardKinematics } from '../src/core/kinematics/chain';

const repo = new URL('../../', import.meta.url).pathname;

describe('URDF / xacro import', () => {
  it('parses the vertical robot URDF from this repository', () => {
    const p = repo + 'vertical_robot_model/urdf/vertical_robot.urdf';
    if (!existsSync(p)) return;
    const model = parseURDF(readFileSync(p, 'utf8'));
    expect(model.name).toBe('vertical_robot');
    expect(model.joints.length).toBeGreaterThan(4);
    expect(model.meshes.some((m) => m.includes('base.stl'))).toBe(true);
    const { chain } = chainFromURDF(model);
    expect(chain.dof).toBeGreaterThan(0);
    const fk = forwardKinematics(chain, new Array(chain.dof).fill(0));
    expect(fk.flange.length).toBe(16);
  });

  it('parses the palletizer xacro with includes', () => {
    const dir = repo + 'palletizer_model_pkg/urdf/';
    const p = dir + 'palletizer_model.urdf.xacro';
    if (!existsSync(p)) return;
    const text = readFileSync(p, 'utf8');
    const robot = robotFromURDF(text, {
      resolveInclude: (f) => {
        const name = f.split('/').pop()!;
        return existsSync(dir + name) ? readFileSync(dir + name, 'utf8') : null;
      },
    });
    expect(robot.dof).toBeGreaterThanOrEqual(3);
    const q = robot.jointsHome();
    const pose = robot.solveFK(q);
    expect(Number.isFinite(pose[12])).toBe(true);
  });

  it('xacro properties, math and macros expand', () => {
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
    expect(out).toContain('size="0.5 1 1"');
    expect(out).toContain('name="b"');
    expect(out).toContain('name="c"');
    expect(out).not.toContain('xacro:');
  });
});
