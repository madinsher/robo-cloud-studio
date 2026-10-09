import { describe, it, expect } from 'vitest';
import { demos } from '../src/demos';
import { ProgramSimulator } from '../src/core/motion/simulator';
import { Program } from '../src/core/items/program';
import { Robot } from '../src/core/items/robot';
import { ItemType } from '../src/core/items/item';
import { compileForPost, getPost } from '../src/posts/base';
import '../src/posts/index';

describe('tutorial demo station', () => {
  it('compiles without problems, reaches every target and exports through the UR post', () => {
    const st = demos.find((d) => d.id === 'tutorial')!.build();
    const prog = st.itemsOfType<Program>(ItemType.PROGRAM)[0];
    const robot = st.itemsOfType<Robot>(ItemType.ROBOT)[0];
    const sim = new ProgramSimulator(st);
    const res = sim.compile(prog);
    expect(res.problems.map((p) => p.message)).toEqual([]);
    expect(res.ok).toBe(true);
    expect(sim.duration).toBeGreaterThan(3);
    expect(sim.duration).toBeLessThan(40);
    // every cartesian target lies inside the reach sphere with margin
    for (const t of st.itemsOfType(ItemType.TARGET)) {
      const p = t.poseAbs(); const b = robot.poseAbs();
      const d = Math.hypot(p[12] - b[12], p[13] - b[13], p[14] - b[14]);
      expect(d, t.name).toBeLessThan(robot.reach * 0.85);
    }
    const post = getPost(robot.postProcessor)!;
    const files = post.generate(compileForPost(st, prog));
    expect(files[0].content).toMatch(/movel|movej/i);
  });
  it('all demo stations build and their first program compiles', () => {
    for (const d of demos) {
      const st = d.build();
      const prog = st.itemsOfType<Program>(ItemType.PROGRAM)[0];
      if (!prog) continue;
      const res = new ProgramSimulator(st).compile(prog);
      expect(res.problems.filter((p) => p.severity === 'error').map((p) => `${d.id}: ${p.message}`)).toEqual([]);
    }
  });
});
