import { it, expect } from 'vitest';
import { Station, Frame, Target } from '../src/core/items/item';
import { Program } from '../src/core/items/program';
import { createRobotFromLibrary } from '../src/core/items/library';
import { ProgramSimulator } from '../src/core/motion/simulator';
import { transl } from '../src/core/math/pose';

it('threads run two robots in parallel and the joint list covers the program', () => {
  const st = new Station();
  const a = st.addChild(createRobotFromLibrary('UR5e', 'A'));
  const b = st.addChild(createRobotFromLibrary('UR5e', 'B'));
  b.setPose(transl(2000, 0, 0));
  const pa = st.addChild(new Program('Main')); pa.setRobot(a);
  const pb = st.addChild(new Program('Side')); pb.setRobot(b);
  const ta = st.addChild(new Target('ta')); ta.setJoints([30, -90, 90, -90, -90, 0]); ta.setAsJointTarget();
  const tb = st.addChild(new Target('tb')); tb.setJoints([-60, -90, 90, -90, -90, 0]); tb.setAsJointTarget();
  pb.addMoveJ(tb); pb.pause(2000);
  pa.startThread(pb);
  pa.addMoveJ(ta);
  pa.waitSignal('done', true, 500);
  const sim = new ProgramSimulator(st);
  const res = sim.compile(pa);
  expect(res.problems.filter((p) => p.severity === 'error')).toEqual([]);
  // the thread (move + 2 s pause) is longer than the main program: duration covers it
  expect(sim.duration).toBeGreaterThan(2);
  sim.runToEnd();
  expect(Math.round(a.joints()[0])).toBe(30);
  expect(Math.round(b.joints()[0])).toBe(-60);
  const rows = sim.jointsList(a.id, 0.1);
  expect(rows.length).toBeGreaterThan(3);
  expect(rows[0].length).toBe(2 + 6);
});
