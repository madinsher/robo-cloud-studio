import { describe, it, expect } from 'vitest';
import { demos } from '../src/demos';
import { Station, ItemType, SceneObject, Target, Tool } from '../src/core/items/item';
import { Robot } from '../src/core/items/robot';
import { Program } from '../src/core/items/program';
import { createRobotFromLibrary } from '../src/core/items/library';
import { ProgramSimulator } from '../src/core/motion/simulator';
import { getPos, distance, transl, multiply, rotationAngle } from '../src/core/math/pose';
import { Component, ProcessSimulator } from '../src/vc/component';
import { MRS_EXAMPLES } from '../src/mrs/examples';
import { buildGroupScene } from '../src/mrs/scene';
import { FleetRuntime, RUNNABLE_KINDS } from '../src/mrs/runtime';
import { parseSwarm } from '../src/mrs/dsl';

const station = (id: string) => demos.find((d) => d.id === id)!.build();
const simulate = (st: Station, program: Program) => {
  const sim = new ProgramSimulator(st);
  sim.collisionOptions = { enabled: true, selfOnly: true, sampleStep: 0.02 };
  expect(sim.compile(program).problems).toEqual([]);
  return sim;
};

describe('demo motion and playback regressions', () => {
  for (const demo of demos) it(`${demo.id}: every program is free of self-collisions and reaches its Cartesian targets`, () => {
    const st = demo.build();
    for (const program of st.itemsOfType<Program>(ItemType.PROGRAM)) {
      const sim = simulate(st, program), robot = program.robot() as Robot;
      for (const step of sim.steps) {
        const data = step.instruction.data;
        if (data.kind !== 'move' || !data.targetId) continue;
        const target = st.findById(data.targetId) as Target;
        if (target.isJointTarget) continue;
        sim.seek(step.t1);
        expect(distance(getPos(robot.poseTCPAbs()), getPos(target.poseAbs())), target.name).toBeLessThan(0.1);
        if (robot.dof === 6) expect(rotationAngle(robot.poseTCPAbs(), target.poseAbs()), target.name).toBeLessThan(0.001);
      }
    }
  });

  for (const id of ['tutorial', 'pickplace']) it(`${id}: large seeks, small ticks and replay place the same objects`, () => {
    const run = (ticks: boolean) => {
      const st = station(id), prog = st.itemsOfType<Program>(ItemType.PROGRAM)[0], sim = simulate(st, prog);
      const objects = st.itemsOfType<SceneObject>(ItemType.OBJECT).filter((o) => /^(Part|Box \d+)$/.test(o.name));
      const initial = objects.map((o) => getPos(o.poseAbs()));
      if (ticks) { sim.play(); while (sim.tick(0.017)) {} } else sim.runToEnd();
      const result = objects.map((o) => getPos(o.poseAbs()));
      result.forEach((p, i) => expect(distance(p, initial[i])).toBeGreaterThan(100));
      sim.stop();
      objects.forEach((o, i) => expect(distance(getPos(o.poseAbs()), initial[i])).toBeLessThan(1e-6));
      expect(sim.attachments.size).toBe(0);
      expect((prog.robot() as Robot).activeTool()!.attached).toEqual([]);
      sim.runToEnd();
      objects.forEach((o, i) => expect(distance(getPos(o.poseAbs()), result[i])).toBeLessThan(1e-6));
      return { result, st };
    };
    const direct = run(false), incremental = run(true);
    direct.result.forEach((p, i) => expect(distance(p, incremental.result[i])).toBeLessThan(0.001));
    if (id === 'pickplace') {
      const conveyor = direct.st.find('Out conveyor') as Component;
      direct.result.forEach((p) => {
        expect(p[1]).toBeCloseTo(conveyor.poseAbs()[13], 3);
        expect(p[2]).toBeCloseTo(conveyor.poseAbs()[14] + 1, 3);
      });
      expect(conveyor.geometry.length).toBe(5);
    }
  });

  it('MoveL uses a tool selected inside the program without changing the robot during compilation', () => {
    const st = new Station(), r = st.addChild(createRobotFromLibrary('UR5e'));
    const a = r.addChild(new Tool('Short')), b = r.addChild(new Tool('Long'));
    a.setPoseTool(transl(0, 0, 50)); b.setPoseTool(transl(0, 0, 220)); r.setTool(a);
    const target = st.addChild(new Target('Endpoint'));
    const q = r.joints().map((v, i) => v + (i === 0 ? 5 : 0));
    target.setPose(multiply(r.poseAbs(), r.solveFK(q, b.poseTool())));
    const prog = st.addChild(new Program('Tool change')); prog.setRobot(r); prog.setTool(b); prog.addMoveL(target);
    const sim = new ProgramSimulator(st); expect(sim.compile(prog).problems).toEqual([]);
    expect(r.activeTool()).toBe(a);
    sim.runToEnd();
    expect(r.activeTool()).toBe(b);
    expect(distance(getPos(r.poseTCPAbs()), getPos(target.poseAbs()))).toBeLessThan(0.1);
  });

  it('a colliding initial posture is reported and cannot hide future self-collisions or start playback', () => {
    const st = new Station(), r = st.addChild(createRobotFromLibrary('UR10e'));
    r.setJoints([0, -112, 176, -154, -90, 35]);
    const p = st.addChild(new Program('Folded')); p.setRobot(r); p.addMoveJ(r.joints());
    const sim = new ProgramSimulator(st); sim.collisionOptions = { enabled: true, selfOnly: true };
    expect(sim.compile(p).ok).toBe(false); expect(sim.collisions.length).toBeGreaterThan(0);
    sim.play(); expect(sim.playing).toBe(false);
  });

  it('packing products stay at the grader support height', () => {
    const st = station('packing'), sim = new ProcessSimulator(st), grader = st.find('Optical grader') as Component;
    let observed = false;
    for (let i = 0; i < 300; i++) {
      sim.step(0.1);
      for (const id of grader.products) { observed = true; expect(st.findById(id)!.poseAbs()[14]).toBe(800); }
    }
    expect(observed).toBe(true);
  });
});

describe('runnable group examples', () => {
  for (const ex of MRS_EXAMPLES.filter((e) => RUNNABLE_KINDS.includes(e.kind) && (e.kind !== 'swarm' || ['boids', 'vicsek', 'pso'].includes(parseSwarm(e.source).model)))) {
    it(`${ex.id}: builds a complete scene and runs with finite poses`, () => {
      const st = new Station(), robots = buildGroupScene(st, ex.kind, ex.source);
      expect(robots.length).toBeGreaterThan(0);
      const count = [...st.walk()].length;
      expect(buildGroupScene(st, ex.kind, ex.source).map((r) => r.id)).toEqual(robots.map((r) => r.id));
      expect([...st.walk()].length).toBe(count);
      const rt = new FleetRuntime({ kind: ex.kind, source: ex.source, robots, station: st, maxSeconds: 3 });
      for (let i = 0; i < 100 && !rt.done; i++) rt.tick(0.05);
      expect(rt.done).toBe(true);
      for (const r of robots) expect([r.state.x, r.state.y, r.state.theta].every(Number.isFinite)).toBe(true);
      expect(rt.log.some((l) => /only \d+ of/.test(l))).toBe(false);
    });
  }
});
