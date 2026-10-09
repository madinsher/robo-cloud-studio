import { describe, it, expect } from 'vitest';
import { Station, Frame, Target, Tool, SceneObject } from '../src/core/items/item';
import { Program } from '../src/core/items/program';
import { createRobotFromLibrary } from '../src/core/items/library';
import { ProgramSimulator } from '../src/core/motion/simulator';
import { compileForPost, listPosts, getPost } from '../src/posts/index';
import { importProgram } from '../src/io/programs/import';
import { importTargets, exportTargets } from '../src/io/robodk/targets';
import { stationToRoboDKScript } from '../src/io/robodk/station_script';
import { mul, transl, rotx, DEG, getPos, distance } from '../src/core/math/pose';
import { saveStation, loadStation } from '../src/io/station-file';

function buildCell() {
  const st = new Station('Cell');
  const robot = st.addChild(createRobotFromLibrary('UR10e', 'UR10e'));
  const tool = robot.addChild(new Tool('Gripper'));
  tool.setPoseTool(transl(0, 0, 120));
  robot.setTool(tool);
  const frame = st.addChild(new Frame('Table'));
  frame.setPose(transl(600, 0, 0));
  robot.setFrame(frame);
  const home = frame.addChild(new Target('Home'));
  home.setJoints([0, -90, 90, -90, -90, 0]);
  home.setAsJointTarget();
  const t1 = frame.addChild(new Target('Approach'));
  t1.setPose(mul(transl(100, 100, 300), rotx(180 * DEG)));
  const t2 = frame.addChild(new Target('Pick'));
  t2.setPose(mul(transl(100, 100, 150), rotx(180 * DEG)));
  const box = st.addChild(new SceneObject('Box'));
  box.setPose(transl(700, 100, 100));
  const prog = st.addChild(new Program('PickPlace'));
  prog.setRobot(robot);
  prog.setSpeed(400, 120);
  prog.addMoveJ(home);
  prog.addMoveJ(t1);
  prog.addMoveL(t2);
  prog.event('attach', box.id);
  prog.pause(500);
  prog.addMoveL(t1);
  prog.setDO('DO_1', true);
  prog.addMoveJ(home);
  prog.event('detach', box.id);
  return { st, robot, prog, frame, box, tool };
}

describe('program simulation', () => {
  it('compiles and simulates a pick & place with attachments', () => {
    const { st, prog, robot, box } = buildCell();
    const sim = new ProgramSimulator(st);
    const res = sim.compile(prog);
    expect(res.problems.filter((p) => p.severity === 'error')).toEqual([]);
    expect(res.duration).toBeGreaterThan(1);
    expect(res.distance).toBeGreaterThan(100);
    const boxBefore = getPos(box.poseAbs());
    // seek to the middle of the retreat move: box should have moved with the tool
    const retreat = sim.steps.find((s) => s.instruction.name === 'MoveL' && s.t0 > 1)!;
    sim.seek(retreat.t1);
    const boxAfter = getPos(box.poseAbs());
    expect(distance(boxBefore, boxAfter)).toBeGreaterThan(50);
    sim.runToEnd();
    expect(robot.joints().map((v) => Math.round(v))).toEqual([0, -90, 90, -90, -90, 0]);
    expect(sim.io['DO_1']).toBe(true);
  });

  it('reports unreachable targets', () => {
    const { st, prog, frame } = buildCell();
    const far = frame.addChild(new Target('Far'));
    far.setPose(transl(5000, 0, 0));
    prog.addMoveL(far);
    const res = new ProgramSimulator(st).compile(prog);
    expect(res.ok).toBe(false);
    expect(res.problems.some((p) => /unreachable|resolved|MoveL|jump|singularity/.test(p.message))).toBe(true);
  });
});

describe('post processors', () => {
  it('generates code for every registered post', () => {
    const { st, prog } = buildCell();
    const compiled = compileForPost(st, prog);
    expect(compiled.problems).toEqual([]);
    expect(compiled.events.filter((e) => e.kind.startsWith('move')).length).toBe(5);
    for (const post of listPosts()) {
      const files = post.generate(compiled);
      expect(files.length, post.id).toBeGreaterThan(0);
      expect(files[0].content.length, post.id).toBeGreaterThan(50);
    }
    const krl = getPost('KUKA_KRC4')!.generate(compiled)[0].content;
    expect(krl).toMatch(/DEF PICKPLACE/);
    expect(krl).toMatch(/LIN \{X/);
    const rapid = getPost('ABB_RAPID_IRC5')!.generate(compiled)[0].content;
    expect(rapid).toMatch(/MoveL \[\[/);
    const ur = getPost('Universal_Robots')!.generate(compiled)[0].content;
    expect(ur).toMatch(/movel\(pose_trans/);
    const ls = getPost('Fanuc_R30iA')!.generate(compiled)[0].content;
    expect(ls).toMatch(/\/PROG/);
    expect(ls).toMatch(/P\[1/);
  });

  it('round-trips through KRL, RAPID, LS and URScript importers', () => {
    const { st, prog, robot } = buildCell();
    const compiled = compileForPost(st, prog);
    for (const [postId, lang] of [['KUKA_KRC4', 'krl'], ['ABB_RAPID_IRC5', 'rapid'], ['Fanuc_R30iA', 'ls'], ['Universal_Robots', 'urscript'], ['Generic', 'csv']] as const) {
      const code = getPost(postId)!.generate(compiled)[0].content;
      const res = importProgram(code, { station: st, robot, language: lang, name: `Imported_${lang}` });
      const moves = res.program.instructions().filter((i) => i.data.kind === 'move');
      expect(moves.length, postId).toBe(5);
      // the cartesian target poses must match (in the active frame)
      const pick = res.targets.find((t) => /Pick/i.test(t.name)) ?? res.targets[2];
      const orig = compiled.events.find((e) => e.kind === 'moveL')!;
      if (pick && orig.kind === 'moveL' && !pick.isJointTarget) {
        expect(distance(getPos(pick.pose()), getPos(orig.pose)), postId).toBeLessThan(0.05);
      }
    }
  });

  it('RoboDK station script and target CSV export/import work', () => {
    const { st, frame } = buildCell();
    const script = stationToRoboDKScript(st);
    expect(script).toContain('RDK.AddProgram');
    expect(script).toContain('AddTarget');
    const csv = exportTargets(frame.childrenOfType(6));
    const st2 = new Station();
    const f2 = st2.addChild(new Frame('F'));
    const imported = importTargets(csv, { parent: f2 });
    expect(imported.length).toBe(3);
    expect(distance(getPos(imported[1].pose()), [100, 100, 300])).toBeLessThan(1e-3);
  });

  it('station file save/load preserves programs and robots', () => {
    const { st } = buildCell();
    const file = saveStation(st);
    const st2 = loadStation(JSON.parse(JSON.stringify(file)));
    const prog = st2.find('PickPlace') as Program;
    expect(prog.instructions().length).toBe(10);
    const res = new ProgramSimulator(st2).compile(prog);
    expect(res.ok).toBe(true);
  });
});
