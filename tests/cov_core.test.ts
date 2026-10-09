import { describe, it, expect, vi } from 'vitest';
import { EventBus, appEvents } from '../src/core/events';
import { EventQueue } from '../src/core/events-queue';
import { mm, m, clamp, lerp, wrapPi, wrap180, approx, round, uid, MM_PER_M } from '../src/core/units';
import { Program, Instruction, InstructionType, describeInstruction, moveData, PathItem } from '../src/core/items/program';
import { ProgramSimulator, estimateProgram } from '../src/core/motion/simulator';
import { trapezoid, planMoveJ, planMoveL, planMoveC, sampleAt } from '../src/core/motion/trajectory';
import { SpraySimulator } from '../src/core/motion/spray';
import { generateMachining } from '../src/core/motion/machining';
import { calibrateTcpByPoint, calibrateTcpByLine } from '../src/core/calibration/tcp';
import { pathAccuracy, createBallbarProgram, ballbarAnalysis, simulateTrackerMeasure } from '../src/core/calibration/iso9283';
import { calibrateFrame, CALIBRATE_FRAME_3P_P1_ON_X, CALIBRATE_FRAME_3P_P1_ORIGIN } from '../src/core/calibration/frame';
import { Clipboard, clipboard } from '../src/core/items/clone';
import { createRobotFromLibrary, ROBOT_LIBRARY } from '../src/core/items/library';
import { Station, Frame, Target, Tool, SceneObject, Folder, ItemType } from '../src/core/items/item';
import { AssetStore } from '../src/scene/assets';
import { writeBinarySTL } from '../src/io/mesh/stl';
import { transl, mul, rotx, roty, rotz, DEG, getPos, identity, distance, invert, multiply, transformPoint } from '../src/core/math/pose';

describe('EventBus', () => {
  it('subscribes, unsubscribes, fires once, isolates throwing listeners and clears', () => {
    const bus = new EventBus<{ a: number; b: string }>();
    const got: number[] = [];
    const off = bus.on('a', (v) => got.push(v));
    bus.emit('a', 1);
    const onceFn = vi.fn();
    bus.once('a', onceFn);
    bus.emit('a', 2);
    bus.emit('a', 3);
    expect(got).toEqual([1, 2, 3]);
    expect(onceFn).toHaveBeenCalledTimes(1);
    expect(onceFn).toHaveBeenCalledWith(2);
    off();
    bus.emit('a', 4);
    expect(got).toEqual([1, 2, 3]);
    const err = vi.spyOn(console, 'error').mockImplementation(() => {});
    const after = vi.fn();
    bus.on('b', () => { throw new Error('boom'); });
    bus.on('b', after);
    bus.emit('b', 'x');
    expect(after).toHaveBeenCalledWith('x');
    expect(err).toHaveBeenCalledTimes(1);
    err.mockRestore();
    bus.emit('unknown' as any, 1); // no listeners: no-op
    bus.clear();
    bus.emit('b', 'y');
    expect(after).toHaveBeenCalledTimes(1);
    // off() of an unknown event is harmless; the app bus is a working instance
    bus.off('a', () => {});
    const spy = vi.fn();
    const offApp = appEvents.on('cov-test', spy);
    appEvents.emit('cov-test', { ok: true });
    offApp();
    expect(spy).toHaveBeenCalledWith({ ok: true });
  });
});

describe('EventQueue', () => {
  it('queues, hands events to waiters, times out and drains', async () => {
    const q = new EventQueue();
    q.push({ type: 'changed', itemId: 'x' });
    const first = await q.wait(10);
    expect(first).toMatchObject({ type: 'changed', itemId: 'x' });
    expect(first!.t).toBeGreaterThan(0);
    const pending = q.wait(1000);
    q.push({ type: 'selection', data: [1] });
    expect(await pending).toMatchObject({ type: 'selection', data: [1] });
    expect(await q.wait(5)).toBeNull();
    for (let i = 0; i < 505; i++) q.push({ type: `e${i}` });
    const all = q.drain();
    expect(all.length).toBe(500); // capped
    expect(all[0].type).toBe('e5');
    expect(q.drain()).toEqual([]);
    q.push({ type: 'z' });
    q.clear();
    expect(await q.wait(1)).toBeNull();
  });
});

describe('units', () => {
  it('converts and wraps', () => {
    expect(MM_PER_M).toBe(1000);
    expect(mm(1.5)).toBe(1500);
    expect(m(250)).toBe(0.25);
    expect(clamp(5, 0, 3)).toBe(3);
    expect(clamp(-1, 0, 3)).toBe(0);
    expect(lerp(10, 20, 0.25)).toBe(12.5);
    expect(wrapPi(3 * Math.PI)).toBeCloseTo(Math.PI, 12);
    expect(wrapPi(-3.5 * Math.PI)).toBeCloseTo(0.5 * Math.PI, 12);
    expect(wrap180(190)).toBe(-170);
    expect(wrap180(-190)).toBe(170);
    expect(wrap180(540)).toBe(180);
    expect(approx(1, 1 + 1e-7)).toBe(true);
    expect(approx(1, 1.1)).toBe(false);
    expect(approx(1, 1.1, 0.2)).toBe(true);
    expect(round(Math.PI)).toBe(3.142);
    expect(round(1234.5678, 0)).toBe(1235);
    const a = uid(), b = uid('t');
    expect(a).toMatch(/^id_/);
    expect(b).toMatch(/^t_/);
    expect(a).not.toBe(uid());
  });
});

describe('Program / Instruction model', () => {
  it('maps every instruction kind to an InstructionType and a description', () => {
    const p = new Program('P');
    const t = new Target('T');
    const sub = new Program('S');
    const cases: Array<[Instruction, InstructionType, RegExp]> = [
      [p.addMoveJ(t), InstructionType.MOVE, /^MoveJ$/],
      [p.addMoveC(t, t), InstructionType.MOVEC, /^MoveC$/],
      [p.addInstruction({ kind: 'jointPath', joints: [[1], [2]], source: 'csv' }), InstructionType.MOVE, /Joint path \(2 points, csv\)/],
      [p.setSpeed(100), InstructionType.CHANGESPEED, /Set Speed \(100 mm\/s, - deg\/s\)/],
      [p.setFrame(null), InstructionType.CHANGEFRAME, /Set Reference Frame/],
      [p.setTool(null), InstructionType.CHANGETOOL, /Set Tool Frame/],
      [p.pause(50), InstructionType.PAUSE, /Pause 50 ms/],
      [p.pause(-1), InstructionType.PAUSE, /Pause \(wait user\)/],
      [p.event('attach', 'o'), InstructionType.EVENT, /Event: attach/],
      [p.runInstruction('foo'), InstructionType.CODE, /Call foo/],
      [p.runInstruction('a very long piece of code that keeps going', false), InstructionType.CODE, /^Code: a very long piece of code that$/], // 30 chars
      [p.showMessage('hi'), InstructionType.PRINT, /Show "hi"/],
      [p.comment('c'), InstructionType.PRINT, /\/\/ c/],
      [p.setRounding(4), InstructionType.ROUNDING, /Rounding 4 mm/],
      [p.setDO('D', 1), InstructionType.IO, /Set D = 1/],
      [p.waitDI('D', 0), InstructionType.IO, /Wait D = 0/],
      [p.callProgram(sub), InstructionType.CALL, /Call program S/],
      [p.navigateTo(10.4, 20.6, 0, { label: 'dock' }), InstructionType.MOBILE_MOVE, /Navigate to \(10, 21\) dock/],
      [p.addInstruction({ kind: 'mobile_follow', pathId: null }), InstructionType.MOBILE_FOLLOW, /Follow path/],
      [p.addInstruction({ kind: 'mission_task', task: 'spray', params: {} }), InstructionType.MISSION_TASK, /Task: spray/],
      [p.addInstruction({ kind: 'signal', signal: 's', value: 1, wait: false }), InstructionType.SET_SIGNAL, /Set signal s = 1/],
      [p.addInstruction({ kind: 'signal', signal: 's', value: 1, wait: true }), InstructionType.WAIT_SIGNAL, /Wait signal s = 1/],
      [p.addInstruction({ kind: 'loop', count: 3, bodyProgramId: null }), InstructionType.LOOP, /Loop x3/],
      [p.addInstruction({ kind: 'if', condition: 'x>1', thenProgramId: null }), InstructionType.IF, /If x>1/],
      [p.startThread(sub), InstructionType.THREAD, /Start thread S/],
      [p.addInstruction({ kind: 'wait', what: 'time', timeMs: 20 }), InstructionType.WAIT, /Wait 20 ms/],
      [p.waitSignal('s', true), InstructionType.WAIT, /Wait s = true/],
      [p.addInstruction({ kind: 'wait', what: 'move_done' }), InstructionType.WAIT, /Wait move done/],
    ];
    for (const [ins, type, re] of cases) {
      expect(ins.insType, ins.name).toBe(type);
      expect(ins.name).toMatch(re);
      expect(ins).toBeInstanceOf(Instruction);
    }
    expect(p.instructions().length).toBe(cases.length);
    expect(describeInstruction({ kind: 'call', programId: 'id1' })).toBe('Call program id1');
    expect(describeInstruction({ kind: 'thread', programId: null })).toBe('Start thread ');
    expect(describeInstruction({ kind: 'jointPath', joints: [] })).toBe('Joint path (0 points)');
    // moveData variants
    expect(moveData('MoveL', identity()).pose!.length).toBe(16);
    expect(moveData('MoveJ', [1, 2], { speed: 5 })).toEqual({ kind: 'move', moveType: 'MoveJ', targetId: null, joints: [1, 2], speed: 5 });
    expect(moveData('MoveJ', t).targetId).toBe(t.id);
  });
  it('serializes programs, instructions, run mode and paths through the station', () => {
    const st = new Station('s');
    const robot = st.addChild(createRobotFromLibrary('UR5e'));
    const f = st.addChild(new Frame('F'));
    const tool = robot.addChild(new Tool('T'));
    const p = st.addChild(new Program('P'));
    p.setRobot(robot);
    p.frameId = f.id;
    p.toolId = tool.id;
    p.runMode = 'run_on_robot';
    const ins = p.addMoveJ([1, 2, 3, 4, 5, 6]);
    ins.enabled = false;
    p.pause(10);
    const path = st.addChild(new PathItem('Path'));
    path.points = [[0, 0, 0], [3, 4, 0], [3, 4, 12]];
    path.speed = 250;
    path.width = 50;
    expect(path.length()).toBe(17);
    path.closed = true;
    expect(path.length()).toBe(17 + Math.hypot(3, 4, 12));
    expect(new PathItem().length()).toBe(0);
    const st2 = Station.deserialize(JSON.parse(JSON.stringify(st.serialize())));
    const p2 = st2.find('P') as Program;
    expect(p2.robot()?.name).toBe(robot.name);
    expect(p2.frameId).toBe(f.id);
    expect(p2.toolId).toBe(tool.id);
    expect(p2.runMode).toBe('run_on_robot');
    expect(p2.instructions().length).toBe(2);
    expect(p2.instructions()[0].enabled).toBe(false);
    expect(p2.instructions()[0].data).toEqual(ins.data);
    expect(p2.instructions()[1].enabled).toBe(true);
    const path2 = st2.find('Path') as PathItem;
    expect(path2.points).toEqual(path.points);
    expect(path2.closed).toBe(true);
    expect(path2.speed).toBe(250);
    expect(path2.width).toBe(50);
    p.setRobot(null);
    expect(p.robot()).toBeNull();
    expect(new Program('lonely').robot()).toBeNull();
  });
});

describe('trajectory planning', () => {
  it('trapezoid profiles: zero, triangular and trapezoidal', () => {
    expect(trapezoid(0, 1, 1).duration).toBe(0);
    expect(trapezoid(0, 1, 1).s(1)).toBe(0);
    const tri = trapezoid(1, 10, 1); // never reaches v
    expect(tri.duration).toBeCloseTo(2, 12);
    expect(tri.s(0)).toBe(0);
    expect(tri.s(1)).toBeCloseTo(0.5, 12);
    expect(tri.s(2)).toBeCloseTo(1, 12);
    expect(tri.s(99)).toBeCloseTo(1, 12);
    const trap = trapezoid(10, 1, 1);
    expect(trap.duration).toBeCloseTo(11, 12);
    expect(trap.s(0.5)).toBeCloseTo(0.125, 12);
    expect(trap.s(1)).toBeCloseTo(0.5, 12);
    expect(trap.s(5.5)).toBeCloseTo(5, 12);
    expect(trap.s(11)).toBeCloseTo(10, 12);
    expect(trap.s(-1)).toBe(0);
  });
  it('plans joint, linear and circular moves and samples them', () => {
    const r = createRobotFromLibrary('UR10e');
    const q0 = [0, -90, 90, -90, -90, 0];
    const q1 = [30, -80, 80, -90, -90, 20];
    const j = planMoveJ(r, q0, q1, 90, 360);
    expect(j.ok).toBe(true);
    expect(j.duration).toBeCloseTo(30 / 90 + 90 / 360, 9);
    expect(j.samples[0].joints).toEqual(q0);
    expect(j.samples.at(-1)!.joints.map((v) => +v.toFixed(9))).toEqual(q1);
    expect(j.length).toBeGreaterThan(0);
    const mid = sampleAt(j, j.duration / 2);
    expect(mid.joints[0]).toBeCloseTo(15, 6);
    expect(sampleAt(j, -1)).toBe(j.samples[0]);
    expect(sampleAt(j, 99)).toBe(j.samples.at(-1));
    expect(() => sampleAt({ samples: [], duration: 0, length: 0, ok: true }, 0)).toThrow(/empty/);
    const same = planMoveJ(r, q0, q0, 90, 360);
    expect(same.duration).toBe(0);
    expect(same.samples.length).toBe(2);
    const bad = planMoveJ(r, q0, [0, -90, 90, -90, -90, 1000], 90, 360);
    expect(bad.ok).toBe(false);
    expect(bad.error).toMatch(/outside limits/);

    const p0 = r.solveFK(q0);
    const p1 = mul(p0, transl(0, 0, 150)); // pure translation in tool frame
    const l = planMoveL(r, q0, p1, 300, 2000);
    expect(l.ok).toBe(true);
    expect(l.length).toBeCloseTo(150, 3);
    expect(l.duration).toBeCloseTo(trapezoid(150, 300, 2000).duration, 9);
    expect(distance(getPos(l.samples.at(-1)!.pose), getPos(p1))).toBeLessThan(1e-6);
    expect(distance(getPos(r.solveFK(l.samples.at(-1)!.joints)), getPos(p1))).toBeLessThan(0.5);
    // rotation-dominated MoveL: duration bounded by the rotational profile
    const rot = planMoveL(r, q0, mul(p0, rotz(90 * DEG)), 5000, 50000, 0.02, 45);
    expect(rot.ok).toBe(true);
    expect(rot.duration).toBeGreaterThan(1.5);
    const far = planMoveL(r, q0, transl(4000, 0, 0), 300, 2000);
    expect(far.ok).toBe(false);
    expect(far.error).toMatch(/unreachable|singularity|Joint jump/);

    const pVia = mul(p0, transl(50, 50, 0));
    const pEnd = mul(p0, transl(100, 0, 0));
    const c = planMoveC(r, q0, pVia, pEnd, 300, 2000);
    expect(c.ok).toBe(true);
    // the three points lie on a circle of radius 50 around (50, 0): arc length = pi * 50
    expect(c.length).toBeCloseTo(Math.PI * 50, 0);
    expect(distance(getPos(c.samples.at(-1)!.pose), getPos(pEnd))).toBeLessThan(1e-6);
    const viaPos = getPos(pVia);
    expect(Math.min(...c.samples.map((s) => distance(getPos(s.pose), viaPos)))).toBeLessThan(5);
    // collinear via -> falls back to a straight line
    const lin = planMoveC(r, q0, mul(p0, transl(50, 0, 0)), pEnd, 300, 2000);
    expect(lin.ok).toBe(true);
    expect(lin.length).toBeCloseTo(100, 3);
    // going the other way round (via beyond the end)
    const c2 = planMoveC(r, q0, mul(p0, transl(50, -50, 0)), pEnd, 300, 2000);
    expect(c2.ok).toBe(true);
    expect(c2.length).toBeCloseTo(Math.PI * 50, 0);
    const cFar = planMoveC(r, q0, mul(p0, transl(1500, 1500, 0)), mul(p0, transl(3000, 0, 0)), 300, 2000);
    expect(cFar.ok).toBe(false);
    expect(cFar.error).toMatch(/MoveC unreachable/);
  });
});

describe('ProgramSimulator', () => {
  function cell() {
    const st = new Station('sim');
    const robot = st.addChild(createRobotFromLibrary('UR10e', 'R'));
    robot.setPose(transl(0, 0, 500));
    const tool = robot.addChild(new Tool('Grip'));
    tool.setPoseTool(transl(0, 0, 100));
    robot.setTool(tool);
    const frame = st.addChild(new Frame('F'));
    frame.setPose(transl(600, 0, 0));
    robot.setFrame(frame);
    const home = frame.addChild(new Target('Home'));
    home.setJoints([0, -100, 110, -100, -90, 0]);
    home.setAsJointTarget();
    const above = frame.addChild(new Target('Above'));
    above.setPose(mul(transl(0, 0, 500), rotx(180 * DEG)));
    const pick = frame.addChild(new Target('Pick'));
    pick.setPose(mul(transl(0, 0, 400), rotx(180 * DEG)));
    const via = frame.addChild(new Target('Via'));
    via.setPose(mul(transl(50, 50, 400), rotx(180 * DEG)));
    const box = frame.addChild(new SceneObject('Box'));
    box.setPose(transl(0, 0, 300));
    box.geometry = [{ primitive: { kind: 'box', size: [100, 100, 100] }, origin: Array.from(transl(0, 0, 50)) }];
    return { st, robot, tool, frame, home, above, pick, via, box };
  }
  it('compiles a full program with attachments, sub programs, loops, threads and waits; plays it back', () => {
    const { st, robot, tool, frame, home, above, pick, via, box } = cell();
    const sub = st.addChild(new Program('Sub'));
    sub.setRobot(robot);
    sub.addMoveJ(home);
    sub.pause(100);
    const par = st.addChild(new Program('Par'));
    par.setRobot(robot);
    par.pause(3000);
    const prog = st.addChild(new Program('Main'));
    prog.setRobot(robot);
    prog.setSpeed(800, 120, 3000, 500);
    prog.setRounding(5);
    prog.addMoveJ(home);
    prog.addMoveJ(above);
    prog.addMoveL(pick, { speed: 200 });
    prog.event('gripper_close', box.id);
    prog.setDO('Vac', true);
    prog.addMoveL(above);
    prog.addMoveC(via, pick);
    prog.addInstruction({ kind: 'move', moveType: 'MoveC', targetId: above.id, viaPose: Array.from(mul(transl(30, 30, 450), rotx(180 * DEG))) });
    prog.event('detach');
    prog.event('hide', box.id);
    prog.event('show', box.id);
    prog.addMoveJ([10, -100, 110, -100, -90, 0]);
    prog.addMoveJ(above);
    prog.addMoveL(mul(transl(0, 0, 450), rotx(180 * DEG)));
    prog.addInstruction({ kind: 'move', moveType: 'MoveJ', targetId: null, pose: Array.from(mul(transl(0, 0, 480), rotx(180 * DEG))) });
    prog.addInstruction({ kind: 'jointPath', joints: [[0, -100, 110, -100, -90, 0], [5, -100, 110, -100, -90, 0], [5, -95, 110, -100, -90, 0, 99]], dt: 0.1 });
    prog.setFrame(null);
    prog.setFrame(frame);
    prog.setTool(null);
    prog.setTool(tool);
    prog.waitDI('Sensor', true);
    prog.addInstruction({ kind: 'signal', signal: 'S', value: 'go', wait: false });
    prog.addInstruction({ kind: 'signal', signal: 'S', value: 'go', wait: true });
    prog.runInstruction('CustomCode', false);
    prog.showMessage('Hello');
    prog.comment('quiet');
    prog.addInstruction({ kind: 'mission_task', task: 'x', params: {} });
    prog.callProgram(sub);
    prog.addInstruction({ kind: 'loop', count: 2, bodyProgramId: sub.id });
    prog.addInstruction({ kind: 'loop', count: 2, bodyProgramId: null });
    prog.addInstruction({ kind: 'if', condition: 'c', thenProgramId: sub.id });
    prog.startThread(par);
    prog.addInstruction({ kind: 'wait', what: 'time', timeMs: 200 });
    prog.waitSignal('S', 'go', 300);
    prog.addInstruction({ kind: 'wait', what: 'move_done' });
    prog.navigateTo(1, 2);
    prog.addInstruction({ kind: 'mobile_follow', pathId: null });
    const disabled = prog.pause(5000);
    disabled.enabled = false;
    const sim = new ProgramSimulator(st);
    const messages: string[] = [];
    const ios: string[] = [];
    sim.events.on('message', (e) => messages.push(e.text));
    sim.events.on('io', (e) => ios.push(`${e.name}=${e.value}`));
    const res = sim.compile(prog);
    expect(res.problems.filter((p) => p.severity === 'error')).toEqual([]);
    expect(res.problems.map((p) => p.message)).toContain('Conditional: simulated "then" branch');
    expect(res.problems.map((p) => p.message)).toContain('Thread uses the same robot as the main program');
    expect(res.ok).toBe(true);
    expect(res.executed).toBeGreaterThan(30);
    expect(res.distance).toBeGreaterThan(500);
    expect(prog.lastResult).toBe(res);
    // the parallel thread (3 s pause) starts near the end and extends the total duration beyond the main timeline
    expect(sim.duration).toBeGreaterThan(res.duration);
    expect(sim.duration).toBeGreaterThanOrEqual(3);
    // steps are sorted by start time
    for (let i = 1; i < sim.steps.length; i++) expect(sim.steps[i].t0).toBeGreaterThanOrEqual(sim.steps[i - 1].t0);
    // end joints recorded for the robot
    expect(sim.endJoints(robot.id)!.length).toBe(6);
    expect(sim.endJoints('nobody')).toBeUndefined();
    // playback in small increments: discrete events fire, the box follows the gripper while attached
    const boxStart = getPos(box.poseAbs());
    const ins = prog.instructions();
    const upStep = sim.steps.find((s) => s.instruction === ins[7])!; // MoveL back to Above while holding the box
    const firstMove = sim.steps.find((s) => s.instruction === ins[2])!;
    let boxAtUp: number[] | null = null;
    sim.play();
    while (sim.tick(0.05)) { if (!boxAtUp && sim.time >= upStep.t1) boxAtUp = getPos(box.poseAbs()); }
    expect(sim.playing).toBe(false);
    expect(ios).toContain('Vac=true');
    expect(messages).toContain('Hello');
    expect(sim.io.Vac).toBe(true);
    expect(sim.signals.S).toBe('go');
    expect(box.visible).toBe(true);
    expect(boxAtUp![2] - boxStart[2]).toBeGreaterThan(80); // lifted ~100 mm with the tool
    // released at "Above" (100 mm higher than where it was picked)
    expect(distance(getPos(box.poseAbs()), boxStart)).toBeGreaterThan(50);
    expect(sim.attachments.size).toBe(0);
    expect(tool.closed).toBe(true);
    // seeking backwards re-runs the discrete events from scratch: the attachment exists again right after the close
    const closeStep = sim.steps.find((s) => s.instruction === ins[5])!;
    sim.seek(closeStep.t0 + 0.01);
    expect(sim.attachments.size).toBe(1);
    expect(tool.attached).toContain(box.id);
    sim.seek((firstMove.t0 + firstMove.t1) / 2);
    expect(robot.state.moving).toBe(true);
    expect(robot.state.progress).toBeGreaterThan(0);
    expect(robot.state.progress).toBeLessThan(1);
    // tick-based play
    sim.stop();
    expect(sim.time).toBe(0);
    sim.speedFactor = 4;
    sim.play();
    expect(sim.playing).toBe(true);
    let ticks = 0;
    while (sim.tick(0.5) && ticks < 1000) ticks++;
    expect(sim.playing).toBe(false);
    expect(ticks).toBeGreaterThan(0);
    expect(sim.time).toBe(sim.duration);
    expect(robot.state.moving).toBe(false);
    expect(sim.tick(1)).toBe(false);
    sim.play(); // restarts from 0 when at the end
    expect(sim.time).toBe(0);
    sim.pause();
    expect(sim.playing).toBe(false);
    // joint list & step lookup
    const rows = sim.jointsList(robot.id, 0.1);
    expect(rows.length).toBeGreaterThan(20);
    expect(rows[0].length).toBe(8);
    for (let i = 1; i < rows.length; i++) expect(rows[i][0]).toBeGreaterThanOrEqual(rows[i - 1][0]);
    expect(sim.jointsList('nobody')).toEqual([]);
    expect(sim.stepIndexAt(-1)).toBe(-1);
    expect(sim.stepIndexAt(sim.duration + 1)).toBe(sim.steps.length - 1);
    expect(estimateProgram(st, sub).ok).toBe(true);
  });
  it('reports every compile-time problem', () => {
    const { st, robot, home, above, pick } = cell();
    const orphan = st.addChild(new Program('Orphan'));
    orphan.addMoveJ(home);
    orphan.addInstruction({ kind: 'jointPath', joints: [[1]] });
    const r0 = new ProgramSimulator(st).compile(orphan);
    expect(r0.problems.map((p) => p.message)).toEqual(['Program has no robot', 'Program has no robot']);
    const prog = st.addChild(new Program('Bad'));
    prog.setRobot(robot);
    prog.addInstruction({ kind: 'move', moveType: 'MoveJ', targetId: 'missing' });
    const far = st.addChild(new Target('Far'));
    far.setPose(transl(9000, 0, 0));
    prog.addMoveJ(far);
    prog.addMoveL(far);
    prog.addInstruction({ kind: 'move', moveType: 'MoveJ', targetId: null });
    prog.addInstruction({ kind: 'move', moveType: 'MoveL', targetId: null });
    prog.addInstruction({ kind: 'move', moveType: 'MoveC', targetId: pick.id });
    prog.addInstruction({ kind: 'move', moveType: 'MoveJ', targetId: null, pose: Array.from(transl(9000, 0, 0)) });
    prog.addInstruction({ kind: 'jointPath', joints: [[1, 2]] });
    prog.addInstruction({ kind: 'call', programId: 'ghost' });
    prog.addInstruction({ kind: 'call', programId: null, programName: 'nothing' });
    prog.startThread({ id: 'ghost', name: 'Ghost' } as any);
    prog.callProgram(prog); // recursion
    prog.addMoveJ(above);
    const res = new ProgramSimulator(st).compile(prog);
    const msgs = res.problems.map((p) => p.message);
    expect(res.ok).toBe(false);
    expect(msgs).toEqual(expect.arrayContaining(['Target not found', 'Far unreachable', 'pose unreachable', 'pose could not be resolved', 'MoveC via point missing', 'Joint path is empty', 'Program ghost not found', 'Program nothing not found', 'Thread program Ghost not found', 'Recursive call to Bad']));
    expect(msgs.some((x) => /unreachable|Joint jump|singularity/.test(x))).toBe(true);
    // a valid move after the errors still gets a trajectory
    expect(res.executed).toBe(prog.instructions().length);
    const okStep = new ProgramSimulator(st);
    okStep.compile(prog);
    expect(okStep.steps.some((s) => s.trajectory?.ok)).toBe(true);
  });
  it('checks collisions along trajectories when enabled and ignores resting contacts', () => {
    const { st, robot, frame, home, above, pick } = cell();
    const pedestal = st.addChild(new SceneObject('Pedestal'));
    pedestal.geometry = [{ primitive: { kind: 'cylinder', radius: 150, length: 500 }, origin: Array.from(transl(0, 0, 250)) }];
    const wall = frame.addChild(new SceneObject('Wall'));
    wall.setPose(transl(0, 0, 380));
    wall.geometry = [{ primitive: { kind: 'box', size: [600, 600, 100] }, origin: Array.from(transl(0, 0, 50)) }];
    const prog = st.addChild(new Program('Crash'));
    prog.setRobot(robot);
    prog.addMoveJ(home);
    prog.addMoveJ(above);
    prog.addMoveL(pick); // TCP 400 above frame: the tool capsule (flange at 500) enters the wall slab at 380-480
    const sim = new ProgramSimulator(st);
    sim.collisionOptions = { enabled: true, sampleStep: 0.25 };
    const before = robot.joints();
    const res = sim.compile(prog);
    expect(robot.joints()).toEqual(before); // robot state restored
    expect(sim.collisions.length).toBeGreaterThanOrEqual(1);
    expect(res.problems.some((p) => p.severity === 'error' && /Collision: /.test(p.message))).toBe(true);
    expect(sim.collisions[0].pairs[0].a.item).toBeTruthy();
    // ignoring the pair silences it
    const pair = sim.collisions[0].pairs[0];
    sim.collisionOptions = { enabled: true, sampleStep: 0.25, ignore: [[pair.a.item.id, pair.b.item.id]] };
    const res2 = sim.compile(prog);
    expect(res2.problems.filter((p) => /Collision: .*Wall/.test(p.message)).length).toBeLessThanOrEqual(res.problems.filter((p) => /Collision/.test(p.message)).length);
  });
});

describe('spray simulation surfaces', () => {
  it('samples meshes, cylinders, spheres and bbox fallbacks; per-gun stats, clear and remove', () => {
    const st = new Station();
    const r = st.addChild(createRobotFromLibrary('UR10e'));
    const tool = r.addChild(new Tool('Gun'));
    tool.setPoseTool(transl(0, 0, 150));
    r.setTool(tool);
    const assets = new AssetStore();
    const tri = new Float32Array([-200, -200, 0, 200, -200, 0, 200, 200, 0, -200, -200, 0, 200, 200, 0, -200, 200, 0]);
    assets.registerRaw('panel.stl', 'stl', new Uint8Array(writeBinarySTL(tri)), 'panel');
    const meshPart = st.addChild(new SceneObject('Mesh panel'));
    meshPart.geometry = [{ mesh: 'panel.stl', origin: Array.from(transl(0, 0, 10)), scale: [1, 1, 1] }];
    meshPart.setPose(transl(700, 0, 0));
    const cyl = st.addChild(new SceneObject('Drum'));
    cyl.geometry = [{ primitive: { kind: 'cylinder', radius: 100, length: 300 } }];
    const sph = st.addChild(new SceneObject('Ball'));
    sph.geometry = [{ primitive: { kind: 'sphere', radius: 80 } }];
    const bboxOnly = st.addChild(new SceneObject('Blob'));
    bboxOnly.bbox = { min: [0, 0, 0], max: [100, 100, 50] };
    const nothing = st.addChild(new SceneObject('Nothing'));
    const spray = new SpraySimulator(st, assets);
    const g1 = spray.add(tool, meshPart, { angleDeg: 20, range: 600, sampleSpacing: 25 });
    const g2 = spray.add(tool, cyl, { sampleSpacing: 30 });
    const g3 = spray.add(tool, sph, { sampleSpacing: 30 });
    const g4 = spray.add(tool, bboxOnly, { sampleSpacing: 25 });
    const g5 = spray.add(tool, nothing);
    expect(g1.samples.length).toBeGreaterThan(200); // 400x400 panel at 25 mm
    expect(g1.samples.every((s) => Math.abs(s.p[2] - 10) < 1e-9)).toBe(true); // origin offset applied
    expect(g2.samples.length).toBeGreaterThan(100);
    expect(g2.samples.every((s) => Math.abs(Math.hypot(s.p[0], s.p[1]) - 100) < 1e-6)).toBe(true);
    expect(g3.samples.every((s) => Math.abs(Math.hypot(...s.p) - 80) < 1e-6)).toBe(true);
    expect(g4.samples.length).toBe(25);
    expect(g4.samples.every((s) => s.p[2] === 50)).toBe(true);
    expect(g5.samples.length).toBe(0);
    const res = r.solveIK(mul(transl(700, 0, 320), rotx(180 * DEG)));
    expect(res.ok).toBe(true);
    r.setJoints(res.joints);
    spray.setState('all', true);
    for (let i = 0; i < 10; i++) spray.step(0.1);
    const s1 = spray.stats(g1.id);
    expect(s1.coverage).toBeGreaterThan(3);
    expect(s1.max).toBeGreaterThan(0);
    expect(s1.samples).toBe(g1.samples.length);
    expect(s1.oversprayTime).toBe(0);
    // the other guns point nowhere near their parts: only overspray accumulates
    expect(spray.stats(g2.id).coverage).toBe(0);
    expect(spray.stats(g2.id).oversprayTime).toBeCloseTo(1, 9);
    expect(spray.stats(99).samples).toBe(0);
    expect(spray.stats().samples).toBe(g1.samples.length + g2.samples.length + g3.samples.length + g4.samples.length);
    spray.setState(g1.id, false);
    spray.step(0.1);
    expect(spray.stats(g1.id).max).toBe(s1.max);
    spray.clear(g1.id);
    expect(spray.stats(g1.id).coverage).toBe(0);
    expect(spray.stats(g2.id).oversprayTime).toBeGreaterThan(1);
    spray.remove(g2.id);
    expect(spray.guns.has(g2.id)).toBe(false);
    spray.remove();
    expect(spray.guns.size).toBe(0);
    // a gun whose tool or object vanished is skipped
    const gx = spray.add(tool, sph);
    sph.delete();
    spray.setState(gx.id, true);
    spray.step(0.1);
    expect(gx.totalTime).toBe(0);
  });
});

describe('machining resampling', () => {
  it('resamples cut curves at a fixed step', () => {
    const st = new Station();
    const r = st.addChild(createRobotFromLibrary('UR10e'));
    const tool = r.addChild(new Tool('Spindle'));
    tool.setPoseTool(transl(0, 0, 100));
    r.setTool(tool);
    const part = st.addChild(new SceneObject('Plate'));
    part.setPose(transl(500, 0, 300));
    (part as any).curves = [{ name: 'cut 1', points: [[0, 0, 0], [100, 0, 0], [100, 50, 0]], kind: 'cut', feed: 600 }];
    const res = generateMachining(st, r, part, { step: 20 });
    const coarse = generateMachining(st, r, part, {});
    expect(res.program.instructions().length).toBeGreaterThan(coarse.program.instructions().length);
    const moves = res.program.instructions().filter((i) => i.data.kind === 'move');
    expect(moves.length).toBeGreaterThanOrEqual(3 + 8); // approach/retract + 150 mm / 20 mm samples
  });
});

describe('calibration', () => {
  it('finds a TCP from flange poses touching one point (by point and by line)', () => {
    const tcp: [number, number, number] = [10, -5, 120];
    const point: [number, number, number] = [600, 100, 300];
    const flanges = [rotx(0), rotx(30 * DEG), roty(-40 * DEG), mul(rotz(50 * DEG), rotx(25 * DEG)), mul(roty(20 * DEG), rotz(-60 * DEG))].map((R) => {
      const tcpInBase = transformPoint(R, tcp);
      return mul(transl(point[0] - tcpInBase[0], point[1] - tcpInBase[1], point[2] - tcpInBase[2]), R);
    });
    const res = calibrateTcpByPoint(flanges);
    expect(distance(res.tcp, tcp)).toBeLessThan(1e-6);
    expect(distance(res.point, point)).toBeLessThan(1e-6);
    expect(res.maxError).toBeLessThan(1e-6);
    expect(res.errors.length).toBe(5);
    expect(() => calibrateTcpByPoint(flanges.slice(0, 2))).toThrow(/At least 3/);
    // noisy: still close
    const noisy = flanges.map((T, i) => mul(transl(0.05 * (i % 2 ? 1 : -1), 0, 0.03), T));
    const nr = calibrateTcpByPoint(noisy);
    expect(distance(nr.tcp, tcp)).toBeLessThan(0.5);
    expect(nr.meanError).toBeGreaterThan(0);
    // line poses: the same pin touched with the flange displaced along the tool Z axis
    const linePoses = [40, 80].map((d) => mul(flanges[0], transl(0, 0, -d)));
    const lr = calibrateTcpByLine(flanges, linePoses);
    expect(distance(lr.tcp, tcp)).toBeLessThan(1e-6);
    expect(lr.zAxis[2]).toBeCloseTo(1, 6);
    expect(Math.abs(lr.zAxis[0])).toBeLessThan(1e-6);
    // degenerate line poses (coincident with the TCP) fall back to +Z
    expect(calibrateTcpByLine(flanges, [flanges[0]]).zAxis).toEqual([0, 0, 1]);
  });
  it('ISO 9283 helpers: path accuracy, ballbar program and analysis, tracker noise', () => {
    const cmd = [[0, 0, 0], [100, 0, 0], [100, 100, 0]];
    const pa = pathAccuracy(cmd, [[50, 1, 0], [100, 50, 2], [120, 100, 0]]);
    expect(pa.max).toBeCloseTo(20, 9);
    expect(pa.mean).toBeCloseTo((1 + 2 + 20) / 3, 9);
    expect(pathAccuracy(cmd, [])).toEqual({ mean: 0, max: 0 });
    const st = new Station();
    const robot = st.addChild(createRobotFromLibrary('UR10e'));
    robot.setPose(transl(0, 0, 400));
    const { program, frame } = createBallbarProgram(st, robot, [600, 0, 300], 150, 12, 80);
    expect(frame.name).toBe('Ballbar R150');
    expect(getPos(frame.poseAbs())).toEqual([600, 0, 700]);
    expect(robot.activeFrame()).toBe(frame);
    const moves = program.instructions().filter((i) => i.data.kind === 'move');
    expect(moves.length).toBe(12 + 1 + 1); // 13 circle points + return to first
    expect((moves[0].data as any).moveType).toBe('MoveJ');
    expect((moves[1].data as any).moveType).toBe('MoveL');
    expect((moves.at(-1)!.data as any).moveType).toBe('MoveJ');
    expect(frame.children.length).toBe(13);
    expect(distance(getPos(frame.children[3].pose()), [150 * Math.cos(Math.PI / 2), 150 * Math.sin(Math.PI / 2), 0])).toBeLessThan(1e-9);
    const sim = new ProgramSimulator(st).compile(program);
    expect(sim.problems.filter((p) => p.severity === 'error')).toEqual([]);
    const measured: [number, number, number][] = [[151, 0, 0], [0, 149, 0], [-150, 0, 0]];
    const ba = ballbarAnalysis(measured, [0, 0, 0], 150);
    expect(ba.circularity).toBeCloseTo(2, 9);
    expect(ba.meanRadius).toBeCloseTo(150, 9);
    expect(ba.deviations).toEqual([1, -1, 0]);
    const seed = { s: 7 };
    const exact = getPos(robot.poseTCPAbs());
    const meas = simulateTrackerMeasure(robot, 0.1, seed);
    expect(distance(meas, exact)).toBeLessThan(0.3);
    expect(distance(meas, exact)).toBeGreaterThan(0);
    expect(seed.s).not.toBe(7);
    expect(distance(simulateTrackerMeasure(robot, 0), exact)).toBe(0);
  });
  it('frame calibration with P1 on the X axis projects the origin', () => {
    const p1 = [100, 0, 0] as any, p2 = [300, 0, 0] as any, p3 = [200, 50, 0] as any;
    const res = calibrateFrame([p1, p2, p3], CALIBRATE_FRAME_3P_P1_ON_X);
    expect(getPos(res.pose)).toEqual([200, 0, 0]);
    expect(res.pose[0]).toBeCloseTo(1, 9); // X axis along p1->p2
    expect(res.pose[5]).toBeCloseTo(1, 9); // Y towards p3
    const origin = calibrateFrame([p1, p2, p3], CALIBRATE_FRAME_3P_P1_ORIGIN);
    expect(getPos(origin.pose)).toEqual([100, 0, 0]);
    expect(() => calibrateFrame([p1, p2], CALIBRATE_FRAME_3P_P1_ON_X)).toThrow(/3 points/);
  });
});

describe('clipboard and robot library', () => {
  it('copies and pastes subtrees with unique names', () => {
    const st = new Station();
    const f = st.addChild(new Frame('Fixture'));
    f.setPose(transl(1, 2, 3));
    const t = f.addChild(new Target('T'));
    t.setJoints([1, 2, 3]);
    const cb = new Clipboard();
    expect(cb.hasContent()).toBe(false);
    expect(cb.paste(st, st)).toBeNull();
    cb.copy(f);
    expect(cb.hasContent()).toBe(true);
    const folder = st.addChild(new Folder('Copies'));
    const c1 = cb.paste(folder, st)!;
    expect(c1.name).toBe('Fixture 2');
    expect(c1.parent).toBe(folder);
    expect(c1.id).not.toBe(f.id);
    expect(getPos(c1.pose())).toEqual([1, 2, 3]);
    expect((c1.children[0] as Target).joints).toEqual([1, 2, 3]);
    expect(c1.children[0].id).not.toBe(t.id);
    const c2 = cb.paste(folder, st)!;
    expect(c2.name).toBe('Fixture 3');
    expect(clipboard).toBeInstanceOf(Clipboard);
  });
  it('builds every library robot with consistent joint limits and reach', () => {
    for (const e of ROBOT_LIBRARY) {
      const r = createRobotFromLibrary(e.id);
      expect(r.dof, e.id).toBe(e.dof);
      expect(r.params.libraryId).toBe(e.id);
      expect(r.brand).toBe(e.brand);
      expect(r.postProcessor).toBe(e.postProcessor);
      expect(r.jointsValid(r.joints()), e.id).toBe(true);
      for (const j of r.chain.joints) if (j.type !== 'fixed') expect(j.upper, `${e.id} ${j.name}`).toBeGreaterThan(j.lower);
    }
    expect(() => createRobotFromLibrary('nope')).toThrow(/Unknown robot/);
    const named = createRobotFromLibrary('KUKA_KR16_R2010', 'My KR16');
    expect(named.name).toBe('My KR16');
    expect(named.model).toMatch(/KR ?16/i);
    // KUKA limits are shifted by +90 on A2 (vendor convention) and vendor limits apply
    const kr = createRobotFromLibrary('KUKA_KR210_R2700');
    expect(kr.chain.joints[1].lower).toBeLessThan(0);
    expect(kr.chain.joints[0].lower).toBeLessThanOrEqual(-170);
  });
});

export { invert, multiply, ItemType };
