// T3 (sim): station runtime contract (I1/I2/I5/I9): concurrency, queueing, signals, threads, reset, play-to-error,
// script runner, mobile planner hook, id index and separate colliders.
import { describe, it, expect, afterEach } from 'vitest';
import { Station, Frame, Target, Tool, SceneObject, ItemType, Item } from '../src/core/items/item';
import { Program } from '../src/core/items/program';
import { Robot } from '../src/core/items/robot';
import { createRobotFromLibrary } from '../src/core/items/library';
import { StationRuntime, registerScriptRunner, evalCondition, signalEquals, ScriptContext } from '../src/core/motion/runtime';
import { ProgramSimulator, setMobileMotionPlanner } from '../src/core/motion/simulator';
import { MobileRobot } from '../src/mobile/items';
import { instantiateSubtree } from '../src/core/items/clone';
import { checkRobotCollisions } from '../src/core/collision/collision';
import { transl, mul, rotx, DEG, multiply, invert, getPos, distance } from '../src/core/math/pose';

function cell() {
  const st = new Station('rt');
  const a = st.addChild(createRobotFromLibrary('UR5e', 'A'));
  const b = st.addChild(createRobotFromLibrary('UR5e', 'B'));
  b.setPose(transl(2000, 0, 0));
  for (const r of [a, b]) r.setJoints([0, -90, 90, -90, -90, 0]);
  const jt = (name: string, q: number[]) => { const t = st.addChild(new Target(name)); t.setJoints(q); t.setAsJointTarget(); return t; };
  return { st, a, b, jt };
}
const prog = (st: Station, name: string, robot: Robot | null) => { const p = st.addChild(new Program(name)); if (robot) p.setRobot(robot); return p; };

describe('StationRuntime', () => {
  afterEach(() => setMobileMotionPlanner(null));

  it('runs programs of different robots concurrently on one clock and queues a second program of a busy robot', () => {
    const { st, a, b, jt } = cell();
    const ta = jt('ta', [90, -90, 90, -90, -90, 0]), tb = jt('tb', [-90, -90, 90, -90, -90, 0]), ta2 = jt('ta2', [0, -90, 90, -90, -90, 0]);
    const pa = prog(st, 'PA', a); pa.setSpeed(undefined, 45); pa.addMoveJ(ta);
    const pb = prog(st, 'PB', b); pb.setSpeed(undefined, 45); pb.addMoveJ(tb);
    const pa2 = prog(st, 'PA2', a); pa2.addMoveJ(ta2);
    const rt = new StationRuntime(st);
    const ha = rt.start(pa), hb = rt.start(pb), ha2 = rt.start(pa2);
    expect([ha.status, hb.status, ha2.status]).toEqual(['running', 'running', 'queued']);
    rt.runFor(1);
    // both arms moved during the same second (90 deg at 45 deg/s takes > 2 s)
    expect(a.joints()[0]).toBeGreaterThan(5); expect(a.joints()[0]).toBeLessThan(85);
    expect(b.joints()[0]).toBeLessThan(-5); expect(b.joints()[0]).toBeGreaterThan(-85);
    rt.runFor(10);
    expect([ha.status, hb.status, ha2.status]).toEqual(['done', 'done', 'done']);
    // the queued program started when PA ended (same robot), PB ran in parallel with PA
    expect(ha2.startTime!).toBeCloseTo(ha.endTime!, 6);
    expect(hb.startTime!).toBe(ha.startTime!);
    expect(a.joints()[0]).toBeCloseTo(0, 6);
    expect(b.joints()[0]).toBeCloseTo(-90, 6);
  });

  it('waitDI waits at run time for a DO set by a thread on another robot (S7), both live and in the compiled timeline', () => {
    const { st, a, b, jt } = cell();
    const th = prog(st, 'Thread', b); th.addMoveJ(jt('far', [90, -90, 90, -90, -90, 0])); th.pause(5000); th.setDO('Ready', 1);
    const main = prog(st, 'Main', a); main.startThread(th); main.waitDI('Ready', 1); main.addMoveJ(jt('go', [45, -90, 90, -90, -90, 0]));
    const sim = new ProgramSimulator(st);
    const res = sim.compile(main);
    expect(res.problems.filter((p) => p.severity === 'error')).toEqual([]);
    const doStep = sim.steps.find((s) => s.instruction.data.kind === 'io' && !(s.instruction.data as any).wait)!;
    const moveStep = sim.steps.find((s) => s.instruction.parent === main && s.instruction.data.kind === 'move')!;
    expect(doStep.t0).toBeGreaterThan(5);
    expect(moveStep.t0).toBeCloseTo(doStep.t0, 9);
    // live
    const rt = new StationRuntime(st);
    const h = rt.start(main);
    rt.runFor(doStep.t0 - 0.2);
    expect(a.joints()[0]).toBeCloseTo(0, 9); // still waiting
    expect(rt.signals.get('Ready')).toBeUndefined();
    rt.runFor(moveStep.t1 - moveStep.t0 + 1);
    expect(rt.signals.get('Ready')).toBe(1);
    expect(h.status).toBe('done');
    expect(a.joints()[0]).toBeCloseTo(45, 6);
  });

  it('an external signal (control model / PLC) releases a waiting program; IF picks its branch at run time', () => {
    const { st, a, jt } = cell();
    const left = prog(st, 'Left', a); left.addMoveJ(jt('L', [60, -90, 90, -90, -90, 0]));
    const right = prog(st, 'Right', a); right.addMoveJ(jt('R', [-60, -90, 90, -90, -90, 0]));
    const main = prog(st, 'Main', a);
    main.waitSignal('start', true);
    main.addInstruction({ kind: 'if', condition: 'side == "left" && start', thenProgramId: left.id, elseProgramId: right.id });
    const rt = new StationRuntime(st);
    const h = rt.start(main);
    rt.runFor(3);
    expect(h.status).toBe('running');
    expect(a.joints()[0]).toBe(0);
    rt.signals.set('side', 'right');
    rt.signals.set('start', true);
    rt.runFor(5);
    expect(h.status).toBe('done');
    expect(a.joints()[0]).toBeCloseTo(-60, 6);
    rt.reset();
    expect(a.joints()[0]).toBeCloseTo(0, 9);
    expect(rt.signals.get('start')).toBeUndefined();
    rt.signals.set('side', 'left'); rt.signals.set('start', 1);
    rt.start(main); rt.runFor(5);
    expect(a.joints()[0]).toBeCloseTo(60, 6);
  });

  it('plays up to the first error and stops there; other runs continue (S12)', () => {
    const { st, a, b, jt } = cell();
    const frame = st.addChild(new Frame('W')); frame.setPose(transl(400, -200, 0));
    const ok = jt('ok', [30, -90, 90, -90, -90, 0]);
    const far = frame.addChild(new Target('TooFar')); far.setPose(mul(transl(3000, 0, 300), rotx(180 * DEG)));
    const p = prog(st, 'P', a); p.addMoveJ(ok); p.addMoveL(far); p.addMoveJ(jt('never', [-30, -90, 90, -90, -90, 0]));
    const q = prog(st, 'Q', b); q.addMoveJ(jt('qb', [40, -90, 90, -90, -90, 0])); q.pause(3000);
    const rt = new StationRuntime(st);
    const hp = rt.start(p), hq = rt.start(q);
    rt.runFor(20);
    expect(hp.status).toBe('error');
    expect(hp.error!.instructionIndex).toBe(1);
    expect(hp.error!.message).toMatch(/unreachable|singularity/);
    expect(a.joints()[0]).toBeCloseTo(30, 6); // the first move was played, the third never started
    expect(hq.status).toBe('done');
    expect(b.joints()[0]).toBeCloseTo(40, 6);
    // the program simulator lists the error and plays up to it (compiled from the home state)
    rt.reset();
    expect(a.joints()[0]).toBeCloseTo(0, 9);
    const sim = new ProgramSimulator(st);
    const res = sim.compile(p);
    expect(res.ok).toBe(false);
    sim.play();
    expect(sim.playing).toBe(true);
    while (sim.tick(0.05)) { /* play */ }
    expect(sim.time).toBeCloseTo(sim.errorTime!, 9);
    expect(sim.errorTime!).toBeLessThan(sim.duration);
    expect(a.joints()[0]).toBeCloseTo(30, 3);
  });

  it('Run instructions start threads on other robots that run concurrently; stop() stops the thread tree', () => {
    const { st, a, b, jt } = cell();
    const side = prog(st, 'Side', b); side.addMoveJ(jt('s1', [-90, -90, 90, -90, -90, 0])); side.addMoveJ(jt('s2', [90, -90, 90, -90, -90, 0]));
    const main = prog(st, 'Main', a); main.startThread(side); main.addMoveJ(jt('m', [90, -90, 90, -90, -90, 0]));
    const sim = new ProgramSimulator(st);
    sim.compile(main);
    const mainMove = sim.steps.find((s) => s.instruction.parent === main && s.trajectory)!;
    const sideMoves = sim.steps.filter((s) => s.instruction.parent === side && s.trajectory);
    expect(sideMoves[0].t0).toBe(mainMove.t0); // both start together
    expect(sim.duration).toBeCloseTo(Math.max(mainMove.t1, sideMoves[1].t1), 9);
    const rt = new StationRuntime(st);
    const h = rt.start(main);
    rt.runFor(0.5);
    const child = rt.runs.find((r) => r.program === side)!;
    expect(child.status).toBe('running');
    expect(child.parentId).toBe(h.id);
    rt.stop(h.id);
    expect(h.status).toBe('stopped');
    expect(child.status).toBe('stopped');
    const qb = b.joints()[0];
    rt.runFor(2);
    expect(b.joints()[0]).toBe(qb); // nothing moves after stop
  });

  it('attaches the object closest to the TCP within reach (by geometry, not origin), detaches to its frame, reset restores', () => {
    const { st, a } = cell();
    const tool = a.addChild(new Tool('Gripper')); tool.setPoseTool(transl(0, 0, 100)); a.setTool(tool);
    const table = st.addChild(new Frame('Table'));
    const tcp = multiply(a.poseAbs(), a.solveFK(a.joints(), tool.poseTool()));
    // a long bar whose origin is 250 mm from the TCP but whose end touches it, and a small cube 120 mm away
    const bar = table.addChild(new SceneObject('Bar'));
    bar.geometry = [{ primitive: { kind: 'box', size: [500, 40, 40] } }];
    bar.setPose(multiply(transl(tcp[12] - 250 + 5, tcp[13], tcp[14]), rotx(0)));
    const cube = table.addChild(new SceneObject('Cube'));
    cube.geometry = [{ primitive: { kind: 'box', size: [20, 20, 20] } }];
    cube.setPose(transl(tcp[12], tcp[13] + 120, tcp[14]));
    const up = st.addChild(new Target('Up')); up.setJoints([20, -80, 80, -90, -90, 0]); up.setAsJointTarget();
    const p = prog(st, 'Pick', a); p.event('attach'); p.addMoveJ(up); p.event('detach');
    const barStart = bar.poseAbs();
    const rt = new StationRuntime(st);
    rt.captureHome();
    rt.start(p);
    rt.runFor(0.01);
    expect(bar.parent).toBe(tool);
    expect(tool.attached).toEqual([bar.id]);
    expect(cube.parent).toBe(table);
    rt.runFor(10);
    expect(bar.parent).toBe(table); // detached back to the frame it came from
    expect(tool.attached).toEqual([]);
    expect(distance(getPos(bar.poseAbs()), getPos(barStart))).toBeGreaterThan(50); // carried with the arm
    rt.reset();
    expect(bar.parent).toBe(table);
    expect(distance(getPos(bar.poseAbs()), getPos(barStart))).toBeLessThan(1e-9);
    expect(a.joints()).toEqual([0, -90, 90, -90, -90, 0]);
  });

  it('RoboDK "Replace objects" restores the stored placements; two Attach events carry a hidden raw part and its visible twin', () => {
    const { st, a } = cell();
    const tool = a.addChild(new Tool('Gripper')); tool.setPoseTool(transl(0, 0, 100)); a.setTool(tool);
    const input = st.addChild(new Frame('Input')), output = st.addChild(new Frame('Output'));
    output.setPose(transl(0, -1500, 0));
    const tcp = multiply(a.poseAbs(), a.solveFK(a.joints(), tool.poseTool()));
    const raw = output.addChild(new SceneObject('Part 1')), done = output.addChild(new SceneObject('Part Done 1')), next = input.addChild(new SceneObject('Part 2'));
    for (const o of [raw, done, next]) o.geometry = [{ primitive: { kind: 'box', size: [40, 40, 40] } }];
    raw.setVisible(false); // the saved station shows the finished twin at the output (after a previous simulation)
    next.setPose(transl(tcp[12], tcp[13] + 60, tcp[14]));
    const at = Array.from(transl(tcp[12], tcp[13], tcp[14]));
    const p = prog(st, 'Cycle', a);
    p.addInstruction({ kind: 'event', action: 'custom', objectId: null, name: 'Replace objects', restore: [{ id: raw.id, parentId: input.id, pose: at }, { id: done.id, parentId: input.id, pose: at }] } as any);
    p.event('attach'); p.event('attach');
    const rt = new StationRuntime(st);
    rt.start(p); rt.runFor(0.01);
    expect(raw.parent).toBe(tool); expect(done.parent).toBe(tool); // both twins, not the visible neighbour 60 mm away
    expect(next.parent).toBe(input);
    expect(distance(getPos(raw.poseAbs()), getPos(tcp))).toBeLessThan(1e-6);
  });

  it('calls a station Python program through the registered script runner (I5) and waits for it', async () => {
    const { st, a, jt } = cell();
    const py = new Item(ItemType.PROGRAM_PYTHON, 'SetRPM'); st.addChild(py);
    py.params.pythonCode = 'print(1)';
    const p = prog(st, 'Main', a); p.runInstruction('SetRPM(70, "fast")', true); p.addMoveJ(jt('t', [10, -90, 90, -90, -90, 0]));
    const calls: ScriptContext[] = [];
    let release!: () => void;
    const rt = new StationRuntime(st);
    const off = rt.registerScriptRunner(async (ctx) => { calls.push(ctx); await new Promise<void>((r) => (release = r)); });
    const h = rt.start(p);
    rt.runFor(2);
    expect(calls.map((c) => [c.name, c.args, c.program])).toEqual([['SetRPM', ['70', 'fast'], py]]);
    expect(a.joints()[0]).toBe(0); // blocked on the script
    release();
    await new Promise((r) => setTimeout(r, 0));
    rt.runFor(3);
    expect(h.status).toBe('done');
    expect(a.joints()[0]).toBeCloseTo(10, 6);
    off();
    // a failing script stops the program at the call (error with its index)
    const offG = registerScriptRunner(async () => { throw new Error('spindle fault'); });
    const h2 = rt.start(p);
    await rt.runUntilDone(5);
    expect(h2.status).toBe('error');
    expect(h2.error!.message).toMatch(/spindle fault/);
    expect(h2.error!.instructionIndex).toBe(0);
    offG();
  });

  it('a station Python program started as a thread runs through the script runner in live playback (not when compiling)', async () => {
    const { st, a, jt } = cell();
    const py = new Item(ItemType.PROGRAM_PYTHON, 'Conveyor'); st.addChild(py);
    const main = prog(st, 'Main', a); main.addInstruction({ kind: 'thread', programId: py.id, programName: 'Conveyor' }); main.waitSignal('PartReady', 1); main.addMoveJ(jt('t', [20, -90, 90, -90, -90, 0]));
    const sim = new ProgramSimulator(st);
    let ran = 0;
    const off = registerScriptRunner(async (ctx) => { ran++; ctx.runtime.signals.set('PartReady', 1); });
    const res = sim.compile(main);
    expect(ran).toBe(0); // compiling never executes scripts
    expect(res.problems.filter((p) => p.severity === 'error')).toEqual([]);
    expect(res.problems.some((p) => /Conveyor is a Python program/.test(p.message))).toBe(true);
    const rt = new StationRuntime(st);
    const h = rt.start(main);
    await rt.runUntilDone(10);
    off();
    expect(ran).toBe(1);
    expect(h.status).toBe('done');
    expect(rt.runs.find((r) => r.program === py)?.parentId).toBe(h.id);
    expect(a.joints()[0]).toBeCloseTo(20, 6);
  });

  it('script runners are chained: a runner resolving false passes the call to the next one', async () => {
    const { st, a, jt } = cell();
    const p = prog(st, 'Main', a); p.runInstruction('VisionDetect(cam)', true); p.runInstruction('SetRPM(70)', true); p.addMoveJ(jt('t', [10, -90, 90, -90, -90, 0]));
    const seen: string[] = [];
    const rt = new StationRuntime(st);
    const offA = rt.registerScriptRunner(async (ctx) => { if (ctx.name !== 'VisionDetect') return false; seen.push(`vision:${ctx.args}`); });
    const offB = registerScriptRunner(async (ctx) => { seen.push(`python:${ctx.name}`); });
    const h = rt.start(p);
    await rt.runUntilDone(10);
    offA(); offB();
    expect(h.status).toBe('done');
    expect(seen).toEqual(['vision:cam', 'python:SetRPM']);
  });

  it('StartOptions.fromIndex starts at an instruction with the tool / speed set before it ("Run from here")', () => {
    const { st, a, jt } = cell();
    const t300 = a.addChild(new Tool('T300')); t300.setPoseTool(transl(0, 0, 300));
    const p = prog(st, 'P', a); p.addMoveJ(jt('first', [60, -90, 90, -90, -90, 0])); p.setTool(t300); p.setSpeed(undefined, 30); p.addMoveJ(jt('second', [30, -90, 90, -90, -90, 0]));
    const rt = new StationRuntime(st);
    const h = rt.start(p, { fromIndex: 3 });
    rt.runFor(0.5);
    expect(a.activeTool()).toBe(t300);
    expect(a.joints()[0]).toBeGreaterThan(0); // moving from 0 towards 30 (the first move was skipped)
    rt.runFor(5);
    expect(h.status).toBe('done');
    expect(h.endTime! - h.startTime!).toBeGreaterThan(1); // 30 deg at 30 deg/s
    expect(a.joints()[0]).toBeCloseTo(30, 6);
  });

  it('reset() restores robot IO set by a run', () => {
    const { st, a } = cell();
    a.state.io = { Gripper: 0 };
    const p = prog(st, 'P', a); p.setDO('Gripper', 1);
    const rt = new StationRuntime(st);
    rt.start(p); rt.runFor(0.1);
    expect(a.state.io.Gripper).toBe(1);
    expect(rt.signals.get('Gripper')).toBe(1);
    rt.reset();
    expect(a.state.io.Gripper).toBe(0);
    expect(rt.signals.get('Gripper')).toBeUndefined();
  });

  it('a called program inherits the caller tool and speed (robot state persists like RoboDK: S4, S4b)', () => {
    const st = new Station();
    const robot = st.addChild(createRobotFromLibrary('UR5e', 'R'));
    robot.setJoints([0, -90, 90, -90, -90, 0]);
    const t150 = robot.addChild(new Tool('T150')); t150.setPoseTool(transl(0, 0, 150)); robot.setTool(t150);
    const t300 = robot.addChild(new Tool('T300')); t300.setPoseTool(transl(0, 0, 300));
    // well inside the reach (the old 400..600 mm / z 300 line with a 300 mm tool crossed the elbow singularity)
    const frame = st.addChild(new Frame('W')); frame.setPose(transl(300, -200, 0));
    const t = frame.addChild(new Target('T')); t.setPose(mul(transl(0, 0, 200), rotx(180 * DEG)));
    const t2 = frame.addChild(new Target('T2')); t2.setPose(mul(transl(200, 0, 200), rotx(180 * DEG)));
    const sub = st.addChild(new Program('Sub')); sub.setRobot(robot); sub.addMoveL(t2);
    const main = st.addChild(new Program('Main')); main.setRobot(robot); main.setTool(t300); main.addMoveJ(t); main.setSpeed(50); main.callProgram(sub);
    const sim = new ProgramSimulator(st);
    const res = sim.compile(main);
    expect(res.problems.filter((p) => p.severity === 'error')).toEqual([]);
    sim.runToEnd();
    expect(robot.activeTool()).toBe(t300);
    expect(distance(getPos(multiply(robot.poseAbs(), robot.solveFK(robot.joints(), t300.poseTool()))), getPos(t2.poseAbs()))).toBeLessThan(0.1);
    const subMove = sim.steps.find((s) => s.instruction.parent === sub)!;
    expect(subMove.t1 - subMove.t0).toBeGreaterThan(200 / 50); // 200 mm at the inherited 50 mm/s
    expect(subMove.t1 - subMove.t0).toBeLessThan(200 / 50 + 0.2);
  });

  it('mobile Navigate/Follow use the registered planner (I2) and default to a straight line at the instruction speed', () => {
    const st = new Station();
    const amr = st.addChild(new MobileRobot('AMR'));
    amr.setPose2D(0, 0, 0);
    const p = prog(st, 'Drive', null); p.setRobot(amr); p.navigateTo(3000, 4000, undefined, { speed: 500 });
    const sim = new ProgramSimulator(st);
    const res = sim.compile(p);
    expect(res.problems).toEqual([]);
    // 5 m at 500 mm/s plus turning towards the goal (53 deg at the default 90 deg/s)
    expect(res.duration).toBeCloseTo(5000 / 500 + 53.13 / 90, 2);
    sim.runToEnd();
    expect(amr.state.x).toBeCloseTo(3000, 6); expect(amr.state.y).toBeCloseTo(4000, 6);
    amr.setPose2D(0, 0, 0); // a compile starts from the current state: back to the origin
    const seen: string[] = [];
    setMobileMotionPlanner((station, mobile, ins, start) => { seen.push(`${mobile.name}:${ins.kind}:${start.x}`); return { ok: true, duration: 7, sample: (t) => ({ x: start.x + t, y: 0, heading: 0 }) }; });
    const sim2 = new ProgramSimulator(st);
    expect(sim2.compile(p).duration).toBe(7);
    expect(seen).toEqual(['AMR:mobile_move:0']);
    sim2.runToEnd();
    expect(amr.state.x).toBeCloseTo(7, 9);
  });

  it('evaluates conditions and loose signal equality like RoboDK IO', () => {
    const sig: Record<string, unknown> = { 'DI[3]': 1, Busy: false, S: 'go', n: 4 };
    const get = (k: string) => sig[k];
    expect(evalCondition('DI[3] == 1 && !Busy', get)).toBe(true);
    expect(evalCondition('S == "go" and n > 3', get)).toBe(true);
    expect(evalCondition('n >= 5 or unknown', get)).toBe(false);
    expect(evalCondition('(n - 1) * 2 == 6', get)).toBe(true);
    expect(signalEquals(true, 1)).toBe(true);
    expect(signalEquals('1', 1)).toBe(true);
    expect(signalEquals('on', true)).toBe(true);
    expect(signalEquals(undefined, 0)).toBe(true);
    expect(signalEquals('a', 'b')).toBe(false);
  });
});

describe('Station id index and separate colliders (I9)', () => {
  it('findById follows add, remove, move, clone and deserialize', () => {
    const st = new Station();
    const f = st.addChild(new Frame('F'));
    const t = new Target('T');
    const sub = new Frame('Sub'); sub.addChild(t); // built off-tree, then attached
    expect(st.findById(t.id)).toBeNull();
    f.addChild(sub);
    expect(st.findById(t.id)).toBe(t);
    const g = st.addChild(new Frame('G'));
    g.addChild(sub);
    expect(st.findById(t.id)).toBe(t);
    sub.delete();
    expect(st.findById(t.id)).toBeNull();
    expect(st.findById(sub.id)).toBeNull();
    f.addChild(sub);
    const copy = instantiateSubtree(sub.serialize(), g, st);
    expect(st.findById(copy.id)).toBe(copy);
    expect(st.findById(copy.children[0].id)).toBe(copy.children[0]);
    const back = Station.deserialize(st.serialize());
    expect(back.findById(t.id)?.name).toBe('T');
    expect(back.findById(t.id)).not.toBe(t);
    // O(1): 20k lookups in a 5k-item tree take a few ms (the old recursive walk took seconds)
    for (let i = 0; i < 5000; i++) f.addChild(new Target(`t${i}`));
    const ids = f.children.map((c) => c.id);
    // timed without expect() calls (their cost under a loaded machine made the bound flaky); the old O(n) walk needs
    // ~1e8 node visits here (seconds), the index a few ms
    let misses = 0;
    const t0 = performance.now();
    for (let k = 0; k < 20000; k++) if (!st.findById(ids[k % ids.length])) misses++;
    const ms = performance.now() - t0;
    expect(misses).toBe(0);
    expect(ms).toBeLessThan(2500);
  });

  it('SceneObject.colliders replaces the visual geometry for collision (Webots boundingObject)', () => {
    const st = new Station();
    const r = st.addChild(createRobotFromLibrary('UR10e'));
    const elbow = r.fk().linkPoses[3];
    const o = st.addChild(new SceneObject('Shell'));
    o.setPose(transl(elbow[12], elbow[13], elbow[14]));
    o.geometry = [{ primitive: { kind: 'box', size: [300, 300, 300] } }]; // visual crosses the arm
    expect(checkRobotCollisions(st, r).some((p) => p.a.item === o || p.b.item === o)).toBe(true);
    o.colliders = [{ primitive: { kind: 'sphere', radius: 10 }, origin: Array.from(transl(0, 0, 2000)) }]; // collider far away
    expect(checkRobotCollisions(st, r).some((p) => p.a.item === o || p.b.item === o)).toBe(false);
    o.colliders = [];
    expect(checkRobotCollisions(st, r).some((p) => p.a.item === o || p.b.item === o)).toBe(false);
    const back = Station.deserialize(st.serialize());
    expect((back.find('Shell') as SceneObject).colliders).toEqual([]);
    void invert;
  });
});
