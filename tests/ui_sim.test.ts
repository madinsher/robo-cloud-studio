// @vitest-environment jsdom
/**
 * T11: deterministic simulation in the app on top of the StationRuntime. Repeatable runs, pause / resume without
 * restart, selection and previews that never disturb playback, concurrent programs on different robots, Reset
 * ("Replace objects") restoring poses, deterministic seeking, and play up to the first error with its marker.
 */
import { describe, it, expect, beforeAll } from 'vitest';
import { installDomStubs, makeApp } from './ui_harness';
import { App, ItemType, Program, Robot, SceneObject, Target } from '../src/app';
import { transl } from '../src/core/math/pose';
import { ProgramEditor } from '../src/ui/program-editor';

beforeAll(() => installDomStubs());
const tick = (app: App, s: number, dt = 1 / 30) => { for (let t = 0; t < s - 1e-9; t += dt) (app as any).tick(dt); };
const posOf = (o: { poseAbs(): number[] | Float64Array }) => Array.from(o.poseAbs()).slice(12, 15);
const boxes = (app: App) => ['Box 1', 'Box 2', 'Box 3'].map((n) => app.station.find(n) as SceneObject);
const dist = (a: number[], b: number[]) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);

describe('station simulation in the app', () => {
  it('Run → End → Run → End is repeatable: the boxes end where the first run left them (no flying parts)', () => {
    const app = makeApp('pickplace');
    const start = boxes(app).map(posOf);
    app.runProgram(); app.runToEnd();
    expect(app.runs.at(-1)!.status).toBe('done');
    const first = boxes(app).map(posOf);
    first.forEach((p, i) => expect(dist(p, start[i])).toBeGreaterThan(100)); // every box was carried away
    app.runProgram(); app.runToEnd();
    const second = boxes(app).map(posOf);
    second.forEach((p, i) => expect(dist(p, first[i])).toBeLessThan(1));
  });

  it('Pause freezes the clock and the robot; Play resumes from the paused time with the same duration', () => {
    const app = makeApp('pickplace');
    const robot = app.activeRobot!;
    app.runProgram(); tick(app, 2.5);
    const d0 = app.simDuration, t0 = app.simTime, q0 = robot.joints();
    expect(t0).toBeGreaterThan(2.4);
    app.pauseProgram(); tick(app, 1);
    expect(app.simTime).toBeCloseTo(t0, 9); expect(robot.joints()).toEqual(q0);
    app.runProgram(); tick(app, 0.5);
    expect(app.simTime).toBeGreaterThan(t0 + 0.45); expect(app.simTime).toBeLessThan(t0 + 0.55);
    expect(app.simDuration).toBeCloseTo(d0, 6); expect(app.runs.length).toBe(1);
  });

  it('selecting another program, previewing and validating while playing never stops or moves the playing robot', () => {
    const app = makeApp('welding');
    const progs = app.station.itemsOfType<Program>(ItemType.PROGRAM);
    const left = progs.find((p) => /left/i.test(p.name))!, right = progs.find((p) => /right/i.test(p.name))!;
    app.select(left); app.runProgram(left); tick(app, 1);
    const rl = left.robot() as Robot; const q = rl.joints(); const t = app.simTime;
    app.select(right); app.previewProgram(); app.validateProgram(right);
    expect(app.isPlaying()).toBe(true); expect(app.simTime).toBe(t); expect(rl.joints()).toEqual(q);
    tick(app, 0.5); expect(rl.joints()).not.toEqual(q);
  });

  it('validating the playing program while it holds a part leaves the part attached where it was', () => {
    const app = makeApp('pickplace');
    const tool = app.activeRobot!.activeTool()!;
    app.runProgram();
    for (let i = 0; i < 600 && !tool.attached.length; i++) tick(app, 0.05);
    expect(tool.attached.length).toBe(1);
    const held = app.station.findById(tool.attached[0]) as SceneObject;
    const pose = Array.from(held.poseAbs()), parent = held.parent, q = app.activeRobot!.joints(), t = app.simTime;
    const r = app.validateProgram(app.activeProgram)!;
    expect(r.duration).toBeGreaterThan(5);
    expect(tool.attached).toEqual([held.id]); expect(held.parent).toBe(parent);
    expect(Array.from(held.poseAbs())).toEqual(pose); expect(app.activeRobot!.joints()).toEqual(q); expect(app.simTime).toBe(t);
    tick(app, 0.5);
    expect(dist(posOf(held), pose.slice(12, 15))).toBeGreaterThan(1); // still carried by the moving tool
  });

  it('programs on different robots run at the same time on the shared clock', () => {
    const app = makeApp('welding');
    const progs = app.station.itemsOfType<Program>(ItemType.PROGRAM);
    const left = progs.find((p) => /left/i.test(p.name))!, right = progs.find((p) => /right/i.test(p.name))!;
    const rl = left.robot() as Robot, rr = right.robot() as Robot;
    expect(rl).not.toBe(rr);
    app.runProgram(left); tick(app, 0.5); app.runProgram(right);
    const ql = rl.joints(), qr = rr.joints();
    tick(app, 1.5);
    expect(app.activeRuns().length).toBe(2);
    expect(rl.joints()).not.toEqual(ql); expect(rr.joints()).not.toEqual(qr);
  });

  it('Reset restores the home state exactly (object poses, robot joints) and seeking is deterministic', () => {
    const app = makeApp('pickplace');
    const robot = app.activeRobot!;
    const home = { q: robot.joints(), boxes: boxes(app).map((b) => Array.from(b.poseAbs())) };
    app.runProgram(); tick(app, 6);
    app.pauseProgram();
    app.seekProgram(5); const at5 = { q: robot.joints(), b: boxes(app).map(posOf) };
    app.seekProgram(2); expect(robot.joints()).not.toEqual(at5.q);
    app.seekProgram(5);
    robot.joints().forEach((v, i) => expect(v).toBeCloseTo(at5.q[i], 6));
    boxes(app).map(posOf).forEach((p, i) => expect(dist(p, at5.b[i])).toBeLessThan(1e-6));
    app.resetSimulation();
    expect(app.simTime).toBe(0); expect(app.atHome).toBe(true);
    expect(robot.joints()).toEqual(home.q);
    boxes(app).forEach((b, i) => Array.from(b.poseAbs()).forEach((v, k) => expect(v).toBeCloseTo(home.boxes[i][k], 9)));
  });

  it('a station keeps its home state across tab switches: Reset after switching back restores it', () => {
    const app = makeApp('pickplace');
    const st1 = app.station; app.stations.push(st1);
    const robot = app.activeRobot!;
    const home = { q: robot.joints(), b: boxes(app).map(posOf) };
    app.runProgram(); tick(app, 6);
    expect(robot.joints()).not.toEqual(home.q);
    app.addStation('Other'); expect(app.station).not.toBe(st1);
    app.setStation(st1);
    expect(app.atHome).toBe(false);
    app.resetSimulation();
    expect(robot.joints()).toEqual(home.q);
    boxes(app).map(posOf).forEach((p, i) => expect(dist(p, home.b[i])).toBeLessThan(1e-6));
  });

  it('"Run from here" starts at the chosen instruction: only the last box is carried, the first two stay put', () => {
    const app = makeApp('pickplace');
    const prog = app.activeProgram!;
    const ins = prog.instructions();
    const k = ins.findIndex((i) => i.data.kind === 'move' && /3/.test((app.station.findById(i.data.targetId ?? '') as Target | null)?.name ?? ''));
    expect(k).toBeGreaterThan(5);
    const start = boxes(app).map(posOf);
    const h = app.runProgram(prog, { fromIndex: k })!;
    tick(app, 0.1);
    expect(h.instructionIndex).toBeGreaterThanOrEqual(k);
    app.runToEnd();
    expect(h.status).toBe('done');
    const end = boxes(app).map(posOf);
    expect(dist(end[0], start[0])).toBeLessThan(1e-6); expect(dist(end[1], start[1])).toBeLessThan(1e-6);
    expect(dist(end[2], start[2])).toBeGreaterThan(100);
    // seeking back replays the session from the same instruction (deterministic): box 3 back home, then carried again
    app.seekProgram(0.05);
    boxes(app).map(posOf).forEach((p, i) => expect(dist(p, start[i])).toBeLessThan(1e-6));
    app.seekProgram(app.simDuration + 1);
    boxes(app).map(posOf).forEach((p, i) => expect(dist(p, end[i])).toBeLessThan(1e-6));
  });

  it('a program plays up to its first error; the editor marker points at that instruction', () => {
    const app = makeApp('pickplace');
    const prog = app.activeProgram!;
    const robot = app.activeRobot!;
    const far = new Target('Unreachable'); app.station.addChild(far); far.setPose(transl(9000, 0, 500));
    const k = 6; // after Speed, Rounding, MoveJ Home, MoveJ Above 1, MoveL Pick 1, grip
    prog.addInstruction({ kind: 'move', moveType: 'MoveL', targetId: far.id }, prog.children.indexOf(prog.instructions()[k]));
    const q0 = robot.joints();
    app.runProgram(prog); app.runToEnd();
    const h = app.runs.at(-1)!;
    expect(h.status).toBe('error');
    expect(h.error!.instructionIndex).toBe(k);
    expect(app.errorInstruction(prog)).toBe(k);
    expect(robot.joints()).not.toEqual(q0); // the instructions before the error were played
    // the program editor marks exactly that row
    const pe = new ProgramEditor(app); pe.render();
    const rows = [...pe.el.querySelectorAll('.ins-row')];
    expect(rows.findIndex((r) => r.classList.contains('error-at'))).toBe(k);
    expect(rows.filter((r) => r.classList.contains('error-at')).length).toBe(1);
  });
});
