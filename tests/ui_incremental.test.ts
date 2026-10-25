// @vitest-environment jsdom
/**
 * T11: panels update incrementally (no rebuild on joint / pose events, rows keep their DOM nodes so clicks during a
 * simulation are never lost), listener hygiene and per-tab undo across station switches, one-step teach undo,
 * virtualised tree / instruction list for big stations, URDF mesh drops without stray objects, .tool mounting and the
 * TCP gizmo jog.
 */
import { describe, it, expect, beforeAll } from 'vitest';
import { existsSync, readFileSync } from 'node:fs';
import { installDomStubs, makeApp, tickMs } from './ui_harness';
import { App, ItemType, Program, Robot, SceneObject, Target, Frame, Tool } from '../src/app';
import { Station } from '../src/core/items/item';
import { TreePanel } from '../src/ui/tree';
import { ProgramEditor } from '../src/ui/program-editor';
import { multiply, transl, invert } from '../src/core/math/pose';

beforeAll(() => installDomStubs());
const listenerCount = (st: Station) => [...((st.events as any).listeners as Map<string, Set<unknown>>).values()].reduce((n, s) => n + s.size, 0);

describe('incremental panels', () => {
  it('joint and pose updates never rebuild the tree or the program editor; rows keep their DOM nodes', async () => {
    const app = makeApp('pickplace');
    const tree = new TreePanel(app); const ed = new ProgramEditor(app);
    document.body.append(tree.el, ed.el);
    await tickMs(40);
    const t0 = tree.rebuilds, e0 = ed.rebuilds;
    const rowOf = (name: string) => [...tree.el.querySelectorAll<HTMLElement>('.tree-row')].find((r) => r.querySelector('.tree-name')!.childNodes[0].textContent === name)!;
    const pedestal = rowOf('Pedestal'); const insRow = ed.el.querySelector('.ins-row');
    const robot = app.activeRobot!; const box = app.station.find('Box 1')!;
    for (let i = 0; i < 120; i++) { robot.setJoints(robot.joints().map((q, k) => q + (k === 0 ? 0.5 : 0))); box.setPose(transl(i, 0, 0)); if (i % 20 === 0) await tickMs(35); }
    await tickMs(40);
    expect(tree.rebuilds).toBe(t0); expect(ed.rebuilds).toBe(e0);
    expect(rowOf('Pedestal')).toBe(pedestal); expect(ed.el.querySelector('.ins-row')).toBe(insRow);
  });

  it('every click on a tree row selects its item while two programs play (10 of 10)', async () => {
    const app = makeApp('welding');
    const tree = new TreePanel(app); document.body.append(tree.el);
    const progs = app.station.itemsOfType<Program>(ItemType.PROGRAM);
    for (const p of progs.filter((x) => x.robot() instanceof Robot).slice(0, 2)) app.runProgram(p);
    expect(app.activeRuns().length).toBe(2);
    const names = [...app.station.walk()].filter((it) => it !== app.station && it.type !== ItemType.INSTRUCTION).map((it) => it.name).slice(0, 10);
    let hits = 0;
    for (const n of names) {
      for (let k = 0; k < 4; k++) (app as any).tick(1 / 30);
      await tickMs(35); // debounced repaints fire between clicks, as during real playback
      const row = [...tree.el.querySelectorAll<HTMLElement>('.tree-row')].find((r) => r.querySelector('.tree-name')!.childNodes[0].textContent === n);
      row?.click();
      if (app.station.selection[0]?.name === n) hits++;
    }
    expect(hits).toBe(10);
    expect(app.isPlaying()).toBe(true);
  });

  it('station tab round trips keep the listener count and each station keeps its undo history', () => {
    const app = makeApp('pickplace');
    const tree = new TreePanel(app); const ed = new ProgramEditor(app); void tree; void ed;
    const a = app.station;
    app.addFrame(app.station, 'Undoable');
    const nA = listenerCount(a);
    const b = app.addStation('Second');
    const nB = listenerCount(b);
    for (let i = 0; i < 5; i++) { app.setStation(a); app.setStation(b); }
    expect(listenerCount(b)).toBe(nB);
    app.setStation(a);
    expect(listenerCount(a)).toBe(nA);
    expect(app.canUndo).toBe(true);
    app.undo();
    expect(app.station.find('Undoable')).toBeNull();
  });

  it('one teach is one undo step: the instruction and its target go together', () => {
    const app = makeApp('pickplace');
    const prog = app.activeProgram!;
    const nIns = prog.instructions().length, nT = app.station.itemsOfType(ItemType.TARGET).length;
    app.activeRobot!.setJoints([10, -70, -100, -100, -90, 0]);
    app.teach('MoveJ');
    expect(prog.instructions().length).toBe(nIns + 1); expect(app.station.itemsOfType(ItemType.TARGET).length).toBe(nT + 1);
    app.undo();
    expect(app.activeProgram!.instructions().length).toBe(nIns); expect(app.station.itemsOfType(ItemType.TARGET).length).toBe(nT);
  });

  it('a 6000-item station and a 6000-instruction program render a bounded number of rows quickly', () => {
    const app = makeApp();
    const r = app.addRobotFromLibrary('UR5e');
    const f = app.station.addChild(new Frame('Many'));
    const p = new Program('Big'); app.station.addChild(p); p.setRobot(r);
    const t = new Target('T'); f.addChild(t); t.setJoints(r.joints());
    for (let i = 0; i < 6000; i++) { const o = new SceneObject(`Part ${i}`); f.addChild(o); p.addInstruction({ kind: 'move', moveType: 'MoveJ', targetId: t.id }); }
    const t0 = performance.now();
    const tree = new TreePanel(app); const ed = new ProgramEditor(app);
    app.setStation(app.station); app.select(p);
    tree.render(); ed.render();
    const ms = performance.now() - t0;
    expect(tree.el.querySelectorAll('.tree-row').length).toBeLessThanOrEqual(400);
    expect(ed.el.querySelectorAll('.ins-row').length).toBeLessThanOrEqual(400);
    expect(ms).toBeLessThan(3000);
    // the big program is not compiled on the UI thread on selection
    expect(p.lastResult).toBeNull();
  });
});

describe('opening files', () => {
  const urdf = `<robot name="probe_bot"><link name="base_link"><visual><geometry><mesh filename="package://probe/meshes/base_link.stl"/></geometry></visual></link>
<link name="arm_link"><visual><geometry><mesh filename="package://probe/meshes/arm_link.stl"/></geometry></visual></link>
<joint name="j1" type="revolute"><parent link="base_link"/><child link="arm_link"/><origin xyz="0 0 0.1"/><axis xyz="0 0 1"/><limit lower="-3" upper="3" effort="1" velocity="1"/></joint></robot>`;
  const stl = (s: number) => { const n = 1; const b = new ArrayBuffer(84 + n * 50); const dv = new DataView(b); dv.setUint32(80, n, true); [0, 0, 0, s, 0, 0, 0, s, 0].forEach((v, k) => dv.setFloat32(84 + 12 + k * 4, v, true)); return new Uint8Array(b); };
  const objects = (app: App) => app.station.itemsOfType<SceneObject>(ItemType.OBJECT).map((o) => o.name);

  it('URDF with its STLs, and URDF first then STLs: the meshes resolve for the robot, no stray objects', async () => {
    for (const order of ['together', 'late'] as const) {
      const app = makeApp();
      const files = [new File([urdf], 'probe_bot.urdf'), new File([stl(0.1)], 'base_link.stl'), new File([stl(0.2)], 'arm_link.stl')];
      if (order === 'together') await app.openFiles(files);
      else { await app.openFiles([files[0]]); expect(app.assets.resolve('package://probe/meshes/base_link.stl')).toBeUndefined(); await app.openFiles(files.slice(1)); }
      const robot = app.station.itemsOfType<Robot>(ItemType.ROBOT)[0];
      expect(robot?.name, order).toBe('probe_bot');
      expect(objects(app), order).toEqual([]);
      for (const l of robot.chain.links) for (const v of l.visuals) if (v.mesh) expect(app.assets.resolve(v.mesh), `${order} ${v.mesh}`).toBeTruthy();
    }
  });

  it('an STL nobody references becomes an object', async () => {
    const app = makeApp();
    await app.openFiles([new File([stl(0.1)], 'bracket.stl')]);
    expect(objects(app)).toEqual(['bracket']);
  });

  const TOOL = 'C:/RoboDK/Library/Spindle.tool';
  it.skipIf(!existsSync(TOOL))('a .tool opened with a robot active is mounted on that robot and becomes its active tool', async () => {
    const app = makeApp('pickplace');
    const robot = app.activeRobot!;
    const before = robot.tools().length;
    const bytes = readFileSync(TOOL);
    await app.openFiles([new File([bytes], 'Spindle.tool')]);
    expect(robot.tools().length).toBe(before + 1);
    const tool = robot.activeTool()!;
    expect(tool.parent).toBe(robot);
    expect(robot.tools()).toContain(tool);
    expect(app.station.children.some((c) => c instanceof Tool)).toBe(false);
  });
});

describe('TCP gizmo', () => {
  it('dragging the gizmo on a mounted tool jogs the robot through IK; the tool TCP definition is unchanged', () => {
    const app = makeApp('pickplace');
    const robot = app.activeRobot!; const tool = robot.activeTool()!;
    const tcpDef = Array.from(tool.pose()); const q0 = robot.joints();
    const goal = multiply(transl(0, -60, 40), tool.poseAbs());
    app.select(tool);
    app.editPose(tool, goal);
    expect(Array.from(tool.pose())).toEqual(tcpDef);
    expect(robot.joints()).not.toEqual(q0);
    const reached = tool.poseAbs();
    for (const i of [12, 13, 14]) expect(reached[i]).toBeCloseTo(goal[i], 1);
    const err = multiply(invert(goal), reached); expect(Math.abs(err[0] - 1) + Math.abs(err[5] - 1) + Math.abs(err[10] - 1)).toBeLessThan(1e-4);
  });
  it('the gizmo on an ordinary object moves it', () => {
    const app = makeApp('pickplace');
    const box = app.station.find('Box 1')!;
    const goal = multiply(transl(100, 0, 0), box.poseAbs());
    app.editPose(box, goal);
    expect(box.poseAbs()[12]).toBeCloseTo(goal[12], 6);
  });
});
