import { describe, it, expect } from 'vitest';
import { Station, SceneObject, Tool } from '../src/core/items/item';
import * as RL from '../src/api/robolink';
import { demos } from '../src/demos';
import { Robot } from '../src/core/items/robot';
import { transl, mul, rotx, DEG } from '../src/core/math/pose';

const { Robolink, robomath, ITEM_TYPE_ROBOT, ITEM_TYPE_PROGRAM, ITEM_TYPE_OBJECT, COLLISION_OFF, COLLISION_ON, MAKE_ROBOT_6DOF, MAKE_ROBOT_1T, CALIBRATE_FRAME_3P_P1_ORIGIN } = RL;

describe('RoboDK API — extended surface', () => {
  it('attach/detach, collisions, copy/paste, scale, joint list', () => {
    const st = demos[0].build();
    const RDK = new Robolink(st);
    const robot = RDK.Item('UR10e', ITEM_TYPE_ROBOT);
    const box = RDK.Item('Box 1', ITEM_TYPE_OBJECT);
    const pick = RDK.Item('Pick 1');
    robot.MoveJ(pick);
    const attached = robot.AttachClosest();
    expect(attached.Valid()).toBe(true);
    expect(attached.Parent().Type()).toBe(4); // tool
    robot.MoveJ(RDK.Item('Home'));
    const det = robot.DetachAll();
    expect(det).toBeTruthy();
    expect(box.Parent().Type()).not.toBe(4);
    expect(RDK.Collisions()).toBeGreaterThanOrEqual(0);
    RDK.setCollisionActive(COLLISION_OFF);
    expect(RDK.Collisions()).toBe(0);
    RDK.setCollisionActive(COLLISION_ON);
    expect(RDK.Collision_SetPair(robot, box, -1, -1, false)).toBe(true);
    const prog = RDK.Item('PickPlace', ITEM_TYPE_PROGRAM);
    const [msg, rows, status] = prog.InstructionListJoints();
    expect(status).toBe(0);
    expect(msg).toBe('Success');
    expect(rows.length).toBeGreaterThan(20);
    expect(rows[0].length).toBe(8);
    RDK.Copy(prog);
    const pasted = RDK.Paste() as RL.RobolinkItem;
    expect(pasted.Name()).toBe('PickPlace 2');
    expect(pasted.InstructionCount()).toBe(prog.InstructionCount());
    box.Scale(2);
    expect((box.item as SceneObject).geometry[0].primitive!.kind).toBe('box');
    expect(robot.JointsConfig(robot.Joints()).length).toBe(9);
    expect(robot.MoveJ_Test(robot.Joints(), robot.Joints())).toBe(0);
    expect(robot.JointPoses().length).toBe(7);
    const [hit, item] = RDK.Collision_Line([700, -150, 2000], [700, -150, -100]);
    expect(hit).toBe(true);
    expect(item.Valid()).toBe(true);
  });

  it('BuildMechanism, AddTargetJ, machining project, spray, calibration helpers, events', async () => {
    const st = new Station();
    const RDK = new Robolink(st);
    const r = RDK.BuildMechanism(MAKE_ROBOT_6DOF, [], [400, 25, 455, 35, 420, 80], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [1, 1, 1, 1, 1, 1], [-170, -100, -120, -185, -120, -350], [170, 135, 156, 185, 120, 350], undefined, undefined, 'Built');
    expect(r.DOF()).toBe(6);
    const rail = RDK.BuildMechanism(MAKE_ROBOT_1T, [], [3000], [0], [0], [1], [0], [3000], undefined, undefined, 'Rail');
    expect(rail.DOF()).toBe(1);
    const tj = RDK.AddTargetJ('J1', [10, -30, 40, 0, 20, 0], undefined, r);
    expect(tj.isJointTarget()).toBe(true);
    const part = RDK.AddCurve([[0, 0, 0], [100, 0, 0], [150, 50, 0]]);
    part.setPose(robomath.transl(650, 0, 300));
    const proj = RDK.AddMachiningProject('Weld', r);
    const [, points, unreachable] = proj.setMachiningParameters('', part, 'Step=10 Approach=30 FreeZ');
    expect(points).toBeGreaterThan(10);
    expect(unreachable).toBe(0);
    expect(RDK.ItemList(ITEM_TYPE_PROGRAM, true)).toContain('Weld');
    const tool = r.AddTool(robomath.transl(0, 0, 120), 'Gun');
    const panel = RDK.Item('Curve');
    (panel.item as SceneObject).geometry = [{ primitive: { kind: 'box', size: [300, 300, 20] }, origin: Array.from(transl(0, 0, 10)) }];
    const gid = RDK.Spray_Add(tool, panel, 'ANGLE=25 RANGE=500 STEP=25');
    expect(gid).toBeGreaterThan(0);
    RDK.Spray_SetState(1, gid);
    const q = (r.item as Robot).solveIK(mul(transl(800, 0, 600), rotx(180 * DEG)), { seed: (r.item as Robot).joints() }, transl(0, 0, 120));
    if (q.ok) { (r.item as Robot).setJoints(q.joints); for (let i = 0; i < 10; i++) RDK.spray.step(0.1); const [, stats] = RDK.Spray_GetStats(gid); expect(stats[0]).toBeGreaterThan(0); }
    RDK.Spray_Clear(gid);
    const frame = RDK.Calibrate_Reference([[100, 0, 0], [200, 0, 0], [100, 100, 0]], CALIBRATE_FRAME_3P_P1_ORIGIN, false, r);
    expect(frame.Pos()[0]).toBeCloseTo(100);
    const [tcp, stats] = RDK.CalibrateTool([[0, -60, 60, 0, 0, 0], [20, -60, 60, 0, 30, 0], [-20, -50, 70, 10, -20, 5], [5, -70, 50, 40, 40, 20]].map((j) => j), RL.JOINT_FORMAT, RL.CALIBRATE_TCP_BY_POINT, r, tool);
    expect(tcp.length).toBe(3);
    expect(stats.length).toBeGreaterThan(2);
    const iso = RDK.Popup_ISO9283_CubeProgram(r, [800, 0, 600], 300);
    expect(iso.Valid()).toBe(true);
    expect(RDK.Popup_ISO9283_CubeProgram(r).Update()[1]).toBeGreaterThan(0);
    RDK.EventsListen();
    RDK.events.push({ type: 'selection', itemId: r.item!.id });
    const ev = await RDK.WaitForEvent(1);
    expect(ev?.[0]).toBe(RL.EVENT_SELECTION_TREE_CHANGED);
    expect(ev?.[1].Name()).toBe('Built');
    RDK.registerPlugin('demo', { command: (c, v) => `${c}:${v}` });
    expect(RDK.PluginCommand('demo', 'ping', 1)).toBe('ping:1');
    expect(RDK.PluginLoad('demo')).toBe(true);
    expect(RDK.getOpenStations().length).toBe(1);
    RDK.setFlagsItem(r, RL.FLAG_ITEM_SELECTABLE);
    expect(RDK.getFlagsItem(r)).toBe(RL.FLAG_ITEM_SELECTABLE);
    const merged = RDK.MergeItems([RDK.AddShape([[0, 0, 0, 100, 0, 0, 0, 100, 0]]), RDK.AddShape([[0, 0, 100, 100, 0, 100, 0, 100, 100]])]);
    expect(merged.Valid()).toBe(true);
    void Tool;
  });

  it('robot calibration through the API improves accuracy', () => {
    const st = new Station();
    const RDK = new Robolink(st);
    const r = RDK.AddRobot('UR5e');
    const robot = r.item as Robot;
    const realDh = robot.chain.dh!.map((d, i) => ({ ...d, theta: d.theta + [0.2, -0.1, 0.15, 0.05, -0.2, 0.1][i] }));
    const { simulateMeasurements } = require_calib();
    const joints: number[][] = [];
    let s = 5;
    const rnd = () => { s = (s * 1103515245 + 12345) & 0x7fffffff; return s / 0x7fffffff; };
    for (let i = 0; i < 25; i++) joints.push([rnd() * 180 - 90, -rnd() * 120 - 20, rnd() * 140 - 70, rnd() * 180 - 90, rnd() * 180 - 90, rnd() * 360 - 180]);
    const ms = simulateMeasurements(robot, realDh, joints, 0, [0, 0, 50]);
    const res = RDK.Calibrate_Robot(ms.map((m: any) => [...m.joints, ...m.xyz]), r, { tool: [0, 0, 50] });
    expect(res.after.max).toBeLessThan(res.before.mean);
    expect(res.after.max).toBeLessThan(0.1);
  });
});
import * as calib from '../src/core/calibration/robot';
function require_calib() { return calib; }
