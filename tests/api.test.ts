import { describe, it, expect } from 'vitest';
import { Station } from '../src/core/items/item';
import { Robolink, ITEM_TYPE_ROBOT, ITEM_TYPE_PROGRAM, ITEM_TYPE_TARGET, INSTRUCTION_COMMENT, robomath } from '../src/api/robolink';
import { executeRpc } from '../src/api/rpc';
import { demos } from '../src/demos';

describe('RoboDK-compatible API', () => {
  it('scripts a station like a RoboDK script would', () => {
    const st = new Station('API');
    const RDK = new Robolink(st);
    const robot = RDK.AddRobot('UR5e');
    expect(robot.Valid()).toBe(true);
    const frame = RDK.AddFrame('Frame 1');
    frame.setPose(robomath.transl(500, 0, 0));
    robot.setPoseFrame(frame);
    const tool = robot.AddTool(robomath.transl(0, 0, 100), 'Tool 1');
    robot.setPoseTool(tool);
    robot.setJoints([0, -90, 90, -90, -90, 0]);
    const prog = RDK.AddProgram('Prog 1', robot);
    const t1 = RDK.AddTarget('T1', frame, robot);
    prog.MoveJ(t1);
    const p = robot.Pose();
    robot.MoveL(p.mul(robomath.transl(0, 0, 50)));
    const t2 = RDK.AddTarget('T2', frame, robot);
    prog.MoveL(t2);
    prog.RunInstruction('done', INSTRUCTION_COMMENT);
    const [valid, time, dist] = prog.Update();
    expect(valid).toBeGreaterThan(0);
    expect(time).toBeGreaterThan(0);
    expect(dist).toBeGreaterThan(40);
    expect(RDK.ItemList(ITEM_TYPE_TARGET).length).toBe(2);
    expect(RDK.Item('', ITEM_TYPE_ROBOT).Name()).toBe('Universal Robots UR5e');
    expect(RDK.Item('prog', ITEM_TYPE_PROGRAM).Valid()).toBe(true);
    const [ok, code] = prog.MakeProgram('', 'Universal_Robots');
    expect(ok).toBe(true);
    expect(code).toContain('movel');
    const q = robot.SolveIK(robot.SolveFK([10, -80, 80, -90, -90, 10]));
    expect(q.length).toBe(6);
  });

  it('executes JSON-RPC requests (server protocol)', () => {
    const st = demos[0].build();
    const RDK = new Robolink(st);
    const r1 = executeRpc(RDK, { id: 1, method: 'Item', params: ['', ITEM_TYPE_ROBOT] });
    expect(r1.error).toBeUndefined();
    const robotId = r1.result.$item;
    const r2 = executeRpc(RDK, { id: 2, method: 'Joints', target: robotId });
    expect(r2.result.length).toBe(6);
    const r3 = executeRpc(RDK, { id: 3, method: 'Pose', target: robotId });
    expect(r3.result.$pose.length).toBe(4);
    const r4 = executeRpc(RDK, { id: 4, method: 'setPose', target: robotId, params: [{ $pose: r3.result.$pose }] });
    expect(r4.error).toBeUndefined();
    const r5 = executeRpc(RDK, { id: 5, method: 'Nope' });
    expect(r5.error).toMatch(/Unknown method/);
    const r6 = executeRpc(RDK, { id: 6, method: 'ItemList', params: [ITEM_TYPE_PROGRAM, true] });
    expect(r6.result).toContain('PickPlace');
  });

  it('all demos build and validate', () => {
    for (const d of demos) {
      const st = d.build();
      expect([...st.walk()].length).toBeGreaterThan(3);
    }
  });
});
