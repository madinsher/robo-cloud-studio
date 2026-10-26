/**
 * T5 P0 e): server/drivers/ros2.ts maps joints by NAME. A fake rosbridge server plays a UR ros2_control setup:
 * /joint_states in the UR driver order (shoulder_lift first) plus a prismatic gripper joint, the joint trajectory
 * controller's `joints` parameter and robot_state_publisher's robot_description answered through call_service.
 */
import { describe, it, expect, beforeAll, afterAll } from 'vitest';
import { WebSocketServer, WebSocket as WsSocket } from 'ws';
import { Ros2Driver } from '../server/drivers/ros2';

const UR = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'];
const STATE_ORDER = ['shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint', 'shoulder_pan_joint', 'finger_joint'];
const DESC = `<robot name="ur">${UR.map((j) => `<joint name="${j}" type="revolute"/>`).join('')}<joint type="prismatic" name="finger_joint"/></robot>`;

describe('Ros2Driver (rosbridge, name-based joint mapping)', () => {
  let wss: WebSocketServer;
  let port = 0;
  const inbox: any[] = [];
  let sock: WsSocket | null = null;
  let answerController = true;
  beforeAll(async () => {
    wss = new WebSocketServer({ port: 0, host: '127.0.0.1' });
    await new Promise<void>((r) => wss.on('listening', () => r()));
    port = (wss.address() as any).port;
    wss.on('connection', (s) => {
      sock = s;
      s.on('message', (d) => {
        const m = JSON.parse(d.toString());
        inbox.push(m);
        if (m.op === 'call_service' && m.service === '/joint_trajectory_controller/get_parameters' && answerController) s.send(JSON.stringify({ op: 'service_response', id: m.id, service: m.service, result: true, values: { values: [{ type: 9, string_array_value: UR }] } }));
        if (m.op === 'call_service' && m.service === '/robot_state_publisher/get_parameters') s.send(JSON.stringify({ op: 'service_response', id: m.id, service: m.service, result: true, values: { values: [{ type: 4, string_value: DESC }] } }));
      });
    });
  });
  afterAll(async () => { sock?.terminate(); await new Promise<void>((r) => wss.close(() => r())); });
  const until = async (f: () => boolean, ms = 3000) => { const t0 = Date.now(); while (!f()) { if (Date.now() - t0 > ms) throw new Error('timeout'); await new Promise((r) => setTimeout(r, 10)); } };

  it('commands the controller joints in their order, reads /joint_states by name, converts prismatic units, advertises IO', async () => {
    const d = new Ros2Driver();
    d.jointNames = [];
    await d.connect('127.0.0.1', port);
    expect(d.jointNames).toEqual(UR);
    expect(d.jointTypes.get('finger_joint')).toBe('prismatic');
    // UR driver order + gripper: pan = 0.1 rad is the 6th entry
    sock!.send(JSON.stringify({ op: 'publish', topic: '/joint_states', msg: { name: STATE_ORDER, position: [-1.0, 0.5, 0, 0, 0, 0.1, 0.02] } }));
    await until(() => d.state().joints.length === 6);
    expect(d.state().joints[0]).toBeCloseTo(0.1 * 180 / Math.PI, 9); // shoulder_pan, not shoulder_lift
    expect(d.state().joints[1]).toBeCloseTo(-1.0 * 180 / Math.PI, 9);
    await d.moveJ([10, -90, 90, -90, -90, 0], 60, false);
    await until(() => inbox.some((m) => m.op === 'publish' && m.topic === '/joint_trajectory_controller/joint_trajectory'));
    const traj = inbox.find((m) => m.op === 'publish' && m.topic === '/joint_trajectory_controller/joint_trajectory').msg;
    expect(traj.joint_names).toEqual(UR);
    expect(traj.points[0].positions.length).toBe(6);
    expect(traj.points[0].positions[0]).toBeCloseTo(10 * Math.PI / 180, 12);
    await expect(d.moveJ([1, 2, 3, 4, 5, 6, 7], 60, false)).rejects.toThrow(/7 joint values for 6 controller joints/);
    await d.setDO('gripper_close', true);
    await until(() => inbox.some((m) => m.op === 'publish' && m.topic === '/io/gripper_close'));
    const ia = inbox.findIndex((m) => m.op === 'advertise' && m.topic === '/io/gripper_close');
    const ip = inbox.findIndex((m) => m.op === 'publish' && m.topic === '/io/gripper_close');
    expect(inbox[ia].type).toBe('std_msgs/msg/Bool');
    expect(ia).toBeLessThan(ip);
    expect(await d.getDI('part_present')).toBe(false);
    await until(() => inbox.some((m) => m.op === 'subscribe' && m.topic === '/io/part_present'));
    sock!.send(JSON.stringify({ op: 'publish', topic: '/io/part_present', msg: { data: true } }));
    await until(() => (d as any).inputs.get('part_present') === true);
    expect(await d.getDI('part_present')).toBe(true);
    await d.disconnect();
  });

  it('prismatic joints: m on the ROS side, mm in the studio; without a controller answer the joint list comes from /joint_states (warned)', async () => {
    answerController = false;
    const d = new Ros2Driver();
    d.jointNames = [];
    await d.connect('127.0.0.1', port);
    sock!.send(JSON.stringify({ op: 'publish', topic: '/joint_states', msg: { name: ['finger_joint'], position: [0.02] } }));
    await until(() => d.state().joints.length === 1);
    expect(d.state().joints[0]).toBeCloseTo(20, 9);
    expect(d.state().error).toMatch(/joint list taken from \/joint_states/);
    inbox.length = 0;
    await d.moveJ([35], 60, false);
    await until(() => inbox.some((m) => m.op === 'publish'));
    expect(inbox.find((m) => m.op === 'publish').msg.points[0].positions[0]).toBeCloseTo(0.035, 12);
    await d.disconnect();
    answerController = true;
  });
});
