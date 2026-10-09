/**
 * Generic ROS 2 driver through rosbridge (ws://robot:9090): publishes trajectory_msgs/JointTrajectory to a
 * joint trajectory controller and follows sensor_msgs/JointState. Works with any robot exposing ros2_control
 * (Fanuc, Yaskawa, Stäubli, UR, KUKA, Techman… ROS 2 drivers) and with the controllers of this repository.
 */
import WebSocket from 'ws';
import { RobotDriver, DriverState, registerDriver } from './driver.ts';

const DEG = Math.PI / 180, RAD = 180 / Math.PI;

export class Ros2Driver implements RobotDriver {
  readonly id = 'ROS2';
  private ws: WebSocket | null = null;
  private st: DriverState = { connected: false, joints: [], status: 'disconnected' };
  jointNames: string[] = [];
  trajectoryTopic = process.env.ROS2_TRAJECTORY_TOPIC ?? '/joint_trajectory_controller/joint_trajectory';
  jointStateTopic = process.env.ROS2_JOINT_STATES ?? '/joint_states';
  cmdJointStateTopic = '/cmd_joint_state';

  async connect(ip: string, port = 9090): Promise<void> {
    await new Promise<void>((resolve, reject) => {
      this.ws = new WebSocket(`ws://${ip}:${port}`);
      this.ws.on('open', () => resolve());
      this.ws.on('error', (e) => reject(e));
      this.ws.on('message', (d) => this.onMessage(String(d)));
    });
    this.send({ op: 'subscribe', topic: this.jointStateTopic, type: 'sensor_msgs/msg/JointState' });
    this.send({ op: 'advertise', topic: this.trajectoryTopic, type: 'trajectory_msgs/msg/JointTrajectory' });
    this.send({ op: 'advertise', topic: this.cmdJointStateTopic, type: 'sensor_msgs/msg/JointState' });
    this.st.connected = true;
    this.st.status = 'ready';
  }
  private send(o: any) { this.ws?.send(JSON.stringify(o)); }
  private onMessage(text: string) {
    try {
      const m = JSON.parse(text);
      if (m.op === 'publish' && m.topic === this.jointStateTopic) {
        const names: string[] = m.msg.name ?? [];
        const pos: number[] = m.msg.position ?? [];
        if (!this.jointNames.length) this.jointNames = names;
        this.st.joints = this.jointNames.map((n) => { const i = names.indexOf(n); return i >= 0 ? pos[i] * RAD : 0; });
      }
    } catch { /* ignore */ }
  }
  async disconnect(): Promise<void> { this.ws?.close(); this.ws = null; this.st.connected = false; this.st.status = 'disconnected'; }
  state(): DriverState { return { ...this.st }; }
  async moveJ(joints: number[], speedDegS = 60, wait = true): Promise<void> {
    const names = this.jointNames.length ? this.jointNames : joints.map((_, i) => `joint_${i + 1}`);
    const dmax = this.st.joints.length ? Math.max(...joints.map((v, i) => Math.abs(v - (this.st.joints[i] ?? v)))) : 90;
    const t = Math.max(0.5, dmax / speedDegS);
    this.send({ op: 'publish', topic: this.trajectoryTopic, msg: { joint_names: names, points: [{ positions: joints.map((v) => v * DEG), time_from_start: { sec: Math.floor(t), nanosec: Math.round((t % 1) * 1e9) } }] } });
    this.send({ op: 'publish', topic: this.cmdJointStateTopic, msg: { name: names, position: joints.map((v) => v * DEG) } });
    if (wait) {
      const t0 = Date.now();
      while (Date.now() - t0 < (t + 5) * 1000) { if (this.st.joints.length && joints.every((v, i) => Math.abs(v - this.st.joints[i]) < 0.5)) return; await new Promise((r) => setTimeout(r, 50)); }
    }
  }
  async moveL(_pose: number[], _speed = 250, _wait = true): Promise<void> { throw new Error('ROS 2 driver: linear moves must be pre-sampled (use the ROS2 post: JointTrajectory with many points)'); }
  async setDO(io: string, value: boolean | number): Promise<void> { this.send({ op: 'publish', topic: `/io/${io}`, msg: { data: !!value } }); }
  async getDI(_io: string): Promise<boolean> { return false; }
  /** Accepts the JSON trajectory produced by the ROS2 post and publishes it in one message. */
  async runScript(text: string): Promise<void> {
    const traj = JSON.parse(text);
    this.send({ op: 'publish', topic: this.trajectoryTopic, msg: { joint_names: traj.joint_names, points: traj.points.map((p: any) => ({ positions: p.positions, time_from_start: p.time_from_start })) } });
  }
  async stop(): Promise<void> { this.send({ op: 'publish', topic: this.trajectoryTopic, msg: { joint_names: this.jointNames, points: [] } }); }
}
registerDriver('ROS2', () => new Ros2Driver());
