/**
 * Generic ROS 2 driver through rosbridge (ws://robot:9090): publishes trajectory_msgs/JointTrajectory to a
 * joint trajectory controller and follows sensor_msgs/JointState. Works with any robot exposing ros2_control
 * (Fanuc, Yaskawa, Stäubli, UR, KUKA, Techman… ROS 2 drivers) and with the controllers of this repository.
 *
 * Joints are mapped by NAME, never by the order of /joint_states (UR drivers publish shoulder_lift first, and
 * /joint_states also carries gripper / wheel joints). The command joint list, in the studio's joint order, is:
 *   1. `jointNames` set by the caller / `ROS2_JOINT_NAMES` (comma separated), else
 *   2. the controller's `joints` parameter (`<controller>/get_parameters` service through rosbridge), else
 *   3. the /joint_states names (with a warning in `state().error`: the order may not match the studio robot).
 * Prismatic joints (types from the robot_state_publisher `robot_description`) are in metres on the ROS side and mm
 * in the studio, revolute ones in rad / deg. Digital IO: `/io/<name>` std_msgs/Bool, advertised before publishing,
 * inputs subscribed on first read.
 */
import WebSocket from 'ws';
import { RobotDriver, DriverState, registerDriver } from './driver.ts';

const DEG = Math.PI / 180, RAD = 180 / Math.PI;

export class Ros2Driver implements RobotDriver {
  readonly id = 'ROS2';
  private ws: WebSocket | null = null;
  private st: DriverState = { connected: false, joints: [], status: 'disconnected' };
  /** Controlled joints in the studio's order (see the header for how they are found). */
  jointNames: string[] = (process.env.ROS2_JOINT_NAMES ?? '').split(',').map((s) => s.trim()).filter(Boolean);
  /** Joint types from robot_description (name -> 'prismatic' | 'revolute' | ...). */
  jointTypes = new Map<string, string>();
  trajectoryTopic = process.env.ROS2_TRAJECTORY_TOPIC ?? '/joint_trajectory_controller/joint_trajectory';
  jointStateTopic = process.env.ROS2_JOINT_STATES ?? '/joint_states';
  cmdJointStateTopic = '/cmd_joint_state';
  /** Latest positions by joint name (ROS units). */
  private positions = new Map<string, number>();
  private namesFrom: 'config' | 'controller' | 'joint_states' | '' = '';
  private inputs = new Map<string, boolean>();
  private advertised = new Set<string>();
  private calls = new Map<string, (values: any) => void>();
  private callId = 0;

  async connect(ip: string, port = 9090): Promise<void> {
    await new Promise<void>((resolve, reject) => {
      this.ws = new WebSocket(`ws://${ip}:${port}`);
      this.ws.on('open', () => resolve());
      this.ws.on('error', (e) => reject(e));
      this.ws.on('message', (d) => this.onMessage(String(d)));
    });
    if (this.jointNames.length) this.namesFrom = 'config';
    this.send({ op: 'subscribe', topic: this.jointStateTopic, type: 'sensor_msgs/msg/JointState' });
    this.advertise(this.trajectoryTopic, 'trajectory_msgs/msg/JointTrajectory');
    this.advertise(this.cmdJointStateTopic, 'sensor_msgs/msg/JointState');
    const controller = this.trajectoryTopic.replace(/\/joint_trajectory$/, '');
    const [joints, desc] = await Promise.all([
      this.namesFrom ? Promise.resolve(null) : this.getParameter(controller, 'joints'),
      this.getParameter('/robot_state_publisher', 'robot_description'),
    ]);
    const list = joints?.string_array_value;
    if (!this.namesFrom && Array.isArray(list) && list.length) { this.jointNames = list.map(String); this.namesFrom = 'controller'; }
    if (typeof desc?.string_value === 'string') this.parseDescription(desc.string_value);
    this.st.connected = true;
    this.st.status = 'ready';
  }

  private send(o: any) { this.ws?.send(JSON.stringify(o)); }
  private advertise(topic: string, type: string) { if (this.advertised.has(topic)) return; this.advertised.add(topic); this.send({ op: 'advertise', topic, type }); }

  /** rcl_interfaces/srv/GetParameters through rosbridge; resolves to the ParameterValue or null (no answer in `timeoutMs`). */
  getParameter(node: string, name: string, timeoutMs = 1500): Promise<any | null> {
    const id = `t5_${++this.callId}`;
    return new Promise((resolve) => {
      const timer = setTimeout(() => { this.calls.delete(id); resolve(null); }, timeoutMs);
      this.calls.set(id, (values) => { clearTimeout(timer); resolve(values?.values?.[0] ?? null); });
      this.send({ op: 'call_service', id, service: `${node}/get_parameters`, type: 'rcl_interfaces/srv/GetParameters', args: { names: [name] } });
    });
  }

  private parseDescription(xml: string) {
    for (const m of xml.matchAll(/<joint\s[^>]*?name\s*=\s*"([^"]+)"[^>]*?type\s*=\s*"([^"]+)"|<joint\s[^>]*?type\s*=\s*"([^"]+)"[^>]*?name\s*=\s*"([^"]+)"/g)) {
      const name = m[1] ?? m[4], type = m[2] ?? m[3];
      this.jointTypes.set(name, type);
    }
  }

  private toStudio(name: string, v: number) { return this.jointTypes.get(name) === 'prismatic' ? v * 1000 : v * RAD; }
  private toRos(name: string, v: number) { return this.jointTypes.get(name) === 'prismatic' ? v / 1000 : v * DEG; }

  private onMessage(text: string) {
    try {
      const m = JSON.parse(text);
      if (m.op === 'service_response' && m.id && this.calls.has(m.id)) { const cb = this.calls.get(m.id)!; this.calls.delete(m.id); cb(m.result === false ? null : m.values); return; }
      if (m.op !== 'publish') return;
      if (m.topic === this.jointStateTopic) {
        const names: string[] = m.msg?.name ?? [];
        const pos: number[] = m.msg?.position ?? [];
        names.forEach((n, i) => { if (typeof pos[i] === 'number') this.positions.set(n, pos[i]); });
        if (!this.jointNames.length && names.length) {
          this.jointNames = [...names];
          this.namesFrom = 'joint_states';
          this.st.error = 'ROS 2 driver: joint list taken from /joint_states (order may differ from the robot) — set ROS2_JOINT_NAMES or expose the controller joints parameter';
        }
        this.st.joints = this.jointNames.map((n) => (this.positions.has(n) ? this.toStudio(n, this.positions.get(n)!) : 0));
      } else if (m.topic.startsWith('/io/')) this.inputs.set(m.topic.slice(4), !!m.msg?.data);
    } catch { /* ignore */ }
  }

  async disconnect(): Promise<void> { this.ws?.close(); this.ws = null; this.st.connected = false; this.st.status = 'disconnected'; }
  state(): DriverState { return { ...this.st, joints: [...this.st.joints] }; }

  async moveJ(joints: number[], speedDegS = 60, wait = true): Promise<void> {
    const names = this.jointNames.length ? this.jointNames.slice(0, joints.length) : joints.map((_, i) => `joint_${i + 1}`);
    if (names.length < joints.length) throw new Error(`ROS 2 driver: ${joints.length} joint values for ${names.length} controller joints (${names.join(', ')})`);
    const cur = this.st.joints;
    const dmax = cur.length ? Math.max(...joints.map((v, i) => Math.abs(v - (cur[i] ?? v)))) : 90;
    const t = Math.max(0.5, dmax / speedDegS);
    const positions = joints.map((v, i) => this.toRos(names[i], v));
    this.send({ op: 'publish', topic: this.trajectoryTopic, msg: { joint_names: names, points: [{ positions, velocities: positions.map(() => 0), time_from_start: { sec: Math.floor(t), nanosec: Math.round((t % 1) * 1e9) } }] } });
    this.send({ op: 'publish', topic: this.cmdJointStateTopic, msg: { name: names, position: positions } });
    if (wait) {
      const t0 = Date.now();
      while (Date.now() - t0 < (t + 5) * 1000) { if (this.st.joints.length && joints.every((v, i) => Math.abs(v - this.st.joints[i]) < 0.5)) return; await new Promise((r) => setTimeout(r, 50)); }
    }
  }
  async moveL(_pose: number[], _speed = 250, _wait = true): Promise<void> { throw new Error('ROS 2 driver: linear moves must be pre-sampled (use the ROS2 post: JointTrajectory with many points)'); }
  async setDO(io: string, value: boolean | number): Promise<void> {
    const topic = `/io/${io}`;
    this.advertise(topic, 'std_msgs/msg/Bool');
    this.send({ op: 'publish', topic, msg: { data: !!value } });
  }
  async getDI(io: string): Promise<boolean> {
    const topic = `/io/${io}`;
    if (!this.inputs.has(io)) { this.send({ op: 'subscribe', topic, type: 'std_msgs/msg/Bool' }); this.inputs.set(io, false); }
    return this.inputs.get(io)!;
  }
  /** Accepts the JSON trajectory produced by the ROS2 post and publishes it in one message. */
  async runScript(text: string): Promise<void> {
    const traj = JSON.parse(text);
    this.send({ op: 'publish', topic: this.trajectoryTopic, msg: { joint_names: traj.joint_names, points: traj.points.map((p: any) => ({ positions: p.positions, time_from_start: p.time_from_start })) } });
  }
  async stop(): Promise<void> { this.send({ op: 'publish', topic: this.trajectoryTopic, msg: { joint_names: this.jointNames, points: [] } }); }
}
registerDriver('ROS2', () => new Ros2Driver());
