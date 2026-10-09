/**
 * Universal Robots driver: URScript over the primary/secondary interface (TCP 30002) and joint feedback
 * from the real-time interface (TCP 30003, 8-byte doubles: q_actual at byte offset 252 for CB3/e-Series).
 */
import net from 'node:net';
import { RobotDriver, DriverState, registerDriver } from './driver.ts';

const DEG = Math.PI / 180, RAD = 180 / Math.PI;

export class URDriver implements RobotDriver {
  readonly id = 'UR';
  private cmd: net.Socket | null = null;
  private rt: net.Socket | null = null;
  private st: DriverState = { connected: false, joints: [0, 0, 0, 0, 0, 0], status: 'disconnected' };
  private buf = Buffer.alloc(0);

  async connect(ip: string, port = 30002): Promise<void> {
    await new Promise<void>((resolve, reject) => {
      this.cmd = net.createConnection({ host: ip, port }, () => resolve());
      this.cmd.on('error', (e) => { this.st.error = e.message; reject(e); });
    });
    this.rt = net.createConnection({ host: ip, port: 30003 });
    this.rt.on('data', (d) => this.onRt(d));
    this.rt.on('error', (e) => { this.st.error = e.message; });
    this.st.connected = true;
    this.st.status = 'ready';
  }
  private onRt(d: Buffer) {
    this.buf = Buffer.concat([this.buf, d]);
    while (this.buf.length >= 4) {
      const len = this.buf.readInt32BE(0);
      if (this.buf.length < len) break;
      const pkt = this.buf.subarray(0, len);
      this.buf = this.buf.subarray(len);
      if (len >= 252 + 48) {
        const q: number[] = [];
        for (let i = 0; i < 6; i++) q.push(pkt.readDoubleBE(252 + i * 8) * RAD);
        this.st.joints = q;
      }
    }
  }
  async disconnect(): Promise<void> { this.cmd?.destroy(); this.rt?.destroy(); this.cmd = this.rt = null; this.st.connected = false; this.st.status = 'disconnected'; }
  state(): DriverState { return { ...this.st }; }
  private send(script: string): Promise<void> {
    return new Promise((resolve, reject) => { if (!this.cmd) return reject(new Error('not connected')); this.cmd.write(script.endsWith('\n') ? script : script + '\n', (e) => (e ? reject(e) : resolve())); });
  }
  private async waitJoints(target: number[], tolDeg = 0.5, timeoutMs = 60000): Promise<void> {
    const t0 = Date.now();
    while (Date.now() - t0 < timeoutMs) {
      if (target.every((v, i) => Math.abs(v - this.st.joints[i]) < tolDeg)) return;
      await new Promise((r) => setTimeout(r, 50));
    }
    throw new Error('UR move timeout');
  }
  async moveJ(joints: number[], speedDegS = 60, wait = true): Promise<void> {
    await this.send(`movej([${joints.map((v) => (v * DEG).toFixed(5)).join(',')}], a=1.4, v=${(speedDegS * DEG).toFixed(3)})`);
    if (wait) await this.waitJoints(joints);
  }
  async moveL(pose: number[], speedMmS = 250, wait = true): Promise<void> {
    // pose: column-major 4x4 in mm -> UR p[x,y,z,rx,ry,rz] in m/rad
    const m = pose;
    const tr = m[0] + m[5] + m[10];
    const angle = Math.acos(Math.max(-1, Math.min(1, (tr - 1) / 2)));
    let rx = 0, ry = 0, rz = 0;
    if (angle > 1e-9) { const s = 1 / (2 * Math.sin(angle)); rx = (m[6] - m[9]) * s * angle; ry = (m[8] - m[2]) * s * angle; rz = (m[1] - m[4]) * s * angle; }
    await this.send(`movel(p[${(m[12] / 1000).toFixed(5)},${(m[13] / 1000).toFixed(5)},${(m[14] / 1000).toFixed(5)},${rx.toFixed(5)},${ry.toFixed(5)},${rz.toFixed(5)}], a=1.2, v=${(speedMmS / 1000).toFixed(3)})`);
    if (wait) await new Promise((r) => setTimeout(r, 200));
  }
  async setDO(io: string, value: boolean | number): Promise<void> { await this.send(`set_standard_digital_out(${io.replace(/\D/g, '') || 0}, ${value ? 'True' : 'False'})`); }
  async getDI(_io: string): Promise<boolean> { return false; /* requires RTDE input recipe; not part of the primary interface */ }
  async runScript(text: string): Promise<void> { await this.send(text); }
  async stop(): Promise<void> { await this.send('stopj(2)'); }
}
registerDriver('UR', () => new URDriver());
