/**
 * KUKA driver via KUKAVARPROXY (KVP, TCP 7000): reads $AXIS_ACT and writes variables that a small KRL
 * host program polls (COM_ACTION / COM_E6AXIS style, as used by RoboDK's KUKA driver). Requires
 * KUKAVARPROXY running on the KRC and a KRL program reading the variables.
 */
import net from 'node:net';
import { RobotDriver, DriverState, registerDriver } from './driver.ts';

export class KukaKvpDriver implements RobotDriver {
  readonly id = 'KUKA_KVP';
  private sock: net.Socket | null = null;
  private st: DriverState = { connected: false, joints: [0, 0, 0, 0, 0, 0], status: 'disconnected' };
  private msgId = 0;
  private pending: Array<(v: string) => void> = [];
  private buf = Buffer.alloc(0);
  private poll: NodeJS.Timeout | null = null;

  async connect(ip: string, port = 7000): Promise<void> {
    await new Promise<void>((resolve, reject) => { this.sock = net.createConnection({ host: ip, port }, () => resolve()); this.sock.on('error', reject); this.sock.on('data', (d) => this.onData(d)); });
    this.st.connected = true; this.st.status = 'ready';
    this.poll = setInterval(() => this.readVar('$AXIS_ACT').then((v) => { const m = [...v.matchAll(/A(\d)\s+([-\d.]+)/g)]; if (m.length >= 6) this.st.joints = m.slice(0, 6).map((x) => parseFloat(x[2])); }).catch(() => {}), 100);
  }
  private onData(d: Buffer) {
    this.buf = Buffer.concat([this.buf, d]);
    while (this.buf.length >= 4) {
      const len = this.buf.readUInt16BE(2);
      if (this.buf.length < 4 + len) break;
      const body = this.buf.subarray(4, 4 + len);
      this.buf = this.buf.subarray(4 + len);
      const vlen = body.readUInt16BE(1);
      const value = body.subarray(3, 3 + vlen).toString('latin1');
      this.pending.shift()?.(value);
    }
  }
  private frame(fn: 0 | 1, name: string, value = ''): Buffer {
    const n = Buffer.from(name, 'latin1'), v = Buffer.from(value, 'latin1');
    const body = fn === 0 ? Buffer.concat([Buffer.from([0]), u16(n.length), n]) : Buffer.concat([Buffer.from([1]), u16(n.length), n, u16(v.length), v]);
    const id = (this.msgId = (this.msgId + 1) & 0xffff);
    return Buffer.concat([u16(id), u16(body.length), body]);
  }
  readVar(name: string): Promise<string> { return new Promise((resolve, reject) => { if (!this.sock) return reject(new Error('not connected')); this.pending.push(resolve); this.sock.write(this.frame(0, name)); }); }
  writeVar(name: string, value: string): Promise<string> { return new Promise((resolve, reject) => { if (!this.sock) return reject(new Error('not connected')); this.pending.push(resolve); this.sock.write(this.frame(1, name, value)); }); }
  async disconnect(): Promise<void> { if (this.poll) clearInterval(this.poll); this.sock?.destroy(); this.sock = null; this.st.connected = false; this.st.status = 'disconnected'; }
  state(): DriverState { return { ...this.st }; }
  async moveJ(joints: number[], _speed = 60, wait = true): Promise<void> {
    await this.writeVar('COM_E6AXIS', `{E6AXIS: A1 ${joints[0].toFixed(3)}, A2 ${joints[1].toFixed(3)}, A3 ${joints[2].toFixed(3)}, A4 ${joints[3].toFixed(3)}, A5 ${joints[4].toFixed(3)}, A6 ${joints[5].toFixed(3)}, E1 0, E2 0, E3 0, E4 0, E5 0, E6 0}`);
    await this.writeVar('COM_ACTION', '2');
    if (wait) { const t0 = Date.now(); while (Date.now() - t0 < 60000) { if (joints.every((v, i) => Math.abs(v - this.st.joints[i]) < 0.5)) return; await new Promise((r) => setTimeout(r, 100)); } }
  }
  async moveL(pose: number[], _speed = 250, wait = true): Promise<void> {
    const m = pose;
    const b = Math.asin(-Math.max(-1, Math.min(1, m[2])));
    const a = Math.atan2(m[1], m[0]), c = Math.atan2(m[6], m[10]);
    const R = 180 / Math.PI;
    await this.writeVar('COM_FRAME', `{FRAME: X ${m[12].toFixed(3)}, Y ${m[13].toFixed(3)}, Z ${m[14].toFixed(3)}, A ${(a * R).toFixed(3)}, B ${(b * R).toFixed(3)}, C ${(c * R).toFixed(3)}}`);
    await this.writeVar('COM_ACTION', '3');
    if (wait) await new Promise((r) => setTimeout(r, 500));
  }
  async setDO(io: string, value: boolean | number): Promise<void> { await this.writeVar(`$OUT[${io.replace(/\D/g, '') || 1}]`, value ? 'TRUE' : 'FALSE'); }
  async getDI(io: string): Promise<boolean> { return (await this.readVar(`$IN[${io.replace(/\D/g, '') || 1}]`)).trim() === 'TRUE'; }
  async runScript(_text: string): Promise<void> { throw new Error('KVP driver cannot upload programs; use the KRL post and the KRC file transfer'); }
  async stop(): Promise<void> { await this.writeVar('COM_ACTION', '0'); }
}
function u16(n: number): Buffer { const b = Buffer.alloc(2); b.writeUInt16BE(n); return b; }
registerDriver('KUKA_KVP', () => new KukaKvpDriver());
