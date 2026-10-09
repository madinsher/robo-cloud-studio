/**
 * ABB Robot Web Services (RWS, IRC5/OmniCore) driver over HTTP: reads joint targets, uploads RAPID modules
 * and starts execution. Requires the controller in automatic mode with RWS enabled (default user/password
 * "Default User" / "robotics"). Uses digest/basic auth via fetch.
 */
import { RobotDriver, DriverState, registerDriver } from './driver.ts';

export class ABBRwsDriver implements RobotDriver {
  readonly id = 'ABB_RWS';
  private base = '';
  private auth = 'Basic ' + Buffer.from('Default User:robotics').toString('base64');
  private st: DriverState = { connected: false, joints: [0, 0, 0, 0, 0, 0], status: 'disconnected' };
  private poll: NodeJS.Timeout | null = null;

  async connect(ip: string, port = 80): Promise<void> {
    this.base = `http://${ip}:${port}`;
    await this.readJoints();
    this.st.connected = true;
    this.st.status = 'ready';
    this.poll = setInterval(() => this.readJoints().catch(() => {}), 200);
  }
  private async req(path: string, init: RequestInit = {}): Promise<Response> {
    const res = await fetch(this.base + path, { ...init, headers: { Authorization: this.auth, Accept: 'application/json', ...(init.headers ?? {}) } });
    if (!res.ok) throw new Error(`RWS ${path}: ${res.status}`);
    return res;
  }
  private async readJoints(): Promise<void> {
    const res = await this.req('/rw/motionsystem/mechunits/ROB_1/jointtarget?json=1');
    const j = await res.json();
    const st = j?._embedded?._state?.[0] ?? {};
    this.st.joints = [1, 2, 3, 4, 5, 6].map((i) => parseFloat(st[`rax_${i}`] ?? '0'));
  }
  async disconnect(): Promise<void> { if (this.poll) clearInterval(this.poll); this.poll = null; this.st.connected = false; this.st.status = 'disconnected'; }
  state(): DriverState { return { ...this.st }; }
  /** Uploads and runs a small RAPID module for the move (requires mastership, automatic mode). */
  private async runRapid(body: string): Promise<void> {
    const mod = `MODULE VBS_DRIVER\nPROC main()\n${body}\nENDPROC\nENDMODULE\n`;
    await this.req('/fileservice/$home/VBS_DRIVER.mod', { method: 'PUT', body: mod, headers: { 'Content-Type': 'text/plain' } });
    await this.req('/rw/rapid/tasks/T_ROB1/modules?action=load', { method: 'POST', body: 'modulepath=$home/VBS_DRIVER.mod&replace=true', headers: { 'Content-Type': 'application/x-www-form-urlencoded' } });
    await this.req('/rw/rapid/execution?action=resetpp', { method: 'POST' });
    await this.req('/rw/rapid/execution?action=start', { method: 'POST', body: 'regain=continue&execmode=continue&cycle=once&condition=none&stopatbp=disabled&alltaskbytsp=false', headers: { 'Content-Type': 'application/x-www-form-urlencoded' } });
  }
  async moveJ(joints: number[], speedDegS = 60, wait = true): Promise<void> {
    await this.runRapid(`MoveAbsJ [[${joints.slice(0, 6).map((v) => v.toFixed(3)).join(',')}],[9E9,9E9,9E9,9E9,9E9,9E9]],v${Math.max(5, Math.round(speedDegS * 5))},fine,tool0;`);
    if (wait) await this.waitJoints(joints);
  }
  private async waitJoints(target: number[], tolDeg = 0.5, timeoutMs = 60000): Promise<void> {
    const t0 = Date.now();
    while (Date.now() - t0 < timeoutMs) { if (target.every((v, i) => Math.abs(v - this.st.joints[i]) < tolDeg)) return; await new Promise((r) => setTimeout(r, 100)); }
    throw new Error('ABB move timeout');
  }
  async moveL(pose: number[], speedMmS = 250, wait = true): Promise<void> {
    const m = pose;
    const tr = m[0] + m[5] + m[10];
    let w: number, x: number, y: number, z: number;
    if (tr > 0) { const s = Math.sqrt(tr + 1) * 2; w = 0.25 * s; x = (m[6] - m[9]) / s; y = (m[8] - m[2]) / s; z = (m[1] - m[4]) / s; }
    else if (m[0] > m[5] && m[0] > m[10]) { const s = Math.sqrt(1 + m[0] - m[5] - m[10]) * 2; w = (m[6] - m[9]) / s; x = 0.25 * s; y = (m[4] + m[1]) / s; z = (m[8] + m[2]) / s; }
    else if (m[5] > m[10]) { const s = Math.sqrt(1 + m[5] - m[0] - m[10]) * 2; w = (m[8] - m[2]) / s; x = (m[4] + m[1]) / s; y = 0.25 * s; z = (m[9] + m[6]) / s; }
    else { const s = Math.sqrt(1 + m[10] - m[0] - m[5]) * 2; w = (m[1] - m[4]) / s; x = (m[8] + m[2]) / s; y = (m[9] + m[6]) / s; z = 0.25 * s; }
    await this.runRapid(`MoveL [[${m[12].toFixed(3)},${m[13].toFixed(3)},${m[14].toFixed(3)}],[${w.toFixed(6)},${x.toFixed(6)},${y.toFixed(6)},${z.toFixed(6)}],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]],v${Math.round(speedMmS)},fine,tool0;`);
    if (wait) await new Promise((r) => setTimeout(r, 500));
  }
  async setDO(io: string, value: boolean | number): Promise<void> {
    await this.req(`/rw/iosystem/signals/${encodeURIComponent(io)}?action=set`, { method: 'POST', body: `lvalue=${value ? 1 : 0}`, headers: { 'Content-Type': 'application/x-www-form-urlencoded' } });
  }
  async getDI(io: string): Promise<number> {
    const res = await this.req(`/rw/iosystem/signals/${encodeURIComponent(io)}?json=1`);
    const j = await res.json();
    return parseFloat(j?._embedded?._state?.[0]?.lvalue ?? '0');
  }
  async runScript(text: string): Promise<void> { await this.runRapid(text); }
  async stop(): Promise<void> { await this.req('/rw/rapid/execution?action=stop', { method: 'POST', body: 'stopmode=stop', headers: { 'Content-Type': 'application/x-www-form-urlencoded' } }); }
}
registerDriver('ABB_RWS', () => new ABBRwsDriver());
