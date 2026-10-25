/**
 * Yaskawa Motoman INFORM (.JBI) posts, output-compatible with RoboDK's Motoman and Motoman_Cartesian posts
 * (tests/posts_gt.test.ts): pulse positions (RoboDK's default 1 pulse/deg, configurable through options.pulsesPerDeg),
 * 3-point MOVC (start, via, end), VJ / V speeds, PL or ACC/DEC (RoboDK's precedence rules), OT#/IN# IO, TIMER, MSG,
 * 29-column comments, frame/tool reminders or SETTOOL, BASE/RECTAN positions with RCONF in the Cartesian variant,
 * and program splitting with an M_<name> caller job. Studio extensions: loops (counter + JUMP), IF jumps.
 */
import { Mat4, multiply } from '../core/math/pose';
import { PostProcessor, PostProgram, PostFile, registerPost } from './base';
import { RoboPost, RoboPostCtx, runRoboPost, fx, pyRound, IoName, CtlCondition } from './robopost';
import { pose2Motoman } from './euler';

interface Job { name: string; pos: string[]; ins: string[]; nPos: number; posType: '' | 'pulse' | 'rectan'; posTool: number; rconf: string; tool0: number | null; first: boolean }

export class MotomanPost extends RoboPost {
  override PROG_EXT = 'JBI';
  protected jobs: Job[] = [];
  protected progJobs: Job[] = [];
  protected cur!: Job;
  protected nameLen = 7;
  protected vj = '50.00';
  protected v = '100.0';
  protected pl = '';
  protected accL: number | null = null; protected accJ: number | null = null;
  protected accStrL = ''; protected accStrJ = '';
  protected linAccMode = false; protected jointAccMode = false;
  protected tool = 9;
  protected frame: Mat4 | null = null;
  protected lastJoints: number[] | null = null;
  protected lastPose: Mat4 | null = null;
  protected lastConf: number[] = [0, 0, 0];
  protected nLines = 0;
  protected maxLines: number;
  protected lbl = 0;
  protected pulses: number[];
  constructor(ctx: RoboPostCtx, protected cartesian: boolean) {
    super(ctx);
    this.maxLines = ctx.options.maxLines ?? 2000;
    this.pulses = ctx.options.pulsesPerDeg ?? [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1];
  }
  protected jobName(n: string) { return n.replace(/[^A-Za-z0-9_]/g, '').toUpperCase().slice(0, this.nameLen); }
  protected newJob(name: string, first: boolean): void {
    this.cur = { name, pos: [], ins: [], nPos: 0, posType: '', posTool: -999, rconf: '', tool0: null, first };
    this.jobs.push(this.cur); this.progJobs.push(this.cur); this.nLines = 0;
  }
  override ProgStart(name: string): void { this.PROG_NAME = this.jobName(name); this.progJobs = []; this.newJob(this.PROG_NAME, true); }
  override ProgFinish(): void {
    if (this.progJobs.length > 1) {
      const m: Job = { name: `M_${this.PROG_NAME}`.slice(0, this.nameLen), pos: [], ins: this.progJobs.map((j) => `CALL JOB:${j.name}`), nPos: 0, posType: '', posTool: -999, rconf: '', tool0: null, first: false };
      this.jobs.push(m);
    }
  }
  protected splitIfNeeded(): void {
    if (this.nLines < this.maxLines) return;
    this.newJob(`${this.PROG_NAME}${this.progJobs.length + 1}`, false);
  }
  override addline(s: string): void { this.cur.ins.push(s); this.nLines++; }
  protected comment(t: string) { for (let i = 0; i < t.length || i === 0; i += 29) { this.addline(`'${t.slice(i, i + 29)}`); if (!t.length) break; } }
  override ProgSave(): PostFile[] {
    const d = this.ctx.date, p2 = (n: number) => String(n).padStart(2, '0');
    // RoboDK writes the last program's jobs first, then the others in order: when truncated names clash
    // (DropPart1..3 -> DROPPAR) the file left on disk is the last one written. Same result, main job listed first.
    const written = new Map<string, Job>();
    for (const j of [...this.progJobs, ...this.jobs.filter((x) => !this.progJobs.includes(x))]) written.set(j.name, j);
    const jobs = this.jobs.filter((j) => written.get(j.name) === j);
    if (jobs.length < this.jobs.length) this.ctx.warn(`Motoman: job names clash after truncation to ${this.nameLen} characters (${[...new Set(this.jobs.filter((j) => !jobs.includes(j)).map((j) => j.name))].join(', ')}): only one job per name is written, rename the programs`);
    return jobs.map((j) => {
      const L = ['/JOB', `//NAME ${j.name}`, '//POS', `///NPOS ${j.nPos},0,0,0,0,0`, ...j.pos, '//INST', `///DATE ${d.getFullYear()}/${p2(d.getMonth() + 1)}/${p2(d.getDate())} ${p2(d.getHours())}:${p2(d.getMinutes())}`,
        '///COMM Generated using VerticalBot Studio', `///ATTR SC,RW${this.cartesian ? ',RJ' : ''}`, '///GROUP1 RB1', 'NOP', ...j.ins, 'END'];
      return { name: `${j.name}.JBI`, content: L.join('\r\n') + '\r\n' };
    });
  }
  protected cName(n: number) { return `C${String(n).padStart(5, '0')}`; }
  protected pulse(joints: number[]): number {
    const j = this.cur;
    if (j.tool0 === null) j.tool0 = this.tool;
    if (j.posType !== 'pulse') { if (j.posType === '') j.pos.push(`///TOOL ${this.tool}`); j.pos.push('///POSTYPE PULSE', '///PULSE'); j.posType = 'pulse'; }
    const n = j.nPos++;
    j.pos.push(`${this.cName(n)}=${joints.slice(0, Math.max(6, Math.min(this.nAxes, joints.length))).map((v, i) => pyRound(v * (this.pulses[i] ?? 1))).join(',')}`);
    return n;
  }
  protected rectan(pose: Mat4, joints: number[], conf: number[]): number {
    const j = this.cur;
    if (j.tool0 === null) j.tool0 = this.tool;
    const rc = `${conf[2] ? 0 : 1},${conf[1] ? 1 : 0},${conf[0] ? 1 : 0},${(joints[3] ?? 0) >= 180 ? 1 : 0},${(joints[5] ?? 0) >= 180 ? 1 : 0},${(joints[0] ?? 0) >= 180 ? 1 : 0},0,0`;
    if (j.posType !== 'rectan') { j.pos.push(`///TOOL ${this.tool}`, '///POSTYPE BASE', '///RECTAN', `///RCONF ${rc}`); j.posType = 'rectan'; j.posTool = this.tool; j.rconf = rc; }
    else {
      if (j.posTool !== this.tool) { j.pos.push(`///TOOL ${this.tool}`); j.posTool = this.tool; }
      if (j.rconf !== rc) { j.pos.push(`///RCONF ${rc}`); j.rconf = rc; }
    }
    const n = j.nPos++;
    const [x, y, z, r, p, w] = pose2Motoman(this.frame ? multiply(this.frame, pose) : pose);
    j.pos.push(`${this.cName(n)}=${fx(x)},${fx(y)},${fx(z)},${fx(r, 2)},${fx(p, 2)},${fx(w, 2)}`);
    return n;
  }
  protected point(pose: Mat4 | null, joints: number[], conf: number[]): number {
    return this.cartesian && pose ? this.rectan(pose, joints, conf) : this.pulse(joints);
  }
  MoveJ(pose: Mat4 | null, joints: number[]): void {
    this.splitIfNeeded();
    const c = this.pulse(joints);
    this.addline(`MOVJ ${this.cName(c)} VJ=${this.vj}${this.jointAccMode ? this.accStrJ : this.pl}`);
    this.lastJoints = joints; this.lastPose = pose; this.lastWasCirc = false;
  }
  MoveL(pose: Mat4 | null, joints: number[], conf: number[]): void {
    this.splitIfNeeded();
    const c = this.point(pose, joints, conf);
    this.addline(`MOVL ${this.cName(c)} V=${this.v}${this.linAccMode ? this.accStrL : this.pl}`);
    this.lastJoints = joints; this.lastPose = pose; this.lastConf = conf; this.lastWasCirc = false;
  }
  override MoveC(pose1: Mat4 | null, joints1: number[], pose2: Mat4 | null, joints2: number[], conf1: number[], conf2: number[]): void {
    this.splitIfNeeded();
    // INFORM circles need 3 MOVC points: start (current position), via, end; a circle that continues a circle reuses
    // the previous end point as its start (RoboDK writes only via + end then)
    const pts = [];
    if (!this.lastWasCirc) pts.push(this.lastJoints ? this.point(this.lastPose, this.lastJoints, this.lastConf) : this.point(pose1, joints1, conf1));
    pts.push(this.point(pose1, joints1, conf1), this.point(pose2, joints2, conf2));
    for (const c of pts) this.addline(`MOVC ${this.cName(c)} V=${this.v}${this.pl}`);
    this.lastJoints = joints2; this.lastPose = pose2; this.lastConf = conf2;
    this.lastWasCirc = true;
  }
  protected lastWasCirc = false;
  override setFrame(pose: Mat4, _id: number, name: string): void {
    this.frame = pose;
    this.comment(`Using ${name} (targets wrt base):`);
    this.comment(pose2Motoman(pose).map((v) => fx(v, 1)).join(','));
  }
  override setTool(pose: Mat4, id: number, name: string): void {
    const v = pose2Motoman(pose);
    if (this.cartesian && id <= 0) {
      this.tool = 9;
      this.comment(`Setting Tool 9 (${name}):`);
      v.forEach((x, i) => this.addline(`SETE P00095 (${i + 1}) ${pyRound(x * (i < 3 ? 1000 : 100))}`));
      this.addline('SETTOOL TL#(9) P00095');
      return;
    }
    this.tool = id > 0 ? id : 9;
    this.comment(id > 0 ? `Tool ${id} (${name}) should be close to:` : `Tool ${name} should be close to:`);
    this.comment(v.map((x) => fx(x, 1)).join(','));
  }
  override Pause(ms: number): void { this.addline(ms <= 0 ? 'PAUSE' : `TIMER T=${fx(ms / 1000, 2)}`); }
  protected accStr(a: number) { const v = Math.min(100, pyRound(a)); return ` ACC=${v} DEC=${v}`; }
  override setSpeed(v: number): void { this.v = fx(v, 1); if (this.accL !== null) this.accStrL = this.accStr(this.accL); }
  override setSpeedJoints(w: number): void { this.vj = fx(Math.min(100, w), 2).slice(0, 5); if (this.accJ !== null) this.accStrJ = this.accStr(this.accJ); }
  override setAcceleration(a: number): void { this.accL = a; this.linAccMode = true; }
  override setAccelerationJoints(a: number): void { this.accJ = a; this.jointAccMode = true; }
  override setZoneData(z: number): void { this.pl = z < 0 ? '' : ` PL=${pyRound(Math.min(z, 8))}`; }
  protected onoff(v: number | boolean | string) { return (typeof v === 'string' ? ['1', 'true', 'on'].includes(v.toLowerCase()) : Number(v) > 0) ? 'ON' : 'OFF'; }
  override setDO(io: IoName, v: number | boolean | string): void { this.addline(`DOUT ${typeof io === 'number' ? `OT#(${io})` : io} ${this.onoff(v)}`); }
  /** Analog output: INFORM AOUT AO#(n) <value>. */
  override setAO(io: IoName, v: number | string): void { this.addline(`AOUT ${typeof io === 'number' ? `AO#(${io})` : io} ${v}`); }
  override waitDI(io: IoName, v: number | boolean | string, timeoutMs = -1): void {
    this.addline(`WAIT ${typeof io === 'number' ? `IN#(${io})` : io}=${this.onoff(v)}${timeoutMs > 0 ? ` T=${fx(timeoutMs / 1000, 2)}` : ''}`);
  }
  override RunCode(code: string, isCall = false): void { this.addline(isCall ? `CALL JOB:${code.toUpperCase().slice(0, this.nameLen)}` : code); }
  /** MSG texts are limited to 25 characters on the pendant: RoboDK writes one MSG per 25-character chunk. */
  override RunMessage(msg: string, isComment = false): void {
    if (isComment) { this.comment(msg); return; }
    for (let i = 0; i < msg.length || i === 0; i += 25) { this.addline(`MSG "${msg.slice(i, i + 25)}"`); if (!msg.length) break; }
  }
  // ---- control flow (INFORM) ----
  override ctlLoop(count: number, body: string): void {
    const k = ++this.lbl, b = this.jobName(body);
    if (count <= 0) { this.addline(`*L${k}`); this.addline(`CALL JOB:${b}`); this.addline(`JUMP *L${k}`); return; }
    this.addline(`SET I090 0`);
    this.addline(`*L${k}`);
    this.addline(`CALL JOB:${b}`);
    this.addline('INC I090');
    this.addline(`JUMP *L${k} IF I090<${Math.trunc(count)}`);
  }
  override ctlIf(cond: CtlCondition, thenProg: string | null, elseProg: string | null): void {
    if (cond.io === undefined) { super.ctlIf(cond, thenProg, elseProg); return; }
    const on = (cond.op === '!=') !== (Number(cond.value ?? 1) > 0);
    const src = (state: boolean) => `${typeof cond.io === 'number' ? `IN#(${cond.io})` : cond.io}=${state ? 'ON' : 'OFF'}`;
    if (thenProg) this.addline(`CALL JOB:${this.jobName(thenProg)} IF ${src(on)}`);
    if (elseProg) this.addline(`CALL JOB:${this.jobName(elseProg)} IF ${src(!on)}`);
  }
}

function motomanPost(id: string, name: string, cartesian: boolean, aliases: string[]): PostProcessor {
  return {
    id, name, brand: 'Yaskawa', extension: 'JBI', aliases,
    description: `INFORM job (RoboDK ${id} compatible${cartesian ? ', Cartesian positions w.r.t. the robot base' : ''}).`,
    generate(p: PostProgram, options: Record<string, any> = {}): PostFile[] { return runRoboPost((ctx) => new MotomanPost(ctx, cartesian), p, options, true); },
  };
}
export const MOTOMAN = motomanPost('Motoman', 'Yaskawa Motoman (INFORM JBI)', false, []);
export const MOTOMAN_CARTESIAN = motomanPost('Motoman_Cartesian', 'Yaskawa Motoman Cartesian (INFORM JBI)', true, ['Motoman_CartesianOnly']);
registerPost(MOTOMAN);
registerPost(MOTOMAN_CARTESIAN);
