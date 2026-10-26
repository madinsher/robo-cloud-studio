/**
 * Fanuc TP (.LS) posts, output-compatible with RoboDK's Fanuc_R30i (R-30iA/iB/iC) and Fanuc_RJ3 posts
 * (tests/posts_gt.test.ts): numbered /MN lines, /POS with joint or Cartesian positions and RoboDK CONFIG flags /
 * turn numbers, UFRAME/UTOOL selection by index (1..9) or through PR[9] for un-numbered frames, J% / mm/sec speeds,
 * CNT / FINE, ACC, two-line circular moves, WAIT with timeout labels, MESSAGE / comment splitting at the 72-column
 * limit and program splitting with an M_<name> caller. Studio extensions: FOR/ENDFOR loops, IF ... CALL, RUN threads.
 */
import { Mat4 } from '../core/math/pose';
import { PostProcessor, PostProgram, PostFile, registerPost } from './base';
import { RoboPost, RoboPostCtx, runRoboPost, fx, pyRound, IoName, CtlCondition } from './robopost';
import { pose2Fanuc } from './euler';

interface Part { name: string; lines: string[]; pos: string[]; nPos: number }

export class FanucPost extends RoboPost {
  override PROG_EXT = 'LS';
  protected files: Part[] = [];
  protected progParts: Part[] = [];
  protected cur!: Part;
  protected uf = 9;
  protected ut = 9;
  protected speed = '200mm/sec';
  protected jspeed = '20%';
  protected cnt = 'FINE';
  protected acc = '';
  protected lbl = 0;
  protected nLines = 0;
  protected maxLines: number;
  protected nameLen: number;
  protected callLen: number;
  protected regN = 0;
  constructor(ctx: RoboPostCtx, protected rj3: boolean) {
    super(ctx);
    this.maxLines = ctx.options.maxLines ?? 9999;
    this.nameLen = rj3 ? 6 : 10;
    this.callLen = rj3 ? 8 : 12;
  }
  protected safe(n: string) { return n.replace(/[^A-Za-z0-9_]/g, ''); }
  protected newPart(name: string): Part { const p: Part = { name, lines: [], pos: [], nPos: 0 }; this.files.push(p); this.progParts.push(p); this.cur = p; this.nLines = 0; return p; }
  override ProgStart(name: string): void {
    this.PROG_NAME = this.safe(name).slice(0, this.nameLen);
    this.progParts = [];
    this.newPart(this.PROG_NAME);
  }
  override ProgFinish(): void {
    if (this.progParts.length > 1) {
      const main: Part = { name: `M_${this.PROG_NAME}`.slice(0, this.nameLen), lines: this.progParts.map((p) => `  CALL ${p.name} ;`), pos: [], nPos: 0 };
      this.files.push(main);
    }
  }
  protected splitIfNeeded(): void {
    if (this.nLines < this.maxLines) return;
    this.newPart(`${this.PROG_NAME.slice(0, this.nameLen)}${this.progParts.length + 1}`);
  }
  /** One /MN line (without number). */
  protected mn(s: string): void { this.cur.lines.push(s); this.nLines++; }
  override addline(s: string): void { this.mn(`  ${s} ;`); }
  override ProgSave(): PostFile[] {
    const d = this.ctx.date, p2 = (n: number) => String(n).padStart(2, '0');
    const date = `DATE ${p2(d.getFullYear() % 100)}-${p2(d.getMonth() + 1)}-${p2(d.getDate())}  TIME ${p2(d.getHours())}:${p2(d.getMinutes())}:${p2(d.getSeconds())}`;
    return this.files.map((p) => {
      const L = [`/PROG  ${p.name}`, '/ATTR', 'OWNER\t\t= MNEDITOR;', 'COMMENT\t\t= "VerticalBot sequence";', 'PROG_SIZE\t= 0;', `CREATE\t\t= ${date};`, `MODIFIED\t= ${date};`, `FILE_NAME\t= ${p.name};`, 'VERSION\t\t= 0;', `LINE_COUNT\t= ${p.lines.length};`, 'MEMORY_SIZE\t= 0;', 'PROTECT\t\t= READ_WRITE;',
        'TCD:  STACK_SIZE\t= 0,', '      TASK_PRIORITY\t= 50,', '      TIME_SLICE\t= 0,', '      BUSY_LAMP_OFF\t= 0,', '      ABORT_REQUEST\t= 0,', '      PAUSE_REQUEST\t= 0;', `DEFAULT_GROUP\t= ${this.rj3 ? '1,*,*,*,*' : '1,*,*,*,*,*,*'};`, 'CONTROL_CODE\t= 00000000 00000000;', '/MN'];
      p.lines.forEach((l, i) => L.push(`${String(i + 1).padStart(4)}:${l.replace('\n', `\r\n       `)}`));
      L.push('/POS', ...p.pos, '/END');
      return { name: `${p.name}.LS`, content: L.join('\r\n') + '\r\n' };
    });
  }
  protected addPos(pose: Mat4 | null, joints: number[], conf: number[] | null): number {
    const n = ++this.cur.nPos;
    const P = this.cur.pos;
    P.push(`P[${n}]{`, '   GP1:');
    if (!pose || !conf) {
      P.push(`    UF : ${this.uf}, UT : ${this.ut},    `);
      const j = (i: number) => `J${i + 1}=    ${fx(joints[i] ?? 0)} deg`;
      P.push(`\t${j(0)},\t${j(1)},\t${j(2)},`, `\t${j(3)},\t${j(4)},\t${j(5)}`);
    } else {
      // turn 0 for (-180, 180]: a joint at exactly -180 deg is turn -1 (RoboDK 5.9.4, 03.j Laser P[77] J4 -180)
      const turn = (i: number) => Math.ceil(((joints[i] ?? 0) - 180) / 360) || 0;
      const cfg = `'${conf[2] ? 'F' : 'N'} ${conf[1] ? 'D' : 'U'} ${conf[0] ? 'B' : 'T'}, ${turn(0)}, ${turn(3)}, ${turn(5)}'`;
      const [x, y, z, w, p, r] = pose2Fanuc(pose);
      const c = (v: number) => fx(v).padStart(9);
      P.push(`    UF : ${this.uf}, UT : ${this.ut},        CONFIG : ${cfg},`, `\tX = ${c(x)}  mm,\tY = ${c(y)}  mm,\tZ = ${c(z)}  mm,`, `\tW = ${c(w)} deg,\tP = ${c(p)} deg,\tR = ${c(r)} deg`);
    }
    P.push('};');
    return n;
  }
  protected accTxt() { return this.acc ? ` ACC${this.acc}` : ''; }
  MoveJ(_pose: Mat4 | null, joints: number[]): void {
    this.splitIfNeeded();
    this.motion(`J P[${this.addPos(null, joints, null)}] ${this.jspeed} ${this.cnt}${this.accTxt()} ;`);
  }
  MoveL(pose: Mat4 | null, joints: number[], conf: number[]): void {
    this.splitIfNeeded();
    this.motion(`L P[${this.addPos(pose, joints, pose ? conf : null)}] ${this.speed} ${this.cnt}${this.accTxt()} ;`);
  }
  override MoveC(pose1: Mat4 | null, joints1: number[], pose2: Mat4 | null, joints2: number[], conf1: number[], conf2: number[]): void {
    this.splitIfNeeded();
    const a = this.addPos(pose1, joints1, pose1 ? conf1 : null), b = this.addPos(pose2, joints2, pose2 ? conf2 : null);
    this.motion(`C P[${a}] \nP[${b}] ${this.speed} ${this.cnt} ;`);
  }
  protected frameLines(kind: 'UFRAME' | 'UTOOL', pose: Mat4, id: number): number {
    const v = pose2Fanuc(pose);
    if (id >= 1 && id <= 9) {
      this.addline(`${kind}_NUM=${id}`);
      this.addline(`! ${kind === 'UFRAME' ? 'UF' : 'UT'}${id}:${v.map((x) => fx(x, 1)).join(',')}`);
      return id;
    }
    v.forEach((x, i) => this.addline(`PR[9,${i + 1}]=${fx(x)}`));
    this.addline(`${kind}[9]=PR[9]`);
    this.addline(`${kind}_NUM=9`);
    return 9;
  }
  override setFrame(pose: Mat4, id: number): void { this.uf = this.frameLines('UFRAME', pose, id); }
  override setTool(pose: Mat4, id: number): void { this.ut = this.frameLines('UTOOL', pose, id); }
  override Pause(ms: number): void { this.addline(ms <= 0 ? 'PAUSE' : `WAIT  ${fx(ms / 1000, 2)}(sec)`); }
  protected pct(x: number) { return `${Math.max(1, Math.min(100, pyRound(x)))}%`; }
  override setSpeed(v: number): void { this.speed = `${pyRound(v)}mm/sec`; this.jspeed = this.pct(v / 50); }
  override setSpeedJoints(w: number): void { this.jspeed = this.pct(w / 2); }
  override setAcceleration(a: number): void { this.acc = String(Math.min(100, pyRound(a))); }
  override setAccelerationJoints(a: number): void { this.acc = String(Math.min(100, pyRound(a))); }
  override setZoneData(z: number): void { this.cnt = z < 0 ? 'FINE' : `CNT${Math.min(100, Math.trunc(z))}`; }
  protected onoff(v: number | boolean | string) { return (typeof v === 'string' ? ['1', 'true', 'on'].includes(v.toLowerCase()) : Number(v) > 0) ? 'ON' : 'OFF'; }
  override setDO(io: IoName, v: number | boolean | string): void { this.addline(`${typeof io === 'number' ? `DO[${io}]` : io}=${this.onoff(v)}`); }
  override setAO(io: IoName, v: number | string): void { this.addline(`${typeof io === 'number' ? `AO[${io}]` : io}=(${v})`); }
  override waitDI(io: IoName, v: number | boolean | string, timeoutMs = -1): void {
    const c = `${typeof io === 'number' ? `DI[${io}]` : io}=${this.onoff(v)}`;
    if (timeoutMs <= 0) { this.addline(`WAIT ${c}`); return; }
    const k = ++this.lbl;
    this.addline(`$WAITTMOUT=${Math.trunc(timeoutMs)}`);
    this.addline(`WAIT ${c} TIMEOUT, LBL[${k}]`);
    this.addline(`MESSAGE[Timed out for LBL ${k}]`);
    this.addline('PAUSE');
    this.addline(`LBL[${k}]`);
  }
  protected callName(code: string) {
    const m = code.trim().match(/^([^(]*)(\(.*\))?$/)!;
    // RoboDK hands program names with their blanks (RunCode("Pick EarCup 1", True)); its Fanuc posts drop them
    const base = m[1].trim().replace(/\s+/g, '');
    // like RoboDK's Fanuc posts: a call keeps up to 8 (RJ3) / 12 (R30i) characters, also for a program generated in
    // this run whose file name is cut to 6 / 10 (03.j MainProgram: CALL PickEarC for PickEa.LS); the program must be
    // renamed for the controller (reported)
    if (!m[2] && this.ctx.programs.includes(base) && this.safe(base).length > this.nameLen) this.addlog(`Program ${base} is written to ${this.safe(base).slice(0, this.nameLen)}.LS but called as ${base.slice(0, this.callLen)} (RoboDK's Fanuc name limits): rename it to ${this.nameLen} characters or less`);
    return `${base.slice(0, this.callLen)}${m[2] ?? ''}`;
  }
  /** Last motion line (RoboDK's arc-welding calls modify it). */
  protected lastMotion: { part: Part; idx: number } | null = null;
  protected motion(s: string): void { this.mn(s); this.lastMotion = { part: this.cur, idx: this.cur.lines.length - 1 }; }
  override RunCode(code: string, isCall = false): void {
    // RoboDK arc welding: ArcStart / ArcEnd become options of the previous motion line (FINE termination) and
    // ArcStart adds a weave instruction ("ArcStart(3)" -> Weave Sine[3])
    const arc = isCall ? code.trim().match(/^Arc(Start|End)\s*(?:\((.*)\))?$/i) : null;
    if (arc && this.lastMotion) {
      const { part, idx } = this.lastMotion;
      part.lines[idx] = part.lines[idx].replace(/ CNT\d+/, ' FINE').replace(/ ;$/, ` Arc ${arc[1].toLowerCase() === 'start' ? 'Start' : 'End'}[11] ;`);
      if (arc[1].toLowerCase() === 'start') this.addline(`Weave Sine[${arc[2] ?? '2.0Hz,8.0mm,0.075s,0.075'}]`);
      return;
    }
    if (isCall) this.addline(`CALL ${this.callName(code)}`);
    else this.mn(`  ${code};`);
  }
  /** Fanuc lines are limited to 72 columns: texts are cut in fixed-size chunks like RoboDK does. */
  protected chunks(text: string, n: number): string[] {
    const out: string[] = [];
    for (let i = 0; i < text.length; i += n) { const c = text.slice(i, i + n).trim(); if (c) out.push(c); }
    return out.length ? out : [''];
  }
  override RunMessage(msg: string, isComment = false): void {
    if (isComment) for (const c of this.chunks(msg, 61)) this.addline(`! ${c}`);
    else for (const c of this.chunks(msg, 54)) this.addline(`MESSAGE[${c}]`);
  }
  // ---- control flow (Fanuc TP) ----
  override ctlLoop(count: number, body: string): void {
    const name = this.callName(body);
    if (count <= 0) { const k = ++this.lbl; this.addline(`LBL[${k}]`); this.addline(`CALL ${name}`); this.addline(`JMP LBL[${k}]`); return; }
    const r = 10 + (this.regN++ % 10);
    this.addline(`FOR R[${r}]=1 TO ${Math.trunc(count)}`);
    this.addline(`CALL ${name}`);
    this.addline('ENDFOR');
  }
  override ctlIf(cond: CtlCondition, thenProg: string | null, elseProg: string | null): void {
    const c = cond.io !== undefined ? `${typeof cond.io === 'number' ? `DI[${cond.io}]` : cond.io}${cond.op === '!=' ? '<>' : '='}${this.onoff(cond.value ?? 1)}` : cond.text;
    if (!elseProg) { this.addline(`IF ${c},CALL ${thenProg ? this.callName(thenProg) : ''}`); return; }
    this.addline(`IF (${c}) THEN`);
    if (thenProg) this.addline(`CALL ${this.callName(thenProg)}`);
    this.addline('ELSE');
    this.addline(`CALL ${this.callName(elseProg)}`);
    this.addline('ENDIF');
  }
  override ctlThread(name: string): void { this.addline(`RUN ${this.callName(name)}`); }
}

function fanucPost(id: string, name: string, rj3: boolean, aliases: string[]): PostProcessor {
  return {
    id, name, brand: 'Fanuc', extension: 'LS', aliases,
    description: `Fanuc TP program (.LS, RoboDK ${rj3 ? 'Fanuc_RJ3' : 'Fanuc_R30i'} compatible).`,
    generate(p: PostProgram, options: Record<string, any> = {}): PostFile[] { return runRoboPost((ctx) => new FanucPost(ctx, rj3), p, options, true); },
  };
}
export const FANUC_R30iA = fanucPost('Fanuc_R30iA', 'Fanuc R-30iA/iB/iC (LS)', false, ['Fanuc_R30i', 'Fanuc_R30iB', 'Fanuc_R30iC']);
export const FANUC_RJ3 = fanucPost('Fanuc_RJ3', 'Fanuc RJ3 (LS)', true, ['Fanuc_RJ3_DripFeed']);
registerPost(FANUC_R30iA);
registerPost(FANUC_RJ3);
