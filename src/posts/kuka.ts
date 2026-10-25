/**
 * KUKA KRL posts (KR C2 / KR C4 / KR C5), output-compatible with RoboDK's KUKA_KRC2 / KUKA_KRC4 posts
 * (verified token by token against RoboDK 5.9 in tests/posts_gt.test.ts): $BASE/$TOOL frames with optional
 * BASE_DATA/TOOL_DATA index comments, $VEL/$ACC/$APO settings, PTP/LIN/CIRC with C_PTP/C_DIS, external axes E1..E6,
 * named or numbered IO, waits with timeout, program splitting (cascaded calls on KRC2, a <name>Main program on KRC4).
 * Studio extensions: FOR loops, IF on inputs and CONTINUE-free sub-program calls for threads.
 */
import { Mat4 } from '../core/math/pose';
import { PostProcessor, PostProgram, PostFile, registerPost } from './base';
import { RoboPost, RoboPostCtx, runRoboPost, fx, IoName, CtlCondition } from './robopost';
import { pose2Kuka, pose2xyzrpw } from './euler';

const KUKA_HEADER = `

; GLOBAL INTERRUPT DECL 3 WHEN $STOPMESS==TRUE DO IR_STOPM ( )
; INTERRUPT ON 3

;FOLD Initialise and set default speed
BAS (#INITMOV,0)
BAS (#VEL_PTP,100)
BAS (#ACC_PTP,20)
$VEL.CP=0.2
BAS (#TOOL,0)
BAS (#BASE,0)
;ENDFOLD

;;FOLD STARTPOS
;$BWDSTART = FALSE
;PDAT_ACT = PDEFAULT
;BAS(#PTP_DAT)
;FDAT_ACT = {TOOL_NO 0,BASE_NO 0,IPO_FRAME #BASE}
;BAS(#FRAMES)
;;ENDFOLD

$ADVANCE = 5

;FOLD ---- Quickly skip BCO ----
; PTP $AXIS_ACT
;ENDFOLD

;FOLD ---- GO HOME ----
; PTP {A1 0.000, A2 -90.000, A3 90.000, A4 0.000, A5 0.000, A6 0.000, E1 0, E2 0, E3 0, E4 0, E5 0, E6 0}
;ENDFOLD

`;

export const kukaFrame = (m: Mat4): string => { const [x, y, z, a, b, c] = pose2Kuka(m); return `{FRAME: X ${fx(x)},Y ${fx(y)},Z ${fx(z)},A ${fx(a)},B ${fx(b)},C ${fx(c)}}`; };
const AXES = ['A1', 'A2', 'A3', 'A4', 'A5', 'A6', 'E1', 'E2', 'E3', 'E4', 'E5', 'E6'];

interface Part { name: string; lines: string[]; calls: string[]; first: boolean }

export class KukaPost extends RoboPost {
  PROG_EXT = 'src';
  protected nameLen = 16;
  protected parts: Part[] = [];
  protected cur!: Part;
  protected nLines = 0;
  protected zone = -1;
  protected maxLines: number;
  protected loopVar = 0;
  constructor(ctx: RoboPostCtx, protected krc: 2 | 4) {
    super(ctx);
    this.maxLines = ctx.options.maxLines ?? 2500;
  }
  protected safe(name: string): string { return name.replace(/[^A-Za-z0-9_]/g, '').slice(0, this.nameLen) || 'Prog'; }
  protected callName(code: string): string { return code.replace(/ /g, '_').replace(/[^A-Za-z0-9_]/g, '').slice(0, this.nameLen); }
  override addline(s: string): void { this.cur.lines.push(s); this.nLines++; }
  protected addCall(name: string) { if (!this.cur.calls.includes(name)) this.cur.calls.push(name); }

  override ProgStart(name: string): void {
    this.PROG_NAME = this.safe(name);
    this.cur = { name: this.PROG_NAME, lines: [], calls: [], first: true };
    this.parts.push(this.cur);
  }
  override ProgFinish(): void { /* END written by ProgSave */ }
  /** Start a new file when the current one is full (only between motions so no block is cut). */
  protected splitIfNeeded(): void {
    if (this.nLines < this.maxLines) return;
    const next: Part = { name: `${this.PROG_NAME}${this.parts.length}`, lines: [], calls: [], first: false };
    if (this.krc <= 2) {
      this.cur.lines.push('; Continue running program as cascaded execution', `${next.name}()`, '');
      this.cur.calls.push(next.name);
    }
    this.parts.push(next);
    this.cur = next;
    this.nLines = 0;
  }
  override ProgSave(): PostFile[] {
    const files: PostFile[] = [];
    for (const p of this.parts) {
      const L = ['&ACCESS RVP', '&REL 1', '&PARAM TEMPLATE = C:\\KRC\\Roboter\\Template\\vorgabe', '&PARAM EDITMASK = *', `DEF ${p.name} ( )`];
      if (this.krc <= 2 && p.calls.length) L.push('', '; External program calls:', ...p.calls.map((c) => `EXT ${c}()`), '');
      if (p.first) {
        if (this.krc <= 2) L.push('EXT BAS (BAS_COMMAND :IN,REAL :IN )');
        L.push(...KUKA_HEADER.split('\n'));
      }
      L.push(...p.lines, 'END');
      files.push({ name: `${p.name}.src`, content: L.join('\r\n') + '\r\n' });
    }
    if (this.krc > 2 && this.parts.length > 1) {
      const main = `${this.PROG_NAME}Main`;
      files.push({ name: `${main}.src`, content: ['&ACCESS RVP', '&REL 1', '&PARAM TEMPLATE = C:\\KRC\\Roboter\\Template\\vorgabe', '&PARAM EDITMASK = *', `DEF ${main} ( )`, ...this.parts.map((p) => `${p.name}()`), 'END'].join('\r\n') + '\r\n' });
    }
    return files;
  }

  protected axes(joints: number[]): string {
    return joints.slice(0, 12).map((v, i) => `${AXES[i]} ${fx(v, 5)}`).join(',');
  }
  protected extAxes(joints: number[]): string {
    const n = Math.min(this.nAxes, joints.length);
    return n > 6 ? joints.slice(6, n).map((v, i) => `,E${i + 1} ${fx(v, 5)}`).join('') : '';
  }
  protected target(pose: Mat4, joints: number[]): string {
    const [x, y, z, a, b, c] = pose2Kuka(pose);
    return `{X ${fx(x)},Y ${fx(y)},Z ${fx(z)},A ${fx(a)},B ${fx(b)},C ${fx(c)}${this.extAxes(joints)}}`;
  }
  MoveJ(_pose: Mat4 | null, joints: number[]): void {
    this.splitIfNeeded();
    this.addline(`PTP {${this.axes(joints.slice(0, Math.max(6, Math.min(this.nAxes, joints.length))))}}${this.zone >= 0 ? ' C_PTP' : ''}`);
  }
  MoveL(pose: Mat4 | null, joints: number[]): void {
    this.splitIfNeeded();
    if (!pose) { this.MoveJ(null, joints); return; }
    this.addline(`LIN ${this.target(pose, joints)}${this.zone >= 0 ? ' C_DIS' : ''}`);
  }
  override MoveC(pose1: Mat4 | null, joints1: number[], pose2: Mat4 | null, joints2: number[]): void {
    this.splitIfNeeded();
    if (!pose1 || !pose2) { super.MoveC(pose1, joints1, pose2, joints2, [], []); return; }
    this.addline(`CIRC ${this.target(pose1, joints1)},${this.target(pose2, joints2)}${this.zone >= 0 ? ' C_DIS' : ''}`);
  }
  override setFrame(pose: Mat4, frameId: number, name: string): void {
    const fr = kukaFrame(pose);
    this.addline('; ---- Setting reference (Base) ----');
    if (/base$/i.test(name.trim())) this.addline(`$BASE = ${fr}`), this.addline(`; BASE_DATA[1] = ${fr}`), this.addline('; $BASE = BASE_DATA[1]');
    else if (frameId > 0) this.addline(`; BASE_DATA[${frameId}] = ${fr}`), this.addline(`$BASE = ${fr}`), this.addline(`; $BASE = BASE_DATA[${frameId}]`);
    else this.addline(`$BASE = ${fr}`);
    this.addline('; --------------------------');
  }
  override setTool(pose: Mat4, toolId: number): void {
    const fr = kukaFrame(pose);
    this.addline('; ---- Setting tool (TCP) ----');
    if (toolId > 0) this.addline(`; TOOL_DATA[${toolId}] = ${fr}`), this.addline(`$TOOL = ${fr}`), this.addline(`; $TOOL = TOOL_DATA[${toolId}]`);
    else this.addline(`$TOOL = ${fr}`);
    this.addline('; --------------------------');
  }
  override Pause(ms: number): void { this.addline(ms <= 0 ? 'HALT' : `WAIT SEC ${fx(ms / 1000)}`); }
  override setSpeed(v: number): void { this.addline(`$VEL.CP = ${fx(v / 1000, 5)}`); }
  override setAcceleration(a: number): void { this.addline(`$ACC.CP = ${fx(a / 1000, 5)}`); }
  override setSpeedJoints(w: number): void { this.addline(`$VEL.ORI1 = ${fx(w, 5)}`); this.addline(`$VEL.ORI2 = ${fx(w, 5)}`); }
  override setAccelerationJoints(a: number): void { this.addline(`$ACC.ORI1 = ${fx(a, 5)}`); this.addline(`$ACC.ORI2 = ${fx(a, 5)}`); }
  override setZoneData(z: number): void {
    this.zone = z;
    if (z < 0) return;
    this.addline(`$APO.CPTP = ${Math.min(100, Math.trunc(z))}`);
    this.addline(`$APO.CDIS = ${fx(z)}`);
  }
  protected ioOut(io: IoName) { return typeof io === 'number' ? `$OUT[${io}]` : io; }
  protected ioIn(io: IoName) { return typeof io === 'number' ? `$IN[${io}]` : io; }
  protected bool(v: number | boolean | string) { return (typeof v === 'string' ? ['1', 'true', 'on'].includes(v.toLowerCase()) : Number(v) > 0) ? 'TRUE' : 'FALSE'; }
  override setDO(io: IoName, v: number | boolean | string): void { this.addline(`${this.ioOut(io)}=${this.bool(v)}`); }
  override waitDI(io: IoName, v: number | boolean | string, timeoutMs = -1): void {
    const cond = `${this.ioIn(io)}==${this.bool(v)}`;
    if (timeoutMs <= 0) { this.addline(`WAIT FOR (${cond})`); return; }
    for (const l of ['START_TIMER:', '$TIMER_STOP[1]=TRUE', '$TIMER_FLAG[1]=FALSE', `$TIMER[1]=${fx(timeoutMs / 1000)}`, '$TIMER_STOP[1]=FALSE', `WAIT FOR (${cond} OR $TIMER_FLAG[1])`, '$TIMER_STOP[1]=TRUE', 'IF $TIMER_FLAG[1]== TRUE THEN', '    HALT ; Timed out!', '    GOTO START_TIMER', 'ENDIF']) this.addline(l);
  }
  override RunCode(code: string, isCall = false): void {
    if (!isCall) { this.addline(code); return; }
    const m = code.match(/^([^(]*)(\(.*)?$/)!;
    const name = this.callName(m[1].trim());
    this.addCall(name);
    this.addline(`${name}${m[2] ?? '()'}`);
  }
  override RunMessage(msg: string, isComment = false): void {
    if (isComment) { this.addline(`; ${msg}`); return; }
    this.addline('Wait for StrClear($LOOP_MSG[])');
    this.addline('$LOOP_CONT = TRUE');
    this.addline(`$LOOP_MSG[] = "${msg}"`);
  }
  // ---- control flow (KRL) ----
  override ctlLoop(count: number, body: string): void {
    const name = this.callName(body);
    this.addCall(name);
    if (count <= 0) { this.addline('LOOP'); this.addline(`  ${name}()`); this.addline('ENDLOOP'); return; }
    const v = `I_LOOP${++this.loopVar}`;
    this.addline(`DECL INT ${v}`);
    this.addline(`FOR ${v} = 1 TO ${Math.trunc(count)}`);
    this.addline(`  ${name}()`);
    this.addline('ENDFOR');
  }
  override ctlIf(cond: CtlCondition, thenProg: string | null, elseProg: string | null): void {
    const c = cond.io !== undefined ? `${this.ioIn(cond.io)}${cond.op === '!=' ? '<>' : '=='}${this.bool(cond.value ?? 1)}` : cond.text;
    this.addline(`IF ${c} THEN`);
    if (thenProg) { const n = this.callName(thenProg); this.addCall(n); this.addline(`  ${n}()`); }
    if (elseProg) { const n = this.callName(elseProg); this.addCall(n); this.addline('ELSE'); this.addline(`  ${n}()`); }
    this.addline('ENDIF');
  }
}

/** Kept for importers / tests that format KUKA frames. */
export const frameKRL = (m: Mat4) => { const [x, y, z, a, b, c] = pose2Kuka(m); return `{X ${fx(x)},Y ${fx(y)},Z ${fx(z)},A ${fx(a)},B ${fx(b)},C ${fx(c)}}`; };
void pose2xyzrpw;

function kukaPost(id: string, name: string, krc: 2 | 4, aliases: string[]): PostProcessor {
  return {
    id, name, brand: 'KUKA', extension: 'src', aliases,
    description: `KRL for KUKA KR C${krc} controllers (RoboDK ${id} compatible).`,
    generate(p: PostProgram, options: Record<string, any> = {}): PostFile[] { return runRoboPost((ctx) => new KukaPost(ctx, krc), p, options); },
  };
}
export const KUKA_KRC2 = kukaPost('KUKA_KRC2', 'KUKA KRC2 (KRL)', 2, ['KUKA_KRC2_DAT', 'KUKA_KRC1_DAT', 'KUKA_custom', 'KUKA_KRC2_CamRob']);
export const KUKA_KRC4 = kukaPost('KUKA_KRC4', 'KUKA KRC4/KRC5 (KRL)', 4, ['KUKA_KRC4_DAT', 'KUKA_KRC5', 'KUKA_KRC4_Slow']);
registerPost(KUKA_KRC2);
registerPost(KUKA_KRC4);
