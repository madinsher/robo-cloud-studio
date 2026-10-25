/**
 * Vendor posts reproducing RoboDK's own post processors (tests/posts_gt.test.ts compares them token by token with the
 * output of RoboDK's compiled posts on the same events): Kawasaki AS, Comau C5G PDL2, ... Each class mirrors the
 * RoboDK RobotPost callbacks through RoboPost; deliberate deviations from RoboDK (its defects) are listed in the class
 * comments and normalised in tests/posts_gt_util.ts (fixGt).
 */
import { Mat4, multiply } from '../core/math/pose';
import { PostProcessor, PostProgram, PostFile, registerPost } from './base';
import { RoboPost, RoboPostCtx, runRoboPost, fx, pyRound, IoName, CtlCondition, arcPoses } from './robopost';
import { pose2Adept, pose2Staubli, pose2Techman, pose2xyzrpw, pose2Quaternion } from './euler';

const truthy = (v: number | boolean | string) => (typeof v === 'string' ? ['1', 'true', 'on', 'high'].includes(v.toLowerCase()) : Number(v) > 0);
const isIdentity = (m: Mat4) => m.every((v, i) => Math.abs(v - (i % 5 === 0 ? 1 : 0)) < 1e-9);
const callParts = (code: string) => { const m = code.trim().match(/^([^(]*?)\s*(\(.*\))?$/)!; return { name: m[1].replace(/\s+/g, ''), args: m[2] ?? '' }; };

function vendorPost(id: string, name: string, brand: string, extension: string, make: (ctx: RoboPostCtx) => RoboPost, opts: { aliases?: string[]; include?: boolean; description?: string } = {}): PostProcessor {
  const p: PostProcessor = {
    id, name, brand, extension, aliases: opts.aliases ?? [], description: opts.description ?? `${name} (RoboDK ${id} compatible).`,
    generate(prog: PostProgram, options: Record<string, any> = {}): PostFile[] { return runRoboPost(make, prog, options, opts.include ?? false); },
  };
  registerPost(p);
  return p;
}

// ---- Kawasaki AS ---------------------------------------------------------------------------------------------------
/** RoboDK Kawasaki: one .pg per program, TRANS(x,y,z,O,A,T) with Euler ZYZ, C1MOVE/C2MOVE circles, SIGNAL/SWAIT IO. */
export class KawasakiPost extends RoboPost {
  override PROG_EXT = 'pg';
  override addline(s: string): void { this.PROG.push(`  ${s}`); }
  protected trans(m: Mat4) { const [x, y, z, o, a, t] = pose2Adept(m); return `TRANS(${fx(x)},${fx(y)},${fx(z)},${fx(o, 4)},${fx(a, 4)},${fx(t, 4)})`; }
  override ProgStart(name: string): void { this.PROG_NAME = name; this.PROG.push(`.PROGRAM ${name}()`); }
  override ProgFinish(): void { this.addline('BASE NULL'); this.PROG.push('.END'); }
  override ProgSave(name: string): PostFile[] { return [{ name: `${name}.${this.PROG_EXT}`, content: this.PROG.join('\r\n') + '\r\n' }]; }
  MoveJ(_pose: Mat4 | null, joints: number[]): void { this.addline(`JMOVE #[${joints.slice(0, this.nAxes).map((v) => fx(v, 5)).join(', ')}]`); }
  MoveL(pose: Mat4 | null, joints: number[]): void { if (!pose) { this.MoveJ(null, joints); return; } this.addline(`LMOVE ${this.trans(pose)}`); }
  override MoveC(pose1: Mat4 | null, joints1: number[], pose2: Mat4 | null, joints2: number[], conf1: number[], conf2: number[]): void {
    if (!pose1 || !pose2) { super.MoveC(pose1, joints1, pose2, joints2, conf1, conf2); return; }
    this.addline(`C1MOVE ${this.trans(pose1)}`);
    this.addline(`C2MOVE ${this.trans(pose2)}`);
  }
  override setFrame(pose: Mat4): void { this.addline(`BASE ${this.trans(pose)}`); }
  override setTool(pose: Mat4): void { this.addline(`TOOL ${this.trans(pose)}`); }
  override Pause(ms: number): void { this.addline(ms <= 0 ? 'PAUSE' : `TWAIT ${fx(ms / 1000)}`); }
  override setSpeed(v: number): void { this.addline(`SPEED ${fx(v, 1)} MM/S ALWAYS`); }
  override setSpeedJoints(w: number): void { this.addline(`SPEED ${fx(Math.min(100, w / 5), 1)} ALWAYS`); }
  /** RoboDK writes a fixed 100 % acceleration / deceleration. */
  override setAcceleration(): void { this.addline('ACCEL 100.0 100.0'); }
  override setZoneData(z: number): void { this.addline(`ACCURACY ${fx(z)} ALWAYS`); }
  override setDO(io: IoName, v: number | boolean | string): void { this.addline(`SIGNAL ${truthy(v) ? '' : '-'}${io}`); }
  override waitDI(io: IoName, v: number | boolean | string, timeoutMs = -1): void {
    if (timeoutMs > 0) this.addline('; Timeout not implemented');
    this.addline(`WAIT SIG(${truthy(v) ? '' : '-'}${io})`);
  }
  override RunCode(code: string, isCall = false): void { this.addline(isCall ? `CALL ${code.trim()}` : code); }
  override RunMessage(msg: string, isComment = false): void { this.addline(isComment ? `; ${msg}` : `TYPE "${msg}"`); }
  // ---- control flow (AS language) ----
  protected lbl = 0;
  protected sig(cond: CtlCondition): string { return cond.io !== undefined ? `SIG(${(cond.op === '!=') === truthy(cond.value ?? 1) ? '-' : ''}${cond.io})` : cond.text; }
  override ctlLoop(count: number, body: string): void {
    if (count <= 0) { const k = 100 + ++this.lbl; this.PROG.push(`${k}`); this.addline(`CALL ${body}`); this.addline(`GOTO ${k}`); return; }
    this.addline(`FOR loop_i = 1 TO ${Math.trunc(count)}`);
    this.addline(`  CALL ${body}`);
    this.addline('END');
  }
  override ctlIf(cond: CtlCondition, thenProg: string | null, elseProg: string | null): void {
    this.addline(`IF ${this.sig(cond)} THEN`);
    if (thenProg) this.addline(`  CALL ${thenProg}`);
    if (elseProg) { this.addline('ELSE'); this.addline(`  CALL ${elseProg}`); }
    this.addline('END');
  }
  override ctlThread(name: string): void { this.addline(`PCEXECUTE ${name}`); }
}
export const KAWASAKI = vendorPost('Kawasaki', 'Kawasaki (AS)', 'Kawasaki', 'pg', (ctx) => new KawasakiPost(ctx), { description: 'Kawasaki AS program (.pg, RoboDK Kawasaki compatible).' });

// ---- Comau C5G PDL2 -------------------------------------------------------------------------------------------------
interface PdlPart { name: string; lines: string[]; imports: string[]; routines: { name: string; lines: string[] }[] }
const PDL_INIT = ['$ORNT_TYPE := RS_WORLD', '$MOVE_TYPE := JOINT', '$JNT_MTURN := TRUE', '$CNFG_CARE := TRUE', '$TURN_CARE := TRUE', '$SING_CARE := FALSE', '$TERM_TYPE := NOSETTLE', '$FLY_TYPE := FLY_CART', '$FLY_TRAJ := FLY_PASS', '$FLY_DIST := -1.000', '$STRESS_PER:= 65'];
/**
 * RoboDK Comau_C5G: one .pdl with the sub-programs as routines before the main routine, IMPORT of external routines,
 * MOVE / MOVEFLY ... ADVANCE, POS(...,'T1:<J4 turn> T2:<J6 turn> T3:0'), consecutive identical linear targets skipped,
 * program splitting into <name>, <name>1.. plus a <name>Main caller.
 * Deviations from RoboDK (defects): split parts end with their own routine / program name (RoboDK repeats the first
 * one), call arguments keep their commas (RoboDK writes Foo(1,2) as R_Foo(12)), a MoveJ resets the duplicate-target
 * check (RoboDK skips a MoveL back to the last linear target even after a joint move elsewhere).
 */
export class ComauPost extends RoboPost {
  override PROG_EXT = 'pdl';
  protected parts: PdlPart[] = [];
  protected cur!: { lines: string[] };
  protected part!: PdlPart;
  protected first = true;
  protected nLines = 0;
  protected maxLines: number;
  protected fly = -1;
  protected lastLin: string | null = null;
  constructor(ctx: RoboPostCtx) { super(ctx); this.maxLines = ctx.options.maxLines ?? 5000; }
  override addline(s: string): void { this.cur.lines.push(`  ${s}`); this.nLines++; }
  protected pos(m: Mat4, joints: number[] = []) {
    const [x, y, z, a, b, c] = pose2Adept(m);
    const turn = (q: number | undefined) => Math.floor(((q ?? 0) + 180) / 360);
    return `POS(${[x, y, z, a, b, c].map((v) => fx(v, 4)).join(', ')}, ${joints.length ? `'T1:${turn(joints[3])} T2:${turn(joints[5])} T3:0'` : "''"})`;
  }
  protected newPart(name: string): void {
    this.part = { name, lines: [], imports: [], routines: [] };
    this.parts.push(this.part);
    this.cur = this.part;
    this.nLines = 0;
  }
  override ProgStart(name: string): void {
    this.PROG_NAME = name;
    this.lastLin = null;
    if (this.first) { this.first = false; this.newPart(name); }
    else { const r = { name, lines: [] }; this.parts[0].routines.push(r); this.cur = r; }
  }
  protected splitIfNeeded(): void {
    if (this.cur !== this.part || this.nLines < this.maxLines) return;
    this.newPart(`${this.parts[0].name}${this.parts.length}`);
  }
  override ProgSave(name: string): PostFile[] {
    const files: PostFile[] = [];
    const nl = '\r\n';
    for (const p of this.parts) {
      const L = [`PROGRAM ${p.name}`, ...p.imports.map((i) => `IMPORT '${i}'`), 'VAR', `ROUTINE R_${p.name} EXPORTED FROM ${p.name} GLOBAL`, ''];
      for (const r of p.routines) L.push(`ROUTINE R_${r.name}`, 'VAR', 'BEGIN', ...r.lines, `END R_${r.name}`, '');
      L.push(`ROUTINE R_${p.name}`, 'VAR', 'BEGIN', ...PDL_INIT.map((l) => `  ${l}`), ...p.lines, `END R_${p.name}`, '', 'BEGIN', `  R_${p.name}`, `END ${p.name}`, '');
      files.push({ name: `${p.name}.${this.PROG_EXT}`, content: L.join(nl) + nl });
    }
    if (this.parts.length > 1) {
      const m = `${name}Main`;
      const L = [`PROGRAM ${m}`, ...this.parts.map((p) => `IMPORT '${p.name}'`), 'CONST', 'VAR', 'BEGIN', ...this.parts.map((p) => `  R_${p.name}()`), `END ${m}`];
      files.push({ name: `${m}.${this.PROG_EXT}`, content: L.join(nl) + nl });
    }
    return files;
  }
  MoveJ(_pose: Mat4 | null, joints: number[]): void {
    this.splitIfNeeded();
    this.lastLin = null;
    this.addline(`MOVE TO {${joints.slice(0, this.nAxes).map((v) => fx(v, 5)).join(',')}}`);
  }
  MoveL(pose: Mat4 | null, joints: number[]): void {
    if (!pose) { this.MoveJ(null, joints); return; }
    const p = this.pos(pose, joints);
    const key = p.replace(/, '[^']*'\)$/, '');
    if (key === this.lastLin) return;
    this.splitIfNeeded();
    this.lastLin = key;
    this.addline(this.fly > 0 ? `MOVEFLY LINEAR TO ${p} ADVANCE` : `MOVE LINEAR TO ${p}`);
  }
  override MoveC(pose1: Mat4 | null, joints1: number[], pose2: Mat4 | null, joints2: number[], conf1: number[], conf2: number[]): void {
    if (!pose1 || !pose2) { super.MoveC(pose1, joints1, pose2, joints2, conf1, conf2); return; }
    this.splitIfNeeded();
    const end = this.pos(pose2, joints2);
    this.lastLin = end.replace(/, '[^']*'\)$/, '');
    this.addline(`${this.fly > 0 ? 'MOVEFLY' : 'MOVE'} CIRCULAR TO ${end} VIA ${this.pos(pose1, joints1)}${this.fly > 0 ? ' ADVANCE' : ''}`);
  }
  override setFrame(pose: Mat4): void { this.addline(`$UFRAME := ${this.pos(pose)}`); }
  override setTool(pose: Mat4): void { this.addline(`$TOOL := ${this.pos(pose)}`); }
  override Pause(ms: number): void { this.addline(ms <= 0 ? 'PAUSE' : `DELAY ${Math.trunc(ms)}`); }
  override setSpeed(v: number): void { this.addline('$SPD_OPT := SPD_LIN'); this.addline(`$LIN_SPD := ${fx(v / 1000)}`); }
  override setSpeedJoints(w: number): void { this.addline(`$ROT_SPD := ${fx((w * Math.PI) / 180)}`); }
  override setZoneData(z: number): void { this.fly = z; this.addline(`$FLY_DIST := ${fx(z)}`); }
  protected onoff(v: number | boolean | string) { return truthy(v) ? 'ON' : 'OFF'; }
  override setDO(io: IoName, v: number | boolean | string): void { this.addline(`${typeof io === 'number' ? `$DOUT[${io}]` : io} := ${this.onoff(v)}`); }
  override waitDI(io: IoName, v: number | boolean | string, timeoutMs = -1): void {
    const c = `(${typeof io === 'number' ? `$DIN[${io}]` : io} = ${this.onoff(v)})`;
    this.addline('$TIMER[1] := 0');
    this.addline('REPEAT');
    this.addline('  DELAY 1');
    if (timeoutMs <= 0) { this.addline(`UNTIL ${c}`); return; }
    const t = fx(timeoutMs, 1);
    this.addline(`UNTIL ${c} OR ($TIMER[1] > ${t})`);
    this.addline(`IF $TIMER[1] > ${t} THEN`);
    this.addline('-- TIMED OUT! Important: This section must be updated accordingly');
    this.addline('  DELAY 2000');
    this.addline('ENDIF');
  }
  override RunCode(code: string, isCall = false): void {
    if (!isCall) { this.addline(code); return; }
    const { name, args } = callParts(code);
    if (!this.ctx.programs.includes(name) && !this.part.imports.includes(name)) this.part.imports.push(name);
    this.addline(`R_${name}${args}`);
  }
  override RunMessage(msg: string): void { this.addline(`-- ${msg}`); }
  // ---- control flow (PDL2) ----
  protected cond(c: CtlCondition): string { return c.io !== undefined ? `${typeof c.io === 'number' ? `$DIN[${c.io}]` : c.io} ${c.op === '!=' ? '<>' : '='} ${this.onoff(c.value ?? 1)}` : c.text; }
  protected callName(n: string) { if (!this.ctx.programs.includes(n) && !this.part.imports.includes(n)) this.part.imports.push(n); return `R_${n}`; }
  override ctlLoop(count: number, body: string): void {
    const r = this.callName(body);
    if (count <= 0) { this.addline('REPEAT'); this.addline(`  ${r}`); this.addline('UNTIL FALSE'); return; }
    this.addline(`FOR $INT_VAR[1] := 1 TO ${Math.trunc(count)} DO`);
    this.addline(`  ${r}`);
    this.addline('ENDFOR');
  }
  override ctlIf(cond: CtlCondition, thenProg: string | null, elseProg: string | null): void {
    this.addline(`IF ${this.cond(cond)} THEN`);
    if (thenProg) this.addline(`  ${this.callName(thenProg)}`);
    if (elseProg) { this.addline('ELSE'); this.addline(`  ${this.callName(elseProg)}`); }
    this.addline('ENDIF');
  }
}
export const COMAU_C5G = vendorPost('Comau_C5G', 'Comau C5G (PDL2)', 'Comau', 'pdl', (ctx) => new ComauPost(ctx), { include: true, description: 'Comau C5G PDL2 program (RoboDK Comau_C5G compatible).' });

// ---- Mecademic (Meca500 program text) -------------------------------------------------------------------------------
/**
 * RoboDK Mecademic: MoveJoints / MoveLin (XYZ mobile Euler), SetWRF / SetTRF, velocities and accelerations clamped to the
 * Meca500 limits, calls as StartProgram(<n>) (number taken from the call or allocated in call order until a numbered
 * call is seen), native GripperOpen / GripperClose. IO and pendant messages are not available on the Meca500.
 * Deviation from RoboDK (defect): RoboDK drops circular moves; the studio writes them as linear moves along the arc
 * (marked by a "// Circular move approximated with N linear moves" comment).
 */
export class MecademicPost extends RoboPost {
  override PROG_EXT = 'txt';
  protected calls = new Map<string, number>();
  protected explicitCalls = false;
  protected last: Mat4 | null = null;
  override ProgSave(name: string): PostFile[] { return [{ name: `${name}.${this.PROG_EXT}`, content: this.PROG.join('\r\n') + '\r\n' }]; }
  protected xyzabc(m: Mat4) { return pose2Staubli(m).map((v) => fx(v)).join(', '); }
  MoveJ(pose: Mat4 | null, joints: number[]): void { this.addline(`MoveJoints(${joints.slice(0, this.nAxes).map((v) => fx(v, 6)).join(', ')})`); this.last = pose; }
  MoveL(pose: Mat4 | null, joints: number[]): void { if (!pose) { this.MoveJ(null, joints); return; } this.addline(`MoveLin(${this.xyzabc(pose)})`); this.last = pose; }
  override MoveC(pose1: Mat4 | null, joints1: number[], pose2: Mat4 | null, joints2: number[], conf1: number[], conf2: number[]): void {
    const arc = this.last && pose1 && pose2 ? arcPoses(this.last, pose1, pose2) : null;
    if (!arc) { super.MoveC(pose1, joints1, pose2, joints2, conf1, conf2); return; }
    this.addlog('Circular move written as linear moves along the arc (no circle instruction on the Meca500)');
    this.addline(`// Circular move approximated with ${arc.poses.length} linear moves (R${fx(arc.radius, 1)} mm)`);
    for (const p of arc.poses) this.addline(`MoveLin(${this.xyzabc(p)})`);
    this.last = pose2;
  }
  override setFrame(pose: Mat4): void { this.addline(`SetWRF(${this.xyzabc(pose)})`); }
  override setTool(pose: Mat4): void { this.addline(`SetTRF(${this.xyzabc(pose)})`); }
  override Pause(ms: number): void { if (ms < 0) { this.addlog('Stop not supported'); return; } this.addline(`Delay(${fx(ms / 1000)})`); }
  override setSpeed(v: number): void { this.addline(`SetCartLinVel(${fx(Math.min(1000, v))})`); }
  override setSpeedJoints(w: number): void { this.addline(`SetJointVel(${fx(Math.min(100, w))})`); }
  override setAcceleration(a: number): void { this.addline(`SetCartAcc(${fx(Math.min(600, a))})`); }
  override setAccelerationJoints(a: number): void { this.addline(`SetJointAcc(${fx(Math.min(150, a))})`); }
  override setZoneData(z: number): void { this.addline(`SetBlending(${fx(Math.max(0, Math.min(100, z)))})`); }
  override setDO(io: IoName, v: number | boolean | string): void { const t = `Setting Digital Outputs is not implemented (${io}=${truthy(v) ? 1 : 0})`; this.addlog(t); this.addline(`// ${t}`); }
  override waitDI(io: IoName, v: number | boolean | string): void { const t = `Waiting for Digital Input is not implemented (${io}==${truthy(v) ? 1 : 0})`; this.addlog(t); this.addline(`// ${t}`); }
  override RunCode(code: string, isCall = false): void {
    if (!isCall) { this.addline(code); return; }
    const c = code.trim();
    if (c === 'GripperOpen' || c === 'GripperClose') { this.addline(c); return; }
    const nums = c.match(/\d+/g);
    const n = nums ? Number(nums[nums.length - 1]) : 0;
    if (n >= 1 && n <= 999) {
      this.explicitCalls = true;
      if (c !== `Prog${n}`) this.addline(`// ${c}`);
      this.addline(`StartProgram(${n})`);
      return;
    }
    if (this.explicitCalls) {
      const t = `${c}: Calling programs by name is not supported. Specify program number (e.g. "${c} 5") or inline programs by changing this setting: Tools-Options-Program-Inline programs.`;
      this.addlog(t);
      this.addline(`// ${t}`);
      return;
    }
    if (!this.calls.has(c)) this.calls.set(c, this.calls.size + 1);
    this.addline(`// ${c}`);
    this.addline(`StartProgram(${this.calls.get(c)})`);
  }
  override RunMessage(msg: string, isComment = false): void { if (isComment) this.addline(`// ${msg}`); else this.addlog(`Show message on teach pendant not implemented (${msg})`); }
}
export const MECADEMIC = vendorPost('Mecademic', 'Mecademic Meca500 (program)', 'Mecademic', 'txt', (ctx) => new MecademicPost(ctx), { aliases: ['Mecademic_program'], description: 'Mecademic Meca500 program text (RoboDK Mecademic compatible).' });

// ---- RoboDK CSV ----------------------------------------------------------------------------------------------------
/** RoboDK's CSV post: one row per instruction (poses as X,Y,Z,Rx,Ry,Rz = TxyzRxyz in degrees, joints with 6 decimals). */
export class CsvPost extends RoboPost {
  override PROG_EXT = 'csv';
  override ProgStart(name: string): void {
    this.PROG_NAME = name;
    const j = Array.from({ length: this.nAxes }, (_, i) => `J${i + 1} (deg)`);
    this.addline(['Instruction', 'X (mm)', 'Y (mm)', 'Z (mm)', 'Rx (deg)', 'Ry (deg)', 'Rz (deg)', ...j].join(','));
    this.addline(`Program Start,${name}`);
  }
  override ProgFinish(name: string): void { this.addline(`Program Finish,${name}`); }
  override ProgSave(name: string): PostFile[] { return [{ name: `${name}.${this.PROG_EXT}`, content: this.PROG.join('\r\n') + '\r\n' }]; }
  protected p6(m: Mat4 | null) { return (m ? pose2Staubli(m) : [0, 0, 0, 0, 0, 0]).map((v) => fx(v)).join(','); }
  protected row(kind: string, pose: Mat4 | null, joints: number[]) { this.addline(`${kind},${this.p6(pose)},${joints.slice(0, this.nAxes).map((v) => fx(v, 6)).join(',')}`); }
  MoveJ(pose: Mat4 | null, joints: number[]): void { this.row('Move Joints', pose, joints); }
  MoveL(pose: Mat4 | null, joints: number[]): void { this.row('Move Linear', pose, joints); }
  override MoveC(pose1: Mat4 | null, joints1: number[], pose2: Mat4 | null, joints2: number[]): void { this.row('Move Circular (1)', pose1, joints1); this.row('Move Circular (2)', pose2, joints2); }
  override setFrame(pose: Mat4): void { this.addline(`Set Reference,${this.p6(pose)}`); }
  override setTool(pose: Mat4): void { this.addline(`Set Tool,${this.p6(pose)}`); }
  override Pause(ms: number): void { this.addline(ms < 0 ? 'Stop' : `Pause (ms),${fx(ms)}`); }
  override setSpeed(v: number): void { this.addline(`Set Speed (mm/s),${fx(v)}`); }
  override setSpeedJoints(w: number): void { this.addline(`Set Joint Speed (deg/s),${fx(w)}`); }
  override setAcceleration(a: number): void { this.addline(`Set Acceleration (mm/s2),${fx(a)}`); }
  override setAccelerationJoints(a: number): void { this.addline(`Set Joint Acceleration (deg/s2),${fx(a)}`); }
  override setZoneData(z: number): void { this.addline(`Set Rounding,${fx(z)}`); }
  override setDO(io: IoName, v: number | boolean | string): void { this.addline(`Set DO,${typeof io === 'number' ? `OUT[${io}]` : io},${truthy(v) ? 1 : 0}`); }
  override waitDI(io: IoName, v: number | boolean | string, timeoutMs = -1): void { this.addline(`Wait DI,${typeof io === 'number' ? `IN[${io}]` : io},${truthy(v) ? 1 : 0}${timeoutMs > 0 ? `,${fx(timeoutMs, 1)}` : ''}`); }
  override RunCode(code: string, isCall = false): void { this.addline(isCall ? `Run Program,${code}` : code); }
  override RunMessage(msg: string, isComment = false): void { this.addline(`${isComment ? 'Comment' : 'Show Message'},${msg}`); }
}
export const CSV_POST = vendorPost('CSV', 'RoboDK CSV', 'Generic', 'csv', (ctx) => new CsvPost(ctx), { description: 'One CSV row per instruction (RoboDK CSV post compatible).' });

// ---- Doosan Robotics DRL --------------------------------------------------------------------------------------------
const DOOSAN_HEADER: string[] = ["  ", "  #--------------------------", "  # Add your default header and subprograms here", "  # For example, to drive a gripper as a program call:", "  # def Gripper_Open():", "  #   set_digital_output(3, 1)", "  #", "  # def Gripper_Close():", "  #   set_digital_output(3, 0)", "  #", "  # Example to drive a spray gun:", "  def SprayOn(value):", "    # use the value as an output:", "    DO_SPRAY = 5", "    if value == 0:", "      set_digital_output(DO_SPRAY, 0)", "    else:", "      set_digital_output(DO_SPRAY, 1)", "", "  #--------------------------", "  ", "  "];
/**
 * RoboDK Doosan_Robotics: DRL (Python) program per file, set_ref_coord(set_user_cart_coord(posx(...))) frames, set_tcp by
 * name, posx with Euler ZYZ, posj, movec, begin_blend / end_blend, set_velx / set_velj / set_accx / set_accj,
 * set_digital_output / wait_digital_input, tp_popup.
 * Deviation from RoboDK (defect): RoboDK ends wait_digital_input(...) with a ":" (a DRL syntax error); the studio does not.
 */
export class DoosanPost extends RoboPost {
  override PROG_EXT = 'drl';
  override addline(s: string): void { this.PROG.push(`  ${s}`); }
  override ProgStart(name: string): void { this.PROG_NAME = name; this.PROG.push(`def ${name}():`, ...DOOSAN_HEADER, '  # Main program:'); }
  override ProgFinish(name: string): void { this.PROG.push('  # End of main program', '  ', `${name}()`); }
  override ProgSave(name: string): PostFile[] { return [{ name: `${name}.${this.PROG_EXT}`, content: this.PROG.join('\r\n') + '\r\n' }]; }
  protected posx(m: Mat4) { const [x, y, z, a, b, c] = pose2Adept(m); return `posx(${fx(x)},${fx(y)},${fx(z)},${fx(a, 4)},${fx(b, 4)},${fx(c, 4)})`; }
  MoveJ(_pose: Mat4 | null, joints: number[]): void { this.addline(`movej(posj(${joints.slice(0, this.nAxes).map((v) => fx(v, 6)).join(',')}))`); }
  MoveL(pose: Mat4 | null, joints: number[]): void { if (!pose) { this.MoveJ(null, joints); return; } this.addline(`movel(${this.posx(pose)})`); }
  override MoveC(pose1: Mat4 | null, joints1: number[], pose2: Mat4 | null, joints2: number[], conf1: number[], conf2: number[]): void {
    if (!pose1 || !pose2) { super.MoveC(pose1, joints1, pose2, joints2, conf1, conf2); return; }
    this.addline(`movec(${this.posx(pose1)},${this.posx(pose2)})`);
  }
  override setFrame(pose: Mat4): void { this.addline(`set_ref_coord(set_user_cart_coord(${this.posx(pose)},ref=DR_BASE))`); }
  override setTool(pose: Mat4, _id: number, name: string): void { this.addline(`# TCP: ${this.posx(pose)}`); this.addline(`set_tcp("${name}")`); }
  override Pause(ms: number): void { this.addline(ms <= 0 ? 'halt() # reimplement this function to force stop' : `wait(${fx(ms / 1000)})`); }
  override setSpeed(v: number): void { this.addline(`set_velx(${fx(v)})`); }
  override setSpeedJoints(w: number): void { this.addline(`set_velj(${fx(w)})`); }
  override setAcceleration(a: number): void { this.addline(`set_accx(${fx(a)})`); }
  override setAccelerationJoints(a: number): void { this.addline(`set_accj(${fx(a)})`); }
  override setZoneData(z: number): void { this.addline(z < 0 ? 'end_blend()' : `begin_blend(radius=${fx(z)})`); }
  override setDO(io: IoName, v: number | boolean | string): void { this.addline(typeof io === 'number' ? `set_digital_output(${io}, ${truthy(v) ? 1 : 0})` : `${io} = ${truthy(v) ? 1 : 0}`); }
  override waitDI(io: IoName, v: number | boolean | string, timeoutMs = -1): void { this.addline(`wait_digital_input(${io},${truthy(v) ? 1 : 0}${timeoutMs > 0 ? `,${fx(timeoutMs / 1000)}` : ''})`); }
  override RunCode(code: string, isCall = false): void { this.addline(isCall && !code.includes('(') ? `${code.replace(/ /g, '')}()` : code); }
  override RunMessage(msg: string, isComment = false): void { this.addline(isComment ? `# ${msg}` : `tp_popup("${msg}", DR_PM_MESSAGE)`); }
  // ---- control flow (DRL is Python) ----
  override ctlLoop(count: number, body: string): void {
    if (count <= 0) { this.addline('while True:'); this.addline(`  ${body}()`); return; }
    this.addline(`for _i in range(${Math.trunc(count)}):`);
    this.addline(`  ${body}()`);
  }
  override ctlIf(cond: CtlCondition, thenProg: string | null, elseProg: string | null): void {
    const c = cond.io !== undefined ? `get_digital_input(${cond.io}) ${cond.op === '!=' ? '!=' : '=='} ${truthy(cond.value ?? 1) ? 1 : 0}` : cond.text;
    this.addline(`if ${c}:`);
    this.addline(thenProg ? `  ${thenProg}()` : '  pass');
    if (elseProg) { this.addline('else:'); this.addline(`  ${elseProg}()`); }
  }
  override ctlThread(name: string): void { this.addline(`thread_run(${name}, loop=False)`); }
}
export const DOOSAN = vendorPost('Doosan_Robotics', 'Doosan Robotics (DRL)', 'Doosan', 'drl', (ctx) => new DoosanPost(ctx), { aliases: ['Doosan'], description: 'Doosan DRL program (RoboDK Doosan_Robotics compatible).' });

// ---- Techman / Omron TM (TMscript) ----------------------------------------------------------------------------------
/** RoboDK Techman_TMscript: "main { ... Exit() }", PTP / Line / Circle in Cartesian (CPP) with Pose_2_Techman angles. */
export class TechmanPost extends RoboPost {
  override PROG_EXT = 'scpt';
  protected speed = 100;
  protected blend = 0;
  protected precise = true;
  override addline(s: string): void { this.PROG.push(`  ${s}`); }
  override ProgStart(name: string): void { this.PROG_NAME = name; this.PROG.push('main {'); }
  override ProgFinish(): void { this.addline('Exit()'); this.PROG.push('}'); }
  override ProgSave(name: string): PostFile[] { return [{ name: `${name}.${this.PROG_EXT}`, content: this.PROG.join('\r\n') + '\r\n' }]; }
  protected p6(m: Mat4) { return pose2Techman(m).map((v) => fx(v)).join(', '); }
  protected tail() { return `${this.blend},${this.precise ? 'True' : 'False'})`; }
  MoveJ(pose: Mat4 | null, joints: number[]): void {
    if (!pose) { this.addline(`PTP("JPP", ${joints.slice(0, 6).map((v) => fx(v)).join(', ')},100,200,${this.tail()}`); return; }
    this.addline(`PTP("CPP", ${this.p6(pose)},100,200,${this.tail()}`);
  }
  MoveL(pose: Mat4 | null, joints: number[]): void { if (!pose) { this.MoveJ(null, joints); return; } this.addline(`Line("CPP", ${this.p6(pose)},${this.speed},200,${this.tail()}`); }
  override MoveC(pose1: Mat4 | null, joints1: number[], pose2: Mat4 | null, joints2: number[], conf1: number[], conf2: number[]): void {
    if (!pose1 || !pose2) { super.MoveC(pose1, joints1, pose2, joints2, conf1, conf2); return; }
    this.addline(`Circle("CPP", ${this.p6(pose1)}, ${this.p6(pose2)}, ${this.speed}, 200, ${this.blend}, 0, ${this.precise ? 'True' : 'False'}) `);
  }
  override setFrame(pose: Mat4): void { this.addline(`ChangeBase(${this.p6(pose)})`); }
  override setTool(pose: Mat4): void { this.addline(`ChangeTCP(${this.p6(pose)})`); }
  override Pause(ms: number): void { this.addline(ms < 0 ? 'Pause()' : `WaitFor( ${fx(ms)})`); }
  override setSpeed(v: number): void { this.speed = Math.min(100, v); }
  override setZoneData(z: number): void { this.blend = Math.trunc(z); this.precise = z < 0; }
  override setDO(io: IoName, v: number | boolean | string): void { this.addline(`${typeof io === 'number' ? `IO["ControlBox"].DO[${io}]` : io}=${truthy(v) ? 1 : 0}`); }
  override waitDI(io: IoName, v: number | boolean | string, timeoutMs = -1): void { this.addline(`WaitFor(${typeof io === 'number' ? `IO["ControlBox"].DI[${io}]` : io}==${truthy(v) ? 1 : 0}, ${timeoutMs > 0 ? fx(timeoutMs, 1) : '-1'})`); }
  override RunCode(code: string, isCall = false): void { this.addline(isCall && !code.includes('(') ? `${code.replace(/ /g, '')}()` : code); }
  override RunMessage(msg: string, isComment = false): void { this.addline(isComment ? `// ${msg}` : `Display("${msg}")`); }
}
export const TECHMAN = vendorPost('Techman_TM', 'Techman / Omron TM (TMscript)', 'Techman', 'scpt', (ctx) => new TechmanPost(ctx), { aliases: ['Techman_TMscript', 'Techman', 'Omron'], description: 'Omron / Techman TMscript (RoboDK Techman_TMscript compatible).' });

// ---- Denso RC8 (PacScript) -------------------------------------------------------------------------------------------
/**
 * RoboDK Denso_RC8: PacScript with Work / Tool definitions + ChangeWork / ChangeTool (P(x,y,z,rx,ry,rz) =
 * pose_2_xyzrpw), MOVE P / L / C with @P (pass) or @0, joint speed in % (deg/s / 2) and Mps() linear speed.
 * Deviation from RoboDK (defect): call arguments keep their commas (RoboDK writes X(1,2) as X(12)).
 */
export class DensoPost extends RoboPost {
  override PROG_EXT = 'pcs';
  protected speed = 100;
  protected jspeed = 20;
  protected pass = '@0';
  override addline(s: string): void { this.PROG.push(`  ${s}`); }
  override ProgStart(name: string): void { this.PROG_NAME = name; this.PROG.push(`'!TITLE \u201D${name}\u201D`, '', 'Sub Main', '  TAKEARM Keep = 0', ''); }
  override ProgFinish(): void { this.PROG.push('End Sub'); }
  override ProgSave(name: string): PostFile[] { return [{ name: `${name}.${this.PROG_EXT}`, content: this.PROG.join('\r\n') + '\r\n' }]; }
  protected p(m: Mat4) { return `P(${pose2xyzrpw(m).map((v) => fx(v)).join(',')})`; }
  MoveJ(_pose: Mat4 | null, joints: number[]): void { this.addline(`MOVE P, ${this.pass} J(${joints.slice(0, this.nAxes).map((v) => fx(v, 5)).join(',')}), Speed=${this.jspeed}`); }
  MoveL(pose: Mat4 | null, joints: number[]): void { if (!pose) { this.MoveJ(null, joints); return; } this.addline(`MOVE L, ${this.pass} ${this.p(pose)}, Speed=Mps(${fx(this.speed, 2)})`); }
  override MoveC(pose1: Mat4 | null, joints1: number[], pose2: Mat4 | null, joints2: number[], conf1: number[], conf2: number[]): void {
    if (!pose1 || !pose2) { super.MoveC(pose1, joints1, pose2, joints2, conf1, conf2); return; }
    this.addline(`MOVE C, ${this.p(pose1)},${this.pass} ${this.p(pose2)}, Speed=Mps(${fx(this.speed, 2)})`);
  }
  override setFrame(pose: Mat4, id: number): void { const k = id > 0 ? id : 3; this.addline(`Work ${k}, ${this.p(pose)}`); this.addline(`ChangeWork ${k}`); }
  override setTool(pose: Mat4, id: number): void { const k = id > 0 ? id : 2; this.addline(`Tool ${k}, ${this.p(pose)}`); this.addline(`ChangeTool ${k}`); }
  override Pause(ms: number): void { this.addline(ms <= 0 ? 'PAUSE' : `DELAY ${Math.trunc(ms)}`); }
  override setSpeed(v: number): void { this.speed = v; }
  override setSpeedJoints(w: number): void { this.jspeed = Math.min(100, Math.trunc(w / 2)); }
  override setZoneData(z: number): void { this.pass = z > 0 ? '@P' : '@0'; }
  override setDO(io: IoName, v: number | boolean | string): void { this.addline(`OUT ${typeof io === 'number' ? `IO[${io}]` : io} = ${truthy(v) ? 'ON' : 'OFF'}`); }
  override waitDI(io: IoName, v: number | boolean | string, timeoutMs = -1): void { this.addline(`WAIT ${typeof io === 'number' ? `IO[${io}]` : io} = ${truthy(v) ? 'ON' : 'OFF'}${timeoutMs > 0 ? `, ${Math.trunc(timeoutMs)}` : ''}`); }
  override RunCode(code: string, isCall = false): void { if (!isCall) { this.addline(code); return; } const { name, args } = callParts(code); this.addline(`Call ${name}${args || '()'}`); }
  override RunMessage(msg: string, isComment = false): void { this.addline(isComment ? `' ${msg}` : `'Output "${msg}"`); }
}
export const DENSO_RC8 = vendorPost('Denso_RC8', 'Denso RC8 (PacScript)', 'Denso', 'pcs', (ctx) => new DensoPost(ctx), { aliases: ['Denso_RC7', 'Denso_RC9'], description: 'Denso PacScript (RoboDK Denso_RC8 compatible).' });

// ---- Epson RC+ (SPEL+) ----------------------------------------------------------------------------------------------
/**
 * RoboDK Epson_RC: SPEL+ Function with Local / TLSet frames (XY(x,y,z,u,v,w): u,v,w about Z,Y,X), Go JA / Move XY /lN
 * with CP when blending, Arc via / end, SpeedS / Speed / Accel, On / Off / Wait In().
 * Deviations from RoboDK (defects): a program stop is written as "Pause" (RoboDK writes "Wait -0.001"), call arguments
 * keep their commas.
 */
export class EpsonPost extends RoboPost {
  override PROG_EXT = 'prg';
  protected local = 0;
  protected cp = false;
  override addline(s: string): void { this.PROG.push(`    ${s}`); }
  override ProgStart(name: string): void { this.PROG_NAME = name; this.PROG.push(`Function ${name}`, '    Motor On', '    Power High'); }
  override ProgFinish(): void { this.PROG.push('Fend'); }
  override ProgSave(name: string): PostFile[] { return [{ name: `${name}.${this.PROG_EXT}`, content: this.PROG.join('\r\n') + '\r\n' }]; }
  protected xy(m: Mat4) { const [x, y, z, r, p, w] = pose2xyzrpw(m); return `XY(${[x, y, z, w, p, r].map((v) => fx(v)).join(', ')})`; }
  MoveJ(_pose: Mat4 | null, joints: number[]): void { this.addline(`Go JA(${joints.slice(0, this.nAxes).map((v) => fx(v)).join(',')})${this.cp ? ' CP' : ''}`); }
  MoveL(pose: Mat4 | null, joints: number[]): void { if (!pose) { this.MoveJ(null, joints); return; } this.addline(`Move ${this.xy(pose)}${this.local ? ` /${this.local}` : ''}${this.cp ? ' CP' : ''}`); }
  override MoveC(pose1: Mat4 | null, joints1: number[], pose2: Mat4 | null, joints2: number[], conf1: number[], conf2: number[]): void {
    if (!pose1 || !pose2) { super.MoveC(pose1, joints1, pose2, joints2, conf1, conf2); return; }
    this.addline(`Arc ${this.xy(pose1)}, ${this.xy(pose2)} `);
  }
  override setFrame(pose: Mat4, id: number, name: string): void {
    this.addline(`' Using reference: ${name}`);
    if (isIdentity(pose)) { this.local = 0; this.addline("' Exporting targets with respect to the robot base"); return; }
    this.local = id > 0 ? id : 1;
    this.addline(`Local ${this.local}, ${this.xy(pose)}`);
  }
  override setTool(pose: Mat4, id: number, name: string): void { const k = id > 0 ? id : 1; this.addline(`' Using Tool frame ${name}: ${this.xy(pose)}`); this.addline(`TLSet ${k}, ${this.xy(pose)}`); this.addline(`Tool ${k}`); }
  override Pause(ms: number): void { this.addline(ms < 0 ? 'Pause' : `Wait ${fx(ms / 1000)}`); }
  override setSpeed(v: number): void { this.addline(`SpeedS ${fx(v, 2)}`); }
  override setSpeedJoints(w: number): void { this.addline(`Speed ${fx(w, 2)}`); }
  override setAccelerationJoints(a: number): void { this.addline(`Accel ${fx(a, 2)}`); }
  override setZoneData(z: number): void { this.cp = z > 0; }
  override setDO(io: IoName, v: number | boolean | string): void { this.addline(`${truthy(v) ? 'On' : 'Off'} ${io}`); }
  override waitDI(io: IoName, v: number | boolean | string, timeoutMs = -1): void { this.addline(`Wait ${typeof io === 'number' ? `In(${io})` : io} = ${truthy(v) ? 'On' : 'Off'}${timeoutMs > 0 ? ` Timeout = ${fx(timeoutMs, 1)}` : ''}`); }
  override RunCode(code: string, isCall = false): void { if (!isCall) { this.addline(`Xqt 2, ${code}`); return; } const { name, args } = callParts(code); this.addline(`Call ${name}${args}`); }
  override RunMessage(msg: string, isComment = false): void { this.addline(isComment ? `' ${msg}` : `Print "${msg}"`); }
}
export const EPSON_RC = vendorPost('Epson_RC', 'Epson RC+ (SPEL+)', 'Epson', 'prg', (ctx) => new EpsonPost(ctx), { description: 'Epson SPEL+ program (RoboDK Epson_RC compatible).' });

// ---- Nachi AX / FD ---------------------------------------------------------------------------------------------------
/**
 * RoboDK Nachi_AX_FD: file <robot model>-01-A.<program number> (number from the program name, default 005), MOVEX
 * M1J / M1X L / C1 C2 with S= speed, H= tool, SM= smoothing (rounding / 25) or A=1P (fine), CHGCOORD n for numbered
 * frames ("U" positions) and targets converted to the robot base otherwise, SET / RESET / WAITI / WAITJ, CALLP, program
 * splitting into numbered parts called by the main number.
 */
export class NachiPost extends RoboPost {
  override PROG_EXT = '';
  protected parts: string[][] = [[]];
  protected nLines = 0;
  protected maxLines: number;
  protected speed = 500;
  protected tool = 1;
  protected sm = '';
  protected coord = 0;
  protected frame: Mat4 | null = null;
  constructor(ctx: RoboPostCtx) { super(ctx); this.maxLines = ctx.options.maxLines ?? 9999; }
  override addline(s: string): void { this.parts[this.parts.length - 1].push(s); this.nLines++; }
  protected num(name: string): number { const m = name.match(/(\d+)(?!.*\d)/); return m ? +m[1] : 5; }
  protected file(n: number) { return `${(this.ROBOT_NAME.trim().split(/\s+/).pop() ?? 'Robot')}-01-A.${String(n).padStart(3, '0')}`; }
  override ProgStart(name: string): void { this.PROG_NAME = name; this.parts = [[]]; this.nLines = 0; }
  override ProgFinish(): void { this.parts[this.parts.length - 1].push('END'); }
  override ProgSave(name: string): PostFile[] {
    const n = this.num(name), nl = '\r\n';
    if (this.parts.length === 1) return [{ name: this.file(n), content: this.parts[0].join(nl) + nl }];
    const files = this.parts.map((p, i) => ({ name: this.file(n + 1 + i), content: [...p, ...(i < this.parts.length - 1 ? ['END', `' Program ${name}`] : [])].join(nl) + nl }));
    files.unshift({ name: this.file(n), content: [`' Main program ${name} calls ${this.parts.length} subprograms`, ...this.parts.map((_p, i) => `CALLP [${String(n + 1 + i).padStart(3, '0')}]`), 'END'].join(nl) + nl });
    return files;
  }
  protected move(s: string): void {
    if (this.nLines >= this.maxLines) { this.parts.push([]); this.nLines = 0; }
    this.addline(s);
  }
  protected p(m: Mat4): string {
    const pose = this.coord === 0 && this.frame ? multiply(this.frame, m) : m;
    const [x, y, z, r, p, w] = pose2xyzrpw(pose);
    return `(${[x, y, z, w, p, r].map((v) => fx(v)).join(',')})${this.coord ? 'U' : ''}`;
  }
  protected tail() { return `S=${fx(this.speed, 2)},H=${this.tool},MS`; }
  MoveJ(_pose: Mat4 | null, joints: number[]): void { this.move(`MOVEX M1J,P,(${joints.slice(0, this.nAxes).map((v) => fx(v, 5)).join(',')}),${this.tail()}`); }
  MoveL(pose: Mat4 | null, joints: number[]): void { if (!pose) { this.MoveJ(null, joints); return; } this.move(`MOVEX ${this.sm}M1X,L,${this.p(pose)},${this.tail()}`); }
  override MoveC(pose1: Mat4 | null, joints1: number[], pose2: Mat4 | null, joints2: number[], conf1: number[], conf2: number[]): void {
    if (!pose1 || !pose2) { super.MoveC(pose1, joints1, pose2, joints2, conf1, conf2); return; }
    this.move(`MOVEX ${this.sm}M1X,C1,${this.p(pose1)},${this.tail()}`);
    this.move(`MOVEX ${this.sm}M1X,C2,${this.p(pose2)},${this.tail()}`);
  }
  override setFrame(pose: Mat4, id: number): void {
    const [x, y, z, r, p, w] = pose2xyzrpw(pose);
    this.addline("' Using reference frame:");
    this.addline(`' USERFRAME=(${[x, y, z, w, p, r].map((v) => fx(v)).join(',')})`);
    if (id > 0 && !isIdentity(pose)) { this.coord = id; this.frame = null; }
    else { this.coord = 0; this.frame = pose; this.addline("' (using all targets with respect to the robot base)"); }
    this.addline(`CHGCOORD ${this.coord}`);
  }
  override setTool(pose: Mat4, id: number): void {
    this.tool = id > 0 ? id : 1;
    const [x, y, z, r, p, w] = pose2xyzrpw(pose);
    this.addline("' Using the tool:");
    this.addline(`' TOOL${this.tool}=(${[x, y, z, w, p, r].map((v) => fx(v)).join(',')})`);
    this.move(`MOVEX M1J,P,P*,S=${fx(this.speed, 1)},H=${this.tool},MS`);
  }
  override Pause(ms: number): void { this.addline(ms <= 0 ? 'PAUSE 99999' : `PAUSE ${fx(ms / 1000)}`); }
  override setSpeed(v: number): void { this.speed = v; }
  override setZoneData(z: number): void { this.sm = z < 0 ? 'A=1P,SM=0,' : `SM=${Math.min(3, Math.floor(z / 25))},`; }
  override setDO(io: IoName, v: number | boolean | string): void { this.addline(`${truthy(v) ? 'SET' : 'RESET'} ${typeof io === 'number' ? `[${String(io).padStart(2, '0')}]` : io}`); }
  override waitDI(io: IoName, v: number | boolean | string): void { this.addline(`${truthy(v) ? 'WAITI' : 'WAITJ'} ${typeof io === 'number' ? `[${String(io).padStart(2, '0')}]` : io}`); }
  override RunCode(code: string, isCall = false): void {
    if (!isCall) { this.addline(code); return; }
    const m = code.match(/(\d+)(?!.*\d)/);
    this.addline(`' Call program ${code}:`);
    this.addline(`CALLP ${m ? String(+m[1]).padStart(3, '0') : '-01'}`);
  }
  override RunMessage(msg: string, isComment = false): void { this.addline(isComment ? `' ${msg}` : `REM "${msg}"`); }
}
export const NACHI = vendorPost('Nachi_AX_FD', 'Nachi AX / FD', 'Nachi', 'prg', (ctx) => new NachiPost(ctx), { description: 'Nachi AX/FD program <model>-01-A.<nnn> (RoboDK Nachi_AX_FD compatible).' });

// ---- Mitsubishi MELFA BASIC ------------------------------------------------------------------------------------------
/**
 * RoboDK Mitsubishi: MELFA BASIC V with numbered lines and declared position variables (P<name><n>=(x,y,z,a,b,c)(FL1,FL2)
 * from pose_2_xyzrpw, FL1 from the configuration, FL2 multi-turn nibbles; J<name><n> joint positions), Mov / Mvs / Mvr
 * (start, via, end), Base Inv() / Tool, Spd, Cnt, M_OUT / Wait M_IN with timeout loops, GoSub, program splitting.
 */
export class MitsubishiPost extends RoboPost {
  override PROG_EXT = 'prg';
  protected parts: Array<{ name: string; decl: string[]; lines: string[] }> = [];
  protected maxLines: number;
  protected nPos = 0;
  protected base = '';
  protected tool = '';
  protected spd = '';
  protected last: { pose: Mat4; joints: number[]; conf: number[] } | null = null;
  constructor(ctx: RoboPostCtx) { super(ctx); this.maxLines = ctx.options.maxLines ?? 9999; }
  protected get cur() { return this.parts[this.parts.length - 1]; }
  override addline(s: string): void { this.cur.lines.push(`${this.cur.lines.length + 1} ${s}`); }
  protected fmt(v: number, d = 3) { const s = fx(v, d); return s.startsWith('-') ? s : `+${s}`; }
  override ProgStart(name: string): void {
    this.PROG_NAME = name.toUpperCase().replace(/[^A-Z0-9_]/g, '').slice(0, 12);
    this.parts = [{ name: this.PROG_NAME, decl: [], lines: [] }];
    this.addline(`*${this.PROG_NAME} ' prog. no 1`);
  }
  override ProgFinish(): void { this.addline('End'); if (this.parts.length === 1) this.addline(`'${'='.repeat(61)}`); }
  override ProgSave(): PostFile[] {
    const nl = '\r\n';
    const files = this.parts.map((p) => ({ name: `${p.name}.${this.PROG_EXT}`, content: [...p.decl, ...p.lines].join(nl) + nl }));
    if (this.parts.length > 1) {
      const main = `MAIN${this.PROG_NAME}`.slice(0, 12);
      files.unshift({ name: `${main}.${this.PROG_EXT}`, content: [`*${main}`, ...this.parts.map((p) => `CallP "${p.name}"`), 'End'].map((l, i) => `${i + 1} ${l}`).join(nl) + nl });
    }
    return files;
  }
  /** Split before a motion when the part is full; the new part restates frame, tool and speed. */
  protected splitIfNeeded(): void {
    if (this.cur.lines.length < this.maxLines - 1) return;
    if (this.parts.length === 1) this.cur.lines[0] = this.cur.lines[0].replace(/ ' prog\. no 1$/, '');
    this.addline('End');
    const name = `${this.PROG_NAME}${this.parts.length}`;
    this.parts.push({ name, decl: [], lines: [] });
    this.addline(`*${name}`);
    for (const s of [this.base, this.tool, this.spd]) if (s) this.addline(s);
  }
  protected fl2(joints: number[]): number {
    let v = 0;
    joints.slice(0, 8).forEach((q, i) => { v += Math.floor((q + 180) / 360) * 16 ** i; });
    return ((v % 2 ** 24) + 2 ** 24) % 2 ** 24;
  }
  protected pvar(pose: Mat4, joints: number[], conf: number[]): string {
    const n = `P${this.cur.name}${++this.nPos}`;
    const fl1 = 7 - (4 * (conf[0] ? 1 : 0) + 2 * (conf[1] ? 1 : 0) + (conf[2] ? 1 : 0));
    this.cur.decl.push(`${n}=(${pose2xyzrpw(pose).map((v) => this.fmt(v)).join(',')})(${fl1},${this.fl2(joints)})`);
    return n;
  }
  MoveJ(pose: Mat4 | null, joints: number[]): void {
    this.splitIfNeeded();
    const n = `J${this.cur.name}${++this.nPos}`;
    this.cur.decl.push(`${n}=(${joints.slice(0, this.nAxes).map((v) => this.fmt(v, 6)).join(',')})`);
    this.addline(`Mov ${n}`);
    this.last = pose ? { pose, joints, conf: [0, 0, 0] } : null;
  }
  MoveL(pose: Mat4 | null, joints: number[], conf: number[]): void {
    if (!pose) { this.MoveJ(null, joints); return; }
    this.splitIfNeeded();
    this.addline(`Mvs ${this.pvar(pose, joints, conf)}`);
    this.last = { pose, joints, conf };
  }
  override MoveC(pose1: Mat4 | null, joints1: number[], pose2: Mat4 | null, joints2: number[], conf1: number[], conf2: number[]): void {
    if (!pose1 || !pose2 || !this.last) { super.MoveC(pose1, joints1, pose2, joints2, conf1, conf2); return; }
    this.splitIfNeeded();
    const a = this.pvar(this.last.pose, joints1, conf1), b = this.pvar(pose1, joints1, conf1), c = this.pvar(pose2, joints2, conf2);
    this.addline(`Mvr ${a},${b}, ${c}`);
    this.last = { pose: pose2, joints: joints2, conf: conf2 };
  }
  override setFrame(pose: Mat4): void { this.base = `Base Inv((${pose2xyzrpw(pose).map((v) => this.fmt(v)).join(',')}))`; this.addline(this.base); }
  override setTool(pose: Mat4): void { this.tool = `Tool (${pose2xyzrpw(pose).map((v) => this.fmt(v)).join(',')})`; this.addline(this.tool); }
  override Pause(ms: number): void { this.addline(ms < 0 ? 'Hlt' : `Dly ${fx(ms / 1000)}`); }
  override setSpeed(v: number): void { this.spd = `Spd ${fx(v, 1)}`; this.addline(this.spd); }
  override setZoneData(z: number): void { this.addline(z > 0 ? `Cnt 1, ${fx(z, 1)}, ${fx(z, 1)}` : 'Cnt 0'); }
  override setDO(io: IoName, v: number | boolean | string): void { this.addline(`${typeof io === 'number' ? `M_OUT(${io})` : io}=${truthy(v) ? 1 : 0}`); }
  override waitDI(io: IoName, v: number | boolean | string, timeoutMs = -1): void {
    const src = typeof io === 'number' ? `M_IN(${io})` : io, val = truthy(v) ? 1 : 0;
    if (timeoutMs <= 0) { this.addline(`Wait ${src}=${val}`); return; }
    this.addline('M_Timer(1) = 0');
    const lbl = `*cond_${this.cur.lines.length + 1}`;
    this.addline(lbl);
    this.addline(`If ${src}<>${val} AND M_timer(1) < ${Math.trunc(timeoutMs)} THEN ${lbl}`);
  }
  override RunCode(code: string, isCall = false): void {
    if (!isCall) { this.addline(code); return; }
    const { name, args } = callParts(code);
    this.addline(`GoSub *${name.toUpperCase().slice(0, 16)}${args}`);
  }
  override RunMessage(msg: string, isComment = false): void {
    if (isComment) { this.addline(`'${msg}`); return; }
    this.addline('Open "COM1:" As #1');
    this.addline(`Print #1,"${msg}"`);
    this.addline('Close #1');
  }
}
export const MITSUBISHI = vendorPost('Mitsubishi_Melfa', 'Mitsubishi MELFA BASIC', 'Mitsubishi', 'prg', (ctx) => new MitsubishiPost(ctx), { aliases: ['Mitsubishi'], description: 'Mitsubishi MELFA BASIC V program (RoboDK Mitsubishi compatible).' });

// ---- Dobot CR (V3 Lua) -------------------------------------------------------------------------------------------------
/**
 * RoboDK Dobot_V3: Lua function per program, SetUser / SetTool 9, MoveJ joint / Move coordinate (pose_2_xyzrpw) /
 * Arc3, SpeedS / Speed / AccelS / Accel in % (<= 100), CP blending, DO / WaitDI, Wait (ms).
 * Deviation from RoboDK (defect): call arguments keep their commas.
 */
export class DobotPost extends RoboPost {
  override PROG_EXT = 'lua';
  override addline(s: string): void { this.PROG.push(`\t${s}`); }
  override ProgStart(name: string): void { this.PROG_NAME = name; this.PROG.push('--[[Dobot CR program (V3) generated from RoboDK post processor', '    Dobot script language is based on lua - refer to Dobot V3 Programming Manual', '--]]', `function ${name}()`); }
  override ProgFinish(name: string): void { this.PROG.push('end', '', '', '--Main program call:--', `${name}()`); }
  override ProgSave(name: string): PostFile[] { return [{ name: `${name}.${this.PROG_EXT}`, content: this.PROG.join('\r\n') + '\r\n' }]; }
  protected c6(m: Mat4) { return `{coordinate={${pose2xyzrpw(m).map((v) => fx(v, 4)).join(',')}}}`; }
  protected pct(v: number) { return Math.min(100, pyRound(v)); }
  MoveJ(_pose: Mat4 | null, joints: number[]): void { this.addline(`MoveJ({joint={${joints.slice(0, this.nAxes).map((v) => fx(v, 6)).join(', ')}}})`); }
  MoveL(pose: Mat4 | null, joints: number[]): void { if (!pose) { this.MoveJ(null, joints); return; } this.addline(`Move(${this.c6(pose)},'User=9 Tool=9')`); }
  override MoveC(pose1: Mat4 | null, joints1: number[], pose2: Mat4 | null, joints2: number[], conf1: number[], conf2: number[]): void {
    if (!pose1 || !pose2) { super.MoveC(pose1, joints1, pose2, joints2, conf1, conf2); return; }
    this.addline(`Arc3(${this.c6(pose1)},${this.c6(pose2)},'User=9 Tool=9')`);
  }
  override setFrame(pose: Mat4): void { this.addline(`SetUser(9,{${pose2xyzrpw(pose).map((v) => fx(v, 4)).join(',')}})`); }
  override setTool(pose: Mat4): void { this.addline(`SetTool(9,{${pose2xyzrpw(pose).map((v) => fx(v, 4)).join(',')}})`); }
  override Pause(ms: number): void { if (ms > 0) this.addline(`Wait(${Math.trunc(ms)})`); }
  override setSpeed(v: number): void { this.addline(`SpeedS(${this.pct(v)})`); }
  override setSpeedJoints(w: number): void { this.addline(`Speed(${this.pct(w)})`); }
  override setAcceleration(a: number): void { this.addline(`AccelS(${this.pct(a)})`); }
  override setAccelerationJoints(a: number): void { this.addline(`Accel(${this.pct(a)})`); }
  override setZoneData(z: number): void { this.addline(`CP(${Math.min(100, Math.max(0, Math.trunc(z)))})`); }
  override setDO(io: IoName, v: number | boolean | string): void { this.addline(`DO(${io}, ${truthy(v) ? 1 : 0})`); }
  override waitDI(io: IoName, v: number | boolean | string, timeoutMs = -1): void { this.addline(`WaitDI(${io}, ${truthy(v) ? 1 : 0}${timeoutMs > 0 ? `, ${pyRound(timeoutMs)}` : ''})`); }
  override RunCode(code: string, isCall = false): void { if (!isCall) { this.addline(code); return; } const { name, args } = callParts(code); this.addline(`${name}${args || '()'}`); }
  override RunMessage(msg: string, isComment = false): void { this.addline(isComment ? `-- ${msg}` : `print("${msg}")`); }
}
export const DOBOT = vendorPost('Dobot_CR', 'Dobot CR (V3 Lua)', 'Dobot', 'lua', (ctx) => new DobotPost(ctx), { aliases: ['Dobot_V3'], description: 'Dobot CR V3 Lua script (RoboDK Dobot_V3 compatible).' });

// ---- Hanwha HCR (JavaScript) ---------------------------------------------------------------------------------------
/**
 * RoboDK HCR: JavaScript function per program; poses converted to the robot base (the reference frame is applied),
 * TxyzRxyz degrees, moveJoint / moveLinear('tcp') with speed and acceleration, moveArc, setToolCenterPoint,
 * setGeneralDigitalOutput, console.log / popup. Like RoboDK, input waits are written as comments (no documented API).
 */
export class HcrPost extends RoboPost {
  override PROG_EXT = 'js';
  protected frame: Mat4 | null = null;
  protected v = 500; protected a = 1000; protected vj = 50; protected aj = 100;
  override addline(s: string): void { this.PROG.push(`    ${s}`); }
  override ProgStart(name: string): void { this.PROG_NAME = name; this.PROG.push('// -----------------------', `// Program ${name}`, `function ${name}(){`); }
  override ProgFinish(name: string): void { this.PROG.push('}', `// End of program ${name}`, '', ''); }
  override ProgSave(name: string): PostFile[] { return [{ name: `${name}.${this.PROG_EXT}`, content: this.PROG.join('\r\n') + '\r\n' }]; }
  protected obj(m: Mat4) { const [x, y, z, rx, ry, rz] = pose2Staubli(m); return `{x: ${fx(x)}, y: ${fx(y)}, z: ${fx(z)}, rx: ${fx(rx)}, ry: ${fx(ry)}, rz: ${fx(rz)}}`; }
  protected base(m: Mat4) { return this.frame ? multiply(this.frame, m) : m; }
  MoveJ(_pose: Mat4 | null, joints: number[]): void { this.addline(`moveJoint([${joints.slice(0, this.nAxes).map((v) => fx(v, 6)).join(', ')}], ${pyStrNum(this.vj)}, ${pyStrNum(this.aj)});`); }
  MoveL(pose: Mat4 | null, joints: number[]): void { if (!pose) { this.MoveJ(null, joints); return; } this.addline(`moveLinear('tcp', ${this.obj(this.base(pose))}, ${pyStrNum(this.v)}, ${pyStrNum(this.a)});`); }
  override MoveC(pose1: Mat4 | null, joints1: number[], pose2: Mat4 | null, joints2: number[], conf1: number[], conf2: number[]): void {
    if (!pose1 || !pose2) { super.MoveC(pose1, joints1, pose2, joints2, conf1, conf2); return; }
    this.addline(`moveArc('tcp', ${this.obj(this.base(pose1))}, ${this.obj(this.base(pose2))});`);
  }
  override setFrame(pose: Mat4): void { this.frame = isIdentity(pose) ? null : pose; }
  override setTool(pose: Mat4): void { this.addline(`setToolCenterPoint(${this.obj(pose)});`); this.addline('// setPayload(0.5, {x:0, y:0, z:150}); // sample 0.5kg, CoG is (0,0,150)'); }
  override Pause(ms: number): void { this.addline(ms < 0 ? 'halt();' : `wait(${fx(ms)});`); }
  override setSpeed(v: number): void { this.v = v; }
  override setSpeedJoints(w: number): void { this.vj = w; }
  override setAcceleration(a: number): void { this.a = a; }
  override setAccelerationJoints(a: number): void { this.aj = a; }
  override setDO(io: IoName, v: number | boolean | string): void { this.addline(`setGeneralDigitalOutput(${io}, ${truthy(v) ? 1 : 0});`); }
  override waitDI(io: IoName, v: number | boolean | string, timeoutMs = -1): void {
    this.addline(timeoutMs > 0 ? `// waitGeneralDigitalInput(${typeof io === 'number' ? `In${io}` : io}, ${truthy(v) ? 1 : 0}, ${fx(timeoutMs, 1)});` : `// wait(getGeneralDigitalInput(0) == ${truthy(v) ? 1 : 0});`);
  }
  override RunCode(code: string, isCall = false): void { this.addline(isCall && !code.includes('(') ? `${code.replace(/ /g, '')}()` : code); }
  override RunMessage(msg: string, isComment = false): void { this.addline(isComment ? `console.log('Message: ', '${msg}');` : `popup('${msg}');`); }
}
/** Python str() of a number as RoboDK prints speeds (300.0 -> "300", 12.5 -> "12.5"). */
const pyStrNum = (v: number) => (Number.isInteger(v) ? String(v) : String(v));
export const HCR = vendorPost('Hanwha_HCR', 'Hanwha HCR (JavaScript)', 'Hanwha', 'js', (ctx) => new HcrPost(ctx), { aliases: ['HCR'], description: 'Hanwha HCR JavaScript program (RoboDK HCR compatible).' });

// ---- JAKA (JKS script) ----------------------------------------------------------------------------------------------
const JAKA_HEADER = ['# This script was generated with the JAKA post and is suitable for JAKA 1.7.x.', '', '# Global parameters:', 'endPosJ =[0,0,0,0,0,0]', 'endPosL =[0,0,0,0,0,0]', 'pos_mvl =[0,0,0,0,0,0]', 'pos_waypoint =[0,0,0,0,0,0]', 'set_tool_id(0)', 'set_user_frame_id(0)', '', '# Main program:'];
/**
 * RoboDK JAKA (.jks script for JAKA 1.7): movj / movl / movc with speed, acceleration and blend, set_user_frame /
 * set_tool, set_digital_output / wait_input on 0-based IO, sleep. RoboDK's Scratch-like .zu / zip project is not
 * written (its block ids are random); calls to other programs are not supported by the controller script.
 */
export class JakaPost extends RoboPost {
  override PROG_EXT = 'jks';
  protected v = 250; protected a = 250; protected vj = 60; protected aj = 200; protected blend = 1; protected moved = false;
  override ProgStart(name: string): void { this.PROG_NAME = name; if (!this.PROG.length) this.PROG.push(...JAKA_HEADER); }
  /** Like RoboDK, the closing comment is dropped once the program exceeds its line limit. */
  override ProgFinish(): void { if (this.PROG.length <= (this.ctx.options.maxLines ?? Infinity)) this.addline('# End of main program'); }
  override ProgSave(name: string): PostFile[] { return [{ name: `${name}.${this.PROG_EXT}`, content: this.PROG.join('\r\n') + '\r\n' }]; }
  protected n(v: number) { return String(+v.toFixed(6)); }
  protected list(vals: number[]) { return `[${vals.map((v) => this.n(v)).join(', ')}]`; }
  MoveJ(_pose: Mat4 | null, joints: number[]): void {
    this.addline(`endPosJ=${this.list(joints.slice(0, 6))}`);
    this.addline(`movj(endPosJ,0,${this.n(this.vj)},${this.n(this.aj)},${this.moved ? this.n(this.blend) : '0.0'})`);
    this.moved = true;
  }
  MoveL(pose: Mat4 | null, joints: number[]): void {
    if (!pose) { this.MoveJ(null, joints); return; }
    this.addline(`endPosJ = ${this.list(joints.slice(0, 6))}`);
    this.addline(`endPosL = ${this.list(pose2xyzrpw(pose))}`);
    this.addline(`movl(endPosL,0,${this.n(this.v)},${this.n(this.a)},${this.n(this.blend)})`);
    this.moved = true;
  }
  override MoveC(pose1: Mat4 | null, joints1: number[], pose2: Mat4 | null, joints2: number[], conf1: number[], conf2: number[]): void {
    if (!pose1 || !pose2) { super.MoveC(pose1, joints1, pose2, joints2, conf1, conf2); return; }
    this.addline(`passPos = ${this.list(pose2xyzrpw(pose1))}`);
    this.addline(`endPos = ${this.list(pose2xyzrpw(pose2))}`);
    this.addline(`movc(passPos,endPos,0,${this.n(this.v)},${this.n(this.a)},${this.n(this.blend)},0)`);
    this.moved = true;
  }
  override setFrame(pose: Mat4): void { this.addline(`ucspos = ${this.list(pose2xyzrpw(pose))}`); this.addline('set_user_frame(ucspos)'); }
  override setTool(pose: Mat4): void { this.addline(`toolpos = ${this.list(pose2xyzrpw(pose))}`); this.addline('set_tool(toolpos)'); }
  override Pause(ms: number): void { this.addline(`sleep(${this.n(ms / 1000)})`); }
  override setSpeed(v: number): void { this.v = v; }
  override setSpeedJoints(w: number): void { this.vj = w; }
  override setAcceleration(a: number): void { this.a = a; }
  override setAccelerationJoints(a: number): void { this.aj = a; }
  override setZoneData(z: number): void { this.blend = Math.max(0, z); }
  override setDO(io: IoName, v: number | boolean | string): void {
    if (typeof io !== 'number') { this.addlog(`Invalid DO index: "${io}". Use [1,] range or "AO_X".`); return; }
    this.addline(`set_digital_output(0,${io - 1},${truthy(v) ? 1 : 0},1)`);
  }
  override waitDI(io: IoName, v: number | boolean | string, timeoutMs = -1): void {
    if (typeof io !== 'number') { this.addlog(`Invalid DI index: "${io}". Use [1,] range or "DI_X".`); return; }
    this.addline(`wait_input(0,${io - 1},${truthy(v) ? 1 : 0},${timeoutMs > 0 ? this.n(timeoutMs / 1000) : '0'})`);
  }
  override RunCode(code: string, isCall = false): void { if (isCall) { this.addlog(`Subprograms are not supported by the JAKA script: ${code} not called`); return; } this.addline(code); }
  override RunMessage(msg: string, isComment = false): void { if (isComment) this.addline(`# ${msg}`); else this.addlog(`"RunMessage" function not implemented (${msg})`); }
}
export const JAKA = vendorPost('JAKA', 'JAKA (JKS script)', 'JAKA', 'jks', (ctx) => new JakaPost(ctx), { description: 'JAKA 1.7 script (RoboDK JAKA compatible, .jks).' });

// ---- AUBO (aubo script) ---------------------------------------------------------------------------------------------
/**
 * RoboDK AUBO: <name>/main.aubo Lua script inside a loop function, moves by joint values (move_joint / move_line,
 * radians) grouped in logic-tree Move items with velocity / acceleration / blend profiles, robot board IO by name
 * (U_DO_nn / U_DI_nn), popup_message, procedures. The logic-tree items carry random UUIDs like RoboDK's; RoboDK's
 * main.xml (AUBO teach-pendant tree) is not written.
 */
export class AuboPost extends RoboPost {
  override PROG_EXT = 'aubo';
  protected vj = 0.3; protected aj = 1.1; protected v = 0.25; protected a = 0.25; protected blend = 0.001;
  protected vjSet = false;
  protected group: string | null = null;
  protected uuid(): string { let s = ''; for (let i = 0; i < 32; i++) s += Math.floor(Math.random() * 16).toString(16); return s; }
  override addline(s: string): void { this.group = null; this.PROG.push(`      ${s}`); }
  override ProgStart(): void { this.PROG.push('--Main Program', '--script function definition', '  function loopScriptFunc_0()'); }
  override ProgFinish(): void {
    this.PROG.push('  end', '', '--(Logic tree item : Project_Program) Project_Program', '  --(Logic tree item : Loop) Loop', '  while (true) do', '    sleep(0.001)', '    if (loopScriptFunc_0() == false) then', '      break', '    end', '  end', '--End of main program', '');
  }
  override ProgSave(name: string): PostFile[] { return [{ name: `${name}/main.aubo`, content: this.PROG.join('\r\n') + '\r\n' }]; }
  protected rad(joints: number[]) { return `{${joints.slice(0, 6).map((v) => fx((v * Math.PI) / 180, 6)).join(',')}}`; }
  protected move(kind: 'MoveJ' | 'MoveL', cmd: string, joints: number[]): void {
    const profile = kind === 'MoveJ'
      ? [`set_joint_maxvelc({${Array(6).fill(this.vjSet ? String(this.vj) : String(this.vj)).join(',')}})`, `set_joint_maxacc({${Array(6).fill(String(this.aj)).join(',')}})`]
      : [`set_end_maxvelc(${fx(this.v, 6)})`, `set_end_maxacc(${fx(this.a, 6)})`];
    const key = `${kind}|${profile.join('|')}|${this.blend}`;
    if (this.group !== key) {
      this.addline(`--(Logic tree item : Move) ${kind}`);
      this.addline('init_global_move_profile()');
      for (const p of profile) this.addline(p);
      if (this.blend > 0) this.addline(`set_arrival_ahead_blend_mode(${fx(this.blend, 4)})`);
    }
    this.PROG.push('        --(Logic tree item : Waypoint) Waypoint', `        set_current_logic_tree_item("${this.uuid()}")`, `        ${cmd}(${this.rad(joints)}, true)`, '        set_step_breakpoint()');
    this.group = key;
    this.lastJoints = joints;
  }
  MoveJ(_pose: Mat4 | null, joints: number[]): void { this.move('MoveJ', 'move_joint', joints); }
  MoveL(_pose: Mat4 | null, joints: number[]): void { this.move('MoveL', 'move_line', joints); }
  /** Circle through the current, via and end joint positions (add_waypoint x3 + move_track Arc). */
  override MoveC(_pose1: Mat4 | null, joints1: number[], _pose2: Mat4 | null, joints2: number[]): void {
    const start = this.lastJoints ?? joints1;
    this.addline('--(Logic tree item : Move) MoveC');
    this.addline('init_global_move_profile()');
    this.addline(`set_end_maxvelc(${fx(this.v, 6)})`);
    this.addline(`set_end_maxacc(${fx(this.a, 6)})`);
    if (this.blend > 0) this.addline(`set_arrival_ahead_blend_mode(${fx(this.blend, 4)})`);
    for (const j of [start, joints1, joints2]) this.PROG.push('        --(Logic tree item : Waypoint) Waypoint', `        set_current_logic_tree_item("${this.uuid()}")`, `        add_waypoint(${this.rad(j)})`);
    this.PROG.push('        move_track(MoveTrackType.Arc, true)', '        set_step_breakpoint()');
    this.lastJoints = joints2;
    this.group = null;
  }
  protected lastJoints: number[] | null = null;
  override setTool(pose: Mat4): void { const q = pose2Quaternion(pose); this.addline(`--set_tool_kinematics_param({${[pose[12], pose[13], pose[14]].map((v) => fx(v, 6)).join(',')}},{${q.map((v) => fx(v, 6)).join(',')}})`); }
  override Pause(ms: number): void {
    if (ms <= 0) { this.group = null; this.PROG.push('  halt() # reimplement this function to force stop'); return; }
    this.item('Wait', 'Wait', `sleep(${fx(ms / 1000)})`, 'set_step_breakpoint()');
  }
  protected item(type: string, alias: string, ...lines: string[]): void {
    this.addline(`--(Logic tree item : ${type}) ${alias}`);
    this.addline(`set_current_logic_tree_item("${this.uuid()}")`);
    for (const l of lines) this.addline(l);
  }
  override setSpeed(v: number): void { this.v = v / 1000; }
  override setSpeedJoints(w: number): void { this.vj = (w * Math.PI) / 180; this.vjSet = true; }
  override setAcceleration(a: number): void { this.a = a / 1000; }
  override setAccelerationJoints(a: number): void { this.aj = (a * Math.PI) / 180; }
  override setZoneData(z: number): void { this.blend = Math.max(0, z) / 1000; }
  protected ioName(io: IoName, kind: 'DO' | 'DI') { return typeof io === 'number' ? `U_${kind}_${String(io).padStart(2, '0')}` : String(io); }
  override setDO(io: IoName, v: number | boolean | string): void { this.item('Set', 'Set', `set_robot_io_status(RobotIOType.RobotBoardUserDO, "${this.ioName(io, 'DO')}", ${truthy(v) ? 1 : 0})`); this.addline('set_step_breakpoint()'); }
  override waitDI(io: IoName, v: number | boolean | string): void { this.item('Wait', 'Wait', `while (not (    get_robot_io_status(RobotIOType.RobotBoardUserDI, "${this.ioName(io, 'DI')}") == ${truthy(v) ? 1 : 0})) do`, '  sleep(0.01)', 'end'); }
  /** Like RoboDK: a call with arguments becomes a Script item (upper case), raw code a comment. */
  override RunCode(code: string, isCall = false): void {
    if (!isCall) { this.group = null; this.PROG.push(`  -- ${code}`); return; }
    const c = code.trim();
    if (c.includes('(')) { this.item('Script', 'Script', c.toUpperCase(), 'set_step_breakpoint()'); return; }
    this.addline(`--(Logic tree item : Procedure) ${c}`);
    this.addline(`${c.replace(/ /g, '')}()`);
  }
  override RunMessage(msg: string, isComment = false): void { if (isComment) this.addline(`-- ${msg}`); else this.item('Message', 'Message', `popup_message(0, "${msg}", false)`); }
}
export const AUBO = vendorPost('AUBO', 'AUBO (aubo script)', 'AUBO', 'aubo', (ctx) => new AuboPost(ctx), { description: 'AUBO script project (RoboDK AUBO compatible, main.aubo).' });

// ---- Elite Robots EC (JBI V3) ----------------------------------------------------------------------------------------
/**
 * RoboDK Elite_Robots_EC_V3: SETJOINT / SETPOSE variable definitions (poses in the user frame, radians), user frame /
 * tool through V000, a Lua conversion of the user-frame poses to the base (B010 / B000), then the motion block
 * (MOVJ / MOVL / MOVC with VJ %, V mm/s, CR <= 7 mm). Like RoboDK's post, IO / waits / calls / pauses are written in
 * the definition section, i.e. BEFORE the motions: this is reported as a warning. Split programs: parts <name>,
 * <name>2... each re-run all motions up to their end, M_<name> calls the parts then runs every motion (RoboDK's rule).
 */
export class ElitePost extends RoboPost {
  override PROG_EXT = 'jbi';
  /** Definition lines of the current part (comments, frames, SETJOINT / SETPOSE, IO, waits, calls). */
  protected defs: string[] = [];
  /** Every motion line of the program (each part re-runs all motions up to its end, like RoboDK). */
  protected moves: string[] = [];
  protected parts: { defs: string[]; nMoves: number; nV: number }[] = [];
  protected nV = 0; protected nP = 0;
  protected nLines = 0;
  protected maxLines: number;
  protected speed = 'V=100.0';
  protected vj = 100;
  protected cr = 0;
  protected lastV: string | null = null;
  protected codeBetweenMoves = false;
  constructor(ctx: RoboPostCtx) { super(ctx); this.maxLines = ctx.options.maxLines ?? 2000; }
  /** RoboDK counts every definition and motion line; the part is closed before the line that exceeds the limit. */
  protected count(): void {
    if (this.nLines >= this.maxLines) { this.parts.push({ defs: this.defs, nMoves: this.moves.length, nV: this.nV }); this.defs = []; this.nLines = 0; }
    this.nLines++;
  }
  protected def(...lines: string[]): void { for (const s of lines) { this.count(); this.defs.push(s); } }
  protected move(s: string): void { this.count(); this.moves.push(s); }
  override addline(s: string): void { this.def(s); if (this.moves.length) this.codeBetweenMoves = true; }
  override ProgStart(name: string): void { this.PROG_NAME = name.replace(/\s+/g, '').slice(0, 20); }
  protected header(): string[] {
    const d = this.ctx.date, p2 = (n: number) => String(n).padStart(2, '0');
    return ['NOP', '// HEADER', `// DATE ${d.getFullYear()}/${p2(d.getMonth() + 1)}/${p2(d.getDate())} ${p2(d.getHours())}:${p2(d.getMinutes())}`, '// COMM Generated using VerticalBot Studio'];
  }
  protected tailBlock(nV: number, nMoves: number): string[] {
    return ['//set total pose number and use lua to tranf pose in user to in_base', `SET B010 ${Math.max(0, nV - 1)}`, 'SET B000 1', 'TIMER T=0.2 S', 'WAIT B000=0  ', ...this.moves.slice(0, nMoves), 'END'];
  }
  override ProgFinish(): void {
    if (this.codeBetweenMoves) this.ctx.warn('Elite (RoboDK-compatible output): IO, waits, calls and pauses are executed before the motion block');
  }
  override ProgSave(): PostFile[] {
    const file = (name: string, lines: string[]): PostFile => ({ name: `${name}.${this.PROG_EXT}`, content: lines.join('\r\n') + '\r\n' });
    if (!this.parts.length) return [file(this.PROG_NAME, [...this.header(), ...this.defs, ...this.tailBlock(this.nV, this.moves.length)])];
    const all = [...this.parts, { defs: this.defs, nMoves: this.moves.length, nV: this.nV }];
    const names = all.map((_, i) => (i ? `${this.PROG_NAME}${i + 1}` : this.PROG_NAME));
    this.ctx.warn(`Elite: program ${this.PROG_NAME} split into ${all.length} parts called by M_${this.PROG_NAME} (each part re-runs the previous motions, like RoboDK)`);
    return [
      file(`M_${this.PROG_NAME}`, [...this.header(), ...names.map((n) => `CALL JOB:${n}`), ...this.tailBlock(this.nV, this.moves.length)]),
      ...all.map((p, i) => file(names[i], [...this.header(), ...p.defs, ...this.tailBlock(p.nV, p.nMoves)])),
    ];
  }
  protected num(v: number) { return String(+v.toFixed(6)); }
  protected tail() { return `CR=${fx(Math.min(7, Math.max(0, pyRound(this.cr))), 1)}MM ACC=50 DEC=50`; }
  protected pose(m: Mat4, angDec = 5) { const [x, y, z, r, p, w] = pose2xyzrpw(m); return [x, y, z].map((v) => String(v)).concat([r, p, w].map((a) => fx((a * Math.PI) / 180, angDec))).join(','); }
  protected newV(m: Mat4): string { const v = `V${String(this.nV).padStart(3, '0')}`; this.def(`SETPOSE ${v} ${this.pose(m)}`); this.nV++; return v; }
  MoveJ(_pose: Mat4 | null, joints: number[]): void {
    const p = `P${String(this.nP++).padStart(3, '0')}`;
    this.def(`SETJOINT ${p} ${joints.slice(0, this.nAxes).map((v) => fx(v, 5)).join(',')}`);
    this.move(`MOVJ ${p} VJ=${Math.min(100, pyRound(this.vj))}% ${this.tail()}`);
  }
  MoveL(pose: Mat4 | null, joints: number[]): void {
    if (!pose) { this.MoveJ(null, joints); return; }
    const v = this.newV(pose);
    this.move(`MOVL ${v} ${this.speed} ${this.tail()}`);
    this.lastV = v;
  }
  override MoveC(pose1: Mat4 | null, joints1: number[], pose2: Mat4 | null, joints2: number[], conf1: number[], conf2: number[]): void {
    if (!pose1 || !pose2 || !this.lastV) { super.MoveC(pose1, joints1, pose2, joints2, conf1, conf2); return; }
    const a = this.newV(pose1), b = this.newV(pose2);
    for (const v of [this.lastV, a, b]) this.move(`MOVC ${v} ${this.speed} ${this.tail()}`);
    this.lastV = b;
  }
  protected frameLine(m: Mat4) { const [x, y, z, r, p, w] = pose2xyzrpw(m); return `SETPOSE V000 ${[x, y, z].map((v) => this.num(v)).join(',')},${[r, p, w].map((a) => fx((a * Math.PI) / 180, 4)).join(',')}`; }
  override setFrame(pose: Mat4): void { this.def(this.frameLine(pose), 'SETUSERFRAME USER #(1) V000'); }
  override setTool(pose: Mat4): void { this.def(this.frameLine(pose), 'SETTOOLFRAME TOOL#(1) V000', 'SETTOOLNUMBER TF=1'); }
  override Pause(ms: number): void { this.addline(`TIMER T=${this.num(ms < 0 ? -0.001 : ms / 1000)}`); }
  override setSpeed(v: number): void { this.speed = `V=${pyRound(v)}MM/S`; }
  override setSpeedJoints(w: number): void { this.vj = w; }
  override setZoneData(z: number): void { this.cr = z; }
  override setDO(io: IoName, v: number | boolean | string): void { this.addline(`DOUT ${typeof io === 'number' ? `OT#(${io})` : io} ${truthy(v) ? 1 : 0}`); }
  override waitDI(io: IoName, v: number | boolean | string, timeoutMs = -1): void { this.addline(`WAIT ${typeof io === 'number' ? `IN#(${io})` : io}=${truthy(v) ? 1 : 0}${timeoutMs > 0 ? ` T=${this.num(timeoutMs / 1000)}` : ''}`); }
  override RunCode(code: string, isCall = false): void { this.addline(isCall ? `CALL JOB:${code.replace(/\s+/g, '').slice(0, 20)}` : code); }
  override RunMessage(msg: string, isComment = false): void {
    if (!isComment) { this.addline(`TPWRITE ${msg}`); return; }
    if (!msg.length) { this.def('// '); return; }
    for (let i = 0; i < msg.length; i += 73) this.def(`// ${msg.slice(i, i + 73)}`);
  }
}
export const ELITE = vendorPost('Elite_EC', 'Elite Robots EC (JBI V3)', 'Elite', 'jbi', (ctx) => new ElitePost(ctx), { aliases: ['Elite_Robots_EC_V3', 'Elite_Robots_EC'], description: 'Elite Robots EC JBI program (RoboDK Elite_Robots_EC_V3 compatible).' });
