/**
 * ABB RAPID posts (IRC5 .mod, OmniCore .modx, S4C .prg), output-compatible with RoboDK's ABB_RAPID_IRC5 /
 * ABB_RAPID_OmniCore / ABB_RAPID_S4C posts (tests/posts_gt.test.ts): one module with tooldata / wobjdata declarations,
 * sub-programs as PROCs and the first program as PROC Main, MoveAbsJ / MoveL / MoveC with RoboDK configuration data
 * [cf1,cf4,cf6,cfx], standard or custom speeddata / zonedata, D_OUT_n / D_IN_n numbered IO, WaitDI \MaxTime, and
 * program splitting into dynamically loaded modules. Studio extensions: FOR / WHILE TRUE loops, IF on inputs.
 */
import { Mat4 } from '../core/math/pose';
import { PostProcessor, PostProgram, PostFile, registerPost } from './base';
import { RoboPost, RoboPostCtx, runRoboPost, fx, IoName, CtlCondition } from './robopost';
import { pose2Quaternion } from './euler';

const STD_SPEEDS = [5, 10, 20, 30, 40, 50, 60, 80, 100, 150, 200, 300, 400, 500, 600, 800, 1000, 1500, 2000, 2500, 3000, 4000, 5000, 6000, 7000];
const STD_ZONES = [0, 1, 5, 10, 15, 20, 30, 40, 50, 60, 80, 100, 150, 200];
const f1 = (v: number) => fx(v, 1);

export const rapidPose = (m: Mat4): string => { const q = pose2Quaternion(m); return `[[${fx(m[12])},${fx(m[13])},${fx(m[14])}],[${q.map((v) => fx(v, 8)).join(',')}]]`; };
export function rapidZone(z: number): string {
  if (z < 0) return 'fine';
  if (STD_ZONES.includes(z)) return `z${z}`;
  return `[FALSE,${f1(z)},${f1(1.5 * z)},${f1(1.5 * z)},${f1(0.15 * z)},${f1(1.5 * z)},${f1(0.15 * z)}]`;
}
export function rapidSpeed(v: number): string {
  if (v > 7000) return 'v7000';
  if (STD_SPEEDS.includes(v)) return `v${v}`;
  return `[${fx(v, 2)},500,5000,1000]`;
}

type Variant = 'IRC5' | 'OmniCore' | 'S4C';
interface Proc { name: string; parts: string[][] }

export class AbbPost extends RoboPost {
  protected decls = new Map<string, string>(); // name -> 'tool' | 'wobj'
  protected toolDecl: string[] = [];
  protected wobjDecl: string[] = [];
  protected procs: Proc[] = [];
  protected cur!: Proc;
  protected modName = '';
  protected speed = 'v500';
  protected zone = 'z1';
  protected tool = 'tool0';
  protected wobj = 'wobj0';
  protected nLines = 0;
  protected maxLines: number;
  protected loopVar = 0;
  constructor(ctx: RoboPostCtx, protected variant: Variant) {
    super(ctx);
    this.PROG_EXT = variant === 'IRC5' ? 'mod' : variant === 'OmniCore' ? 'modx' : 'prg';
    this.maxLines = ctx.options.maxLines ?? (variant === 'S4C' ? 5000 : 20000);
  }
  protected get charLimit() { return this.variant === 'S4C' ? 13 : 1000; }
  /** RAPID identifier as RoboDK writes it (spaces and punctuation removed, letter prefix when it starts with a digit). */
  protected ident(name: string, prefix: string): string {
    let s = name.replace(/[^A-Za-z0-9_()]/g, '');
    if (/^\d/.test(s)) s = prefix + s;
    return s.slice(0, this.charLimit) || prefix;
  }
  /** RoboDK renames a tooldata / wobjdata that would clash with a program (PROC) name: Frame "TopSurf" -> TopSurf2. */
  protected noClash(nm: string): string {
    const used = new Set(this.ctx.programs.map((p) => this.ident(p, 'P').toLowerCase()));
    if (!used.has(nm.toLowerCase())) return nm;
    let k = 2;
    while (used.has(`${nm}${k}`.toLowerCase())) k++;
    return `${nm}${k}`;
  }
  override addline(s: string): void { this.cur.parts[this.cur.parts.length - 1].push(s); this.nLines++; }
  override ProgStart(name: string): void {
    const first = this.procs.length === 0;
    if (first) { this.PROG_NAME = this.variant === 'S4C' ? name.replace(/[^A-Za-z0-9_()]/g, '').slice(0, 13) : name; this.modName = `MOD_${this.ident(name, 'P')}`; }
    this.cur = { name: first ? 'Main' : this.ident(name, 'P'), parts: [[]] };
    this.procs.push(this.cur);
    this.nLines = 0;
    this.addline('ConfJ \\On;');
    this.addline('ConfL \\Off;');
  }
  protected splitIfNeeded(): void {
    if (this.nLines < this.maxLines || this.procs.length !== 1) return;
    this.cur.parts.push([]);
    this.nLines = 0;
  }
  protected fileName(name: string) { return this.variant === 'S4C' ? name.replace(/[^A-Za-z0-9_()]/g, '').slice(0, 13) : name; }
  override ProgSave(name: string): PostFile[] {
    const nl = '\r\n';
    const head = ['%%%', '  VERSION:1', '  LANGUAGE:ENGLISH', '%%%'];
    const decl = ['', '    ! -------------------------------', '    ! Define customized variables here', '    ! ...', '', '\t! Tool variables: ', ...this.toolDecl, '', '\t! Reference variables:', ...this.wobjDecl, ''];
    const base = this.fileName(name);
    const main = this.procs[0];
    const out: PostFile[] = [];
    const body: string[] = [];
    const subs = this.procs.slice(1);
    for (const p of subs) body.push(`\tPROC ${p.name}()`, ...p.parts[0].map((l) => `\t\t${l}`), '\tENDPROC', '');
    if (main.parts.length > 1) {
      const n = main.parts.length, path = '/hd0a/Enter-Serial-Number/HOME/RoboDK', ext = this.PROG_EXT;
      const file = (i: number) => `${path}/${base}_${i}.${ext}`;
      body.push('', '\tLOCAL PERS string modulepath1 := "";', '\tLOCAL PERS string modulepath2 := "";', '\tLOCAL VAR loadsession load1;', '\tLOCAL VAR loadsession load2;', '\tLOCAL VAR string module_unload := "";', '\t');
      body.push('\tPROC Main()', '\t\t! This program has automatically been split into smaller subprograms', '\t\tTPErase;', '');
      for (let i = 1; i <= n; i++) {
        const c = i % 2 === 1 ? 1 : 2, o = 3 - c;
        body.push(`\t\tTPWrite "Starting Subprogram ${i}/${n}";`);
        if (i === 1) body.push('\t\t! Load the first program (may take some time)', `\t\tmodulepath1 := "${file(0)}";`, '\t\tLoad \\Dynamic, modulepath1;');
        else body.push('\t\t! Wait for the program to be ready (fast)', `\t\tWaitLoad load${c};`);
        if (i < n) body.push(`\t\tmodulepath${o} := "${file(i)}";`, `\t\tStartLoad modulepath${o}, load${o};`);
        body.push(`\t\t%"MOD_Tmp${i}:Prg_Tmp${i}"%;`, `\t\tUnload modulepath${c};`, `\t\tmodulepath${c} := "";`, '', '');
      }
      body.push('\t\tTPWrite "Program completed";', '', '\tENDPROC', '');
      main.parts.forEach((lines, i) => {
        out.push({ name: `${base}_${i}.${ext}`, content: [...head, `MODULE MOD_Tmp${i + 1}`, '', `\tPROC Prg_Tmp${i + 1}()`, '', ...lines.map((l) => `\t\t${l}`), '', '\tENDPROC', '', 'ENDMODULE'].join(nl) + nl });
      });
    } else body.push('\tPROC Main()', ...main.parts[0].map((l) => `\t\t${l}`), '\tENDPROC', '');
    out.unshift({ name: `${base}.${this.PROG_EXT}`, content: [...head, `MODULE ${this.variant === 'S4C' ? this.modName.slice(0, 17) : this.modName}`, ...decl, ...body, 'ENDMODULE'].join(nl) + nl });
    return out;
  }

  protected conf(joints: number[], conf: number[]): string {
    const c = (i: number) => Math.floor((joints[i] ?? 0) / 90);
    const cfx = joints.length >= 6 ? 4 * (conf[0] ?? 0) + 2 * (conf[1] ?? 0) + (conf[2] ?? 0) : 0;
    return joints.length >= 6 ? `[${c(0)},${c(3)},${c(5)},${cfx}]` : '[0,0,0,0]';
  }
  protected extax(joints: number[]): string {
    const e = joints.slice(6, Math.max(6, Math.min(this.nAxes, joints.length)));
    return `[${[0, 1, 2, 3, 4, 5].map((i) => (e[i] !== undefined ? fx(e[i], 6) : '9E+09')).join(',')}]`;
  }
  protected robtarget(pose: Mat4, joints: number[], conf: number[]) { return `[${rapidPose(pose).slice(1, -1)},${this.conf(joints, conf)},${this.extax(joints)}]`; }
  protected tail() { return `${this.speed},${this.zone},${this.tool} \\WObj:=${this.wobj};`; }
  MoveJ(_pose: Mat4 | null, joints: number[]): void {
    this.splitIfNeeded();
    this.addline(`MoveAbsJ [[${[0, 1, 2, 3, 4, 5].map((i) => fx(joints[i] ?? 0, 6)).join(',')}],${this.extax(joints)}],${this.tail()}`);
  }
  MoveL(pose: Mat4 | null, joints: number[], conf: number[]): void {
    this.splitIfNeeded();
    // a linear move to a joint target: RoboDK writes MoveL CalcRobT(jointtarget, tool \WObj)
    if (!pose) { this.addline(`MoveL CalcRobT([[${[0, 1, 2, 3, 4, 5].map((i) => fx(joints[i] ?? 0, 6)).join(',')}],${this.extax(joints)}],${this.tool} \\WObj:=${this.wobj}),${this.tail()}`); return; }
    this.addline(`MoveL ${this.robtarget(pose, joints, conf)},${this.tail()}`);
  }
  override MoveC(pose1: Mat4 | null, joints1: number[], pose2: Mat4 | null, joints2: number[], conf1: number[], conf2: number[]): void {
    if (!pose1 || !pose2) { super.MoveC(pose1, joints1, pose2, joints2, conf1, conf2); return; }
    this.splitIfNeeded();
    this.addline(`MoveC ${this.robtarget(pose1, joints1, conf1)},${this.robtarget(pose2, joints2, conf2)},${this.tail()}`);
  }
  override setTool(pose: Mat4, _id: number, name: string): void {
    const nm = this.noClash(this.ident(name, 't'));
    const data = `[TRUE,${rapidPose(pose)},[1,[0,0,20],[1,0,0,0],0,0,0.005]]`;
    if (this.decls.has(nm)) this.addline(`${nm} := ${data};`);
    else { this.decls.set(nm, 'tool'); this.toolDecl.push(`\tPERS tooldata ${nm} := ${data};`); }
    this.tool = nm;
  }
  override setFrame(pose: Mat4, _id: number, name: string): void {
    const nm = this.variant !== 'S4C' && /base$/i.test(name.trim()) ? 'RobotBase' : this.noClash(this.ident(name, 'w'));
    const p = rapidPose(pose);
    const data = this.variant === 'S4C' ? `[FALSE, TRUE, "", ${p},[[0,0,0],[1,0,0,0]]]` : `[FALSE, TRUE, "", [[0,0,0],[1,0,0,0]],${p}]`;
    if (this.decls.has(nm)) this.addline(`${nm} := ${data};`);
    else { this.decls.set(nm, 'wobj'); this.wobjDecl.push(`\tPERS wobjdata ${nm} := ${data};`); }
    this.wobj = nm;
  }
  override Pause(ms: number): void { this.addline(ms <= 0 ? 'STOP;' : `WaitTime ${fx(ms / 1000)};`); }
  override setSpeed(v: number): void { this.speed = rapidSpeed(v); }
  override setZoneData(z: number): void { this.zone = rapidZone(z); }
  protected bit(v: number | boolean | string) { return (typeof v === 'string' ? ['1', 'true', 'on'].includes(v.toLowerCase()) : Number(v) > 0) ? 1 : 0; }
  override setDO(io: IoName, v: number | boolean | string): void { this.addline(`SetDO ${typeof io === 'number' ? `D_OUT_${io}` : io}, ${this.bit(v)};`); }
  override setAO(io: IoName, v: number | string): void { this.addline(`SetAO ${typeof io === 'number' ? `A_OUT_${io}` : io}, ${v};`); }
  override waitDI(io: IoName, v: number | boolean | string, timeoutMs = -1): void {
    this.addline(`WaitDI ${typeof io === 'number' ? `D_IN_${io}` : io}, ${this.bit(v)}${timeoutMs > 0 ? `, \\MaxTime:=${f1(timeoutMs / 1000)}` : ''};`);
  }
  override RunCode(code: string, isCall = false): void {
    if (!isCall) { this.addline(code); return; }
    const m = code.trim().match(/^([^(]*)(?:\((.*)\))?\s*$/);
    if (!m) { this.addline(`${code};`); return; }
    const name = m[1].trim().replace(/ /g, '');
    if (/^extruder$/i.test(name)) return; // RoboDK: 3D-printing extruder calls drive the synchronized axis, no code
    this.addline(m[2] === undefined ? `${name};` : `${name} ${m[2]};`);
  }
  override RunMessage(msg: string, isComment = false): void { this.addline(isComment ? `! ${msg}` : `TPWrite "${msg}";`); }
  // ---- control flow (RAPID) ----
  override ctlLoop(count: number, body: string): void {
    const b = this.ident(body, 'P');
    if (count <= 0) { this.addline('WHILE TRUE DO'); this.addline(`    ${b};`); this.addline('ENDWHILE'); return; }
    const v = `i${++this.loopVar}`;
    this.addline(`FOR ${v} FROM 1 TO ${Math.trunc(count)} DO`);
    this.addline(`    ${b};`);
    this.addline('ENDFOR');
  }
  override ctlIf(cond: CtlCondition, thenProg: string | null, elseProg: string | null): void {
    const c = cond.io !== undefined ? `${typeof cond.io === 'number' ? `D_IN_${cond.io}` : cond.io} ${cond.op === '!=' ? '<>' : '='} ${this.bit(cond.value ?? 1)}` : cond.text;
    this.addline(`IF ${c} THEN`);
    if (thenProg) this.addline(`    ${this.ident(thenProg, 'P')};`);
    if (elseProg) { this.addline('ELSE'); this.addline(`    ${this.ident(elseProg, 'P')};`); }
    this.addline('ENDIF');
  }
}

function abbPost(id: string, name: string, variant: Variant, aliases: string[]): PostProcessor {
  return {
    id, name, brand: 'ABB', extension: variant === 'IRC5' ? 'mod' : variant === 'OmniCore' ? 'modx' : 'prg', aliases,
    description: `RAPID module for ABB ${variant} controllers (RoboDK ${id} compatible).`,
    generate(p: PostProgram, options: Record<string, any> = {}): PostFile[] { return runRoboPost((ctx) => new AbbPost(ctx, variant), p, options, true); },
  };
}
export const ABB_RAPID = abbPost('ABB_RAPID_IRC5', 'ABB RAPID (IRC5)', 'IRC5', ['ABB_RAPID_IRC5_Robtargets']);
export const ABB_RAPID_OMNICORE = abbPost('ABB_RAPID_OmniCore', 'ABB RAPID (OmniCore)', 'OmniCore', []);
export const ABB_RAPID_S4C = abbPost('ABB_RAPID_S4C', 'ABB RAPID (S4C legacy)', 'S4C', []);
registerPost(ABB_RAPID);
registerPost(ABB_RAPID_OMNICORE);
registerPost(ABB_RAPID_S4C);
