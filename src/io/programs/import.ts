/**
 * Import robot programs written in vendor languages into Program items (with their targets, frames and tools):
 *  - KUKA KRL (.src + .dat): PTP/LIN/CIRC (+ spline S*), inline aggregates (typed {E6POS: ...} / {AXIS: ...} or
 *    plain), .dat points (DECL E6POS / POS / E6AXIS / AXIS / FRAME / FDAT), $BASE / $TOOL (FRAME aggregates,
 *    BASE_DATA / TOOL_DATA, FDAT_ACT), $VEL.CP / $APO.CDIS / C_DIS, E1..E6 external axes, $OUT / WAIT FOR $IN (with
 *    RoboDK's timer blocks), WAIT SEC, HALT, messages, calls and local DEF sub-programs.
 *  - ABB RAPID (.mod/.modx/.prg): every PROC (main = "main" or the first one, the others become called sub-programs),
 *    robtarget / jointtarget / tooldata / wobjdata / speeddata / zonedata declarations, MoveJ/MoveL/MoveC/MoveAbsJ with
 *    Offs() and RelTool(), speeds, zones, tool and \WObj changes, SetDO / Set / Reset / WaitDI \MaxTime, WaitTime,
 *    TPWrite, Stop, procedure calls with arguments.
 *  - Fanuc TP (.ls): /POS joint and Cartesian positions (UF / UT, CONFIG, E1..), J/L/C motions with continuation
 *    lines, % / mm/sec / cm/min / inch/min speeds, CNT / FINE, UFRAME / UTOOL definitions (PR[] writes and RoboDK's
 *    "! UFn:" comments), DO / RO / named outputs, WAIT DI with TIMEOUT blocks, WAIT (sec), CALL / RUN, MESSAGE, Arc
 *    Start / Arc End options.
 *  - URScript (.script): the main def and the defs with motions inside it (sub-programs), movej / movel / movep /
 *    movec with positional or named a, v, r (movej v in rad/s -> joint speed), p[...] and pose_trans(ref_frame, p[...]),
 *    ref_frame / set_tcp, speed / blend variables, standard / tool / configurable IO, while-wait loops, sleep, halt,
 *    popup / textmsg, calls, threads.
 *  - CSV: RoboDK's CSV post format and the studio's Generic CSV.
 * Unknown lines become raw code instructions so nothing is lost.
 */
import { Program, MoveType } from '../../core/items/program';
import { Target, Frame, Item, Station, Tool } from '../../core/items/item';
import { kukaToPose, quatToPose, fanucToPose, urToPose, xyzrpwToPose, RAD, DEG, Mat4, multiply, invert, transl, rotx, roty, rotz, mul, identity } from '../../core/math/pose';
import { Robot } from '../../core/items/robot';

export type ProgramLanguage = 'krl' | 'rapid' | 'ls' | 'urscript' | 'csv' | 'auto';

export interface ImportResult {
  /** Main program (first one of the file). */
  program: Program;
  /** Every program created (main first, then sub-programs). */
  programs: Program[];
  targets: Target[];
  frames: Frame[];
  tools: Tool[];
  warnings: string[];
}
export interface ImportOptions {
  station: Station;
  robot?: Robot | null;
  /** Reference for targets when the program sets no frame (default: the robot's active frame, else the station). */
  frame?: Item;
  name?: string;
  language?: ProgramLanguage;
  filename?: string;
  /** Companion files by name (KRL .dat, other RAPID modules / LS programs called by the main one). */
  files?: Record<string, string>;
}

export function detectLanguage(text: string, filename = ''): ProgramLanguage {
  const ext = filename.split('.').pop()?.toLowerCase();
  if (ext === 'src' || ext === 'dat' || (/\bDEF\s+\w+\s*\(/.test(text) && /\b(S?PTP|S?LIN)\b/.test(text))) return 'krl';
  if (ext === 'ls' || /^\/PROG/m.test(text)) return 'ls';
  if (ext === 'mod' || ext === 'modx' || ext === 'sys' || (/\bMODULE\b/.test(text) && /\bENDMODULE\b/.test(text))) return 'rapid';
  if (ext === 'prg' && /\bPROC\b|\bMODULE\b/.test(text)) return 'rapid';
  if (ext === 'prg') return /\bMove[LJC]\b|\bMoveAbsJ\b/.test(text) ? 'rapid' : 'auto';
  if (ext === 'script' || ext === 'urscript' || /\bmovej\(|\bmovel\(/.test(text)) return 'urscript';
  if (ext === 'csv') return 'csv';
  return 'auto';
}

// ---- shared builder ----------------------------------------------------------------------------------------------
type Num6 = [number, number, number, number, number, number];
const num = (s: string) => parseFloat(String(s).replace(',', '.'));
/** Split on top-level commas (ignores commas inside brackets / parentheses / quotes). */
export function splitArgs(s: string): string[] {
  const out: string[] = [];
  let depth = 0, cur = '', q = false;
  for (const ch of s) {
    if (ch === '"') q = !q;
    if (!q) {
      if (ch === '[' || ch === '(' || ch === '{') depth++;
      else if (ch === ']' || ch === ')' || ch === '}') depth--;
      else if (ch === ',' && depth === 0) { out.push(cur.trim()); cur = ''; continue; }
    }
    cur += ch;
  }
  if (cur.trim()) out.push(cur.trim());
  return out;
}
const samePose = (a: Mat4, b: Mat4) => a.every((v, i) => Math.abs(v - b[i]) < 1e-6);

class Builder {
  programs: Program[] = [];
  prog!: Program;
  targets: Target[] = [];
  frames: Frame[] = [];
  tools: Tool[] = [];
  warnings: string[] = [];
  base: Item;
  frame: Item | null = null;
  tool: Tool | null = null;
  n = 0;
  lastSpeed: number | undefined;
  lastSpeedJ: number | undefined;
  lastZone: number | undefined;
  constructor(public opts: ImportOptions) {
    this.base = opts.frame ?? opts.robot?.activeFrame() ?? opts.station;
  }
  get station() { return this.opts.station; }
  get robot() { return this.opts.robot ?? null; }
  newProgram(name: string): Program {
    const p = new Program(name);
    if (this.robot) p.setRobot(this.robot);
    this.station.addChild(p);
    this.programs.push(p);
    this.prog = p;
    this.frame = null; this.tool = null;
    this.lastSpeed = this.lastSpeedJ = this.lastZone = undefined;
    return p;
  }
  /** Frame item for a pose given w.r.t. the robot base (reused when name and pose match). */
  frameItem(name: string, poseInBase: Mat4): Frame {
    const abs = this.robot ? multiply(this.robot.poseAbs(), poseInBase) : poseInBase;
    let f = this.frames.find((x) => x.name === name && samePose(x.poseAbs(), abs));
    if (!f) {
      const dup = this.frames.filter((x) => x.name === name || x.name.startsWith(`${name} (`)).length;
      f = new Frame(dup ? `${name} (${dup + 1})` : name);
      this.station.addChild(f);
      f.setPose(abs);
      this.frames.push(f);
    }
    return f;
  }
  toolItem(name: string, pose: Mat4): Tool | null {
    if (!this.robot) { this.warnings.push(`Tool ${name} ignored (no robot selected)`); return null; }
    let t = this.tools.find((x) => x.name === name && samePose(x.poseTool(), pose));
    if (!t) {
      const dup = this.tools.filter((x) => x.name === name || x.name.startsWith(`${name} (`)).length;
      t = new Tool(dup ? `${name} (${dup + 1})` : name);
      this.robot.addChild(t);
      t.setPoseTool(pose);
      this.tools.push(t);
    }
    return t;
  }
  setFrame(f: Item | null): void { if (f !== this.frame) { this.prog.setFrame(f); this.frame = f; } }
  setTool(t: Tool | null): void { if (t && t !== this.tool) { this.prog.setTool(t); this.tool = t; } }
  speed(v: number | undefined, w?: number): void {
    const sv = v !== undefined && v !== this.lastSpeed ? v : undefined;
    const sw = w !== undefined && w !== this.lastSpeedJ ? w : undefined;
    if (sv === undefined && sw === undefined) return;
    this.prog.setSpeed(sv, sw);
    if (sv !== undefined) this.lastSpeed = sv;
    if (sw !== undefined) this.lastSpeedJ = sw;
  }
  zone(r: number): void { if (r !== this.lastZone) { this.prog.setRounding(r); this.lastZone = r; } }
  target(name: string | undefined, pose: Mat4 | null, joints: number[] | null, parent?: Item): Target {
    this.n++;
    const t = new Target(name || `${this.prog.name}_${this.n}`);
    (parent ?? this.frame ?? this.base).addChild(t);
    if (pose) t.setPose(pose);
    if (joints) { t.joints = joints.slice(); if (!pose) t.isJointTarget = true; }
    if (this.robot) t.robotId = this.robot.id;
    this.targets.push(t);
    return t;
  }
  move(type: MoveType, pose: Mat4 | null, joints: number[] | null, name?: string, parent?: Item): Target {
    const t = this.target(name, pose, joints, parent);
    if (type === 'MoveJ') this.prog.addMoveJ(t); else this.prog.addMoveL(t);
    return t;
  }
  moveC(viaPose: Mat4, viaJoints: number[] | null, pose: Mat4, joints: number[] | null, name?: string, parent?: Item): void {
    const v = this.target(name ? `${name}_via` : undefined, viaPose, viaJoints, parent);
    const t = this.target(name, pose, joints, parent);
    this.prog.addMoveC(v, t);
  }
  result(): ImportResult {
    return { program: this.programs[0], programs: this.programs, targets: this.targets, frames: this.frames, tools: this.tools, warnings: this.warnings };
  }
}

export function importProgram(text: string, opts: ImportOptions): ImportResult {
  const lang = !opts.language || opts.language === 'auto' ? detectLanguage(text, opts.filename) : opts.language;
  const b = new Builder(opts);
  const defName = opts.name ?? opts.filename?.replace(/^.*[\\/]/, '').replace(/\.[^.]+$/, '') ?? 'Imported';
  switch (lang) {
    case 'krl': parseKRL(text, b, defName); break;
    case 'rapid': parseRAPID(text, b, defName); break;
    case 'ls': parseLS(text, b, defName); break;
    case 'urscript': parseURScript(text, b, defName); break;
    case 'csv': b.newProgram(defName); parseCSV(text, b); break;
    default:
      b.newProgram(defName);
      b.warnings.push('Unknown language; imported as raw code');
      text.split(/\r?\n/).forEach((l) => l.trim() && b.prog.runInstruction(l.trim(), false));
  }
  if (!b.programs.length) b.newProgram(defName);
  if (opts.name && b.programs[0]) b.programs[0].name = opts.name;
  return b.result();
}

/** Resolve call targets: calls whose name is an imported program become program calls. */
function linkCalls(b: Builder): void {
  for (const p of b.programs) {
    for (const ins of p.instructions()) {
      const d: any = ins.data;
      if (d.kind !== 'code' || !d.asFunctionCall) continue;
      const code = String(d.code).trim().toLowerCase();
      // exact name, or a controller-truncated program name (Fanuc RJ3 SubPic.LS called as "CALL SubPick")
      const callee = b.programs.find((q) => q !== p && q.name.toLowerCase() === code) ?? b.programs.find((q) => q !== p && q.name.length >= 6 && code.startsWith(q.name.toLowerCase()) && /^\w+$/.test(code));
      if (callee) ins.data = { kind: 'call', programId: callee.id, programName: callee.name };
    }
  }
}

// ---- KUKA KRL ------------------------------------------------------------------------------------------------------
/** Parse a KRL aggregate {X 1,Y 2,...} / {E6POS: X 1, ...} into key -> value. */
function krlAggregate(s: string): Record<string, number> | null {
  const m = s.match(/\{\s*(?:\w+\s*:)?([^{}]*)\}/);
  if (!m) return null;
  const out: Record<string, number> = {};
  for (const part of m[1].split(',')) {
    const kv = part.trim().match(/^([A-Za-z_]\w*)\s+([-+\d.eE]+)$/);
    if (kv) out[kv[1].toUpperCase()] = num(kv[2]);
  }
  return Object.keys(out).length ? out : null;
}
interface KrlPoint { pose: Mat4 | null; joints: number[] | null; ext: number[] }
function krlPoint(a: Record<string, number>): KrlPoint | null {
  const ext = ['E1', 'E2', 'E3', 'E4', 'E5', 'E6'].filter((k) => k in a).map((k) => a[k]);
  if ('A1' in a) return { pose: null, joints: ['A1', 'A2', 'A3', 'A4', 'A5', 'A6'].filter((k) => k in a).map((k) => a[k]), ext };
  if ('X' in a) return { pose: kukaToPose(a.X ?? 0, a.Y ?? 0, a.Z ?? 0, a.A ?? 0, a.B ?? 0, a.C ?? 0), joints: null, ext };
  return null;
}
function parseKRL(text: string, b: Builder, defName: string): void {
  const sources = [text, ...Object.entries(b.opts.files ?? {}).filter(([n]) => /\.dat$/i.test(n)).map(([, t]) => t)];
  const vars = new Map<string, Record<string, number>>();
  const fdat = new Map<string, { tool: number; base: number }>();
  for (const src of sources) {
    for (const m of src.matchAll(/^\s*(?:DECL\s+)?(?:GLOBAL\s+)?(?:DECL\s+)?(E6POS|POS|E6AXIS|AXIS|FRAME)\s+(\w+)\s*=\s*(\{[^}]*\})/gim)) { const a = krlAggregate(m[3]); if (a) vars.set(m[2].toUpperCase(), a); }
    for (const m of src.matchAll(/^\s*(?:DECL\s+)?(?:GLOBAL\s+)?FDAT\s+(\w+)\s*=\s*\{([^}]*)\}/gim)) {
      const t = m[2].match(/TOOL_NO\s+(\d+)/i), bs = m[2].match(/BASE_NO\s+(\d+)/i);
      fdat.set(m[1].toUpperCase(), { tool: t ? +t[1] : 0, base: bs ? +bs[1] : 0 });
    }
  }
  const baseData = new Map<number, Mat4>(), toolData = new Map<number, Mat4>();
  const point = (expr: string): KrlPoint | null => {
    const e = expr.trim();
    if (e.startsWith('{')) { const a = krlAggregate(e); return a ? krlPoint(a) : null; }
    const name = e.match(/^(\w+)/)?.[1]?.toUpperCase();
    if (!name) return null;
    const a = vars.get(name) ?? vars.get(`X${name}`);
    return a ? krlPoint(a) : null;
  };
  const frameFrom = (expr: string): { pose: Mat4; label: string; num?: number } | null => {
    const e = expr.trim();
    let m = e.match(/^(BASE|TOOL)_DATA\[(\d+)\]/i);
    if (m) { const k = +m[2]; const pose = (m[1].toUpperCase() === 'BASE' ? baseData : toolData).get(k) ?? identity(); return { pose, label: `${m[1].toUpperCase() === 'BASE' ? 'Base' : 'Tool'} ${k}`, num: k }; }
    if (/^\$NULLFRAME/i.test(e)) return { pose: identity(), label: 'NULLFRAME' };
    const p = point(e);
    if (p?.pose) return { pose: p.pose, label: '' };
    return null;
  };
  let apo = 0;
  let timeout = -1, inTimer = false;
  const joints = (p: KrlPoint) => (p.joints ? [...p.joints, ...p.ext] : null);
  const setBase = (f: { pose: Mat4; label: string; num?: number }) => { b.setFrame(b.frameItem(f.label || `Frame ${b.frames.length + 1}`, f.pose)); };
  const setTool = (f: { pose: Mat4; label: string; num?: number }) => { b.setTool(b.toolItem(f.label || `Tool ${b.tools.length + 1}`, f.pose)); };
  const zoneFor = (code: string) => { if (/\bC_(DIS|PTP|VEL|ORI)\b/i.test(code)) b.zone(apo > 0 ? apo : 1); else b.zone(-1); };
  let started = false;
  for (const raw of text.split(/\r?\n/)) {
    const line = raw.trim();
    if (!line) continue;
    let m: RegExpMatchArray | null;
    if ((m = line.match(/^(?:GLOBAL\s+)?DEF\s+(\w+)\s*\(/i))) { b.newProgram(started ? m[1] : (b.opts.name ?? m[1])); started = true; apo = 0; continue; }
    if (/^&/.test(line)) continue;
    if (/^DEFDAT\b/i.test(line)) break; // data list (.dat content): declarations only
    if (!started) { b.newProgram(defName); started = true; }
    if (/^(&|END\b|DEFDAT|ENDDAT|EXT\b|DECL\b|;FOLD|;ENDFOLD|;%|BAS\s*\(|INTERRUPT|GLOBAL INTERRUPT|\$ADVANCE|\$BWDSTART|PDAT_ACT|LDAT_ACT|Wait for StrClear|\$LOOP_CONT)/i.test(line)) continue;
    if ((m = line.match(/^FDAT_ACT\s*=\s*(\w+)/i))) {
      const fd = fdat.get(m[1].toUpperCase());
      if (fd) { setBase({ pose: baseData.get(fd.base) ?? identity(), label: fd.base ? `Base ${fd.base}` : 'NULLFRAME' }); if (fd.tool) setTool({ pose: toolData.get(fd.tool) ?? identity(), label: `Tool ${fd.tool}` }); if ((fd.base && !baseData.has(fd.base)) || (fd.tool && !toolData.has(fd.tool))) b.warnings.push(`${m[1]}: BASE_DATA[${fd.base}] / TOOL_DATA[${fd.tool}] are defined on the controller; identity used`); }
      continue;
    }
    if (line.startsWith(';')) { b.prog.comment(line.slice(1).trim()); continue; }
    const semi = line.indexOf(';');
    const code = (semi >= 0 ? line.slice(0, semi) : line).trim();
    const cmt = semi >= 0 ? line.slice(semi + 1).trim() : undefined;
    // RoboDK's WAIT-with-timeout block
    if (/^START_TIMER:|^\$TIMER_STOP\[1\]|^\$TIMER_FLAG\[1\]|^IF \$TIMER_FLAG\[1\]|^GOTO START_TIMER/i.test(code)) { if (/^IF/i.test(code)) inTimer = true; continue; }
    if ((m = code.match(/^\$TIMER\[1\]\s*=\s*([-\d.]+)/i))) { timeout = num(m[1]) * 1000; continue; }
    if (inTimer && /^(HALT|ENDIF)$/i.test(code)) { if (/^ENDIF$/i.test(code)) inTimer = false; continue; }
    if ((m = code.match(/^(BASE|TOOL)_DATA\[(\d+)\]\s*=\s*(.+)$/i))) { const p = point(m[3]); if (p?.pose) (m[1].toUpperCase() === 'BASE' ? baseData : toolData).set(+m[2], p.pose); continue; }
    if ((m = code.match(/^\$BASE\s*=\s*(.+)$/i))) { const f = frameFrom(m[1]); if (f) setBase({ ...f, label: f.label || cmt || '' }); else b.prog.runInstruction(code, false); continue; }
    if ((m = code.match(/^\$TOOL\s*=\s*(.+)$/i))) { const f = frameFrom(m[1]); if (f) setTool({ ...f, label: f.label || cmt || '' }); else b.prog.runInstruction(code, false); continue; }
    if ((m = code.match(/^S?PTP\s+(.+)$/i)) && !/^PTP_REL/i.test(code)) {
      const p = point(m[1].replace(/\s+C_(PTP|DIS|VEL|ORI)\s*$/i, ''));
      if (!p) { b.prog.runInstruction(code, false); b.warnings.push(`unknown point in "${code}"`); continue; }
      zoneFor(code);
      b.move('MoveJ', p.pose, joints(p), cmt);
      continue;
    }
    if ((m = code.match(/^S?LIN\s+(.+)$/i)) && !/^LIN_REL/i.test(code)) {
      const p = point(m[1].replace(/\s+C_(PTP|DIS|VEL|ORI)\s*$/i, ''));
      if (!p?.pose) { b.prog.runInstruction(code, false); b.warnings.push(`unknown point in "${code}"`); continue; }
      zoneFor(code);
      b.move('MoveL', p.pose, joints(p), cmt);
      continue;
    }
    if ((m = code.match(/^S?CIRC\s+(.+)$/i)) && !/^CIRC_REL/i.test(code)) {
      const args = splitArgs(m[1].replace(/\s+C_(PTP|DIS|VEL|ORI)\s*$/i, ''));
      const v = point(args[0] ?? ''), e = point(args[1] ?? '');
      if (!v?.pose || !e?.pose) { b.prog.runInstruction(code, false); b.warnings.push(`unknown points in "${code}"`); continue; }
      if (args[2] && /^CA\b/i.test(args[2])) b.warnings.push(`CIRC with circle angle (${args[2]}) imported up to the end point`);
      zoneFor(code);
      b.moveC(v.pose, joints(v), e.pose, joints(e), cmt);
      continue;
    }
    if ((m = code.match(/^\$VEL\.CP\s*=\s*([-\d.eE]+)/i))) { b.speed(num(m[1]) * 1000); continue; }
    if ((m = code.match(/^\$VEL\.ORI1\s*=\s*([-\d.eE]+)/i))) { b.speed(undefined, num(m[1])); continue; }
    if (/^\$VEL\.ORI2|^\$ACC\.ORI|^\$APO\.CPTP|^\$ACC\.CP|^\$VEL_AXIS|^\$ACC_AXIS/i.test(code)) continue;
    if ((m = code.match(/^\$APO\.CDIS\s*=\s*([-\d.eE]+)/i))) { apo = num(m[1]); continue; }
    if ((m = code.match(/^WAIT\s+SEC\s+([-\d.eE]+)/i))) { b.prog.pause(num(m[1]) * 1000); continue; }
    if ((m = code.match(/^\$OUT\[(\d+)\]\s*=\s*(TRUE|FALSE)/i))) { b.prog.setDO(m[1], m[2].toUpperCase() === 'TRUE'); continue; }
    if ((m = code.match(/^OUT\s+(\d+)\s+'[^']*'\s+State\s*=\s*(TRUE|FALSE)/i))) { b.prog.setDO(m[1], m[2].toUpperCase() === 'TRUE'); continue; }
    if ((m = code.match(/^WAIT\s+FOR\s+\(?\s*\$IN\[(\d+)\]\s*(?:==\s*(TRUE|FALSE))?/i))) { b.prog.waitDI(m[1], (m[2] ?? 'TRUE').toUpperCase() === 'TRUE', timeout); timeout = -1; continue; }
    if ((m = code.match(/^WAIT\s+FOR\s+\(?\s*(\w+)\s*==\s*(TRUE|FALSE)/i))) { b.prog.waitDI(m[1], m[2].toUpperCase() === 'TRUE', timeout); timeout = -1; continue; }
    if (/^HALT\b/i.test(code)) { b.prog.pause(-1); continue; }
    // RoboDK writes named outputs as NAME=TRUE / NAME=FALSE
    if ((m = code.match(/^([A-Za-z_]\w*)\s*=\s*(TRUE|FALSE)$/i))) { b.prog.setDO(m[1], m[2].toUpperCase() === 'TRUE'); continue; }
    if ((m = code.match(/^\$LOOP_MSG\[\]\s*=\s*"(.*)"$/i)) || (m = code.match(/^MsgNotify\s*\(\s*"(.*)"\s*\)$/i))) { b.prog.showMessage(m[1]); continue; }
    if ((m = code.match(/^(\w+)\s*\(\s*\)$/))) { b.prog.runInstruction(m[1], true); continue; }
    if ((m = code.match(/^(\w+)\s*\((.*)\)$/)) && !/^(IF|WHILE|FOR|SWITCH|WAIT)$/i.test(m[1])) { b.prog.runInstruction(`${m[1]}(${m[2].trim()})`, true); continue; }
    if (code) b.prog.runInstruction(code, false);
  }
  // companion .src files (sub-programs written by the post in separate files)
  const companions = Object.entries(b.opts.files ?? {}).filter(([n]) => /\.src$/i.test(n));
  if (companions.length && !(b.opts as any)._companion) {
    const main = b.programs.slice();
    for (const [n, t] of companions) {
      const sub = new Builder({ ...b.opts, name: undefined, files: Object.fromEntries(Object.entries(b.opts.files ?? {}).filter(([k]) => /\.dat$/i.test(k))), _companion: true } as any);
      sub.frames = b.frames; sub.tools = b.tools; sub.targets = b.targets; sub.warnings = b.warnings;
      parseKRL(t, sub, n.replace(/\.src$/i, ''));
      b.programs.push(...sub.programs);
    }
    b.programs.splice(0, b.programs.length, ...main, ...b.programs.filter((p) => !main.includes(p)));
  }
  linkCalls(b);
}

// ---- ABB RAPID -----------------------------------------------------------------------------------------------------
function rapidPoseOf(s: string): { pose: Mat4; ext: number[] } | null {
  const m = s.match(/\[\s*\[\s*([-\d.eE+]+)\s*,\s*([-\d.eE+]+)\s*,\s*([-\d.eE+]+)\s*\]\s*,\s*\[\s*([-\d.eE+]+)\s*,\s*([-\d.eE+]+)\s*,\s*([-\d.eE+]+)\s*,\s*([-\d.eE+]+)\s*\]/);
  if (!m) return null;
  const ext = (s.match(/\]\s*,\s*\[[^\]]*\]\s*,\s*\[([^\]]*)\]\s*\]\s*$/)?.[1] ?? '').split(',').map((v) => num(v)).filter((v) => Number.isFinite(v) && Math.abs(v) < 8e9);
  return { pose: quatToPose(num(m[1]), num(m[2]), num(m[3]), [num(m[4]), num(m[5]), num(m[6]), num(m[7])]), ext };
}
function parseRAPID(text: string, b: Builder, defName: string): void {
  const all = [text, ...Object.entries(b.opts.files ?? {}).filter(([n]) => /\.(mod|modx|prg|sys)$/i.test(n)).map(([, t]) => t)].join('\n');
  const decl = new Map<string, { type: string; value: string }>();
  for (const m of all.matchAll(/(?:LOCAL\s+)?(?:CONST|PERS|VAR|TASK\s+PERS)\s+(robtarget|jointtarget|tooldata|wobjdata|speeddata|zonedata)\s+(\w+)\s*:=\s*(\[[\s\S]*?\]);/gi)) decl.set(m[2].toLowerCase(), { type: m[1].toLowerCase(), value: m[3] });
  const val = (name: string, type: string) => { const d = decl.get(name.trim().toLowerCase()); return d && d.type === type ? d.value : null; };
  /** Pose of a robtarget argument: literal, variable, Offs(p,x,y,z), RelTool(p,x,y,z \Rx:= \Ry:= \Rz:=). */
  const pose = (arg: string): { pose: Mat4; ext: number[] } | null => {
    const a = arg.trim();
    let m = a.match(/^Offs\s*\((.*)\)$/i);
    if (m) { const p = splitArgs(m[1]); const base = pose(p[0]); if (!base) return null; const r = Float64Array.from(base.pose); r[12] += num(p[1]); r[13] += num(p[2]); r[14] += num(p[3]); return { pose: r, ext: base.ext }; }
    m = a.match(/^RelTool\s*\((.*)\)$/i);
    if (m) {
      const p = splitArgs(m[1].replace(/\\R([xyz])\s*:=\s*/gi, ',R$1='));
      const base = pose(p[0]); if (!base) return null;
      const ang = (k: string) => { const e = p.find((x) => x.toUpperCase().startsWith(`R${k}=`)); return e ? num(e.split('=')[1]) * DEG : 0; };
      return { pose: mul(base.pose, transl(num(p[1]), num(p[2]), num(p[3])), rotx(ang('X')), roty(ang('Y')), rotz(ang('Z'))), ext: base.ext };
    }
    if (a.startsWith('[')) return rapidPoseOf(a);
    const v = val(a, 'robtarget');
    return v ? rapidPoseOf(v) : null;
  };
  const speedOf = (s: string): number | undefined => {
    const t = s.trim();
    if (/^vmax$/i.test(t)) return 5000;
    let m = t.match(/^v(\d+(?:\.\d+)?)$/i);
    if (m) return num(m[1]);
    if (t.startsWith('[')) { m = t.match(/^\[\s*([-\d.eE+]+)/); return m ? num(m[1]) : undefined; }
    const v = val(t, 'speeddata');
    m = v?.match(/^\[\s*([-\d.eE+]+)/) ?? null;
    return m ? num(m[1]) : undefined;
  };
  const zoneOf = (s: string): number | undefined => {
    const t = s.trim();
    if (/^fine$/i.test(t)) return -1;
    let m = t.match(/^z(\d+(?:\.\d+)?)$/i);
    if (m) return num(m[1]);
    const v = t.startsWith('[') ? t : val(t, 'zonedata');
    m = v?.match(/^\[\s*(TRUE|FALSE)\s*,\s*([-\d.eE+]+)/i) ?? null;
    return m ? (m[1].toUpperCase() === 'TRUE' ? -1 : num(m[2])) : undefined;
  };
  const toolOf = (name: string) => {
    const t = name.trim();
    if (/^tool0$/i.test(t)) return b.toolItem('tool0', identity());
    const v = val(t, 'tooldata');
    const p = v ? rapidPoseOf(v.replace(/^\[\s*(TRUE|FALSE)\s*,/i, '[')) : null;
    return b.toolItem(t, p?.pose ?? identity());
  };
  const wobjOf = (name: string): Item | null => {
    const t = name.trim();
    if (/^wobj0$/i.test(t)) return null;
    const v = val(t, 'wobjdata');
    if (!v) { b.warnings.push(`wobjdata ${t} not declared: targets kept in the current frame`); return b.frame; }
    const poses = [...v.matchAll(/\[\s*\[\s*([-\d.eE+]+)\s*,\s*([-\d.eE+]+)\s*,\s*([-\d.eE+]+)\s*\]\s*,\s*\[\s*([-\d.eE+]+)\s*,\s*([-\d.eE+]+)\s*,\s*([-\d.eE+]+)\s*,\s*([-\d.eE+]+)\s*\]\s*\]/g)];
    const q = (m: RegExpMatchArray) => quatToPose(num(m[1]), num(m[2]), num(m[3]), [num(m[4]), num(m[5]), num(m[6]), num(m[7])]);
    const uf = poses[0] ? q(poses[0]) : identity(), of = poses[1] ? q(poses[1]) : identity();
    return b.frameItem(t, multiply(uf, of));
  };
  // PROCs (main first)
  const procs: Array<{ name: string; body: string[] }> = [];
  let cur: { name: string; body: string[] } | null = null;
  for (const raw of text.split(/\r?\n/)) {
    const line = raw.trim();
    const m = line.match(/^(?:LOCAL\s+)?PROC\s+(\w+)\s*\(/i);
    if (m) { cur = { name: m[1], body: [] }; procs.push(cur); continue; }
    if (/^ENDPROC\b/i.test(line)) { cur = null; continue; }
    if (cur) cur.body.push(line);
  }
  if (!procs.length) procs.push({ name: defName, body: text.split(/\r?\n/).map((l) => l.trim()) });
  const mainIdx = Math.max(0, procs.findIndex((p) => /^main$/i.test(p.name)));
  const order = [procs[mainIdx], ...procs.filter((_, i) => i !== mainIdx)];
  const modName = text.match(/^\s*MODULE\s+(\w+)/im)?.[1]?.replace(/^MOD_/, '');
  order.forEach((p, i) => {
    b.newProgram(i === 0 ? (b.opts.name ?? (/^main$/i.test(p.name) && modName ? modName : p.name)) : p.name);
    for (const line of p.body) {
      if (!line) continue;
      if (line.startsWith('!')) { b.prog.comment(line.slice(1).trim()); continue; }
      if (/^(ConfJ|ConfL|SingArea|%%%|VERSION:|LANGUAGE:|LOCAL\s+(PERS|VAR|CONST)|(CONST|PERS|VAR)\s)/i.test(line)) continue;
      const bang = line.indexOf('!');
      const code = (bang > 0 && !line.slice(0, bang).includes('"') ? line.slice(0, bang) : line).trim().replace(/;\s*$/, '');
      const cmt = bang > 0 ? line.slice(bang + 1).trim() : undefined;
      let m: RegExpMatchArray | null;
      if ((m = code.match(/^(MoveL|MoveJ|MoveC|MoveAbsJ|MoveLDO|MoveJDO)\s+(.+)$/i))) {
        const kind = m[1].toLowerCase();
        const args = splitArgs(m[2].replace(/\\(V|Z|T|Inpos)\s*:=\s*[^,\\]+/gi, ''));
        // trailing \WObj:= on the tool argument
        const last = args[args.length - 1] ?? '';
        const wm = last.match(/^(.*?)\\WObj\s*:=\s*(\w+)\s*$/i);
        const toolArg = wm ? wm[1].trim() : last.trim();
        const nPos = kind === 'movec' ? 2 : 1;
        const sp = speedOf(args[nPos] ?? ''), zn = zoneOf(args[nPos + 1] ?? '');
        b.setTool(toolOf(toolArg.replace(/\\.*$/, '')));
        b.setFrame(wm ? wobjOf(wm[2]) : null);
        if (zn !== undefined) b.zone(zn);
        if (kind === 'moveabsj') {
          const data = args[0].startsWith('[') ? args[0] : val(args[0], 'jointtarget') ?? '';
          const jm = data.match(/\[\s*\[([^\]]*)\]\s*,\s*\[([^\]]*)\]\s*\]/);
          if (sp !== undefined) b.speed(sp);
          if (!jm) { b.prog.runInstruction(code, false); b.warnings.push(`unknown jointtarget in "${code}"`); continue; }
          const q = jm[1].split(',').map(num), e = jm[2].split(',').map(num).filter((v) => Math.abs(v) < 8e9);
          b.move('MoveJ', null, [...q, ...e], cmt ?? (args[0].startsWith('[') ? undefined : args[0]));
          continue;
        }
        if (sp !== undefined) b.speed(sp);
        if (kind === 'movec') {
          const v = pose(args[0]), e = pose(args[1]);
          if (!v || !e) { b.prog.runInstruction(code, false); b.warnings.push(`unknown robtarget in "${code}"`); continue; }
          b.moveC(v.pose, null, e.pose, null, cmt ?? (args[1].startsWith('[') ? undefined : args[1]));
          continue;
        }
        const p = pose(args[0]);
        if (!p) { b.prog.runInstruction(code, false); b.warnings.push(`unknown robtarget in "${code}"`); continue; }
        b.move(kind.startsWith('movej') ? 'MoveJ' : 'MoveL', p.pose, null, cmt ?? (args[0].startsWith('[') || /^(Offs|RelTool)/i.test(args[0]) ? undefined : args[0]));
        if (kind === 'moveldo' || kind === 'movejdo') { const io = splitArgs(m[2]); b.prog.setDO(io[io.length - 2] ?? '', io[io.length - 1] === '1'); }
        continue;
      }
      if ((m = code.match(/^WaitTime\s+(?:\\InPos\s*,\s*)?([-\d.eE]+)/i))) { b.prog.pause(num(m[1]) * 1000); continue; }
      if ((m = code.match(/^SetDO\s+(?:\\\w+(?:\s*:=\s*[^,]+)?\s*,\s*)?(\w+)\s*,\s*(\d)/i))) { b.prog.setDO(m[1].replace(/^D_OUT_/i, ''), m[2] === '1'); continue; }
      if ((m = code.match(/^(Set|Reset)\s+(\w+)$/i))) { b.prog.setDO(m[2].replace(/^D_OUT_/i, ''), m[1].toLowerCase() === 'set'); continue; }
      if ((m = code.match(/^WaitDI\s+(\w+)\s*,\s*(\d)(?:\s*(?:,\s*)?\\MaxTime\s*:=\s*([-\d.eE]+))?/i))) { b.prog.waitDI(m[1].replace(/^D_IN_/i, ''), m[2] === '1', m[3] ? num(m[3]) * 1000 : -1); continue; }
      if ((m = code.match(/^TPWrite\s+"(.*)"$/i))) { b.prog.showMessage(m[1]); continue; }
      if (/^(Stop|Break|EXIT)\b/i.test(code)) { b.prog.pause(-1); continue; }
      if ((m = code.match(/^(\w+)$/)) && !/^(RETURN|ENDIF|ELSE|ENDFOR|ENDWHILE|ENDTEST)$/i.test(m[1])) { b.prog.runInstruction(m[1], true); continue; }
      // procedure call with arguments: "SetRPM 5000" -> SetRPM(5000)
      if ((m = code.match(/^([A-Za-z]\w*)\s+([^:=]+)$/)) && !/^(IF|WHILE|FOR|TEST|CASE|GOTO|RETURN|RAISE|ELSEIF|ELSE)$/i.test(m[1])) { b.prog.runInstruction(`${m[1]}(${splitArgs(m[2]).join(',')})`, true); continue; }
      b.prog.runInstruction(code, false);
    }
  });
  linkCalls(b);
}

// ---- Fanuc LS -----------------------------------------------------------------------------------------------------
interface LsPos { pose: Mat4 | null; joints: number[] | null; name?: string; uf?: number; ut?: number; conf?: [number, number, number] }
function parseLsPositions(text: string): Map<number, LsPos> {
  const posSection = text.split(/^\/POS/m)[1] ?? '';
  const positions = new Map<number, LsPos>();
  for (const m of posSection.matchAll(/P\[(\d+)(?::"([^"]*)")?\]\s*\{([\s\S]*?)\};/g)) {
    const body = m[3];
    const g = (k: string) => { const r = body.match(new RegExp(`(?:^|[\\s,])${k}\\s*=\\s*([-\\d.eE+]+)`)); return r ? num(r[1]) : null; };
    const uf = body.match(/UF\s*:\s*(\d+)/)?.[1], ut = body.match(/UT\s*:\s*(\d+)/)?.[1];
    const ext = ['E1', 'E2', 'E3'].map(g).filter((v): v is number => v !== null);
    const cfg = body.match(/CONFIG\s*:\s*'([NF])\s*([UD])\s*([TB])/);
    const conf: [number, number, number] | undefined = cfg ? [cfg[3] === 'B' ? 1 : 0, cfg[2] === 'D' ? 1 : 0, cfg[1] === 'F' ? 1 : 0] : undefined;
    const base = { name: m[2], uf: uf ? +uf : undefined, ut: ut ? +ut : undefined, conf };
    if (/J1\s*=/.test(body)) positions.set(+m[1], { ...base, pose: null, joints: [...['J1', 'J2', 'J3', 'J4', 'J5', 'J6'].map((k) => g(k)).filter((v): v is number => v !== null), ...ext] });
    else positions.set(+m[1], { ...base, pose: fanucToPose(g('X') ?? 0, g('Y') ?? 0, g('Z') ?? 0, g('W') ?? 0, g('P') ?? 0, g('R') ?? 0), joints: ext.length ? ext : null });
  }
  return positions;
}
function parseLS(text: string, b: Builder, defName: string): void {
  const texts = [text, ...Object.entries(b.opts.files ?? {}).filter(([n]) => /\.ls$/i.test(n)).map(([, t]) => t)];
  for (const t of texts) parseOneLS(t, b, defName);
  linkCalls(b);
}
function parseOneLS(text: string, b: Builder, defName: string): void {
  const name = text.match(/^\/PROG\s+(\w+)/m)?.[1] ?? defName;
  b.newProgram(b.programs.length === 0 ? (b.opts.name ?? name) : name);
  const positions = parseLsPositions(text);
  const main = text.split(/^\/MN/m)[1]?.split(/^\/POS/m)[0] ?? '';
  // join circular continuation lines ("C P[3] " + "   P[4] 100mm/sec CNT10 ;") into one statement
  const stmts: string[] = [];
  for (const raw of main.split(/\r?\n/)) {
    if (!raw.trim()) continue;
    if (/^\s*\d+:/.test(raw) || !stmts.length) stmts.push(raw.replace(/^\s*\d+:\s*/, '').trim());
    else stmts[stmts.length - 1] += ` ${raw.trim()}`;
  }
  const ufData = new Map<number, Mat4>(), utData = new Map<number, Mat4>();
  const pr = new Map<number, number[]>();
  // RoboDK writes "! UFn:x,y,z,w,p,r" / "! UTn:..." after selecting a numbered frame / tool
  for (const s of stmts) {
    const m = s.match(/^!\s*U([FT])(\d+):\s*([-\d.eE+]+(?:\s*,\s*[-\d.eE+]+){5})/);
    if (m) { const v = m[3].split(',').map(num) as Num6; (m[1] === 'F' ? ufData : utData).set(+m[2], fanucToPose(...v)); }
  }
  let uf = 0, ut = 0, pendingLbl: number | null = null, timeout = -1;
  const frameFor = (k: number): Item | null => (k > 0 && ufData.has(k) ? b.frameItem(`UFrame ${k}`, ufData.get(k)!) : null);
  for (const s of stmts) {
    const line = s.replace(/;\s*$/, '').trim();
    if (!line) continue;
    let m: RegExpMatchArray | null;
    if ((m = line.match(/^PR\[(\d+),(\d)\]\s*=\s*([-\d.eE+]+)/))) { const arr = pr.get(+m[1]) ?? [0, 0, 0, 0, 0, 0]; arr[+m[2] - 1] = num(m[3]); pr.set(+m[1], arr); continue; }
    if ((m = line.match(/^U(FRAME|TOOL)\[(\d+)\]\s*=\s*PR\[(\d+)\]/))) { const v = (pr.get(+m[3]) ?? [0, 0, 0, 0, 0, 0]) as Num6; (m[1] === 'FRAME' ? ufData : utData).set(+m[2], fanucToPose(...v)); continue; }
    if ((m = line.match(/^UFRAME_NUM\s*=\s*(\d+)/))) {
      uf = +m[1];
      if (uf === 0) b.setFrame(null); else if (ufData.has(uf)) b.setFrame(frameFor(uf)); else b.prog.comment(line);
      continue;
    }
    if ((m = line.match(/^UTOOL_NUM\s*=\s*(\d+)/))) { ut = +m[1]; if (utData.has(ut)) b.setTool(b.toolItem(`UTool ${ut}`, utData.get(ut)!)); else b.prog.comment(line); continue; }
    if (/^!\s*U[FT]\d+:/.test(line)) continue;
    if ((m = line.match(/^([JLC])\s+P\[(\d+)(?::"[^"]*")?\](?:\s+P\[(\d+)(?::"[^"]*")?\])?\s*([\d.]+)?\s*(%|mm\/sec|cm\/min|inch\/min|deg\/sec|sec|msec)?\s*(FINE|CNT\s*\d+|CD\s*\d+)?(.*)$/i))) {
      const type = m[1].toUpperCase();
      const p = positions.get(+m[2]);
      const p2 = m[3] ? positions.get(+m[3]) : undefined;
      if (!p || (m[3] && !p2)) { b.warnings.push(`P[${!p ? m[2] : m[3]}] not found`); continue; }
      const v = m[4] !== undefined ? num(m[4]) : undefined, unit = (m[5] ?? '').toLowerCase();
      const lin = unit === 'mm/sec' ? v : unit === 'cm/min' && v !== undefined ? (v * 10) / 60 : unit === 'inch/min' && v !== undefined ? (v * 25.4) / 60 : undefined;
      if (lin !== undefined) b.speed(lin);
      if (m[6]) b.zone(/^FINE/i.test(m[6]) ? -1 : num(m[6].replace(/\D+/, '')));
      const pf = p.uf !== undefined && p.uf !== uf ? (p.uf === 0 ? b.base : frameFor(p.uf) ?? undefined) : undefined;
      if (type === 'C' && p2) {
        if (p.pose && p2.pose) b.moveC(p.pose, p.joints, p2.pose, p2.joints, p2.name, pf);
        else { b.warnings.push('circular move with joint positions imported as two linear moves'); b.move('MoveL', p.pose, p.joints, p.name, pf); b.move('MoveL', p2.pose, p2.joints, p2.name, pf); }
      } else {
        const t = b.move(type === 'J' ? 'MoveJ' : 'MoveL', p.pose, p.joints, p.name, pf);
        if (p.conf) t.config = p.conf;
      }
      const rest = m[7] ?? '';
      if (/Arc\s+End\[/i.test(rest)) b.prog.runInstruction('ArcEnd', true);
      if (/Arc\s+Start\[/i.test(rest)) b.prog.runInstruction('ArcStart', true);
      continue;
    }
    if ((m = line.match(/^Weave\s+Sine\[(.*)\]$/i))) { if (m[1] !== '2.0Hz,8.0mm,0.075s,0.075') { const last = b.prog.instructions().at(-1); if (last && (last.data as any).code === 'ArcStart') (last.data as any).code = `ArcStart(${m[1]})`; } continue; }
    if ((m = line.match(/^\$WAITTMOUT\s*=\s*(\d+)/i))) { timeout = +m[1]; continue; }
    if ((m = line.match(/^WAIT\s+([\d.]+)\s*\(sec\)/i))) { b.prog.pause(num(m[1]) * 1000); continue; }
    if ((m = line.match(/^(DO|RO)\[(\d+)\]\s*=\s*(ON|OFF)/i))) { b.prog.setDO(m[1].toUpperCase() === 'DO' ? m[2] : `RO[${m[2]}]`, m[3].toUpperCase() === 'ON'); continue; }
    if ((m = line.match(/^WAIT\s+(?:DI\[(\d+)\]|(\w+))\s*=\s*(ON|OFF)(?:\s+TIMEOUT,\s*LBL\[(\d+)\])?/i))) {
      b.prog.waitDI(m[1] ?? m[2], m[3].toUpperCase() === 'ON', m[4] ? timeout : -1);
      if (m[4]) pendingLbl = +m[4];
      timeout = -1;
      continue;
    }
    if (pendingLbl !== null) {
      if (new RegExp(`^MESSAGE\\[Timed out for LBL ${pendingLbl}\\]$`).test(line) || /^PAUSE$/i.test(line)) continue;
      if (new RegExp(`^LBL\\[${pendingLbl}\\]$`).test(line)) { pendingLbl = null; continue; }
      pendingLbl = null;
    }
    if ((m = line.match(/^(\w+)\s*=\s*(ON|OFF)$/i))) { b.prog.setDO(m[1], m[2].toUpperCase() === 'ON'); continue; }
    if ((m = line.match(/^CALL\s+(\w+)(\(.*\))?/i))) { b.prog.runInstruction(`${m[1]}${m[2] ?? ''}`, true); continue; }
    if ((m = line.match(/^RUN\s+(\w+)/i))) { b.prog.runInstruction(m[1], true); b.warnings.push(`RUN ${m[1]} (parallel task) imported as a call`); continue; }
    if ((m = line.match(/^MESSAGE\[(.*)\]$/i))) { b.prog.showMessage(m[1]); continue; }
    if (line.startsWith('!')) { b.prog.comment(line.slice(1).trim()); continue; }
    if (/^PAUSE$/i.test(line)) { b.prog.pause(-1); continue; }
    b.prog.runInstruction(line, false);
  }
}

// ---- URScript -------------------------------------------------------------------------------------------------------
interface UrDef { name: string; lines: string[]; children: UrDef[]; motions: boolean }
function parseURScript(text: string, b: Builder, defName: string): void {
  const lines = text.split(/\r?\n/);
  // block structure: def / if / while / for / thread open a block closed by "end" (elif / else continue it)
  const root: UrDef = { name: defName, lines: [], children: [], motions: false };
  const blocks: Array<UrDef | null> = []; // null = control block inside the current def
  let topDef: UrDef | null = null;
  const curDef = () => { for (let k = blocks.length - 1; k >= 0; k--) if (blocks[k]) return blocks[k]!; return null; };
  for (const raw of lines) {
    const t = raw.trim();
    if (!t || (t.startsWith('#') && !curDef())) continue;
    const m = t.match(/^def\s+(\w+)\s*\(([^)]*)\)\s*:/);
    if (m) {
      // functions with parameters (RoboDK's SprayOn(value) helpers) are blocks but never robot sub-programs
      const d: UrDef = { name: m[2].trim() ? `${m[1]}(${m[2]})` : m[1], lines: [], children: [], motions: false };
      const parent = curDef();
      if (parent) parent.children.push(d); else if (!topDef) topDef = d; else root.children.push(d);
      blocks.push(d);
      continue;
    }
    const owner = curDef();
    if (!owner) { if (!/^\w+\(\)$/.test(t) && t !== 'end') root.lines.push(t); continue; }
    if (t === 'end') { const blk = blocks.pop(); if (blk === null) owner.lines.push(t); continue; }
    if (/^(if|while|for|thread)\b.*:$/.test(t)) blocks.push(null);
    if (/^(movej|movel|movep|movec)\(/.test(t)) owner.motions = true;
    owner.lines.push(t);
  }
  const main = topDef ?? root;
  const subs = main.children.filter((d) => (d.motions || d.children.length) && !d.name.includes("("));
  const threadBodies = new Map<string, string>();
  b.newProgram(b.opts.name ?? main.name);
  const mainProg = b.prog;
  const subProgs = new Map<string, Program>();
  for (const s of subs) { b.newProgram(s.name); subProgs.set(s.name, b.prog); }
  const toPose = (expr: string, frame: Mat4 | null): { pose: Mat4; framed: boolean } | null => {
    const e = expr.trim();
    const m = e.match(/p\[\s*([-\d.eE+]+)\s*,\s*([-\d.eE+]+)\s*,\s*([-\d.eE+]+)\s*,\s*([-\d.eE+]+)\s*,\s*([-\d.eE+]+)\s*,\s*([-\d.eE+]+)\s*\]/);
    if (!m) return null;
    const pose = urToPose(num(m[1]) * 1000, num(m[2]) * 1000, num(m[3]) * 1000, num(m[4]), num(m[5]), num(m[6]));
    return { pose, framed: /^pose_trans\(\s*ref_frame\s*,/.test(e) && !!frame };
  };
  const jointsOf = (expr: string) => { const m = expr.trim().match(/^\[([^\]]*)\]$/); return m ? m[1].split(',').map((v) => num(v) * RAD) : null; };
  const run = (d: UrDef, prog: Program) => {
    b.prog = prog;
    b.frame = null; b.tool = null; b.lastSpeed = b.lastSpeedJ = b.lastZone = undefined;
    let refFrame: Mat4 | null = null, refItem: Item | null = null;
    const mainMarker = d.lines.findIndex((l) => /^#\s*Main program:/.test(l));
    let pendingName: string | undefined;
    let blend = 0;
    for (let i = mainMarker >= 0 ? mainMarker + 1 : 0; i < d.lines.length; i++) {
      const line = d.lines[i];
      const hash = line.indexOf('#');
      const code = (hash >= 0 && !line.slice(0, hash).includes('"') ? line.slice(0, hash) : line).trim();
      const cmt = hash >= 0 ? line.slice(hash + 1).trim() : '';
      if (!code) {
        if (/^End of main program/.test(cmt)) break;
        let m = cmt.match(/^Using Ref\.\s+(.*?):\s*p\[/);
        if (m) { pendingName = m[1]; continue; }
        m = cmt.match(/^Using TCP\s+(.*?):\s*p\[/);
        if (m) { pendingName = m[1]; continue; }
        if (/^set_reference\(|^Program generated by|^Using nominal kinematics|^Subprogram\s/.test(cmt)) continue;
        if (cmt) b.prog.comment(cmt);
        continue;
      }
      let m: RegExpMatchArray | null;
      if ((m = code.match(/^(?:global\s+)?ref_frame\s*=\s*(p\[.*\])$/))) { const p = toPose(m[1], null); if (p) { refFrame = p.pose; refItem = b.frameItem(pendingName ?? `Frame ${b.frames.length + 1}`, p.pose); b.setFrame(refItem); } pendingName = undefined; continue; }
      if ((m = code.match(/^set_tcp\((p\[.*\])\)$/))) { const p = toPose(m[1], null); if (p) b.setTool(b.toolItem(pendingName ?? `Tool ${b.tools.length + 1}`, p.pose)); pendingName = undefined; continue; }
      if ((m = code.match(/^(?:global\s+)?speed_ms\s*=\s*([-\d.eE+]+)$/))) { b.speed(num(m[1]) * 1000); continue; }
      if ((m = code.match(/^(?:global\s+)?speed_rads\s*=\s*([-\d.eE+]+)$/))) { b.speed(undefined, num(m[1]) * RAD); continue; }
      if ((m = code.match(/^(?:global\s+)?blend_radius_m\s*=\s*([-\d.eE+]+)$/))) { blend = num(m[1]) * 1000; continue; }
      if (/^(?:global\s+)?(accel_mss|accel_radss)\s*=/.test(code)) continue;
      if ((m = code.match(/^(movej|movel|movep|movec)\((.*)\)$/))) {
        const args = splitArgs(m[2]);
        const named = (k: string) => args.find((a) => new RegExp(`^${k}\\s*=`).test(a))?.split('=')[1]?.trim();
        const pos = args.filter((a) => !/^\w+\s*=/.test(a));
        const isC = m[1] === 'movec';
        const vArg = named('v') ?? pos[isC ? 3 : 2], rArg = named('r') ?? pos[isC ? 4 : 4];
        const lit = (s?: string) => (s !== undefined && /^[-\d.eE+]+$/.test(s) ? num(s) : undefined);
        const v = lit(vArg), r = lit(rArg);
        if (m[1] === 'movej') { if (v !== undefined) b.speed(undefined, v * RAD); } else if (v !== undefined) b.speed(v * 1000);
        const zone = r !== undefined ? r * 1000 : /blend_radius_m/.test(rArg ?? '') ? blend : undefined;
        if (zone !== undefined) b.zone(zone > 0 ? zone : 0);
        const name = cmt || undefined;
        if (isC) {
          const a = toPose(pos[0] ?? '', refFrame), c = toPose(pos[1] ?? '', refFrame);
          if (!a || !c) { b.prog.runInstruction(code, false); continue; }
          b.moveC(a.pose, null, c.pose, null, name, a.framed ? refItem ?? undefined : b.base);
          continue;
        }
        const q = jointsOf(pos[0] ?? '');
        if (q && m[1] === 'movej') { b.move('MoveJ', null, q, name); continue; }
        const p = toPose(pos[0] ?? '', refFrame);
        if (!p) { b.prog.runInstruction(code, false); continue; }
        b.move(m[1] === 'movej' ? 'MoveJ' : 'MoveL', p.pose, null, name, p.framed ? refItem ?? undefined : b.base);
        continue;
      }
      if ((m = code.match(/^sleep\(([-\d.eE+]+)\)$/))) { b.prog.pause(num(m[1]) * 1000); continue; }
      if (/^halt\(\)/.test(code)) { b.prog.pause(-1); continue; }
      if ((m = code.match(/^set_(?:standard_)?digital_out\((\d+)\s*,\s*(True|False|1|0)\)$/))) { b.prog.setDO(m[1], /True|1/.test(m[2])); continue; }
      if ((m = code.match(/^set_tool_digital_out\((\d+)\s*,\s*(True|False|1|0)\)$/))) { b.prog.setDO(String(+m[1] + 10), /True|1/.test(m[2])); continue; }
      if ((m = code.match(/^set_configurable_digital_out\((\d+)\s*,\s*(True|False|1|0)\)$/))) { b.prog.setDO(String(+m[1] + 20), /True|1/.test(m[2])); continue; }
      // RoboDK writes named outputs as an assignment (DO_GRIP = 0 / OUT_A = True)
      if ((m = code.match(/^([A-Z][A-Z0-9_]*)\s*=\s*(True|False|1|0)$/))) { b.prog.setDO(m[1], /True|1/.test(m[2])); continue; }
      if ((m = code.match(/^while\s*\(\s*(?:get_standard_digital_in\((\d+)\)|get_tool_digital_in\((\d+)\)|get_configurable_digital_in\((\d+)\)|(\w+))\s*!=\s*(True|False)\s*\)\s*:$/)) && /^sync\(\)$/.test(d.lines[i + 1] ?? '') && d.lines[i + 2] === 'end') {
        const io = m[1] ?? (m[2] !== undefined ? String(+m[2] + 10) : m[3] !== undefined ? String(+m[3] + 20) : m[4]);
        b.prog.waitDI(io, m[5] === 'True');
        i += 2;
        continue;
      }
      if ((m = code.match(/^(?:popup|textmsg)\(\s*"(.*?)"/))) { b.prog.showMessage(m[1]); continue; }
      if ((m = code.match(/^thread\s+(\w+)\(\)\s*:$/))) { const body = d.lines[i + 1]?.match(/^(\w+)\(\)$/); if (body && d.lines[i + 2] === 'end') { threadBodies.set(m[1], body[1]); i += 2; continue; } }
      if ((m = code.match(/^(?:global\s+)?\w+\s*=\s*run\s+(\w+)\(\)$/))) { const body = threadBodies.get(m[1]) ?? m[1].replace(/^Thread_/, ''); const sp = subProgs.get(body); if (sp) b.prog.startThread(sp); else b.prog.runInstruction(body, true); continue; }
      if ((m = code.match(/^(\w+)\(\)$/))) { const sp = subProgs.get(m[1]); if (sp) b.prog.callProgram(sp); else b.prog.runInstruction(m[1], true); continue; }
      if ((m = code.match(/^(\w+)\((.*)\)$/)) && !/^(if|while|for)$/.test(m[1])) { b.prog.runInstruction(code, true); continue; }
      if (/^(global\s|def\s|thread\s|end$)/.test(code)) continue;
      b.prog.runInstruction(code, false);
    }
  };
  // thread declarations are hoisted before the main program
  for (const l of main.lines) { const m = l.match(/^thread\s+(\w+)\(\)\s*:$/); if (m) { const i = main.lines.indexOf(l); const body = main.lines[i + 1]?.match(/^(\w+)\(\)$/); if (body) threadBodies.set(m[1], body[1]); } }
  run(main, mainProg);
  for (const s of subs) run(s, subProgs.get(s.name)!);
  b.prog = mainProg;
}

// ---- CSV ------------------------------------------------------------------------------------------------------------
function parseCSV(text: string, b: Builder): void {
  const lines = text.split(/\r?\n/);
  if (/^Instruction,X \(mm\)/.test(lines[0] ?? '')) { parseRoboDKCSV(lines, b); return; }
  for (const raw of lines) {
    const line = raw.trim();
    if (!line || line.startsWith('#') || line.startsWith('type,')) continue;
    const c = line.split(',');
    if (c[0] === 'moveJ' || c[0] === 'moveL' || c[0] === 'moveC') {
      const pose = c.slice(2, 8).map(Number);
      const joints = c.slice(8).map(Number).filter((v) => !isNaN(v));
      b.move(c[0] === 'moveJ' ? 'MoveJ' : 'MoveL', pose.every((v) => !isNaN(v)) ? xyzrpwToPose(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]) : null, joints.length ? joints : null, c[1]);
    }
  }
}
/** RoboDK CSV post: "Move Joints,x,y,z,rx,ry,rz,j1..", "Move Circular (1)/(2)", "Set Reference", "Set Tool", "Set DO", ... */
function parseRoboDKCSV(lines: string[], b: Builder): void {
  let via: { pose: Mat4; joints: number[] } | null = null;
  for (const raw of lines.slice(1)) {
    const c = raw.trim().split(',');
    const v = c.slice(1).map(num);
    const pose = () => xyzrpwToPose(v[0], v[1], v[2], v[3], v[4], v[5]);
    switch (c[0]) {
      case 'Program Start': case 'Program Finish': case '': break;
      case 'Comment': if (!/^Program generated by|^Using nominal kinematics/.test(c.slice(1).join(','))) b.prog.comment(c.slice(1).join(',')); break;
      case 'Set Reference': b.setFrame(b.frameItem(`Frame ${b.frames.length + 1}`, pose())); break;
      case 'Set Tool': b.setTool(b.toolItem(`Tool ${b.tools.length + 1}`, pose())); break;
      case 'Set Speed (mm/s)': b.speed(v[0]); break;
      case 'Set Joint Speed (deg/s)': b.speed(undefined, v[0]); break;
      case 'Set Rounding': b.zone(v[0]); break;
      case 'Move Joints': b.move('MoveJ', null, v.slice(6).filter(Number.isFinite)); break;
      case 'Move Linear': b.move('MoveL', pose(), v.slice(6).filter(Number.isFinite)); break;
      case 'Move Circular (1)': via = { pose: pose(), joints: v.slice(6).filter(Number.isFinite) }; break;
      case 'Move Circular (2)': if (via) b.moveC(via.pose, via.joints, pose(), v.slice(6).filter(Number.isFinite)); via = null; break;
      case 'Set DO': b.prog.setDO(c[1].replace(/^OUT\[(\d+)\]$/, '$1'), c[2] === '1'); break;
      case 'Wait DI': b.prog.waitDI(c[1].replace(/^IN\[(\d+)\]$/, '$1'), c[2] === '1', c[3] ? num(c[3]) : -1); break;
      case 'Pause (ms)': b.prog.pause(v[0]); break;
      case 'Stop': b.prog.pause(-1); break;
      case 'Show Message': b.prog.showMessage(c.slice(1).join(',')); break;
      case 'Run Program': b.prog.runInstruction(c.slice(1).join(','), true); break;
      default: b.prog.runInstruction(raw.trim(), false);
    }
  }
}

export { Frame };
void invert;
