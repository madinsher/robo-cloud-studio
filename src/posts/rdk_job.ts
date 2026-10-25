/**
 * RoboDK's intermediate post script ("RoboDK Python Intermediate file to generate robot programs", written to %TEMP%
 * as Post<program>.py by MakeProgram and run with the robot's Python post): the exact RobotPost calls RoboDK makes,
 * with the pose (PosePP = KUKA xyzabc), joints and configuration it chose for every move. Parsed here so that
 *  - the studio's compiled program can be compared call by call with RoboDK's (tests/posts_library_code.test.ts);
 *  - RoboDK's own calls can be replayed through the studio's posts (`rdkJobToPostProgram`).
 * The fixtures come from tests/fixtures/posts_gt/dump_library_loaded.py.
 */
import { Mat4 } from '../core/math/pose';
import { PostEvent, PostProgram } from './base';
import { RoboPost, RoboPostCtx, replayEvents, flattenPrograms, IoName } from './robopost';
import { pose2Kuka, posePP } from './euler';
export { posePP } from './euler';

export type RdkArg = number | string | boolean | null | number[] | { pose: number[] };
export interface RdkCall { fn: string; args: RdkArg[] }
export interface RdkJobProgram { name: string; calls: RdkCall[] }
export interface RdkJobScript {
  post: string;
  robot: string;
  dof: number;
  axesType: string[];
  programs: RdkJobProgram[];
}

/** Tokenise the argument list of one `r.Fn(...)` call (Python literals RoboDK writes). */
function parseArgs(src: string): RdkArg[] {
  let i = 0;
  const ws = () => { while (i < src.length && /\s/.test(src[i])) i++; };
  const num = (): number => {
    const m = /^[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?/.exec(src.slice(i));
    if (!m) throw new Error(`number expected at ${src.slice(i, i + 20)}`);
    i += m[0].length;
    return Number(m[0]);
  };
  const value = (): RdkArg => {
    ws();
    if (src.startsWith('r"""', i) || src.startsWith('"""', i)) {
      i += src[i] === 'r' ? 4 : 3;
      const e = src.indexOf('"""', i);
      const s = src.slice(i, e);
      i = e + 3;
      return s;
    }
    if (src[i] === 'r' && (src[i + 1] === '"' || src[i + 1] === "'")) i++;
    if (src[i] === '"' || src[i] === "'") {
      const q = src[i++];
      let s = '';
      while (i < src.length && src[i] !== q) { if (src[i] === '\\') { s += src[i + 1]; i += 2; } else s += src[i++]; }
      i++;
      return s;
    }
    if (src.startsWith('None', i)) { i += 4; return null; }
    if (src.startsWith('True', i)) { i += 4; return true; }
    if (src.startsWith('False', i)) { i += 5; return false; }
    if (src[i] === '[') {
      i++;
      const out: number[] = [];
      ws();
      while (src[i] !== ']') { out.push(num()); ws(); if (src[i] === ',') i++; ws(); }
      i++;
      return out;
    }
    if (src.startsWith('p(', i)) {
      i += 2;
      const out: number[] = [];
      ws();
      while (src[i] !== ')') { out.push(num()); ws(); if (src[i] === ',') i++; ws(); }
      i++;
      return { pose: out };
    }
    return num();
  };
  const out: RdkArg[] = [];
  ws();
  while (i < src.length) {
    out.push(value());
    ws();
    if (src[i] === ',') i++;
    ws();
  }
  return out;
}

/** Split `name(args)` keeping nested brackets / strings intact. */
function callOf(line: string): { fn: string; args: string } | null {
  const m = /^r\.(\w+)\((.*)\)\s*$/s.exec(line.trim());
  return m ? { fn: m[1], args: m[2] } : null;
}

export function parseRdkJobScript(text: string): RdkJobScript {
  const job: RdkJobScript = { post: '', robot: '', dof: 0, axesType: [], programs: [] };
  // multi-line strings (r"""...""" spanning lines) are joined back into one logical line
  const lines: string[] = [];
  let acc = '';
  for (const raw of text.replace(/\r\n?/g, '\n').split('\n')) {
    acc = acc ? `${acc}\n${raw}` : raw;
    if (((acc.match(/"""/g) ?? []).length % 2) === 0) { lines.push(acc); acc = ''; }
  }
  if (acc) lines.push(acc);
  let cur: RdkJobProgram | null = null;
  for (const l of lines) {
    const rp = /^r = RobotPost\((.*)\)\s*$/s.exec(l.trim());
    if (rp) {
      const a = /^\s*r?"""([\s\S]*?)"""\s*,\s*r?"""([\s\S]*?)"""\s*,\s*(\d+)/.exec(rp[1]);
      if (a) { job.post = a[1]; job.robot = a[2]; job.dof = Number(a[3]); }
      const at = /axes_type=\[([^\]]*)\]/.exec(rp[1]);
      job.axesType = at ? at[1].split(',').map((s) => s.trim().replace(/['"]/g, '')).filter(Boolean) : [];
      continue;
    }
    const c = callOf(l);
    if (!c) continue;
    const args = parseArgs(c.args);
    if (c.fn === 'ProgStart') { cur = { name: String(args[0]), calls: [] }; job.programs.push(cur); continue; }
    if (c.fn === 'ProgFinish' || c.fn === 'ProgSave' || c.fn === 'ProgSendRobot' || c.fn === 'Finish') continue;
    if (cur) cur.calls.push({ fn: c.fn, args });
  }
  return job;
}

// ---- RoboDK PosePP <-> studio Mat4 (euler.ts posePP) -----------------------------------------------------------
const poseArg = (a: RdkArg): Mat4 | null => (a && typeof a === 'object' && !Array.isArray(a) ? posePP(a.pose) : null);
const listArg = (a: RdkArg): number[] => (Array.isArray(a) ? a : []);

/** RoboDK's calls as studio PostEvents (header comments included as RoboDK sends them). */
export function rdkCallsToEvents(calls: RdkCall[]): PostEvent[] {
  const ev: PostEvent[] = [];
  for (const { fn, args } of calls) {
    switch (fn) {
      case 'MoveJ': { const pose = poseArg(args[0]); ev.push({ kind: 'moveJ', joints: listArg(args[1]), pose, name: '', conf: listArg(args[2]).length ? listArg(args[2]) : [0, 0, 0], jointTarget: !pose }); break; }
      case 'MoveL': { const pose = poseArg(args[0]); ev.push({ kind: 'moveL', joints: listArg(args[1]), pose: pose ?? (new Float64Array(16) as unknown as Mat4), name: '', conf: listArg(args[2]).length ? listArg(args[2]) : [0, 0, 0], jointTarget: !pose }); break; }
      case 'MoveC': ev.push({ kind: 'moveC', via: poseArg(args[0])!, viaJoints: listArg(args[1]), pose: poseArg(args[2])!, joints: listArg(args[3]), viaConf: listArg(args[4]), conf: listArg(args[5]), name: '', viaName: '' }); break;
      case 'setFrame': ev.push({ kind: 'setFrame', pose: poseArg(args[0])!, frameId: Number(args[1]), name: String(args[2] ?? '') }); break;
      case 'setTool': ev.push({ kind: 'setTool', pose: poseArg(args[0])!, toolNum: Number(args[1]), name: String(args[2] ?? ''), toolId: null }); break;
      case 'setSpeed': ev.push({ kind: 'setSpeed', speedLinear: Number(args[0]) }); break;
      case 'setSpeedJoints': ev.push({ kind: 'setSpeed', speedJoints: Number(args[0]) }); break;
      case 'setAcceleration': ev.push({ kind: 'setSpeed', accelLinear: Number(args[0]) }); break;
      case 'setAccelerationJoints': ev.push({ kind: 'setSpeed', accelJoints: Number(args[0]) }); break;
      case 'setZoneData': ev.push({ kind: 'setRounding', radius: Number(args[0]) }); break;
      case 'Pause': ev.push({ kind: 'pause', timeMs: Number(args[0]) }); break;
      case 'setDO': ev.push({ kind: 'setDO', io: String(args[0]), value: args[1] as number }); break;
      case 'setAO': ev.push({ kind: 'setAO', io: String(args[0]), value: args[1] as number }); break;
      case 'waitDI': ev.push({ kind: 'waitDI', io: String(args[0]), value: args[1] as number, timeoutMs: args.length > 2 ? Number(args[2]) : -1 }); break;
      case 'RunCode': ev.push({ kind: 'runCode', code: String(args[0]), isCall: args[1] === true }); break;
      case 'RunMessage': ev.push(args[1] === true ? { kind: 'comment', text: String(args[0]) } : { kind: 'message', text: String(args[0]) }); break;
      default: ev.push({ kind: 'comment', text: `unknown RoboDK call ${fn}` });
    }
  }
  return ev;
}

/** RoboDK's job as a PostProgram (main = first ProgStart, the others as sub-programs in RoboDK's order). */
export function rdkJobToPostProgram(job: RdkJobScript, robot: PostProgram['robot'] = null): PostProgram {
  const mk = (p: RdkJobProgram): PostProgram => ({
    name: p.name, robot, robotName: job.robot, dof: job.dof, axesType: job.axesType.length ? job.axesType : undefined, jointNames: [],
    events: rdkCallsToEvents(p.calls), subprograms: [], frame: { pose: posePP([0, 0, 0, 0, 0, 0]), name: 'World' }, tool: { pose: posePP([0, 0, 0, 0, 0, 0]), name: 'Flange' }, problems: [],
  });
  const [main, ...subs] = job.programs.map(mk);
  if (!main) throw new Error('RoboDK job without ProgStart');
  main.subprograms = subs;
  return main;
}

// ---- the studio side: the same calls, recorded from a compiled program ----------------------------------------------
class RecordingPost extends RoboPost {
  readonly programs: RdkJobProgram[] = [];
  private cur: RdkJobProgram | null = null;
  private rec(fn: string, args: RdkArg[]) { this.cur?.calls.push({ fn, args }); }
  private static pose(m: Mat4 | null): RdkArg { return m ? { pose: pose2Kuka(m).map((v) => v) } : null; }
  override ProgStart(name: string): void { this.cur = { name, calls: [] }; this.programs.push(this.cur); }
  MoveJ(pose: Mat4 | null, joints: number[], conf: number[]): void { this.rec('MoveJ', [RecordingPost.pose(pose), joints.slice(), pose ? conf.slice() : null]); }
  MoveL(pose: Mat4 | null, joints: number[], conf: number[]): void { this.rec('MoveL', [RecordingPost.pose(pose), joints.slice(), pose ? conf.slice() : null]); }
  override MoveC(p1: Mat4 | null, j1: number[], p2: Mat4 | null, j2: number[], c1: number[], c2: number[]): void { this.rec('MoveC', [RecordingPost.pose(p1), j1.slice(), RecordingPost.pose(p2), j2.slice(), c1.slice(), c2.slice()]); }
  override setFrame(pose: Mat4, id: number, name: string): void { this.rec('setFrame', [RecordingPost.pose(pose), id, name]); }
  override setTool(pose: Mat4, id: number, name: string): void { this.rec('setTool', [RecordingPost.pose(pose), id, name]); }
  override Pause(ms: number): void { this.rec('Pause', [ms]); }
  override setSpeed(v: number): void { this.rec('setSpeed', [v]); }
  override setAcceleration(v: number): void { this.rec('setAcceleration', [v]); }
  override setSpeedJoints(v: number): void { this.rec('setSpeedJoints', [v]); }
  override setAccelerationJoints(v: number): void { this.rec('setAccelerationJoints', [v]); }
  override setZoneData(v: number): void { this.rec('setZoneData', [v]); }
  override setDO(io: IoName, v: number | boolean | string): void { this.rec('setDO', [String(io), v as number]); }
  override setAO(io: IoName, v: number | string): void { this.rec('setAO', [String(io), v as number]); }
  override waitDI(io: IoName, v: number | boolean | string, t = -1): void { this.rec('waitDI', [String(io), v as number, t]); }
  override RunCode(code: string, isCall = false): void { this.rec('RunCode', [code, isCall]); }
  override RunMessage(msg: string, isComment = false): void { this.rec('RunMessage', [msg, isComment]); }
}

/** The RobotPost calls the studio makes for a compiled program (main first, then its sub-programs). */
export function recordPostCalls(p: PostProgram): RdkJobProgram[] {
  const ctx: RoboPostCtx = { robotName: p.robotName, dof: p.dof, axesType: p.axesType ?? [], options: {}, date: new Date(0), programs: [], warn: () => {} };
  const rec = new RecordingPost(ctx);
  for (const q of flattenPrograms(p)) {
    rec.ProgStart(q.name);
    replayEvents(rec, q.events);
  }
  return rec.programs;
}

/** Same number (RoboDK writes 6 significant digits) within `tol` absolute, angles modulo 360 for poses. */
function near(a: number, b: number, tol: number, angle = false): boolean {
  let d = Math.abs(a - b);
  if (angle) d = Math.min(d, Math.abs(d - 360));
  return d <= tol + 1e-6 * Math.max(Math.abs(a), Math.abs(b));
}
function argEqual(a: RdkArg, b: RdkArg, tol: number): boolean {
  if (a === null || b === null) return a === b;
  if (typeof a === 'number' && typeof b === 'number') return near(a, b, tol);
  if (typeof a === 'string' || typeof a === 'boolean') return String(a) === String(b);
  if (Array.isArray(a) && Array.isArray(b)) return a.length === b.length && a.every((v, k) => near(v, b[k], tol));
  if (typeof a === 'object' && typeof b === 'object' && !Array.isArray(a) && !Array.isArray(b)) {
    const pa = posePP((a as { pose: number[] }).pose), pb = posePP((b as { pose: number[] }).pose);
    for (let k = 0; k < 16; k++) if (Math.abs(pa[k] - pb[k]) > (k >= 12 ? tol : 1e-4)) return false;
    return true;
  }
  return false;
}
export const fmtArg = (a: RdkArg): string => (a === null ? 'None' : typeof a === 'object' && !Array.isArray(a) ? `p(${a.pose.map((v) => +v.toFixed(3)).join(',')})` : Array.isArray(a) ? `[${a.map((v) => +v.toFixed(3)).join(',')}]` : String(a));
export const fmtCall = (c: RdkCall | undefined) => (c ? `${c.fn}(${c.args.map(fmtArg).join(', ')})` : '<none>');

/**
 * First difference between RoboDK's calls and the studio's (null = equal): programs in order, calls in order, poses
 * within `tol` mm and 1e-4 rotation, numbers within `tol`. `skip` filters calls out of both sides (e.g. header comments).
 */
export function diffRdkCalls(rdk: RdkJobProgram[], studio: RdkJobProgram[], tol = 0.01, skip: (c: RdkCall) => boolean = () => false): string | null {
  // RoboDK passes program names with their blanks (RunCode("Pick 1", True)); its posts remove them, the studio removes
  // them before its posts: compared without blanks. RunCode(code) without the flag means is_function_call False.
  const nb = (s: string) => s.replace(/\s+/g, '');
  const norm = (c: RdkCall): RdkCall => (c.fn === 'RunCode' ? { fn: c.fn, args: [nb(String(c.args[0])), c.args[1] === true] } : c.fn === 'RunMessage' ? { fn: c.fn, args: [String(c.args[0]), c.args[1] === true] } : c);
  for (let k = 0; k < Math.max(rdk.length, studio.length); k++) {
    const a = rdk[k], b = studio[k];
    if (!a || !b) return `program ${k + 1}: RoboDK ${a?.name ?? '<none>'} studio ${b?.name ?? '<none>'}`;
    if (nb(a.name) !== nb(b.name)) return `program ${k + 1}: RoboDK ${a.name} studio ${b.name}`;
    const ca = a.calls.filter((c) => !skip(c)).map(norm), cb = b.calls.filter((c) => !skip(c)).map(norm);
    for (let i = 0; i < Math.max(ca.length, cb.length); i++) {
      const x = ca[i], y = cb[i];
      if (!x || !y || x.fn !== y.fn || x.args.length !== y.args.length || !x.args.every((v, j) => argEqual(v, y.args[j], tol))) {
        return `${a.name} call ${i + 1}: RoboDK ${fmtCall(x)} | studio ${fmtCall(y)}`;
      }
    }
  }
  return null;
}
