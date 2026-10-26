/**
 * RoboDK-style post processor framework. Built-in vendor posts subclass RoboPost and implement the same callbacks as
 * RoboDK's Python `RobotPost` (ProgStart, MoveJ, MoveL, MoveC, setFrame, setTool, setSpeed, setZoneData, setDO, waitDI,
 * RunCode, RunMessage, ProgFinish, ProgSave). `runRoboPost` replays a compiled PostProgram exactly the way
 * python/post_shim.py replays it through RoboDK's own posts, so both outputs can be compared line by line
 * (tests/posts_gt.test.ts). Control-flow events (loop / if / thread / wait-signal) go through `ctl*` hooks that
 * vendor posts override with native syntax; the default writes a comment and a warning (I6).
 */
import { Mat4, poseToQuat, quatToPose } from '../core/math/pose';
import { PostEvent, PostFile, PostProgram } from './base';
import { pose2Kuka, posePP } from './euler';
export { nameIndex } from './base';

// ---- Python-compatible number formatting ---------------------------------------------------------------------------
/** printf '%.Nf' (tiny negatives keep their sign like CPython: -0.0001 -> '-0.000'; an exact -0 prints '0.000'). */
export function fx(v: number, d = 3): string {
  if (!Number.isFinite(v)) return String(v);
  return (v === 0 ? 0 : v).toFixed(d);
}
/** Python str(float) / repr. */
export function pyStr(v: number): string {
  if (Number.isInteger(v)) return typeof v === 'number' && Math.abs(v) < 1e16 ? (Object.is(v, -0) ? '-0.0' : v.toFixed(1)) : String(v);
  const a = Math.abs(v);
  if (a !== 0 && (a < 1e-4 || a >= 1e16)) { const e = v.toExponential(); return e.replace(/e([+-])(\d)$/, 'e$10$2'); }
  return String(v);
}
/** Python '%g' (precision 6 by default). */
export function pyG(v: number, prec = 6): string {
  if (v === 0) return Object.is(v, -0) ? '-0' : '0';
  const exp = Math.floor(Math.log10(Math.abs(v)));
  let s: string;
  if (exp < -4 || exp >= prec) {
    s = v.toExponential(prec - 1).replace(/\.?0+e/, 'e').replace(/e([+-])(\d)$/, 'e$10$2');
  } else {
    s = v.toFixed(Math.max(0, prec - 1 - exp));
    if (s.includes('.')) s = s.replace(/\.?0+$/, '');
  }
  return s;
}
/** Python '%i' / '%d' of a float (truncates toward zero). */
export const pyI = (v: number) => String(Math.trunc(v));
/** Python round() (banker's rounding on .5). */
export function pyRound(v: number, d = 0): number {
  const m = 10 ** d, x = v * m, r = Math.round(x);
  return (Math.abs(x % 1) === 0.5 ? (r % 2 === 0 ? r : r - 1) : r) / m;
}

/**
 * Linear approximation of a circular move for controllers without a circle instruction: the circle through the start,
 * via and end positions is sampled every <= maxStepDeg (orientation slerped from start to end, like RoboDK's MoveC).
 * Returns the intermediate poses plus the end pose and the radius; null when the points are (nearly) collinear.
 */
export function arcPoses(p0: Mat4, p1: Mat4, p2: Mat4, maxStepDeg = 10): { poses: Mat4[]; radius: number } | null {
  const P = (m: Mat4) => [m[12], m[13], m[14]];
  const a = P(p0), b = P(p1), c = P(p2);
  const ab = b.map((v, i) => v - a[i]), ac = c.map((v, i) => v - a[i]);
  const cr = (u: number[], v: number[]) => [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]];
  const dt = (u: number[], v: number[]) => u[0] * v[0] + u[1] * v[1] + u[2] * v[2];
  const n = cr(ab, ac), n2 = dt(n, n);
  if (n2 < 1e-9) return null;
  // circumcentre of the three positions
  const t1 = cr(n, ab).map((v) => v * dt(ac, ac)), t2 = cr(ac, n).map((v) => v * dt(ab, ab));
  const ctr = a.map((v, i) => v + (t1[i] + t2[i]) / (2 * n2));
  const radius = Math.hypot(a[0] - ctr[0], a[1] - ctr[1], a[2] - ctr[2]);
  const nn = n.map((v) => v / Math.sqrt(n2));
  const u = a.map((v, i) => (v - ctr[i]) / radius), w = cr(nn, u);
  const ang = (q: number[]) => { const d = q.map((v, i) => v - ctr[i]); let t = Math.atan2(dt(d, w), dt(d, u)); if (t < 0) t += 2 * Math.PI; return t; };
  const total = ang(c);
  const steps = Math.max(2, Math.ceil((total * 180) / Math.PI / maxStepDeg));
  const qa = poseToQuat(p0);
  let qb = poseToQuat(p2);
  if (qa[0] * qb[0] + qa[1] * qb[1] + qa[2] * qb[2] + qa[3] * qb[3] < 0) qb = qb.map((v) => -v) as typeof qb;
  const slerp = (t: number): typeof qa => {
    const d = Math.min(1, Math.abs(qa[0] * qb[0] + qa[1] * qb[1] + qa[2] * qb[2] + qa[3] * qb[3])), th = Math.acos(d);
    if (th < 1e-9) return qa;
    const s0 = Math.sin((1 - t) * th) / Math.sin(th), s1 = Math.sin(t * th) / Math.sin(th);
    return qa.map((v, i) => v * s0 + qb[i] * s1) as typeof qa;
  };
  const poses: Mat4[] = [];
  for (let k = 1; k < steps; k++) {
    const t = k / steps, th = total * t;
    const pt = ctr.map((v, i) => v + radius * (Math.cos(th) * u[i] + Math.sin(th) * w[i]));
    poses.push(quatToPose(pt[0], pt[1], pt[2], slerp(t)));
  }
  poses.push(p2);
  return { poses, radius };
}

export type IoName = string | number;
export interface PostOptions { [k: string]: any }
export interface RoboPostCtx {
  robotName: string;
  dof: number;
  axesType: string[];
  options: PostOptions;
  /** generation date (header comments, Fanuc/Motoman dates) */
  date: Date;
  /** Names of the programs generated in this run (main first). */
  programs: string[];
  /**
   * Programs called by a program of this run and by the sub-programs generated with it, in generation order (its own
   * calls first). RoboDK feeds a program and its sub-programs to one post instance, so a post that declares calls (KUKA
   * KRC2 EXT) lists them all.
   */
  callsOf?(program: string): string[];
  warn(msg: string): void;
}

/** Base class mirroring RoboDK's RobotPost. Subclasses override what they need. */
export abstract class RoboPost {
  PROG: string[] = [];
  LOG: string[] = [];
  PROG_EXT = 'txt';
  PROG_NAME = 'Program';
  ROBOT_NAME: string;
  nAxes: number;
  AXES_TYPE: string[];
  /** Extra files produced by ProgSave besides the main program. */
  protected extraFiles: PostFile[] = [];

  constructor(protected ctx: RoboPostCtx) {
    this.ROBOT_NAME = ctx.robotName;
    this.nAxes = ctx.dof;
    this.AXES_TYPE = ctx.axesType;
  }
  get options(): PostOptions { return this.ctx.options; }

  addline(s: string): void { this.PROG.push(s); }
  addlog(s: string): void { this.LOG.push(s); this.ctx.warn(s); }

  ProgStart(name: string): void { this.PROG_NAME = name; }
  ProgFinish(_name: string): void { /* override */ }
  /** Return the generated files (RoboDK writes them into a folder). */
  ProgSave(name: string): PostFile[] { return [{ name: `${name}.${this.PROG_EXT}`, content: this.PROG.join('\n') + '\n' }, ...this.extraFiles]; }

  abstract MoveJ(pose: Mat4 | null, joints: number[], conf: number[]): void;
  abstract MoveL(pose: Mat4 | null, joints: number[], conf: number[]): void;
  MoveC(pose1: Mat4 | null, joints1: number[], pose2: Mat4 | null, joints2: number[], conf1: number[], conf2: number[]): void {
    this.addlog('Circular moves not supported: converted to 2 linear moves');
    this.MoveL(pose1, joints1, conf1);
    this.MoveL(pose2, joints2, conf2);
  }
  setFrame(_pose: Mat4, _frameId: number, _name: string): void { /* override */ }
  setTool(_pose: Mat4, _toolId: number, _name: string): void { /* override */ }
  Pause(_ms: number): void { /* override */ }
  setSpeed(_mms: number): void { /* override */ }
  setAcceleration(_mmss: number): void { /* override */ }
  setSpeedJoints(_degs: number): void { /* override */ }
  setAccelerationJoints(_degss: number): void { /* override */ }
  setZoneData(_mm: number): void { /* override */ }
  setDO(_io: IoName, _value: number | boolean | string): void { /* override */ }
  setAO(io: IoName, value: number | string): void { this.setDO(io, value); }
  waitDI(_io: IoName, _value: number | boolean | string, _timeoutMs = -1): void { /* override */ }
  RunCode(_code: string, _isCall = false): void { /* override */ }
  RunMessage(_msg: string, _isComment = false): void { /* override */ }

  // ---- studio control-flow extensions (I6). Default: comment + warning. ----
  /** FOR loop calling a sub-program `count` times (count <= 0: endless loop). */
  ctlLoop(count: number, body: string): void {
    this.addlog(`Loops are not supported by this post: ${body} is called once`);
    this.RunMessage(`loop ${count > 0 ? count + 'x' : 'forever'}: ${body}`, true);
    this.RunCode(body, true);
  }
  /** IF on a signal/expression calling sub-programs. */
  ctlIf(cond: CtlCondition, thenProg: string | null, elseProg: string | null): void {
    this.addlog(`IF is not supported by this post (${cond.text}): branches written as comments`);
    this.RunMessage(`if ${cond.text}: ${thenProg ?? '-'} else ${elseProg ?? '-'}`, true);
  }
  /** Start a program in parallel (RoboDK "Start thread"). */
  ctlThread(name: string): void {
    this.addlog(`Threads are not supported by this post: ${name} is called sequentially`);
    this.RunCode(name, true);
  }
}

/** Condition of an IF: an IO/signal comparison when parsable, otherwise raw expression text. */
export interface CtlCondition { text: string; io?: IoName; value?: number | boolean | string; op?: '==' | '!=' }

/** Parse "DI_5 == 1", "partReady", "!busy", "IN[3]==ON" into a CtlCondition. */
export function parseCondition(text: string): CtlCondition {
  const t = text.trim();
  let m = t.match(/^(!|not\s+)?\s*([A-Za-z_][\w.\[\]]*|\d+)\s*$/i);
  if (m) return { text: t, io: ioName(m[2]), value: m[1] ? 0 : 1, op: '==' };
  m = t.match(/^([A-Za-z_][\w.\[\]]*|\d+)\s*(==|=|!=|<>)\s*(true|false|on|off|high|low|-?\d+(?:\.\d+)?)$/i);
  if (m) {
    const v = m[3].toLowerCase();
    const value = v === 'true' || v === 'on' || v === 'high' ? 1 : v === 'false' || v === 'off' || v === 'low' ? 0 : Number(v);
    return { text: t, io: ioName(m[1]), value, op: m[2] === '!=' || m[2] === '<>' ? '!=' : '==' };
  }
  return { text: t };
}

/** RoboDK passes numeric IO names as numbers, names as strings. */
export function ioName(io: string | number): IoName {
  if (typeof io === 'number') return io;
  const s = String(io).trim();
  return /^-?\d+$/.test(s) ? Number(s) : s;
}
const ioValue = (v: number | boolean | string): number | string => (typeof v === 'boolean' ? (v ? 1 : 0) : v);


/** Date in RoboDK's header comment format (dd/mm/yyyy hh:mm:ss). */
export function rdkDate(d: Date): string {
  const p = (n: number) => String(n).padStart(2, '0');
  return `${p(d.getDate())}/${p(d.getMonth() + 1)}/${d.getFullYear()} ${p(d.getHours())}:${p(d.getMinutes())}:${p(d.getSeconds())}`;
}

/**
 * Program name as RoboDK hands it to the posts: blanks and non-ASCII characters removed ("Pick EarCup 1" -> PickEarCup1,
 * 05.b "Sıde0" -> Sde0 in RoboDK's intermediate files).
 */
export const postProgName = (n: string) => n.replace(/\s+/g, '').replace(/[^\x21-\x7e]/g, '');
export function renameEvents(events: PostEvent[]): PostEvent[] {
  return events.map((e) => {
    switch (e.kind) {
      case 'callProgram': return { ...e, name: postProgName(e.name) };
      case 'thread': return { ...e, name: postProgName(e.name) };
      case 'loop': return { ...e, body: postProgName(e.body) };
      case 'if': return { ...e, then: e.then && postProgName(e.then), else: e.else && postProgName(e.else) };
      default: return e;
    }
  });
}
/** Main program followed by every (nested) sub-program once, with post-safe names. */
export function flattenPrograms(p: PostProgram): PostProgram[] {
  const out: PostProgram[] = [p];
  const seen = new Set([p.name]);
  const walk = (q: PostProgram) => { for (const s of q.subprograms) { if (seen.has(s.name)) continue; seen.add(s.name); out.push(s); walk(s); } };
  walk(p);
  return out.map((q) => (postProgName(q.name) !== q.name || q.events.some((e) => (e.kind === 'callProgram' || e.kind === 'thread' || e.kind === 'loop' || e.kind === 'if')) ? { ...q, name: postProgName(q.name), events: renameEvents(q.events) } : q));
}

/** Feed one program's events into a post instance (same mapping as python/post_shim.py `_replay`). */
export function replayEvents(post: RoboPost, events: PostEvent[]): void {
  // per-move speed / rounding overrides (events not produced by compileForPost, which already made them explicit):
  // applied before the move and restored before the next move without override
  let progSpeed: number | undefined, curSpeed: number | undefined, progRound: number | undefined, curRound: number | undefined;
  const overrides = (ev: { speed?: number; rounding?: number }, linear: boolean) => {
    const sp = linear ? ev.speed ?? progSpeed : undefined;
    if (sp !== undefined && sp !== curSpeed) { post.setSpeed(sp); curSpeed = sp; }
    const rd = ev.rounding ?? progRound;
    if (rd !== undefined && rd !== curRound) { post.setZoneData(rd); curRound = rd; }
  };
  for (const ev of events) {
    switch (ev.kind) {
      // a joint target reaches the post with pose None, like in RoboDK
      case 'moveJ': overrides(ev, false); post.MoveJ(ev.jointTarget ? null : ev.pose, ev.joints, ev.conf ?? [0, 0, 0]); break;
      case 'moveL': overrides(ev, true); post.MoveL(ev.jointTarget ? null : ev.pose, ev.joints ?? [], ev.conf ?? [0, 0, 0]); break;
      case 'moveC': overrides(ev, true); post.MoveC(ev.via, ev.viaJoints ?? ev.joints ?? [], ev.pose, ev.joints ?? [], ev.viaConf ?? ev.conf ?? [0, 0, 0], ev.conf ?? [0, 0, 0]); break;
      case 'setFrame': post.setFrame(ev.pose, ev.frameId ?? -1, ev.name); break;
      case 'setTool': post.setTool(ev.pose, ev.toolNum ?? -1, ev.name); break;
      case 'setSpeed':
        if (ev.speedLinear !== undefined) { post.setSpeed(ev.speedLinear); progSpeed = curSpeed = ev.speedLinear; }
        if (ev.speedJoints !== undefined) post.setSpeedJoints(ev.speedJoints);
        if (ev.accelLinear !== undefined) post.setAcceleration(ev.accelLinear);
        if (ev.accelJoints !== undefined) post.setAccelerationJoints(ev.accelJoints);
        break;
      case 'setRounding': post.setZoneData(ev.radius); progRound = curRound = ev.radius; break;
      case 'pause': post.Pause(ev.timeMs); break;
      case 'setDO': post.setDO(ioName(ev.io), ioValue(ev.value)); break;
      case 'setAO': post.setAO(ioName(ev.io), ev.value); break;
      case 'waitDI': post.waitDI(ioName(ev.io), ioValue(ev.value), ev.timeoutMs ?? -1); break;
      case 'runCode': post.RunCode(ev.code, ev.isCall); break;
      case 'comment': post.RunMessage(ev.text, true); break;
      case 'message': post.RunMessage(ev.text, false); break;
      case 'callProgram': post.RunCode(ev.name, true); break;
      case 'gripper': post.RunMessage(ev.text ?? `gripper ${ev.close ? 'close' : 'open'}`, true); break;
      case 'navigate': post.RunMessage(`NAVIGATE X ${fx(ev.x)} Y ${fx(ev.y)}`, true); break;
      case 'task': post.RunMessage(`TASK ${ev.task}`, true); break;
      case 'loop': post.ctlLoop(ev.count, ev.body); break;
      case 'if': post.ctlIf(parseCondition(ev.condition), ev.then, ev.else); break;
      case 'thread': post.ctlThread(ev.name); break;
    }
  }
}

/**
 * The numbers RoboDK's posts receive. RoboDK runs a post through an intermediate script that writes every pose as
 * p(x,y,z,a,b,c) (robomath.PosePP: KUKA xyzabc) and every joint vector with '%g' (6 significant digits): the post's
 * output is computed from those values (RoboDK 5.9.4 Post<program>.py, src/posts/rdk_job.ts). Replaying the studio's
 * full-precision events the same way makes the last digits, Motoman pulses and the sign of angles at +-180 deg
 * identical; python/post_shim.py applies the same rounding to RoboDK's own posts.
 */
export function rdkPrecisionEvents(events: PostEvent[]): PostEvent[] {
  const g = (v: number) => Number(pyG(v, 6));
  // RoboDK writes no pose value below 1e-6 (12330 values of the Library intermediate files: 1137 exact zeros where the
  // studio's decomposition leaves rounding noise such as -1e-15, never a sign flip at +-180): noise is written as 0
  const gp = (v: number) => (Math.abs(v) < 1e-9 ? 0 : g(v));
  const P = <T extends Mat4 | null | undefined>(m: T): T => (m ? posePP(pose2Kuka(m).map(gp)) as T : m);
  const J = (j: number[] | null | undefined) => (j ? j.map(g) : j);
  return events.map((e) => {
    switch (e.kind) {
      case 'moveJ': return { ...e, pose: P(e.pose), joints: J(e.joints) as number[] };
      case 'moveL': return { ...e, pose: P(e.pose), joints: J(e.joints) ?? null };
      case 'moveC': return { ...e, pose: P(e.pose), via: P(e.via), joints: J(e.joints) ?? null, viaJoints: J(e.viaJoints) ?? null };
      case 'setFrame': return { ...e, pose: P(e.pose) };
      case 'setTool': return { ...e, pose: P(e.pose) };
      default: return e;
    }
  });
}

export interface RunOptions extends PostOptions {
  /** Header comments right after ProgStart (RoboDK: "Program generated by RoboDK ..."); false disables them. */
  headerComments?: string[] | false;
  date?: Date;
}

/**
 * Run a RoboPost class over a compiled program and its sub-programs. Like RoboDK's default (INCLUDE_SUB_PROGRAMS
 * False), every program gets its own post instance and files unless the post sets `includeSubPrograms`, in which case
 * one instance receives all programs (main first) and ProgSave is called once.
 */
export function runRoboPost(make: (ctx: RoboPostCtx) => RoboPost, p: PostProgram, options: RunOptions = {}, includeSubPrograms = false): PostFile[] {
  // options.rdkPrecision false: full precision (default: the numbers RoboDK's own posts receive)
  const progs = flattenPrograms(p).map((q) => (options.rdkPrecision === false ? q : { ...q, events: rdkPrecisionEvents(q.events) }));
  const date = options.date ?? new Date();
  const warnings: string[] = [];
  const ctx: RoboPostCtx = {
    robotName: p.robotName || 'Robot', dof: p.dof, axesType: p.axesType ?? Array(p.dof).fill('R'), options, date,
    programs: progs.map((q) => q.name), warn: (m) => warnings.push(m),
    callsOf: (name) => {
      const byName = new Map(progs.map((q) => [q.name, q]));
      const out: string[] = [];
      const seen = new Set<string>();
      const own = (q: PostProgram) => q.events.flatMap((e) => (e.kind === 'callProgram' ? [e.name] : e.kind === 'runCode' && e.isCall ? [e.code] : []));
      // the program, then its sub-programs in generation order (depth first, each once)
      const order: PostProgram[] = [];
      const walk = (q: PostProgram | undefined) => { if (!q || seen.has(q.name)) return; seen.add(q.name); order.push(q); for (const c of own(q)) walk(byName.get(c)); };
      walk(byName.get(name));
      for (const q of order) for (const c of own(q)) if (!out.includes(c)) out.push(c);
      return out;
    },
  };
  const header = options.headerComments === false ? [] : options.headerComments ?? [`Program generated by VerticalBot Studio for ${ctx.robotName} on ${rdkDate(date)}`, 'Using nominal kinematics.'];
  const play = (post: RoboPost, q: PostProgram) => {
    post.ProgStart(q.name);
    // like RoboDK: the header goes to the program being generated, not to the sub-programs generated with it
    if (q === progs[0]) for (const c of header) post.RunMessage(c, true);
    replayEvents(post, q.events);
    post.ProgFinish(q.name);
  };
  const files: PostFile[] = [];
  if (includeSubPrograms) {
    const post = make(ctx);
    for (const q of progs) play(post, q);
    files.push(...post.ProgSave(progs[0].name));
  } else {
    for (const q of progs) {
      const post = make(ctx);
      play(post, q);
      files.push(...post.ProgSave(q.name));
    }
  }
  // same file name twice (vendor name truncation, e.g. Motoman DropPart1/DropPart2 -> DROPPAR.JBI): like RoboDK the last
  // file written wins; the clash is reported because calls then reach the last program only
  const byName = new Map<string, PostFile>();
  for (const f of files) {
    const prev = byName.get(f.name);
    if (prev && prev.content !== f.content) p.problems?.push(`Post: two programs are written to the same file ${f.name} (name truncated by the controller limits); rename them`);
    byName.set(f.name, f);
  }
  for (const w of warnings) if (!p.problems?.includes(w)) p.problems?.push(w);
  return [...byName.values()];
}
