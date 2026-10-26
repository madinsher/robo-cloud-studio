/**
 * FSM → IEC 61131-3 Structured Text (src/ctl/codegen/plc_fsm.ts). The generated PROGRAM is executed by a small ST
 * interpreter (the subset the generator emits: declarations, :=, IF / ELSIF / ELSE, CASE, TON, R_TRIG) against a
 * scripted skill server on the START / DONE / RES / FAIL handshake, and must visit the same states in the same order and
 * end with the same outcome as the studio FsmExecutor on the same model and script — here the MURKA retail mission
 * (nested machines, skills with several outcomes, loops, error continuation) and a model with decisions, waits and a
 * state timeout.
 */
import { describe, it, expect } from 'vitest';
import { fsmToST } from '../src/ctl/codegen/plc_fsm';
import { automatonToST, petriToST } from '../src/ctl/codegen/plc';
import { fsmToPlcOpen, flattenFsm } from '../src/ctl/codegen/plc_fsm_sfc';
import { parseXml, XmlNode } from '../src/ctl/codegen/common';
import { controlExports } from '../src/ctl/codegen';
import { parseFsm, FsmExecutor } from '../src/ctl/fsm';
import type { ExecutorWorld } from '../src/ctl/exec';
import type { BTNode, TickContext } from '../src/ctl/bt';
import { missionFsmDsl, parseMission } from '../src/murka/retail/orchestrator';
import { setValue } from '../src/ctl/runtime';

// ------------------------------------------------------------------ a minimal Structured Text interpreter
type Tok = { k: 'id' | 'num' | 'op' | 'time' | 'str'; v: string };
function lex(src: string): Tok[] {
  const s = src.replace(/\(\*[\s\S]*?\*\)/g, ' '); const out: Tok[] = []; let i = 0;
  while (i < s.length) {
    const c = s[i];
    if (/\s/.test(c)) { i++; continue; }
    const t = /^T#(\d+)MS/.exec(s.slice(i)); if (t) { out.push({ k: 'time', v: t[1] }); i += t[0].length; continue; }
    const id = /^[A-Za-z_][A-Za-z0-9_]*/.exec(s.slice(i)); if (id) { out.push({ k: 'id', v: id[0] }); i += id[0].length; continue; }
    const n = /^\d+(\.\d+)?/.exec(s.slice(i)); if (n) { out.push({ k: 'num', v: n[0] }); i += n[0].length; continue; }
    if (c === "'") { const j = s.indexOf("'", i + 1); out.push({ k: 'str', v: s.slice(i + 1, j) }); i = j + 1; continue; }
    const op = /^(:=|<=|>=|<>|%[IQ][XWD][\d.]+|[=<>+\-*/();:,.])/.exec(s.slice(i)); if (op) { out.push({ k: 'op', v: op[0] }); i += op[0].length; continue; }
    throw new Error(`lex error at ${s.slice(i, i + 20)}`);
  }
  return out;
}
type Stmt = { t: 'assign'; name: string; e: Expr } | { t: 'if'; arms: Array<[Expr, Stmt[]]>; els: Stmt[] } | { t: 'case'; e: Expr; arms: Array<[number[], Stmt[]]>; els: Stmt[] } | { t: 'call'; fb: string; args: Record<string, Expr> };
type Expr = (env: Env) => number | boolean | string;
type Env = Record<string, any>;
class StProgram {
  vars: Env = {}; fbs: Record<string, { type: string; [k: string]: any }> = {}; body: Stmt[] = []; private p = 0; private t: Tok[];
  constructor(src: string, exprOnly = false) { this.t = lex(src); if (!exprOnly) this.parse(); }
  /** Parse a single ST expression (SFC transition conditions). */
  static parseExpr(src: string): Expr { const p = new StProgram(src, true); const e = p.expr(); if (p.p !== p.t.length) throw new Error(`trailing tokens in ${src}`); return e; }
  private peek(o = 0) { return this.t[this.p + o]; }
  private eat(v?: string) { const x = this.t[this.p++]; if (v !== undefined && x?.v !== v) throw new Error(`expected ${v}, got ${x?.v} at ${this.p}`); return x; }
  private parse() {
    this.eat('PROGRAM'); this.eat();
    while (/^VAR/.test(this.peek().v)) {
      this.eat(); if (this.peek().v === 'CONSTANT') this.eat();
      while (this.peek().v !== 'END_VAR') {
        const name = this.eat().v; if (this.peek().v === 'AT') { this.eat(); this.eat(); }
        this.eat(':'); const type = this.eat().v; let init: number | boolean = type === 'BOOL' ? false : 0;
        if (this.peek().v === ':=') { this.eat(); init = this.expr()({}) as number | boolean; }
        this.eat(';');
        if (type === 'TON' || type === 'R_TRIG') this.fbs[name] = { type, Q: false, ET: 0, IN: false, PT: 0, M: false }; else this.vars[name] = init;
      }
      this.eat('END_VAR');
    }
    this.body = this.stmts(['END_PROGRAM']);
  }
  private stmts(stop: string[]): Stmt[] {
    const out: Stmt[] = [];
    for (;;) {
      const x = this.peek(); if (!x || stop.includes(x.v)) return out;
      if (x.k === 'num' && this.peek(1)?.v === ':' ) return out; // next CASE label
      if (x.v === 'IF') { this.eat(); const arms: Array<[Expr, Stmt[]]> = []; let c = this.expr(); this.eat('THEN'); arms.push([c, this.stmts(['ELSIF', 'ELSE', 'END_IF'])]); let els: Stmt[] = []; while (this.peek().v === 'ELSIF') { this.eat(); c = this.expr(); this.eat('THEN'); arms.push([c, this.stmts(['ELSIF', 'ELSE', 'END_IF'])]); } if (this.peek().v === 'ELSE') { this.eat(); els = this.stmts(['END_IF']); } this.eat('END_IF'); this.eat(';'); out.push({ t: 'if', arms, els }); continue; }
      if (x.v === 'CASE') { this.eat(); const e = this.expr(); this.eat('OF'); const arms: Array<[number[], Stmt[]]> = []; let els: Stmt[] = []; while (this.peek().v !== 'END_CASE') { if (this.peek().v === 'ELSE') { this.eat(); els = this.stmts(['END_CASE']); continue; } const labels = [Number(this.eat().v)]; while (this.peek().v === ',') { this.eat(); labels.push(Number(this.eat().v)); } this.eat(':'); arms.push([labels, this.stmts(['END_CASE', 'ELSE'])]); } this.eat('END_CASE'); this.eat(';'); out.push({ t: 'case', e, arms, els }); continue; }
      const name = this.eat().v;
      if (this.peek().v === '(') { this.eat('('); const args: Record<string, Expr> = {}; while (this.peek().v !== ')') { const k = this.eat().v; this.eat(':='); args[k] = this.expr(); if (this.peek().v === ',') this.eat(); } this.eat(')'); this.eat(';'); out.push({ t: 'call', fb: name, args }); continue; }
      this.eat(':='); const e = this.expr(); this.eat(';'); out.push({ t: 'assign', name, e });
    }
  }
  expr(): Expr { return this.or(); }
  private or(): Expr { let a = this.and(); while (this.peek()?.v === 'OR') { this.eat(); const l = a, r = this.and(); a = (e) => !!l(e) || !!r(e); } return a; }
  private and(): Expr { let a = this.not(); while (this.peek()?.v === 'AND') { this.eat(); const l = a, r = this.not(); a = (e) => !!l(e) && !!r(e); } return a; }
  private not(): Expr { if (this.peek()?.v === 'NOT') { this.eat(); const x = this.not(); return (e) => !x(e); } return this.cmp(); }
  private cmp(): Expr { const a = this.add(); const o = this.peek()?.v; if (['=', '<>', '<', '>', '<=', '>='].includes(o)) { this.eat(); const b = this.add(); return (e) => { const x = a(e) as number, y = b(e) as number; return o === '=' ? x === y : o === '<>' ? x !== y : o === '<' ? x < y : o === '>' ? x > y : o === '<=' ? x <= y : x >= y; }; } return a; }
  private add(): Expr { let a = this.mul(); while (['+', '-'].includes(this.peek()?.v)) { const o = this.eat().v; const l = a, r = this.mul(); a = (e) => (o === '+' ? (l(e) as number) + (r(e) as number) : (l(e) as number) - (r(e) as number)); } return a; }
  private mul(): Expr { let a = this.un(); while (['*', '/'].includes(this.peek()?.v)) { const o = this.eat().v; const l = a, r = this.un(); a = (e) => (o === '*' ? (l(e) as number) * (r(e) as number) : (l(e) as number) / (r(e) as number)); } return a; }
  private un(): Expr { if (this.peek()?.v === '-') { this.eat(); const x = this.un(); return (e) => -(x(e) as number); } return this.prim(); }
  private prim(): Expr {
    const x = this.eat();
    if (x.v === '(') { const e = this.expr(); this.eat(')'); return e; }
    if (x.k === 'num') { const n = Number(x.v); return () => n; }
    if (x.k === 'time') { const n = Number(x.v) / 1000; return () => n; }
    if (x.k === 'str') { const v = x.v; return () => v; }
    if (x.v === 'TRUE') return () => true;
    if (x.v === 'FALSE') return () => false;
    if (this.peek()?.v === '.') { this.eat(); const f = this.eat().v; const fb = x.v; return (e) => (this.fbs[fb] ? this.fbs[fb][f] : e[`${fb}.${f}`]); }
    const n = x.v; return (e) => { if (!(n in e)) throw new Error(`undeclared ${n}`); return e[n]; };
  }
  scan(dt: number) { this.exec(this.body, dt); }
  private exec(ss: Stmt[], dt: number) {
    for (const s of ss) {
      if (s.t === 'assign') { if (!(s.name in this.vars)) throw new Error(`assign to undeclared ${s.name}`); this.vars[s.name] = s.e(this.vars); }
      else if (s.t === 'if') { const arm = s.arms.find(([c]) => !!c(this.vars)); this.exec(arm ? arm[1] : s.els, dt); }
      else if (s.t === 'case') { const v = s.e(this.vars) as number; const arm = s.arms.find(([ls]) => ls.includes(v)); this.exec(arm ? arm[1] : s.els, dt); }
      else {
        const fb = this.fbs[s.fb]; if (!fb) throw new Error(`unknown FB ${s.fb}`);
        if (fb.type === 'R_TRIG') { const clk = !!s.args.CLK(this.vars); fb.Q = clk && !fb.M; fb.M = clk; }
        else { const IN = !!s.args.IN(this.vars); if (s.args.PT) fb.PT = s.args.PT(this.vars) as number; if (!IN) { fb.ET = 0; fb.Q = false; } else { if (fb.IN) fb.ET += dt; fb.Q = fb.ET >= fb.PT - 1e-9; } fb.IN = IN; }
      }
    }
  }
}

// ------------------------------------------------------------------ scripted skill server for both executors
type Script = Record<string, string[]>; // state name -> outcomes of its successive activations (cycled at the end)
function outcomeOf(script: Script, counts: Map<string, number>, state: string, named: string[] | null): string {
  const list = script[state] ?? [named ? named[0] : 'ok']; const n = counts.get(state) ?? 0; counts.set(state, n + 1);
  return list[Math.min(n, list.length - 1)];
}

function runExecutor(dsl: string, script: Script, durations: Record<string, number> = {}): { entered: string[]; outcome: string | null } {
  const doc = parseFsm(dsl); const counts = new Map<string, number>(); const started = new Map<string, number>();
  const skill = (ctx: TickContext, _a: Record<string, unknown>, node: BTNode) => {
    const n = node as BTNode & { outcome?: string }; const st = String(ctx.bb.fsm_state).split('/').pop()!;
    if (n.status !== 'running') started.set(n.id, ctx.time);
    if (ctx.time - started.get(n.id)! < (durations[st] ?? 1) - 1e-9) return 'running';
    const s = findState(doc, st); const named = typeof s?.action?.args.outcomes === 'string' ? String(s.action.args.outcomes).split(',') : null;
    const o = outcomeOf(script, counts, st, named);
    if (named) { n.outcome = o; return 'success'; }
    return o === (s?.action?.fail ?? 'fail') ? 'failure' : 'success';
  };
  const waits = new Map<string, number>();
  const set = (ctx: TickContext, args: Record<string, unknown>) => { for (const [k, v] of Object.entries(args)) ctx.bb[k] = setValue(v, ctx.bb); return 'success' as const; };
  const wait = (ctx: TickContext, args: Record<string, unknown>, node: BTNode) => { if (node.status !== 'running') waits.set(node.id, ctx.time); return ctx.time - waits.get(node.id)! >= Number(args.seconds) - 1e-9 ? 'success' as const : 'running' as const; };
  const world: ExecutorWorld = { host: { sense: () => ({}), actions: new Proxy({}, { get: (_t, k) => (k === 'set' ? set : k === 'wait' ? wait : skill), has: () => true }) as never, halt: {} }, forRobot: () => null };
  const ex = new FsmExecutor({ doc, world });
  const entered: string[] = [];
  for (let i = 0; i < 20000 && ex.status === 'running'; i++) ex.tick(0.05);
  for (const l of ex.log) { const m = /\] (\S+): skill /.exec(l); if (m) entered.push(m[1].split('/').slice(1).join('/')); }
  return { entered, outcome: ex.outcome };
}
function findState(doc: ReturnType<typeof parseFsm>, name: string) { const walk = (m: typeof doc.root): any => { for (const s of m.states) { if (s.name === name && s.kind !== 'machine') return s; if (s.machine) { const r = walk(s.machine); if (r) return r; } } return null; }; return walk(doc.root); }

function runPlc(dsl: string, name: string, script: Script, durations: Record<string, number> = {}): { entered: string[]; outcome: string | null; src: string } {
  const [stFile, io] = fsmToST(dsl, name); const prog = new StProgram(stFile.content); const doc = parseFsm(dsl);
  // START_<id> -> state: from the I/O map description "<path>/<state> runs its operation"
  const opState = new Map<string, string>();
  for (const line of io.content.split('\n').slice(1)) { const c = line.split(','); if (c[0]?.startsWith('START_')) opState.set(c[0].slice(6), c[c.length - 1].replace(/ runs its operation$/, '')); }
  const counts = new Map<string, number>(); const active = new Map<string, { t0: number; answered: boolean }>(); const entered: string[] = [];
  let t = 0;
  for (let i = 0; i < 40000 && !prog.vars.FSM_DONE; i++) {
    // skill server: answer DONE / RES / FAIL after the duration; reset when START drops
    for (const [id, path] of opState) {
      const start = prog.vars[`START_${id}`]; const a = active.get(id);
      if (start && !a) { active.set(id, { t0: t, answered: false }); entered.push(path.split('/').slice(1).join('/')); }
      if (!start && a) { active.delete(id); prog.vars[`DONE_${id}`] = false; prog.vars[`FAIL_${id}`] = false; if (`RES_${id}` in prog.vars) prog.vars[`RES_${id}`] = 0; }
      const b = active.get(id); const st = path.split('/').pop()!;
      if (b && !b.answered && t - b.t0 >= (durations[st] ?? 1) - 1e-9) {
        b.answered = true;
        const s = findState(doc, st); const named = typeof s?.action?.args.outcomes === 'string' ? String(s.action.args.outcomes).split(',') : null;
        const o = outcomeOf(script, counts, st, named);
        if (named) { prog.vars[`RES_${id}`] = named.indexOf(o) + 1; prog.vars[`DONE_${id}`] = true; }
        else if (o === (s?.action?.fail ?? 'fail')) prog.vars[`FAIL_${id}`] = true; else prog.vars[`DONE_${id}`] = true;
      }
    }
    prog.scan(0.05); t += 0.05;
  }
  // a state entered and answered in consecutive activations shows as START staying high only when re-entered; the
  // handshake drops START for one scan between activations (the state is left), so every entry is seen
  const outcome = prog.vars.FSM_DONE ? doc.root.outcomes[prog.vars.FSM_OUTCOME - 1] : null;
  return { entered, outcome, src: stFile.content };
}

const L = { targets: { home: { x: 0, y: 0, yaw: 0 } }, dock: { approach: { x: 0, y: 0, yaw: 0 }, standoff: { x: 0, y: 0, yaw: 0 }, radius: 0.06 }, slots: {}, basket: {}, catalog: {}, instances: [], frame: 'map' } as never;

describe('FSM -> Structured Text (executed)', () => {
  it('MURKA retail mission: same state sequence and outcome as FsmExecutor (loops, mismatch retry, error continuation)', () => {
    const dsl = missionFsmDsl(parseMission('full', [], L));
    const script: Script = {
      PLAN: ['found', 'none_left'], PLAN_ITEM: ['found', 'found', 'found', 'to_packing', 'finished'], VERIFY_ITEM: ['mismatch', 'ok', 'ok'],
      UNLOAD_PLAN: ['item', 'item', 'empty'], NAV_DOCK: ['fail'], FINISH: ['error'],
    };
    const a = runExecutor(dsl, script); const b = runPlc(dsl, 'MUROM mission', script);
    expect(a.outcome).toBe('error');
    expect(b.outcome).toBe(a.outcome);
    expect(b.entered).toEqual(a.entered);
    expect(a.entered.slice(0, 4)).toEqual(['PH_SCAN', 'SCAN/SCAN', 'PH_DEPAL', 'DEPAL/PLAN']);
    expect(a.entered).toContain('ERR_DOCK');
    expect(a.entered.filter((s) => s === 'ORDER/VERIFY_ITEM')).toHaveLength(3);
  });

  it('decisions, wait bindings and a state timeout behave like the executor', () => {
    const dsl = `fsm Timed
blackboard tries=0
outcomes done failed
state Start initial
  do skill=work
  ok -> Check
  fail -> failed
state Check
  when "tries >= 2" -> enough
  otherwise -> again
  enough -> Pause
  again -> Count
state Count
  do set=tries value="=tries + 1"
  ok -> Start
state Pause
  do wait=1.5
  ok -> Slow
machine Slow
  outcomes fine late
  state Long initial
    do skill=slow
    timeout 2
    ok -> fine
    timeout -> late
  fine -> done
  late -> failed`;
    const durations = { Slow: 1, Long: 5, Start: 0.5 };
    const a = runExecutor(dsl, {}, durations); const b = runPlc(dsl, 'Timed', {}, durations);
    expect(a.outcome).toBe('failed'); // Long takes 5 s > its 2 s timeout
    expect(b.outcome).toBe(a.outcome);
    expect(b.entered).toEqual(a.entered);
    expect(a.entered).toEqual(['Start', 'Start', 'Start', 'Slow/Long']);
  });

  it('the Export menu of an fsm model offers ST; the program declares every assigned variable and balances its blocks', () => {
    const dsl = missionFsmDsl(parseMission('full', [], L));
    const e = controlExports('fsm', dsl, 'MUROM mission');
    expect(e.map((x) => x.id)).toContain('st');
    const [st, io] = e.find((x) => x.id === 'st')!.files();
    expect(st.name).toBe('MUROM_mission.st');
    expect(() => new StProgram(st.content)).not.toThrow();
    const prog = new StProgram(st.content); for (let i = 0; i < 10; i++) prog.scan(0.05); // scans run with all inputs low
    expect(prog.vars.START_PH_SCAN).toBe(true);
    expect(io.content.split('\n').filter((l) => l.startsWith('START_'))).toHaveLength(26);
  });

  it('PLCopen SFC (flattened): well-formed references, and the chart run as a token game visits the same states as FsmExecutor', () => {
    const dsl = missionFsmDsl(parseMission('full', [], L));
    const xml = fsmToPlcOpen(dsl, 'MUROM mission');
    const rootX = parseXml(xml.content);
    const all: XmlNode[] = []; const walkX = (n: XmlNode) => { all.push(n); n.children.forEach(walkX); }; walkX(rootX);
    const ids = new Set(all.filter((n) => n.attrs.localId).map((n) => n.attrs.localId));
    const refs = all.filter((n) => n.name === 'connection').map((n) => n.attrs.refLocalId);
    expect(refs.length).toBeGreaterThan(50);
    for (const r of refs) expect(ids.has(r), r).toBe(true);
    const flat = flattenFsm(dsl);
    expect(all.filter((n) => n.name === 'step')).toHaveLength(flat.steps.length);
    expect(all.filter((n) => n.name === 'transition')).toHaveLength(flat.transitions.length);
    expect(all.filter((n) => n.name === 'step' && n.attrs.initialStep === 'true').map((n) => n.attrs.name)).toEqual(['PH_SCAN']);
    // token game with the scripted skill server on the handshake signals
    const script: Script = { PLAN: ['found', 'none_left'], PLAN_ITEM: ['found', 'found', 'found', 'to_packing', 'finished'], VERIFY_ITEM: ['mismatch', 'ok', 'ok'], UNLOAD_PLAN: ['item', 'item', 'empty'], NAV_DOCK: ['fail'], FINISH: ['error'] };
    const doc = parseFsm(dsl); const counts = new Map<string, number>();
    const conds = new Map(flat.transitions.map((t) => [t, StProgram.parseExpr(t.cond)]));
    let step = flat.initial; let t0 = 0; let t = 0; const entered: string[] = [];
    const env: Record<string, any> = {};
    const leafState = (stp: string) => findStateByStep(doc, stp);
    let answered = false;
    for (let i = 0; i < 40000 && !step.startsWith('END_'); i++) {
      const stDef = leafState(step); const named = typeof stDef?.action?.args.outcomes === 'string' ? String(stDef.action.args.outcomes).split(',') : null;
      if (t - t0 === 0 && !answered) entered.push(step);
      if (!answered && t - t0 >= 1 - 1e-9) {
        answered = true; const o = outcomeOf(script, counts, stDef.name, named);
        if (named) { env[`DONE_${step}`] = true; env[`RES_${step}`] = named.indexOf(o) + 1; } else if (o === (stDef.action?.fail ?? 'fail')) env[`FAIL_${step}`] = true; else env[`DONE_${step}`] = true;
      }
      env[`${step}.T`] = t - t0;
      const fire = flat.transitions.filter((x) => x.from === step).find((x) => !!conds.get(x)!(new Proxy(env, { get: (o2, k) => (k in o2 ? o2[k as string] : false), has: () => true })));
      t += 0.05;
      if (fire) { env[`DONE_${step}`] = false; env[`FAIL_${step}`] = false; env[`RES_${step}`] = 0; step = fire.to; t0 = t; answered = false; }
    }
    const want = runExecutor(dsl, script);
    expect(step).toBe(`END_${want.outcome}`);
    const names = entered.map((x) => findStateByStep(doc, x)!.name);
    expect(names).toEqual(want.entered.map((p) => p.split('/').pop()));
  });
});
function findStateByStep(doc: ReturnType<typeof parseFsm>, step: string): any {
  const walk = (m: typeof doc.root, prefix: string): any => { for (const s of m.states) { const id = `${prefix}${s.name}`; if (s.machine) { const r = walk(s.machine, `${id}_`); if (r) return r; } else if (id === step) return s; } return null; };
  return walk(doc.root, '');
}

// ------------------------------------------------------------------ directly represented addresses and handshake edges
/** Bytes of a directly represented address: CODESYS indexes words / dwords (%IWn = bytes 2n..), Siemens uses byte offsets. */
function addrBytes(addr: string, model: 'codesys' | 'siemens'): { area: string; cells: Array<[number, number | null]> } {
  const m = /^%([IQ])([XWD])(\d+)(?:\.(\d+))?$/.exec(addr)!; const area = m[1], sz = m[2], n = Number(m[3]);
  if (sz === 'X') return { area, cells: [[n, Number(m[4] ?? 0)]] };
  const nb = sz === 'W' ? 2 : 4; const start = model === 'codesys' ? n * nb : n;
  return { area, cells: Array.from({ length: nb }, (_, k) => [start + k, null] as [number, null]) };
}
function overlaps(st: string): string[] {
  const decls = [...st.matchAll(/^\s*(\w+) AT (%\S+) : (\w+);/gm)].map((x) => ({ n: x[1], a: x[2], t: x[3] }));
  const out: string[] = [];
  for (const d of decls) { const sz = /^%[IQ]([XWD])/.exec(d.a)![1]; const bits = { X: 1, W: 16, D: 32 }[sz]!; const tb = { BOOL: 1, INT: 16, REAL: 32 }[d.t as 'BOOL']; if (tb !== bits) out.push(`${d.n}: ${d.t} at ${d.a}`); }
  for (const model of ['codesys', 'siemens'] as const) {
    const occ = new Map<string, Array<{ n: string; bit: number | null }>>();
    for (const d of decls) { const { area, cells } = addrBytes(d.a, model); for (const [b, bit] of cells) { const k = `${area}${b}`; for (const o of occ.get(k) ?? []) if (bit === null || o.bit === null || bit === o.bit) out.push(`[${model}] ${d.n} (${d.a}) overlaps ${o.n}`); occ.set(k, [...(occ.get(k) ?? []), { n: d.n, bit }]); } }
  }
  return out;
}
const DES_2R = `des Two machines and a buffer
automaton RA
  initial I
  marked I F
  I -sA-> P
  P -fA-> D
  D -sC-> Q
  Q -fC-> F
automaton RB
  initial I
  marked I F
  I -sB-> W
  W -fB-> F
spec Buffer
  events fA sB
  initial E
  marked E G
  E -fA-> A
  A -sB-> G
uncontrollable fA fB fC
action RA.P program=A_Pick robot=R1 done=fA
action RA.Q program=A_Place robot=R1 done=fC
action RB.W program=B_Work robot=R2 done=fB`;
const FSM_GUARD = `fsm Pick all boxes
outcomes done aborted
state PickA initial
  do program=A_Pick timeout=30
  fail -> aborted
  ok -> Work
state Work
  do skill=inspect outcomes=good,bad
  good -> Check
  bad -> aborted
state Check
  when "Ready == 1" -> yes
  otherwise -> no
  yes -> PlaceA
  no -> aborted
state PlaceA
  do program=A_Place
  ok -> done
  fail -> aborted`;

describe('PLC programs: I/O addresses and handshake edges', () => {
  it('FSM, automaton and Petri ST: bits, INT words and REAL inputs never share bytes (CODESYS and Siemens address models)', () => {
    const fsm = fsmToST(FSM_GUARD, 'fsm')[0].content;
    expect(fsm).toMatch(/Ready AT %ID\d+ : REAL;/); expect(fsm).toMatch(/RES_Work AT %IW\d+ : INT;/); expect(fsm).toMatch(/FSM_OUTCOME AT %QW\d+ : INT;/);
    expect(overlaps(fsm)).toEqual([]);
    const des = automatonToST(DES_2R.replace('action RB.W', 'guard sC "Ready == 1"\naction RB.W'), 'des')[0].content;
    expect(overlaps(des)).toEqual([]);
    const petri = petriToST('petri g\nplace a tokens=1\nplace b\nplace c\nplace d\ntransition t1\ntransition t2 guard="Ready == 1 && level > 0.5"\ntransition t3 guard="go"\narc a -> t1 -> b\narc b -> t2 -> c\narc c -> t3 -> d\naction t1 program=P1\naction t2 program=P2', 'petri')[0].content;
    expect(petri).toMatch(/level AT %ID\d+ : REAL;/); expect(petri).toMatch(/go AT %IX\d\.\d : BOOL;/);
    expect(overlaps(petri)).toEqual([]);
    // executed against a scripted cell: A_Pick, the 2-outcome skill, the decision on Ready, A_Place
    const prog = new StProgram(fsm); const answered = new Set<string>(); const starts: string[] = [];
    for (let i = 0; i < 400 && !prog.vars.FSM_DONE; i++) {
      for (const op of ['PickA', 'Work', 'PlaceA']) {
        if (prog.vars[`START_${op}`] && !answered.has(op)) { answered.add(op); starts.push(op); if (op === 'Work') prog.vars.RES_Work = 1; prog.vars[`DONE_${op}`] = true; if (op === 'Work') prog.vars.Ready = 1; }
        if (!prog.vars[`START_${op}`] && answered.has(op)) prog.vars[`DONE_${op}`] = false;
      }
      prog.scan(0.05);
    }
    expect(starts).toEqual(['PickA', 'Work', 'PlaceA']); expect(prog.vars.FSM_OUTCOME).toBe(1);
  });
  it('automaton ST: two operations finishing in the same PLC scan are both taken (no lost DONE edge, no handshake deadlock)', () => {
    const [st] = automatonToST(DES_2R, 'des');
    for (const [dQ, dB] of [[20, 30], [20, 19], [20, 20], [7, 6]]) {
      const prog = new StProgram(st.content);
      const ops: Record<string, [string, string, number]> = { A_Pick: ['START_RA_P', 'DONE_RA_P', 5], A_Place: ['START_RA_Q', 'DONE_RA_Q', dQ], B_Work: ['START_RB_W', 'DONE_RB_W', dB] };
      const srv = Object.fromEntries(Object.keys(ops).map((k) => [k, { state: 'idle', busy: 0 }]));
      const doneAt: Record<string, number> = {};
      for (let k = 0; k < 300; k++) {
        for (const [n, [sv, dv, d]] of Object.entries(ops)) {
          const s = srv[n];
          if (s.state === 'idle' && prog.vars[sv]) { s.state = 'run'; s.busy = d; }
          else if (s.state === 'run' && --s.busy <= 0) { s.state = 'done'; prog.vars[dv] = true; doneAt[n] = k; }
          else if (s.state === 'done' && !prog.vars[sv]) { s.state = 'idle'; prog.vars[dv] = false; }
        }
        prog.scan(0.05);
      }
      expect({ RA: prog.vars.RA_state, RB: prog.vars.RB_state, starts: Object.values(srv).map((s) => s.state) }, `A_Place ${dQ} / B_Work ${dB} scans (DONE at ${JSON.stringify(doneAt)})`).toEqual({ RA: 4, RB: 2, starts: ['idle', 'idle', 'idle'] });
    }
  });
});
