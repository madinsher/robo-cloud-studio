/**
 * IEC 61131-3 Structured Text from an `fsm` model (hierarchical yasmin-style machines), with an I/O map:
 *  - one INT state variable per machine (CASE), an entry flag and an outcome register per machine; a nested machine is
 *    executed inline inside the branch of its parent state, and its outcome is mapped to the parent state's outcome;
 *  - skill / program / move / goto states drive a handshake: START_x is TRUE while the state runs its operation; the
 *    robot or skill server answers DONE_x (rising edge) — for a skill with several outcomes (`outcomes=a,b,c`) RES_x holds
 *    the 1-based index of the outcome (0 = the first) — or FAIL_x for a failure;
 *  - decision states evaluate their `when` expressions in order (`otherwise`), pass states finish at once, `wait=` and
 *    state timeouts are TON timers, `await=` waits for its expression, `set=` / `signal=` bindings and entry / exit
 *    actions are assignments;
 *  - transitions go to a sibling state or end the machine with one of its outcomes (an unmapped outcome that is an outcome
 *    of the enclosing machine ends it, as in yasmin); the root outcome is FSM_OUTCOME (1-based) with FSM_DONE.
 * One transition per machine level per PLC scan; the semantics follow the studio FsmExecutor (src/ctl/fsm.ts).
 */
import { parseFsm, FsmMachine, FsmState, stateOutcomes, initialState, fsmMachines } from '../fsm';
import type { ActionBinding } from '../dsl';
import { GenFile, ident, translateExpr, ST_DIALECT } from './common';
import { allocIo, inputVars, IoPoint } from './plc';

const isOp = (b?: ActionBinding) => !!b && ['program', 'target', 'goto', 'skill'].includes(b.kind);
const stTime = (s: number) => `T#${Math.round(s * 1000)}MS`;
const stVal = (v: unknown) => (typeof v === 'boolean' ? (v ? 'TRUE' : 'FALSE') : typeof v === 'number' ? String(v) : v === 'true' ? 'TRUE' : v === 'false' ? 'FALSE' : Number.isFinite(Number(v)) && String(v).trim() !== '' ? String(Number(v)) : `'${String(v)}'`);

interface Op { id: string; start: string; done: string; fail: string; res: string | null; b: ActionBinding; where: string }

export function fsmToST(source: string, name: string): GenFile[] {
  const doc = parseFsm(source);
  const prog = ident(name);
  const machines = fsmMachines(doc);
  const mId = new Map(machines.map((m, i) => [m.machine, `M${i}`]));
  const idOf = (n: string) => ident(n);
  const st = (e: string) => translateExpr(e, ST_DIALECT(idOf));
  const bb = doc.blackboard ?? {};
  const local = new Set(Object.keys(bb));
  const ops: Op[] = []; const timers: string[] = []; const sigOuts = new Set<string>();
  const exprs: string[] = [];
  const opOf = new Map<FsmState, Op>();
  // collect operations, timers, expressions
  for (const { machine: m, path } of machines) for (const s of m.states) {
    const where = `${path}/${s.name}`;
    const b = s.action;
    for (const a of [...s.entry, ...s.exit, ...(b ? [b] : [])]) { if (a.kind === 'set') local.add(a.value); if (a.kind === 'signal') sigOuts.add(a.value); if (a.kind === 'set' && typeof a.args.value === 'string' && a.args.value.startsWith('=')) exprs.push(a.args.value.slice(1)); }
    if (b && isOp(b)) { const id = ident(`${path.split('/').slice(1).join('_')}_${s.name}`.replace(/^_/, '')); const named = typeof b.args.outcomes === 'string'; const op: Op = { id, start: `START_${id}`, done: `DONE_${id}`, fail: `FAIL_${id}`, res: named ? `RES_${id}` : null, b, where }; ops.push(op); opOf.set(s, op); }
    if (b?.kind === 'await') exprs.push(b.value);
    for (const w of s.when) exprs.push(w.expr);
  }
  const inputs = new Map(inputVars(exprs, local).map((v) => [v.name, v.type]));
  // addresses: bits, words (RES_x, FSM_OUTCOME) and REAL double words in separate regions (allocIo: no overlap in the
  // CODESYS or the Siemens address model)
  const io: IoPoint[] = [];
  for (const o of ops) {
    io.push({ name: o.start, dir: 'out', addr: '', op: `${o.b.kind} ${o.b.value}`, robot: o.b.robot, note: `${o.where} runs its operation` });
    io.push({ name: o.done, dir: 'in', addr: '', op: `${o.b.kind} ${o.b.value}`, robot: o.b.robot, note: 'rises when the operation finished' });
    io.push({ name: o.fail, dir: 'in', addr: '', op: `${o.b.kind} ${o.b.value}`, robot: o.b.robot, note: 'rises when the operation failed' });
    if (o.res) io.push({ name: o.res, dir: 'in', addr: '', size: 'W', op: `${o.b.kind} ${o.b.value}`, robot: o.b.robot, note: `outcome index 1..n of ${String(o.b.args.outcomes)} (0 = first), valid with DONE` });
  }
  for (const sg of sigOuts) io.push({ name: ident(sg), dir: 'out', addr: '', note: 'signal written by the controller' });
  for (const [v, t] of inputs) io.push({ name: ident(v), dir: 'in', addr: '', size: t === 'BOOL' ? 'X' : 'D', note: `${t === 'BOOL' ? 'sensor / signal' : 'numeric value (REAL)'} read by decisions` });
  io.push({ name: 'FSM_DONE', dir: 'out', addr: '', note: 'the root machine finished' }, { name: 'FSM_OUTCOME', dir: 'out', addr: '', size: 'W', note: `root outcome 1..n of ${doc.root.outcomes.join(', ')}` });
  allocIo(io);
  const addrOf = (n: string) => io.find((p) => p.name === n)!.addr;
  const L: string[] = [];
  L.push(`(* Generated by VerticalBot Studio from the FSM model "${name}". IEC 61131-3 Structured Text.`,
    `   ${machines.length} machine(s); one INT state per machine, outcome codes are 1-based indexes of each state's outcome list.`,
    '   Handshake: START_x TRUE while the state runs its operation; DONE_x (rising) = finished (RES_x = outcome index for', '   skills with several outcomes), FAIL_x (rising) = failed. *)');
  L.push(`PROGRAM ${prog}`, 'VAR_INPUT');
  for (const o of ops) { L.push(`  ${o.done} AT ${addrOf(o.done)} : BOOL;`, `  ${o.fail} AT ${addrOf(o.fail)} : BOOL;`); if (o.res) L.push(`  ${o.res} AT ${addrOf(o.res)} : INT;`); }
  for (const [v, t] of inputs) L.push(`  ${ident(v)} AT ${addrOf(ident(v))} : ${t};`);
  L.push('END_VAR', 'VAR_OUTPUT');
  for (const o of ops) L.push(`  ${o.start} AT ${addrOf(o.start)} : BOOL; (* ${o.where}: ${o.b.kind} ${o.b.value} *)`);
  for (const sg of sigOuts) L.push(`  ${ident(sg)} AT ${addrOf(ident(sg))} : BOOL;`);
  L.push(`  FSM_DONE AT ${addrOf('FSM_DONE')} : BOOL;`, `  FSM_OUTCOME AT ${addrOf('FSM_OUTCOME')} : INT;`, 'END_VAR', 'VAR');
  for (const { machine: m, path } of machines) {
    const k = mId.get(m)!; const init = m.states.indexOf(initialState(m)!);
    L.push(`  ${k}_state : INT := ${init}; (* ${path}: ${m.states.map((s, i) => `${i}=${s.name}`).join(' ')} *)`, `  ${k}_entry : BOOL := TRUE;`, `  ${k}_o : INT := 0;`, `  ${k}_out : INT := 0; (* ${m.outcomes.map((o, i) => `${i + 1}=${o}`).join(' ') || 'root'} *)`);
  }
  for (const [kk, v] of Object.entries(bb)) L.push(`  ${ident(kk)} : ${typeof v === 'boolean' ? 'BOOL' : 'LREAL'} := ${stVal(v)};`);
  for (const kk of local) if (!(kk in bb)) L.push(`  ${ident(kk)} : LREAL := 0;`);
  for (const o of ops) L.push(`  R_${o.done} : R_TRIG;`, `  R_${o.fail} : R_TRIG;`);
  // timers: state timeouts and wait bindings
  const timerOf = new Map<FsmState, string>(); const waitOf = new Map<FsmState, string>();
  for (const { machine: m, path } of machines) for (const s of m.states) {
    const base = ident(`${path.split('/').slice(1).join('_')}_${s.name}`.replace(/^_/, ''));
    const to = s.timeout ?? s.action?.timeout;
    if (to !== undefined) { const t = `TO_${base}`; timerOf.set(s, t); timers.push(t); L.push(`  ${t} : TON;`); }
    if (s.action?.kind === 'wait') { const t = `TW_${base}`; waitOf.set(s, t); timers.push(t); L.push(`  ${t} : TON;`); }
  }
  L.push('END_VAR', '');
  const assign = (a: ActionBinding): string | null => {
    if (a.kind === 'set') { const v = typeof a.args.value === 'string' && a.args.value.startsWith('=') ? st(a.args.value.slice(1)) : stVal(a.args.value ?? true); return `${ident(a.value)} := ${v};`; }
    if (a.kind === 'signal') return `${ident(a.value)} := ${stVal(a.args.value ?? true)};`;
    return null; // event / log: no PLC effect
  };
  // scan
  L.push('(* inputs: edges of the handshake answers *)');
  for (const o of ops) L.push(`R_${o.done}(CLK := ${o.done});`, `R_${o.fail}(CLK := ${o.fail});`);
  for (const o of ops) L.push(`${o.start} := FALSE;`);
  L.push('', 'IF NOT FSM_DONE THEN');
  const emit = (m: FsmMachine, ind: string) => {
    const k = mId.get(m)!;
    L.push(`${ind}${k}_o := 0;`, `${ind}CASE ${k}_state OF`);
    m.states.forEach((s, i) => {
      const outs = stateOutcomes(s); const code = (o: string) => outs.indexOf(o) + 1;
      const i2 = `${ind}  `, i3 = `${ind}    `;
      L.push(`${i2}${i}: (* ${s.name} *)`);
      const entry: string[] = s.entry.map(assign).filter((x): x is string => !!x);
      const tmo = timerOf.get(s), tw = waitOf.get(s);
      if (tmo) entry.push(`${tmo}(IN := FALSE);`);
      if (tw) entry.push(`${tw}(IN := FALSE);`);
      if (s.kind === 'machine' && s.machine) { const c = mId.get(s.machine)!; entry.push(`${c}_state := ${s.machine.states.indexOf(initialState(s.machine)!)};`, `${c}_entry := TRUE;`, `${c}_out := 0;`); }
      L.push(`${i3}IF ${k}_entry THEN`, ...entry.map((x) => `${i3}  ${x}`), `${i3}  ${k}_entry := FALSE;`, `${i3}END_IF;`);
      if (s.kind === 'pass') L.push(`${i3}${k}_o := ${code('ok')};`);
      else if (s.kind === 'decision') {
        if (s.when.length) { s.when.forEach((w, j) => L.push(`${i3}${j ? 'ELSIF' : 'IF'} ${st(w.expr)} THEN ${k}_o := ${code(w.outcome)};`)); if (s.otherwise) L.push(`${i3}ELSE ${k}_o := ${code(s.otherwise)};`); L.push(`${i3}END_IF;`); }
        else if (s.otherwise) L.push(`${i3}${k}_o := ${code(s.otherwise)};`);
      } else if (s.kind === 'machine' && s.machine) {
        emit(s.machine, i3);
        const c = mId.get(s.machine)!;
        L.push(`${i3}IF ${c}_out <> 0 THEN`, `${i3}  CASE ${c}_out OF`, ...s.machine.outcomes.map((o, j) => `${i3}    ${j + 1}: ${k}_o := ${code(o)};`), `${i3}  END_CASE;`, `${i3}END_IF;`);
      } else if (s.kind === 'skill' && s.action) {
        const b = s.action; const op = opOf.get(s);
        const named = typeof b.args.outcomes === 'string' ? String(b.args.outcomes).split(',').map((x) => x.trim()).filter(Boolean) : null;
        const okO = named ? named[0] : (b.done ?? 'ok'), failO = named ? named[named.length - 1] : (b.fail ?? 'fail');
        if (op) {
          if (op.res) L.push(`${i3}IF R_${op.done}.Q THEN`, `${i3}  IF ${op.res} >= 1 AND ${op.res} <= ${named!.length} THEN`, `${i3}    CASE ${op.res} OF`, ...named!.map((o, j) => `${i3}      ${j + 1}: ${k}_o := ${code(o)};`), `${i3}    END_CASE;`, `${i3}  ELSE`, `${i3}    ${k}_o := ${code(okO)};`, `${i3}  END_IF;`, `${i3}ELSIF R_${op.fail}.Q THEN`, `${i3}  ${k}_o := ${code(failO)};`, `${i3}END_IF;`);
          else L.push(`${i3}IF R_${op.done}.Q THEN ${k}_o := ${code(okO)}; ELSIF R_${op.fail}.Q THEN ${k}_o := ${code(failO)}; END_IF;`);
        } else if (b.kind === 'wait') L.push(`${i3}${tw}(IN := TRUE, PT := ${stTime(Number(b.value))});`, `${i3}IF ${tw}.Q THEN ${k}_o := ${code(okO)}; END_IF;`);
        else if (b.kind === 'await') L.push(`${i3}IF ${st(b.value)} THEN ${k}_o := ${code(okO)}; END_IF;`);
        else { const a = assign(b); if (a) L.push(`${i3}${a}`); L.push(`${i3}${k}_o := ${code(okO)};`); }
        if (op) L.push(`${i3}IF ${k}_o = 0 THEN ${op.start} := TRUE; END_IF;`);
      }
      if (tmo) { const to = s.timeout ?? s.action?.timeout ?? 0; L.push(`${i3}${tmo}(IN := TRUE, PT := ${stTime(to)});`, `${i3}IF ${k}_o = 0 AND ${tmo}.Q THEN ${k}_o := ${outs.includes('timeout') ? code('timeout') : code(s.action?.fail ?? 'fail')}; END_IF;`); }
    });
    L.push(`${ind}END_CASE;`);
    // transitions of this machine
    L.push(`${ind}IF ${k}_o <> 0 THEN`, `${ind}  CASE ${k}_state OF`);
    m.states.forEach((s, i) => {
      const outs = stateOutcomes(s);
      const exit = s.exit.map(assign).filter((x): x is string => !!x);
      const arms: string[] = [];
      outs.forEach((o, j) => {
        let to = s.transitions[o];
        if (to === undefined && o === 'timeout') to = s.transitions[s.action?.fail ?? 'fail'];
        if (to === undefined && m.outcomes.includes(o)) to = o;
        if (to === undefined) return;
        const sib = m.states.findIndex((x) => x.name === to);
        arms.push(sib >= 0 ? `${j + 1}: ${k}_state := ${sib}; ${k}_entry := TRUE;` : `${j + 1}: ${k}_out := ${m.outcomes.indexOf(to) + 1};`);
      });
      L.push(`${ind}    ${i}: (* ${s.name} *)`, ...exit.map((x) => `${ind}      ${x}`), `${ind}      CASE ${k}_o OF`, ...arms.map((a) => `${ind}        ${a}`), `${ind}      END_CASE;`);
    });
    L.push(`${ind}  END_CASE;`, `${ind}END_IF;`);
  };
  emit(doc.root, '  ');
  const r = mId.get(doc.root)!;
  L.push(`  IF ${r}_out <> 0 THEN FSM_DONE := TRUE; FSM_OUTCOME := ${r}_out; END_IF;`, 'END_IF;', 'END_PROGRAM', '');
  const csv = ['signal,direction,address,robot,operation,description', ...io.map((p) => [p.name, p.dir === 'out' ? 'PLC output' : 'PLC input', p.addr, p.robot ?? '', p.op ?? '', p.note].map((c) => (/[,"]/.test(c) ? `"${c.replace(/"/g, '""')}"` : c)).join(','))].join('\n') + '\n';
  return [{ name: `${prog}.st`, content: L.join('\n'), mime: 'text/plain' }, { name: 'io_map.csv', content: csv, mime: 'text/csv' }];
}
