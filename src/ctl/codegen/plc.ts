/**
 * IEC 61131-3 code from control models, with an I/O map derived from the bindings:
 *  - automata (des) → a Structured Text PROGRAM: one INT state variable per component (CASE state machines), the
 *    supervisor table as a CASE over its state, controllable events issued in a fixed priority order when enabled in
 *    every component, allowed by the supervisor and by their guards; uncontrollable events from robot handshakes (rising
 *    edge of DONE_x), TON timers (wait bindings) and sensor rising edges (R_TRIG);
 *  - Petri nets → a Structured Text PROGRAM (one INT marking per place, START / DONE handshake per bound transition, TON
 *    timers for delays, single-server firing) and a PLCopen XML (TC6 v2.01) project with the same net as an SFC
 *    (places → steps, bound / timed transitions expanded into start → operation step → end), for 1-safe nets (Grafcet);
 *  - the I/O map (CSV): every START_x output and DONE_x input with its address, robot and program.
 * Robot side: each robot runs a server program that waits for START_x, runs the operation and answers DONE_x
 * (see vendor.ts).
 */
import { parseDes, parsePetri, ActionBinding } from '../dsl';
import { DES } from '../des';
import { PetriNet } from '../petri';
import { synthesizeSupervisor } from '../supervisor';
import { GenFile, ident, translateExpr, ST_DIALECT, exprIds, xmlEsc } from './common';

export interface IoPoint { name: string; dir: 'out' | 'in'; addr: string; robot?: string; op?: string; note: string; /** X = BOOL bit (default), W = 16-bit word (INT), D = 32-bit double word (REAL) */ size?: 'X' | 'W' | 'D' }
/**
 * Directly represented addresses of the I/O points, without overlaps in both address models: CODESYS (word / dword
 * index: %IWn = bytes 2n..2n+1, %IDn = bytes 4n..4n+3) and Siemens (byte offset: %IWn = bytes n..n+1, %IDn = bytes
 * n..n+3). Per area (I / Q): bits packed from byte 0, then words at even byte offsets after the bit bytes, then double
 * words at offsets that are multiples of 4 after the words - a Siemens offset k maps to CODESYS bytes >= k, so the
 * regions stay disjoint in both models.
 */
export function allocIo(points: IoPoint[]): IoPoint[] {
  for (const dir of ['in', 'out'] as const) {
    const area = dir === 'in' ? 'I' : 'Q'; const ps = points.filter((p) => p.dir === dir);
    const bits = ps.filter((p) => (p.size ?? 'X') === 'X'), words = ps.filter((p) => p.size === 'W'), dwords = ps.filter((p) => p.size === 'D');
    bits.forEach((p, k) => { p.addr = `%${area}X${Math.floor(k / 8)}.${k % 8}`; });
    let off = Math.ceil(bits.length / 8); off += off % 2;
    words.forEach((p) => { p.addr = `%${area}W${off}`; off += 2; });
    off = Math.ceil(off / 4) * 4;
    dwords.forEach((p) => { p.addr = `%${area}D${off}`; off += 4; });
  }
  return points;
}
const isOp = (b?: ActionBinding) => !!b && ['program', 'target', 'goto', 'skill'].includes(b.kind);
const stTime = (s: number) => `T#${Math.round(s * 1000)}MS`;
const stVal = (v: unknown) => (typeof v === 'boolean' ? (v ? 'TRUE' : 'FALSE') : typeof v === 'number' ? (Number.isInteger(v) ? `${v}` : `${v}`) : v === 'true' ? 'TRUE' : v === 'false' ? 'FALSE' : Number.isFinite(Number(v)) ? String(Number(v)) : `'${String(v)}'`);
function ioMap(points: IoPoint[]): GenFile {
  return { name: 'io_map.csv', mime: 'text/csv', content: ['signal,direction,address,robot,operation,description', ...points.map((p) => [p.name, p.dir === 'out' ? 'PLC output → robot input' : 'robot output → PLC input', p.addr, p.robot ?? '', p.op ?? '', p.note].map((c) => (/[,"]/.test(c) ? `"${c.replace(/"/g, '""')}"` : c)).join(','))].join('\n') + '\n' };
}
/** Variables read by guards / sensors: BOOL when only used as truth values, REAL (a 32-bit %ID input) otherwise. */
export function inputVars(exprs: string[], local: Set<string>): Array<{ name: string; type: 'BOOL' | 'REAL' }> {
  const out = new Map<string, 'BOOL' | 'REAL'>();
  for (const e of exprs) for (const id of exprIds(e)) { if (local.has(id)) continue; const numeric = new RegExp(`\\b${id.replace(/\./g, '\\.')}\\s*(<|>|<=|>=|==|!=|=|\\+|-|\\*|/)|(<|>|<=|>=|==|!=|=|\\+|-|\\*|/)\\s*${id.replace(/\./g, '\\.')}\\b`).test(e); out.set(id, numeric || out.get(id) === 'REAL' ? 'REAL' : 'BOOL'); }
  return [...out].map(([name, type]) => ({ name, type }));
}

// ---------------------------------------------------------------------------------------------
// Automata → ST
// ---------------------------------------------------------------------------------------------

export function automatonToST(source: string, name: string): GenFile[] {
  const doc = parseDes(source); const comps = doc.plant.map((a) => DES.fromSpec(a));
  const prog = ident(name); const uc = new Set(doc.uncontrollable);
  const events = [...new Set(comps.flatMap((c) => [...c.sigma]))];
  const evId = new Map(events.map((e, i) => [e, i + 1]));
  const stateCode = (c: DES) => new Map([...c.X].map((x, i) => [x, i]));
  const codes = new Map(comps.map((c) => [c.name, stateCode(c)]));
  const sv = (c: DES) => `${ident(c.name)}_state`;
  const actionFor = (c: DES, s: string) => doc.actions.find((a) => a.target === `${c.name}.${s}`) ?? doc.actions.find((a) => !a.target.includes('.') && a.target === s && c.X.has(s));
  const bb = doc.blackboard ?? {}; const local = new Set(Object.keys(bb));
  for (const a of doc.actions) if (a.kind === 'set') local.add(a.value);
  const exprs = [...Object.values(doc.guards ?? {}), ...(doc.sensors ?? []).map((s) => s.expr), ...doc.actions.filter((a) => a.kind === 'await').map((a) => a.value), ...doc.actions.filter((a) => a.kind === 'set' && typeof a.args.value === 'string' && a.args.value.startsWith('=')).map((a) => String(a.args.value).slice(1))];
  const inputs = inputVars(exprs, local);
  const idOf = (n: string) => ident(n);
  const st = (e: string) => translateExpr(e, ST_DIALECT(idOf));
  // operations and IO
  const io: IoPoint[] = [];
  const ops: Array<{ c: DES; s: string; b: ActionBinding; start: string; done: string }> = [];
  const timers: Array<{ c: DES; s: string; secs: number; t: string }> = [];
  for (const c of comps) for (const s of c.X) {
    const b = actionFor(c, s); if (!b) continue;
    if (isOp(b)) { const id = ident(`${c.name}_${s}`); ops.push({ c, s, b, start: `START_${id}`, done: `DONE_${id}` }); io.push({ name: `START_${id}`, dir: 'out', addr: '', robot: b.robot, op: `${b.kind} ${b.value}`, note: `${c.name} in ${s}` }, { name: `DONE_${id}`, dir: 'in', addr: '', robot: b.robot, op: `${b.kind} ${b.value}`, note: `rises when the operation finished` }); }
    else if (b.kind === 'wait') timers.push({ c, s, secs: Number(b.value), t: `T_${ident(`${c.name}_${s}`)}` });
  }
  const sigOuts = [...new Set(doc.actions.filter((a) => a.kind === 'signal').map((a) => a.value))];
  for (const sg of sigOuts) io.push({ name: ident(sg), dir: 'out', addr: '', note: 'signal written by the controller' });
  for (const v of inputs) io.push({ name: ident(v.name), dir: 'in', addr: '', size: v.type === 'BOOL' ? 'X' : 'D', note: `${v.type === 'BOOL' ? 'sensor / signal' : 'numeric value (REAL)'} read by guards` });
  allocIo(io);
  const sup = doc.specs.length ? synthesizeSupervisor(doc) : null;
  const L: string[] = [];
  L.push(`(* Generated by VerticalBot Studio from the automaton model "${name}". IEC 61131-3 Structured Text.`, `   One state variable per component; ${sup ? `supervisor table of ${sup.table.states.length} states (${sup.table.observer ? 'observer-based: unobservable events do not move it' : 'full observation'})` : 'no supervisor'}.`, `   Robot handshake: START_x is TRUE while the component is in the state; the robot answers DONE_x when finished. *)`);
  L.push(`PROGRAM ${prog}`, 'VAR_INPUT');
  for (const o of ops) L.push(`  ${o.done} AT ${io.find((p) => p.name === o.done)!.addr} : BOOL; (* ${o.b.kind} ${o.b.value}${o.b.robot ? ` on ${o.b.robot}` : ''} finished *)`);
  for (const v of inputs) L.push(`  ${ident(v.name)} AT ${io.find((p) => p.name === ident(v.name))!.addr} : ${v.type};`);
  L.push('END_VAR', 'VAR_OUTPUT');
  for (const o of ops) L.push(`  ${o.start} AT ${io.find((p) => p.name === o.start)!.addr} : BOOL; (* ${o.c.name} in ${o.s}: ${o.b.kind} ${o.b.value} *)`);
  for (const sg of sigOuts) L.push(`  ${ident(sg)} AT ${io.find((p) => p.name === ident(sg))!.addr} : BOOL;`);
  L.push('END_VAR', 'VAR CONSTANT');
  for (const [e, i] of evId) L.push(`  EV_${ident(e)} : INT := ${i};${uc.has(e) ? ' (* uncontrollable *)' : ''}`);
  L.push('END_VAR', 'VAR');
  for (const c of comps) { L.push(`  ${sv(c)} : INT := ${codes.get(c.name)!.get(c.x0)}; (* ${[...codes.get(c.name)!].map(([x, i]) => `${i}=${x}`).join(' ')} *)`, `  ${sv(c)}_prev : INT := -1;`); }
  if (sup) L.push(`  SUP : INT := ${sup.table.initial};`);
  L.push('  ev : INT := 0;');
  for (const [k, v] of Object.entries(bb)) L.push(`  ${ident(k)} : ${typeof v === 'boolean' ? 'BOOL' : 'LREAL'} := ${stVal(v)};`);
  for (const k of local) if (!(k in bb)) L.push(`  ${ident(k)} : LREAL := 0;`);
  for (const o of ops) L.push(`  R_${o.done} : R_TRIG;`);
  for (const o of ops) if (o.b.done) L.push(`  P_${o.done} : BOOL := FALSE; (* DONE edge seen, ${o.b.done} not yet taken *)`);
  for (const t of timers) L.push(`  ${t.t} : TON;`);
  (doc.sensors ?? []).forEach((s, i) => L.push(`  SENS_${i} : R_TRIG; (* ${s.event} on rising ${s.expr} *)`, `  P_SENS_${i} : BOOL := FALSE;`));
  L.push('END_VAR', '');
  // 1. uncontrollable events
  // one event per scan: the rising edges of DONE_x / sensors are latched (P_x) until their event is taken, so two
  // operations finishing in the same scan are both seen (R_TRIG.Q is TRUE for a single scan only)
  L.push('(* 1. inputs: handshakes, timers, sensors → one uncontrollable event per scan (edges latched until taken) *)', 'ev := 0;');
  for (const o of ops) L.push(`R_${o.done}(CLK := ${o.done});`);
  for (const o of ops) if (o.b.done) L.push(`IF R_${o.done}.Q AND ${sv(o.c)} = ${codes.get(o.c.name)!.get(o.s)} THEN P_${o.done} := TRUE; END_IF;`);
  for (const t of timers) L.push(`${t.t}(IN := ${sv(t.c)} = ${codes.get(t.c.name)!.get(t.s)}, PT := ${stTime(t.secs)});`);
  (doc.sensors ?? []).forEach((s, i) => L.push(`SENS_${i}(CLK := ${st(s.expr)});`, `IF SENS_${i}.Q THEN P_SENS_${i} := TRUE; END_IF;`));
  for (const o of ops) if (o.b.done) L.push(`IF ev = 0 AND ${sv(o.c)} = ${codes.get(o.c.name)!.get(o.s)} AND P_${o.done} THEN ev := EV_${ident(o.b.done)}; P_${o.done} := FALSE; END_IF;`);
  for (const t of timers) { const b = actionFor(t.c, t.s)!; if (b.done) L.push(`IF ev = 0 AND ${t.t}.Q THEN ev := EV_${ident(b.done)}; END_IF;`); }
  (doc.sensors ?? []).forEach((s, i) => { if (evId.has(s.event)) L.push(`IF ev = 0 AND P_SENS_${i} THEN ev := EV_${ident(s.event)}; P_SENS_${i} := FALSE; END_IF;`); });
  // settled components
  const settled = (c: DES) => { const parts = ops.filter((o) => o.c === c && !o.b.done).map((o) => `(${sv(c)} <> ${codes.get(c.name)!.get(o.s)} OR ${o.done})`); const busy = [...ops.filter((o) => o.c === c && o.b.done), ...timers.filter((t) => t.c === c)].map((o) => `${sv(c)} <> ${codes.get(c.name)!.get(o.s)}`); const all = [...parts, ...busy]; return all.length ? all.join(' AND ') : 'TRUE'; };
  L.push('', '(* 2. controllable events in priority order: enabled in every component (all settled), guard, supervisor *)');
  for (const e of events.filter((x) => !uc.has(x))) {
    const conds: string[] = ['ev = 0'];
    for (const c of comps) if (c.sigma.has(e)) { const xs = [...c.X].filter((x) => c.next(x, e) !== undefined).map((x) => codes.get(c.name)!.get(x)); conds.push(xs.length ? `(${xs.map((v) => `${sv(c)} = ${v}`).join(' OR ')})` : 'FALSE'); conds.push(`(${settled(c)})`); }
    const g = doc.guards?.[e]; if (g) conds.push(`(${st(g)})`);
    if (sup) { const ss = sup.table.states.filter((s) => s.enabled_controllable.includes(e)).map((s) => s.id); conds.push(ss.length ? `(${ss.map((v) => `SUP = ${v}`).join(' OR ')})` : 'FALSE'); }
    L.push(`IF ${conds.join(' AND ')} THEN ev := EV_${ident(e)}; END_IF;`);
  }
  // 3. apply
  L.push('', '(* 3. the event moves the supervisor and every component that has it *)', 'IF ev <> 0 THEN');
  if (sup) {
    L.push('  CASE SUP OF');
    for (const s of sup.table.states) { const nx = Object.entries(s.next).filter(([e]) => evId.has(e)); if (!nx.length) continue; L.push(`    ${s.id}: ${nx.map(([e, n], k) => `${k ? 'ELSIF' : 'IF'} ev = EV_${ident(e)} THEN SUP := ${n};`).join(' ')} END_IF;`); }
    L.push('  END_CASE;');
  }
  for (const c of comps) {
    L.push(`  CASE ${sv(c)} OF`);
    for (const x of c.X) { const nx = c.active(x); if (!nx.length) continue; L.push(`    ${codes.get(c.name)!.get(x)}: ${nx.map((e, k) => `${k ? 'ELSIF' : 'IF'} ev = EV_${ident(e)} THEN ${sv(c)} := ${codes.get(c.name)!.get(c.next(x, e)!)};`).join(' ')} END_IF; (* ${x} *)`); }
    L.push('  END_CASE;');
  }
  L.push('END_IF;', '', '(* 4. outputs and entry actions; a latched DONE edge is dropped when its component has left the state *)');
  for (const o of ops) if (o.b.done) L.push(`IF ${sv(o.c)} <> ${codes.get(o.c.name)!.get(o.s)} THEN P_${o.done} := FALSE; END_IF;`);
  for (const o of ops) L.push(`${o.start} := ${sv(o.c)} = ${codes.get(o.c.name)!.get(o.s)};`);
  for (const c of comps) {
    const entries = [...c.X].map((x) => ({ x, b: actionFor(c, x) })).filter((e) => e.b && (e.b.kind === 'set' || e.b.kind === 'signal'));
    if (entries.length) { L.push(`IF ${sv(c)} <> ${sv(c)}_prev THEN`); for (const { x, b } of entries) { const v = b!.kind === 'set' && typeof b!.args.value === 'string' && b!.args.value.startsWith('=') ? st(String(b!.args.value).slice(1)) : stVal(b!.args.value ?? true); L.push(`  IF ${sv(c)} = ${codes.get(c.name)!.get(x)} THEN ${ident(b!.value)} := ${v}; END_IF;`); } L.push('END_IF;'); }
    L.push(`${sv(c)}_prev := ${sv(c)};`);
  }
  L.push('END_PROGRAM', '');
  return [{ name: `${prog}.st`, content: L.join('\n'), mime: 'text/plain' }, ioMap(io)];
}

// ---------------------------------------------------------------------------------------------
// Petri nets → ST and PLCopen XML SFC
// ---------------------------------------------------------------------------------------------

interface NetCode { net: PetriNet; io: IoPoint[]; ops: Map<number, { b: ActionBinding; start: string; done: string }>; name: string; guards: Map<number, string>; inputs: Array<{ name: string; type: 'BOOL' | 'REAL' }> }
function netCode(source: string, name: string): NetCode {
  const doc = parsePetri(source); const net = new PetriNet(doc.spec);
  const io: IoPoint[] = []; const ops = new Map<number, { b: ActionBinding; start: string; done: string }>();
  net.transitions.forEach((t, j) => { const b = doc.actions.find((a) => a.target === t.id); if (isOp(b)) { const id = ident(t.id); ops.set(j, { b: b!, start: `START_${id}`, done: `DONE_${id}` }); io.push({ name: `START_${id}`, dir: 'out', addr: '', robot: b!.robot, op: `${b!.kind} ${b!.value}`, note: `transition ${t.id} running` }, { name: `DONE_${id}`, dir: 'in', addr: '', robot: b!.robot, op: `${b!.kind} ${b!.value}`, note: 'rises when the operation finished' }); } });
  const guards = new Map<number, string>(); net.transitions.forEach((t, j) => { if (t.guard) guards.set(j, t.guard); });
  const inputs = inputVars([...guards.values()], new Set());
  for (const v of inputs) io.push({ name: ident(v.name), dir: 'in', addr: '', size: v.type === 'BOOL' ? 'X' : 'D', note: `read by a transition guard${v.type === 'REAL' ? ' (numeric value, REAL)' : ''}` });
  allocIo(io);
  return { net, io, ops, name, guards, inputs };
}

export function petriToST(source: string, name: string): GenFile[] {
  const c = netCode(source, name); const { net } = c; const prog = ident(name);
  const st = (e: string) => translateExpr(e, ST_DIALECT((n) => ident(n)));
  const m = (i: number) => `m_${ident(net.places[i].id)}`;
  const enabled = (j: number) => { const parts: string[] = []; net.places.forEach((p, i) => { if (net.pre[j][i]) parts.push(`${m(i)} >= ${net.pre[j][i]}`); if (net.inhibitor[j][i]) parts.push(`${m(i)} < ${net.inhibitor[j][i]}`); if (p.capacity !== undefined && net.post[j][i]) parts.push(`${m(i)} - ${net.pre[j][i]} + ${net.post[j][i]} <= ${p.capacity}`); }); return parts.length ? parts.join(' AND ') : 'TRUE'; };
  const consume = (j: number) => net.places.map((_, i) => (net.pre[j][i] ? `${m(i)} := ${m(i)} - ${net.pre[j][i]};` : '')).filter(Boolean).join(' ');
  const produce = (j: number) => net.places.map((_, i) => (net.post[j][i] ? `${m(i)} := ${m(i)} + ${net.post[j][i]};` : '')).filter(Boolean).join(' ');
  const order = net.transitions.map((_, j) => j).sort((a, b) => Number(!!net.transitions[b].immediate) - Number(!!net.transitions[a].immediate) || (net.transitions[b].priority ?? 0) - (net.transitions[a].priority ?? 0) || a - b);
  const L: string[] = [`(* Generated by VerticalBot Studio from the Petri net "${name}". IEC 61131-3 Structured Text.`, '   One INT marking per place; a bound transition starts when enabled (single server), raises START_x and completes on the', '   rising edge of DONE_x; delays use TON timers; immediate transitions fire first. *)', `PROGRAM ${prog}`, 'VAR_INPUT'];
  for (const [, o] of c.ops) L.push(`  ${o.done} AT ${c.io.find((p) => p.name === o.done)!.addr} : BOOL; (* ${o.b.kind} ${o.b.value}${o.b.robot ? ` on ${o.b.robot}` : ''} finished *)`);
  for (const v of c.inputs) L.push(`  ${ident(v.name)} AT ${c.io.find((p) => p.name === ident(v.name))!.addr} : ${v.type};`);
  L.push('END_VAR', 'VAR_OUTPUT');
  for (const [, o] of c.ops) L.push(`  ${o.start} AT ${c.io.find((p) => p.name === o.start)!.addr} : BOOL;`);
  L.push('END_VAR', 'VAR');
  net.places.forEach((p, i) => L.push(`  ${m(i)} : INT := ${p.tokens ?? 0};${p.label ? ` (* ${p.label} *)` : ''}`));
  net.transitions.forEach((t, j) => { L.push(`  busy_${ident(t.id)} : BOOL := FALSE;`); if (c.ops.has(j)) L.push(`  R_${ident(t.id)} : R_TRIG;`); if (!c.ops.has(j) && (t.delay ?? 0) > 0) L.push(`  T_${ident(t.id)} : TON;`); });
  L.push('END_VAR', '', '(* completions *)');
  net.transitions.forEach((t, j) => {
    const id = ident(t.id);
    if (c.ops.has(j)) { const o = c.ops.get(j)!; L.push(`R_${id}(CLK := ${o.done});`, `IF busy_${id} AND R_${id}.Q THEN ${produce(j)} busy_${id} := FALSE; ${o.start} := FALSE; END_IF;`); }
    else if ((t.delay ?? 0) > 0) L.push(`T_${id}(IN := busy_${id}, PT := ${stTime(t.delay!)});`, `IF busy_${id} AND T_${id}.Q THEN ${produce(j)} busy_${id} := FALSE; END_IF;`);
  });
  L.push('', '(* firings (immediate first, then priority, then document order) *)');
  for (const j of order) {
    const t = net.transitions[j]; const id = ident(t.id); const g = c.guards.get(j);
    const cond = `NOT busy_${id} AND ${enabled(j)}${g ? ` AND (${st(g)})` : ''}`;
    if (c.ops.has(j)) L.push(`IF ${cond} THEN ${consume(j)} busy_${id} := TRUE; ${c.ops.get(j)!.start} := TRUE; END_IF;`);
    else if ((t.delay ?? 0) > 0) L.push(`IF ${cond} THEN ${consume(j)} busy_${id} := TRUE; END_IF;`);
    else L.push(`IF ${cond} THEN ${consume(j)} ${produce(j)} END_IF;`);
  }
  L.push('END_PROGRAM', '');
  return [{ name: `${prog}.st`, content: L.join('\n'), mime: 'text/plain' }, ioMap(c.io)];
}

/**
 * PLCopen XML (TC6 v2.01) project: the net as an SFC program with its I/O variables (1-safe nets, Grafcet). Places
 * become steps (initial step = marked place); a bound or timed transition t becomes `t_start` → step `X_t` (action N
 * START_t) → `t_end` (condition DONE_t, or X_t.T ≥ delay). Several input places → simultaneous convergence, several
 * outputs → simultaneous divergence, a step with several consuming transitions → selection divergence, several
 * producing transitions → selection convergence.
 */
export function petriToPlcOpen(source: string, name: string): GenFile {
  const c = netCode(source, name); const { net } = c; const prog = ident(name);
  if (net.places.some((p) => (p.tokens ?? 0) > 1)) throw new Error('SFC export needs a 1-safe net (no place with more than one initial token)');
  const st = (e: string) => translateExpr(e, ST_DIALECT((n) => ident(n)));
  // graph: steps and transitions
  interface S { id: number; name: string; initial: boolean; action?: string; ins: number[]; outs: number[] }
  interface T { id: number; label: string; cond: string; ins: number[]; outs: number[] }
  let lid = 0; const nid = () => ++lid;
  const steps: S[] = net.places.map((p) => ({ id: nid(), name: ident(p.id), initial: (p.tokens ?? 0) === 1, ins: [], outs: [] }));
  const trs: T[] = [];
  const link = (t: T) => { trs.push(t); for (const s of t.ins) steps.find((x) => x.id === s)!.outs.push(t.id); for (const s of t.outs) steps.find((x) => x.id === s)!.ins.push(t.id); };
  net.transitions.forEach((t, j) => {
    const ins = net.places.map((_, i) => i).filter((i) => net.pre[j][i]).map((i) => steps[i].id);
    const outs = net.places.map((_, i) => i).filter((i) => net.post[j][i]).map((i) => steps[i].id);
    const g = c.guards.get(j); const guard = g ? st(g) : 'TRUE';
    const op = c.ops.get(j); const timed = !op && (t.delay ?? 0) > 0;
    if (!op && !timed) { link({ id: nid(), label: t.id, cond: guard, ins, outs }); return; }
    const x: S = { id: nid(), name: `X_${ident(t.id)}`, initial: false, action: op?.start, ins: [], outs: [] }; steps.push(x);
    link({ id: nid(), label: `${t.id}_start`, cond: guard, ins, outs: [x.id] });
    link({ id: nid(), label: `${t.id}_end`, cond: op ? op.done : `${x.name}.T >= ${stTime(t.delay!)}`, ins: [x.id], outs });
  });
  // divergence / convergence elements
  const selDiv = new Map<number, number>(), selConv = new Map<number, number>(), simDiv = new Map<number, number>(), simConv = new Map<number, number>();
  for (const s of steps) { if (s.outs.length > 1) selDiv.set(s.id, nid()); if (s.ins.length > 1) selConv.set(s.id, nid()); }
  for (const t of trs) { if (t.outs.length > 1) simDiv.set(t.id, nid()); if (t.ins.length > 1) simConv.set(t.id, nid()); }
  const ref = (id: number, fp?: string) => `<connection refLocalId="${id}"${fp ? ` formalParameter="${fp}"` : ''}/>`;
  /** connection from step s towards transition t */
  const fromStep = (s: S, t: number) => (selDiv.has(s.id) ? ref(selDiv.get(s.id)!, `o${s.outs.indexOf(t)}`) : ref(s.id));
  /** connection from transition t towards step s */
  const fromTr = (t: T, s: number) => (simDiv.has(t.id) ? ref(simDiv.get(t.id)!, `o${t.outs.indexOf(s)}`) : ref(t.id));
  const stepOf = (id: number) => steps.find((x) => x.id === id)!; const trOf = (id: number) => trs.find((x) => x.id === id)!;
  let row = 0; const pos = () => `<position x="${40 + (row % 5) * 180}" y="${40 + 60 * Math.floor(row++ / 5)}"/>`;
  const out: string[] = [];
  for (const s of steps) {
    const inXml = selConv.has(s.id) ? `<connectionPointIn>${ref(selConv.get(s.id)!)}</connectionPointIn>` : s.ins.length ? `<connectionPointIn>${fromTr(trOf(s.ins[0]), s.id)}</connectionPointIn>` : '';
    out.push(`<step localId="${s.id}" name="${xmlEsc(s.name)}" initialStep="${s.initial}">${pos()}${inXml}<connectionPointOut formalParameter="sfc"/></step>`);
    if (selConv.has(s.id)) out.push(`<selectionConvergence localId="${selConv.get(s.id)}">${pos()}${s.ins.map((t) => `<connectionPointIn>${fromTr(trOf(t), s.id)}</connectionPointIn>`).join('')}<connectionPointOut/></selectionConvergence>`);
    if (selDiv.has(s.id)) out.push(`<selectionDivergence localId="${selDiv.get(s.id)}">${pos()}<connectionPointIn>${ref(s.id)}</connectionPointIn>${s.outs.map((_, k) => `<connectionPointOut formalParameter="o${k}"/>`).join('')}</selectionDivergence>`);
    if (s.action) out.push(`<actionBlock localId="${nid()}">${pos()}<connectionPointIn>${ref(s.id)}</connectionPointIn><action localId="${nid()}" qualifier="N"><reference name="${s.action}"/></action></actionBlock>`);
  }
  for (const t of trs) {
    const inXml = simConv.has(t.id) ? ref(simConv.get(t.id)!) : t.ins.length ? fromStep(stepOf(t.ins[0]), t.id) : '';
    if (simConv.has(t.id)) out.push(`<simultaneousConvergence localId="${simConv.get(t.id)}">${pos()}${t.ins.map((s) => `<connectionPointIn>${fromStep(stepOf(s), t.id)}</connectionPointIn>`).join('')}<connectionPointOut/></simultaneousConvergence>`);
    out.push(`<transition localId="${t.id}">${pos()}${inXml ? `<connectionPointIn>${inXml}</connectionPointIn>` : ''}<condition><inline name="${xmlEsc(t.label)}"><ST><xhtml:p>${xmlEsc(t.cond)}</xhtml:p></ST></inline></condition><connectionPointOut/></transition>`);
    if (simDiv.has(t.id)) out.push(`<simultaneousDivergence localId="${simDiv.get(t.id)}">${pos()}<connectionPointIn>${ref(t.id)}</connectionPointIn>${t.outs.map((_, k) => `<connectionPointOut formalParameter="o${k}"/>`).join('')}</simultaneousDivergence>`);
  }
  const vars = (list: Array<{ n: string; t: string; a?: string; doc?: string }>) => list.map((v) => `<variable name="${xmlEsc(v.n)}"${v.a ? ` address="${v.a}"` : ''}><type><${v.t}/></type>${v.doc ? `<documentation><xhtml:p>${xmlEsc(v.doc)}</xhtml:p></documentation>` : ''}</variable>`).join('');
  const ins = [...[...c.ops.values()].map((o) => ({ n: o.done, t: 'BOOL', a: c.io.find((p) => p.name === o.done)!.addr, doc: `${o.b.kind} ${o.b.value} finished` })), ...c.inputs.map((v) => ({ n: ident(v.name), t: v.type, a: c.io.find((p) => p.name === ident(v.name))!.addr }))];
  const outs = [...c.ops.values()].map((o) => ({ n: o.start, t: 'BOOL', a: c.io.find((p) => p.name === o.start)!.addr, doc: `start ${o.b.kind} ${o.b.value}${o.b.robot ? ` on ${o.b.robot}` : ''}` }));
  const now = new Date().toISOString().replace(/\.\d+Z$/, '');
  const xml = `<?xml version="1.0" encoding="UTF-8"?>
<project xmlns="http://www.plcopen.org/xml/tc6_0201" xmlns:xhtml="http://www.w3.org/1999/xhtml">
  <fileHeader companyName="VerticalBot Studio" productName="VerticalBot Studio" productVersion="1" creationDateTime="${now}"/>
  <contentHeader name="${xmlEsc(name)}" modificationDateTime="${now}"><coordinateInfo><fbd><scaling x="1" y="1"/></fbd><ld><scaling x="1" y="1"/></ld><sfc><scaling x="1" y="1"/></sfc></coordinateInfo></contentHeader>
  <types>
    <dataTypes/>
    <pous>
      <pou name="${prog}" pouType="program">
        <interface><inputVars>${vars(ins)}</inputVars><outputVars>${vars(outs)}</outputVars></interface>
        <body><SFC>
${out.map((l) => `          ${l}`).join('\n')}
        </SFC></body>
      </pou>
    </pous>
  </types>
  <instances><configurations><configuration name="Config0"><resource name="Res0"><task name="MainTask" priority="1" interval="T#20ms"><pouInstance name="${prog}_1" typeName="${prog}"/></task></resource></configuration></configurations></instances>
</project>
`;
  return { name: `${prog}.plcopen.xml`, content: xml, mime: 'application/xml' };
}
