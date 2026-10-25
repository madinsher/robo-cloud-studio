/**
 * Per-robot programs with IO handshakes, generated from a control model (automaton, Petri net, FSM, behavior tree):
 * the controller (PLC program from plc.ts, the RoboDK station script, or the studio executor in handshake mode) raises
 * START_x, the robot runs operation x and answers DONE_x; START_x falls, DONE_x falls (four-phase handshake).
 *
 *  - `controlOperations` lists the operations in the order of the PLC I/O map (op k ↔ PLC %QXk / %IXk ↔ robot DI/DO
 *    ioBase + k), with the robot that executes each one (program's robot, target's robot);
 *  - `buildHandshakePrograms` writes studio Programs: per robot a server `<P>R<r>` (short names for every controller; reset the DONE outputs, endless
 *    loop over a poll program: IF START_k → operation wrapper; a robot with a single operation blocks on WaitDI instead
 *    of polling), one wrapper per operation (call the bound program / move to the bound target, DONE handshake) and a
 *    robot-less `Main <model>` that starts every server as a thread (RoboDK "Start thread"). They run on the station
 *    runtime (tests drive them with the executor in handshake mode) and post like any program;
 *  - `vendorPrograms` posts the servers with the vendor post of each robot (or a chosen one) through compileForPost (I6:
 *    loop / if / wait DI / DO in the vendor language), with numeric IO, plus the I/O map.
 */
import { Station, ItemType, Target, Item } from '../../core/items/item';
import { Program } from '../../core/items/program';
import { Robot } from '../../core/items/robot';
import type { ControlKind } from '../model';
import { parseDes, parsePetri, parseBt, ActionBinding } from '../dsl';
import { parseFsm, fsmMachines } from '../fsm';
import { DES } from '../des';
import { btWalk } from '../graphdoc';
import { robotForTarget } from '../host';
import { compileForPost, getPost, PostFile } from '../../posts';
import { GenFile, ident } from './common';

export interface ControlOp { k: number; id: string; binding: ActionBinding; start: string; done: string }
const isOp = (b?: ActionBinding) => !!b && ['program', 'target', 'goto', 'skill'].includes(b.kind);

/** The operations of a model (same order and names as the PLC I/O map). */
export function controlOperations(kind: ControlKind, source: string): ControlOp[] {
  const out: ControlOp[] = [];
  const push = (id: string, b: ActionBinding) => { const i = ident(id); out.push({ k: out.length, id: i, binding: b, start: `START_${i}`, done: `DONE_${i}` }); };
  if (kind === 'des') {
    const doc = parseDes(source);
    for (const a of doc.plant) { const c = DES.fromSpec(a); for (const s of c.X) { const b = doc.actions.find((x) => x.target === `${c.name}.${s}`) ?? doc.actions.find((x) => !x.target.includes('.') && x.target === s && c.X.has(s)); if (isOp(b)) push(`${c.name}_${s}`, b!); } }
  } else if (kind === 'petri') {
    const doc = parsePetri(source);
    for (const t of doc.spec.transitions) { const b = doc.actions.find((a) => a.target === t.id); if (isOp(b)) push(t.id, b!); }
  } else if (kind === 'fsm') {
    const doc = parseFsm(source);
    for (const { path, machine } of fsmMachines(doc)) for (const s of machine.states) { if (isOp(s.action)) push(`${path.split('/').slice(1).concat(s.name).join('_')}`, s.action!); }
  } else if (kind === 'bt') {
    const doc = parseBt(source); let n = 0;
    for (const { node } of btWalk(doc.root)) {
      if (node.type !== 'action') continue; n++; const a = (node.args ?? {}) as Record<string, string | number | boolean>;
      if (node.fn === 'program') push(`${node.name && node.name !== node.fn ? node.name : `program_${n}`}`, { target: '', kind: 'program', value: String(a.program ?? a.name ?? ''), robot: a.robot !== undefined ? String(a.robot) : undefined, args: {} });
      else if (node.fn === 'move') push(`${node.name && node.name !== node.fn ? node.name : `move_${n}`}`, { target: '', kind: 'target', value: String(a.target ?? ''), robot: a.robot !== undefined ? String(a.robot) : undefined, args: { ...(a.linear !== undefined ? { linear: a.linear } : {}) } });
    }
  } else throw new Error(`no robot programs for ${kind} models (des, petri, fsm, bt)`);
  // the same operation bound twice (two states running the same program) keeps one id per binding
  const seen = new Map<string, number>(); for (const o of out) { const c = seen.get(o.id) ?? 0; seen.set(o.id, c + 1); if (c) { o.id = `${o.id}_${c + 1}`; o.start = `START_${o.id}`; o.done = `DONE_${o.id}`; } }
  return out;
}

const byName = <T extends Item>(st: Station, type: ItemType, name: string): T | undefined => st.itemsOfType<T>(type).find((i) => i.name === name) ?? st.itemsOfType<T>(type).find((i) => i.name.toLowerCase() === name.toLowerCase());
/** The robot executing an operation and what it runs (null robot: not a robot operation — mobile goto / skill, or unresolved). */
export function opExecutor(st: Station, o: ControlOp): { robot: Robot | null; program?: Program; target?: Target; why?: string } {
  const b = o.binding;
  if (b.kind === 'program') { const p = byName<Program>(st, ItemType.PROGRAM, b.value); if (!p) return { robot: null, why: `program ${b.value} not found` }; const r = p.robot(); const robot = r instanceof Robot ? r : (b.robot ? byName<Robot>(st, ItemType.ROBOT, b.robot) ?? null : null); return robot ? { robot, program: p } : { robot: null, program: p, why: `program ${p.name} has no robot` }; }
  if (b.kind === 'target') { const t = byName<Target>(st, ItemType.TARGET, b.value); if (!t) return { robot: null, why: `target ${b.value} not found` }; const { robot, why } = robotForTarget(st, t, b.robot); return robot ? { robot, target: t } : { robot: null, why }; }
  return { robot: null, why: `${b.kind} ${b.value} is executed by the mobile robot / cell controller, not by an arm program` };
}

export interface HandshakePrograms { main: Program; servers: Array<{ robot: Robot; program: Program; ops: ControlOp[] }>; ops: ControlOp[]; created: Program[]; skipped: string[]; io: (o: ControlOp, dir: 'start' | 'done') => string }

/**
 * Write the handshake programs into the station. `numericIo`: robot DI / DO numbers (ioBase + k) instead of the names
 * START_x / DONE_x (vendor export; in the studio the names keep the PLC / executor signal table readable).
 */
export function buildHandshakePrograms(st: Station, kind: ControlKind, source: string, name: string, opts: { numericIo?: boolean; ioBase?: number; pollMs?: number } = {}): HandshakePrograms {
  const ops = controlOperations(kind, source); const base = opts.ioBase ?? 1; const M = ident(name);
  const io = (o: ControlOp, dir: 'start' | 'done') => (opts.numericIo ? String(base + o.k) : dir === 'start' ? o.start : o.done);
  const created: Program[] = []; const skipped: string[] = [];
  const mk = (n: string, robot: Robot | null) => { const p = st.addChild(new Program(n)); if (robot) p.setRobot(robot); created.push(p); return p; };
  const lanes = new Map<Robot, Array<{ o: ControlOp; program?: Program; target?: Target }>>();
  for (const o of ops) { const e = opExecutor(st, o); if (!e.robot) { skipped.push(`${o.id}: ${e.why}`); continue; } let l = lanes.get(e.robot); if (!l) { l = []; lanes.set(e.robot, l); } l.push({ o, program: e.program, target: e.target }); }
  const servers: HandshakePrograms['servers'] = [];
  // short names (≤ 6 characters up to 9 robots / operations: Fanuc RJ3 limit): <P>R<r> server, <P>R<r>W wait / <P>R<r>P poll, <P>R<r>O<k> operation
  const P = (M.replace(/[^A-Za-z0-9]/g, '').slice(0, 2) || 'CT').toUpperCase(); let rn = 0;
  for (const [robot, list] of lanes) {
    const R = `${P}R${++rn}`;
    const wrappers = list.map(({ o, program, target }) => {
      const w = mk(`${R}O${o.k + 1}`, robot);
      w.comment(`operation ${o.k}: ${o.binding.kind} ${o.binding.value}: started by ${o.start}, answered by ${o.done}`);
      if (program) w.callProgram(program); else if (target) (o.binding.args.linear === true || o.binding.args.linear === 'true' ? w.addMoveL(target) : w.addMoveJ(target));
      w.setDO(io(o, 'done'), 1); w.waitDI(io(o, 'start'), 0); w.setDO(io(o, 'done'), 0);
      return { o, w };
    });
    const server = mk(R, robot);
    server.comment(`${robot.name}: operation server of the control model "${name}" (IO handshake START_x -> DONE_x)`);
    for (const { o } of wrappers) server.setDO(io(o, 'done'), 0);
    if (wrappers.length === 1) {
      // one operation: block on its START input instead of polling
      const body = mk(`${R}W`, robot); const { o, w } = wrappers[0];
      body.waitDI(io(o, 'start'), 1); body.callProgram(w);
      server.addInstruction({ kind: 'loop', count: -1, bodyProgramId: body.id }); // -1: endless (runtime and posts)
    } else {
      const poll = mk(`${R}P`, robot);
      for (const { o, w } of wrappers) poll.addInstruction({ kind: 'if', condition: `${io(o, 'start')} == 1`, thenProgramId: w.id, elseProgramId: null });
      poll.pause(opts.pollMs ?? 50);
      server.addInstruction({ kind: 'loop', count: -1, bodyProgramId: poll.id });
    }
    servers.push({ robot, program: server, ops: list.map((x) => x.o) });
  }
  const main = mk(`Main ${name}`, null);
  main.comment(`starts the operation server of every robot (${servers.map((s) => s.robot.name).join(', ') || 'none'}); the controller drives START_x / DONE_x`);
  for (const s of servers) main.startThread(s.program);
  return { main, servers, ops, created, skipped, io };
}

/** Remove the programs written by buildHandshakePrograms. */
export function removeHandshakePrograms(h: HandshakePrograms): void { for (const p of h.created) p.delete(); }

/**
 * Robot programs in vendor languages: every robot's server posted with `post` (default: the robot's own post — Robot.postProcessor —,
 * else RoboDK Python), numeric IO DI/DO ioBase + k. The station is left as it was.
 */
export function vendorPrograms(st: Station, kind: ControlKind, source: string, name: string, opts: { post?: string; ioBase?: number } = {}): GenFile[] {
  const h = buildHandshakePrograms(st, kind, source, name, { numericIo: true, ioBase: opts.ioBase });
  const files: GenFile[] = [];
  try {
    const rows = ['operation,robot DI (START),robot DO (DONE),PLC output,PLC input,robot,binding'];
    for (const o of h.ops) { const e = opExecutor(st, o); rows.push([o.id, String((opts.ioBase ?? 1) + o.k), String((opts.ioBase ?? 1) + o.k), o.start, o.done, e.robot?.name ?? '(not a robot operation)', `${o.binding.kind} ${o.binding.value}`].map((c) => (/[,"]/.test(c) ? `"${c.replace(/"/g, '""')}"` : c)).join(',')); }
    files.push({ name: `${ident(name)}_robot_io.csv`, content: rows.join('\n') + '\n', mime: 'text/csv' });
    for (const s of h.servers) {
      const post = getPost(opts.post ?? s.robot.postProcessor) ?? getPost('RoboDK_Python')!;
      const pp = compileForPost(st, s.program);
      const out: PostFile[] = post.generate(pp, {});
      for (const f of out) files.push({ name: `${ident(s.robot.name)}/${f.name}`, content: f.content, mime: f.mime ?? (f.encoding === 'base64' ? 'application/octet-stream' : undefined), ...(f.encoding === 'base64' ? { encoding: 'base64' as const } : {}) });
      if (pp.problems.length) files.push({ name: `${ident(s.robot.name)}/PROBLEMS.txt`, content: pp.problems.join('\n') + '\n' });
    }
    if (h.skipped.length) files.push({ name: `${ident(name)}_not_robot_ops.txt`, content: `Operations executed by the cell controller / mobile robot, not by an arm program:\n${h.skipped.join('\n')}\n` });
  } finally { removeHandshakePrograms(h); }
  return files;
}
