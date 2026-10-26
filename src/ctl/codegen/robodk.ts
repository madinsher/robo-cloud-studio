/**
 * RoboDK station script generator: an automaton, Petri net, FSM or behavior tree becomes a Python program that drives
 * the RoboDK station (robolink) — or the studio's RoboDK-compatible API server — with the same semantics as the studio
 * executors (the embedded runtime is a port of them, see py_runtime.ts). The model is embedded as JSON with every
 * expression (guards, sensors, decisions, `=expr` values) translated to Python.
 */
import type { ControlKind } from '../model';
import { parseDes, parsePetri, parseBt, ActionBinding } from '../dsl';
import { parseFsm, FsmMachine } from '../fsm';
import { DES } from '../des';
import { PetriNet } from '../petri';
import type { BTNodeSpec } from '../bt';
import { synthesizeSupervisor } from '../supervisor';
import { PY_RUNTIME } from './py_runtime';
import { GenFile, translateExpr, PY_DIALECT, exprIds, ident } from './common';

const py = (e: string) => translateExpr(e, PY_DIALECT);
type JBinding = Omit<ActionBinding, 'target'> & { py?: string };
/** Binding for the Python runtime: `await` / `=expr` values pre-translated. */
function jb(b: ActionBinding | Omit<ActionBinding, 'target'>, sig: Set<string>): JBinding {
  const out: JBinding = { kind: b.kind, value: b.value, robot: b.robot, done: b.done, fail: b.fail, timeout: b.timeout, component: b.component, args: { ...b.args } };
  if (b.kind === 'await') { out.py = py(b.value); exprIds(b.value).forEach((i) => sig.add(i)); }
  if (b.kind === 'set' && typeof b.args.value === 'string' && b.args.value.startsWith('=')) { out.py = py(b.args.value.slice(1)); }
  if (b.kind === 'signal') sig.add(b.value);
  return out;
}

/** The model as data for the Python runtime (also used by the ROS 2 generator). */
export function runtimeModel(kind: ControlKind, source: string): { kind: 'des' | 'petri' | 'fsm' | 'bt'; model: Record<string, unknown>; signals: string[] } {
  const sig = new Set<string>(); const addIds = (e: string) => exprIds(e).forEach((i) => sig.add(i));
  if (kind === 'des') {
    const doc = parseDes(source);
    const comps = doc.plant.map((a) => { const d = DES.fromSpec(a); const delta: Record<string, Array<[string, string]>> = {}; for (const x of d.X) delta[x] = [...(d.f.get(x)?.entries() ?? [])]; return { name: d.name, x0: d.x0, marked: [...d.xm], sigma: [...d.sigma], delta }; });
    const actions: Record<string, JBinding> = {};
    for (const a of doc.actions) { const t = a.target.includes('.') ? a.target : (doc.plant.find((p) => p.transitions.some((tr) => tr.from === a.target || tr.to === a.target) || p.initial === a.target)?.name ?? '') + '.' + a.target; actions[t] = jb(a, sig); }
    const guards: Record<string, string> = {}; for (const [e, g] of Object.entries(doc.guards ?? {})) { guards[e] = py(g); addIds(g); }
    const sensors = (doc.sensors ?? []).map((s) => { addIds(s.expr); return [s.event, py(s.expr)]; });
    const sup = doc.specs.length ? synthesizeSupervisor(doc) : null;
    return { kind: 'des', model: { comps, uc: doc.uncontrollable, guards, sensors, actions, supervisor: sup?.table ?? null, bb: doc.blackboard ?? {}, terminate: doc.terminate ?? null }, signals: [...sig] };
  }
  if (kind === 'petri') {
    const doc = parsePetri(source); const net = new PetriNet(doc.spec);
    const places = net.places.map((p, i) => ({ id: p.id, tokens: p.tokens ?? 0, capacity: p.capacity, kind: p.kind, out: net.placeOut(i).length > 0 }));
    const w = (row: number[]) => Object.fromEntries(row.map((v, i) => [net.places[i].id, v]).filter(([, v]) => (v as number) > 0));
    const transitions = net.transitions.map((t, j) => { if (t.guard) addIds(t.guard); return { id: t.id, pre: w(net.pre[j]), post: w(net.post[j]), inh: w(net.inhibitor[j]), delay: t.delay, rate: t.rate, immediate: !!t.immediate, priority: t.priority ?? 0, servers: t.servers ?? 1, guard: t.guard ? py(t.guard) : undefined, on: t.on }; });
    const actions = Object.fromEntries(doc.actions.map((a) => [a.target, jb(a, sig)]));
    const sensors = (doc.sensors ?? []).map((s) => { addIds(s.expr); return [s.event, py(s.expr)]; });
    return { kind: 'petri', model: { places, transitions, actions, finals: doc.finals ?? [], sensors, bb: doc.blackboard ?? {} }, signals: [...sig] };
  }
  if (kind === 'fsm') {
    const doc = parseFsm(source);
    const mach = (m: FsmMachine): Record<string, unknown> => ({ name: m.name, outcomes: m.outcomes, states: m.states.map((s) => { s.when.forEach((w) => addIds(w.expr)); return { name: s.name, kind: s.kind, initial: !!s.initial, action: s.action ? jb(s.action, sig) : null, entry: s.entry.map((b) => jb(b, sig)), exit: s.exit.map((b) => jb(b, sig)), timeout: s.timeout ?? null, when: s.when.map((w) => [py(w.expr), w.outcome]), otherwise: s.otherwise ?? null, transitions: s.transitions, machine: s.machine ? mach(s.machine) : null }; }) });
    const root = mach(doc.root); (root as { name: string }).name = ident(doc.root.name || 'FSM');
    const sensors = doc.sensors.map((s) => { addIds(s.expr); return [s.event, py(s.expr)]; });
    return { kind: 'fsm', model: { root, bb: doc.blackboard, sensors }, signals: [...sig] };
  }
  if (kind === 'bt') {
    const doc = parseBt(source);
    const conv = (n: BTNodeSpec): Record<string, unknown> => {
      const out: Record<string, unknown> = { type: n.type, name: n.name, memory: !!n.memory, threshold: n.threshold, count: n.count, seconds: n.seconds, timeout: n.timeout, args: n.args ?? {}, fn: n.fn, children: (n.children ?? []).map(conv) };
      if (n.type === 'condition' && n.fn) { out.py = py(n.fn); addIds(n.fn); }
      if (n.pre) { out.pre_py = py(n.pre); addIds(n.pre); } if (n.post) { out.post_py = py(n.post); addIds(n.post); }
      if (n.type === 'action') out.binding = btBinding(n, sig);
      return out;
    };
    const supSrc = doc.supervisor ? null : null; void supSrc;
    return { kind: 'bt', model: { root: conv(doc.root), loop: JSON.stringify(doc.root).includes('"report"'), supervisor: null, bb: {} }, signals: [...sig] };
  }
  throw new Error(`no station script for ${kind} models (des, petri, fsm, bt)`);
}

/** BT action → runtime binding (mobile skills become programs of the same name in RoboDK). */
function btBinding(n: BTNodeSpec, sig: Set<string>): JBinding | null {
  const a = (n.args ?? {}) as Record<string, string | number | boolean>; const fn = n.fn ?? '';
  const mk = (kind: ActionBinding['kind'] | 'check', value: string, extra: Partial<JBinding> = {}): JBinding => ({ kind: kind as ActionBinding['kind'], value, args: { ...a }, ...extra });
  switch (fn) {
    case 'program': return mk('program', String(a.program ?? a.name ?? ''));
    case 'move': return mk('target', String(a.target ?? ''), { robot: a.robot !== undefined ? String(a.robot) : undefined });
    case 'signal': sig.add(String(a.signal ?? a.name)); return mk('signal', String(a.signal ?? a.name ?? ''));
    case 'wait': return mk('wait', String(a.seconds ?? 1));
    case 'event': return mk('event', String(a.emit ?? a.name ?? ''));
    case 'await': { const e = String(a.expr ?? a.await ?? 'true'); exprIds(e).forEach((i) => sig.add(i)); return mk('await', e, { py: py(e) }); }
    case 'set': { const [k, v] = Object.entries(a).find(([key]) => !['event', 'done', 'robot'].includes(key)) ?? ['flag', true]; const isExpr = typeof v === 'string' && v.startsWith('='); return { kind: 'set', value: k, args: { value: v }, ...(isExpr ? { py: py(String(v).slice(1)) } : {}) }; }
    case 'report': return { kind: 'set', value: 'reported', args: { value: true } };
    case 'select_target': return mk('check', 'targets > 0', { py: py('targets > 0') });
    case 'goto': return mk('goto', String(a.zone ?? a.target ?? ''));
    case 'log': return mk('wait', '0');
    default: return mk('skill', fn);
  }
}

/** The RoboDK station script (Python, robolink) of a control model. */
export function robodkStationScript(kind: ControlKind, source: string, name: string, opts: { supervisorSource?: string } = {}): GenFile {
  const m = runtimeModel(kind, source);
  if (kind === 'bt' && opts.supervisorSource) m.model.supervisor = synthesizeSupervisor(opts.supervisorSource)?.table ?? null;
  const header = `# -*- coding: utf-8 -*-
"""Control program "${name.replace(/"/g, "'")}" (${kind}) generated by VerticalBot Studio.

Run it from RoboDK (Tools > Run Script, or as a station Python program) or from any Python with the robodk package
against RoboDK or the studio API server (npm run server). Programs of different robots run concurrently (RunProgram
is non-blocking); a robot executes one action at a time; signals are station parameters (RDK.setParam / getParam);
mobile skills / goto call RoboDK programs named after them (goto_<zone>, dock, ...).
Usage: python script.py [seconds] [SPEED] - the run sets RoboDK's simulation speed to SPEED (default 1 = real time,
as the studio runs the model; 0 keeps RoboDK's own speed) and restores it at the end; delays, waits and timeouts run
on the station's simulation clock (RDK.SimulationTime()), so their race with the program durations does not depend on
the speed. Guards / conditions / decisions read the signals again after a program ends.
"""`;
  const body = `${header}\n${PY_RUNTIME}\nMODEL = json.loads(r'''${JSON.stringify({ kind: m.kind, model: m.model, signals: m.signals })}''')\n\nif __name__ == '__main__':\n    main(MODEL)\n`;
  return { name: `${ident(name).toLowerCase()}_robodk.py`, content: body, mime: 'text/x-python' };
}
