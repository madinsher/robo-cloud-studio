/**
 * Text DSL for control-design documents. One document = one model kind; the first keyword names the kind.
 * Lines starting with `#` or `//` are comments. `key=value` options accept numbers, booleans and quoted strings.
 *
 *   des        automaton / spec blocks, transitions `from -event-> to`, uncontrollable / unobservable / faults, checks
 *   petri      place / transition / arc lines (arcs `a, b -> t -> c`; inhibitor `p -o t`)
 *   s3pr       resource R capacity; process P jobs=n: step[R1,R2] duration; …
 *   smv        var / define / init / next … case … esac / fairness / ltlspec / ctlspec
 *   bt         indented tree: fallback | sequence | parallel N | retry N | repeat N | timeout S | inverter | action fn k=v | condition expr
 *   statechart state lines with parent= region= initial history; transitions `A -event [guard]-> B`
 *   gr1        env / sys variable declarations, env_init env_trans env_live sys_init sys_trans sys_live lines
 *   hybrid     var, initial, mode lines, `A -> B when guard dwell=s`
 *   pddl       raw PDDL domain + problem, optional `method` / `task` lines for HTN
 *   stn        `a -> b [min,max]`, contingent `a => b [l,u]`
 *   mdp        header options, `terminal s=value`, `s action cost : s1 p1, s2 p2`
 *   jobshop    machines, `job J type=A due=60 : M1 5 ; S1 22 | S2 26 ; M2 5`, setup lines
 *   realtime   `task id C= T= D= cs=res:len`, `chain id:T/R … mode=async`
 *   fta        indented gates `and|or id "name"` and `event id p=… "name"`
 *   reliability `component id lambda= redundancy=`, `mttr h`, `ssm v= tr= ts= …`, `pl S F P cat mttfd dc`
 *   perf       `resource R time= capacity=`, `op id duration= resources=a,b after=x`, `little wip= throughput=`
 *   pomdp / mrta / mapf / fmea / acceptance / stl: header line followed by JSON (stl: `spec` lines + optional `signal` table)
 */
import { AutomatonSpec, Transition } from './des';
import { PetriNetSpec, S3PRSpec, PlaceKind } from './petri';
import { SmvSpec, SmvCase } from './smv';
import { BTNodeSpec, BTNodeType, StatechartSpec, BTStatus } from './bt';
import { GR1Spec } from './gr1';
import { HybridSpec } from './hybrid';
import { STNConstraint, HTNDomain, Cond, parseSExpr, SExpr } from './planning';
import { MDPSpec } from './mdp';
import { JobShop, Job } from './sched';
import { RTTask } from './realtime';
import { FTNode, Component } from './reliability';
import { ResourceLoad, CycleOperation } from './perf';
import { ControlKind, CtlKind } from './model';
import { MRS_TEMPLATES, detectMrsKind } from '../mrs/dsl';

import { Opts, tokenize, lines, DslError } from './dsl_core';
export { tokenize, lines, DslError } from './dsl_core';
export type { Opts } from './dsl_core';

/** Detect the document kind from its first keyword. */
export function detectKind(src: string): ControlKind | null {
  const first = lines(src)[0]?.text.trim().split(/\s+/)[0]?.toLowerCase();
  const map: Record<string, ControlKind> = { automaton: 'des', spec: 'des', des: 'des', plant: 'des', petri: 'petri', s3pr: 's3pr', smv: 'smv', module: 'smv', bt: 'bt', behavior: 'bt', statechart: 'statechart', gr1: 'gr1', hybrid: 'hybrid', modes: 'hybrid', pddl: 'pddl', '(define': 'pddl', stn: 'stn', stnu: 'stn', mdp: 'mdp', pomdp: 'pomdp', jobshop: 'jobshop', realtime: 'realtime', fta: 'fta', reliability: 'reliability', mrta: 'mrta', mapf: 'mapf', stl: 'stl', fmea: 'fmea', acceptance: 'acceptance', perf: 'perf' };
  if (!first) return null;
  if (first.startsWith('(define')) return 'pddl';
  return map[first] ?? detectMrsKind(src);
}

// ---------------------------------------------------------------------------------------------
// DES
// ---------------------------------------------------------------------------------------------

export interface DesDoc { name: string; plant: AutomatonSpec[]; specs: AutomatonSpec[]; uncontrollable: string[]; unobservable: string[]; faults: string[]; checks: Array<{ kind: 'ltl' | 'ctl'; formula: string; name?: string }>; supervisorName?: string; /** Diagram positions per automaton (graphical editor). */ layout?: Record<string, Record<string, [number, number]>>; /** Robot actions bound to states (`Automaton.state`). */ actions: ActionBinding[] }

const TRANS_RE = /^(\S+)\s*-+\s*(.+?)\s*-+>\s*(\S+)$/;

/** A robot / cell action bound to an automaton state (`Automaton.state`) or a Petri transition (see ctl/exec.ts). */
export interface ActionBinding { target: string; kind: 'program' | 'target' | 'goto' | 'signal' | 'wait' | 'event' | 'set'; value: string; robot?: string; done?: string; component?: string; args: Record<string, string | number | boolean> }
export const ACTION_KINDS: ActionBinding['kind'][] = ['program', 'target', 'goto', 'signal', 'wait', 'event', 'set'];
function parseAction(words: string[], opts: Opts, n: number): ActionBinding {
  const target = words[1]; if (!target) throw new DslError('action <state|transition> program=|target=|goto=|signal=|wait=|event=|set= …', n);
  const kind = ACTION_KINDS.find((k) => opts[k] !== undefined); if (!kind) throw new DslError(`action ${target}: one of ${ACTION_KINDS.map((k) => `${k}=`).join(' ')} is required`, n);
  const args: ActionBinding['args'] = {}; for (const [k, v] of Object.entries(opts)) if (!['robot', 'done', 'component', kind].includes(k)) args[k] = v;
  return { target, kind, value: String(opts[kind]), robot: opts.robot !== undefined ? String(opts.robot) : undefined, done: opts.done !== undefined ? String(opts.done) : undefined, component: opts.component !== undefined ? String(opts.component) : undefined, args };
}

/** `key=x,y` options → positions. */
function layoutOf(opts: Opts): Record<string, [number, number]> {
  const L: Record<string, [number, number]> = {};
  for (const [k, v] of Object.entries(opts)) { const m = /^(-?[\d.]+),(-?[\d.]+)$/.exec(String(v)); if (m) L[k] = [Number(m[1]), Number(m[2])]; }
  return L;
}

export function parseDes(src: string): DesDoc {
  const doc: DesDoc = { name: 'DES', plant: [], specs: [], uncontrollable: [], unobservable: [], faults: [], checks: [], actions: [] };
  let cur: AutomatonSpec | null = null; let list: AutomatonSpec[] = doc.plant;
  for (const l of lines(src)) {
    const t = l.text.trim(); const { words, quoted, opts } = tokenize(t); const kw = words[0]?.toLowerCase();
    if (kw === 'des' || kw === 'plant') { doc.name = words.slice(1).join(' ') || doc.name; continue; }
    if (kw === 'automaton' || kw === 'spec' || kw === 'supervisor') { cur = { name: words[1] ?? `${kw}${list.length + 1}`, events: [], initial: '', marked: [], transitions: [], states: [] }; list = kw === 'automaton' ? doc.plant : doc.specs; list.push(cur); continue; }
    if (kw === 'uncontrollable') { doc.uncontrollable.push(...words.slice(1)); continue; }
    if (kw === 'unobservable') { doc.unobservable.push(...words.slice(1)); continue; }
    if (kw === 'faults' || kw === 'fault') { doc.faults.push(...words.slice(1)); continue; }
    if (kw === 'check') { const kind = words[1]?.toLowerCase() as 'ltl' | 'ctl'; const name = quoted[0]; const rest = t.replace(/^check\s+\w+\s*/i, '').replace(name ? `"${name}"` : '', '').trim(); if (kind !== 'ltl' && kind !== 'ctl') throw new DslError('check ltl|ctl <formula>', l.n); doc.checks.push({ kind, formula: rest, name }); continue; }
    if (kw === 'layout') { const auto = words[1]; if (!auto) throw new DslError('layout <automaton> state=x,y …', l.n); (doc.layout ??= {})[auto] = { ...(doc.layout?.[auto] ?? {}), ...layoutOf(opts) }; continue; }
    if (kw === 'action') { doc.actions.push(parseAction(words, opts, l.n)); continue; }
    if (!cur) throw new DslError(`expected 'automaton <name>' before '${t}'`, l.n);
    if (kw === 'events') { cur.events.push(...words.slice(1)); continue; }
    if (kw === 'initial') { cur.initial = words[1]; continue; }
    if (kw === 'marked') { cur.marked.push(...words.slice(1)); continue; }
    if (kw === 'states') { cur.states!.push(...words.slice(1)); continue; }
    const m = TRANS_RE.exec(t);
    if (m) { for (const ev of m[2].split(/[,\s]+/).filter(Boolean)) { cur.transitions.push({ from: m[1], event: ev, to: m[3] }); if (!cur.events.includes(ev)) cur.events.push(ev); } if (!cur.initial) cur.initial = m[1]; continue; }
    throw new DslError(`cannot parse '${t}'`, l.n);
  }
  for (const a of [...doc.plant, ...doc.specs]) { if (!a.initial) a.initial = a.transitions[0]?.from ?? a.states?.[0] ?? 'q0'; if (!a.marked.length) a.marked = [a.initial]; }
  return doc;
}

export { automatonToDsl } from './graphdoc';

// ---------------------------------------------------------------------------------------------
// Petri nets and S³PR
// ---------------------------------------------------------------------------------------------

export function parsePetri(src: string): { spec: PetriNetSpec; checks: Array<{ kind: 'ltl' | 'ctl'; formula: string; name?: string }>; horizon: number; layout: Record<string, [number, number]>; actions: ActionBinding[] } {
  const spec: PetriNetSpec = { name: 'Petri net', places: [], transitions: [], arcs: [] }; const checks: DesDoc['checks'] = []; let horizon = 300; let layout: Record<string, [number, number]> = {}; const actions: ActionBinding[] = [];
  for (const l of lines(src)) {
    const t = l.text.trim(); const { words, opts, quoted } = tokenize(t); const kw = words[0]?.toLowerCase();
    if (kw === 'petri') { spec.name = words.slice(1).filter((w) => !w.includes('=')).join(' ') || spec.name; if (typeof opts.horizon === 'number') horizon = opts.horizon; continue; }
    if (kw === 'place') { spec.places.push({ id: words[1], tokens: Number(opts.tokens ?? opts.m0 ?? 0), kind: (opts.kind as PlaceKind) ?? undefined, capacity: opts.capacity as number | undefined, label: quoted[0] }); continue; }
    if (kw === 'transition' || kw === 'trans') { spec.transitions.push({ id: words[1], delay: opts.delay as number | undefined, rate: opts.rate as number | undefined, immediate: words.includes('immediate') || opts.immediate === true, weight: opts.weight as number | undefined, priority: opts.priority as number | undefined, label: quoted[0] }); continue; }
    if (kw === 'arc') {
      const body = t.replace(/^arc\s+/i, '').replace(/\s+weight=\S+/, '');
      const inhibitor = /-o\s/.test(body) || / -o$/.test(body);
      const parts = body.split(/\s*(?:->|-o)\s*/).map((p) => p.split(/\s*,\s*/).map((x) => x.trim()).filter(Boolean));
      if (parts.length < 2) throw new DslError('arc needs "a -> b"', l.n);
      for (let i = 0; i + 1 < parts.length; i++) for (const a of parts[i]) for (const b of parts[i + 1]) spec.arcs.push({ from: a, to: b, weight: opts.weight as number | undefined, inhibitor: inhibitor && i === 0 ? true : undefined });
      continue;
    }
    if (kw === 'check') { const kind = words[1]?.toLowerCase() as 'ltl' | 'ctl'; const name = quoted[0]; checks.push({ kind, formula: t.replace(/^check\s+\w+\s*/i, '').replace(name ? `"${name}"` : '', '').trim(), name }); continue; }
    if (kw === 'layout') { layout = { ...layout, ...layoutOf(opts) }; continue; }
    if (kw === 'action') { actions.push(parseAction(words, opts, l.n)); continue; }
    throw new DslError(`cannot parse '${t}'`, l.n);
  }
  return { spec, checks, horizon, layout, actions };
}

export function parseS3PR(src: string): S3PRSpec {
  const spec: S3PRSpec = { name: 'S3PR', resources: {}, processes: [] };
  for (const l of lines(src)) {
    const t = l.text.trim(); const { words, opts } = tokenize(t); const kw = words[0]?.toLowerCase();
    if (kw === 's3pr') { spec.name = words.slice(1).join(' ') || spec.name; continue; }
    if (kw === 'resource') { spec.resources[words[1]] = Number(words[2] ?? opts.capacity ?? 1); continue; }
    if (kw === 'process') {
      const m = /^process\s+(\S+)(.*?):\s*(.+)$/i.exec(t); if (!m) throw new DslError('process P jobs=n: step[R] d; …', l.n);
      const steps = m[3].split(';').map((s) => s.trim()).filter(Boolean).map((s) => { const sm = /^(\S+?)\[([^\]]*)\]\s*(\d+(?:\.\d+)?)?$/.exec(s); if (!sm) throw new DslError(`bad step '${s}'`, l.n); return { id: sm[1], resources: sm[2].split(',').map((x) => x.trim()).filter(Boolean), duration: sm[3] ? Number(sm[3]) : 1 }; });
      const jobs = /jobs\s*=\s*(\d+)/.exec(m[2]); spec.processes.push({ id: m[1], jobs: jobs ? Number(jobs[1]) : 1, steps }); continue;
    }
    throw new DslError(`cannot parse '${t}'`, l.n);
  }
  return spec;
}

// ---------------------------------------------------------------------------------------------
// SMV
// ---------------------------------------------------------------------------------------------

export function parseSmv(src: string): SmvSpec {
  const spec: SmvSpec = { name: 'model', vars: [], defines: {}, init: {}, next: {}, fairness: [], specs: [] };
  const ls = lines(src); let i = 0;
  const dom = (s: string) => s.replace(/[{}]/g, '').split(',').map((x) => x.trim()).filter(Boolean).map((x) => (x === 'boolean' ? ['false', 'true'] : x === 'true' ? true : x === 'false' ? false : /^-?\d+$/.test(x) ? Number(x) : x)).flat();
  while (i < ls.length) {
    const t = ls[i].text.trim(); const n = ls[i].n; i++;
    const kw = t.split(/\s+/)[0].toLowerCase();
    if (kw === 'smv' || kw === 'module') { spec.name = t.split(/\s+/).slice(1).join(' ') || spec.name; continue; }
    let m: RegExpExecArray | null;
    if ((m = /^var\s+(\S+)\s*:\s*(.+?);?$/i.exec(t))) { const d = m[2].trim(); spec.vars.push({ name: m[1], domain: d === 'boolean' || d === 'bool' ? [false, true] : /^\d+\.\.\d+$/.test(d) ? Array.from({ length: Number(d.split('..')[1]) - Number(d.split('..')[0]) + 1 }, (_, k) => Number(d.split('..')[0]) + k) : dom(d) }); continue; }
    if ((m = /^define\s+(\S+)\s*:=\s*(.+?);?$/i.exec(t))) { spec.defines![m[1]] = m[2].trim(); continue; }
    if ((m = /^init\s+(\S+)\s*:=\s*(.+?);?$/i.exec(t))) { spec.init![m[1]] = m[2].trim(); continue; }
    if ((m = /^next\s+(\S+)\s*:=\s*(.*)$/i.exec(t))) {
      const v = m[1]; const rest = m[2].trim();
      if (/^case\b/i.test(rest)) {
        const cases: SmvCase[] = []; let inl = rest.replace(/^case\s*/i, '');
        const addCases = (text: string) => { for (const part of text.split(';')) { const c = part.trim(); if (!c || /^esac/i.test(c)) continue; const idx = c.indexOf(':'); if (idx < 0) throw new DslError(`case needs 'cond : value' (${c})`, n); cases.push({ cond: c.slice(0, idx).trim(), value: c.slice(idx + 1).trim() }); } };
        if (inl) addCases(inl);
        while (i < ls.length && !/^esac/i.test(ls[i].text.trim())) { addCases(ls[i].text.trim()); i++; }
        if (i < ls.length && /^esac/i.test(ls[i].text.trim())) { const tail = ls[i].text.trim().replace(/^esac\s*;?/i, '').trim(); i++; if (tail) addCases(tail); }
        spec.next![v] = cases;
      } else spec.next![v] = [{ cond: 'TRUE', value: rest.replace(/;$/, '') }];
      continue;
    }
    if ((m = /^fairness\s+(.+?);?$/i.exec(t))) { spec.fairness!.push(m[1]); continue; }
    if ((m = /^(ltlspec|ctlspec)\s+(?:"([^"]*)"\s+)?(.+?);?$/i.exec(t))) { spec.specs!.push({ kind: m[1].toLowerCase() === 'ltlspec' ? 'LTL' : 'CTL', name: m[2], formula: m[3] }); continue; }
    throw new DslError(`cannot parse '${t}'`, n);
  }
  return spec;
}

// ---------------------------------------------------------------------------------------------
// Behavior trees and statecharts
// ---------------------------------------------------------------------------------------------

const BT_TYPES: Record<string, BTNodeType> = { fallback: 'fallback', selector: 'fallback', '?': 'fallback', sequence: 'sequence', seq: 'sequence', '->': 'sequence', parallel: 'parallel', inverter: 'inverter', not: 'inverter', retry: 'retry', repeat: 'repeat', timeout: 'timeout', force_success: 'force_success', force_failure: 'force_failure', action: 'action', act: 'action', condition: 'condition', cond: 'condition' };

export interface BtDoc { name: string; root: BTNodeSpec; outcomes: Record<string, BTStatus[]>; checks: DesDoc['checks']; leafModels: Record<string, { p?: number; ticks?: number }>; supervisor?: string; monitors: string[]; modes?: string }

export function parseBt(src: string): BtDoc {
  const doc: BtDoc = { name: 'Behavior tree', root: { type: 'sequence', name: 'root', children: [] }, outcomes: {}, checks: [], leafModels: {}, monitors: [] };
  const stack: Array<{ indent: number; node: BTNodeSpec }> = []; let rootSet = false;
  for (const l of lines(src)) {
    const t = l.text.trim(); const { words, opts, quoted } = tokenize(t); const kw = words[0]?.toLowerCase();
    if (kw === 'bt' || kw === 'behavior') { doc.name = words.slice(1).join(' ') || doc.name; continue; }
    if (kw === 'outcomes') { for (const [leaf, vals] of Object.entries(opts)) doc.outcomes[leaf] = String(vals).split(',') as BTStatus[]; continue; }
    if (kw === 'model') { const leaf = words[1]; doc.leafModels[leaf] = { p: opts.p as number | undefined, ticks: opts.ticks as number | undefined }; continue; }
    if (kw === 'supervisor') { doc.supervisor = words[1]; continue; }
    if (kw === 'modes') { doc.modes = words[1]; continue; }
    if (kw === 'monitor') { doc.monitors.push(t.replace(/^monitor\s+/i, '')); continue; }
    if (kw === 'check') { const kind = words[1]?.toLowerCase() as 'ltl' | 'ctl'; const name = quoted[0]; doc.checks.push({ kind, formula: t.replace(/^check\s+\w+\s*/i, '').replace(name ? `"${name}"` : '', '').trim(), name }); continue; }
    const type = BT_TYPES[kw];
    if (!type) throw new DslError(`unknown node type '${kw}'`, l.n);
    const node: BTNodeSpec = { type };
    const rest = words.slice(1).filter((w) => !/^[A-Za-z_][\w.]*=/.test(w) || quoted.includes(w));
    if (type === 'action' || type === 'condition') { node.fn = rest[0]; if (type === 'condition' && !node.fn) throw new DslError('condition needs an expression or binding', l.n); if (typeof opts.name === 'string') node.name = opts.name; else if (quoted[0] && quoted[0] !== rest[0]) node.name = quoted[0]; const args: Record<string, unknown> = {}; for (const [k, v] of Object.entries(opts)) if (!['name', 'timeout', 'pre', 'post'].includes(k)) args[k] = v; node.args = args; if (opts.timeout !== undefined) node.timeout = Number(opts.timeout); if (typeof opts.pre === 'string') node.pre = opts.pre; if (typeof opts.post === 'string') node.post = opts.post; }
    else {
      const num = rest.find((w) => /^\d+(\.\d+)?$/.test(w)); const nm = quoted[0] ?? rest.find((w) => !/^\d+(\.\d+)?$/.test(w) && w !== 'memory');
      node.name = (opts.name as string) ?? nm ?? type;
      if (type === 'parallel') node.threshold = num !== undefined ? Number(num) : undefined;
      if (type === 'retry' || type === 'repeat') node.count = num !== undefined ? Number(num) : Number(opts.count ?? 1);
      if (type === 'timeout') node.seconds = num !== undefined ? Number(num) : Number(opts.seconds ?? 1);
      if (rest.includes('memory') || opts.memory === true) node.memory = true;
      node.children = [];
    }
    while (stack.length && stack[stack.length - 1].indent >= l.indent) stack.pop();
    if (!stack.length) { if (rootSet) throw new DslError('only one root node is allowed', l.n); doc.root = node; rootSet = true; }
    else { const parent = stack[stack.length - 1].node; if (parent.type === 'action' || parent.type === 'condition') throw new DslError('leaf nodes cannot have children', l.n); parent.children!.push(node); }
    stack.push({ indent: l.indent, node });
  }
  if (!rootSet) throw new DslError('empty behavior tree');
  return doc;
}

export function parseStatechart(src: string): StatechartSpec & { checks: DesDoc['checks'] } {
  const spec: StatechartSpec & { checks: DesDoc['checks'] } = { name: 'Statechart', states: [], transitions: [], checks: [] };
  for (const l of lines(src)) {
    const t = l.text.trim(); const { words, opts, quoted } = tokenize(t); const kw = words[0]?.toLowerCase();
    if (kw === 'statechart') { spec.name = words.slice(1).join(' ') || spec.name; continue; }
    if (kw === 'state') { spec.states.push({ id: words[1], parent: opts.parent as string | undefined, region: opts.region as string | undefined, initial: words.includes('initial') || opts.initial === true, history: words.includes('history') || opts.history === true, entry: opts.entry as string | undefined, exit: opts.exit as string | undefined }); continue; }
    if (kw === 'check') { const kind = words[1]?.toLowerCase() as 'ltl' | 'ctl'; const name = quoted[0]; spec.checks.push({ kind, formula: t.replace(/^check\s+\w+\s*/i, '').replace(name ? `"${name}"` : '', '').trim(), name }); continue; }
    const m = /^(\S+)\s*-+\s*(\S+?)(?:\s*\[(.+?)\])?\s*-+>\s*(\S+)(?:\s*\/\s*(.+))?$/.exec(t);
    if (m) { spec.transitions.push({ from: m[1], event: m[2], guard: m[3], to: m[4], action: m[5] }); continue; }
    throw new DslError(`cannot parse '${t}'`, l.n);
  }
  return spec;
}

// ---------------------------------------------------------------------------------------------
// GR(1), hybrid
// ---------------------------------------------------------------------------------------------

export function parseGr1(src: string): GR1Spec {
  const spec: GR1Spec = { name: 'GR(1)', vars: [], envInit: [], envTrans: [], envLive: [], sysInit: [], sysTrans: [], sysLive: [] };
  for (const l of lines(src)) {
    const t = l.text.trim(); const kw = t.split(/\s+/)[0].toLowerCase();
    if (kw === 'gr1') { spec.name = t.split(/\s+/).slice(1).join(' ') || spec.name; continue; }
    let m: RegExpExecArray | null;
    if ((m = /^(env|sys)\s+(\S+)\s*:\s*(.+)$/i.exec(t))) { const d = m[3].trim(); spec.vars.push({ name: m[2], owner: m[1].toLowerCase() as 'env' | 'sys', domain: d === 'bool' || d === 'boolean' ? [false, true] : d.replace(/[{}]/g, '').split(/[,\s]+/).filter(Boolean).map((x) => (/^-?\d+$/.test(x) ? Number(x) : x === 'true' ? true : x === 'false' ? false : x.replace(/^'|'$/g, ''))) }); continue; }
    if ((m = /^(env_init|env_trans|env_live|sys_init|sys_trans|sys_live|assume_init|assume|assume_live|guarantee_init|guarantee|guarantee_live)\s+(.+)$/i.exec(t))) {
      const key = ({ assume_init: 'envInit', assume: 'envTrans', assume_live: 'envLive', guarantee_init: 'sysInit', guarantee: 'sysTrans', guarantee_live: 'sysLive', env_init: 'envInit', env_trans: 'envTrans', env_live: 'envLive', sys_init: 'sysInit', sys_trans: 'sysTrans', sys_live: 'sysLive' } as const)[m[1].toLowerCase() as 'env_init'];
      (spec[key] as string[]).push(m[2].trim()); continue;
    }
    throw new DslError(`cannot parse '${t}'`, l.n);
  }
  return spec;
}

export function parseHybrid(src: string): HybridSpec {
  const spec: HybridSpec = { name: 'Modes', modes: [], initial: '', transitions: [], vars: {} };
  for (const l of lines(src)) {
    const t = l.text.trim(); const { words, opts, quoted } = tokenize(t); const kw = words[0]?.toLowerCase();
    if (kw === 'hybrid' || kw === 'modes') { spec.name = words.slice(1).join(' ') || spec.name; continue; }
    if (kw === 'var' || kw === 'vars') { for (const [k, v] of Object.entries(opts)) spec.vars![k] = typeof v === 'string' ? Number(v) : (v as number | boolean); continue; }
    if (kw === 'initial') { spec.initial = words[1]; continue; }
    if (kw === 'mode') { spec.modes.push({ id: words[1], invariant: opts.invariant as string | undefined, vMax: opts.vmax as number | undefined, dynamics: (opts.dynamics as string | undefined) ?? quoted.find((q) => q !== opts.invariant) }); continue; }
    const m = /^(\S+)\s*->\s*(\S+)\s+when\s+(.+?)(?:\s+dwell=(\S+))?(?:\s+reset=(.+))?$/i.exec(t);
    if (m) { spec.transitions.push({ from: m[1], to: m[2], guard: m[3].trim(), dwell: m[4] ? Number(m[4]) : undefined, reset: m[5] }); continue; }
    throw new DslError(`cannot parse '${t}'`, l.n);
  }
  if (!spec.initial) spec.initial = spec.modes[0]?.id ?? '';
  return spec;
}

// ---------------------------------------------------------------------------------------------
// PDDL + HTN, STN, MDP
// ---------------------------------------------------------------------------------------------

export interface PddlDoc { domain: string; problem: string; htn: HTNDomain; tasks: Array<{ name: string; args: string[] }>; options: Opts }

/** The document is raw PDDL; `method` / `task` / `options` lines outside the s-expressions describe the HTN part. */
export function parsePddlDoc(src: string): PddlDoc {
  const doc: PddlDoc = { domain: '', problem: '', htn: { methods: [] }, tasks: [], options: {} };
  const extra: string[] = []; const sexpr: string[] = [];
  // separate top-level s-expressions from directive lines
  let depth = 0, buf = '';
  for (const raw of src.split(/\r?\n/)) {
    const line = raw.replace(/;.*$/, '');
    if (depth === 0 && !line.trim().startsWith('(')) { if (line.trim()) extra.push(line.trim()); continue; }
    for (const ch of line) { if (ch === '(') depth++; else if (ch === ')') depth--; }
    buf += line + '\n';
    if (depth === 0) { sexpr.push(buf); buf = ''; }
  }
  for (const s of sexpr) { if (/\(define\s*\(domain/i.test(s)) doc.domain += s; else if (/\(define\s*\(problem/i.test(s)) doc.problem += s; }
  for (const e of extra) {
    const { words, opts } = tokenize(e); const kw = words[0]?.toLowerCase();
    if (kw === 'pddl') { Object.assign(doc.options, opts); continue; }
    if (kw === 'options') { Object.assign(doc.options, opts); continue; }
    if (kw === 'task') { const m = /^task\s+([\w-]+)\s*\(([^)]*)\)/i.exec(e); if (!m) throw new DslError(`task name(args): '${e}'`); doc.tasks.push({ name: m[1].toLowerCase(), args: m[2].split(/[\s,]+/).filter(Boolean).map((x) => x.toLowerCase()) }); continue; }
    if (kw === 'method') {
      const m = /^method\s+([\w-]+)\s*\(([^)]*)\)\s*(?:"([^"]*)")?\s*:\s*(.*?)\s*=>\s*(.*)$/i.exec(e); if (!m) throw new DslError(`method task(?a ?b) "name" : (pre) => sub(?a) …: '${e}'`);
      const pre = m[4].trim() ? condFromSExpr(parseSExpr(m[4])[0]) : ({ k: 'true' } as Cond);
      const subtasks = [...m[5].matchAll(/([\w-]+)\s*\(([^)]*)\)/g)].map((x) => ({ name: x[1].toLowerCase(), args: x[2].split(/[\s,]+/).filter(Boolean).map((y) => y.toLowerCase()) }));
      doc.htn.methods.push({ task: m[1].toLowerCase(), name: m[3], params: m[2].split(/[\s,]+/).filter(Boolean).map((x) => x.toLowerCase()), pre, subtasks }); continue;
    }
    throw new DslError(`cannot parse '${e}'`);
  }
  return doc;
}
function condFromSExpr(e: SExpr): Cond {
  if (!Array.isArray(e)) return { k: 'atom', name: String(e), args: [] };
  const h = String(e[0]);
  if (h === 'and') return { k: 'and', items: e.slice(1).map(condFromSExpr) }; if (h === 'or') return { k: 'or', items: e.slice(1).map(condFromSExpr) }; if (h === 'not') return { k: 'not', a: condFromSExpr(e[1]) }; if (h === '=') return { k: 'eq', a: String(e[1]), b: String(e[2]) };
  return { k: 'atom', name: h, args: e.slice(1).map(String) };
}

export function parseStn(src: string): { name: string; constraints: STNConstraint[] } {
  const out = { name: 'STN', constraints: [] as STNConstraint[] };
  for (const l of lines(src)) {
    const t = l.text.trim(); const kw = t.split(/\s+/)[0].toLowerCase();
    if (kw === 'stn' || kw === 'stnu') { out.name = t.split(/\s+/).slice(1).join(' ') || out.name; continue; }
    const m = /^(\S+)\s*(->|=>)\s*(\S+)\s*\[\s*(-?[\d.]+|inf)\s*,\s*(-?[\d.]+|inf)\s*\]/.exec(t);
    if (!m) throw new DslError(`expected 'a -> b [min,max]' or contingent 'a => b [l,u]': '${t}'`, l.n);
    out.constraints.push({ from: m[1], to: m[3], min: m[4] === 'inf' ? -Infinity : Number(m[4]), max: m[5] === 'inf' ? Infinity : Number(m[5]), contingent: m[2] === '=>' });
  }
  return out;
}

export function parseMdp(src: string): MDPSpec {
  const spec: MDPSpec = { states: [], actions: [], transitions: {}, reward: {}, gamma: 0.95, terminal: [] };
  const addState = (s: string) => { if (!spec.states.includes(s)) spec.states.push(s); };
  for (const l of lines(src)) {
    const t = l.text.trim(); const { words, opts } = tokenize(t); const kw = words[0]?.toLowerCase();
    if (kw === 'mdp') { if (typeof opts.gamma === 'number') spec.gamma = opts.gamma; if (words.includes('minimize')) spec.minimize = true; continue; }
    if (kw === 'terminal') { for (const w of words.slice(1)) { addState(w); spec.terminal!.push(w); spec.reward[w] = { '*': 0 }; } for (const [s, v] of Object.entries(opts)) { addState(s); spec.terminal!.push(s); spec.reward[s] = { '*': Number(v) }; } continue; }
    const m = /^(\S+)\s+(\S+)\s+(-?[\d.]+)\s*:\s*(.+)$/.exec(t);
    if (!m) throw new DslError(`expected 's action reward : s1 p1, s2 p2': '${t}'`, l.n);
    const [, s, a, r, rest] = m; addState(s); if (!spec.actions.includes(a)) spec.actions.push(a);
    const outs = rest.split(',').map((x) => x.trim().split(/\s+/)).map(([to, p]) => { addState(to); return { to, p: Number(p ?? 1) }; });
    spec.transitions[s] = spec.transitions[s] ?? {}; spec.transitions[s][a] = outs; spec.reward[s] = spec.reward[s] ?? {}; spec.reward[s][a] = Number(r);
  }
  return spec;
}

// ---------------------------------------------------------------------------------------------
// Job shop, real time, FTA, reliability, performance
// ---------------------------------------------------------------------------------------------

export function parseJobshop(src: string): JobShop & { name: string } {
  const shop: JobShop & { name: string } = { name: 'Job shop', machines: [], jobs: [] }; const setups: Array<[string, string, number]> = [];
  for (const l of lines(src)) {
    const t = l.text.trim(); const { words, opts } = tokenize(t); const kw = words[0]?.toLowerCase();
    if (kw === 'jobshop') { shop.name = words.slice(1).join(' ') || shop.name; continue; }
    if (kw === 'machines') { shop.machines.push(...words.slice(1)); continue; }
    if (kw === 'setup') { const m = /^setup\s+(\S+)\s*->\s*(\S+)\s+([\d.]+)/.exec(t); if (!m) throw new DslError('setup A->B time', l.n); setups.push([m[1], m[2], Number(m[3])]); continue; }
    if (kw === 'job') {
      const m = /^job\s+(\S+)(.*?):\s*(.+)$/.exec(t); if (!m) throw new DslError('job J k=v : M1 5 ; S1 22 | S2 26', l.n);
      const ops = m[3].split(';').map((o) => ({ alternatives: o.split('|').map((alt) => { const [machine, p] = alt.trim().split(/\s+/); if (!shop.machines.includes(machine)) shop.machines.push(machine); return { machine, p: Number(p) }; }) }));
      const job: Job = { id: m[1], ops, due: opts.due as number | undefined, release: opts.release as number | undefined, weight: opts.weight as number | undefined, type: opts.type as string | undefined }; shop.jobs.push(job); continue;
    }
    throw new DslError(`cannot parse '${t}'`, l.n);
  }
  if (setups.length) shop.setup = (from, to) => setups.find(([a, b]) => (a === from || a === '*') && (b === to || b === '*'))?.[2] ?? 0;
  return shop;
}

export function parseRealtime(src: string): { name: string; tasks: RTTask[]; chains: Array<{ name: string; mode: 'async' | 'sync'; stages: Array<{ id: string; T: number; R: number }> }>; protocol: 'pcp' | 'pip' | 'none'; speed?: number; tolerance?: number } {
  const out = { name: 'Real time', tasks: [] as RTTask[], chains: [] as Array<{ name: string; mode: 'async' | 'sync'; stages: Array<{ id: string; T: number; R: number }> }>, protocol: 'pcp' as 'pcp' | 'pip' | 'none', speed: undefined as number | undefined, tolerance: undefined as number | undefined };
  for (const l of lines(src)) {
    const t = l.text.trim(); const { words, opts } = tokenize(t); const kw = words[0]?.toLowerCase();
    if (kw === 'realtime') { out.name = words.slice(1).filter((w) => !w.includes('=')).join(' ') || out.name; if (typeof opts.protocol === 'string') out.protocol = opts.protocol as 'pcp'; if (typeof opts.speed === 'number') out.speed = opts.speed; if (typeof opts.tolerance === 'number') out.tolerance = opts.tolerance; continue; }
    if (kw === 'task') { const cs: Record<string, number> = {}; if (typeof opts.cs === 'string') for (const part of opts.cs.split(',')) { const [r, len] = part.split(':'); cs[r] = Number(len); } out.tasks.push({ id: words[1], C: Number(opts.C ?? opts.c), T: Number(opts.T ?? opts.t), D: opts.D !== undefined ? Number(opts.D) : undefined, criticalSections: Object.keys(cs).length ? cs : undefined, priority: opts.priority as number | undefined }); continue; }
    if (kw === 'chain') { const stages = words.slice(1).filter((w) => w.includes(':') && !w.includes('=')).map((w) => { const [id, tr] = w.split(':'); const [T, R] = tr.split('/').map(Number); return { id, T, R }; }); out.chains.push({ name: (opts.name as string) ?? `chain ${out.chains.length + 1}`, mode: (opts.mode as 'async') ?? 'async', stages }); continue; }
    throw new DslError(`cannot parse '${t}'`, l.n);
  }
  return out;
}

export function parseFta(src: string): { name: string; tree: FTNode } {
  let name = 'Fault tree'; const stack: Array<{ indent: number; node: FTNode }> = []; let root: FTNode | null = null;
  for (const l of lines(src)) {
    const t = l.text.trim(); const { words, opts, quoted } = tokenize(t); const kw = words[0]?.toLowerCase();
    if (kw === 'fta') { name = words.slice(1).join(' ') || name; continue; }
    let node: FTNode;
    if (kw === 'and' || kw === 'or') node = { kind: kw, id: words[1], name: quoted[0], children: [] };
    else if (kw === 'event') node = { kind: 'event', id: words[1], p: Number(opts.p ?? 0), name: quoted[0] };
    else throw new DslError(`expected and|or|event: '${t}'`, l.n);
    while (stack.length && stack[stack.length - 1].indent >= l.indent) stack.pop();
    if (!stack.length) { if (root) throw new DslError('only one top event', l.n); root = node; } else { const p = stack[stack.length - 1].node; if (p.kind === 'event') throw new DslError('events have no children', l.n); p.children.push(node); }
    stack.push({ indent: l.indent, node });
  }
  if (!root) throw new DslError('empty fault tree');
  return { name, tree: root };
}

export interface ReliabilityDoc { name: string; components: Component[]; mttr: number; ssm?: { vRobot: number; tReaction: number; tStop: number; decel?: number; Zd?: number; Zr?: number; C?: number; distance?: number }; pl?: { S: 1 | 2; F: 1 | 2; P: 1 | 2; cat?: string; mttfdYears?: number; dc?: number; ccf?: number }; fdir?: { rateHz: number; falseAlarmsPerDay: number }; weibull?: { eta: number; beta: number; costPlanned: number; costFailure: number } }
export function parseReliability(src: string): ReliabilityDoc {
  const doc: ReliabilityDoc = { name: 'Reliability', components: [], mttr: 4 };
  for (const l of lines(src)) {
    const t = l.text.trim(); const { words, opts } = tokenize(t); const kw = words[0]?.toLowerCase();
    if (kw === 'reliability') { doc.name = words.slice(1).join(' ') || doc.name; continue; }
    if (kw === 'component') { doc.components.push({ id: words[1], lambda: typeof opts.lambda === 'number' ? opts.lambda : 1 / Number(opts.mttf ?? 1e6), redundancy: opts.redundancy as number | undefined }); continue; }
    if (kw === 'mttr') { doc.mttr = Number(words[1] ?? opts.hours ?? 4); continue; }
    if (kw === 'ssm') { doc.ssm = { vRobot: Number(opts.v ?? opts.vrobot ?? 1), tReaction: Number(opts.tr ?? 0.15), tStop: Number(opts.ts ?? 0.35), decel: opts.decel as number | undefined, Zd: opts.zd as number | undefined, Zr: opts.zr as number | undefined, C: opts.c as number | undefined, distance: opts.distance as number | undefined }; continue; }
    if (kw === 'pl') { doc.pl = { S: Number(opts.s) as 1, F: Number(opts.f) as 1, P: Number(opts.p) as 1, cat: opts.cat !== undefined ? String(opts.cat) : undefined, mttfdYears: opts.mttfd as number | undefined, dc: opts.dc as number | undefined, ccf: opts.ccf as number | undefined }; continue; }
    if (kw === 'fdir') { doc.fdir = { rateHz: Number(opts.rate ?? 100), falseAlarmsPerDay: Number(opts.false_alarms ?? opts.fa ?? 1) }; continue; }
    if (kw === 'weibull') { doc.weibull = { eta: Number(opts.eta), beta: Number(opts.beta), costPlanned: Number(opts.cp ?? 1), costFailure: Number(opts.cf ?? 10) }; continue; }
    throw new DslError(`cannot parse '${t}'`, l.n);
  }
  return doc;
}

export interface PerfDoc { name: string; loads: ResourceLoad[]; ops: CycleOperation[]; capacity: Record<string, number>; little?: { wip?: number; throughput?: number; leadTime?: number }; saturate?: string }
export function parsePerf(src: string): PerfDoc {
  const doc: PerfDoc = { name: 'Performance', loads: [], ops: [], capacity: {} };
  for (const l of lines(src)) {
    const t = l.text.trim(); const { words, opts } = tokenize(t); const kw = words[0]?.toLowerCase();
    if (kw === 'perf') { doc.name = words.slice(1).filter((w) => !w.includes('=')).join(' ') || doc.name; if (typeof opts.saturate === 'string') doc.saturate = opts.saturate; continue; }
    if (kw === 'resource') { doc.loads.push({ resource: words[1], timePerCycle: Number(opts.time ?? 0), capacity: Number(opts.capacity ?? 1) }); doc.capacity[words[1]] = Number(opts.capacity ?? 1); continue; }
    if (kw === 'op') { doc.ops.push({ id: words[1], duration: Number(opts.duration ?? opts.d ?? 1), resources: String(opts.resources ?? '').split(',').map((x) => x.trim()).filter(Boolean), after: opts.after ? String(opts.after).split(',').map((x) => x.trim()) : undefined }); continue; }
    if (kw === 'little') { doc.little = { wip: opts.wip as number | undefined, throughput: opts.throughput as number | undefined, leadTime: opts.leadtime as number | undefined }; continue; }
    throw new DslError(`cannot parse '${t}'`, l.n);
  }
  return doc;
}

/** Header line + JSON body (pomdp, mrta, mapf, fmea, acceptance). */
export function parseJsonDoc<T>(src: string): { name: string; body: T } {
  const ls = src.split(/\r?\n/); const idx = ls.findIndex((x) => x.trim().length && !x.trim().startsWith('#') && !x.trim().startsWith('//'));
  const header = ls[idx] ?? ''; const name = header.trim().split(/\s+/).slice(1).join(' ');
  const body = ls.slice(idx + 1).join('\n').trim();
  try { return { name, body: JSON.parse(body || '{}') as T }; } catch (e) { throw new DslError(`invalid JSON body: ${(e as Error).message}`); }
}

export interface StlDoc { name: string; specs: Array<{ name?: string; formula: string }>; signal: { t: number[]; values: Record<string, number | boolean>[] } | null; falsify?: { space: Record<string, [number, number]>; budget?: number } }
export function parseStl(src: string): StlDoc {
  const doc: StlDoc = { name: 'STL', specs: [], signal: null };
  const ls = lines(src); let i = 0; let cols: string[] | null = null;
  while (i < ls.length) {
    const t = ls[i].text.trim(); const n = ls[i].n; i++; const { words, quoted, opts } = tokenize(t); const kw = words[0]?.toLowerCase();
    if (kw === 'stl') { doc.name = words.slice(1).join(' ') || doc.name; continue; }
    if (kw === 'spec') { const name = quoted[0]; doc.specs.push({ name, formula: t.replace(/^spec\s*/i, '').replace(name ? `"${name}"` : '', '').trim() }); continue; }
    if (kw === 'falsify') { const space: Record<string, [number, number]> = {}; for (const [k, v] of Object.entries(opts)) { if (k === 'budget') continue; const m = /^\[?(-?[\d.]+),(-?[\d.]+)\]?$/.exec(String(v)); if (m) space[k] = [Number(m[1]), Number(m[2])]; } doc.falsify = { space, budget: opts.budget as number | undefined }; continue; }
    if (kw === 'signal') { cols = words.slice(1); doc.signal = { t: [], values: [] }; continue; }
    if (cols && doc.signal) { const nums = t.split(/[\s,]+/); if (nums.length !== cols.length) throw new DslError(`signal row needs ${cols.length} values`, n); const row: Record<string, number | boolean> = {}; cols.forEach((c, k) => { row[c] = nums[k] === 'true' ? true : nums[k] === 'false' ? false : Number(nums[k]); }); doc.signal.t.push(Number(row[cols[0]])); doc.signal.values.push(row); continue; }
    throw new DslError(`cannot parse '${t}'`, n);
  }
  return doc;
}

// ---------------------------------------------------------------------------------------------
// Templates
// ---------------------------------------------------------------------------------------------

const CTL_TEMPLATES: Record<CtlKind, string> = {
  des: `des Two machines and a buffer
automaton M1
  initial I
  marked I
  I -start1-> W
  W -finish1-> I
automaton M2
  initial I
  marked I
  I -start2-> W
  W -finish2-> I
spec Buffer   # one slot: M2 may start only after M1 finished
  initial E
  marked E
  E -finish1-> F
  F -start2-> E
uncontrollable finish1 finish2
check ltl G(M1='W' -> F M1='I')`,
  petri: `petri Two processes sharing a robot and a zone
place p1 tokens=1 kind=idle
place a1 kind=activity
place a2 kind=activity
place p2 tokens=1 kind=idle
place b1 kind=activity
place b2 kind=activity
place rM tokens=1 kind=resource "robot"
place rZ tokens=1 kind=resource "zone"
transition t1a delay=4
transition t2a delay=5
transition t3a delay=1
transition t1b delay=5
transition t2b delay=4
transition t3b delay=1
arc p1, rM -> t1a -> a1
arc a1, rZ -> t2a -> a2
arc a2 -> t3a -> rM, rZ, p1
arc p2, rZ -> t1b -> b1
arc b1, rM -> t2b -> b2
arc b2 -> t3b -> rZ, rM, p2
check ctl AG !deadlock`,
  s3pr: `s3pr Cell
resource M1 1
resource Z 1
process A jobs=1: take[M1] 4; zone[M1,Z] 5
process B jobs=1: zone[Z] 5; robot[Z,M1] 4`,
  smv: `smv Shared zone with priority
var r1.state : {idle, waiting, in_zone, leaving}
var r2.state : {idle, waiting, in_zone, leaving}
var owner : {none, one, two}
define g1 := (owner = none) | (owner = one)
define g2 := (owner = none) & !(r1.state = waiting)
init r1.state := idle
init r2.state := idle
init owner := none
next r1.state := case
  r1.state = idle : {idle, waiting};
  r1.state = waiting & g1 : in_zone;
  r1.state = waiting & !g1 : waiting;
  r1.state = in_zone : {in_zone, leaving};
  r1.state = leaving : {idle, waiting};
esac
next r2.state := case
  r2.state = idle : {idle, waiting};
  r2.state = waiting & g2 : in_zone;
  r2.state = waiting & !g2 : waiting;
  r2.state = in_zone : {in_zone, leaving};
  r2.state = leaving : {idle, waiting};
esac
next owner := case
  r1.state = waiting & g1 : one;
  r2.state = waiting & g2 : two;
  r1.state = leaving & owner = one : none;
  r2.state = leaving & owner = two : none;
  TRUE : owner;
esac
fairness !(r1.state = in_zone)
fairness !(r2.state = in_zone)
ltlspec "mutual exclusion" G !(r1.state = in_zone & r2.state = in_zone)
ltlspec "no starvation r2" G (r2.state = waiting -> F r2.state = in_zone)
ctlspec "recoverable" AG EF owner = none`,
  bt: `bt Mission
fallback root
  sequence emergency
    condition estop
    action halt timeout=1
  sequence battery
    condition "battery < 0.2"
    sequence "to dock" memory
      action goto zone=dock timeout=120
      action dock timeout=30
  fallback mission
    sequence collect
      condition "targets > 0 || held"
      sequence "collect one" memory
        action goto zone=table timeout=120
        retry 2
          sequence grasp memory
            action grasp timeout=12 post="held"
        action goto zone=bin timeout=120
        action place timeout=10
    sequence finish memory
      action goto zone=home timeout=120
      action report
outcomes halt=success,running
check ltl G("ok:estop" -> "tick:halt")
monitor G(held -> (held U placed))`,
  statechart: `statechart Operation modes
state Off initial
state On
state Auto parent=On initial history
state Idle parent=Auto region=ctl initial
state Busy parent=Auto region=ctl
state Cold parent=Auto region=temp initial
state Hot parent=Auto region=temp
state Manual parent=On
state Fault parent=On
Off -power_on-> On
On -power_off-> Off
Idle -start-> Busy
Busy -done-> Idle
Cold -heat-> Hot
Hot -cool-> Cold
Auto -manual-> Manual
Manual -auto-> Auto
On -error-> Fault
Fault -reset-> Auto`,
  gr1: `gr1 Patrol with a door
sys loc : a b
env door : bool
sys_init loc = 'a'
sys_trans (loc = 'a' && loc' = 'b') -> door
sys_trans (loc = 'b' && loc' = 'a') -> door
sys_live loc = 'a'
sys_live loc = 'b'
env_live door`,
  hybrid: `hybrid Speed modes
var dist_human=10 v=0 human=false estop=false ack=false
initial NORMAL
mode NORMAL invariant="v <= 1.2" vmax=1.2
mode SLOW invariant="v <= 0.3" vmax=0.3
mode STOP vmax=0
NORMAL -> SLOW when dist_human < 2.0 dwell=0.4
SLOW -> NORMAL when dist_human > 2.5 dwell=0.4
NORMAL -> STOP when human || estop
SLOW -> STOP when human || estop
STOP -> NORMAL when !human && !estop && ack`,
  pddl: `pddl search=gbfs heuristic=hff
(define (domain pick-place)
  (:requirements :strips :typing)
  (:types location object)
  (:predicates (at-robot ?l - location) (connected ?a ?b - location) (on ?o - object ?l - location) (holding ?o - object) (empty))
  (:action move :parameters (?a ?b - location) :precondition (and (at-robot ?a) (connected ?a ?b)) :effect (and (at-robot ?b) (not (at-robot ?a))))
  (:action pick :parameters (?o - object ?l - location) :precondition (and (at-robot ?l) (on ?o ?l) (empty)) :effect (and (holding ?o) (not (on ?o ?l)) (not (empty))))
  (:action place :parameters (?o - object ?l - location) :precondition (and (at-robot ?l) (holding ?o)) :effect (and (on ?o ?l) (empty) (not (holding ?o)))))
(define (problem p1)
  (:domain pick-place)
  (:objects base table bin - location bolt - object)
  (:init (at-robot base) (empty) (on bolt table) (connected base table) (connected table base) (connected table bin) (connected bin table))
  (:goal (and (on bolt bin) (at-robot base))))`,
  stn: `stnu Delivery to a machine
z -> start [20,20]
start => arrive [8,14]
z => free [30,40]
free -> unload [0,5]
arrive -> unload [0,100]`,
  mdp: `mdp Grasp strategy gamma=1 minimize
terminal s2=0 s4=60
s0 refine 3 : s1 1.0
s0 grasp 8 : s2 0.55, s3 0.45
s0 abort 0 : s4 1.0
s1 grasp 8 : s2 0.88, s3 0.12
s1 abort 0 : s4 1.0
s3 refine 5 : s1 0.8, s3 0.2
s3 abort 0 : s4 1.0`,
  pomdp: `pomdp Classification before placing
{ "states": ["bolt", "nut", "other"], "actions": ["look", "look_closer", "place_bolt", "place_nut", "skip"], "observations": ["obs_bolt", "obs_nut", "obs_other"],
  "terminalActions": ["place_bolt", "place_nut", "skip"], "gamma": 1, "transitions": {},
  "observation": { "look": { "bolt": {"obs_bolt": 0.7, "obs_nut": 0.2, "obs_other": 0.1}, "nut": {"obs_bolt": 0.2, "obs_nut": 0.7, "obs_other": 0.1}, "other": {"obs_bolt": 0.15, "obs_nut": 0.15, "obs_other": 0.7} },
                  "look_closer": { "bolt": {"obs_bolt": 0.94, "obs_nut": 0.04, "obs_other": 0.02}, "nut": {"obs_bolt": 0.04, "obs_nut": 0.94, "obs_other": 0.02}, "other": {"obs_bolt": 0.03, "obs_nut": 0.03, "obs_other": 0.94} } },
  "reward": { "bolt": {"look": -0.5, "look_closer": -4, "place_bolt": 20, "place_nut": -50, "skip": -10}, "nut": {"look": -0.5, "look_closer": -4, "place_bolt": -50, "place_nut": 20, "skip": -10}, "other": {"look": -0.5, "look_closer": -4, "place_bolt": -50, "place_nut": -50, "skip": 0} },
  "belief": { "bolt": 0.4, "nut": 0.4, "other": 0.2 }, "horizon": 2 }`,
  jobshop: `jobshop Cell with two machines
machines M1 S1 S2 M2
job J1 type=A : M1 5 ; S1 22 | S2 26 ; M2 5
job J2 type=B : M1 5 ; S1 30 | S2 24 ; M2 5
job J3 type=A : M1 5 ; S1 22 | S2 26 ; M2 5`,
  realtime: `realtime Onboard computer protocol=pcp speed=0.15 tolerance=0.015
task drive C=1.2 T=5
task localization C=8 T=50
task planner C=25 T=100
task perception C=30 T=200
chain camera:33/33 image:50/93.8 pose:50/12 planner:20/15 drive:5/1.2 mode=async name="visual servoing"`,
  fta: `fta Contact with a human
and top "unacceptable contact"
  or undetected "human in zone, not detected in time"
    event A p=1e-5 "scanner failure"
    event B p=1e-2 "human outside the field of view"
    event C p=1e-3 "processing delay"
  and moving "robot moves at a dangerous speed"
    event D p=1 "motion command"
    and protect "protective layer failed"
      event E p=1e-3 "CBF filter failure"
      event F p=1e-7 "hardware chain failure"`,
  reliability: `reliability Production cell
component M1 lambda=2e-4
component M2 lambda=2e-4
component S lambda=5e-4
component conveyor1 lambda=1e-4
component conveyor2 lambda=1e-4
component controller lambda=5e-5
component network lambda=3e-5
component AMR lambda=4e-4
mttr 4
ssm v=1.2 tr=0.15 ts=0.35 zd=0.10 zr=0.05 c=0.20 decel=1.5 distance=1.0
pl s=2 f=2 p=2 cat=3 mttfd=40 dc=0.95 ccf=70
fdir rate=100 fa=1`,
  mrta: `mrta Three AMRs and three pallets
{ "cost": [[55, 62, 70], [40, 12, 35], [48, 30, 14]], "robots": ["AMR1", "AMR2", "AMR3"], "tasks": ["cell1", "cell2", "cell3"],
  "auction": { "robots": [{"id": "r1", "at": {"x": 0, "y": 0}}, {"id": "r2", "at": {"x": 10, "y": 0}}], "tasks": [{"id": "t1", "at": {"x": 2, "y": 1}}, {"id": "t2", "at": {"x": 8, "y": 1}}, {"id": "t3", "at": {"x": 5, "y": 5}}] } }`,
  mapf: `mapf Corridor with a pocket
{ "nodes": ["a", "c1", "c2", "b", "p"], "edges": [["a", "c1"], ["c1", "c2"], ["c2", "b"], ["c1", "p"]],
  "agents": [{"id": "A", "start": "a", "goal": "b"}, {"id": "B", "start": "b", "goal": "a"}], "delays": {"A": 3} }`,
  stl: `stl Distance to a human
spec "R6" G[0,60] (human -> dist >= 0.8)
signal t dist human
0 5 false
1 4 false
2 3 true
3 1.2 true
4 0.9 true
5 2 false`,
  fmea: `fmea Mobile manipulator
[ {"element": "gripper", "failureMode": "does not hold", "systemEffect": "object falls", "detection": "presence sensor + monitor", "S": 7, "O": 4, "D": 3, "measure": "force control, zone restriction"},
  {"element": "camera", "failureMode": "no frames", "systemEffect": "no recognition", "detection": "QoS deadline", "S": 4, "O": 3, "D": 2, "measure": "wait mode"},
  {"element": "base encoder", "failureMode": "no signal", "systemEffect": "uncontrolled motion", "detection": "consistency check", "S": 9, "O": 2, "D": 3, "measure": "stop category 1 via hardware chain"} ]`,
  acceptance: `acceptance Mission time and grasp success
{ "trials": {"n": 100, "failures": 3, "target": 0.95},
  "samples": {"name": "mission time (min)", "values": [11.2, 10.8, 12.5, 11.9, 9.8, 13.1, 11.4, 10.2, 12.8, 11.0, 12.2, 10.6, 11.7, 13.4, 9.9, 11.3, 12.0, 10.9, 11.6, 12.4], "limit": 15, "kind": "max"},
  "simReal": {"sim": [52.1, 51.5, 53.0, 52.8, 51.9, 52.6, 53.3, 51.2], "real": [58.0, 61.5, 55.2, 60.1, 57.4, 62.3, 56.8, 59.9]},
  "pairwise": {"object": ["bolt", "nut", "washer", "other"], "light": ["dim", "normal", "bright"], "position": ["p1", "p2", "p3", "p4", "p5"], "neighbour": ["no", "yes"], "gripper": ["empty", "holding"]},
  "requirements": [{"id": "R1", "text": "arm still while base moves", "cls": "safety", "formal": "G !(base_mv & arm_mv)", "verification": "model checking + monitor", "result": "pass", "components": ["supervisor"]},
                   {"id": "R4", "text": "mission within 15 min", "cls": "performance", "verification": "20 runs", "result": "open", "components": ["planner"]}] }`,
  perf: `perf Production cell saturate=S
resource M1 time=9 capacity=1
resource S time=22 capacity=1
resource M2 time=9 capacity=1
resource Z time=10 capacity=1
resource AMR time=6 capacity=1
op u1 duration=4 resources=M1
op u2 duration=5 resources=M1,S,Z after=u1
op u3 duration=22 resources=S after=u2
op u4 duration=5 resources=S,M2,Z after=u3
op u5 duration=4 resources=M2 after=u4
little throughput=0.0278 leadtime=90`,
};
export const TEMPLATES: Record<ControlKind, string> = { ...CTL_TEMPLATES, ...MRS_TEMPLATES };

export type { Transition };
