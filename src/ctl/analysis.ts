/**
 * Analysis dispatcher: a control document (kind + DSL text) → structured report (sections, tables, graph views,
 * metrics) + Markdown. Every course method is reachable from here; the UI, the API and the demo scenarios all use it.
 */
import { ControlKind, CtlKind } from './model';
import { MRS_ANALYSERS } from '../mrs/analysis';
import * as dsl from './dsl';
import { DES, parallel, analyseBlocking, supcon, checkControllability, checkNonconflict, observer, checkObservability, checkDiagnosability, supervisorTable, AutomatonSpec } from './des';
import { PetriNet, analysePetriNet, preventDeadlocks, simulateTimed, analyseGspn, buildS3PR } from './petri';
import { bottleneckBound, saturationCurve, cycleTimeFromOperations, littlesLaw } from './perf';
import { checkLTL, checkCTL, kripkeFromDES, kripkeFromPetri, Kripke, LTL3Monitor } from './temporal';
import { SmvModel } from './smv';
import { checkBehaviorTree, estimateFTS, btKripke, Statechart, BTNodeSpec } from './bt';
import { synthesizeGR1 } from './gr1';
import { checkHybrid, dwellTime } from './hybrid';
import { parseDomain, parseProblem, ground, plan, validatePlan, scheduleGantt, htnPlan, solveSTN, checkSTNU, Heuristic } from './planning';
import { valueIteration, policyIteration, POMDPSpec, beliefUpdate, qmdp, alphaVectorBackup, lookaheadValue, bestTerminalAction, decisionThreshold } from './mdp';
import { hungarian, greedyAssignment, bottleneckAssignment, sequentialAuction, cbba, priorityPlanning, cbs, buildTPG, simulateTPG, criticalSections, Graph, Agent, AuctionRobot, AuctionTask } from './mrta';
import { dispatch, lowerBounds, bestAssignmentBound, branchAndBound, criticalPath, scheduleRobustness, DispatchRule } from './sched';
import { responseTimes, edfTest, blockingBounds, endToEndLatency, latencyBudget } from './realtime';
import { systemReliability, minimalCutSets, topEventProbability, ftaSensitivity, requiredPL, achievedPL, mttfdClass, dcClass, plMeets, separationDistance, admissibleSpeed, detectionThreshold, optimalReplacement, fmeaTable, FmeaRow, Category } from './reliability';
import { parseSTL, robustness, formatSTL, pairwise, ruleOfThree, clopperPearsonLower, trialsForTarget, simRealGap, acceptanceCheck, traceabilitySummary, Requirement } from './vv';

export interface GraphView { kind: 'automaton' | 'petri' | 'tree' | 'modes' | 'statechart' | 'controller' | 'network'; nodes: Array<{ id: string; label: string; kind?: string; initial?: boolean; marked?: boolean; tokens?: number }>; edges: Array<{ from: string; to: string; label?: string }>; /** draw edges without arrowheads (communication graphs) */ undirected?: boolean; /** pinned positions in model units (scaled to the drawing) */ positions?: Record<string, [number, number]> }
/** A chart in a report section (time series, planar trajectories, a cell grid or bars); rendered as SVG by the Control tab. */
export interface PlotSeries { name: string; points: Array<[number, number]>; color?: string; dashed?: boolean; markers?: boolean }
export interface PlotView { kind: 'lines' | 'paths' | 'grid' | 'bars'; title?: string; xlabel?: string; ylabel?: string; series?: PlotSeries[]; /** grid cells (rows of values) */ cells?: number[][]; cellStyle?: 'binary' | 'heat' | 'labels'; /** rectangles (shelves, walls) in plot units */ obstacles?: Array<{ x: number; y: number; w: number; h: number }>; markers?: Array<{ x: number; y: number; label?: string; kind?: 'goal' | 'robot' | 'source' | 'station' | 'task' | 'start' | 'obstacle' }>; bars?: Array<{ label: string; value: number }>; equal?: boolean; width?: number; height?: number; ylog?: boolean }
export interface ReportSection { title: string; level: 'ok' | 'warn' | 'error' | 'info'; lines: string[]; table?: { head: string[]; rows: Array<Array<string | number>> }; graph?: GraphView; plot?: PlotView }
export interface Report { kind: ControlKind; title: string; ok: boolean; sections: ReportSection[]; metrics: Record<string, number | string | boolean>; markdown: string; error?: string; durationMs: number }

const f = (v: number, d = 2) => (Number.isFinite(v) ? Number(v.toFixed(d)).toString() : String(v));
const sec = (title: string, level: ReportSection['level'], lines: string[] = [], extra: Partial<ReportSection> = {}): ReportSection => ({ title, level, lines, ...extra });

export function analyse(kind: ControlKind, source: string): Report {
  const t0 = Date.now();
  const sections: ReportSection[] = []; const metrics: Record<string, number | string | boolean> = {}; let title: string = kind; let ok = true;
  try {
    const r = ANALYSERS[kind](source, sections, metrics);
    title = r.title; ok = r.ok;
  } catch (e) {
    const msg = (e as Error).message;
    sections.push(sec('Error', 'error', [msg]));
    return { kind, title, ok: false, sections, metrics, markdown: toMarkdown(title, sections, metrics, false), error: msg, durationMs: Date.now() - t0 };
  }
  return { kind, title, ok, sections, metrics, markdown: toMarkdown(title, sections, metrics, ok), durationMs: Date.now() - t0 };
}

export function toMarkdown(title: string, sections: ReportSection[], metrics: Record<string, number | string | boolean>, ok: boolean): string {
  const out = [`## ${title} — ${ok ? 'OK' : 'issues found'}`, ''];
  for (const s of sections) {
    out.push(`### ${{ ok: '✅', warn: '⚠️', error: '❌', info: 'ℹ️' }[s.level]} ${s.title}`, '');
    for (const l of s.lines) out.push(`- ${l}`);
    if (s.table) { out.push('', `| ${s.table.head.join(' | ')} |`, `|${s.table.head.map(() => '---').join('|')}|`); for (const r of s.table.rows) out.push(`| ${r.join(' | ')} |`); }
    if (s.plot) out.push('', `*[chart: ${s.plot.title ?? s.plot.kind}]*`);
    out.push('');
  }
  if (Object.keys(metrics).length) { out.push('### Metrics', '', '| Metric | Value |', '|---|---|'); for (const [k, v] of Object.entries(metrics)) out.push(`| ${k} | ${v} |`); }
  return out.join('\n');
}

export type Analyser = (src: string, sections: ReportSection[], metrics: Record<string, number | string | boolean>) => { title: string; ok: boolean };

function runChecks(k: Kripke, checks: Array<{ kind: 'ltl' | 'ctl'; formula: string; name?: string }>, sections: ReportSection[], fairness?: string[]): boolean {
  let ok = true;
  for (const c of checks) {
    const label = `${c.kind.toUpperCase()}${c.name ? ` "${c.name}"` : ''}: ${c.formula}`;
    try {
      if (c.kind === 'ltl') { const r = checkLTL(k, c.formula, { fairness }); ok = ok && r.holds; sections.push(sec(label, r.holds ? 'ok' : 'error', r.holds ? [`holds (Büchi ${r.buchiStates} states, product ${r.productStates})`] : [`VIOLATED — counterexample: ${r.prefix.join(' → ')}${r.cycle.length ? ` then repeat [${r.cycle.join(' → ')}]` : ''}`])); }
      else { const r = checkCTL(k, c.formula); ok = ok && r.holds; sections.push(sec(label, r.holds ? 'ok' : 'error', r.holds ? [`holds in ${r.satisfying}/${r.total} states`] : [`VIOLATED — ${r.explanation}${r.counterexample ? `: ${r.counterexample.map((s) => k.describe?.(s) ?? s).join(' → ')}` : ''}`])); }
    } catch (e) { ok = false; sections.push(sec(label, 'error', [`cannot evaluate: ${(e as Error).message}`])); }
  }
  return ok;
}

function automatonGraph(g: DES, limit = 80): GraphView | undefined {
  if (g.X.size > limit) return undefined;
  return { kind: 'automaton', nodes: [...g.X].map((x) => ({ id: x, label: x, initial: x === g.x0, marked: g.xm.has(x) })), edges: g.transitions().map((t) => ({ from: t.from, to: t.to, label: t.event })) };
}

const CTL_ANALYSERS: Record<CtlKind, Analyser> = {
  des: (src, S, M) => {
    const doc = dsl.parseDes(src); let ok = true;
    if (!doc.plant.length) throw new Error('no automaton blocks');
    const comps = doc.plant.map((a) => DES.fromSpec(a)); const specs = doc.specs.map((a) => DES.fromSpec(a));
    S.push(sec('Components', 'info', [], { table: { head: ['Automaton', 'States', 'Events', 'Transitions', 'Marked'], rows: comps.map((c) => [c.name, c.X.size, c.sigma.size, c.transitionCount, [...c.xm].join(' ')]) } }));
    if (doc.actions.length) { const bad = doc.actions.filter((a) => { const [blk, st] = a.target.includes('.') ? [a.target.slice(0, a.target.lastIndexOf('.')), a.target.slice(a.target.lastIndexOf('.') + 1)] : [null, a.target]; return !comps.some((c) => (blk === null || c.name === blk) && c.X.has(st)); }); S.push(sec(`Bound actions (${doc.actions.length})`, bad.length ? 'warn' : 'info', [...doc.actions.map((a) => `${a.target}: ${a.kind} ${a.value}${a.robot ? ` on ${a.robot}` : ''}${a.done ? ` → ${a.done}` : ''}`), ...bad.map((a) => `unknown state ${a.target}`)])); }
    const g = comps.length === 1 ? comps[0] : parallel(...comps);
    if (comps.length === 1) { g.components = [g.name]; for (const x of g.X) g.parts.set(x, [x]); }
    const upper = comps.reduce((p, c) => p * c.X.size, 1);
    const b = analyseBlocking(g);
    M['plant states'] = b.reachable; M['plant transitions'] = b.transitions; M['blocking states'] = b.blocking.length;
    S.push(sec('Plant G = ' + comps.map((c) => c.name).join(' || '), b.nonblocking ? 'ok' : 'error', [
      `upper bound ${upper} states, reachable ${b.reachable}, transitions ${b.transitions}`,
      b.nonblocking ? 'non-blocking: every reachable state can reach a marked state' : `BLOCKING: ${b.blocking.length} states cannot reach a marked state (${b.deadlocks.length} deadlocks, ${b.livelocks.length} livelock components)`,
      ...(b.deadlockTrace ? [`deadlock trace: ${b.deadlockTrace.join(' ')} → ${g.run(b.deadlockTrace)!.slice(-1)[0]}`] : []),
      ...(b.livelockTrace ? [`livelock trace: ${b.livelockTrace.join(' ')} (cycle of ${b.livelocks[0].length} states without a marked state)`] : []),
    ], { graph: automatonGraph(g) }));
    if (!b.nonblocking && !specs.length) ok = false;
    const uc = doc.uncontrollable;
    if (specs.length) {
      for (const sp of specs) { const c = checkControllability(g, sp, uc); if (!c.controllable) S.push(sec(`Specification ${sp.name}: controllability`, 'warn', [`${sp.name} is NOT controllable as a language: ${c.issues.length} states where the plant enables an uncontrollable event the spec forbids (e.g. ${c.issues[0].event} after "${c.issues[0].trace.join(' ')}"). Synthesis will restrict the behaviour further (or find it empty).`])); }
      const r = supcon(g, specs, uc);
      M['supervisor states'] = r.states.size; M['product states'] = r.productStates; M['realizable'] = r.realizable;
      const lines = [`H = G || ${specs.map((s) => s.name).join(' || ')}: ${r.productStates} states; supervisor: ${r.states.size} states, ${r.supervisor.transitionCount} transitions`, ...r.log.map((l) => `iteration ${l.iteration}: removed ${l.removedControllability} by controllability, ${l.removedBlocking} by blocking / reachability`)];
      if (r.realizable) { lines.push(`disabled controllable events (states where the plant allows the event but the supervisor forbids it): ${Object.entries(r.disabled).sort((a, b2) => b2[1] - a[1]).map(([e, n]) => `${e} ×${n}`).join(', ') || 'none'}`); const marked = r.supervisor.traceTo((x) => r.supervisor.xm.has(x) && x !== r.supervisor.x0); if (marked) lines.push(`shortest marked run under supervision (${marked.length} events): ${marked.join(' ')}`); }
      else { ok = false; lines.push('UNREALISABLE: the supremal controllable non-blocking sublanguage is empty — an uncontrollable event forbidden by a specification cannot be prevented, or every behaviour blocks. Weaken the specification (e.g. forbid the event only in some contexts) or add controllable means.'); }
      S.push(sec('Supervisor synthesis (Ramadge–Wonham, Algorithm 3.1)', r.realizable ? 'ok' : 'error', lines, { graph: r.realizable ? automatonGraph(r.supervisor) : undefined }));
      if (specs.length > 1 && r.realizable) {
        const mods = specs.map((sp) => supcon(g, [sp], uc));
        const nc = checkNonconflict(g, mods.map((m) => m.supervisor));
        M['modular nonconflicting'] = nc.nonconflicting;
        S.push(sec('Modular supervisors', nc.nonconflicting ? 'ok' : 'warn', [`${specs.map((sp, i) => `${sp.name}: ${mods[i].states.size} states`).join(', ')}`, nc.nonconflicting ? `joint behaviour of the ${specs.length} modular supervisors: ${nc.jointStates} states, non-conflicting` : `CONFLICT: ${nc.blocking} blocking states in the joint behaviour (trace ${nc.trace?.join(' ')}) — use the monolithic supervisor or a coordinator`]));
      }
      if (doc.unobservable.length && r.realizable) {
        const obs = observer(r.supervisor, doc.unobservable);
        const ctrl = [...g.sigma].filter((e) => !uc.includes(e));
        const ob = checkObservability(g, r.states, r.supervisor, doc.unobservable, ctrl);
        M['observer states'] = obs.states.size;
        const amb = [...obs.states.values()].filter((cell) => cell.length > 1).length;
        S.push(sec(`Partial observation (unobservable: ${doc.unobservable.join(', ')})`, ob.observable ? 'ok' : 'warn', [`observer: ${obs.states.size} states, ${amb} with more than one possible plant state`, ob.observable ? 'the supervisor decisions are observable: inside every observer cell each controllable event is uniformly enabled or disabled' : `NOT observable: ${ob.issues.length} decisions depend on unobservable history (e.g. ${ob.issues[0].event} in cell ${ob.issues[0].cell.slice(0, 60)}) — add sensing or make the decision conservative`]));
      }
      if (doc.faults.length) { const d = checkDiagnosability(g, doc.faults, doc.unobservable); M['diagnosable'] = d.diagnosable; S.push(sec(`Diagnosability of ${doc.faults.join(', ')}`, d.diagnosable ? 'ok' : 'warn', [d.diagnosable ? 'every fault is detected after finitely many observable events (no indeterminate cycle in the twin plant)' : `NOT diagnosable: the fault can stay hidden forever (ambiguous cycle reached by ${d.witness?.join(' ')})`])); }
      if (doc.checks.length) { const k = kripkeFromDES(r.realizable ? r.supervisor : g); ok = runChecks(k, doc.checks, S) && ok; }
      if (r.realizable) { const tbl = supervisorTable(r.supervisor, uc, doc.unobservable, comps.length); M['supervisor table entries'] = tbl.states.length; }
    } else {
      if (doc.unobservable.length) { const obs = observer(g, doc.unobservable); S.push(sec('Observer', 'info', [`${obs.states.size} observer states under unobservable ${doc.unobservable.join(', ')}`])); }
      if (doc.faults.length) { const d = checkDiagnosability(g, doc.faults, doc.unobservable); M['diagnosable'] = d.diagnosable; S.push(sec(`Diagnosability of ${doc.faults.join(', ')}`, d.diagnosable ? 'ok' : 'warn', [d.diagnosable ? 'diagnosable' : `not diagnosable (ambiguous cycle via ${d.witness?.join(' ')})`])); }
      if (doc.checks.length) ok = runChecks(kripkeFromDES(g), doc.checks, S) && ok;
    }
    return { title: doc.name, ok };
  },

  petri: (src, S, M) => petriReport(dsl.parsePetri(src), S, M),
  s3pr: (src, S, M) => { const spec = dsl.parseS3PR(src); const net = buildS3PR(spec); return petriReport({ spec: net.spec, checks: [], horizon: 300 }, S, M, spec.name); },

  perf: (src, S, M) => {
    const doc = dsl.parsePerf(src); let ok = true;
    if (doc.loads.length) {
      const b = bottleneckBound(doc.loads); M['bottleneck'] = b.bottleneck; M['cycle time bound (s)'] = f(b.cycleTime); M['throughput bound (/h)'] = f(b.throughputPerHour, 1);
      S.push(sec('Bottleneck bound (Proposition 5.1)', 'info', [`T_cycle ≥ ${f(b.cycleTime)} s at resource ${b.bottleneck} → Θ ≤ ${f(b.throughputPerHour, 1)} parts/h`], { table: { head: ['Resource', 'busy s / cycle', 'capacity', 'cycle bound s'], rows: doc.loads.map((l) => [l.resource, l.timePerCycle, l.capacity, f(l.timePerCycle / l.capacity)]) } }));
      const sat = saturationCurve(doc.loads, doc.saturate ?? b.bottleneck, 4);
      S.push(sec(`Saturation curve: capacity of ${doc.saturate ?? b.bottleneck}`, 'info', [], { table: { head: ['capacity', 'cycle s', 'parts/h', 'new bottleneck'], rows: sat.map((x) => [x.capacity, f(x.cycleTime), f(x.throughputPerHour, 1), x.bottleneck]) } }));
    }
    if (doc.ops.length) { const c = cycleTimeFromOperations(doc.ops, doc.capacity); M['max-plus cycle time (s)'] = f(c.cycleTime); S.push(sec('Max-plus cycle time (resource circuits, §5.6)', 'info', [`λ = ${f(c.cycleTime)} s (${f(c.throughputPerHour, 1)} parts/h); critical circuit: ${c.criticalCycle.join(' → ')}`], { table: { head: ['resource', 'holding time', 'tokens', 'time/token'], rows: c.resourceCycles.map((r) => [r.resource, r.time, r.tokens, f(r.mean)]) } })); }
    if (doc.little) { const l = littlesLaw(doc.little); S.push(sec("Little's law", 'info', [`WIP ${f(l.wip)} = Θ ${f(l.throughput, 4)} /s × lead time ${f(l.leadTime)} s`])); }
    return { title: doc.name, ok };
  },

  smv: (src, S, M) => {
    const spec = dsl.parseSmv(src); const model = new SmvModel(spec); const k = model.kripke();
    M['states'] = k.states.length; M['initial states'] = k.initial.length;
    S.push(sec('Kripke structure', 'info', [`${k.states.length} reachable states from ${k.initial.length} initial state(s); variables ${spec.vars.map((v) => `${v.name} (${v.domain.length})`).join(', ')}`, ...(spec.fairness?.length ? [`fairness: ${spec.fairness.join('; ')}`] : [])]));
    const ok = runChecks(k, (spec.specs ?? []).map((s) => ({ kind: s.kind.toLowerCase() as 'ltl', formula: s.formula, name: s.name })), S, spec.fairness);
    return { title: spec.name, ok };
  },

  bt: (src, S, M) => {
    const doc = dsl.parseBt(src); let ok = true;
    const issues = checkBehaviorTree(doc.root); const errors = issues.filter((i) => i.level === 'error');
    if (errors.length) ok = false;
    const count = (n: BTNodeSpec): number => 1 + (n.children ?? []).reduce((s, c) => s + count(c), 0);
    M['nodes'] = count(doc.root); M['structural errors'] = errors.length;
    S.push(sec('Structure', errors.length ? 'error' : issues.length ? 'warn' : 'ok', issues.length ? issues.map((i) => `${i.level}: ${i.node} — ${i.message}`) : ['no structural issues'], { graph: treeGraph(doc.root) }));
    const fts = estimateFTS(doc.root, { estop: { p: 0 }, ...doc.leafModels }, 100, 400, 3);
    M['P(success)'] = f(fts.pSuccess, 2);
    S.push(sec('Finite-time success (Monte Carlo over the leaf models)', fts.pSuccess > 0.5 ? 'info' : 'warn', [`P(success) = ${f(fts.pSuccess, 2)}, P(failure) = ${f(fts.pFailure, 2)}, P(timeout) = ${f(fts.pTimeout, 2)}; mean ticks to success ${f(fts.meanTicks, 1)}`, `leaf models: ${Object.entries(doc.leafModels).map(([k, v]) => `${k} p=${v.p ?? 1}${v.ticks ? ` ticks=${v.ticks}` : ''}`).join(', ') || 'defaults (p = 1)'}`]));
    if (doc.checks.length) { const k = btKripke(doc.root, doc.outcomes, 4000); M['abstract states'] = k.states.length; S.push(sec('Abstraction for model checking', 'info', [`${k.states.length} abstract states over non-deterministic leaf outcomes${Object.keys(doc.outcomes).length ? `; restricted: ${Object.entries(doc.outcomes).map(([l, o]) => `${l} ∈ {${o.join(',')}}`).join(', ')}` : ''}`])); ok = runChecks(k, doc.checks, S) && ok; }
    for (const m of doc.monitors) { try { const mon = new LTL3Monitor(m); S.push(sec(`Runtime monitor ${mon.formula}`, 'info', [`${mon.atoms.length} atoms; ${mon.table().states.length} monitor states — attached at runtime`])); } catch (e) { ok = false; S.push(sec(`Monitor ${m}`, 'error', [(e as Error).message])); } }
    return { title: doc.name, ok };
  },

  statechart: (src, S, M) => {
    const spec = dsl.parseStatechart(src); const sc = new Statechart(spec); const flat = sc.flatten();
    M['states'] = spec.states.length; M['configurations'] = flat.states?.length ?? 0;
    S.push(sec('Flattening', 'info', [`${spec.states.length} states (${spec.states.filter((s) => sc.isComposite(s.id)).length} composite), ${flat.states?.length} reachable configurations, ${flat.transitions.length} flat transitions, ${flat.events.length} events`], { graph: { kind: 'statechart', nodes: spec.states.map((s) => ({ id: s.id, label: s.id + (s.history ? ' (H)' : ''), kind: s.parent ?? '', initial: !!s.initial })), edges: spec.transitions.map((t) => ({ from: t.from, to: t.to, label: t.event + (t.guard ? `[${t.guard}]` : '') })) } }));
    const g = DES.fromSpec(flat); const b = analyseBlocking(g);
    S.push(sec('Reachability', 'info', [`initial configuration ${flat.initial}; ${b.deadlocks.length} configurations without outgoing transitions`]));
    const ok = runChecks(kripkeFromDES(g), spec.checks, S);
    return { title: spec.name, ok };
  },

  gr1: (src, S, M) => {
    const spec = dsl.parseGr1(src); const r = synthesizeGR1(spec);
    M['game states'] = r.gameStates; M['winning states'] = r.winning; M['realizable'] = r.realizable; if (r.controller) M['controller states'] = r.controller.size;
    const lines = [`game: ${r.gameStates} states (${spec.vars.filter((v) => v.owner === 'env').length} env / ${spec.vars.filter((v) => v.owner === 'sys').length} sys variables), winning region ${r.winning} states`, `assumptions: ${(spec.envLive ?? []).length} GF, guarantees: ${(spec.sysLive ?? []).length} GF`];
    if (r.realizable) lines.push(`REALISABLE — controller with ${r.controller!.size} states extracted (Mealy machine: env input → sys next values)`);
    else { lines.push('UNREALISABLE'); lines.push(...r.diagnosis); for (const c of r.counterStrategy) lines.push(`counter-strategy: from ${Object.entries(c.state).map(([k, v]) => `${k}=${v}`).join(' ')} the environment plays ${Object.entries(c.envMove).map(([k, v]) => `${k}=${v}`).join(' ')} (${c.note})`); }
    let graph: GraphView | undefined;
    if (r.controller && r.controller.size <= 60) graph = { kind: 'controller', nodes: r.controller.states.map((s) => ({ id: String(s.id), label: `${Object.entries(s.values).filter(([k]) => spec.vars.find((v) => v.name === k)?.owner === 'sys').map(([, v]) => v).join(',')} j${s.goal}`, initial: r.controller!.initial.includes(s.id) })), edges: [...r.controller.transitions.entries()].flatMap(([from, m]) => [...m.entries()].map(([lbl, to]) => ({ from: String(from), to: String(to), label: lbl }))) };
    S.push(sec('GR(1) synthesis', r.realizable ? 'ok' : 'error', lines, { graph }));
    return { title: spec.name ?? 'GR(1)', ok: r.realizable };
  },

  hybrid: (src, S, M) => {
    const spec = dsl.parseHybrid(src); const issues = checkHybrid(spec); const errors = issues.filter((i) => i.level === 'error');
    M['modes'] = spec.modes.length; M['transitions'] = spec.transitions.length; M['chattering pairs'] = errors.filter((e) => /chattering/.test(e.message)).length;
    S.push(sec('Mode automaton', errors.length ? 'error' : 'ok', issues.length ? issues.map((i) => `${i.level}: ${i.message}`) : ['no issues'], { graph: { kind: 'modes', nodes: spec.modes.map((m) => ({ id: m.id, label: m.id + (m.vMax !== undefined ? ` ≤${m.vMax} m/s` : ''), initial: m.id === spec.initial })), edges: spec.transitions.map((t) => ({ from: t.from, to: t.to, label: t.guard + (t.dwell ? ` ⏱${t.dwell}s` : '') })) } }));
    S.push(sec('Dwell time (Theorem 9.3)', 'info', [`for decay λ = 2 s⁻¹ and Lyapunov jump μ = 4: τ_d > ${f(dwellTime(2, 4), 3)} s — the mode logic must not switch faster; transitions with dwell: ${spec.transitions.filter((t) => t.dwell).length}/${spec.transitions.length}`]));
    return { title: spec.name, ok: errors.length === 0 };
  },

  pddl: (src, S, M) => {
    const doc = dsl.parsePddlDoc(src); if (!doc.domain) throw new Error('no (define (domain …)) block'); if (!doc.problem) throw new Error('no (define (problem …)) block');
    const d = parseDomain(doc.domain), p = parseProblem(doc.problem); const t = ground(d, p);
    M['ground actions'] = t.actions.length; M['atoms'] = t.atoms.length;
    S.push(sec('Grounding', 'info', [`domain ${d.name}: ${d.actions.length} schemas, ${d.predicates.length} predicates; problem ${p.name}: ${p.objects.length} objects → ${t.actions.length} ground actions, ${t.atoms.length} atoms`]));
    const search = (doc.options.search as 'gbfs' | 'astar') ?? 'gbfs'; const heuristic = (doc.options.heuristic as Heuristic) ?? (search === 'astar' ? 'hmax' : 'hff');
    const r = plan(t, { search, heuristic, maxExpansions: Number(doc.options.max_expansions ?? 200000), timeLimitMs: 20000 });
    let ok = r.found;
    M['plan found'] = r.found; if (r.found) { M['plan length'] = r.plan.length; M['plan cost'] = r.cost; } M['expanded'] = r.expanded;
    const v = r.found ? validatePlan(t, r.plan) : null;
    S.push(sec(`Plan (${search.toUpperCase()} / ${heuristic})`, r.found ? 'ok' : 'error', r.found ? [`${r.plan.length} actions, cost ${r.cost}, ${r.expanded} expansions in ${r.timeMs} ms; validation: ${v!.valid ? 'ok' : v!.reason}`, ...r.plan.map((a, i) => `${i + 1}. ${a.name}${a.cost !== 1 ? ` (cost ${a.cost})` : ''}`)] : [`no plan: ${r.message}`]));
    if (r.found && d.actions.some((a) => a.temporal)) { const g = scheduleGantt(r.plan); M['makespan'] = g.makespan; S.push(sec('Schedule (durative actions, ASAP with resource precedence)', 'info', [`makespan ${g.makespan}`], { table: { head: ['action', 'start', 'end'], rows: g.rows.map((x) => [x.action, x.start, x.end]) } })); }
    if (doc.tasks.length) { const h = htnPlan(t, doc.htn, doc.tasks); ok = ok && h.found; M['HTN plan found'] = h.found; S.push(sec('HTN decomposition', h.found ? 'ok' : 'error', h.found ? [`${h.plan.length} primitive actions after ${h.nodes} decomposition nodes`, ...h.trace.slice(0, 20), ...h.plan.map((a, i) => `${i + 1}. ${a.name}`)] : [`no decomposition (${h.nodes} nodes explored)`, ...h.trace.slice(-5)])); }
    return { title: `${d.name} / ${p.name}`, ok };
  },

  stn: (src, S, M) => {
    const doc = dsl.parseStn(src); const stn = solveSTN(doc.constraints);
    M['consistent'] = stn.consistent;
    S.push(sec('Simple temporal network (Floyd–Warshall)', stn.consistent ? 'ok' : 'error', [stn.consistent ? `consistent; makespan between ${f(stn.makespan[0])} and ${f(stn.makespan[1])}` : `INCONSISTENT: negative cycle through ${stn.negativeCycle?.join(', ')}`], { table: stn.consistent ? { head: ['node', 'earliest', 'latest'], rows: stn.nodes.map((n) => [n, f(stn.earliest[n]), f(stn.latest[n])]) } : undefined }));
    let ok = stn.consistent;
    if (doc.constraints.some((c) => c.contingent)) { const dc = checkSTNU(doc.constraints); M['dynamically controllable'] = dc.controllable; ok = ok && dc.controllable; S.push(sec('STNU dynamic controllability (Morris reductions)', dc.controllable ? 'ok' : 'error', [dc.controllable ? `dynamically controllable (${dc.rounds} reduction rounds, ${dc.edges} labelled edges): a dispatch strategy exists whatever the contingent durations turn out to be` : `NOT dynamically controllable: ${dc.witness} — loosen a requirement, start earlier, or reduce the spread of the contingent durations`])); }
    return { title: doc.name, ok };
  },

  mdp: (src, S, M) => {
    const spec = dsl.parseMdp(src); const vi = valueIteration(spec); const pi = policyIteration(spec);
    M['states'] = spec.states.length; M['VI iterations'] = vi.iterations; M['PI iterations'] = pi.iterations;
    S.push(sec(`Optimal policy (${spec.minimize ? 'minimise cost' : 'maximise reward'}, γ = ${spec.gamma})`, 'ok', [`value iteration converged in ${vi.iterations} sweeps, policy iteration in ${pi.iterations}`], { table: { head: ['state', 'V*', 'π*', 'Q values'], rows: spec.states.map((s) => [s, f(vi.values[s], 3), vi.policy[s] ?? (spec.terminal?.includes(s) ? 'terminal' : '—'), Object.entries(vi.qValues[s] ?? {}).map(([a, q]) => `${a}: ${f(q, 3)}`).join(', ')]) } }));
    return { title: 'MDP', ok: true };
  },

  pomdp: (src, S, M) => {
    const { name, body } = dsl.parseJsonDoc<POMDPSpec & { belief?: Record<string, number>; horizon?: number }>(src);
    const m: POMDPSpec = { ...body, transitions: body.transitions ?? {} };
    const b0 = body.belief ?? Object.fromEntries(m.states.map((s) => [s, 1 / m.states.length]));
    const av = alphaVectorBackup(m, body.horizon ?? 2); const best = av.value(b0); const q = qmdp(m, b0);
    M['α-vectors'] = av.vectors.length; M['best action'] = best.action; M['V(b0)'] = f(best.value, 2);
    S.push(sec(`Belief b0 = ${Object.entries(b0).map(([s, p]) => `${s}:${f(p, 2)}`).join(' ')}`, 'info', [`exact ${body.horizon ?? 2}-step value ${f(best.value, 2)} with action ${best.action} (${av.vectors.length} α-vectors); QMDP suggests ${q.action} (never gathers information)`]));
    const rows: Array<Array<string | number>> = [];
    for (const a of m.actions) {
      if ((m.terminalActions ?? []).includes(a)) rows.push([a, f(m.states.reduce((s, x) => s + b0[x] * (m.reward[x]?.[a] ?? 0), 0), 2), 'terminal', '']);
      else { const la = lookaheadValue(m, b0, a, (b2) => bestTerminalAction(m, b2)); rows.push([a, f(la.value, 2), 'observe then act', la.branches.map((br) => `${br.o} (p ${f(br.p, 2)}) → ${br.next} ${f(br.value, 1)}`).join('; ')]); }
    }
    S.push(sec('One-step lookahead per action (§12.4.2)', 'info', [], { table: { head: ['action', 'expected value', 'type', 'branches'], rows } }));
    const obsA = m.actions.find((a) => !(m.terminalActions ?? []).includes(a)); if (obsA) { const upd = beliefUpdate(m, b0, obsA, m.observations[0]); S.push(sec(`Belief update after ${obsA} → ${m.observations[0]}`, 'info', [`P(obs) = ${f(upd.pObs, 3)}; b' = ${Object.entries(upd.belief).map(([s, p]) => `${s}:${f(p, 3)}`).join(' ')}`])); }
    const rewards = m.states.flatMap((s) => Object.values(m.reward[s] ?? {})); const pos = Math.max(...rewards), neg = Math.abs(Math.min(...rewards)); const obsCost = Math.abs(Math.min(...m.actions.filter((a) => !(m.terminalActions ?? []).includes(a)).map((a) => m.reward[m.states[0]]?.[a] ?? 0)));
    S.push(sec('Decision threshold (§12.4.3)', 'info', [`act immediately only when the belief in the target class exceeds β ≥ ${f(decisionThreshold(pos, neg, obsCost), 3)} (reward ${pos}, penalty ${neg}, observation cost ${obsCost}) — the threshold is a property of the task, not of the classifier`]));
    return { title: name || 'POMDP', ok: true };
  },

  mrta: (src, S, M) => {
    const { name, body } = dsl.parseJsonDoc<{ cost?: number[][]; robots?: string[]; tasks?: string[]; auction?: { robots: AuctionRobot[]; tasks: AuctionTask[] } }>(src);
    if (body.cost) {
      const h = hungarian(body.cost), g = greedyAssignment(body.cost), b = bottleneckAssignment(body.cost);
      M['optimal sum'] = h.total; M['greedy sum'] = g.total; M['minimax'] = b.makespan;
      S.push(sec('Assignment (ST-SR-IA)', 'ok', [`Hungarian: total ${h.total} — ${h.assignment.map((j, i) => `${body.robots?.[i] ?? 'r' + i} → ${j >= 0 ? body.tasks?.[j] ?? 't' + j : '—'}`).join(', ')}`, `greedy: total ${g.total} (${g.total > h.total ? `+${f(100 * (g.total / h.total - 1), 0)} %` : 'same'})`, `minimax (bottleneck) assignment: makespan ${b.makespan} — ${b.assignment.map((j, i) => `${body.robots?.[i] ?? 'r' + i} → ${body.tasks?.[j] ?? 't' + j}`).join(', ')}`]));
    }
    if (body.auction) {
      const a = sequentialAuction(body.auction.robots, body.auction.tasks); const c = cbba(body.auction.robots, body.auction.tasks);
      M['auction total'] = f(a.totalCost, 1); M['CBBA total'] = f(c.totalCost, 1);
      S.push(sec('Distributed allocation', 'info', [`sequential greedy auction: total ${f(a.totalCost, 1)}, makespan ${f(a.makespan, 1)} — ${Object.entries(a.bundles).map(([r, ts]) => `${r}: ${ts.join(' ')}`).join('; ')}`, `CBBA: converged in ${c.iterations} iterations (${c.converged}); total ${f(c.totalCost, 1)}, makespan ${f(c.makespan, 1)} — ${Object.entries(c.bundles).map(([r, ts]) => `${r}: ${ts.join(' ')}`).join('; ')}`]));
    }
    return { title: name || 'Task allocation', ok: true };
  },

  mapf: (src, S, M) => {
    const { name, body } = dsl.parseJsonDoc<Graph & { agents: Agent[]; delays?: Record<string, number>; order?: string[] }>(src);
    const g: Graph = { nodes: body.nodes, edges: body.edges, directed: body.directed };
    const pp = priorityPlanning(g, body.agents, body.order); const r = cbs(g, body.agents);
    M['priority failed'] = pp.failed.length; M['CBS found'] = r.found; if (r.found) { M['sum of costs'] = r.sumOfCosts; M['makespan'] = r.makespan; }
    S.push(sec('Priority planning', pp.failed.length ? 'warn' : 'ok', [pp.failed.length ? `incomplete: no path for ${pp.failed.join(', ')} with this priority order` : `all ${body.agents.length} agents planned, sum of costs ${pp.sumOfCosts}, makespan ${pp.makespan}`]));
    S.push(sec('Conflict-based search (optimal)', r.found ? 'ok' : 'error', r.found ? [`sum of costs ${r.sumOfCosts}, makespan ${r.makespan}, ${r.expanded} CT nodes`, ...Object.entries(r.paths).map(([a, p]) => `${a}: ${p.join(' → ')}`)] : ['no solution']));
    if (r.found) { const tpg = buildTPG(r.paths); const sim = simulateTPG(tpg, body.delays ?? {}); M['TPG acyclic'] = tpg.acyclic; S.push(sec('Temporal plan graph execution', tpg.acyclic && sim.collisions === 0 ? 'ok' : 'error', [`${tpg.events.length} events, ${tpg.arcs.filter((a) => a.kind === 'priority').length} priority arcs, ${tpg.acyclic ? 'acyclic' : 'CYCLE = deadlock'}`, `execution with delays ${JSON.stringify(body.delays ?? {})}: ${sim.finished ? 'finished' : 'stuck'} in ${sim.makespan} steps, ${sim.collisions} collisions`])); }
    const crit = criticalSections(g, body.agents); if (crit.length) S.push(sec('Critical sections (robot failure analysis)', 'warn', [`blocking any of ${crit.join(', ')} disconnects a start from its goal — plan an evacuation procedure or a detour`]));
    return { title: name || 'MAPF', ok: r.found };
  },

  jobshop: (src, S, M) => {
    const shop = dsl.parseJobshop(src); const lb = lowerBounds(shop); const ab = bestAssignmentBound(shop);
    const rules: DispatchRule[] = ['SPT', 'LPT', 'EDD', 'MWKR', 'FIFO', 'CR', 'ATC'];
    const results = rules.map((r) => ({ r, s: dispatch(shop, r) })); const bb = branchAndBound(shop, { maxNodes: 100000 });
    const best = bb.schedule; const cp = criticalPath(best); const rob = scheduleRobustness(shop, best, 0.15, 30);
    const LB = Math.max(lb.lb, ab.lb);
    M['lower bound'] = LB; M['best makespan'] = best.makespan; M['optimal'] = bb.optimal; M['gap %'] = f(100 * (best.makespan / LB - 1), 1);
    S.push(sec('Lower bounds (Theorem 14.6)', 'info', [`LB1 machine load ${f(lb.lb1)} (${lb.bottleneck}), LB2 job length ${f(lb.lb2)}, LB3 heads+tails ${f(lb.lb3)}, best machine assignment ${f(ab.lb)} → LB = ${f(LB)}`]));
    S.push(sec('Dispatch rules', 'info', [], { table: { head: ['rule', 'makespan', 'ΣC', 'Lmax', 'late'], rows: results.map(({ r, s }) => [r, f(s.makespan), f(s.sumC), f(s.Lmax), s.late]) } }));
    S.push(sec(`Branch and bound: makespan ${f(best.makespan)} (${bb.optimal ? 'optimal' : 'best found'}, ${bb.nodes} nodes), gap to LB ${f(100 * (best.makespan / LB - 1), 1)} %`, 'ok', [`critical path: ${cp.map((o) => `${o.job}/${o.machine}[${o.start}-${o.end}]`).join(' → ')}`, `robustness (cv 0.15, 30 runs): makespan ${f(rob.mean, 1)} ± ${f(rob.ci95, 1)} (p95 ${f(rob.p95, 1)})`], { table: { head: ['job', 'op', 'machine', 'start', 'end'], rows: [...best.ops].sort((a, b) => a.start - b.start).map((o) => [o.job, o.op, o.machine, f(o.start), f(o.end)]) } }));
    return { title: shop.name, ok: true };
  },

  realtime: (src, S, M) => {
    const doc = dsl.parseRealtime(src); let ok = true;
    if (doc.tasks.length) {
      const B = blockingBounds(doc.tasks, doc.protocol); const r = responseTimes(doc.tasks, B); const e = edfTest(doc.tasks);
      M['utilisation'] = f(r.U, 3); M['RM schedulable'] = r.schedulable; M['EDF schedulable'] = e.schedulable;
      ok = ok && r.schedulable;
      S.push(sec(`Rate-monotonic analysis (U = ${f(r.U, 3)}, Liu–Layland bound ${f(r.llBound, 3)} ${r.llSufficient ? 'satisfied' : 'not satisfied → exact test'})`, r.schedulable ? 'ok' : 'error', [r.schedulable ? 'all deadlines met (Joseph–Pandya response times)' : 'DEADLINE MISS', `blocking protocol: ${doc.protocol}`], { table: { head: ['task', 'C', 'T', 'D', 'B', 'R', 'ok'], rows: r.results.map((x) => { const t = doc.tasks.find((y) => y.id === x.id)!; return [x.id, t.C, t.T, x.D, f(x.B), f(x.R, 1), x.schedulable ? '✓' : '✗']; }) } }));
      S.push(sec('EDF', e.schedulable ? 'ok' : 'warn', [`${e.schedulable ? 'schedulable' : 'NOT schedulable'} — ${e.note}; note: EDF degrades catastrophically under overload, RM predictably`]));
    }
    for (const ch of doc.chains) { const a = endToEndLatency(ch.stages, 'async'), s = endToEndLatency(ch.stages, 'sync'); const lines = [`asynchronous (register buffers) Σ(T+R) = ${f(a.latency, 1)} ms; synchronous (event-triggered) ΣR = ${f(s.latency, 1)} ms`, `largest contribution: ${a.contributions.sort((x, y) => y.value - x.value)[0].id}`]; if (doc.speed && doc.tolerance) { const b = latencyBudget((ch.mode === 'sync' ? s.latency : a.latency) / 1000, doc.speed, doc.tolerance); lines.push(`at ${doc.speed} m/s the latency costs ${f(b.errorM * 1000, 0)} mm (tolerance ${doc.tolerance * 1000} mm) → ${b.ok ? 'OK' : `NOT OK; max speed ${f(b.maxSpeedMps, 3)} m/s or move the heaviest stage to its own core`}`); ok = ok && b.ok; } S.push(sec(`Chain ${ch.name} (${ch.mode})`, 'info', lines)); }
    return { title: doc.name, ok };
  },

  fta: (src, S, M) => {
    const { name, tree } = dsl.parseFta(src); const cs = minimalCutSets(tree); const p = topEventProbability(tree); const sens = ftaSensitivity(tree);
    M['minimal cut sets'] = cs.length; M['min order'] = p.minOrder; M['P(top) rare-event'] = p.rareEvent.toExponential(2);
    S.push(sec('Minimal cut sets (MOCUS)', p.minOrder <= 1 ? 'error' : 'ok', [`${cs.length} minimal cut sets, minimal order ${p.minOrder}${p.minOrder <= 1 ? ' — a SINGLE failure causes the top event: no redundancy' : ''}`, `P(top) ≤ ${p.rareEvent.toExponential(2)} (rare-event approximation)${p.exact !== null ? `, exact ${p.exact.toExponential(3)}` : ''}`, `dominant cut set: {${p.dominant.join(', ')}}`], { table: { head: ['cut set', 'order', 'probability'], rows: p.cutSets.map((c) => [`{${c.events.join(', ')}}`, c.events.length, c.p.toExponential(2)]) } }));
    S.push(sec('Sensitivity: reduce each basic event ×10', 'info', [], { table: { head: ['event', 'gain in P(top)'], rows: sens.map((x) => [x.event, `${f(100 * x.gain, 1)} %`]) } }));
    return { title: name, ok: p.minOrder > 1 };
  },

  reliability: (src, S, M) => {
    const doc = dsl.parseReliability(src); let ok = true;
    if (doc.components.length) { const r = systemReliability(doc.components, doc.mttr); M['MTTF (h)'] = f(r.mttf, 0); M['availability'] = f(r.availability, 4); S.push(sec('System reliability (series, exponential)', 'info', [`λ_sys = ${r.lambda.toExponential(2)} h⁻¹ → MTTF ${f(r.mttf, 0)} h (${f(r.mttf / 8, 0)} shifts); availability ${f(r.availability, 4)} at MTTR ${doc.mttr} h (${f(r.downtimePerWeekMin, 0)} min lost per 80-h week)`], { table: { head: ['component', 'λ (h⁻¹)', 'share'], rows: r.contributions.map((c) => [c.id, c.lambda.toExponential(2), `${f(100 * c.share, 1)} %`]) } })); }
    if (doc.weibull) { const w = optimalReplacement(doc.weibull.eta, doc.weibull.beta, doc.weibull.costPlanned, doc.weibull.costFailure); S.push(sec('Weibull preventive replacement', 'info', [`optimal age-replacement interval ${f(w.interval, 0)} h with cost rate ${w.costRate.toExponential(2)} vs run-to-failure ${w.runToFailureRate.toExponential(2)}`])); }
    if (doc.ssm) { const s = separationDistance(doc.ssm); M['S_p (m)'] = f(s.Sp, 2); const lines = [`S_p = S_h ${f(s.Sh, 2)} + S_r ${f(s.Sr, 2)} + S_s ${f(s.Ss, 2)} + C + Z_d + Z_r = ${f(s.Sp, 2)} m at ${doc.ssm.vRobot} m/s`]; if (doc.ssm.distance && doc.ssm.decel) { const v = admissibleSpeed({ distance: doc.ssm.distance, tReaction: doc.ssm.tReaction, decel: doc.ssm.decel, Zd: doc.ssm.Zd, Zr: doc.ssm.Zr, C: doc.ssm.C }); M['admissible speed (m/s)'] = f(v, 3); lines.push(`with only ${doc.ssm.distance} m available the admissible speed is ${f(v, 3)} m/s → a SLOW mode with hysteresis in the mode automaton`); } S.push(sec('ISO/TS 15066 speed and separation monitoring', 'info', lines)); }
    if (doc.pl) { const plr = requiredPL(doc.pl.S, doc.pl.F, doc.pl.P); const lines = [`risk graph S${doc.pl.S} F${doc.pl.F} P${doc.pl.P} → PLr = ${plr}`]; if (doc.pl.cat && doc.pl.mttfdYears !== undefined && doc.pl.dc !== undefined) { const mc = mttfdClass(doc.pl.mttfdYears); const a = mc ? achievedPL(doc.pl.cat as Category, mc, dcClass(doc.pl.dc), doc.pl.ccf ?? 65) : { pl: null, note: 'MTTFd below 3 years' }; const meets = plMeets(a.pl, plr); ok = ok && meets; lines.push(`${a.note} → ${meets ? 'meets' : 'DOES NOT meet'} PLr ${plr}`); lines.push('only certified safety components count (scanner, safety PLC, hardware chain); CBF filters, monitors and learned perception improve quality but not PL'); M['PLr'] = plr; M['PL'] = a.pl ?? 'n/a'; } S.push(sec('ISO 13849-1 performance level', ok ? 'ok' : 'error', lines)); }
    if (doc.fdir) { const d = detectionThreshold({ checkRateHz: doc.fdir.rateHz, maxFalseAlarmsPerDay: doc.fdir.falseAlarmsPerDay }); S.push(sec('FDIR detection threshold', 'info', [`≤ ${doc.fdir.falseAlarmsPerDay} false alarm(s)/day at ${doc.fdir.rateHz} Hz → p ≤ ${d.pPerCheck.toExponential(2)} per check: a single ${f(d.kSigma, 1)}σ threshold, or 3σ confirmed in ${d.confirmSteps3Sigma} consecutive checks (${f(d.delayMs, 0)} ms detection delay, keeps sensitivity)`])); }
    return { title: doc.name, ok };
  },

  fmea: (src, S, M) => {
    const { name, body } = dsl.parseJsonDoc<FmeaRow[]>(src); const rows = fmeaTable(body);
    M['rows'] = rows.length; M['high priority'] = rows.filter((r) => r.ap === 'H').length; M['mandatory (S ≥ 9)'] = rows.filter((r) => r.mandatory).length;
    S.push(sec('FMEA with action priority (AIAG/VDA)', rows.some((r) => r.ap === 'H') ? 'warn' : 'ok', ['RPN is listed for reference only; the action priority puts severity first, and S ≥ 9 always requires a measure'], { table: { head: ['AP', 'element', 'failure mode', 'system effect', 'S', 'O', 'D', 'RPN', 'measure'], rows: rows.map((r) => [r.ap + (r.mandatory ? ' !' : ''), r.element, r.failureMode, r.systemEffect, r.S, r.O, r.D, r.rpn, r.measure]) } }));
    return { title: name || 'FMEA', ok: true };
  },

  acceptance: (src, S, M) => {
    const { name, body } = dsl.parseJsonDoc<{ trials?: { n: number; failures: number; target: number }; samples?: { name: string; values: number[]; limit: number; kind: 'max' | 'min' }; simReal?: { sim: number[]; real: number[] }; pairwise?: Record<string, string[]>; requirements?: Requirement[] }>(src);
    let ok = true;
    if (body.trials) { const { n, failures, target } = body.trials; const lower = clopperPearsonLower(n - failures, n); const need = trialsForTarget(target, failures); M['lower bound'] = f(lower, 3); ok = ok && lower >= target; S.push(sec('Trials and confidence', lower >= target ? 'ok' : 'warn', [`${n - failures}/${n} successes: point estimate ${f((n - failures) / n, 3)}, Clopper–Pearson 95 % lower bound ${f(lower, 3)} ${lower >= target ? '≥' : '<'} target ${target}`, `rule of three: ${n} failure-free trials would bound the failure probability by ${f(ruleOfThree(n), 3)}`, lower >= target ? 'requirement demonstrated' : `NOT demonstrated: ${need} trials are needed with ${failures} allowed failures`])); }
    if (body.samples) { const a = acceptanceCheck(body.samples.values, body.samples.limit, body.samples.kind); ok = ok && a.passWithConfidence; S.push(sec(`${body.samples.name}: ${body.samples.kind === 'max' ? '≤' : '≥'} ${body.samples.limit}`, a.passWithConfidence ? 'ok' : 'warn', [`mean ${f(a.mean, 2)}, 95 % bound ${f(a.bound95, 2)} — ${a.note}`])); }
    if (body.simReal) { const g = simRealGap(body.simReal.sim, body.simReal.real); S.push(sec('Sim-to-real gap', g.significant ? 'warn' : 'ok', [`bias ${f(g.bias, 2)} (${f(g.biasPercent, 1)} %), spread ratio ${f(g.spreadRatio, 2)}, Welch t = ${f(g.t, 2)} → ${g.significant ? 'significant: apply a correction factor and find the missing effect in the model' : 'not significant'}`])); }
    if (body.pairwise) { const rows = pairwise(body.pairwise); const full = Object.values(body.pairwise).reduce((s, v) => s * v.length, 1); M['pairwise tests'] = rows.length; S.push(sec('Pairwise covering array', 'info', [`${rows.length} tests cover every pair of factor values (full factorial ${full})`], { table: { head: Object.keys(body.pairwise), rows: rows.slice(0, 40).map((r) => Object.keys(body.pairwise!).map((k) => r[k])) } })); }
    if (body.requirements) { const t = traceabilitySummary(body.requirements); S.push(sec('Traceability matrix', t.fail ? 'error' : t.open ? 'warn' : 'ok', [`${t.total} requirements: ${t.pass} pass, ${t.fail} fail, ${t.open} open, ${t.degenerate} degenerate; without formalisation: ${t.unformalised.join(', ') || 'none'}`], { table: { head: ['id', 'class', 'requirement', 'formal', 'verification', 'result'], rows: body.requirements.map((r) => [r.id, r.cls, r.text, r.formal ?? '', r.verification, r.result ?? 'open']) } })); ok = ok && !t.fail; }
    return { title: name || 'Acceptance', ok };
  },

  stl: (src, S, M) => {
    const doc = dsl.parseStl(src); let ok = true;
    for (const sp of doc.specs) {
      const phi = parseSTL(sp.formula);
      if (doc.signal) { const rho = robustness(phi, doc.signal); ok = ok && rho > 0; M[`ρ(${sp.name ?? formatSTL(phi)})`] = f(rho, 3); S.push(sec(`${sp.name ? sp.name + ': ' : ''}${formatSTL(phi)}`, rho > 0 ? 'ok' : 'error', [`robustness ρ = ${f(rho, 3)} on the ${doc.signal.t.length}-sample signal: ${rho > 0 ? `satisfied with margin ${f(rho, 3)} (any perturbation smaller than this keeps it true, Theorem 17.2)` : 'VIOLATED'}`])); }
      else S.push(sec(`${sp.name ? sp.name + ': ' : ''}${formatSTL(phi)}`, 'info', ['parsed; attach a signal table (`signal t …`) or run it as a monitor on a recorded mission']));
    }
    return { title: doc.name, ok };
  },
};
const ANALYSERS: Record<ControlKind, Analyser> = { ...CTL_ANALYSERS, ...(MRS_ANALYSERS as Record<string, Analyser>) } as Record<ControlKind, Analyser>;

function petriReport(parsed: { spec: import('./petri').PetriNetSpec; checks: Array<{ kind: 'ltl' | 'ctl'; formula: string; name?: string }>; horizon: number }, S: ReportSection[], M: Record<string, number | string | boolean>, title?: string): { title: string; ok: boolean } {
  const net = new PetriNet(parsed.spec); const a = analysePetriNet(net); let ok = a.live && a.bounded;
  M['reachable markings'] = a.reachable; M['live'] = a.live; M['bounded'] = a.bounded; M['deadlocks'] = a.deadlocks.length; M['bad siphons'] = a.badSiphons.length;
  const bound = (parsed as { actions?: dsl.ActionBinding[] }).actions ?? [];
  if (bound.length) { const bad = bound.filter((a) => !parsed.spec.transitions.some((t) => t.id === a.target)); S.push(sec(`Bound actions (${bound.length})`, bad.length ? 'warn' : 'info', [...bound.map((a) => `${a.target}: ${a.kind} ${a.value}${a.robot ? ` on ${a.robot}` : ''}`), ...bad.map((a) => `unknown transition ${a.target}`)])); }
  const graph: GraphView = { kind: 'petri', nodes: [...net.places.map((p) => ({ id: p.id, label: p.id, kind: p.kind ?? 'place', tokens: p.tokens ?? 0 })), ...net.transitions.map((t) => ({ id: t.id, label: t.label ? `${t.id}: ${t.label}` : t.id, kind: 'transition' }))], edges: parsed.spec.arcs.map((x) => ({ from: x.from, to: x.to, label: x.weight && x.weight > 1 ? String(x.weight) : x.inhibitor ? 'o' : undefined })) };
  S.push(sec('Structure', a.coveredByPInvariants ? 'ok' : 'warn', [`${net.placeCount} places, ${net.transitionCount} transitions, ${parsed.spec.arcs.length} arcs`, `P-invariants: ${a.pInvariants.join('; ') || 'none'}`, `T-invariants: ${a.tInvariants.join('; ') || 'none'}`, a.coveredByPInvariants ? 'every place is covered by a P-invariant → structurally bounded' : `places not covered by any P-invariant: ${a.uncovered.join(', ')}`], { graph }));
  S.push(sec('Behaviour (reachability graph)', a.live && !a.deadlocks.length ? 'ok' : 'error', [`${a.reachable} reachable markings${a.truncated ? ' (truncated)' : ''}; bounds ${Object.entries(a.bounds).map(([p, b]) => `${p}:${b}`).join(' ')}; ${a.safe ? 'safe (1-bounded)' : 'not safe'}`, a.live ? 'live: every transition can always fire again' : `NOT live: dead ${a.deadTransitions.join(', ') || '—'}; non-live ${a.nonLiveTransitions.join(', ') || '—'}`, a.reversible ? 'reversible (M0 always reachable)' : 'not reversible', ...a.deadlocks.map((d) => `DEADLOCK marking {${d.marking}} reached by ${d.trace.join(' ')}`), ...a.livelocks.map((l) => `LIVELOCK: terminal cycle without return to M0 through {${l.join('}, {')}}`)]));
  S.push(sec('Siphons and traps', a.badSiphons.length ? 'error' : 'ok', [`${a.siphons.length} minimal siphons, ${a.traps.length} minimal traps`, ...a.siphons.map((s) => `{${s.places.join(', ')}}: M0 = ${s.initialTokens}, ${s.trap ? `contains the marked trap {${s.trap.join(', ')}} → never empties (Commoner)` : s.emptyAt ? `EMPTIES at {${s.emptyAt}} after ${s.trace?.join(' ')} → deadlock cause` : 'never empties in the reachable markings'}`)]));
  if (a.badSiphons.length) {
    try {
      const p = preventDeadlocks(net); M['monitors added'] = p.monitors.length;
      S.push(sec('Deadlock prevention with GMEC monitors (Theorem 4.9)', p.live ? 'ok' : 'warn', [...p.monitors.map((m) => `monitor ${m.id}: ${m.constraint} (for siphon {${m.siphon.join(', ')}}) — a semaphore place with C_V = −lᵀC, M0(V) = β − lᵀM0`), p.live ? `the controlled net is live after ${p.iterations} iteration(s)` : 'monitors did not restore liveness within the iteration budget — consider the banker\'s algorithm or a resource ordering']));
    } catch (e) { S.push(sec('Deadlock prevention with GMEC monitors (Theorem 4.9)', 'warn', [`monitor synthesis not applicable: ${(e as Error).message}`])); }
    ok = false;
  }
  if (net.transitions.some((t) => t.delay)) { const sim = simulateTimed(net, parsed.horizon, { seed: 1 }); S.push(sec(`Timed simulation (${parsed.horizon} s, deterministic delays)`, sim.deadlockAt !== null ? 'error' : 'info', [sim.deadlockAt !== null ? `DEADLOCK at t = ${f(sim.deadlockAt, 1)} s` : sim.zeno ? 'stopped: zero-time firings without progress (a transition without input places or delay)' : 'no deadlock', `throughput: ${Object.entries(sim.throughput).map(([t, v]) => `${t} ${f(3600 * v, 1)}/h`).join(', ')}`, `busy fraction: ${net.places.filter((p) => p.kind === 'resource').map((p) => `${p.id} ${f(100 * (1 - sim.busy[p.id]), 0)} %`).join(', ') || '—'}`])); }
  if (net.transitions.some((t) => t.rate)) { try { const g = analyseGspn(net); S.push(sec('GSPN steady state (CTMC)', 'info', [`${g.tangible} tangible + ${g.vanishing} vanishing markings`, `throughput: ${Object.entries(g.throughput).map(([t, v]) => `${t} ${f(3600 * v, 1)}/h`).join(', ')}`, `P(place marked): ${Object.entries(g.utilisation).map(([p, v]) => `${p} ${f(100 * v, 1)} %`).join(', ')}`], { table: { head: ['marking', 'π'], rows: g.pi.slice(0, 20).map((x) => [x.marking, f(x.p, 4)]) } })); } catch (e) { S.push(sec('GSPN', 'warn', [(e as Error).message])); } }
  if (parsed.checks.length) ok = runChecks(kripkeFromPetri(net.reachability()), parsed.checks, S) && ok;
  return { title: title ?? parsed.spec.name, ok };
}

function treeGraph(root: BTNodeSpec): GraphView {
  const nodes: GraphView['nodes'] = []; const edges: GraphView['edges'] = []; let seq = 0;
  const walk = (n: BTNodeSpec, parent: string | null): void => { const id = `n${seq++}`; const label = n.type === 'action' || n.type === 'condition' ? `${n.type === 'action' ? '▭' : '◯'} ${n.name ?? n.fn}` : `${{ sequence: '→', fallback: '?', parallel: '⇉' }[n.type as 'sequence'] ?? '◇'} ${n.name ?? n.type}${n.memory ? '*' : ''}`; nodes.push({ id, label, kind: n.type }); if (parent) edges.push({ from: parent, to: id }); for (const c of n.children ?? []) walk(c, id); };
  walk(root, null);
  return { kind: 'tree', nodes, edges };
}

/** Automaton spec → DSL helper for exports. */
export function specsToDsl(plant: AutomatonSpec[], specs: AutomatonSpec[], uc: string[], uo: string[] = []): string {
  return [...plant.map((a) => dsl.automatonToDsl(a, 'automaton')), ...specs.map((a) => dsl.automatonToDsl(a, 'spec')), uc.length ? `uncontrollable ${uc.join(' ')}` : '', uo.length ? `unobservable ${uo.join(' ')}` : ''].filter(Boolean).join('\n');
}
