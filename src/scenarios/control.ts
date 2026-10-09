/**
 * Demo scenarios for the control-design (course) methods: every chapter has at least one reproducible case whose
 * numbers match the course text; `Load into the studio` puts the corresponding DSL documents into the station tree.
 */
import { Station, ItemType } from '../core/items/item';
import { MobileRobot, ZoneItem } from '../mobile/items';
import { analyse } from '../ctl/analysis';
import { addControlModel, ControlKind } from '../ctl/model';
import { COURSE_EXAMPLES, YOUBOT_BT_DSL, YOUBOT_DES_DSL, YOUBOT_DES_E4_DSL, YOUBOT_GR1_DSL, YOUBOT_PDDL_DSL, CELL_PETRI_DSL, CELL_SMV_DSL, CELL_SMV_FAIR_DSL } from '../ctl/examples_dsl';
import { TEMPLATES, parseBt, parseGr1 } from '../ctl/dsl';
import { parallel, supcon, supervisorTable } from '../ctl/des';
import { youbotPlant, youbotSpecs, YOUBOT_UC, YOUBOT_UO, YOUBOT_PDDL_DOMAIN, YOUBOT_PDDL_PROBLEM } from '../ctl/examples';
import { ControlRuntime, stationBindings } from '../ctl/runtime';
import { synthesizeGR1 } from '../ctl/gr1';
import { parseDomain, parseProblem, ground, executeWithReplanning } from '../ctl/planning';
import { falsify, robustness, parseSTL } from '../ctl/vv';
import { Signal } from '../ctl/vv';
import type { Scenario, ScenarioResult, ScenarioMetric } from './index';

const M = (v: number, d = 0) => Number(v.toFixed(d));
const metric = (name: string, value: number | string, unit?: string, ok?: boolean, bound?: string): ScenarioMetric => ({ name, value, unit, ok, bound });

function stationWith(docs: Array<{ kind: ControlKind; name: string; source: string }>, name: string): Station {
  const st = new Station(name);
  for (const d of docs) addControlModel(st, d.kind, d.name, d.source);
  return st;
}
const ex = (id: string) => COURSE_EXAMPLES.find((e) => e.id === id)!;

function scenario(def: { id: string; title: string; method: string; description: string; howTo: string; docs: Array<{ kind: ControlKind; name: string; source: string }>; /** Scene items (robots, zones) added to the station for both the headless run and "Load into the studio". */ setup?: (st: Station) => void; evaluate: (st: Station) => Promise<{ metrics: ScenarioMetric[]; notes?: string[] }> | { metrics: ScenarioMetric[]; notes?: string[] } }): Scenario {
  return {
    id: def.id, title: def.title, group: 'control', method: def.method, description: def.description, howTo: def.howTo,
    build: () => { const st = stationWith(def.docs, def.title); def.setup?.(st); return { station: st, focus: st.itemsOfType(111 as never)[0] }; },
    async run(): Promise<ScenarioResult> {
      const t0 = Date.now();
      const st = stationWith(def.docs, def.title); def.setup?.(st);
      const r = await def.evaluate(st);
      return { id: def.id, title: def.title, group: 'control', method: def.method, pass: r.metrics.every((m) => m.ok !== false), metrics: r.metrics, notes: r.notes ?? [], durationMs: Date.now() - t0 };
    },
  };
}

const HOWTO = 'Control › Course examples… → pick the document → Analyse (Control tab shows the report and the graph).';

export const CONTROL_SCENARIOS: Scenario[] = [
  scenario({
    id: 'ctl_des_supervisor', title: 'DES: youBot plant, supervisor synthesis, unrealisable E4 (chapters 2–3)', method: 'supcon',
    description: 'Five component automata compose to 648 states; E1 E2 E3 E5 give a 324-state non-blocking supervisor; E4 "never drop" is unrealisable because g_slip is uncontrollable; observer under unobservable g_slip.',
    howTo: HOWTO, docs: [ex('A-des'), ex('A-des-e4')],
    evaluate: () => {
      const a = analyse('des', YOUBOT_DES_DSL), b = analyse('des', YOUBOT_DES_E4_DSL);
      return { metrics: [metric('plant states', a.metrics['plant states'] as number, undefined, a.metrics['plant states'] === 648, '= 648'), metric('supervisor states', a.metrics['supervisor states'] as number, undefined, a.metrics['supervisor states'] === 324, '= 324'), metric('modular non-conflicting', String(a.metrics['modular nonconflicting']), undefined, a.metrics['modular nonconflicting'] === true, 'true'), metric('observer states', a.metrics['observer states'] as number, undefined, a.metrics['observer states'] === 324, '= 324'), metric('LTL/CTL checks', a.ok ? 'pass' : 'fail', undefined, a.ok, 'R1 holds under supervision'), metric('E4 realisable', String(b.metrics['realizable']), undefined, b.metrics['realizable'] === false, 'false (g_slip uncontrollable)'), metric('analysis time', a.durationMs + b.durationMs, 'ms')] };
    },
  }),
  scenario({
    id: 'ctl_petri_deadlock', title: 'Petri net: siphon → deadlock → GMEC monitor (chapter 4)', method: 'siphon_monitor',
    description: 'Two processes acquire the robot and the zone in opposite orders. Reachability finds the deadlock; the siphon {rM, rZ, a2, b2} empties; one monitor place (a1 + b1 ≤ 1) restores liveness; the S³PR builder reproduces the same net.',
    howTo: HOWTO, docs: [ex('B-petri'), ex('B-s3pr'), ex('B-perf')],
    evaluate: () => {
      const a = analyse('petri', CELL_PETRI_DSL); const p = analyse('perf', ex('B-perf').source); const s = analyse('s3pr', ex('B-s3pr').source);
      return { metrics: [metric('deadlock markings', a.metrics['deadlocks'] as number, undefined, a.metrics['deadlocks'] === 1, '= 1'), metric('bad siphons', a.metrics['bad siphons'] as number, undefined, (a.metrics['bad siphons'] as number) >= 1, '≥ 1'), metric('monitors added', a.metrics['monitors added'] as number, undefined, a.metrics['monitors added'] === 1, '= 1 (net live afterwards)'), metric('S3PR bad siphons', s.metrics['bad siphons'] as number, undefined, (s.metrics['bad siphons'] as number) >= 1, '≥ 1'), metric('bottleneck', String(p.metrics['bottleneck']), undefined, p.metrics['bottleneck'] === 'S', 'machine S'), metric('cycle time bound', p.metrics['cycle time bound (s)'] as string, 's', p.metrics['cycle time bound (s)'] === '22', '22 s'), metric('max-plus cycle', p.metrics['max-plus cycle time (s)'] as string, 's', p.metrics['max-plus cycle time (s)'] === '32', '32 s')] };
    },
  }),
  scenario({
    id: 'ctl_model_checking', title: 'Model checking: starvation under static priority, fixed by alternation (chapter 7)', method: 'ltl_ctl',
    description: 'Synchronous model of two robots and a shared zone. Mutual exclusion holds; "no starvation r2" fails with a cyclic counterexample; the alternation fix passes all specifications; CTL premises are checked for non-degeneracy.',
    howTo: HOWTO, docs: [ex('B-smv'), ex('B-smv-fair')],
    evaluate: () => {
      const a = analyse('smv', CELL_SMV_DSL), b = analyse('smv', CELL_SMV_FAIR_DSL);
      const starve = a.sections.find((s) => /no starvation r2/.test(s.title)); const mutex = a.sections.find((s) => /mutual exclusion/.test(s.title));
      return { metrics: [metric('states', a.metrics['states'] as number), metric('mutual exclusion', mutex?.level === 'ok' ? 'holds' : 'violated', undefined, mutex?.level === 'ok', 'holds'), metric('no starvation r2 (priority)', starve?.level === 'error' ? 'violated' : 'holds', undefined, starve?.level === 'error', 'violated with a lasso counterexample'), metric('counterexample', starve?.lines[0]?.includes('repeat') ? 'lasso' : 'none', undefined, !!starve?.lines[0]?.includes('repeat'), 'prefix + cycle'), metric('alternation model', b.ok ? 'all specs pass' : 'fails', undefined, b.ok, 'all pass')] };
    },
  }),
  scenario({
    id: 'ctl_gr1_synthesis', title: 'GR(1) synthesis of the mission controller (chapter 8)', method: 'gr1',
    description: '160-state game (5 zones × holding × 4 environment bits). Realisable with GF ¬human_present; dropping the assumption makes it unrealisable and the environment counter-strategy shows why.',
    howTo: HOWTO, docs: [ex('A-gr1')],
    evaluate: () => {
      const a = analyse('gr1', YOUBOT_GR1_DSL);
      const spec = parseGr1(YOUBOT_GR1_DSL); spec.envLive = spec.envLive!.filter((x) => !/human/.test(x)); const bad = synthesizeGR1(spec);
      return { metrics: [metric('game states', a.metrics['game states'] as number, undefined, a.metrics['game states'] === 160, '= 160'), metric('realisable', String(a.metrics['realizable']), undefined, a.metrics['realizable'] === true, 'true'), metric('controller states', a.metrics['controller states'] as number, undefined, (a.metrics['controller states'] as number) > 5, '> 5'), metric('without GF ¬human', bad.realizable ? 'realisable' : 'unrealisable', undefined, !bad.realizable, 'unrealisable'), metric('counter-strategy steps', bad.counterStrategy.length, undefined, bad.counterStrategy.length > 0, '≥ 1')] };
    },
  }),
  scenario({
    id: 'ctl_mission_runtime', title: 'Mission runtime: behavior tree under the supervisor with monitors, on a station robot (chapters 3, 6, 9)', method: 'bt_supervisor_monitor',
    description: 'A mobile robot with zones home / table / bin / dock runs the course mission tree. Every controllable event is gated by the synthesised supervisor, R2 is monitored at runtime; an unsafe tree (move with the arm extended) is denied.',
    howTo: 'Load the scenario, then Control › Run mission (or the ▶ button in the Control tab) with the youBot supervisor selected; watch the log for denials and monitor verdicts.',
    docs: [ex('A-bt'), ex('A-des')],
    setup: (st) => {
      const r = st.addChild(new MobileRobot('youBot')); r.setPose2D(0, 0, 0);
      const mk = (name: string, x: number, y: number) => { const z = st.addChild(new ZoneItem(name)); z.polygon = [[x - 1000, y - 1000], [x + 1000, y - 1000], [x + 1000, y + 1000], [x - 1000, y + 1000]]; return z; };
      mk('home', 0, 0); mk('table', 8000, 0); mk('bin', 8000, 6000); mk('dock', 0, 6000);
    },
    evaluate: (st) => {
      const r = st.itemsOfType<MobileRobot>(ItemType.MOBILE_ROBOT)[0];
      const g = parallel(...youbotPlant(true)); const S = youbotSpecs(); const sup = supervisorTable(supcon(g, [S.E1, S.E2, S.E3, S.E5], YOUBOT_UC).supervisor, YOUBOT_UC, YOUBOT_UO, 5);
      const w = stationBindings({ robot: r, station: st, durations: { grasp: 1, place: 1, reach: 0.5, stow: 0.5 } });
      const rt = new ControlRuntime({ bt: parseBt(YOUBOT_BT_DSL), supervisor: sup, bindings: { ...w, step: (dt) => w.stepRobot(dt) } });
      rt.bb.targets = 1;
      rt.run(200, 0.1, (bb) => bb.reported === true);
      const s = rt.summary();
      const unsafe = new ControlRuntime({ bt: parseBt('bt unsafe\nsequence root memory\n  action goto zone=table event=b_go_table arrive=b_arrive timeout=60\n  action reach event=a_reach done=a_reached timeout=10\n  action goto zone=bin event=b_go_box arrive=b_arrive timeout=60'), supervisor: sup, bindings: (() => { const r2 = st.addChild(new MobileRobot('youBot 2')); r2.setPose2D(0, -3000, 0); const w2 = stationBindings({ robot: r2, station: st, durations: { reach: 0.5 } }); return { ...w2, step: (dt: number) => w2.stepRobot(dt) }; })() });
      unsafe.run(60, 0.1, () => unsafe.status !== 'running');
      const rho = robustness(parseSTL('G (held -> F[0,40] !held)'), rt.trace);
      return { metrics: [metric('mission reported', rt.bb.reported ? 'yes' : 'no', undefined, rt.bb.reported === true, 'yes'), metric('final plant state', s.plantState ?? '', undefined, /^H,S,O/.test(s.plantState ?? ''), 'H,S,O,… (home, arm stowed, gripper open)'), metric('supervisor denials', s.denied, undefined, s.denied === 0, '= 0 for the course tree'), metric('model mismatches', s.supervisorErrors, undefined, s.supervisorErrors === 0, '= 0'), metric('monitor violations', s.violations, undefined, s.violations === 0, '= 0'), metric('STL ρ(held → F≤40 !held)', M(rho, 2), 's', rho > 0, '> 0'), metric('unsafe tree denied', unsafe.denied, undefined, unsafe.denied >= 1 && unsafe.status === 'failure', '≥ 1 (b_go_box with the arm extended)'), metric('sim time', M(s.time, 1), 's')], notes: rt.log.slice(0, 5) };
    },
  }),
  scenario({
    id: 'ctl_planning', title: 'Task planning: PDDL (GBFS / A*), HTN, plan–execute–replan (chapters 10–11)', method: 'pddl_htn',
    description: 'Course domain of example A (3 parts, 2 tables, 2 bins): GBFS/h_FF plan validated, A*/h_max optimal, HTN methods with recursion; the execution loop replans when a pick fails because a part is blocked.',
    howTo: HOWTO, docs: [ex('A-pddl'), ex('B-stn')],
    evaluate: async () => {
      const a = analyse('pddl', YOUBOT_PDDL_DSL);
      const t = ground(parseDomain(YOUBOT_PDDL_DOMAIN), parseProblem(YOUBOT_PDDL_PROBLEM));
      let failed = false; const r = await executeWithReplanning(t, { execute: (act) => { if (act.name === 'pick bolt1 table1' && !failed) { failed = true; return false; } return true; }, observe: (s) => { if (failed && s.has(t.atomIndex.get('on(bolt2,table1)')!)) { const n = new Set(s); n.add(t.atomIndex.get('blocked(bolt1,bolt2)')!); return n; } return s; } });
      const stn = analyse('stn', ex('B-stn').source);
      return { metrics: [metric('ground actions', a.metrics['ground actions'] as number), metric('plan found', String(a.metrics['plan found']), undefined, a.metrics['plan found'] === true, 'true'), metric('plan length', a.metrics['plan length'] as number), metric('HTN plan', String(a.metrics['HTN plan found']), undefined, a.metrics['HTN plan found'] === true, 'true'), metric('replans on failure', r.replans, undefined, r.success && r.replans === 1, '= 1 and mission completed'), metric('STNU controllable', String(stn.metrics['dynamically controllable']), undefined, stn.metrics['dynamically controllable'] === true, 'true')] };
    },
  }),
  scenario({
    id: 'ctl_decisions', title: 'Decisions under uncertainty: grasp MDP and classification POMDP (chapter 12)', method: 'mdp_pomdp',
    description: 'Value / policy iteration reproduce V(s0) = 12.94 s (refine first) and the 0.70 switch point; the POMDP shows that a 0.72 belief is not enough to place (look closer: 11.07 vs 0.26) and derives the 0.943 threshold.',
    howTo: HOWTO, docs: [ex('A-mdp'), ex('A-pomdp')],
    evaluate: () => {
      const a = analyse('mdp', ex('A-mdp').source), b = analyse('pomdp', ex('A-pomdp').source);
      const row = a.sections[0].table!.rows.find((r) => r[0] === 's0')!;
      return { metrics: [metric('V*(s0)', row[1], 's', row[1] === '12.943', '12.943 s'), metric('π*(s0)', row[2], undefined, row[2] === 'refine', 'refine'), metric('POMDP best action at b0', String(b.metrics['best action']), undefined, ['look', 'look_closer'].includes(String(b.metrics['best action'])), 'an information action'), metric('α-vectors', b.metrics['α-vectors'] as number)] };
    },
  }),
  scenario({
    id: 'ctl_coordination', title: 'Coordination: assignment, auction, MAPF with CBS and TPG (chapter 13)', method: 'hungarian_cbs',
    description: 'Three AMRs / three pallets: Hungarian 81 s vs greedy; minimax 55 s; corridor with a pocket: priority planning fails for one order, CBS finds the optimum, the temporal plan graph stays collision-free under delays.',
    howTo: HOWTO, docs: [ex('B-mrta'), ex('B-mapf')],
    evaluate: () => {
      const a = analyse('mrta', ex('B-mrta').source), b = analyse('mapf', ex('B-mapf').source);
      return { metrics: [metric('Hungarian total', a.metrics['optimal sum'] as number, 's', a.metrics['optimal sum'] === 81, '81 s'), metric('minimax makespan', a.metrics['minimax'] as number, 's', a.metrics['minimax'] === 55, '55 s'), metric('CBBA converged', a.metrics['CBBA total'] !== undefined ? 'yes' : 'no', undefined, a.metrics['CBBA total'] !== undefined, 'yes'), metric('priority planning failures', b.metrics['priority failed'] as number, undefined, b.metrics['priority failed'] === 1, '= 1'), metric('CBS found', String(b.metrics['CBS found']), undefined, b.metrics['CBS found'] === true, 'true'), metric('TPG acyclic', String(b.metrics['TPG acyclic']), undefined, b.metrics['TPG acyclic'] === true, 'true')] };
    },
  }),
  scenario({
    id: 'ctl_scheduling', title: 'Job-shop scheduling of the cell: 54 s against a 44 s bound (chapter 14)', method: 'jobshop',
    description: 'Dispatch rules, branch and bound and tabu search on the three-job FJSP instance; the 22.7 % gap to the lower bound is structural; robustness under duration noise.',
    howTo: HOWTO, docs: [ex('B-jobshop')],
    evaluate: () => { const a = analyse('jobshop', ex('B-jobshop').source); return { metrics: [metric('lower bound', a.metrics['lower bound'] as number, 's', a.metrics['lower bound'] === 44, '44 s'), metric('best makespan', a.metrics['best makespan'] as number, 's', a.metrics['best makespan'] === 54, '54 s'), metric('optimal', String(a.metrics['optimal']), undefined, a.metrics['optimal'] === true, 'true'), metric('gap', a.metrics['gap %'] as string, '%')] }; },
  }),
  scenario({
    id: 'ctl_realtime', title: 'Real-time analysis: response times and the visual-servoing latency budget (chapter 15)', method: 'rta_e2e',
    description: 'Four onboard tasks: U = 0.80 above the Liu–Layland bound but schedulable by exact analysis (R = 1.2 / 11.6 / 43.8 / 93.8 ms); the 313 ms asynchronous chain costs 47 mm at 0.15 m/s — an architectural problem, not an algorithmic one.',
    howTo: HOWTO, docs: [ex('A-realtime')],
    evaluate: () => { const a = analyse('realtime', ex('A-realtime').source); const rta = a.sections[0].table!.rows.map((r) => r[5]); return { metrics: [metric('utilisation', a.metrics['utilisation'] as string, undefined, a.metrics['utilisation'] === '0.8', '0.80'), metric('RM schedulable', String(a.metrics['RM schedulable']), undefined, a.metrics['RM schedulable'] === true, 'true (exact test)'), metric('response times', rta.join(' / '), 'ms', rta.join('/') === '1.2/11.6/43.8/93.8', '1.2 / 11.6 / 43.8 / 93.8'), metric('latency budget', a.ok ? 'ok' : 'exceeded', undefined, !a.ok, 'exceeded at 0.15 m/s (47 mm > 15 mm)')] }; },
  }),
  scenario({
    id: 'ctl_safety', title: 'Reliability and functional safety: MTTF, FTA, ISO 13849 PL, ISO/TS 15066 SSM (chapter 16)', method: 'reliability_safety',
    description: 'Cell budget MTTF 633 h and A = 0.9937 with the machine at 31.6 %; contact fault tree with cut sets of order 4 (P ≈ 1.1e-12); PLr e / PL d with category 3; protective distance 1.54 m and 0.31 m/s in a 1 m aisle; FMEA action priorities.',
    howTo: HOWTO, docs: [ex('B-reliability'), ex('A-fta'), { kind: 'fmea', name: 'A · FMEA', source: TEMPLATES.fmea }],
    evaluate: () => {
      const a = analyse('reliability', ex('B-reliability').source), b = analyse('fta', ex('A-fta').source), c = analyse('fmea', TEMPLATES.fmea);
      return { metrics: [metric('MTTF', a.metrics['MTTF (h)'] as string, 'h', a.metrics['MTTF (h)'] === '633', '633 h'), metric('availability', a.metrics['availability'] as string, undefined, a.metrics['availability'] === '0.9937', '0.9937'), metric('S_p', a.metrics['S_p (m)'] as string, 'm', a.metrics['S_p (m)'] === '1.54', '1.54 m'), metric('admissible speed (1 m)', a.metrics['admissible speed (m/s)'] as string, 'm/s', a.metrics['admissible speed (m/s)'] === '0.311', '0.311 m/s'), metric('PL', String(a.metrics['PL']), undefined, a.metrics['PL'] === 'd', 'd'), metric('FTA min cut order', b.metrics['min order'] as number, undefined, b.metrics['min order'] === 4, '4'), metric('P(top)', String(b.metrics['P(top) rare-event']), undefined, String(b.metrics['P(top) rare-event']).startsWith('1.10e-12'), '≈ 1.1e-12'), metric('FMEA mandatory rows', c.metrics['mandatory (S ≥ 9)'] as number, undefined, c.metrics['mandatory (S ≥ 9)'] === 1, '1 (S = 9)')] };
    },
  }),
  scenario({
    id: 'ctl_vv', title: 'V&V: STL falsification beats random search; acceptance statistics (chapter 17)', method: 'stl_falsification',
    description: 'A human walking towards the robot path: optimisation of the STL robustness finds a violating scenario (ρ < 0) that random search with the same budget misses or barely reaches; rule of three, Clopper–Pearson and the sim-to-real gap are computed for the acceptance report.',
    howTo: HOWTO + ' STL documents accept a `signal` table; falsification runs from the API / scenarios with a simulation callback.', docs: [ex('A-stl'), ex('B-acceptance')],
    evaluate: () => {
      const simulate = (p: Record<string, number>): Signal => { const t: number[] = [], values: Record<string, number | boolean>[] = []; for (let k = 0; k <= 60; k++) { const rx = k < 30 ? k : 30, ry = k < 30 ? 0 : k - 30; const hx = p.px + p.v * Math.cos(p.a) * k, hy = p.py + p.v * Math.sin(p.a) * k; const dist = Math.hypot(rx - hx, ry - hy); t.push(k); values.push({ dist, human: dist < 3 }); } return { t, values }; };
      const f = falsify('G[0,60] (human -> dist >= 0.8)', { px: [0, 40], py: [3, 8], v: [0.5, 1.6], a: [-Math.PI, Math.PI] }, simulate, { budget: 300, seed: 11 });
      const acc = analyse('acceptance', ex('B-acceptance').source); const stl = analyse('stl', ex('A-stl').source);
      return { metrics: [metric('min robustness (optimised)', M(f.minRobustness, 3), 'm', f.falsified, '< 0 (violation found)'), metric('min robustness (random)', M(f.randomBaseline, 3), 'm', f.randomBaseline >= f.minRobustness, '≥ optimised'), metric('evaluations', f.evaluations), metric('signal ρ(R6)', String(Object.values(stl.metrics)[0]), 'm', stl.ok, '> 0: satisfied with a 0.1 m margin on the sample signal'), metric('97/100 demonstrates 0.95', acc.metrics['lower bound'] as string, undefined, Number(acc.metrics['lower bound']) < 0.95, 'no (lower bound < 0.95)'), metric('pairwise tests', acc.metrics['pairwise tests'] as number, undefined, (acc.metrics['pairwise tests'] as number) <= 30, '≤ 30 of 240')] };
    },
  }),
  scenario({
    id: 'ctl_modes_cbf', title: 'Hybrid modes with hysteresis and the CBF safety filter (chapter 9)', method: 'hybrid_cbf',
    description: 'The mode automaton with 2.0 / 2.5 m hysteresis and 0.4 s dwell passes the chattering check; a variant without hysteresis is flagged; the CBF filter reduces a 0.8 m/s outward command to 0.414 m/s at 2.8 m from the zone centre.',
    howTo: HOWTO, docs: [ex('A-modes')],
    evaluate: async () => {
      const a = analyse('hybrid', ex('A-modes').source);
      const bad = analyse('hybrid', 'hybrid chatter\ninitial NORMAL\nmode NORMAL\nmode SLOW\nNORMAL -> SLOW when d < 2\nSLOW -> NORMAL when d >= 2');
      const { cbfFilter, zoneConstraint } = await import('../ctl/hybrid');
      const u = cbfFilter([0.8, 0], [zoneConstraint([2.8, 0], [0, 0], 3, 2)]).u[0];
      return { metrics: [metric('course automaton', a.ok ? 'ok' : 'issues', undefined, a.ok, 'no chattering'), metric('chattering pairs (no hysteresis)', bad.metrics['chattering pairs'] as number, undefined, bad.metrics['chattering pairs'] === 1, '= 1'), metric('CBF radial speed', M(u, 3), 'm/s', Math.abs(u - 0.414) < 0.002, '0.414 m/s')] };
    },
  }),
];
