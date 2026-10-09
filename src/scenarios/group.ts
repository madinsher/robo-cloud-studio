/**
 * Demo scenarios of the group-control module (course *control of distributed robotic systems*): one reproducible
 * case per practicum (ПР1–ПР6), the warehouse homework and the chapter examples. Each evaluates the course documents
 * headlessly and compares the numbers with the text; `Load into the studio` puts the documents (and, for the
 * runnable ones, the robots) into the station so that **Run on fleet** shows them in the 3D view.
 */
import { Station, ItemType } from '../core/items/item';
import { MobileRobot } from '../mobile/items';
import { analyse } from '../ctl/analysis';
import { addControlModel, ControlKind } from '../ctl/model';
import { MRS_EXAMPLES, MrsExample } from '../mrs/examples';
import { parseWarehouse } from '../mrs/dsl';
import { buildWarehouseScene, FleetRuntime } from '../mrs/runtime';
import { ZoneItem } from '../mobile/items';
import type { Scenario, ScenarioResult, ScenarioMetric } from './index';

const M = (v: number, d = 0) => Number(v.toFixed(d));
const metric = (name: string, value: number | string, unit?: string, ok?: boolean, bound?: string): ScenarioMetric => ({ name, value, unit, ok, bound });
const ex = (id: string): MrsExample => MRS_EXAMPLES.find((e) => e.id === id)!;
const num = (v: unknown) => (typeof v === 'number' ? v : NaN);

function scenario(def: { id: string; title: string; method: string; description: string; howTo?: string; docs: MrsExample[]; setup?: (st: Station) => void; evaluate: (st: Station) => { metrics: ScenarioMetric[]; notes?: string[] } }): Scenario {
  const build = () => { const st = new Station(def.title); for (const d of def.docs) addControlModel(st, d.kind as ControlKind, d.name, d.source); def.setup?.(st); return st; };
  return {
    id: def.id, title: def.title, group: 'group', method: def.method, description: def.description, howTo: def.howTo ?? 'Group › Course examples… → pick the document → Analyse (Control tab shows the report, the charts and the graph); runnable kinds: ▶ Run on fleet.',
    build: () => { const st = build(); return { station: st, focus: st.itemsOfType(111 as never)[0] }; },
    async run(): Promise<ScenarioResult> { const t0 = Date.now(); const st = build(); const r = def.evaluate(st); return { id: def.id, title: def.title, group: 'group', method: def.method, pass: r.metrics.every((m) => m.ok !== false), metrics: r.metrics, notes: r.notes ?? [], durationMs: Date.now() - t0 }; },
  };
}
const A = (e: MrsExample) => analyse(e.kind as ControlKind, e.source);
/** Robots for the runnable scenarios: a small grid around the origin. */
const robots = (n: number, spread = 1500) => (st: Station) => { for (let i = 0; i < n; i++) { const r = st.addChild(new MobileRobot(`r${i + 1}`)); r.setPose2D((i % 3) * spread, Math.floor(i / 3) * spread, 0); } };

export const GROUP_SCENARIOS: Scenario[] = [
  scenario({
    id: 'grp_pr1_consensus', title: 'ПР1: Laplacian, consensus iterations, formation, rendezvous, W-MSR (chapters 4, 16)', method: 'consensus / formation / W-MSR', docs: [ex('pr1-chain'), ex('pr1-formation'), ex('pr1-rendezvous'), ex('pr1-wmsr'), ex('pr1-sparse'), ex('pr1-event-transport')], setup: robots(6),
    description: 'Chain P₄ with x(0) = (0, 4, 8, 12), ε = 0.25 reproduces the text (λ₂ = 0.586, ρ = 0.854, ≈ 30 iterations to 1 %); a ring formation assembles to < 1 mm; the connectivity-preserving rendezvous never breaks an initial link; W-MSR with F = 1 on K₇ survives a ±100 liar (3-robust) while the path graph (1-robust) does not.',
    evaluate: () => { const c = A(ex('pr1-chain')), fm = A(ex('pr1-formation')), rz = A(ex('pr1-rendezvous')), w = A(ex('pr1-wmsr')), sp = A(ex('pr1-sparse')); return { metrics: [metric('λ₂ of P₄', num(c.metrics['lambda2']), undefined, Math.abs(num(c.metrics['lambda2']) - 0.5858) < 1e-3, '= 2 − √2'), metric('ρ at ε = 0.25', num(c.metrics['rho']), undefined, Math.abs(num(c.metrics['rho']) - 0.8536) < 1e-3, '= 0.854'), metric('iterations to 1 %', num(c.metrics['iterations to 1%']), undefined, c.metrics['iterations to 1%'] === 30, '≈ 29–30'), metric('formation error', num(fm.metrics['formation error']), 'm', num(fm.metrics['formation error']) < 1e-3, '< 1 mm'), metric('links broken (rendezvous)', num(rz.metrics['links broken']), undefined, rz.metrics['links broken'] === 0, '= 0'), metric('W-MSR on K₇', w.ok ? 'converges' : 'fails', undefined, w.ok, 'normal agents inside [0, 0.9]'), metric('W-MSR on P₇', sp.ok ? 'converges' : 'hijacked', undefined, !sp.ok, '1-robust chain fails'), metric('analysis time', c.durationMs + fm.durationMs + rz.durationMs + w.durationMs, 'ms')] }; },
  }),
  scenario({
    id: 'grp_pr2_swarm', title: 'ПР2: Reynolds flock, Vicsek transition, PSO and robot source seeking (chapter 5)', method: 'boids / vicsek / pso', docs: [ex('pr2-boids'), ex('pr2-vicsek'), ex('pr2-pso'), ex('pr2-robots'), ex('pr2-aco')], setup: robots(9, 400),
    description: 'Twenty boids align (φ > 0.9) without collisions; the Vicsek order parameter is > 0.8 at low noise and < 0.3 at high noise; lbest PSO finds the Gaussian maximum to 0.02; eight robots with a 3 cm step, repulsion, noise and a distractor reach the source within 0.3 m; ACO beats the nearest-neighbour route.',
    evaluate: () => { const b = A(ex('pr2-boids')), v = A(ex('pr2-vicsek')), p = A(ex('pr2-pso')), r = A(ex('pr2-robots')), a = A(ex('pr2-aco')); return { metrics: [metric('flock polarization', num(b.metrics['polarization']), undefined, num(b.metrics['polarization']) > 0.9, '> 0.9'), metric('flock min distance', num(b.metrics['min distance']), 'm', num(b.metrics['min distance']) > 0.075, '> r_sep / 2'), metric('Vicsek order, η = 0.3', num(v.metrics['order low noise']), undefined, num(v.metrics['order low noise']) > 0.8, '> 0.8'), metric('Vicsek order, η = 6', num(v.metrics['order high noise']), undefined, num(v.metrics['order high noise']) < 0.3, '< 0.3'), metric('PSO source error', num(p.metrics['source error']), 'm', num(p.metrics['source error']) < 0.02, '< 0.02'), metric('robot swarm error', num(r.metrics['source error']), 'm', num(r.metrics['source error']) < 0.3, '< 0.3'), metric('ACO vs greedy', `${a.metrics['tour length']} / ${a.metrics['greedy length']}`, undefined, a.ok, 'ACO ≤ greedy')] }; },
  }),
  scenario({
    id: 'grp_pr3_allocation', title: 'ПР3: greedy vs Hungarian, SSI auction, CBBA, Vickrey (chapters 9–10)', method: 'hungarian / ssi / cbba', docs: [ex('pr3-course-matrix'), ex('pr3-ssi-cbba'), ex('pr3-line')],
    description: 'The greedy assignment pays for its first cheap pair; SSI assigns every task once within 2× of the exhaustive optimum; CBBA converges conflict-free on a complete graph and on a chain; second-price payments and contract-net message counts.',
    evaluate: () => { const m = A(ex('pr3-course-matrix')), s = A(ex('pr3-ssi-cbba')), l = A(ex('pr3-line')); return { metrics: [metric('greedy / optimal (course matrix)', num(m.metrics['greedy / optimal']), undefined, num(m.metrics['greedy / optimal']) > 1.2, '> 1'), metric('SSI rounds', num(s.metrics['ssi rounds']), undefined, s.metrics['ssi rounds'] === 9, '= tasks'), metric('SSI / exhaustive optimum (course matrix)', num(m.metrics['ssi / optimum']), undefined, num(m.metrics['ssi / optimum']) <= 2, '≤ 2'), metric('CBBA conflict-free (K₃)', String(s.metrics['cbba conflict-free']), undefined, s.metrics['cbba conflict-free'] === true, 'true'), metric('CBBA assigned', `${s.metrics['cbba assigned']}/9`, undefined, s.metrics['cbba assigned'] === 9, 'all'), metric('CBBA conflict-free (chain)', String(l.metrics['cbba conflict-free']), undefined, l.metrics['cbba conflict-free'] === true, 'true'), metric('CBBA iterations (chain)', num(l.metrics['cbba iterations']), undefined, num(l.metrics['cbba iterations']) < 200, '< 200')] }; },
  }),
  scenario({
    id: 'grp_pr4_mapf', title: 'ПР4: space-time A*, prioritized planning, CBS and ADG execution with delays (chapter 11)', method: 'cbs / adg', docs: [ex('pr4-warehouse'), ex('pr4-incomplete'), ex('pr4-random')], setup: robots(4),
    description: 'Four robots cross the shelf grid: prioritized planning and CBS both give the optimal sum of costs; in the corridor with a pocket prioritized planning fails and CBS succeeds; executing the plans through the action dependency graph with 30 % random delays never collides, clock-driven execution does.',
    evaluate: () => { const w = A(ex('pr4-warehouse')), i = A(ex('pr4-incomplete')), r = A(ex('pr4-random')); return { metrics: [metric('CBS sum of costs (crossing)', num(w.metrics['cbs SOC']), undefined, num(w.metrics['cbs SOC']) <= num(w.metrics['prioritized SOC']), '≤ prioritized'), metric('ADG collisions (crossing)', num(w.metrics['adg collisions']), undefined, w.metrics['adg collisions'] === 0, '= 0'), metric('naive collisions (crossing)', num(w.metrics['naive collisions'])), metric('prioritized (pocket)', String(i.metrics['prioritized SOC']), undefined, i.metrics['prioritized SOC'] === 'fail', 'fail (incomplete)'), metric('CBS (pocket)', num(i.metrics['cbs SOC']), undefined, num(i.metrics['cbs SOC']) > 0, 'solved'), metric('ADG collisions (6 random)', num(r.metrics['adg collisions']), undefined, r.metrics['adg collisions'] === 0, '= 0')] }; },
  }),
  scenario({
    id: 'grp_pr5_coverage', title: 'ПР5: Voronoi / Lloyd coverage, limited range, information consensus, covariance intersection (chapters 4, 11)', method: 'lloyd / information consensus', docs: [ex('pr5-lloyd'), ex('pr5-limited'), ex('pr5-estimation')], setup: robots(6, 500),
    description: 'Six robots start in a corner and gather around the hot spot with a monotone coverage functional; limited-range Lloyd spreads eight robots; every robot\'s information-form estimate converges to the centralised one while plain averaging is worse; CI picks ω = 0.5.',
    evaluate: () => { const l = A(ex('pr5-lloyd')), r = A(ex('pr5-limited')), e = A(ex('pr5-estimation')); return { metrics: [metric('H reduction', num(l.metrics['reduction %']), '%', num(l.metrics['reduction %']) > 70, '> 70 %'), metric('H monotone', String(l.metrics['monotone']), undefined, l.metrics['monotone'] === true, 'true'), metric('limited-range verdict', r.ok ? 'spread' : 'clustered', undefined, r.ok, 'robots spread'), metric('deviation from centralised', num(e.metrics['final deviation']), 'm', num(e.metrics['final deviation']) < 1e-4, '< 1e-4'), metric('naive vs centralised error', `${e.metrics['naive error']} vs ${e.metrics['centralised error']}`, 'm', num(e.metrics['naive error']) >= num(e.metrics['centralised error']), 'naive ≥ centralised'), metric('CI ω', num(e.metrics['ci omega']), undefined, e.metrics['ci omega'] === 0.5, '= 0.5')] }; },
  }),
  scenario({
    id: 'grp_pr6_safety', title: 'ПР6: barrier-function filter, antipodal deadlock and the keep-right rule, stale data (chapters 13, 16)', method: 'cbf / unstuck / stale', docs: [ex('pr6-crossing'), ex('pr6-antipodal'), ex('pr6-stale')], setup: robots(8, 800),
    description: 'Two crossing robots keep 0.2 m where the nominal law collides; eight robots swapping across a circle deadlock with the plain filter and all arrive with the hysteresis rule; a robot facing an uncooperative neighbour known with 0.5 s delay keeps the enlarged margin.',
    evaluate: () => { const c = A(ex('pr6-crossing')), a = A(ex('pr6-antipodal')), s = A(ex('pr6-stale')); return { metrics: [metric('crossing min distance', num(c.metrics['min distance']), 'm', num(c.metrics['min distance']) >= 0.197, '≥ d_safe'), metric('nominal min distance', num(c.metrics['nominal min distance']), 'm', num(c.metrics['nominal min distance']) < 0.2, '< d_safe without the filter'), metric('deadlock without the rule', String(a.metrics['deadlock without rule']), undefined, a.metrics['deadlock without rule'] === true, 'true'), metric('arrival with the rule', num(a.metrics['arrival time']), 's', a.metrics['deadlock'] === false, 'all goals reached'), metric('antipodal min distance', num(a.metrics['min distance']), 'm', num(a.metrics['min distance']) >= 0.197, '≥ d_safe'), metric('stale-data min distance', num(s.metrics['min distance']), 'm', num(s.metrics['min distance']) >= 0.297, '≥ 0.3')] }; },
  }),
  scenario({
    id: 'grp_hw_warehouse', title: 'Homework: multi-robot warehouse — CBBA orders, executor FSM, cell reservation, faults (stages 1–4)', method: 'cbba / traffic / fleet sim', docs: [ex('hw-baseline'), ex('hw-radio'), ex('hw-faults')],
    setup: (st) => { const d = parseWarehouse(ex('hw-baseline').source); buildWarehouseScene(st, d.cfg, d.stationKinds); },
    description: 'Four robots serve a stream of orders on the homework map: 600 s baseline with no double commits and no path conflicts; a poor radio (2.5 m, 30 % loss) produces conflicting claims; a robot that stops at 120 s loses its orders after lost_after and the protective layer keeps the others away until it recovers.',
    evaluate: (st) => { const b = A(ex('hw-baseline')), r = A(ex('hw-radio')), f = A(ex('hw-faults')); const robots = st.itemsOfType<MobileRobot>(ItemType.MOBILE_ROBOT); const rt = new FleetRuntime({ kind: 'warehouse', source: ex('hw-baseline').source.replace('duration 600', 'duration 90'), robots, station: st }); for (let t = 0; t < 95 && !rt.done; t += 0.1) rt.tick(0.1); return { metrics: [metric('delivered (600 s)', num(b.metrics['delivered']), undefined, num(b.metrics['delivered']) >= 15, '≥ 15'), metric('throughput', num(b.metrics['throughput /h']), 'orders/h'), metric('mean latency', num(b.metrics['mean latency s']), 's'), metric('double commits (baseline)', num(b.metrics['double commits']), undefined, b.metrics['double commits'] === 0, '= 0'), metric('path conflicts (baseline)', num(b.metrics['path conflicts']), undefined, b.metrics['path conflicts'] === 0, '= 0'), metric('delivered (poor radio)', num(r.metrics['delivered']), undefined, num(r.metrics['delivered']) > 0, '> 0'), metric('path conflicts (fault)', num(f.metrics['path conflicts']), undefined, f.metrics['path conflicts'] === 0, '= 0 with the protective layer'), metric('fleet runtime on the station (90 s)', rt.done ? 'ran' : 'stopped', undefined, rt.done && rt.log.some((l) => /commits to/.test(l)), 'robots commit and move')], notes: [rt.status()] }; },
  }),
  scenario({
    id: 'grp_chapters', title: 'Chapters 4, 7, 9, 12, 13, 16: games, Q-learning, GA/DE/ES, cellular automata, fuzzy control, Byzantine agreement', method: 'theory examples', docs: [ex('ch9-game'), ex('ch9-dilemma'), ex('ch7-qlearning'), ex('ch7-grid'), ex('ch12-evo'), ex('ch4-ca'), ex('ch4-rule30'), ex('ch13-fuzzy'), ex('ch16-resilience'), ex('ch16-three')],
    description: 'The worked examples of the theory chapters: Nash equilibria and the mixed payoff 5.2, Shapley (26.67, 41.67, 51.67) in the core; table 7.2 of Q-learning and the optimum 6.2 / 8 / 10; the GA generation of §12.2.6 and x* = 31; rule 90 vs rule 30 and the glider; the fuzzy controller firing rules 5 and 2 at 0.4 / 0.3; four agents with one traitor agree, three do not; switching between two stable modes destabilises below the dwell time.',
    evaluate: () => { const g = A(ex('ch9-game')), q = A(ex('ch7-qlearning')), e = A(ex('ch12-evo')), c = A(ex('ch4-ca')), fz = A(ex('ch13-fuzzy')), r = A(ex('ch16-resilience')), t = A(ex('ch16-three')); return { metrics: [metric('pure Nash equilibria', num(g.metrics['pure equilibria']), undefined, g.metrics['pure equilibria'] === 2, '= 2'), metric('mixed payoff', num(g.metrics['mixed payoff']), undefined, g.metrics['mixed payoff'] === 5.2, '= 5.2'), metric('Shapley in the core', String(g.metrics['shapley']), undefined, g.metrics['in core'] === true, '(26.67, 41.67, 51.67)'), metric('Q*(s₁)', num(q.metrics['Q*(start)']), undefined, q.metrics['Q*(start)'] === 6.2, '= 6.2'), metric('GA optimum', num(e.metrics['ga best x']), undefined, e.metrics['ga best x'] === 31, 'x* = 31'), metric('ES step vs ε*', `${e.metrics['es eps']} vs ${e.metrics['eps optimum']}`, undefined, e.ok, 'ES finds 2/(λ₂ + λₙ)'), metric('glider period', num(c.metrics['life period']), undefined, c.metrics['life period'] === 4, '= 4'), metric('fuzzy v', num(fz.metrics['v']), undefined, num(fz.metrics['v']) > 0.25 && num(fz.metrics['v']) < 0.5, '0.3–0.4 of max'), metric('Byzantine n = 4', String(r.metrics['byzantine agreement']), undefined, r.metrics['byzantine agreement'] === true, 'agree'), metric('Byzantine n = 3', t.sections[0].lines[1].includes('tied') ? 'ambiguous' : 'decided', undefined, t.sections[0].lines[1].includes('tied'), 'evidence ties'), metric('dwell time', String(r.metrics['dwell time']), 's', r.metrics['dwell time'] !== 'none', 'found')] }; },
  }),
  scenario({
    id: 'grp_architectures', title: 'Architectures: the same mission centralised, decentralised and hybrid — coordinator outage, robot failure, partition (chapter 3)', method: 'mission / architectures',
    docs: [ex('arch-compare'), ex('arch-partition'), ex('arch-station')],
    setup: (st) => { for (let i = 0; i < 4; i++) { const r = st.addChild(new MobileRobot(`r${i + 1}`)); r.setPose2D(i * 1200, 0, 0); r.kin.maxSpeed = 700; r.home = { x: i * 1200, y: 0, theta: 0 }; } const zone = (name: string, x: number, y: number, kind: ZoneItem['kind'] = 'work', half = 600) => { const z = st.addChild(new ZoneItem(name)); z.kind = kind; z.polygon = [[x - half, y - half], [x + half, y - half], [x + half, y + half], [x - half, y + half]]; }; zone('Dock', 6000, 0); zone('Shelf A', 9000, 2000); zone('Shelf B', 9000, -2000); zone('Shelf C', 11000, 2000); zone('Shelf D', 11000, -2000); zone('Pillar', 3000, 2000, 'nogo', 400); addControlModel(st, 'des', 'Fleet supervisor', FLEET_SUPERVISOR); addControlModel(st, 'hybrid', 'Fleet modes', FLEET_MODES); },
    description: 'A five-phase mission (formation, move, task allocation, gathering, return) with a robot failure at 40 s and a coordinator outage 60–110 s runs under the three architectures of chapter 3: the centralised group stalls while its coordinator is down, the decentralised group never depends on it (but takes slots by index and needs agreement time), the hybrid group plans globally, keeps the formation locally and falls back during the outage. A second document puts robots beyond the coordinator\'s range (radio 3.5 m with 20 % loss, coordinator 3 m); a third runs on configured station robots with zones, a no-go pillar, a des supervisor and a mode automaton.',
    evaluate: (st) => {
      const c = A(ex('arch-compare')), p = A(ex('arch-partition')); const robots = st.itemsOfType<MobileRobot>(ItemType.MOBILE_ROBOT);
      const rt = new FleetRuntime({ kind: 'mission', source: ex('arch-station').source, robots, station: st }); for (let t = 0; t < 420 && !rt.done; t += 0.1) rt.tick(0.1);
      const home = robots.every((r) => Math.hypot(r.state.x - r.home!.x, r.state.y - r.home!.y) < 400);
      return { metrics: [metric('all architectures complete', String(c.metrics['all complete']), undefined, c.metrics['all complete'] === true, 'true'), metric('centralised stalled', num(c.metrics['centralized stalled s']), 's', num(c.metrics['centralized stalled s']) > 40, '≈ outage 50 s'), metric('decentralised stalled', num(c.metrics['decentralized stalled s']), 's', c.metrics['decentralized stalled s'] === 0, '= 0'), metric('hybrid fallback', num(c.metrics['hybrid fallback s']), 's', num(c.metrics['hybrid fallback s']) > 0 && num(c.metrics['hybrid fallback s']) < 60, '> 0, < outage'), metric('min distance (all)', Math.min(num(c.metrics['centralized min distance']), num(c.metrics['decentralized min distance']), num(c.metrics['hybrid min distance'])), 'm', Math.min(num(c.metrics['centralized min distance']), num(c.metrics['decentralized min distance']), num(c.metrics['hybrid min distance'])) >= 0.395, '≥ d_safe'), metric('beyond the coordinator: decentralised completes', String(p.metrics['decentralized time'] !== 'incomplete'), undefined, p.metrics['decentralized time'] !== 'incomplete', 'true'), metric('beyond the coordinator: hybrid completes with fallback', `${p.metrics['hybrid time']} s, fallback ${p.metrics['hybrid fallback s']} s`, undefined, p.metrics['hybrid time'] !== 'incomplete' && num(p.metrics['hybrid fallback s']) > 0, 'complete, fallback > 0'), metric('beyond the coordinator: centralised incomplete, stalled', num(p.metrics['centralized stalled s']), 's', p.metrics['centralized time'] === 'incomplete' && num(p.metrics['centralized stalled s']) > 100, 'incomplete, stalled > 100 s'), metric('station mission (hybrid, zones, supervisor, modes)', rt.done && home ? 'complete, robots home' : 'incomplete', undefined, rt.done && home, 'phases through Dock and the shelves, plant P10')], notes: [rt.status()] };
    },
  }),
];
const FLEET_SUPERVISOR = `des Fleet supervisor
automaton Mission
  initial P0
  marked P0 P1 P2 P3 P4 P5 P6 P7 P8 P9 P10
  P0 -form_start-> P1
  P1 -form_done-> P2
  P2 -goto_start-> P3
  P3 -goto_done-> P4
  P4 -allocate_start-> P5
  P5 -allocate_done-> P6
  P6 -gather_start-> P7
  P7 -gather_done-> P8
  P8 -home_start-> P9
  P9 -home_done-> P10
uncontrollable form_done goto_done allocate_done gather_done home_done`;
const FLEET_MODES = `hybrid Fleet modes
var dist_human=10 phase=0
initial NORMAL
mode NORMAL vmax=0.6
mode SLOW vmax=0.1
NORMAL -> SLOW when dist_human < 2 dwell=0.2
SLOW -> NORMAL when dist_human > 3 dwell=0.2`;
