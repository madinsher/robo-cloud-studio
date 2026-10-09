import { describe, it, expect } from 'vitest';
import { BehaviorTree, BTNodeSpec, seq, fallback, action, condition, skill, retry, checkBehaviorTree, estimateFTS, btKripke, Statechart, BTStatus } from '../src/ctl/bt';
import { parseLTL, parseCTL, fmt, ltlToBuchi, checkLTL, checkCTL, LTL3Monitor, Kripke, kripkeFromDES, kripkeFromPetri } from '../src/ctl/temporal';
import { SmvModel, cellZoneModel } from '../src/ctl/smv';
import { synthesizeGR1, stepController } from '../src/ctl/gr1';
import { checkHybrid, dwellTime, cbfFilter, zoneConstraint, distanceConstraint, MOBILE_MANIPULATOR_MODES, simulateModes, HybridSpec } from '../src/ctl/hybrid';
import { expr } from '../src/ctl/expr';
import { parallel } from '../src/ctl/des';
import { youbotPlant, youbotGR1, CELL_DEADLOCK_NET } from '../src/ctl/examples';
import { PetriNet } from '../src/ctl/petri';

const mission = (): BTNodeSpec => fallback('root', [
  seq('emergency', [condition('estop', {}, 'ConditionEStop'), action('halt', {}, { name: 'ActionHalt' })]),
  seq('battery', [condition('battery_low', {}, 'ConditionBatteryLow'), seq('to dock', [action('stow'), action('navigate', { target: 'dock' }, { name: 'NavDock' }), action('dock')], true)]),
  fallback('mission', [
    seq('collect', [condition('targets_remain', {}, 'ConditionTargetsRemain'), seq('collect one', [action('select_target'), action('navigate', { target: 'table' }, { name: 'NavTable' }),
      retry(2, seq('grasp', [action('refine_pose'), action('grasp', {}, { name: 'ActionGrasp', timeout: 12 }), condition('object_held', {}, 'ConditionObjectHeld')], true)),
      action('navigate', { target: 'bin' }, { name: 'NavBin' }), action('place')], true)]),
    seq('finish', [action('navigate', { target: 'home' }, { name: 'NavHome' }), action('report')], true),
  ]),
]);

describe('Behavior trees', () => {
  it('runs the course mission, pre-empts a running navigation on e-stop and halts it', () => {
    const bb: Record<string, unknown> = { estop: false, battery: 0.9, targets: 2, held: false, nav: 0 };
    const halted: string[] = [];
    const bt = new BehaviorTree(mission(), {
      conditions: { estop: (c) => !!c.bb.estop, battery_low: (c) => (c.bb.battery as number) < 0.2, targets_remain: (c) => (c.bb.targets as number) > 0, object_held: (c) => !!c.bb.held },
      actions: { halt: () => 'success', stow: () => 'success', dock: () => 'success', select_target: () => 'success', refine_pose: () => 'success', report: (c) => { c.bb.reported = true; return 'success'; },
        navigate: (c) => { c.bb.nav = (c.bb.nav as number) + 1; if ((c.bb.nav as number) >= 3) { c.bb.nav = 0; return 'success'; } return 'running'; },
        grasp: (c) => { c.bb.held = true; return 'success'; }, place: (c) => { c.bb.held = false; c.bb.targets = (c.bb.targets as number) - 1; return 'success'; } },
      halt: { navigate: () => { halted.push('navigate'); } },
    }, bb);
    for (let i = 0; i < 60; i++) bt.tick(0.5);
    expect(bb.targets).toBe(0); expect(bb.reported).toBe(true);
    bt.reset(); bb.targets = 1; bb.nav = 0;
    bt.tick(0.5); expect(bt.running.size).toBe(1); // NavTable running
    bb.estop = true; expect(bt.tick(0.5)).toBe('success'); expect(halted).toEqual(['navigate']);
  });
  it('skill contracts fail on post-conditions, retries recover, timeouts fire', () => {
    const s = skill({ name: 'Grasp', pre: 'detected', action: 'grasp', post: 'held', timeout: 2, retries: 2 });
    let attempts = 0;
    const bt = new BehaviorTree(s, { actions: { grasp: (c) => { attempts++; c.bb.held = attempts >= 2; return 'success'; } } }, { detected: true, held: false });
    expect(bt.tick()).toBe('running'); // first attempt fails the post-condition → retry
    expect(bt.tick()).toBe('success'); expect(attempts).toBe(2);
    const slow = new BehaviorTree(skill({ name: 'Slow', action: 'go', timeout: 1 }), { actions: { go: () => 'running' } });
    let st: BTStatus = 'running'; for (let i = 0; i < 20 && st === 'running'; i++) st = slow.tick(0.5);
    expect(st).toBe('failure');
    const noPre = new BehaviorTree(s, {}, { detected: false }); let np: BTStatus = 'running'; for (let i = 0; i < 4 && np === 'running'; i++) np = noPre.tick(); expect(np).toBe('failure');
  });
  it('structural checks and Monte-Carlo finite-time success', () => {
    const issues = checkBehaviorTree(mission());
    expect(issues.some((i) => i.level === 'error')).toBe(false);
    expect(issues.some((i) => /timeout/.test(i.message))).toBe(true);
    const bad = checkBehaviorTree({ type: 'sequence', children: [] });
    expect(bad[0].level).toBe('error');
    const fts = estimateFTS(mission(), { estop: { p: 0 }, battery_low: { p: 0 }, targets_remain: { p: 0.5 }, grasp: { p: 0.7, ticks: 3 }, object_held: { p: 1 }, navigate: { ticks: 4 } }, 100, 400, 3);
    expect(fts.pSuccess).toBeGreaterThan(0.8); expect(fts.meanTicks).toBeGreaterThan(5);
  });
  it('abstracts the tree to a Kripke structure and model-checks safety: halt follows e-stop in the same tick', () => {
    const k = btKripke(mission(), { ActionHalt: ['success', 'running'] }, 3000);
    expect(k.states.length).toBeGreaterThan(5);
    const r = checkLTL(k, 'G ("ok:ConditionEStop" -> "tick:ActionHalt")');
    expect(r.holds).toBe(true);
    const r2 = checkLTL(k, 'G ("ok:ConditionEStop" -> !"run:NavTable")');
    expect(r2.holds, r2.explanation).toBe(true);
    const r3 = checkLTL(k, 'G !"run:NavTable"'); // navigation does run somewhere
    expect(r3.holds).toBe(false); expect(r3.prefix.length + r3.cycle.length).toBeGreaterThan(0);
  });
  it('statecharts: hierarchy, orthogonal regions and history flatten to an automaton', () => {
    const sc = new Statechart({ name: 'ops', states: [
      { id: 'Off', initial: true }, { id: 'On' }, { id: 'Auto', parent: 'On', initial: true, history: true },
      { id: 'Idle', parent: 'Auto', region: 'ctl', initial: true }, { id: 'Busy', parent: 'Auto', region: 'ctl' },
      { id: 'Cold', parent: 'Auto', region: 'temp', initial: true }, { id: 'Hot', parent: 'Auto', region: 'temp' },
      { id: 'Manual', parent: 'On' }, { id: 'Fault', parent: 'On' },
    ], transitions: [
      { from: 'Off', to: 'On', event: 'power_on' }, { from: 'On', to: 'Off', event: 'power_off' },
      { from: 'Idle', to: 'Busy', event: 'start' }, { from: 'Busy', to: 'Idle', event: 'done' }, { from: 'Cold', to: 'Hot', event: 'heat' }, { from: 'Hot', to: 'Cold', event: 'cool' },
      { from: 'Auto', to: 'Manual', event: 'manual' }, { from: 'Manual', to: 'Auto', event: 'auto' }, { from: 'On', to: 'Fault', event: 'error' }, { from: 'Fault', to: 'Auto', event: 'reset' },
    ] });
    expect(sc.initialConfiguration()).toEqual(['Off']);
    const h = new Map<string, string>();
    let c = sc.step(['Off'], 'power_on', h).config; expect(c).toEqual(['Cold', 'Idle']);
    c = sc.step(c, 'start', h).config; c = sc.step(c, 'heat', h).config; expect(c).toEqual(['Busy', 'Hot']);
    c = sc.step(c, 'error', h).config; expect(c).toEqual(['Fault']); // ancestor transition pre-empts both regions
    c = sc.step(c, 'reset', h).config; expect(c).toEqual(['Busy', 'Hot']); // history restores the regions
    const flat = sc.flatten();
    expect(flat.states!.length).toBe(1 + 4 + 1 + 1);
    expect(flat.transitions.length).toBeGreaterThan(15);
  });
});

describe('Temporal logic: parsing, Büchi, LTL / CTL model checking', () => {
  it('parses and normalises formulas', () => {
    expect(fmt(parseLTL('G (a -> F b)'))).toBe('G (!a | F b)');
    expect(fmt(parseLTL('[] (r1.state = waiting -> <> r1.state = in_zone)'))).toContain('r1.state=waiting');
    expect(fmt(parseCTL('AG EF owner = none'))).toBe('AG EF owner=none');
    expect(() => parseLTL('G (a ->')).toThrow();
    const b = ltlToBuchi(parseLTL('G F p')); expect(b.nodes.length).toBeGreaterThan(0); expect(b.accepting.size).toBeGreaterThan(0);
  });
  it('reproduces the course cell verification: mutual exclusion holds, r2 starves under static priority, alternation fixes it', () => {
    const m = new SmvModel(cellZoneModel(false)); const k = m.kripke();
    expect(k.states.length).toBeGreaterThan(10);
    const spec = cellZoneModel(false);
    const res = spec.specs!.map((s) => (s.kind === 'LTL' ? checkLTL(k, s.formula, { fairness: spec.fairness }) : checkCTL(k, s.formula)));
    expect(res[0].holds).toBe(true); expect(res[1].holds).toBe(true);
    expect(res[2].holds).toBe(false); // starvation of r2
    const cex = res[2] as ReturnType<typeof checkLTL>;
    expect(cex.cycle.length).toBeGreaterThan(0); expect(cex.cycle.every((s) => /r2\.state=waiting/.test(s))).toBe(true);
    expect(res[3].holds).toBe(true); expect(res[4].holds).toBe(true); expect(res[5].holds).toBe(true);
    const fair = cellZoneModel(true); const kf = new SmvModel(fair).kripke();
    expect(checkLTL(kf, fair.specs![2].formula, { fairness: fair.fairness }).holds).toBe(true);
    expect(checkLTL(kf, fair.specs![1].formula, { fairness: fair.fairness }).holds).toBe(true);
    expect(checkLTL(kf, fair.specs![0].formula).holds).toBe(true);
  });
  it('CTL counterexamples and witnesses', () => {
    const k: Kripke = { states: ['a', 'b', 'c'], initial: ['a'], next: (s) => ({ a: ['b'], b: ['c', 'a'], c: ['c'] }[s]!), holds: (s, at) => (at === 'p' ? s !== 'c' : at === 'q' ? s === 'c' : false) };
    const r = checkCTL(k, 'AG p'); expect(r.holds).toBe(false); expect(r.counterexample).toEqual(['a', 'b', 'c']);
    expect(checkCTL(k, 'EF q').holds).toBe(true);
    const af = checkCTL(k, 'AF q'); expect(af.holds).toBe(false); expect(af.counterexample!.length).toBeGreaterThan(1);
    expect(checkCTL(k, 'A[p U q]').holds).toBe(false); expect(checkCTL(k, 'E[p U q]').holds).toBe(true);
    expect(checkCTL(k, 'AG (q -> AX q)').holds).toBe(true);
  });
  it('LTL over automata and Petri nets: R1 on the youBot plant, deadlock freedom of the cell', () => {
    const g = parallel(...youbotPlant(true)); const k = kripkeFromDES(g);
    const r1 = checkLTL(k, "G !((Base = 'mT' | Base = 'mB' | Base = 'mH') & (Arm = 'R' | Arm = 'W'))");
    expect(r1.holds).toBe(false); // the raw plant allows base and arm motion together — the supervisor E1 enforces it
    expect(r1.prefix.length).toBeGreaterThan(0);
    const kp = kripkeFromPetri(new PetriNet(CELL_DEADLOCK_NET).reachability());
    const dl = checkCTL(kp, 'AG !deadlock'); expect(dl.holds).toBe(false); expect(dl.counterexample!.length).toBe(3);
    expect(checkLTL(kp, 'G (a1 -> F a2)').holds).toBe(false);
    expect(checkCTL(kp, 'AG (rM + a1 + a2 + b2 = 1)').holds).toBe(true);
  });
  it('LTL3 monitor for R2: G(held -> (held U placed)) behaves like the course MonitorR2', () => {
    const m = new LTL3Monitor('G (held -> (held U placed))');
    expect(m.verdict).toBe('?');
    for (const o of [{ held: false, placed: false }, { held: true, placed: false }, { held: true, placed: false }, { held: true, placed: true }, { held: false, placed: false }]) expect(m.step(o)).toBe('?');
    m.step({ held: true, placed: false }); expect(m.step({ held: false, placed: false })).toBe('⊥'); // lost before placing
    const live = new LTL3Monitor('F done'); live.step({ done: false }); expect(live.verdict).toBe('?'); expect(live.step({ done: true })).toBe('⊤');
    const t = live.table(); expect(t.states.length).toBeGreaterThanOrEqual(2); expect(t.states.some((s) => s.verdict === '⊤')).toBe(true);
    const safety = new LTL3Monitor('G (human -> X stopped)'); safety.step(['human']); expect(safety.step([])).toBe('⊥');
  });
});

describe('GR(1) synthesis', () => {
  it('door example: unrealisable without the liveness assumption, realisable with it', () => {
    const base = { vars: [{ name: 'loc', owner: 'sys' as const, domain: ['a', 'b'] }, { name: 'door', owner: 'env' as const, domain: [false, true] }],
      sysInit: ["loc = 'a'"], sysTrans: ["(loc = 'a' && loc' = 'b') -> door", "(loc = 'b' && loc' = 'a') -> door"], sysLive: ["loc = 'a'", "loc = 'b'"] };
    const bad = synthesizeGR1({ ...base }); expect(bad.realizable).toBe(false); expect(bad.counterStrategy.length).toBeGreaterThan(0); expect(bad.diagnosis.join(' ')).toMatch(/liveness/);
    const good = synthesizeGR1({ ...base, envLive: ['door'] }); expect(good.realizable).toBe(true);
    const c = good.controller!; expect(c.size).toBeGreaterThan(1);
    // drive it: door open every second step; the controller must alternate locations
    let id = c.initial[0]; const locs: string[] = [];
    for (let i = 0; i < 12; i++) { const nid = stepController(c, id, { door: i % 2 === 0 }, base.vars); expect(nid).not.toBeNull(); id = nid!; locs.push(String(c.states[id].values.loc)); }
    expect(new Set(locs)).toEqual(new Set(['a', 'b']));
  });
  it('course example A mission is realisable with GF !human_present and unrealisable without it', () => {
    const ok = synthesizeGR1(youbotGR1());
    expect(ok.gameStates).toBe(160); expect(ok.realizable).toBe(true); expect(ok.controller!.size).toBeGreaterThan(10);
    const bad = synthesizeGR1(youbotGR1({ humanLeaves: false }));
    expect(bad.realizable).toBe(false); expect(bad.counterStrategy.length).toBeGreaterThan(0); expect(bad.winning).toBe(0);
  });
});

describe('Hybrid modes and CBF', () => {
  it('flags chattering without hysteresis, accepts the course mode automaton, computes dwell time', () => {
    const issues = checkHybrid(MOBILE_MANIPULATOR_MODES);
    expect(issues.some((i) => i.level === 'error')).toBe(false); expect(issues.some((i) => /hysteresis 0.5/.test(i.message))).toBe(true);
    const bad: HybridSpec = { name: 'x', initial: 'NORMAL', modes: [{ id: 'NORMAL' }, { id: 'SLOW' }], transitions: [{ from: 'NORMAL', to: 'SLOW', guard: 'd < 2' }, { from: 'SLOW', to: 'NORMAL', guard: 'd >= 2' }] };
    expect(checkHybrid(bad).some((i) => i.level === 'error' && /chattering/.test(i.message))).toBe(true);
    expect(dwellTime(2, 4)).toBeCloseTo(0.347, 3);
    // noisy distance around 2 m: the hysteresis automaton switches a few times, the bad one chatters
    let s = 1; const rnd = () => { s = (s * 1664525 + 1013904223) >>> 0; return s / 4294967296; };
    const samples = Array.from({ length: 400 }, (_, i) => ({ t: i * 0.05, d: 2 + 0.2 * (rnd() - 0.5), dist_human: 2 + 0.2 * (rnd() - 0.5), dist_target: 5, v: 0.5, arm_stowed: true, human: false, estop: false }));
    const good = simulateModes(MOBILE_MANIPULATOR_MODES, samples), chat = simulateModes(bad, samples);
    expect(chat.switches).toBeGreaterThan(20); expect(good.switches).toBeLessThan(4);
  });
  it('CBF filter reproduces the numeric example (0.414 m/s radial) and keeps the human distance', () => {
    const c = zoneConstraint([2.8, 0], [0, 0], 3, 2);
    const r = cbfFilter([0.8, 0], [c]);
    expect(r.u[0]).toBeCloseTo(0.414, 3); expect(r.active[0]).toBe(true);
    const r2 = cbfFilter([0.8, 0], [zoneConstraint([2.95, 0], [0, 0], 3, 2)]); expect(r2.u[0]).toBeCloseTo(0.101, 2);
    const h = distanceConstraint([0, 0], [1.5, 0], 1, 2);
    const r3 = cbfFilter([1, 0], [h]); expect(r3.u[0]).toBeLessThan(1); expect(r3.margin[0]).toBeGreaterThanOrEqual(-1e-9);
    const both = cbfFilter([1, 1], [zoneConstraint([2.9, 0], [0, 0], 3, 2), distanceConstraint([2.9, 0], [3.5, 0.5], 1, 2)]);
    expect(both.margin.every((m) => m >= -1e-6)).toBe(true);
    expect(expr("a && b' = 2 && x >= 1.5").bool({ a: true, x: 2 }, { b: 2 })).toBe(true);
  });
});
