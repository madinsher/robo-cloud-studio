import { describe, it, expect } from 'vitest';
import { DES, parallel, analyseBlocking, supcon, checkNonconflict, observer, checkControllability, checkDiagnosability, supervisorTable, SupervisorRuntime, checkObservability } from '../src/ctl/des';
import { youbotPlant, youbotSpecs, YOUBOT_UC, YOUBOT_UO } from '../src/ctl/examples';

describe('DES: course example A (youBot mobile manipulator)', () => {
  it('composes the plant: 504 reachable states, 144 blocking (object on the floor) without o_giveup', () => {
    const g0 = parallel(...youbotPlant(false));
    const r = analyseBlocking(g0);
    expect(g0.X.size).toBe(504);
    expect(r.transitions).toBe(2356);
    expect(r.blocking.length).toBe(144);
    expect(r.trace).toEqual(['a_reach', 'a_reached', 'b_go_table', 'b_arrive', 'g_close', 'g_ok', 'g_slip']);
    expect(r.blocking.every((b) => g0.part(b, 'Object') === 'Fl')).toBe(true);
    expect(r.deadlocks.length + r.livelocks.flat().length).toBeGreaterThan(0);
  });
  it('is non-blocking once the failure outcome o_giveup is modelled (648 states)', () => {
    const g = parallel(...youbotPlant(true));
    const r = analyseBlocking(g);
    expect(g.X.size).toBe(648); expect(r.transitions).toBe(3172); expect(r.nonblocking).toBe(true);
  });
  it('monolithic synthesis E1,E2,E3 gives a 324-state supervisor and reveals early g_close; E5 fixes it', () => {
    const g = parallel(...youbotPlant(true));
    const S = youbotSpecs();
    const r = supcon(g, [S.E1, S.E2, S.E3], YOUBOT_UC);
    expect(r.productStates).toBe(324); expect(r.states.size).toBe(324); expect(r.supervisor.transitionCount).toBe(1138);
    expect(r.disabled).toEqual({ b_go_table: 162, a_reach: 108, b_go_home: 108, g_close: 72, b_go_box: 54 });
    const early = new Set([...r.supervisor.f.entries()].filter(([, m]) => m.has('g_close')).map(([x]) => x).filter((x) => r.supervisor.part(x, 'Arm') !== 'X').map((x) => r.supervisor.parts.get(x)!.slice(0, 5).join()));
    expect(early.size).toBe(60);
    const r5 = supcon(g, [S.E1, S.E2, S.E3, S.E5], YOUBOT_UC);
    expect(r5.states.size).toBe(324); expect(r5.supervisor.transitionCount).toBe(1078);
    const mission = r5.supervisor.traceTo((x) => r5.supervisor.xm.has(x) && r5.supervisor.part(x, 'Object') === 'Bx');
    expect(mission).toEqual('b_go_table b_arrive a_reach a_reached v_detect g_close g_ok a_stow a_stowed b_go_box b_arrive a_reach a_reached g_put a_stow a_stowed b_go_home b_arrive'.split(' '));
    expect(analyseBlocking(r5.supervisor).nonblocking).toBe(true);
    // table + runtime
    const tbl = supervisorTable(r5.supervisor, YOUBOT_UC, YOUBOT_UO, 5);
    const rt = new SupervisorRuntime(tbl);
    for (const e of mission!) { expect(rt.allowed(e)).toBe(true); rt.observe(e); }
    expect(rt.marked()).toBe(true);
    const rt2 = new SupervisorRuntime(tbl);
    for (const e of ['b_go_table', 'b_arrive', 'a_reach', 'a_reached']) rt2.observe(e);
    expect(rt2.allowed('b_go_box')).toBe(false);
  });
  it('modular supervisors are non-conflicting; E4 (never drop) is unrealisable, E4\' gives 224 states', () => {
    const g = parallel(...youbotPlant(true));
    const S = youbotSpecs();
    const mods = [S.E1, S.E2, S.E3, S.E5].map((sp) => supcon(g, [sp], YOUBOT_UC));
    expect(mods.map((m) => m.states.size)).toEqual([432, 648, 648, 648]);
    const nc = checkNonconflict(g, mods.map((m) => m.supervisor));
    expect(nc.nonconflicting).toBe(true); expect(nc.jointStates).toBe(324);
    const r4 = supcon(g, [S.E1, S.E2, S.E3, S.E5, S.E4], YOUBOT_UC);
    expect(r4.realizable).toBe(false); expect(r4.log[0].removedControllability).toBe(36); expect(r4.log[0].removedBlocking).toBe(144);
    const r4m = supcon(g, [S.E1, S.E2, S.E3, S.E5, S.E4m], YOUBOT_UC);
    expect(r4m.states.size).toBe(224); expect(r4m.log.map((l) => [l.removedControllability, l.removedBlocking])).toEqual([[6, 94], [0, 0]]);
    const bx = [...r4m.supervisor.xm].filter((x) => r4m.supervisor.part(x, 'Object') === 'Bx');
    expect(bx.length).toBe(0);
    const cc = checkControllability(g, S.E4, YOUBOT_UC);
    expect(cc.controllable).toBe(false); expect(cc.issues[0].event).toBe('g_slip');
  });
  it('observer under unobservable g_slip has 324 states, 36 ambiguous', () => {
    const g = parallel(...youbotPlant(true));
    const S = youbotSpecs();
    const r5 = supcon(g, [S.E1, S.E2, S.E3, S.E5], YOUBOT_UC);
    const obs = observer(r5.supervisor, YOUBOT_UO);
    expect(obs.states.size).toBe(324);
    const amb = [...obs.states.values()].filter((cell) => { const o = new Set(cell.map((x) => r5.supervisor.part(x, 'Object'))); return o.has('Hn') && o.has('Fl'); });
    expect(amb.length).toBe(36);
    const seq = 'v_detect b_go_table b_arrive a_reach a_reached g_close g_ok a_stow a_stowed b_go_box b_arrive'.split(' ');
    let cur = obs.initial; for (const e of seq) cur = obs.delta.get(cur)!.get(e)!;
    expect([...new Set(obs.states.get(cur)!.map((x) => r5.supervisor.part(x, 'Object')))].sort()).toEqual(['Fl', 'Hn']);
    const ob = checkObservability(g, r5.states, r5.supervisor, YOUBOT_UO, ['b_go_table', 'b_go_box', 'b_go_home', 'a_reach', 'a_stow', 'g_close', 'g_put', 'b_replan', 'o_giveup']);
    expect(ob.observable).toBe(true); // decisions agree inside every cell: o_giveup is only physically possible in Fl
    const diag = checkDiagnosability(g, ['g_slip'], YOUBOT_UO);
    expect(diag.diagnosable).toBe(false);
  });
});

describe('DES basics', () => {
  it('detects deadlock vs livelock and traces', () => {
    const g = new DES('g', ['a', 'b', 'c'], [{ from: 's0', event: 'a', to: 's1' }, { from: 's0', event: 'b', to: 'd' }, { from: 's1', event: 'c', to: 'l1' }, { from: 'l1', event: 'a', to: 'l2' }, { from: 'l2', event: 'b', to: 'l1' }, { from: 's1', event: 'a', to: 'm' }], 's0', ['m']);
    const r = analyseBlocking(g);
    expect(r.deadlocks).toEqual(['d']); expect(r.livelocks).toEqual([expect.arrayContaining(['l1', 'l2'])]);
    expect(r.deadlockTrace).toEqual(['b']); expect(r.livelockTrace).toEqual(['a', 'c']);
  });
  it('flags a supervisor whose decision depends on an unobservable event', () => {
    // plant: 0 -c-> 5 ; 0 -u-> 3 -c-> 4 ; u unobservable. Spec forbids c after u, so the decision on c depends on u.
    const g = new DES('g', ['u', 'c'], [{ from: '0', event: 'c', to: '5' }, { from: '0', event: 'u', to: '3' }, { from: '3', event: 'c', to: '4' }], '0', ['5', '3']);
    const spec = new DES('E', ['u', 'c'], [{ from: 'p', event: 'c', to: 'p' }, { from: 'p', event: 'u', to: 'q' }], 'p', ['p', 'q']);
    const r = supcon(g, [spec], ['u']);
    const ob = checkObservability(g, r.states, r.supervisor, ['u'], ['c']);
    expect(ob.observable).toBe(false); expect(ob.issues[0].event).toBe('c');
  });
  it('diagnoses a fault that is followed by a distinguishing observable event', () => {
    const g = new DES('g', ['f', 'a', 'b'], [{ from: '0', event: 'a', to: '1' }, { from: '0', event: 'f', to: '2' }, { from: '2', event: 'b', to: '3' }, { from: '1', event: 'a', to: '1' }, { from: '3', event: 'b', to: '3' }], '0', ['1', '3']);
    expect(checkDiagnosability(g, ['f'], ['f']).diagnosable).toBe(true);
    const h = new DES('h', ['f', 'a'], [{ from: '0', event: 'a', to: '1' }, { from: '0', event: 'f', to: '2' }, { from: '2', event: 'a', to: '2' }, { from: '1', event: 'a', to: '1' }], '0', ['1', '2']);
    expect(checkDiagnosability(h, ['f'], ['f']).diagnosable).toBe(false);
  });
});
