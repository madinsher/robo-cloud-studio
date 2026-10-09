import { describe, it, expect } from 'vitest';
import { Warehouse, HOMEWORK_WAREHOUSE, CbbaAgent, Order, Executor, Traffic, FleetSim, HOMEWORK_FLEET, TrafficState } from '../src/mrs/warehouse';

const wh = new Warehouse(HOMEWORK_WAREHOUSE);
const agent = (name: string, station: string, cap = 3) => { const a = new CbbaAgent(name, wh, cap, 0.98, 0.3, 3); a.startStation = station; return a; };
const orders = (): Order[] => [{ id: 'o1', pickup: 'P1', drop: 'D1', reward: 10 }, { id: 'o2', pickup: 'P3', drop: 'D3', reward: 10 }, { id: 'o3', pickup: 'P2', drop: 'D2', reward: 10 }];
const st = (stamp: number, holds: Array<[number, number]>, want?: [number, number], since?: number): TrafficState => ({ stamp, holds, want: want ?? null, wantSince: since ?? null });

describe('homework — warehouse model', () => {
  it('map, stations, shortest paths and distances', () => { expect(wh.rows).toBe(10); expect(wh.cols).toBe(16); expect(wh.free([1, 1])).toBe(false); expect(wh.path([0, 0], [0, 3])!.length).toBe(4); expect(wh.stationDistance('P1', 'D1')).toBeCloseTo(7.5, 9); expect(wh.cellXY([3, 0])).toEqual([0, -1.5]); expect(wh.xyCell(7.5, -1.5)).toEqual([3, 15]); });
});
describe('homework stage 2 — CBBA on orders', () => {
  it('bundle respects capacity and bids; skips committed and outbid orders; marginal gain equals the score difference', () => {
    const a = agent('r1', 'H1', 2); for (const o of orders()) a.addOrder(o); const added = a.buildBundle(); expect(added.length).toBe(2); expect(a.bundle).toEqual(added); for (const oid of a.bundle) { expect(a.z[oid]).toBe('r1'); expect(a.y[oid]).toBeGreaterThan(0); } expect([...a.path].sort()).toEqual([...a.bundle].sort());
    const b = agent('r1', 'H1'); for (const o of orders()) b.addOrder(o); b.committed.add('o1'); b.y['o2'] = 1e6; b.z['o2'] = 'r9'; b.buildBundle(); expect(b.bundle).not.toContain('o1'); expect(b.bundle).not.toContain('o2');
    const c = agent('r1', 'P1', 1); const o: Order = { id: 'o1', pickup: 'P1', drop: 'D1', reward: 10 }; c.addOrder(o); c.buildBundle(); expect(c.y['o1']).toBeCloseTo(10 * 0.98 ** c.orderTime('P1', o), 9);
  });
  it('a pair converges conflict-free; stale neighbours are ignored; committed orders are removed; silent winners are forgotten', () => {
    const a = agent('r1', 'H1'), b = agent('r2', 'H4'); for (const ag of [a, b]) for (const o of orders()) ag.addOrder(o); let t = 0;
    for (let k = 0; k < 20; k++) { t += 0.5; a.buildBundle(); b.buildBundle(); const sa = a.exportState(t), sb = b.exportState(t); const ca = a.consensus({ r2: sb }, t), cb = b.consensus({ r1: sa }, t); if (!ca && !cb) break; }
    expect(a.bundle.filter((x) => b.bundle.includes(x))).toEqual([]); for (const oid of [...a.bundle, ...b.bundle]) expect(a.z[oid]).toBe(b.z[oid]); expect(a.bundle.length + b.bundle.length).toBe(3);
    const s = agent('r1', 'H1'); for (const o of orders()) s.addOrder(o); s.buildBundle(); const before = s.bundle.slice(); const fake = { stamp: 0, y: Object.fromEntries(before.map((o) => [o, 1e6])), z: Object.fromEntries(before.map((o) => [o, 'r0'])), committed: [] }; s.consensus({ r0: fake }, 10, 3); expect(s.bundle).toEqual(before); s.consensus({ r0: fake }, 1, 3); expect(s.bundle).toEqual([]);
    const c = agent('r1', 'H1'); for (const o of orders()) c.addOrder(o); c.buildBundle(); const victim = c.bundle[0]; c.consensus({ r2: { stamp: 1, y: {}, z: {}, committed: [victim] } }, 1); expect(c.orders.has(victim)).toBe(false); expect(c.bundle).not.toContain(victim); expect(c.committed.has(victim)).toBe(true);
    const f = agent('r1', 'H1'); for (const o of orders()) f.addOrder(o); f.y['o1'] = 50; f.z['o1'] = 'r2'; expect(f.forgetSilentWinners(0, 6)).toEqual([]); f.lastHeard['r2'] = 1; expect(f.forgetSilentWinners(5, 6)).toEqual([]); expect(f.forgetSilentWinners(8, 6)).toEqual(['o1']); expect(f.z['o1']).toBe(''); expect(f.y['o1']).toBe(0);
  });
});
describe('homework stage 2 — executor state machine', () => {
  it('nominal cycle, busy rejects, nav failures, stop / reset, go home and interrupt', () => {
    const e = new Executor('H1', 1); expect(e.assign('o1', 'P1', 'D1')).toEqual([['goto', 'P1']]); expect(e.state).toBe('TO_PICKUP'); expect(e.step(['arrived', null])).toEqual([['handle', 'pick']]); expect(e.step(['handled', null])).toEqual([['goto', 'D1']]); expect(e.step(['arrived', null])).toEqual([['handle', 'drop']]); expect(e.step(['handled', null])).toEqual([['delivered', 'o1']]); expect(e.state).toBe('IDLE'); expect(e.order).toBeNull();
    const b = new Executor('H1'); b.assign('o1', 'P1', 'D1'); expect(b.assign('o2', 'P2', 'D2')).toEqual([]); expect(b.order).toBe('o1'); expect(b.pickup).toBe('P1');
    const n = new Executor('H1', 1); n.assign('o1', 'P1', 'D1'); expect(n.step(['nav_failed', null])).toEqual([['goto', 'P1']]); expect(n.step(['nav_failed', null])).toEqual([['release', 'o1']]); expect(n.state).toBe('IDLE');
    const l = new Executor('H1', 1); l.assign('o1', 'P1', 'D1'); l.step(['arrived', null]); l.step(['handled', null]); for (let k = 0; k < 5; k++) expect(l.step(['nav_failed', null])).toEqual([['goto', 'D1']]); expect(l.state).toBe('TO_DROP');
    const s = new Executor('H1'); s.assign('o1', 'P1', 'D1'); const cmds = s.step(['stop', null]); expect(cmds).toContainEqual(['cancel', null]); expect(cmds).toContainEqual(['release', 'o1']); expect(s.state).toBe('FAILED'); expect(s.step(['arrived', null])).toEqual([]); expect(s.assign('o2', 'P2', 'D2')).toEqual([]);
    const w = new Executor('H1'); w.assign('o1', 'P1', 'D1'); w.step(['arrived', null]); w.step(['handled', null]); expect(w.step(['stop', null])).toEqual([['cancel', null]]); expect(w.order).toBe('o1'); expect(w.step(['reset', null])).toEqual([['goto', 'D1']]); expect(w.state).toBe('TO_DROP');
    const h = new Executor('H1'); h.assign('o1', 'P1', 'D1'); for (const ev of ['arrived', 'handled', 'arrived', 'handled'] as const) h.step([ev, null]); expect(h.goHome()).toEqual([['goto', 'H1']]); expect(h.state).toBe('TO_HOME'); expect(h.assign('o2', 'P2', 'D2')).toEqual([['cancel', null], ['goto', 'P2']]); expect(h.state).toBe('TO_PICKUP');
  });
});
describe('homework stage 3 — cell reservation', () => {
  it('needs an own claim and settle; held cells; acknowledgement; priority by time then name', () => {
    const t = new Traffic('r1', wh, 0.3); expect(t.mayEnter([0, 1], {}, 1)).toBe(false); t.want = [0, 1]; t.wantSince = 1; expect(t.mayEnter([0, 1], {}, 1.2)).toBe(false); expect(t.mayEnter([0, 1], {}, 1.31)).toBe(true); expect(t.mayEnter([0, 2], {}, 1.31)).toBe(false);
    const h = new Traffic('r1', wh, 0.3); h.want = [0, 1]; h.wantSince = 0; expect(h.mayEnter([0, 1], { r2: st(1, [[0, 1]]) }, 1)).toBe(false);
    const a = new Traffic('r1', wh, 0.3); a.want = [0, 1]; a.wantSince = 1; expect(a.mayEnter([0, 1], { r2: st(1.2, [[5, 5]]) }, 2)).toBe(false); expect(a.mayEnter([0, 1], { r2: st(1.4, [[5, 5]]) }, 2)).toBe(true);
    const p = new Traffic('r2', wh, 0.3); p.want = [0, 1]; p.wantSince = 1; expect(p.mayEnter([0, 1], { r3: st(1.5, [[0, 2]], [0, 1], 0.9) }, 1.6)).toBe(false); expect(p.mayEnter([0, 1], { r3: st(1.5, [[0, 2]], [0, 1], 1.1) }, 1.6)).toBe(true); expect(p.mayEnter([0, 1], { r1: st(1.5, [[0, 0]], [0, 1], 1) }, 1.6)).toBe(false); expect(p.mayEnter([0, 1], { r4: st(1.5, [[0, 2]], [0, 1], 1) }, 1.6)).toBe(true);
  });
  it('two robots never reserve the same cell and both arrive; detour around a blocker; no detour keeps the route', () => {
    const a = new Traffic('r1', wh, 0.2), b = new Traffic('r2', wh, 0.2); expect(a.setGoal([0, 0], [0, 4])).toBe(true); expect(b.setGoal([0, 4], [0, 0])).toBe(true); let now = 0;
    for (let k = 0; k < 400; k++) { now += 0.1; const sa = a.exportState(now - 0.05), sb = b.exportState(now - 0.05); a.step({ r2: sb }, now); b.step({ r1: sa }, now); const ha = a.holds().map(String), hb = b.holds().map(String); expect(ha.filter((x) => hb.includes(x))).toEqual([]); for (const tr of [a, b]) if (tr.nextTarget()) tr.arrived(); if (a.done() && b.done()) break; }
    expect(a.done() && b.done()).toBe(true);
    const t = new Traffic('r1', wh, 0.1, 2, 1); expect(t.setGoal([0, 0], [0, 6])).toBe(true); const changed = t.replanAround(t.fresh({ r2: st(0, [[0, 3]]) }, 0), 0); expect(changed).toBe(true); expect(t.route.some((c) => c[0] === 0 && c[1] === 3)).toBe(false); expect(t.route[t.route.length - 1]).toEqual([0, 6]); expect(wh.grid[0][3]).toBe(false);
    const n = new Traffic('r1', wh); expect(n.setGoal([3, 0], [4, 0])).toBe(true); const before = n.route.map(String); n.replanAround(n.fresh({ r2: st(5, [[3, 1], [2, 0]]) }, 5), 5); expect(n.route.map(String)).toEqual(before);
  });
});
describe('homework — fleet simulation', () => {
  it('600 s of the homework warehouse: orders are delivered, no double commits, no path conflicts', () => {
    const sim = new FleetSim({ ...HOMEWORK_FLEET, duration: 600 }); const m = sim.run();
    expect(m.delivered).toBeGreaterThanOrEqual(15); expect(m.doubleCommits).toBe(0); expect(m.pathConflicts).toBe(0); expect(m.meanLatency).toBeGreaterThan(0); expect(m.utilisation).toBeGreaterThan(0.3); expect(Object.values(m.perRobot).filter((v) => v > 0).length).toBeGreaterThanOrEqual(3);
  }, 60000);
  it('stage 4: a stopped robot loses its orders to the others after lost_after; the fleet keeps delivering', () => {
    const sim = new FleetSim({ ...HOMEWORK_FLEET, duration: 500, faults: [{ robot: 'r2', at: 60, kind: 'stop' }] }); const m = sim.run();
    expect(m.delivered).toBeGreaterThanOrEqual(10); expect(m.pathConflicts).toBe(0); expect(sim.log.some((l) => /reclaims/.test(l))).toBe(true); expect(sim.robots.find((r) => r.name === 'r2')!.delivered).toBeLessThan(3);
  }, 60000);
});
