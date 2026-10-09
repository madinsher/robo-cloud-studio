import { describe, it, expect } from 'vitest';
import { PetriNet, analysePetriNet, preventDeadlocks, bankerSafe, bankerRequest, resourceOrder, simulateTimed, analyseGspn, buildS3PR, farkas } from '../src/ctl/petri';
import { bottleneckBound, saturationCurve, littlesLaw, steadyState, maxCycleMean, cycleTimeFromOperations, simulateSerialLine, eventGraphCycleTime } from '../src/ctl/perf';
import { CELL_DEADLOCK_NET, CELL_S3PR, CELL_LOADS, CELL_OPERATIONS, MACHINE_GSPN } from '../src/ctl/examples';

describe('Petri nets: course example B cell fragment', () => {
  it('finds resource invariants, the bad siphon {rM, rZ, a2, b2} and the deadlock trace', () => {
    const net = new PetriNet(CELL_DEADLOCK_NET);
    const a = analysePetriNet(net);
    expect(a.bounded).toBe(true); expect(a.safe).toBe(true);
    expect(a.pInvariants).toContain('M(a1) + M(a2) + M(b2) + M(rM) = 1');
    expect(a.pInvariants).toContain('M(a2) + M(b1) + M(b2) + M(rZ) = 1');
    expect(a.coveredByPInvariants).toBe(true);
    expect(a.live).toBe(false);
    expect(a.deadlocks.length).toBe(1);
    expect(a.deadlocks[0].marking).toBe('a1 b1');
    expect(a.deadlocks[0].trace.sort()).toEqual(['t1a', 't1b']);
    const bad = a.badSiphons.find((s) => s.places.join() === 'a2,b2,rM,rZ');
    expect(bad).toBeDefined(); expect(bad!.emptyAt).toBe('a1 b1'); expect(bad!.controlled).toBe(false);
    expect(a.tInvariants.length).toBe(2);
  });
  it('adds a GMEC monitor for the siphon and the net becomes live', () => {
    const net = new PetriNet(CELL_DEADLOCK_NET);
    const r = preventDeadlocks(net);
    expect(r.live).toBe(true); expect(r.monitors.length).toBe(1);
    expect(r.monitors[0].constraint).toBe('a1 + b1 ≤ 1'); // thieves of the siphon resources
    const a = analysePetriNet(r.net);
    expect(a.deadlocks.length).toBe(0); expect(a.reversible).toBe(true);
    const V = r.net.places.find((p) => p.kind === 'monitor')!;
    expect(V.tokens).toBe(1);
  });
  it('S3PR builder reproduces the same deadlock; resource ordering explains it', () => {
    const net = buildS3PR(CELL_S3PR);
    const a = analysePetriNet(net);
    expect(a.live).toBe(false); expect(a.badSiphons.length).toBeGreaterThan(0);
    const ord = resourceOrder([['M1', 'Z'], ['Z', 'M1']]);
    expect(ord.ok).toBe(false); expect(ord.cycle).toEqual(['M1', 'Z', 'M1']);
    expect(resourceOrder([['M1', 'Z'], ['M1', 'Z']])).toMatchObject({ ok: true, order: ['M1', 'Z'] });
  });
  it('banker: unsafe request deferred, safe granted', () => {
    // 2 resources (M1, Z) capacity 1 each; P1 holds M1 needs Z; P2 holds nothing needs Z then M1
    expect(bankerSafe([0, 1], [[1, 0], [0, 0]], [[0, 1], [1, 1]])).toMatchObject({ safe: true, order: [0, 1] });
    expect(bankerRequest([0, 1], [[1, 0], [0, 0]], [[0, 1], [1, 1]], 1, [0, 1]).granted).toBe(false);
    expect(bankerRequest([0, 1], [[1, 0], [0, 0]], [[0, 1], [1, 1]], 0, [0, 1]).granted).toBe(true);
  });
  it('timed simulation deadlocks the raw net and runs the monitored one', () => {
    const net = new PetriNet(CELL_DEADLOCK_NET);
    const s = simulateTimed(net, 200, { seed: 3 });
    expect(s.deadlockAt).not.toBeNull();
    const fixed = preventDeadlocks(net).net;
    const s2 = simulateTimed(fixed, 500, { seed: 3 });
    expect(s2.deadlockAt).toBeNull(); expect(s2.firings['t3a']).toBeGreaterThan(10); expect(s2.firings['t3b']).toBeGreaterThan(10);
  });
  it('farkas finds minimal invariants of a simple cycle', () => {
    expect(farkas([[-1, 1], [1, -1]])).toEqual([[1, 1]]);
  });
});

describe('Performance (chapter 5)', () => {
  it('bottleneck bound: machine S, 22 s, 163.6 parts/h; second machine moves the bottleneck to 11 s', () => {
    const b = bottleneckBound(CELL_LOADS);
    expect(b.bottleneck).toBe('S'); expect(b.cycleTime).toBe(22); expect(b.throughputPerHour).toBeCloseTo(163.6, 1);
    const sat = saturationCurve(CELL_LOADS, 'S', 3);
    expect(sat[1].cycleTime).toBe(11); expect(sat[2].cycleTime).toBe(10); expect(sat[2].bottleneck).toBe('Z');
    expect(littlesLaw({ throughput: 100 / 3600, leadTime: 90 }).wip).toBeCloseTo(2.5);
  });
  it('GSPN machine with failures: π = (0.1546, 0.6763, 0.1691), 110.7 parts/h', () => {
    const g = analyseGspn(new PetriNet(MACHINE_GSPN));
    const p = Object.fromEntries(g.pi.map((x) => [x.marking, x.p]));
    expect(p['idle']).toBeCloseTo(0.1546, 3); expect(p['work']).toBeCloseTo(0.6763, 3); expect(p['fail']).toBeCloseTo(0.1691, 3);
    expect(g.throughput['finish'] * 3600).toBeCloseTo(110.7, 0);
    const Q = [[-0.2, 0.2, 0], [1 / 22, -(1 / 22 + 1 / 3600), 1 / 3600], [1 / 900, 0, -1 / 900]];
    expect(steadyState(Q)[1]).toBeCloseTo(0.6763, 3);
  });
  it('GSPN with immediate choice eliminates vanishing markings', () => {
    const net = new PetriNet({ name: 'g', places: [{ id: 'p0', tokens: 1 }, { id: 'p1' }, { id: 'p2' }, { id: 'p3' }],
      transitions: [{ id: 'arrive', rate: 1 }, { id: 'toA', immediate: true, weight: 3 }, { id: 'toB', immediate: true, weight: 1 }, { id: 'serveA', rate: 2 }, { id: 'serveB', rate: 2 }],
      arcs: [{ from: 'p0', to: 'arrive' }, { from: 'arrive', to: 'p1' }, { from: 'p1', to: 'toA' }, { from: 'p1', to: 'toB' }, { from: 'toA', to: 'p2' }, { from: 'toB', to: 'p3' }, { from: 'p2', to: 'serveA' }, { from: 'p3', to: 'serveB' }, { from: 'serveA', to: 'p0' }, { from: 'serveB', to: 'p0' }] });
    const g = analyseGspn(net);
    expect(g.vanishing).toBe(1); expect(g.tangible).toBe(3);
    expect(g.throughput['serveA'] / g.throughput['serveB']).toBeCloseTo(3, 5);
  });
  it('max-plus: Karp cycle mean and the cell cycle time 32 s (112.5 parts/h); 2 machines → 16 s, 3 → 10.67 s', () => {
    const k = maxCycleMean(3, [[{ to: 1, w: 2 }], [{ to: 2, w: 3 }, { to: 0, w: 1 }], [{ to: 0, w: 4 }]]);
    expect(k.mean).toBeCloseTo(3, 9); // cycle 0→1→2→0: (2+3+4)/3 = 3 vs 0→1→0: 1.5
    const c = cycleTimeFromOperations(CELL_OPERATIONS);
    expect(c.cycleTime).toBeCloseTo(32, 6); expect(c.throughputPerHour).toBeCloseTo(112.5, 1);
    expect(cycleTimeFromOperations(CELL_OPERATIONS, { S: 2 }).cycleTime).toBeCloseTo(16, 6);
    expect(cycleTimeFromOperations(CELL_OPERATIONS, { S: 3 }).cycleTime).toBeCloseTo(32 / 3, 6);
    expect(cycleTimeFromOperations(CELL_OPERATIONS, { S: 4 }).cycleTime).toBeCloseTo(10, 6);
    expect(eventGraphCycleTime(['a', 'b'], [{ from: 'a', to: 'b', delay: 3, tokens: 0 }, { from: 'b', to: 'a', delay: 2, tokens: 1 }]).cycleTime).toBeCloseTo(5, 6);
  });
  it('Monte Carlo serial line: variability reduces throughput below the deterministic bound', () => {
    const det = simulateSerialLine([{ id: 'M1', mean: 9 }, { id: 'S', mean: 22 }, { id: 'M2', mean: 9 }], [1, 1], 200);
    expect(det.throughputPerHour).toBeCloseTo(163.6, -1);
    const bal = simulateSerialLine([{ id: 'A', mean: 10 }, { id: 'B', mean: 10 }, { id: 'C', mean: 10 }], [0, 0], 400, 7);
    const rnd = simulateSerialLine([{ id: 'A', mean: 10, cv: 0.3 }, { id: 'B', mean: 10, cv: 0.3 }, { id: 'C', mean: 10, cv: 0.3 }], [0, 0], 400, 7);
    expect(bal.throughputPerHour).toBeCloseTo(360, -1);
    expect(rnd.throughputPerHour).toBeLessThan(0.9 * bal.throughputPerHour);
    expect(rnd.cycleTime.ci95).toBeGreaterThan(0);
  });
});
