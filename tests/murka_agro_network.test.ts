/**
 * Contract-net message model (latency, seeded loss, retransmission): the S1 dispatcher (D1 / D2) and the ctl group FSM
 * contract net (bid hook + network option). Default parameters (no latency, no loss) are covered by the S1 / group FSM
 * tests; here: assignments arrive exactly one latency after the decision, announce + bid take 2 latencies, lost
 * messages are counted and retransmitted, the mission still completes with every row harvested.
 */
import { describe, it, expect, beforeAll } from 'vitest';
import { Station, ItemType } from '../src/core/items/item';
import { MobileRobot, ZoneItem } from '../src/mobile/items';
import { parseFsm } from '../src/ctl/fsm';
import { makeStationWorld } from '../src/ctl/world';
import { simulatedBackend } from '../src/ctl/host';
import { GroupFsmExecutor, GARDEN_GROUP_FSM, applyCapabilityOntology } from '../src/ctl/group_fsm';
import { murkaCost } from '../src/fleet/murka_alloc';
import { buildMurkaOrchardStation, OrchardStationResult } from '../src/murka/agro';
import { runS1Benchmark } from '../src/murka/agro/s1';
import { HAS_MURKA, MURKA } from './webots_helpers';

const ONTOLOGY = `robot_types:
  UTP-G: {capabilities: [move_ground, carry]}
  UTP-A: {capabilities: [fly, survey]}
equipment:
  sprayer: {capabilities: [spray], slot: Top}
robots:
  G1: {type: UTP-G, equipment: [sprayer]}
  G2: {type: UTP-G, equipment: [sprayer]}
  A1: {type: UTP-A}
`;
function garden() {
  const st = new Station('garden');
  const zone = (n: string, x: number, y: number) => { const z = st.addChild(new ZoneItem(n)); z.polygon = [[x - 1500, y - 1500], [x + 1500, y - 1500], [x + 1500, y + 1500], [x - 1500, y + 1500]]; return z; };
  zone('base', 0, 0); zone('S1', 30000, 0); zone('S2', 0, 20000);
  const robot = (n: string, x: number, y: number) => { const r = st.addChild(new MobileRobot(n)); r.setPose2D(x, y, 0); r.home = { x, y, theta: 0 }; return r; };
  const R = { G1: robot('G1', 0, 3000), G2: robot('G2', 22000, 0), A1: robot('A1', 0, -3000) };
  applyCapabilityOntology(st, ONTOLOGY);
  return { st, ...R };
}
const group = (st: Station, extra: Partial<ConstructorParameters<typeof GroupFsmExecutor>[0]> = {}) => new GroupFsmExecutor({ doc: parseFsm(GARDEN_GROUP_FSM), world: makeStationWorld({ station: st, programs: simulatedBackend({}) }), station: st, ...extra });

describe('group FSM contract net: bid hook and network model', () => {
  it('announce + bid travel 2 × latency: the award comes ≥ 2 s after the announcement with 1 s latency', () => {
    const { st } = garden();
    const g = group(st, { network: { latency: 1 } });
    g.run(400);
    const c = g.contracts[0];
    const t = (s: string) => c.history.find((h) => h.state === s)!.t;
    expect(t('ACCEPTED') - t('PROPOSED')).toBeGreaterThanOrEqual(2 - 1e-9);
    expect(t('ACCEPTED') - t('PROPOSED')).toBeLessThan(2 + 0.1 + 1e-9);
    expect(g.netStats).toEqual({ announced: 1, bids: 2, lost: 0 });
  });
  it('every bid lost (loss = 1): the contract is cancelled, the losses are counted', () => {
    const { st } = garden();
    const g = group(st, { network: { loss: 1, seed: 3 } });
    g.run(300);
    expect(g.contracts[0].history.map((h) => h.state)).toEqual(['PROPOSED', 'CANCELLED']);
    expect(g.netStats.bids).toBe(2);
    expect(g.netStats.lost).toBe(2);
  });
  it('bid hook: with the MuRKA cost (1) the drained robot loses although it is closer (the default bid picks it)', () => {
    const run = (hook: boolean) => {
      const { st, G2 } = garden();
      G2.setPose2D(8000, 0, 0); // 22 m from S1 (G1: 30.1 m)
      G2.battery.levelWh = 0.05 * G2.battery.capacityWh;
      const zoneAt = (name: string) => st.itemsOfType<ZoneItem>(ItemType.ZONE).find((z) => z.name === name)!.centroid();
      const g = group(st, hook ? { bid: (m, c) => { const [x, y] = zoneAt(c.zone); return murkaCost({ id: m.name, x: m.robot.state.x / 1000, y: m.robot.state.y / 1000, battery: m.robot.batteryLevel(), caps: new Set(m.caps) }, { id: c.task, x: x / 1000, y: y / 1000, caps: new Set([c.cap]), priority: 1 }); } } : {});
      g.run(600);
      return g.contracts[0];
    };
    const murka = run(true), dflt = run(false);
    const cost = (c: typeof murka, r: string) => c.bids.find((b) => b.robot === r)!.cost;
    // MuRKA: d + 10·(1 − b): G2 ≈ 22 + 9.5 > G1 ≈ 30.15 + small → G1; default d + 5·(1 − b): G2 ≈ 26.75 < G1 → G2
    expect(cost(murka, 'G2')).toBeGreaterThan(31.4);
    expect(cost(murka, 'G1')).toBeLessThan(30.5);
    expect(murka.winner).toBe('G1');
    expect(dflt.winner).toBe('G2');
  });
});

describe.skipIf(!HAS_MURKA)('S1 dispatcher with message latency and loss', () => {
  let o: OrchardStationResult;
  beforeAll(async () => { o = await buildMurkaOrchardStation({ world: 'orchard_demo', root: MURKA }); }, 120000);

  it('latency 0.5 s, no loss: every assignment arrives exactly one latency after its decision; announce + bid add one more each', () => {
    const r = runS1Benchmark(o.station, { level: 'A', controller: 'contract', seed: 1, assets: o.assets, contacts: false, params: { msgLatency: 0.5 } });
    expect(r.metrics.outcome).toBe('done');
    const assigned = r.events.filter((e) => e.type === 'row_assigned' || e.type === 'swap_request');
    expect(assigned.length).toBeGreaterThan(8);
    for (const e of r.events.filter((x) => x.type === 'row_assigned')) expect(e.t - Number(e.data!.decided_at)).toBeCloseTo(0.5, 3);
    // the first awards come 3 latencies after the survey map (announce, bid, award)
    expect(r.events.find((e) => e.type === 'row_assigned')!.t - r.metrics.surveyDone).toBeCloseTo(1.5, 3);
    expect(r.metrics.comm.lost).toBe(0);
    expect(r.metrics.comm.messages).toBe(r.metrics.comm.announce + r.metrics.comm.bids + r.metrics.comm.awards);
  }, 300000);

  it('30 % message loss: losses counted and retransmitted, every row harvested, all kg delivered (both controllers)', () => {
    for (const controller of ['contract', 'hungarian'] as const) {
      const r = runS1Benchmark(o.station, { level: 'A', controller, seed: 2, assets: o.assets, contacts: false, params: { msgLatency: 0.2, msgLoss: 0.3 } });
      const m = r.metrics;
      expect(m.outcome, controller).toBe('done');
      expect(m.comm.lost).toBeGreaterThan(0);
      expect(m.comm.retransmissions).toBeGreaterThan(0);
      expect([...Object.values(m.rowsByHarvester).flat()].sort()).toEqual(['r1', 'r2', 'r3', 'r4', 'r5', 'r6', 'r7', 'r8']);
      expect(m.completeness).toBeCloseTo(1, 12);
      expect(m.boxesDelivered).toBe(m.boxesExpected);
    }
  }, 300000);
});
