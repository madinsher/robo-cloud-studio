/**
 * Group FSM templates over a fleet (agromarca-like): capability ontology import, per-robot instances of one FSM,
 * messages and the contract net (PROPOSED → ACCEPTED / REJECTED → EXECUTING → COMPLETED). Assertions on robot motion,
 * contract history and timing.
 */
import { describe, it, expect } from 'vitest';
import { Station, ItemType } from '../src/core/items/item';
import { MobileRobot, ZoneItem } from '../src/mobile/items';
import { parseFsm, checkFsm, fsmToDsl } from '../src/ctl/fsm';
import { makeStationWorld } from '../src/ctl/world';
import { simulatedBackend } from '../src/ctl/host';
import { GroupFsmExecutor, GARDEN_GROUP_FSM, parseCapabilityOntology, applyCapabilityOntology, parseYamlLite } from '../src/ctl/group_fsm';

const ONTOLOGY = `# garden robots (murca_ontology style)
robot_types:
  UTP-G:
    capabilities: [move_ground, carry]
  UTP-A: {capabilities: [fly, survey]}
equipment:
  sprayer:
    capabilities: [spray]
    slot: Top
  harvester_arm: {capabilities: [harvest, prune], slot: Front}
robots:
  G1: {type: UTP-G, equipment: [sprayer]}
  G2:
    type: UTP-G
    equipment:
      - sprayer
      - harvester_arm
  A1: {type: UTP-A}
`;

function garden() {
  const st = new Station('garden'); const zone = (n: string, x: number, y: number) => { const z = st.addChild(new ZoneItem(n)); z.polygon = [[x - 1500, y - 1500], [x + 1500, y - 1500], [x + 1500, y + 1500], [x - 1500, y + 1500]]; return z; };
  zone('base', 0, 0); zone('S1', 30000, 0); zone('S2', 0, 20000);
  const robot = (n: string, x: number, y: number) => { const r = st.addChild(new MobileRobot(n)); r.setPose2D(x, y, 0); r.home = { x, y, theta: 0 }; return r; };
  return { st, G1: robot('G1', 0, 3000), G2: robot('G2', 22000, 0), A1: robot('A1', 0, -3000) };
}

describe('capability ontology import', () => {
  it('robot types + equipment give each robot its capabilities; unknown references are errors', () => {
    expect(parseYamlLite('a: 1\nb:\n  - x\n  - y\nc: {d: [1, 2]}')).toEqual({ a: 1, b: ['x', 'y'], c: { d: [1, 2] } });
    const caps = parseCapabilityOntology(ONTOLOGY);
    expect(caps).toEqual({ G1: ['move_ground', 'carry', 'spray'], G2: ['move_ground', 'carry', 'spray', 'harvest', 'prune'], A1: ['fly', 'survey'] });
    expect(parseCapabilityOntology('{"R1": ["spray"]}')).toEqual({ R1: ['spray'] });
    expect(() => parseCapabilityOntology('robots:\n  X: {type: Nope}')).toThrow(/unknown type Nope/);
    const { st } = garden(); const r = applyCapabilityOntology(st, ONTOLOGY + '  B9: {type: UTP-A}\n');
    expect(r.updated.map((u) => u.robot)).toEqual(['G1', 'G2', 'A1']); expect(r.missing).toEqual(['B9']);
    expect(st.itemsOfType<MobileRobot>(ItemType.MOBILE_ROBOT).find((x) => x.name === 'A1')!.capabilities).toEqual(['fly', 'survey']);
  });
});

describe('group FSM over the fleet', () => {
  it('the scout announces a spray task at S1, the closest sprayer wins and executes it, the other is rejected and stays', () => {
    const { st, G1, G2, A1 } = garden(); applyCapabilityOntology(st, ONTOLOGY);
    const doc = parseFsm(GARDEN_GROUP_FSM); expect(doc.group).toEqual({ all: true }); expect(checkFsm(doc).filter((i) => i.level === 'error')).toEqual([]);
    expect(parseFsm(fsmToDsl(doc)).group).toEqual({ all: true });
    const world = makeStationWorld({ station: st, programs: simulatedBackend({}) });
    const g = new GroupFsmExecutor({ doc, world, station: st });
    const g1 = [G1.state.x, G1.state.y]; let a1AtS1 = -1, g2AtS1 = -1; let g2Moved = -1;
    const inZone = (r: MobileRobot, z: string) => st.itemsOfType<ZoneItem>(ItemType.ZONE).find((x) => x.name === z)!.contains(r.state.x, r.state.y);
    for (let i = 0; i < 3000 && g.status === 'running'; i++) { g.tick(0.1); if (a1AtS1 < 0 && inZone(A1, 'S1')) a1AtS1 = g.time; if (g2AtS1 < 0 && inZone(G2, 'S1')) g2AtS1 = g.time; if (g2Moved < 0 && Math.hypot(G2.state.x - 22000, G2.state.y) > 50) g2Moved = g.time; }
    expect(g.status).toBe('idle'); expect(g.members.map((m) => m.exec.outcome)).toEqual(['done', 'done', 'done']);
    const c = g.contracts[0]; expect(g.contracts.length).toBe(1);
    expect(c.history.map((h) => `${h.state}${h.robot ? ':' + h.robot : ''}`)).toEqual(['PROPOSED:A1', 'ACCEPTED:G2', 'REJECTED:G1', 'EXECUTING:G2', 'COMPLETED:G2']);
    expect(c.bids.map((b) => b.robot).sort()).toEqual(['G1', 'G2']); expect(c.bids.find((b) => b.robot === 'G2')!.cost).toBeLessThan(c.bids.find((b) => b.robot === 'G1')!.cost);
    // order and timing: the scout reaches S1, then announces; G2 starts only after the award; it sprays ≥ 3 s before completing
    const t = (s: string) => c.history.find((h) => h.state === s)!.t;
    expect(a1AtS1).toBeGreaterThan(0); expect(t('PROPOSED')).toBeGreaterThanOrEqual(a1AtS1); expect(g2Moved).toBeGreaterThanOrEqual(t('ACCEPTED'));
    expect(g2AtS1).toBeGreaterThan(t('EXECUTING')); expect(t('COMPLETED') - g2AtS1).toBeGreaterThanOrEqual(3 - 0.2);
    expect(inZone(G2, 'S1')).toBe(true); expect(Math.hypot(G1.state.x - g1[0], G1.state.y - g1[1])).toBeLessThan(10); expect(inZone(A1, 'base')).toBe(true);
    expect(g.messages.map((m) => `${m.from}->${m.to}:${m.msg}`)).toEqual(['A1->G2:awarded', 'A1->G1:rejected', 'G2->A1:task_done']);
  });
  it('no capable bidder: the contract is cancelled and the announcer is told; `group capability=` selects the members', () => {
    const { st } = garden(); applyCapabilityOntology(st, ONTOLOGY);
    const src = GARDEN_GROUP_FSM.replace('cap=spray', 'cap=pollinate');
    const g = new GroupFsmExecutor({ doc: parseFsm(src), world: makeStationWorld({ station: st, programs: simulatedBackend({}) }), station: st }); g.run(300);
    expect(g.contracts[0].history.map((h) => h.state)).toEqual(['PROPOSED', 'CANCELLED']); expect(g.messages.map((m) => m.msg)).toEqual(['no_bidder']);
    const only = new GroupFsmExecutor({ doc: parseFsm(GARDEN_GROUP_FSM.replace('group all', 'group capability=spray')), world: makeStationWorld({ station: st, programs: simulatedBackend({}) }), station: st });
    expect(only.members.map((m) => m.name)).toEqual(['G1', 'G2']);
    expect(() => new GroupFsmExecutor({ doc: parseFsm(GARDEN_GROUP_FSM.replace('group all', 'group robots=G1,Z9')), world: makeStationWorld({ station: st, programs: simulatedBackend({}) }), station: st })).toThrow(/Z9 is not on the station/);
  });
});
