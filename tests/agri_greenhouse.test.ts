/**
 * Greenhouse layout (src/agri/greenhouse.ts) and pipe-rail trolleys (src/mobile/rails.ts, navigation method rail_1d):
 * gutters along the rows, a pair of heating pipes in every path, a corridor across the row heads; the greenhouse demo's
 * trolleys harvest with the FleetManager while staying on the rail network — on a rail they only drive along its axis
 * (never turning on it), they turn on the corridor only.
 */
import { describe, it, expect } from 'vitest';
import { demos } from '../src/demos';
import { ItemType, SceneObject } from '../src/core/items/item';
import { MobileRobot } from '../src/mobile/items';
import { FleetItem, FleetManager } from '../src/fleet/fleet';
import { FieldItem, CropRow, MissionItem } from '../src/agri/items';
import { planMission } from '../src/agri/missions';
import { armOf, armLift } from '../src/agri/harvest';
import { fruitWorldPositions } from '../src/agri/orchard';
import { RailNetwork, railRoute, locateOnRails, projectOnSegment } from '../src/mobile/rails';
import { fruitCounts } from './agri_helpers';

const wrap180 = (a: number) => ((((a + 180) % 360) + 360) % 360) - 180;

describe('rail network routing', () => {
  const net: RailNetwork = { corridor: [[0, 0], [0, 20000]], corridorWidth: 2400, rails: [[[0, 4000], [30000, 4000]], [[0, 8000], [30000, 8000]]] };
  it('rail → corridor (right-hand lane) → other rail; same rail stays on it', () => {
    const r = railRoute(net, [20000, 4000], [10000, 8000]);
    // leaves rail 1 along its axis, runs the corridor lane (x = +600 when going +y: right of the travel direction), enters rail 2
    expect(r.map((p) => p.map(Math.round))).toEqual([[20000, 4000], [600, 4000], [600, 8000], [0, 8000], [10000, 8000]].map((p) => p));
    const back = railRoute(net, [10000, 8000], [20000, 4000]);
    expect(back[1]).toEqual([-600, 8000]); // going −y: the other lane
    expect(railRoute(net, [5000, 4000], [25000, 4000])).toEqual([[5000, 4000], [25000, 4000]]);
    expect(locateOnRails(net, [12000, 8100]).rail).toBe(1);
  });
});

describe('greenhouse demo: gutters, heating-pipe rails, rail-constrained trolleys', () => {
  it('builds one gutter per row and pipes per path; trolleys harvest staying on the rail network', () => {
    const st = demos.find((d) => d.id === 'greenhouse')!.build();
    const field = st.itemsOfType<FieldItem>(ItemType.FIELD)[0];
    const rows = field.rows();
    const objs = st.children.filter((c): c is SceneObject => c instanceof SceneObject);
    expect(objs.filter((o) => o.getParam('greenhouse') === 'gutter')).toHaveLength(rows.length);
    expect(objs.filter((o) => o.getParam('greenhouse') === 'pipe_rail')).toHaveLength(rows.length + 1);
    const net = field.getParam('railNetwork') as unknown as RailNetwork;
    expect(net.rails).toHaveLength(rows.length + 1);
    const fleet = st.itemsOfType<FleetItem>(ItemType.FLEET)[0];
    const fm = new FleetManager(st, fleet);
    const robots = st.itemsOfType<MobileRobot>(ItemType.MOBILE_ROBOT);
    const before = fruitCounts(st);
    let off = 0, turnOnRail = 0, samples = 0;
    const prev = new Map(robots.map((r) => [r.id, { x: r.state.x, y: r.state.y, th: r.state.theta }]));
    const onRailOnly = (x: number, y: number) => { const loc = locateOnRails(net, [x, y]); const c = projectOnSegment([x, y], net.corridor[0], net.corridor[1]); return loc.rail >= 0 && c.d > net.corridorWidth / 2; };
    // the pickers on their scissor lifts: TCP at every fruit when it is removed, lift travel, joint configurations
    const arms = robots.map((r) => armOf(r)!);
    const dirty = new Set<CropRow>();
    st.events.on('changed', ({ item, what }: { item: unknown; what: string }) => { if (what === 'plants' && item instanceof CropRow) dirty.add(item); });
    const seen = new Set<unknown>();
    const tcpMiss: number[] = [];
    const configs = new Set<string>();
    let liftMax = 0;
    while (fm.time < 3600) {
      fm.step(0.2);
      for (const a of arms) { configs.add(`${a.id}:${a.joints().map((v) => v.toFixed(0)).join(',')}`); liftMax = Math.max(liftMax, armLift(a).h); }
      for (const row of dirty) for (const w of fruitWorldPositions(row, false)) {
        if (!w.fruit.picked || seen.has(w.fruit)) continue;
        seen.add(w.fruit);
        const d = Math.min(...arms.map((a) => { const t = a.poseTCPAbs(); return Math.hypot(t[12] - w.p[0], t[13] - w.p[1], t[14] - w.p[2]); }));
        if (d > 20) tcpMiss.push(Math.round(d));
      }
      dirty.clear();
      for (const r of robots) {
        samples++;
        const loc = locateOnRails(net, [r.state.x, r.state.y]);
        const homeLeg = r.home && Math.abs(r.state.x - r.home.x) < 5 && r.state.y < Math.min(net.corridor[0][1], net.corridor[1][1]);
        if (loc.d > 60 && !homeLeg) off++;
        const p = prev.get(r.id)!;
        if (onRailOnly(r.state.x, r.state.y) && Math.abs(wrap180(r.state.theta - p.th)) > 1 && Math.abs(wrap180(r.state.theta)) > 2 && Math.abs(wrap180(r.state.theta - 180)) > 2) turnOnRail++;
        prev.set(r.id, { x: r.state.x, y: r.state.y, th: r.state.theta });
      }
    }
    const k = fm.kpis();
    const after = fruitCounts(st);
    expect(samples).toBeGreaterThan(10000);
    expect(off).toBe(0); // always on a rail axis (± 60 mm) or on the corridor
    expect(turnOnRail).toBe(0); // headings on a rail are the rail axis (0° / 180°)
    expect(k.fruitPicked).toBeGreaterThan(0);
    expect(k.fruitPicked).toBe(after.picked - before.picked);
    expect(fleet.tasks.filter((t) => t.status === 'done').length).toBeGreaterThan(0);
    expect(k.contacts.robotRobotSteps).toBe(0);
    // V3 F2: the pickers move to every tomato they remove and use their lifts
    expect(seen.size).toBe(k.fruitPicked);
    expect(tcpMiss).toEqual([]);
    expect(liftMax).toBeGreaterThan(1500);
    expect(configs.size).toBeGreaterThan(1000);
  }, 300000);

  it('unloading trips mid-row: the trolley keeps its rail reserved until it is out, the other trolley is not let in head-on', () => {
    // two rows sharing the rail between them (Row 9 R / Row 10 L) and small bins: frequent unloading trips back along
    // the rail; before, a trip released the rail reservation while the trolley was still on it and both stalled head-on
    const st = demos.find((d) => d.id === 'greenhouse')!.build();
    const fleet = st.itemsOfType<FleetItem>(ItemType.FLEET)[0];
    fleet.tasks.length = 0;
    st.itemsOfType<MissionItem>(ItemType.MISSION)[0].delete();
    const field = st.itemsOfType<FieldItem>(ItemType.FIELD)[0];
    const fm = new FleetManager(st, fleet);
    const m = st.addChild(new MissionItem('Rows 9-10'));
    m.missionType = 'harvest'; m.fieldId = field.id; m.fleetId = fleet.id; m.rowIds = field.rows().slice(8, 10).map((r) => r.id);
    m.settings = { workSpeed: 300, bothSides: true, sideOffset: field.crop.rowSpacing / 2, secondsPerFruit: 1, animateArm: false, binCapacityKg: 5 };
    planMission(st, m, fm);
    expect(fleet.tasks).toHaveLength(4);
    while (fm.time < 14000 && fleet.tasks.some((t) => t.status !== 'done')) fm.step(0.2);
    expect(fleet.tasks.map((t) => t.status)).toEqual(['done', 'done', 'done', 'done']);
    expect(fm.kpis().process.harvest.unloads).toBeGreaterThan(10);
    expect(fm.kpis().contacts.robotRobotSteps).toBe(0);
  }, 300000);
});
