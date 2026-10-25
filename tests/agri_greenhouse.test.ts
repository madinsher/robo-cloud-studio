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
import { FieldItem } from '../src/agri/items';
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
    while (fm.time < 2400) {
      fm.step(0.2);
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
  }, 300000);
});
