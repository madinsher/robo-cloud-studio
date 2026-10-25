/**
 * MURKA agro worlds as studio stations (C2 buildMurkaOrchardStation / runMissionB):
 *  - orchard_demo.wbt: sectors → fields / rows measured on the tree geometry (trunk = boundingObject cylinder r 40 mm,
 *    crown z[1.06; 3.55] m, r ≤ 0.55 m as documented in the world file), Mission B roles + equipment, aerial UTP-A,
 *    2-slot depot zone; Mission B through the wrapper ends at the poses of the real Python replay (T6 reference);
 *  - murca_apple_garden.wbt (25 trees; the round-1 report's "26" was a miscount) / murca_garden.wbt: the static legacy worlds become fields with fruit (5 apples per tree
 *    from the geometry; seeded fruit in the sphere crowns of murca_garden).
 */
import { describe, it, expect, beforeAll } from 'vitest';
import fs from 'node:fs';
import path from 'node:path';
import zlib from 'node:zlib';
import { buildMurkaOrchardStation, runMissionB, crownRadiusIn, OrchardStationResult } from '../src/murka/agro';
import { addHarvestFleet } from '../src/murka/agro/orchard';
import { murkaKpiFromFleet, murkaKpiArtifact } from '../src/murka/agro/kpi';
import zlibK from 'node:zlib';
import fsK from 'node:fs';
import type { FleetManager, FleetItem } from '../src/fleet/fleet';
import { HAS_MURKA, MURKA } from './webots_helpers';
import { ItemType } from '../src/core/items/item';
import { ZoneItem } from '../src/mobile/items';
import { StationFleetWorld, shortestAngle } from '../src/io/webots';
import type { Pose4 } from '../src/io/webots';

const REF = HAS_MURKA ? JSON.parse(zlib.gunzipSync(fs.readFileSync(path.resolve(__dirname, 'fixtures/webots/mission_b_reference.json.gz'))).toString('utf8')) as { final: Record<string, Pose4> } : null;

describe.skipIf(!HAS_MURKA)(`MURKA orchard_demo station (needs ${MURKA})`, () => {
  let o: OrchardStationResult;
  beforeAll(async () => { o = await buildMurkaOrchardStation({ world: 'orchard_demo', root: MURKA }); }, 120000);

  it('4 sector fields, 8 rows of 3 trees on the 3 m grid + the single tree of sector 1, measured tree profile', () => {
    expect(o.fields.map((f) => f.name)).toEqual(['sector_1', 'sector_2', 'sector_3', 'sector_4']);
    const rows = o.fields.flatMap((f) => f.rows());
    expect(rows.filter((r) => r.plants.length === 3)).toHaveLength(8);
    expect(rows.reduce((a, r) => a + r.plants.length, 0)).toBe(25);
    for (const r of rows.filter((x) => x.plants.length === 3)) expect(r.plants.map((p) => p.s)).toEqual([0, 3000, 6000]);
    const p = o.profile!;
    expect(p.trunkRadius).toBe(40); // boundingObject Cylinder r 0.04
    // world file: crown z[1.06; 3.55] m, radius <= 0.55 m (pallet.dae scaled 0.45 x 0.45 x 1, raised by 0.2578 m)
    expect(p.crownBase).toBeGreaterThanOrEqual(1000);
    expect(p.crownBase).toBeLessThanOrEqual(1100);
    expect(p.crownTop).toBeGreaterThanOrEqual(3500);
    expect(p.crownTop).toBeLessThanOrEqual(3600);
    expect(p.crownRadius).toBeGreaterThan(500);
    expect(p.crownRadius).toBeLessThanOrEqual(550);
    // the spindle is narrow just above the crown base
    expect(crownRadiusIn(p, 1000, 1200)).toBeLessThan(crownRadiusIn(p, 2300, 2700));
    for (const f of o.fields) { expect(f.crop.trunkDiameter).toBe(80); expect(f.crop.canopyBase).toBe(p.crownBase); }
    // no fruit geometry in orchard_demo: 40 seeded fruit per tree inside the crown band
    const fruit = rows.flatMap((r) => r.plants.flatMap((pl) => pl.fruit));
    expect(fruit).toHaveLength(25 * 40);
    for (const f of fruit) { expect(f.p[2]).toBeGreaterThanOrEqual(p.crownBase); expect(Math.hypot(f.p[0], f.p[1])).toBeLessThanOrEqual(p.crownRadius); }
  });

  it('Mission B roles, modular equipment, aerial UTP-A and the 2-slot depot', () => {
    const R = o.robots;
    expect(Object.keys(R).sort()).toEqual(['UTP-A-01', 'UTP-G-01', 'UTP-G-02', 'UTP-G-03', 'UTP-G-04']);
    expect(R['UTP-G-01'].capabilities).toContain('harvest');
    expect(R['UTP-G-02'].capabilities).toContain('scout');
    expect(R['UTP-G-03'].capabilities).toContain('spray');
    expect(R['UTP-G-04'].capabilities).toContain('transport');
    expect(R['UTP-G-03'].resources.tank?.level).toBe(400);
    expect(R['UTP-A-01'].kin.drive).toBe('aerial');
    expect(R['UTP-G-01'].kin.drive).toBe('ackermann');
    expect(R['UTP-G-01'].kin.minTurnRadius).toBeCloseTo(2190 / Math.tan(0.3), 0);
    const depot = o.station.itemsOfType<ZoneItem>(ItemType.ZONE).find((z) => z.kind === 'depot')!;
    expect(depot.capacity).toBe(2);
    expect(depot.getParam('reconfigSeconds')).toBe(180);
    const [cx, cy] = depot.centroid();
    expect(cx).toBeCloseTo(-15000, -2); expect(cy).toBeCloseTo(-9800, -2);
  });

  it('runMissionB plays the garden_supervisor script: final poses = the real Python replay (1 cm / 0.01 rad), no errors', () => {
    const world = new StationFleetWorld(o.station);
    const res = runMissionB(o.station, { world, tEnd: 92 });
    expect(res.errors).toEqual([]);
    for (const [n, ref] of Object.entries(REF!.final)) {
      const p = world.getPose(n);
      expect(Math.hypot(p[0] - ref[0], p[1] - ref[1]), n).toBeLessThan(0.01);
      expect(Math.abs(p[2] - ref[2]), n).toBeLessThan(0.01);
      expect(Math.abs(shortestAngle(p[3] - ref[3])), n).toBeLessThan(0.01);
    }
  });
});

describe.skipIf(!HAS_MURKA)('legacy MURKA gardens become fields with fruit', () => {
  it('murca_apple_garden.wbt: 25 trees (DEF TREE_SHAPE + 24 USE) in 4 sectors, 5 apples each from the geometry, sphere crown z[2; 5] m', async () => {
    const o = await buildMurkaOrchardStation({ world: 'murca_apple_garden', root: MURKA });
    const rows = o.fields.flatMap((f) => f.rows());
    expect(rows.reduce((a, r) => a + r.plants.length, 0)).toBe(25);
    const fruit = rows.flatMap((r) => r.plants.flatMap((p) => p.fruit));
    expect(fruit).toHaveLength(125);
    for (const f of fruit) { expect(f.p[2]).toBeGreaterThanOrEqual(2800); expect(f.p[2]).toBeLessThanOrEqual(3400); }
    expect(o.profile!.trunkRadius).toBe(150);
    expect(o.profile!.crownBase).toBeGreaterThanOrEqual(2000); // first 100 mm band wider than 3 x trunk
    expect(o.profile!.crownBase).toBeLessThanOrEqual(2100);
    expect(o.profile!.crownRadius).toBeCloseTo(1500, -1);
  }, 120000);

  it('murca_garden.wbt: 7 sphere trees (crown r 1.2 m at 1.5 m) with seeded fruit', async () => {
    const o = await buildMurkaOrchardStation({ world: 'murca_garden', root: MURKA, fruitPerTree: 30, fruitSeed: 7 });
    const rows = o.fields.flatMap((f) => f.rows());
    expect(rows.reduce((a, r) => a + r.plants.length, 0)).toBe(7);
    expect(rows.flatMap((r) => r.plants.flatMap((p) => p.fruit))).toHaveLength(7 * 30);
    expect(o.profile!.crownRadius).toBeCloseTo(1200, -1);
    // reproducible with the seed
    const again = await buildMurkaOrchardStation({ world: 'murca_garden', root: MURKA, fruitPerTree: 30, fruitSeed: 7 });
    expect(again.fields[0].rows()[0].plants[0].fruit).toEqual(o.fields[0].rows()[0].plants[0].fruit);
  }, 120000);
});

/** Run a fleet until every task is done (or the time cap); returns the harvest counters. */
function runFleet(o: OrchardStationResult, fm: FleetManager, fleet: FleetItem, cap = 4000) {
  const fruit = () => o.fields.flatMap((f) => f.rows().flatMap((r) => r.plants.flatMap((p) => p.fruit)));
  const ripe = fruit().filter((f) => f.ripe >= 0.5 && !f.picked).length;
  while (fm.time < cap) { fm.step(0.2); if (fleet.tasks.every((t) => t.status === 'done')) break; }
  return { ripe, picked: fruit().filter((f) => f.picked).length, unripePicked: fruit().filter((f) => f.picked && f.ripe < 0.5).length, k: fm.kpis() };
}

describe.skipIf(!HAS_MURKA)('the studio harvest mission runs on the legacy MURKA gardens', () => {
  it('arm joint motion during the fleet harvest: the UR20 TCP is on each apple when it is picked, joints move smoothly', async () => {
    const o = await buildMurkaOrchardStation({ world: 'murca_apple_garden', root: MURKA });
    const { fm, fleet, robots } = addHarvestFleet(o, { robots: 1, footprint: [1800, 1000, 1900], animateArm: true });
    const arm = robots[0].children.find((c) => c.type === ItemType.ROBOT) as unknown as { joints(): number[]; poseTCPAbs(): Float64Array };
    const rows = o.fields.flatMap((f) => f.rows());
    const world = (row: (typeof rows)[number], pi: number, fi: number) => { const abs = row.parent!.poseAbs(); const [lx, ly] = row.pointAt(row.plants[pi].s); const f = row.plants[pi].fruit[fi].p; return [abs[0] * lx + abs[4] * ly + abs[12] + f[0], abs[1] * lx + abs[5] * ly + abs[13] + f[1], f[2]]; };
    const seen = new Set<string>(); const dists: number[] = []; let maxStep = 0; let prev = arm.joints();
    while (fm.time < 4000) {
      fm.step(0.2);
      const q = arm.joints(); maxStep = Math.max(maxStep, ...q.map((v, i) => Math.abs(v - prev[i]))); prev = q;
      const tcp = arm.poseTCPAbs();
      rows.forEach((row, ri) => row.plants.forEach((pl, pi) => pl.fruit.forEach((f, fi) => { const k = `${ri}:${pi}:${fi}`; if (f.picked && !seen.has(k)) { seen.add(k); const w = world(row, pi, fi); dists.push(Math.hypot(tcp[12] - w[0], tcp[13] - w[1], tcp[14] - w[2])); } })));
      if (fleet.tasks.every((t) => t.status === 'done')) break;
    }
    expect(seen.size).toBe(125);
    // at the pick time the TCP is on the apple (IK per fruit); the joints move continuously (no snap per pick)
    const onFruit = dists.filter((d) => d < 30).length;
    expect(onFruit / dists.length).toBeGreaterThan(0.9);
    expect(maxStep).toBeLessThan(40);
  }, 300000);

  it('murca_apple_garden.wbt: a 1.9 m platform (below the 2 m crown base) with a UR20 harvests all 125 apples, no contact', async () => {
    const o = await buildMurkaOrchardStation({ world: 'murca_apple_garden', root: MURKA });
    const { fm, fleet } = addHarvestFleet(o, { robots: 1, footprint: [1800, 1000, 1900] });
    expect(fleet.tasks).toHaveLength(18); // 8 three-tree rows + the single tree, both sides
    const r = runFleet(o, fm, fleet);
    expect(fleet.tasks.filter((t) => t.status !== 'done').map((t) => t.id)).toEqual([]);
    expect(r.ripe).toBe(125);
    expect(r.picked).toBe(125);
    expect(r.k.fruitPicked).toBe(r.picked);
    expect(r.k.contacts.robotObstacleSteps).toBe(0);
    expect(r.k.process.harvest?.unreachable ?? 0).toBe(0);
    // MURKA KPI exporter: an exp3-schema record of the studio run (same keys as the Python model's records)
    const rec = murkaKpiFromFleet(fm);
    const py = JSON.parse(zlibK.gunzipSync(fsK.readFileSync(path.resolve(__dirname, 'fixtures/murka_agro/article_py310/exp3_full_scenario.json.gz'))).toString()).records.murca[0];
    for (const key of Object.keys(py).filter((k) => k !== 'wallclock_s')) expect(rec, key).toHaveProperty(key);
    expect(rec.coverage_percent).toBe(100);
    expect(rec.missed_tasks).toEqual([]);
    expect(rec.completed_tasks).toHaveLength(18);
    expect(rec.total_distance).toBeCloseTo(r.k.distanceTravelled / 1000, 9);
    expect(rec.total_energy).toBeCloseTo(0.2 * rec.total_distance, 9);
    expect(rec.studio_fruit_picked).toBe(125);
    const art = murkaKpiArtifact([rec]) as { modes: string[]; aggregate: Record<string, { avg_coverage_percent: number }> };
    expect(art.aggregate[art.modes[0]].avg_coverage_percent).toBe(100);
  }, 300000);

  it('murca_garden.wbt: seeded fruit in the sphere crowns; every ripe fruit is picked or reported unreachable, no contact', async () => {
    const o = await buildMurkaOrchardStation({ world: 'murca_garden', root: MURKA, fruitPerTree: 30 });
    expect(o.profile!.crownBase).toBeLessThanOrEqual(300); // bare sphere from 0.3 m: no trunk
    const { fm, fleet } = addHarvestFleet(o, { robots: 1 });
    const r = runFleet(o, fm, fleet);
    expect(fleet.tasks.filter((t) => t.status !== 'done').map((t) => t.id)).toEqual([]);
    expect(r.k.fruitPicked).toBe(r.picked);
    expect(r.unripePicked).toBe(0);
    expect(r.picked + (r.k.process.harvest?.unreachable ?? 0)).toBe(r.ripe);
    expect(r.picked).toBeGreaterThan(r.ripe * 0.7);
    expect(r.k.contacts.robotObstacleSteps).toBe(0);
  }, 300000);
});
