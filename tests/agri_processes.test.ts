// Acceptance (agri_mobile §5, items 11–13) and the mission process models: thinning removes unripe fruit without
// yield, scouting records observations and triggers follow-up work, spraying doses by canopy volume with tank refills,
// mowing books coverage, pruning costs time per plant; generators handle concave fields, dense crops and pergolas.
import { describe, it, expect } from 'vitest';
import { Station, ItemType, Tool, Camera } from '../src/core/items/item';
import { createRobotFromLibrary } from '../src/core/items/library';
import { MobileRobot, ZoneItem } from '../src/mobile/items';
import { FieldItem, MissionItem, MissionType, cropParams, CropRow } from '../src/agri/items';
import { generateOrchard, buildFieldMap, rectPolygon, fruitWorldPositions } from '../src/agri/orchard';
import { planMission, updateMissionProgress } from '../src/agri/missions';
import { infectPlants } from '../src/agri/process';
import { pointInPolygon } from '../src/mobile/footprint';
import { FleetItem, FleetManager } from '../src/fleet/fleet';
import { planAndFollow } from '../src/mobile/navigate';
import { mul, transl, rotz, rotx, DEG } from '../src/core/math/pose';
import { detectFruit } from '../src/agri/vision';
import { fruitCounts } from './agri_helpers';

/** Small apple block (2 x 22 m rows), a fleet, a service zone and one robot per capability set. */
function block(robots: Array<{ name: string; caps: string[]; arm?: boolean; drive?: MobileRobot['kin']['drive']; footprint?: [number, number, number] }>) {
  const st = new Station('block');
  const field = st.addChild(new FieldItem('Block'));
  field.polygon = rectPolygon(30000, 11500);
  field.crop = cropParams('apple', { rowHeading: 0, headland: 4000, ripeFraction: 0.5 });
  generateOrchard(field, 7);
  st.addChild(buildFieldMap(field, 250));
  const depot = st.addChild(new ZoneItem('Service')); depot.kind = 'loading'; depot.polygon = rectPolygon(3000, 3000, -6000, 4500);
  const fleet = st.addChild(new FleetItem('Fleet'));
  const fm = new FleetManager(st, fleet);
  const out: MobileRobot[] = [];
  robots.forEach((d, i) => {
    const m = st.addChild(new MobileRobot(d.name));
    Object.assign(m.kin, { drive: d.drive ?? 'tracked', wheelBase: 1600, track: 1300, maxSpeed: 1300, footprint: d.footprint ?? [2400, 1500, 1400] });
    m.capabilities = d.caps;
    m.setPose2D(-3000, 2000 + i * 3500, 0);
    m.home = { x: -3000, y: 2000 + i * 3500, theta: 0 };
    fm.addRobot(m);
    if (d.arm) {
      const arm = m.addChild(createRobotFromLibrary('UR10e', `${d.name} arm`));
      arm.setPose(mul(transl(0, -850, 1400), rotz(-90 * DEG)));
      const g = arm.addChild(new Tool('Gripper')); g.toolKind = 'gripper'; g.setPoseTool(transl(0, 0, 180)); arm.setTool(g);
      arm.setJoints([0, -60, 80, -110, -90, 0]);
    }
    out.push(m);
  });
  return { st, field, fleet, fm, robots: out };
}

function mission(st: Station, field: FieldItem, fleet: FleetItem, type: MissionType, settings: Record<string, any>, rows = field.rows()) {
  const m = st.addChild(new MissionItem(`${type} mission`));
  m.missionType = type; m.fieldId = field.id; m.fleetId = fleet.id; m.rowIds = rows.map((r) => r.id);
  m.settings = { workSpeed: 600, sideOffset: field.crop.rowSpacing / 2, ...settings };
  return m;
}

function runAll(fm: FleetManager, fleet: FleetItem, maxT = 6000) {
  for (let i = 0; fm.time < maxT; i++) {
    fm.step(0.1);
    if (i % 50 === 0 && fleet.tasks.length && fleet.tasks.every((t) => t.status === 'done' || t.status === 'failed' || t.type === 'charge')) break;
  }
}

describe('process models (acceptance 11)', () => {
  it('thinning removes unripe fruit above the target load, books no yield and leaves ripe fruit on the tree', () => {
    const { st, field, fleet, fm } = block([{ name: 'Picker', caps: ['harvest'], arm: true }]);
    const row = field.rows()[0];
    const m = mission(st, field, fleet, 'thin', { bothSides: false, keepFruitPerPlant: 40, secondsPerFruit: 1 }, [row]);
    const plan = planMission(st, m, fm);
    expect(plan.tasks.map((t) => t.type)).toEqual(['thin']);
    const before = fruitCounts(st);
    runAll(fm, fleet, 4000);
    expect(fleet.tasks.every((t) => t.status === 'done')).toBe(true);
    const after = fruitCounts(st);
    const k = fm.kpis();
    expect(k.yieldKg).toBe(0);
    expect(k.fruitPicked).toBe(0);
    expect(after.unripePicked - before.unripePicked).toBeGreaterThan(20);
    expect(after.picked - after.unripePicked).toBe(before.picked - before.unripePicked); // no ripe fruit removed
    expect(k.process.thin.fruitRemoved).toBe(after.unripePicked - before.unripePicked);
    updateMissionProgress(m, fm);
    // the excess fruit above the fixed-mount arm's reach stay: the mission is incomplete and says why (V3 F6)
    const t = fleet.tasks[0];
    const left = (t.meta.unreachable ?? 0) + (t.meta.armUnreachable ?? 0);
    expect(left).toBeGreaterThan(0);
    expect(m.fruit).toMatchObject({ picked: k.process.thin.fruitRemoved, left });
    expect(m.status).toBe('incomplete');
    expect(m.warnings).toContain(`Result: ${left} excess fruit stayed on the plants (out of the arms' reach)`);
  }, 120000);

  it('a harvester with a camera picks only the fruit its camera detects at each stop; misses stay on the tree', () => {
    const { st, field, fleet, fm, robots } = block([{ name: 'Picker', caps: ['harvest'], arm: true }]);
    const cam = robots[0].addChild(new Camera('Row camera'));
    cam.setPose(mul(transl(0, -800, 1800), rotx(90 * DEG)));
    cam.far = 2500;
    const row = field.rows()[0];
    const m = mission(st, field, fleet, 'harvest', { bothSides: false, secondsPerFruit: 1 }, [row]);
    const plan = planMission(st, m, fm);
    const seen = new Set<unknown>();
    const orig = fm.step.bind(fm);
    fm.step = (dt: number) => { orig(dt); if (robots[0].state.status === 'working') for (const d of detectFruit(st, cam, { onlyRipe: true, solids: [] })) seen.add(d.fruit); };
    const before = fruitCounts(st);
    runAll(fm, fleet, 5000);
    expect(plan.tasks[0].status).toBe('done');
    expect(plan.tasks[0].meta.fruitSource).toBe('vision');
    const k = fm.kpis();
    const picked = row.plants.flatMap((p) => p.fruit).filter((f) => f.picked);
    expect(picked.length).toBe(fruitCounts(st).picked - before.picked);
    expect(picked.length).toBeGreaterThan(10);
    expect(picked.every((f) => seen.has(f))).toBe(true); // nothing picked that the camera never saw
    expect(k.process.harvest.missedByVision).toBeGreaterThan(0); // reachable fruit outside the camera view / occluded
    expect(k.fruitPicked).toBe(picked.length);
  }, 120000);

  it('scouting records noisy health observations, finds the infected plants and creates a follow-up spray task', () => {
    const { st, field, fleet, fm } = block([{ name: 'Scout', caps: ['scout'] }, { name: 'Sprayer', caps: ['spray'] }]);
    const rows = field.rows();
    const sick = infectPlants(st, [15000, 4000], 2500, 0.6);
    expect(sick).toBeGreaterThan(2);
    const m = mission(st, field, fleet, 'scout', { followUp: 'spray', diseaseThreshold: 0.6, tankL: 300 });
    planMission(st, m, fm);
    runAll(fm, fleet, 6000);
    expect(m.observations.length).toBe(rows.reduce((a, r) => a + r.plants.length, 0));
    const diseased = m.observations.filter((o) => o.diseased);
    expect(diseased.length).toBeGreaterThanOrEqual(sick - 1);
    // every detection is a plant near the outbreak (noise ±0.06 cannot make a healthy plant look sick)
    for (const o of diseased) { const row = st.findById(o.rowId) as CropRow; const [x, y] = row.pointAt(row.plants[o.plant].s); expect(Math.hypot(x - 15000, y - 4000)).toBeLessThanOrEqual(2600); }
    const follow = fleet.tasks.filter((t) => t.meta?.followUpOf);
    expect(follow.length).toBeGreaterThan(0);
    expect(follow.every((t) => t.type === 'spray' && t.status === 'done')).toBe(true);
    expect(fm.kpis().process.spray.plantsTreated).toBeGreaterThan(0);
  }, 120000);

  it('spraying doses by canopy volume, refills the tank at the service zone and records the dose per plant', () => {
    const { st, field, fleet, fm, robots } = block([{ name: 'Sprayer', caps: ['spray'], drive: 'ackermann' as const, footprint: [3000, 1500, 1500] }]);
    Object.assign(robots[0].kin, { maxSteer: 30, minTurnRadius: 3000 });
    const rows = field.rows();
    const m = mission(st, field, fleet, 'spray', { bothSides: false, tankL: 5, doseLPerM3: 0.07, refillLps: 1, workSpeed: 1000 }, rows);
    planMission(st, m, fm);
    runAll(fm, fleet, 8000);
    expect(fleet.tasks.every((t) => t.status === 'done')).toBe(true);
    const k = fm.kpis();
    const len = rows.reduce((a, r) => a + r.length(), 0) / 1000;
    const lpm = (field.crop.canopyHeight / 1000) * (field.crop.canopyWidth / 1000) * 0.07;
    expect(k.process.spray.litres).toBeGreaterThan(lpm * len * 0.9);
    expect(k.process.spray.litres).toBeLessThan(lpm * len * 1.1);
    expect(k.process.spray.refills).toBeGreaterThanOrEqual(Math.floor((lpm * len) / 5) - 1);
    const sprayed = rows.flatMap((r) => r.plants).filter((p) => ((p as any).sprayedL ?? 0) > 0).length;
    expect(sprayed).toBeGreaterThan(rows.reduce((a, r) => a + r.plants.length, 0) * 0.9);
    expect(k.contacts.robotObstacleSteps).toBe(0);
  }, 180000);

  it('mowing books the swath area and the coverage of the inter-rows; pruning costs time per plant', () => {
    const { st, field, fleet, fm } = block([{ name: 'Mower', caps: ['mow'] }, { name: 'Pruner', caps: ['prune'], arm: true }]);
    const rows = field.rows();
    planMission(st, mission(st, field, fleet, 'mow', {}), fm);
    const pm = mission(st, field, fleet, 'prune', { bothSides: false, secondsPerPlant: 3 }, [rows[0]]);
    planMission(st, pm, fm);
    runAll(fm, fleet, 6000);
    expect(fleet.tasks.every((t) => t.status === 'done')).toBe(true);
    const k = fm.kpis();
    const mowArea = rows.reduce((a, r) => a + (r.length() / 1000) * (field.crop.rowSpacing / 1000), 0);
    expect(k.process.mow.areaM2).toBeGreaterThan(mowArea * 0.95);
    expect(k.process.mow.coverage).toBeGreaterThan(0.95);
    expect(k.process.prune.plants).toBe(rows[0].plants.length);
    expect(k.process.prune.seconds).toBeCloseTo(3 * rows[0].plants.length, 6);
    expect(rows[0].plants.every((p) => (p as any).prunedAt !== undefined)).toBe(true);
    const pt = fleet.tasks.find((t) => t.type === 'prune')!;
    expect(pt.finishedAt! - pt.startedAt!).toBeGreaterThan(3 * rows[0].plants.length);
  }, 180000);
});

describe('orchard generators (acceptance 12-13)', () => {
  it('a concave L-shaped field is filled (>= 90 % of area / (row x plant spacing)) with no plant outside', () => {
    const st = new Station('L');
    const f = st.addChild(new FieldItem('L'));
    f.polygon = [[0, 0], [60000, 0], [60000, 20000], [20000, 20000], [20000, 50000], [0, 50000]];
    f.crop = cropParams('apple', { rowHeading: 0, headland: 0 });
    generateOrchard(f, 3);
    const plants = f.rows().flatMap((r) => r.plants.map((p) => r.pointAt(p.s)));
    const area = 60000 * 20000 + 20000 * 30000;
    expect(plants.length).toBeGreaterThanOrEqual(0.9 * area / (3500 * 1000));
    expect(plants.filter(([x, y]) => !pointInPolygon(x, y, f.polygon)).length).toBe(0);
  });
  it('dense crops are not capped at 400 fruit per plant and fruit lie inside the canopy envelope', () => {
    const st = new Station('almond');
    const f = st.addChild(new FieldItem('Almond'));
    f.polygon = rectPolygon(30000, 20000);
    f.crop = cropParams('almond', { rowHeading: 0, headland: 5000 });
    generateOrchard(f, 4);
    const plants = f.rows().flatMap((r) => r.plants);
    const mean = plants.reduce((a, p) => a + p.fruit.length, 0) / plants.length;
    expect(mean).toBeGreaterThan(0.85 * 800);
    expect(Math.max(...plants.map((p) => p.fruit.length))).toBeGreaterThan(400);
    for (const p of plants) for (const fr of p.fruit) {
      expect(Math.hypot(fr.p[0], fr.p[1])).toBeLessThanOrEqual(p.width / 2 + 1e-6);
      expect(fr.p[2]).toBeGreaterThanOrEqual(f.crop.canopyBase - 1e-6);
      expect(fr.p[2]).toBeLessThanOrEqual(f.crop.canopyBase + p.height + 1e-6);
    }
    expect(fruitWorldPositions(f.rows()[0], false).length).toBe(f.rows()[0].plants.reduce((a, p) => a + p.fruit.length, 0));
  });
  it('kiwi pergola: a low robot drives straight from one alley to the next under the canopy, a tall one cannot enter', () => {
    const st = new Station('kiwi');
    const f = st.addChild(new FieldItem('Kiwi'));
    f.polygon = rectPolygon(40000, 30000);
    f.crop = cropParams('kiwi', { rowHeading: 0, headland: 6000 });
    generateOrchard(f, 3);
    st.addChild(buildFieldMap(f, 250));
    const rows = f.rows();
    const yA = (rows[0].start[1] + rows[1].start[1]) / 2, yB = (rows[2].start[1] + rows[3].start[1]) / 2;
    const low = st.addChild(new MobileRobot('Low'));
    Object.assign(low.kin, { drive: 'differential', footprint: [1200, 800, 1200] });
    low.setPose2D(20000, yA, 90);
    const r1 = planAndFollow(st, low, { x: 20000, y: yB });
    expect(r1.ok).toBe(true);
    expect(r1.length!).toBeLessThan((yB - yA) * 1.2); // straight across the rows, between the trunks
    // a 2.5 m tall robot (canopy base 1.8 m, canopies 5 m wide close the 5 m alleys): no way into the pergola
    const tall = st.addChild(new MobileRobot('Tall'));
    Object.assign(tall.kin, { drive: 'differential', footprint: [1200, 800, 2500] });
    tall.setPose2D(2000, yA, 0);
    const r2 = planAndFollow(st, tall, { x: 20000, y: yB });
    expect(r2.ok).toBe(false);
    low.setPose2D(2000, yA, 0);
    expect(planAndFollow(st, low, { x: 20000, y: yB }).ok).toBe(true);
  });
});
