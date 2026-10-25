// Acceptance (agri_mobile §5, items 1–3): the orchard demo as the app loads it — a new FleetManager on the station
// built by the demo — runs its harvest and spray missions to the end without any robot touching a tree or another
// robot, books exactly the fruit it removed, and the picking arm always faces the row it harvests; (4) the arms move to
// every fruit they pick within their joint speed limits, (5) the harvest mission ends done because no ripe fruit is
// left on rows 1-4 (V3 findings F2 / F6).
import { describe, it, expect } from 'vitest';
import { demos } from '../src/demos';
import { FleetItem, FleetManager } from '../src/fleet/fleet';
import { ItemType } from '../src/core/items/item';
import { MobileRobot } from '../src/mobile/items';
import { CropRow, MissionItem } from '../src/agri/items';
import { armOf, armReach, armLift } from '../src/agri/harvest';
import { fruitWorldPositions } from '../src/agri/orchard';
import { updateMissionProgress } from '../src/agri/missions';
import { plantDisks, cropClearance, footprintsOverlap, fruitCounts, rowAxisDistance } from './agri_helpers';

describe('orchard demo fleet (acceptance 1-3)', () => {
  it('finishes every task with no robot-tree / robot-robot contact, books exactly the removed fruit, arm faces the booked row', () => {
    const st = demos.find((d) => d.id === 'orchard')!.build();
    const fleet = st.itemsOfType<FleetItem>(ItemType.FLEET)[0];
    const fm = new FleetManager(st, fleet); // what App.fleetManager() does for a loaded station (B4)
    const robots = st.itemsOfType<MobileRobot>(ItemType.MOBILE_ROBOT);
    const plants = plantDisks(st);
    const before = fruitCounts(st);
    expect(fleet.tasks.length).toBe(12); // 8 harvest row sides + 4 spray rows, planned by the demo
    let trunkSteps = 0, canopySteps = 0, overlapSteps = 0, armSamples = 0, armWorst = 0;
    const reach = new Map(robots.map((r) => { const a = armOf(r); return [r.id, a ? armReach(a) : 0]; }));
    // picks: TCP of the picking arm at the fruit when it is removed; arm joint configurations and lift heights
    const mission = st.itemsOfType<MissionItem>(ItemType.MISSION).find((m) => m.missionType === 'harvest')!;
    const dirty = new Set<CropRow>();
    st.events.on('changed', ({ item, what }: { item: unknown; what: string }) => { if (what === 'plants' && item instanceof CropRow) dirty.add(item); });
    const seen = new Set<unknown>();
    const tcpMiss: number[] = [];
    const arms = robots.map((r) => armOf(r)).filter((a): a is NonNullable<typeof a> => !!a);
    const configs = new Map(arms.map((a) => [a.id, new Set<string>()]));
    let liftMax = 0, overSpeed = 0;
    const prevQ = new Map(arms.map((a) => [a.id, a.joints()]));
    const dt = 0.1;
    for (let i = 0; fm.time < 30000; i++) {
      fm.step(dt);
      updateMissionProgress(mission, fm);
      for (const a of arms) {
        const q = a.joints(), p = prevQ.get(a.id)!, vmax = a.jointMaxVelocities();
        if (q.some((v, k) => Math.abs(v - p[k]) / dt > vmax[k] * 1.05)) overSpeed++;
        prevQ.set(a.id, q);
        if (i % 10 === 0) configs.get(a.id)!.add(q.map((v) => v.toFixed(0)).join(','));
        liftMax = Math.max(liftMax, armLift(a).h);
      }
      for (const row of dirty) for (const w of fruitWorldPositions(row, false)) {
        if (!w.fruit.picked || seen.has(w.fruit)) continue;
        seen.add(w.fruit);
        const d = Math.min(...arms.map((a) => { const t = a.poseTCPAbs(); return Math.hypot(t[12] - w.p[0], t[13] - w.p[1], t[14] - w.p[2]); }));
        if (d > 20) tcpMiss.push(Math.round(d));
      }
      dirty.clear();
      for (const r of robots) {
        const c = cropClearance(r, plants);
        if (c.trunk < 0) trunkSteps++;
        if (c.canopy < 0) canopySteps++;
      }
      for (let a = 0; a < robots.length; a++) for (let b = a + 1; b < robots.length; b++) if (footprintsOverlap(robots[a], robots[b])) overlapSteps++;
      if (i % 20 === 0) for (const r of robots) {
        const task = r.state.taskId ? fleet.tasks.find((t) => t.id === r.state.taskId) : null;
        if (!task || task.type !== 'harvest' || task.status !== 'working' || task.meta._rt?.phase !== 'work') continue;
        const row = st.findById(task.meta.rowId) as CropRow;
        const arm = armOf(r)!;
        const p = arm.poseAbs();
        const d = rowAxisDistance(row, p[12], p[13]);
        armWorst = Math.max(armWorst, d / reach.get(r.id)!);
        armSamples++;
      }
      if (fleet.tasks.every((t) => t.status === 'done' || t.type === 'charge')) break;
    }
    const k = fm.kpis();
    const after = fruitCounts(st);
    // 1: every task done, no contact at any step (independent geometry, trunk r = 75 mm, canopy = plant disk)
    expect(fleet.tasks.filter((t) => t.status !== 'done').map((t) => `${t.id}:${t.status}`)).toEqual([]);
    expect(trunkSteps).toBe(0);
    expect(canopySteps).toBe(0);
    expect(overlapSteps).toBe(0);
    expect(k.contacts.robotObstacleSteps).toBe(0);
    expect(k.contacts.robotRobotSteps).toBe(0);
    // 2: the yield KPI is the fruit really removed from the trees (and only ripe fruit are harvested)
    const flipped = after.picked - before.picked;
    expect(flipped).toBeGreaterThan(3000);
    expect(k.fruitPicked).toBe(flipped);
    expect(after.unripePicked - before.unripePicked).toBe(0);
    expect(k.yieldKg).toBeGreaterThan(flipped * 0.1);
    // 3: while harvesting, the arm base stays within 0.8 x the arm's real reach of the row axis it books
    expect(armSamples).toBeGreaterThan(500);
    expect(armWorst).toBeLessThan(0.8);
    // spray mission: liquid applied along the rows
    expect(k.process.spray?.litres).toBeGreaterThan(50);
    // 4 (V3 F2): the arms move — the TCP is on every fruit when it is removed (fruit Ø ~75 mm), the joints never exceed
    // their speed limits, every arm passes through many configurations and rides its lift up the canopy
    expect(seen.size).toBe(flipped);
    expect(tcpMiss).toEqual([]);
    expect(overSpeed).toBe(0);
    for (const a of arms) expect(configs.get(a.id)!.size).toBeGreaterThan(1000);
    expect(liftMax).toBeGreaterThan(1800);
    // 5 (V3 F6): the harvest mission is done only because the crop is: no ripe fruit left on the sides of rows 1-4
    const left = mission.rowIds.reduce((a, id) => a + (st.findById(id) as CropRow).ripeFruitCount(), 0);
    expect(left).toBe(0);
    expect(k.process.harvest?.unreachable ?? 0).toBe(0);
    expect(k.process.harvest?.armUnreachable ?? 0).toBe(0);
    expect(mission.status).toBe('done');
    expect(mission.progress).toBe(1);
    expect(mission.fruit).toMatchObject({ picked: flipped, left: 0 });
  }, 900000);
});
