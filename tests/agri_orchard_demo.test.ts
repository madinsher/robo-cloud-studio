// Acceptance (agri_mobile §5, items 1–3): the orchard demo as the app loads it — a new FleetManager on the station
// built by the demo — runs its harvest and spray missions to the end without any robot touching a tree or another
// robot, books exactly the fruit it removed, and the picking arm always faces the row it harvests.
import { describe, it, expect } from 'vitest';
import { demos } from '../src/demos';
import { FleetItem, FleetManager } from '../src/fleet/fleet';
import { ItemType } from '../src/core/items/item';
import { MobileRobot } from '../src/mobile/items';
import { CropRow } from '../src/agri/items';
import { armOf, armReach } from '../src/agri/harvest';
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
    const dt = 0.1;
    for (let i = 0; fm.time < 15000; i++) {
      fm.step(dt);
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
  }, 600000);
});
