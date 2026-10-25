/**
 * Fleet harvest with the picking arm (src/agri/process.ts): the arm moves to every fruit it picks — TCP on the fruit at
 * the pick time, joint speeds within the arm's limits, pick configurations free of collisions with the arm itself, its
 * carrier and mast and the rigid crop (independent check with the simulator's checker) — and the mission result is the
 * crop, not the task list: a harvest that leaves ripe fruit on the plants ends `incomplete`, progress = picked /
 * (picked + left), with the reason; with the arm on a lift the whole canopy is reached and the mission ends `done`.
 */
import { describe, it, expect } from 'vitest';
import { Station, Tool } from '../src/core/items/item';
import { createRobotFromLibrary } from '../src/core/items/library';
import { Robot } from '../src/core/items/robot';
import { MobileRobot } from '../src/mobile/items';
import { FieldItem, MissionItem, cropParams, CropRow } from '../src/agri/items';
import { generateOrchard, buildFieldMap, rectPolygon, fruitWorldPositions } from '../src/agri/orchard';
import { planMission, updateMissionProgress } from '../src/agri/missions';
import { armLift, setArmLift, mountArmOnLift, addLiftColumn } from '../src/agri/harvest';
import { cropColliders } from '../src/agri/collision_world';
import { robotCollisionChecker } from '../src/core/collision/collision';
import { FleetItem, FleetManager } from '../src/fleet/fleet';
import { mul, transl, rotz, DEG } from '../src/core/math/pose';

/** One apple row (spindles 0.6–4.3 m), one tracked platform with a UR10e at the row-side edge (optionally on a lift). */
function block(lift: number | null, settings: Record<string, unknown> = {}) {
  const st = new Station('harvest arm');
  const field = st.addChild(new FieldItem('Block'));
  field.polygon = rectPolygon(24000, 8000);
  field.crop = cropParams('apple', { rowHeading: 0, headland: 4000, ripeFraction: 0.5 });
  generateOrchard(field, 3);
  st.addChild(buildFieldMap(field, 250));
  const fleet = st.addChild(new FleetItem('Fleet'));
  const fm = new FleetManager(st, fleet);
  const m = st.addChild(new MobileRobot('Picker'));
  Object.assign(m.kin, { drive: 'tracked', wheelBase: 1800, track: 1400, maxSpeed: 1300, footprint: [2600, 1600, 1400] });
  m.capabilities = ['harvest'];
  m.setPose2D(-4000, 2000, 0);
  m.home = { x: -4000, y: 2000, theta: 0 };
  fm.addRobot(m);
  const arm = m.addChild(createRobotFromLibrary('UR10e', 'Arm'));
  arm.setPose(mul(transl(0, -900, 1400), rotz(-90 * DEG)));
  if (lift) {
    mountArmOnLift(arm, lift);
    addLiftColumn(arm, 4, 85);
  }
  const g = arm.addChild(new Tool('Gripper')); g.toolKind = 'gripper'; g.setPoseTool(transl(0, 0, 180)); arm.setTool(g);
  arm.setJoints([0, -60, 80, -110, -90, 0]);
  const row = field.rows()[0];
  const mission = st.addChild(new MissionItem('Harvest row 1'));
  mission.missionType = 'harvest'; mission.fieldId = field.id; mission.fleetId = fleet.id; mission.rowIds = [row.id];
  mission.settings = { workSpeed: 400, bothSides: false, sideOffset: field.crop.rowSpacing / 2, secondsPerFruit: 4, ...settings };
  const plan = planMission(st, mission, fm);
  return { st, fm, fleet, robot: m, arm, row, mission, plan };
}

/** Run to the end; at every pick the TCP distance to the fruit, joint speeds, distinct configurations, and an
 *  independent collision check of every `checkEvery`-th pick configuration. */
function run(b: ReturnType<typeof block>, checkEvery = 0) {
  const { st, fm, fleet, arm, mission } = b;
  const dirty = new Set<CropRow>();
  const off = st.events.on('changed', ({ item, what }: { item: unknown; what: string }) => { if (what === 'plants' && item instanceof CropRow) dirty.add(item); });
  const seen = new Set<unknown>();
  const dists: number[] = [];
  const configs = new Set<string>();
  let prev = arm.joints(), speedMax = 0, speedSteps = 0, lifts = new Set<number>(), collisions = 0, checked = 0;
  const vmax = arm.jointMaxVelocities();
  const dt = 0.1;
  while (fm.time < 8000) {
    fm.step(dt);
    updateMissionProgress(mission, fm);
    const q = arm.joints();
    const ratio = Math.max(...q.map((v, i) => Math.abs(v - prev[i]) / dt / vmax[i]));
    speedMax = Math.max(speedMax, ratio); if (ratio > 1.05) speedSteps++;
    prev = q;
    configs.add(q.map((v) => v.toFixed(0)).join(','));
    lifts.add(Math.round(armLift(arm).h / 100));
    for (const row of dirty) for (const w of fruitWorldPositions(row, false)) {
      if (!w.fruit.picked || seen.has(w.fruit)) continue;
      seen.add(w.fruit);
      const tcp = arm.poseTCPAbs();
      dists.push(Math.hypot(tcp[12] - w.p[0], tcp[13] - w.p[1], tcp[14] - w.p[2]));
      if (checkEvery && dists.length % checkEvery === 0) {
        // independent check at the pick: arm vs itself, carrier, mast, station, trunks (contacts of the travel pose ignored)
        checked++;
        const crop = cropColliders(st, [b.robot.state.x, b.robot.state.y], 3500, 'test crop');
        const c = robotCollisionChecker(st, arm as Robot, {});
        const key = (p: { a: { item: { id: string }; part: string }; b: { item: { id: string }; part: string } }) => `${p.a.item.id}/${p.a.part}|${p.b.item.id}/${p.b.part}`;
        const atPick = c.check(1).map(key);
        const keep = { q: arm.joints(), h: armLift(arm).h };
        arm.setJoints([0, -60, 80, -110, -90, 0]); setArmLift(arm, 0);
        const base = new Set(c.check(2).map(key));
        arm.setJoints(keep.q); setArmLift(arm, keep.h);
        if (atPick.some((k) => !base.has(k))) collisions++;
        crop?.delete();
      }
    }
    dirty.clear();
    if (fleet.tasks.every((t) => t.status === 'done' || t.status === 'failed')) break;
  }
  off();
  return { dists, configs: configs.size, speedMax, speedSteps, lifts: lifts.size, collisions, checked, k: fm.kpis() };
}

describe('fleet harvest: the arm moves to every fruit it picks; the mission reports the crop left on the plants', () => {
  it('fixed mount: TCP on every picked fruit, joints within their speed limits, pick configurations collision-free; out-of-reach fruit stay and the mission ends incomplete', () => {
    const b = block(null);
    const ripe0 = fruitWorldPositions(b.row, true).filter((f) => f.side === b.plan.tasks[0].meta.side).length;
    expect(b.plan.unreachableFruit).toBeGreaterThan(0); // planning warns before the run
    expect(b.mission.warnings.some((w) => /out of the arm's reach/.test(w))).toBe(true);
    const r = run(b, 10);
    expect(b.fleet.tasks[0].status).toBe('done');
    expect(r.dists.length).toBe(r.k.fruitPicked);
    expect(r.dists.length).toBeGreaterThan(100);
    // the arm really moves: at every pick the TCP is on the fruit (fruit Ø 75 mm)
    expect(r.dists.filter((d) => d > 20)).toEqual([]);
    expect(r.configs).toBeGreaterThan(500);
    expect(r.speedSteps).toBe(0);
    expect(r.checked).toBeGreaterThan(10);
    expect(r.collisions).toBe(0);
    // the crop decides the result: fruit above the fixed arm's reach stay on the tree, the mission says so
    const left = fruitWorldPositions(b.row, true).filter((f) => f.side === b.plan.tasks[0].meta.side).length;
    expect(left).toBeGreaterThan(0);
    expect(r.k.fruitPicked + left).toBe(ripe0);
    expect(b.mission.status).toBe('incomplete');
    expect(b.mission.progress).toBeCloseTo(r.k.fruitPicked / ripe0, 6);
    expect(b.mission.fruit).toMatchObject({ picked: r.k.fruitPicked, left });
    expect(b.mission.warnings.some((w) => w.startsWith(`Result: ${left} of ${ripe0} ripe fruit`) && /out of the arms' reach/.test(w))).toBe(true);
  }, 300000);

  it('on a 2.1 m lift the same arm reaches the whole canopy: every ripe fruit of the row side picked, mission done (progress 1)', () => {
    const b = block(2100);
    expect(b.plan.unreachableFruit).toBe(0);
    const ripe0 = fruitWorldPositions(b.row, true).filter((f) => f.side === b.plan.tasks[0].meta.side).length;
    const r = run(b, 10);
    expect(r.k.fruitPicked).toBe(ripe0);
    expect(r.dists.filter((d) => d > 20)).toEqual([]);
    expect(r.lifts).toBeGreaterThan(15); // the lift travels over its stroke
    expect(r.speedSteps).toBe(0);
    expect(r.collisions).toBe(0);
    expect(fruitWorldPositions(b.row, true).filter((f) => f.side === b.plan.tasks[0].meta.side)).toHaveLength(0);
    expect(b.mission.status).toBe('done');
    expect(b.mission.progress).toBe(1);
    expect(b.mission.warnings.filter((w) => w.startsWith('Result: '))).toEqual([]);
  }, 300000);

  it('animateArm: false keeps the timing-only model: the joints never move, the same fruit are picked', () => {
    const a = run(block(2100));
    const b = block(2100, { animateArm: false });
    const r = run(b);
    expect(r.configs).toBe(1);
    expect(r.k.fruitPicked).toBe(a.k.fruitPicked);
  }, 300000);
});
