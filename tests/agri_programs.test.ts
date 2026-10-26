// Acceptance (agri_mobile §5, items 7–9): mobile spray program runs in the program timeline; the harvest pick program
// is planned for the arm at its current base pose, collision-free against carrier and trees, removes fruit only when
// the gripper closes during simulation, re-solves at the next base stop; a detect -> pick loop closes on detections.
import { describe, it, expect } from 'vitest';
import { demos } from '../src/demos';
import { ItemType, Camera } from '../src/core/items/item';
import { MobileRobot } from '../src/mobile/items';
import { FieldItem, CropRow } from '../src/agri/items';
import { generateSprayProgram, generateHarvestProgram, bindHarvestEvents, matchDetections } from '../src/agri/missions';
import { armOf, fruitByRef } from '../src/agri/harvest';
import { detectFruit } from '../src/agri/vision';
import { ProgramSimulator } from '../src/core/motion/simulator';
import { mul, transl, rotx, DEG } from '../src/core/math/pose';
import { fruitCounts } from './agri_helpers';

function orchard() {
  const st = demos.find((d) => d.id === 'orchard')!.build();
  const field = st.itemsOfType<FieldItem>(ItemType.FIELD)[0];
  return { st, field };
}

describe('mobile spray program (acceptance 7)', () => {
  it('Navigate legs along two rows take longer than row length / speed and end at the last row exit', () => {
    const { st, field } = orchard();
    const sprayer = st.find('Sprayer', ItemType.MOBILE_ROBOT) as MobileRobot;
    const rows = field.rows().slice(4, 6);
    const prog = generateSprayProgram(st, sprayer, rows, field.crop.rowSpacing / 2, 800);
    const sim = new ProgramSimulator(st);
    const res = sim.compile(prog);
    expect(res.problems.filter((p) => p.severity === 'error')).toEqual([]);
    const len = rows[0].length() + rows[1].length();
    expect(res.duration).toBeGreaterThan(len / 800);
    sim.runToEnd();
    // the second row is driven in the opposite direction: exit at its start
    const exit = rows[1].travelLine(-field.crop.rowSpacing / 2).start;
    expect(Math.hypot(sprayer.state.x - exit[0], sprayer.state.y - exit[1])).toBeLessThan(150);
    expect(Math.abs(((sprayer.state.theta - 180 + 540) % 360) - 180)).toBeLessThan(3);
  });
});

describe('harvest pick program (acceptance 8)', () => {
  it('leaves fruit on the tree until the program runs, has no collision with carrier / trees, and re-solves at the next stop', () => {
    const { st } = orchard();
    const carrier = st.find('Harvest platform 1', ItemType.MOBILE_ROBOT) as MobileRobot;
    const arm = armOf(carrier)!;
    carrier.setPose2D(20000, 12250, 0); // in alley 1, arm (mounted right) facing the row at y = 10500
    const rows = st.itemsOfType<CropRow>(ItemType.CROP_ROW).filter((r) => Math.abs(r.start[1] - 10500) < 10);
    expect(rows.length).toBe(1);
    const before = fruitCounts(st);
    const hp = generateHarvestProgram(st, arm, rows, { limit: 12 });
    expect(hp.source).toBe('ground_truth');
    expect(hp.picked).toBeGreaterThan(4);
    expect(fruitCounts(st).picked).toBe(before.picked); // nothing removed at generation
    const sim = new ProgramSimulator(st);
    sim.collisionOptions = { enabled: true, sampleStep: 0.05 };
    const res = sim.compile(hp.program);
    expect(res.problems.filter((p) => p.severity === 'error')).toEqual([]);
    expect(sim.collisions).toEqual([]);
    // the carrier and the trunks were part of the check (not an empty scene)
    expect(st.find(`${arm.name} crop colliders`, ItemType.OBJECT)).toBeTruthy();
    const h = bindHarvestEvents(sim, st);
    sim.runToEnd();
    expect(h.picked).toBe(hp.picked);
    expect(h.missed).toBe(0);
    expect(fruitCounts(st).picked - before.picked).toBe(hp.picked);
    for (const ref of hp.targets) expect(fruitByRef(st, ref)!.fruit.picked).toBe(true);
    h.off();
    // next stop 2 m further: the old program no longer matches the fruit, regenerating re-solves at the new base pose
    carrier.setPose2D(22000, 12250, 0);
    const hp2 = generateHarvestProgram(st, arm, rows, { limit: 12 });
    expect(hp2.program).toBe(hp.program); // same program item, re-solved
    expect(hp2.picked).toBeGreaterThan(4);
    expect(hp2.targets.filter((t) => hp.targets.includes(t))).toEqual([]); // picked fruit are not planned again
    const sim2 = new ProgramSimulator(st);
    sim2.collisionOptions = { enabled: true, sampleStep: 0.05 };
    const res2 = sim2.compile(hp2.program);
    expect(res2.problems.filter((p) => p.severity === 'error')).toEqual([]);
    expect(sim2.collisions).toEqual([]);
  }, 120000);
});

describe('detect -> pick loop (acceptance 9)', () => {
  it('picks planned from camera detections remove exactly the detected fruit; they are no longer detected afterwards', () => {
    const { st } = orchard();
    const carrier = st.find('Harvest platform 1', ItemType.MOBILE_ROBOT) as MobileRobot;
    const arm = armOf(carrier)!;
    carrier.setPose2D(30000, 12250, 0);
    // a camera on the front corner of the carrier looking at the row on the arm side (-y), optical axis horizontal
    const cam = carrier.addChild(new Camera('Row camera'));
    cam.setPose(mul(transl(1000, -850, 1700), rotx(90 * DEG)));
    cam.far = 3000;
    const ripe = () => detectFruit(st, cam, { onlyRipe: true });
    const det0 = ripe();
    expect(det0.length).toBeGreaterThan(5);
    const matched = matchDetections(st, det0.map((d) => ({ p: d.p })));
    expect(matched.every((m) => m.ref !== 'fruit:none')).toBe(true); // simulated detections are exact
    const rows = st.itemsOfType<CropRow>(ItemType.CROP_ROW);
    // the camera looks ahead of the arm: advance the base by the camera offset, then pick from the (world) detections
    carrier.setPose2D(31000, 12250, 0);
    const hp = generateHarvestProgram(st, arm, rows, { detections: matched, limit: 8 });
    expect(hp.source).toBe('vision');
    expect(hp.picked).toBeGreaterThan(0);
    const sim = new ProgramSimulator(st);
    const h = bindHarvestEvents(sim, st);
    sim.compile(hp.program);
    sim.runToEnd();
    expect(h.picked).toBe(hp.picked);
    carrier.setPose2D(30000, 12250, 0); // back to the detection pose
    const det1 = ripe();
    const key = (p: number[]) => p.map((v) => v.toFixed(1)).join(',');
    const before = new Set(det0.map((d) => key(d.p)));
    const after = new Set(det1.map((d) => key(d.p)));
    const pickedKeys = hp.targets.map((ref) => matched.find((m) => m.ref === ref)!.p).map(key);
    for (const k of pickedKeys) { expect(before.has(k)).toBe(true); expect(after.has(k)).toBe(false); }
    // detections after = detections before - picked + fruit that were hidden behind the picked ones
    const revealed = det1.filter((d) => !before.has(key(d.p))).length;
    expect(det1.length).toBe(det0.length - hp.picked + revealed);
  }, 120000);
});
