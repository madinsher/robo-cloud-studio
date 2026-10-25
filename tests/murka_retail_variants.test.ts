/**
 * MURKA retail — the other robot / world variants: robot-retailer-murka (v1-classic: STL robot, 4-DOF arms, no wrist,
 * tip grasp, head turned 90° for the scan) runs the whole `full` mission; the renamed copies of
 * simulation_ws/src/MURKA_Simulation (Murka / MurkaClassic, gondola levels 0.9/1.2/1.5, robot starting at home, charging
 * pad as a plain 5 cm Solid) load and run the scan + dock stages.
 */
import { describe, it, expect } from 'vitest';
import { buildMurkaRetailStation, runMurkaRetailMission } from '../src/murka/retail';
import { AssetStore } from '../src/scene/assets';
import fs from 'node:fs';
import path from 'node:path';
import { HAS_MURKA, MURKA } from './murka_retail_helpers';

describe.skipIf(!HAS_MURKA)('MURKA retail variants', () => {
  it('v1-classic: full mission done — 24/24, box_blue to bulk_B3, tea + juice packed, docked; time ≈ Webots / MuJoCo v1', async () => {
    const b = await buildMurkaRetailStation({ variant: 'v1', assets: new AssetStore(), loadMeshes: false });
    expect(b.robot.hasWrist).toBe(false);
    expect([b.robot.left.dof, b.robot.right.dof, b.robot.head.dof]).toEqual([4, 4, 2]);
    const run = runMurkaRetailMission(b);
    const r = await run.done;
    expect(r.outcome, r.report.join('\n')).toBe('done');
    expect(r.checks.scan).toEqual({ visible: 24, correct: 24, wrong: [], webotsVisible: 24 });
    expect(r.checks.depal!.moved).toEqual(['box_blue_7 -> bulk_B3_L0_S0']);
    expect(r.checks.order!.ok).toBe(true);
    expect(r.checks.docked!.ok).toBe(true);
    expect(r.checks.collisions.armLinks).toBe(0);
    expect(r.checks.collisions.base).toBe(0);
    // classic tip grasp: the product hangs GRIP_CLEARANCE under the tip and the place target is the surface itself — on
    // the packing table the joint-linear approach lowers it a few mm into the top (as in the original classic scheme)
    // (the tip IK is accepted up to IK_TOL = 2 cm, so the dip depends on the parking error of each run)
    for (const c of r.collisions) { expect(c.part).toMatch(/^load /); expect(c.item).toBe('packing_table'); expect(c.depth).toBeLessThan(0.05); }
    // v1 reference: Webots 706 s wall at RTF 0.73 (≈ 515 s simulated), MuJoCo 499 s at RTF 1.0
    expect(r.reference.webotsSim).toBeCloseTo(706 * 0.73, 6);
    expect(r.simTime / 499).toBeGreaterThan(0.8); expect(r.simTime / 499).toBeLessThan(1.2);
    // the classic robot scans with the head turned 90° (no compact view): robot yaw 0 at the views
    const views = run.session.orch.navLog.filter((n) => n.state === 'NAV_VIEW');
    expect(views).toHaveLength(6);
    for (const v of views) expect(Math.abs(v.outcome.finalError![2])).toBeLessThan(0.031);
    expect(JSON.parse(views[0].goal)[2]).toBe(0);
  }, 900000);

  for (const variant of ['murka', 'murka_classic'] as const) {
    it(`${variant} (MURKA_Simulation copy): scan + dock stages done, every visible slot correct, docked at the dock target`, async () => {
      const b = await buildMurkaRetailStation({ variant, assets: new AssetStore(), loadMeshes: false });
      expect(b.robot.base.name).toMatch(/^murka/i);
      expect(b.robot.hasWrist).toBe(variant === 'murka');
      expect(b.layout.dock).toBeNull(); // the renamed worlds have a dock target, no charger slot
      const [x0, y0] = [b.robot.base.state.x, b.robot.base.state.y];
      expect([x0, y0]).toEqual([-5500, 3000]);
      const r = await runMurkaRetailMission(b, { params: { depal: '', order: '' } }).done;
      expect(r.outcome, r.report.join('\n')).toBe('done');
      expect(r.stages.map((s) => [s.name, s.outcome])).toEqual([['SCAN', 'done'], ['DOCK', 'done']]);
      expect(r.checks.scan!.wrong).toEqual([]);
      expect(r.checks.scan!.visible).toBe(36); // levels 0.9 / 1.2 / 1.5: the top level is in frame too
      expect(r.checks.scan!.correct).toBe(36);
      expect(r.checks.docked!.distance).toBeLessThan(0.03);
      expect(r.checks.collisions.ok).toBe(true);
    }, 900000);
  }

  it('browser path: a picked MURKA_Simulation folder (files map with v1 and v2 projects) builds the requested variant', async () => {
    const files = new Map<string, Uint8Array>();
    for (const proj of ['robot-retailer-murka-v2', 'robot-retailer-murka']) {
      const src = path.join(MURKA, proj, 'ros2_ws/src');
      const add = (rel: string) => files.set(`MURKA_Simulation/${proj}/ros2_ws/src/${rel}`, new Uint8Array(fs.readFileSync(path.join(src, rel))));
      for (const d of ['murom_webots/worlds', 'murom_webots/protos']) for (const f of fs.readdirSync(path.join(src, d))) if (/\.(wbt|proto)$/.test(f)) add(`${d}/${f}`);
      add('murom_world/config/products.yaml'); add('murom_world/config/locations.yaml');
    }
    const v2 = await buildMurkaRetailStation({ variant: 'v2', files, loadMeshes: false });
    const v1 = await buildMurkaRetailStation({ variant: 'v1', files, loadMeshes: false });
    expect(v2.robot.hasWrist).toBe(true); expect(v1.robot.hasWrist).toBe(false);
    expect(v2.layout.instances).toHaveLength(50);
    expect(v2.station.find('tea_box_1')).not.toBeNull();
  }, 300000);
});
