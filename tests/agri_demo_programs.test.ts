/**
 * The orchard and greenhouse demos carry a program to run with ▶ (not only fleet missions): the picking platform
 * drives into a row, its arm picks the fruit within reach (a fruit leaves the plant when the gripper closes on it)
 * and the platform drives home.
 */
import { describe, it, expect } from 'vitest';
import { demos } from '../src/demos';
import { Program } from '../src/core/items/program';
import { ItemType } from '../src/core/items/item';
import { MobileRobot } from '../src/mobile/items';
import { CropRow } from '../src/agri/items';
import { StationRuntime } from '../src/core/motion/runtime';
import { bindHarvestEvents } from '../src/agri/missions';

const picked = (rows: CropRow[]) => rows.reduce((n, r) => n + r.plants.reduce((m, p) => m + p.fruit.filter((f) => f.picked).length, 0), 0);

describe('agri demo programs', () => {
  for (const id of ['orchard', 'greenhouse']) {
    it(`${id}: the harvest-stop program drives in, picks fruit with the arm and drives home`, { timeout: 120000 }, () => {
      const st = demos.find((d) => d.id === id)!.build();
      const progs = st.itemsOfType<Program>(ItemType.PROGRAM);
      const prog = progs.find((p) => p.name.startsWith('Harvest stop'));
      expect(prog, `${id} programs: ${progs.map((p) => p.name)}`).toBeTruthy();
      const mob = prog!.robot() as MobileRobot;
      const call = prog!.instructions().find((i) => i.data.kind === 'call')!.data as { programId: string };
      const pick = st.findById(call.programId) as Program;
      const planned = pick.instructions().filter((i) => i.data.kind === 'event' && (i.data as any).action === 'gripper_close').length;
      expect(planned).toBeGreaterThan(0);
      const rows = st.itemsOfType<CropRow>(ItemType.CROP_ROW);
      const home = [mob.state.x, mob.state.y];
      const rt = StationRuntime.of(st);
      const h = bindHarvestEvents(rt as any, st);
      const run = rt.start(prog!);
      let farthest = 0;
      for (let k = 0; k < 900 && run.status !== 'done' && run.status !== 'error'; k++) {
        rt.runFor(1, 0.05);
        farthest = Math.max(farthest, Math.hypot(mob.state.x - home[0], mob.state.y - home[1]));
      }
      expect(run.status, JSON.stringify(run.error)).toBe('done');
      expect(farthest).toBeGreaterThan(2000);                                  // it really drove into the row
      expect(Math.hypot(mob.state.x - home[0], mob.state.y - home[1])).toBeLessThan(300);   // and came back
      expect(h.picked).toBe(planned);                                         // every planned fruit was cut
      expect(picked(rows)).toBe(planned);
    });
  }
});
