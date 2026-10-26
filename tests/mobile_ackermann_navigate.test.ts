/**
 * F1 (V3 finding): Navigate instructions with a goal heading on an Ackermann base. The tracker stopped 80 mm before
 * the end of the Reeds–Shepp approach arc (3° of heading left on a 1.5 m radius) and the 3° acceptance failed the leg;
 * the failed motion was then dropped, so the base did not move at all (1 of 4 legs driven, the next leg started from
 * the wrong place and timed out in a walled map). Now the last leg is driven to its end, a car-like base is accepted
 * within 5° (Nav2's default yaw tolerance is 14°), and a drive that does end off the goal still plays before the
 * instruction fails (like an arm motion that plays up to its failing point).
 */
import { describe, it, expect } from 'vitest';
import { Station } from '../src/core/items/item';
import { Program } from '../src/core/items/program';
import { MobileRobot, MapItem } from '../src/mobile/items';
import { ProgramSimulator } from '../src/core/motion/simulator';
import { StationRuntime } from '../src/core/motion/runtime';

const GOALS: Array<[number, number, number]> = [[13000, 15000, 90], [21000, 4000, 0], [25600, 16500, 90], [3000, 3000, 180]];
function cell(walls: boolean) {
  const st = new Station('nav');
  const map = st.addChild(new MapItem('map'));
  map.resize(300, 200, 100, 0, 0);
  if (walls) { map.fillRect(9000, 0, 9300, 12000); map.fillRect(17000, 8000, 17300, 20000); map.fillRect(24000, 14000, 24200, 19000); map.fillRect(27000, 14000, 27200, 19000); map.fillRect(24000, 18800, 27200, 19000); }
  const r = st.addChild(new MobileRobot('Base'));
  Object.assign(r.kin, { drive: 'ackermann', footprint: [1400, 900, 800], maxSpeed: 1200, maxReverseSpeed: 600, maxAccel: 600, wheelBase: 1000, maxSteer: 35, minTurnRadius: 1500, maxSteerRate: 60 });
  r.setPose2D(3000, 3000, 0);
  const p = st.addChild(new Program('Drive'));
  p.setRobot(r);
  return { st, r, p };
}

describe('Ackermann Navigate legs with a goal heading', () => {
  for (const walls of [false, true]) {
    it(`${walls ? 'walled' : 'empty'} 30 x 20 m map: all 4 legs reach their goal pose and the program has no problem`, () => {
      const { st, r, p } = cell(walls);
      for (const [x, y, h] of GOALS) p.navigateTo(x, y, h, { speed: 1000 });
      const sim = new ProgramSimulator(st);
      const res = sim.compile(p);
      expect(res.problems.map((q) => q.message)).toEqual([]);
      // every leg ends at its goal: sample the end of each recorded step
      const steps = sim.steps.filter((s) => s.mobile);
      expect(steps.length).toBe(4);
      steps.forEach((s, k) => {
        const e = s.mobile!.sample(s.mobile!.duration);
        const [x, y, h] = GOALS[k];
        expect(Math.hypot(e.x - x, e.y - y), `leg ${k + 1} position`).toBeLessThan(100);
        expect(Math.abs(((e.heading - h + 540) % 360) - 180), `leg ${k + 1} heading`).toBeLessThan(5);
      });
      sim.seek(res.duration);
      expect(Math.hypot(r.state.x - 3000, r.state.y - 3000)).toBeLessThan(100);
    });
  }

  it('a drive that ends off its goal is played (the base moves) before the instruction fails', () => {
    const { st, r, p } = cell(false);
    p.navigateTo(13000, 15000, 90, { speed: 1000, tolerance: 0.5 } as any);   // unreachable precision
    const rt = new StationRuntime(st);
    const h: any = rt.start(p, { robot: r as any });
    for (let t = 0; t < 600 && h.status === 'running'; t += 0.1) rt.tick(0.1);
    expect(h.status).toBe('error');
    expect(h.error.message).toMatch(/from the goal/);
    expect(Math.hypot(r.state.x - 13000, r.state.y - 15000)).toBeLessThan(200);   // it drove there
    expect(h.endTime - h.startTime).toBeGreaterThan(10);
  });
});
