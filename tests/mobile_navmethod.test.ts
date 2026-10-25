/**
 * The navigation method chosen in a robot's navigation stack (Navigation & SLAM dialog, robot.params.navStack) selects
 * the planner and the controller (was: only the localization estimator and the ROS export):
 *  - Nav2 Smac Hybrid-A* + RPP → curvature-limited Hybrid A* also for differential drives (no rotation in place) and
 *    regulated pure pursuit (speed × r / r_min on curves tighter than r_min = 0.9 m);
 *  - NavFn + DWB / Smac 2D + MPPI → the 2-D footprint grid planner (rotations in place allowed);
 *  - line following / rail → the fixed straight route, no avoidance (blocked = no path);
 *  - no stack configured → unchanged default.
 */
import { describe, it, expect } from 'vitest';
import { Station } from '../src/core/items/item';
import { MobileRobot, MapItem } from '../src/mobile/items';
import { CollisionWorld } from '../src/mobile/footprint';
import { planForRobot } from '../src/mobile/planner';
import { followPath, stepMobile } from '../src/mobile/controller';
import { defaultNavStack, setNavStack } from '../src/mobile/navstack';

const wrap180 = (a: number) => ((((a + 180) % 360) + 360) % 360) - 180;
function setup() {
  const st = new Station('nav');
  const map = st.addChild(new MapItem('map'));
  map.resize(200, 120, 100, -10000, -6000); // 20 m × 12 m
  // a wall in the middle with a gap above it
  for (let y = 0; y < 80; y++) for (let x = 98; x < 102; x++) map.cells[y * map.width + x] = 100;
  const r = st.addChild(new MobileRobot('amr'));
  Object.assign(r.kin, { drive: 'differential', wheelBase: 500, track: 500, wheelRadius: 100, maxSpeed: 1000, maxYawRate: 90, maxAccel: 800, footprint: [800, 600, 600] });
  r.setPose2D(-6000, -3000, 0);
  return { st, r, world: new CollisionWorld({ map }) };
}
const withMethod = (r: MobileRobot, id: string | null) => { if (id === null) { delete r.params.navStack; return; } const c = defaultNavStack('amr', 'warehouse'); c.navigation = id; setNavStack(r, c); };
const inPlaceTurns = (path: number[][]) => path.filter((p, i) => i > 0 && Math.hypot(p[0] - path[i - 1][0], p[1] - path[i - 1][1]) < 1 && Math.abs(wrap180(p[2] - path[i - 1][2])) > 1).length;

describe('navigation method → planner and controller', () => {
  const start = { x: -6000, y: -3000, theta: 0 }, goal = { x: 6000, y: -3000 };

  it('no stack (default) and NavFn + DWB: 2-D grid planner, turning in place allowed', () => {
    for (const m of [null, 'nav2_navfn_dwb'] as const) {
      const { r, world } = setup(); withMethod(r, m);
      const res = planForRobot(r, start, goal, { world, margin: 100 });
      expect(res.ok, String(m)).toBe(true);
      expect(res.method).not.toBe('hybrid');
      expect(inPlaceTurns(res.path)).toBeGreaterThan(0); // turn-then-drive around the wall end
    }
  });

  it('Smac Hybrid-A* + RPP: curvature-limited arcs for the differential robot, no rotation in place', () => {
    const { r, world } = setup(); withMethod(r, 'nav2_smac_hybrid_rpp');
    const res = planForRobot(r, start, goal, { world, margin: 100 });
    expect(res.ok).toBe(true);
    expect(res.method).toBe('hybrid');
    expect(inPlaceTurns(res.path)).toBe(0);
    // heading change per arc length ≤ 1 / max(800, W)
    let kmax = 0;
    for (let i = 1; i < res.path.length; i++) { const ds = Math.hypot(res.path[i][0] - res.path[i - 1][0], res.path[i][1] - res.path[i - 1][1]); if (ds > 50) kmax = Math.max(kmax, Math.abs(wrap180(res.path[i][2] - res.path[i - 1][2])) * Math.PI / 180 / ds); }
    expect(1 / kmax).toBeGreaterThan(800 - 5);
  });

  it('line following: the fixed straight route; blocked by the wall → no path; free → 2 poses', () => {
    const { r, world } = setup(); withMethod(r, 'line_following');
    const blocked = planForRobot(r, start, goal, { world, margin: 100 });
    expect(blocked.ok).toBe(false);
    expect(blocked.error).toMatch(/route blocked/);
    const free = planForRobot(r, { x: -6000, y: 3000, theta: 0 }, { x: 6000, y: 3000 }, { world, margin: 100 });
    expect(free.ok).toBe(true);
    expect(free.path).toHaveLength(2);
    expect(free.method).toBe('line');
  });

  it('regulated pure pursuit slows the robot on curves tighter than 0.9 m (v ≤ vmax · r / 0.9 m); default tracker does not', () => {
    // a quarter circle of radius 600 mm as a pose path (forward)
    const arc = (r: MobileRobot) => { const R = 600, pts: number[][] = [[0, 0, 0, 1], [1500, 0, 0, 1]]; for (let a = 2; a <= 90; a += 2) pts.push([1500 + R * Math.sin(a * Math.PI / 180), R - R * Math.cos(a * Math.PI / 180), a, 1]); pts.push([2100, 2600, 90, 1]); r.setPose2D(0, 0, 0); followPath(r, pts); };
    const maxSpeedOnArc = (m: string | null) => {
      const { r } = setup(); withMethod(r, m); arc(r);
      let vmax = 0;
      for (let i = 0; i < 400; i++) { if (!stepMobile(r, 0.05, {})) break; if (r.state.x > 1700 && r.state.y < 500) vmax = Math.max(vmax, Math.abs(r.state.v)); }
      return vmax;
    };
    const rpp = maxSpeedOnArc('nav2_smac_hybrid_rpp');
    const dflt = maxSpeedOnArc(null);
    expect(rpp).toBeLessThanOrEqual(1000 * (600 / 900) + 30);
    expect(dflt).toBeGreaterThan(rpp + 100);
  });
});
