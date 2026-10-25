// Acceptance (agri_mobile §5, item 14): group control "Run on fleet" drives Ackermann robots through their own drive
// model — observed yaw rate within the robot's limit, no sideways motion of the rear axle, steering within its limit.
import { describe, it, expect } from 'vitest';
import { Station } from '../src/core/items/item';
import { MobileRobot } from '../src/mobile/items';
import { FleetRuntime } from '../src/mrs/runtime';
import { rearAxleOffset } from '../src/mobile/controller';

describe('group control on Ackermann robots (acceptance 14)', () => {
  it('a consensus formation run on the fleet respects yaw-rate, steering and no-slip constraints', () => {
    const st = new Station('group');
    const robots: MobileRobot[] = [];
    for (let i = 0; i < 6; i++) {
      const r = st.addChild(new MobileRobot(`Car ${i + 1}`));
      Object.assign(r.kin, { drive: 'ackermann', wheelBase: 600, maxSteer: 30, minTurnRadius: 1100, maxSteerRate: 60, maxSpeed: 800, maxReverseSpeed: 400, maxYawRate: 40, footprint: [900, 500, 400] });
      r.setPose2D(i * 2000, (i % 2) * 1500, i * 37);
      robots.push(r);
    }
    const src = `consensus Formation of six robots
robots 6
graph ring n=6
formation circle r=3 gain=0.5 steps=1200 dt=0.05`;
    const rt = new FleetRuntime({ kind: 'consensus', source: src, robots, station: st, vmax: 0.6 });
    let maxYaw = 0, maxLat = 0, maxSteer = 0;
    const prev = robots.map((r) => ({ x: r.state.x, y: r.state.y, th: r.state.theta }));
    const dt = 0.05;
    for (let k = 0; k < 1200 && !rt.done; k++) {
      rt.tick(dt);
      robots.forEach((r, i) => {
        const p = prev[i];
        const dth = ((r.state.theta - p.th + 540) % 360) - 180;
        maxYaw = Math.max(maxYaw, Math.abs(dth) / dt);
        const thm = ((p.th + dth / 2) * Math.PI) / 180;
        // the rear axle (body x = −wheelBase/2, the bicycle model's reference point) of each car
        const ax = rearAxleOffset(r.kin), c0 = Math.cos(p.th * Math.PI / 180), s0 = Math.sin(p.th * Math.PI / 180), c1 = Math.cos(r.state.theta * Math.PI / 180), s1 = Math.sin(r.state.theta * Math.PI / 180);
        const vx = (r.state.x + c1 * ax - p.x - c0 * ax) / dt, vy = (r.state.y + s1 * ax - p.y - s0 * ax) / dt;
        maxLat = Math.max(maxLat, Math.abs(-Math.sin(thm) * vx + Math.cos(thm) * vy));
        maxSteer = Math.max(maxSteer, Math.abs(r.state.steer));
        prev[i] = { x: r.state.x, y: r.state.y, th: r.state.theta };
      });
    }
    const travelled = robots.reduce((a, r) => a + r.state.odometer, 0);
    expect(travelled).toBeGreaterThan(1000); // the group law really moved the robots
    expect(maxYaw).toBeLessThanOrEqual(40 + 1e-6);
    expect(maxSteer).toBeLessThanOrEqual(30 + 1e-6);
    expect(maxLat).toBeLessThan(5); // mm/s: the rear axle never slides sideways (no teleporting)
  });
});
