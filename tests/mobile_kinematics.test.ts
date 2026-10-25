// T8 agri/mobile: drive models and path tracking against kinematic invariants (acceptance §5 items 4 and 5).
import { describe, it, expect } from 'vitest';
import { Station } from '../src/core/items/item';
import { MobileRobot, MapItem } from '../src/mobile/items';
import { followPath, stepMobile, integrate, minTurnRadius } from '../src/mobile/controller';
import { reedsSheppPath, reedsSheppCandidates, dubinsCandidates, integrateWord } from '../src/mobile/reeds_shepp';

function mk(drive: MobileRobot['kin']['drive'], extra: Partial<MobileRobot['kin']> = {}) {
  const st = new Station('K');
  const r = st.addChild(new MobileRobot(drive));
  Object.assign(r.kin, { drive, ...extra });
  r.setPose2D(0, 0, 0);
  return { st, r };
}

/** Run until the path is done; record body-frame lateral velocity, yaw rate, steering and the trajectory. */
function run(r: MobileRobot, maxT = 600, dt = 0.05) {
  let t = 0, maxLat = 0, maxYaw = 0, maxSteer = 0;
  let px = r.state.x, py = r.state.y, pth = r.state.theta;
  const trail: number[][] = [];
  while (t < maxT && stepMobile(r, dt)) {
    t += dt;
    const vx = (r.state.x - px) / dt, vy = (r.state.y - py) / dt;
    const th = ((pth + r.state.theta) / 2) * Math.PI / 180;
    // lateral velocity of the reference point (mid-step heading)
    const dth = ((r.state.theta - pth + 540) % 360) - 180;
    if (Math.abs(dth) < 90) maxLat = Math.max(maxLat, Math.abs(-Math.sin(th) * vx + Math.cos(th) * vy));
    maxYaw = Math.max(maxYaw, Math.abs(dth) / dt);
    maxSteer = Math.max(maxSteer, Math.abs(r.state.steer));
    px = r.state.x; py = r.state.y; pth = r.state.theta;
    trail.push([r.state.x, r.state.y, r.state.theta]);
  }
  return { t, maxLat, maxYaw, maxSteer, trail, done: r.state.path === null };
}

describe('mobile: Reeds–Shepp / Dubins words', () => {
  it('every returned word ends exactly at the goal and RS is never longer than Dubins', () => {
    let rng = 7;
    const rand = () => { rng = (rng * 16807) % 2147483647; return rng / 2147483647; };
    for (let i = 0; i < 300; i++) {
      const g = { x: (rand() - 0.5) * 20000, y: (rand() - 0.5) * 20000, theta: (rand() - 0.5) * 360 };
      const rs = reedsSheppPath({ x: 0, y: 0, theta: 0 }, g, 3000, { reverseCost: 1, switchCost: 0 });
      const du = reedsSheppPath({ x: 0, y: 0, theta: 0 }, g, 3000, { allowReverse: false });
      expect(rs).not.toBeNull();
      expect(du).not.toBeNull();
      const end = rs!.poses[rs!.poses.length - 1];
      expect(Math.hypot(end[0] - g.x, end[1] - g.y)).toBeLessThan(1);
      expect(Math.abs(((end[2] - g.theta + 540) % 360) - 180)).toBeLessThan(0.01);
      expect(du!.word.lengths.every((l) => l >= -1e-9)).toBe(true);
      expect(rs!.length).toBeLessThanOrEqual(du!.length + 1e-6);
      // consecutive samples are at most one step apart (no jumps)
      for (let k = 1; k < rs!.poses.length; k++) expect(Math.hypot(rs!.poses[k][0] - rs!.poses[k - 1][0], rs!.poses[k][1] - rs!.poses[k - 1][1])).toBeLessThan(101);
    }
  });
  it('straight back is a single reverse segment; a parallel park needs a direction switch', () => {
    const back = reedsSheppPath({ x: 0, y: 0, theta: 0 }, { x: -5000, y: 0, theta: 0 }, 3000)!;
    expect(back.length).toBeCloseTo(5000, 3);
    expect(back.poses.slice(1).every((p) => p[3] === -1)).toBe(true);
    const park = reedsSheppPath({ x: 0, y: 0, theta: 0 }, { x: 0, y: 1000, theta: 0 }, 3000)!;
    const dirs = new Set(park.poses.slice(1).map((p) => p[3]));
    expect(dirs.has(1) && dirs.has(-1)).toBe(true);
    // the classic formulas produce all families; spot-check that candidates verify for a generic goal
    const n = reedsSheppCandidates(1.3, -0.7, 2.1).filter((w) => { const e = integrateWord(w); return Math.hypot(e[0] - 1.3, e[1] + 0.7) < 1e-6; }).length;
    expect(n).toBeGreaterThan(4);
    expect(dubinsCandidates(4, 1, 1).length).toBeGreaterThanOrEqual(4);
  });
});

describe('mobile: drive models (acceptance 5)', () => {
  it('omni strafes to (0, 2000) with < 5° heading change and no rotation in place', () => {
    const { r } = mk('omni');
    followPath(r, [[0, 0], [0, 2000]]);
    const res = run(r, 60);
    expect(res.done).toBe(true);
    expect(Math.hypot(r.state.x, r.state.y - 2000)).toBeLessThan(80);
    expect(Math.max(...res.trail.map((p) => Math.abs(p[2])))).toBeLessThan(5);
  });
  it('differential turns on the spot and drives with zero lateral velocity', () => {
    const { r } = mk('differential');
    followPath(r, [[0, 0], [0, 2000]]);
    const res = run(r, 60);
    expect(res.done).toBe(true);
    expect(Math.hypot(r.state.x, r.state.y - 2000)).toBeLessThan(80);
    expect(res.maxLat).toBeLessThan(5);
    expect(Math.abs(res.trail[Math.floor(res.trail.length / 2)][2] - 90)).toBeLessThan(10);
    expect(res.maxYaw).toBeLessThanOrEqual(r.kin.maxYawRate + 1e-6);
  });
  it('tracked with slip differs from differential on the same U-turn, both arrive', () => {
    const path = [[0, 0], [10000, 0], [10000, 3500], [0, 3500]];
    const d = mk('differential', { footprint: [1600, 1000, 800] });
    const t = mk('tracked', { footprint: [1600, 1000, 800], slip: 0.08, turnSlip: 0.6 });
    followPath(d.r, path); followPath(t.r, path);
    const rd = run(d.r, 300), rt = run(t.r, 300);
    expect(rd.done && rt.done).toBe(true);
    expect(Math.hypot(t.r.state.x, t.r.state.y - 3500)).toBeLessThan(150);
    // slip: the tracked robot needs longer and its trajectory differs measurably
    expect(rt.t).toBeGreaterThan(rd.t * 1.03);
    const n = Math.min(rd.trail.length, rt.trail.length);
    let maxDiff = 0;
    for (let i = 0; i < n; i += 10) maxDiff = Math.max(maxDiff, Math.hypot(rd.trail[i][0] - rt.trail[i][0], rd.trail[i][1] - rt.trail[i][1]));
    expect(maxDiff).toBeGreaterThan(300);
  });
  it('legged strafes slower than omni (lateral speed limit)', () => {
    const o = mk('omni'), l = mk('legged');
    for (const r of [o.r, l.r]) for (let i = 0; i < 60; i++) integrate(r, { v: 0, vy: 1000, omega: 0 }, 0.05);
    expect(o.r.state.vy).toBeCloseTo(1000, 3);
    expect(l.r.state.vy).toBeCloseTo(0.4 * l.r.kin.maxSpeed, 3);
    expect(Math.abs(l.r.state.x)).toBeLessThan(1e-6);
    expect(o.r.state.y).toBeGreaterThan(l.r.state.y * 1.4);
  });
  it('aerial takes off to cruise altitude, flies, and battery drains faster than on the ground', () => {
    const { r } = mk('aerial', { cruiseAltitude: 4000, maxClimbRate: 2000, maxSpeed: 5000, maxAccel: 2000 });
    r.battery.hoverW = 600;
    const lvl0 = r.battery.levelWh;
    followPath(r, [[0, 0], [20000, 5000]]);
    const res = run(r, 120);
    expect(res.done).toBe(true);
    expect(r.state.z!).toBeGreaterThan(3500);
    expect(r.poseAbs()[14]).toBeCloseTo(r.state.z!, 3);
    expect(Math.hypot(r.state.x - 20000, r.state.y - 5000)).toBeLessThan(150);
    const usedPerS = (lvl0 - r.battery.levelWh) / res.t;
    expect(usedPerS * 3600).toBeGreaterThan(600);
  });
});

describe('mobile: Ackermann bicycle model (acceptance 4)', () => {
  const ack = { wheelBase: 1800, maxSteer: 30, minTurnRadius: 3000, maxSteerRate: 40, maxSpeed: 2000, maxReverseSpeed: 800, footprint: [3000, 1500, 1500] as [number, number, number] };
  it('steering never exceeds maxSteer, no lateral velocity, curvature ≤ 1/R_min', () => {
    const { r } = mk('ackermann', ack);
    for (let i = 0; i < 200; i++) integrate(r, { v: 1500, omega: 200 }, 0.05);
    expect(Math.abs(r.state.steer)).toBeLessThanOrEqual(30 + 1e-9);
    const R = (Math.abs(r.state.v) / Math.abs(r.state.omega * Math.PI / 180));
    expect(R).toBeGreaterThanOrEqual(minTurnRadius(r.kin) - 1);
    // steering rate limit: from straight, steer grows at most maxSteerRate·dt per step
    const { r: r2 } = mk('ackermann', ack);
    integrate(r2, { v: 1000, omega: 0, steer: 30 }, 0.1);
    expect(Math.abs(r2.state.steer)).toBeLessThanOrEqual(4 + 1e-9);
  });
  it('reaches a goal 5 m behind within 60 s by reversing', () => {
    const { r } = mk('ackermann', ack);
    followPath(r, [[0, 0], [-5000, 0]]);
    const res = run(r, 60);
    expect(res.done).toBe(true);
    expect(res.t).toBeLessThan(60);
    expect(Math.hypot(r.state.x + 5000, r.state.y)).toBeLessThan(100);
    expect(res.maxLat).toBeLessThan(5);
  });
  it('a 2 m sideways goal is reached with a kinematically feasible manoeuvre', () => {
    const { r } = mk('ackermann', ack);
    followPath(r, [[0, 0], [0, 2000]], { goalHeading: 0 });
    const res = run(r, 120);
    expect(res.done).toBe(true);
    expect(Math.hypot(r.state.x, r.state.y - 2000)).toBeLessThan(100);
    expect(Math.abs(r.state.theta)).toBeLessThan(3);
    expect(res.maxSteer).toBeLessThanOrEqual(30 + 1e-9);
    expect(res.maxLat).toBeLessThan(5);
  });
});
