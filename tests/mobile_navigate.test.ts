// T8: goal navigation (contract I8), precision docking and program motion of mobile bases (contract I2).
import { describe, it, expect } from 'vitest';
import { Station } from '../src/core/items/item';
import { Program, PathItem } from '../src/core/items/program';
import { MobileRobot, MapItem } from '../src/mobile/items';
import { planAndFollow } from '../src/mobile/navigate';
import { mobileProgramPlanner } from '../src/mobile/program_motion';
import { getMobileMotionPlanner, setMobileMotionPlanner } from '../src/core/motion/mobile_hook';
import { ProgramSimulator } from '../src/core/motion/simulator';
import { stationWorld } from '../src/mobile/world';

function station(drive: MobileRobot['kin']['drive'], kin: Partial<MobileRobot['kin']> = {}) {
  const st = new Station('nav');
  const map = st.addChild(new MapItem('map'));
  map.resize(200, 150, 100, 0, 0);
  // a bay: walls on three sides of a 2.2 m wide slot at the top of the map
  map.fillRect(8000, 11000, 8200, 15000);
  map.fillRect(10600, 11000, 10800, 15000);
  map.fillRect(8000, 14800, 10800, 15000);
  const r = st.addChild(new MobileRobot('AMR'));
  Object.assign(r.kin, { drive, footprint: [1400, 900, 800], ...kin });
  r.setPose2D(3000, 3000, 0);
  return { st, r };
}

describe('mobile navigation (I8) and precision docking', () => {
  it('importing the mobile items registers the mobile program planner (I2)', () => {
    expect(getMobileMotionPlanner()).toBe(mobileProgramPlanner);
  });

  it('planAndFollow drives a differential robot into a bay forwards within 20 mm / 1 deg without touching the walls', () => {
    const { st, r } = station('differential');
    const res = planAndFollow(st, r, { x: 9400, y: 13600, heading: 90, precise: true });
    expect(res.ok).toBe(true);
    expect(res.finalError!).toBeLessThanOrEqual(20);
    expect(res.finalHeadingError!).toBeLessThanOrEqual(1);
    const w = stationWorld(st);
    let minClear = Infinity;
    for (let t = 0; t <= res.duration; t += 0.05) { const p = res.sample(t); minClear = Math.min(minClear, w.clearance(p.x, p.y, p.heading, 1400, 900)); }
    expect(minClear).toBeGreaterThan(0);
    // the robot itself did not move: navigation is planned and simulated on a copy
    expect([r.state.x, r.state.y]).toEqual([3000, 3000]);
    // 13 m of travel at <= 1.5 m/s with turns takes longer than the straight-line time
    expect(res.duration).toBeGreaterThan(Math.hypot(6400, 10600) / 1500);
  });

  it('an Ackermann robot reverses into the bay (cusp, last leg backwards) and docks within 20 mm', () => {
    const { st, r } = station('ackermann', { wheelBase: 1000, maxSteer: 35, minTurnRadius: 1500, maxSteerRate: 60, maxSpeed: 1500, maxReverseSpeed: 600 });
    const res = planAndFollow(st, r, { x: 9400, y: 13400, heading: -90, reverse: true });
    expect(res.ok).toBe(true);
    expect(res.finalError!).toBeLessThanOrEqual(20);
    expect(res.finalHeadingError!).toBeLessThanOrEqual(1);
    const path = res.path!;
    expect(path[path.length - 1][3]).toBe(-1); // arrives driving backwards
    // while entering the bay the robot moves against its heading (reverse)
    const a = res.sample(res.duration - 2), b = res.sample(res.duration - 1);
    const fwd = (b.x - a.x) * Math.cos(a.heading * Math.PI / 180) + (b.y - a.y) * Math.sin(a.heading * Math.PI / 180);
    expect(fwd).toBeLessThan(0);
  });

  it('reports a failure (not a zero-time success) when the goal is inside an obstacle', () => {
    const { st, r } = station('differential');
    const res = planAndFollow(st, r, { x: 8100, y: 13000 });
    expect(res.ok).toBe(false);
    expect(res.error).toBeTruthy();
  });
});

describe('mobile program motion (I2)', () => {
  it('Navigate instructions drive the base along a planned, drive-feasible trajectory in the program timeline', () => {
    const { st, r } = station('differential', { maxSpeed: 1000, maxAccel: 500 });
    const p = st.addChild(new Program('Drive'));
    p.setRobot(r);
    p.navigateTo(9400, 13600, 90, { speed: 800 });
    p.navigateTo(3000, 3000, 0, { speed: 800 });
    const sim = new ProgramSimulator(st);
    const res = sim.compile(p);
    expect(res.problems.filter((x) => x.severity === 'error')).toEqual([]);
    // two legs of >= 12.3 m each at 0.8 m/s (around the bay walls: longer than the straight line)
    expect(res.duration).toBeGreaterThan((2 * Math.hypot(6400, 10600)) / 800);
    sim.seek(res.duration / 2 - 1);
    const mid = { x: r.state.x, y: r.state.y };
    expect(Math.hypot(mid.x - 9400, mid.y - 13600)).toBeLessThan(1500);
    sim.runToEnd();
    expect(Math.hypot(r.state.x - 3000, r.state.y - 3000)).toBeLessThan(100);
  });

  it('Follow path instructions use the drive model on the path polyline (Ackermann: rounded corners, no turn on the spot)', () => {
    const st = new Station('follow');
    const r = st.addChild(new MobileRobot('Tractor'));
    Object.assign(r.kin, { drive: 'ackermann', wheelBase: 2000, maxSteer: 30, minTurnRadius: 4000, maxSpeed: 2000, footprint: [3500, 1800, 2000] });
    r.setPose2D(0, 0, 0);
    const path = st.addChild(new PathItem('Lane'));
    path.points = [[10000, 0], [10000, 10000]];
    const p = st.addChild(new Program('Lane'));
    p.setRobot(r);
    p.addInstruction({ kind: 'mobile_follow', pathId: path.id, speed: 1500 });
    const sim = new ProgramSimulator(st);
    const res = sim.compile(p);
    expect(res.problems.filter((x) => x.severity === 'error')).toEqual([]);
    expect(res.duration).toBeGreaterThan(20000 / 1500 * 0.8);
    let maxYawRate = 0, prev = { th: 0, t: 0 };
    for (let t = 0.1; t <= res.duration; t += 0.1) { sim.seek(t); const d = Math.abs(((r.state.theta - prev.th + 540) % 360) - 180); maxYawRate = Math.max(maxYawRate, d / 0.1); prev = { th: r.state.theta, t }; }
    // turning at R >= 4 m at <= 2 m/s: yaw rate <= v / R = 29 deg/s (a turn on the spot would be much faster)
    expect(maxYawRate).toBeLessThan(35);
    sim.runToEnd();
    expect(Math.hypot(r.state.x - 10000, r.state.y - 10000)).toBeLessThan(200);
  });

  it('items that are not mobile robots keep the straight-line default', () => {
    const st = new Station('x');
    const prog = st.addChild(new Program('p'));
    const plan = mobileProgramPlanner(st, prog, { kind: 'mobile_move', x: 1000, y: 0, speed: 500 }, { x: 0, y: 0, heading: 0 });
    expect(plan.ok).toBe(true);
    expect(plan.duration).toBeCloseTo(2, 6);
    setMobileMotionPlanner(mobileProgramPlanner);
  });
});
