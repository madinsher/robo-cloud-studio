// V3 F4 (medium): an Ackermann base is referenced to its rear axle — the bicycle model, the turning-radius limit, the
// Hybrid A* arcs / Reeds–Shepp joins, the path tracker and the S1 legs hold there, the item origin (footprint centre,
// the Webots UTP-G proto origin = wheelbase midpoint) follows as a rigid body. Measured from consecutive poses, the rear
// wheels must not slide sideways (< 0.5°), the origin does (atan(a·κ) in a turn): before the fix the origin had no side
// slip and the rear axle slid 8–19° (S1 UTP-G, orchard Sprayer).
import { describe, it, expect, beforeAll } from 'vitest';
import { Station, ItemType } from '../src/core/items/item';
import { MobileRobot, MapItem } from '../src/mobile/items';
import * as controller from '../src/mobile/controller';
import { integrate, minTurnRadius } from '../src/mobile/controller';
import { planForRobot, headlandTurns } from '../src/mobile/planner';
import { planAndFollow } from '../src/mobile/navigate';
import { stationWorld } from '../src/mobile/world';
import { demos } from '../src/demos';
import { FleetItem, FleetManager } from '../src/fleet/fleet';
import { plantDisks, cropClearance } from './agri_helpers';
import { HAS_MURKA, MURKA } from './webots_helpers';

const DEG = Math.PI / 180;
/** Rear axle of an Ackermann base (body x): kin.rearAxle, else the axles are symmetric about the origin. */
const rearOf = (k: MobileRobot['kin']) => k.rearAxle ?? -k.wheelBase / 2;
type P2 = { x: number; y: number; th: number };
const at = (p: P2, a: number) => [p.x + Math.cos(p.th * DEG) * a, p.y + Math.sin(p.th * DEG) * a];

/** Side-slip angle (deg) of the body point x = a between two poses (mid heading); null when it hardly moved. */
function slip(p: P2, q: P2, a: number): { deg: number; lon: number } | null {
  const [x0, y0] = at(p, a), [x1, y1] = at(q, a);
  const dx = x1 - x0, dy = y1 - y0;
  if (Math.hypot(dx, dy) < 2) return null;
  const th = (p.th + ((((q.th - p.th) + 540) % 360) - 180) / 2) * DEG;
  const lon = Math.cos(th) * dx + Math.sin(th) * dy, lat = -Math.sin(th) * dx + Math.cos(th) * dy;
  return { deg: Math.atan2(Math.abs(lat), Math.abs(lon)) / DEG, lon };
}

/** Slip statistics over a sampled trajectory; windows containing a cusp (the motion reverses inside) are skipped. */
function slipStats(poseAt: (t: number) => P2, t0: number, t1: number, dt: number, a: number) {
  const vals: number[] = [];
  let cusps = 0;
  for (let t = t0; t + dt <= t1 + 1e-9; t += dt) {
    const w = slip(poseAt(t), poseAt(t + dt), a);
    if (!w) continue;
    // cusp: some sub-step of the window moves against the window's net direction
    let rev = false;
    for (let k = 0; k < 10 && !rev; k++) { const s = slip(poseAt(t + (dt * k) / 10), poseAt(t + (dt * (k + 1)) / 10), a); if (s && s.lon * w.lon < 0) rev = true; }
    if (rev) { cusps++; continue; }
    vals.push(w.deg);
  }
  vals.sort((x, y) => x - y);
  return { n: vals.length, cusps, max: vals.length ? vals[vals.length - 1] : 0, p50: vals.length ? vals[vals.length >> 1] : 0 };
}

/** Heading change per arc length (1/mm) of the body point x = a along a pose path, over windows without a cusp. */
function maxCurvature(path: number[][], a: number, window = 300): number {
  const pts = path.map((p) => { const [x, y] = at({ x: p[0], y: p[1], th: p[2] }, a); return [x, y, p[2], p[3]]; });
  const s = [0];
  for (let i = 1; i < pts.length; i++) s.push(s[i - 1] + Math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]));
  let k = 0;
  for (let i = 0, j = 0; i < pts.length; i++) {
    while (j < pts.length - 1 && s[j] - s[i] < window) j++;
    if (s[j] - s[i] < window * 0.5) continue;
    if (pts.slice(i + 1, j + 1).some((p, q) => p[3] !== pts[i + q][3] && q > 0) || pts.slice(i + 1, j + 1).some((p) => p[3] === 0)) continue;
    k = Math.max(k, Math.abs((((pts[j][2] - pts[i][2]) + 540) % 360) - 180) * DEG / (s[j] - s[i]));
  }
  return k;
}

const SPRAYER = { drive: 'ackermann' as const, wheelBase: 1800, track: 1300, wheelRadius: 400, maxSpeed: 2500, footprint: [3000, 1500, 1500] as [number, number, number], minTurnRadius: 3000 };

describe('Ackermann bases are referenced to the rear axle (V3 F4)', () => {
  it('bicycle model: the rear axle rolls on a circle of radius L/tan δ without side slip, the origin slips by atan(a/R)', () => {
    const st = new Station('k');
    const r = st.addChild(new MobileRobot('car'));
    Object.assign(r.kin, SPRAYER, { maxSteerRate: 1000, maxYawRate: 1000 });
    r.setPose2D(0, 0, 0);
    const a = rearOf(r.kin);
    expect(a).toBe(-900); // default: axles symmetric about the origin
    const poses: P2[] = [];
    for (let i = 0; i < 400; i++) { integrate(r, { v: 1000, omega: 0, steer: 25 }, 0.05); poses.push({ x: r.state.x, y: r.state.y, th: r.state.theta }); }
    const s = slipStats((t) => poses[Math.round(t / 0.05)], 1, 19.9, 0.05, a);
    expect(s.max).toBeLessThan(1e-6);
    expect(controller.rearAxleOffset(r.kin)).toBe(-900);
    const o = slipStats((t) => poses[Math.round(t / 0.05)], 1, 19.9, 0.05, 0);
    const R = 1800 / Math.tan(25 * DEG);
    expect(o.p50).toBeCloseTo(Math.atan(900 / R) / DEG, 3);
    // rear-axle circle: the rear axle stays at distance R from the instantaneous centre (fit through three points)
    const [p1, p2, p3] = [poses[100], poses[200], poses[300]].map((p) => at(p, a));
    const D = 2 * (p1[0] * (p2[1] - p3[1]) + p2[0] * (p3[1] - p1[1]) + p3[0] * (p1[1] - p2[1]));
    const ux = ((p1[0] ** 2 + p1[1] ** 2) * (p2[1] - p3[1]) + (p2[0] ** 2 + p2[1] ** 2) * (p3[1] - p1[1]) + (p3[0] ** 2 + p3[1] ** 2) * (p1[1] - p2[1])) / D;
    const uy = ((p1[0] ** 2 + p1[1] ** 2) * (p3[0] - p2[0]) + (p2[0] ** 2 + p2[1] ** 2) * (p1[0] - p3[0]) + (p3[0] ** 2 + p3[1] ** 2) * (p2[0] - p1[0])) / D;
    expect(Math.hypot(p1[0] - ux, p1[1] - uy)).toBeCloseTo(R, 0);
    // the odometer counts the rear-axle distance: 20 s at 1 m/s (after the 1.25 s ramp)
    expect(r.state.odometer).toBeGreaterThan(19000);
    expect(r.state.odometer).toBeLessThan(20000);
  });

  it('Hybrid A* and Reeds–Shepp joins: curvature ≤ 1/R at the rear axle (not at the origin)', () => {
    const st = new Station('walls');
    const map = st.addChild(new MapItem('map'));
    map.resize(400, 300, 100, 0, 0);
    map.fillRect(15000, 0, 16000, 20000);
    const r = st.addChild(new MobileRobot('car'));
    Object.assign(r.kin, SPRAYER);
    const R = minTurnRadius(r.kin) * 1.15;
    const res = planForRobot(r, { x: 5000, y: 5000, theta: 0 }, { x: 25000, y: 5000 }, { world: stationWorld(st), goalHeading: 180 });
    expect(res.ok).toBe(true);
    expect(maxCurvature(res.path, -900)).toBeLessThanOrEqual((1 / R) * 1.02);
    expect(maxCurvature(res.path, 0)).toBeLessThan((1 / R) * 0.99); // the origin turns on a wider circle
    // agri headland turns (the Sprayer's row change) are rear-axle Reeds–Shepp paths too
    const turns = headlandTurns({ x: 0, y: 0, theta: 0 }, { x: 0, y: 5000, theta: 180 }, R, [3000, 1500], rearOf(r.kin));
    expect(turns.length).toBeGreaterThan(0);
    for (const t of turns) expect(maxCurvature(t.path, -900)).toBeLessThanOrEqual((1 / R) * 1.02);
  });

  it('Navigate with the tracker (walled map, reverse docking): rear wheels never slide, footprint clear of the walls', () => {
    const st = new Station('nav');
    const map = st.addChild(new MapItem('map'));
    map.resize(200, 150, 100, 0, 0);
    map.fillRect(8000, 11000, 8200, 15000); map.fillRect(10600, 11000, 10800, 15000); map.fillRect(8000, 14800, 10800, 15000);
    const r = st.addChild(new MobileRobot('car'));
    Object.assign(r.kin, { drive: 'ackermann', footprint: [1400, 900, 800], wheelBase: 1000, maxSteer: 35, minTurnRadius: 1500, maxSteerRate: 60, maxSpeed: 1500, maxReverseSpeed: 600 });
    r.setPose2D(3000, 3000, 0);
    const res = planAndFollow(st, r, { x: 9400, y: 13400, heading: -90, reverse: true });
    expect(res.ok).toBe(true);
    const poseAt = (t: number) => { const p = res.sample(t); return { x: p.x, y: p.y, th: p.heading }; };
    const s = slipStats(poseAt, 0, res.duration, 0.05, rearOf(r.kin));
    expect(s.n).toBeGreaterThan(200);
    expect(s.max).toBeLessThan(0.5);
    const w = stationWorld(st);
    let minClear = Infinity;
    for (let t = 0; t <= res.duration; t += 0.05) { const p = res.sample(t); minClear = Math.min(minClear, w.clearance(p.x, p.y, p.heading, 1400, 900)); }
    expect(minClear).toBeGreaterThan(0);
  });

  it('orchard demo Sprayer (fleet, 300 s of spraying): rear wheels never slide, no contact with the trees', () => {
    const st = demos.find((d) => d.id === 'orchard')!.build();
    const fleet = st.itemsOfType<FleetItem>(ItemType.FLEET)[0];
    const fm = new FleetManager(st, fleet);
    const sp = st.itemsOfType<MobileRobot>(ItemType.MOBILE_ROBOT).find((m) => m.kin.drive === 'ackermann')!;
    const plants = plantDisks(st);
    const poses: P2[] = [];
    let trunk = Infinity, canopy = Infinity;
    for (let i = 0; i < 6000; i++) {
      fm.step(0.05);
      poses.push({ x: sp.state.x, y: sp.state.y, th: sp.state.theta });
      const c = cropClearance(sp, plants);
      trunk = Math.min(trunk, c.trunk); canopy = Math.min(canopy, c.canopy);
    }
    const a = rearOf(sp.kin);
    const s = slipStats((t) => poses[Math.min(poses.length - 1, Math.round(t / 0.05))], 0, 299.9, 0.05, a);
    expect(s.n).toBeGreaterThan(3000);
    expect(s.max).toBeLessThan(0.5);
    // the origin (footprint centre) does slip in the headland turns: the reference point really moved to the axle
    expect(slipStats((t) => poses[Math.min(poses.length - 1, Math.round(t / 0.05))], 0, 299.9, 0.05, 0).max).toBeGreaterThan(10);
    expect(trunk).toBeGreaterThan(0);
    expect(canopy).toBeGreaterThan(0);
    expect(fleet.tasks.filter((t) => t.type === 'spray' && t.status === 'done').length).toBeGreaterThanOrEqual(3);
    expect(fm.kpis().contacts.robotObstacleSteps).toBe(0);
  }, 120000);
});

describe.skipIf(!HAS_MURKA)('S1 UTP-G platforms on the rear axle (needs the MURKA folder)', () => {
  let o: Awaited<ReturnType<typeof import('../src/murka/agro/orchard').buildMurkaOrchardStation>>;
  beforeAll(async () => { const { buildMurkaOrchardStation } = await import('../src/murka/agro/orchard'); o = await buildMurkaOrchardStation({ world: 'orchard_demo', root: MURKA }); }, 120000);

  it('A / Hungarian / seed 0: the rear axle (−1.095 m) of every UTP-G rolls without side slip, R ≥ 7.08 m there', async () => {
    const { runS1Benchmark } = await import('../src/murka/agro/s1');
    const robots = o.station.itemsOfType<MobileRobot>(ItemType.MOBILE_ROBOT);
    const res = runS1Benchmark(o.station, { level: 'A', controller: 'hungarian', seed: 0, assets: o.assets, contacts: false });
    expect(res.metrics.outcome).toBe('done');
    for (const n of ['UTP-G-01', 'UTP-G-03', 'UTP-G-04']) {
      const poseAt = (t: number) => res.timeline.poseAt(n, t);
      const rear = slipStats(poseAt, 0, res.timeline.end, 0.1, -1095);
      expect(rear.n, n).toBeGreaterThan(2000);
      expect(rear.max, n).toBeLessThan(0.5);
      // the wheelbase midpoint slips by atan(1.095 / 7.08) = 8.8° in the turns
      expect(slipStats(poseAt, 0, res.timeline.end, 0.1, 0).max, n).toBeGreaterThan(8);
      expect(res.metrics.kinematics.minTurnRadius[n], n).toBeGreaterThan(7079);
      expect(res.metrics.kinematics.maxStep[n], n).toBeLessThanOrEqual((n === 'UTP-G-04' ? 1000 : 600) * 0.1 + 1e-6);
      // the importer records the proto's rear wheels (x = −1.095 m) as the rear axle
      const k = robots.find((m) => m.name === n)!.kin;
      expect(k.rearAxle, n).toBeCloseTo(-1095, 6);
      expect(controller.rearAxleOffset(k), n).toBeCloseTo(-1095, 6);
    }
  }, 300000);
});
