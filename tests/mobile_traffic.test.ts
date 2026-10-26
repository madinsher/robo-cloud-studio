// Acceptance (agri_mobile §5, items 6, 10, 15) and traffic management: head-on robots never overlap and both arrive,
// single-lane corridors and no-go zones are respected, allocators differ as designed, a station saved mid-mission
// resumes without finishing tasks early.
import { describe, it, expect } from 'vitest';
import { Station, ItemType } from '../src/core/items/item';
import { MobileRobot, MapItem, ZoneItem } from '../src/mobile/items';
import { FieldItem, cropParams } from '../src/agri/items';
import { generateOrchard, buildFieldMap } from '../src/agri/orchard';
import '../src/agri/missions';
import { FleetItem, FleetManager, FleetTask, AllocationMode } from '../src/fleet/fleet';
import { demos } from '../src/demos';
import { saveStation, loadStation } from '../src/io/station-file';
import { footprintsOverlap } from './agri_helpers';
import { pointInPolygon } from '../src/mobile/footprint';

function headOn(st: Station, fm: FleetManager, a: MobileRobot, b: MobileRobot, ga: [number, number], gb: [number, number], maxSteps = 12000) {
  const ta = fm.addTask({ type: 'goto', location: ga, duration: 1 });
  const tb = fm.addTask({ type: 'goto', location: gb, duration: 1 });
  expect(fm.startTask(a, ta)).toBe(true);
  expect(fm.startTask(b, tb)).toBe(true);
  let minD = Infinity, overlap = 0;
  for (let i = 0; i < maxSteps && !(ta.status === 'done' && tb.status === 'done'); i++) {
    fm.step(0.1);
    minD = Math.min(minD, Math.hypot(a.state.x - b.state.x, a.state.y - b.state.y));
    if (footprintsOverlap(a, b)) overlap++;
  }
  return { ta, tb, minD, overlap };
}

describe('traffic management (acceptance 6)', () => {
  it('two robots sent through the same orchard alley in opposite directions keep a footprint length apart and both arrive', () => {
    const st = new Station('alley');
    const f = st.addChild(new FieldItem('F'));
    f.polygon = [[0, 0], [30000, 0], [30000, 11500], [0, 11500]];
    f.crop = cropParams('apple', { rowHeading: 0, headland: 4000 });
    generateOrchard(f, 2);
    st.addChild(buildFieldMap(f, 250));
    const fleet = st.addChild(new FleetItem('fl'));
    const fm = new FleetManager(st, fleet);
    const mk = (n: string, x: number, th: number) => { const m = st.addChild(new MobileRobot(n)); Object.assign(m.kin, { drive: 'tracked', wheelBase: 1800, track: 1400, maxSpeed: 1300, footprint: [2600, 1600, 1400] }); m.setPose2D(x, 5750, th); fm.addRobot(m); return m; };
    const a = mk('A', -2000, 0), b = mk('B', 32000, 180);
    const res = headOn(st, fm, a, b, [33000, 5750], [-3000, 5750]);
    expect(res.ta.status).toBe('done');
    expect(res.tb.status).toBe('done');
    expect(res.overlap).toBe(0);
    expect(res.minD).toBeGreaterThanOrEqual(2600);
    expect(fm.kpis().contacts.robotRobotSteps).toBe(0);
  });

  it('a single-lane corridor (lane zone) is used by one robot at a time: the other waits, yields and both arrive', () => {
    const st = new Station('lane');
    const map = st.addChild(new MapItem('m'));
    map.resize(400, 200, 100, 0, 0);
    map.fillRect(12000, 0, 28000, 8500); map.fillRect(12000, 11500, 28000, 20000);
    const lane = st.addChild(new ZoneItem('Corridor')); lane.kind = 'lane'; lane.polygon = [[12000, 8500], [28000, 8500], [28000, 11500], [12000, 11500]];
    const fleet = st.addChild(new FleetItem('fl'));
    const fm = new FleetManager(st, fleet);
    const mk = (n: string, x: number, th: number) => { const m = st.addChild(new MobileRobot(n)); Object.assign(m.kin, { drive: 'differential', maxSpeed: 1200, footprint: [1800, 1000, 900] }); m.setPose2D(x, 10000, th); fm.addRobot(m); return m; };
    const a = mk('A', 6000, 0), b = mk('B', 34000, 180);
    let bothInside = 0;
    const poly = lane.worldPolygon();
    const orig = fm.step.bind(fm);
    fm.step = (dt: number) => { orig(dt); if (pointInPolygon(a.state.x, a.state.y, poly) && pointInPolygon(b.state.x, b.state.y, poly)) bothInside++; };
    const res = headOn(st, fm, a, b, [36000, 10000], [4000, 10000]);
    expect(res.ta.status).toBe('done');
    expect(res.tb.status).toBe('done');
    expect(res.overlap).toBe(0);
    expect(bothInside).toBe(0);
    expect(res.minD).toBeGreaterThanOrEqual(1800);
    expect(fm.kpis().traffic.waitSeconds).toBeGreaterThan(5);
  });

  it('planned paths never enter a no-go zone; speed-limit zones cap the speed', () => {
    const st = new Station('zones');
    const map = st.addChild(new MapItem('m'));
    map.resize(300, 200, 100, 0, 0);
    const nogo = st.addChild(new ZoneItem('Pond')); nogo.kind = 'nogo'; nogo.polygon = [[12000, 4000], [18000, 4000], [18000, 16000], [12000, 16000]];
    const slow = st.addChild(new ZoneItem('Slow')); slow.kind = 'speed_limit'; slow.speedLimit = 300; slow.polygon = [[20000, 0], [30000, 0], [30000, 20000], [20000, 20000]];
    const fleet = st.addChild(new FleetItem('fl'));
    const fm = new FleetManager(st, fleet);
    const r = st.addChild(new MobileRobot('R')); Object.assign(r.kin, { maxSpeed: 1500, footprint: [1200, 800, 600] }); r.setPose2D(3000, 10000, 0); fm.addRobot(r);
    const t = fm.addTask({ type: 'goto', location: [26000, 10000], duration: 1 });
    let inNogo = 0, maxSlow = 0;
    for (let i = 0; i < 6000 && t.status !== 'done'; i++) {
      fm.step(0.1);
      if (pointInPolygon(r.state.x, r.state.y, nogo.worldPolygon())) inNogo++;
      if (r.state.x > 20500) maxSlow = Math.max(maxSlow, Math.abs(r.state.v));
    }
    expect(t.status).toBe('done');
    expect(inNogo).toBe(0);
    expect(maxSlow).toBeLessThanOrEqual(301);
    expect(maxSlow).toBeGreaterThan(100);
  });
});

describe('allocators (acceptance 15)', () => {
  function scenario(mode: AllocationMode) {
    const st = new Station('alloc');
    const fleet = st.addChild(new FleetItem('fl'));
    fleet.allocation = mode;
    const fm = new FleetManager(st, fleet);
    const r1 = st.addChild(new MobileRobot('R1')); r1.setPose2D(0, 0, 0); fm.addRobot(r1);
    const r2 = st.addChild(new MobileRobot('R2')); r2.setPose2D(40000, 0, 0); fm.addRobot(r2);
    const ta = fm.addTask({ type: 'goto', location: [5000, 0], duration: 5, priority: 2 });
    const tb = fm.addTask({ type: 'goto', location: [500, 0], duration: 5, priority: 1 });
    fm.allocate();
    const who = (t: FleetTask) => (t.robotId ? st.findById(t.robotId)!.name : '-');
    return { a: who(ta), b: who(tb), statusB: tb.status };
  }
  it('nearest, auction (queued work), Hungarian (min total cost) and contract net give the designed assignments', () => {
    const nearest = scenario('nearest'), auction = scenario('auction'), hung = scenario('hungarian'), cnet = scenario('contract_net');
    // nearest: the high-priority task goes to the nearest idle robot (R1), the other task to the remaining robot
    expect(nearest).toEqual({ a: 'R1', b: 'R2', statusB: 'travelling' });
    // auction: R1 bids for the second task too (queued after the first, 4.5 m back, beats R2's 39.5 m drive)
    expect(auction.a).toBe('R1');
    expect(auction.b).toBe('R1');
    expect(auction.statusB).toBe('assigned');
    // Hungarian: one task per idle robot with the minimum total cost (R1 -> 0.5 m, R2 -> 35 m beats 5 m + 39.5 m)
    expect(hung).toEqual({ a: 'R2', b: 'R1', statusB: 'travelling' });
    expect(cnet.a).toBe('R1');
    expect([auction, hung, cnet].some((x) => x.b !== nearest.b)).toBe(true);
  });
  it('capabilities are enforced: a spray task waits for a robot that can spray', () => {
    const st = new Station('cap');
    const fleet = st.addChild(new FleetItem('fl'));
    const fm = new FleetManager(st, fleet);
    const r1 = st.addChild(new MobileRobot('Hauler')); r1.capabilities = ['transport']; fm.addRobot(r1);
    const t = fm.addTask({ type: 'spray', location: [3000, 0], duration: 5, requiredCapabilities: ['spray'] });
    for (let i = 0; i < 50; i++) fm.step(0.1);
    expect(t.status).toBe('pending');
    const r2 = st.addChild(new MobileRobot('Sprayer')); r2.capabilities = ['spray']; r2.setPose2D(0, 2000, 0); fm.addRobot(r2);
    for (let i = 0; i < 400 && t.status !== 'done'; i++) fm.step(0.1);
    expect(t.status).toBe('done');
    expect(t.robotId).toBe(r2.id);
  });
});

describe('mission persistence (acceptance 10)', () => {
  it('a station saved mid-mission resumes: no task finishes before its work path is traversed, paths and ids survive', () => {
    const st = demos.find((d) => d.id === 'orchard')!.build();
    const fleet = st.itemsOfType<FleetItem>(ItemType.FLEET)[0];
    const fm = new FleetManager(st, fleet);
    for (let i = 0; i < 6000; i++) fm.step(0.1); // 600 s: harvesters mid-row, sprayer done or on its rows
    const working = fleet.tasks.filter((t) => t.status === 'working' && t.type === 'harvest');
    expect(working.length).toBeGreaterThan(0);
    const doneBefore = new Set(fleet.tasks.filter((t) => t.status === 'done').map((t) => t.id));
    const file = JSON.parse(JSON.stringify(saveStation(st)));
    const st2 = loadStation(file);
    const fleet2 = st2.itemsOfType<FleetItem>(ItemType.FLEET)[0];
    const fm2 = new FleetManager(st2, fleet2);
    // robots keep the path they were following
    for (const r of st2.itemsOfType<MobileRobot>(ItemType.MOBILE_ROBOT)) {
      const orig = st.findById(r.id) as MobileRobot;
      expect(r.state.path?.length ?? 0).toBe(orig.state.path?.length ?? 0);
      if (orig.state.path) expect(r.state.pathIndex).toBe(orig.state.pathIndex);
    }
    const prematurely: string[] = [];
    const seen = new Set(doneBefore);
    for (let i = 0; i < 4000; i++) {
      fm2.step(0.1);
      for (const t of fleet2.tasks) {
        if (t.status !== 'done' || seen.has(t.id)) continue;
        seen.add(t.id);
        if (t.workPath && t.type !== 'charge') {
          const r = st2.findById(t.robotId!) as MobileRobot;
          const end = t.workPath[t.workPath.length - 1];
          if (Math.hypot(r.state.x - end[0], r.state.y - end[1]) > 1500) prematurely.push(t.id);
        }
      }
    }
    expect(prematurely).toEqual([]);
    // the resumed harvest tasks keep picking (fruit removed after the reload)
    expect(fm2.kpis().fruitPicked).toBeGreaterThan(0);
    // new tasks get ids that do not collide with the saved ones (module counter reset on reload)
    const ids = new Set(fleet2.tasks.map((t) => t.id));
    const nt = fm2.addTask({ type: 'goto', location: [0, 0] });
    expect(ids.has(nt.id)).toBe(false);
  }, 300000);
});
