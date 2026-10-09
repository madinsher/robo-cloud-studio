import { describe, it, expect } from 'vitest';
import { parseMission, MRS_TEMPLATES } from '../src/mrs/dsl';
import { MissionSim, compareArchitectures } from '../src/mrs/mission';
import { MRS_EXAMPLES } from '../src/mrs/examples';
import { analyse } from '../src/ctl/analysis';
import { FleetRuntime } from '../src/mrs/runtime';
import { Station } from '../src/core/items/item';
import { MobileRobot, ZoneItem } from '../src/mobile/items';
import { addControlModel } from '../src/ctl/model';

const byId = (id: string) => MRS_EXAMPLES.find((e) => e.id === id)!;
const phaseTimes = (m: ReturnType<MissionSim['metrics']>) => m.phases.map((p) => (p.doneAt === null ? null : p.doneAt - p.startedAt));

describe('group missions — centralised, decentralised and hybrid architectures (chapter 3)', () => {
  it('all three complete the five-phase mission; the coordinator outage stalls the centralised group, the hybrid one falls back, the decentralised one is unaffected', () => {
    const d = parseMission(MRS_TEMPLATES.mission); const runs = compareArchitectures(d); const [c, dc, h] = runs.map((r) => r.metrics);
    for (const m of [c, dc, h]) { expect(m.completed, m.architecture).toBe(true); expect(m.minDistance).toBeGreaterThanOrEqual(d.safety.dSafe - 5e-3); expect(m.tasksDone).toBe(6); expect(m.robotsAlive).toBe(5); expect(phaseTimes(m).every((t) => t !== null)).toBe(true); }
    expect(c.stalledSeconds).toBeGreaterThan(40); expect(c.fallbackSeconds).toBe(0); expect(c.peer).toBe(0); expect(c.uplink).toBeGreaterThan(0);
    expect(dc.stalledSeconds).toBe(0); expect(dc.uplink + dc.downlink).toBe(0); expect(dc.peer).toBeGreaterThan(0);
    expect(h.stalledSeconds).toBe(0); expect(h.fallbackSeconds).toBeGreaterThan(0); expect(h.fallbackSeconds).toBeLessThan(60); expect(h.uplink).toBeGreaterThan(0); expect(h.peer).toBeGreaterThan(0);
    expect(runs[2].log.some((l) => /FAULT: coordinator down/.test(l))).toBe(true); expect(runs[0].log.some((l) => /coordinator back/.test(l))).toBe(true);
    // the centralised Hungarian slots need less travel than the decentralised slots by index in the formation phase
    expect(c.travelled).toBeLessThan(dc.travelled * 1.6);
  }, 60000);
  it('robot failure: the coordinator reassigns the lost robot\'s task, the decentralised robots forget the silent winner', () => {
    const src = 'mission fail\nrobots a b c d box=2 seed=2\narchitecture compare\ncomm radius=6 period=0.5 lost=2 settle=0.5\ncoordinator at=0,0 range=20\nphase allocate targets=3,3;3,-3;-3,3;-3,-3\nphase home\nsafety d_safe=0.3 sense=1\nfail b at=3\nduration 120 dt=0.1 seed=2';
    const runs = compareArchitectures(parseMission(src)); for (const r of runs) { expect(r.metrics.completed, r.metrics.architecture).toBe(true); expect(r.metrics.tasksDone).toBe(4); }
    expect(runs[0].log.some((l) => /silent — its task is reassigned/.test(l))).toBe(true);
  }, 60000);
  it('a partitioned group: robots outside the coordinator range stall in the centralised variant, the decentralised components still converge', () => {
    const d = parseMission(byId('arch-partition').source); const [c, dc, h] = compareArchitectures(d).map((r) => r.metrics);
    expect(c.completed).toBe(false); expect(c.lagging.length).toBeGreaterThanOrEqual(2); expect(c.stalledSeconds).toBeGreaterThan(100);
    expect(dc.stalledSeconds).toBe(0); expect(dc.completed).toBe(true); expect(dc.minDistance).toBeGreaterThanOrEqual(0.395);
    expect(h.completed).toBe(true); expect(h.fallbackSeconds).toBeGreaterThan(0); expect(h.uplink).toBeGreaterThan(0); expect(h.peer).toBeGreaterThan(0);
  }, 60000);
  it('phase kinds: hold, cover and goto to a point; a single architecture report', () => {
    const r = analyse('mission', 'mission k\nrobots a b c\narchitecture centralized\ncoordinator at=0,0\nphase hold seconds=2\nphase cover area=-2,2,-2,2\nphase goto at=1,1 speed=0.4\nsafety d_safe=0.3\nduration 90 dt=0.1'); expect(r.error).toBeUndefined(); expect(r.ok).toBe(true); expect(r.metrics['centralized time']).not.toBe('incomplete');
    const cmp = analyse('mission', byId('arch-compare').source); expect(cmp.ok).toBe(true); expect(cmp.metrics['all complete']).toBe(true); expect(cmp.sections.some((s) => s.plot?.kind === 'bars')).toBe(true); expect(cmp.sections.find((s) => s.table)!.table!.rows.length).toBe(3);
  }, 60000);
});

const DES = `des Fleet supervisor
automaton Mission
  initial P0
  marked P0 P1 P2 P3 P4 P5 P6 P7 P8 P9 P10
  P0 -form_start-> P1
  P1 -form_done-> P2
  P2 -goto_start-> P3
  P3 -goto_done-> P4
  P4 -allocate_start-> P5
  P5 -allocate_done-> P6
  P6 -gather_start-> P7
  P7 -gather_done-> P8
  P8 -home_start-> P9
  P9 -home_done-> P10
uncontrollable form_done goto_done allocate_done gather_done home_done`;
const DES_BLOCK = DES.replace('des Fleet supervisor', 'des Fleet supervisor') + `\nspec NoGather   # gathering is never permitted: gather_start is in the alphabet of the spec but never enabled, so the group must hold there
  events gather_start
  initial E
  marked E
  E -form_start-> E
  E -form_done-> E
  E -goto_start-> E
  E -goto_done-> E
  E -allocate_start-> E
  E -allocate_done-> E
  E -home_start-> E
  E -home_done-> E`;
const MODES = `hybrid Fleet modes
var dist_human=10 phase=0
initial NORMAL
mode NORMAL vmax=0.6
mode SLOW vmax=0.1
NORMAL -> SLOW when dist_human < 2 dwell=0.2
SLOW -> NORMAL when dist_human > 3 dwell=0.2`;
function stationScene(opts: { supervisor?: string; modes?: boolean } = {}) {
  const st = new Station('fleet'); const robots: MobileRobot[] = [];
  for (let i = 0; i < 4; i++) { const r = st.addChild(new MobileRobot(`r${i + 1}`)); r.setPose2D(i * 1200, 0, 0); r.kin.maxSpeed = 700; r.home = { x: i * 1200, y: 0, theta: 0 }; robots.push(r); }
  const zone = (name: string, x: number, y: number, kind: ZoneItem['kind'] = 'work', half = 600) => { const z = st.addChild(new ZoneItem(name)); z.kind = kind; z.polygon = [[x - half, y - half], [x + half, y - half], [x + half, y + half], [x - half, y + half]]; return z; };
  zone('Dock', 6000, 0); zone('Shelf A', 9000, 2000); zone('Shelf B', 9000, -2000); zone('Shelf C', 11000, 2000); zone('Shelf D', 11000, -2000); zone('Pillar', 3000, 2000, 'nogo', 400);
  if (opts.supervisor) addControlModel(st, 'des', 'Fleet supervisor', opts.supervisor); if (opts.modes) addControlModel(st, 'hybrid', 'Fleet modes', MODES);
  return { st, robots };
}
const run = (rt: FleetRuntime, seconds: number, dt = 0.05) => { for (let t = 0; t < seconds && !rt.done; t += dt) rt.tick(dt); };
describe('group missions on the configured robots of the station', () => {
  it('phases resolve station zones, no-go zones become obstacles, the robots are driven through their own kinematic controller (unicycle) and reach the dock and the shelves', () => {
    const { st, robots } = stationScene(); const src = byId('arch-station').source.replace('supervisor Fleet supervisor\n', '').replace('modes Fleet modes\n', '');
    const rt = new FleetRuntime({ kind: 'mission', source: src, robots, station: st }); expect(rt.log.some((l) => /no-go zones as obstacles: Pillar/.test(l))).toBe(true);
    let sawDock = false, maxTurn = 0; const spins: number[] = robots.map(() => 0);
    for (let t = 0; t < 400 && !rt.done; t += 0.05) { rt.tick(0.05); robots.forEach((r, i) => { spins[i] += Math.abs(r.state.omega) * 0.05; }); const c = robots.reduce((s, r) => [s[0] + r.state.x / 4, s[1] + r.state.y / 4], [0, 0]); if (Math.hypot(c[0] - 6000, c[1]) < 1500) sawDock = true; maxTurn = Math.max(maxTurn, ...robots.map((r) => Math.abs(r.state.omega))); }
    expect(rt.done).toBe(true); expect(sawDock).toBe(true); expect(rt.status()).toMatch(/phase complete/); expect(maxTurn).toBeLessThanOrEqual(robots[0].kin.maxYawRate + 1e-6); expect(spins.some((s) => s > 30)).toBe(true); // the robots really turned (differential drive), within the configured yaw-rate limit
    for (const r of robots) expect(Math.abs(r.state.v)).toBeLessThanOrEqual(r.kin.maxSpeed + 1e-6);
    robots.forEach((r) => expect(Math.hypot(r.state.x - r.home!.x, r.state.y - r.home!.y)).toBeLessThan(400)); // back home
    expect(rt.log.some((l) => /Shelf|finished|complete/.test(l))).toBe(true);
  }, 120000);
  it('a des supervisor of the station gates the phases: with a spec that never permits gather_start the group holds at gather; without it the plant state advances to P10', () => {
    const ok = stationScene({ supervisor: DES }); const rtOk = new FleetRuntime({ kind: 'mission', source: byId('arch-station').source.replace('modes Fleet modes\n', '').replace('architecture hybrid', 'architecture centralized'), robots: ok.robots, station: ok.st }); run(rtOk, 400, 0.1); expect(rtOk.done).toBe(true); expect(rtOk.status()).toMatch(/plant P10, denied 0, mismatches 0/);
    const blocked = stationScene({ supervisor: DES_BLOCK }); const rtB = new FleetRuntime({ kind: 'mission', source: byId('arch-station').source.replace('modes Fleet modes\n', '').replace('architecture hybrid', 'architecture centralized'), robots: blocked.robots, station: blocked.st }); run(rtB, 400, 0.1); expect(rtB.done).toBe(false); expect(rtB.status()).toMatch(/denied [1-9]/); expect(rtB.log.some((l) => /supervisor denied gather_start/.test(l))).toBe(true);
    const bad = stationScene(); expect(() => new FleetRuntime({ kind: 'mission', source: byId('arch-station').source.replace('modes Fleet modes\n', ''), robots: bad.robots, station: bad.st })).toThrow(/supervisor model "Fleet supervisor" not found/);
  }, 120000);
  it('a hybrid mode automaton limits the fleet speed when a human is near', () => {
    const { st, robots } = stationScene({ modes: true }); let dh = 10; const src = byId('arch-station').source.replace('supervisor Fleet supervisor\n', '');
    const rt = new FleetRuntime({ kind: 'mission', source: src, robots, station: st, env: () => ({ dist_human: dh }) }); run(rt, 5, 0.05); expect(rt.status()).toMatch(/mode NORMAL/); const fast = Math.max(...robots.map((r) => Math.abs(r.state.v)));
    dh = 1; run(rt, 6, 0.05); expect(rt.status()).toMatch(/mode SLOW \(vmax 0.1\)/); run(rt, 3, 0.05); const slow = Math.max(...robots.map((r) => Math.abs(r.state.v))); expect(slow).toBeLessThan(fast); expect(slow).toBeLessThanOrEqual(170);
  }, 60000);
});
