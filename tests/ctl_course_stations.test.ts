/**
 * Course examples A and B as runnable stations (library robots, zones, parts, programs, full bindings) driven through
 * the station runtime (I1). Assertions: robot / part motion order and timing, signals, markings — not status strings.
 */
import { describe, it, expect } from 'vitest';
import { ItemType, Station } from '../src/core/items/item';
import { StationRuntime } from '../src/core/motion/runtime';
import { parseDes, parsePetri, parseBt } from '../src/ctl/dsl';
import { AutomatonExecutor, PetriExecutor } from '../src/ctl/exec';
import { ControlRuntime } from '../src/ctl/runtime';
import { stationRuntimeBackend } from '../src/ctl/host';
import { makeStationWorld } from '../src/ctl/world';
import { synthesizeSupervisor, applyDeadlockMonitors } from '../src/ctl/supervisor';
import { buildCourseStationA, buildCourseStationB, YOUBOT_STATION_DES, YOUBOT_STATION_BT, CELL_STATION_PETRI } from '../src/ctl/course_stations';
import { ZoneItem, MobileRobot } from '../src/mobile/items';
import { analyse } from '../src/ctl/analysis';

/** Zone visits of the base (entry times) and the part's pose history. */
function recorder(st: Station, base: MobileRobot) {
  const zones = st.itemsOfType<ZoneItem>(ItemType.ZONE); const visits: Array<{ t: number; zone: string }> = []; let last = '';
  return { visits, sample(t: number) { const z = zones.find((zz) => zz.contains(base.state.x, base.state.y))?.name ?? ''; if (z && z !== last) visits.push({ t, zone: z }); last = z; } };
}
const inBin = (p: Float64Array | number[]) => Math.abs(p[12] - 4000) < 300 && Math.abs(p[13] - 3550) < 300 && p[14] > 400 - 1;

describe('course example A (youBot mobile manipulator) on the station', () => {
  it('the automaton under its supervisor drives the base table → bin → home, grasps at the table, releases into the bin, ends marked', () => {
    const { station: st, base, part } = buildCourseStationA(); const p0 = part.poseAbs();
    const rt = new StationRuntime(st); const world = makeStationWorld({ station: st, programs: stationRuntimeBackend(rt) });
    const sup = synthesizeSupervisor(YOUBOT_STATION_DES)!; expect(sup.implementable).toBe(true);
    const x = new AutomatonExecutor({ doc: parseDes(YOUBOT_STATION_DES), world, supervisor: sup.table });
    const rec = recorder(st, base); let partMovedAt = -1;
    for (let i = 0; i < 2400 && x.status === 'running'; i++) { x.tick(0.05); rec.sample(x.time); if (partMovedAt < 0 && Math.hypot(part.poseAbs()[12] - p0[12], part.poseAbs()[13] - p0[13]) > 50) partMovedAt = x.time; }
    expect(x.verdict).toBe('done'); expect(x.mismatches).toBe(0); expect(x.failed).toBe(0);
    expect(rec.visits.map((v) => v.zone)).toEqual(['home', 'table', 'bin', 'home']);
    const ev = (e: string, k = 0) => x.trace.filter((r) => r.event === e)[k]?.t ?? NaN;
    // order: arrive at the table, reach, grasp done (g_ok), stow, leave; arrive at the bin, reach, put, stow, go home
    expect(ev('b_arrive', 0)).toBeLessThan(ev('a_reached', 0)); expect(ev('a_reached', 0)).toBeLessThanOrEqual(ev('g_close')); expect(ev('g_close')).toBeLessThan(ev('g_ok'));
    expect(ev('g_ok')).toBeLessThanOrEqual(ev('a_stow', 0)); expect(ev('a_stowed', 0)).toBeLessThanOrEqual(ev('b_go_box'));
    expect(ev('b_arrive', 1)).toBeLessThan(ev('g_put')); expect(ev('g_put')).toBeLessThan(ev('b_go_home'));
    // the part leaves the table only after the grasp and ends in the bin; the cell signals report the delivery
    expect(partMovedAt).toBeGreaterThan(ev('g_close')); expect(inBin(part.poseAbs())).toBe(true);
    expect(x.bb.Holding).toBe(0); expect(x.bb.Delivered).toBe(1);
    expect(x.time).toBeGreaterThan(20); expect(x.time).toBeLessThan(45);
  });
  it('the BT mission on the same station (arm skills = the arm programs) delivers the part with 0 supervisor mismatches', () => {
    const { station: st, base, part } = buildCourseStationA();
    const rt = new StationRuntime(st); const world = makeStationWorld({ station: st, programs: stationRuntimeBackend(rt) });
    const w = world.forRobot('youBot')!;
    const ctl = new ControlRuntime({ bt: parseBt(YOUBOT_STATION_BT), supervisor: synthesizeSupervisor(YOUBOT_STATION_DES)!.table, bindings: { sense: () => world.sense!(), actions: { ...world.host.actions, ...w.actions }, halt: { ...world.host.halt, ...w.halt }, events: () => world.events!(), step: (dt) => world.step!(dt) } });
    ctl.bb.targets = 1; const rec = recorder(st, base);
    for (let i = 0; i < 2400 && !ctl.bb.reported; i++) { ctl.tick(0.05); rec.sample(ctl.time); }
    expect(ctl.bb.reported).toBe(true); expect(ctl.supervisorErrors).toBe(0); expect(ctl.denied).toBe(0);
    expect(rec.visits.map((v) => v.zone)).toEqual(['home', 'table', 'bin', 'home']);
    expect(rt.runs.map((r) => r.program.name)).toEqual(['Reach', 'Grasp', 'Stow', 'Reach', 'Release', 'Stow']);
    expect(inBin(part.poseAbs())).toBe(true);
  });
});

describe('course example B (production cell) on the station', () => {
  it('the course net deadlocks at {a1 b1} once M1 holds a part and the AMR is in zone Z: verdict deadlock, robots stopped', () => {
    const { station: st } = buildCourseStationB();
    const rt = new StationRuntime(st); const x = new PetriExecutor({ doc: parsePetri(CELL_STATION_PETRI), world: makeStationWorld({ station: st, programs: stationRuntimeBackend(rt) }) });
    x.run(120, 0.05);
    expect(x.verdict).toBe('deadlock'); expect(x.where()).toBe('a1 b1');
    expect(x.firings).toMatchObject({ t1a: 1, t1b: 1 }); expect(x.firings.t2a).toBeUndefined(); expect(x.firings.t2b).toBeUndefined();
    expect(x.time).toBeGreaterThan(3); expect(x.time).toBeLessThan(10); // A1 (3.2 s) and the AMR drive to Z have ended
    expect(x.log.some((l) => /DEADLOCK \(partial\)/.test(l))).toBe(true);
  });
  it('with the synthesised GMEC monitor a1 + b1 ≤ 1 the cell runs live: ≥ 10 cycles of A and B, M1 one program at a time, M2 concurrent', () => {
    const r = applyDeadlockMonitors(CELL_STATION_PETRI);
    expect(r.monitors.map((m) => m.constraint)).toEqual(['a1 + b1 ≤ 1']); expect(r.live).toBe(true);
    expect(analyse('petri', r.source).markdown).not.toMatch(/deadlock reachable/i);
    const { station: st } = buildCourseStationB();
    const rt = new StationRuntime(st); const x = new PetriExecutor({ doc: parsePetri(r.source), world: makeStationWorld({ station: st, programs: stationRuntimeBackend(rt) }) });
    x.run(600, 0.05);
    expect(x.status).toBe('running'); expect(x.firings.t3a).toBeGreaterThanOrEqual(10); expect(x.firings.t3b).toBeGreaterThanOrEqual(10);
    const runs = (robot: string) => rt.runs.filter((h) => (h.robot as { name?: string } | null)?.name === robot && h.startTime !== undefined).map((h) => [h.startTime!, h.endTime ?? x.time] as [number, number]);
    const m1 = runs('M1').sort((a, b) => a[0] - b[0]); for (let i = 1; i < m1.length; i++) expect(m1[i][0]).toBeGreaterThanOrEqual(m1[i - 1][1] - 1e-6);
    const m2 = runs('M2'); const overlap = m1.reduce((s, a) => s + m2.reduce((q, b) => q + Math.max(0, Math.min(a[1], b[1]) - Math.max(a[0], b[0])), 0), 0);
    expect(overlap).toBeGreaterThan(100); // M2 unloads the machine while M1 works
  });
});
