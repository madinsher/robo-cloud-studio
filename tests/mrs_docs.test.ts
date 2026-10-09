import { describe, it, expect } from 'vitest';
import { analyse } from '../src/ctl/analysis';
import { TEMPLATES, detectKind } from '../src/ctl/dsl';
import { CONTROL_KINDS } from '../src/ctl/model';
import { MRS_KINDS } from '../src/mrs/model';
import { MRS_TEMPLATES, parseConsensus, parseWarehouse, parseGridMapf, parseGame } from '../src/mrs/dsl';
import { MRS_EXAMPLES } from '../src/mrs/examples';
import { FleetRuntime, buildWarehouseScene, RUNNABLE_KINDS } from '../src/mrs/runtime';
import { Station } from '../src/core/items/item';
import { MobileRobot } from '../src/mobile/items';

const byId = (id: string) => MRS_EXAMPLES.find((e) => e.id === id)!;

describe('group-control module — documents, templates, examples', () => {
  it('the 15 kinds are registered in the Control kinds under the multi-robot group, with templates that detect and analyse', () => {
    expect(MRS_KINDS.length).toBe(15); for (const k of MRS_KINDS) { const c = CONTROL_KINDS.find((x) => x.kind === k.kind)!; expect(c.group).toBe('multi-robot'); expect(c.practicum).toBe(k.practicum); expect(TEMPLATES[k.kind]).toBe(MRS_TEMPLATES[k.kind]); expect(detectKind(MRS_TEMPLATES[k.kind])).toBe(k.kind); const r = analyse(k.kind, MRS_TEMPLATES[k.kind]); expect(r.error, `${k.kind}: ${r.error}`).toBeUndefined(); expect(r.ok, `${k.kind} template should be OK`).toBe(true); expect(r.sections.some((s) => s.plot || s.graph || s.table)).toBe(true); }
  });
  it('every course example analyses without error and the verdicts match the course', () => {
    const R = Object.fromEntries(MRS_EXAMPLES.map((e) => [e.id, analyse(e.kind, e.source)]));
    for (const [id, r] of Object.entries(R)) expect(r.error, `${id}: ${r.error}`).toBeUndefined();
    expect(new Set(MRS_EXAMPLES.map((e) => e.id)).size).toBe(MRS_EXAMPLES.length);
    expect(R['pr1-chain'].metrics['lambda2']).toBeCloseTo(0.5858, 3); expect(R['pr1-chain'].metrics['rho']).toBeCloseTo(0.8536, 3); expect(R['pr1-chain'].metrics['iterations to 1%']).toBe(30);
    expect(R['pr1-formation'].ok).toBe(true); expect(R['pr1-rendezvous'].metrics['links broken']).toBe(0); expect(R['pr1-wmsr'].ok).toBe(true); expect(R['pr1-wmsr'].metrics['robustness']).toBeGreaterThanOrEqual(3);
    expect(R['pr1-sparse'].ok).toBe(false); expect(R['pr1-sparse'].metrics['robustness']).toBe(1);
    expect(R['pr2-boids'].metrics['polarization']).toBeGreaterThan(0.9); expect(R['pr2-vicsek'].metrics['order low noise']).toBeGreaterThan(0.8); expect(R['pr2-vicsek'].metrics['order high noise']).toBeLessThan(0.3); expect(R['pr2-pso'].metrics['source error']).toBeLessThan(0.02); expect(R['pr2-robots'].metrics['source error']).toBeLessThan(0.3); expect(R['pr2-aco'].ok).toBe(true);
    expect(R['pr3-course-matrix'].metrics['greedy / optimal']).toBeGreaterThan(1.2); expect(R['pr3-ssi-cbba'].metrics['cbba conflict-free']).toBe(true); expect(R['pr3-line'].metrics['cbba conflict-free']).toBe(true);
    expect(R['pr4-warehouse'].metrics['adg collisions']).toBe(0); expect(R['pr4-incomplete'].metrics['prioritized SOC']).toBe('fail'); expect(R['pr4-incomplete'].metrics['cbs SOC']).toBeGreaterThan(0); expect(R['pr4-random'].metrics['adg collisions']).toBe(0);
    expect(R['pr5-lloyd'].metrics['monotone']).toBe(true); expect(R['pr5-lloyd'].metrics['reduction %']).toBeGreaterThan(70); expect(R['pr5-limited'].ok).toBe(true); expect(R['pr5-estimation'].metrics['final deviation']).toBeLessThan(1e-4); expect(R['pr5-estimation'].metrics['ci omega']).toBe(0.5);
    expect(R['pr6-crossing'].metrics['min distance']).toBeGreaterThanOrEqual(0.197); expect(R['pr6-antipodal'].metrics['deadlock without rule']).toBe(true); expect(R['pr6-antipodal'].metrics['deadlock']).toBe(false); expect(R['pr6-stale'].metrics['min distance']).toBeGreaterThanOrEqual(0.297);
    expect(R['hw-baseline'].metrics['path conflicts']).toBe(0); expect(R['hw-baseline'].metrics['double commits']).toBe(0); expect(R['hw-baseline'].metrics['delivered']).toBeGreaterThanOrEqual(15); expect(R['hw-radio'].metrics['delivered']).toBeGreaterThan(0); expect(R['hw-faults'].metrics['path conflicts']).toBe(0); expect(R['hw-faults'].sections.some((s) => s.lines.some((l) => /recovered/.test(l)))).toBe(true);
    expect(R['ch9-game'].metrics['pure equilibria']).toBe(2); expect(R['ch9-game'].metrics['mixed payoff']).toBe(5.2); expect(R['ch9-game'].metrics['shapley']).toBe('26.67 / 41.67 / 51.67'); expect(R['ch9-game'].metrics['in core']).toBe(true); expect(R['ch9-dilemma'].metrics['pure equilibria']).toBe(1);
    expect(R['ch7-qlearning'].metrics['Q*(start)']).toBe(6.2); expect(R['ch7-qlearning'].metrics['greedy steps']).toBe(3); expect(R['ch7-grid'].ok).toBe(true);
    expect(R['ch12-evo'].metrics['ga best x']).toBe(31); expect(R['ch12-evo'].ok).toBe(true); expect(R['ch4-ca'].metrics['life period']).toBe(4); expect(R['ch4-rule30'].metrics['ca symmetric']).toBe(false);
    expect(R['ch13-fuzzy'].metrics['v']).toBeGreaterThan(0.25); expect(R['ch13-fuzzy'].metrics['v']).toBeLessThan(0.5); expect(R['ch13-fuzzy'].ok).toBe(true);
    expect(R['ch16-resilience'].metrics['byzantine agreement']).toBe(true); expect(R['ch16-resilience'].metrics['dwell time']).not.toBe('none'); expect(R['ch16-three'].sections[0].lines[1]).toMatch(/tied evidence/);
  }, 120000);
  it('DSL errors are reported with line numbers; map blocks keep rows that start with #', () => {
    expect(() => parseConsensus('consensus x\ngraph ring n=4\nx0 1 2 3')).toThrow(/x0 has 3 values/);
    expect(() => parseConsensus('consensus x\nfoo bar')).toThrow(/line 2: unknown consensus line/);
    expect(analyse('gridmapf', 'gridmapf x\nmap\n  ...\nend\nagent a start=0,0 goal=0,5').error).toMatch(/outside the map/);
    const g = parseGridMapf('gridmapf hash rows\nmap\n  #..#\n  ....\nend\nagent a start=1,0 goal=1,3'); expect(g.map).toEqual(['#..#', '....']);
    expect(() => parseWarehouse('warehouse w\nrobots r1 homes=Z9')).toThrow(/home station Z9/);
    expect(() => parseGame('game g\nplayers A B')).toThrow(/payoff lines/);
    expect(analyse('swarm', 'swarm s\nmodel boids\nmodel pso').error).toMatch(/one model/);
  });
});

const stationWith = (n: number, spread = 2000) => { const st = new Station('fleet'); const robots: MobileRobot[] = []; for (let i = 0; i < n; i++) { const r = st.addChild(new MobileRobot(`r${i + 1}`)); r.setPose2D((i % 3) * spread, Math.floor(i / 3) * spread, 0); robots.push(r); } return { st, robots }; };
const run = (rt: FleetRuntime, seconds: number, dt = 0.05) => { for (let t = 0; t < seconds && !rt.done; t += dt) rt.tick(dt); };
describe('group-control module — fleet runtime on station robots', () => {
  it('formation, rendezvous and coverage drive the robots to the predicted configuration', () => {
    const { st, robots } = stationWith(6); const rt = new FleetRuntime({ kind: 'consensus', source: byId('pr1-formation').source.replace('fail 4 at=10', ''), robots, station: st }); run(rt, 60); expect(rt.done).toBe(true); expect(rt.status()).toMatch(/formation error 0\.00\d\d m/);
    const xs = robots.map((r) => r.state.x / 1000), ys = robots.map((r) => r.state.y / 1000); const cx = xs.reduce((a, b) => a + b) / 6, cy = ys.reduce((a, b) => a + b) / 6; for (let i = 0; i < 6; i++) expect(Math.hypot(xs[i] - cx, ys[i] - cy)).toBeCloseTo(0.3, 2);
    const g = stationWith(4, 1000); const rz = new FleetRuntime({ kind: 'consensus', source: 'consensus gather\nrobots 4\ngraph complete n=4', robots: g.robots, station: g.st }); run(rz, 60); expect(rz.done).toBe(true); const spread = Math.max(...g.robots.map((r) => Math.hypot(r.state.x - g.robots[0].state.x, r.state.y - g.robots[0].state.y))); expect(spread).toBeLessThan(5);
    const c = stationWith(6, 500); const rc = new FleetRuntime({ kind: 'coverage', source: byId('pr5-lloyd').source, robots: c.robots, station: c.st }); run(rc, 120); expect(rc.status()).toMatch(/H = /); const mx = c.robots.reduce((s, r) => s + r.state.x, 0) / 6000, my = c.robots.reduce((s, r) => s + r.state.y, 0) / 6000; expect(Math.hypot(mx - 3, my - 1)).toBeLessThan(1);
  });
  it('safety crossing keeps the distance; the ADG plan is executed to the goals; boids run continuously', () => {
    const s = stationWith(2, 1000); s.robots[1].setPose2D(1000, 50, 0); const rs = new FleetRuntime({ kind: 'safety', source: byId('pr6-crossing').source, robots: s.robots, station: s.st }); let dmin = Infinity; for (let t = 0; t < 40 && !rs.done; t += 0.02) { rs.tick(0.02); dmin = Math.min(dmin, Math.hypot(s.robots[0].state.x - s.robots[1].state.x, s.robots[0].state.y - s.robots[1].state.y) / 1000); } expect(rs.done).toBe(true); expect(dmin).toBeGreaterThanOrEqual(0.195);
    const g = stationWith(4); const rg = new FleetRuntime({ kind: 'gridmapf', source: byId('pr4-warehouse').source, robots: g.robots, station: g.st }); run(rg, 200, 0.1); expect(rg.done).toBe(true); expect(rg.status()).toMatch(/4\/4 at their goals/); expect(g.robots[0].state.x / 1000).toBeCloseTo(9.5, 1); expect(g.robots[0].state.y / 1000).toBeCloseTo(-6.5, 1);
    const b = stationWith(9, 300); const rb = new FleetRuntime({ kind: 'swarm', source: 'swarm nine\nmodel boids n=9 r_sep=0.15 r_view=1', robots: b.robots, station: b.st }); run(rb, 30); expect(rb.done).toBe(false); expect(rb.status()).toMatch(/polarization [01]\.\d+ · min distance \d/);
    expect(() => new FleetRuntime({ kind: 'game', source: TEMPLATES.game, robots: b.robots, station: b.st })).toThrow(/analysed, not executed/); expect(RUNNABLE_KINDS).toContain('warehouse');
  });
  it('the warehouse scene is built from the model and the fleet runs on it', () => {
    const st = new Station('wh'); const d = parseWarehouse(byId('hw-baseline').source); const scene = buildWarehouseScene(st, d.cfg, d.stationKinds); expect(scene.robots.length).toBe(4); expect(scene.zones.length).toBe(10); expect(scene.robots[0].state.x).toBeCloseTo(1000, 3); expect(scene.robots[0].state.y).toBeCloseTo(-4500, 3);
    const rt = new FleetRuntime({ kind: 'warehouse', source: byId('hw-baseline').source.replace('duration 600', 'duration 120'), robots: scene.robots, station: st }); run(rt, 130, 0.1); expect(rt.done).toBe(true); expect(rt.status()).toMatch(/delivered/); expect(rt.log.some((l) => /commits to/.test(l))).toBe(true); expect(scene.robots.some((r) => Math.hypot(r.state.x - 1000, r.state.y + 4500) > 100)).toBe(true);
    const empty = new Station('e'); const rt2 = new FleetRuntime({ kind: 'warehouse', source: byId('hw-baseline').source, robots: [], station: empty, createRobots: (name, x, y) => { const r = empty.addChild(new MobileRobot(name)); r.setPose2D(x, y, 0); return r; } }); expect(rt2.robots.length).toBe(4);
  }, 60000);
});
