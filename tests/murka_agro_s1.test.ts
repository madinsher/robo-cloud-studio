/**
 * Benchmark S1 «scouting → harvest → crate logistics» (agromarca article/BENCHMARK_SCENARIO.md) on the imported
 * orchard_demo.wbt (C2 runS1Benchmark). No Python model of S1 exists in murca_experiment (§11 is a plan), so the run is
 * checked against the scenario's analytical expectations and physical invariants:
 *  - zero robot–trunk / robot–crown / robot–static / robot–robot contacts (3-D colliders, measured crown profile);
 *  - Ackermann paths (curvature ≤ 1/R_min, no turning in place), speeds of §2;
 *  - survey time = takeoff + boustrophedon length / 5 m/s (closed form, 1e-6 s);
 *  - boxes = the §4.2 fill rule over the seeded yields, kg delivered = kg on the trees (S1-A/B), swap 45 s, unload 30 s;
 *  - WaitSwap of the event log = time in the WaitSwap state of the logical FSM trace; ≥ 45 s per box;
 *  - makespan ≥ the single-logist lower bound Σ (45 + 30 + 2 d_min / v_L) after the first box_full;
 *  - D1: the Hungarian initial assignment is the brute-force optimum of cost (1); contracts ≤ 2 per harvester;
 *  - level B: sector 2 yields 2× the estimate → more boxes; level C: harvester 1 fails at 50 % of its 2nd row, its crate
 *    is lost, the rest of the row is harvested from the other side; contract net re-awards at once, Hungarian later.
 */
import { describe, it, expect, beforeAll } from 'vitest';
import { buildMurkaOrchardStation, OrchardStationResult } from '../src/murka/agro';
import { runS1Benchmark, S1Result, s1World, S1_DEFAULTS } from '../src/murka/agro/s1';
import { S1_FSMS } from '../src/murka/agro/s1_fsm';
import { parseFsm, checkFsm } from '../src/ctl/fsm';
import { murkaCost } from '../src/fleet/murka_alloc';
import { murkaKpiFromS1 } from '../src/murka/agro/kpi';
import { HAS_MURKA, MURKA } from './webots_helpers';

const P = S1_DEFAULTS;
/** §4.2 fill rule: box_full when the next tree does not fit, partial box at the end of every row. */
const boxesOf = (ys: number[]) => { let n = 0, fill = 0; ys.forEach((y, i) => { fill += y; const next = ys[i + 1]; if (next !== undefined ? fill + next > P.boxKg : fill > 0) { n++; fill = 0; } }); return n; };

describe('S1 automata are valid studio fsm documents', () => {
  it.each(Object.entries(S1_FSMS))('%s FSM parses and checks without errors', (_k, src) => {
    const doc = parseFsm(src);
    expect(checkFsm(doc).filter((i) => i.level === 'error')).toEqual([]);
  });
  it('the harvester automaton has the states of §4.2 (Wait, Start, GotoWaypoints{FollowWaypoints, ReachedGoal}, AlignRow, HarvestLoop{PickTree, WaitSwap, AdvanceTree}, RowDone, ReturnHome, End)', () => {
    const doc = parseFsm(S1_FSMS.harvester);
    const names = (m: typeof doc.root): string[] => m.states.flatMap((s) => [s.name, ...(s.machine ? names(s.machine) : [])]);
    expect(names(doc.root)).toEqual(expect.arrayContaining(['Wait', 'Start', 'GotoWaypoints', 'FollowWaypoints', 'ReachedGoal', 'AlignRow', 'HarvestLoop', 'PickTree', 'WaitSwap', 'AdvanceTree', 'RowDone', 'ReturnHome', 'End']));
  });
});

describe.skipIf(!HAS_MURKA)(`S1 on the imported orchard_demo.wbt (needs ${MURKA})`, () => {
  let o: OrchardStationResult;
  const runs: Record<string, S1Result> = {};
  const run = (key: string, opts: Parameters<typeof runS1Benchmark>[1]) => (runs[key] ??= runS1Benchmark(o.station, { assets: o.assets, ...opts }));
  beforeAll(async () => { o = await buildMurkaOrchardStation({ world: 'orchard_demo', root: MURKA }); }, 120000);

  const ev = (r: S1Result, type: string, robot?: string) => r.events.filter((e) => e.type === type && (!robot || e.robot === robot));

  it('world: 8 rows of 3 trees, lanes 1.5 m from the row lines, the CP-side lane of sector 3 moved to the inner aisle', () => {
    const w = s1World(o.station);
    expect(w.rows.map((r) => [r.id, r.sector, r.laneY, r.heading])).toEqual([
      ['r1', 'sector_1', 3500, 180], ['r2', 'sector_1', 9500, 180], ['r3', 'sector_2', 3500, 0], ['r4', 'sector_2', 9500, 0],
      ['r5', 'sector_3', -4500, 180], ['r6', 'sector_3', -1500, 180], ['r7', 'sector_4', -7500, 0], ['r8', 'sector_4', -1500, 0],
    ]);
    for (const r of w.rows) for (const [i, s] of r.stops.entries()) { expect(s.x).toBe(r.trees[i].x); expect(Math.abs(s.y - r.trees[i].y)).toBe(1500); }
    expect(w.flat.map((f) => f.name)).toContain('collection_point_1');
  });

  it('S1-A Hungarian: completes, zero contacts, Ackermann paths, exact durations, metrics = analytical expectations', () => {
    const r = run('A-h', { level: 'A', controller: 'hungarian', seed: 0, contactStep: 0.25 });
    const m = r.metrics;
    expect(m.outcome).toBe('done');
    expect(m.missionEnd).toBeLessThan(P.tMax);
    // contacts (3-D colliders, crown profile, collider-less obstacles), sampled every 0.25 s while robots move
    expect(m.contacts.samples).toBeGreaterThan(3000);
    expect({ trunk: m.contacts.trunk, crown: m.contacts.crown, static: m.contacts.static, robot: m.contacts.robot, flat: m.contacts.flat }).toEqual({ trunk: 0, crown: 0, static: 0, robot: 0, flat: 0 });
    for (const v of Object.values(m.contacts.minCrownClearance)) expect(v).toBeGreaterThan(0);
    // kinematics: UTP-G turning radius 2.19 m / tan(0.3 rad), no turn in place, speeds of §2, no teleport
    const Rmin = 2190 / Math.tan(0.3);
    for (const n of ['UTP-G-01', 'UTP-G-03', 'UTP-G-04']) expect(m.kinematics.minTurnRadius[n]).toBeGreaterThan(Rmin - 5);
    expect(m.kinematics.inPlaceRotations).toBe(0);
    expect(m.kinematics.maxSpeed).toEqual({ 'UTP-A-01': 5000, 'UTP-G-01': 600, 'UTP-G-03': 600, 'UTP-G-04': 1000 });
    expect(m.kinematics.maxStep['UTP-G-04']).toBeLessThanOrEqual(1000 * 0.1 + 1e-6);
    expect(m.kinematics.maxStep['UTP-G-01']).toBeLessThanOrEqual(600 * 0.1 + 1e-6);
    // survey: takeoff 1.2 → 5 m at 1 m/s, then home → first leg start → legs → turns at 5 m/s (closed form)
    const w = s1World(o.station);
    const tx = w.trees.map((t) => t.poseAbs()[12]), ty = w.trees.map((t) => t.poseAbs()[13]);
    const cr = w.o.profile.crownRadius;
    const x0 = Math.min(...tx) - cr, x1 = Math.max(...tx) + cr, y0 = Math.min(...ty) - cr, y1 = Math.max(...ty) + cr;
    const n = Math.ceil((y1 - y0) / P.swath);
    const ys = Array.from({ length: n }, (_, k) => y0 + P.swath / 2 + (k * (y1 - y0 - P.swath)) / (n - 1));
    const home = o.robots['UTP-A-01'];
    const z0 = home.state.z ?? home.pose()[14];
    let len = Math.hypot(x0 - home.state.x, ys[0] - home.state.y);
    ys.forEach((y, k) => { len += x1 - x0; if (k + 1 < n) len += ys[k + 1] - y; });
    expect(m.surveyDone).toBeCloseTo((P.altitude - z0) / P.climb + len / P.vD, 3);
    // yields, boxes, kg
    const kgTotal = r.rows.reduce((a, row) => a + row.yields.reduce((b, y) => b + y, 0), 0);
    expect(m.kgTotal).toBeCloseTo(kgTotal, 3);
    expect(m.completeness).toBeCloseTo(1, 12);
    const expected = r.rows.reduce((a, row) => a + boxesOf(row.yields), 0);
    expect(m.boxesExpected).toBe(expected);
    expect(m.boxesDelivered).toBe(expected);
    expect(ev(r, 'box_full')).toHaveLength(expected);
    // every box: box_full ≤ swap_request ≤ arrival + 45 s = swap_done ≤ CP arrival + 30 s = box_delivered
    for (const bf of ev(r, 'box_full')) {
      const id = bf.data!.box;
      const sd = r.events.filter((e) => e.type === 'swap_done' && e.data?.box === id);
      const bd = r.events.filter((e) => e.type === 'box_delivered' && e.data?.box === id);
      expect(sd).toHaveLength(1); expect(bd).toHaveLength(1);
      const sr = r.events.find((e) => e.type === 'swap_request' && e.data?.box === id)!;
      expect(sr.t).toBeGreaterThanOrEqual(bf.t);
      const arrive = r.events.filter((e) => e.type === 'o_rob_Arrived' && e.robot === 'UTP-G-04' && e.t <= sd[0].t + 1e-9).pop()!;
      expect(sd[0].t - arrive.t).toBeCloseTo(P.swapS, 9);
      const atCp = r.events.filter((e) => e.type === 'o_rob_Arrived' && e.robot === 'UTP-G-04' && e.data?.at === 'CP' && e.t <= bd[0].t + 1e-9).pop()!;
      expect(bd[0].t - atCp.t).toBeCloseTo(P.unloadS, 9);
      expect(Number(bd[0].data!.kg)).toBeCloseTo(Number(bf.data!.kg), 3);
    }
    // pick time = 6 s/kg of every tree picked; every tree of every row picked exactly once
    const picked = ev(r, 'tree_picked');
    expect(picked).toHaveLength(24);
    for (const h of ['UTP-G-01', 'UTP-G-03']) expect(m.pickTime[h]).toBeCloseTo(picked.filter((e) => e.robot === h).reduce((a, e) => a + Number(e.data!.kg), 0) * P.pickSPerKg, 2);
    expect(new Set(picked.map((e) => `${e.data!.row}:${e.data!.tree}`)).size).toBe(24);
    // WaitSwap: event log = logical FSM trace; ≥ 45 s per box
    for (const h of ['UTP-G-01', 'UTP-G-03']) {
      const st = r.states[h];
      let inWait = 0;
      st.forEach((s, i) => { if (s.state === 'HarvestLoop/WaitSwap') inWait += st[i + 1].t - s.t; });
      expect(m.waitSwap[h]).toBeCloseTo(inWait, 3);
      expect(m.waitSwap[h]).toBeGreaterThanOrEqual(ev(r, 'box_full', h).length * P.swapS - 1e-6);
    }
    // single-logist lower bound on the makespan after the first box_full
    const cp = w.unloadPose;
    const dmin = Math.min(...w.rows.flatMap((row) => row.stops.map((s) => Math.hypot(s.x - cp.x, s.y - cp.y))));
    const firstFull = Math.min(...ev(r, 'box_full').map((e) => e.t));
    const lb = firstFull + expected * (P.swapS + P.unloadS + (2 * dmin) / P.vL);
    expect(m.makespan).toBeGreaterThanOrEqual(lb);
    // energy model of §2: 0.2 u/m + 0.5 u/kg (harvesters)
    for (const h of ['UTP-G-01', 'UTP-G-03']) expect(m.energy[h]).toBeCloseTo(0.2 * m.distance[h] + 0.5 * picked.filter((e) => e.robot === h).reduce((a, e) => a + Number(e.data!.kg), 0), 2);
    // all robots back home, mission ends with o_rob_WD of everyone
    expect(ev(r, 'o_rob_WD').map((e) => e.robot).sort()).toEqual(['UTP-A-01', 'UTP-G-01', 'UTP-G-03', 'UTP-G-04']);
    for (const n of ['UTP-G-01', 'UTP-G-03', 'UTP-G-04']) { const p = r.timeline.poseAt(n, r.timeline.end), h = o.robots[n]; expect(Math.hypot(p.x - h.state.x, p.y - h.state.y)).toBeLessThan(1); }
    // the event log is a JSONL trace
    expect(r.eventsJsonl().trim().split('\n').map((l) => JSON.parse(l).type)).toContain('survey_done');
  }, 300000);

  it('D1 Hungarian: the first assignment is the brute-force optimum of the article\'s cost (1) from the start poses', () => {
    const r = run('A-h', { level: 'A', controller: 'hungarian', seed: 0, contactStep: 0.25 });
    const w = s1World(o.station);
    const H = ['UTP-G-01', 'UTP-G-03'];
    const cost = (h: string, row: number) => murkaCost({ id: h, x: o.robots[h].state.x / 1000, y: o.robots[h].state.y / 1000, battery: 1, caps: new Set(['harvest']) }, { id: w.rows[row].id, x: w.rows[row].entrance.x / 1000, y: w.rows[row].entrance.y / 1000, caps: new Set(['harvest']), priority: 1 });
    let best = Infinity, pair: [number, number] = [0, 0];
    for (let a = 0; a < 8; a++) for (let b = 0; b < 8; b++) if (a !== b && cost(H[0], a) + cost(H[1], b) < best) { best = cost(H[0], a) + cost(H[1], b); pair = [a, b]; }
    const first = ev(r, 'row_assigned').slice(0, 2);
    expect(Object.fromEntries(first.map((e) => [e.robot, e.data!.row]))).toEqual({ [H[0]]: w.rows[pair[0]].id, [H[1]]: w.rows[pair[1]].id });
    // assignments come after the survey map (PLAN), never before
    expect(first[0].t).toBeCloseTo(r.metrics.surveyDone, 9);
  }, 300000);

  it('S1-A contract net: same kg / boxes, ≤ 2 contracts per harvester, zero contacts; differs from Hungarian only in decisions', () => {
    const h = run('A-h', { level: 'A', controller: 'hungarian', seed: 0, contactStep: 0.25 });
    const c = run('A-c', { level: 'A', controller: 'contract', seed: 0, contactStep: 0.25 });
    expect(c.metrics.outcome).toBe('done');
    expect(c.metrics.kgDelivered).toBeCloseTo(h.metrics.kgDelivered, 6);
    expect(c.metrics.boxesDelivered).toBe(h.metrics.boxesDelivered);
    expect([c.metrics.contacts.trunk, c.metrics.contacts.crown, c.metrics.contacts.static, c.metrics.contacts.robot, c.metrics.contacts.flat]).toEqual([0, 0, 0, 0, 0]);
    for (const e of ev(c, 'row_assigned')) expect((e.data!.queue as string[]).length).toBeLessThanOrEqual(2);
    expect(c.metrics.comm.announce).toBeGreaterThan(0);
    expect(c.metrics.comm.messages).toBe(c.metrics.comm.announce + c.metrics.comm.bids + c.metrics.comm.awards);
    expect(h.metrics.comm.matrixCells).toBeGreaterThan(0);
    // all rows harvested once in both
    expect([...Object.values(c.metrics.rowsByHarvester).flat()].sort()).toEqual(['r1', 'r2', 'r3', 'r4', 'r5', 'r6', 'r7', 'r8']);
    expect([...Object.values(h.metrics.rowsByHarvester).flat()].sort()).toEqual(['r1', 'r2', 'r3', 'r4', 'r5', 'r6', 'r7', 'r8']);
    // MURKA KPI exporter (exp3 record schema)
    const k = murkaKpiFromS1(c);
    expect([k.mode, k.coverage_percent, k.missed_tasks, k.completed_tasks.length, k.duration_s]).toEqual(['contract', 100, [], 8, c.metrics.makespan]);
    expect(k.total_energy).toBeCloseTo(0.2 * k.total_distance, 9);
  }, 300000);

  it('S1-A hybrid controller completes with the same harvest', () => {
    const y = run('A-y', { level: 'A', controller: 'hybrid', seed: 0, contacts: false });
    expect(y.metrics.outcome).toBe('done');
    expect(y.metrics.boxesDelivered).toBe(y.metrics.boxesExpected);
    expect(y.metrics.completeness).toBeCloseTo(1, 12);
  }, 300000);

  it('S1-B: sector 2 yields twice the survey estimate → more boxes than S1-A, all delivered', () => {
    const a = run('A-h', { level: 'A', controller: 'hungarian', seed: 0, contactStep: 0.25 });
    const b = run('B-h', { level: 'B', controller: 'hungarian', seed: 0, contactStep: 0.5 });
    expect(b.metrics.outcome).toBe('done');
    const s2 = b.rows.filter((row) => row.sector === 'sector_2');
    for (const row of s2) expect(row.yields.reduce((x, y) => x + y, 0)).toBeCloseTo(2 * row.estimate, 3);
    expect(b.metrics.boxesExpected).toBe(b.rows.reduce((x, row) => x + boxesOf(row.yields), 0));
    expect(b.metrics.boxesDelivered).toBe(b.metrics.boxesExpected);
    expect(b.metrics.boxesDelivered).toBeGreaterThan(a.metrics.boxesDelivered);
    expect(b.metrics.completeness).toBeCloseTo(1, 12);
    expect(b.metrics.makespan).toBeGreaterThan(a.metrics.makespan);
    expect([b.metrics.contacts.trunk, b.metrics.contacts.crown, b.metrics.contacts.static, b.metrics.contacts.robot, b.metrics.contacts.flat]).toEqual([0, 0, 0, 0, 0]);
  }, 300000);

  it('S1-C: harvester 1 fails at 50 % of its 2nd row; its crate is lost; the rest is re-assigned (contracts at once, Hungarian at the next row_done)', () => {
    const c = run('C-c', { level: 'C', controller: 'contract', seed: 0, contactStep: 0.5 });
    const h = run('C-h', { level: 'C', controller: 'hungarian', seed: 0, contacts: false });
    for (const r of [c, h]) {
      const m = r.metrics;
      expect(m.outcome).toBe('done');
      const f = ev(r, 'robot_failed');
      expect(f).toHaveLength(1);
      expect(f[0].robot).toBe('UTP-G-01');
      // 50 % of the second row: the middle tree is half picked
      expect(ev(r, 'row_done', 'UTP-G-01')).toHaveLength(1);
      expect(f[0].data!.tree).toBe(1);
      expect(m.reaction!.lostKg).toBeCloseTo(Number(f[0].data!.fill_kg), 3);
      expect(m.kgDelivered).toBeCloseTo(m.kgTotal - m.reaction!.lostKg, 6);
      expect(m.completeness).toBeLessThan(1);
      expect(m.reaction!.recoveredFraction).toBe(1);
      // the failed platform stays where it failed; nobody touches it
      expect(m.contacts.robot).toBe(0);
    }
    // reaction (§8): the failed robot's work is re-awarded as soon as the survivor can take it — contracts: at once when
    // it holds < 2 contracts, else at its next row_done; Hungarian: when it is free (its row_done)
    for (const [r, kind] of [[c, 'contract'], [h, 'hungarian']] as const) {
      const tf = r.metrics.reaction!.failedAt;
      let queue: string[] = [];
      for (const e of r.events) { if (e.t > tf + 1e-9) break; if (e.robot !== 'UTP-G-03') continue; if (e.type === 'row_assigned') queue = [...(e.data!.queue as string[])]; if (e.type === 'row_done') queue.shift(); }
      const nextDone = ev(r, 'row_done', 'UTP-G-03').map((e) => e.t).find((t) => t >= tf - 1e-9)!;
      if (kind === 'contract') {
        // the failed robot's contracts are re-announced first: awarded at once when the survivor has a free slot
        expect(r.metrics.reaction!.reassignedAt, kind).toBeCloseTo(queue.length < 2 ? tf : nextDone, 9);
      } else {
        // Hungarian: nothing before the survivor is free; then the global recomputation picks by cost (1)
        const at = r.metrics.reaction!.reassignedAt!;
        expect(at).toBeGreaterThanOrEqual((queue.length === 0 ? tf : nextDone) - 1e-9);
        expect(ev(r, 'row_assigned', 'UTP-G-03').some((e) => Math.abs(e.t - at) < 1e-9 && /b$/.test(String(e.data!.row)) || Math.abs(e.t - at) < 1e-9)).toBe(true);
      }
      expect(r.metrics.reaction!.reactionS!).toBeGreaterThanOrEqual(0);
    }
    expect(c.metrics.reaction!.reactionS!).toBeLessThanOrEqual(h.metrics.reaction!.reactionS!);
    expect([c.metrics.contacts.trunk, c.metrics.contacts.crown, c.metrics.contacts.static, c.metrics.contacts.robot, c.metrics.contacts.flat]).toEqual([0, 0, 0, 0, 0]);
  }, 300000);
});

describe.skipIf(!HAS_MURKA)('S1-D: reconfiguration through the depot (D4)', () => {
  let o: OrchardStationResult;
  beforeAll(async () => { o = await buildMurkaOrchardStation({ world: 'orchard_demo', root: MURKA }); }, 120000);
  it('Hungarian keeps fixed roles (= S1-B); the contract net re-roles the first free harvester into a logist at a depot slot (180 s), zero contacts', () => {
    const b = runS1Benchmark(o.station, { assets: o.assets, level: 'B', controller: 'hungarian', seed: 0, contacts: false });
    const dh = runS1Benchmark(o.station, { assets: o.assets, level: 'D', controller: 'hungarian', seed: 0, contacts: false });
    expect(dh.metrics.reconfigurations).toEqual([]);
    expect(dh.metrics.makespan).toBe(b.metrics.makespan);
    expect(dh.metrics.boxesDelivered).toBe(b.metrics.boxesDelivered);
    const dc = runS1Benchmark(o.station, { assets: o.assets, level: 'D', controller: 'contract', seed: 0, contactStep: 0.5 });
    const m = dc.metrics;
    expect(m.outcome).toBe('done');
    expect(m.reconfigurations).toHaveLength(1);
    const rc = m.reconfigurations[0];
    expect(rc.startAt).toBeGreaterThanOrEqual(rc.decidedAt);
    expect(rc.doneAt - rc.startAt).toBeCloseTo(S1_DEFAULTS.reconfigS, 9);
    // the platform stands on the depot slot during the swap, then runs the Logistics automaton
    const slot = o.station.children.find((c) => c.name === rc.slot)!.poseAbs();
    for (const t of [rc.startAt + 1, (rc.startAt + rc.doneAt) / 2, rc.doneAt - 1]) { const p = dc.timeline.poseAt(rc.robot, t); expect(Math.hypot(p.x - slot[12], p.y - slot[13])).toBeLessThan(1); }
    expect(dc.states[rc.robot].map((s) => s.state)).toEqual(expect.arrayContaining(['Reconfigure', 'role: logistics', 'Idle']));
    expect(ev(dc, 'reconfigured', rc.robot)).toHaveLength(1);
    // the other harvester was still harvesting when the contract was awarded
    const other = ['UTP-G-01', 'UTP-G-03'].find((n) => n !== rc.robot)!;
    expect(Math.max(...ev(dc, 'row_done', other).map((e) => e.t))).toBeGreaterThan(rc.decidedAt);
    expect(m.completeness).toBeCloseTo(1, 12);
    expect([m.contacts.trunk, m.contacts.crown, m.contacts.static, m.contacts.robot, m.contacts.flat]).toEqual([0, 0, 0, 0, 0]);
    function ev(r: S1Result, type: string, robot?: string) { return r.events.filter((e) => e.type === type && (!robot || e.robot === robot)); }
  }, 300000);
});
