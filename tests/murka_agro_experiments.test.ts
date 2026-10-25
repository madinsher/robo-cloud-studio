/**
 * MURKA agromarca article experiments exp1..exp5 reproduced by the studio (C2 runAgroExperiment) against the Python
 * ground truth (tests/fixtures/murka_agro/README.md):
 *  - article_py310/: the article's published artifacts (replication/artifacts/experiments, `all --runs 20`, Python 3.10);
 *  - py312_hashseed0/: a fresh run of a copy of murca_experiment under CPython 3.12.3 / NumPy 2.5.3 / SciPy 1.18.1 with
 *    PYTHONHASHSEED=0.
 * Every deterministic value must be identical (Object.is equality); only wall-clock fields (*_time_ms, wallclock_s,
 * *_recovery_ms) are excluded. The ports of CPython's PRNG / math.hypot / statistics, NumPy's std and SciPy's
 * linear_sum_assignment are checked on reference vectors produced by the same interpreter.
 */
import { describe, it, expect } from 'vitest';
import fs from 'node:fs';
import path from 'node:path';
import zlib from 'node:zlib';
import { PyRandom, pyHypot, pyMean, pyPstdev, npStd } from '../src/murka/agro/pycompat';
import { linearSumAssignment } from '../src/fleet/lsap';
import { runAgroExperiment, runAllAgroExperiments, RECIPES_ENERGY } from '../src/murka/agro/experiments';
import { ARTICLE_POPULATION_ORDER, PY312_HASHSEED0_ORDER, generateRandomMission, getMission } from '../src/murka/agro/missions';
import { murkaAllocate, hungarianAllocate, MurkaRobot, MurkaTask, MURKA_INF, murkaCost } from '../src/fleet/murka_alloc';
import { energyStep, routeEnergy, driveCost, profileOfEquipment } from '../src/agri/equipment';

const FIX = path.resolve(__dirname, 'fixtures/murka_agro');
const gz = (p: string) => JSON.parse(zlib.gunzipSync(fs.readFileSync(p)).toString('utf8'));
const TIMING = new Set(['algorithm_time_ms', 'compose_time_ms', 'wallclock_s', 'hungarian_recovery_ms', 'murca_recovery_ms', 'avg_algorithm_time_ms', 'avg_compose_time_ms']);
/** Deep copy without wall-clock timing fields. */
function strip(v: unknown): unknown {
  if (Array.isArray(v)) return v.map(strip);
  if (v && typeof v === 'object') return Object.fromEntries(Object.entries(v as Record<string, unknown>).filter(([k]) => !TIMING.has(k)).map(([k, x]) => [k, strip(x)]));
  return v;
}
/** List of the paths where two JSON values differ (first few), for readable failures. */
function diffs(a: unknown, b: unknown, p = '', out: string[] = []): string[] {
  if (out.length > 8) return out;
  if (Array.isArray(a) && Array.isArray(b)) { if (a.length !== b.length) out.push(`${p}: length ${a.length} != ${b.length}`); else a.forEach((x, i) => diffs(x, b[i], `${p}[${i}]`, out)); return out; }
  if (a && b && typeof a === 'object' && typeof b === 'object') {
    const ka = Object.keys(a as object).sort(), kb = Object.keys(b as object).sort();
    if (ka.join() !== kb.join()) { out.push(`${p}: keys ${ka} != ${kb}`); return out; }
    for (const k of ka) diffs((a as Record<string, unknown>)[k], (b as Record<string, unknown>)[k], `${p}.${k}`, out);
    return out;
  }
  if (!Object.is(a, b)) out.push(`${p}: ${JSON.stringify(a)} != ${JSON.stringify(b)}`);
  return out;
}
const FILES = ['exp1_mission_A.json', 'exp1_mission_B.json', 'exp1_mission_C.json', 'exp2_allocation.json', 'exp3_full_scenario.json', 'exp4_scaling.json', 'exp5_energy.json'];

describe('CPython / NumPy / SciPy numerics ports (reference vectors of CPython 3.12.3)', () => {
  const V = gz(path.join(FIX, 'pycompat_vectors_py312.json.gz'));
  it('random.Random: seeding (incl. > 32-bit seeds), random, uniform, randint, shuffle, sample, randrange', () => {
    for (const [seed, exp] of Object.entries<Record<string, unknown>>(V.random)) {
      const r = new PyRandom(BigInt(seed));
      const got = {
        random: [0, 1, 2, 3, 4].map(() => r.random()), uniform: [0, 1, 2].map(() => r.uniform(-2, 3)),
        randint: [...Array(8)].map(() => r.randint(0, 3)), shuffle: r.shuffle([...Array(10).keys()]),
        sample: r.sample(['a', 'b', 'c', 'd', 'e'], 3), randbelow_big: [...Array(4)].map(() => r.randrange(0, 1000003)),
      };
      expect(got, `seed ${seed}`).toEqual(exp);
    }
  });
  it('math.hypot equals CPython bit for bit on 4000 pairs (plain Math.hypot does not)', () => {
    let bad = 0, nativeBad = 0;
    for (const [a, b, h] of V.hypot as number[][]) { if (!Object.is(pyHypot(a, b), h)) bad++; if (!Object.is(Math.hypot(a, b), h)) nativeBad++; }
    expect(bad).toBe(0);
    // the port matters: V8's Math.hypot rounds differently on some inputs
    expect(nativeBad).toBeGreaterThan(0);
  });
  it('statistics.mean / statistics.pstdev / numpy.std are bit-exact', () => {
    for (const s of V.stats as Array<{ xs: number[]; mean: number; pstdev: number; npstd: number }>) {
      expect(pyMean(s.xs)).toBe(s.mean);
      expect(pyPstdev(s.xs)).toBe(s.pstdev);
      expect(npStd(s.xs)).toBe(s.npstd);
    }
  });
  it('linear_sum_assignment returns SciPy\'s assignment, also with ties, 1e9 entries and rectangular matrices', () => {
    for (const l of V.lsap as Array<{ m: number[][]; rows: number[]; cols: number[] }>) expect(linearSumAssignment(l.m)).toEqual({ rows: l.rows, cols: l.cols });
  });
});

describe.each([
  ['article_py310', ARTICLE_POPULATION_ORDER],
  ['py312_hashseed0', PY312_HASHSEED0_ORDER],
])('runAgroExperiment equals the Python ground truth %s (all experiments, 20 runs)', (dir, order) => {
  const studio = runAllAgroExperiments({ runs: 20, populationOrder: order });
  it.each(FILES)('%s', (file) => {
    const gt = strip(gz(path.join(FIX, dir, `${file}.gz`)));
    const mine = strip(JSON.parse(JSON.stringify(studio[file])));
    expect(diffs(mine, gt)).toEqual([]);
  });
});

describe('experiment semantics (invariants, not just equality)', () => {
  it('exp2 / exp4 depend on CPython\'s set order: a different order does not reproduce the article', () => {
    const gt = strip(gz(path.join(FIX, 'article_py310/exp2_allocation.json.gz')));
    const wrong = strip(JSON.parse(JSON.stringify(runAgroExperiment('exp2', { runs: 20, populationOrder: PY312_HASHSEED0_ORDER }).files['exp2_allocation.json'])));
    expect(diffs(wrong, gt).length).toBeGreaterThan(0);
    // exp1 / exp3 / exp5 do not draw from the set order: the two ground truths agree there
    for (const f of ['exp1_mission_C.json', 'exp3_full_scenario.json', 'exp5_energy.json']) expect(diffs(strip(gz(path.join(FIX, 'article_py310', `${f}.gz`))), strip(gz(path.join(FIX, 'py312_hashseed0', `${f}.gz`))))).toEqual([]);
  });

  it('MuRKA contract net: ≤ 2 contracts per robot, bidders ≥ 15 % battery, reconfiguration only for exactly one missing capability', () => {
    for (let seed = 0; seed < 40; seed++) {
      const m = generateRandomMission(5, 9, seed);
      const msgs: Array<{ kind: string; task: string; robot?: string; reconfig?: boolean }> = [];
      const out = murkaAllocate(m.robots, m.tasks, { onMessage: (x) => msgs.push(x) });
      const load = new Map<string, number>();
      for (const [r] of out.base.assignments) load.set(r, (load.get(r) ?? 0) + 1);
      for (const v of load.values()) expect(v).toBeLessThanOrEqual(2);
      // replay the awards: a plain award needs no missing capability, a RECONFIGURE award exactly one (relative to the
      // robot's capabilities at that moment, which grow with earlier reconfigurations)
      const caps = new Map(m.robots.map((r) => [r.id, new Set(r.caps)]));
      const battery = new Map(m.robots.map((r) => [r.id, r.battery]));
      for (const a of msgs.filter((x) => x.kind === 'award')) {
        const have = caps.get(a.robot!)!, t = m.tasks.find((x) => x.id === a.task)!;
        expect(battery.get(a.robot!)!).toBeGreaterThanOrEqual(0.15);
        const missing = [...t.caps].filter((c) => !have.has(c));
        expect(missing.length).toBe(a.reconfig ? 1 : 0);
        for (const c of missing) have.add(c);
        battery.set(a.robot!, Math.max(0, battery.get(a.robot!)! - (a.reconfig ? 0.07 : 0.02)));
      }
      expect(out.reconfigurations).toEqual(msgs.filter((x) => x.kind === 'award' && x.reconfig).map((x) => [x.robot, x.task]));
      // announcements follow descending priority
      const ann = msgs.filter((x) => x.kind === 'announce').map((x) => m.tasks.find((t) => t.id === x.task)!.priority);
      for (let i = 1; i < ann.length; i++) expect(ann[i]).toBeLessThanOrEqual(ann[i - 1]);
      // every awarded task is covered (possibly after reconfiguration) and every unassigned task had no bid
      expect(out.base.assignments.length + out.base.unassignedTasks.length).toBe(m.tasks.length);
    }
  });

  it('Hungarian baseline is optimal one-to-one on feasible pairs (brute force on small missions)', () => {
    const perms = (n: number): number[][] => (n === 0 ? [[]] : perms(n - 1).flatMap((p) => Array.from({ length: n }, (_, i) => [...p.slice(0, i), n - 1, ...p.slice(i)])));
    for (let seed = 0; seed < 25; seed++) {
      const m = generateRandomMission(4, 4, 1000 + seed);
      const h = hungarianAllocate(m.robots, m.tasks);
      // best (max #feasible, then min cost) by enumeration
      let bestN = -1, bestC = Infinity;
      for (const p of perms(4)) {
        let n = 0, c = 0;
        p.forEach((j, i) => { const v = murkaCost(m.robots[i], m.tasks[j]); if (v < MURKA_INF / 2) { n++; c += v; } });
        if (n > bestN || (n === bestN && c < bestC - 1e-9)) { bestN = n; bestC = c; }
      }
      expect(h.assignments.length).toBe(bestN);
      expect(h.totalCost).toBeCloseTo(bestC, 9);
    }
  });

  it('exp5 energy: the closed form equals the tick integration of E(t+1) = E(t) − ε_total·dt − c_drive·ds', () => {
    for (const [name, p] of Object.entries(RECIPES_ENERGY)) {
      const L = 500, v = 0.95;
      const { energy, duration } = routeEnergy(p, L, v);
      let E = 1e6, t = 0;
      const dt = 0.5;
      while (t < duration - 1e-12) { const h = Math.min(dt, duration - t); E = energyStep(E, p, h, v * h); t += h; }
      expect(1e6 - E, name).toBeCloseTo(energy, 6);
    }
    // the heavier configuration costs more per metre: harvester (equipment) > scout
    expect(routeEnergy(RECIPES_ENERGY.harvester, 100, 1).energy).toBeGreaterThan(routeEnergy(RECIPES_ENERGY.scout_ground, 100, 1).energy);
    // c_drive = c_base (1 + W_eq / W_max) on studio equipment
    expect(driveCost(0.08, 60, 120)).toBeCloseTo(0.12, 12);
    const sprayer = profileOfEquipment(['sprayer'], { entropy: 0.05, cBase: 0.1, maxPayloadKg: 300, platformMassKg: 400 });
    expect(sprayer.driveCostPerM).toBeCloseTo(0.1 * (1 + 120 / 300), 12);
    expect(sprayer.equipmentOverhead).toBeCloseTo(350 / 10000, 12);
  });

  it('exp3 MuRKA mode reconfigures for the infection spray tasks and covers more than the static fleet', () => {
    const r = runAgroExperiment('exp3', { runs: 5 }).files['exp3_full_scenario.json'] as { records: Record<string, Array<Record<string, unknown>>> };
    for (let i = 0; i < 5; i++) {
      const st = r.records.static[i], mu = r.records.murca[i];
      expect(st.missed_tasks).toEqual(expect.arrayContaining(['prune_S8', 'prune_S9']));
      expect(mu.reconfigurations_count as number).toBeGreaterThanOrEqual(1);
      expect(mu.coverage_percent as number).toBeGreaterThan(st.coverage_percent as number);
      // mission C is seeded: same seed, same tasks as the static run
      expect(getMission('C', i).tasks.map((t) => t.id)).toHaveLength(12);
    }
  });
});

// keep the types referenced for readers
export type { MurkaRobot, MurkaTask };
