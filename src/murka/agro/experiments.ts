/**
 * MURKA agromarca article experiments exp1..exp5 (murca_experiment/article_experiments.py) on the studio's models:
 *
 *  - exp1 morphogenesis: greedy set-cover fleet composition (fleet/murka_alloc.composeMinimalFleet) over presets A/B/C;
 *  - exp2 allocation: Hungarian (SciPy LSAP port) vs MuRKA contract net vs hybrid on random N×M missions + failure
 *    recovery (the most loaded robot fails, its contracts are re-allocated);
 *  - exp3 full scenario: static fleet (one Hungarian round, uncovered capabilities missed) vs MuRKA (scouting phase,
 *    3 infection spray tasks discovered, main phase with reconfiguration);
 *  - exp4 scaling: MuRKA on n × 3n random missions, n ∈ {2, 4, 6, 8, 10};
 *  - exp5 energy: configuration-dependent route energy per recipe (agri/equipment.routeEnergy, E = ε_total·t + c_drive·L)
 *    with the seeded ±10 % speed and ±2 % noise of the Python model.
 *
 * The output objects have the schema of the Python JSON artifacts (`artifacts/experiments/exp*.json`, snake_case keys)
 * so `article_plots.py` can plot studio runs; deterministic values equal the Python ground truth exactly (same seeds,
 * same operation order, CPython-exact PRNG / hypot / statistics); `*_time_ms`, `wallclock_s` are wall-clock timings.
 */
import { PyRandom, pyHypot, pyMean, pyPstdev } from './pycompat';
import {
  AllocationResult, MurkaRobot, MurkaTask, composeMinimalFleet, hungarianAllocate, hybridAllocate, loadImbalance, murkaAllocate,
} from '../../fleet/murka_alloc';
import { EnergyProfile, entropyRate, routeEnergy } from '../../agri/equipment';
import { ARTICLE_POPULATION_ORDER, MISSION_FACTORIES, Mission, OrchardScenario, coveragePercent, generateRandomMission, getMission, orchardScenario } from './missions';

export type AgroExperimentId = 'exp1' | 'exp2' | 'exp3' | 'exp4' | 'exp5';
export interface AgroExperimentOptions {
  /** Runs per point (Python CLI default: exp1 10, exp2 20, exp3 10, exp4 10, exp5 10; the article used 20). */
  runs?: number;
  /** CPython set order of the extra capabilities in generate_random_mission (exp2 / exp4). */
  populationOrder?: readonly string[];
  /** exp1: 'all' | 'A' | 'B' | 'C'. */
  missionPreset?: string;
  /** exp2: 'all' | 'hungarian' | 'murca' | 'hybrid', sweep over {4,5,6}², or a single n×m. */
  algorithm?: string; sweep?: boolean; nRobots?: number; mTasks?: number;
  /** exp3: 'all' | 'static' | 'murca'. */
  mode?: string;
  /** exp4: 'all' | fleet size. */
  fleetSize?: string | number;
  /** exp5: 'all' | recipe; route length (m). */
  recipe?: string; routeLengthM?: number;
}
type Json = Record<string, unknown>;

const DEFAULT_RUNS: Record<AgroExperimentId, number> = { exp1: 10, exp2: 20, exp3: 10, exp4: 10, exp5: 10 };
const now = () => (typeof performance !== 'undefined' ? performance.now() : Date.now());

function algorithmPayload(name: string, res: AllocationResult, reconfigurations: number): Json {
  return {
    algorithm: name,
    assignments: res.assignments.map(([robot_id, task_id]) => ({ robot_id, task_id })),
    total_cost: res.totalCost,
    algorithm_time_ms: res.algorithmTimeMs,
    unassigned_tasks: [...res.unassignedTasks],
    unassigned_robots: [...res.unassignedRobots],
    reconfigurations,
  };
}

const sortedPy = (s: Iterable<string>) => [...s].sort((a, b) => (a < b ? -1 : a > b ? 1 : 0));

// ---------------------------------------------------------------------------------------------------------------
// exp1
// ---------------------------------------------------------------------------------------------------------------

function composeRecord(mission: Mission, seed: number): Json {
  const required = mission.requiredCapabilities;
  const { chosen, covered, ms } = composeMinimalFleet(mission.robots, required, new PyRandom(seed));
  const used = chosen.length, theoreticalMin = Math.max(1, mission.minimumFleetSize);
  return {
    mission_id: mission.missionId, seed, platforms_used: used, minimum_required: mission.minimumFleetSize,
    coverage_percent: coveragePercent(mission, covered), required_capabilities: sortedPy(required), covered_capabilities: sortedPy(covered),
    redundancy: Math.max(0.0, (used - theoreticalMin) / theoreticalMin), compose_time_ms: ms, chosen_robots: chosen.map((r) => r.id),
  };
}

export function runExp1(runs: number, missionPreset = 'all'): Json[] {
  const presets = missionPreset === 'all' ? Object.keys(MISSION_FACTORIES) : [missionPreset];
  return presets.map((preset) => {
    const records = Array.from({ length: runs }, (_, i) => composeRecord(getMission(preset, i), i));
    return {
      experiment_id: 'exp1', mission_preset: preset, runs, records,
      aggregate: {
        avg_platforms_used: pyMean(records.map((r) => r.platforms_used as number)),
        avg_coverage_percent: pyMean(records.map((r) => r.coverage_percent as number)),
        avg_redundancy: pyMean(records.map((r) => r.redundancy as number)),
        avg_compose_time_ms: pyMean(records.map((r) => r.compose_time_ms as number)),
      },
    };
  });
}

// ---------------------------------------------------------------------------------------------------------------
// exp2
// ---------------------------------------------------------------------------------------------------------------

function evaluateAllocation(algorithm: string, robots: MurkaRobot[], tasks: MurkaTask[]): Json {
  let res: AllocationResult, reconfig: number;
  if (algorithm === 'hungarian') { res = hungarianAllocate(robots, tasks); reconfig = 0; }
  else if (algorithm === 'murca') { const o = murkaAllocate(robots, tasks); res = o.base; reconfig = o.reconfigurations.length; }
  else if (algorithm === 'hybrid') { const o = hybridAllocate(robots, tasks); res = o.base; reconfig = o.reconfigurations.length; }
  else throw new Error(`unknown algorithm: ${algorithm}`);
  const payload = algorithmPayload(algorithm, res, reconfig);
  payload.load_std = loadImbalance(robots, res.assignments);
  payload.feasible_fraction = res.assignments.length / Math.max(1, tasks.length);
  return payload;
}

function exp2Combination(n: number, m: number, runs: number, algorithms: string[], order: readonly string[]): Json {
  const records: Json[] = [];
  for (let trial = 0; trial < runs; trial++) {
    const mission = generateRandomMission(n, m, trial * 97 + n * 13 + m, order);
    const per: Json = {};
    for (const a of algorithms) per[a] = evaluateAllocation(a, mission.robots, mission.tasks);
    records.push({ trial, mission_id: mission.missionId, algorithms: per });
  }
  const aggregate: Json = {};
  const field = (a: string, k: string) => records.map((r) => ((r.algorithms as Json)[a] as Json)[k] as number);
  for (const a of algorithms) {
    aggregate[a] = {
      avg_total_cost: pyMean(field(a, 'total_cost')), avg_algorithm_time_ms: pyMean(field(a, 'algorithm_time_ms')),
      avg_load_std: pyMean(field(a, 'load_std')), avg_feasible_fraction: pyMean(field(a, 'feasible_fraction')),
      avg_reconfigurations: pyMean(field(a, 'reconfigurations')),
    };
  }
  return { n_robots: n, m_tasks: m, runs, records, aggregate };
}

function exp2Recovery(n: number, m: number, runs: number, order: readonly string[]): Json[] {
  const records: Json[] = [];
  for (let trial = 0; trial < runs; trial++) {
    const mission = generateRandomMission(n, m, trial * 127 + n, order);
    const out = murkaAllocate(mission.robots, mission.tasks);
    const assigned = new Set(out.base.assignments.map(([, t]) => t));
    const remainingTasks = mission.tasks.filter((t) => !assigned.has(t.id));
    const load = new Map<string, number>();
    for (const [rid] of out.base.assignments) load.set(rid, (load.get(rid) ?? 0) + 1);
    if (!load.size) continue;
    let killed = '', best = -Infinity;
    for (const [rid, l] of load) if (l > best) { best = l; killed = rid; }
    const survivors = mission.robots.filter((r) => r.id !== killed);
    const dropped = [...out.base.assignments.filter(([rid]) => rid === killed).map(([, t]) => t), ...remainingTasks.map((t) => t.id)];
    const droppedSet = new Set(dropped);
    const droppedTasks = mission.tasks.filter((t) => droppedSet.has(t.id));
    let t0 = now();
    const hung = hungarianAllocate(survivors, droppedTasks);
    const hungMs = now() - t0;
    t0 = now();
    const re = murkaAllocate(survivors, droppedTasks);
    const murcaMs = now() - t0;
    records.push({
      trial, killed_robot: killed, dropped_tasks: dropped, hungarian_recovery_ms: hungMs, murca_recovery_ms: murcaMs,
      hungarian_recovered_fraction: hung.assignments.length / Math.max(1, droppedTasks.length),
      murca_recovered_fraction: re.base.assignments.length / Math.max(1, droppedTasks.length),
    });
  }
  return records;
}

export function runExp2(runs: number, o: AgroExperimentOptions = {}): Json {
  const order = o.populationOrder ?? ARTICLE_POPULATION_ORDER;
  const algorithms = !o.algorithm || o.algorithm === 'all' ? ['hungarian', 'murca', 'hybrid'] : [o.algorithm];
  const nR = o.nRobots ?? 5, mT = o.mTasks ?? 5;
  const grid: Array<[number, number]> = (o.sweep ?? true) ? [4, 5, 6].flatMap((n) => [4, 5, 6].map((m) => [n, m] as [number, number])) : [[nR, mT]];
  const combinations = grid.map(([n, m]) => exp2Combination(n, m, runs, algorithms, order));
  const recovery = exp2Recovery(Math.max(nR, 5), Math.max(mT, 5), Math.max(10, Math.floor(runs / 2)), order);
  return { experiment_id: 'exp2', runs_per_combination: runs, algorithms, grid, combinations, recovery };
}

// ---------------------------------------------------------------------------------------------------------------
// exp3
// ---------------------------------------------------------------------------------------------------------------

/** Static fleet (fixed_fleet_baseline.run_static_mission). */
export function runStaticMission(mission: Mission, scenario: OrchardScenario = orchardScenario()): Json {
  const covered = new Set(mission.robots.flatMap((r) => [...r.caps]));
  const reachable: MurkaTask[] = [], missed: string[] = [];
  for (const t of mission.tasks) { if (t.caps.size && [...t.caps].some((c) => !covered.has(c))) missed.push(t.id); else reachable.push(t); }
  const result = hungarianAllocate(mission.robots, reachable);
  const completed = result.assignments.map(([, t]) => t);
  missed.push(...result.unassignedTasks);
  const perRobot: Record<string, number> = Object.fromEntries(mission.robots.map((r) => [r.id, 0.0]));
  const rob = new Map(mission.robots.map((r) => [r.id, r])), tsk = new Map(mission.tasks.map((t) => [t.id, t]));
  let totalEnergy = 0.0;
  for (const [rid, tid] of result.assignments) {
    const r = rob.get(rid)!, t = tsk.get(tid)!;
    const dist = pyHypot(r.x - t.x, r.y - t.y);
    perRobot[rid] += dist;
    totalEnergy += dist * 0.2;
  }
  const total = mission.tasks.length;
  return {
    mission_id: mission.missionId, mode: 'static', duration_s: scenario.durationS,
    coverage_percent: total === 0 ? 100.0 : 100.0 * completed.length / total, reconfigurations_count: 0,
    missed_tasks: missed, completed_tasks: completed, per_robot_distance: perRobot, total_energy: totalEnergy, algorithm_time_ms: result.algorithmTimeMs,
  };
}

/** MuRKA mode of the orchard scenario (_run_murca_scenario). */
export function runMurkaScenario(mission: Mission, scenario: OrchardScenario, seed: number): Json {
  const isScout = (t: MurkaTask) => t.caps.size === 1 && t.caps.has('perception');
  const scoutTasks = mission.tasks.filter(isScout);
  const mainTasks = mission.tasks.filter((t) => !scoutTasks.includes(t));
  const t0 = now();
  const scouting = murkaAllocate(mission.robots, scoutTasks);
  const rng = new PyRandom(seed);
  const infection: MurkaTask[] = [];
  for (let i = 0; i < 3; i++) infection.push({ id: `spray_infect_${i + 1}`, x: 12.0 + rng.uniform(-1, 1), y: 10.0 + i * 1.5, caps: new Set(['spray']), priority: 3.0 });
  const combined = [...mainTasks, ...infection];
  const main = murkaAllocate(mission.robots, combined, { allowReconfig: true });
  const wall = (now() - t0) / 1000;
  const assignments = [...scouting.base.assignments, ...main.base.assignments];
  const attempted = scoutTasks.length + combined.length;
  const rob = new Map(mission.robots.map((r) => [r.id, r])), tsk = new Map([...mission.tasks, ...infection].map((t) => [t.id, t]));
  let totalDistance = 0.0;
  for (const [rid, tid] of assignments) { const r = rob.get(rid)!, t = tsk.get(tid)!; totalDistance += pyHypot(r.x - t.x, r.y - t.y); }
  return {
    mission_id: mission.missionId, mode: 'murca', duration_s: scenario.durationS,
    coverage_percent: attempted ? 100.0 * assignments.length / attempted : 100.0,
    reconfigurations_count: main.reconfigurations.length,
    reconfiguration_events: main.reconfigurations.map(([robot_id, trigger_task]) => ({ robot_id, trigger_task })),
    missed_tasks: main.base.unassignedTasks, completed_tasks: assignments.map(([, t]) => t),
    total_distance: totalDistance, total_energy: totalDistance * 0.2 + main.reconfigurations.length * 5.0,
    algorithm_time_ms: scouting.base.algorithmTimeMs + main.base.algorithmTimeMs, wallclock_s: wall,
  };
}

export function runExp3(runs: number, mode = 'all'): Json {
  const modes = mode === 'all' ? ['static', 'murca'] : [mode];
  const records: Record<string, Json[]> = Object.fromEntries(modes.map((m) => [m, []]));
  for (let trial = 0; trial < runs; trial++) {
    const mission = getMission('C', trial);
    const scenario = orchardScenario();
    if (modes.includes('static')) records.static.push(runStaticMission(mission, scenario));
    if (modes.includes('murca')) records.murca.push(runMurkaScenario(mission, scenario, trial));
  }
  const aggregate: Json = {};
  for (const m of modes) {
    const rs = records[m];
    if (!rs.length) continue;
    aggregate[m] = {
      avg_coverage_percent: pyMean(rs.map((r) => r.coverage_percent as number)),
      avg_reconfigurations: pyMean(rs.map((r) => (r.reconfigurations_count as number) ?? 0)),
      avg_total_energy: pyMean(rs.map((r) => r.total_energy as number)),
      avg_duration_s: pyMean(rs.map((r) => r.duration_s as number)),
      avg_missed_tasks: pyMean(rs.map((r) => (r.missed_tasks as string[]).length)),
    };
  }
  return { experiment_id: 'exp3', runs, modes, records, aggregate };
}

// ---------------------------------------------------------------------------------------------------------------
// exp4
// ---------------------------------------------------------------------------------------------------------------

function exp4Fleet(size: number, runs: number, order: readonly string[]): Json {
  const records: Json[] = [];
  for (let trial = 0; trial < runs; trial++) {
    const mission = generateRandomMission(size, size * 3, trial * 31 + size, order);
    const out = murkaAllocate(mission.robots, mission.tasks);
    const load = new Map<string, number>();
    for (const [rid] of out.base.assignments) load.set(rid, (load.get(rid) ?? 0) + 1);
    const avgLoad = load.size ? pyMean(load.values()) : 0.0;
    const maxLoad = load.size ? Math.max(...load.values()) : 0;
    const rob = new Map(mission.robots.map((r) => [r.id, r])), tsk = new Map(mission.tasks.map((t) => [t.id, t]));
    let totalDistance = 0.0;
    for (const [rid, tid] of out.base.assignments) { const r = rob.get(rid)!, t = tsk.get(tid)!; totalDistance += pyHypot(r.x - t.x, r.y - t.y); }
    records.push({
      trial, fleet_size: size, assignments_count: out.base.assignments.length, contracts_count: out.base.assignments.length,
      reconfigurations: out.reconfigurations.length, avg_load: avgLoad, max_load: maxLoad,
      parallelism_factor: out.base.assignments.length / Math.max(1, maxLoad), duration_s: maxLoad * 20.0,
      total_distance: totalDistance, total_energy: totalDistance * 0.2 + out.reconfigurations.length * 5.0, algorithm_time_ms: out.base.algorithmTimeMs,
    });
  }
  const f = (k: string) => records.map((r) => r[k] as number);
  return { fleet_size: size, records, aggregate: { avg_duration_s: pyMean(f('duration_s')), avg_total_energy: pyMean(f('total_energy')), avg_contracts_count: pyMean(f('contracts_count')), avg_load: pyMean(f('avg_load')), avg_parallelism_factor: pyMean(f('parallelism_factor')) } };
}

export function runExp4(runs: number, fleetSize: string | number = 'all', order: readonly string[] = ARTICLE_POPULATION_ORDER): Json {
  const sizes = fleetSize === 'all' ? [2, 4, 6, 8, 10] : [Number(fleetSize)];
  return { experiment_id: 'exp4', runs_per_size: runs, sizes, entries: sizes.map((n) => exp4Fleet(n, runs, order)) };
}

// ---------------------------------------------------------------------------------------------------------------
// exp5
// ---------------------------------------------------------------------------------------------------------------

/** Recipe energy profiles of the article (RECIPES_ENERGY: entropy u/s, drive u/m). */
export const RECIPES_ENERGY: Record<string, EnergyProfile> = {
  scout_ground: { baseEntropy: 0.02, driveCostPerM: 0.08, equipmentOverhead: 0.0, massKg: 35.0 },
  harvester: { baseEntropy: 0.04, driveCostPerM: 0.12, equipmentOverhead: 0.03, massKg: 60.0 },
  sprayer: { baseEntropy: 0.05, driveCostPerM: 0.14, equipmentOverhead: 0.04, massKg: 55.0 },
};

/** One seeded route (_simulate_route): speed 1 ± 0.1 m/s, energy from the configuration-dependent model, ±2 % noise. */
export function simulateRoute(recipe: string, lengthM: number, seed: number): Json {
  const rng = new PyRandom(seed);
  const p = RECIPES_ENERGY[recipe];
  if (!p) throw new Error(`unknown recipe ${recipe}`);
  const speed = 1.0 + rng.uniform(-0.1, 0.1);
  const { energy: e0, duration } = routeEnergy(p, lengthM, speed);
  const noise = rng.uniform(-0.02, 0.02) * e0;
  const energy = Math.max(0.0, e0 + noise);
  return { recipe, seed, route_length_m: lengthM, duration_s: duration, entropy_rate: entropyRate(p), drive_cost_per_m: p.driveCostPerM, total_energy: energy, energy_per_meter: energy / lengthM };
}

export function runExp5(runs: number, recipe = 'all', routeLengthM = 500.0): Json {
  const recipes = recipe === 'all' ? Object.keys(RECIPES_ENERGY) : [recipe];
  const entries = recipes.map((rec) => {
    const records = Array.from({ length: runs }, (_, trial) => simulateRoute(rec, routeLengthM, trial));
    const f = (k: string) => records.map((r) => r[k] as number);
    return { recipe: rec, records, aggregate: { avg_total_energy: pyMean(f('total_energy')), std_total_energy: records.length > 1 ? pyPstdev(f('total_energy')) : 0.0, avg_duration_s: pyMean(f('duration_s')), avg_energy_per_meter: pyMean(f('energy_per_meter')) } };
  });
  return { experiment_id: 'exp5', runs_per_recipe: runs, route_length_m: routeLengthM, entries };
}

// ---------------------------------------------------------------------------------------------------------------
// C2 entry point + KPI exporter
// ---------------------------------------------------------------------------------------------------------------

export interface AgroExperimentResult {
  id: AgroExperimentId;
  /** File name → KPI JSON in the schema of murca_experiment's artifacts/experiments/*.json. */
  files: Record<string, Json>;
}

/** Run one MURKA article experiment with the studio's models; returns the KPI JSON files (Python artifact schema). */
export function runAgroExperiment(id: AgroExperimentId, opts: AgroExperimentOptions = {}): AgroExperimentResult {
  const runs = opts.runs ?? DEFAULT_RUNS[id];
  const order = opts.populationOrder ?? ARTICLE_POPULATION_ORDER;
  switch (id) {
    case 'exp1': return { id, files: Object.fromEntries(runExp1(runs, opts.missionPreset ?? 'all').map((p) => [`exp1_mission_${p.mission_preset}.json`, p])) };
    case 'exp2': return { id, files: { 'exp2_allocation.json': runExp2(runs, { ...opts, populationOrder: order }) } };
    case 'exp3': return { id, files: { 'exp3_full_scenario.json': runExp3(runs, opts.mode ?? 'all') } };
    case 'exp4': return { id, files: { 'exp4_scaling.json': runExp4(runs, opts.fleetSize ?? 'all', order) } };
    case 'exp5': return { id, files: { 'exp5_energy.json': runExp5(runs, opts.recipe ?? 'all', opts.routeLengthM ?? 500.0) } };
    default: throw new Error(`unknown experiment: ${String(id)}`);
  }
}

/** All five experiments (`article_experiments all --runs N`). */
export function runAllAgroExperiments(opts: AgroExperimentOptions = {}): Record<string, Json> {
  const out: Record<string, Json> = {};
  for (const id of ['exp1', 'exp2', 'exp3', 'exp4', 'exp5'] as AgroExperimentId[]) Object.assign(out, runAgroExperiment(id, opts).files);
  return out;
}

/** Python `json.dump(indent=2)`-like text of a KPI file (for download / article_plots.py). */
export function kpiJsonText(payload: Json): string { return JSON.stringify(payload, null, 2); }
