/**
 * MURKA KPI exporter: studio runs (FleetManager missions, benchmark S1) as records in the schema of
 * murca_experiment's experiment 3 artifacts (`artifacts/experiments/exp3_full_scenario.json` → `records[mode][]`), so
 * `article_plots.py` (and the article tables) can plot studio runs next to the Python model:
 * `mission_id, mode, duration_s, coverage_percent, reconfigurations_count, reconfiguration_events, missed_tasks,
 * completed_tasks, total_distance, total_energy, algorithm_time_ms` (+ studio extras prefixed `studio_`).
 *
 * Energy follows the article's proxy (0.2 u/m + 5 u per reconfiguration) so the values compare with the Python model;
 * the measured battery energy (Wh) is kept as `studio_energy_wh`.
 */
import type { FleetManager } from '../../fleet/fleet';
import type { S1Result } from './s1';

export interface MurkaKpiRecord {
  mission_id: string;
  mode: string;
  duration_s: number;
  coverage_percent: number;
  reconfigurations_count: number;
  reconfiguration_events: Array<{ robot_id: string; trigger_task: string }>;
  missed_tasks: string[];
  completed_tasks: string[];
  total_distance: number;
  total_energy: number;
  algorithm_time_ms: number;
  [extra: `studio_${string}`]: unknown;
}

/** Record of a FleetManager run (tasks of the fleet; robots' distance in m). */
export function murkaKpiFromFleet(fm: FleetManager, missionId = fm.fleet.name, mode = `studio_${fm.fleet.allocation}`): MurkaKpiRecord {
  const k = fm.kpis();
  const tasks = fm.fleet.tasks.filter((t) => t.type !== 'charge');
  const done = tasks.filter((t) => t.status === 'done');
  const reconf = tasks.filter((t) => t.meta?.reconfiguredFrom || t.meta?._rt?.reconfig?.recipeApplied).map((t) => ({ robot_id: fm.station.findById(t.robotId ?? '')?.name ?? String(t.robotId), trigger_task: t.id }));
  const nReconf = Number((k.process as Record<string, Record<string, number>>).equipment?.reconfigurations ?? reconf.length);
  const dist = k.distanceTravelled / 1000;
  return {
    mission_id: missionId, mode,
    duration_s: fm.time,
    coverage_percent: tasks.length ? (100.0 * done.length) / tasks.length : 100.0,
    reconfigurations_count: nReconf,
    reconfiguration_events: reconf,
    missed_tasks: tasks.filter((t) => t.status !== 'done').map((t) => t.id),
    completed_tasks: done.map((t) => t.id),
    total_distance: dist,
    total_energy: dist * 0.2 + nReconf * 5.0,
    algorithm_time_ms: 0,
    studio_energy_wh: k.energyUsedWh,
    studio_fruit_picked: k.fruitPicked,
    studio_yield_kg: k.yieldKg,
    studio_contacts: k.contacts.robotObstacleSteps + k.contacts.robotRobotSteps,
  };
}

/** Record of a benchmark S1 run (rows = tasks; drone survey + swaps are part of the mission). */
export function murkaKpiFromS1(r: S1Result): MurkaKpiRecord {
  const m = r.metrics;
  const rows = r.rows.map((x) => x.id);
  const done = new Set(Object.values(m.rowsByHarvester).flat().map((id) => id.replace(/b$/, '')));
  const dist = Object.values(m.distance).reduce((a, b) => a + b, 0);
  return {
    mission_id: `S1-${r.level}-seed${r.seed}`, mode: r.controller,
    duration_s: m.makespan,
    coverage_percent: rows.length ? (100.0 * rows.filter((id) => done.has(id)).length) / rows.length : 100.0,
    studio_kg_completeness: m.completeness,
    reconfigurations_count: 0,
    reconfiguration_events: [],
    missed_tasks: rows.filter((id) => !done.has(id)),
    completed_tasks: rows.filter((id) => done.has(id)),
    total_distance: dist,
    total_energy: dist * 0.2,
    algorithm_time_ms: 0,
    studio_wait_swap_s: m.waitSwapTotal,
    studio_boxes: m.boxesDelivered,
    studio_mission_end_s: m.missionEnd,
    studio_messages: m.comm.messages,
  };
}

/** An exp3-style artifact (`{experiment_id, runs, modes, records, aggregate}`) from studio records. */
export function murkaKpiArtifact(records: MurkaKpiRecord[], experimentId = 'studio_exp3'): Record<string, unknown> {
  const modes = [...new Set(records.map((r) => r.mode))];
  const by = Object.fromEntries(modes.map((mo) => [mo, records.filter((r) => r.mode === mo)]));
  const mean = (a: number[]) => (a.length ? a.reduce((x, y) => x + y, 0) / a.length : 0);
  const aggregate = Object.fromEntries(modes.map((mo) => [mo, {
    avg_coverage_percent: mean(by[mo].map((r) => r.coverage_percent)),
    avg_reconfigurations: mean(by[mo].map((r) => r.reconfigurations_count)),
    avg_total_energy: mean(by[mo].map((r) => r.total_energy)),
    avg_duration_s: mean(by[mo].map((r) => r.duration_s)),
    avg_missed_tasks: mean(by[mo].map((r) => r.missed_tasks.length)),
  }]));
  return { experiment_id: experimentId, runs: Math.max(0, ...modes.map((mo) => by[mo].length)), modes, records: by, aggregate };
}
