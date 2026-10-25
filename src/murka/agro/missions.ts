/**
 * MURKA agromarca missions (murca_experiment/mission_presets.py): the canonical fleet pool (UTP-G ground platforms,
 * UTP-A aerial platforms), presets A (light scouting), B (harvest-heavy), C (mixed critical, + sprayer UTP-G-05),
 * random missions for experiments 2 / 4 and the orchard scenario of agent.md §8 used by experiment 3.
 *
 * Determinism: the presets draw from CPython's `random.Random(seed)` (PyRandom). `generate_random_mission` samples
 * extra capabilities from `list(set(ALL_CAPABILITIES) - caps)`, whose order is CPython's set iteration order and
 * therefore depends on PYTHONHASHSEED: it is the `populationOrder` parameter here. ARTICLE_POPULATION_ORDER reproduces
 * the article's published artifacts (replication/artifacts/experiments), PY312_HASHSEED0_ORDER a CPython 3.12 run with
 * PYTHONHASHSEED=0.
 */
import { PyRandom } from './pycompat';
import type { MurkaRobot, MurkaTask } from '../../fleet/murka_alloc';

export interface Mission {
  missionId: string;
  description: string;
  robots: MurkaRobot[];
  tasks: MurkaTask[];
  requiredCapabilities: Set<string>;
  minimumFleetSize: number;
  expectedReconfigurations: number;
}

export function coveragePercent(m: Mission, chosen: ReadonlySet<string>): number {
  if (!m.requiredCapabilities.size) return 100.0;
  let covered = 0; for (const c of chosen) if (m.requiredCapabilities.has(c)) covered++;
  return 100.0 * covered / m.requiredCapabilities.size;
}

const R = (id: string, x: number, y: number, battery: number, caps: string[]): MurkaRobot => ({ id, x, y, battery, caps: new Set(caps) });
const T = (id: string, x: number, y: number, caps: string[], priority: number): MurkaTask => ({ id, x, y, caps: new Set(caps), priority });

/** Canonical fleet pool (BASE_ROBOTS). */
export function baseRobots(): MurkaRobot[] {
  return [
    R('UTP-G-01', 2.0, 14.0, 1.0, ['perception', 'navigation']),
    R('UTP-G-02', 2.0, 16.0, 0.95, ['perception', 'manipulation', 'harvest', 'cargo_carry']),
    R('UTP-G-03', 4.0, 14.0, 0.9, ['perception', 'manipulation', 'harvest', 'cargo_carry']),
    R('UTP-G-04', 4.0, 16.0, 1.0, ['cargo_carry', 'navigation']),
    R('UTP-A-01', 0.0, 0.0, 1.0, ['perception', 'navigation', 'aerial']),
    R('UTP-A-02', 0.0, 1.0, 1.0, ['perception', 'navigation', 'aerial']),
  ];
}

export function missionPresetA(rng: PyRandom = new PyRandom(1)): Mission {
  const tasks: MurkaTask[] = [];
  for (let i = 0; i < 4; i++) { const x = 10.0 + rng.uniform(-2, 2); const y = 10.0 + i * 4.0 + rng.uniform(-1, 1); tasks.push(T(`scout_S${i + 1}`, x, y, ['perception'], 1.0)); }
  return { missionId: 'A', description: 'Light scouting — 4 perception tasks.', robots: baseRobots().filter((r) => !r.caps.has('aerial')), tasks, requiredCapabilities: new Set(['perception', 'navigation']), minimumFleetSize: 2, expectedReconfigurations: 0 };
}

export function missionPresetB(rng: PyRandom = new PyRandom(2)): Mission {
  const tasks: MurkaTask[] = [];
  for (let i = 0; i < 4; i++) { const x = 12.0 + rng.uniform(-1, 1); const y = 8.0 + i * 3.0 + rng.uniform(-1, 1); tasks.push(T(`harvest_S${i + 1}`, x, y, ['perception', 'manipulation', 'harvest'], 2.0)); }
  for (let i = 0; i < 2; i++) tasks.push(T(`scout_row_${i + 1}`, 8.0 + rng.uniform(-1, 1), 6.0 + i * 6.0, ['perception'], 1.0));
  for (let i = 0; i < 2; i++) tasks.push(T(`transport_${i + 1}`, 4.0 + rng.uniform(-1, 1), 18.0 - i * 6.0, ['cargo_carry'], 1.5));
  return { missionId: 'B', description: 'Harvest-heavy — 4 harvest + 2 scout + 2 transport.', robots: baseRobots(), tasks, requiredCapabilities: new Set(['perception', 'navigation', 'manipulation', 'harvest', 'cargo_carry']), minimumFleetSize: 4, expectedReconfigurations: 1 };
}

export function missionPresetC(rng: PyRandom = new PyRandom(3)): Mission {
  const tasks: MurkaTask[] = [];
  for (let i = 0; i < 4; i++) tasks.push(T(`harvest_S${i + 1}`, 12.0 + rng.uniform(-1, 1), 8.0 + i * 3.0, ['perception', 'manipulation', 'harvest'], 2.0));
  for (let i = 0; i < 3; i++) tasks.push(T(`spray_S${i + 5}`, 14.0 + rng.uniform(-1, 1), 12.0 + i * 2.0, ['spray'], 2.5));
  for (let i = 0; i < 2; i++) tasks.push(T(`prune_S${i + 8}`, 10.0, 14.0 + i * 2.0, ['manipulation', 'prune'], 1.5));
  for (let i = 0; i < 2; i++) tasks.push(T(`scout_full_row_${i + 1}`, 6.0, 4.0 + i * 8.0, ['perception'], 1.0));
  tasks.push(T('transport_depot', 2.0, 10.0, ['cargo_carry'], 1.0));
  const robots = [...baseRobots(), R('UTP-G-05', 6.0, 16.0, 1.0, ['perception', 'spray', 'navigation'])];
  return { missionId: 'C', description: 'Mixed critical — harvest + spray + prune + scout + transport.', robots, tasks, requiredCapabilities: new Set(['perception', 'navigation', 'manipulation', 'harvest', 'cargo_carry', 'spray', 'prune']), minimumFleetSize: 6, expectedReconfigurations: 2 };
}

export const MISSION_FACTORIES: Record<'A' | 'B' | 'C', (rng?: PyRandom) => Mission> = { A: missionPresetA, B: missionPresetB, C: missionPresetC };

export function getMission(preset: string, seed?: number): Mission {
  const f = MISSION_FACTORIES[preset as 'A'];
  if (!f) throw new Error(`Unknown mission preset: ${preset}`);
  return f(seed === undefined ? undefined : new PyRandom(seed));
}

export const ALL_CAPABILITIES = ['perception', 'navigation', 'manipulation', 'harvest', 'cargo_carry', 'spray', 'prune'] as const;
/** Set order of {manipulation, harvest, cargo_carry, spray, prune} in the article's run (Python 3.10, random hash seed). */
export const ARTICLE_POPULATION_ORDER = ['harvest', 'manipulation', 'cargo_carry', 'prune', 'spray'];
/** Same set under CPython 3.12, PYTHONHASHSEED=0. */
export const PY312_HASHSEED0_ORDER = ['harvest', 'cargo_carry', 'manipulation', 'prune', 'spray'];

/** generate_random_mission(n_robots, n_tasks, seed, area=(30, 30)). */
export function generateRandomMission(nRobots: number, nTasks: number, seed: number, populationOrder: readonly string[] = ARTICLE_POPULATION_ORDER, area: [number, number] = [30.0, 30.0]): Mission {
  const extras0 = ALL_CAPABILITIES.filter((c) => c !== 'perception' && c !== 'navigation');
  if (populationOrder.length !== extras0.length || extras0.some((c) => !populationOrder.includes(c))) throw new Error(`populationOrder must be a permutation of ${extras0.join(', ')}`);
  const rng = new PyRandom(seed);
  const robots: MurkaRobot[] = [];
  for (let i = 0; i < nRobots; i++) {
    const caps = new Set(['perception', 'navigation']);
    const k = rng.randint(0, 3);
    for (const c of rng.sample(populationOrder, k)) caps.add(c);
    const x = rng.uniform(0, area[0]), y = rng.uniform(0, area[1]), battery = rng.uniform(0.6, 1.0);
    robots.push({ id: `R${i + 1}`, x, y, battery, caps });
  }
  const tasks: MurkaTask[] = [];
  for (let j = 0; j < nTasks; j++) {
    const choice = rng.random();
    const required = choice < 0.3 ? ['perception'] : choice < 0.55 ? ['perception', 'manipulation', 'harvest'] : choice < 0.75 ? ['spray'] : choice < 0.9 ? ['manipulation', 'prune'] : ['cargo_carry'];
    const x = rng.uniform(0, area[0]), y = rng.uniform(0, area[1]), priority = rng.uniform(1.0, 3.0);
    tasks.push(T(`T${j + 1}`, x, y, required, priority));
  }
  return {
    missionId: `random_${nRobots}x${nTasks}_${seed}`, description: `Random mission with ${nRobots} robots / ${nTasks} tasks.`, robots, tasks,
    requiredCapabilities: new Set(tasks.flatMap((t) => [...t.caps])), minimumFleetSize: Math.max(1, Math.floor(nTasks / 3)), expectedReconfigurations: 0,
  };
}

/** The canonical orchard scenario of agent.md §8 (experiment 3). */
export interface OrchardScenario { durationS: number; sectors: string[]; fruitTreesPerSector: number; infectedSector: string; expectedReconfigurations: number }
export const orchardScenario = (): OrchardScenario => ({ durationS: 100.0, sectors: ['S1', 'S2', 'S3', 'S4'], fruitTreesPerSector: 25, infectedSector: 'S3', expectedReconfigurations: 2 });
