/**
 * Group-control (multi-robot systems) module — model kinds. The module implements the course *Управление
 * распределёнными робототехническими системами* (control of distributed robotic systems, BMSTU): consensus and
 * formations on communication graphs, swarm behaviour and swarm intelligence, task allocation, multi-agent path
 * finding, coverage and distributed estimation, group safety with barrier functions, the multi-robot warehouse of
 * the homework, and the chapter theory (games, learning, evolution, cellular automata, fuzzy control, resilience).
 *
 * The documents are `ControlModelItem`s of these kinds (they live in the same station tree and Control tab as the
 * control-design models); the analysis is in mrs/analysis.ts, the DSL in mrs/dsl.ts, the fleet runtime in
 * mrs/runtime.ts.
 */
export type MrsKind = 'mission' | 'consensus' | 'swarm' | 'allocation' | 'gridmapf' | 'coverage' | 'estimation' | 'safety' | 'warehouse' | 'game' | 'marl' | 'evo' | 'ca' | 'fuzzy' | 'resilience';

export interface MrsKindInfo { kind: MrsKind; label: string; /** chapters of the group-control course */ chapter: string; /** practicum / homework that the kind implements */ practicum: string; /** can be executed on the station fleet (Run on fleet) */ runnable: boolean }

export const MRS_KINDS: MrsKindInfo[] = [
  { kind: 'mission', label: 'Group mission over configured robots: centralised / decentralised / hybrid', chapter: '3, 4, 10, 16', practicum: 'architectures', runnable: true },
  { kind: 'consensus', label: 'Consensus, formation, connectivity (graph Laplacian)', chapter: '4, 13, 16', practicum: 'ПР1', runnable: true },
  { kind: 'swarm', label: 'Swarm: boids, Vicsek, PSO / ACO source search', chapter: '5', practicum: 'ПР2', runnable: true },
  { kind: 'allocation', label: 'Task allocation: greedy, Hungarian, SSI, CBBA, Vickrey', chapter: '9–10', practicum: 'ПР3', runnable: false },
  { kind: 'gridmapf', label: 'Grid MAPF: space-time A*, prioritized, CBS, ADG execution', chapter: '11', practicum: 'ПР4', runnable: true },
  { kind: 'coverage', label: 'Coverage: Voronoi / Lloyd (limited range)', chapter: '11', practicum: 'ПР5', runnable: true },
  { kind: 'estimation', label: 'Distributed estimation: information consensus, CI', chapter: '4, 11', practicum: 'ПР5', runnable: false },
  { kind: 'safety', label: 'Group safety: barrier functions, deadlock rule, stale data', chapter: '13, 16', practicum: 'ПР6', runnable: true },
  { kind: 'warehouse', label: 'Warehouse fleet: CBBA orders, executor FSM, cell reservation', chapter: '10–11, 16', practicum: 'ДЗ', runnable: true },
  { kind: 'game', label: 'Games: Nash, Pareto, fictitious play, potential, Shapley', chapter: '9', practicum: '—', runnable: false },
  { kind: 'marl', label: 'Reinforcement learning: Q-learning, independent MARL', chapter: '7', practicum: '—', runnable: false },
  { kind: 'evo', label: 'Evolutionary tuning: GA, DE, ES', chapter: '12', practicum: '—', runnable: false },
  { kind: 'ca', label: 'Cellular automata: Wolfram rules, Life, pheromone', chapter: '4 §4.6, 5', practicum: '—', runnable: false },
  { kind: 'fuzzy', label: 'Fuzzy (Mamdani) obstacle avoidance', chapter: '13 §13.4', practicum: '—', runnable: false },
  { kind: 'resilience', label: 'Resilience: Byzantine agreement, trust, degradation, switching', chapter: '16, 13 §13.5', practicum: '—', runnable: false },
];
export const MRS_KIND_SET = new Set<string>(MRS_KINDS.map((k) => k.kind));
export const isMrsKind = (k: string): k is MrsKind => MRS_KIND_SET.has(k);
export const mrsInfo = (k: MrsKind): MrsKindInfo => MRS_KINDS.find((x) => x.kind === k)!;
