/**
 * Analysis dispatcher of the group-control module: kind + document text → report sections, metrics and charts.
 * Plugged into ctl/analysis.ts (`analyse(kind, source)`), so the Control tab, the API and the demo scenarios use
 * the same entry point for both courses.
 */
import type { Analyser } from '../ctl/analysis';
import { MrsKind } from './model';
import { parseMission, parseConsensus, parseSwarm, parseAllocation, parseGridMapf, parseCoverage, parseEstimation, parseSafety, parseWarehouse, parseGame, parseMarl, parseEvo, parseCa, parseFuzzy, parseResilience } from './dsl';
import { analyseConsensus, analyseSwarm, analyseAllocation, analyseGridMapf, analyseCoverage, analyseEstimation, analyseSafety, analyseWarehouse } from './an_practicum';
import { analyseMission } from './an_mission';
import { analyseGame, analyseMarl, analyseEvo, analyseCa, analyseFuzzy, analyseResilience } from './an_theory';

export const MRS_ANALYSERS: Record<MrsKind, Analyser> = {
  mission: (src, S, M) => { const d = parseMission(src); return { title: `Group mission (${d.architecture}): ${d.name}`, ok: analyseMission(d, S, M) }; },
  consensus: (src, S, M) => { const d = parseConsensus(src); return { title: `Consensus & formation: ${d.name}`, ok: analyseConsensus(d, S, M) }; },
  swarm: (src, S, M) => { const d = parseSwarm(src); return { title: `Swarm (${d.model}): ${d.name}`, ok: analyseSwarm(d, S, M) }; },
  allocation: (src, S, M) => { const d = parseAllocation(src); return { title: `Task allocation: ${d.name}`, ok: analyseAllocation(d, S, M) }; },
  gridmapf: (src, S, M) => { const d = parseGridMapf(src); return { title: `Grid MAPF: ${d.name}`, ok: analyseGridMapf(d, S, M) }; },
  coverage: (src, S, M) => { const d = parseCoverage(src); return { title: `Coverage: ${d.name}`, ok: analyseCoverage(d, S, M) }; },
  estimation: (src, S, M) => { const d = parseEstimation(src); return { title: `Distributed estimation: ${d.name}`, ok: analyseEstimation(d, S, M) }; },
  safety: (src, S, M) => { const d = parseSafety(src); return { title: `Group safety: ${d.name}`, ok: analyseSafety(d, S, M) }; },
  warehouse: (src, S, M) => { const d = parseWarehouse(src); return { title: `Warehouse fleet: ${d.name}`, ok: analyseWarehouse(d, S, M) }; },
  game: (src, S, M) => { const d = parseGame(src); return { title: `Game: ${d.name}`, ok: analyseGame(d, S, M) }; },
  marl: (src, S, M) => { const d = parseMarl(src); return { title: `Reinforcement learning: ${d.name}`, ok: analyseMarl(d, S, M) }; },
  evo: (src, S, M) => { const d = parseEvo(src); return { title: `Evolutionary tuning: ${d.name}`, ok: analyseEvo(d, S, M) }; },
  ca: (src, S, M) => { const d = parseCa(src); return { title: `Cellular automata: ${d.name}`, ok: analyseCa(d, S, M) }; },
  fuzzy: (src, S, M) => { const d = parseFuzzy(src); return { title: `Fuzzy control: ${d.name}`, ok: analyseFuzzy(d, S, M) }; },
  resilience: (src, S, M) => { const d = parseResilience(src); return { title: `Resilience: ${d.name}`, ok: analyseResilience(d, S, M) }; },
};
