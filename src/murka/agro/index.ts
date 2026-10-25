/**
 * MURKA agromarca reproduced with the studio (contract C2):
 *  - buildMurkaOrchardStation / runMissionB — the Webots orchards and the Mission B scripted fleet (orchard.ts);
 *  - runS1Benchmark — benchmark scenario S1 "scouting → harvest → crate logistics" with role FSMs (s1.ts);
 *  - runAgroExperiment — the article experiments exp1..exp5 with KPI JSON in the Python artifact schema (experiments.ts).
 */
export { buildMurkaOrchardStation, runMissionB, configureOrchardRobots, treeProfile, crownRadiusIn, ORCHARD_WORLDS, AGRO_WS_REL, MISSION_B_ROLES } from './orchard';
export type { OrchardWorld, OrchardStationOptions, OrchardStationResult, TreeProfile, MissionBOptions } from './orchard';
export { runS1Benchmark, s1World, S1_DEFAULTS } from './s1';
export type { S1Options, S1Result, S1Metrics, S1Event, S1Level, S1Controller, S1Params, S1Timeline } from './s1';
export { S1_FSMS } from './s1_fsm';
export { murkaKpiFromFleet, murkaKpiFromS1, murkaKpiArtifact } from './kpi';
export type { MurkaKpiRecord } from './kpi';
export { addHarvestFleet } from './orchard';
export type { HarvestFleetOptions } from './orchard';
export { runAgroExperiment, runAllAgroExperiments, kpiJsonText, RECIPES_ENERGY, runExp1, runExp2, runExp3, runExp4, runExp5 } from './experiments';
export type { AgroExperimentId, AgroExperimentOptions, AgroExperimentResult } from './experiments';
export { generateRandomMission, getMission, ARTICLE_POPULATION_ORDER, PY312_HASHSEED0_ORDER } from './missions';
export { PyRandom, pyHypot, pyMean, pyPstdev, npStd } from './pycompat';
