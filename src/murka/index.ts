/**
 * MURKA reproductions (shared entry point, loaded with a dynamic import by the app's MURKA menu — contract C4).
 * Retail store (R3): buildMurkaRetailStation / runMurkaRetailMission. Orchard / agro (R4) add their exports below.
 */
export { buildMurkaRetailStation, runMurkaRetailMission, runMurkaE2E, createRetailSession, retailStationOf, retailSessionOf, RETAIL_VARIANTS, parseMission, missionFsmDsl } from './retail';
export type { RetailVariant, RetailStationOptions, RetailStationResult, RetailMissionResult, RetailMissionOptions, RetailStage, MissionTask, ProgressEvent, RetailE2EResult } from './retail';
// R4 (murka-agro) — contract C2 (line added by R3 when creating this file; R4 owns it)
export { buildMurkaOrchardStation, runMissionB, runS1Benchmark, runS1InWorker, s1WorkerAvailable, s1DownloadJson, runAgroExperiment, runAllAgroExperiments, kpiJsonText } from './agro';
export type { OrchardWorld, OrchardStationOptions, OrchardStationResult, S1Options, S1Result, S1Level, S1Controller, AgroExperimentId, AgroExperimentOptions, AgroExperimentResult } from './agro';
