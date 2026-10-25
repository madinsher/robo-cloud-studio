/**
 * MURKA retail robot (МУРОМ / MURKA, robot-retailer-murka v1 / v2 and the MURKA_Simulation copies) reproduced with the
 * studio's own simulator — contract C1: buildMurkaRetailStation + runMurkaRetailMission. See docs/murka-retail.md.
 */
export { buildMurkaRetailStation, muromHandles, configureMurom, retailStationOf, RETAIL_VARIANTS, DEFAULT_MURKA_ROOT } from './station';
export type { RetailVariant, RetailStationOptions, RetailStationResult, MuromHandles, VariantInfo } from './station';
export { runMurkaRetailMission, createRetailSession, retailSessionOf } from './mission';
export type { RetailMissionResult, RetailMissionOptions, RetailStage, RetailChecks, RetailSession } from './mission';
export { parseMission, missionFsmDsl, RetailOrchestrator, MISSION_DEFAULTS } from './orchestrator';
export type { MissionTask, ProgressEvent } from './orchestrator';
export { parseRetailLayout } from './layout';
export type { RetailLayout, RetailSlot } from './layout';
export { ArmKinematics } from './kinematics';
export { MuromSim } from './sim';
export { Navigator } from './nav';
export { ManipServer } from './manip';
export { recognize, classify, calibrate, renderHeadFrame, loadCatalog } from './perception';
export { STUDIO_APPEARANCE } from './appearance';
