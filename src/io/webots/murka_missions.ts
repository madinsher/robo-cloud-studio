/**
 * MURKA agromarca Mission B (Fig. 10 of the article): the deterministic script of
 * `garden_supervisor.default_mission_b_script` (orchard_demo.wbt layout of 2026-10-01), ported as data so the studio
 * plays the scenario without Python. Played with `playScriptedFleet` (scripted_fleet.ts) on the imported
 * orchard_demo.wbt: UTP-G-01 carries its crate to the S1 working cell, UTP-G-04 brings an empty crate, the crates are
 * swapped in the aisle, the full one is delivered to collection point 1; UTP-G-02 scouts S3 and is reconfigured
 * (scout mast -> sprayer) at depot slot 1 at t = 60 s; UTP-G-03 drives to S2; UTP-A-01 flies to S4.
 */
import type { FleetScript, WaypointJson } from './scripted_fleet';

export const MISSION_B = {
  LANE_W1: -13.5, LANE_W2: -10.5, LANE_E1: 4.5, LANE_E2: 7.5, CELL_N: 6.5, CELL_S: -4.5, Y_PATH: 1.0,
  AERIAL_Z: 5.0, PAD_REST_Z: 0.0505, HALF_PI: 1.5708, TREE_TOP_Z: 3.55,
  /** Robots, crates and depot modules of the scenario (DEF = name.upper(), '-' -> '_'). */
  ROBOTS: ['UTP-G-01', 'UTP-G-02', 'UTP-G-03', 'UTP-G-04', 'UTP-A-01'],
  CRATES: ['crate_work', 'crate_empty_1', 'crate_empty_2'],
  DEPOT_MODULES: ['module_sprayer', 'module_harvester_arm', 'module_scout_mast'],
  /** Length of the reference playback (s) and the supervisor time step (orchard_demo.wbt basicTimeStep 16 ms). */
  T_END: 92.0, DT: 0.016,
};

function wp(robot: string, t0: number, t1: number, x: number, y: number, yaw?: number, z?: number, drive?: string, mode?: string): WaypointJson {
  const target: WaypointJson['target'] = { x, y };
  if (z !== undefined) target.z = z;
  if (yaw !== undefined) target.yaw = yaw;
  const w: WaypointJson = { robot_id: robot, start_s: t0, end_s: t1, target };
  if (drive !== undefined) w.drive = drive;
  if (mode !== undefined) w.yaw_mode = mode;
  return w;
}
/** A crate pushed along the floor: straight line, own yaw kept (interp, trapezoid). */
function slide(crate: string, t0: number, t1: number, x: number, y: number, yaw: number = MISSION_B.HALF_PI, z?: number): WaypointJson {
  const w = wp(crate, t0, t1, x, y, yaw, z, undefined, 'interp');
  w.profile = 'trapezoid';
  return w;
}

/** The default Mission B script (same JSON as the Python supervisor; `includeHarvester` false leaves UTP-G-01 to ros2_control). */
export function defaultMissionBScript(includeHarvester = true): FleetScript {
  const { LANE_W1, LANE_W2, LANE_E1, CELL_N, CELL_S, Y_PATH, AERIAL_Z, PAD_REST_Z, HALF_PI } = MISSION_B;
  const N = HALF_PI, WEST = 2 * HALF_PI;
  const [G1, G2, G3, G4, A1] = MISSION_B.ROBOTS;
  const harvester = [wp(G1, 1.0, 6.0, LANE_W2, -8.5, N), wp(G1, 6.0, 19.5, LANE_W2, CELL_N)];
  const logistics = [
    wp(G4, 1.5, 12.5, -4.9, -10.4, N),
    slide('crate_empty_2', 12.5, 16.0, -4.9, -10.4, 0.0),
    wp(G4, 18.5, 28.5, -4.9, -1.0),
    wp(G4, 28.5, 41.5, LANE_W2, 2.0, N),
    slide('crate_work', 42.5, 44.5, LANE_W2, 4.25),
    slide('crate_work', 44.5, 46.0, LANE_W2 + 0.9, 4.25),
    slide('crate_empty_2', 46.0, 50.0, LANE_W2, CELL_N),
    slide('crate_work', 50.0, 51.5, LANE_W2, 4.25),
    slide('crate_work', 51.5, 54.0, LANE_W2, 2.0),
    wp(G4, 55.0, 67.5, LANE_W2, -9.0, N, undefined, 'reverse'),
    slide('crate_work', 67.5, 70.0, -9.45, -9.0),
    slide('crate_work', 70.0, 70.02, -9.45, -9.0, undefined, PAD_REST_Z),
    slide('crate_work', 70.1, 72.5, -8.4, -9.0, undefined, PAD_REST_Z),
  ];
  const scout = [
    wp(G2, 2.5, 4.5, -8.0, -12.9, N, undefined, 'reverse'),
    wp(G2, 4.5, 20.7, LANE_W2, -12.9, N),
    wp(G2, 20.7, 31.0, -11.4, -9.6, WEST),
    wp(G2, 31.0, 40.0, LANE_W1, -9.6, N),
    wp(G2, 40.0, 47.0, LANE_W1, CELL_S),
    wp(G2, 49.0, 57.0, LANE_W1, -9.8, N, undefined, 'reverse'),
    wp(G2, 66.0, 84.0, LANE_W1, CELL_S, N),
  ];
  const sprayer = [wp(G3, 2.0, 15.0, -4.5, Y_PATH, N), wp(G3, 15.0, 31.0, LANE_E1, Y_PATH), wp(G3, 31.0, 42.0, LANE_E1, CELL_N)];
  const drone = [wp(A1, 0.0, 3.0, 0.8, -12.5, undefined, AERIAL_Z), wp(A1, 3.0, 20.0, 6.0, CELL_S, undefined, AERIAL_Z)];
  const aligned = () => ({ x: 0.0, y: 0.0, yaw: 0.0 });
  return {
    yaw_mode: 'tangent',
    waypoints: [...(includeHarvester ? harvester : []), ...logistics, ...scout, ...sprayer, ...drone],
    cmd_vel: [],
    attachments: [{ child: 'module_scout_mast', carrier: G2 }],
    carries: [
      { child: 'crate_work', carrier: G1, start_s: 1.0, end_s: 19.5, offset: aligned() },
      { child: 'crate_empty_2', carrier: G4, start_s: 16.0, end_s: 42.0, offset: aligned() },
      { child: 'crate_work', carrier: G4, start_s: 54.0, end_s: 67.5, offset: aligned() },
    ],
    reconfigs: [{ robot_id: G2, start_s: 60.0, depot_slot: 'depot_slot_1', old_module: 'module_scout_mast', new_module: 'module_sprayer', transfer_s: 2.5 }],
  };
}
