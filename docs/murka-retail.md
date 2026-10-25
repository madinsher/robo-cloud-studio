# MURKA retail robot (МУРОМ) in the studio simulator

The retail scenario of the MURKA / МУРОМ project (`robot-retailer-murka-v2`, `robot-retailer-murka` v1-classic and the
renamed copies in `simulation_ws/src/MURKA_Simulation`) — originally ROS 2 Jazzy + Webots R2025a (and MuJoCo) — runs in
VerticalBot Studio with the studio's **own** simulator: the Webots world is imported, the ROS nodes are reproduced as
studio code (same algorithms, same parameters), the mission FSM is a studio control model executed by the studio's FSM
executor. Nothing of ROS or Webots runs.

```ts
import { buildMurkaRetailStation, runMurkaRetailMission } from './src/murka';      // or src/murka/retail
const built = await buildMurkaRetailStation({ variant: 'v2', root: MURKA_DIR });   // browser: { files } of the picked folder
const run = runMurkaRetailMission(built, { mission: 'full' });                     // 'depalletize' | 'order' | 'scan' | 'goto' | 'dock' | 'undock' | 'home'
const result = await run.done;                                                     // run.cancel() = /murom/task/cancel
console.log(result.report.join('\n'));
```

## What is reproduced, and from which MURKA source

| MURKA (ROS 2 / Webots) | Studio (src/murka/retail) | Fidelity |
|---|---|---|
| `retail_store.wbt` + PROTOs (StoreShelf JS template, pallets, packing table, charging station, Murom v1/v2) | `station.ts` on the Webots importer (docs/webots.md): shelves, colliders, MUROM split into a differential MobileRobot, two arm Robots (6 DOF v2 / 4 DOF v1) and a head Robot with `head_camera` + `head_depth`, lidar | every shape within 0.5 mm of Webots' own pose dump (T6); arm FK = `arm_kinematics.py` to 1e-9 m |
| supervisor plugin: products from `products.yaml` into slots of `locations.yaml`, registry, magnetic attach / detach | `sim.ts`: registry, attach (palm centre ≤ 0.18 m from the product top; two hands: palm middle ≤ 0.15 m from the box centre, palm spacing = width ± 0.03 m), carried products follow the palm / the two-palm basis, basket products ride with the robot | same rules and tolerances |
| `murom_driver`: `/cmd_vel` with 0.6 s watchdog, `/murom/joint_trajectory` linear interpolation, start pose | `sim.ts` `cmdVel` / `sendTraj`, motor velocity limits of the PROTO, v2 travel pose [0.3, 0, 0, -2.2] | same |
| `battery_node`: 0.01 %/s idle, 0.30 %/m, 0.04 %/rad, 0.8 %/s within 6 cm / 0.15 rad of the dock | `sim.ts` battery | same parameters |
| `navigate_server` + `parking.py`: parking corridors, reverse undock with wiggle, Nav2 to the standoff, glide docking 2 cm / 0.03 rad with retreats | `nav.ts`: same controllers and constants (`nav2_params.yaml`: 240 s nav timeout, 60 s fine timeout, 0.12 m/s, 0.4 rad/s) | Nav2 = studio footprint planner (I8) + a port of the Regulated Pure Pursuit law with the yaml parameters (0.4 m/s, lookahead 0.45–0.9 m, approach scaling 0.6 m / 0.08 m/s, rotate-to-heading > 1 rad at 0.7 rad/s, goal tolerance 0.30 m); no costmap / recovery behaviours |
| `pick_place_server` + `arm_kinematics.py`: DLS IK, palm IK, `ik_palm_select` (torso / basket / furniture boxes, clean joint-linear path), stage / pre / grasp, two-hand box grasp, carry poses, basket via pose, `check_place`, 2 attempts, settle 0.4 s, 3.0 / 1.5 s moves, cancel retreats | `kinematics.ts` + `manip.ts` | IK / palm IK / selection = the Python module to 1e-6 rad on the scenario targets (test runs the original with numpy); the 48 random selection seeds are numpy's `default_rng(0)` values |
| `murom_perception` (`recognition.py`, `calibrate.py`): slot search boxes, range-finder back-projection, median HSV + face size, nearest prototype, confidence by margin, empty slots | `perception.ts` on studio renders (`src/vision/sensing.ts` software rasterizer: planar metric depth + shaded colour, head camera 320×240, FOV 1.2 rad horizontal) and studio prototypes (`appearance.ts`, made by `calibrate()` like `ros2 run murom_perception calibrate`) | same output as `recognition.py` on the same frames (72 slot results compared) |
| `murom_fsm/orchestrator.py` (yasmin): mission `full` = PH_SCAN → SCAN → PH_DEPAL → DEPAL → PH_ORDER → ORDER → PH_DOCK → DOCK → FINISH with ERR_* continuation; nested depal / order machines; single tasks; blackboard, retries, HeldObjects / RELEASE_HELD, cancel | `orchestrator.ts`: the machines as an `fsm` DSL document (`missionFsmDsl`) run by `FsmExecutor` (src/ctl/fsm.ts, skills with named outcomes), every state a skill `murom.<state>` | states / outcomes / transitions equal to the yasmin machines of the Python source for 11 task variants (the test imports orchestrator.py with stubbed ROS modules) |

In the app: **File › MURKA examples › Retail store mission (robot-retailer-murka)…** (variant, task, speed; then pick
the MURKA_Simulation folder or one project — the builder takes the variant's own world and YAML files from it) runs the
task in real time on the studio's 3D view; *Stop retail mission* is the cancel.

The FSM document of a task can be opened in the Control tab like any `fsm` model:

```
fsm MUROM mission
outcomes done error
state PH_SCAN initial
  do skill=murom.phase phase=SCAN outcomes=next
  next -> SCAN
machine SCAN
  outcomes done error
  state SCAN
    do skill=murom.scan outcomes=ok,fail
    …
machine DEPAL
  outcomes done error
  state PLAN
    do skill=murom.plan_depal outcomes=found,none_left,error
    found -> NAV_PICK
    none_left -> done
    error -> error
  …
```

## Results (headless, `npx vitest run tests/murka_retail`)

MURKA reference: README "Что проверено" (2026-10-18). `mission_test.py` measures **wall** time; the simulated time of
the Webots run is wall × RTF. The studio time is simulated time (the headless studio run takes 5–12 s of wall time).

### v2 "capsule" — mission `full`

| Stage | Studio (simulated) | Outcome | What happened |
|---|---|---|---|
| SCAN (undock + 6 views) | 159 s | done | reverse glide off the charger, 6 views (robot turned 0.8 rad to the gondola, arms lowered), **24/24 visible slots correct**, 4 empty slots of A3 L1 detected, L2 out of frame (as in Webots) |
| DEPAL | 85 s | done | box_blue_7 (pallet_2 top layer, west column) → bulk_B3_L0_S0 (L1 refused by check_place, as in MURKA) |
| ORDER | 295 s | done | tea_box_1 and juice_pack_1 chosen from the camera inventory, verified by the camera (confidence 1.00), into the basket, two unload trips to packing_S0/S1 |
| DOCK | 55 s | done | glide-docked 2.7 cm / 1.7° from the charger, charging (battery 71.8 %) |
| **Total** | **594 s** | **done** | Webots: 1069 s wall at RTF 0.84 ≈ **898 s** simulated (studio / Webots = 0.66); MuJoCo: 990 s |

### v1-classic — mission `full`

| Stage | Studio (simulated) | Outcome |
|---|---|---|
| SCAN | 82 s | done (24/24; head turned 90°, robot along the aisle) |
| DEPAL | 86 s | done (box_blue_7 → bulk_B3_L0_S0) |
| ORDER | 262 s | done (tea_box_1, juice_pack_1 on packing) |
| DOCK | 56 s | done |
| **Total** | **486–489 s** | Webots: 706 s wall at RTF 0.73 ≈ **515 s** simulated (0.95); MuJoCo: 499 s |

### MURKA_Simulation copies (`murka`, `murka_classic`) — scan + dock

Robot starts at home (-5.5, 3); gondola levels 0.9 / 1.2 / 1.5 m, so all 36 slots are in frame: **36/36 correct**; dock
target reached within 3 cm. (The 5 cm charging pad of these worlds is driven over: the occupancy band starts at 6 cm.)

### Single tasks and cancel (v2)

`undock` (reverse glide onto the standoff line, no fine alignment), `goto <slot>` (≤ 2 cm / 0.03 rad at the approach of
the slot's arm), `home` (leaves the tight parking backwards first), `scan shelf_A2`, `order tea_box coffee_can`
(e2e_test.py), `dock`, `depalletize pallet_1 box_red bulk_B1 2` (box_red_7 → bulk_B1_L0_S0, box_red_8 → S1), and the
cancel_test.py sequence on a fresh store: cancel during the pick approach → `canceled`, hand empty, arm back in the travel
pose; cancel during PLACE_BASKET → `canceled`, product kept in the hand and named in the result; the next order first
puts it into the basket (`RELEASE_HELD`) and completes.

### Collisions (independent monitor)

`sim.ts` checks every tick, against the furniture colliders of the imported world (Webots boundingObjects): arm links as
capsules (radius = largest xy radius of the link meshes), the hand as the wrist / palm mesh vertices thinned to 1 cm
(like MURKA's `test_clearance.py`), carried loads, and the robot body (Webots boundingObjects of MUROM: torso box, wheels);
tolerance 5 mm.

* v2 `full`: arm links 0, loads 0, body 0. **One hand contact (5–8 mm, depending on the glide end pose)**: while gliding
  into the bulk_B3 L0 parking with the box, the fingertips of the inward palms touch the underside of the L1 board
  (1.62 m). Cause in the original: the two-hand carry pose `BOTH_CARRY = (0.26, 1.50)` of pick_place_server.py puts the
  palm centres at 1.50 m with the fingers pointing up to ≈ 1.63 m, and the parking places the box inside the shelf;
  `test_clearance.py` checks manipulation at nominal parking, not the drive-in, and the Webots hand has no
  boundingObject, so the original does not notice it. `runMurkaRetailMission(…, { fixes: { bothCarryZ: 1.46 } })` (off by
  default, to stay faithful) lowers the carry by 4 cm: zero contacts of every kind (tested on `depalletize`).
  The contact depends on the task's glide end pose: 4.9–6.8 mm in `full`, up to 6.5 mm in the scenario example and in
  `scan= order=coffee_can` (independent mesh monitor of the verification, l_palm × bulk_B1 / bulk_B3).
* v1-classic: arm links 0, body 0; the classic tip grasp hangs the product `GRIP_CLEARANCE` under the tip and aims the
  place at the surface itself with a tip IK accepted up to 2 cm, and the joints move linearly from the pre-place to the
  place pose, so the carried product **passes through the packing-table edge** on the way down: its bottom is up to
  ≈ 70 mm below the table top inside the table footprint for about a second (≈ 20 ticks) of PLACE_PACK, ≈ 40 mm for
  the full-run monitor (juice / tea), and rests on the top at the end. This is the original classic scheme
  (pick_place_server.py, the same joint-linear moves); products are kinematic in Webots too, so it passes there as well.
* Basket: the server puts a product where it actually is in the hand over its basket slot (`basket_xy`, as
  pick_place_server.py), so two products in neighbouring slots can overlap by a few mm (6 mm juice × tea in `full`,
  16 mm after a cancel put the juice into basket_0); products are kinematic, as in MURKA.

## Differences to the Webots / ROS runs (limits)

* Time: the studio has no Nav2 behaviour-tree / costmap / recovery overheads, no Nav2 aborts and 3 s retries, no
  camera frame latency beyond 0.2 s per recognition, and joints reach their targets exactly (no PID lag beyond
  `JOINT_TOL`), so the v2 mission is shorter than the Webots run; the classic robot (fewer of these effects) matches.
* Perception runs on the studio's renders (headlight shading, no PBR / textures); prototypes are calibrated on them, as
  MURKA calibrates per simulator. Robot meshes are rendered when loaded (`loadMeshes: true`).
* Physics: base and arms are kinematic (drive model with the Nav2 velocity / acceleration limits; joints with the motor
  velocity limits); products are kinematic like in MURKA (magnetic grasp).
* The MURKA bridge (WebSocket protocol), screenshots / recording services and the ROS interface are not part of this
  module (the ROS 'sim' backend of docs/ros.md serves the same station over rosbridge).

## Files

`src/murka/retail/`: `station.ts` (C1 build, variants), `layout.ts` (YAML), `kinematics.ts` (arm_kinematics port),
`sim.ts` (driver + supervisor + battery + collision monitor), `nav.ts` (navigate_server + RPP), `manip.ts`
(pick_place_server), `perception.ts` (recognition / calibrate), `appearance.ts` (studio prototypes),
`orchestrator.ts` (FSM document + action library), `mission.ts` (C1 run, report, checks), `tasks.ts` (sim-time
coroutines). Tests: `tests/murka_retail_{kinematics,perception,mission,tasks,variants}.test.ts` (MURKA folder needed;
the parity tests also need Python 3 with numpy and PyYAML — skipped visibly otherwise; `MURKA_DIR`, `MURKA_PYTHON`).
