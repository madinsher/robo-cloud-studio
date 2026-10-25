# Agricultural robotics workflow

1. **Field**: Agriculture › Create field / orchard. Pick a crop preset (apple spindle, pear 2D trellis, cherry V,
   citrus hedgerow, grape, strawberry/tomato/cucumber gutters, blueberry, olive, almond, kiwi pergola, row crop),
   dimensions, row spacing/heading, headland, ripe fraction. Or import a GeoJSON boundary (File › Open) and
   regenerate rows on it. The generator creates `CropRow` items with plants and fruit (inside the canopy envelope:
   cone for spindles, ellipsoid otherwise; no per-plant cap, almond ≈ 800 fruit), one row per inside interval of
   every row line (concave fields are filled), a navigation map and headland zones.
2. **Fleet**: Mobile & Fleet › Create fleet: platforms (tracked harvest platform, orchard sprayer, Ackermann tractor,
   AMR, scout rover, UAV), allocation strategy, charging zone. Platforms can carry arms (drag a library robot onto a
   mobile robot in the tree) and modular equipment (see *Equipment*).
3. **Mission**: Agriculture › Create mission (harvest, spray, mow, prune, scout, weed, pollinate, transport, thin,
   irrigate), select rows (`1-5,8`), work speed, both sides. Planning produces one fleet task per row side (one per
   alley for centred implements) with the alley as traffic segment. For arm missions the work direction is chosen so
   the arm faces the row (re-oriented for the robot that gets the task). Car-like robots get a warning when the
   headland is shorter than the Ω / fish-tail turn their turning radius needs.
4. **Run**: start the world clock. The Fleet tab shows robots, batteries, tasks and KPIs; the process models below
   decide what happens on the rows.
5. **Arm programs**: Robot › Fruit picking program (`generateHarvestProgram`): for the arm at its *current* base pose
   (targets in a frame that moves with the carrier, regenerated per stop), reachable fruit (measured reach, not the
   catalogue sum of link lengths) become approach / pick / retreat targets; every configuration and joint move
   (2° samples, including the way home) is checked with the simulator's collision checker against the arm itself,
   the carrier and the rigid crop (trunks). Targets come from camera detections when given (`matchDetections`), else
   from the crop model — flagged *GROUND TRUTH* in the program. Fruit are removed only when the simulator closes the
   gripper on them (`bindHarvestEvents`), never at generation.
6. **Mobile programs**: `Navigate` / `Follow path` instructions really drive the base in the program timeline: the
   path is planned on the station (map + trees, footprint, drive model) and the drive model + tracker are simulated
   for the trajectory and its duration (contract I2, `src/mobile/program_motion.ts`).
7. **Export**: missions/tasks through the API, programs via post processors, the scene as `.vbstation` (fleet clock,
   task phases, robot paths and process state are saved: a station saved mid-mission resumes), fields as GeoJSON.

## What the simulation models

**Crop collision world** (`src/agri/collision_world.ts`): every plant is a trunk cylinder (r = 75 mm for trees) from
the ground to the canopy top and a canopy disk of the plant's own width from `canopyBase` up. Robots lower than the
canopy base pass under it (kiwi pergola), taller ones are blocked. The same trunks are rigid colliders for arm
programs; foliage is soft.

**Drive models** (`src/mobile/controller.ts`): differential (wheel saturation), tracked / skid steer (track slip, ICR
expansion, turning losses), omni (vx, vy, ω), legged (lateral speed limit), Ackermann bicycle (steering angle with
angle and rate limits, reverse, yaw-rate limit), aerial (x, y, z, yaw, take-off to cruise altitude, hover / climb
power). Pose paths carry a direction per point (forward / reverse / rotate in place); the tracker stops at cusps.

**Planning** (`src/mobile/planner.ts`, `hybrid_astar.ts`, `reeds_shepp.ts`, `navigate.ts`): footprint-aware grid A*
with oriented string pulling (turn-then-drive, no corner cutting at alley mouths) for differential / tracked /
legged / omni, Hybrid A* with Reeds–Shepp shots for Ackermann (curvature ≤ 1/R_min, three-point turns), headland
U / Ω / fish-tail turns sized to R_min, precise docking (`planAndFollow(…, { precise | reverse })`: straight final
leg along the goal heading, ≤ 20 mm / 1°). `planAndFollow(station, mobile, goal)` (contract I8) returns the
simulated trajectory — used by control-model `goto`.

**Traffic management** (`src/fleet/fleet.ts`): robots stop when their path, sampled by arc length, would bring their
footprint within the safety gap of another robot's footprint or of its stopping footprint (along that robot's own
path); a robot that is already too close may only move away. Alleys (and zones of kind `lane`) are reserved for
transit as well as work; robots wait a standoff before the mouth so the robot leaving can get out; a wait-for-graph
resolver lets a robot back out of the way (straight retreat, else a planned parking pose off the other robot's route)
— never creep. Idle robots left in an alley clear it. `nogo` zones are never entered, `speed_limit` zones cap the speed
(with braking before entry). Every step the manager measures robot–tree and robot–robot contacts (KPI `contacts`).

**Allocation**: `nearest` (greedy straight-line, one task per idle robot), `auction` (sequential auction on marginal
completion time including queued work), `hungarian` (optimal one-to-one), `cbba` (bundles with consensus),
`contract_net` (announce / bid / award rounds), `round_robin`; custom allocators via `registerAllocator`. Costs see
segment conflicts (two robots are not sent into the two sides of one alley at once) and capabilities (a robot that
lacks them is only considered if it can be reconfigured at a depot).

**Process models** (`src/agri/process.ts`, registered for every FleetManager):

| Mission | Model |
|---|---|
| harvest | base stops placed so the fruit of the row side are within the arm's reach; the base stands still while the arm picks (`secondsPerFruit` each); each fruit is removed and booked at its pick time; bin capacity with unload trips; unreachable fruit stay on the tree and are counted |
| thin | removes unripe fruit above `keepFruitPerPlant`; books no yield |
| spray / irrigate | tree-row-volume dose (canopy height × width × `doseLPerM3`, default 0.07 L/m³) or litres per plant; tank / water capacity with refill trips; dose recorded per plant; spraying treats diseased plants |
| mow / weed | swath area and coverage of the inter-rows; weeding stops per plant |
| prune / pollinate | time per plant from the canopy volume (or `secondsPerPlant`), plants stamped |
| scout | noisy health observations stored on the mission, diseased plants, optional follow-up tasks (`followUp: 'spray'`) |
| transport | load, deliver to `meta.dropoff`, unload |

**Equipment** (`src/agri/equipment.ts`, MURKA agromarca): modules with a mount slot, mass, continuous power,
capabilities, resources and per-action energy (harvester arm, fruit crate, 400 L sprayer, scout mast, pruner, mower
deck, cargo bed, UAV gimbal camera / mini sprayer); recipes per role; `applyEquipment` recomputes capabilities,
resources and the energy model E(t+Δt) = E(t) − (ε_base + Σ ε_module)·Δt − c_action with traction cost × (1 + W_eq/W_max).
When no robot can do a task, an idle robot drives to the nearest `depot` zone (capacity = slots, `reconfigSeconds`
param, default 180 s), waits for a slot, is re-equipped and continues (KPI `process.equipment.reconfigurations`).
UAVs fly above ground traffic (no waiting for ground robots, no alley reservations) and only collide with obstacles
reaching their altitude.

**Process components** (`src/vc`) share the station's single signal table with robot programs and control models:
a program can wait on a conveyor sensor or stop a conveyor.

## Greenhouses, rails, palletizing, per-robot controllers

- **Greenhouse layout** (`buildGreenhouse`, src/agri/greenhouse.ts; the greenhouse demo): a hanging gutter per row
  (with wires), a pair of Ø 51 mm heating pipes 550 mm apart in every path, a concrete corridor across the row heads,
  and a rail network. **Pipe-rail trolleys** (`useRails`: navigation method `rail_1d`) route on that network
  (src/mobile/rails.ts): on a rail they drive along its axis only (forward or backward, never turning on it), they turn
  and pass each other on the corridor (right-hand lanes); a turnaround on the same rail goes through the corridor.
  Arms mounted on the trolley centreline (turret) work both rows of a path without turning.
- **Navigation method → planner / controller** (Navigation & SLAM stack of a robot): Nav2 Smac Hybrid-A* + RPP =
  curvature-limited Hybrid A* for every drive (no rotation in place) and regulated pure pursuit (speed × r / 0.9 m on
  tighter curves, anticipated over the braking distance); NavFn + DWB / Smac 2D + MPPI = 2-D footprint grid planner;
  line following = the fixed straight route (blocked = no path); rail = the rail network; no stack = default.
- **Arm motion during fleet harvest**: mission setting `animateArm` (or `harvestAnimation.enabled`): one IK solve per
  fruit, linear joint motion that reaches the fruit at its pick time, back to the travel pose between stops.
- **Per-robot FSM** (src/fleet/fleet_fsm.ts, `attachFleetFsm`): a studio `fsm` document is a fleet robot's own
  controller — it gets a task only in a state with `skill=fleet_execute`, charges when its FSM decides
  (`fleet_charge`), waits (`wait=`); the trace is its lifecycle with the task ids.
- **MuRKA contract net allocator** (`fleet.allocation = 'murka'`): the article's cost (1), ≤ 2 contracts per robot,
  battery ≥ 15 %, reconfiguration bids through the depot (src/fleet/murka_alloc.ts); MURKA reproduction in
  docs/murka-agro.md.
- **Palletizing in the packing line**: the KR16 is a robot-driven transfer component (`motion`): it picks every crate at
  the conveyor end, the crate rides on the gripper, it stacks the crate on its pallet slot; the pallet goes to the truck
  when full (`releaseWhenFull`); the equivalent program "Palletize" can be posted for the robot. The VerticalBot demo
  stacks 4 cartons from a table onto the pallet.

## Limits (honest list)

- The fleet process models time the arm (base stops, seconds per fruit) unless `animateArm` is on; full joint-level
  picking with collision checks runs through the harvest *program* (step 5).
- Two harvesters in narrow 3 m orchards with short headlands can deadlock in the FleetManager (they keep re-planning
  around each other); MURKA's legacy apple garden is harvested with one platform (docs/murka-agro.md).
- Greenhouse benches, trolley lifts and rail switching hardware are not modelled (the trolley changes rails on the
  corridor).

## Verification

`npx vitest run tests/agri_ tests/mobile_` — among others: the orchard demo as the app loads it finishes all 12 tasks
with 0 steps of robot–tree or robot–robot contact (independent geometry check), books exactly the fruit it removed
and keeps the arm within 0.8 × reach of the row it harvests (`agri_orchard_demo`); head-on robots in one alley /
lane, allocators, save–load mid-mission (`mobile_traffic`); Ackermann alley-to-alley turns and wall detours
(`mobile_kinematics`); docking and program motion (`mobile_navigate`); harvest / spray programs and the detect → pick
loop (`agri_programs`); process models and generators (`agri_processes`); equipment and depot reconfiguration
(`agri_equipment`); group control on Ackermann robots (`mobile_group`); component ↔ program signals
(`agri_vc_signals`); greenhouse rails (`agri_greenhouse`); palletizer cells (`agri_vc_palletizer`); navigation method →
planner / controller (`mobile_navmethod`); per-robot FSM controllers (`mobile_fleet_fsm`).

Models are parametric and deterministic (seeded), so scenarios are reproducible for what-if studies.
