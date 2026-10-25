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
An Ackermann base is referenced to its **rear axle** (`kin.rearAxle`, body x; default −wheelBase/2, i.e. axles
symmetric about the item origin; the Webots importer takes the proto's rear wheels, −1.095 m on the UTP-G): the bicycle
model, the turning-radius limit, the Hybrid A* arcs, Reeds–Shepp joins, headland turns and the path tracker hold at
the rear axle, the item origin / footprint centre follows as a rigid body (it slips by atan(a/R) in a turn, about 14° on
the orchard Sprayer at R = 3.6 m), and the 3-D view draws the wheels on those axles. Measured from consecutive poses the
rear wheels have no side slip (< 0.5°; `mobile_ackermann_rear_axle`). The tracker previews curvature switches for the
steering-rate lag (the transition is centred on the switch and at most 100 mm long, braking-distance aware), and steers
at standstill at a cusp.

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
— never creep. Idle robots left in an alley clear it; a robot leaving its alley for a service trip (unloading, refill)
keeps it reserved until it is out, so on a one-way pipe rail no robot is let in head-on behind it. `nogo` zones are
never entered, `speed_limit` zones cap the speed (with braking before entry). Every step the manager measures robot–tree and robot–robot contacts (KPI `contacts`).

**Allocation**: `nearest` (greedy straight-line, one task per idle robot), `auction` (sequential auction on marginal
completion time including queued work), `hungarian` (optimal one-to-one), `cbba` (bundles with consensus),
`contract_net` (announce / bid / award rounds), `round_robin`; custom allocators via `registerAllocator`. Costs see
segment conflicts (two robots are not sent into the two sides of one alley at once) and capabilities (a robot that
lacks them is only considered if it can be reconfigured at a depot).

**Process models** (`src/agri/process.ts`, registered for every FleetManager):

| Mission | Model |
|---|---|
| harvest | base stops placed so the fruit of the row side are within the arm's reach (an arm on a lift reaches every height of its stroke); the base stands still while the arm picks (`secondsPerFruit` each); the arm moves to every fruit it picks (see *Arm motion*); each fruit is removed and booked at its pick time; bin capacity with unload trips; unreachable fruit stay on the tree and are counted |
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
- **Arm motion during fleet harvest / thinning** (on by default; mission setting `animateArm: false` or
  `harvestAnimation.enabled = false` keeps the timing-only model for batch runs): per fruit a collision-free IK
  configuration (approach from the arm base, horizontal or tilted up to ±80°, every closed-form solution, the travel
  pose's configuration family first; tested with the simulator's checker against the arm itself, its carrier and lift
  column, the other robots and the station's objects — `armCollisionCheck: false` skips the test) and a pre-approach pose 250 mm
  back along the gripper axis; the arm retracts from the last fruit, transfers to the next pre-approach and approaches,
  linear joint motion per leg timed by the joint speed limits (a pick waits for the arm when it needs longer, the base
  stands still meanwhile), and reaches the fruit at its pick time; between the stops of a row it stays at the retract
  pose, it returns to its travel pose at the end of the row and for unloading trips. A fruit with no free configuration
  stays on the plant (KPI `process.harvest.armUnreachable`).
- **Lifts** (`mountArmOnLift(arm, stroke)`, `addLiftColumn(arm, stages)`, src/agri/harvest.ts): the arm base rises on a
  telescopic column under it (params `liftStroke`, `liftBaseZ`; the column stages are scene objects of the carrier, so
  they are drawn and collision-checked, and nothing of the lift is above the arm base); stops and IK use the whole
  stroke, a stop's fruit are picked in height order (up at one stop, down at the next) and the lift moves before the
  joints going up, after them going down. The orchard demo's UR10e arms ride a 2.1 m column (arm base 1.4–3.5 m for
  the 0.6–4.3 m spindles), the greenhouse trolleys' UR5e a 2.5 m column (0.5–3.0 m for the 0.3–3.2 m crop).
- **Mission result**: a harvest mission's progress is the crop — fruit picked / (picked + ripe fruit still on the
  worked row sides) — and when every task has finished it is `done` only if no ripe fruit is left (thinning: no excess
  fruit out of reach; no failed task), else `incomplete`, with a `Result:` warning that says how many fruit stayed on the
  plants and why (out of the arms' reach, missed by the camera). Planning already warns when ripe fruit are out of the
  arm's reach from the alleys (`MissionPlan.unreachableFruit`).
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

- The fleet arm motion tests the pick and pre-approach configurations for collisions, not every intermediate position
  of the joint-space legs between them: an independent check of every leg of the demos at 4° samples (simulator
  checker, trunks included) finds a graze in 83 of 47 631 legs in the orchard (0.2 %: link vs lift column / deck / own
  link) and 317 of 43 475 in the greenhouse (0.7 %: forearm vs the trolley body with the lift low); picking with every
  joint move checked runs through the harvest *program* (step 5). The gripper is not modelled closing on the fruit (it
  is removed at its pick time with the TCP on it).
- Two harvesters in narrow 3 m orchards with short headlands can deadlock in the FleetManager (they keep re-planning
  around each other); MURKA's legacy apple garden is harvested with one platform (docs/murka-agro.md).
- Greenhouse benches and rail switching hardware are not modelled (the trolley changes rails on the corridor). With
  unloading trips every few dozen fruit (bins of a few kg; the demo's 400 kg bin unloads twice per house) two trolleys
  can still block each other on the corridor near the parking end: the wait-for-graph resolver does not back one out
  there.

## Verification

`npx vitest run tests/agri_ tests/mobile_` — among others: the orchard demo as the app loads it finishes all 12 tasks
with 0 steps of robot–tree or robot–robot contact (independent geometry check), books exactly the fruit it removed,
picks every ripe fruit of rows 1–4 with the TCP on each fruit at its pick time (mission `done`, progress 1) and keeps
the arm within 0.8 × reach of the row it harvests (`agri_orchard_demo`); arm motion, joint speed limits, collision-free
pick configurations, the `incomplete` result of a fixed-mount arm and the opt-out (`agri_harvest_arm`); head-on robots in one alley /
lane, allocators, save–load mid-mission (`mobile_traffic`); Ackermann alley-to-alley turns and wall detours
(`mobile_kinematics`); Ackermann rear axle without side slip in the bicycle model, Hybrid A* / headland turns, Navigate
docking, the orchard demo Sprayer and the S1 UTP-G platforms (`mobile_ackermann_rear_axle`); docking and program motion (`mobile_navigate`); harvest / spray programs and the detect → pick
loop (`agri_programs`); process models and generators (`agri_processes`); equipment and depot reconfiguration
(`agri_equipment`); group control on Ackermann robots (`mobile_group`); component ↔ program signals
(`agri_vc_signals`); greenhouse rails (`agri_greenhouse`); palletizer cells (`agri_vc_palletizer`); navigation method →
planner / controller (`mobile_navmethod`); per-robot FSM controllers (`mobile_fleet_fsm`).

Models are parametric and deterministic (seeded), so scenarios are reproducible for what-if studies.
