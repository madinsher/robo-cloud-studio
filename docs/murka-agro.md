# MURKA agromarca with the studio (orchards, benchmark S1, article experiments)

The agricultural part of MURKA (`MURKA_Simulation/simulation_ws/src/agromarca`: ROS 2 + Webots worlds + the headless
experiments of the article) reproduced with the studio's own models and simulator. Entry points (contract C2,
`src/murka/agro/index.ts`, also re-exported by `src/murka/index.ts` for the app's MURKA menu):

| Function | What it does |
|---|---|
| `buildMurkaOrchardStation({ world, root \| files \| source, assets, fruitSeed, fruitPerTree })` | imports `orchard_demo.wbt`, `murca_apple_garden.wbt` or `murca_garden.wbt` (T6 importer), turns the trees into fields / rows with measured crop parameters and fruit, configures the platforms (Mission B roles, modular equipment, aerial UTP-A), adds the 2-slot depot, collection-point and charging zones |
| `runMissionB(station, opts)` | the garden_supervisor Mission B (T6 port of `scripted_fleet.py`) on the station |
| `addHarvestFleet(result, opts)` | a studio harvest fleet + mission on an imported garden (FleetManager, agri process models) |
| `runS1Benchmark(station, { level, controller, seed, … })` | benchmark scenario S1 «scouting → harvest → crate logistics» with the role automata as studio `fsm` documents |
| `runAgroExperiment('exp1' … 'exp5', { runs, populationOrder, … })` | the article experiments; KPI JSON in the schema of `murca_experiment/artifacts/experiments/*.json` (`article_plots.py` can plot it) |

`root` is the MURKA_Simulation folder (Node / server; `MURKA_DIR` overrides the default path of the owner's machine);
in the browser pass the dropped `files` of the `murca_garden_ws` (or `murca_webots`) folder.

## 1. Article experiments exp1..exp5 — equal to the Python ground truth

`murca_experiment.article_experiments` is pure Python (no physics: EXPERIMENT_SCOPE.md §1). The studio runs the same
logic on its own models:

- allocation: `src/fleet/murka_alloc.ts` — the article's cost (1) `c_ij = w1·d + w2·(B − b) + w3·(1 − match)`,
  Hungarian baseline on the 1e9-padded matrix with `src/fleet/lsap.ts` (a line-by-line port of SciPy's
  `linear_sum_assignment`, including its tie rules), the MuRKA contract net (tasks announced by priority, ≤ 2
  contracts per robot, battery ≥ 15 %, RECONFIGURE bid for exactly one missing capability: +35, −5 % battery), hybrid,
  load σ, greedy set-cover fleet composition (exp1); the same contract net is the FleetManager allocator `murka`;
- energy: `src/agri/equipment.ts` — `E(t+Δt) = E(t) − ε_total·Δt − c_action`, `ε_total = ε_base + Σ ε_module`,
  `c_drive = c_base·(1 + W_eq / W_max)` (`energyStep`, `routeEnergy` = its closed form, `driveCost`,
  `profileOfEquipment` for the studio equipment catalogue);
- missions: `src/murka/agro/missions.ts` (presets A / B / C, random missions, orchard scenario of agent.md §8);
- numerics: `src/murka/agro/pycompat.ts` — CPython's `random.Random` (MT19937, `init_by_array`, `random`, `uniform`,
  `randint`, `shuffle`, `sample`), `math.hypot` of CPython ≥ 3.10, `statistics.mean` / `pstdev` (exact rationals),
  `numpy.std` (pairwise sum).

### Ground truth

| Set | Origin | Interpreter |
|---|---|---|
| `tests/fixtures/murka_agro/article_py310/` | the article's published artifacts (`article/replication/artifacts/experiments`, `all --runs 20`) | CPython 3.10, random PYTHONHASHSEED |
| `tests/fixtures/murka_agro/py312_hashseed0/` | a fresh run of a copy of `murca_experiment` (`gt_run.sh`) | CPython 3.12.3, NumPy 2.5.3, SciPy 1.18.1, PYTHONHASHSEED=0 |

**Finding (reproducibility of the article).** exp2 and exp4 depend on PYTHONHASHSEED: `generate_random_mission`
samples extra capabilities from `list(set(ALL_CAPABILITIES) - caps)`, and the order of a Python set of strings depends
on the hash seed. The published numbers are reproduced by the order `harvest, manipulation, cargo_carry, prune, spray`
(found by trying the 120 orders: exactly one matches); `runAgroExperiment` takes the order as `populationOrder`
(`ARTICLE_POPULATION_ORDER` default, `PY312_HASHSEED0_ORDER`). exp1, exp3, exp5 do not depend on it. CPython 3.7 (the
RoboDK interpreter) differs from 3.10+ by 1 ulp in `math.hypot` (exp3 `per_robot_distance`) and in `statistics.pstdev`.

### Comparison (20 runs; every deterministic field of every record is identical — `Object.is` — to both ground truths)

Aggregates, Python (article) → studio:

| exp1 preset | avg platforms used | avg coverage % | avg redundancy |
|---|---|---|---|
| A | 1 → 1 | 100 → 100 | 0 → 0 |
| B | 2 → 2 | 100 → 100 | 0 → 0 |
| C | 2 → 2 | 85.71428571 → 85.71428571 | 0 → 0 |

| exp2 n×m, algorithm | avg total cost | avg feasible fraction | avg load σ | avg reconfigurations |
|---|---|---|---|---|
| 4×4 Hungarian | 36.24824564 → 36.24824564 | 0.6625 → 0.6625 | 0.3564582562 → same | 0 → 0 |
| 4×4 MuRKA | 71.72581605 → 71.72581605 | 0.925 → 0.925 | 0.7702881334 → same | 0.7 → 0.7 |
| 4×4 hybrid | 75.00591142 → 75.00591142 | 0.925 → 0.925 | 0.5910075241 → same | 0.7 → 0.7 |
| 5×5 Hungarian | 45.38493044 → 45.38493044 | 0.65 → 0.65 | 0.3939387691 → same | 0 → 0 |
| 5×5 MuRKA | 110.9847056 → 110.9847056 | 0.99 → 0.99 | 0.8216294906 → same | 1.3 → 1.3 |
| 5×5 hybrid | 107.5017665 → 107.5017665 | 0.99 → 0.99 | 0.7787146304 → same | 1.1 → 1.1 |
| 6×6 Hungarian | 49.02985623 → 49.02985623 | 0.65 → 0.65 | 0.4067313074 → same | 0 → 0 |
| 6×6 MuRKA | 124.3098653 → 124.3098653 | 0.9916666667 → same | 0.8425093489 → same | 1.25 → 1.25 |
| 6×6 hybrid | 114.3892768 → 114.3892768 | 1 → 1 | 0.7766196756 → same | 0.8 → 0.8 |
| recovery (mean recovered fraction) | Hungarian 0.2 → 0.2 | MuRKA 1 → 1 | | |

(all nine n×m combinations are equal; the table shows three)

| exp3 mode | avg coverage % | avg reconfigurations | avg total energy | avg missed tasks |
|---|---|---|---|---|
| static | 50 → 50 | 0 → 0 | 8.621597076 → same | 6 → 6 |
| MuRKA | 73.33333333 → same | 3 → 3 | 33.64924136 → same | 4 → 4 |

| exp4 fleet size | avg duration s | avg total energy | avg contracts | avg parallelism |
|---|---|---|---|---|
| 2 | 39 → 39 | 18.69480113 → same | 3.75 → 3.75 | 1.9 → 1.9 |
| 4 | 40 → 40 | 33.47535910 → same | 7.85 → 7.85 | 3.925 → 3.925 |
| 6 | 40 → 40 | 46.11939181 → same | 12 → 12 | 6 → 6 |
| 8 | 40 → 40 | 64.69279648 → same | 16 → 16 | 8 → 8 |
| 10 | 40 → 40 | 73.60610944 → same | 19.95 → 19.95 | 9.975 → 9.975 |

| exp5 recipe (500 m route) | avg total energy | σ total energy | avg energy / m |
|---|---|---|---|
| scout_ground | 50.29014511 → same | 0.7387659858 → same | 0.1005802902 → same |
| harvester | 95.69071531 → same | 2.077476608 → same | 0.1913814306 → same |
| sprayer | 115.8590632 → same | 2.636410696 → same | 0.2317181264 → same |

Excluded: wall-clock fields (`*_time_ms`, `wallclock_s`, `*_recovery_ms`).

## 2. Orchards (orchard_demo, legacy gardens) and Mission B

- `orchard_demo.wbt`: 4 sector fields, 8 rows of 3 trees on the 3 m grid + the single tree of sector 1. Tree profile
  measured on the mesh: trunk = the boundingObject cylinder (r 40 mm), crown z ∈ [1.0–1.1; 3.5–3.6] m, r ≤ 0.55 m — what
  the world file states (pallet.dae scaled 0.45 × 0.45 × 1). 40 seeded fruit per tree in the crown shell (no fruit
  geometry in the world). Platforms: UTP-G-01 harvester (arm + crate), G-02 scout mast, G-03 sprayer (400 L), G-04
  logistics (cargo bed), UTP-A-01 aerial (5 m/s); depot zone over the two depot slots (2 slots, 180 s).
- Mission B through `runMissionB` ends at the poses of the real Python replay (1 cm / 0.01 rad), no errors.
- `murca_apple_garden.wbt`: 25 trees (one `DEF TREE_SHAPE` + 24 `USE`; the round-1 report's 26 was a miscount), 5 apples
  each (125, from the red spheres), sphere crowns z ∈ [2; 5] m, trunks r 150 mm. **The studio harvest mission runs
  there**: one 1.9 m tracked platform (below the crown base) with a UR20 harvests all 125 apples, KPI = apples removed,
  no contact (FleetManager, agri process models).
- `murca_garden.wbt`: 7 bare sphere crowns (r 1.2 m from z 0.3 m, no trunk) with seeded fruit: every ripe fruit is picked
  or reported unreachable from the 1.6 m aisles, no contact.
- Fixes found on the way: a harvest stop at the very end of a work path was never reached (the work phase ended first:
  apples of the first / last tree on one side stayed on the tree) — stops are now placed ≥ 50 mm before the path end;
  single-tree rows get the field's row heading.

## 3. Benchmark S1 «scouting → harvest → crate logistics»

`article/BENCHMARK_SCENARIO.md` specifies S1 (§11 of it plans a Python model, which does not exist in
murca_experiment), so the studio's run is checked against the scenario's analytical expectations.

**Automata.** MissionFSM (IDLE → SURVEY → PLAN → EXECUTE → RETURN → done / aborted at T_max = 2 h), Drone FSM
(Wait → Start → Takeoff → SurveyLeg{FlyLeg, TurnNextLeg} → ReportMap → ReturnHome → Land → End), Harvester FSM
(… GotoWaypoints{FollowWaypoints, ReachedGoal} → AlignRow → HarvestLoop{PickTree, WaitSwap, AdvanceTree} → RowDone …),
Logistics FSM (Idle → GotoHarvester → SwapBox → GotoCP → Unload → Idle …) are `fsm` documents (`S1_FSMS`,
`src/murka/agro/s1_fsm.ts`) — editable, checkable and exportable in the Control tab — run by the ctl `FsmExecutor`; the
`skill=s1_*` states are bound to the robots. Every robot keeps a logical clock: a skill starts at the robot's time (or
at the time of the event it waits for) and ends exactly start + duration, so the event times do not depend on the
executor step (§9 «дискретно-событийно»).

**World and motion** (`src/murka/agro/s1_world.ts`, on the imported `orchard_demo.wbt`):

- rows harvested from their outer lane (row line ± 1.5 m = the centre of the 3 m aisle, where MURKA measured 0.18 m
  clearance); the south lane of sector 3 touches the collection-point pad, so that row uses the inner aisle; one stop
  per tree (platform abreast of the trunk); lanes entered from the open corridor between the blocks, left by reversing;
- UTP-G legs: Hybrid A* + Reeds–Shepp with the platform's turning radius 2.19 m / tan 0.3 = 7.08 m (cusps, no turning
  in place) **at the rear axle** — body x −1.095 m, the proto's rear wheels; the platform origin is the wheelbase
  midpoint and swings on 7.16 m —, exact outlines of the static colliders, crown discs, collider-less obstacles
  (collection-point pad, depot shelf / modules) as forbidden polygons against the footprint at the origin; parking
  bays reached by planning the time-reversed path; constant rear-axle speeds (harvester 0.6 m/s, 0.4 m/s along rows;
  logist 1.0 m/s), the poses interpolated along circular arcs between the planner's samples, so the rear wheels roll
  without side slip (< 0.5° between 0.1 s samples, cusps excepted; `tests/mobile_ackermann_rear_axle.test.ts`); a robot
  starts a leg only when it meets no committed motion of another robot (otherwise it waits, or plans around a standing
  robot);
- drone: boustrophedon over the trees' bounding box (+ crown radius) at 5 m and 5 m/s, swath 10 m; takeoff / landing at
  1 m/s;
- deviations from the scenario text (the world was re-laid out on 2026-10-01, after the scenario was written): the
  collection point is the world's `collection_point_1` (−8.4, −9) (the text's (−12, −13) touches the equipment depot
  now); the base is each platform's spawn pose; group: UTP-A-01, harvesters UTP-G-01 and UTP-G-03, logist UTP-G-04
  (UTP-G-02's spawn is boxed in by the crate station for a 7 m turning radius; it stays parked as an obstacle).

**Decisions.** D1 rows → harvesters: Hungarian (SciPy-exact LSAP on cost (1) from the current poses, recomputed at every
`row_done`), contract net (MuRKA: announce / bid / award, ≤ 2 contracts per harvester, re-announced when a slot frees,
the failed robot's contracts first), hybrid; D2 swap requests → logists: Hungarian over {requests} × {idle logists} vs
contracts with score `E_ratio − 0.1·n_contracts`; D3 (level C): global recomputation vs re-contracting.
Levels: A nominal; B sector 2 yields 2 × the survey estimate; C harvester 1 fails at 50 % of its second row (its crate
is lost, the rest of the row is harvested from the other side of the row); D = B's yields with D4: the Hungarian
baseline keeps fixed roles, the contract net gives a RECONFIGURE contract to a harvester that has no row left while
the other still harvests (harvester automaton Released → Reconfigure → outcome `reconfigured`; the platform drives to
a depot slot, 180 s module swap, then runs the Logistics automaton).

**Level D finding.** With the three ground platforms of S1 there is never a free platform before the end: the first
harvester runs out of rows at ≈ 1900–2200 s, reaches the depot ≈ 90 s later and is a logist 180 s after that, when the
last swap requests have already been served (seeds 0–2: makespan D = B for every controller, the new logist delivers
no box). The value of modularity claimed for S1-D therefore needs a reserve platform or an earlier trigger (trading
harvest capacity for logistics capacity); the studio reproduces the mechanism, not a gain.

**Checks** (`tests/murka_agro_s1.test.ts`, seed 0; documented tolerance: event times exact to 1e-9 s, kg to 1e-3):
zero robot–trunk / robot–crown / robot–static / robot–robot / robot–pad contacts (3-D colliders of the world, crown
radial profile against the chassis collision boxes, replay every 0.25 s); curvature radius ≥ 7.08 m at the rear axle and
no rotation in place; speeds of §2 (displacement of the rear axle per 0.1 s); survey time = takeoff + boustrophedon length / 5 m/s; boxes = the §4.2 fill rule over the seeded
yields; kg delivered = kg on the trees (A, B); swap 45 s and unload 30 s exactly; WaitSwap of the event log = time in the
WaitSwap state of the logical FSM trace, ≥ 45 s per box; makespan ≥ the single-logist bound
`t(first box_full) + Σ (45 + 30 + 2·d_min/v_L)`; the Hungarian first assignment is the brute-force optimum of cost (1);
≤ 2 contracts per harvester; B has more boxes than A; C: lost kg = the crate at the failure, contract-net reaction ≤
Hungarian reaction. Event log: `result.eventsJsonl()` (events.jsonl lines).

**Results** (10 seeds per level and controller, median [IQR], all runs completed, zero contacts of any kind in all 90
at the default 0.5 s replay step; smallest chassis–crown clearance 21 mm; re-run 2026-10-25 after the rear-axle fix):

| Level / controller | makespan s | WaitSwap s (% of makespan) | boxes | completeness % | logist path m | decision messages / matrix cells | reaction to the failure s |
|---|---|---|---|---|---|---|---|
| A Hungarian | 1818 [1715–1924] | 1287 [1096–1386] (71) | 12 [11–13] | 100 | 559 [514–611] | 49 cells · 20 messages | — |
| A contract net | 1880 [1805–1990] | 1240 [1026–1486] (67) | 12 [11–13] | 100 | 571 [526–620] | 64 [62–66] messages | — |
| A hybrid | 1883 [1756–1956] | 1284 [1110–1498] (68) | 12 [11–13] | 100 | 559 [515–612] | 58 [56–60] messages + 16 cells | — |
| B Hungarian | 2304 [2271–2330] | 1559 [1537–1640] (68) | 15 [14–16] | 100 | 699 [671–731] | 52 cells · 23 messages | — |
| B contract net | 2274 [2235–2344] | 1681 [1631–1807] (74) | 15 [14–16] | 100 | 710 [683–751] | 73 [71–75] messages | — |
| B hybrid | 2195 [2089–2301] | 1558 [1466–1730] (71) | 15 [14–16] | 100 | 701 [672–740] | 67 [65–69] messages + 16 cells | — |
| C Hungarian | 2486 [2440–2631] | 963 [791–1035] (38) | 11.5 [10–12] | 94 | 540 [495–596] | 53 cells · 20.5 messages | 1543 [1476–1651] |
| C contract net | 2687 [2559–2809] | 948 [842–1150] (36) | 11.5 [11–13] | 94 | 544 [510–589] | 68.5 [67–72] messages | 291 [108–300] |
| C hybrid | 2573 [2508–2633] | 952 [790–1050] (36) | 11.5 [11–12] | 94 | 550 [507–568] | 62.5 [61–64] messages + 16 cells | 42 [34–229] |

Decision messages = `metrics.comm.messages`: every message of the decision protocol — an announcement to each eligible
bidder (a harvester that is failed or at its 2-contract limit is not addressed), each bid, each award / assignment (the
Hungarian assignments too) and every retransmission (none here: `msgLoss` = 0); matrix cells = `metrics.comm.matrixCells`,
Σ free robots × open tasks over the Hungarian decisions. `tests/murka_agro_s1_table.test.ts` re-runs the 90 runs and
checks the makespan and this column against the table.

Why the table changed on 2026-10-25: the earlier runs referenced the UTP-G bicycle model to the platform origin (the
wheelbase midpoint), so the rear wheels slid sideways 8–9° in every turn (V3 F4). With the radius limit and the leg speed
at the rear axle the transit paths are different curves (the origin swings wider, the rear axle cuts inside) and the
legs are timed by the rear-axle arc length; distances in the table are rear-axle (odometry) distances. Individual runs
move by a few tens of seconds (A / Hungarian / seed 0: makespan 2116.9 → 2091.0 s), medians by ≤ 1.6 % (makespan) and
≤ 5 % (logist path, B); the decision column is unchanged, the conclusions below hold.

Reading: with one logist the logistics dominate (two thirds of the makespan are harvester idle time in WaitSwap, the
logist cycle ≈ travel + 45 s + travel + 30 s per box); the decision mechanism changes the makespan by a few percent; the
contract net re-awards the failed harvester's work far sooner (reaction 0.3 k s vs 1.5 k s) but in this layout finishes
later. Battery (§2: 100 u, 0.2 u/m + 0.5 u/kg) is reported, not enforced: a harvester picking ~125 kg and the logist
driving ~600 m exceed 100 u — the scenario's battery model is not consistent with its yields (no charging in S1).

Run time ≈ 1–15 s per run (Node, contacts included; plans are cached per station, the 90 runs on one station take
5.5 min).

**In the app** (File › MURKA examples › Orchard: benchmark S1…, levels A–D): the run is computed in a module Web Worker
(`src/murka/agro/s1_worker.ts`, `s1_remote.ts`) on a copy of the open station — its serialized item tree and the meshes
it references (the crown mesh the tree profile is measured on); the progress pill shows the mission time, then the
contact replay, and the event log streams into the Log while it computes; the result comes back as plain data (the
legs of the timeline, metrics, events, traces), the JSON is downloaded and the run plays on the world clock
(`result.timeline.apply(t)` each world step). The worker's run equals the in-page run (metrics, events, FSM traces,
contacts, timeline poses; `tests/murka_agro_s1_worker.test.ts`, which also drives the worker's message protocol).
Without module workers (Node, jsdom, `?worker=off`) the run is computed on the page thread as before.
Chrome check (2026-10-25, production build, Intel Iris Xe through ANGLE D3D11, `murca_garden_ws` picked, level A,
Hungarian): a second run on the open station (seed 1) took 9.8–13.2 s with **no long task** (PerformanceObserver
`longtask`, > 50 ms) and page round-trips ≤ 19 ms; with `?worker=off` the same run is one 8.0 s long task. The first
run after picking the folder still has two 0.7–2.1 s long tasks in its first 2.2 s — the first 3-D frames of the
just-imported orchard (`renderer.frame` measured at the same times), present with and without the worker — and the
import before it blocks 2–3.5 s; after those frames the page answers in ~10 ms and the camera orbits with the mouse
while the worker computes. In SwiftShader (software WebGL) every frame of the orchard view is a ~1 s long task, so the check
needs a GPU.

**Message model** (S1 params `msgLatency`, `msgLoss`, `msgTimeout`): every decision message takes the latency; a lost
announcement or bid drops that bidder from the round (re-announced after the timeout), lost assignments are
retransmitted; `metrics.comm` counts messages, losses and retransmissions. The ctl group FSM contract net has the same
model (`GroupFsmOptions.network`) and a bid hook (`bid`, e.g. the MuRKA cost (1)). Checked in
`tests/murka_agro_network.test.ts` (assignment arrival = decision + latency; 30 % loss still completes every row).

**KPI exporter** (src/murka/agro/kpi.ts): `murkaKpiFromS1`, `murkaKpiFromFleet` (any FleetManager run) and
`murkaKpiArtifact` give records / artifacts with the keys of the Python model's exp3 records (`coverage_percent`,
`reconfigurations_count`, `total_energy` = 0.2 u/m + 5 u per reconfiguration, …) so `article_plots.py` can plot studio
runs; measured quantities are kept as `studio_*` fields.

## 4. Not done / limits

- 30 seeds per point (the table uses 10), a Python S1 model (does not exist upstream); level D only with the trigger
  described above.
- FleetManager with two harvesters in the 3 m apple-garden aisles deadlocks in the west headland (robots keep re-planning
  around each other); the legacy-garden harvest runs with one platform.
- `murca_garden.wbt` tree names (`tree_S1R1_01`) are not split into sectors by the semantic upgrade (one field, rows by
  line).

## 5. How to run

```
npx vitest run tests/murka_agro_experiments.test.ts          # exp1..exp5 = Python ground truth (3 s, no MURKA needed)
npx vitest run tests/murka_agro_orchard.test.ts tests/murka_agro_s1.test.ts   # need the MURKA folder (MURKA_DIR)
```
Regenerate the ground truth: `tests/fixtures/murka_agro/README.md`.
