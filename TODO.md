# VerticalBot Studio — TODO / roadmap

Living task list for the studio. Keep it honest: an item is *done* only when it is merged with tests and
documented in `docs/` (Read the Docs). Дорожная карта студии; статус «сделано» ставится только после
тестов и документации.

Legend: `[x]` done · `[~]` partial / best-effort · `[ ]` open

## Status (September 2026)

| | |
|---|---|
| Code | ~30 k lines TypeScript in `src/` + server, Python `robodk` drop-in, Blender add-on |
| Verification | four layers: 542 vitest tests in 62 files (unit, integration, jsdom UI, A/B pairs; 93 % line coverage of `src/`, 7 network tests skipped offline), 7 Playwright end-to-end tests over the production build (`npm run test:e2e`), 54 demo scenarios (`npm run scenarios`), `npm run test:report` → `docs/test-report.md`; GitHub Actions (typecheck, coverage, build, e2e, scenarios, report, Sphinx `-W`) |
| Docs | ~67 Read the Docs pages with screenshots generated from the running app, RoboDK guide map, integration page, scenario results, control-design section with the course demo guides |
| Done this cycle | group-control module (the course on distributed robotic systems as features: consensus / formations / W-MSR, swarms, allocation, grid MAPF, coverage and estimation, group safety with barrier functions, the warehouse homework fleet, games / RL / evolution / CA / fuzzy / resilience; 14 kinds, charts, fleet runtime, scene builder, 8 scenarios, 5 test suites, 11 docs pages); before that: control-design layer (all 17 chapters of the control-of-robotic-complexes course as features: supervisory control, Petri nets, model checking, GR(1), behavior trees, planning, decisions, coordination, scheduling, real-time, safety, V&V; Control tab, DSL, reports with graph views, mission runtime under a supervisor, supervisor export, course examples A / B, 13 scenarios, demo guides); before that: machine-vision stack, navigation & SLAM stack, demo scenarios, rosbridge publishers, API params / webhooks |

### Next up (приоритеты)

0. Control design follow-ups: symbolic (BDD) model checking and synthesis for larger plants, supervisor deployment as a ROS 2 node from the exported Python class, timed automata (UPPAAL-style) and probabilistic model checking (PRISM-style), schedule Gantt view in the Process tab.
1. Hardware validation loop: real camera + arm through the exported perception package (ROS 2 detections → targets), real AMR through the exported Nav2 package; record the first field logs and calibrate the simulated noise models from them.
2. Photorealistic camera simulation so real detectors can run on simulated images (textures, lighting, leaves, motion blur) and depth from the WebGL depth buffer.
3. Open-RMF / Nav2 action bridge for real AMRs; VDA 5050 with a real fleet manager.
4. Packaging (Docker image server + static app, PWA offline bundle) and server authentication.
5. Russian documentation (sphinx-intl).

## 1. RoboDK parity (ядро, паритет с RoboDK)

- [x] Station tree, frames, targets, tools, programs, folders, notes; RoboDK item-type numbering
- [x] Kinematics: DH / modified DH / URDF chains, FK, Jacobian, DLS IK with configuration awareness, <6-DOF relaxation, external axes (combined IK)
- [x] Programs: MoveJ/L/C, speed, rounding, frames/tools, pauses, IO, events, code, calls, threads, wait, mobile moves, joint-path replay
- [x] Simulator: trapezoidal profiles, timeline, attachments, collision sampling, singularity diagnostics, KPIs
- [x] Post processors: 28 built-in + original RoboDK Python posts in the browser (Pyodide) — `Program › Import RoboDK post processors`
- [x] Program importers: KRL, RAPID, LS, URScript, CSV, G-code/NC
- [x] Robot library: 22 built-in + 91-robot online library (ROS-Industrial / vendor URDF packages)
- [x] `.rdk/.robot/.tool`: lossless import through a headless RoboDK behind the server (`rdk2vbs.py`), best-effort binary scan otherwise
- [x] Machining: curve/point follow, NC → milling/cutting/3D-printing programs (feeds, spindle, extruder), RoboDK machining projects transfer
- [x] Calibration: TCP, frames, robot DH identification, ISO 9283, ballbar; cameras; spray deposition; mechanism builder
- [x] API: Robolink/Item (Python drop-in package, C#, C++, MATLAB, browser console), WS/TCP server, drivers UR/ABB RWS/KUKA KVP/ROS 2
- [x] Tutorial station and guide, RoboDK documentation map (feature-by-feature coverage)
- [~] Deep `.rdk` container parsing without RoboDK — needs real sample files (`.rdk`, `.robot`) to go beyond names/poses/meshes
- [ ] RoboDK plug-in interface (C++ `IAppRoboDK`) — not planned; the JS plugin API covers the same hooks
- [ ] More posts on request (Yaskawa MotoPlus, Fanuc KAREL, Stäubli VAL3 advanced, Siemens NX CAM handshake)
- [ ] Accuracy: robot calibration with real tracker drivers (Leica / API / FARO) — only simulated measurements today
- [ ] Mesh/mesh collision everywhere (BVH), self-collision matrices per robot from URDF `<disable_collisions>`

## 2. Visual Components / KUKA.Sim (процессное моделирование)

- [x] Process components (feeder, conveyor, process, buffer, sink), signals, threads, world clock, KPIs
- [~] `.vcmx` / `.vcm` import — meshes + XML metadata; behaviours and `.rsc` geometry are proprietary (need sample files to improve)
- [ ] Component behaviour scripting parity (VC Python API subset) — design a mapping onto the studio plugin API
- [ ] Layout-level export towards VC/KUKA.Sim beyond glTF/URDF (e.g. a `.vcm`-compatible layout skeleton once the schema is confirmed)
- [ ] KUKA.Sim / OfficeLite handshake: round-trip KRL with `$CONFIG`/`.dat` variables verified on a real controller

## 3. Mobile robotics & fleets (мобильные роботы и флот)

- [x] Drive models (diff, ackermann, omni, tracked), A* / coverage / row traversal, pure pursuit, occupancy maps, zones
- [x] Fleet manager: auction allocation, alley reservations, deadlock-free braking, charging, KPIs incl. yield
- [x] VDA 5050 v2 (master + AGV-twin bridge over MQTT), tested against an embedded broker
- [x] Navigation & SLAM stack: catalogue of 21 localization + 7 navigation methods, recommender (platform / environment / sensors / constraints), localization error simulation driving the controller (drift, GNSS outages under canopy, loop closures, scale drift, tracking loss), 2D LiDAR + SLAM map view, Nav2 + SLAM + EKF ROS 2 package export
- [x] Demo scenario per localization method (warehouse loop, greenhouse rail, orchard alleys with GNSS-denied canopy, open-field passes) with RMSE / deviation / lost-event metrics — `npm run scenarios`
- [x] ROS 2 publishing of the simulated stack: `odom_estimate`, `ground_truth`, `scan`, `slam_map`; `navEstimate` through the API
- [ ] Navigation stack: 3D LiDAR ray-casting against scene meshes for SLAM (today: analytic primitives), multi-robot map sharing, Nav2 behaviour trees export, sensor-noise calibration from real logs
- [ ] Field validation of VDA 5050 with a real broker / KUKA Fleet / MiR — needs access to a fleet
- [ ] Open-RMF adapter (fleet adapter API) and ROS 2 Nav2 action bridge for real AMRs
- [ ] Multi-map / elevator / door handling in VDA orders (zones, `zoneSetId`)
- [ ] Battery/energy models per terrain slope and load (agriculture)

## 4. Machine vision — СТЗ (техническое зрение)

- [x] Catalogue: 15 sensors with depth error models, 10 compute targets, 45 models (detection, segmentation, classification, tracking, keypoints, 6D pose, depth, point-cloud nets, grasping, VLM, VLA) with latency per compute class, licences and sources
- [x] Recommender: tasks + environment + modality + compute + working distance + constraints → ranked sensor / compute / model stacks with depth error budget, pipeline latency, warnings
- [x] Pipeline on station cameras: ground-truth capture (fruit, trunks, objects, vehicles, occlusion), simulated detector statistics, ByteTrack, 3D localisation (depth median, mono size prior, LiDAR clusters), grasp / approach targets, follow controller, VLM queries, VLA actions applied to the TCP
- [x] Pluggable real models: ONNX Runtime Web (Ultralytics exports: detect / seg / pose / cls, RT-DETR, YOLO-World), studio server `/vision/infer` (ultralytics / onnxruntime / forwarding), OpenAI-compatible VLM endpoints, VLA policy servers (openpi, OpenVLA, studio JSON), ROS 2 `vision_msgs` via rosbridge
- [x] Point clouds: PCD / PLY import & export, voxel, RANSAC ground, Euclidean clustering with shape classes, trunk slice + row lines, canopy metrics, simulated 3D LiDAR / depth camera
- [x] Vision tab (overlay, bird's-eye cloud, precision / recall / position error vs truth), wizard, camera properties, ROS 2 perception package export (drivers, yolo_ros, hand-eye TF, cloud pipeline, VLM / VLA bridges)
- [x] Demo scenario per task family (stereo picking, mono size prior, segmentation / keypoints / classification, RGB-D bin picking, LiDAR rows, ToF canopy, following, VLM, VLA, open vocabulary); protocol tests with mock inference / VLM / VLA servers and a real onnxruntime-web run
- [x] Publishing: `vision_msgs` detections / 3D detections / targets / `PointCloud2` / `CompressedImage` over rosbridge, `visionLast` through the API, webhooks
- [ ] Photorealistic rendering for real detectors on simulated images (textures, lighting, leaves, motion blur); depth from the WebGL depth buffer instead of ground truth
- [ ] Real perception validated on hardware (ROS 2 detections → targets with a physical camera and arm); noise models calibrated from field data
- [ ] Hand-eye calibration workflow in the UI (collect TCP / camera correspondences, `rigidTransform`, write the static TF)
- [ ] Full 6D orientation from depth (surface normals / PCA of the mask cloud) and in-browser FoundationPose-class models when WebGPU allows
- [ ] Dataset export: rendered images + ground-truth labels (YOLO / COCO) for training and auto-labelling with VLMs

## 5. Agriculture (сельское хозяйство)

- [x] Crop presets, orchard/greenhouse generators, GeoJSON import, missions (harvest, spray, mow, prune, scout, pollinate, weed), fruit detection sim, canopy GNSS-denied zones
- [ ] Terrain elevation (DEM import, slopes in planning and energy)
- [ ] Seasonal/phenology model for yield forecasting; weather windows in mission scheduling
- [ ] Multi-arm harvesting cycle optimisation (fruit assignment between arms, reach clustering)
- [ ] Spray dose maps from canopy volume (vision) → variable-rate spraying missions

## 6. Interoperability (обмен данными)

- [x] URDF/xacro import (+ online package meshes), URDF package export, STL/OBJ/COLLADA/glTF/STEP import
- [x] Blender: animated glTF export, `.vbstation` add-on (import with FK + animation, export meshes/pose logs)
- [x] Integration page: ports, protocols, ROS 2 topics, JSON contracts (inference, VLM, VLA, webhook, API), example nodes (`python/examples/`)
- [ ] Test the Blender add-on inside real Blender 3.6 / 4.x (only the bpy-free core is CI-tested)
- [ ] COLLADA / glTF assets inside `.vbstation` decoded by the add-on (today: STL/OBJ)
- [ ] Import glTF animations from Blender onto robots (retargeting joint curves)
- [ ] USD / OpenUSD export for Omniverse / Isaac Sim

## 6b. Group control — multi-robot systems (управление распределёнными робототехническими системами)

- [x] Graph / consensus core: Laplacian spectrum (Jacobi), λ₂, 1/Δmax, ρ and ε*, topologies, Metropolis weights, r-robustness, formation by offsets, Ji–Egerstedt rendezvous, W-MSR, event-triggered consensus, cooperative transport, potential fields; group simulator with radio model, failures, noise
- [x] Swarm: boids, polarization, Vicsek torus model, PSO lbest, robot PSO with repulsion / noise / forgetting, ACO TSP, firefly, grey wolf, bee colony
- [x] Allocation: greedy vs Hungarian, SSI with best insertion and exhaustive optimum, CBBA (bundle / consensus with release, stale-winner rule), Vickrey second price, contract-net traffic
- [x] Grid MAPF: space-time A*, prioritized planning, first conflict, CBS, action dependency graph execution with delays vs naive
- [x] Coverage / estimation: Voronoi labels, masses / centroids, coverage cost, Lloyd and limited-range Lloyd, information consensus, centralised estimate, covariance intersection
- [x] Group safety: pair CBF, exact planar QP by active sets, decentralised controller, keep-right rule with hysteresis, stale-data constraints, antipodal / crossing scenarios
- [x] Warehouse homework: warehouse model + A*, CBBA on orders with commitment and lost winners, executor FSM, cell reservation (settle / ack / priority / asymmetric detours), fleet simulator with orders, radio range / loss / latency, faults, protective layer, metrics
- [x] Chapters: games (Nash, mixed 2×2, Pareto, best response, fictitious play, potential, replicator, Shapley, core), Q-learning / SARSA / IQL, GA / DE / ES, Wolfram / Life / pheromone, Mamdani fuzzy avoidance, Byzantine OM(1), trust consensus, degradation, switched-system dwell time
- [x] Group missions over configured robots (chapter 3): centralised / decentralised / hybrid architectures with a coordinator, radio model, outages, robot failures, fallback and resynchronisation; closed loop on the station robots through their kinematic controllers (unicycle drive), station zones as goals, no-go zones as obstacles, `des` supervisor gating of the phases, `hybrid` speed modes; comparison report, scenario, tests, guides
- [x] DSL (15 kinds), analysers with charts (lines / paths / grid / bars), templates, 34 course documents, Group menu, Group examples, Run on fleet runtime (6 kinds), warehouse scene builder, 8 demo scenarios, tests (practicum acceptance, chapters, warehouse, documents + runtime), docs section with practicum guides
- [ ] Follow-ups: 3-D consensus / formations with altitude, MAPF on the station's occupancy map instead of a text grid, ROS 2 export of the CBBA / reservation nodes (the homework's `urrts_fleet` package) from a warehouse model, CTDE / QMIX beyond independent learners

## 6a. Control design — course methods (проектирование верхнего уровня управления)

- [x] DES automata: composition, trim, deadlock / livelock traces, Ramadge–Wonham synthesis (Algorithm 3.1), controllability, modular supervisors and non-conflict, observer / observability, twin-plant diagnosability, supervisor table (JSON) + Python runtime
- [x] Petri nets and S³PR: P/T-invariants (Farkas), reachability, bounds, liveness, reversibility, minimal siphons / traps (Commoner), GMEC monitors with resource "thieves", banker, resource ordering, timed simulation, GSPN steady state; performance: bottleneck, saturation, max-plus / resource circuits, Little, Monte Carlo
- [x] Behavior trees (tick semantics, memory, preemption with halt, skill contracts, Monte Carlo FTS, Kripke abstraction), statecharts (hierarchy, regions, history, flattening)
- [x] LTL → Büchi (GPVW) + nested DFS with shortest lasso counterexamples and fairness, CTL marking with witnesses, LTL₃ monitors, SMV-style synchronous models
- [x] GR(1) synthesis (three nested fixpoints, controller extraction, counter-strategy), hybrid modes (hysteresis, dwell time), CBF QP filter
- [x] PDDL (typed, `forall` / `exists` / `imply`, costs, durative) with h_max / h_add / h_FF, A* / GBFS, validation, HTN, STN / STNU (Morris), plan–execute–replan, TAMP helpers; MDP VI / PI, POMDP (α-vectors, QMDP, thresholds), shield
- [x] MRTA (Hungarian, bottleneck, auction, CBBA), MAPF (priority, CBS, TPG, critical sections), job shop (rules, bounds, B&B, tabu, FJSP, right shift, OEE), real-time (RM / RTA, EDF, PCP, latency chains, QoS), reliability (Weibull, Markov availability, FMEA AP, FTA MOCUS, FDIR, ISO 13849 PL, ISO/TS 15066 SSM), V&V (STL robustness, falsification, pairwise, rule of three, Clopper–Pearson, Welch, acceptance, traceability)
- [x] Control tab (models in the tree, DSL editor, reports with tables and SVG graph views, exports), Control menu, properties section, tree badges; mission runtime on a station robot with supervisor gating, monitors and mode machine; course examples A / B; 13 demo scenarios; 66 tests; docs section with demo guides
- [ ] Symbolic (BDD / SAT) engines for plants beyond ~10⁶ product states; timed automata; probabilistic model checking
- [x] Graphical editors for automata and Petri nets synchronised with the DSL text (Diagram / Text views, `layout` lines), inspector with robot-action bindings (programs, targets, zones, signals) and executors that run an automaton / a net on the station
- [x] Graphical behavior-tree editor (tree layout, type menu, reorder / drag to reparent, inspector with bindings, contracts, supervisor events, monitors)
- [x] Undo / redo of diagram and text edits per model (↶ ↷, Ctrl+Z / Ctrl+Y in the diagrams)
- [ ] Deploy the exported supervisor / behavior tree as a ROS 2 node (BehaviorTree.CPP XML export) and drive a real robot through the studio server

## 7. Platform & quality (платформа и качество)

- [x] Vite + TypeScript + three.js app, vitest (157 tests), Playwright smoke, GitHub Actions
- [x] Read the Docs documentation (`docs/`, Sphinx + MyST) — this roadmap is published there
- [x] Screenshots generated from the running app (`studio/scripts/docs-screenshots.mjs`), tutorial station + guide, RoboDK documentation map
- [x] Demo scenarios suite with a generated Markdown report (`npm run scenarios`, Help › Demo scenarios…)
- [ ] Russian translation of the documentation (sphinx-intl) — UI is already RU/EN
- [ ] Undo/redo coverage audit for every dialog; keyboard-only workflow
- [ ] Performance: instanced rendering for orchards > 10k trees, worker-thread simulation, faster scenario dialog load
- [ ] Authentication and multi-user sessions for the server (today: trusted network)
- [ ] Packaging: Docker image for server + static app; Electron/PWA offline bundle
