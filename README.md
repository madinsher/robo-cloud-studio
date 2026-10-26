# VerticalBot Studio

Browser-native robot simulation, offline programming and perception / navigation engineering platform in the
class of **RoboDK** and **Visual Components**, extended to **mobile robotics, fleet management and agricultural
robotics** (orchards, vineyards, greenhouses, field crops and the surrounding logistics). No install: it runs in
the browser; a small Node server adds the RoboDK-compatible API, robot drivers, VDA 5050, vision inference and
headless batch use.

Part of the `vertical_bot_ros` repository — the ROS 2 packages in the repo root (vertical robot, palletizer) can be
imported directly (URDF/xacro + STL) and driven through the ROS 2 post processor or the rosbridge connection.

Documentation: **https://vertical-bot-ros.readthedocs.io** (built from `../docs/`). Roadmap and status: `TODO.md`.

## Group control (multi-robot systems)

The **Group** menu and the *Group control* kinds of the Control tab implement the course on distributed robotic
systems: consensus / formations on communication graphs, swarms, task allocation, grid MAPF, coverage, group safety
with barrier functions, the warehouse fleet of the homework and the chapter examples (games, learning, evolution,
cellular automata, fuzzy control, resilience). Reports carry charts; seven kinds (mission, consensus, swarm, coverage,
safety, grid MAPF, warehouse) run on the station robots (**Run on fleet**) through each robot's own kinematic controller
(speed, yaw-rate and acceleration limits; no teleports). See `docs/group/` on Read the Docs and `src/mrs/`.

## What it does

| Area | Highlights | Docs |
|---|---|---|
| **Industrial OLP (RoboDK class)** | station tree, FK equal to RoboDK SolveFK for every Library robot (joint senses, base/flange offsets, Fanuc/Comau couplings, deltas, 7-axis), closed-form IK (spherical-wrist / UR-type 6R, delta) with RoboDK configuration flags plus a DLS fallback, synchronised external axes (rails, gantries, turntables; docs/kinematics.md), 22 built-in + **91 online robots** (ROS-Industrial / vendor URDFs), MoveJ/L/C programs with IO / events / threads, station runtime (concurrent programs on one clock, run-time waits / IF, script calls, play to the first error) and RoboDK motion semantics (blending at constant path speed, MoveJ IK on the pose, frame-relative targets, joint limits, RoboDK's wrist-singularity and Comau J3 rules; RoboDK's own `Item.Update()` cycle-time model — every Library program RoboDK rates valid within ±10 %, joints at every move end within 0.5°, all 57 Library stations checked against RoboDK 5.9.4 run program by program: docs/simulation.md, docs/robodk-library-sim.md), collision checking with link meshes and the RoboDK collision map, singularity diagnostics, **post processors equal to RoboDK's own posts token for token (27 RoboDK posts, 153 ground-truth cases incl. 45 Library programs and joint targets; docs/posts.md); station → code equal to RoboDK 5.9.4's MakeProgram for 91 of 167 Library program files (programs of other robots / mechanisms called only, frames / tools as RoboDK sends them, joint targets, RoboDK configuration flags; the rest: IK branch details, Fanuc R30iA, Motoman pulses, machining-regenerated programs) + RoboDK's original posts run on the studio server (compiled ones) or in the browser (source posts, Pyodide)** — with `?server=` they are listed in the Export dialog, and an API client's `MakeProgram` can use them (the native KUKA_KRC2 post through the official `robodk` package equals RoboDK's own output, e2e), control flow in vendor syntax, a ROS 2 package post (FollowJointTrajectory, colcon-built in the tests), program importers (KRL .src/.dat, RAPID, LS, URScript, RoboDK CSV, G-code, APT / CL data), calibration (TCP, frames, DH identification, ISO 9283, ballbar), machining (curve / point follow, NC → milling / cutting / 3D printing), mechanism builder, spray simulation | Basic Guide, Robots, Programming |
| **RoboDK interoperability** | RoboDK-compatible API (JS console, Python `robodk` drop-in with the official constants/signatures and legacy `from robolink import *`, the **official RoboDK clients unmodified over RoboDK's binary protocol**, C#/C++/MATLAB, WS/TCP server on 20500/20501 relayed to the browser; station Python programs run on CPython (server) or Pyodide in the browser (real Pyodide e2e on Library stations 01.a / 06.a / 07.a); `docs/api.md`), native `.rdk/.robot/.tool/.sld` reader checked against RoboDK 5.9.4 itself on all 96 Library files (tree, poses, robot FK < 0.001 mm, targets, programs by item reference, Python programs, curves, collision map; `docs/robodk-library-compat.md`), optional re-save through a headless RoboDK behind the server, export of a station as a RoboDK API script | Interop › RoboDK, Reference › RoboDK coverage |
| **Process simulation (Visual Components class)** | feeders, conveyors, machines with failures, buffers, sinks; signals, threads, world clock, KPIs; best-effort `.vcmx` import | Process components |
| **Mobile robots & fleets** | drive models differential / tracked (slip) / omni / legged / Ackermann bicycle (steering limits, reverse) / aerial; footprint-aware grid A*, Hybrid A* + Reeds–Shepp for car-like robots, headland turns sized to R_min, precise docking, `Navigate` / `Follow path` program instructions that really drive the base; fleet manager (nearest / auction / Hungarian / CBBA / contract-net / MuRKA allocation, per-robot FSM controllers, the navigation method of a robot's stack selects planner and controller (Smac Hybrid + RPP, NavFn, fixed routes, rails), alley and lane reservations, wait-and-yield deadlock resolution — measured: no robot–robot or robot–tree contact in the orchard demo —, no-go and speed zones, charging, depot reconfiguration of modular equipment, persisted mid-mission state, KPIs), **VDA 5050 v2** master and AGV-twin bridge over MQTT | Mobile robots, Fleet, VDA 5050 |
| **Navigation & SLAM stack** | catalogue of 21 localization methods (2D / 3D LiDAR SLAM, LIO, visual SLAM, VIO, RTK / INS, UWB, tape / QR / reflectors / rail, hybrids, dead reckoning) and 7 navigation methods with a recommender per platform / environment / sensors; localization error simulation (drift, outages under canopy, loop closures, scale drift, tracking loss) that drives the controller; 2D LiDAR and SLAM-map view; **Nav2 + SLAM + EKF ROS 2 package export** | Mobile › Navigation & SLAM stack |
| **Machine-vision stack (СТЗ)** | catalogue of 15 sensors (mono, stereo, RGB-D, ToF, 3D LiDAR, thermal, multispectral, event), 10 compute targets, 45 models (YOLOv8/10/11, RT-DETR, YOLO-World, Grounding DINO, YOLO-seg, FastSAM, SAM 2, Mask2Former, CLIP, ByteTrack / BoT-SORT / OC-SORT, YOLO-pose, FoundationPose / MegaPose / DOPE, Depth Anything, RAFT-Stereo, PCL / Patchwork++ / PointPillars / PointNet++, GraspNet, Florence-2 / PaliGemma / Qwen2.5-VL / LLaVA, OpenVLA / π0 / Octo / GR00T / ACT) with a recommender; the **pipeline runs on the station's cameras** with one camera model (RoboDK vertical / Webots horizontal FOV) and pixel-true renders (WebGL or software: metric depth + instance IDs, so robots, meshes and same-class parts occlude): ground truth with model statistics, or real models through **ONNX Runtime Web** (bundled WASM), an inference server, an **OpenAI-compatible VLM**, a **VLA policy server** or ROS 2 `vision_msgs` / `yolo_msgs` (explicit errors when not configured); tracking, 3D localisation from the depth image (face, yaw, truncation-aware centre), grasp / approach targets checked by IK + collision, `VisionDetect("<camera>")` in robot programs (`VISION_FOUND` for IF), RoboDK `Cam2D_*` (PNG / grey32 snapshots), hand-eye calibration, YOLO / COCO dataset export, point clouds (PCD / PLY, ground, clusters, crop rows, canopy metrics), ROS 2 camera streams (Image, CameraInfo, 32FC1 depth, TF) and perception package export (docs/vision.md) | Programming › Machine vision stack |
| **Control design (course methods)** | the Control tab: 22 model kinds in a small text DSL — automata + Ramadge–Wonham supervisory control (supremal controllable sublanguage, modular non-conflict, observer / observability, twin-plant diagnosability, supervisor table + Python export), Petri nets and S³PR (invariants, reachability, liveness, siphons / traps, GMEC monitor synthesis, banker, timed and GSPN analysis), performance bounds (bottleneck, saturation, max-plus, Little), behavior trees (contracts, preemption, Monte Carlo, Kripke abstraction) and statecharts, LTL / CTL model checking with lasso counterexamples and fairness, GR(1) synthesis with controller / counter-strategy, hybrid modes (hysteresis, dwell time) + CBF filter, PDDL (typed, `forall`, costs) with h_max / h_add / h_FF and HTN, STN / STNU, MDP / POMDP, MRTA (Hungarian, auction, CBBA), MAPF (priority, CBS, TPG), job shop (rules, bounds, B&B, robustness), real-time (RTA, EDF, PCP, latency budgets), reliability & safety (MTTF, FMEA, FTA, ISO 13849 PL, ISO/TS 15066 SSM, FDIR), V&V (STL robustness, falsification, pairwise, Clopper–Pearson, sim-to-real, traceability); **graphical editors** for automata, Petri nets, executable FSMs and behavior trees synchronised with the DSL (Diagram / Text views) with inspectors that bind robot programs, targets, zones and signals to states, transitions and action leaves; **Run on station** executes automata (entry actions, guards, sensors, fail outcomes, observer-based supervisor), Petri nets (timed / GSPN / immediate / capacity / guards / input events, deadlock vs final-marking verdict) and **yasmin-style hierarchical FSMs** (outcomes, nested machines, blackboard decisions, timeouts, cancel), behavior trees on arm-only or mobile stations — robot programs run concurrently on the station runtime, one action per robot; **code generation** (docs/control.md): RoboDK station script (robolink, same trace as the studio), per-robot vendor programs with IO handshakes through the posts, IEC 61131-3 ST + PLCopen SFC with I/O map, ROS 2 packages (rclpy supervisor node, yasmin FSM, BehaviorTree.CPP v4 with skill stubs, launch; colcon-built and run in ROS 2 Jazzy by the tests), PNML / SCXML / NuSMV / UPPAAL; course examples A (youBot) and B (production cell) as runnable stations | Control design |
| **Agriculture** | 15 crop presets, orchard generator (concave fields, canopy envelopes, pergolas), trees as collision geometry (trunks + canopies with canopy base), GeoJSON fields, missions with process models (harvest with base stops and picking time, thin, spray dose + tank refills, mow coverage, prune workload, scout observations + follow-up tasks, irrigate, transport), mobile-manipulation harvest programs (collision-checked against carrier and trees, fruit removed when the gripper closes, targets from detections), MURKA-style modular equipment / recipes / depot / configuration-dependent energy model, canopy GNSS-denied zones; greenhouse layout (gutters, heating-pipe rails, corridor) with rail-constrained pipe-rail trolleys; palletizer cells driven by the process flow; **MURKA agromarca reproduced**: the article experiments exp1..exp5 equal the Python ground truth bit for bit (CPython PRNG / hypot / statistics, SciPy LSAP ports), benchmark S1 (survey → harvest → crate logistics) with role FSMs, Hungarian / contract-net / hybrid decisions and zero contacts on the imported orchard_demo.wbt, the legacy Webots gardens harvested by studio fleets (docs/murka-agro.md) | Agriculture |
| **Interoperability** | **Webots `.wbt` / `.proto` import** (JavaScript template PROTOs, offline standard PROTOs, boundingObject colliders, robots split into a mobile base + one arm per joint chain with Webots motor names, cameras / lidar; every shape of 43 worlds (MURKA + Webots samples with standard robots) matches Webots R2025a's own pose dump within 0.5 mm; MURKA Mission B replayed like the Webots supervisor; docs/webots.md; the **MURKA retail robot mission** (МУРОМ v1/v2: undock, camera inventory 24/24, two-hand depalletizing, verified order picking, docking) runs in the studio's own simulator with the original FSM, navigation, IK and perception algorithms — docs/murka-retail.md), URDF / xacro (ROS 2 xacro semantics, ROS package / workspace folders, SRDF) / STL / OBJ+MTL / COLLADA (materials, textures) / glTF / STEP import, URDF package (kept names, inertials — estimated from the geometry for studio-native robots —, ros2_control, launch + SRDF), **MoveIt 2 config export** (mock-hardware bring-up plans and executes in ROS 2 Jazzy) and animated glTF export, **Blender add-on** (`blender/`), best-effort Visual Components / KUKA.Sim import | Interop › Blender, ROS, VC |
| **Integration** | rosbridge digital twin — `sim` mode is a webots_ros2-compatible simulator backend (/clock, /joint_states by URDF name, /robot_description, /tf, odom, imu, scan, camera images; /cmd_vel and JointTrajectory commands) and, on the MURKA retail station, **a drop-in replacement of the MURKA Webots backend** (the full interfaces.md §1–2 set incl. head camera bgra8 / depth 32FC1, lidar, /murom/sim/attach|detach, objects; headless Node backend + CLI): MURKA's own smoke_test.py passes and its FSM + Nav2 undock / dock the studio robot (docs/ros.md) plus mirrors (joints, TCP; navigation estimate / truth / scan / SLAM map; `vision_msgs` detections, targets, point clouds, images), `visionLast` / `navEstimate` readable through the API, webhooks, HTTP contracts for inference / VLM / VLA, drivers (UR, ABB RWS, KUKA KVP, ROS 2) | Developer › Integration |
| **Verification** | 2226 vitest tests in 204 files (last full run on Windows: 2211 pass, 15 skipped — opt-in WSL / ROS 2, network and sweep tests; see `docs/test-report.md`): RoboDK 5.9.4 ground truth for the Library import, kinematics, simulation and 153 post cases + station → code against RoboDK's MakeProgram, the official `robodk` package over the binary protocol, station Python programs on CPython, ROS 2 Jazzy colcon builds in WSL, VDA 5050 with an embedded MQTT broker, vision adapters with mock servers, every control-design method against the course numbers; **browser end-to-end on the production build** (system Chrome): every RoboDK Library station (57) opened and its main program played = the headless runtime, station scripts in real Pyodide, the studio server driven by the official `robodk` package (RoboDK's native posts equal RoboDK's own output), MURKA retail / orchard examples from real folder picks, a UI sweep of every menu; **54 demo scenarios** (every localization method, every vision task, every control-design method) with published results | Reference › Demo scenarios, `docs/test-report.md` |

UI in English and Russian (View › Language).

## Quick start

```bash
npm install
npm run dev          # http://localhost:5173  (?demo=orchard | pickplace | tutorial | packing | welding | greenhouse | verticalbot)
npm test             # unit + integration tests
npm run test:e2e     # browser end-to-end (after npm run build; system Chrome; RoboDK Library / MURKA used when present)
npm run scenarios    # 54 demo scenarios → docs/scenario-results.md
npm run build        # static build in dist/
npm run server       # RoboDK-compatible API server: ws/http :20500, tcp :20501 (+ drivers, VDA 5050, /vision/infer)
```

Typical first session: *Help › Quick start* or the documentation tutorial (`?demo=tutorial`). To see the stacks:
*Help › Demo scenarios…* → *Load into the studio* (navigation scenarios open the Navigation tab with truth vs
estimate; vision scenarios open the Vision tab with detections on the camera image; control scenarios open the
Control tab with the course models). *Control › Course examples…* adds the worked examples of the control course.

Drive the browser session from Python (RoboDK API):

```bash
npm run server                                   # in one terminal
# open http://localhost:5173/?server=ws://localhost:20500 in the browser
PYTHONPATH=python python python/examples/hello_studio.py
python python/examples/robolink_poll_vision_nav.py --camera "Camera 1" --robot "Harvest platform 1"
```

Import this repository's robots: drag `vertical_robot_model/urdf/vertical_robot.urdf` together with the STL files
from `vertical_robot_model/meshes/` onto the 3D view (or `palletizer_model_pkg/urdf/*.xacro` + meshes). Meshes
dropped after the URDF replace its placeholders. A whole ROS package folder or Webots project (`.wbt` / `.proto`)
can be dropped too, or opened with `File › Open folder…`.

## Connecting to the outside world

| Path | Port / protocol | Use |
|---|---|---|
| RoboDK-compatible API | ws/http 20500, tcp 20501 (JSON-RPC), RoboDK binary protocol on `STUDIO_RDK_PORT` | Python / C# / C++ / MATLAB scripts, `getParam('visionLast' / 'navEstimate')` |
| rosbridge | ws 9090 | ROS 2 twin, navigation and perception topics, real robot feedback |
| VDA 5050 | MQTT 1883 / 8883 | KUKA Fleet, MiR, any VDA 5050 master or vehicles |
| Inference / VLM / VLA | http (8500 example, 11434 Ollama, 8000 vLLM / openpi) | real models behind the vision stack |
| Webhook | http POST to your URL | custom nodes without ROS |
| Robot drivers | UR 30001–30003, KUKA KVP 7000, ABB RWS 80/443, ROS 2 9090 | online programming |

Details, message layouts and example nodes (`python/examples/`): documentation page *Developer › Integration*.

## Layout

```
studio/
  src/core        pose math, item tree, kinematics (FK/IK/DH), motion, program simulator, collision, calibration, robot library
  src/io          station file, URDF/xacro, meshes (STL/OBJ/DAE/glTF/STEP), RoboDK (.rdk, targets, scripts), programs, containers, exports
  src/posts       post processors (RoboDK-equivalent KUKA, ABB, Fanuc, UR/URP, Motoman, Stäubli, Kawasaki, Comau, Denso, Epson, Nachi, Mitsubishi, Doosan, Techman, Dobot, HCR, JAKA, AUBO, Mecademic, CSV; JSON, RoboDK API, ROS 2 package; native RoboDK posts via the server, source posts in Pyodide)
  src/vc          Visual-Components-style process components and simulator
  src/mobile      mobile robot items, maps, planners, controllers, navigation & SLAM stack (catalogue, recommender, estimator, ROS 2 export)
  src/fleet       fleet manager (tasks, allocation, traffic, charging, KPIs), VDA 5050
  src/agri        geo, fields/rows/missions, orchard generator, mission planners, harvest program generator, fruit detection sim
  src/vision      machine-vision stack: catalogue & recommender, model adapters, pipeline, point clouds, camera model, ROS 2 export
  src/ctl         control design: DES / supervisory control, Petri nets, performance, behavior trees, temporal logic, GR(1), hybrid, planning, MDP/POMDP, MRTA/MAPF, scheduling, real-time, reliability, V&V, DSL parsers, analysis reports, mission runtime
  src/scenarios   demo scenarios (navigation per method, vision per task, control design per course method) and the report generator
  src/api         RoboDK-compatible JS API + JSON-RPC + browser bridge
  src/ros         rosbridge client and ROS 2 message publishers (navigation, perception)
  src/scene       three.js renderer & assets
  src/ui          panels, dialogs, menu, tabs (Program, Simulation, Fleet, Process, Navigation, Vision, Control, Camera, Console, Log)
  server/         WebSocket/HTTP/TCP server (relay + headless), drivers, VDA 5050 service, vision inference endpoint
  python/         robodk drop-in package, rdk_export.py / rdk2vbs.py, vision_infer.py, examples (ROS 2 consumer, webhook sink, inference server, polling)
  blender/        Blender add-on (.vbstation import/export, program animation)
  scripts/        Playwright smoke and documentation screenshots (docs-screenshots.mjs, docs-screenshots-control.mjs)
  tests/          vitest suites (unit, integration with mock servers / embedded broker, demo scenarios)
  docs/           developer notes included in the Read the Docs build (architecture, RoboDK compatibility, formats, scenario results)
```

## Status and honest limits

- Built-in library robots not marked *(official DH)* use representative geometry; use the online library or import
  the vendor URDF for exact kinematics.
- RoboDK files are read natively and compared with RoboDK 5.9.4 for every Library file (`docs/robodk-library-compat.md`,
  format in `docs/robodk-file-format.md`); the import report lists what is not reproduced (machining project settings,
  Comau parallelogram rods / delta rods / 4-bar gripper links are static — counterbalances of KUKA / ABB / Fanuc arms
  follow J2 like RoboDK —, camera and notes contents). Files from other RoboDK
  versions can be re-saved by a RoboDK installed behind the server (`POST /convert/rdk`, `python/rdk2vbs.py`).
- Navigation and vision **simulations model statistics** (drift, outages, loss of tracking; recall vs. object size,
  false positives, box jitter, attribute confusion, sensor depth error), not physics or appearance (vision geometry —
  depth, masks, occlusion — is rendered pixel-true; images are flat-shaded, not photorealistic). They compare
  stacks honestly before hardware exists; they do not certify a stack. Real models, VLM / VLA servers and MQTT brokers
  are verified at the protocol level with mock servers and an embedded broker, not against live services.
- Collision checking is triangle-exact (BVH) for meshes when the station's assets are loaded and uses primitives / capsules otherwise; RoboDK's own collision results on the Library are not reproduced yet (docs/simulation.md).
- ROS 2 integration is via rosbridge websocket; native DDS is out of scope for a browser.
- The studio server runs code for its clients (station Python programs, RoboDK posts, the whole RoboDK API). It
  listens on 127.0.0.1 only (`STUDIO_HOST=0.0.0.0` opts in to other hosts, like RoboDK's "allow remote API
  connections"), serves browser pages only from local origins (`STUDIO_ALLOWED_ORIGINS` adds others; foreign web pages
  get 403, WebSocket upgrades included), checks the Host header against DNS rebinding and takes an optional shared
  secret `STUDIO_TOKEN` (docs/api.md). Never expose it to an untrusted network.

License: MIT.

# robo-cloud-studio
