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
cellular automata, fuzzy control, resilience). Reports carry charts; six kinds run on the station robots (**Run on
fleet**). See `docs/group/` on Read the Docs and `src/mrs/`.

## What it does

| Area | Highlights | Docs |
|---|---|---|
| **Industrial OLP (RoboDK class)** | station tree, FK / Jacobian / DLS IK with configuration awareness and < 6-DOF relaxation, external axes, 22 built-in + **91 online robots** (ROS-Industrial / vendor URDFs), MoveJ/L/C programs with IO / events / threads, trapezoidal simulator with timeline, collision checking, singularity diagnostics, **28 post processors + RoboDK's own Python posts running in the browser**, program importers (KRL, RAPID, LS, URScript, CSV, G-code), calibration (TCP, frames, DH identification, ISO 9283, ballbar), machining (curve / point follow, NC → milling / cutting / 3D printing), mechanism builder, spray simulation | Basic Guide, Robots, Programming |
| **RoboDK interoperability** | RoboDK-compatible API (JS console, Python `robodk` drop-in, C#/C++/MATLAB, WS/TCP server on 20500/20501), lossless `.rdk/.robot/.tool` import through a headless RoboDK behind the server, targets / programs / machining projects / joint paths transfer, export of a station as a RoboDK API script | Interop › RoboDK, Reference › RoboDK coverage |
| **Process simulation (Visual Components class)** | feeders, conveyors, machines with failures, buffers, sinks; signals, threads, world clock, KPIs; best-effort `.vcmx` import | Process components |
| **Mobile robots & fleets** | differential / Ackermann / omni / tracked platforms, occupancy maps, A*, coverage and row-traversal planners, pure pursuit, fleet manager (auctions, alley reservations, deadlock-free braking, charging, KPIs), **VDA 5050 v2** master and AGV-twin bridge over MQTT | Mobile robots, Fleet, VDA 5050 |
| **Navigation & SLAM stack** | catalogue of 21 localization methods (2D / 3D LiDAR SLAM, LIO, visual SLAM, VIO, RTK / INS, UWB, tape / QR / reflectors / rail, hybrids, dead reckoning) and 7 navigation methods with a recommender per platform / environment / sensors; localization error simulation (drift, outages under canopy, loop closures, scale drift, tracking loss) that drives the controller; 2D LiDAR and SLAM-map view; **Nav2 + SLAM + EKF ROS 2 package export** | Mobile › Navigation & SLAM stack |
| **Machine-vision stack (СТЗ)** | catalogue of 15 sensors (mono, stereo, RGB-D, ToF, 3D LiDAR, thermal, multispectral, event), 10 compute targets, 45 models (YOLOv8/10/11, RT-DETR, YOLO-World, Grounding DINO, YOLO-seg, FastSAM, SAM 2, Mask2Former, CLIP, ByteTrack / BoT-SORT / OC-SORT, YOLO-pose, FoundationPose / MegaPose / DOPE, Depth Anything, RAFT-Stereo, PCL / Patchwork++ / PointPillars / PointNet++, GraspNet, Florence-2 / PaliGemma / Qwen2.5-VL / LLaVA, OpenVLA / π0 / Octo / GR00T / ACT) with a recommender; the **pipeline runs on the station's cameras**: simulated ground truth with model statistics, or real models through **ONNX Runtime Web**, an inference server, an **OpenAI-compatible VLM**, a **VLA policy server** or ROS 2 `vision_msgs`; tracking, 3D localisation with sensor error models, grasp / approach targets, following, point clouds (PCD / PLY, ground, clusters, crop rows, canopy metrics), ROS 2 perception package export | Programming › Machine vision stack |
| **Control design (course methods)** | the Control tab: 22 model kinds in a small text DSL — automata + Ramadge–Wonham supervisory control (supremal controllable sublanguage, modular non-conflict, observer / observability, twin-plant diagnosability, supervisor table + Python export), Petri nets and S³PR (invariants, reachability, liveness, siphons / traps, GMEC monitor synthesis, banker, timed and GSPN analysis), performance bounds (bottleneck, saturation, max-plus, Little), behavior trees (contracts, preemption, Monte Carlo, Kripke abstraction) and statecharts, LTL / CTL model checking with lasso counterexamples and fairness, GR(1) synthesis with controller / counter-strategy, hybrid modes (hysteresis, dwell time) + CBF filter, PDDL (typed, `forall`, costs) with h_max / h_add / h_FF and HTN, STN / STNU, MDP / POMDP, MRTA (Hungarian, auction, CBBA), MAPF (priority, CBS, TPG), job shop (rules, bounds, B&B, robustness), real-time (RTA, EDF, PCP, latency budgets), reliability & safety (MTTF, FMEA, FTA, ISO 13849 PL, ISO/TS 15066 SSM, FDIR), V&V (STL robustness, falsification, pairwise, Clopper–Pearson, sim-to-real, traceability); **graphical editors** for automata, Petri nets and behavior trees synchronised with the DSL (Diagram / Text views) with inspectors that bind robot programs, targets, zones and signals to states, transitions and action leaves; **mission runtime** executes a behavior tree on a station robot under the synthesised supervisor with LTL₃ monitors and a mode machine, and **Run on station** executes an automaton (entry actions) or a Petri net (transitions as operations) on the real station items; both course examples (A mobile manipulator, B production cell) as ready models with demo guides | Control design |
| **Agriculture** | 15 crop presets, orchard / greenhouse generators, GeoJSON fields, missions (harvest, spray, mow, prune, scout, pollinate, weed, transport, thin, irrigate), harvesting program generator, canopy GNSS-denied zones | Agriculture |
| **Interoperability** | URDF / xacro / STL / OBJ / COLLADA / glTF / STEP import, URDF package and animated glTF export, **Blender add-on** (`blender/`), best-effort Visual Components / KUKA.Sim import | Interop › Blender, ROS, VC |
| **Integration** | rosbridge digital twin (joints, TCP, cmd_vel; navigation estimate / truth / scan / SLAM map; `vision_msgs` detections, targets, point clouds, images), `visionLast` / `navEstimate` readable through the API, webhooks, HTTP contracts for inference / VLM / VLA, drivers (UR, ABB RWS, KUKA KVP, ROS 2) | Developer › Integration |
| **Verification** | 236 vitest tests (kinematics, posts, importers, planners, fleet, VDA 5050 with an embedded MQTT broker, vision adapters with mock servers, a real onnxruntime-web run, every control-design method against the course numbers), Playwright smoke, **45 demo scenarios** (every localization method, every vision task, every control-design method) with published results | Reference › Demo scenarios |

UI in English and Russian (View › Language).

## Quick start

```bash
npm install
npm run dev          # http://localhost:5173  (?demo=orchard | pickplace | tutorial | packing | welding | greenhouse | verticalbot)
npm test             # unit + integration tests
npm run scenarios    # 45 demo scenarios → docs/scenario-results.md
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
from `vertical_robot_model/meshes/` onto the 3D view (or `palletizer_model_pkg/urdf/*.xacro` + meshes).

## Connecting to the outside world

| Path | Port / protocol | Use |
|---|---|---|
| RoboDK-compatible API | ws/http 20500, tcp 20501 (JSON-RPC) | Python / C# / C++ / MATLAB scripts, `getParam('visionLast' / 'navEstimate')` |
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
  src/posts       post processors (KUKA, ABB, Fanuc, UR, Motoman, Stäubli, Doosan, Mecademic, CSV, JSON, RoboDK, ROS 2, Pyodide runner)
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
- The `.rdk` / `.robot` binary containers are proprietary. Lossless import needs a RoboDK installation behind the
  server (`STUDIO_ROBODK_PYTHON`); without it the reader is best-effort. `rdk_export.py` inside RoboDK and the API
  bridge remain the other lossless paths.
- Navigation and vision **simulations model statistics** (drift, outages, loss of tracking; recall vs. object size,
  false positives, box jitter, attribute confusion, sensor depth error), not physics or appearance. They compare
  stacks honestly before hardware exists; they do not certify a stack. Real models, VLM / VLA servers and MQTT brokers
  are verified at the protocol level with mock servers and an embedded broker, not against live services.
- Collision checking is analytic-primitive based with optional triangle checks; mesh/mesh everywhere is on the roadmap.
- ROS 2 integration is via rosbridge websocket; native DDS is out of scope for a browser. The server has no
  authentication — run it on a trusted network.

License: MIT.

# robo-cloud-studio
