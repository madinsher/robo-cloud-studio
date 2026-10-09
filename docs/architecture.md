# Architecture

```
┌──────────────────────────── browser ────────────────────────────┐
│  UI (tree / properties / program editor / dashboards / console) │
│        │                                                        │
│  App (commands, undo, files, sim loop) ── SceneRenderer (three) │
│        │                                                        │
│  Station item tree  ◄── Robolink API (JS) ◄── ws-client (relay) │
│   ├ Robot (chain, FK/IK)      ├ ProgramSimulator                │
│   ├ Program/Instruction       ├ FleetManager (per FleetItem)    │
│   ├ MobileRobot/Map/Zone      ├ ProcessSimulator (components)   │
│   ├ Field/CropRow/Mission     ├ posts (compileForPost → files)  │
│   ├ Component (VC behaviours) ├ NavRuntime (estimator, SLAM)    │
│   ├ Camera (visionStack)      ├ VisionRuntime (models, tracker) │
│   └ ControlModelItem (DSL)    └ ControlRuntime (BT + supervisor │
│        analyse() → Report        + monitors + modes, worldHooks)│
│                                  │  publishers (ROS msgs, JSON) │
└──────────────────────────────────┼──────────────────────────────┘
            ▲ JSON-RPC (ws://:20500)  │ rosbridge (ws://:9090)  │ http: inference / VLM / VLA / webhook
   ┌────────┴────────┐          ┌─────┴───────┐            ┌─────┴──────────────┐
   │ studio server   │◄─ python │ ROS 2 graph │            │ model servers,     │
   │ relay/headless  │  robodk  │ Nav2, yolo_ │            │ Ollama / vLLM,     │
   │ drivers, VDA,   │          │ ros, RViz   │            │ openpi, your nodes │
   │ /vision/infer   │          └─────────────┘            └────────────────────┘
   └─────────────────┘
```

## Core concepts

- **Item tree** (`src/core/items`): every entity is an `Item` with a pose relative to its parent. `poseAbs()`
  composes the chain; `Tool.poseAbs()` goes through the robot's flange FK so anything mounted on a tool follows
  the arm, and arms mounted on mobile robots follow the platform.
- **Kinematics** (`src/core/kinematics`): `ChainDef` = joints (origin, axis, optional DH post transform,
  limits, mimic) + links (visuals) + flange. `forwardKinematics` returns link/joint frames; `jacobian` is geometric;
  `inverseKinematics` is damped least squares with step limiting, limit clamping, stagnation exit, deterministic
  restarts, optional position-only / free-tool-Z relaxation.
- **Motion** (`src/core/motion`): trapezoidal profiles; MoveJ synchronises joints on the slowest axis; MoveL/MoveC
  interpolate pose (slerp) and solve IK per sample, detecting unreachable segments and configuration jumps.
  `ProgramSimulator.compile` turns a program into timed steps (trajectories + discrete events) so the timeline can
  seek in both directions; `tick`/`seek` apply joints and attachments.
- **Posts** (`src/posts`): `compileForPost` resolves targets into poses in the active reference frame and joint
  values, producing a flat `PostEvent[]` that each vendor emitter renders.
- **Mobile/fleet** (`src/mobile`, `src/fleet`): 2D state on the ground plane synchronised with the 3D pose;
  `FleetManager.step(dt)` allocates tasks, plans paths on the `MapItem`, follows them with pure pursuit, reserves
  row segments, manages charging and accumulates KPIs.
- **Agri** (`src/agri`): `FieldItem` polygon + `CropParams` → `CropRow`s with `PlantRecord`s (fruit positions,
  ripeness). `planMission` converts a `MissionItem` into fleet tasks (work lines beside rows); `generateHarvestProgram`
  creates arm programs from fruit positions using `poseFromZ` approach frames and IK.
- **Navigation stack** (`src/mobile/navstack.ts`, `navstack_ros.ts`): a catalogue of localization / navigation
  methods with sensor requirements, suitability and error figures; `recommendStacks` ranks them; the chosen
  `NavStackConfig` lives in `robot.params.navStack`. `stepNavRuntime` runs a `LocalizationEstimator` (drift,
  GNSS availability from `gnss_denied` zones, loop closures, scale drift, tracking loss) and an incremental
  `SlamMap` from a simulated 2D LiDAR; the fleet controller steers from the *estimate*, so localization errors
  become path errors. `generateRosNavPackage` writes Nav2 / SLAM / EKF configuration and launch files.
- **Vision stack** (`src/vision`): `stack.ts` holds sensors, compute targets and models with a recommender and the
  per-camera `VisionStackConfig` (`camera.params.visionStack`); `models.ts` defines the `VisionModel` adapter
  interface (simulated, ONNX Runtime Web, HTTP inference, OpenAI-compatible VLM, VLA policy servers, ROS 2 topics)
  and `ByteTracker`; `pipeline.ts` (`VisionRuntime`) captures ground truth / depth / clouds from the station,
  runs the tasks, tracks, localises in 3D with the sensor error model and produces targets, follow commands and
  VLA actions; `pointcloud.ts` and `camera_model.ts` are the geometry libraries; `vision_ros.ts` exports the ROS 2
  perception package.
- **Publishers and scenarios** (`src/ros/publishers.ts`, `src/scenarios`): ROS 2 message builders shared by the
  rosbridge twin, the API summaries (`visionLast`, `navEstimate`) and webhooks; the scenario module builds small
  stations per method, drives them headlessly and scores the result (report → docs).
- **Rendering** (`src/scene`): flat map item → `THREE.Group`, matrices set from `poseAbs()` each frame; robots get
  per-link groups updated from FK; orchards use instanced meshes; maps are canvas textures.

## Collision model

`src/core/collision`: every collidable item yields `Collider`s — capsules for procedural links (mirroring the
renderer), oriented boxes for boxes/short cylinders/mesh bounds, capsules for long cylinders, spheres. Pairs are
tested with analytic distance functions (capsule/capsule, capsule/box, box/box SAT, sphere/…); mesh/mesh pairs can
be confirmed triangle-by-triangle. Same-robot neighbouring links, tools vs their wrist, attached objects and a
robot's base vs the item it is mounted on are excluded. `ProgramSimulator` samples each trajectory (default every
0.1 s) and reports the first colliding sample per instruction; contacts that already exist before motion (robot on
its pedestal) are reported once as warnings and ignored along the trajectory.

## Extending

- New robot: add DH/URDF-style entry in `src/core/items/library.ts` or import a URDF at runtime.
- New post processor: implement `PostProcessor` and `registerPost` (see `src/posts/misc.ts`).
- New item type: extend `Item`, register with `registerItemType`, add a renderer branch in `SceneRenderer.buildItem`
  and a properties section.
- New behaviour component: add a `Behaviour` variant and handle it in `ProcessSimulator.step`.
- New mission type: extend `MissionType` maps in `src/agri/missions.ts`.
- New localization / navigation method: add an entry to `LOCALIZATION_METHODS` / `NAVIGATION_METHODS` (accuracy,
  drift, sensors, software), a config block in `navstack_ros.ts`, and a `NAV_CASES` entry in `src/scenarios`.
- New vision sensor / model / adapter: add to `VISION_SENSORS` / `VISION_MODELS` (with `adapter`), implement
  `VisionModel` in `models.ts` if it is a new execution path, map it in `createModel`, and add a scenario.
- New outgoing protocol: build the message in `src/ros/publishers.ts` and call it from `RosBridge.publishState`
  or the `onOutput` hook of `VisionRuntime`.
- New group-control method: add the algorithm to the matching module in `src/mrs/`, a line to the kind's parser in
  `src/mrs/dsl.ts`, a section in the analyser, a course document in `src/mrs/examples.ts` and a test.
- New control-design method: add the algorithm to the matching module in `src/ctl/` (`des`, `petri`, `perf`,
  `bt`, `temporal`, `gr1`, `hybrid`, `planning`, `mdp`, `mrta`, `sched`, `realtime`, `reliability`, `vv`), a
  parser keyword in `dsl.ts`, a report section in `analysis.ts` (and a `GraphView` if it has a picture), a template
  in `TEMPLATES`, a course example in `examples_dsl.ts`, a scenario in `src/scenarios/control.ts` and a test. A
  new runtime action for behavior trees goes into `stationBindings` / `simulatedBindings` in `runtime.ts`.

## Group-control module

`src/mrs` is the second course as a separate package: pure algorithms per topic (`graph`, `consensus`, `sim`, `swarm`,
`allocation`, `mapf`, `coverage`, `safety`, `warehouse`, `games`, `learning`, `evo`, `ca`, `fuzzy`, `resilience`), a text
DSL (`dsl.ts`, 14 kinds, templates), analysers that produce report sections with charts (`an_practicum.ts`,
`an_theory.ts`, dispatched by `analysis.ts`), the course documents (`examples.ts`) and a fleet runtime (`runtime.ts`)
that drives station `MobileRobot`s inside the world loop and builds warehouse scenes. It plugs into the control-design
layer through `ctl/model.ts` (the kinds join `CONTROL_KINDS` under the group `multi-robot`), `ctl/dsl.ts` (templates,
detection) and `ctl/analysis.ts` (analyser map, `PlotView` in report sections); `ui/plots.ts` renders the charts and
`ui/control_ui.ts` adds *Group examples*, **Run on fleet** and **Build scene**. Demo scenarios: `src/scenarios/group.ts`.

## Control-design layer

`src/ctl` is independent of the renderer and the UI: pure algorithms per chapter of the course, a text DSL
(`dsl.ts`) that turns documents into model objects, `analysis.ts` that turns a document into a `Report`
(sections with level / lines / table / graph, metrics, Markdown), and `runtime.ts` that executes a behavior tree
against `WorldBindings`. `ControlModelItem` (type 111) stores the document in the station tree; the Control tab
(`src/ui/control_ui.ts`) renders reports and SVG graph views and drives `ControlRuntime` from the world loop
(`app.worldHooks`, stepped inside `App.tick` with the fleet and process simulators). The supervisor table format
(`SupervisorTable`) is shared by the runtime, the JSON export and the generated Python class. `graphdoc.ts` is the
document model of the graphical editors (`ui/graph_editor.ts`): automata and Petri nets edited as diagrams and
written back to the DSL with `layout` and `action` lines; `exec.ts` executes them on the station (entry actions,
transition operations) through the host actions of `runtime.ts` (programs, targets, signals) — the bridge between
the control models and the ordinary robot programming of the studio.
