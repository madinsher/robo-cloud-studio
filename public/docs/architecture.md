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
│   ├ Field/CropRow/Mission     └ posts (compileForPost → files)  │
│   └ Component (VC behaviours)                                   │
└─────────────────────────────────────────────────────────────────┘
            ▲ JSON-RPC (ws://:20500)             ▲ rosbridge (ws://:9090)
   ┌────────┴────────┐                    ┌──────┴──────┐
   │ studio server   │◄── python/robodk   │ ROS 2 graph │
   │ relay/headless  │    (drop-in)       │ (this repo) │
   └─────────────────┘                    └─────────────┘
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
