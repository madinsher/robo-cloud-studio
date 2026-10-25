# Simulation: station runtime, motion planning, collisions

Source: `src/core/motion/` (runtime.ts, planner.ts, simulator.ts, trajectory.ts, mobile_hook.ts, machining.ts,
pathfollow.ts) and `src/core/collision/collision.ts`. Tests: `tests/sim_*.test.ts`, `tests/collision*.test.ts`.

## Station runtime (`StationRuntime`, contract I1)

One runtime per station (`StationRuntime.of(station)`), one shared simulation clock.

- `start(program, { robot?, startJoints?, args?, fromIndex? })` returns a `RunHandle` (`id`, `status`
  `queued | running | done | error | stopped`, `error { message, instructionIndex }`, `done` promise, start / end time).
  Programs on different robots run concurrently; a run holds the robots it moves (its own moves and those of the
  programs it calls, not the robot its program is linked to) and a run that needs a held robot is queued until it is
  free (RoboDK 06.e: StartSimulation, linked to UR10 A, only calls TakeConveyor on UR10 B while its thread
  FeedConveyor drives UR10 A).
  `fromIndex` starts at an instruction ("Run from here"); the speed, rounding, tool and frame set before it apply.
- `tick(dt)`, `runFor(seconds)`, `runUntilDone()`, `stop(id?)` (a run and the threads it started), `resume()` (after
  "Pause, wait for the user"), `captureHome()` / `reset()` (object poses, parents / attachments, visibility, robot joints,
  tools, frames, robot IO, signals and robot motion state).
- `signals` is the single `SignalTable` (`get`, `set`, `on`, `snapshot`) for robot DI/DO, program signals, process
  component signals and control-model events. Wait DI / wait signal and IF conditions are evaluated when the
  instruction runs (`evalCondition`: `DI[3] == 1 && !Busy`, `S == "go" and n > 3`, loose equality `true == 1 == "1"`).
- A run plays up to its first error and stops there (status `error`, instruction index); the failing move is not
  executed. Other runs continue.
- Program calls inherit the robot state (speed, rounding, tool, frame belong to the robot, like RoboDK). Threads
  ("Run on a separate thread") start child runs. Calls that name a station Python program, and code calls such as
  `SetRPM(70)`, go through the script runners (`registerScriptRunner`, contract I5: Pyodide in the browser, CPython on
  the server, the vision runner). Runners are tried in order; a runner that resolves `false` does not handle the call.
- Events: attach takes the object closest to the TCP whose bounding box is within the tool's reach; a hidden object
  only as the twin (same pose) of a part the tool already holds (RoboDK stations carry a visible part and its hidden
  twin with two Attach events); detach puts the object back under the frame it came from; show / hide; RoboDK
  "Replace objects" restores the stored placements. A held part does not collide with the gripper mechanism holding it.
- Mobile robots: Navigate / Follow path ask the registered planner (`setMobileMotionPlanner`, contract I2; default:
  straight line / path polyline at the instruction speed).

## Program simulator (`ProgramSimulator`)

`compile(program, startJoints?)` runs the same interpreter in record mode (event driven, continues after errors to
list every problem, never executes scripts) and returns the timed steps used for previews, validation, cycle times and
timeline scrubbing; `play()` stops at the first error (`errorTime`). `jointsList()` gives RoboDK-style
InstructionListJoints rows. Every move step carries `qTarget` (joints at its target; a blended move passes beside it)
and `estimate` (its Item.Update() time).

The result has two times:

- `duration`: the simulated time (programmed speeds, accelerations and blending; what playback shows);
- `updateTime`: **RoboDK's cycle time**, the `Item.Update()` estimate RoboDK reports, measured on RoboDK 5.9.4 (probes
  in `tests/sim_rdk_update.test.ts`): every move starts and ends at rest; the robot's own motion state is used (speed,
  acceleration, rounding stored with the robot or set with `robot.setSpeed`), the program's Set speed / acceleration /
  rounding instructions are ignored; linear moves take trapezoid(TCP path length, v, a) (MoveC: the arc), joint moves and
  linear moves without translation take trapezoid(largest joint travel of the arm and its carrier, vj, aj); a robot
  rounding > 0 removes the acceleration phases (length / speed); a robot table without joint speed / acceleration runs
  joints at 2·v deg/s and 2·a deg/s²; pauses add their time. A call into a program of another robot than its caller's
  starts from that robot's joints at the start of the update (06.d Stamp1: 17.925 s like RoboDK); a program stops
  counting at its failing instruction and its caller goes on. `estimateProgram(st, p, { roboDKUpdate: true })` returns
  `updateTime` as `duration`.

## Motion (planner.ts)

- MoveJ to a Cartesian target solves IK on the target pose with the active tool; the stored joints only choose the
  configuration. Stored joints outside the joint limits carry no configuration: the IK keeps the robot's current one
  and takes its closest solution (RoboDK, 03.g Out Approach 2 with J3 = -251°). Joint targets, and any target of a
  < 5-axis mechanism (gripper, gun, positioner), go to their joints.
- Reference frames like RoboDK: a Cartesian target is its pose relative to its parent, applied to the active reference
  frame (Set Ref. / program frame / robot active frame), so one program serves several parts (01.c). A robot imported
  from RoboDK applies it from the start (its active frame); a studio program that never sets a reference keeps absolute
  targets.
- Validity like RoboDK: a linear / circular move that passes within 2.5° of the wrist singularity of a spherical wrist
  fails (03.a Oil MoveL 312 at 2.42°, 03.d Polish MoveL 424 at 1.8° fail in RoboDK, 2.52° passes); Comau arms (J2/J3
  parallelogram) check the J3 limits on q3 − q2 (09.c).
- MoveL / MoveC: Cartesian path relative to the reference frame; IK is solved at adaptive knots (a knot interval is
  accepted when the joint-interpolated midpoint stays within 0.1 mm / 0.1 deg), so dense machining programs stay cheap.
- Rounding (blending): a zone of radius min(r, L/2) around the corner (quadratic Bézier, Cartesian between Cartesian
  moves, joint space otherwise); the path speed is kept through the zone. Fine points stop the robot.
- Timing: one time-optimal velocity profile per blended chain. Limits: programmed linear speed, rotation speed (joint
  speed in deg/s), joint speed for joint moves, per-joint maximum velocities, programmed accelerations on the limiting
  joint / the path, per-joint maximum accelerations.
- Synchronised external axes (`MotionGroup`): rails / gantries / turntables (robot.externalAxes, the carrier the arm is
  mounted on) are interpolated with the arm; targets with 6+N joints (and inline machining targets) drive them; a MoveJ /
  MoveL out of the arm's reach on a carrier picks carrier values with the combined IK.
- Tools mounted on a mechanism the robot carries (weld gun, gripper robot) are resolved through that mechanism.

### RoboDK parity (`node scripts/robodk-gt.mjs --sim`, report `docs/robodk-library-sim.md`)

Ground truth: `tests/fixtures/robodk_gt/sim/*.json.gz`, written by `dump_sim.py` with RoboDK 5.9.4 itself: every
program of the 57 Library stations run from the station as loaded (the station is reloaded per program):
`Update(COLLISION_OFF)`, `InstructionListJoints` (a row at every move end), `Update(COLLISION_ON)` and the colliding
item pairs. Checks (tests/robodk_gt.test.ts with ROBODK_GT_SIM=1): validity both ways, `updateTime` within ±10 % of
`Update()`, joints at every listed move end within 0.5°, collision verdict (ROBODK_GT_COLL=1).

- 97 of 97 files pass; Item.Update() cycle times are equal to RoboDK's to the millisecond on most programs (machining
  programs with hundreds of moves included: 02.c 101.45 s, 03.g 26.886 s, 05.d 177.45 s).
- RoboDK behaviours established on RoboDK itself and reproduced: the Update() model above; `InstructionListJoints`
  places the robot at the first target (no motion), lists only the program robot's moves and a called program once,
  and lists every move's target also when the path is blended past it (compared through `qTarget`).
- Verified RoboDK behaviours the studio does not copy (reported as notes per file): `Update()` keeps a positioner where
  it stands although the program turns it through a called program (05.b TopSurf / Inside1 / Side0 / Side180 /
  Side270 are invalid in RoboDK at the stored turntable angle and valid at the angle their own TTxdeg call selects —
  evidence `sim/…05.b….positioner.json`, `dump_turntable.py`; from that angle the studio gives the same validity and
  cycle time); a caller is rated valid when only a called program fails (09.c MainProgram, 05.b Main); a program that
  calls another one several times is listed once by InstructionListJoints (joints compared through the called
  programs).

## Collisions

- Robot links with imported meshes are tested triangle-exactly through local BVHs when an asset store is given;
  procedural links use capsules. Tools without geometry have no collider. `SceneObject.colliders` (Webots
  boundingObject, URDF collision) replaces the visual geometry.
- Default rules: adjacent links, tool vs its last links, a part vs the tool holding it, robot base vs its mount.
- The station collision map is honoured: user pairs (`setCollisionPair`) and the imported RoboDK map
  (`station.params.collisionMap = { entities, pairs }`: between two entities only RoboDK's checked pairs are tested).
  RoboDK entities are mesh links: every imported robot visual carries its `meshLink` (passive counterbalance parts are
  entities of their own although they ride on links 1 / 2).
- RoboDK semantics on stations with RoboDK's map: every checked pair in contact is a collision, also when it is in
  contact from the start and between items the program does not move (RoboDK 5.9.4 reports 02.i Wood_Holder ×
  Finish_Part1, 06.e UR10 A × GripperA Opened, 06.f gripper fingers, 01.d Camera 2 × Camera view on the first row of
  `InstructionListJoints(COLLISION_ON)`). Studio stations keep ignoring contacts present at the start.
- Imported geometry is placed like RoboDK (checked against RoboDK with probe shapes at mesh vertices,
  `RDK.Collision`): tool and object meshes sit at the inverse of the frame stored in their table; counterbalance
  cylinders / rods are look-at parts (`LinkVisual.passive`, `visualOrigin()`; RoboDK link poses of KR 90 / KR 100,
  IRB 6640 / 6700, R-2000iB at 6 joint vectors, `tests/kin_passive.test.ts`).
- Exact mesh / mesh tests: faces in contact count (coplanar overlap within 1 µm: the JAKA Zu5 tool on its flange
  collides in RoboDK at 0 mm and not 0.001 mm away); two meshes are taken as touching only after 4 M triangle tests
  (250 k reported robot links inside the bars of a cage enclosure as touching, 06.d).
- `collisionOptions.ignoreEvents` reproduces RoboDK's `Item.Update(COLLISION_ON)`: show / hide / attach / detach
  events are not applied during the check, threads are not run and called programs of other robots are checked only
  where they move (RoboDK 5.9.4: the hidden 05.e Paint gun shown by its program does not collide in Update(COLLISION_ON)
  and does when made visible beforehand; 06.f Init / Gripper2DropOutput). Playback keeps the events.
- Along timelines (`collisionOptions.enabled`): samples every 2 deg of joint motion (start posture always checked;
  coarser for very long programs, `maxSamples`), other robots and carriers re-evaluated at every sample, initial
  contacts ignored (except self-collision), at most `maxCollisions` colliding instructions and `timeBudgetMs`.

## Performance

Whole Library (57 stations, 235 programs) imports and compiles in about 4 minutes on one worker; the slowest
compiles are 05.c Pillar (rail + turntable, 4800 moves) 8.8 s and CE3_3DBenchy (15 k moves) 5.9 s.
