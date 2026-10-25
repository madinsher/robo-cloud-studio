# Simulation: station runtime, motion planning, collisions

Source: `src/core/motion/` (runtime.ts, planner.ts, simulator.ts, trajectory.ts, mobile_hook.ts, machining.ts,
pathfollow.ts) and `src/core/collision/collision.ts`. Tests: `tests/sim_*.test.ts`, `tests/collision*.test.ts`.

## Station runtime (`StationRuntime`, contract I1)

One runtime per station (`StationRuntime.of(station)`), one shared simulation clock.

- `start(program, { robot?, startJoints?, args?, fromIndex? })` returns a `RunHandle` (`id`, `status`
  `queued | running | done | error | stopped`, `error { message, instructionIndex }`, `done` promise, start / end time).
  Programs on different robots run concurrently; a program for a busy robot is queued until the robot is free.
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
InstructionListJoints rows.

## Motion (planner.ts)

- MoveJ to a Cartesian target solves IK on the target pose with the active tool; the stored joints only choose the
  configuration. Joint targets, and any target of a < 5-axis mechanism (gripper, gun, positioner), go to their joints.
- Reference frames like RoboDK: a Cartesian target is its pose relative to its parent, applied to the active reference
  frame (Set Ref. / program frame / robot active frame), so one program serves several parts (01.c).
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

### RoboDK parity (Library probe `tests/_probe/sim_t3_gt.test.ts`, GT `tests/fixtures/robodk_gt`)

- Cycle times equal to RoboDK's `Update()` to 0.01 s on the clean cases (01.a ApproachMove 2.13 s, 01.b 7.48 s, 01.c
  15.75 / 7.99 / 7.75 s, 06.f Pick / Drop / gripper programs) and path lengths to < 0.1 % (01.c, 03.i 6108 mm).
- The simulator honours Set speed instructions, as RoboDK's own playback does (GT jl rows of 07.c run at 20 mm/s).
  RoboDK's `Item.Update()` estimate of some programs (07.c, 02.c) is consistent with those instructions being ignored;
  `ProgramSimulator.speedInstructions = 'ignore'` / `estimateProgram(st, p, { roboDKUpdate: true })` reproduce it.
- Final Library probe (57 stations, 235 programs): every compile < 10 s (max 6 s); validity equal to RoboDK in 206 of
  213 programs; the 7 programs invalid in both fail at the same instruction; joints at every move end within 0.5 deg
  of RoboDK in 138 of 147 comparable programs; cycle time within 15 % of `Update()` in 99 of 168 (speeds honoured) /
  124 of 168 (Update-estimate mode).
- Validity rules: a MoveL / MoveC that starts exactly at a wrist singularity fails (RoboDK rule); a branch switch at a
  joint limit is reported as such.
- Remaining differences: import instruction order in some programs, programs whose timing depends on station Python
  (a compile never runs scripts), RoboDK collision results (70 of 141 colliding programs reproduced; 53 of 57 clean
  ones clean).

## Collisions

- Robot links with imported meshes are tested triangle-exactly through local BVHs when an asset store is given;
  procedural links use capsules. Tools without geometry have no collider. `SceneObject.colliders` (Webots
  boundingObject, URDF collision) replaces the visual geometry.
- Default rules: adjacent links, tool vs its last links, a part vs the tool holding it, robot base vs its mount.
- The station collision map is honoured: user pairs (`setCollisionPair`) and the imported RoboDK map
  (`station.params.collisionMap = { entities, pairs }`: between two entities only RoboDK's checked pairs are tested).
- Along timelines (`collisionOptions.enabled`): samples every 2 deg of joint motion (start posture always checked;
  coarser for very long programs, `maxSamples`), other robots and carriers re-evaluated at every sample, initial
  contacts ignored (except self-collision), at most `maxCollisions` colliding instructions and `timeBudgetMs`.

## Performance

Whole Library (57 stations, 235 programs) imports and compiles in about 4 minutes on one worker; the slowest
compiles are 05.c Pillar (rail + turntable, 4800 moves) 8.8 s and CE3_3DBenchy (15 k moves) 5.9 s.
