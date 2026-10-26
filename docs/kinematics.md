# Kinematics

Robot kinematics live in `src/core/kinematics/` and `src/core/items/robot.ts`. Units: mm and degrees (prismatic joints
in mm). Every claim below has a regression test in `tests/kin_*.test.ts`. Most of them check against RoboDK 5.9.4 ground
truth (`tests/fixtures/robodk_gt`, robot tables read from `C:/RoboDK/Library`).

## Chain model (`chain.ts`)

`ChainDef` is a serial chain: `base · Π (origin_i · motion_i(q) · post_i) · flange`. It carries the following:

| Field | Meaning | RoboDK source |
|---|---|---|
| `joints[i].axis` | motion axis. Its sign is the **joint sense**; a prismatic axis may be scaled (gripper fingers travel q/2) | table `[61]` joint convention, `[64+i]` senses |
| `base` | fixed transform from the robot item frame to joint 1. It is part of the kinematics, not of the item pose | table `[180..185]` |
| `flange` | flange offset after the last link | table `[560..565]` |
| `joints[i].couple` | `angle = q_i + m · q_src`. Used for the Fanuc J2/J3 interaction and the Comau parallelogram | joint convention 2 / 11 |
| `joints[i].mimic` | passive joint following another one | URDF mimic, grippers |
| `flangeLink` | link that carries the flange. A gripper's flange stays on the base (link 0) | robot class 15-17 |
| `parallel` (`DeltaDef`) | closed-chain delta (ABB IRB 360). FK uses trilateration and IK is closed form; links are `[base, arm1, arm2, arm3, platform]` | robot class 5 |
| `jointsType` | RoboDK joint convention code (for the Comau `q2 + q3` limit) | `[61]` |

`rdk_model.ts` turns a RoboDK robot table into a chain with `rdkParamsFromTable(t)` and then `chainFromRobodk(p)`. FK
matches RoboDK `SolveFK` to within 0.01 mm / 0.01° for every Library robot. That covers KUKA, ABB, Fanuc, Yaskawa,
Kawasaki, Stäubli, Comau, Denso, JAKA, Omron, Doosan, Meca500, the iiwa 7-axis arm, the 3D/4D deltas, rails,
turntables, gantries and grippers (`tests/kin_rdk_model.test.ts`). If you edit a chain's joints in place, call
`invalidateChain(chain)` afterwards.

## Inverse kinematics (`ik_analytic.ts`, `ik.ts`)

- **Closed form**. The solver works from screws plus Paden–Kahan subproblems, derived from the chain geometry. Joint
  senses, offsets, base/flange transforms and J2/J3 couplings are therefore handled automatically.
  - `spherical6`: axes 2 ∥ 3 and axes 4, 5, 6 meet in one point. Up to 8 solutions.
  - `ur6`: axes 2 ∥ 3 ∥ 4, with 4∩5 and 5∩6. Up to 8 solutions.
  - `delta`: one solution per arm, elbow out.

  Input poses are re-orthonormalised first, because files store rotations rounded to 6 decimals.
- **RoboDK configuration flags**. `configFlags(chain, q)` returns `[REAR, LOWERARM, FLIP]`, which equals RoboDK
  `JointsConfig` for every 6-axis Library robot. `solveIKBest(chain, pose, {seed, config})` returns the solution closest
  to the seed that has the requested flags. With a seed, consecutive calls follow one continuous branch, so MoveL keeps
  working through q5 = 0.
- **All solutions**. `ikAll(chain, pose)` returns the solutions inside the limits, sorted by configuration. Like RoboDK
  `SolveIK_All`, it adds the ±360° variants that fall strictly inside the limits.
- **Numeric fallback**. Chains with no closed form (offset wrists such as the Cobotta, 7-axis arms, SCARA,
  palletizers, URDF trees) use `inverseKinematics`. It is Levenberg–Marquardt with damping scaled to the Jacobian,
  clamping to joint limits, and restarts. A residual below 50× the tolerance at a singular Jacobian is accepted as
  `nearSingular` with a `warning`, not reported as an error.
- **Unreachable poses**. When the closed form proves that no solution exists, the result has `unreachable: true`. This
  distinguishes an unreachable pose from a singularity crossing, even when the numeric residual is small.
- **Performance**. The analysis is cached per chain, and the FK fast path works in place for ±z rotations and z
  translations. The test requires a closed-form solve under 0.5 ms on a loaded CI machine. A typical solve takes about
  20–60 µs (8 branches, each verified by FK).

## Robot API (`robot.ts`), RoboDK semantics

- `solveFKRef(q, tool?, reference?)`: flange w.r.t. the base, or the TCP / pose in a reference. The active tool is
  ignored, as in RoboDK `SolveFK`.
- `solveIKRef(pose, {seed, tool, reference, config})`, `solveIKAllRef(pose, tool?, reference?)`, `jointsConfig(q)`.
- The legacy TCP helpers (`solveFK`, `solveIK`, `solveIKAll`, `solveIKFrame`, `poseTCP`) still use the active tool.
- `jointMaxVelocities()` / `jointMaxAccelerations()` (and their setters), and `motionDefaults` + `resetMotion()` for
  the robot's default speeds.
- `reach` / `estimateReach(chain)`: the largest radius from axis 1 to the wrist centre (or to the flange). It matches
  the RoboDK table value to within 0.2 % for most arms. For a delta it is the platform workspace radius.

## Synchronised external axes (`combined.ts`)

`SyncedSystem` describes an arm plus its mechanisms in RoboDK external-axis order. Each mechanism is a `carrier` (rail,
gantry) or a `positioner` (turntable). The joint vector is `[arm..., ext...]`, as in RoboDK's 7/8/9-joint targets.

- `syncedFK(sys, q, tool?)` is RoboDK `SolveFK` of a synchronised robot. It gives the flange relative to the first
  positioner's flange, or relative to the carrier base when there is no positioner. It matches RoboDK for Library 05.c
  (IRB 6640 on a rail, synchronised with a turntable) and 05.d (Comau on a Gudel gantry).
- `syncedIK(sys, pose, {ext, seed, config, tool})` keeps any external values the target gives. For free axes it picks
  the feasible values closest to the seed, using the closed-form arm solver; otherwise it falls back to a numeric
  solve on the combined chain.
- Other helpers: `syncedIKAll` (matches the RoboDK count for both cells), `syncedSystemFor(robot)`,
  `robotSolveFK(robot, q)`, `applySynced`, `syncedJoints`. The legacy `solveIKWithCarrier` keeps its signature.

## Known gaps

- Fanuc `SolveIK_All` count. RoboDK lists more solutions (24, where the studio lists 16 for the M-10iA at a pose with no
  limit ambiguity). The rule RoboDK uses to limit J2/J3 interaction has not been decoded.
- UR-type `SolveIK_All` count. RoboDK lists no turn variants for JAKA (4). For the UR10 it gives 12, 14 or 23 at
  equivalent poses, so the count does not look deterministic and has not been reproduced.
- `reach` uses the wrist-centre definition. Catalogues that measure to the flange (Stäubli, Doosan, Meca500) and the
  UR catalogue values differ by 8–20 %.
