# RoboDK ground truth for the Library compatibility tests

One `<file>.json.gz` per file of `C:/RoboDK/Library` (`.rdk`, `.robot`, `.tool`), dumped from RoboDK 5.9.4 itself
through its official Python API by `dump_gt.py`:

```bash
C:/RoboDK/Python37/python dump_gt.py "C:/RoboDK/Library/<file>" out.json <api-port> <code-dir>
```

Fields: `items` (type, name, parent, `pose` / `poseAbs` as 16 row-major numbers; note that RoboDK returns the TCP for
`robot.Pose()` and the base for `robot.PoseAbs()`), `robots` (joints, home, limits, active tool, 12 FK samples
`{q, flange}` with the flange relative to the robot base), `tools` (`poseTool`), `targets` (pose in the parent frame,
joints, `isJoint`), `objects`, `frames`, `python` (station Python programs), `machining`, `curves`, `cameras`,
`programs` (`instructions` from `Item.Instruction(i)`, `update` = `Item.Update()` →
[valid instructions, cycle time s, distance mm, valid ratio, message], `updateCollision` with collisions on,
`jl` = rows of `InstructionListJoints(5 mm, 5 deg, flags=1)` `[J1..Jn, ERROR, MM_STEP, DEG_STEP, MOVE_ID, TIME, X, Y, Z]`
keeping the last row of every MOVE_ID and every 20th row, `make` = `MakeProgram()` result).

`code/robodk_generated_code.tgz` holds the programs RoboDK generated with each robot's own post processor
(free licence: machining programs and programs over the licence limits are missing).

Re-dumps (T1, round 1): `Example-03.g…` and `Example-02.f…Turntable…` were truncated in the first dump (the hidden Fanuc
post blocked `MakeProgram` until RoboDK died; the collision-checked `Update` of the 02.f machining program never
finished). They were dumped again with `DUMP_NO_MAKE=1` (no generated code) and, for 02.f turntable,
`DUMP_NO_COLLISION=1` (`updateCollision` = null).

`synthetic/` holds stations RoboDK 5.9.4 wrote itself through its API (`make_synthetic.py`): `T1-instructions.rdk`
(+ its ground truth) covers what the Library lacks — IO set/wait, analog output, "Stop", rounding, joint speeds and
accelerations, every RunInstruction variant, inline joint/pose moves, MoveC, duplicate target names, a station
Python program and parameter, Cyrillic names; `T1-collision.rdk` pins the collision map layout.

## Simulation ground truth (round 2, R1)

`sim/<file>.json.gz` (`dump_sim.py`, RoboDK 5.9.4): every program of the 57 Library stations run from the station AS
LOADED — the station is reloaded before every program (dump_gt.py updated the machining projects first, which made
RoboDK regenerate their programs, and chained the programs: every `Update(COLLISION_ON)` leaves the robot where it
stopped). Per program: `start` (joints of every robot), `update` = `Update(COLLISION_OFF)`, `jl` =
`InstructionListJoints(5 mm, 5 deg, flags=1)` rows at every MOVE_ID change (every listed move end, also inside calls)
and every 20th row, `updateCollision` = `Update(COLLISION_ON)` and, after a collision, the colliding items.

```bash
C:/RoboDK/Python37/python -I dump_sim.py "C:/RoboDK/Library/<file>.rdk" sim/<file>_.json <api-port>   # then gzip
C:/RoboDK/Python37/python -I dump_turntable.py "<05.b file>" sim/<05.b>_.positioner.json <port> Turntable 0 90 180 270
C:/RoboDK/Python37/python -I dump_linkposes.py "<file>" <port> "<robot>" passive/<tag>.json
```

- `sim/Example-05.b…positioner.json`: `Update()` of every 05.b program with the turntable set beforehand to 0/90/180/270°
  (RoboDK's Update() keeps a positioner where it stands although the program turns it through a called program).
- `passive/*.json`: `ObjectLink(k).PoseAbs()` of every link (counterbalance links 7 / 8 included) of KR 90, KR 100,
  IRB 6640, IRB 6700 and R-2000iB at 6 joint vectors (joints as RoboDK applied them).
