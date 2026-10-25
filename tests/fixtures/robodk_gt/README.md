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
