# RoboDK compatibility

VerticalBot Studio mirrors RoboDK's item model and API so RoboDK users, scripts and workflows carry over.

## Item model

| RoboDK | Studio | Notes |
|---|---|---|
| Station (.rdk) | `Station` (.vbstation JSON) | `.rdk` import is best-effort (proprietary container); use `python/rdk_export.py` inside RoboDK for lossless export |
| Reference frame | `Frame` | pose relative to parent, `setParentStatic` keeps absolute pose |
| Robot | `Robot` | chain from library (DH) or URDF; active frame/tool; `SolveFK/SolveIK/SolveIK_All`; joint limits |
| Tool | `Tool` | pose = TCP w.r.t. flange; geometry in flange coordinates |
| Target | `Target` | cartesian (pose w.r.t. parent frame) or joint target; recorded joints |
| Program | `Program` + `Instruction` children | MoveJ/L/C, speed, rounding, frame, tool, pause, IO, code, comment, message, call, events |
| Object | `SceneObject` | STL meshes / primitives, curves and points (for path following) |
| Camera | `Camera` | |
| Folder / Notes | `Folder` / `Notes` | |

Type constants match RoboDK's `ITEM_TYPE_*` values (STATION=1, ROBOT=2, FRAME=3, TOOL=4, OBJECT=5,
TARGET=6, PROGRAM=8, INSTRUCTION=9, FOLDER=17, CAMERA=19). Studio extensions use values ≥ 100
(MOBILE_ROBOT=100, COMPONENT=101, FLEET=102, MAP=103, FIELD=104, MISSION=105, …).

## Pose conventions

Poses are 4×4 homogeneous matrices in mm. `Pose_2_TxyzRxyz` / `TxyzRxyz_2_Pose` (RoboDK default,
`transl·rotx·roty·rotz`), `Pose_2_KUKA` (ZYX A-B-C), `Pose_2_Fanuc` (W-P-R), `Pose_2_ABB` (quaternion),
`Pose_2_UR` (rotation vector) are implemented identically in `src/core/math/pose.ts` and `python/robodk/robomath.py`.

## API (Robolink / Item)

Supported (JavaScript console, Python drop-in, JSON-RPC):

`Robolink`: `Item, ItemList, ItemUserPick, ActiveStation, AddStation, AddFrame, AddFolder, AddTarget, AddProgram,
AddRobot*, AddMobileRobot*, AddShape, AddCurve, AddPoints, ShowMessage, Render, Update, setRunMode/RunMode,
setSimulationSpeed, Save, getParam/setParam, Command, Version, License, Selection/setSelection, Delete, Collisions`.

`Item`: `Valid, Name/setName, Type, Parent, Childs, Delete, Visible/setVisible, setParent/setParentStatic,
Pose/setPose, PoseAbs/setPoseAbs, PoseTool/setPoseTool, PoseFrame/setPoseFrame, Joints/setJoints, JointsHome,
JointLimits/setJointLimits, SolveFK, SolveIK, SolveIK_All, Connect/ConnectedState, setSpeed, setRounding/setZoneData,
MoveJ, MoveL, MoveC, Pause, setDO, waitDI, RunInstruction/RunCodeCustom, InstructionCount, Instruction,
InstructionList, InstructionDelete, setRobot, getLink, AddTool, AddFrame, AddTarget, setAsCartesianTarget,
setAsJointTarget, isJointTarget, Update, RunProgram/RunCode, MakeProgram, setParam/getParam, setColor/Recolor,
setGeometryPose, Busy, Stop, WaitMove, DOF`.

`*` = studio extension. Not supported (no-op or message): `AddFile` from a path (use drag & drop or the
station file), 2D camera snapshots, collision maps, machining projects, calibration projects.

Differences:
- `MoveJ/MoveL` on a robot item move instantly in the API (the simulator animates programs, not API moves).
- `MakeProgram(folder, post_id)` returns `(ok, code, filename)` and accepts the studio post id
  (`KUKA_KRC4`, `ABB_RAPID_IRC5`, `Fanuc_R30iA`, `Universal_Robots`, `Motoman`, `Staubli_VAL3`, `Doosan_Robotics`,
  `Mecademic`, `Generic`, `JSON`, `RoboDK_Python`, `ROS2`).
- `Update()` returns `[valid_instructions, program_time_s, program_distance_mm, valid_ratio, message]`.

## Workflows

**RoboDK → Studio**: run `python/rdk_export.py` in RoboDK (Tools › Run Script). It writes `<station>.vbstation`
and STL meshes; open the file in the studio. Robots are matched to the library by name; joints, limits, targets,
programs, frames and tools are preserved.

**Studio → RoboDK**: `File › Export station as RoboDK API script`, run the `.py` in RoboDK, save as `.rdk`.
Programs can also be exported individually with the `RoboDK_Python` post.

**Scripts**: existing RoboDK Python scripts run against the studio server with `PYTHONPATH=studio/python`
(the `robodk` package there shadows the real one). Set `STUDIO_URL=ws://host:20500` for remote servers.

## Transport

JSON-RPC over WebSocket: `{ "id", "method", "params", "target" }` where `target` is an item id. Items are
encoded as `{"$item": id, "name", "type"}`, poses as `{"$pose": [[...4 rows...]]}`. The server relays to a
browser "host" session when one is connected, otherwise executes against a headless station.

## Robot library and project import

RoboDK files (`.rdk` stations, `.robot`, `.tool`, `.sld`) are decoded natively (`src/io/robodk/`, format in
[robodk-file-format.md](robodk-file-format.md)). Every file of `C:/RoboDK/Library` is compared with what RoboDK 5.9.4
itself reports for it (`tests/robodk_gt.test.ts`, ground truth in `tests/fixtures/robodk_gt`, per-file results in
[robodk-library-compat.md](robodk-library-compat.md)): item tree and absolute poses, robot DOF / limits / current and
home joints / FK (worst 0.0009 mm over 111 robots, synchronised rails / turntables / gantries included) / active tool
and frame, tool TCPs, targets, programs (robot, every instruction with its target / inline pose), Python programs,
machining links. A station RoboDK wrote through its API (`tests/fixtures/robodk_gt/synthetic`) pins IO, waits,
"Stop", rounding, joint speeds, every call variant and Cyrillic names.

| Path | Needs | Fidelity |
|---|---|---|
| Native reader (`importRdkBestEffort` → `importRobodkBinary`), used when a file is opened | nothing | RoboDK's tree, kinematics, link meshes, targets, programs by item reference, Python programs (source), folders, curves / points, station parameters, collision map; `report.dropped` lists what is not reproduced |
| `POST /convert/rdk` on the studio server → `python/rdk2vbs.py`: a new hidden RoboDK on a free port (21500+) re-saves the file; the studio decodes it and cross-checks it with RoboDK's API listing (`importRdkExport`) | RoboDK installed next to the server | same as the native reader, for files written by other RoboDK versions |
| `python/rdk_export.py` run inside RoboDK (Tools › Run script) | RoboDK on the user's machine | same (embeds the re-saved station) plus the API walk |

For the *robot library* itself no RoboDK is needed: `.robot` files decode directly, and the online library
(`Add › Robot from online library`) provides industrial robots from the open ROS-Industrial / vendor URDF packages.

### What the import carries

Robots: kinematics from the robot table (`chainFromRobodk`: brand joint senses, Fanuc / Comau J2-J3 coupling, base
offset in the kinematics, flange offset, deltas, grippers), every link mesh (merged per colour), home and current
joints, default speeds, post processor id, synchronised external axes (`robot.externalAxes`, `params.syncAxes` with
carrier / positioner roles and joint indices). Programs: robot by stored reference, Set Ref / Set Tool (lost links
resolved from the pose stored in the instruction), MoveJ / MoveL / MoveC with targets or inline joints + pose
(external axes included), speeds and accelerations, rounding, pauses, IO set / wait, events (attach / detach / show /
hide / replace objects), calls, `Run` = thread, calls to routines and Python programs = code calls (`SetRPM(70)`).
Python programs: `ItemType.PROGRAM_PYTHON` with `params.pythonCode` (exact source). Machining projects: links to
robot, frame, tool, part, generated program and the NC / APT input file. Not reproduced (listed per file): machining
settings, animated counterbalance / delta rod / 4-bar gripper links (drawn rigid), camera and notes contents.

### RoboDK post processors

The built-in posts reproduce 27 RoboDK posts token for token (`tests/posts_gt.test.ts`, docs/posts.md). RoboDK's own
posts: compiled ones (`libspp`, most of `C:/RoboDK/Posts`) run on the studio server with RoboDK's embedded Python
(`server/posts_native.ts`, `POST /posts/native/run`); source posts also run in the browser (Pyodide, official
`robodk.robomath`/`robofileio`, headless `robodialogs`); the compiled program drives
`ProgStart/MoveJ/MoveL/MoveC/setFrame/setTool/Pause/setDO/waitDI/RunCode/RunMessage/ProgFinish/ProgSave`.
On the server the same happens with the system Python (`python/post_shim.py`).
