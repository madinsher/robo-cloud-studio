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

RoboDK's online library (`.robot` files) and stations (`.rdk`) are proprietary binary containers. The studio offers
three import paths, tried in this order when a file is dropped:

| Path | Needs | Fidelity |
|---|---|---|
| `POST /convert/rdk` on the studio server → `python/rdk2vbs.py` opens the file in a hidden RoboDK instance and exports it with `rdk_export.py` | RoboDK installed next to the server, `STUDIO_ROBODK_PYTHON` pointing at a Python with the `robodk` package | lossless: items, poses, robot kinematics (DH), tools, targets, programs, meshes |
| `rdk_export.py` run inside RoboDK (Tools › Run script) | RoboDK on the user's machine | lossless, manual |
| Best-effort binary scan (`src/io/robodk/rdk_container.ts`) | nothing | names, poses and embedded meshes recovered heuristically; kinematics matched by name to the library |

For the *robot library* itself no RoboDK is needed: the online library (`Add › Robot from online library`) provides the
same industrial robots from the open ROS-Industrial / vendor URDF packages, with exact kinematics and meshes — see
`docs/file-formats.md`.

### What the RoboDK conversion carries

`rdk_export.py` (run inside RoboDK, or automatically through `rdk2vbs.py`) writes: frames; robots (pose, joints,
limits, DH when exposed, active frame/tool); tools (TCP, mesh); targets (pose, joints, joint/cartesian flag) under
their frames; objects (STL, plus curves and points via `GetPoints`); programs (instruction list incl. speed, frame
and tool changes, pauses, IO, code, comments, program frame/tool, **and** the simulated joint path from
`InstructionListJoints`, stored as `params.jointsList`); machining projects (robot / part / generated program
links, update ratio); Python programs (source when available). The importer (`src/io/robodk/rdk_import.ts`)
rebuilds instructions, links machining items and regenerates curve/point-follow programs when RoboDK's generated
program was not exported.

### RoboDK post processors

The studio runs RoboDK's own post processors: `Program › Import RoboDK post processors (.py)` (select the files
from `C:/RoboDK/Posts`). They execute in Pyodide with the official `robodk.robomath`/`robofileio` and a headless
`robodialogs`; the compiled program drives `ProgStart/MoveJ/MoveL/MoveC/setFrame/setTool/Pause/setDO/waitDI/RunCode/RunMessage/ProgFinish`.
On the server the same happens with the system Python (`python/post_shim.py`).
