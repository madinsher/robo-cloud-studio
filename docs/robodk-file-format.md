# RoboDK file format (.rdk / .robot / .tool / .sld) as read by the studio

The studio decodes RoboDK files natively (`src/io/robodk/`, entry point `importRdkBestEffort` → `importRobodkBinary`).
Every fact below is verified against RoboDK 5.9.4: the 96 files of `C:/RoboDK/Library` are imported and compared
with what RoboDK itself reports for them (`tests/fixtures/robodk_gt`, `tests/robodk_gt.test.ts`, results in
[robodk-library-compat.md](robodk-library-compat.md)), and a station RoboDK wrote through its API
(`tests/fixtures/robodk_gt/synthetic`, `tests/rdk_synthetic.test.ts`) pins the instruction encodings the Library
does not use (IO, waits, "Stop", joint speeds, Cyrillic names).

## Container

`u32 big-endian uncompressed size | zlib stream`. Inside: little-endian data; strings are `u32 byte length + UTF-16LE`
(any code point); poses are 16 doubles, column-major (the studio's `Mat4` layout).

## Station (.rdk)

`"Station File 01.01"`, then a linked list of chunks, then `"Station File End"`, `"STATION DATA"`… `"TREE DATA"`.

    chunk = str "lvl" | u32 kind (RoboDK ITEM_TYPE_*) | u32 depth | u64 offset of the next chunk | payload

The chunk index in file order (station = 0) is the **item reference** used by every other chunk (program robot,
Set Ref / Set Tool, move targets, calls, events, external axes, machining links). The importer builds
`index → Item` in one pass and resolves references in a second one: linear time (3DBenchy, 15 423 moves: ~2 s;
the old label-driven importer needed 876 s).

| kind | payload |
|---|---|
| 1 station | u8, str name, …, `u32 n, n × (str key, str value)` station parameters, `[u32 0]` (older files), `f64 200` … |
| 3 frame | str "FrameChkptStart", str name, u8 visible, pose (in the parent), str "FrameChkptEnd" |
| 2 robot | robot table (below) + link meshes, 16 zero bytes, i32 active frame, i32 active tool, driver strings, `u32 n, n × i32` synchronised mechanisms (external axes in joint order) |
| 4 tool / 5 object | object table (below) + shapes + points + curves |
| 6 target | str name, u8, i32 robot, u8 isJoint, u8, `u32 n, n × f64` joints, pose (in the parent frame) |
| 8 program | str name, i32 -1, u32 flags, u8, i32 robot, u32 nIns, instructions |
| 10 Python program | str name, u8, u32 nBytes, u32, UTF-8 source (exact bytes) |
| 11 machining | str "MachiningChkptStart", str name, u8, i32 robot, i32 frame, i32 tool, i32 part, i32 program, settings (not decoded) |
| 17 folder | str name, u8 visible |
| 19 camera, 22 notes | name only |

A program/project link of `-1` means "none"; RoboDK lists called programs again under the caller in its tree — those
are references, not items.

### Instructions

    str "ins" | u32 index | u32 INS_TYPE | u64 next | str label | data

| INS_TYPE | data | studio instruction |
|---|---|---|
| 0 move, 1 MoveC | u32 move type (1 J, 2 L, 3 C), then one point (two for MoveC: via, end). Point = `u32 1, i32 target` or inline `u32 2, u16 isJoint, u32 n, n × f64 joints (external axes included), pose (in the active frame), 0xFF×8` | `move` (target, or inline `joints` / `pose`; `viaTargetId` / `viaPose`) |
| 2 speed | `u8 useVL, u8 useAL, f64 vL, f64 aL, i32 -1, u8 useVJ, u8 useAJ, f64 vJ, f64 aJ` (only flagged values apply) | `speed` |
| 3 Set Ref, 4 Set Tool | `i32 item, u8, pose` — the pose is the frame relative to the robot base / the TCP; it is used when the item link was lost (copied programs): the importer links the same-named frame at that pose or recreates the frame / tool | `frame` / `tool` |
| 6 pause | f64 ms (-1 = "Stop", wait for the operator) | `pause` |
| 7 event | u32 action (0 attach, 1 detach, 2 show, 3 hide, 4/5 replace objects), i32 tool, i32 frame, item lists / poses | `event` |
| 8 code | u32 kind (0 call, 1 insert code, 2 start thread ("Run"), 3 comment, 4 message), str text | `call` (to a program) / `code` (`asFunctionCall` for calls to routines and Python programs) / `thread` / `print` |
| 9 print | str message | `print` (message) |
| 10 rounding | f64 mm (-1 = "Accurate") | `rounding` |
| 11 IO | u32 action (0 set DO, 1 wait DI, 2 set AO, 3 wait AI), str variable, str value, f64 timeout ms (-1 none) | `io` (`analog: true` for AO / AI; `timeoutMs`), non-numeric values → `signal` / `wait` |

## Robot table (.robot body, robot chunks)

4000 doubles (`[0] = 1`, `[1] = 4000`), then the link meshes. Kinematics: `rdkParamsFromTable` / `chainFromRobodk`
(`src/core/kinematics/rdk_model.ts`, FK = RoboDK SolveFK for every Library robot).

| index | meaning |
|---|---|
| 20 | robot class (1 serial, 3 7-axis, 5 delta, 6 cobot, 8 Denso, 15 / 17 grippers, 0 mechanisms) |
| 21 / 22 | DOF / mesh link count |
| 61, 64+i | joint convention (1 ABB, 2 Fanuc, 3 Yaskawa, 5 KUKA, 7 / 11 / 12 table senses), per-joint sense |
| 101 | reach |
| 120..124 | default linear speed, linear accel, rounding, joint speed, joint accel → `robot.setMotionDefaults` |
| 180..185 | base offset x, y, z, rx, ry, rz (rad) → `chain.base` (part of SolveFK, not the item pose) |
| 200+20i | DHM row: alpha (rad), a, theta (rad), d, type (1 linear), extra values (delta / gripper geometry) |
| 560..565 | flange offset → `chain.flange` |
| 600+i / 620+i | joint limits, synchronised external axes included after the arm joints |
| 680+i / 700+i | home / current joints |
| 980+i | Motoman pulses per unit |
| 1060 / 1080 | passive link definition (counterbalance: mesh link numbers + pivots) → `robot.params.rdkPassive` |
| 1800 / 3800 | name / post processor id: UTF-8 bytes stored one per double as signed chars |

Joint max speeds / accelerations are not stored in the Library files (zeros).

Opening a `.robot` file in RoboDK creates a frame "`<robot>` Base" at the negated base-offset translation and puts
the robot under it; the importer does the same.

### Link meshes

`u64 nLinks`; per link (`link > 0`: two zero u64) `u64 nShapes`; shape = `u64 nTri, RGBA f32, nTri·3 × (xyz f32 +
2 bytes)`. Link 0 is drawn in the base-offset frame, link i in DHM frame i. Extra mesh links are passive parts:
counterbalance cylinder / rod (rigid on links 1 / 2, not animated), delta lower-arm rods (aimed at the import pose),
the 4-bar linkage of the RobotiQ 2F-85 (static). The importer merges the shapes of a link per colour (one mesh and
draw call per colour) and keeps every triangle (UR5e 264 588, IRB 6700 LeanID 445 929).

## Object / tool table (.tool, .sld, tool and object chunks)

1600 doubles: `[32..47]` geometry origin, `[48]` visible, `[50]` tool flag, `[96..111]` object pose, `[160..175]` TCP,
`[1440]` name (UTF-8 signed chars). Then `u64 nShapes` + shapes, `u64 nPoints` + points
(`i32 -1 | str name, RGBA, xyz, xyz, normal, f32 size`), `u64 nCurves` + curves
(`u64 n, str name | i32 -1, RGBA, n × (f32 flag, xyz, normal, tangent, xyz)`); curve normals may be NaN (edges
imported from STEP): the point is kept with a zero normal.

## What is not decoded

Machining project settings other than the links and the input file (the generated program is imported; RoboDK
regenerates it at load anyway), camera and notes contents, collision map, view state. The import report
(`report.dropped`, `fidelity: 'partial'`) lists every such item per file.
