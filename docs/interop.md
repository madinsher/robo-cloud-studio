# Interoperability: RoboDK, Blender, ROS, Visual Components / KUKA.Sim

This page describes what can be moved in and out of the studio, by which path, and with what fidelity.
Everything here is implemented; limits are stated explicitly.

## RoboDK

| What | Path | Fidelity |
|---|---|---|
| Robot library | `Add › Robot from online library` (91 robots from open ROS-Industrial / vendor URDF packages) | exact kinematics, vendor meshes, `tool0` flange |
| `.rdk` stations, `.robot`, `.tool` | dropped into the browser → `POST /convert/rdk` on the studio server → `python/rdk2vbs.py` (hidden RoboDK instance + `rdk_export.py`) | lossless: frames, robots (DH), tools, targets, programs, objects with curves/points, machining projects, joint paths |
| same, without RoboDK on the server | best-effort binary scan (`src/io/robodk/rdk_container.ts`) | names, poses, embedded meshes |
| Targets | exported with pose + joints + cartesian/joint flag, kept under their reference frames | exact |
| Programs | instruction list (moves, speeds, frame/tool changes, pauses, IO, code, comments) **and** the joint path RoboDK simulated (`InstructionListJoints`, 10 mm steps, `params.jointsList`) | exact instructions; the joint path replays motion even for robots the studio cannot solve |
| Machining projects (curve follow, point follow, 3D printing, milling from NC) | `ITEM_TYPE_MACHINING` items with robot / part / generated program links; part curves and points exported through `GetPoints`; the generated program is exported as a normal program; if it is missing the studio regenerates it with its own curve/point follow (`generateCurveFollow` / `generatePointFollow`) | RoboDK's own machining settings dialog values are not readable through the public API — the generated program and the geometry are |
| NC / G-code (milling, cutting, dispensing, 3D printing) | drop `.nc`/`.gcode`/`.tap` → rapid/cut segments with feeds, spindle, tool and extrusion state on an object → `Robot › Robot machining project` | program with NC feeds as speeds, spindle/extruder outputs, approach/retract; same through the API (`setMachiningParameters`) |
| Programs of robots the studio cannot solve | the RoboDK joint path becomes a `jointPath` instruction | exact replay of RoboDK's motion, exportable through posts |
| Python programs / macros | name; source when RoboDK exposes it or the `.py` sits next to the station | scripts run against the studio's `robodk` drop-in package (`python/robodk`) |
| Post processors | `Program › Import RoboDK post processors (.py)` — select any number of files from RoboDK's `Posts/` folder | the **original** RoboDK Python posts run unmodified in the browser (Pyodide) with the official Apache-2.0 `robodk.robomath` / `robofileio` modules and a headless `robodialogs`; persisted in the browser, selectable per robot and in the export dialog |
| RoboDK API | `RDK = Robolink()` from Python/C#/C++/MATLAB against the studio server, or the browser console | see `docs/robodk-compatibility.md` |

Why not ship RoboDK's posts directly? They are distributed with the RoboDK installer, not in a public repository
we can fetch from; importing them from your own installation is a one-time drag-and-drop and keeps their licence
with you. The posts run exactly as in RoboDK: `ProgStart/MoveJ/MoveL/MoveC/setFrame/setTool/Pause/setDO/waitDI/RunCode/RunMessage/ProgFinish`
are driven by the studio's compiled program (`src/posts/base.ts › compileForPost`).

## Blender

### Blender add-on (direct .vbstation exchange)

`blender/vertical_bot_studio.py` — install in Blender (Edit › Preferences › Add-ons › Install…). It adds:

- *File › Import › VerticalBot Studio station (.vbstation)*: objects with meshes/primitives, robots as link
  hierarchies computed with the studio's exact forward kinematics (DH post transforms, mimic and prismatic joints),
  tools on the flange, frames/targets as empties; when the file comes from *File › Save for Blender add-on* it also
  contains the active program sampled at 30 Hz — every link, flange and moved object gets keyframes, the scene
  frame range and fps are set. No glTF round trip, no unit surprises (mm → m applied on import).
- *File › Export › VerticalBot Studio station*: selected meshes as a `.vbstation` (binary STL assets in mm) the
  studio opens directly.
- *File › Export › VerticalBot Studio pose log (.csv)*: selected objects' world poses per frame as XYZ + Rx Ry Rz
  (mm/deg) — drop the CSV into the studio to get targets along an animated path.

The add-on's kinematics are unit-tested against the TypeScript implementation (`tests/blender_addon.test.ts`).

### Studio → Blender

- **Animated glTF** — `Tools › Export animation as glTF (Blender)…`. The active program is simulated from start to
  end at N keyframes per second; every robot link, flange, tool and moved object (attachments, mobile robots,
  conveyor parts) receives position/rotation keyframes. The file is a standard `.glb` with one animation clip,
  root node scaled 0.001 (mm → m). In Blender: *File › Import › glTF 2.0*, press play. Also imports into Unity,
  Unreal, three.js viewers, Omniverse.
- **Static glTF** — `Tools › Export scene as glTF (.glb)` (scene as is, mm units).
- **URDF package** — `File › Export URDF package (robot / station, zip)…`. One robot or the whole station as a
  ROS description package (`urdf/*.urdf`, `meshes/*.stl` in metres, `package.xml`). Blender opens it with the
  [Phobos](https://github.com/dfki-ric/phobos) add-on (*Import › URDF*) as an articulated armature; ROS opens it
  with `robot_state_publisher` / RViz / MoveIt / Gazebo. DH-defined robots are converted to pure URDF joints
  (the DH "post" transforms are folded into the next joint origin); tools become fixed links with a `_tcp` frame.

### Blender → Studio

- **glTF / GLB** — drop a `.glb`/`.gltf` onto the 3D view: all meshes are flattened into one object (metres → mm,
  Y-up → Z-up; files exported by the studio are recognised and not re-rotated).
- **COLLADA / OBJ / STL** — drop the files; `.dae` honours the unit and up-axis, `.obj`/`.stl` are taken as mm.
- **URDF from Phobos** — export the armature as URDF + STL from Phobos and drop the `.urdf` together with the mesh
  files; joints, limits and meshes become a robot with IK.
- **Motion from Blender** — export the animation as glTF and load it as an object today (geometry only); joint
  trajectories authored in Blender should go through ROS bags / CSV joint lists (`File › Open` accepts CSV joint
  targets) — the studio does not retarget arbitrary glTF animations onto robots.

## ROS / ROS 2

- Import: `.urdf`/`.xacro` (+ meshes, `package://` resolved from dropped files or the online library repositories).
- Export: URDF package (above); programs via the ROS 2 post (`ros2_control` JointTrajectory YAML/Python).
- Live: rosbridge client in the browser (`src/ros/rosbridge.ts`) and the ROS 2 driver on the server (`server/drivers/ros2.ts`).

## Visual Components / KUKA.Sim Pro

Visual Components (and KUKA.Sim, which is built on it) store components as `.vcmx` and layouts as `.vcm` zip
archives whose behaviours and geometry live in proprietary binary resources (`.rsc`) and Python scripts bound to
the VC API. There is no public specification, so:

- **Import (best effort)** — drop a `.vcmx` / `.vcm` / `.zip`: every standard mesh inside (STL/OBJ/DAE/glTF) is
  loaded; XML/JSON metadata is scanned for component names, node transforms (matrices or positions next to the
  mesh name) and joint/axis definitions (name, type, axis, limits), which are reported in the log so the
  mechanism can be rebuilt with `Tools › Mechanism builder`. Binary `.rsc` geometry cannot be read — export the
  component's geometry from VC/KUKA.Sim (*File › Export › Geometry* as STL/glTF) and drop that instead.
- **Concepts** — VC-style process modelling exists natively: process components (feeders, conveyors, buffers,
  processes, sinks, `src/vc/component.ts`), signals/threads in programs, mobile robots and fleets
  (`src/mobile`, `src/fleet`), KPIs. KUKA robots come from the online library (`kuka_experimental` URDFs) and
  programs export through the KUKA KRC4/KRC5 posts (`.src/.dat`) or imported RoboDK KUKA posts; a
  KUKA.Sim/OfficeLite exchange goes through those KRL files.
- **Export** — glTF (static or animated) and URDF packages are the exchange formats VC/KUKA.Sim can consume
  (VC imports glTF/STL/OBJ/DAE geometry; kinematics are re-created with VC's mechanism wizard).
- **KUKA Fleet / AMR** — the fleet manager is the studio's own (auctions, alley reservations, charging, KPIs) and
  speaks **VDA 5050** in both directions: master (dispatch orders to KUKA / MiR / any VDA 5050 vehicles and mirror
  them) and AGV bridge (the station's robots appear as VDA 5050 vehicles to KUKA Fleet or another master). See
  `docs/fleet-vda5050.md`. ROS 2 (Nav2) stays available through the ROS 2 driver and rosbridge.
