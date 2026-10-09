# File formats

| Extension | Direction | Description |
|---|---|---|
| `.vbstation` / `.json` | open/save | Native station: serialized item tree + base64 mesh assets (`src/io/station-file.ts`) |
| `.urdf`, `.xacro` | import | Robots via built-in URDF/xacro processor (properties, macros, includes, if/unless, math, quoted literals). Meshes referenced by `package://` are resolved by file name from dropped STL/DAE files, or downloaded automatically when the package belongs to a known GitHub repository (see *Online robot library*) |
| `.stl` | import | Binary/ASCII meshes (mm). Also used as robot link meshes |
| `.obj` | import | Wavefront meshes (triangulated on load) |
| `.glb`, `.gltf` | import/export | glTF 2.0: import flattens all meshes into one object (m → mm, Y-up → Z-up); export static scene (`Tools › Export scene as glTF`) or **animated** program simulation with keyframes on every link/tool/object for Blender (`Tools › Export animation as glTF`) |
| `.zip`, `.vcmx`, `.vcm` | import (best effort) | Zip containers incl. Visual Components / KUKA.Sim components and layouts: standard meshes inside are loaded, XML/JSON metadata scanned for names, transforms and joint definitions (see `docs/interop.md`) |
| URDF package (`.zip`) | export | `File › Export URDF package`: robot or whole station as `urdf/*.urdf` + `meshes/*.stl` (metres) + `package.xml` for ROS 2, Gazebo, MoveIt, Blender (Phobos) |
| `.py` (RoboDK post) | import | `Program › Import RoboDK post processors`: original RoboDK posts run unmodified in the browser (Pyodide, official robomath/robofileio, headless robodialogs); persisted per browser |
| `.dae` | import | COLLADA meshes (triangles/polylist/polygons/tristrips, node transforms, `<unit>` scale, Y-up → Z-up). Standalone imports are converted to mm; URDF link meshes are kept in metres and scaled by the visual |
| `.dh` / `.json` with `dh` | import/export | Robot from a standard DH table: `theta, d, a, alpha, lower, upper[, prismatic, home]` per joint (mm/deg) |
| `.rdk`, `.robot`, `.tool` | import | RoboDK containers. If the studio server runs next to a RoboDK installation (`STUDIO_ROBODK_PYTHON` + `robodk` package), the file is converted losslessly by `python/rdk2vbs.py` through `POST /convert/rdk` — frames, robots, tools, targets, programs (+ simulated joint path), objects with curves/points, machining projects (curve/point follow, 3D printing, milling), Python programs; otherwise a best-effort binary scan recovers names, poses and embedded meshes |
| `.src` / `.dat` | import/export | KUKA KRL |
| `.mod` / `.prg` | import/export | ABB RAPID |
| `.ls` | import/export | Fanuc TP (ASCII) |
| `.script` | import/export | Universal Robots URScript |
| `.JBI` | export | Yaskawa INFORM |
| `.pgx` | export | Stäubli VAL3 |
| `.drl` | export | Doosan DRL |
| `.py` | export | Mecademic Python, RoboDK API script (program or whole station), ROS 2 rclpy node |
| `.nc`, `.gcode`, `.ngc`, `.tap`, `.cnc`, `.gco` | import | NC / G-code (mm or inch, G0/G1/G2/G3, planes, F feeds, M3/M5 spindle, T tool changes, E extrusion for 3D printing) → object with rapid/cut curves + metadata; `Robot › Robot machining project` turns it into a program (speeds from feeds, spindle/extruder IO, approach/retract) |
| `.csv` / `.txt` | import/export | Targets (X,Y,Z,Rx,Ry,Rz[,joints]) in RoboDK / KUKA / Fanuc conventions; generic program CSV |
| `.json` | export | Program JSON (poses, quaternions, joints) and ROS 2 JointTrajectory |
| `.geojson` | import/export | Field boundaries (WGS84) → local mm polygons around a computed origin |
| `.png` | export | Viewport screenshot |

## Online robot library

`Add › Robot from online library (ROS-Industrial)…` downloads a robot description straight from GitHub
(`raw.githubusercontent.com`, CORS-enabled) and builds it in the browser:

1. the top-level xacro is fetched, `xacro:include` targets (`$(find pkg)/…`, `package://…`, relative paths) are
   resolved to the repository that hosts the package and fetched recursively — including includes whose file
   name is only known after macro expansion (the expansion is re-run until no include is missing);
2. the URDF is expanded with the built-in xacro processor and turned into a serial chain ending at the
   ROS-Industrial `tool0` frame (or `flange` / `tool_frame`), so the flange matches the vendor controller;
3. the visual meshes (`.stl`, `.dae`, `.obj`) are downloaded in parallel and registered under their
   `package://` URIs; COLLADA files keep their metre units.

Catalogue (`ONLINE_ROBOT_LIBRARY`, 91 robots): Fanuc (LR Mate 200i…200iD, M‑6iB, M‑10iA, M‑16iB, M‑20iA/iB,
M‑430iA, CR‑7iA, CR‑35iA, M‑710iC, M‑900iA/iB, R‑1000iA, R‑2000iC), ABB (IRB 120, 2400, 4400L, 5400, 6600, 6640),
KUKA (KR 3, KR 5 arc, KR 6/10 Agilus, KR 16, KR 120/150/210, LBR iiwa 14), Yaskawa Motoman (GP4…GP200R, HC10/HC20,
MH5…MH110, MA2010, SIA5/10/20), Stäubli (TX60, TX90, TX2‑60/90/90L, RX160), Universal Robots (UR3/5/10, UR3e/5e/10e),
Franka Emika Panda, Kinova Gen3 (6/7 axes) and Gen3 lite, Doosan (M0609…M1509, A0509/A0912, H2017/H2515).
Sources: `ros-industrial/{fanuc,abb,kuka_experimental,motoman,staubli_experimental,staubli,universal_robot}`,
`frankaemika/franka_ros`, `Kinovarobotics/ros_kortex`, `doosan-robotics/doosan-robot` (BSD/Apache licences).

The same resolver (`resolvePackageUri`) is used when a URDF dropped onto the viewport references
`package://` meshes of a known package — the missing meshes are fetched automatically.
Programmatic use: `fetchOnlineRobot(id, { assets })`, `fetchRobotFromUrl(url, …)` for any xacro/URDF URL.
