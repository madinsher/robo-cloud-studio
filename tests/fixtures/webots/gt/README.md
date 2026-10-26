# Webots pose ground truth (tests/webots_gt.test.ts)

`*.json.gz` were written by Webots R2025a itself: the supervisor controller `pose_dump.py` runs in batch mode
(`webots --batch --mode=fast --no-rendering --minimize`) on **copies** of the MURKA worlds and of Webots sample worlds
(`webots_*.json.gz`, from `C:/Program Files/Webots/projects`) and dumps, before the first physics step (initial state),

* every node reachable through `children` / `endPoint` (PROTO internals and template expansions included) with type,
  base type, DEF, name and — for Pose-derived nodes — the absolute 4x4 pose (row-major, metres);
* every `Shape` / `CadShape` with its geometry fields (Box size, Cylinder radius/height, Mesh/CadShape url, ...) and the
  pose of its nearest posed ancestor; `Transform` scales (Webots poses are scale-free);
* every `boundingObject` geometry of Solids / Robots / devices with its absolute pose;
* joint parameters (position, axis, anchor) and joint devices of every axis (motor / sensor names);
* the Webots node id of top-level nodes (`wid`, the `context.id` of PROTO templates).

`prep_world.py` makes the copy deterministic: `gravity 0`, every controller `<none>` (also on PROTO robots whose
default is `<extern>`), plus the `gt_pose_dump` supervisor robot.

Regenerate (Windows, Git Bash; Webots and a Python >= 3.8 — default `C:\RoboDK\Python-Embedded\python.exe`, override
with `GT_PYTHON`):

```sh
# copy each MURKA package (worlds/, protos/, and assets/ three levels above protos/) into a scratch folder first
bash run_all.sh <this folder> <scratch copy>/.../worlds/retail_store.wbt ...
gzip -9 -c <scratch copy>/.../worlds/gt_retail_store.wbt.json > retail_store_v2.json.gz
```

| fixture | MURKA world |
|---|---|
| retail_store_v2 | robot-retailer-murka-v2/ros2_ws/src/murom_webots/worlds/retail_store.wbt |
| retail_store_v1 | robot-retailer-murka/ros2_ws/src/murom_webots/worlds/retail_store.wbt |
| murka_sim_retail_store(_classic) | simulation_ws/src/MURKA_Simulation/ros2_ws/src/murka_simulation/worlds/ |
| retail_store_film | simulation_ws/src/murka_vla_study/experiments/materials/worlds/retail_store_film.wbt |
| legacy_retail_store | _legacy_2026-08/ros2_ws/src/murka_simulation/worlds/retail_store.wbt |
| orchard_demo, murca_garden, murca_apple_garden | simulation_ws/src/agromarca/murca_garden_ws/src/murca_webots/worlds/ |
| replication_murca_garden, replication_murca_apple_garden | simulation_ws/src/agromarca/article/replication/src/murca_webots/worlds/ |
| webots_&lt;world&gt; | the Webots sample world of that name (robots/abb/irb, robots/kuka/youbot, robots/gctronic/e-puck, robots/clearpath/moose, samples/demos, samples/geometries) |

Not dumped: `orchard_trellis_stills.wbt` (Webots did not finish loading its 75 MB of OBJ meshes within 5 min in batch
mode). Findings confirmed by these dumps: `RectangleArena { wallHeight 0 }` still has 0.1 m walls (Webots resets
non-positive sizes to the defaults); a world without EXTERNPROTO lines resolves the standard PROTOs by name.
