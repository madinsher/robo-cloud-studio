# Webots import (.wbt / .proto)

The studio opens Webots worlds and PROTO files natively (`src/io/webots/`, no Webots installation needed) and turns them
into an ordinary station: objects, colliders, mobile robots, arms, cameras and sensors that the studio's own simulator,
collision checker, vision and code generators use. Verified against **Webots R2025a itself** on the MURKA simulations
(see "Verification").

## Entry points

| function | use |
|---|---|
| `importWebotsFiles(files: Map<path, bytes>, assets, opts?)` (I7) | dropped folder / picker selection: imports the shallowest `.wbt` (or `opts.world`), else previews every `.proto` with its default fields; adds the semantic overlays below |
| `importWebotsWorld(worldPath, source, opts?)` | Node / server: lazy `WebotsSource` (`read(path)`, `list()`) over a directory |
| `buildStationItemsFromTree(tree, opts)` (`src/io/kintree.ts`, I3) | generic link/joint tree -> MobileRobot + one Robot per actuated serial chain; also used by the URDF / ROS package import |
| `loadRetailLayout`, `rasterizeOccupancy`, `upgradeOrchard` | P1 overlays (retail products/slots, occupancy map, orchard fields) |

Every import returns a `report` (one line per approximation or unsupported feature); nothing is dropped silently.

## Language support

* Lexer/parser: `#VRML_SIM` header, comments, strings with escapes, `EXTERNPROTO` / `IMPORTABLE EXTERNPROTO`, `DEF`/`USE`
  (shared instances), `IS`, `NULL`, MF values with or without brackets, PROTO interfaces (`field`, `vrmlField`,
  `hiddenField`, `deprecatedField`, enumerations `{ ... }` over several lines). Syntax errors carry the line number.
* PROTO resolution: EXTERNPROTO relative to the declaring file, `webots://` and `https://raw.githubusercontent.com/
  cyberbotics/webots/...` URLs mapped to clean-room built-ins, by-name fallback for worlds without EXTERNPROTO lines
  (what Webots does too), cycle guard, per-instance expansion cache. Any other standard PROTO (robots, objects) and the
  meshes / textures it references are read through an optional `remote(url)` fetcher, e.g. over the local Webots asset
  cache (`%LOCALAPPDATA%/Cyberbotics/Webots/cache/assets`, files named sha1(URL); `webots://` is mapped to the raw
  GitHub URL of the world's Webots version).
* JavaScript template PROTOs (`%< >%`, `%<= >%`): `fields.X.value / defaultValue` with Webots value shapes, `context`
  (world, proto, id, coordinate_system, webots_version, ...), the `wbgeometry` / `wbrotation` / `wbvector2` / `wbvector3` /
  `wbutility` / `wbrandom` modules with Webots' API and formulas (same circles, B-splines and LCG random sequence;
  compared call by call with Webots' own modules in tests/webots_template_modules.test.ts), template `console.error`
  lines in the report, isolated from studio globals and stopped after a step budget. Lua templates are reported.
* Built-in equivalents (written from the documented interfaces, no Webots asset source): `Floor`, `RectangleArena`,
  `TexturedBackground`, `TexturedBackgroundLight`, `SolidBox` and ~30 appearances (`Parquetry`, `BrushedAluminium`,
  `GalvanizedMetal`, ...; textures replaced by representative colours). Like Webots, non-positive sizes
  (`RectangleArena { wallHeight 0 }`) are reset to the field default with a report line.

## Node mapping

| Webots | studio |
|---|---|
| `WorldInfo`, `Viewpoint`, `Background`, lights, `Fog` | `station.settings.webots / viewpoint / background / lights / fog` |
| `Solid` (+ `Shape`, `Pose`, `Transform`, `Group` children) | `SceneObject` with primitives / meshes; Transform scale baked into sizes; nested Solids are child items |
| `Slot` | transparent: its `endPoint` is built in place (also as a joint endPoint) |
| Solid-derived devices outside robots (`Charger`, `Connector`, ...), `Fluid` | `SceneObject` (`params.webotsType`); a Fluid's boundingObject is its immersion volume, not a collider |
| `boundingObject` | `SceneObject.colliders` (I9: collision uses them instead of the visuals; Plane bounds tag the ground) |
| `Physics` | `mass`, `params.physics` |
| `Box`, `Cylinder`, `Sphere`, `Cone`, `Capsule`, `Plane`, `Mesh`, `CadShape`, `IndexedFaceSet`, `ElevationGrid` | primitives / registered meshes (mm); OBJ+MTL and COLLADA materials, UVs and textures kept |
| `Robot` with joints | link/joint tree -> `MobileRobot` (differential / Ackermann / omni detected from wheel motors: wheel radius, track, wheel base, max steer) + one `Robot` per maximal actuated serial chain (mounted on the flange of its parent chain); motor names = joint names; limits / velocities from the motors |
| `Camera`, `RangeFinder` | `Camera` with `fovAxis: 'horizontal'` (I4), the Webots +X optical axis converted to the studio +Z |
| `Lidar`, `GPS`, `InertialUnit`, `Gyro`, `Accelerometer`, ... | sensors on the right link (`params.webotsDevice`) |
| supervisor robots without geometry | frames (controller name kept) |

## Semantic overlays (P1)

* **Retail layout** (`murom_world/config/products.yaml` + `locations.yaml` next to the world): products spawned
  through the world's IMPORTABLE product PROTO (`RetailProduct`) at `slot + size.z/2 + 2 mm` exactly like the MURKA
  supervisor, slot / approach / standoff / dock / home `Target`s, a charging `ZoneItem` (dock radius), a packing zone and
  basket targets on the robot. Products have no collider (as in Webots).
* **Occupancy** `rasterizeOccupancy(station, {zMin, zMax, resolution})`: `MapItem` from colliders intersecting a height
  band (robots, decals, ground excluded).
* **Orchard** `upgradeOrchard`: tree Solids grouped into `FieldItem`s with `CropRow`s; apples of the tree geometry become
  fruit of the plant records.

## Scripted fleet player (MURKA Mission B)

`src/io/webots/scripted_fleet.ts` is a port of the agromarca `garden_supervisor/scripted_fleet.py` (v2): the same
script JSON (`waypoints` with `interp` / `tangent` heading, `linear` / `trapezoid` profiles and `forward` / `reverse` /
`auto` drive, `attachments`, `carries`, `reconfigs`), the same tick semantics and the same visible wheel / steering
state. `StationFleetWorld` binds it to an imported station (node names resolve by name, then by DEF), and
`playScriptedFleet(station, script, {tEnd, dt})` runs it like the Webots supervisor (t = n * dt).
`defaultMissionBScript()` (`murka_missions.ts`) is Mission B of the article as data:

```ts
const { station } = importWebotsWorld('src/murca_webots/worlds/orchard_demo.wbt', source, { assets });
playScriptedFleet(station, defaultMissionBScript(), { tEnd: 92, dt: 0.016, depotModules: MISSION_B.DEPOT_MODULES });
```

## Viewpoints and shots

`viewpointLook(vp)` turns a Webots viewpoint (R2022a+: looks along its +X, +Z up, horizontal fieldOfView) into eye /
target / up for a renderer camera. `parseShots` + `applyShot` play the MURKA trellis stills
(`orchard_trellis_stills_shots.json`): the shot's `DYN_<KEY>` group comes up to z = 0, the other groups stay parked at
-1000 m and `station.settings.viewpoint` takes the shot's camera.

## Verification

* `tests/webots_gt.test.ts` — **ground truth from Webots R2025a** on 43 worlds (11 MURKA worlds and 32 Webots sample worlds
  with standard robots: ABB IRB 4600 cell, KUKA youBot, e-puck family, Clearpath Moose, hexapod, Stewart platform, ...).
  A supervisor (`tests/fixtures/webots/gt/pose_dump.py`)
  dumped every node pose, shape and boundingObject of 11 MURKA worlds in batch mode. The studio reproduces every visual
  shape (template expansions, PROTO internals, CadShapes, all MUROM meshes at the initial joints) within 0.5 mm, every
  static collider within 0.5 mm, and every robot motor as exactly one studio joint (MUROM: 16).
* `tests/webots_mission_b.test.ts` — the scripted-fleet port equals the replay of the real Python supervisor
  (`mission_b_reference.json.gz`: final poses, 187 trace samples, per-tick steps, wheel fields, plan report); on the
  imported `orchard_demo.wbt` the final poses are within 1 cm, and no platform, crate or module touches a trunk, fence,
  the base, the depot or a rock along the 92 s replay.
* `tests/webots_murka.test.ts` — all 12 worlds + 27 PROTOs parse with node counts equal to the inventory and import
  without error lines; MUROM v2 decomposition (differential base, 6 + 6 DOF arms, 2 DOF head, head camera, lidar),
  UTP-G Ackermann variants, colliders (trunk-only trees), trellis OBJ materials.
* `tests/webots_parser|scene|kintree|layout|render.test.ts` — synthetic fixtures (`tests/fixtures/webots/mini`).

## Limitations (reported, not silent)

* Pre-R2022a worlds (NUE axes) and non-ENU `coordinateSystem` are not converted.
* Lua template PROTOs; remote PROTO / mesh URLs when no `remote` fetcher (or no cached copy) is available — 124 of the
  303 Webots sample worlds import without an error line against this machine's cache, the others miss cached PROTOs.
* Templates that seed `wbrandom` with `context.id` (the Webots node id, e.g. `WoodenPalletStack`) get a different random
  jitter than in Webots: the id depends on Webots' internal node creation order.
* Lights inside robots / LEDs, `Skin`, `TrackWheel`, `IndexedLineSet` / `PointSet` (not rendered).
* `Mesh` sub-mesh selection (`name` / `materialIndex`), tracked drives (`Track` = static), closed loops
  (`SolidReference`), joints without motors (fixed), nested Robots (fixed links).
* Textures of standard appearances are approximated by colours; the controllers themselves are not run — behaviour is
  reproduced with the studio's programs, control models (Petri nets / automata) and ROS bridge.
