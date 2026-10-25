# Machine vision (СТЗ)

The vision stack runs on the station's camera items: it renders what the camera sees, detects / segments / tracks
objects, localises them in 3D, and turns the result into robot targets, program branches, ROS 2 topics or training data.
Code: `src/vision/*`, `src/agri/vision.ts`, `src/ui/vision_ui.ts`. Tests: `tests/vision_*.test.ts`.

## Camera model (one for everything)

`Camera.fov` spans `Camera.fovAxis` of the image:

| fovAxis | used by | meaning |
|---|---|---|
| `vertical` (default) | RoboDK `Cam2D_Add FOV=`, three.js | 2·atan(H/2 / f) |
| `horizontal` | Webots `fieldOfView`, URDF/SDF `horizontal_fov`, sensor datasheets (wizard) | 2·atan(W/2 / f) |

`cameraIntrinsics(cam, w?, h?)` (`src/vision/camera_model.ts`) is the only place K is derived: square pixels,
principal point at the image centre, pixel centres at +0.5. The renderer (`threeFovDeg`), the vision maths, the wizard
(`applySensorToCamera`), RoboDK `Cam2D_*` (`setFocalLengthPx` for `FOCAL_LENGTH`/`PIXELSIZE`), ROS `CameraInfo` and
overlays on canvases of another size (`rescalePixel`) all go through it. A Webots camera of 320×240 with
`fieldOfView 1.2` gets f = 160 / tan 0.6 = 233.87 px — the K webots_ros2 publishes.

## Pixel-true sensing

`src/vision/sensing.ts` turns the station into the same triangles the 3D view draws (primitives, STL/OBJ meshes,
robot links, tools, vehicles, orchard trunks / canopies / fruit) and renders, per camera:

- metric depth (mm along the optical +Z, 0 = no return),
- an instance-ID buffer (`object:<id>`, `robot:<id>`, `tool:<id>`, `vehicle:<id>`, `component:<id>`, `field:<id>:<part>`, `ground`),
- optionally a shaded RGB image.

In the browser `SceneRenderer.renderCameraBuffers` does it on the GPU (no antialiasing, 24-bit depth); Node, the server
and the headless API use the software rasterizer. Both agree to 99.8 % of the ID pixels and 0.02–0.08 mm mean depth.
Ground truth (`captureTruth`) comes from the ID buffer: exact boxes and masks, occlusion by robots, tools, meshes and
same-class objects; an object is visible with ≥ 20 px and ≥ 50 % of its unoccluded silhouette. `detectFruit` ray-casts
the same triangles, so walls and machines hide fruit (the canopy itself stays a statistical leaf model).

## Pipeline

`VisionRuntime.capture` → `run`: detect / segment / keypoints / classify (simulated from truth, ONNX Runtime Web,
studio server, ROS 2 `vision_msgs` or `yolo_msgs`, OpenAI-compatible VLM) → ByteTrack (α-β velocity filter) → 3D →
targets.

- Depth modalities (stereo, RGB-D, ToF) always produce a depth image (catalogue noise σ(z) when `simulate`), so real
  detectors get 3D too.
- `simulate: false` is an exact detector (truth boxes and masks, no false positives); without ground truth a simulated
  model reports nothing. With `simulate` on (the wizard default) boxes jitter like a detector trained on the task
  (corner σ = 4 % × (1 − mAP) of the box + 0.5 px, size σ = 5 % × (1 − mAP): IoU ≈ 0.9+ for a YOLOv10 on a 300 px
  part), small objects are missed and about 2 × (1 − mAP) false positives per frame appear on the background. The
  Vision tab statistics give the position error of the true positives and count the false positives, naming those that
  got a 3D position (`stats.fp3d`).
- A real runtime that is not configured (no ONNX model, no server, no topic, no VLM URL) fails with an explicit warning
  (`UnavailableModel`) instead of silently simulating.
- ONNX Runtime Web loads its WASM from the bundled package asset (no CDN).

3D localisation from depth (`src/vision/localize.ts`): the detection's pixels are split into depth layers and the
layer filling the box centre is kept (object, not table or leaves). Planes are fitted (RANSAC + least squares; thin
slivers rejected), the face a top grasp uses is chosen, grown beyond a loose box, and outlined with a minimum-area
rectangle. With an object model (truth dims or `objectDims`) truncated faces are completed and the centre is pushed
back by half the thickness. An object known only by a size prior is a face when its visible surface is one plane (a
box: the grasp is normal to the face) and round otherwise (fruit: the centre one radius behind the visible surface). The
face axis is folded by symmetry (90° square / 180° rectangle) towards the robot X axis — the "lazy" wrist of RoboDK
Example 06.c.

A detection gets no 3D position, hence no target, when nothing stands out of its support, whatever the detector
(simulated false positives, ONNX, server, ROS): the surface it lies on continues all around the box (in a ring of pixels
around it nothing lies behind the surface and most of it lies on it: a box on the bare table or inside a part's top
face), the face runs on past the detection window on three sides (a box across a part's edge), or the box / face is far
smaller than the object model or size prior (< 40 %, not cut by the image border).

Targets (`graspFromSurface`): TCP on the face centre + 2 mm, tool Z into the part, approach 100 mm back along the tool
axis. `detectionsToPickTargets(station, robot, targets)` checks IK (base-relative) and collisions at the grasp and
approach joints before anything is created; `createTargetsFromOutput` refreshes the frame "Vision <camera>" (in the
robot's active reference frame) and keeps `Pick` / `Pick approach` on the most confident feasible target (detector
score, then nearest to the camera), not on the first one in detector order.

## Vision in programs and the API

A code instruction `VisionDetect("<camera>"[, "<robot>"])` (aliases `Vision_Run`, `VisionRun`, `DetectParts`) runs the
pipeline on the current station state through the StationRuntime script runner (`installVisionRunner(runtime)`; the
app registers it globally at start). It refreshes the Pick targets and sets the signals / station params
`VISION_FOUND`, `VISION_<CAMERA>_FOUND`, `VISION_TARGET` for an IF instruction:

```
VisionDetect("Cell camera")
IF VISION_FOUND > 0 THEN PickPart ELSE NoPart      # PickPart: MoveJ Pick approach, MoveL Pick, attach, MoveL Pick approach
```

`runVisionCall` / `runVisionOnce` are the headless entry points; through the API, `RDK.Cam2D_Detect(cam[, robot])`
(studio extension) does the same and returns the number found (`RDK.getParam('VISION_FOUND')` afterwards).

## RoboDK camera API

`Cam2D_Add(frame, params)` understands `FOV` (vertical), `SIZE`, `FAR_LENGTH` and RoboDK's own `FAR_LENGHT` spelling,
`NEAR_LENGTH`, `FOCAL_LENGTH` + `PIXELSIZE`, `SNAPSHOT`, `DEPTH`, `BG_COLOR` (lights are accepted and ignored).
`Cam2D_Snapshot(file, cam)` saves a PNG (or RoboDK's grey32 depth for `.grey32` / `DEPTH`) and returns 1;
`Cam2D_Snapshot("", cam[, 'DEPTH'])` returns the bytes (PNG for `cv2.imdecode`, grey32 = big-endian u32 w, h, rows
bottom-up, depth / FAR_LENGTH · (2³² − 1), as `Macros/Depth_Camera.py` and `Open3D_Depth_Camera.py` decode it).
`Cam2D_Settings` returns RoboDK's own string, e.g. `FOV=32.000 PixelSize=7.168635 Size=640x480 SHOW_LIMITS
FOCAL_LENGTH=6.00 FAR_LENGTH=1000.00 BG_COLOR=#000000 Persp WindowResize` (`tests/vision_rdkcam.test.ts`).

Pixel parity with RoboDK 5.9.4 (the same scripts run against RoboDK and the studio; ground truth written by RoboDK in
`tests/fixtures/vision_gt/robodk_cam2d_depth`, checked by `tests/vision_cam2d_depth_parity.test.ts`):
- RGB snapshots are centre-sampled in both: V6's three-plate scene gives the same 11 729 lit pixels (XOR 0).
- DEPTH snapshots: RoboDK samples a depth pixel (−0.125, −0.375) px from its centre (v down) — sample 0 of the
  standard 4× MSAA pattern. Measured on planes tilted 20° about the camera y / x axis: −0.122 / −0.373 px at 640×480
  FOV 32 and −0.125 / −0.375 px at 320×240 FOV 50. The studio samples there too (`ROBODK_DEPTH_SAMPLE_OFFSET`, render
  option `sampleOffset`; the returned K has the shifted principal point). V6's scene now has the same 11 878 depth
  pixels as RoboDK (before: 149 missing, one column / row at the right and bottom edges); the top faces agree to
  0.02 mm (RoboDK writes 900.0145 for 900 mm), the row that sees a plate's side face to 0.1 mm.
- Depth snapshots are rendered by the software rasterizer in the browser too (exact sub-pixel sampling); RGB
  snapshots use the WebGL buffers when the 3D view registered them.

## ROS 2

- `publishCameraFrame(ros, cam, frame)`: `sensor_msgs/Image` (rgb8 / bgr8 / rgba8 / bgra8), `CameraInfo` with the render
  K (principal point W/2 − 0.5 in the ROS integer-pixel convention), 32FC1 depth in metres (NaN = no return) with its
  CameraInfo, and TF `map → <cam>_link → <cam>_optical_frame`. The Vision tab publishes them when "Publish to ROS 2" is on;
  rosbridge mode `sim` publishes every rgb / depth camera at 5 Hz (generic twin) or the MURKA head camera / range finder
  (5 Hz while subscribed, like the webots_ros2 devices of `murom_webots.urdf`; docs/ros.md).
- Generated parameter files are typed ROS 2 YAML (`toRosYaml`: doubles always with a decimal point, the integer
  parameters `imgsz_*`, `*_reliability`, `max_det`, `track_buffer`, `depth_image_units_divisor`, `min_points` as integers).
- `detectFruit` skips harvested (`picked`) fruit, so a detect → pick loop never re-detects them.
- Perception package export (`generateRosVisionPackage`): driver, yolo_ros, depth → 3D, cloud / VLM / VLA bridge nodes as
  ament_python `console_scripts`, hand-eye static TF `base_link` (eye-to-hand) or `tool0` (eye-in-hand) → `camera_link`
  (REP 103 body frame) → `camera_optical_frame`. Verified with `colcon build` + `ros2 pkg executables` on ROS 2 Jazzy.

## Calibration and datasets

- Hand-eye (camera properties › Hand-eye calibration…): the robot visits reachable poses, the camera measures a marker
  (TCP for eye-to-hand, a fixed object for eye-in-hand, occlusion and sensor noise simulated); Horn / linear + Gauss–Newton
  solvers; the result is stored as `cfg.handEye` and exported as the static TF.
- Dataset (camera properties › Dataset (YOLO / COCO)…): N frames with the selected objects randomised; labels from the
  ID buffer — YOLO boxes or polygons + `data.yaml`, or COCO with RLE masks (area = mask pixels).

## Limits

- Appearance is flat-shaded (no textures / lighting model): detectors trained on real images are exercised on geometry,
  not photorealism; the simulated detectors are statistical.
- Canopy occlusion of fruit is a density model, not leaf geometry.
- MURKA's `perception_node` has not been run against studio topics: the studio now serves the MURKA head camera
  (bgra8) and range finder (32FC1) through rosbridge (docs/ros.md "Studio as the MURKA simulator backend"), but the
  node's SKU colour prototypes are calibrated per simulator (`appearance_<sim>.yaml`) and none exists for the studio's
  flat-shaded renders.
