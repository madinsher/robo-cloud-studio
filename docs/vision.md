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
  model reports nothing.
- A real runtime that is not configured (no ONNX model, no server, no topic, no VLM URL) fails with an explicit warning
  (`UnavailableModel`) instead of silently simulating.
- ONNX Runtime Web loads its WASM from the bundled package asset (no CDN).

3D localisation from depth (`src/vision/localize.ts`): the detection's pixels are split into depth layers and the
layer filling the box centre is kept (object, not table or leaves). Planes are fitted (RANSAC + least squares; thin
slivers rejected), the face a top grasp uses is chosen, grown beyond a loose box, and outlined with a minimum-area
rectangle. With an object model (truth dims or `objectDims`) truncated faces are completed and the centre is pushed
back by half the thickness. Fruit (size prior only) take the centre one radius behind the visible surface. The face
axis is folded by symmetry (90° square / 180° rectangle) towards the robot X axis — the "lazy" wrist of RoboDK
Example 06.c.

Targets (`graspFromSurface`): TCP on the face centre + 2 mm, tool Z into the part, approach 100 mm back along the tool
axis. `detectionsToPickTargets(station, robot, targets)` checks IK (base-relative) and collisions at the grasp and
approach joints before anything is created; `createTargetsFromOutput` refreshes the frame "Vision <camera>" (in the
robot's active reference frame) and keeps `Pick` / `Pick approach` on the first feasible target.

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
`Cam2D_Settings` returns `FOV= FAR_LENGTH= NEAR_LENGTH= SIZE= [DEPTH]`.

## ROS 2

- `publishCameraFrame(ros, cam, frame)`: `sensor_msgs/Image` (rgb8 / bgr8 / rgba8 / bgra8), `CameraInfo` with the render
  K (principal point W/2 − 0.5 in the ROS integer-pixel convention), 32FC1 depth in metres (NaN = no return) with its
  CameraInfo, and TF `map → <cam>_link → <cam>_optical_frame`. The Vision tab publishes them when "Publish to ROS 2" is on.
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
- MURKA's `perception_node` has not been run against studio topics (formats verified; rosbridge not available in the
  test WSL).
