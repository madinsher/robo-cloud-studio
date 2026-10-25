/**
 * T9 vision — ROS 2: the exported perception package (hand-eye TF conventions, ament_python console_scripts, exec bits)
 * and the studio camera streams (sensor_msgs/Image, CameraInfo with the render K, 32FC1 metric depth, camera TF).
 * Ported from tests/_probe/vision_ros_export.test.ts (2 probes; the tilted-camera probe compared camera_link with the
 * optical pose — the correct invariant is camera_link · R(link→optical) = studio camera pose) + new checks.
 * Optional: builds the package with colcon in WSL ROS 2 Jazzy (skipped when WSL is absent).
 */
import { it, expect, describe } from 'vitest';
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import { execFileSync, spawnSync } from 'node:child_process';
import { unzipSync } from 'fflate';
import { Station, Camera, SceneObject, Frame } from '../src/core/items/item';
import { createRobotFromLibrary } from '../src/core/items/library';
import { mul, transl, rotx, roty, rotz, DEG, Mat4, multiply, invert } from '../src/core/math/pose';
import { generateRosVisionPackage, rosVisionPackageZip, cameraInfoMsg, depthImageMsg, imageMsg, cameraTfMsg, publishCameraFrame, rosRpy, LINK_TO_OPTICAL, studioCameraTopics } from '../src/vision/vision_ros';
import { RosTwin, cameraHasColour } from '../src/ros/twin';
import { setVisionStack, getVisionStack, VisionStackConfig } from '../src/vision/stack';
import { renderCamera } from '../src/vision/sensing';

/** What tf2 static_transform_publisher --x --y --z --roll --pitch --yaw builds: T · Rz(yaw) · Ry(pitch) · Rx(roll). */
const rosTf = (x: number, y: number, z: number, r: number, p: number, yw: number): Mat4 => mul(transl(x * 1000, y * 1000, z * 1000), rotz(yw), roty(p), rotx(r));
const cfgOf = (over: Partial<VisionStackConfig> = {}): VisionStackConfig => ({ tasks: ['detect', 'pose'], modality: 'rgbd', environment: 'factory', sensor: 'realsense_d455', compute: 'intel_ipc', models: { detect: 'yolov8s', pose: 'depth_centroid' }, classes: ['box'], confidence: 0.4, iou: 0.5, runtime: 'simulated', workingDistance: 1.2, simulate: true, handEyeMode: 'eye_to_hand', ...over });
function launchTfs(cam: Camera) {
  const pkg = generateRosVisionPackage(cam, getVisionStack(cam)!);
  const launch = pkg.files['launch/perception.launch.py'];
  const m = /name='hand_eye_tf', arguments=\['--x', '([-\d.]+)', '--y', '([-\d.]+)', '--z', '([-\d.]+)', '--roll', '([-\d.]+)', '--pitch', '([-\d.]+)', '--yaw', '([-\d.]+)', '--frame-id', '(\w+)'/.exec(launch)!;
  const o = /name='optical_tf', arguments=\['--roll', '([-\d.]+)', '--pitch', '([-\d.]+)', '--yaw', '([-\d.]+)'/.exec(launch)!;
  const v = m.slice(1, 7).map(Number);
  const link = rosTf(v[0], v[1], v[2], v[3], v[4], v[5]);
  const optical = mul(link, rosTf(0, 0, 0, Number(o[1]), Number(o[2]), Number(o[3])));
  return { pkg, parent: m[7], link, optical };
}
const maxDiff = (a: ArrayLike<number>, b: ArrayLike<number>, n = 12) => { let d = 0; for (let i = 0; i < n; i++) if (i % 4 !== 3) d = Math.max(d, Math.abs(a[i] - b[i]) / (i >= 12 ? 1000 : 1)); return d; };

describe('exported perception package: hand-eye TF', () => {
  it('camera looking straight down: the exported optical frame +Z looks down like the studio camera', () => {
    const cam = new Station().addChild(new Camera('Cell camera'));
    cam.setPose(mul(transl(600, 0, 1900), rotx(180 * DEG)));
    setVisionStack(cam, cfgOf());
    const { optical, link, parent } = launchTfs(cam);
    expect(parent).toBe('world'); // no robot in the station
    expect(optical[10]).toBeLessThan(-0.9999);
    expect(link[2]).toBeLessThan(-0.9999); // camera_link +X (forward, REP 103) points down too
  });

  it('tilted camera (rotz 30 · rotx 150): camera_link · R_link→optical reproduces the studio pose (Euler convention)', () => {
    const cam = new Station().addChild(new Camera('Cell camera'));
    const pose = mul(transl(600, 0, 1900), rotz(30 * DEG), rotx(150 * DEG));
    cam.setPose(pose);
    setVisionStack(cam, cfgOf());
    const { optical } = launchTfs(cam);
    expect(maxDiff(optical, pose, 16)).toBeLessThan(1e-3); // rotation 1e-3, translation 1 µm·1000 (printed in m with 4 decimals)
  });

  it('eye-to-hand with a robot: parent base_link, pose relative to the robot base', () => {
    const st = new Station();
    const robot = st.addChild(createRobotFromLibrary('UR10e'));
    robot.setPose(mul(transl(200, -300, 700), rotz(40 * DEG)));
    const cam = st.addChild(new Camera('Cell camera'));
    const pose = mul(transl(900, 100, 1800), rotz(-20 * DEG), rotx(170 * DEG));
    cam.setPose(pose);
    setVisionStack(cam, cfgOf());
    const { optical, parent } = launchTfs(cam);
    expect(parent).toBe('base_link');
    expect(maxDiff(multiply(robot.poseAbs(), optical), pose, 16)).toBeLessThan(1e-3);
  });

  it('eye-in-hand: parent tool0 (the flange), not the TCP / camera parent', () => {
    const st = new Station();
    const robot = st.addChild(createRobotFromLibrary('UR10e'));
    robot.setJoints([10, -80, -100, -60, 90, 20]);
    const holder = robot.addChild(new Frame('Holder'));
    const cam = holder.addChild(new Camera('Wrist camera'));
    holder.setPose(transl(0, 0, 0));
    cam.setPose(mul(transl(0, 80, 60), rotx(-10 * DEG)));
    setVisionStack(cam, cfgOf({ handEyeMode: 'eye_in_hand' }));
    const { optical, parent } = launchTfs(cam);
    expect(parent).toBe('tool0');
    const flange = multiply(robot.poseAbs(), robot.fk().flange);
    expect(maxDiff(multiply(flange, optical), cam.poseAbs(), 16)).toBeLessThan(1e-3);
  });

  it('rosRpy is the inverse of Rz(yaw)·Ry(pitch)·Rx(roll), including the gimbal case', () => {
    for (const [r, p, y] of [[0.3, -0.4, 2.0], [-2.5, 1.2, -0.7], [0.2, Math.PI / 2, 0.5]]) {
      const R = mul(rotz(y), roty(p), rotx(r));
      const [rr, pp, yy] = rosRpy(R);
      expect(maxDiff(mul(rotz(yy), roty(pp), rotx(rr)), R)).toBeLessThan(1e-9);
    }
    // camera_link -> optical: x_link = z_optical (forward), y_link = -x_optical (left), z_link = -y_optical (up)
    expect(Array.from(LINK_TO_OPTICAL).slice(0, 11).map((v) => Math.round(v))).toEqual([0, -1, 0, 0, 0, 0, -1, 0, 1, 0, 0]);
  });

  it('ament_python package: every node is a console_scripts entry with main(), scripts are 0755 in the zip', () => {
    const cam = new Station().addChild(new Camera('Cell camera'));
    setVisionStack(cam, cfgOf({ tasks: ['detect', 'cloud_detect', 'vlm_query', 'vla_policy'], vlm: { url: 'http://x/v1', model: 'm' } as any, vla: { url: 'http://y', model: 'pi0', format: 'openpi' } as any }));
    const pkg = generateRosVisionPackage(cam, getVisionStack(cam)!);
    const nodes = Object.keys(pkg.files).filter((f) => /^scripts\/.+\.py$/.test(f)).map((f) => f.slice(8, -3));
    expect(nodes.sort()).toEqual(['cloud_pipeline', 'vla_bridge', 'vlm_bridge']);
    const setup = pkg.files['setup.py'];
    for (const n of nodes) {
      expect(setup).toContain(`'${n} = ${pkg.name}.${n}:main'`);
      expect(pkg.files[`${pkg.name}/${n}.py`]).toMatch(/\ndef main\(/);
      expect(pkg.files['launch/perception.launch.py']).toContain(`executable='${n}'`);
    }
    expect(pkg.files[`${pkg.name}/__init__.py`]).toBe('');
    expect(pkg.files['setup.cfg']).toContain(`install_scripts=$base/lib/${pkg.name}`);
    // zip entries keep Unix modes: read the central directory external attributes
    const zip = rosVisionPackageZip(pkg);
    const dv = new DataView(zip.buffer, zip.byteOffset, zip.byteLength);
    const modes = new Map<string, number>();
    for (let i = 0; i + 46 < zip.length; i++) if (dv.getUint32(i, true) === 0x02014b50) {
      const nlen = dv.getUint16(i + 28, true), elen = dv.getUint16(i + 30, true), clen = dv.getUint16(i + 32, true);
      const name = new TextDecoder().decode(zip.subarray(i + 46, i + 46 + nlen));
      modes.set(name, dv.getUint32(i + 38, true) >>> 16);
      i += 45 + nlen + elen + clen;
    }
    expect(modes.get(`${pkg.name}/scripts/cloud_pipeline.py`)! & 0o777).toBe(0o755);
    expect(modes.get(`${pkg.name}/package.xml`)! & 0o777).toBe(0o644);
    expect(Object.keys(unzipSync(zip)).length).toBe(Object.keys(pkg.files).length);
  });
});

describe('studio camera → ROS 2 messages', () => {
  function scene() {
    const st = new Station();
    const box = st.addChild(new SceneObject('Box'));
    box.geometry = [{ primitive: { kind: 'box', size: [200, 200, 100] }, color: '#3366ff' }];
    box.setPose(transl(50, -30, 50));
    const cam = st.addChild(new Camera('head_camera'));
    cam.fov = 1.2 / DEG; cam.fovAxis = 'horizontal'; cam.width = 320; cam.height = 240; cam.near = 100; cam.far = 5000;
    cam.setPose(mul(transl(0, 0, 1100), rotx(180 * DEG)));
    return { st, cam };
  }
  it('32FC1 depth is metric (m, NaN = no return) and matches the render pixel by pixel', () => {
    const { st, cam } = scene();
    const r = renderCamera(st, cam);
    const msg = depthImageMsg(r.depth, r.width, r.height, 'head_camera_optical_frame', 12.5);
    expect([msg.encoding, msg.step, msg.is_bigendian, msg.width, msg.height]).toEqual(['32FC1', 320 * 4, 0, 320, 240]);
    expect(msg.header.stamp).toEqual({ sec: 12, nanosec: 500000000 });
    const f = new Float32Array(new Uint8Array(Buffer.from(msg.data, 'base64')).buffer);
    const c = 120 * 320 + 160;
    expect(f[c]).toBeCloseTo(1.0, 5); // box top at z = 100, camera at 1100
    expect(Number.isNaN(f[0])).toBe(true); // background: no measurement
    let maxErr = 0; for (let i = 0; i < f.length; i++) if (r.depth[i] > 0) maxErr = Math.max(maxErr, Math.abs(f[i] * 1000 - r.depth[i]));
    expect(maxErr).toBeLessThan(1e-3);
  });
  it('bgra8 image (MURKA perception input) carries the rendered colours; publishCameraFrame sends Image, CameraInfo, depth, TF', () => {
    const { st, cam } = scene();
    const r = renderCamera(st, cam, { rgb: true });
    const img = imageMsg(r.rgba!, r.width, r.height, 'f', 0, 'bgra8');
    const px = new Uint8Array(Buffer.from(img.data, 'base64'));
    const c = (120 * 320 + 160) * 4;
    expect([px[c + 2], px[c + 1], px[c]]).toEqual([r.rgba![c], r.rgba![c + 1], r.rgba![c + 2]]); // B,G,R order
    expect(r.rgba![c + 2]).toBeGreaterThan(r.rgba![c]); // the blue box
    const sent: Array<[string, string, any]> = [];
    const topics = publishCameraFrame({ publish: (t, ty, m) => sent.push([t, ty, m]) }, cam, { rgba: r.rgba, depth: r.depth, timeS: 3 }, { encoding: 'bgra8', topics: { image: '/head_camera/image_color', info: '/head_camera/camera_info', depth: '/head_depth/image' } });
    expect(topics).toEqual(['/head_camera/image_color', '/head_camera/camera_info', '/head_depth/image', '/head_camera/depth/camera_info', '/tf']);
    const info = sent.find((s) => s[0] === '/head_camera/camera_info')![2];
    expect(info.k[0]).toBeCloseTo(160 / Math.tan(0.6), 9);
    expect(info.header.frame_id).toBe('head_camera_optical_frame');
  });
  it('camera TF: world → <cam>_link → <cam>_optical_frame reconstructs the studio camera pose', () => {
    const cam = new Camera('Cam');
    const pose = mul(transl(1200, -400, 900), rotz(25 * DEG), roty(-15 * DEG), rotx(160 * DEG));
    new Station().addChild(cam).setPose(pose);
    const tf = cameraTfMsg(cam, { worldFrame: 'map' });
    const toMat = (t: any): Mat4 => {
      const { x, y, z, w } = t.transform.rotation, tr = t.transform.translation;
      const m = transl(tr.x * 1000, tr.y * 1000, tr.z * 1000);
      m[0] = 1 - 2 * (y * y + z * z); m[1] = 2 * (x * y + z * w); m[2] = 2 * (x * z - y * w);
      m[4] = 2 * (x * y - z * w); m[5] = 1 - 2 * (x * x + z * z); m[6] = 2 * (y * z + x * w);
      m[8] = 2 * (x * z + y * w); m[9] = 2 * (y * z - x * w); m[10] = 1 - 2 * (x * x + y * y);
      return m;
    };
    expect(tf.transforms.map((t: any) => [t.header.frame_id, t.child_frame_id])).toEqual([['map', 'cam_link'], ['cam_link', 'cam_optical_frame']]);
    expect(maxDiff(multiply(toMat(tf.transforms[0]), toMat(tf.transforms[1])), pose, 16)).toBeLessThan(1e-9);
    expect(cameraInfoMsg(cam, 'x').header.frame_id).toBe('x');
    void invert;
  });
});

// V5 F12 / V6 F5: an RGB-D camera (wizard rgbd / stereo / ToF -> kind 'depth') has a colour stream in the ROS twin, on the
// topics the exported perception package subscribes to with source:=studio; the launch survives missing packages
describe('RGB-D camera: studio twin topics = exported perception package topics', () => {
  function cell() {
    const st = new Station('cell');
    const box = st.addChild(new SceneObject('Box'));
    box.geometry = [{ primitive: { kind: 'box', size: [200, 200, 100] }, color: '#3366ff' }];
    box.setPose(transl(0, 0, 50));
    const cam = st.addChild(new Camera('Cell camera'));
    cam.kind = 'depth'; cam.fov = 60; cam.width = 160; cam.height = 120; cam.near = 100; cam.far = 4000;
    cam.setPose(mul(transl(0, 0, 1200), rotx(180 * DEG)));
    setVisionStack(cam, cfgOf({ tasks: ['detect', 'pose'] }));
    return { st, cam };
  }
  it('the twin publishes colour (rgb8) + 32FC1 depth for a kind "depth" camera; a depth-only device publishes no colour', () => {
    const { st, cam } = cell();
    const sent: Array<{ topic: string; type: string; msg: any }> = [];
    const twin = new RosTwin(st, { publish: (topic, type, msg) => sent.push({ topic, type, msg }), subscribe: () => undefined }, { cameras: true });
    twin.start();
    const topics = twin.publishCameras();
    const t = studioCameraTopics(cam);
    expect(t.image).toBe('/cell_camera/image_raw');
    expect(topics).toEqual(expect.arrayContaining([t.image, t.info, t.depth, t.depthInfo]));
    const img = sent.find((x) => x.topic === t.image)!.msg;
    expect([img.encoding, img.width, img.height, img.header.frame_id]).toEqual(['rgb8', 160, 120, 'cell_camera_optical_frame']);
    const px = new Uint8Array(Buffer.from(img.data, 'base64'));
    const c = (60 * 160 + 80) * 3; // image centre: the blue box top
    expect(px[c + 2]).toBeGreaterThan(px[c]);
    expect(sent.find((x) => x.topic === t.depth)!.msg.encoding).toBe('32FC1');
    // a Webots RangeFinder / Gazebo depth_camera has no colour stream (like webots_ros2 / ros_gz)
    for (const p of [{ webotsDevice: 'RangeFinder' }, { sensorType: 'depth_camera' }, { depthOnly: true }]) {
      const c2 = cell();
      Object.assign(c2.cam.params, p);
      expect(cameraHasColour(c2.cam)).toBe(false);
      const sent2: string[] = [];
      const tw2 = new RosTwin(c2.st, { publish: (topic) => sent2.push(topic), subscribe: () => undefined }, { cameras: true });
      tw2.start();
      tw2.publishCameras();
      expect(sent2).not.toContain(t.image);
      expect(sent2).toContain(t.depth);
    }
    expect(cameraHasColour(Object.assign(new Camera('rgbd'), { kind: 'depth' as const }))).toBe(true);
  });

  it('exported launch: source:=studio remaps to the twin topics, the driver / yolo_ros are optional, yolo_ros is declared', () => {
    for (const sensor of ['realsense_d455', 'zivid']) {
      const { cam } = cell();
      setVisionStack(cam, cfgOf({ sensor, tasks: ['detect', 'track', 'pose', 'grasp'] }));
      const pkg = generateRosVisionPackage(cam, getVisionStack(cam)!);
      const launch = pkg.files['launch/perception.launch.py'];
      const t = studioCameraTopics(cam);
      expect(launch).toContain(`'studio': {'image': '${t.image}', 'info': '${t.info}', 'depth': '${t.depth}', 'depth_info': '${t.depthInfo}'}`);
      expect(launch).toMatch(/DeclareLaunchArgument\('source', default_value='driver', choices=\['driver', 'studio'\]/);
      // every non-core package behind opt(): a stock Jazzy install launches (static TFs) and logs what is missing
      const driver = sensor === 'zivid' ? 'zivid_camera' : 'realsense2_camera';
      expect(launch).toContain(`nodes += opt('${driver}', lambda:`);
      for (const ex of ['yolo_node', 'tracking_node', 'detect_3d_node', 'debug_node']) expect(launch).toMatch(new RegExp(`opt\\('yolo_ros', lambda: Node\\(package='yolo_ros', executable='${ex}'`));
      expect(launch).not.toMatch(/^\s*nodes\.append\(Node\(package='(yolo_ros|zivid_camera|realsense2_camera)'/m);
      expect(launch).toContain("remappings=[('image_raw', T['image'])]");
      expect(launch).toContain("remappings=[('depth_image', T['depth']), ('depth_info', T['depth_info'])]");
      expect(pkg.files['package.xml']).toContain('<exec_depend>yolo_ros</exec_depend>');
      expect(pkg.files['package.xml']).toContain(`<exec_depend>${driver}</exec_depend>`);
      expect(pkg.files['README.md']).toContain('source:=studio');
    }
  });
});

// ------------------------------------------------------------------------------------------------
// colcon build in WSL ROS 2 Jazzy (ground truth for "ros2 run / launch find the executables")
// ------------------------------------------------------------------------------------------------
const wslOk = (() => {
  if (process.platform !== 'win32' || process.env.VBS_NO_WSL) return false;
  try { return spawnSync('wsl.exe', ['-d', 'Ubuntu-22.04', '--', 'test', '-f', '/opt/ros/jazzy/setup.bash'], { timeout: 30000, env: { ...process.env, MSYS_NO_PATHCONV: '1' } }).status === 0; } catch { return false; }
})();
it.skipIf(!wslOk)('WSL ROS 2 Jazzy: colcon builds the exported package and ros2 lists its executables', () => {
  const cam = new Station().addChild(new Camera('Cell camera'));
  setVisionStack(cam, cfgOf({ tasks: ['detect', 'cloud_detect', 'vlm_query'], vlm: { url: 'http://x/v1', model: 'm' } as any }));
  const pkg = generateRosVisionPackage(cam, getVisionStack(cam)!);
  const dir = fs.mkdtempSync(path.join(os.tmpdir(), 'vbs-rosvision-'));
  try {
    fs.writeFileSync(path.join(dir, 'pkg.zip'), rosVisionPackageZip(pkg));
    const wdir = '/mnt/' + dir.replace(/\\/g, '/').replace(/^([A-Za-z]):/, (_, d: string) => d.toLowerCase());
    const script = `set -e; source /opt/ros/jazzy/setup.bash; W=$(mktemp -d); mkdir -p $W/src; cd $W/src; python3 -c "import zipfile,sys; zipfile.ZipFile(sys.argv[1]).extractall('.')" '${wdir}/pkg.zip'; cd $W; colcon build --packages-select ${pkg.name} > build.log 2>&1 || (cat build.log; exit 1); source install/setup.bash; ros2 pkg executables ${pkg.name}; python3 -c "import ast,sys; [ast.parse(open(f).read()) for f in sys.argv[1:]]" src/${pkg.name}/${pkg.name}/*.py && echo SYNTAX_OK`;
    fs.writeFileSync(path.join(dir, 'run.sh'), script);
    const out = execFileSync('wsl.exe', ['-d', 'Ubuntu-22.04', '--', 'bash', `${wdir}/run.sh`], { timeout: 240000, env: { ...process.env, MSYS_NO_PATHCONV: '1' } }).toString();
    for (const n of ['cloud_pipeline', 'vlm_bridge']) expect(out).toContain(`${pkg.name} ${n}`);
    expect(out).toContain('SYNTAX_OK');
  } finally { fs.rmSync(dir, { recursive: true, force: true }); }
}, 300000);

// V5 F12: "ros2 launch <pkg> perception.launch.py" on a stock Jazzy (no zivid_camera, no yolo_ros) used to die with
// "package 'zivid_camera' not found"; now it starts the static TFs and logs the missing packages, in both sources
it.skipIf(!wslOk)('WSL ROS 2 Jazzy: the exported perception launch starts without the driver / yolo_ros installed (source driver and studio)', () => {
  const cam = new Station().addChild(new Camera('Camera 1'));
  cam.kind = 'depth';
  setVisionStack(cam, cfgOf({ sensor: 'zivid', tasks: ['detect', 'track', 'pose', 'grasp'] }));
  const pkg = generateRosVisionPackage(cam, getVisionStack(cam)!);
  const dir = fs.mkdtempSync(path.join(os.tmpdir(), 'vbs-rosvision-launch-'));
  try {
    fs.writeFileSync(path.join(dir, 'pkg.zip'), rosVisionPackageZip(pkg));
    const wdir = '/mnt/' + dir.replace(/\\/g, '/').replace(/^([A-Za-z]):/, (_, d: string) => d.toLowerCase());
    const script = [
      'set -e', 'source /opt/ros/jazzy/setup.bash', 'export ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1', 'W=$(mktemp -d)', 'mkdir -p $W/src', 'cd $W/src',
      `python3 -c "import zipfile,sys; zipfile.ZipFile(sys.argv[1]).extractall('.')" '${wdir}/pkg.zip'`, 'cd $W',
      `colcon build --packages-select ${pkg.name} > build.log 2>&1 || (cat build.log; exit 1)`, 'source install/setup.bash',
      'set +e',
      `echo "=== driver"; timeout -s INT 12 ros2 launch ${pkg.name} perception.launch.py 2>&1 | head -40`,
      `echo "=== studio"; timeout -s INT 12 ros2 launch ${pkg.name} perception.launch.py source:=studio 2>&1 | head -40`,
      'rm -rf $W', 'echo DONE',
    ].join('\n');
    fs.writeFileSync(path.join(dir, 'run.sh'), script);
    const out = execFileSync('wsl.exe', ['-d', 'Ubuntu-22.04', '--', 'bash', `${wdir}/run.sh`], { timeout: 280000, env: { ...process.env, MSYS_NO_PATHCONV: '1' } }).toString();
    const [driver, studio] = out.split('=== studio');
    expect(out).toContain('DONE');
    expect(out).not.toMatch(/not found|Caught exception|Traceback/);
    expect(driver).toMatch(/package 'zivid_camera' is not installed: skipped/);
    expect(driver).toMatch(/static_transform_publisher-\d+\]: process started|process started with pid/);
    expect(studio).toMatch(/perception source: studio \(image \/camera_1\/image_raw, depth \/camera_1\/depth\/image_raw\)/);
    expect(studio).not.toMatch(/zivid_camera/);
    expect(out).toMatch(/package 'yolo_ros' is not installed: skipped/);
  } finally { fs.rmSync(dir, { recursive: true, force: true }); }
}, 300000);
