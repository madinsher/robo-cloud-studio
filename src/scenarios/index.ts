/**
 * Demo scenarios: reproducible end-to-end cases for every navigation / localization method and every
 * machine-vision task. Each scenario builds a small station, runs the simulation headlessly (Node or browser),
 * measures what matters for that method (localization RMSE, path deviation, lost events, detection precision /
 * recall, 3D position error, row-line error, follow range error, VLA convergence…) and reports pass / fail
 * against an expectation derived from the catalogue (sensor error model, method accuracy and drift).
 *
 *  - `npm run scenarios` runs everything and writes `docs/scenario-results.md` (published on Read the Docs);
 *  - `tests/scenarios.test.ts` asserts every scenario passes in CI;
 *  - *Help › Demo scenarios…* in the studio loads a scenario into the 3D view or runs the whole suite.
 */
import { Station, Camera as CameraItem, Frame, SceneObject, ItemType } from '../core/items/item';
import { mul, multiply, transl, rotx, rotz, roty, DEG, Vec3 } from '../core/math/pose';
import { MobileRobot, MapItem, ZoneItem } from '../mobile/items';
import { followPath, stepMobile, integrate } from '../mobile/controller';
import { FieldItem, cropParams } from '../agri/items';
import { generateOrchard, rectPolygon, buildFieldMap, makeCanopyZones } from '../agri/orchard';
import { LOCALIZATION_METHODS, NAVIGATION_METHODS, NavStackConfig, PlatformType, Environment, SensorKind, setNavStack, stepNavRuntime, NavRuntime, PLATFORM_DEFAULT_SENSORS } from '../mobile/navstack';
import { VisionStackConfig, VisionTask, setVisionStack, getVisionStack, VISION_SENSORS } from '../vision/stack';
import { VisionRuntime, followCommand, createTargetsFromOutput, PipelineOutput } from '../vision/pipeline';
import { intrinsicsFromCamera } from '../vision/camera_model';
import { pointCount } from '../vision/pointcloud';

// ---------------------------------------------------------------------------------------------
// Types
// ---------------------------------------------------------------------------------------------

export interface ScenarioMetric { name: string; value: number | string; unit?: string; /** Pass criterion text, when the metric is checked. */ bound?: string; ok?: boolean }
export type ScenarioGroup = 'navigation' | 'vision' | 'control' | 'group';
export interface ScenarioResult { id: string; title: string; group: ScenarioGroup; method: string; pass: boolean; metrics: ScenarioMetric[]; notes: string[]; durationMs: number }
export interface Scenario {
  id: string;
  title: string;
  group: ScenarioGroup;
  /** Catalogue method(s) exercised. */
  method: string;
  description: string;
  /** How to reproduce it by hand in the studio. */
  howTo: string;
  /** Build the station (used by the UI to load the scenario and by run()). */
  build(): { station: Station; focus?: { id: string } };
  run(opts?: { seed?: number }): Promise<ScenarioResult>;
}

const M = (v: number, d = 0) => Number(v.toFixed(d));
const metric = (name: string, value: number | string, unit?: string, ok?: boolean, bound?: string): ScenarioMetric => ({ name, value, unit, ok, bound });

// ---------------------------------------------------------------------------------------------
// Scenes
// ---------------------------------------------------------------------------------------------

/** 30 × 20 m warehouse: perimeter walls, four rack rows, a loop path around them. */
function warehouseScene(): { station: Station; map: MapItem; path: number[][]; start: [number, number, number] } {
  const st = new Station('Warehouse scenario');
  const map = new MapItem('Warehouse map'); map.resize(300, 200, 100, 0, 0); st.addChild(map);
  map.fillRect(0, 0, 30000, 200); map.fillRect(0, 19800, 30000, 20000); map.fillRect(0, 0, 200, 20000); map.fillRect(29800, 0, 30000, 20000);
  for (let i = 0; i < 4; i++) map.fillRect(6000 + i * 6000, 5000, 6000 + i * 6000 + 1200, 15000); // racks
  map.fillRect(2000, 9000, 3000, 11000); // pillar
  const path = [[3500, 3000], [26500, 3000], [26500, 17000], [3500, 17000], [3500, 3000]];
  return { station: st, map, path, start: [3500, 3000, 0] };
}

/** Apple orchard 60 × 24 m (6 rows), occupancy map, canopy GNSS-denied zones; path: two alleys with a headland turn. */
function orchardScene(): { station: Station; map: MapItem; zones: ZoneItem[]; field: FieldItem; path: number[][]; start: [number, number, number] } {
  const st = new Station('Orchard scenario');
  const field = st.addChild(new FieldItem('Block'));
  field.polygon = rectPolygon(60000, 24000);
  field.crop = cropParams('apple', { headland: 5000 });
  const rows = generateOrchard(field, 5);
  const map = buildFieldMap(field, 250, 6000); st.addChild(map);
  const zones = makeCanopyZones(field); for (const z of zones) field.addChild(z);
  const half = field.crop.rowSpacing / 2;
  const r0 = rows[1], r1 = rows[2];
  const a = r0.travelLine(-half), b = r1.travelLine(-half);
  const path = [[a.start[0] - 3000, a.start[1]], [a.end[0] + 3000, a.end[1]], [b.end[0] + 3000, b.end[1]], [b.start[0] - 3000, b.start[1]]];
  return { station: st, map, zones, field, path, start: [path[0][0], path[0][1], 0] };
}

/** Open field: no map, 3 × 80 m coverage passes (GNSS everywhere). */
function openFieldScene(): { station: Station; map: MapItem | null; path: number[][]; start: [number, number, number] } {
  const st = new Station('Open field scenario');
  const path: number[][] = [];
  for (let i = 0; i < 3; i++) { const y = i * 6000; path.push(i % 2 ? [80000, y] : [0, y]); path.push(i % 2 ? [0, y] : [80000, y]); }
  return { station: st, map: null, path, start: [0, 0, 0] };
}

/** Greenhouse: 6 gutters 30 m long, pipe rails between them (1-D travel along one rail). */
function greenhouseScene(): { station: Station; map: MapItem; path: number[][]; start: [number, number, number] } {
  const st = new Station('Greenhouse scenario');
  const map = new MapItem('Greenhouse map'); map.resize(360, 160, 100, 0, 0); st.addChild(map);
  map.fillRect(0, 0, 36000, 200); map.fillRect(0, 15800, 36000, 16000); map.fillRect(0, 0, 200, 16000); map.fillRect(35800, 0, 36000, 16000);
  for (let i = 0; i < 6; i++) map.fillRect(3000, 2000 + i * 2200, 33000, 2000 + i * 2200 + 700);
  const y = 2000 + 2 * 2200 + 700 + 750; // rail between gutter 2 and 3
  const path = [[3500, y], [32500, y]];
  return { station: st, map, path, start: [3500, y, 0] };
}

// ---------------------------------------------------------------------------------------------
// Navigation scenarios — one per localization method
// ---------------------------------------------------------------------------------------------

interface NavCase { localization: string; navigation: string; platform: PlatformType; environment: Environment; sensors: SensorKind[]; scene: 'warehouse' | 'orchard' | 'open_field' | 'greenhouse'; title: string; description: string; gnssAvailability?: number }

const NAV_CASES: NavCase[] = [
  { localization: 'slam_toolbox_2d', navigation: 'nav2_navfn_dwb', platform: 'amr', environment: 'warehouse', sensors: ['wheel_odom', 'imu', 'lidar2d'], scene: 'warehouse', title: 'AMR in a warehouse — 2D LiDAR SLAM (slam_toolbox) + Nav2 NavFn/DWB', description: 'Differential AMR loops around four rack rows; the estimate comes from 2D LiDAR SLAM with loop closures.' },
  { localization: 'cartographer_2d', navigation: 'nav2_smac2d_mppi', platform: 'amr', environment: 'factory', sensors: ['wheel_odom', 'imu', 'lidar2d'], scene: 'warehouse', title: 'AMR in a factory — Cartographer 2D + Smac 2D/MPPI', description: 'Same loop with Cartographer and an MPPI controller.' },
  { localization: 'laser_reflectors', navigation: 'nav2_navfn_dwb', platform: 'agv_natural', environment: 'warehouse', sensors: ['wheel_odom', 'lidar2d', 'reflectors'], scene: 'warehouse', title: 'AGV — laser triangulation with reflectors', description: 'Natural-navigation AGV localised against surveyed reflectors.' },
  { localization: 'magnetic_tape', navigation: 'line_following', platform: 'agv_line', environment: 'warehouse', sensors: ['wheel_odom', 'magnetic_tape'], scene: 'warehouse', title: 'AGV — magnetic tape line following', description: 'Fixed route on tape: centimetre lateral accuracy, no global drift.' },
  { localization: 'qr_grid', navigation: 'line_following', platform: 'agv_line', environment: 'warehouse', sensors: ['wheel_odom', 'imu', 'qr_grid'], scene: 'warehouse', title: 'AGV — QR floor grid', description: 'Dead reckoning between floor fiducials, absolute fix at each code.' },
  { localization: 'uwb', navigation: 'nav2_navfn_dwb', platform: 'amr', environment: 'greenhouse', sensors: ['wheel_odom', 'imu', 'uwb'], scene: 'greenhouse', title: 'Greenhouse AMR — UWB beacons', description: 'UWB trilateration (decimetre noise) fused with odometry.' },
  { localization: 'hybrid_uwb_lidar', navigation: 'nav2_navfn_dwb', platform: 'amr', environment: 'greenhouse', sensors: ['wheel_odom', 'imu', 'uwb', 'lidar2d'], scene: 'greenhouse', title: 'Greenhouse AMR — UWB + 2D LiDAR SLAM hybrid', description: 'UWB bounds the global error, LiDAR SLAM smooths it.' },
  { localization: 'rail_encoder', navigation: 'rail_1d', platform: 'rail', environment: 'greenhouse', sensors: ['encoder_rail'], scene: 'greenhouse', title: 'Pipe-rail trolley — rail encoder', description: 'One-dimensional position on a pipe rail from the wheel encoder.' },
  { localization: 'wheel_imu_dr', navigation: 'nav2_navfn_dwb', platform: 'amr', environment: 'warehouse', sensors: ['wheel_odom', 'imu'], scene: 'warehouse', title: 'AMR — wheel odometry + IMU dead reckoning only', description: 'Reference case: drift grows with distance, no corrections (expected to deviate).' },
  { localization: 'gnss_rtk', navigation: 'gnss_waypoints', platform: 'tractor', environment: 'open_field', sensors: ['wheel_odom', 'imu', 'gnss_rtk'], scene: 'open_field', title: 'Tractor in an open field — RTK GNSS waypoints', description: 'Coverage passes with RTK fix everywhere.' },
  { localization: 'gnss_ins', navigation: 'gnss_waypoints', platform: 'tractor', environment: 'open_field', sensors: ['wheel_odom', 'imu', 'gnss'], scene: 'open_field', title: 'Tractor — standard GNSS/INS', description: 'Metre-level GNSS smoothed by INS.' },
  { localization: 'gnss_rtk', navigation: 'row_following', platform: 'tracked', environment: 'orchard', sensors: ['wheel_odom', 'imu', 'gnss_rtk'], scene: 'orchard', title: 'Orchard platform — RTK only under canopy (outages)', description: 'RTK loses the fix inside the canopy zones and dead-reckons: shows why RTK alone is not enough in orchards.' },
  { localization: 'hybrid_rtk_lio', navigation: 'row_following', platform: 'tracked', environment: 'orchard', sensors: ['wheel_odom', 'imu', 'gnss_rtk', 'lidar3d'], scene: 'orchard', title: 'Orchard platform — RTK + LiDAR-inertial hybrid', description: 'LIO bridges the GNSS-denied rows; the recommended orchard stack.' },
  { localization: 'hybrid_rtk_vio', navigation: 'row_following', platform: 'tracked', environment: 'vineyard', sensors: ['wheel_odom', 'imu', 'gnss_rtk', 'stereo_camera'], scene: 'orchard', title: 'Vineyard platform — RTK + visual-inertial hybrid', description: 'Camera-based odometry bridges the outages (daylight).' },
  { localization: 'lio_sam', navigation: 'nav2_smac_hybrid_rpp', platform: 'tracked', environment: 'orchard', sensors: ['wheel_odom', 'imu', 'lidar3d'], scene: 'orchard', title: 'Orchard platform — LIO-SAM (no GNSS)', description: '3D LiDAR-inertial SLAM with loop closures at the headlands.' },
  { localization: 'fast_lio2', navigation: 'row_following', platform: 'tracked', environment: 'orchard', sensors: ['imu', 'lidar3d'], scene: 'orchard', title: 'Orchard platform — FAST-LIO2 odometry', description: 'Tightly coupled LiDAR-inertial odometry; drift accumulates without loop closure.' },
  { localization: 'kiss_icp', navigation: 'row_following', platform: 'tracked', environment: 'forest', sensors: ['lidar3d'], scene: 'orchard', title: 'Forest / orchard — KISS-ICP LiDAR odometry', description: 'LiDAR-only odometry, no IMU.' },
  { localization: 'rtabmap_rgbd', navigation: 'nav2_navfn_dwb', platform: 'amr', environment: 'greenhouse', sensors: ['wheel_odom', 'rgbd_camera'], scene: 'greenhouse', title: 'Greenhouse AMR — RTAB-Map RGB-D SLAM', description: 'RGB-D SLAM indoors with loop closures.' },
  { localization: 'orb_slam3_mono', navigation: 'row_following', platform: 'amr', environment: 'orchard', sensors: ['mono_camera'], scene: 'orchard', title: 'Orchard — monocular ORB-SLAM3 (scale drift)', description: 'Single camera: scale drifts and tracking can be lost in feature-poor headlands.' },
  { localization: 'orb_slam3_vi', navigation: 'row_following', platform: 'amr', environment: 'orchard', sensors: ['mono_camera', 'imu'], scene: 'orchard', title: 'Orchard — visual-inertial ORB-SLAM3', description: 'IMU fixes the scale; drift bounded by loop closures.' },
  { localization: 'openvins', navigation: 'row_following', platform: 'amr', environment: 'vineyard', sensors: ['stereo_camera', 'imu'], scene: 'orchard', title: 'Vineyard — OpenVINS visual-inertial odometry (no GPS)', description: 'VIO without any global reference: drift proportional to distance.' },
  { localization: 'vins_fusion', navigation: 'row_following', platform: 'amr', environment: 'orchard', sensors: ['stereo_camera', 'imu', 'gnss'], scene: 'orchard', title: 'Orchard — VINS-Fusion with standard GNSS', description: 'VIO fused with metre-level GNSS at the headlands.' },
];

function methodExpectation(cfg: NavStackConfig, distanceMm: number, gnssFraction: number): { rmseBoundMm: number; text: string } {
  const m = LOCALIZATION_METHODS.find((x) => x.id === cfg.localization)!;
  const acc = m.accuracy * 1000;
  const noFixDistance = m.global ? (m.family === 'gnss_rtk' || m.family === 'gnss_ins' || m.family === 'hybrid' ? distanceMm * (1 - gnssFraction) : 0) : distanceMm;
  const drift = m.driftPerMeter * noFixDistance;
  const loopClosure = m.family.endsWith('slam') || m.family === 'lio' || m.family === 'rgbd_slam' ? 0.5 : 1; // loop closures cut the accumulated drift
  const bound = Math.max(150, 3 * acc + drift * loopClosure);
  return { rmseBoundMm: bound, text: `≤ ${M(bound)} mm (3σ ${M(3 * acc)} + drift ${M(drift * loopClosure)} over ${M(noFixDistance / 1000, 1)} m without global fix)` };
}

function makeNavScenario(c: NavCase): Scenario {
  const id = `nav_${c.localization}${NAV_CASES.filter((x) => x.localization === c.localization).length > 1 ? `_${c.scene}` : ''}`;
  const build = () => {
    const scene = c.scene === 'warehouse' ? warehouseScene() : c.scene === 'orchard' ? orchardScene() : c.scene === 'greenhouse' ? greenhouseScene() : openFieldScene();
    const st = scene.station;
    const r = new MobileRobot(c.platform === 'tractor' ? 'Tractor' : c.platform === 'rail' ? 'Rail trolley' : c.platform.startsWith('agv') ? 'AGV' : c.platform === 'tracked' ? 'Tracked platform' : 'AMR');
    r.kin.drive = c.platform === 'tracked' ? 'tracked' : 'differential';
    r.kin.maxSpeed = c.platform === 'tractor' ? 2000 : c.platform === 'rail' ? 600 : 1200;
    r.kin.maxYawRate = 60;
    st.addChild(r);
    r.setPose2D(scene.start[0], scene.start[1], scene.start[2]);
    const cfg: NavStackConfig = { platform: c.platform, environment: c.environment, sensors: c.sensors, localization: c.localization, navigation: c.navigation, fusion: c.sensors.filter((s) => s === 'wheel_odom' || s === 'imu'), simulate: true, gnssAvailability: c.gnssAvailability };
    setNavStack(r, cfg);
    return { station: st, robot: r, map: scene.map, zones: ((scene as any).zones ?? []) as ZoneItem[], path: scene.path };
  };
  return {
    id, title: c.title, group: 'navigation', method: c.localization, description: c.description,
    howTo: `Mobile & Fleet › Add mobile robot (${c.platform}) → Navigation & SLAM stack… → environment "${c.environment}", sensors ${c.sensors.join(', ')} → pick "${LOCALIZATION_METHODS.find((m) => m.id === c.localization)?.name}" → enable "Simulate localization" → Mobile & Fleet › Start world simulation → Navigation tab.`,
    build: () => { const b = build(); return { station: b.station, focus: b.robot }; },
    async run() {
      const t0 = Date.now();
      const { station, robot: r, map, zones, path } = build();
      followPath(r, path);
      const dt = 0.05;
      let rt: NavRuntime | null = null;
      let maxDev = 0, devSum = 0, devN = 0, gnssOn = 0, lostSteps = 0, steps = 0;
      const devTo = (x: number, y: number) => { let best = Infinity; for (let i = 1; i < path.length; i++) { const [ax, ay] = path[i - 1], [bx, by] = path[i]; const vx = bx - ax, vy = by - ay, l2 = vx * vx + vy * vy || 1; const t = Math.max(0, Math.min(1, ((x - ax) * vx + (y - ay) * vy) / l2)); best = Math.min(best, Math.hypot(x - (ax + vx * t), y - (ay + vy * t))); } return best; };
      for (let i = 0; i < 12000 && r.state.path; i++) {
        rt = stepNavRuntime(r, dt, map, zones);
        const e = rt?.estimator.state;
        const pose = e && !e.lost ? { x: e.x, y: e.y, theta: e.theta } : undefined;
        if (e?.lost) { integrate(r, { v: 0, omega: 0 }, dt); lostSteps++; }
        else stepMobile(r, dt, { pose });
        steps++;
        if (e?.gnssAvailable) gnssOn++;
        const d = devTo(r.state.x, r.state.y); maxDev = Math.max(maxDev, d); devSum += d; devN++;
        void station;
      }
      const e = rt!.estimator.state;
      const dist = r.state.odometer;
      const gnssFraction = steps ? gnssOn / steps : 0;
      const exp = methodExpectation(getNavStackOf(r), dist, gnssFraction);
      const m = LOCALIZATION_METHODS.find((x) => x.id === c.localization)!;
      const arrived = !r.state.path;
      const rmseOk = e.rmse <= exp.rmseBoundMm;
      const devBound = m.global && m.family !== 'gnss_ins' ? 1500 : 4000;
      // relative (drifting) methods are expected to leave the path: the deviation is reported, only global methods are judged on it
      const devOk = !m.global || maxDev <= devBound || !arrived;
      const metrics: ScenarioMetric[] = [
        metric('distance', M(dist / 1000, 1), 'm'), metric('arrived', arrived ? 'yes' : 'no (stopped)'),
        metric('localization RMSE', M(e.rmse), 'mm', rmseOk, exp.text), metric('max error', M(e.maxError), 'mm'),
        metric('path deviation max', M(maxDev), 'mm', m.global ? devOk : undefined, m.global ? `≤ ${devBound} mm when arrived` : 'reported only (relative method)'), metric('path deviation mean', M(devN ? devSum / devN : 0), 'mm'),
        metric('lost events', e.lostEvents), metric('lost time', M(steps ? (100 * lostSteps) / steps : 0), '%'), metric('loop closures / fixes', e.fixes),
        metric('GNSS fix time', M(100 * gnssFraction), '%'), metric('scale', M(e.scale, 3)),
      ];
      if (rt!.slam) metrics.push(metric('SLAM map explored', M(100 * rt!.slam.coverage()), '%'), metric('SLAM map agreement', M(100 * rt!.slam.agreement(map!)), '%'));
      const notes: string[] = [];
      if (!arrived) notes.push('the robot stopped before the end of the path (lost localization or blocked)');
      if (m.family === 'gnss_rtk' && gnssFraction < 0.8 && c.scene === 'orchard') notes.push('RTK without odometry bridging: expected to degrade under the canopy — compare with hybrid_rtk_lio');
      if (!m.global) notes.push('relative method: error grows with distance, no global frame');
      return { id, title: c.title, group: 'navigation', method: c.localization, pass: rmseOk && devOk, metrics, notes, durationMs: Date.now() - t0 };
    },
  };
}
function getNavStackOf(r: MobileRobot): NavStackConfig { return r.params.navStack as unknown as NavStackConfig; }

export const NAV_SCENARIOS: Scenario[] = NAV_CASES.map(makeNavScenario);

// ---------------------------------------------------------------------------------------------
// Vision scenarios
// ---------------------------------------------------------------------------------------------

function orchardCameraStation(opts: { kind?: CameraItem['kind']; fov?: number; width?: number; height?: number; lateral?: number; height_mm?: number } = {}): { station: Station; cam: CameraItem; field: FieldItem } {
  const st = new Station('Orchard vision scenario');
  const field = st.addChild(new FieldItem('Block'));
  field.polygon = rectPolygon(24000, 12000);
  field.crop = cropParams('apple', { headland: 2000 });
  const rows = generateOrchard(field, 3);
  const row = rows[0];
  const [x, y] = row.pointAt(row.length() / 2, opts.lateral ?? -1500);
  const cam = st.addChild(new CameraItem('Camera'));
  cam.fov = opts.fov ?? 70; cam.width = opts.width ?? 640; cam.height = opts.height ?? 480; cam.far = 8000; cam.kind = opts.kind ?? 'depth';
  cam.setPose(mul(transl(x, y, opts.height_mm ?? 1500), rotz(90 * DEG), roty(90 * DEG)));
  return { station: st, cam, field };
}

/** Bin-picking cell: table with boxes, eye-to-hand RGB-D camera above. */
function binPickingStation(): { station: Station; cam: CameraItem; objects: SceneObject[] } {
  const st = new Station('Bin picking scenario');
  const table = st.addChild(new Frame('Table')); table.setPose(transl(600, 0, 700));
  const objects: SceneObject[] = [];
  const spec: Array<[string, number, number, number, [number, number, number]]> = [['Box 1', -150, -120, 15, [120, 80, 60]], ['Box 2', 60, -100, 0, [100, 100, 50]], ['Box 3', -40, 120, 40, [90, 60, 70]], ['Bottle 1', 180, 80, 0, [60, 60, 180]], ['Box 4', 170, -140, -30, [80, 120, 40]]];
  for (const [name, x, y, rz, size] of spec) {
    const o = table.addChild(new SceneObject(name));
    o.geometry = [{ primitive: { kind: 'box', size } }];
    o.setPose(mul(transl(x, y, size[2] / 2), rotz(rz * DEG)));
    objects.push(o);
  }
  const cam = st.addChild(new CameraItem('Cell camera'));
  cam.kind = 'depth'; cam.fov = 60; cam.width = 1280; cam.height = 720; cam.near = 100; cam.far = 3000;
  cam.setPose(mul(transl(600, 0, 1900), rotx(180 * DEG)));
  return { station: st, cam, objects };
}

const vcfg = (over: Partial<VisionStackConfig>): VisionStackConfig => ({ tasks: ['detect'], modality: 'stereo', environment: 'orchard', sensor: 'zed_x', compute: 'jetson_orin_nx', models: { detect: 'yolov8s' }, classes: ['apple'], confidence: 0.35, iou: 0.5, runtime: 'simulated', workingDistance: 1.5, simulate: true, seed: 5, ...over });

function detectionStats(rt: VisionRuntime): { precision: number; recall: number; posErr: number } {
  const s = rt.stats;
  return { precision: s.tp + s.fp ? s.tp / (s.tp + s.fp) : 0, recall: s.tp + s.fn ? s.tp / (s.tp + s.fn) : 0, posErr: s.posErrN ? s.posErrSum / s.posErrN : NaN };
}

function visionScenario(def: { id: string; title: string; method: string; description: string; howTo: string; build: () => { station: Station; cam: CameraItem; extra?: any }; cfg: VisionStackConfig; evaluate: (ctx: { station: Station; cam: CameraItem; rt: VisionRuntime; extra?: any }) => Promise<{ metrics: ScenarioMetric[]; notes?: string[] }> }): Scenario {
  return {
    id: def.id, title: def.title, group: 'vision', method: def.method, description: def.description, howTo: def.howTo,
    build: () => { const b = def.build(); setVisionStack(b.cam, def.cfg); return { station: b.station, focus: b.cam }; },
    async run(opts = {}) {
      const t0 = Date.now();
      const b = def.build();
      setVisionStack(b.cam, { ...def.cfg, seed: opts.seed ?? def.cfg.seed });
      const rt = new VisionRuntime(b.cam, getVisionStack(b.cam)!);
      const { metrics, notes = [] } = await def.evaluate({ station: b.station, cam: b.cam, rt, extra: b.extra });
      const pass = metrics.every((m) => m.ok !== false);
      return { id: def.id, title: def.title, group: 'vision', method: def.method, pass, metrics, notes, durationMs: Date.now() - t0 };
    },
  };
}

export const VISION_SCENARIOS: Scenario[] = [
  visionScenario({
    id: 'vis_stereo_detect_track_pose', title: 'Orchard — stereo camera (ZED X): YOLO detection + ByteTrack + 3D positions', method: 'yolov8s + bytetrack + depth_centroid',
    description: 'Camera 1.5 m from an apple row. Detections are tracked over 10 frames and localised in 3D through the stereo error model; grasp/approach targets are created for the picker.',
    howTo: 'Add › Camera / vision sensor near a row → Tools › Machine vision stack… → tasks detect, track, 6D pose, grasp; modality stereo → Apply → Vision tab › Run / live → Targets → station.',
    build: () => orchardCameraStation(), cfg: vcfg({ tasks: ['detect', 'track', 'pose', 'grasp'], models: { detect: 'yolov8s', track: 'bytetrack', pose: 'depth_centroid', grasp: 'approach_grasp' } }),
    evaluate: async ({ station, cam, rt }) => {
      let out: PipelineOutput | null = null;
      for (let i = 0; i < 10; i++) out = await rt.step(station, { time: i / 15 });
      const s = detectionStats(rt);
      const sensor = VISION_SENSORS.find((x) => x.id === 'zed_x')!;
      const sigma = (sensor.depthError!.a + sensor.depthError!.b * 2.25) * 1000;
      const targets = createTargetsFromOutput(station, cam, out!, {});
      const tracked = out!.detections.filter((d) => d.id !== undefined).length;
      return { metrics: [metric('frames', rt.stats.frames), metric('precision', M(100 * s.precision), '%', s.precision >= 0.8, '≥ 80 %'), metric('recall (objects ≥ 10 px)', M(100 * s.recall), '%', s.recall >= 0.5, '≥ 50 %'), metric('3D position error', M(s.posErr), 'mm', s.posErr <= 4 * sigma + 30, `≤ 4σ + 30 mm (σ ${M(sigma)} mm at 1.5 m)`), metric('tracked ids in last frame', tracked, undefined, tracked > 3, '> 3'), metric('targets created', targets.length / 2, 'pairs', targets.length > 0, '> 0')] };
    },
  }),
  visionScenario({
    id: 'vis_mono_size_prior', title: 'Orchard — mono global-shutter camera: detection + range from the fruit size prior', method: 'yolov8s + size_prior_position',
    description: 'No depth sensor: range comes from z = f·D/w with the crop diameter as prior. Shows the accuracy you get from a single camera.',
    howTo: 'Same as the stereo case with modality "Mono RGB camera" and "Object size prior" = fruit diameter.',
    build: () => orchardCameraStation({ kind: 'rgb' }), cfg: vcfg({ tasks: ['detect', 'pose'], modality: 'mono', sensor: 'gs_rgb', models: { detect: 'yolov8s', pose: 'size_prior_position' }, objectSizeMm: 75 }),
    evaluate: async ({ station, rt }) => {
      const out = await rt.step(station, { time: 0 });
      const truthById = new Map(out.frame.truth!.map((t) => [t.id, t]));
      const rel = out.detections.filter((b) => b.z && b.truthId).map((b) => Math.abs(b.z! - truthById.get(b.truthId!)!.z) / truthById.get(b.truthId!)!.z);
      const mean = rel.reduce((a, b) => a + b, 0) / Math.max(1, rel.length);
      return { metrics: [metric('detections', out.detections.length, undefined, out.detections.length > 5, '> 5'), metric('range error (relative)', M(100 * mean, 1), '%', mean < 0.3, '< 30 %'), metric('3D position error', M(detectionStats(rt).posErr), 'mm')], notes: ['range from size prior: ±(fruit size spread) — use it for approach, not for the final grasp'] };
    },
  }),
  visionScenario({
    id: 'vis_seg_keypoints_classify', title: 'Orchard — segmentation masks, stem keypoints and ripeness classification', method: 'yolov8_seg + yolov8_pose + efficientnet',
    description: 'Instance masks for fruit, stem keypoints for the cutting point, ripeness attribute per detection; classification confusion follows the model quality.',
    howTo: 'Machine vision stack… → tasks segmentation, keypoints, classification (+ detect); any modality.',
    build: () => orchardCameraStation(), cfg: vcfg({ tasks: ['detect', 'segment', 'keypoints', 'classify', 'pose'], models: { detect: 'yolov8s', segment: 'yolov8_seg', keypoints: 'yolov8_pose', classify: 'efficientnet', pose: 'depth_centroid' } }),
    evaluate: async ({ station, rt }) => {
      const out = await rt.step(station, { time: 0 });
      const truthById = new Map(out.frame.truth!.map((t) => [t.id, t]));
      const masks = out.detections.filter((b) => b.mask).length, kps = out.detections.filter((b) => b.keypoints?.length).length;
      const kpErr = out.detections.filter((b) => b.keypoints?.length && b.truthId).map((b) => { const t = truthById.get(b.truthId!)!; return Math.hypot(b.keypoints![0][0] - t.keypoints![0][0], b.keypoints![0][1] - t.keypoints![0][1]); });
      const meanKp = kpErr.reduce((a, b) => a + b, 0) / Math.max(1, kpErr.length);
      const withRipe = out.detections.filter((b) => b.attr && 'ripe' in b.attr && b.truthId);
      const ripeOk = withRipe.filter((b) => b.attr!.ripe === truthById.get(b.truthId!)!.attrs!.ripe).length;
      const acc = withRipe.length ? ripeOk / withRipe.length : 0;
      return { metrics: [metric('masks', masks, undefined, masks > 3, '> 3'), metric('keypoint sets', kps, undefined, kps > 3, '> 3'), metric('stem keypoint error', M(meanKp, 1), 'px', meanKp < 6, '< 6 px'), metric('ripeness accuracy', M(100 * acc), '%', acc >= 0.6, '≥ 60 %'), metric('image labels', out.results.classify?.labels?.map((l) => l.cls).slice(0, 3).join(', ') ?? '–')] };
    },
  }),
  visionScenario({
    id: 'vis_rgbd_bin_picking', title: 'Cell — RGB-D camera above a table: boxes → 6D grasp targets', method: 'yolov8s + depth_centroid + approach_grasp',
    description: 'Eye-to-hand RealSense 1.2 m above five parts. Every part gets a grasp pose (tool Z along the camera ray) and an approach pose 100 mm above.',
    howTo: 'Add › Box (several) on a frame, Add › Camera above them (Rx 180°) → Machine vision stack… → environment factory, tasks detect, 6D pose, grasp, modality RGB-D → Run → Targets → station → use the targets in a program.',
    build: () => { const b = binPickingStation(); return { station: b.station, cam: b.cam, extra: b.objects }; },
    cfg: vcfg({ tasks: ['detect', 'pose', 'grasp'], modality: 'rgbd', environment: 'factory', sensor: 'realsense_d455', compute: 'intel_ipc', models: { detect: 'yolov8s', pose: 'depth_centroid', grasp: 'approach_grasp' }, classes: ['box', 'bottle'], workingDistance: 1.2 }),
    evaluate: async ({ station, cam, rt, extra }) => {
      const objects = extra as SceneObject[];
      let out: PipelineOutput | null = null;
      for (let i = 0; i < 3; i++) out = await rt.step(station, { time: i / 10 });
      const truthById = new Map(out!.frame.truth!.map((t) => [t.id, t]));
      const errs = out!.detections.filter((b) => b.p && b.truthId).map((b) => { const t = truthById.get(b.truthId!)!; return Math.hypot(b.p![0] - t.p[0], b.p![1] - t.p[1], b.p![2] - t.p[2]); });
      const mean = errs.reduce((a, b) => a + b, 0) / Math.max(1, errs.length);
      const targets = createTargetsFromOutput(station, cam, out!, {});
      const grasp = out!.targets[0];
      const zDown = grasp ? grasp.grasp[10] : 0;
      return { metrics: [metric('objects in view', out!.frame.truth!.filter((t) => t.visible).length), metric('detected', out!.detections.length, undefined, out!.detections.length >= objects.length - 1, `≥ ${objects.length - 1}`), metric('3D position error', M(mean), 'mm', mean < 60, '< 60 mm (≈ 3σ, RealSense σ ≈ 19 mm at 1.2 m)'), metric('grasp targets', targets.length / 2, 'pairs', targets.length >= 2 * (objects.length - 2), `≥ ${objects.length - 2}`), metric('grasp tool Z · world Z', M(zDown, 2), undefined, zDown < -0.9, '< -0.9 (pointing down)')] };
    },
  }),
  visionScenario({
    id: 'vis_lidar_rows', title: 'Orchard — 3D LiDAR (Ouster OS1): ground removal, trunk clusters, crop-row lines', method: 'pcl_pipeline + patchwork',
    description: 'LiDAR at 1.2 m in an alley: the point cloud is voxelised, the ground plane removed, trunks clustered in a 0.2–0.9 m slice and row lines fitted. The detected row is compared with the true row line.',
    howTo: 'Add › Camera, set kind 3D LiDAR (or pick a LiDAR in the wizard) → tasks point-cloud detection + segmentation → Run → bird\'s-eye view in the Vision tab → Export cloud (.pcd).',
    build: () => { const b = orchardCameraStation({ kind: 'lidar3d', lateral: -1750, height_mm: 1200 }); b.cam.setPose(mul(transl(b.cam.poseAbs()[12], b.cam.poseAbs()[13], 1200), rotz(90 * DEG))); b.cam.far = 40000; return { station: b.station, cam: b.cam, extra: b.field }; },
    cfg: vcfg({ tasks: ['cloud_detect', 'cloud_segment', 'measure'], modality: 'lidar3d', sensor: 'ouster_os1', compute: 'jetson_orin_nx', models: { cloud_detect: 'pcl_pipeline', cloud_segment: 'patchwork', measure: 'pcl_pipeline' }, classes: ['trunk'] }),
    evaluate: async ({ station, rt, extra }) => {
      const field = extra as FieldItem;
      const out = await rt.step(station, { time: 0, lidar: { channels: 32, hres: 0.5, range: 20000 } });
      const c = out.cloud!;
      const rows = field.rows();
      const abs = field.poseAbs();
      // true row lines in world
      const trueLines = rows.map((r) => { const [sx, sy] = r.pointAt(0), [ex, ey] = r.pointAt(r.length()); return { a: [abs[0] * sx + abs[4] * sy + abs[12], abs[1] * sx + abs[5] * sy + abs[13]], b: [abs[0] * ex + abs[4] * ey + abs[12], abs[1] * ex + abs[5] * ey + abs[13]] }; });
      const lineErr = (l: { point: [number, number]; dir: [number, number] }) => Math.min(...trueLines.map((t) => { const vx = t.b[0] - t.a[0], vy = t.b[1] - t.a[1], L = Math.hypot(vx, vy) || 1; return Math.abs(-(l.point[0] - t.a[0]) * vy / L + (l.point[1] - t.a[1]) * vx / L); }));
      const errs = c.rows.map(lineErr);
      const best = errs.length ? Math.min(...errs) : Infinity;
      const trunks = c.clusters.filter((k) => k.shape === 'trunk').length;
      return { metrics: [metric('points (after voxel)', pointCount(c.cloud)), metric('ground points', pointCount(c.ground), undefined, pointCount(c.ground) > 200, '> 200'), metric('clusters', c.clusters.length), metric('trunk clusters', trunks, undefined, trunks >= 4, '≥ 4'), metric('row lines', c.rows.length, undefined, c.rows.length >= 1, '≥ 1'), metric('best row lateral error', M(best), 'mm', best < 400, '< 400 mm'), metric('ground normal z', M(c.ground.xyz.length ? 1 : 0, 2))] };
    },
  }),
  visionScenario({
    id: 'vis_tof_canopy_measure', title: 'Orchard — ToF depth camera: canopy volume and height from the depth cloud', method: 'depth cloud + pcl_pipeline (measure)',
    description: 'Depth image → point cloud → ground removal → canopy clusters with height / width / volume, the inputs for pruning and spraying dose maps.',
    howTo: 'Camera with kind Depth, wizard modality ToF, tasks measurement + point-cloud segmentation → Run.',
    build: () => orchardCameraStation({ lateral: -2500, height_mm: 1600, fov: 75, width: 640, height: 480 }),
    cfg: vcfg({ tasks: ['cloud_segment', 'measure'], modality: 'tof', sensor: 'kinect_azure_tof', models: { cloud_segment: 'patchwork', measure: 'pcl_pipeline' }, classes: ['tree'], workingDistance: 2.5 }),
    evaluate: async ({ station, rt }) => {
      const out = await rt.step(station, { time: 0 });
      const c = out.cloud!;
      const canopies = c.clusters.filter((k) => k.shape === 'canopy' || k.shape === 'wall' || k.shape === 'blob');
      const tallest = canopies.reduce((a, b) => (b.size[2] > (a?.size[2] ?? 0) ? b : a), canopies[0]);
      const h = tallest ? tallest.size[2] : 0;
      return { metrics: [metric('cloud points', pointCount(out.frame.cloud!), undefined, pointCount(out.frame.cloud!) > 500, '> 500'), metric('canopy clusters', canopies.length, undefined, canopies.length >= 1, '≥ 1'), metric('tallest canopy height', M(h / 1000, 2), 'm', h > 1500 && h < 4500, '1.5–4.5 m (apple spindle ≈ 3.2 m)'), metric('canopy volume', tallest ? M((tallest.size[0] * tallest.size[1] * tallest.size[2]) / 1e9, 2) : 0, 'm³')] };
    },
  }),
  visionScenario({
    id: 'vis_follow_vehicle', title: 'Following — stereo camera on an AMR tracks and follows a leading vehicle', method: 'yolov8n + bytetrack + follow controller',
    description: 'A leader drives a 40 m path at 0.8 m/s; the follower detects it (class "vehicle"), tracks it and keeps 3 m distance with the follow controller (bearing → yaw rate, range → speed).',
    howTo: 'Two mobile robots; camera on the follower → wizard tasks detect, track, follow, classes "vehicle" → Vision tab: choose the follower under "Follow with", tick "follow tracked target", start the leader (mission or path).',
    build: () => {
      const st = new Station('Follow scenario');
      const leader = st.addChild(new MobileRobot('Leader')); leader.kin.maxSpeed = 800; leader.setPose2D(4000, 0, 0);
      const follower = st.addChild(new MobileRobot('Follower')); follower.kin.maxSpeed = 1500; follower.kin.maxYawRate = 90; follower.setPose2D(0, 300, 0);
      const cam = follower.addChild(new CameraItem('Follow camera')); cam.kind = 'depth'; cam.fov = 90; cam.width = 640; cam.height = 400; cam.far = 15000;
      cam.setPose(multiply(transl(follower.kin.footprint[0] / 2, 0, 900), Float64Array.from([0, -1, 0, 0, 0, 0, -1, 0, 1, 0, 0, 0, 0, 0, 0, 1])));
      return { station: st, cam, extra: { leader, follower } };
    },
    cfg: vcfg({ tasks: ['detect', 'track', 'follow', 'pose'], sensor: 'oak_d_pro', compute: 'jetson_orin_nano', models: { detect: 'yolov8n', track: 'bytetrack', follow: 'bytetrack', pose: 'depth_centroid' }, classes: ['vehicle'], environment: 'urban', workingDistance: 3 }),
    evaluate: async ({ station, cam, rt, extra }) => {
      const { leader, follower } = extra as { leader: MobileRobot; follower: MobileRobot };
      followPath(leader, [[4000, 0], [20000, 0], [28000, 6000], [40000, 6000]]);
      const K = intrinsicsFromCamera(cam);
      const dt = 0.1;
      let trackedSteps = 0, steps = 0, rangeErrSum = 0, rangeN = 0;
      const desired = 3000;
      for (let i = 0; i < 600 && leader.state.path; i++) {
        stepMobile(leader, dt);
        const out = await rt.step(station, { time: i * dt });
        const tgt = out.detections.filter((b) => b.id !== undefined)[0] ?? out.detections[0] ?? null;
        const cmd = followCommand(tgt, K, { desiredRange: desired, maxSpeed: follower.kin.maxSpeed, maxYawRate: follower.kin.maxYawRate });
        integrate(follower, { v: cmd.v, omega: cmd.omega }, dt);
        steps++;
        if (tgt) trackedSteps++;
        if (i > 100) { const d = Math.hypot(leader.state.x - follower.state.x, leader.state.y - follower.state.y); rangeErrSum += Math.abs(d - desired); rangeN++; }
      }
      const finalDist = Math.hypot(leader.state.x - follower.state.x, leader.state.y - follower.state.y);
      const meanRangeErr = rangeN ? rangeErrSum / rangeN : NaN;
      const trackedPct = 100 * trackedSteps / Math.max(1, steps);
      return { metrics: [metric('leader path', M(leader.state.odometer / 1000, 1), 'm'), metric('target tracked', M(trackedPct), '% of steps', trackedPct > 80, '> 80 %'), metric('mean range error (after 10 s)', M(meanRangeErr), 'mm', meanRangeErr < 1200, '< 1200 mm'), metric('final distance', M(finalDist), 'mm', finalDist > 1500 && finalDist < 6000, '1.5–6 m (no collision, not lost)'), metric('follower distance', M(follower.state.odometer / 1000, 1), 'm')] };
    },
  }),
  visionScenario({
    id: 'vis_vlm_count', title: 'VLM — "how many ripe apples do you see?" on the camera image', method: 'qwen2_vl (simulated adapter; same contract as Ollama / vLLM)',
    description: 'Vision-language query: the answer text and the grounded boxes are parsed like a real OpenAI-compatible reply. The count is compared to the ground truth.',
    howTo: 'Wizard: task VLM query → Runtime & endpoints: VLM URL (Ollama http://localhost:11434/v1, model qwen2.5vl:7b) → Vision tab: type the question → Ask / step.',
    build: () => orchardCameraStation(), cfg: vcfg({ tasks: ['vlm_query'], models: { vlm_query: 'qwen2_vl' }, compute: 'jetson_agx_orin' }),
    evaluate: async ({ station, rt }) => {
      const out = await rt.step(station, { time: 0, prompt: 'How many apples do you see? Answer with a number.' });
      const truthCount = out.frame.truth!.filter((t) => t.visible && t.cls === 'apple').length;
      const said = Number((out.text ?? '').match(/(\d+)\s+apple/)?.[1] ?? NaN);
      const ok = Number.isFinite(said) && Math.abs(said - truthCount) <= Math.max(2, truthCount * 0.2);
      const ground = await rt.step(station, { time: 0.1, prompt: 'Find the apples' });
      return { metrics: [metric('apples visible (truth)', truthCount), metric('apples in the answer', Number.isFinite(said) ? said : 'n/a', undefined, ok, '±20 %'), metric('grounded boxes', ground.detections.length, undefined, ground.detections.length > 0, '> 0'), metric('answer', (out.text ?? '').slice(0, 80))] };
    },
  }),
  visionScenario({
    id: 'vis_vla_approach', title: 'VLA — "pick the apple": policy actions move the eye-in-hand camera to the fruit', method: 'pi0 (simulated adapter; openpi /infer contract)',
    description: 'Each step the image + instruction go to the policy; the returned 7-DoF action is applied as a TCP delta (20 mm / 5° per step) until the policy signals the grasp.',
    howTo: 'Camera on a tool → wizard task VLA policy → Runtime: VLA URL / format openpi → Vision tab: instruction → Ask / step repeatedly (or live).',
    build: () => { const b = orchardCameraStation({ lateral: -900, height_mm: 1700, fov: 80 }); return b; },
    cfg: vcfg({ tasks: ['vla_policy', 'detect'], models: { vla_policy: 'pi0', detect: 'yolov8s' }, compute: 'jetson_thor', vla: { url: '', model: 'pi0', format: 'openpi', instruction: 'pick the apple', actionScaleMm: 25, actionScaleDeg: 5 } }),
    evaluate: async ({ station, cam, rt }) => {
      let steps = 0, grasped = false, start = Math.hypot(0, 0);
      let nearest = Infinity;
      for (let i = 0; i < 80; i++) {
        const out = await rt.step(station, { time: i / 5, instruction: 'pick the apple', tcp: cam.poseAbs() });
        steps++;
        if (!out.vla) break;
        if (out.vla.tcp) cam.setPose(out.vla.tcp);
        const t = out.frame.truth!.filter((x) => x.visible && x.cls === 'apple');
        if (t.length) nearest = Math.min(...t.map((x) => x.z));
        if (i === 0) start = nearest;
        if (out.vla.text === 'grasp' || out.vla.gripper === 1) { grasped = true; break; }
      }
      return { metrics: [metric('start distance to nearest apple', M(start), 'mm'), metric('steps', steps), metric('grasp signalled', grasped ? 'yes' : 'no', undefined, grasped, 'yes within 80 steps'), metric('final distance to apple', M(nearest), 'mm', nearest < 200, '< 200 mm')] };
    },
  }),
  visionScenario({
    id: 'vis_open_vocab', title: 'Open vocabulary — YOLO-World with typed classes ("trunk", "vehicle") and no training', method: 'yolo_world',
    description: 'Text-prompted classes: trunks and a parked platform are detected without a dataset; recall is lower than a fine-tuned detector.',
    howTo: 'Wizard: tick "Open vocabulary", classes "trunk, vehicle" → YOLO-World is recommended → Run.',
    build: () => { const b = orchardCameraStation({ lateral: -2500, height_mm: 1300, fov: 90 }); const v = b.station.addChild(new MobileRobot('Parked platform')); v.setPose2D(b.cam.poseAbs()[12] + 4000, b.cam.poseAbs()[13] + 500, 0); return b; },
    cfg: vcfg({ tasks: ['detect', 'pose'], models: { detect: 'yolo_world', pose: 'depth_centroid' }, classes: ['trunk', 'vehicle'], compute: 'jetson_orin_nx' }),
    evaluate: async ({ station, rt }) => {
      const out = await rt.step(station, { time: 0 });
      const trunks = out.detections.filter((b) => b.cls === 'trunk').length, vehicles = out.detections.filter((b) => b.cls === 'vehicle').length;
      const s = detectionStats(rt);
      return { metrics: [metric('trunks', trunks, undefined, trunks >= 2, '≥ 2'), metric('vehicles', vehicles, undefined, vehicles >= 1, '≥ 1'), metric('precision', M(100 * s.precision), '%', s.precision >= 0.4, '≥ 40 % (open-vocabulary models produce more false positives)'), metric('recall', M(100 * s.recall), '%')] };
    },
  }),
];

import { CONTROL_SCENARIOS } from './control';
import { GROUP_SCENARIOS } from './group';
export { CONTROL_SCENARIOS, GROUP_SCENARIOS };
export const ALL_SCENARIOS: Scenario[] = [...NAV_SCENARIOS, ...VISION_SCENARIOS, ...CONTROL_SCENARIOS, ...GROUP_SCENARIOS];

export async function runScenarios(filter?: (s: Scenario) => boolean, onResult?: (r: ScenarioResult) => void): Promise<ScenarioResult[]> {
  const out: ScenarioResult[] = [];
  for (const s of ALL_SCENARIOS) {
    if (filter && !filter(s)) continue;
    let r: ScenarioResult;
    try { r = await s.run(); }
    catch (e) { r = { id: s.id, title: s.title, group: s.group, method: s.method, pass: false, metrics: [], notes: [`error: ${(e as Error).message}`], durationMs: 0 }; }
    out.push(r); onResult?.(r);
  }
  return out;
}

/** Markdown report (published in the documentation). */
export function scenarioReportMarkdown(results: ScenarioResult[], opts: { generatedAt?: string } = {}): string {
  const lines: string[] = [];
  lines.push(`<!-- generated by \`npm run scenarios\` in studio/ — do not edit by hand -->`, '');
  lines.push(`Generated ${opts.generatedAt ?? new Date().toISOString().slice(0, 10)} · ${results.filter((r) => r.pass).length}/${results.length} scenarios pass`, '');
  for (const group of ['navigation', 'vision', 'control', 'group'] as const) {
    lines.push(`### ${group === 'navigation' ? 'Navigation & localization' : group === 'vision' ? 'Machine vision' : group === 'control' ? 'Control design (course methods)' : 'Group control (multi-robot systems)'}`, '');
    lines.push('| Scenario | Method | Result | Key metrics |', '|---|---|---|---|');
    for (const r of results.filter((x) => x.group === group)) {
      const key = r.metrics.filter((m) => m.ok !== undefined).map((m) => `${m.name} ${m.value}${m.unit ? ' ' + m.unit : ''} ${m.ok ? '✓' : '✗'}`).join('<br>');
      lines.push(`| ${r.title} | \`${r.method}\` | ${r.pass ? '**pass**' : '**FAIL**'} | ${key} |`);
    }
    lines.push('');
    for (const r of results.filter((x) => x.group === group)) {
      const sc = ALL_SCENARIOS.find((s) => s.id === r.id);
      lines.push(`<details><summary><code>${r.id}</code> — ${r.title}</summary>`, '');
      if (sc) lines.push(sc.description, '', `*How to reproduce:* ${sc.howTo}`, '');
      lines.push('| Metric | Value | Criterion |', '|---|---|---|');
      for (const m of r.metrics) lines.push(`| ${m.name} | ${m.value}${m.unit ? ' ' + m.unit : ''} | ${m.bound ?? ''} ${m.ok === undefined ? '' : m.ok ? '✓' : '✗'} |`);
      if (r.notes.length) lines.push('', r.notes.map((n) => `- ${n}`).join('\n'));
      lines.push('', '</details>', '');
    }
  }
  return lines.join('\n');
}

export const SCENARIO_DOCS = ALL_SCENARIOS.map((s) => ({ id: s.id, title: s.title, group: s.group, method: s.method, description: s.description, howTo: s.howTo }));
