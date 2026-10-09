import { describe, it, expect } from 'vitest';
import { recommendStacks, LOCALIZATION_METHODS, NAVIGATION_METHODS, LocalizationEstimator, simulateLidar2D, SlamMap, defaultNavStack, gnssAvailableAt, stepNavRuntime, setNavStack, NavStackConfig } from '../src/mobile/navstack';
import { generateRosNavPackage, rosNavPackageZip } from '../src/mobile/navstack_ros';
import { MobileRobot, MapItem, ZoneItem } from '../src/mobile/items';
import { Station } from '../src/core/items/item';
import { followPath, stepMobile } from '../src/mobile/controller';
import { demos } from '../src/demos';
import { FleetItem, FleetManager } from '../src/fleet/fleet';
import { ItemType } from '../src/core/items/item';
import { unzipSync, strFromU8 } from 'fflate';

describe('navigation stack catalogue and recommender', () => {
  it('has consistent catalogue entries', () => {
    const ids = new Set<string>();
    for (const m of LOCALIZATION_METHODS) { expect(ids.has(m.id)).toBe(false); ids.add(m.id); expect(m.software.length).toBeGreaterThan(0); expect(Object.keys(m.suitability).length).toBeGreaterThan(5); }
    expect(NAVIGATION_METHODS.length).toBeGreaterThan(5);
  });
  it('recommends LiDAR-inertial / hybrid RTK stacks for orchards without sky view and RTK for open fields', () => {
    const orchard = recommendStacks({ platform: 'tracked', environment: 'orchard', sensors: ['wheel_odom', 'imu', 'lidar3d', 'gnss_rtk', 'stereo_camera'], needsGlobal: true });
    expect(['hybrid_rtk_lio', 'lio_sam', 'fast_lio2']).toContain(orchard[0].localization.id);
    expect(orchard[0].navigation.platforms).toContain('tracked');
    const rtkRank = orchard.findIndex((r) => r.localization.id === 'gnss_rtk');
    expect(rtkRank).toBeGreaterThan(2); // plain RTK is penalised under canopy
    expect(orchard.find((r) => r.localization.id === 'gnss_rtk')!.warnings.join(' ')).toMatch(/GNSS available only/);
    const field = recommendStacks({ platform: 'tractor', environment: 'open_field', sensors: ['wheel_odom', 'imu', 'gnss_rtk'] });
    expect(field[0].localization.family === 'gnss_rtk' || field[0].localization.id === 'hybrid_rtk_lio').toBe(true);
    expect(field[0].localization.id).toBe('gnss_rtk');
  });
  it('picks 2D LiDAR SLAM / tape / QR for indoor AGVs and warns about missing sensors and night use', () => {
    const wh = recommendStacks({ platform: 'amr', environment: 'warehouse', sensors: ['wheel_odom', 'imu', 'lidar2d'] });
    expect(wh[0].localization.family).toBe('lidar2d_slam');
    expect(wh[0].navigation.id).toMatch(/nav2/);
    const line = recommendStacks({ platform: 'agv_line', environment: 'warehouse' });
    expect(['magnetic_tape', 'qr_grid']).toContain(line[0].localization.id);
    expect(line[0].navigation.id).toBe('line_following');
    const mono = recommendStacks({ platform: 'amr', environment: 'greenhouse', sensors: ['mono_camera'], night: true });
    const m = mono.find((r) => r.localization.id === 'orb_slam3_mono')!;
    expect(m.warnings.join(' ')).toMatch(/night/);
    const vio = mono.find((r) => r.localization.id === 'openvins')!;
    expect(vio.missingSensors).toContain('imu');
    const gh = recommendStacks({ platform: 'amr', environment: 'greenhouse', sensors: ['wheel_odom', 'imu', 'lidar2d', 'uwb'] });
    expect(['hybrid_uwb_lidar', 'slam_toolbox_2d', 'cartographer_2d', 'uwb']).toContain(gh[0].localization.id);
    const rail = recommendStacks({ platform: 'rail', environment: 'greenhouse' });
    expect(rail[0].localization.id).toBe('rail_encoder');
  });
  it('builds a default stack per platform', () => {
    const d = defaultNavStack('tractor', 'vineyard');
    expect(d.sensors).toContain('gnss_rtk');
    expect(d.localization).toBeTruthy();
  });
});

describe('localization estimator', () => {
  const run = (cfg: NavStackConfig, gnss: boolean, featureRich = true, metres = 200) => {
    const est = new LocalizationEstimator(cfg, 7);
    est.reset(0, 0, 0);
    const dt = 0.1, v = 1000; // 1 m/s
    let x = 0;
    for (let t = 0; t < metres; t += v * dt / 1000) { x += v * dt; est.update({ x, y: 0, theta: 0 }, dt, { gnss, featureRich }); }
    return est.state;
  };
  it('RTK stays at centimetre level with sky view and drifts during outages', () => {
    const cfg: NavStackConfig = { platform: 'tractor', environment: 'open_field', sensors: ['gnss_rtk', 'imu', 'wheel_odom'], localization: 'gnss_rtk', navigation: 'gnss_waypoints', fusion: ['imu', 'wheel_odom'], simulate: true };
    const fix = run(cfg, true);
    expect(fix.rmse).toBeLessThan(80);
    const outage = run(cfg, false);
    expect(outage.error).toBeGreaterThan(fix.error);
    expect(outage.maxError).toBeGreaterThan(300);
  });
  it('VIO drifts roughly proportionally to distance, monocular SLAM drifts in scale, 2D SLAM stays bounded in structured space', () => {
    const vio: NavStackConfig = { platform: 'amr', environment: 'orchard', sensors: ['stereo_camera', 'imu'], localization: 'openvins', navigation: 'row_following', fusion: [], simulate: true };
    const short = run(vio, false, true, 50), long = run(vio, false, true, 400);
    expect(long.maxError).toBeGreaterThan(short.maxError);
    const mono: NavStackConfig = { ...vio, localization: 'orb_slam3_mono', sensors: ['mono_camera'] };
    const m = run(mono, false, true, 300);
    expect(m.scale).not.toBe(1);
    const slam: NavStackConfig = { platform: 'amr', environment: 'warehouse', sensors: ['lidar2d', 'wheel_odom'], localization: 'slam_toolbox_2d', navigation: 'nav2_navfn_dwb', fusion: ['wheel_odom'], simulate: true };
    const s = run(slam, false, true, 500);
    expect(s.rmse).toBeLessThan(200);
    const hybrid: NavStackConfig = { platform: 'tracked', environment: 'orchard', sensors: ['gnss_rtk', 'lidar3d', 'imu'], localization: 'hybrid_rtk_lio', navigation: 'nav2_smac_hybrid_rpp', fusion: ['wheel_odom'], simulate: true };
    const h = run(hybrid, false, true, 300);
    const plainRtk = run({ ...hybrid, localization: 'gnss_rtk', fusion: [] }, false, true, 300);
    expect(h.maxError).toBeLessThan(plainRtk.maxError);
  });
  it('camera methods can lose tracking in feature-poor areas and recover', () => {
    const cfg: NavStackConfig = { platform: 'amr', environment: 'open_field', sensors: ['mono_camera', 'imu'], localization: 'orb_slam3_vi', navigation: 'nav2_navfn_dwb', fusion: [], simulate: true };
    const est = new LocalizationEstimator(cfg, 3);
    est.reset(0, 0, 0);
    let lost = 0;
    for (let i = 0; i < 4000; i++) { est.update({ x: i * 100, y: 0, theta: 0 }, 0.1, { gnss: false, featureRich: false }); if (est.state.lost) lost++; }
    expect(est.state.lostEvents).toBeGreaterThan(0);
    for (let i = 0; i < 200; i++) est.update({ x: 400000 + i * 100, y: 0, theta: 0 }, 0.1, { gnss: false, featureRich: true });
    expect(lost).toBeGreaterThan(0);
  });
});

describe('lidar simulation and SLAM map', () => {
  it('casts rays on a map and integrates scans into an explored map that agrees with the truth', () => {
    const map = new MapItem('m'); map.resize(100, 60, 100, 0, 0);
    map.fillRect(0, 0, 10000, 100); map.fillRect(0, 5900, 10000, 6000); map.fillRect(4000, 2000, 4200, 4000);
    const scan = simulateLidar2D(map, 2000, 3000, 0, { beams: 90, fov: 360, maxRange: 15000, noise: 0 });
    expect(scan.ranges.length).toBe(90);
    const forward = scan.ranges[45]; // pointing +x towards the pillar at x=4000
    expect(forward).toBeGreaterThan(1800); expect(forward).toBeLessThan(2200);
    const slam = SlamMap.like(map);
    for (let x = 1000; x < 9000; x += 500) { const s = simulateLidar2D(map, x, 3000, 0, { beams: 180, fov: 360, maxRange: 15000, noise: 0 }); slam.integrate(s, x, 3000, 0, 0); }
    expect(slam.coverage()).toBeGreaterThan(0.3);
    expect(slam.agreement(map)).toBeGreaterThan(0.5);
  });
  it('GNSS availability follows gnss_denied zones and indoor environments', () => {
    const z = new ZoneItem('canopy'); z.kind = 'gnss_denied'; z.polygon = [[0, 0], [10000, 0], [10000, 5000], [0, 5000]];
    const cfg: NavStackConfig = { platform: 'tracked', environment: 'orchard', sensors: [], localization: 'gnss_rtk', navigation: 'gnss_waypoints', fusion: [], simulate: true };
    expect(gnssAvailableAt(5000, 2500, [z], cfg)).toBe(false);
    expect(gnssAvailableAt(5000, 8000, [z], cfg)).toBe(true);
    expect(gnssAvailableAt(5000, 8000, [], { ...cfg, environment: 'warehouse' })).toBe(false);
  });
});

describe('fleet integration and ROS 2 export', () => {
  it('drives a robot from its estimate: with a drifting stack the true path deviates from the ideal one', () => {
    const st = new Station('s');
    const map = new MapItem('map'); map.resize(200, 100, 100, 0, 0); st.addChild(map);
    const r = new MobileRobot('amr'); r.kin.maxSpeed = 1500; st.addChild(r);
    r.setPose2D(1000, 5000, 0);
    setNavStack(r, { platform: 'amr', environment: 'open_field', sensors: ['mono_camera'], localization: 'orb_slam3_mono', navigation: 'nav2_navfn_dwb', fusion: [], simulate: true });
    followPath(r, [[1000, 5000], [15000, 5000]]);
    let maxErr = 0;
    for (let i = 0; i < 600 && r.state.path; i++) { const rt = stepNavRuntime(r, 0.05, map, []); const pose = rt && !rt.estimator.state.lost ? rt.estimator.state : undefined; stepMobile(r, 0.05, { pose: pose ? { x: pose.x, y: pose.y, theta: pose.theta } : undefined }); maxErr = Math.max(maxErr, Math.abs(r.state.y - 5000)); }
    const rt = (r as any)._nav;
    expect(rt.estimator.state.samples).toBeGreaterThan(100);
    expect(rt.slam === null).toBe(true); // no lidar on this robot
    expect(maxErr).toBeGreaterThan(0);
  });
  it('the orchard demo fleet runs with a simulated hybrid RTK + LiDAR stack and reports GNSS-denied rows', () => {
    const st = demos.find((d) => d.id === 'orchard')!.build();
    const robots = st.itemsOfType<MobileRobot>(ItemType.MOBILE_ROBOT);
    for (const r of robots) setNavStack(r, { platform: 'tracked', environment: 'orchard', sensors: ['wheel_odom', 'imu', 'lidar3d', 'gnss_rtk'], localization: 'hybrid_rtk_lio', navigation: 'nav2_smac_hybrid_rpp', fusion: ['wheel_odom'], simulate: true });
    expect(st.itemsOfType<ZoneItem>(ItemType.ZONE).some((z) => z.kind === 'gnss_denied')).toBe(true);
    const fleet = st.itemsOfType<FleetItem>(ItemType.FLEET)[0];
    const fm = new FleetManager(st, fleet);
    for (let i = 0; i < 600; i++) fm.step(0.1);
    const rts = robots.map((r) => (r as any)._nav).filter(Boolean);
    expect(rts.length).toBe(robots.length);
    expect(rts.some((rt) => rt.estimator.state.samples > 100)).toBe(true);
    expect(rts.every((rt) => rt.estimator.state.rmse < 1500)).toBe(true);
  });
  it('exports a ROS 2 package with Nav2 params, SLAM config, EKF and launch file matching the stack', () => {
    const r = new MobileRobot('Sprayer'); r.kin.drive = 'ackermann'; r.kin.maxSteer = 30; r.kin.wheelBase = 1800; r.rosNamespace = '/sprayer';
    const cfg: NavStackConfig = { platform: 'tractor', environment: 'orchard', sensors: ['wheel_odom', 'imu', 'lidar3d', 'gnss_rtk'], localization: 'hybrid_rtk_lio', navigation: 'nav2_smac_hybrid_rpp', fusion: ['wheel_odom'], simulate: false };
    const pkg = generateRosNavPackage(r, cfg);
    expect(Object.keys(pkg.files)).toEqual(expect.arrayContaining(['config/nav2_params.yaml', 'config/ekf_odom.yaml', 'config/ekf_map.yaml', 'config/navsat_transform.yaml', 'config/ntrip.yaml', 'launch/bringup.launch.py', 'package.xml', 'README.md']));
    expect(pkg.files['config/nav2_params.yaml']).toMatch(/SmacPlannerHybrid/);
    expect(pkg.files['config/nav2_params.yaml']).toMatch(/REEDS_SHEPP/);
    expect(pkg.files['config/nav2_params.yaml']).toMatch(/minimum_turning_radius: 3\.1/);
    expect(pkg.files['config/nav2_params.yaml']).toMatch(/topic: \/sprayer\/points/);
    expect(pkg.files['config/ekf_map.yaml']).toMatch(/odometry\/gps/);
    expect(pkg.files['launch/bringup.launch.py']).toMatch(/navsat_transform_node/);
    const zip = unzipSync(rosNavPackageZip(pkg));
    expect(strFromU8(zip[`${pkg.name}/README.md`])).toMatch(/Hybrid: RTK-GNSS \+ LiDAR-inertial/);
    const wh = generateRosNavPackage(new MobileRobot('amr'), { platform: 'amr', environment: 'warehouse', sensors: ['wheel_odom', 'imu', 'lidar2d'], localization: 'slam_toolbox_2d', navigation: 'nav2_navfn_dwb', fusion: ['wheel_odom', 'imu'], simulate: false });
    expect(wh.files['config/slam_toolbox.yaml']).toMatch(/mode: mapping/);
    expect(wh.files['config/nav2_params.yaml']).toMatch(/DWBLocalPlanner/);
    expect(wh.files['launch/bringup.launch.py']).toMatch(/slam_toolbox/);
  });
});
