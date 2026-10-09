/**
 * ROS 2 message builders and publishers for the perception and navigation runtimes (rosbridge JSON).
 *
 * Vision (per camera, namespace `<ns>/vision/<camera>`):
 *   detections      vision_msgs/msg/Detection2DArray   boxes, class, score, track id (as `id`)
 *   detections_3d   vision_msgs/msg/Detection3DArray   3D positions (map frame, metres) + box size
 *   targets         geometry_msgs/msg/PoseArray        grasp poses (map frame)
 *   cloud           sensor_msgs/msg/PointCloud2        simulated / analysed cloud (xyz float32, base64 data)
 *   image/compressed sensor_msgs/msg/CompressedImage   the rendered camera image (jpeg)
 *   vlm/answer      std_msgs/msg/String
 * Navigation (per mobile robot, namespace `<ns>`):
 *   odom_estimate   nav_msgs/msg/Odometry              localization estimate with covariance
 *   ground_truth    nav_msgs/msg/Odometry              true pose (simulation only)
 *   scan            sensor_msgs/msg/LaserScan          simulated 2D LiDAR
 *   slam_map        nav_msgs/msg/OccupancyGrid         incremental SLAM map
 *
 * The same builders produce the compact JSON summaries stored in item params (`visionLast`, `navEstimate`) that
 * API clients (Robolink `getParam`) and webhooks receive.
 */
import type { Camera as CameraItem } from '../core/items/item';
import type { MobileRobot } from '../mobile/items';
import type { NavRuntime, LidarScan, SlamMap } from '../mobile/navstack';
import type { PipelineOutput } from '../vision/pipeline';
import type { PointCloud } from '../vision/pointcloud';
import { pointCount } from '../vision/pointcloud';
import { poseToQuat, Mat4, DEG } from '../core/math/pose';

export interface RosPublisher { publish(topic: string, type: string, msg: any): void }

export function stamp(timeS?: number): { sec: number; nanosec: number } {
  const t = timeS !== undefined ? timeS * 1000 : Date.now();
  return { sec: Math.floor(t / 1000), nanosec: Math.round((t % 1000) * 1e6) };
}
const header = (frame_id: string, timeS?: number) => ({ stamp: stamp(timeS), frame_id });
const M = (mm: number) => mm / 1000;

export function toBase64(u8: Uint8Array): string {
  if (typeof Buffer !== 'undefined') return Buffer.from(u8.buffer, u8.byteOffset, u8.byteLength).toString('base64');
  let s = ''; for (let i = 0; i < u8.length; i += 0x8000) s += String.fromCharCode(...u8.subarray(i, i + 0x8000));
  return btoa(s);
}

function poseMsg(m: Mat4) {
  const [w, x, y, z] = poseToQuat(m);
  return { position: { x: M(m[12]), y: M(m[13]), z: M(m[14]) }, orientation: { x, y, z, w } };
}
function yawQuat(thetaDeg: number) { const h = (thetaDeg * DEG) / 2; return { x: 0, y: 0, z: Math.sin(h), w: Math.cos(h) }; }

// ---------------------------------------------------------------------------------------------
// Vision
// ---------------------------------------------------------------------------------------------

export function detection2DArrayMsg(out: PipelineOutput, frameId: string) {
  return {
    header: header(frameId, out.frame.time),
    detections: out.detections.map((b) => ({
      header: header(frameId, out.frame.time),
      results: [{ hypothesis: { class_id: b.cls, score: b.score }, pose: { pose: b.p ? { position: { x: M(b.p[0]), y: M(b.p[1]), z: M(b.p[2]) }, orientation: { x: 0, y: 0, z: 0, w: 1 } } : { position: { x: 0, y: 0, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } }, covariance: new Array(36).fill(0) } }],
      bbox: { center: { position: { x: b.x + b.w / 2, y: b.y + b.h / 2 }, theta: 0 }, size_x: b.w, size_y: b.h },
      id: b.id !== undefined ? String(b.id) : '',
    })),
  };
}

export function detection3DArrayMsg(out: PipelineOutput, frameId = 'map') {
  const dets = out.detections.filter((b) => b.p);
  return {
    header: header(frameId, out.frame.time),
    detections: dets.map((b) => {
      const sz = b.attr && typeof b.attr.sizeX === 'number' ? { x: M(b.attr.sizeX as number), y: M(b.attr.sizeY as number), z: M(b.attr.sizeZ as number) } : { x: M((b.z ?? 1000) * b.w / out.frame.K.fx), y: M((b.z ?? 1000) * b.h / out.frame.K.fy), z: M((b.z ?? 1000) * b.w / out.frame.K.fx) };
      return { header: header(frameId, out.frame.time), results: [{ hypothesis: { class_id: b.cls, score: b.score }, pose: { pose: { position: { x: M(b.p![0]), y: M(b.p![1]), z: M(b.p![2]) }, orientation: { x: 0, y: 0, z: 0, w: 1 } }, covariance: new Array(36).fill(0) } }], bbox: { center: { position: { x: M(b.p![0]), y: M(b.p![1]), z: M(b.p![2]) }, orientation: { x: 0, y: 0, z: 0, w: 1 } }, size: sz }, id: b.id !== undefined ? String(b.id) : '' };
    }),
  };
}

export function poseArrayMsg(out: PipelineOutput, frameId = 'map') {
  return { header: header(frameId, out.frame.time), poses: out.targets.map((t) => poseMsg(t.grasp)) };
}

/** sensor_msgs/PointCloud2 with float32 x y z (+ intensity), little-endian, data base64 (rosbridge). */
export function pointCloud2Msg(cloud: PointCloud, frameId: string, timeS?: number) {
  const n = pointCount(cloud);
  const hasI = !!cloud.intensity;
  const step = hasI ? 16 : 12;
  const buf = new Uint8Array(n * step);
  const dv = new DataView(buf.buffer);
  for (let i = 0; i < n; i++) {
    dv.setFloat32(i * step, M(cloud.xyz[i * 3]), true); dv.setFloat32(i * step + 4, M(cloud.xyz[i * 3 + 1]), true); dv.setFloat32(i * step + 8, M(cloud.xyz[i * 3 + 2]), true);
    if (hasI) dv.setFloat32(i * step + 12, cloud.intensity![i], true);
  }
  const fields = [{ name: 'x', offset: 0, datatype: 7, count: 1 }, { name: 'y', offset: 4, datatype: 7, count: 1 }, { name: 'z', offset: 8, datatype: 7, count: 1 }];
  if (hasI) fields.push({ name: 'intensity', offset: 12, datatype: 7, count: 1 });
  return { header: header(frameId, timeS), height: 1, width: n, fields, is_bigendian: false, point_step: step, row_step: step * n, data: toBase64(buf), is_dense: true };
}

export function compressedImageMsg(dataUrl: string, frameId: string, timeS?: number) {
  const m = dataUrl.match(/^data:image\/(\w+);base64,(.*)$/);
  return { header: header(frameId, timeS), format: m ? m[1] : 'jpeg', data: m ? m[2] : '' };
}

/** Compact summary stored in `camera.params.visionLast` and sent to webhooks. */
export function visionSummary(cam: CameraItem, out: PipelineOutput) {
  return {
    camera: cam.name, time: out.frame.time, frame: 'map', units: 'mm',
    detections: out.detections.map((b) => ({ cls: b.cls, score: Number(b.score.toFixed(3)), id: b.id ?? null, box: [Math.round(b.x), Math.round(b.y), Math.round(b.w), Math.round(b.h)], z: b.z ? Math.round(b.z) : null, p: b.p ? b.p.map((v) => Math.round(v)) : null, attr: b.attr ?? null })),
    targets: out.targets.map((t) => ({ name: t.name, cls: t.cls, p: t.p.map((v) => Math.round(v)), grasp: Array.from(t.grasp).map((v) => Number(v.toFixed(3))), approach: Array.from(t.approach).map((v) => Number(v.toFixed(3))), sigmaMm: Math.round(t.sigmaMm) })),
    cloud: out.cloud ? { points: pointCount(out.cloud.cloud), clusters: out.cloud.clusters.map((c) => ({ id: c.id, shape: c.shape, centroid: c.centroid.map((v) => Math.round(v)), size: c.size.map((v) => Math.round(v)), points: c.count })), rows: out.cloud.rows.map((r) => ({ point: r.point.map((v) => Math.round(v)), dir: r.dir.map((v) => Number(v.toFixed(4))), length: Math.round(r.length) })) } : null,
    text: out.text ?? null, vla: out.vla ? { action: out.vla.action, gripper: out.vla.gripper ?? null, text: out.vla.text ?? null } : null, warnings: out.warnings,
  };
}

export function publishVisionOutput(ros: RosPublisher, cam: CameraItem, out: PipelineOutput, opts: { namespace?: string; frameId?: string; worldFrame?: string; cloud?: boolean; image?: boolean } = {}): string[] {
  const ns = `${(opts.namespace ?? '').replace(/\/$/, '')}/vision/${cam.name.toLowerCase().replace(/[^a-z0-9]+/g, '_')}`;
  const frame = opts.frameId ?? `${cam.name.toLowerCase().replace(/[^a-z0-9]+/g, '_')}_optical_frame`;
  const world = opts.worldFrame ?? 'map';
  const topics: string[] = [];
  ros.publish(`${ns}/detections`, 'vision_msgs/msg/Detection2DArray', detection2DArrayMsg(out, frame)); topics.push(`${ns}/detections`);
  if (out.detections.some((b) => b.p)) { ros.publish(`${ns}/detections_3d`, 'vision_msgs/msg/Detection3DArray', detection3DArrayMsg(out, world)); topics.push(`${ns}/detections_3d`); }
  if (out.targets.length) { ros.publish(`${ns}/targets`, 'geometry_msgs/msg/PoseArray', poseArrayMsg(out, world)); topics.push(`${ns}/targets`); }
  if (opts.cloud !== false && out.cloud) { ros.publish(`${ns}/cloud`, 'sensor_msgs/msg/PointCloud2', pointCloud2Msg(out.cloud.cloud, world, out.frame.time)); topics.push(`${ns}/cloud`); }
  if (opts.image !== false && out.frame.dataUrl) { ros.publish(`${ns}/image/compressed`, 'sensor_msgs/msg/CompressedImage', compressedImageMsg(out.frame.dataUrl, frame, out.frame.time)); topics.push(`${ns}/image/compressed`); }
  if (out.text) { ros.publish(`${ns}/vlm/answer`, 'std_msgs/msg/String', { data: out.text }); topics.push(`${ns}/vlm/answer`); }
  return topics;
}

// ---------------------------------------------------------------------------------------------
// Navigation
// ---------------------------------------------------------------------------------------------

export function odometryMsg(p: { x: number; y: number; theta: number }, vel: { v: number; omega: number }, opts: { frame?: string; child?: string; sigmaMm?: number; sigmaDeg?: number; timeS?: number } = {}) {
  const cov = new Array(36).fill(0);
  const s = (opts.sigmaMm ?? 0) / 1000, sd = ((opts.sigmaDeg ?? 0) * Math.PI) / 180;
  cov[0] = s * s; cov[7] = s * s; cov[35] = sd * sd;
  return { header: header(opts.frame ?? 'map', opts.timeS), child_frame_id: opts.child ?? 'base_link', pose: { pose: { position: { x: M(p.x), y: M(p.y), z: 0 }, orientation: yawQuat(p.theta) }, covariance: cov }, twist: { twist: { linear: { x: M(vel.v), y: 0, z: 0 }, angular: { x: 0, y: 0, z: vel.omega * DEG } }, covariance: new Array(36).fill(0) } };
}

export function laserScanMsg(scan: LidarScan, frameId = 'laser', timeS?: number) {
  const n = scan.angles.length;
  const inc = n > 1 ? scan.angles[1] - scan.angles[0] : 0;
  return { header: header(frameId, timeS), angle_min: scan.angles[0] ?? 0, angle_max: scan.angles[n - 1] ?? 0, angle_increment: inc, time_increment: 0, scan_time: 0.1, range_min: 0.05, range_max: M(scan.maxRange), ranges: scan.ranges.map((r) => M(r)), intensities: [] };
}

export function occupancyGridMsg(map: { width: number; height: number; resolution: number; originX: number; originY: number; cells: Uint8Array }, frameId = 'map', timeS?: number) {
  const data: number[] = new Array(map.width * map.height);
  for (let i = 0; i < data.length; i++) { const v = map.cells[i]; data[i] = v === 255 ? -1 : Math.min(100, v); }
  return { header: header(frameId, timeS), info: { map_load_time: stamp(timeS), resolution: M(map.resolution), width: map.width, height: map.height, origin: { position: { x: M(map.originX), y: M(map.originY), z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } } }, data };
}

/** Compact summary stored in `robot.params.navEstimate`. */
export function navSummary(robot: MobileRobot, rt: NavRuntime) {
  const e = rt.estimator.state;
  return { x: Math.round(e.x), y: Math.round(e.y), theta: Number(e.theta.toFixed(2)), error: Math.round(e.error), rmse: Math.round(e.rmse), maxError: Math.round(e.maxError), lost: e.lost, lostEvents: e.lostEvents, fixes: e.fixes, gnss: e.gnssAvailable, distanceSinceFix: Math.round(e.distanceSinceFix), scale: Number(e.scale.toFixed(4)), method: rt.estimator.config.localization, truth: { x: Math.round(robot.state.x), y: Math.round(robot.state.y), theta: Number(robot.state.theta.toFixed(2)) }, slamCoverage: rt.slam ? Number(rt.slam.coverage().toFixed(3)) : null };
}

export function publishNavRuntime(ros: RosPublisher, robot: MobileRobot, rt: NavRuntime, opts: { namespace?: string; map?: boolean; timeS?: number } = {}): string[] {
  const ns = (opts.namespace ?? robot.rosNamespace ?? '').replace(/\/$/, '');
  const e = rt.estimator.state;
  const topics: string[] = [];
  ros.publish(`${ns}/odom_estimate`, 'nav_msgs/msg/Odometry', odometryMsg({ x: e.x, y: e.y, theta: e.theta }, { v: robot.state.v, omega: robot.state.omega }, { sigmaMm: Math.max(10, e.error), sigmaDeg: 1, timeS: opts.timeS })); topics.push(`${ns}/odom_estimate`);
  ros.publish(`${ns}/ground_truth`, 'nav_msgs/msg/Odometry', odometryMsg({ x: robot.state.x, y: robot.state.y, theta: robot.state.theta }, { v: robot.state.v, omega: robot.state.omega }, { timeS: opts.timeS })); topics.push(`${ns}/ground_truth`);
  if (rt.lastScan) { ros.publish(`${ns}/scan`, 'sensor_msgs/msg/LaserScan', laserScanMsg(rt.lastScan, 'laser', opts.timeS)); topics.push(`${ns}/scan`); }
  if (opts.map !== false && rt.slam) { ros.publish(`${ns}/slam_map`, 'nav_msgs/msg/OccupancyGrid', occupancyGridMsg(rt.slam as SlamMap, 'map', opts.timeS)); topics.push(`${ns}/slam_map`); }
  return topics;
}
