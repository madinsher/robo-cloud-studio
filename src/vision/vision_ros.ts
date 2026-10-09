/**
 * ROS 2 perception package export for a configured vision stack: camera / LiDAR driver launch, detector
 * (yolo_ros / Isaac ROS / ultralytics node), tracker, depth → 3D positions, static hand-eye TF, point-cloud
 * pipeline node (Open3D), VLM and VLA bridge nodes with the endpoints configured in the studio, README.
 */
import { zipSync, strToU8 } from 'fflate';
import type { Camera as CameraItem } from '../core/items/item';
import { poseToXyzrpw, fromArray } from '../core/math/pose';
import { VisionStackConfig, VISION_SENSORS, COMPUTE_TARGETS, VISION_MODELS, VisionSensor, TASK_LABELS } from './stack';

export interface RosVisionPackage { name: string; files: Record<string, string>; packages: string[]; notes: string[] }

const DRIVER_LAUNCH: Record<string, (s: VisionSensor, ns: string) => { pkg: string; node: string; params: Record<string, unknown>; topics: { image: string; depth?: string; info: string; cloud?: string } }> = {
  usb_rgb: (_s, ns) => ({ pkg: 'usb_cam', node: 'usb_cam_node_exe', params: { video_device: '/dev/video0', image_width: 1280, image_height: 720, framerate: 30.0, pixel_format: 'mjpeg2rgb', camera_frame_id: `${ns}camera_optical_frame` }, topics: { image: 'image_raw', info: 'camera_info' } }),
  gs_rgb: (_s, ns) => ({ pkg: 'pylon_ros2_camera_wrapper', node: 'pylon_ros2_camera_node', params: { camera_frame: `${ns}camera_optical_frame`, image_encoding: 'rgb8', frame_rate: 30.0, gige: { mtu_size: 9000 } }, topics: { image: 'image_raw', info: 'camera_info' } }),
  zed_x: () => ({ pkg: 'zed_wrapper', node: 'zed_camera.launch.py', params: { camera_model: 'zedx', 'depth.depth_mode': 'NEURAL', 'depth.min_depth': 0.3, 'pos_tracking.pos_tracking_enabled': false }, topics: { image: 'zed/zed_node/rgb/image_rect_color', depth: 'zed/zed_node/depth/depth_registered', info: 'zed/zed_node/rgb/camera_info', cloud: 'zed/zed_node/point_cloud/cloud_registered' } }),
  oak_d_pro: () => ({ pkg: 'depthai_ros_driver', node: 'camera.launch.py', params: { 'camera.i_nn_type': 'none', 'stereo.i_align_depth': true, 'stereo.i_subpixel': true, 'rgb.i_fps': 30.0 }, topics: { image: 'oak/rgb/image_raw', depth: 'oak/stereo/image_raw', info: 'oak/rgb/camera_info', cloud: 'oak/points' } }),
  realsense_d455: () => ({ pkg: 'realsense2_camera', node: 'rs_launch.py', params: { align_depth: { enable: true }, pointcloud: { enable: true }, depth_module: { profile: '848x480x30' }, rgb_camera: { profile: '1280x720x30' } }, topics: { image: 'camera/camera/color/image_raw', depth: 'camera/camera/aligned_depth_to_color/image_raw', info: 'camera/camera/color/camera_info', cloud: 'camera/camera/depth/color/points' } }),
  orbbec_gemini: () => ({ pkg: 'orbbec_camera', node: 'gemini2.launch.py', params: { depth_registration: true, enable_point_cloud: true }, topics: { image: 'camera/color/image_raw', depth: 'camera/depth/image_raw', info: 'camera/color/camera_info', cloud: 'camera/depth/points' } }),
  kinect_azure_tof: () => ({ pkg: 'azure_kinect_ros_driver', node: 'driver.launch.py', params: { depth_mode: 'NFOV_UNBINNED', color_resolution: '720P', fps: 30, point_cloud: true }, topics: { image: 'rgb/image_raw', depth: 'depth_to_rgb/image_raw', info: 'rgb/camera_info', cloud: 'points2' } }),
  zivid: () => ({ pkg: 'zivid_camera', node: 'zivid_camera', params: { frame_id: 'zivid_optical_frame' }, topics: { image: 'color/image_color', depth: 'depth/image', info: 'color/camera_info', cloud: 'points/xyzrgba' } }),
  velodyne_vlp16: () => ({ pkg: 'velodyne', node: 'velodyne-all-nodes-VLP16-launch.py', params: { device_ip: '192.168.1.201', model: 'VLP16' }, topics: { image: '', info: '', cloud: 'velodyne_points' } }),
  ouster_os1: () => ({ pkg: 'ouster_ros', node: 'driver.launch.py', params: { sensor_hostname: 'os-122xxxxxx.local', lidar_mode: '1024x10', point_type: 'xyzir' }, topics: { image: 'ouster/reflec_image', info: '', cloud: 'ouster/points' } }),
  livox_mid360: () => ({ pkg: 'livox_ros_driver2', node: 'msg_MID360_launch.py', params: { xfer_format: 0, publish_freq: 10.0 }, topics: { image: '', info: '', cloud: 'livox/lidar' } }),
  hesai_xt32: () => ({ pkg: 'hesai_ros_driver', node: 'start.py', params: { lidar_type: 'PandarXT32' }, topics: { image: '', info: '', cloud: 'lidar_points' } }),
  flir_boson: () => ({ pkg: 'flir_boson_usb', node: 'flir_boson_usb_node', params: { dev: '/dev/video0', frame_rate: 60.0 }, topics: { image: 'flir_boson/image_raw', info: 'flir_boson/camera_info' } }),
  micasense: () => ({ pkg: 'custom_multispectral', node: 'micasense_http_node', params: { host: '192.168.10.254' }, topics: { image: 'multispectral/rededge/image_raw', info: 'multispectral/camera_info' } }),
  prophesee_evk: () => ({ pkg: 'metavision_driver', node: 'driver_node', params: { serial: '' }, topics: { image: 'event_camera/events', info: '' } }),
};

const yaml = (o: Record<string, unknown>, ind = 0): string => Object.entries(o).map(([k, v]) => v && typeof v === 'object' && !Array.isArray(v) ? `${' '.repeat(ind)}${k}:\n${yaml(v as Record<string, unknown>, ind + 2)}` : `${' '.repeat(ind)}${k}: ${Array.isArray(v) ? JSON.stringify(v) : typeof v === 'string' ? JSON.stringify(v) : v}`).join('\n');

export function generateRosVisionPackage(cam: CameraItem, cfg: VisionStackConfig, opts: { packageName?: string; namespace?: string } = {}): RosVisionPackage {
  const s = VISION_SENSORS.find((x) => x.id === cfg.sensor) ?? VISION_SENSORS[0];
  const c = COMPUTE_TARGETS.find((x) => x.id === cfg.compute) ?? COMPUTE_TARGETS[3];
  const name = (opts.packageName ?? `${cam.name}_perception`).toLowerCase().replace(/[^a-z0-9_]+/g, '_');
  const ns = opts.namespace ?? '';
  const files: Record<string, string> = {};
  const packages = new Set<string>(['rclpy', 'sensor_msgs', 'vision_msgs', 'geometry_msgs', 'tf2_ros', 'cv_bridge', 'image_transport', 'message_filters']);
  const notes: string[] = [];
  const drv = (DRIVER_LAUNCH[s.id] ?? DRIVER_LAUNCH.usb_rgb)(s, ns);
  packages.add(drv.pkg);
  const models = Object.fromEntries(Object.entries(cfg.models).map(([t, id]) => [t, VISION_MODELS.find((m) => m.id === id)!]).filter(([, m]) => m)) as Record<string, (typeof VISION_MODELS)[number]>;
  const det = models.detect ?? models.segment ?? models.keypoints;
  const tensorrt = c.runtimes.includes('tensorrt');
  const device = tensorrt || c.class === 'workstation' ? 'cuda:0' : 'cpu';
  const yoloTask = models.segment ? 'segment' : models.keypoints ? 'pose' : 'detect';
  const hasDepth = !!drv.topics.depth;
  const isLidar = s.modality === 'lidar3d';
  // hand-eye static transform
  const he = cfg.handEye && cfg.handEye.length === 16 ? poseToXyzrpw(fromArray(cfg.handEye)) : poseToXyzrpw(cam.pose());
  const parentFrame = cfg.handEyeMode === 'eye_to_hand' ? 'base_link' : 'tool0';
  const M = (v: number) => (v / 1000).toFixed(4), R = (d: number) => ((d * Math.PI) / 180).toFixed(5);

  files['config/detector.yaml'] = `# ${det?.name ?? 'detector'} — ${TASK_LABELS[yoloTask === 'pose' ? 'keypoints' : yoloTask]}\n` + yaml({ yolo_node: { ros__parameters: { model: cfg.modelUrl ? cfg.modelUrl.split('/').pop() : `${det?.id ?? 'yolov8n'}${tensorrt ? '.engine' : '.pt'}`, model_type: det?.family === 'detr' ? 'RTDETR' : det?.id === 'yolo_world' ? 'World' : 'YOLO', task: yoloTask, device, threshold: cfg.confidence, iou: cfg.iou, imgsz_width: cfg.inputSize ?? 640, imgsz_height: cfg.inputSize ?? 640, half: tensorrt, input_image_topic: drv.topics.image, image_reliability: 2, classes: cfg.classes, enable: true } } });
  files['config/tracker.yaml'] = yaml({ tracking_node: { ros__parameters: { tracker: models.track?.id === 'botsort' ? 'botsort.yaml' : 'bytetrack.yaml', track_high_thresh: 0.5, track_low_thresh: 0.1, match_thresh: 0.8, track_buffer: 30 } } });
  if (hasDepth) files['config/detect_3d.yaml'] = yaml({ detect_3d_node: { ros__parameters: { target_frame: parentFrame === 'tool0' ? 'base_link' : 'base_link', maximum_detection_threshold: cfg.workingDistance * 2, depth_image_units_divisor: 1000, depth_image_topic: drv.topics.depth, depth_info_topic: drv.topics.info } } });
  files['config/hand_eye.yaml'] = `# camera pose in ${parentFrame} (${cfg.handEyeMode ?? 'eye_in_hand'}), metres / radians\nparent_frame: ${parentFrame}\nchild_frame: ${ns}camera_link\nxyz: [${M(he[0])}, ${M(he[1])}, ${M(he[2])}]\nrpy: [${R(he[3])}, ${R(he[4])}, ${R(he[5])}]\n# optical frame: +Z forward, +X right, +Y down (REP 103)\n`;
  if (isLidar || cfg.tasks.some((t) => t === 'cloud_detect' || t === 'cloud_segment')) {
    packages.add('sensor_msgs_py');
    files['scripts/cloud_pipeline.py'] = CLOUD_PIPELINE_PY(drv.topics.cloud ?? 'points', isLidar);
    files['config/cloud_pipeline.yaml'] = yaml({ cloud_pipeline: { ros__parameters: { input_topic: drv.topics.cloud ?? 'points', voxel_m: isLidar ? 0.15 : 0.04, ground_thresh_m: 0.12, cluster_tol_m: isLidar ? 0.5 : 0.12, min_points: isLidar ? 6 : 15, max_range_m: cfg.workingDistance * 4 } } });
    notes.push('cloud_pipeline.py needs `pip install open3d numpy` (falls back to numpy-only voxel + clustering when Open3D is missing)');
  }
  if (cfg.tasks.includes('vlm_query') || cfg.runtime === 'vlm') {
    files['scripts/vlm_bridge.py'] = VLM_BRIDGE_PY(drv.topics.image || 'image_raw');
    files['config/vlm.yaml'] = yaml({ vlm_bridge: { ros__parameters: { url: cfg.vlm?.url ?? 'http://localhost:11434/v1', model: cfg.vlm?.model ?? 'qwen2.5vl:7b', prompt: cfg.vlm?.prompt ?? 'Describe the scene for a robot.', classes: cfg.classes, period_s: 2.0 } } });
    notes.push(`VLM bridge talks to ${cfg.vlm?.url ?? 'http://localhost:11434/v1'} (Ollama / vLLM OpenAI-compatible); set VLM_API_KEY for hosted APIs`);
  }
  if (cfg.tasks.includes('vla_policy')) {
    files['scripts/vla_bridge.py'] = VLA_BRIDGE_PY(drv.topics.image || 'image_raw', cfg.vla?.format ?? 'studio');
    files['config/vla.yaml'] = yaml({ vla_bridge: { ros__parameters: { url: cfg.vla?.url ?? 'http://localhost:8000', model: cfg.vla?.model ?? 'pi0', format: cfg.vla?.format ?? 'studio', instruction: cfg.vla?.instruction ?? '', rate_hz: 5.0, action_scale_m: (cfg.vla?.actionScaleMm ?? 20) / 1000, action_scale_rad: ((cfg.vla?.actionScaleDeg ?? 5) * Math.PI) / 180, servo_topic: '/servo_node/delta_twist_cmds', gripper_topic: '/gripper/command' } } });
    packages.add('moveit_servo');
    notes.push('vla_bridge.py publishes TwistStamped deltas for moveit_servo (or your own Cartesian controller) at rate_hz — keep the safety stop wired');
  }
  files['launch/perception.launch.py'] = LAUNCH_PY(name, drv, s, !!det, hasDepth, isLidar, cfg, parentFrame, ns, he);
  files['package.xml'] = `<?xml version="1.0"?>\n<package format="3">\n  <name>${name}</name>\n  <version>0.1.0</version>\n  <description>Perception stack generated by VerticalBot Studio for ${cam.name}: ${s.name} on ${c.name}; ${Object.values(models).map((m) => m.name).join(', ')}</description>\n  <maintainer email="robotics@example.com">VerticalBot</maintainer>\n  <license>Apache-2.0</license>\n${[...packages].map((p) => `  <exec_depend>${p}</exec_depend>`).join('\n')}\n  <export><build_type>ament_python</build_type></export>\n</package>\n`;
  files['setup.py'] = `from setuptools import setup\nimport glob\npackage_name = '${name}'\nsetup(name=package_name, version='0.1.0', packages=[], data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]), ('share/' + package_name, ['package.xml']), ('share/' + package_name + '/launch', glob.glob('launch/*.py')), ('share/' + package_name + '/config', glob.glob('config/*.yaml')), ('lib/' + package_name, glob.glob('scripts/*.py'))], install_requires=['setuptools'], zip_safe=True)\n`;
  files[`resource/${name}`] = '';
  files['README.md'] = README(name, cam, cfg, s, c, models, drv, notes);
  return { name, files, packages: [...packages], notes };
}

export function rosVisionPackageZip(pkg: RosVisionPackage): Uint8Array {
  const entries: Record<string, Uint8Array> = {};
  for (const [p, c] of Object.entries(pkg.files)) entries[`${pkg.name}/${p}`] = strToU8(c);
  return zipSync(entries, { level: 6 });
}

function LAUNCH_PY(name: string, drv: ReturnType<(typeof DRIVER_LAUNCH)[string]>, s: VisionSensor, hasDet: boolean, hasDepth: boolean, isLidar: boolean, cfg: VisionStackConfig, parentFrame: string, ns: string, he: number[]): string {
  const M = (v: number) => (v / 1000).toFixed(4), R = (d: number) => ((d * Math.PI) / 180).toFixed(5);
  const driverIsLaunch = drv.node.endsWith('.py');
  return `# Generated by VerticalBot Studio — perception bring-up for ${s.name}
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    share = get_package_share_directory('${name}')
    cfg = lambda f: os.path.join(share, 'config', f)
    nodes = []
    # 1. sensor driver (${drv.pkg})
${driverIsLaunch ? `    try:
        drv_share = get_package_share_directory('${drv.pkg}')
        nodes.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(drv_share, 'launch', '${drv.node}')), launch_arguments={${Object.entries(drv.params).filter(([, v]) => typeof v !== 'object').map(([k, v]) => `'${k}': '${v}'`).join(', ')}}.items()))
    except Exception as e:  # driver package not installed
        print('driver ${drv.pkg} not found:', e)` : `    nodes.append(Node(package='${drv.pkg}', executable='${drv.node}', name='camera', namespace='${ns.replace(/\/$/, '')}', parameters=[${JSON.stringify(drv.params).replace(/"/g, "'")}]))`}
    # 2. static hand-eye transform (${cfg.handEyeMode ?? 'eye_in_hand'}): ${parentFrame} -> camera_link
    nodes.append(Node(package='tf2_ros', executable='static_transform_publisher', name='hand_eye_tf', arguments=['--x', '${M(he[0])}', '--y', '${M(he[1])}', '--z', '${M(he[2])}', '--roll', '${R(he[3])}', '--pitch', '${R(he[4])}', '--yaw', '${R(he[5])}', '--frame-id', '${parentFrame}', '--child-frame-id', '${ns}camera_link']))
    nodes.append(Node(package='tf2_ros', executable='static_transform_publisher', name='optical_tf', arguments=['--roll', '-1.5708', '--pitch', '0', '--yaw', '-1.5708', '--frame-id', '${ns}camera_link', '--child-frame-id', '${ns}camera_optical_frame']))
${hasDet ? `    # 3. detector + tracker (yolo_ros: https://github.com/mgonzs13/yolo_ros)
    nodes.append(Node(package='yolo_ros', executable='yolo_node', name='yolo_node', parameters=[cfg('detector.yaml')], remappings=[('image_raw', '${drv.topics.image}')]))
    nodes.append(Node(package='yolo_ros', executable='tracking_node', name='tracking_node', parameters=[cfg('tracker.yaml')], remappings=[('image_raw', '${drv.topics.image}')]))
${hasDepth ? `    # 4. 2D detections + depth -> 3D poses (vision_msgs/Detection3DArray in base_link)
    nodes.append(Node(package='yolo_ros', executable='detect_3d_node', name='detect_3d_node', parameters=[cfg('detect_3d.yaml')], remappings=[('depth_image', '${drv.topics.depth}'), ('depth_info', '${drv.topics.info}')]))
` : ''}    nodes.append(Node(package='yolo_ros', executable='debug_node', name='debug_node', remappings=[('image_raw', '${drv.topics.image}')]))
` : ''}${isLidar || cfg.tasks.some((t) => t === 'cloud_detect' || t === 'cloud_segment') ? `    # point-cloud pipeline: voxel -> ground -> clusters -> Detection3DArray + rows
    nodes.append(Node(package='${name}', executable='cloud_pipeline.py', name='cloud_pipeline', parameters=[cfg('cloud_pipeline.yaml')]))
` : ''}${cfg.tasks.includes('vlm_query') || cfg.runtime === 'vlm' ? `    nodes.append(Node(package='${name}', executable='vlm_bridge.py', name='vlm_bridge', parameters=[cfg('vlm.yaml')]))
` : ''}${cfg.tasks.includes('vla_policy') ? `    nodes.append(Node(package='${name}', executable='vla_bridge.py', name='vla_bridge', parameters=[cfg('vla.yaml')]))
` : ''}    return LaunchDescription(nodes)
`;
}

const CLOUD_PIPELINE_PY = (topic: string, lidar: boolean) => `#!/usr/bin/env python3
"""Point-cloud pipeline (generated by VerticalBot Studio): voxel down-sample -> RANSAC ground -> Euclidean clusters
-> vision_msgs/Detection3DArray (+ crop-row lines as MarkerArray). Mirrors studio/src/vision/pointcloud.ts."""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose
from visualization_msgs.msg import MarkerArray, Marker
try:
    import open3d as o3d
except Exception:  # noqa: BLE001
    o3d = None


class CloudPipeline(Node):
    def __init__(self):
        super().__init__('cloud_pipeline')
        p = self.declare_parameters('', [('input_topic', '${topic}'), ('voxel_m', ${lidar ? 0.15 : 0.04}), ('ground_thresh_m', 0.12), ('cluster_tol_m', ${lidar ? 0.5 : 0.12}), ('min_points', ${lidar ? 6 : 15}), ('max_range_m', 20.0)])
        self.v = {x.name: x.value for x in p}
        self.sub = self.create_subscription(PointCloud2, self.v['input_topic'], self.cb, 1)
        self.pub = self.create_publisher(Detection3DArray, 'cloud/detections', 1)
        self.pub_ground = self.create_publisher(PointCloud2, 'cloud/ground', 1)
        self.pub_obj = self.create_publisher(PointCloud2, 'cloud/objects', 1)
        self.pub_rows = self.create_publisher(MarkerArray, 'cloud/rows', 1)

    def cb(self, msg):
        pts = np.array([[x, y, z] for x, y, z in point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)], dtype=np.float32)
        if len(pts) < 10:
            return
        pts = pts[np.linalg.norm(pts[:, :2], axis=1) < self.v['max_range_m']]
        if o3d is not None:
            pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts)).voxel_down_sample(self.v['voxel_m'])
            plane, inl = pc.segment_plane(self.v['ground_thresh_m'], 3, 200)
            ground = pc.select_by_index(inl)
            objects = pc.select_by_index(inl, invert=True)
            labels = np.array(objects.cluster_dbscan(eps=self.v['cluster_tol_m'], min_points=int(self.v['min_points'])))
            opts = np.asarray(objects.points)
            gpts = np.asarray(ground.points)
        else:
            vox = np.floor(pts / self.v['voxel_m']).astype(np.int64)
            _, idx = np.unique(vox, axis=0, return_index=True)
            pts = pts[idx]
            zmin = np.percentile(pts[:, 2], 5)
            gmask = np.abs(pts[:, 2] - zmin) < self.v['ground_thresh_m']
            gpts, opts = pts[gmask], pts[~gmask]
            labels = self.simple_cluster(opts)
        det = Detection3DArray(); det.header = msg.header
        centroids = []
        for lab in set(labels.tolist()):
            if lab < 0:
                continue
            c = opts[labels == lab]
            if len(c) < self.v['min_points']:
                continue
            mn, mx = c.min(0), c.max(0)
            d = Detection3D(); d.header = msg.header
            d.bbox.center.position.x, d.bbox.center.position.y, d.bbox.center.position.z = [float(v) for v in (mn + mx) / 2]
            d.bbox.size.x, d.bbox.size.y, d.bbox.size.z = [float(v) for v in (mx - mn)]
            h = ObjectHypothesisWithPose(); h.hypothesis.class_id = self.shape(mx - mn, mn[2]); h.hypothesis.score = min(1.0, len(c) / 50.0)
            d.results.append(h); det.detections.append(d)
            if h.hypothesis.class_id in ('trunk', 'canopy', 'blob'):
                centroids.append(((mn[0] + mx[0]) / 2, (mn[1] + mx[1]) / 2))
        self.pub.publish(det)
        self.pub_ground.publish(point_cloud2.create_cloud_xyz32(msg.header, gpts.tolist()))
        self.pub_obj.publish(point_cloud2.create_cloud_xyz32(msg.header, opts.tolist()))
        self.pub_rows.publish(self.rows(centroids, msg.header))

    @staticmethod
    def shape(size, zmin):
        foot, h = max(size[0], size[1]), size[2]
        if foot > 4.0 and h < 2.5: return 'wall'
        if h > 1.2 and foot < 0.5: return 'trunk'
        if 1.3 < h < 2.2 and 0.3 < foot < 1.0: return 'person'
        if h > 0.8 and foot > 0.8: return 'canopy'
        if h < 0.6 and zmin < 0.4: return 'low'
        return 'blob'

    def simple_cluster(self, pts):
        tol = self.v['cluster_tol_m']; labels = -np.ones(len(pts), dtype=int); cur = 0
        grid = {}
        for i, p in enumerate(pts):
            grid.setdefault(tuple(np.floor(p / tol).astype(int)), []).append(i)
        for i in range(len(pts)):
            if labels[i] >= 0: continue
            stack = [i]; labels[i] = cur
            while stack:
                j = stack.pop(); g = tuple(np.floor(pts[j] / tol).astype(int))
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        for dz in (-1, 0, 1):
                            for k in grid.get((g[0] + dx, g[1] + dy, g[2] + dz), []):
                                if labels[k] < 0 and np.linalg.norm(pts[k] - pts[j]) <= tol:
                                    labels[k] = cur; stack.append(k)
            cur += 1
        return labels

    def rows(self, cents, header):
        ma = MarkerArray()
        if len(cents) < 3: return ma
        c = np.array(cents); m = c.mean(0); u, s, vt = np.linalg.svd(c - m)
        d = vt[0]; proj = (c - m) @ d
        mk = Marker(); mk.header = header; mk.type = Marker.LINE_STRIP; mk.scale.x = 0.05; mk.color.g = 1.0; mk.color.a = 1.0; mk.id = 0
        from geometry_msgs.msg import Point
        for t in (proj.min(), proj.max()):
            p = Point(); p.x, p.y = float(m[0] + d[0] * t), float(m[1] + d[1] * t); p.z = 0.0; mk.points.append(p)
        ma.markers.append(mk)
        return ma


def main():
    rclpy.init(); n = CloudPipeline(); rclpy.spin(n)

if __name__ == '__main__':
    main()
`;

const VLM_BRIDGE_PY = (image: string) => `#!/usr/bin/env python3
"""VLM bridge (generated by VerticalBot Studio): periodically sends the camera image with a prompt to an
OpenAI-compatible endpoint (Ollama / vLLM / hosted) and publishes the answer (std_msgs/String) and any grounded
boxes (vision_msgs/Detection2DArray, 0-1000 normalised frame -> pixels). Ask questions on ~/question."""
import base64, json, os, re, urllib.request
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2


class VlmBridge(Node):
    def __init__(self):
        super().__init__('vlm_bridge')
        p = self.declare_parameters('', [('url', 'http://localhost:11434/v1'), ('model', 'qwen2.5vl:7b'), ('prompt', 'Describe the scene.'), ('classes', ['object']), ('period_s', 2.0)])
        self.v = {x.name: x.value for x in p}
        self.bridge = CvBridge(); self.img = None; self.question = None
        self.create_subscription(Image, '${image}', lambda m: setattr(self, 'img', m), 1)
        self.create_subscription(String, '~/question', lambda m: setattr(self, 'question', m.data), 1)
        self.pub_text = self.create_publisher(String, '~/answer', 1)
        self.pub_det = self.create_publisher(Detection2DArray, '~/detections', 1)
        self.create_timer(float(self.v['period_s']), self.tick)

    def tick(self):
        if self.img is None: return
        cv = self.bridge.imgmsg_to_cv2(self.img, 'bgr8'); h, w = cv.shape[:2]
        ok, buf = cv2.imencode('.jpg', cv, [cv2.IMWRITE_JPEG_QUALITY, 80])
        data_url = 'data:image/jpeg;base64,' + base64.b64encode(buf.tobytes()).decode()
        q = self.question or self.v['prompt']; self.question = None
        if not q.strip().endswith('?') and 'JSON' not in q:
            q += ' If you list objects reply with JSON {"detections":[{"label":..,"box_2d":[x1,y1,x2,y2],"confidence":..}]} in a 0-1000 frame.'
        body = {'model': self.v['model'], 'temperature': 0, 'max_tokens': 600, 'messages': [{'role': 'user', 'content': [{'type': 'text', 'text': q}, {'type': 'image_url', 'image_url': {'url': data_url}}]}]}
        hdr = {'content-type': 'application/json'}
        if os.environ.get('VLM_API_KEY'): hdr['authorization'] = 'Bearer ' + os.environ['VLM_API_KEY']
        try:
            req = urllib.request.Request(self.v['url'].rstrip('/') + '/chat/completions', data=json.dumps(body).encode(), headers=hdr)
            with urllib.request.urlopen(req, timeout=120) as r: j = json.loads(r.read())
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'VLM request failed: {e}'); return
        text = j.get('choices', [{}])[0].get('message', {}).get('content', '')
        self.pub_text.publish(String(data=text))
        m = re.search(r'\\{[\\s\\S]*\\}', text)
        if not m: return
        try: dets = json.loads(m.group(0)).get('detections', [])
        except Exception: return  # noqa: BLE001
        out = Detection2DArray(); out.header = self.img.header
        for d in dets:
            b = d.get('box_2d') or d.get('bbox') or []
            if len(b) != 4: continue
            x1, y1, x2, y2 = [float(v) for v in b]
            sx, sy = (w / 1000.0, h / 1000.0) if max(b) <= 1000 else (1.0, 1.0)
            det = Detection2D(); det.header = self.img.header
            det.bbox.center.position.x = (x1 + x2) / 2 * sx; det.bbox.center.position.y = (y1 + y2) / 2 * sy
            det.bbox.size_x = abs(x2 - x1) * sx; det.bbox.size_y = abs(y2 - y1) * sy
            hyp = ObjectHypothesisWithPose(); hyp.hypothesis.class_id = str(d.get('label', 'object')); hyp.hypothesis.score = float(d.get('confidence', 0.7))
            det.results.append(hyp); out.detections.append(det)
        self.pub_det.publish(out)


def main():
    rclpy.init(); rclpy.spin(VlmBridge())

if __name__ == '__main__':
    main()
`;

const VLA_BRIDGE_PY = (image: string, format: string) => `#!/usr/bin/env python3
"""VLA bridge (generated by VerticalBot Studio): image + instruction + joint state -> policy server
(${format}: openpi /infer, OpenVLA /act, or the studio JSON contract) -> 7-DoF action -> TwistStamped for
moveit_servo + gripper command. Actions are normalised [-1, 1] and scaled by action_scale_m / action_scale_rad."""
import base64, json, urllib.request
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String, Float64
from geometry_msgs.msg import TwistStamped
from cv_bridge import CvBridge
import cv2


class VlaBridge(Node):
    def __init__(self):
        super().__init__('vla_bridge')
        p = self.declare_parameters('', [('url', 'http://localhost:8000'), ('model', 'pi0'), ('format', '${format}'), ('instruction', ''), ('rate_hz', 5.0), ('action_scale_m', 0.02), ('action_scale_rad', 0.087), ('servo_topic', '/servo_node/delta_twist_cmds'), ('gripper_topic', '/gripper/command')])
        self.v = {x.name: x.value for x in p}
        self.bridge = CvBridge(); self.img = None; self.joints = []; self.instruction = self.v['instruction']; self.enabled = False
        self.create_subscription(Image, '${image}', lambda m: setattr(self, 'img', m), 1)
        self.create_subscription(JointState, '/joint_states', lambda m: setattr(self, 'joints', list(m.position)), 1)
        self.create_subscription(String, '~/instruction', self.set_instruction, 1)
        self.create_subscription(String, '~/enable', lambda m: setattr(self, 'enabled', m.data.lower() in ('1', 'true', 'on')), 1)
        self.pub_twist = self.create_publisher(TwistStamped, self.v['servo_topic'], 1)
        self.pub_grip = self.create_publisher(Float64, self.v['gripper_topic'], 1)
        self.create_timer(1.0 / float(self.v['rate_hz']), self.tick)

    def set_instruction(self, m):
        self.instruction = m.data; self.enabled = True

    def tick(self):
        if not self.enabled or self.img is None: return
        cv = self.bridge.imgmsg_to_cv2(self.img, 'rgb8')
        ok, buf = cv2.imencode('.jpg', cv[:, :, ::-1], [cv2.IMWRITE_JPEG_QUALITY, 85])
        img = 'data:image/jpeg;base64,' + base64.b64encode(buf.tobytes()).decode()
        fmt = self.v['format']; url = self.v['url'].rstrip('/')
        if fmt == 'openvla': url += '/act'; body = {'image': img, 'instruction': self.instruction, 'unnorm_key': self.v['model']}
        elif fmt == 'openpi': url += '/infer'; body = {'observation': {'image': img, 'state': self.joints, 'prompt': self.instruction}}
        else: url += '/act'; body = {'model': self.v['model'], 'image': img, 'instruction': self.instruction, 'proprio': {'joints': self.joints}}
        try:
            req = urllib.request.Request(url, data=json.dumps(body).encode(), headers={'content-type': 'application/json'})
            with urllib.request.urlopen(req, timeout=10) as r: j = json.loads(r.read())
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'policy server: {e}'); return
        acts = j.get('actions') or j.get('action') or j
        a = acts[0] if isinstance(acts, list) and acts and isinstance(acts[0], list) else acts
        if not isinstance(a, list) or len(a) < 6: return
        sm, sr, hz = float(self.v['action_scale_m']), float(self.v['action_scale_rad']), float(self.v['rate_hz'])
        t = TwistStamped(); t.header.stamp = self.get_clock().now().to_msg(); t.header.frame_id = 'camera_optical_frame'
        t.twist.linear.x, t.twist.linear.y, t.twist.linear.z = [float(v) * sm * hz for v in a[:3]]
        t.twist.angular.x, t.twist.angular.y, t.twist.angular.z = [float(v) * sr * hz for v in a[3:6]]
        self.pub_twist.publish(t)
        if len(a) > 6: self.pub_grip.publish(Float64(data=float(a[6])))


def main():
    rclpy.init(); rclpy.spin(VlaBridge())

if __name__ == '__main__':
    main()
`;

function README(name: string, cam: CameraItem, cfg: VisionStackConfig, s: VisionSensor, c: (typeof COMPUTE_TARGETS)[number], models: Record<string, (typeof VISION_MODELS)[number]>, drv: ReturnType<(typeof DRIVER_LAUNCH)[string]>, notes: string[]): string {
  return `# ${name}

Perception stack generated by VerticalBot Studio for camera **${cam.name}**.

| | |
|---|---|
| Sensor | ${s.name} (${s.modality}) — driver \`${drv.pkg}\` |
| Compute | ${c.name} (${c.class}) |
| Tasks | ${cfg.tasks.map((t) => TASK_LABELS[t]).join(', ')} |
| Models | ${Object.entries(models).map(([t, m]) => `${t}: ${m.name} (${m.source})`).join('; ')} |
| Classes | ${cfg.classes.join(', ')} |
| Working distance | ${cfg.workingDistance} m |

## Install

\`\`\`bash
sudo apt install ros-$ROS_DISTRO-vision-msgs ros-$ROS_DISTRO-cv-bridge ros-$ROS_DISTRO-image-transport ${drv.pkg.includes('_') ? '' : `ros-$ROS_DISTRO-${drv.pkg.replace(/_/g, '-')}`}
# detector / tracker / 3D nodes
cd ~/ros2_ws/src && git clone https://github.com/mgonzs13/yolo_ros.git && pip3 install -r yolo_ros/requirements.txt
# models: export from Ultralytics -> put next to the launch or give an absolute path in config/detector.yaml
yolo export model=${models.detect?.id ?? 'yolov8n'}.pt format=${c.runtimes.includes('tensorrt') ? 'engine half=True' : 'onnx'}
cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash
ros2 launch ${name} perception.launch.py
\`\`\`

## Topics

- \`${drv.topics.image || drv.topics.cloud}\` — sensor
- \`/yolo/detections\` (vision_msgs/Detection2DArray), \`/yolo/tracking\` (with ids), \`/yolo/detections_3d\` (Detection3DArray in base_link)${cfg.tasks.some((t) => t === 'cloud_detect' || t === 'cloud_segment') ? '\n- `/cloud/detections`, `/cloud/ground`, `/cloud/objects`, `/cloud/rows`' : ''}${cfg.tasks.includes('vlm_query') ? '\n- `/vlm_bridge/answer`, `/vlm_bridge/detections`; ask with `ros2 topic pub /vlm_bridge/question std_msgs/String "{data: how many ripe apples?}"`' : ''}${cfg.tasks.includes('vla_policy') ? '\n- `/vla_bridge/instruction` (start), `/vla_bridge/enable` — publishes `TwistStamped` for moveit_servo' : ''}

## Hand-eye

\`config/hand_eye.yaml\` holds the camera pose in \`${cfg.handEyeMode === 'eye_to_hand' ? 'base_link' : 'tool0'}\` taken from the studio scene; refine it with
\`easy_handeye2\` or \`moveit_calibration\` and update the static transform in the launch file.

## Notes
${notes.map((n) => `- ${n}`).join('\n')}
- Studio simulation and this package share the same class list, thresholds and coordinate conventions (optical frame +Z forward).
`;
}
