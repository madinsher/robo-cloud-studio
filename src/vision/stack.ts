/**
 * Machine-vision (СТЗ) stack: hardware + software selection for mono / stereo / RGB-D / ToF / LiDAR perception.
 *
 * A "vision stack" is (1) a sensor (camera, depth camera, 3D LiDAR…), (2) a compute target (Jetson, x86+GPU,
 * accelerator…), and (3) one model per task — detection, tracking, classification, segmentation, 6D pose /
 * position estimation, keypoints, depth, point-cloud detection/segmentation, grasp planning, VLM queries and
 * VLA policies, plus the "follow" behaviour built on tracking. This module provides:
 *
 *  - catalogues of sensors, compute targets and ready-made models (YOLO family, DETR, SAM, trackers, 6D pose,
 *    monocular/stereo depth, point-cloud networks, VLMs, VLAs) with the real projects/packages behind them;
 *  - a recommender: tasks + environment + modality + compute budget + working distance → ranked stacks;
 *  - the per-camera configuration (`VisionStackConfig`) stored in `Camera.params.visionStack`.
 *
 * Model *execution* (simulated ground truth, ONNX Runtime Web in the browser, the studio server, an
 * OpenAI-compatible VLM endpoint, a VLA action server, ROS 2 `vision_msgs`) lives in `models.ts`; point-cloud
 * processing in `pointcloud.ts`; camera geometry and depth error models in `camera_model.ts`; the runtime that
 * ties them to a station camera in `pipeline.ts`; ROS 2 perception package export in `vision_ros.ts`.
 */
import type { Camera as CameraItem } from '../core/items/item';
import type { Environment } from '../mobile/navstack';

// ---------------------------------------------------------------------------------------------
// Types
// ---------------------------------------------------------------------------------------------

export type VisionModality = 'mono' | 'stereo' | 'rgbd' | 'tof' | 'lidar3d' | 'thermal' | 'multispectral' | 'event';
export type VisionTask = 'detect' | 'track' | 'classify' | 'segment' | 'pose' | 'keypoints' | 'depth' | 'cloud_detect' | 'cloud_segment' | 'grasp' | 'vlm_query' | 'vla_policy' | 'follow' | 'measure';
export type ComputeClass = 'mcu' | 'edge_low' | 'edge_mid' | 'edge_high' | 'workstation' | 'cloud';
export type ModelFamily = 'yolo' | 'detr' | 'sam' | 'cnn' | 'clip' | 'tracker' | 'pose6d' | 'depth' | 'pointcloud' | 'vlm' | 'vla' | 'classical' | 'grasp';
export type Runtime = 'onnx' | 'tensorrt' | 'pytorch' | 'openvino' | 'tflite' | 'http' | 'ros2' | 'pcl' | 'open3d';
/** How the studio executes the model. */
export type Adapter = 'simulated' | 'onnx_yolo' | 'onnx_cls' | 'http' | 'vlm_openai' | 'vla_http' | 'ros2' | 'builtin';
export type VisionEnvironment = Environment;

export interface VisionSensor {
  id: string;
  name: string;
  vendor: string;
  modality: VisionModality;
  /** Image resolution (px) and frame rate. */
  resolution: [number, number];
  fps: number;
  /** Horizontal / vertical field of view (deg). */
  hfov: number;
  vfov: number;
  /** Usable depth range (m) — null for pure 2D sensors. */
  depthRange: [number, number] | null;
  /** Depth error model: sigma(z) = a + b * z^2 (m). Stereo grows quadratically, ToF/LiDAR ~constant. */
  depthError: { a: number; b: number } | null;
  /** Stereo baseline (m) when applicable. */
  baseline?: number;
  globalShutter: boolean;
  /** 0..1 robustness to direct sunlight / outdoor use. */
  outdoor: number;
  /** Works in darkness (active illumination, thermal, LiDAR). */
  night: boolean;
  /** Relative cost 1..5. */
  cost: number;
  ip: string;
  interface: string;
  rosDriver: string;
  /** LiDAR: channels and points per second. */
  channels?: number;
  pointsPerSecond?: number;
  notes: string;
}

export interface ComputeTarget {
  id: string;
  name: string;
  class: ComputeClass;
  /** INT8/FP16 TOPS (approximate, marketing figures). */
  tops: number;
  powerW: number;
  runtimes: Runtime[];
  cost: number;
  notes: string;
}

export interface VisionModelSpec {
  id: string;
  name: string;
  family: ModelFamily;
  tasks: VisionTask[];
  inputs: Array<'image' | 'depth' | 'pointcloud' | 'text' | 'proprio'>;
  /** Prompted with free-text classes (open vocabulary). */
  openVocabulary: boolean;
  /** Parameters (millions) — 0 for classical algorithms. */
  paramsM: number;
  /** Typical inference latency per compute class (ms); missing = not practical there. */
  latencyMs: Partial<Record<ComputeClass, number>>;
  /** 0..1 quality figure on the model's benchmark class (mAP / mIoU / success rate, normalised). */
  accuracy: number;
  runtimes: Runtime[];
  formats: string[];
  license: string;
  source: string;
  /** 0..1 fit for agriculture (fruit, foliage, outdoor lighting). */
  agriculture: number;
  adapter: Adapter;
  notes: string;
}

export const TASK_LABELS: Record<VisionTask, string> = {
  detect: 'Detection (boxes)', track: 'Tracking (IDs over time)', classify: 'Classification', segment: 'Segmentation (masks)', pose: '6D pose / 3D position', keypoints: 'Keypoints (stems, joints)', depth: 'Depth estimation', cloud_detect: 'Point-cloud detection (3D boxes)', cloud_segment: 'Point-cloud segmentation (ground, rows, canopy)', grasp: 'Grasp pose generation', vlm_query: 'VLM query (describe / ground / answer)', vla_policy: 'VLA policy (language → robot actions)', follow: 'Following (servo on a tracked target)', measure: 'Measurement (size, distance, volume)',
};
export const MODALITY_LABELS: Record<VisionModality, string> = { mono: 'Mono RGB camera', stereo: 'Stereo camera', rgbd: 'RGB-D (structured light / active stereo)', tof: 'Time-of-flight camera', lidar3d: '3D LiDAR', thermal: 'Thermal camera', multispectral: 'Multispectral / NIR camera', event: 'Event camera' };
export const COMPUTE_LABELS: Record<ComputeClass, string> = { mcu: 'Microcontroller (Coral / Hailo module)', edge_low: 'Edge low (Jetson Orin Nano / RPi 5 + accelerator)', edge_mid: 'Edge mid (Jetson Orin NX)', edge_high: 'Edge high (Jetson AGX Orin / Thor)', workstation: 'Workstation / IPC with RTX GPU', cloud: 'Cloud GPU (remote inference)' };

// ---------------------------------------------------------------------------------------------
// Sensors
// ---------------------------------------------------------------------------------------------

export const VISION_SENSORS: VisionSensor[] = [
  { id: 'usb_rgb', name: 'USB / MIPI RGB camera (rolling shutter)', vendor: 'generic', modality: 'mono', resolution: [1920, 1080], fps: 30, hfov: 70, vfov: 43, depthRange: null, depthError: null, globalShutter: false, outdoor: 0.6, night: false, cost: 1, ip: 'IP40', interface: 'USB3 / MIPI-CSI', rosDriver: 'usb_cam / v4l2_camera', notes: 'Cheapest option; motion blur and rolling-shutter skew on moving platforms.' },
  { id: 'gs_rgb', name: 'Global-shutter industrial RGB (GigE / USB3 Vision)', vendor: 'Basler / FLIR / Allied', modality: 'mono', resolution: [2448, 2048], fps: 35, hfov: 60, vfov: 50, depthRange: null, depthError: null, globalShutter: true, outdoor: 0.8, night: false, cost: 3, ip: 'IP67 (housed)', interface: 'GigE Vision / USB3 Vision', rosDriver: 'pylon_ros2_camera / spinnaker_camera_driver', notes: 'Sharp images on moving vehicles, PoE, hardware trigger for stereo/strobe sync.' },
  { id: 'zed_x', name: 'Stereolabs ZED X / ZED 2i', vendor: 'Stereolabs', modality: 'stereo', resolution: [1920, 1200], fps: 60, hfov: 110, vfov: 80, depthRange: [0.3, 20], depthError: { a: 0.01, b: 0.004 }, baseline: 0.12, globalShutter: true, outdoor: 0.9, night: false, cost: 3, ip: 'IP66', interface: 'GMSL2 (ZED X) / USB3', rosDriver: 'zed_wrapper (zed-ros2-wrapper)', notes: 'Passive stereo, works in sunlight, neural depth on Jetson; no depth in darkness or on textureless surfaces.' },
  { id: 'oak_d_pro', name: 'Luxonis OAK-D Pro (W / PoE)', vendor: 'Luxonis', modality: 'stereo', resolution: [1280, 800], fps: 60, hfov: 95, vfov: 70, depthRange: [0.2, 12], depthError: { a: 0.01, b: 0.006 }, baseline: 0.075, globalShutter: true, outdoor: 0.7, night: true, cost: 2, ip: 'IP67 (PoE)', interface: 'USB3 / PoE', rosDriver: 'depthai_ros_driver', notes: 'Active IR dot projector for low-texture scenes; on-board Myriad X runs YOLO models (depthai blob).' },
  { id: 'realsense_d455', name: 'Intel RealSense D455 / D435i', vendor: 'Intel', modality: 'rgbd', resolution: [1280, 720], fps: 30, hfov: 87, vfov: 58, depthRange: [0.4, 6], depthError: { a: 0.005, b: 0.01 }, baseline: 0.095, globalShutter: true, outdoor: 0.5, night: true, cost: 2, ip: 'IP40', interface: 'USB3', rosDriver: 'realsense2_camera', notes: 'Active IR stereo; depth washes out in direct sunlight beyond ~3 m; IMU on board.' },
  { id: 'orbbec_gemini', name: 'Orbbec Gemini 2 / Femto', vendor: 'Orbbec', modality: 'rgbd', resolution: [1280, 800], fps: 30, hfov: 91, vfov: 66, depthRange: [0.25, 10], depthError: { a: 0.005, b: 0.008 }, baseline: 0.05, globalShutter: true, outdoor: 0.5, night: true, cost: 2, ip: 'IP40', interface: 'USB3', rosDriver: 'orbbec_camera (OrbbecSDK_ROS2)', notes: 'RealSense-class active stereo; Femto Mega adds ToF.' },
  { id: 'kinect_azure_tof', name: 'ToF camera (Azure Kinect / Basler blaze / PMD)', vendor: 'Microsoft / Basler / pmd', modality: 'tof', resolution: [640, 576], fps: 30, hfov: 75, vfov: 65, depthRange: [0.3, 8], depthError: { a: 0.006, b: 0.0005 }, globalShutter: true, outdoor: 0.4, night: true, cost: 3, ip: 'IP67 (blaze)', interface: 'USB3 / GigE', rosDriver: 'azure_kinect_ros_driver / blaze_ros', notes: 'Constant depth error, independent of texture; struggles in bright sun and on dark/shiny surfaces.' },
  { id: 'zivid', name: 'Zivid 2+ / Photoneo structured light (bin picking)', vendor: 'Zivid / Photoneo', modality: 'rgbd', resolution: [2448, 2048], fps: 5, hfov: 45, vfov: 35, depthRange: [0.3, 2.5], depthError: { a: 0.0002, b: 0.0002 }, globalShutter: true, outdoor: 0.2, night: true, cost: 5, ip: 'IP65', interface: 'GigE', rosDriver: 'zivid_ros / phoxi_camera', notes: 'Sub-millimetre depth for industrial picking and metrology; indoor, static scenes.' },
  { id: 'velodyne_vlp16', name: 'Velodyne VLP-16 (Puck)', vendor: 'Velodyne', modality: 'lidar3d', resolution: [1800, 16], fps: 10, hfov: 360, vfov: 30, depthRange: [0.5, 100], depthError: { a: 0.03, b: 0 }, globalShutter: true, outdoor: 1, night: true, cost: 3, ip: 'IP67', interface: 'Ethernet', rosDriver: 'velodyne_driver + velodyne_pointcloud', channels: 16, pointsPerSecond: 300000, notes: 'Classic 16-channel spinning LiDAR; sparse vertically.' },
  { id: 'ouster_os1', name: 'Ouster OS1-64 / OS0-128', vendor: 'Ouster', modality: 'lidar3d', resolution: [2048, 64], fps: 10, hfov: 360, vfov: 45, depthRange: [0.3, 120], depthError: { a: 0.02, b: 0 }, globalShutter: true, outdoor: 1, night: true, cost: 4, ip: 'IP68', interface: 'Ethernet (PoE)', rosDriver: 'ouster-ros', channels: 64, pointsPerSecond: 1300000, notes: 'Dense digital LiDAR with reflectivity/near-IR images usable by 2D networks.' },
  { id: 'livox_mid360', name: 'Livox Mid-360 / HAP', vendor: 'Livox (DJI)', modality: 'lidar3d', resolution: [0, 0], fps: 10, hfov: 360, vfov: 59, depthRange: [0.1, 70], depthError: { a: 0.02, b: 0 }, globalShutter: true, outdoor: 1, night: true, cost: 2, ip: 'IP67', interface: 'Ethernet', rosDriver: 'livox_ros_driver2', pointsPerSecond: 200000, notes: 'Low-cost non-repetitive scan pattern; dense over time, standard on FAST-LIO builds.' },
  { id: 'hesai_xt32', name: 'Hesai XT32 / Pandar', vendor: 'Hesai', modality: 'lidar3d', resolution: [2000, 32], fps: 10, hfov: 360, vfov: 31, depthRange: [0.05, 120], depthError: { a: 0.01, b: 0 }, globalShutter: true, outdoor: 1, night: true, cost: 4, ip: 'IP67', interface: 'Ethernet', rosDriver: 'HesaiLidar_ROS_2.0', channels: 32, pointsPerSecond: 640000, notes: 'Zero blind spot near the sensor; automotive grade.' },
  { id: 'flir_boson', name: 'Thermal camera (FLIR Boson / Lepton)', vendor: 'Teledyne FLIR', modality: 'thermal', resolution: [640, 512], fps: 60, hfov: 50, vfov: 40, depthRange: null, depthError: null, globalShutter: true, outdoor: 0.9, night: true, cost: 4, ip: 'IP40 (module)', interface: 'USB / MIPI', rosDriver: 'flir_boson_usb', notes: 'People/animal detection at night, water-stress scouting; low resolution.' },
  { id: 'micasense', name: 'Multispectral camera (MicaSense RedEdge / Altum)', vendor: 'MicaSense', modality: 'multispectral', resolution: [1456, 1088], fps: 1, hfov: 47, vfov: 35, depthRange: null, depthError: null, globalShutter: true, outdoor: 1, night: false, cost: 5, ip: 'IP40', interface: 'Ethernet / trigger', rosDriver: 'custom (HTTP API)', notes: 'NDVI/NDRE indices for scouting and disease mapping; not for real-time control.' },
  { id: 'prophesee_evk', name: 'Event camera (Prophesee EVK4 / iniVation)', vendor: 'Prophesee', modality: 'event', resolution: [1280, 720], fps: 10000, hfov: 60, vfov: 45, depthRange: null, depthError: null, globalShutter: true, outdoor: 0.8, night: true, cost: 5, ip: 'IP40', interface: 'USB3', rosDriver: 'metavision_ros_driver', notes: 'Microsecond latency and 120 dB dynamic range; needs event-based algorithms, few ready models.' },
];

// ---------------------------------------------------------------------------------------------
// Compute
// ---------------------------------------------------------------------------------------------

export const COMPUTE_TARGETS: ComputeTarget[] = [
  { id: 'coral_tpu', name: 'Google Coral Edge TPU / Hailo-8 M.2 (+ SBC)', class: 'edge_low', tops: 8, powerW: 4, runtimes: ['tflite', 'onnx'], cost: 1, notes: 'INT8 only; YOLOv8n-class models at 30+ fps; no VLM.' },
  { id: 'oak_myriad', name: 'On-camera Myriad X (OAK-D)', class: 'mcu', tops: 4, powerW: 3, runtimes: ['openvino'], cost: 1, notes: 'Runs the detector inside the camera; host only receives detections + depth.' },
  { id: 'rpi5', name: 'Raspberry Pi 5 (+ Hailo-8L AI kit)', class: 'edge_low', tops: 13, powerW: 8, runtimes: ['onnx', 'tflite'], cost: 1, notes: 'Small detectors at 15–30 fps with Hailo; CPU-only inference is 1–3 fps.' },
  { id: 'jetson_orin_nano', name: 'NVIDIA Jetson Orin Nano 8 GB', class: 'edge_low', tops: 40, powerW: 15, runtimes: ['tensorrt', 'onnx', 'pytorch'], cost: 2, notes: 'YOLOv8s/11s 30+ fps TensorRT; small VLMs (Florence-2 base, Moondream) at ~1–2 s.' },
  { id: 'jetson_orin_nx', name: 'NVIDIA Jetson Orin NX 16 GB', class: 'edge_mid', tops: 100, powerW: 25, runtimes: ['tensorrt', 'onnx', 'pytorch'], cost: 3, notes: 'Detection + segmentation + tracking in real time; Isaac ROS accelerated nodes.' },
  { id: 'jetson_agx_orin', name: 'NVIDIA Jetson AGX Orin 64 GB', class: 'edge_high', tops: 275, powerW: 60, runtimes: ['tensorrt', 'onnx', 'pytorch'], cost: 4, notes: 'Multi-camera pipelines, SAM2/Depth-Anything, 7B VLMs (Qwen2-VL, LLaVA) at a few tokens/s.' },
  { id: 'jetson_thor', name: 'NVIDIA Jetson Thor', class: 'edge_high', tops: 2000, powerW: 130, runtimes: ['tensorrt', 'onnx', 'pytorch'], cost: 5, notes: 'VLA policies (π0, OpenVLA, GR00T) on the robot.' },
  { id: 'ipc_rtx', name: 'x86 IPC + RTX 4070/4090 (or A2000 embedded)', class: 'workstation', tops: 400, powerW: 350, runtimes: ['tensorrt', 'onnx', 'pytorch', 'openvino'], cost: 4, notes: 'Full PyTorch stack, training on site, any VLM/VLA at interactive rates.' },
  { id: 'intel_ipc', name: 'x86 IPC (Intel CPU / iGPU, OpenVINO)', class: 'edge_mid', tops: 20, powerW: 45, runtimes: ['openvino', 'onnx'], cost: 2, notes: 'Fanless industrial PCs; YOLO at 10–30 fps with OpenVINO; no VLM.' },
  { id: 'cloud_gpu', name: 'Cloud / on-prem GPU server (vLLM, Triton)', class: 'cloud', tops: 2000, powerW: 700, runtimes: ['pytorch', 'tensorrt', 'http'], cost: 3, notes: 'Any model; requires connectivity (LTE/5G/WiFi) — 100–500 ms round trip, not for closed-loop control.' },
];

// ---------------------------------------------------------------------------------------------
// Models
// ---------------------------------------------------------------------------------------------

const L = (a: number, b: number, c: number, d: number, e: number, f: number): Partial<Record<ComputeClass, number>> => ({ mcu: a, edge_low: b, edge_mid: c, edge_high: d, workstation: e, cloud: f });
const NA = 1e9;

export const VISION_MODELS: VisionModelSpec[] = [
  // --- detection
  { id: 'yolov8n', name: 'YOLOv8n / YOLO11n', family: 'yolo', tasks: ['detect'], inputs: ['image'], openVocabulary: false, paramsM: 3.2, latencyMs: L(35, 12, 6, 3, 1.5, 1), accuracy: 0.37, runtimes: ['onnx', 'tensorrt', 'openvino', 'tflite', 'pytorch'], formats: ['.onnx', '.pt', '.engine', '.blob'], license: 'AGPL-3.0 (Ultralytics)', source: 'https://github.com/ultralytics/ultralytics', agriculture: 0.8, adapter: 'onnx_yolo', notes: 'Smallest real-time detector; fine-tune on your fruit/weed dataset (Roboflow export → ONNX).' },
  { id: 'yolov8s', name: 'YOLOv8s / YOLO11s', family: 'yolo', tasks: ['detect'], inputs: ['image'], openVocabulary: false, paramsM: 11, latencyMs: L(90, 25, 10, 5, 2, 1.5), accuracy: 0.45, runtimes: ['onnx', 'tensorrt', 'openvino', 'pytorch'], formats: ['.onnx', '.pt', '.engine'], license: 'AGPL-3.0 (Ultralytics)', source: 'https://github.com/ultralytics/ultralytics', agriculture: 0.85, adapter: 'onnx_yolo', notes: 'Best accuracy/latency trade-off on Jetson Orin.' },
  { id: 'yolov8m', name: 'YOLOv8m / YOLO11m', family: 'yolo', tasks: ['detect'], inputs: ['image'], openVocabulary: false, paramsM: 26, latencyMs: L(NA, 60, 22, 10, 4, 3), accuracy: 0.50, runtimes: ['onnx', 'tensorrt', 'pytorch'], formats: ['.onnx', '.pt', '.engine'], license: 'AGPL-3.0 (Ultralytics)', source: 'https://github.com/ultralytics/ultralytics', agriculture: 0.85, adapter: 'onnx_yolo', notes: 'Small fruit at distance; needs Orin NX or better for 30 fps.' },
  { id: 'yolov10', name: 'YOLOv10 (NMS-free)', family: 'yolo', tasks: ['detect'], inputs: ['image'], openVocabulary: false, paramsM: 7, latencyMs: L(70, 20, 8, 4, 2, 1.5), accuracy: 0.46, runtimes: ['onnx', 'tensorrt', 'pytorch'], formats: ['.onnx', '.pt'], license: 'AGPL-3.0', source: 'https://github.com/THU-MIG/yolov10', agriculture: 0.8, adapter: 'onnx_yolo', notes: 'End-to-end (no NMS post-processing), lower latency variance.' },
  { id: 'rtdetr', name: 'RT-DETR / D-FINE', family: 'detr', tasks: ['detect'], inputs: ['image'], openVocabulary: false, paramsM: 32, latencyMs: L(NA, 90, 35, 15, 5, 4), accuracy: 0.53, runtimes: ['onnx', 'tensorrt', 'pytorch'], formats: ['.onnx', '.pt'], license: 'Apache-2.0', source: 'https://github.com/lyuwenyu/RT-DETR', agriculture: 0.8, adapter: 'onnx_yolo', notes: 'Transformer detector without NMS; better on crowded scenes (clusters of fruit).' },
  { id: 'yolo_world', name: 'YOLO-World (open vocabulary)', family: 'yolo', tasks: ['detect'], inputs: ['image', 'text'], openVocabulary: true, paramsM: 40, latencyMs: L(NA, 120, 45, 18, 7, 5), accuracy: 0.35, runtimes: ['onnx', 'tensorrt', 'pytorch'], formats: ['.onnx', '.pt'], license: 'GPL-3.0', source: 'https://github.com/AILab-CVC/YOLO-World', agriculture: 0.7, adapter: 'onnx_yolo', notes: 'Type the class names ("ripe apple", "irrigation pipe") — no training; export with the vocabulary baked in.' },
  { id: 'grounding_dino', name: 'Grounding DINO 1.5 / OWLv2', family: 'detr', tasks: ['detect', 'vlm_query'], inputs: ['image', 'text'], openVocabulary: true, paramsM: 172, latencyMs: L(NA, NA, 600, 250, 80, 60), accuracy: 0.52, runtimes: ['pytorch', 'http'], formats: ['.pth'], license: 'Apache-2.0', source: 'https://github.com/IDEA-Research/GroundingDINO', agriculture: 0.7, adapter: 'http', notes: 'Best zero-shot grounding; too slow for closed loop, ideal for auto-labelling and VLM pipelines.' },
  // --- segmentation
  { id: 'yolov8_seg', name: 'YOLOv8-seg / YOLO11-seg', family: 'yolo', tasks: ['segment', 'detect'], inputs: ['image'], openVocabulary: false, paramsM: 12, latencyMs: L(150, 35, 14, 6, 3, 2), accuracy: 0.42, runtimes: ['onnx', 'tensorrt', 'pytorch'], formats: ['.onnx', '.pt'], license: 'AGPL-3.0 (Ultralytics)', source: 'https://github.com/ultralytics/ultralytics', agriculture: 0.85, adapter: 'onnx_yolo', notes: 'Instance masks for fruit/branches — masks give better 3D centroids from depth than boxes.' },
  { id: 'fastsam', name: 'FastSAM / MobileSAM', family: 'sam', tasks: ['segment'], inputs: ['image', 'text'], openVocabulary: true, paramsM: 68, latencyMs: L(NA, 80, 30, 12, 5, 4), accuracy: 0.6, runtimes: ['onnx', 'tensorrt', 'pytorch'], formats: ['.onnx', '.pt'], license: 'AGPL-3.0 / Apache-2.0', source: 'https://github.com/CASIA-IVA-Lab/FastSAM', agriculture: 0.7, adapter: 'onnx_yolo', notes: 'Segment-anything at YOLO speed; prompt with boxes from a detector.' },
  { id: 'sam2', name: 'SAM 2 (video segmentation)', family: 'sam', tasks: ['segment', 'track'], inputs: ['image', 'text'], openVocabulary: true, paramsM: 224, latencyMs: L(NA, NA, 200, 70, 25, 20), accuracy: 0.75, runtimes: ['pytorch', 'onnx', 'http'], formats: ['.pt', '.onnx'], license: 'Apache-2.0', source: 'https://github.com/facebookresearch/sam2', agriculture: 0.8, adapter: 'http', notes: 'Promptable masks that persist across frames — tracking a chosen fruit/branch through occlusion.' },
  { id: 'mask2former', name: 'Mask2Former / SegFormer (semantic)', family: 'cnn', tasks: ['segment'], inputs: ['image'], openVocabulary: false, paramsM: 44, latencyMs: L(NA, 150, 60, 25, 9, 7), accuracy: 0.6, runtimes: ['onnx', 'pytorch'], formats: ['.onnx', '.pt'], license: 'MIT / NVIDIA', source: 'https://github.com/facebookresearch/Mask2Former', agriculture: 0.75, adapter: 'http', notes: 'Semantic classes (soil, crop row, canopy, sky, path) for traversability and row following.' },
  // --- classification
  { id: 'efficientnet', name: 'EfficientNet-B0 / MobileNetV3 (classifier)', family: 'cnn', tasks: ['classify'], inputs: ['image'], openVocabulary: false, paramsM: 5, latencyMs: L(10, 4, 2, 1, 0.5, 0.5), accuracy: 0.77, runtimes: ['onnx', 'tensorrt', 'tflite', 'openvino'], formats: ['.onnx', '.tflite'], license: 'Apache-2.0', source: 'https://github.com/huggingface/pytorch-image-models', agriculture: 0.8, adapter: 'onnx_cls', notes: 'Ripeness / disease / grade classification on crops of detected boxes.' },
  { id: 'yolov8_cls', name: 'YOLOv8-cls / YOLO11-cls', family: 'yolo', tasks: ['classify'], inputs: ['image'], openVocabulary: false, paramsM: 6, latencyMs: L(12, 4, 2, 1, 0.5, 0.5), accuracy: 0.76, runtimes: ['onnx', 'tensorrt', 'pytorch'], formats: ['.onnx', '.pt'], license: 'AGPL-3.0 (Ultralytics)', source: 'https://github.com/ultralytics/ultralytics', agriculture: 0.8, adapter: 'onnx_cls', notes: 'Same training tooling as the detector.' },
  { id: 'clip', name: 'CLIP / SigLIP (zero-shot classification)', family: 'clip', tasks: ['classify', 'vlm_query'], inputs: ['image', 'text'], openVocabulary: true, paramsM: 150, latencyMs: L(NA, 90, 35, 15, 5, 4), accuracy: 0.7, runtimes: ['onnx', 'pytorch', 'http'], formats: ['.onnx', '.pt'], license: 'MIT / Apache-2.0', source: 'https://github.com/openai/CLIP', agriculture: 0.6, adapter: 'http', notes: 'Classify with text labels ("ripe", "unripe", "damaged") without training data.' },
  // --- tracking
  { id: 'bytetrack', name: 'ByteTrack', family: 'tracker', tasks: ['track', 'follow'], inputs: ['image'], openVocabulary: false, paramsM: 0, latencyMs: L(1, 1, 0.5, 0.3, 0.2, 0.2), accuracy: 0.8, runtimes: ['onnx', 'pytorch', 'ros2'], formats: [], license: 'MIT', source: 'https://github.com/ifzhang/ByteTrack', agriculture: 0.85, adapter: 'builtin', notes: 'Motion-only association of detector boxes (built into the studio, Ultralytics and yolo_ros); the default choice.' },
  { id: 'botsort', name: 'BoT-SORT / DeepSORT (appearance re-ID)', family: 'tracker', tasks: ['track', 'follow'], inputs: ['image'], openVocabulary: false, paramsM: 2, latencyMs: L(NA, 8, 4, 2, 1, 1), accuracy: 0.83, runtimes: ['pytorch', 'onnx'], formats: ['.pt'], license: 'MIT', source: 'https://github.com/NirAharon/BoT-SORT', agriculture: 0.8, adapter: 'builtin', notes: 'Re-identifies targets after occlusion (following a person between trees).' },
  { id: 'ocsort', name: 'OC-SORT', family: 'tracker', tasks: ['track', 'follow'], inputs: ['image'], openVocabulary: false, paramsM: 0, latencyMs: L(1, 1, 0.5, 0.3, 0.2, 0.2), accuracy: 0.8, runtimes: ['pytorch'], formats: [], license: 'MIT', source: 'https://github.com/noahcao/OC_SORT', agriculture: 0.8, adapter: 'builtin', notes: 'Observation-centric; robust to non-linear motion (swinging fruit, people).' },
  // --- pose / keypoints
  { id: 'yolov8_pose', name: 'YOLOv8-pose / YOLO11-pose (keypoints)', family: 'yolo', tasks: ['keypoints', 'detect'], inputs: ['image'], openVocabulary: false, paramsM: 11, latencyMs: L(90, 25, 10, 5, 2, 1.5), accuracy: 0.5, runtimes: ['onnx', 'tensorrt', 'pytorch'], formats: ['.onnx', '.pt'], license: 'AGPL-3.0 (Ultralytics)', source: 'https://github.com/ultralytics/ultralytics', agriculture: 0.8, adapter: 'onnx_yolo', notes: 'Custom keypoints (stem/calyx, peduncle, cutting point) trained like the detector.' },
  { id: 'foundationpose', name: 'FoundationPose (6D pose, CAD or few-shot)', family: 'pose6d', tasks: ['pose'], inputs: ['image', 'depth'], openVocabulary: false, paramsM: 140, latencyMs: L(NA, NA, 400, 150, 50, 40), accuracy: 0.85, runtimes: ['pytorch', 'tensorrt', 'ros2'], formats: ['.pth'], license: 'NVIDIA Source Code License', source: 'https://github.com/NVlabs/FoundationPose', agriculture: 0.5, adapter: 'ros2', notes: 'Isaac ROS node; needs RGB-D and a mesh of the object — industrial parts, bins, crates.' },
  { id: 'megapose', name: 'MegaPose / CosyPose', family: 'pose6d', tasks: ['pose'], inputs: ['image', 'depth'], openVocabulary: false, paramsM: 80, latencyMs: L(NA, NA, 800, 300, 100, 80), accuracy: 0.8, runtimes: ['pytorch', 'http'], formats: ['.pth'], license: 'Apache-2.0', source: 'https://github.com/megapose6d/megapose6d', agriculture: 0.5, adapter: 'http', notes: 'CAD-based 6D pose from a detector box (happypose server).' },
  { id: 'dope', name: 'DOPE (NVIDIA deep object pose)', family: 'pose6d', tasks: ['pose'], inputs: ['image'], openVocabulary: false, paramsM: 60, latencyMs: L(NA, 120, 45, 20, 8, 6), accuracy: 0.7, runtimes: ['tensorrt', 'ros2'], formats: ['.onnx', '.pth'], license: 'CC BY-NC-SA 4.0', source: 'https://github.com/NVlabs/Deep_Object_Pose', agriculture: 0.4, adapter: 'ros2', notes: 'RGB-only 6D pose of known objects trained on synthetic data (Isaac Sim / this studio).' },
  { id: 'depth_centroid', name: 'Depth-centroid position (box/mask + depth image)', family: 'classical', tasks: ['pose', 'measure'], inputs: ['image', 'depth'], openVocabulary: false, paramsM: 0, latencyMs: L(1, 1, 0.5, 0.3, 0.2, 0.2), accuracy: 0.75, runtimes: ['pcl', 'ros2'], formats: [], license: 'built-in', source: 'studio', agriculture: 0.9, adapter: 'builtin', notes: 'Median depth inside the mask → 3D point; enough for fruit picking with a compliant gripper. Orientation from the local surface normal.' },
  { id: 'size_prior_position', name: 'Mono position from known object size', family: 'classical', tasks: ['pose'], inputs: ['image'], openVocabulary: false, paramsM: 0, latencyMs: L(1, 1, 0.5, 0.3, 0.2, 0.2), accuracy: 0.5, runtimes: ['pcl'], formats: [], license: 'built-in', source: 'studio', agriculture: 0.7, adapter: 'builtin', notes: 'z = f·D/w with the crop diameter as prior; ±15 % range error, no depth sensor needed.' },
  // --- depth
  { id: 'depth_anything', name: 'Depth Anything V2 (monocular metric depth)', family: 'depth', tasks: ['depth'], inputs: ['image'], openVocabulary: false, paramsM: 25, latencyMs: L(NA, 120, 45, 20, 7, 5), accuracy: 0.6, runtimes: ['onnx', 'tensorrt', 'pytorch'], formats: ['.onnx', '.pth'], license: 'Apache-2.0', source: 'https://github.com/DepthAnything/Depth-Anything-V2', agriculture: 0.7, adapter: 'http', notes: 'Relative depth from one camera; metric variants need scale from a known size or a few LiDAR points.' },
  { id: 'raft_stereo', name: 'RAFT-Stereo / FoundationStereo', family: 'depth', tasks: ['depth'], inputs: ['image'], openVocabulary: false, paramsM: 11, latencyMs: L(NA, 200, 80, 35, 12, 10), accuracy: 0.85, runtimes: ['pytorch', 'tensorrt', 'ros2'], formats: ['.pth', '.onnx'], license: 'MIT', source: 'https://github.com/princeton-vl/RAFT-Stereo', agriculture: 0.85, adapter: 'ros2', notes: 'Learned stereo matching (Isaac ROS ESS is the TensorRT equivalent) — dense depth through foliage.' },
  { id: 'sgbm', name: 'OpenCV SGBM stereo (classical)', family: 'classical', tasks: ['depth'], inputs: ['image'], openVocabulary: false, paramsM: 0, latencyMs: L(NA, 40, 20, 10, 5, 5), accuracy: 0.6, runtimes: ['pcl', 'ros2'], formats: [], license: 'Apache-2.0', source: 'https://opencv.org', agriculture: 0.7, adapter: 'builtin', notes: 'stereo_image_proc; no GPU; holes on low texture.' },
  // --- point clouds
  { id: 'pcl_pipeline', name: 'PCL / Open3D pipeline (voxel → ground RANSAC → Euclidean clustering)', family: 'classical', tasks: ['cloud_detect', 'cloud_segment', 'measure'], inputs: ['pointcloud'], openVocabulary: false, paramsM: 0, latencyMs: L(NA, 30, 15, 8, 4, 4), accuracy: 0.65, runtimes: ['pcl', 'open3d', 'ros2'], formats: [], license: 'BSD', source: 'https://pointclouds.org', agriculture: 0.85, adapter: 'builtin', notes: 'Trunk/obstacle clusters, row lines, canopy volume; implemented in the studio (`pointcloud.ts`) and exported as a ROS 2 Python node.' },
  { id: 'pointpillars', name: 'PointPillars / CenterPoint (OpenPCDet, mmdetection3d)', family: 'pointcloud', tasks: ['cloud_detect'], inputs: ['pointcloud'], openVocabulary: false, paramsM: 5, latencyMs: L(NA, 60, 25, 12, 5, 4), accuracy: 0.7, runtimes: ['tensorrt', 'pytorch', 'ros2'], formats: ['.onnx', '.pth'], license: 'Apache-2.0', source: 'https://github.com/open-mmlab/OpenPCDet', agriculture: 0.6, adapter: 'ros2', notes: '3D boxes for people/vehicles/trees from LiDAR; retrain for orchard classes (trunk, post, bin).' },
  { id: 'pointnet2', name: 'PointNet++ / KPConv (point-cloud segmentation)', family: 'pointcloud', tasks: ['cloud_segment'], inputs: ['pointcloud'], openVocabulary: false, paramsM: 2, latencyMs: L(NA, 120, 50, 20, 8, 6), accuracy: 0.7, runtimes: ['pytorch', 'onnx'], formats: ['.pth', '.onnx'], license: 'MIT', source: 'https://github.com/charlesq34/pointnet2', agriculture: 0.7, adapter: 'http', notes: 'Per-point labels (trunk, branch, leaf, fruit) for pruning and phenotyping.' },
  { id: 'patchwork', name: 'Patchwork++ (ground segmentation)', family: 'classical', tasks: ['cloud_segment'], inputs: ['pointcloud'], openVocabulary: false, paramsM: 0, latencyMs: L(NA, 15, 8, 4, 2, 2), accuracy: 0.85, runtimes: ['ros2', 'pcl'], formats: [], license: 'GPL-3.0', source: 'https://github.com/url-kaist/patchwork-plusplus', agriculture: 0.9, adapter: 'builtin', notes: 'Fast, robust ground removal on slopes and ruts; feeds obstacle clustering.' },
  // --- grasping
  { id: 'graspnet', name: 'GraspNet / Contact-GraspNet / AnyGrasp', family: 'grasp', tasks: ['grasp'], inputs: ['pointcloud', 'depth'], openVocabulary: false, paramsM: 20, latencyMs: L(NA, NA, 300, 120, 40, 30), accuracy: 0.75, runtimes: ['pytorch', 'http'], formats: ['.pth'], license: 'MIT / non-commercial (AnyGrasp)', source: 'https://github.com/graspnet/graspnet-baseline', agriculture: 0.7, adapter: 'http', notes: '6-DoF parallel-gripper grasps from a point cloud; fruit needs suction/soft grippers with approach-only grasps.' },
  { id: 'approach_grasp', name: 'Approach-vector grasp (centroid + normal + approach offset)', family: 'classical', tasks: ['grasp'], inputs: ['image'], openVocabulary: false, paramsM: 0, latencyMs: L(1, 1, 0.5, 0.3, 0.2, 0.2), accuracy: 0.7, runtimes: ['pcl'], formats: [], license: 'built-in', source: 'studio', agriculture: 0.9, adapter: 'builtin', notes: 'Studio built-in: target at the 3D centroid, tool Z along the camera ray or surface normal, approach target offset back — what harvesting arms actually use.' },
  // --- VLM
  { id: 'florence2', name: 'Florence-2 (base/large)', family: 'vlm', tasks: ['vlm_query', 'detect', 'segment', 'classify'], inputs: ['image', 'text'], openVocabulary: true, paramsM: 770, latencyMs: L(NA, 1500, 600, 250, 80, 60), accuracy: 0.6, runtimes: ['pytorch', 'onnx', 'http'], formats: ['.safetensors', '.onnx'], license: 'MIT', source: 'https://huggingface.co/microsoft/Florence-2-large', agriculture: 0.65, adapter: 'http', notes: 'Small VLM with grounding/segmentation/OCR tasks; runs on Orin Nano.' },
  { id: 'paligemma', name: 'PaliGemma 2 (3B)', family: 'vlm', tasks: ['vlm_query', 'detect', 'segment'], inputs: ['image', 'text'], openVocabulary: true, paramsM: 3000, latencyMs: L(NA, NA, 2500, 900, 250, 200), accuracy: 0.65, runtimes: ['pytorch', 'http'], formats: ['.safetensors'], license: 'Gemma licence', source: 'https://huggingface.co/google/paligemma2-3b-pt-448', agriculture: 0.65, adapter: 'vlm_openai', notes: 'Fine-tunable VLM with detection tokens; serve with vLLM.' },
  { id: 'qwen2_vl', name: 'Qwen2.5-VL (3B/7B) via Ollama / vLLM', family: 'vlm', tasks: ['vlm_query', 'detect', 'classify'], inputs: ['image', 'text'], openVocabulary: true, paramsM: 7000, latencyMs: L(NA, NA, 4000, 1500, 400, 300), accuracy: 0.72, runtimes: ['http'], formats: ['.gguf', '.safetensors'], license: 'Apache-2.0', source: 'https://huggingface.co/Qwen/Qwen2.5-VL-7B-Instruct', agriculture: 0.7, adapter: 'vlm_openai', notes: 'Strong grounding (returns box coordinates), counting and reasoning; OpenAI-compatible API — the studio VLM adapter default.' },
  { id: 'llava', name: 'LLaVA-NeXT / Moondream / SmolVLM', family: 'vlm', tasks: ['vlm_query', 'classify'], inputs: ['image', 'text'], openVocabulary: true, paramsM: 2000, latencyMs: L(NA, 3000, 1200, 500, 150, 120), accuracy: 0.55, runtimes: ['http'], formats: ['.gguf'], license: 'Apache-2.0', source: 'https://github.com/vikhyat/moondream', agriculture: 0.6, adapter: 'vlm_openai', notes: 'Light VLMs for scene description, anomaly questions ("is the row blocked?"); weak precise grounding.' },
  { id: 'gpt4o_class', name: 'Hosted VLM (GPT-4o / Claude / Gemini class)', family: 'vlm', tasks: ['vlm_query', 'detect', 'classify'], inputs: ['image', 'text'], openVocabulary: true, paramsM: 0, latencyMs: L(NA, NA, NA, NA, NA, 2500), accuracy: 0.8, runtimes: ['http'], formats: [], license: 'commercial API', source: 'OpenAI-compatible endpoint', agriculture: 0.75, adapter: 'vlm_openai', notes: 'Best reasoning, needs connectivity; use for supervision, reports and mission planning — not control loops.' },
  // --- VLA
  { id: 'openvla', name: 'OpenVLA (7B) / OpenVLA-OFT', family: 'vla', tasks: ['vla_policy'], inputs: ['image', 'text', 'proprio'], openVocabulary: true, paramsM: 7000, latencyMs: L(NA, NA, NA, 400, 120, 100), accuracy: 0.6, runtimes: ['pytorch', 'http'], formats: ['.safetensors'], license: 'MIT', source: 'https://github.com/openvla/openvla', agriculture: 0.5, adapter: 'vla_http', notes: 'Language-conditioned 7-DoF end-effector deltas at 3–10 Hz; fine-tune on teleop demos (LeRobot/RLDS).' },
  { id: 'pi0', name: 'π0 / π0-FAST (Physical Intelligence, openpi)', family: 'vla', tasks: ['vla_policy'], inputs: ['image', 'text', 'proprio'], openVocabulary: true, paramsM: 3300, latencyMs: L(NA, NA, NA, 150, 60, 50), accuracy: 0.7, runtimes: ['pytorch', 'http'], formats: ['.safetensors'], license: 'Apache-2.0', source: 'https://github.com/Physical-Intelligence/openpi', agriculture: 0.55, adapter: 'vla_http', notes: 'Flow-matching action chunks (50 Hz), websocket policy server — the studio VLA adapter speaks its JSON.' },
  { id: 'octo', name: 'Octo (small/base)', family: 'vla', tasks: ['vla_policy'], inputs: ['image', 'text', 'proprio'], openVocabulary: true, paramsM: 93, latencyMs: L(NA, NA, 200, 80, 30, 25), accuracy: 0.5, runtimes: ['pytorch', 'http'], formats: ['.pt'], license: 'MIT', source: 'https://github.com/octo-models/octo', agriculture: 0.45, adapter: 'vla_http', notes: 'Small generalist policy; feasible on Orin NX.' },
  { id: 'groot', name: 'NVIDIA GR00T N1 / Isaac GR00T', family: 'vla', tasks: ['vla_policy'], inputs: ['image', 'text', 'proprio'], openVocabulary: true, paramsM: 2000, latencyMs: L(NA, NA, NA, 100, 40, 35), accuracy: 0.65, runtimes: ['pytorch', 'tensorrt', 'http'], formats: ['.safetensors'], license: 'NVIDIA', source: 'https://github.com/NVIDIA/Isaac-GR00T', agriculture: 0.5, adapter: 'vla_http', notes: 'Humanoid/arm foundation policy; Jetson Thor target.' },
  { id: 'rt_x_act', name: 'ACT / Diffusion Policy (LeRobot, task-specific)', family: 'vla', tasks: ['vla_policy'], inputs: ['image', 'proprio'], openVocabulary: false, paramsM: 80, latencyMs: L(NA, 40, 15, 8, 4, 3), accuracy: 0.75, runtimes: ['pytorch', 'onnx', 'http'], formats: ['.safetensors', '.onnx'], license: 'Apache-2.0', source: 'https://github.com/huggingface/lerobot', agriculture: 0.7, adapter: 'vla_http', notes: 'Not language-conditioned but the most reliable learned policy for one task (e.g. picking) from 50–200 demos; runs on Orin Nano.' },
];

// ---------------------------------------------------------------------------------------------
// Configuration stored on the camera
// ---------------------------------------------------------------------------------------------

export interface VlmEndpoint { url: string; model: string; apiKey?: string; prompt?: string }
export interface VlaEndpoint { url: string; model: string; format?: 'studio' | 'openpi' | 'openvla'; instruction?: string; actionScaleMm?: number; actionScaleDeg?: number }

export interface VisionStackConfig {
  tasks: VisionTask[];
  modality: VisionModality;
  environment: VisionEnvironment;
  sensor: string;
  compute: string;
  /** Model id per task. */
  models: Partial<Record<VisionTask, string>>;
  /** Class names the detector/segmenter/classifier reports (also the open-vocabulary prompt). */
  classes: string[];
  confidence: number;
  iou: number;
  /** Where inference runs. */
  runtime: 'simulated' | 'onnx' | 'server' | 'ros2' | 'vlm';
  /** ONNX model URL/file for the browser runtime (detector); classifier/segmenter/pose variants optional. */
  modelUrl?: string;
  inputSize?: number;
  /** Server runtime: model name/path known to the studio server (`/vision/infer`). */
  serverModel?: string;
  /** ROS 2 runtime: detection topic (vision_msgs/Detection2DArray) through rosbridge. */
  rosTopic?: string;
  vlm?: VlmEndpoint;
  vla?: VlaEndpoint;
  /** Camera pose in the robot flange (eye-in-hand) or robot base (eye-to-hand) frame, 16 values column-major (mm). */
  handEye?: number[];
  handEyeMode?: 'eye_in_hand' | 'eye_to_hand';
  /** Prior object size (mm) for mono position estimation. */
  objectSizeMm?: number;
  /** Working distance (m) used for the error budget. */
  workingDistance: number;
  /** Webhook: POST the JSON summary of every run to this URL (custom nodes without ROS). */
  publishUrl?: string;
  /** Publish results to ROS 2 through the connected rosbridge (Vision tab / RosBridge). */
  publishRos?: boolean;
  /** Simulation: inject false negatives / positives / box noise according to the model accuracy. */
  simulate: boolean;
  seed?: number;
}

export function getVisionStack(cam: CameraItem): VisionStackConfig | null {
  const c = cam.params.visionStack as unknown as VisionStackConfig | undefined;
  return c && c.tasks && c.modality ? c : null;
}
export function setVisionStack(cam: CameraItem, cfg: VisionStackConfig | null): void {
  if (cfg) cam.setParam('visionStack', cfg as any); else delete cam.params.visionStack;
}

export function modalityForCamera(cam: CameraItem): VisionModality {
  switch (cam.kind) {
    case 'depth': return 'rgbd';
    case 'lidar3d': return 'lidar3d';
    case 'lidar2d': return 'lidar3d';
    default: return 'mono';
  }
}
export function cameraKindForModality(m: VisionModality): CameraItem['kind'] {
  return m === 'lidar3d' ? 'lidar3d' : m === 'rgbd' || m === 'stereo' || m === 'tof' ? 'depth' : 'rgb';
}

// ---------------------------------------------------------------------------------------------
// Recommender
// ---------------------------------------------------------------------------------------------

export interface VisionRequest {
  tasks: VisionTask[];
  environment: VisionEnvironment;
  /** Modalities the user already has / is willing to use; empty = any. */
  modalities?: VisionModality[];
  compute?: ComputeClass;
  /** Working distance (m). */
  workingDistance?: number;
  /** 3D positions required (picking, obstacle distances). */
  needs3D?: boolean;
  night?: boolean;
  /** Vehicle in motion (favours global shutter / LiDAR). */
  moving?: boolean;
  openVocabulary?: boolean;
  maxCost?: number;
  /** Required end-to-end rate (Hz). */
  minFps?: number;
  /** Connectivity available (cloud models allowed). */
  online?: boolean;
  /** Target position accuracy (m) for pose tasks. */
  targetAccuracy?: number;
}

export interface VisionRecommendation {
  sensor: VisionSensor;
  compute: ComputeTarget;
  models: Partial<Record<VisionTask, VisionModelSpec>>;
  score: number;
  reasons: string[];
  warnings: string[];
  /** Estimated pipeline latency (ms) on the chosen compute. */
  latencyMs: number;
  /** Depth 1σ error at the working distance (m), null when the sensor gives no depth. */
  depthErrorM: number | null;
  /** Ordered processing steps. */
  pipeline: string[];
}

const OUTDOOR: VisionEnvironment[] = ['orchard', 'vineyard', 'open_field', 'forest', 'urban', 'mixed'];
const AGRI: VisionEnvironment[] = ['orchard', 'vineyard', 'open_field', 'greenhouse', 'forest'];

export function depthSigma(s: VisionSensor, z: number): number | null {
  if (!s.depthError || !s.depthRange) return null;
  return s.depthError.a + s.depthError.b * z * z;
}

function taskInputsSatisfied(m: VisionModelSpec, s: VisionSensor): boolean {
  const hasDepth = !!s.depthRange && s.modality !== 'lidar3d';
  const hasCloud = s.modality === 'lidar3d' || hasDepth;
  const hasImage = s.modality !== 'lidar3d' || s.id === 'ouster_os1';
  for (const i of m.inputs) {
    if (i === 'image' && !hasImage) return false;
    if (i === 'depth' && !hasDepth) return false;
    if (i === 'pointcloud' && !hasCloud) return false;
  }
  return true;
}

/** Pick the best model for a task given the sensor, compute and request. */
export function bestModelFor(task: VisionTask, sensor: VisionSensor, compute: ComputeTarget, req: VisionRequest): { model: VisionModelSpec; score: number; why: string } | null {
  let best: { model: VisionModelSpec; score: number; why: string } | null = null;
  for (const m of VISION_MODELS) {
    if (!m.tasks.includes(task)) continue;
    if (!taskInputsSatisfied(m, sensor)) continue;
    const lat = m.latencyMs[compute.class];
    if (lat === undefined || lat >= NA) continue;
    if (compute.class !== 'cloud' && m.runtimes.length && !m.runtimes.some((r) => compute.runtimes.includes(r) || r === 'http' || r === 'ros2' || r === 'pcl' || r === 'open3d')) continue;
    if (!req.online && m.id === 'gpt4o_class') continue;
    let score = m.accuracy * 50;
    const agri = AGRI.includes(req.environment);
    score += (agri ? m.agriculture : 0.7) * 20;
    const fpsNeed = req.minFps ?? (task === 'vlm_query' || task === 'vla_policy' ? 0.5 : 10);
    const budget = 1000 / fpsNeed;
    score += lat <= budget * 0.5 ? 15 : lat <= budget ? 8 : -20;
    if (req.openVocabulary) score += m.openVocabulary ? 15 : -10; else if (m.openVocabulary && task !== 'vlm_query' && task !== 'vla_policy') score -= 3;
    if (task === 'pose' && req.needs3D === false && m.inputs.includes('depth')) score -= 5;
    if (task === 'pose' && sensor.depthRange && m.id === 'size_prior_position') score -= 15;
    if (task === 'pose' && (req.targetAccuracy ?? 0.02) < 0.01 && m.family === 'classical') score -= 10;
    if (task === 'grasp' && agri && m.id === 'approach_grasp') score += 8;
    if ((task === 'track' || task === 'follow') && m.id === 'bytetrack') score += 5;
    if (task === 'segment' && m.family === 'sam' && agri) score -= 4; // class-specific masks preferred for picking
    if (task === 'depth' && sensor.modality === 'stereo' && m.id === 'depth_anything') score -= 15; // stereo already gives depth
    if (task === 'depth' && sensor.modality === 'mono' && (m.id === 'raft_stereo' || m.id === 'sgbm')) continue;
    if (task === 'cloud_segment' && agri && m.id === 'patchwork') score += 3;
    if (task === 'detect' && m.family === 'vlm') score -= 12; // VLM detection is slow
    if (task === 'detect' && m.tasks[0] !== 'detect') score -= 8; // prefer plain detectors over -seg / -pose heads for detection alone
    if (task === 'vlm_query' && m.family !== 'vlm') score -= 15; // CLIP / grounding models cannot answer questions
    if (task === 'vla_policy' && !req.openVocabulary && m.id === 'rt_x_act') score += 6;
    if (task === 'vla_policy' && req.openVocabulary && m.id === 'pi0') score += 4;
    const why = `${m.name}: ${Math.round(lat)} ms on ${compute.name}, quality ${(m.accuracy * 100).toFixed(0)}`;
    if (!best || score > best.score) best = { model: m, score, why };
  }
  return best;
}

export function recommendVisionStacks(req: VisionRequest): VisionRecommendation[] {
  const out: VisionRecommendation[] = [];
  const z = req.workingDistance ?? 1.5;
  const outdoor = OUTDOOR.includes(req.environment);
  const agri = AGRI.includes(req.environment);
  const needs3D = req.needs3D ?? req.tasks.some((t) => ['pose', 'grasp', 'cloud_detect', 'cloud_segment', 'measure', 'depth'].includes(t));
  const cloudTask = req.tasks.some((t) => t === 'cloud_detect' || t === 'cloud_segment');
  const imageTask = req.tasks.some((t) => !['cloud_detect', 'cloud_segment'].includes(t));
  const computes = req.compute ? COMPUTE_TARGETS.filter((c) => c.class === req.compute) : COMPUTE_TARGETS.filter((c) => c.class !== 'cloud' || req.online);
  for (const s of VISION_SENSORS) {
    if (req.modalities?.length && !req.modalities.includes(s.modality)) continue;
    if (req.maxCost !== undefined && s.cost > req.maxCost) continue;
    const reasons: string[] = [], warnings: string[] = [];
    let score = 50;
    // depth / 3D
    const sig = depthSigma(s, z);
    if (needs3D) {
      if (sig === null) { if (s.modality === 'mono') { score -= 25; warnings.push('no depth: 3D positions only from a size prior or a monocular depth network (±10–20 %)'); } else { score -= 30; warnings.push('sensor gives no 3D data'); } }
      else {
        const acc = req.targetAccuracy ?? (agri ? 0.02 : 0.005);
        if (sig <= acc) { score += 20; reasons.push(`depth 1σ ${(sig * 1000).toFixed(1)} mm at ${z} m meets ${(acc * 1000).toFixed(0)} mm`); }
        else if (sig <= acc * 3) { score += 8; warnings.push(`depth 1σ ${(sig * 1000).toFixed(1)} mm at ${z} m, target ${(acc * 1000).toFixed(0)} mm — average several frames or move closer`); }
        else { score -= 10; warnings.push(`depth error ${(sig * 1000).toFixed(0)} mm at ${z} m is too large for ${(acc * 1000).toFixed(0)} mm`); }
        if (s.depthRange && (z < s.depthRange[0] || z > s.depthRange[1])) { score -= 25; warnings.push(`working distance ${z} m outside depth range ${s.depthRange[0]}–${s.depthRange[1]} m`); }
      }
    } else if (sig !== null && s.modality !== 'lidar3d') { score -= 3; }
    if (cloudTask && s.modality !== 'lidar3d' && !s.depthRange) { score -= 40; warnings.push('point-cloud tasks need a LiDAR or a depth camera'); }
    if (cloudTask && s.modality === 'lidar3d') { score += 15; reasons.push('native point clouds, 360° coverage'); }
    if (imageTask && s.modality === 'lidar3d') { if (s.id === 'ouster_os1') { score -= 5; warnings.push('image tasks run on the LiDAR reflectivity image — lower quality than RGB'); } else { score -= 35; warnings.push('image tasks need a camera in addition to the LiDAR'); } }
    if (imageTask && s.modality === 'thermal') { score -= 15; warnings.push('thermal images: people/animals only, colour-based classes (ripeness) impossible'); }
    if (imageTask && s.modality === 'multispectral') { score -= 25; warnings.push('multispectral: mapping/scouting, not real-time detection'); }
    if (s.modality === 'event') { score -= 30; warnings.push('event cameras have few ready-made models'); }
    // environment
    if (outdoor) { score += (s.outdoor - 0.5) * 40; if (s.outdoor < 0.6) warnings.push('depth degrades in direct sunlight — shade the scene or work at dawn/dusk'); else if (s.outdoor >= 0.8) reasons.push('sunlight-robust'); }
    else if (s.modality === 'rgbd' || s.modality === 'tof') { score += 8; reasons.push('active depth reliable indoors'); }
    if (req.night) { if (s.night) { score += 10; reasons.push('works in darkness'); } else { score -= 20; warnings.push('no depth/detections in darkness — add LED lighting (typical for night harvesting)'); } }
    if (req.moving) { if (s.globalShutter) { score += 6; reasons.push('global shutter'); } else { score -= 10; warnings.push('rolling shutter on a moving vehicle: motion blur and skew'); } }
    if (agri && s.id === 'zivid') { score -= 20; warnings.push('metrology camera: indoor static scenes only'); }
    if (agri && (s.id === 'zed_x' || s.id === 'oak_d_pro')) score += 6;
    score -= (s.cost - 2) * 4;
    // best compute + models
    let bestPick: VisionRecommendation | null = null;
    for (const c of computes) {
      if (req.maxCost !== undefined && c.cost > req.maxCost + 1) continue;
      const models: Partial<Record<VisionTask, VisionModelSpec>> = {};
      let mscore = 0, lat = 0;
      const mwarn: string[] = [], mreasons: string[] = [];
      let ok = true;
      for (const t of req.tasks) {
        const b = bestModelFor(t, s, c, req);
        if (!b) { ok = false; mwarn.push(`no model runs ${TASK_LABELS[t]} on ${c.name} with this sensor`); break; }
        models[t] = b.model; mscore += b.score; lat += b.model.latencyMs[c.class] ?? 0; mreasons.push(b.why);
      }
      if (!ok) continue;
      // tracking / follow reuse the detector latency; VLM/VLA latency counted once
      const fpsNeed = req.minFps ?? 10;
      let cs = mscore / Math.max(1, req.tasks.length) + (lat <= 1000 / fpsNeed ? 10 : -15) - (c.cost - 2) * 3 - (c.powerW > 60 && (agri || req.moving) ? 5 : 0);
      if (c.class === 'cloud') { cs -= 10; mwarn.push('cloud inference: not usable for closed-loop control without connectivity'); }
      if (c.id === 'oak_myriad' && s.id !== 'oak_d_pro') continue;
      if (c.id === 'oak_myriad') { cs += 5; mreasons.push('detector runs inside the camera'); }
      const rec: VisionRecommendation = { sensor: s, compute: c, models, score: 0, reasons: [...reasons, ...mreasons.slice(0, 2), ...(c.notes ? [] : [])], warnings: [...warnings, ...mwarn], latencyMs: lat, depthErrorM: sig, pipeline: [] };
      rec.score = Math.round(score + cs);
      if (!bestPick || rec.score > bestPick.score) bestPick = rec;
    }
    if (!bestPick) continue;
    bestPick.pipeline = pipelineSteps(req.tasks, bestPick);
    out.push(bestPick);
  }
  out.sort((a, b) => b.score - a.score);
  return out;
}

function pipelineSteps(tasks: VisionTask[], rec: VisionRecommendation): string[] {
  const steps: string[] = [`${rec.sensor.rosDriver} → ${rec.sensor.modality === 'lidar3d' ? 'sensor_msgs/PointCloud2' : 'sensor_msgs/Image' + (rec.sensor.depthRange ? ' + depth/CameraInfo' : '')}`];
  const m = rec.models;
  if (m.detect) steps.push(`${m.detect.name} → vision_msgs/Detection2DArray`);
  if (m.segment) steps.push(`${m.segment.name} → masks`);
  if (m.keypoints) steps.push(`${m.keypoints.name} → keypoints`);
  if (m.classify) steps.push(`${m.classify.name} on crops → class + score`);
  if (m.track || m.follow) steps.push(`${(m.track ?? m.follow)!.name} → track IDs, velocities`);
  if (m.depth) steps.push(`${m.depth.name} → depth image`);
  if (m.cloud_segment) steps.push(`${m.cloud_segment.name} → ground / obstacle points`);
  if (m.cloud_detect) steps.push(`${m.cloud_detect.name} → vision_msgs/Detection3DArray`);
  if (m.pose) steps.push(`${m.pose.name} → geometry_msgs/PoseArray in camera frame → tf (hand-eye) → robot base`);
  if (m.grasp) steps.push(`${m.grasp.name} → grasp targets (approach + grasp)`);
  if (m.measure) steps.push(`${m.measure.name} → size / distance / volume`);
  if (m.vlm_query) steps.push(`${m.vlm_query.name} (OpenAI-compatible chat/completions with the image) → JSON answer / boxes`);
  if (m.vla_policy) steps.push(`${m.vla_policy.name} (image + instruction + proprio) → action chunk → robot TCP deltas`);
  if (tasks.includes('follow')) steps.push('follow controller: keep the tracked target centred at the desired range (mobile base) or in the image centre (arm servoing)');
  return steps;
}

/** Default configuration for a camera from the top recommendation. */
export function configFromRecommendation(rec: VisionRecommendation, req: VisionRequest, classes: string[]): VisionStackConfig {
  const models: Partial<Record<VisionTask, string>> = {};
  for (const [t, m] of Object.entries(rec.models)) if (m) models[t as VisionTask] = m.id;
  const vlmSpec = rec.models.vlm_query, vlaSpec = rec.models.vla_policy;
  return {
    tasks: [...req.tasks], modality: rec.sensor.modality, environment: req.environment, sensor: rec.sensor.id, compute: rec.compute.id, models, classes: classes.length ? classes : ['object'],
    confidence: 0.4, iou: 0.5, runtime: 'simulated', inputSize: 640, workingDistance: req.workingDistance ?? 1.5, simulate: true,
    vlm: vlmSpec ? { url: 'http://localhost:11434/v1', model: vlmSpec.id === 'qwen2_vl' ? 'qwen2.5vl:7b' : vlmSpec.id === 'llava' ? 'llava' : vlmSpec.id, prompt: '' } : undefined,
    vla: vlaSpec ? { url: 'http://localhost:8000', model: vlaSpec.id, format: vlaSpec.id === 'pi0' ? 'openpi' : vlaSpec.id === 'openvla' ? 'openvla' : 'studio', instruction: 'pick the ripe apple', actionScaleMm: 20, actionScaleDeg: 5 } : undefined,
    handEyeMode: 'eye_in_hand', objectSizeMm: 75,
  };
}

/** Error budget of a configured stack: depth error at working distance, pixel → mm, expected end-to-end latency. */
export function errorBudget(cfg: VisionStackConfig): { depthSigmaMm: number | null; pixelMm: number; latencyMs: number; notes: string[] } {
  const s = VISION_SENSORS.find((x) => x.id === cfg.sensor), c = COMPUTE_TARGETS.find((x) => x.id === cfg.compute);
  const notes: string[] = [];
  const z = cfg.workingDistance;
  const sig = s ? depthSigma(s, z) : null;
  const fx = s ? (s.resolution[0] / 2) / Math.tan((s.hfov * Math.PI) / 360) : 600;
  const pixelMm = (z * 1000) / fx;
  let lat = 0;
  for (const id of Object.values(cfg.models)) { const m = VISION_MODELS.find((x) => x.id === id); if (m && c) lat += m.latencyMs[c.class] ?? 0; }
  if (cfg.runtime === 'vlm' || cfg.vlm) notes.push('VLM answers take seconds — use them for decisions, not servoing');
  if (sig !== null && sig * 1000 > 3 * pixelMm) notes.push('depth noise dominates over pixel resolution: average 3–5 frames or use the mask median');
  if (cfg.modality === 'mono' && cfg.tasks.includes('pose')) notes.push(`mono position uses the object size prior ${cfg.objectSizeMm ?? 75} mm: ±(size spread) range error`);
  return { depthSigmaMm: sig === null ? null : sig * 1000, pixelMm, latencyMs: lat, notes };
}
