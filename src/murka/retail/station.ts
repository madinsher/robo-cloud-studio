/**
 * MURKA retail store as a studio station (contract C1, buildMurkaRetailStation): the Webots world retail_store.wbt with
 * its PROTOs (T6 importer: shelves expanded from the StoreShelf template, pallets with split colliders, packing table,
 * charging column, MUROM decomposed into a differential MobileRobot with two arm Robots and a head Robot carrying the
 * head camera + range finder, lidar), the products and slots of murom_world/config (products.yaml + locations.yaml,
 * spawned like the supervisor plugin), the dock / packing zones, an occupancy map rasterised from the colliders in the
 * MUROM height band (0.06–0.62 m, 5 cm cells; lower steps are driven over) and the MURKA joint limits / start pose / drive limits.
 *
 * Variants: 'v2' (robot-retailer-murka-v2, "capsule" robot, 6-DOF arms with wrists), 'v1' (robot-retailer-murka,
 * classic STL robot, 4-DOF arms), 'murka' / 'murka_classic' (simulation_ws/src/MURKA_Simulation renamed copies).
 */
import { Station, Item, Camera, ItemType, Tool } from '../../core/items/item';
import { Robot } from '../../core/items/robot';
import { MobileRobot, MapItem } from '../../mobile/items';
import { transl } from '../../core/math/pose';
import type { AssetStore } from '../../scene/assets';
import { importWebotsWorld, mapSource, rasterizeOccupancy, loadRetailLayout } from '../../io/webots';
import type { WebotsSource } from '../../io/webots';
import { parseRetailLayout, RetailLayout } from './layout';
import { ControlModelItem } from '../../ctl/model';
import { missionFsmDsl, parseMission } from './orchestrator';

export type RetailVariant = 'v2' | 'v1' | 'murka' | 'murka_classic';
export interface VariantInfo {
  /** Folder (relative to the MURKA_Simulation root) holding the ROS 2 packages. */
  base: string;
  /** World folder (relative to base): holds worlds/ and protos/. */
  webots: string;
  world: string;
  products: string;
  locations: string;
  /** Reference mission time of the MURKA authors (Webots, `full`), s; null when not published. */
  referenceTime: number | null;
  referenceRtf: number | null;
  /** MuJoCo reference time (s), when published. */
  mujocoTime: number | null;
}
export const RETAIL_VARIANTS: Record<RetailVariant, VariantInfo> = {
  v2: { base: 'robot-retailer-murka-v2/ros2_ws/src', webots: 'murom_webots', world: 'worlds/retail_store.wbt', products: 'murom_world/config/products.yaml', locations: 'murom_world/config/locations.yaml', referenceTime: 1069, referenceRtf: 0.84, mujocoTime: 990 },
  v1: { base: 'robot-retailer-murka/ros2_ws/src', webots: 'murom_webots', world: 'worlds/retail_store.wbt', products: 'murom_world/config/products.yaml', locations: 'murom_world/config/locations.yaml', referenceTime: 706, referenceRtf: 0.73, mujocoTime: 499 },
  murka: { base: 'simulation_ws/src/MURKA_Simulation/ros2_ws/src', webots: 'murka_simulation', world: 'worlds/retail_store.wbt', products: 'murka_simulation/config/products.yaml', locations: 'murka_simulation/config/locations.yaml', referenceTime: null, referenceRtf: null, mujocoTime: null },
  murka_classic: { base: 'simulation_ws/src/MURKA_Simulation/ros2_ws/src', webots: 'murka_simulation', world: 'worlds/retail_store_classic.wbt', products: 'murka_simulation/config/products.yaml', locations: 'murka_simulation/config/locations.yaml', referenceTime: null, referenceRtf: null, mujocoTime: null },
};

/** Default MURKA_Simulation folder of the owner's machine (Node only; overridable with MURKA_DIR). */
export const DEFAULT_MURKA_ROOT = 'C:/Users/TRITON 300 SE/YandexDisk/Работа/КБНЦ РАН/MURKA_Simulation';

// MURKA joint limits (rad): arm_kinematics.JOINT_LIMITS / WRIST_LIMITS, orchestrator HEAD_PAN_LIMIT / HEAD_TILT_LIMITS
export const HEAD_PAN_LIMIT = 1.6;
export const HEAD_TILT_LIMITS: [number, number] = [-0.9, 0.6];
/** Travel pose of the v2 arms (driver START_POSE, pick server HOME_V2, orchestrator TRAVEL_ARM). */
export const TRAVEL_ARM = [0.3, 0, 0, -2.2];
export const HOME_CLASSIC = [0, 0, 0, -1.8];
/** Nav2 velocity smoother of murom_navigation (max_velocity [0.4, 0, 0.8], max_accel [0.6, 0, 1.5]). */
/** Obstacles lower than this (m) are driven over (wheel radius 0.177 m). */
export const TRAVERSABLE_STEP = 0.06;
export const NAV_LIMITS = { v: 0.4, w: 0.8, a: 0.6, aw: 1.5 };

export interface MuromHandles {
  base: MobileRobot;
  left: Robot; right: Robot; head: Robot;
  camera: Camera | null; depth: Camera | null; lidar: Camera | null;
  /** v2 capsule (wrist joints present). */
  hasWrist: boolean;
  /** joint name -> [robot, index] for every arm / head joint (Webots motor names). */
  joints: Map<string, [Robot, number]>;
}

export interface RetailStationOptions {
  variant?: RetailVariant;
  /** MURKA_Simulation root folder (Node: read from disk). */
  root?: string;
  /** Files of a dropped folder (path -> bytes); the world and the two YAML files are found by their path suffix. */
  files?: Map<string, Uint8Array>;
  /** A Webots source rooted at the world folder (holds worlds/ and protos/) plus the YAML texts. */
  source?: WebotsSource; productsYaml?: string; locationsYaml?: string;
  assets?: AssetStore;
  /** Load the robot / product meshes (default true; headless tests switch it off for speed). */
  loadMeshes?: boolean;
}

export interface RetailStationResult {
  station: Station;
  robot: MuromHandles;
  layout: RetailLayout;
  variant: RetailVariant;
  info: VariantInfo;
  map: MapItem;
  report: string[];
  /** Mesh assets of the import (collision geometry of the hand, rendering of the robot for the head camera). */
  assets?: AssetStore | null;
}

const deg = (r: number) => (r * 180) / Math.PI;

/** A lazy directory source (Node only). */
async function nodeDirSource(root: string): Promise<WebotsSource & { readText(p: string): string | undefined }> {
  const fsName = 'node:fs', pathName = 'node:path';
  const fs = await import(/* @vite-ignore */ fsName) as typeof import('node:fs');
  const path = await import(/* @vite-ignore */ pathName) as typeof import('node:path');
  let listing: string[] | null = null;
  const walk = (d: string, out: string[]) => {
    let ents: import('node:fs').Dirent[];
    try { ents = fs.readdirSync(d, { withFileTypes: true }); } catch { return; }
    for (const e of ents) {
      if (e.isDirectory()) { if (!['.git', 'build', 'install', 'log', '__pycache__', 'node_modules'].includes(e.name)) walk(path.join(d, e.name), out); }
      else out.push(path.relative(root, path.join(d, e.name)).split(path.sep).join('/'));
    }
  };
  const read = (p: string) => { try { const b = fs.readFileSync(path.resolve(root, p)); return new Uint8Array(b.buffer, b.byteOffset, b.byteLength); } catch { return undefined; } };
  return { read, list: () => { if (!listing) { listing = []; walk(root, listing); } return listing; }, readText: (p) => { const b = read(p); return b ? new TextDecoder().decode(b) : undefined; } };
}

/** Find the MUROM / Murka robot items of an imported station. */
export function muromHandles(station: Station): MuromHandles {
  const base = station.itemsOfType<MobileRobot>(ItemType.MOBILE_ROBOT).find((m) => /^(murom|murka)/i.test(m.name)) ?? station.itemsOfType<MobileRobot>(ItemType.MOBILE_ROBOT)[0];
  if (!base) throw new Error('no MUROM mobile robot in the station');
  const robots = base.childrenOfType<Robot>(ItemType.ROBOT);
  const byPrefix = (p: string) => robots.find((r) => r.jointNames()[0]?.startsWith(p));
  const left = byPrefix('l_shoulder'), right = byPrefix('r_shoulder'), head = byPrefix('head_');
  if (!left || !right || !head) throw new Error(`MUROM arms / head not found (robots: ${robots.map((r) => r.jointNames().join(',')).join(' | ')})`);
  const cams = [...head.walk()].filter((c): c is Camera => c instanceof Camera);
  const joints = new Map<string, [Robot, number]>();
  for (const r of [left, right, head]) r.jointNames().forEach((n, i) => joints.set(n, [r, i]));
  return {
    base, left, right, head,
    camera: cams.find((c) => c.name === 'head_camera') ?? cams.find((c) => c.kind === 'rgb') ?? null,
    depth: cams.find((c) => c.name === 'head_depth') ?? cams.find((c) => c.kind === 'depth') ?? null,
    lidar: (base.children.find((c) => c instanceof Camera && (c as Camera).kind === 'lidar2d') as Camera | undefined) ?? null,
    hasWrist: joints.has('l_wrist_roll_joint'),
    joints,
  };
}

/** Apply the MURKA joint limits, start pose and Nav2 drive limits to the imported robot. */
export function configureMurom(h: MuromHandles): void {
  const armLim: Array<[number, number]> = [[-2.6, 2.6], [-2.1, 2.1], [-1.6, 1.6], [-2.4, 2.4], [-2.6, 2.6], [-1.55, 1.55]];
  for (const arm of [h.left, h.right]) {
    const n = arm.dof;
    arm.setJointLimits(armLim.slice(0, n).map((l) => deg(l[0])), armLim.slice(0, n).map((l) => deg(l[1])));
    const q0 = h.hasWrist ? [...TRAVEL_ARM, 0, 0] : [0, 0, 0, 0];
    arm.setJointsHome(q0.slice(0, n).map(deg));
    arm.setJoints(q0.slice(0, n).map(deg));
  }
  // TCP = the MURKA grasp point: v2 the palm surface centre (palm frame (0, ∓0.016, -0.054)), classic the forearm tip
  for (const [arm, m] of [[h.left, 1], [h.right, -1]] as Array<[Robot, number]>) {
    if (arm.tools().length) continue;
    const t = new Tool(`${arm.jointNames()[0][0]}_grip`); t.toolKind = 'gripper';
    arm.addChild(t); t.setPose(h.hasWrist ? transl(0, -m * 16, -54) : transl(0, 0, -248)); arm.setTool(t);
  }
  h.head.setJointLimits([deg(-HEAD_PAN_LIMIT), deg(HEAD_TILT_LIMITS[0])], [deg(HEAD_PAN_LIMIT), deg(HEAD_TILT_LIMITS[1])]);
  h.head.setJoints([0, 0]);
  const k = h.base.kin;
  h.base.params.motorMaxSpeed = k.maxSpeed;
  k.maxSpeed = NAV_LIMITS.v * 1000; k.maxReverseSpeed = NAV_LIMITS.v * 1000; k.maxAccel = NAV_LIMITS.a * 1000; k.maxYawRate = deg(NAV_LIMITS.w);
}

/** C1: build the MURKA retail station. */
export async function buildMurkaRetailStation(opts: RetailStationOptions = {}): Promise<RetailStationResult> {
  const variant = opts.variant ?? 'v2';
  const info = RETAIL_VARIANTS[variant];
  const report: string[] = [];
  let src: WebotsSource; let worldPath: string; let productsYaml = opts.productsYaml, locationsYaml = opts.locationsYaml;
  if (opts.source) {
    src = opts.source; worldPath = info.world;
  } else if (opts.files) {
    const all = mapSource(opts.files);
    const keys = all.list();
    // a picked MURKA_Simulation folder holds several retail worlds: the variant's own project path wins
    const pick = (rel: string) => keys.find((k) => k.endsWith(`${info.base}/${rel}`)) ?? keys.find((k) => k.endsWith(rel));
    const world = pick(`${info.webots}/${info.world}`) ?? keys.find((k) => k.endsWith(info.world));
    if (!world) throw new Error(`${info.world} not found in the files`);
    const prefix = world.slice(0, world.length - info.world.length);
    src = { read: (p) => all.read(prefix + p), list: () => keys.filter((k) => k.startsWith(prefix)).map((k) => k.slice(prefix.length)) };
    worldPath = info.world;
    const dec = (k: string | undefined) => (k ? new TextDecoder().decode(all.read(k)!) : undefined);
    productsYaml ??= dec(pick(info.products) ?? keys.find((k) => /(^|\/)products\.ya?ml$/.test(k)));
    locationsYaml ??= dec(pick(info.locations) ?? keys.find((k) => /(^|\/)locations\.ya?ml$/.test(k)));
  } else {
    const root = opts.root ?? (typeof process !== 'undefined' ? process.env?.MURKA_DIR : undefined) ?? DEFAULT_MURKA_ROOT;
    const base = `${root}/${info.base}`;
    const ws = await nodeDirSource(`${base}/${info.webots}`);
    const cfg = await nodeDirSource(base);
    src = ws; worldPath = info.world;
    productsYaml ??= cfg.readText(info.products);
    locationsYaml ??= cfg.readText(info.locations);
  }
  if (!productsYaml || !locationsYaml) throw new Error('products.yaml / locations.yaml not found');
  const r = importWebotsWorld(worldPath, src, { assets: opts.assets, loadMeshes: opts.loadMeshes ?? true });
  report.push(...r.report.filter((l) => /error|not found|failed/i.test(l)));
  const station = r.station;
  station.name = `MURKA retail (${variant})`;
  const lr = loadRetailLayout(station, productsYaml, locationsYaml, { builder: r.builder, worldPath });
  report.push(...lr.report);
  const layout = parseRetailLayout(productsYaml, locationsYaml);
  const robot = muromHandles(station);
  configureMurom(robot);
  // band from 6 cm (steps below a third of the 0.177 m wheel radius are driven over in Webots: the 5 cm charging pad of
  // the MURKA_Simulation worlds) to 0.62 m (lidar / torso height of MUROM)
  const map = rasterizeOccupancy(station, { zMin: TRAVERSABLE_STEP * 1000, zMax: 620, resolution: 50, assets: opts.assets, name: 'occupancy (0.06–0.62 m)' });
  map.inflation = 0;
  station.addChild(map);
  station.params.murkaRetail = variant;
  report.push(`MURKA retail ${variant}: ${lr.products.length} products, ${Object.keys(layout.slots).length} slots, robot ${robot.base.name} (${robot.hasWrist ? '6' : '4'}-DOF arms), map ${map.width}x${map.height} @ ${map.resolution} mm`);
  // the mission FSM as a control model of the station (Control tab: view, diagram, exports, Run on station)
  const cm = new ControlModelItem('MUROM mission (full)');
  cm.kind = 'fsm'; cm.source = missionFsmDsl(parseMission('full', [], layout)); cm.robotIds = [robot.base.id];
  station.addChild(cm);
  const result: RetailStationResult = { station, robot, layout, variant, info, map, report, assets: opts.assets ?? null };
  built.set(station, result);
  return result;
}

const built = new WeakMap<Station, RetailStationResult>();
/** The build result of a station made by buildMurkaRetailStation (null for other stations). */
export function retailStationOf(station: Station): RetailStationResult | null { return built.get(station) ?? null; }

/** The item a product name refers to. */
export function productItem(station: Station, name: string): Item | null { return station.find(name); }
