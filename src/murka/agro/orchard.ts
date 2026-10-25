/**
 * MURKA agromarca orchards as studio stations (contract C2: buildMurkaOrchardStation, runMissionB).
 *
 * Worlds of `murca_garden_ws/src/murca_webots/worlds` imported with the studio's Webots importer (T6):
 *  - `orchard_demo.wbt` (Mission B showcase: 25 apple trees on a 3 m grid in 4 sectors, base station, 2-slot equipment
 *    depot, crate station, collection points, UTP-G-01..04 Ackermann platforms, UTP-A-01 aerial platform);
 *  - the legacy `murca_apple_garden.wbt` (26 trees, sphere crowns, 5 apples each) and `murca_garden.wbt` (7 sphere trees,
 *    no fruit) — static scenes in Webots (robots are never spawned there).
 * The trees become FieldItems / CropRows (one field per sector) whose crop parameters are measured on the tree geometry
 * (trunk radius from the boundingObject, crown base / radius from the mesh or primitive profile); trees without fruit
 * get seeded fruit inside their crown, so the studio's harvest missions (FleetManager, agri process models) run there.
 * Robots get their Mission B roles and modular equipment (agri/equipment.ts), the equipment depot becomes a depot zone
 * (2 slots, 180 s reconfiguration), UTP-A-01 an aerial MobileRobot (5 m/s), collection points unloading zones.
 */
import { Station, Item, ItemType, SceneObject, GeometryRef } from '../../core/items/item';
import { fromArray, identity, transformPoint } from '../../core/math/pose';
import { MobileRobot, ZoneItem } from '../../mobile/items';
import { FieldItem, CropRow } from '../../agri/items';
import { applyEquipment, RECIPES } from '../../agri/equipment';
import { AssetStore } from '../../scene/assets';
import { createRobotFromLibrary } from '../../core/items/library';
import { Tool } from '../../core/items/item';
import { mul, transl, rotz, DEG } from '../../core/math/pose';
import { FleetItem, FleetManager } from '../../fleet/fleet';
import { MissionItem } from '../../agri/items';
import { planMission } from '../../agri/missions';
import { importWebotsWorld, mapSource, upgradeOrchard, playScriptedFleet, defaultMissionBScript, MISSION_B, StationFleetWorld } from '../../io/webots';
import type { WebotsSource, FleetScript, ScriptedFleet, FleetWorld } from '../../io/webots';
import { PyRandom } from './pycompat';

export type OrchardWorld = 'orchard_demo' | 'murca_apple_garden' | 'murca_garden';
/** Folder of the agro workspace relative to the MURKA_Simulation root, and the worlds relative to it. */
export const AGRO_WS_REL = 'simulation_ws/src/agromarca/murca_garden_ws';
export const ORCHARD_WORLDS: Record<OrchardWorld, string> = {
  orchard_demo: 'src/murca_webots/worlds/orchard_demo.wbt',
  murca_apple_garden: 'src/murca_webots/worlds/murca_apple_garden.wbt',
  murca_garden: 'src/murca_webots/worlds/murca_garden.wbt',
};
/** Default MURKA_Simulation folder of the owner's machine (Node only; overridable with MURKA_DIR). */
export const DEFAULT_MURKA_ROOT = 'C:/Users/TRITON 300 SE/YandexDisk/Работа/КБНЦ РАН/MURKA_Simulation';

/** Mission B roles of the orchard_demo platforms (garden_supervisor) and their equipment recipe. */
export const MISSION_B_ROLES: Record<string, { role: string; recipe: string | null; caps: string[] }> = {
  'UTP-G-01': { role: 'harvester', recipe: 'harvester', caps: ['navigation', 'perception', 'manipulation'] },
  'UTP-G-02': { role: 'scout', recipe: 'scout', caps: ['navigation', 'perception'] },
  'UTP-G-03': { role: 'sprayer', recipe: 'sprayer', caps: ['navigation'] },
  'UTP-G-04': { role: 'logistics', recipe: 'transporter', caps: ['navigation'] },
  'UTP-A-01': { role: 'aerial scout', recipe: 'uav_scout', caps: ['navigation', 'perception', 'aerial'] },
};

export interface OrchardStationOptions {
  world?: OrchardWorld;
  /** MURKA_Simulation root (Node): the world is read from <root>/<AGRO_WS_REL>. */
  root?: string;
  /** Dropped files (paths → bytes) holding the world, its PROTOs and meshes (browser). */
  files?: Map<string, Uint8Array>;
  /** Any Webots source rooted at the agro workspace (murca_garden_ws). */
  source?: WebotsSource;
  assets?: AssetStore;
  /** Seed and count of the generated fruit for trees without fruit geometry (default 0 / 40 per tree). */
  fruitSeed?: number;
  fruitPerTree?: number;
}

export interface TreeProfile { trunkRadius: number; crownBase: number; crownTop: number; crownRadius: number; bands: Array<{ z: number; r: number }> }

export interface OrchardStationResult {
  station: Station;
  world: OrchardWorld;
  worldPath: string;
  report: string[];
  fields: FieldItem[];
  /** Mobile robots by name (orchard_demo: UTP-G-01..04, UTP-A-01). */
  robots: Record<string, MobileRobot>;
  /** Trees (SceneObjects) and their measured profiles. */
  trees: SceneObject[];
  profile: TreeProfile | null;
  assets: AssetStore;
}

async function nodeDirSource(root: string): Promise<WebotsSource> {
  const fsName = 'node:fs', pathName = 'node:path';
  const fs = await import(/* @vite-ignore */ fsName) as typeof import('node:fs');
  const path = await import(/* @vite-ignore */ pathName) as typeof import('node:path');
  let listing: string[] | null = null;
  const SKIP = new Set(['.git', 'build', 'install', 'log', 'node_modules', '__pycache__', 'stills_out', 'meshes_trellis']);
  const walk = (d: string, out: string[]) => {
    let ents: import('node:fs').Dirent[];
    try { ents = fs.readdirSync(d, { withFileTypes: true }); } catch { return; }
    for (const e of ents) {
      if (e.isDirectory()) { if (!SKIP.has(e.name)) walk(path.join(d, e.name), out); }
      else out.push(path.relative(root, path.join(d, e.name)).split(path.sep).join('/'));
    }
  };
  return {
    read: (p: string) => { try { const b = fs.readFileSync(path.resolve(root, p)); return new Uint8Array(b.buffer, b.byteOffset, b.byteLength); } catch { return undefined; } },
    list: () => { if (!listing) { listing = []; walk(root, listing); } return listing; },
  };
}

/** Prefix of the dropped files under which `worlds/<file>` of the agro workspace lies (so PROTO paths resolve). */
function rootedFiles(files: Map<string, Uint8Array>, world: OrchardWorld): { src: WebotsSource; worldPath: string } {
  const src = mapSource(files);
  const file = ORCHARD_WORLDS[world].split('/').pop()!;
  const hit = src.list().find((p) => p.endsWith(`/worlds/${file}`) || p === `worlds/${file}`);
  if (!hit) throw new Error(`${file} is not among the dropped files (drop the murca_garden_ws or murca_webots folder)`);
  return { src, worldPath: hit };
}

/**
 * Radial profile of a tree about its origin (the trunk axis): max radius per 100 mm band of station Z, from mesh
 * vertices and primitives; trunk radius from the boundingObject cylinder (else the lowest bands); the crown starts
 * where the radius first exceeds max(3 × trunk, 150 mm).
 */
export function treeProfile(tree: SceneObject, assets?: AssetStore, band = 100): TreeProfile {
  const P = tree.poseAbs();
  const bands = new Map<number, number>();
  const put = (z: number, r: number) => { const b = Math.floor(z / band); bands.set(b, Math.max(bands.get(b) ?? 0, r)); };
  for (const g of tree.geometry) {
    const T = g.origin ? fromArray(g.origin) : identity();
    const s = g.scale ?? [1, 1, 1];
    const m = g.mesh && assets ? (assets.resolve(g.mesh) ?? assets.get(g.mesh))?.mesh : null;
    if (m) {
      for (let i = 0; i < m.positions.length; i += 3) {
        const q = transformPoint(T, [m.positions[i] * s[0], m.positions[i + 1] * s[1], m.positions[i + 2] * s[2]]);
        put(q[2] + P[14], Math.hypot(q[0], q[1]));
      }
      continue;
    }
    const pr = g.primitive;
    if (!pr) continue;
    const c = transformPoint(T, [0, 0, 0]);
    const off = Math.hypot(c[0], c[1]);
    if (pr.kind === 'sphere') for (let dz = -pr.radius; dz <= pr.radius; dz += band / 2) put(c[2] + dz + P[14], off + Math.sqrt(Math.max(0, pr.radius * pr.radius - dz * dz)));
    else if (pr.kind === 'cylinder' || pr.kind === 'cone') for (let dz = -pr.length / 2; dz <= pr.length / 2; dz += band / 2) put(c[2] + dz + P[14], off + pr.radius);
    else if (pr.kind === 'box') for (let dz = -pr.size[2] / 2; dz <= pr.size[2] / 2; dz += band / 2) put(c[2] + dz + P[14], off + Math.hypot(pr.size[0], pr.size[1]) / 2);
  }
  const list = [...bands.entries()].sort((a, b) => a[0] - b[0]).map(([b, r]) => ({ z: b * band, r }));
  const cols = ((tree as unknown as { colliders?: GeometryRef[] }).colliders ?? []).map((g) => g.primitive).filter((p) => p?.kind === 'cylinder') as Array<{ kind: 'cylinder'; radius: number }>;
  const low = list.slice(0, 3).map((b) => b.r);
  // no geometry near the ground (murca_garden: a bare sphere crown from 0.3 m): no trunk, the crown starts at the bottom
  const trunkless = !cols.length && list.length > 0 && list[0].z > 150;
  const trunk = cols.length ? Math.max(...cols.map((c) => c.radius)) : trunkless ? 0 : low.length ? Math.min(...low) : 50;
  const thr = trunkless ? 0 : Math.max(3 * trunk, 150);
  const crown = list.filter((b) => b.r > thr);
  return {
    trunkRadius: trunk,
    crownBase: crown.length ? crown[0].z : 0,
    crownTop: list.length ? list[list.length - 1].z + band : 0,
    crownRadius: crown.length ? Math.max(...crown.map((b) => b.r)) : low.length ? Math.max(...low) : 500,
    bands: list,
  };
}

/** Crown radius of a profile within a height band [z0, z1] (mm). */
export function crownRadiusIn(p: TreeProfile, z0: number, z1: number): number {
  let r = 0;
  for (const b of p.bands) if (b.z + 100 > z0 && b.z < z1 && b.z >= p.crownBase) r = Math.max(r, b.r);
  return r;
}

/** Seeded fruit in the outer shell of the crown (positions relative to the plant base), CPython PRNG. */
function seedFruit(row: CropRow, profile: TreeProfile, n: number, rng: PyRandom, ripeFraction: number, base: number): void {
  const z0 = Math.max(profile.crownBase, 600), z1 = Math.max(z0 + 200, profile.crownTop - 200);
  for (const p of row.plants) {
    if (p.fruit.length) continue;
    for (let i = 0; i < n; i++) {
      const z = rng.uniform(z0, z1);
      const rb = crownRadiusIn(profile, z, z + 100) || profile.crownRadius;
      const r = rb * rng.uniform(0.7, 0.95), a = rng.uniform(-Math.PI, Math.PI);
      p.fruit.push({ p: [r * Math.cos(a), r * Math.sin(a), z - base], ripe: rng.random() < ripeFraction ? 1 : 0.3, picked: false, d: 75 });
    }
  }
}

/** Mission B roles, equipment and drives for the orchard_demo platforms; aerial drive for UTP-A. */
export function configureOrchardRobots(station: Station): Record<string, MobileRobot> {
  const robots: Record<string, MobileRobot> = {};
  for (const m of station.itemsOfType<MobileRobot>(ItemType.MOBILE_ROBOT)) {
    robots[m.name] = m;
    const spec = MISSION_B_ROLES[m.name];
    if (/^UTP-A/i.test(m.name)) {
      Object.assign(m.kin, { drive: 'aerial', maxSpeed: 5000, maxReverseSpeed: 5000, maxAccel: 2000, maxYawRate: 90 });
      m.state.z = m.pose()[14];
    }
    if (!spec) continue;
    m.capabilities = [...spec.caps];
    m.baseCapabilities = null;
    m.equipment = [];
    if (spec.recipe) applyEquipment(m, RECIPES[spec.recipe]);
    m.setParam('murkaRole', spec.role);
    if (!m.home) m.home = { x: m.state.x, y: m.state.y, theta: m.state.theta };
  }
  return robots;
}

function zoneAround(station: Station, name: string, kind: ZoneItem['kind'], items: Item[], pad: number): ZoneItem | null {
  if (!items.length) return null;
  const xs: number[] = [], ys: number[] = [];
  for (const it of items) {
    const P = it.poseAbs();
    const g = (it as SceneObject).geometry?.[0]?.primitive;
    const hx = g?.kind === 'box' ? g.size[0] / 2 : g?.kind === 'cylinder' ? g.radius : 500, hy = g?.kind === 'box' ? g.size[1] / 2 : g?.kind === 'cylinder' ? g.radius : 500;
    xs.push(P[12] - hx - pad, P[12] + hx + pad); ys.push(P[13] - hy - pad, P[13] + hy + pad);
  }
  const z = station.addChild(new ZoneItem(name));
  z.kind = kind;
  z.polygon = [[Math.min(...xs), Math.min(...ys)], [Math.max(...xs), Math.min(...ys)], [Math.max(...xs), Math.max(...ys)], [Math.min(...xs), Math.max(...ys)]];
  return z;
}

/** Build a MURKA agro world as a studio station (C2). */
export async function buildMurkaOrchardStation(opts: OrchardStationOptions = {}): Promise<OrchardStationResult> {
  const world = opts.world ?? 'orchard_demo';
  const assets = opts.assets ?? new AssetStore();
  let src: WebotsSource, worldPath = ORCHARD_WORLDS[world];
  if (opts.source) src = opts.source;
  else if (opts.files) ({ src, worldPath } = rootedFiles(opts.files, world));
  else {
    const root = opts.root ?? (typeof process !== 'undefined' ? process.env?.MURKA_DIR : undefined) ?? DEFAULT_MURKA_ROOT;
    src = await nodeDirSource(`${root.replace(/[\\/]+$/, '')}/${AGRO_WS_REL}`);
  }
  const r = importWebotsWorld(worldPath, src, { assets });
  const station = r.station;
  const report = [...r.report];
  const up = upgradeOrchard(station, { assets });
  report.push(...up.report);
  const trees = station.children.filter((c): c is SceneObject => c instanceof SceneObject && /tree/i.test(c.name));
  const profile = trees.length ? treeProfile(trees[0], assets) : null;
  const rng = new PyRandom(opts.fruitSeed ?? 0);
  let generated = 0;
  for (const f of up.fields) {
    if (profile) {
      f.crop.trunkDiameter = 2 * Math.max(profile.trunkRadius, 25);
      f.crop.canopyBase = Math.max(0, profile.crownBase);
      f.crop.canopyWidth = 2 * profile.crownRadius;
      f.crop.canopyHeight = Math.max(100, profile.crownTop - profile.crownBase);
    }
    for (const row of f.rows()) {
      // a single-tree row (orchard_demo / apple garden S1_07) has no direction: give it the field's row heading
      if (Math.hypot(row.end[0] - row.start[0], row.end[1] - row.start[1]) < 1) { const h = (f.crop.rowHeading ?? 0) * Math.PI / 180; row.end = [row.start[0] + Math.cos(h), row.start[1] + Math.sin(h)]; }
      for (const p of row.plants) { if (profile) { p.width = 2 * profile.crownRadius; p.height = Math.max(100, profile.crownTop - f.crop.canopyBase); } }
      if (profile) {
        const before = row.plants.reduce((a, p) => a + p.fruit.length, 0);
        seedFruit(row, profile, opts.fruitPerTree ?? 40, rng, f.crop.ripeFraction, 0);
        generated += row.plants.reduce((a, p) => a + p.fruit.length, 0) - before;
      }
    }
  }
  if (generated) report.push(`${generated} fruit generated in the crowns of trees without fruit geometry (seed ${opts.fruitSeed ?? 0})`);
  const robots = configureOrchardRobots(station);
  const byName = (re: RegExp) => station.children.filter((c) => re.test(c.name));
  const depot = zoneAround(station, 'Equipment depot (2 slots)', 'depot', byName(/^depot_slot_/), 300);
  if (depot) { depot.capacity = 2; depot.setParam('reconfigSeconds', 180); }
  byName(/^collection_point_/).forEach((cp, i) => zoneAround(station, `Collection point ${i + 1}`, 'unloading', [cp], 200));
  const charging = zoneAround(station, 'Charging pads', 'charging', byName(/^charging_pad_/), 300);
  void charging;
  const fruit = up.fields.reduce((a, f) => a + f.rows().reduce((b, row) => b + row.plants.reduce((c, p) => c + p.fruit.length, 0), 0), 0);
  report.push(`${world}: ${trees.length} trees in ${up.fields.length} fields, ${fruit} fruit, ${Object.keys(robots).length} mobile robots`);
  return { station, world, worldPath, report, fields: up.fields, robots, trees, profile, assets };
}

export interface MissionBOptions {
  /** Supervisor tick (s, default 0.016 like basicTimeStep 16). */
  dt?: number;
  /** Script end (s, default: last window of the script + 1). */
  tEnd?: number;
  script?: FleetScript;
  /** Include UTP-G-01 (the physical platform in Webots; scripted here). Default true. */
  includeG01?: boolean;
  world?: FleetWorld;
  onTick?: (t: number, fleet: ScriptedFleet) => void;
}

/** MURKA Mission B (garden_supervisor scripted fleet, T6 port) played on an orchard station. */
export function runMissionB(station: Station, opts: MissionBOptions = {}) {
  const script = opts.script ?? defaultMissionBScript(opts.includeG01 ?? true);
  const tEnd = opts.tEnd ?? Math.max(0, ...(script.waypoints ?? []).map((w) => Number(w.end_s) || 0), ...(script.carries ?? []).map((c) => Number(c.end_s) || 0), ...(script.reconfigs ?? []).map((c) => (Number(c.start_s) || 0) + (Number(c.transfer_s) || 0))) + 1;
  return playScriptedFleet(station, script, { dt: opts.dt ?? 0.016, tEnd, depotModules: MISSION_B.DEPOT_MODULES, world: opts.world ?? new StationFleetWorld(station), onTick: opts.onTick });
}

export interface HarvestFleetOptions {
  /** Number of harvesting platforms (default 2). */
  robots?: number;
  /** Arm of the platforms (robot library id, default the UR20, measured reach 1.77 m). */
  arm?: string;
  /** Platform footprint L × W × H (mm); default 1800 × 1000 × 1600 (fits the 1.6 m aisles of murca_garden and stays below
   *  the 2 m crown base of murca_apple_garden). */
  footprint?: [number, number, number];
  /** Fleet allocator (default 'auction'; 'murka' = the MuRKA contract net). */
  allocation?: string;
  secondsPerFruit?: number;
  /** Move the arm joints to every fruit while picking (IK per fruit, linear joint motion; agri process `animateArm`). */
  animateArm?: boolean;
}

/**
 * A studio harvest fleet on an imported MURKA garden: tracked platforms with picking arms parked south of the fields,
 * a FleetItem and a harvest mission over every row (both sides) planned by the agri mission planner — so the
 * FleetManager harvests the trees of the Webots world (fruit removed from the PlantRecords, contacts checked).
 */
export function addHarvestFleet(o: OrchardStationResult, opts: HarvestFleetOptions = {}): { fleet: FleetItem; fm: FleetManager; robots: MobileRobot[]; mission: MissionItem } {
  const st = o.station;
  const fleet = st.addChild(new FleetItem('MURKA harvest fleet'));
  if (opts.allocation) (fleet as { allocation: string }).allocation = opts.allocation;
  const fm = new FleetManager(st, fleet);
  const fp = opts.footprint ?? [1800, 1000, 1600];
  const ys = o.fields.flatMap((f) => f.rows().map((r) => Math.min(r.start[1], r.end[1])));
  const xs = o.fields.flatMap((f) => f.rows().flatMap((r) => [r.start[0], r.end[0]]));
  const y0 = Math.min(...ys) - (o.profile?.crownRadius ?? 1000) - 2500, x0 = (Math.min(...xs) + Math.max(...xs)) / 2;
  const robots: MobileRobot[] = [];
  for (let i = 0; i < (opts.robots ?? 2); i++) {
    const m = st.addChild(new MobileRobot(`Harvester ${i + 1}`));
    Object.assign(m.kin, { drive: 'tracked', wheelBase: 1200, track: 800, wheelRadius: 200, maxSpeed: 1000, footprint: fp });
    m.capabilities = ['harvest', 'transport'];
    m.color = '#c92a2a';
    m.setPose2D(x0 + (i - ((opts.robots ?? 2) - 1) / 2) * 3000, y0, 90);
    m.home = { x: m.state.x, y: m.state.y, theta: 90 };
    fm.addRobot(m);
    const arm = m.addChild(createRobotFromLibrary(opts.arm ?? 'UR20', `Arm ${i + 1}`));
    arm.setPose(mul(transl(0, -fp[1] / 2 + 150, fp[2]), rotz(-90 * DEG)));
    const g = arm.addChild(new Tool('Soft gripper'));
    g.toolKind = 'gripper';
    g.setPoseTool(transl(0, 0, 150));
    arm.setTool(g);
    robots.push(m);
  }
  const field = o.fields[0];
  const mission = st.addChild(new MissionItem('Harvest the MURKA garden'));
  mission.missionType = 'harvest';
  mission.fieldId = field.id;
  mission.fleetId = fleet.id;
  mission.rowIds = o.fields.flatMap((f) => f.rows().map((r) => r.id));
  mission.settings = { workSpeed: 300, bothSides: true, sideOffset: field.crop.rowSpacing / 2, secondsPerFruit: opts.secondsPerFruit ?? 3, ...(opts.animateArm ? { animateArm: true } : {}) };
  planMission(st, mission, fm);
  return { fleet, fm, robots, mission };
}
