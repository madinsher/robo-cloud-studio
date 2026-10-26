/**
 * Pose ground truth from Webots itself (webots.md P1): tests/fixtures/webots/gt/*.json.gz were dumped by a supervisor
 * controller (tests/fixtures/webots/gt/pose_dump.py) running in Webots R2025a batch mode on copies of the MURKA worlds
 * (gravity 0, other controllers off): absolute pose of every Pose-derived node — PROTO internals and template expansions
 * included — every Shape / CadShape geometry and every boundingObject geometry after the first step.
 * The studio import of the same world must reproduce every visual shape (kind, dimensions, world-space placement up to
 * the symmetry of the primitive) and every collider of the static Solids. Skipped visibly without the MURKA folder.
 */
import { describe, it, expect, beforeAll } from 'vitest';
import fs from 'node:fs';
import path from 'node:path';
import zlib from 'node:zlib';
import { importWebotsWorld, WebotsBuildResult, BUILTIN_PROTOS } from '../src/io/webots';
import { AssetStore } from '../src/scene/assets';
import { Item, GeometryRef, Camera } from '../src/core/items/item';
import { Robot } from '../src/core/items/robot';
import { Mat4, Vec3, identity, multiply, transformPoint, fromArray, transl } from '../src/core/math/pose';
import { HAS_MURKA, RETAIL_V2, RETAIL_V1, AGRO, murkaSource, dirSource, HAS_WEBOTS_CACHE, WEBOTS_HOME, webotsCacheFetcher } from './webots_helpers';

interface GtGeom { type: string; size?: number[]; radius?: number; height?: number; bottomRadius?: number; url?: string[] }
interface GtNode {
  id: number; parent: number | null; type: string; base: string; def?: string; name?: string; pose?: number[]; scale?: number[];
  geometry?: GtGeom; url?: string[]; position?: number | null; devices?: Array<{ type: string; name: string }>;
  bounding?: Array<{ geometry: GtGeom; pose: number[] | null }>;
}
const GT_DIR = path.resolve(__dirname, 'fixtures/webots/gt');
const loadGt = (f: string): GtNode[] => JSON.parse(zlib.gunzipSync(fs.readFileSync(path.join(GT_DIR, f))).toString('utf8')).nodes;

/** A shape in world coordinates (mm): kind, characteristic points (unordered) and size signature. */
interface WShape { kind: string; pts: Vec3[]; sig: number[]; label: string; mesh?: string; item?: Item }

/** Webots row-major 4x4 (m) -> studio column-major Mat4 (mm), scale-free. */
function wPose(p: number[]): Mat4 {
  const M = identity();
  for (let i = 0; i < 3; i++) { for (let j = 0; j < 3; j++) M[j * 4 + i] = p[i * 4 + j]; M[12 + i] = p[i * 4 + 3] * 1000; }
  return M;
}

/** Characteristic points of a primitive at rigid pose M (mm): box corners, cylinder/cone cap centres, plane corners, centre. */
function shapeOf(kind: string, dims: number[], M: Mat4, label: string, mesh?: string): WShape {
  const P = (x: number, y: number, z: number) => transformPoint(M, [x, y, z]);
  switch (kind) {
    case 'box': { const [a, b, c] = dims.map((v) => v / 2); const pts: Vec3[] = []; for (let k = 0; k < 8; k++) pts.push(P(k & 1 ? a : -a, k & 2 ? b : -b, k & 4 ? c : -c)); return { kind, pts, sig: [...dims].sort((x, y) => x - y), label }; }
    case 'cylinder': return { kind, pts: [P(0, 0, dims[1] / 2), P(0, 0, -dims[1] / 2)], sig: dims, label };
    case 'cone': return { kind, pts: [P(0, 0, dims[1] / 2), P(0, 0, -dims[1] / 2)], sig: dims, label }; // apex / base order kept
    case 'plane': { const [a, b] = dims.map((v) => v / 2); return { kind, pts: [P(a, b, 0), P(-a, b, 0), P(a, -b, 0), P(-a, -b, 0)], sig: [...dims].sort((x, y) => x - y), label }; }
    case 'sphere': return { kind, pts: [P(0, 0, 0)], sig: dims, label };
    default: return { kind, pts: [P(0, 0, 0), P(100, 0, 0), P(0, 100, 0), P(0, 0, 100)], sig: dims, label, mesh }; // meshes: full frame
  }
}

const base = (u: string) => u.split(/[\\/]/).pop()!.toLowerCase();

/** Expected shapes from the Webots dump (visual Shapes / CadShapes, or the boundingObjects of the selected Solids). */
function gtShapes(nodes: GtNode[], opts: { colliders?: boolean; skip?: (n: GtNode, chain: GtNode[]) => boolean } = {}): { shapes: WShape[]; ignored: string[] } {
  const by = new Map(nodes.map((n) => [n.id, n]));
  const chainOf = (n: GtNode) => { const c: GtNode[] = []; for (let p = n.parent; p != null; p = by.get(p)!.parent) c.push(by.get(p)!); return c; };
  const scaleOf = (chain: GtNode[]) => chain.reduce((s, a) => (a.base === 'Transform' && a.scale ? s.map((v, k) => v * a.scale![k]) : s), [1, 1, 1]);
  const shapes: WShape[] = [], ignored: string[] = [];
  const add = (g: GtGeom, pose: number[], sc: number[], label: string) => {
    const M = wPose(pose);
    switch (g.type) {
      case 'Box': shapes.push(shapeOf('box', g.size!.map((v, k) => v * 1000 * sc[k]), M, label)); break;
      case 'Cylinder': shapes.push(shapeOf('cylinder', [g.radius! * 1000 * Math.max(sc[0], sc[1]), g.height! * 1000 * sc[2]], M, label)); break;
      case 'Capsule': { // studio: cylinder + two end spheres
        const rr = g.radius! * 1000 * Math.max(sc[0], sc[1]), hh = g.height! * 1000 * sc[2];
        shapes.push(shapeOf('cylinder', [rr, hh], M, label));
        for (const z of [hh / 2, -hh / 2]) shapes.push(shapeOf('sphere', [rr], multiply(M, transl(0, 0, z)), `${label} (cap)`));
        break;
      }
      case 'Cone': shapes.push(shapeOf('cone', [g.bottomRadius! * 1000 * Math.max(sc[0], sc[1]), g.height! * 1000 * sc[2]], M, label)); break;
      case 'Sphere': shapes.push(shapeOf('sphere', [g.radius! * 1000 * Math.max(...sc)], M, label)); break;
      case 'Plane': shapes.push(shapeOf('plane', g.size!.map((v, k) => v * 1000 * sc[k]), M, label)); break;
      case 'Mesh': shapes.push(shapeOf('mesh', sc, M, label, base(g.url![0]))); break;
      case 'IndexedFaceSet': case 'ElevationGrid': shapes.push(shapeOf('mesh', sc, M, label, g.type.toLowerCase())); break; // generated meshes: placement only
      default: ignored.push(`${label}: ${g.type}`);
    }
  };
  for (const n of nodes) {
    const chain = chainOf(n);
    if (opts.skip?.(n, chain)) continue;
    if (!opts.colliders && n.geometry && /IndexedFaceSet|ElevationGrid/.test(n.geometry.type) && chain.some((a) => BUILTIN_PROTOS[a.type])) { ignored.push(`${n.type} of the built-in ${chain.find((a) => BUILTIN_PROTOS[a.type])!.type}`); continue; }
    const label = [...chain.reverse().map((a) => a.name ?? a.def ?? a.type), n.name ?? n.def ?? n.type].join('/');
    // (a Fluid's boundingObject is its immersion volume, not a collider)
    if (opts.colliders) { if (n.base !== 'Fluid') for (const b of n.bounding ?? []) if (b.pose) add(b.geometry, b.pose, [1, 1, 1], `${label} [bounding]`); continue; }
    // a Shape without a posed ancestor (top level / in a Group) sits at the world origin
    const pose = n.pose ?? (n.base === 'Shape' || n.base === 'CadShape' ? [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1] : null);
    if (!pose) continue;
    if (n.base === 'Shape' && n.geometry) add(n.geometry, pose, scaleOf(chainOf(n)), label);
    else if (n.base === 'CadShape' && n.url?.length) shapes.push(shapeOf('mesh', scaleOf(chainOf(n)), wPose(pose), label, base(n.url[0])));
  }
  return { shapes, ignored };
}

/** Studio geometry in world coordinates: object / vehicle geometry (or colliders) and robot link visuals at the current joints. */
function studioShapes(root: Item, assets: AssetStore, colliders = false): WShape[] {
  const out: WShape[] = [];
  let owner: Item | undefined;
  const add = (g: GeometryRef | { primitive?: GeometryRef['primitive']; mesh?: string; scale?: number[] }, M: Mat4, label: string) => {
    const n0 = out.length;
    addShape(g, M, label);
    for (let k = n0; k < out.length; k++) out[k].item = owner;
  };
  const addShape = (g: GeometryRef | { primitive?: GeometryRef['primitive']; mesh?: string; scale?: number[] }, M: Mat4, label: string) => {
    const p = g.primitive;
    if (p?.kind === 'box') out.push(shapeOf('box', p.size, M, label));
    else if (p?.kind === 'cylinder' || p?.kind === 'cone') out.push(shapeOf(p.kind, [p.radius, p.length], M, label));
    else if (p?.kind === 'sphere') out.push(shapeOf('sphere', [p.radius], M, label));
    else if (p?.kind === 'plane') out.push(shapeOf('plane', p.size, M, label));
    else if (g.mesh) {
      const sc = (g.scale ?? [1, 1, 1]).map((v) => v / 1000);
      const gen = /#(IndexedFaceSet|ElevationGrid)-/.exec(g.mesh); // meshes generated from IndexedFaceSet / ElevationGrid nodes
      out.push(shapeOf('mesh', sc, M, label, gen ? gen[1].toLowerCase() : base(g.mesh.split('#')[0]))); // textured variants: '<mesh id>#tex=<image>'
    }
  };
  for (const it of root.walk()) {
    owner = it;
    if (it instanceof Robot) {
      if (colliders) continue;
      const fk = it.fk();
      const P = it.poseAbs();
      it.chain.links.forEach((l, i) => { const L = multiply(P, fk.linkPoses[i]); for (const v of l.visuals) add(v, multiply(L, v.origin), `${it.name}/${l.name}`); });
      continue;
    }
    const list = colliders ? (it as unknown as { colliders?: GeometryRef[] }).colliders : (it as unknown as { geometry?: GeometryRef[] }).geometry;
    if (!Array.isArray(list)) continue;
    const P = it.poseAbs();
    for (const g of list) add(g, multiply(P, g.origin ? fromArray(g.origin) : identity()), it.name);
  }
  return out;
}

/** Distance between two unordered point sets (max over the best pairing; cones keep their order). */
function ptsDist(a: WShape, b: WShape): number {
  if (a.pts.length !== b.pts.length) return Infinity;
  const d = (p: Vec3, q: Vec3) => Math.hypot(p[0] - q[0], p[1] - q[1], p[2] - q[2]);
  if (a.kind === 'cone' || a.kind === 'mesh') return Math.max(...a.pts.map((p, i) => d(p, b.pts[i])));
  let worst = 0;
  const used = new Set<number>();
  for (const p of a.pts) {
    let best = Infinity, bi = -1;
    b.pts.forEach((q, i) => { if (!used.has(i)) { const v = d(p, q); if (v < best) { best = v; bi = i; } } });
    used.add(bi);
    worst = Math.max(worst, best);
  }
  return worst;
}

/** Match every expected shape to a distinct studio shape of the same kind / size within `tol` mm. */
function matchShapes(expected: WShape[], actual: WShape[], tol: number): { missing: string[]; maxErr: number; matched: number; extra: WShape[] } {
  const free = new Set(actual.map((_, i) => i));
  const missing: string[] = [];
  let maxErr = 0, matched = 0;
  for (const e of expected) {
    let best = Infinity, bi = -1;
    for (const i of free) {
      const a = actual[i];
      if (a.kind !== e.kind || (e.mesh && a.mesh !== e.mesh)) continue;
      if (a.sig.length !== e.sig.length || a.sig.some((v, k) => Math.abs(v - e.sig[k]) > (e.kind === 'mesh' ? 1e-4 : 0.5))) continue;
      const dd = ptsDist(e, a);
      if (dd < best) { best = dd; bi = i; }
    }
    if (bi >= 0 && best <= tol) { free.delete(bi); matched++; maxErr = Math.max(maxErr, best); }
    else missing.push(`${e.label} (${e.kind} ${e.mesh ?? e.sig.map((v) => v.toFixed(1)).join('x')} at ${e.pts[0].map((v) => v.toFixed(1)).join(',')}; nearest ${best.toFixed(1)} mm)`);
  }
  return { missing, maxErr, matched, extra: [...free].map((i) => actual[i]) };
}

/** `webots`: a sample world of the installed Webots, imported with the standard PROTOs of the local Webots asset cache. */
interface Case { name: string; gt: string; root: string; world: string; webots?: boolean }
const AGRO_WS = path.posix.dirname(AGRO).replace(/\/src$/, ''); // murca_garden_ws (holds assets/ for ../../../assets URLs)
const MSIM = 'simulation_ws/src/MURKA_Simulation/ros2_ws/src/murka_simulation';
const REPL = 'simulation_ws/src/agromarca/article/replication/src/murca_webots';
const CASES: Case[] = [
  { name: 'retail_store.wbt (MUROM v2)', gt: 'retail_store_v2.json.gz', root: RETAIL_V2, world: 'worlds/retail_store.wbt' },
  { name: 'retail_store.wbt (MUROM v1 classic)', gt: 'retail_store_v1.json.gz', root: RETAIL_V1, world: 'worlds/retail_store.wbt' },
  { name: 'murka_simulation retail_store.wbt (Murka)', gt: 'murka_sim_retail_store.json.gz', root: MSIM, world: 'worlds/retail_store.wbt' },
  { name: 'murka_simulation retail_store_classic.wbt', gt: 'murka_sim_retail_store_classic.json.gz', root: MSIM, world: 'worlds/retail_store_classic.wbt' },
  { name: 'retail_store_film.wbt (PROTOs four levels up)', gt: 'retail_store_film.json.gz', root: 'simulation_ws/src', world: 'murka_vla_study/experiments/materials/worlds/retail_store_film.wbt' },
  { name: '_legacy_2026-08 retail_store.wbt', gt: 'legacy_retail_store.json.gz', root: '_legacy_2026-08/ros2_ws/src/murka_simulation', world: 'worlds/retail_store.wbt' },
  { name: 'orchard_demo.wbt', gt: 'orchard_demo.json.gz', root: AGRO_WS, world: 'src/murca_webots/worlds/orchard_demo.wbt' },
  { name: 'murca_garden.wbt', gt: 'murca_garden.json.gz', root: AGRO_WS, world: 'src/murca_webots/worlds/murca_garden.wbt' },
  { name: 'murca_apple_garden.wbt', gt: 'murca_apple_garden.json.gz', root: AGRO_WS, world: 'src/murca_webots/worlds/murca_apple_garden.wbt' },
  { name: 'replication murca_garden.wbt', gt: 'replication_murca_garden.json.gz', root: REPL, world: 'worlds/murca_garden.wbt' },
  // R2023b world without EXTERNPROTO lines: Webots R2025a resolves Floor / TexturedBackground(Light) by name too
  { name: 'replication murca_apple_garden.wbt (no EXTERNPROTO)', gt: 'replication_murca_apple_garden.json.gz', root: REPL, world: 'worlds/murca_apple_garden.wbt' },
  // Webots' own sample worlds (standard robot / object PROTOs: ABB IRB 4600, KUKA youBot, e-puck, Clearpath Moose, ...);
  // a world whose standard PROTOs are not in the local Webots cache is skipped (visibly) instead of compared
  { name: 'Webots robots/abb/irb/worlds/inverse_kinematics.wbt', gt: 'webots_inverse_kinematics.json.gz', root: '', world: 'robots/abb/irb/worlds/inverse_kinematics.wbt', webots: true },
  { name: 'Webots robots/clearpath/moose/worlds/moose_demo.wbt', gt: 'webots_moose_demo.json.gz', root: '', world: 'robots/clearpath/moose/worlds/moose_demo.wbt', webots: true },
  { name: 'Webots robots/gctronic/e-puck/worlds/e-puck.wbt', gt: 'webots_e-puck.json.gz', root: '', world: 'robots/gctronic/e-puck/worlds/e-puck.wbt', webots: true },
  { name: 'Webots robots/gctronic/e-puck/worlds/e-puck2.wbt', gt: 'webots_e-puck2.json.gz', root: '', world: 'robots/gctronic/e-puck/worlds/e-puck2.wbt', webots: true },
  { name: 'Webots robots/gctronic/e-puck/worlds/e-puck2_pi-puck.wbt', gt: 'webots_e-puck2_pi-puck.json.gz', root: '', world: 'robots/gctronic/e-puck/worlds/e-puck2_pi-puck.wbt', webots: true },
  { name: 'Webots robots/gctronic/e-puck/worlds/e-puck2_server.wbt', gt: 'webots_e-puck2_server.json.gz', root: '', world: 'robots/gctronic/e-puck/worlds/e-puck2_server.wbt', webots: true },
  { name: 'Webots robots/gctronic/e-puck/worlds/e-puck_botstudio.wbt', gt: 'webots_e-puck_botstudio.json.gz', root: '', world: 'robots/gctronic/e-puck/worlds/e-puck_botstudio.wbt', webots: true },
  { name: 'Webots robots/gctronic/e-puck/worlds/e-puck_botstudio_with_floor_sensors.wbt', gt: 'webots_e-puck_botstudio_with_floor_sensors.json.gz', root: '', world: 'robots/gctronic/e-puck/worlds/e-puck_botstudio_with_floor_sensors.wbt', webots: true },
  { name: 'Webots robots/gctronic/e-puck/worlds/e-puck_cross-compilation.wbt', gt: 'webots_e-puck_cross-compilation.json.gz', root: '', world: 'robots/gctronic/e-puck/worlds/e-puck_cross-compilation.wbt', webots: true },
  { name: 'Webots robots/gctronic/e-puck/worlds/e-puck_kinematic.wbt', gt: 'webots_e-puck_kinematic.json.gz', root: '', world: 'robots/gctronic/e-puck/worlds/e-puck_kinematic.wbt', webots: true },
  { name: 'Webots robots/gctronic/e-puck/worlds/e-puck_line.wbt', gt: 'webots_e-puck_line.json.gz', root: '', world: 'robots/gctronic/e-puck/worlds/e-puck_line.wbt', webots: true },
  { name: 'Webots robots/gctronic/e-puck/worlds/e-puck_line_demo.wbt', gt: 'webots_e-puck_line_demo.json.gz', root: '', world: 'robots/gctronic/e-puck/worlds/e-puck_line_demo.wbt', webots: true },
  { name: 'Webots robots/kuka/youbot/worlds/youbot.wbt', gt: 'webots_youbot.json.gz', root: '', world: 'robots/kuka/youbot/worlds/youbot.wbt', webots: true },
  { name: 'Webots robots/kuka/youbot/worlds/youbot_matlab.wbt', gt: 'webots_youbot_matlab.json.gz', root: '', world: 'robots/kuka/youbot/worlds/youbot_matlab.wbt', webots: true },
  { name: 'Webots samples/demos/worlds/anaglyph.wbt', gt: 'webots_anaglyph.json.gz', root: '', world: 'samples/demos/worlds/anaglyph.wbt', webots: true },
  { name: 'Webots samples/demos/worlds/gantry.wbt', gt: 'webots_gantry.json.gz', root: '', world: 'samples/demos/worlds/gantry.wbt', webots: true },
  { name: 'Webots samples/demos/worlds/hexapod.wbt', gt: 'webots_hexapod.json.gz', root: '', world: 'samples/demos/worlds/hexapod.wbt', webots: true },
  { name: 'Webots samples/demos/worlds/moon.wbt', gt: 'webots_moon.json.gz', root: '', world: 'samples/demos/worlds/moon.wbt', webots: true },
  { name: 'Webots samples/demos/worlds/soccer.wbt', gt: 'webots_soccer.json.gz', root: '', world: 'samples/demos/worlds/soccer.wbt', webots: true },
  { name: 'Webots samples/demos/worlds/stewart_platform.wbt', gt: 'webots_stewart_platform.json.gz', root: '', world: 'samples/demos/worlds/stewart_platform.wbt', webots: true },
  { name: 'Webots samples/geometries/worlds/extended_solids.wbt', gt: 'webots_extended_solids.json.gz', root: '', world: 'samples/geometries/worlds/extended_solids.wbt', webots: true },
  { name: 'Webots samples/geometries/worlds/extrusion.wbt', gt: 'webots_extrusion.json.gz', root: '', world: 'samples/geometries/worlds/extrusion.wbt', webots: true },
  { name: 'Webots samples/geometries/worlds/floating_geometries.wbt', gt: 'webots_floating_geometries.json.gz', root: '', world: 'samples/geometries/worlds/floating_geometries.wbt', webots: true },
  { name: 'Webots samples/geometries/worlds/geometric_primitives.wbt', gt: 'webots_geometric_primitives.json.gz', root: '', world: 'samples/geometries/worlds/geometric_primitives.wbt', webots: true },
  { name: 'Webots samples/geometries/worlds/high_resolution_indexedfaceset.wbt', gt: 'webots_high_resolution_indexedfaceset.json.gz', root: '', world: 'samples/geometries/worlds/high_resolution_indexedfaceset.wbt', webots: true },
  { name: 'Webots samples/geometries/worlds/muscle.wbt', gt: 'webots_muscle.json.gz', root: '', world: 'samples/geometries/worlds/muscle.wbt', webots: true },
  { name: 'Webots samples/geometries/worlds/physics_primitives.wbt', gt: 'webots_physics_primitives.json.gz', root: '', world: 'samples/geometries/worlds/physics_primitives.wbt', webots: true },
  { name: 'Webots samples/geometries/worlds/polygons.wbt', gt: 'webots_polygons.json.gz', root: '', world: 'samples/geometries/worlds/polygons.wbt', webots: true },
  { name: 'Webots samples/geometries/worlds/textured_boxes.wbt', gt: 'webots_textured_boxes.json.gz', root: '', world: 'samples/geometries/worlds/textured_boxes.wbt', webots: true },
  { name: 'Webots samples/geometries/worlds/textured_proto_shapes.wbt', gt: 'webots_textured_proto_shapes.json.gz', root: '', world: 'samples/geometries/worlds/textured_proto_shapes.wbt', webots: true },
  { name: 'Webots samples/geometries/worlds/textured_shapes.wbt', gt: 'webots_textured_shapes.json.gz', root: '', world: 'samples/geometries/worlds/textured_shapes.wbt', webots: true },
  { name: 'Webots samples/geometries/worlds/webots_box.wbt', gt: 'webots_webots_box.json.gz', root: '', world: 'samples/geometries/worlds/webots_box.wbt', webots: true },
];

/**
 * PROTOs whose template seeds wbrandom with `context.id` (the Webots node id, which depends on Webots' internal node
 * creation order): the studio's random jitter differs (e.g. WoodenPalletStack misalignment, up to 11 mm). Documented
 * limitation; their shapes are not compared.
 */
const NONDETERMINISTIC = new Set(['WoodenPalletStack']);
const nondeterministic = (chain: GtNode[]) => chain.some((a) => NONDETERMINISTIC.has(a.type));
const isDumper = (n: GtNode, chain: GtNode[]) => n.name === 'gt_pose_dump' || chain.some((a) => a.name === 'gt_pose_dump');
const inRobot = (n: GtNode, chain: GtNode[]) => n.base === 'Robot' || chain.some((a) => a.base === 'Robot');

for (const c of CASES) {
  describe.skipIf(c.webots ? !HAS_WEBOTS_CACHE : !HAS_MURKA)(`Webots ground truth: ${c.name}`, () => {
    let r: WebotsBuildResult;
    let assets: AssetStore;
    let gt: GtNode[];
    let offline: string[] = [];
    beforeAll(() => {
      gt = loadGt(c.gt);
      assets = new AssetStore();
      r = c.webots ? importWebotsWorld(c.world, dirSource(path.join(WEBOTS_HOME, 'projects')), { assets, remote: webotsCacheFetcher() }) : importWebotsWorld(c.world, murkaSource(c.root), { assets });
      offline = r.report.filter((l) => /not available offline/.test(l));
    }, 180000);

    it('every visual shape of Webots (PROTO internals, templates, robots at their initial joints) is placed identically (<= 0.5 mm)', (t) => {
      if (offline.length) return t.skip(`standard PROTOs missing from the Webots cache: ${offline.join('; ')}`);
      const { shapes, ignored } = gtShapes(gt, { skip: (n, ch) => isDumper(n, ch) || nondeterministic(ch) });
      const res = matchShapes(shapes, studioShapes(r.station, assets), 0.5);
      expect(res.missing, `${c.name}: ${res.matched}/${shapes.length} matched`).toEqual([]);
      expect(shapes.length).toBeGreaterThanOrEqual(1);
      // the only studio shapes Webots does not have replace the IndexedFaceSets of the standard PROTOs (floor / arena)
      const builtin = new Set([...r.station.walk()].filter((i) => i.params.webotsBuiltin).map((i) => i.name));
      const inNondet = (i?: Item) => { for (let p: Item | null | undefined = i; p; p = p.parent) if (NONDETERMINISTIC.has(String(p.params.proto ?? ''))) return true; return false; };
      expect(res.extra.filter((e) => !builtin.has(e.label) && !inNondet(e.item)).map((e) => e.label)).toEqual([]);
      if (process.env.GT_DEBUG) fs.appendFileSync(process.env.GT_DEBUG, c.name + ' extra studio shapes: ' + JSON.stringify(res.extra.map((e) => `${e.label} ${e.kind} ${e.mesh ?? e.sig.map((v) => v.toFixed(1)).join('x')}`)) + '\n');
      expect(ignored.filter((l) => !/of the built-in|IndexedLineSet|PointSet/.test(l))).toEqual([]); // lines / points are not rendered (report line)
    });

    it('every motor of every Webots robot is exactly one studio joint (arm / head joint, drive wheel or steering joint)', (t) => {
      if (offline.length) return t.skip(`standard PROTOs missing from the Webots cache: ${offline.join('; ')}`);
      const by = new Map(gt.map((n) => [n.id, n]));
      const topRobot = (n: GtNode): GtNode | undefined => { let t: GtNode | undefined; for (let p: GtNode | undefined = n; p; p = p.parent != null ? by.get(p.parent) : undefined) if (p.base === 'Robot') t = p; return t; };
      const motors = new Map<string, string[]>();
      for (const n of gt) for (const d of n.devices ?? []) if (/Motor$/.test(d.type)) { const t = topRobot(n)!; motors.set(t.name!, [...(motors.get(t.name!) ?? []), d.name]); }
      for (const [robot, names] of motors) {
        const it = r.station.find(robot)!;
        expect(it, robot).toBeTruthy();
        const studio: string[] = [...((it.params.wheelJoints as string[]) ?? []), ...((it.params.steerJoints as string[]) ?? [])];
        for (const sub of it.walk()) if (sub instanceof Robot) studio.push(...sub.jointNames());
        expect([...studio].sort(), robot).toEqual([...names].sort());
      }
      if (c.gt === 'retail_store_v2.json.gz') expect(motors.get('murom')).toHaveLength(16);
    });

    it('every Webots Camera / RangeFinder / Lidar is a studio camera at the same place, optical +Z = Webots +X, image right = Webots -Y', (t) => {
      if (offline.length) return t.skip(`standard PROTOs missing from the Webots cache: ${offline.join('; ')}`);
      const by = new Map(gt.map((n) => [n.id, n]));
      const topRobot = (n: GtNode): GtNode | undefined => { let t: GtNode | undefined; for (let p: GtNode | undefined = n; p; p = p.parent != null ? by.get(p.parent) : undefined) if (p.base === 'Robot') t = p; return t; };
      const devs = gt.filter((n) => ['Camera', 'RangeFinder', 'Lidar'].includes(n.base) && n.pose && !isDumper(n, []));
      for (const d of devs) {
        const owner = topRobot(d);
        const scope: Item = owner ? r.station.find(owner.name!)! : r.station;
        const cams = [...scope.walk()].filter((i) => i instanceof Camera && i.name === d.name) as Camera[];
        expect(cams.length, `${c.name}: ${d.name}`).toBe(1);
        const P = cams[0].poseAbs(), W = wPose(d.pose!);
        expect(Math.hypot(P[12] - W[12], P[13] - W[13], P[14] - W[14]), d.name).toBeLessThan(0.5);
        const dot = (a: number, b: number) => P[a * 4] * W[b * 4] + P[a * 4 + 1] * W[b * 4 + 1] + P[a * 4 + 2] * W[b * 4 + 2]; // column a of studio . column b of Webots
        expect(dot(2, 0), `${d.name} optical axis`).toBeCloseTo(1, 6);
        expect(dot(0, 1), `${d.name} image right`).toBeCloseTo(-1, 6);
      }
      if (c.gt === 'retail_store_v2.json.gz') expect(devs.map((d) => d.name).sort()).toEqual(['head_camera', 'head_depth', 'lidar']);
    });

    it('every boundingObject of the static Solids becomes a collider at the same place (<= 0.5 mm)', (t) => {
      if (offline.length) return t.skip(`standard PROTOs missing from the Webots cache: ${offline.join('; ')}`);
      const { shapes } = gtShapes(gt, { colliders: true, skip: (n, ch) => isDumper(n, ch) || inRobot(n, ch) || NONDETERMINISTIC.has(n.type) || nondeterministic(ch) });
      const exp = shapes.filter((s) => s.kind !== 'plane'); // Plane bounds mark the ground (no collider)
      const res = matchShapes(exp, studioShapes(r.station, assets, true), 0.5);
      expect(res.missing, `${c.name}: ${res.matched}/${exp.length} matched`).toEqual([]);
    });
  });
}
