/**
 * Reader for RoboDK .robot / .tool / .sld / .rdk containers.
 *
 * After a 4-byte big-endian uncompressed size the payload is zlib. A robot or tool payload is a
 * table of little-endian doubles (kinematics, limits, name as character codes) followed by triangle
 * meshes. A station payload is "Station File 01.01": length-prefixed UTF-16 strings and a linked
 * list of `lvl` chunks (frame, robot, tool, object, program, machining).
 */
import { Station, Item, ItemType, Frame, SceneObject, Tool, Target } from '../../core/items/item';
import { Robot, RobotBrand } from '../../core/items/robot';
import { Program } from '../../core/items/program';
import { chainFromDHM } from '../../core/kinematics/mechanism';
import { identity, getPos, transl, rotx, roty, rotz, mul } from '../../core/math/pose';
import type { Mat4 } from '../../core/math/pose';
import type { MeshData } from '../mesh/stl';
import type { AssetStore } from '../../scene/assets';
import type { RdkScanReport } from './rdk_container';

const RAD2DEG = 180 / Math.PI;
let meshSeq = 0;
function meshId(name: string, i: number): string {
  meshSeq += 1;
  return `rdk:${meshSeq}:${name}:${i}`;
}

export interface RobodkLoad {
  station: Station;
  report: RdkScanReport;
}

interface Shape {
  positions: Float32Array;
  triangles: number;
  color: string;
  min: [number, number, number];
  max: [number, number, number];
}

function dvOf(b: Uint8Array): DataView {
  return new DataView(b.buffer, b.byteOffset, b.byteLength);
}

function u32(dv: DataView, o: number): number {
  return dv.getUint32(o, true);
}

function u64(dv: DataView, o: number): number {
  return dv.getUint32(o, true) + dv.getUint32(o + 4, true) * 0x100000000;
}

function f64(dv: DataView, o: number): number {
  return dv.getFloat64(o, true);
}

async function inflate(buf: Uint8Array, format: 'deflate' | 'gzip'): Promise<Uint8Array | null> {
  try {
    if (typeof DecompressionStream === 'undefined') return null;
    const ds = new DecompressionStream(format);
    const stream = new Blob([buf as unknown as BlobPart]).stream().pipeThrough(ds);
    return new Uint8Array(await new Response(stream).arrayBuffer());
  } catch {
    return null;
  }
}

/** Strip the RoboDK size prefix and inflate. Raw payloads are returned unchanged. */
export async function unwrapRobodk(data: ArrayBuffer): Promise<{ bytes: Uint8Array; compressed: RdkScanReport['compressed'] }> {
  const bytes = new Uint8Array(data);
  const zlibAt = (i: number) => i + 1 < bytes.length && bytes[i] === 0x78 && (bytes[i + 1] === 0x9c || bytes[i + 1] === 0xda || bytes[i + 1] === 0x01);
  if (bytes.length > 6 && zlibAt(4)) {
    const inflated = await inflate(bytes.subarray(4), 'deflate');
    if (inflated && inflated.length > 64) return { bytes: inflated, compressed: 'zlib' };
  }
  if (zlibAt(0)) {
    const inflated = await inflate(bytes, 'deflate');
    if (inflated && inflated.length > 64) return { bytes: inflated, compressed: 'zlib' };
  }
  if (bytes[0] === 0x1f && bytes[1] === 0x8b) {
    const inflated = await inflate(bytes, 'gzip');
    if (inflated) return { bytes: inflated, compressed: 'gzip' };
  }
  return { bytes, compressed: 'none' };
}

function readUtf16(dv: DataView, b: Uint8Array, o: number): { text: string; next: number } | null {
  if (o < 0 || o + 4 > b.length) return null;
  const n = u32(dv, o);
  if (n < 2 || n > 8000 || n % 2 !== 0 || o + 4 + n > b.length) return null;
  for (let i = 0; i < n; i += 2) {
    const c = b[o + 4 + i];
    if (b[o + 4 + i + 1] !== 0 || c < 32 || c > 126) return null;
  }
  let text = '';
  for (let i = 0; i < n; i += 2) text += String.fromCharCode(b[o + 4 + i]);
  return { text, next: o + 4 + n };
}

/** ASCII stored one character code per double. */
function charsAt(dv: DataView, index: number, max = 64): string {
  let s = '';
  for (let i = 0; i < max; i++) {
    const off = (index + i) * 8;
    if (off + 8 > dv.byteLength) break;
    const v = f64(dv, off);
    if (!Number.isFinite(v) || Math.abs(v - Math.round(v)) > 1e-6) break;
    const c = Math.round(v);
    if (c === 0) break;
    if (c < 32 || c > 126) break;
    s += String.fromCharCode(c);
  }
  return s;
}

function hexColor(r: number, g: number, b: number): string {
  const h = (v: number) => Math.max(0, Math.min(255, Math.round(v * 255))).toString(16).padStart(2, '0');
  return `#${h(r)}${h(g)}${h(b)}`;
}

/**
 * Triangle soup after the parameter table.
 * `skipExtra` consumes the extra count that robot files store between the shape count and the first mesh.
 */
function readShapeGroup(b: Uint8Array, start: number, skipExtra: boolean): { shapes: Shape[]; end: number } | null {
  const dv = dvOf(b);
  let o = start;
  if (o + (skipExtra ? 24 : 16) > b.length) return null;
  const nshapes = u64(dv, o);
  o += 8;
  if (nshapes < 1 || nshapes > 8192) return null;
  if (skipExtra) o += 8;
  const shapes: Shape[] = [];
  for (let s = 0; s < nshapes; s++) {
    if (o + 24 > b.length) break;
    const nt = u64(dv, o);
    o += 8;
    if (nt === 0) break;
    if (nt > 2_000_000 || o + 16 + nt * 3 * 14 > b.length) break;
    const cr = dv.getFloat32(o, true), cg = dv.getFloat32(o + 4, true), cb = dv.getFloat32(o + 8, true);
    o += 16;
    if (![cr, cg, cb].every((c) => Number.isFinite(c) && c >= -0.05 && c <= 1.5)) break;
    const positions = new Float32Array(nt * 9);
    const min: [number, number, number] = [Infinity, Infinity, Infinity];
    const max: [number, number, number] = [-Infinity, -Infinity, -Infinity];
    let ok = true;
    for (let i = 0; i < nt * 3; i++) {
      const x = dv.getFloat32(o, true), y = dv.getFloat32(o + 4, true), z = dv.getFloat32(o + 8, true);
      o += 14;
      if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z) || Math.abs(x) > 1e6 || Math.abs(y) > 1e6 || Math.abs(z) > 1e6) { ok = false; break; }
      positions[i * 3] = x; positions[i * 3 + 1] = y; positions[i * 3 + 2] = z;
      min[0] = Math.min(min[0], x); min[1] = Math.min(min[1], y); min[2] = Math.min(min[2], z);
      max[0] = Math.max(max[0], x); max[1] = Math.max(max[1], y); max[2] = Math.max(max[2], z);
    }
    if (!ok) break;
    shapes.push({ positions, triangles: nt, color: hexColor(cr, cg, cb), min, max });
  }
  return shapes.length ? { shapes, end: o } : null;
}

/** One colored triangle soup: triangle count, RGBA, then 14-byte vertices. Used when a mesh is not wrapped in a shape-group count. */
function readOneShape(b: Uint8Array, start: number): { shape: Shape; end: number } | null {
  const dv = dvOf(b);
  if (start + 24 > b.length) return null;
  const nt = u64(dv, start);
  if (nt < 1 || nt > 2_000_000 || start + 24 + nt * 3 * 14 > b.length) return null;
  const cr = dv.getFloat32(start + 8, true), cg = dv.getFloat32(start + 12, true), cb = dv.getFloat32(start + 16, true);
  const ca = dv.getFloat32(start + 20, true);
  if (![cr, cg, cb, ca].every((c) => Number.isFinite(c) && c >= -0.05 && c <= 1.5)) return null;
  let o = start + 24;
  const positions = new Float32Array(nt * 9);
  const min: [number, number, number] = [Infinity, Infinity, Infinity];
  const max: [number, number, number] = [-Infinity, -Infinity, -Infinity];
  for (let i = 0; i < nt * 3; i++) {
    const x = dv.getFloat32(o, true), y = dv.getFloat32(o + 4, true), z = dv.getFloat32(o + 8, true);
    o += 14;
    if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z) || Math.abs(x) > 1e6 || Math.abs(y) > 1e6 || Math.abs(z) > 1e6) return null;
    positions[i * 3] = x; positions[i * 3 + 1] = y; positions[i * 3 + 2] = z;
    min[0] = Math.min(min[0], x); min[1] = Math.min(min[1], y); min[2] = Math.min(min[2], z);
    max[0] = Math.max(max[0], x); max[1] = Math.max(max[1], y); max[2] = Math.max(max[2], z);
  }
  return { shape: { positions, triangles: nt, color: hexColor(cr, cg, cb), min, max }, end: o };
}

/**
 * A 1-axis mechanism stores `nLinks`, then one shape-count and that many soups per link.
 * Link 0 is the fixed body. The next link is the part that travels (a door, a finger, a belt).
 */
function readCountedLinks(b: Uint8Array, start: number, nLinks: number): Shape[][] | null {
  if (nLinks < 2 || nLinks > 4) return null;
  const dv = dvOf(b);
  if (start + 16 > b.length || u64(dv, start) !== nLinks) return null;
  let o = start + 8;
  const groups: Shape[][] = [];
  for (let link = 0; link < nLinks; link++) {
    if (o + 8 > b.length) return null;
    let n = u64(dv, o);
    o += 8;
    // Zero words separate the fixed body from the travelling link's shape count.
    if (n === 0 && link > 0) {
      while (o + 8 <= b.length && u64(dv, o) === 0) o += 8;
      if (o + 8 <= b.length) {
        const next = u64(dv, o);
        if (next > 0 && next <= 20000) { n = next; o += 8; }
      }
    }
    if (n > 20000) return null;
    const shapes: Shape[] = [];
    for (let i = 0; i < n; i++) {
      const one = readOneShape(b, o);
      if (!one) return null;
      shapes.push(one.shape);
      o = one.end;
    }
    groups.push(shapes);
  }
  return groups.some((g) => g.length) ? groups : null;
}

/** One RoboDK object is a sequence of shape groups. A robot stores one group per link. */
function readAllGroups(b: Uint8Array, start: number, limit = 8192): Shape[][] {
  const dv = dvOf(b);
  let o = start;
  const groups: Shape[][] = [];
  while (o + 24 < b.length && groups.length < limit) {
    while (o + 8 <= b.length && u64(dv, o) === 0) o += 8;
    const plain = readShapeGroup(b, o, false);
    const extra = readShapeGroup(b, o, true);
    const score = (g: { shapes: Shape[]; end: number } | null) => g?.shapes.reduce((n, s) => n + s.triangles, 0) ?? 0;
    const best = score(extra) > score(plain) ? extra : plain;
    if (best && best.end > o) {
      groups.push(best.shapes);
      o = best.end;
      continue;
    }
    const one = readOneShape(b, o);
    if (!one || one.end <= o) break;
    groups.push([one.shape]);
    o = one.end;
  }
  return groups;
}

function bestShapes(b: Uint8Array, start: number): Shape[] {
  return readAllGroups(b, start)[0] ?? [];
}

function toMesh(shape: Shape): MeshData {
  return { positions: shape.positions, normals: new Float32Array(0), min: shape.min, max: shape.max, triangles: shape.triangles };
}

/** Current joint pose is stored at double index 700 of a robot parameter table. */
function savedJoints(dv: DataView, dof: number): number[] | null {
  if (dof < 1 || (700 + dof) * 8 > dv.byteLength) return null;
  const q: number[] = [];
  for (let i = 0; i < dof; i++) {
    const v = f64(dv, (700 + i) * 8);
    if (!Number.isFinite(v) || Math.abs(v) > 1e4) return null;
    q.push(v);
  }
  return q.some((v) => Math.abs(v) > 1e-6) ? q : null;
}

/** xyz (mm) plus XYZ Euler radians at double index 180. Arms leave this block at zero. */
function mechanismPose(dv: DataView): Mat4 | null {
  if (188 * 8 > dv.byteLength) return null;
  const x = f64(dv, 180 * 8), y = f64(dv, 181 * 8), z = f64(dv, 182 * 8);
  const rx = f64(dv, 183 * 8), ry = f64(dv, 184 * 8), rz = f64(dv, 185 * 8);
  if (![x, y, z, rx, ry, rz].every(Number.isFinite)) return null;
  if (Math.hypot(x, y, z) < 0.5 && Math.abs(rx) < 1e-4 && Math.abs(ry) < 1e-4 && Math.abs(rz) < 1e-4) return null;
  if (Math.hypot(x, y, z) > 1e7) return null;
  return mul(transl(x, y, z), rotx(rx), roty(ry), rotz(rz));
}

function tablePose(dv: DataView, index: number): Mat4 | null {
  if ((index + 16) * 8 > dv.byteLength) return null;
  if (!looksLikePose(dv, index * 8)) return null;
  const pose = readPose(dv, index * 8);
  return poseUsable(pose) ? pose : null;
}

function brandOf(name: string): RobotBrand {
  const n = name.toLowerCase();
  const table: Array<[string, RobotBrand]> = [['kuka', 'KUKA'], ['abb', 'ABB'], ['fanuc', 'Fanuc'], ['universal', 'UR'], [' ur', 'UR'], ['yaskawa', 'Yaskawa'], ['motoman', 'Yaskawa'], ['staubli', 'Staubli'], ['doosan', 'Doosan'], ['kawasaki', 'Kawasaki'], ['denso', 'Denso'], ['mecademic', 'Mecademic']];
  for (const [k, b] of table) if (n.includes(k)) return b;
  return 'Generic';
}

function paramBlock(dv: DataView): { kind: 'robot' | 'object'; count: number } | null {
  if (dv.byteLength < 32) return null;
  const v0 = f64(dv, 0), v1 = f64(dv, 8);
  if (v0 !== 1) return null;
  const count = Math.round(v1);
  if (count < 200 || count > 20000 || count * 8 > dv.byteLength + 8) return null;
  if (Math.abs(v1 - count) > 1e-6) return null;
  if (count >= 3000) return { kind: 'robot', count };
  return { kind: 'object', count };
}

/** Build a robot from a parameter-table payload (the inflated .robot body, or an embedded copy). */
export function robotFromTable(bytes: Uint8Array, fallbackName: string, assets?: AssetStore): Robot | null {
  const dv = dvOf(bytes);
  const block = paramBlock(dv);
  if (!block || block.kind !== 'robot') return null;
  const dof = Math.round(f64(dv, 21 * 8));
  if (dof < 1 || dof > 12 ||  (200 + dof * 20) * 8 > dv.byteLength) return null;
  const name = charsAt(dv, 1800) || fallbackName || 'Robot';
  const post = charsAt(dv, 3800);
  const rows: number[][] = [];
  const types: Array<'revolute' | 'prismatic'> = [];
  const lower: number[] = [];
  const upper: number[] = [];
  for (let i = 0; i < dof; i++) {
    const base = (200 + i * 20) * 8;
    rows.push([f64(dv, base) * RAD2DEG, f64(dv, base + 8), f64(dv, base + 16) * RAD2DEG, f64(dv, base + 24)]);
    const jointKind = f64(dv, base + 32);
    types.push(Math.round(jointKind) === 1 ? 'prismatic' : 'revolute');
    const lo = f64(dv, (600 + i) * 8), hi = f64(dv, (620 + i) * 8);
    lower.push(Number.isFinite(lo) ? lo : -180);
    upper.push(Number.isFinite(hi) ? hi : 180);
  }
  const chain = chainFromDHM(name, rows, lower, upper, types);
  for (let i = 0; i < chain.joints.length; i++) {
    const sense = f64(dv, (64 + i) * 8);
    if (sense === 1 || sense === -1) chain.joints[i].axis = [0, 0, sense];
  }
  // Comau Smart5 stores 11 at double index 61. Joint 3 is parallelogram-coupled to joint 2.
  if (Math.round(f64(dv, 61 * 8)) === 11 && chain.joints.length >= 3) {
    const s2 = chain.joints[1].axis[2] || 1;
    const s3 = chain.joints[2].axis[2] || 1;
    chain.joints[2].couple = { joint: chain.joints[1].name, multiplier: -s2 / s3 };
  }
  const meshAt = block.count * 8;
  let counted = readCountedLinks(bytes, meshAt, chain.links.length);
  if (!counted && chain.joints.length === 1) {
    // A parallel-jaw gripper stores the fixed body plus two fingers that travel opposite ways.
    const paired = readCountedLinks(bytes, meshAt, chain.links.length + 1);
    if (paired) {
      const src = chain.joints[0];
      // The joint value is the full jaw opening. Each finger travels half of it, in opposite directions.
      src.axis = [src.axis[0] * 0.5, src.axis[1] * 0.5, src.axis[2] * 0.5];
      chain.joints.push({
        name: `${src.name}b`, type: src.type, origin: rotx(Math.PI), axis: [0, 0, 1],
        lower: src.lower, upper: src.upper, maxVelocity: src.maxVelocity,
        mimic: { joint: src.name, multiplier: 1, offset: 0 },
      });
      chain.links.push({ name: `link_${chain.links.length}`, visuals: [] });
      counted = paired;
    }
  }
  const declared = u64(dv, meshAt);
  const groups = counted ?? readAllGroups(bytes, meshAt, declared >= chain.links.length && declared <= chain.links.length + 8 ? declared : 8192);
  const reachGuess = f64(dv, 101 * 8);
  const perLink = counted != null || (groups.length > 1 && (groups.length === chain.links.length || declared === groups.length));
  const linkShapes = !perLink && groups.length === 1 && perLinkMeshes(groups[0] ?? [], reachGuess > 50 ? reachGuess : 1000) ? groups[0] : null;
  const shapes = perLink ? groups.flat() : (linkShapes ?? groups.flat());
  if (assets && shapes.length) {
    const place = (shape: Shape, linkIndex: number) => {
      const id = meshId(name, linkIndex);
      assets.registerMesh(id, toMesh(shape), `${name} link ${linkIndex}`);
      chain.links[Math.min(linkIndex, chain.links.length - 1)].visuals.push({
        mesh: id,
        origin: counted != null && linkIndex === 0 && chain.joints.some((j) => j.mimic) ? roty(-Math.PI / 2) : identity(),
        color: shape.color,
      });
    };
    if (perLink) groups.forEach((group, li) => group.forEach((shape) => place(shape, li < chain.links.length ? li : Math.min(2, chain.links.length - 1))));
    else if (linkShapes) linkShapes.forEach((shape, i) => place(shape, i));
    else shapes.forEach((shape) => place(shape, 0));
  }
  const robot = new Robot(name, chain);
  robot.brand = brandOf(name);
  robot.model = name;
  if (post) robot.postProcessor = post;
  const reach = f64(dv, 101 * 8);
  if (reach > 50 && reach < 20000) robot.reach = reach;
  const saved = savedJoints(dv, dof);
  if (saved && saved.every((v) => Math.abs(v) <= 720)) robot.setJoints(saved);
  const mounted = mechanismPose(dv);
  if (mounted) robot.setPose(mounted);
  robot.params.rdkMeshes = shapes.length;
  robot.params.rdkMeshNote = shapes.length ? '' : 'link meshes are not in joint frames; the kinematic model is shown';
  return robot;
}

/** True when the stored shapes are spread out like separate links, not one cluster of details. */
function perLinkMeshes(shapes: Shape[], reach: number): boolean {
  if (shapes.length < 2) return false;
  const center = (s: Shape) => [(s.min[0] + s.max[0]) / 2, (s.min[1] + s.max[1]) / 2, (s.min[2] + s.max[2]) / 2];
  const cs = shapes.map(center);
  let spread = 0;
  for (let i = 0; i < cs.length; i++) for (let j = i + 1; j < cs.length; j++) {
    spread = Math.max(spread, Math.hypot(cs[i][0] - cs[j][0], cs[i][1] - cs[j][1], cs[i][2] - cs[j][2]));
  }
  const biggest = Math.max(...shapes.map((s) => Math.hypot(s.max[0] - s.min[0], s.max[1] - s.min[1], s.max[2] - s.min[2])));
  return spread > reach * 0.2 || biggest > reach * 0.35;
}

function objectFromTable(bytes: Uint8Array, fallbackName: string, asTool: boolean, assets?: AssetStore): SceneObject | null {
  const dv = dvOf(bytes);
  const block = paramBlock(dv);
  if (!block) return null;
  const name = charsAt(dv, 1440) || charsAt(dv, 1800) || fallbackName || (asTool ? 'Tool' : 'Object');
  const item = asTool ? new Tool(name) : new SceneObject(name);
  const placed = tablePose(dv, 96);
  if (!asTool && placed) item.setPose(placed);
  if (!asTool && f64(dv, 48 * 8) === 0) item.setVisible(false);
  if (asTool) {
    const tcp = tablePose(dv, 160);
    if (tcp) item.setPose(tcp);
  }
  const geom = tablePose(dv, 32);
  const origin = geom ? Array.from(geom) : Array.from(identity());
  const shapes = readAllGroups(bytes, block.count * 8).flat();
  if (assets) {
    shapes.forEach((shape, i) => {
      const id = meshId(name, i);
      assets.registerMesh(id, toMesh(shape), name);
      item.geometry.push({ mesh: id, origin: origin.slice(), color: shape.color });
    });
    if (shapes.length) {
      const min = shapes.reduce((a, s) => a.map((v, i) => Math.min(v, s.min[i])) as [number, number, number], [Infinity, Infinity, Infinity] as [number, number, number]);
      const max = shapes.reduce((a, s) => a.map((v, i) => Math.max(v, s.max[i])) as [number, number, number], [-Infinity, -Infinity, -Infinity] as [number, number, number]);
      item.bbox = { min, max };
    }
  }
  item.params.rdkMeshes = shapes.length;
  return item;
}

function findTable(b: Uint8Array, from: number, to: number, marker: number): number {
  const dv = dvOf(b);
  const end = Math.min(to, b.length - 16);
  for (let o = from; o <= end; o++) {
    if (f64(dv, o) === 1 && Math.round(f64(dv, o + 8)) === marker) return o;
  }
  return -1;
}

function looksLikePose(dv: DataView, o: number): boolean {
  if (o < 0 || o + 128 > dv.byteLength) return false;
  const m15 = f64(dv, o + 120);
  if (Math.abs(m15 - 1) > 1e-4) return false;
  const c0 = Math.hypot(f64(dv, o), f64(dv, o + 8), f64(dv, o + 16));
  const c1 = Math.hypot(f64(dv, o + 32), f64(dv, o + 40), f64(dv, o + 48));
  return Math.abs(c0 - 1) < 1e-3 && Math.abs(c1 - 1) < 1e-3;
}

function readPose(dv: DataView, o: number): Mat4 {
  const m = identity();
  for (let i = 0; i < 16; i++) m[i] = f64(dv, o + i * 8);
  return m;
}

interface Chunk { kind: number; depth: number; payload: number; end: number }

function chunksOf(b: Uint8Array): Chunk[] {
  const dv = dvOf(b);
  let o = 0;
  const limit = Math.min(b.length, 512);
  while (o < limit) {
    const s = readUtf16(dv, b, o);
    if (s?.text === 'lvl') break;
    o++;
  }
  const out: Chunk[] = [];
  const seen = new Set<number>();
  while (o + 20 < b.length) {
    const tag = readUtf16(dv, b, o);
    if (tag?.text !== 'lvl') break;
    const base = tag.next;
    const kind = u32(dv, base);
    const depth = u32(dv, base + 4);
    const next = u32(dv, base + 8);
    out.push({ kind, depth, payload: base + 12, end: next > o && next <= b.length ? next : b.length });
    if (next <= o || next >= b.length || seen.has(next)) break;
    seen.add(o);
    o = next;
  }
  return out;
}

function firstText(b: Uint8Array, from: number, to: number, skip: Set<string>): string | null {
  const dv = dvOf(b);
  for (let o = from; o < to - 4 && o < from + 400; o++) {
    const s = readUtf16(dv, b, o);
    if (!s) continue;
    if (!skip.has(s.text)) return s.text;
    o = s.next - 1;
  }
  return null;
}

function frameOf(b: Uint8Array, payload: number, end: number): { name: string; pose: Mat4 } {
  const dv = dvOf(b);
  let name = 'Frame';
  let pose = identity();
  for (let o = payload; o < end - 4; o++) {
    const s = readUtf16(dv, b, o);
    if (!s) continue;
    if (s.text === 'FrameChkptStart') {
      let n = readUtf16(dv, b, s.next);
      if (!n) {
        for (let p = s.next; p < end - 4 && !n; p++) n = readUtf16(dv, b, p);
      }
      if (n && n.text !== 'FrameChkptEnd') {
        name = n.text;
        for (let d = 0; d <= 8; d++) if (looksLikePose(dv, n.next + d)) { pose = readPose(dv, n.next + d); break; }
      }
      break;
    }
    o = s.next - 1;
  }
  return { name, pose };
}

function afterName(label: string): string {
  const m = label.match(/:\s*(.+)$/);
  return (m ? m[1] : label).trim();
}

/** "MoveL (Top Paint 1)" -> "Top Paint 1". */
function moveTargetName(label: string): string {
  const m = label.match(/\((.*)\)\s*$/);
  return (m ? m[1] : label).trim();
}

function poseUsable(m: Mat4): boolean {
  if (Math.abs(m[15] - 1) > 1e-3) return false;
  const n0 = Math.hypot(m[0], m[1], m[2]);
  const n1 = Math.hypot(m[4], m[5], m[6]);
  if (Math.abs(n0 - 1) > 1e-2 || Math.abs(n1 - 1) > 1e-2) return false;
  const t = Math.hypot(m[12], m[13], m[14]);
  return Number.isFinite(t) && t < 1e7;
}

/** Target records store dof, joint values and a 4×4 pose after the name, not on an 8-byte grid. */
function targetFromChunk(b: Uint8Array, payload: number, end: number): Target | null {
  const dv = dvOf(b);
  const name = firstText(b, payload, end, new Set(['lvl', 'FrameChkptStart', 'FrameChkptEnd'])) ?? 'Target';
  let from = payload;
  for (let o = payload; o < Math.min(end, payload + 200); o++) {
    const s = readUtf16(dv, b, o);
    if (s && s.text === name) { from = s.next; break; }
  }
  for (let d = 0; d <= 24; d++) {
    if (from + d + 4 > end) break;
    const dof = u32(dv, from + d);
    if (dof < 1 || dof > 12) continue;
    const jAt = from + d + 4;
    const pAt = jAt + dof * 8;
    if (pAt + 128 > end) continue;
    const joints: number[] = [];
    let ok = true;
    for (let i = 0; i < dof; i++) {
      const v = f64(dv, jAt + i * 8);
      if (!Number.isFinite(v) || Math.abs(v) > 1e4) { ok = false; break; }
      joints.push(v);
    }
    if (!ok || !looksLikePose(dv, pAt)) continue;
    const pose = readPose(dv, pAt);
    if (!poseUsable(pose)) continue;
    const target = new Target(name);
    target.setPose(pose);
    target.setJoints(joints);
    return target;
  }
  const target = new Target(name);
  return target;
}

function fillProgram(program: Program, b: Uint8Array, payload: number, end: number, station: Station): void {
  const dv = dvOf(b);
  const spots: Array<{ name: string; at: number }> = [];
  for (let o = payload; o < end - 4; o++) {
    const s = readUtf16(dv, b, o);
    if (!s) continue;
    if (s.text === 'ins') {
      let p = s.next;
      let name: { text: string; next: number } | null = null;
      const stop = Math.min(end, s.next + 80);
      while (p < stop) {
        const t = readUtf16(dv, b, p);
        if (t && t.text !== 'ins') { name = t; break; }
        p++;
      }
      if (name) spots.push({ name: name.text, at: name.next });
      o = (name?.next ?? s.next) - 1;
    } else o = s.next - 1;
  }
  let frame: Item = station;
  for (const ins of spots) {
    const label = ins.name;
    const call = label.match(/^(?:Call|Run)\s+(.+)$/i);
    if (call) {
      const spec = call[1].trim();
      const arg = spec.match(/^(.*?)\((.*)\)\s*$/);
      const name = (arg ? arg[1] : spec).trim();
      const sub = station.find(name, ItemType.PROGRAM);
      if (sub instanceof Program) program.callProgram(sub);
      else program.comment(label);
    } else if (/^move/i.test(label) && ins.at + 14 <= b.length) {
      const moveType = u32(dv, ins.at);
      const named = station.find(moveTargetName(label), ItemType.TARGET) ?? station.find(label, ItemType.TARGET);
      let target = named instanceof Target ? named : null;
      if (!target) {
        const isJoint = dv.getUint16(ins.at + 8, true);
        const dof = dv.getUint16(ins.at + 10, true);
        const joints: number[] = [];
        let p = ins.at + 14;
        if (dof > 0 && dof <= 16 && p + dof * 8 <= b.length) {
          for (let i = 0; i < dof; i++) joints.push(f64(dv, p)), p += 8;
        }
        const pose = identity();
        if (p + 15 * 8 <= b.length) {
          for (let i = 0; i < 12; i++) pose[i] = f64(dv, p + i * 8);
          pose[12] = f64(dv, p + 12 * 8);
          pose[13] = f64(dv, p + 13 * 8);
          pose[14] = f64(dv, p + 14 * 8);
          pose[15] = 1;
        }
        target = new Target(moveTargetName(label));
        if (poseUsable(pose)) target.setPose(pose);
        if (joints.length && joints.every((v) => Number.isFinite(v) && Math.abs(v) < 1e4)) target.setJoints(joints);
        if (isJoint === 1) target.setAsJointTarget();
        frame.addChild(target);
      }
      if (moveType === 1) program.addMoveJ(target);
      else program.addMoveL(target);
    } else if (/speed/i.test(label)) {
      const v = parseFloat((label.match(/-?\d+(?:\.\d+)?/) ?? [])[0] ?? '');
      if (/joint|deg/i.test(label)) program.setSpeed(undefined, Number.isFinite(v) ? v : undefined);
      else program.setSpeed(Number.isFinite(v) ? v : undefined);
    } else if (/set ref/i.test(label)) {
      const found = station.find(afterName(label), ItemType.FRAME);
      if (found) { frame = found; program.setFrame(found); }
      else program.comment(label);
    } else if (/set tool/i.test(label)) {
      const found = station.find(afterName(label), ItemType.TOOL);
      if (found) program.setTool(found);
      else program.comment(label);
    } else if (/^replace objects$/i.test(label)) {
      program.event('custom', null, 'Replace objects');
    } else if (/^(show|hide)\s+(.+)$/i.test(label)) {
      const vis = label.match(/^(show|hide)\s+(.+)$/i)!;
      const what = vis[2].trim();
      const obj = station.find(what);
      program.event(vis[1].toLowerCase() === 'show' ? 'show' : 'hide', obj?.id ?? null, what);
    } else if (/^(attach to|detach from)\s+(.+)$/i.test(label)) {
      const grip = label.match(/^(attach to|detach from)\s+(.+)$/i)!;
      program.event(grip[1].toLowerCase().startsWith('attach') ? 'attach' : 'detach', null, grip[2].trim());
    } else if (/pause|wait/i.test(label)) {
      const v = parseFloat((label.match(/-?\d+(?:\.\d+)?/) ?? [])[0] ?? '0');
      program.pause(/ms/i.test(label) ? v : v * 1000);
    } else program.comment(label);
  }
}

function jointsFit(robot: Robot, joints: number[]): boolean {
  if (joints.length < robot.dof) return false;
  const { lower, upper } = robot.jointLimits();
  for (let i = 0; i < robot.dof; i++) {
    const v = joints[i];
    if (!Number.isFinite(v) || v < lower[i] - 0.5 || v > upper[i] + 0.5) return false;
  }
  return joints.slice(robot.dof).every((v) => Math.abs(v) < 1e-3);
}

/** When a padded joint vector fits several mechanisms, keep the one standing at the target. */
function nearestFit(fitted: Robot[], target: Item): Robot | null {
  if (fitted.length === 1) return fitted[0];
  if (!fitted.length) return null;
  const p = getPos(target.poseAbs());
  let best = fitted[0];
  let bestD = Infinity;
  for (const r of fitted) {
    const o = getPos(r.poseAbs());
    const d = Math.hypot(o[0] - p[0], o[1] - p[1], o[2] - p[2]);
    if (d < bestD) { best = r; bestD = d; }
  }
  return best;
}
function namedMechanism(robots: Robot[], label: string): Robot | null {
  const key = label.toLowerCase().replace(/[^a-z0-9]/g, '');
  const nums = [...label.matchAll(/\d+/g)].map((m) => m[0]);
  const skip = new Set(['open', 'close', 'move', 'home', 'base', 'robot', 'axis']);
  const hits = robots.filter((r) => {
    const words = r.name.toLowerCase().split(/[^a-z]+/).filter((w) => w.length >= 4 && !skip.has(w));
    if (!words.some((w) => key.includes(w))) return false;
    const rnums = [...r.name.matchAll(/\d+/g)].map((m) => m[0]);
    return !rnums.length || !nums.length || rnums.some((n) => nums.includes(n));
  });
  return hits.length === 1 ? hits[0] : null;
}

function stationFromFile(b: Uint8Array, fallbackName: string, assets?: AssetStore): Station | null {
  const dv = dvOf(b);
  const head = readUtf16(dv, b, 0);
  if (!head || !head.text.startsWith('Station File')) return null;
  const chunks = chunksOf(b);
  if (!chunks.length) return null;
  const station = new Station(fallbackName || 'RoboDK station');
  const stack: Array<{ depth: number; item: Item }> = [{ depth: -1, item: station }];
  const parentAt = (depth: number): Item => {
    while (stack.length > 1 && stack[stack.length - 1].depth >= depth) stack.pop();
    return stack[stack.length - 1].item;
  };
  const names: string[] = [];
  const programChunks: Array<{ program: Program; chunk: Chunk }> = [];
  let robots = 0, meshes = 0;
  for (const chunk of chunks) {
    const parent = parentAt(chunk.depth);
    if (chunk.kind === 1) {
      const title = firstText(b, chunk.payload, chunk.end, new Set(['lvl']));
      if (title) station.setName(title);
      stack.push({ depth: chunk.depth, item: station });
      names.push(station.name);
      continue;
    }
    let item: Item | null = null;
    if (chunk.kind === 3) {
      const frame = frameOf(b, chunk.payload, chunk.end);
      const f = new Frame(frame.name);
      f.setPose(frame.pose);
      item = f;
    } else if (chunk.kind === 2) {
      const at = findTable(b, chunk.payload, chunk.end, 4000);
      const robot = at >= 0 ? robotFromTable(b.subarray(at, chunk.end), 'Robot', assets) : null;
      if (robot) { item = robot; robots++; meshes += Number(robot.params.rdkMeshes ?? 0); }
    } else if (chunk.kind === 4 || chunk.kind === 5) {
      const at = findTable(b, chunk.payload, chunk.end, 1600);
      const obj = at >= 0 ? objectFromTable(b.subarray(at, chunk.end), chunk.kind === 4 ? 'Tool' : 'Object', chunk.kind === 4, assets) : null;
      if (obj) { item = obj; meshes += Number(obj.params.rdkMeshes ?? 0); }
    } else if (chunk.kind === 6) {
      item = targetFromChunk(b, chunk.payload, chunk.end);
    } else if (chunk.kind === 8) {
      const name = firstText(b, chunk.payload, chunk.end, new Set(['lvl', 'ins', 'S'])) ?? 'Program';
      const program = new Program(name);
      programChunks.push({ program, chunk });
      item = program;
    } else if (chunk.kind === 11) {
      const name = firstText(b, chunk.payload, chunk.end, new Set(['lvl', 'MachiningChkptStart', 'MachiningChkptEnd'])) ?? 'Machining';
      item = new Item(ItemType.MACHINING, name);
    }
    if (!item) continue;
    parent.addChild(item);
    stack.push({ depth: chunk.depth, item });
    names.push(item.name);
  }
  for (const { program, chunk } of programChunks) fillProgram(program, b, chunk.payload, chunk.end, station);
  const mechanisms = station.itemsOfType<Robot>(ItemType.ROBOT);
  const robot = mechanisms[0];
  for (const program of station.itemsOfType<Program>(ItemType.PROGRAM)) {
    const votes = new Map<string, { robot: Robot; n: number }>();
    for (const ins of program.instructions()) {
      const data = ins.data;
      if (data.kind !== 'move' || !data.targetId) continue;
      const target = station.findById(data.targetId);
      const joints = target instanceof Target ? target.joints : null;
      const fitted = joints ? mechanisms.filter((r) => jointsFit(r, joints)) : [];
      const named = namedMechanism(fitted, `${program.name} ${target?.name ?? ''}`);
      const siblings = target?.parent?.children ?? [];
      const owners = siblings.filter((c): c is Robot => c instanceof Robot);
      const owner = named ?? (target ? nearestFit(fitted, target) : null) ?? (owners.length === 1 ? owners[0] : null);
      if (!owner) continue;
      const cur = votes.get(owner.id);
      if (cur) cur.n += 1;
      else votes.set(owner.id, { robot: owner, n: 1 });
    }
    let best: Robot | null = null;
    let bestN = 0;
    for (const v of votes.values()) if (v.n > bestN) { best = v.robot; bestN = v.n; }
    if (best) program.setRobot(best);
    else if (!program.robotId && robot) program.setRobot(robot);
  }
  station.params.rdkSummary = { robots, programs: programChunks.length, meshes, items: names.length };
  station.params.rdkNames = names;
  return station;
}

function reportFor(bytes: number, compressed: RdkScanReport['compressed'], station: Station, notes: string[]): RdkScanReport {
  const names = (station.params.rdkNames as string[] | undefined) ?? [...station.walk()].map((i) => i.name).filter((n) => n && n !== station.name);
  const summary = station.params.rdkSummary as { robots: number; programs: number; meshes: number } | undefined;
  const meshes = summary?.meshes ?? 0;
  return {
    bytes,
    compressed,
    strings: names.slice(0, 80),
    poses: [...station.walk()].filter((i) => { const p = getPos(i.pose()); return p[0] !== 0 || p[1] !== 0 || p[2] !== 0; }).length,
    meshes: meshes ? [{ offset: 0, triangles: meshes }] : [],
    notes,
    fidelity: 'full',
  };
}

/** Full import of a .rdk / .robot / .tool / .sld buffer. Returns null when the bytes are not this format. */
export async function importRobodkBinary(data: ArrayBuffer, name = 'RoboDK import', assets?: AssetStore): Promise<RobodkLoad | null> {
  const { bytes, compressed } = await unwrapRobodk(data);
  const stationFile = stationFromFile(bytes, name.replace(/\.rdk$/i, ''), assets);
  if (stationFile) {
    const summary = stationFile.params.rdkSummary as { robots: number; programs: number; meshes: number };
    const notes = [`RoboDK station “${stationFile.name}”: ${summary.robots} robot(s), ${summary.programs} program(s), ${summary.meshes} meshes.`];
    const meshNote = [...stationFile.walk()].map((i) => i.params.rdkMeshNote).find((n) => typeof n === 'string' && n);
    if (meshNote) notes.push(String(meshNote));
    if (!summary.robots) notes.push('No robot parameter table was found in this station.');
    return { station: stationFile, report: reportFor(bytes.length, compressed, stationFile, notes) };
  }
  const dv = dvOf(bytes);
  const block = paramBlock(dv);
  if (!block) return null;
  const station = new Station(name);
  if (block.kind === 'robot') {
    const robot = robotFromTable(bytes, name, assets);
    if (!robot) return null;
    station.addChild(robot);
    station.params.rdkNames = [robot.name];
    station.params.rdkSummary = { robots: 1, programs: 0, meshes: Number(robot.params.rdkMeshes ?? 0) };
    const notes = [`RoboDK robot “${robot.name}”: ${robot.dof} DOF, post ${robot.postProcessor}, ${robot.params.rdkMeshes ?? 0} link meshes.`];
    return { station, report: reportFor(bytes.length, compressed, station, notes) };
  }
  const asTool = /\.tool$/i.test(name);
  const obj = objectFromTable(bytes, name, asTool, assets);
  if (!obj) return null;
  station.addChild(obj);
  station.params.rdkNames = [obj.name];
  station.params.rdkSummary = { robots: 0, programs: 0, meshes: Number(obj.params.rdkMeshes ?? 0) };
  const notes = [`RoboDK ${asTool ? 'tool' : 'object'} “${obj.name}”: ${obj.params.rdkMeshes ?? 0} meshes.`];
  return { station, report: reportFor(bytes.length, compressed, station, notes) };
}
