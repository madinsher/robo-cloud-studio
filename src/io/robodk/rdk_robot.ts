/**
 * RoboDK robot / mechanism and object / tool parameter tables (the body of .robot, .tool, .sld files and of the
 * robot, tool and object chunks of a .rdk station).
 *
 * Robot table: 4000 doubles, then the link meshes. Indices verified against RoboDK 5.9.4 (`SolveFK`, `JointLimits`,
 * `JointsHome` of every robot in tests/fixtures/robodk_gt):
 *   [20] kinematic class (1 serial 6-axis, 3 7-axis, 5 delta, 6 UR-like cobot, 8 Denso, 15 two-finger gripper,
 *        17 4-bar gripper, 0 1-3 axis mechanism), [21] DOF, [22] mesh link count, [61] joint convention
 *        (0 generic, 1 ABB, 2 Fanuc, 3 Motoman, 5 KUKA, 7 explicit senses, 11 Comau, 12 Mecademic/JAKA),
 *   [64+i] joint sense (0 = brand default), [101] reach, [120..124] default speeds,
 *   [180..185] base offset xyz + rx ry rz (rad), [200+20i] DHM row: alpha (rad), a, theta (rad), d, type (1 prismatic),
 *   [560..565] flange offset, [600+i]/[620+i] joint limits (incl. synchronised external axes),
 *   [680+i] home joints, [700+i] current joints, [980+i] Motoman pulses per unit, [1800] name, [3800] post processor.
 * FK(q) = pose6(180) · Π DHM_i(sense_i · q_i [+ q_2 for J3 when the convention couples J2/J3]) · pose6(560).
 *
 * Mesh section: u64 nLinks; per link: (link > 0: two zero u64) u64 nShapes; shape = u64 nTri, RGBA f32,
 * nTri·3 × (xyz f32 + 2 bytes). Link 0 is drawn in the base-offset frame, link i in DHM frame i.
 */
import { Robot, RobotBrand } from '../../core/items/robot';
import { SceneObject, Tool } from '../../core/items/item';
import type { ChainDef, JointDef, LinkDef } from '../../core/kinematics/chain';
import { identity, invert, multiply, mul, rotx, rotz, transl } from '../../core/math/pose';
import type { Mat4 } from '../../core/math/pose';
import type { MeshData } from '../mesh/stl';
import type { AssetStore } from '../../scene/assets';
import { RdkBytes, pose6, tableChars } from './rdk_format';

let meshSeq = 0;

export interface RdkShape { positions: Float32Array; triangles: number; color: string; min: [number, number, number]; max: [number, number, number] }

function hexColor(r: number, g: number, b: number): string {
  const h = (v: number) => Math.max(0, Math.min(255, Math.round(v * 255))).toString(16).padStart(2, '0');
  return `#${h(r)}${h(g)}${h(b)}`;
}

/** One shape at `o`: u64 nTri, RGBA, vertices. Returns null when the record is implausible. */
function readShape(r: RdkBytes, o: number, end: number): { shape: RdkShape; next: number } | null {
  if (o + 24 > end) return null;
  const nt = r.u64(o);
  if (nt > 50_000_000 || o + 24 + nt * 42 > end) return null;
  const cr = r.f32(o + 8), cg = r.f32(o + 12), cb = r.f32(o + 16);
  const positions = new Float32Array(nt * 9);
  const min: [number, number, number] = [Infinity, Infinity, Infinity];
  const max: [number, number, number] = [-Infinity, -Infinity, -Infinity];
  let p = o + 24;
  for (let i = 0, k = 0; i < nt * 3; i++, p += 14) {
    const x = r.f32(p), y = r.f32(p + 4), z = r.f32(p + 8);
    positions[k++] = x; positions[k++] = y; positions[k++] = z;
    if (x < min[0]) min[0] = x; if (x > max[0]) max[0] = x;
    if (y < min[1]) min[1] = y; if (y > max[1]) max[1] = y;
    if (z < min[2]) min[2] = z; if (z > max[2]) max[2] = z;
  }
  const ok = (v: number) => Number.isFinite(v) ? v : 0.6;
  return { shape: { positions, triangles: nt, color: hexColor(ok(cr), ok(cg), ok(cb)), min, max }, next: p };
}

/** Robot mesh section: one shape list per mesh link. */
export function readMeshLinks(r: RdkBytes, o: number, end: number): { links: RdkShape[][]; next: number } | null {
  if (o + 8 > end) return null;
  const n = r.u64(o);
  if (n < 1 || n > 64) return null;
  o += 8;
  const links: RdkShape[][] = [];
  for (let l = 0; l < n; l++) {
    if (l > 0) o += 16;
    if (o + 8 > end) return null;
    const ns = r.u64(o);
    o += 8;
    if (ns > 1_000_000) return null;
    const shapes: RdkShape[] = [];
    for (let s = 0; s < ns; s++) {
      const one = readShape(r, o, end);
      if (!one) return null;
      shapes.push(one.shape);
      o = one.next;
    }
    links.push(shapes);
  }
  return { links, next: o };
}

/** Object / tool mesh section: u64 nShapes then shapes. */
export function readShapes(r: RdkBytes, o: number, end: number): { shapes: RdkShape[]; next: number } | null {
  if (o + 8 > end) return null;
  const ns = r.u64(o);
  if (ns > 1_000_000) return null;
  o += 8;
  const shapes: RdkShape[] = [];
  for (let s = 0; s < ns; s++) {
    const one = readShape(r, o, end);
    if (!one) return null;
    shapes.push(one.shape);
    o = one.next;
  }
  return { shapes, next: o };
}

/** Merge the shapes of one link / object that share a colour into one mesh (one draw call per colour). */
function mergeByColor(shapes: RdkShape[]): RdkShape[] {
  const groups = new Map<string, RdkShape[]>();
  for (const s of shapes) { if (!s.triangles) continue; const g = groups.get(s.color); if (g) g.push(s); else groups.set(s.color, [s]); }
  const out: RdkShape[] = [];
  for (const [color, list] of groups) {
    if (list.length === 1) { out.push(list[0]); continue; }
    const tri = list.reduce((n, s) => n + s.triangles, 0);
    const positions = new Float32Array(tri * 9);
    const min: [number, number, number] = [Infinity, Infinity, Infinity];
    const max: [number, number, number] = [-Infinity, -Infinity, -Infinity];
    let k = 0;
    for (const s of list) {
      positions.set(s.positions, k); k += s.positions.length;
      for (let i = 0; i < 3; i++) { min[i] = Math.min(min[i], s.min[i]); max[i] = Math.max(max[i], s.max[i]); }
    }
    out.push({ positions, triangles: tri, color, min, max });
  }
  return out;
}

function toMesh(s: RdkShape): MeshData {
  return { positions: s.positions, normals: new Float32Array(0), min: s.min, max: s.max, triangles: s.triangles };
}

function registerShapes(assets: AssetStore | undefined, name: string, shapes: RdkShape[], label: string): Array<{ mesh: string; color: string; triangles: number }> {
  const out: Array<{ mesh: string; color: string; triangles: number }> = [];
  for (const s of mergeByColor(shapes)) {
    meshSeq += 1;
    const id = `rdk:${meshSeq}:${name}`;
    assets?.registerMesh(id, toMesh(s), label);
    out.push({ mesh: id, color: s.color, triangles: s.triangles });
  }
  return out;
}

function brandOf(name: string, convention: number): RobotBrand {
  const n = name.toLowerCase();
  if (convention === 1 || /\babb\b/.test(n)) return 'ABB';
  if (convention === 2 || /fanuc/.test(n)) return 'Fanuc';
  if (convention === 3 || /motoman|yaskawa/.test(n)) return 'Yaskawa';
  if (convention === 5 || /\bkuka\b/.test(n)) return 'KUKA';
  const table: Array<[RegExp, RobotBrand]> = [[/universal robots|^ur\d|\bur\d+e?\b/, 'UR'], [/st(a|ä)ubli/, 'Staubli'], [/doosan/, 'Doosan'], [/kawasaki/, 'Kawasaki'], [/denso/, 'Denso'], [/mecademic|meca500/, 'Mecademic']];
  for (const [re, b] of table) if (re.test(n)) return b;
  return 'Generic';
}

/** Joint senses implied by the brand convention when the table leaves [64+i] at 0. */
const SENSES: Record<number, number[]> = { 2: [1, 1, -1, -1, -1, -1], 3: [1, 1, -1, -1, -1, -1], 5: [-1, 1, 1, -1, 1, -1] };

export interface RdkRobotDecode {
  robot: Robot;
  /** Offset after the mesh section (start of the station-only trailer). */
  meshEnd: number;
  /** Things the importer could not reproduce exactly. */
  notes: string[];
}

/** Decode a robot table starting at `at` (doubles [0]=1, [1]=table length). */
export function decodeRobotTable(r: RdkBytes, at: number, end: number, fallbackName: string, assets?: AssetStore): RdkRobotDecode | null {
  if (!r.ok(at, 32) || r.f64(at) !== 1) return null;
  const count = Math.round(r.f64(at + 8));
  if (count < 3000 || count > 20000 || !r.ok(at, count * 8)) return null;
  const t = (i: number) => r.f64(at + i * 8);
  const dof = Math.round(t(21));
  if (!(dof >= 1 && dof <= 12)) return null;
  const notes: string[] = [];
  const cls = Math.round(t(20)), conv = Math.round(t(61)), nMesh = Math.round(t(22));
  const name = tableChars(t, 1800) || fallbackName || 'Robot';
  const post = tableChars(t, 3800);
  const base = pose6(t(180), t(181), t(182), t(183), t(184), t(185));
  const flange = pose6(t(560), t(561), t(562), t(563), t(564), t(565));
  const sense = (i: number) => { const s = t(64 + i); return s !== 0 && Number.isFinite(s) ? s : (SENSES[conv]?.[i] ?? 1); };
  const lim = (i: number, lo: number) => { const v = t(lo + i); return Number.isFinite(v) ? v : 0; };
  const joints: JointDef[] = [];
  for (let i = 0; i < dof; i++) {
    const row = 200 + 20 * i;
    const alpha = t(row), a = t(row + 1), theta = t(row + 2), d = t(row + 3);
    const prismatic = Math.round(t(row + 4)) === 1;
    const origin = mul(rotx(alpha), transl(a, 0, 0), rotz(theta));
    joints.push({
      name: `J${i + 1}`, type: prismatic ? 'prismatic' : 'revolute',
      origin: i === 0 ? multiply(base, origin) : origin, axis: [0, 0, sense(i)], post: transl(0, 0, d),
      lower: lim(i, 600), upper: lim(i, 620), home: lim(i, 680), maxVelocity: prismatic ? 1000 : 180,
    });
  }
  // Fanuc and Comau measure J3 from the horizontal: the DH angle of J3 also turns with J2.
  if ((conv === 2 || conv === 11) && dof >= 3) joints[2].couple = { joint: 'J2', multiplier: 1 };
  const links: LinkDef[] = [{ name: 'base', visuals: [] }];
  for (let i = 0; i < dof; i++) links.push({ name: `link_${i + 1}`, visuals: [] });
  const chain: ChainDef = { name, joints, links, flange, dof };
  const gripper = cls === 15 || cls === 17;
  if (gripper && dof === 1) gripperChain(chain);
  if (cls === 5) notes.push(`${name}: delta (parallel) kinematics are not supported by the chain model; FK differs from RoboDK`);

  // ---- meshes
  const meshAt = at + count * 8;
  const ml = readMeshLinks(r, meshAt, end);
  let meshEnd = ml?.next ?? end;
  let triangles = 0;
  if (!ml) notes.push(`${name}: link mesh section could not be read`);
  else {
    if (ml.links.length !== nMesh) notes.push(`${name}: ${ml.links.length} mesh links stored, table says ${nMesh}`);
    ml.links.forEach((shapes, li) => {
      if (!shapes.length) return;
      let link = li, origin: Mat4 = identity();
      if (gripper && dof === 1) {
        // body, finger 1, finger 2 (cls 15); the 4-bar parts of a cls 17 gripper stay on the body
        if (cls === 15 && li <= 2) link = li; else link = 0;
        if (link === 0) origin = base;
      } else if (li > dof) {
        // passive parts (counterbalance cylinder + rod, dress pack): carried rigidly by the link they hang on
        link = li === dof + 1 ? 1 : Math.min(2, dof);
        if (li === dof + 1) notes.push(`${name}: ${ml.links.length - dof - 1} passive mesh link(s) attached rigidly (not animated)`);
      } else if (li === 0) origin = base;
      for (const v of registerShapes(assets, name, shapes, `${name} link ${li}`)) {
        chain.links[link].visuals.push({ mesh: v.mesh, origin, color: v.color });
        triangles += v.triangles;
      }
    });
  }
  const robot = new Robot(name, chain);
  robot.brand = brandOf(name, conv);
  robot.model = name;
  if (post) robot.postProcessor = post;
  const reach = t(101);
  if (reach > 50 && reach < 20000) robot.reach = reach;
  const cur: number[] = [];
  for (let i = 0; i < dof; i++) cur.push(t(700 + i));
  if (cur.every((v) => Number.isFinite(v) && Math.abs(v) < 1e6)) robot.setJoints(cur);
  robot.params.rdkClass = cls;
  robot.params.rdkConvention = conv;
  robot.params.rdkMeshes = triangles;
  robot.params.rdkMeshLinks = ml?.links.length ?? 0;
  const speeds = { linear: t(120), linearAccel: t(121), joints: t(123), jointsAccel: t(124) };
  if (Object.values(speeds).every((v) => Number.isFinite(v) && v > 0)) robot.params.rdkSpeeds = speeds;
  if (conv === 3) {
    const pulses: number[] = [];
    for (let i = 0; i < dof; i++) pulses.push(t(980 + i));
    if (pulses.some((v) => v > 1)) robot.params.motomanPulses = pulses;
  }
  if (gripper && dof === 1 && cls === 17) notes.push(`${name}: 4-bar finger linkage shown static at its build pose`);
  return { robot, meshEnd, notes };
}

/** Values of the external axes stored after the arm joints (synchronised rail / turntable / gantry). */
export function externalJointInfo(r: RdkBytes, at: number, dof: number, n: number): { lower: number[]; upper: number[]; home: number[]; current: number[] } {
  const t = (i: number) => r.f64(at + i * 8);
  const pick = (base: number) => Array.from({ length: n }, (_, k) => t(base + dof + k));
  return { lower: pick(600), upper: pick(620), home: pick(680), current: pick(700) };
}

/**
 * Parallel two-finger gripper (RoboDK class 15/17, 1 DOF, joint sense 0.5): finger 1 travels +q/2 along the
 * finger axis, finger 2 (same mesh, turned 180° about x) travels the other way, and the flange stays put —
 * RoboDK's SolveFK of a gripper is the constant flange offset.
 */
function gripperChain(chain: ChainDef): void {
  const j = chain.joints[0];
  const k = j.axis[2];
  chain.joints.push({ name: 'J1b', type: j.type, origin: rotx(Math.PI), axis: [0, 0, 1], post: identity(), lower: j.lower, upper: j.upper, mimic: { joint: 'J1', multiplier: 2 * k, offset: 0 } });
  chain.links.push({ name: 'link_1b', visuals: [] });
  // Undo both finger motions so the last link (and the flange) does not move with the joint.
  chain.joints.push({ name: 'J1c', type: j.type, origin: identity(), axis: [0, 0, -1], post: identity(), lower: j.lower, upper: j.upper, mimic: { joint: 'J1', multiplier: k, offset: 0 } });
  chain.links.push({ name: 'link_1c', visuals: [] });
  // link_1c at any q = base · origin1 · post1 · Rx(pi): the flange is expressed relative to it.
  const fixed = mul(j.origin, j.post ?? identity(), rotx(Math.PI));
  chain.flange = multiply(invert(fixed), chain.flange);
  chain.dof = 1;
}

/** Object / tool table: 1600 doubles (name [1440], geometry origin [32], visible [48], pose [96], TCP [160]). */
export interface RdkObjectDecode { item: SceneObject; end: number; curves: number; points: number; notes: string[] }

export function decodeObjectTable(r: RdkBytes, at: number, end: number, fallbackName: string, asTool: boolean, assets?: AssetStore): RdkObjectDecode | null {
  if (!r.ok(at, 32) || r.f64(at) !== 1) return null;
  const count = Math.round(r.f64(at + 8));
  if (count < 200 || count > 3000 || !r.ok(at, count * 8)) return null;
  const t = (i: number) => r.f64(at + i * 8);
  const notes: string[] = [];
  // A tool chunk without the tool flag [50] whose header holds uninitialised memory is loaded by RoboDK as an
  // unnamed object (Example 05.b "Tool RDK Ry15").
  let invalidTool = false;
  if (asTool && t(50) !== 1) for (let i = 4; i < 16; i++) if (!Number.isFinite(t(i))) { invalidTool = true; break; }
  const name = invalidTool ? '' : tableChars(t, 1440) || fallbackName;
  if (invalidTool) { asTool = false; notes.push(`tool chunk "${tableChars(t, 1440)}" has a corrupted table: imported as an unnamed object like RoboDK`); }
  const item = asTool ? new Tool(name) : new SceneObject(name);
  const rd = (i: number) => r.pose(at + i * 8);
  if (asTool) item.setPose(rd(160));
  else item.setPose(rd(96));
  if (t(48) === 0) item.setVisible(false);
  const origin = Array.from(rd(32));
  const sh = readShapes(r, at + count * 8, end);
  let o = sh?.next ?? end;
  if (!sh) notes.push(`${name}: mesh section could not be read`);
  else if (sh.shapes.length) {
    for (const v of registerShapes(assets, name, sh.shapes, name)) item.geometry.push({ mesh: v.mesh, origin: origin.slice(), color: v.color });
    const min: [number, number, number] = [Infinity, Infinity, Infinity], max: [number, number, number] = [-Infinity, -Infinity, -Infinity];
    for (const s of sh.shapes) for (let i = 0; i < 3; i++) { min[i] = Math.min(min[i], s.min[i]); max[i] = Math.max(max[i], s.max[i]); }
    item.bbox = { min, max };
    item.params.rdkMeshes = sh.shapes.reduce((n, s) => n + s.triangles, 0);
  }
  // ---- point and curve features (object coordinates)
  let curves = 0, points = 0;
  if (sh && o + 16 <= end) {
    const feat = readFeatures(r, o, end, name);
    if (feat) {
      for (const p of feat.points) item.points.push(p);
      for (const c of feat.curves) item.curves.push(c);
      points = feat.points.length; curves = feat.curves.length;
      if (feat.note) notes.push(feat.note);
      o = feat.next;
    }
  }
  return { item, end: o, curves, points, notes };
}

/**
 * Features after an object's shapes:
 *   u64 nPoints, point = i32 -1, RGBA, xyz, 3 f32, normal ijk, f32 size (60 bytes)
 *   u64 nCurves, curve = u64 n, str name, RGBA, n × (f32 flag, xyz, normal, tangent, 3 f32) (52 bytes)
 * Point clouds stored as named point sets (Example 09.b) are not decoded.
 */
function readFeatures(r: RdkBytes, o: number, end: number, name: string): { points: SceneObject['points']; curves: SceneObject['curves']; next: number; note?: string } | null {
  const np = r.u64(o);
  const points: SceneObject['points'] = [];
  const curves: SceneObject['curves'] = [];
  o += 8;
  if (np > 0) {
    // point = name (string, or -1 when unnamed) | RGBA | xyz | xyz | normal ijk | f32 size
    if (np > 20_000_000 || o + np * 60 > end) return { points, curves, next: o, note: `${name}: ${np} points stored in a layout that is not decoded` };
    for (let k = 0; k < np; k++) {
      const s = r.i32(o) === -1 ? { text: '', next: o + 4 } : r.str(o, 512);
      if (!s || s.next + 56 > end) return { points: [], curves, next: o, note: `${name}: ${np} points stored in a layout that is not decoded` };
      const p = s.next + 16;
      points.push({ name: s.text || `Point ${k + 1}`, point: [r.f32(p), r.f32(p + 4), r.f32(p + 8), r.f32(p + 24), r.f32(p + 28), r.f32(p + 32)] });
      o = p + 40;
    }
  }
  if (o + 8 > end) return { points, curves, next: o };
  const nc = r.u64(o);
  if (nc > 100000) return { points, curves, next: o };
  o += 8;
  const sane = (v: number) => Number.isFinite(v) && Math.abs(v) < 1e7;
  const fail = { points, curves: [] as SceneObject['curves'], next: o, note: `${name}: ${nc} curve(s) stored in a layout that is not decoded` };
  for (let c = 0; c < nc; c++) {
    if (o + 8 > end) break;
    const n = r.u64(o);
    // the curve name is a string, or -1 when the curve has none
    const s = r.i32(o + 8) === -1 ? { text: '', next: o + 12 } : r.str(o + 8, 512);
    if (!s || /[\u0000-\u001f]/.test(s.text) || n > 10_000_000 || s.next + 16 + n * 52 > end) return fail;
    let p = s.next + 16;
    const pts: number[][] = [];
    for (let k = 0; k < n; k++, p += 52) {
      const pt = [r.f32(p + 4), r.f32(p + 8), r.f32(p + 12), r.f32(p + 16), r.f32(p + 20), r.f32(p + 24)];
      if (!pt.every(sane)) return fail;
      pts.push(pt);
    }
    curves.push({ name: s.text || `Curve ${c + 1}`, points: pts });
    o = p;
  }
  return { points, curves, next: o };
}

