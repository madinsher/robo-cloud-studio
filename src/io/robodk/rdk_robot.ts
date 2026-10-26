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
import type { ChainDef, LinkVisual } from '../../core/kinematics/chain';
import { forwardKinematics, visualOrigin } from '../../core/kinematics/chain';
import { chainFromRobodk, rdkParamsFromTable } from '../../core/kinematics/rdk_model';
import { fromRows, getPos, identity, invert, multiply, rotx, transl } from '../../core/math/pose';
import type { Mat4 } from '../../core/math/pose';
import type { MeshData } from '../mesh/stl';
import type { AssetStore } from '../../scene/assets';
import { RdkBytes, tableChars } from './rdk_format';

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
  if (convention === 11 || /\bcomau\b/.test(n)) return 'Comau';
  const table: Array<[RegExp, RobotBrand]> = [[/universal robots|^ur\d|\bur\d+e?\b/, 'UR'], [/st(a|ä)ubli/, 'Staubli'], [/doosan/, 'Doosan'], [/kawasaki/, 'Kawasaki'], [/denso/, 'Denso'], [/mecademic|meca500/, 'Mecademic']];
  for (const [re, b] of table) if (re.test(n)) return b;
  return 'Generic';
}

/**
 * Post processor RoboDK uses for a robot whose file stores none (Item.getParam('PostProcessor') == ''), recognised
 * from the robot name / joint convention. RoboDK 5.9.4 then generates with its BUILT-IN generators (compiled into
 * RoboDK.exe, src/posts/rdk_builtin.ts): Library GT .LS of the Fanuc of 06.d / 02.d, .mod of the ABBs of 03.e / 05.c /
 * 06.b / 10.a, .src of the KUKAs of 08.a, .pdl of the Comau of 05.d, .script of the UR of 07.e
 * (tests/posts_library_code.test.ts). Mechanisms (turntables, rails, grippers) get RoboDK's "undefined post processor"
 * placeholder. Brands without a built-in generator verified against RoboDK keep their brand's post.
 */
export function defaultPostOf(name: string, convention: number, dof: number): string {
  const n = name.toLowerCase();
  const brand = brandOf(name, convention);
  const byBrand: Partial<Record<RobotBrand, string>> = {
    ABB: 'RoboDK_Builtin_ABB', Fanuc: 'RoboDK_Builtin_Fanuc', Yaskawa: 'Motoman', KUKA: 'RoboDK_Builtin_KUKA', Comau: 'RoboDK_Builtin_Comau', UR: 'RoboDK_Builtin_UR',
    Staubli: 'Staubli_VAL3', Doosan: 'Doosan_Robotics', Kawasaki: 'Kawasaki', Denso: 'Denso_RC8', Mecademic: 'Mecademic',
  };
  if (byBrand[brand] && dof >= 4) return byBrand[brand]!;
  const byName: Array<[RegExp, string]> = [
    [/techman|\btm\d/, 'Techman_TM'], [/\bepson\b/, 'Epson_RC'], [/\bnachi\b/, 'Nachi_AX_FD'], [/mitsubishi|\bmelfa\b/, 'Mitsubishi_Melfa'],
    [/\bdobot\b/, 'Dobot_CR'], [/hanwha|\bhcr-/, 'Hanwha_HCR'], [/\bjaka\b/, 'JAKA'], [/\baubo\b/, 'AUBO'], [/\belite\b/, 'Elite_EC'], [/kinova/, 'Kinova_Kortex'],
  ];
  if (dof >= 4) for (const [re, id] of byName) if (re.test(n)) return id;
  return 'RoboDK_Builtin_Generic';
}

export interface RdkRobotDecode {
  robot: Robot;
  /** Offset after the mesh section (start of the station-only trailer). */
  meshEnd: number;
  /** Things the importer could not reproduce exactly. */
  notes: string[];
}

/** The robot table as doubles (index 0 = the leading 1.0). */
export function robotTableAt(r: RdkBytes, at: number, count: number): Float64Array {
  const tab = new Float64Array(count);
  for (let i = 0; i < count; i++) tab[i] = r.f64(at + i * 8);
  return tab;
}

/**
 * Decode a robot table starting at `at` (doubles [0]=1, [1]=table length). The kinematics come from
 * `chainFromRobodk` (core/kinematics/rdk_model.ts, FK equal to RoboDK SolveFK for every Library robot): brand joint
 * senses, Fanuc/Comau J2-J3 coupling, base offset in `chain.base`, flange offset, delta robots, grippers.
 */
export function decodeRobotTable(r: RdkBytes, at: number, end: number, fallbackName: string, assets?: AssetStore): RdkRobotDecode | null {
  if (!r.ok(at, 32) || r.f64(at) !== 1) return null;
  const count = Math.round(r.f64(at + 8));
  if (count < 3000 || count > 20000 || !r.ok(at, count * 8)) return null;
  const tab = robotTableAt(r, at, count);
  const t = (i: number) => tab[i];
  const dof = Math.round(t(21));
  if (!(dof >= 1 && dof <= 12)) return null;
  const notes: string[] = [];
  const cls = Math.round(t(20)), conv = Math.round(t(61)), nMesh = Math.round(t(22));
  const name = tableChars(t, 1800) || fallbackName || 'Robot';
  const post = tableChars(t, 3800);
  const params = rdkParamsFromTable(tab, name);
  if (!params) return null;
  const chain = chainFromRobodk(params);
  const base = chain.base ?? identity();
  const delta = !!chain.parallel;
  const gripper = chain.flangeLink === 0 && dof === 1;
  if (gripper && cls === 15) gripperFinger2(chain);

  // ---- meshes: link 0 in the base-offset frame, link i in DHM frame i; delta: base, 3 upper arms (drawn from the
  // elbow), 6 lower-arm rods, platform, 4th-axis shaft; passive links (counterbalance, 4-bar fingers) ride on a link
  const meshAt = at + count * 8;
  const ml = readMeshLinks(r, meshAt, end);
  const meshEnd = ml?.next ?? end;
  let triangles = 0;
  let meshLinkOf: number[] | null = null;
  if (!ml) notes.push(`${name}: link mesh section could not be read`);
  else {
    if (ml.links.length !== nMesh) notes.push(`${name}: ${ml.links.length} mesh links stored, table says ${nMesh}`);
    const placeDelta = delta ? deltaVisuals(chain, tab) : null;
    let passive = 0, animated = 0;
    // counterbalance (table [1060..1065] or [1080..]: mesh A, mesh B (1-based), A = (ax, 0, ay) in link 1, B = (bx, by,
    // 0) in link 2): the cylinder hangs at A, the rod at B, both aimed from A to B with Y along the J2 axis (verified
    // against RoboDK's link poses: KR 90 / KR 100, IRB 6640 / 6700, R-2000iB). Comau's parallelogram rods differ: rigid.
    let look: { ma: number; mb: number; a: [number, number, number]; b: [number, number, number] } | null = null;
    for (const k of [1060, 1080]) {
      if (look || !(t(k) > dof && t(k + 1) > dof && t(k) < 32) || conv === 11 || dof < 3) continue;
      look = { ma: Math.round(t(k)) - 1, mb: Math.round(t(k + 1)) - 1, a: [t(k + 2), 0, t(k + 3)], b: [t(k + 4), t(k + 5), 0] };
    }
    meshLinkOf = [];
    ml.links.forEach((shapes, li) => {
      meshLinkOf![li] = li <= dof ? li : -1;
      if (!shapes.length) return;
      let link = li, origin: Mat4 = identity();
      if (placeDelta) ({ link, origin } = placeDelta(li));
      else if (gripper) {
        // body, finger 1, finger 2 (class 15); the 4-bar parts of a class 17 gripper stay on the body
        link = cls === 15 && li <= 2 ? li : 0;
        // the body mesh is stored in the robot base frame, not in the base-offset frame of the fingers (RoboDK 5.9.4:
        // ObjectLink(0) of the 06.f Hand-E grippers; probe shapes at its vertices collide only without the offset)
      } else if (li > dof) {
        // passive parts (counterbalance cylinder + rod, dress pack): carried rigidly by the link they hang on
        link = li === dof + 1 ? 1 : Math.min(2, dof);
        passive++;
      } else if (li === 0) origin = base;
      meshLinkOf![li] = link;
      const lookAt = look && (li === look.ma || li === look.mb) && !placeDelta && !gripper ? { at: li === look.ma ? 'a' as const : 'b' as const, a: look.a, linkA: 1, b: look.b, linkB: 2, axisLink: 2 } : undefined;
      if (lookAt) { link = lookAt.at === 'a' ? 1 : 2; meshLinkOf![li] = link; animated++; }
      for (const v of registerShapes(assets, name, shapes, `${name} link ${li}`)) {
        const vis: LinkVisual = { mesh: v.mesh, origin, color: v.color, meshLink: li };
        if (lookAt) { vis.passive = lookAt; vis.origin = visualOrigin(forwardKinematics(chain, Array.from({ length: dof }, (_, i) => t(700 + i))).linkPoses, link, vis); }
        chain.links[link].visuals.push(vis);
        triangles += v.triangles;
      }
    });
    if (passive > animated) notes.push(`${name}: ${passive - animated} passive mesh link(s) (counterbalance) attached rigidly to links 1/2, not animated`);
    if (delta) notes.push(`${name}: delta lower-arm rods are drawn rigidly at the import pose (not re-aimed when the robot moves)`);
    if (gripper && cls === 17) notes.push(`${name}: 4-bar finger linkage shown static at its build pose`);
  }
  const robot = new Robot(name, chain);
  robot.brand = brandOf(name, conv);
  robot.model = name;
  // the file's post, else RoboDK's choice for a robot without one (the brand's default post)
  robot.postProcessor = post || defaultPostOf(name, conv, dof);
  robot.params.rdkPostProcessor = post;
  // payload (kg): RoboDK's built-in ABB generator declares rdkTool with 0.8 x this mass
  if (Number.isFinite(t(100)) && t(100) > 0) robot.params.rdkPayload = t(100);
  const reach = t(101);
  if (reach > 50 && reach < 20000) robot.reach = reach;
  const cur: number[] = [];
  for (let i = 0; i < dof; i++) cur.push(t(700 + i));
  if (cur.every((v) => Number.isFinite(v) && Math.abs(v) < 1e6)) robot.setJoints(cur);
  if (params.motion) {
    robot.setMotionDefaults(params.motion);
    robot.params.rdkSpeeds = { linear: params.motion.speedLinear, linearAccel: params.motion.accelLinear, joints: params.motion.speedJoints, jointsAccel: params.motion.accelJoints };
  }
  robot.params.rdkClass = cls;
  robot.params.rdkConvention = conv;
  robot.params.rdkMeshes = triangles;
  robot.params.rdkMeshLinks = ml?.links.length ?? 0;
  // mesh link -> chain link (RoboDK collision map entities are mesh links; KR 90 R3100: 9 meshes on 7 links)
  if (meshLinkOf) robot.params.rdkMeshLinkOf = meshLinkOf;
  // passive-link definition (mesh link numbers + pivot data) kept for a later animated counterbalance
  for (const k of [1060, 1080]) if (t(k) > dof && t(k + 1) > dof && t(k) < 32) robot.params.rdkPassive = Array.from(tab.subarray(k, k + 6));
  if (conv === 3) {
    const pulses: number[] = [];
    for (let i = 0; i < dof; i++) pulses.push(t(980 + i));
    // RoboDK's Motoman posts multiply the joints by these ratios as stored, zeros included (Library 05.e: the ES200D has
    // none and RoboDK writes C00000=0,0,0,0,0,0)
    if (pulses.every((v) => Number.isFinite(v) && v >= 0)) robot.params.motomanPulses = pulses;
  }
  return { robot, meshEnd, notes };
}

/** Values of the external axes stored after the arm joints (synchronised rail / turntable / gantry). */
export function externalJointInfo(r: RdkBytes, at: number, dof: number, n: number): { lower: number[]; upper: number[]; home: number[]; current: number[] } {
  const t = (i: number) => r.f64(at + i * 8);
  const pick = (base: number) => Array.from({ length: n }, (_, k) => t(base + dof + k));
  return { lower: pick(600), upper: pick(620), home: pick(680), current: pick(700) };
}

/**
 * Second finger of a parallel two-finger gripper (RoboDK class 15, 1 DOF, joint sense 0.5): finger 1 travels +q/2
 * along the finger axis, finger 2 (same mesh, turned 180° about x) travels the other way. The flange stays on the
 * body (`flangeLink` 0, set by chainFromRobodk), so the extra mimic joint does not change SolveFK.
 */
function gripperFinger2(chain: ChainDef): void {
  const j = chain.joints[0];
  const k = j.axis[2];
  chain.joints.push({ name: 'J1b', type: j.type, origin: rotx(Math.PI), axis: [0, 0, 1], post: identity(), lower: j.lower, upper: j.upper, mimic: { joint: j.name, multiplier: 2 * k, offset: 0 } });
  chain.links.push({ name: 'link_1b', visuals: [] });
  chain.dof = 1;
}

/**
 * Delta (IRB 360) mesh placement. Mesh links: 0 base (base-offset frame), 1-3 upper arms (mesh frame at the elbow,
 * arm along -x), 4-9 lower-arm rods (frame at the elbow ball, rod along +x; two per arm), 10 platform, 11 4th-axis
 * shaft. The rods are aimed from the elbow at the platform for the current joints (the chain has no rod links).
 */
function deltaVisuals(chain: ChainDef, tab: Float64Array): (li: number) => { link: number; origin: Mat4 } {
  const d = chain.parallel!;
  const fk = forwardKinematics(chain, Array.from({ length: chain.dof ?? 3 }, (_, i) => tab[700 + i]));
  const B = chain.base ?? identity();
  const elbow = transl(d.upperArm, 0, 0);
  const half = 50; // half spacing of the parallelogram rods (mm), estimated from the meshes
  return (li) => {
    if (li === 0) return { link: 0, origin: B };
    if (li <= 3) return { link: li, origin: elbow };
    if (li <= 9) {
      const arm = Math.floor((li - 4) / 2), side = (li - 4) % 2 === 0 ? 1 : -1;
      const armPose = fk.linkPoses[arm + 1];
      const a = getPos(multiply(armPose, transl(d.upperArm, side * half, 0)));
      const ph = d.armAngles[arm] * Math.PI / 180;
      const pc = getPos(fk.linkPoses[4]);
      const c = Math.cos(ph), s = Math.sin(ph);
      const b = [pc[0] + d.platformRadius * c - side * half * s, pc[1] + d.platformRadius * s + side * half * c, pc[2]];
      const x = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
      const n = Math.hypot(x[0], x[1], x[2]) || 1;
      x[0] /= n; x[1] /= n; x[2] /= n;
      // y axis: the motor axis direction (tangent), made orthogonal to x
      let y = [-s, c, 0];
      const dp = y[0] * x[0] + y[1] * x[1];
      y = [y[0] - dp * x[0], y[1] - dp * x[1], y[2] - dp * x[2]];
      const ny = Math.hypot(y[0], y[1], y[2]) || 1;
      y = y.map((v) => v / ny);
      const z = [x[1] * y[2] - x[2] * y[1], x[2] * y[0] - x[0] * y[2], x[0] * y[1] - x[1] * y[0]];
      const world = fromRows([[x[0], y[0], z[0], a[0]], [x[1], y[1], z[1], a[1]], [x[2], y[2], z[2], a[2]], [0, 0, 0, 1]]);
      return { link: arm + 1, origin: multiply(invert(armPose), world) };
    }
    return { link: 4, origin: identity() };
  };
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
  // table [32..] is the item frame expressed in the geometry frame: the meshes sit at its inverse (verified against
  // RoboDK 5.9.4 collisions with probe shapes at mesh vertices: 06.c Europallet, 02.c Spindle, 01.a Paint gun)
  const origin = Array.from(invert(rd(32)));
  const sh = readShapes(r, at + count * 8, end);
  let o = sh?.next ?? end;
  if (!sh) notes.push(`${name}: mesh section could not be read`);
  else if (sh.shapes.length) {
    for (const v of registerShapes(assets, name, sh.shapes, name)) item.geometry.push({ mesh: v.mesh, origin: origin.slice(), color: v.color });
    const min: [number, number, number] = [Infinity, Infinity, Infinity], max: [number, number, number] = [-Infinity, -Infinity, -Infinity];
    // bounds in the item frame: the shape bounds (geometry frame) through the geometry origin
    for (const s of sh.shapes) for (let k = 0; k < 8; k++) {
      const c = [k & 1 ? s.max[0] : s.min[0], k & 2 ? s.max[1] : s.min[1], k & 4 ? s.max[2] : s.min[2]];
      for (let i = 0; i < 3; i++) {
        const v = origin[i] * c[0] + origin[4 + i] * c[1] + origin[8 + i] * c[2] + origin[12 + i];
        min[i] = Math.min(min[i], v); max[i] = Math.max(max[i], v);
      }
    }
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
  // keep the curves decoded before an unreadable one and say how many were lost
  const fail = (c: number) => ({ points, curves, next: o, note: `${name}: curve ${c + 1} of ${nc} stored in a layout that is not decoded (${curves.length} kept)` });
  for (let c = 0; c < nc; c++) {
    if (o + 8 > end) break;
    const n = r.u64(o);
    // the curve name is a string, or -1 when the curve has none
    const s = r.i32(o + 8) === -1 ? { text: '', next: o + 12 } : r.str(o + 8, 512);
    if (!s || /[\u0000-\u001f]/.test(s.text) || n > 10_000_000 || s.next + 16 + n * 52 > end) return fail(c);
    let p = s.next + 16;
    const pts: number[][] = [];
    for (let k = 0; k < n; k++, p += 52) {
      // point = f32 flag | xyz | normal | tangent | xyz (copy)
      const pt = [r.f32(p + 4), r.f32(p + 8), r.f32(p + 12), r.f32(p + 16), r.f32(p + 20), r.f32(p + 24)];
      if (!sane(pt[0]) || !sane(pt[1]) || !sane(pt[2])) return fail(c);
      // edges imported from STEP may carry no normal (NaN): keep the point with a zero (undefined) normal
      for (let i = 3; i < 6; i++) if (!sane(pt[i])) pt[i] = 0;
      pts.push(pt);
    }
    curves.push({ name: s.text || `Curve ${c + 1}`, points: pts });
    o = p;
  }
  return { points, curves, next: o };
}

