import type { GeometryRef } from '../items/item';
import { Mat4, Vec3, identity, multiply, rotAxis, transl, clone, invert, getPos, cross, sub, transformDir, mul, rotz, roty, rotx, DEG } from '../math/pose';

export type JointType = 'revolute' | 'prismatic' | 'fixed' | 'continuous';

export interface JointDef {
  name: string;
  type: JointType;
  /** Static transform from previous link frame to this joint frame (before motion). */
  origin: Mat4;
  /**
   * Motion axis in the joint frame. Its sign is the joint sense (RoboDK "joint senses": KUKA A1/A4/A6, Fanuc and
   * Yaskawa J3..J6 run against the DH axis): rotation by `value` deg about `axis`, i.e. by -value about -axis.
   * A revolute axis is normalised; a prismatic axis may be scaled (gripper fingers travel half the opening).
   */
  axis: Vec3;
  /** Static transform applied after the motion (used by DH-defined robots). */
  post?: Mat4;
  /** Limits (degrees for revolute, mm for prismatic). */
  lower: number;
  upper: number;
  /** Max velocity (deg/s or mm/s) and acceleration (deg/s² or mm/s²). */
  maxVelocity?: number;
  maxAcceleration?: number;
  /** Mimic another joint: q = multiplier * q_src + offset. */
  mimic?: { joint: string; multiplier: number; offset: number };
  /**
   * This joint's motion also depends on another actuated joint:
   * angle = q + multiplier * q_src, then about `axis` (which already carries the joint sense).
   * Comau parallelogram: joint 3 tracks joint 2 (axis -z, multiplier -s2/s3).
   * Fanuc J2/J3 interaction (RoboDK jointsType 2): θ3 = -q3 - q2, i.e. axis [0,0,-1], couple { J2, multiplier 1 }.
   */
  couple?: { joint: string; multiplier: number };
  /** Home value (degrees / mm). */
  home?: number;
}

export interface LinkVisual {
  /** Mesh id in the asset registry, or primitive description. */
  mesh?: string;
  primitive?: { kind: 'box'; size: Vec3 } | { kind: 'cylinder'; radius: number; length: number } | { kind: 'sphere'; radius: number };
  /** Local transform of the visual relative to the link frame. */
  origin: Mat4;
  color?: string;
  scale?: Vec3;
  /** RoboDK mesh link this visual came from (collision map entity of imported robots; passive links ride on a chain link). */
  meshLink?: number;
  /**
   * Passive "look-at" part (counterbalance cylinder / rod): its frame sits at pivot `a` (a point of link `linkA`) or
   * `b` (a point of link `linkB`), its X axis points from a to b and its Y axis is the Z axis of link `axisLink`
   * (the J2 axis). The visual hangs on the link it is attached to; `visualOrigin()` gives its current origin.
   */
  passive?: PassiveLookAt;
}

export interface PassiveLookAt { at: 'a' | 'b'; a: Vec3; linkA: number; b: Vec3; linkB: number; axisLink: number }

/**
 * Origin of a link visual relative to its link for the current link poses (`fk.linkPoses`): `v.origin`, or for a
 * passive look-at part the frame described by `v.passive` (RoboDK counterbalances: verified against RoboDK's link
 * poses of KUKA KR 90 / KR 100, ABB IRB 6640 / 6700, Fanuc R-2000iB at several J2 / J3 values).
 */
export function visualOrigin(linkPoses: Mat4[], link: number, v: LinkVisual): Mat4 {
  const p = v.passive;
  if (!p || !linkPoses[p.linkA] || !linkPoses[p.linkB] || !linkPoses[p.axisLink] || !linkPoses[link]) return v.origin;
  const at = (m: Mat4, q: Vec3): Vec3 => [m[0] * q[0] + m[4] * q[1] + m[8] * q[2] + m[12], m[1] * q[0] + m[5] * q[1] + m[9] * q[2] + m[13], m[2] * q[0] + m[6] * q[1] + m[10] * q[2] + m[14]];
  const A = at(linkPoses[p.linkA], p.a), B = at(linkPoses[p.linkB], p.b);
  const ax = linkPoses[p.axisLink];
  let x: Vec3 = [B[0] - A[0], B[1] - A[1], B[2] - A[2]];
  const lx = Math.hypot(x[0], x[1], x[2]);
  if (lx < 1e-9) return v.origin;
  x = [x[0] / lx, x[1] / lx, x[2] / lx];
  const y0: Vec3 = [ax[8], ax[9], ax[10]];
  let z: Vec3 = [x[1] * y0[2] - x[2] * y0[1], x[2] * y0[0] - x[0] * y0[2], x[0] * y0[1] - x[1] * y0[0]];
  const lz = Math.hypot(z[0], z[1], z[2]) || 1;
  z = [z[0] / lz, z[1] / lz, z[2] / lz];
  const y: Vec3 = [z[1] * x[2] - z[2] * x[1], z[2] * x[0] - z[0] * x[2], z[0] * x[1] - z[1] * x[0]];
  const o = p.at === 'a' ? A : B;
  const F = Float64Array.from([x[0], x[1], x[2], 0, y[0], y[1], y[2], 0, z[0], z[1], z[2], 0, o[0], o[1], o[2], 1]) as Mat4;
  return multiply(invert(linkPoses[link]), F);
}

export interface LinkDef {
  name: string;
  visuals: LinkVisual[];
  /**
   * Collision geometry declared by the source separately from the visuals (Webots boundingObject, URDF <collision>),
   * origin relative to the link frame (16 numbers, column-major, or a Mat4). When present it is what collision checks
   * use for this link instead of the visual meshes.
   */
  collisions?: GeometryRef[];
}

/**
 * Parallel delta robot (ABB IRB 360 / FlexPicker, RoboDK robot class 5). The chain's joints are
 * [J1, J2, J3, J4 | fixed 'platform']: three motor joints and either the platform rotation (4-axis delta)
 * or a fixed joint. Links are [base, upper arm 1, upper arm 2, upper arm 3, platform].
 * Frames: chain.base maps the robot base frame to the motor plane (z up). Arm i hangs at
 * Rz(armAngles[i]) · T(baseRadius, 0, 0) and turns about that frame's +Y axis by q_i (positive = down).
 * The lower arms (length lowerArm) end at platformRadius from the platform centre, whose frame is
 * T(P) · platform · Rz(q4) (P = platform centre in the motor frame), followed by chain.flange.
 */
export interface DeltaDef {
  kind: 'delta';
  /** Distance from the motor-plane centre to each motor axis (mm). RoboDK: base triangle side / (2√3). */
  baseRadius: number;
  /** Distance from the platform centre to each lower-arm joint line (mm). RoboDK: platform triangle side / (2√3). */
  platformRadius: number;
  upperArm: number;
  lowerArm: number;
  /** Direction of each arm about the motor-plane Z (deg). RoboDK IRB 360: [0, -120, -240] (clockwise). */
  armAngles: [number, number, number];
  /** Platform orientation relative to the motor plane, applied before the 4th axis. Default identity. */
  platform?: Mat4;
}

/** A serial kinematic chain: base -> joint_1 -> link_1 -> ... -> joint_n -> link_n -> flange. */
export interface ChainDef {
  name: string;
  joints: JointDef[];
  /** Links; links[0] is the base link (attached before any joint). links[i] follows joints[i-1]. */
  links: LinkDef[];
  /** Transform from the last link to the mechanical flange. */
  flange: Mat4;
  /**
   * Fixed transform from the robot base frame (the item pose, where links[0] is drawn) to the first joint frame.
   * RoboDK robot "base offset" (Fanuc/Yaskawa/Stäubli base frame at J2 height, JAKA/iiwa rotated 180°). Default identity.
   */
  base?: Mat4;
  /**
   * Link the flange is attached to (default: the last link). RoboDK grippers keep the flange on the base (links[0])
   * while the fingers move: flange = linkPoses[flangeLink] · flange.
   */
  flangeLink?: number;
  /** Closed-chain kinematics; when set, FK and IK use this model instead of the serial joint list. */
  parallel?: DeltaDef;
  /** Optional per-joint DH parameters (retained for RoboDK export). */
  dh?: DHParams[];
  /** Number of actuated joints (mimic/fixed excluded). */
  dof?: number;
  /** RoboDK joint convention code (robot table [61], JointLimits()[2]): 0 generic/UR, 1 ABB, 2 Fanuc, 3 Yaskawa, 5 KUKA, 7 table senses, 11 Comau, 12 table senses. */
  jointsType?: number;
}

export interface DHParams {
  /** Standard DH: theta offset (deg), d (mm), a (mm), alpha (deg). */
  theta: number;
  d: number;
  a: number;
  alpha: number;
  /** 'revolute' | 'prismatic' */
  type?: JointType;
  lower?: number;
  upper?: number;
  maxVelocity?: number;
  home?: number;
  /** Direction sign for joint value (some brands use inverted axes). */
  sign?: 1 | -1;
}

/** Build a chain from standard Denavit-Hartenberg parameters. */
export function chainFromDH(name: string, dh: DHParams[], flange: Mat4 = identity()): ChainDef {
  const joints: JointDef[] = dh.map((p, i) => {
    const type = p.type ?? 'revolute';
    const post = mul(rotz(p.theta * DEG), transl(0, 0, type === 'prismatic' ? 0 : p.d), transl(p.a, 0, 0), rotx(p.alpha * DEG));
    // For prismatic joints, d is the variable: motion along Z applies d offset too.
    const origin = type === 'prismatic' ? transl(0, 0, p.d) : identity();
    return {
      name: `J${i + 1}`,
      type,
      origin,
      axis: [0, 0, p.sign ?? 1] as Vec3,
      post,
      lower: p.lower ?? (type === 'prismatic' ? 0 : -180),
      upper: p.upper ?? (type === 'prismatic' ? 1000 : 180),
      maxVelocity: p.maxVelocity ?? (type === 'prismatic' ? 1000 : 180),
      home: p.home ?? 0,
    };
  });
  const links: LinkDef[] = [{ name: 'base', visuals: [] }];
  for (let i = 0; i < dh.length; i++) links.push({ name: `link_${i + 1}`, visuals: [] });
  return { name, joints, links, flange: clone(flange), dh: dh.map((d) => ({ ...d })), dof: joints.filter((j) => j.type !== 'fixed').length };
}

// -- Cached structural info (index maps) -------------------------------------------------------------

interface ChainInfo {
  /** Snapshot used to detect a rebuilt joint list. */
  joints: JointDef[];
  n: number;
  idx: number[];
  /** For every joint: index into q (actuated) or -1. */
  qIndex: Int32Array;
  mimicSrc: Int32Array;
  coupleSrc: Int32Array;
  refs: Array<{ j: JointDef; type: JointType; mimic: JointDef['mimic']; couple: JointDef['couple'] }>;
}

const infoCache = new WeakMap<ChainDef, ChainInfo>();

function sameJoints(a: ChainInfo, chain: ChainDef): boolean {
  if (a.joints !== chain.joints || a.n !== chain.joints.length) return false;
  // joints edited in place (type, mimic or couple replaced) invalidate the maps
  for (let i = 0; i < a.n; i++) {
    const j = chain.joints[i], r = a.refs[i];
    if (r.j !== j || r.type !== j.type || r.mimic !== j.mimic || r.couple !== j.couple) return false;
  }
  return true;
}

/** Index maps of a chain (cached; call invalidateChain after editing joint types, mimics or couples in place). */
export function chainInfo(chain: ChainDef): ChainInfo {
  const c = infoCache.get(chain);
  if (c && sameJoints(c, chain)) return c;
  const n = chain.joints.length;
  const byName = new Map(chain.joints.map((j, i) => [j.name, i]));
  const idx: number[] = [];
  const qIndex = new Int32Array(n).fill(-1), mimicSrc = new Int32Array(n).fill(-1), coupleSrc = new Int32Array(n).fill(-1);
  chain.joints.forEach((j, i) => {
    if (j.type !== 'fixed' && !j.mimic) { qIndex[i] = idx.length; idx.push(i); }
    if (j.mimic) mimicSrc[i] = byName.get(j.mimic.joint) ?? -1;
    if (j.couple) coupleSrc[i] = byName.get(j.couple.joint) ?? -1;
  });
  const refs = chain.joints.map((j) => ({ j, type: j.type, mimic: j.mimic, couple: j.couple }));
  const info: ChainInfo = { joints: chain.joints, n, idx, qIndex, mimicSrc, coupleSrc, refs };
  infoCache.set(chain, info);
  return info;
}

/** Drop cached analysis of a chain edited in place (joint type, mimic, couple, axis or origin changes). */
export function invalidateChain(chain: ChainDef): void {
  infoCache.delete(chain);
  for (const h of invalidateHooks) h(chain);
}
const invalidateHooks: Array<(c: ChainDef) => void> = [];
/** Lets dependent caches (analytic IK) clear themselves on invalidateChain. */
export function onInvalidateChain(h: (c: ChainDef) => void): void { invalidateHooks.push(h); }

/** Indices of independent (actuated) joints. */
export function actuatedIndices(chain: ChainDef): number[] {
  return chainInfo(chain).idx.slice();
}

/** Expand actuated joint values into all joint values (resolving mimics and fixed joints). */
export function expandJoints(chain: ChainDef, q: number[]): number[] {
  const info = chainInfo(chain);
  const full = new Array(info.n).fill(0);
  for (let i = 0; i < info.n; i++) if (info.qIndex[i] >= 0) full[i] = q[info.qIndex[i]] ?? 0;
  for (let i = 0; i < info.n; i++) {
    const s = info.mimicSrc[i];
    if (s >= 0) { const m = chain.joints[i].mimic!; full[i] = m.multiplier * full[s] + m.offset; }
  }
  return full;
}

/** Motion value of joint i (its own value plus a coupled joint's contribution), in the joint's units. */
export function motionValue(chain: ChainDef, full: number[], i: number): number {
  const s = chainInfo(chain).coupleSrc[i];
  return s >= 0 ? full[i] + chain.joints[i].couple!.multiplier * full[s] : full[i];
}

export function jointMotion(j: JointDef, value: number): Mat4 {
  switch (j.type) {
    case 'revolute':
    case 'continuous':
      return rotAxis(j.axis, value * DEG);
    case 'prismatic':
      return transl(j.axis[0] * value, j.axis[1] * value, j.axis[2] * value);
    default:
      return identity();
  }
}

export interface FKResult {
  /** Pose of each link frame relative to the robot base frame (links[0] = identity). */
  linkPoses: Mat4[];
  /** Pose of each joint frame (after origin, before motion) relative to base. */
  jointFrames: Mat4[];
  /** Flange pose relative to base. */
  flange: Mat4;
}

/** Forward kinematics. q = actuated joint values (deg/mm). */
export function forwardKinematics(chain: ChainDef, q: number[]): FKResult {
  if (chain.parallel?.kind === 'delta') return deltaFK(chain, q);
  const full = expandJoints(chain, q);
  let T = chain.base ? clone(chain.base) : identity();
  const linkPoses: Mat4[] = [identity()];
  const jointFrames: Mat4[] = [];
  for (let i = 0; i < chain.joints.length; i++) {
    const j = chain.joints[i];
    T = multiply(T, j.origin);
    jointFrames.push(clone(T));
    T = multiply(T, jointMotion(j, motionValue(chain, full, i)));
    if (j.post) T = multiply(T, j.post);
    linkPoses.push(clone(T));
  }
  const fl = chain.flangeLink;
  const flange = multiply(fl !== undefined && fl >= 0 && fl < linkPoses.length ? linkPoses[fl] : T, chain.flange);
  return { linkPoses, jointFrames, flange };
}

// Scratch buffers for the allocation-free flange evaluation (IK inner loops).
const sA = new Float64Array(16), sB = new Float64Array(16), sM = new Float64Array(16);

function setRotAxis(out: Mat4, x: number, y: number, z: number, rad: number): void {
  const l = Math.hypot(x, y, z) || 1;
  x /= l; y /= l; z /= l;
  const c = Math.cos(rad), s = Math.sin(rad), t = 1 - c;
  out[0] = t * x * x + c; out[4] = t * x * y - s * z; out[8] = t * x * z + s * y; out[12] = 0;
  out[1] = t * x * y + s * z; out[5] = t * y * y + c; out[9] = t * y * z - s * x; out[13] = 0;
  out[2] = t * x * z - s * y; out[6] = t * y * z + s * x; out[10] = t * z * z + c; out[14] = 0;
  out[3] = 0; out[7] = 0; out[11] = 0; out[15] = 1;
}

/** Flange pose only, without intermediate allocations (writes into `out`). */
export function flangePoseInto(chain: ChainDef, q: ArrayLike<number>, out: Mat4): Mat4 {
  if (chain.parallel?.kind === 'delta' || (chain.flangeLink !== undefined && chain.flangeLink !== chain.joints.length)) { out.set(forwardKinematics(chain, Array.from(q)).flange); return out; }
  const info = chainInfo(chain);
  const n = info.n;
  const full = fullScratch(n);
  for (let i = 0; i < n; i++) full[i] = info.qIndex[i] >= 0 ? (q[info.qIndex[i]] ?? 0) : 0;
  for (let i = 0; i < n; i++) if (info.mimicSrc[i] >= 0) { const m = chain.joints[i].mimic!; full[i] = m.multiplier * full[info.mimicSrc[i]] + m.offset; }
  let cur = sA, nxt = sB;
  if (chain.base) cur.set(chain.base); else { cur.fill(0); cur[0] = cur[5] = cur[10] = cur[15] = 1; }
  for (let i = 0; i < n; i++) {
    const j = chain.joints[i];
    multiply(cur, j.origin, nxt); [cur, nxt] = [nxt, cur];
    const v = info.coupleSrc[i] >= 0 ? full[i] + j.couple!.multiplier * full[info.coupleSrc[i]] : full[i];
    if ((j.type === 'revolute' || j.type === 'continuous') && j.axis[0] === 0 && j.axis[1] === 0 && j.axis[2] !== 0) {
      // rotation about ±z (DH joints): right-multiply by Rz in place (columns 0 and 1)
      const t = (j.axis[2] > 0 ? v : -v) * DEG, c = Math.cos(t), s = Math.sin(t);
      for (let r = 0; r < 3; r++) { const x = cur[r], y = cur[4 + r]; cur[r] = c * x + s * y; cur[4 + r] = c * y - s * x; }
    } else if (j.type === 'revolute' || j.type === 'continuous') { setRotAxis(sM, j.axis[0], j.axis[1], j.axis[2], v * DEG); multiply(cur, sM, nxt); [cur, nxt] = [nxt, cur]; }
    else if (j.type === 'prismatic') {
      // translate along the axis in the current frame
      const tx = j.axis[0] * v, ty = j.axis[1] * v, tz = j.axis[2] * v;
      cur[12] += cur[0] * tx + cur[4] * ty + cur[8] * tz;
      cur[13] += cur[1] * tx + cur[5] * ty + cur[9] * tz;
      cur[14] += cur[2] * tx + cur[6] * ty + cur[10] * tz;
    }
    const p = j.post;
    if (p) {
      if (p[0] === 1 && p[5] === 1 && p[10] === 1 && p[1] === 0 && p[2] === 0 && p[4] === 0 && p[6] === 0 && p[8] === 0 && p[9] === 0 && p[12] === 0 && p[13] === 0) {
        // pure translation along z (DH d): column 3 += d · column 2
        const d = p[14];
        if (d !== 0) { cur[12] += d * cur[8]; cur[13] += d * cur[9]; cur[14] += d * cur[10]; }
      } else { multiply(cur, p, nxt); [cur, nxt] = [nxt, cur]; }
    }
  }
  return multiply(cur, chain.flange, out);
}
let fullBuf = new Float64Array(16);
function fullScratch(n: number): Float64Array { if (fullBuf.length < n) fullBuf = new Float64Array(n * 2); return fullBuf; }

export function flangePose(chain: ChainDef, q: number[]): Mat4 {
  return flangePoseInto(chain, q, new Float64Array(16));
}

// -- Delta robot ---------------------------------------------------------------------------------------

/** Motor-frame pose of arm i (before its rotation), relative to the motor plane. */
export function deltaMotorFrame(d: DeltaDef, i: number): Mat4 {
  return multiply(rotz(d.armAngles[i] * DEG), transl(d.baseRadius, 0, 0));
}

/**
 * Delta platform centre in the motor-plane frame for motor angles q (deg), or null when the arms cannot close.
 * The platform hangs below the motor plane (the lower of the two sphere intersections).
 */
export function deltaPlatformPosition(d: DeltaDef, q: ArrayLike<number>): Vec3 | null {
  const c: Vec3[] = [];
  for (let i = 0; i < 3; i++) {
    const ph = d.armAngles[i] * DEG, a = (q[i] ?? 0) * DEG;
    const rad = d.baseRadius + d.upperArm * Math.cos(a) - d.platformRadius;
    c.push([rad * Math.cos(ph), rad * Math.sin(ph), -d.upperArm * Math.sin(a)]);
  }
  // trilateration of three spheres of radius lowerArm
  const [p1, p2, p3] = c;
  const e21 = sub(p2, p1), e31 = sub(p3, p1);
  const dd = Math.hypot(...e21);
  if (dd < 1e-9) return null;
  const ex: Vec3 = [e21[0] / dd, e21[1] / dd, e21[2] / dd];
  const i1 = ex[0] * e31[0] + ex[1] * e31[1] + ex[2] * e31[2];
  const eyr: Vec3 = [e31[0] - ex[0] * i1, e31[1] - ex[1] * i1, e31[2] - ex[2] * i1];
  const jn = Math.hypot(...eyr);
  if (jn < 1e-9) return null;
  const ey: Vec3 = [eyr[0] / jn, eyr[1] / jn, eyr[2] / jn];
  const ez = cross(ex, ey);
  const x = dd / 2;
  const y = (i1 * i1 + jn * jn) / (2 * jn) - (i1 * x) / jn;
  const zz = d.lowerArm * d.lowerArm - x * x - y * y;
  if (zz < -1e-9) return null;
  const z = Math.sqrt(Math.max(0, zz));
  const a: Vec3 = [p1[0] + ex[0] * x + ey[0] * y, p1[1] + ex[1] * x + ey[1] * y, p1[2] + ex[2] * x + ey[2] * y];
  const s = ez[2] > 0 ? -z : z; // pick the solution below the motor plane
  return [a[0] + ez[0] * s, a[1] + ez[1] * s, a[2] + ez[2] * s];
}

function deltaFK(chain: ChainDef, q: number[]): FKResult {
  const d = chain.parallel!;
  const B = chain.base ? clone(chain.base) : identity();
  const P = deltaPlatformPosition(d, q) ?? ([0, 0, -(d.upperArm + d.lowerArm)] as Vec3);
  const jointFrames: Mat4[] = [];
  const linkPoses: Mat4[] = [identity()];
  for (let i = 0; i < 3; i++) {
    const f = multiply(B, deltaMotorFrame(d, i));
    jointFrames.push(f);
    linkPoses.push(multiply(f, roty((q[i] ?? 0) * DEG)));
  }
  let plat = multiply(B, transl(P[0], P[1], P[2]));
  if (d.platform) plat = multiply(plat, d.platform);
  const j4 = chain.joints[3];
  if (j4) {
    jointFrames.push(clone(plat));
    if (j4.type !== 'fixed') plat = multiply(plat, jointMotion(j4, q[3] ?? 0));
  }
  linkPoses.push(clone(plat));
  for (let i = 4; i < chain.joints.length; i++) { jointFrames.push(clone(plat)); linkPoses.push(clone(plat)); }
  while (linkPoses.length < chain.links.length) linkPoses.push(clone(plat));
  return { linkPoses, jointFrames, flange: multiply(plat, chain.flange) };
}

/**
 * Geometric Jacobian (6 x dof) of the flange w.r.t. actuated joints, expressed in base frame.
 * Linear part in mm, angular part in rad, per unit joint (rad for revolute, mm for prismatic).
 */
export function jacobian(chain: ChainDef, q: number[]): number[][] {
  const n = chainInfo(chain).idx.length;
  return jacobianInto(chain, q, Array.from({ length: 6 }, () => new Array(n).fill(0)));
}

// scratch for jacobianInto: per joint frame position and axis (world), grown on demand
let jacP = new Float64Array(48), jacW = new Float64Array(48);
const jA = new Float64Array(16), jB = new Float64Array(16), jM = new Float64Array(16);

/** Geometric Jacobian written into `J` (6 rows x dof, zero-filled here) without per-call matrix allocations. */
export function jacobianInto(chain: ChainDef, q: ArrayLike<number>, J: number[][]): number[][] {
  if (chain.parallel || (chain.flangeLink !== undefined && chain.flangeLink !== chain.joints.length)) {
    const Jn = numericJacobian(chain, Array.from(q));
    for (let r = 0; r < 6; r++) for (let k = 0; k < Jn[r].length; k++) J[r][k] = Jn[r][k];
    return J;
  }
  const info = chainInfo(chain);
  const n = info.n;
  if (jacP.length < 3 * n) { jacP = new Float64Array(6 * n); jacW = new Float64Array(6 * n); }
  const full = fullScratch(n);
  for (let i = 0; i < n; i++) full[i] = info.qIndex[i] >= 0 ? (q[info.qIndex[i]] ?? 0) : 0;
  for (let i = 0; i < n; i++) if (info.mimicSrc[i] >= 0) { const m = chain.joints[i].mimic!; full[i] = m.multiplier * full[info.mimicSrc[i]] + m.offset; }
  let cur = jA, nxt = jB;
  if (chain.base) cur.set(chain.base); else { cur.fill(0); cur[0] = cur[5] = cur[10] = cur[15] = 1; }
  for (let i = 0; i < n; i++) {
    const j = chain.joints[i];
    multiply(cur, j.origin, nxt); [cur, nxt] = [nxt, cur];
    // joint frame: position and motion axis in the base frame
    const ax = j.axis[0], ay = j.axis[1], az = j.axis[2];
    let wx = cur[0] * ax + cur[4] * ay + cur[8] * az, wy = cur[1] * ax + cur[5] * ay + cur[9] * az, wz = cur[2] * ax + cur[6] * ay + cur[10] * az;
    if (j.type !== 'prismatic') { const l = Math.hypot(wx, wy, wz) || 1; wx /= l; wy /= l; wz /= l; }
    jacP[3 * i] = cur[12]; jacP[3 * i + 1] = cur[13]; jacP[3 * i + 2] = cur[14];
    jacW[3 * i] = wx; jacW[3 * i + 1] = wy; jacW[3 * i + 2] = wz;
    const v = info.coupleSrc[i] >= 0 ? full[i] + j.couple!.multiplier * full[info.coupleSrc[i]] : full[i];
    if (j.type === 'revolute' || j.type === 'continuous') { setRotAxis(jM, ax, ay, az, v * DEG); multiply(cur, jM, nxt); [cur, nxt] = [nxt, cur]; }
    else if (j.type === 'prismatic') {
      const tx = ax * v, ty = ay * v, tz = az * v;
      cur[12] += cur[0] * tx + cur[4] * ty + cur[8] * tz;
      cur[13] += cur[1] * tx + cur[5] * ty + cur[9] * tz;
      cur[14] += cur[2] * tx + cur[6] * ty + cur[10] * tz;
    }
    if (j.post) { multiply(cur, j.post, nxt); [cur, nxt] = [nxt, cur]; }
  }
  const F = chain.flange;
  const px = cur[0] * F[12] + cur[4] * F[13] + cur[8] * F[14] + cur[12];
  const py = cur[1] * F[12] + cur[5] * F[13] + cur[9] * F[14] + cur[13];
  const pz = cur[2] * F[12] + cur[6] * F[13] + cur[10] * F[14] + cur[14];
  const nq = info.idx.length;
  for (let r = 0; r < 6; r++) for (let k = 0; k < nq; k++) J[r][k] = 0;
  // Column of every moving joint (mimic joints add their screw, scaled, to their source column).
  const addScrew = (ji: number, k: number, scale: number) => {
    const t = chain.joints[ji].type;
    const wx = jacW[3 * ji], wy = jacW[3 * ji + 1], wz = jacW[3 * ji + 2];
    if (t === 'prismatic') { J[0][k] += scale * wx; J[1][k] += scale * wy; J[2][k] += scale * wz; return; }
    if (t !== 'revolute' && t !== 'continuous') return;
    const rx = px - jacP[3 * ji], ry = py - jacP[3 * ji + 1], rz = pz - jacP[3 * ji + 2];
    J[0][k] += scale * (wy * rz - wz * ry); J[1][k] += scale * (wz * rx - wx * rz); J[2][k] += scale * (wx * ry - wy * rx);
    J[3][k] += scale * wx; J[4][k] += scale * wy; J[5][k] += scale * wz;
  };
  for (let ji = 0; ji < n; ji++) {
    const j = chain.joints[ji];
    if (j.type === 'fixed') continue;
    // which actuated joint drives this one, and how much (a mimic of a mimic is resolved one level)
    let k = info.qIndex[ji], scale = 1;
    if (k < 0 && info.mimicSrc[ji] >= 0) { k = info.qIndex[info.mimicSrc[ji]]; scale = j.mimic!.multiplier; }
    if (k < 0) continue;
    addScrew(ji, k, scale);
    // A coupled joint rotates when its source joint moves, so its screw is added to the source column too.
    const cs = info.coupleSrc[ji];
    if (cs >= 0) {
      let ks = info.qIndex[cs], sc = j.couple!.multiplier;
      if (ks < 0 && info.mimicSrc[cs] >= 0) { ks = info.qIndex[info.mimicSrc[cs]]; sc *= chain.joints[cs].mimic!.multiplier; }
      if (ks >= 0) addScrew(ji, ks, sc);
    }
  }
  return J;
}

/** Central-difference Jacobian (any chain, including parallel models). */
export function numericJacobian(chain: ChainDef, q: number[]): number[][] {
  const idx = actuatedIndices(chain);
  const J: number[][] = Array.from({ length: 6 }, () => new Array(idx.length).fill(0));
  const f0 = flangePose(chain, q);
  const Rt = invert(f0);
  for (let k = 0; k < idx.length; k++) {
    const rev = chain.joints[idx[k]].type !== 'prismatic';
    const h = rev ? 1e-4 : 1e-3; // deg or mm
    const qp = q.slice(), qm = q.slice();
    qp[k] += h; qm[k] -= h;
    const fp = flangePose(chain, qp), fm = flangePose(chain, qm);
    const scale = rev ? 1 / (2 * h * DEG) : 1 / (2 * h);
    for (let r = 0; r < 3; r++) J[r][k] = (fp[12 + r] - fm[12 + r]) * scale;
    // angular velocity from dR · Rᵀ (skew part)
    const dR = multiply(fp, Rt), dRm = multiply(fm, Rt);
    J[3][k] = ((dR[6] - dR[9]) / 2 - (dRm[6] - dRm[9]) / 2) * scale;
    J[4][k] = ((dR[8] - dR[2]) / 2 - (dRm[8] - dRm[2]) / 2) * scale;
    J[5][k] = ((dR[1] - dR[4]) / 2 - (dRm[1] - dRm[4]) / 2) * scale;
  }
  return J;
}

export function jointLimits(chain: ChainDef): { lower: number[]; upper: number[] } {
  const idx = chainInfo(chain).idx;
  return { lower: idx.map((i) => chain.joints[i].lower), upper: idx.map((i) => chain.joints[i].upper) };
}

export function homeJoints(chain: ChainDef): number[] {
  return chainInfo(chain).idx.map((i) => chain.joints[i].home ?? 0);
}

export function withinLimits(chain: ChainDef, q: number[], tol = 1e-6): boolean {
  const { lower, upper } = jointLimits(chain);
  return q.every((v, i) => v >= lower[i] - tol && v <= upper[i] + tol);
}

export function invertPose(m: Mat4): Mat4 {
  return invert(m);
}
