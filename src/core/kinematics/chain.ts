import { Mat4, Vec3, identity, multiply, rotAxis, transl, clone, invert, getPos, cross, sub, transformDir, mul, rotz, roty, rotx, DEG } from '../math/pose';

export type JointType = 'revolute' | 'prismatic' | 'fixed' | 'continuous';

export interface JointDef {
  name: string;
  type: JointType;
  /** Static transform from previous link frame to this joint frame (before motion). */
  origin: Mat4;
  /** Motion axis in the joint frame. */
  axis: Vec3;
  /** Static transform applied after the motion (used by DH-defined robots). */
  post?: Mat4;
  /** Limits (degrees for revolute, mm for prismatic). */
  lower: number;
  upper: number;
  /** Max velocity (deg/s or mm/s) and acceleration. */
  maxVelocity?: number;
  maxAcceleration?: number;
  /** Mimic another joint: q = multiplier * q_src + offset. */
  mimic?: { joint: string; multiplier: number; offset: number };
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
}

export interface LinkDef {
  name: string;
  visuals: LinkVisual[];
}

/** A serial kinematic chain: base -> joint_1 -> link_1 -> ... -> joint_n -> link_n -> flange. */
export interface ChainDef {
  name: string;
  joints: JointDef[];
  /** Links; links[0] is the base link (attached before any joint). links[i] follows joints[i-1]. */
  links: LinkDef[];
  /** Transform from the last link to the mechanical flange. */
  flange: Mat4;
  /** Optional per-joint DH parameters (retained for RoboDK export). */
  dh?: DHParams[];
  /** Number of actuated joints (mimic/fixed excluded). */
  dof?: number;
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

/** Indices of independent (actuated) joints. */
export function actuatedIndices(chain: ChainDef): number[] {
  const out: number[] = [];
  chain.joints.forEach((j, i) => {
    if (j.type !== 'fixed' && !j.mimic) out.push(i);
  });
  return out;
}

/** Expand actuated joint values into all joint values (resolving mimics and fixed joints). */
export function expandJoints(chain: ChainDef, q: number[]): number[] {
  const full = new Array(chain.joints.length).fill(0);
  const idx = actuatedIndices(chain);
  idx.forEach((ji, k) => (full[ji] = q[k] ?? 0));
  const byName = new Map(chain.joints.map((j, i) => [j.name, i]));
  chain.joints.forEach((j, i) => {
    if (j.mimic) {
      const src = byName.get(j.mimic.joint);
      if (src !== undefined) full[i] = j.mimic.multiplier * full[src] + j.mimic.offset;
    }
  });
  return full;
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
  /** Pose of each link frame relative to the chain base (links[0] = identity). */
  linkPoses: Mat4[];
  /** Pose of each joint frame (after origin, before motion) relative to base. */
  jointFrames: Mat4[];
  /** Flange pose relative to base. */
  flange: Mat4;
}

/** Forward kinematics. q = actuated joint values (deg/mm). */
export function forwardKinematics(chain: ChainDef, q: number[]): FKResult {
  const full = expandJoints(chain, q);
  let T = identity();
  const linkPoses: Mat4[] = [clone(T)];
  const jointFrames: Mat4[] = [];
  for (let i = 0; i < chain.joints.length; i++) {
    const j = chain.joints[i];
    T = multiply(T, j.origin);
    jointFrames.push(clone(T));
    T = multiply(T, jointMotion(j, full[i]));
    if (j.post) T = multiply(T, j.post);
    linkPoses.push(clone(T));
  }
  const flange = multiply(T, chain.flange);
  return { linkPoses, jointFrames, flange };
}

export function flangePose(chain: ChainDef, q: number[]): Mat4 {
  return forwardKinematics(chain, q).flange;
}

/**
 * Geometric Jacobian (6 x dof) of the flange w.r.t. actuated joints, expressed in base frame.
 * Linear part in mm, angular part in rad, per unit joint (rad for revolute, mm for prismatic).
 */
export function jacobian(chain: ChainDef, q: number[]): number[][] {
  const fk = forwardKinematics(chain, q);
  const pe = getPos(fk.flange);
  const idx = actuatedIndices(chain);
  const J: number[][] = Array.from({ length: 6 }, () => new Array(idx.length).fill(0));
  idx.forEach((ji, k) => {
    const j = chain.joints[ji];
    const frame = fk.jointFrames[ji];
    const axisW = transformDir(frame, j.axis);
    if (j.type === 'prismatic') {
      J[0][k] = axisW[0]; J[1][k] = axisW[1]; J[2][k] = axisW[2];
    } else {
      const pj = getPos(frame);
      const lin = cross(axisW, sub(pe, pj));
      J[0][k] = lin[0]; J[1][k] = lin[1]; J[2][k] = lin[2];
      J[3][k] = axisW[0]; J[4][k] = axisW[1]; J[5][k] = axisW[2];
    }
  });
  return J;
}

export function jointLimits(chain: ChainDef): { lower: number[]; upper: number[] } {
  const idx = actuatedIndices(chain);
  return { lower: idx.map((i) => chain.joints[i].lower), upper: idx.map((i) => chain.joints[i].upper) };
}

export function homeJoints(chain: ChainDef): number[] {
  return actuatedIndices(chain).map((i) => chain.joints[i].home ?? 0);
}

export function withinLimits(chain: ChainDef, q: number[], tol = 1e-6): boolean {
  const { lower, upper } = jointLimits(chain);
  return q.every((v, i) => v >= lower[i] - tol && v <= upper[i] + tol);
}

export function invertPose(m: Mat4): Mat4 {
  return invert(m);
}
