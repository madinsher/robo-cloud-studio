/**
 * Mechanism builder — RoboDK `BuildMechanism` / `setRobotParams` (modified DH).
 * Types: MAKE_ROBOT_1R, 1T, 2R, 2T, 3R, 3T, 4R, 6DOF, 7DOF?, linear rail/turntable.
 * 6-DOF parameters follow RoboDK's robot builder: [d1, a1, a2, a3, d4, d6] (mm); 1R/1T/2R/... use axis lengths.
 */
import { ChainDef, JointDef, DHParams, chainFromDH } from './chain';
import { Mat4, identity, transl, rotx, mul, DEG, rotz } from '../math/pose';
import { Robot } from '../items/robot';

export const MAKE_ROBOT_1R = 1, MAKE_ROBOT_1T = 2, MAKE_ROBOT_2R = 3, MAKE_ROBOT_2T = 4, MAKE_ROBOT_3R = 5, MAKE_ROBOT_3T = 6, MAKE_ROBOT_4R = 7, MAKE_ROBOT_4T = 8, MAKE_ROBOT_6DOF = 9, MAKE_ROBOT_7DOF = 10, MAKE_ROBOT_SCARA = 11, MAKE_ROBOT_1R1T = 12, MAKE_ROBOT_GENERIC = 20;

export interface MechanismSpec {
  type: number;
  parameters: number[];
  jointsBuild?: number[];
  jointsHome?: number[];
  jointsSenses?: number[];
  lower?: number[];
  upper?: number[];
  base?: Mat4;
  tool?: Mat4;
  name?: string;
}

const rev = (name: string, origin: Mat4, axis: [number, number, number], lower = -360, upper = 360, home = 0): JointDef => ({ name, type: 'revolute', origin, axis, lower, upper, maxVelocity: 180, home });
const prism = (name: string, origin: Mat4, axis: [number, number, number], lower = 0, upper = 1000, home = 0): JointDef => ({ name, type: 'prismatic', origin, axis, lower, upper, maxVelocity: 1000, home });

export function buildMechanismChain(spec: MechanismSpec): ChainDef {
  const p = spec.parameters;
  const name = spec.name ?? 'Mechanism';
  const lim = (i: number, lo: number, hi: number) => [spec.lower?.[i] ?? lo, spec.upper?.[i] ?? hi] as const;
  const sense = (i: number) => (spec.jointsSenses?.[i] ?? 1) < 0 ? -1 : 1;
  let joints: JointDef[] = [];
  let flange: Mat4 = identity();
  switch (spec.type) {
    case MAKE_ROBOT_1R: { const [lo, hi] = lim(0, -360, 360); joints = [rev('J1', identity(), [0, 0, sense(0)], lo, hi)]; flange = transl(0, 0, p[0] ?? 0); break; }
    case MAKE_ROBOT_1T: { const [lo, hi] = lim(0, 0, p[0] ?? 1000); joints = [prism('J1', identity(), [sense(0), 0, 0], lo, hi)]; break; }
    case MAKE_ROBOT_1R1T: { const [l0, h0] = lim(0, 0, p[0] ?? 1000); const [l1, h1] = lim(1, -360, 360); joints = [prism('J1', identity(), [sense(0), 0, 0], l0, h0), rev('J2', transl(0, 0, p[1] ?? 0), [0, 0, sense(1)], l1, h1)]; break; }
    case MAKE_ROBOT_2R: { const [l0, h0] = lim(0, -360, 360), [l1, h1] = lim(1, -360, 360); joints = [rev('J1', identity(), [0, 0, sense(0)], l0, h0), rev('J2', mul(transl(0, 0, p[0] ?? 0), rotx(-90 * DEG)), [0, 0, sense(1)], l1, h1)]; flange = transl(0, 0, p[1] ?? 0); break; }
    case MAKE_ROBOT_2T: { const [l0, h0] = lim(0, 0, p[0] ?? 1000), [l1, h1] = lim(1, 0, p[1] ?? 1000); joints = [prism('J1', identity(), [sense(0), 0, 0], l0, h0), prism('J2', identity(), [0, sense(1), 0], l1, h1)]; break; }
    case MAKE_ROBOT_3T: { const [l0, h0] = lim(0, 0, p[0] ?? 1000), [l1, h1] = lim(1, 0, p[1] ?? 1000), [l2, h2] = lim(2, 0, p[2] ?? 1000); joints = [prism('X', identity(), [sense(0), 0, 0], l0, h0), prism('Y', identity(), [0, sense(1), 0], l1, h1), prism('Z', identity(), [0, 0, sense(2)], l2, h2)]; break; }
    case MAKE_ROBOT_3R: { const [l0, h0] = lim(0, -360, 360), [l1, h1] = lim(1, -360, 360), [l2, h2] = lim(2, -360, 360); joints = [rev('J1', identity(), [0, 0, sense(0)], l0, h0), rev('J2', mul(transl(p[1] ?? 0, 0, p[0] ?? 0), rotx(-90 * DEG)), [0, 0, sense(1)], l1, h1), rev('J3', transl(p[2] ?? 0, 0, 0), [0, 0, sense(2)], l2, h2)]; flange = transl(p[3] ?? 0, 0, 0); break; }
    case MAKE_ROBOT_SCARA: { const dh: DHParams[] = [{ theta: 0, d: p[2] ?? 300, a: p[0] ?? 300, alpha: 0 }, { theta: 0, d: 0, a: p[1] ?? 250, alpha: 180 }, { theta: 0, d: 0, a: 0, alpha: 0, type: 'prismatic', lower: 0, upper: p[3] ?? 200 }, { theta: 0, d: 0, a: 0, alpha: 0 }]; return chainFromDH(name, dh); }
    case MAKE_ROBOT_6DOF: case MAKE_ROBOT_7DOF: {
      // RoboDK robot builder: d1, a1, a2, a3, d4, d6
      const [d1 = 400, a1 = 25, a2 = 455, a3 = 35, d4 = 420, d6 = 80] = p;
      const dh: DHParams[] = [
        { theta: 0, d: d1, a: a1, alpha: -90, lower: spec.lower?.[0] ?? -170, upper: spec.upper?.[0] ?? 170 },
        { theta: -90, d: 0, a: a2, alpha: 0, lower: spec.lower?.[1] ?? -190, upper: spec.upper?.[1] ?? 45 },
        { theta: 0, d: 0, a: a3, alpha: -90, lower: spec.lower?.[2] ?? -120, upper: spec.upper?.[2] ?? 156 },
        { theta: 0, d: d4, a: 0, alpha: 90, lower: spec.lower?.[3] ?? -185, upper: spec.upper?.[3] ?? 185 },
        { theta: 0, d: 0, a: 0, alpha: -90, lower: spec.lower?.[4] ?? -120, upper: spec.upper?.[4] ?? 120 },
        { theta: 0, d: d6, a: 0, alpha: 0, lower: spec.lower?.[5] ?? -350, upper: spec.upper?.[5] ?? 350 },
      ];
      if (spec.type === MAKE_ROBOT_7DOF) dh.splice(3, 0, { theta: 0, d: 0, a: 0, alpha: 90, lower: -170, upper: 170 });
      dh.forEach((d, i) => { if (spec.jointsSenses?.[i] !== undefined && spec.jointsSenses[i] < 0) d.sign = -1; d.home = spec.jointsHome?.[i] ?? 0; });
      const c = chainFromDH(name, dh, spec.tool ?? identity());
      return c;
    }
    case MAKE_ROBOT_4R: { const dh: DHParams[] = [{ theta: 0, d: p[0] ?? 400, a: p[1] ?? 100, alpha: -90 }, { theta: -90, d: 0, a: p[2] ?? 600, alpha: 0 }, { theta: 0, d: 0, a: p[3] ?? 600, alpha: 0 }, { theta: 0, d: 0, a: 0, alpha: 0 }]; return chainFromDH(name, dh, spec.tool ?? identity()); }
    case MAKE_ROBOT_4T: { joints = [0, 1, 2, 3].map((i) => prism(`T${i + 1}`, identity(), i === 0 ? [1, 0, 0] : i === 1 ? [0, 1, 0] : [0, 0, 1], 0, p[i] ?? 1000)); break; }
    default: throw new Error(`Unsupported mechanism type ${spec.type}`);
  }
  joints.forEach((j, i) => { j.home = spec.jointsHome?.[i] ?? 0; });
  const links = [{ name: 'base', visuals: [] }, ...joints.map((j) => ({ name: `link_${j.name}`, visuals: [] }))];
  return { name, joints, links, flange: spec.tool ? mul(flange, spec.tool) : flange, dof: joints.length };
}

export function buildMechanism(spec: MechanismSpec): Robot {
  const r = new Robot(spec.name ?? 'Mechanism', buildMechanismChain(spec));
  if (spec.base) r.setPose(spec.base);
  if (spec.jointsBuild) r.setJoints(spec.jointsBuild);
  r.params.mechanismType = spec.type;
  return r;
}

/**
 * Modified (Craig) DH table -> chain:  T_i = Rx(alpha_{i-1}) · Tx(a_{i-1}) · Rz(theta_i) · Tz(d_i).
 * Rows: [alpha, a, theta, d] (RoboDK `setRobotParams` DHM order: alpha, a, theta, d).
 */
export function chainFromDHM(name: string, dhm: number[][], lower?: number[], upper?: number[]): ChainDef {
  const joints: JointDef[] = dhm.map((row, i) => {
    const [alpha, a, theta, d] = row;
    return { name: `J${i + 1}`, type: 'revolute', origin: mul(rotx(alpha * DEG), transl(a, 0, 0), rotz(theta * DEG)), axis: [0, 0, 1], post: transl(0, 0, d), lower: lower?.[i] ?? -180, upper: upper?.[i] ?? 180, maxVelocity: 180, home: 0 };
  });
  const links = [{ name: 'base', visuals: [] }, ...joints.map((j, i) => ({ name: `link_${i + 1}`, visuals: [] }))];
  return { name, joints, links, flange: identity(), dof: joints.length };
}

/** Standard DH table (theta, d, a, alpha) -> modified DH rows (alpha, a, theta, d) for export. */
export function dhToDHM(dh: DHParams[]): number[][] {
  // classic -> modified conversion: alpha_{i-1}, a_{i-1} shift by one joint
  const rows: number[][] = [];
  for (let i = 0; i < dh.length; i++) rows.push([i === 0 ? 0 : dh[i - 1].alpha, i === 0 ? 0 : dh[i - 1].a, dh[i].theta, dh[i].d]);
  return rows;
}
