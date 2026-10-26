/**
 * RoboDK robot model -> ChainDef. RoboDK stores every robot (`.robot`, and each robot embedded in an `.rdk`) as a parameter
 * table of doubles; this module turns the kinematic part of that table into a ChainDef whose FK equals RoboDK `SolveFK`
 * (verified against RoboDK 5.9.4 for all 96 Library robots, tests/kin_rdk_model.test.ts).
 *
 * Table layout (double index): [20] robot class (5 delta, 15/17 gripper), [21] DOF, [61] joints type (brand convention),
 * [64+i] joint senses (0 = brand default), [180..185] base offset x,y,z (mm), rx,ry,rz (rad), rows at [200+20i]:
 * alpha (rad), a, theta (rad), d, type (0 rotary / 1 linear), extra values [5..19]; [560..565] flange offset,
 * [600+i]/[620+i] limits, [680+i] home joints, [700+i] current joints, [120..124] motion state (linear speed, linear accel,
 * rounding, joint speed, joint accel).
 *
 * Model: flange = B · Π_i Rx(alpha_i) Tx(a_i) Rz(theta_i + s_i q_i) Tz(d_i) · F (linear joints: Tz(d_i + s_i q_i)),
 * B = pose(base), F = pose(flange), pose(x,y,z,rx,ry,rz) = transl · rotx · roty · rotz.
 * Senses s by joints type: 5 KUKA (-1,1,1,-1,1,-1), 2 Fanuc and 3 Yaskawa (1,1,-1,-1,-1,-1), 7/11/12 table values, else +1.
 * Fanuc (2) and Comau parallelogram (11): θ3 = s3 q3 - s2 q2 (J2/J3 interaction).
 * Delta (class 5): row 0 [5..9] = base triangle side, platform triangle side, upper arm, lower arm, (visual);
 * arms clockwise at 0, -120, -240 deg; 4-axis: platform rotated by Rx(row3[8]) then J4 about its Z.
 * Grippers (class 15-17, or one linear joint with sense 0.5): the flange stays on the base (flangeLink 0).
 */
import { ChainDef, JointDef, LinkDef, DeltaDef } from './chain';
import { Mat4, identity, transl, rotx, roty, rotz, mul, DEG } from '../math/pose';

export interface RdkRobotParams {
  name: string;
  /** Rows [alpha(rad), a, theta(rad), d, type, ...extra] as stored at table index 200+20i (at least 5 values each). */
  rows: number[][];
  /** Joint convention code (table [61], RoboDK JointLimits()[2]). */
  jointsType: number;
  /** Raw sense values (table [64+i]); 0 or missing = brand default. */
  senses?: number[];
  /** Base offset (table [180..185]): x, y, z (mm), rx, ry, rz (rad). */
  base?: number[];
  /** Flange offset (table [560..565]). */
  flange?: number[];
  lower?: number[];
  upper?: number[];
  home?: number[];
  /** Max joint speeds (deg/s | mm/s) and accelerations, when known. */
  maxVelocity?: number[];
  maxAcceleration?: number[];
  /** Robot class (table [20]): 5 delta, 15..17 gripper. */
  robotClass?: number;
  jointNames?: string[];
  /** Robot motion state stored with the robot (table [120..124]): linear speed mm/s, linear accel mm/s², rounding mm (-1 fine), joint speed deg/s, joint accel deg/s². */
  motion?: { speedLinear: number; accelLinear: number; rounding: number; speedJoints: number; accelJoints: number };
}

/** RoboDK 6-value pose: transl(x,y,z) · rotx(rx) · roty(ry) · rotz(rz), angles in radians. */
export function rdkPose(v?: ArrayLike<number> | null): Mat4 {
  if (!v || v.length < 6) return identity();
  const vals = Array.from(v).slice(0, 6).map((x) => (Number.isFinite(x) ? x : 0));
  const [x, y, z, rx, ry, rz] = vals;
  if (!x && !y && !z && !rx && !ry && !rz) return identity();
  return mul(transl(x, y, z), rotx(rx), roty(ry), rotz(rz));
}

/** Brand default joint senses for a RoboDK joints type (null = use the table values, missing = +1). */
export function rdkDefaultSenses(jointsType: number, dof: number): number[] | null {
  const pad = (s: number[]) => Array.from({ length: dof }, (_, i) => s[i] ?? 1);
  switch (Math.round(jointsType)) {
    case 5: return pad([-1, 1, 1, -1, 1, -1]);
    case 2: case 3: return pad([1, 1, -1, -1, -1, -1]);
    case 7: case 11: case 12: return null;
    default: return pad([]);
  }
}

/** Effective joint senses (scale factors; ±1 for rotary joints, 0.5 for gripper fingers). */
export function rdkSenses(jointsType: number, dof: number, table?: number[]): number[] {
  const def = rdkDefaultSenses(jointsType, dof);
  return Array.from({ length: dof }, (_, i) => {
    const t = table?.[i];
    const tableOk = t !== undefined && Number.isFinite(t) && t !== 0 && Math.abs(t) <= 10;
    if (def === null) return tableOk ? t! : 1;
    // Brands with fixed conventions ignore zero table entries; an explicit non-unit scale (gripper) wins.
    return tableOk && Math.abs(Math.abs(t!) - 1) > 1e-9 ? t! : def[i];
  });
}

/** True when the joints type couples J3 to J2 (Fanuc J2/J3 interaction, Comau parallelogram). */
export function rdkCouplesJ3(jointsType: number): boolean {
  const t = Math.round(jointsType);
  return t === 2 || t === 11;
}

export function chainFromRobodk(p: RdkRobotParams): ChainDef {
  const dof = p.rows.length;
  const s = rdkSenses(p.jointsType, dof, p.senses);
  const isDelta = Math.round(p.robotClass ?? -1) === 5 && dof >= 3 && (p.rows[0]?.[7] ?? 0) > 0 && (p.rows[0]?.[8] ?? 0) > 0;
  const linear = p.rows.map((r) => Math.round(r[4] ?? 0) === 1);
  const name = (i: number) => p.jointNames?.[i] ?? `J${i + 1}`;
  const lim = (i: number, lo: number, hi: number) => [p.lower?.[i] ?? lo, p.upper?.[i] ?? hi] as const;
  const extras = (i: number): Partial<JointDef> => ({
    home: p.home?.[i] ?? 0,
    maxVelocity: p.maxVelocity?.[i] ?? (linear[i] ? 1000 : 180),
    ...(p.maxAcceleration?.[i] !== undefined ? { maxAcceleration: p.maxAcceleration[i] } : {}),
  });
  const base = rdkPose(p.base);
  const flange = rdkPose(p.flange);
  if (isDelta) {
    const r0 = p.rows[0];
    const d: DeltaDef = {
      kind: 'delta',
      baseRadius: r0[5] / (2 * Math.sqrt(3)),
      platformRadius: r0[6] / (2 * Math.sqrt(3)),
      upperArm: r0[7],
      lowerArm: r0[8],
      armAngles: [0, -120, -240],
    };
    if (dof >= 4 && Math.abs(p.rows[3]?.[8] ?? 0) > 1e-12) d.platform = rotx(p.rows[3][8]);
    const joints: JointDef[] = [0, 1, 2].map((i) => {
      const [lo, hi] = lim(i, -60, 110);
      return { name: name(i), type: 'revolute', origin: identity(), axis: [0, 1, 0], lower: lo, upper: hi, ...extras(i) } as JointDef;
    });
    if (dof >= 4) {
      const [lo, hi] = lim(3, -360, 360);
      joints.push({ name: name(3), type: 'revolute', origin: identity(), axis: [0, 0, s[3] < 0 ? -1 : 1], lower: lo, upper: hi, ...extras(3) } as JointDef);
    } else joints.push({ name: 'platform', type: 'fixed', origin: identity(), axis: [0, 0, 1], lower: 0, upper: 0 });
    const links: LinkDef[] = ['base', 'arm_1', 'arm_2', 'arm_3', 'platform'].map((n) => ({ name: n, visuals: [] }));
    return { name: p.name, joints, links, flange, base, parallel: d, dof: Math.min(dof, 4), jointsType: p.jointsType };
  }
  const joints: JointDef[] = p.rows.map((r, i) => {
    const [alpha, a, theta, dd] = r;
    const type = linear[i] ? 'prismatic' : 'revolute';
    const [lo, hi] = lim(i, linear[i] ? 0 : -180, linear[i] ? 1000 : 180);
    const sense = s[i] || 1;
    return {
      name: name(i), type,
      origin: mul(rotx(alpha), transl(a, 0, 0), rotz(theta)),
      // rotary: the sign is the joint sense (rotAxis normalises); linear: the axis length scales the travel
      axis: [0, 0, linear[i] ? sense : Math.sign(sense)],
      post: transl(0, 0, dd),
      lower: lo, upper: hi, ...extras(i),
    } as JointDef;
  });
  if (rdkCouplesJ3(p.jointsType) && dof >= 3 && !linear[1] && !linear[2]) {
    const s2 = Math.sign(s[1] || 1), s3 = Math.sign(s[2] || 1);
    joints[2].couple = { joint: joints[1].name, multiplier: -s2 / s3 };
  }
  const links: LinkDef[] = [{ name: 'base', visuals: [] }, ...joints.map((_, i) => ({ name: `link_${i + 1}`, visuals: [] }))];
  const gripper = (p.robotClass !== undefined && Math.round(p.robotClass) >= 15 && Math.round(p.robotClass) <= 17) || (dof === 1 && linear[0] && Math.abs(Math.abs(s[0]) - 0.5) < 1e-9);
  const chain: ChainDef = { name: p.name, joints, links, flange, dof, jointsType: p.jointsType };
  if (base.some((v, i) => Math.abs(v - (i % 5 === 0 ? 1 : 0)) > 1e-12)) chain.base = base;
  if (gripper) chain.flangeLink = 0;
  return chain;
}

/** Read the kinematic parameters from a RoboDK robot table given as an array of doubles (index 0 = the leading 1.0). */
export function rdkParamsFromTable(t: ArrayLike<number>, name = 'Robot'): RdkRobotParams | null {
  const at = (i: number) => (i < t.length && Number.isFinite(t[i]) ? t[i] : 0);
  const dof = Math.round(at(21));
  if (dof < 1 || dof > 12 || t.length < 200 + 20 * dof) return null;
  const rows: number[][] = [];
  for (let i = 0; i < dof; i++) rows.push(Array.from({ length: 20 }, (_, k) => at(200 + 20 * i + k)));
  const range = (s: number, n: number) => Array.from({ length: n }, (_, i) => at(s + i));
  const out: RdkRobotParams = {
    name, rows, jointsType: at(61), senses: range(64, dof), base: range(180, 6), flange: range(560, 6),
    lower: range(600, dof), upper: range(620, dof), home: range(680, dof), robotClass: at(20),
  };
  const [vl, al, rnd, vj, aj] = range(120, 5);
  // no joint speed / acceleration in the table (0: ABB IRB 6640, KUKA KR 120 of the Library): RoboDK moves the joints at
  // 2·v deg/s and 2·a deg/s² (measured with Item.Update() on 03.h / 05.c)
  if (vl > 0 && al > 0) out.motion = { speedLinear: vl, accelLinear: al, rounding: rnd, speedJoints: vj > 0 ? vj : 2 * vl, accelJoints: aj > 0 ? aj : 2 * al };
  return out;
}

export { DEG };
