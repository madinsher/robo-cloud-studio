/**
 * External axes: combine a carrier mechanism (linear track, gantry, turntable, mobile base lift)
 * with a robot arm into one kinematic chain so IK can use all joints (RoboDK "synchronised axes").
 */
import { ChainDef, JointDef } from './chain';
import { Mat4, multiply, identity, invert } from '../math/pose';
import { Robot } from '../items/robot';
import { inverseKinematics, IKOptions, IKResult } from './ik';

/** Build a chain = carrier chain -> (carrier flange -> robot base transform) -> robot chain. */
export function combineChains(carrier: ChainDef, robotBaseOnCarrierFlange: Mat4, robot: ChainDef, name = 'combined'): ChainDef {
  const joints: JointDef[] = [
    ...carrier.joints.map((j) => ({ ...j, name: `${carrier.name}:${j.name}`, mimic: j.mimic ? { ...j.mimic, joint: `${carrier.name}:${j.mimic.joint}` } : undefined })),
    // fixed joint carrying the mount transform
    { name: `${carrier.name}:mount`, type: 'fixed', origin: multiply(carrier.flange, robotBaseOnCarrierFlange), axis: [0, 0, 1], lower: 0, upper: 0 },
    ...robot.joints.map((j) => ({ ...j, name: `${robot.name}:${j.name}`, mimic: j.mimic ? { ...j.mimic, joint: `${robot.name}:${j.mimic.joint}` } : undefined })),
  ];
  const links = [...carrier.links, { name: 'mount', visuals: [] }, ...robot.links.slice(1)];
  // links count must be joints + 1: carrier.links (n_c+1) + mount (1) + robot links minus base (n_r) = n_c + n_r + 2 = joints + 1 ✓
  return { name, joints, links, flange: robot.flange, dof: joints.filter((j) => j.type !== 'fixed' && !j.mimic).length };
}

/**
 * Solve IK for a robot mounted on a carrier Robot item (e.g. linear rail). Target is the TCP pose
 * relative to the carrier base. Returns carrier joints and robot joints.
 */
export function solveIKWithCarrier(carrier: Robot, robot: Robot, targetInCarrierBase: Mat4, opts: IKOptions & { carrierWeight?: number } = {}): { ok: boolean; carrierJoints: number[]; robotJoints: number[]; result: IKResult } {
  // robot base relative to carrier flange
  const carrierFlangeAbs = multiply(carrier.poseAbs(), carrier.solveFKFlange());
  const mount = multiply(invert(carrierFlangeAbs), robot.poseAbs());
  const chain = combineChains(carrier.chain, mount, robot.chain);
  const tool = robot.poseTool();
  const flangeTarget = multiply(targetInCarrierBase, invert(tool));
  const dof = robot.dof;
  const nc = carrier.dof;
  const relax: IKOptions = dof <= 3 ? { positionOnly: true } : dof <= 5 ? { freeToolZ: true } : {};
  const tp: [number, number, number] = [targetInCarrierBase[12], targetInCarrierBase[13], targetInCarrierBase[14]];
  const { lower, upper } = carrier.jointLimits();
  // Seeds: current configuration, then carrier pre-positioned so the arm base is as close as possible to the target
  // (coordinate descent over the carrier axes), each combined with the current arm joints and the arm home.
  const seeds: number[][] = [[...carrier.joints(), ...robot.joints()]];
  let best = carrier.joints();
  const baseDist = (qc: number[]) => {
    const f = multiply(carrier.solveFKFlange(qc), mount);
    return Math.hypot(f[12] - tp[0], f[13] - tp[1], f[14] - tp[2]);
  };
  for (let pass = 0; pass < 2; pass++) {
    for (let k = 0; k < nc; k++) {
      if (upper[k] - lower[k] < 1e-9) continue;
      let bd = baseDist(best), bv = best[k];
      for (let i = 0; i <= 24; i++) {
        const v = lower[k] + ((upper[k] - lower[k]) * i) / 24;
        const q = [...best]; q[k] = v;
        const d = baseDist(q);
        if (d < bd) { bd = d; bv = v; }
      }
      best[k] = bv;
    }
  }
  seeds.push([...best, ...robot.joints()], [...best, ...robot.jointsHome()]);
  let out: IKResult | null = null;
  const { seed: _ignored, carrierWeight: _w, ...rest } = opts;
  void _ignored; void _w;
  if (opts.seed) seeds.unshift([...opts.seed]);
  for (const seed of seeds) {
    const res = inverseKinematics(chain, flangeTarget, { restarts: 2, ...relax, ...rest, seed });
    if (!out || res.ok || res.posError < out.posError) out = res;
    if (res.ok) break;
  }
  const res = out!;
  return { ok: res.ok, carrierJoints: res.joints.slice(0, nc), robotJoints: res.joints.slice(nc), result: res };
}

/** Apply a combined solution to both items. */
export function applyCombined(carrier: Robot, robot: Robot, sol: { carrierJoints: number[]; robotJoints: number[] }): void {
  carrier.setJoints(sol.carrierJoints);
  robot.setJoints(sol.robotJoints);
}

/** Find the carrier of a robot: the nearest ancestor that is a Robot (rail, gantry, positioner). */
export function carrierOf(robot: Robot): Robot | null {
  let p = robot.parent;
  while (p) {
    if (p instanceof Robot) return p;
    p = p.parent;
  }
  return null;
}

export { identity };
