/**
 * Synchronised external axes (RoboDK "synchronised" rails, gantries, turntables, positioners).
 *
 * A SyncedSystem is an arm plus mechanisms listed in RoboDK external-axis order; the joint vector is
 * q = [arm joints..., axes[0] joints..., axes[1] joints...] (RoboDK 7/8/9-joint targets). A *carrier* holds the arm
 * (linear track, gantry), a *positioner* holds the work (turntable, 2-axis positioner).
 *
 * RoboDK SolveFK of a synchronised robot (verified against RoboDK 5.9.4: Library 05.c rail + turntable, 05.d Gudel
 * gantry, tests/kin_combined.test.ts): the flange pose relative to the first positioner's flange (the moving part
 * frame) when there is a positioner, otherwise relative to the first carrier's base, otherwise the arm base.
 * syncedIK solves the inverse problem over arm + external axes, honouring the external-axis values a target gives and
 * choosing the free ones (closest feasible values to the seed) with the closed-form arm solver.
 */
import { ChainDef, JointDef, flangePose, jointLimits, actuatedIndices, homeJoints } from './chain';
import { Mat4, multiply, identity, invert } from '../math/pose';
import { Robot } from '../items/robot';
import { inverseKinematics, solveIKBest, ikAll, IKOptions, IKResult, jointDistance } from './ik';

/** One synchronised mechanism. */
export interface SyncAxis {
  chain: ChainDef;
  role: 'carrier' | 'positioner';
  /** Mechanism base pose in the common root frame (station / world). */
  base: Mat4;
  /** Carrier only: arm base relative to the carrier flange. */
  mount?: Mat4;
  /** Joint limits of the synchronised axes (RoboDK keeps them with the arm); default: the mechanism's own limits. */
  lower?: number[];
  upper?: number[];
  /** Item id (item-level systems). */
  id?: string;
}

export interface SyncedSystem {
  arm: ChainDef;
  /** Arm base in the root frame (used when no carrier holds the arm). */
  armBase: Mat4;
  /** Mechanisms in external-axis order. */
  axes: SyncAxis[];
}

const dofOf = (c: ChainDef) => actuatedIndices(c).length;

/** Total number of joints (arm + external axes). */
export function syncedDof(sys: SyncedSystem): number {
  return dofOf(sys.arm) + sys.axes.reduce((s, a) => s + dofOf(a.chain), 0);
}

/** Split a full joint vector into arm joints and per-axis joint lists (missing values: home). */
export function splitSynced(sys: SyncedSystem, q: number[]): { arm: number[]; ext: number[][] } {
  const na = dofOf(sys.arm);
  const arm = q.slice(0, na);
  const ah = homeJoints(sys.arm);
  while (arm.length < na) arm.push(ah[arm.length] ?? 0);
  let k = na;
  const ext = sys.axes.map((a) => {
    const n = dofOf(a.chain), h = homeJoints(a.chain);
    const v = Array.from({ length: n }, (_, i) => (Number.isFinite(q[k + i]) ? q[k + i] : h[i] ?? 0));
    k += n;
    return v;
  });
  return { arm, ext };
}

/** Limits of the external joints (flattened, external-axis order). */
export function syncedExtLimits(sys: SyncedSystem): { lower: number[]; upper: number[] } {
  const lower: number[] = [], upper: number[] = [];
  for (const a of sys.axes) {
    const l = jointLimits(a.chain);
    l.lower.forEach((v, i) => { lower.push(a.lower?.[i] ?? v); upper.push(a.upper?.[i] ?? l.upper[i]); });
  }
  return { lower, upper };
}

/** Root pose of the arm base for external values (per axis). */
function armBaseAt(sys: SyncedSystem, ext: number[][]): Mat4 {
  const k = sys.axes.findIndex((a) => a.role === 'carrier');
  if (k < 0) return sys.armBase;
  const c = sys.axes[k];
  return multiply(multiply(c.base, flangePose(c.chain, ext[k])), c.mount ?? identity());
}

/** Root pose of the SolveFK reference: first positioner flange, else first carrier base, else the arm base. */
function referenceAt(sys: SyncedSystem, ext: number[][]): Mat4 {
  const p = sys.axes.findIndex((a) => a.role === 'positioner');
  if (p >= 0) return multiply(sys.axes[p].base, flangePose(sys.axes[p].chain, ext[p]));
  const c = sys.axes.find((a) => a.role === 'carrier');
  return c ? c.base : sys.armBase;
}

/** Root pose of the arm base for a full joint vector. */
export function syncedArmBaseAbs(sys: SyncedSystem, q: number[]): Mat4 {
  return armBaseAt(sys, splitSynced(sys, q).ext);
}

/** Root pose of the RoboDK SolveFK reference frame for a full joint vector. */
export function syncedReferenceAbs(sys: SyncedSystem, q: number[]): Mat4 {
  return referenceAt(sys, splitSynced(sys, q).ext);
}

/** Root pose of the flange (or of the TCP with `tool`). */
export function syncedPoseAbs(sys: SyncedSystem, q: number[], tool?: Mat4 | null): Mat4 {
  const { arm, ext } = splitSynced(sys, q);
  const m = multiply(armBaseAt(sys, ext), flangePose(sys.arm, arm));
  return tool ? multiply(m, tool) : m;
}

/** RoboDK SolveFK of a synchronised robot: flange (TCP with `tool`) relative to the synchronised reference. */
export function syncedFK(sys: SyncedSystem, q: number[], tool?: Mat4 | null): Mat4 {
  const { arm, ext } = splitSynced(sys, q);
  const m = multiply(multiply(invert(referenceAt(sys, ext)), armBaseAt(sys, ext)), flangePose(sys.arm, arm));
  return tool ? multiply(m, tool) : m;
}

export interface SyncedIKOptions extends IKOptions {
  /** External-axis values to honour (flattened, external-axis order); undefined / null / NaN entries are free. */
  ext?: Array<number | null | undefined>;
  /** TCP w.r.t. the flange (the pose is then a TCP pose). */
  tool?: Mat4 | null;
  /** Free revolute external axes are searched within seed ± this many degrees (default 180). */
  revoluteWindow?: number;
}

export interface SyncedIKResult extends IKResult {
  /** Arm joints followed by the external joints. */
  joints: number[];
  armJoints: number[];
  extJoints: number[];
}

/**
 * IK of a synchronised robot. `pose` is the flange (TCP with `tool`) relative to the synchronised reference evaluated
 * at the solution's external values (the part frame on a positioner, the carrier base, or the arm base), i.e. the
 * inverse of syncedFK. External values given in `opts.ext` are kept; free ones start at the seed and, when the arm
 * cannot reach the pose there, are chosen by a coarse-to-fine search for the feasible values closest to the seed.
 */
export function syncedIK(sys: SyncedSystem, pose: Mat4, opts: SyncedIKOptions = {}): SyncedIKResult {
  const na = dofOf(sys.arm);
  const { tool, ext: given, revoluteWindow = 180, ...ikOpts } = opts;
  const seedFull = opts.seed ?? [...homeJoints(sys.arm), ...sys.axes.flatMap((a) => homeJoints(a.chain))];
  const split = splitSynced(sys, seedFull);
  const seedArm = split.arm;
  const e0 = split.ext.flat();
  const { lower, upper } = syncedExtLimits(sys);
  const extTypes = sys.axes.flatMap((a) => actuatedIndices(a.chain).map((i) => a.chain.joints[i].type));
  const free: number[] = [];
  const qe = e0.map((v, i) => {
    const g = given?.[i];
    if (g !== undefined && g !== null && Number.isFinite(g)) return g;
    free.push(i);
    return Math.max(lower[i], Math.min(upper[i], v));
  });
  const toLists = (flat: number[]): number[][] => { let k = 0; return sys.axes.map((a) => { const n = dofOf(a.chain); const v = flat.slice(k, k + n); k += n; return v; }); };
  const toolInv = tool ? invert(tool) : null;
  const flangeTarget = (flat: number[]): Mat4 => {
    const ext = toLists(flat);
    let t = multiply(multiply(invert(armBaseAt(sys, ext)), referenceAt(sys, ext)), pose);
    if (toolInv) t = multiply(t, toolInv);
    return t;
  };
  const relax: IKOptions = na <= 3 ? { positionOnly: true } : na <= 5 ? { freeToolZ: true } : {};
  const solveArm = (flat: number[], quick: boolean): IKResult =>
    solveIKBest(sys.arm, flangeTarget(flat), { ...relax, ...ikOpts, seed: seedArm, ...(quick ? { restarts: 0, maxIterations: Math.min(ikOpts.maxIterations ?? 60, 60) } : {}) });
  const pack = (r: IKResult, flat: number[]): SyncedIKResult => ({ ...r, joints: [...r.joints, ...flat], armJoints: r.joints, extJoints: flat.slice() });

  const first = solveArm(qe, false);
  if (first.ok || !free.length) return pack(first, qe);

  // free external axes: coordinate search, cost = joint motion when the arm reaches the pose, else a large penalty
  // plus the distance from the arm base to the target (pulls a rail / gantry toward the work)
  const span = free.map((i) => {
    if (extTypes[i] === 'prismatic') return [lower[i], upper[i]];
    return [Math.max(lower[i], e0[i] - revoluteWindow), Math.min(upper[i], e0[i] + revoluteWindow)];
  });
  const scaleE = free.map((_, k) => Math.max(1e-9, span[k][1] - span[k][0]));
  const cost = (flat: number[]): { c: number; r: IKResult | null } => {
    const ft = flangeTarget(flat);
    const r = solveIKBest(sys.arm, ft, { ...relax, ...ikOpts, seed: seedArm, restarts: 0, maxIterations: 60 });
    let move = 0;
    free.forEach((i, k) => { move += ((flat[i] - e0[i]) / scaleE[k]) ** 2; });
    if (r.ok) return { c: jointDistance(r.joints, seedArm) / 1e4 + move, r };
    return { c: 1e6 + Math.hypot(ft[12], ft[13], ft[14]) / 1e3 + move, r: null };
  };
  let best = qe.slice();
  let bc = cost(best);
  const N = 32;
  const win = span.map((s) => [s[0], s[1]]);
  for (let pass = 0; pass < 6; pass++) {
    for (let k = 0; k < free.length; k++) {
      const i = free[k];
      const [lo, hi] = win[k];
      if (hi - lo < 1e-9) continue;
      for (let s = 0; s <= N; s++) {
        const cand = best.slice();
        cand[i] = lo + ((hi - lo) * s) / N;
        const c = cost(cand);
        if (c.c < bc.c) { bc = c; best = cand; }
      }
    }
    // shrink every window around the current best (only once a feasible point exists)
    if (bc.r) for (let k = 0; k < free.length; k++) {
      const i = free[k], w = (win[k][1] - win[k][0]) / 8;
      win[k] = [Math.max(span[k][0], best[i] - w), Math.min(span[k][1], best[i] + w)];
    }
  }
  if (bc.r) return pack(solveArm(best, false), best);
  // no closed-form success anywhere: combined numeric solve over arm + free axes from the best point found
  return numericSynced(sys, pose, best, free, seedArm, toolInv, { ...relax, ...ikOpts }, pack, first);
}

function numericSynced(sys: SyncedSystem, pose: Mat4, best: number[], free: number[], seedArm: number[], toolInv: Mat4 | null, ik: IKOptions,
  pack: (r: IKResult, flat: number[]) => SyncedIKResult, fallback: IKResult): SyncedIKResult {
  // only the carrier's free axes are folded into one chain; positioners stay at their best values
  const k = sys.axes.findIndex((a) => a.role === 'carrier');
  if (k < 0) return pack({ ...fallback, ok: false }, best);
  const c = sys.axes[k];
  let off = 0;
  for (let j = 0; j < k; j++) off += dofOf(sys.axes[j].chain);
  const nc = dofOf(c.chain);
  const lists = (() => { let o = 0; return sys.axes.map((a) => { const n = dofOf(a.chain); const v = best.slice(o, o + n); o += n; return v; }); })();
  const chain = combineChains(withLimits(c), c.mount ?? identity(), sys.arm);
  // target in the carrier base frame
  let t = multiply(multiply(invert(c.base), referenceAt(sys, lists)), pose);
  if (toolInv) t = multiply(t, toolInv);
  const res = inverseKinematics(chain, t, { restarts: 2, ...ik, seed: [...lists[k], ...seedArm] });
  const flat = best.slice();
  const freeC = new Set(free);
  for (let j = 0; j < nc; j++) if (freeC.has(off + j)) flat[off + j] = res.joints[j];
  const arm = res.joints.slice(nc);
  return { ...res, joints: [...arm, ...flat], armJoints: arm, extJoints: flat };
}

function withLimits(a: SyncAxis): ChainDef {
  if (!a.lower && !a.upper) return a.chain;
  const idx = actuatedIndices(a.chain);
  return { ...a.chain, joints: a.chain.joints.map((j, ji) => { const k = idx.indexOf(ji); return k < 0 ? j : { ...j, lower: a.lower?.[k] ?? j.lower, upper: a.upper?.[k] ?? j.upper }; }) };
}

/** All arm solutions (inside the limits) for fixed external values, each followed by those values (RoboDK SolveIK_All). */
export function syncedIKAll(sys: SyncedSystem, pose: Mat4, ext: number[], opts: { seed?: number[]; tool?: Mat4 | null } = {}): number[][] {
  const lists = (() => { let o = 0; return sys.axes.map((a) => { const n = dofOf(a.chain); const v = ext.slice(o, o + n); o += n; return v; }); })();
  let t = multiply(multiply(invert(armBaseAt(sys, lists)), referenceAt(sys, lists)), pose);
  if (opts.tool) t = multiply(t, invert(opts.tool));
  return ikAll(sys.arm, t, { seed: opts.seed?.slice(0, dofOf(sys.arm)) }).map((q) => [...q, ...ext]);
}

// -- item level ------------------------------------------------------------------------------------------------

/** Mechanisms synchronised with a robot: its externalAxes (RoboDK order), the carrier first when it is not listed. */
export function syncedMechanisms(robot: Robot): Robot[] {
  const st = robot.station;
  const out: Robot[] = [];
  for (const id of robot.externalAxes ?? []) { const m = st?.findById(id); if (m instanceof Robot && m !== robot && !out.includes(m)) out.push(m); }
  const c = carrierOf(robot);
  if (c && !out.includes(c)) out.unshift(c);
  return out;
}

/**
 * The synchronised system of a robot in station coordinates, built from the current item poses and joints
 * (carrier: an ancestor mechanism; positioner: any other synchronised mechanism), or null without external axes.
 * Limits stored with the arm by the RoboDK import (params.syncLimits) override the mechanisms' own limits.
 */
export function syncedSystemFor(robot: Robot): SyncedSystem | null {
  const mechs = syncedMechanisms(robot);
  if (!mechs.length) return null;
  const sl = robot.params.syncLimits as { lower?: number[]; upper?: number[] } | undefined;
  let off = 0;
  const axes: SyncAxis[] = mechs.map((m) => {
    const n = m.dof;
    const lower = Array.isArray(sl?.lower) && sl!.lower.length >= off + n ? sl!.lower.slice(off, off + n) : undefined;
    const upper = Array.isArray(sl?.upper) && sl!.upper.length >= off + n ? sl!.upper.slice(off, off + n) : undefined;
    off += n;
    const base = m.poseAbs();
    const ax: SyncAxis = { chain: m.chain, role: m.isAncestorOf(robot) ? 'carrier' : 'positioner', base, id: m.id };
    if (lower && upper && lower.every((v, i) => v < upper[i])) { ax.lower = lower; ax.upper = upper; }
    if (ax.role === 'carrier') ax.mount = multiply(invert(multiply(base, m.solveFKFlange())), robot.poseAbs());
    return ax;
  });
  return { arm: robot.chain, armBase: robot.poseAbs(), axes };
}

/** Current full joint vector of a robot and its synchronised mechanisms. */
export function syncedJoints(robot: Robot): number[] {
  return [...robot.joints(), ...syncedMechanisms(robot).flatMap((m) => m.joints())];
}

/** Apply a full joint vector to a robot and its synchronised mechanisms. */
export function applySynced(robot: Robot, q: number[]): void {
  robot.setJoints(q.slice(0, robot.dof));
  let k = robot.dof;
  for (const m of syncedMechanisms(robot)) { if (q.length >= k + m.dof) m.setJoints(q.slice(k, k + m.dof)); k += m.dof; }
}

/** RoboDK `SolveFK` for a joint vector that may include the synchronised external axes (flange w.r.t. the robot base otherwise). */
export function robotSolveFK(robot: Robot, q: number[], tool?: Mat4 | null): Mat4 {
  const sys = q.length > robot.dof ? syncedSystemFor(robot) : null;
  if (!sys) return robot.solveFKRef(q.slice(0, robot.dof), tool);
  return syncedFK(sys, q, tool);
}

// -- legacy helpers ------------------------------------------------------------------------------------------

/** Build a chain = carrier chain -> (carrier flange -> robot base transform) -> robot chain. */
export function combineChains(carrier: ChainDef, robotBaseOnCarrierFlange: Mat4, robot: ChainDef, name = 'combined'): ChainDef {
  const joints: JointDef[] = [
    ...carrier.joints.map((j) => ({ ...j, name: `${carrier.name}:${j.name}`, mimic: j.mimic ? { ...j.mimic, joint: `${carrier.name}:${j.mimic.joint}` } : undefined, couple: j.couple ? { ...j.couple, joint: `${carrier.name}:${j.couple.joint}` } : undefined })),
    // fixed joint carrying the mount transform (and the arm's own base offset)
    { name: `${carrier.name}:mount`, type: 'fixed', origin: multiply(multiply(carrier.flange, robotBaseOnCarrierFlange), robot.base ?? identity()), axis: [0, 0, 1], lower: 0, upper: 0 },
    ...robot.joints.map((j) => ({ ...j, name: `${robot.name}:${j.name}`, mimic: j.mimic ? { ...j.mimic, joint: `${robot.name}:${j.mimic.joint}` } : undefined, couple: j.couple ? { ...j.couple, joint: `${robot.name}:${j.couple.joint}` } : undefined })),
  ];
  const links = [...carrier.links, { name: 'mount', visuals: [] }, ...robot.links.slice(1)];
  const out: ChainDef = { name, joints, links, flange: robot.flange, dof: joints.filter((j) => j.type !== 'fixed' && !j.mimic).length };
  if (carrier.base) out.base = carrier.base;
  return out;
}

/**
 * Solve IK for a robot mounted on a carrier Robot item (e.g. linear rail). Target is the TCP pose (active tool)
 * relative to the carrier base; `opts.seed` is [carrier joints..., robot joints...]. Closed-form arms use syncedIK
 * (exact, carrier values closest to the seed); other arms the combined numeric chain.
 */
export function solveIKWithCarrier(carrier: Robot, robot: Robot, targetInCarrierBase: Mat4, opts: IKOptions & { carrierWeight?: number } = {}): { ok: boolean; carrierJoints: number[]; robotJoints: number[]; result: IKResult } {
  const nc = carrier.dof, dof = robot.dof;
  const mount = multiply(invert(multiply(carrier.poseAbs(), carrier.solveFKFlange())), robot.poseAbs());
  const tool = robot.poseTool();
  const { seed, carrierWeight: _w, ...rest } = opts;
  void _w;
  const seedC = seed ? seed.slice(0, nc) : carrier.joints();
  const seedR = seed ? seed.slice(nc, nc + dof) : robot.joints();
  if (!rest.positionOnly && !rest.freeToolZ && dof >= 6 && robot.hasAnalyticIK()) {
    const sys: SyncedSystem = { arm: robot.chain, armBase: identity(), axes: [{ chain: carrier.chain, role: 'carrier', base: identity(), mount }] };
    const r = syncedIK(sys, targetInCarrierBase, { ...rest, tool, seed: [...seedR, ...seedC] });
    if (r.ok) return { ok: true, carrierJoints: r.extJoints, robotJoints: r.armJoints, result: { ...r, joints: [...r.extJoints, ...r.armJoints] } };
  }
  return solveIKWithCarrierNumeric(carrier, robot, mount, tool, targetInCarrierBase, [...seedC, ...seedR], rest, !!seed);
}

function solveIKWithCarrierNumeric(carrier: Robot, robot: Robot, mount: Mat4, tool: Mat4, targetInCarrierBase: Mat4, seed0: number[], rest: IKOptions, userSeed: boolean) {
  const chain = combineChains(carrier.chain, mount, robot.chain);
  const flangeTarget = multiply(targetInCarrierBase, invert(tool));
  const dof = robot.dof, nc = carrier.dof;
  const relax: IKOptions = dof <= 3 ? { positionOnly: true } : dof <= 5 ? { freeToolZ: true } : {};
  const tp: [number, number, number] = [targetInCarrierBase[12], targetInCarrierBase[13], targetInCarrierBase[14]];
  const { lower, upper } = carrier.jointLimits();
  // Seeds: current configuration, then carrier pre-positioned so the arm base is as close as possible to the target
  // (coordinate descent over the carrier axes), each combined with the current arm joints and the arm home.
  const seeds: number[][] = [[...carrier.joints(), ...robot.joints()]];
  const best = carrier.joints();
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
  if (userSeed) seeds.unshift(seed0);
  let out: IKResult | null = null;
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
