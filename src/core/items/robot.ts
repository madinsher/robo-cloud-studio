import { Mat4, identity, multiply, invert, clone, fromArray } from '../math/pose';
import { Item, ItemType, Frame, Tool, Target, SerializedItem, DeserializeContext, registerItemType } from './item';
import { ChainDef, forwardKinematics, flangePose, homeJoints, jointLimits, actuatedIndices, withinLimits, FKResult, deltaPlatformPosition } from '../kinematics/chain';
import { IKOptions, IKResult, closestConfiguration, solveIKBest, ikAll } from '../kinematics/ik';
import { configFlags as analyticConfigFlags, analyzeChain } from '../kinematics/ik_analytic';

export interface RobotMotionParams {
  /** Linear speed mm/s, joint speed deg/s, accelerations. */
  speedLinear: number;
  speedJoints: number;
  accelLinear: number;
  accelJoints: number;
  /** Rounding / blending radius (mm); -1 = fine point. */
  rounding: number;
}

export type RobotBrand = 'Generic' | 'KUKA' | 'ABB' | 'Fanuc' | 'UR' | 'Yaskawa' | 'Staubli' | 'Doosan' | 'Kawasaki' | 'Denso' | 'Mecademic' | 'Comau' | 'ROS2' | 'Custom';

export interface RobotState {
  joints: number[];
  moving: boolean;
  /** Fraction of last programmed motion completed. */
  progress: number;
  faults: string[];
  /** Digital I/O */
  io: Record<string, number | boolean>;
}

/** Options of the RoboDK-style IK call (flange pose unless a tool is given; pose relative to the base unless a reference is given). */
export interface SolveIKRefOptions extends IKOptions {
  /** TCP w.r.t. the flange. Omitted / null = the pose is the flange (RoboDK SolveIK default). */
  tool?: Mat4 | null;
  /** Reference frame w.r.t. the robot base. Omitted / null = the robot base. */
  reference?: Mat4 | null;
}

const DEFAULT_MOTION: RobotMotionParams = { speedLinear: 500, speedJoints: 90, accelLinear: 2000, accelJoints: 360, rounding: -1 };

/**
 * Serial robot arm (RoboDK ITEM_TYPE_ROBOT). The robot pose is its base frame w.r.t. parent.
 * Active tool: a Tool child. Active reference frame: a Frame anywhere in the station.
 *
 * Kinematics API (all poses in mm, joints in deg/mm):
 *  - flange-based, RoboDK semantics: solveFKRef(q, tool?, reference?), solveIKRef(pose, {seed, tool, reference, config}),
 *    solveIKAllRef(pose, tool?, reference?), jointsConfig(q)  (tool/reference default to identity, like RoboDK SolveFK/SolveIK)
 *  - legacy TCP helpers (active tool by default): solveFK(q, tool), solveIK(tcpInBase, opts, tool), solveIKAll(tcpInBase, tool),
 *    solveIKFrame, poseTCP, poseTCPAbs.
 */
export class Robot extends Item {
  chain: ChainDef;
  private _joints: number[];
  brand: RobotBrand = 'Generic';
  model = '';
  /** Post-processor id. */
  postProcessor = 'Generic';
  /** Controller IP / ROS namespace for connections. */
  connection: { ip?: string; port?: number; rosNamespace?: string } = {};
  /** Current motion state (program speed / acceleration / rounding instructions change it). */
  motion: RobotMotionParams = { ...DEFAULT_MOTION };
  /** Robot default speeds and accelerations (from the robot file); resetMotion() restores them. */
  motionDefaults: RobotMotionParams = { ...DEFAULT_MOTION };
  activeToolId: string | null = null;
  activeFrameId: string | null = null;
  state: RobotState = { joints: [], moving: false, progress: 1, faults: [], io: {} };
  /** Reach (mm): largest distance from axis 1 to the wrist centre (or flange) over the joint ranges. */
  reach = 0;
  /** External axes / turntable ids synchronised with this robot. */
  externalAxes: string[] = [];

  constructor(name: string, chain: ChainDef, id?: string) {
    super(ItemType.ROBOT, name, id);
    this.chain = chain;
    this._joints = homeJoints(chain);
    this.state.joints = [...this._joints];
    this.reach = estimateReach(chain);
  }

  get dof(): number {
    return actuatedIndices(this.chain).length;
  }

  jointNames(): string[] {
    return actuatedIndices(this.chain).map((i) => this.chain.joints[i].name);
  }

  joints(): number[] {
    return [...this._joints];
  }

  setJoints(q: number[]): this {
    const n = this.dof;
    const v = q.slice(0, n);
    while (v.length < n) v.push(0);
    this._joints = v;
    this.state.joints = [...v];
    this.notify('joints');
    return this;
  }

  jointsHome(): number[] {
    return homeJoints(this.chain);
  }

  setJointsHome(q: number[]): void {
    actuatedIndices(this.chain).forEach((ji, k) => { if (q[k] !== undefined) this.chain.joints[ji].home = q[k]; });
    this.notify('home');
  }

  jointLimits(): { lower: number[]; upper: number[] } {
    return jointLimits(this.chain);
  }

  setJointLimits(lower: number[], upper: number[]): void {
    actuatedIndices(this.chain).forEach((ji, k) => {
      if (lower[k] !== undefined) this.chain.joints[ji].lower = lower[k];
      if (upper[k] !== undefined) this.chain.joints[ji].upper = upper[k];
    });
    this.notify('limits');
  }

  // -- Speed / acceleration limits ------------------------------------------

  /** Max joint speeds (deg/s or mm/s). Joints without a value report the chain default (180 deg/s, 1000 mm/s). */
  jointMaxVelocities(): number[] {
    return actuatedIndices(this.chain).map((i) => { const j = this.chain.joints[i]; return j.maxVelocity ?? (j.type === 'prismatic' ? 1000 : 180); });
  }

  setJointMaxVelocities(v: number[]): void {
    actuatedIndices(this.chain).forEach((ji, k) => { if (v[k] !== undefined && v[k] > 0) this.chain.joints[ji].maxVelocity = v[k]; });
    this.notify('limits');
  }

  /** Max joint accelerations (deg/s² or mm/s²); defaults to 4 × the max speed per second when unknown. */
  jointMaxAccelerations(): number[] {
    return actuatedIndices(this.chain).map((i, k) => this.chain.joints[i].maxAcceleration ?? this.jointMaxVelocities()[k] * 4);
  }

  setJointMaxAccelerations(a: number[]): void {
    actuatedIndices(this.chain).forEach((ji, k) => { if (a[k] !== undefined && a[k] > 0) this.chain.joints[ji].maxAcceleration = a[k]; });
    this.notify('limits');
  }

  /** Set the robot default speeds (file values) and make them current. */
  setMotionDefaults(m: Partial<RobotMotionParams>): void {
    this.motionDefaults = { ...this.motionDefaults, ...m };
    this.motion = { ...this.motionDefaults };
  }

  /** Restore the default speeds / accelerations / rounding (start of a program run). */
  resetMotion(): void {
    this.motion = { ...this.motionDefaults };
  }

  // -- Tool / frame ---------------------------------------------------------

  tools(): Tool[] {
    return this.children.filter((c) => c instanceof Tool) as Tool[];
  }

  activeTool(): Tool | null {
    if (this.activeToolId) {
      const t = this.station?.findById(this.activeToolId);
      if (t instanceof Tool) return t;
    }
    return this.tools()[0] ?? null;
  }

  setTool(tool: Tool | null): void {
    this.activeToolId = tool?.id ?? null;
    this.notify('tool');
  }

  activeFrame(): Frame | Item | null {
    if (this.activeFrameId) {
      const f = this.station?.findById(this.activeFrameId);
      if (f) return f;
    }
    return this.parent && this.parent.type !== ItemType.STATION ? this.parent : null;
  }

  setFrame(frame: Item | null): void {
    this.activeFrameId = frame?.id ?? null;
    this.notify('frame');
  }

  /** TCP pose relative to the flange (identity if no tool). */
  poseTool(): Mat4 {
    return this.activeTool()?.poseTool() ?? identity();
  }

  /** Active reference frame pose relative to the robot base. */
  poseFrame(): Mat4 {
    const f = this.activeFrame();
    if (!f) return identity();
    return multiply(invert(this.poseAbs()), f.poseAbs());
  }

  // -- Kinematics: RoboDK semantics (flange-based, explicit tool / reference) ----------------------------

  /**
   * RoboDK `SolveFK(joints, tool, reference)`: the flange pose w.r.t. the robot base; with `tool` the TCP pose, with
   * `reference` (reference frame w.r.t. the robot base) the pose expressed in that reference.
   */
  solveFKRef(q: number[] = this._joints, tool?: Mat4 | null, reference?: Mat4 | null): Mat4 {
    let m = flangePose(this.chain, q);
    if (tool) m = multiply(m, tool);
    if (reference) m = multiply(invert(reference), m);
    return m;
  }

  /** RoboDK `SolveIK(pose, joints_approx, tool, reference)`: closest solution to the seed (default: current joints). */
  solveIKRef(pose: Mat4, opts: SolveIKRefOptions = {}): IKResult {
    const { tool, reference, ...ik } = opts;
    let flange = pose;
    if (tool) flange = multiply(flange, invert(tool));
    if (reference) flange = multiply(reference, flange);
    return this.solveFlange(flange, { seed: this._joints, ...ik });
  }

  /** RoboDK `SolveIK_All(pose, tool, reference)`: every solution inside the limits, in a stable configuration order. */
  solveIKAllRef(pose: Mat4, tool?: Mat4 | null, reference?: Mat4 | null): number[][] {
    let flange = pose;
    if (tool) flange = multiply(flange, invert(tool));
    if (reference) flange = multiply(reference, flange);
    return ikAll(this.chain, flange, { seed: this._joints });
  }

  /** RoboDK `JointsConfig(joints)`: [REAR, LOWERARM, FLIP, turns(0)]. */
  jointsConfig(q: number[] = this._joints): [number, number, number, number] {
    const f = analyticConfigFlags(this.chain, q);
    if (f) return [f[0], f[1], f[2], 0];
    const c = this.legacyFlags(q);
    return [c.front ? 0 : 1, c.elbowUp ? 0 : 1, c.wristFlip ? 1 : 0, 0];
  }

  /** True when IK is solved in closed form for this robot (6R spherical wrist, UR-type, delta). */
  hasAnalyticIK(): boolean {
    return analyzeChain(this.chain).kind !== null;
  }

  /** Flange-target IK with the automatic relaxation of < 6-DOF robots. */
  private solveFlange(flangeTarget: Mat4, opts: IKOptions): IKResult {
    const dof = this.dof;
    const parallelOk = !!this.chain.parallel && analyzeChain(this.chain).kind === 'delta';
    const relax: IKOptions = parallelOk ? {} : dof <= 3 ? { positionOnly: true } : dof <= 5 ? { freeToolZ: true } : {};
    const seed = opts.seed ?? this._joints;
    const res = solveIKBest(this.chain, flangeTarget, { ...relax, ...opts, seed });
    if (res.ok && res.method !== 'analytic') res.joints = closestConfiguration(this.chain, res.joints, seed);
    if (!res.ok || res.method === 'analytic' || dof < 6 || opts.keepFirstSolution) return res;
    return this.pickConfiguration(flangeTarget, res, seed, relax, opts);
  }

  /** Numeric chains: prefer the solution whose configuration matches the seed's (tries flipped seeds). */
  private pickConfiguration(flangeTarget: Mat4, res: IKResult, seed: number[], relax: IKOptions, opts: IKOptions): IKResult {
    if (Math.max(...res.joints.map((v, i) => Math.abs(v - seed[i]))) < 60) return res;
    const seedCfg = this.legacyFlags(seed);
    const candidates: IKResult[] = [res];
    const flips: number[][] = [
      [seed[0], seed[1] + seed[2], -seed[2], seed[3] - seed[2], seed[4], seed[5]],              // elbow flip
      [seed[0], seed[1], seed[2], seed[3] + 180, -seed[4], seed[5] + 180],                       // wrist flip
      [seed[0], seed[1] + seed[2], -seed[2], seed[3] - seed[2] + 180, -seed[4], seed[5] + 180],  // both
    ];
    const { lower, upper } = this.jointLimits();
    for (const f of flips) {
      const sd = f.map((v, i) => Math.max(lower[i] ?? -360, Math.min(upper[i] ?? 360, v)));
      const r = solveIKBest(this.chain, flangeTarget, { ...relax, ...opts, seed: sd, restarts: 0, maxIterations: 120 });
      if (r.ok) { r.joints = closestConfiguration(this.chain, r.joints, seed); candidates.push(r); }
    }
    const dist = (q: number[]) => q.reduce((acc, v, i) => acc + Math.abs(v - seed[i]) * (i < 3 ? 1.5 : 1), 0);
    let best = candidates[0], bestScore = Infinity;
    for (const c of candidates) {
      const cfg = this.legacyFlags(c.joints);
      const score = (cfg.elbowUp !== seedCfg.elbowUp ? 1000 : 0) + (cfg.wristFlip !== seedCfg.wristFlip ? 300 : 0) + dist(c.joints);
      if (score < bestScore) { bestScore = score; best = c; }
    }
    return best;
  }

  // -- Kinematics: legacy TCP helpers --------------------------------------------------------------------

  fk(q: number[] = this._joints): FKResult {
    return forwardKinematics(this.chain, q);
  }

  /** Flange pose relative to the robot base. */
  solveFKFlange(q: number[] = this._joints): Mat4 {
    return flangePose(this.chain, q);
  }

  /** TCP pose relative to the robot base (active tool by default; pass identity() for the flange). */
  solveFK(q: number[] = this._joints, tool: Mat4 = this.poseTool()): Mat4 {
    return multiply(flangePose(this.chain, q), tool);
  }

  /** TCP pose w.r.t. the active reference frame (RoboDK Item.Pose for a robot). */
  poseTCP(q: number[] = this._joints): Mat4 {
    return multiply(invert(this.poseFrame()), this.solveFK(q));
  }

  /** Absolute TCP pose. */
  poseTCPAbs(q: number[] = this._joints): Mat4 {
    return multiply(this.poseAbs(), this.solveFK(q));
  }

  /**
   * Inverse kinematics: target = TCP pose relative to the robot base (active tool by default).
   * Closed form (6R spherical wrist / UR-type / delta) returns the solution closest to `opts.seed` (default: current
   * joints), honouring `opts.config`; other chains use damped least squares.
   */
  solveIK(target: Mat4, opts: IKOptions = {}, tool: Mat4 = this.poseTool()): IKResult {
    return this.solveFlange(multiply(target, invert(tool)), { seed: this._joints, ...opts });
  }

  /** Configuration flags of a 6-axis arm for a joint vector: elbow up/down, wrist flip and front (RoboDK conventions when available). */
  configFlags(q: number[]): { elbowUp: boolean; wristFlip: boolean; front: boolean } {
    const f = analyticConfigFlags(this.chain, q);
    if (f) return { front: f[0] === 0, elbowUp: f[1] === 0, wristFlip: f[2] === 1 };
    return this.legacyFlags(q);
  }

  /** Geometric flags for chains without a closed form. */
  private legacyFlags(q: number[]): { elbowUp: boolean; wristFlip: boolean; front: boolean } {
    const fk = this.fk(q);
    const lp = fk.linkPoses;
    const shoulder = lp[Math.min(1, lp.length - 1)], elbow = lp[Math.min(2, lp.length - 1)], wrist = lp[Math.min(4, lp.length - 1)];
    // elbow above the shoulder->wrist chord, measured along the base Z axis
    const chordDir = [wrist[12] - shoulder[12], wrist[13] - shoulder[13], wrist[14] - shoulder[14]];
    const l2 = chordDir[0] ** 2 + chordDir[1] ** 2 + chordDir[2] ** 2 || 1;
    const t = ((elbow[12] - shoulder[12]) * chordDir[0] + (elbow[13] - shoulder[13]) * chordDir[1] + (elbow[14] - shoulder[14]) * chordDir[2]) / l2;
    const chordZ = shoulder[14] + chordDir[2] * t;
    return { elbowUp: elbow[14] >= chordZ, wristFlip: (q[4] ?? 0) < 0, front: Math.abs(((q[0] ?? 0) + 540) % 360 - 180) < 90 };
  }

  /** All IK solutions of a TCP pose relative to the robot base (active tool by default), inside the limits. */
  solveIKAll(target: Mat4, tool: Mat4 = this.poseTool(), _attempts = 12): number[][] {
    return ikAll(this.chain, multiply(target, invert(tool)), { seed: this._joints });
  }

  /** IK for a pose given in the active reference frame. */
  solveIKFrame(targetInFrame: Mat4, opts: IKOptions = {}): IKResult {
    return this.solveIK(multiply(this.poseFrame(), targetInFrame), opts);
  }

  /** Move immediately (simulation) to a target or joints. */
  moveJInstant(target: Target | number[] | Mat4): boolean {
    const q = this.jointsForTarget(target);
    if (!q) return false;
    this.setJoints(q);
    return true;
  }

  /** Resolve a target / joints / pose (in active frame) into joint values. */
  jointsForTarget(target: Target | number[] | Mat4, opts: IKOptions = {}): number[] | null {
    if (Array.isArray(target)) return target;
    if (target instanceof Float64Array) {
      const r = this.solveIKFrame(target, opts);
      return r.ok ? r.joints : null;
    }
    if (target.isJointTarget && target.joints) return target.joints;
    // Cartesian target: its pose is relative to its parent (a frame). Convert to the robot base.
    const abs = target.poseAbs();
    const inBase = multiply(invert(this.poseAbs()), abs);
    const r = this.solveIK(inBase, { seed: target.joints ?? this._joints, ...opts });
    return r.ok ? r.joints : null;
  }

  /** Check whether joints are inside limits. */
  jointsValid(q: number[]): boolean {
    return withinLimits(this.chain, q);
  }

  setPoseTool(m: Mat4): void {
    const t = this.activeTool();
    if (t) t.setPoseTool(m);
  }

  // -- Serialization ------------------------------------------------------

  protected override serializeExtra() {
    return {
      chain: serializeChain(this.chain),
      joints: this._joints,
      brand: this.brand,
      model: this.model,
      postProcessor: this.postProcessor,
      connection: this.connection,
      motion: this.motion,
      motionDefaults: this.motionDefaults,
      activeToolId: this.activeToolId,
      activeFrameId: this.activeFrameId,
      externalAxes: this.externalAxes,
      reach: this.reach,
    };
  }

  override deserializeExtra(d: SerializedItem, _ctx: DeserializeContext) {
    if (d.chain) this.chain = deserializeChain(d.chain as any);
    this.reach = typeof d.reach === 'number' && d.reach > 0 ? d.reach : estimateReach(this.chain);
    this._joints = (d.joints as number[]) ?? homeJoints(this.chain);
    this.state.joints = [...this._joints];
    this.brand = (d.brand as RobotBrand) ?? 'Generic';
    this.model = (d.model as string) ?? '';
    this.postProcessor = (d.postProcessor as string) ?? 'Generic';
    this.connection = (d.connection as any) ?? {};
    this.motion = { ...this.motion, ...((d.motion as any) ?? {}) };
    this.motionDefaults = { ...this.motionDefaults, ...((d.motionDefaults as any) ?? (d.motion as any) ?? {}) };
    this.activeToolId = (d.activeToolId as string | null) ?? null;
    this.activeFrameId = (d.activeFrameId as string | null) ?? null;
    this.externalAxes = (d.externalAxes as string[]) ?? [];
  }
}

export function serializeChain(c: ChainDef): any {
  return {
    name: c.name,
    joints: c.joints.map((j) => ({ ...j, origin: Array.from(j.origin), post: j.post ? Array.from(j.post) : undefined })),
    links: c.links.map((l) => ({ name: l.name, visuals: l.visuals.map((v) => ({ ...v, origin: Array.from(v.origin) })), ...(l.collisions ? { collisions: l.collisions.map((g) => ({ ...g, ...(g.origin ? { origin: Array.from(g.origin) } : {}) })) } : {}) })),
    flange: Array.from(c.flange),
    base: c.base ? Array.from(c.base) : undefined,
    flangeLink: c.flangeLink,
    parallel: c.parallel ? { ...c.parallel, armAngles: [...c.parallel.armAngles], platform: c.parallel.platform ? Array.from(c.parallel.platform) : undefined } : undefined,
    jointsType: c.jointsType,
    dh: c.dh,
    dof: c.dof,
  };
}

export function deserializeChain(d: any): ChainDef {
  const c: ChainDef = {
    name: d.name,
    joints: d.joints.map((j: any) => ({ ...j, origin: fromArray(j.origin), post: j.post ? fromArray(j.post) : undefined })),
    // link collision geometry (Webots boundingObject / URDF <collision>) survives undo snapshots and .vbstation files
    links: d.links.map((l: any) => ({ name: l.name, visuals: (l.visuals ?? []).map((v: any) => ({ ...v, origin: fromArray(v.origin) })), ...(Array.isArray(l.collisions) ? { collisions: l.collisions.map((g: any) => ({ ...g })) } : {}) })),
    flange: fromArray(d.flange),
    dh: d.dh,
    dof: d.dof,
  };
  if (d.base) c.base = fromArray(d.base);
  if (typeof d.flangeLink === 'number') c.flangeLink = d.flangeLink;
  if (d.parallel) c.parallel = { ...d.parallel, platform: d.parallel.platform ? fromArray(d.parallel.platform) : undefined };
  if (typeof d.jointsType === 'number') c.jointsType = d.jointsType;
  return c;
}

/**
 * Reach (mm): the largest distance from axis 1 to the wrist centre (closed-form arms) or to the flange, sampled over
 * the joint ranges of joints 2..4 (other joints at home); parallel and short chains fall back to the link-length sum.
 */
export function estimateReach(chain: ChainDef): number {
  const idx = actuatedIndices(chain);
  if (chain.parallel?.kind === 'delta') {
    // largest radial distance of the platform centre from the motor-plane axis over the motor ranges
    const d = chain.parallel, { lower, upper } = jointLimits(chain), N = 20;
    let best = 0;
    for (let a = 0; a <= N; a++) for (let b = 0; b <= N; b++) for (let c = 0; c <= N; c++) {
      const p = deltaPlatformPosition(d, [lower[0] + ((upper[0] - lower[0]) * a) / N, lower[1] + ((upper[1] - lower[1]) * b) / N, lower[2] + ((upper[2] - lower[2]) * c) / N]);
      if (p) best = Math.max(best, Math.hypot(p[0], p[1]));
    }
    if (best > 1) return best;
  }
  if (!chain.parallel && idx.length >= 3 && chain.joints[idx[0]]?.type !== 'prismatic') {
    try {
      const { lower, upper } = jointLimits(chain);
      const home = homeJoints(chain);
      const a = analyzeChain(chain);
      const fk0 = forwardKinematics(chain, home);
      const f1 = fk0.jointFrames[idx[0]];
      const ax = [f1[8], f1[9], f1[10]], p1 = [f1[12], f1[13], f1[14]];
      const l = Math.hypot(ax[0], ax[1], ax[2]) || 1;
      ax[0] /= l; ax[1] /= l; ax[2] /= l;
      const radial = (p: number[]) => { const v = [p[0] - p1[0], p[1] - p1[1], p[2] - p1[2]]; const d = v[0] * ax[0] + v[1] * ax[1] + v[2] * ax[2]; return Math.hypot(v[0] - d * ax[0], v[1] - d * ax[1], v[2] - d * ax[2]); };
      // point measured: the wrist centre (fixed in link 3 for spherical wrists) or the flange
      const wristLink = a.kind === 'spherical6' ? idx[2] + 1 : -1;
      const wLocal = wristLink > 0 ? (() => { const L = forwardKinematics(chain, home.map(() => 0)).linkPoses[wristLink]; const inv = invert(L); const w = a.wA; return [inv[0] * w[0] + inv[4] * w[1] + inv[8] * w[2] + inv[12], inv[1] * w[0] + inv[5] * w[1] + inv[9] * w[2] + inv[13], inv[2] * w[0] + inv[6] * w[1] + inv[10] * w[2] + inv[14]]; })() : null;
      let best = 0;
      const N = 36;
      const span = (k: number) => [Math.max(lower[k], -180), Math.min(upper[k], 180)];
      const [l2, u2] = span(1), [l3, u3] = span(2);
      for (let i = 0; i <= N; i++) for (let j = 0; j <= N; j++) {
        const q = home.slice();
        q[1] = l2 + ((u2 - l2) * i) / N; q[2] = l3 + ((u3 - l3) * j) / N;
        const fk = forwardKinematics(chain, q);
        let p: number[];
        if (wLocal) { const L = fk.linkPoses[wristLink]; p = [L[0] * wLocal[0] + L[4] * wLocal[1] + L[8] * wLocal[2] + L[12], L[1] * wLocal[0] + L[5] * wLocal[1] + L[9] * wLocal[2] + L[13], L[2] * wLocal[0] + L[6] * wLocal[1] + L[10] * wLocal[2] + L[14]]; }
        else p = [fk.flange[12], fk.flange[13], fk.flange[14]];
        best = Math.max(best, radial(p));
      }
      if (best > 1) return best;
    } catch { /* fall back */ }
  }
  let r = 0;
  for (const j of chain.joints) {
    r += Math.hypot(j.origin[12], j.origin[13], j.origin[14]);
    if (j.post) r += Math.hypot(j.post[12], j.post[13], j.post[14]);
    if (j.type === 'prismatic') r += Math.max(Math.abs(j.upper), Math.abs(j.lower));
  }
  r += Math.hypot(chain.flange[12], chain.flange[13], chain.flange[14]);
  if (chain.parallel) r = chain.parallel.baseRadius + chain.parallel.upperArm + chain.parallel.lowerArm - chain.parallel.platformRadius;
  return r;
}

// Robot deserialization needs a chain: register a factory that creates a placeholder chain.
registerItemType(ItemType.ROBOT, (n, id) => new Robot(n, { name: 'empty', joints: [], links: [{ name: 'base', visuals: [] }], flange: identity() }, id));

export { clone as clonePose };
