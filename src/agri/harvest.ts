/**
 * Mobile-manipulation harvesting: arm reach and mount on the carrier, fruit references, pick programs built for the
 * arm at its *current* base pose (targets in a frame that moves with the carrier, regenerated per stop), approach and
 * IK solution checked against the carrier, the arm itself and the rigid crop (trunks / leaders), targets from vision
 * detections when given (else ground truth, flagged), and fruit removed when the gripper closes during simulation.
 */
import { Station, Frame, Target, ItemType, Item } from '../core/items/item';
import { Robot } from '../core/items/robot';
import { Program } from '../core/items/program';
import { MobileRobot } from '../mobile/items';
import { CropRow, PlantRecord } from './items';
import { fruitWorldPositions } from './orchard';
import { cropColliders } from './collision_world';
import { robotCollisionChecker } from '../core/collision/collision';
import { poseFromZ, multiply, invert, transl, mul, Mat4, getPos, normalize, sub } from '../core/math/pose';

/** Reference of one fruit: `fruit:<rowId>:<plant>:<index>`. */
export function fruitRef(row: CropRow, plant: number, fruit: number): string { return `fruit:${row.id}:${plant}:${fruit}`; }

export function fruitByRef(station: Station, ref: string): { row: CropRow; plant: PlantRecord; fruit: PlantRecord['fruit'][number] } | null {
  const m = /^fruit:(.+):(\d+):(\d+)$/.exec(ref);
  if (!m) return null;
  const row = station.findById(m[1]);
  if (!(row instanceof CropRow)) return null;
  const plant = row.plants[+m[2]];
  const fruit = plant?.fruit[+m[3]];
  return plant && fruit ? { row, plant, fruit } : null;
}

/** World positions of a row's fruit with their references. */
export function fruitTargets(row: CropRow, onlyRipe = true): Array<{ p: [number, number, number]; ref: string; side: 1 | -1; fruit: PlantRecord['fruit'][number] }> {
  const out: Array<{ p: [number, number, number]; ref: string; side: 1 | -1; fruit: PlantRecord['fruit'][number] }> = [];
  const all = fruitWorldPositions(row, false);
  let k = 0;
  row.plants.forEach((pl, pi) => pl.fruit.forEach((f, fi) => {
    const w = all[k++];
    if (onlyRipe && (f.ripe < 0.5 || f.picked)) return;
    out.push({ p: w.p, ref: fruitRef(row, pi, fi), side: w.side, fruit: f });
  }));
  return out;
}

const reachCache = new Map<string, number>();
/**
 * Real reach of an arm (mm): max distance from the base origin to the TCP over sampled joint configurations (the
 * catalogue `robot.reach` is a sum of link offsets and overestimates, e.g. UR10e 1775 vs ~1300).
 */
export function armReach(arm: Robot): number {
  const key = `${arm.brand}:${arm.model || arm.name}:${arm.dof}:${arm.reach}`;
  const c = reachCache.get(key);
  if (c) return c;
  const { lower, upper } = arm.jointLimits();
  let best = 0;
  let seed = 12345;
  const rnd = () => { seed = (seed * 16807) % 2147483647; return seed / 2147483647; };
  const flange = invert(arm.poseTool());
  void flange;
  for (let i = 0; i < 1500; i++) {
    const q = lower.map((lo, k) => lo + (upper[k] - lo) * rnd());
    const p = getPos(arm.solveFKFlange(q));
    best = Math.max(best, Math.hypot(p[0], p[1], p[2] - 0));
  }
  // horizontal reach measured from the shoulder height is what matters for side picking: use the flange envelope
  const r = Math.min(best, arm.reach || best);
  reachCache.set(key, r);
  return r;
}

/** Arm mounted on a carrier: mount offset in the carrier frame (mm) and the side it faces (sign of the mount y). */
export function armMount(arm: Robot): { x: number; y: number; z: number; side: 1 | -1 } {
  const m = arm.pose();
  return { x: m[12], y: m[13], z: m[14], side: m[13] >= 0 ? 1 : -1 };
}

/** First arm (Robot child) of a carrier. */
export function armOf(carrier: Item): Robot | null {
  return (carrier.children.find((c) => c instanceof Robot) as Robot | undefined) ?? null;
}

export interface HarvestProgramOptions {
  /** Approach distance along the picking direction (mm). */
  approach?: number;
  /** Max reach filter (mm) from robot base (default: 0.92 x measured reach). */
  maxReach?: number;
  /** Max number of picks. */
  limit?: number;
  /** Bin/drop pose relative to the arm base (or null: open the gripper after the retreat). */
  dropPose?: Mat4 | null;
  /** Direction from which fruit is approached: 'horizontal' (from the inter-row) or 'radial' (from canopy centre). */
  approachMode?: 'horizontal' | 'radial';
  /** Max number of IK attempts before giving up (bounds planning time). */
  maxAttempts?: number;
  /** Reject configurations colliding with the carrier, the arm itself or the rigid crop (default true). */
  collisionCheck?: boolean;
  /** Fruit positions from perception (world mm); default: ground truth from the crop model (flagged in the program). */
  detections?: Array<{ p: [number, number, number]; ref?: string }>;
  /** Only fruit on this side of the rows (relative to row direction). */
  side?: 1 | -1;
}

export interface HarvestProgramResult {
  program: Program;
  /** Number of planned picks (fruit are removed only when the program runs, see bindHarvestEvents). */
  picked: number;
  skipped: number;
  /** Planned fruit references in pick order ('fruit:none' for detections that match no fruit). */
  targets: string[];
  /** 'vision' or 'ground_truth'. */
  source: string;
  /** Rejections because of collisions. */
  collisionRejects: number;
}

/**
 * Generate (or regenerate) a picking program for an arm at its current base pose: for each reachable fruit,
 * approach → pick (close gripper, event carries the fruit reference) → retreat → drop. Targets live in a frame that is
 * a sibling of the arm with the arm's mount pose, so they move with a mobile carrier; the frame and program are reused
 * by name, so calling this again at the next stop re-solves the program instead of piling up items.
 */
export function generateHarvestProgram(station: Station, robot: Robot, rows: CropRow[], opts: HarvestProgramOptions = {}): HarvestProgramResult {
  const approach = opts.approach ?? 150;
  const maxReach = opts.maxReach ?? armReach(robot) * 0.92;
  const limit = opts.limit ?? 200;
  const base = robot.poseAbs();
  const basePos = getPos(base);
  const parent = robot.parent ?? station;
  const frameName = `${robot.name} picking targets`;
  let frame = parent.children.find((c) => c instanceof Frame && c.name === frameName) as Frame | undefined;
  if (!frame) frame = parent.addChild(new Frame(frameName));
  for (const c of [...frame.children]) c.delete();
  frame.setPose(robot.pose());
  robot.setFrame(frame);
  const progName = `Harvest ${robot.name}`;
  let prog = station.find(progName, ItemType.PROGRAM) as Program | null;
  if (prog) for (const i of prog.instructions()) i.delete(); else prog = station.addChild(new Program(progName));
  prog.setRobot(robot);
  prog.setSpeed(600, 90);
  prog.setRounding(20);
  const source = opts.detections ? 'vision' : 'ground_truth';
  prog.setParam('fruitSource', source);
  prog.comment(`Harvest targets from ${source === 'vision' ? 'camera detections' : 'GROUND TRUTH crop model (no perception)'}`);
  const startQ = robot.joints();
  const home = frame.addChild(new Target('Home'));
  home.setJoints(startQ);
  home.setAsJointTarget();
  prog.addMoveJ(home);
  // candidates
  type Cand = { p: [number, number, number]; ref: string; dist: number };
  const cands: Cand[] = [];
  if (opts.detections) for (const d of opts.detections) cands.push({ p: d.p, ref: d.ref ?? 'fruit:none', dist: Math.hypot(d.p[0] - basePos[0], d.p[1] - basePos[1], d.p[2] - basePos[2]) });
  else for (const row of rows) for (const f of fruitTargets(row)) if (!opts.side || f.side === opts.side) cands.push({ p: f.p, ref: f.ref, dist: Math.hypot(f.p[0] - basePos[0], f.p[1] - basePos[1], f.p[2] - basePos[2]) });
  const reachable = cands.filter((c) => c.dist <= maxReach).sort((a, b) => a.dist - b.dist);
  // collision context: the same checker the program simulator uses (arm links + tool against themselves, the carrier
  // and the rest of the station, collision map honoured) plus the rigid crop near the arm; joint paths are sampled
  // every 2 deg like the simulator's timeline check
  const check = opts.collisionCheck !== false;
  let crop: Item | null = null;
  let checker: { check(version?: number): unknown[] } | null = null;
  let baseline = 0;
  if (check) {
    crop = cropColliders(station, [basePos[0], basePos[1]], maxReach + 1500, `${robot.name} crop colliders`);
    checker = robotCollisionChecker(station, robot, {});
    robot.setJoints(startQ);
    baseline = checker.check(0).length;
  }
  const collides = (qq: number[]) => {
    if (!checker) return false;
    robot.setJoints(qq);
    return checker.check(0).length > baseline;
  };
  const pathCollides = (q0: number[], q1: number[]) => {
    if (!checker) return false;
    const n = Math.max(2, Math.ceil(Math.max(...q0.map((v, k) => Math.abs(q1[k] - v))) / 2));
    for (let i = 1; i <= n; i++) { const t = i / n; if (collides(q0.map((v, k) => v + (q1[k] - v) * t))) return true; }
    return false;
  };
  let picked = 0, skipped = 0, attempts = 0, rejects = 0;
  const maxAttempts = opts.maxAttempts ?? limit * 4;
  const targets: string[] = [];
  let q = startQ;
  const inFrame = (m: Mat4) => multiply(invert(frame!.poseAbs()), m);
  for (const c of reachable) {
    if (picked >= limit || attempts++ >= maxAttempts) break;
    const dir: [number, number, number] = (opts.approachMode ?? 'horizontal') === 'horizontal' ? normalize([c.p[0] - basePos[0], c.p[1] - basePos[1], 0]) : normalize(sub(c.p, [basePos[0], basePos[1], c.p[2]]));
    const pickAbs = poseFromZ(c.p, dir, [0, 0, -1]);
    const approachAbs = mul(pickAbs, transl(0, 0, -approach));
    let ok = false;
    let qa: number[] = [], qp: number[] = [];
    for (const seed of [q, startQ]) {
      const r1 = robot.solveIK(multiply(invert(base), approachAbs), { seed, restarts: 1, maxIterations: 80 });
      if (!r1.ok) continue;
      const r2 = robot.solveIK(multiply(invert(base), pickAbs), { seed: r1.joints, restarts: 0, maxIterations: 60 });
      if (!r2.ok) continue;
      // approach, pick, the joint moves in between and the way back home must all be free
      if (collides(r1.joints) || collides(r2.joints) || pathCollides(q, r1.joints) || pathCollides(r1.joints, r2.joints) || pathCollides(r1.joints, startQ)) { rejects++; continue; }
      qa = r1.joints; qp = r2.joints; ok = true;
      break;
    }
    if (!ok) { skipped++; continue; }
    picked++;
    const tA = frame.addChild(new Target(`Approach ${picked}`));
    tA.setPose(inFrame(approachAbs));
    tA.setJoints(qa);
    const tP = frame.addChild(new Target(`Pick ${picked}`));
    tP.setPose(inFrame(pickAbs));
    tP.setJoints(qp);
    prog.addMoveJ(tA);
    prog.addMoveL(tP);
    // the fruit is not a scene object: the explicit (non-item) object reference keeps the simulator from attaching
    // the nearest object (the crop colliders) to the gripper; bindHarvestEvents removes the fruit by its name
    prog.event('gripper_close', c.ref, c.ref);
    prog.pause(300);
    prog.addMoveL(tA);
    if (opts.dropPose) {
      const tD = frame.addChild(new Target(`Drop ${picked}`));
      tD.setPose(opts.dropPose);
      prog.addMoveJ(tD);
    }
    prog.event('gripper_open');
    targets.push(c.ref);
    q = qa;
  }
  prog.addMoveJ(home);
  robot.setJoints(startQ);
  return { program: prog, picked, skipped, targets, source, collisionRejects: rejects };
}

/**
 * Remove fruit when the simulator closes the gripper on them (event name = fruit reference). Returns a handle with the
 * counters and an unsubscribe function. A closing on a reference that is gone (already picked, false detection)
 * counts as a miss.
 */
export function bindHarvestEvents(sim: { events: { on: (ev: 'instruction', fn: (p: { instruction: any }) => void) => () => void } }, station: Station, onPick?: (ref: string) => void): { picked: number; missed: number; off: () => void } {
  const h = { picked: 0, missed: 0, off: () => {} };
  h.off = sim.events.on('instruction', ({ instruction }) => {
    const d = instruction?.data ?? instruction;
    if (!d || d.kind !== 'event' || d.action !== 'gripper_close' || typeof d.name !== 'string' || !d.name.startsWith('fruit:')) return;
    const f = fruitByRef(station, d.name);
    if (!f || f.fruit.picked) { h.missed++; return; }
    f.fruit.picked = true;
    h.picked++;
    f.row.notify('plants');
    onPick?.(d.name);
  });
  return h;
}

/** Match perception detections (world positions) to crop fruit within `tol` (mm): references for the pick events. */
export function matchDetections(station: Station, detections: Array<{ p: [number, number, number] }>, tol = 60): Array<{ p: [number, number, number]; ref: string }> {
  const all: Array<{ p: [number, number, number]; ref: string }> = [];
  for (const row of station.itemsOfType<CropRow>(ItemType.CROP_ROW)) for (const f of fruitTargets(row, false)) if (!f.fruit.picked) all.push({ p: f.p, ref: f.ref });
  return detections.map((d) => {
    let best = 'fruit:none', bd = tol;
    for (const f of all) { const e = Math.hypot(f.p[0] - d.p[0], f.p[1] - d.p[1], f.p[2] - d.p[2]); if (e < bd) { bd = e; best = f.ref; } }
    return { p: d.p, ref: best };
  });
}

export { MobileRobot };
