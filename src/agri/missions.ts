/**
 * Mission planning for agricultural fleets: converts a MissionItem into fleet tasks (row work paths)
 * and, for harvesting, into arm programs (fruit approach/pick/retreat targets) for a mounted robot.
 */
import { Station, Frame, Target, ItemType } from '../core/items/item';
import { Robot } from '../core/items/robot';
import { Program } from '../core/items/program';
import { FieldItem, CropRow, MissionItem, MissionType } from './items';
import { fruitWorldPositions } from './orchard';
import { FleetManager, FleetTask, TaskType } from '../fleet/fleet';
import { MobileRobot } from '../mobile/items';
import { orderRowsNearest } from '../mobile/planner';
import { poseFromZ, multiply, invert, transl, mul, Mat4, getPos, sub, normalize, scale, add } from '../core/math/pose';

const TASK_OF: Record<MissionType, TaskType> = { harvest: 'harvest', spray: 'spray', mow: 'mow', prune: 'prune', scout: 'scout', transport: 'transport', pollinate: 'pollinate', weed: 'weed', thin: 'harvest', irrigate: 'custom' };
const CAP_OF: Record<MissionType, string> = { harvest: 'harvest', spray: 'spray', mow: 'mow', prune: 'prune', scout: 'scout', transport: 'transport', pollinate: 'pollinate', weed: 'weed', thin: 'harvest', irrigate: 'irrigate' };

export interface MissionPlan {
  tasks: FleetTask[];
  totalPathLength: number;
  estimatedHours: number;
  rows: number;
  fruitTargets?: number;
}

/**
 * Plan a mission: one fleet task per row side (or per row for centred implements like mowers).
 * Work paths are travel lines beside the row, in the station frame.
 */
export function planMission(station: Station, mission: MissionItem, fleet: FleetManager): MissionPlan {
  const field = mission.fieldId ? (station.findById(mission.fieldId) as FieldItem | null) : station.itemsOfType<FieldItem>(ItemType.FIELD)[0];
  if (!field) throw new Error('Mission has no field');
  const rows = (mission.rowIds.length ? mission.rowIds.map((id) => station.findById(id)).filter((r): r is CropRow => r instanceof CropRow) : field.rows());
  const abs = field.poseAbs();
  const toWorld = (p: number[]) => [abs[0] * p[0] + abs[4] * p[1] + abs[12], abs[1] * p[0] + abs[5] * p[1] + abs[13]];
  const s = mission.settings;
  const centred = mission.missionType === 'mow' || mission.missionType === 'weed' || mission.missionType === 'irrigate' || mission.missionType === 'scout';
  const sideOffset = centred ? field.crop.rowSpacing / 2 : (s.sideOffset ?? field.crop.rowSpacing / 2);
  const workSpeed = s.workSpeed ?? 500;
  // Build work lines
  const lines: Array<{ row: CropRow; side: 1 | -1; start: number[]; end: number[] }> = [];
  for (const r of rows) {
    if (centred) lines.push({ row: r, side: 1, ...r.travelLine(sideOffset) });
    else {
      lines.push({ row: r, side: 1, ...r.travelLine(sideOffset) });
      if (s.bothSides !== false) lines.push({ row: r, side: -1, ...r.travelLine(-sideOffset) });
    }
  }
  // Order by nearest neighbour from the first fleet robot
  const robots = fleet.robots();
  const from = robots.length ? [robots[0].state.x, robots[0].state.y] : toWorld(rows[0]?.start ?? [0, 0]);
  const order = orderRowsNearest(lines.map((l) => ({ start: toWorld(l.start), end: toWorld(l.end) })), from);
  const tasks: FleetTask[] = [];
  let total = 0;
  let fruitTargets = 0;
  order.forEach((li, k) => {
    const l = lines[li];
    const a = toWorld(l.start), b = toWorld(l.end);
    // alternate direction for consecutive lines to shorten headland turns
    const flip = k % 2 === 1;
    const path = flip ? [b, a] : [a, b];
    const ripe = mission.missionType === 'harvest' ? fruitWorldPositions(l.row).filter((f) => f.side === l.side).length : 0;
    fruitTargets += ripe;
    const len = Math.hypot(b[0] - a[0], b[1] - a[1]);
    total += len;
    const duration = mission.missionType === 'harvest' ? ripe * (s.secondsPerFruit ?? 6) : 0;
    const t = fleet.addTask({
      type: TASK_OF[mission.missionType],
      location: [path[0][0], path[0][1]],
      workPath: path,
      workSpeed: mission.missionType === 'harvest' && ripe > 0 ? Math.max(50, len / Math.max(1, duration)) : workSpeed,
      priority: s.priority ?? 1,
      requiredCapabilities: [CAP_OF[mission.missionType]],
      // traffic segment = the alley the robot drives in (shared by the two rows bordering it)
      segments: [centred ? (l.row.segmentId || l.row.id) : `${field.id}:alley${l.row.index + (l.side > 0 ? 0 : -1)}`],
      meta: { missionId: mission.id, rowId: l.row.id, side: l.side, fruit: ripe, fruitKg: fruitMassKg(field.crop.fruitDiameter), swathM: (centred ? field.crop.rowSpacing : sideOffset) / 1000 },
    });
    tasks.push(t);
  });
  installHarvestHook(station, fleet);
  mission.taskIds = tasks.map((t) => t.id);
  mission.status = 'planned';
  const est = robots.length ? (total / workSpeed + fruitTargets * (s.secondsPerFruit ?? 6)) / robots.length / 3600 : 0;
  return { tasks, totalPathLength: total, estimatedHours: est, rows: rows.length, fruitTargets };
}

/** Approximate fruit mass from diameter (sphere, density ~0.9 g/cm³). */
export function fruitMassKg(diameterMm: number): number {
  const r = diameterMm / 20; // cm
  return (4 / 3) * Math.PI * r * r * r * 0.9 / 1000;
}

/** Install the fleet hook that depletes fruit on rows when harvest tasks finish. */
export function installHarvestHook(station: Station, fleet: FleetManager): void {
  fleet.onHarvestDone = (task) => {
    const row = task.meta?.rowId ? (station.findById(task.meta.rowId) as CropRow | null) : null;
    if (!row) return { fruit: task.meta?.fruit ?? 0, kg: (task.meta?.fruit ?? 0) * (task.meta?.fruitKg ?? 0.18) };
    let n = 0;
    const side = task.meta?.side ?? 0;
    for (const p of row.plants) for (const f of p.fruit) {
      if (f.picked || f.ripe < 0.5) continue;
      if (side && (f.p[1] >= 0 ? 1 : -1) !== side) continue;
      f.picked = true;
      n++;
    }
    row.notify('plants');
    (row.parent as FieldItem | null)?.notify('rows');
    return { fruit: n, kg: n * (task.meta?.fruitKg ?? 0.18) };
  };
}

/** Update mission progress from the fleet task states. */
export function updateMissionProgress(mission: MissionItem, fleet: FleetManager): void {
  const tasks = fleet.fleet.tasks.filter((t) => mission.taskIds.includes(t.id));
  if (!tasks.length) return;
  const done = tasks.filter((t) => t.status === 'done').length;
  mission.progress = done / tasks.length;
  if (mission.progress >= 1) mission.status = 'done';
  else if (tasks.some((t) => t.status !== 'pending')) mission.status = 'running';
}

export interface HarvestProgramOptions {
  /** Approach distance along the picking direction (mm). */
  approach?: number;
  /** Max reach filter (mm) from robot base. */
  maxReach?: number;
  /** Robot base position relative to the fruit is taken from the current robot pose. */
  limit?: number;
  /** Bin/drop pose in the robot's active frame (or null to skip). */
  dropPose?: Mat4 | null;
  /** Direction from which fruit is approached: 'horizontal' (from the inter-row) or 'radial' (from canopy centre). */
  approachMode?: 'horizontal' | 'radial';
  /** Max number of IK attempts before giving up (bounds planning time). */
  maxAttempts?: number;
}

/**
 * Generate a picking program for an arm at its current pose: for each reachable ripe fruit,
 * approach -> pick (close gripper) -> retreat -> (drop). Targets are created under a frame.
 */
export function generateHarvestProgram(station: Station, robot: Robot, rows: CropRow[], opts: HarvestProgramOptions = {}): { program: Program; picked: number; skipped: number } {
  const approach = opts.approach ?? 150;
  const maxReach = opts.maxReach ?? robot.reach * 0.95;
  const limit = opts.limit ?? 200;
  const base = robot.poseAbs();
  const basePos = getPos(base);
  const frame = station.addChild(new Frame(`${robot.name} picking targets`));
  frame.setPoseAbs(base);
  robot.setFrame(frame);
  const prog = station.addChild(new Program(`Harvest ${robot.name}`));
  prog.setRobot(robot);
  prog.setSpeed(600, 90);
  prog.setRounding(20);
  const startQ = robot.joints();
  const home = frame.addChild(new Target('Home'));
  home.setJoints(startQ);
  home.setAsJointTarget();
  prog.addMoveJ(home);
  let picked = 0, skipped = 0;
  const candidates: Array<{ p: [number, number, number]; fruit: any; dist: number; side: 1 | -1; row: CropRow }> = [];
  for (const row of rows) for (const f of fruitWorldPositions(row)) {
    const dist = Math.hypot(f.p[0] - basePos[0], f.p[1] - basePos[1], f.p[2] - basePos[2]);
    if (dist <= maxReach) candidates.push({ ...f, dist, row });
  }
  candidates.sort((a, b) => a.dist - b.dist);
  let q = startQ;
  let n = 0;
  let attempts = 0;
  const maxAttempts = opts.maxAttempts ?? limit * 4;
  for (const c of candidates) {
    if (n >= limit || attempts++ >= maxAttempts) break;
    // approach direction: horizontal from robot towards fruit (typical for side-picking platforms)
    let dir: [number, number, number];
    if ((opts.approachMode ?? 'horizontal') === 'horizontal') dir = normalize([c.p[0] - basePos[0], c.p[1] - basePos[1], 0]);
    else dir = normalize(sub(c.p, [basePos[0], basePos[1], c.p[2]]));
    const pickAbs = poseFromZ(c.p, dir, [0, 0, -1]);
    const approachAbs = mul(pickAbs, transl(0, 0, -approach));
    const inFrame = (m: Mat4) => multiply(invert(frame.poseAbs()), m);
    const r1 = robot.solveIK(multiply(invert(base), approachAbs), { seed: q, restarts: 1, maxIterations: 80 });
    if (!r1.ok) { skipped++; continue; }
    const r2 = robot.solveIK(multiply(invert(base), pickAbs), { seed: r1.joints, restarts: 0, maxIterations: 60 });
    if (!r2.ok) { skipped++; continue; }
    n++;
    const tA = frame.addChild(new Target(`Approach ${n}`));
    tA.setPose(inFrame(approachAbs));
    tA.setJoints(r1.joints);
    const tP = frame.addChild(new Target(`Pick ${n}`));
    tP.setPose(inFrame(pickAbs));
    tP.setJoints(r2.joints);
    prog.addMoveJ(tA);
    prog.addMoveL(tP);
    prog.event('gripper_close');
    prog.pause(300);
    prog.addMoveL(tA);
    if (opts.dropPose) {
      const tD = frame.addChild(new Target(`Drop ${n}`));
      tD.setPose(opts.dropPose);
      prog.addMoveJ(tD);
      prog.event('gripper_open');
    } else prog.event('gripper_open');
    q = r1.joints;
    picked++;
    c.fruit.picked = true;
  }
  prog.addMoveJ(home);
  return { program: prog, picked, skipped };
}

/** Spray mission helper: emit a program for a boom sprayer mounted on a mobile robot (signals on/off along rows). */
export function generateSprayProgram(station: Station, robot: MobileRobot, rows: CropRow[], sideOffset: number, speed = 800): Program {
  const prog = station.addChild(new Program(`Spray ${robot.name}`));
  prog.setRobot(robot);
  const field = rows[0]?.parent as FieldItem | undefined;
  const abs = field ? field.poseAbs() : transl(0, 0, 0);
  const toWorld = (p: number[]) => [abs[0] * p[0] + abs[4] * p[1] + abs[12], abs[1] * p[0] + abs[5] * p[1] + abs[13]];
  rows.forEach((r, i) => {
    const line = r.travelLine(i % 2 === 0 ? sideOffset : -sideOffset);
    const a = toWorld(i % 2 === 0 ? line.start : line.end), b = toWorld(i % 2 === 0 ? line.end : line.start);
    prog.navigateTo(a[0], a[1], undefined, { speed, label: `${r.name} entry` });
    prog.addInstruction({ kind: 'signal', signal: 'spray', value: true, wait: false });
    prog.navigateTo(b[0], b[1], undefined, { speed, label: `${r.name} exit` });
    prog.addInstruction({ kind: 'signal', signal: 'spray', value: false, wait: false });
  });
  return prog;
}

export { add, scale };
