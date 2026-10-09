/**
 * Post-processing for stations exported from RoboDK with python/rdk_export.py:
 *  - robots arrive as placeholders with params { library_hint, joints, lower, upper, dh? } → attach a kinematic
 *    chain from the DH table if present, else the closest library robot (by name tokens);
 *  - programs arrive with params.instructions (RoboDK Instruction() tuples) → real Instruction children;
 *  - active frame/tool names are resolved to ids.
 */
import { Station, Item, ItemType, Target, Tool, Frame, SceneObject } from '../../core/items/item';
import { generateCurveFollow, generatePointFollow } from '../../core/motion/pathfollow';
import { Robot } from '../../core/items/robot';
import { Program } from '../../core/items/program';
import { ROBOT_LIBRARY } from '../../core/items/library';
import { chainFromDH, DHParams } from '../../core/kinematics/chain';
import { fromArray, identity } from '../../core/math/pose';

export interface RdkImportReport {
  robots: Array<{ name: string; matched: string | null; source: 'dh' | 'library' | 'none' }>;
  programs: Array<{ name: string; instructions: number; jointPath?: number }>;
  machining: Array<{ name: string; part: string | null; program: string | null; generated: boolean; points?: number }>;
  pythonPrograms: Array<{ name: string; hasSource: boolean }>;
}

/** Score how well a library entry matches a RoboDK robot name (e.g. "UR10e", "KUKA KR 6 R900 sixx"). */
function matchScore(rdkName: string, entryName: string, entryId: string): number {
  const norm = (s: string) => s.toLowerCase().replace(/[^a-z0-9]+/g, ' ').trim();
  const a = norm(rdkName), b = norm(entryName), c = norm(entryId);
  if (!a) return 0;
  if (b === a || c === a) return 100;
  const tokens = a.split(' ').filter((t) => t.length > 1);
  let score = 0;
  for (const t of tokens) if (b.includes(t) || c.includes(t)) score += t.length * (/\d/.test(t) ? 3 : 1);
  return score;
}

export function isRdkExport(station: Station): boolean {
  return station.params.source === 'rdk_export.py' || [...station.walk()].some((i) => i.params.rdk_type !== undefined);
}

export function postProcessRdkExport(station: Station): RdkImportReport {
  const report: RdkImportReport = { robots: [], programs: [], machining: [], pythonPrograms: [] };
  // Robots
  for (const r of station.itemsOfType<Robot>(ItemType.ROBOT)) {
    if (r.chain.joints.length && r.chain.name !== 'empty') continue;
    const hint = String(r.params.library_hint ?? r.name);
    const dh = r.params.dh as any;
    let source: 'dh' | 'library' | 'none' = 'none';
    let matched: string | null = null;
    if (Array.isArray(dh) && dh.length >= 3 && Array.isArray(dh[0])) {
      // RoboDK JointsDH rows: [theta, d, a, alpha] (deg/mm) in some builds; accept 4+ numeric columns
      const rows: DHParams[] = dh.map((row: number[], i: number) => ({ theta: +row[0] || 0, d: +row[1] || 0, a: +row[2] || 0, alpha: +row[3] || 0, lower: (r.params.lower as number[])?.[i] ?? -180, upper: (r.params.upper as number[])?.[i] ?? 180 }));
      r.chain = chainFromDH(r.name, rows);
      source = 'dh';
    } else {
      let best: (typeof ROBOT_LIBRARY)[number] | null = null, bs = 0;
      for (const e of ROBOT_LIBRARY) { const s = matchScore(hint, e.name, e.id); if (s > bs) { bs = s; best = e; } }
      if (best && bs >= 3) {
        r.chain = best.build();
        r.brand = best.brand;
        r.model = best.name;
        r.postProcessor = best.postProcessor;
        matched = best.id;
        source = 'library';
      }
    }
    if (source !== 'none') {
      const lower = r.params.lower as number[] | undefined, upper = r.params.upper as number[] | undefined;
      if (lower && upper && lower.length === r.dof) r.setJointLimits(lower, upper);
      const q = r.params.joints as number[] | undefined;
      if (q && q.length >= r.dof) r.setJoints(q.slice(0, r.dof));
      (r as any).reach = Math.max(r.reach, 1);
    }
    const af = r.params.activeFrame as string | null, at = r.params.activeTool as string | null;
    if (af) { const f = station.find(af, ItemType.FRAME); if (f) r.setFrame(f); }
    if (at) { const t = r.children.find((c) => c instanceof Tool && c.name === at) ?? station.find(at, ItemType.TOOL); if (t instanceof Tool) r.setTool(t); }
    report.robots.push({ name: r.name, matched, source });
  }
  // Programs
  for (const p of station.itemsOfType<Program>(ItemType.PROGRAM)) {
    const ins = p.params.instructions as any[] | undefined;
    if (!ins || p.instructions().length) continue;
    const robotName = p.params.robotName as string | undefined;
    const robot = robotName ? station.find(robotName, ItemType.ROBOT) : station.itemsOfType(ItemType.ROBOT)[0];
    if (robot) p.setRobot(robot);
    const frame = robot instanceof Robot ? robot.activeFrame() ?? station : station;
    let n = 0;
    for (const e of ins) {
      // RoboDK INS_TYPE: 0 invalid, 1 move, 2 movec, 3 changespeed, 4 changeframe, 5 changetool, 6 changerobot, 7 pause, 8 event, 9 code, 10 print
      const type = Number(e.type);
      if (type === 1 || type === 2) {
        const t = new Target(e.name || `T${++n}`);
        if (e.pose) t.setPose(fromArray(e.pose));
        if (e.joints?.length) t.setJoints(e.joints);
        if (e.isJointTarget) t.setAsJointTarget();
        frame.addChild(t);
        const mt = Number(e.moveType);
        if (type === 2 || mt === 3) p.addMoveL(t); // circular imported as linear (via point follows)
        else if (mt === 2) p.addMoveL(t);
        else p.addMoveJ(t);
      } else if (type === 7) p.pause(Number(e.pauseMs ?? parseFirstNumber(e.name, 0) * 1000));
      else if (type === 10) p.comment(String(e.name ?? ''));
      else if (type === 9) p.runInstruction(String(e.name ?? ''), true);
      else if (type === 3) {
        // RoboDK names speed instructions e.g. "Set Speed (250.0 mm/s)" / "Set Joint Speed (50 deg/s)"
        const v = parseFirstNumber(e.name, NaN);
        if (/joint|deg/i.test(String(e.name)) && !isNaN(v)) p.setSpeed(undefined, v);
        else if (/accel/i.test(String(e.name)) && !isNaN(v)) p.setSpeed(undefined, undefined, v);
        else p.setSpeed(!isNaN(v) ? v : Number(e.speedLinear) || undefined, Number(e.speedJoints) || undefined);
      } else if (type === 4) {
        const f = station.find(frameNameOf(e.name), ItemType.FRAME);
        if (f) p.setFrame(f); else p.comment(`Set reference ${e.name}`);
      } else if (type === 5) {
        const t = station.find(frameNameOf(e.name), ItemType.TOOL);
        if (t) p.setTool(t); else p.comment(`Set tool ${e.name}`);
      } else if (type === 8) p.comment(`Event: ${e.name ?? ''}`);
      else p.comment(`${e.name ?? ''} (RoboDK instruction type ${type})`);
    }
    // program-level frame / tool (used when the instruction list has no explicit change)
    const fn = p.params.frameName as string | undefined, tn = p.params.toolName as string | undefined;
    if (fn && !p.instructions().some((i) => i.data.kind === 'frame')) { const f = station.find(fn, ItemType.FRAME); if (f) p.addInstruction({ kind: 'frame', frameId: f.id }, 0); }
    if (tn && !p.instructions().some((i) => i.data.kind === 'tool')) { const t = station.find(tn, ItemType.TOOL); if (t) p.addInstruction({ kind: 'tool', toolId: t.id }, 0); }
    delete p.params.instructions;
    const jl = p.params.jointsList as number[][] | undefined;
    // Robots whose kinematics could not be reconstructed still replay RoboDK's simulated joint path exactly
    const robotItem = p.robot();
    const noKin = robotItem instanceof Robot && report.robots.find((r) => r.name === robotItem.name)?.source === 'none';
    if (jl && jl.length > 1 && noKin) {
      for (const i of p.instructions()) if (i.data.kind === 'move') i.delete();
      const step = Number(p.params.jointsListStepMm ?? 10);
      p.addInstruction({ kind: 'jointPath', joints: jl, source: `RoboDK ${step} mm` });
    }
    report.programs.push({ name: p.name, instructions: p.instructions().length, jointPath: jl?.length });
  }
  // Machining projects (curve follow / point follow / 3D printing / milling): link robot, part and generated program
  for (const m of station.itemsOfType(ItemType.MACHINING)) {
    const robot = m.params.robotName ? station.find(String(m.params.robotName), ItemType.ROBOT) : station.itemsOfType(ItemType.ROBOT)[0];
    const part = m.params.partName ? station.find(String(m.params.partName), ItemType.OBJECT) : null;
    const prog = m.params.programName ? station.find(String(m.params.programName), ItemType.PROGRAM) : null;
    if (robot) m.setParam('robotId', robot.id);
    if (part) m.setParam('partId', part.id);
    if (prog) m.setParam('programId', prog.id);
    let generated = false, points: number | undefined;
    if (!prog && robot instanceof Robot && part instanceof SceneObject && (part.curves.length || part.points.length)) {
      try {
        const res = part.curves.length
          ? generateCurveFollow(station, robot, part, { points: part.curves.flatMap((c) => c.points) }, { name: m.name })
          : generatePointFollow(station, robot, part, part.points.map((p) => ({ point: p.point, normal: (p as any).normal })), { name: m.name });
        m.setParam('programId', res.program.id);
        generated = true;
        points = res.points;
      } catch { /* leave unlinked */ }
    }
    report.machining.push({ name: m.name, part: part?.name ?? null, program: prog?.name ?? null, generated, points });
  }
  for (const py of station.itemsOfType(ItemType.PROGRAM_PYTHON)) report.pythonPrograms.push({ name: py.name, hasSource: !!py.params.source });
  return report;
}

function parseFirstNumber(text: unknown, def: number): number {
  const m = String(text ?? '').match(/-?\d+(?:\.\d+)?/);
  return m ? parseFloat(m[0]) : def;
}
/** "Set Ref.: Frame 2" / "Set Tool: Gripper" -> "Frame 2" / "Gripper" */
function frameNameOf(text: unknown): string {
  const s = String(text ?? '');
  const m = s.match(/:\s*(.+)$/) ?? s.match(/\((.+)\)\s*$/);
  return (m ? m[1] : s).trim();
}

export { Item, Frame, identity };
