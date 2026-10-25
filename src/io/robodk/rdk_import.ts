/**
 * Stations exported from RoboDK by python/rdk_export.py (run inside RoboDK, or by python/rdk2vbs.py behind the studio
 * server's POST /convert/rdk).
 *
 *  - `importRdkExport` (lossless): the export embeds the station re-saved by RoboDK itself (`station.params.rdkFile`,
 *    base64 .rdk); it is decoded by the binary reader (rdk_binary.ts, verified against RoboDK ground truth) and
 *    cross-checked with the API walk of the same export (every robot / frame / tool / target / program RoboDK listed
 *    must exist, programs must have RoboDK's instruction count).
 *  - `postProcessRdkExport` (API walk only, older exports): robots get the closest library model of the same brand
 *    and DOF (else they replay RoboDK's joint path when exported), programs are rebuilt from the Instruction()
 *    tuples (official INS_TYPE numbering in `insType`; `type` = legacy fixtures), active frame / tool names resolved.
 *    The RoboDK API does not say whether a code instruction is a call, inserted code, a thread or a comment, nor
 *    which item an event refers to: those become code calls / comments (the binary path keeps them exactly).
 */
import { Station, Item, ItemType, Target, Tool, Frame, SceneObject } from '../../core/items/item';
import { generateCurveFollow, generatePointFollow } from '../../core/motion/pathfollow';
import { Robot } from '../../core/items/robot';
import { Program } from '../../core/items/program';
import type { InstructionData, MoveInstruction } from '../../core/items/program';
import { ROBOT_LIBRARY } from '../../core/items/library';
import { fromArray, identity } from '../../core/math/pose';
import type { AssetStore } from '../../scene/assets';
import { base64ToBytes } from '../station-file';
import { importRobodkBinary } from './rdk_binary';
import type { RobodkLoad } from './rdk_binary';

export interface RdkImportReport {
  robots: Array<{ name: string; matched: string | null; source: 'dh' | 'library' | 'none' }>;
  programs: Array<{ name: string; instructions: number; jointPath?: number }>;
  machining: Array<{ name: string; part: string | null; program: string | null; generated: boolean; points?: number }>;
  pythonPrograms: Array<{ name: string; hasSource: boolean }>;
}

const BRANDS = ['abb', 'fanuc', 'kuka', 'motoman', 'yaskawa', 'universal', 'ur', 'staubli', 'stäubli', 'kawasaki', 'denso', 'doosan', 'mecademic', 'comau', 'omron', 'jaka', 'robotiq', 'gudel', 'techman', 'nachi', 'epson', 'franka', 'kinova'];
const brandOf = (s: string) => { const t = s.toLowerCase().replace(/[^a-zäö0-9]+/g, ' ').split(' '); if (/^ur\d/.test(t[0] ?? '')) return 'ur'; const b = BRANDS.find((x) => t.includes(x)); return b === 'yaskawa' ? 'motoman' : b === 'universal' ? 'ur' : b === 'stäubli' ? 'staubli' : b ?? null; };

/** Score how well a library entry matches a RoboDK robot name (e.g. "UR10e", "KUKA KR 6 R900 sixx"); 0 = different brand. */
function matchScore(rdkName: string, entryName: string, entryId: string): number {
  const norm = (s: string) => s.toLowerCase().replace(/[^a-z0-9]+/g, ' ').trim();
  const a = norm(rdkName), b = norm(entryName), c = norm(entryId);
  if (!a) return 0;
  if (b === a || c === a) return 100;
  const ba = brandOf(rdkName), bb = brandOf(entryName) ?? brandOf(entryId);
  if (ba && bb && ba !== bb) return 0;
  const tokens = a.split(' ').filter((t) => t.length > 1);
  const have = new Set([...b.split(' '), ...c.split(' ')]);
  let score = 0, numeric = 0;
  // whole tokens only: "6" must not match "16", "120" must not match "1200"
  for (const t of tokens) if (have.has(t)) { score += t.length * (/\d/.test(t) ? 3 : 1); if (/\d/.test(t)) numeric++; }
  // a model number must match (KR 6 vs KR 16, IRB 120 vs IRB 1600): without one only a brand match, not enough
  return numeric ? score : 0;
}

export function isRdkExport(station: Station): boolean {
  return station.params.source === 'rdk_export.py' || [...station.walk()].some((i) => i.params.rdk_type !== undefined);
}

/**
 * Lossless import of an rdk_export.py / rdk2vbs.py file: decode the embedded RoboDK-saved .rdk and cross-check it
 * with the API walk. Returns null when the export carries no embedded station (use loadStation + postProcessRdkExport).
 */
export async function importRdkExport(data: unknown, assets?: AssetStore): Promise<RobodkLoad | null> {
  const root = (data as { station?: { name?: string; params?: Record<string, any>; children?: any[] } })?.station;
  const f = root?.params?.rdkFile as { name?: string; data?: string } | undefined;
  if (!f?.data) return null;
  const bytes = base64ToBytes(f.data);
  const load = await importRobodkBinary(bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength) as ArrayBuffer, f.name ?? `${root?.name ?? 'RoboDK'}.rdk`, assets);
  if (!load) return null;
  const { station, report } = load;
  // cross-check: RoboDK's own listing of the same station
  const ours = new Map<string, Item[]>();
  for (const it of station.walk()) { const k = `${it.type}|${it.name}`; const l = ours.get(k); if (l) l.push(it); else ours.set(k, [it]); }
  const missing: string[] = [];
  let checked = 0;
  // RoboDK lists the programs a program calls again under the caller: those are references, not items
  const visit = (n: any, inProgram = false) => {
    if (inProgram) return;
    if ([2, 3, 4, 5, 6, 8, 10, 11, 17].includes(n.type)) {
      checked++;
      const l = ours.get(`${n.type}|${n.name}`);
      const it = l?.shift();
      if (!it) missing.push(`${ItemType[n.type] ?? n.type} "${n.name}"`);
      else if (it instanceof Program && Array.isArray(n.params?.instructions) && it.instructions().length !== n.params.instructions.length && !it.params.machiningId) missing.push(`program "${n.name}": ${it.instructions().length} instructions, RoboDK ${n.params.instructions.length}`);
    }
    for (const c of n.children ?? []) visit(c, n.type === 8);
  };
  for (const c of root?.children ?? []) visit(c);
  report.dropped = [...(report.dropped ?? []), ...missing.map((m) => `RoboDK lists ${m}, the decoded station differs`)];
  report.fidelity = report.dropped.length ? 'partial' : 'full';
  const ver = String(root?.params?.robodkVersion ?? '');
  report.notes.unshift(`Converted by RoboDK${ver ? ` ${ver}` : ''}: station re-saved by RoboDK and decoded by the binary reader; ${checked} items cross-checked with RoboDK's API listing${missing.length ? `, ${missing.length} differences` : ''}.`);
  station.params.rdkConvertedBy = `RoboDK${ver ? ` ${ver}` : ''}`;
  station.params.rdkReport = JSON.parse(JSON.stringify(report));
  return load;
}

/** RoboDK INS_TYPE of an exported instruction: `insType` (official numbering) or `type` (legacy fixtures: move = 1). */
function insTypeOf(e: any, legacy: boolean): number {
  if (e.insType !== undefined && e.insType !== null) return Number(e.insType);
  return legacy ? Number(e.type) - 1 : Number(e.type);
}

export function postProcessRdkExport(station: Station): RdkImportReport {
  const report: RdkImportReport = { robots: [], programs: [], machining: [], pythonPrograms: [] };
  // Robots
  for (const r of station.itemsOfType<Robot>(ItemType.ROBOT)) {
    if (r.chain.joints.length && r.chain.name !== 'empty') continue;
    const hint = String(r.params.library_hint ?? r.name);
    const lower = r.params.lower as number[] | undefined, upper = r.params.upper as number[] | undefined;
    let source: 'dh' | 'library' | 'none' = 'none';
    let matched: string | null = null;
    let best: (typeof ROBOT_LIBRARY)[number] | null = null, bs = 0;
    for (const e of ROBOT_LIBRARY) { const s = matchScore(hint, e.name, e.id); if (s > bs) { bs = s; best = e; } }
    if (best && bs >= 3) {
      const chain = best.build();
      // a library model with another number of joints is a different machine (iiwa 7 axes vs KR 6, rails, grippers)
      if (!lower || lower.length === (chain.dof ?? chain.joints.length)) {
        r.chain = chain;
        r.brand = best.brand;
        r.model = best.name;
        r.postProcessor = best.postProcessor;
        matched = best.id;
        source = 'library';
      }
    }
    if (source !== 'none') {
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
    // legacy fixtures numbered move = 1; current exports write the official numbering (move = 0) in `insType`
    const legacy = !ins.some((e) => e.insType !== undefined) && !ins.some((e) => Number(e.type) === 0);
    let frame: Item = robot instanceof Robot ? robot.activeFrame() ?? station : station;
    const targetIn = (name: string): Target | null => {
      const local = frame.children.find((c): c is Target => c instanceof Target && c.name === name);
      return local ?? (station.find(name, ItemType.TARGET) as Target | null);
    };
    for (const e of ins) {
      const type = insTypeOf(e, legacy);
      const name = String(e.name ?? '');
      let d: InstructionData | null = null;
      if (type === 0 || type === 1) {
        const mt = type === 1 || Number(e.moveType) === 3 ? 'MoveC' : Number(e.moveType) === 2 ? 'MoveL' : 'MoveJ';
        const refs = (name.match(/\((.*)\)\s*$/)?.[1] ?? '').split(',').map((s) => s.trim()).filter(Boolean);
        const end = targetIn(refs[refs.length - 1] ?? '');
        const m: MoveInstruction = { kind: 'move', moveType: mt, targetId: end?.id ?? null };
        if (!end) {
          if (e.joints?.length) m.joints = e.joints;
          if (!e.isJointTarget && e.pose) m.pose = e.pose;
          if (!m.joints && !m.pose) { const t = new Target(name || 'Target'); frame.addChild(t); m.targetId = t.id; }
        }
        if (mt === 'MoveC') { const via = refs.length > 1 ? targetIn(refs[0]) : null; m.viaTargetId = via?.id ?? null; }
        d = m;
      } else if (type === 2) {
        const v = parseFirstNumber(name, NaN);
        if (!Number.isNaN(v)) d = /accel/i.test(name) ? (/deg/i.test(name) ? { kind: 'speed', accelJoints: v } : { kind: 'speed', accelLinear: v }) : /deg/i.test(name) ? { kind: 'speed', speedJoints: v } : { kind: 'speed', speedLinear: v };
      } else if (type === 3) {
        const f = station.find(frameNameOf(name), ItemType.FRAME) ?? station.find(frameNameOf(name), ItemType.ROBOT);
        if (f) { frame = f; d = { kind: 'frame', frameId: f.id }; }
      } else if (type === 4) {
        const n = frameNameOf(name);
        const t = (robot?.children.find((c) => c instanceof Tool && c.name === n) as Tool | undefined) ?? station.find(n, ItemType.TOOL);
        if (t) d = { kind: 'tool', toolId: t.id };
      } else if (type === 6) d = { kind: 'pause', timeMs: e.pauseMs !== undefined ? Number(e.pauseMs) : pauseMs(name) };
      else if (type === 8) {
        const callee = station.find(name, ItemType.PROGRAM);
        d = callee ? { kind: 'call', programId: callee.id, programName: callee.name } : { kind: 'code', code: name, asFunctionCall: true };
      } else if (type === 9) d = { kind: 'print', message: name.replace(/^Print:\s*/i, ''), isComment: false };
      else if (type === 10) d = { kind: 'rounding', radius: /accurate|fine/i.test(name) ? -1 : parseFirstNumber(name, -1) };
      else if (type === 11) {
        const io = name.match(/^(Set|Wait)\s+([^=\s]+)\s*=\s*([^\s]+)(?:\s+T([\d.]+)s)?/i);
        if (io) {
          const wait = /^wait/i.test(io[1]);
          const value = Number(io[3]);
          const ins: Record<string, unknown> = { kind: 'io', io: io[2], value: Number.isFinite(value) ? value : io[3], wait };
          if (io[4]) ins.timeoutMs = parseFloat(io[4]) * 1000;
          d = ins as unknown as InstructionData;
        }
      }
      p.addInstruction(d ?? { kind: 'print', message: type === 7 ? `Event: ${name}` : `${name} (RoboDK instruction type ${type})`, isComment: true });
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
  for (const py of station.itemsOfType(ItemType.PROGRAM_PYTHON)) report.pythonPrograms.push({ name: py.name, hasSource: !!(py.params.pythonCode ?? py.params.source) });
  return report;
}

function parseFirstNumber(text: unknown, def: number): number {
  const m = String(text ?? '').match(/-?\d+(?:\.\d+)?/);
  return m ? parseFloat(m[0]) : def;
}
/** "Pause 500 ms" -> 500, "Pause 2 s" -> 2000, "Stop" / "Pause" -> -1 (wait for the operator). */
function pauseMs(text: string): number {
  const m = text.match(/(-?\d+(?:\.\d+)?)\s*(ms|s)?\b/i);
  if (!m) return -1;
  return parseFloat(m[1]) * (m[2]?.toLowerCase() === 's' ? 1000 : 1);
}
/** "Set Ref.: Frame 2" / "Set Tool: Gripper" -> "Frame 2" / "Gripper" */
function frameNameOf(text: unknown): string {
  const s = String(text ?? '');
  const m = s.match(/:\s*(.+)$/) ?? s.match(/\((.+)\)\s*$/);
  return (m ? m[1] : s).trim();
}

export { Item, Frame, identity, fromArray };
