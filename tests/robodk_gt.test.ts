/**
 * RoboDK Library compatibility: every file of C:/RoboDK/Library imported by the studio and compared with the ground
 * truth RoboDK 5.9.4 dumped for it (tests/fixtures/robodk_gt/*.json.gz, see the README there).
 *
 * Import-level checks (default): item tree (type, name, parent), absolute poses, robot DOF / limits / current and home
 * joints / FK at 12 random joint vectors (synchronised external axes included) / active tool and frame, tool TCPs,
 * targets (pose, joints, joint-target flag, robot link), programs (robot link, instruction count, label, type, move
 * type, target joints and pose), Python programs, machining project links.
 * Simulation checks (ROBODK_GT_SIM=1): validity, cycle time and joints at the end of every move vs RoboDK.
 *
 * Env: ROBODK_GT_ONLY=<substring of fixture name>, ROBODK_GT_OUT=<dir> (one JSON result per file, used by
 * scripts/robodk-gt.mjs to write docs/robodk-library-compat.md), ROBODK_GT_SIM=1.
 *
 * Comparison semantics (RoboDK API): robot.Pose() is the TCP, robot.PoseAbs() the robot base; tool.Pose() is the
 * identity and tool.PoseAbs() the flange (the studio keeps the TCP as the tool pose); SolveFK(q, eye, eye) is the
 * flange in the robot base; for a robot with synchronised axes SolveFK(q_all) is the flange in the positioner flange
 * (or in the carrier base when there is no positioner); InstructionListJoints MOVE_ID counts executed moves.
 */
import { describe, it, expect } from 'vitest';
import { readFileSync, readdirSync, existsSync, writeFileSync, mkdirSync } from 'node:fs';
import { gunzipSync } from 'node:zlib';
import { importRdkBestEffort } from '../src/io/robodk/rdk_container';
import { AssetStore } from '../src/scene/assets';
import { Robot } from '../src/core/items/robot';
import { Program, Instruction } from '../src/core/items/program';
import type { MoveInstruction } from '../src/core/items/program';
import { Item, ItemType, Station, Tool, Target } from '../src/core/items/item';
import { fromRows, getPos, multiply, invert, identity } from '../src/core/math/pose';
import type { Mat4 } from '../src/core/math/pose';

const LIB = 'C:/RoboDK/Library';
const GT_DIR = 'tests/fixtures/robodk_gt';
const ONLY = process.env.ROBODK_GT_ONLY ?? '';
const OUT = process.env.ROBODK_GT_OUT ?? '';
const SIM = process.env.ROBODK_GT_SIM === '1';
const TOL_MM = 0.01, TOL_RAD = 1e-4, TOL_DEG = 1e-3;

type GT = any;
function loadGt(file: string): { gt: GT | null; error?: string } {
  const text = gunzipSync(readFileSync(`${GT_DIR}/${file}`)).toString('utf8');
  try { return { gt: JSON.parse(text) }; } catch { /* RoboDK dumps NaN for undefined joints */ }
  try { return { gt: JSON.parse(text.replace(/\bNaN\b/g, 'null').replace(/-?\bInfinity\b/g, 'null')) }; } catch (e) { return { gt: null, error: `ground truth file is truncated or invalid (${(e as Error).message.slice(0, 80)})` }; }
}
const m16 = (a: number[] | null | undefined): Mat4 | null => (a && a.length === 16 ? fromRows([a.slice(0, 4), a.slice(4, 8), a.slice(8, 12), a.slice(12, 16)]) : null);
/** Translation error (mm) and rotation angle (rad); the angle comes from the Frobenius norm of Ra - Rb (exact, well conditioned near 0). */
function err(a: Mat4, b: Mat4) {
  const p = getPos(a), q = getPos(b);
  let f = 0;
  for (const i of [0, 1, 2, 4, 5, 6, 8, 9, 10]) f += (a[i] - b[i]) ** 2;
  return { mm: Math.hypot(p[0] - q[0], p[1] - q[1], p[2] - q[2]), rad: 2 * Math.asin(Math.min(1, Math.sqrt(f) / (2 * Math.SQRT2))) };
}
const bad = (e: { mm: number; rad: number }) => e.mm > TOL_MM || e.rad > TOL_RAD;
const fmt = (e: { mm: number; rad: number }) => `${e.mm.toFixed(3)} mm ${(e.rad * 180 / Math.PI).toFixed(4)}°`;
const jdiff = (a: number[] | null | undefined, b: number[] | null | undefined) => {
  if (!a || !b) return a === b || (!a?.length && !b?.length) ? 0 : Infinity;
  if (a.length !== b.length) return Infinity;
  return a.reduce((m, v, i) => Math.max(m, Math.abs(v - b[i])), 0);
};
/** Joint vectors of a target / move: RoboDK pads to 6 values with zeros, the studio keeps the robot's own axes. */
const jdiffAxes = (ours: number[] | null | undefined, gt: number[] | null | undefined) => (ours && gt && gt.length > ours.length && gt.slice(ours.length).every((v) => v === 0) ? jdiff(ours, gt.slice(0, ours.length)) : jdiff(ours, gt));
const limit = (list: string[], n = 8) => `${list.slice(0, n).join('; ')}${list.length > n ? ` (+${list.length - n})` : ''}`;

/** RoboDK INS_TYPE -> studio instruction kinds. */
const KIND: Record<number, string[]> = { 0: ['move'], 1: ['move'], 2: ['speed'], 3: ['frame'], 4: ['tool'], 5: ['robot'], 6: ['pause'], 7: ['event'], 8: ['call', 'code', 'thread', 'print'], 9: ['print'], 10: ['rounding'], 11: ['io', 'wait', 'signal'] };

export interface GtResult {
  file: string; ok: boolean; issues: string[]; importMs: number; fidelity?: string; dropped?: string[];
  /** Differences explained by RoboDK itself (machining programs regenerated at load), not import defects. */
  notes: string[];
  fk: Record<string, number>; counts: Record<string, number>;
  programs: Array<{ name: string; gtCount: number; ourCount: number; gtValid?: number; gtTime?: number; ok?: boolean; ourTime?: number; timeRatio?: number; worstJointDeg?: number; errors?: string[] }>;
  gtIssue?: string;
}

/** Absolute pose RoboDK reports for an item. */
function ourPoseAbs(it: Item): Mat4 {
  if (it instanceof Tool) return it.flangeAbs();
  return it.poseAbs();
}

async function compareFile(file: string): Promise<GtResult> {
  const res: GtResult = { file, ok: false, issues: [], notes: [], importMs: 0, fk: {}, counts: {}, programs: [] };
  const { gt, error } = loadGt(file);
  if (!gt) { res.gtIssue = error; res.issues.push(`GT: ${error}`); return res; }
  const issues = res.issues;
  const t0 = Date.now();
  const buf = readFileSync(gt.file);
  const assets = new AssetStore();
  const name = String(gt.file).split('/').pop()!;
  const { station, report } = await importRdkBestEffort(buf.buffer.slice(buf.byteOffset, buf.byteOffset + buf.byteLength), name, assets);
  res.importMs = Date.now() - t0;
  res.fidelity = report.fidelity;
  res.dropped = report.dropped;
  res.counts = report.counts ?? {};
  if (report.fidelity === 'scan') issues.push(`import fell back to the string scan: ${report.notes.join(' ')}`);

  // ---- item tree: RoboDK lists called programs again under the caller; those are references, not items
  // items come in depth-first order: the parent of item k is the closest preceding item with the parent's name
  // (a program is only the parent of the programs it calls: a folder "Pick" may hold a program "Pick" and its siblings)
  const calls = new Map<string, Set<string>>();
  for (const p of gt.programs ?? []) if (!calls.has(p.name)) calls.set(p.name, new Set((p.instructions ?? []).filter((x: any) => x && x.type === 8).map((x: any) => x.name)));
  const parentIdx: number[] = gt.items.map((i: any, k: number) => {
    for (let j = k - 1; j >= 0; j--) if (gt.items[j].name === i.parent && (gt.items[j].type !== 8 || i.type !== 8 || !calls.has(i.parent) || calls.get(i.parent)!.has(i.name))) return j;
    return -1;
  });
  const underProgram = (k: number): boolean => { for (let j = parentIdx[k]; j >= 0; j = parentIdx[j]) if (gt.items[j].type === 8) return true; return false; };
  const gtItems: any[] = gt.items.filter((i: any, k: number) => i.type !== 9 && !underProgram(k));
  const ours = [...station.walk()].filter((i) => i !== station && i.type !== ItemType.INSTRUCTION);
  const pool = new Map<string, Item[]>();
  const keyOf = (type: number, nm: string, parent: string) => `${type}|${nm}|${parent}`;
  const parentName = (it: Item) => (!it.parent || it.parent instanceof Station ? '<station>' : it.parent.name);
  for (const it of ours) { const k = keyOf(it.type, it.name, parentName(it)); const l = pool.get(k); if (l) l.push(it); else pool.set(k, [it]); }
  const matched = new Map<any, Item>();
  const missing: string[] = [];
  for (const g of gtItems) {
    const par = g.parent === gt.station ? '<station>' : g.parent;
    const l = pool.get(keyOf(g.type, g.name, par));
    if (l?.length) matched.set(g, l.shift()!);
    else missing.push(`${ItemType[g.type] ?? g.type} "${g.name}" under "${par}"`);
  }
  if (missing.length) issues.push(`items missing or misplaced (${missing.length}): ${limit(missing)}`);
  const extra = [...pool.values()].flat().filter((i) => !i.params.rdkFromInstruction);
  if (extra.length) issues.push(`items not in RoboDK (${extra.length}): ${limit(extra.map((i) => `${ItemType[i.type]} "${i.name}"`))}`);
  const byName = (type: number, nm: string, k = 0): Item | undefined => { for (const [g, it] of matched) if (g.type === type && g.name === nm && k-- === 0) return it; return undefined; };

  // ---- absolute poses
  const poseBad: string[] = [];
  for (const [g, it] of matched) {
    if (![2, 3, 4, 5, 6].includes(g.type)) continue;
    const want = m16(g.poseAbs);
    if (!want) continue;
    const e = err(ourPoseAbs(it), want);
    if (bad(e)) poseBad.push(`${it.name}: ${fmt(e)}`);
  }
  if (poseBad.length) issues.push(`absolute poses differ (${poseBad.length}): ${limit(poseBad)}`);

  // ---- robots
  for (const gr of gt.robots) {
    const r = byName(2, gr.name);
    if (!(r instanceof Robot)) continue;
    const sync = (r.params.syncAxes as Array<{ id: string; role: string; joints: number[] }> | undefined) ?? [];
    const nExt = sync.reduce((n, s) => n + s.joints.length, 0);
    const nAll = gr.lower?.length ?? r.dof;
    if (r.dof + nExt !== nAll) issues.push(`robot ${gr.name}: ${r.dof} joints + ${nExt} external axes, RoboDK ${nAll}`);
    const lim = r.jointLimits();
    const sl = r.params.syncLimits as { lower: number[]; upper: number[] } | undefined;
    const lower = [...lim.lower, ...(sl?.lower ?? [])], upper = [...lim.upper, ...(sl?.upper ?? [])];
    if (jdiff(lower, gr.lower) > TOL_DEG || jdiff(upper, gr.upper) > TOL_DEG) issues.push(`robot ${gr.name}: limits ${JSON.stringify([lower, upper])} != ${JSON.stringify([gr.lower, gr.upper])}`);
    const cur = [...r.joints(), ...((r.params.syncJoints as number[] | undefined) ?? [])];
    if (gr.joints && jdiff(cur, gr.joints) > TOL_DEG) issues.push(`robot ${gr.name}: joints ${cur.map((v) => +v.toFixed(3))} != ${gr.joints}`);
    if (gr.jointsHome && jdiff(r.jointsHome(), gr.jointsHome.slice(0, r.dof)) > TOL_DEG) issues.push(`robot ${gr.name}: home ${r.jointsHome()} != ${gr.jointsHome}`);
    // FK at RoboDK's random joint vectors
    const mech = sync.map((s) => ({ ...s, robot: station.findById(s.id) as Robot | null }));
    let worst = 0;
    for (const s of gr.fk ?? []) {
      const want = m16(s.flange);
      if (!want) continue;
      const q: number[] = s.q;
      let got = r.solveFKFlange(q.slice(0, r.dof));
      if (mech.length && q.length > r.dof) {
        // synchronised: flange of the arm expressed in the positioner flange (else in the carrier base)
        const carrier = mech.find((m) => m.role === 'carrier' && m.robot);
        const positioner = mech.find((m) => m.role === 'positioner' && m.robot);
        let baseAbs = r.poseAbs();
        if (carrier?.robot) {
          const c = carrier.robot;
          const qc = carrier.joints.map((j) => q[j]);
          // arm base = carrier flange at qc · (arm pose relative to the carrier flange)
          const rel = multiply(invert(multiply(c.poseAbs(), c.solveFKFlange())), r.poseAbs());
          baseAbs = multiply(multiply(c.poseAbs(), c.solveFKFlange(qc)), rel);
        }
        const ref = positioner?.robot ? multiply(positioner.robot.poseAbs(), positioner.robot.solveFKFlange(positioner.joints.map((j) => q[j]))) : carrier?.robot?.poseAbs() ?? identity();
        got = multiply(invert(ref), multiply(baseAbs, got));
      }
      worst = Math.max(worst, err(got, want).mm + err(got, want).rad * 1000);
    }
    res.fk[gr.name] = +worst.toFixed(4);
    if (worst > TOL_MM) issues.push(`robot ${gr.name}: FK error ${worst.toFixed(3)} (mm + mrad)`);
    const tcp = m16(gr.poseTool);
    if (tcp && bad(err(r.poseTool(), tcp))) issues.push(`robot ${gr.name}: active tool ${fmt(err(r.poseTool(), tcp))}${r.activeTool() ? ` (${r.activeTool()!.name})` : ' (none)'}`);
    // PoseFrame: the active frame in the positioner flange (synchronised turntable), else in the base of the
    // synchronised carrier (rail / gantry), else in the robot's own base (also for a gripper mounted on an arm)
    const frm = m16(gr.poseFrame);
    const fr = r.activeFrame();
    if (frm && fr) {
      const positioner = mech.find((m) => m.role === 'positioner' && m.robot)?.robot;
      const carrier = mech.find((m) => m.role === 'carrier' && m.robot)?.robot;
      const ref = positioner ? multiply(positioner.poseAbs(), positioner.solveFKFlange()) : carrier ? carrier.poseAbs() : r.poseAbs();
      // a robot used as reference frame stands for its flange (where the items it carries are attached)
      const got = multiply(invert(ref), fr instanceof Robot ? multiply(fr.poseAbs(), fr.solveFKFlange()) : fr.poseAbs());
      if (bad(err(got, frm))) issues.push(`robot ${gr.name}: active frame ${fmt(err(got, frm))} (${fr.name})`);
    } else if (frm && bad(err(identity(), frm))) issues.push(`robot ${gr.name}: no active frame, RoboDK has one`);
  }

  // ---- tools
  for (const gt4 of gt.tools) {
    const t = byName(4, gt4.name);
    const want = m16(gt4.poseTool);
    // a RoboDK tool must be a studio Tool (a plain object would silently lose the TCP: V1, Library .tool files)
    if (t && !(t instanceof Tool)) issues.push(`tool ${gt4.name}: imported as ${t.constructor.name}, not a Tool (no TCP)`);
    if (t instanceof Tool && want && bad(err(t.poseTool(), want))) issues.push(`tool ${gt4.name}: TCP ${fmt(err(t.poseTool(), want))}`);
  }

  // ---- targets
  const tBad: string[] = [];
  for (const gtt of gt.targets) {
    const g = gtItems.find((i) => i.type === 6 && i.name === gtt.name && i.parent === gtt.parent && !(i as any)._used);
    if (!g) continue;
    (g as any)._used = true;
    const t = matched.get(g);
    if (!(t instanceof Target)) continue;
    const want = m16(gtt.pose);
    if (want && bad(err(t.pose(), want))) tBad.push(`${gtt.name} pose ${fmt(err(t.pose(), want))}`);
    if (gtt.joints && jdiffAxes(t.joints, gtt.joints) > TOL_DEG) tBad.push(`${gtt.name} joints ${t.joints} != ${gtt.joints}`);
    if (gtt.isJoint != null && !!gtt.isJoint !== t.isJointTarget) tBad.push(`${gtt.name} joint-target flag`);
    const rb = t.robotId ? station.findById(t.robotId)?.name ?? '' : '';
    if ((gtt.robot ?? '') !== rb) tBad.push(`${gtt.name} robot link "${rb}" != "${gtt.robot}"`);
  }
  if (tBad.length) issues.push(`targets (${tBad.length}): ${limit(tBad)}`);

  // ---- programs
  const progSeen = new Map<string, number>();
  for (const gp of gt.programs) {
    // Item.Update() of a machining project without a linked program (called by the dump) creates a new program with
    // the same name after the items were listed: it is not part of the file
    const k = progSeen.get(gp.name) ?? 0;
    progSeen.set(gp.name, k + 1);
    if (k >= gtItems.filter((i) => i.type === 8 && i.name === gp.name).length) { res.notes.push(`program ${gp.name}: created by RoboDK during the ground-truth dump (machining project update), not stored in the file`); continue; }
    const prog = byName(8, gp.name, k);
    if (prog instanceof Program && prog.params.machiningId) { compareGenerated(gp, prog, station, issues, res); continue; }
    const pr = { name: gp.name, gtCount: gp.count, ourCount: 0, gtValid: gp.update?.[3], gtTime: gp.update?.[1] } as GtResult['programs'][number];
    res.programs.push(pr);
    if (!(prog instanceof Program)) continue;
    const ins = prog.instructions();
    pr.ourCount = ins.length;
    const rb = prog.robot()?.name ?? '';
    if ((gp.robot ?? '') !== rb) issues.push(`program ${gp.name}: robot "${rb}" != "${gp.robot}"`);
    if (ins.length !== gp.count) issues.push(`program ${gp.name}: ${ins.length} instructions, RoboDK ${gp.count}`);
    const seq: string[] = [];
    for (let i = 0; i < Math.min(ins.length, gp.instructions.length); i++) {
      const g = gp.instructions[i];
      if (!g) continue;
      const d = ins[i].data;
      // Item.Instruction() names a call / code instruction by its code text; other instructions by the tree label
      const text = d.kind === 'code' ? d.code : d.kind === 'call' || d.kind === 'thread' ? d.programName ?? '' : d.kind === 'print' && (g.type === 8 || g.type === 9) ? d.message : ins[i].name;
      if (text !== g.name) seq.push(`#${i} label "${text}" != "${g.name}"`);
      if (!(KIND[g.type] ?? []).includes(d.kind)) { seq.push(`#${i} "${g.name}" ${d.kind} != type ${g.type}`); continue; }
      if (d.kind !== 'move') continue;
      const m = d as MoveInstruction;
      const mt = g.type === 1 ? 'MoveC' : g.move === 1 ? 'MoveJ' : 'MoveL';
      if (m.moveType !== mt) seq.push(`#${i} ${m.moveType} != ${mt}`);
      for (const s of moveMismatch(i, g, m, station)) seq.push(s);
    }
    if (seq.length) issues.push(`program ${gp.name}: ${seq.length} instruction mismatches: ${limit(seq, 5)}`);
  }
  for (const g of gtItems) if (g.type === 8 && !gt.programs.some((p: any) => p.name === g.name)) res.notes.push(`program ${g.name}: the RoboDK dump has no instruction data for it (not compared)`);
  // ---- Python programs and machining projects
  const py = [...station.walk()].filter((i) => i.type === ItemType.PROGRAM_PYTHON);
  const gpy = (gt.python ?? []).map((p: any) => p.name).sort();
  if (JSON.stringify(py.map((p) => p.name).sort()) !== JSON.stringify(gpy)) issues.push(`Python programs ${JSON.stringify(py.map((p) => p.name))} != ${JSON.stringify(gpy)}`);
  const empty = py.filter((p) => typeof p.params.pythonCode !== 'string' || !(p.params.pythonCode as string).trim());
  if (empty.length) issues.push(`Python programs without source: ${empty.map((p) => p.name).join(', ')}`);
  for (const gm of gt.machining ?? []) {
    const m = byName(11, gm.name);
    if (!m) continue;
    const link = (m.params.rdkMachining ?? {}) as Record<string, string>;
    const nm = (id?: string) => (id ? station.findById(id)?.name ?? '' : '');
    if ((gm.program ?? '') !== nm(link.programId)) issues.push(`machining ${gm.name}: program "${nm(link.programId)}" != "${gm.program}"`);
    if ((gm.robot ?? '') !== nm(link.robotId)) issues.push(`machining ${gm.name}: robot "${nm(link.robotId)}" != "${gm.robot}"`);
  }

  if (SIM) await simulate(gt, station, assets, res, file);
  res.ok = issues.length === 0;
  return res;
}

/**
 * Joint targets compare joints; Cartesian targets compare the pose (RoboDK re-solves the joints of a Cartesian target
 * for Item.Instruction(), so the stored joints are only a configuration hint).
 */
function moveMismatch(i: number, g: any, m: MoveInstruction, station: Station): string[] {
  const out: string[] = [];
  const tgt = m.targetId ? station.findById(m.targetId) as Target | null : null;
  const isJoint = tgt ? tgt.isJointTarget : !m.pose;
  const q = tgt ? tgt.joints : m.joints;
  if (isJoint && g.joints && jdiffAxes(q, g.joints) > TOL_DEG) out.push(`#${i} "${g.name}" joints ${q?.map((v) => +v.toFixed(3))} != ${g.joints}`);
  if (!!g.isJoint !== isJoint) out.push(`#${i} "${g.name}" joint-target flag ${isJoint} != ${!!g.isJoint}`);
  const want = m16(g.pose);
  const pose = tgt ? tgt.pose() : m.pose ? Float64Array.from(m.pose) : null;
  if (!isJoint && want && pose && bad(err(pose, want))) out.push(`#${i} "${g.name}" pose ${fmt(err(pose, want))}`);
  return out;
}

/**
 * Programs generated by a machining / curve-follow project: RoboDK 5.9.4 regenerates them when the station is loaded
 * (header instructions in another order, speed / spindle instructions merged, points resampled, move names and robot
 * configurations chosen again), so the stored program can only be compared move by move when RoboDK kept the same
 * number of moves. A different joint vector for the same flange pose is a configuration choice, not a decode error.
 */
function compareGenerated(gp: any, prog: Program, station: Station, issues: string[], res: GtResult): void {
  const ins = prog.instructions();
  res.programs.push({ name: gp.name, gtCount: gp.count, ourCount: ins.length, gtValid: gp.update?.[3], gtTime: gp.update?.[1] });
  const robot = prog.robot();
  const rb = robot?.name ?? '';
  if ((gp.robot ?? '') !== rb) issues.push(`program ${gp.name}: robot "${rb}" != "${gp.robot}"`);
  const ours = ins.filter((x) => x.data.kind === 'move');
  const gtMoves = (gp.instructions as any[]).filter((g) => g && (g.type === 0 || g.type === 1));
  if (gp.count > gp.instructions.length) res.notes.push(`program ${gp.name}: RoboDK dump keeps the first ${gp.instructions.length} of ${gp.count} instructions`);
  if (gp.count <= gp.instructions.length && ours.length !== gtMoves.length) { res.notes.push(`program ${gp.name} (machining): RoboDK 5.9.4 regenerates it at load: ${gtMoves.length} moves vs ${ours.length} stored in the file`); return; }
  if (ins.length !== gp.count) res.notes.push(`program ${gp.name} (machining): regenerated by RoboDK with ${gp.count} instructions, ${ins.length} stored`);
  const seq: string[] = [];
  for (let i = 0; i < Math.min(ours.length, gtMoves.length); i++) {
    const g = gtMoves[i], m = ours[i].data as MoveInstruction;
    const mt = g.type === 1 ? 'MoveC' : g.move === 1 ? 'MoveJ' : 'MoveL';
    if (m.moveType !== mt) seq.push(`move ${i} ${m.moveType} != ${mt}`);
    for (const s of moveMismatch(i, g, m, station)) {
      // same flange pose with another robot configuration
      const q = m.targetId ? (station.findById(m.targetId) as Target | null)?.joints : m.joints;
      if (robot instanceof Robot && q && g.joints && / joints /.test(s) && jdiff(q.slice(robot.dof), g.joints.slice(robot.dof)) <= TOL_DEG) {
        const fa = robot.solveFKFlange(q.slice(0, robot.dof)), fb = robot.solveFKFlange(g.joints.slice(0, robot.dof));
        if (!bad(err(fa, fb))) { res.notes.push(`program ${gp.name} move ${i}: RoboDK picked another configuration for the same pose`); continue; }
        // the machining optimiser may also turn the tool about its own axis (free rotation of a milling / laser path)
        const tool = toolBefore(ins, ours[i], station) ?? robot.poseTool();
        if (sameToolAxis(multiply(fa, tool), multiply(fb, tool))) { res.notes.push(`program ${gp.name} move ${i}: RoboDK re-optimised the tool rotation about its axis`); continue; }
      }
      if (/ pose /.test(s)) {
        const pose = m.targetId ? (station.findById(m.targetId) as Target | null)?.pose() : m.pose ? Float64Array.from(m.pose) : null;
        const want = m16(g.pose);
        if (pose && want && sameToolAxis(pose, want)) { res.notes.push(`program ${gp.name} move ${i}: RoboDK re-optimised the tool rotation about its axis`); continue; }
      }
      seq.push(s);
    }
  }
  if (seq.length) issues.push(`program ${gp.name} (machining): ${seq.length} move mismatches: ${limit(seq, 5)}`);
}

/** Same TCP position and tool Z axis (rotation about the tool axis is a free parameter of machining paths). */
function sameToolAxis(a: Mat4, b: Mat4): boolean {
  const dz = Math.hypot(a[8] - b[8], a[9] - b[9], a[10] - b[10]);
  return err(a, b).mm <= TOL_MM && dz <= TOL_RAD;
}

/** TCP of the last tool instruction before `ins` in the program. */
function toolBefore(all: Instruction[], ins: Instruction, station: Station): Mat4 | null {
  let tool: Mat4 | null = null;
  for (const x of all) {
    if (x === ins) break;
    const d = x.data;
    if (d?.kind === 'tool' && d.toolId) { const t = station.findById(d.toolId); if (t instanceof Tool) tool = t.poseTool(); }
  }
  return tool;
}

/** Simulation ground truth of `file` (tests/fixtures/robodk_gt/sim, dump_sim.py): every program from the station as loaded. */
function loadSimGt(file: string): any | null {
  const p = `${GT_DIR}/sim/${file}`;
  if (!existsSync(p)) return null;
  try { return JSON.parse(gunzipSync(readFileSync(p)).toString('utf8').replace(/\bNaN\b/g, 'null').replace(/-?\bInfinity\b/g, 'null')); } catch { return null; }
}

/**
 * Move ends in the order RoboDK's InstructionListJoints lists them: the program's moves with every called program
 * expanded the FIRST time it is called only (a second call of GoHome / DrillingPart / Rim is not listed again; 01.c,
 * 05.a), threads not listed, and only the moves of the program's own robot (the Kawasaki of 05.a, not its turntable).
 */
function listedMoveEnds(steps: any[], top: Program, robot: string, dof: number, sampleAt: (tr: any, t: number) => { joints: number[] }): Array<number[] | null> {
  const moves = steps.filter((s) => s.trajectory && s.trajectory.samples.length);
  const main = moves.find((s) => s.instruction.parent === top)?.runId ?? moves[0]?.runId;
  const seen = new Set<unknown>([top]);
  const out: Array<number[] | null> = [];
  let cur: unknown = null, last = -1, skip = false;
  for (const s of moves) {
    if (s.runId !== main) continue;
    const p = s.instruction.parent as Program;
    const idx = p ? p.instructions().indexOf(s.instruction) : -1;
    // a new call instance starts when the executing program changes or its instruction index goes back
    if (p !== cur || idx <= last) { skip = p !== top && seen.has(p); seen.add(p); cur = p; }
    last = idx;
    if (skip) continue;
    // RoboDK lists the target of every move (also when the path is blended past it)
    if (s.robot?.name === robot) out.push((s.qTarget ?? sampleAt(s.trajectory, s.trajectory.duration).joints).slice(0, dof));
  }
  return out;
}

/** A program and every program it calls (directly or through code calls by name). */
function programTree(top: Program): Program[] {
  const out: Program[] = [top];
  for (let k = 0; k < out.length && out.length < 200; k++) {
    for (const ins of out[k].instructions()) {
      const d: any = ins.data;
      const sub = d.kind === 'call' && d.programId ? top.station?.findById(d.programId) : d.kind === 'code' && d.asFunctionCall && top.station ? [...top.station.walk()].find((i) => i instanceof Program && i.name === String(d.code).replace(/\(.*$/, '').trim()) : null;
      if (sub instanceof Program && !out.includes(sub)) out.push(sub);
    }
  }
  return out;
}

/** Whether a program (with the programs it calls) calls some program more than once. */
function repeatsCall(top: Program): boolean {
  const seen = new Set<Program>();
  const station = top.station;
  const walk = (p: Program, depth: number): boolean => {
    if (depth > 16) return false;
    for (const ins of p.instructions()) {
      const d: any = ins.data;
      let sub: unknown = null;
      if (d.kind === 'call') sub = d.programId ? station?.findById(d.programId) : null;
      else if (d.kind === 'code' && d.asFunctionCall) sub = station ? [...station.walk()].find((i) => i instanceof Program && i.name === String(d.code).replace(/\(.*$/, '').trim()) : null;
      if (!(sub instanceof Program)) continue;
      if (seen.has(sub)) return true;
      seen.add(sub);
      if (walk(sub, depth + 1)) return true;
    }
    return false;
  };
  return walk(top, 0);
}

/** Positioner evidence of a file (dump_turntable.py): Item.Update() per program with the positioner set to given angles. */
function loadPositionerGt(file: string): { positioner: string; angles: Record<string, Record<string, any[]>> } | null {
  const p = `${GT_DIR}/sim/${file.replace(/.json.gz$/, '')}.positioner.json`;
  if (!existsSync(p)) return null;
  try { return JSON.parse(readFileSync(p, 'utf8')); } catch { return null; }
}

/** RoboDK Update(COLLISION_ON) result -> collision verdict (valid ratio < 1 because of a collision). */
// (a collision-checked update that stopped on an unreachable target did not finish its check: no verdict)
const gtCollides = (u: any[] | null | undefined): boolean | null => (!Array.isArray(u) ? null : /collision detected/i.test(String(u[4])) ? true : u[3] >= 0.999 ? false : null);
/** Acceptance tolerances: cycle time vs RoboDK Item.Update(), joints at every move end vs InstructionListJoints. */
const TIME_TOL = 0.10;
const JOINT_TOL = 0.5;

/**
 * Simulation checks. With the simulation ground truth (sim/<file>, every program run by RoboDK from the station as
 * loaded): validity (ours ok == RoboDK valid), cycle time within ±10 % of Item.Update(), joints at the end of every
 * move within 0.5° of InstructionListJoints (also through calls) and, with ROBODK_GT_COLL=1, the collision verdict of
 * Update(COLLISION_ON). Without it: the dump's sequential Update() numbers (older semantics, see dump_gt.py).
 */
async function simulate(gt: GT, station: Station, assets: AssetStore, res: GtResult, file: string): Promise<void> {
  const { ProgramSimulator } = await import('../src/core/motion/simulator');
  const { sampleAt } = await import('../src/core/motion/trajectory');
  const sgt = loadSimGt(file);
  const COLL = process.env.ROBODK_GT_COLL === '1';
  const seen = new Map<string, number>();
  const robots = [...station.walk()].filter((i): i is Robot => i instanceof Robot);
  for (const pr of res.programs) {
    const k = seen.get(pr.name) ?? 0;
    seen.set(pr.name, k + 1);
    const gp = gt.programs.find((p: any) => p.name === pr.name);
    const prog = [...station.walk()].filter((i) => i instanceof Program && i.name === pr.name)[k] as Program | undefined;
    const sp = sgt ? (sgt.programs as any[]).filter((p) => p.name === pr.name)[k] : null;
    if (!gp || !prog) continue;
    const upd: any[] | undefined = sp ? sp.update : gp.update;
    pr.gtValid = upd?.[3]; pr.gtTime = upd?.[1];
    // start state: the robots' joints when RoboDK ran the program (the stored joints for the sim ground truth)
    let start: Map<string, number[]> | undefined;
    if (sp?.start) {
      start = new Map();
      for (const r of robots) { const q = sp.start[r.name]; if (Array.isArray(q) && robots.filter((x) => x.name === r.name).length === 1) start.set(r.id, q.slice(0, r.joints().length)); }
    }
    const sim = new ProgramSimulator(station);
    sim.collisionOptions = { enabled: false };
    let out;
    try { out = sim.compile(prog, start); } catch (e) { pr.errors = [`compile threw: ${(e as Error).message}`]; res.issues.push(`sim ${pr.name}: compile threw`); continue; }
    const errors = out.problems.filter((p) => p.severity === 'error').map((p) => p.message);
    // RoboDK's cycle time is Item.Update(): moves rest to rest at the robot's motion state (ProgramRunResult.updateTime)
    const ourTime = out.updateTime ?? out.duration;
    pr.ok = out.ok; pr.ourTime = +ourTime.toFixed(3); pr.errors = errors.slice(0, 3);
    const gtValid = (upd?.[3] ?? 0) >= 0.999;
    // RoboDK rates a program valid when only a program it calls fails (the callee itself is rated invalid; 09.c
    // MainProgram, 05.b Main): Item.Update() counts the caller's own instructions and the callees' time up to their
    // failure, with the robot left where the callee stopped (the studio's validation goes on): time not compared
    const calleeOnly = !!sp && gtValid && errors.length > 0 && out.problems.filter((p) => p.severity === 'error').every((p) => {
      const ins = [...station.walk()].flatMap((i) => (i instanceof Program ? i.instructions() : [])).find((x) => x.id === p.instructionId);
      const owner = ins?.parent;
      const rec = owner instanceof Program && owner !== prog ? (sgt.programs as any[]).find((x) => x.name === owner.name) : null;
      return !!rec && rec.update?.[3] < 0.999;
    });
    if (calleeOnly) res.notes.push(`sim ${pr.name}: RoboDK rates it valid although a called program fails (rated invalid by RoboDK too): ${errors[0]}`);
    else if (gtValid && errors.length) res.issues.push(`sim ${pr.name}: errors where RoboDK is valid: ${errors.slice(0, 2).join(' | ')}`);
    // positioner evidence (dump_turntable.py): RoboDK's Update() keeps a positioner at its current angle although the
    // program turns it through a called program; at the angle where RoboDK rates the program valid the studio must give
    // the same validity and cycle time from that start (05.b TopSurf / Inside1 / Side0 / Side180 / Side270, Main)
    const posGt = loadPositionerGt(file);
    if (sp && posGt) {
      const mech = robots.find((r) => r.name === posGt.positioner);
      const callsMech = !!mech && programTree(prog).some((p) => p !== prog && p.robot() === mech);
      if (callsMech) {
        const ent = Object.entries(posGt.angles as Record<string, Record<string, any[]>>).find(([, rec]) => rec[pr.name]?.[3] >= 0.999 && !programTree(prog).some((p) => p !== prog && rec[p.name] && rec[p.name][3] < 0.999));
        let verified = '';
        if (ent && !gtValid) {
          const s2 = new Map(start ?? []);
          s2.set(mech!.id, [Number(ent[0])]);
          const vs = new ProgramSimulator(station);
          vs.collisionOptions = { enabled: false };
          const o2 = vs.compile(prog, s2);
          const t2 = o2.updateTime ?? o2.duration, want = ent[1][pr.name][1];
          if (o2.ok && Math.abs(t2 - want) <= Math.max(0.05, TIME_TOL * want)) verified = `from ${posGt.positioner} at ${ent[0]}° RoboDK and the studio agree (valid, ${t2.toFixed(2)} s vs ${(+want).toFixed(2)} s)`;
          else res.issues.push(`sim ${pr.name}: from ${posGt.positioner} at ${ent[0]}° RoboDK rates it valid in ${(+want).toFixed(2)} s, the studio ${o2.ok ? `in ${t2.toFixed(2)} s` : `fails: ${o2.problems.find((p) => p.severity === 'error')?.message}`}`);
        }
        const sameTime = gtValid && !!upd && Math.abs(ourTime - upd[1]) <= Math.max(0.05, TIME_TOL * upd[1]);
        if (verified || (gtValid && !sameTime)) {
          res.notes.push(`sim ${pr.name}: RoboDK Item.Update() keeps ${posGt.positioner} at its stored angle although the program turns it (rated ${gtValid ? 'valid with the callees evaluated that way' : 'invalid'} as loaded; the studio turns it)${verified ? `; ${verified}` : ''}`);
          pr.worstJointDeg = -1;
          continue;
        }
      }
    }
    if (sp && !gtValid && out.ok && upd) res.issues.push(`sim ${pr.name}: valid here, RoboDK stops at ${(upd[3] * 100).toFixed(1)} % (${String(upd[4]).replace(/<[^>]+>/g, ' ').replace(/\s+/g, ' ').trim().slice(0, 140)})`);
    if (upd && upd[1] > 0.2 && ourTime > 0) pr.timeRatio = +(ourTime / upd[1]).toFixed(3);
    if (sp && gtValid && out.ok && upd) {
      const dt = Math.abs(ourTime - upd[1]);
      if (dt > 0.05 && dt > TIME_TOL * upd[1]) res.issues.push(`sim ${pr.name}: cycle time ${ourTime.toFixed(2)} s, RoboDK ${(+upd[1]).toFixed(2)} s (ratio ${(ourTime / Math.max(1e-9, upd[1])).toFixed(3)}; distance ${out.distance.toFixed(0)} mm, RoboDK ${(+upd[2]).toFixed(0)} mm)`);
    }
    // collision verdict (Update(COLLISION_ON) from the same start)
    if (sp && COLL && sp.updateCollision) {
      const want = gtCollides(sp.updateCollision);
      const cs = new ProgramSimulator(station);
      // RoboDK's Update(COLLISION_ON) does not apply show / hide / attach events (verified on 05.e Paint): same here
      cs.collisionOptions = { enabled: true, assets: assets as any, timeBudgetMs: 120000, ignoreEvents: true };
      let got = false, first = '';
      try { const o2 = cs.compile(prog, start); const c = o2.problems.find((p) => p.severity === 'error' && /^Collision/.test(p.message)); got = !!c; first = c?.message ?? ''; } catch (e) { first = `threw ${(e as Error).message}`; }
      (pr as any).collision = { ours: got, roboDK: want, first, pairs: sp.collision?.pairs };
      if (want !== null && got !== want) res.issues.push(`sim ${pr.name}: collision ${got ? `reported here (${first})` : 'not reported here'}, RoboDK ${want ? `reports one (${JSON.stringify(sp.collision?.pairs ?? [])})` : 'none'}`);
    }
    const robotG = gt.robots.find((x: any) => x.name === gp.robot);
    const dof = robotG?.lower?.length ?? 6;
    const rows: any[] | undefined = sp ? sp.jl : gp.jl;
    if (!sp) {
      // dump_gt.py semantics: machining programs regenerated by the dump, MOVE_ID restarting per call
      if (prog.params.machiningId && gp.count !== prog.instructions().length) { res.notes.push(`sim ${pr.name}: regenerated by RoboDK at load (${gp.count} instructions, ${prog.instructions().length} stored), joints not compared`); pr.worstJointDeg = -1; continue; }
      if (prog.instructions().some((i) => i.data.kind === 'call' || i.data.kind === 'thread' || (i.data.kind === 'code' && i.data.asFunctionCall))) { res.notes.push(`sim ${pr.name}: calls other programs, joints not compared move by move (MOVE_ID repeats per call)`); pr.worstJointDeg = -1; continue; }
    }
    // jl rows: [J.., ERROR, MM_STEP, DEG_STEP, MOVE_ID, TIME, X, Y, Z]; the last row of each run of one MOVE_ID is the end
    // of that move. When RoboDK stops early (status < 0) only the moves before the failing one are complete.
    const gtEnds: number[][] = [];
    let failed = false;
    const list = rows ?? [];
    for (let i = 0; i < list.length; i++) {
      const row = list[i], L = row.length, id = row[L - 5];
      if (row[L - 8] !== 0) { failed = true; break; }
      const next = list[i + 1];
      if (!next || next[next.length - 5] !== id) gtEnds.push(row.slice(0, L - 8));
    }
    const status = sp ? sp.jlStatus : gp.jlStatus;
    const partial = (status ?? 0) < 0 || failed;
    if (partial && !sp) { res.notes.push(`sim ${pr.name}: RoboDK InstructionListJoints stopped (${gp.jlMsg ?? 'error'}); joints not compared`); pr.worstJointDeg = -1; continue; }
    if (partial) gtEnds.pop(); // the move RoboDK stopped in
    if (sp && repeatsCall(prog)) {
      // InstructionListJoints lists a program called several times only once, and not with the state of its first call
      // (01.c MainProgram lists DrillingPart at Part 2): the called programs are compared on their own
      res.notes.push(`sim ${pr.name}: calls a program more than once, RoboDK InstructionListJoints lists it once; joints compared through the called programs`);
      pr.worstJointDeg = -1;
      continue;
    }
    // InstructionListJoints does not move the robot to its first target: it places it there (the first row) and lists
    // the program from that posture (stored target joints when they are valid, 01.c DrillProgram1; IK of the pose
    // otherwise, 01.c GoHome). The listed motion is compared from the same posture.
    let jsim = sim;
    const row0 = list[0];
    const rj = robots.find((x) => x.name === gp.robot);
    if (sp && row0 && rj) {
      const q = row0.slice(0, row0.length - 8).slice(0, rj.joints().length);
      if (q.length === rj.joints().length && q.every(Number.isFinite)) {
        const s2 = new Map(start ?? []);
        s2.set(rj.id, q);
        jsim = new ProgramSimulator(station);
        jsim.collisionOptions = { enabled: false };
        try { jsim.compile(prog, s2); } catch { jsim = sim; }
      }
    }
    const ourEnds: Array<number[] | null> = sp ? listedMoveEnds(jsim.steps, prog, gp.robot, dof, sampleAt) : sim.steps.filter((s) => s.trajectory && s.trajectory.samples.length && (s.robot?.name === gp.robot)).map((s) => sampleAt(s.trajectory!, s.trajectory!.duration).joints.slice(0, dof));
    let worst = 0, worstAt = -1;
    // with the sim ground truth the first listed move is RoboDK's placement (not a motion): compared from the second one
    for (let i = sp ? 1 : 0; i < Math.min(gtEnds.length, ourEnds.length); i++) { const o = ourEnds[i]; if (!o) continue; const d = jdiff(o.slice(0, gtEnds[i].length), gtEnds[i].slice(0, o.length)); if (d > worst) { worst = d; worstAt = i; } }
    pr.worstJointDeg = Number.isFinite(worst) ? +worst.toFixed(3) : -1;
    if (!partial && gtEnds.length && gtEnds.length !== ourEnds.length && (out.ok || !sp)) res.issues.push(`sim ${pr.name}: ${ourEnds.length} moves, RoboDK ${gtEnds.length}`);
    if (worst > (sp ? JOINT_TOL : 0.1)) res.issues.push(`sim ${pr.name}: joints differ from RoboDK by ${worst.toFixed(2)}° at move ${worstAt + 1}`);
  }
}

// Library fixtures (need C:/RoboDK/Library) + synthetic stations made by RoboDK itself and stored in the repo
// (tests/fixtures/robodk_gt/synthetic, built by make_synthetic.py: IO, waits, pauses, rounding, every call variant,
// inline moves, Cyrillic names)
const list = (dir: string) => (existsSync(`${GT_DIR}/${dir}`) ? readdirSync(`${GT_DIR}/${dir}`).filter((f) => f.endsWith('.json.gz')).map((f) => dir + f) : []);
const fixtures = [...list(''), ...list('synthetic/')].filter((f) => !ONLY || f.includes(ONLY)).sort();
const haveLib = existsSync(LIB);

describe('RoboDK Library vs RoboDK ground truth', () => {
  it('has ground truth fixtures', () => { expect(fixtures.length).toBeGreaterThan(ONLY ? 0 : 90); });
  for (const f of fixtures) {
    it.skipIf(!haveLib && !f.startsWith('synthetic/'))(`${f.replace(/_\.json\.gz$/, '')} imports like RoboDK`, async () => {
      const res = await compareFile(f);
      if (OUT) { mkdirSync(OUT, { recursive: true }); writeFileSync(`${OUT}/${f.replace(/\.json\.gz$/, '').replace(/\//g, '_')}.json`, JSON.stringify(res, null, 1)); }
      expect(res.issues).toEqual([]);
    }, SIM ? 3_600_000 : 600_000);
  }
});

