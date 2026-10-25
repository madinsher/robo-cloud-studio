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
import { Program } from '../src/core/items/program';
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
  const parentIdx: number[] = gt.items.map((i: any, k: number) => { for (let j = k - 1; j >= 0; j--) if (gt.items[j].name === i.parent) return j; return -1; });
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
  const extra = [...pool.values()].flat().filter((i) => !(i instanceof Tool && i.params.rdkFromInstruction));
  if (extra.length) issues.push(`items not in RoboDK (${extra.length}): ${limit(extra.map((i) => `${ItemType[i.type]} "${i.name}"`))}`);
  const byName = (type: number, nm: string): Item | undefined => { for (const [g, it] of matched) if (g.type === type && g.name === nm) return it; return undefined; };

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
    // PoseFrame: the active frame in the positioner flange (synchronised turntable), else in the base of the robot
    // carrying this one (rail / gantry / arm holding a mechanism), else in the robot base
    const frm = m16(gr.poseFrame);
    const fr = r.activeFrame();
    if (frm && fr) {
      const positioner = mech.find((m) => m.role === 'positioner' && m.robot)?.robot;
      let carrier: Item | null = r.parent;
      while (carrier && !(carrier instanceof Robot)) carrier = carrier.parent;
      const ref = positioner ? multiply(positioner.poseAbs(), positioner.solveFKFlange()) : carrier instanceof Robot ? carrier.poseAbs() : r.poseAbs();
      const got = multiply(invert(ref), fr instanceof Robot ? fr.poseAbs() : fr.poseAbs());
      if (bad(err(got, frm))) issues.push(`robot ${gr.name}: active frame ${fmt(err(got, frm))} (${fr.name})`);
    } else if (frm && bad(err(identity(), frm))) issues.push(`robot ${gr.name}: no active frame, RoboDK has one`);
  }

  // ---- tools
  for (const gt4 of gt.tools) {
    const t = byName(4, gt4.name);
    const want = m16(gt4.poseTool);
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
    if (gtt.joints && jdiff(t.joints, gtt.joints) > TOL_DEG) tBad.push(`${gtt.name} joints ${t.joints} != ${gtt.joints}`);
    if (gtt.isJoint != null && !!gtt.isJoint !== t.isJointTarget) tBad.push(`${gtt.name} joint-target flag`);
    const rb = t.robotId ? station.findById(t.robotId)?.name ?? '' : '';
    if ((gtt.robot ?? '') !== rb) tBad.push(`${gtt.name} robot link "${rb}" != "${gtt.robot}"`);
  }
  if (tBad.length) issues.push(`targets (${tBad.length}): ${limit(tBad)}`);

  // ---- programs
  for (const gp of gt.programs) {
    const prog = byName(8, gp.name);
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

  if (SIM) await simulate(gt, station, assets, res);
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
  if (isJoint && g.joints && jdiff(q, g.joints) > TOL_DEG) out.push(`#${i} "${g.name}" joints ${q?.map((v) => +v.toFixed(3))} != ${g.joints}`);
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
      if (robot instanceof Robot && q && g.joints && / joints /.test(s)) {
        const e = err(robot.solveFKFlange(q.slice(0, robot.dof)), robot.solveFKFlange(g.joints.slice(0, robot.dof)));
        if (!bad(e) && jdiff(q.slice(robot.dof), g.joints.slice(robot.dof)) <= TOL_DEG) { res.notes.push(`program ${gp.name} move ${i}: RoboDK picked another configuration for the same pose`); continue; }
      }
      seq.push(s);
    }
  }
  if (seq.length) issues.push(`program ${gp.name} (machining): ${seq.length} move mismatches: ${limit(seq, 5)}`);
}

/** Simulation checks against Item.Update() and InstructionListJoints(). */
async function simulate(gt: GT, station: Station, assets: AssetStore, res: GtResult): Promise<void> {
  const { ProgramSimulator } = await import('../src/core/motion/simulator');
  const { sampleAt } = await import('../src/core/motion/trajectory');
  for (const pr of res.programs) {
    const gp = gt.programs.find((p: any) => p.name === pr.name);
    const prog = [...station.walk()].find((i) => i instanceof Program && i.name === pr.name) as Program | undefined;
    if (!gp || !prog) continue;
    const sim = new ProgramSimulator(station);
    (sim as any).collisionOptions = { enabled: false };
    let out;
    try { out = sim.compile(prog); } catch (e) { pr.errors = [`compile threw: ${(e as Error).message}`]; res.issues.push(`sim ${pr.name}: compile threw`); continue; }
    const errors = out.problems.filter((p) => p.severity === 'error').map((p) => p.message);
    pr.ok = out.ok; pr.ourTime = +out.duration.toFixed(3); pr.errors = errors.slice(0, 3);
    if ((gp.update?.[3] ?? 0) >= 0.999 && errors.length) res.issues.push(`sim ${pr.name}: errors where RoboDK is valid: ${errors.slice(0, 2).join(' | ')}`);
    if (gp.update?.[1] > 0.2 && out.duration > 0) pr.timeRatio = +(out.duration / gp.update[1]).toFixed(3);
    const robotG = gt.robots.find((x: any) => x.name === gp.robot);
    const dof = robotG?.lower?.length ?? 6;
    const ends = new Map<number, number[]>();
    for (const row of gp.jl ?? []) ends.set(row[row.length - 5], row.slice(0, row.length - 8));
    const gtEnds = [...ends.entries()].sort((a, b) => a[0] - b[0]).map((e) => e[1]);
    const ourEnds = sim.steps.filter((s) => s.trajectory && s.trajectory.samples.length && (s.robot?.name === gp.robot)).map((s) => sampleAt(s.trajectory!, s.trajectory!.duration).joints.slice(0, dof));
    let worst = 0;
    for (let i = 0; i < Math.min(gtEnds.length, ourEnds.length); i++) worst = Math.max(worst, jdiff(ourEnds[i].slice(0, gtEnds[i].length), gtEnds[i].slice(0, ourEnds[i].length)));
    pr.worstJointDeg = Number.isFinite(worst) ? +worst.toFixed(3) : -1;
    if (gtEnds.length && gtEnds.length !== ourEnds.length) res.issues.push(`sim ${pr.name}: ${ourEnds.length} moves, RoboDK ${gtEnds.length}`);
    if (worst > 0.1) res.issues.push(`sim ${pr.name}: joints differ from RoboDK by ${worst.toFixed(2)}°`);
  }
}

const fixtures = existsSync(GT_DIR) ? readdirSync(GT_DIR).filter((f) => f.endsWith('.json.gz') && (!ONLY || f.includes(ONLY))).sort() : [];
const haveLib = existsSync(LIB);

describe('RoboDK Library vs RoboDK ground truth', () => {
  it('has ground truth fixtures', () => { expect(fixtures.length).toBeGreaterThan(ONLY ? 0 : 90); });
  for (const f of fixtures) {
    it.skipIf(!haveLib)(`${f.replace(/_\.json\.gz$/, '')} imports like RoboDK`, async () => {
      const res = await compareFile(f);
      if (OUT) { mkdirSync(OUT, { recursive: true }); writeFileSync(`${OUT}/${f.replace(/\.json\.gz$/, '')}.json`, JSON.stringify(res, null, 1)); }
      expect(res.issues).toEqual([]);
    }, SIM ? 3_600_000 : 600_000);
  }
});

