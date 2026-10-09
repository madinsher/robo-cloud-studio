/**
 * Task and motion planning (course chapter 11): planar reachability maps built by sampling a serial arm, inverse
 * reachability for base placement, geometric predicates (reach, blocking, placement), the skeleton–parameters loop
 * with feedback "geometric failure → symbolic fact", recursive object rearrangement with a depth bound, grasp
 * robustness to pose error and plan robustness by perturbation.
 */
import { GroundTask, GroundAction, plan, PlanResult, Domain, Problem, ground, findAction } from './planning';

export interface Point2 { x: number; y: number }
export interface PlanarArm { /** link lengths (m) */ links: number[]; /** joint limits per link as [min, max] in rad */ limits?: Array<[number, number]>; /** base height offset for a vertical plane; ignored for planar maps */ base?: Point2 }

/** Reachability map: grid over the workspace with the fraction of sampled approach angles reachable in each cell (Definition 11.2). */
export interface ReachMap { origin: Point2; cell: number; cols: number; rows: number; index: Float32Array; /** max reach radius */ radius: number }

export function forwardKinematics(arm: PlanarArm, q: number[]): { tip: Point2; angle: number } {
  let x = arm.base?.x ?? 0, y = arm.base?.y ?? 0, th = 0;
  for (let i = 0; i < arm.links.length; i++) { th += q[i]; x += arm.links[i] * Math.cos(th); y += arm.links[i] * Math.sin(th); }
  return { tip: { x, y }, angle: th };
}

/** Sample joint space uniformly (N samples) and accumulate reached cells; the index counts distinct approach-angle bins (8) hit per cell. */
export function buildReachMap(arm: PlanarArm, cell = 0.05, samples = 20000, seed = 1): ReachMap {
  const R = arm.links.reduce((s, l) => s + l, 0); const origin = { x: (arm.base?.x ?? 0) - R, y: (arm.base?.y ?? 0) - R };
  const cols = Math.ceil(2 * R / cell) + 1, rows = cols;
  const bins = new Uint8Array(cols * rows);
  let s = seed >>> 0; const rnd = () => { s = (s * 1664525 + 1013904223) >>> 0; return s / 4294967296; };
  for (let k = 0; k < samples; k++) {
    const q = arm.links.map((_, i) => { const [lo, hi] = arm.limits?.[i] ?? [-Math.PI, Math.PI]; return lo + (hi - lo) * rnd(); });
    const { tip, angle } = forwardKinematics(arm, q);
    const c = Math.floor((tip.x - origin.x) / cell), r = Math.floor((tip.y - origin.y) / cell);
    if (c < 0 || r < 0 || c >= cols || r >= rows) continue;
    const bin = ((Math.round(angle / (Math.PI / 4)) % 8) + 8) % 8;
    bins[r * cols + c] |= 1 << bin;
  }
  const index = new Float32Array(cols * rows);
  for (let i = 0; i < index.length; i++) { let n = 0; for (let b = 0; b < 8; b++) if (bins[i] & (1 << b)) n++; index[i] = n / 8; }
  return { origin, cell, cols, rows, index, radius: R };
}

export function reachIndexAt(map: ReachMap, p: Point2): number {
  const c = Math.floor((p.x - map.origin.x) / map.cell), r = Math.floor((p.y - map.origin.y) / map.cell);
  if (c < 0 || r < 0 || c >= map.cols || r >= map.rows) return 0;
  return map.index[r * map.cols + c];
}

/**
 * Inverse reachability (Definition 11.3): candidate base poses for reaching `target`, i.e. target minus every offset with a
 * good index; filtered by free space and sorted by index then travel cost from `from`.
 */
export function basePlacements(map: ReachMap, target: Point2, free: (p: Point2) => boolean, from?: Point2, minIndex = 0.25, limit = 20): Array<{ base: Point2; index: number; travel: number }> {
  const out: Array<{ base: Point2; index: number; travel: number }> = [];
  for (let r = 0; r < map.rows; r++) for (let c = 0; c < map.cols; c++) {
    const idx = map.index[r * map.cols + c]; if (idx < minIndex) continue;
    const off = { x: map.origin.x + (c + 0.5) * map.cell, y: map.origin.y + (r + 0.5) * map.cell };
    const base = { x: target.x - off.x, y: target.y - off.y };
    if (!free(base)) continue;
    out.push({ base, index: idx, travel: from ? Math.hypot(base.x - from.x, base.y - from.y) : 0 });
  }
  out.sort((a, b) => b.index - a.index || a.travel - b.travel);
  return out.slice(0, limit);
}

// ---------------------------------------------------------------------------------------------
// Geometric predicates on a table top
// ---------------------------------------------------------------------------------------------

export interface TableObject { id: string; x: number; y: number; r: number }
export interface TableScene { objects: TableObject[]; /** table bounds */ xMin: number; xMax: number; yMin: number; yMax: number; /** gripper corridor half-width for a top-down approach */ approachHalfWidth: number; /** approach direction (unit) for side grasps; top grasps use a disc of radius approachHalfWidth */ approach?: Point2 }

/** Objects whose footprint intersects the approach corridor / clearance disc of `target` (blocking set B). */
export function blockingSet(scene: TableScene, targetId: string): string[] {
  const t = scene.objects.find((o) => o.id === targetId); if (!t) return [];
  const out: string[] = [];
  for (const o of scene.objects) {
    if (o.id === targetId) continue;
    if (scene.approach) {
      // corridor from the table edge along -approach to the target
      const d = { x: o.x - t.x, y: o.y - t.y }; const along = -(d.x * scene.approach.x + d.y * scene.approach.y); const perp = Math.abs(d.x * scene.approach.y - d.y * scene.approach.x);
      if (along > 0 && perp < scene.approachHalfWidth + o.r) out.push(o.id);
    } else if (Math.hypot(o.x - t.x, o.y - t.y) < scene.approachHalfWidth + t.r + o.r) out.push(o.id);
  }
  return out;
}

/** Find a free placement for object `id` that blocks neither `protect` targets nor other objects (sampled on the table). */
export function samplePlacement(scene: TableScene, id: string, protect: string[], seed = 1, tries = 400): Point2 | null {
  const o = scene.objects.find((x) => x.id === id)!;
  let s = seed >>> 0; const rnd = () => { s = (s * 1664525 + 1013904223) >>> 0; return s / 4294967296; };
  for (let k = 0; k < tries; k++) {
    const p = { x: scene.xMin + o.r + rnd() * (scene.xMax - scene.xMin - 2 * o.r), y: scene.yMin + o.r + rnd() * (scene.yMax - scene.yMin - 2 * o.r) };
    const moved: TableScene = { ...scene, objects: scene.objects.map((x) => (x.id === id ? { ...x, ...p } : x)) };
    const collides = moved.objects.some((x) => x.id !== id && Math.hypot(x.x - p.x, x.y - p.y) < x.r + o.r + 0.005);
    if (collides) continue;
    if (protect.some((t) => blockingSet(moved, t).includes(id))) continue;
    return p;
  }
  return null;
}

export interface RearrangeStep { kind: 'move' | 'pick'; object: string; to?: Point2 }

/** §11.7.2: clear the blocking set recursively (depth-bounded), then pick the target. Returns null on failure. */
export function rearrangementPlan(scene: TableScene, targetId: string, opts: { maxDepth?: number; seed?: number } = {}): { steps: RearrangeStep[]; moves: number } | null {
  const maxDepth = opts.maxDepth ?? 4; const steps: RearrangeStep[] = []; let seed = opts.seed ?? 1;
  let cur: TableScene = { ...scene, objects: scene.objects.map((o) => ({ ...o })) };
  const clear = (id: string, protect: string[], depth: number): boolean => {
    if (depth > maxDepth) return false;
    for (const b of blockingSet(cur, id)) {
      if (protect.includes(b)) continue; // the object we are clearing for is not moved
      if (!clear(b, [...protect, id], depth + 1)) return false;
      const p = samplePlacement(cur, b, [...protect, id], seed++); if (!p) return false;
      cur = { ...cur, objects: cur.objects.map((o) => (o.id === b ? { ...o, ...p } : o)) };
      steps.push({ kind: 'move', object: b, to: p });
    }
    return true;
  };
  if (!clear(targetId, [], 0)) return null;
  steps.push({ kind: 'pick', object: targetId });
  return { steps, moves: steps.filter((s) => s.kind === 'move').length };
}

// ---------------------------------------------------------------------------------------------
// Skeleton + parameters with feedback
// ---------------------------------------------------------------------------------------------

export interface GeometricCheck { /** returns null when feasible, otherwise the facts to add to the symbolic problem (e.g. [{name:'blocked', args:['bolt1','nut2']}]); `task` gives access to the atom index of the current grounding */ (action: GroundAction, state: Set<number>, task: GroundTask): Array<{ name: string; args: string[] }> | null }

export interface TampResult { found: boolean; plan: GroundAction[]; skeletons: number; feedback: string[]; last: PlanResult | null }

/** Sequential TAMP: plan a skeleton, verify geometry per action, feed the failure back as facts and replan (§11.4.1). */
export function tampSolve(domain: Domain, problem: Problem, check: GeometricCheck, opts: { maxSkeletons?: number; planOpts?: Parameters<typeof plan>[1] } = {}): TampResult {
  const feedback: string[] = []; let prob: Problem = { ...problem, init: [...problem.init] }; let last: PlanResult | null = null;
  for (let k = 1; k <= (opts.maxSkeletons ?? 10); k++) {
    const task: GroundTask = ground(domain, prob);
    last = plan(task, opts.planOpts);
    if (!last.found) return { found: false, plan: [], skeletons: k, feedback, last };
    let state = task.init; let failed = false;
    for (const a of last.plan) {
      const facts = check(a, state, task);
      if (facts && facts.length) { for (const f of facts) { if (!prob.init.some((x) => x.name === f.name && x.args.join() === f.args.join())) prob.init.push(f); feedback.push(`skeleton ${k}: ${a.name} infeasible → (${f.name} ${f.args.join(' ')})`); } failed = true; break; }
      state = new Set(state); for (const x of a.del) state.delete(x); for (const x of a.add) state.add(x);
    }
    if (!failed) return { found: true, plan: last.plan, skeletons: k, feedback, last };
  }
  return { found: false, plan: [], skeletons: opts.maxSkeletons ?? 10, feedback, last };
}

// ---------------------------------------------------------------------------------------------
// Robustness
// ---------------------------------------------------------------------------------------------

/** Success probability of a grasp with a rectangular tolerance window under Gaussian pose error σ (mm), by sampling. */
export function graspSuccessRate(sigmaMm: number, toleranceMm: { x: number; y: number }, samples = 2000, seed = 1): number {
  let s = seed >>> 0; const rnd = () => { s = (s * 1664525 + 1013904223) >>> 0; return s / 4294967296; };
  const g = () => { let acc = 0; for (let i = 0; i < 12; i++) acc += rnd(); return (acc - 6) * sigmaMm; };
  let ok = 0; for (let k = 0; k < samples; k++) if (Math.abs(g()) <= toleranceMm.x && Math.abs(g()) <= toleranceMm.y) ok++;
  return ok / samples;
}

/** Allowed perception error for a target success rate (step 7 of the design method: demand twice the margin). */
export function allowedPoseError(toleranceMm: { x: number; y: number }, targetRate = 0.9): number {
  let lo = 0, hi = Math.max(toleranceMm.x, toleranceMm.y) * 2;
  for (let i = 0; i < 30; i++) { const mid = (lo + hi) / 2; if (graspSuccessRate(mid, toleranceMm, 1500, 7) >= targetRate) lo = mid; else hi = mid; }
  return lo;
}

/** Plan robustness: perturb object positions n times and count successful replays of a rearrangement plan. */
export function planRobustness(scene: TableScene, targetId: string, sigma: number, runs = 20, seed = 3): { successRate: number; failures: number } {
  let s = seed >>> 0; const rnd = () => { s = (s * 1664525 + 1013904223) >>> 0; return s / 4294967296; };
  const g = () => { let acc = 0; for (let i = 0; i < 12; i++) acc += rnd(); return (acc - 6) * sigma; };
  const nominal = rearrangementPlan(scene, targetId, { seed });
  if (!nominal) return { successRate: 0, failures: runs };
  let ok = 0;
  for (let k = 0; k < runs; k++) {
    const perturbed: TableScene = { ...scene, objects: scene.objects.map((o) => ({ ...o, x: o.x + g(), y: o.y + g() })) };
    // replay: apply the nominal moves and check the target is clear
    let cur = perturbed;
    for (const st of nominal.steps) if (st.kind === 'move' && st.to) cur = { ...cur, objects: cur.objects.map((o) => (o.id === st.object ? { ...o, ...st.to! } : o)) };
    if (blockingSet(cur, targetId).length === 0) ok++;
  }
  return { successRate: ok / runs, failures: runs - ok };
}

export { findAction };
