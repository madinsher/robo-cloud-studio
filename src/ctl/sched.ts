/**
 * Dispatching and production scheduling (course chapter 14): single-machine rules (SPT, EDD, Moore–Hodgson,
 * Johnson for F2), job-shop scheduling on the disjunctive graph (dispatch rules SPT / LPT / EDD / MWKR / FIFO /
 * CR, critical path, lower bounds LB1–LB3, branch-and-bound for small instances, tabu search on the critical path
 * N1 neighbourhood), flexible job shop (machine alternatives), rescheduling (right shift, full re-plan with a
 * stability penalty), robustness Monte Carlo, production metrics (OEE, WIP by Little's law).
 */
import { mulberry32, randn, stats } from './perf';

// ---------------------------------------------------------------------------------------------
// Single machine
// ---------------------------------------------------------------------------------------------

export interface SingleJob { id: string; p: number; d?: number; r?: number; w?: number }
export interface SingleResult { order: string[]; completion: Record<string, number>; sumC: number; Lmax: number; late: number; sumT: number }

export function evaluateSequence(jobs: SingleJob[], order: string[]): SingleResult {
  const byId = new Map(jobs.map((j) => [j.id, j])); let t = 0; const completion: Record<string, number> = {}; let sumC = 0, Lmax = -Infinity, late = 0, sumT = 0;
  for (const id of order) { const j = byId.get(id)!; t = Math.max(t, j.r ?? 0) + j.p; completion[id] = t; sumC += t; const L = t - (j.d ?? Infinity); if (j.d !== undefined) { Lmax = Math.max(Lmax, L); if (L > 0) { late++; sumT += L; } } }
  return { order, completion, sumC, Lmax: Number.isFinite(Lmax) ? Lmax : 0, late, sumT };
}
/** SPT: minimises Σ C_j (Theorem 14.1). */
export const spt = (jobs: SingleJob[]) => evaluateSequence(jobs, [...jobs].sort((a, b) => a.p - b.p).map((j) => j.id));
/** EDD: minimises L_max (Theorem 14.2). */
export const edd = (jobs: SingleJob[]) => evaluateSequence(jobs, [...jobs].sort((a, b) => (a.d ?? Infinity) - (b.d ?? Infinity)).map((j) => j.id));
/** Moore–Hodgson: minimises the number of late jobs. */
export function moore(jobs: SingleJob[]): SingleResult {
  const sorted = [...jobs].sort((a, b) => (a.d ?? Infinity) - (b.d ?? Infinity)); const onTime: SingleJob[] = []; const rejected: SingleJob[] = []; let t = 0;
  for (const j of sorted) { onTime.push(j); t += j.p; if (t > (j.d ?? Infinity)) { const k = onTime.reduce((a, b) => (b.p > a.p ? b : a)); onTime.splice(onTime.indexOf(k), 1); rejected.push(k); t -= k.p; } }
  return evaluateSequence(jobs, [...onTime, ...rejected].map((j) => j.id));
}
/** Johnson's rule for F2 || C_max: jobs with p1 ≤ p2 first by increasing p1, the rest by decreasing p2 (Theorem 14.3). */
export function johnson(jobs: Array<{ id: string; p1: number; p2: number }>): { order: string[]; makespan: number } {
  const first = jobs.filter((j) => j.p1 <= j.p2).sort((a, b) => a.p1 - b.p1), second = jobs.filter((j) => j.p1 > j.p2).sort((a, b) => b.p2 - a.p2);
  const order = [...first, ...second]; let t1 = 0, t2 = 0;
  for (const j of order) { t1 += j.p1; t2 = Math.max(t1, t2) + j.p2; }
  return { order: order.map((j) => j.id), makespan: t2 };
}

// ---------------------------------------------------------------------------------------------
// Job shop
// ---------------------------------------------------------------------------------------------

export interface Operation { /** alternative machines with durations (FJSP); one entry = classic job shop */ alternatives: Array<{ machine: string; p: number }> }
export interface Job { id: string; ops: Operation[]; due?: number; release?: number; weight?: number; type?: string }
export interface JobShop { jobs: Job[]; machines: string[]; /** sequence-dependent setup time when a machine switches between job types */ setup?: (fromType: string | undefined, toType: string | undefined) => number }
export interface ScheduledOp { job: string; op: number; machine: string; start: number; end: number }
export interface Schedule { ops: ScheduledOp[]; makespan: number; jobCompletion: Record<string, number>; machineLoad: Record<string, number>; sumC: number; Lmax: number; sumT: number; late: number }

export type DispatchRule = 'SPT' | 'LPT' | 'EDD' | 'MWKR' | 'FIFO' | 'CR' | 'ATC';

function finish(shop: JobShop, ops: ScheduledOp[]): Schedule {
  const jobCompletion: Record<string, number> = {}; const machineLoad: Record<string, number> = Object.fromEntries(shop.machines.map((m) => [m, 0]));
  for (const o of ops) { jobCompletion[o.job] = Math.max(jobCompletion[o.job] ?? 0, o.end); machineLoad[o.machine] = (machineLoad[o.machine] ?? 0) + (o.end - o.start); }
  const makespan = Math.max(0, ...ops.map((o) => o.end));
  let sumC = 0, Lmax = 0, sumT = 0, late = 0;
  for (const j of shop.jobs) { const C = jobCompletion[j.id] ?? 0; sumC += C; if (j.due !== undefined) { const L = C - j.due; Lmax = Math.max(Lmax, L); if (L > 0) { late++; sumT += L; } } }
  return { ops, makespan, jobCompletion, machineLoad, sumC, Lmax, sumT, late };
}

/** Event-driven list scheduling with a dispatch rule; FJSP alternatives pick the machine with the earliest finish. */
export function dispatch(shop: JobShop, rule: DispatchRule, opts: { seed?: number } = {}): Schedule {
  const next = new Map(shop.jobs.map((j) => [j.id, 0])); const jobReady = new Map(shop.jobs.map((j) => [j.id, j.release ?? 0]));
  const machineFree = new Map(shop.machines.map((m) => [m, 0])); const machineType = new Map<string, string | undefined>();
  const ops: ScheduledOp[] = []; const remaining = (j: Job, k: number) => j.ops.slice(k).reduce((s, o) => s + Math.min(...o.alternatives.map((a) => a.p)), 0);
  const seq = new Map(shop.jobs.map((j, i) => [j.id, i]));
  void opts;
  for (let guard = 0; guard < 100000; guard++) {
    const cands = shop.jobs.filter((j) => next.get(j.id)! < j.ops.length);
    if (!cands.length) break;
    // earliest possible start for each candidate (best alternative)
    const scored = cands.map((j) => {
      const k = next.get(j.id)!; const op = j.ops[k];
      let best = { machine: '', start: Infinity, p: 0, setup: 0 };
      for (const alt of op.alternatives) { const setup = shop.setup ? shop.setup(machineType.get(alt.machine), j.type) : 0; const start = Math.max(jobReady.get(j.id)!, machineFree.get(alt.machine)! + setup); if (start + alt.p < best.start + best.p) best = { machine: alt.machine, start, p: alt.p, setup }; }
      return { j, k, ...best };
    });
    const t = Math.min(...scored.map((s) => s.start));
    const ready = scored.filter((s) => s.start <= t + 1e-9);
    const key = (s: typeof ready[number]): number => {
      switch (rule) {
        case 'SPT': return s.p; case 'LPT': return -s.p; case 'EDD': return s.j.due ?? Infinity; case 'MWKR': return -remaining(s.j, s.k); case 'FIFO': return seq.get(s.j.id)!;
        case 'CR': return ((s.j.due ?? Infinity) - t) / Math.max(1e-9, remaining(s.j, s.k));
        case 'ATC': { const slack = Math.max(0, (s.j.due ?? Infinity) - s.p - t); const pAvg = scored.reduce((a, b) => a + b.p, 0) / scored.length; return -((s.j.weight ?? 1) / s.p) * Math.exp(-slack / (2 * pAvg)); }
      }
    };
    ready.sort((a, b) => key(a) - key(b) || seq.get(a.j.id)! - seq.get(b.j.id)!);
    const c = ready[0];
    ops.push({ job: c.j.id, op: c.k, machine: c.machine, start: c.start, end: c.start + c.p });
    machineFree.set(c.machine, c.start + c.p); machineType.set(c.machine, c.j.type); jobReady.set(c.j.id, c.start + c.p); next.set(c.j.id, c.k + 1);
  }
  return finish(shop, ops);
}

/** Lower bounds LB1 (machine load), LB2 (job length), LB3 (heads + load + tails), Theorem 14.6; FJSP uses the minimum duration per operation. */
export function lowerBounds(shop: JobShop): { lb1: number; lb2: number; lb3: number; lb: number; bottleneck: string } {
  const minP = (o: Operation) => Math.min(...o.alternatives.map((a) => a.p));
  const load: Record<string, number> = Object.fromEntries(shop.machines.map((m) => [m, 0]));
  const heads: Record<string, number[]> = Object.fromEntries(shop.machines.map((m) => [m, []])), tails: Record<string, number[]> = Object.fromEntries(shop.machines.map((m) => [m, []]));
  for (const j of shop.jobs) {
    let head = j.release ?? 0;
    j.ops.forEach((o, k) => {
      const tail = j.ops.slice(k + 1).reduce((s, x) => s + minP(x), 0);
      if (o.alternatives.length === 1) { const m = o.alternatives[0].machine; load[m] += o.alternatives[0].p; heads[m].push(head); tails[m].push(tail); }
      else { // flexible: distribute the minimum load evenly as a valid (weak) bound
        const share = minP(o) / o.alternatives.length; for (const a of o.alternatives) { load[a.machine] += share; heads[a.machine].push(head); tails[a.machine].push(tail); }
      }
      head += minP(o);
    });
  }
  const lb1 = Math.max(0, ...Object.values(load)); const bottleneck = shop.machines.reduce((a, b) => (load[b] > load[a] ? b : a));
  const lb2 = Math.max(0, ...shop.jobs.map((j) => (j.release ?? 0) + j.ops.reduce((s, o) => s + minP(o), 0)));
  const lb3 = Math.max(0, ...shop.machines.map((m) => (heads[m].length ? Math.min(...heads[m]) + load[m] + Math.min(...tails[m]) : 0)));
  return { lb1, lb2, lb3, lb: Math.max(lb1, lb2, lb3), bottleneck };
}

/** Flexible-shop assignment bound: try every machine assignment (small instances) and take the best balanced load. */
export function bestAssignmentBound(shop: JobShop, maxCombos = 20000): { lb: number; assignment: Record<string, string[]> } {
  const flexOps = shop.jobs.flatMap((j) => j.ops.map((o, k) => ({ job: j.id, k, o }))).filter((x) => x.o.alternatives.length > 1);
  let best = Infinity; let bestAssign: Record<string, string[]> = {};
  const total = flexOps.reduce((s, x) => s * x.o.alternatives.length, 1);
  if (total > maxCombos) return { lb: lowerBounds(shop).lb, assignment: {} };
  for (let c = 0; c < total; c++) {
    let rem = c; const load: Record<string, number> = Object.fromEntries(shop.machines.map((m) => [m, 0])); const assign: Record<string, string[]> = Object.fromEntries(shop.jobs.map((j) => [j.id, []]));
    for (const j of shop.jobs) for (const [k, o] of j.ops.entries()) { const fx = flexOps.find((x) => x.job === j.id && x.k === k); let alt = o.alternatives[0]; if (fx) { const i = rem % o.alternatives.length; rem = Math.floor(rem / o.alternatives.length); alt = o.alternatives[i]; } load[alt.machine] += alt.p; assign[j.id][k] = alt.machine; }
    const lb = Math.max(...Object.values(load)); if (lb < best) { best = lb; bestAssign = assign; }
  }
  return { lb: best, assignment: bestAssign };
}

/** Critical path of a schedule: chain of operations whose delay would delay the makespan (job or machine predecessors with zero slack). */
export function criticalPath(s: Schedule): ScheduledOp[] {
  const last = s.ops.reduce((a, b) => (b.end > a.end ? b : a)); const path: ScheduledOp[] = [last]; let cur = last;
  for (let g = 0; g < s.ops.length; g++) {
    const pred = s.ops.find((o) => o !== cur && Math.abs(o.end - cur.start) < 1e-9 && ((o.job === cur.job && o.op === cur.op - 1) || o.machine === cur.machine));
    if (!pred) break; path.push(pred); cur = pred;
  }
  return path.reverse();
}

/** Machine-order representation: for each machine the sequence of (job, op); evaluate by longest paths on the disjunctive graph (Theorem 14.4). */
export function scheduleFromOrders(shop: JobShop, orders: Record<string, Array<{ job: string; op: number }>>, assign: (job: string, op: number) => { machine: string; p: number }): Schedule | null {
  const start = new Map<string, number>(); const ops: ScheduledOp[] = [];
  const key = (j: string, k: number) => `${j}#${k}`;
  const mPrev = new Map<string, { job: string; op: number }>(); for (const [m, seq] of Object.entries(orders)) seq.forEach((x, i) => { if (i > 0) mPrev.set(key(x.job, x.op), seq[i - 1]); void m; });
  const visiting = new Set<string>();
  const startOf = (j: string, k: number): number | null => {
    const kk = key(j, k); if (start.has(kk)) return start.get(kk)!; if (visiting.has(kk)) return null; visiting.add(kk);
    const job = shop.jobs.find((x) => x.id === j)!; let t = job.release ?? 0;
    if (k > 0) { const s = startOf(j, k - 1); if (s === null) return null; t = Math.max(t, s + assign(j, k - 1).p); }
    const mp = mPrev.get(kk); if (mp) { const s = startOf(mp.job, mp.op); if (s === null) return null; t = Math.max(t, s + assign(mp.job, mp.op).p); }
    visiting.delete(kk); start.set(kk, t); return t;
  };
  for (const j of shop.jobs) for (let k = 0; k < j.ops.length; k++) { const s = startOf(j.id, k); if (s === null) return null; const a = assign(j.id, k); ops.push({ job: j.id, op: k, machine: a.machine, start: s, end: s + a.p }); }
  return finish(shop, ops);
}

/** Tabu search on the N1 neighbourhood (swap adjacent critical operations on the same machine), starting from a dispatch schedule. */
export function tabuSearch(shop: JobShop, opts: { iterations?: number; tabuTenure?: number; start?: Schedule } = {}): { schedule: Schedule; iterations: number; improved: number } {
  let cur = opts.start ?? dispatch(shop, 'MWKR');
  const assignOf = (s: Schedule) => (job: string, op: number) => { const o = s.ops.find((x) => x.job === job && x.op === op)!; return { machine: o.machine, p: o.end - o.start }; };
  const ordersOf = (s: Schedule): Record<string, Array<{ job: string; op: number }>> => { const o: Record<string, Array<{ job: string; op: number }>> = Object.fromEntries(shop.machines.map((m) => [m, []])); for (const x of [...s.ops].sort((a, b) => a.start - b.start)) o[x.machine].push({ job: x.job, op: x.op }); return o; };
  let best = cur; const tabu: string[] = []; let improved = 0; const tenure = opts.tabuTenure ?? 7;
  for (let it = 0; it < (opts.iterations ?? 200); it++) {
    const cp = criticalPath(cur); const orders = ordersOf(cur); const assign = assignOf(cur);
    let bestMove: { s: Schedule; key: string } | null = null;
    for (let i = 0; i + 1 < cp.length; i++) {
      const a = cp[i], b = cp[i + 1]; if (a.machine !== b.machine || a.job === b.job) continue;
      const seq = orders[a.machine]; const ia = seq.findIndex((x) => x.job === a.job && x.op === a.op), ib = seq.findIndex((x) => x.job === b.job && x.op === b.op); if (Math.abs(ia - ib) !== 1) continue;
      const nseq = [...seq]; [nseq[ia], nseq[ib]] = [nseq[ib], nseq[ia]];
      const s = scheduleFromOrders(shop, { ...orders, [a.machine]: nseq }, assign); if (!s) continue;
      const k = `${a.job}#${a.op}<>${b.job}#${b.op}`; if (tabu.includes(k) && s.makespan >= best.makespan) continue;
      if (!bestMove || s.makespan < bestMove.s.makespan) bestMove = { s, key: k };
    }
    if (!bestMove) break;
    cur = bestMove.s; tabu.push(bestMove.key); if (tabu.length > tenure) tabu.shift();
    if (cur.makespan < best.makespan) { best = cur; improved++; }
  }
  return { schedule: best, iterations: opts.iterations ?? 200, improved };
}

/** Branch and bound over machine orders for small instances (≤ ~8 operations per machine); returns the optimal makespan with the LB3-based bound. */
export function branchAndBound(shop: JobShop, opts: { maxNodes?: number } = {}): { schedule: Schedule; optimal: boolean; nodes: number } {
  const heur = tabuSearch(shop, { iterations: 100 }).schedule; let best = heur; let nodes = 0; let optimal = true;
  const minP = (o: Operation) => Math.min(...o.alternatives.map((a) => a.p));
  // state: next op per job, machine free times, job ready times, partial ops
  const rec = (next: number[], mFree: Map<string, number>, jReady: number[], ops: ScheduledOp[]): void => {
    if (++nodes > (opts.maxNodes ?? 200000)) { optimal = false; return; }
    const cands = shop.jobs.map((j, i) => i).filter((i) => next[i] < shop.jobs[i].ops.length);
    if (!cands.length) { const s = finish(shop, ops); if (s.makespan < best.makespan) best = s; return; }
    // bound: max over jobs of ready + remaining, and machine free + remaining load
    const loadLeft: Record<string, number> = Object.fromEntries(shop.machines.map((m) => [m, 0]));
    let lb = 0;
    for (const i of cands) { const j = shop.jobs[i]; let t = jReady[i]; for (let k = next[i]; k < j.ops.length; k++) { const o = j.ops[k]; t += minP(o); if (o.alternatives.length === 1) loadLeft[o.alternatives[0].machine] += o.alternatives[0].p; } lb = Math.max(lb, t); }
    for (const m of shop.machines) lb = Math.max(lb, mFree.get(m)! + loadLeft[m]);
    lb = Math.max(lb, ...ops.map((o) => o.end));
    if (lb >= best.makespan) return;
    // active schedule generation (Giffler–Thompson): earliest completion among candidates, branch on conflicting ops
    const ec = cands.map((i) => { const o = shop.jobs[i].ops[next[i]]; let bestAlt = { machine: '', start: Infinity, p: 0 }; for (const a of o.alternatives) { const st = Math.max(jReady[i], mFree.get(a.machine)!); if (st + a.p < bestAlt.start + bestAlt.p) bestAlt = { machine: a.machine, start: st, p: a.p }; } return { i, ...bestAlt }; });
    const minEnd = Math.min(...ec.map((x) => x.start + x.p)); const pivot = ec.find((x) => x.start + x.p === minEnd)!;
    const conflict = ec.filter((x) => x.machine === pivot.machine && x.start < minEnd);
    for (const c of conflict) {
      const nf = new Map(mFree); nf.set(c.machine, c.start + c.p); const nr = [...jReady]; nr[c.i] = c.start + c.p; const nn = [...next]; nn[c.i]++;
      rec(nn, nf, nr, [...ops, { job: shop.jobs[c.i].id, op: next[c.i], machine: c.machine, start: c.start, end: c.start + c.p }]);
    }
  };
  rec(shop.jobs.map(() => 0), new Map(shop.machines.map((m) => [m, 0])), shop.jobs.map((j) => j.release ?? 0), []);
  return { schedule: best, optimal, nodes };
}

// ---------------------------------------------------------------------------------------------
// Rescheduling and robustness
// ---------------------------------------------------------------------------------------------

/** Right-shift after a delay of `delay` on `machine` starting at time `t` (all later operations on that machine and their job successors slide). */
export function rightShift(shop: JobShop, s: Schedule, machine: string, t: number, delay: number): Schedule {
  const orders: Record<string, Array<{ job: string; op: number }>> = Object.fromEntries(shop.machines.map((m) => [m, []]));
  for (const x of [...s.ops].sort((a, b) => a.start - b.start)) orders[x.machine].push({ job: x.job, op: x.op });
  const assign = (job: string, op: number) => { const o = s.ops.find((x) => x.job === job && x.op === op)!; const p = o.end - o.start; return { machine: o.machine, p: o.machine === machine && o.start >= t && o === firstAfter ? p + delay : p }; };
  const firstAfter = s.ops.filter((o) => o.machine === machine && o.end > t).sort((a, b) => a.start - b.start)[0];
  return scheduleFromOrders(shop, orders, assign) ?? s;
}

/** Stability of a new schedule vs the old one: Σ |s_new − s_old| and the number of moved operations (§14.9.2). */
export function scheduleDeviation(oldS: Schedule, newS: Schedule): { shift: number; moved: number } {
  let shift = 0, moved = 0;
  for (const o of newS.ops) { const p = oldS.ops.find((x) => x.job === o.job && x.op === o.op); if (!p) continue; const d = Math.abs(o.start - p.start); shift += d; if (d > 1e-9 || p.machine !== o.machine) moved++; }
  return { shift, moved };
}

/** Monte Carlo robustness: execute the machine orders with duration noise (cv) and report the makespan distribution. */
export function scheduleRobustness(shop: JobShop, s: Schedule, cv: number, runs = 50, seed = 1): ReturnType<typeof stats> {
  const rng = mulberry32(seed); const orders: Record<string, Array<{ job: string; op: number }>> = Object.fromEntries(shop.machines.map((m) => [m, []]));
  for (const x of [...s.ops].sort((a, b) => a.start - b.start)) orders[x.machine].push({ job: x.job, op: x.op });
  const ms: number[] = [];
  for (let r = 0; r < runs; r++) { const noise = new Map<string, number>(); const assign = (job: string, op: number) => { const o = s.ops.find((x) => x.job === job && x.op === op)!; const k = `${job}#${op}`; if (!noise.has(k)) noise.set(k, Math.max(0.1, 1 + cv * randn(rng))); return { machine: o.machine, p: (o.end - o.start) * noise.get(k)! }; }; const sim = scheduleFromOrders(shop, orders, assign); if (sim) ms.push(sim.makespan); }
  return stats(ms);
}

// ---------------------------------------------------------------------------------------------
// Metrics
// ---------------------------------------------------------------------------------------------

/** OEE = availability × performance × quality (§14.10 example: 0.900 × 0.860 × 0.981 = 0.759). */
export function oee(v: { shiftMin: number; plannedBreaksMin: number; downtimeMin: number; nominalCycleS: number; produced: number; scrap: number }): { availability: number; performance: number; quality: number; oee: number; dominantLoss: string } {
  const planned = v.shiftMin - v.plannedBreaksMin; const run = planned - v.downtimeMin;
  const availability = run / planned; const performance = (v.produced * v.nominalCycleS / 60) / run; const quality = (v.produced - v.scrap) / v.produced;
  const losses = { availability, performance, quality }; const dominantLoss = (Object.entries(losses).sort((a, b) => a[1] - b[1])[0])[0];
  return { availability, performance, quality, oee: availability * performance * quality, dominantLoss };
}

/** Graham notation classification helper (α | β | γ) → complexity note. */
export function classify(alpha: string, beta: string[], gamma: string): { notation: string; complexity: string; method: string } {
  const notation = `${alpha} | ${beta.join(', ')} | ${gamma}`;
  const table: Array<[RegExp, RegExp, string, string]> = [
    [/^1$/, /^ΣC/, 'O(n log n)', 'SPT rule'], [/^1$/, /^Lmax/, 'O(n log n)', 'EDD rule'], [/^1$/, /^ΣU/, 'O(n log n)', 'Moore–Hodgson'], [/^1$/, /^ΣT/, 'NP-hard', 'branch and bound / dynamic programming'],
    [/^F2$/, /^Cmax/, 'O(n log n)', 'Johnson'], [/^P/, /^Cmax/, 'NP-hard', 'LPT (4/3-approximation) / CP-SAT'], [/^J/, /^Cmax/, 'NP-hard (m ≥ 3)', 'dispatch rules, tabu search, CP-SAT'], [/^FJ/, /./, 'NP-hard', 'assignment + sequencing; CP-SAT / metaheuristics'], [/^O/, /^Cmax/, 'NP-hard (m ≥ 3)', 'dense schedules'],
  ];
  const hit = table.find(([a, g]) => a.test(alpha) && g.test(gamma));
  return { notation, complexity: hit?.[2] ?? 'see Pinedo (2022)', method: hit?.[3] ?? 'constraint programming' };
}

/** Course §14.11 cell instance: three jobs, loader M1 (5 s), machine S1 or S2 (type-dependent), unloader M2 (5 s). */
export function cellJobShop(): JobShop {
  const job = (id: string, p1: number, p2: number, type: string): Job => ({ id, type, ops: [{ alternatives: [{ machine: 'M1', p: 5 }] }, { alternatives: [{ machine: 'S1', p: p1 }, { machine: 'S2', p: p2 }] }, { alternatives: [{ machine: 'M2', p: 5 }] }] });
  return { machines: ['M1', 'S1', 'S2', 'M2'], jobs: [job('J1', 22, 26, 'A'), job('J2', 30, 24, 'B'), job('J3', 22, 26, 'A')] };
}
