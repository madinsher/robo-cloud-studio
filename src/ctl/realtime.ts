/**
 * Real-time analysis of the computing architecture (course chapter 15): periodic task sets, utilisation and the
 * Liu–Layland bound for RM, exact response-time analysis (Joseph–Pandya) with blocking terms, EDF test, priority
 * ceiling / inheritance blocking bounds, end-to-end latency of processing chains (asynchronous and synchronous),
 * WCET margins, placement rules and DDS QoS recommendations.
 */

export interface RTTask { id: string; /** WCET */ C: number; /** period / minimal inter-arrival */ T: number; /** relative deadline (default T) */ D?: number; /** longest critical section of this task per resource */ criticalSections?: Record<string, number>; /** resources used */ resources?: string[]; priority?: number }

export function utilisation(tasks: RTTask[]): number { return tasks.reduce((s, t) => s + t.C / t.T, 0); }
export function liuLaylandBound(n: number): number { return n * (Math.pow(2, 1 / n) - 1); }

/** Rate-monotonic priority order (shorter period = higher priority); explicit priorities win when given. */
export function rmOrder(tasks: RTTask[]): RTTask[] { return [...tasks].sort((a, b) => (a.priority !== undefined && b.priority !== undefined ? b.priority - a.priority : a.T - b.T)); }

export interface RTAResult { id: string; R: number; D: number; schedulable: boolean; iterations: number; B: number; interference: number }

/** Response-time analysis: R_i = C_i + B_i + Σ_{j<i} ⌈R_i/T_j⌉ C_j (Theorem 15.3, §15.5.3). */
export function responseTimes(tasks: RTTask[], blocking: Record<string, number> = {}): { results: RTAResult[]; schedulable: boolean; U: number; llBound: number; llSufficient: boolean } {
  const order = rmOrder(tasks); const results: RTAResult[] = [];
  for (let i = 0; i < order.length; i++) {
    const t = order[i]; const D = t.D ?? t.T; const B = blocking[t.id] ?? 0;
    let R = t.C + B; let it = 0; let ok = true;
    for (; it < 10000; it++) {
      const next = t.C + B + order.slice(0, i).reduce((s, hp) => s + Math.ceil(R / hp.T) * hp.C, 0);
      if (next === R) break;
      R = next; if (R > D) { ok = false; break; }
    }
    results.push({ id: t.id, R, D, schedulable: ok && R <= D, iterations: it + 1, B, interference: R - t.C - B });
  }
  const U = utilisation(tasks); const llBound = liuLaylandBound(tasks.length);
  return { results, schedulable: results.every((r) => r.schedulable), U, llBound, llSufficient: U <= llBound && tasks.every((t) => (t.D ?? t.T) >= t.T) };
}

/** EDF: schedulable iff U ≤ 1 for implicit deadlines (Theorem 15.4); processor-demand check for constrained deadlines. */
export function edfTest(tasks: RTTask[]): { schedulable: boolean; U: number; note: string } {
  const U = utilisation(tasks);
  if (tasks.every((t) => (t.D ?? t.T) >= t.T)) return { schedulable: U <= 1 + 1e-12, U, note: 'implicit deadlines: U ≤ 1 is necessary and sufficient' };
  // processor demand bound over the hyperperiod-limited horizon
  const L = Math.min(1e6, lcm(tasks.map((t) => t.T)));
  const points = new Set<number>(); for (const t of tasks) for (let k = 0; ; k++) { const d = (t.D ?? t.T) + k * t.T; if (d > L) break; points.add(d); }
  for (const l of points) { const demand = tasks.reduce((s, t) => s + Math.max(0, Math.floor((l - (t.D ?? t.T)) / t.T) + 1) * t.C, 0); if (demand > l) return { schedulable: false, U, note: `demand ${demand} exceeds interval ${l}` }; }
  return { schedulable: true, U, note: 'processor demand criterion satisfied' };
}
function gcd(a: number, b: number): number { return b ? gcd(b, a % b) : a; }
function lcm(xs: number[]): number { return xs.reduce((a, b) => (a * b) / gcd(a, b), 1); }

/** Blocking bounds: PCP → longest single critical section of a lower-priority task on a resource with ceiling ≥ own priority; PIP → sum over such sections (Theorem 15.5 / Proposition 15.3). */
export function blockingBounds(tasks: RTTask[], protocol: 'pcp' | 'pip' | 'none'): Record<string, number> {
  const order = rmOrder(tasks); const out: Record<string, number> = {};
  const ceiling = new Map<string, number>(); // resource → index of the highest-priority user
  order.forEach((t, i) => { for (const r of t.resources ?? Object.keys(t.criticalSections ?? {})) if (!ceiling.has(r) || ceiling.get(r)! > i) ceiling.set(r, i); });
  order.forEach((t, i) => {
    if (protocol === 'none') { out[t.id] = Infinity; return; }
    const sections: number[] = [];
    order.slice(i + 1).forEach((lp) => { for (const [r, len] of Object.entries(lp.criticalSections ?? {})) if ((ceiling.get(r) ?? Infinity) <= i) sections.push(len); });
    out[t.id] = protocol === 'pcp' ? Math.max(0, ...sections) : sections.reduce((s, v) => s + v, 0);
  });
  // tasks using no shared resource with anyone above them are never blocked
  for (const t of order) if (!Object.keys(t.criticalSections ?? {}).length && !(t.resources ?? []).length && protocol !== 'none') out[t.id] = out[t.id] ?? 0;
  return out;
}

/** End-to-end latency of a chain (§15.7.2): asynchronous (register buffers) Σ(T_i + R_i); synchronous (event-triggered) Σ R_i. */
export function endToEndLatency(chain: Array<{ id: string; T: number; R: number }>, mode: 'async' | 'sync'): { latency: number; contributions: Array<{ id: string; value: number }> } {
  const contributions = chain.map((s) => ({ id: s.id, value: mode === 'async' ? s.T + s.R : s.R }));
  return { latency: contributions.reduce((a, c) => a + c.value, 0), contributions };
}

/** Positioning error from latency at an approach speed, and the speed that meets a tolerance. */
export function latencyBudget(latencyS: number, speedMps: number, toleranceM: number): { errorM: number; ok: boolean; maxSpeedMps: number } { return { errorM: speedMps * latencyS, ok: speedMps * latencyS <= toleranceM, maxSpeedMps: toleranceM / latencyS }; }

/** WCET from measurements: max, p99.9 and a margin (30 % firm, 50 % hard) — a convention, not a guarantee (§15.6). */
export function wcetEstimate(samples: number[], criticality: 'hard' | 'firm' | 'soft'): { max: number; median: number; ratio: number; budget: number; margin: number } {
  const s = [...samples].sort((a, b) => a - b); const max = s[s.length - 1] ?? 0; const median = s[Math.floor(s.length / 2)] ?? 0;
  const margin = criticality === 'hard' ? 0.5 : criticality === 'firm' ? 0.3 : 0.1;
  return { max, median, ratio: median ? max / median : 0, budget: max * (1 + margin), margin };
}

export type DdsStream = 'sensor_stream' | 'command' | 'event' | 'state' | 'safety';
/** DDS QoS recommendation table (§15.8.3). */
export function qosRecommendation(kind: DdsStream, rateHz?: number): { reliability: string; durability: string; history: string; deadline: string; liveliness: string; rationale: string } {
  switch (kind) {
    case 'sensor_stream': return { reliability: 'BEST_EFFORT', durability: 'VOLATILE', history: 'KEEP_LAST(1–5)', deadline: rateHz ? `${(2000 / rateHz).toFixed(0)} ms` : '2 periods', liveliness: 'AUTOMATIC', rationale: 'retransmitting a stale frame only adds latency; the deadline detects a dead source' };
    case 'command': return { reliability: 'RELIABLE', durability: 'VOLATILE', history: 'KEEP_LAST(10)', deadline: rateHz ? `${(2000 / rateHz).toFixed(0)} ms` : 'command period', liveliness: 'MANUAL_BY_TOPIC', rationale: 'commands must arrive; liveliness lets the actuator stop when the commander dies' };
    case 'event': return { reliability: 'RELIABLE', durability: 'TRANSIENT_LOCAL', history: 'KEEP_ALL', deadline: 'none', liveliness: 'AUTOMATIC', rationale: 'late joiners must see the last event (e.g. e-stop state)' };
    case 'state': return { reliability: 'RELIABLE', durability: 'TRANSIENT_LOCAL', history: 'KEEP_LAST(1)', deadline: '1 s', liveliness: 'AUTOMATIC', rationale: 'latest state for new subscribers' };
    case 'safety': return { reliability: 'RELIABLE', durability: 'TRANSIENT_LOCAL', history: 'KEEP_LAST(1)', deadline: '≤ 50 ms', liveliness: 'MANUAL_BY_TOPIC', rationale: 'missing heartbeat = fault → safe stop; note: software QoS does not replace the certified safety channel' };
  }
}

/** Placement rules for a function (§15.8.2). */
export function placementAdvice(fn: { name: string; closedLoop: boolean; safety: boolean; rateHz: number; heavy: boolean; dataSource?: 'sensor' | 'network' | 'none' }): { level: 'drive' | 'robot' | 'cell' | 'plant' | 'enterprise'; reasons: string[] } {
  const reasons: string[] = [];
  if (fn.safety) { reasons.push('protective functions as low as possible (rule 2)'); return { level: fn.rateHz >= 500 ? 'drive' : 'robot', reasons }; }
  if (fn.closedLoop) { reasons.push('a closed loop must not cross an unreliable boundary (rule 1)'); return { level: fn.rateHz >= 500 ? 'drive' : 'robot', reasons }; }
  if (fn.heavy && fn.rateHz < 1) { reasons.push('heavy and non-critical computations go up (rule 4)'); return { level: fn.rateHz < 0.01 ? 'enterprise' : 'plant', reasons }; }
  if (fn.dataSource === 'sensor') { reasons.push('process data close to its source (rule 5)'); return { level: 'robot', reasons }; }
  reasons.push('longer horizon / lower rate → higher level (rule 3)');
  return { level: fn.rateHz >= 10 ? 'cell' : 'plant', reasons };
}

/** Course §15.3.4 task set. */
export const COURSE_TASKS: RTTask[] = [
  { id: 'drive loop', C: 1.2, T: 5 }, { id: 'lidar + localization', C: 8, T: 50 }, { id: 'local planner', C: 25, T: 100 }, { id: 'perception', C: 30, T: 200 },
];
