// V3 F3 (medium, remainder): the S1 benchmark run from the app blocked the page ~20 s (synchronous on the main thread).
// It runs in a module worker now (src/murka/agro/s1_worker.ts): the station is sent as its serialized tree + the meshes
// it references, the worker posts progress and the event log, the result comes back as plain data and plays on the
// app's own station. Checked here without a browser: the worker's computation equals the in-page run (metrics, events,
// FSM traces, contacts, timeline poses), the worker protocol (progress messages, log lines, result), and the timeline
// drives the app's robots. The browser check (Chrome, long tasks during the computation) is in docs/murka-agro.md §3.
import { describe, it, expect, beforeAll } from 'vitest';
import { ItemType } from '../src/core/items/item';
import { MobileRobot } from '../src/mobile/items';
import { HAS_MURKA, MURKA } from './webots_helpers';

describe.skipIf(!HAS_MURKA)('S1 in a Web Worker (needs the MURKA folder)', () => {
  let o: Awaited<ReturnType<typeof import('../src/murka/agro/orchard').buildMurkaOrchardStation>>;
  beforeAll(async () => { const { buildMurkaOrchardStation } = await import('../src/murka/agro/orchard'); o = await buildMurkaOrchardStation({ world: 'orchard_demo', root: MURKA }); }, 120000);

  it('the worker computes the same run as the page: metrics, events, traces, contacts, timeline', async () => {
    const { runS1Benchmark } = await import('../src/murka/agro/s1');
    const { s1Payload, runS1OnPayload, s1ResultFromData } = await import('../src/murka/agro/s1_remote');
    const payload = structuredClone(s1Payload(o.station, o.assets)); // what postMessage delivers
    expect(payload.meshes.some((m) => /pallet\.dae$/i.test(m.id))).toBe(true); // the crown mesh the tree profile is measured on
    const opts = { level: 'B' as const, controller: 'contract' as const, seed: 3 };
    const remote = runS1OnPayload(payload, opts);
    const local = runS1Benchmark(o.station, { ...opts, assets: o.assets });
    const strip = (m: any) => ({ ...m, planner: { ...m.planner, ms: 0 } }); // wall time of the planner
    expect(strip(structuredClone(remote.data.metrics))).toEqual(strip(local.metrics));
    expect(remote.data.events).toEqual(local.events);
    expect(remote.data.states).toEqual(local.states);
    expect(remote.data.fsmTraces).toEqual(local.fsmTraces);
    expect(remote.data.contacts).toEqual(local.contacts);
    expect(local.metrics.contacts.samples).toBeGreaterThan(1000);
    // the timeline rebuilt on the app's station from the plain data = the in-page timeline
    const back = s1ResultFromData(o.station, structuredClone(remote.data));
    expect(back.timeline.end).toBe(local.timeline.end);
    for (const n of local.timeline.agents) for (let t = 0; t <= local.timeline.end; t += 7.3) expect(back.timeline.poseAt(n, t), `${n} ${t}`).toEqual(local.timeline.poseAt(n, t));
    expect(back.eventsJsonl()).toBe(local.eventsJsonl());
    // ... and drives the station's robots
    const g4 = o.station.itemsOfType<MobileRobot>(ItemType.MOBILE_ROBOT).find((r) => r.name === 'UTP-G-04')!;
    const t = local.timeline.end / 2, p = local.timeline.poseAt('UTP-G-04', t);
    back.timeline.apply(t);
    expect([g4.state.x, g4.state.y, g4.state.theta]).toEqual([p.x, p.y, p.th]);
    back.timeline.apply(0);
    // the download is what the page saved before (no functions; timeline as end + robots)
    const dl = JSON.parse(remote.json);
    expect(dl.metrics.makespan).toBe(local.metrics.makespan);
    expect(dl.events.length).toBe(local.events.length);
    expect(dl.timeline).toEqual({ end: local.timeline.end, agents: local.timeline.agents });
  }, 300000);

  it('worker protocol: progress while it computes, the event log lines, then the result', async () => {
    const { s1Payload } = await import('../src/murka/agro/s1_remote');
    const posted: any[] = [];
    const g = globalThis as any;
    const prevSelf = g.self;
    g.self = { postMessage: (m: unknown) => posted.push(structuredClone(m)), onmessage: null };
    try {
      await import('../src/murka/agro/s1_worker');
      const payload = structuredClone(s1Payload(o.station, o.assets));
      await g.self.onmessage({ data: { id: 7, payload, opts: { level: 'A', controller: 'hungarian', seed: 1 } } });
    } finally { g.self = prevSelf; }
    expect(posted.every((m) => m.id === 7)).toBe(true);
    const progress = posted.filter((m) => m.progress);
    expect(progress[0].progress.phase).toBe('world');
    expect(progress.some((m) => m.progress.phase === 'mission' && m.progress.t > 0)).toBe(true);
    expect(progress.some((m) => m.progress.phase === 'contacts')).toBe(true);
    const lines = posted.flatMap((m) => m.lines ?? []);
    expect(lines.some((l: string) => /mission_start/.test(l))).toBe(true);
    expect(lines.some((l: string) => /mission_done/.test(l))).toBe(true);
    const last = posted[posted.length - 1];
    expect(last.ok).toBe(true);
    expect(last.result.data.metrics.outcome).toBe('done');
    expect(last.result.data.timelineData.agents.map((a: any) => a.name)).toEqual(['UTP-A-01', 'UTP-G-01', 'UTP-G-03', 'UTP-G-04']);
    expect(typeof last.result.json).toBe('string');
  }, 300000);
});
