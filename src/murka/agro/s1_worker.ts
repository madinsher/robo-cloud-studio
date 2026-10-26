/**
 * Module worker of benchmark S1 (s1_remote.ts): runs `runS1Benchmark` on the station of the payload and posts the
 * progress (≤ 5 messages per second, with the event log lines since the last one), then the plain-data result and the
 * download JSON. The page stays responsive while the run computes (V3 F3).
 */
import { runS1OnPayload, S1Payload, S1RemoteOptions } from './s1_remote';

const ctx = self as unknown as { postMessage(m: unknown): void; onmessage: ((e: MessageEvent) => void) | null };

ctx.onmessage = (e: MessageEvent) => {
  const { id, payload, opts } = e.data as { id: number; payload: S1Payload; opts: S1RemoteOptions };
  let lines: string[] = [];
  let last = 0;
  const flush = (progress?: { phase: string; t: number; end: number }) => { ctx.postMessage({ id, progress: progress ?? null, lines }); lines = []; };
  try {
    ctx.postMessage({ id, progress: { phase: 'world', t: 0, end: 0 } });
    const { data, json } = runS1OnPayload(payload, opts, {
      log: (m) => { lines.push(m); },
      progress: (p) => { const now = Date.now(); if (now - last >= 200) { last = now; flush(p); } },
    });
    if (lines.length) flush({ phase: 'done', t: data.timelineData.end, end: data.timelineData.end });
    ctx.postMessage({ id, ok: true, result: { data, json } });
  } catch (err) {
    ctx.postMessage({ id, ok: false, error: (err as Error)?.message ?? String(err) });
  }
};
