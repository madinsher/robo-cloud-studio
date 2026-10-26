/**
 * docs/murka-agro.md §3 S1 results table = what the benchmark computes: the 90 runs (levels A / B / C × Hungarian /
 * contract net / hybrid × seeds 0..9) are re-run and the medians of the makespan and of the "decision messages / matrix
 * cells" column (`metrics.comm.messages`, `metrics.comm.matrixCells`) must equal the published values.
 */
import { describe, it, expect, beforeAll } from 'vitest';
import * as fs from 'node:fs';
import * as path from 'node:path';
import { buildMurkaOrchardStation, OrchardStationResult } from '../src/murka/agro/orchard';
import { runS1Benchmark } from '../src/murka/agro/s1';
import { HAS_MURKA, MURKA } from './webots_helpers';

const DOC = process.env.MURKA_AGRO_DOC ?? path.join(__dirname, '..', 'docs', 'murka-agro.md');
const LEVELS = ['A', 'B', 'C'] as const;
const CONTROLLERS = { Hungarian: 'hungarian', 'contract net': 'contract', hybrid: 'hybrid' } as const;

const median = (v: number[]) => { const s = [...v].sort((a, b) => a - b); const n = s.length; return n % 2 ? s[(n - 1) / 2] : (s[n / 2 - 1] + s[n / 2]) / 2; };

/** Rows of the S1 results table: level, controller, makespan median, messages median, matrix cells median. */
function docTable(): Array<{ level: string; controller: string; makespan: number; messages: number | null; cells: number | null }> {
  const lines = fs.readFileSync(DOC, 'utf8').split(/\r?\n/);
  const out: Array<{ level: string; controller: string; makespan: number; messages: number | null; cells: number | null }> = [];
  for (const l of lines) {
    const m = /^\| ([ABC]) (Hungarian|contract net|hybrid) \|(.*)\|\s*$/.exec(l);
    if (!m) continue;
    const cols = m[3].split('|').map((c) => c.trim());
    const dec = cols[5];
    const msg = /(\d+(?:\.\d+)?)(?: \[[^\]]*\])? messages/.exec(dec);
    const cells = /(\d+(?:\.\d+)?) cells/.exec(dec);
    out.push({ level: m[1], controller: CONTROLLERS[m[2] as keyof typeof CONTROLLERS], makespan: Number(cols[0].split(' ')[0]), messages: msg ? Number(msg[1]) : null, cells: cells ? Number(cells[1]) : null });
  }
  return out;
}

describe.skipIf(!HAS_MURKA)('S1 results table of docs/murka-agro.md (needs the MURKA folder)', () => {
  let o: OrchardStationResult;
  beforeAll(async () => { o = await buildMurkaOrchardStation({ world: 'orchard_demo', root: MURKA }); }, 120000);

  it('every row: makespan and decision messages / matrix cells medians over seeds 0..9 equal the published values', () => {
    const rows = docTable();
    expect(rows).toHaveLength(9);
    const got: string[] = [], want: string[] = [];
    for (const level of LEVELS) for (const controller of Object.values(CONTROLLERS)) {
      const doc = rows.find((r) => r.level === level && r.controller === controller)!;
      const runs = Array.from({ length: 10 }, (_, seed) => runS1Benchmark(o.station, { level, controller, seed, assets: o.assets, contacts: false }).metrics);
      expect(runs.every((m) => m.outcome === 'done')).toBe(true);
      const messages = median(runs.map((m) => m.comm.messages));
      const cells = median(runs.map((m) => m.comm.matrixCells));
      // the column shows messages for every controller and matrix cells where the Hungarian solver ran
      got.push(`${level} ${controller}: makespan ${Math.round(median(runs.map((m) => m.makespan)))} messages ${messages} cells ${cells || null}`);
      want.push(`${level} ${controller}: makespan ${doc.makespan} messages ${doc.messages} cells ${doc.cells}`);
    }
    expect(got).toEqual(want);
  }, 600000);
});
