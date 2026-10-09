import { describe, it, expect } from 'vitest';
import { writeFileSync, mkdirSync } from 'node:fs';
import path from 'node:path';
import { ALL_SCENARIOS, NAV_SCENARIOS, VISION_SCENARIOS, runScenarios, scenarioReportMarkdown, ScenarioResult } from '../src/scenarios';
import { LOCALIZATION_METHODS } from '../src/mobile/navstack';

/**
 * Every demo scenario must pass. `SCENARIO_REPORT=1` (npm run scenarios) also writes docs/scenario-results.md,
 * which Read the Docs publishes.
 */
describe('demo scenarios', () => {
  const results: ScenarioResult[] = [];
  it('covers every localization method and every vision task family', () => {
    const covered = new Set(NAV_SCENARIOS.map((s) => s.method));
    for (const m of LOCALIZATION_METHODS) expect(covered.has(m.id), `no scenario for ${m.id}`).toBe(true);
    expect(VISION_SCENARIOS.length).toBeGreaterThanOrEqual(9);
    expect(new Set(ALL_SCENARIOS.map((s) => s.id)).size).toBe(ALL_SCENARIOS.length);
  });
  for (const s of ALL_SCENARIOS) {
    it(`${s.group}: ${s.title}`, async () => {
      const r = await s.run();
      results.push(r);
      const failed = r.metrics.filter((m) => m.ok === false).map((m) => `${m.name}=${m.value}${m.unit ?? ''} (${m.bound})`);
      expect(r.pass, `${s.id}: ${failed.join('; ')} ${r.notes.join('; ')}`).toBe(true);
    }, 120000);
  }
  it('writes the report when SCENARIO_REPORT is set', async () => {
    if (!process.env.SCENARIO_REPORT) return;
    const missing = ALL_SCENARIOS.filter((s) => !results.some((r) => r.id === s.id));
    const extra = await runScenarios((s) => missing.includes(s));
    const all = [...results, ...extra].sort((a, b) => ALL_SCENARIOS.findIndex((s) => s.id === a.id) - ALL_SCENARIOS.findIndex((s) => s.id === b.id));
    const md = scenarioReportMarkdown(all);
    const out = path.join(process.cwd(), 'docs', 'scenario-results.md');
    mkdirSync(path.dirname(out), { recursive: true });
    writeFileSync(out, md);
    expect(md).toMatch(/scenarios pass/);
  }, 600000);
});
