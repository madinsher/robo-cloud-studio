#!/usr/bin/env node
/**
 * npm run scenarios: run the demo scenarios with SCENARIO_REPORT=1 (writes docs/scenario-results.md). The variable is
 * set here, not with the POSIX `VAR=1 cmd` form, so the script runs the same in cmd / PowerShell / bash.
 */
import { spawnSync } from 'node:child_process';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const vitest = path.join(root, 'node_modules', 'vitest', 'vitest.mjs');
const r = spawnSync(process.execPath, [vitest, 'run', 'tests/scenarios.test.ts', ...process.argv.slice(2)], { cwd: root, stdio: 'inherit', env: { ...process.env, SCENARIO_REPORT: '1' } });
process.exit(r.status ?? 1);
