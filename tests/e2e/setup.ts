/** Global setup for the browser tests: serve the production build (dist/) on a free port with `vite preview`. */
import { spawn, ChildProcess } from 'node:child_process';
import { existsSync } from 'node:fs';
import path from 'node:path';

const PORT = Number(process.env.E2E_PORT ?? 4180);
let child: ChildProcess | null = null;

async function up(url: string, ms: number): Promise<boolean> {
  const t0 = Date.now();
  while (Date.now() - t0 < ms) { try { const r = await fetch(url); if (r.ok) return true; } catch { /* not yet */ } await new Promise((r) => setTimeout(r, 300)); }
  return false;
}

export default async function setup(): Promise<() => Promise<void>> {
  const root = path.resolve(__dirname, '../..');
  if (!existsSync(path.join(root, 'dist/index.html'))) throw new Error('dist/index.html not found — run `npm run build` before `npm run test:e2e`');
  const base = `http://127.0.0.1:${PORT}/`;
  if (!(await up(base, 500))) {
    child = spawn('npx', ['vite', 'preview', '--port', String(PORT), '--strictPort'], { cwd: root, stdio: 'ignore', detached: true });
    if (!(await up(base, 30000))) throw new Error(`preview server did not start on ${base}`);
  }
  process.env.E2E_BASE = base;
  return async () => { if (child?.pid) { try { process.kill(-child.pid, 'SIGTERM'); } catch { /* gone */ } } };
}
