/**
 * V1 finding: Library 08.a "Synchronize 3 Robots (Script matching timings)" failed in the browser with "RuntimeError:
 * can't start new thread" (Pyodide has no threads). With WebAssembly JSPI the threads of a station script are coroutines
 * of the interpreter (python/robodk/_threads_pyodide.py): MoveRobots starts one MoveRobot thread per robot, they
 * synchronise through a lock and a busy-wait on a shared counter, and the main script joins them.
 * Real Pyodide from jsDelivr, the production build, system Chrome (skips offline / without the Library).
 */
import { describe, it, expect, beforeAll, afterAll } from 'vitest';
import { chromium, Browser, Page } from 'playwright';
import { existsSync } from 'node:fs';

const BASE = process.env.E2E_BASE ?? 'http://127.0.0.1:4180/';
const CHROME = process.env.E2E_CHROME ?? ['C:/Program Files/Google/Chrome/Application/chrome.exe', '/usr/bin/google-chrome'].find((p) => existsSync(p));
const FILE = 'C:/RoboDK/Library/Example-08.a-Synchronize 3 Robots with a Screen (Script matching timings).rdk';
let browser: Browser; let page: Page; let online = false;
const errors: string[] = [];

beforeAll(async () => {
  try { online = (await fetch('https://cdn.jsdelivr.net/pyodide/v0.26.4/full/pyodide.js', { method: 'HEAD' })).ok; } catch { online = false; }
  browser = await chromium.launch({ ...(CHROME ? { executablePath: CHROME } : {}), args: ['--use-gl=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'] });
  page = await browser.newPage({ viewport: { width: 1280, height: 800 } });
  page.on('pageerror', (e) => errors.push(`[pageerror] ${e.message}`));
}, 120000);
afterAll(async () => { await browser?.close(); });

describe('station Python programs with threads in the browser (Pyodide + JSPI)', () => {
  it.skipIf(!existsSync(FILE))('08.a MoveRobots: three MoveRobot threads drive R1-R3 to all their targets and the script finishes', async (t) => {
    if (!online) t.skip();
    await page.goto(`${BASE}?demo=none&server=off`, { waitUntil: 'load' });
    await page.waitForSelector('.menubar');
    await page.click('.menu-btn:has-text("File")');
    const [fc] = await Promise.all([page.waitForEvent('filechooser'), page.click('.ctx-item:has-text("Open / import")')]);
    await fc.setFiles(FILE);
    await page.waitForFunction(() => (window as any).app.station.name !== 'New station', null, { timeout: 300000 });
    await page.waitForFunction(() => { const el = document.querySelector<HTMLElement>('.progress-pill'); return !el || el.style.display === 'none'; }, null, { timeout: 300000 });
    const r = await page.evaluate(async () => {
      const app = (window as any).app;
      app.simSpeed = 5;
      const p = [...app.station.walk()].find((i: any) => i.name === 'MoveRobots' && i.type === 10);
      const robots = [...app.station.walk()].filter((i: any) => i.dof === 6);
      const q0 = new Map(robots.map((x: any) => [x.name, x.joints()]));
      const h = app.runProgram(p);
      const t0 = performance.now();
      while (['queued', 'running', 'paused', 'blocked'].includes(h.status) && performance.now() - t0 < 280000) await new Promise((res) => setTimeout(res, 100));
      const runner = (window as any).pyRunner;
      return {
        status: h.status, error: h.error?.message, cooperative: runner.cooperative?.(),
        out: runner.history.filter((x: any) => x.name === 'MoveRobots').map((x: any) => x.output).join('\n'),
        moved: robots.map((x: any) => Math.max(...x.joints().map((v: number, k: number) => Math.abs(v - (q0.get(x.name) as number[])[k])))),
      };
    });
    if (!r.cooperative) t.skip();   // no JSPI in this browser: the runner refuses threads with a clear message
    expect(r.status, `${r.error ?? ''}\n${r.out.slice(-800)}`).toBe('done');
    expect(r.out).not.toMatch(/can't start new thread/);
    for (const n of ['R1', 'R2', 'R3']) expect(r.out).toContain(`Robot ${n} finished`);
    expect(r.out).toContain('Main program finished');
    expect(r.moved.every((m: number) => m > 10)).toBe(true);
    expect(errors).toEqual([]);
  }, 600000);
});
