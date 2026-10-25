/**
 * F1 (V1 critical): RoboDK Library stations driven by background station scripts (`while True: ... pause()` loops that
 * simulate conveyors, sensors and grippers) play in the browser with REAL Pyodide (no studio server, ?server=off),
 * opened through File › Open and played with the app's runtime like a user does. Before the fix the first endless
 * script blocked the page thread for good (06.c, 06.d laser, 06.e, 06.f delta froze the tab).
 * Checks: the page keeps answering and rendering, the scripts run concurrently with the programs (parts travel on the
 * conveyor, the robots pick them), the main program ends with RoboDK's result (all parts stored), and Stop ends the
 * background scripts. Needs network access to cdn.jsdelivr.net (Pyodide) and WebAssembly JSPI (Chrome 137+).
 */
import { describe, it, expect, beforeAll, afterAll, beforeEach } from 'vitest';
import { chromium, Browser, Page, BrowserContext } from 'playwright';
import { existsSync } from 'node:fs';

const BASE = process.env.E2E_BASE ?? 'http://127.0.0.1:4180/';
const CHROME = process.env.E2E_CHROME ?? ['C:/Program Files/Google/Chrome/Application/chrome.exe', '/usr/bin/google-chrome'].find((p) => existsSync(p));
const LIB = 'C:/RoboDK/Library';
const IGNORED = [/favicon/i, /WebGL/i, /GPU stall/i, /Context Lost/i, /ERR_CONNECTION_REFUSED/i, /WebSocket connection/i, /Failed to load resource/i];
let browser: Browser; let ctx: BrowserContext; let page: Page; let errors: string[] = [];
let online = false;
const ev = <T>(fn: (...a: any[]) => T, arg?: unknown): Promise<T> => page.evaluate(fn as any, arg) as Promise<T>;
const race = <T>(p: Promise<T>, ms: number): Promise<T> => Promise.race([p, new Promise<T>((_, rej) => setTimeout(() => rej(new Error(`page did not answer within ${ms} ms`)), ms))]);
const have = (f: string) => existsSync(`${LIB}/${f}`);

async function openLibrary(file: string): Promise<void> {
  await page.goto(`${BASE}?demo=none&server=off`, { waitUntil: 'load' });
  await page.waitForSelector('.menubar');
  await page.click('.menu-btn:has-text("File")');
  const [fc] = await Promise.all([page.waitForEvent('filechooser'), page.click('.ctx-item:has-text("Open / import")')]);
  await fc.setFiles([`${LIB}/${file}`]);
  await page.waitForFunction(() => (window as any).app.station.name !== 'New station', null, { timeout: 300000 });
  await page.waitForFunction(() => { const el = document.querySelector<HTMLElement>('.progress-pill'); return !el || el.style.display === 'none' || !/Opening|Importing/.test(el.textContent ?? ''); }, null, { timeout: 300000 });
  // the station holds Python programs: Pyodide starts loading on open
  await page.waitForFunction(() => (window as any).pyRunner.loaded() && !(window as any).pyRunner.loading(), null, { timeout: 180000 });
}

interface Sample { t: number; status: string; gap: number; parts: Record<string, number>; first: number[] | null; robots: number[][]; running: string[] }
/**
 * Play `main` at `speed` and sample the page every second (each sample must come back within 5 s: the tab is alive).
 * Stops when the run ends or after `maxS` seconds of wall time.
 */
async function playAndWatch(main: string, speed: number, maxS: number, partRe = '^Part'): Promise<Sample[]> {
  await ev((a: any) => {
    const w = window as any; const app = w.app;
    w.__gap = 0; let last = performance.now(); const f = () => { const n = performance.now(); w.__gap = Math.max(w.__gap, n - last); last = n; requestAnimationFrame(f); }; requestAnimationFrame(f);
    app.simSpeed = a.speed;
    const p = app.station.itemsOfType(8).find((x: any) => x.name === a.main);
    w.__h = app.runProgram(p);
  }, { main, speed });
  const out: Sample[] = [];
  const t0 = Date.now();
  while (Date.now() - t0 < maxS * 1000) {
    await new Promise((r) => setTimeout(r, 1000));
    const s = await race(ev((re: string) => {
      const w = window as any; const app = w.app; const g = w.__gap; w.__gap = 0;
      const parts = app.station.itemsOfType(5).filter((o: any) => new RegExp(re).test(o.name));
      const byParent: Record<string, number> = {};
      for (const o of parts) byParent[o.parent?.name ?? ''] = (byParent[o.parent?.name ?? ''] ?? 0) + 1;
      const p0 = parts[0] ? Array.from(parts[0].poseAbs() as Float64Array).slice(12, 15) : null;
      return { t: app.simTime, status: w.__h?.status, gap: g, parts: byParent, first: p0, robots: app.station.itemsOfType(2).map((r: any) => r.joints()), running: w.pyRunner.running() };
    }, partRe), 5000);
    out.push(s);
    if (s.status === 'done' || s.status === 'error' || s.status === 'stopped') break;
  }
  return out;
}

beforeAll(async () => {
  try { online = (await fetch('https://cdn.jsdelivr.net/pyodide/v0.26.4/full/pyodide.js', { method: 'HEAD' })).ok; } catch { online = false; }
  // real-time scripts need real frame rates: the GPU where there is one (software GL renders these stations at ~3 fps,
  // so a 1 ms sensor poll would see the conveyor jump 60 mm per frame and miss the part, in RoboDK too)
  const gl = process.platform === 'win32' && !process.env.E2E_SWIFTSHADER ? ['--use-angle=d3d11', '--enable-gpu', '--ignore-gpu-blocklist'] : ['--use-gl=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'];
  browser = await chromium.launch({ ...(CHROME ? { executablePath: CHROME } : {}), args: gl });
  ctx = await browser.newContext({ viewport: { width: 1280, height: 800 } });
  page = await ctx.newPage();
  page.on('pageerror', (e) => errors.push(`[pageerror] ${e.message}`));
  page.on('console', (m) => { if (m.type() === 'error') errors.push(`[console] ${m.text()}`); });
}, 120000);
afterAll(async () => { await browser?.close(); });
beforeEach(() => { errors = []; });
const noErrors = () => { const bad = errors.filter((e) => !IGNORED.some((re) => re.test(e))); expect(bad, bad.join('\n')).toEqual([]); };

describe('background station scripts in the browser (Pyodide, cooperative)', () => {
  it.skipIf(!have('Example-06.e-Conveyor with 2 UR robots.rdk'))('06.e StartSimulation: RunConveyor / SimulateToolOpen loop beside the robots, UR10 B takes both parts, Stop ends the loops', async (t) => {
    if (!online) t.skip();
    await openLibrary('Example-06.e-Conveyor with 2 UR robots.rdk');
    expect(await ev(() => (window as any).pyRunner.cooperative())).toBe(true);
    const s = await playAndWatch('StartSimulation', 5, 300);
    const last = s.at(-1)!;
    expect(last.status).toBe('done');
    expect(Math.max(...s.map((x) => x.gap))).toBeLessThan(1500);                   // the page kept rendering
    // the endless scripts ran beside the programs (RoboDK: they keep running after StartSimulation returns)
    expect(s.some((x) => x.running.includes('RunConveyor') && x.running.includes('SimulateToolOpen'))).toBe(true);
    expect(last.running).toEqual(expect.arrayContaining(['RunConveyor', 'SimulateToolOpen']));
    // a part travelled along the conveyor (+Y of ConveyorReference at 20 mm/s), UR10 B left its wait pose
    const ys = s.map((x) => x.first?.[1] ?? NaN).filter(Number.isFinite);
    expect(Math.max(...ys) - Math.min(...ys)).toBeGreaterThan(1000);
    const b0 = s[0].robots[1], bMoved = s.some((x) => x.robots[1].some((q, i) => Math.abs(q - b0[i]) > 20));
    expect(bMoved).toBe(true);
    // both parts were picked from the conveyor and stored on table B by the program's Detach (TableReference)
    expect(last.parts).toEqual({ TableReference: 2 });
    // Stop (RoboDK: stopping the simulation ends the scripts the programs started)
    await ev(() => (window as any).app.resetSimulation());
    await page.waitForFunction(() => (window as any).pyRunner.running().length === 0, null, { timeout: 20000 });
    const hist = await ev(() => (window as any).pyRunner.history.map((h: any) => ({ name: h.name, code: h.code, stopped: !!h.stopped })));
    expect(hist.filter((h: any) => h.stopped).map((h: any) => h.name).sort()).toEqual(['RunConveyor', 'SimulateToolOpen']);
    expect(hist.every((h: any) => h.code === 0)).toBe(true);
    noErrors();
  }, 600000);

  it.skipIf(!have('Example-06.d-Pick and place with laser sensor.rdk'))('06.d laser sensor MainSimulation: the sensor / robot-A threads run, all 18 parts end on PalletB', async (t) => {
    if (!online) t.skip();
    await openLibrary('Example-06.d-Pick and place with laser sensor.rdk');
    const s = await playAndWatch('MainSimulation', 10, 240);
    const last = s.at(-1)!;
    expect(last.status).toBe('done');
    expect(Math.max(...s.map((x) => x.gap))).toBeLessThan(1500);
    expect(last.parts).toEqual({ PalletB: 18 });
    const hist = await ev(() => (window as any).pyRunner.history.map((h: any) => `${h.name}:${h.code}`));
    expect(hist).toEqual(expect.arrayContaining(['PrepareSimulation:0', 'RobotA_GetParts:0', 'RobotB_StoreParts:0']));
    await ev(() => (window as any).app.resetSimulation());
    await page.waitForFunction(() => (window as any).pyRunner.running().length === 0, null, { timeout: 20000 });
    noErrors();
  }, 400000);

  it.skipIf(!have('Example-06.c-Pick and place with 2D camera.rdk'))('06.c 2D camera MainSimulation: all 12 parts end on PalletB', async (t) => {
    if (!online) t.skip();
    await openLibrary('Example-06.c-Pick and place with 2D camera.rdk');
    const s = await playAndWatch('MainSimulation', 10, 240);
    const last = s.at(-1)!;
    expect(last.status).toBe('done');
    expect(Math.max(...s.map((x) => x.gap))).toBeLessThan(1500);
    expect(last.parts).toEqual({ PalletB: 12 });
    await ev(() => (window as any).app.resetSimulation());
    await page.waitForFunction(() => (window as any).pyRunner.running().length === 0, null, { timeout: 20000 });
    noErrors();
  }, 400000);

  it.skipIf(!have('Example-06.f-Pick and Place with Delta robots.rdk'))('06.f delta MainProgram: the delta robot keeps picking from the running conveyor and the tab stays responsive', async (t) => {
    if (!online) t.skip();
    await openLibrary('Example-06.f-Pick and Place with Delta robots.rdk');
    const s = await playAndWatch('MainProgram', 2, 40);
    expect(s.length).toBeGreaterThan(20);                                            // every sample answered
    expect(Math.max(...s.map((x) => x.gap))).toBeLessThan(1500);
    const q = s.map((x) => x.robots[0][0]);
    expect(Math.max(...q) - Math.min(...q)).toBeGreaterThan(20);                      // the delta moves
    expect(s.at(-1)!.running).toEqual(expect.arrayContaining(['RunConveyor']));
    await ev(() => (window as any).app.resetSimulation());
    await page.waitForFunction(() => (window as any).pyRunner.running().length === 0, null, { timeout: 20000 });
    noErrors();
  }, 400000);
});
