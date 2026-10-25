/**
 * R2: RoboDK station Python programs run in the browser with REAL Pyodide (installPyodideRunner in main.ts, no studio
 * server: ?server=off), on RoboDK Library stations opened through File › Open. The programs call the scripts like
 * RoboDK does (Gripper(1), SprayOn(1), ...), or the script item runs on its own (double-click in RoboDK = the tree's
 * "Run Python program"). Effects are checked against what the scripts do in RoboDK (same assertions as the headless
 * CPython runs of tests/api_station_scripts.test.ts).
 * Pyodide (~10 MB) is downloaded from cdn.jsdelivr.net on the first call: the suite skips without network access.
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
const noErrors = () => { const bad = errors.filter((e) => !IGNORED.some((re) => re.test(e))); expect(bad, bad.join('\n')).toEqual([]); };

async function openLibrary(file: string): Promise<void> {
  await page.goto(`${BASE}?demo=none&server=off`, { waitUntil: 'load' });
  await page.waitForSelector('.menubar');
  await page.click('.menu-btn:has-text("File")');
  const [fc] = await Promise.all([page.waitForEvent('filechooser'), page.click('.ctx-item:has-text("Open / import")')]);
  await fc.setFiles([`${LIB}/${file}`]);
  await page.waitForFunction(() => (window as any).app.station.name !== 'New station', null, { timeout: 300000 });
  await page.waitForFunction(() => { const el = document.querySelector<HTMLElement>('.progress-pill'); return !el || el.style.display === 'none' || !/Opening|Importing/.test(el.textContent ?? ''); }, null, { timeout: 300000 });
}
/** Play a program (or a Python item) with the real UI clock at `speed`× until it ends; returns status + script history. */
async function play(name: string, speed: number, timeoutS = 240): Promise<{ status: string; error?: string; history: Array<{ name: string; args: string[]; code: number; output: string }>; loaded: boolean }> {
  return ev(async (a: any) => {
    const app = (window as any).app;
    app.simSpeed = a.speed;
    const p = [...app.station.walk()].find((i: any) => i.name === a.name && (typeof i.instructions === 'function' || i.type === 10));
    if (!p) return { status: 'missing', history: [], loaded: false };
    const h = app.runProgram(p);
    const t0 = performance.now();
    while (['queued', 'running', 'paused', 'blocked'].includes(h.status) && performance.now() - t0 < a.timeoutS * 1000) await new Promise((r) => setTimeout(r, 100));
    const runner = (window as any).pyRunner;
    return { status: h.status, error: h.error?.message, history: runner.history.map((x: any) => ({ name: x.name, args: x.args, code: x.code, output: x.output })), loaded: runner.loaded() };
  }, { name, speed, timeoutS });
}

beforeAll(async () => {
  try { online = (await fetch('https://cdn.jsdelivr.net/pyodide/v0.26.4/full/pyodide.js', { method: 'HEAD' })).ok; } catch { online = false; }
  browser = await chromium.launch({ ...(CHROME ? { executablePath: CHROME } : {}), args: ['--use-gl=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'] });
  ctx = await browser.newContext({ viewport: { width: 1280, height: 800 } });
  page = await ctx.newPage();
  page.on('pageerror', (e) => errors.push(`[pageerror] ${e.message}`));
  page.on('console', (m) => { if (m.type() === 'error') errors.push(`[console] ${m.text()}`); });
}, 120000);
afterAll(async () => { await browser?.close(); });
beforeEach(() => { errors = []; });

const have = (f: string) => existsSync(`${LIB}/${f}`);
describe('station Python programs in the browser (Pyodide)', () => {
  it.skipIf(!have('Example-06.a-Pick and place - Mecademic.rdk'))('06.a ProgramTest drives the gripper through Gripper(1)/Gripper(0)', async (t) => {
    if (!online) t.skip();
    await openLibrary('Example-06.a-Pick and place - Mecademic.rdk');
    const r = await play('ProgramTest', 20);
    expect(r.status, r.error).toBe('done');
    expect(r.loaded).toBe(true);
    const g = r.history.filter((x) => x.name === 'Gripper');
    expect(g.length).toBeGreaterThanOrEqual(2);
    expect(g.every((x) => x.code === 0), g.map((x) => x.output).join('\n')).toBe(true);
    expect(new Set(g.map((x) => x.args.join(',')))).toEqual(new Set(['1', '0']));
    const last = Number(g.at(-1)!.args[0]) * 6;
    const st = await ev(() => { const app = (window as any).app; const y = (n: string) => app.station.find(n).pose()[13]; return { param: app.station.getParam('Gripper'), a: y('FingerA'), b: y('FingerB') }; });
    expect(Number(st.param)).toBe(last);
    expect(st.b).toBeCloseTo(last, 6); expect(st.a).toBeCloseTo(-last, 6);
    noErrors();
  }, 400000);

  it.skipIf(!have('Example-01.a-Robot Painting - Universal Robots.rdk'))('01.a MainProg switches the spray gun with SprayOn(1) / SprayOn(0) and paints the part', async (t) => {
    if (!online) t.skip();
    await openLibrary('Example-01.a-Robot Painting - Universal Robots.rdk');
    await ev(() => { (window as any).__gunOn = 0; setInterval(() => { if ([...(window as any).RDK.spray.guns.values()].some((g: any) => g.on)) (window as any).__gunOn++; }, 50); });
    const r = await play('MainProg', 4);
    expect(r.status, r.error).toBe('done');
    const s = r.history.filter((x) => x.name === 'SprayOn');
    expect(s.map((x) => x.args)).toEqual([['1'], ['0']]);
    expect(s.every((x) => x.code === 0)).toBe(true);
    const st = await ev(() => { const R = (window as any).RDK; return { onSamples: (window as any).__gunOn, offNow: [...R.spray.guns.values()].every((g: any) => !g.on), rows: R.Spray_GetStats()[1] }; });
    expect(st.onSamples).toBeGreaterThan(5);
    expect(st.offNow).toBe(true);
    // material deposited on the part while the gun was on (Material used %, as RoboDK's Spray_GetStats)
    expect(Math.max(...st.rows.map((row: number[]) => row[1]))).toBeGreaterThan(5);
    noErrors();
  }, 400000);

  it.skipIf(!have('Example-07.a-Weld Script Hexagon.rdk'))('07.a the DoWeld script item welds the 200 mm hexagon around Target 1 and returns Home', async (t) => {
    if (!online) t.skip();
    await openLibrary('Example-07.a-Weld Script Hexagon.rdk');
    await ev(() => {
      const app = (window as any).app;
      const robot = app.station.itemsOfType(2)[0];
      (window as any).__tcp = [];
      app.station.events.on('changed', ({ item, what }: any) => { if (item === robot && what === 'joints') { const m = robot.poseTCPAbs(); (window as any).__tcp.push([m[12], m[13], m[14]]); } });
    });
    const r = await play('DoWeld', 20);
    expect(r.status, r.error).toBe('done');
    const run = r.history.find((x) => x.name === 'DoWeld');
    expect(run?.code, run?.output).toBe(0);
    const res = await ev(() => {
      const app = (window as any).app;
      const t1 = app.station.find('Target 1', 6), home = app.station.find('Home', 6), robot = app.station.itemsOfType(2)[0];
      const ref = t1.parent.poseAbs(), tp = t1.pose();
      const mul = (a: number[], b: number[]) => { const o = new Array(16).fill(0); for (let c = 0; c < 4; c++) for (let r = 0; r < 4; r++) for (let k = 0; k < 4; k++) o[c * 4 + r] += a[k * 4 + r] * b[c * 4 + k]; return o; };
      const verts = [0, 1, 2, 3, 4, 5].map((i) => { const a = (i * Math.PI) / 3, c = Math.cos(a), s = Math.sin(a); const rz = [c, s, 0, 0, -s, c, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]; const tx = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 200, 0, 0, 1]; const m = mul(Array.from(ref), mul(Array.from(tp), mul(rz, tx))); return [m[12], m[13], m[14]]; });
      const pts: number[][] = (window as any).__tcp;
      const dist = verts.map((v) => Math.min(...pts.map((p) => Math.hypot(p[0] - v[0], p[1] - v[1], p[2] - v[2]))));
      return { n: pts.length, dist, q: robot.joints(), qh: home.joints ?? robot.jointsForTarget(home) };
    });
    expect(res.n).toBeGreaterThan(50);
    res.dist.forEach((d: number, i: number) => expect(d, `vertex ${i}`).toBeLessThan(1));
    res.q.forEach((q: number, i: number) => expect(q).toBeCloseTo(res.qh[i], 3));
    noErrors();
  }, 400000);
});
