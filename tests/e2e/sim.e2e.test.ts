/**
 * T11 browser end-to-end tests of the simulation UX against the production build, on the system Chrome
 * (vitest.e2e.config.ts / tests/e2e/setup.ts). Real UI paths: toolbar and Simulation-tab buttons, tree clicks during
 * playback, the Ctrl+O file chooser, the Ctrl+S download, a fresh page reopening the saved file. RoboDK Library
 * stations are checked against the RoboDK 5.9.4 ground truth in tests/fixtures/robodk_gt.
 */
import { describe, it, expect, beforeAll, afterAll, beforeEach } from 'vitest';
import { chromium, Browser, Page, BrowserContext } from 'playwright';
import { existsSync, readFileSync, mkdtempSync } from 'node:fs';
import { gunzipSync } from 'node:zlib';
import { join } from 'node:path';
import { tmpdir } from 'node:os';

const BASE = process.env.E2E_BASE ?? 'http://127.0.0.1:4180/';
const CHROME = process.env.E2E_CHROME ?? ['C:/Program Files/Google/Chrome/Application/chrome.exe', '/usr/bin/google-chrome'].find((p) => existsSync(p));
const LIB = 'C:/RoboDK/Library';
const GT = join(__dirname, '../fixtures/robodk_gt');
let browser: Browser; let ctx: BrowserContext; let page: Page; let errors: string[] = []; let crashed = false;
const IGNORED = [/favicon/i, /WebGL/i, /GPU stall/i, /Context Lost/i, /ERR_CONNECTION_REFUSED/i, /WebSocket connection/i, /Failed to load resource/i];
const ev = <T>(fn: (...a: any[]) => T, arg?: unknown): Promise<T> => page.evaluate(fn as any, arg) as Promise<T>;
const noErrors = () => { const bad = errors.filter((e) => !IGNORED.some((re) => re.test(e))); expect(bad, bad.join('\n')).toEqual([]); expect(crashed, 'page crashed').toBe(false); };

async function newPage(): Promise<Page> {
  const p = await ctx.newPage();
  p.on('pageerror', (e) => errors.push(`[pageerror] ${e.message}`));
  p.on('console', (m) => { if (m.type() === 'error') errors.push(`[console] ${m.text()}`); });
  p.on('crash', () => { crashed = true; });
  return p;
}
const load = async (q: string) => { await page.goto(`${BASE}?${q}&server=off`, { waitUntil: 'load' }); await page.waitForSelector('.menubar'); await page.waitForTimeout(400); };
const tab = async (label: string) => { await page.click(`.tab-bar .tab:has-text("${label}")`); await page.waitForTimeout(150); };
const simBtn = (k: string) => page.click(`.sim-panel button[data-sim="${k}"]`);
/** Open files through the real File › Open dialog (file chooser) and wait until the import finished. */
async function openFiles(paths: string[], waitName = true): Promise<void> {
  const before = await ev(() => (window as any).app.station.name);
  await page.click('.menu-btn:has-text("File")');
  const [fc] = await Promise.all([page.waitForEvent('filechooser'), page.click('.ctx-item:has-text("Open / import")')]);
  await fc.setFiles(paths);
  if (waitName) await page.waitForFunction((b) => (window as any).app.station.name !== b, before, { timeout: 300000 });
  // the import is done when no "Opening…" job is shown (a background preview compile of a long program may continue)
  await page.waitForFunction(() => { const el = document.querySelector<HTMLElement>('.progress-pill'); return !el || el.style.display === 'none' || !/Opening|Importing/.test(el.textContent ?? ''); }, null, { timeout: 300000 });
  await page.waitForTimeout(800);
}
const meshStats = () => ev(() => { let meshes = 0, missing = 0; (window as any).app.renderer.scene.traverse((o: any) => { if (o.isMesh && !o.userData.missingMesh) meshes++; if (o.userData.missingMesh) missing++; }); return { meshes, missing }; });
const gt = (file: string) => { const p = join(GT, file.replace(/ /g, '_') + '_.json.gz'); return existsSync(p) ? JSON.parse(gunzipSync(readFileSync(p)).toString()) : null; };
const rows16 = (m: number[]) => [m[3], m[7], m[11]]; // RoboDK row-major 4x4 -> translation

beforeAll(async () => {
  browser = await chromium.launch({ ...(CHROME ? { executablePath: CHROME } : {}), args: ['--use-gl=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'] });
  ctx = await browser.newContext({ viewport: { width: 1440, height: 900 }, acceptDownloads: true });
  page = await newPage();
});
afterAll(async () => { await browser?.close(); });
beforeEach(() => { errors = []; crashed = false; });

describe('simulation UX in the browser', () => {
  it('?demo=none is an empty station', async () => {
    await load('demo=none');
    expect(await ev(() => (window as any).app.station.children.length)).toBe(0);
    expect(await page.locator('.tree-row').count()).toBe(1);
    noErrors();
  });

  it('Run → End → Run → End repeats exactly; Pause holds the clock and Play resumes without restarting', async () => {
    await load('demo=pickplace');
    await tab('Simulation');
    const boxes = () => ev(() => ['Box 1', 'Box 2', 'Box 3'].map((n) => Array.from((window as any).app.station.find(n).poseAbs() as number[]).slice(12, 15)));
    const start = await boxes();
    await simBtn('run'); await page.waitForTimeout(1500);
    await simBtn('pause');
    const t1 = await ev(() => (window as any).app.simTime);
    expect(t1).toBeGreaterThan(0.5);
    await page.waitForTimeout(800);
    expect(await ev(() => (window as any).app.simTime)).toBe(t1);
    await page.click('.toolbar button[data-sim="play"]'); await page.waitForTimeout(600);
    const after = await ev(() => ({ t: (window as any).app.simTime, runs: (window as any).app.runs.length }));
    expect(after.t).toBeGreaterThan(t1 + 0.2); expect(after.runs).toBe(1);
    await simBtn('end');
    const first = await boxes();
    first.forEach((p, i) => expect(Math.hypot(p[0] - start[i][0], p[1] - start[i][1])).toBeGreaterThan(100));
    await simBtn('run'); await page.waitForTimeout(300); await simBtn('end');
    const second = await boxes();
    second.forEach((p, i) => expect(Math.hypot(p[0] - first[i][0], p[1] - first[i][1], p[2] - first[i][2])).toBeLessThan(1));
    await simBtn('reset');
    (await boxes()).forEach((p, i) => expect(Math.hypot(p[0] - start[i][0], p[1] - start[i][1], p[2] - start[i][2])).toBeLessThan(1e-6));
    noErrors();
  });

  it('tree rows stay clickable while two robots play (10 of 10 real clicks), and both robots move', async () => {
    await load('demo=welding');
    const q0 = await ev(() => { const app = (window as any).app; const ps = [...app.station.walk()].filter((i: any) => typeof i.instructions === 'function' && i.robot()); ps.slice(0, 2).forEach((p: any) => app.runProgram(p)); return ps.slice(0, 2).map((p: any) => p.robot().joints()); });
    expect(q0.length).toBe(2);
    const names: string[] = await ev(() => [...document.querySelectorAll('.tree-row .tree-name')].slice(1, 11).map((e) => e.childNodes[0].textContent!));
    let hits = 0;
    for (const n of names) {
      await page.locator('.tree-row', { has: page.locator(`.tree-name:text-is("${n}")`) }).first().click({ timeout: 5000 });
      if ((await ev(() => (window as any).app.station.selection[0]?.name)) === n) hits++;
    }
    expect(hits).toBe(names.length);
    const q1 = await ev(() => { const app = (window as any).app; return app.runs.slice(0, 2).map((r: any) => r.robot.joints()); });
    q1.forEach((q: number[], i: number) => expect(q).not.toEqual(q0[i]));
    expect(await ev(() => (window as any).app.isPlaying())).toBe(true);
    noErrors();
  });

  const PP2 = `${LIB}/Example-06.b-Pick and place 2 tables.rdk`;
  it.skipIf(!existsSync(PP2))('an imported RoboDK station saved with Ctrl+S reopens in a fresh page with every mesh', async () => {
    await load('demo=none');
    await openFiles([PP2]);
    const before = await meshStats();
    expect(before.missing).toBe(0); expect(before.meshes).toBeGreaterThan(10);
    const items = await ev(() => [...(window as any).app.station.walk()].length);
    const [dl] = await Promise.all([page.waitForEvent('download'), page.keyboard.press('Control+s')]);
    const file = join(mkdtempSync(join(tmpdir(), 'vbs-')), dl.suggestedFilename());
    await dl.saveAs(file);
    const fresh = await newPage(); const old = page; page = fresh;
    try {
      await load('demo=none');
      await openFiles([file]);
      const after = await meshStats();
      expect(after.missing).toBe(0);
      expect(after.meshes).toBe(before.meshes);
      expect(await ev(() => [...(window as any).app.station.walk()].length)).toBe(items);
    } finally { await fresh.close(); page = old; }
    noErrors();
  }, 300000);

  for (const name of ['Example-02.b-Robot Machining Mold - Motoman.rdk', 'Example-05.c-Robot machining with a rail and turntable - ABB.rdk']) {
    it.skipIf(!existsSync(`${LIB}/${name}`))(`large station opens without freezing or crashing and its program plays: ${name}`, async () => {
      await load('demo=none');
      await ev(() => { (window as any).__gap = 0; let last = performance.now(); const loop = () => { const n = performance.now(); (window as any).__gap = Math.max((window as any).__gap, n - last); last = n; requestAnimationFrame(loop); }; requestAnimationFrame(loop); });
      await openFiles([`${LIB}/${name}`]);
      const gap = await ev(() => (window as any).__gap);
      expect(gap, 'longest frame gap while importing (ms)').toBeLessThan(5000);
      const prog = await ev(() => { const app = (window as any).app; const p = [...app.station.walk()].filter((i: any) => typeof i.instructions === 'function').sort((a: any, b: any) => b.instructions().length - a.instructions().length)[0]; app.select(p); return { name: p.name, n: p.instructions().length, q: app.activeRobot?.joints() }; });
      expect(prog.n).toBeGreaterThan(1000);
      expect(await page.locator('.ins-row').count()).toBeLessThan(400); // virtualised
      await tab('Simulation'); await simBtn('run'); await page.waitForTimeout(3000);
      const st = await ev(() => { const app = (window as any).app; return { t: app.simTime, status: app.runs.at(-1)?.status, q: app.activeRobot?.joints() }; });
      expect(st.status).toBe('running'); expect(st.t).toBeGreaterThan(1); expect(st.q).not.toEqual(prog.q);
      await simBtn('stop');
      noErrors();
    }, 300000);
  }

  // RoboDK Library stations: main program run to its end; cycle time, final joints and final object poses vs RoboDK
  const LIBRARY: Array<{ file: string; main: string; then?: string }> = [
    { file: 'Example-01.b-Robot Drilling - KUKA.rdk', main: 'DrillingPart' },
    { file: 'Example-01.a-Robot Painting - Universal Robots.rdk', main: 'MainProg' },
    { file: 'Example-06.b-Pick and place 2 tables.rdk', main: 'Pick and place', then: 'Replace objects' },
  ];
  for (const s of LIBRARY) {
    const truth = gt(s.file);
    it.skipIf(!existsSync(`${LIB}/${s.file}`) || !truth)(`RoboDK ${s.file}: "${s.main}" runs to its end like RoboDK`, async () => {
      const prog = truth.programs.find((p: any) => p.name === s.main);
      await load('demo=none');
      await openFiles([`${LIB}/${s.file}`]);
      const res = await ev((a) => {
        const app = (window as any).app;
        const progs = [...app.station.walk()].filter((i: any) => typeof i.instructions === 'function' || i.type === 10);
        const p = progs.find((x: any) => x.name === a.main);
        if (!p) return { error: `program ${a.main} not found among ${progs.map((x: any) => x.name).join(', ')}` };
        app.select(p);
        const h = app.runProgram(p); app.runToEnd(3 * a.cycle + 60);
        const out: any = { status: h.status, error: h.error?.message, t: typeof h.endTime === 'number' ? h.endTime - (h.startTime ?? 0) : app.simTime, joints: (h.robot ?? app.activeRobot)?.joints?.() ?? null };
        if (a.then) { const q = progs.find((x: any) => x.name === a.then); if (q) { const h2 = app.runProgram(q); app.runToEnd(60); out.thenStatus = h2.status; } }
        out.objects = Object.fromEntries([...app.station.walk()].filter((i: any) => i.type === 5).map((o: any) => [o.name, Array.from(o.poseAbs()).slice(12, 15)]));
        return out;
      }, { main: s.main, then: s.then, cycle: prog.update[1] });
      expect(res.error, JSON.stringify(res)).toBeUndefined();
      expect(res.status).toBe('done');
      // objects: either never moved, or (06.b) restored by "Replace objects" — final poses equal RoboDK's station poses
      for (const o of truth.objects) {
        const p = res.objects[o.name];
        expect(p, `object ${o.name}`).toBeTruthy();
        const g = rows16(o.poseAbs);
        expect(Math.hypot(p[0] - g[0], p[1] - g[1], p[2] - g[2]), `object ${o.name}`).toBeLessThan(1);
      }
      const last = prog.jl?.at(-1);
      if (last && res.joints) res.joints.forEach((q: number, i: number) => expect(Math.abs(q - last[i]), `final J${i + 1}: ${res.joints.map((x: number) => x.toFixed(2))} vs RoboDK ${last.slice(0, res.joints.length)}`).toBeLessThan(1));
      expect(Math.abs(res.t - prog.update[1]) / prog.update[1], `cycle ${res.t} s vs RoboDK ${prog.update[1]} s`).toBeLessThan(0.1);
      noErrors();
    }, 300000);
  }
});
