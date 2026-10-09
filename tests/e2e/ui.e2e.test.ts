/**
 * Browser end-to-end tests of the studio UI against the production build (see vitest.e2e.config.ts / tests/e2e/setup.ts).
 * Every test drives the real page with Playwright: menus, dialogs, tabs, demos, programs, the world clock, the Control
 * and Group tabs, the demo scenarios, the language switch. A page error or a console error anywhere fails the test.
 */
import { describe, it, expect, beforeAll, afterAll, beforeEach } from 'vitest';
import { chromium, Browser, Page } from 'playwright';
import { existsSync, readdirSync } from 'node:fs';
import { join } from 'node:path';

const BASE = process.env.E2E_BASE ?? 'http://127.0.0.1:4180/';
/** The browser: E2E_CHROME, else the Chromium Playwright resolves, else any Chromium under PLAYWRIGHT_BROWSERS_PATH (a pinned image may carry another build than the installed playwright expects). */
const CHROME = (() => {
  if (process.env.E2E_CHROME) return process.env.E2E_CHROME;
  try { const p = chromium.executablePath(); if (existsSync(p)) return p; } catch { /* resolved below */ }
  const root = process.env.PLAYWRIGHT_BROWSERS_PATH; if (!root || !existsSync(root)) return undefined;
  for (const d of readdirSync(root).filter((x) => x.startsWith('chromium-')).sort().reverse()) for (const sub of ['chrome-linux64/chrome', 'chrome-linux/chrome']) { const p = join(root, d, sub); if (existsSync(p)) return p; }
  return undefined;
})();
let browser: Browser; let page: Page; let errors: string[] = [];
const IGNORED = [/favicon/i, /WebGL/i, /GPU stall/i, /THREE\.WebGLRenderer: Context Lost/i, /ERR_CONNECTION_REFUSED/i, /WebSocket connection/i, /Failed to load resource/i];
const ev = <T>(fn: (...a: any[]) => T, arg?: unknown): Promise<T> => page.evaluate(fn as any, arg) as Promise<T>;
const load = async (q = 'demo=pickplace') => { await page.goto(`${BASE}?${q}&server=off`, { waitUntil: 'load' }); await page.waitForSelector('.menubar'); await page.waitForTimeout(600); };
const openMenu = async (top: string) => { await ev(() => document.querySelectorAll('.ctx-menu').forEach((m) => m.remove())); await page.click(`.menu-btn:has-text("${top}")`); await page.waitForSelector('.ctx-menu'); };
const menu = async (top: string, item: string) => {
  await openMenu(top);
  const ok = await ev((txt) => { const el = [...document.querySelectorAll('.ctx-item')].find((e) => e.textContent!.includes(txt)); if (el) { (el as HTMLElement).click(); return true; } return false; }, item);
  if (!ok) { await page.keyboard.press('Escape'); throw new Error(`menu item not found: ${top} › ${item}`); }
  await page.waitForTimeout(300);
};
const closeDialogs = async () => { for (let i = 0; i < 3; i++) { await ev(() => { document.querySelectorAll<HTMLElement>('.dialog .dialog-actions .btn:not(.primary)').forEach((b) => b.click()); document.querySelectorAll<HTMLElement>('.dialog .dialog-title .btn-icon').forEach((b) => b.click()); }); await page.keyboard.press('Escape'); await page.waitForTimeout(80); if (!(await ev(() => !!document.querySelector('.dialog, .overlay')))) return; } await ev(() => document.querySelectorAll('.overlay').forEach((o) => o.remove())); };
const tab = async (label: string) => { await page.click(`.tab-bar .tab:has-text("${label}")`); await page.waitForTimeout(200); };
const noErrors = () => { const bad = errors.filter((e) => !IGNORED.some((re) => re.test(e))); expect(bad, bad.join('\n')).toEqual([]); };

beforeAll(async () => {
  browser = await chromium.launch({ ...(CHROME ? { executablePath: CHROME } : {}), args: ['--use-gl=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'] });
  page = await browser.newPage({ viewport: { width: 1440, height: 900 } });
  page.on('pageerror', (e) => errors.push(`[pageerror] ${e.message}`));
  page.on('console', (m) => { if (m.type() === 'error') errors.push(`[console] ${m.text()}`); });
});
afterAll(async () => { await browser?.close(); });
beforeEach(() => { errors = []; });

describe('studio in the browser', () => {
  it('every demo station loads, its program runs to the end without problems and the world clock advances', async () => {
    await load('demo=none');
    const demos: string[] = await ev(() => ((window as any).app && document.querySelectorAll('.menu-btn').length ? ['pickplace', 'tutorial', 'welding', 'packing', 'orchard', 'greenhouse', 'verticalbot'] : []));
    expect(demos.length).toBe(7);
    for (const demo of demos) {
      await load(`demo=${demo}`);
      const info = await ev(() => {
        const app = (window as any).app; const out: any = { station: app.station.name, items: [...app.station.walk()].length, program: app.activeProgram?.name ?? null };
        if (app.activeProgram) { app.runProgram(); app.sim.runToEnd(); out.duration = app.sim.duration; out.problems = app.sim.result.problems.map((p: any) => p.message); }
        app.simSpeed = 20; app.startWorld(); return out;
      });
      expect(info.items, demo).toBeGreaterThan(1);
      if (info.program) { expect(info.duration, `${demo} program duration`).toBeGreaterThan(0); expect(info.problems, `${demo} problems`).toEqual([]); }
      await page.waitForTimeout(1200);
      const world = await ev(() => { const app = (window as any).app; const t = app.worldTime; app.pauseWorld(); return t; });
      expect(world, `${demo} world time`).toBeGreaterThan(0);
      noErrors();
    }
  });

  it('every menu entry opens without errors; dialogs open and close', async () => {
    await load('demo=pickplace');
    const tops = await ev(() => [...document.querySelectorAll('.menu-btn')].map((b) => b.textContent!.trim()));
    expect(tops).toEqual(['File', 'Edit', 'Add', 'Program', 'Robot', 'Mobile & Fleet', 'Agriculture', 'Control', 'Group', 'Tools', 'Connect', 'View', 'Help']);
    const skip = [/^Language/, /compatibility notes/, /New station tab/, /Close station tab/, /^New station$/, /Demo stations/, /^Demo:/, /Delete selection/, /Export screenshot/]; // demo stations are covered by the first test (the heavy orchard scene makes software GL too slow for menu clicks)
    let clicked = 0; const opened: string[] = [];
    for (const top of tops) {
      await openMenu(top);
      const labels = await ev(() => [...document.querySelectorAll('.ctx-item')].map((e) => e.textContent!.replace(/\s*(Ctrl\+\w|Del|▸|F\d+)\s*$/, '').trim()));
      await page.keyboard.press('Escape');
      expect(labels.length, top).toBeGreaterThan(0);
      for (const label of labels) {
        if (skip.some((re) => re.test(label))) continue;
        await menu(top, label); clicked++;
        await page.waitForTimeout(250);
        const hasDialog = await ev(() => !!document.querySelector('.dialog'));
        if (hasDialog) { opened.push(label); await closeDialogs(); }
        expect(await ev(() => !!document.querySelector('.overlay')), `overlay left open after ${top} › ${label}`).toBe(false);
        await ev(() => { const app = (window as any).app; app.pauseWorld(); app.stopProgram(); });
      }
    }
    expect(clicked).toBeGreaterThan(80);
    expect(opened.length).toBeGreaterThanOrEqual(20);
    noErrors();
  });

  it('bottom tabs render and the RoboDK-API console executes scripts', async () => {
    await load('demo=pickplace');
    for (const label of ['Program', 'Simulation', 'Fleet', 'Process', 'Navigation', 'Vision', 'Control', 'Camera', 'Console (RoboDK API)', 'Log']) {
      await tab(label);
      const visible = await ev((l) => { const t = [...document.querySelectorAll('.tab-bar .tab')].find((x) => x.textContent!.includes(l)); return t?.classList.contains('active') && [...document.querySelectorAll<HTMLElement>('.tab-content > *')].some((el) => el.style.display !== 'none' && el.offsetHeight > 0); }, label);
      expect(visible, label).toBe(true);
    }
    await tab('Console (RoboDK API)');
    await page.fill('.console-in', 'const r = RDK.Item("", ITEM_TYPE_ROBOT); const p = RDK.AddProgram("E2E", r); p.MoveJ(r.Joints()); print(p.Update()); RDK.ItemList(ITEM_TYPE_ROBOT, true)');
    await page.click('text=Run (Ctrl+Enter)');
    await page.waitForTimeout(400);
    const out = (await page.textContent('.console-out'))!.replace(/\s+/g, ' ');
    expect(out).toMatch(/UR10e|\[/);
    expect(await ev(() => (window as any).app.station.find('E2E') !== null)).toBe(true);
    noErrors();
  });

  it('add a robot from the library, teach targets, run and export with every post processor; undo / redo; save → reload round trip', async () => {
    await load('demo=none');
    await menu('Add', 'Robot from library');
    await page.selectOption('.dialog select', 'KUKA_KR6_R900');
    await page.click('.dialog .btn.primary');
    await page.waitForTimeout(400);
    expect(await ev(() => (window as any).app.activeRobot?.name ?? null)).toMatch(/KR ?6/);
    await menu('Program', 'New program');
    await ev(() => { const app = (window as any).app; app.activeRobot.setJoints([20, -80, 100, 15, 60, 10]); }); // a non-singular start (the library home of a 6R arm has an aligned wrist)
    await menu('Program', 'Teach MoveJ');
    await ev(() => { const app = (window as any).app; const r = app.activeRobot; r.setJoints(r.joints().map((j: number, i: number) => j + (i === 0 ? 30 : 0))); });
    await menu('Program', 'Teach MoveL');
    const run = await ev(() => { const app = (window as any).app; app.runProgram(); app.sim.runToEnd(); return { n: app.activeProgram.instructions().length, dur: app.sim.duration, problems: app.sim.result.problems.map((p: any) => p.message) }; });
    expect(run.n).toBe(2); expect(run.dur).toBeGreaterThan(0); expect(run.problems).toEqual([]);
    const posts = await ev(() => { const app = (window as any).app; return app.posts().map((p: any) => { const files = app.exportProgram(p.id); return { id: p.id, files: files.length, bytes: files.reduce((s: number, f: any) => s + String(f.content).length, 0) }; }); });
    expect(posts.length).toBeGreaterThan(10);
    for (const p of posts) { expect(p.files, p.id).toBeGreaterThan(0); expect(p.bytes, p.id).toBeGreaterThan(20); }
    // undo the last teach through the Edit menu, redo it
    await menu('Edit', 'Undo');
    expect(await ev(() => (window as any).app.activeProgram?.instructions().length ?? 0)).toBe(1);
    await menu('Edit', 'Redo');
    expect(await ev(() => (window as any).app.activeProgram?.instructions().length ?? 0)).toBe(2);
    // save → reload
    const rt = await ev(() => { const app = (window as any).app; const json = app.saveToJSON(); const before = [...app.station.walk()].length; app.newStation(); const f = new File([json], 'e2e.vbstation'); return app.openFiles([f]).then(() => ({ before, after: [...app.station.walk()].length, name: app.station.name })); });
    expect(rt.after).toBe(rt.before);
    noErrors();
  });

  it('Control tab: course examples load, every model analyses, the diagram editors open; Group tab: examples analyse and a model runs on the fleet', async () => {
    await load('demo=none');
    await menu('Control', 'Course examples');
    await page.selectOption('.dialog select', '*');
    await page.click('.dialog .btn.primary');
    await page.waitForTimeout(800);
    const models = await ev(() => (window as any).app.station.itemsOfType(111).length);
    expect(models).toBeGreaterThan(20);
    await tab('Control');
    await page.click('.ctl-right button:has-text("Analyse all")');
    await page.waitForFunction(() => (window as any).app.station.itemsOfType(111).every((m: any) => m.lastReport !== null), null, { timeout: 120000 });
    const verdicts: Array<{ name: string; kind: string; ok: boolean | null; err: string | null }> = await ev(() => (window as any).app.station.itemsOfType(111).map((m: any) => ({ name: m.name, kind: m.kind, ok: m.lastOk, err: /^(Error|✗ error|Parse error)/m.test(m.lastReport ?? '') ? m.lastReport.slice(0, 120) : null })));
    const failed = verdicts.filter((v) => v.err);
    expect(failed, JSON.stringify(failed)).toEqual([]);
    expect(verdicts.filter((v) => v.ok === true).length).toBeGreaterThanOrEqual(15);
    // diagram editors
    for (const kind of ['des', 'petri', 'bt']) {
      const name = await ev((k) => (window as any).app.station.itemsOfType(111).find((x: any) => x.kind === k)?.name ?? null, kind);
      expect(name, kind).not.toBeNull();
      await page.click(`.ctl-item:has-text("${name!.slice(0, 24)}")`);
      await page.waitForTimeout(300);
      await page.click('.ctl-right button:has-text("Diagram")');
      await page.waitForTimeout(400);
      expect(await ev(() => !!document.querySelector('.ctl-right svg, .graph-editor svg, .bt-editor'))).toBe(true);
      await page.click('.ctl-right button:has-text("Text")');
    }
    // group control
    await menu('Group', 'Course examples');
    await page.selectOption('.dialog select', '*');
    await page.click('.dialog .btn.primary');
    await page.waitForTimeout(800);
    const group = await ev(() => (window as any).app.station.itemsOfType(111).filter((m: any) => (m.kind as string) && !['des', 'petri', 'bt', 'smv', 'gr1', 'hybrid', 'stn', 'plan', 'htn', 'mdp', 'mrta', 'mapf', 'sched', 'rt', 'rel', 'vv', 'tamp', 'sc', 'ltl'].includes(m.kind)).length);
    expect(group).toBeGreaterThan(30);
    const runnable = await ev(() => { const app = (window as any).app; for (let i = 0; i < 4; i++) app.addMobileRobot('amr'); return app.station.itemsOfType(111).find((x: any) => x.kind === 'consensus')?.name ?? null; });
    expect(runnable).not.toBeNull();
    await page.click(`.ctl-item:has-text("${runnable!.slice(0, 24)}")`);
    await page.waitForTimeout(300);
    await page.click('.ctl-run button:has-text("Run on fleet")');
    await page.waitForTimeout(2500);
    const status = await page.textContent('.ctl-run');
    expect(status).toMatch(/t=\d/);
    await page.click('.ctl-run button:has-text("Stop")');
    noErrors();
  }, 300000);

  it('demo scenarios dialog runs the whole suite headless and every scenario passes', async () => {
    await load('demo=none');
    await menu('Help', 'Demo scenarios');
    await page.waitForSelector('.dialog');
    const n = await ev(() => document.querySelectorAll('.dialog .nav-rec').length);
    expect(n).toBeGreaterThan(50);
    await page.click('.dialog button:has-text("Run all")');
    await page.waitForFunction(() => /scenarios pass/.test(document.querySelector('.dialog .hint')?.textContent ?? ''), null, { timeout: 280000 });
    const summary = (await page.textContent('.dialog .hint'))!;
    const badges = await ev(() => [...document.querySelectorAll('.dialog .badge')].map((b) => b.textContent!.trim()));
    expect(badges.filter((b) => b === 'FAIL'), summary).toEqual([]);
    expect(badges.filter((b) => b === 'pass').length).toBe(n);
    await closeDialogs();
    noErrors();
  }, 300000);

  it('language switch: the menubar and the tabs read in Russian and switch back', async () => {
    await load('demo=pickplace');
    await menu('View', 'Language');
    await page.waitForTimeout(300);
    const ru = (await page.textContent('.menubar'))!;
    expect(ru).toMatch(/Файл/); expect(ru).toMatch(/Справка/);
    expect((await page.textContent('.tab-bar'))!).toMatch(/Программа/);
    await menu('Вид', 'Язык');
    await page.waitForTimeout(300);
    expect((await page.textContent('.menubar'))!).toMatch(/File/);
    noErrors();
  });
});
