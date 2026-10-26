/**
 * R2 / contract C4: File › MURKA examples in the production build (system Chrome). The MURKA projects are picked as
 * folders like a user does (webkitdirectory input), the stations are built by R3 / R4's providers in the browser and
 * the missions run on the studio's own simulator:
 *  - retail store (robot-retailer-murka-v2): the undock mission ends 'done' and moves the robot off the dock;
 *  - orchard (murca_garden_ws, orchard_demo.wbt): Mission B plays on the world clock and moves the fleet.
 * Skips (visible reason) when the MURKA_Simulation folder is absent.
 */
import { describe, it, expect, beforeAll, afterAll } from 'vitest';
import { chromium, Browser, Page } from 'playwright';
import { existsSync } from 'node:fs';

const BASE = process.env.E2E_BASE ?? 'http://127.0.0.1:4180/';
const CHROME = process.env.E2E_CHROME ?? ['C:/Program Files/Google/Chrome/Application/chrome.exe', '/usr/bin/google-chrome'].find((p) => existsSync(p));
const MURKA = process.env.MURKA_DIR ?? 'C:/Users/TRITON 300 SE/YandexDisk/Работа/КБНЦ РАН/MURKA_Simulation';
const RETAIL = `${MURKA}/robot-retailer-murka-v2`;
const GARDEN = `${MURKA}/simulation_ws/src/agromarca/murca_garden_ws`;
const IGNORED = [/favicon/i, /WebGL/i, /GPU stall/i, /Context Lost/i, /ERR_CONNECTION_REFUSED/i, /WebSocket connection/i, /Failed to load resource/i];
let browser: Browser; let page: Page; let errors: string[] = [];
const ev = <T>(fn: (...a: any[]) => T, arg?: unknown): Promise<T> => page.evaluate(fn as any, arg) as Promise<T>;

async function murkaEntry(label: string): Promise<void> {
  await page.click('.menu-btn:has-text("File")');
  await page.hover('.ctx-item:has-text("MURKA examples")');
  await page.click(`.ctx-menu .ctx-menu .ctx-item:has-text("${label}")`);
}
async function pickFolder(dir: string): Promise<void> {
  const fc = await page.waitForEvent('filechooser', { timeout: 30000 });
  await fc.setFiles(dir);
}

beforeAll(async () => {
  browser = await chromium.launch({ ...(CHROME ? { executablePath: CHROME } : {}), args: ['--use-gl=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'] });
  page = await (await browser.newContext({ viewport: { width: 1280, height: 800 } })).newPage();
  page.on('pageerror', (e) => errors.push(`[pageerror] ${e.message}`));
  page.on('console', (m) => { if (m.type() === 'error') errors.push(`[console] ${m.text()}`); });
}, 120000);
afterAll(async () => { await browser?.close(); });

describe('File › MURKA examples in the browser (C4)', () => {
  it.skipIf(!existsSync(RETAIL))('retail store: the folder pick builds the MURKA v2 store and the undock mission ends done', async () => {
    errors = [];
    await page.goto(`${BASE}?demo=none&server=off`, { waitUntil: 'load' });
    await page.waitForSelector('.menubar');
    await murkaEntry('Retail store mission');
    await page.selectOption('.overlay select >> nth=1', 'undock');
    await page.fill('.overlay input[type=number]', '20');
    await Promise.all([pickFolder(RETAIL), page.click('.overlay .dialog-actions .btn.primary')]);
    await page.waitForFunction(() => (window as any).app.logs.some((l: any) => /^MURKA undock: /.test(l.text)), null, { timeout: 600000 });
    const r = await ev(() => { const app = (window as any).app; return { line: app.logs.find((l: any) => /^MURKA undock: /.test(l.text)).text, station: app.station.name, mobiles: app.station.itemsOfType(100).length, events: app.logs.filter((l: any) => /^\[MURKA undock\]/.test(l.text)).length }; });
    // the mission result: outcome, simulated time and the UNDOCK stage of the MURKA FSM
    expect(r.line, JSON.stringify(r)).toMatch(/^MURKA undock: done in \d+ s — UNDOCK: done$/);
    expect(Number(/in (\d+) s/.exec(r.line)![1])).toBeGreaterThan(0);
    expect(r.station).toMatch(/MURKA|murom|retail/i);
    expect(r.mobiles).toBeGreaterThan(0);
    expect(errors.filter((e) => !IGNORED.some((re) => re.test(e)))).toEqual([]);
  }, 900000);

  it.skipIf(!existsSync(GARDEN))('orchard: Mission B plays on the world clock and moves the MURKA fleet', async () => {
    errors = [];
    await page.goto(`${BASE}?demo=none&server=off`, { waitUntil: 'load' });
    await page.waitForSelector('.menubar');
    await Promise.all([pickFolder(GARDEN), murkaEntry('Mission B')]);
    await page.waitForFunction(() => (window as any).app.logs.some((l: any) => /^Scripted fleet: \d+ waypoints/.test(l.text)), null, { timeout: 600000 });
    const p0: number[][] = await ev(() => { const app = (window as any).app; return ['UTP-G-01', 'UTP-G-04', 'UTP-A-01'].map((n) => Array.from(app.station.find(n).poseAbs() as number[]).slice(12, 15)); });
    await ev(() => { (window as any).app.simSpeed = 10; });
    await page.waitForFunction(() => (window as any).app.worldTime > 20, null, { timeout: 300000 });
    const p1: number[][] = await ev(() => { const app = (window as any).app; return ['UTP-G-01', 'UTP-G-04', 'UTP-A-01'].map((n) => Array.from(app.station.find(n).poseAbs() as number[]).slice(12, 15)); });
    const moved = p0.map((p: number[], i: number) => Math.hypot(p1[i][0] - p[0], p1[i][1] - p[1], p1[i][2] - p[2]));
    // the harvester and the logistics robot drive their first waypoints (several metres) within 20 s of script time
    expect(moved[0], 'UTP-G-01 (mm)').toBeGreaterThan(1000);
    expect(moved[1], 'UTP-G-04 (mm)').toBeGreaterThan(1000);
    expect(errors.filter((e) => !IGNORED.some((re) => re.test(e)))).toEqual([]);
  }, 900000);
});
