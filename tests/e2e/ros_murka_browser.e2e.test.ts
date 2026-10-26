/**
 * G3 / V5 F2: the studio IN THE BROWSER as the simulator backend of the real MURKA ROS 2 stack (opt-in: ROS_WSL=1;
 * needs system Chrome, the MURKA_Simulation folder and WSL Ubuntu-22.04 with ROS 2 Jazzy — the workspaces are built
 * once by tests/fixtures/ros_gt/tools/murka_sim/setup_ws.sh, see tests/ros_murka_e2e.test.ts).
 *
 * What a user does: File › Open folder ros2_ws/src/murom_webots (the Webots world alone), Connect › ROS 2 via
 * rosbridge…, "Load retail layout (murom_world/config)…" (the supervisor's $MUROM_CONFIG_DIR is a sibling package),
 * mode "Simulator backend", OK. WSL runs rosbridge_server + rosapi + robot_state_publisher + MURKA localization_node.
 * Then MURKA's own scripts/smoke_test.py (unmodified copy) must print SMOKE PASS (50 products, the robot drives
 * >= 0.3 m in 3 s of wall time, odometry = ground truth), and a subscriber to the head camera / range finder must
 * get bgra8 / 32FC1 320x240 frames at the camera rate (5 Hz in sim time) while /clock keeps real time.
 * The 3D view's WebGL runs on the hardware GPU by default, as a user's browser does (E2E_GL=swiftshader: software GL,
 * which also passes but loads every core with the 3D view, so on a busy machine the WSL rosbridge can lag behind).
 */
import { describe, it, expect, beforeAll, afterAll } from 'vitest';
import { chromium, Browser, Page } from 'playwright';
import { existsSync, mkdirSync, writeFileSync } from 'node:fs';
import path from 'node:path';
import { spawn, execFileSync, type ChildProcess } from 'node:child_process';

const BASE = process.env.E2E_BASE ?? 'http://127.0.0.1:4180/';
const CHROME = process.env.E2E_CHROME ?? ['C:/Program Files/Google/Chrome/Application/chrome.exe', '/usr/bin/google-chrome'].find((p) => existsSync(p));
const MURKA = process.env.MURKA_DIR ?? 'C:/Users/TRITON 300 SE/YandexDisk/Работа/КБНЦ РАН/MURKA_Simulation';
const V2 = `${MURKA}/robot-retailer-murka-v2`;
const SRC = `${V2}/ros2_ws/src`;
const WSL = process.env.ROS_WSL === '1';
const TOOLS = path.resolve(__dirname, '../fixtures/ros_gt/tools/murka_sim');
const PORT = Number(process.env.G3_PORT ?? 9164);
const ENV = { ...process.env, MSYS_NO_PATHCONV: '1', R5_PORT: String(PORT), R5_DOMAIN: process.env.G3_DOMAIN ?? '64', R5_RUN: '/tmp/g3_e2e_murka', WSLENV: 'R5_PORT/u:R5_DOMAIN/u:R5_RUN/u' };
const toWsl = (p: string) => `/mnt/${p[0].toLowerCase()}${p.slice(2).split(path.sep).join('/').replace(/\\/g, '/')}`;
const wsl = (args: string[], timeout = 600000) => execFileSync('wsl.exe', ['-d', 'Ubuntu-22.04', '--', ...args], { encoding: 'utf8', timeout, env: ENV, windowsHide: true });
function wslAsync(args: string[], timeout: number): Promise<string> {
  return new Promise((resolve) => {
    let out = '';
    const p = spawn('wsl.exe', ['-d', 'Ubuntu-22.04', '--', ...args], { env: ENV, windowsHide: true });
    const timer = setTimeout(() => { out += ' [timeout]'; p.kill(); }, timeout);
    p.stdout.on('data', (d) => { out += String(d); });
    p.stderr.on('data', (d) => { out += String(d); });
    p.on('close', () => { clearTimeout(timer); resolve(out); });
  });
}
function startStack(): Promise<ChildProcess> {
  return new Promise((resolve, reject) => {
    let out = '';
    const proc = spawn('wsl.exe', ['-d', 'Ubuntu-22.04', '--', 'bash', toWsl(path.join(TOOLS, 'run_stack.sh')), 'loc'], { env: ENV, windowsHide: true });
    const timer = setTimeout(() => reject(new Error(`stack did not start: ${out}`)), 150000);
    proc.stdout.on('data', (d) => { out += String(d); if (/STACK_READY/.test(out)) { clearTimeout(timer); resolve(proc); } if (/STACK_TIMEOUT/.test(out)) { clearTimeout(timer); reject(new Error(out)); } });
    proc.stderr.on('data', (d) => { out += String(d); });
  });
}
const stopStack = () => { try { wsl(['bash', toWsl(path.join(TOOLS, 'stop_stack.sh'))], 60000); } catch { /* not running */ } };

const READY = WSL && !!CHROME && existsSync(`${SRC}/murom_webots/worlds/retail_store.wbt`);
let browser: Browser; let page: Page; let stack: ChildProcess | null = null;
const errors: string[] = [];
const IGNORED = [/favicon/i, /WebGL/i, /GPU stall/i, /ERR_NO_BUFFER_SPACE/i, /Failed to load resource/i];

beforeAll(async () => {
  if (!READY) return;
  const setup = wsl(['bash', toWsl(path.join(TOOLS, 'setup_ws.sh')), toWsl(V2)], 1800000);
  if (!/R5_SETUP_OK/.test(setup)) throw new Error(`WSL workspaces: ${setup.slice(-800)}`);
  stopStack();
  stack = await startStack();
  const gl = process.env.E2E_GL === 'swiftshader' ? ['--use-gl=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'] : ['--enable-gpu', ...(process.platform === 'win32' ? ['--use-angle=d3d11'] : []), '--ignore-gpu-blocklist'];
  browser = await chromium.launch({ executablePath: CHROME, args: gl });
  page = await (await browser.newContext({ viewport: { width: 1280, height: 800 } })).newPage();
  page.on('pageerror', (e) => errors.push(`[pageerror] ${e.message}`));
  page.on('console', (m) => { if (m.type() === 'error') errors.push(`[console] ${m.text()}`); });
}, 2400000);
afterAll(async () => {
  try { await page?.evaluate(() => (window as any).app.ros?.disconnect?.()); } catch { /* closed */ }
  await browser?.close();
  if (READY) stopStack();
  stack?.kill();
}, 300000);

describe.skipIf(!READY)(`the browser studio as the MURKA ROS 2 backend (ROS_WSL=1${READY ? '' : ': skipped — needs ROS_WSL=1, Chrome and the MURKA folder'})`, () => {
  it('Open folder murom_webots + layout + Simulator backend: MURKA smoke_test.py passes and the cameras stream when subscribed', async () => {
    for (let k = 0; k < 3; k++) { await page.goto(`${BASE}?demo=none&server=off`, { waitUntil: 'load' }); if (await page.waitForFunction(() => !!(window as any).app, null, { timeout: 30000 }).then(() => true, () => false)) break; }
    // File › Open folder: the Webots world alone (no murom_world/config in it)
    await page.click('.menu-btn:has-text("File")');
    const [fc] = await Promise.all([page.waitForEvent('filechooser'), page.click('.ctx-item:has-text("Open folder")')]);
    await fc.setFiles(`${SRC}/murom_webots`);
    // murom_webots is a ROS package that ships a Webots world: the studio asks which one to open
    await page.waitForSelector('.overlay select', { timeout: 120000 });
    await page.selectOption('.overlay select >> nth=0', 'webots');
    await page.click('.overlay .dialog-actions .btn.primary');
    await page.waitForFunction(() => (window as any).app.logs.some((l: any) => /^Opened Webots world/.test(l.text)), null, { timeout: 300000 });
    expect(await page.evaluate(() => [...(window as any).app.station.walk()].filter((i: any) => typeof i.params?.sku === 'string').length)).toBe(0);
    // Connect › ROS 2: the dialog offers the retail layout of the supervisor ($MUROM_CONFIG_DIR = murom_world/config)
    await page.click('.menu-btn:has-text("Connect")');
    await page.click('.ctx-item:has-text("ROS 2 via rosbridge")');
    await page.waitForSelector('.overlay');
    const [fc2] = await Promise.all([page.waitForEvent('filechooser'), page.click('.overlay button:has-text("Load retail layout")')]);
    await fc2.setFiles(`${SRC}/murom_world/config`);
    await page.waitForFunction(() => /Retail layout loaded: 50 products/.test(document.querySelector('.overlay')?.textContent ?? ''), null, { timeout: 30000 });
    await page.fill('.overlay input[type=text] >> nth=0', `ws://127.0.0.1:${PORT}`);
    await page.selectOption('.overlay select >> nth=0', 'sim');
    await page.click('.overlay .dialog-actions .btn.primary');
    await page.waitForFunction(() => !!(window as any).app.ros?.connected, null, { timeout: 60000 });
    const where = await page.evaluate(() => (window as any).app.ros.backendStats());
    expect(where).toMatchObject({ where: 'worker', kind: 'murom' });
    const start = await page.evaluate(() => { const m = (window as any).app.station.itemsOfType(100).find((x: any) => x.name === 'murom'); return { odo: m.state.odometer ?? 0 }; });

    const s0 = await page.evaluate(() => (window as any).app.ros.backendStats());
    const smoke = await wslAsync(['bash', toWsl(path.join(TOOLS, 'smoke.sh')), toWsl(V2)], 300000);
    const s1 = await page.evaluate(() => (window as any).app.ros.backendStats());
    const report = (extra = '') => { mkdirSync(path.resolve(__dirname, '../../test-results'), { recursive: true }); writeFileSync(path.resolve(__dirname, '../../test-results/ros_murka_browser.txt'), [`GL ${process.env.E2E_GL ?? 'gpu'}`, smoke, `backend before smoke ${JSON.stringify(s0)}`, `backend after smoke ${JSON.stringify(s1)}`, extra].join('\n')); };
    report();
    expect(smoke, smoke).toContain('SMOKE PASS');
    expect(smoke).toMatch(/объектов в реестре: 50/);
    const moved = Number(/смещение ([\d.]+) м/.exec(smoke)?.[1]);
    expect(moved).toBeGreaterThan(0.5); // 3 s at 0.25 m/s = 0.75 m at RTF 1 (the test needs 0.3)
    // the page's station follows the worker (3D view): the base drove there and back
    const after = await page.evaluate(() => { const m = (window as any).app.station.itemsOfType(100).find((x: any) => x.name === 'murom'); return { odo: m.state.odometer ?? 0 }; });
    expect(after.odo - start.odo).toBeGreaterThan(1000);

    // a subscriber turns the head camera and the range finder on: 5 Hz in sim time, /clock at real time
    const cam = await wslAsync(['bash', toWsl(path.join(TOOLS, 'cam_rate.sh')), '12'], 300000);
    const line = /CAM_RATE (\{.*\})/.exec(cam);
    expect(line, cam).not.toBeNull();
    const r = JSON.parse(line![1]);
    const s2 = await page.evaluate(() => (window as any).app.ros.backendStats());
    report(`${cam}\nbackend after cameras ${JSON.stringify(s2)}`);
    expect(r['/head_camera/image_color']).toMatchObject({ w: 320, h: 240, enc: 'bgra8', bytes: 307200 });
    expect(r['/head_depth/image']).toMatchObject({ w: 320, h: 240, enc: '32FC1', bytes: 307200 });
    // 5 Hz in sim time while rosbridge keeps up; frames are dropped (camera.dropped) only while its queue lags
    // behind (the /clock echo), so /clock keeps real time on the ROS side in any case
    const rate = Math.min(r['/head_camera/image_color'].rate_sim, r['/head_depth/image'].rate_sim);
    expect(rate, JSON.stringify(s2.camera)).toBeGreaterThan(s2.camera.dropped ? 1 : 4.5);
    expect(r.clock_span / r.wall, JSON.stringify(s2)).toBeGreaterThan(0.85);
    expect(s2.camera.rendered).toBeGreaterThan(20);
    expect(s2.rosLag).toBeLessThan(1);
    expect(s2.rtf).toBeGreaterThan(0.9);
    expect(s2.time - s1.time).toBeGreaterThan(10);
    expect(errors.filter((e) => !IGNORED.some((re) => re.test(e)))).toEqual([]);
  }, 1200000);
});
