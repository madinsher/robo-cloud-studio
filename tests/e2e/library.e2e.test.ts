/**
 * R2 Library sweep in the browser: every RoboDK Library station (C:/RoboDK/Library/*.rdk) is opened through
 * File › Open in the production build (system Chrome), its main program is played to its end with the app's runtime,
 * and the final state is compared with
 *  - the headless runtime (same importer + StationRuntime in Node): status, cycle time, joints of every robot and
 *    absolute pose of every object must be equal (the browser path — worker import, deserialisation, App playback —
 *    changes nothing);
 *  - the RoboDK 5.9.4 ground truth (tests/fixtures/robodk_gt): cycle time (Update), final joints (last row of
 *    InstructionListJoints when complete) and the station's object poses; reported per station in
 *    test-results/library-sweep.json (RoboDK parity itself is R1's tests/robodk_gt.test.ts).
 * No console errors may occur. Station Python programs are not run here (the Pyodide and vision runners are
 * uninstalled, as the headless reference has none): script calls are reported, not executed —
 * tests/e2e/pyodide.e2e.test.ts runs them, tests/ui_r2_wiring.test.ts checks runToEnd with an answering runner.
 * Filter: E2E_LIBRARY_ONLY=<substring>[,<substring>...].
 */
import { describe, it, expect, beforeAll, afterAll } from 'vitest';
import { chromium, Browser, Page, BrowserContext } from 'playwright';
import { existsSync, readdirSync, readFileSync, writeFileSync, mkdirSync } from 'node:fs';
import { gunzipSync } from 'node:zlib';
import path from 'node:path';
import { importRdkBestEffort } from '../../src/io/robodk/rdk_container';
import { StationRuntime } from '../../src/core/motion/runtime';
import { AssetStore } from '../../src/scene/assets';
import { Program } from '../../src/core/items/program';
import { Robot } from '../../src/core/items/robot';
import { ItemType, Station } from '../../src/core/items/item';

const BASE = process.env.E2E_BASE ?? 'http://127.0.0.1:4180/';
const CHROME = process.env.E2E_CHROME ?? ['C:/Program Files/Google/Chrome/Application/chrome.exe', '/usr/bin/google-chrome'].find((p) => existsSync(p));
const LIB = 'C:/RoboDK/Library';
const GT = path.resolve(__dirname, '../fixtures/robodk_gt');
const OUT = path.resolve(__dirname, '../../test-results/library-sweep.json');
const IGNORED = [/favicon/i, /WebGL/i, /GPU stall/i, /Context Lost/i, /ERR_CONNECTION_REFUSED/i, /WebSocket connection/i, /Failed to load resource/i];
const ONLY = (process.env.E2E_LIBRARY_ONLY ?? '').split(',').map((s) => s.trim()).filter(Boolean);
const FILES = existsSync(LIB) ? readdirSync(LIB).filter((f) => f.endsWith('.rdk') && (!ONLY.length || ONLY.some((o) => f.includes(o)))).sort() : [];

let browser: Browser; let ctx: BrowserContext; let page: Page; let errors: string[] = [];
const ev = <T>(fn: (...a: any[]) => T, arg?: unknown): Promise<T> => page.evaluate(fn as any, arg) as Promise<T>;
const report: Record<string, unknown> = {};

const gtOf = (file: string): any => { const p = path.join(GT, file.replace(/ /g, '_').replace(/[()]/g, '_') + '_.json.gz'); return existsSync(p) ? JSON.parse(gunzipSync(readFileSync(p)).toString().replace(/\bNaN\b/g, 'null')) : null; };
/**
 * Main program: a program no other program calls, preferring names starting with "Main", then the longest RoboDK cycle
 * time (GT Update), else the most instructions in the studio's import.
 */
function mainProgram(station: Station, gt: any): Program | null {
  const progs = station.itemsOfType<Program>(ItemType.PROGRAM);
  if (!progs.length) return null;
  const called = new Set<string>();
  for (const p of progs) for (const i of p.instructions()) { const d: any = i.data; if (d.kind === 'call' && d.programName) called.add(String(d.programName)); if (d.kind === 'call' && d.programId) { const q = station.findById(d.programId); if (q) called.add(q.name); } }
  for (const p of gt?.programs ?? []) for (const i of p.instructions ?? []) if (i.type === 8) called.add(String(i.name).replace(/\(.*$/, '').trim());
  const roots = progs.filter((p) => !called.has(p.name));
  const cyc = (p: Program) => { const g = (gt?.programs ?? []).find((x: any) => x.name === p.name); return g?.update && g.update[3] >= 0.999 ? g.update[1] : -1; };
  const cands = (roots.length ? roots : progs).slice().sort((a, b) => Number(/^main/i.test(b.name)) - Number(/^main/i.test(a.name)) || cyc(b) - cyc(a) || b.instructions().length - a.instructions().length);
  return cands[0] ?? null;
}
interface FinalState { status: string; error?: string; t: number; joints: Array<[string, number[]]>; objects: Array<[string, number[]]> }
const rows16 = (m: number[]) => [m[3], m[7], m[11]];

/** Headless reference: the same importer and runtime in Node, no script runner. */
async function headless(file: string, main: string, maxT: number): Promise<FinalState & { items: number }> {
  const b = readFileSync(path.join(LIB, file));
  const assets = new AssetStore();
  const { station } = await importRdkBestEffort(b.buffer.slice(b.byteOffset, b.byteOffset + b.byteLength) as ArrayBuffer, file.replace(/\.rdk$/i, ''), assets);
  const p = station.itemsOfType<Program>(ItemType.PROGRAM).find((x) => x.name === main)!;
  const rt = StationRuntime.of(station);
  rt.assets = assets;
  rt.sink.message = () => {};
  const h: any = rt.start(p, p.robot() ? { robot: p.robot()! } : {});
  // until the main program ends, then on to the same absolute time as the browser (threads it started keep running)
  const t0 = rt.time;
  for (let t = 0; t < maxT - 1e-9 && ['queued', 'running', 'paused', 'blocked'].includes(h.status);) { const d = Math.min(1, maxT - t); rt.tick(d); t += d; }
  const end = t0 + (h.status === 'done' || h.status === 'error' ? Math.min(maxT, Math.ceil(h.endTime - t0) + 2) : maxT);
  while (rt.time < end - 1e-9) rt.tick(Math.min(1, end - rt.time));
  return {
    status: h.status, error: h.error?.message, t: typeof h.endTime === 'number' ? h.endTime - (h.startTime ?? 0) : rt.time,
    joints: station.itemsOfType<Robot>(ItemType.ROBOT).map((r) => [r.name, r.joints()]),
    objects: station.itemsOfType(ItemType.OBJECT).map((o) => [o.name, Array.from(o.poseAbs()).slice(12, 15)]),
    items: [...station.walk()].length,
  };
}

beforeAll(async () => {
  browser = await chromium.launch({ ...(CHROME ? { executablePath: CHROME } : {}), args: ['--use-gl=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'] });
  ctx = await browser.newContext({ viewport: { width: 1280, height: 800 } });
  page = await ctx.newPage();
  page.on('pageerror', (e) => errors.push(`[pageerror] ${e.message}`));
  page.on('console', (m) => { if (m.type() === 'error') errors.push(`[console] ${m.text()}`); });
  page.on('crash', () => errors.push('[crash] page crashed'));
}, 120000);
afterAll(async () => {
  await browser?.close();
  mkdirSync(path.dirname(OUT), { recursive: true });
  writeFileSync(OUT, JSON.stringify(report, null, 1));
});

describe.skipIf(!FILES.length)('RoboDK Library sweep in the browser', () => {
  for (const file of FILES) {
    it(`${file}: opens, its main program plays to the end like the headless runtime`, async () => {
      errors = [];
      const gt = gtOf(file);
      await page.goto(`${BASE}?demo=none&server=off`, { waitUntil: 'load' });
      await page.waitForSelector('.menubar');
      // no script runner at all (Pyodide, vision): script calls resolve at once, as in the headless runtime
      await ev(() => { (window as any).pyRunner?.uninstall?.(); (window as any).visionRunnerOff?.(); });
      await page.click('.menu-btn:has-text("File")');
      const [fc] = await Promise.all([page.waitForEvent('filechooser'), page.click('.ctx-item:has-text("Open / import")')]);
      await fc.setFiles([path.join(LIB, file)]);
      await page.waitForFunction(() => (window as any).app.station.name !== 'New station', null, { timeout: 600000 });
      await page.waitForFunction(() => { const el = document.querySelector<HTMLElement>('.progress-pill'); return !el || el.style.display === 'none' || !/Opening|Importing/.test(el.textContent ?? ''); }, null, { timeout: 600000 });
      // the headless import decides the main program (same importer)
      const b = readFileSync(path.join(LIB, file));
      const ref0 = await importRdkBestEffort(b.buffer.slice(b.byteOffset, b.byteOffset + b.byteLength) as ArrayBuffer, file.replace(/\.rdk$/i, ''), new AssetStore());
      const main = mainProgram(ref0.station, gt);
      const opened = await ev(() => { const app = (window as any).app; return { items: [...app.station.walk()].length, missing: (() => { let n = 0; app.renderer.scene.traverse((o: any) => { if (o.userData.missingMesh) n++; }); return n; })() }; });
      expect(opened.items, 'items in the browser vs the headless import').toBe([...ref0.station.walk()].length);
      expect(opened.missing, 'meshes missing in the 3D view').toBe(0);
      const entry: any = { items: opened.items, main: main?.name ?? null };
      report[file] = entry;
      if (!main) { expect(errors.filter((e) => !IGNORED.some((re) => re.test(e)))).toEqual([]); return; }
      const g = (gt?.programs ?? []).find((x: any) => x.name === main.name);
      const maxT = Math.min(60000, Math.max(300, 4 * (g?.update?.[1] ?? 0) + 120));
      const browserState: FinalState & { scriptCalls: number } = await ev(async (a: any) => {
        const app = (window as any).app;
        const p = [...app.station.walk()].find((i: any) => i.name === a.main && typeof i.instructions === 'function');
        const n0 = app.logs.length;
        const t0 = app.runtime.time;
        const h = app.runProgram(p);
        await app.runToEnd(a.maxT);
        // threads the program started keep running: advance to the same absolute time as the headless reference
        const rt = app.runtime;
        const end = t0 + (h.status === 'done' || h.status === 'error' ? Math.min(a.maxT, Math.ceil(h.endTime - t0) + 2) : a.maxT);
        while (rt.time < end - 1e-9) rt.tick(Math.min(1, end - rt.time));
        return {
          scriptCalls: app.logs.slice(n0).filter((l: any) => /script|Call /i.test(l.text)).length,
          status: h.status, error: h.error?.message, t: typeof h.endTime === 'number' ? h.endTime - (h.startTime ?? 0) : app.simTime,
          joints: app.station.itemsOfType(2).map((r: any) => [r.name, r.joints()]),
          objects: app.station.itemsOfType(5).map((o: any) => [o.name, Array.from(o.poseAbs()).slice(12, 15)]),
        };
      }, { main: main.name, maxT });
      const ref = await headless(file, main.name, maxT);
      Object.assign(entry, { status: browserState.status, headlessStatus: ref.status, cycle: browserState.t, error: browserState.error });
      // browser == headless: same status; a finished run has the same cycle time and end state (a run still going after
      // maxT in both — a stall, reported — is compared by status only: its frontier time depends on the step size)
      const diffs: string[] = [];
      if (browserState.status !== ref.status) diffs.push(`status ${browserState.status} (${browserState.error ?? ''}) vs headless ${ref.status} (${ref.error ?? ''})`);
      else if (browserState.status !== 'running') {
        if (Math.abs(browserState.t - ref.t) > 1e-3) diffs.push(`cycle ${browserState.t} vs headless ${ref.t}`);
        if (JSON.stringify(browserState.joints.map(([n]) => n)) !== JSON.stringify(ref.joints.map(([n]) => n))) diffs.push('robot list differs');
        else browserState.joints.forEach(([n, q], i) => q.forEach((v, k) => { const d = Math.abs(v - ref.joints[i][1][k]); if (d > 1e-3) diffs.push(`${n} J${k + 1}: ${v} vs headless ${ref.joints[i][1][k]}`); }));
        if (JSON.stringify(browserState.objects.map(([n]) => n)) !== JSON.stringify(ref.objects.map(([n]) => n))) diffs.push('object list differs');
        else browserState.objects.forEach(([n, p], i) => { const d = Math.hypot(p[0] - ref.objects[i][1][0], p[1] - ref.objects[i][1][1], p[2] - ref.objects[i][1][2]); if (d > 1e-3) diffs.push(`${n}: ${d.toFixed(3)} mm from headless`); });
      } else entry.unfinished = `still running after ${maxT.toFixed(0)} s in the browser and headless`;
      entry.diffs = diffs;
      expect(diffs, diffs.slice(0, 12).join('\n')).toEqual([]);
      // RoboDK ground truth (reported)
      if (g?.update) entry.gtCycle = g.update[1];
      if (g?.update && browserState.status === 'done' && g.update[1] > 0) entry.cycleErr = Math.abs(browserState.t - g.update[1]) / g.update[1];
      const last = (g?.jlStatus ?? -1) > 0 ? g.jl?.at(-1) : null;
      const robot = browserState.joints.find(([n]) => n === (main.robot()?.name ?? ''));
      if (last && robot) entry.jointErrDeg = Math.max(...robot[1].map((v, k) => Math.abs(v - last[k])));
      if (gt?.objects) {
        const left = [...browserState.objects];
        let worst = 0;
        for (const o of gt.objects) {
          const t = rows16(o.poseAbs);
          const c = left.map((e) => ({ e, d: Math.hypot(e[1][0] - t[0], e[1][1] - t[1], e[1][2] - t[2]) })).filter((x) => x.e[0] === o.name).sort((a, b2) => a.d - b2.d)[0];
          if (c) { worst = Math.max(worst, c.d); left.splice(left.indexOf(c.e), 1); }
        }
        entry.objectMaxDevMm = worst;
      }
      const bad = errors.filter((e) => !IGNORED.some((re) => re.test(e)));
      expect(bad, bad.join('\n')).toEqual([]);
    }, 900000);
  }
});
