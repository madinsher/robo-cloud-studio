/**
 * R2 end-to-end smoke of the studio SERVER (dist-server bundle, built here with vite.server.config.ts) driven by the
 * OFFICIAL RoboDK Python package (C:/RoboDK/Python, python -I, robodk_ip=127.0.0.1 so it never starts RoboDK) over the
 * RoboDK binary protocol (STUDIO_RDK_PORT), headless: load a RoboDK Library station with AddFile, validate and run its
 * program, then MakeProgram with RoboDK's own post executed natively by the server (RDK_<post>, RoboDK's embedded Python).
 * Ground truth: RoboDK 5.9.4 (tests/fixtures/robodk_gt: Update(), final joints, the program RoboDK generated itself).
 */
import { describe, it, expect, beforeAll, afterAll } from 'vitest';
import { spawn, spawnSync, ChildProcess } from 'node:child_process';
import { existsSync, mkdtempSync, readFileSync, writeFileSync, rmSync } from 'node:fs';
import { createServer } from 'node:net';
import { gunzipSync } from 'node:zlib';
import { tmpdir } from 'node:os';
import path from 'node:path';
import { diffPrograms } from '../posts_gt_util';

const ROOT = path.resolve(__dirname, '../..');
const OFFICIAL = 'C:/RoboDK/Python';
const LIB_01B = 'C:/RoboDK/Library/Example-01.b-Robot Drilling - KUKA.rdk';
const GT = path.join(ROOT, 'tests/fixtures/robodk_gt');
const py = ['py', 'python', 'python3'].find((c) => spawnSync(c, ['-c', 'import sys; assert sys.version_info >= (3, 7)'], { windowsHide: true }).status === 0);
const ready = !!py && existsSync(`${OFFICIAL}/robodk/robolink.py`) && existsSync(LIB_01B) && existsSync('C:/RoboDK/Posts/KUKA_KRC2.py');

const freePort = () => new Promise<number>((resolve, reject) => { const s = createServer(); s.listen(0, '127.0.0.1', () => { const p = (s.address() as any).port; s.close(() => resolve(p)); }); s.on('error', reject); });
/** Files of a .tgz (ustar) archive. */
function untgz(buf: Buffer): Map<string, Buffer> {
  const tar = gunzipSync(buf), out = new Map<string, Buffer>();
  for (let off = 0; off + 512 <= tar.length;) {
    const name = tar.subarray(off, off + 100).toString('utf8').replace(/\0.*$/s, '');
    if (!name) break;
    const size = parseInt(tar.subarray(off + 124, off + 136).toString('utf8').replace(/\0.*$/s, '').trim() || '0', 8);
    const type = String.fromCharCode(tar[off + 156]);
    if (type === '0' || type === '\0') out.set(name.replace(/^\.\//, ''), tar.subarray(off + 512, off + 512 + size));
    off += 512 + Math.ceil(size / 512) * 512;
  }
  return out;
}

function runOfficial(port: number, cwd: string, body: string, timeout = 240000): Promise<{ status: number | null; out: string; err: string; json: any }> {
  const file = path.join(cwd, 'case.py');
  writeFileSync(file, `import sys, json, time, os\nsys.path.insert(0, ${JSON.stringify(OFFICIAL)})\nfrom robodk.robolink import *\nfrom robodk.robomath import *\nimport robodk.robolink as _rl\nassert _rl.__file__.replace('\\\\', '/').startswith(${JSON.stringify(OFFICIAL)}), _rl.__file__\nRDK = Robolink(robodk_ip="127.0.0.1", port=${port})\ndef out(**k):\n    print('JSON:' + json.dumps(k))\n${body}\n`);
  return new Promise((resolve) => {
    const child = spawn(py!, ['-I', '-B', '-u', file], { cwd, windowsHide: true, env: { ...process.env, PYTHONIOENCODING: 'utf-8' } });
    let out = '', err = '';
    child.stdout.on('data', (d) => (out += d));
    child.stderr.on('data', (d) => (err += d));
    const timer = setTimeout(() => { err += '[timeout]'; child.kill(); }, timeout);
    child.on('close', (status) => { clearTimeout(timer); const line = out.split(/\r?\n/).reverse().find((l) => l.startsWith('JSON:')); resolve({ status, out, err, json: line ? JSON.parse(line.slice(5)) : null }); });
  });
}

describe.skipIf(!ready)('studio server driven by the official RoboDK Python package (headless)', () => {
  let server: ChildProcess | null = null;
  let log = '';
  let ports: { ws: number; tcp: number; rdk: number };
  let work: string;
  beforeAll(async () => {
    const vite = path.join(ROOT, 'node_modules/vite/bin/vite.js');
    const b = spawnSync(process.execPath, [vite, 'build', '-c', 'vite.server.config.ts', '--logLevel', 'error'], { cwd: ROOT, encoding: 'utf8', windowsHide: true });
    expect(b.status, b.stderr).toBe(0);
    ports = { ws: await freePort(), tcp: await freePort(), rdk: await freePort() };
    work = mkdtempSync(path.join(tmpdir(), 'vbs-r2-server-'));
    server = spawn(process.execPath, ['dist-server/index.js'], { cwd: ROOT, windowsHide: true, env: { ...process.env, STUDIO_PORT: String(ports.ws), STUDIO_TCP_PORT: String(ports.tcp), STUDIO_RDK_PORT: String(ports.rdk), STUDIO_SIM_SPEED: '5', STUDIO_STATION: '' } });
    server.stdout!.on('data', (d) => (log += d)); server.stderr!.on('data', (d) => (log += d));
    const t0 = Date.now();
    while (!/binary API protocol on port/.test(log)) { if (Date.now() - t0 > 60000) throw new Error(`server did not start:\n${log}`); await new Promise((r) => setTimeout(r, 200)); }
  }, 180000);
  afterAll(() => {
    if (server?.pid) { if (process.platform === 'win32') spawnSync('taskkill', ['/pid', String(server.pid), '/T', '/F'], { stdio: 'ignore' }); else server.kill(); }
    if (work) rmSync(work, { recursive: true, force: true });
  });

  it('/health: RoboDK converter found with RoboDK\'s own robodk package; native posts listed', async () => {
    const h = await (await fetch(`http://127.0.0.1:${ports.ws}/health`)).json();
    expect(h.ok).toBe(true);
    expect(h.rdkConverter.ok, JSON.stringify(h.rdkConverter)).toBe(true);
    expect(h.rdkConverter.detail.replace(/\\/g, '/')).toMatch(/RoboDK\/Python\/robodk\/robolink\.py$/i);
    const n = await (await fetch(`http://127.0.0.1:${ports.ws}/posts/native`)).json();
    expect(n.available).toBe(true);
    expect(n.posts).toEqual(expect.arrayContaining(['KUKA_KRC2', 'ABB_RAPID_IRC5', 'Fanuc_R30i', 'Universal_Robots']));
    expect(log).toMatch(new RegExp(`${n.posts.length} RoboDK post processors registered`));
  });

  it('Example 01.b: AddFile, Update, RunProgram to the end and MakeProgram with RoboDK\'s KUKA_KRC2 post equal RoboDK 5.9.4', async () => {
    const gt = JSON.parse(gunzipSync(readFileSync(path.join(GT, 'Example-01.b-Robot_Drilling_-_KUKA.rdk_.json.gz'))).toString());
    const gtProg = gt.programs.find((p: any) => p.name === 'DrillingPart');
    const r = await runOfficial(ports.rdk, work, `
st = RDK.AddFile(${JSON.stringify(LIB_01B)})
prog = RDK.Item('DrillingPart', ITEM_TYPE_PROGRAM)
robot = RDK.Item('', ITEM_TYPE_ROBOT)
upd = prog.Update()
home = robot.Joints().list()
prog.RunProgram()
time.sleep(0.2)
busy0 = prog.Busy()
while prog.Busy():
    time.sleep(0.1)
joints = robot.Joints().list()
post0 = robot.setParam('PostProcessor', 'RDK_KUKA_KRC2')
native = prog.MakeProgram(os.path.abspath('native'))
robot.setParam('PostProcessor', 'KUKA_KRC2')
builtin = prog.MakeProgram(os.path.abspath('builtin'))
out(station=st.Name(), robot=robot.Name(), upd=list(upd), home=home, busy0=busy0, joints=joints, native=list(native), builtin=list(builtin),
    nfiles=sorted(os.listdir('native')), bfiles=sorted(os.listdir('builtin')))
`);
    expect(r.json, `${r.out}\n${r.err}\n${log}`).not.toBeNull();
    const j = r.json;
    expect([j.station, j.robot]).toEqual(['Robot drilling', 'KUKA KR 10 R900 sixx']);
    // Update(): [valid instructions, cycle time, distance, ratio, message] = RoboDK's
    expect(j.upd[0]).toBe(gtProg.update[0]);
    expect(Math.abs(j.upd[1] - gtProg.update[1])).toBeLessThan(1e-3);
    expect(Math.abs(j.upd[2] - gtProg.update[2])).toBeLessThan(1e-2);
    expect(j.upd[3]).toBe(1);
    // the program really ran (busy right after the start) and ended at RoboDK's last joint row
    expect(!!j.busy0).toBe(true);
    const last = gtProg.jl.at(-1);
    j.joints.forEach((q: number, i: number) => expect(Math.abs(q - last[i]), `J${i + 1}`).toBeLessThan(0.01));
    expect(j.joints).not.toEqual(j.home);
    // MakeProgram: status OK, one file each
    expect(j.native[0]).toBe(true); expect(j.builtin[0]).toBe(true);
    expect(j.native[1]).toMatch(/KUKA_KRC2 \(RoboDK post, server\)/);
    expect(j.nfiles).toEqual(['DrillingPart.src']); expect(j.bfiles).toEqual(['DrillingPart.src']);
    // RoboDK's own generated program for this station (RoboDK 5.9.4 MakeProgram, free licence dump)
    const code = untgz(readFileSync(path.join(GT, 'code/robodk_generated_code.tgz')));
    const rdk = code.get('Example-01.b-Robot_Drilling_-_KUKA.rdk_/DrillingPart/DrillingPart.src')!.toString('latin1');
    const native = readFileSync(path.join(work, 'native/DrillingPart.src'), 'latin1');
    const builtin = readFileSync(path.join(work, 'builtin/DrillingPart.src'), 'latin1');
    expect(diffPrograms(rdk, native)).toBeNull();
    expect(diffPrograms(rdk, builtin)).toBeNull();
  }, 300000);

  it('a station Python program call from a RoboDK program reaches the server\'s CPython runner (python/run)', async () => {
    const r = await (await fetch(`http://127.0.0.1:${ports.ws}/python/run`, { method: 'POST', headers: { 'content-type': 'application/json' }, body: JSON.stringify({ name: 'r2probe', code: 'from robodk.robolink import *\nRDK = Robolink()\nprint("items", len(RDK.ItemList()))\nRDK.AddFrame("R2 frame from python")\n', args: [] }) })).json();
    expect(r.ok, JSON.stringify(r)).toBe(true);
    expect(r.output).toMatch(/items \d+/);
    const r2 = await runOfficial(ports.rdk, work, `out(found=RDK.Item('R2 frame from python', ITEM_TYPE_FRAME).Valid())`);
    expect(!!r2.json?.found, r2.out + r2.err).toBe(true);
  }, 120000);

  it('the browser opened with ?server= lists RoboDK\'s posts; the Export path through the server equals the built-in post', async () => {
    const BASE = process.env.E2E_BASE ?? 'http://127.0.0.1:4180/';
    const CHROME = process.env.E2E_CHROME ?? ['C:/Program Files/Google/Chrome/Application/chrome.exe', '/usr/bin/google-chrome'].find((p) => existsSync(p));
    const { chromium } = await import('playwright');
    const browser = await chromium.launch({ ...(CHROME ? { executablePath: CHROME } : {}), args: ['--use-gl=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'] });
    try {
      const page = await (await browser.newContext({ viewport: { width: 1280, height: 800 } })).newPage();
      const errors: string[] = [];
      page.on('pageerror', (e) => errors.push(e.message));
      // demo station (no .rdk: opening one with ?server= would ask the server to convert it with RoboDK)
      await page.goto(`${BASE}?demo=pickplace&server=ws://127.0.0.1:${ports.ws}`, { waitUntil: 'load' });
      await page.waitForFunction(() => (window as any).app.posts().some((p: any) => p.id === 'RDK_Universal_Robots'), null, { timeout: 60000 });
      const r = await page.evaluate(async () => {
        const app = (window as any).app;
        const ids = app.posts().map((p: any) => p.id);
        const native = await app.exportProgramAsync('RDK_Universal_Robots');
        const builtin = app.exportProgramChecked('Universal_Robots');
        return { n: ids.filter((i: string) => i.startsWith('RDK_')).length, native: native.files.map((f: any) => [f.name, f.content]), builtin: builtin.files.map((f: any) => [f.name, f.content]), program: app.activeProgram.name };
      });
      const n = await (await fetch(`http://127.0.0.1:${ports.ws}/posts/native`)).json();
      expect(r.n).toBe(n.posts.length);
      const main = (files: Array<[string, string]>) => files.find(([name]) => name.endsWith('.script'));
      expect(main(r.native)?.[0]).toBe(`${r.program}.script`);
      expect(main(r.builtin)?.[0]).toBe(`${r.program}.script`);
      // RoboDK's own Universal_Robots post (run by the server) and the studio's built-in post write the same program
      expect(diffPrograms(main(r.native)![1], main(r.builtin)![1])).toBeNull();
      expect(errors).toEqual([]);
    } finally { await browser.close(); }
  }, 300000);
});
