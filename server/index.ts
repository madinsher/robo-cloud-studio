/**
 * VerticalBot Studio server.
 *
 *  - WebSocket JSON-RPC endpoint (default port 20500, STUDIO_PORT) speaking the RoboDK-compatible Robolink API.
 *    Python clients use python/robodk/robolink.py (drop-in, also `from robolink import *`).
 *  - Plain TCP JSON-lines endpoint on port+1 (MATLAB / C++ / PLC clients), relayed like the WebSocket.
 *  - RoboDK binary API protocol (official `robodk` package unmodified) on STUDIO_RDK_PORT when set (src/api/rdk_protocol.ts).
 *  - Relay mode: when a browser tab is connected as "host", every request is forwarded to it and executed against the
 *    live 3D station (what the user sees). Otherwise the server runs a headless station itself, with a realtime clock
 *    (STUDIO_SIM_SPEED) so robot moves, programs and polling scripts behave like in RoboDK.
 *  - Station Python programs (RoboDK PROGRAM_PYTHON items, program calls like SprayOn(1)) run on CPython with the
 *    drop-in on PYTHONPATH (src/api/runner_cpython.ts); the browser asks for them through POST /python/run.
 *  - HTTP: GET /health, GET/POST /station.json, POST /convert/rdk, POST /python/run, /vda5050/*, /vision/*.
 *
 * Run: npm run server   (node --experimental-strip-types server/index.ts)
 */
import http from 'node:http';
import { readFileSync, writeFileSync, existsSync, mkdtempSync, rmSync } from 'node:fs';
import { execFile, execFileSync } from 'node:child_process';
import { tmpdir } from 'node:os';
import path from 'node:path';
import { Station, ItemType, Item } from '../src/core/items/item.ts';
import { RpcRequest, RpcResponse, READ_ONLY_METHODS } from '../src/api/rpc.ts';
import { createNodeHost } from '../src/api/node_host.ts';
import { findPython } from '../src/api/runner_cpython.ts';
import { loadStation, saveStation } from '../src/io/station-file.ts';
import '../src/posts/index.ts';
import { createDriver, drivers, RobotDriver } from './drivers/index.ts';
import { Vda5050Service, mqttConnector } from './vda5050.ts';
import { handleVisionHttp } from './vision.ts';
import { handlePostsHttp, runNativePost, listNativePosts, roboDKPython, roboDKPostsDir } from './posts_native.ts';
import { setLocalNativePosts, registerNativePosts } from '../src/posts/native_client.ts';
import { installVisionRunnerGlobal } from '../src/vision/program_vision.ts';
import { setSensingAssets } from '../src/vision/sensing.ts';

const PORT = Number(process.env.STUDIO_PORT ?? 20500);

// RoboDK's own posts (compiled ones included) for the headless station: MakeProgram with post "RDK_<name>" (or a
// RoboDK post name the studio has no built-in equivalent for) runs them in-process with RoboDK's embedded Python
if (roboDKPython() && roboDKPostsDir()) {
  setLocalNativePosts({ list: listNativePosts, run: (post, job, source) => runNativePost(post, job, { source }) });
  const ids = await registerNativePosts();
  console.log(`[studio-server] ${ids.length} RoboDK post processors registered (RDK_<name>)`);
}
const STATION_FILE = process.env.STUDIO_STATION ?? '';

let initial = new Station('Headless station');
if (STATION_FILE && existsSync(STATION_FILE)) {
  initial = loadStation(JSON.parse(readFileSync(STATION_FILE, 'utf8')));
  initial.filePath = STATION_FILE;
  console.log(`[studio-server] loaded ${STATION_FILE}`);
}

/** Live robot drivers keyed by robot name (RUNMODE_RUN_ROBOT). */
const liveDrivers = new Map<string, RobotDriver>();
async function driverCommand(msg: any): Promise<any> {
  const { action, robot, driver, ip, port, joints, pose, speed, io, value, script } = msg.params?.[0] ?? {};
  if (action === 'list') return { drivers: [...drivers.keys()], connected: [...liveDrivers.entries()].map(([k, d]) => ({ robot: k, ...d.state() })) };
  if (action === 'connect') { const d = createDriver(driver); await d.connect(ip, port); liveDrivers.set(robot, d); return d.state(); }
  const d = liveDrivers.get(robot);
  if (!d) throw new Error(`Robot ${robot} is not connected to a driver`);
  switch (action) {
    case 'disconnect': await d.disconnect(); liveDrivers.delete(robot); return { ok: true };
    case 'state': return d.state();
    case 'moveJ': await d.moveJ(joints, speed, true); return d.state();
    case 'moveL': await d.moveL(pose, speed, true); return d.state();
    case 'setDO': await d.setDO(io, value); return { ok: true };
    case 'getDI': return { value: await d.getDI(io) };
    case 'runScript': await d.runScript(script); return { ok: true };
    case 'stop': await d.stop(); return { ok: true };
    default: throw new Error(`Unknown driver action ${action}`);
  }
}
const special = (req: RpcRequest): Promise<RpcResponse> | undefined => {
  if (req.method !== '__driver__') return undefined;
  return driverCommand(req).then((result) => ({ id: req.id, result }), (e) => ({ id: req.id, error: String(e?.message ?? e) }));
};

const httpServer = http.createServer();
await new Promise<void>((resolve) => httpServer.listen(PORT, () => resolve()));
const host = await createNodeHost({
  station: initial, server: httpServer, tcpPort: Number(process.env.STUDIO_TCP_PORT ?? PORT + 1), special,
  afterHeadless: (req) => { if (STATION_FILE && !READ_ONLY_METHODS.has(req.method)) { try { writeFileSync(STATION_FILE, JSON.stringify(saveStation(host.station(), host.rdk.assets))); } catch { /* ignore */ } } },
});
const station = () => host.station();
// VisionDetect("<camera>") in robot programs and Cam2D_Detect / Vision_Run of headless API clients: the vision runner
// renders with the headless station's meshes
installVisionRunnerGlobal();
setSensingAssets(host.rdk.assets ?? null);

/** VDA 5050 (AGV fleets over MQTT): master + digital-twin bridge, see server/vda5050.ts */
const vda = new Vda5050Service(() => station(), (url, opts) => { throw new Error(`MQTT connector not ready for ${url} (${Object.keys(opts).length} opts)`); });
mqttConnector().then((fn) => { vda.connectFn = fn; if (process.env.STUDIO_MQTT_URL) vda.connect({ url: process.env.STUDIO_MQTT_URL, prefix: process.env.STUDIO_VDA_PREFIX, role: (process.env.STUDIO_VDA_ROLE as any) ?? 'master', manufacturer: process.env.STUDIO_VDA_MANUFACTURER }); }).catch((e) => console.warn(`[studio-server] mqtt unavailable: ${e.message}`));
const readJson = (req: http.IncomingMessage) => new Promise<any>((resolve, reject) => { let body = ''; req.on('data', (c) => (body += c)); req.on('end', () => { try { resolve(body ? JSON.parse(body) : {}); } catch (e) { reject(e); } }); req.on('error', reject); });

/** Interpreter for the RoboDK converter: STUDIO_ROBODK_PYTHON, RoboDK's own Python, else the default Python 3. */
function converterPython(): string {
  if (process.env.STUDIO_ROBODK_PYTHON) return process.env.STUDIO_ROBODK_PYTHON;
  for (const p of ['C:/RoboDK/Python37/python.exe', 'C:/RoboDK/Python/python.exe']) if (existsSync(p)) return p;
  try { return findPython().cmd; } catch { return process.platform === 'win32' ? 'python' : 'python3'; }
}
/** The converter needs the *official* robodk package: run isolated (-I) so python/robodk (the drop-in) cannot shadow it. */
let converterStatus: { ok: boolean; detail: string } | null = null;
function checkConverter(): { ok: boolean; detail: string } {
  if (converterStatus) return converterStatus;
  try {
    // same import path as python/rdk2vbs.py: RoboDK's own API folder first (<RoboDK>/Python), nothing from the studio
    const api = path.join(process.env.ROBODK_PATH ?? 'C:/RoboDK', 'Python').split(path.sep).join('/');
    const out = execFileSync(converterPython(), ['-I', '-B', '-c', `import sys, os; sys.path.insert(0, ${JSON.stringify(api)}); import robodk.robolink as r; print(os.path.abspath(r.__file__))`], { encoding: 'utf8', timeout: 20000, windowsHide: true }).trim();
    const dropIn = path.resolve(process.cwd(), 'python').toLowerCase();
    converterStatus = { ok: !!out && !out.toLowerCase().startsWith(dropIn), detail: out };
  } catch (e: any) { converterStatus = { ok: false, detail: String(e?.message ?? e).slice(0, 300) }; }
  return converterStatus;
}

httpServer.on('request', async (req, res) => {
  if ((req.url ?? '').startsWith('/vda5050/') && (await vda.handleHttp(req, res, () => readJson(req)))) return;
  if ((req.url ?? '').startsWith('/vision/') && (await handleVisionHttp(req, res, () => readJson(req)))) return;
  if ((req.url ?? '').startsWith('/posts/native') && (await handlePostsHttp(req, res, () => readJson(req)))) return;
  if (req.method === 'OPTIONS') { res.writeHead(204, { 'access-control-allow-origin': '*', 'access-control-allow-headers': '*', 'access-control-allow-methods': 'GET,POST,OPTIONS' }); res.end(); return; }
  res.setHeader('access-control-allow-origin', '*');
  const json = (code: number, body: any) => { res.writeHead(code, { 'content-type': 'application/json' }); res.end(JSON.stringify(body)); };
  if (req.url === '/health') {
    let python: string | null = null; try { python = findPython().cmd; } catch { python = null; }
    json(200, { ok: true, host: host.api.hasHost(), station: station().name, python, pythonScripts: host.runner.running(), rdkConverter: checkConverter(), vda5050: vda.connected, vision: existsSync(path.join(process.cwd(), 'python', 'vision_infer.py')) });
    return;
  }
  if (req.url === '/station.json' && req.method === 'GET') { json(200, saveStation(station(), host.rdk.assets)); return; }
  if (req.url === '/station.json' && req.method === 'POST') {
    let body = '';
    req.on('data', (c) => (body += c));
    req.on('end', () => { try { host.setStation(loadStation(JSON.parse(body), host.rdk.assets)); res.writeHead(200); res.end('ok'); } catch (e: any) { res.writeHead(400); res.end(String(e.message)); } });
    return;
  }
  if (req.url === '/python/run' && req.method === 'POST') {
    // run a station Python program on CPython; its API calls go to this server (relayed to the browser host if any)
    try {
      const { name, code, args } = await readJson(req);
      const item = station().find(String(name ?? ''), ItemType.PROGRAM_PYTHON);
      const program: Item = item ?? Object.assign(new Item(ItemType.PROGRAM_PYTHON, String(name ?? 'script')), {});
      if (code) program.params.pythonCode = String(code);
      const before = host.runner.history.length;
      await host.runner({ station: station(), runtime: host.rdk.exec.rt(), name: String(name ?? 'script'), args: (args ?? []).map(String), robot: null, program });
      json(200, { ok: true, output: host.runner.history.slice(before).map((h) => h.output).join('\n') });
    } catch (e: any) { json(500, { error: String(e?.message ?? e) }); }
    return;
  }
  if (req.url === '/convert/rdk' && req.method === 'POST') {
    // Convert an uploaded .rdk/.robot/.tool with a locally installed RoboDK (official robodk package) -> .vbstation JSON
    const chunks: Buffer[] = [];
    req.on('data', (c) => chunks.push(c));
    req.on('end', () => {
      const name = decodeURIComponent(String(req.headers['x-filename'] ?? 'station.rdk')).replace(/[^\w.-]/g, '_');
      const dir = mkdtempSync(path.join(tmpdir(), 'vbs-rdk-'));
      const src = path.join(dir, name), out = path.join(dir, 'out.vbstation');
      writeFileSync(src, Buffer.concat(chunks));
      execFile(converterPython(), ['-I', path.join(process.cwd(), 'python', 'rdk2vbs.py'), src, out], { timeout: 180000 }, (err, stdout, stderr) => {
        if (err || !existsSync(out)) json(501, { error: 'RoboDK conversion unavailable', detail: String(stderr || stdout || err?.message).slice(0, 2000) });
        else { res.writeHead(200, { 'content-type': 'application/json' }); res.end(readFileSync(out)); }
        rmSync(dir, { recursive: true, force: true });
      });
    });
    return;
  }
  res.writeHead(404); res.end();
});

// Official RoboDK binary protocol (opt-in: the official robodk package / RoboDK C#, C++, MATLAB clients)
if (process.env.STUDIO_RDK_PORT) {
  import('../src/api/rdk_protocol.ts').then(({ startRdkProtocolServer }) => startRdkProtocolServer({ port: Number(process.env.STUDIO_RDK_PORT), handle: (req: RpcRequest) => host.api.handle(req) }))
    .then((s) => console.log(`[studio-server] RoboDK binary API protocol on port ${s.port}`))
    .catch((e) => console.warn(`[studio-server] RoboDK protocol unavailable: ${e.message}`));
}

console.log((`[studio-server] listening on ws://localhost:${PORT} (RoboDK-compatible API), TCP JSON-lines on ${host.tcpPort} — open the studio with ?server=ws://localhost:${PORT} to relay to the browser`));
