/**
 * VerticalBot Studio server.
 *
 *  - WebSocket JSON-RPC endpoint (default port 20500, same as RoboDK's API port) speaking the
 *    RoboDK-compatible Robolink API. Python clients use python/robodk/robolink.py (drop-in).
 *  - Relay mode: when a browser tab is connected as "host", every request is forwarded to it and
 *    executed against the live 3D station (what the user sees). Otherwise the server runs a
 *    headless station itself (CI, batch post-processing, fleet simulations on a server).
 *  - HTTP: GET /station.json (headless station), POST /station.json (replace), GET /health.
 *
 * Run: npm run server   (node --experimental-strip-types server/index.ts)
 */
import { WebSocketServer, WebSocket } from 'ws';
import http from 'node:http';
import { readFileSync, writeFileSync, existsSync, mkdtempSync, rmSync } from 'node:fs';
import { execFile } from 'node:child_process';
import { tmpdir } from 'node:os';
import path from 'node:path';
import { Station } from '../src/core/items/item.ts';
import { Robolink } from '../src/api/robolink.ts';
import { executeRpcAsync, RpcRequest } from '../src/api/rpc.ts';
import { loadStation, saveStation } from '../src/io/station-file.ts';
import '../src/posts/index.ts';
import { createDriver, drivers, RobotDriver } from './drivers/index.ts';
import { parseTcpJsonLines } from './tcp.ts';
import { Vda5050Service, mqttConnector } from './vda5050.ts';
import { handleVisionHttp } from './vision.ts';

const PORT = Number(process.env.STUDIO_PORT ?? 20500);
const STATION_FILE = process.env.STUDIO_STATION ?? '';

let station = new Station('Headless station');
if (STATION_FILE && existsSync(STATION_FILE)) {
  station = loadStation(JSON.parse(readFileSync(STATION_FILE, 'utf8')));
  console.log(`[studio-server] loaded ${STATION_FILE}`);
}
let RDK = new Robolink(station);
RDK.onMessage = (m) => console.log(`[studio-server] message: ${m}`);

let host: WebSocket | null = null;
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
const pending = new Map<string, { ws: WebSocket; id: number | string }>();
let seq = 0;

/** VDA 5050 (AGV fleets over MQTT): master + digital-twin bridge, see server/vda5050.ts */
const vda = new Vda5050Service(() => station, (url, opts) => { throw new Error(`MQTT connector not ready for ${url} (${Object.keys(opts).length} opts)`); });
mqttConnector().then((fn) => { vda.connectFn = fn; if (process.env.STUDIO_MQTT_URL) vda.connect({ url: process.env.STUDIO_MQTT_URL, prefix: process.env.STUDIO_VDA_PREFIX, role: (process.env.STUDIO_VDA_ROLE as any) ?? 'master', manufacturer: process.env.STUDIO_VDA_MANUFACTURER }); }).catch((e) => console.warn(`[studio-server] mqtt unavailable: ${e.message}`));
const readJson = (req: http.IncomingMessage) => new Promise<any>((resolve, reject) => { let body = ''; req.on('data', (c) => (body += c)); req.on('end', () => { try { resolve(body ? JSON.parse(body) : {}); } catch (e) { reject(e); } }); req.on('error', reject); });

const httpServer = http.createServer(async (req, res) => {
  if ((req.url ?? '').startsWith('/vda5050/') && (await vda.handleHttp(req, res, () => readJson(req)))) return;
  if ((req.url ?? '').startsWith('/vision/') && (await handleVisionHttp(req, res, () => readJson(req)))) return;
  if (req.method === 'OPTIONS') { res.writeHead(204, { 'access-control-allow-origin': '*', 'access-control-allow-headers': '*', 'access-control-allow-methods': 'GET,POST,OPTIONS' }); res.end(); return; }
  res.setHeader('access-control-allow-origin', '*');
  if (req.url === '/health') { res.writeHead(200, { 'content-type': 'application/json', 'access-control-allow-origin': '*' }); res.end(JSON.stringify({ ok: true, host: !!host, station: station.name, rdkConverter: !!process.env.STUDIO_ROBODK_PYTHON || existsSync(path.join(process.cwd(), 'python', 'rdk2vbs.py')), vda5050: vda.connected, vision: existsSync(path.join(process.cwd(), 'python', 'vision_infer.py')) })); return; }
  if (req.url === '/station.json' && req.method === 'GET') { res.writeHead(200, { 'content-type': 'application/json' }); res.end(JSON.stringify(saveStation(station))); return; }
  if (req.url === '/convert/rdk' && req.method === 'POST') {
    // Convert an uploaded .rdk/.robot/.tool with a locally installed RoboDK (python + robodk package) -> .vbstation JSON
    const python = process.env.STUDIO_ROBODK_PYTHON ?? process.env.PYTHON ?? 'python3';
    const chunks: Buffer[] = [];
    req.on('data', (c) => chunks.push(c));
    req.on('end', () => {
      const name = decodeURIComponent(String(req.headers['x-filename'] ?? 'station.rdk')).replace(/[^\w.-]/g, '_');
      const dir = mkdtempSync(path.join(tmpdir(), 'vbs-rdk-'));
      const src = path.join(dir, name), out = path.join(dir, 'out.vbstation');
      writeFileSync(src, Buffer.concat(chunks));
      execFile(python, [path.join(process.cwd(), 'python', 'rdk2vbs.py'), src, out], { timeout: 180000 }, (err, stdout, stderr) => {
        if (err || !existsSync(out)) { res.writeHead(501, { 'content-type': 'application/json' }); res.end(JSON.stringify({ error: 'RoboDK conversion unavailable', detail: String(stderr || stdout || err?.message).slice(0, 2000) })); }
        else { res.writeHead(200, { 'content-type': 'application/json' }); res.end(readFileSync(out)); }
        rmSync(dir, { recursive: true, force: true });
      });
    });
    return;
  }
  if (req.url === '/station.json' && req.method === 'POST') {
    let body = '';
    req.on('data', (c) => (body += c));
    req.on('end', () => {
      try { station = loadStation(JSON.parse(body)); RDK = new Robolink(station); res.writeHead(200); res.end('ok'); } catch (e: any) { res.writeHead(400); res.end(String(e.message)); }
    });
    return;
  }
  res.writeHead(404); res.end();
});

const wss = new WebSocketServer({ server: httpServer });
wss.on('connection', (ws) => {
  let role: 'client' | 'host' = 'client';
  ws.on('message', (data) => {
    let msg: any;
    try { msg = JSON.parse(String(data)); } catch { return; }
    if (msg.role === 'host') { role = 'host'; host = ws; console.log('[studio-server] browser host connected'); return; }
    if (role === 'host') {
      // response from the browser for a relayed request
      const key = String(msg.id);
      const p = pending.get(key);
      if (p) { pending.delete(key); p.ws.send(JSON.stringify({ ...msg, id: p.id })); }
      return;
    }
    const req = msg as RpcRequest;
    if (req.method === '__driver__') {
      driverCommand(req).then((result) => ws.send(JSON.stringify({ id: req.id, result }))).catch((e) => ws.send(JSON.stringify({ id: req.id, error: String(e?.message ?? e) })));
      return;
    }
    if (host && host.readyState === WebSocket.OPEN) {
      const key = `r${++seq}`;
      pending.set(key, { ws, id: req.id });
      host.send(JSON.stringify({ ...req, id: key }));
      setTimeout(() => { if (pending.has(key)) { pending.delete(key); ws.send(JSON.stringify({ id: req.id, error: 'host timeout' })); } }, 30000);
    } else {
      executeRpcAsync(RDK, req).then((res) => {
        ws.send(JSON.stringify(res));
        if (STATION_FILE && req.method !== 'Item' && req.method !== 'ItemList') {
          try { writeFileSync(STATION_FILE, JSON.stringify(saveStation(station))); } catch { /* ignore */ }
        }
      });
    }
  });
  ws.on('close', () => { if (ws === host) { host = null; console.log('[studio-server] browser host disconnected'); } });
});

// Plain TCP JSON-lines endpoint (port+1) for MATLAB / C++ / PLC clients without WebSocket support
parseTcpJsonLines(PORT + 1, async (req) => {
  if (req.method === '__driver__') { try { return { id: req.id, result: await driverCommand(req) }; } catch (e: any) { return { id: req.id, error: String(e?.message ?? e) }; } }
  return executeRpcAsync(RDK, req);
});

httpServer.listen(PORT, () => console.log(`[studio-server] listening on ws://localhost:${PORT} (RoboDK-compatible API) — open the studio with ?server=ws://localhost:${PORT} to relay to the browser`));
