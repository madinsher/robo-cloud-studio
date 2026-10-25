/**
 * Node-only API server core used by server/index.ts and the tests: JSON-RPC over WebSocket (Python drop-in, browser
 * host), newline-delimited JSON over TCP (MATLAB / C++ / PLC clients) and, optionally, the RoboDK binary protocol
 * (official robodk package, src/api/rdk_protocol.ts).
 *
 * Relay: when a browser tab connects as {"role":"host"}, every client request (WebSocket *and* TCP) is forwarded to
 * it and executed against the live station; per-method timeouts (rpc.ts methodTimeoutMs, 0 = none) replace the
 * former 30 s cap. Without a host the requests run headless against getRDK().
 */
import { WebSocketServer, WebSocket } from 'ws';
import net from 'node:net';
import type http from 'node:http';
import { executeRpcAsync, methodTimeoutMs, RpcRequest, RpcResponse } from './rpc';
import type { Robolink } from './robolink';
import { wsVerifyClient } from './security';

export interface ApiServerOptions {
  /** HTTP server to attach the WebSocket endpoint to (or a port to listen on). */
  server?: http.Server;
  port?: number;
  /** Plain TCP JSON-lines port (0 = ephemeral, -1 / undefined = disabled). */
  tcpPort?: number;
  /** Address the WebSocket (port mode) and TCP sockets bind to (default 127.0.0.1: this machine only, like RoboDK). */
  host?: string;
  /** WebSocket upgrade filter (default: security.ts wsVerifyClient with no extra origins: refuses foreign web pages). */
  verifyClient?: (info: { origin: string; secure: boolean; req: http.IncomingMessage }, cb: (ok: boolean, code?: number, msg?: string) => void) => void;
  getRDK: () => Robolink;
  /** Extra request handler (e.g. __driver__); return undefined to fall through. */
  special?: (req: RpcRequest) => Promise<RpcResponse> | undefined;
  /** Called after a headless request (persisting the station, ...). */
  afterHeadless?: (req: RpcRequest) => void;
  log?: (msg: string) => void;
}

export interface ApiServer {
  wss: WebSocketServer;
  tcp: net.Server | null;
  /** Execute a request: relayed to the browser host when connected, else headless. */
  handle(req: RpcRequest): Promise<RpcResponse>;
  hasHost(): boolean;
  close(): Promise<void>;
  /** Resolves when listening (port known). */
  ready: Promise<{ port: number; tcpPort: number }>;
}

export function startApiServer(opts: ApiServerOptions): ApiServer {
  const log = opts.log ?? ((m: string) => console.log(`[studio-server] ${m}`));
  const bindHost = opts.host ?? '127.0.0.1';
  const verify = { verifyClient: opts.verifyClient ?? wsVerifyClient({}, log) };
  const wss = opts.server ? new WebSocketServer({ server: opts.server, ...verify }) : new WebSocketServer({ port: opts.port ?? 0, host: bindHost, ...verify });
  let host: WebSocket | null = null;
  const pending = new Map<string, { resolve: (r: RpcResponse) => void; timer: any }>();
  let seq = 0;

  const relay = (req: RpcRequest): Promise<RpcResponse> => new Promise((resolve) => {
    const key = `r${++seq}`;
    const ms = methodTimeoutMs(req.method, req.params);
    const timer = ms > 0 ? setTimeout(() => { if (pending.delete(key)) resolve({ id: req.id, error: `host timeout after ${ms / 1000} s (${req.method})` }); }, ms) : null;
    pending.set(key, { resolve: (r) => resolve({ ...r, id: req.id }), timer });
    host!.send(JSON.stringify({ ...req, id: key }));
  });
  const handle = async (req: RpcRequest): Promise<RpcResponse> => {
    const sp = opts.special?.(req);
    if (sp) return sp;
    if (host && host.readyState === WebSocket.OPEN) return relay(req);
    const res = await executeRpcAsync(opts.getRDK(), req);
    opts.afterHeadless?.(req);
    return res;
  };
  /** A client that opened offline programs and disconnects without Finish(): generate them (RoboDK behaviour). */
  const finishOnClose = (startedOffline: boolean) => { if (startedOffline) handle({ id: 'auto-finish', method: 'Finish' }).catch(() => {}); };

  wss.on('connection', (ws) => {
    let role: 'client' | 'host' = 'client';
    let offline = false;
    ws.on('message', (data) => {
      let msg: any;
      try { msg = JSON.parse(String(data)); } catch { return; }
      if (msg.role === 'host') { role = 'host'; host = ws; log('browser host connected'); return; }
      if (role === 'host') {
        const p = pending.get(String(msg.id));
        if (p) { pending.delete(String(msg.id)); if (p.timer) clearTimeout(p.timer); p.resolve(msg); }
        return;
      }
      const req = msg as RpcRequest;
      if (req.method === 'ProgramStart') offline = true;
      if (req.method === 'Finish') offline = false;
      handle(req).then((res) => { if (ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(res)); }).catch((e) => { if (ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify({ id: req.id, error: String(e?.message ?? e) })); });
    });
    ws.on('close', () => {
      if (ws === host) { host = null; log('browser host disconnected'); for (const [k, p] of pending) { if (p.timer) clearTimeout(p.timer); p.resolve({ id: k, error: 'browser host disconnected' }); } pending.clear(); }
      else finishOnClose(offline);
    });
    ws.on('error', () => {});
  });

  let tcp: net.Server | null = null;
  let tcpReady: Promise<number> = Promise.resolve(0);
  if (opts.tcpPort !== undefined && opts.tcpPort >= 0 && opts.tcpPort !== -1) {
    tcp = net.createServer((sock) => {
      let buf = '';
      let offline = false;
      sock.on('data', (d) => {
        buf += d.toString('utf8');
        let i: number;
        while ((i = buf.indexOf('\n')) >= 0) {
          const line = buf.slice(0, i).trim();
          buf = buf.slice(i + 1);
          if (!line) continue;
          let req: RpcRequest;
          try { req = JSON.parse(line); } catch { sock.write(JSON.stringify({ id: null, error: 'invalid JSON' }) + '\n'); continue; }
          if (req.method === 'ProgramStart') offline = true;
          if (req.method === 'Finish') offline = false;
          handle(req).then((res) => { if (!sock.destroyed) sock.write(JSON.stringify(res) + '\n'); }).catch((e) => { if (!sock.destroyed) sock.write(JSON.stringify({ id: req.id, error: String(e?.message ?? e) }) + '\n'); });
        }
      });
      sock.on('close', () => finishOnClose(offline));
      sock.on('error', () => {});
    });
    tcpReady = new Promise((resolve) => tcp!.listen(opts.tcpPort, bindHost, () => resolve((tcp!.address() as net.AddressInfo).port)));
  }
  const wsReady = new Promise<number>((resolve) => {
    if (opts.server) { const a = opts.server.address(); if (a && typeof a === 'object') resolve(a.port); else opts.server.on('listening', () => resolve((opts.server!.address() as net.AddressInfo).port)); }
    else wss.on('listening', () => resolve((wss.address() as net.AddressInfo).port));
  });
  const ready = Promise.all([wsReady, tcpReady]).then(([port, tcpPort]) => ({ port, tcpPort }));

  return {
    wss, tcp, handle, ready,
    hasHost: () => !!host && host.readyState === WebSocket.OPEN,
    close: () => new Promise<void>((resolve) => {
      for (const c of wss.clients) c.terminate();
      wss.close(() => { if (tcp) tcp.close(() => resolve()); else resolve(); });
    }),
  };
}
