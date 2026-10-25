/**
 * Studio server access control (src/api/security.ts, server/index.ts, src/api/api_server.ts): the server runs code
 * (POST /python/run, /posts/native/run, RunProgram of Python items), so it binds loopback by default, refuses foreign
 * browser origins (drive-by requests from any web page, cross-site WebSocket hijacking), checks the Host header against
 * DNS rebinding and never answers with access-control-allow-origin: *.
 */
import { describe, it, expect, afterEach } from 'vitest';
import http from 'node:http';
import { readFileSync } from 'node:fs';
import path from 'node:path';
import { WebSocket } from 'ws';
import { originAllowed, hostAllowed, guardHttp, bindHost, securityFromEnv } from '../src/api/security';
import { startApiServer, ApiServer } from '../src/api/api_server';
import { Robolink } from '../src/api/robolink';
import { Station } from '../src/core/items/item';

const ROOT = path.resolve(__dirname, '..');

describe('origin / host policy', () => {
  it('local pages and non-browser clients are allowed, foreign pages are not', () => {
    expect(originAllowed(undefined)).toBe(true);
    expect(originAllowed('http://localhost:5173')).toBe(true);
    expect(originAllowed('http://127.0.0.1:4173')).toBe(true);
    expect(originAllowed('http://[::1]:4173')).toBe(true);
    expect(originAllowed('http://studio.localhost:8080')).toBe(true);
    expect(originAllowed('http://evil.example')).toBe(false);
    expect(originAllowed('http://192.168.0.105:20961')).toBe(false);
    expect(originAllowed('null')).toBe(false);
    expect(originAllowed('http://localhost.evil.example')).toBe(false);
    expect(originAllowed('https://my.studio.example', { allowedOrigins: ['https://my.studio.example'] })).toBe(true);
  });
  it('Host header: loopback names and IP literals pass, rebinding DNS names do not', () => {
    expect(hostAllowed('localhost:20500')).toBe(true);
    expect(hostAllowed('127.0.0.1:20500')).toBe(true);
    expect(hostAllowed('[::1]:20500')).toBe(true);
    expect(hostAllowed('192.168.0.105:20500')).toBe(true);
    expect(hostAllowed('rebind.evil.example:20500')).toBe(false);
    expect(hostAllowed('robot-pc:20500', { allowedHosts: ['robot-pc'] })).toBe(true);
  });
  it('the server binds loopback unless STUDIO_HOST says otherwise', () => {
    expect(bindHost({})).toBe('127.0.0.1');
    expect(bindHost({ STUDIO_HOST: '0.0.0.0' })).toBe('0.0.0.0');
    expect(securityFromEnv({ STUDIO_ALLOWED_ORIGINS: 'https://a.example, https://b.example', STUDIO_TOKEN: 's' })).toEqual({ allowedOrigins: ['https://a.example', 'https://b.example'], allowedHosts: [], token: 's' });
  });
});

describe('HTTP guard (as used by server/index.ts before every route)', () => {
  let srv: http.Server | null = null;
  afterEach(() => new Promise<void>((r) => (srv ? srv.close(() => r()) : r())));
  const start = (token?: string) => new Promise<number>((resolve) => {
    srv = http.createServer((req, res) => {
      if (!guardHttp(req, res, { token })) return;
      res.writeHead(200, { 'content-type': 'text/plain' }); res.end('ran');
    });
    srv.listen(0, '127.0.0.1', () => resolve((srv!.address() as any).port));
  });
  const post = (port: number, headers: Record<string, string>) => new Promise<{ status: number; body: string; acao?: string }>((resolve, reject) => {
    const req = http.request({ host: '127.0.0.1', port, path: '/python/run', method: 'POST', headers }, (res) => {
      let body = ''; res.on('data', (c) => (body += c)); res.on('end', () => resolve({ status: res.statusCode!, body, acao: res.headers['access-control-allow-origin'] as string | undefined }));
    });
    req.on('error', reject); req.end('{"name":"x","code":"print(1)"}');
  });

  it('a drive-by POST from a foreign page (text/plain simple request) is refused before anything runs', async () => {
    const port = await start();
    const r = await post(port, { origin: 'http://192.168.0.105:20961', 'content-type': 'text/plain' });
    expect(r.status).toBe(403);
    expect(r.body).not.toContain('ran');
    expect(r.acao).toBeUndefined();
  });
  it('a DNS-rebinding request (Host = attacker domain) is refused', async () => {
    const port = await start();
    expect((await post(port, { host: `rebind.evil.example:${port}`, origin: `http://rebind.evil.example:${port}` })).status).toBe(403);
  });
  it('the studio page (local origin) and a Python client (no Origin) are served; the origin is echoed, never *', async () => {
    const port = await start();
    const page = await post(port, { origin: 'http://127.0.0.1:4173', 'content-type': 'application/json' });
    expect(page.status).toBe(200);
    expect(page.acao).toBe('http://127.0.0.1:4173');
    expect((await post(port, { 'content-type': 'application/json' })).status).toBe(200);
  });
  it('STUDIO_TOKEN: requests without the token are refused', async () => {
    const port = await start('s3cret');
    expect((await post(port, {})).status).toBe(403);
    expect((await post(port, { 'x-studio-token': 's3cret' })).status).toBe(200);
  });
});

describe('API server sockets', () => {
  let api: ApiServer | null = null;
  afterEach(async () => { await api?.close(); api = null; });
  const open = (url: string, origin?: string) => new Promise<'open' | 'refused'>((resolve) => {
    const ws = new WebSocket(url, origin ? { origin } : {});
    ws.on('open', () => { ws.close(); resolve('open'); });
    ws.on('error', () => resolve('refused'));
    ws.on('unexpected-response', () => resolve('refused'));
  });
  it('WebSocket upgrades from a foreign web page are refused (cross-site WebSocket hijacking)', async () => {
    api = startApiServer({ port: 0, tcpPort: 0, getRDK: () => new Robolink(new Station('S')), log: () => {} });
    const { port, tcpPort } = await api.ready;
    expect(await open(`ws://127.0.0.1:${port}`, 'http://evil.example')).toBe('refused');
    expect(await open(`ws://127.0.0.1:${port}`, 'http://localhost:5173')).toBe('open');
    expect(await open(`ws://127.0.0.1:${port}`)).toBe('open');
    // WebSocket and TCP sockets listen on loopback only
    const addr = (api.wss.address() as any).address, tcpAddr = (api.tcp!.address() as any).address;
    expect(addr).toBe('127.0.0.1');
    expect(tcpAddr).toBe('127.0.0.1');
    expect(tcpPort).toBeGreaterThan(0);
  });
  it('server/index.ts guards every HTTP route, binds STUDIO_HOST and sets no wildcard CORS header', () => {
    const src = readFileSync(path.join(ROOT, 'server/index.ts'), 'utf8');
    const handler = src.slice(src.indexOf("httpServer.on('request'"));
    expect(handler.indexOf('guardHttp(')).toBeGreaterThan(0);
    expect(handler.indexOf('guardHttp(')).toBeLessThan(handler.indexOf('/python/run'));
    expect(handler.indexOf('guardHttp(')).toBeLessThan(handler.indexOf('handlePostsHttp'));
    expect(src).toMatch(/httpServer\.listen\(PORT, HOST/);
    for (const f of ['server/index.ts', 'server/posts_native.ts', 'server/vision.ts', 'server/vda5050.ts']) expect(readFileSync(path.join(ROOT, f), 'utf8')).not.toMatch(/allow-origin['"]?\s*[:,]\s*['"]\*/);
  });
});
