/**
 * Access control of the studio server (server/index.ts, src/api/api_server.ts). The server executes code (station Python programs on CPython, native RoboDK
 * posts, user post sources, the whole RoboDK API including RunProgram), so like RoboDK's own API port it only serves
 * this machine by default:
 *
 *  - bind address: STUDIO_HOST (default 127.0.0.1). Binding another address (e.g. 0.0.0.0 for WSL / LAN clients) is
 *    an explicit opt-in, equivalent to RoboDK's "allow remote API connections".
 *  - browser origins: a request that carries an Origin header (every cross-site fetch, form POST and WebSocket opened
 *    by a web page) is served only when that origin is a local one (http(s)://localhost / 127.0.0.1 / [::1] /
 *    *.localhost, any port: the studio page served by vite) or is listed in STUDIO_ALLOWED_ORIGINS (comma separated;
 *    "*" disables the check). Clients that are not browsers (Python, MATLAB, C#, curl) send no Origin, or a local one.
 *  - Host header (DNS rebinding): the Host must be a loopback name, an IP literal, or listed in STUDIO_ALLOWED_HOSTS.
 *  - optional shared secret STUDIO_TOKEN: when set, HTTP requests need `x-studio-token: <token>` (or
 *    `Authorization: Bearer <token>`, or `?token=<token>`), WebSocket upgrades need `?token=<token>`.
 *  - CORS: only an allowed origin is echoed in access-control-allow-origin (never `*`).
 */
import type http from 'node:http';
import { isIP } from 'node:net';

export interface SecurityOptions {
  /** Extra allowed browser origins ("*" = any). */
  allowedOrigins?: string[];
  /** Extra allowed Host header names (DNS names the server is reached by). */
  allowedHosts?: string[];
  /** Shared secret required on every request when set. */
  token?: string;
}

export function securityFromEnv(env: NodeJS.ProcessEnv = process.env): SecurityOptions {
  const list = (v?: string) => (v ?? '').split(',').map((s) => s.trim()).filter(Boolean);
  return { allowedOrigins: list(env.STUDIO_ALLOWED_ORIGINS), allowedHosts: list(env.STUDIO_ALLOWED_HOSTS), token: env.STUDIO_TOKEN || undefined };
}

/** Default bind address: loopback only unless STUDIO_HOST says otherwise. */
export function bindHost(env: NodeJS.ProcessEnv = process.env): string {
  return env.STUDIO_HOST?.trim() || '127.0.0.1';
}

const LOOPBACK_NAMES = new Set(['localhost', '127.0.0.1', '::1', '[::1]']);
export function isLoopbackHostname(h: string): boolean {
  const n = h.toLowerCase().replace(/\.$/, '');
  return LOOPBACK_NAMES.has(n) || n.endsWith('.localhost') || /^127\.\d+\.\d+\.\d+$/.test(n) || n === '::ffff:127.0.0.1';
}
export function isLoopbackAddress(addr: string | undefined | null): boolean {
  if (!addr) return false;
  return addr === '::1' || /^127\./.test(addr) || /^::ffff:127\./.test(addr);
}

/** Whether a browser Origin header may use the server (undefined / absent = not a browser page: allowed). */
export function originAllowed(origin: string | undefined, opts: SecurityOptions = {}): boolean {
  if (origin === undefined || origin === '') return true;
  const extra = opts.allowedOrigins ?? [];
  if (extra.includes('*')) return true;
  if (extra.some((o) => o.replace(/\/$/, '').toLowerCase() === origin.replace(/\/$/, '').toLowerCase())) return true;
  // "null" (sandboxed iframes, file:// pages, data: URLs) is never trusted implicitly
  let u: URL;
  try { u = new URL(origin); } catch { return false; }
  if (u.protocol !== 'http:' && u.protocol !== 'https:') return false;
  return isLoopbackHostname(u.hostname.replace(/^\[|\]$/g, ''));
}

/** Host header check against DNS rebinding: loopback names and IP literals are fine, other names must be listed. */
export function hostAllowed(hostHeader: string | undefined, opts: SecurityOptions = {}): boolean {
  if (!hostHeader) return true;   // HTTP/1.0 clients
  let name = hostHeader.trim().toLowerCase();
  if (name.startsWith('[')) name = name.slice(1, name.indexOf(']'));
  else if (name.includes(':') && name.indexOf(':') === name.lastIndexOf(':')) name = name.slice(0, name.indexOf(':'));
  if (isLoopbackHostname(name) || isIP(name)) return true;
  return (opts.allowedHosts ?? []).some((h) => h.toLowerCase() === name);
}

function tokenOf(req: http.IncomingMessage): string | null {
  const h = req.headers['x-studio-token'];
  if (typeof h === 'string' && h) return h;
  const auth = req.headers.authorization;
  if (typeof auth === 'string' && /^bearer\s+/i.test(auth)) return auth.replace(/^bearer\s+/i, '').trim();
  try { return new URL(req.url ?? '/', 'http://x').searchParams.get('token'); } catch { return null; }
}

/** Why a request must be refused (null = allowed). Applies to HTTP requests and WebSocket upgrades alike. */
export function rejectReason(req: http.IncomingMessage, opts: SecurityOptions = {}, ws = false): string | null {
  const origin = typeof req.headers.origin === 'string' ? req.headers.origin : undefined;
  if (!hostAllowed(typeof req.headers.host === 'string' ? req.headers.host : undefined, opts)) return `host ${req.headers.host} not allowed (set STUDIO_ALLOWED_HOSTS)`;
  if (!originAllowed(origin, opts)) return `origin ${origin} not allowed (set STUDIO_ALLOWED_ORIGINS)`;
  if (opts.token) {
    const t = ws ? (() => { try { return new URL(req.url ?? '/', 'http://x').searchParams.get('token'); } catch { return null; } })() ?? tokenOf(req) : tokenOf(req);
    if (t !== opts.token) return 'missing or wrong STUDIO_TOKEN';
  }
  return null;
}

/** CORS headers for an allowed request: the origin is echoed (never "*"), Vary: Origin. */
export function corsHeaders(req: http.IncomingMessage): Record<string, string> {
  const origin = typeof req.headers.origin === 'string' ? req.headers.origin : '';
  return origin ? { 'access-control-allow-origin': origin, vary: 'Origin', 'access-control-allow-headers': 'content-type, x-studio-token, authorization', 'access-control-allow-methods': 'GET,POST,OPTIONS' } : {};
}

/**
 * Guard an HTTP request: refused requests get 403 (and are fully answered here); allowed ones get the CORS headers set
 * on `res`. Returns true when the request may proceed.
 */
export function guardHttp(req: http.IncomingMessage, res: http.ServerResponse, opts: SecurityOptions = {}): boolean {
  const why = rejectReason(req, opts);
  if (why) {
    res.writeHead(403, { 'content-type': 'application/json' });
    res.end(JSON.stringify({ error: `forbidden: ${why}` }));
    return false;
  }
  for (const [k, v] of Object.entries(corsHeaders(req))) res.setHeader(k, v);
  return true;
}

/** `verifyClient` for ws.WebSocketServer: refuses upgrades from foreign web pages (cross-site WebSocket hijacking). */
export function wsVerifyClient(opts: SecurityOptions = {}, log?: (m: string) => void) {
  return (info: { origin: string; req: http.IncomingMessage }, cb: (ok: boolean, code?: number, msg?: string) => void): void => {
    const why = rejectReason(info.req, opts, true);
    if (why) { log?.(`WebSocket refused: ${why}`); cb(false, 403, 'Forbidden'); } else cb(true);
  };
}
