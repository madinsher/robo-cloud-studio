/**
 * rosbridge v2 protocol client (rosbridge_suite websocket) usable in the browser and in Node (headless simulator
 * backend, tests): publish / subscribe / advertise (latched = transient local) / call_service with a promise /
 * advertise_service (rosbridge forwards ROS service calls as `call_service` ops, the client answers with
 * `service_response`). JSON wire format with the two rosbridge peculiarities handled:
 *  - non-finite floats (LaserScan +inf beyond range, NaN) are written as the bare tokens `Infinity` / `-Infinity` /
 *    `NaN`, which rosbridge's JSON backends accept (Python json; ujson 5.9 checked) — JSON.stringify would write `null`, and rosbridge
 *    rejects `None` for a float field);
 *  - rosbridge (Python json.dumps) sends the same tokens back; they are parsed instead of failing the whole message.
 * uint8[] fields (sensor_msgs/Image data) are sent as base64 strings (rosbridge decodes them).
 */
export interface RosServiceHandler { (args: any): any | Promise<any> }

/** What a simulator interface needs from the transport. */
export interface RosTransport {
  publish(topic: string, type: string, msg: any, opts?: { latch?: boolean }): void;
  subscribe(topic: string, type: string, cb: (msg: any) => void): void;
  advertiseService?(service: string, type: string, handler: RosServiceHandler): void;
}

type WsLike = { readyState: number; send(data: string): void; close(): void; onopen: ((ev: any) => void) | null; onclose: ((ev: any) => void) | null; onerror: ((ev: any) => void) | null; onmessage: ((ev: { data: any }) => void) | null };
export type WebSocketCtor = new (url: string) => WsLike;

const NONFINITE = '\u0000NF:';
/** JSON with `Infinity` / `-Infinity` / `NaN` tokens for non-finite numbers (rosbridge / Python json dialect). */
export function rosJsonStringify(o: unknown): string {
  let found = false;
  const s = JSON.stringify(o, (_k, v) => {
    if (typeof v === 'number' && !Number.isFinite(v)) { found = true; return NONFINITE + (Number.isNaN(v) ? 'NaN' : v > 0 ? 'Infinity' : '-Infinity'); }
    return v;
  });
  return found ? s.replace(/"\\u0000NF:(NaN|Infinity|-Infinity)"/g, '$1') : s;
}

/** JSON.parse that also accepts the bare `NaN` / `Infinity` / `-Infinity` tokens Python's json module writes. */
export function rosJsonParse(text: string): any {
  try { return JSON.parse(text); } catch (e) {
    if (!/NaN|Infinity/.test(text)) throw e;
    // replace the tokens outside strings
    let out = '', inStr = false;
    for (let i = 0; i < text.length; i++) {
      const c = text[i];
      if (inStr) { out += c; if (c === '\\') { out += text[++i] ?? ''; } else if (c === '"') inStr = false; continue; }
      if (c === '"') { inStr = true; out += c; continue; }
      // the sentinel goes in as a JSON escape (a raw control character is invalid inside a JSON string)
      if (text.startsWith('NaN', i)) { out += '"\\u0000NF:NaN"'; i += 2; continue; }
      if (text.startsWith('-Infinity', i)) { out += '"\\u0000NF:-Infinity"'; i += 8; continue; }
      if (text.startsWith('Infinity', i)) { out += '"\\u0000NF:Infinity"'; i += 7; continue; }
      out += c;
    }
    return JSON.parse(out, (_k, v) => (typeof v === 'string' && v.startsWith(NONFINITE) ? (v.endsWith('NaN') ? NaN : v.endsWith('-Infinity') ? -Infinity : Infinity) : v));
  }
}

export class RosbridgeClient implements RosTransport {
  ws: WsLike | null = null;
  connected = false;
  /** Bytes / messages sent (diagnostics). */
  stats = { sent: 0, bytes: 0, received: 0, serviceCalls: 0 };
  readonly log: (msg: string, level?: 'info' | 'warn' | 'error') => void;
  private subs = new Map<string, Array<(msg: any) => void>>();
  private subTypes = new Map<string, string>();
  private advertised = new Map<string, { type: string; latch: boolean }>();
  private services = new Map<string, { type: string; handler: RosServiceHandler }>();
  private pending = new Map<string, { resolve: (v: any) => void; reject: (e: Error) => void }>();
  private nextId = 1;

  constructor(readonly url: string, readonly opts: { WebSocket?: WebSocketCtor; log?: (msg: string, level?: 'info' | 'warn' | 'error') => void } = {}) {
    this.log = opts.log ?? (() => {});
  }

  connect(timeoutMs = 10000): Promise<void> {
    const Ctor: WebSocketCtor | undefined = this.opts.WebSocket ?? (globalThis as unknown as { WebSocket?: WebSocketCtor }).WebSocket;
    if (!Ctor) return Promise.reject(new Error('no WebSocket implementation (pass opts.WebSocket)'));
    return new Promise((resolve, reject) => {
      let settled = false;
      const ws = new Ctor(this.url);
      this.ws = ws;
      const timer = setTimeout(() => { if (!settled) { settled = true; try { ws.close(); } catch { /* ignore */ } reject(new Error(`rosbridge connection timed out: ${this.url}`)); } }, timeoutMs);
      ws.onopen = () => {
        this.connected = true;
        // re-announce everything (reconnect)
        for (const [topic, a] of this.advertised) this.raw(a.latch ? { op: 'advertise', topic, type: a.type, latch: true } : { op: 'advertise', topic, type: a.type });
        for (const [topic, type] of this.subTypes) this.raw({ op: 'subscribe', topic, type });
        for (const [service, s] of this.services) this.raw({ op: 'advertise_service', service, type: s.type });
        if (!settled) { settled = true; clearTimeout(timer); resolve(); }
      };
      ws.onerror = () => { if (!settled) { settled = true; clearTimeout(timer); reject(new Error(`rosbridge connection failed: ${this.url}`)); } };
      ws.onclose = () => { this.connected = false; for (const p of this.pending.values()) p.reject(new Error('rosbridge closed')); this.pending.clear(); };
      ws.onmessage = (ev) => this.onMessage(typeof ev.data === 'string' ? ev.data : String(ev.data));
    });
  }

  close(): void { try { this.ws?.close(); } catch { /* ignore */ } this.connected = false; }

  private raw(o: unknown): void {
    if (!this.ws || this.ws.readyState !== 1) return;
    const s = rosJsonStringify(o);
    this.stats.sent++; this.stats.bytes += s.length;
    this.ws.send(s);
  }

  private onMessage(text: string): void {
    let m: any;
    try { m = rosJsonParse(text); } catch { this.log(`rosbridge: unparsable message (${text.slice(0, 80)})`, 'warn'); return; }
    this.stats.received++;
    if (m.op === 'publish') { for (const cb of this.subs.get(m.topic) ?? []) { try { cb(m.msg); } catch (e) { this.log(`subscriber of ${m.topic} failed: ${(e as Error).message}`, 'error'); } } }
    else if (m.op === 'call_service') void this.serve(m);
    else if (m.op === 'service_response') {
      const p = this.pending.get(m.id);
      if (p) { this.pending.delete(m.id); if (m.result === false) p.reject(new Error(typeof m.values === 'string' ? m.values : JSON.stringify(m.values))); else p.resolve(m.values); }
    } else if (m.op === 'status') this.log(`rosbridge ${m.level ?? 'status'}: ${m.msg}`, m.level === 'error' ? 'error' : 'warn');
  }

  private async serve(m: { id?: string; service: string; args?: any }): Promise<void> {
    const s = this.services.get(m.service);
    if (!s) return;
    this.stats.serviceCalls++;
    try {
      const values = await s.handler(m.args ?? {});
      this.raw({ op: 'service_response', service: m.service, id: m.id, values: values ?? {}, result: true });
    } catch (e) {
      this.log(`service ${m.service} failed: ${(e as Error).message}`, 'error');
      this.raw({ op: 'service_response', service: m.service, id: m.id, values: {}, result: false });
    }
  }

  advertise(topic: string, type: string, latch = false): void {
    const a = this.advertised.get(topic);
    if (a && a.type === type && a.latch === latch) return;
    this.advertised.set(topic, { type, latch });
    this.raw(latch ? { op: 'advertise', topic, type, latch: true } : { op: 'advertise', topic, type });
  }

  publish(topic: string, type: string, msg: any, opts?: { latch?: boolean }): void {
    if (!this.advertised.has(topic)) this.advertise(topic, type, !!opts?.latch);
    this.raw({ op: 'publish', topic, msg });
  }

  subscribe(topic: string, type: string, cb: (msg: any) => void): void {
    const list = this.subs.get(topic) ?? [];
    list.push(cb);
    this.subs.set(topic, list);
    if (this.subTypes.get(topic) === type) return;
    this.subTypes.set(topic, type);
    this.raw({ op: 'subscribe', topic, type, queue_length: 10 });
  }

  advertiseService(service: string, type: string, handler: RosServiceHandler): void {
    this.services.set(service, { type, handler });
    this.raw({ op: 'advertise_service', service, type });
  }

  /** Call a ROS service through rosbridge; resolves with the response values. */
  callService(service: string, args: any = {}, type?: string, timeoutMs = 10000): Promise<any> {
    const id = `vbs_call:${service}:${this.nextId++}`;
    return new Promise((resolve, reject) => {
      const timer = setTimeout(() => { if (this.pending.delete(id)) reject(new Error(`service ${service} timed out`)); }, timeoutMs);
      this.pending.set(id, { resolve: (v) => { clearTimeout(timer); resolve(v); }, reject: (e) => { clearTimeout(timer); reject(e); } });
      this.raw(type ? { op: 'call_service', id, service, type, args } : { op: 'call_service', id, service, args });
    });
  }
}
