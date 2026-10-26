/**
 * A fake rosbridge_server (v2 protocol subset) for contract tests: records every op a client sends (advertise,
 * publish, subscribe, advertise_service, service_response), parses the raw text with the Python json dialect
 * (bare Infinity / NaN accepted, like rosbridge), can publish to the client's subscriptions and call the services the
 * client advertised (call_service with an id, awaiting the matching service_response), and answer the client's own
 * service calls for services registered in `answers` (rosapi: /rosapi/subscribers); other calls get no answer.
 */
import { WebSocketServer, WebSocket as WsClient } from 'ws';
import { rosJsonParse } from '../src/ros/rosbridge_client';

export interface FakeOp { op: string; topic?: string; service?: string; type?: string; msg?: any; latch?: boolean; id?: string; values?: any; result?: boolean; raw: string; t: number }

export class FakeRosbridge {
  readonly ops: FakeOp[] = [];
  readonly url: string;
  private sock: WsClient | null = null;
  private pending = new Map<string, (op: FakeOp) => void>();
  private nextId = 1;
  /** Services this fake answers when the client calls them (handler result = response values; undefined = result false). */
  readonly answers = new Map<string, (args: any) => any>();
  private constructor(private wss: WebSocketServer, port: number) { this.url = `ws://127.0.0.1:${port}`; }

  static async start(): Promise<FakeRosbridge> {
    const wss = new WebSocketServer({ port: 0, host: '127.0.0.1' });
    await new Promise<void>((r) => wss.once('listening', () => r()));
    const fake = new FakeRosbridge(wss, (wss.address() as { port: number }).port);
    wss.on('connection', (s) => {
      fake.sock = s;
      s.on('message', (data) => {
        const raw = String(data);
        const m = rosJsonParse(raw);
        const op: FakeOp = { ...m, raw, t: Date.now() };
        fake.ops.push(op);
        if (m.op === 'service_response') { const cb = fake.pending.get(m.id); if (cb) { fake.pending.delete(m.id); cb(op); } }
        if (m.op === 'call_service' && fake.answers.has(m.service)) {
          const values = fake.answers.get(m.service)!(m.args ?? {});
          s.send(JSON.stringify(values === undefined ? { op: 'service_response', service: m.service, id: m.id, values: `Service ${m.service} does not exist`, result: false } : { op: 'service_response', service: m.service, id: m.id, values, result: true }));
        }
      });
    });
    return fake;
  }

  /** Every `publish` op of a topic (messages in order). */
  published(topic: string): any[] { return this.ops.filter((o) => o.op === 'publish' && o.topic === topic).map((o) => o.msg); }
  last(topic: string): any { const p = this.published(topic); return p[p.length - 1]; }
  raws(topic: string): string[] { return this.ops.filter((o) => o.op === 'publish' && o.topic === topic).map((o) => o.raw); }
  advertised(): Map<string, { type: string; latch: boolean; index: number }> {
    const m = new Map<string, { type: string; latch: boolean; index: number }>();
    this.ops.forEach((o, i) => { if (o.op === 'advertise' && o.topic && !m.has(o.topic)) m.set(o.topic, { type: o.type!, latch: !!o.latch, index: i }); });
    return m;
  }
  subscriptions(): Map<string, string> { const m = new Map<string, string>(); for (const o of this.ops) if (o.op === 'subscribe' && o.topic) m.set(o.topic, o.type!); return m; }
  services(): Map<string, string> { const m = new Map<string, string>(); for (const o of this.ops) if (o.op === 'advertise_service' && o.service) m.set(o.service, o.type!); return m; }

  /** rosbridge -> client: a message on a topic the client subscribed. */
  send(topic: string, msg: any): void { this.sock?.send(JSON.stringify({ op: 'publish', topic, msg })); }
  /** rosbridge -> client: a ROS service call on a service the client advertised; resolves with the response op. */
  call(service: string, args: any): Promise<FakeOp> {
    const id = `service_request:${service}:${this.nextId++}`;
    return new Promise((resolve, reject) => {
      const timer = setTimeout(() => reject(new Error(`no service_response for ${id}`)), 5000);
      this.pending.set(id, (op) => { clearTimeout(timer); resolve(op); });
      this.sock?.send(JSON.stringify({ op: 'call_service', id, service, args }));
    });
  }
  async close(): Promise<void> { for (const c of this.wss.clients) c.terminate(); await new Promise<void>((r) => this.wss.close(() => r())); }
}

/** Wait until `pred` holds (event loop keeps running: websocket traffic is delivered). */
export async function until(pred: () => boolean, timeoutMs = 5000): Promise<void> {
  const t0 = Date.now();
  while (!pred()) { if (Date.now() - t0 > timeoutMs) throw new Error('timeout'); await new Promise((r) => setTimeout(r, 5)); }
}
