/**
 * VDA 5050 over MQTT for the studio server.
 *
 *  - Master role: connects to the fleet broker, mirrors every AGV that publishes state/connection/factsheet,
 *    sends orders / instant actions on request (HTTP API used by the browser UI and by scripts).
 *  - Bridge role: every mobile robot of the loaded station becomes a VDA 5050 AGV (digital twin). Orders received
 *    from a master (KUKA Fleet, Open-RMF, your dispatcher) drive the simulated robot; state / visualization /
 *    connection (with last-will OFFLINE) / factsheet are published like a real vehicle.
 *
 * HTTP API (JSON):
 *   GET  /vda5050/status                       connection, AGV summary, bridge twins
 *   GET  /vda5050/states                       raw last state message per AGV (for the browser shadow)
 *   POST /vda5050/connect   {url, prefix?, mapId?, role?: 'master'|'bridge'|'both', manufacturer?, username?, password?, stateHz?}
 *   POST /vda5050/disconnect
 *   POST /vda5050/order     {manufacturer, serial, path:[[x,y]...] (mm), task?, opts?}
 *   POST /vda5050/instantAction {manufacturer, serial, actionType, params?, blockingType?}
 *   POST /vda5050/link      {manufacturer, serial, robotId}
 */
import type { IncomingMessage, ServerResponse } from 'node:http';
import { Station, ItemType } from '../src/core/items/item';
import { MobileRobot } from '../src/mobile/items';
import { followPath, stepMobile } from '../src/mobile/controller';
import { Vda5050Master, Vda5050Agv, vdaTopic, parseVdaTopic, serialFor, VdaOrder, VdaInstantActions } from '../src/fleet/vda5050';

/** Minimal MQTT client surface (mqtt.js compatible) so tests can inject a fake broker. */
export interface MqttLike {
  publish(topic: string, payload: string, opts?: { retain?: boolean; qos?: 0 | 1 | 2 }): unknown;
  subscribe(topic: string | string[], opts?: unknown): unknown;
  on(event: 'message', cb: (topic: string, payload: Buffer | Uint8Array | string) => void): unknown;
  on(event: 'connect' | 'close' | 'error' | 'reconnect' | 'offline', cb: (...a: any[]) => void): unknown;
  end(force?: boolean, opts?: unknown, cb?: () => void): unknown;
  connected?: boolean;
}

export interface VdaConnectOptions {
  url: string;
  prefix?: string;
  mapId?: string;
  role?: 'master' | 'bridge' | 'both';
  /** Manufacturer used for bridge twins (topic segment). */
  manufacturer?: string;
  username?: string;
  password?: string;
  stateHz?: number;
  simHz?: number;
  /** Simulated seconds per real second for bridge twins (default 1; >1 for fast CI runs). */
  timeScale?: number;
}

export type MqttConnectFn = (url: string, opts: Record<string, unknown>) => MqttLike;

export class Vda5050Service {
  client: MqttLike | null = null;
  master: Vda5050Master;
  twins = new Map<string, Vda5050Agv>();
  options: VdaConnectOptions | null = null;
  connected = false;
  lastError = '';
  private timers: NodeJS.Timeout[] = [];
  private log: (m: string) => void;

  constructor(private getStation: () => Station, public connectFn: MqttConnectFn, log: (m: string) => void = (m) => console.log(`[vda5050] ${m}`)) {
    this.log = log;
    this.master = new Vda5050Master((topic, payload, retain) => this.client?.publish(topic, payload, { retain: !!retain, qos: 1 }), 'uagv', 'station');
  }

  status() {
    return {
      connected: this.connected,
      url: this.options?.url ?? null,
      prefix: this.master.prefix,
      role: this.options?.role ?? null,
      mapId: this.master.mapId,
      error: this.lastError || undefined,
      agvs: this.master.summary(),
      twins: [...this.twins.values()].map((t) => ({ manufacturer: t.manufacturer, serial: t.serial, robot: t.robot.name, robotId: t.robot.id, orderId: t.order?.orderId ?? '', lastNodeSequenceId: t.lastNodeSequenceId, status: t.robot.state.status, x: t.robot.state.x, y: t.robot.state.y, theta: t.robot.state.theta, battery: t.robot.batteryLevel() })),
    };
  }

  states() {
    return [...this.master.agvs.values()].filter((a) => a.lastState).map((a) => ({ manufacturer: a.manufacturer, serial: a.serial, robotId: a.robotId, state: a.lastState }));
  }

  connect(opts: VdaConnectOptions): void {
    this.disconnect();
    this.options = { role: 'master', prefix: 'uagv', mapId: 'station', manufacturer: 'VerticalBot', stateHz: 1, simHz: 10, timeScale: 1, ...opts };
    this.master.prefix = this.options.prefix!;
    this.master.mapId = this.options.mapId!;
    const role = this.options.role!;
    const bridge = role === 'bridge' || role === 'both';
    const manufacturer = this.options.manufacturer!;
    // bridge twins: every mobile robot in the station
    this.twins.clear();
    if (bridge) {
      for (const r of this.getStation().itemsOfType<MobileRobot>(ItemType.MOBILE_ROBOT)) {
        const serial = serialFor(r.name);
        this.twins.set(`${manufacturer}/${serial}`, new Vda5050Agv(r, manufacturer, serial, (robot, path) => followPath(robot, path), (robot) => { robot.state.path = null; robot.state.v = 0; }, this.master.mapId));
      }
    }
    const will = bridge && this.twins.size ? (() => { const t = [...this.twins.values()][0]; return { topic: vdaTopic(this.master.prefix, t.manufacturer, t.serial, 'connection'), payload: JSON.stringify(t.connection('CONNECTIONBROKEN')), retain: true, qos: 1 }; })() : undefined;
    const client = this.connectFn(this.options.url, { username: this.options.username, password: this.options.password, reconnectPeriod: 3000, clean: true, will });
    this.client = client;
    client.on('connect', () => {
      this.connected = true;
      this.lastError = '';
      this.log(`connected to ${this.options!.url} (${role})`);
      if (role === 'master' || role === 'both') {
        for (const t of ['state', 'connection', 'factsheet', 'visualization']) client.subscribe(`${this.master.prefix}/+/+/+/${t}`, { qos: 1 });
      }
      if (bridge) {
        for (const t of this.twins.values()) {
          client.subscribe(vdaTopic(this.master.prefix, t.manufacturer, t.serial, 'order'), { qos: 1 });
          client.subscribe(vdaTopic(this.master.prefix, t.manufacturer, t.serial, 'instantActions'), { qos: 1 });
          client.publish(vdaTopic(this.master.prefix, t.manufacturer, t.serial, 'connection'), JSON.stringify(t.connection('ONLINE')), { retain: true, qos: 1 });
          client.publish(vdaTopic(this.master.prefix, t.manufacturer, t.serial, 'factsheet'), JSON.stringify(t.factsheet()), { retain: true, qos: 1 });
        }
      }
    });
    client.on('error', (e: any) => { this.lastError = String(e?.message ?? e); this.log(`error ${this.lastError}`); });
    client.on('close', () => { this.connected = false; });
    client.on('message', (topic, payload) => this.onMessage(topic, typeof payload === 'string' ? payload : Buffer.from(payload).toString('utf8')));
    if (bridge) {
      const simDt = 1 / this.options.simHz!;
      const simStep = simDt * (this.options.timeScale ?? 1);
      this.timers.push(setInterval(() => { for (const t of this.twins.values()) { if (t.robot.state.path && !t.paused) stepMobile(t.robot, simStep); t.tick(); } }, simDt * 1000));
      this.timers.push(setInterval(() => { for (const t of this.twins.values()) this.publishTwin(t); }, 1000 / this.options.stateHz!));
      this.timers.push(setInterval(() => { for (const t of this.twins.values()) client.publish(vdaTopic(this.master.prefix, t.manufacturer, t.serial, 'visualization'), JSON.stringify(t.visualization()), { qos: 0 }); }, 200));
    }
  }

  publishTwin(t: Vda5050Agv): void {
    this.client?.publish(vdaTopic(this.master.prefix, t.manufacturer, t.serial, 'state'), JSON.stringify(t.state()), { qos: 1 });
  }

  /** Route an incoming MQTT message to the master (AGV telemetry) or to a bridge twin (orders / instant actions). */
  onMessage(topic: string, payload: string): void {
    const t = parseVdaTopic(topic);
    if (!t) return;
    const twin = this.twins.get(`${t.manufacturer}/${t.serial}`);
    if (twin && (t.type === 'order' || t.type === 'instantActions')) {
      try {
        const msg = JSON.parse(payload);
        if (t.type === 'order') {
          const err = twin.acceptOrder(msg as VdaOrder);
          if (err) { twin.errors = [{ errorType: 'orderError', errorLevel: 'WARNING', errorDescription: err, errorReferences: [{ referenceKey: 'orderId', referenceValue: String(msg.orderId) }] }]; this.log(`twin ${t.serial}: order rejected — ${err}`); }
          else this.log(`twin ${t.serial}: order ${msg.orderId} update ${msg.orderUpdateId}, ${msg.nodes?.length ?? 0} nodes`);
        } else twin.instantActions(msg as VdaInstantActions);
        this.publishTwin(twin);
      } catch (e) { this.log(`twin ${t.serial}: bad ${t.type} — ${(e as Error).message}`); }
      return;
    }
    if (t.type === 'state' || t.type === 'connection' || t.type === 'factsheet' || t.type === 'visualization') {
      if (this.twins.has(`${t.manufacturer}/${t.serial}`)) return; // our own twin telemetry
      this.master.receive(topic, payload);
    }
  }

  disconnect(): void {
    for (const tm of this.timers) clearInterval(tm);
    this.timers = [];
    if (this.client) {
      for (const t of this.twins.values()) { try { this.client.publish(vdaTopic(this.master.prefix, t.manufacturer, t.serial, 'connection'), JSON.stringify(t.connection('OFFLINE')), { retain: true, qos: 1 }); } catch { /* closing */ } }
      try { this.client.end(false); } catch { /* ignore */ }
    }
    this.client = null;
    this.connected = false;
  }

  /** HTTP handler; returns false when the URL is not a VDA endpoint. */
  async handleHttp(req: IncomingMessage, res: ServerResponse, readJson: () => Promise<any>): Promise<boolean> {
    const url = (req.url ?? '').split('?')[0];
    if (!url.startsWith('/vda5050/')) return false;
    const send = (code: number, body: unknown) => { res.writeHead(code, { 'content-type': 'application/json', 'access-control-allow-origin': '*' }); res.end(JSON.stringify(body)); };
    try {
      if (req.method === 'GET' && url === '/vda5050/status') return send(200, this.status()), true;
      if (req.method === 'GET' && url === '/vda5050/states') return send(200, this.states()), true;
      if (req.method !== 'POST') return send(405, { error: 'POST expected' }), true;
      const body = await readJson();
      switch (url) {
        case '/vda5050/connect': this.connect(body); return send(200, this.status()), true;
        case '/vda5050/disconnect': this.disconnect(); return send(200, this.status()), true;
        case '/vda5050/order': {
          if (!this.client) return send(409, { error: 'not connected' }), true;
          const o = this.master.sendOrder(body.manufacturer, body.serial, body.path, body.opts ?? {}, body.task);
          return send(200, o), true;
        }
        case '/vda5050/instantAction': {
          if (!this.client) return send(409, { error: 'not connected' }), true;
          return send(200, this.master.instantAction(body.manufacturer, body.serial, body.actionType, body.params ?? {}, body.blockingType ?? 'HARD')), true;
        }
        case '/vda5050/link': { const a = this.master.agv(body.manufacturer, body.serial); a.robotId = body.robotId; return send(200, a), true; }
        default: return send(404, { error: 'unknown VDA 5050 endpoint' }), true;
      }
    } catch (e) {
      return send(500, { error: (e as Error).message }), true;
    }
  }
}

/** Default connector using mqtt.js (loaded lazily so the module stays testable without a broker). */
export async function mqttConnector(): Promise<MqttConnectFn> {
  const mqtt = await import('mqtt');
  return (url, opts) => mqtt.connect(url, opts as any) as unknown as MqttLike;
}
