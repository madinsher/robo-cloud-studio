/**
 * Browser-side VDA 5050 client: talks to the studio server's /vda5050 HTTP API (the server holds the MQTT
 * connection), mirrors real AGVs onto station mobile robots (digital shadow) and dispatches fleet tasks
 * as VDA orders when the fleet manager assigns them.
 */
import type { App } from '../app';
import { MobileRobot } from '../mobile/items';
import { ItemType } from '../core/items/item';
import { applyStateToRobot, serialFor, VdaState } from './vda5050';
import { FleetItem } from './fleet';

export interface VdaClientOptions {
  url: string;
  prefix?: string;
  mapId?: string;
  role?: 'master' | 'bridge' | 'both';
  manufacturer?: string;
  username?: string;
  password?: string;
  /** Mirror AGV states onto robots with the same serial (name). Default true. */
  shadow?: boolean;
  /** Send fleet tasks as orders when the fleet manager assigns them to a linked robot. Default true. */
  autoDispatch?: boolean;
  pollMs?: number;
}

export class VdaClient {
  status: any = null;
  private timer: number | null = null;
  private sentTasks = new Map<string, string>(); // robotId -> taskId already dispatched
  private lastLinkedSerials = new Set<string>();
  constructor(private app: App, private base: string, public opts: VdaClientOptions) {}

  private async post(path: string, body: unknown): Promise<any> {
    const r = await fetch(`${this.base}/vda5050/${path}`, { method: 'POST', headers: { 'content-type': 'application/json' }, body: JSON.stringify(body) });
    if (!r.ok) throw new Error(`${path}: ${r.status} ${await r.text()}`);
    return r.json();
  }
  private async get(path: string): Promise<any> {
    const r = await fetch(`${this.base}/vda5050/${path}`);
    if (!r.ok) throw new Error(`${path}: ${r.status}`);
    return r.json();
  }

  async connect(): Promise<any> {
    this.status = await this.post('connect', { url: this.opts.url, prefix: this.opts.prefix, mapId: this.opts.mapId, role: this.opts.role ?? 'master', manufacturer: this.opts.manufacturer, username: this.opts.username, password: this.opts.password });
    this.start();
    return this.status;
  }
  async disconnect(): Promise<void> {
    this.stop();
    try { await this.post('disconnect', {}); } catch { /* server gone */ }
  }
  start(): void {
    this.stop();
    this.timer = window.setInterval(() => this.poll().catch((e) => this.app.log(`VDA 5050: ${e.message}`, 'warn')), this.opts.pollMs ?? 1000);
  }
  stop(): void { if (this.timer) { clearInterval(this.timer); this.timer = null; } }

  /** Robots that map to AGVs: same serial (sanitized name) or an explicit `vdaSerial` param. */
  linkedRobots(): Array<{ robot: MobileRobot; manufacturer: string; serial: string }> {
    const out: Array<{ robot: MobileRobot; manufacturer: string; serial: string }> = [];
    const agvs: any[] = this.status?.agvs ?? [];
    for (const r of this.app.station.itemsOfType<MobileRobot>(ItemType.MOBILE_ROBOT)) {
      const serial = String(r.getParam('vdaSerial') ?? serialFor(r.name));
      const agv = agvs.find((a) => a.serial === serial || a.robotId === r.id);
      if (agv) out.push({ robot: r, manufacturer: agv.manufacturer, serial: agv.serial });
    }
    return out;
  }

  async poll(): Promise<void> {
    this.status = await this.get('status');
    if (!this.status?.connected) return;
    const linked = this.linkedRobots();
    // tell the server which robot each AGV mirrors (once per pairing)
    for (const l of linked) {
      const k = `${l.manufacturer}/${l.serial}`;
      if (!this.lastLinkedSerials.has(k)) { this.lastLinkedSerials.add(k); this.post('link', { manufacturer: l.manufacturer, serial: l.serial, robotId: l.robot.id }).catch(() => {}); }
    }
    if (this.opts.shadow !== false && linked.length) {
      const states: Array<{ manufacturer: string; serial: string; state: VdaState }> = await this.get('states');
      for (const s of states) {
        const l = linked.find((x) => x.serial === s.serial && x.manufacturer === s.manufacturer);
        if (l && s.state) applyStateToRobot(s.state, l.robot);
      }
    }
    if (this.opts.autoDispatch !== false) await this.dispatchAssigned(linked);
  }

  /** Fleet tasks newly assigned to linked robots become VDA orders (path = the planned route). */
  private async dispatchAssigned(linked: Array<{ robot: MobileRobot; manufacturer: string; serial: string }>): Promise<void> {
    const fleets = this.app.station.itemsOfType<FleetItem>(ItemType.FLEET);
    for (const l of linked) {
      const taskId = l.robot.state.taskId;
      if (!taskId || this.sentTasks.get(l.robot.id) === taskId) continue;
      const task = fleets.flatMap((f) => f.tasks).find((t) => t.id === taskId);
      const path = l.robot.state.path ?? [[l.robot.state.x, l.robot.state.y], task?.location ?? [l.robot.state.x, l.robot.state.y]];
      if (!task) continue;
      this.sentTasks.set(l.robot.id, taskId);
      try {
        const o = await this.post('order', { manufacturer: l.manufacturer, serial: l.serial, path, task, opts: { mapId: this.opts.mapId } });
        this.app.log(`VDA 5050: order ${o.orderId} (${o.nodes.length} nodes) → ${l.serial} for task ${task.type} ${task.id}`);
      } catch (e) { this.app.log(`VDA 5050 order failed: ${(e as Error).message}`, 'warn'); }
    }
  }

  sendOrder(manufacturer: string, serial: string, path: number[][], opts: Record<string, unknown> = {}) { return this.post('order', { manufacturer, serial, path, opts }); }
  instantAction(manufacturer: string, serial: string, actionType: string, params: Record<string, unknown> = {}) { return this.post('instantAction', { manufacturer, serial, actionType, params }); }
}
