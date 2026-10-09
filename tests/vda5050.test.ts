import { describe, it, expect } from 'vitest';
import { vdaTopic, parseVdaTopic, pathToOrder, taskToOrder, actionsForTask, robotToState, applyStateToRobot, Vda5050Master, Vda5050Agv, serialFor, MM } from '../src/fleet/vda5050';
import { Vda5050Service, MqttLike } from '../server/vda5050';
import { MobileRobot } from '../src/mobile/items';
import { Station } from '../src/core/items/item';
import { followPath, stepMobile } from '../src/mobile/controller';
import type { FleetTask } from '../src/fleet/fleet';

function task(partial: Partial<FleetTask> = {}): FleetTask {
  return { id: 't1', type: 'transport', location: [5000, 2000], priority: 1, requiredCapabilities: [], status: 'assigned', robotId: null, createdAt: 0, meta: {}, ...partial } as FleetTask;
}

/** In-memory MQTT broker shared by fake clients (topic wildcards + and #). */
class FakeBroker {
  clients: FakeClient[] = [];
  retained = new Map<string, string>();
  publish(topic: string, payload: string, retain?: boolean) {
    if (retain) this.retained.set(topic, payload);
    for (const c of this.clients) for (const s of c.subs) if (match(s, topic)) c.deliver(topic, payload);
  }
}
function match(filter: string, topic: string): boolean {
  const f = filter.split('/'), t = topic.split('/');
  for (let i = 0; i < f.length; i++) { if (f[i] === '#') return true; if (f[i] !== '+' && f[i] !== t[i]) return false; }
  return f.length === t.length;
}
class FakeClient implements MqttLike {
  subs: string[] = [];
  handlers: Record<string, Array<(...a: any[]) => void>> = {};
  published: Array<{ topic: string; payload: string; retain?: boolean }> = [];
  connected = false;
  constructor(public broker: FakeBroker) { broker.clients.push(this); setTimeout(() => { this.connected = true; (this.handlers.connect ?? []).forEach((h) => h()); }, 0); }
  publish(topic: string, payload: string, opts?: { retain?: boolean }) { this.published.push({ topic, payload, retain: opts?.retain }); this.broker.publish(topic, payload, opts?.retain); }
  subscribe(topic: string | string[]) { for (const t of Array.isArray(topic) ? topic : [topic]) this.subs.push(t); }
  on(event: string, cb: (...a: any[]) => void) { (this.handlers[event] ??= []).push(cb); }
  end() { this.connected = false; this.broker.clients = this.broker.clients.filter((c) => c !== this); }
  deliver(topic: string, payload: string) { (this.handlers.message ?? []).forEach((h) => h(topic, Buffer.from(payload))); }
}
const tick = (ms = 5) => new Promise((r) => setTimeout(r, ms));

describe('VDA 5050 messages', () => {
  it('builds and parses topics', () => {
    const t = vdaTopic('uagv', 'KUKA', 'KMP600-01', 'order');
    expect(t).toBe('uagv/v2/KUKA/KMP600-01/order');
    expect(parseVdaTopic(t)).toEqual({ prefix: 'uagv', protocol: 'v2', manufacturer: 'KUKA', serial: 'KMP600-01', type: 'order' });
    expect(parseVdaTopic('foo/bar')).toBeNull();
    expect(serialFor('Harvester #2 (north)')).toBe('Harvester_2_north_');
  });
  it('converts a mm path into a metre/radian order with base/horizon and task actions', () => {
    const path = [[0, 0], [1000, 0], [2000, 0], [3000, 0], [3000, 1000], [3000, 2000]];
    const o = taskToOrder({ headerId: 1, manufacturer: 'VB', serialNumber: 'r1' }, task({ type: 'charge' }), path, { nodeSpacing: 1000, baseNodes: 3, maxSpeed: 1.2 });
    expect(o.version).toBe('2.0.0');
    expect(o.nodes.length).toBe(6);
    expect(o.edges.length).toBe(5);
    expect(o.nodes[0].released && o.nodes[2].released && !o.nodes[3].released).toBe(true);
    expect(o.nodes[5].nodePosition).toMatchObject({ x: 3, y: 2, mapId: 'station' });
    expect(o.nodes[5].actions[0].actionType).toBe('startCharging');
    expect(o.edges[0].length).toBeCloseTo(1, 6);
    expect(o.nodes.map((n) => n.sequenceId)).toEqual([0, 2, 4, 6, 8, 10]);
    expect(actionsForTask(task({ type: 'harvest', duration: 30 }))[0]).toMatchObject({ actionType: 'harvest', blockingType: 'HARD' });
    expect(actionsForTask(task({ type: 'transport', payloadKg: 12 }))[0].actionParameters).toContainEqual({ key: 'loadMass', value: 12 });
  });
  it('round-trips a robot state through robotToState / applyStateToRobot', () => {
    const r = new MobileRobot('AMR 1');
    r.state.x = 1234; r.state.y = -500; r.state.theta = 90; r.state.v = 800; r.state.status = 'moving';
    r.battery.levelWh = r.battery.capacityWh * 0.42;
    const st = robotToState(r, { header: { headerId: 7, manufacturer: 'VB', serialNumber: 'AMR_1' } });
    expect(st.agvPosition).toMatchObject({ x: 1.234, y: -0.5, mapId: 'station', positionInitialized: true });
    expect(st.agvPosition!.theta).toBeCloseTo(Math.PI / 2, 9);
    expect(st.batteryState.batteryCharge).toBeCloseTo(42, 6);
    expect(st.driving).toBe(true);
    const r2 = new MobileRobot('AMR 1');
    const res = applyStateToRobot(st, r2);
    expect(res.moved).toBe(true);
    expect([r2.state.x, r2.state.y]).toEqual([1234, -500]);
    expect(r2.state.theta).toBeCloseTo(90, 6);
    expect(r2.state.status).toBe('moving');
    expect(r2.batteryLevel()).toBeCloseTo(0.42, 6);
  });
});

describe('VDA 5050 master bookkeeping', () => {
  it('numbers headers, issues order updates while an order runs, and detects completion', () => {
    const sent: Array<{ topic: string; payload: any }> = [];
    const m = new Vda5050Master((topic, payload) => sent.push({ topic, payload: JSON.parse(payload) }));
    const o1 = m.sendOrder('KUKA', 'kmp1', [[0, 0], [5000, 0]], { nodeSpacing: 1000 });
    expect(sent[0].topic).toBe('uagv/v2/KUKA/kmp1/order');
    expect(o1.headerId).toBe(1);
    expect(o1.orderUpdateId).toBe(0);
    // AGV reports progress on that order -> next send is an update of the same order
    m.receive('uagv/v2/KUKA/kmp1/state', JSON.stringify({ ...robotToState(new MobileRobot('kmp1'), { header: { headerId: 1, manufacturer: 'KUKA', serialNumber: 'kmp1' }, order: o1, lastNodeSequenceId: 0 }) }));
    const o2 = m.sendOrder('KUKA', 'kmp1', [[2000, 0], [5000, 0], [5000, 3000]], { nodeSpacing: 1000 });
    expect(o2.orderId).toBe(o1.orderId);
    expect(o2.orderUpdateId).toBe(1);
    expect(o2.headerId).toBe(2);
    const a = m.agv('KUKA', 'kmp1');
    expect(m.orderDone(a)).toBe(false);
    m.receive('uagv/v2/KUKA/kmp1/state', JSON.stringify(robotToState(new MobileRobot('kmp1'), { header: { headerId: 2, manufacturer: 'KUKA', serialNumber: 'kmp1' }, order: o2, lastNodeSequenceId: 999 })));
    expect(m.orderDone(a)).toBe(true);
    m.cancelOrder('KUKA', 'kmp1');
    expect(sent[sent.length - 1].topic).toBe('uagv/v2/KUKA/kmp1/instantActions');
    expect(sent[sent.length - 1].payload.actions[0].actionType).toBe('cancelOrder');
    expect(m.summary()[0]).toMatchObject({ manufacturer: 'KUKA', serial: 'kmp1', remainingNodes: 0 });
  });
});

describe('VDA 5050 AGV twin', () => {
  it('drives a simulated robot along an order, tracks nodes and actions, handles instant actions', () => {
    const r = new MobileRobot('twin');
    r.kin.maxSpeed = 2000;
    const agv = new Vda5050Agv(r, 'VB', 'twin', (robot, path) => followPath(robot, path), (robot) => { robot.state.path = null; });
    const order = taskToOrder({ headerId: 1, manufacturer: 'VB', serialNumber: 'twin' }, task({ type: 'charge' }), [[0, 0], [3000, 0], [3000, 3000]], { nodeSpacing: 1000 });
    expect(agv.acceptOrder(order)).toBeNull();
    expect(agv.acceptOrder(order)).toMatch(/not newer/);
    expect(r.state.path).not.toBeNull();
    for (let i = 0; i < 400 && r.state.path; i++) { stepMobile(r, 0.05); agv.tick(); }
    agv.tick();
    const st = agv.state();
    expect(st.nodeStates.length).toBe(0);
    expect(st.lastNodeId).toBe(order.nodes[order.nodes.length - 1].nodeId);
    expect(st.actionStates.find((a) => a.actionType === 'startCharging')?.actionStatus).toBe('RUNNING');
    expect(r.state.status).toBe('charging');
    expect(Math.hypot(r.state.x - 3000, r.state.y - 3000)).toBeLessThan(400);
    const res = agv.instantActions({ headerId: 2, timestamp: '', version: '2.0.0', manufacturer: 'VB', serialNumber: 'twin', actions: [{ actionId: 'ip', actionType: 'initPosition', blockingType: 'HARD', actionParameters: [{ key: 'x', value: 1 }, { key: 'y', value: 2 }, { key: 'theta', value: Math.PI }, { key: 'mapId', value: 'station' }] }, { actionId: 'c', actionType: 'cancelOrder', blockingType: 'HARD' }] });
    expect(res.map((a) => a.actionStatus)).toEqual(['FINISHED', 'FINISHED']);
    expect([r.state.x, r.state.y]).toEqual([1000, 2000]);
    expect(agv.order).toBeNull();
    expect(agv.factsheet().typeSpecification.agvKinematic).toBe('DIFF');
  });
});

describe('VDA 5050 service over a fake broker', () => {
  it('bridge twins answer a master: order in, state out; master mirrors a foreign AGV', async () => {
    const broker = new FakeBroker();
    const st = new Station('Farm');
    const r = new MobileRobot('Sprayer 1'); r.kin.maxSpeed = 3000; st.addChild(r);
    const svc = new Vda5050Service(() => st, () => new FakeClient(broker), () => {});
    svc.connect({ url: 'mqtt://fake', role: 'both', manufacturer: 'VB', stateHz: 50, simHz: 50, timeScale: 20 });
    await tick(10);
    expect(svc.connected).toBe(true);
    expect(svc.status().twins[0]).toMatchObject({ serial: 'Sprayer_1', robot: 'Sprayer 1' });
    expect(broker.retained.get('uagv/v2/VB/Sprayer_1/connection')).toContain('ONLINE');
    expect(broker.retained.has('uagv/v2/VB/Sprayer_1/factsheet')).toBe(true);
    // an external master (another client) sends an order to the twin
    const master = new FakeClient(broker);
    await tick(2);
    master.subscribe('uagv/v2/VB/Sprayer_1/state');
    const received: any[] = [];
    master.on('message', (_t: string, p: Buffer) => received.push(JSON.parse(p.toString())));
    const order = pathToOrder({ headerId: 1, manufacturer: 'VB', serialNumber: 'Sprayer_1' }, [[0, 0], [2000, 0]], { nodeSpacing: 1000 });
    master.publish('uagv/v2/VB/Sprayer_1/order', JSON.stringify(order));
    await tick(400);
    expect(received.length).toBeGreaterThan(3);
    const last = received[received.length - 1];
    expect(last.orderId).toBe(order.orderId);
    expect(last.agvPosition.x).toBeGreaterThan(1.5);
    expect(last.nodeStates.length).toBe(0);
    // a foreign AGV publishes state: the master side mirrors it and can address it
    const agvClient = new FakeClient(broker);
    await tick(2);
    agvClient.publish('uagv/v2/KUKA/KMP600-7/connection', JSON.stringify({ headerId: 1, timestamp: '', version: '2.0.0', manufacturer: 'KUKA', serialNumber: 'KMP600-7', connectionState: 'ONLINE' }), { retain: true });
    agvClient.publish('uagv/v2/KUKA/KMP600-7/state', JSON.stringify(robotToState(new MobileRobot('x'), { header: { headerId: 1, manufacturer: 'KUKA', serialNumber: 'KMP600-7' } })));
    await tick(5);
    const s = svc.status();
    const kmp = s.agvs.find((a: any) => a.serial === 'KMP600-7');
    expect(kmp).toMatchObject({ connection: 'ONLINE', manufacturer: 'KUKA' });
    agvClient.subscribe('uagv/v2/KUKA/KMP600-7/order');
    const got: any[] = [];
    agvClient.on('message', (_t: string, p: Buffer) => got.push(JSON.parse(p.toString())));
    svc.master.sendOrder('KUKA', 'KMP600-7', [[0, 0], [10000, 0]], {}, task({ type: 'spray' }));
    await tick(2);
    expect(got[0].nodes[got[0].nodes.length - 1].actions[0].actionType).toBe('spray');
    expect(svc.states().some((x: any) => x.serial === 'KMP600-7')).toBe(true);
    svc.disconnect();
    expect(broker.retained.get('uagv/v2/VB/Sprayer_1/connection')).toContain('OFFLINE');
  });
});
