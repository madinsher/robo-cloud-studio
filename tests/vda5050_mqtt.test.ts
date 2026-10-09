/** End-to-end VDA 5050 over a real MQTT broker (aedes in-process) with the real mqtt.js client. */
import { describe, it, expect, beforeAll, afterAll } from 'vitest';
import net from 'node:net';
import { Vda5050Service, mqttConnector } from '../server/vda5050';
import { Station } from '../src/core/items/item';
import { MobileRobot } from '../src/mobile/items';
import { pathToOrder, robotToState } from '../src/fleet/vda5050';

let broker: any, server: net.Server, port = 0;
const wait = (ms: number) => new Promise((r) => setTimeout(r, ms));
const until = async (cond: () => boolean, ms = 5000) => { const t0 = Date.now(); while (!cond()) { if (Date.now() - t0 > ms) throw new Error('timeout'); await wait(20); } };

beforeAll(async () => {
  const { default: Aedes } = await import('aedes');
  broker = new (Aedes as any)();
  server = net.createServer(broker.handle);
  await new Promise<void>((r) => server.listen(0, '127.0.0.1', () => r()));
  port = (server.address() as net.AddressInfo).port;
});
afterAll(async () => { broker.close(); (server as any).closeAllConnections?.(); await new Promise<void>((r) => server.close(() => r())); });

describe('VDA 5050 over MQTT (aedes broker, mqtt.js client)', () => {
  it('twin publishes retained connection/factsheet, executes an order from a master client, master mirrors a foreign AGV', async () => {
    const connect = await mqttConnector();
    const st = new Station('Orchard');
    const r = new MobileRobot('Picker 1'); r.kin.maxSpeed = 3000; st.addChild(r);
    const svc = new Vda5050Service(() => st, connect, () => {});
    svc.connect({ url: `mqtt://127.0.0.1:${port}`, role: 'both', manufacturer: 'VB', stateHz: 20, simHz: 50, timeScale: 20 });
    await until(() => svc.connected);
    // an external master
    const mqtt = await import('mqtt');
    const master = mqtt.connect(`mqtt://127.0.0.1:${port}`);
    const got: Record<string, any[]> = { state: [], connection: [], factsheet: [], order: [] };
    master.on('message', (topic, payload) => { const type = topic.split('/').pop()!; (got[type] ??= []).push(JSON.parse(payload.toString())); });
    await new Promise<void>((r) => master.on('connect', () => r()));
    master.subscribe(['uagv/v2/VB/Picker_1/state', 'uagv/v2/VB/Picker_1/connection', 'uagv/v2/VB/Picker_1/factsheet'], { qos: 1 });
    await until(() => got.connection.length > 0 && got.factsheet.length > 0);
    expect(got.connection[0].connectionState).toBe('ONLINE');
    expect(got.factsheet[0].typeSpecification.seriesName).toBe('Picker 1');
    const order = pathToOrder({ headerId: 1, manufacturer: 'VB', serialNumber: 'Picker_1' }, [[0, 0], [1500, 0], [3000, 0]], { nodeSpacing: 1000 });
    master.publish('uagv/v2/VB/Picker_1/order', JSON.stringify(order), { qos: 1 });
    await until(() => got.state.some((s) => s.orderId === order.orderId && s.nodeStates.length === 0 && s.agvPosition.x > 2.5 && !s.driving), 8000);
    const done = got.state.filter((s) => s.orderId === order.orderId && !s.driving).pop()!;
    expect(done.nodeStates.length).toBe(0);
    expect(done.lastNodeId).toBe('n2');
    // foreign AGV talking to the studio master
    const agv = mqtt.connect(`mqtt://127.0.0.1:${port}`);
    await new Promise<void>((r) => agv.on('connect', () => r()));
    const orders: any[] = [];
    agv.on('message', (_t, p) => orders.push(JSON.parse(p.toString())));
    agv.subscribe('uagv/v2/MiR/MiR250-3/order', { qos: 1 });
    agv.publish('uagv/v2/MiR/MiR250-3/state', JSON.stringify(robotToState(new MobileRobot('m'), { header: { headerId: 1, manufacturer: 'MiR', serialNumber: 'MiR250-3' } })), { qos: 1 });
    await until(() => svc.master.agvs.has('MiR/MiR250-3'));
    svc.master.sendOrder('MiR', 'MiR250-3', [[0, 0], [4000, 0]]);
    await until(() => orders.length > 0);
    expect(orders[0].serialNumber).toBe('MiR250-3');
    expect(orders[0].nodes.length).toBe(2);
    svc.disconnect();
    await until(() => got.connection.some((c) => c.connectionState === 'OFFLINE'));
    master.end(true); agv.end(true);
  }, 30000);
});
