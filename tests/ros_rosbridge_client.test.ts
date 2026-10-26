/**
 * R5: rosbridge v2 transport of the simulator backend (src/ros/rosbridge_client.ts) and of the browser RosBridge
 * 'sim' mode, against a fake rosbridge_server: the Python json dialect (bare Infinity / NaN — rosbridge rejects
 * JSON null for float fields), advertise-before-publish with the latch flag, typed subscriptions, ROS services
 * served by the studio (advertise_service -> call_service with an id -> service_response with the same id), service
 * calls made by the studio, and the browser RosBridge choosing the MURKA interface on a MUROM station.
 */
import { describe, it, expect, afterEach } from 'vitest';
import { RosbridgeClient, rosJsonStringify, rosJsonParse } from '../src/ros/rosbridge_client';
import { RosBridge } from '../src/ros/rosbridge';
import { FakeRosbridge, until } from './ros_fake_rosbridge';
import { HAS_MURKA } from './webots_helpers';
import { buildRetailStation } from './ros_murka_helpers';

const fakes: FakeRosbridge[] = [];
afterEach(async () => { while (fakes.length) await fakes.pop()!.close(); });
async function pair() { const fake = await FakeRosbridge.start(); fakes.push(fake); const c = new RosbridgeClient(fake.url); await c.connect(); return { fake, c }; }

describe('rosbridge JSON dialect', () => {
  it('writes non-finite floats as the bare tokens Python json reads, leaves strings alone, and parses them back', () => {
    const s = rosJsonStringify({ ranges: [1.5, Infinity, -Infinity, NaN], note: 'Infinity NaN', n: { v: 0 } });
    expect(s).toBe('{"ranges":[1.5,Infinity,-Infinity,NaN],"note":"Infinity NaN","n":{"v":0}}');
    const back = rosJsonParse(s);
    expect(back.ranges[0]).toBe(1.5); expect(back.ranges[1]).toBe(Infinity); expect(back.ranges[2]).toBe(-Infinity); expect(Number.isNaN(back.ranges[3])).toBe(true);
    expect(back.note).toBe('Infinity NaN');
    // what rosbridge (Python json.dumps) sends: tokens after ':' / ',' / '[' and escaped quotes inside strings
    const py = '{"op": "publish", "topic": "/x", "msg": {"data": [NaN, -Infinity, 2.0], "s": "a \\"NaN\\" b", "z": Infinity}}';
    const m = rosJsonParse(py);
    expect(Number.isNaN(m.msg.data[0])).toBe(true);
    expect(m.msg.data.slice(1)).toEqual([-Infinity, 2]);
    expect(m.msg.s).toBe('a "NaN" b');
    expect(m.msg.z).toBe(Infinity);
    expect(rosJsonStringify({ a: [1, 2] })).toBe(JSON.stringify({ a: [1, 2] }));
  });
});

describe('RosbridgeClient (fake rosbridge)', () => {
  it('advertises once (latch = transient local) before publishing, subscribes with the type, delivers publishes', async () => {
    const { fake, c } = await pair();
    const got: any[] = [];
    c.subscribe('/cmd_vel', 'geometry_msgs/msg/Twist', (m) => got.push(m));
    c.publish('/robot_description', 'std_msgs/msg/String', { data: '<robot/>' }, { latch: true });
    c.publish('/scan', 'sensor_msgs/msg/LaserScan', { ranges: [1, Infinity] });
    c.publish('/scan', 'sensor_msgs/msg/LaserScan', { ranges: [2, Infinity] });
    await until(() => fake.ops.length >= 6);
    expect(fake.ops.map((o) => o.op)).toEqual(['subscribe', 'advertise', 'publish', 'advertise', 'publish', 'publish']);
    expect(fake.ops[0]).toMatchObject({ topic: '/cmd_vel', type: 'geometry_msgs/msg/Twist' });
    expect(fake.ops[1]).toMatchObject({ topic: '/robot_description', type: 'std_msgs/msg/String', latch: true });
    expect(fake.ops[3].latch).toBeUndefined();
    expect(fake.raws('/scan')).toEqual(['{"op":"publish","topic":"/scan","msg":{"ranges":[1,Infinity]}}', '{"op":"publish","topic":"/scan","msg":{"ranges":[2,Infinity]}}']);
    fake.send('/cmd_vel', { linear: { x: 0.2 } });
    await until(() => got.length === 1);
    expect(got[0].linear.x).toBe(0.2);
    c.close();
  });

  it('serves ROS services: call_service id -> service_response with the same id and values; a failing handler answers result=false', async () => {
    const { fake, c } = await pair();
    c.advertiseService('/murom/sim/attach', 'murom_msgs/srv/AttachObject', (req) => ({ success: req.arm === 'left', message: `got ${req.object_name}` }));
    c.advertiseService('/boom', 'std_srvs/srv/Trigger', () => { throw new Error('nope'); });
    c.advertiseService('/slow', 'std_srvs/srv/Trigger', async () => { await new Promise((r) => setTimeout(r, 30)); return { success: true, message: 'late' }; });
    await until(() => fake.services().size === 3);
    expect(Object.fromEntries(fake.services())).toEqual({ '/murom/sim/attach': 'murom_msgs/srv/AttachObject', '/boom': 'std_srvs/srv/Trigger', '/slow': 'std_srvs/srv/Trigger' });
    const [a, b, s] = await Promise.all([fake.call('/murom/sim/attach', { object_name: 'tea_box_1', arm: 'left' }), fake.call('/boom', {}), fake.call('/slow', {})]);
    expect(a).toMatchObject({ op: 'service_response', service: '/murom/sim/attach', result: true, values: { success: true, message: 'got tea_box_1' } });
    expect(a.id).toBe('service_request:/murom/sim/attach:1');
    expect(b).toMatchObject({ service: '/boom', result: false });
    expect(s.values).toEqual({ success: true, message: 'late' });
    c.close();
  });

  it('calls ROS services through rosbridge and resolves with the response values', async () => {
    const { fake, c } = await pair();
    const p = c.callService('/murom/task/mission', { mission: 'scan', args: [] }, 'murom_msgs/srv/StartMission');
    await until(() => fake.ops.some((o) => o.op === 'call_service'));
    const op = fake.ops.find((o) => o.op === 'call_service')!;
    expect(op).toMatchObject({ service: '/murom/task/mission', type: 'murom_msgs/srv/StartMission', args: { mission: 'scan', args: [] } } as never);
    (fake as unknown as { sock: { send(s: string): void } }).sock.send(JSON.stringify({ op: 'service_response', service: '/murom/task/mission', id: op.id, values: { accepted: true }, result: true }));
    await expect(p).resolves.toEqual({ accepted: true });
    c.close();
  });
});

describe.skipIf(!HAS_MURKA)('browser RosBridge mode "sim" on the MURKA retail station', () => {
  it('runs the MURKA interface (not the generic twin): MURKA services served, /scan with Infinity tokens, /clock advancing', async () => {
    const { station, assets } = buildRetailStation();
    const { setSensingAssets } = await import('../src/vision/sensing');
    setSensingAssets(assets);
    const fake = await FakeRosbridge.start(); fakes.push(fake);
    const g = globalThis as unknown as { window?: unknown };
    const hadWindow = 'window' in g; if (!hadWindow) g.window = globalThis;
    const logs: string[] = [];
    const app = { station, simTime: 0, log: (m: string) => logs.push(m) } as never;
    const rb = new RosBridge(app, { url: fake.url, mode: 'sim', murom: { cameraRate: 0, lidarNoise: false } });
    try {
      await rb.connect();
      expect(rb.murom).not.toBeNull();
      expect(rb.twin).toBeNull();
      await until(() => fake.published('/scan').length >= 1 && fake.published('/clock').length >= 5, 15000);
      expect(fake.raws('/scan')[0]).toMatch(/Infinity/);
      const res = await fake.call('/murom/sim/attach', { object_name: 'tea_box_1', arm: 'right' });
      expect(res.values.success).toBe(false);
      expect(res.values.message).toMatch(/^Ладонь слишком далеко от tea_box_1/);
      const clocks = fake.published('/clock').map((m) => m.clock.sec + m.clock.nanosec * 1e-9);
      for (let i = 1; i < clocks.length; i++) expect(clocks[i] - clocks[i - 1]).toBeCloseTo(0.032, 9);
    } finally {
      rb.disconnect();
      if (!hadWindow) delete g.window;
    }
  }, 120000);
});
