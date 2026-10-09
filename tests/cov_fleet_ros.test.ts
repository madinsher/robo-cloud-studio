/**
 * Coverage tests for the fleet / ROS bridges: the browser-side VDA 5050 client (HTTP API of the studio server,
 * stubbed with `fetch`), the fleet manager's remaining paths (serialization, cancellation, round-robin allocation,
 * charging zones) and the rosbridge client speaking the rosbridge JSON protocol to a real `ws` server.
 */
import { describe, it, expect, vi, beforeAll, afterAll } from 'vitest';
import { WebSocketServer, WebSocket as WsSocket } from 'ws';
import { Station, Camera, ItemType } from '../src/core/items/item';
import { MobileRobot, MapItem, ZoneItem } from '../src/mobile/items';
import { FleetItem, FleetManager } from '../src/fleet/fleet';
import { VdaClient } from '../src/fleet/vda_client';
import { robotToState, VdaState } from '../src/fleet/vda5050';
import { RosBridge } from '../src/ros/rosbridge';
import { publishVisionOutput, publishNavRuntime, poseArrayMsg, odometryMsg } from '../src/ros/publishers';
import { setNavStack, stepNavRuntime, NavRuntime } from '../src/mobile/navstack';
import { setVisionStack } from '../src/vision/stack';
import { ensureVisionRuntime, visionRuntimeOf, PipelineOutput } from '../src/vision/pipeline';
import { intrinsicsFromCamera } from '../src/vision/camera_model';
import { createRobotFromLibrary } from '../src/core/items/library';
import { buildMechanism } from '../src/core/kinematics/mechanism';
import { transl, DEG, RAD } from '../src/core/math/pose';
import { MAKE_ROBOT_1T } from '../src/api/robolink';

const wait = (ms: number) => new Promise((r) => setTimeout(r, ms));
const until = async (cond: () => boolean, ms = 5000) => { const t0 = Date.now(); while (!cond()) { if (Date.now() - t0 > ms) throw new Error('timeout waiting for condition'); await wait(10); } };

beforeAll(() => {
  // the browser-only modules schedule with window.setInterval
  vi.stubGlobal('window', { setInterval: (fn: () => void, ms: number) => setInterval(fn, ms), clearInterval: (t: any) => clearInterval(t) });
});
afterAll(() => { vi.unstubAllGlobals(); });

function fakeApp(station: Station) {
  const logs: Array<[string, string | undefined]> = [];
  return { station, logs, log: (text: string, level?: string) => { logs.push([text, level]); } } as any;
}

// ---------------------------------------------------------------------------------------------
// VDA 5050 browser client
// ---------------------------------------------------------------------------------------------

describe('VdaClient (studio server /vda5050 HTTP API)', () => {
  function setup() {
    const st = new Station('Fleet');
    const r1 = st.addChild(new MobileRobot('AMR 1'));
    const r2 = st.addChild(new MobileRobot('Second'));
    r2.setParam('vdaSerial', 'KMP-7');
    const r3 = st.addChild(new MobileRobot('By id'));
    st.addChild(new MobileRobot('Unlinked'));
    const fleetItem = st.addChild(new FleetItem('Fleet'));
    const fm = new FleetManager(st, fleetItem);
    fm.addRobot(r1); fm.addRobot(r3);
    const task = fm.addTask({ type: 'transport', location: [5000, 0] });
    r1.state.taskId = task.id;
    r1.state.path = [[0, 0], [2500, 0], [5000, 0]];
    const app = fakeApp(st);
    // stubbed studio server
    const calls: Array<{ method: string; path: string; body?: any }> = [];
    let connected = true;
    let failStatus = false;
    let failOrderFor: string | null = null;
    const shadow = new MobileRobot('shadow');
    shadow.setPose2D(2000, 3000, 90);
    shadow.state.status = 'moving';
    shadow.battery.levelWh = shadow.battery.capacityWh / 2;
    const shadowState: VdaState = robotToState(shadow, { header: { headerId: 1, manufacturer: 'VB', serialNumber: 'AMR_1' } });
    const respond = (data: any, ok = true, status = 200) => ({ ok, status, json: async () => data, text: async () => (ok ? '' : 'server says no') });
    const fetchMock = vi.fn(async (url: string, init?: RequestInit) => {
      const path = url.replace(/^.*\/vda5050\//, '');
      const body = init?.body ? JSON.parse(String(init.body)) : undefined;
      calls.push({ method: init?.method ?? 'GET', path, body });
      if (path === 'status') return failStatus ? respond(null, false, 500) : respond({ connected, agvs: [{ manufacturer: 'VB', serial: 'AMR_1' }, { manufacturer: 'KUKA', serial: 'KMP-7' }, { manufacturer: 'X', serial: 'other', robotId: r3.id }] });
      if (path === 'states') return respond([{ manufacturer: 'VB', serial: 'AMR_1', state: shadowState }, { manufacturer: 'KUKA', serial: 'KMP-7', state: null }, { manufacturer: 'ZZ', serial: 'nobody', state: shadowState }]);
      if (path === 'order') return body.serial === failOrderFor ? respond(null, false, 500) : respond({ orderId: `o_${body.serial}`, nodes: body.path.map((_: any, i: number) => ({ nodeId: `n${i}` })) });
      if (path === 'connect') return respond({ connected: true, url: body.url, role: body.role, agvs: [] });
      if (path === 'disconnect') return connected ? respond({ ok: true }) : respond(null, false, 503);
      return respond({ echo: body });
    });
    vi.stubGlobal('fetch', fetchMock);
    return { st, r1, r2, r3, fm, task, app, calls, fetchMock, shadowState, setConnected: (v: boolean) => { connected = v; }, setFailStatus: (v: boolean) => { failStatus = v; }, setFailOrderFor: (s: string | null) => { failOrderFor = s; } };
  }

  it('connects, links robots to AGVs, mirrors states and dispatches assigned tasks as orders once', async () => {
    const s = setup();
    const client = new VdaClient(s.app, 'http://studio.local:8765', { url: 'mqtt://broker:1883', prefix: 'uagv', mapId: 'hall', manufacturer: 'VB', pollMs: 60_000 });
    const status = await client.connect();
    expect(status).toMatchObject({ connected: true, url: 'mqtt://broker:1883', role: 'master' });
    expect(s.calls[0]).toMatchObject({ method: 'POST', path: 'connect', body: { prefix: 'uagv', mapId: 'hall', manufacturer: 'VB' } });
    client.stop();
    await client.poll();
    const linked = client.linkedRobots();
    expect(linked.map((l) => `${l.manufacturer}/${l.serial}`).sort()).toEqual(['KUKA/KMP-7', 'VB/AMR_1', 'X/other']);
    expect(linked.find((l) => l.serial === 'other')!.robot).toBe(s.r3);
    const links = s.calls.filter((c) => c.path === 'link');
    expect(links.length).toBe(3);
    expect(links.find((l) => l.body.serial === 'AMR_1')!.body.robotId).toBe(s.r1.id);
    // digital shadow: AMR 1 follows the state of the real AGV (metres -> mm, rad -> deg)
    expect(s.r1.state.x).toBeCloseTo(2000, 3);
    expect(s.r1.state.y).toBeCloseTo(3000, 3);
    expect(s.r1.state.theta).toBeCloseTo(90, 6);
    expect(s.r1.state.status).toBe('moving');
    expect(s.r1.batteryLevel()).toBeCloseTo(0.5, 3);
    expect((s.r1.getParam('vda5050') as any).headerId).toBe(1);
    expect(s.r2.state.x).toBe(0);
    // order for the assigned task, with the planned path
    const orders = s.calls.filter((c) => c.path === 'order');
    expect(orders.length).toBe(1);
    expect(orders[0].body).toMatchObject({ manufacturer: 'VB', serial: 'AMR_1', path: [[0, 0], [2500, 0], [5000, 0]], task: { id: s.task.id, type: 'transport' }, opts: { mapId: 'hall' } });
    expect(s.app.logs.some(([t]: [string]) => t.includes(`order o_AMR_1 (3 nodes)`) && t.includes(s.task.id))).toBe(true);
    // second poll: pairing and dispatch are not repeated
    await client.poll();
    expect(s.calls.filter((c) => c.path === 'link').length).toBe(3);
    expect(s.calls.filter((c) => c.path === 'order').length).toBe(1);
    // a task id unknown to the fleet is skipped; a failing order is logged as a warning and not retried
    s.r2.state.taskId = 'ghost';
    const t2 = s.fm.addTask({ type: 'goto', location: [1000, 1000] });
    s.r3.state.taskId = t2.id;
    s.r3.state.path = null;
    s.setFailOrderFor('other');
    await client.poll();
    const ordersNow = s.calls.filter((c) => c.path === 'order');
    expect(ordersNow.length).toBe(2);
    expect(ordersNow[1].body.serial).toBe('other');
    expect(ordersNow[1].body.path).toEqual([[s.r3.state.x, s.r3.state.y], [1000, 1000]]);
    expect(s.app.logs.some(([t, l]: [string, string]) => /order failed/.test(t) && l === 'warn')).toBe(true);
    // direct commands
    const order = await client.sendOrder('KUKA', 'KMP-7', [[0, 0], [1000, 0]], { nodeSpacing: 500 });
    expect(order.orderId).toBe('o_KMP-7');
    expect(s.calls[s.calls.length - 1].body.opts).toEqual({ nodeSpacing: 500 });
    const ia = await client.instantAction('KUKA', 'KMP-7', 'startPause', { reason: 'test' });
    expect(ia.echo).toMatchObject({ actionType: 'startPause', params: { reason: 'test' } });
    // disconnected server: poll returns early, errors propagate from get/post
    s.setConnected(false);
    const before = s.calls.length;
    await client.poll();
    expect(s.calls.length - before).toBe(1);
    s.setFailStatus(true);
    await expect(client.poll()).rejects.toThrow(/status: 500/);
    s.setFailStatus(false);
    s.setFailOrderFor('KMP-7');
    await expect(client.sendOrder('KUKA', 'KMP-7', [[0, 0]])).rejects.toThrow(/order: 500 server says no/);
    await client.disconnect(); // POST disconnect fails with 503 here and is swallowed
    expect(s.calls[s.calls.length - 1].path).toBe('disconnect');
  });

  it('polls on a timer, logs poll failures and honours shadow / autoDispatch switches', async () => {
    const s = setup();
    const client = new VdaClient(s.app, 'http://studio.local:8765', { url: 'mqtt://broker', pollMs: 5, shadow: false, autoDispatch: false });
    client.start();
    await until(() => s.calls.filter((c) => c.path === 'status').length >= 2);
    s.setFailStatus(true);
    await until(() => s.app.logs.some(([t, l]: [string, string]) => t.startsWith('VDA 5050: status: 500') && l === 'warn'));
    client.stop();
    client.stop();
    s.setFailStatus(false);
    const n = s.calls.length;
    await wait(30);
    expect(s.calls.length).toBe(n);
    expect(s.calls.some((c) => c.path === 'states')).toBe(false);
    expect(s.calls.some((c) => c.path === 'order')).toBe(false);
    expect(s.r1.state.x).toBe(0);
    await client.disconnect();
    expect(s.calls[s.calls.length - 1]).toMatchObject({ method: 'POST', path: 'disconnect' });
    expect(new VdaClient(s.app, 'http://x', { url: 'mqtt://y' }).linkedRobots()).toEqual([]);
  });
});

// ---------------------------------------------------------------------------------------------
// Fleet manager: remaining paths
// ---------------------------------------------------------------------------------------------

describe('FleetManager: serialization, cancellation, round robin, charging zones', () => {
  it('round-trips the fleet item through the station file and cancels tasks', () => {
    const st = new Station('S');
    const r1 = st.addChild(new MobileRobot('A'));
    const r2 = st.addChild(new MobileRobot('B'));
    r2.setPose2D(20000, 0, 0);
    const fi = st.addChild(new FleetItem('Fleet'));
    fi.allocation = 'round_robin';
    fi.segmentCapacity = 2;
    fi.chargingZoneIds = ['z1'];
    const fm = new FleetManager(st, fi);
    fm.addRobot(r1); fm.addRobot(r2);
    fm.addRobot(r2);
    expect(fi.robotIds).toEqual([r1.id, r2.id]);
    const t1 = fm.addTask({ type: 'goto', location: [5000, 0] });
    const t2 = fm.addTask({ type: 'goto', location: [25000, 0] });
    fm.step(0.1);
    // round robin: the first robot that can take the task gets it, regardless of distance
    expect(t1.robotId).toBe(r1.id);
    expect(t2.robotId).toBe(r2.id);
    expect(r1.state.status).toBe('moving');
    expect(fm.getLog().some((e) => /Task .* -> A/.test(e.text))).toBe(true);
    expect(fm.pending).toBeInstanceOf(Map);
    fm.cancelTask(t1.id);
    expect(t1.status).toBe('cancelled');
    expect(r1.state.taskId).toBeNull();
    expect(r1.state.path).toBeNull();
    expect(r1.state.status).toBe('idle');
    fm.cancelTask('does-not-exist');
    const unassigned = fm.addTask({ type: 'goto', location: [1, 1], requiredCapabilities: ['fly'] });
    fm.cancelTask(unassigned.id);
    expect(unassigned.status).toBe('cancelled');
    const data = st.serialize();
    const copy = Station.deserialize(JSON.parse(JSON.stringify(data)));
    const fi2 = copy.itemsOfType<FleetItem>(ItemType.FLEET)[0];
    expect(fi2).toBeInstanceOf(FleetItem);
    expect(fi2.allocation).toBe('round_robin');
    expect(fi2.segmentCapacity).toBe(2);
    expect(fi2.chargingZoneIds).toEqual(['z1']);
    expect(fi2.robotIds).toEqual([r1.id, r2.id]);
    expect(fi2.tasks.map((t) => t.status)).toEqual(['cancelled', 'travelling', 'cancelled']);
    const fresh = new FleetItem('F');
    fresh.deserializeExtra({} as any);
    expect([fresh.allocation, fresh.segmentCapacity, fresh.mapId, fresh.tasks, fresh.robotIds]).toEqual(['auction', 1, null, [], []]);
    const fm2 = new FleetManager(copy, fi2);
    expect(fm2.robots().map((r) => r.name)).toEqual(['A', 'B']);
    expect(fm2.kpis().tasksPending).toBe(0);
  });

  it('sends a low-battery robot without a home to the charging zone and back to idle when charged', () => {
    const st = new Station('S');
    const r = st.addChild(new MobileRobot('Low'));
    r.battery.levelWh = 100; // 5 %
    r.battery.chargeW = 5e6; // charge almost instantly once docked
    r.kin.maxSpeed = 3000;
    const zone = st.addChild(new ZoneItem('Charger'));
    zone.kind = 'charging';
    zone.setPose(transl(1000, 0, 0));
    zone.polygon = [[3000, -500], [4000, -500], [4000, 500], [3000, 500]];
    const fi = st.addChild(new FleetItem('Fleet'));
    const fm = new FleetManager(st, fi);
    fm.addRobot(r);
    fm.step(0.1);
    const charge = fi.tasks.find((t) => t.type === 'charge')!;
    expect(charge).toBeTruthy();
    expect(charge.location[0]).toBeCloseTo(4500);
    expect(charge.location[1]).toBeCloseTo(0);
    expect(charge.robotId).toBe(r.id);
    expect(r.state.status).toBe('moving');
    let n = 0;
    while (r.state.status !== 'charging' && n++ < 2000) fm.step(0.05);
    expect(r.state.status).toBe('charging');
    expect(charge.status).toBe('done');
    expect(Math.hypot(r.state.x - 4500, r.state.y)).toBeLessThan(300);
    n = 0;
    while (r.state.status === 'charging' && n++ < 100) fm.step(1);
    expect(r.state.status).toBe('idle');
    expect(r.batteryLevel()).toBeGreaterThanOrEqual(0.95);
    expect(fm.getLog().some((e) => /charged/.test(e.text))).toBe(true);
    const k = fm.kpis();
    expect(k.tasksDone).toBe(0); // charge tasks are not counted as work
    expect(k.perRobot[r.id].status).toBe('idle');
  });
});

// ---------------------------------------------------------------------------------------------
// rosbridge client
// ---------------------------------------------------------------------------------------------

describe('RosBridge (rosbridge JSON protocol over WebSocket)', () => {
  let wss: WebSocketServer;
  let port = 0;
  const sockets: WsSocket[] = [];
  const inbox = new Map<WsSocket, any[]>();
  beforeAll(async () => {
    wss = new WebSocketServer({ port: 0, host: '127.0.0.1' });
    await new Promise<void>((r) => wss.on('listening', () => r()));
    port = (wss.address() as any).port;
    wss.on('connection', (s) => { sockets.push(s); inbox.set(s, []); s.on('message', (d) => inbox.get(s)!.push(JSON.parse(d.toString()))); });
  });
  afterAll(async () => { for (const s of sockets) s.terminate(); await new Promise<void>((r) => wss.close(() => r())); });
  const lastSocket = () => sockets[sockets.length - 1];

  function fakeOutput(cam: Camera): PipelineOutput {
    return {
      frame: { width: cam.width, height: cam.height, K: intrinsicsFromCamera(cam), camWorld: cam.poseAbs(), time: 1.5, dataUrl: 'data:image/jpeg;base64,QUJD' },
      results: {},
      detections: [{ x: 10, y: 20, w: 30, h: 40, score: 0.9, cls: 'box', id: 7, z: 1000, p: [100, 200, 300] }],
      targets: [{ name: 'box 1', cls: 'box', grasp: transl(100, 200, 300), approach: transl(100, 200, 200), p: [100, 200, 300], z: 1000, sigmaMm: 5 }],
      text: 'one box on the table',
      warnings: [],
    };
  }

  it('publishes arm, mobile, navigation and vision topics at the configured rate', async () => {
    const st = new Station('Cell');
    const arm = st.addChild(createRobotFromLibrary('UR5e', 'Arm'));
    arm.connection.rosNamespace = '/arm/';
    arm.setJoints([10, -90, 90, -90, -90, 0]);
    const amr = st.addChild(new MobileRobot('AMR 1'));
    amr.rosNamespace = '/amr1';
    amr.setPose2D(1000, 2000, 90);
    amr.state.v = 500; amr.state.omega = 10;
    const map = st.addChild(new MapItem('Map'));
    map.resize(100, 100, 100, -5000, -5000);
    map.fillRect(3000, -5000, 3200, 5000);
    setNavStack(amr, { platform: 'amr', environment: 'warehouse', sensors: ['lidar2d', 'wheel_odom'], localization: 'slam_toolbox_2d', navigation: 'nav2_navfn_dwb', fusion: ['wheel_odom'], simulate: true });
    const rt = stepNavRuntime(amr, 0.6, map, [])!;
    expect(rt.lastScan).not.toBeNull();
    const cam = amr.addChild(new Camera('Cam A'));
    const cfg: any = { tasks: ['detect'], modality: 'mono', environment: 'warehouse', sensor: 'usb_rgb', compute: 'jetson_orin_nano', models: {}, classes: ['box'], confidence: 0.4, iou: 0.5, runtime: 'simulated', workingDistance: 1, simulate: true, publishRos: true };
    setVisionStack(cam, cfg);
    const vrt = ensureVisionRuntime(cam)!;
    expect(visionRuntimeOf(cam)).toBe(vrt);
    vrt.last = fakeOutput(cam);
    const silent = st.addChild(new Camera('Silent'));
    setVisionStack(silent, { ...cfg, publishRos: false });
    ensureVisionRuntime(silent)!.last = fakeOutput(silent);
    const app = fakeApp(st);
    const bridge = new RosBridge(app, { url: `ws://127.0.0.1:${port}`, rate: 50, mode: 'publish' });
    bridge.publish('/early', 'std_msgs/msg/String', { data: 'dropped: not connected yet' });
    await bridge.connect();
    expect(bridge.connected).toBe(true);
    expect(app.logs[0][0]).toMatch(/rosbridge connected/);
    const sock = lastSocket();
    const msgs = () => inbox.get(sock)!;
    await until(() => msgs().filter((m) => m.op === 'publish' && m.topic === '/arm/tcp_pose').length >= 3);
    // publish mode never subscribes; every topic is advertised exactly once with its type
    expect(msgs().some((m) => m.op === 'subscribe')).toBe(false);
    expect(msgs().some((m) => m.topic === '/early')).toBe(false);
    const adverts = msgs().filter((m) => m.op === 'advertise');
    const topics = adverts.map((a) => a.topic);
    expect(new Set(topics).size).toBe(topics.length);
    expect(adverts.find((a) => a.topic === '/arm/cmd_joint_state').type).toBe('sensor_msgs/msg/JointState');
    expect(adverts.find((a) => a.topic === '/amr1/cmd_vel').type).toBe('geometry_msgs/msg/Twist');
    expect(adverts.find((a) => a.topic === '/amr1/odom_estimate').type).toBe('nav_msgs/msg/Odometry');
    expect(adverts.find((a) => a.topic === '/amr1/scan').type).toBe('sensor_msgs/msg/LaserScan');
    // arm: joints in radians, TCP in metres
    const js = msgs().find((m) => m.op === 'publish' && m.topic === '/arm/cmd_joint_state').msg;
    expect(js.name).toEqual(arm.jointNames());
    expect(js.position[0]).toBeCloseTo(10 * DEG, 9);
    expect(js.header.frame_id).toBe('Arm');
    const tcp = msgs().find((m) => m.op === 'publish' && m.topic === '/arm/tcp_pose').msg;
    const fk = arm.solveFK();
    expect(tcp.pose.position.z).toBeCloseTo(fk[14] / 1000, 6);
    const pt = msgs().find((m) => m.op === 'publish' && m.topic === '/arm/cmd_point').msg;
    expect(pt.data[0]).toBeCloseTo(fk[12] / 1000, 6);
    // mobile robot: velocities, pose, battery
    const vel = msgs().find((m) => m.op === 'publish' && m.topic === '/amr1/cmd_vel').msg;
    expect(vel.linear.x).toBeCloseTo(0.5);
    expect(vel.angular.z).toBeCloseTo(10 * DEG);
    const pose = msgs().find((m) => m.op === 'publish' && m.topic === '/amr1/robot_pose').msg;
    expect(pose.position.x).toBeCloseTo(1);
    expect(pose.position.y).toBeCloseTo(2);
    expect(pose.orientation.z).toBeCloseTo(Math.sin(45 * DEG), 6);
    expect(msgs().find((m) => m.op === 'publish' && m.topic === '/amr1/battery_state').msg.percentage).toBe(1);
    // navigation runtime: ground truth odometry and the LiDAR scan
    const gt = msgs().find((m) => m.op === 'publish' && m.topic === '/amr1/ground_truth').msg;
    expect(gt.pose.pose.position.x).toBeCloseTo(1);
    expect(gt.child_frame_id).toBe('base_link');
    const scan = msgs().find((m) => m.op === 'publish' && m.topic === '/amr1/scan').msg;
    expect(scan.ranges.length).toBe(rt.lastScan!.ranges.length);
    expect(scan.range_max).toBeCloseTo(15);
    // vision: published once per new output, under the mobile robot namespace; the camera without publishRos is silent
    const det = msgs().filter((m) => m.op === 'publish' && m.topic === '/amr1/vision/cam_a/detections');
    expect(det.length).toBe(1);
    expect(det[0].msg.detections[0].results[0].hypothesis.class_id).toBe('box');
    expect(det[0].msg.detections[0].id).toBe('7');
    expect(det[0].msg.header.frame_id).toBe('cam_a_optical_frame');
    const tg = msgs().find((m) => m.op === 'publish' && m.topic === '/amr1/vision/cam_a/targets').msg;
    expect(tg.poses[0].position).toEqual({ x: 0.1, y: 0.2, z: 0.3 });
    expect(tg.poses[0].orientation.w).toBeCloseTo(1);
    expect(msgs().find((m) => m.op === 'publish' && m.topic === '/amr1/vision/cam_a/detections_3d').msg.detections[0].bbox.center.position.z).toBeCloseTo(0.3);
    expect(msgs().find((m) => m.op === 'publish' && m.topic === '/amr1/vision/cam_a/image/compressed').msg).toMatchObject({ format: 'jpeg', data: 'QUJD' });
    expect(msgs().find((m) => m.op === 'publish' && m.topic === '/amr1/vision/cam_a/vlm/answer').msg.data).toBe('one box on the table');
    expect(msgs().some((m) => String(m.topic).includes('silent'))).toBe(false);
    // a new pipeline output is published again
    vrt.last = fakeOutput(cam);
    await until(() => msgs().filter((m) => m.op === 'publish' && m.topic === '/amr1/vision/cam_a/detections').length >= 2);
    bridge.callService('/reset', { hard: true });
    await until(() => msgs().some((m) => m.op === 'call_service'));
    expect(msgs().find((m) => m.op === 'call_service')).toEqual({ op: 'call_service', service: '/reset', args: { hard: true } });
    bridge.disconnect();
    await until(() => !bridge.connected);
    expect(app.logs.some(([t, l]: [string, string]) => t === 'rosbridge disconnected' && l === 'warn')).toBe(true);
    const n = msgs().length;
    await wait(60);
    expect(msgs().length).toBe(n); // publishing timer stopped with the connection
    bridge.publish('/late', 'std_msgs/msg/String', { data: 'ignored' });
  });

  it('follows /joint_states (revolute and prismatic joints) and /odom in follow mode', async () => {
    const st = new Station('Twin');
    const arm = st.addChild(createRobotFromLibrary('UR5e', 'Arm'));
    const rail = st.addChild(buildMechanism({ type: MAKE_ROBOT_1T, parameters: [3000], jointsBuild: [0], jointsHome: [0], jointsSenses: [1], lower: [0], upper: [3000], name: 'Rail' }));
    const amr = st.addChild(new MobileRobot('AMR'));
    const app = fakeApp(st);
    const bridge = new RosBridge(app, { url: `ws://127.0.0.1:${port}`, mode: 'follow' });
    await bridge.connect();
    const sock = lastSocket();
    const msgs = () => inbox.get(sock)!;
    await until(() => msgs().filter((m) => m.op === 'subscribe').length >= 2);
    expect(msgs().filter((m) => m.op === 'subscribe').map((m) => [m.topic, m.type])).toEqual([['/joint_states', 'sensor_msgs/msg/JointState'], ['/odom', 'nav_msgs/msg/Odometry']]);
    expect(msgs().some((m) => m.op === 'advertise')).toBe(false);
    const names = arm.jointNames();
    const before = arm.joints();
    rail.chain.joints[0].name = 'rail_slide'; // distinct from the arm's J1..J6
    const railJoint = rail.jointNames()[0];
    sock.send('garbage {');
    sock.send(JSON.stringify({ op: 'status', level: 'info' }));
    sock.send(JSON.stringify({ op: 'publish', topic: '/unknown', msg: {} }));
    sock.send(JSON.stringify({ op: 'publish', topic: '/joint_states', msg: { name: [names[1], 'not_a_joint', railJoint, names[0]], position: [-1.0, 9, 1.25, 0.5] } }));
    await until(() => Math.abs(arm.joints()[1] + 1.0 * RAD) < 1e-6);
    expect(arm.joints()[0]).toBeCloseTo(0.5 * RAD, 6);
    expect(arm.joints()[2]).toBe(before[2]); // joints absent from the message keep their value
    expect(rail.joints()[0]).toBeCloseTo(1250, 6);
    sock.send(JSON.stringify({ op: 'publish', topic: '/joint_states', msg: { position: [1, 2, 3] } }));
    sock.send(JSON.stringify({ op: 'publish', topic: '/odom', msg: { header: {} } }));
    sock.send(JSON.stringify({ op: 'publish', topic: '/odom', msg: { pose: { pose: { position: { x: 1.5, y: -2, z: 0 }, orientation: { x: 0, y: 0, z: Math.sin(45 * DEG), w: Math.cos(45 * DEG) } } } } }));
    await until(() => Math.abs(amr.state.x - 1500) < 1e-6);
    expect(amr.state.y).toBeCloseTo(-2000, 6);
    expect(amr.state.theta).toBeCloseTo(90, 6);
    expect(arm.joints()[0]).toBeCloseTo(0.5 * RAD, 6); // the message without names changed nothing
    const n = msgs().length;
    await wait(60);
    expect(msgs().length).toBe(n); // follow mode publishes nothing
    bridge.disconnect();
    await until(() => !bridge.connected);
  });

  it('ignores odometry when the station has no mobile robot and rejects unreachable servers', async () => {
    const st = new Station('Empty');
    const app = fakeApp(st);
    const bridge = new RosBridge(app, { url: `ws://127.0.0.1:${port}` });
    await bridge.connect();
    const sock = lastSocket();
    const msgs = () => inbox.get(sock)!;
    await until(() => msgs().filter((m) => m.op === 'subscribe').length >= 2);
    sock.send(JSON.stringify({ op: 'publish', topic: '/odom', msg: { pose: { pose: { position: { x: 1, y: 1 }, orientation: { x: 0, y: 0, z: 0, w: 1 } } } } }));
    await wait(30);
    expect(st.itemsOfType(ItemType.MOBILE_ROBOT).length).toBe(0);
    bridge.disconnect();
    await until(() => !bridge.connected);
    const dead = new RosBridge(app, { url: 'ws://127.0.0.1:1' });
    await expect(dead.connect()).rejects.toThrow(/rosbridge connection failed/);
    expect(dead.connected).toBe(false);
  });
});

// ---------------------------------------------------------------------------------------------
// Publishers (message builders used by the bridge)
// ---------------------------------------------------------------------------------------------

describe('ROS message builders', () => {
  it('builds PoseArray / Odometry messages and lists the topics it published', () => {
    const cam = new Camera('Eye');
    const out: PipelineOutput = { frame: { width: 640, height: 480, K: intrinsicsFromCamera(cam), camWorld: cam.poseAbs(), time: 2 }, results: {}, detections: [], targets: [{ name: 't', cls: 'apple', grasp: transl(1000, 0, 500), approach: transl(1000, 0, 400), p: [1000, 0, 500], z: 800, sigmaMm: 3 }], warnings: [] };
    const pa = poseArrayMsg(out, 'odom');
    expect(pa.header.frame_id).toBe('odom');
    expect(pa.header.stamp).toEqual({ sec: 2, nanosec: 0 });
    expect(pa.poses[0].position).toEqual({ x: 1, y: 0, z: 0.5 });
    const published: string[] = [];
    const topics = publishVisionOutput({ publish: (t) => published.push(t) }, cam, out, { namespace: '/r/', frameId: 'eye_frame', worldFrame: 'odom', cloud: false, image: false });
    expect(topics).toEqual(['/r/vision/eye/detections', '/r/vision/eye/targets']);
    expect(published).toEqual(topics);
    const odom = odometryMsg({ x: 1000, y: 0, theta: 180 }, { v: 1000, omega: 90 }, { sigmaMm: 100, sigmaDeg: 2, timeS: 3 });
    expect(odom.pose.covariance[0]).toBeCloseTo(0.01);
    expect(odom.twist.twist.angular.z).toBeCloseTo(Math.PI / 2);
    expect(Math.abs(odom.pose.pose.orientation.z)).toBeCloseTo(1, 6);
    const st = new Station('N');
    const amr = st.addChild(new MobileRobot('N1'));
    amr.rosNamespace = '/n1';
    const map = st.addChild(new MapItem('M'));
    map.resize(50, 50, 200, 0, 0);
    map.fillRect(4000, 0, 4400, 10000);
    setNavStack(amr, { platform: 'amr', environment: 'warehouse', sensors: ['lidar2d'], localization: 'slam_toolbox_2d', navigation: 'nav2_navfn_dwb', fusion: [], simulate: true });
    amr.setPose2D(1000, 5000, 0);
    const rt = stepNavRuntime(amr, 1, map, []) as NavRuntime;
    const sent: Array<[string, string]> = [];
    const navTopics = publishNavRuntime({ publish: (t, ty) => sent.push([t, ty]) }, amr, rt, { map: true, timeS: 4 });
    expect(navTopics).toEqual(['/n1/odom_estimate', '/n1/ground_truth', '/n1/scan', '/n1/slam_map']);
    expect(sent.find(([t]) => t === '/n1/slam_map')![1]).toBe('nav_msgs/msg/OccupancyGrid');
  });
});
