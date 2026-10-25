/**
 * G3 (V5 F2 / fix_F2 §8): the studio running in the browser as the MURKA ROS 2 backend.
 *  - Open folder murom_webots alone gives a MUROM world without products (murom_world/config, the supervisor's
 *    $MUROM_CONFIG_DIR, is a sibling package): the Connect dialog's layout loader (src/ros/murka_layout.ts) adds the
 *    50 products at the poses the Webots supervisor spawns them (same as a folder import that holds the YAML files).
 *  - The browser backend runs in a Web Worker (src/ros/sim_worker.ts) on a structured clone of the station and the
 *    studio station follows it through state snapshots (src/ros/sim_mirror.ts): driven here through a fake rosbridge,
 *    the mirrored base / joints / carried product equal what the backend published.
 *  - Camera devices render only while subscribed (webots_ros2 devices without <alwaysOn>), via rosapi
 *    /rosapi/subscribers; without rosapi they publish at their rate as before.
 *  - The page-thread fallback (no Worker) is paced to the wall clock with catch-up frames: a stalled page thread no
 *    longer loses sim time (the old setInterval loop advanced one 32 ms frame per tick, whatever the delay).
 * The end-to-end check (Chrome + WSL ROS 2 + MURKA smoke_test.py) is tests/e2e/ros_murka_browser.e2e.test.ts.
 */
import { describe, it, expect, afterAll } from 'vitest';
import fs from 'node:fs';
import path from 'node:path';
import { importWebotsWorld } from '../src/io/webots';
import { AssetStore } from '../src/scene/assets';
import { SceneObject, type Station } from '../src/core/items/item';
import { Robot } from '../src/core/items/robot';
import { MobileRobot } from '../src/mobile/items';
import { needsRetailLayout, loadRetailLayoutFiles, findLayoutPair, retailProductCount } from '../src/ros/murka_layout';
import { applySnapshot, StateMirror, stationTransfer, type MirrorSnapshot } from '../src/ros/sim_mirror';
import { MuromSimInterface, findMurom } from '../src/ros/murom_sim';
import { SubscriberWatch, startSimBackend } from '../src/ros/sim_backend';
import { RosBridge, simWorkerStart } from '../src/ros/rosbridge';
import { RosbridgeClient } from '../src/ros/rosbridge_client';
import { loadStation } from '../src/io/station-file';
import { DEG } from '../src/core/math/pose';
import { HAS_MURKA, RETAIL_V2, murkaSource } from './webots_helpers';
import { buildRetailStation, MURKA_CONFIG } from './ros_murka_helpers';
import { FakeRosbridge, until } from './ros_fake_rosbridge';

const fakes: FakeRosbridge[] = [];
afterAll(async () => { for (const f of fakes) await f.close(); });
const tSec = (s: { sec: number; nanosec: number }) => s.sec + s.nanosec * 1e-9;
const sleep = (ms: number) => new Promise((r) => setTimeout(r, ms));
const layoutFiles = () => new Map([
  ['config/products.yaml', fs.readFileSync(path.join(MURKA_CONFIG, 'products.yaml'), 'utf8')],
  ['config/locations.yaml', fs.readFileSync(path.join(MURKA_CONFIG, 'locations.yaml'), 'utf8')],
]);

describe('retail layout of the MURKA backend (Open folder murom_webots + murom_world/config)', () => {
  it('finds the products.yaml / locations.yaml pair of one folder, murom_world/config first', () => {
    expect(findLayoutPair(['a/products.yaml', 'b/locations.yaml'])).toBeNull();
    expect(findLayoutPair(['x/y/products.yaml', 'x/y/locations.yaml', 'src/murom_world/config/products.yaml', 'src/murom_world/config/locations.yaml']))
      .toEqual({ products: 'src/murom_world/config/products.yaml', locations: 'src/murom_world/config/locations.yaml' });
    expect(findLayoutPair(['config\\products.yml', 'config\\locations.yml'])).toEqual({ products: 'config\\products.yml', locations: 'config\\locations.yml' });
  });

  it.skipIf(!HAS_MURKA)('a murom_webots world alone has no products; the layout loader adds the 50 products at the supervisor poses', async () => {
    const assets = new AssetStore();
    const alone = importWebotsWorld('worlds/retail_store.wbt', murkaSource(RETAIL_V2), { assets }).station;
    expect(findMurom(alone)).not.toBeNull();
    expect(retailProductCount(alone)).toBe(0);
    expect(needsRetailLayout(alone)).toBe(true);
    const res = await loadRetailLayoutFiles(alone, layoutFiles());
    expect(res.products).toBe(50);
    expect(res.report.join('\n')).toMatch(/50 products, 93 slots/);
    expect(needsRetailLayout(alone)).toBe(false);
    // the same products, SKUs, slots and poses as the import of a folder that holds the YAML files (RetailProduct PROTO)
    const ref = buildRetailStation().station;
    const products = (st: Station) => new Map([...st.walk()].filter((i): i is SceneObject => i instanceof SceneObject && typeof i.params.sku === 'string').map((p) => [p.name, p]));
    const a = products(alone), b = products(ref);
    expect([...a.keys()].sort()).toEqual([...b.keys()].sort());
    for (const [name, p] of a) {
      const q = b.get(name)!;
      expect([p.params.sku, p.params.slot]).toEqual([q.params.sku, q.params.slot]);
      const P = p.poseAbs(), Q = q.poseAbs();
      for (let k = 0; k < 16; k++) expect(Math.abs(P[k] - Q[k])).toBeLessThan(1e-6);
    }
    // the backend registers them (/murom/sim/objects)
    const sim = new MuromSimInterface(alone, { publish: () => {}, subscribe: () => {} }, { assets });
    expect(sim.products.size).toBe(50);
  });
});

describe.skipIf(!HAS_MURKA)('MURKA backend in a Web Worker on a copy of the station (browser path)', () => {
  it('the worker drives the studio station through snapshots: base and joints equal what it published; cameras only while subscribed', async () => {
    const { station, assets } = buildRetailStation();
    const fake = await FakeRosbridge.start(); fakes.push(fake);
    let subscribers: string[] = [];
    fake.answers.set('/rosapi/subscribers', (args) => ({ subscribers: /head_camera|head_depth/.test(args.topic) ? subscribers : [] }));
    const sim0 = new MuromSimInterface(station, { publish: () => {}, subscribe: () => {} }, { assets });
    const base0 = { x: sim0.base.state.x, y: sim0.base.state.y };
    // the worker global scope: postMessage to the page, onmessage from it (structured clones both ways, like a Worker)
    const fromWorker: any[] = [];
    const g = globalThis as unknown as { self?: unknown };
    const hadSelf = 'self' in g; const oldSelf = g.self;
    const scope = { postMessage: (m: unknown) => fromWorker.push(structuredClone(m)), onmessage: null as null | ((e: { data: unknown }) => void) };
    g.self = scope;
    await import('../src/ros/sim_worker');
    if (hadSelf) g.self = oldSelf; else delete g.self;
    const start = simWorkerStart(station, assets, fake.url, { cameraRate: 5, lidarNoise: false });
    expect(start.meshes.length).toBeGreaterThan(10);
    scope.onmessage!({ data: structuredClone(start) });
    await until(() => fromWorker.some((m) => m.type === 'connected' || m.type === 'error'), 20000);
    expect(fromWorker.find((m) => m.type === 'error')).toBeUndefined();
    const conn = fromWorker.find((m) => m.type === 'connected');
    expect(conn.kind).toBe('murom');
    let applied = 0, missing = 0, seen = 0;
    const pump = () => { for (; seen < fromWorker.length; seen++) { const m = fromWorker[seen]; if (m.type === 'state') { const r = applySnapshot(station, m.snap as MirrorSnapshot); applied += r.applied; missing += r.missing; } } };
    // drive backwards 1 s (cmd_vel at 20 Hz, like smoke_test.py), then let the arms settle
    const t0 = Date.now();
    while (Date.now() - t0 < 1000) { fake.send('/cmd_vel', { linear: { x: -0.25, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } }); pump(); await sleep(50); }
    await sleep(1500); pump();
    // no camera subscriber (rosapi answers []): no images, like a webots_ros2 camera without <alwaysOn>
    expect(fake.published('/head_camera/image_color').length).toBe(0);
    expect(fake.published('/head_depth/image').length).toBe(0);
    expect(fake.ops.some((o) => o.op === 'call_service' && o.service === '/rosapi/subscribers')).toBe(true);
    // a subscriber appears: images at the camera rate (5 Hz sim time) within the rosapi poll period
    subscribers = ['/murom_perception'];
    await until(() => fake.published('/head_camera/image_color').length >= 6, 10000);
    pump();
    const imgs = fake.published('/head_camera/image_color');
    const stamps = imgs.map((m) => tSec(m.header.stamp));
    // 5 Hz on the 32 ms control frames: every gap is 6 or 7 frames, the mean period 0.2 s within one frame / gaps
    for (let i = 1; i < stamps.length; i++) expect([0.192, 0.224].some((g) => Math.abs(stamps[i] - stamps[i - 1] - g) < 1e-6)).toBe(true);
    expect(Math.abs((stamps[stamps.length - 1] - stamps[0]) / (stamps.length - 1) - 0.2)).toBeLessThan(0.032 / (stamps.length - 1) + 1e-6);
    expect(imgs[0]).toMatchObject({ width: 320, height: 240, encoding: 'bgra8', step: 1280 });
    expect(fake.published('/head_depth/image').length).toBeGreaterThanOrEqual(5);
    // the robot moved backwards in the worker; the studio copy follows exactly once the final snapshot is applied
    scope.onmessage!({ data: { type: 'stop' } });
    await until(() => fromWorker.some((m) => m.type === 'closed'), 10000);
    pump();
    expect(missing).toBe(0);
    expect(applied).toBeGreaterThan(10);
    const gt = fake.last('/murom/ground_truth_pose');
    const sim = findMurom(station)!;
    // the last published ground truth is at most one frame older than the final state (no command any more: equal)
    expect(Math.abs(sim.base.state.x / 1000 - gt.pose.position.x)).toBeLessThan(1e-9);
    expect(Math.abs(sim.base.state.y / 1000 - gt.pose.position.y)).toBeLessThan(1e-9);
    expect(Math.hypot(sim.base.state.x - base0.x, sim.base.state.y - base0.y)).toBeGreaterThan(150);
    const js = fake.last('/joint_states');
    const byName = new Map<string, number>();
    for (const r of sim.robots) r.jointNames().forEach((n, i) => byName.set(n, r.joints()[i] * DEG));
    for (let i = 2; i < js.name.length; i++) expect(Math.abs(byName.get(js.name[i])! - js.position[i])).toBeLessThan(1e-6);
    expect(byName.get('l_elbow_joint')!).toBeLessThan(-2); // v2 travel pose reached in the studio copy too
    const last = fromWorker.filter((m) => m.type === 'state').pop();
    expect(last.rtf).toBeGreaterThan(0.8);
    expect(last.time).toBeGreaterThan(2);
  }, 60000);

  it('a snapshot carries only what changed, by item id; unknown ids are counted, not fatal', () => {
    const { station, assets } = buildRetailStation();
    const copy = loadStation(stationTransfer(station, assets).tree);
    const mirror = new StateMirror(copy);
    expect(mirror.snapshot(0)).toEqual({ t: 0, bases: [], joints: [], poses: [] });
    const m = findMurom(copy)!;
    m.base.setPose2D(1000, 2000, 30);
    const arm = m.robots.find((r) => r.jointNames().includes('l_elbow_joint'))!;
    const q = arm.joints(); q[3] = -40; arm.setJoints(q);
    const prod = [...copy.walk()].find((i): i is SceneObject => i instanceof SceneObject && i.name === 'tea_box_1')!;
    const P = Float64Array.from(prod.pose()); P[14] += 100; prod.setPose(P as never);
    const s = mirror.snapshot(1.5);
    expect(s.bases.map(([id]) => id)).toEqual([m.base.id]);
    expect(s.joints.map(([id]) => id)).toEqual([arm.id]);
    expect(s.poses.map(([id]) => id)).toEqual([prod.id]);
    expect(mirror.snapshot(1.6)).toEqual({ t: 1.6, bases: [], joints: [], poses: [] });
    const r = applySnapshot(station, s);
    expect(r).toEqual({ applied: 3, missing: 0 });
    const ms = findMurom(station)!;
    expect([ms.base.state.x, ms.base.state.y, ms.base.state.theta]).toEqual([1000, 2000, 30]);
    expect(Array.from(ms.base.poseAbs())).toEqual(Array.from(m.base.poseAbs()));
    expect((station.findById(arm.id) as Robot).joints()[3]).toBe(-40);
    expect(Array.from(station.findById(prod.id)!.pose())).toEqual(Array.from(P));
    expect(station.simTime).toBe(1.5);
    expect(applySnapshot(station, { t: 2, bases: [['nope', [0, 0, 0, 0, 0, 0]]], joints: [], poses: [['gone', Array.from(P)]] })).toEqual({ applied: 0, missing: 2 });
    expect(station.findById(m.base.id)).toBeInstanceOf(MobileRobot);
  });
});

describe('camera on demand (rosapi /rosapi/subscribers)', () => {
  it('SubscriberWatch: off until answered, counts per topic, unknown (always on) for good without rosapi', async () => {
    const fake = await FakeRosbridge.start(); fakes.push(fake);
    const c = new RosbridgeClient(fake.url); await c.connect();
    let subs: Record<string, string[]> = { '/a': [], '/b': ['/n1'] };
    fake.answers.set('/rosapi/subscribers', (a) => ({ subscribers: subs[a.topic] ?? [] }));
    const w = new SubscriberWatch(c, 50);
    expect(w.has(['/a', '/b'])).toBe(false);
    await until(() => w.available === true, 3000);
    expect(w.has(['/a'])).toBe(false);
    await until(() => w.has(['/a', '/b']) === true, 3000);
    subs = { '/a': ['/n2'], '/b': [] };
    await until(() => w.has(['/a']) === true && w.has(['/b']) === false, 3000);
    w.stop(); c.close();
    // no rosapi: the calls fail, the demand stays unknown (cameras always on)
    const fake2 = await FakeRosbridge.start(); fakes.push(fake2);
    fake2.answers.set('/rosapi/subscribers', () => undefined);
    const c2 = new RosbridgeClient(fake2.url); await c2.connect();
    const w2 = new SubscriberWatch(c2, 20);
    expect(w2.has(['/x'])).toBe(false);
    await until(() => w2.available === false, 3000);
    expect(w2.has(['/x'])).toBeUndefined();
    c2.close();
  });

  it.skipIf(!HAS_MURKA)('MuromSimInterface renders the head sensors only when sensorDemand says they are subscribed', () => {
    const { station, assets } = buildRetailStation();
    const pubs: string[] = [];
    let want: Record<string, boolean | undefined> = { rgb: false, depth: false };
    const sim = new MuromSimInterface(station, { publish: (t) => pubs.push(t), subscribe: () => {} }, { assets, cameraRate: 5, lidarNoise: false, sensorDemand: (topics) => (topics[0].startsWith('/head_camera') ? want.rgb : want.depth) });
    sim.start(0);
    const run = (frames: number) => { for (let i = 0; i < frames; i++) sim.step(0.032, Math.round((sim.time + 0.032) * 1e6) / 1e6); };
    run(32);
    expect(pubs.filter((t) => /head_/.test(t))).toEqual([]);
    expect(sim.cameraStats.rendered).toBe(0);
    expect(sim.cameraStats.skipped).toBeGreaterThanOrEqual(5);
    want = { rgb: false, depth: true };
    pubs.length = 0; run(32);
    expect(pubs.filter((t) => t === '/head_depth/image').length).toBeGreaterThanOrEqual(5);
    expect(pubs.filter((t) => t === '/head_camera/image_color').length).toBe(0);
    want = { rgb: undefined, depth: undefined }; // unknown (no rosapi): always on
    pubs.length = 0; run(32);
    expect(pubs.filter((t) => t === '/head_camera/image_color').length).toBeGreaterThanOrEqual(5);
    expect(pubs.filter((t) => t === '/head_depth/image').length).toBeGreaterThanOrEqual(5);
  });
});

describe.skipIf(!HAS_MURKA)('camera back pressure: frames dropped while rosbridge lags, the clock keeps real time', () => {
  it('the /clock echo through rosbridge measures the lag; frames stop 1 s above its baseline and resume below 0.4 s', async () => {
    const { station, assets } = buildRetailStation();
    const fake = await FakeRosbridge.start(); fakes.push(fake);
    fake.answers.set('/rosapi/subscribers', () => ({ subscribers: ['/perception'] }));
    let delayMs = 0, seen = 0;
    // rosbridge echoes the backend's /clock to its own /clock subscription, after its queue (delayMs)
    const pump = setInterval(() => { for (; seen < fake.ops.length; seen++) { const o = fake.ops[seen]; if (o.op === 'publish' && o.topic === '/clock') { const m = o.msg; setTimeout(() => fake.send('/clock', m), delayMs); } } }, 5);
    const h = await startSimBackend({ url: fake.url, station, assets, rtf: 1, murom: { cameraRate: 5, lidarNoise: false }, log: () => {} });
    try {
      await until(() => fake.subscriptions().has('/clock'), 5000);
      expect(fake.subscriptions().get('/clock')).toBe('rosgraph_msgs/msg/Clock');
      expect(fake.ops.find((o) => o.op === 'subscribe' && o.topic === '/clock')!.raw).toMatch(/"throttle_rate":100/);
      const sim = h.sim as MuromSimInterface;
      await until(() => sim.cameraStats.rendered >= 5, 10000);
      expect(h.rosLag()).toBeDefined(); // the in-process fake parses the images too: a few frames of baseline lag
      // rosbridge falls 1.6 s behind: frames are dropped, nothing else
      delayMs = 1600;
      await until(() => sim.cameraStats.dropped >= 3, 10000);
      const r0 = sim.cameraStats.rendered, c0 = fake.published('/clock').length;
      await sleep(800);
      expect(sim.cameraStats.rendered).toBe(r0);
      expect(h.rosLag()!).toBeGreaterThan(1.4);
      expect(fake.published('/clock').length - c0).toBeGreaterThan(15); // the clock runs on (31 Hz)
      // the queue drains: frames again
      delayMs = 0;
      await until(() => sim.cameraStats.rendered > r0 + 2, 10000);
      expect(h.rtf()).toBeGreaterThan(0.9);
    } finally { clearInterval(pump); h.stop(); await h.done; }
  }, 60000);
});

describe.skipIf(!HAS_MURKA)('page-thread MURKA backend (no Worker): paced to the wall clock', () => {
  it('a stalled page thread does not lose sim time: the loop catches up (the old setInterval loop did not)', async () => {
    const { station, assets } = buildRetailStation();
    const { setSensingAssets } = await import('../src/vision/sensing');
    setSensingAssets(assets);
    const fake = await FakeRosbridge.start(); fakes.push(fake);
    const g = globalThis as unknown as { window?: unknown };
    const hadWindow = 'window' in g; if (!hadWindow) g.window = globalThis;
    const app = { station, simTime: 0, log: () => {} } as never;
    const rb = new RosBridge(app, { url: fake.url, mode: 'sim', murom: { cameraRate: 0, lidarNoise: false } });
    try {
      await rb.connect();
      expect(rb.backendStats()?.where).toBe('thread');
      const t0 = Date.now(), s0 = rb.backendStats()!.time;
      await sleep(300);
      const block = Date.now(); while (Date.now() - block < 400) { /* the page thread is busy (a long 3D frame, a big import) */ }
      await sleep(700);
      const wall = (Date.now() - t0) / 1000, sim = rb.backendStats()!.time - s0;
      expect(sim / wall).toBeGreaterThan(0.9);
      expect(sim / wall).toBeLessThan(1.1);
      const clocks = fake.published('/clock').map((m) => tSec(m.clock));
      for (let i = 1; i < clocks.length; i++) expect(clocks[i] - clocks[i - 1]).toBeCloseTo(0.032, 9);
    } finally {
      rb.disconnect();
      if (!hadWindow) delete g.window;
    }
  }, 60000);
});

it('startSimBackend reports the frame cost (busy) for the diagnostics of the Connect dialog', async () => {
  const { Station: St } = await import('../src/core/items/item');
  const fake = await FakeRosbridge.start(); fakes.push(fake);
  const h = await startSimBackend({ url: fake.url, station: new St('empty'), rtf: 1, log: () => {} });
  await sleep(200);
  expect(h.kind).toBe('twin');
  expect(h.steps()).toBeGreaterThan(2);
  expect(h.busy().ms).toBeGreaterThanOrEqual(0);
  h.stop();
  await h.done;
});
