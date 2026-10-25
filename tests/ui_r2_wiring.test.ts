// @vitest-environment jsdom
/**
 * R2 integration: requests of round 1 wired into the app and checked against ground truth.
 *  - Export: asynchronous posts through generateProgram, compiler + post problems shown in the dialog.
 *  - Analog outputs (RoboDK "Set AO") reach the posts' setAO in vendor syntax.
 *  - The in-browser post stub carries RoboDK's own robolink constants (INS_TYPE_MOVE = 0 ...).
 *  - .vbstation / RoboDK-converter exports with an embedded .rdk are decoded by the binary reader (same station as a
 *    direct import of that .rdk).
 *  - Webots Viewpoint -> initial 3D view; renderer.lookAt horizontal field of view.
 *  - renderer.rebuildAll with a removed subtree; RoboDK Display POINTSIZE point clouds.
 *  - Fleet tab KPI groups, Camera tab fruit overlay through the camera model, zone dialog lane / depot.
 */
import { describe, it, expect, beforeAll, afterEach } from 'vitest';
import { existsSync, readFileSync } from 'node:fs';
import { spawnSync } from 'node:child_process';
import { join } from 'node:path';
import * as THREE from 'three';
import { installDomStubs, makeApp, stubRenderer, currentDialog, setField, clickOk, clickCancel, waitFor, tickMs, closeAllDialogs, clearToasts, downloads, dialogTitle } from './ui_harness';
import { App, ItemType, Program, Robot, SceneObject, Camera, Target } from '../src/app';
import { Station } from '../src/core/items/item';
import { exportDialog, zoneDialog } from '../src/ui/dialogs';
import { registerPost, PostProgram, compileForPost, getPost } from '../src/posts/index';
import { ROBOLINK_STUB, officialConstants } from '../src/posts/python_post';
import { SceneRenderer, pointCloudOf } from '../src/scene/renderer';
import { AssetStore } from '../src/scene/assets';
import { importRdkBestEffort } from '../src/io/robodk/rdk_container';
import { ZoneItem } from '../src/mobile/items';
import { FleetItem } from '../src/fleet/fleet';
import { BottomPanel, fruitOverlayBoxes } from '../src/ui/bottom';
import { cameraIntrinsics, threeFovDeg } from '../src/vision/camera_model';
import { Robolink } from '../src/api/robolink';
import { transl, multiply, rotx, getPos } from '../src/core/math/pose';

beforeAll(() => installDomStubs());
afterEach(() => { closeAllDialogs(); clearToasts(); });
const LIB = 'C:/RoboDK/Library';

describe('export dialog: asynchronous posts and problems', () => {
  it('an asynchronous post is generated through generateProgram; compiler and post problems are listed', async () => {
    const app = makeApp('pickplace');
    const prog = app.station.itemsOfType<Program>(ItemType.PROGRAM)[0];
    const robot = prog.robot() as Robot;
    // a target 10 m away: compileForPost keeps the move and reports it unreachable
    const far = robot.addChild(new Target('Far away')); far.setPose(multiply(transl(10000, 0, 500), rotx(Math.PI)));
    prog.addMoveL(far);
    const seen: PostProgram[] = [];
    registerPost({ id: 'R2_ASYNC', name: 'R2 async test post', brand: 'Test', extension: 'txt', generate: () => [{ name: 'sync.txt', content: 'SYNC PATH' }],
      generateAsync: async (p) => { seen.push(p); await tickMs(5); p.problems.push('post warning: name truncated'); return [{ name: `${p.name}.txt`, content: `moves=${p.events.filter((e) => e.kind.startsWith('move')).length}` }, { name: 'blob.bin', content: btoa('\u0001\u0002\u0003'), encoding: 'base64' }]; } });
    const direct = await app.exportProgramAsync('R2_ASYNC', prog);
    expect(direct.files.map((f) => f.name)).toEqual([`${prog.name}.txt`, 'blob.bin']);
    const moves = compileForPost(app.station, prog).events.filter((e) => e.kind.startsWith('move')).length;
    expect(direct.files[0].content).toBe(`moves=${moves}`);
    expect(direct.problems.some((p) => /Far away.*unreachable/.test(p))).toBe(true);
    expect(direct.problems).toContain('post warning: name truncated');
    // the dialog: preview and problem list follow the selected (asynchronous) post
    const n0 = downloads.length;
    const p = exportDialog(app, prog);
    const d = currentDialog();
    setField('Post processor', 'R2_ASYNC');
    await waitFor(() => /moves=\d+/.test(d.querySelector('pre.code-preview')!.textContent ?? ''));
    expect(d.querySelector('pre.code-preview')!.textContent).toContain('(binary, 3 bytes)');
    const items = [...d.querySelectorAll('ul.export-problems li')].map((li) => li.textContent);
    expect(items.some((t) => /Far away.*unreachable/.test(t!))).toBe(true);
    expect(items).toContain('post warning: name truncated');
    // a synchronous post shows its code at once
    setField('Post processor', 'KUKA_KRC4');
    expect(d.querySelector('pre.code-preview')!.textContent).toMatch(/DEF |PTP|LIN/);
    setField('Post processor', 'R2_ASYNC');
    clickOk(); await p;
    expect(downloads.slice(n0).map((x) => x.name)).toEqual([`${prog.name}.txt`, 'blob.bin']);
    expect(seen.length).toBeGreaterThanOrEqual(3);
  });

  it('a failing asynchronous post shows its error instead of code', async () => {
    const app = makeApp('pickplace');
    registerPost({ id: 'R2_FAIL', name: 'R2 failing post', brand: 'Test', extension: 'txt', generate: () => [], generateAsync: async () => { throw new Error('RoboDK post failed: no licence'); } });
    const p = exportDialog(app);
    const d = currentDialog();
    setField('Post processor', 'R2_FAIL');
    await waitFor(() => /failed/.test(d.querySelector('pre.code-preview')!.textContent ?? ''));
    expect(d.querySelector('pre.code-preview')!.textContent).toBe('Post processor failed: RoboDK post failed: no licence');
    clickCancel(); await p;
  });
});

describe('analog outputs in vendor syntax', () => {
  const expected: Record<string, RegExp> = {
    KUKA_KRC4: /^\$ANOUT\[3\]=0\.75$/m, ABB_RAPID_IRC5: /^\s*SetAO A_OUT_3, 0\.75;$/m, Fanuc_R30i: /AO\[3\]=\(0\.75\)/, Universal_Robots: /set_standard_analog_out\(3, 0\.75\)/, Motoman: /^AOUT AO#\(3\) 0\.75$/m,
  };
  for (const [post, re] of Object.entries(expected)) {
    it(`${post}: setAO for an analog output, the digital output keeps its syntax`, () => {
      const app = makeApp('pickplace');
      const prog = app.station.itemsOfType<Program>(ItemType.PROGRAM)[0];
      prog.addInstruction({ kind: 'io', io: '3', value: 0.75, wait: false, analog: true } as any);
      prog.addInstruction({ kind: 'io', io: '4', value: 1, wait: false } as any);
      const compiled = compileForPost(app.station, prog);
      expect(compiled.events.filter((e) => e.kind === 'setAO')).toEqual([{ kind: 'setAO', io: '3', value: 0.75 }]);
      const code = getPost(post)!.generate(compiled).map((f) => f.content).join('\n');
      expect(code).toMatch(re);
      expect(code).not.toMatch(/DO\[3\]|\$OUT\[3\]|set_standard_digital_out\(3/);
    });
  }
});

describe('browser post stub', () => {
  const OFFICIAL = 'C:/RoboDK/Python/robodk/robolink.py';
  const constants = (src: string) => new Map([...src.matchAll(/^((?:INS_TYPE|ITEM_TYPE|MOVE_TYPE|RUNMODE)_[A-Z_]+)\s*=\s*(-?\d+)/gm)].map((m) => [m[1], Number(m[2])]));
  it('carries the generated official constants (INS_TYPE_MOVE = 0, INS_TYPE_MOVEC = 1, INS_TYPE_PAUSE = 6, INS_TYPE_CODE = 8)', () => {
    const c = constants(ROBOLINK_STUB);
    expect([c.get('INS_TYPE_MOVE'), c.get('INS_TYPE_MOVEC'), c.get('INS_TYPE_CHANGESPEED'), c.get('INS_TYPE_CHANGEFRAME'), c.get('INS_TYPE_CHANGETOOL'), c.get('INS_TYPE_PAUSE'), c.get('INS_TYPE_CODE'), c.get('INS_TYPE_PRINT')]).toEqual([0, 1, 2, 3, 4, 6, 8, 9]);
    expect([c.get('MOVE_TYPE_JOINT'), c.get('MOVE_TYPE_LINEAR'), c.get('MOVE_TYPE_CIRCULAR'), c.get('RUNMODE_SIMULATE'), c.get('RUNMODE_MAKE_ROBOTPROG')]).toEqual([1, 2, 3, 1, 3]);
    expect(officialConstants('A_B = 1\nlower = 2\nC = "x"\nD = foo(1)\n')).toBe('A_B = 1\nC = "x"');
  });
  it.skipIf(!existsSync(OFFICIAL))('equals the values of RoboDK 5.9 robolink.py', () => {
    const official = constants(readFileSync(OFFICIAL, 'utf8')), stub = constants(ROBOLINK_STUB);
    expect(official.size).toBeGreaterThan(40);
    expect(Object.fromEntries(stub)).toEqual(Object.fromEntries(official));
  });
  const py = ['python', 'python3'].find((c) => spawnSync(c, ['-c', 'pass']).status === 0);
  it.skipIf(!py)('is valid Python', () => {
    const r = spawnSync(py!, ['-I', '-c', `${ROBOLINK_STUB}\nprint(INS_TYPE_MOVE, INS_TYPE_CODE, ITEM_TYPE_ROBOT)`], { encoding: 'utf8' });
    expect(r.stderr).toBe('');
    expect(r.stdout.trim()).toBe('0 8 2');
  });
});

describe('RoboDK exports with an embedded .rdk (rdk2vbs / rdk_export / server converter)', () => {
  const FILE = `${LIB}/Example-06.a-Pick and place - Mecademic.rdk`;
  const sig = (s: Station) => [...s.walk()].filter((i) => i !== s).map((i) => [i.type, i.name, getPos(i.poseAbs()).map((v) => v.toFixed(2)).join(','), i instanceof Program ? i.instructions().length : ''].join('|'));
  const exportOf = (bytes: Buffer) => JSON.stringify({ format: 'vbstation', station: { name: 'Example-06.a', type: 1, params: { source: 'rdk_export.py', robodkVersion: '5.9.4', rdkFile: { name: 'Example-06.a.rdk', data: bytes.toString('base64') } }, children: [] } });

  it.skipIf(!existsSync(FILE))('opening the .vbstation decodes the embedded RoboDK station (same items as a direct import)', async () => {
    const bytes = readFileSync(FILE);
    const direct = await importRdkBestEffort(bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength) as ArrayBuffer, 'x.rdk', new AssetStore());
    const app = makeApp('pickplace');
    await app.openFiles([new File([exportOf(bytes)], 'Example-06.a.vbstation')]);
    expect(app.station.params.rdkConvertedBy).toBe('RoboDK 5.9.4');
    expect(sig(app.station)).toEqual(sig(direct.station));
    expect(app.logs.some((l) => /^Converted by RoboDK 5\.9\.4/.test(l.text))).toBe(true);
    // every robot link mesh resolves in the app's asset store
    for (const r of app.station.itemsOfType<Robot>(ItemType.ROBOT)) for (const l of r.chain.links) for (const v of l.visuals) if (v.mesh) expect(app.assets.resolve(v.mesh), `${r.name} ${v.mesh}`).toBeTruthy();
  }, 120_000);

  it.skipIf(!existsSync(FILE))('a .rdk converted by the server (POST /convert/rdk) goes through the same decoder', async () => {
    const bytes = readFileSync(FILE);
    const app = makeApp('pickplace');
    const prevFetch = globalThis.fetch;
    const calls: string[] = [];
    window.history.replaceState({}, '', '/?server=ws://127.0.0.1:1');
    globalThis.fetch = (async (url: string) => { calls.push(String(url)); return new Response(exportOf(bytes), { status: 200, headers: { 'content-type': 'application/json' } }); }) as any;
    try {
      await app.openFiles([new File([bytes], 'Example-06.a-Pick and place - Mecademic.rdk')]);
    } finally { globalThis.fetch = prevFetch; window.history.replaceState({}, '', '/'); }
    expect(calls).toEqual(['http://127.0.0.1:1/convert/rdk']);
    expect(app.station.params.rdkConvertedBy).toBe('RoboDK 5.9.4');
    expect(app.logs.some((l) => /converted with RoboDK \(lossless\)/.test(l.text))).toBe(true);
    expect(app.station.itemsOfType(ItemType.ROBOT).length).toBeGreaterThan(0);
  }, 120_000);
});

describe('3D view', () => {
  it('a Webots world opens at its Viewpoint (eye, look direction, up)', async () => {
    const app = makeApp();
    const looks: any[] = [];
    stubRenderer(app, { lookAt: (...a: any[]) => looks.push(a), fitAll: () => looks.push(['fit']) });
    const st = new Station('vp');
    // mini.wbt: position 5 0 2 (m), orientation 0 0 1 pi -> looks along -X
    st.settings.viewpoint = { position: [5000, 0, 2000], orientation: [0, 0, 1, Math.PI], fieldOfView: 0.785398 };
    app.setStation(st);
    await tickMs(80);
    expect(looks.length).toBe(1);
    const [eye, target, up, fov, axis] = looks[0];
    expect(eye).toEqual([5000, 0, 2000]);
    expect(target[0]).toBeCloseTo(4000, 6); expect(target[1]).toBeCloseTo(0, 6); expect(target[2]).toBeCloseTo(2000, 6);
    expect(up[2]).toBeCloseTo(1, 9);
    expect(fov).toBeCloseTo(45, 4); expect(axis).toBe('horizontal');
    app.setStation(new Station('plain')); await tickMs(80);
    expect(looks.at(-1)).toEqual(['fit']);
  });
  it('lookAt: camera at the eye looking at the target; a horizontal fov is converted with the aspect ratio', () => {
    const camera = new THREE.PerspectiveCamera(45, 2, 5, 1e6);
    const controls = { target: new THREE.Vector3(), update: () => {} };
    const fake: any = { camera, controls, dirty: false };
    (SceneRenderer.prototype as any).lookAt.call(fake, [1000, 2000, 3000], [0, 0, 500], [0, 0, 1], 90, 'horizontal');
    expect(camera.fov).toBeCloseTo((2 * Math.atan(Math.tan(Math.PI / 4) / 2) * 180) / Math.PI, 9);
    const dir = new THREE.Vector3(0, 0, -1).applyQuaternion(camera.quaternion);
    const want = new THREE.Vector3(-1000, -2000, -2500).normalize();
    expect(dir.distanceTo(want)).toBeLessThan(1e-9);
    expect(controls.target.toArray()).toEqual([0, 0, 500]);
  });
  it('rebuildAll drops a removed subtree once (no stale id lookup)', () => {
    const st = new Station('s');
    const parent = st.addChild(new SceneObject('parent')); const child = parent.addChild(new SceneObject('child'));
    const removed: string[] = [];
    const entries = new Map<string, any>([[parent.id, { item: parent, root: {} }], [child.id, { item: child, root: {} }]]);
    parent.parent!.removeChild?.(parent);
    if (st.children.includes(parent)) (st as any).children.splice(st.children.indexOf(parent), 1);
    const fake: any = { station: st, entries, structural: new Set(), dirty: false, ensureEntry: () => {}, scene: { remove: () => {} }, unindexMeshes: (e: any) => removed.push(e.item.name), removeEntry: (SceneRenderer.prototype as any).removeEntry };
    (SceneRenderer.prototype as any).rebuildAll.call(fake);
    expect(entries.size).toBe(0);
    expect(removed).toEqual(['parent', 'child']);
    void child;
  });
  it('RoboDK Display POINTSIZE=4 COLOR=#FF771111 draws the object as a point cloud of its vertices', () => {
    const assets = new AssetStore();
    const tri = new Float32Array([0, 0, 0, 100, 0, 0, 0, 100, 0, 0, 0, 0, 0, 100, 0, 0, 0, 100]);
    assets.registerMesh('tri.stl', { positions: tri, normals: new Float32Array(tri.length), min: [0, 0, 0], max: [100, 100, 100], triangles: 2 });
    const st = new Station('s');
    const obj = st.addChild(new SceneObject('cloud'));
    obj.geometry = [{ mesh: 'tri.stl', origin: Array.from(transl(10, 20, 30)) }];
    const rdk = new Robolink(st);
    expect(rdk.Item('cloud').setParam('Display', 'POINTSIZE=4 COLOR=#FF771111')).toBe('OK');
    expect(obj.params.display).toEqual({ raw: 'POINTSIZE=4 COLOR=#FF771111', pointSize: 4, color: '#771111', alpha: 1 });
    const e: any = { item: obj, root: new THREE.Group(), sig: '' };
    (SceneRenderer.prototype as any).buildItem.call({ assets, frameSize: 100 }, obj, e);
    const pts: THREE.Points[] = []; const meshes: THREE.Mesh[] = [];
    e.root.traverse((o: any) => { if (o.isPoints) pts.push(o); if (o.isMesh) meshes.push(o); });
    expect(pts.length).toBe(1);
    const mat = pts[0].material as THREE.PointsMaterial;
    expect([mat.size, mat.sizeAttenuation, mat.color.getHexString()]).toEqual([4, false, '771111']);
    const pos = Array.from((pts[0].geometry.attributes.position as THREE.BufferAttribute).array);
    // the 6 vertices, offset by the geometry origin (10, 20, 30)
    const asSet = (a: ArrayLike<number>, o = [0, 0, 0]) => Array.from({ length: a.length / 3 }, (_, i) => [a[i * 3] + o[0], a[i * 3 + 1] + o[1], a[i * 3 + 2] + o[2]].join(',')).sort();
    expect(asSet(pos)).toEqual(asSet(tri, [10, 20, 30]));
    expect(meshes.every((m) => !m.visible)).toBe(true);
    // pointCloudOf on its own: vertices in the group frame
    const g = new THREE.Group(); const m = new THREE.Mesh(new THREE.BufferGeometry().setAttribute('position', new THREE.BufferAttribute(new Float32Array([1, 2, 3]), 3))); m.position.set(100, 0, 0); g.add(m);
    expect(Array.from(pointCloudOf(g, 2, '#ffffff').geometry.attributes.position.array)).toEqual([101, 2, 3]);
  });
});

describe('bottom panel', () => {
  it('Fleet tab shows the KPI groups contacts / traffic / process / equipment with the fleet manager values', async () => {
    const app = makeApp('orchard');
    const bottom = new BottomPanel(app);
    document.body.appendChild(bottom.el);
    const f = app.station.itemsOfType<FleetItem>(ItemType.FLEET)[0];
    const fm = app.fleetManager(f);
    for (let i = 0; i < 400; i++) fm.step(0.25);
    bottom.show('fleet');
    const groups = [...bottom.el.querySelectorAll<HTMLElement>('.kpi-group')];
    expect(groups.map((g) => g.dataset.kpi)).toEqual(['contacts', 'traffic', 'process', 'equipment']);
    const k = fm.kpis();
    const row = (g: HTMLElement, name: string) => [...g.querySelectorAll('.kv')].find((r) => r.children[0].textContent === name)?.children[1].textContent;
    expect(row(groups[1], 'replans')).toBe(String(k.traffic.replans));
    expect(row(groups[1], 'waiting')).toBe(`${k.traffic.waitSeconds.toFixed(1)} s`);
    expect(row(groups[0], 'robot–robot events')).toBe(String(k.contacts.robotRobot));
    const procRows = Object.entries(k.process).filter(([kind]) => kind !== 'equipment').flatMap(([kind, v]) => Object.keys(v).map((n) => `${kind}: ${n}`));
    expect([...groups[2].querySelectorAll('.kv')].map((r) => r.children[0].textContent)).toEqual(procRows.length ? procRows : ['—']);
    bottom.el.remove();
  });

  it('Camera tab fruit boxes follow the camera model when the view aspect differs from the camera', () => {
    // camera 640x480 (4:3), vertical fov 50; view 640x400 (16:10): pixel u,v of a world point in the view = K_view projection
    const cam = new Camera('cam'); cam.width = 640; cam.height = 480; cam.fov = 50; cam.fovAxis = 'vertical';
    const K = cameraIntrinsics(cam), Kv = cameraIntrinsics(cam, 640, 400);
    const pts: Array<[number, number, number]> = [[0, 0, 2000], [300, -200, 2500], [-450, 120, 1800]];
    const det = pts.map(([x, y, z]) => ({ u: K.cx + (K.fx * x) / z, v: K.cy + (K.fy * y) / z, w: (K.fx * 80) / z, h: (K.fy * 80) / z, ripe: 1 }));
    const boxes = fruitOverlayBoxes(cam, det, 640, 400);
    pts.forEach(([x, y, z], i) => {
      const b = boxes[i];
      expect(b.x + b.w / 2).toBeCloseTo(Kv.cx + (Kv.fx * x) / z, 6);
      expect(b.y + b.h / 2).toBeCloseTo(Kv.cy + (Kv.fy * y) / z, 6);
      expect(b.w).toBeCloseTo((Kv.fx * 80) / z, 6);
    });
    // and the view is rendered with that K: the three.js vertical fov of the view equals the camera model's
    expect(threeFovDeg(cam, 640, 400)).toBeCloseTo((2 * Math.atan(200 / Kv.fy) * 180) / Math.PI, 9);
  });
});

describe('zone dialog', () => {
  it('creates lane and depot zones with capacity and reconfiguration time', async () => {
    const app: App = makeApp('pickplace');
    let p = zoneDialog(app); setField('Name', 'Lane A'); setField('Kind', 'lane'); setField('Capacity (robots inside a lane / depot slots)', 2); clickOk(); await p;
    p = zoneDialog(app); setField('Name', 'Depot'); setField('Kind', 'depot'); setField('Capacity (robots inside a lane / depot slots)', 3); setField('Reconfiguration time per swap (s, depot)', 240); clickOk(); await p;
    expect(dialogTitle).toBeTypeOf('function');
    const zones = app.station.itemsOfType<ZoneItem>(ItemType.ZONE);
    expect(zones.map((z) => [z.name, z.kind, z.capacity])).toEqual([['Lane A', 'lane', 2], ['Depot', 'depot', 3]]);
    expect(zones[1].getParam('reconfigSeconds')).toBe(240);
    expect(zones[0].getParam('reconfigSeconds')).toBeUndefined();
  });
});

describe('API requests of the vision track (T9)', () => {
  it("camera item setParam('Settings') returns RoboDK's settings string and applies a new one; Vision_Run = Cam2D_Detect", async () => {
    const st = new Station('cam');
    const cam = st.addChild(new Camera('Cell camera'));
    cam.width = 640; cam.height = 480; cam.fov = 40; cam.near = 10; cam.far = 3000;
    const rdk = new Robolink(st);
    const item = rdk.Item('Cell camera');
    expect(item.setParam('Settings')).toBe('FOV=40 FAR_LENGTH=3000 NEAR_LENGTH=10 SIZE=640x480');
    expect(item.setParam('Settings', 'FOV=32 FAR_LENGTH=5000 SIZE=1280x720')).toBe('FOV=32 FAR_LENGTH=5000 NEAR_LENGTH=10 SIZE=1280x720');
    expect([cam.fov, cam.far, cam.width, cam.height, cam.fovAxis]).toEqual([32, 5000, 1280, 720, 'vertical']);
    // no vision stack on the camera: both report the same error
    await expect(rdk.Cam2D_Detect(item)).rejects.toThrow('camera "Cell camera" has no vision st');
    await expect(rdk.Vision_Run(item)).rejects.toThrow('camera "Cell camera" has no vision st');
    await expect(rdk.Vision_Run()).rejects.toThrow('Cam2D_Detect: no camera with a vision stack');
    const py = readFileSync(join(__dirname, '../python/robodk/robolink.py'), 'utf8');
    expect(py).toMatch(/def Vision_Run\(self, cam_handle=0, robot=None\):\s+"""[^"]+"""\s+return self\._call\("Vision_Run"/);
  });
});

describe('mission dialog process settings (T8)', () => {
  it('follow-up, tank, dose, fruit kept and bin capacity go to the mission settings (0 = process default)', async () => {
    const app = makeApp('orchard');
    const { missionDialog } = await import('../src/ui/dialogs');
    let p = missionDialog(app);
    setField('Name', 'Scout + spray'); setField('Type', 'scout'); setField('Follow-up task where problems are found (scout)', 'spray');
    setField('Tank (L, spray / irrigate; 0 = default)', 300); setField('Dose (L per m³ of canopy; 0 = default 0.07)', 0.12); setField('Plan now (create fleet tasks)', false);
    clickOk(); await p;
    const m = app.station.itemsOfType<any>(ItemType.MISSION).find((x) => x.name === 'Scout + spray');
    expect(m.settings).toMatchObject({ followUp: 'spray', tankL: 300, doseLPerM3: 0.12 });
    expect(m.settings.binCapacityKg).toBeUndefined(); expect(m.settings.keepFruitPerPlant).toBeUndefined();
    p = missionDialog(app);
    setField('Name', 'Harvest'); setField('Bin capacity (kg, harvest; 0 = default 400)', 250); setField('Fruit kept per plant (thin; 0 = default)', 20); setField('Plan now (create fleet tasks)', false);
    clickOk(); await p;
    const h2 = app.station.itemsOfType<any>(ItemType.MISSION).find((x) => x.name === 'Harvest');
    expect(h2.settings).toMatchObject({ binCapacityKg: 250, keepFruitPerPlant: 20 });
    expect(h2.settings.followUp).toBeUndefined();
  });
});

describe('File › MURKA examples (C4)', () => {
  it('the submenu lists the MURKA entries; agro experiment exp5 downloads its KPI JSON (murca_experiment schema)', async () => {
    const app = makeApp('pickplace');
    const { murkaMenu, agroExperimentDialog } = await import('../src/ui/murka_ui');
    expect(murkaMenu(app).filter((e) => !e.separator).map((e) => e.label)).toEqual([
      'Retail store mission (robot-retailer-murka)…', 'Stop retail mission', 'Orchard: Mission B (scripted fleet, orchard_demo.wbt)…', 'Orchard: benchmark S1…', 'Agro experiments exp1–exp5 (KPI JSON)…', 'Play scripted fleet (Mission B or a script JSON)…']);
    const m: any = await import('../src/murka/index');
    const ref = m.runAgroExperiment('exp5', { runs: 1 });
    const n0 = downloads.length;
    const p = agroExperimentDialog(app);
    await waitFor(() => document.querySelector('.overlay') !== null);
    setField('Experiment', 'exp5'); setField('Runs per point (0 = the Python default)', 1);
    clickOk(); await p;
    expect(downloads.slice(n0).map((d) => d.name)).toEqual(Object.keys(ref.files));
    expect(app.logs.at(-1)!.text).toBe(`MURKA exp5: ${Object.keys(ref.files).join(', ')}`);
  }, 300_000);
});

describe('App.runToEnd with asynchronous script runners', () => {
  const FILE = `${LIB}/Example-06.b-Pick and place 2 tables.rdk`;
  it.skipIf(!existsSync(FILE))('a run calling scripts ends like a run without runners, whether the runner declines at once or answers later (06.b, AirValve_On/Off)', async () => {
    const { registerScriptRunner } = await import('../src/core/motion/runtime');
    // the Vision panel of an earlier test registered the global vision runner: remove it, the baseline has no runner
    (await import('../src/vision/program_vision')).installVisionRunnerGlobal()();
    const play = async (runner: ((ctx: any) => Promise<boolean | void>) | null, sliced = false) => {
      const app = makeApp();
      const b = readFileSync(FILE);
      await app.openFiles([new File([b], 'Example-06.b-Pick and place 2 tables.rdk')]);
      const calls: string[] = [];
      const off = runner ? registerScriptRunner(((ctx: any) => { calls.push(ctx.name); return runner(ctx); }) as any) : () => {};
      try {
        const p = app.station.itemsOfType<Program>(ItemType.PROGRAM).find((x) => x.name === 'Pick and place')!;
        const h: any = app.runProgram(p);
        if (sliced) { for (let t = 0; t < 600 && app.activeRuns().length; t += 0.25) await app.runToEnd(0.25); }
        else {
          const r = app.runToEnd(600);
          if (runner) expect(r).toBeInstanceOf(Promise); else expect(r).toBeUndefined();
          await r;
        }
        const ball = app.station.itemsOfType(ItemType.OBJECT).filter((o) => /^ball/i.test(o.name)).map((o) => Array.from(o.poseAbs()).slice(12, 15));
        return { status: h.status, cycle: h.endTime - h.startTime, joints: (p.robot() as Robot).joints(), ball, calls };
      } finally { off(); }
    };
    const none = await play(null);
    const declines = await play(async () => false);
    const later = await play(() => new Promise((r) => setTimeout(() => r(undefined), 15)));
    // 0.25 s slices (the e2e sampling loop): a wait left open at the end of a slice is settled by the next one
    const sliced = await play(() => new Promise((r) => setTimeout(() => r(undefined), 5)), true);
    expect(none.status).toBe('done');
    expect(declines.calls.length).toBeGreaterThan(3);
    expect(later.calls).toEqual(declines.calls);
    for (const r of [declines, later, sliced]) {
      expect(r.status).toBe('done');
      expect(Math.abs(r.cycle - none.cycle)).toBeLessThan(1e-9);
      r.joints.forEach((q, i) => expect(q).toBeCloseTo(none.joints[i], 9));
      r.ball.forEach((p, i) => p.forEach((v, k) => expect(v).toBeCloseTo(none.ball[i][k], 6)));
    }
  }, 300_000);
});

describe('Connect › ROS 2 via rosbridge', () => {
  it("offers the simulator-backend mode 'sim' (RosBridge mode for the MURKA Webots interface)", async () => {
    const app = makeApp('pickplace');
    const { MenuBar } = await import('../src/ui/menu');
    const menu = new MenuBar(app, () => {});
    const p = menu.rosDialog();
    const sel = currentDialog().querySelector('select') as HTMLSelectElement;
    expect([...sel.options].map((o) => o.value)).toEqual(['both', 'publish', 'follow', 'sim']);
    clickCancel(); await p;
  });
});
