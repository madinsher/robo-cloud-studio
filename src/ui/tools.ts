/** Extra UI tools: station tabs, collision map, measurements, camera parameters, video and 3D HTML export. */
import { App } from '../app';
import { Item, ItemType, Camera as CameraItem } from '../core/items/item';
import { Robot } from '../core/items/robot';
import { h, clear, dialog, toast, downloadText, downloadBlob, fmt, formField, pickFiles } from './dom';
import { getCollisionMap, setCollisionPair, checkCollisionsMapped } from '../core/collision/collision';
import { getPos, distance, rotationAngle, RAD } from '../core/math/pose';

/** Tab bar listing open stations. */
export class StationTabs {
  el: HTMLElement;
  constructor(readonly app: App) {
    this.el = h('div', { class: 'station-tabs' });
    app.events.on('stationChanged', () => this.render());
    this.render();
  }
  render(): void {
    clear(this.el);
    for (const st of this.app.stations) {
      const tab = h('button', { class: `stab ${st === this.app.station ? 'active' : ''}`, onClick: () => this.app.setStation(st) }, st.name,
        this.app.stations.length > 1 ? h('span', { class: 'stab-close', title: 'Close station', onClick: (e: MouseEvent) => { e.stopPropagation(); if (st === this.app.station) this.app.closeStation(); else { const i = this.app.stations.indexOf(st); if (i >= 0) this.app.stations.splice(i, 1); this.render(); } } }, '×') : null);
      this.el.appendChild(tab);
    }
    this.el.appendChild(h('button', { class: 'stab add', title: 'New station tab', onClick: () => this.app.addStation(`Station ${this.app.stations.length + 1}`) }, '+'));
  }
}

/** Collision map dialog: matrix of item pairs with check/uncheck (RoboDK Tools > Collision map). */
export async function collisionMapDialog(app: App): Promise<void> {
  const items = [...app.station.walk()].filter((i) => i !== app.station && [ItemType.ROBOT, ItemType.OBJECT, ItemType.TOOL, ItemType.MOBILE_ROBOT, ItemType.COMPONENT].includes(i.type));
  if (items.length < 2) return toast('At least two collidable items are required', 'warn');
  const map = getCollisionMap(app.station);
  const isDisabled = (a: Item, b: Item) => map.disabled.some((p) => (p[0] === a.id && p[2] === b.id) || (p[0] === b.id && p[2] === a.id));
  const table = h('table', { class: 'grid cmap' });
  const head = h('tr', null, h('th', null, ''), ...items.map((i) => h('th', { class: 'rot' }, h('div', null, i.name))));
  table.appendChild(head);
  items.forEach((a, ai) => {
    const row = h('tr', null, h('th', null, a.name));
    items.forEach((b, bi) => {
      if (bi <= ai) { row.appendChild(h('td', { class: 'na' }, bi === ai ? '—' : '')); return; }
      const cb = h('input', { type: 'checkbox', checked: !isDisabled(a, b), title: `${a.name} × ${b.name}` }) as HTMLInputElement;
      cb.addEventListener('change', () => setCollisionPair(app.station, a, b, -1, -1, cb.checked));
      row.appendChild(h('td', null, cb));
    });
    table.appendChild(row);
  });
  const active = formField({ key: 'active', label: 'Collision checking active', type: 'checkbox', value: map.active }, (v) => { map.active = v; app.station.notify('collisionMap'); });
  const now = h('div', { class: 'hint' });
  const refresh = () => { const pairs = checkCollisionsMapped(app.station, { assets: app.assets }); now.textContent = pairs.length ? `Current collisions: ${pairs.map((p) => `${p.a.item.name} × ${p.b.item.name}`).join(', ')}` : 'No collisions in the current state'; };
  refresh();
  await dialog('Collision map', [], { width: Math.min(900, 300 + items.length * 40), body: h('div', null, active.el, h('div', { class: 'scroll-x' }, table), h('div', { class: 'btn-row' }, h('button', { class: 'btn', onClick: refresh }, 'Check now'), h('button', { class: 'btn', onClick: () => { map.disabled = []; map.enabled = []; app.station.notify('collisionMap'); table.querySelectorAll('input').forEach((c) => ((c as HTMLInputElement).checked = true)); } }, 'Reset (check all)')), now), okLabel: 'Close' });
  app.snapshot();
}

/** Measurement tool: distance and angle between two selected items (or item and TCP). */
export function measureDialog(app: App): void {
  const sel = app.station.selection;
  if (sel.length < 2) { const r = app.activeRobot; if (sel.length === 1 && r) return showMeasure(app, sel[0], r, true); return toast('Select two items (Shift+click) to measure', 'warn'); }
  showMeasure(app, sel[0], sel[1], false);
}
function showMeasure(app: App, a: Item, b: Item, tcp: boolean) {
  const pa = a.poseAbs();
  const pb = tcp && b instanceof Robot ? b.poseTCPAbs() : b.poseAbs();
  const d = distance(getPos(pa), getPos(pb));
  const dv = [pb[12] - pa[12], pb[13] - pa[13], pb[14] - pa[14]];
  const ang = rotationAngle(pa, pb) * RAD;
  const rows = [['Distance', `${fmt(d, 3)} mm`], ['ΔX / ΔY / ΔZ', `${fmt(dv[0], 3)} / ${fmt(dv[1], 3)} / ${fmt(dv[2], 3)} mm`], ['Angle between frames', `${fmt(ang, 3)}°`], ['A', `${a.name} (${getPos(pa).map((v) => fmt(v, 1)).join(', ')})`], ['B', `${tcp ? b.name + ' TCP' : b.name} (${getPos(pb).map((v) => fmt(v, 1)).join(', ')})`]];
  dialog('Measure', [], { body: h('div', { class: 'kv' }, ...rows.map(([k, v]) => h('div', { class: 'kv-row' }, h('span', { class: 'k' }, k), h('span', { class: 'v' }, v)))), okLabel: 'Close' });
}

/** Camera parameters dialog (RoboDK Cam2D_SetParams). */
export async function cameraDialog(app: App, cam: CameraItem): Promise<void> {
  const r = await dialog<any>(`Camera ${cam.name}`, [
    { key: 'kind', label: 'Type', type: 'select', value: cam.kind, options: ['rgb', 'depth', 'lidar2d', 'lidar3d'].map((k) => ({ value: k, label: k })) },
    { key: 'fov', label: 'Field of view (deg)', type: 'number', value: cam.fov },
    { key: 'w', label: 'Width (px)', type: 'number', value: cam.width },
    { key: 'h', label: 'Height (px)', type: 'number', value: cam.height },
    { key: 'near', label: 'Near (mm)', type: 'number', value: cam.near },
    { key: 'far', label: 'Far (mm)', type: 'number', value: cam.far },
  ]);
  if (!r) return;
  app.cmd(() => { cam.kind = r.kind; cam.fov = r.fov; cam.width = r.w; cam.height = r.h; cam.near = r.near; cam.far = r.far; cam.notify('camera'); });
}

/** Record the 3D view to a WebM video while the simulation plays. */
export class VideoRecorder {
  private rec: MediaRecorder | null = null;
  private chunks: Blob[] = [];
  constructor(readonly app: App) {}
  get recording(): boolean { return !!this.rec && this.rec.state === 'recording'; }
  start(fps = 30): void {
    const canvas = this.app.renderer.renderer.domElement;
    const stream = (canvas as any).captureStream?.(fps);
    if (!stream || typeof MediaRecorder === 'undefined') return toast('Video capture is not supported in this browser', 'error');
    this.chunks = [];
    const mime = ['video/webm;codecs=vp9', 'video/webm;codecs=vp8', 'video/webm'].find((m) => MediaRecorder.isTypeSupported(m)) ?? '';
    this.rec = new MediaRecorder(stream, mime ? { mimeType: mime, videoBitsPerSecond: 8_000_000 } : undefined);
    this.rec.ondataavailable = (e) => { if (e.data.size) this.chunks.push(e.data); };
    this.rec.onstop = () => { downloadBlob(`${this.app.station.name.replace(/\W+/g, '_')}.webm`, new Blob(this.chunks, { type: 'video/webm' })); toast('Video saved', 'ok'); };
    this.rec.start(250);
    toast('Recording… use Tools > Stop video recording to finish', 'info', 5000);
  }
  stop(): void { this.rec?.stop(); this.rec = null; }
}

/** Export the current scene as a self-contained 3D HTML viewer (glTF embedded, three.js from CDN). */
export async function exportHtml3D(app: App): Promise<void> {
  const { GLTFExporter } = await import('three/addons/exporters/GLTFExporter.js');
  const exporter = new GLTFExporter();
  const scene = app.renderer.scene;
  const gizmo = app.renderer.transform.getHelper();
  const wasVisible = gizmo.visible;
  gizmo.visible = false;
  const glb = await new Promise<ArrayBuffer>((resolve, reject) => exporter.parse(scene, (r) => resolve(r as ArrayBuffer), (e) => reject(e), { binary: true, onlyVisible: true }));
  gizmo.visible = wasVisible;
  const b64 = btoa(String.fromCharCode(...new Uint8Array(glb).subarray(0, 0)) + Array.from(new Uint8Array(glb), (b) => String.fromCharCode(b)).join(''));
  const html = `<!doctype html><html><head><meta charset="utf-8"><title>${app.station.name} — 3D</title><style>html,body{margin:0;height:100%;background:#1e2229;color:#ddd;font-family:sans-serif}#info{position:fixed;left:10px;top:10px;font-size:13px}canvas{display:block}</style></head><body><div id="info">${app.station.name} — exported from VerticalBot Studio. Drag to orbit, wheel to zoom.</div>
<script type="importmap">{"imports":{"three":"https://cdn.jsdelivr.net/npm/three@0.170.0/build/three.module.js","three/addons/":"https://cdn.jsdelivr.net/npm/three@0.170.0/examples/jsm/"}}</script>
<script type="module">
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
const data = Uint8Array.from(atob("${b64}"), c => c.charCodeAt(0)).buffer;
const renderer = new THREE.WebGLRenderer({ antialias: true }); renderer.setSize(innerWidth, innerHeight); document.body.appendChild(renderer.domElement);
const scene = new THREE.Scene(); scene.background = new THREE.Color(0x1e2229);
const camera = new THREE.PerspectiveCamera(45, innerWidth / innerHeight, 1, 1e6); camera.up.set(0, 0, 1);
const controls = new OrbitControls(camera, renderer.domElement);
scene.add(new THREE.HemisphereLight(0xdfe8ff, 0x30363d, 1.0)); const sun = new THREE.DirectionalLight(0xffffff, 1.5); sun.position.set(2, -1.5, 3); scene.add(sun);
new GLTFLoader().parse(data, '', (g) => { scene.add(g.scene); const box = new THREE.Box3().setFromObject(g.scene); const c = box.getCenter(new THREE.Vector3()); const s = box.getSize(new THREE.Vector3()).length(); controls.target.copy(c); camera.position.set(c.x + s * 0.7, c.y - s * 0.9, c.z + s * 0.6); camera.far = s * 20; camera.updateProjectionMatrix(); });
addEventListener('resize', () => { camera.aspect = innerWidth / innerHeight; camera.updateProjectionMatrix(); renderer.setSize(innerWidth, innerHeight); });
renderer.setAnimationLoop(() => { controls.update(); renderer.render(scene, camera); });
</script></body></html>`;
  downloadText(`${app.station.name.replace(/\W+/g, '_')}_3d.html`, html, 'text/html');
  toast('3D HTML exported', 'ok');
}

/** Export the scene as GLB (for other tools). */
export async function exportGlb(app: App): Promise<void> {
  const { GLTFExporter } = await import('three/addons/exporters/GLTFExporter.js');
  const exporter = new GLTFExporter();
  const glb = await new Promise<ArrayBuffer>((resolve, reject) => exporter.parse(app.renderer.scene, (r) => resolve(r as ArrayBuffer), (e) => reject(e), { binary: true, onlyVisible: true }));
  downloadBlob(`${app.station.name.replace(/\W+/g, '_')}.glb`, new Blob([glb], { type: 'model/gltf-binary' }));
}


/** Export the current program simulation as an animated glTF (Blender › File › Import › glTF 2.0). */
export async function exportAnimationGltf(app: App): Promise<void> {
  const prog = app.activeProgram;
  if (!prog) return toast('Select or create a program first', 'warn');
  const r = await dialog<{ fps: number; helpers: boolean }>('Export animation (glTF for Blender)', [
    { key: 'fps', label: 'Keyframes per second', type: 'number', value: 30, min: 1, max: 120 },
    { key: 'helpers', label: 'Include frame / target helpers', type: 'checkbox', value: false },
  ], { okLabel: 'Export', body: h('p', { class: 'hint' }, 'The program is simulated from start to end; every robot link, tool and moved object gets position/rotation keyframes. Units are converted to metres (root node scaled 0.001). Blender: File › Import › glTF 2.0, then play the timeline.') });
  if (!r) return;
  app.previewProgram();
  if (!app.sim.duration) return toast('Program has no duration (compile problems?)', 'warn');
  toast('Sampling simulation…', 'info', 2500);
  const { exportAnimatedGLB } = await import('../io/export/gltf_anim');
  const res = await exportAnimatedGLB(app.renderer, app.sim, { fps: r.fps || 30, name: prog.name, hideHelpers: !r.helpers });
  downloadBlob(`${app.station.name.replace(/\W+/g, '_')}_${prog.name.replace(/\W+/g, '_')}_anim.glb`, new Blob([res.glb], { type: 'model/gltf-binary' }));
  app.log(`Animated glTF: ${res.frames} frames, ${res.duration.toFixed(2)} s, ${res.tracks} tracks`);
  toast(`Animation exported (${res.frames} frames, ${res.tracks} tracks)`, 'ok');
}

/** Export the active robot or the whole station as a URDF package (zip with meshes). */
export async function exportUrdfPackage(app: App): Promise<void> {
  const robots = app.station.itemsOfType<Robot>(ItemType.ROBOT);
  const r = await dialog<{ what: string; pkg: string }>('Export URDF package', [
    { key: 'what', label: 'Content', type: 'select', value: app.activeRobot ? app.activeRobot.id : 'station', options: [{ value: 'station', label: `Whole station (${robots.length} robots + objects)` }, ...robots.map((x) => ({ value: x.id, label: `Robot: ${x.name}` }))] },
    { key: 'pkg', label: 'ROS package name (optional)', type: 'text', value: '' },
  ], { okLabel: 'Export', body: h('p', { class: 'hint' }, 'Writes urdf/*.urdf + meshes/*.stl (metres) + package.xml in a zip. Use it in ROS 2 (robot_state_publisher, RViz, MoveIt), Gazebo, or Blender via the Phobos add-on.') });
  if (!r) return;
  const { exportRobotURDF, exportStationURDF, packageZip } = await import('../io/urdf/urdf_export');
  const target = r.what === 'station' ? null : (app.station.findById(r.what) as Robot | null);
  const res = target ? exportRobotURDF(target, app.assets, { packageName: r.pkg || undefined }) : exportStationURDF(app.station, app.assets, { packageName: r.pkg || undefined });
  for (const w of res.warnings.slice(0, 8)) app.log(w, 'warn');
  const zip = packageZip(res);
  downloadBlob(`${res.packageName}.zip`, new Blob([zip.buffer.slice(zip.byteOffset, zip.byteOffset + zip.byteLength) as ArrayBuffer], { type: 'application/zip' }));
  app.log(`URDF package ${res.packageName}: ${Object.keys(res.files).length} files`);
  toast(`URDF package exported (${Object.keys(res.files).length} files)`, 'ok');
}

/** Import RoboDK post processors (one or many .py files, e.g. the whole RoboDK/Posts folder). */
export async function importRoboDKPosts(app: App): Promise<void> {
  const files = await pickFiles('.py', true);
  if (!files.length) return;
  const { importPostFiles, listUserPosts } = await import('../posts/user_posts');
  const r = await importPostFiles(files);
  app.log(`RoboDK posts: ${r.added.length} imported${r.skipped.length ? `, ${r.skipped.length} skipped (no RobotPost class): ${r.skipped.slice(0, 5).join(', ')}` : ''}; ${listUserPosts().length} user posts available in the export dialog`);
  toast(`${r.added.length} post processors imported`, r.added.length ? 'ok' : 'warn');
}


/** Save the station as .vbstation with the active program sampled into an animation block for the Blender add-on. */
export async function saveForBlender(app: App): Promise<void> {
  const { saveStation } = await import('../io/station-file');
  const file: any = saveStation(app.station, app.assets);
  const prog = app.activeProgram;
  if (prog) {
    app.previewProgram();
    const sim = app.sim;
    const dt = 1 / 30;
    const robots = app.station.itemsOfType<Robot>(ItemType.ROBOT);
    const anim: any = { program: prog.name, duration: sim.duration, dt, robots: {}, items: {} };
    for (const r of robots) anim.robots[r.id] = sim.jointsList(r.id, dt);
    // items whose absolute pose changes during the program (attached objects, mobile robots, tools)
    const movers = app.station.itemsOfType(ItemType.OBJECT).concat(app.station.itemsOfType(ItemType.MOBILE_ROBOT), app.station.itemsOfType(ItemType.TOOL));
    const frames = Math.max(1, Math.ceil(sim.duration / dt) + 1);
    const series = new Map<string, number[][]>(movers.map((m) => [m.id, []]));
    const saved = sim.time;
    for (let f = 0; f < frames; f++) { sim.seek(f * dt); for (const m of movers) series.get(m.id)!.push(Array.from(m.poseAbs())); }
    sim.seek(saved);
    for (const [id, poses] of series) { const p0 = poses[0]; if (poses.some((p) => p.some((v, i) => Math.abs(v - p0[i]) > 1e-6))) anim.items[id] = poses; }
    file.animation = anim;
  }
  downloadText(`${app.station.name.replace(/\W+/g, '_')}_blender.vbstation`, JSON.stringify(file), 'application/json');
  toast(prog ? `Saved with ${prog.name} animation for the Blender add-on` : 'Saved for the Blender add-on (no active program: no animation)', 'ok', 5000);
}
