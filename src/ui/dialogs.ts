import { App } from '../app';
import { Item, ItemType, Frame, Target, Tool, SceneObject, Folder, Station } from '../core/items/item';
import { Robot } from '../core/items/robot';
import { Program, Instruction } from '../core/items/program';
import { MobileRobot, MapItem, ZoneItem } from '../mobile/items';
import { FleetItem } from '../fleet/fleet';
import { FieldItem, MissionItem, CropRow, cropParams, CropType, CROP_PRESETS, MissionType } from '../agri/items';
import { generateOrchard, buildFieldMap, rectPolygon, makeHeadlandZones, makeCanopyZones } from '../agri/orchard';
import { planMission, generateHarvestProgram } from '../agri/missions';
import { Component, makeConveyor, makeFeeder, makeProcess, makeSink, makeBuffer } from '../vc/component';
import { dialog, MenuEntry, toast, downloadText, h, pickFiles } from './dom';
import { transl, mul, rotz, DEG, poseToXyzrpw, xyzrpwToPose, multiply, invert, getPos } from '../core/math/pose';
import { ROBOT_LIBRARY } from '../core/items/library';
import { ONLINE_ROBOT_LIBRARY, ONLINE_REPOS } from '../io/library/online_library';

/** Context menu entries for an item (tree & viewport). */
export function itemContextMenu(app: App, item: Item): MenuEntry[] {
  const sel = app.station.selection.length > 1 ? app.station.selection : [item];
  const entries: MenuEntry[] = [];
  const add = (e: MenuEntry) => entries.push(e);
  const addMenu: MenuEntry[] = [
    { label: 'Reference frame', action: () => app.addFrame(item instanceof Robot || item instanceof Tool ? app.station : item) },
    { label: 'Target (at robot TCP)', action: () => app.addTarget(item instanceof Frame ? item : undefined) },
    { label: 'Program', action: () => app.addProgram() },
    { separator: true },
    { label: 'Robot from library…', action: () => robotLibraryDialog(app) },
    { label: 'Robot from online library (ROS-Industrial)…', action: () => onlineLibraryDialog(app) },
    { label: 'Mobile robot…', action: () => mobileRobotDialog(app) },
    { label: 'Tool (on active robot)', action: () => { const r = app.activeRobot; if (!r) return toast('Select a robot', 'warn'); app.cmd(() => { const t = new Tool(`Tool ${r.tools().length + 1}`); t.setPoseTool(transl(0, 0, 150)); r.addChild(t); r.setTool(t); app.select(t); }); } },
    { separator: true },
    { label: 'Box', action: () => app.addObjectPrimitive('box') },
    { label: 'Cylinder', action: () => app.addObjectPrimitive('cylinder', [150, 150, 600]) },
    { label: 'Sphere', action: () => app.addObjectPrimitive('sphere', [200]) },
    { label: 'Folder', action: () => app.cmd(() => item.addChild(new Folder('Folder'))) },
    { label: 'Camera (on selected item)', action: () => app.cmd(() => { const { Camera } = require_items(); const c = new Camera('Camera'); item.addChild(c); app.select(c); }) },
    { separator: true },
    { label: 'Field / orchard…', action: () => orchardDialog(app) },
    { label: 'Fleet…', action: () => fleetDialog(app) },
    { label: 'Mission…', action: () => missionDialog(app) },
    { label: 'Map (occupancy grid)…', action: () => mapDialog(app) },
    { label: 'Zone (charging / no-go)…', action: () => zoneDialog(app) },
    { separator: true },
    { label: 'Process component…', action: () => componentDialog(app) },
  ];
  add({ label: 'Add', children: addMenu });
  if (item instanceof Target) {
    add({ label: 'Move robot here (MoveJ)', action: () => app.moveRobotTo(item) });
    add({ label: 'Move robot here (MoveL)', action: () => app.moveRobotTo(item, true) });
    add({ label: 'Teach current position', action: () => app.cmd(() => { const r = app.activeRobot; if (!r) return; item.setPoseAbs(r.poseTCPAbs()); item.setJoints(r.joints()); }) });
    add({ label: item.isJointTarget ? 'Set as Cartesian target' : 'Set as Joint target', action: () => app.cmd(() => (item.isJointTarget ? item.setAsCartesianTarget() : item.setAsJointTarget())) });
    add({ label: 'Add MoveJ to program', action: () => { const p = app.activeProgram ?? app.addProgram(); app.cmd(() => p.addMoveJ(item)); app.previewProgram(); } });
    add({ label: 'Add MoveL to program', action: () => { const p = app.activeProgram ?? app.addProgram(); app.cmd(() => p.addMoveL(item)); app.previewProgram(); } });
  }
  if (item instanceof Robot) {
    add({ label: 'Set as active robot', action: () => app.setActiveRobot(item) });
    add({ label: 'Home', action: () => app.cmd(() => item.setJoints(item.jointsHome())) });
    add({ label: 'Generate picking program for nearby fruit…', action: () => harvestArmDialog(app, item) });
    add({ label: 'Follow curve / points of an object…', action: () => curveFollowDialog(app, item) });
    add({ label: 'Move with external axes (rail/gantry)…', action: () => railIKDialog(app, item) });
  }
  if (item instanceof Frame) add({ label: 'Set as active reference', action: () => { const r = app.activeRobot; if (r) app.cmd(() => r.setFrame(item)); } });
  if (item instanceof Tool && item.parent instanceof Robot) add({ label: 'Set as active tool', action: () => app.cmd(() => (item.parent as Robot).setTool(item)) });
  if (item instanceof Program) {
    add({ label: 'Run', action: () => app.runProgram(item) });
    add({ label: 'Validate', action: () => { const r = app.sim.compile(item); toast(r.ok ? `OK: ${r.duration.toFixed(1)} s, ${(r.distance / 1000).toFixed(2)} m` : r.problems.map((p) => p.message).join('; '), r.ok ? 'ok' : 'error', 6000); } });
    add({ label: 'Export (post processor)…', action: () => exportDialog(app, item) });
  }
  if (item instanceof FieldItem) {
    add({ label: 'Regenerate rows…', action: () => orchardDialog(app, item) });
    add({ label: 'Build navigation map', action: () => app.cmd(() => { app.station.addChild(buildFieldMap(item)); makeHeadlandZones(item); makeCanopyZones(item); }) });
  }
  if (item instanceof MissionItem) add({ label: 'Plan mission (create fleet tasks)', action: () => runMissionPlan(app, item) });
  if (item instanceof MobileRobot) {
    add({ label: 'Set home here', action: () => app.cmd(() => { item.home = { x: item.state.x, y: item.state.y, theta: item.state.theta }; }) });
    add({ label: 'Navigate to point…', action: () => navigateDialog(app, item) });
  }
  if (item instanceof FleetItem) add({ label: 'Add all mobile robots to fleet', action: () => app.cmd(() => { const fm = app.fleetManager(item); for (const m of app.station.itemsOfType<MobileRobot>(ItemType.MOBILE_ROBOT)) fm.addRobot(m); }) });
  add({ separator: true });
  add({ label: 'Focus', action: () => app.renderer.focusItem(item) });
  add({ label: item.visible ? 'Hide' : 'Show', action: () => app.cmd(() => item.setVisible(!item.visible)) });
  add({ label: 'Rename…', action: async () => { const r = await dialog<{ name: string }>('Rename', [{ key: 'name', label: 'Name', type: 'text', value: item.name }]); if (r?.name) app.cmd(() => item.setName(r.name)); } });
  if (item.type !== ItemType.STATION) add({ label: `Delete${sel.length > 1 ? ` (${sel.length})` : ''}`, shortcut: 'Del', action: () => app.deleteItems(sel) });
  return entries;
}

export async function robotLibraryDialog(app: App): Promise<void> {
  const cats = [...new Set(ROBOT_LIBRARY.map((r) => r.category))];
  const r = await dialog<{ id: string }>('Add robot from library', [
    { key: 'id', label: 'Robot', type: 'select', options: cats.flatMap((c) => ROBOT_LIBRARY.filter((x) => x.category === c).map((x) => ({ value: x.id, label: `[${c}] ${x.name} — ${x.dof} DOF, ${x.reach} mm, ${x.payload} kg${x.approximate ? ' (approx.)' : ''}` }))) },
  ], { width: 560, body: h('p', { class: 'hint' }, 'Robots marked approx. use representative DH geometry. Import a URDF (drag & drop) for exact kinematics and meshes.') });
  if (r) app.addRobotFromLibrary(r.id);
}

export async function onlineLibraryDialog(app: App): Promise<void> {
  const brands = [...new Set(ONLINE_ROBOT_LIBRARY.map((r) => r.brand))];
  const label = (x: (typeof ONLINE_ROBOT_LIBRARY)[number]) => `[${x.brand === 'Generic' ? 'Other' : x.brand}] ${x.name} — ${x.dof} DOF, ${x.reach} mm, ${x.payload} kg`;
  const status = h('div', { class: 'hint', style: { minHeight: '1.2em' } });
  const r = await dialog<{ id: string; meshes: boolean }>('Add robot from online library', [
    { key: 'id', label: 'Robot', type: 'select', options: brands.flatMap((b) => ONLINE_ROBOT_LIBRARY.filter((x) => x.brand === b).map((x) => ({ value: x.id, label: label(x) }))) },
    { key: 'meshes', label: 'Download 3D meshes', type: 'checkbox', value: true },
  ], {
    width: 620,
    okLabel: 'Download',
    body: h('div', null,
      h('p', { class: 'hint' }, `${ONLINE_ROBOT_LIBRARY.length} robots with exact kinematics and meshes from the open URDF packages published by ROS-Industrial and the vendors (${ONLINE_REPOS.map((x) => `${x.owner}/${x.repo}`).filter((v, i, a) => a.indexOf(v) === i).length} GitHub repositories, BSD/Apache licences). Requires Internet access; files are fetched directly by the browser.`),
      status),
  });
  if (!r) return;
  const e = ONLINE_ROBOT_LIBRARY.find((x) => x.id === r.id);
  if (!e) return;
  toast(`Downloading ${e.name}…`, 'info', 2500);
  try {
    const res = await app.addOnlineRobot(e.id, app.station, { meshes: !!r.meshes, onProgress: (m) => app.log(m) });
    toast(`${e.name}: ${res.robot.dof} DOF, ${res.meshes.loaded.length} meshes`, 'ok');
  } catch (err) {
    toast(`Download failed: ${(err as Error).message}`, 'error', 6000);
    app.log(`Online library ${e.id}: ${(err as Error).message}`, 'error');
  }
}

/** VDA 5050 fleet connection (KUKA Fleet, MiR, Open-RMF, any VDA 5050 master/AGVs) through the studio server. */
export async function vdaDialog(app: App): Promise<void> {
  const base = app.serverHttpBase();
  if (!base) return toast('Start the studio server (npm run server) and open the studio with ?server=ws://host:20500 — the server holds the MQTT connection', 'warn', 7000);
  const current = (app as any).vda as import('../fleet/vda_client').VdaClient | null;
  const status = h('pre', { class: 'code-preview', style: { maxHeight: '200px' } }, current?.status ? JSON.stringify({ connected: current.status.connected, agvs: current.status.agvs?.length, twins: current.status.twins?.length }, null, 1) : 'not connected');
  const r = await dialog<{ url: string; prefix: string; role: string; manufacturer: string; mapId: string; user: string; pass: string; shadow: boolean; auto: boolean }>('VDA 5050 fleet interface (AGV / AMR)', [
    { key: 'url', label: 'MQTT broker URL', type: 'text', value: current?.opts.url ?? 'mqtt://localhost:1883', hint: 'mqtt://, mqtts://, ws:// or wss:// — the broker of KUKA Fleet / your fleet manager' },
    { key: 'prefix', label: 'Topic prefix (interfaceName)', type: 'text', value: current?.opts.prefix ?? 'uagv' },
    { key: 'role', label: 'Role', type: 'select', value: current?.opts.role ?? 'master', options: [{ value: 'master', label: 'Master: dispatch fleet tasks to real AGVs and mirror them here' }, { value: 'bridge', label: 'AGV bridge: expose the station\'s mobile robots as VDA 5050 AGVs (digital twins)' }, { value: 'both', label: 'Both' }] },
    { key: 'manufacturer', label: 'Manufacturer (bridge twins topic segment)', type: 'text', value: current?.opts.manufacturer ?? 'VerticalBot' },
    { key: 'mapId', label: 'Map id', type: 'text', value: current?.opts.mapId ?? 'station' },
    { key: 'user', label: 'Username (optional)', type: 'text', value: '' },
    { key: 'pass', label: 'Password (optional)', type: 'password' as any, value: '' },
    { key: 'shadow', label: 'Mirror AGV states onto robots with the same name (serial)', type: 'checkbox', value: current?.opts.shadow ?? true },
    { key: 'auto', label: 'Send fleet tasks as VDA orders automatically', type: 'checkbox', value: current?.opts.autoDispatch ?? true },
  ], { width: 620, okLabel: current ? 'Reconnect' : 'Connect', body: h('div', null, h('p', { class: 'hint' }, 'VDA 5050 v2: orders with nodes/edges/actions, instantActions (cancelOrder, startPause, initPosition…), state, connection (LWT), visualization, factsheet. Metres/radians on the wire, mm/deg in the station. Robots are paired by serial number = robot name.'), status) });
  if (!r) return;
  const { VdaClient } = await import('../fleet/vda_client');
  if (current) await current.disconnect();
  const client = new VdaClient(app, base, { url: r.url, prefix: r.prefix, role: r.role as any, manufacturer: r.manufacturer, mapId: r.mapId, username: r.user || undefined, password: r.pass || undefined, shadow: !!r.shadow, autoDispatch: !!r.auto });
  (app as any).vda = client;
  try {
    const st = await client.connect();
    app.log(`VDA 5050: ${st.connected ? 'connected' : 'connecting'} to ${r.url} as ${r.role}; ${st.agvs?.length ?? 0} AGVs seen, ${st.twins?.length ?? 0} twins`);
    toast(`VDA 5050 ${st.connected ? 'connected' : 'connecting…'}`, 'ok');
  } catch (e) {
    toast(`VDA 5050: ${(e as Error).message}`, 'error', 6000);
  }
}

export async function mobileRobotDialog(app: App): Promise<void> {
  const r = await dialog<{ preset: any; name: string }>('Add mobile robot', [
    { key: 'preset', label: 'Type', type: 'select', value: 'amr', options: [{ value: 'amr', label: 'AMR (differential)' }, { value: 'tractor', label: 'Autonomous tractor (Ackermann)' }, { value: 'harvester', label: 'Harvest platform (tracked)' }, { value: 'sprayer', label: 'Orchard sprayer' }, { value: 'scout', label: 'Scout rover' }] },
    { key: 'name', label: 'Name', type: 'text', value: '' },
  ]);
  if (!r) return;
  const m = app.addMobileRobot(r.preset);
  if (r.name) app.cmd(() => m.setName(r.name));
}

export async function orchardDialog(app: App, field?: FieldItem): Promise<void> {
  const crops = Object.keys(CROP_PRESETS) as CropType[];
  const cur = field?.crop ?? cropParams('apple');
  const r = await dialog<any>(field ? `Regenerate ${field.name}` : 'Create field / orchard', [
    { key: 'name', label: 'Name', type: 'text', value: field?.name ?? 'Orchard block A' },
    { key: 'crop', label: 'Crop', type: 'select', value: cur.crop, options: crops.map((c) => ({ value: c, label: c })) },
    { key: 'width', label: 'Field width (m)', type: 'number', value: field ? Math.round(Math.max(...field.polygon.map((p) => p[0])) / 1000) : 80, min: 5 },
    { key: 'length', label: 'Field length (m)', type: 'number', value: field ? Math.round(Math.max(...field.polygon.map((p) => p[1])) / 1000) : 40, min: 5 },
    { key: 'rowSpacing', label: 'Row spacing (m)', type: 'number', value: cur.rowSpacing / 1000, step: 0.1 },
    { key: 'plantSpacing', label: 'Plant spacing (m)', type: 'number', value: cur.plantSpacing / 1000, step: 0.1 },
    { key: 'rowHeading', label: 'Row heading (deg)', type: 'number', value: cur.rowHeading },
    { key: 'headland', label: 'Headland (m)', type: 'number', value: cur.headland / 1000, step: 0.5 },
    { key: 'ripe', label: 'Ripe fraction', type: 'range', value: cur.ripeFraction, min: 0, max: 1, step: 0.05 },
    { key: 'indoor', label: 'Greenhouse (indoor)', type: 'checkbox', value: field?.indoor ?? false },
    { key: 'map', label: 'Build navigation map + headland zones', type: 'checkbox', value: !field },
  ], { width: 520 });
  if (!r) return;
  app.cmd(() => {
    const f = field ?? app.station.addChild(new FieldItem(r.name));
    f.setName(r.name);
    f.polygon = rectPolygon(r.width * 1000, r.length * 1000);
    f.indoor = !!r.indoor;
    f.crop = cropParams(r.crop, { rowSpacing: r.rowSpacing * 1000, plantSpacing: r.plantSpacing * 1000, rowHeading: r.rowHeading, headland: r.headland * 1000, ripeFraction: r.ripe });
    const rows = generateOrchard(f, Math.floor(Math.random() * 1000));
    if (r.map) { app.station.addChild(buildFieldMap(f)); makeHeadlandZones(f); makeCanopyZones(f); }
    toast(`${rows.length} rows, ${rows.reduce((s, x) => s + x.plants.length, 0)} plants`, 'ok');
    app.select(f);
  });
  setTimeout(() => app.renderer.fitAll(), 50);
}

export async function fleetDialog(app: App): Promise<void> {
  const r = await dialog<any>('Create fleet', [
    { key: 'name', label: 'Fleet name', type: 'text', value: 'Orchard fleet' },
    { key: 'n', label: 'Number of robots', type: 'number', value: 3, min: 0, max: 50 },
    { key: 'preset', label: 'Robot type', type: 'select', value: 'harvester', options: [{ value: 'harvester', label: 'Harvest platform' }, { value: 'sprayer', label: 'Sprayer' }, { value: 'tractor', label: 'Tractor' }, { value: 'amr', label: 'AMR' }, { value: 'scout', label: 'Scout rover' }] },
    { key: 'allocation', label: 'Allocation', type: 'select', value: 'auction', options: [{ value: 'auction', label: 'Cost-based auction' }, { value: 'nearest', label: 'Nearest' }, { value: 'round_robin', label: 'Round robin' }] },
    { key: 'charger', label: 'Add charging zone at start', type: 'checkbox', value: true },
  ]);
  if (!r) return;
  app.cmd(() => {
    const fleet = app.station.addChild(new FleetItem(r.name));
    fleet.allocation = r.allocation;
    const fm = app.fleetManager(fleet);
    const field = app.station.itemsOfType<FieldItem>(ItemType.FIELD)[0];
    const fp = field?.poseAbs();
    const x0 = fp ? fp[12] - 4000 : -4000, y0 = fp ? fp[13] + 3000 : 0;
    for (let i = 0; i < r.n; i++) {
      const m = app.addMobileRobot(r.preset);
      m.setName(`${m.name} ${i + 1}`);
      m.setPose2D(x0, y0 + i * 3000, 0);
      m.home = { x: x0, y: y0 + i * 3000, theta: 0 };
      fm.addRobot(m);
    }
    if (r.charger) {
      const z = app.station.addChild(new ZoneItem('Charging station'));
      z.kind = 'charging';
      z.polygon = rectPolygon(3000, Math.max(3000, r.n * 3000), x0 - 1500, y0 - 1500);
      fleet.chargingZoneIds.push(z.id);
    }
    app.select(fleet);
  });
}

export async function missionDialog(app: App): Promise<void> {
  const fields = app.station.itemsOfType<FieldItem>(ItemType.FIELD);
  const fleets = app.station.itemsOfType<FleetItem>(ItemType.FLEET);
  if (!fields.length) return toast('Create a field first (Add > Field / orchard)', 'warn');
  const types: MissionType[] = ['harvest', 'spray', 'mow', 'prune', 'scout', 'weed', 'pollinate', 'transport', 'thin', 'irrigate'];
  const r = await dialog<any>('Create mission', [
    { key: 'name', label: 'Name', type: 'text', value: 'Harvest mission' },
    { key: 'type', label: 'Type', type: 'select', value: 'harvest', options: types.map((t) => ({ value: t, label: t })) },
    { key: 'field', label: 'Field', type: 'select', value: fields[0].id, options: fields.map((f) => ({ value: f.id, label: f.name })) },
    { key: 'fleet', label: 'Fleet', type: 'select', value: fleets[0]?.id ?? '', options: fleets.length ? fleets.map((f) => ({ value: f.id, label: f.name })) : [{ value: '', label: '(create a fleet first)' }] },
    { key: 'rows', label: 'Rows (e.g. 1-5,8; empty = all)', type: 'text', value: '' },
    { key: 'speed', label: 'Work speed (m/s)', type: 'number', value: 0.5, step: 0.1 },
    { key: 'bothSides', label: 'Work both sides of each row', type: 'checkbox', value: true },
    { key: 'plan', label: 'Plan now (create fleet tasks)', type: 'checkbox', value: true },
  ]);
  if (!r) return;
  const field = app.station.findById(r.field) as FieldItem;
  const rowsAll = field.rows();
  const sel = parseRanges(r.rows, rowsAll.length).map((i) => rowsAll[i]).filter(Boolean);
  const mission = app.cmd(() => {
    const m = app.station.addChild(new MissionItem(r.name));
    m.missionType = r.type;
    m.fieldId = field.id;
    m.fleetId = r.fleet || null;
    m.rowIds = sel.map((x) => x.id);
    m.settings = { workSpeed: r.speed * 1000, bothSides: r.bothSides, sideOffset: field.crop.rowSpacing / 2, secondsPerFruit: 6 };
    return m;
  });
  if (r.plan && r.fleet) runMissionPlan(app, mission);
  app.select(mission);
}

export function runMissionPlan(app: App, mission: MissionItem): void {
  const fleet = (mission.fleetId ? app.station.findById(mission.fleetId) : app.station.itemsOfType<FleetItem>(ItemType.FLEET)[0]) as FleetItem | null;
  if (!fleet) return toast('Mission has no fleet', 'warn');
  try {
    const plan = planMission(app.station, mission, app.fleetManager(fleet));
    toast(`${plan.tasks.length} tasks over ${plan.rows} rows, ${(plan.totalPathLength / 1000).toFixed(0)} m${plan.fruitTargets ? `, ${plan.fruitTargets} ripe fruit` : ''}, est. ${plan.estimatedHours.toFixed(1)} h`, 'ok', 7000);
    app.snapshot();
  } catch (e: any) {
    toast(e.message, 'error');
  }
}

function parseRanges(s: string, n: number): number[] {
  if (!s.trim()) return Array.from({ length: n }, (_, i) => i);
  const out = new Set<number>();
  for (const part of s.split(',')) {
    const m = part.trim().match(/^(\d+)(?:-(\d+))?$/);
    if (!m) continue;
    const a = +m[1] - 1, b = m[2] ? +m[2] - 1 : a;
    for (let i = Math.min(a, b); i <= Math.max(a, b); i++) if (i >= 0 && i < n) out.add(i);
  }
  return [...out].sort((a, b) => a - b);
}

export async function mapDialog(app: App): Promise<void> {
  const r = await dialog<any>('Create occupancy map', [
    { key: 'w', label: 'Width (m)', type: 'number', value: 50 },
    { key: 'h', label: 'Height (m)', type: 'number', value: 50 },
    { key: 'res', label: 'Resolution (m/cell)', type: 'number', value: 0.25, step: 0.05 },
    { key: 'fromObjects', label: 'Rasterise objects footprints', type: 'checkbox', value: true },
  ]);
  if (!r) return;
  app.cmd(() => {
    const m = app.station.addChild(new MapItem('Map'));
    m.resize(Math.ceil(r.w / r.res), Math.ceil(r.h / r.res), r.res * 1000, -r.w * 500, -r.h * 500);
    if (r.fromObjects) for (const o of app.station.itemsOfType<SceneObject>(ItemType.OBJECT)) {
      const p = o.poseAbs();
      const b = o.bbox ?? { min: [-250, -250, 0], max: [250, 250, 0] };
      m.fillRect(p[12] + b.min[0], p[13] + b.min[1], p[12] + b.max[0], p[13] + b.max[1]);
    }
    app.select(m);
  });
}

export async function zoneDialog(app: App): Promise<void> {
  const r = await dialog<any>('Create zone', [
    { key: 'name', label: 'Name', type: 'text', value: 'Zone' },
    { key: 'kind', label: 'Kind', type: 'select', value: 'charging', options: ['work', 'nogo', 'charging', 'loading', 'unloading', 'parking', 'speed_limit', 'headland', 'gnss_denied'].map((k) => ({ value: k, label: k === 'gnss_denied' ? 'gnss_denied (canopy / indoor: no satellite fix)' : k })) },
    { key: 'w', label: 'Width (m)', type: 'number', value: 4 },
    { key: 'h', label: 'Length (m)', type: 'number', value: 4 },
  ]);
  if (!r) return;
  app.cmd(() => { const z = app.station.addChild(new ZoneItem(r.name)); z.kind = r.kind; z.polygon = rectPolygon(r.w * 1000, r.h * 1000, -r.w * 500, -r.h * 500); app.select(z); });
}

export async function componentDialog(app: App): Promise<void> {
  const comps = app.station.itemsOfType<Component>(ItemType.COMPONENT);
  const r = await dialog<any>('Add process component', [
    { key: 'type', label: 'Type', type: 'select', value: 'conveyor', options: [{ value: 'feeder', label: 'Feeder (creates products)' }, { value: 'conveyor', label: 'Conveyor' }, { value: 'process', label: 'Process / machine' }, { value: 'buffer', label: 'Buffer / pallet' }, { value: 'sink', label: 'Sink (consumes)' }] },
    { key: 'name', label: 'Name', type: 'text', value: '' },
    { key: 'p1', label: 'Length (mm) / cycle time (s) / interval (s) / capacity', type: 'number', value: 3000 },
    { key: 'p2', label: 'Speed (mm/s) [conveyor]', type: 'number', value: 400 },
    { key: 'next', label: 'Connect output to', type: 'select', value: '', options: [{ value: '', label: '(none)' }, ...comps.map((c) => ({ value: c.id, label: c.name }))] },
  ]);
  if (!r) return;
  app.cmd(() => {
    let c: Component;
    const name = r.name || r.type;
    switch (r.type) {
      case 'feeder': c = makeFeeder(name, r.p1 || 5, { name: 'Crate', geometry: { primitive: { kind: 'box', size: [400, 300, 250] }, origin: Array.from(transl(0, 0, 125)), color: '#d9a066' }, massKg: 12 }); break;
      case 'conveyor': c = makeConveyor(name, r.p1 || 3000, r.p2 || 400); break;
      case 'process': c = makeProcess(name, r.p1 || 5); break;
      case 'buffer': c = makeBuffer(name, r.p1 || 12, { cols: 3, rows: 2, layers: 4, pitch: [420, 320, 260] }); break;
      default: c = makeSink(name);
    }
    (c.behaviour as any).next = r.next || null;
    const n = comps.length;
    c.setPose(transl(0, -2500 - n * 1200, r.type === 'conveyor' ? 800 : 0));
    app.station.addChild(c);
    app.select(c);
  });
}

export async function navigateDialog(app: App, m: MobileRobot): Promise<void> {
  const r = await dialog<any>(`Navigate ${m.name}`, [
    { key: 'x', label: 'X (m)', type: 'number', value: (m.state.x / 1000 + 10).toFixed(1) },
    { key: 'y', label: 'Y (m)', type: 'number', value: (m.state.y / 1000).toFixed(1) },
  ]);
  if (!r) return;
  const { planPath } = await import('../mobile/planner');
  const { followPath } = await import('../mobile/controller');
  const map = app.station.itemsOfType<MapItem>(ItemType.MAP)[0];
  const goal: [number, number] = [r.x * 1000, r.y * 1000];
  const path = map ? planPath(map, [m.state.x, m.state.y], goal) : { ok: true, path: [[m.state.x, m.state.y], goal], length: 0, expanded: 0 };
  if (!path.ok) return toast(path.error ?? 'No path', 'error');
  followPath(m, path.path);
  app.startWorld();
}

export async function harvestArmDialog(app: App, robot: Robot): Promise<void> {
  const rows = app.station.itemsOfType<CropRow>(ItemType.CROP_ROW);
  if (!rows.length) return toast('No crop rows in the station', 'warn');
  const r = await dialog<any>('Generate fruit picking program', [
    { key: 'limit', label: 'Max fruit', type: 'number', value: 30 },
    { key: 'approach', label: 'Approach distance (mm)', type: 'number', value: 150 },
    { key: 'mode', label: 'Approach', type: 'select', value: 'horizontal', options: [{ value: 'horizontal', label: 'Horizontal (from inter-row)' }, { value: 'radial', label: 'Radial (toward canopy centre)' }] },
  ]);
  if (!r) return;
  const base = robot.poseAbs();
  const near = rows.filter((row) => { const p = row.parent!.poseAbs(); const a = row.pointAt(0), b = row.pointAt(row.length()); const ax = p[12] + a[0], ay = p[13] + a[1], bx = p[12] + b[0], by = p[13] + b[1]; return distToSeg(base[12], base[13], ax, ay, bx, by) < robot.reach * 2; });
  if (!near.length) return toast('No rows within reach of this robot — move it next to a row', 'warn');
  const res = app.cmd(() => generateHarvestProgram(app.station, robot, near, { limit: r.limit, approach: r.approach, approachMode: r.mode }));
  app.setActiveProgram(res.program);
  app.select(res.program);
  toast(`Program with ${res.picked} picks (${res.skipped} unreachable)`, 'ok');
}

function distToSeg(px: number, py: number, ax: number, ay: number, bx: number, by: number): number {
  const dx = bx - ax, dy = by - ay;
  const l2 = dx * dx + dy * dy || 1;
  const t = Math.max(0, Math.min(1, ((px - ax) * dx + (py - ay) * dy) / l2));
  return Math.hypot(px - (ax + t * dx), py - (ay + t * dy));
}

/** Robot machining from an NC program object (milling, cutting, dispensing, 3D printing). */
export async function machiningDialog(app: App, robot: Robot): Promise<void> {
  const parts = app.station.itemsOfType<SceneObject>(ItemType.OBJECT).filter((o) => o.curves.length);
  if (!parts.length) return toast('Import an NC / G-code file first (drop .nc/.gcode/.tap onto the 3D view) or an object with curves', 'warn', 5000);
  const ncParts = parts.filter((o) => o.curves.some((c) => (c as any).kind));
  const r = await dialog<{ part: string; approach: number; rapid: number; cut: number; spindle: string; extruder: string; z: string; free: boolean; step: number; rounding: number }>('Robot machining project (NC / G-code)', [
    { key: 'part', label: 'Part / NC program object', type: 'select', options: [...ncParts, ...parts.filter((p) => !ncParts.includes(p))].map((o) => ({ value: o.id, label: `${o.name} (${o.curves.length} ${ncParts.includes(o) ? 'NC segments' : 'curves'})` })) },
    { key: 'approach', label: 'Approach / retract distance (mm)', type: 'number', value: 50 },
    { key: 'rapid', label: 'Rapid speed (mm/s)', type: 'number', value: 250 },
    { key: 'cut', label: 'Cutting speed override (mm/s, 0 = use NC feed)', type: 'number', value: 0 },
    { key: 'spindle', label: 'Spindle / laser digital output (blank = none)', type: 'text', value: 'Spindle' },
    { key: 'extruder', label: 'Extruder digital output (3D printing, blank = none)', type: 'text', value: '' },
    { key: 'z', label: 'Tool Z direction', type: 'select', value: 'down', options: [{ value: 'down', label: '-Z of the part (milling, printing)' }, { value: 'normal', label: 'Along curve normals (surface following)' }] },
    { key: 'free', label: 'Free rotation about tool axis (symmetric tool)', type: 'checkbox', value: true },
    { key: 'step', label: 'Resample cuts every (mm, 0 = keep NC points)', type: 'number', value: 0 },
    { key: 'rounding', label: 'Rounding radius (mm)', type: 'number', value: 1 },
  ], { width: 560, okLabel: 'Generate program', body: h('p', { class: 'hint' }, 'Same as RoboDK\'s robot machining / 3D printing project: rapids at rapid speed, cuts at the NC feed rate, spindle/extruder outputs switched per segment, approach and retract added. The program stays attached to the part frame.') });
  if (!r) return;
  const part = app.station.findById(r.part) as SceneObject | null;
  if (!part) return;
  const { generateMachining } = await import('../core/motion/machining');
  const res = app.cmd(() => generateMachining(app.station, robot, part, { approach: r.approach, rapidSpeed: r.rapid, cutSpeed: r.cut > 0 ? r.cut : undefined, spindleIO: r.spindle || undefined, extruderIO: r.extruder || undefined, zMode: r.z as 'down' | 'normal', freeToolZ: !!r.free, step: r.step > 0 ? r.step : undefined, rounding: r.rounding }));
  app.setActiveProgram(res.program);
  app.previewProgram();
  const msg = `${res.program.name}: ${res.points} points, ${res.segments} segments, cut ${(res.cutLength / 1000).toFixed(2)} m, rapid ${(res.rapidLength / 1000).toFixed(2)} m, est. ${res.estimatedTime.toFixed(0)} s${res.unreachable ? `, ${res.unreachable} unreachable` : ''}`;
  app.log(msg, res.unreachable ? 'warn' : 'info');
  toast(msg, res.unreachable ? 'warn' : 'ok', 6000);
}

export async function curveFollowDialog(app: App, robot: Robot): Promise<void> {
  const objs = app.station.itemsOfType<SceneObject>(ItemType.OBJECT).filter((o) => o.curves.length || o.points.length);
  if (!objs.length) return toast('No object with curves/points. Import an object with curves or use the API (RDK.AddCurve / AddPoints).', 'warn');
  const r = await dialog<any>(`Follow curve / points with ${robot.name}`, [
    { key: 'obj', label: 'Object', type: 'select', value: objs[0].id, options: objs.map((o) => ({ value: o.id, label: `${o.name} (${o.curves.length} curves, ${o.points.length} points)` })) },
    { key: 'what', label: 'Follow', type: 'select', value: 'curve', options: [{ value: 'curve', label: 'Curves (continuous path)' }, { value: 'points', label: 'Points (approach each)' }] },
    { key: 'step', label: 'Point spacing along curve (mm, 0 = vertices)', type: 'number', value: 10 },
    { key: 'approach', label: 'Approach / retract (mm)', type: 'number', value: 50 },
    { key: 'z', label: 'Tool Z', type: 'select', value: 'normal', options: [{ value: 'normal', label: 'Along surface normal (if available)' }, { value: 'down', label: 'Vertical (-Z)' }] },
    { key: 'free', label: 'Free rotation about tool Z (symmetric tool)', type: 'checkbox', value: true },
    { key: 'speed', label: 'Speed (mm/s)', type: 'number', value: 50 },
    { key: 'io', label: 'Digital output while following (empty = none)', type: 'text', value: 'DO_1' },
  ], { width: 520 });
  if (!r) return;
  const { generateCurveFollow, generatePointFollow } = await import('../core/motion/pathfollow');
  const obj = app.station.findById(r.obj) as SceneObject;
  const res = app.cmd(() => {
    if (r.what === 'points' && obj.points.length) return generatePointFollow(app.station, robot, obj, obj.points.map((p) => ({ point: p.point })), { approach: r.approach, freeToolZ: r.free, speed: r.speed, io: r.io || undefined });
    const merged = { points: obj.curves.flatMap((c) => c.points), normals: undefined as number[][] | undefined };
    return generateCurveFollow(app.station, robot, obj, merged, { step: r.step, approach: r.approach, zMode: r.z, freeToolZ: r.free, speed: r.speed, io: r.io || undefined });
  });
  app.setActiveProgram(res.program);
  app.select(res.program);
  toast(`${res.points} points programmed, ${res.unreachable} unreachable`, res.unreachable ? 'warn' : 'ok');
}

export async function railIKDialog(app: App, robot: Robot): Promise<void> {
  const { carrierOf, solveIKWithCarrier, applyCombined } = await import('../core/kinematics/combined');
  const carrier = carrierOf(robot);
  if (!carrier) return toast('This robot is not mounted on another mechanism (drag it onto a rail/gantry robot in the tree)', 'warn');
  const sel = app.station.selection.find((i) => i instanceof Target) as Target | undefined;
  const r = await dialog<any>(`Move ${robot.name} + ${carrier.name} (external axes)`, [
    { key: 'x', label: 'Target X (mm, carrier base frame)', type: 'number', value: sel ? getPos(multiply(invert(carrier.poseAbs()), sel.poseAbs()))[0].toFixed(1) : 1000 },
    { key: 'y', label: 'Target Y', type: 'number', value: sel ? getPos(multiply(invert(carrier.poseAbs()), sel.poseAbs()))[1].toFixed(1) : 0 },
    { key: 'z', label: 'Target Z', type: 'number', value: sel ? getPos(multiply(invert(carrier.poseAbs()), sel.poseAbs()))[2].toFixed(1) : 500 },
  ], { body: h('p', { class: 'hint' }, 'Solves IK over the carrier axes and the arm together (tool Z down unless a target is selected).') });
  if (!r) return;
  const target = sel ? multiply(invert(carrier.poseAbs()), sel.poseAbs()) : mul(transl(r.x, r.y, r.z), rotx180());
  const sol = solveIKWithCarrier(carrier, robot, target);
  if (!sol.ok) return toast(`Not reachable (residual ${sol.result.posError.toFixed(1)} mm)`, 'error');
  app.cmd(() => applyCombined(carrier, robot, sol));
  toast(`Carrier: [${sol.carrierJoints.map((v) => v.toFixed(0)).join(', ')}]`, 'ok');
}
function rotx180() { return xyzrpwToPose(0, 0, 0, 180, 0, 0); }

export async function exportDialog(app: App, program: Program | null = app.activeProgram): Promise<void> {
  if (!program) return toast('Select a program', 'warn');
  const robot = program.robot();
  const posts = app.posts();
  const def = robot instanceof Robot ? robot.postProcessor : 'JSON';
  const preview = h('pre', { class: 'code-preview' });
  const pyBtn = h('button', { class: 'btn', onClick: async () => {
    const files = await pickFiles('.py');
    if (!files.length) return;
    const src = await files[0].text();
    const id = `PY_${files[0].name.replace(/\W+/g, '_')}`;
    const { registerPythonPost } = await import('../posts/python_post');
    const post = registerPythonPost(id, files[0].name, src);
    preview.textContent = 'Running the Python post in Pyodide (first run downloads ~10 MB)…';
    try {
      const { compileForPost } = await import('../posts/base');
      const out = await post.generateAsync(compileForPost(app.station, program));
      preview.textContent = out.map((f) => `# ---- ${f.name} ----\n${f.content}`).join('\n');
      for (const f of out) downloadText(f.name, f.content);
      toast(`Python post ${files[0].name} generated ${out.length} file(s)`, 'ok');
    } catch (e: any) { preview.textContent = `Python post failed: ${e.message ?? e}`; }
  } }, 'Run a RoboDK Python post (.py)…');
  const body = h('div', null, h('div', { class: 'btn-row' }, pyBtn, h('span', { class: 'hint' }, 'Load any RoboDK post processor file (class RobotPost) and run it in the browser.')), preview);
  const update = (v: Record<string, any>) => { const files = app.exportProgram(v.post, program); preview.textContent = files.map((f) => `# ---- ${f.name} ----\n${f.content}`).join('\n').slice(0, 20000); };
  update({ post: posts.some((p) => p.id === def) ? def : 'Generic' });
  const r = await dialog<{ post: string }>(`Export ${program.name}`, [
    { key: 'post', label: 'Post processor', type: 'select', value: posts.some((p) => p.id === def) ? def : 'Generic', options: posts.map((p) => ({ value: p.id, label: `${p.name} (.${p.extension})` })) },
  ], { width: 760, okLabel: 'Download', body, onChange: update });
  if (!r) return;
  for (const f of app.exportProgram(r.post, program)) downloadText(f.name, f.content, f.mime);
}

export async function importDialog(app: App): Promise<void> {
  const files = await pickFiles('.vbstation,.json,.geojson,.urdf,.xacro,.stl,.obj,.step,.stp,.iges,.igs,.brep,.dh,.rdk,.robot,.tool,.src,.mod,.prg,.ls,.script,.csv,.txt,.nc,.gcode,.ngc,.tap,.pcd,.ply,.glb,.gltf,.dae,.zip,.vcmx,.vcm', true);
  if (files.length) await app.openFiles(files);
}

export async function robotParametersDialog(app: App, robot: Robot): Promise<void> {
  const posts = app.posts();
  const r = await dialog<any>(`${robot.name} parameters`, [
    { key: 'name', label: 'Name', type: 'text', value: robot.name },
    { key: 'post', label: 'Post processor', type: 'select', value: robot.postProcessor, options: posts.map((p) => ({ value: p.id, label: p.name })) },
    { key: 'speedL', label: 'Linear speed (mm/s)', type: 'number', value: robot.motion.speedLinear },
    { key: 'speedJ', label: 'Joint speed (deg/s)', type: 'number', value: robot.motion.speedJoints },
    { key: 'accL', label: 'Linear accel (mm/s²)', type: 'number', value: robot.motion.accelLinear },
    { key: 'accJ', label: 'Joint accel (deg/s²)', type: 'number', value: robot.motion.accelJoints },
    { key: 'ip', label: 'Controller IP / ROS namespace', type: 'text', value: robot.connection.ip ?? robot.connection.rosNamespace ?? '' },
  ]);
  if (!r) return;
  app.cmd(() => { robot.setName(r.name); robot.postProcessor = r.post; robot.motion = { ...robot.motion, speedLinear: r.speedL, speedJoints: r.speedJ, accelLinear: r.accL, accelJoints: r.accJ }; robot.connection = { ...robot.connection, ip: r.ip, rosNamespace: r.ip }; });
}

import * as itemsModule from '../core/items/item';
function require_items() { return itemsModule; }
export { poseToXyzrpw, mul, rotz, DEG, Station, Instruction, Target, Frame };
