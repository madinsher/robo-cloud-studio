/** UI for the navigation & SLAM stack: properties section, selection wizard, ROS 2 export and the Navigation tab. */
import type { App } from '../app';
import { h, clear, dialog, toast, downloadBlob, formField, fmt } from './dom';
import { MobileRobot, MapItem, ZoneItem } from '../mobile/items';
import { ItemType } from '../core/items/item';
import { LOCALIZATION_METHODS, NAVIGATION_METHODS, SENSOR_LABELS, PLATFORM_DEFAULT_SENSORS, recommendStacks, getNavStack, setNavStack, platformForRobot, NavStackConfig, PlatformType, Environment, SensorKind, NavRuntime, StackRecommendation } from '../mobile/navstack';
import { generateRosNavPackage, rosNavPackageZip } from '../mobile/navstack_ros';
import { t } from './i18n';

const PLATFORMS: Array<[PlatformType, string]> = [['amr', 'AMR — free navigation (differential / omni)'], ['agv_natural', 'AGV — natural navigation (laser / reflectors)'], ['agv_line', 'AGV — line following (tape / QR grid)'], ['tractor', 'Autonomous tractor / Ackermann platform'], ['tracked', 'Tracked / skid-steer platform'], ['legged', 'Legged robot'], ['rail', 'Pipe-rail trolley (greenhouse)']];
const ENVIRONMENTS: Array<[Environment, string]> = [['orchard', 'Orchard (canopy, GNSS-denied rows)'], ['vineyard', 'Vineyard'], ['open_field', 'Open field'], ['greenhouse', 'Greenhouse'], ['warehouse', 'Warehouse'], ['factory', 'Factory'], ['forest', 'Forest'], ['urban', 'Urban / yard'], ['underground', 'Underground / tunnel'], ['mixed', 'Mixed indoor–outdoor']];

function section(title: string, ...children: (HTMLElement | null)[]): HTMLElement {
  return h('div', { class: 'section' }, h('div', { class: 'prop-title' }, t(title)), ...children.filter(Boolean) as HTMLElement[]);
}
const kv = (k: string, v: string) => h('div', { class: 'kv-row' }, h('span', { class: 'k' }, t(k)), h('span', { class: 'v' }, v));

/** Properties-panel section for a mobile robot. */
export function navStackSection(app: App, robot: MobileRobot, liveFields: Array<() => void>): HTMLElement {
  const cfg = getNavStack(robot);
  const stats = h('div', { class: 'kv' });
  const upd = () => {
    clear(stats);
    const c = getNavStack(robot);
    if (!c) { stats.append(kv('Stack', 'not configured')); return; }
    const loc = LOCALIZATION_METHODS.find((m) => m.id === c.localization), nav = NAVIGATION_METHODS.find((n) => n.id === c.navigation);
    stats.append(kv('Platform', `${c.platform} · ${c.environment}`), kv('Localization', loc?.name ?? c.localization), kv('Navigation', nav?.name ?? c.navigation), kv('Sensors', c.sensors.map((s) => SENSOR_LABELS[s]).join(', ')));
    const rt = (robot as any)._nav as NavRuntime | undefined;
    if (c.simulate && rt) {
      const e = rt.estimator.state;
      stats.append(kv('Estimate error', `${fmt(e.error / 1000, 2)} m (RMSE ${fmt(e.rmse / 1000, 2)}, max ${fmt(e.maxError / 1000, 2)})`), kv('GNSS fix', e.gnssAvailable ? 'yes' : 'no (denied)'), kv('Localization', e.lost ? `LOST (${e.lostEvents} events)` : `ok · ${e.fixes} loop closures`), kv('Since last fix', `${fmt(e.distanceSinceFix / 1000, 1)} m`));
      if (rt.slam) stats.append(kv('SLAM map', `${(rt.slam.coverage() * 100).toFixed(0)} % explored`));
    }
  };
  upd();
  liveFields.push(upd);
  const simToggle = formField({ key: 'sim', label: 'Simulate localization (estimate drives the controller)', type: 'checkbox', value: cfg?.simulate ?? false }, (v) => { const c = getNavStack(robot); if (c) app.cmd(() => setNavStack(robot, { ...c, simulate: !!v })); });
  return section('Navigation & localization', stats, cfg ? simToggle.el : null,
    h('div', { class: 'btn-row' },
      h('button', { class: 'btn primary', onClick: () => navStackDialog(app, robot) }, t(cfg ? 'Change stack…' : 'Select stack…')),
      cfg ? h('button', { class: 'btn', onClick: () => exportNavPackage(app, robot) }, t('Export ROS 2 package')) : null,
      cfg ? h('button', { class: 'btn', onClick: () => { (app as any).bottom?.show?.('nav'); } }, t('Navigation tab')) : null));
}

/** Wizard: platform + environment + sensors + constraints → ranked stacks; applies the chosen one. */
export async function navStackDialog(app: App, robot: MobileRobot): Promise<void> {
  const cur = getNavStack(robot);
  const platform0 = cur?.platform ?? platformForRobot(robot);
  const env0 = cur?.environment ?? (app.station.itemsOfType(ItemType.FIELD).length ? 'orchard' : 'warehouse');
  const sensors0 = new Set<SensorKind>(cur?.sensors ?? PLATFORM_DEFAULT_SENSORS[platform0]);
  const list = h('div', { class: 'nav-recs' });
  let recs: StackRecommendation[] = [];
  let chosen = 0;
  const state = { platform: platform0 as PlatformType, environment: env0 as Environment, gnss: cur?.gnssAvailability ?? -1, night: false, global: true, maxCost: 5, compute: 'high' as 'low' | 'mid' | 'high' };
  const render = () => {
    recs = recommendStacks({ platform: state.platform, environment: state.environment, sensors: [...sensors0], gnssAvailability: state.gnss >= 0 ? state.gnss : undefined, night: state.night, needsGlobal: state.global, maxCost: state.maxCost, compute: state.compute });
    chosen = Math.min(chosen, recs.length - 1);
    clear(list);
    recs.slice(0, 8).forEach((r, i) => {
      const row = h('div', { class: `nav-rec ${i === chosen ? 'active' : ''}`, onClick: () => { chosen = i; render(); } },
        h('div', { class: 'nav-rec-head' }, h('b', null, `${i + 1}. ${r.localization.name}`), h('span', { class: 'badge' }, `${r.score}`)),
        h('div', { class: 'hint' }, `${r.navigation.name}`),
        h('div', { class: 'hint' }, `✓ ${r.reasons.join(' · ')}`),
        r.warnings.length ? h('div', { class: 'hint warn' }, `⚠ ${r.warnings.join(' · ')}`) : null,
        h('div', { class: 'hint' }, `accuracy ${r.localization.accuracy} m · drift ${(r.localization.driftPerMeter * 100).toFixed(1)} %/m · cost ${r.localization.cost}/5 · ${r.localization.compute} compute · ${r.localization.software.join(', ')}`));
      list.appendChild(row);
    });
  };
  const sensorKeys = Object.keys(SENSOR_LABELS) as SensorKind[];
  const sensorBoxes = sensorKeys.map((s) => formField({ key: s, label: SENSOR_LABELS[s], type: 'checkbox', value: sensors0.has(s) }, (v) => { if (v) sensors0.add(s); else sensors0.delete(s); render(); }).el);
  const sensorsGrid = h('div', { class: 'grid small' }, ...sensorBoxes);
  const body = h('div', null,
    h('div', { class: 'grid' },
      formField({ key: 'p', label: 'Platform', type: 'select', value: state.platform, options: PLATFORMS.map(([v, l]) => ({ value: v, label: l })) }, (v) => { state.platform = v; sensors0.clear(); for (const s of PLATFORM_DEFAULT_SENSORS[v as PlatformType]) sensors0.add(s); sensorKeys.forEach((k, i) => { (sensorBoxes[i].querySelector('input') as HTMLInputElement).checked = sensors0.has(k); }); render(); }).el,
      formField({ key: 'e', label: 'Environment', type: 'select', value: state.environment, options: ENVIRONMENTS.map(([v, l]) => ({ value: v, label: l })) }, (v) => { state.environment = v; render(); }).el,
      formField({ key: 'g', label: 'GNSS sky view (0–1, −1 = environment default)', type: 'number', value: state.gnss, step: 0.05, min: -1, max: 1 }, (v) => { state.gnss = v; render(); }).el,
      formField({ key: 'c', label: 'Max hardware cost (1–5)', type: 'number', value: state.maxCost, min: 1, max: 5 }, (v) => { state.maxCost = v; render(); }).el,
      formField({ key: 'cp', label: 'Compute budget', type: 'select', value: state.compute, options: ['low', 'mid', 'high'].map((x) => ({ value: x, label: x })) }, (v) => { state.compute = v; render(); }).el,
      formField({ key: 'n', label: 'Night operation', type: 'checkbox', value: state.night }, (v) => { state.night = !!v; render(); }).el,
      formField({ key: 'gl', label: 'Global repeatable frame needed (row entries, docking, fleet)', type: 'checkbox', value: state.global }, (v) => { state.global = !!v; render(); }).el),
    h('div', { class: 'prop-title' }, t('Sensors on the vehicle')), sensorsGrid,
    h('div', { class: 'prop-title' }, t('Recommended stacks (click to select)')), list);
  render();
  const r = await dialog<{ sim: boolean }>('Navigation & SLAM stack', [{ key: 'sim', label: 'Simulate localization errors in the world simulation', type: 'checkbox', value: cur?.simulate ?? true }], { width: 760, okLabel: 'Apply stack', body });
  if (!r || !recs.length) return;
  const rec = recs[chosen];
  const cfg: NavStackConfig = { platform: state.platform, environment: state.environment, sensors: [...sensors0], localization: rec.localization.id, navigation: rec.navigation.id, fusion: rec.fusion, simulate: !!r.sim, gnssAvailability: state.gnss >= 0 ? state.gnss : undefined };
  app.cmd(() => setNavStack(robot, cfg));
  (robot as any)._nav = undefined;
  app.log(`${robot.name}: ${rec.localization.name} + ${rec.navigation.name} (score ${rec.score})${rec.missingSensors.length ? ` — add ${rec.missingSensors.map((s) => SENSOR_LABELS[s]).join(', ')}` : ''}`, rec.missingSensors.length ? 'warn' : 'info');
  toast(`${rec.localization.name} applied to ${robot.name}`, 'ok');
  app.select(robot);
}

export function exportNavPackage(app: App, robot: MobileRobot): void {
  const cfg = getNavStack(robot);
  if (!cfg) return toast('Select a navigation stack first', 'warn');
  const pkg = generateRosNavPackage(robot, cfg);
  downloadBlob(`${pkg.name}.zip`, new Blob([rosNavPackageZip(pkg).buffer as ArrayBuffer], { type: 'application/zip' }));
  app.log(`ROS 2 navigation package ${pkg.name}: ${Object.keys(pkg.files).length} files; packages: ${pkg.packages.join(', ')}`);
  for (const n of pkg.notes) app.log(n);
  toast(`ROS 2 package ${pkg.name}.zip exported`, 'ok');
}

/** Bottom "Navigation" tab: 2D view of the map, GNSS-denied zones, SLAM map, true vs estimated trajectory, lidar. */
export function buildNavPanel(app: App): { el: HTMLElement; render: () => void } {
  const sel = h('select') as HTMLSelectElement;
  const info = h('div', { class: 'hint' });
  const canvas = h('canvas', { width: 900, height: 300, class: 'nav-canvas' }) as HTMLCanvasElement;
  const el = h('div', { class: 'pad nav-panel' }, h('div', { class: 'btn-row' }, h('span', null, t('Robot: ')), sel, h('button', { class: 'btn small', onClick: () => { const r = robotOf(); if (r) navStackDialog(app, r); } }, t('Stack…')), h('button', { class: 'btn small', onClick: () => { const r = robotOf(); if (r) exportNavPackage(app, r); } }, t('Export ROS 2')), h('button', { class: 'btn small', onClick: () => { const r = robotOf(); if (r) { (r as any)._nav = undefined; } } }, t('Reset estimate'))), info, canvas);
  const robotOf = () => app.station.findById(sel.value) as MobileRobot | null;
  const refreshList = () => {
    const robots = app.station.itemsOfType<MobileRobot>(ItemType.MOBILE_ROBOT);
    const cur = sel.value;
    clear(sel);
    for (const r of robots) sel.appendChild(h('option', { value: r.id }, `${r.name}${getNavStack(r) ? ` — ${LOCALIZATION_METHODS.find((m) => m.id === getNavStack(r)!.localization)?.name ?? ''}` : ''}`));
    if (robots.some((r) => r.id === cur)) sel.value = cur;
  };
  const render = () => {
    refreshList();
    const ctx = canvas.getContext('2d')!;
    ctx.fillStyle = '#1b1f26'; ctx.fillRect(0, 0, canvas.width, canvas.height);
    const robot = robotOf();
    if (!robot) { info.textContent = t('Add a mobile robot and select a navigation stack (Mobile & Fleet › Navigation & SLAM stack…)'); return; }
    const cfg = getNavStack(robot);
    const rt = (robot as any)._nav as NavRuntime | undefined;
    const map = app.station.itemsOfType<MapItem>(ItemType.MAP)[0] ?? null;
    const zones = app.station.itemsOfType<ZoneItem>(ItemType.ZONE);
    // world window around the map or the robot
    let x0: number, y0: number, w: number, hgt: number;
    if (map) { x0 = map.originX; y0 = map.originY; w = map.width * map.resolution; hgt = map.height * map.resolution; }
    else { x0 = robot.state.x - 15000; y0 = robot.state.y - 8000; w = 30000; hgt = 16000; }
    const scale = Math.min(canvas.width / w, canvas.height / hgt);
    const X = (x: number) => (x - x0) * scale, Y = (y: number) => canvas.height - (y - y0) * scale;
    if (map) {
      const img = ctx.createImageData(map.width, map.height);
      for (let cy = 0; cy < map.height; cy++) for (let cx = 0; cx < map.width; cx++) { const v = map.get(cx, cy); const i = ((map.height - 1 - cy) * map.width + cx) * 4; const g = v >= 50 ? 70 : v === 255 ? 35 : 48; img.data[i] = g; img.data[i + 1] = g + (v >= 50 ? 10 : 0); img.data[i + 2] = g + 8; img.data[i + 3] = 255; }
      const off = document.createElement('canvas'); off.width = map.width; off.height = map.height; off.getContext('2d')!.putImageData(img, 0, 0);
      ctx.imageSmoothingEnabled = false;
      ctx.drawImage(off, 0, 0, map.width, map.height, X(map.originX), Y(map.originY + map.height * map.resolution), map.width * map.resolution * scale, map.height * map.resolution * scale);
    }
    for (const z of zones) {
      const abs = z.poseAbs();
      ctx.beginPath();
      z.polygon.forEach((p, i) => { const px = abs[12] + p[0], py = abs[13] + p[1]; i ? ctx.lineTo(X(px), Y(py)) : ctx.moveTo(X(px), Y(py)); });
      ctx.closePath();
      ctx.fillStyle = z.kind === 'gnss_denied' ? 'rgba(156,54,181,0.25)' : z.kind === 'nogo' ? 'rgba(224,49,49,0.25)' : z.kind === 'charging' ? 'rgba(28,126,214,0.25)' : 'rgba(245,159,0,0.12)';
      ctx.fill();
    }
    if (rt?.slam) {
      const s = rt.slam;
      ctx.fillStyle = 'rgba(255,255,255,0.10)';
      for (let cy = 0; cy < s.height; cy++) for (let cx = 0; cx < s.width; cx++) { const v = s.cells[cy * s.width + cx]; if (v === 255) continue; ctx.fillStyle = v === 100 ? 'rgba(255,212,59,0.9)' : 'rgba(255,255,255,0.08)'; ctx.fillRect(X(s.originX + cx * s.resolution), Y(s.originY + (cy + 1) * s.resolution), Math.max(1, s.resolution * scale), Math.max(1, s.resolution * scale)); }
    }
    if (rt?.lastScan) {
      ctx.strokeStyle = 'rgba(64,192,255,0.25)'; ctx.beginPath();
      const e = rt.estimator.state; const dTh = (e.theta - robot.state.theta) * Math.PI / 180;
      rt.lastScan.angles.forEach((a, i) => { const r = rt.lastScan!.ranges[i]; ctx.moveTo(X(e.x), Y(e.y)); ctx.lineTo(X(e.x + Math.cos(a + dTh) * r), Y(e.y + Math.sin(a + dTh) * r)); });
      ctx.stroke();
    }
    const trail = (pts: number[][], color: string) => { if (pts.length < 2) return; ctx.strokeStyle = color; ctx.lineWidth = 2; ctx.beginPath(); pts.forEach((p, i) => (i ? ctx.lineTo(X(p[0]), Y(p[1])) : ctx.moveTo(X(p[0]), Y(p[1])))); ctx.stroke(); ctx.lineWidth = 1; };
    if (rt) { trail(rt.trueTrail, '#51cf66'); trail(rt.estTrail, '#ff922b'); }
    const arrow = (x: number, y: number, th: number, color: string) => { const a = th * Math.PI / 180; ctx.fillStyle = color; ctx.beginPath(); ctx.moveTo(X(x + Math.cos(a) * 900), Y(y + Math.sin(a) * 900)); ctx.lineTo(X(x + Math.cos(a + 2.5) * 500), Y(y + Math.sin(a + 2.5) * 500)); ctx.lineTo(X(x + Math.cos(a - 2.5) * 500), Y(y + Math.sin(a - 2.5) * 500)); ctx.closePath(); ctx.fill(); };
    arrow(robot.state.x, robot.state.y, robot.state.theta, '#51cf66');
    if (rt) arrow(rt.estimator.state.x, rt.estimator.state.y, rt.estimator.state.theta, '#ff922b');
    if (robot.state.path) { ctx.strokeStyle = 'rgba(255,255,255,0.35)'; ctx.setLineDash([4, 4]); ctx.beginPath(); robot.state.path.forEach((p, i) => (i ? ctx.lineTo(X(p[0]), Y(p[1])) : ctx.moveTo(X(p[0]), Y(p[1])))); ctx.stroke(); ctx.setLineDash([]); }
    const loc = cfg ? LOCALIZATION_METHODS.find((m) => m.id === cfg.localization) : null;
    if (!cfg) info.textContent = t('No navigation stack on this robot — press Stack… to choose one.');
    else if (!cfg.simulate) info.textContent = `${loc?.name}: ${t('localization simulation is off (enable it in the robot properties)')}`;
    else if (!rt) info.textContent = `${loc?.name}: ${t('waiting for the world simulation (Mobile & Fleet › Start world simulation)')}`;
    else { const e = rt.estimator.state; info.textContent = `${loc?.name} · error ${fmt(e.error / 1000, 2)} m · RMSE ${fmt(e.rmse / 1000, 2)} m · max ${fmt(e.maxError / 1000, 2)} m · GNSS ${e.gnssAvailable ? 'fix' : 'denied'} · ${e.lost ? 'LOST' : 'tracking'} · loop closures ${e.fixes} · lost events ${e.lostEvents}${rt.slam ? ` · SLAM explored ${(rt.slam.coverage() * 100).toFixed(0)} %` : ''} — green: truth, orange: estimate`; }
  };
  return { el, render };
}
