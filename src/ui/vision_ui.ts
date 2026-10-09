/** UI for the machine-vision stack: camera properties section, selection wizard, Vision tab, ROS 2 export, point-cloud import. */
import type { App } from '../app';
import { h, clear, dialog, toast, downloadBlob, formField, fmt } from './dom';
import { Camera as CameraItem, ItemType, Item, SceneObject } from '../core/items/item';
import { Robot } from '../core/items/robot';
import { MobileRobot } from '../mobile/items';
import { integrate } from '../mobile/controller';
import { TASK_LABELS, MODALITY_LABELS, COMPUTE_LABELS, VISION_SENSORS, COMPUTE_TARGETS, VISION_MODELS, recommendVisionStacks, configFromRecommendation, getVisionStack, setVisionStack, modalityForCamera, cameraKindForModality, errorBudget, VisionStackConfig, VisionTask, VisionModality, ComputeClass, VisionRecommendation, VisionRequest } from '../vision/stack';
import { ensureVisionRuntime, visionRuntimeOf, createTargetsFromOutput, followCommand, summarize, PipelineOutput, analyseCloud } from '../vision/pipeline';
import { generateRosVisionPackage, rosVisionPackageZip } from '../vision/vision_ros';
import { parsePCD, parsePLY, PointCloud, decimate, pointCount, cloudStats, writePCD } from '../vision/pointcloud';
import { intrinsicsFromCamera } from '../vision/camera_model';
import { visionSummary, publishVisionOutput } from '../ros/publishers';
import { multiply, transl, rotx, DEG } from '../core/math/pose';
import { t } from './i18n';

const ENVIRONMENTS: Array<[string, string]> = [['orchard', 'Orchard'], ['vineyard', 'Vineyard'], ['open_field', 'Open field'], ['greenhouse', 'Greenhouse'], ['warehouse', 'Warehouse'], ['factory', 'Factory / cell'], ['forest', 'Forest'], ['urban', 'Urban / yard'], ['underground', 'Underground'], ['mixed', 'Mixed']];
const TASK_ORDER: VisionTask[] = ['detect', 'track', 'classify', 'segment', 'keypoints', 'pose', 'grasp', 'depth', 'cloud_detect', 'cloud_segment', 'measure', 'follow', 'vlm_query', 'vla_policy'];

function section(title: string, ...children: (HTMLElement | null)[]): HTMLElement {
  return h('div', { class: 'section' }, h('div', { class: 'prop-title' }, t(title)), ...children.filter(Boolean) as HTMLElement[]);
}
const kv = (k: string, v: string) => h('div', { class: 'kv-row' }, h('span', { class: 'k' }, t(k)), h('span', { class: 'v' }, v));

/** Cameras in the station (any kind). */
export function camerasOf(app: App): CameraItem[] { return app.station.itemsOfType<CameraItem>(ItemType.CAMERA); }

/** Add a camera under the selected item (tool / robot flange / mobile robot / frame) or the station. */
export function addCamera(app: App, parent?: Item, modality: VisionModality = 'mono'): CameraItem {
  const p = parent ?? app.station.selection[0] ?? app.station;
  const cam = app.cmd(() => {
    const c = new CameraItem(`Camera ${camerasOf(app).length + 1}`);
    c.kind = cameraKindForModality(modality);
    if (modality === 'lidar3d') { c.fov = 360; c.far = 60000; c.width = 1024; c.height = 64; }
    p.addChild(c);
    // default pose: on a mobile robot look forward and slightly down from 1.2 m; elsewhere look along the parent's +Z
    // optical frame looking forward (+X of the vehicle): cam X = -Y, cam Y = -Z, cam Z = +X; tilted 15° down
    if (p instanceof MobileRobot) c.setPose(multiply(multiply(transl(p.kin.footprint[0] / 2, 0, 1200), Float64Array.from([0, -1, 0, 0, 0, 0, -1, 0, 1, 0, 0, 0, 0, 0, 0, 1])), rotx(-15 * DEG)));
    else if (p === app.station) c.setPose(multiply(transl(0, -1500, 1500), rotx(-90 * DEG)));
    return c;
  });
  app.select(cam);
  return cam;
}

/** Properties section for a camera item. */
export function visionSection(app: App, cam: CameraItem, liveFields: Array<() => void>): HTMLElement {
  const stats = h('div', { class: 'kv' });
  const upd = () => {
    clear(stats);
    const c = getVisionStack(cam);
    if (!c) { stats.append(kv('Stack', 'not configured')); return; }
    const s = VISION_SENSORS.find((x) => x.id === c.sensor), cp = COMPUTE_TARGETS.find((x) => x.id === c.compute);
    stats.append(kv('Sensor', `${s?.name ?? c.sensor} (${MODALITY_LABELS[c.modality]})`), kv('Compute', cp?.name ?? c.compute), kv('Tasks', c.tasks.map((x) => TASK_LABELS[x].split(' (')[0]).join(', ')), kv('Models', Object.entries(c.models).map(([, id]) => VISION_MODELS.find((m) => m.id === id)?.name.split(' (')[0] ?? id).join(', ')), kv('Runtime', c.runtime + (c.runtime === 'onnx' && c.modelUrl ? ` · ${c.modelUrl.split('/').pop()}` : '') + (c.vlm ? ` · VLM ${c.vlm.model}` : '') + (c.vla ? ` · VLA ${c.vla.model}` : '')));
    const eb = errorBudget(c);
    stats.append(kv('Error budget', `${eb.depthSigmaMm === null ? 'no depth' : `depth σ ${eb.depthSigmaMm.toFixed(1)} mm`} · ${eb.pixelMm.toFixed(2)} mm/px · ${eb.latencyMs.toFixed(0)} ms`));
    const rt = visionRuntimeOf(cam);
    if (rt?.last) { const st = rt.stats; const p = st.tp + st.fp ? st.tp / (st.tp + st.fp) : 0, r = st.tp + st.fn ? st.tp / (st.tp + st.fn) : 0; stats.append(kv('Last run', `${rt.last.detections.length} detections · P ${(p * 100).toFixed(0)} % R ${(r * 100).toFixed(0)} % · pos. error ${st.posErrN ? (st.posErrSum / st.posErrN).toFixed(0) : '–'} mm · ${st.lastLatencyMs.toFixed(0)} ms`)); }
  };
  upd();
  liveFields.push(upd);
  const cfg = getVisionStack(cam);
  return section('Machine vision', stats,
    h('div', { class: 'btn-row' },
      h('button', { class: 'btn primary', onClick: () => visionStackDialog(app, cam) }, t(cfg ? 'Change vision stack…' : 'Select vision stack…')),
      cfg ? h('button', { class: 'btn', onClick: () => { (app as any).bottom?.show?.('vision'); } }, t('Vision tab')) : null,
      cfg ? h('button', { class: 'btn', onClick: () => exportVisionPackage(app, cam) }, t('Export ROS 2 package')) : null,
      cfg ? h('button', { class: 'btn', onClick: () => { app.cmd(() => setVisionStack(cam, null)); app.select(cam); } }, t('Remove')) : null));
}

/** Wizard: tasks + environment + modality + compute + constraints → ranked sensor/compute/model stacks. */
export async function visionStackDialog(app: App, cam: CameraItem): Promise<void> {
  const cur = getVisionStack(cam);
  const isAgri = app.station.itemsOfType(ItemType.FIELD).length > 0;
  const state = {
    tasks: new Set<VisionTask>(cur?.tasks ?? ['detect', 'track', 'pose']),
    environment: cur?.environment ?? (isAgri ? 'orchard' : 'factory'),
    modality: (cur?.modality ?? (cam.kind === 'lidar3d' ? modalityForCamera(cam) : 'any')) as VisionModality | 'any',
    compute: 'any' as ComputeClass | 'any',
    distance: cur?.workingDistance ?? 1.5,
    needs3D: true, night: false, moving: !!(cam.parent instanceof MobileRobot), openVocab: false, online: false, maxCost: 5, minFps: 10,
    classes: cur?.classes.join(', ') ?? (isAgri ? String((app.station.itemsOfType(ItemType.FIELD)[0] as any).crop?.crop ?? 'apple') + ', trunk, person' : 'box, part, person'),
  };
  const list = h('div', { class: 'nav-recs' });
  let recs: VisionRecommendation[] = [];
  let chosen = 0;
  const request = (): VisionRequest => ({ tasks: [...state.tasks], environment: state.environment as any, modalities: state.modality === 'any' ? undefined : [state.modality], compute: state.compute === 'any' ? undefined : state.compute, workingDistance: state.distance, needs3D: state.needs3D, night: state.night, moving: state.moving, openVocabulary: state.openVocab, online: state.online, maxCost: state.maxCost, minFps: state.minFps });
  const render = () => {
    recs = state.tasks.size ? recommendVisionStacks(request()) : [];
    chosen = Math.max(0, Math.min(chosen, recs.length - 1));
    clear(list);
    if (!recs.length) { list.appendChild(h('div', { class: 'hint warn' }, t('No stack satisfies these constraints — relax the modality, compute or cost.'))); return; }
    recs.slice(0, 8).forEach((r, i) => {
      const models = Object.entries(r.models).map(([task, m]) => `${TASK_LABELS[task as VisionTask].split(' (')[0]}: ${m!.name}`);
      list.appendChild(h('div', { class: `nav-rec ${i === chosen ? 'active' : ''}`, onClick: () => { chosen = i; render(); } },
        h('div', { class: 'nav-rec-head' }, h('b', null, `${i + 1}. ${r.sensor.name}`), h('span', { class: 'badge' }, `${r.score}`)),
        h('div', { class: 'hint' }, `${r.compute.name} · ${MODALITY_LABELS[r.sensor.modality]} · ${r.depthErrorM === null ? 'no depth' : `depth σ ${(r.depthErrorM * 1000).toFixed(1)} mm @ ${state.distance} m`} · pipeline ≈ ${r.latencyMs.toFixed(0)} ms`),
        h('div', { class: 'hint' }, models.join(' · ')),
        r.reasons.length ? h('div', { class: 'hint' }, `✓ ${r.reasons.join(' · ')}`) : null,
        r.warnings.length ? h('div', { class: 'hint warn' }, `⚠ ${r.warnings.join(' · ')}`) : null,
        h('details', null, h('summary', { class: 'hint' }, t('Pipeline')), h('ol', { class: 'hint' }, ...r.pipeline.map((p) => h('li', null, p))))));
    });
  };
  const taskBoxes = TASK_ORDER.map((task) => formField({ key: task, label: TASK_LABELS[task], type: 'checkbox', value: state.tasks.has(task) }, (v) => { if (v) state.tasks.add(task); else state.tasks.delete(task); render(); }).el);
  const runtimeBox = h('div', { class: 'grid' });
  const rtState = { runtime: cur?.runtime ?? 'simulated', modelUrl: cur?.modelUrl ?? '', inputSize: cur?.inputSize ?? 640, serverModel: cur?.serverModel ?? 'yolov8n.pt', rosTopic: cur?.rosTopic ?? '/yolo/detections', vlmUrl: cur?.vlm?.url ?? 'http://localhost:11434/v1', vlmModel: cur?.vlm?.model ?? 'qwen2.5vl:7b', vlmKey: cur?.vlm?.apiKey ?? '', vlaUrl: cur?.vla?.url ?? 'http://localhost:8000', vlaModel: cur?.vla?.model ?? 'pi0', vlaFormat: cur?.vla?.format ?? 'openpi', instruction: cur?.vla?.instruction ?? 'pick the ripe apple', simulate: cur?.simulate ?? true, objectSize: cur?.objectSizeMm ?? 75, publishUrl: cur?.publishUrl ?? '', publishRos: cur?.publishRos ?? false, handEye: cur?.handEyeMode ?? (cam.parent instanceof Robot || cam.parent?.type === ItemType.TOOL ? 'eye_in_hand' : 'eye_to_hand') };
  runtimeBox.append(
    formField({ key: 'rt', label: 'Runtime (where models execute)', type: 'select', value: rtState.runtime, options: [{ value: 'simulated', label: 'Simulated (ground truth + model statistics)' }, { value: 'onnx', label: 'ONNX Runtime Web in this browser (YOLO .onnx URL)' }, { value: 'server', label: 'Studio server (/vision/infer: ultralytics / onnxruntime)' }, { value: 'vlm', label: 'VLM endpoint for detection/classification too' }, { value: 'ros2', label: 'ROS 2 topic via rosbridge (vision_msgs)' }] }, (v) => { rtState.runtime = v; }).el,
    formField({ key: 'mu', label: 'ONNX model URL / path (Ultralytics export)', type: 'text', value: rtState.modelUrl, hint: 'e.g. /models/yolov8n.onnx or https://…/best.onnx' }, (v) => { rtState.modelUrl = v; }).el,
    formField({ key: 'is', label: 'Input size (px)', type: 'number', value: rtState.inputSize, min: 224, max: 1280, step: 32 }, (v) => { rtState.inputSize = v; }).el,
    formField({ key: 'sm', label: 'Server model (name in STUDIO_VISION_MODELS or path)', type: 'text', value: rtState.serverModel }, (v) => { rtState.serverModel = v; }).el,
    formField({ key: 'rtp', label: 'ROS 2 detections topic', type: 'text', value: rtState.rosTopic }, (v) => { rtState.rosTopic = v; }).el,
    formField({ key: 'vu', label: 'VLM endpoint (OpenAI-compatible base URL)', type: 'text', value: rtState.vlmUrl, hint: 'Ollama: http://localhost:11434/v1 · vLLM: http://host:8000/v1' }, (v) => { rtState.vlmUrl = v; }).el,
    formField({ key: 'vm', label: 'VLM model', type: 'text', value: rtState.vlmModel }, (v) => { rtState.vlmModel = v; }).el,
    formField({ key: 'vk', label: 'VLM API key (optional)', type: 'text', value: rtState.vlmKey }, (v) => { rtState.vlmKey = v; }).el,
    formField({ key: 'au', label: 'VLA policy server URL', type: 'text', value: rtState.vlaUrl }, (v) => { rtState.vlaUrl = v; }).el,
    formField({ key: 'am', label: 'VLA model / checkpoint', type: 'text', value: rtState.vlaModel }, (v) => { rtState.vlaModel = v; }).el,
    formField({ key: 'af', label: 'VLA server format', type: 'select', value: rtState.vlaFormat, options: [{ value: 'openpi', label: 'openpi (π0) /infer' }, { value: 'openvla', label: 'OpenVLA /act' }, { value: 'studio', label: 'studio JSON /act (LeRobot, ACT, custom)' }] }, (v) => { rtState.vlaFormat = v; }).el,
    formField({ key: 'ai', label: 'Default instruction', type: 'text', value: rtState.instruction }, (v) => { rtState.instruction = v; }).el,
    formField({ key: 'os', label: 'Object size prior for mono (mm)', type: 'number', value: rtState.objectSize, min: 1 }, (v) => { rtState.objectSize = v; }).el,
    formField({ key: 'pu', label: 'Publish results to URL (webhook, POST JSON after every run)', type: 'text', value: rtState.publishUrl, hint: 'e.g. http://localhost:8765/vision — see python/examples/vision_webhook_sink.py' }, (v) => { rtState.publishUrl = v; }).el,
    formField({ key: 'pr', label: 'Publish to ROS 2 through rosbridge (Connect › ROS 2): vision_msgs, PoseArray, PointCloud2, CompressedImage', type: 'checkbox', value: rtState.publishRos }, (v) => { rtState.publishRos = !!v; }).el,
    formField({ key: 'he', label: 'Camera mounting', type: 'select', value: rtState.handEye, options: [{ value: 'eye_in_hand', label: 'eye-in-hand (on the flange / tool)' }, { value: 'eye_to_hand', label: 'eye-to-hand (fixed, on the vehicle or cell)' }] }, (v) => { rtState.handEye = v; }).el);
  const body = h('div', null,
    h('div', { class: 'prop-title' }, t('Tasks')), h('div', { class: 'vision-tasks' }, ...taskBoxes),
    h('div', { class: 'grid' },
      formField({ key: 'e', label: 'Environment', type: 'select', value: state.environment, options: ENVIRONMENTS.map(([v, l]) => ({ value: v, label: l })) }, (v) => { state.environment = v; render(); }).el,
      formField({ key: 'm', label: 'Sensor modality', type: 'select', value: state.modality, options: [{ value: 'any', label: 'Any — recommend' }, ...(Object.keys(MODALITY_LABELS) as VisionModality[]).map((m) => ({ value: m, label: MODALITY_LABELS[m] }))] }, (v) => { state.modality = v; render(); }).el,
      formField({ key: 'c', label: 'Compute', type: 'select', value: state.compute, options: [{ value: 'any', label: 'Any — recommend' }, ...(Object.keys(COMPUTE_LABELS) as ComputeClass[]).map((c) => ({ value: c, label: COMPUTE_LABELS[c] }))] }, (v) => { state.compute = v; render(); }).el,
      formField({ key: 'd', label: 'Working distance (m)', type: 'number', value: state.distance, step: 0.1, min: 0.1 }, (v) => { state.distance = v; render(); }).el,
      formField({ key: 'f', label: 'Required rate (Hz)', type: 'number', value: state.minFps, min: 0.1, step: 1 }, (v) => { state.minFps = v; render(); }).el,
      formField({ key: 'k', label: 'Max hardware cost (1–5)', type: 'number', value: state.maxCost, min: 1, max: 5 }, (v) => { state.maxCost = v; render(); }).el,
      formField({ key: 'cl', label: 'Classes (comma separated; prompt for open-vocabulary models)', type: 'text', value: state.classes }, (v) => { state.classes = v; }).el,
      formField({ key: '3d', label: '3D positions needed (picking, distances)', type: 'checkbox', value: state.needs3D }, (v) => { state.needs3D = !!v; render(); }).el,
      formField({ key: 'n', label: 'Night / dark operation', type: 'checkbox', value: state.night }, (v) => { state.night = !!v; render(); }).el,
      formField({ key: 'mv', label: 'Sensor on a moving vehicle', type: 'checkbox', value: state.moving }, (v) => { state.moving = !!v; render(); }).el,
      formField({ key: 'ov', label: 'Open vocabulary (text-prompted classes, no training)', type: 'checkbox', value: state.openVocab }, (v) => { state.openVocab = !!v; render(); }).el,
      formField({ key: 'on', label: 'Connectivity available (cloud / hosted models allowed)', type: 'checkbox', value: state.online }, (v) => { state.online = !!v; render(); }).el),
    h('div', { class: 'prop-title' }, t('Recommended stacks (click to select)')), list,
    h('details', { open: !!cur && cur.runtime !== 'simulated' }, h('summary', { class: 'prop-title' }, t('Runtime & endpoints (plug in real models)')), runtimeBox));
  render();
  const r = await dialog<{ sim: boolean }>(`Machine vision stack — ${cam.name}`, [{ key: 'sim', label: 'Simulate detections in the world simulation (ground truth + model statistics)', type: 'checkbox', value: rtState.simulate }], { width: 820, okLabel: 'Apply stack', body });
  if (!r || !recs.length) return;
  const rec = recs[chosen];
  const classes = state.classes.split(',').map((s) => s.trim()).filter(Boolean);
  const cfg: VisionStackConfig = { ...configFromRecommendation(rec, request(), classes), runtime: rtState.runtime as VisionStackConfig['runtime'], modelUrl: rtState.modelUrl || undefined, inputSize: rtState.inputSize, serverModel: rtState.serverModel || undefined, rosTopic: rtState.rosTopic || undefined, simulate: !!r.sim, objectSizeMm: rtState.objectSize, handEyeMode: rtState.handEye as any, publishUrl: rtState.publishUrl || undefined, publishRos: rtState.publishRos };
  if (state.tasks.has('vlm_query') || rtState.runtime === 'vlm') cfg.vlm = { url: rtState.vlmUrl, model: rtState.vlmModel, apiKey: rtState.vlmKey || undefined, prompt: '' };
  if (state.tasks.has('vla_policy')) cfg.vla = { url: rtState.vlaUrl, model: rtState.vlaModel, format: rtState.vlaFormat as any, instruction: rtState.instruction, actionScaleMm: 20, actionScaleDeg: 5 };
  app.cmd(() => {
    setVisionStack(cam, cfg);
    cam.kind = cameraKindForModality(rec.sensor.modality);
    if (rec.sensor.modality === 'lidar3d') { cam.fov = rec.sensor.hfov; cam.far = Math.max(cam.far, rec.sensor.depthRange![1] * 1000); }
    else { cam.fov = rec.sensor.hfov; cam.width = rec.sensor.resolution[0] > 1280 ? 1280 : rec.sensor.resolution[0]; cam.height = Math.round(cam.width * rec.sensor.resolution[1] / rec.sensor.resolution[0]); if (rec.sensor.depthRange) cam.far = Math.max(cam.far, rec.sensor.depthRange[1] * 1000); }
    cam.notify('kind');
  });
  app.log(`${cam.name}: ${rec.sensor.name} + ${rec.compute.name} — ${Object.entries(rec.models).map(([task, m]) => `${task}: ${m!.name}`).join(', ')} (score ${rec.score})${rec.warnings.length ? ` ⚠ ${rec.warnings.join('; ')}` : ''}`, rec.warnings.length ? 'warn' : 'info');
  toast(`${rec.sensor.name} applied to ${cam.name}`, 'ok');
  app.select(cam);
  (app as any).bottom?.show?.('vision');
}

export function exportVisionPackage(app: App, cam: CameraItem): void {
  const cfg = getVisionStack(cam);
  if (!cfg) return toast('Select a vision stack first', 'warn');
  const pkg = generateRosVisionPackage(cam, cfg);
  downloadBlob(`${pkg.name}.zip`, new Blob([rosVisionPackageZip(pkg).buffer as ArrayBuffer], { type: 'application/zip' }));
  app.log(`ROS 2 perception package ${pkg.name}: ${Object.keys(pkg.files).length} files; packages: ${pkg.packages.join(', ')}`);
  for (const n of pkg.notes) app.log(n);
  toast(`ROS 2 package ${pkg.name}.zip exported`, 'ok');
}

/** Drop / open a .pcd / .ply: keep the full cloud for analysis and show a decimated copy as object points. */
export function importPointCloud(app: App, name: string, buf: ArrayBuffer): SceneObject {
  const cloud = /\.ply$/i.test(name) ? parsePLY(buf) : parsePCD(buf);
  const clouds: Map<string, PointCloud> = ((app as any).pointClouds ??= new Map());
  const obj = app.cmd(() => {
    const o = new SceneObject(name.replace(/\.(pcd|ply)$/i, ''));
    const dec = decimate(cloud, 8000);
    o.points = Array.from({ length: pointCount(dec) }, (_, i) => ({ name: '', point: [dec.xyz[i * 3], dec.xyz[i * 3 + 1], dec.xyz[i * 3 + 2]] }));
    o.setParam('pointCloud', { points: pointCount(cloud), source: name } as any);
    o.color = '#4dabf7';
    app.station.addChild(o);
    return o;
  });
  clouds.set(obj.id, { ...cloud, frame: 'world' });
  const st = cloudStats(cloud);
  app.log(`${name}: ${st.n} points, extent ${((st.max[0] - st.min[0]) / 1000).toFixed(1)} × ${((st.max[1] - st.min[1]) / 1000).toFixed(1)} × ${((st.max[2] - st.min[2]) / 1000).toFixed(1)} m — Vision tab › Analyse cloud`);
  app.select(obj);
  return obj;
}

/** Bottom "Vision" tab: camera image with detections / masks / keypoints / tracks, point-cloud BEV, stats, VLM/VLA. */
export function buildVisionPanel(app: App): { el: HTMLElement; render: () => void; stop: () => void } {
  const sel = h('select') as HTMLSelectElement;
  const robotSel = h('select') as HTMLSelectElement;
  const info = h('div', { class: 'hint' });
  const statsEl = h('div', { class: 'hint' });
  const canvas = h('canvas', { width: 640, height: 400, class: 'cam-canvas vision-canvas' }) as HTMLCanvasElement;
  const bev = h('canvas', { width: 300, height: 300, class: 'cam-canvas vision-bev' }) as HTMLCanvasElement;
  const off = document.createElement('canvas');
  const live = h('input', { type: 'checkbox' }) as HTMLInputElement;
  const follow = h('input', { type: 'checkbox' }) as HTMLInputElement;
  const prompt = h('input', { type: 'text', class: 'vision-prompt', placeholder: 'VLM question / VLA instruction…' }) as HTMLInputElement;
  const answer = h('div', { class: 'hint vision-answer' });
  const showTruth = h('input', { type: 'checkbox' }) as HTMLInputElement;
  let busy = false;
  let lastOut: PipelineOutput | null = null;
  let lastWebhook = 0;
  const camOf = () => app.station.findById(sel.value) as CameraItem | null;
  const robotOf = () => app.station.findById(robotSel.value) as MobileRobot | null;
  const refreshLists = () => {
    const cams = camerasOf(app), cur = sel.value; clear(sel);
    for (const c of cams) sel.appendChild(h('option', { value: c.id }, `${c.name}${getVisionStack(c) ? ` — ${VISION_SENSORS.find((s) => s.id === getVisionStack(c)!.sensor)?.name.split(' (')[0] ?? ''}` : ''}`));
    if (cams.some((c) => c.id === cur)) sel.value = cur;
    const robots = app.station.itemsOfType<MobileRobot>(ItemType.MOBILE_ROBOT), rc = robotSel.value; clear(robotSel);
    for (const r of robots) robotSel.appendChild(h('option', { value: r.id }, r.name));
    if (robots.some((r) => r.id === rc)) robotSel.value = rc;
  };
  const capturePixels = (cam: CameraItem): { rgba: Uint8ClampedArray; dataUrl: string } | null => {
    if (cam.kind === 'lidar3d') return null;
    const w = Math.min(1280, cam.width), hgt = Math.round(w * cam.height / cam.width);
    off.width = w; off.height = hgt;
    try { app.renderer.renderFromItem(cam, off, cam.fov, cam.near, cam.far, false); } catch { return null; }
    const ctx = off.getContext('2d')!;
    return { rgba: ctx.getImageData(0, 0, w, hgt).data, dataUrl: off.toDataURL('image/jpeg', 0.85) };
  };
  const draw = (cam: CameraItem, out: PipelineOutput | null) => {
    const ctx = canvas.getContext('2d')!;
    const cfg = getVisionStack(cam);
    ctx.fillStyle = '#111'; ctx.fillRect(0, 0, canvas.width, canvas.height);
    if (cam.kind !== 'lidar3d') {
      if (off.width) ctx.drawImage(off, 0, 0, canvas.width, canvas.height);
    } else { ctx.fillStyle = '#ccc'; ctx.font = '13px sans-serif'; ctx.fillText('3D LiDAR — see the point-cloud view', 12, 24); }
    if (!out) return;
    const sx = canvas.width / cam.width, sy = canvas.height / cam.height;
    ctx.lineWidth = 1.5; ctx.font = '11px sans-serif';
    if (showTruth.checked && out.frame.truth) { ctx.strokeStyle = 'rgba(255,255,255,0.35)'; for (const tr of out.frame.truth) if (tr.visible) ctx.strokeRect(tr.box.x * sx, tr.box.y * sy, tr.box.w * sx, tr.box.h * sy); }
    const colorFor = (cls: string, id?: number) => { let hsh = 0; for (const ch of `${cls}${id ?? ''}`) hsh = (hsh * 31 + ch.charCodeAt(0)) >>> 0; return `hsl(${hsh % 360} 90% 60%)`; };
    for (const b of out.detections) {
      const col = colorFor(b.cls, b.id);
      if (b.mask) {
        const img = ctx.createImageData(Math.max(1, Math.round(b.mask.width * sx)), Math.max(1, Math.round(b.mask.height * sy)));
        const [r, g, bl] = hslToRgb(col);
        for (let y = 0; y < img.height; y++) for (let x = 0; x < img.width; x++) { const mx = Math.min(b.mask.width - 1, Math.floor(x / sx)), my = Math.min(b.mask.height - 1, Math.floor(y / sy)); if (b.mask.data[my * b.mask.width + mx]) { const i = (y * img.width + x) * 4; img.data[i] = r; img.data[i + 1] = g; img.data[i + 2] = bl; img.data[i + 3] = 110; } }
        const tmp = document.createElement('canvas'); tmp.width = img.width; tmp.height = img.height; tmp.getContext('2d')!.putImageData(img, 0, 0);
        ctx.drawImage(tmp, b.x * sx, b.y * sy);
      }
      ctx.strokeStyle = col; ctx.strokeRect(b.x * sx, b.y * sy, Math.max(3, b.w * sx), Math.max(3, b.h * sy));
      if (b.keypoints) for (const [kx, ky] of b.keypoints) { ctx.fillStyle = col; ctx.beginPath(); ctx.arc(kx * sx, ky * sy, 3, 0, Math.PI * 2); ctx.fill(); }
      if (b.id !== undefined && (b.vx || b.vy)) { ctx.beginPath(); const cx = (b.x + b.w / 2) * sx, cy = (b.y + b.h / 2) * sy; ctx.moveTo(cx, cy); ctx.lineTo(cx + (b.vx ?? 0) * sx * 0.5, cy + (b.vy ?? 0) * sy * 0.5); ctx.stroke(); }
      const label = `${b.cls}${b.id !== undefined ? ` #${b.id}` : ''} ${(b.score * 100).toFixed(0)}%${b.z ? ` ${(b.z / 1000).toFixed(2)}m` : ''}${b.attr && 'ripe' in b.attr ? (b.attr.ripe ? ' ripe' : ' unripe') : ''}`;
      ctx.fillStyle = 'rgba(0,0,0,0.6)'; const tw = ctx.measureText(label).width; ctx.fillRect(b.x * sx, b.y * sy - 13, tw + 6, 13); ctx.fillStyle = col; ctx.fillText(label, b.x * sx + 3, b.y * sy - 3);
    }
    if (out.vla) { ctx.fillStyle = '#fff'; ctx.fillText(`VLA [${out.vla.action.map((v) => v.toFixed(2)).join(' ')}] ${out.vla.text ?? ''}`, 8, canvas.height - 8); }
    // BEV of the cloud
    const bctx = bev.getContext('2d')!;
    bctx.fillStyle = '#1b1f26'; bctx.fillRect(0, 0, bev.width, bev.height);
    if (out.cloud) {
      bev.style.display = '';
      const c = out.cloud.cloud; const n = pointCount(c);
      const camW = out.frame.camWorld; const cx0 = camW[12], cy0 = camW[13];
      const R = cfg ? Math.max(3000, cfg.workingDistance * 1000 * 4) : 10000;
      const X = (x: number) => bev.width / 2 + ((x - cx0) / R) * (bev.width / 2), Y = (y: number) => bev.height / 2 - ((y - cy0) / R) * (bev.height / 2);
      for (let i = 0; i < n; i++) { const z = c.xyz[i * 3 + 2]; const lab = c.label?.[i] ?? 0; bctx.fillStyle = lab ? colorFor(String(lab)) : z < 200 ? 'rgba(120,120,120,0.5)' : `hsl(${Math.min(120, z / 30)} 80% 55%)`; bctx.fillRect(X(c.xyz[i * 3]), Y(c.xyz[i * 3 + 1]), 1.5, 1.5); }
      bctx.strokeStyle = '#ffd43b';
      for (const cl of out.cloud.clusters) { bctx.strokeRect(X(cl.min[0]), Y(cl.max[1]), Math.max(2, ((cl.max[0] - cl.min[0]) / R) * (bev.width / 2)), Math.max(2, ((cl.max[1] - cl.min[1]) / R) * (bev.height / 2))); }
      bctx.strokeStyle = '#51cf66'; bctx.lineWidth = 2;
      for (const r of out.cloud.rows) { bctx.beginPath(); bctx.moveTo(X(r.point[0] - r.dir[0] * r.length / 2), Y(r.point[1] - r.dir[1] * r.length / 2)); bctx.lineTo(X(r.point[0] + r.dir[0] * r.length / 2), Y(r.point[1] + r.dir[1] * r.length / 2)); bctx.stroke(); }
      bctx.lineWidth = 1; bctx.fillStyle = '#ff922b'; bctx.beginPath(); bctx.arc(X(cx0), Y(cy0), 4, 0, Math.PI * 2); bctx.fill();
      bctx.fillStyle = '#ccc'; bctx.font = '11px sans-serif'; bctx.fillText(`${n} pts · ${out.cloud.clusters.length} clusters · ${out.cloud.rows.length} rows · ±${(R / 1000).toFixed(0)} m`, 6, 14);
    } else bev.style.display = 'none';
  };
  const run = async (opts: { prompt?: string } = {}) => {
    const cam = camOf();
    if (!cam || busy) return;
    const rt = ensureVisionRuntime(cam, { serverBase: app.serverHttpBase(), ros: (app as any).ros, onOutput: (o, c) => {
      const cfg = getVisionStack(c);
      if (cfg?.publishUrl && performance.now() - lastWebhook > 200) { lastWebhook = performance.now(); fetch(cfg.publishUrl, { method: 'POST', headers: { 'content-type': 'application/json' }, body: JSON.stringify(visionSummary(c, o)) }).catch((e) => app.log(`webhook ${cfg.publishUrl}: ${(e as Error).message}`, 'warn')); }
      const ros = (app as any).ros;
      if (cfg?.publishRos && ros?.connected) { const parent = c.parent instanceof MobileRobot ? c.parent : null; publishVisionOutput(ros, c, o, { namespace: parent?.rosNamespace ?? '' }); }
    } });
    if (!rt) { info.textContent = t('No vision stack on this camera — press Stack… to choose one.'); draw(cam, null); return; }
    busy = true;
    try {
      const px = capturePixels(cam);
      const robot = cam.parent instanceof Robot ? cam.parent : (cam.parent?.parent instanceof Robot ? cam.parent.parent : null);
      const out = await rt.step(app.station, { rgba: px?.rgba, dataUrl: px?.dataUrl, time: performance.now() / 1000, prompt: opts.prompt, instruction: opts.prompt, tcp: robot ? robot.poseTCPAbs() : undefined, proprio: robot ? { joints: robot.joints(), tcp: Array.from(robot.poseTCPAbs()) } : undefined });
      lastOut = out;
      draw(cam, out);
      info.textContent = summarize(out, rt.cfg);
      const st = rt.stats; const p = st.tp + st.fp ? st.tp / (st.tp + st.fp) : 0, r = st.tp + st.fn ? st.tp / (st.tp + st.fn) : 0;
      statsEl.textContent = `${st.frames} frames · precision ${(p * 100).toFixed(0)} % · recall ${(r * 100).toFixed(0)} % · position error ${st.posErrN ? (st.posErrSum / st.posErrN).toFixed(0) : '–'} mm · ${st.lastLatencyMs.toFixed(0)} ms/frame${out.frame.truth ? ` · truth ${out.frame.truth.filter((x) => x.visible).length} visible` : ''}`;
      if (out.text) answer.textContent = out.text;
      if (out.vla?.tcp && robot && rt.cfg.vla && opts.prompt !== undefined) { const ik = robot.solveIK(out.vla.tcp); if (ik.ok) app.cmd(() => robot.setJoints(ik.joints)); }
      if (follow.checked) doFollow(out);
      for (const w of out.warnings) app.log(`${cam.name}: ${w}`, 'warn');
    } catch (e) { info.textContent = `Vision error: ${(e as Error).message}`; app.log(`vision: ${(e as Error).message}`, 'error'); }
    finally { busy = false; }
  };
  let lastFollow = 0;
  const doFollow = (out: PipelineOutput) => {
    const robot = robotOf(); const cam = camOf();
    if (!robot || !cam) return;
    const K = intrinsicsFromCamera(cam);
    const tracked = out.detections.filter((b) => b.id !== undefined).sort((a, b) => (a.id! - b.id!))[0] ?? out.detections[0] ?? null;
    const cmd = followCommand(tracked, K, { desiredRange: (getVisionStack(cam)?.workingDistance ?? 2) * 1000, maxSpeed: robot.kin.maxSpeed, maxYawRate: robot.kin.maxYawRate });
    const nowS = performance.now() / 1000; const dt = lastFollow ? Math.min(0.5, nowS - lastFollow) : 0.1; lastFollow = nowS;
    if (!app.worldRunning) { integrate(robot, { v: cmd.v, omega: cmd.omega }, dt); robot.notify('state'); }
    else robot.state.path = null;
    robot.setParam('follow', { state: cmd.state, v: Math.round(cmd.v), omega: Math.round(cmd.omega), target: tracked ? `${tracked.cls}#${tracked.id ?? ''}` : null } as any);
  };
  const createTargets = () => {
    const cam = camOf(); if (!cam || !lastOut) return toast('Run the pipeline first', 'warn');
    if (!lastOut.targets.length) return toast('No 3D positions — enable the "6D pose / 3D position" task and a depth-capable stack', 'warn');
    const robot = app.activeRobot;
    const n = app.cmd(() => createTargetsFromOutput(app.station, cam, lastOut!, { robotId: robot?.id ?? null }).length);
    app.log(`${cam.name}: ${n / 2} grasp + approach target pairs created under "Vision ${cam.name}"`);
    toast(`${n / 2} targets created`, 'ok');
  };
  const analyseImportedCloud = () => {
    const clouds: Map<string, PointCloud> | undefined = (app as any).pointClouds;
    const selObj = app.station.selection.find((i) => clouds?.has(i.id));
    const cloud = selObj ? clouds!.get(selObj.id)! : clouds ? [...clouds.values()].pop() : undefined;
    if (!cloud) return toast('Drop a .pcd / .ply file first (it appears as an object with points)', 'warn');
    const a = analyseCloud(cloud, { voxel: 50, tol: 250, minPts: 10 });
    const cam = camOf();
    lastOut = { frame: { width: 1, height: 1, K: { fx: 1, fy: 1, cx: 0, cy: 0, width: 1, height: 1 }, camWorld: transl(a.cloud.xyz[0] ?? 0, a.cloud.xyz[1] ?? 0, 0), time: 0 }, results: {}, detections: [], cloud: a, targets: [], warnings: [] };
    if (cam) draw(cam, lastOut); else { const bctx = bev.getContext('2d')!; bctx.fillStyle = '#1b1f26'; bctx.fillRect(0, 0, bev.width, bev.height); }
    info.textContent = `${pointCount(cloud)} points → ${pointCount(a.cloud)} after voxel · ground ${pointCount(a.ground)} · objects ${pointCount(a.objects)} · ${a.clusters.length} clusters (${a.classes.join(', ')}) · ${a.rows.length} row lines`;
    app.log(info.textContent);
  };
  const exportCloud = () => {
    const out = lastOut; if (!out?.cloud) return toast('No point cloud in the last run', 'warn');
    downloadBlob(`${camOf()?.name ?? 'cloud'}.pcd`, new Blob([writePCD(out.cloud.cloud).buffer as ArrayBuffer], { type: 'application/octet-stream' }));
  };
  const el = h('div', { class: 'pad vision-panel' },
    h('div', { class: 'btn-row' }, h('span', null, t('Camera: ')), sel,
      h('button', { class: 'btn small', onClick: () => { const c = camOf(); if (c) visionStackDialog(app, c); else addCamera(app); } }, t('Stack…')),
      h('button', { class: 'btn small', onClick: () => addCamera(app) }, t('Add camera')),
      h('button', { class: 'btn small primary', onClick: () => run() }, t('Run')),
      h('label', null, live, ' ', t('live')), h('label', null, showTruth, ' ', t('truth')),
      h('button', { class: 'btn small', onClick: createTargets }, t('Targets → station')),
      h('button', { class: 'btn small', onClick: () => { const c = camOf(); if (c) exportVisionPackage(app, c); } }, t('Export ROS 2')),
      h('button', { class: 'btn small', onClick: () => { const c = camOf(); if (c) { visionRuntimeOf(c)?.reset(); lastOut = null; } } }, t('Reset stats'))),
    h('div', { class: 'btn-row' }, h('span', null, t('Follow with: ')), robotSel, h('label', null, follow, ' ', t('follow tracked target')),
      h('button', { class: 'btn small', onClick: analyseImportedCloud }, t('Analyse imported cloud')),
      h('button', { class: 'btn small', onClick: exportCloud }, t('Export cloud (.pcd)')),
      prompt, h('button', { class: 'btn small', onClick: () => run({ prompt: prompt.value }) }, t('Ask / step'))),
    info, statsEl, answer,
    h('div', { class: 'vision-views' }, canvas, bev));
  bev.style.display = 'none';
  const timer = setInterval(() => { if (live.checked && el.offsetParent !== null) run(); }, 400);
  const render = () => { refreshLists(); const cam = camOf(); if (!cam) { info.textContent = t('Add a camera (Add › Camera / vision sensor) and select a vision stack.'); return; } if (!lastOut) run(); else draw(cam, lastOut); };
  return { el, render, stop: () => clearInterval(timer) };
}

function hslToRgb(hsl: string): [number, number, number] {
  const m = hsl.match(/hsl\((\d+)\s+(\d+)%\s+(\d+)%\)/); if (!m) return [255, 255, 255];
  const hh = Number(m[1]) / 360, s = Number(m[2]) / 100, l = Number(m[3]) / 100;
  const q = l < 0.5 ? l * (1 + s) : l + s - l * s, p = 2 * l - q;
  const f = (tt: number) => { if (tt < 0) tt += 1; if (tt > 1) tt -= 1; if (tt < 1 / 6) return p + (q - p) * 6 * tt; if (tt < 1 / 2) return q; if (tt < 2 / 3) return p + (q - p) * (2 / 3 - tt) * 6; return p; };
  return [Math.round(f(hh + 1 / 3) * 255), Math.round(f(hh) * 255), Math.round(f(hh - 1 / 3) * 255)];
}

/** Minimal formatting helper re-export used by the properties panel. */
export { fmt };
