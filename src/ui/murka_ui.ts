/**
 * File › MURKA examples (contract C4): the MURKA reproductions of R3 (retail store, robot-retailer-murka) and R4 (orchard
 * agro robots, murca_garden_ws / murca_experiment) driven from the app. The MURKA module is loaded on demand
 * (`import('../murka/index')`); an entry whose provider function is missing says so instead of failing.
 *
 * In the browser the MURKA project files come from a folder the user picks (the MURKA_Simulation folder or one of its
 * projects: the builders find the Webots world and its YAML files by their path suffix).
 */
import type { App } from '../app';
import { MenuEntry, dialog, toast, h, pickFolder, downloadText } from './dom';
import type { Station } from '../core/items/item';

type Murka = Record<string, any>;
const loadMurka = async (): Promise<Murka> => (await import('../murka/index')) as Murka;
async function provider(name: string): Promise<((...a: any[]) => any) | null> {
  try {
    const m = await loadMurka();
    if (typeof m[name] === 'function') return m[name];
  } catch (e) { toast(`MURKA module failed to load: ${(e as Error).message}`, 'error', 8000); return null; }
  toast(`${name} is not available in this build`, 'warn', 6000);
  return null;
}

/** Build / install / log trees of a ROS workspace, VCS and caches: never needed by the MURKA builders. */
const SKIP_DIRS = /(^|\/)(build|install|log|\.git|\.vscode|__pycache__|node_modules|artifacts|_audit[^/]*|_legacy[^/]*)\//;

/** What the MURKA builders read: Webots worlds / PROTOs, their meshes and textures, YAML / JSON configuration. */
const NEEDED = /\.(wbt|proto|ya?ml|json|stl|dae|obj|mtl|png|jpe?g|hdr|tiff?|bmp|glb|gltf|wbo|csv|txt|xml|py)$/i;
/** Datasets, checkpoints, recordings, renders: never read by the builders (the MURKA_Simulation folder holds GBs of them). */
const SKIP_HEAVY = /(^|\/)(datasets?|checkpoints?|weights|models?_cache|rosbags?|bags|recordings?|videos?|renders?|frames|murka_vla_study|wandb|runs_out|figures)\//i;
const MAX_FILE = 64 * 1024 * 1024;

/**
 * The files of a picked folder the MURKA builders need: under `base` when the folder holds it (the MURKA_Simulation
 * root holds every project: only the chosen one is read), Webots / configuration types, at most 64 MB each. Picking
 * the whole MURKA_Simulation folder read every file before (19 653 files, 17.5 GB with the VLA checkpoints).
 */
export function selectProjectFiles(files: File[], base?: string): File[] {
  const rel = (f: File) => ((f as any).webkitRelativePath || (f as any).relativePath || f.name).replace(/\\/g, '/') as string;
  let keep = files.filter((f) => !SKIP_DIRS.test(rel(f)) && !SKIP_HEAVY.test(rel(f)) && NEEDED.test(f.name) && f.size <= MAX_FILE);
  if (base) {
    const b = base.replace(/\/+$/, '');
    const under = keep.filter((f) => rel(f).includes(`/${b}/`) || rel(f).startsWith(`${b}/`));
    if (under.some((f) => /\.wbt$/i.test(f.name))) keep = under;
  }
  return keep;
}

/**
 * Files of a picked folder as a path -> bytes map (paths relative to the picked folder's parent, like the importers
 * expect). Files that cannot be read (cloud placeholders of a synced folder, broken links) are skipped and counted.
 */
export async function readProjectFiles(app: App, files: File[], base?: string): Promise<{ map: Map<string, Uint8Array>; skipped: string[] }> {
  const map = new Map<string, Uint8Array>();
  const skipped: string[] = [];
  const keep = selectProjectFiles(files, base);
  if (keep.length < files.length) app.log(`MURKA: reading ${keep.length} of ${files.length} files (${base ? `${base}, ` : ''}Webots / configuration files only)`);
  const label = 'Reading MURKA files';
  app.events.emit('progress', { label, detail: `${keep.length} files` });
  try {
    let n = 0;
    for (const f of keep) {
      const p = ((f as any).webkitRelativePath || (f as any).relativePath || f.name).replace(/\\/g, '/');
      try { map.set(p, new Uint8Array(await f.arrayBuffer())); } catch { skipped.push(p); }
      if (++n % 200 === 0) app.events.emit('progress', { label, detail: `${n} / ${keep.length} files` });
    }
  } finally { app.events.emit('progress', { label, done: true }); }
  if (skipped.length) app.log(`${skipped.length} file(s) could not be read and were skipped: ${skipped.slice(0, 5).join(', ')}${skipped.length > 5 ? ', …' : ''}`, 'warn');
  return { map, skipped };
}

async function pickProjectFiles(app: App, hint: string, base?: string): Promise<Map<string, Uint8Array> | null> {
  toast(`Select ${hint}`, 'info', 6000);
  const files = await pickFolder();
  if (!files.length) return null;
  return (await readProjectFiles(app, files, base)).map;
}

/** Retail tasks of each store variant: the MURKA_Simulation copy (murka / murka_classic) has only depalletize / collect_order. */
export const RETAIL_TASKS: Record<string, string[]> = {
  v2: ['full', 'scan', 'depalletize', 'order', 'goto', 'dock', 'undock', 'home'],
  v1: ['full', 'scan', 'depalletize', 'order', 'goto', 'dock', 'undock', 'home'],
  murka: ['depalletize', 'order'],
  murka_classic: ['depalletize', 'order'],
};
/** Task arguments of the dialog -> runMurkaRetailMission options (full: scan= / depal= / order=; others: positional). */
export function retailTaskOptions(mission: string, text: string): { params?: Record<string, string>; args?: string[] } {
  const parts = String(text ?? '').trim().split(/\s+/).filter(Boolean);
  if (mission === 'full') {
    const params: Record<string, string> = {};
    for (const q of parts) { const i = q.indexOf('='); if (i > 0) params[q.slice(0, i)] = q.slice(i + 1); }
    return Object.keys(params).length ? { params } : {};
  }
  return parts.length ? { args: parts } : {};
}

/** A running retail mission (one at a time): Stop cancels it. */
let retailRun: { cancel(): void } | null = null;

export async function retailMissionDialog(app: App): Promise<void> {
  const build = await provider('buildMurkaRetailStation');
  const run = await provider('runMurkaRetailMission');
  if (!build || !run) return;
  // the store already open (and its simulation state: inventory, held boxes, robot pose) can take the next task
  const { retailStationOf, RETAIL_VARIANTS } = await loadMurka();
  const current = typeof retailStationOf === 'function' ? retailStationOf(app.station) : null;
  const variants = [{ value: 'v2', label: 'robot-retailer-murka-v2 (MURKA v2)' }, { value: 'v1', label: 'robot-retailer-murka (v1)' }, { value: 'murka', label: 'murka (MURKA_Simulation copy, v2 robot)' }, { value: 'murka_classic', label: 'murka_classic (MURKA_Simulation copy, classic robot)' }];
  const variant0 = current?.variant ?? 'v2';
  const hintFor = (m: string) => ({ full: 'full: scan=S1,S2 depal=pallet_1,box_red,bulk_B1,1 order=tea_box,coffee_can (empty: the MURKA defaults)', scan: 'scan: shelf ids (S0 S1 …)', depalletize: 'depalletize: pallet sku target_shelf count (e.g. pallet_1 box_red bulk_B1 1)', order: 'order: SKUs (e.g. tea_box coffee_can)', goto: 'goto: location name, or x y yaw' } as Record<string, string>)[m] ?? '';
  const argsHint = h('p', { class: 'hint' }, hintFor(RETAIL_TASKS[variant0][0]));
  const r = await dialog<{ variant: string; mission: string; args: string; reuse: boolean; speed: number }>('MURKA retail store mission', [
    { key: 'variant', label: 'Store variant', type: 'select', value: variant0, options: variants },
    { key: 'mission', label: 'Task', type: 'select', value: RETAIL_TASKS[variant0][0], options: RETAIL_TASKS[variant0].map((m) => ({ value: m, label: m })) },
    { key: 'args', label: 'Task arguments', type: 'text', value: '' },
    { key: 'reuse', label: current ? `Run on the open store (${current.variant}: keep inventory, held boxes and the robot where it is)` : 'Run on the open store (no MURKA store is open)', type: 'checkbox', value: !!current },
    { key: 'speed', label: 'Simulation speed (×)', type: 'number', value: 4 },
  ], {
    body: h('div', null, argsHint, h('p', { class: 'hint' }, 'A new store is built from a picked folder: the MURKA_Simulation folder (only the chosen project is read) or the project itself. The store world, its products.yaml and locations.yaml are read from it; the tasks run on the studio\'s own simulator.')),
    onChange: (v) => {
      // the task list follows the variant (the MURKA_Simulation copy has depalletize / collect_order only)
      const sel = [...document.querySelectorAll<HTMLSelectElement>('.dialog select')].find((x) => [...x.options].some((o) => o.value === 'depalletize'));
      const tasks = RETAIL_TASKS[String(v.variant)] ?? RETAIL_TASKS.v2;
      if (sel && [...sel.options].map((o) => o.value).join() !== tasks.join()) { const keep = tasks.includes(sel.value) ? sel.value : tasks[0]; sel.replaceChildren(...tasks.map((t) => h('option', { value: t }, t))); sel.value = keep; }
      argsHint.textContent = hintFor(String(sel?.value ?? v.mission));
    },
  });
  if (!r) return;
  const tasks = RETAIL_TASKS[r.variant] ?? RETAIL_TASKS.v2;
  if (!tasks.includes(r.mission)) { toast(`${r.variant}: task ${r.mission} is not in that project's orchestrator (${tasks.join(', ')})`, 'warn', 8000); return; }
  let res: any = r.reuse && current && current.variant === r.variant ? current : null;
  if (!res) {
    const info = RETAIL_VARIANTS?.[r.variant];
    const files = await pickProjectFiles(app, 'the MURKA_Simulation folder (or the store project)', info?.base);
    if (!files) return;
    try { res = await build({ variant: r.variant, files, assets: app.assets }); } catch (e) { toast(`Retail station: ${(e as Error).message}`, 'error', 8000); return; }
    app.setStation(res.station as Station);
    for (const line of res.report ?? []) app.log(String(line));
  }
  retailRun?.cancel();
  const label = `[MURKA ${r.mission}]`;
  const fmtEv = (e: any) => `${label} ${typeof e === 'string' ? e : `${Number(e?.t ?? 0).toFixed(1)} s ${e?.state ?? ''} ${e?.phase ?? ''} ${e?.detail ?? ''} ${e?.percent !== undefined ? `${Math.round(e.percent)} %` : ''}`.replace(/\s+/g, ' ').trim()}`;
  let handle: any;
  try { handle = run(res, { mission: r.mission, ...retailTaskOptions(r.mission, r.args), runtime: 'realtime', speed: Number(r.speed) || 1, onEvent: (e: any) => app.log(fmtEv(e)) }); }
  catch (e) { toast(`MURKA ${r.mission}: ${(e as Error).message}`, 'error', 8000); return; }
  // the default session of runMurkaRetailMission ignores opts.onEvent: hook the orchestrator's progress for this task
  // and give the previous hook back afterwards (a later task from the Control tab is not labelled with this one)
  const orchOpts = handle.session?.orch?.opts;
  const prevHook = orchOpts?.onEvent;
  if (orchOpts) orchOpts.onEvent = (e: any) => app.log(fmtEv(e));
  retailRun = handle;
  toast(`MURKA ${r.mission} started (${r.variant}${res === current ? ', open store' : ''})`, 'ok');
  const out = await handle.done.catch((e: Error) => ({ outcome: 'error', stages: [], error: e.message }));
  if (orchOpts) orchOpts.onEvent = prevHook;
  if (retailRun === handle) retailRun = null;
  const stages = (out.stages ?? []).map((s: any) => `${s.name}: ${s.outcome}`).join(', ');
  app.log(`MURKA ${r.mission}: ${out.outcome}${out.simTime !== undefined ? ` in ${Number(out.simTime).toFixed(0)} s` : ''}${stages ? ` — ${stages}` : ''}${out.error ? ` (${out.error})` : ''}`, out.outcome === 'done' ? 'info' : 'warn');
  toast(`MURKA ${r.mission}: ${out.outcome}`, out.outcome === 'done' ? 'ok' : 'warn', 8000);
}

/** Build the MURKA orchard (orchard_demo.wbt) from a picked folder; returns its station or null. */
async function orchardStation(app: App): Promise<Station | null> {
  const build = await provider('buildMurkaOrchardStation');
  if (!build) return null;
  const cur = app.station;
  if (['UTP-G-01', 'UTP-G-04', 'UTP-A-01'].every((n) => cur.find(n))) return cur;   // already open
  const files = await pickProjectFiles(app, 'the MURKA_Simulation folder (or murca_garden_ws)', 'murca_garden_ws');
  if (!files) return null;
  try {
    const res = await build({ world: 'orchard_demo', files, assets: app.assets });
    app.setStation(res.station as Station);
    for (const line of res.report ?? []) app.log(String(line));
    return res.station as Station;
  } catch (e) { toast(`Orchard station: ${(e as Error).message}`, 'error', 8000); return null; }
}

export async function orchardMissionB(app: App): Promise<void> {
  const st = await orchardStation(app);
  if (!st) return;
  const { MISSION_B } = await import('../io/webots/murka_missions');
  try {
    const { tEnd } = await app.playScriptedFleet(undefined, { depotModules: [...MISSION_B.DEPOT_MODULES] });
    toast(`Mission B playing on the world clock (${tEnd.toFixed(0)} s)`, 'ok');
  } catch (e) { toast(`Mission B: ${(e as Error).message}`, 'error', 8000); }
}

export async function s1BenchmarkDialog(app: App): Promise<void> {
  const run = await provider('runS1Benchmark');
  if (!run) return;
  const r = await dialog<{ level: string; controller: string; seed: number }>('MURKA benchmark S1 (scouting → harvest → crate logistics)', [
    { key: 'level', label: 'Level', type: 'select', value: 'A', options: ['A', 'B', 'C', 'D'].map((v) => ({ value: v, label: v })) },
    { key: 'controller', label: 'Allocation', type: 'select', value: 'hungarian', options: [{ value: 'hungarian', label: 'hungarian' }, { value: 'contract', label: 'contract net' }, { value: 'hybrid', label: 'hybrid' }] },
    { key: 'seed', label: 'Seed', type: 'number', value: 0 },
  ]);
  if (!r) return;
  const st = await orchardStation(app);
  if (!st) return;
  // the benchmark computes the whole run first (planning, allocation, contact replay): say so before the page is busy
  const label = 'S1 benchmark';
  app.events.emit('progress', { label, detail: `level ${r.level}, ${r.controller}: planning the run` });
  toast('S1: computing the run (planning and allocation take a while), then it plays on the world clock', 'info', 6000);
  await new Promise((res) => setTimeout(res, 60));
  try {
    const res = run(st, { level: r.level, controller: r.controller, seed: Number(r.seed) || 0, assets: app.assets, log: (m: string) => app.log(`[S1] ${m}`) });
    const metrics = res.metrics ?? res;
    app.log(`S1 ${r.level}/${r.controller}: ${JSON.stringify(metrics).slice(0, 600)}`);
    downloadText(`s1_${r.level}_${r.controller}_seed${r.seed}.json`, JSON.stringify(res, (k, v) => (typeof v === 'function' ? undefined : v), 1), 'application/json');
    if (res.timeline) playS1Timeline(app, st, res.timeline);
    toast(`S1 benchmark computed (metrics downloaded): playing ${Number(res.timeline?.end ?? 0).toFixed(0)} s on the world clock`, 'ok', 7000);
  } catch (e) { toast(`S1: ${(e as Error).message}`, 'error', 8000); }
  finally { app.events.emit('progress', { label, done: true }); }
}

/** Play an S1 run on the app's world clock: every robot is posed by the run's timeline (as the S1 report describes it). */
export function playS1Timeline(app: App, st: Station, timeline: { end: number; apply(t: number): void }): void {
  let t = 0;
  const hook = (dt: number) => {
    if (app.station !== st) return;
    t = Math.min(timeline.end, t + dt);
    timeline.apply(t);
    if (t >= timeline.end) { const i = app.worldHooks.indexOf(hook); if (i >= 0) app.worldHooks.splice(i, 1); app.log(`S1 playback finished at ${t.toFixed(0)} s`); }
  };
  timeline.apply(0);
  app.worldHooks.push(hook);
  app.startWorld();
}

export async function agroExperimentDialog(app: App): Promise<void> {
  const run = await provider('runAgroExperiment');
  if (!run) return;
  const r = await dialog<{ id: string; runs: number }>('MURKA agro experiments (article exp1–exp5)', [
    { key: 'id', label: 'Experiment', type: 'select', value: 'exp1', options: [
      { value: 'exp1', label: 'exp1 — mission presets A/B/C' }, { value: 'exp2', label: 'exp2 — task allocation (hungarian / murca / hybrid)' },
      { value: 'exp3', label: 'exp3 — full scenario' }, { value: 'exp4', label: 'exp4 — fleet scaling' }, { value: 'exp5', label: 'exp5 — energy of equipment recipes' }] },
    { key: 'runs', label: 'Runs per point (0 = the Python default)', type: 'number', value: 0 },
  ], { body: h('p', { class: 'hint' }, 'The KPI JSON files have the schema of murca_experiment artifacts/experiments/*.json and are downloaded.') });
  if (!r) return;
  try {
    const res = run(r.id, Number(r.runs) > 0 ? { runs: Number(r.runs) } : {});
    const files = Object.entries(res.files ?? {}) as Array<[string, unknown]>;
    for (const [name, json] of files) downloadText(name, JSON.stringify(json, null, 1), 'application/json');
    app.log(`MURKA ${r.id}: ${files.map(([n]) => n).join(', ')}`);
    toast(`${r.id}: ${files.length} KPI file(s) downloaded`, 'ok');
  } catch (e) { toast(`${r.id}: ${(e as Error).message}`, 'error', 8000); }
}

/** An action's failure becomes a toast (never an unhandled rejection). */
function guard(p: Promise<unknown>): void { p.catch((e) => toast(`MURKA: ${(e as Error)?.message ?? e}`, 'error', 8000)); }

/** File › MURKA examples submenu. */
export function murkaMenu(app: App): MenuEntry[] {
  return [
    { label: 'Retail store mission (robot-retailer-murka)…', action: () => guard(retailMissionDialog(app)) },
    { label: 'Stop retail mission', action: () => { if (retailRun) { retailRun.cancel(); retailRun = null; toast('MURKA mission canceled', 'info'); } else toast('No MURKA mission running', 'info'); } },
    { separator: true },
    { label: 'Orchard: Mission B (scripted fleet, orchard_demo.wbt)…', action: () => guard(orchardMissionB(app)) },
    { label: 'Orchard: benchmark S1…', action: () => guard(s1BenchmarkDialog(app)) },
    { label: 'Agro experiments exp1–exp5 (KPI JSON)…', action: () => guard(agroExperimentDialog(app)) },
    { separator: true },
    { label: 'Play scripted fleet (Mission B or a script JSON)…', action: () => guard(scriptedFleetDialog(app)) },
  ];
}

/** Play a scripted fleet (agromarca scripted_fleet JSON) on the world clock; default: MURKA Mission B. */
export async function scriptedFleetDialog(app: App): Promise<void> {
  const r = await dialog<{ source: string }>('Play scripted fleet', [
    { key: 'source', label: 'Script', type: 'select', value: 'missionB', options: [{ value: 'missionB', label: 'MURKA Mission B (orchard_demo.wbt robots UTP-G-01..04, UTP-A-01)' }, { value: 'file', label: 'Script JSON file…' }] },
  ], { body: h('p', { class: 'hint' }, 'Robots and modules are the station items named like the Webots nodes (name or DEF). The script time is the world clock: use Pause / Reset world.') });
  if (!r) return;
  let script: unknown;
  if (r.source === 'file') {
    const { pickFiles } = await import('./dom');
    const files = await pickFiles('.json');
    if (!files.length) return;
    try { script = JSON.parse(await files[0].text()); } catch (e) { toast(`Invalid script JSON: ${(e as Error).message}`, 'error'); return; }
  }
  try {
    const { MISSION_B } = await import('../io/webots/murka_missions');
    const { tEnd } = await app.playScriptedFleet(script, { depotModules: script ? [] : [...MISSION_B.DEPOT_MODULES] });
    toast(`Scripted fleet playing (${tEnd.toFixed(0)} s of script time)`, 'ok');
  } catch (e) { toast(`Scripted fleet: ${(e as Error).message}`, 'error', 8000); }
}
