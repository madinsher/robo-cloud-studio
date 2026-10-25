/**
 * runMurkaRetailMission (contract C1): runs a MURKA task (`full` mission or a single task) on a station built by
 * buildMurkaRetailStation — the orchestrator FSM on the studio's FsmExecutor, ticking the station simulation (the
 * shared StationRuntime clock advances with it) — and reports the stages, the camera inventory, the simulated mission
 * time against the Webots reference, collisions and the success criteria of the MURKA mission test.
 */
import type { Station } from '../../core/items/item';
import type { AssetStore } from '../../scene/assets';
import { MuromSim, CollisionEvent } from './sim';
import { Navigator } from './nav';
import { ManipServer } from './manip';
import { RetailOrchestrator, parseMission, MissionTask, ProgressEvent, describeHeld } from './orchestrator';
import { registerStationFsmProvider } from '../../ctl/world';
import { loadCatalog, Appearance } from './perception';
import { RetailStationResult, retailStationOf } from './station';
import { STUDIO_APPEARANCE } from './appearance';
import { TaskRunner } from './tasks';

export interface RetailStage { name: string; outcome: 'done' | 'error' | 'canceled' | 'skipped'; t0: number; t1: number; detail: string }
export interface RetailChecks {
  fsmDone: boolean;
  scan: { visible: number; correct: number; wrong: string[]; webotsVisible: number | null } | null;
  depal: { moved: string[]; ok: boolean } | null;
  order: { onPacking: Record<string, string[]>; ok: boolean } | null;
  docked: { distance: number; yawDeg: number; ok: boolean } | null;
  collisions: { armLinks: number; hands: number; loads: number; base: number; ok: boolean };
}
export interface RetailMissionResult {
  outcome: 'done' | 'error' | 'canceled';
  stages: RetailStage[];
  /** slot -> [sku, confidence] from the camera ('' empty, 'unknown' not recognised). */
  inventory: Record<string, [string, number]>;
  simTime: number;
  collisions: CollisionEvent[];
  baseContacts: CollisionEvent[];
  checks: RetailChecks;
  /**
   * MURKA reference (README "Что проверено"): mission_test.py measures WALL time — Webots `full` 1069 s at RTF 0.84
   * (v2) / 706 s at RTF 0.73 (v1), MuJoCo 990 s / 499 s at RTF 1.0. webotsSim = wall × RTF is the simulated-time
   * estimate the studio time (simulated) is compared with.
   */
  reference: { webotsWall: number | null; webotsRtf: number | null; webotsSim: number | null; mujoco: number | null; ratio: number | null };
  battery: { pct: number; docked: boolean };
  detail: string;
  /** FSM trace (state path, outcome, target) and the progress telemetry (/murom/fsm/progress). */
  trace: Array<{ t: number; path: string; outcome: string; to: string }>;
  progress: ProgressEvent[];
  registry: Record<string, { sku: string; location: string; pos: [number, number, number]; yaw: number }>;
  report: string[];
  dsl: string;
  wallMs: number;
}
export interface RetailMissionOptions {
  /** full (default), depalletize, order, scan, goto, dock, undock, home */
  mission?: string;
  /** full: scan= / depal= / order= values; other tasks: args= (comma separated). */
  params?: Record<string, string>;
  args?: string[];
  /** 'headless' (default in Node: as fast as possible) or 'realtime' (sim time follows the wall clock × speed). */
  runtime?: 'headless' | 'realtime';
  speed?: number;
  dt?: number;
  /** Simulated time limit (s). */
  maxTime?: number;
  onEvent?: (e: ProgressEvent) => void;
  /** Called after every tick batch (UI redraw). */
  onTick?: (simTime: number) => void;
  /** Called after every simulation tick (tests: cancel at a given moment). */
  onStep?: (simTime: number, state: string) => void;
  verifyWithCamera?: boolean;
  appearance?: Appearance | null;
  assets?: AssetStore | null;
  dumpFrames?: boolean;
  /**
   * Deviations from the MURKA code, off by default. bothCarryZ: height (m) of the two-hand carry grip (BOTH_CARRY z =
   * 1.50 in pick_place_server): at 1.50 m the fingertips of the inward palms reach the underside of the bulk L1 board
   * (1.62 m) when the robot glides into the bulk L0 parking with the box; 1.46 m keeps 2 cm clearance.
   */
  fixes?: { bothCarryZ?: number };
  /** Reuse the simulation state of an earlier task (inventory, held objects, parking) — a task queue. Default: the
   *  station's own session (kept between tasks like the ROS nodes) unless appearance / dumpFrames / verifyWithCamera /
   *  assets ask for a fresh one. */
  session?: RetailSession;
  /** Run this (edited) FSM document instead of the task's generated one (its skills are the murom.* library). */
  dsl?: string;
}

/** Simulation + servers + orchestrator of one station (kept between tasks like the ROS nodes). */
export interface RetailSession { sim: MuromSim; nav: Navigator; manip: ManipServer; orch: RetailOrchestrator; built: RetailStationResult }

export function createRetailSession(built: RetailStationResult, opts: { assets?: AssetStore | null; appearance?: Appearance | null; verifyWithCamera?: boolean; dumpFrames?: boolean; onEvent?: (e: ProgressEvent) => void } = {}): RetailSession {
  const sim = new MuromSim(built, { assets: opts.assets ?? null });
  const orch0: { o?: RetailOrchestrator } = {};
  const nav = new Navigator(sim, built.layout, () => orch0.o?.cancelRequested ?? false);
  const manip = new ManipServer(sim, built.layout);
  const appearance = opts.appearance === undefined ? STUDIO_APPEARANCE[built.variant] ?? null : opts.appearance;
  const orch = new RetailOrchestrator(sim, nav, manip, built.layout, { catalog: loadCatalog(built.layout, appearance), assets: opts.assets ?? null, verifyWithCamera: opts.verifyWithCamera, dumpFrames: opts.dumpFrames, onEvent: opts.onEvent });
  orch0.o = orch;
  return { sim, nav, manip, orch, built };
}

const sessions = new WeakMap<RetailStationResult, RetailSession>();
/** The station's own session (created on first use, kept between tasks). */
export function retailSessionOf(built: RetailStationResult): RetailSession {
  let s = sessions.get(built);
  if (!s) { s = createRetailSession(built, { assets: built.assets ?? null }); sessions.set(built, s); }
  return s;
}

// Control tab: an fsm document with murom.* skills on a MURKA retail station runs on the station's session
registerStationFsmProvider((station, source, log) => {
  const built = retailStationOf(station);
  if (!built || !/skill=murom\./.test(source)) return null;
  const s = retailSessionOf(built);
  const { exec } = s.orch.executor(RetailOrchestrator.taskOfDsl(source, built.layout), source, log);
  return { exec, stop: () => s.orch.cancel() };
});

const yieldToLoop = () => new Promise<void>((r) => setTimeout(r, 0));

/** C1: run a mission; `done` resolves with the result, `cancel()` requests /murom/task/cancel semantics. */
export function runMurkaRetailMission(station: Station | RetailStationResult, opts: RetailMissionOptions = {}): { done: Promise<RetailMissionResult>; cancel(): void; session: RetailSession } {
  const built = 'robot' in station && 'layout' in station ? station : retailStationOf(station as Station);
  if (!built) throw new Error('not a MURKA retail station (build it with buildMurkaRetailStation)');
  const custom = opts.assets !== undefined || opts.appearance !== undefined || opts.dumpFrames !== undefined || opts.verifyWithCamera !== undefined;
  const session = opts.session ?? (custom ? createRetailSession(built, { assets: opts.assets ?? built.assets ?? null, appearance: opts.appearance, verifyWithCamera: opts.verifyWithCamera, dumpFrames: opts.dumpFrames, onEvent: opts.onEvent }) : retailSessionOf(built));
  if (opts.onEvent && opts.session) session.orch.opts.onEvent = opts.onEvent;
  const { sim, orch } = session;
  if (opts.fixes?.bothCarryZ !== undefined) session.manip.bothCarry = [session.manip.bothCarry[0], opts.fixes.bothCarryZ];
  const name = opts.mission ?? 'full';
  const args = opts.args ?? (opts.params ? (['full', 'mission', ''].includes(name) ? Object.entries(opts.params).map(([k, v]) => `${k}=${v}`) : (opts.params.args ?? '').split(',').filter(Boolean)) : []);
  const task = opts.dsl && !opts.mission ? RetailOrchestrator.taskOfDsl(opts.dsl, built.layout) : parseMission(name, args, built.layout);
  const { exec, dsl } = orch.executor(task, opts.dsl);
  const dt = opts.dt ?? 0.05, maxTime = opts.maxTime ?? 3600;
  const t0 = sim.time; const wall0 = Date.now();
  const initial = sim.registry();
  let cancelled = false;
  const done = (async () => {
    const realtime = opts.runtime === 'realtime';
    const speed = opts.speed ?? 1;
    let lastWall = Date.now();
    while (exec.status === 'running' && sim.time - t0 < maxTime) {
      let budget = 400;
      if (realtime) { const now = Date.now(); budget = Math.max(1, Math.min(200, Math.round(((now - lastWall) / 1000) * speed / dt))); lastWall = now; }
      for (let i = 0; i < budget && exec.status === 'running'; i++) { exec.tick(dt); opts.onStep?.(sim.time, String(exec.bb.murom_state ?? '')); }
      opts.onTick?.(sim.time);
      await (realtime ? new Promise<void>((r) => setTimeout(r, 16)) : yieldToLoop());
    }
    if (exec.status === 'running') exec.stop('timeout');
    // _execute epilogue: canceled / error -> head and arms back to the travel pose before the result
    let outcome: RetailMissionResult['outcome'] = exec.outcome === 'done' ? 'done' : 'error';
    if (orch.cancelRequested || cancelled) outcome = 'canceled';
    const bb = exec.bb as Record<string, any>;
    const parts: string[] = [...((bb.failed_items ?? []) as Array<[string, string]>).map(([n, m]) => `${n}: ${m}`), ...(bb.errors ?? [])];
    if (bb.scan_report) parts.unshift(`осмотр: ${bb.scan_report}`);
    if (outcome !== 'done') {
      const tr = new TaskRunner(orch.restorePose(), sim); let n = 0;
      while (!tr.done && n++ < 400) { tr.tick(); sim.step(dt); }
      const held = describeHeld(orch.heldNow()); if (held) parts.push(`в руках остался товар: ${held}`);
    }
    return summarize(session, task, outcome, exec.trace, parts.join('; '), t0, initial, dsl, Date.now() - wall0);
  })();
  return { done, cancel: () => { cancelled = true; orch.cancel(); }, session };
}

function summarize(s: RetailSession, task: MissionTask, outcome: RetailMissionResult['outcome'], trace: RetailMissionResult['trace'], detail: string, t0: number, initial: Record<string, { sku: string; location: string }>, dsl: string, wallMs: number): RetailMissionResult {
  const { sim, orch, built } = s;
  const L = built.layout;
  // stages from the FSM trace: PH_<STAGE> -> <STAGE> machine outcome
  const stages: RetailStage[] = [];
  const root = `MUROM_${task.type}`;
  if (task.type === 'mission') {
    for (const e of trace) {
      const m = new RegExp(`^${root}/PH_(\\w+)$`).exec(e.path);
      if (m) { stages.push({ name: m[1], outcome: 'canceled', t0: e.t - t0, t1: e.t - t0, detail: '' }); continue; }
      const st = stages.find((x) => e.path === `${root}/${x.name}`);
      if (st) { st.outcome = e.outcome === 'done' ? 'done' : 'error'; st.t1 = e.t - t0; }
    }
  } else stages.push({ name: task.type.toUpperCase(), outcome, t0: 0, t1: sim.time - t0, detail });
  for (const st of stages) if (st.outcome === 'error') st.detail = detail;
  // success criteria
  const truth: Record<string, string> = {}; for (const i of L.instances) truth[i.slot] = i.sku;
  const reg = sim.registry();
  const scanned = task.type === 'mission' ? task.scan.length > 0 : task.type === 'scan';
  let scan: RetailChecks['scan'] = null;
  if (scanned) {
    const wrong: string[] = []; let visible = 0, correct = 0;
    for (const [slot, [sku]] of orch.inventory) { if (!slot.startsWith('shelf_')) continue; visible++; if ((truth[slot] ?? '') === sku) correct++; else wrong.push(`${slot}: ${sku || 'пусто'} вместо ${truth[slot] || 'пусто'}`); }
    scan = { visible, correct, wrong, webotsVisible: built.variant === 'v1' || built.variant === 'v2' ? 24 : null };
  }
  let depal: RetailChecks['depal'] = null;
  const dp = task.type === 'mission' ? (task.depal.length ? task.depal : null) : task.type === 'depalletize' ? [task.pallet_id, task.sku, task.target_shelf, task.count] as [string, string, string, number] : null;
  if (dp) { const moved = Object.entries(reg).filter(([n, o]) => o.sku === dp[1] && initial[n]?.location.startsWith(`${dp[0]}_L`) && o.location.startsWith(`${dp[2]}_L`)).map(([n, o]) => `${n} -> ${o.location}`); depal = { moved, ok: moved.length === dp[3] }; }
  let order: RetailChecks['order'] = null;
  const skus = task.type === 'mission' ? task.order : task.type === 'collect_order' ? task.skus : null;
  if (skus && skus.length) { const onPacking: Record<string, string[]> = {}; for (const sku of skus) onPacking[sku] = Object.entries(reg).filter(([, o]) => o.sku === sku && o.location.startsWith('packing_')).map(([n, o]) => `${n} @ ${o.location}`); order = { onPacking, ok: skus.every((k) => onPacking[k].length >= skus.filter((x) => x === k).length) }; }
  let docked: RetailChecks['docked'] = null;
  const dockPose = L.dock?.approach ?? L.targets.dock;
  if ((task.type === 'mission' || task.type === 'dock') && dockPose) { const [x, y, th] = sim.pose(); const d = Math.hypot(x - dockPose.x, y - dockPose.y); const yaw = Math.abs(Math.atan2(Math.sin(th - dockPose.yaw), Math.cos(th - dockPose.yaw))) * 180 / Math.PI; docked = { distance: d, yawDeg: yaw, ok: d <= 0.06 && yaw <= 3 }; }
  const coll = sim.collisions.filter((c) => c.t >= t0);
  const armLinks = coll.filter((c) => /_(upper_arm|forearm)$/.test(c.part)).length, hands = coll.filter((c) => /_hand$/.test(c.part)).length, loads = coll.filter((c) => c.part.startsWith('load ')).length;
  const base = sim.baseContacts.filter((c) => c.t >= t0).length;
  const checks: RetailChecks = { fsmDone: outcome === 'done', scan, depal, order, docked, collisions: { armLinks, hands, loads, base, ok: coll.length === 0 && base === 0 } };
  const simTime = sim.time - t0;
  const full = task.type === 'mission' && task.scan.length > 0 && task.depal.length > 0 && task.order.length > 0;
  const wallRef = full ? built.info.referenceTime : null, rtf = full ? built.info.referenceRtf : null;
  const ref = wallRef !== null && rtf !== null ? wallRef * rtf : null;
  const inventory: Record<string, [string, number]> = {}; for (const [k, v] of [...orch.inventory].sort()) inventory[k] = v;
  const report = [
    `MURKA retail ${built.variant}: task ${task.type} -> ${outcome} in ${simTime.toFixed(1)} s simulated (${(wallMs / 1000).toFixed(1)} s wall)${ref ? `; Webots ${wallRef} s wall at RTF ${rtf} ≈ ${ref.toFixed(0)} s simulated (studio / Webots ${(simTime / ref).toFixed(2)}), MuJoCo ${built.info.mujocoTime} s` : ''}`,
    ...stages.map((st) => `  ${st.name.padEnd(6)} ${st.outcome.padEnd(8)} ${st.t0.toFixed(1)} -> ${st.t1.toFixed(1)} s (${(st.t1 - st.t0).toFixed(1)} s)`),
    ...(scan ? [`  scan: ${scan.correct}/${scan.visible} visible gondola slots correct${scan.webotsVisible ? ` (Webots / MuJoCo: ${scan.webotsVisible}/${scan.webotsVisible})` : ''}${scan.wrong.length ? `; wrong: ${scan.wrong.join(', ')}` : ''}`] : []),
    ...(depal ? [`  depal: ${depal.moved.join(', ') || 'nothing moved'}`] : []),
    ...(order ? [`  order: ${Object.entries(order.onPacking).map(([k, v]) => `${k}: ${v.join(', ') || '—'}`).join('; ')}`] : []),
    ...(docked ? [`  dock: ${(docked.distance * 100).toFixed(1)} cm, ${docked.yawDeg.toFixed(1)}°, battery ${sim.battery.pct.toFixed(1)} %${sim.battery.docked ? ' (charging)' : ''}`] : []),
    `  collisions: arm links ${armLinks}, hands ${hands}, loads ${loads}, base ${base} (monitor: ${sim.armGeom.left.source} hand model, tolerance 5 mm)`,
    ...(detail ? [`  detail: ${detail}`] : []),
  ];
  return { outcome, stages, inventory, simTime, collisions: coll, baseContacts: sim.baseContacts.filter((c) => c.t >= t0), checks, reference: { webotsWall: wallRef, webotsRtf: rtf, webotsSim: ref, mujoco: full ? built.info.mujocoTime : null, ratio: ref ? simTime / ref : null }, battery: { pct: sim.battery.pct, docked: sim.battery.docked }, detail, trace, progress: orch.progress, registry: reg, report, dsl, wallMs };
}
