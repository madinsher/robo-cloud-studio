/** Application context: station, renderer, simulation runtime, previews, undo/redo, file I/O and high-level commands. */
import { Station, Item, ItemType, Frame, Target, Tool, SceneObject, Folder, Camera } from './core/items/item';
import { Robot } from './core/items/robot';
import { Program, Instruction, PathItem, ProgramRunResult } from './core/items/program';
import { createRobotFromLibrary, ROBOT_LIBRARY } from './core/items/library';
import { ProgramSimulator } from './core/motion/simulator';
import { StationRuntime } from './core/motion/runtime';
import { planMoveJ, planMoveL, Trajectory } from './core/motion/trajectory';
import { AssetStore } from './scene/assets';
import { SceneRenderer } from './scene/renderer';
import { EventBus } from './core/events';
import { Mat4, multiply, invert, transl, identity, getPos } from './core/math/pose';
import { saveStation, loadStation, StationFile, referencedMeshIds } from './io/station-file';
import { robotFromURDF, parseURDF } from './io/urdf/urdf';
import { fetchOnlineRobot, fetchPackageMeshes, resolvePackageUri, OnlineRobotResult } from './io/library/online_library';
import { restoreUserPosts } from './posts/user_posts';
import { importProgram, detectLanguage } from './io/programs/import';
import { importTargets } from './io/robodk/targets';
import { importRdkBestEffort } from './io/robodk/rdk_container';
import { parseOBJ } from './io/mesh/obj';
import { parseDHText, robotFromDH } from './io/robodk/dh';
import { isRdkExport, postProcessRdkExport } from './io/robodk/rdk_import';
import { parseGcode, gcodeToMachiningCurves, gcodeStats } from './io/programs/gcode';
import { isCadFile, importCad } from './io/mesh/step';
import { stationToRoboDKScript } from './io/robodk/station_script';
import { compileForPost, getPost, listPosts, PostFile } from './posts/index';
import { MobileRobot, MapItem, ZoneItem } from './mobile/items';
import { FleetItem, FleetManager } from './fleet/fleet';
import { ProcessSimulator, Component } from './vc/component';
import { FieldItem, MissionItem, CropRow } from './agri/items';
import { updateMissionProgress } from './agri/missions';
import { workerImportRdk, workerCompile, workerAvailable, cancelWorkerJobs, ImportProgress } from './scene/worker_client';
import { toast } from './ui/dom';

export interface AppEvents extends Record<string, unknown> {
  stationChanged: { station: Station };
  activeProgram: { program: Program | null };
  activeRobot: { robot: Robot | null };
  simulation: { playing: boolean; time: number; duration: number; paused?: boolean };
  /** A preview / validation result for a program is available (cycle time, steps, problems). */
  preview: { program: Program };
  log: { text: string; level: 'info' | 'warn' | 'error' };
  undo: { canUndo: boolean; canRedo: boolean };
  /** Long operation progress (import / compile in the worker). done=true closes the indicator. */
  progress: { label: string; detail?: string; done?: boolean };
}

/** Run handle of the station runtime (I1). The optional fields are read when the runtime provides them. */
export interface RunInfo {
  id: string | number;
  program: Program;
  robot?: Item | null;
  status: 'queued' | 'running' | 'done' | 'error' | 'stopped';
  error?: { message: string; instructionIndex: number };
  done: Promise<unknown>;
  /** Index (in program.instructions()) of the instruction being executed, when the runtime reports it. */
  instructionIndex?: number;
}

/** Step timing of a preview compile, by instruction (main-thread compile or worker). */
export interface StepInfo { t0: number; t1: number; trajectory?: { duration: number } }

/** Programs above this size are not compiled on the UI thread automatically (worker / explicit Validate). */
export const PREVIEW_LIMIT = 400;
const ACTIVE = new Set(['queued', 'running']);

export class App {
  station: Station;
  readonly assets = new AssetStore();
  renderer!: SceneRenderer;
  /**
   * Preview / validation / cycle-time simulator. It never plays: playback runs on `runtime`.
   * `sim.playing` mirrors the runtime playback state for older callers.
   */
  sim: ProgramSimulator;
  /** Deterministic station runtime (shared clock, concurrent programs on different robots, signals). */
  runtime!: StationRuntime;
  /** Applied to every new runtime (script runners, signal bridges registered by other modules). */
  runtimeHooks: Array<(rt: StationRuntime, app: App) => void> = [];
  processSim: ProcessSimulator;
  fleets = new Map<string, FleetManager>();
  readonly events = new EventBus<AppEvents>();
  activeProgram: Program | null = null;
  activeRobot: Robot | null = null;
  /** Undo history per open station. */
  private history = new Map<Station, { undo: string[]; redo: string[]; last: string }>();
  simSpeed = 1;
  /** Global sim clock for fleet & process simulation (independent from program playback). */
  worldRunning = false;
  worldTime = 0;
  /** Extra per-step callbacks run inside the world loop (control runtimes, plugins). */
  worldHooks: Array<(dt: number) => void> = [];
  logs: Array<{ t: number; text: string; level: 'info' | 'warn' | 'error' }> = [];
  /** Open stations (multi-station tabs); `station` is the active one. */
  stations: Station[] = [];
  /** External listeners for API event queues. */
  onStationEvent: ((type: string, itemId?: string, data?: any) => void) | null = null;
  private stationOffs: Array<() => void> = [];

  // -- Playback state --------------------------------------------------------
  /** 'idle': nothing runs; 'playing': the clock advances; 'paused': runs are frozen and resume on Play. */
  simState: 'idle' | 'playing' | 'paused' = 'idle';
  /** Simulation time since the last reset (s). */
  simTime = 0;
  /** Runs of the current session (since the last reset). */
  runs: RunInfo[] = [];
  /** Starts of the current session (time + options), replayed for deterministic seeking. */
  private session: Array<{ program: Program; at: number; opts: RunOptions }> = [];
  /** True while the station is in its home state (nothing ran since load / reset). */
  atHome = true;
  /** Restart from the home state when Play is pressed after the session finished (repeatable runs). */
  restartFromHome = true;
  private previewDeferred = false;
  private previewSteps = new Map<string, StepInfo>();
  private problemCache = new WeakMap<ProgramRunResult, Map<string, ProgramRunResult['problems'][number]>>();

  constructor() {
    try { const n = restoreUserPosts(); if (n) console.info(`Restored ${n} user post processors`); } catch { /* no storage */ }
    this.station = new Station('New station');
    this.sim = this.makePreviewSim(this.station);
    this.processSim = new ProcessSimulator(this.station);
    this.makeRuntime(this.station);
  }

  /** Collision checking during program validation. */
  checkCollisions = false;

  mount(host: HTMLElement): void {
    this.renderer = new SceneRenderer(host, this.station, this.assets);
    this.renderer.onSelect = (item, additive) => this.select(item, additive);
    this.renderer.onPoseEdit = (item, pose) => this.editPose(item, pose);
    this.renderer.onGizmoEnd = () => { this.snapshot(); this.previewProgram(); };
    this.renderer.onAnimate((dt) => this.tick(dt));
    this.snapshot();
  }

  /**
   * Gizmo drag. On a tool mounted on a robot the gizmo is the robot TCP: dragging jogs the robot through IK
   * (the tool's TCP definition is only edited in its dialog). Anything else is moved.
   */
  editPose(item: Item, pose: Mat4): void {
    if (item instanceof Tool && item.parent instanceof Robot) {
      const robot = item.parent;
      const res = robot.solveIK(multiply(invert(robot.poseAbs()), pose), { seed: robot.joints() }, item.poseTool());
      if (res.ok && robot.jointsValid(res.joints)) robot.setJoints(res.joints);
      return;
    }
    item.setPoseAbs(pose);
    if (item instanceof MobileRobot) item.syncStateFromPose();
  }

  // -- Logging -------------------------------------------------------------
  log(text: string, level: 'info' | 'warn' | 'error' = 'info'): void {
    this.logs.push({ t: Date.now(), text, level });
    if (this.logs.length > 1000) this.logs.shift();
    this.events.emit('log', { text, level });
    if (level !== 'info') toast(text, level === 'warn' ? 'warn' : 'error');
  }

  // -- Station lifecycle ---------------------------------------------------
  /** Open a new empty station tab and make it active. */
  addStation(name = 'New station'): Station {
    const st = new Station(name);
    this.stations.push(st);
    this.setStation(st);
    return st;
  }

  /** Close the active station (keeps at least one open). */
  closeStation(): void {
    const i = this.stations.indexOf(this.station);
    if (i >= 0) this.stations.splice(i, 1);
    this.history.delete(this.station);
    if (!this.stations.length) this.stations.push(new Station('New station'));
    this.setStation(this.stations[Math.min(Math.max(0, i), this.stations.length - 1)]);
  }

  private makePreviewSim(st: Station): ProgramSimulator {
    const sim = new ProgramSimulator(st);
    // the preview simulator never plays; older callers read `app.sim.playing` as "a program is playing"
    Object.defineProperty(sim, 'playing', { get: () => this.simState === 'playing', set: () => { /* preview never plays */ }, configurable: true });
    return sim;
  }

  private makeRuntime(st: Station): void {
    try { this.runtime?.stop(); } catch { /* old runtime already gone */ }
    if (this.workerJob && workerAvailable()) { this.workerJob++; cancelWorkerJobs('compile', 'station closed'); }
    // the station's shared runtime (StationRuntime.of): the API, control models and the UI drive the same clock
    this.runtime = StationRuntime.of(st);
    this.runtime.stop();
    this.runtime.assets = this.assets;
    this.runtime.sink.message ??= (text, level) => this.log(text, level);
    for (const hook of this.runtimeHooks) { try { hook(this.runtime, this); } catch (e) { console.error(e); } }
    // a station that already ran (tab switched back, API runs) keeps its home state: Reset / the next Play restore it
    this.simState = 'idle'; this.simTime = 0; this.runs = []; this.session = []; this.atHome = !this.runtime.hasHome;
  }

  /** Bind the per-station subscriptions (released on switch, so tab round trips do not accumulate listeners). */
  private bindStation(st: Station): void {
    for (const off of this.stationOffs) off();
    this.stationOffs = [
      st.events.on('selection', ({ items }) => this.onStationEvent?.('selection', items[0]?.id)),
      st.events.on('changed', ({ item, what }) => { if (what === 'pose') this.onStationEvent?.('moved', item.id); else if (what === 'joints') this.onStationEvent?.('robotMoved', item.id); else if (what === 'name') this.onStationEvent?.('renamed', item.id); else if (what === 'visible') this.onStationEvent?.('visibility', item.id); }),
    ];
  }

  setStation(st: Station): void {
    if (!this.stations.includes(st)) {
      const i = this.stations.indexOf(this.station);
      if (i >= 0) { this.history.delete(this.stations[i]); this.stations[i] = st; } else this.stations.push(st);
    }
    this.worldHooks.length = 0;
    this.worldRunning = false;
    this.worldTime = 0;
    this.station = st;
    this.bindStation(st);
    this.onStationEvent?.('station', st.id);
    this.sim = this.makePreviewSim(st);
    this.makeRuntime(st);
    this.processSim = new ProcessSimulator(st);
    this.fleets.clear();
    this.previewSteps.clear();
    this.activeProgram = st.itemsOfType<Program>(ItemType.PROGRAM)[0] ?? null;
    this.activeRobot = st.itemsOfType<Robot>(ItemType.ROBOT)[0] ?? null;
    this.renderer?.bindStation(st);
    if (!this.history.has(st)) this.snapshot(); else this.emitUndo();
    this.events.emit('stationChanged', { station: st });
    this.events.emit('activeProgram', { program: this.activeProgram });
    this.events.emit('activeRobot', { robot: this.activeRobot });
    this.renderer?.showTrajectoryPreview(null, []);
    this.renderer?.setCollisionHighlight([]);
    this.emitSim();
    setTimeout(() => this.renderer?.fitAll(), 50);
  }

  newStation(): void {
    this.setStation(new Station('New station'));
  }

  // -- Undo/redo (snapshot based, per station) ---------------------------------
  private hist() {
    let h = this.history.get(this.station);
    if (!h) { h = { undo: [], redo: [], last: '' }; this.history.set(this.station, h); }
    return h;
  }
  private emitUndo() { const h = this.hist(); this.events.emit('undo', { canUndo: h.undo.length > 0, canRedo: h.redo.length > 0 }); }
  snapshot(): void {
    const h = this.hist();
    const s = JSON.stringify(this.station.serialize());
    if (s === h.last) return;
    if (h.last) h.undo.push(h.last);
    // whole-station snapshots: keep fewer of them for big stations (memory bound ~100 MB)
    const max = s.length > 2e6 ? Math.max(3, Math.floor(1e8 / s.length)) : 50;
    while (h.undo.length > max) h.undo.shift();
    h.redo = [];
    h.last = s;
    this.emitUndo();
  }
  /** Run a mutating command and record one undo step. */
  cmd<T>(fn: () => T): T {
    const r = fn();
    this.snapshot();
    return r;
  }
  get canUndo(): boolean { return this.hist().undo.length > 0; }
  undo(): void {
    const h = this.hist();
    const s = h.undo.pop();
    if (!s) return;
    h.redo.push(h.last);
    this.restore(s, h);
  }
  redo(): void {
    const h = this.hist();
    const s = h.redo.pop();
    if (!s) return;
    h.undo.push(h.last);
    this.restore(s, h);
  }
  private restore(s: string, h: { undo: string[]; redo: string[]; last: string }) {
    h.last = s;
    const st = loadStation(JSON.parse(s), this.assets);
    const progId = this.activeProgram?.id, robId = this.activeRobot?.id, selIds = this.station.selection.map((x) => x.id);
    const i = this.stations.indexOf(this.station);
    if (i >= 0) this.stations[i] = st; else this.stations.push(st);
    this.history.delete(this.station);
    this.history.set(st, h);
    this.worldHooks.length = 0;
    this.worldRunning = false;
    this.worldTime = 0;
    this.station = st;
    this.bindStation(st);
    this.sim = this.makePreviewSim(st);
    this.makeRuntime(st);
    this.processSim = new ProcessSimulator(st);
    this.fleets.clear();
    this.previewSteps.clear();
    this.renderer?.bindStation(st);
    this.activeProgram = (progId && (st.findById(progId) as Program)) || st.itemsOfType<Program>(ItemType.PROGRAM)[0] || null;
    this.activeRobot = (robId && (st.findById(robId) as Robot)) || st.itemsOfType<Robot>(ItemType.ROBOT)[0] || null;
    this.events.emit('stationChanged', { station: st });
    this.events.emit('activeProgram', { program: this.activeProgram });
    this.events.emit('activeRobot', { robot: this.activeRobot });
    this.emitUndo();
    const sel = selIds.map((id) => st.findById(id)).filter((x): x is Item => !!x);
    if (sel.length) st.setSelection(sel);
    this.renderer?.showTrajectoryPreview(null, []);
    this.renderer?.setCollisionHighlight([]);
    this.emitSim();
  }

  // -- Selection -----------------------------------------------------------
  select(item: Item | null, additive = false): void {
    this.station.select(item, additive);
    const sel = this.station.selection[0] ?? null;
    if (sel instanceof Program) this.setActiveProgram(sel);
    if (sel instanceof Instruction && sel.parent instanceof Program) this.setActiveProgram(sel.parent);
    if (sel instanceof Robot) this.setActiveRobot(sel);
    if (sel instanceof Tool && sel.parent instanceof Robot) this.setActiveRobot(sel.parent);
    this.renderer?.setGizmo(sel && !(sel instanceof Program) && !(sel instanceof Instruction) && !(sel instanceof CropRow) ? sel : null);
  }
  setActiveProgram(p: Program | null): void {
    if (this.activeProgram === p) return;
    this.activeProgram = p;
    this.events.emit('activeProgram', { program: p });
    this.previewProgram();
  }
  setActiveRobot(r: Robot | null): void {
    if (this.activeRobot === r) return;
    this.activeRobot = r;
    this.events.emit('activeRobot', { robot: r });
  }

  // -- Item creation --------------------------------------------------------
  /**
   * Download a robot from the online library (ROS-Industrial / vendor URDF packages on GitHub) and
   * add it to the station with exact kinematics and meshes. Resolves once meshes are in.
   */
  async addOnlineRobot(id: string, parent: Item = this.station, opts: { meshes?: boolean; onProgress?: (msg: string, done?: number, total?: number) => void } = {}): Promise<OnlineRobotResult> {
    const res = await fetchOnlineRobot(id, { assets: this.assets, meshes: opts.meshes ?? true, onProgress: opts.onProgress });
    const r = res.robot;
    this.cmd(() => {
      const n = this.station.itemsOfType(ItemType.ROBOT).length;
      if (n) r.setPose(transl(n * 1500, 0, 0));
      parent.addChild(r);
      const tool = new Tool('Tool 1');
      tool.setPoseTool(transl(0, 0, 100));
      r.addChild(tool);
      r.setTool(tool);
      this.setActiveRobot(r);
      this.select(r);
    });
    for (const w of res.warnings.slice(0, 5)) this.log(w, 'warn');
    this.log(`Online library: ${r.name} (${r.dof} DOF, ${res.meshes.loaded.length} meshes${res.meshes.failed.length ? `, ${res.meshes.failed.length} failed` : ''}) from ${res.url}`);
    return res;
  }

  addRobotFromLibrary(id: string, parent: Item = this.station): Robot {
    return this.cmd(() => {
      const r = createRobotFromLibrary(id);
      const n = this.station.itemsOfType(ItemType.ROBOT).length;
      if (n) r.setPose(transl(n * 1500, 0, 0));
      parent.addChild(r);
      const tool = new Tool('Tool 1');
      tool.setPoseTool(transl(0, 0, 100));
      r.addChild(tool);
      r.setTool(tool);
      this.setActiveRobot(r);
      this.select(r);
      return r;
    });
  }
  addFrame(parent: Item = this.station, name?: string): Frame {
    return this.cmd(() => { const f = parent.addChild(new Frame(name ?? `Frame ${this.station.itemsOfType(ItemType.FRAME).length + 1}`)); this.select(f); return f; });
  }
  /** Create a target at the active robot's TCP (no undo step: callers wrap it). */
  private makeTarget(parent?: Item, name?: string): Target {
    const robot = this.activeRobot;
    const frame = parent ?? robot?.activeFrame() ?? this.station;
    const t = new Target(name ?? `Target ${this.station.itemsOfType(ItemType.TARGET).length + 1}`);
    if (robot) {
      const tcpAbs = robot.poseTCPAbs();
      frame.addChild(t);
      t.setPoseAbs(tcpAbs);
      t.setJoints(robot.joints());
      t.robotId = robot.id;
    } else frame.addChild(t);
    return t;
  }
  addTarget(parent?: Item, name?: string): Target | null {
    return this.cmd(() => { const t = this.makeTarget(parent, name); this.select(t); return t; });
  }
  private makeProgram(robot: Robot | MobileRobot | null, name?: string): Program {
    const p = this.station.addChild(new Program(name ?? `Prog ${this.station.itemsOfType(ItemType.PROGRAM).length + 1}`));
    if (robot) p.setRobot(robot);
    return p;
  }
  addProgram(robot: Robot | MobileRobot | null = this.activeRobot, name?: string): Program {
    return this.cmd(() => {
      const p = this.makeProgram(robot, name);
      this.setActiveProgram(p);
      this.select(p);
      return p;
    });
  }
  addObjectPrimitive(kind: 'box' | 'cylinder' | 'sphere', size: number[] = [500, 500, 500], parent: Item = this.station): SceneObject {
    return this.cmd(() => {
      const o = new SceneObject(kind[0].toUpperCase() + kind.slice(1));
      o.geometry = [kind === 'box' ? { primitive: { kind: 'box', size: [size[0], size[1], size[2]] }, origin: Array.from(transl(0, 0, size[2] / 2)), color: '#8ca0b3' } : kind === 'cylinder' ? { primitive: { kind: 'cylinder', radius: size[0], length: size[2] }, origin: Array.from(transl(0, 0, size[2] / 2)), color: '#8ca0b3' } : { primitive: { kind: 'sphere', radius: size[0] }, origin: Array.from(transl(0, 0, size[0])), color: '#8ca0b3' }];
      parent.addChild(o);
      this.select(o);
      return o;
    });
  }
  addMobileRobot(preset: 'amr' | 'tractor' | 'harvester' | 'sprayer' | 'scout' = 'amr'): MobileRobot {
    return this.cmd(() => {
      const m = new MobileRobot(preset === 'amr' ? 'AMR' : preset === 'tractor' ? 'Tractor' : preset === 'harvester' ? 'Harvest platform' : preset === 'sprayer' ? 'Sprayer' : 'Scout rover');
      if (preset === 'tractor') Object.assign(m.kin, { drive: 'ackermann', wheelBase: 2200, track: 1600, wheelRadius: 600, maxSpeed: 3000, maxYawRate: 40, footprint: [3800, 1900, 1800], minTurnRadius: 3500 });
      if (preset === 'harvester') { Object.assign(m.kin, { drive: 'tracked', wheelBase: 1800, track: 1400, wheelRadius: 300, maxSpeed: 1200, footprint: [2600, 1600, 1400] }); m.capabilities = ['harvest', 'transport']; m.color = '#c92a2a'; }
      if (preset === 'sprayer') { Object.assign(m.kin, { drive: 'ackermann', wheelBase: 1800, track: 1300, wheelRadius: 400, maxSpeed: 2500, footprint: [3000, 1500, 1500], minTurnRadius: 3000 }); m.capabilities = ['spray', 'transport']; m.color = '#1971c2'; }
      if (preset === 'scout') { Object.assign(m.kin, { drive: 'differential', wheelBase: 500, track: 450, wheelRadius: 150, maxSpeed: 2000, footprint: [700, 500, 400] }); m.capabilities = ['scout', 'pollinate']; m.color = '#f08c00'; m.battery.capacityWh = m.battery.levelWh = 600; }
      const n = this.station.itemsOfType(ItemType.MOBILE_ROBOT).length;
      m.setPose2D(n * 2500, -3000, 0);
      this.station.addChild(m);
      this.select(m);
      return m;
    });
  }

  deleteItems(items: Item[]): void {
    this.cmd(() => {
      for (const it of items) if (it !== this.station) it.delete();
      if (items.includes(this.activeProgram as Item)) this.setActiveProgram(null);
      if (items.includes(this.activeRobot as Item)) this.setActiveRobot(null);
      this.select(null);
    });
  }

  // -- Program helpers ------------------------------------------------------
  /** Teach the current robot position as a new target and append a move to the active program: one undo step. */
  teach(moveType: 'MoveJ' | 'MoveL' = 'MoveJ'): void {
    const robot = this.activeRobot;
    if (!robot) return this.log('Select a robot first', 'warn');
    this.cmd(() => {
      let prog = this.activeProgram;
      if (!prog) { prog = this.makeProgram(robot); this.activeProgram = prog; this.events.emit('activeProgram', { program: prog }); }
      const t = this.makeTarget();
      if (moveType === 'MoveJ') { prog.addMoveJ(t); t.setAsJointTarget(); } else prog.addMoveL(t);
      this.select(t);
    });
    this.previewProgram();
  }

  /** Problems of the program's last compile, by instruction id. */
  problemsOf(p: Program): Map<string, ProgramRunResult['problems'][number]> {
    const r = p.lastResult;
    if (!r) return new Map();
    let m = this.problemCache.get(r);
    if (!m) { m = new Map(); for (const pr of r.problems) if (!m.has(pr.instructionId)) m.set(pr.instructionId, pr); this.problemCache.set(r, m); }
    return m;
  }
  /** Timing of an instruction in the last preview compile of its program. */
  stepOf(ins: Instruction): StepInfo | undefined { return this.previewSteps.get(ins.id); }

  /**
   * Compile the active program for its preview (trajectory line, cycle time, per-instruction times). Never touches
   * the playing runtime: deferred while a simulation plays; programs above PREVIEW_LIMIT go to the worker.
   */
  previewProgram(): void {
    const p = this.activeProgram;
    if (!p || !(p.robot() instanceof Robot)) {
      this.renderer?.showTrajectoryPreview(null, []);
      this.renderer?.setCollisionHighlight([]);
      return;
    }
    if (this.simState === 'playing') { this.previewDeferred = true; return; }
    if (p.instructions().length > PREVIEW_LIMIT) { void this.compileInWorker(p, false); return; }
    this.compilePreview(p, false);
  }

  /** Explicit validation: compile with self-collision checks (station collisions when enabled). */
  validateProgram(p: Program | null = this.activeProgram): ProgramRunResult | null {
    if (!p) return null;
    if (p.instructions().length > PREVIEW_LIMIT && workerAvailable()) { void this.compileInWorker(p, true); return p.lastResult; }
    return this.compilePreview(p, true);
  }

  private compilePreview(p: Program, collisions: boolean): ProgramRunResult {
    const robot = p.robot() as Robot | null;
    const saved = this.station.itemsOfType<Robot>(ItemType.ROBOT).map((r) => [r, r.joints()] as const);
    this.sim.collisionOptions = { enabled: collisions, selfOnly: !this.checkCollisions, assets: this.assets, sampleStep: 0.05 };
    const res = this.sim.compile(p);
    for (const [r, q] of saved) if (r.joints().some((v, i) => v !== q[i])) r.setJoints(q);
    this.previewSteps.clear();
    for (const s of this.sim.steps) this.previewSteps.set(s.instruction.id, { t0: s.t0, t1: s.t1, trajectory: s.trajectory ? { duration: s.trajectory.duration } : undefined });
    this.renderer?.setCollisionHighlight(this.sim.collisions.flatMap((c) => c.pairs.flatMap((pr) => [pr.a.item.id, pr.b.item.id])));
    if (robot instanceof Robot) this.renderer?.showTrajectoryPreview(robot, this.sim.steps.filter((s) => s.trajectory).map((s) => s.trajectory as Trajectory));
    this.events.emit('preview', { program: p });
    this.emitSim();
    return res;
  }

  private workerJob = 0;
  /** Compile a large program in the worker (UI stays responsive), then show its preview. */
  private async compileInWorker(p: Program, collisions: boolean): Promise<void> {
    if (!workerAvailable()) {
      this.renderer?.showTrajectoryPreview(null, []);
      this.log(`${p.name}: ${p.instructions().length} instructions — preview skipped on the UI thread (use Validate)`, 'info');
      return;
    }
    const job = ++this.workerJob;
    this.events.emit('progress', { label: `Compiling ${p.name}`, detail: `${p.instructions().length} instructions` });
    try {
      const file = collisions ? saveStation(this.station, this.assets) : { format: 'vbstation', version: 1, savedAt: '', app: '', station: this.station.serialize(), assets: {} } as StationFile;
      const out = await workerCompile(file, p.id, { collisions, selfOnly: !this.checkCollisions });
      if (job !== this.workerJob || !p.station) return;
      p.lastResult = out.result;
      this.previewSteps.clear();
      for (const [id, t0, t1, d] of out.steps) this.previewSteps.set(id, { t0, t1, trajectory: d >= 0 ? { duration: d } : undefined });
      this.renderer?.showPathPreview?.(out.points);
      for (const pr of out.result.problems.slice(0, 5)) if (collisions) this.log(`${p.name}: ${pr.message}`, pr.severity === 'error' ? 'error' : 'warn');
      this.events.emit('preview', { program: p });
      this.emitSim();
    } catch (e) {
      if (job === this.workerJob) this.log(`Compile of ${p.name} failed: ${(e as Error).message}`, 'error');
    } finally {
      if (job === this.workerJob) this.events.emit('progress', { label: `Compiling ${p.name}`, done: true });
    }
  }

  // -- Simulation (StationRuntime) ---------------------------------------------
  isPlaying(): boolean { return this.simState === 'playing'; }
  /** Runs that are queued or running. */
  activeRuns(): RunInfo[] { return this.runs.filter((r) => ACTIVE.has(r.status)); }

  /** Estimated end of the session (s): program start time + its cycle-time estimate. */
  get simDuration(): number {
    let d = 0;
    for (const s of this.session) d = Math.max(d, s.at + (s.program.lastResult?.duration ?? 0));
    if (!this.session.length && this.activeProgram?.lastResult) d = this.activeProgram.lastResult.duration;
    return Math.max(d, this.simTime);
  }

  private emitSim() {
    this.events.emit('simulation', { playing: this.simState === 'playing', paused: this.simState === 'paused', time: this.simTime, duration: this.simDuration });
  }

  /**
   * Play a program on the station runtime. Programs on other robots keep running (concurrent, shared clock);
   * a program on a busy robot is queued by the runtime. When paused, Play resumes. After a finished session
   * the station is restored to its home state first (repeatable runs); `restartFromHome=false` continues
   * from the current state like RoboDK.
   */
  runProgram(p: Program | null = this.activeProgram, opts: RunOptions = {}): RunInfo | null {
    if (!p) { this.log('No program selected', 'warn'); return null; }
    if (this.simState === 'paused') this.resumeProgram();
    const running = this.activeRuns().find((r) => r.program === p);
    if (running) {
      if (opts.fromIndex === undefined) return running;
      this.runtime.stop(running.id as any);
    }
    if (!this.activeRuns().length) {
      if (!this.atHome && this.restartFromHome) this.resetSimulation();
      if (this.atHome) { this.runtime.captureHome(); this.atHome = false; }
    }
    if (!p.lastResult && p.instructions().length <= PREVIEW_LIMIT && p.robot() instanceof Robot) this.compilePreview(p, false);
    const start: Record<string, unknown> = {};
    const robot = p.robot();
    if (robot) start.robot = robot;
    if (opts.args) start.args = opts.args;
    if (opts.fromIndex !== undefined && opts.fromIndex > 0) {
      start.fromIndex = opts.fromIndex;
      const q = this.jointsBefore(p, opts.fromIndex);
      if (q && robot instanceof Robot) { start.startJoints = q; robot.setJoints(q); }
    }
    const h = this.runtime.start(p, start as any) as unknown as RunInfo;
    if (opts.fromIndex) this.startAt(h, opts.fromIndex);
    this.runs.push(h);
    this.session.push({ program: p, at: this.simTime, opts });
    this.simState = 'playing';
    void h.done.then(() => this.onRunFinished(h), (e) => this.log(`${p.name}: ${(e as Error)?.message ?? e}`, 'error'));
    this.emitSim();
    return h;
  }

  /**
   * "Run from here": a run that has not executed anything yet starts at instruction `index`. Uses the runtime's
   * StartOptions.fromIndex when it honours it (requested from T3); otherwise the new run's top frame is positioned
   * before its first step (start() only queues / begins it, instructions execute on the next tick).
   */
  private startAt(h: RunInfo, index: number) {
    if (typeof h.instructionIndex === 'number' && h.instructionIndex >= index) return;
    const rt = this.runtime as unknown as { ctxs?: Array<{ handle: unknown; frames: Array<{ pc: number; list: unknown[] }> }>; queue?: Array<{ handle: unknown; frames: Array<{ pc: number; list: unknown[] }> }> };
    const c = [...(rt.ctxs ?? []), ...(rt.queue ?? [])].find((x) => x.handle === h);
    const fr = c?.frames?.[0];
    if (fr && fr.pc === 0 && index < fr.list.length) { fr.pc = index; h.instructionIndex = index; }
  }

  /** Joints of the program's robot at the start of instruction `index` (from the preview compile). */
  private jointsBefore(p: Program, index: number): number[] | null {
    const ins = p.instructions();
    for (let i = Math.min(index, ins.length) - 1; i >= 0; i--) {
      const d = ins[i].data;
      if (d.kind !== 'move') continue;
      const t = d.targetId ? this.station.findById(d.targetId) as Target | null : null;
      if (d.joints) return [...d.joints];
      if (t?.joints && p.robot() instanceof Robot) { const r = p.robot() as Robot; const q = r.jointsForTarget(t); if (q) return q; }
    }
    return null;
  }

  private onRunFinished(h: RunInfo) {
    if (h.status === 'error' && h.error) {
      this.log(`${h.program.name}: stopped at instruction ${h.error.instructionIndex + 1}: ${h.error.message}`, 'error');
    }
    if (!this.activeRuns().length && this.simState === 'playing') { this.simState = 'idle'; this.flushDeferredPreview(); }
    this.emitSim();
  }

  private flushDeferredPreview() {
    if (this.previewDeferred && this.simState !== 'playing') { this.previewDeferred = false; this.previewProgram(); }
  }

  /** Index of the instruction a live run of `p` is executing (-1 when not running). */
  runningInstruction(p: Program): number {
    for (let i = this.runs.length - 1; i >= 0; i--) {
      const r = this.runs[i];
      if (r.program === p && ACTIVE.has(r.status)) return typeof r.instructionIndex === 'number' ? r.instructionIndex : -1;
    }
    return -1;
  }
  /** Instruction index where the last run of `p` stopped with an error (-1 if none). */
  errorInstruction(p: Program): number {
    for (let i = this.runs.length - 1; i >= 0; i--) {
      const r = this.runs[i];
      if (r.program === p) return r.status === 'error' && r.error ? r.error.instructionIndex : -1;
    }
    return -1;
  }

  pauseProgram(): void {
    if (this.simState === 'playing') this.simState = 'paused';
    this.emitSim();
  }
  resumeProgram(): void {
    if (this.simState === 'paused') this.simState = this.activeRuns().length ? 'playing' : 'idle';
    this.emitSim();
  }
  /** Stop every run; the station stays where it is (Reset restores the home state). */
  stopProgram(): void {
    if (this.activeRuns().length) this.runtime.stop();
    this.simState = 'idle';
    this.flushDeferredPreview();
    this.emitSim();
  }
  /** Stop and restore the home state: object poses, attachments, robot joints, IO ("Replace objects"). */
  resetSimulation(): void {
    this.runtime.stop();
    if (!this.atHome) this.runtime.reset();
    this.runs = [];
    this.session = [];
    this.simTime = 0;
    this.simState = 'idle';
    this.atHome = true;
    this.flushDeferredPreview();
    this.emitSim();
  }
  /** Make the current station state the home state restored by Reset. */
  setHome(): void {
    this.runtime.captureHome();
    this.atHome = false;
    this.runs = this.activeRuns();
    this.session = this.session.filter((s) => this.runs.some((r) => r.program === s.program));
    this.simTime = 0;
    this.emitSim();
  }

  /**
   * Seek the session to time t, deterministically: forward runs the clock, backward restores the home state and
   * replays the session's starts. With no session yet, the active program is started from home and run to t.
   */
  seekProgram(t: number): void {
    t = Math.max(0, t);
    if (!this.session.length) {
      if (!this.activeProgram || t <= 0) return;
      const state = this.simState;
      this.runProgram(this.activeProgram);
      this.advance(t);
      if (state !== 'playing') this.simState = this.activeRuns().length ? 'paused' : 'idle';
      this.emitSim();
      return;
    }
    if (t < this.simTime) {
      const replay = [...this.session];
      const state = this.simState;
      this.resetSimulation();
      this.atHome = false;
      for (const s of replay) {
        if (s.at > t) break;
        this.advance(s.at - this.simTime);
        const from = s.opts.fromIndex ?? 0, rb = s.program.robot();
        const q = from > 0 ? this.jointsBefore(s.program, from) : null;
        const h = this.runtime.start(s.program, { ...(rb ? { robot: rb } : {}), ...(s.opts.args ? { args: s.opts.args } : {}), ...(q && rb instanceof Robot ? { startJoints: q } : {}), ...(from > 0 ? { fromIndex: from } : {}) } as any) as unknown as RunInfo;
        if (from > 0) this.startAt(h, from);
        this.runs.push(h);
        this.session.push({ program: s.program, at: s.at, opts: s.opts });
        void h.done.then(() => this.onRunFinished(h));
      }
      this.advance(t - this.simTime);
      this.simState = this.activeRuns().length ? (state === 'playing' ? 'playing' : 'paused') : 'idle';
    } else this.advance(t - this.simTime);
    this.emitSim();
  }

  /**
   * Advance the shared clock by dt simulated seconds (no real-time pacing). Instructions are event driven inside the
   * runtime; the step only sets how often robots are sampled on the way, so long seeks use coarser steps.
   */
  private advance(dt: number) {
    if (dt <= 0) return;
    this.runtime.runFor(dt, Math.min(1, Math.max(0.02, dt / 500)));
    this.simTime += dt;
  }

  /** Run every active program to its end (bounded by `maxSeconds` of simulated time). */
  runToEnd(maxSeconds = 36000): void {
    let left = maxSeconds;
    while (this.activeRuns().length && left > 0) {
      const step = Math.min(left, Math.max(1, this.simDuration - this.simTime + 0.5));
      this.advance(step);
      left -= step;
    }
    if (!this.activeRuns().length) this.simState = 'idle';
    this.emitSim();
  }

  /**
   * Run `fn` (an exporter sampling the preview simulator, which moves robots and objects) and put the station
   * back exactly as it was: object parents / poses / visibility, robot joints / frame / tool, gripper state.
   */
  async withStateRestored<T>(fn: () => T | Promise<T>): Promise<T> {
    const objs: Array<[SceneObject, Item | null, Mat4, boolean]> = [];
    const robots: Array<[Robot, number[], string | null, string | null]> = [];
    const tools: Array<[Tool, boolean, string[]]> = [];
    for (const it of this.station.walk()) {
      if (it instanceof SceneObject) objs.push([it, it.parent, it.pose(), it.visible]);
      if (it instanceof Tool) tools.push([it, it.closed, [...it.attached]]);
      if (it instanceof Robot) robots.push([it, it.joints(), it.activeFrameId, it.activeToolId]);
    }
    try { return await fn(); } finally {
      for (const [r, q, f, t] of robots) { r.setJoints(q); r.activeFrameId = f; r.activeToolId = t; }
      for (const [o, parent, pose, vis] of objs) { if (parent && o.parent !== parent) parent.addChild(o); o.setPose(pose); if (o.visible !== vis) o.setVisible(vis); }
      for (const [t, closed, att] of tools) { t.closed = closed; t.attached = att; }
    }
  }

  /** Wait (in real time, while the frame loop ticks) until no program is running. */
  async waitIdle(): Promise<void> {
    await Promise.all(this.activeRuns().map((r) => r.done.catch(() => undefined)));
  }

  /** Move the active robot to a target instantly (or preview MoveJ/MoveL path). */
  moveRobotTo(target: Target, linear = false): void {
    const robot = this.activeRobot;
    if (!robot) return;
    const q0 = robot.joints();
    const q1 = robot.jointsForTarget(target);
    if (!q1) return this.log(`${target.name} is unreachable`, 'warn');
    const tr = linear ? planMoveL(robot, q0, robot.solveFK(q1), robot.motion.speedLinear, robot.motion.accelLinear) : planMoveJ(robot, q0, q1, robot.motion.speedJoints, robot.motion.accelJoints);
    if (!tr.ok) this.log(tr.error ?? 'motion failed', 'warn');
    robot.setJoints(q1);
    this.renderer?.showTrajectoryPreview(robot, [tr]);
  }

  /** Check the current static station for collisions and report them. */
  checkStationCollisions(): number {
    const { checkCollisions } = require_collision();
    const pairs = checkCollisions(this.station, { assets: this.assets });
    this.renderer?.setCollisionHighlight(pairs.flatMap((p) => [p.a.item.id, p.b.item.id]));
    if (!pairs.length) this.log('No collisions in the current state');
    for (const p of pairs.slice(0, 10)) this.log(`Collision: ${p.a.item.name}${p.a.part !== p.a.item.name ? '/' + p.a.part : ''} × ${p.b.item.name}${p.b.part !== p.b.item.name ? '/' + p.b.part : ''} (${p.depth.toFixed(0)} mm)`, 'warn');
    return pairs.length;
  }

  // -- World simulation (fleet + process) --------------------------------------
  fleetManager(fleet: FleetItem): FleetManager {
    let fm = this.fleets.get(fleet.id);
    if (!fm || fm.fleet !== fleet) { fm = new FleetManager(this.station, fleet); this.fleets.set(fleet.id, fm); fm.events.on('log', (e) => this.log(`[${fleet.name}] ${e.text}`, e.level)); }
    return fm;
  }
  startWorld(): void { this.worldRunning = true; }
  pauseWorld(): void { this.worldRunning = false; }
  resetWorld(): void {
    this.worldRunning = false;
    this.worldTime = 0;
    this.worldHooks.length = 0;
    this.processSim.reset();
    for (const f of this.station.itemsOfType<FleetItem>(ItemType.FLEET)) { f.tasks = []; this.fleets.delete(f.id); }
    for (const m of this.station.itemsOfType<MobileRobot>(ItemType.MOBILE_ROBOT)) { m.state.path = null; m.state.taskId = null; m.state.status = 'idle'; m.state.v = 0; if (m.home) m.setPose2D(m.home.x, m.home.y, m.home.theta); }
    for (const mi of this.station.itemsOfType<MissionItem>(ItemType.MISSION)) { mi.taskIds = []; mi.progress = 0; mi.status = 'draft'; }
  }

  private tick(dt: number): void {
    if (this.simState === 'playing') {
      const step = dt * this.simSpeed;
      this.runtime.tick(step);
      this.simTime += step;
      if (!this.activeRuns().length) { this.simState = 'idle'; this.flushDeferredPreview(); }
      this.emitSim();
    }
    if (this.worldRunning) {
      const step = dt * this.simSpeed;
      // sub-step for stability at high speed factors
      const n = Math.max(1, Math.ceil(step / 0.05));
      const h = step / n;
      for (let i = 0; i < n; i++) {
        this.worldTime += h;
        for (const f of this.station.itemsOfType<FleetItem>(ItemType.FLEET)) this.fleetManager(f).step(h);
        if (this.station.itemsOfType(ItemType.COMPONENT).length) this.processSim.step(h);
        for (const hook of this.worldHooks) hook(h);
      }
      for (const mi of this.station.itemsOfType<MissionItem>(ItemType.MISSION)) {
        const f = mi.fleetId ? (this.station.findById(mi.fleetId) as FleetItem | null) : this.station.itemsOfType<FleetItem>(ItemType.FLEET)[0];
        if (f) updateMissionProgress(mi, this.fleetManager(f));
      }
    }
  }

  // -- Files ---------------------------------------------------------------
  saveToJSON(): string {
    return JSON.stringify(saveStation(this.station, this.assets));
  }

  /** Read every file of a drop / picker into a path -> bytes map (paths relative to the dropped folder). */
  static async filesToMap(files: File[]): Promise<Map<string, Uint8Array>> {
    const m = new Map<string, Uint8Array>();
    for (const f of files) m.set(((f as any).webkitRelativePath || (f as any).relativePath || f.name).replace(/\\/g, '/'), new Uint8Array(await f.arrayBuffer()));
    return m;
  }

  /**
   * Webots projects (.wbt / .proto, folders with worlds/ protos/) and ROS packages (package.xml, urdf/ meshes/)
   * are imported as a whole; returns the files left for the per-file importers.
   */
  private async openProjects(files: File[]): Promise<File[]> {
    const path = (f: File) => ((f as any).webkitRelativePath || (f as any).relativePath || f.name) as string;
    // a .zip holding a Webots project (.wbt) or a ROS package (package.xml) goes to that importer: entry names are read
    // without inflating anything
    const zipKind = new Map<File, 'webots' | 'ros'>();
    for (const f of files) {
      if (!/\.zip$/i.test(f.name)) continue;
      const names: string[] = [];
      try { const { unzipSync } = await import('fflate'); unzipSync(new Uint8Array(await f.arrayBuffer()), { filter: (e) => { names.push(e.name); return false; } }); } catch { continue; }
      if (names.some((n) => /\.wbt$/i.test(n))) zipKind.set(f, 'webots');
      else if (names.some((n) => /(^|\/)package\.xml$/i.test(n))) zipKind.set(f, 'ros');
    }
    const webots = files.some((f) => /\.(wbt|proto)$/i.test(f.name)) || [...zipKind.values()].includes('webots');
    const rosPkg = !webots && (files.some((f) => /(^|\/)package\.xml$/i.test(path(f))) || [...zipKind.values()].includes('ros'));
    if (!webots && !rosPkg) return files;
    const map = await App.filesToMap(files);
    if (webots) {
      // the Webots importer reads plain files: expand project zips in place (paths inside the zip kept)
      for (const [f, k] of zipKind) if (k === 'webots') {
        const { unzipSync } = await import('fflate');
        map.delete(path(f).replace(/\\/g, '/'));
        for (const [zk, zv] of Object.entries(unzipSync(new Uint8Array(await f.arrayBuffer())))) if (!zk.endsWith('/')) map.set(zk, zv);
      }
      this.events.emit('progress', { label: 'Importing Webots world', detail: `${map.size} files` });
      try {
        const { importWebotsFiles } = await import('./io/webots/index');
        const { station, report } = await importWebotsFiles(map, this.assets);
        this.setStation(station);
        for (const n of report) this.log(String(n), 'info');
        this.log(`Opened Webots world ${station.name}`);
      } finally { this.events.emit('progress', { label: 'Importing Webots world', done: true }); }
      return [];
    }
    this.events.emit('progress', { label: 'Importing ROS package', detail: `${map.size} files` });
    try {
      const { importRosPackageFiles } = await import('./io/urdf/package');
      const res = await importRosPackageFiles(map, this.assets);
      const items = res.items as Item[];
      if (items.length) this.cmd(() => { for (const it of items) this.station.addChild(it); const r = items.find((x) => x instanceof Robot) as Robot | undefined; if (r) { this.setActiveRobot(r); this.select(r); } else this.select(items[0]); });
      for (const n of res.report ?? []) this.log(String(n), 'info');
      for (const e of res.errors ?? []) this.log(`ROS package: ${e}`, 'error');
      this.log(`Imported ROS package: ${items.map((x) => x.name).join(', ') || 'no items'}`, items.length ? 'info' : 'warn');
    } finally { this.events.emit('progress', { label: 'Importing ROS package', done: true }); }
    return [];
  }

  /** Basenames (lower case) of mesh references in the station that no asset resolves yet. */
  private missingMeshNames(): Set<string> {
    const out = new Set<string>();
    for (const ref of referencedMeshIds(this.station.serialize())) if (!this.assets.resolve(ref)) out.add(ref.split(/[\\/]/).pop()!.toLowerCase());
    return out;
  }

  async openFiles(files: File[]): Promise<void> {
    files = await this.openProjects(files);
    if (!files.length) { this.previewProgram(); return; }
    // meshes needed by URDFs in this drop, or by items already in the station, are assets only (no stray objects)
    const consumed = this.missingMeshNames();
    const others = new Map(files.map((x) => [x.name, x]));
    const urdfCache = new Map<string, string>();
    for (const [n, x] of others) if (/\.(xacro|urdf)$/i.test(n)) urdfCache.set(n, await x.text());
    for (const [n, text] of urdfCache) {
      try { for (const m of parseURDF(text, { resolveInclude: (fn) => urdfCache.get(fn.split('/').pop()!) ?? null }).meshes) consumed.add(m.split(/[\\/]/).pop()!.toLowerCase()); } catch { /* reported when the URDF is opened */ void n; }
    }
    const isConsumed = (name: string) => consumed.has(name.toLowerCase());
    // Load meshes first so URDFs can resolve them
    const meshes = files.filter((f) => /\.(stl|obj|dae|glb|gltf)$/i.test(f.name));
    for (const f of meshes) {
      const buf = new Uint8Array(await f.arrayBuffer());
      const ext = f.name.split('.').pop()!.toLowerCase();
      if (ext === 'stl') this.assets.registerRaw(f.name, 'stl', buf, f.name);
      else if (ext === 'obj') this.assets.registerRaw(f.name, 'obj', buf, f.name);
      else if (ext === 'dae') this.assets.registerRaw(f.name, 'dae', buf, f.name);
      else if (ext === 'glb' || ext === 'gltf') {
        try {
          const { parseGLTF } = await import('./io/mesh/gltf');
          const mesh = await parseGLTF(ext === 'glb' ? buf.buffer.slice(buf.byteOffset, buf.byteOffset + buf.byteLength) as ArrayBuffer : new TextDecoder().decode(buf));
          const a = this.assets.registerMesh(f.name, mesh, f.name);
          a.type = 'glb';
          a.source = buf;
        } catch (e) { this.log(`glTF ${f.name}: ${(e as Error).message}`, 'error'); }
      }
    }
    for (const f of files) {
      const name = f.name;
      const ext = name.split('.').pop()?.toLowerCase() ?? '';
      try {
        if (ext === 'vbstation' || (ext === 'json' && /station/i.test(name))) {
          const data = JSON.parse(await f.text());
          const st = loadStation(data, this.assets);
          if (isRdkExport(st)) {
            const rep = postProcessRdkExport(st);
            for (const r of rep.robots) this.log(`RoboDK robot ${r.name}: ${r.source === 'dh' ? 'kinematics from DH table' : r.source === 'library' ? `matched library ${r.matched}` : 'no kinematics found (placeholder)'}`, r.source === 'none' ? 'warn' : 'info');
            for (const p of rep.programs) this.log(`RoboDK program ${p.name}: ${p.instructions} instructions`);
          }
          this.setStation(st);
          this.station.filePath = name;
          this.log(`Opened station ${name}`);
        } else if (ext === 'json') {
          const data = JSON.parse(await f.text());
          if (data.format === 'vbstation' || data.type === ItemType.STATION) { this.setStation(loadStation(data, this.assets)); this.log(`Opened station ${name}`); }
          else if (Array.isArray(data.dh)) { const robot = this.cmd(() => { const r = robotFromDH(data, name.replace(/\.json$/i, '')); this.station.addChild(r); this.select(r); this.setActiveRobot(r); return r; }); this.log(`Imported DH robot ${robot.name} (${robot.dof} DOF)`); }
          else if (data.type === 'FeatureCollection' || data.type === 'Feature') await this.importGeoJSON(data, name);
          else this.log(`Unknown JSON content in ${name}`, 'warn');
        } else if (ext === 'pcd' || ext === 'ply') {
          const { importPointCloud } = await import('./ui/vision_ui');
          importPointCloud(this, f.name, await f.arrayBuffer());
        } else if (ext === 'geojson') {
          await this.importGeoJSON(JSON.parse(await f.text()), name);
        } else if (ext === 'urdf' || ext === 'xacro') {
          const text = urdfCache.get(name) ?? await f.text();
          const robot = robotFromURDF(text, { resolveInclude: (fn) => urdfCache.get(fn.split('/').pop()!) ?? null });
          this.cmd(() => { this.station.addChild(robot); this.select(robot); this.setActiveRobot(robot); });
          const model = parseURDF(text, { resolveInclude: (fn) => urdfCache.get(fn.split('/').pop()!) ?? null });
          const missing = model.meshes.filter((m) => !this.assets.resolve(m));
          const online = missing.filter((m) => resolvePackageUri(m));
          this.log(`Imported URDF ${robot.name} (${robot.dof} DOF)${missing.length ? `; ${missing.length} meshes missing${online.length ? ` — fetching ${online.length} from the online library` : ' — drop the mesh files onto the viewport'}` : ''}`, missing.length && !online.length ? 'warn' : 'info');
          if (online.length) fetchPackageMeshes(online, this.assets).then((r) => { this.log(`Online meshes for ${robot.name}: ${r.loaded.length} loaded${r.failed.length ? `, ${r.failed.length} failed` : ''}`, r.failed.length ? 'warn' : 'info'); }).catch((e) => this.log(`Mesh download failed: ${(e as Error).message}`, 'warn'));
        } else if (ext === 'dh') {
          const text = await f.text();
          const robot = this.cmd(() => { const r = robotFromDH(parseDHText(text), name.replace(/\.dh$/i, '')); this.station.addChild(r); this.select(r); this.setActiveRobot(r); return r; });
          this.log(`Imported DH robot ${robot.name} (${robot.dof} DOF)`);
        } else if (ext === 'stl' || ext === 'obj' || ext === 'dae' || ((ext === 'glb' || ext === 'gltf') && this.assets.has(name))) {
          if (isConsumed(name)) { this.log(`Mesh ${name} resolved for the robot / items that reference it`); continue; }
          const a = this.assets.get(name);
          if (!a) continue;
          this.cmd(() => {
            const o = new SceneObject(name.replace(/\.(stl|obj|dae|glb|gltf)$/i, ''));
            o.geometry = [{ mesh: name, origin: Array.from(identity()), color: '#a5b1c2' }];
            if (a.mesh) o.bbox = { min: a.mesh.min, max: a.mesh.max };
            this.station.addChild(o);
            this.select(o);
          });
          this.log(`Imported mesh ${name}`);
        } else if ((ext === 'rdk' || ext === 'robot' || ext === 'tool') && (await this.convertWithRoboDK(f))) {
          // converted losslessly by a RoboDK instance behind the studio server
        } else if (ext === 'robot' || ext === 'tool' || ext === 'sld') {
          const { station, report } = await this.importRdk(await f.arrayBuffer(), name.replace(/\.(robot|tool|sld)$/i, ''));
          const level = report.fidelity === 'full' ? 'info' : 'warn';
          this.cmd(() => this.mergeImported(station, ext === 'tool'));
          for (const n of report.notes) this.log(n, level);
        } else if (ext === 'rdk') {
          const { station, report } = await this.importRdk(await f.arrayBuffer(), name.replace(/\.rdk$/i, ''));
          this.setStation(station);
          const robot = station.itemsOfType<Robot>(ItemType.ROBOT)[0];
          if (robot) this.setActiveRobot(robot);
          for (const n of report.notes) this.log(n, report.fidelity === 'full' ? 'info' : 'warn');
          if (report.fidelity !== 'full') this.log(`RoboDK .rdk scan: ${report.strings.slice(0, 12).join(', ')}${report.strings.length > 12 ? '…' : ''}`, 'warn');
        } else if (['src', 'mod', 'prg', 'ls', 'script', 'csv', 'txt'].includes(ext)) {
          const text = await f.text();
          const lang = detectLanguage(text, name);
          if (ext === 'csv' || ext === 'txt') {
            if (lang === 'csv' && /^type,name/m.test(text)) { const r = this.cmd(() => importProgram(text, { station: this.station, robot: this.activeRobot, filename: name })); this.setActiveProgram(r.program); this.log(`Imported program ${r.program.name}`); }
            else { const parent = this.activeRobot?.activeFrame() ?? this.station; const n = this.cmd(() => importTargets(text, { parent, namePrefix: name.replace(/\.[^.]+$/, '') })).length; this.log(`Imported ${n} targets from ${name}`); }
          } else {
            const r = this.cmd(() => importProgram(text, { station: this.station, robot: this.activeRobot, filename: name }));
            for (const w of r.warnings) this.log(w, 'warn');
            this.setActiveProgram(r.program);
            this.log(`Imported ${lang.toUpperCase()} program ${r.program.name} (${r.targets.length} targets)`);
          }
        } else if (isCadFile(name)) {
          this.log(`Loading OpenCascade (WebAssembly) for ${name}…`);
          const res = await importCad(await f.arrayBuffer(), name);
          this.cmd(() => {
            const root = new SceneObject(name.replace(/\.[^.]+$/, ''));
            this.station.addChild(root);
            res.meshes.forEach((m, i) => {
              const id = `${name}#${i}`;
              this.assets.registerMesh(id, m.mesh, m.name);
              root.geometry.push({ mesh: id, origin: Array.from(identity()), color: m.color ?? '#a5b1c2' });
            });
            const mn = res.meshes.reduce((a, m) => a.map((v, i) => Math.min(v, m.mesh.min[i])), [Infinity, Infinity, Infinity]);
            const mx = res.meshes.reduce((a, m) => a.map((v, i) => Math.max(v, m.mesh.max[i])), [-Infinity, -Infinity, -Infinity]);
            if (res.meshes.length) root.bbox = { min: mn as any, max: mx as any };
            this.select(root);
          });
          this.log(`Imported CAD ${name}: ${res.meshes.length} solids, ${res.meshes.reduce((a, m) => a + m.mesh.triangles, 0)} triangles`);
        } else if (['nc', 'gcode', 'ngc', 'tap', 'cnc', 'apt', 'gco'].includes(ext)) {
          const g = parseGcode(await f.text());
          const curves = gcodeToMachiningCurves(g);
          const st = gcodeStats(g);
          this.cmd(() => {
            const o = new SceneObject(name.replace(/\.[^.]+$/, ''));
            o.curves = curves;
            o.setParam('nc', { file: name, units: g.units, ...st });
            const frame = this.activeRobot?.activeFrame() ?? this.station;
            frame.addChild(o);
            this.select(o);
          });
          this.log(`Imported NC program ${name}: ${st.cuts} cuts (${(st.cutLength / 1000).toFixed(2)} m), ${st.rapids} rapids${st.extruding ? `, ${(st.extruding / 1000).toFixed(2)} m extruding (3D print)` : ''}${st.tools.length ? `, tools ${st.tools.join(',')}` : ''}, feeds ${st.feeds.slice(0, 4).join('/')} mm/min (${g.units}). Use Robot > Robot machining (NC) to program it.`);
        } else if (['zip', 'vcmx', 'vcm', 'vcax', 'rdkp'].includes(ext)) {
          // Visual Components / KUKA.Sim components and layouts, RoboDK packages, generic zips: best-effort container import
          const { importZipContainer } = await import('./io/containers/zip_import');
          const { folder, report } = await importZipContainer(await f.arrayBuffer(), name, this.assets);
          this.cmd(() => { this.station.addChild(folder); this.select(folder); });
          this.log(`${report.format === 'zip' ? 'Archive' : report.format === 'kuka-sim' ? 'KUKA.Sim' : 'Visual Components'} ${name}: ${report.meshes.length} meshes, ${report.metadata.length} metadata files, ${report.joints.length} joints${report.names.length ? `; names: ${report.names.slice(0, 6).join(', ')}` : ''}`, report.meshes.length ? 'info' : 'warn');
          for (const n of report.notes) this.log(n, 'warn');
        } else if (['png', 'jpg'].includes(ext)) {
          this.log(`${ext.toUpperCase()} files are not importable as geometry`, 'warn');
        } else this.log(`Unsupported file ${name}`, 'warn');
      } catch (e: any) {
        this.log(`Failed to open ${name}: ${e.message ?? e}`, 'error');
        console.error(e);
      }
    }
    this.previewProgram();
  }

  /**
   * Add the items of an imported .robot/.tool/.sld station. A tool is mounted on the active robot's flange
   * (RoboDK behaviour) and becomes its active tool.
   */
  private mergeImported(station: Station, isTool: boolean): void {
    const kids = [...station.children];
    const robot = this.activeRobot;
    if (isTool && robot) {
      if (!kids.some((c) => c instanceof Tool)) {
        // importer returned the tool body as a plain object: mount it as a tool (geometry in flange coordinates)
        const body = kids.find((c) => c instanceof SceneObject) as SceneObject | undefined;
        if (body) {
          const tool = new Tool(body.name);
          tool.geometry = body.geometry.map((g) => ({ ...g, origin: Array.from(multiply(body.pose(), (g.origin ?? Array.from(identity())) as unknown as Mat4)) }));
          tool.color = body.color;
          const tcp = body.params?.poseTool;
          if (Array.isArray(tcp) && tcp.length === 16) tool.setPoseTool(tcp as unknown as Mat4);
          kids.splice(kids.indexOf(body), 1, tool);
          if (!Array.isArray(tcp)) this.log(`${body.name}: no TCP in the file, TCP set to the flange`, 'warn');
        }
      }
      for (const c of kids) {
        if (c instanceof Tool) { robot.addChild(c); robot.setTool(c); }
        else robot.addChild(c);
      }
      const tool = kids.find((c) => c instanceof Tool);
      if (tool) { this.select(tool); this.log(`Tool ${tool.name} mounted on ${robot.name}`); }
      return;
    }
    for (const c of kids) this.station.addChild(c);
    const r = kids.find((c) => c instanceof Robot);
    if (r instanceof Robot) { this.select(r); this.setActiveRobot(r); }
    else if (kids[0]) this.select(kids[0]);
  }

  /** RoboDK binary import, in the worker when available (progress, no UI freeze), else on this thread. */
  private async importRdk(data: ArrayBuffer, name: string): Promise<{ station: Station; report: Awaited<ReturnType<typeof importRdkBestEffort>>['report'] }> {
    if (!workerAvailable()) return importRdkBestEffort(data, name, this.assets);
    const label = `Opening ${name}`;
    const onProgress = (p: ImportProgress) => this.events.emit('progress', { label, detail: p.detail });
    this.events.emit('progress', { label, detail: `${(data.byteLength / 1e6).toFixed(1)} MB` });
    try {
      const out = await workerImportRdk(data, name, onProgress);
      for (const m of out.meshes) {
        const a = this.assets.registerMesh(m.id, m.mesh, m.name, m.units);
        if (m.source) { a.source = m.source; a.type = m.type; }
      }
      onProgress({ detail: `building ${out.itemCount} items` });
      await new Promise((r) => setTimeout(r, 0));
      return { station: Station.deserialize(out.tree), report: out.report };
    } catch (e) {
      this.log(`Worker import failed (${(e as Error).message}); importing on the UI thread`, 'warn');
      return importRdkBestEffort(data, name, this.assets);
    } finally {
      this.events.emit('progress', { label, done: true });
    }
  }

  /** Ask the studio server to convert a RoboDK file with a local RoboDK installation; returns true when loaded. */
  private async convertWithRoboDK(f: File): Promise<boolean> {
    const base = this.serverHttpBase();
    if (!base) return false;
    try {
      const res = await fetch(`${base}/convert/rdk`, { method: 'POST', body: await f.arrayBuffer(), headers: { 'x-filename': encodeURIComponent(f.name) } });
      if (!res.ok) { const j = await res.json().catch(() => ({})); this.log(`RoboDK converter not available on the server (${j.error ?? res.status}); using best-effort import`, 'warn'); return false; }
      const data = await res.json();
      const st = loadStation(data, this.assets);
      if (isRdkExport(st)) postProcessRdkExport(st);
      if (f.name.toLowerCase().endsWith('.rdk')) this.setStation(st);
      else this.cmd(() => this.mergeImported(st, f.name.toLowerCase().endsWith('.tool')));
      this.log(`${f.name} converted with RoboDK (lossless)`);
      return true;
    } catch { return false; }
  }

  /** HTTP base of the studio server if the page was opened with ?server=ws://host:port. */
  serverHttpBase(): string | null {
    const p = new URLSearchParams(location.search).get('server');
    if (!p || p === 'off') return null;
    return p.replace(/^ws/, 'http').replace(/\/$/, '');
  }

  private async importGeoJSON(data: any, name: string) {
    const { fieldsFromGeoJSON } = await import('./agri/geo');
    const { fields, origin } = fieldsFromGeoJSON(data);
    this.cmd(() => {
      for (const f of fields) {
        const fi = new FieldItem(f.name);
        fi.polygon = f.polygon;
        fi.geoOrigin = origin;
        this.station.addChild(fi);
      }
    });
    this.log(`Imported ${fields.length} field(s) from ${name}`);
  }

  exportProgram(postId: string, program: Program | null = this.activeProgram): PostFile[] {
    if (!program) { this.log('No program to export', 'warn'); return []; }
    const post = getPost(postId);
    if (!post) { this.log(`Unknown post ${postId}`, 'error'); return []; }
    const compiled = compileForPost(this.station, program);
    for (const p of compiled.problems) this.log(p, 'warn');
    return post.generate(compiled);
  }

  exportRoboDKScript(): string {
    return stationToRoboDKScript(this.station);
  }

  posts() {
    return listPosts();
  }
  get library() {
    return ROBOT_LIBRARY;
  }
}

export interface RunOptions { fromIndex?: number; args?: string[] }

import * as collisionModule from './core/collision/collision';
function require_collision() { return collisionModule; }

export { Item, ItemType, Frame, Target, Tool, SceneObject, Folder, Camera, Robot, Program, Instruction, PathItem, MobileRobot, MapItem, ZoneItem, FleetItem, Component, FieldItem, MissionItem, CropRow, multiply, invert, getPos };
export type { Mat4, StationFile };
