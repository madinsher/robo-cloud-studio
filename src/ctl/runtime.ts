/**
 * Mission runtime: executes a behavior tree against a world (the simulated station or a stand-in), with
 *  - a Ramadge–Wonham supervisor gating the controllable events actions emit (`event=` / `done=` arguments),
 *  - LTL₃ monitors over the blackboard (verdict ⊥ → violation callback, the tree sees `violation`),
 *  - a mode automaton (hybrid) providing `mode` / `vmax` to the blackboard,
 *  - a recorded signal for STL robustness after the run.
 *
 * `stationBindings` drives real station items (mobile robots to zones, timers for grasp / place / dock);
 * `simulatedBindings` is a tiny stand-in world for tests and headless scenarios.
 */
import { BehaviorTree, BTBindings, BTStatus, TickContext, BTNode } from './bt';
import { BtDoc } from './dsl';
import { SupervisorRuntime, SupervisorTable, SupervisorViolation } from './des';
import { LTL3Monitor, Verdict } from './temporal';
import { ModeMachine, HybridSpec } from './hybrid';
import { Signal } from './vv';
import { Env, Value, expr } from './expr';
import type { Station, Item } from '../core/items/item';
import { ItemType } from '../core/items/item';
import { MobileRobot, ZoneItem } from '../mobile/items';
import { Robot } from '../core/items/robot';
import type { Program } from '../core/items/program';
import type { Target } from '../core/items/item';
import type { Trajectory } from '../core/motion/trajectory';
import { MapItem } from '../mobile/items';
import { followPath, stepMobile } from '../mobile/controller';
import { planPath } from '../mobile/planner';
import { ProgramBackend, ProgramRunHandle, planArmMove, robotForTarget, sampleAt, carryAttachedObjects } from './host';
import { invert } from '../core/math/pose';

export interface WorldBindings {
  /** Values merged into the blackboard every tick (battery, dist_human, estop, …). */
  sense(): Env;
  actions: Record<string, (ctx: TickContext, args: Record<string, unknown>, node: BTNode) => BTStatus | void>;
  /** Halt a running action; `node` identifies the action instance (so a halt stops only its own program / move). */
  halt?: Record<string, (ctx: TickContext, args: Record<string, unknown>, node?: BTNode) => void>;
  /** Uncontrollable events that happened since the last tick (fed to the supervisor). */
  events?(): string[];
  /** Advance the world by dt (simulated worlds only; the station is stepped by the app). */
  step?(dt: number): void;
  onViolation?(formula: string, env: Env): void;
}

export interface RuntimeOptions { bt: BtDoc; supervisor?: SupervisorTable; monitors?: string[]; modes?: HybridSpec; bindings: WorldBindings; log?: (msg: string) => void }

export class ControlRuntime {
  readonly tree: BehaviorTree;
  readonly supervisor: SupervisorRuntime | null;
  readonly monitors: LTL3Monitor[];
  readonly modes: ModeMachine | null;
  readonly bb: Record<string, unknown> = {};
  readonly trace: Signal = { t: [], values: [] };
  readonly log: string[] = [];
  denied = 0; violations = 0; supervisorErrors = 0; ticks = 0; time = 0;
  status: BTStatus = 'running';
  constructor(readonly opts: RuntimeOptions) {
    this.supervisor = opts.supervisor ? new SupervisorRuntime(opts.supervisor) : null;
    this.monitors = (opts.monitors ?? opts.bt.monitors ?? []).map((m) => new LTL3Monitor(m));
    this.modes = opts.modes ? new ModeMachine(opts.modes) : null;
    const b = opts.bindings;
    const actions: BTBindings['actions'] = {};
    for (const [name, fn] of Object.entries(b.actions)) {
      actions[name] = (ctx, args, node) => {
        const ev = typeof args.event === 'string' ? args.event : undefined;
        if (this.supervisor) this.drain();
        if (ev && this.supervisor && node.status !== 'running') {
          if (!this.supervisor.allowed(ev)) { this.denied++; this.say(`supervisor denied ${ev} (${name}) in plant state ${this.supervisor.plantState().join(',')}`); ctx.bb.lastDenied = ev; return 'failure'; }
          this.observe(ev);
        }
        const st = fn(ctx, args, node) ?? 'success';
        if (st !== 'running') Object.assign(ctx.bb, b.sense()); // post-conditions see the world after the action
        if (st !== 'running' && typeof args.done === 'string' && this.supervisor && st === 'success') this.observe(args.done);
        return st;
      };
    }
    this.tree = new BehaviorTree(opts.bt.root, { actions, halt: b.halt }, this.bb, (m) => this.say(m));
  }
  private say(msg: string): void { this.log.push(`[${this.time.toFixed(1)}] ${msg}`); this.opts.log?.(msg); }
  /** Feed the uncontrollable events the world reported so far (arrivals, sensor events) to the supervisor. */
  private drain(): void { for (const ev of this.opts.bindings.events?.() ?? []) this.observe(ev); }
  /** Never throws: an impossible event is a model mismatch (counted, logged), an unobservable one is ignored by the observer-based table. */
  private observe(ev: string): void { if (!this.supervisor) return; try { this.supervisor.observe(ev); } catch (e) { this.supervisorErrors++; this.say(`model mismatch: ${e instanceof SupervisorViolation ? e.message : (e as Error).message}`); } }

  tick(dt = 0.1): BTStatus {
    const b = this.opts.bindings;
    this.drain();
    Object.assign(this.bb, b.sense());
    this.bb.time = this.time;
    if (this.modes) { const env = envOf(this.bb); const m = this.modes.step(this.time, env); this.bb.mode = m; this.bb.vmax = this.modes.current().vMax ?? null; }
    if (this.supervisor) { this.bb.plantState = this.supervisor.plantState().join(','); this.bb.enabled = this.supervisor.enabled().join(' '); }
    this.status = this.tree.tick(dt);
    const env = envOf(this.bb);
    this.trace.t.push(this.time); this.trace.values.push(env);
    for (const m of this.monitors) {
      if (m.verdict !== '?') continue;
      const v: Verdict = m.step(env);
      if (v === '⊥') { this.violations++; this.bb.violation = m.formula; this.say(`MONITOR VIOLATED: ${m.formula}`); b.onViolation?.(m.formula, env); }
      else if (v === '⊤') this.say(`monitor satisfied: ${m.formula}`);
    }
    b.step?.(dt);
    this.time += dt; this.ticks++;
    return this.status;
  }
  run(seconds: number, dt = 0.1, stopOn?: (bb: Record<string, unknown>) => boolean): BTStatus { const n = Math.ceil(seconds / dt); for (let i = 0; i < n; i++) { this.tick(dt); if (stopOn?.(this.bb)) break; } return this.status; }
  verdicts(): Array<{ formula: string; verdict: Verdict }> { return this.monitors.map((m) => ({ formula: m.formula, verdict: m.verdict })); }
  summary(): { ticks: number; time: number; status: BTStatus; denied: number; violations: number; supervisorErrors: number; mode: string | null; plantState: string | null; verdicts: Array<{ formula: string; verdict: Verdict }> } {
    return { ticks: this.ticks, time: this.time, status: this.status, denied: this.denied, violations: this.violations, supervisorErrors: this.supervisorErrors, mode: this.modes?.mode ?? null, plantState: this.supervisor ? this.supervisor.plantState().join(',') : null, verdicts: this.verdicts() };
  }
}

function envOf(bb: Record<string, unknown>): Env { const e: Env = {}; for (const [k, v] of Object.entries(bb)) if (typeof v === 'number' || typeof v === 'boolean' || typeof v === 'string') e[k] = v as Value; return e; }

// ---------------------------------------------------------------------------------------------
// Station bindings
// ---------------------------------------------------------------------------------------------

/** Path planner for `goto` (I8: src/mobile/navigate.ts registers a footprint / drive-aware planner). Returns waypoints (mm) or null. */
export type GotoPlanner = (station: Station, robot: MobileRobot, goal: [number, number]) => number[][] | null;
let gotoPlanner: GotoPlanner | null = null;
export function registerGotoPlanner(p: GotoPlanner | null): void { gotoPlanner = p; }
/** Waypoints for a goto: the registered planner, else A* on the station's occupancy map (inflated by half the footprint width), else a straight line. */
export function gotoPath(station: Station, robot: MobileRobot, goal: [number, number]): number[][] | null {
  if (gotoPlanner) return gotoPlanner(station, robot, goal);
  const map = station.itemsOfType<MapItem>(ItemType.MAP)[0];
  if (map) { const r = planPath(map, [robot.state.x, robot.state.y], goal, { inflation: Math.max(robot.kin.footprint[0], robot.kin.footprint[1]) / 2, smooth: true }); return r.ok ? r.path : null; }
  return [[robot.state.x, robot.state.y], goal];
}

export interface StationWorldOptions { robot: MobileRobot; station: Station; /** seconds for timed actions */ durations?: Partial<Record<'grasp' | 'place' | 'dock' | 'stow' | 'reach', number>>; humanPosition?: () => [number, number] | null; /** arm programs replacing the timed skills (a mobile manipulator: Reach / Stow / Grasp / Place programs of its arm) */ skills?: Partial<Record<'reach' | 'stow' | 'grasp' | 'place', (ctx: TickContext, args: Record<string, unknown>, node: BTNode) => BTStatus | void>> }

/** Bind BT actions to a mobile robot in the station: goto zone=<name>, dock, grasp, place, stow, reach, halt, wait seconds=, set key=value, event emit=<uncontrollable event>. */
export function stationBindings(o: StationWorldOptions): WorldBindings & { stepRobot(dt: number): void } {
  const { robot, station } = o; const dur = { grasp: 4, place: 3, dock: 5, stow: 2, reach: 2, ...(o.durations ?? {}) };
  const timers = new Map<string, number>();
  const zones = () => station.itemsOfType<ZoneItem>(ItemType.ZONE);
  const zoneByName = (name: string) => zones().find((z) => z.name.toLowerCase() === name.toLowerCase());
  const timed = (key: string, seconds: number, ctx: TickContext, node: BTNode, onDone?: () => void): BTStatus => { const k = `${key}#${node.id}`; if (node.status !== 'running') timers.set(k, ctx.time); if (ctx.time - (timers.get(k) ?? ctx.time) >= seconds) { timers.delete(k); onDone?.(); return 'success'; } return 'running'; };
  const pending: string[] = [];
  let arrivedLast = false;
  /** A skill run by an arm program (o.skills); null when the skill has no program (the timed stand-in is used). */
  const viaArm = (name: 'reach' | 'stow' | 'grasp' | 'place', ctx: TickContext, args: Record<string, unknown>, node: BTNode, onOk: () => void, onFail?: () => void): BTStatus | null => {
    const h = o.skills?.[name]; if (!h) return null;
    const st = h(ctx, args, node) ?? 'success';
    if (st === 'success') onOk(); else if (st === 'failure') onFail?.();
    return st;
  };
  const bindings: WorldBindings & { stepRobot(dt: number): void } = {
    sense: () => {
      const z = zones().find((zz) => zz.contains(robot.state.x, robot.state.y));
      const hp = o.humanPosition?.() ?? null;
      const distHuman = hp ? Math.hypot(hp[0] - robot.state.x, hp[1] - robot.state.y) / 1000 : 99;
      return { battery: robot.batteryLevel(), x: robot.state.x, y: robot.state.y, v: Math.abs(robot.state.v) / 1000, zone: z?.name ?? '', moving: robot.state.status === 'moving', dist_human: distHuman, human: distHuman < 2, held: !!robot.params.holding, placed: !!robot.params.placed, arm_extended: !!robot.params.armExtended, docked: robot.state.status === 'charging' };
    },
    events: () => { const e = pending.splice(0); if (arrivedLast) { arrivedLast = false; } return e; },
    actions: {
      goto: (ctx, args, node) => {
        const name = String(args.zone ?? args.target ?? ''); const z = zoneByName(name);
        if (!z) { ctx.bb.error = `zone ${name} not found`; return 'failure'; }
        if (node.status !== 'running') { const path = gotoPath(station, robot, z.centroid()); if (!path) { ctx.bb.error = `no collision-free path to ${name}`; ctx.log?.(String(ctx.bb.error)); return 'failure'; } followPath(robot, path); return 'running'; }
        if (robot.state.path) return 'running';
        if (typeof args.arrive === 'string') pending.push(args.arrive);
        return z.contains(robot.state.x, robot.state.y) || Math.hypot(robot.state.x - z.centroid()[0], robot.state.y - z.centroid()[1]) < 600 ? 'success' : 'failure';
      },
      halt: () => { robot.state.path = null; robot.state.v = 0; robot.state.omega = 0; robot.state.status = 'idle'; return 'success'; },
      wait: (ctx, args, node) => timed('wait', Number(args.seconds ?? 1), ctx, node),
      dock: (ctx, _a, node) => timed('dock', dur.dock, ctx, node, () => { robot.state.status = 'charging'; }),
      undock: () => { robot.state.status = 'idle'; return 'success'; },
      reach: (ctx, a, node) => viaArm('reach', ctx, a, node, () => { robot.params.armExtended = true; }) ?? timed('reach', dur.reach, ctx, node, () => { robot.params.armExtended = true; }),
      stow: (ctx, a, node) => viaArm('stow', ctx, a, node, () => { robot.params.armExtended = false; }) ?? timed('stow', dur.stow, ctx, node, () => { robot.params.armExtended = false; }),
      grasp: (ctx, args, node) => viaArm('grasp', ctx, args, node, () => { robot.params.placed = false; robot.params.holding = true; if (typeof args.ok === 'string') pending.push(args.ok); }, () => { robot.params.holding = false; if (typeof args.miss === 'string') pending.push(args.miss); }) ?? timed('grasp', dur.grasp, ctx, node, () => { const p = Number(args.p ?? 1); robot.params.placed = false; robot.params.holding = Math.random() < p; if (robot.params.holding && typeof args.ok === 'string') pending.push(args.ok); else if (!robot.params.holding && typeof args.miss === 'string') pending.push(args.miss); }),
      place: (ctx, args, node) => viaArm('place', ctx, args, node, () => { robot.params.holding = false; robot.params.placed = true; const n = Number(ctx.bb.targets ?? 0); ctx.bb.targets = Math.max(0, n - 1); if (typeof args.put === 'string') pending.push(args.put); }) ?? timed('place', dur.place, ctx, node, () => { robot.params.holding = false; robot.params.placed = true; const n = Number(ctx.bb.targets ?? 0); ctx.bb.targets = Math.max(0, n - 1); if (typeof args.put === 'string') pending.push(args.put); }),
      set: (ctx, args) => { for (const [k, v] of Object.entries(args)) if (k !== 'event' && k !== 'done') ctx.bb[k] = v; return 'success'; },
      event: (_c, args) => { const e = args.emit ?? args.name; if (typeof e === 'string') pending.push(e); return 'success'; },
      report: (ctx) => { ctx.bb.reported = true; return 'success'; },
      select_target: (ctx) => ((ctx.bb.targets as number) > 0 ? 'success' : 'failure'),
      refine_pose: (ctx, _a, node) => timed('refine', 1, ctx, node),
    },
    halt: { goto: () => { robot.state.path = null; robot.state.v = 0; robot.state.status = 'idle'; } },
    stepRobot: (dt) => { const vmax = robot.params.vmaxOverride as number | undefined; stepMobile(robot, dt, vmax ? { speed: vmax * 1000 } : {}); },
  };
  return bindings;
}


// ---------------------------------------------------------------------------------------------
// Host actions: the studio's own programming artefacts as actions (robot programs, targets, component signals)
// ---------------------------------------------------------------------------------------------

/** What the app exposes to the control runtimes (built in ui/control_ui.ts over App, or headless in tests). */
export interface StationHost {
  station: Station;
  /** Who runs programs (per-robot lanes, real outcomes) and owns the signal table — see ctl/host.ts. */
  programs: ProgramBackend;
  /** Motion planner for `move` (default: MoveJ / MoveL with the robot's motion parameters). */
  planMove?(robot: Robot, target: Target, linear: boolean): { traj: Trajectory | null; error?: string };
  log?(msg: string, level?: 'info' | 'warn' | 'error'): void;
}

/** Robot resources: one owner (action node) per robot, FIFO among the waiting nodes. Programs and moves share them. */
export class RobotLocks {
  readonly owner = new Map<string, string>(); private waiting = new Map<string, string[]>();
  acquire(robotId: string, node: string): boolean {
    const o = this.owner.get(robotId); if (o === node) return true;
    let q = this.waiting.get(robotId); if (!q) { q = []; this.waiting.set(robotId, q); }
    if (!o && (!q.length || q[0] === node)) { if (q[0] === node) q.shift(); this.owner.set(robotId, node); return true; }
    if (!q.includes(node)) q.push(node);
    return false;
  }
  release(node: string): void { for (const [r, o] of [...this.owner]) if (o === node) this.owner.delete(r); for (const q of this.waiting.values()) { const i = q.indexOf(node); if (i >= 0) q.splice(i, 1); } }
}

const parseValue = (v: unknown): unknown => (typeof v === 'number' || typeof v === 'boolean' ? v : v === 'false' ? false : v === 'true' ? true : typeof v === 'string' && v.trim() !== '' && Number.isFinite(Number(v)) ? Number(v) : v ?? true);
/** `set` values: a string starting with `=` is an expression over the blackboard (`remaining = "=remaining - 1"`). */
export function setValue(v: unknown, bb: Record<string, unknown>): unknown { if (typeof v === 'string' && v.startsWith('=')) return expr(v.slice(1)).eval(envOf(bb)); return v; }
/** Signal name of a `signal` action: `component=` maps `name` to the component's `<name>Signal` field (stop → stopSignal), else `component.name`. */
function signalName(st: Station, name: string, component: unknown): string {
  if (typeof component !== 'string' || !component) return name;
  const c = st.itemsOfType<Item>(ItemType.COMPONENT).find((i) => i.name === component) as unknown as { behaviour?: Record<string, unknown> } | undefined;
  const field = c?.behaviour?.[`${name}Signal`];
  return typeof field === 'string' && field ? field : `${component}.${name}`;
}

/**
 * Actions available to every model:
 *  - `program program= [robot=]` runs a Program item on its robot through the program backend; programs of different
 *    robots run concurrently, a busy robot makes the action wait (FIFO). Fails when the program does not exist, does not
 *    compile (the compiler message is logged) or is stopped.
 *  - `move target= [robot=] [linear=true]` plays a planned MoveJ / MoveL to a Target; the robot is the explicit one, the
 *    target's own robot link, its robot parent, or the only robot of the station (ambiguity is a failure).
 *  - `signal signal= value= [component=]` writes the signal table; `wait seconds=`; `set key=value` (`=expr` evaluates);
 *    `await expr=` succeeds when the expression over the blackboard (signals included) holds; `event emit=` raises an event;
 *    `log message=`.
 * The halt of an action stops only that action's own program / move and frees its robot.
 */
export function hostBindings(host: StationHost): WorldBindings & { locks: RobotLocks } {
  const st = host.station; const pending: string[] = []; const locks = new RobotLocks(); const carried = new Map<string, { tool: string; local: ReturnType<typeof invert> }>();
  const timers = new Map<string, number>();
  const find = <T extends Item>(type: ItemType, name: unknown): T | undefined => (typeof name === 'string' ? (st.itemsOfType<T>(type).find((i) => i.name === name) ?? st.itemsOfType<T>(type).find((i) => i.name.toLowerCase() === name.toLowerCase())) : undefined);
  const runs = new Map<string, { program: Program; robot: Robot | null; run: ProgramRunHandle | null }>();
  const moves = new Map<string, { robot: Robot; tgt: Target; linear: boolean; traj: Trajectory | null; t0: number }>();
  const fail = (ctx: TickContext, node: BTNode, msg: string): BTStatus => { ctx.log?.(msg); ctx.bb.error = msg; host.log?.(msg, 'warn'); locks.release(node.id); runs.delete(node.id); moves.delete(node.id); return 'failure'; };
  const plan = host.planMove ?? planArmMove;
  return {
    locks,
    sense: () => { const env: Env = {}; for (const [k, v] of Object.entries(host.programs.signals.snapshot())) if (typeof v === 'number' || typeof v === 'boolean' || typeof v === 'string') env[k] = v; env.program_playing = host.programs.active().some((h) => h.status === 'running'); return env; },
    events: () => pending.splice(0),
    step: (dt) => { host.programs.tick(dt); carryAttachedObjects(st, carried); },
    actions: {
      program: (ctx, args, node) => {
        if (node.status !== 'running' || !runs.has(node.id)) {
          const name = args.program ?? args.name; const p = find<Program>(ItemType.PROGRAM, name);
          if (!p) return fail(ctx, node, `program ${String(name)} not found`);
          const own = p.robot(); const robot = own instanceof Robot ? own : (find<Robot>(ItemType.ROBOT, args.robot) ?? null);
          if (own instanceof Robot && typeof args.robot === 'string' && args.robot && own.name !== args.robot) ctx.log?.(`program ${p.name} belongs to ${own.name}, not ${args.robot} — it runs on ${own.name}`);
          runs.set(node.id, { program: p, robot, run: null });
        }
        const s = runs.get(node.id)!;
        if (!s.run) { if (s.robot && !locks.acquire(s.robot.id, node.id)) return 'running'; s.run = host.programs.start(s.program, { robot: s.robot }); }
        const status = s.run.status;
        if (status === 'queued' || status === 'running') return 'running';
        if (status === 'done') { locks.release(node.id); runs.delete(node.id); return 'success'; }
        return fail(ctx, node, `program ${s.program.name} ${status === 'error' ? `failed: ${s.run.error?.message ?? 'error'}` : status}`);
      },
      move: (ctx, args, node) => {
        if (node.status !== 'running' || !moves.has(node.id)) {
          const tgt = find<Target>(ItemType.TARGET, args.target); if (!tgt) return fail(ctx, node, `target ${String(args.target)} not found`);
          const { robot, why } = robotForTarget(st, tgt, args.robot); if (!robot) return fail(ctx, node, why);
          moves.set(node.id, { robot, tgt, linear: args.linear === true || args.linear === 'true', traj: null, t0: 0 });
        }
        const m = moves.get(node.id)!;
        if (!m.traj) {
          if (!locks.acquire(m.robot.id, node.id)) return 'running';
          const r = plan(m.robot, m.tgt, m.linear); if (!r.traj) return fail(ctx, node, r.error ?? `cannot move ${m.robot.name} to ${m.tgt.name}`);
          m.traj = r.traj; m.t0 = ctx.time;
        }
        const t = ctx.time - m.t0;
        m.robot.setJoints(sampleAt(m.traj, Math.min(t, m.traj.duration)).joints);
        if (t >= m.traj.duration - 1e-9) { locks.release(node.id); moves.delete(node.id); return 'success'; }
        return 'running';
      },
      signal: (_ctx, args) => { const name = String(args.signal ?? args.name); host.programs.signals.set(signalName(st, name, args.component), parseValue(args.value)); return 'success'; },
      wait: (ctx, args, node) => { const k = `w#${node.id}`; if (node.status !== 'running') timers.set(k, ctx.time); if (ctx.time - (timers.get(k) ?? ctx.time) >= Number(args.seconds ?? 1) - 1e-9) { timers.delete(k); return 'success'; } return 'running'; },
      set: (ctx, args) => { for (const [k, v] of Object.entries(args)) if (!['event', 'done', 'robot', 'fail', 'timeout'].includes(k)) ctx.bb[k] = setValue(v, ctx.bb); return 'success'; },
      await: (ctx, args) => { const e = String(args.expr ?? args.await ?? args.condition ?? 'true'); try { return expr(e).bool(envOf(ctx.bb)) ? 'success' : 'running'; } catch (err) { ctx.log?.(`await ${e}: ${(err as Error).message}`); return 'failure'; } },
      event: (_c, args) => { const e = args.emit ?? args.name; if (typeof e === 'string') pending.push(e); return 'success'; },
      log: (ctx, args) => { ctx.log?.(String(args.message ?? args.text ?? '')); return 'success'; },
    },
    halt: {
      program: (_c, _a, node) => { if (!node) return; const s = runs.get(node.id); if (s?.run) host.programs.stop(s.run); runs.delete(node.id); locks.release(node.id); },
      move: (_c, _a, node) => { if (!node) return; moves.delete(node.id); locks.release(node.id); },
      wait: (_c, _a, node) => { if (node) timers.delete(`w#${node.id}`); },
    },
  };
}

/**
 * Stand-in host for tests and headless scenarios: programs take `programs[name]` seconds on the lane of their robot
 * (`robot=` argument; programs of different robots overlap, same robot → FIFO), moves take `moveSeconds`, signals are
 * recorded. `log` keeps start / end times for timing assertions.
 */
export function simulatedHost(opts: { programs?: Record<string, number>; moveSeconds?: number; fail?: string[] } = {}): WorldBindings & { signals: Record<string, unknown>; runs: string[]; playing: string | null; log: Array<{ t: number; program: string; robot: string; event: 'start' | 'end' }> } {
  const timers = new Map<string, number>(); const pending: string[] = []; const locks = new RobotLocks();
  const out = { signals: {} as Record<string, unknown>, runs: [] as string[], playing: null as string | null, log: [] as Array<{ t: number; program: string; robot: string; event: 'start' | 'end' }> };
  const started = new Map<string, number>();
  const lane = (args: Record<string, unknown>, fallback: string) => (typeof args.robot === 'string' && args.robot ? args.robot : fallback);
  const timedOnLane = (key: string, laneId: string, seconds: number, ctx: TickContext, node: BTNode, label: string, onDone?: () => void): BTStatus => {
    const k = `${key}#${node.id}`;
    if (!started.has(k)) { if (!locks.acquire(laneId, node.id)) return 'running'; started.set(k, ctx.time); out.runs.push(label); out.playing = label; out.log.push({ t: ctx.time, program: label, robot: laneId, event: 'start' }); }
    if (ctx.time - started.get(k)! >= seconds - 1e-9) { started.delete(k); locks.release(node.id); out.log.push({ t: ctx.time, program: label, robot: laneId, event: 'end' }); if (out.playing === label) out.playing = null; onDone?.(); return 'success'; }
    return 'running';
  };
  return {
    ...out,
    get playing() { return out.playing; },
    sense: () => ({ program_playing: out.playing !== null, ...Object.fromEntries(Object.entries(out.signals).filter(([, v]) => typeof v !== 'object')) as Env }),
    events: () => pending.splice(0),
    actions: {
      program: (ctx, args, node) => { const name = String(args.program ?? args.name); const d = opts.programs?.[name]; if (d === undefined) { ctx.log?.(`program ${name} not found`); return 'failure'; } if (opts.fail?.includes(name)) { ctx.log?.(`program ${name} failed`); return 'failure'; } return timedOnLane('prog', lane(args, `prog:${name}`), d, ctx, node, name); },
      move: (ctx, args, node) => timedOnLane('move', lane(args, 'arm'), opts.moveSeconds ?? 1, ctx, node, `move ${String(args.target)}`),
      signal: (_c, args) => { out.signals[String(args.signal ?? args.name)] = parseValue(args.value); return 'success'; },
      wait: (ctx, args, node) => { const k = `wait#${node.id}`; if (node.status !== 'running') timers.set(k, ctx.time); if (ctx.time - (timers.get(k) ?? ctx.time) >= Number(args.seconds ?? 1) - 1e-9) { timers.delete(k); return 'success'; } return 'running'; },
      set: (ctx, args) => { for (const [k, v] of Object.entries(args)) if (!['event', 'done', 'robot', 'fail', 'timeout'].includes(k)) ctx.bb[k] = setValue(v, ctx.bb); return 'success'; },
      await: (ctx, args) => (expr(String(args.expr ?? args.await ?? 'true')).bool(envOf(ctx.bb)) ? 'success' : 'running'),
      event: (_c, args) => { const e = args.emit ?? args.name; if (typeof e === 'string') pending.push(e); return 'success'; },
      log: (ctx, args) => { ctx.log?.(String(args.message ?? '')); return 'success'; },
    },
    halt: { program: (_c, _a, node) => { if (!node) return; for (const k of [...started.keys()]) if (k.endsWith(`#${node.id}`)) started.delete(k); locks.release(node.id); out.playing = null; }, move: (_c, _a, node) => { if (!node) return; for (const k of [...started.keys()]) if (k.endsWith(`#${node.id}`)) started.delete(k); locks.release(node.id); } },
  };
}

// ---------------------------------------------------------------------------------------------
// Simulated stand-in world (tests, headless scenarios)
// ---------------------------------------------------------------------------------------------

export interface SimWorldState { x: number; y: number; battery: number; estop: boolean; humanDist: number; held: boolean; placed: boolean; targets: number; docked: boolean; zone: string; moving: boolean; events: string[]; armExtended: boolean }

export function simulatedBindings(init: Partial<SimWorldState> = {}, zones: Record<string, [number, number]> = { home: [0, 0], table: [10, 0], bin: [10, 8], dock: [0, 8] }, opts: { speed?: number; graspP?: number; seed?: number } = {}): WorldBindings & { state: SimWorldState } {
  const st: SimWorldState = { x: 0, y: 0, battery: 0.9, estop: false, humanDist: 10, held: false, placed: false, targets: 1, docked: false, zone: 'home', moving: false, events: [], armExtended: false, ...init };
  const speed = opts.speed ?? 1; let seed = opts.seed ?? 1; const rnd = () => { seed = (seed * 1664525 + 1013904223) >>> 0; return seed / 4294967296; };
  let target: [number, number] | null = null; const timers = new Map<string, number>();
  const timed = (key: string, seconds: number, ctx: TickContext, node: BTNode, onDone?: () => void): BTStatus => { const k = `${key}#${node.id}`; if (node.status !== 'running') timers.set(k, ctx.time); if (ctx.time - (timers.get(k) ?? ctx.time) >= seconds) { timers.delete(k); onDone?.(); return 'success'; } return 'running'; };
  const zoneAt = () => Object.entries(zones).find(([, p]) => Math.hypot(p[0] - st.x, p[1] - st.y) < 0.5)?.[0] ?? '';
  return {
    state: st,
    sense: () => ({ battery: st.battery, estop: st.estop, dist_human: st.humanDist, human: st.humanDist < 2, held: st.held, placed: st.placed, targets: st.targets, docked: st.docked, zone: st.zone, moving: st.moving, x: st.x, y: st.y, arm_extended: st.armExtended }),
    events: () => st.events.splice(0),
    step: (dt) => {
      if (target && !st.estop) { const dx = target[0] - st.x, dy = target[1] - st.y; const d = Math.hypot(dx, dy); const s = Math.min(d, speed * dt); if (d > 1e-6) { st.x += (dx / d) * s; st.y += (dy / d) * s; } st.moving = true; st.battery -= 0.002 * dt; if (d <= speed * dt) { target = null; st.moving = false; } }
      else st.moving = false;
      if (st.docked) st.battery = Math.min(1, st.battery + 0.05 * dt);
      st.zone = zoneAt();
    },
    actions: {
      goto: (ctx, args, node) => { const name = String(args.zone ?? ''); const p = zones[name]; if (!p) return 'failure'; if (node.status !== 'running') { target = p; st.docked = false; return 'running'; } if (target) return 'running'; if (typeof args.arrive === 'string') st.events.push(args.arrive); return zoneAt() === name ? 'success' : 'failure'; },
      halt: () => { target = null; st.moving = false; return 'success'; },
      wait: (ctx, args, node) => timed('wait', Number(args.seconds ?? 1), ctx, node),
      dock: (ctx, _a, node) => timed('dock', 2, ctx, node, () => { st.docked = true; }),
      reach: (ctx, _a, node) => timed('reach', 1, ctx, node, () => { st.armExtended = true; }),
      stow: (ctx, _a, node) => timed('stow', 1, ctx, node, () => { st.armExtended = false; }),
      grasp: (ctx, args, node) => timed('grasp', 2, ctx, node, () => { st.placed = false; st.held = rnd() < (opts.graspP ?? 1); if (st.held && typeof args.ok === 'string') st.events.push(args.ok); if (!st.held && typeof args.miss === 'string') st.events.push(args.miss); }),
      drop: () => { st.held = false; st.events.push('g_slip'); return 'success'; },
      place: (ctx, args, node) => timed('place', 1, ctx, node, () => { st.held = false; st.placed = true; st.targets = Math.max(0, st.targets - 1); if (typeof args.put === 'string') st.events.push(args.put); }),
      set: (ctx, args) => { for (const [k, v] of Object.entries(args)) if (k !== 'event' && k !== 'done') { ctx.bb[k] = v; if (k in st) (st as unknown as Record<string, unknown>)[k] = v; } return 'success'; },
      event: (_c, args) => { const e = args.emit ?? args.name; if (typeof e === 'string') st.events.push(e); return 'success'; },
      report: (ctx) => { ctx.bb.reported = true; return 'success'; },
      select_target: () => (st.targets > 0 ? 'success' : 'failure'),
      refine_pose: (ctx, _a, node) => timed('refine', 0.5, ctx, node),
    },
    halt: { goto: () => { target = null; st.moving = false; } },
  };
}
