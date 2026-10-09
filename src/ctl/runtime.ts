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
import { Env, Value } from './expr';
import type { Station, Item } from '../core/items/item';
import { ItemType } from '../core/items/item';
import { MobileRobot, ZoneItem } from '../mobile/items';
import type { Robot } from '../core/items/robot';
import type { Program } from '../core/items/program';
import type { Target } from '../core/items/item';
import { followPath, stepMobile } from '../mobile/controller';

export interface WorldBindings {
  /** Values merged into the blackboard every tick (battery, dist_human, estop, …). */
  sense(): Env;
  actions: Record<string, (ctx: TickContext, args: Record<string, unknown>, node: BTNode) => BTStatus | void>;
  halt?: Record<string, (ctx: TickContext, args: Record<string, unknown>) => void>;
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
    this.tree = new BehaviorTree(opts.bt.root, { actions, halt: b.halt }, this.bb);
  }
  private say(msg: string): void { this.log.push(`[${this.time.toFixed(1)}] ${msg}`); this.opts.log?.(msg); }
  /** Feed the uncontrollable events the world reported so far (arrivals, sensor events) to the supervisor. */
  private drain(): void { for (const ev of this.opts.bindings.events?.() ?? []) this.observe(ev); }
  private observe(ev: string): void { if (!this.supervisor) return; try { this.supervisor.observe(ev); } catch (e) { if (e instanceof SupervisorViolation) { this.supervisorErrors++; this.say(`model mismatch: ${e.message}`); } else throw e; } }

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

export interface StationWorldOptions { robot: MobileRobot; station: Station; /** seconds for timed actions */ durations?: Partial<Record<'grasp' | 'place' | 'dock' | 'stow' | 'reach', number>>; humanPosition?: () => [number, number] | null }

/** Bind BT actions to a mobile robot in the station: goto zone=<name>, dock, grasp, place, stow, reach, halt, wait seconds=, set key=value, event emit=<uncontrollable event>. */
export function stationBindings(o: StationWorldOptions): WorldBindings & { stepRobot(dt: number): void } {
  const { robot, station } = o; const dur = { grasp: 4, place: 3, dock: 5, stow: 2, reach: 2, ...(o.durations ?? {}) };
  const timers = new Map<string, number>();
  const zones = () => station.itemsOfType<ZoneItem>(ItemType.ZONE);
  const zoneByName = (name: string) => zones().find((z) => z.name.toLowerCase() === name.toLowerCase());
  const timed = (key: string, seconds: number, ctx: TickContext, node: BTNode, onDone?: () => void): BTStatus => { const k = `${key}#${node.id}`; if (node.status !== 'running') timers.set(k, ctx.time); if (ctx.time - (timers.get(k) ?? ctx.time) >= seconds) { timers.delete(k); onDone?.(); return 'success'; } return 'running'; };
  const pending: string[] = [];
  let arrivedLast = false;
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
        if (node.status !== 'running') { const [cx, cy] = z.centroid(); followPath(robot, [[robot.state.x, robot.state.y], [cx, cy]]); return 'running'; }
        if (robot.state.path) return 'running';
        if (typeof args.arrive === 'string') pending.push(args.arrive);
        return z.contains(robot.state.x, robot.state.y) || Math.hypot(robot.state.x - z.centroid()[0], robot.state.y - z.centroid()[1]) < 600 ? 'success' : 'failure';
      },
      halt: () => { robot.state.path = null; robot.state.v = 0; robot.state.omega = 0; robot.state.status = 'idle'; return 'success'; },
      wait: (ctx, args, node) => timed('wait', Number(args.seconds ?? 1), ctx, node),
      dock: (ctx, _a, node) => timed('dock', dur.dock, ctx, node, () => { robot.state.status = 'charging'; }),
      undock: () => { robot.state.status = 'idle'; return 'success'; },
      reach: (ctx, _a, node) => timed('reach', dur.reach, ctx, node, () => { robot.params.armExtended = true; }),
      stow: (ctx, _a, node) => timed('stow', dur.stow, ctx, node, () => { robot.params.armExtended = false; }),
      grasp: (ctx, args, node) => timed('grasp', dur.grasp, ctx, node, () => { const p = Number(args.p ?? 1); robot.params.placed = false; robot.params.holding = Math.random() < p; if (robot.params.holding && typeof args.ok === 'string') pending.push(args.ok); else if (!robot.params.holding && typeof args.miss === 'string') pending.push(args.miss); }),
      place: (ctx, args, node) => timed('place', dur.place, ctx, node, () => { robot.params.holding = false; robot.params.placed = true; const n = Number(ctx.bb.targets ?? 0); ctx.bb.targets = Math.max(0, n - 1); if (typeof args.put === 'string') pending.push(args.put); }),
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

/** What the app exposes to the control runtimes (implemented in ui/control_ui.ts over App). */
export interface StationHost {
  station: Station;
  /** Start an arm program in the program simulator (one at a time). */
  runProgram(p: Program): void;
  programPlaying(): boolean;
  stopProgram(): void;
  /** Joint trajectory duration (s) for a MoveJ from the current joints to the target; null when unreachable. */
  moveDuration(robot: Robot, target: Target, linear?: boolean): number | null;
  /** Process-component signal (the world clock's signal table). */
  setSignal(name: string, value: boolean | number): void;
}

/**
 * Actions available to every model: `program name= [robot=]` (runs a Program item; waits while the simulator plays
 * another one), `move target= [robot=] [linear=true]` (moves an arm to a Target over the planned duration),
 * `signal name= value=` (sets a process signal used by the components), `wait seconds=`, `set key=value`,
 * `event emit=` (an uncontrollable event for the supervisor / executor).
 */
export function hostBindings(host: StationHost): WorldBindings {
  const st = host.station; const pending: string[] = [];
  const timers = new Map<string, number>();
  const owner = { node: null as string | null }; const queue: string[] = []; // first come, first served for the single program simulator
  const find = <T extends Item>(type: ItemType, name: unknown): T | undefined => (typeof name === 'string' ? (st.itemsOfType<T>(type).find((i) => i.name === name) ?? st.itemsOfType<T>(type).find((i) => i.name.toLowerCase() === name.toLowerCase())) : undefined);
  const robotOf = (args: Record<string, unknown>, fallback?: Item | null): Robot | null => (find<Robot>(ItemType.ROBOT, args.robot) ?? (fallback as Robot | null) ?? st.itemsOfType<Robot>(ItemType.ROBOT)[0] ?? null);
  const moves = new Map<string, { robot: Robot; q0: number[]; q1: number[]; t0: number; dur: number }>();
  return {
    sense: () => ({ program_playing: host.programPlaying(), time_host: 0 }),
    events: () => pending.splice(0),
    actions: {
      program: (ctx, args, node) => {
        const p = find<Program>(ItemType.PROGRAM, args.name ?? args.program);
        if (!p) { ctx.log?.(`program ${String(args.name)} not found`); return 'failure'; }
        if (node.status !== 'running' || owner.node !== node.id) {
          if (!queue.includes(node.id)) queue.push(node.id);
          if (host.programPlaying() || queue[0] !== node.id) return 'running'; // another program is playing or queued earlier: wait for the simulator
          queue.shift(); host.runProgram(p); owner.node = node.id; timers.set(node.id, ctx.time); return 'running';
        }
        if (host.programPlaying() && ctx.time - (timers.get(node.id) ?? ctx.time) < 0.2) return 'running';
        if (host.programPlaying()) return 'running';
        owner.node = null; timers.delete(node.id); return 'success';
      },
      move: (ctx, args, node) => {
        const tgt = find<Target>(ItemType.TARGET, args.target); if (!tgt) { ctx.log?.(`target ${String(args.target)} not found`); return 'failure'; }
        if (node.status !== 'running') {
          const robot = robotOf(args, (tgt.parent && (tgt.parent as Item).type === ItemType.ROBOT ? (tgt.parent as Item) : null)); if (!robot) { ctx.log?.('no robot for move'); return 'failure'; }
          const dur = host.moveDuration(robot, tgt, args.linear === true); if (dur === null) { ctx.log?.(`${tgt.name} unreachable for ${robot.name}`); return 'failure'; }
          const q1 = robot.jointsForTarget(tgt); if (!q1) return 'failure';
          moves.set(node.id, { robot, q0: robot.joints(), q1, t0: ctx.time, dur: Math.max(dur, 0.05) }); return 'running';
        }
        const m = moves.get(node.id); if (!m) return 'success';
        const s = Math.min(1, (ctx.time - m.t0) / m.dur); const sm = s * s * (3 - 2 * s);
        m.robot.setJoints(m.q0.map((v, i) => v + (m.q1[i] - v) * sm));
        if (s >= 1) { moves.delete(node.id); return 'success'; }
        return 'running';
      },
      signal: (_ctx, args) => {
        const v = args.value; host.setSignal(String(args.name), typeof v === 'number' || typeof v === 'boolean' ? v : v === 'false' ? false : v === 'true' ? true : Number.isFinite(Number(v)) ? Number(v) : true); return 'success';
      },
      wait: (ctx, args, node) => { const k = `w#${node.id}`; if (node.status !== 'running') timers.set(k, ctx.time); if (ctx.time - (timers.get(k) ?? ctx.time) >= Number(args.seconds ?? 1)) { timers.delete(k); return 'success'; } return 'running'; },
      set: (ctx, args) => { for (const [k, v] of Object.entries(args)) if (k !== 'event' && k !== 'done' && k !== 'robot') ctx.bb[k] = v; return 'success'; },
      event: (_c, args) => { const e = args.emit ?? args.name; if (typeof e === 'string') pending.push(e); return 'success'; },
    },
    halt: { program: (_c, _a) => { void _a; if (owner.node) { host.stopProgram(); owner.node = null; } queue.length = 0; }, move: (_c, args) => { void args; } },
  };
}

/** Stand-in host for tests and headless scenarios: programs and moves take `durations` seconds, signals are recorded. */
export function simulatedHost(opts: { programs?: Record<string, number>; moveSeconds?: number } = {}): WorldBindings & { signals: Record<string, unknown>; runs: string[]; playing: string | null } {
  const timers = new Map<string, number>(); const pending: string[] = []; const queue: string[] = [];
  const out = { signals: {} as Record<string, unknown>, runs: [] as string[], playing: null as string | null };
  const timed = (key: string, seconds: number, ctx: TickContext, node: BTNode, onStart?: () => boolean, onDone?: () => void): BTStatus => { const k = `${key}#${node.id}`; if (!timers.has(k)) { if (onStart && !onStart()) return 'running'; timers.set(k, ctx.time); } if (ctx.time - timers.get(k)! >= seconds) { timers.delete(k); onDone?.(); return 'success'; } return 'running'; };
  return {
    ...out,
    sense: () => ({ program_playing: out.playing !== null }),
    events: () => pending.splice(0),
    actions: {
      program: (ctx, args, node) => { const name = String(args.name); const d = opts.programs?.[name]; if (d === undefined) return 'failure'; return timed('prog', d, ctx, node, () => { if (!queue.includes(node.id)) queue.push(node.id); if (out.playing || queue[0] !== node.id) return false; queue.shift(); out.playing = `${name}#${node.id}`; out.runs.push(name); return true; }, () => { out.playing = null; }); },
      move: (ctx, args, node) => timed('move', opts.moveSeconds ?? 1, ctx, node, undefined, () => { out.runs.push(`move ${String(args.target)}`); }),
      signal: (_c, args) => { out.signals[String(args.name)] = args.value; return 'success'; },
      wait: (ctx, args, node) => timed('wait', Number(args.seconds ?? 1), ctx, node),
      set: (ctx, args) => { for (const [k, v] of Object.entries(args)) ctx.bb[k] = v; return 'success'; },
      event: (_c, args) => { const e = args.emit ?? args.name; if (typeof e === 'string') pending.push(e); return 'success'; },
    },
    halt: { program: () => { out.playing = null; } },
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
