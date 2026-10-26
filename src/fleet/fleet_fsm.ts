/**
 * Per-robot FSM bound to fleet tasks: a studio `fsm` document (Control tab kind) is the robot's own controller inside
 * a FleetManager. The fleet keeps the robot on hold (`fm.hold`): it receives a task only while its FSM is in a state
 * whose skill asks for one (`fleet_execute`), goes to charge when its FSM decides so (`fleet_charge`), waits
 * (`wait=`), … The executor runs on the fleet clock (call `tick(dt)` after every `fm.step(dt)`), its trace is the
 * robot's lifecycle (Idle / Execute / Charge …) with the fleet time and the task each Execute ran.
 *
 * Skills bound to the fleet:
 *  - `fleet_execute`  — release the hold, take the task the allocator gives (queued first), run it until done → `ok`;
 *                       `fail` (outcome mapped with `fail=`) when no task comes within `wait` s (default 2) or it fails;
 *  - `fleet_charge`   — drive home / to the charging zone and charge to 95 % → `ok` (`fail` when no path);
 *  - `fleet_idle`     — refresh the blackboard (battery, pending, tasks_done) → `ok`;
 *  - `wait=<s>`       — fleet-time wait.
 * Blackboard: `battery` (0..1), `pending` (open fleet tasks), `tasks_done` (by this robot), `task` (current id).
 */
import { parseFsm, FsmExecutor, FsmDoc } from '../ctl/fsm';
import type { ExecutorWorld } from '../ctl/exec';
import type { WorldBindings } from '../ctl/runtime';
import type { BTStatus, TickContext } from '../ctl/bt';
import type { MobileRobot } from '../mobile/items';
import type { FleetManager } from './fleet';

/** Default robot controller: charge when below 30 %, else ask for work; no work → wait 5 s and ask again. */
export const FLEET_ROBOT_FSM = `fsm Fleet robot
blackboard battery=1 pending=0 tasks_done=0 task=""
outcomes stopped
state Idle initial
  do skill=fleet_idle
  ok -> Decide
  fail -> stopped
state Decide decision
  when "battery < 0.3" -> charge
  when "pending > 0" -> work
  otherwise -> rest
  charge -> Charge
  work -> Execute
  rest -> Rest
state Execute
  do skill=fleet_execute fail=no_task
  ok -> Idle
  no_task -> Rest
state Rest
  do wait=5
  ok -> Idle
  fail -> Idle
state Charge
  do skill=fleet_charge
  ok -> Idle
  fail -> Rest`;

export interface FleetFsmTraceEntry { t: number; state: string; task?: string }

export class FleetRobotFsm {
  readonly exec: FsmExecutor;
  /** States entered (fleet time) with the task an Execute ran. */
  readonly trace: FleetFsmTraceEntry[] = [];
  private runs = new Map<string, { phase: string; t0: number; task?: string; until?: number }>();
  private done = 0;
  constructor(readonly fm: FleetManager, readonly robot: MobileRobot, doc: FsmDoc | string = FLEET_ROBOT_FSM) {
    fm.hold.add(robot.id);
    const d = typeof doc === 'string' ? parseFsm(doc) : doc;
    const host: WorldBindings = { sense: () => this.sense(), actions: this.actions() };
    const world: ExecutorWorld = { host, forRobot: () => null, events: () => [], sense: () => this.sense() };
    this.exec = new FsmExecutor({ doc: d, world });
  }
  private sense() {
    const pending = this.fm.fleet.tasks.filter((t) => t.type !== 'charge' && (t.status === 'pending' || (t.status === 'assigned' && t.robotId === this.robot.id))).length;
    return { battery: this.robot.batteryLevel(), pending, tasks_done: this.done, task: this.robot.state.taskId ?? '' };
  }
  private enter(state: string, task?: string): void { this.trace.push({ t: this.fm.time, state, ...(task ? { task } : {}) }); }
  private actions(): WorldBindings['actions'] {
    const fm = this.fm, r = this.robot;
    const run = (node: { id: string }) => this.runs.get(node.id);
    return {
      fleet_idle: (ctx: TickContext) => { this.enter('Idle'); Object.assign(ctx.bb, this.sense()); return 'success'; },
      fleet_execute: (_ctx: TickContext, args: Record<string, unknown>, node: { id: string }): BTStatus => {
        let s = run(node);
        if (!s) { s = { phase: 'request', t0: fm.time }; this.runs.set(node.id, s); fm.hold.delete(r.id); this.enter('Execute'); }
        if (s.phase === 'request') {
          const t = r.state.taskId ? fm.fleet.tasks.find((k) => k.id === r.state.taskId) : null;
          if (t && t.type !== 'charge') { s.phase = 'run'; s.task = t.id; fm.hold.add(r.id); this.trace[this.trace.length - 1].task = t.id; return 'running'; }
          if (fm.time - s.t0 >= Number(args.wait ?? 2)) { fm.hold.add(r.id); this.runs.delete(node.id); return 'failure'; }
          return 'running';
        }
        const t = fm.fleet.tasks.find((k) => k.id === s!.task);
        if (!t || t.status === 'done') { this.runs.delete(node.id); this.done++; return 'success'; }
        if (t.status === 'failed' || t.status === 'cancelled') { this.runs.delete(node.id); return 'failure'; }
        return 'running';
      },
      fleet_charge: (_ctx: TickContext, _args: Record<string, unknown>, node: { id: string }): BTStatus => {
        let s = run(node);
        if (!s) {
          this.enter('Charge');
          if (!fm.sendToCharge(r)) return 'failure';
          s = { phase: 'go', t0: fm.time }; this.runs.set(node.id, s);
        }
        if (s.phase === 'go') { if (r.state.status === 'charging') s.phase = 'charge'; return 'running'; }
        if (r.state.status !== 'charging') { this.runs.delete(node.id); return 'success'; }
        return 'running';
      },
      wait: (_ctx: TickContext, args: Record<string, unknown>, node: { id: string }): BTStatus => {
        let s = run(node);
        if (!s) { s = { phase: 'wait', t0: fm.time, until: fm.time + Number(args.seconds ?? 1) }; this.runs.set(node.id, s); this.enter('Rest'); }
        if (fm.time + 1e-9 >= s.until!) { this.runs.delete(node.id); return 'success'; }
        return 'running';
      },
    };
  }
  /** Advance the controller (after fm.step(dt)). */
  tick(dt: number): void { this.exec.tick(dt); }
  /** Stop controlling: the robot returns to the fleet's own allocation. */
  release(): void { this.exec.stop(); this.fm.hold.delete(this.robot.id); }
}

/** Put a robot of a fleet under its own FSM controller. */
export function attachFleetFsm(fm: FleetManager, robot: MobileRobot, doc?: FsmDoc | string): FleetRobotFsm {
  return new FleetRobotFsm(fm, robot, doc);
}
