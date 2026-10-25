/**
 * The station as an executor world, independent of the UI (the Control tab and the headless tests build the same thing):
 * host actions over a program backend, one set of mobile bindings per mobile robot (goto / dock / … and, for a mobile
 * manipulator, reach / stow / grasp / place run by its arm's programs), the world blackboard and the world step.
 *
 * Blackboard: the signal table, `program_playing`, and per mobile robot its sense values prefixed with the robot name
 * (`youBot.zone`); the values of the first mobile robot are also unprefixed (`zone`, `battery`, `held`…).
 */
import { ItemType, Station, Item } from '../core/items/item';
import { Robot } from '../core/items/robot';
import { Program } from '../core/items/program';
import { MobileRobot } from '../mobile/items';
import type { BTNode, TickContext } from './bt';
import type { Env } from './expr';
import type { ExecutorWorld } from './exec';
import { hostBindings, stationBindings, StationHost, WorldBindings, RobotLocks } from './runtime';
import type { ProgramBackend } from './host';

const safe = (s: string) => s.replace(/[^A-Za-z0-9_]/g, '_');

/** Arm programs of a mobile manipulator for the skills: a program of an arm mounted on the base whose name starts with the skill (Place also matches Release). */
export function armSkillPrograms(station: Station, base: MobileRobot): Partial<Record<'reach' | 'stow' | 'grasp' | 'place', Program>> {
  const arms = new Set<Item>(); const walk = (it: Item) => { for (const c of it.children) { if (c instanceof Robot) arms.add(c); walk(c); } }; walk(base);
  if (!arms.size) return {};
  const progs = station.itemsOfType<Program>(ItemType.PROGRAM).filter((p) => { const r = p.robot(); return !!r && arms.has(r); });
  const out: Partial<Record<'reach' | 'stow' | 'grasp' | 'place', Program>> = {};
  for (const sk of ['reach', 'stow', 'grasp', 'place'] as const) { const p = progs.find((x) => x.name.toLowerCase().startsWith(sk)) ?? (sk === 'place' ? progs.find((x) => /^release/i.test(x.name)) : undefined); if (p) out[sk] = p; }
  return out;
}

export interface StationWorld extends ExecutorWorld { host: WorldBindings & { locks: RobotLocks }; programs: ProgramBackend; stop(): void; mobiles: Map<string, WorldBindings & { stepRobot(dt: number): void }> }

export function makeStationWorld(sh: StationHost, opts: { inFleet?: (r: MobileRobot) => boolean; humanPosition?: () => [number, number] | null } = {}): StationWorld {
  const st = sh.station; const host = hostBindings(sh);
  const mobiles = new Map<string, WorldBindings & { stepRobot(dt: number): void }>();
  const bindingsOf = (r: MobileRobot) => {
    let b = mobiles.get(r.id);
    if (!b) {
      const skills: Record<string, (ctx: TickContext, args: Record<string, unknown>, node: BTNode) => ReturnType<NonNullable<WorldBindings['actions'][string]>>> = {};
      for (const [sk, p] of Object.entries(armSkillPrograms(st, r))) skills[sk] = (ctx, args, node) => host.actions.program(ctx, { ...args, program: p!.name, robot: undefined }, node);
      b = stationBindings({ robot: r, station: st, humanPosition: opts.humanPosition, skills });
      // the arm programs started by the skills must be halted with the skill
      b.halt = { ...(b.halt ?? {}), ...Object.fromEntries(Object.keys(skills).map((k) => [k, (ctx: TickContext, args: Record<string, unknown>, node?: BTNode) => host.halt?.program?.(ctx, args, node)])) };
      mobiles.set(r.id, b);
    }
    return b;
  };
  const robots = () => st.itemsOfType<MobileRobot>(ItemType.MOBILE_ROBOT);
  const forRobot = (name?: string) => { const r = robots().find((m) => m.name === name) ?? (!name ? robots()[0] : undefined); return r ? bindingsOf(r) : null; };
  return {
    host, programs: sh.programs, mobiles, forRobot,
    sense: () => {
      const env: Env = { ...host.sense() };
      robots().forEach((r, i) => { const s = bindingsOf(r).sense(); for (const [k, v] of Object.entries(s)) { env[`${safe(r.name)}.${k}`] = v; if (i === 0 && !(k in env)) env[k] = v; } });
      return env;
    },
    step: (dt) => { host.step?.(dt); for (const r of robots()) { const b = mobiles.get(r.id); if (b && !opts.inFleet?.(r)) b.stepRobot(dt); } },
    events: () => [...(host.events?.() ?? []), ...[...mobiles.values()].flatMap((b) => b.events?.() ?? [])],
    stop: () => { sh.programs.stopAll(); for (const r of robots()) if (mobiles.has(r.id)) { r.state.path = null; r.state.v = 0; r.state.status = 'idle'; } },
  };
}
