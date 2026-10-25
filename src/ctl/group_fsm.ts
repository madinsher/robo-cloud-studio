/**
 * Group FSMs (agromarca-style "intellectons"): an `fsm` document with a `group` line is a per-robot template,
 * instantiated for every member robot of the station, all members on one clock with:
 *
 *  - per-robot bindings: `goto=`, `dock`, … act on the member's own mobile robot; its blackboard has `robot`, `caps`,
 *    `cap_<c>` (capabilities from the capability ontology), its own `zone` / `battery` / `x` / `y`;
 *  - messages: `skill=send msg=<m> to=<robot|all|cap:<c>>` → the recipients see `ev_<m>` for one tick and `msg_<m>`
 *    (until they clear it with `set=msg_<m> value=false`);
 *  - contract net (PROPOSED → ACCEPTED / REJECTED → EXECUTING → COMPLETED / CANCELLED, as in murca_core/contract.py):
 *    `skill=announce task=<id> cap=<c> zone=<z>` opens a contract; at the next group tick every member with the
 *    capability that is not `busy` bids its cost (distance to the zone in m + 5·(1 − battery)); the cheapest is
 *    awarded (`msg_awarded`, blackboard `task`, `task_zone`, `busy`), the other bidders get `msg_rejected`, no bidder →
 *    CANCELLED and `msg_no_bidder` to the announcer; `skill=start_task task=$task` → EXECUTING, `skill=complete
 *    task=$task` → COMPLETED and `msg_task_done` to the announcer, `skill=cancel_task task=$task` → CANCELLED.
 *
 * Capability ontology: `capabilities robot=c1,c2` lines in the document, or a YAML / JSON file (robot types, equipment,
 * robots with type + equipment) imported with `parseCapabilityOntology` / `applyCapabilityOntology`.
 */
import { ItemType, Station } from '../core/items/item';
import { MobileRobot, ZoneItem } from '../mobile/items';
import type { BTStatus } from './bt';
import type { ExecutorWorld } from './exec';
import type { Env } from './expr';
import { FsmDoc, FsmExecutor } from './fsm';
import type { WorldBindings } from './runtime';

export type ContractState = 'PROPOSED' | 'ACCEPTED' | 'REJECTED' | 'EXECUTING' | 'COMPLETED' | 'CANCELLED';
export interface Contract { id: string; task: string; cap: string; zone: string; manager: string; state: ContractState; bids: Array<{ robot: string; cost: number }>; winner?: string; history: Array<{ t: number; state: ContractState; robot?: string }> }
export interface GroupMember { name: string; robot: MobileRobot; caps: string[]; exec: FsmExecutor; inbox: string[] }

// ---------------------------------------------------------------------------------------------
// Capability ontology
// ---------------------------------------------------------------------------------------------

/** Minimal YAML (block mappings, `- item` and `[a, b]` lists, `{k: v}` flow maps, scalars) or JSON → object. */
export function parseYamlLite(text: string): unknown {
  const t = text.trim(); if (t.startsWith('{') || t.startsWith('[')) return JSON.parse(t);
  const scalar = (v: string): unknown => { const x = v.trim().replace(/\s+#.*$/, ''); if (x === '') return null; if (/^\[.*\]$/.test(x)) return x.slice(1, -1).split(',').map((e) => scalar(e)).filter((e) => e !== null); if (/^\{.*\}$/.test(x)) { const o: Record<string, unknown> = {}; for (const part of splitTop(x.slice(1, -1))) { const i = part.indexOf(':'); if (i > 0) o[part.slice(0, i).trim()] = scalar(part.slice(i + 1)); } return o; } if (/^-?\d+(\.\d+)?$/.test(x)) return Number(x); if (x === 'true' || x === 'false') return x === 'true'; return x.replace(/^["']|["']$/g, ''); };
  const splitTop = (s: string) => { const out: string[] = []; let depth = 0, cur = ''; for (const c of s) { if (c === '[' || c === '{') depth++; if (c === ']' || c === '}') depth--; if (c === ',' && depth === 0) { out.push(cur); cur = ''; } else cur += c; } if (cur.trim()) out.push(cur); return out; };
  const lines = text.split(/\r?\n/).map((l) => l.replace(/\t/g, '  ')).filter((l) => l.trim() && !l.trim().startsWith('#'));
  let i = 0;
  const block = (indent: number): unknown => {
    if (i < lines.length && lines[i].trim().startsWith('- ')) { const arr: unknown[] = []; while (i < lines.length && ind(lines[i]) === indent && lines[i].trim().startsWith('- ')) { arr.push(scalar(lines[i].trim().slice(2))); i++; } return arr; }
    const obj: Record<string, unknown> = {};
    while (i < lines.length && ind(lines[i]) === indent) {
      const l = lines[i].trim(); const k = l.indexOf(':'); if (k < 0) throw new Error(`YAML: expected 'key: value' at '${l}'`);
      const key = l.slice(0, k).trim().replace(/^["']|["']$/g, ''); const rest = l.slice(k + 1).trim(); i++;
      if (rest) obj[key] = scalar(rest); else obj[key] = i < lines.length && ind(lines[i]) > indent ? block(ind(lines[i])) : null;
    }
    return obj;
  };
  const ind = (l: string) => l.length - l.trimStart().length;
  return block(lines.length ? ind(lines[0]) : 0);
}

/**
 * Capabilities per robot from an ontology: `robots: {R1: {type: T, equipment: [e1], capabilities: [extra]}}` with
 * `robot_types: {T: {capabilities: [...]}}` and `equipment: {e1: {capabilities: [...]}}`; or a flat `{R1: [caps]}`.
 */
export function parseCapabilityOntology(text: string): Record<string, string[]> {
  const o = parseYamlLite(text) as Record<string, unknown>; if (!o || typeof o !== 'object') throw new Error('capability ontology: a mapping is expected');
  const list = (v: unknown): string[] => (Array.isArray(v) ? v.map(String) : typeof v === 'string' ? v.split(',').map((x) => x.trim()).filter(Boolean) : []);
  const types = (o.robot_types ?? o.types ?? {}) as Record<string, Record<string, unknown>>; const equip = (o.equipment ?? {}) as Record<string, Record<string, unknown>>;
  const robots = (o.robots ?? null) as Record<string, unknown> | null;
  const out: Record<string, string[]> = {};
  if (robots) for (const [r, spec] of Object.entries(robots)) {
    if (Array.isArray(spec) || typeof spec === 'string') { out[r] = list(spec); continue; }
    const s = (spec ?? {}) as Record<string, unknown>; const caps = new Set<string>(list(s.capabilities));
    const ty = s.type !== undefined ? types[String(s.type)] : undefined; if (s.type !== undefined && !ty) throw new Error(`capability ontology: robot ${r} has the unknown type ${String(s.type)}`);
    for (const c of list(ty?.capabilities)) caps.add(c);
    for (const e of list(s.equipment)) { const eq = equip[e]; if (!eq) throw new Error(`capability ontology: robot ${r} carries the unknown equipment ${e}`); for (const c of list(eq.capabilities)) caps.add(c); }
    out[r] = [...caps];
  } else for (const [r, v] of Object.entries(o)) out[r] = list(v);
  return out;
}

/** Set MobileRobot.capabilities from an ontology; returns the robots updated (unknown robot names are reported). */
export function applyCapabilityOntology(station: Station, text: string): { updated: Array<{ robot: string; caps: string[] }>; missing: string[] } {
  const caps = parseCapabilityOntology(text); const robots = station.itemsOfType<MobileRobot>(ItemType.MOBILE_ROBOT);
  const updated: Array<{ robot: string; caps: string[] }> = []; const missing: string[] = [];
  for (const [name, c] of Object.entries(caps)) { const r = robots.find((x) => x.name === name); if (!r) { missing.push(name); continue; } r.capabilities = [...c]; updated.push({ robot: name, caps: [...c] }); }
  return { updated, missing };
}

// ---------------------------------------------------------------------------------------------
// Executor
// ---------------------------------------------------------------------------------------------

const safe = (s: string) => s.replace(/[^A-Za-z0-9_]/g, '_');

export interface GroupFsmOptions { doc: FsmDoc; world: ExecutorWorld; station: Station; log?: (m: string) => void }

export class GroupFsmExecutor {
  readonly members: GroupMember[] = [];
  readonly contracts: Contract[] = [];
  readonly log: string[] = [];
  readonly messages: Array<{ t: number; from: string; to: string; msg: string }> = [];
  time = 0; ticks = 0; status: 'running' | 'idle' | 'stopped' = 'running';
  private seq = 0;
  constructor(readonly opts: GroupFsmOptions) {
    const { doc, world, station } = opts; const g = doc.group ?? { all: true };
    const robots = station.itemsOfType<MobileRobot>(ItemType.MOBILE_ROBOT);
    const capsOf = (r: MobileRobot) => doc.capabilities?.[r.name] ?? r.capabilities ?? [];
    const chosen = g.robots ? g.robots.map((n) => { const r = robots.find((x) => x.name === n); if (!r) throw new Error(`group: robot ${n} is not on the station`); return r; }) : robots.filter((r) => g.all || (g.capability ? capsOf(r).includes(g.capability) : true));
    if (!chosen.length) throw new Error('group: no member robot (add mobile robots, or check the group line / capabilities)');
    const allCaps = [...new Set(chosen.flatMap((r) => capsOf(r)))];
    for (const r of chosen) {
      const caps = capsOf(r); const inbox: string[] = []; const name = r.name;
      const own = world.forRobot(name);
      const host: WorldBindings = { ...world.host, actions: { ...world.host.actions, ...this.groupActions(name) } };
      const rw: ExecutorWorld = {
        host, forRobot: (n?: string) => world.forRobot(n ?? name), events: () => inbox.splice(0),
        sense: () => { const env: Env = { ...(world.sense?.() ?? world.host.sense()) }; const pre = `${safe(name)}.`; for (const [k, v] of Object.entries(env)) if (k.startsWith(pre)) env[k.slice(pre.length)] = v; if (own) Object.assign(env, own.sense()); env.robot = name; env.caps = caps.join(','); for (const c of allCaps) env[`cap_${safe(c)}`] = caps.includes(c); return env; },
      };
      const exec = new FsmExecutor({ doc, world: rw, log: (m) => this.say(`${name}: ${m}`) });
      this.members.push({ name, robot: r, caps, exec, inbox });
    }
    this.say(`group "${doc.name}" on ${this.members.map((m) => `${m.name} [${m.caps.join(', ') || '—'}]`).join(', ')}`);
  }
  private say(m: string): void { this.log.push(`[${this.time.toFixed(1)}] ${m}`); this.opts.log?.(m); }
  member(name: string): GroupMember | undefined { return this.members.find((m) => m.name === name); }
  /** Deliver a message: one-tick event `ev_<m>` and the persistent flag `msg_<m>`. */
  private deliver(from: string, to: GroupMember, msg: string): void { to.inbox.push(msg); to.exec.bb[`msg_${msg}`] = true; this.messages.push({ t: this.time, from, to: to.name, msg }); }
  private recipients(to: string, from: string): GroupMember[] { if (to === 'all' || to === '*') return this.members.filter((m) => m.name !== from); if (to.startsWith('cap:')) return this.members.filter((m) => m.name !== from && m.caps.includes(to.slice(4))); const m = this.member(to); return m ? [m] : []; }
  private setState(c: Contract, state: ContractState, robot?: string): void { c.state = state; c.history.push({ t: this.time, state, robot }); this.say(`contract ${c.id} (${c.task}) ${state}${robot ? ` — ${robot}` : ''}`); }
  private groupActions(me: string): WorldBindings['actions'] {
    const find = (task: unknown) => [...this.contracts].reverse().find((c) => c.task === String(task) || c.id === String(task));
    const ok = (): BTStatus => 'success';
    return {
      send: (ctx, a) => { const msg = String(a.msg ?? a.message ?? a.name ?? ''); if (!msg) { ctx.log?.('send: msg= is required'); return 'failure'; } const rs = this.recipients(String(a.to ?? 'all'), me); if (!rs.length) { ctx.log?.(`send ${msg}: no recipient ${String(a.to)}`); return 'failure'; } for (const r of rs) this.deliver(me, r, msg); return ok(); },
      announce: (ctx, a) => { const task = String(a.task ?? `task${this.seq + 1}`); const c: Contract = { id: `c${++this.seq}`, task, cap: String(a.cap ?? a.capability ?? ''), zone: String(a.zone ?? ''), manager: me, state: 'PROPOSED', bids: [], history: [] }; this.contracts.push(c); this.setState(c, 'PROPOSED', me); void ctx; return ok(); },
      start_task: (ctx, a) => { const c = find(a.task); if (!c || c.winner !== me) { ctx.log?.(`start_task ${String(a.task)}: not awarded to ${me}`); return 'failure'; } this.setState(c, 'EXECUTING', me); return ok(); },
      complete: (ctx, a) => { const c = find(a.task); if (!c || c.winner !== me) { ctx.log?.(`complete ${String(a.task)}: not awarded to ${me}`); return 'failure'; } this.setState(c, 'COMPLETED', me); const m = this.member(me)!; m.exec.bb.busy = false; const mg = this.member(c.manager); if (mg) this.deliver(me, mg, 'task_done'); return ok(); },
      cancel_task: (ctx, a) => { const c = find(a.task); if (!c) { ctx.log?.(`cancel_task ${String(a.task)}: unknown task`); return 'failure'; } this.setState(c, 'CANCELLED', me); const w = c.winner ? this.member(c.winner) : undefined; if (w) { w.exec.bb.busy = false; if (w.name !== me) this.deliver(me, w, 'cancelled'); } return ok(); },
    };
  }
  /** Award the contracts announced before this tick (contract net). */
  private award(): void {
    const zones = this.opts.station.itemsOfType<ZoneItem>(ItemType.ZONE);
    for (const c of this.contracts) {
      if (c.state !== 'PROPOSED' || c.history[0].t >= this.time) continue;
      const z = zones.find((x) => x.name === c.zone); const at = z ? z.centroid() : null;
      for (const m of this.members) {
        if (m.name === c.manager || (c.cap && !m.caps.includes(c.cap)) || m.exec.bb.busy === true || m.exec.status !== 'running') continue;
        const d = at ? Math.hypot(m.robot.state.x - at[0], m.robot.state.y - at[1]) / 1000 : 0;
        c.bids.push({ robot: m.name, cost: Number((d + 5 * (1 - m.robot.batteryLevel())).toFixed(3)) });
      }
      const mg = this.member(c.manager);
      if (!c.bids.length) { this.setState(c, 'CANCELLED'); if (mg) this.deliver('contract-net', mg, 'no_bidder'); continue; }
      const best = c.bids.slice().sort((a, b) => a.cost - b.cost || a.robot.localeCompare(b.robot))[0];
      c.winner = best.robot; this.setState(c, 'ACCEPTED', best.robot);
      const w = this.member(best.robot)!; Object.assign(w.exec.bb, { task: c.task, task_zone: c.zone, busy: true }); this.deliver(c.manager, w, 'awarded');
      for (const b of c.bids) if (b.robot !== best.robot) { c.history.push({ t: this.time, state: 'REJECTED', robot: b.robot }); this.deliver(c.manager, this.member(b.robot)!, 'rejected'); }
    }
  }
  tick(dt = 0.1): 'running' | 'idle' | 'stopped' {
    if (this.status !== 'running') return this.status;
    this.award();
    for (const m of this.members) m.exec.tick(dt);
    this.opts.world.step?.(dt);
    this.time += dt; this.ticks++;
    if (this.members.every((m) => m.exec.status !== 'running')) { this.status = 'idle'; this.say(`group finished: ${this.members.map((m) => `${m.name} → ${m.exec.outcome ?? m.exec.verdict}`).join(', ')}`); }
    return this.status;
  }
  run(seconds: number, dt = 0.1): 'running' | 'idle' | 'stopped' { const n = Math.ceil(seconds / dt - 1e-9); for (let i = 0; i < n && this.status === 'running'; i++) this.tick(dt); return this.status; }
  stop(): void { for (const m of this.members) if (m.exec.status === 'running') m.exec.stop(); this.status = 'stopped'; }
  where(): string { return this.members.map((m) => `${m.name}: ${m.exec.where()}`).join(' · '); }
}

/** Example: garden robots (agromarca-like) — a scout surveys a sector and announces a spray task, the closest sprayer wins it. */
export const GARDEN_GROUP_FSM = `fsm Garden robot (group template)
group all
blackboard busy=false task="" task_zone="" msg_awarded=false msg_rejected=false msg_task_done=false msg_no_bidder=false
outcomes done
state Role decision initial
  when "cap_survey" -> scout
  otherwise -> worker
  scout -> Scout
  worker -> Wait
state Scout
  do goto=S1 timeout=120
  ok -> Report
  fail -> done
state Report
  do skill=announce task=spray_S1 cap=spray zone=S1
  ok -> Supervise
  fail -> done
state Supervise decision
  when "msg_task_done || msg_no_bidder" -> finished
  finished -> Home
state Home
  do goto=base timeout=120
  ok -> done
  fail -> done
state Wait decision
  when "msg_awarded" -> go
  when "msg_rejected" -> stay
  go -> Execute
  stay -> done
state Execute
  entry skill=start_task task=$task
  do goto=$task_zone timeout=120
  ok -> Spray
  fail -> done
state Spray
  do wait=3
  exit skill=complete task=$task
  ok -> done
  fail -> done`;
