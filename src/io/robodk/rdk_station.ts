/**
 * RoboDK .rdk station decoder: one pass over the chunk list builds `chunk index -> Item`, a second pass resolves every
 * stored item reference (program robot, Set Ref / Set Tool, move targets, calls, events, external axes, machining links).
 * Linear in the file size: no name lookups, no tree walks per instruction.
 *
 * Chunk payloads (all offsets relative to the chunk payload):
 *   station    u8 | str name | flags | [u32 n | n × (str key, str value)] u32 0 | f64 200 | ...
 *   frame      str "FrameChkptStart" | str name | u8 visible | pose | str "FrameChkptEnd"
 *   robot      robot table (rdk_robot.ts) | meshes | 16 zero bytes | i32 activeFrame | i32 activeTool | driver strings |
 *              u32 nSync | nSync × i32 synchronised mechanism (external axes in joint order)
 *   tool/object  object table (rdk_robot.ts) | shapes | points | curves
 *   target     str name | u8 | i32 robot | u8 isJoint | u8 | u32 n | n × f64 joints | pose (relative to parent)
 *   program    str name | i32 -1 | u32 flags | u8 | i32 robot | u32 nIns | instruction*
 *              instruction = str "ins" | u32 index | u32 INS_TYPE | u64 next | str label | data
 *   python     str name | u8 | u32 nBytes | u32 | UTF-8 source
 *   machining  str "MachiningChkptStart" | str name | u8 | i32 robot | i32 frame | i32 tool | i32 part | i32 program | ...
 *   folder     str name | u8
 */
import { Station, Item, ItemType, Frame, Folder, SceneObject, Tool, Target, Camera, Notes } from '../../core/items/item';
import { Robot } from '../../core/items/robot';
import { Program, Instruction } from '../../core/items/program';
import type { InstructionData, MoveInstruction } from '../../core/items/program';
import type { Mat4 } from '../../core/math/pose';
import type { AssetStore } from '../../scene/assets';
import { RdkBytes, RdkChunk, rdkChunks } from './rdk_format';
import { decodeRobotTable, decodeObjectTable, externalJointInfo } from './rdk_robot';

export interface StationDecodeReport {
  counts: Record<string, number>;
  /** Data present in the file that the import could not reproduce (shown to the user, drives `fidelity`). */
  dropped: string[];
  notes: string[];
}

const KIND_NAME: Record<number, string> = { 1: 'station', 2: 'robot', 3: 'frame', 4: 'tool', 5: 'object', 6: 'target', 7: 'curve', 8: 'program', 9: 'instruction', 10: 'python', 11: 'machining', 17: 'folder', 19: 'camera', 22: 'notes' };

interface Pending {
  robots: Array<{ robot: Robot; frameRef: number; toolRef: number; sync: number[]; tableAt: number }>;
  targets: Array<{ target: Target; robotRef: number }>;
  programs: Array<{ program: Program; chunk: RdkChunk; robotRef: number; insAt: number; nIns: number }>;
  machining: Array<{ item: Item; refs: number[] }>;
}

function findMarker(r: RdkBytes, from: number, end: number, marker: number): number {
  for (let o = from; o < Math.min(end - 16, from + 64); o++) if (r.f64(o) === 1 && r.f64(o + 8) === marker) return o;
  return -1;
}

/** Station parameters: the only run of `u32 n, n × (key, value)` strings that ends right before `u32 0, f64 200`. */
function stationParams(r: RdkBytes, from: number, end: number): Array<[string, string]> {
  for (let p = from; p + 8 < end; p++) {
    const n = r.u32(p);
    if (n < 1 || n > 1000) continue;
    let q = p + 4;
    const kv: Array<[string, string]> = [];
    let ok = true;
    for (let k = 0; k < n && ok; k++) {
      const a = r.str(q, 4000); const b = a ? r.str(a.next, 1 << 16) : null;
      if (!a || !b || !a.text) { ok = false; break; }
      kv.push([a.text, b.text]); q = b.next;
    }
    if (ok && q + 12 <= end && r.u32(q) === 0 && r.f64(q + 4) === 200) return kv;
  }
  return [];
}

export function decodeStation(r: RdkBytes, fallbackName: string, assets?: AssetStore): { station: Station; report: StationDecodeReport } | null {
  const chunks = rdkChunks(r);
  if (!chunks.length || chunks[0].kind !== 1) return null;
  const report: StationDecodeReport = { counts: {}, dropped: [], notes: [] };
  const station = new Station(fallbackName || 'RoboDK station');
  const items: Array<Item | null> = new Array(chunks.length).fill(null);
  const parentOf: number[] = new Array(chunks.length).fill(-1);
  const pending: Pending = { robots: [], targets: [], programs: [], machining: [] };
  const stack: number[] = [];
  const count = (k: string, n = 1) => { report.counts[k] = (report.counts[k] ?? 0) + n; };
  const drop = (msg: string) => { if (report.dropped.length < 400) report.dropped.push(msg); };

  for (const c of chunks) {
    while (stack.length && chunks[stack[stack.length - 1]].depth >= c.depth) stack.pop();
    parentOf[c.index] = stack.length ? stack[stack.length - 1] : -1;
    stack.push(c.index);
    let parentIdx = parentOf[c.index];
    while (parentIdx >= 0 && !items[parentIdx]) parentIdx = parentOf[parentIdx];
    const parent: Item = parentIdx >= 0 ? items[parentIdx]! : station;
    const p = c.payload;
    let item: Item | null = null;
    try {
      switch (c.kind) {
        case 1: {
          const nm = r.str(p + 1, 4096);
          if (nm?.text) station.setName(nm.text);
          if (nm) {
            const kv = stationParams(r, nm.next, c.end);
            for (const [k, v] of kv) station.params[k] = v;
            if (kv.length) { station.params.rdkStationParams = kv.map(([k]) => k); count('station parameters', kv.length); }
          }
          items[c.index] = station;
          continue;
        }
        case 3: {
          const h = r.tag(p, 'FrameChkptStart');
          const nm = h > 0 ? r.str(h, 4096) : null;
          if (!nm) break;
          const f = new Frame(nm.text);
          if (r.u8(nm.next) === 0) f.setVisible(false);
          f.setPose(r.pose(nm.next + 1));
          item = f;
          break;
        }
        case 2: {
          const at = findMarker(r, p, c.end, 4000);
          const dec = at >= 0 ? decodeRobotTable(r, at, c.end, 'Robot', assets) : null;
          if (!dec) { drop(`robot chunk #${c.index}: parameter table not readable`); break; }
          for (const n of dec.notes) report.notes.push(n);
          let o = dec.meshEnd + 16;
          let frameRef = -1, toolRef = -1;
          const sync: number[] = [];
          if (r.ok(o, 8)) {
            frameRef = r.i32(o); toolRef = r.i32(o + 4); o += 8;
            // driver connection: str ip | u32 port | u32 | u32 | u32 | str path | str user | u32 | str driver | u32 nSync | refs
            const ip = r.str(o, 4096);
            if (ip) {
              o = ip.next + 16;
              const s1 = r.str(o, 1 << 16), s2 = s1 ? r.str(s1.next, 1 << 16) : null;
              const s3 = s2 ? r.str(s2.next + 4, 1 << 16) : null;
              if (s3) {
                o = s3.next;
                const n = r.ok(o, 4) ? r.u32(o) : 0;
                if (n > 0 && n <= 8 && r.ok(o, 4 + 4 * n)) for (let k = 0; k < n; k++) sync.push(r.i32(o + 4 + 4 * k));
              }
            }
          }
          pending.robots.push({ robot: dec.robot, frameRef, toolRef, sync, tableAt: at });
          item = dec.robot;
          break;
        }
        case 4:
        case 5: {
          const at = findMarker(r, p, c.end, 1600);
          const dec = at >= 0 ? decodeObjectTable(r, at, c.end, c.kind === 4 ? 'Tool' : 'Object', c.kind === 4, assets) : null;
          if (!dec) { drop(`${KIND_NAME[c.kind]} chunk #${c.index}: parameter table not readable`); break; }
          for (const n of dec.notes) drop(n);
          if (dec.curves) count('curves', dec.curves);
          if (dec.points) count('points', dec.points);
          item = dec.item;
          break;
        }
        case 6: {
          const nm = r.str(p, 4096);
          if (!nm) break;
          const t = new Target(nm.text);
          let o = nm.next;
          const robotRef = r.i32(o + 1);
          const isJoint = r.u8(o + 5) === 1;
          const n = r.u32(o + 7);
          o += 11;
          if (n <= 32 && r.ok(o, n * 8 + 128)) {
            const q: number[] = [];
            for (let k = 0; k < n; k++) q.push(r.f64(o + 8 * k));
            if (n) t.setJoints(q);
            t.setPose(r.pose(o + n * 8));
          } else drop(`target ${nm.text}: joint/pose record not readable`);
          if (isJoint) t.setAsJointTarget();
          pending.targets.push({ target: t, robotRef });
          item = t;
          break;
        }
        case 8: {
          const nm = r.str(p, 4096);
          if (!nm) break;
          const prog = new Program(nm.text);
          const h = nm.next;
          if (r.ok(h, 17)) pending.programs.push({ program: prog, chunk: c, robotRef: r.i32(h + 9), insAt: h + 17, nIns: r.u32(h + 13) });
          item = prog;
          break;
        }
        case 9:
          continue;
        case 10: {
          const nm = r.str(p, 4096);
          if (!nm) break;
          const py = new Item(ItemType.PROGRAM_PYTHON, nm.text);
          const n = r.u32(nm.next + 1);
          if (r.ok(nm.next + 9, n)) py.params.pythonCode = new TextDecoder('utf-8').decode(r.b.subarray(nm.next + 9, nm.next + 9 + n));
          else drop(`Python program ${nm.text}: source not readable`);
          item = py;
          break;
        }
        case 11: {
          const h = r.tag(p, 'MachiningChkptStart');
          const nm = h > 0 ? r.str(h, 4096) : null;
          if (!nm) break;
          const m = new Item(ItemType.MACHINING, nm.text);
          const refs = [0, 1, 2, 3, 4].map((k) => r.i32(nm.next + 1 + 4 * k));
          // first file path stored in the settings (NC / APT / G-code input), when any
          for (let o = nm.next + 21; o < Math.min(c.end, nm.next + 2000); o++) {
            const s = r.str(o, 2048);
            if (s && s.text.length > 4 && /^[A-Za-z]:[\\/]|^\//.test(s.text) && /\.\w{1,5}$/.test(s.text)) { m.params.ncFile = s.text; break; }
          }
          pending.machining.push({ item: m, refs });
          drop(`machining project ${nm.text}: settings other than robot/frame/tool/part/program links and the input file are not decoded`);
          item = m;
          break;
        }
        case 17: {
          const nm = r.str(p, 4096);
          if (!nm) break;
          const f = new Folder(nm.text);
          if (r.u8(nm.next) === 0) f.setVisible(false);
          item = f;
          break;
        }
        default: {
          const nm = r.str(p, 4096);
          const name = nm?.text && nm.text.length < 200 ? nm.text : `${KIND_NAME[c.kind] ?? `item type ${c.kind}`}`;
          item = c.kind === 19 ? new Camera(name) : c.kind === 22 ? new Notes(name) : new Item(c.kind as ItemType, name);
          drop(`${KIND_NAME[c.kind] ?? `item type ${c.kind}`} "${name}": only the name is imported`);
        }
      }
    } catch (e) {
      drop(`${KIND_NAME[c.kind] ?? c.kind} chunk #${c.index}: ${(e as Error).message}`);
      item = null;
    }
    if (!item) continue;
    parent.addChild(item);
    items[c.index] = item;
    count(KIND_NAME[c.kind] ?? `type ${c.kind}`);
  }

  const at = (ref: number): Item | null => (ref >= 0 && ref < items.length ? items[ref] : null);

  // ---- robots: active tool / frame, synchronised external axes
  for (const pr of pending.robots) {
    const { robot } = pr;
    const tool = at(pr.toolRef);
    if (tool instanceof Tool) robot.activeToolId = tool.id;
    const frame = at(pr.frameRef);
    if (frame) robot.activeFrameId = frame.id;
    const mechs = pr.sync.map(at).filter((m): m is Robot => m instanceof Robot && m !== robot);
    if (mechs.length) {
      robot.externalAxes = mechs.map((m) => m.id);
      const n = mechs.reduce((s, m) => s + m.dof, 0);
      const ext = externalJointInfo(r, pr.tableAt, robot.dof, n);
      let k = robot.dof;
      robot.params.syncAxes = mechs.map((m) => {
        const role = m.isAncestorOf(robot) ? 'carrier' : 'positioner';
        const joints = Array.from({ length: m.dof }, (_, i) => k + i);
        k += m.dof;
        return { id: m.id, name: m.name, role, joints };
      });
      robot.params.syncLimits = { lower: ext.lower, upper: ext.upper };
      robot.params.syncJoints = ext.current;
      count('synchronised external axes', n);
    }
  }
  for (const pt of pending.targets) { const rb = at(pt.robotRef); if (rb instanceof Robot) pt.target.robotId = rb.id; }
  const programsByName = new Map<string, Item>();
  for (const it of items) if (it instanceof Program && !programsByName.has(it.name)) programsByName.set(it.name, it);
  for (const pm of pending.machining) {
    const [robot, frame, tool, part, linked] = pm.refs.map(at);
    // without a stored link RoboDK (re)generates the program named after the project
    const program = linked instanceof Program ? linked : programsByName.get(pm.item.name.replace(/\s+settings$/i, '')) ?? null;
    const link: Record<string, string> = {};
    if (robot) link.robotId = robot.id;
    if (frame) link.frameId = frame.id;
    if (tool) link.toolId = tool.id;
    if (part) link.partId = part.id;
    if (program) { link.programId = program.id; program.params.machiningId = pm.item.id; }
    pm.item.params.rdkMachining = link;
  }

  // ---- programs
  const robots = items.filter((i): i is Robot => i instanceof Robot);
  const callables = new Map<string, Item>();
  for (const it of items) if (it && (it.type === ItemType.PROGRAM || it.type === ItemType.PROGRAM_PYTHON) && !callables.has(it.name)) callables.set(it.name, it);
  for (const pp of pending.programs) decodeProgram(r, pp, at, robots, callables, report);
  if (report.counts.instruction) delete report.counts.instruction;
  return { station, report };
}

const MOVE: Record<number, MoveInstruction['moveType']> = { 1: 'MoveJ', 2: 'MoveL', 3: 'MoveC' };

/** One move point: link 1 = target reference, link 2 = inline joints + pose (relative to the active frame). */
function movePoint(r: RdkBytes, o: number, at: (ref: number) => Item | null): { next: number; target: Target | null; ref: number; joints?: number[]; pose?: number[]; isJoint?: boolean } | null {
  const link = r.u32(o);
  if (link === 1) { const ref = r.i32(o + 4); const t = at(ref); return { next: o + 8, target: t instanceof Target ? t : null, ref }; }
  if (link === 2) {
    const isJoint = r.u16(o + 4) === 1;
    const n = r.u32(o + 6);
    if (n > 32 || !r.ok(o + 10, n * 8 + 132)) return null;
    const joints: number[] = [];
    for (let k = 0; k < n; k++) joints.push(r.f64(o + 10 + 8 * k));
    const pose = Array.from(r.pose(o + 10 + 8 * n));
    return { next: o + 10 + 8 * n + 132, target: null, ref: -1, joints, pose, isJoint };
  }
  return null;
}

function decodeProgram(r: RdkBytes, pp: Pending['programs'][number], at: (ref: number) => Item | null, robots: Robot[], callables: Map<string, Item>, report: StationDecodeReport): void {
  const { program, chunk } = pp;
  const end = chunk.end;
  const robotItem = at(pp.robotRef);
  let o = pp.insAt;
  let firstTargetRobot: string | null = null;
  const unknown: string[] = [];
  for (let k = 0; k < pp.nIns && o < end; k++) {
    const h = r.tag(o, 'ins');
    if (h < 0) { report.dropped.push(`program ${program.name}: instruction ${k + 1} of ${pp.nIns} not readable`); break; }
    const type = r.u32(h + 4);
    const next = r.u64(h + 8);
    const lab = r.str(h + 16, 1 << 20);
    if (!lab) { report.dropped.push(`program ${program.name}: instruction ${k + 1} label not readable`); break; }
    const label = lab.text;
    const d = lab.next;
    const dataEnd = next > o && next <= end ? next : end;
    o = dataEnd;
    let data: InstructionData | null = null;
    switch (type) {
      case 0:
      case 1: {
        const mt = MOVE[r.u32(d)] ?? (type === 1 ? 'MoveC' : 'MoveL');
        const a = movePoint(r, d + 4, at);
        const b = a && mt === 'MoveC' ? movePoint(r, a.next, at) : null;
        const endPt = mt === 'MoveC' ? b : a;
        if (!a || !endPt) break;
        if (endPt.ref >= 0 && !endPt.target) { data = { kind: 'print', message: `${label} (target deleted in RoboDK)`, isComment: true }; unknown.push(label); break; }
        const m: MoveInstruction = { kind: 'move', moveType: mt, targetId: endPt.target?.id ?? null };
        if (!endPt.target) { m.joints = endPt.joints; if (!endPt.isJoint) m.pose = endPt.pose; }
        if (mt === 'MoveC') { if (a.target) m.viaTargetId = a.target.id; else { m.viaTargetId = null; m.viaPose = a.pose; } }
        if (endPt.target?.robotId && !firstTargetRobot) firstTargetRobot = endPt.target.robotId;
        data = m;
        break;
      }
      case 2: {
        // u16 mask (1 linear speed, 2 linear accel, 4 joint speed, 8 joint accel) | f64 vL | f64 aL | i32 | f64 vJ | f64 aJ
        const mask = r.u16(d);
        const s: InstructionData = { kind: 'speed' };
        if (mask & 1) s.speedLinear = r.f64(d + 2);
        if (mask & 2) s.accelLinear = r.f64(d + 10);
        if (mask & 4) s.speedJoints = r.f64(d + 22);
        if (mask & 8) s.accelJoints = r.f64(d + 30);
        data = s;
        break;
      }
      case 3: case 4: {
        const ref = r.i32(d);
        const it = at(ref);
        if (type === 3) {
          if (it) data = { kind: 'frame', frameId: it.id };
          else if (ref === -1 && /^Set (Ref|Frame)/i.test(label) && !label.includes(':')) data = { kind: 'frame', frameId: null };
        } else if (it instanceof Tool) data = { kind: 'tool', toolId: it.id };
        else {
          // the tool item no longer exists: RoboDK keeps the TCP in the instruction, so keep it as a tool on the robot
          const owner = robotItem instanceof Robot ? robotItem : robots[0];
          if (owner) {
            const name = label.replace(/^Set Tool:?\s*/i, '') || 'Tool';
            let tool = owner.children.find((c): c is Tool => c instanceof Tool && c.name === name && c.params.rdkFromInstruction === true);
            if (!tool) { tool = new Tool(name); tool.setPose(r.pose(d + 5) as Mat4); tool.params.rdkFromInstruction = true; owner.addChild(tool); report.notes.push(`${program.name}: "${label}" refers to a deleted tool; recreated from the TCP stored in the instruction`); }
            data = { kind: 'tool', toolId: tool.id };
          }
        }
        break;
      }
      case 6: data = { kind: 'pause', timeMs: r.f64(d) }; break;
      case 7: data = eventData(r, d, dataEnd, label, at); break;
      case 8: {
        const sub = r.u32(d);
        const s = r.str(d + 4, 1 << 20);
        const text = s?.text ?? label.replace(/^(Call|Run)\s+/i, '');
        const callee = text.match(/^\s*([^\s(]+)\s*(\((.*)\))?\s*$/);
        const name = callee ? callee[1] : text;
        const found = callables.get(name) ?? null;
        const target = callee && !callee[2] ? found : null;
        const py = found && found.type === ItemType.PROGRAM_PYTHON ? found : null;
        if (sub === 0) data = target instanceof Program ? { kind: 'call', programId: target.id, programName: target.name } : { kind: 'code', code: text, asFunctionCall: true };
        else if (sub === 1) data = { kind: 'code', code: text, asFunctionCall: false };
        else if (sub === 2) { const t = target ?? py; data = t ? { kind: 'thread', programId: t.id, programName: t.name } : { kind: 'code', code: text, asFunctionCall: true }; }
        else if (sub === 3) data = { kind: 'print', message: text, isComment: true };
        else if (sub === 4) data = { kind: 'print', message: text, isComment: false };
        break;
      }
      case 9: { const s = r.str(d, 1 << 20); data = { kind: 'print', message: s?.text ?? label.replace(/^Print:\s*/i, ''), isComment: false }; break; }
      case 10: data = { kind: 'rounding', radius: r.f64(d) }; break;
    }
    if (!data) { data = { kind: 'print', message: label, isComment: true }; unknown.push(`${label} (type ${type})`); }
    program.addChild(new Instruction(data, label));
  }
  if (unknown.length) report.dropped.push(`program ${program.name}: ${unknown.length} instruction(s) kept as comments: ${unknown.slice(0, 4).join('; ')}${unknown.length > 4 ? '…' : ''}`);
  // program robot: the stored reference; RoboDK leaves it unset for programs that only call others
  const rb = robotItem instanceof Robot ? robotItem : (firstTargetRobot ? robots.find((x) => x.id === firstTargetRobot) : null) ?? (robots.length === 1 ? robots[0] : null);
  if (rb) program.setRobot(rb);
}

/**
 * Event (INS_TYPE 7): u32 action (0 attach, 1 detach, 2 show, 3 hide, 4/5 replace objects) | i32 tool | i32 frame | ...
 *   show / hide: u32 0 | u32 n | n × i32 item
 *   replace:     u32 n | n poses | u32 n | n × i32 item | u32 n | n × i32 parent
 */
function eventData(r: RdkBytes, d: number, end: number, label: string, at: (ref: number) => Item | null): InstructionData | null {
  const sub = r.u32(d);
  const tool = at(r.i32(d + 4));
  const frame = at(r.i32(d + 8));
  if (sub === 0 || sub === 1) {
    const ev: Record<string, unknown> = { kind: 'event', action: sub === 0 ? 'attach' : 'detach', objectId: null, name: tool?.name ?? label.replace(/^(Attach to|Detach from)\s+/i, '') };
    if (tool) ev.toolId = tool.id;
    if (sub === 1 && frame) ev.frameId = frame.id;
    return ev as unknown as InstructionData;
  }
  if (sub === 2 || sub === 3) {
    const n = r.u32(d + 16);
    const ids: string[] = [];
    const names: string[] = [];
    for (let k = 0; k < n && r.ok(d + 20 + 4 * k, 4) && d + 20 + 4 * k < end; k++) { const it = at(r.i32(d + 20 + 4 * k)); if (it) { ids.push(it.id); names.push(it.name); } }
    const ev: Record<string, unknown> = { kind: 'event', action: sub === 2 ? 'show' : 'hide', objectId: ids[0] ?? null, name: ids.length === 1 ? names[0] : 'objects' };
    if (ids.length > 1) ev.objectIds = ids;
    return ev as unknown as InstructionData;
  }
  if (sub === 4 || sub === 5) {
    const n = r.u32(d + 12);
    if (n > 100000 || !r.ok(d + 16, n * 128 + 8 + 8 * n)) return { kind: 'event', action: 'custom', objectId: null, name: 'Replace objects' };
    let o = d + 16 + n * 128;
    const n2 = r.u32(o); o += 4;
    const refs: number[] = [];
    for (let k = 0; k < n2; k++) refs.push(r.i32(o + 4 * k));
    o += 4 * n2;
    const n3 = r.u32(o); o += 4;
    const parents: number[] = [];
    for (let k = 0; k < n3 && r.ok(o + 4 * k, 4); k++) parents.push(r.i32(o + 4 * k));
    const restore: Array<{ id: string; parentId: string | null; pose: number[] }> = [];
    for (let k = 0; k < Math.min(n, refs.length); k++) {
      const it = at(refs[k]);
      if (!it || it.type === ItemType.STATION) continue;
      const par = at(parents[k] ?? -1);
      restore.push({ id: it.id, parentId: par?.id ?? null, pose: Array.from(r.pose(d + 16 + k * 128)) });
    }
    return { kind: 'event', action: 'custom', objectId: null, name: 'Replace objects', restore } as unknown as InstructionData;
  }
  return null;
}
