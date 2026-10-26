/**
 * Control models deployed on the station runtime (I1, shared clock):
 *  - the executors start robot programs through StationRuntime: programs of different robots run concurrently;
 *  - the generated per-robot handshake programs (vendor.ts) run on the station runtime and the executor drives them as
 *    the cell controller (START_x / DONE_x in the runtime signal table) — the same cycle as direct execution;
 *  - the same programs posted in vendor languages (ABB RAPID, Fanuc LS) with numeric IO.
 * Assertions are about robot motion, timing and signals.
 */
import { describe, it, expect } from 'vitest';
import { demos } from '../src/demos';
import { ItemType, Station } from '../src/core/items/item';
import type { Robot } from '../src/core/items/robot';
import { Program } from '../src/core/items/program';
import { ProgramSimulator } from '../src/core/motion/simulator';
import { StationRuntime } from '../src/core/motion/runtime';
import { parsePetri, parseDes } from '../src/ctl/dsl';
import { PetriExecutor, AutomatonExecutor } from '../src/ctl/exec';
import { stationRuntimeBackend, handshakeBackend, ProgramBackend } from '../src/ctl/host';
import { makeStationWorld } from '../src/ctl/world';
import { buildHandshakePrograms, removeHandshakePrograms, vendorPrograms, controlOperations } from '../src/ctl/codegen/vendor';

const welding = (): Station => demos.find((d) => d.id === 'welding')!.build();
const diff = (a: number[], b: number[]) => Math.max(...a.map((v, i) => Math.abs(v - b[i])));
const progDuration = (st: Station, name: string) => { const s = new ProgramSimulator(st); s.collisionOptions = { enabled: false }; s.compile(st.itemsOfType<Program>(ItemType.PROGRAM).find((p) => p.name === name)!); return s.duration; };
function drive(x: { tick(dt: number): string; status: string }, robots: Robot[], seconds: number, dt = 0.05) {
  const prev = robots.map((r) => r.joints()); const moving: number[][] = robots.map(() => []); let t = 0;
  for (let i = 0; i < seconds / dt && x.status === 'running'; i++) { x.tick(dt); t += dt; robots.forEach((r, k) => { const q = r.joints(); if (diff(q, prev[k]) > 1e-6) moving[k].push(t); prev[k] = q; }); }
  return moving.map((m) => (m.length ? { first: m[0], last: m[m.length - 1] } : null));
}
const FORK_JOIN = `petri Weld both
place start tokens=1
place pL
place pR
place dL
place dR
place done
transition fork
transition tL
transition tR
transition join
arc start -> fork -> pL, pR
arc pL -> tL -> dL
arc pR -> tR -> dR
arc dL, dR -> join -> done
action tL program="Weld left"
action tR program="Weld right"`;

describe('executors on the station runtime (I1)', () => {
  it('Petri fork/join through StationRuntime: both robots weld at the same time, each program once, cycle ≈ the longer program', () => {
    const st = welding(); const robots = st.itemsOfType<Robot>(ItemType.ROBOT).slice(0, 2);
    const dL = progDuration(st, 'Weld left'), dR = progDuration(st, 'Weld right');
    const rt = new StationRuntime(st); const be = stationRuntimeBackend(rt); expect(be.kind).toBe('station-runtime');
    const x = new PetriExecutor({ doc: parsePetri(FORK_JOIN), world: makeStationWorld({ station: st, programs: be }) });
    const [a, b] = drive(x, robots, 150);
    expect(x.verdict).toBe('done'); expect(x.where()).toBe('done');
    expect(rt.runs.map((r) => r.program.name).sort()).toEqual(['Weld left', 'Weld right']); expect(rt.runs.every((r) => r.status === 'done')).toBe(true);
    expect(Math.min(a!.last, b!.last) - Math.max(a!.first, b!.first)).toBeGreaterThan(0.85 * Math.min(dL, dR));
    expect(x.time).toBeLessThanOrEqual(1.05 * Math.max(dL, dR) + 0.3); expect(rt.time).toBeCloseTo(x.time, 1);
  });
  it('the runtime is not advanced twice when the app also ticks it (sub-steps)', () => {
    const st = welding(); const rt = new StationRuntime(st); const be = stationRuntimeBackend(rt);
    for (let i = 0; i < 10; i++) { rt.tick(0.05); be.tick(0.05); } // the app ticked first, then the executor's world step
    expect(rt.time).toBeCloseTo(0.5, 6);
    for (let i = 0; i < 10; i++) be.tick(0.05); // executor alone
    expect(rt.time).toBeCloseTo(1.0, 6);
  });
});

describe('generated handshake programs (robot side) driven by the executor (controller side)', () => {
  it('the Petri cell deployed as IO handshakes reproduces the direct cycle: START once per operation, robots concurrent', () => {
    const st = welding(); const robots = st.itemsOfType<Robot>(ItemType.ROBOT).slice(0, 2); const n0 = st.itemsOfType(ItemType.PROGRAM).length;
    const dL = progDuration(st, 'Weld left'), dR = progDuration(st, 'Weld right');
    const h = buildHandshakePrograms(st, 'petri', FORK_JOIN, 'Weld both');
    expect(h.servers.map((s) => s.robot.name)).toEqual(['IRB 2600 L', 'M-20iA R']); expect(h.skipped).toEqual([]);
    const rt = new StationRuntime(st); rt.start(h.main);
    const starts: Record<string, number> = {}; rt.signals.on('START_tL', (v) => { if (v === 1) starts.tL = (starts.tL ?? 0) + 1; }); rt.signals.on('START_tR', (v) => { if (v === 1) starts.tR = (starts.tR ?? 0) + 1; });
    const ops = h.ops.map((o) => ({ program: o.binding.value, start: o.start, done: o.done }));
    const hs = handshakeBackend(stationRuntimeBackend(rt), ops) as ProgramBackend & { log: Array<{ t: number; op: string; event: string }> };
    const x = new PetriExecutor({ doc: parsePetri(FORK_JOIN), world: makeStationWorld({ station: st, programs: hs }) });
    const [a, b] = drive(x, robots, 150);
    expect(x.verdict).toBe('done'); expect(starts).toEqual({ tL: 1, tR: 1 });
    expect(hs.log.filter((e) => e.event === 'start').map((e) => e.op).sort()).toEqual(['Weld left', 'Weld right']);
    expect(Math.min(a!.last, b!.last) - Math.max(a!.first, b!.first)).toBeGreaterThan(0.85 * Math.min(dL, dR));
    expect(x.time).toBeLessThanOrEqual(Math.max(dL, dR) + 0.6); // handshake + one poll / wait cycle
    // four-phase handshake completed: START low, DONE low again after the robot saw START fall
    for (let i = 0; i < 10; i++) rt.tick(0.05);
    expect(rt.signals.get('START_tL')).toBe(0); expect(rt.signals.get('DONE_tL')).toBe(0); expect(rt.signals.get('DONE_tR')).toBe(0);
    // the servers keep waiting for the next START (endless loops), Main started one thread per robot
    expect(rt.active().filter((r) => r.status === 'running').length).toBeGreaterThanOrEqual(2);
    rt.stop(); removeHandshakePrograms(h); expect(st.itemsOfType(ItemType.PROGRAM).length).toBe(n0);
  });
  it('an automaton with two operations on one robot polls both START inputs (IF + wait) and serialises them', () => {
    const st = welding(); const r1 = st.itemsOfType<Robot>(ItemType.ROBOT)[0];
    const src = `des one robot
automaton A
  initial I
  marked E
  I -go-> W1
  W1 -d1-> W2
  W2 -d2-> E
uncontrollable d1 d2
action A.W1 program="Weld left" done=d1
action A.W2 program="Weld left" done=d2`;
    const h = buildHandshakePrograms(st, 'des', src, 'One');
    expect(h.ops.map((o) => o.id)).toEqual(['A_W1', 'A_W2']); expect(h.servers.length).toBe(1);
    const poll = st.itemsOfType<Program>(ItemType.PROGRAM).find((p) => p.name === 'ONR1P')!; expect(poll.instructions().map((i) => i.data.kind)).toEqual(['if', 'if', 'pause']);
    const rt = new StationRuntime(st); rt.start(h.main);
    const hs = handshakeBackend(stationRuntimeBackend(rt), h.ops.map((o) => ({ program: o.binding.value, start: o.start, done: o.done })));
    // both bindings run the same program: the handshake backend maps by program name, so give each op its own entry
    const x = new AutomatonExecutor({ doc: parseDes(src), world: makeStationWorld({ station: st, programs: hs }) });
    const q0 = r1.joints(); let moved = 0; for (let i = 0; i < 3000 && x.status === 'running'; i++) { const q = r1.joints(); x.tick(0.05); if (diff(q, r1.joints()) > 1e-6) moved++; }
    expect(x.verdict).toBe('done'); expect(x.trace.map((e) => e.event)).toEqual(['go', 'd1', 'd2']);
    expect(moved * 0.05).toBeGreaterThan(1.6 * progDuration(st, 'Weld left')); void q0;
    rt.stop(); removeHandshakePrograms(h);
  });
});

describe('vendor robot programs (posts, numeric IO)', () => {
  it('ABB RAPID and Fanuc LS servers: endless loop, wait / poll on DI, DONE handshake on DO; the station is unchanged', () => {
    const st = welding(); const n0 = st.itemsOfType(ItemType.PROGRAM).length;
    const net = FORK_JOIN.replace('action tR program="Weld right"', 'action tR program="Weld right"\naction join program="Weld left"');
    const ops = controlOperations('petri', net); expect(ops.map((o) => `${o.id}:${o.k}`)).toEqual(['tL:0', 'tR:1', 'join:2']);
    const files = vendorPrograms(st, 'petri', net, 'Weld both');
    expect(st.itemsOfType(ItemType.PROGRAM).length).toBe(n0);
    const io = files.find((f) => f.name.endsWith('_robot_io.csv'))!.content.split('\n');
    expect(io[1]).toBe('tL,1,1,START_tL,DONE_tL,IRB 2600 L,program Weld left'); expect(io[2]).toBe('tR,2,2,START_tR,DONE_tR,M-20iA R,program Weld right');
    const abb = files.find((f) => f.name.startsWith('IRB_2600_L/') && f.name.endsWith('.mod'))!.content; // the server is the module's main PROC
    expect(abb).toMatch(/PROC Main\(\)[\s\S]*SetDO D_OUT_1, 0;[\s\S]*SetDO D_OUT_3, 0;[\s\S]*WHILE TRUE DO\s+WER1P;\s+ENDWHILE/);
    expect(abb).toMatch(/PROC WER1P\(\)[\s\S]*IF D_IN_1 = 1 THEN\s+WER1O1;\s+ENDIF[\s\S]*IF D_IN_3 = 1 THEN\s+WER1O3;\s+ENDIF[\s\S]*WaitTime 0\.050;/);
    expect(abb).toMatch(/PROC WER1O1\(\)[\s\S]*Weld_?left;\s+SetDO D_OUT_1, 1;\s+WaitDI D_IN_1, 0;\s+SetDO D_OUT_1, 0;/);
    const fan = Object.fromEntries(files.filter((f) => f.name.startsWith('M_20iA_R/')).map((f) => [f.name.slice(9), f.content]));
    expect(fan['WER2.LS']).toMatch(/DO\[2\]=OFF ;[\s\S]*LBL\[1\] ;[\s\S]*CALL WER2W ;[\s\S]*JMP LBL\[1\] ;/);
    expect(fan['WER2W.LS']).toMatch(/WAIT DI\[2\]=ON ;[\s\S]*CALL WER2O2 ;/);
    expect(fan['WER2O2.LS']).toMatch(/CALL Weld_?right ;[\s\S]*DO\[2\]=ON ;[\s\S]*WAIT DI\[2\]=OFF ;[\s\S]*DO\[2\]=OFF ;/);
    const sub = Object.keys(fan).find((k) => /^Weld_?right\.LS$/i.test(k))!; expect(fan[sub]).toMatch(/\/PROG\s+Weld_?right/); // the operation's own program is posted too (name per the Fanuc post)
  });
});
