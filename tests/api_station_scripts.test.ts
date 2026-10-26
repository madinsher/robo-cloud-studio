/**
 * T10 acceptance (I5): the Python programs embedded in the RoboDK Library stations run on the studio and reproduce
 * their RoboDK effects. Each station is imported from C:/RoboDK/Library (T1 importer), served by a headless studio
 * host (StationRuntime clock, CPython worker runner, unattended GUI) and driven the way a RoboDK user would: run the
 * main program (which calls the scripts with their arguments, e.g. SprayOn(1)) or double-click the script.
 * Assertions check the effect in the station: material sprayed on the part, robot paths, created items, station
 * variables. The scripted sweep over all 54 programs with the results table lives in the T10 status report.
 */
import { describe, it, expect, afterEach } from 'vitest';
import { readFileSync, existsSync, mkdtempSync } from 'node:fs';
import { tmpdir } from 'node:os';
import path from 'node:path';
import { importRdkBestEffort } from '../src/io/robodk/rdk_container';
import { unwrapRobodk } from '../src/io/robodk/rdk_binary';
import { AssetStore } from '../src/scene/assets';
import { Station, Item, ItemType, Target, SceneObject } from '../src/core/items/item';
import { Robot } from '../src/core/items/robot';
import { Program } from '../src/core/items/program';
import { createNodeHost, NodeHost } from '../src/api/node_host';
import { findPython } from '../src/api/runner_cpython';
import { RobolinkItem } from '../src/api/robolink';
import { multiply, invert, transl, rotz, getPos } from '../src/core/math/pose';
import '../src/posts/index';

const LIB = 'C:/RoboDK/Library';
let py = false;
try { findPython(); py = true; } catch { py = false; }
const have = (f: string) => py && existsSync(`${LIB}/${f}`);

/** Station variables stored in the .rdk (count + UTF-16 key/value strings ending at f64 200). Used when the importer misses them. */
async function rdkStationParams(file: string): Promise<Record<string, string>> {
  const b = readFileSync(file);
  const { bytes } = await unwrapRobodk(b.buffer.slice(b.byteOffset, b.byteOffset + b.byteLength));
  const dv = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  const str = (o: number): [string, number] | null => {
    if (o + 4 > bytes.length) return null;
    const n = dv.getUint32(o, true);
    if (n > 8000 || n % 2 || o + 4 + n > bytes.length) return null;
    let s = ''; for (let i = 0; i < n; i += 2) s += String.fromCharCode(dv.getUint16(o + 4 + i, true));
    return [s, o + 4 + n];
  };
  for (let p = 0; p < Math.min(bytes.length - 8, 20000); p++) {
    const n = dv.getUint32(p, true);
    if (n < 1 || n > 200) continue;
    let q = p + 4; const kv: Record<string, string> = {}; let ok = true;
    for (let k = 0; k < n && ok; k++) { const a = str(q); const v = a && a[0] ? str(a[1]) : null; if (!a || !a[0] || !v) { ok = false; break; } kv[a[0]] = v[0]; q = v[1]; }
    if (ok && q + 8 <= bytes.length && dv.getFloat64(q, true) === 200) return kv;
  }
  return {};
}

interface Cell { station: Station; host: NodeHost; logs: string[]; missingParams: string[] }
let open: Cell | null = null;
afterEach(async () => { if (open) { open.host.runner.stopAll(); await open.host.close(); open = null; } });

async function load(file: string, speed = 10): Promise<Cell> {
  const b = readFileSync(`${LIB}/${file}`);
  const assets = new AssetStore();
  const { station } = await importRdkBestEffort(b.buffer.slice(b.byteOffset, b.byteOffset + b.byteLength), file, assets);
  // T1 importer gap workaround: station variables the scripts read (SizeBox, PartCycles, ...)
  const missingParams: string[] = [];
  for (const [k, v] of Object.entries(await rdkStationParams(`${LIB}/${file}`))) if (station.getParam(k) == null) { station.setParam(k, v); missingParams.push(k); }
  const logs: string[] = [];
  const host = await createNodeHost({ station, assets, port: 0, tcpPort: -1, speed, scriptTimeoutS: 240, headlessGui: true, log: (m) => logs.push(m) });
  // scripts write next to the open station (CSV, images): a scratch folder, never C:/RoboDK (read-only)
  const scratch = mkdtempSync(path.join(tmpdir(), 'vbs-station-')).split(path.sep).join('/');
  host.rdk.paths.PATH_OPENSTATION = scratch;
  host.rdk.paths.FILE_OPENSTATION = `${scratch}/${file}`;
  host.rdk.paths.PATH_PROGRAMS = `${scratch}/programs`;   // MakeProgram output
  open = { station, host, logs, missingParams };
  return open;
}

/** Wait (wall clock) until nothing runs on the station runtime and no script is running. */
async function idle(c: Cell, timeoutS: number): Promise<void> {
  const t0 = Date.now();
  while (c.host.rdk.exec.busy(null) || c.host.runner.running() > 0) {
    if (Date.now() - t0 > timeoutS * 1000) throw new Error(`still running after ${timeoutS} s: ${c.logs.slice(-5).join(' | ')}`);
    await new Promise((r) => setTimeout(r, 50));
  }
}
/** Run a program like RoboDK's "Run" and wait for it and everything it started (threads, scripts). */
async function runProgram(c: Cell, name: string, timeoutS = 180, expectError?: { message: RegExp; index: number }) {
  const p = c.station.find(name, ItemType.PROGRAM);
  expect(p, `program ${name}`).toBeTruthy();
  new RobolinkItem(c.host.rdk, p).RunCode();
  const handles = c.host.rdk.exec.handlesOf(p!);
  await idle(c, timeoutS);
  const bad = handles.find((h) => h.status === 'error');
  if (expectError) {
    // a program RoboDK itself cannot complete stops at the same instruction with the same reason
    expect(bad?.error?.message ?? '', `${name} run error`).toMatch(expectError.message);
    expect(bad?.error?.instructionIndex, `${name} failing instruction`).toBe(expectError.index);
  } else expect(bad?.error?.message ?? '', `${name} run error`).toBe('');
  return handles;
}
/** Run a station Python program directly (double-click), with sys.argv arguments; returns its output. */
async function runScript(c: Cell, name: string, args: string[] = []) {
  const it = c.station.find(name, ItemType.PROGRAM_PYTHON);
  expect(it, `script ${name}`).toBeTruthy();
  const before = c.host.runner.history.length;
  await c.host.runner({ station: c.station, runtime: c.host.rdk.exec.rt() as any, name, args, robot: null, program: it! });
  await idle(c, 120);
  return c.host.runner.history.slice(before).map((h) => h.output).join('\n');
}
/** Sample the absolute TCP position of a robot while `fn` runs. */
async function sampled(robot: Robot, fn: () => Promise<unknown>): Promise<number[][]> {
  const pts: number[][] = [];
  const iv = setInterval(() => pts.push([...getPos(robot.poseTCPAbs())]), 5);
  try { await fn(); } finally { clearInterval(iv); }
  pts.push([...getPos(robot.poseTCPAbs())]);
  return pts;
}
const near = (pts: number[][], p: number[], tol: number) => Math.min(...pts.map((q) => Math.hypot(q[0] - p[0], q[1] - p[1], q[2] - p[2]))) <= tol;
/** Spray guns of the station: [id, material used %, wasted %, particles, coverage %]. */
const sprayRows = (c: Cell) => c.host.rdk.Spray_GetStats()[1] as number[][];
const calls = (c: Cell, name: string) => c.host.runner.history.filter((h) => h.name === name).map((h) => h.args.join(','));
const sec = 1000;

describe('RoboDK Library station Python programs (I5, CPython runner)', () => {
  // ---- spray / deposition stations: program calls SprayOn(1)/GunOn ... and material lands on the part ----
  // 03.a Oil: RoboDK 5.9.4 cannot complete the program as saved in the Library either: Update() validates 317 of 782
  // instructions and RunProgram stops there ("Joint 5 is too close to a singularity", joints [17.5, 63.2, -46.1,
  // -72.9, -2.5, -196.3]); GunEnd at the end of the program is never reached
  const OIL_STOP = { message: /wrist singularity/, index: 317 };
  const SPRAY: Array<[string, string, string[], number?, { message: RegExp; index: number }?]> = [
    ['Example-01.a-Robot Painting - Universal Robots.rdk', 'MainProg', ['SprayOn']],
    ['Painting-Tutorial-2.rdk', 'MainProg', ['SprayOn']],
    ['Tutorial-UR-Painting.rdk', 'MainProg', ['SprayOn']],
    ['Example-03.a-Liquid dispensing - Kawasaki.rdk', 'Oil', ['GunStart'], undefined, OIL_STOP],
    ['Example-03.h-Arc Welding - ABB.rdk', 'MainProgram', ['GunOn', 'GunOff']],
    ['Example-10.a-Shader-Customized-Light.rdk', 'MainProgram', ['GunOn', 'GunOff']],
    ['Example-05.b-Glazing with a turntable - ABB.rdk', 'Main', ['GunOn']],
    ['Additive-Manufacturing-and-Machining-with-Mecademic-Meca500-R3.rdk', 'CE3_3DBenchy', ['FilamentStart', 'FilamentEnd'], 200],
  ];
  for (const [file, prog, scripts, speed, stop] of SPRAY) {
    it.skipIf(!have(file))(`${file}: ${prog} calls ${scripts.join('/')} and sprays the part${stop ? ' until it stops where RoboDK stops' : ''}`, async () => {
      const c = await load(file, speed ?? 10);
      await runProgram(c, prog, 240, stop);
      for (const s of scripts) expect(calls(c, s).length, `${s} calls`).toBeGreaterThan(0);
      expect(c.host.runner.history.every((h) => h.code === 0)).toBe(true);
      // the gun was switched on by the script and its cone hit the part while the robot moved (material used > 0)
      const rows = sprayRows(c);
      expect(rows.length).toBeGreaterThan(0);
      expect(Math.max(...rows.map((r) => r[1]))).toBeGreaterThan(5);
      if (file.startsWith('Example-01.a') || file.startsWith('Painting') || file.startsWith('Tutorial')) {
        expect(calls(c, 'SprayOn')).toEqual(['1', '0']);
        expect([...c.host.rdk.spray.guns.values()].every((g) => !g.on)).toBe(true);   // SprayOn(0) at the end
      }
    }, 300 * sec);
  }

  it.skipIf(!have('Example-03.i-Spot Welding with a KUKA robot and HMDTech weld gun.rdk'))('03.i DoWeld(10) closes the weld gun axis to 10 mm and reopens it (home)', async () => {
    const c = await load('Example-03.i-Spot Welding with a KUKA robot and HMDTech weld gun.rdk');
    const gun = c.station.find('HMDTech_SpotWeld_Gun', ItemType.ROBOT) as Robot;
    const home = gun.jointsHome()[0];
    const seen: number[] = [];
    const iv = setInterval(() => seen.push(gun.joints()[0]), 2);
    await runScript(c, 'DoWeld', ['10']);
    clearInterval(iv);
    expect(Math.min(...seen.map((v) => Math.abs(v - 10)))).toBeLessThan(0.5);
    expect(gun.joints()[0]).toBeCloseTo(home, 6);
  }, 120 * sec);

  it.skipIf(!have('Example-06.a-Pick and place - Mecademic.rdk'))('06.a ProgramTest drives the gripper through Gripper(1)/Gripper(0)', async () => {
    const c = await load('Example-06.a-Pick and place - Mecademic.rdk');
    await runProgram(c, 'ProgramTest');
    const g = calls(c, 'Gripper');
    expect(g.length).toBeGreaterThanOrEqual(2);
    const last = Number(g[g.length - 1]) * 6;
    expect(Number(c.station.getParam('Gripper'))).toBe(last);
    expect(getPos((c.station.find('FingerB') as Item).pose())[1]).toBeCloseTo(last, 6);
    expect(getPos((c.station.find('FingerA') as Item).pose())[1]).toBeCloseTo(-last, 6);
  }, 200 * sec);

  it.skipIf(!have('Example-07.a-Weld Script Hexagon.rdk'))('07.a DoWeld welds the 200 mm hexagon around Target 1 and returns Home', async () => {
    const c = await load('Example-07.a-Weld Script Hexagon.rdk');
    const robot = c.station.itemsOfType<Robot>(ItemType.ROBOT)[0];
    const t1 = c.station.find('Target 1', ItemType.TARGET) as Target;
    const home = c.station.find('Home', ItemType.TARGET) as Target;
    const pts = await sampled(robot, () => runScript(c, 'DoWeld'));
    // TCP poses relative to the robot's reference: Target 1 pose * rotz(a) * transl(200,0,0)
    const ref = t1.parent!.poseAbs();
    for (let i = 0; i < 6; i++) {
      const a = (i * Math.PI) / 3;
      const v = getPos(multiply(ref, multiply(t1.pose(), multiply(rotz(a), transl(200, 0, 0)))));
      expect(near(pts, [...v], 1.0), `vertex ${i}`).toBe(true);
    }
    const qh = home.joints ?? robot.jointsForTarget(home)!;
    robot.joints().forEach((q, i) => expect(q).toBeCloseTo(qh[i], 3));
  }, 200 * sec);

  it.skipIf(!have('Example-07.e-Path Move_Test - Move Linear or Joints.rdk'))('07.e Make Cube Program builds the cube program from the reachable targets', async () => {
    const c = await load('Example-07.e-Path Move_Test - Move Linear or Joints.rdk');
    const before = c.station.itemsOfType<Target>(ItemType.TARGET).length;
    const out = await runScript(c, 'Make Cube Program');
    expect(out).toMatch(/Program done with 189 targets/);           // 3 x 3 x 21 candidate poses
    const created = (out.match(/Creating target \d+/g) ?? []).length;
    const skipped = (out.match(/target not reachable/g) ?? []).length;
    expect(created + skipped).toBe(189);
    expect(created).toBeGreaterThan(10);
    const targets = c.station.itemsOfType<Target>(ItemType.TARGET).slice(before);
    expect(targets.length).toBe(created);
    // the station already holds an 'AutoCreated' program from RoboDK: the script adds a new one (last in the tree)
    const prog = c.station.itemsOfType<Program>(ItemType.PROGRAM).filter((p) => p.name === 'AutoCreated').at(-1)!;
    expect(prog.instructions().filter((i) => i.data.kind === 'move').length).toBe(created + 1);
    // every created target is reachable by the robot (IK on its pose)
    const robot = c.station.itemsOfType<Robot>(ItemType.ROBOT)[0];
    for (const t of targets) expect(robot.jointsForTarget(t), t.name).toBeTruthy();
  }, 200 * sec);

  it.skipIf(!have('Example-08.c-Weld with 3 Robots (Script).rdk'))('08.c the 3-thread DoWeldAll welds with every robot and finishes at home', async () => {
    const c = await load('Example-08.c-Weld with 3 Robots (Script).rdk');
    const robots = c.station.itemsOfType<Robot>(ItemType.ROBOT);
    expect(robots.length).toBe(3);
    const out = await runScript(c, 'DoWeldAll');
    for (const r of robots) expect(out).toContain(`Robot ${r.name} finished`);
    expect(out).toContain('Main program finished');
    for (const r of robots) r.joints().forEach((q, i) => expect(q).toBeCloseTo(r.jointsHome()[i], 3));
  }, 200 * sec);

  it.skipIf(!have('Example-08.a-Synchronize 3 Robots with a Screen (Script matching timings).rdk'))('08.a (script) records relative positions, teaches targets and moves the robots in sync', async () => {
    const c = await load('Example-08.a-Synchronize 3 Robots with a Screen (Script matching timings).rdk');
    const robots = c.station.itemsOfType<Robot>(ItemType.ROBOT);
    await runScript(c, 'RecordRelativePosition');
    const keys = Object.keys(c.station.params).filter((k) => /^ShiftRel_/.test(k));
    expect(keys.length).toBe(robots.length * (robots.length - 1));
    const t0 = c.station.itemsOfType<Target>(ItemType.TARGET).length;
    await runScript(c, 'TeachTargets');
    expect(c.station.itemsOfType<Target>(ItemType.TARGET).length - t0).toBe(robots.length);
    // MoveRobots: one thread per robot, every robot visits each target of its frame, synchronised move by move
    const goals = robots.map((r) => r.parent!.children.filter((t): t is Target => t instanceof Target).map((t) => [...getPos(t.poseAbs())]));
    const paths: number[][][] = robots.map(() => []);
    const iv = setInterval(() => robots.forEach((r, i) => paths[i].push([...getPos(r.poseTCPAbs())])), 5);
    const out = await runScript(c, 'MoveRobots');
    clearInterval(iv);
    expect(out).toContain('Main program finished');
    robots.forEach((r, i) => { expect(out).toContain(`Robot ${r.name} finished`); goals[i].forEach((g, k) => expect(near(paths[i], g, 2), `${r.name} target ${k}`).toBe(true)); });
  }, 300 * sec);

  // ---- pick and place cells: MainSimulation calls PrepareSimulation / camera or sensor / robot scripts ----
  const PALLET: Array<[string, number]> = [['Example-06.c-Pick and place with 2D camera.rdk', 12], ['Example-06.d-Pick and place with laser sensor.rdk', 18]];
  for (const [file, nparts] of PALLET) {
    it.skipIf(!have(file))(`${file}: MainSimulation moves all ${nparts} parts from pallet A over the conveyor to pallet B`, async () => {
      // speed 10 (was 20): the scripts poll the camera / sensor in real time like in RoboDK; at 20x a machine loaded by
      // parallel jobs let a part run past the detection window between two polls and the pick failed (V6: intermittent)
      const c = await load(file, 10);
      const parts = () => c.station.itemsOfType<SceneObject>(ItemType.OBJECT).filter((o) => /^Part \d+$/.test(o.name));
      const p = c.station.find('MainSimulation', ItemType.PROGRAM)!;
      new RobolinkItem(c.host.rdk, p).RunCode();
      const h = c.host.rdk.exec.handlesOf(p);
      // the MoveConveyor thread runs for hours (RoboDK too): the cycle is over when MainSimulation returns
      const t0 = Date.now();
      while (h.some((x) => x.status === 'running' || x.status === 'queued') && Date.now() - t0 < 240 * sec) await new Promise((r) => setTimeout(r, 100));
      expect(h.map((x) => x.status), `${h.map((x) => x.error?.message ?? '').join(' | ')} :: ${c.host.runner.history.filter((x) => x.code !== 0).map((x) => `${x.name}: ${x.output.slice(-400)}`).join(' | ')} :: ${c.logs.slice(-8).join(' | ')}`).toEqual(['done']);
      expect(parts().length).toBe(nparts);                          // PrepareSimulation re-creates the parts (SizePallet)
      expect(parts().every((o) => o.parent?.name === 'PalletB')).toBe(true);
      expect(c.host.runner.history.every((x) => x.code === 0)).toBe(true);
    }, 300 * sec);
  }

  it.skipIf(!have('Example-07.b-Weld Script Graphical User Interface.rdk'))('07.b Move Polygon GUI (Simulate/Generate pressed) moves the robot around the 8-sided polygon', async () => {
    const c = await load('Example-07.b-Weld Script Graphical User Interface.rdk');
    const robot = c.station.itemsOfType<Robot>(ItemType.ROBOT)[0];
    const ref = c.station.find('Target Reference', ItemType.TARGET) as Target;
    const pts = await sampled(robot, () => runScript(c, 'Move Polygon GUI'));
    expect(c.logs.some((l) => /pressing "Simulate\/Generate"/.test(l))).toBe(true);
    const base = ref.parent!.poseAbs();
    for (let i = 0; i < 8; i++) {
      const a = (i * 2 * Math.PI) / 8;
      const v = getPos(multiply(base, multiply(ref.pose(), multiply(rotz(a), transl(200, 0, 0)))));
      expect(near(pts, [...v], 1.0), `polygon vertex ${i}`).toBe(true);
    }
  }, 200 * sec);

  it.skipIf(!have('Example-07.c-Script Curve Follow Custom.rdk'))('07.c CurveSetupGUI projects the grid like RoboDK and regenerates both curve follow programs', async () => {
    const c = await load('Example-07.c-Script Curve Follow Custom.rdk');
    // RoboDK's own projection result is saved in the station (curve objects under Section1/Section2): ground truth
    const gt = new Map<string, number[][]>();
    for (const s of ['Section1', 'Section2']) {
      const fr = c.station.find(s, ItemType.FRAME)!;
      gt.set(s, (fr.children.find((x) => x instanceof SceneObject) as SceneObject).curves.flatMap((k) => k.points));
    }
    await runScript(c, 'CurveSetupGUI One Pass');
    for (const s of ['Section1', 'Section2']) {
      const obj = c.station.find(s, ItemType.FRAME)!.children.find((x) => x instanceof SceneObject) as SceneObject;
      const ours = obj.curves.flatMap((k) => k.points);
      const ref = gt.get(s)!;
      expect(ours.length, s).toBe(ref.length);                       // same points kept by the script's window filter
      ours.forEach((p, i) => { for (let k = 0; k < 3; k++) expect(Math.abs(p[k] - ref[i][k]), `${s} point ${i}`).toBeLessThan(0.01); });
      // the curve follow project was re-solved into a new program on the KUKA (path reachability itself is T3's
      // curve-follow generator / simulator, reported in the T10 table, not asserted here)
      const mp = c.station.find(s, ItemType.MACHINING)!;
      const prog = c.station.findById(String(mp.getParam('programId'))) as Program;
      expect(prog?.name, s).toBe(s);
      expect(prog.robot()?.name).toBe('KUKA KR 5 arc');
      expect(prog.instructions().filter((i) => i.data.kind === 'move').length).toBeGreaterThan(5);
    }
    const main = c.station.find('MainInspection Part', ItemType.PROGRAM) as Program;
    expect(main.instructions().map((i) => (i.data as any).programName ?? (i.data as any).code)).toEqual(['Section1', 'Section2']);
  }, 200 * sec);

  it.skipIf(!have('Example-08.a-Synchronize 3 Robots with a Screen (GUI).rdk'))('08.a (GUI) the 3 robot programs synchronise through SyncR1..3 at every move', async () => {
    const c = await load('Example-08.a-Synchronize 3 Robots with a Screen (GUI).rdk');
    const ids = () => [1, 2, 3].map((k) => Number(c.station.getParam(`MOVE_ID_R${k}`) ?? 0));
    // SyncRk(0) first resets each robot's MOVE_ID (saved values differ); from then on no robot may get a full move ahead
    let maxSpread = 0;
    const reset = new Set<number>();
    const iv = setInterval(() => { const v = ids(); v.forEach((x, k) => { if (x === 0) reset.add(k); }); if (reset.size === 3) maxSpread = Math.max(maxSpread, Math.max(...v) - Math.min(...v)); }, 2);
    await runProgram(c, 'MainProgram', 180);
    clearInterval(iv);
    for (const n of ['Prog1', 'Prog2', 'Prog3']) expect(c.host.rdk.exec.handlesOf(c.station.find(n, ItemType.PROGRAM)).every((h) => h.status !== 'error')).toBe(true);
    const final = ids();
    expect(new Set(final).size).toBe(1);                           // every robot reached the last sync point
    expect(final[0]).toBeGreaterThanOrEqual(5);
    expect(maxSpread).toBeLessThanOrEqual(1);                      // no robot ran ahead of the others by a full move
  }, 200 * sec);

  it.skipIf(!have('Example-04.a-3D printing Material Deposition.rdk'))('04.a Print3D calls Extruder(e) with the program values; SimulateFinalResult replays the path', async () => {
    const c = await load('Example-04.a-3D printing Material Deposition.rdk', 400);
    const prog = c.station.find('Print3D', ItemType.PROGRAM) as Program;
    const expected = prog.instructions().map((i) => i.data as any).filter((d) => d.kind === 'code' && /^Extruder/.test(d.code)).map((d) => (d.code.match(/\((.*)\)/)?.[1] ?? ''));
    new RobolinkItem(c.host.rdk, prog).RunCode();
    const t0 = Date.now();
    while (calls(c, 'Extruder').length < 60 && Date.now() - t0 < 120 * sec) await new Promise((r) => setTimeout(r, 50));
    c.host.rdk.exec.stop();
    const got = calls(c, 'Extruder');
    expect(got.length).toBeGreaterThanOrEqual(60);
    expect(got).toEqual(expected.slice(0, got.length));             // same arguments, same order as the program
    expect(sprayRows(c).length).toBeGreaterThan(0);
    await idle(c, 60);
    const out = await runScript(c, 'SimulateFinalResult');
    expect(out).toContain('Using program Print3D');
    expect(out).toMatch(/Done/);
  }, 300 * sec);

  it.skipIf(!have('Example-09.a-Script-Model Robot DH-DHM Mirror Test - ABB IRB 120.rdk'))('09.a MirrorRobot2 keeps the absolute DH frames on the robot links while the robot moves', async () => {
    const c = await load('Example-09.a-Script-Model Robot DH-DHM Mirror Test - ABB IRB 120.rdk');
    const robot = c.station.itemsOfType<Robot>(ItemType.ROBOT)[0];
    const parent = c.station.find('Robot base (absolute frames)', ItemType.FRAME)!;
    const frames = parent.children.filter((x) => x.type === ItemType.FRAME);
    expect(frames.length).toBe(6);
    const it = c.station.find('MirrorRobot2', ItemType.PROGRAM_PYTHON)!;
    const run = c.host.runner({ station: c.station, runtime: c.host.rdk.exec.rt() as any, name: 'MirrorRobot2', args: [], robot: null, program: it }).catch((e) => e);
    for (const q of [[10, -20, 30, 40, 50, 60], [-35, 15, -10, 90, -45, 120]]) {
      robot.setJoints(q);
      await new Promise((r) => setTimeout(r, 2500));
      // the 6th DH frame is the flange of the IRB 120 (relative to the mirror base frame): it must match the robot's
      // own flange relative to its base
      const f6 = getPos(frames[5].pose()), fl = getPos(robot.solveFKFlange(q));
      expect(Math.hypot(f6[0] - fl[0], f6[1] - fl[1], f6[2] - fl[2]), `joints ${q}`).toBeLessThan(1);
    }
    c.host.runner.stopAll();
    await run;
  }, 120 * sec);

  it.skipIf(!have('Example-10.a-Shader-Customized-Light.rdk'))('10.a Camera Setup adds the two shader cameras on their frames', async () => {
    const c = await load('Example-10.a-Shader-Customized-Light.rdk');
    await runScript(c, 'Camera Setup');
    for (const f of ['Camera 1', 'Camera 2']) {
      const cams = c.station.find(f, ItemType.FRAME)!.children.filter((x) => x.type === ItemType.CAMERA) as any[];
      expect(cams.length, f).toBe(1);
      expect(cams[0].fov).toBe(32);
      expect(cams[0].far).toBe(3000);
    }
  }, 120 * sec);

  it.skipIf(!have('Example-08.a-Synchronize 3 Robots with a Screen (Script matching timings).rdk'))('08.a (script) FollowLeaderRel keeps the followers at the recorded relative pose while the leader moves', async () => {
    const c = await load('Example-08.a-Synchronize 3 Robots with a Screen (Script matching timings).rdk');
    const robots = c.station.itemsOfType<Robot>(ItemType.ROBOT);
    const leader = robots[0];                                     // headless ItemUserPick: the first robot
    await runScript(c, 'RecordRelativePosition');
    const pose = (r: Robot) => new RobolinkItem(c.host.rdk, r).Pose().m;   // TCP w.r.t. the robot's reference (RoboDK Pose)
    const rel = (r: Robot) => multiply(invert(pose(leader)), pose(r));
    const rel0 = robots.slice(1).map(rel);
    const it = c.station.find('FollowLeaderRel', ItemType.PROGRAM_PYTHON)!;
    const run = c.host.runner({ station: c.station, runtime: c.host.rdk.exec.rt() as any, name: 'FollowLeaderRel', args: [], robot: null, program: it }).catch((e) => e);
    await new Promise((r) => setTimeout(r, 1500));
    const q = leader.joints().map((v, i) => (i === 0 ? v + 4 : i === 4 ? v - 3 : v));
    leader.setJoints(q);
    await new Promise((r) => setTimeout(r, 3500));
    robots.slice(1).forEach((r, k) => {
      const a = getPos(rel(r)), b = getPos(rel0[k]);
      expect(Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]), `${r.name} relative position`).toBeLessThan(1);
    });
    c.host.runner.stopAll();
    await run;
  }, 200 * sec);

  it.skipIf(!have('Example-09.b-PointCloud faster simulation.rdk'))('09.b ShowAsPointCloud sets the point-cloud display on the selected object', async () => {
    const c = await load('Example-09.b-PointCloud faster simulation.rdk');
    const obj = c.station.itemsOfType<SceneObject>(ItemType.OBJECT)[0];
    c.station.setSelection([obj]);
    await runScript(c, 'ShowAsPointCloud');
    const d = obj.getParam('display') as any;
    expect(d?.raw).toBe('POINTSIZE=4 COLOR=#FF771111');
    expect(d.pointSize).toBe(4);
    expect(d.color).toBe('#771111');
  }, 120 * sec);
});
