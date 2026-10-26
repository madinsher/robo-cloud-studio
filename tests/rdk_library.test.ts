/**
 * RoboDK Library acceptance checks that the per-file ground-truth comparison (robodk_gt.test.ts) does not express
 * directly: full mesh coverage, MoveC via points, inline machining moves kept apart, frame changes resolved by
 * reference, unknown routines kept as code calls, threads, station parameters, synchronised external axes,
 * save/reload of imported geometry and linear-time import. Skipped (visibly) when C:/RoboDK/Library is absent.
 */
import { describe, it, expect } from 'vitest';
import { readFileSync, existsSync } from 'node:fs';
import { importRdkBestEffort } from '../src/io/robodk/rdk_container';
import { AssetStore } from '../src/scene/assets';
import { Robot } from '../src/core/items/robot';
import { Program } from '../src/core/items/program';
import type { MoveInstruction } from '../src/core/items/program';
import { ItemType, Station, Target } from '../src/core/items/item';
import { saveStation, loadStation } from '../src/io/station-file';

const LIB = 'C:/RoboDK/Library';
const has = (f: string) => existsSync(`${LIB}/${f}`);
async function open(f: string, assets = new AssetStore()) {
  const b = readFileSync(`${LIB}/${f}`);
  const t0 = Date.now();
  const res = await importRdkBestEffort(b.buffer.slice(b.byteOffset, b.byteOffset + b.byteLength), f, assets);
  return { ...res, ms: Date.now() - t0, assets };
}
const progs = (s: Station) => s.itemsOfType<Program>(ItemType.PROGRAM);
const prog = (s: Station, name: string) => progs(s).find((p) => p.name === name)!;
const triangles = (r: Robot, assets: AssetStore) => r.chain.links.reduce((n, l) => n + l.visuals.reduce((m, v) => m + (assets.get(v.mesh ?? '')?.mesh?.triangles ?? 0), 0), 0);

describe('RoboDK Library acceptance', () => {
  // triangle totals counted by an independent walker of the files (investigation report, meshwalk_all.txt)
  const MESH: Array<[string, number]> = [['UR5e.robot', 264588], ['KUKA-KR-6-R900-sixx.robot', 32463], ['Mecademic-Meca500-R3.robot', 71747], ['RobotiQ-2F85-Gripper.robot', 38022], ['ABB-IRB-6700-150-3-20-LeanID.robot', 445929]];
  for (const [f, n] of MESH) {
    it.skipIf(!has(f))(`keeps every link mesh triangle of ${f} (${n})`, async () => {
      const { station, assets } = await open(f);
      const robot = station.itemsOfType<Robot>(ItemType.ROBOT)[0];
      expect(triangles(robot, assets)).toBe(n);
      // every actuated link carries geometry (the 2F-85 4-bar linkage is drawn on the gripper body: class 17)
      if (robot.params.rdkClass !== 17) for (let i = 0; i <= Math.min(robot.dof, 6); i++) expect(robot.chain.links[i].visuals.length, `link ${i}`).toBeGreaterThan(0);
    }, 120_000);
  }

  it.skipIf(!has('Example-02.b-Robot Machining Mold - Motoman.rdk'))('02.b mold: 2516 MoveC with their own via and end poses, imported in linear time', async () => {
    const { station, ms } = await open('Example-02.b-Robot Machining Mold - Motoman.rdk');
    const moves = prog(station, 'Mold').instructions().map((i) => i.data).filter((d): d is MoveInstruction => d.kind === 'move');
    const circ = moves.filter((m) => m.moveType === 'MoveC');
    expect(circ.length).toBe(2516);
    expect(circ.every((m) => m.pose?.length === 16 && m.viaPose?.length === 16 && m.viaPose.some((v, i) => v !== m.pose![i]))).toBe(true);
    // the old label-driven importer needed 56.8 s (quadratic); the structural decoder is linear
    expect(ms).toBeLessThan(15_000);
  }, 120_000);

  it.skipIf(!has('Example-02.g-Robot Machining Tour Eiffel - Staubli.rdk'))('02.g Tour Eiffel: inline machining moves keep their own poses', async () => {
    const { station } = await open('Example-02.g-Robot Machining Tour Eiffel - Staubli.rdk');
    const moves = progs(station).flatMap((p) => p.instructions().map((i) => i.data)).filter((d): d is MoveInstruction => d.kind === 'move');
    expect(moves.length).toBeGreaterThanOrEqual(8962);
    const distinct = new Set(moves.map((m) => (m.pose ?? m.joints ?? []).map((v) => v.toFixed(3)).join(',')));
    expect(distinct.size).toBeGreaterThanOrEqual(8900);
  }, 120_000);

  it.skipIf(!has('Example-04.a-3D printing Material Deposition.rdk'))('04.a 3D printing: renamed frame resolved by reference, extruder calls kept as code', async () => {
    const { station } = await open('Example-04.a-3D printing Material Deposition.rdk');
    const p = progs(station).sort((a, b) => b.instructions().length - a.instructions().length)[0];
    const first = p.instructions()[0].data;
    expect(first.kind).toBe('frame');
    expect(station.findById((first as { frameId: string }).frameId)?.name).toBe('Reference');
    const extruder = p.instructions().filter((i) => i.data.kind === 'code' && /^Extruder\(/.test(i.data.code) && i.data.asFunctionCall);
    expect(extruder.length).toBeGreaterThanOrEqual(3500);
  }, 120_000);

  it.skipIf(!has('Example-08.a-Synchronize 3 Robots with a Screen (GUI).rdk'))('08.a: programs run on their stored robots, the main program starts 3 threads', async () => {
    const { station } = await open('Example-08.a-Synchronize 3 Robots with a Screen (GUI).rdk');
    expect(['Prog1', 'Prog2', 'Prog3'].map((n) => prog(station, n).robot()?.name)).toEqual(['R1', 'R2', 'R3']);
    const threads = progs(station).flatMap((p) => p.instructions()).filter((i) => i.data.kind === 'thread').map((i) => (i.data as { programName?: string }).programName);
    expect(threads).toEqual(expect.arrayContaining(['Prog1', 'Prog2', 'Prog3']));
    // Prog2's "Target 1" is R2's own target, not R1's of the same name
    const t = prog(station, 'Prog2').instructions().map((i) => i.data).find((d): d is MoveInstruction => d.kind === 'move' && !!d.targetId)!;
    expect((station.findById(t.targetId!) as Target).robotId).toBe(prog(station, 'Prog2').robot()!.id);
  }, 120_000);

  it.skipIf(!has('Example-05.b-Glazing with a turntable - ABB.rdk'))('05.b: station parameters and non-ASCII names', async () => {
    const { station } = await open('Example-05.b-Glazing with a turntable - ABB.rdk');
    expect([station.params.IO_5, station.params.GUN_OUT, station.params.GUN_IO]).toEqual(['1', '23', '23']);
    expect(progs(station).some((p) => p.name === 'Sıde0')).toBe(true);
    expect(station.itemsOfType(ItemType.FRAME).some((f) => f.name === 'SıdeSurface180')).toBe(true);
  }, 120_000);

  it.skipIf(!has('Example-05.c-Robot machining with a rail and turntable - ABB.rdk'))('05.c: the arm is synchronised with its rail (carrier) and turntable (positioner)', async () => {
    const { station } = await open('Example-05.c-Robot machining with a rail and turntable - ABB.rdk');
    const arm = station.itemsOfType<Robot>(ItemType.ROBOT).find((r) => r.name === 'ABB IRB 6640-130/3.2')!;
    const sync = arm.params.syncAxes as Array<{ id: string; name: string; role: string; joints: number[] }>;
    expect(sync.map((s) => [s.role, s.joints])).toEqual([['carrier', [6]], ['positioner', [7]]]);
    expect(arm.externalAxes).toEqual(sync.map((s) => s.id));
    expect(prog(station, 'Pillar').robot()).toBe(arm);
    // 8-axis inline moves carry the rail and turntable values after the arm joints
    const m = prog(station, 'Pillar').instructions().map((i) => i.data).find((d): d is MoveInstruction => d.kind === 'move' && !!d.joints)!;
    expect(m.joints!.length).toBe(8);
  }, 120_000);

  it.skipIf(!has('Tutorial-UR-Painting.rdk'))('an imported station saves and reloads with all its geometry', async () => {
    const { station, assets } = await open('Tutorial-UR-Painting.rdk');
    const file = JSON.parse(JSON.stringify(saveStation(station, assets)));
    const fresh = new AssetStore();
    const back = loadStation(file, fresh);
    const tri = (s: Station, a: AssetStore) => [...s.walk()].flatMap((it) => [...((it as any).geometry ?? []), ...(it instanceof Robot ? it.chain.links.flatMap((l) => l.visuals) : [])]).map((g: { mesh?: string }) => a.get(g.mesh ?? '')?.mesh?.triangles ?? -1);
    const before = tri(station, assets), after = tri(back, fresh);
    // one mesh per colour and link / object (39 for this station)
    expect(before.length).toBeGreaterThan(30);
    expect(before.every((n) => n > 0)).toBe(true);
    expect(after).toEqual(before);
  }, 120_000);
});
