/**
 * RoboDK instruction decoding against a station RoboDK 5.9.4 made itself (tests/fixtures/robodk_gt/synthetic,
 * built by make_synthetic.py through the official API). The Library examples contain no IO, waits, analog outputs,
 * "Stop" pauses, joint speeds or Cyrillic tool names; this fixture pins every one of them to the values the API call
 * wrote, so the decoded program is exactly what RoboDK simulates and posts.
 */
import { describe, it, expect } from 'vitest';
import { readFileSync } from 'node:fs';
import { importRdkBestEffort } from '../src/io/robodk/rdk_container';
import { AssetStore } from '../src/scene/assets';
import { Robot } from '../src/core/items/robot';
import { Program } from '../src/core/items/program';
import type { InstructionData, MoveInstruction } from '../src/core/items/program';
import { Item, ItemType, Station, Target, Tool } from '../src/core/items/item';
import { getPos, mul, rotx, roty, transl } from '../src/core/math/pose';
import type { Mat4 } from '../src/core/math/pose';

const FILE = 'tests/fixtures/robodk_gt/synthetic/T1-instructions.rdk';

async function load(): Promise<{ station: Station; report: Awaited<ReturnType<typeof importRdkBestEffort>>['report'] }> {
  const b = readFileSync(FILE);
  return importRdkBestEffort(b.buffer.slice(b.byteOffset, b.byteOffset + b.byteLength), 'T1-instructions.rdk', new AssetStore());
}
const near = (a: Mat4, b: Mat4) => Array.from(a).every((v, i) => Math.abs(v - b[i]) < 1e-6);
const named = (station: Station, type: ItemType, name: string) => [...station.walk()].filter((i) => i.type === type && i.name === name);

describe('RoboDK-made synthetic station (IO, waits, speeds, calls, inline moves, Cyrillic names)', () => {
  it('decodes every instruction to the value the RoboDK API wrote', async () => {
    const { station, report } = await load();
    expect(report.fidelity).toBe('full');
    expect(report.dropped).toEqual([]);
    expect(station.name).toBe('T1 Синтетика');
    const robot = named(station, ItemType.ROBOT, 'ABB IRB 1600-6/1.45')[0] as Robot;
    const tool = named(station, ItemType.TOOL, 'Инструмент 1')[0] as Tool;
    expect(tool.parent).toBe(robot);
    expect(near(tool.poseTool(), mul(transl(0, 0, 150), roty(Math.PI / 6)))).toBe(true);
    const frameA = named(station, ItemType.FRAME, 'Рамка 1')[0], frameB = named(station, ItemType.FRAME, 'Frame B')[0];
    const tgt = (name: string, frame: Item) => named(station, ItemType.TARGET, name).find((t) => t.parent === frame) as Target;
    const prog = named(station, ItemType.PROGRAM, 'Программа')[0] as Program;
    const sub = named(station, ItemType.PROGRAM, 'Подпрограмма')[0] as Program;
    expect(prog.robot()).toBe(robot);
    const d: InstructionData[] = prog.instructions().map((i) => i.data);
    expect(d.length).toBe(29);
    const mv = (k: number) => d[k] as MoveInstruction;
    expect(d[0]).toEqual({ kind: 'frame', frameId: frameA.id });
    expect(d[1]).toEqual({ kind: 'tool', toolId: tool.id });
    expect(d[2]).toEqual({ kind: 'speed', speedLinear: 150, accelLinear: 500, speedJoints: 90, accelJoints: 300 });
    expect(d[3]).toEqual({ kind: 'rounding', radius: 5 });
    expect(mv(4)).toMatchObject({ moveType: 'MoveJ', targetId: tgt('Дом', frameA).id });
    expect(tgt('Дом', frameA).isJointTarget).toBe(true);
    expect(tgt('Дом', frameA).joints).toEqual([0, 10, 10, 0, 40, 0]);
    expect(mv(5)).toMatchObject({ moveType: 'MoveL', targetId: tgt('Цель 1', frameA).id });
    expect(mv(6)).toMatchObject({ moveType: 'MoveC', viaTargetId: tgt('Target 2', frameA).id, targetId: tgt('Target 3', frameA).id });
    expect(d[7]).toEqual({ kind: 'pause', timeMs: 500 });
    expect(d[8]).toEqual({ kind: 'pause', timeMs: -1 });
    expect(d[9]).toEqual({ kind: 'io', io: 'DO_5', value: 1, wait: false });
    expect(d[10]).toEqual({ kind: 'io', io: 'AO_1', value: 2.5, wait: false, analog: true });
    expect(d[11]).toEqual({ kind: 'io', io: 'DI_3', value: 1, wait: true, timeoutMs: 2000 });
    expect(d[12]).toEqual({ kind: 'io', io: 'DI_4', value: 0, wait: true });
    expect(d[13]).toEqual({ kind: 'code', code: 'SetRPM(70)', asFunctionCall: true });
    expect(d[14]).toEqual({ kind: 'call', programId: sub.id, programName: 'Подпрограмма' });
    expect(d[15]).toEqual({ kind: 'thread', programId: sub.id, programName: 'Подпрограмма' });
    // a call naming a station Python program stays a code call; the runtime resolves it to the PROGRAM_PYTHON item (I5)
    expect(d[16]).toEqual({ kind: 'code', code: 'ПитонПрог', asFunctionCall: true });
    expect(d[17]).toEqual({ kind: 'code', code: 'LaserOn', asFunctionCall: false });
    expect(d[18]).toEqual({ kind: 'print', message: 'Комментарий', isComment: true });
    expect(d[19]).toEqual({ kind: 'print', message: 'Hello operator', isComment: false });
    expect(d[20]).toEqual({ kind: 'frame', frameId: frameB.id });
    // duplicate target name: the stored reference picks the one in Frame B
    expect(mv(21)).toMatchObject({ moveType: 'MoveL', targetId: tgt('Target 2', frameB).id });
    expect(getPos(tgt('Target 2', frameB).pose())).toEqual([50, 0, 0]);
    // inline moves keep their own joints / pose (no target item)
    expect(mv(22)).toMatchObject({ moveType: 'MoveJ', targetId: null, joints: [10, -10, 20, 0, 40, 0] });
    expect(mv(22).pose).toBeUndefined();
    expect(mv(23).targetId).toBeNull();
    expect(near(Float64Array.from(mv(23).pose!), mul(transl(40, 20, 0), rotx(Math.PI)))).toBe(true);
    expect(d[24]).toEqual({ kind: 'rounding', radius: -1 });
    expect(d[25]).toEqual({ kind: 'speed', speedJoints: 45 });
    expect(d[26]).toEqual({ kind: 'speed', accelJoints: 200 });
    expect(d[27]).toEqual({ kind: 'speed', accelLinear: 800 });
    expect(mv(28)).toMatchObject({ moveType: 'MoveJ', targetId: tgt('Дом', frameA).id });
  });

  it('decodes the collision map (robot links + objects, checked pairs) of a RoboDK-made station', async () => {
    // T1-collision.rdk: ABB IRB 1600 + box "B", setCollisionActivePair(OFF, robot, box, link 0..3) (make_synthetic.py)
    const b = readFileSync('tests/fixtures/robodk_gt/synthetic/T1-collision.rdk');
    const { station } = await importRdkBestEffort(b.buffer.slice(b.byteOffset, b.byteOffset + b.byteLength), 'T1-collision.rdk', new AssetStore());
    const map = station.params.collisionMap as { entities: Array<{ id: string; name: string; link?: number }>; pairs: Array<[number, number]> };
    expect(map.entities.map((e) => [e.name, e.link])).toEqual([0, 1, 2, 3, 4, 5, 6].map((k) => ['ABB IRB 1600-6/1.45', k] as [string, number | undefined]).concat([['B', undefined]]));
    // RoboDK default for a robot: every pair of links except adjacent ones; the box is checked against links 4..6 only
    const self: Array<[number, number]> = [];
    for (let i = 0; i < 7; i++) for (let j = i + 2; j < 7; j++) self.push([i, j]);
    expect(map.pairs).toEqual([...self, [4, 7], [5, 7], [6, 7]].sort((x, y) => x[0] - y[0] || x[1] - y[1]));
  });

  it('imports the station Python program and station parameters', async () => {
    const { station } = await load();
    const py = named(station, ItemType.PROGRAM_PYTHON, 'ПитонПрог');
    expect(py.length).toBe(1);
    // exact bytes RoboDK stored (make_synthetic.py wrote the script on Windows, hence CRLF)
    expect(py[0].params.pythonCode).toBe('# Скрипт станции\r\nfrom robodk.robolink import *\r\nRDK = Robolink()\r\nprint("hello")\r\n');
    expect(station.params.T1_PARAM).toBe('42');
  });
});
