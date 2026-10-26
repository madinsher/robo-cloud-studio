/**
 * F1 (V1 finding): a joint move with synchronised linear external axes plays at RoboDK's speed. RoboDK applies the
 * robot's joint speed / acceleration to every axis of a MoveJ, linear axes included (mm/s, mm/s²). Library 05.d (Comau
 * Smart NM 45 on a Gudel FP-6-300 gantry, speed joints 200, linear speed 16.7 mm/s): RoboDK 5.9.4 playback moves the
 * gantry 0 -> 3847 mm in about 21 s and Item.Update() reports 200.82 s for "Simulation"; the studio limited the gantry
 * by the linear speed (231 s for the MoveJ, 621.7 s for the program).
 */
import { describe, it, expect } from 'vitest';
import { existsSync, readFileSync } from 'node:fs';
import { importRdkBestEffort } from '../src/io/robodk/rdk_container';
import { StationRuntime } from '../src/core/motion/runtime';
import { AssetStore } from '../src/scene/assets';
import { Program } from '../src/core/items/program';
import { Robot } from '../src/core/items/robot';
import { Station, ItemType } from '../src/core/items/item';
import { createRobotFromLibrary } from '../src/core/items/library';
import { MotionGroup } from '../src/core/motion/planner';

const FILE = 'C:/RoboDK/Library/Example-05.d-H bot gantry - Comau.rdk';

describe('joint moves with linear external axes (RoboDK speed rule)', () => {
  it('MotionGroup.jointSpeed: the joint speed applies to linear axes too', () => {
    const st = new Station('s');
    const r = createRobotFromLibrary('UR10e');
    st.addChild(r);
    const g = new MotionGroup(r);
    const s = { speedLinear: 16.7, speedJoints: 200, accelLinear: 1500, accelJoints: 150, rounding: -1 };
    (g as any).prismatic[0] = true;               // a linear axis in the group, no own speed limit
    (g as any).vmax[0] = Infinity;
    expect(g.jointSpeed(0, s)).toBe(200);         // not the linear speed 16.7 mm/s
    expect(g.jointSpeed(1, s)).toBe(Math.min(200, (g as any).vmax[1]));
  });

  it.skipIf(!existsSync(FILE))('05.d Simulation: the gantry MoveJ takes ~21 s and the program 200.8 s like RoboDK', async () => {
    const b = readFileSync(FILE);
    const assets = new AssetStore();
    const { station } = await importRdkBestEffort(b.buffer.slice(b.byteOffset, b.byteOffset + b.byteLength) as ArrayBuffer, 'Example-05.d-H bot gantry - Comau', assets);
    const p = station.itemsOfType<Program>(ItemType.PROGRAM).find((x) => x.name === 'Simulation')!;
    const gantry = station.itemsOfType<Robot>(ItemType.ROBOT).find((r) => /Gudel/.test(r.name))!;
    const rt = StationRuntime.of(station);
    rt.assets = assets;
    rt.sink.message = () => {};
    const h: any = rt.start(p, { robot: p.robot()! });
    let reached = -1;
    for (let t = 0; t < 1000 && h.status === 'running'; t += 0.05) {
      rt.tick(0.05);
      if (reached < 0 && gantry.joints()[0] > 3847.7 - 0.5) reached = rt.time;
    }
    expect(h.status).toBe('done');
    // RoboDK 5.9.4 playback at simulation speed 1: the gantry at 3830.7 mm after 20.7 s (move ~21 s)
    expect(reached).toBeGreaterThan(18);
    expect(reached).toBeLessThan(23);
    // the program's cycle: RoboDK Item.Update() 200.82 s (no speed instructions: playback = Update here)
    expect(h.endTime - h.startTime).toBeGreaterThan(200.82 * 0.97);
    expect(h.endTime - h.startTime).toBeLessThan(200.82 * 1.03);
  }, 120000);
});
