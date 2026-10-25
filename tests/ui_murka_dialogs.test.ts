// @vitest-environment jsdom
/**
 * V2 / V3 findings on File › MURKA examples:
 *  - picking the MURKA_Simulation folder (as the dialog suggests) read every file of it (19 653 files, 17.5 GB with the
 *    VLA checkpoints): only the chosen project's Webots / configuration files are read now;
 *  - the retail dialog could not pass task arguments, offered tasks the MURKA_Simulation copy does not have, missed
 *    murka_classic and rebuilt the store for every task;
 *  - the S1 benchmark was computed but never animated, and level D was not offered.
 */
import { describe, it, expect, beforeAll } from 'vitest';
import { installDomStubs, makeApp } from './ui_harness';
import { selectProjectFiles, retailTaskOptions, RETAIL_TASKS, playS1Timeline } from '../src/ui/murka_ui';
import { RETAIL_VARIANTS } from '../src/murka/retail/station';

beforeAll(() => installDomStubs());
const file = (rel: string, size = 100) => { const f = new File([new Uint8Array(0)], rel.split('/').pop()!); Object.defineProperty(f, 'webkitRelativePath', { value: rel }); Object.defineProperty(f, 'size', { value: size }); return f; };

describe('MURKA folder reading', () => {
  it('the MURKA_Simulation root: only the chosen project, Webots / config files, no checkpoints or datasets', () => {
    const files = [
      file('MURKA_Simulation/robot-retailer-murka-v2/ros2_ws/src/murom_webots/worlds/retail_store.wbt'),
      file('MURKA_Simulation/robot-retailer-murka-v2/ros2_ws/src/murom_webots/protos/Murom.proto'),
      file('MURKA_Simulation/robot-retailer-murka-v2/ros2_ws/src/murom_world/config/products.yaml'),
      file('MURKA_Simulation/robot-retailer-murka-v2/ros2_ws/src/murom_description/meshes/torso_link.stl', 2e6),
      file('MURKA_Simulation/robot-retailer-murka/ros2_ws/src/murom_webots/worlds/retail_store.wbt'),
      file('MURKA_Simulation/simulation_ws/src/murka_vla_study/checkpoints/model.safetensors', 865e6),
      file('MURKA_Simulation/simulation_ws/src/murka_vla_study/datasets/ep0001/frame_0001.png', 2e5),
      file('MURKA_Simulation/robot-retailer-murka-v2/ros2_ws/build/murom_webots/libx.so', 1e6),
      file('MURKA_Simulation/robot-retailer-murka-v2/ros2_ws/src/murom_webots/videos/run.mp4', 3e8),
    ];
    const keep = selectProjectFiles(files, RETAIL_VARIANTS.v2.base).map((f) => (f as any).webkitRelativePath);
    expect(keep).toEqual(files.slice(0, 4).map((f) => (f as any).webkitRelativePath));
    // a project folder picked directly (no base prefix in the paths): all its Webots / config files
    const direct = [file('murom_webots/worlds/retail_store.wbt'), file('murom_webots/protos/Murom.proto'), file('murom_webots/big.bin', 1e9)];
    expect(selectProjectFiles(direct, RETAIL_VARIANTS.v2.base).length).toBe(2);
  });
});

describe('MURKA retail dialog', () => {
  it('task lists per variant (the MURKA_Simulation copy: its e2e_test.py, depalletize / order only) and murka_classic is offered', () => {
    expect(RETAIL_TASKS.murka).toEqual(['e2e', 'depalletize', 'order']);
    expect(RETAIL_TASKS.murka_classic).toEqual(['e2e', 'depalletize', 'order']);
    // the dialog offers exactly the tasks of each project's orchestrator (+ the copy's acceptance test)
    for (const v of Object.keys(RETAIL_VARIANTS)) expect(RETAIL_TASKS[v], v).toEqual(RETAIL_VARIANTS[v as keyof typeof RETAIL_VARIANTS].tasks);
    expect(RETAIL_TASKS.v2).toEqual(expect.arrayContaining(['full', 'scan', 'depalletize', 'order', 'goto', 'dock', 'undock', 'home']));
    expect(Object.keys(RETAIL_TASKS).sort()).toEqual(Object.keys(RETAIL_VARIANTS).sort());
  });
  it('task arguments: full takes scan= / depal= / order=, other tasks positional arguments (MURKA scenario.md)', () => {
    expect(retailTaskOptions('full', 'scan=S1,S2 depal=pallet_1,box_red,bulk_B1,1 order=tea_box,coffee_can')).toEqual({ params: { scan: 'S1,S2', depal: 'pallet_1,box_red,bulk_B1,1', order: 'tea_box,coffee_can' } });
    expect(retailTaskOptions('depalletize', 'pallet_1 box_red bulk_B1 1')).toEqual({ args: ['pallet_1', 'box_red', 'bulk_B1', '1'] });
    expect(retailTaskOptions('order', ' tea_box  coffee_can ')).toEqual({ args: ['tea_box', 'coffee_can'] });
    expect(retailTaskOptions('dock', '')).toEqual({});
  });
});

describe('MURKA S1 playback', () => {
  it('the computed run plays on the world clock (timeline.apply each world step) and stops at its end', () => {
    const app = makeApp();
    const seen: number[] = [];
    playS1Timeline(app, app.station, { end: 2, apply: (t) => seen.push(t) });
    expect(app.worldRunning).toBe(true);
    for (let i = 0; i < 40; i++) (app as any).tick(0.1);
    expect(seen[0]).toBe(0);
    expect(Math.max(...seen)).toBeCloseTo(2, 6);
    expect(app.worldHooks.length).toBe(0);
  });
});
