// @vitest-environment jsdom
/**
 * The Control tab's station host over the real App: programs start on the app's station runtime (I1, `app.runtime`),
 * guards and sensors read one merged signal table (runtime signals + process-component signals), and a run started
 * with "Run on station" ends with the verdict of the executor.
 */
import { describe, it, expect, beforeAll } from 'vitest';
import type { App } from '../src/app';
import { ItemType } from '../src/core/items/item';
import type { Robot } from '../src/core/items/robot';
import { addControlModel } from '../src/ctl/model';
import { stationWorld, startModelRun, openGardenStation, startGroupFsmRun } from '../src/ui/control_ui';
import { installDomStubs, makeApp } from './ui_harness';

let app: App;
beforeAll(() => { installDomStubs(); app = makeApp('pickplace'); });

describe('Control tab host on the app', () => {
  it('the world senses process-component signals and runtime signals; programs start on app.runtime', () => {
    const w = stationWorld(app); expect(w.programs.kind).toBe('station-runtime');
    app.processSim.setSignal('part_present', true); app.runtime.signals.set('GripperClosed', 1);
    const env = w.sense!(); expect(env.part_present).toBe(true); expect(env.GripperClosed).toBe(1); expect(env.program_playing).toBe(false);
    w.programs.signals.set('part_present', false); expect(app.processSim.signals.get('part_present')).toBe(false); // a component signal written by a model reaches the component
  });
  it('Run on station: a Petri transition guarded by a component signal starts the robot program only after the signal rises', () => {
    app.processSim.setSignal('part_present', false);
    const prog = app.station.itemsOfType(ItemType.PROGRAM)[0]; const robot = app.station.itemsOfType<Robot>(ItemType.ROBOT)[0];
    const m = addControlModel(app.station, 'petri', 'Guarded pick', `petri g\nplace wait tokens=1\nplace done\ntransition pick guard="part_present"\narc wait -> pick -> done\naction pick program="${prog.name}"\nfinal done`);
    const run = startModelRun(app, m, { maxSeconds: 300 }); const q0 = robot.joints();
    for (let i = 0; i < 20; i++) run.hook(0.05);
    expect(run.exec!.summary().fired).toBe(0); expect(robot.joints()).toEqual(q0); expect(app.runtime.runs.some((r) => r.program === prog)).toBe(false);
    app.processSim.setSignal('part_present', true);
    for (let i = 0; i < 6000 && !run.done; i++) run.hook(0.05);
    expect(run.done).toBe(true); expect(run.exec!.summary().verdict).toBe('done');
    expect(app.runtime.runs.filter((r) => r.program === prog).map((r) => r.status)).toEqual(['done']);
    expect(robot.joints().some((v, k) => Math.abs(v - q0[k]) > 1e-3) || app.runtime.runs.some((r) => r.program === prog && (r.endTime ?? 0) > (r.startTime ?? 0))).toBe(true);
  });
  it('Group examples › Garden robots: a new station whose group FSM runs on the fleet (contract won by the closest sprayer)', () => {
    const n0 = app.stations.length; const m = openGardenStation(app); expect(app.stations.length).toBe(n0 + 1); expect(app.station.itemsOfType(ItemType.MOBILE_ROBOT).length).toBe(3);
    const run = startGroupFsmRun(app, m); for (let i = 0; i < 6000 && !run.done; i++) run.hook(0.05);
    expect(run.done).toBe(true); expect(run.status()).toMatch(/spray_S1: COMPLETED/); expect(app.logs.some((l) => /contracts spray_S1 COMPLETED \(G2\)/.test(l.text))).toBe(true);
  });
});
