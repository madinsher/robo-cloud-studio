// @vitest-environment jsdom
/**
 * F1 (V1 finding): a station Python call must not inflate the cycle time the user sees in live playback.
 *  - while the script runner is still loading its interpreter (Pyodide download), the clock of the run waiting for
 *    the script stands still (StationRuntime.holding(); App.tick): V1 measured 01.a MainProg at 99.1 s instead of 49.1 s;
 *  - a script that answers in the same page task (SprayOn, GunOn: no wait in the script) takes no simulated time: its
 *    run resumes at the call, not one animation frame later per call (04.a, 50x: one frame = 0.8 s per Extruder call);
 *  - a run that stops while its script still runs tells the runner (ctx.abort) so a background loop can end.
 */
import { describe, it, expect, beforeAll } from 'vitest';
import { installDomStubs, makeApp } from './ui_harness';
import { Item, ItemType } from '../src/core/items/item';
import { Robot } from '../src/core/items/robot';
import { Program } from '../src/core/items/program';
import { createRobotFromLibrary } from '../src/core/items/library';
import { registerScriptRunner, StationRuntime } from '../src/core/motion/runtime';

beforeAll(() => installDomStubs());

function cell(app: ReturnType<typeof makeApp>) {
  const st = app.station;
  const r = createRobotFromLibrary('UR10e');
  st.addChild(r);
  r.setJoints([0, -90, 90, -90, -90, 0]);
  const py = new Item(ItemType.PROGRAM_PYTHON, 'SprayOn');
  py.params.pythonCode = 'print(1)';
  st.addChild(py);
  const p = new Program('Main');
  st.addChild(p);
  p.setRobot(r);
  p.addInstruction({ kind: 'move', moveType: 'MoveJ', targetId: null, joints: [30, -90, 90, -90, -90, 0] } as any);
  p.addInstruction({ kind: 'code', code: 'SprayOn(1)', asFunctionCall: true } as any);
  p.addInstruction({ kind: 'move', moveType: 'MoveJ', targetId: null, joints: [60, -90, 90, -90, -90, 0] } as any);
  return { r, p };
}
/** The live animation loop: App.tick(dt) at `fps` (wall-time pacing is irrelevant here, the runner is fake). */
async function play(app: ReturnType<typeof makeApp>, h: any, dt: number, maxSteps = 4000) {
  for (let i = 0; i < maxSteps && ['queued', 'running'].includes(h.status); i++) { (app as any).tick(dt); await new Promise((r) => setTimeout(r, 0)); }
}

describe('live clock and station scripts', () => {
  it('reference: the cycle without any script runner', async () => {
    const app = makeApp();
    app.newStation();
    const { p } = cell(app);
    const h: any = app.runProgram(p);
    await play(app, h, 1 / 60);
    expect(h.status).toBe('done');
    (globalThis as any).__refCycle = h.endTime - h.startTime;
  });

  it('the clock holds while the runner loads; an instant script costs no simulated time', async () => {
    const app = makeApp();
    app.newStation();
    const { p } = cell(app);
    app.simSpeed = 50;                                    // 04.a at 50x: one frame = 0.83 s of simulation
    let loading = true;
    const calls: string[] = [];
    const runner = Object.assign(async (ctx: any) => {
      calls.push(ctx.name);
      while (loading) await new Promise((r) => setTimeout(r, 5));
      ctx.elapsed = 0;                                    // answered without waiting (like a Pyodide script with no pause)
    }, { loading: () => loading });
    const off = registerScriptRunner(runner as any);
    try {
      const h: any = app.runProgram(p);
      // play until the run waits for the script, then 200 frames while the runner "downloads"
      for (let i = 0; i < 2000 && !calls.length; i++) { (app as any).tick(1 / 60); await new Promise((r) => setTimeout(r, 0)); }
      expect(calls).toEqual(['SprayOn']);
      const rt = StationRuntime.of(app.station);
      expect(rt.holding()).toBe(true);
      const t0 = app.simTime, rt0 = rt.time;
      for (let i = 0; i < 200; i++) { (app as any).tick(1 / 60); await new Promise((r) => setTimeout(r, 0)); }
      expect(app.simTime).toBe(t0);                       // held: 200 frames at 50x would be 167 s
      expect(rt.time).toBe(rt0);
      loading = false;
      await play(app, h, 1 / 60);
      expect(h.status).toBe('done');
      // same cycle as without a runner (to the frame quantisation of the reference run)
      expect(Math.abs(h.endTime - h.startTime - (globalThis as any).__refCycle)).toBeLessThan(0.05);
    } finally { off(); }
  });

  it('Stop while a script runs sets ctx.abort (the runner ends background loops)', async () => {
    const app = makeApp();
    app.newStation();
    const { p } = cell(app);
    let ctxSeen: any = null;
    const off = registerScriptRunner((async (ctx: any) => { ctxSeen = ctx; while (!ctx.abort?.aborted) await new Promise((r) => setTimeout(r, 5)); }) as any);
    try {
      const h: any = app.runProgram(p);
      for (let i = 0; i < 2000 && !ctxSeen; i++) { (app as any).tick(1 / 60); await new Promise((r) => setTimeout(r, 0)); }
      expect(ctxSeen?.abort?.aborted).toBe(false);
      app.stopProgram();                                  // the Simulation tab's Stop
      expect(h.status).toBe('stopped');
      expect(ctxSeen.abort.aborted).toBe(true);
    } finally { off(); }
  });
});

describe('validation of calls handled by a script runner (V6: VisionDetect)', () => {
  it('VisionDetect is not reported as "ignored in simulation" while the vision runner is installed; unknown calls still are', async () => {
    const { installVisionRunnerGlobal } = await import('../src/vision/program_vision');
    const { ProgramSimulator } = await import('../src/core/motion/simulator');
    const off = installVisionRunnerGlobal();
    try {
      const app = makeApp();
      app.newStation();
      const { p } = cell(app);
      p.addInstruction({ kind: 'code', code: 'VisionDetect("Cell camera")', asFunctionCall: true } as any);
      p.addInstruction({ kind: 'code', code: 'NoSuchRoutine(3)', asFunctionCall: true } as any);
      const r = new ProgramSimulator(app.station).compile(p);
      const msgs = r.problems.map((x) => x.message);
      expect(msgs.some((m) => /VisionDetect/.test(m) && /ignored in simulation/.test(m))).toBe(false);
      expect(msgs.some((m) => /NoSuchRoutine/.test(m) && /ignored in simulation/.test(m))).toBe(true);
    } finally { off(); }
  });
});

describe('Properties: cycle time like RoboDK (V1: 05.e Main_Program 1542.6 s shown, RoboDK 237.7 s)', () => {
  it('"Cycle time" is the Item.Update() time (Set speed not applied); the playback time is listed beside it', async () => {
    const { PropertiesPanel } = await import('../src/ui/properties');
    const app = makeApp();
    app.newStation();
    const { p } = cell(app);
    // slow the last move down like 05.e Paint (Set speed 16.7 mm/s): RoboDK's Update() ignores it, playback honours it
    p.addInstruction({ kind: 'speed', speedLinear: 16.7, speedJoints: 5 } as any);
    p.addInstruction({ kind: 'move', moveType: 'MoveJ', targetId: null, joints: [0, -90, 90, -90, -90, 0] } as any);
    const props = new PropertiesPanel(app);
    document.body.append(props.el);
    app.select(p);
    app.validateProgram(p);
    app.select(p);
    const res = p.lastResult!;
    expect(res.updateTime).toBeDefined();
    expect(res.duration).toBeGreaterThan(res.updateTime! + 1);
    const text = props.el.textContent ?? '';
    expect(text).toContain(`Cycle time${res.updateTime!.toFixed(2)} s`);
    expect(text).toContain(`Playback time${res.duration.toFixed(2)} s`);
    props.el.remove();
  });
});
