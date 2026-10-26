/**
 * Vision inside robot programs and the API (headless).
 *
 * A program calls the camera with a code instruction `VisionDetect("<camera>"[, "<robot>"])` (aliases Vision_Run,
 * VisionRun, DetectParts). The StationRuntime hands the call to visionScriptRunner, which runs the camera's pipeline
 * on the current station state (software render in Node, WebGL in the browser), refreshes the targets of the frame
 * "Vision <camera>" — `Pick` / `Pick approach` hold the most confident target the robot reaches without collision — and sets
 * the signals the program branches on with an IF instruction:
 *   VISION_FOUND = number of feasible targets, VISION_<CAMERA>_FOUND = same, VISION_TARGET = name of the picked target.
 * Station parameters with the same names are written as well (RoboDK getParam style for the API / Python).
 */
import { Station, Camera as CameraItem, ItemType, Item } from '../core/items/item';
import { Robot } from '../core/items/robot';
import { getVisionStack } from './stack';
import { runVisionOnce, VisionRunResult, ModelContext } from './pipeline';
import { registerScriptRunner, type ScriptRunner } from '../core/motion/runtime';

export const VISION_CALL = /^(VisionDetect|Vision_Run|VisionRun|DetectParts)$/i;

/** Signal name with the per-camera count. */
export function visionSignalName(cam: CameraItem): string { return `VISION_${cam.name.toUpperCase().replace(/[^A-Z0-9]+/g, '_').replace(/^_|_$/g, '')}_FOUND`; }

/** Camera for a call argument: by name, else the only camera with a vision stack. */
export function findVisionCamera(station: Station, name?: string): CameraItem | null {
  const cams = station.itemsOfType<CameraItem>(ItemType.CAMERA);
  if (name) return cams.find((c) => c.name === name) ?? cams.find((c) => c.name.toLowerCase() === name.toLowerCase()) ?? null;
  const withStack = cams.filter((c) => getVisionStack(c));
  return withStack.length ? withStack[0] : null;
}

interface SignalSink { set(name: string, v: number | boolean | string): void }

/** Run the vision call on a station, publish the result as signals / station params. */
export async function runVisionCall(station: Station, args: string[], opts: { robot?: Item | null; signals?: SignalSink | null; time?: number; ctx?: ModelContext } = {}): Promise<VisionRunResult> {
  const cam = findVisionCamera(station, args[0]);
  if (!cam) throw new Error(`VisionDetect: camera "${args[0] ?? ''}" not found`);
  const named = args[1] ? station.find(args[1], ItemType.ROBOT) : null;
  const robot = named instanceof Robot ? named : opts.robot instanceof Robot ? opts.robot : null;
  const res = await runVisionOnce(station, cam, { robot, time: opts.time, ctx: opts.ctx });
  const frame = station.find(`Vision ${cam.name}`, ItemType.FRAME);
  const pickT = frame?.children.find((c) => c.name === 'Pick');
  const pick = res.targets.length && pickT ? String((pickT.params.vision as any)?.source ?? '') : '';
  const out: Record<string, number | string> = { VISION_FOUND: res.found, [visionSignalName(cam)]: res.found, VISION_TARGET: res.found ? String(pick) : '' };
  for (const [k, v] of Object.entries(out)) { opts.signals?.set(k, v); station.setParam(k, v as any); }
  return res;
}

/**
 * StationRuntime script runner (contract I5): handles the vision calls and resolves `false` for any other name, so the
 * runtime can pass the call to the next runner (Pyodide / CPython).
 */
export async function visionScriptRunner(ctx: { station: Station; runtime: { signals?: SignalSink; time?: number }; name: string; args: string[]; robot: Item | null }): Promise<boolean> {
  if (!VISION_CALL.test(ctx.name)) return false;
  await runVisionCall(ctx.station, ctx.args, { robot: ctx.robot, signals: ctx.runtime.signals ?? null, time: ctx.runtime.time });
  return true;
}

/** Adapter for StationRuntime.registerScriptRunner: vision calls run the pipeline; any other name keeps the runtime's usual
 *  diagnostic (the call is not executed in simulation) — StationRuntime calls only its first runner (see impl_T9.md). */
function adapter(): ScriptRunner {
  const fn: ScriptRunner = async (ctx) => {
    if (await visionScriptRunner(ctx as any)) return;
    ctx.runtime.events?.emit('message', { text: `Call ${ctx.name}: no program or script named ${ctx.name} in the station (ignored in simulation)`, level: 'warn' });
  };
  // validation (record mode) does not report the vision calls as ignored: they run in live playback
  fn.handles = (name: string) => VISION_CALL.test(name);
  return fn;
}

/** Register the vision runner on one runtime (returns the unregister function). */
export function installVisionRunner(runtime: { registerScriptRunner(fn: ScriptRunner): (() => void) | void }): () => void {
  const off = runtime.registerScriptRunner(adapter());
  return typeof off === 'function' ? off : () => undefined;
}

let globalOff: (() => void) | null = null;
/** Register the vision runner for every StationRuntime (browser app start, server, headless API). Idempotent. */
export function installVisionRunnerGlobal(): () => void {
  if (!globalOff) { const off = registerScriptRunner(adapter()); globalOff = () => { off(); globalOff = null; }; }
  return globalOff;
}
