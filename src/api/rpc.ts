/**
 * JSON-RPC execution over the Robolink API, shared by the browser bridge, the headless Node server, the TCP endpoint
 * and the in-process (Pyodide) transport.
 *
 * Wire format (both directions, decoded recursively inside lists and dicts):
 *   {"$item": id|null, name?, type?}  item (null = invalid item)
 *   {"$pose": [[r0],[r1],[r2],[r3]]}  4x4 pose, row-major rows (16-element arrays are column-major Mat4)
 *   {"$mat": [[...],...]}             general matrix, rows
 *   {"$bytes": base64}                binary data
 *   (x_item / x_pose are accepted for $item / $pose: MATLAB's jsonencode cannot write '$' field names)
 * A request without "target" is a Robolink call; "target": null is a call on an invalid item (raises, like RoboDK),
 * "target": id a call on that item.
 */
import { Robolink, RobolinkItem, Mat } from './robolink';
import { fromRows } from '../core/math/pose';

export interface RpcRequest { id: number | string; method: string; params?: any[]; target?: string | null }
export interface RpcResponse { id: number | string; result?: any; error?: string }

/** Read-only calls: no undo snapshot / re-render needed after them (browser bridge). */
export const READ_ONLY_METHODS = new Set([
  'Item', 'ItemList', 'ItemUserPick', 'Valid', 'Name', 'Type', 'Parent', 'Childs', 'Visible', 'Pose', 'PoseAbs', 'PoseTool', 'PoseFrame', 'PoseWrt',
  'Joints', 'SimulatorJoints', 'JointsHome', 'JointLimits', 'JointPoses', 'JointsConfig', 'SolveFK', 'SolveIK', 'SolveIK_All', 'FilterTarget',
  'Busy', 'WaitMove', 'WaitFinished', 'WaitForEvent', 'EventsListen', 'getParam', 'getParams', 'getDI', 'getAI', 'InstructionCount', 'Instruction',
  'InstructionList', 'InstructionListJoints', 'Update', 'Collisions', 'Collision', 'CollisionItems', 'CollisionPairs', 'Collision_Line', 'IsInside',
  'MoveJ_Test', 'MoveL_Test', 'MoveJ_Test_Blend', 'Version', 'License', 'Selection', 'RunMode', 'SimulationSpeed', 'SimulationTime', 'ViewPose',
  'Color', 'GeometryPose', 'isJointTarget', 'getLink', 'getLinks', 'ObjectLink', 'ConnectedState', 'ConnectionParams', 'DOF', 'getFlagsItem',
  'getFlagsRoboDK', 'getOpenStations', 'ActiveStation', 'RunType', 'AccuracyActive', 'Spray_GetStats', 'MachiningParameters', 'RobotParams',
  'Value', 'Htool', 'Tool', 'Frame', 'Cam2D_Snapshot', 'ProjectPoints', 'Render', 'Plugins', 'equals', 'SelectedFeature', 'GetPoints',
  'CollisionActivePairList', 'MeasurePose', 'LaserTracker_Measure', 'StereoCamera_Measure', 'MakeProgram',
]);

/** Server-side relay timeout (ms) for one call; long-running methods get long or unlimited (0) timeouts. */
export function methodTimeoutMs(method: string, params: any[] = []): number {
  const n = (i: number, d: number) => (typeof params[i] === 'number' && isFinite(params[i]) ? params[i] : d);
  switch (method) {
    case 'WaitForEvent': return (n(0, 3600) + 30) * 1000;
    case 'WaitMove': return n(0, 360000) + 30000;
    case 'MoveJ': case 'MoveL': case 'MoveC': case 'SearchL': return params[params.length - 1] === false ? 60000 : 0;
    case 'WaitFinished': case 'waitDI': case 'RunProgram': case 'RunCode': case 'Pause': case 'Finish': return 0;
    case 'Update': case 'InstructionListJoints': case 'setMachiningParameters': case 'setMillingParameters': case 'Command': case 'PluginCommand':
    case 'MoveJ_Test': case 'MoveL_Test': case 'MoveJ_Test_Blend': case 'Popup_ISO9283_CubeProgram': case 'CalibrateTool': case '__app__': return 3600000;
    case 'MakeProgram': case 'AddFile': case 'Save': case 'ProjectPoints': case 'Collisions': case 'Cam2D_Snapshot': case 'BuildMechanism': return 600000;
    default: return 120000;
  }
}

function b64ToBytes(s: string): Uint8Array {
  if (typeof atob === 'function') { const bin = atob(s); const out = new Uint8Array(bin.length); for (let i = 0; i < bin.length; i++) out[i] = bin.charCodeAt(i); return out; }
  return Uint8Array.from((globalThis as any).Buffer.from(s, 'base64'));
}
function bytesToB64(b: Uint8Array): string {
  if ((globalThis as any).Buffer) return (globalThis as any).Buffer.from(b).toString('base64');
  let s = ''; for (let i = 0; i < b.length; i++) s += String.fromCharCode(b[i]); return btoa(s);
}

export function decodeValue(RDK: Robolink, v: any): any {
  if (Array.isArray(v)) return v.map((x) => decodeValue(RDK, x));
  if (v && typeof v === 'object') {
    // x_item / x_pose: the same markers as MATLAB's jsonencode/jsondecode spell them ('$' is not a valid field name)
    if ('$item' in v || 'x_item' in v) { const id = v.$item ?? v.x_item; return new RobolinkItem(RDK, id ? RDK.station.findById(String(id)) : null); }
    if ('$pose' in v || 'x_pose' in v) { const p = v.$pose ?? v.x_pose; return new Mat(Array.isArray(p[0]) ? fromRows(p) : Float64Array.from(p)); }
    if ('$mat' in v) return (v.$mat as any[]).map((r) => (Array.isArray(r) ? r.map(Number) : [Number(r)]));
    if ('$bytes' in v) return b64ToBytes(v.$bytes);
    const o: any = {}; for (const [k, x] of Object.entries(v)) o[k] = decodeValue(RDK, x); return o;
  }
  return v;
}

export function encodeValue(v: any): any {
  if (v instanceof RobolinkItem) return { $item: v.item?.id ?? null, name: v.item?.name ?? '', type: v.item ? v.Type() : -1 };
  if (v instanceof Mat) return { $pose: v.rows().map((r) => r.map(fin)) };
  if (v instanceof Float64Array) return v.length === 16 ? { $pose: new Mat(v).rows().map((r) => r.map(fin)) } : Array.from(v, fin);
  if (v instanceof Uint8Array) return { $bytes: bytesToB64(v) };
  if (ArrayBuffer.isView(v)) return Array.from(v as any as ArrayLike<number>, fin);
  if (Array.isArray(v)) return v.map(encodeValue);
  if (v instanceof Map) { const o: any = {}; for (const [k, x] of v) o[String(k)] = encodeValue(x); return o; }
  if (v && typeof v === 'object' && typeof v.then !== 'function') { const o: any = {}; for (const [k, x] of Object.entries(v)) o[k] = encodeValue(x); return o; }
  if (typeof v === 'number') return fin(v);
  return v;
}
const fin = (x: number) => (Number.isFinite(x) ? x : null);

/** Async variant: awaits promise results (blocking moves, WaitForEvent, WaitFinished, AddFile, ...). */
export async function executeRpcAsync(RDK: Robolink, req: RpcRequest, ctx: { app?: any } = {}): Promise<RpcResponse> {
  const res = executeRpc(RDK, req, ctx);
  if (res.result && typeof res.result.then === 'function') {
    try { return { id: req.id, result: encodeValue(await res.result) }; } catch (e: any) { return { id: req.id, error: errText(e) }; }
  }
  return res;
}

const errText = (e: any) => String(e?.message ?? e);
/** Methods answered for an invalid item instead of raising. */
const INVALID_OK: Record<string, any> = { Valid: false, Type: -1 };

export function executeRpc(RDK: Robolink, req: RpcRequest, ctx: { app?: any } = {}): RpcResponse {
  try {
    const params = (req.params ?? []).map((p) => decodeValue(RDK, p));
    let target: any = RDK;
    if (req.target !== undefined && req.method !== '__app__') {
      const it = req.target ? RDK.station.findById(String(req.target)) : null;
      if (!it) {
        if (req.method in INVALID_OK) return { id: req.id, result: INVALID_OK[req.method] };
        return { id: req.id, error: `InputError: Invalid item (${req.target ? `item ${req.target} was deleted` : 'item not found'}) in ${req.method}` };
      }
      target = new RobolinkItem(RDK, it);
    }
    if (req.method === '__app__') {
      if (!ctx.app) return { id: req.id, error: '__app__ needs the browser studio (no app in headless mode)' };
      const appResult = callPath(ctx.app, params[0], params.slice(1));
      if (appResult && typeof appResult.then === 'function') return { id: req.id, result: appResult };
      return { id: req.id, result: encodeValue(appResult) };
    }
    if (req.method.startsWith('_') || req.method === 'constructor') return { id: req.id, error: `Unknown method ${req.method}` };
    const fn = target[req.method];
    if (typeof fn !== 'function') return { id: req.id, error: `Unknown method ${req.method}${target === RDK ? '' : ` for ${target.Name()}`}: not supported by VerticalBot Studio` };
    RDK.callDepth++;
    let result: any;
    try { result = fn.apply(target, params); } finally { RDK.callDepth--; }
    if (result && typeof result.then === 'function') return { id: req.id, result };
    return { id: req.id, result: encodeValue(result) };
  } catch (e: any) {
    return { id: req.id, error: errText(e) };
  }
}

function callPath(root: any, path: string, args: any[]): any {
  const parts = String(path).split('.');
  let obj = root;
  for (let i = 0; i < parts.length - 1; i++) obj = obj[parts[i]];
  const fn = obj[parts[parts.length - 1]];
  return typeof fn === 'function' ? fn.apply(obj, args) : fn;
}
