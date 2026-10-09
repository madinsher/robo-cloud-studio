/**
 * JSON-RPC execution over the Robolink API. Items are referenced by id; poses as 16-element arrays
 * (column-major) or 4x4 nested rows. Shared by the browser bridge and the headless Node server.
 */
import { Robolink, RobolinkItem, Mat } from './robolink';
import { fromRows } from '../core/math/pose';

export interface RpcRequest { id: number | string; method: string; params?: any[]; target?: string | null }
export interface RpcResponse { id: number | string; result?: any; error?: string }

function decode(RDK: Robolink, v: any): any {
  if (v && typeof v === 'object' && !Array.isArray(v)) {
    if (v.$item !== undefined) return new RobolinkItem(RDK, v.$item ? RDK.station.findById(v.$item) : null);
    if (v.$pose) return new Mat(Array.isArray(v.$pose[0]) ? fromRows(v.$pose) : Float64Array.from(v.$pose));
  }
  if (Array.isArray(v) && v.length === 4 && Array.isArray(v[0]) && v[0].length === 4) return new Mat(fromRows(v));
  return v;
}

function encode(v: any): any {
  if (v instanceof RobolinkItem) return { $item: v.item?.id ?? null, name: v.Name(), type: v.Type() };
  if (v instanceof Mat) return { $pose: v.rows() };
  if (v instanceof Float64Array) return { $pose: new Mat(v).rows() };
  if (Array.isArray(v)) return v.map(encode);
  if (v && typeof v === 'object') { const o: any = {}; for (const [k, x] of Object.entries(v)) o[k] = encode(x); return o; }
  return v;
}

/** Async variant: awaits promise results (e.g. WaitForEvent). */
export async function executeRpcAsync(RDK: Robolink, req: RpcRequest, ctx: { app?: any } = {}): Promise<RpcResponse> {
  const res = executeRpc(RDK, req, ctx);
  if (res.result && typeof res.result.then === 'function') {
    try { return { id: req.id, result: encode(await res.result) }; } catch (e: any) { return { id: req.id, error: String(e?.message ?? e) }; }
  }
  return res;
}

export function executeRpc(RDK: Robolink, req: RpcRequest, ctx: { app?: any } = {}): RpcResponse {
  try {
    const params = (req.params ?? []).map((p) => decode(RDK, p));
    let target: any = RDK;
    if (req.target) {
      const it = RDK.station.findById(req.target);
      target = new RobolinkItem(RDK, it);
    }
    if (req.method === '__app__' && ctx.app) {
      const appResult = callPath(ctx.app, params[0], params.slice(1));
      if (appResult && typeof appResult.then === 'function') return { id: req.id, result: appResult };
      return { id: req.id, result: encode(appResult) };
    }
    const fn = target[req.method];
    if (typeof fn !== 'function') return { id: req.id, error: `Unknown method ${req.method}` };
    const result = fn.apply(target, params);
    if (result && typeof result.then === 'function') return { id: req.id, result };
    return { id: req.id, result: encode(result) };
  } catch (e: any) {
    return { id: req.id, error: String(e?.message ?? e) };
  }
}

function callPath(root: any, path: string, args: any[]): any {
  const parts = path.split('.');
  let obj = root;
  for (let i = 0; i < parts.length - 1; i++) obj = obj[parts[i]];
  const fn = obj[parts[parts.length - 1]];
  return typeof fn === 'function' ? fn.apply(obj, args) : fn;
}
