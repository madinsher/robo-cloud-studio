/**
 * Browser side of the studio server bridge. The server relays JSON-RPC requests from Python/other clients to the
 * browser, which executes them against the live station through the Robolink API.
 *
 * Undo snapshots are debounced (one undo step per burst of mutating calls) and never taken for read-only calls; the
 * program preview is not recompiled after API calls (that used to cancel RunProgram playback). Robot moves and
 * programs started through the API play in real time through rdk.exec (the app's shared runtime when attached).
 * Station Python programs called by programs run on the server's CPython (POST /python/run) while connected.
 */
import { App } from '../app';
import { Robolink, RobolinkItem, Mat } from './robolink';
import { executeRpcAsync, READ_ONLY_METHODS } from './rpc';
import { pythonSource } from './exec';
import type { ScriptContext } from './exec';
import { registerScriptRunner } from '../core/motion/runtime';

export interface BridgeOptions { snapshotDebounceMs?: number }

export async function connectToStudioServer(app: App, RDK: Robolink, url: string, opts: BridgeOptions = {}): Promise<void> {
  let ws: WebSocket;
  try { ws = new WebSocket(url); } catch { return; }
  await new Promise<void>((resolve, reject) => { ws.onopen = () => resolve(); ws.onerror = () => reject(new Error('ws error')); });
  ws.send(JSON.stringify({ role: 'host', name: 'browser' }));
  app.log(`Connected to studio server at ${url} (Python robolink clients can now drive this station)`);
  RDK.exec.mode = 'realtime';
  // station Python programs: while connected they run on the server's CPython (threads, numpy, Python posts), their
  // API calls come back through this bridge; the previous runner (Pyodide) is restored when the server goes away
  const httpBase = url.replace(/^ws/, 'http').replace(/\/+$/, '');
  const prevRunner = RDK.exec.getScriptRunner();
  const prevKind = (RDK.exec as any).scriptRunnerKind ?? null;
  const serverRunner = async (ctx: ScriptContext) => {
    const code = pythonSource(ctx);
    if (code == null) return false as any;      // not a station Python program: not handled
    const res = await fetch(`${httpBase}/python/run`, { method: 'POST', headers: { 'content-type': 'application/json' }, body: JSON.stringify({ name: ctx.name, code, args: ctx.args }) });
    const body = await res.json().catch(() => ({}));
    if (body.output) app.log(String(body.output).trim().split(/\r?\n/).slice(-20).join('\n'));
    if (!res.ok || body.error) throw new Error(body.error ?? `HTTP ${res.status}`);
  };
  RDK.exec.setScriptRunner(serverRunner);
  (RDK.exec as any).scriptRunnerKind = 'server';
  const offGlobal = registerScriptRunner(serverRunner as any);
  let timer: ReturnType<typeof setTimeout> | null = null;
  const debounce = opts.snapshotDebounceMs ?? 400;
  ws.onmessage = (ev) => {
    let msg: any;
    try { msg = JSON.parse(ev.data); } catch { return; }
    if (!msg.method) return;
    executeRpcAsync(RDK, msg, { app }).then((res) => {
      ws.send(JSON.stringify(res));
      if (READ_ONLY_METHODS.has(msg.method) || res.error) return;
      if (timer) clearTimeout(timer);
      timer = setTimeout(() => { timer = null; app.snapshot(); }, debounce);
    });
  };
  ws.onclose = () => {
    offGlobal();
    if (RDK.exec.getScriptRunner() === serverRunner) { RDK.exec.setScriptRunner(prevRunner); (RDK.exec as any).scriptRunnerKind = prevKind; }
    app.log('Studio server disconnected');
  };
}

export { RobolinkItem, Mat };
