/**
 * Browser side of the studio server bridge. The server relays JSON-RPC requests from Python/other
 * clients to the browser, which executes them against the live station through the Robolink API.
 */
import { App } from '../app';
import { Robolink, RobolinkItem, Mat } from './robolink';
import { executeRpcAsync } from './rpc';

export async function connectToStudioServer(app: App, RDK: Robolink, url: string): Promise<void> {
  let ws: WebSocket;
  try { ws = new WebSocket(url); } catch { return; }
  await new Promise<void>((resolve, reject) => { ws.onopen = () => resolve(); ws.onerror = () => reject(new Error('ws error')); });
  ws.send(JSON.stringify({ role: 'host', name: 'browser' }));
  app.log(`Connected to studio server at ${url} (Python robolink clients can now drive this station)`);
  ws.onmessage = (ev) => {
    let msg: any;
    try { msg = JSON.parse(ev.data); } catch { return; }
    if (msg.method) {
      executeRpcAsync(RDK, msg, { app }).then((res) => { ws.send(JSON.stringify(res)); if (!/^(Item|ItemList|Joints|Pose|Valid|Name|WaitForEvent)$/.test(msg.method)) { app.snapshot(); app.previewProgram(); } });
    }
  };
  ws.onclose = () => app.log('Studio server disconnected');
}

export { RobolinkItem, Mat };
