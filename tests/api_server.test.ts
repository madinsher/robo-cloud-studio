/**
 * T10: the API server core (src/api/api_server.ts) with the browser bridge (src/api/ws-client.ts): WebSocket and TCP
 * clients are relayed to the browser host when one is connected (else headless), per-method timeouts replace the old
 * 30 s cap, a host disconnect fails pending calls, and the bridge answers quickly without an undo snapshot per call.
 */
import { describe, it, expect, afterEach } from 'vitest';
import net from 'node:net';
import { WebSocket as WsClient } from 'ws';
import { Station } from '../src/core/items/item';
import { Robot } from '../src/core/items/robot';
import { createRobotFromLibrary } from '../src/core/items/library';
import { Robolink } from '../src/api/robolink';
import { startApiServer, ApiServer } from '../src/api/api_server';
import { methodTimeoutMs } from '../src/api/rpc';
import { connectToStudioServer } from '../src/api/ws-client';

const servers: ApiServer[] = [];
const sockets: Array<{ close(): void }> = [];
afterEach(async () => { for (const s of sockets.splice(0)) { try { s.close(); } catch { /* closed */ } } for (const s of servers.splice(0)) await s.close(); });

function headless(): { rdk: Robolink; robot: Robot } {
  const st = new Station('Headless');
  const robot = createRobotFromLibrary('UR10e');
  st.addChild(robot);
  return { rdk: new Robolink(st), robot };
}
async function server(rdk: Robolink): Promise<{ api: ApiServer; port: number; tcpPort: number }> {
  const api = startApiServer({ port: 0, tcpPort: 0, getRDK: () => rdk, log: () => {} });
  servers.push(api);
  const r = await api.ready;
  return { api, ...r };
}
function wsCall(port: number) {
  const ws = new WsClient(`ws://127.0.0.1:${port}`);
  sockets.push(ws);
  const open = new Promise<void>((r) => ws.on('open', () => r()));
  const waiting = new Map<any, (m: any) => void>();
  ws.on('message', (d) => { const m = JSON.parse(String(d)); waiting.get(m.id)?.(m); });
  let id = 0;
  return async (method: string, params: any[] = [], target?: string | null) => {
    await open;
    const req: any = { id: ++id, method, params };
    if (target !== undefined) req.target = target;
    return new Promise<any>((resolve) => { waiting.set(req.id, resolve); ws.send(JSON.stringify(req)); });
  };
}
function tcpCall(port: number, req: any): Promise<any> {
  return new Promise((resolve, reject) => {
    const s = net.connect(port, '127.0.0.1', () => s.write(JSON.stringify(req) + '\n'));
    let buf = '';
    s.on('data', (d) => { buf += d; const i = buf.indexOf('\n'); if (i >= 0) { resolve(JSON.parse(buf.slice(0, i))); s.end(); } });
    s.on('error', reject);
  });
}
/** A fake browser host answering every request with a tag (and optional delay). */
async function fakeHost(port: number, answer: (m: any) => any, delayMs = 0) {
  const ws = new WsClient(`ws://127.0.0.1:${port}`);
  sockets.push(ws);
  await new Promise<void>((r) => ws.on('open', () => r()));
  ws.send(JSON.stringify({ role: 'host' }));
  ws.on('message', (d) => { const m = JSON.parse(String(d)); setTimeout(() => ws.send(JSON.stringify({ id: m.id, result: answer(m) })), delayMs); });
  await new Promise((r) => setTimeout(r, 50));
  return ws;
}

describe('studio API server relay', () => {
  it('WebSocket and TCP clients run headless without a host and on the browser host when one is connected', async () => {
    const { rdk, robot } = headless();
    const { api, port, tcpPort } = await server(rdk);
    const call = wsCall(port);
    expect((await call('Item', ['', 2])).result.$item).toBe(robot.id);
    expect((await tcpCall(tcpPort, { id: 7, method: 'Item', params: ['', 2] })).result.$item).toBe(robot.id);
    const host = await fakeHost(port, (m) => `host:${m.method}`);
    expect(api.hasHost()).toBe(true);
    expect((await call('Version')).result).toBe('host:Version');
    expect(await tcpCall(tcpPort, { id: 8, method: 'ItemList', params: [] })).toEqual({ id: 8, result: 'host:ItemList' });   // TCP is relayed too
    host.close();
    await new Promise((r) => setTimeout(r, 100));
    expect(api.hasHost()).toBe(false);
    expect((await call('Item', ['', 2])).result.$item).toBe(robot.id);                                        // back to headless
  });

  it('MATLAB client wire format: jsonencode output with "$pose" (strrep) or raw x_item/x_pose sets the pose', async () => {
    const { rdk } = headless();
    const frame = rdk.AddFrame('Fm');
    const { tcpPort } = await server(rdk);
    // what clients/matlab/Robolink.m writes for setPose(frame, transl(10,20,30)) (jsonencode of a 4x4 matrix = rows)
    const rows = '[[1,0,0,10],[0,1,0,20],[0,0,1,30],[0,0,0,1]]';
    const r1 = await tcpCall(tcpPort, JSON.parse(`{"id":1,"method":"setPose","params":[{"$pose":${rows}}],"target":"${frame.item!.id}"}`));
    expect(r1.error).toBeUndefined();
    expect(frame.Pose().Pos()).toEqual([10, 20, 30]);
    // MATLAB's own field names without the strrep (x_pose / x_item) are accepted as well
    const r2 = await tcpCall(tcpPort, JSON.parse(`{"id":2,"method":"setPose","params":[{"x_pose":[[1,0,0,1],[0,1,0,2],[0,0,1,3],[0,0,0,1]]}],"target":"${frame.item!.id}"}`));
    expect(r2.error).toBeUndefined();
    expect(frame.Pose().Pos()).toEqual([1, 2, 3]);
    const robot = rdk.ItemList(2)[0];
    const r3 = await tcpCall(tcpPort, { id: 3, method: 'AddProgram', params: ['Pm', { x_item: robot.item!.id }] });
    expect(r3.result.name).toBe('Pm');
    expect((rdk.Item('Pm', 8).item as any).robot().id).toBe(robot.item!.id);   // linked to the robot passed as x_item
    const r4 = await tcpCall(tcpPort, { id: 4, method: 'Pose', params: [], target: frame.item!.id });
    expect(r4.result.$pose).toEqual([[1, 0, 0, 1], [0, 1, 0, 2], [0, 0, 1, 3], [0, 0, 0, 1]]);   // rows: MATLAB jsondecode -> 4x4
  });

  it('per-method timeouts: long calls are not cut at 30 s, a host disconnect fails pending calls', async () => {
    expect(methodTimeoutMs('WaitForEvent', [45])).toBe(75000);
    expect(methodTimeoutMs('WaitFinished')).toBe(0);                 // no timeout
    expect(methodTimeoutMs('MoveJ', [[0, 0, 0, 0, 0, 0], true])).toBe(0);
    expect(methodTimeoutMs('Version')).toBe(120000);
    const { rdk } = headless();
    const { port } = await server(rdk);
    const call = wsCall(port);
    await fakeHost(port, (m) => (m.method === 'WaitForEvent' ? [1, null] : null), 1200);
    const t0 = Date.now();
    expect((await call('WaitForEvent', [0.5])).result).toEqual([1, null]);   // answered after 1.2 s > its 0.5 s wait
    expect(Date.now() - t0).toBeGreaterThanOrEqual(1100);
    const slow = new WsClient(`ws://127.0.0.1:${port}`);
    sockets.push(slow);
    await new Promise<void>((r) => slow.on('open', () => r()));
    slow.send(JSON.stringify({ role: 'host' }));                      // a new host that never answers
    await new Promise((r) => setTimeout(r, 50));
    const pending = call('WaitFinished', []);
    await new Promise((r) => setTimeout(r, 100));
    slow.close();
    expect((await pending).error).toMatch(/browser host disconnected/);
  });

  it('the browser bridge answers fast and takes one debounced undo snapshot per burst, none for reads', async () => {
    const { rdk, robot } = headless();
    const { port } = await server(new Robolink(new Station('unused')));
    let snapshots = 0, previews = 0;
    const app: any = { station: rdk.station, log: () => {}, snapshot: () => { snapshots++; }, previewProgram: () => { previews++; } };
    await connectToStudioServer(app, rdk, `ws://127.0.0.1:${port}`, { snapshotDebounceMs: 200 });
    await new Promise((r) => setTimeout(r, 50));
    const call = wsCall(port);
    for (let i = 0; i < 20; i++) await call('Joints', [], robot.id);
    await new Promise((r) => setTimeout(r, 300));
    expect(snapshots).toBe(0);                                        // read-only calls: no undo step
    const t0 = Date.now();
    for (let i = 0; i < 50; i++) await call('setJoints', [[i, -90, 90, -90, -90, 0]], robot.id);
    const dt = Date.now() - t0;
    expect(robot.joints()[0]).toBe(49);
    expect(dt).toBeLessThan(5000);                                    // was ~0.6 s per call (snapshot + preview each time)
    await new Promise((r) => setTimeout(r, 400));
    expect(snapshots).toBe(1);                                        // one undo step for the burst
    expect(previews).toBe(0);
  });
});
