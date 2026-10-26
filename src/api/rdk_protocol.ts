/**
 * RoboDK binary API protocol server (Node). The official clients speak this protocol over TCP: the robodk pip package
 * (robodk.robolink), RoboDK's embedded Python, and RoboDK's C#, C++ and MATLAB APIs. With this server they drive the
 * studio unmodified: `Robolink(port=STUDIO_RDK_PORT)`.
 *
 * Wire format (robodk/robolink.py _send_* / _rec_*): a command is a line ('\n'-terminated, UTF-8) followed by its
 * arguments; big-endian int32, float64, uint64 item pointers (+ int32 type when the server returns an item), poses as
 * 16 float64 column-major, arrays as int32 n + n float64, matrices as int32 rows, int32 cols + column-major float64,
 * bytes as uint32 length + data. Every reply ends with an int32 status (0 OK, 2/3 + message warning/error, 10..13 +
 * message for TargetReachError, StoppedError, InputError, LicenseError). On an error the reply still carries outputs
 * of the expected shape (zeros) so the stream stays in sync.
 *
 * Each command is translated into a JSON-RPC request handled by `handle` (the studio's relay-aware API server:
 * the browser host when connected, else the headless station), so semantics are those of src/api/robolink.ts.
 */
import net from 'node:net';
import type { RpcRequest, RpcResponse } from './rpc';

export interface RdkProtocolOptions {
  port?: number;
  host?: string;
  handle: (req: RpcRequest) => Promise<RpcResponse>;
  log?: (msg: string) => void;
}
export interface RdkProtocolServer { server: net.Server; port: number; close(): Promise<void> }

/** Build number reported to clients (RoboDK 5.9.x): clients enable their newest code paths (AddShape5, ...). */
const BUILD = 25000;
class Closed extends Error {}

class Reader {
  private buf: Buffer = Buffer.alloc(0);
  private waiters: Array<() => void> = [];
  private ended = false;
  push(d: Buffer): void { this.buf = this.buf.length ? Buffer.concat([this.buf, d]) : d; this.wake(); }
  end(): void { this.ended = true; this.wake(); }
  private wake(): void { const w = this.waiters; this.waiters = []; for (const f of w) f(); }
  private async need(n: number): Promise<void> {
    while (this.buf.length < n) { if (this.ended) throw new Closed('closed'); await new Promise<void>((r) => this.waiters.push(r)); }
  }
  async take(n: number): Promise<Buffer> { await this.need(n); const out = this.buf.subarray(0, n); this.buf = this.buf.subarray(n); return out; }
  async line(): Promise<string> {
    for (;;) {
      const i = this.buf.indexOf(10);
      if (i >= 0) { const s = this.buf.subarray(0, i).toString('utf8'); this.buf = this.buf.subarray(i + 1); return s.replace(/\r$/, '').replace(/<br>/g, '\n'); }
      if (this.ended) throw new Closed('closed');
      await new Promise<void>((r) => this.waiters.push(r));
    }
  }
  async int(): Promise<number> { return (await this.take(4)).readInt32BE(0); }
  async u64(): Promise<bigint> { return (await this.take(8)).readBigUInt64BE(0); }
  async f64s(n: number): Promise<number[]> { const b = await this.take(8 * n); const out: number[] = []; for (let i = 0; i < n; i++) out.push(b.readDoubleBE(8 * i)); return out; }
  async f32s(n: number): Promise<number[]> { const b = await this.take(4 * n); const out: number[] = []; for (let i = 0; i < n; i++) out.push(b.readFloatBE(4 * i)); return out; }
  /** 4x4 pose, column-major on the wire -> rows. */
  async pose(): Promise<number[][]> { const v = await this.f64s(16); return [0, 1, 2, 3].map((r) => [0, 1, 2, 3].map((c) => v[c * 4 + r])); }
  async xyz(): Promise<number[]> { return this.f64s(3); }
  async array(): Promise<number[]> { const n = await this.int(); return n > 0 ? this.f64s(n) : []; }
  /** Matrix (rows x cols, column-major) -> rows. */
  async matrix(): Promise<number[][]> {
    const r = await this.int(), c = await this.int();
    const v = r * c > 0 ? await this.f64s(r * c) : [];
    return Array.from({ length: r }, (_, i) => Array.from({ length: c }, (_, j) => v[j * r + i]));
  }
  async matrixF32(): Promise<number[][]> {
    const r = await this.int(), c = await this.int();
    const v = r * c > 0 ? await this.f32s(r * c) : [];
    return Array.from({ length: r }, (_, i) => Array.from({ length: c }, (_, j) => v[j * r + i]));
  }
  async arrayF32(): Promise<number[]> { const n = await this.int(); return n > 0 ? this.f32s(n) : []; }
  async bytes(): Promise<Buffer> { const n = (await this.take(4)).readUInt32BE(0); return n ? Buffer.from(await this.take(n)) : Buffer.alloc(0); }
}

class Writer {
  parts: Buffer[] = [];
  constructor(private sock: net.Socket) {}
  line(s: string): void { this.parts.push(Buffer.from(String(s ?? '').replace(/\r?\n/g, '<br>') + '\n', 'utf8')); }
  int(n: number): void { const b = Buffer.alloc(4); b.writeInt32BE(Math.max(-2147483648, Math.min(2147483647, Math.round(Number(n) || 0))), 0); this.parts.push(b); }
  u64(v: bigint): void { const b = Buffer.alloc(8); b.writeBigUInt64BE(v, 0); this.parts.push(b); }
  f64s(v: number[]): void { const b = Buffer.alloc(8 * v.length); v.forEach((x, i) => b.writeDoubleBE(Number.isFinite(Number(x)) ? Number(x) : 0, 8 * i)); this.parts.push(b); }
  pose(rows: number[][] | null): void { const m = rows && rows.length === 4 ? rows : [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]; const v: number[] = []; for (let c = 0; c < 4; c++) for (let r = 0; r < 4; r++) v.push(m[r][c]); this.f64s(v); }
  xyz(p: number[] | null): void { this.f64s([0, 1, 2].map((i) => Number(p?.[i] ?? 0))); }
  array(v: number[] | null): void { const a = (v ?? []).map(Number); this.int(a.length); if (a.length) this.f64s(a); }
  matrix(rows: number[][] | null): void {
    const m = rows ?? [];
    const r = m.length, c = r ? m[0].length : 0;
    this.int(r); this.int(c);
    const v: number[] = []; for (let j = 0; j < c; j++) for (let i = 0; i < r; i++) v.push(m[i][j]);
    if (v.length) this.f64s(v);
  }
  bytes(b: Uint8Array | null): void { const d = Buffer.from(b ?? new Uint8Array(0)); const h = Buffer.alloc(4); h.writeUInt32BE(d.length, 0); this.parts.push(h, d); }
  flush(): void { if (this.parts.length && !this.sock.destroyed) this.sock.write(Buffer.concat(this.parts)); this.parts = []; }
  discard(): void { this.parts = []; }
}

/** Item pointer table: studio item ids <-> 64-bit pointers handed to clients (0 = invalid item). */
class Pointers {
  private toPtr = new Map<string, bigint>();
  private toId = new Map<bigint, string>();
  private next = 0x1000n;
  ptr(id: string | null | undefined): bigint {
    if (!id) return 0n;
    let p = this.toPtr.get(id);
    if (p === undefined) { p = this.next; this.next += 0x10n; this.toPtr.set(id, p); this.toId.set(p, id); }
    return p;
  }
  id(p: bigint): string | null { return p === 0n ? null : this.toId.get(p) ?? `__unknown_${p}`; }
}

type Ref = string | null;
interface Ctx {
  r: Reader;
  w: Writer;
  item(): Promise<Ref>;
  ptr(): Promise<bigint>;
  /** JSON-RPC call (params: Ref items as {$item}, rows poses as {$pose}); throws with the API error message. */
  call(method: string, params?: any[], target?: Ref): Promise<any>;
  /** Write outputs + status 0, or (on error) the fallback outputs + error status. */
  reply(body: () => Promise<void> | void, fallback?: () => void): Promise<void>;
  wItem(v: any): void;
  wItems(list: any[]): void;
  /** Pointer of an item result / item id of a pointer (camera handles). */
  ptrOf(v: any): bigint;
  idOf(p: bigint): Ref;
  /** Command being handled. */
  cmd?: string;
}
type Handler = (c: Ctx) => Promise<void>;

const it = (ref: Ref) => ({ $item: ref });
const P = (rows: number[][]) => ({ $pose: rows });
const M = (rows: number[][]) => ({ $mat: rows });
/** Result -> rows of a pose. */
const rowsOf = (v: any): number[][] | null => (v && v.$pose ? v.$pose : Array.isArray(v) && v.length === 4 && Array.isArray(v[0]) ? v : null);
const matOf = (v: any): number[][] => (v && v.$mat ? v.$mat : v && v.$pose ? v.$pose : Array.isArray(v) ? v.map((r: any) => (Array.isArray(r) ? r.map(Number) : [Number(r)])) : []);
const numsOf = (v: any): number[] => (Array.isArray(v) ? v.flat(2).map(Number) : v == null ? [] : [Number(v)]);
/** RoboDK matrices of points / joints are column vectors: list of rows-vectors -> matrix whose columns are them. */
const cols = (list: number[][]): number[][] => { if (!list.length) return []; const n = Math.max(...list.map((x) => x.length)); return Array.from({ length: n }, (_, i) => list.map((x) => Number(x[i] ?? 0))); };
const colsToList = (m: number[][]): number[][] => (m.length ? m[0].map((_, j) => m.map((r) => r[j])) : []);

const H: Record<string, Handler> = {};
const def = (names: string | string[], h: Handler) => { for (const n of Array.isArray(names) ? names : [names]) H[n] = h; };

// ---- station ----------------------------------------------------------------------------------------------------
def('G_Item', async (c) => { const name = await c.r.line(); await c.reply(async () => c.wItem(await c.call('Item', [name])), () => c.wItem(null)); });
def('G_Item2', async (c) => { const name = await c.r.line(); const t = await c.r.int(); await c.reply(async () => c.wItem(await c.call('Item', [name, t])), () => c.wItem(null)); });
def('G_List_Items', async (c) => { await c.reply(async () => { const l = await c.call('ItemList', [-1, true]); c.w.int(l.length); for (const s of l) c.w.line(s); }, () => c.w.int(0)); });
def('G_List_Items_Type', async (c) => { const t = await c.r.int(); await c.reply(async () => { const l = await c.call('ItemList', [t, true]); c.w.int(l.length); for (const s of l) c.w.line(s); }, () => c.w.int(0)); });
def('G_List_Items_ptr', async (c) => { await c.reply(async () => c.wItems(await c.call('ItemList', [-1, false])), () => c.w.int(0)); });
def('G_List_Items_Type_ptr', async (c) => { const t = await c.r.int(); await c.reply(async () => c.wItems(await c.call('ItemList', [t, false])), () => c.w.int(0)); });
def('PickItem', async (c) => { const msg = await c.r.line(); const t = await c.r.int(); await c.reply(async () => c.wItem(await c.call('ItemUserPick', [msg, t])), () => c.wItem(null)); });
def('PickItemList', async (c) => { const msg = await c.r.line(); const n = await c.r.int(); const l: Ref[] = []; for (let i = 0; i < n; i++) l.push(await c.item()); await c.reply(async () => c.wItem(await c.call('ItemUserPick', [msg, l.map(it)])), () => c.wItem(null)); });
def('RAISE', async (c) => c.reply(() => c.call('ShowRoboDK')));
def('HIDE', async (c) => c.reply(() => c.call('HideRoboDK')));
def('QUIT', async (c) => c.reply(() => c.call('CloseRoboDK')));
def('Version', async (c) => c.reply(async () => { const v = String(await c.call('Version')); c.w.line('RoboDK'); c.w.int(64); c.w.line(v.split(' ')[0] || '5.9.4'); c.w.line('VerticalBot Studio'); }));
def('S_WindowState', async (c) => { const s = await c.r.int(); await c.reply(() => c.call('setWindowState', [s])); });
def('S_RoboDK_Rights', async (c) => { const f = await c.r.int(); await c.reply(() => c.call('setFlagsRoboDK', [f])); });
def('S_Item_Rights', async (c) => { const i = await c.item(); const f = await c.r.int(); await c.reply(() => c.call('setFlagsItem', [it(i), f])); });
def('G_Item_Rights', async (c) => { const i = await c.item(); await c.reply(async () => c.w.int(await c.call('getFlagsItem', [it(i)])), () => c.w.int(0)); });
def('ShowMessage', async (c) => { const m = await c.r.line(); await c.reply(() => c.call('ShowMessage', [m, true])); });
def('ShowMessageStatus', async (c) => { const m = await c.r.line(); await c.reply(() => c.call('ShowMessage', [m, false])); });
def('Copy2', async (c) => { const i = await c.item(); const ch = await c.r.int(); await c.reply(() => c.call('Copy', [it(i), !!ch])); });
def('Copy', async (c) => { const i = await c.item(); await c.reply(() => c.call('Copy', [it(i)])); });
def('Paste', async (c) => { const i = await c.item(); await c.reply(async () => c.wItem(await c.call('Paste', [it(i)])), () => c.wItem(null)); });
def('PastN', async (c) => { const i = await c.item(); const n = await c.r.int(); await c.reply(async () => { const r = await c.call('Paste', [it(i), n]); c.wItems(Array.isArray(r) ? r : [r]); }, () => c.w.int(0)); });
def('Add', async (c) => { const f = await c.r.line(); const p = await c.item(); await c.reply(async () => c.wItem(await c.call('AddFile', [f, it(p)])), () => c.wItem(null)); });
def('AddShape2', async (c) => { const m = await c.r.matrix(); const to = await c.item(); const ov = await c.r.int(); await c.reply(async () => c.wItem(await c.call('AddShape', [colsToList(m), it(to), !!ov])), () => c.wItem(null)); });
def(['AddShape4', 'AddShape5'], async (c) => {
  const f32 = c.cmd === 'AddShape5';
  const to = await c.item(); const ov = await c.r.int(); const n = await c.r.int();
  const shapes: number[][][] = [];
  for (let i = 0; i < n; i++) { const m = f32 ? await c.r.matrixF32() : await c.r.matrix(); if (f32) await c.r.arrayF32(); else await c.r.array(); shapes.push(colsToList(m)); }
  await c.reply(async () => c.wItem(await c.call('AddShape', [shapes, it(to), !!ov])), () => c.wItem(null));
});
def('AddWire', async (c) => { const m = await c.r.matrix(); const ref = await c.item(); const add = await c.r.int(); const proj = await c.r.int(); await c.reply(async () => c.wItem(await c.call('AddCurve', [colsToList(m), it(ref), !!add, proj])), () => c.wItem(null)); });
def('AddWir2', async (c) => { const ref = await c.item(); const add = await c.r.int(); const proj = await c.r.int(); const n = await c.r.int(); const l: number[][][] = []; for (let i = 0; i < n; i++) l.push(colsToList(await c.r.matrix())); await c.reply(async () => c.wItem(await c.call('AddCurve', [l, it(ref), !!add, proj, true])), () => c.wItem(null)); });
def('AddPoints', async (c) => { const m = await c.r.matrix(); const ref = await c.item(); const add = await c.r.int(); const proj = await c.r.int(); await c.reply(async () => c.wItem(await c.call('AddPoints', [colsToList(m), it(ref), !!add, proj])), () => c.wItem(null)); });
def('ProjectPoints', async (c) => { const m = await c.r.matrix(); const obj = await c.item(); const proj = await c.r.int(); await c.reply(async () => c.w.matrix(cols(matOf(await c.call('ProjectPoints', [colsToList(m), it(obj), proj])))), () => c.w.matrix([])); });
def('RemoveStn', async (c) => c.reply(() => c.call('CloseStation')));
def('RemoveLst', async (c) => { const n = await c.r.int(); const l: Ref[] = []; for (let i = 0; i < n; i++) l.push(await c.item()); await c.reply(() => c.call('Delete', [l.map(it)])); });
def('Save', async (c) => { const f = await c.r.line(); const i = await c.item(); await c.reply(() => c.call('Save', [f, it(i)])); });
def('NewStation', async (c) => { const n = await c.r.line(); await c.reply(async () => c.wItem(await c.call('AddStation', [n])), () => c.wItem(null)); });
def('Add_TARGET', async (c) => { const n = await c.r.line(); const p = await c.item(); const r = await c.item(); await c.reply(async () => c.wItem(await c.call('AddTarget', [n, it(p), it(r)])), () => c.wItem(null)); });
def('Add_FRAME', async (c) => { const n = await c.r.line(); const p = await c.item(); await c.reply(async () => c.wItem(await c.call('AddFrame', [n, it(p)])), () => c.wItem(null)); });
def('Add_PROG', async (c) => { const n = await c.r.line(); const r = await c.item(); await c.reply(async () => c.wItem(await c.call('AddProgram', [n, it(r)])), () => c.wItem(null)); });
def('Add_MACHINING', async (c) => { const n = await c.r.line(); const r = await c.item(); await c.reply(async () => c.wItem(await c.call('AddMachiningProject', [n, it(r)])), () => c.wItem(null)); });
def('RunCode', async (c) => { const fcn = await c.r.int(); const code = await c.r.line(); await c.reply(async () => c.w.int(await c.call('RunCode', [code, !!fcn])), () => c.w.int(0)); });
def('RunMessage', async (c) => { const cm = await c.r.int(); const m = await c.r.line(); await c.reply(() => c.call('RunMessage', [m, !!cm])); });
def('Render', async (c) => { const v = await c.r.int(); await c.reply(() => c.call('Render', [v])); });
def('Refresh', async (c) => { await c.r.int(); await c.reply(() => c.call('Update')); });
def('IsInside', async (c) => { const a = await c.item(); const b = await c.item(); await c.reply(async () => c.w.int(await c.call('IsInside', [it(a), it(b)])), () => c.w.int(0)); });
def('Collision_SetState', async (c) => { const s = await c.r.int(); await c.reply(async () => c.w.int(await c.call('setCollisionActive', [s])), () => c.w.int(0)); });
def('Collision_SetPair', async (c) => { const a = await c.item(); const b = await c.item(); const i1 = await c.r.int(); const i2 = await c.r.int(); const s = await c.r.int(); await c.reply(async () => c.w.int((await c.call('setCollisionActivePair', [s, it(a), it(b), i1, i2])) ? 1 : 0), () => c.w.int(0)); });
def('Collision_SetPairList', async (c) => {
  const n = await c.r.int(); const a: Ref[] = [], b: Ref[] = [], i1: number[] = [], i2: number[] = [], s: number[] = [];
  for (let k = 0; k < n; k++) { a.push(await c.item()); b.push(await c.item()); i1.push(await c.r.int()); i2.push(await c.r.int()); s.push(await c.r.int()); }
  await c.reply(async () => c.w.int(await c.call('setCollisionActivePairList', [s, a.map(it), b.map(it), i1, i2])), () => c.w.int(0));
});
const pairs = (method: string) => async (c: Ctx) => c.reply(async () => { const l = await c.call(method); c.w.int(l.length); for (const p of l) { c.wItem(p[0]); c.w.int(p[2] ?? -1); c.wItem(p[1]); c.w.int(p[3] ?? -1); } }, () => c.w.int(0));
def('Collision_GetPairList', pairs('CollisionActivePairList'));
def('Collision_Pairs', pairs('CollisionPairs'));
def('Collisions', async (c) => c.reply(async () => c.w.int(await c.call('Collisions')), () => c.w.int(0)));
def('Collided', async (c) => { const a = await c.item(); const b = await c.item(); await c.reply(async () => c.w.int(await c.call('Collision', [it(a), it(b)])), () => c.w.int(0)); });
def('Collision_Items', async (c) => c.reply(async () => { const l = await c.call('CollisionItems'); c.w.int(l.length); for (const x of l) { c.wItem(x); c.w.int(0); c.w.int(1); } }, () => c.w.int(0)));
def('CollisionLine', async (c) => { const a = await c.r.xyz(); const b = await c.r.xyz(); await c.reply(async () => { const r = await c.call('Collision_Line', [a, b]); c.wItem(r?.[1] ?? null); c.w.xyz(r?.[2] ?? [0, 0, 0]); }, () => { c.wItem(null); c.w.xyz(null); }); });
def('CollisionLineLst', async (c) => {
  const n = await c.r.int(); const a: number[][] = [], b: number[][] = [];
  for (let k = 0; k < n; k++) { a.push(await c.r.xyz()); b.push(await c.r.xyz()); }
  await c.reply(async () => { let first: any = null; const pts: number[][] = []; for (let k = 0; k < n; k++) { const r = await c.call('Collision_Line', [a[k], b[k]]); if (!first && r?.[0]) first = r[1]; pts.push(r?.[0] ? r[2] : b[k]); } c.wItem(first); c.w.matrix(cols(pts)); }, () => { c.wItem(null); c.w.matrix([]); });
});
def('SimulateSpeed', async (c) => { const v = await c.r.int(); await c.reply(() => c.call('setSimulationSpeed', [v / 1000])); });
def('GetSimulateSpeed', async (c) => c.reply(async () => c.w.int(Number(await c.call('SimulationSpeed')) * 1000), () => c.w.int(1000)));
def('GetSimTime', async (c) => c.reply(async () => c.w.int(Number(await c.call('SimulationTime')) * 1000), () => c.w.int(0)));
def('S_RunMode', async (c) => { const m = await c.r.int(); await c.reply(() => c.call('setRunMode', [m])); });
def('G_RunMode', async (c) => c.reply(async () => c.w.int(await c.call('RunMode')), () => c.w.int(1)));
def('G_Params', async (c) => c.reply(async () => { const l = await c.call('getParams'); c.w.int(l.length); for (const [k, v] of l) { c.w.line(String(k)); c.w.line(String(v)); } }, () => c.w.int(0)));
def('G_Param', async (c) => { const k = await c.r.line(); await c.reply(async () => { const v = await c.call('getParam', [k]); c.w.line(v == null ? `UNKNOWN ${k}` : typeof v === 'object' ? JSON.stringify(v) : String(v)); }, () => c.w.line(`UNKNOWN ${k}`)); });
def('G_DataParam', async (c) => { const k = await c.r.line(); await c.reply(async () => { const v = await c.call('getParam', [k, false]); c.w.bytes(v instanceof Uint8Array ? v : v && v.$bytes ? Buffer.from(v.$bytes, 'base64') : Buffer.from(v == null ? '' : String(v), 'utf8')); }, () => c.w.bytes(null)); });
def('S_Param', async (c) => { const k = await c.r.line(); const v = await c.r.line(); await c.reply(() => c.call('setParam', [k, v])); });
def('S_DataParam', async (c) => { const k = await c.r.line(); const v = await c.r.bytes(); await c.reply(() => c.call('setParam', [k, { $bytes: v.toString('base64') }])); });
def('SCMD', async (c) => { const k = await c.r.line(); const v = await c.r.line(); await c.reply(async () => c.w.line(String(await c.call('Command', [k, v]) ?? '')), () => c.w.line('')); });
def('BlindSCMD', async (c) => { const k = await c.r.line(); const v = await c.r.line(); await c.call('Command', [k, v]).catch(() => {}); });
def('G_Gen_Mat', async (c) => { const i = await c.item(); const k = await c.r.line(); const m = await c.r.matrix(); await c.reply(async () => { const r = i ? await c.call('setParam', [k, M(m)], i) : await c.call('Command', [k, M(m)]); const l = Array.isArray(r) && r.length && Array.isArray(r[0]?.[0]) ? r : r && (r.$mat || Array.isArray(r)) ? [matOf(r)] : []; c.w.int(l.length); for (const x of l) c.w.matrix(matOf(x)); }, () => c.w.int(0)); });
def('G_AllStn', async (c) => c.reply(async () => c.wItems(await c.call('getOpenStations')), () => c.w.int(0)));
def('G_ActiveStn', async (c) => c.reply(async () => c.wItem(await c.call('ActiveStation')), () => c.wItem(null)));
def('S_ActiveStn', async (c) => { const s = await c.item(); await c.reply(() => c.call('setActiveStation', [it(s)])); });
def('S_Hlocals', async (c) => { const n = await c.r.int(); const l: Ref[] = [], p: number[][][] = []; for (let k = 0; k < n; k++) { l.push(await c.item()); p.push(await c.r.pose()); } await c.reply(() => c.call('setPoses', [l.map(it), p.map(P)])); });
def('S_Hlocal_AbsS', async (c) => { const n = await c.r.int(); const l: Ref[] = [], p: number[][][] = []; for (let k = 0; k < n; k++) { l.push(await c.item()); p.push(await c.r.pose()); } await c.reply(() => c.call('setPosesAbs', [l.map(it), p.map(P)])); });
def('G_ThetasList', async (c) => {
  // interleaved: the client sends one robot and reads its joints before sending the next
  const n = await c.r.int(); let err: Error | null = null;
  for (let k = 0; k < n; k++) { const r = await c.item(); try { c.w.array(numsOf(await c.call('Joints', [], r))); } catch (e: any) { err ??= e; c.w.array([]); } c.w.flush(); }
  await c.reply(() => { if (err) throw err; });
});
def('S_ThetasList', async (c) => { const n = await c.r.int(); const l: Ref[] = [], j: number[][] = []; for (let k = 0; k < n; k++) { l.push(await c.item()); j.push(await c.r.array()); } await c.reply(() => c.call('setJoints', [l.map(it), j])); });
def('ProgramStart', async (c) => { const n = await c.r.line(); const f = await c.r.line(); const post = await c.r.line(); const r = await c.item(); await c.reply(async () => c.w.int(await c.call('ProgramStart', [n, f, post, it(r)])), () => c.w.int(-1)); });
def('S_ViewPose', async (c) => { const p = await c.r.pose(); await c.reply(() => c.call('setViewPose', [P(p)])); });
def('G_ViewPose', async (c) => c.reply(async () => c.w.pose(rowsOf(await c.call('ViewPose'))), () => c.w.pose(null)));
def('Cam2D_PtrAdd', async (c) => { const ref = await c.item(); const cam = await c.item(); const prm = await c.r.line(); await c.reply(async () => c.wItem(await c.call('Cam2D_Add', [it(ref), prm, cam ? it(cam) : null])), () => c.wItem(null)); });
def('Cam2D_Add', async (c) => { const ref = await c.item(); const prm = await c.r.line(); await c.reply(async () => { const r = await c.call('Cam2D_Add', [it(ref), prm]); c.w.u64(c.ptrOf(r)); }, () => c.w.u64(0n)); });
def('Cam2D_PtrSnapshot', async (c) => {
  const cam = await c.item(); const file = await c.r.line(); const prm = await c.r.line();
  await c.reply(async () => { const r = await c.call('Cam2D_Snapshot', [file, cam ? it(cam) : 0, prm]); if (!file) c.w.bytes(r instanceof Uint8Array ? r : r && r.$bytes ? Buffer.from(r.$bytes, 'base64') : typeof r === 'string' && r.startsWith('data:') ? Buffer.from(r.split(',')[1], 'base64') : null); else c.w.int(r ? 1 : 0); }, () => (file ? c.w.bytes(null) : c.w.int(0)));
});
def('Cam2D_Snapshot', async (c) => { const p = await c.ptr(); const file = await c.r.line(); await c.reply(async () => c.w.int((await c.call('Cam2D_Snapshot', [file, p ? it(c.idOf(p)) : 0])) ? 1 : 0), () => c.w.int(0)); });
def('Cam2D_CloseAll', async (c) => c.reply(async () => c.w.int((await c.call('Cam2D_Close', [0])) ? 1 : 0), () => c.w.int(0)));
def('Cam2D_Close', async (c) => { const p = await c.ptr(); await c.reply(async () => c.w.int((await c.call('Cam2D_Close', [it(c.idOf(p))])) ? 1 : 0), () => c.w.int(0)); });
def('Cam2D_PtrClose', async (c) => { const cam = await c.item(); await c.reply(async () => c.w.int((await c.call('Cam2D_Close', [it(cam)])) ? 1 : 0), () => c.w.int(0)); });
def('Cam2D_SetParams', async (c) => { const p = await c.ptr(); const prm = await c.r.line(); await c.reply(async () => c.w.int((await c.call('Cam2D_SetParams', [prm, it(c.idOf(p))])) ? 1 : 0), () => c.w.int(0)); });
def('Cam2D_PtrSetParams', async (c) => { const cam = await c.item(); const prm = await c.r.line(); await c.reply(async () => c.w.int((await c.call('Cam2D_SetParams', [prm, it(cam)])) ? 1 : 0), () => c.w.int(0)); });
def('Gun_Add', async (c) => { const t = await c.item(); const o = await c.item(); const prm = await c.r.line(); const pts = await c.r.matrix(); const geo = await c.r.matrix(); await c.reply(async () => c.w.int(await c.call('Spray_Add', [it(t), it(o), prm, M(pts), M(geo)])), () => c.w.int(-1)); });
def('Gun_SetState', async (c) => { const s = await c.r.int(); const id = await c.r.int(); await c.reply(async () => c.w.int(await c.call('Spray_SetState', [s, id])), () => c.w.int(0)); });
def('Gun_Stats', async (c) => { const id = await c.r.int(); await c.reply(async () => { const [info, rows] = await c.call('Spray_GetStats', [id]); c.w.line(String(info ?? '').replace(/<br>/g, '\t')); c.w.matrix(cols(matOf(rows))); }, () => { c.w.line(''); c.w.matrix([]); }); });
def('Gun_Clear', async (c) => { const id = await c.r.int(); await c.reply(async () => c.w.int(await c.call('Spray_Clear', [id])), () => c.w.int(0)); });
def('G_License2', async (c) => c.reply(async () => { const l = await c.call('License'); c.w.line(String(l?.[0] ?? '')); c.w.line(String(l?.[1] ?? '')); }, () => { c.w.line(''); c.w.line(''); }));
def('G_Selection', async (c) => c.reply(async () => c.wItems(await c.call('Selection')), () => c.w.int(0)));
def('S_Selection', async (c) => { const n = await c.r.int(); const l: Ref[] = []; for (let k = 0; k < n; k++) l.push(await c.item()); await c.reply(() => c.call('setSelection', [l.map(it)])); });
def('MergeItems', async (c) => { const n = await c.r.int(); const l: Ref[] = []; for (let k = 0; k < n; k++) l.push(await c.item()); await c.reply(async () => c.wItem(await c.call('MergeItems', [l.map(it)])), () => c.wItem(null)); });
def('Popup_ProgISO9283', async (c) => { const r = await c.item(); await c.reply(async () => c.wItem(await c.call('Popup_ISO9283_CubeProgram', [it(r)])), () => c.wItem(null)); });
def('S_InteractiveMode', async (c) => { const mode = await c.r.int(); const flags = await c.r.int(); const n = await c.r.int(); for (let k = 0; k < n; k++) { await c.item(); await c.r.int(); } await c.reply(() => c.call('setInteractiveMode', [mode, flags])); });
def('PluginCommand', async (c) => { const p = await c.r.line(); const cmd = await c.r.line(); const v = await c.r.line(); await c.reply(async () => c.w.line(String(await c.call('PluginCommand', [p, cmd, v]) ?? '')), () => c.w.line('')); });
def('PluginLoad', async (c) => { const p = await c.r.line(); const l = await c.r.int(); await c.reply(async () => c.w.int((await c.call('PluginLoad', [p, l])) ? 1 : 0), () => c.w.int(0)); });
def('BuildMechanism', async (c) => {
  const base = await c.item(); const name = await c.r.line(); const type = await c.r.int(); const n = await c.r.int(); const list: Ref[] = []; for (let k = 0; k < n; k++) list.push(await c.item());
  const pb = await c.r.pose(); const pt = await c.r.pose(); const params = await c.r.array(); const lim = await c.r.matrix();
  await c.reply(async () => c.wItem(await c.call('BuildMechanism', [type, list.map(it), params, lim.length >= 3 ? lim[0] : null, lim.length >= 3 ? lim[1] : null, lim.length >= 3 ? lim[2] : null, name, P(pb), P(pt), it(base)])), () => c.wItem(null));
});
const desktopOnly = (what: string) => () => { throw new Error(`${what} needs the RoboDK desktop application (mouse / window): not available in VerticalBot Studio`); };
def('Proj2d3d', async (c) => { await c.r.int(); await c.r.int(); await c.reply(desktopOnly('CursorXYZ'), () => { c.w.int(0); c.wItem(null); c.w.xyz(null); }); });
def('WinProcDock', async (c) => { await c.r.line(); await c.r.line(); await c.r.array(); await c.r.line(); await c.r.int(); await c.r.int(); await c.r.int(); await c.reply(desktopOnly('EmbedWindow'), () => c.w.int(0)); });
def('G_ObjPoint', async (c) => {
  const i = await c.item(); const ft = await c.r.int(); const fid = await c.r.int();
  if (i) await c.reply(async () => { const r = await c.call('GetPoints', [ft, fid], i); c.w.matrix(cols(matOf(Array.isArray(r) ? r[0] : r))); c.w.line(String(Array.isArray(r) ? r[1] ?? '' : '')); }, () => { c.w.matrix([]); c.w.line(''); });
  else await c.reply(desktopOnly('GetPoints(FEATURE_HOVER_*)'), () => { if (ft === 10) c.w.matrix([]); c.wItem(null); c.w.int(0); c.w.int(0); c.w.int(0); c.w.line(''); });
});
def('MeasLT2', async (c) => { const est = await c.r.xyz(); const search = await c.r.int(); await c.reply(async () => c.w.array(numsOf(await c.call('LaserTracker_Measure', [est, !!search]))), () => c.w.array([])); });
def('MeasPose4', async (c) => { const a = await c.r.array(); await c.reply(async () => { const r = await c.call('MeasurePose', [a[0] ?? -1, a[1] ?? 0, a.length >= 5 ? a.slice(2, 5) : null]); c.w.pose(rowsOf(Array.isArray(r) ? r[0] : r)); c.w.array(numsOf(Array.isArray(r) ? r.slice(1) : [])); }, () => { c.w.pose(null); c.w.array([]); }); });
def('CalibTCP3', async (c) => {
  const m = await c.r.matrix(); const fmt = await c.r.int(); const alg = await c.r.array(); const robot = await c.item(); const tool = await c.item();
  await c.reply(async () => { const r = await c.call('CalibrateTool', [colsToList(m), fmt, alg[0] ?? 0, it(robot), it(tool)]); c.w.array(numsOf(r?.[0])); c.w.array(numsOf(r?.[1])); c.w.matrix(cols((r?.[2] ?? []).map((x: any) => numsOf(x)))); }, () => { c.w.array([]); c.w.array([]); c.w.matrix([]); });
});
def('CalibFrame', async (c) => {
  const m = await c.r.matrix(); const uj = await c.r.int(); const method = await c.r.int(); const robot = await c.item();
  await c.reply(async () => { const r = await c.call('CalibrateReference', [colsToList(m), method, uj !== 0, it(robot)]); c.w.pose(rowsOf(Array.isArray(r) && !rowsOf(r) ? r[0] : r)); c.w.array(numsOf(Array.isArray(r) && !rowsOf(r) ? r[1] : [])); }, () => { c.w.pose(null); c.w.array([]); });
});

// ---- items --------------------------------------------------------------------------------------------------------
const get = (method: string, write: (c: Ctx, r: any) => void, fallback: (c: Ctx) => void) => async (c: Ctx) => { const i = await c.item(); await c.reply(async () => write(c, await c.call(method, [], i)), () => fallback(c)); };
const set = (method: string, read: (c: Ctx) => Promise<any[]>) => async (c: Ctx) => { const i = await c.item(); const a = await read(c); await c.reply(() => c.call(method, a, i)); };
const wInt = (c: Ctx, r: any) => c.w.int(typeof r === 'boolean' ? (r ? 1 : 0) : Number(r) || 0);
def('G_Item_Type', get('Type', wInt, (c) => c.w.int(-1)));
def('CopyFaces', async (c) => { const i = await c.item(); const from = await c.item(); const p = await c.r.pose(); await c.reply(() => c.call('AddGeometry', [it(from), P(p)], i)); });
def('Remove', async (c) => { const i = await c.item(); await c.reply(() => c.call('Delete', [], i)); });
def('S_Parent', async (c) => { const i = await c.item(); const p = await c.item(); await c.reply(() => c.call('setParent', [it(p)], i)); });
def('S_Parent_Static', async (c) => { const i = await c.item(); const p = await c.item(); await c.reply(() => c.call('setParentStatic', [it(p)], i)); });
def('Attach_Closest2', async (c) => { const i = await c.item(); const kw = await c.r.line(); const tol = await c.r.array(); const n = await c.r.int(); const l: Ref[] = []; for (let k = 0; k < n; k++) l.push(await c.item()); await c.reply(async () => c.wItem(await c.call('AttachClosest', [kw, tol[0] ?? -1, l.map(it)], i)), () => c.wItem(null)); });
def('Attach_Closest', async (c) => { const i = await c.item(); await c.reply(async () => c.wItem(await c.call('AttachClosest', [], i)), () => c.wItem(null)); });
def('Detach_Closest', async (c) => { const i = await c.item(); const p = await c.item(); await c.reply(async () => c.wItem(await c.call('DetachClosest', [it(p)], i)), () => c.wItem(null)); });
def('Detach_All', async (c) => { const i = await c.item(); const p = await c.item(); await c.reply(() => c.call('DetachAll', [it(p)], i)); });
def('G_Parent', get('Parent', (c, r) => c.wItem(r), (c) => c.wItem(null)));
def('G_Childs', get('Childs', (c, r) => c.wItems(r), (c) => c.w.int(0)));
def('G_Visible', get('Visible', wInt, (c) => c.w.int(0)));
def('S_Visible', set('setVisible', async (c) => { const v = await c.r.int(); const f = await c.r.int(); return [v, f]; }));
def('G_Name', get('Name', (c, r) => c.w.line(String(r ?? '')), (c) => c.w.line('')));
def('S_Name', set('setName', async (c) => [await c.r.line()]));
def('S_ValueMat', async (c) => { const i = await c.item(); const k = await c.r.line(); const m = await c.r.matrix(); await c.reply(async () => { const r = await c.call('setValue', [k, M(m)], i); c.w.matrix(r && (r.$mat || Array.isArray(r)) ? matOf(r) : m); }, () => c.w.matrix([])); });
def('S_Gen_Str', set('setValue', async (c) => { const k = await c.r.line(); return [k, await c.r.line()]; }));
def('ICMD', async (c) => { const i = await c.item(); const k = await c.r.line(); const v = await c.r.line(); await c.reply(async () => { const r = await c.call('setParam', [k, v], i); c.w.line(typeof r === 'object' && r ? JSON.stringify(r) : String(r ?? '')); }, () => c.w.line('')); });
def('BlindICMD', async (c) => { const i = await c.item(); const k = await c.r.line(); const v = await c.r.line(); await c.call('setParam', [k, v], i).catch(() => {}); });
def('G_ItmDataParam', async (c) => { const i = await c.item(); const k = await c.r.line(); await c.reply(async () => { const v = await c.call('getParam', [k], i); c.w.bytes(v && v.$bytes ? Buffer.from(v.$bytes, 'base64') : Buffer.from(v == null ? '' : typeof v === 'object' ? JSON.stringify(v) : String(v), 'utf8')); }, () => c.w.bytes(null)); });
def('S_ItmDataParam', async (c) => { const i = await c.item(); const k = await c.r.line(); const v = await c.r.bytes(); await c.reply(() => c.call('setParam', [k, { $bytes: v.toString('base64') }], i)); });
def('S_Hlocal', set('setPose', async (c) => [P(await c.r.pose())]));
def('G_Hlocal', get('Pose', (c, r) => c.w.pose(rowsOf(r)), (c) => c.w.pose(null)));
def('S_Hgeo2', set('setGeometryPose', async (c) => { const p = await c.r.pose(); return [P(p), !!(await c.r.int())]; }));
def('S_Hgeo', set('setGeometryPose', async (c) => [P(await c.r.pose())]));
def('G_Hgeom', get('GeometryPose', (c, r) => c.w.pose(rowsOf(r)), (c) => c.w.pose(null)));
def('S_Hlocal_Abs', set('setPoseAbs', async (c) => [P(await c.r.pose())]));
def('G_Hlocal_Abs', get('PoseAbs', (c, r) => c.w.pose(rowsOf(r)), (c) => c.w.pose(null)));
def('Recolor', set('Recolor', async (c) => { const a = await c.r.array(); const to = a.slice(0, 4), from = a.length >= 8 ? a.slice(4, 8) : null; return [to, from, a[8] ?? 0.1]; }));
def('S_Color', set('setColor', async (c) => [await c.r.array()]));
def('S_ShapeColor', set('setColorShape', async (c) => { const k = await c.r.int(); return [await c.r.array(), k]; }));
def('S_CurveColor', set('setColorCurve', async (c) => { const k = await c.r.int(); return [await c.r.array(), k]; }));
def('G_Color', get('Color', (c, r) => c.w.array(numsOf(r)), (c) => c.w.array([])));
def('Scale', set('Scale', async (c) => [await c.r.array()]));
def('TrScale', async (c) => { const i = await c.item(); const s = await c.r.array(); const pre = await c.r.pose(); const post = await c.r.pose(); await c.reply(async () => { await c.call('Scale', [s, P(pre), P(post)], i); c.w.int(1); }, () => c.w.int(0)); });
def('G_ObjSelection', get('SelectedFeature', (c, r) => { c.w.int(r?.[0] ?? 0); c.w.int(r?.[1] ?? 0); c.w.int(r?.[2] ?? 0); }, (c) => { c.w.int(0); c.w.int(0); c.w.int(0); }));
def('S_MachiningParams', async (c) => { const i = await c.item(); const nc = await c.r.line(); const part = await c.item(); const prm = await c.r.line(); await c.reply(async () => { const r = await c.call('setMachiningParameters', [nc, it(part), prm], i); c.wItem(r?.[0] ?? null); c.w.int(Number(r?.[1] ?? -1) * 1000); }, () => { c.wItem(null); c.w.int(-1000); }); });
def('S_Target_As_RT', set('setAsCartesianTarget', async () => []));
def('S_Target_As_JT', set('setAsJointTarget', async () => []));
def('Target_Is_JT', get('isJointTarget', wInt, (c) => c.w.int(0)));
def('G_Thetas', get('Joints', (c, r) => c.w.array(numsOf(r)), (c) => c.w.array([])));
def('G_Thetas_Sim', get('SimulatorJoints', (c, r) => c.w.array(numsOf(r)), (c) => c.w.array([])));
def('G_LinkPoses', async (c) => { const i = await c.item(); const q = await c.r.array(); await c.reply(async () => { const l = await c.call('JointPoses', q.length ? [q] : [], i); c.w.int(l.length); for (const p of l) c.w.pose(rowsOf(p)); }, () => c.w.int(0)); });
def('G_Home', get('JointsHome', (c, r) => c.w.array(numsOf(r)), (c) => c.w.array([])));
def('S_Home', async (c) => { const q = await c.r.array(); const i = await c.item(); await c.reply(() => c.call('setJointsHome', [q], i)); });
def('G_LinkObjId', async (c) => { const i = await c.item(); const id = await c.r.int(); await c.reply(async () => c.wItem(await c.call('ObjectLink', [id], i)), () => c.wItem(null)); });
def('G_LinkType', async (c) => { const i = await c.item(); const t = await c.r.int(); await c.reply(async () => c.wItem(await c.call('getLink', [t], i)), () => c.wItem(null)); });
def('S_Link_ptr', async (c) => {
  // Item.setLink(item) and Item.setPoseFrame(frame item) share this command: a frame linked to a robot/program/project is its reference
  const link = await c.item(); const i = await c.item();
  await c.reply(async () => { const t = link ? await c.call('Type', [], link) : -1; await c.call(t === 3 ? 'setPoseFrame' : 'setLink', [it(link)], i); });
});
def('S_Thetas', async (c) => { const q = await c.r.array(); const i = await c.item(); await c.reply(() => c.call('setJoints', [q], i)); });
def('G_RobLimits', get('JointLimits', (c, r) => { c.w.array(numsOf(r?.[0])); c.w.array(numsOf(r?.[1])); c.w.int(r?.[2] ?? 0); }, (c) => { c.w.array([]); c.w.array([]); c.w.int(0); }));
def('S_RobLimits', set('setJointLimits', async (c) => { const lo = await c.r.array(); return [lo, await c.r.array()]; }));
def('S_Robot', set('setRobot', async (c) => [it(await c.item())]));
def('S_Frame', async (c) => { const p = await c.r.pose(); const i = await c.item(); await c.reply(() => c.call('setPoseFrame', [P(p)], i)); });
def('S_Tool_ptr', async (c) => { const t = await c.item(); const i = await c.item(); await c.reply(() => c.call('setPoseTool', [it(t)], i)); });
def('S_Tool', async (c) => { const p = await c.r.pose(); const i = await c.item(); await c.reply(() => c.call('setPoseTool', [P(p)], i)); });
def('G_Tool', get('PoseTool', (c, r) => c.w.pose(rowsOf(r)), (c) => c.w.pose(null)));
def('G_Frame', get('PoseFrame', (c, r) => c.w.pose(rowsOf(r)), (c) => c.w.pose(null)));
def('AddToolEmpty', async (c) => { const i = await c.item(); const p = await c.r.pose(); const n = await c.r.line(); await c.reply(async () => c.wItem(await c.call('AddTool', [P(p), n], i)), () => c.wItem(null)); });
def('G_FK', async (c) => { const q = await c.r.array(); const i = await c.item(); await c.reply(async () => c.w.pose(rowsOf(await c.call('SolveFK', [q], i))), () => c.w.pose(null)); });
def('G_Thetas_Config', async (c) => { const q = await c.r.array(); const i = await c.item(); await c.reply(async () => c.w.array(numsOf(await c.call('JointsConfig', [q], i))), () => c.w.array([])); });
def('G_IK', async (c) => { const p = await c.r.pose(); const i = await c.item(); await c.reply(async () => c.w.array(numsOf(await c.call('SolveIK', [P(p)], i))), () => c.w.array([])); });
def('G_IK_jnts', async (c) => { const p = await c.r.pose(); const q = await c.r.array(); const i = await c.item(); await c.reply(async () => c.w.array(numsOf(await c.call('SolveIK', [P(p), q], i))), () => c.w.array([])); });
def('G_IK_cmpl', async (c) => { const p = await c.r.pose(); const i = await c.item(); await c.reply(async () => { const l = await c.call('SolveIK_All', [P(p)], i); c.w.matrix(cols(matOf(l))); }, () => c.w.matrix([])); });
def('FilterTarget', async (c) => { const p = await c.r.pose(); const q = await c.r.array(); const i = await c.item(); await c.reply(async () => { const r = await c.call('FilterTarget', [P(p), q], i); c.w.pose(rowsOf(r?.[0])); c.w.array(numsOf(r?.[1])); }, () => { c.w.pose(null); c.w.array([]); }); });
def('Connect2', async (c) => { const i = await c.item(); const ip = await c.r.line(); const b = await c.r.int(); await c.reply(async () => c.w.int(await c.call('Connect', [ip, !!b], i)), () => c.w.int(-2)); });
def('ConnectParams', get('ConnectionParams', (c, r) => { c.w.line(String(r?.[0] ?? '')); c.w.int(r?.[1] ?? 0); c.w.line(String(r?.[2] ?? '')); c.w.line(String(r?.[3] ?? '')); c.w.line(String(r?.[4] ?? '')); }, (c) => { c.w.line(''); c.w.int(0); c.w.line(''); c.w.line(''); c.w.line(''); }));
def('setConnectParams', set('setConnectionParams', async (c) => { const ip = await c.r.line(); const port = await c.r.int(); const path = await c.r.line(); const user = await c.r.line(); return [ip, port, path, user, await c.r.line()]; }));
def('ConnectedState', get('ConnectedState', (c, r) => { c.w.int(Array.isArray(r) ? r[0] : r); c.w.line(String(Array.isArray(r) ? r[1] ?? '' : '')); }, (c) => { c.w.int(-2); c.w.line(''); }));
def('Disconnect', get('Disconnect', wInt, (c) => c.w.int(0)));
def('CollisionMove', async (c) => { const i = await c.item(); const a = await c.r.array(); const b = await c.r.array(); const st = await c.r.int(); await c.reply(async () => c.w.int(await c.call('MoveJ_Test', [a, b, st / 1000], i)), () => c.w.int(-1)); });
def('CollisionMoveBlend', async (c) => { const i = await c.item(); const a = await c.r.array(); const b = await c.r.array(); const d = await c.r.array(); const st = await c.r.int(); const bl = await c.r.int(); await c.reply(async () => c.w.int(await c.call('MoveJ_Test_Blend', [a, b, d, bl / 1000, st / 1000], i)), () => c.w.int(-1)); });
def('CollisionMoveL', async (c) => { const i = await c.item(); const a = await c.r.array(); const p = await c.r.pose(); const st = await c.r.int(); await c.reply(async () => c.w.int(await c.call('MoveL_Test', [a, P(p), st / 1000], i)), () => c.w.int(-1)); });
def('S_Speed4', set('setSpeed', async (c) => { const v = await c.r.array(); return [v[0] ?? -1, v[1] ?? -1, v[2] ?? -1, v[3] ?? -1]; }));
def('S_Speed', set('setSpeed', async (c) => [await c.r.int() / 1000]));
def('S_ZoneData', async (c) => { const r = await c.r.int(); const i = await c.item(); await c.reply(() => c.call('setRounding', [r / 1000], i)); });
def('Show_SeqPoses', async (c) => { const i = await c.item(); const opts = await c.r.array(); const n = await c.r.int(); const joints = (opts[0] ?? -1) > 0 && ((opts[0] | 0) & 2048) !== 0; const l: any[] = []; for (let k = 0; k < n; k++) l.push(joints ? await c.r.array() : P(await c.r.pose())); await c.reply(() => c.call('ShowSequence', [l, opts[0] ?? -1, opts[1] ?? -1], i)); });
def('Show_Seq', async (c) => { const m = await c.r.matrix(); const i = await c.item(); await c.reply(() => c.call('ShowSequence', [colsToList(m)], i)); });
def('IsBusy', get('Busy', wInt, (c) => c.w.int(0)));
def('Stop', async (c) => { const i = await c.item(); await c.reply(() => c.call('Stop', [], i)); });
def('WaitMove', async (c) => { const i = await c.item(); await c.reply(() => undefined); c.w.flush(); await c.reply(() => c.call('WaitMove', [], i)); });
def('S_AbsAccOn', set('setAccuracyActive', async (c) => [await c.r.int()]));
def('G_AbsAccOn', get('AccuracyActive', wInt, (c) => c.w.int(0)));
def('S_ParamCalibTool', async (c) => { const i = await c.item(); await c.r.array(); await c.reply(() => c.call('Valid', [], i)); });
def('FilterProg2', async (c) => { const i = await c.item(); const f = await c.r.line(); await c.reply(async () => { const r = await c.call('FilterProgram', [f], i); c.w.int(Array.isArray(r) ? r[0] : r); c.w.line(String(Array.isArray(r) ? r[1] ?? '' : '')); }, () => { c.w.int(-1); c.w.line(''); }); });
def('MakeProg2', async (c) => { const i = await c.item(); const f = await c.r.line(); const mode = await c.r.int(); await c.reply(async () => { const r = await c.call('MakeProgram', [f, mode], i); c.w.int(r?.[0] ? 1 : -1); c.w.line(String(r?.[1] ?? '')); c.w.int(r?.[2] ? 1 : -1); }, () => { c.w.int(-1); c.w.line(''); c.w.int(-1); }); });
def('S_ProgRunType', set('setRunType', async (c) => [await c.r.int()]));
def('G_ProgRunType', get('RunType', wInt, (c) => c.w.int(1)));
def('RunProgParam', async (c) => { const i = await c.item(); const p = await c.r.line(); await c.reply(async () => c.w.int(await c.call('RunCode', [p ? p.split(',') : []], i)), () => c.w.int(-1)); });
def('RunProg', async (c) => { const i = await c.item(); await c.reply(async () => c.w.int(await c.call('RunCode', [], i)), () => c.w.int(-1)); });
def('RunCode2', async (c) => { const i = await c.item(); const code = await c.r.line(); const t = await c.r.int(); await c.reply(async () => { const r = await c.call('RunInstruction', [code, t], i); c.w.int(typeof r === 'number' ? r : 0); }, () => c.w.int(-1)); });
def('RunPause', async (c) => { const i = await c.item(); const ms = await c.r.int(); await c.reply(() => c.call('Pause', [ms / 1000], i)); });
def('setDO', set('setDO', async (c) => { const k = await c.r.line(); return [k, await c.r.line()]; }));
def('setAO', set('setAO', async (c) => { const k = await c.r.line(); return [k, await c.r.line()]; }));
def('getDI', async (c) => { const i = await c.item(); const k = await c.r.line(); await c.reply(async () => c.w.line(String(await c.call('getDI', [k], i) ?? '')), () => c.w.line('')); });
def('getAI', async (c) => { const i = await c.item(); const k = await c.r.line(); await c.reply(async () => c.w.line(String(await c.call('getAI', [k], i) ?? '')), () => c.w.line('')); });
def('waitDI', set('waitDI', async (c) => { const k = await c.r.line(); const v = await c.r.line(); return [k, v, await c.r.int()]; }));
def('InsCustom2', set('customInstruction', async (c) => { const n = await c.r.line(); const p = await c.r.line(); const py = await c.r.line(); const ic = await c.r.line(); return [n, p, py, ic, await c.r.int()]; }));
def('Add_INSMOVE', async (c) => { const t = await c.item(); const i = await c.item(); const mt = await c.r.int(); await c.reply(() => c.call(mt === 2 ? 'MoveL' : mt === 5 ? 'SearchL' : 'MoveJ', [it(t)], i)); });
def('Add_INSMOVEC', async (c) => { const a = await c.item(); const b = await c.item(); const i = await c.item(); await c.reply(() => c.call('MoveC', [it(a), it(b)], i)); });
def('Prog_ShowIns', set('ShowInstructions', async (c) => [!!(await c.r.int())]));
def('Prog_ShowTargets', set('ShowTargets', async (c) => [!!(await c.r.int())]));
def('Prog_Nins', get('InstructionCount', wInt, (c) => c.w.int(0)));
def('Prog_SelIns', async (c) => { const i = await c.item(); const k = await c.r.int(); await c.reply(async () => c.w.int(await c.call('InstructionSelect', [k], i)), () => c.w.int(-1)); });
def('Prog_DelIns', async (c) => { const i = await c.item(); const k = await c.r.int(); await c.reply(async () => c.w.int((await c.call('InstructionDelete', [k], i)) ? 1 : 0), () => c.w.int(0)); });
def('Prog_GIns', async (c) => {
  const i = await c.item(); const k = await c.r.int();
  await c.reply(async () => {
    const r = await c.call('Instruction', [k], i);
    const a = Array.isArray(r) ? r : [r?.name, r?.type, r?.moveType, r?.isJointTarget, r?.pose, r?.joints];
    c.w.line(String(a[0] ?? '')); c.w.int(a[1] ?? -1);
    if (a[1] === 0) { c.w.int(a[2] ?? 1); c.w.int(a[3] ?? 0); c.w.pose(rowsOf(a[4])); c.w.array(numsOf(a[5])); }
  }, () => { c.w.line(''); c.w.int(-1); });
});
def('Prog_SIns', async (c) => {
  const i = await c.item(); const k = await c.r.int(); const name = await c.r.line(); const t = await c.r.int();
  let mv: any[] = [-1, 0, null, []];
  if (t === 0) { const mt = await c.r.int(); const jt = await c.r.int(); const p = await c.r.pose(); mv = [mt, jt, P(p), await c.r.array()]; }
  await c.reply(() => c.call('setInstruction', [k, name, t, ...mv], i));
});
def('Update2', async (c) => { const i = await c.item(); const a = await c.r.array(); await c.reply(async () => { const r = await c.call('Update', [a[0] ?? 0, 3600, a[1] ?? -1, a[2] ?? -1], i); c.w.array([r?.[0] ?? 0, r?.[1] ?? 0, r?.[2] ?? 0, r?.[3] ?? 0]); c.w.line(String(r?.[4] ?? '')); }, () => { c.w.array([0, 0, 0, -1]); c.w.line(''); }); });
// the API returns one row per instruction / sample; RoboDK matrices hold them as columns
def('G_ProgInsList', async (c) => { const i = await c.item(); await c.reply(async () => { const r = await c.call('InstructionList', [], i); c.w.matrix(cols(matOf(r?.[0]))); c.w.int(r?.[1] ?? 0); }, () => { c.w.matrix([]); c.w.int(-1); }); });
def('G_ProgJointList', async (c) => {
  const i = await c.item(); const a = await c.r.array(); const file = await c.r.line();
  await c.reply(async () => {
    const r = await c.call('InstructionListJoints', [a[0] ?? 10, a[1] ?? 5, file || '', a[2] ?? 0, a[3] ?? 0, a[4] ?? 0.1], i);
    if (!file) c.w.matrix(cols(matOf(r?.[1])));
    c.w.int(r?.[2] ?? 0); c.w.line(String(r?.[0] ?? ''));
  }, () => { if (!file) c.w.matrix([]); c.w.int(-1); c.w.line(''); });
});

// ---- moves (RoboDK _moveX / MoveC): status after validation, a second status after the motion when blocking ----
async function readTarget(c: Ctx): Promise<any> {
  const kind = await c.r.int(); const values = await c.r.array(); const target = await c.item();
  if (kind === 3) return it(target);
  if (kind === 2) return P([0, 1, 2, 3].map((r) => [0, 1, 2, 3].map((col) => values[col * 4 + r])));   // rows of tr(pose): column-major
  return values;
}
const move = (blocking: boolean, circular: boolean) => async (c: Ctx) => {
  const mt = await c.r.int();
  const t1 = await readTarget(c);
  const t2 = circular ? await readTarget(c) : null;
  const robot = await c.item();
  const method = circular ? 'MoveC' : mt === 2 ? 'MoveL' : mt === 5 ? 'SearchL' : 'MoveJ';
  let ok = true;
  await c.reply(async () => { try { await c.call(method, circular ? [t1, t2, false] : [t1, false], robot); } catch (e) { ok = false; throw e; } });
  if (!blocking || !ok) return;            // a rejected move raises on the first status: no second one
  c.w.flush();
  await c.reply(() => c.call('WaitMove', [], robot));
};
def('MoveX', move(false, false));
def('MoveXb', move(true, false));
def('MoveC', move(false, true));
def('MoveCb', move(true, true));

// ---- server -------------------------------------------------------------------------------------------------------
const STATUS: Array<[RegExp, number]> = [[/^TargetReachError/, 10], [/^StoppedError/, 11], [/^InputError/, 12], [/^LicenseError/, 13]];

export function startRdkProtocolServer(opts: RdkProtocolOptions): Promise<RdkProtocolServer> {
  const log = opts.log ?? ((m: string) => console.log(`[rdk-protocol] ${m}`));
  const ptrs = new Pointers();
  let seq = 0;
  const server = net.createServer((sock) => {
    sock.setNoDelay(true);
    const r = new Reader();
    const w = new Writer(sock);
    sock.on('data', (d) => r.push(d));
    sock.on('end', () => r.end());
    sock.on('close', () => r.end());
    sock.on('error', () => r.end());
    const encode = (v: any): any => v;
    const call = async (method: string, params: any[] = [], target?: Ref): Promise<any> => {
      const req: RpcRequest = { id: `rdk${++seq}`, method, params: params.map(encode) };
      if (target !== undefined) req.target = target && target.startsWith('__unknown_') ? null : target;
      const res = await opts.handle(req);
      if (res.error) throw new Error(res.error);
      return res.result;
    };
    const wItem = (v: any) => { const id = v && typeof v === 'object' && '$item' in v ? v.$item : null; w.u64(ptrs.ptr(id)); w.int(id ? Number(v.type ?? -1) : -1); };
    const ctx: Ctx = {
      r, w,
      item: async () => ptrs.id(await r.u64()),
      ptr: async () => r.u64(),
      call,
      wItem,
      wItems: (list: any[]) => { const l = Array.isArray(list) ? list : []; w.int(l.length); for (const x of l) wItem(x); },
      ptrOf: (v: any) => ptrs.ptr(v && v.$item ? v.$item : null),
      idOf: (p: bigint) => ptrs.id(p),
      reply: async (body, fallback) => {
        try { await body(); w.int(0); }
        catch (e: any) {
          w.discard();
          fallback?.();
          const msg = String(e?.message ?? e);
          const st = STATUS.find(([re]) => re.test(msg));
          w.int(st ? st[1] : 3);
          w.line(st ? msg.replace(/^\w+Error:?\s*/, '') : msg);
        }
      },
    };
    (async () => {
      try {
        // handshake: new clients send RDK_API + [SAFE_MODE, AUTO_UPDATE, skipstatus]; old ones CMD_START + "1 0"
        const hello = await r.line();
        if (hello === 'RDK_API') {
          const a = await r.array();
          void a;
          w.line('RDK_API'); w.int(1); w.int(BUILD); w.int(0);
          w.flush();
        } else if (hello === 'CMD_START') {
          const opt = await r.line();
          if (!/\s1\s*$/.test(opt)) { w.line('READY'); w.flush(); }
        } else { sock.destroy(); return; }
        for (;;) {
          const cmd = await r.line();
          if (!cmd) continue;
          const h = H[cmd];
          ctx.cmd = cmd;
          if (!h) { w.int(3); w.line(`Unknown RoboDK API command ${cmd} (not supported by VerticalBot Studio)`); w.flush(); log(`unsupported command ${cmd}`); continue; }
          await h(ctx);
          w.flush();
        }
      } catch (e) {
        if (!(e instanceof Closed)) log(`connection error: ${(e as Error)?.message ?? e}`);
        sock.destroy();
      }
    })();
  });
  return new Promise((resolve) => server.listen(opts.port ?? 0, opts.host ?? '127.0.0.1', () => {
    const port = (server.address() as net.AddressInfo).port;
    resolve({ server, port, close: () => new Promise<void>((res) => server.close(() => res())) });
  }));
}
