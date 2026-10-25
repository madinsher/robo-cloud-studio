/**
 * Native station file (.vbstation / .json): the serialized item tree plus an asset manifest.
 * The file is self-contained: every mesh the tree references is embedded, either as its original
 * STL/OBJ/DAE bytes (base64) or, for geometry registered without source bytes (RoboDK binary import,
 * glTF, STEP/IGES, zip containers, API AddShape), as the serialized MeshData itself. Unreferenced
 * assets left in the store by earlier stations are not written.
 */
import { Station, SerializedItem } from '../core/items/item';
import '../core/items/robot';
import '../core/items/program';
import '../mobile/items';
import '../vc/component';
import '../agri/items';
import { AssetStore } from '../scene/assets';
import type { MeshData } from './mesh/stl';

export interface StationFileAsset {
  /** 'mesh': `data` is a packed MeshData (JSON with base64 typed arrays); otherwise base64 file bytes. */
  type: 'stl' | 'glb' | 'obj' | 'dae' | 'mesh';
  data: string;
  name?: string;
  units?: 'mm' | 'm';
  /** Scale the source bytes were parsed with (STL/OBJ in metres from containers). */
  scale?: number;
}

export interface StationFile {
  format: 'vbstation';
  version: 1;
  savedAt: string;
  app: string;
  station: SerializedItem;
  assets: Record<string, StationFileAsset>;
}

/** Every mesh id referenced anywhere in a serialized tree (geometry, colliders, robot link visuals, components...). */
export function referencedMeshIds(tree: unknown): Set<string> {
  const out = new Set<string>();
  const visit = (v: unknown) => {
    if (!v || typeof v !== 'object') return;
    if (Array.isArray(v)) { for (const x of v) visit(x); return; }
    for (const [k, x] of Object.entries(v as Record<string, unknown>)) {
      if (k === 'mesh' && typeof x === 'string' && x) out.add(x);
      else if (x && typeof x === 'object') visit(x);
    }
  };
  visit(tree);
  return out;
}

export function saveStation(station: Station, assets?: AssetStore): StationFile {
  const tree = station.serialize();
  const file: StationFile = {
    format: 'vbstation',
    version: 1,
    savedAt: new Date().toISOString(),
    app: 'VerticalBot Studio',
    station: tree,
    assets: {},
  };
  if (assets) {
    for (const ref of referencedMeshIds(tree)) {
      const a = assets.resolve(ref);
      if (!a) continue;
      const meta = { name: a.name, ...(a.units ? { units: a.units } : {}), ...(a.scale && a.scale !== 1 ? { scale: a.scale } : {}) };
      // source bytes re-parse synchronously for STL/OBJ/DAE; anything else (glTF, parsed-only geometry) stores the MeshData
      if (a.source && a.type !== 'glb') file.assets[ref] = { type: a.type, data: bytesToBase64(a.source), ...meta };
      else if (a.mesh) file.assets[ref] = { type: 'mesh', data: JSON.stringify(packValue(a.mesh)), ...meta };
    }
  }
  return file;
}

export function loadStation(file: StationFile | SerializedItem, assets?: AssetStore): Station {
  const data = (file as StationFile).format === 'vbstation' ? (file as StationFile) : null;
  const tree = data ? data.station : (file as SerializedItem);
  if (data && assets) {
    for (const [id, a] of Object.entries(data.assets ?? {})) {
      if (a.type === 'mesh') assets.registerMesh(id, unpackValue(JSON.parse(a.data)) as MeshData, a.name, a.units);
      else assets.registerRaw(id, a.type, base64ToBytes(a.data), a.name, a.scale ?? 1, a.units);
    }
  }
  return Station.deserialize(tree);
}

type TypedArray = Float32Array | Float64Array | Int8Array | Int16Array | Int32Array | Uint8Array | Uint16Array | Uint32Array | Uint8ClampedArray;
const TYPED: Record<string, new (b: ArrayBuffer) => TypedArray> = { Float32Array, Float64Array, Int8Array, Int16Array, Int32Array, Uint8Array, Uint16Array, Uint32Array, Uint8ClampedArray };

/** JSON-safe copy of a value whose typed arrays (at any depth) become {$ta: ctor name, b64}. */
export function packValue(v: unknown): unknown {
  if (ArrayBuffer.isView(v) && !(v instanceof DataView)) {
    const ta = v as TypedArray;
    return { $ta: ta.constructor.name, b64: bytesToBase64(new Uint8Array(ta.buffer, ta.byteOffset, ta.byteLength)) };
  }
  if (Array.isArray(v)) return v.map(packValue);
  if (v && typeof v === 'object') { const o: Record<string, unknown> = {}; for (const [k, x] of Object.entries(v)) if (typeof x !== 'function' && x !== undefined) o[k] = packValue(x); return o; }
  return v;
}
export function unpackValue(v: unknown): unknown {
  if (Array.isArray(v)) return v.map(unpackValue);
  if (v && typeof v === 'object') {
    const o = v as Record<string, unknown>;
    if (typeof o.$ta === 'string' && typeof o.b64 === 'string' && TYPED[o.$ta]) {
      const bytes = base64ToBytes(o.b64);
      const buf = bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength) as ArrayBuffer;
      return new TYPED[o.$ta](buf);
    }
    const out: Record<string, unknown> = {};
    for (const [k, x] of Object.entries(o)) out[k] = unpackValue(x);
    return out;
  }
  return v;
}

export function bytesToBase64(bytes: Uint8Array): string {
  if (typeof Buffer !== 'undefined') return Buffer.from(bytes.buffer, bytes.byteOffset, bytes.byteLength).toString('base64');
  let s = '';
  for (let i = 0; i < bytes.length; i += 0x8000) s += String.fromCharCode(...bytes.subarray(i, i + 0x8000));
  return btoa(s);
}
export function base64ToBytes(b64: string): Uint8Array {
  if (typeof Buffer !== 'undefined') return new Uint8Array(Buffer.from(b64, 'base64'));
  const s = atob(b64);
  const out = new Uint8Array(s.length);
  for (let i = 0; i < s.length; i++) out[i] = s.charCodeAt(i);
  return out;
}
