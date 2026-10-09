/**
 * Native station file (.vbstation / .json): the serialized item tree plus an asset manifest.
 * Mesh assets are stored as base64 STL/GLB blobs so a station is a single portable file.
 */
import { Station, SerializedItem } from '../core/items/item';
import '../core/items/robot';
import '../core/items/program';
import '../mobile/items';
import '../vc/component';
import '../agri/items';
import { AssetStore } from '../scene/assets';

export interface StationFile {
  format: 'vbstation';
  version: 1;
  savedAt: string;
  app: string;
  station: SerializedItem;
  assets: Record<string, { type: 'stl' | 'glb' | 'obj' | 'dae'; data: string; name?: string; units?: 'mm' | 'm' }>;
}

export function saveStation(station: Station, assets?: AssetStore): StationFile {
  const file: StationFile = {
    format: 'vbstation',
    version: 1,
    savedAt: new Date().toISOString(),
    app: 'VerticalBot Studio',
    station: station.serialize(),
    assets: {},
  };
  if (assets) {
    for (const [id, a] of assets.entries()) {
      if (a.source) file.assets[id] = { type: a.type, data: bytesToBase64(a.source), name: a.name, ...(a.units ? { units: a.units } : {}) };
    }
  }
  return file;
}

export function loadStation(file: StationFile | SerializedItem, assets?: AssetStore): Station {
  const data = (file as StationFile).format === 'vbstation' ? (file as StationFile) : null;
  const tree = data ? data.station : (file as SerializedItem);
  if (data && assets) {
    for (const [id, a] of Object.entries(data.assets ?? {})) assets.registerRaw(id, a.type, base64ToBytes(a.data), a.name, 1, a.units);
  }
  return Station.deserialize(tree);
}

export function bytesToBase64(bytes: Uint8Array): string {
  if (typeof Buffer !== 'undefined') return Buffer.from(bytes).toString('base64');
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
