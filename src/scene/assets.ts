/** Mesh asset registry (renderer-agnostic). Stores raw bytes and parsed geometry. */
import { parseSTL, MeshData } from '../io/mesh/stl';
import { parseOBJ } from '../io/mesh/obj';
import { parseDAE } from '../io/mesh/dae';

export interface Asset {
  id: string;
  type: 'stl' | 'glb' | 'obj' | 'dae';
  name?: string;
  source?: Uint8Array;
  mesh?: MeshData;
  /** Optional renderer object cache (three.js BufferGeometry). */
  gpu?: unknown;
  /** Units the parsed geometry is expressed in ('mm' default). URDF meshes are kept in metres and scaled by the visual. */
  units?: 'mm' | 'm';
  /** Coordinate scale applied when `source` was parsed (zip containers in metres register with 1000). */
  scale?: number;
}

const baseOf = (s: string) => s.split(/[\/]/).pop()?.toLowerCase() ?? '';

export class AssetStore {
  private assets = new Map<string, Asset>();
  private listeners = new Set<(id: string) => void>();
  /** basename (lower case) -> asset ids, so URDF/package URIs resolve in O(1) instead of a scan per reference. */
  private byBase = new Map<string, Set<string>>();

  entries(): IterableIterator<[string, Asset]> {
    return this.assets.entries();
  }
  get size(): number { return this.assets.size; }
  get(id: string): Asset | undefined {
    return this.assets.get(id);
  }
  has(id: string): boolean {
    return this.assets.has(id);
  }
  /** Exact id, else an asset whose id basename or name equals the basename of `uri` (package://, relative paths). */
  resolve(uri: string): Asset | undefined {
    const a = this.assets.get(uri);
    if (a) return a;
    const ids = this.byBase.get(baseOf(uri));
    if (ids) for (const id of ids) { const b = this.assets.get(id); if (b) return b; }
    return undefined;
  }
  onChange(fn: (id: string) => void): () => void {
    this.listeners.add(fn);
    return () => this.listeners.delete(fn);
  }
  private put(a: Asset): Asset {
    const old = this.assets.get(a.id);
    if (old) this.unindex(old);
    this.assets.set(a.id, a);
    for (const k of new Set([baseOf(a.id), a.name ? a.name.toLowerCase() : ''])) if (k) { let s = this.byBase.get(k); if (!s) this.byBase.set(k, (s = new Set())); s.add(a.id); }
    for (const l of this.listeners) l(a.id);
    return a;
  }
  private unindex(a: Asset) {
    for (const k of [baseOf(a.id), a.name ? a.name.toLowerCase() : '']) this.byBase.get(k)?.delete(a.id);
  }
  /**
   * Register raw file bytes. `scale` multiplies STL/OBJ coordinates. For COLLADA the file's own
   * unit is honoured: `units === 'm'` keeps metres (URDF convention), otherwise the mesh is converted to mm.
   */
  registerRaw(id: string, type: Asset['type'], bytes: Uint8Array, name?: string, scale = 1, units?: 'mm' | 'm'): Asset {
    const a: Asset = { id, type, name, source: bytes };
    if (units) a.units = units;
    if (scale !== 1) a.scale = scale;
    if (type === 'stl') a.mesh = parseSTL(bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength) as ArrayBuffer, scale);
    else if (type === 'obj') a.mesh = parseOBJ(new TextDecoder().decode(bytes), scale);
    else if (type === 'dae') a.mesh = parseDAE(new TextDecoder().decode(bytes), units !== 'm');
    return this.put(a);
  }
  /** Register parsed geometry (no source bytes: the station file serialises the MeshData itself). */
  registerMesh(id: string, mesh: MeshData, name?: string, units?: 'mm' | 'm'): Asset {
    const a: Asset = { id, type: 'stl', name, mesh };
    if (units) a.units = units;
    return this.put(a);
  }
  remove(id: string): void {
    const a = this.assets.get(id);
    if (a) this.unindex(a);
    this.assets.delete(id);
  }
}
