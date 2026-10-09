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
}

export class AssetStore {
  private assets = new Map<string, Asset>();
  private listeners = new Set<(id: string) => void>();

  entries(): IterableIterator<[string, Asset]> {
    return this.assets.entries();
  }
  get(id: string): Asset | undefined {
    return this.assets.get(id);
  }
  has(id: string): boolean {
    return this.assets.has(id);
  }
  onChange(fn: (id: string) => void): () => void {
    this.listeners.add(fn);
    return () => this.listeners.delete(fn);
  }
  /**
   * Register raw file bytes. `scale` multiplies STL/OBJ coordinates. For COLLADA the file's own
   * unit is honoured: `units === 'm'` keeps metres (URDF convention), otherwise the mesh is converted to mm.
   */
  registerRaw(id: string, type: Asset['type'], bytes: Uint8Array, name?: string, scale = 1, units?: 'mm' | 'm'): Asset {
    const a: Asset = { id, type, name, source: bytes };
    if (units) a.units = units;
    if (type === 'stl') a.mesh = parseSTL(bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength) as ArrayBuffer, scale);
    else if (type === 'obj') a.mesh = parseOBJ(new TextDecoder().decode(bytes), scale);
    else if (type === 'dae') a.mesh = parseDAE(new TextDecoder().decode(bytes), units !== 'm');
    this.assets.set(id, a);
    for (const l of this.listeners) l(id);
    return a;
  }
  registerMesh(id: string, mesh: MeshData, name?: string): Asset {
    const a: Asset = { id, type: 'stl', name, mesh };
    this.assets.set(id, a);
    for (const l of this.listeners) l(id);
    return a;
  }
  remove(id: string): void {
    this.assets.delete(id);
  }
}
