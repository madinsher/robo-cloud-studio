/**
 * STEP / IGES / BREP import through OpenCascade compiled to WebAssembly (occt-import-js).
 * Loaded on demand (≈11 MB). Produces MeshData per solid (triangulated with the given linear deflection),
 * plus per-face colours when the file carries them.
 */
import { MeshData } from './stl';

export interface StepImportResult {
  meshes: Array<{ name: string; mesh: MeshData; color?: string }>;
  /** Assembly tree names (for information). */
  names: string[];
}

let occtPromise: Promise<any> | null = null;

async function loadOcct(): Promise<any> {
  if (!occtPromise) {
    occtPromise = (async () => {
      const mod: any = await import('occt-import-js');
      const factory = mod.default ?? mod;
      const isNode = typeof window === 'undefined';
      let wasmUrl = '';
      if (!isNode) {
        const u: any = await import('occt-import-js/dist/occt-import-js.wasm?url');
        wasmUrl = u.default ?? u;
      }
      return factory({ locateFile: (name: string) => (isNode ? new URL('../../../node_modules/occt-import-js/dist/' + name, import.meta.url).pathname : name.endsWith('.wasm') ? wasmUrl : name) });
    })();
  }
  return occtPromise;
}

/** Import a CAD file. `kind` is inferred from the extension when omitted. Units: file units are converted to mm. */
export async function importCad(data: ArrayBuffer, filename: string, opts: { linearDeflection?: number; angularDeflection?: number } = {}): Promise<StepImportResult> {
  const occt = await loadOcct();
  const ext = filename.split('.').pop()?.toLowerCase() ?? 'step';
  const params = { linearUnit: 'millimeter', linearDeflectionType: 'absolute_value', linearDeflection: opts.linearDeflection ?? 0.5, angularDeflection: opts.angularDeflection ?? 0.5 };
  const bytes = new Uint8Array(data);
  const result = ext === 'igs' || ext === 'iges' ? occt.ReadIgesFile(bytes, params) : ext === 'brep' || ext === 'brp' ? occt.ReadBrepFile(bytes, params) : occt.ReadStepFile(bytes, params);
  if (!result?.success) throw new Error(`OpenCascade could not read ${filename}`);
  const meshes: StepImportResult['meshes'] = [];
  const names: string[] = [];
  const walk = (node: any) => { if (node?.name) names.push(node.name); for (const c of node?.children ?? []) walk(c); };
  walk(result.root);
  for (const m of result.meshes ?? []) {
    const pos: number[] = m.attributes?.position?.array ?? [];
    const nor: number[] | undefined = m.attributes?.normal?.array;
    const idx: number[] = m.index?.array ?? [];
    const n = idx.length;
    const positions = new Float32Array(n * 3);
    const normals = new Float32Array(n * 3);
    const min: [number, number, number] = [Infinity, Infinity, Infinity], max: [number, number, number] = [-Infinity, -Infinity, -Infinity];
    for (let i = 0; i < n; i++) {
      const v = idx[i];
      for (let a = 0; a < 3; a++) {
        const p = pos[v * 3 + a];
        positions[i * 3 + a] = p;
        if (nor) normals[i * 3 + a] = nor[v * 3 + a];
        if (p < min[a]) min[a] = p; if (p > max[a]) max[a] = p;
      }
    }
    const c = m.color ? `#${m.color.slice(0, 3).map((x: number) => Math.round(x * 255).toString(16).padStart(2, '0')).join('')}` : undefined;
    meshes.push({ name: m.name || `solid ${meshes.length + 1}`, mesh: { positions, normals, min, max, triangles: n / 3 }, color: c });
  }
  return { meshes, names };
}

export function isCadFile(name: string): boolean {
  return /\.(step|stp|iges|igs|brep|brp)$/i.test(name);
}
