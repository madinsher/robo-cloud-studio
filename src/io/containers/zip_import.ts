/**
 * Best-effort import of zip-based simulation containers.
 *
 * Visual Components (`.vcmx` components, `.vcm` layouts), KUKA.Sim Pro (built on Visual Components), and
 * plain `.zip` bundles exported from CAD tools are zip archives. Their component logic is proprietary
 * (VC's binary `.rsc` resources and Python behaviours cannot be executed here), but they usually carry
 * standard geometry (STL/OBJ/DAE/glTF) and XML/JSON metadata with names and transforms. This importer:
 *   1. lists the archive and loads every supported mesh into the asset store;
 *   2. scans XML/JSON for names, 4x4 matrices / positions attached to those meshes, joint/axis hints;
 *   3. builds a Folder with one SceneObject per mesh (posed when a transform could be matched),
 *      recording everything it could not interpret in the report.
 */
import { unzipSync, strFromU8 } from 'fflate';
import { AssetStore } from '../../scene/assets';
import { Folder, SceneObject, Item } from '../../core/items/item';
import { identity, fromArray, Mat4, transl, multiply } from '../../core/math/pose';
import { parseOBJ } from '../mesh/obj';

export interface ZipImportReport {
  entries: string[];
  meshes: string[];
  metadata: string[];
  /** Names discovered in metadata files. */
  names: string[];
  /** Joint / axis definitions found (VC "Joint"/"Axis" nodes). */
  joints: Array<{ name: string; type?: string; axis?: string; min?: number; max?: number }>;
  notes: string[];
  format: 'visual-components' | 'kuka-sim' | 'zip';
}

const MESH_EXT = /\.(stl|obj|dae|glb|gltf)$/i;
const META_EXT = /\.(xml|json|vcmx\.xml|txt|py|ini|csv|rsc)$/i;

function baseName(p: string): string {
  return p.split('/').pop()!.replace(/\.[^.]+$/, '');
}

/** Find a 4x4 (16 numbers) or 3x4 (12 numbers) matrix near a mesh name inside XML/JSON text. */
function findMatrixNear(text: string, name: string): Mat4 | null {
  const idx = text.toLowerCase().indexOf(name.toLowerCase());
  if (idx < 0) return null;
  const window = text.slice(Math.max(0, idx - 1500), idx + 1500);
  const nums = window.match(/-?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?/g)?.map(Number) ?? [];
  // look for 16 consecutive numbers whose last row (row-major) or last column (column-major) is 0 0 0 1
  for (let i = 0; i + 16 <= nums.length; i++) {
    const m = nums.slice(i, i + 16);
    if (m[12] === 0 && m[13] === 0 && m[14] === 0 && m[15] === 1) {
      // could be row-major with translation at [3],[7],[11] or column-major with translation at [12..14]
      // (translation zero) — prefer row-major (VC XML uses row-major "Matrix" strings)
      const cm = [m[0], m[4], m[8], m[12], m[1], m[5], m[9], m[13], m[2], m[6], m[10], m[14], m[3], m[7], m[11], m[15]];
      return fromArray(cm);
    }
    if (m[3] === 0 && m[7] === 0 && m[11] === 0 && m[15] === 1) return fromArray(m);
  }
  // Position="x y z" style
  const pm = window.match(/(?:Position|Location|Translation)\s*=\s*"?\s*(-?\d+(?:\.\d+)?)[ ,]+(-?\d+(?:\.\d+)?)[ ,]+(-?\d+(?:\.\d+)?)/i);
  if (pm) return transl(+pm[1], +pm[2], +pm[3]);
  return null;
}

function detectUnits(text: string): number {
  if (/unit\w*\s*=\s*"?(mm|millimet)/i.test(text)) return 1;
  if (/unit\w*\s*=\s*"?(m|meter|metre)s?"/i.test(text)) return 1000;
  return 1;
}

export async function importZipContainer(data: ArrayBuffer | Uint8Array, name: string, assets: AssetStore): Promise<{ folder: Folder; report: ZipImportReport }> {
  const bytes = data instanceof Uint8Array ? data : new Uint8Array(data);
  const files = unzipSync(bytes);
  const entries = Object.keys(files);
  const lower = name.toLowerCase();
  const isVC = /\.vcmx?$/.test(lower) || entries.some((e) => /\.vcmx?$|component\.rsc|vcmx\.xml/i.test(e));
  const report: ZipImportReport = { entries, meshes: [], metadata: [], names: [], joints: [], notes: [], format: isVC ? (/kuka/i.test(name) ? 'kuka-sim' : 'visual-components') : 'zip' };
  const folder = new Folder(name.replace(/\.[^.]+$/, ''));

  // metadata first (names, transforms, joints)
  const metaTexts: Array<{ path: string; text: string }> = [];
  for (const e of entries) {
    if (!META_EXT.test(e) || files[e].length > 8 * 1024 * 1024) continue;
    let text: string;
    try { text = strFromU8(files[e]); } catch { continue; }
    if (/\.rsc$/i.test(e)) { report.notes.push(`${e}: Visual Components binary resource (component behaviours/geometry) — not readable without the VC SDK`); continue; }
    metaTexts.push({ path: e, text });
    report.metadata.push(e);
    for (const m of text.matchAll(/\bName\s*=\s*"([^"]{1,80})"/g)) if (!report.names.includes(m[1])) report.names.push(m[1]);
    for (const m of text.matchAll(/<(?:Joint|Axis|Dof)\b([^>]*)>/gi)) {
      const attrs = m[1];
      const get = (k: string) => attrs.match(new RegExp(`${k}\\s*=\\s*"([^"]*)"`, 'i'))?.[1];
      report.joints.push({ name: get('Name') ?? `joint${report.joints.length + 1}`, type: get('Type') ?? get('JointType'), axis: get('Axis') ?? get('Direction'), min: get('Min') ? +get('Min')! : undefined, max: get('Max') ? +get('Max')! : undefined });
    }
  }
  const unitScale = metaTexts.length ? detectUnits(metaTexts[0].text) : 1;

  // meshes
  for (const e of entries) {
    if (!MESH_EXT.test(e)) continue;
    const ext = e.split('.').pop()!.toLowerCase();
    const id = `${folder.name}/${e}`;
    try {
      if (ext === 'stl') assets.registerRaw(id, 'stl', files[e], baseName(e), unitScale);
      else if (ext === 'dae') assets.registerRaw(id, 'dae', files[e], baseName(e));
      else if (ext === 'obj') assets.registerMesh(id, parseOBJ(strFromU8(files[e]), unitScale), baseName(e));
      else if (ext === 'glb' || ext === 'gltf') {
        const { parseGLTF } = await import('../mesh/gltf');
        const src = ext === 'glb' ? files[e].buffer.slice(files[e].byteOffset, files[e].byteOffset + files[e].byteLength) as ArrayBuffer : strFromU8(files[e]);
        assets.registerMesh(id, await parseGLTF(src), baseName(e));
      }
    } catch (err) {
      report.notes.push(`${e}: ${(err as Error).message}`);
      continue;
    }
    report.meshes.push(e);
    const a = assets.get(id);
    const o = new SceneObject(baseName(e));
    o.geometry = [{ mesh: id, origin: Array.from(identity()), color: '#a5b1c2' }];
    if (a?.mesh) o.bbox = { min: a.mesh.min, max: a.mesh.max };
    let pose: Mat4 | null = null;
    for (const mt of metaTexts) { pose = findMatrixNear(mt.text, baseName(e)); if (pose) break; }
    if (pose) o.setPose(pose);
    folder.addChild(o);
  }
  if (!report.meshes.length) report.notes.push('No standard meshes (STL/OBJ/DAE/glTF) found in the archive — Visual Components stores geometry in proprietary .rsc resources; export the component as STL/glTF from VC/KUKA.Sim (File › Export › Geometry) and drop it here.');
  if (report.joints.length) report.notes.push(`${report.joints.length} joint/axis definitions found in metadata: ${report.joints.slice(0, 6).map((j) => j.name).join(', ')} — use Tools › Mechanism builder to recreate the kinematics.`);
  return { folder, report };
}

export { Item, multiply };
