/**
 * Webots import entry points (interface I7). `importWebotsFiles` takes the files of a dropped folder / picker selection
 * (paths -> bytes; worlds, PROTOs, meshes, textures) and returns the station of the first world (or a preview of the
 * PROTOs when no world is present) plus a report. The Node/server side can use `importWebotsWorld` with a lazy source.
 */
import type { AssetStore } from '../../scene/assets';
import { Station, Frame } from '../../core/items/item';
import { transl } from '../../core/math/pose';
import { WebotsSource, normPath, ProtoRegistry } from './proto';
import { buildWebotsWorld, buildWebotsProto, WebotsBuildOptions, WebotsBuildResult } from './scene';
import { upgradeOrchard } from './semantic';
import { loadRetailLayout } from './retail_layout';

export { parseWorld, parseProtoFile, parseNodes, countNodeTypes, countTypesInText } from './vrml';
export type { WNode, WValue, WFile, ProtoFile } from './vrml';
export { expandTemplate, compileTemplate } from './template';
export { ProtoRegistry, resolveUrl, BASE_NODES } from './proto';
export type { WebotsSource } from './proto';
export { BUILTIN_PROTOS } from './builtins';
export { buildWebotsWorld, buildWebotsProto, WebotsSceneBuilder } from './scene';
export type { WebotsBuildOptions, WebotsBuildResult } from './scene';
export { robotToKinTree, WEBOTS_DEVICE_TO_STUDIO_CAMERA } from './robot';
export { rasterizeOccupancy } from './occupancy';
export type { OccupancyOptions } from './occupancy';
export { loadRetailLayout } from './retail_layout';
export { upgradeOrchard } from './semantic';
export { parseYaml } from './yaml';
export { ScriptedFleet, StationFleetWorld, WindowPlan, playScriptedFleet, moduleMountOffset, shortestAngle } from './scripted_fleet';
export type { FleetScript, FleetWorld, Pose4, WheelState, PlanRow, FleetPlayback } from './scripted_fleet';
export { defaultMissionBScript, MISSION_B } from './murka_missions';
export { parseShots, applyShot, viewpointLook } from './shots';
export type { WebotsShot, WebotsViewpoint } from './shots';

/** In-memory source over a path -> bytes map (keys normalised to '/' separators). */
export function mapSource(files: Map<string, Uint8Array>): WebotsSource {
  const m = new Map<string, Uint8Array>();
  for (const [k, v] of files) m.set(normPath(k.replace(/^\.\//, '')), v);
  return { read: (p) => m.get(normPath(p)), list: () => [...m.keys()] };
}

export interface WebotsImportOptions extends Omit<WebotsBuildOptions, 'assets'> {
  /** World to import when several .wbt files are present (path or file name). */
  world?: string;
  /** Add the semantic overlays (orchard fields/rows from tree objects; retail layout from products/locations.yaml). Default true. */
  semantic?: boolean;
}

/** I7: import a set of Webots files. */
export async function importWebotsFiles(files: Map<string, Uint8Array>, assets?: AssetStore, opts: WebotsImportOptions = {}): Promise<{ station: Station; report: string[]; worlds: string[] }> {
  const src = mapSource(files);
  const all = src.list();
  const worlds = all.filter((p) => /\.wbt$/i.test(p)).sort((a, b) => a.split('/').length - b.split('/').length || a.localeCompare(b));
  if (worlds.length) {
    const pick = opts.world ? worlds.find((w) => w === normPath(opts.world!) || w.endsWith(`/${opts.world}`) || w === opts.world) ?? worlds[0] : worlds[0];
    const r = buildWebotsWorld(pick, src, { ...opts, assets });
    const report = [...r.report];
    if (worlds.length > 1) report.unshift(`${worlds.length} worlds found, imported ${pick} (others: ${worlds.filter((w) => w !== pick).join(', ')})`);
    if (opts.semantic !== false) {
      report.push(...upgradeOrchard(r.station, { assets }).report);
      const products = all.find((p) => /(^|\/)products\.ya?ml$/i.test(p)), locations = all.find((p) => /(^|\/)locations\.ya?ml$/i.test(p));
      if (products && locations) {
        try {
          const dec = (p: string) => new TextDecoder().decode(src.read(p)!);
          report.push(...loadRetailLayout(r.station, dec(products), dec(locations), { builder: r.builder, worldPath: pick }).report);
        } catch (e) { report.push(`retail layout (${products}, ${locations}): ${(e as Error).message}`); }
      }
    }
    return { station: r.station, report, worlds };
  }
  const protos = all.filter((p) => /\.proto$/i.test(p));
  if (!protos.length) throw new Error('no .wbt world or .proto file in the selection');
  // no world: preview every PROTO with its default fields, side by side along X
  const station = new Station(protos.length === 1 ? protos[0].split('/').pop()!.replace(/\.proto$/i, '') : 'Webots PROTOs');
  const report: string[] = [];
  let x = 0;
  for (const p of protos) {
    const r: WebotsBuildResult = buildWebotsProto(p, src, { ...opts, assets });
    report.push(...r.report);
    const fr = new Frame(p.split('/').pop()!.replace(/\.proto$/i, ''));
    station.addChild(fr);
    fr.setPose(transl(x, 0, 0));
    for (const c of [...r.station.children]) fr.addChild(c);
    x += 3000;
  }
  return { station, report, worlds: [] };
}

/** Lazy import from any source (server / tests reading a directory). */
export function importWebotsWorld(worldPath: string, src: WebotsSource, opts: WebotsBuildOptions = {}): WebotsBuildResult {
  return buildWebotsWorld(normPath(worldPath), src, opts);
}

/** Resolve the PROTO declarations a world needs (diagnostics: which files would be read). */
export function protoRegistry(src: WebotsSource): ProtoRegistry {
  return new ProtoRegistry(src);
}
