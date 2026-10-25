/**
 * Reader for RoboDK .robot / .tool / .sld / .rdk containers.
 *
 * After a 4-byte big-endian uncompressed size the payload is zlib. A robot or tool payload is a table of
 * little-endian doubles followed by triangle meshes (rdk_robot.ts). A station payload is "Station File 01.01"
 * followed by a linked list of `lvl` chunks whose indices are the item references (rdk_station.ts).
 * The file format is documented in docs/robodk-file-format.md.
 */
import { Station, Frame } from '../../core/items/item';
import type { Robot } from '../../core/items/robot';
import { getPos, transl } from '../../core/math/pose';
import type { AssetStore } from '../../scene/assets';
import type { RdkScanReport } from './rdk_container';
import { RdkBytes, isStationFile } from './rdk_format';
import { decodeRobotTable, decodeObjectTable } from './rdk_robot';
import { decodeStation } from './rdk_station';

export interface RobodkLoad {
  station: Station;
  report: RdkScanReport;
}

async function inflate(buf: Uint8Array, format: 'deflate' | 'gzip'): Promise<Uint8Array | null> {
  try {
    if (typeof DecompressionStream === 'undefined') return null;
    const ds = new DecompressionStream(format);
    const stream = new Blob([buf as unknown as BlobPart]).stream().pipeThrough(ds);
    return new Uint8Array(await new Response(stream).arrayBuffer());
  } catch {
    return null;
  }
}

/** Strip the RoboDK size prefix and inflate. Raw payloads are returned unchanged. */
export async function unwrapRobodk(data: ArrayBuffer): Promise<{ bytes: Uint8Array; compressed: RdkScanReport['compressed'] }> {
  const bytes = new Uint8Array(data);
  const zlibAt = (i: number) => i + 1 < bytes.length && bytes[i] === 0x78 && (bytes[i + 1] === 0x9c || bytes[i + 1] === 0xda || bytes[i + 1] === 0x01);
  if (bytes.length > 6 && zlibAt(4)) {
    const inflated = await inflate(bytes.subarray(4), 'deflate');
    if (inflated && inflated.length > 64) return { bytes: inflated, compressed: 'zlib' };
  }
  if (zlibAt(0)) {
    const inflated = await inflate(bytes, 'deflate');
    if (inflated && inflated.length > 64) return { bytes: inflated, compressed: 'zlib' };
  }
  if (bytes[0] === 0x1f && bytes[1] === 0x8b) {
    const inflated = await inflate(bytes, 'gzip');
    if (inflated) return { bytes: inflated, compressed: 'gzip' };
  }
  return { bytes, compressed: 'none' };
}

/** Kind of a bare item table: robot tables are 4000 doubles, object / tool tables 1600. */
function tableKind(r: RdkBytes): 'robot' | 'object' | null {
  if (r.length < 64 || r.f64(0) !== 1) return null;
  const n = r.f64(8);
  if (n !== Math.round(n) || n * 8 > r.length) return null;
  if (n >= 3000 && n <= 20000) return 'robot';
  if (n >= 200 && n < 3000) return 'object';
  return null;
}

/** Build a robot from a parameter-table payload (the inflated .robot body, or an embedded copy). */
export function robotFromTable(bytes: Uint8Array, fallbackName: string, assets?: AssetStore): Robot | null {
  const r = new RdkBytes(bytes);
  return tableKind(r) === 'robot' ? decodeRobotTable(r, 0, r.length, fallbackName, assets)?.robot ?? null : null;
}

function makeReport(bytes: number, compressed: RdkScanReport['compressed'], station: Station, notes: string[], dropped: string[], counts: Record<string, number>): RdkScanReport {
  const names: string[] = [];
  let poses = 0;
  for (const it of station.walk()) {
    if (it === station) continue;
    if (names.length < 80) names.push(it.name);
    const p = getPos(it.pose());
    if (p[0] || p[1] || p[2]) poses++;
  }
  const meshes = Number(station.params.rdkTriangles ?? 0);
  return { bytes, compressed, strings: names, poses, meshes: meshes ? [{ offset: 0, triangles: meshes }] : [], notes, fidelity: dropped.length ? 'partial' : 'full', dropped, counts };
}

/** Full import of a .rdk / .robot / .tool / .sld buffer. Returns null when the bytes are not this format. */
export async function importRobodkBinary(data: ArrayBuffer, name = 'RoboDK import', assets?: AssetStore): Promise<RobodkLoad | null> {
  const { bytes, compressed } = await unwrapRobodk(data);
  const r = new RdkBytes(bytes);
  if (isStationFile(r)) {
    const dec = decodeStation(r, name.replace(/\.rdk$/i, ''), assets);
    if (!dec) return null;
    const { station, report } = dec;
    let tri = 0;
    for (const it of station.walk()) tri += Number(it.params.rdkMeshes ?? 0);
    station.params.rdkTriangles = tri;
    const c = report.counts;
    const summary = Object.entries(c).map(([k, v]) => `${v} ${k}`).join(', ');
    const notes = [`RoboDK station “${station.name}”: ${summary}; ${tri} mesh triangles.`, ...report.notes];
    if (report.dropped.length) notes.push(`Not imported or approximated (${report.dropped.length}): ${report.dropped.slice(0, 6).join(' | ')}${report.dropped.length > 6 ? ' …' : ''}`);
    station.params.rdkSummary = { robots: c.robot ?? 0, programs: c.program ?? 0, meshes: tri, items: Object.values(c).reduce((a, b) => a + b, 0) };
    return { station, report: makeReport(bytes.length, compressed, station, notes, report.dropped, c) };
  }
  const kind = tableKind(r);
  if (!kind) return null;
  const station = new Station(name);
  if (kind === 'robot') {
    const dec = decodeRobotTable(r, 0, r.length, name, assets);
    if (!dec) return null;
    const { robot } = dec;
    // RoboDK adds a "<robot> Base" reference frame and puts the robot under it when a .robot file is opened. The frame
    // is lifted by the base offset so the robot stands on the floor (Fanuc/Motoman/Staubli bases sit at the J2 height).
    const base = station.addChild(new Frame(`${robot.name} Base`));
    base.setPose(transl(-r.f64(180 * 8), -r.f64(181 * 8), -r.f64(182 * 8)));
    base.addChild(robot);
    station.params.rdkTriangles = Number(robot.params.rdkMeshes ?? 0);
    station.params.rdkSummary = { robots: 1, programs: 0, meshes: station.params.rdkTriangles, items: 2 };
    const notes = [`RoboDK robot “${robot.name}”: ${robot.dof} DOF, post ${robot.postProcessor}, ${robot.params.rdkMeshes ?? 0} mesh triangles in ${robot.params.rdkMeshLinks} links.`];
    return { station, report: makeReport(bytes.length, compressed, station, notes, dec.notes, { robot: 1, frame: 1 }) };
  }
  const asTool = /\.tool$/i.test(name);
  const dec = decodeObjectTable(r, 0, r.length, name.replace(/\.(tool|sld)$/i, ''), asTool, assets);
  if (!dec) return null;
  station.addChild(dec.item);
  station.params.rdkTriangles = Number(dec.item.params.rdkMeshes ?? 0);
  station.params.rdkSummary = { robots: 0, programs: 0, meshes: station.params.rdkTriangles, items: 1 };
  const notes = [`RoboDK ${asTool ? 'tool' : 'object'} “${dec.item.name}”: ${dec.item.params.rdkMeshes ?? 0} mesh triangles.`];
  return { station, report: makeReport(bytes.length, compressed, station, notes, dec.notes, { [asTool ? 'tool' : 'object']: 1 }) };
}
