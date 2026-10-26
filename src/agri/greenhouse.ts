/**
 * Greenhouse layout (Venlo-type tomato / cucumber house): hanging gutters along the crop rows, heating-pipe rails
 * (two Ø 51 mm pipes 550 mm apart at 100 mm) in every path between the gutters, a concrete corridor across the row
 * heads, and a rail network for pipe-rail trolleys (src/mobile/rails.ts, navigation method `rail_1d`): trolleys run on
 * the pipes along a path only and change paths on the corridor.
 */
import { Station, SceneObject, GeometryRef } from '../core/items/item';
import { mul, transl, rotz, rotx, DEG } from '../core/math/pose';
import { FieldItem, CropRow } from './items';
import type { MobileRobot } from '../mobile/items';
import { RailNetwork } from '../mobile/rails';
import { defaultNavStack, setNavStack } from '../mobile/navstack';

export interface GreenhouseOptions {
  /** Gutter height above the floor and cross-section (mm). */
  gutterHeight?: number; gutterWidth?: number;
  /** Heating pipes: centre spacing, diameter, height (mm). */
  pipeSpacing?: number; pipeDiameter?: number; pipeHeight?: number;
  /** Corridor width (mm) and its distance from the row heads (axis, mm). */
  corridorWidth?: number; corridorOffset?: number;
}

export interface GreenhouseLayout { network: RailNetwork; gutters: SceneObject[]; pipes: SceneObject[]; corridor: SceneObject; paths: Array<{ y: number; x0: number; x1: number }> }

const worldOf = (row: CropRow, p: number[]) => { const a = row.parent ? row.parent.poseAbs() : null; return a ? [a[0] * p[0] + a[4] * p[1] + a[12], a[1] * p[0] + a[5] * p[1] + a[13]] : [p[0], p[1]]; };

/** Build the greenhouse structure for an indoor field whose rows run along X; returns the trolley rail network. */
export function buildGreenhouse(st: Station, field: FieldItem, o: GreenhouseOptions = {}): GreenhouseLayout {
  const rows = field.rows();
  if (!rows.length) throw new Error('buildGreenhouse: the field has no rows');
  const gh = o.gutterHeight ?? 900, gw = o.gutterWidth ?? 250, ps = o.pipeSpacing ?? 550, pd = o.pipeDiameter ?? 51, ph = o.pipeHeight ?? 100;
  const cw = o.corridorWidth ?? 2400;
  const lines = rows.map((r) => { const a = worldOf(r, r.start), b = worldOf(r, r.end); return { a, b, y: (a[1] + b[1]) / 2, x0: Math.min(a[0], b[0]), x1: Math.max(a[0], b[0]) }; }).sort((p, q) => p.y - q.y);
  if (lines.some((l) => Math.abs(l.a[1] - l.b[1]) > 1)) throw new Error('buildGreenhouse: rows must run along X');
  const spacing = field.crop.rowSpacing;
  const x0 = Math.min(...lines.map((l) => l.x0)), x1 = Math.max(...lines.map((l) => l.x1));
  const xc = x0 - (o.corridorOffset ?? Math.max(cw / 2 + 400, 1600));
  const gutters: SceneObject[] = [], pipes: SceneObject[] = [];
  for (const [i, l] of lines.entries()) {
    const g = st.addChild(new SceneObject(`Gutter ${i + 1}`));
    const L = l.x1 - l.x0 + 600;
    const geo: GeometryRef[] = [{ primitive: { kind: 'box', size: [L, gw, 150] }, origin: Array.from(transl((l.x0 + l.x1) / 2, l.y, gh)), color: '#d9d9d9' }];
    // hanging wires every 2.5 m
    for (let x = l.x0; x <= l.x1; x += 2500) geo.push({ primitive: { kind: 'cylinder', radius: 4, length: 3500 - gh }, origin: Array.from(transl(x, l.y, gh + (3500 - gh) / 2)), color: '#868e96' });
    g.geometry = geo; g.colliders = [geo[0]];
    g.setParam('greenhouse', 'gutter');
    gutters.push(g);
  }
  // paths: between neighbouring gutters and the two outer ones (the travel lines of the rows, both sides)
  const ys = [lines[0].y - spacing / 2, ...lines.slice(0, -1).map((l, i) => (l.y + lines[i + 1].y) / 2), lines[lines.length - 1].y + spacing / 2];
  const paths = ys.map((y) => ({ y, x0: xc, x1: x1 + 300 }));
  for (const [i, p] of paths.entries()) {
    const obj = st.addChild(new SceneObject(`Heating pipes ${i + 1}`));
    const L = p.x1 - x0 + 300;
    const geo: GeometryRef[] = [-1, 1].map((s) => ({ primitive: { kind: 'cylinder' as const, radius: pd / 2, length: L }, origin: Array.from(mul(transl(x0 + L / 2 - 300, p.y + (s * ps) / 2, ph), mul(rotz(90 * DEG), rotx(90 * DEG)))), color: '#adb5bd' }));
    obj.geometry = geo;
    obj.setParam('greenhouse', 'pipe_rail');
    pipes.push(obj);
  }
  const corridor = st.addChild(new SceneObject('Concrete corridor'));
  const yMin = ys[0] - spacing, yMax = ys[ys.length - 1] + spacing;
  corridor.geometry = [{ primitive: { kind: 'box', size: [cw, yMax - yMin, 20] }, origin: Array.from(transl(xc, (yMin + yMax) / 2, -10)), color: '#ced4da' }];
  corridor.params.ground = true;
  const network: RailNetwork = { corridor: [[xc, yMin], [xc, yMax]], corridorWidth: cw, rails: paths.map((p) => [[xc, p.y], [p.x1, p.y]]) };
  field.setParam('railNetwork', network as never);
  return { network, gutters, pipes, corridor, paths };
}

/** Put a trolley on the rail network: navigation method `rail_1d` (routes on the network). */
export function useRails(robot: MobileRobot, network: RailNetwork): void {
  const cfg = defaultNavStack('rail', 'greenhouse');
  cfg.navigation = 'rail_1d';
  setNavStack(robot, cfg);
  robot.setParam('railNetwork', network as never);
}
