/** Orchard / field generators: rows and plants from a field polygon; greenhouse layouts; map rasterisation. */
import { FieldItem, CropRow, PlantRecord, CropParams } from './items';
import { MapItem, ZoneItem } from '../mobile/items';
import { transl } from '../core/math/pose';

function mulberry32(a: number) {
  return () => {
    a |= 0; a = (a + 0x6d2b79f5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

export function rectPolygon(w: number, h: number, x0 = 0, y0 = 0): number[][] {
  return [[x0, y0], [x0 + w, y0], [x0 + w, y0 + h], [x0, y0 + h]];
}

/**
 * Intervals [t0, t1] where the infinite line p + t·d lies inside a simple polygon (convex or concave): the line's
 * crossings with the edges, sorted and paired (even-odd rule). Vertices on the line are handled by the half-open
 * edge rule.
 */
export function clipLineIntervals(p: number[], d: number[], poly: number[][]): Array<[number, number]> {
  const n = poly.length;
  const nrm = [-d[1], d[0]];
  const side = (q: number[]) => (q[0] - p[0]) * nrm[0] + (q[1] - p[1]) * nrm[1];
  const ts: number[] = [];
  for (let i = 0; i < n; i++) {
    const a = poly[i], b = poly[(i + 1) % n];
    const sa = side(a), sb = side(b);
    if ((sa > 0) === (sb > 0)) continue; // half-open: a vertex exactly on the line counts once
    const u = sa / (sa - sb);
    const x = a[0] + (b[0] - a[0]) * u, y = a[1] + (b[1] - a[1]) * u;
    ts.push((x - p[0]) * d[0] + (y - p[1]) * d[1]);
  }
  ts.sort((x, y) => x - y);
  const out: Array<[number, number]> = [];
  for (let i = 0; i + 1 < ts.length; i += 2) if (ts[i + 1] > ts[i]) out.push([ts[i], ts[i + 1]]);
  return out;
}

/** Clip a line to a polygon: the longest inside interval (kept for callers expecting one interval). */
export function clipLine(p: number[], d: number[], poly: number[][]): [number, number] | null {
  const iv = clipLineIntervals(p, d, poly);
  if (!iv.length) return null;
  return iv.reduce((a, b) => (b[1] - b[0] > a[1] - a[0] ? b : a));
}

/** Generate crop rows and plants for a field. Replaces existing rows. */
export function generateOrchard(field: FieldItem, seed = 1): CropRow[] {
  for (const r of field.rows()) r.delete();
  const c = field.crop;
  const poly = field.polygon;
  if (poly.length < 3) return [];
  const th = (c.rowHeading * Math.PI) / 180;
  const d = [Math.cos(th), Math.sin(th)];
  const nrm = [-d[1], d[0]];
  // project polygon on the normal to find the range of row offsets
  const offs = poly.map((p) => p[0] * nrm[0] + p[1] * nrm[1]);
  const omin = Math.min(...offs), omax = Math.max(...offs);
  const cx = poly.reduce((s, p) => s + p[0], 0) / poly.length, cy = poly.reduce((s, p) => s + p[1], 0) / poly.length;
  const c0 = cx * nrm[0] + cy * nrm[1];
  const rng = mulberry32(seed);
  const rows: CropRow[] = [];
  let idx = 0;
  const firstOff = omin + c.headland;
  let line = 0;
  for (let o = firstOff; o <= omax - c.headland * 0.5; o += c.rowSpacing, line++) {
    const p = [cx + (o - c0) * nrm[0], cy + (o - c0) * nrm[1]];
    // concave fields: one row per inside interval of the line (headland kept at every boundary crossing)
    const parts = clipLineIntervals(p, d, poly).map(([a, b]) => [a + c.headland, b - c.headland]).filter(([a, b]) => b - a >= c.plantSpacing);
    parts.forEach(([t0, t1], part) => {
      const row = new CropRow(parts.length > 1 ? `Row ${line + 1}${String.fromCharCode(97 + part)}` : `Row ${line + 1}`);
      row.index = line;
      row.start = [p[0] + d[0] * t0, p[1] + d[1] * t0];
      row.end = [p[0] + d[0] * t1, p[1] + d[1] * t1];
      row.segmentId = `${field.id}:row${idx}`;
      const len = t1 - t0;
      const nPlants = Math.floor(len / c.plantSpacing);
      const pad = (len - (nPlants - 1) * c.plantSpacing) / 2;
      for (let k = 0; k < nPlants; k++) {
        const plant: PlantRecord = { s: pad + k * c.plantSpacing, height: c.canopyHeight * (0.85 + rng() * 0.3), width: c.canopyWidth * (0.85 + rng() * 0.3), fruit: [], health: 0.9 + rng() * 0.1 };
        const nFruit = Math.round(c.fruitPerPlant * (0.6 + rng() * 0.8));
        // 2-D trellis / gutter canopies are thin across the row
        const thin = c.training === 'trellis_2d' || c.training === 'v_trellis' || c.training === 'greenhouse_gutter' ? 0.35 : 1;
        for (let f = 0; f < nFruit; f++) {
          // inside the canopy envelope: height fraction w, radius of the canopy at that height (spindle = cone,
          // others = ellipsoid), uniform in the disk of that radius
          const w = rng();
          const env = c.training === 'spindle' ? 1 - 0.7 * w : Math.sqrt(Math.max(0, 1 - (2 * w - 1) * (2 * w - 1)));
          const rad = (plant.width / 2) * Math.max(0.15, env) * Math.sqrt(rng());
          const ang = rng() * 2 * Math.PI;
          const z = c.canopyBase + w * plant.height;
          plant.fruit.push({ p: [Math.cos(ang) * rad, Math.sin(ang) * rad * thin, z], ripe: rng() < c.ripeFraction ? 0.6 + rng() * 0.4 : rng() * 0.5, picked: false, d: c.fruitDiameter * (0.85 + rng() * 0.3) });
        }
        row.plants.push(plant);
      }
      field.addChild(row);
      rows.push(row);
      idx++;
    });
  }
  return rows;
}

/** World position (mm, field frame) of a plant base. */
export function plantPosition(row: CropRow, plant: PlantRecord): [number, number] {
  return row.pointAt(plant.s);
}

/** World-frame fruit positions (mm) for a row, with row-aligned local axes. */
export function fruitWorldPositions(row: CropRow, onlyRipe = true): Array<{ p: [number, number, number]; plant: PlantRecord; fruit: PlantRecord['fruit'][number]; side: 1 | -1 }> {
  const out: Array<{ p: [number, number, number]; plant: PlantRecord; fruit: PlantRecord['fruit'][number]; side: 1 | -1 }> = [];
  const [dx, dy] = row.direction();
  const abs = row.parent ? row.parent.poseAbs() : transl(0, 0, 0);
  for (const plant of row.plants) {
    const [bx, by] = plantPosition(row, plant);
    for (const f of plant.fruit) {
      if (onlyRipe && (f.ripe < 0.5 || f.picked)) continue;
      const lx = bx + dx * f.p[0] - dy * f.p[1];
      const ly = by + dy * f.p[0] + dx * f.p[1];
      const wx = abs[0] * lx + abs[4] * ly + abs[12];
      const wy = abs[1] * lx + abs[5] * ly + abs[13];
      out.push({ p: [wx, wy, f.p[2] + abs[14]], plant, fruit: f, side: f.p[1] >= 0 ? 1 : -1 });
    }
  }
  return out;
}

/**
 * Rasterise a field into an occupancy map: every plant's trunk and — when its canopy starts below `clearHeight` — its
 * own canopy disk are obstacles (cell value 90 = crop, so planners holding the exact crop circles can skip them);
 * headlands and inter-rows are free. Pergola canopies above the robots (kiwi, canopyBase 1.8 m) leave the alleys open.
 */
export function buildFieldMap(field: FieldItem, resolution = 250, margin = 10000, opts: { clearHeight?: number } = {}): MapItem {
  const poly = field.polygon;
  const xs = poly.map((p) => p[0]), ys = poly.map((p) => p[1]);
  const minX = Math.min(...xs) - margin, maxX = Math.max(...xs) + margin;
  const minY = Math.min(...ys) - margin, maxY = Math.max(...ys) + margin;
  const map = new MapItem(`${field.name} map`);
  map.resize(Math.ceil((maxX - minX) / resolution), Math.ceil((maxY - minY) / resolution), resolution, minX, minY);
  map.inflation = 400;
  const abs = field.poseAbs();
  map.setPose(abs);
  const clear = opts.clearHeight ?? 1500;
  const c = field.crop;
  const trunkR = c.trunkDiameter ? c.trunkDiameter / 2 : c.training === 'greenhouse_gutter' ? 15 : 75;
  const disk = (x: number, y: number, r: number) => {
    const [cx0, cy0] = map.worldToCell(x, y);
    const rc = Math.ceil(r / resolution) + 1;
    for (let cy = cy0 - rc; cy <= cy0 + rc; cy++) for (let cx = cx0 - rc; cx <= cx0 + rc; cx++) {
      const [wx, wy] = map.cellToWorld(cx, cy);
      // cell intersects the disk (conservative: centre within r + half a cell)
      if (Math.hypot(wx - x, wy - y) <= r + resolution * 0.5) map.set(cx, cy, 90);
    }
  };
  for (const row of field.rows()) for (const p of row.plants) {
    const [x, y] = row.pointAt(p.s);
    disk(x, y, trunkR);
    if (c.canopyBase < clear) disk(x, y, p.width / 2);
  }
  map.notify('map');
  return map;
}

/** Greenhouse layout: benches/gutters in a rectangular house with a central corridor. */
export function greenhousePolygon(width: number, length: number): number[][] {
  return rectPolygon(length, width);
}

export function makeHeadlandZones(field: FieldItem): ZoneItem[] {
  const zones: ZoneItem[] = [];
  const rows = field.rows();
  if (!rows.length) return zones;
  const c = field.crop;
  const first = rows[0], last = rows[rows.length - 1];
  const [dx, dy] = first.direction();
  const nrm = [-dy, dx];
  const mk = (name: string, a: number[], b: number[], sign: number) => {
    const z = new ZoneItem(name);
    z.kind = 'headland';
    const h = c.headland;
    z.polygon = [
      [a[0] - nrm[0] * c.rowSpacing, a[1] - nrm[1] * c.rowSpacing],
      [b[0] + nrm[0] * c.rowSpacing, b[1] + nrm[1] * c.rowSpacing],
      [b[0] + nrm[0] * c.rowSpacing + dx * h * sign, b[1] + nrm[1] * c.rowSpacing + dy * h * sign],
      [a[0] - nrm[0] * c.rowSpacing + dx * h * sign, a[1] - nrm[1] * c.rowSpacing + dy * h * sign],
    ];
    field.addChild(z);
    zones.push(z);
  };
  mk('Headland A', first.start, last.start, -1);
  mk('Headland B', first.end, last.end, 1);
  return zones;
}

/**
 * GNSS-denied zone covering the tree rows (canopy blocks the sky view). Used by the navigation-stack simulation:
 * RTK/GNSS localization falls back to onboard odometry inside it, so orchard fleets are debugged with realistic
 * outages. Headlands stay GNSS-visible.
 */
export function makeCanopyZones(field: FieldItem): ZoneItem[] {
  const rows = field.rows();
  if (!rows.length) return [];
  const c = field.crop;
  const first = rows[0], last = rows[rows.length - 1];
  const [dx, dy] = first.direction();
  const nrm = [-dy, dx];
  const half = c.rowSpacing * 0.5;
  const z = new ZoneItem('Canopy (GNSS denied)');
  z.kind = 'gnss_denied';
  z.polygon = [
    [first.start[0] - nrm[0] * half, first.start[1] - nrm[1] * half],
    [first.end[0] - nrm[0] * half, first.end[1] - nrm[1] * half],
    [last.end[0] + nrm[0] * half, last.end[1] + nrm[1] * half],
    [last.start[0] + nrm[0] * half, last.start[1] + nrm[1] * half],
  ];
  z.setParam('note', `canopy over ${rows.length} rows; sky view only on headlands (${dx.toFixed(2)}, ${dy.toFixed(2)} row direction)`);
  field.addChild(z);
  return [z];
}
