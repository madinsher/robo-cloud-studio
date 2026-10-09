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

/** Clip a line (infinite, through p with direction d) to a convex polygon; returns [tmin, tmax] or null. */
function clipLine(p: number[], d: number[], poly: number[][]): [number, number] | null {
  let tmin = -Infinity, tmax = Infinity;
  const n = poly.length;
  // polygon orientation
  let area = 0;
  for (let i = 0; i < n; i++) { const a = poly[i], b = poly[(i + 1) % n]; area += a[0] * b[1] - b[0] * a[1]; }
  const ccw = area > 0;
  for (let i = 0; i < n; i++) {
    const a = poly[i], b = poly[(i + 1) % n];
    const ex = b[0] - a[0], ey = b[1] - a[1];
    // inward normal
    let nx = ccw ? -ey : ey, ny = ccw ? ex : -ex;
    const num = nx * (a[0] - p[0]) + ny * (a[1] - p[1]);
    const den = nx * d[0] + ny * d[1];
    if (Math.abs(den) < 1e-12) { if (num > 0) return null; continue; }
    const t = num / den;
    if (den > 0) tmin = Math.max(tmin, t); else tmax = Math.min(tmax, t);
    if (tmin > tmax) return null;
  }
  return tmin < tmax ? [tmin, tmax] : null;
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
  for (let o = firstOff; o <= omax - c.headland * 0.5; o += c.rowSpacing) {
    const p = [cx + (o - c0) * nrm[0], cy + (o - c0) * nrm[1]];
    const clip = clipLine(p, d, poly);
    if (!clip) continue;
    let [t0, t1] = clip;
    t0 += c.headland; t1 -= c.headland;
    if (t1 - t0 < c.plantSpacing) continue;
    const row = new CropRow(`Row ${idx + 1}`);
    row.index = idx;
    row.start = [p[0] + d[0] * t0, p[1] + d[1] * t0];
    row.end = [p[0] + d[0] * t1, p[1] + d[1] * t1];
    row.segmentId = `${field.id}:row${idx}`;
    const len = t1 - t0;
    const nPlants = Math.floor(len / c.plantSpacing);
    const pad = (len - (nPlants - 1) * c.plantSpacing) / 2;
    for (let k = 0; k < nPlants; k++) {
      const plant: PlantRecord = { s: pad + k * c.plantSpacing, height: c.canopyHeight * (0.85 + rng() * 0.3), width: c.canopyWidth * (0.85 + rng() * 0.3), fruit: [], health: 0.9 + rng() * 0.1 };
      const nFruit = Math.round(c.fruitPerPlant * (0.6 + rng() * 0.8));
      for (let f = 0; f < nFruit && f < 400; f++) {
        // fruit distributed in the canopy envelope; 2D trellis -> thin in row-normal direction
        const thin = c.training === 'trellis_2d' || c.training === 'v_trellis' || c.training === 'greenhouse_gutter' ? 0.35 : 1;
        const u = rng() * 2 - 1, v = (rng() * 2 - 1) * thin, w = rng();
        const z = c.canopyBase + w * plant.height;
        const rad = plant.width / 2 * Math.sqrt(1 - Math.abs(2 * w - 1) * 0.5);
        plant.fruit.push({ p: [u * rad, v * rad, z], ripe: rng() < c.ripeFraction ? 0.6 + rng() * 0.4 : rng() * 0.5, picked: false, d: c.fruitDiameter * (0.85 + rng() * 0.3) });
      }
      row.plants.push(plant);
    }
    field.addChild(row);
    rows.push(row);
    idx++;
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

/** Rasterise a field into an occupancy map: rows are obstacles, headlands and inter-rows are free. */
export function buildFieldMap(field: FieldItem, resolution = 250, margin = 10000): MapItem {
  const poly = field.polygon;
  const xs = poly.map((p) => p[0]), ys = poly.map((p) => p[1]);
  const minX = Math.min(...xs) - margin, maxX = Math.max(...xs) + margin;
  const minY = Math.min(...ys) - margin, maxY = Math.max(...ys) + margin;
  const map = new MapItem(`${field.name} map`);
  map.resize(Math.ceil((maxX - minX) / resolution), Math.ceil((maxY - minY) / resolution), resolution, minX, minY);
  map.inflation = Math.max(300, field.crop.rowSpacing * 0.2);
  const abs = field.poseAbs();
  map.setPose(abs);
  const halfW = Math.max(150, field.crop.canopyWidth / 2);
  const rc = Math.ceil(halfW / resolution);
  for (const row of field.rows()) {
    const n = Math.ceil(row.length() / (resolution * 0.5));
    for (let i = 0; i <= n; i++) {
      const [x, y] = row.pointAt((row.length() * i) / n);
      const [cx0, cy0] = map.worldToCell(x, y);
      for (let cy = cy0 - rc; cy <= cy0 + rc; cy++) for (let cx = cx0 - rc; cx <= cx0 + rc; cx++) {
        const [wx, wy] = map.cellToWorld(cx, cy);
        if (Math.hypot(wx - x, wy - y) <= halfW) map.set(cx, cy, 100);
      }
    }
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
