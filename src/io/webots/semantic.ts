/**
 * Semantic upgrade of imported orchards: tree objects (name / DEF containing "tree", e.g. MURKA apple_tree_s1_01)
 * are grouped into one FieldItem per sector (`_s<N>_` in the name, else one field) and CropRows through trees that
 * share a row line, with PlantRecords at the tree positions (height / width from the tree geometry, fruit from small
 * red spheres in the tree such as the apples of murca_apple_garden.wbt). The trees stay as SceneObjects (they are what
 * is rendered); the fields are a hidden semantic overlay used by the agri planners and vision ground truth.
 */
import { Station, SceneObject, Item } from '../../core/items/item';
import { FieldItem, CropRow, PlantRecord, cropParams, CropType } from '../../agri/items';
import { geometryBounds } from '../kintree';
import type { AssetStore } from '../../scene/assets';
import { fromArray, identity, transformPoint } from '../../core/math/pose';

export interface OrchardUpgradeOptions {
  /** Tree detection (default: SceneObject whose name or DEF contains "tree"). */
  isTree?: (it: SceneObject) => boolean;
  assets?: AssetStore;
  crop?: CropType;
  /** Max lateral deviation (mm) of a tree from its row line. */
  rowTolerance?: number;
}

const median = (a: number[]) => { if (!a.length) return 0; const s = [...a].sort((x, y) => x - y); return s[Math.floor(s.length / 2)]; };

/** Fruit: spheres with a reddish / orange colour and radius < 80 mm inside the tree geometry. */
function fruitOf(t: SceneObject): PlantRecord['fruit'] {
  const out: PlantRecord['fruit'] = [];
  for (const g of t.geometry) {
    const p = g.primitive;
    if (p?.kind !== 'sphere' || p.radius > 80 || !g.color) continue;
    const r = parseInt(g.color.slice(1, 3), 16), gr = parseInt(g.color.slice(3, 5), 16), b = parseInt(g.color.slice(5, 7), 16);
    if (r < 150 || gr > 160 || b > 120) continue;
    const o = transformPoint(g.origin ? fromArray(g.origin) : identity(), [0, 0, 0]);
    out.push({ p: [o[0], o[1], o[2]], ripe: 1, picked: false, d: 2 * p.radius });
  }
  return out;
}

/** Group the trees of a station into fields and rows. Returns the created fields (added to the station, hidden). */
export function upgradeOrchard(station: Station, opts: OrchardUpgradeOptions = {}): { fields: FieldItem[]; report: string[] } {
  const isTree = opts.isTree ?? ((it: SceneObject) => /tree/i.test(it.name) || /tree/i.test(String(it.params.def ?? '')));
  const tol = opts.rowTolerance ?? 400;
  const trees: SceneObject[] = [];
  for (const it of station.walk()) if (it instanceof SceneObject && it.parent === station && isTree(it)) trees.push(it);
  const report: string[] = [];
  if (trees.length < 2) return { fields: [], report };
  const sectors = new Map<string, SceneObject[]>();
  for (const t of trees) {
    const m = /_s(\d+)_/i.exec(t.name) ?? /_S(\d+)_/.exec(String(t.params.def ?? ''));
    const key = m ? m[1] : '';
    if (!sectors.has(key)) sectors.set(key, []);
    sectors.get(key)!.push(t);
  }
  const fields: FieldItem[] = [];
  for (const [key, list] of [...sectors.entries()].sort()) {
    const pts = list.map((t) => ({ t, x: t.pose()[12], y: t.pose()[13] }));
    // rows: group by Y (rows along X) or by X, whichever gives fewer, longer rows
    const groupBy = (axis: 'x' | 'y') => {
      const sorted = [...pts].sort((a, b) => a[axis] - b[axis]);
      const rows: (typeof pts)[] = [];
      for (const p of sorted) { const last = rows[rows.length - 1]; if (last && Math.abs(p[axis] - last[0][axis]) <= tol) last.push(p); else rows.push([p]); }
      return rows;
    };
    const byY = groupBy('y'), byX = groupBy('x');
    const alongX = byY.length <= byX.length;
    const rows = (alongX ? byY : byX).map((r) => r.sort((a, b) => (alongX ? a.x - b.x : a.y - b.y)));
    const rowCoord = rows.map((r) => median(r.map((p) => (alongX ? p.y : p.x))));
    const spacing = median(rows.flatMap((r) => r.slice(1).map((p, i) => Math.hypot(p.x - r[i].x, p.y - r[i].y))));
    const rowSpacing = rows.length > 1 ? median(rowCoord.slice(1).map((c, i) => Math.abs(c - rowCoord[i]))) : 4000;
    const dims = list.map((t) => geometryBounds(t.geometry, opts.assets)).filter((b): b is NonNullable<typeof b> => !!b);
    const height = dims.length ? median(dims.map((b) => b.max[2])) : 3000;
    const width = dims.length ? median(dims.map((b) => Math.max(b.max[0] - b.min[0], b.max[1] - b.min[1]))) : 2000;
    const field = new FieldItem(key ? `sector_${key}` : 'orchard');
    field.crop = cropParams(opts.crop ?? 'apple', { rowSpacing, plantSpacing: spacing || 2000, rowHeading: alongX ? 0 : 90, canopyHeight: height, canopyWidth: width });
    const xs = pts.map((p) => p.x), ys = pts.map((p) => p.y);
    const mx = (alongX ? spacing : rowSpacing) / 2 || 1000, my = (alongX ? rowSpacing : spacing) / 2 || 1000;
    field.polygon = [[Math.min(...xs) - mx, Math.min(...ys) - my], [Math.max(...xs) + mx, Math.min(...ys) - my], [Math.max(...xs) + mx, Math.max(...ys) + my], [Math.min(...xs) - mx, Math.max(...ys) + my]];
    field.visible = false;
    field.params.semanticOverlay = true;
    field.params.source = 'webots trees';
    station.addChild(field);
    rows.forEach((r, ri) => {
      const row = new CropRow(`${field.name}_row${ri + 1}`);
      row.index = ri;
      row.start = alongX ? [r[0].x, rowCoord[ri]] : [rowCoord[ri], r[0].y];
      row.end = alongX ? [r[r.length - 1].x, rowCoord[ri]] : [rowCoord[ri], r[r.length - 1].y];
      row.segmentId = row.name;
      row.plants = r.map((p, k) => {
        const b = geometryBounds(p.t.geometry, opts.assets);
        p.t.params.row = row.name;
        p.t.params.plantIndex = k;
        return { s: alongX ? p.x - r[0].x : p.y - r[0].y, height: b ? b.max[2] : height, width: b ? Math.max(b.max[0] - b.min[0], b.max[1] - b.min[1]) : width, fruit: fruitOf(p.t), health: 1 };
      });
      field.addChild(row);
    });
    fields.push(field);
    report.push(`${field.name}: ${rows.length} rows, ${list.length} trees (row spacing ${rowSpacing.toFixed(0)} mm, plant spacing ${(spacing || 0).toFixed(0)} mm)`);
  }
  void (null as unknown as Item);
  return { fields, report };
}
