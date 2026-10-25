// Geometry helpers for the agri / mobile acceptance tests: independent of the fleet manager's own contact checks.
import { Station, ItemType } from '../src/core/items/item';
import { CropRow } from '../src/agri/items';
import { MobileRobot } from '../src/mobile/items';

export interface PlantDisk { x: number; y: number; r: number; row: CropRow; idx: number }

/** Station-frame plant positions with their own canopy radius. */
export function plantDisks(st: Station): PlantDisk[] {
  const out: PlantDisk[] = [];
  for (const row of st.itemsOfType<CropRow>(ItemType.CROP_ROW)) {
    const abs = row.parent ? row.parent.poseAbs() : null;
    row.plants.forEach((p, idx) => {
      const [lx, ly] = row.pointAt(p.s);
      out.push({ x: abs ? abs[0] * lx + abs[4] * ly + abs[12] : lx, y: abs ? abs[1] * lx + abs[5] * ly + abs[13] : ly, r: p.width / 2, row, idx });
    });
  }
  return out;
}

/** Distance from a point to the robot's footprint rectangle (0 inside). */
export function pointFootprintDistance(px: number, py: number, r: MobileRobot): number {
  const th = (r.state.theta * Math.PI) / 180, dx = px - r.state.x, dy = py - r.state.y;
  const lx = Math.cos(th) * dx + Math.sin(th) * dy, ly = -Math.sin(th) * dx + Math.cos(th) * dy;
  return Math.hypot(Math.max(0, Math.abs(lx) - r.kin.footprint[0] / 2), Math.max(0, Math.abs(ly) - r.kin.footprint[1] / 2));
}

/** Clearance of a robot footprint to trunks (radius trunkR) and to canopy disks; negative = penetration (mm). */
export function cropClearance(r: MobileRobot, plants: PlantDisk[], trunkR = 75): { trunk: number; canopy: number } {
  let trunk = Infinity, canopy = Infinity;
  const reach = Math.hypot(r.kin.footprint[0], r.kin.footprint[1]) / 2 + 3000;
  for (const p of plants) {
    if (Math.abs(p.x - r.state.x) > reach || Math.abs(p.y - r.state.y) > reach) continue;
    const d = pointFootprintDistance(p.x, p.y, r);
    trunk = Math.min(trunk, d - trunkR);
    canopy = Math.min(canopy, d - p.r);
  }
  return { trunk, canopy };
}

/** Separating-axis overlap of two robot footprints. */
export function footprintsOverlap(a: MobileRobot, b: MobileRobot): boolean {
  const corners = (r: MobileRobot) => {
    const th = (r.state.theta * Math.PI) / 180, c = Math.cos(th), s = Math.sin(th);
    const [L, W] = r.kin.footprint;
    return [[L / 2, W / 2], [L / 2, -W / 2], [-L / 2, -W / 2], [-L / 2, W / 2]].map(([u, v]) => [r.state.x + c * u - s * v, r.state.y + s * u + c * v]);
  };
  const A = corners(a), B = corners(b);
  for (const r of [a, b]) {
    const th = (r.state.theta * Math.PI) / 180;
    for (const [ax, ay] of [[Math.cos(th), Math.sin(th)], [-Math.sin(th), Math.cos(th)]]) {
      const pa = A.map((p) => p[0] * ax + p[1] * ay), pb = B.map((p) => p[0] * ax + p[1] * ay);
      if (Math.max(...pa) < Math.min(...pb) || Math.max(...pb) < Math.min(...pa)) return false;
    }
  }
  return true;
}

/** Fruit counters over the whole station. */
export function fruitCounts(st: Station): { total: number; ripe: number; picked: number; unripePicked: number } {
  let total = 0, ripe = 0, picked = 0, unripePicked = 0;
  for (const row of st.itemsOfType<CropRow>(ItemType.CROP_ROW)) for (const p of row.plants) for (const f of p.fruit) {
    total++;
    if (f.ripe >= 0.5) ripe++;
    if (f.picked) { picked++; if (f.ripe < 0.5) unripePicked++; }
  }
  return { total, ripe, picked, unripePicked };
}

/** Distance (mm) from a point to the infinite line through a row (station frame). */
export function rowAxisDistance(row: CropRow, x: number, y: number): number {
  const abs = row.parent ? row.parent.poseAbs() : null;
  const w = (p: number[]) => (abs ? [abs[0] * p[0] + abs[4] * p[1] + abs[12], abs[1] * p[0] + abs[5] * p[1] + abs[13]] : p);
  const a = w(row.start), b = w(row.end);
  const dx = b[0] - a[0], dy = b[1] - a[1], l = Math.hypot(dx, dy) || 1;
  return Math.abs((x - a[0]) * dy - (y - a[1]) * dx) / l;
}
