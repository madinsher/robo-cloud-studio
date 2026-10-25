/**
 * Webots viewpoint helpers and the "shots" player of the MURKA trellis stills world (orchard_trellis_stills.wbt +
 * orchard_trellis_stills_shots.json): a shot sets the viewpoint (position, axis-angle orientation, fieldOfView) and
 * brings its dynamic group `DYN_<KEY>` from the parking depth (z = -1000 m) to z = 0 while the other groups are parked —
 * what the capture supervisor does in Webots. Viewpoints follow the R2022a+ convention: the camera looks along its +X
 * with +Z up; fieldOfView is horizontal (Webots derives the vertical angle from the aspect ratio).
 */
import type { Item, Station, ParamValue } from '../../core/items/item';
import { Vec3, transl, multiply, invert } from '../../core/math/pose';

export interface WebotsViewpoint { position: Vec3; orientation: [number, number, number, number]; fieldOfView: number }
export interface WebotsShot extends WebotsViewpoint { key: string; shot?: string; frame?: number; t?: number; url?: string }

/** Rotate v by the axis-angle (x, y, z, angle). */
function rotate(aa: number[], v: Vec3): Vec3 {
  const n = Math.hypot(aa[0], aa[1], aa[2]) || 1, k: Vec3 = [aa[0] / n, aa[1] / n, aa[2] / n], c = Math.cos(aa[3]), s = Math.sin(aa[3]);
  const d = k[0] * v[0] + k[1] * v[1] + k[2] * v[2];
  const x: Vec3 = [k[1] * v[2] - k[2] * v[1], k[2] * v[0] - k[0] * v[2], k[0] * v[1] - k[1] * v[0]];
  return [0, 1, 2].map((i) => v[i] * c + x[i] * s + k[i] * d * (1 - c)) as Vec3;
}

/** Eye, look-at point (1 m ahead) and up vector (mm) plus the horizontal FOV (deg) of a Webots viewpoint (position in mm). */
export function viewpointLook(vp: WebotsViewpoint): { eye: Vec3; target: Vec3; up: Vec3; fovDeg: number; fovAxis: 'horizontal' } {
  const f = rotate(vp.orientation, [1, 0, 0]), up = rotate(vp.orientation, [0, 0, 1]);
  return { eye: [...vp.position] as Vec3, target: [0, 1, 2].map((i) => vp.position[i] + 1000 * f[i]) as Vec3, up, fovDeg: (vp.fieldOfView * 180) / Math.PI, fovAxis: 'horizontal' };
}

/** Parse orchard_trellis_stills_shots.json ({shots: [{key, position (m), orientation, fov, ...}]}); positions -> mm. */
export function parseShots(json: string | { shots?: unknown[] }): WebotsShot[] {
  const d = (typeof json === 'string' ? JSON.parse(json) : json) as { shots?: Array<Record<string, unknown>> };
  return (d.shots ?? []).map((s) => {
    const p = s.position as number[], o = s.orientation as number[];
    if (!Array.isArray(p) || p.length < 3 || !Array.isArray(o) || o.length < 4 || typeof s.key !== 'string') throw new Error(`shot ${JSON.stringify(s.key)}: position / orientation / key missing`);
    return { key: s.key, position: [p[0] * 1000, p[1] * 1000, p[2] * 1000], orientation: [o[0], o[1], o[2], o[3]], fieldOfView: Number(s.fov ?? 0.785398), ...(s.shot ? { shot: String(s.shot) } : {}), ...(typeof s.frame === 'number' ? { frame: s.frame } : {}), ...(typeof s.t === 'number' ? { t: s.t } : {}), ...(s.url ? { url: String(s.url) } : {}) };
  });
}

/** Outermost items whose DEF (or name) starts with the prefix (their generated children such as `<DEF>_shapes` excluded). */
const groupsOf = (station: Station, prefix: string): Item[] => {
  const hit = new Set([...station.walk()].filter((i) => i !== station && String(i.params.def ?? i.name).startsWith(prefix)));
  return [...hit].filter((i) => { for (let p = i.parent; p; p = p.parent) if (hit.has(p)) return false; return true; });
};

/**
 * Apply a shot: the viewpoint goes to station.settings.viewpoint (same shape as the imported Viewpoint), DYN_<KEY>
 * is moved to z = 0, every other DYN_* group back to its parking z (the z it had at import, -1000 m in the world).
 */
export function applyShot(station: Station, shot: WebotsShot, opts: { prefix?: string } = {}): { shown: Item | null; parked: Item[] } {
  const prefix = opts.prefix ?? 'DYN_';
  const want = `${prefix}${shot.key.toUpperCase()}`;
  let shown: Item | null = null;
  const parked: Item[] = [];
  for (const g of groupsOf(station, prefix)) {
    if (g.params.parkZ === undefined) g.params.parkZ = g.poseAbs()[14];
    const P = g.poseAbs();
    const z = String(g.params.def ?? g.name) === want ? 0 : Number(g.params.parkZ);
    const abs = multiply(transl(P[12], P[13], z), (() => { const R = new Float64Array(P); R[12] = R[13] = R[14] = 0; return R; })());
    g.setPose(g.parent && g.parent !== station ? multiply(invert(g.parent.poseAbs()), abs) : abs);
    if (z === 0) shown = g; else parked.push(g);
  }
  const vp = (station.settings.viewpoint ?? {}) as Record<string, ParamValue>;
  station.settings.viewpoint = { ...vp, position: [...shot.position], orientation: [...shot.orientation], fieldOfView: shot.fieldOfView, shot: shot.key };
  return { shown, parked };
}
