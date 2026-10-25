/**
 * Rest height of an imported wheeled robot. Webots drops a robot with physics onto the floor during the first steps
 * (MURKA's Murom / Murka: translation z 0.01, base_link at z ≈ -0.007 in Webots); the studio is kinematic and keeps
 * the world translation, which leaves the base — and every IK target expressed in base_link — 1–2 cm too high.
 * restOnGround lowers / raises the base so that the lowest point of its boundingObjects (wheels, caster, bumpers)
 * touches the ground below it: the top of the floor-like Solids under the robot (params.ground), else z = 0. The base
 * is kept level (a two-wheel base with a caster at the wheel bottom, like MURKA); the sink of the soft ODE contact
 * (a few mm, depending on the mass and softCFM) is not modelled.
 */
import type { Station, GeometryRef } from '../../core/items/item';
import { SceneObject } from '../../core/items/item';
import type { MobileRobot } from '../../mobile/items';
import type { AssetStore } from '../../scene/assets';
import { geometryBounds } from '../kintree';
import { multiply, fromArray, identity } from '../../core/math/pose';

export interface RestResult { from: number; to: number; lowest: number; ground: number }

/** Top (mm) of the ground under the point (x, y): the highest floor-like Solid whose extent contains it, else 0. */
export function groundTopAt(station: Station, x: number, y: number, assets?: AssetStore): number {
  let top: number | null = null;
  for (const it of station.walk()) {
    if (!(it instanceof SceneObject) || !it.params.ground) continue;
    const P = it.poseAbs();
    const refs = (it.colliders && it.colliders.length ? it.colliders : it.geometry).map((g): GeometryRef => ({ ...g, origin: Array.from(multiply(P, g.origin ? fromArray(g.origin) : identity())) }));
    const b = geometryBounds(refs, assets);
    if (!b || x < b.min[0] || x > b.max[0] || y < b.min[1] || y > b.max[1]) continue;
    top = top === null ? b.max[2] : Math.max(top, b.max[2]);
  }
  return top ?? 0;
}

/**
 * Put a mobile robot onto the ground like the Webots physics does: base z = ground top - lowest point of its
 * boundingObjects in the base frame. Only a robot standing within `maxGap` (default 100 mm) of the ground is moved
 * (a robot placed on a table or a drone stays where the world puts it). Returns null when nothing was changed.
 */
export function restOnGround(mobile: MobileRobot, station: Station, opts: { assets?: AssetStore; maxGap?: number } = {}): RestResult | null {
  if (mobile.kin.drive === 'aerial' || mobile.params.aerial) return null;
  const refs = mobile.colliders && mobile.colliders.length ? mobile.colliders : null;
  if (!refs) return null;
  const b = geometryBounds(refs, opts.assets);
  if (!b) return null;
  const P = mobile.poseAbs();
  // level base only (rotation about the vertical): the lowest point is then z + min z of the bounds
  if (Math.abs(P[10] - 1) > 1e-6) return null;
  const ground = groundTopAt(station, P[12], P[13], opts.assets);
  const from = P[14], to = ground - b.min[2];
  if (Math.abs(to - from) > (opts.maxGap ?? 100)) return null;
  if (Math.abs(to - from) < 1e-9) return null;
  const Q = Float64Array.from(P); Q[14] = to;
  mobile.setPoseAbs(Q);
  return { from, to, lowest: b.min[2], ground };
}
