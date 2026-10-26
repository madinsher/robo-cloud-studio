/**
 * Station collision world for mobile robots: the navigation map plus circle obstacles contributed by registered
 * providers (the agri module registers tree trunks and canopies with their height bands). Cached per station until
 * the map or a provider signature changes.
 */
import { Station, ItemType } from '../core/items/item';
import { MapItem } from './items';
import { CollisionWorld, CircleObstacle } from './footprint';

export interface ObstacleProvider {
  id: string;
  /** Cheap signature: the world is rebuilt when it changes. */
  signature(station: Station): string;
  obstacles(station: Station): CircleObstacle[];
}

const providers: ObstacleProvider[] = [];

/** Register (or replace by id) an obstacle provider. */
export function registerObstacleProvider(p: ObstacleProvider): void {
  const i = providers.findIndex((q) => q.id === p.id);
  if (i >= 0) providers[i] = p; else providers.push(p);
}

const cache = new WeakMap<Station, { key: string; world: CollisionWorld }>();

/** Collision world of a station (map: the given one, else the first map item). */
export function stationWorld(station: Station, map?: MapItem | null): CollisionWorld {
  const m = map === undefined ? (station.itemsOfType<MapItem>(ItemType.MAP)[0] ?? null) : map;
  const key = `${m ? `${m.id}:${(m as any)._version ?? 0}:${m.width}x${m.height}` : '-'}|${providers.map((p) => `${p.id}:${p.signature(station)}`).join('|')}`;
  const c = cache.get(station);
  if (c && c.key === key) return c.world;
  const circles: CircleObstacle[] = [];
  for (const p of providers) circles.push(...p.obstacles(station));
  const world = new CollisionWorld({ map: m, circles });
  cache.set(station, { key, world });
  return world;
}

/** Drop the cached world (after editing obstacles in place). */
export function invalidateStationWorld(station: Station): void { cache.delete(station); }
