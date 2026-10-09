/** Geodesy helpers: WGS84 lat/lon <-> local East-North (mm) around an origin, GeoJSON field import. */
const R = 6378137;
export interface GeoOrigin { lat: number; lon: number; alt?: number }

export function llToLocal(lat: number, lon: number, origin: GeoOrigin): [number, number] {
  const dLat = ((lat - origin.lat) * Math.PI) / 180;
  const dLon = ((lon - origin.lon) * Math.PI) / 180;
  const x = dLon * R * Math.cos((origin.lat * Math.PI) / 180);
  const y = dLat * R;
  return [x * 1000, y * 1000];
}
export function localToLL(xMm: number, yMm: number, origin: GeoOrigin): [number, number] {
  const lat = origin.lat + ((yMm / 1000 / R) * 180) / Math.PI;
  const lon = origin.lon + ((xMm / 1000 / (R * Math.cos((origin.lat * Math.PI) / 180))) * 180) / Math.PI;
  return [lat, lon];
}

export interface GeoField { name: string; polygon: number[][]; properties: Record<string, any> }

/** Import polygons from GeoJSON (FeatureCollection/Feature/Polygon). Returns local mm polygons. */
export function fieldsFromGeoJSON(geojson: any, origin?: GeoOrigin): { fields: GeoField[]; origin: GeoOrigin } {
  const feats: any[] = geojson.type === 'FeatureCollection' ? geojson.features : geojson.type === 'Feature' ? [geojson] : [{ type: 'Feature', geometry: geojson, properties: {} }];
  const polys: Array<{ ring: number[][]; props: any }> = [];
  for (const f of feats) {
    const g = f.geometry;
    if (!g) continue;
    if (g.type === 'Polygon') polys.push({ ring: g.coordinates[0], props: f.properties ?? {} });
    if (g.type === 'MultiPolygon') for (const p of g.coordinates) polys.push({ ring: p[0], props: f.properties ?? {} });
  }
  if (!polys.length) return { fields: [], origin: origin ?? { lat: 0, lon: 0 } };
  if (!origin) {
    const all = polys.flatMap((p) => p.ring);
    origin = { lat: all.reduce((s, c) => s + c[1], 0) / all.length, lon: all.reduce((s, c) => s + c[0], 0) / all.length };
  }
  const o = origin;
  const fields = polys.map((p, i) => ({
    name: p.props.name ?? p.props.Name ?? `Field ${i + 1}`,
    polygon: p.ring.slice(0, p.ring.length > 1 && p.ring[0][0] === p.ring[p.ring.length - 1][0] && p.ring[0][1] === p.ring[p.ring.length - 1][1] ? -1 : undefined).map(([lon, lat]: number[]) => llToLocal(lat, lon, o)),
    properties: p.props,
  }));
  return { fields, origin: o };
}

export function fieldsToGeoJSON(fields: Array<{ name: string; polygon: number[][] }>, origin: GeoOrigin): any {
  return {
    type: 'FeatureCollection',
    features: fields.map((f) => ({
      type: 'Feature',
      properties: { name: f.name },
      geometry: { type: 'Polygon', coordinates: [[...f.polygon, f.polygon[0]].map(([x, y]) => { const [lat, lon] = localToLL(x, y, origin); return [lon, lat]; })] },
    })),
  };
}

export function polygonArea(poly: number[][]): number {
  let a = 0;
  for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) a += poly[j][0] * poly[i][1] - poly[i][0] * poly[j][1];
  return Math.abs(a) / 2;
}
