/**
 * Retail layout of the MURKA backend. In Webots the supervisor plugin spawns the products from
 * $MUROM_CONFIG_DIR/products.yaml + locations.yaml (murom_world/config, set by murom_webots/webots.launch.py) — a
 * package next to murom_webots. A world opened alone (File › Open folder murom_webots) therefore has no products;
 * the Connect dialog loads the layout from the murom_world/config folder (or any folder holding both files) the same
 * way the Webots import does for a folder that contains them (src/io/webots/retail_layout.ts, products as the
 * RetailProduct PROTO's coloured boxes).
 */
import type { Station } from '../core/items/item';
import { findMurom } from './murom_sim';

/** Products of a MURKA retail station (items with a SKU, spawned from products.yaml). */
export function retailProductCount(station: Station): number {
  let n = 0;
  for (const it of station.walk()) if (typeof it.params?.sku === 'string') n++;
  return n;
}

/** A MUROM station without products: the MURKA supervisor would have spawned them from murom_world/config. */
export function needsRetailLayout(station: Station): boolean {
  return !!findMurom(station) && retailProductCount(station) === 0;
}

/** products.yaml + locations.yaml of a file list (paths -> text), the pair from the same folder preferred. */
export function findLayoutPair(paths: string[]): { products: string; locations: string } | null {
  const norm = (p: string) => p.replace(/\\/g, '/');
  const dir = (p: string) => norm(p).replace(/[^/]*$/, '');
  const prods = paths.filter((p) => /(^|\/)products\.ya?ml$/i.test(norm(p)));
  const locs = paths.filter((p) => /(^|\/)locations\.ya?ml$/i.test(norm(p)));
  // murom_world/config first, then any folder holding both, shallowest first
  const rank = (p: string) => (/murom_world\/config\/[^/]*$/i.test(norm(p)) ? 0 : 1) * 1000 + norm(p).split('/').length;
  for (const p of [...prods].sort((a, b) => rank(a) - rank(b))) {
    const l = locs.find((x) => dir(x) === dir(p));
    if (l) return { products: p, locations: l };
  }
  return null;
}

/** Load the retail layout into a MURKA station from the files of a picked folder; returns the import report. */
export async function loadRetailLayoutFiles(station: Station, files: Map<string, string>): Promise<{ products: number; report: string[]; pair: { products: string; locations: string } | null }> {
  const pair = findLayoutPair([...files.keys()]);
  if (!pair) return { products: 0, report: ['no products.yaml + locations.yaml pair in the selection (murom_world/config)'], pair: null };
  // the Webots parsers stay out of the main bundle (the menu imports this module)
  const { loadRetailLayout } = await import('../io/webots/retail_layout');
  const r = loadRetailLayout(station, files.get(pair.products)!, files.get(pair.locations)!, { worldPath: station.filePath ?? undefined });
  // without the world's PROTO registry the products are built as boxes: RetailProduct.proto is exactly a coloured box
  const report = r.report.map((l) => (/^product PROTO .* products built as plain boxes/.test(l) ? 'products built as coloured boxes of the catalogue size (what RetailProduct.proto is)' : l));
  return { products: r.products.length, report, pair };
}
