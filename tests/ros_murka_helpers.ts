/** R5 test helpers: the MURKA retail store as imported by the studio (retail_store.wbt + murom_world/config). */
import fs from 'node:fs';
import path from 'node:path';
import { importWebotsWorld, loadRetailLayout } from '../src/io/webots';
import { AssetStore } from '../src/scene/assets';
import type { Station } from '../src/core/items/item';
import { MURKA, RETAIL_V2, murkaSource } from './webots_helpers';

export const MURKA_V2 = path.join(MURKA, 'robot-retailer-murka-v2');
export const MURKA_CONFIG = path.join(MURKA_V2, 'ros2_ws/src/murom_world/config');

/** retail_store.wbt (MUROM v2 decomposed: base + arms + head) with the 50 products of products.yaml in their slots. */
export function buildRetailStation(): { station: Station; assets: AssetStore; productsYaml: string; locationsYaml: string } {
  const assets = new AssetStore();
  const r = importWebotsWorld('worlds/retail_store.wbt', murkaSource(RETAIL_V2), { assets });
  const productsYaml = fs.readFileSync(path.join(MURKA_CONFIG, 'products.yaml'), 'utf8');
  const locationsYaml = fs.readFileSync(path.join(MURKA_CONFIG, 'locations.yaml'), 'utf8');
  loadRetailLayout(r.station, productsYaml, locationsYaml, { builder: r.builder, worldPath: 'worlds/retail_store.wbt' });
  return { station: r.station, assets, productsYaml, locationsYaml };
}
