/** Shared helpers of the Webots import tests: a directory-backed WebotsSource and the MURKA paths. */
import fs from 'node:fs';
import path from 'node:path';
import type { WebotsSource } from '../src/io/webots';

export const MURKA = process.env.MURKA_DIR ?? 'C:/Users/TRITON 300 SE/YandexDisk/Работа/КБНЦ РАН/MURKA_Simulation';
export const HAS_MURKA = fs.existsSync(MURKA);
export const RETAIL_V2 = 'robot-retailer-murka-v2/ros2_ws/src/murom_webots';
export const RETAIL_V1 = 'robot-retailer-murka/ros2_ws/src/murom_webots';
export const AGRO = 'simulation_ws/src/agromarca/murca_garden_ws/src/murca_webots';
export const FIXTURES = path.resolve(__dirname, 'fixtures/webots');

const SKIP = new Set(['.git', 'build', 'install', 'log', 'node_modules', '__pycache__', '.venv', 'venv']);

/** Source over a directory tree; paths are relative to `root` with '/' separators. Listing is lazy and cached. */
export function dirSource(root: string): WebotsSource & { reads: string[] } {
  let listing: string[] | null = null;
  const reads: string[] = [];
  const walk = (d: string, out: string[]) => {
    let ents: fs.Dirent[];
    try { ents = fs.readdirSync(d, { withFileTypes: true }); } catch { return; }
    for (const e of ents) {
      if (e.isDirectory()) { if (!SKIP.has(e.name)) walk(path.join(d, e.name), out); }
      else out.push(path.relative(root, path.join(d, e.name)).split(path.sep).join('/'));
    }
  };
  return {
    reads,
    read(p: string) {
      const f = path.resolve(root, p);
      if (!f.startsWith(path.resolve(root))) return undefined;
      try { const b = fs.readFileSync(f); reads.push(p); return new Uint8Array(b.buffer, b.byteOffset, b.byteLength); } catch { return undefined; }
    },
    list() {
      if (!listing) { listing = []; walk(root, listing); }
      return listing;
    },
  };
}

/** Source over a sub-tree of MURKA (keeps listing small): `rel` is the folder holding worlds/ protos/ assets. */
export function murkaSource(rel: string): WebotsSource & { reads: string[] } {
  return dirSource(path.join(MURKA, rel));
}

/** All .wbt / .proto files under MURKA (relative paths). */
export function murkaWebotsFiles(): string[] {
  const out: string[] = [];
  const walk = (d: string) => {
    let ents: fs.Dirent[];
    try { ents = fs.readdirSync(d, { withFileTypes: true }); } catch { return; }
    for (const e of ents) {
      if (e.isDirectory()) { if (!SKIP.has(e.name)) walk(path.join(d, e.name)); }
      else if (/\.(wbt|proto)$/i.test(e.name)) out.push(path.relative(MURKA, path.join(d, e.name)).split(path.sep).join('/'));
    }
  };
  if (HAS_MURKA) walk(MURKA);
  return out.sort();
}
