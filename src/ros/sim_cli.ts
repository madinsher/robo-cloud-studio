/**
 * Command line host of the headless ROS 2 simulator backend (./sim_backend.ts) — the studio in place of Webots for a
 * ROS 2 stack (rosbridge_server must run: `ros2 run rosbridge_server rosbridge_websocket`).
 *
 *   npx vite build --ssr src/ros/sim_cli.ts --outDir dist/ros-sim      (once; dist/ is git-ignored)
 *   node dist/ros-sim/sim_cli.js --murka "<MURKA_Simulation>" [--variant v2|v1] [--url ws://127.0.0.1:9090]
 *   node dist/ros-sim/sim_cli.js --world <dir>/worlds/x.wbt [--layout <dir with products.yaml, locations.yaml>]
 *   node dist/ros-sim/sim_cli.js --station my_cell.vbs
 *   options: --rtf 1 (0 = as fast as possible), --camera-rate 5 (Hz, 0 = no images), --no-lidar-noise, --seed N
 */
import fs from 'node:fs';
import path from 'node:path';
import type { Station } from '../core/items/item';
import { importWebotsWorld, loadRetailLayout, type WebotsSource } from '../io/webots';
import { AssetStore } from '../scene/assets';
import { startSimBackend } from './sim_backend';

const SKIP = new Set(['.git', 'build', 'install', 'log', 'node_modules', '__pycache__']);
/** WebotsSource over a directory (paths relative to `root`, '/' separators). */
export function nodeDirSource(root: string): WebotsSource {
  let listing: string[] | null = null;
  const walk = (d: string, out: string[]) => {
    let ents: fs.Dirent[];
    try { ents = fs.readdirSync(d, { withFileTypes: true }); } catch { return; }
    for (const e of ents) { if (e.isDirectory()) { if (!SKIP.has(e.name)) walk(path.join(d, e.name), out); } else out.push(path.relative(root, path.join(d, e.name)).split(path.sep).join('/')); }
  };
  return {
    read(p: string) { const f = path.resolve(root, p); if (!f.startsWith(path.resolve(root))) return undefined; try { const b = fs.readFileSync(f); return new Uint8Array(b.buffer, b.byteOffset, b.byteLength); } catch { return undefined; } },
    list() { if (!listing) { listing = []; walk(root, listing); } return listing; },
  };
}

const MURKA_VARIANTS: Record<string, string> = { v2: 'robot-retailer-murka-v2', v1: 'robot-retailer-murka' };

/** A Webots world (+ optional retail layout folder) as a station. */
export function loadWorldStation(worldFile: string, layoutDir: string | null, assets: AssetStore): Station {
  // the source root is two levels above the world (worlds/ next to protos/ and ../assets)
  const root = path.resolve(path.dirname(worldFile), '..', '..');
  const rel = path.relative(root, path.resolve(worldFile)).split(path.sep).join('/');
  const r = importWebotsWorld(rel, nodeDirSource(root), { assets });
  if (layoutDir) {
    const p = path.join(layoutDir, 'products.yaml'), l = path.join(layoutDir, 'locations.yaml');
    if (fs.existsSync(p) && fs.existsSync(l)) {
      const lr = loadRetailLayout(r.station, fs.readFileSync(p, 'utf8'), fs.readFileSync(l, 'utf8'), { builder: r.builder, worldPath: rel });
      console.log(`[sim-cli] ${lr.report.join('; ')}`);
    }
  }
  return r.station;
}

async function main(argv: string[]): Promise<void> {
  const arg = (k: string) => { const i = argv.indexOf(k); return i >= 0 ? argv[i + 1] : undefined; };
  const assets = new AssetStore();
  let station: Station;
  if (arg('--murka')) {
    const base = path.join(arg('--murka')!, MURKA_VARIANTS[arg('--variant') ?? 'v2'] ?? MURKA_VARIANTS.v2);
    station = loadWorldStation(path.join(base, 'ros2_ws/src/murom_webots/worlds/retail_store.wbt'), path.join(base, 'ros2_ws/src/murom_world/config'), assets);
  } else if (arg('--world')) station = loadWorldStation(arg('--world')!, arg('--layout') ?? null, assets);
  else if (arg('--station')) {
    const { loadStation } = await import('../io/station-file');
    station = loadStation(JSON.parse(fs.readFileSync(arg('--station')!, 'utf8')), assets);
  } else { console.log('usage: sim_cli --murka <MURKA_Simulation> | --world <x.wbt> [--layout <dir>] | --station <x.vbs> [--url ws://127.0.0.1:9090] [--rtf 1] [--camera-rate 5]'); process.exit(2); }
  const h = await startSimBackend({
    url: arg('--url') ?? 'ws://127.0.0.1:9090', station, assets, rtf: Number(arg('--rtf') ?? 1),
    murom: { cameraRate: Number(arg('--camera-rate') ?? 5), lidarNoise: !argv.includes('--no-lidar-noise'), seed: arg('--seed') ? Number(arg('--seed')) : undefined },
  });
  const timer = setInterval(() => console.log(`[sim-cli] sim ${h.time().toFixed(1)} s, RTF ${h.rtf().toFixed(2)}, ${h.client?.stats.sent ?? 0} messages`), 30000);
  process.on('SIGINT', () => { h.stop(); });
  await h.done;
  clearInterval(timer);
}

const isMain = typeof process !== 'undefined' && process.argv[1] && /sim_cli\.(c?js|ts)$/.test(process.argv[1]);
if (isMain) main(process.argv.slice(2)).catch((e) => { console.error(e); process.exit(1); });
