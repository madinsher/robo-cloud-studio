import { it } from 'vitest';
import { importWebotsWorld } from '../src/io/webots';
import { AssetStore } from '../src/scene/assets';
import { dirSource, MURKA, murkaWebotsFiles } from './webots_helpers';

it('all worlds', () => {
  for (const f of murkaWebotsFiles().filter((p) => p.endsWith('.wbt'))) {
    const t0 = Date.now();
    const assets = new AssetStore();
    // root = 4 levels above the world (covers ../../../assets)
    const parts = f.split('/');
    const root = parts.slice(0, Math.max(0, parts.length - 5)).join('/');
    const rel = parts.slice(Math.max(0, parts.length - 5)).join('/');
    try {
      const r = importWebotsWorld(rel, dirSource(`${MURKA}/${root}`), { assets, loadMeshes: !/trellis/.test(f) });
      console.log(`== ${f} ${Date.now() - t0} ms items=${[...r.station.walk()].length}`);
      console.log(r.report.map((l) => '   ' + l).join('\n'));
    } catch (e) {
      console.log(`== ${f} FAILED ${(e as Error).message}`);
    }
  }
}, 600000);
