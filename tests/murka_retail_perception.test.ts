/**
 * MURKA retail — classical perception (murom_perception) on frames rendered by the studio: the TS port must give the
 * same slot results as the original recognition.py on identical frames; the calibration (calibrate.py) on the studio's
 * own scan frames must reproduce the shipped prototypes; the classifier must see what is really on the shelves (an
 * emptied slot and two swapped products, not the products.yaml layout).
 */
import { describe, it, expect, beforeAll } from 'vitest';
import { buildMurkaRetailStation, runMurkaRetailMission, RetailStationResult, STUDIO_APPEARANCE } from '../src/murka/retail';
import { calibrate, loadCatalog, recognize, Frame } from '../src/murka/retail/perception';
import { AssetStore } from '../src/scene/assets';
import { transl } from '../src/core/math/pose';
import { HAS_MURKA, HAS_PY, runPython, V2_SRC } from './murka_retail_helpers';

let built: RetailStationResult;
let dumps: Array<{ target: string; ids: string[]; frame: Frame }>;

describe.skipIf(!HAS_MURKA)('MURKA perception on studio renders', () => {
  beforeAll(async () => {
    built = await buildMurkaRetailStation({ variant: 'v2', assets: new AssetStore(), loadMeshes: false });
    const run = runMurkaRetailMission(built, { mission: 'scan', dumpFrames: true });
    const res = await run.done;
    expect(res.outcome).toBe('done');
    dumps = run.session.orch.dumps;
  }, 300000);

  it('scan task: 6 views (2 per gondola), 24 visible slots = L0 + L1, all correct, L2 out of frame, 4 empty slots of A3 L1', () => {
    expect(dumps.map((d) => d.target)).toEqual(['shelf_A1', 'shelf_A1', 'shelf_A2', 'shelf_A2', 'shelf_A3', 'shelf_A3']);
    const cat = loadCatalog(built.layout, STUDIO_APPEARANCE.v2);
    const truth: Record<string, string> = {}; for (const i of built.layout.instances) truth[i.slot] = i.sku;
    const seen = new Map<string, string>();
    for (const d of dumps) for (const v of recognize(built.layout, d.ids, d.frame, cat)) if (v.visible) seen.set(v.slotId, v.sku);
    expect(seen.size).toBe(24);
    for (const [slot, sku] of seen) { expect(slot).toMatch(/_L[01]_/); expect(sku, slot).toBe(truth[slot] ?? ''); }
    expect([...seen].filter(([, s]) => s === '').map(([k]) => k).sort()).toEqual(['shelf_A3_L1_S0', 'shelf_A3_L1_S1', 'shelf_A3_L1_S2', 'shelf_A3_L1_S3']);
  });

  it('calibrate() on the scan frames reproduces the shipped studio prototypes (±0.002) with 24/24 before and after', () => {
    const c = calibrate(built.layout, dumps.map((d) => ({ ids: d.ids, frame: d.frame })));
    expect(c.before).toEqual([24, 24]);
    expect(c.after).toEqual([24, 24]);
    const ship = STUDIO_APPEARANCE.v2!;
    expect(Object.keys(c.appearance.skus).sort()).toEqual(Object.keys(ship.skus).sort());
    for (const [sku, d] of Object.entries(c.appearance.skus)) {
      d.rgb.forEach((v, i) => expect(Math.abs(v - ship.skus[sku]!.rgb[i]), sku).toBeLessThan(0.002));
      expect(d.samples).toBe(ship.skus[sku]!.samples);
    }
    // the renderer shades by view angle: prototypes keep the hue of the catalogue colour, not its value
    const cat = loadCatalog(built.layout);
    expect(Math.abs(c.appearance.skus.tea_box.hsv[0] - cat.tea_box.hsv[0])).toBeLessThan(0.01);
    expect(c.appearance.skus.tea_box.hsv[2]).toBeLessThan(cat.tea_box.hsv[2]);
  });

  it('the camera, not the layout file: an emptied slot reads empty, swapped products are recognised where they are', async () => {
    const b = await buildMurkaRetailStation({ variant: 'v2', assets: new AssetStore(), loadMeshes: false });
    const tea = b.station.find('tea_box_1')!, pasta = b.station.find('pasta_box_1')!, rice = b.station.find('rice_bag_1')!;
    const pt = tea.poseAbs(), pp = pasta.poseAbs();
    // tea_box_1 (A1 L0 S0) <-> pasta_box_1 (A1 L1 S0): centres keep each product on its board (size z / 2 + 2 mm)
    tea.setPoseAbs(transl(pp[12], pp[13], pp[14] - 95 + 80)); pasta.setPoseAbs(transl(pt[12], pt[13], pt[14] - 80 + 95));
    rice.setPoseAbs(transl(0, 0, -5000)); // rice_bag_1 (A1 L1 S2) removed
    const run = runMurkaRetailMission(b, { mission: 'scan', args: ['shelf_A1'] });
    const res = await run.done;
    expect(res.outcome).toBe('done');
    expect(res.inventory.shelf_A1_L0_S0[0]).toBe('pasta_box');
    expect(res.inventory.shelf_A1_L1_S0[0]).toBe('tea_box');
    expect(res.inventory.shelf_A1_L1_S2[0]).toBe('');
    expect(res.inventory.shelf_A1_L1_S3[0]).toBe('rice_bag');
    expect(Object.keys(res.inventory).every((k) => k.startsWith('shelf_A1_'))).toBe(true);
  }, 300000);
});

const PY_REC = String.raw`
import json, sys, math, numpy as np
inp = json.load(sys.stdin)
sys.path.insert(0, inp['pkg'])
import yaml
from murom_perception import recognition as rec
slots = yaml.safe_load(open(inp['locations'], encoding='utf-8'))['slots']
catalog = rec.load_catalog(yaml.safe_load(open(inp['products'], encoding='utf-8')))
rec.apply_appearance(catalog, inp['appearance'])
out = []
for i, f in enumerate(inp['frames']):
    h, w = f['h'], f['w']
    rgb = np.fromfile(inp['dir'] + f'/rgb{i}.bin', dtype=np.float32).reshape(h, w, 3).astype(np.float64)
    depth = np.fromfile(inp['dir'] + f'/depth{i}.bin', dtype=np.float32).reshape(h, w).astype(np.float64)
    depth[depth == 0] = np.inf            # no return of a Webots RangeFinder
    K = np.array([[f['K']['fx'], 0, f['K']['cx']], [0, f['K']['fy'], f['K']['cy']], [0, 0, 1]])
    T = np.array(f['T'], dtype=np.float64).reshape(4, 4)
    views = rec.recognize(slots, f['ids'], rgb, depth, K, T, catalog, planar=True)
    out.append([dict(slot=v.slot_id, visible=bool(v.visible), sku=v.sku, conf=float(v.confidence), n=int(v.n_points), width=float(v.width), height=float(v.height), rgb=[float(c) for c in v.rgb]) for v in views])
print(json.dumps(out))
`;

describe.skipIf(!HAS_PY)('MURKA perception: port = original recognition.py on the same studio frames', () => {
  it('every slot of the 6 scan frames: visibility, SKU, point count, face width / height, median colour, confidence', () => {
    const files: Record<string, Buffer> = {};
    dumps.forEach((d, i) => { files[`rgb${i}.bin`] = Buffer.from(new Float32Array(d.frame.rgb).buffer); files[`depth${i}.bin`] = Buffer.from(new Float32Array(d.frame.depth).buffer); });
    const py = runPython(PY_REC, {
      pkg: `${V2_SRC}/murom_perception`, locations: `${V2_SRC}/murom_world/config/locations.yaml`, products: `${V2_SRC}/murom_world/config/products.yaml`,
      appearance: STUDIO_APPEARANCE.v2, frames: dumps.map((d) => ({ h: d.frame.height, w: d.frame.width, K: d.frame.K, T: d.frame.T, ids: d.ids })),
    }, files) as Array<Array<{ slot: string; visible: boolean; sku: string; conf: number; n: number; width: number; height: number; rgb: number[] }>>;
    const cat = loadCatalog(built.layout, STUDIO_APPEARANCE.v2);
    let compared = 0;
    dumps.forEach((d, i) => {
      const ours = recognize(built.layout, d.ids, d.frame, cat);
      ours.forEach((v, j) => {
        const p = py[i][j]; compared++;
        expect([v.slotId, v.visible, v.sku, v.nPoints], `${i} ${v.slotId}`).toEqual([p.slot, p.visible, p.sku, p.n]);
        expect(Math.abs(v.width - p.width)).toBeLessThan(1e-6); expect(Math.abs(v.height - p.height)).toBeLessThan(1e-6);
        expect(Math.abs(v.confidence - p.conf)).toBeLessThan(1e-6);
        v.rgb.forEach((c, k) => expect(Math.abs(c - p.rgb[k])).toBeLessThan(1e-6));
      });
    });
    expect(compared).toBe(72);
  }, 300000);
});
