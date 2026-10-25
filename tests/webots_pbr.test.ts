// @vitest-environment jsdom
/**
 * Webots PBR appearance in the studio: PBRAppearance metalness / roughness / emissiveColor × emissiveIntensity /
 * IBLStrength (Webots defaults: metalness 1, roughness 0) and a legacy Material (metalness 0, roughness 1 - shininess,
 * emissiveColor) are carried from appearanceOf (src/io/webots/geometry.ts) on every geometry — primitives, meshes,
 * textured primitives (their file material), robot link visuals — to the rendered three.js materials
 * (three-geometry.ts materialFor / fileMaterial; studio materials keep metalness 0.22 / roughness 0.42 and no
 * environment reflection). The MURKA worlds are checked shape by shape against what Webots R2025a itself reports for
 * the same shapes (tests/fixtures/webots/gt_appearance: supervisor dumps of the verification, every Shape's full
 * appearance at its world position).
 */
import { describe, it, expect } from 'vitest';
import fs from 'node:fs';
import path from 'node:path';
import zlib from 'node:zlib';
import * as THREE from 'three';
import { importWebotsFiles, importWebotsWorld } from '../src/io/webots';
import { AssetStore } from '../src/scene/assets';
import { Item, GeometryRef, SceneObject } from '../src/core/items/item';
import { Robot } from '../src/core/items/robot';
import { Mat4, Vec3, identity, multiply, transformPoint, fromArray } from '../src/core/math/pose';
import { buildGeometry, materialFor, buildLinkVisuals } from '../src/scene/three-geometry';
import { HAS_MURKA, murkaSource, MURKA } from './webots_helpers';

const enc = (s: string) => new TextEncoder().encode(s);
const PNG = Uint8Array.from(atob('iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAYAAAAfFcSJAAAADUlEQVR42mP8z8BQDwAEhQGAhKmMIQAAAABJRU5ErkJggg=='), (c) => c.charCodeAt(0));
const WORLD = `#VRML_SIM R2025a utf8
WorldInfo { }
Solid { name "defaults" children [ Shape { appearance PBRAppearance { } geometry Box { size 0.1 0.1 0.1 } } ] }
Solid { name "led" children [ Shape { appearance PBRAppearance { baseColor 0 0.62 0.6 roughness 0.3 metalness 0 emissiveColor 0.1 0.95 0.88 emissiveIntensity 1.5 IBLStrength 0.5 } geometry Sphere { radius 0.1 } } ] }
Solid { name "legacy" children [ Shape { appearance Appearance { material Material { diffuseColor 1 0 0 shininess 0.7 emissiveColor 0 0 1 } } geometry Box { size 0.1 0.1 0.1 } } ] }
Solid { name "textured" children [ Shape { appearance PBRAppearance { baseColorMap ImageTexture { url [ "t.png" ] } metalness 0.75 roughness 0.25 } geometry Cylinder { radius 0.1 height 0.2 } } ] }
Robot { name "bot" children [ HingeJoint { jointParameters HingeJointParameters { axis 0 0 1 } device [ RotationalMotor { name "m" } ] endPoint Solid { name "link" children [ Shape { appearance PBRAppearance { baseColor 0.7 0.7 0.7 metalness 1 roughness 0.26 } geometry Box { size 0.1 0.1 0.1 } } ] } } ] }
`;

const geoOf = (st: Item, name: string) => (st.find(name) as SceneObject).geometry;

describe('Webots PBR appearance -> GeometryRef.pbr -> three.js material', () => {
  it('PBRAppearance defaults, an emissive LED, a legacy Material, a textured primitive and a robot link', async () => {
    const assets = new AssetStore();
    const { station } = await importWebotsFiles(new Map([['w/worlds/w.wbt', enc(WORLD)], ['w/worlds/t.png', PNG]]), assets);
    expect(geoOf(station, 'defaults')[0].pbr).toEqual({ metalness: 1, roughness: 0 });
    expect(geoOf(station, 'led')[0].pbr).toEqual({ metalness: 0, roughness: 0.3, emissive: '#1af2e0', emissiveIntensity: 1.5, ibl: 0.5 });
    expect(geoOf(station, 'legacy')[0].pbr).toEqual({ metalness: 0, roughness: expect.closeTo(0.3, 9), emissive: '#0000ff' });
    // the textured cylinder: a textured mesh whose embedded material carries the PBR parameters
    const tex = geoOf(station, 'textured')[0];
    expect(tex.mesh).toMatch(/#tex=/); expect(tex.pbr).toEqual({ metalness: 0.75, roughness: 0.25 });
    const fm = assets.get(tex.mesh!)!.mesh!.materials![0];
    expect([fm.metalness, fm.roughness, fm.ibl]).toEqual([0.75, 0.25, 1]);
    // robot link visuals keep it (LinkVisual)
    const bot = [...station.walk()].find((i): i is Robot => i instanceof Robot)!;
    const v = bot.chain.links.flatMap((l) => l.visuals).find((x) => x.primitive?.kind === 'box') as { pbr?: GeometryRef['pbr'] };
    expect(v.pbr).toEqual({ metalness: 1, roughness: 0.26 });

    // rendered materials
    const mat = (refs: GeometryRef[]) => ((buildGeometry(refs, assets).children[0] as THREE.Mesh).material as THREE.MeshStandardMaterial);
    const led = mat(geoOf(station, 'led'));
    expect([led.metalness, led.roughness, led.emissive.getHexString(), led.emissiveIntensity, led.envMapIntensity]).toEqual([0, 0.3, new THREE.Color('#1af2e0').getHexString(), 1.5, 0.5]);
    const metal = mat(geoOf(station, 'defaults'));
    expect([metal.metalness, metal.roughness, metal.envMapIntensity]).toEqual([1, 0, 1]);
    const t = mat([tex]);
    expect([t.metalness, t.roughness, t.envMapIntensity, !!t.map]).toEqual([0.75, 0.25, 1, true]);
    const link = (buildLinkVisuals(bot.chain.links.flatMap((l) => l.visuals), assets, '#888').children[0] as THREE.Mesh).material as THREE.MeshStandardMaterial;
    expect([link.metalness, link.roughness]).toEqual([1, 0.26]);
    // studio materials are unchanged and take no environment reflection
    const plain = materialFor('#336699');
    expect([plain.metalness, plain.roughness, plain.envMapIntensity, plain.emissive.getHex()]).toEqual([0.22, 0.42, 0, 0]);
  });
});

// ---------------------------------------------------------------------------------------------------------------
// MURKA worlds: every Shape vs what Webots R2025a reports for it
interface FxShape { kind: string; pts: Vec3[]; sig: number[]; mesh?: string; label: string; app: { type: string; metalness?: number; roughness?: number; emissiveColor?: number[]; emissiveIntensity?: number; IBLStrength?: number; material?: { shininess?: number; emissiveColor?: number[] } } }
interface StShape { kind: string; pts: Vec3[]; sig: number[]; mesh?: string; pbr?: GeometryRef['pbr'] }
function shapeOf(kind: string, dims: number[], M: Mat4): { pts: Vec3[]; sig: number[] } {
  const P = (x: number, y: number, z: number) => transformPoint(M, [x, y, z]);
  switch (kind) {
    case 'box': { const [a, b, c] = dims.map((v) => v / 2); const pts: Vec3[] = []; for (let k = 0; k < 8; k++) pts.push(P(k & 1 ? a : -a, k & 2 ? b : -b, k & 4 ? c : -c)); return { pts, sig: [...dims].sort((x, y) => x - y) }; }
    case 'cylinder': case 'cone': return { pts: [P(0, 0, dims[1] / 2), P(0, 0, -dims[1] / 2)], sig: dims };
    case 'plane': { const [a, b] = dims.map((v) => v / 2); return { pts: [P(a, b, 0), P(-a, b, 0), P(a, -b, 0), P(-a, -b, 0)], sig: [...dims].sort((x, y) => x - y) }; }
    case 'sphere': return { pts: [P(0, 0, 0)], sig: dims };
    default: return { pts: [P(0, 0, 0), P(100, 0, 0), P(0, 100, 0), P(0, 0, 100)], sig: dims };
  }
}
const base = (u: string) => u.split(/[\\/]/).pop()!.toLowerCase();
function studioShapes(root: Item): StShape[] {
  const out: StShape[] = [];
  const add = (g: GeometryRef & { pbr?: GeometryRef['pbr'] }, M: Mat4) => {
    const p = g.primitive; let s: { pts: Vec3[]; sig: number[] } | null = null; let kind = '';
    if (p?.kind === 'box') { kind = 'box'; s = shapeOf(kind, p.size, M); }
    else if (p?.kind === 'cylinder' || p?.kind === 'cone') { kind = p.kind; s = shapeOf(kind, [p.radius, p.length], M); }
    else if (p?.kind === 'sphere') { kind = 'sphere'; s = shapeOf(kind, [p.radius], M); }
    else if (p?.kind === 'plane') { kind = 'plane'; s = shapeOf(kind, p.size, M); }
    else if (g.mesh) { kind = 'mesh'; s = shapeOf(kind, (g.scale ?? [1, 1, 1]).map((v) => v / 1000), M); }
    if (s) out.push({ kind, ...s, ...(kind === 'mesh' ? { mesh: base(g.mesh!.split('#')[0]) } : {}), pbr: g.pbr });
  };
  for (const it of root.walk()) {
    if (it instanceof Robot) { const fk = it.fk(); const P = it.poseAbs(); it.chain.links.forEach((l, i) => { const L = multiply(P, fk.linkPoses[i]); for (const v of l.visuals as Array<GeometryRef & { origin: Mat4 }>) add(v, multiply(L, v.origin)); }); continue; }
    const list = (it as { geometry?: GeometryRef[] }).geometry;
    if (!Array.isArray(list)) continue;
    const P = it.poseAbs();
    for (const g of list) add(g, multiply(P, g.origin ? fromArray(g.origin) : identity()));
  }
  return out;
}
function ptsDist(a: { kind: string; pts: Vec3[] }, b: { pts: Vec3[] }): number {
  if (a.pts.length !== b.pts.length) return Infinity;
  const d = (p: Vec3, q: Vec3) => Math.hypot(p[0] - q[0], p[1] - q[1], p[2] - q[2]);
  if (a.kind === 'cone' || a.kind === 'mesh') return Math.max(...a.pts.map((p, i) => d(p, b.pts[i])));
  let worst = 0; const used = new Set<number>();
  for (const p of a.pts) { let best = Infinity, bi = -1; b.pts.forEach((q, i) => { if (!used.has(i)) { const v = d(p, q); if (v < best) { best = v; bi = i; } } }); used.add(bi); worst = Math.max(worst, best); }
  return worst;
}
const hex = (c: number[]) => '#' + c.map((v) => Math.round(Math.max(0, Math.min(1, v)) * 255).toString(16).padStart(2, '0')).join('');

const FIX = path.resolve(__dirname, 'fixtures/webots/gt_appearance');
const AGRO_WS = 'simulation_ws/src/agromarca/murca_garden_ws';
const CASES: Array<{ fx: string; root: string; world: string; metallic: number; emissive: number }> = [
  { fx: 'retail_store_v2', root: 'robot-retailer-murka-v2/ros2_ws/src/murom_webots', world: 'worlds/retail_store.wbt', metallic: 28, emissive: 21 },
  { fx: 'murka_sim_retail_store', root: 'simulation_ws/src/MURKA_Simulation/ros2_ws/src/murka_simulation', world: 'worlds/retail_store.wbt', metallic: 27, emissive: 20 },
  { fx: 'murca_garden', root: AGRO_WS, world: 'src/murca_webots/worlds/murca_garden.wbt', metallic: 12, emissive: 0 },
  { fx: 'murca_apple_garden', root: AGRO_WS, world: 'src/murca_webots/worlds/murca_apple_garden.wbt', metallic: 190, emissive: 0 },
];

describe.skipIf(!HAS_MURKA || !fs.existsSync(MURKA))('MURKA worlds: PBR parameters of every Shape = Webots R2025a', () => {
  for (const c of CASES) {
    it(`${c.fx}: metalness, roughness, emissive colour × intensity of each matched shape as Webots reports them`, () => {
      const fx = JSON.parse(zlib.gunzipSync(fs.readFileSync(path.join(FIX, `${c.fx}.json.gz`))).toString('utf8')) as { shapes: FxShape[] };
      const r = importWebotsWorld(c.world, murkaSource(c.root), { assets: new AssetStore(), loadMeshes: false });
      const act = studioShapes(r.station); const free = new Set(act.map((_, i) => i));
      let matched = 0, metallic = 0, emissive = 0; const bad: string[] = [];
      for (const e of fx.shapes) {
        let best = Infinity, bi = -1;
        for (const i of free) { const a = act[i]; if (a.kind !== e.kind || (e.mesh && a.mesh !== e.mesh)) continue; if (a.sig.length !== e.sig.length || a.sig.some((v, k) => Math.abs(v - e.sig[k]) > (e.kind === 'mesh' ? 1e-3 : 0.5))) continue; const dd = ptsDist(e, a); if (dd < best) { best = dd; bi = i; } }
        if (bi < 0 || best > 0.5) continue;
        free.delete(bi); matched++;
        const p = act[bi].pbr ?? {}; const w = e.app;
        const want = w.type === 'PBRAppearance'
          ? { metalness: w.metalness ?? 1, roughness: w.roughness ?? 0, emissive: (w.emissiveColor ?? [0, 0, 0]).some((v) => v > 0) && (w.emissiveIntensity ?? 1) > 0 ? hex(w.emissiveColor!) : undefined, k: w.emissiveIntensity ?? 1 }
          : { metalness: 0, roughness: 1 - (w.material?.shininess ?? 0.2), emissive: (w.material?.emissiveColor ?? [0, 0, 0]).some((v) => v > 0) ? hex(w.material!.emissiveColor!) : undefined, k: 1 };
        if ((w.metalness ?? 0) >= 0.5) metallic++;
        if (want.emissive) emissive++;
        const ok = Math.abs((p.metalness ?? -1) - want.metalness) < 1e-6 && Math.abs((p.roughness ?? -1) - want.roughness) < 1e-6 && p.emissive === want.emissive && (!want.emissive || Math.abs((p.emissiveIntensity ?? 1) - want.k) < 1e-6);
        if (!ok) bad.push(`${e.label}: webots ${JSON.stringify(want)} studio ${JSON.stringify(p)}`);
      }
      // (the matching itself is tests/webots_gt.test.ts's; a mesh not loaded here still matches by its frame)
      expect(matched / fx.shapes.length, c.fx).toBeGreaterThan(0.95);
      expect(bad.slice(0, 10), c.fx).toEqual([]);
      expect(metallic, c.fx).toBe(c.metallic); expect(emissive, c.fx).toBe(c.emissive);
    }, 300000);
  }
});
