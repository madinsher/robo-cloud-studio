/**
 * P1 semantic layers on imported Webots worlds: YAML subset reader, retail layout loader (products / slots / targets /
 * zones, placed like the MURKA supervisor spawns them), occupancy map rasterised from colliders in a height band, and
 * the orchard field / row overlay built from tree objects.
 */
import { describe, it, expect, beforeAll } from 'vitest';
import fs from 'node:fs';
import path from 'node:path';
import { parseYaml, loadRetailLayout, rasterizeOccupancy, upgradeOrchard, importWebotsWorld, importWebotsFiles, mapSource, WebotsBuildResult } from '../src/io/webots';
import { AssetStore } from '../src/scene/assets';
import { SceneObject, Target, GeometryRef } from '../src/core/items/item';
import { MobileRobot, ZoneItem, MapItem } from '../src/mobile/items';
import { FieldItem, CropRow } from '../src/agri/items';
import { HAS_MURKA, MURKA, RETAIL_V2, dirSource, murkaSource } from './webots_helpers';

const enc = (s: string) => new TextEncoder().encode(s);
const colliders = (it: { [k: string]: unknown }) => (it as { colliders?: GeometryRef[] }).colliders;
const occupied = (m: MapItem, x: number, y: number) => !m.isFree(x, y);

describe('YAML subset', () => {
  it('reads block maps / sequences, flow collections, scalars and comments', () => {
    const v = parseYaml(`# header
frame: map   # inline comment
dock:
  approach: {x: -7.199, y: 4.5, yaw: 3.1416}
  radius: 0.06
list:
  - {name: a, sku: "tea #1", n: [1, 2.5, -3e-1]}
  - name: b
    flag: true
    none: ~
plain: [x, 'y z']
`);
    expect(v).toEqual({
      frame: 'map', dock: { approach: { x: -7.199, y: 4.5, yaw: 3.1416 }, radius: 0.06 },
      list: [{ name: 'a', sku: 'tea #1', n: [1, 2.5, -0.3] }, { name: 'b', flag: true, none: null }], plain: ['x', 'y z'],
    });
    expect(() => parseYaml('a: &anchor 1')).toThrow(/not supported/);
  });
});

describe('retail layout (synthetic)', () => {
  const world = `#VRML_SIM R2025a utf8
IMPORTABLE EXTERNPROTO "../protos/Product.proto"
Robot { name "bot" children [ Shape { geometry Box { size 0.5 0.5 0.5 } } ] }
`;
  const proto = `#VRML_SIM R2025a utf8
PROTO Product [ field SFVec3f translation 0 0 0 field SFRotation rotation 0 0 1 0 field SFString name "p" field SFVec3f size 0.1 0.1 0.1 field SFColor color 1 1 1 ]
{ Solid { translation IS translation rotation IS rotation name IS name children [ Shape { appearance PBRAppearance { baseColor IS color } geometry Box { size IS size } } ] } }`;
  const products = `catalog:\n  tea: {size: [0.07, 0.14, 0.16], color: [0.85, 0.7, 0.2]}\n  big: {size: [0.3, 0.35, 0.28], color: [0.1, 0.2, 0.7], two_hand: true}\ninstances:\n  - {name: tea_1, sku: tea, slot: s0}\n  - {name: big_1, sku: big, slot: pal}\n  - {name: lost, sku: tea, slot: nope}\n`;
  const locations = `frame: map\ntargets:\n  home: {x: 1, y: 2, yaw: 0}\ndock:\n  approach: {x: -2, y: 1, yaw: 3.1416}\n  standoff: {x: -1.25, y: 1, yaw: 3.1416}\n  radius: 0.06\nslots:\n  s0:\n    kind: gondola\n    pose: {x: 0.5, y: 1.0, z: 0.9}\n    approach: {x: 0.6, y: 0.4, yaw: 0}\n    standoff: {x: 0.1, y: 0.2, yaw: 0}\n    arm: left\n    ceiling: 1.2\n  pal:\n    kind: pallet\n    pose: {x: 3, y: 0, z: 0.7}\n    approach: {x: 2.5, y: 0, yaw: 0}\n    arm: both\n  pk:\n    kind: packing\n    pose: {x: -3, y: -2, z: 1}\nbasket_slots:\n  basket_0: {dx: -0.26, dy: 0.12, dz: 0.862, arm: left}\n`;

  it('spawns products through the IMPORTABLE PROTO at slot + size/2 + 2 mm, with targets, zones and basket slots', async () => {
    const files = new Map([['w/worlds/w.wbt', enc(world)], ['w/protos/Product.proto', enc(proto)], ['w/config/products.yaml', enc(products)], ['w/config/locations.yaml', enc(locations)]]);
    const { station, report } = await importWebotsFiles(files, new AssetStore());
    const tea = station.find('tea_1') as SceneObject;
    expect(tea.params.proto).toBe('Product');
    expect(tea.params.sku).toBe('tea');
    expect(Array.from(tea.poseAbs()).slice(12, 15).map((v) => Math.round(v * 1000) / 1000)).toEqual([500, 1000, 900 + 80 + 2]);
    expect(tea.geometry[0].primitive).toEqual({ kind: 'box', size: [70, 140, 160] });
    expect(colliders(tea as never)).toEqual([]);
    expect((station.find('big_1') as SceneObject).params.twoHand).toBe(true);
    expect(report.some((l) => l.includes('lost: slot nope not in locations.yaml'))).toBe(true);
    const s0 = station.find('s0') as Target;
    expect(Array.from(s0.poseAbs()).slice(12, 15)).toEqual([500, 1000, 900]);
    expect(s0.params.arm).toBe('left');
    expect(station.find('s0_approach')).toBeInstanceOf(Target);
    expect(station.find('pal_standoff')).toBeNull();
    const dock = station.find('dock') as ZoneItem;
    expect(dock.kind).toBe('charging');
    expect(dock.contains(-2000 + 50, 1000)).toBe(true);
    expect(dock.contains(-2000 + 70, 1000)).toBe(false);
    expect((station.find('packing') as ZoneItem).contains(-3000, -2000)).toBe(true);
    const bot = station.find('bot') as MobileRobot;
    const basket = bot.children.find((c) => c.name === 'basket_0')!;
    expect(Array.from(basket.pose()).slice(12, 15).map((v) => Math.round(v))).toEqual([-260, 120, 862]);
  });
});

describe('occupancy from colliders (synthetic)', () => {
  it('marks the cells whose vertical band meets a collider; robots, decals and the ground are free', async () => {
    const world = `#VRML_SIM R2025a utf8
Solid { translation 0 0 -0.01 name "floor" children [ Shape { geometry Box { size 10 10 0.02 } } ] boundingObject Box { size 10 10 0.02 } }
Solid { translation 1 0 0.5 name "pillar" boundingObject Cylinder { radius 0.2 height 1 } }
Solid { translation -1 0 1.5 name "high" boundingObject Box { size 1 1 0.4 } }
Solid { translation 0 2 0.0025 name "decal" children [ Shape { geometry Box { size 1 1 0.005 } } ] }
Solid { translation 0 -2 0.3 rotation 0 0 1 0.7854 name "box" boundingObject Box { size 1 0.2 0.6 } }
Robot { translation 2 2 0 name "r" children [ Shape { geometry Box { size 0.5 0.5 0.5 } } ] boundingObject Box { size 0.5 0.5 0.5 } }
`;
    const { station } = await importWebotsFiles(new Map([['w.wbt', enc(world)]]), new AssetStore(), { semantic: false });
    expect(station.find('floor')!.params.ground).toBe(true);
    const m = rasterizeOccupancy(station, { zMin: 20, zMax: 620, resolution: 50 });
    expect(m.originX).toBe(-5000);
    expect(m.width).toBe(200);
    expect(occupied(m, 1000, 0)).toBe(true);
    expect(occupied(m, 1000 + 175, 0)).toBe(true);
    expect(occupied(m, 1000 + 260, 0)).toBe(false);
    expect(occupied(m, -1000, 0)).toBe(false); // above the band
    expect(occupied(m, 0, 2000)).toBe(false); // decal: no collider
    expect(occupied(m, 2000, 2000)).toBe(false); // robot
    expect(occupied(m, 300, -2000 + 300)).toBe(true); // rotated box diagonal
    expect(occupied(m, 300, -2000 - 300)).toBe(false);
    const counts = Array.from(m.cells).filter((c) => c >= 100).length;
    const pillarCells = Math.PI * 200 * 200 / 2500;
    const boxCells = 1000 * 200 / 2500;
    expect(counts).toBeGreaterThan(0.8 * (pillarCells + boxCells));
    expect(counts).toBeLessThan(1.6 * (pillarCells + boxCells));
  });
});

describe.skipIf(!HAS_MURKA)('MURKA retail layout and occupancy', () => {
  let r: WebotsBuildResult;
  const cfg = HAS_MURKA ? path.join(MURKA, 'robot-retailer-murka-v2/ros2_ws/src/murom_world/config') : '';
  beforeAll(() => {
    r = importWebotsWorld('worlds/retail_store.wbt', murkaSource(RETAIL_V2), { assets: new AssetStore(), loadMeshes: false });
  });

  it('products sit in their slots like the supervisor spawns them; gondola slot heights equal the shelf boards of the world', () => {
    const products = fs.readFileSync(path.join(cfg, 'products.yaml'), 'utf8'), locations = fs.readFileSync(path.join(cfg, 'locations.yaml'), 'utf8');
    const res = loadRetailLayout(r.station, products, locations, { builder: r.builder, worldPath: 'worlds/retail_store.wbt' });
    const inst = (parseYaml(products) as { instances: Array<{ name: string; sku: string; slot: string }> }).instances;
    const cat = (parseYaml(products) as { catalog: Record<string, { size: number[] }> }).catalog;
    const slots = (parseYaml(locations) as { slots: Record<string, { kind: string; pose: { x: number; y: number; z: number } }> }).slots;
    expect(res.products).toHaveLength(inst.length);
    for (const i of inst) {
      const p = r.station.find(i.name) as SceneObject;
      expect(p.params.proto, i.name).toBe('RetailProduct');
      expect(colliders(p as never), i.name).toEqual([]);
      const sp = slots[i.slot].pose;
      const want = [sp.x, sp.y, sp.z + cat[i.sku].size[2] / 2 + 0.002].map((v) => Math.round(v * 1e4) / 10);
      expect(Array.from(p.poseAbs()).slice(12, 15).map((v) => Math.round(v * 10) / 10), i.name).toEqual(want);
    }
    // cross-check of two independent files: every gondola slot surface is the top of a board of its shelf
    for (const [id, s] of Object.entries(slots).filter(([, s]) => s.kind === 'gondola')) {
      const shelf = r.station.find(id.replace(/_L\d+_S\d+$/, '')) as SceneObject;
      const tops = shelf.geometry.filter((g) => g.primitive?.kind === 'box' && (g.primitive as { size: number[] }).size[2] === 30).map((g) => g.origin![14] + 15);
      expect(tops.some((t) => Math.abs(t - s.pose.z * 1000) < 1), id).toBe(true);
    }
    const dock = r.station.find('dock') as ZoneItem;
    const murom = r.station.find('murom') as MobileRobot;
    expect(dock.contains(murom.state.x, murom.state.y)).toBe(true); // the robot starts on the dock
    expect(murom.children.filter((c) => c instanceof Target && /^basket_/.test(c.name))).toHaveLength(4);
    expect(res.slots.length).toBe(Object.keys(slots).length);
  });

  it('occupancy at the MUROM height band: shelves, pallets, walls and the charging column block; aisles and the dock pad are free', () => {
    const m = rasterizeOccupancy(r.station, { zMin: 20, zMax: 620, resolution: 50 });
    for (const [x, y] of [[-2000, 1500], [2000, -1500], [6000, 0], [0, 6100], [-8100, 0], [-7620, 4500], [-6000, -4500]]) expect(occupied(m, x, y), `${x},${y}`).toBe(true);
    for (const [x, y] of [[0, 0], [-4000, 0], [-7199, 4500], [-6449, 4500], [4000, 3000]]) expect(occupied(m, x, y), `${x},${y}`).toBe(false);
  });
});

describe.skipIf(!HAS_MURKA)('MURKA orchard overlays', () => {
  it('orchard_demo: 4 sector fields with rows through the trees; occupancy has the trunks and keeps the aisles free', () => {
    const assets = new AssetStore();
    const r = importWebotsWorld('src/murca_webots/worlds/orchard_demo.wbt', dirSource(path.join(MURKA, 'simulation_ws/src/agromarca/murca_garden_ws')), { assets });
    const { fields } = upgradeOrchard(r.station, { assets });
    expect(fields.map((f) => f.name)).toEqual(['sector_1', 'sector_2', 'sector_3', 'sector_4']);
    const s1 = fields[0];
    const rows = s1.rows();
    expect(rows.map((x) => x.plants.length)).toEqual([3, 3, 1]);
    expect(rows[0].start).toEqual([-15000, 5000]);
    expect(rows[0].end).toEqual([-9000, 5000]);
    expect(s1.crop.rowSpacing).toBe(3000);
    expect(s1.crop.plantSpacing).toBe(3000);
    expect(s1.visible).toBe(false);
    expect(rows[0].plants[0].height).toBeGreaterThan(1000);
    expect(r.station.find('apple_tree_s1_02')!.params.row).toBe(rows[0].name);
    const m = rasterizeOccupancy(r.station, { zMin: 20, zMax: 1500, resolution: 50 });
    expect(occupied(m, -15000, 5000)).toBe(true);
    expect(occupied(m, -13500, 5000)).toBe(false);
    expect(occupied(m, -8400, -9000)).toBe(false); // collection point 1: no collider
    expect(occupied(m, -10800, -12500)).toBe(true); // crate_work
    expect(occupied(m, -10800, -11000)).toBe(false); // UTP-G-01 is a robot: not in the map
  }, 60000);

  it('murca_apple_garden: apples of the tree geometry become fruit of the plant records', () => {
    const r = importWebotsWorld('src/murca_webots/worlds/murca_apple_garden.wbt', dirSource(path.join(MURKA, 'simulation_ws/src/agromarca/murca_garden_ws')), { assets: new AssetStore() });
    const { fields } = upgradeOrchard(r.station);
    const plant = fields[0].rows()[0].plants[0];
    expect(plant.fruit).toHaveLength(5);
    expect(plant.fruit[0].p.map((v) => Math.round(v))).toEqual([500, 300, 2800]);
    expect(plant.fruit[0].d).toBeCloseTo(80, 9);
    expect(plant.height).toBeCloseTo(5000, 6); // canopy sphere top: 3.5 + 1.5 m
    expect(fields.every((f) => f instanceof FieldItem && f.rows().every((x) => x instanceof CropRow))).toBe(true);
  });
});

void mapSource;
