/**
 * F1 (V7 finding): the collision geometry a source declares separately from the visuals (Webots boundingObject, URDF
 * <collision>) is what collision checks use, and it survives undo snapshots and .vbstation save / open.
 *  - robot links with `collisions` collide with them (not with their visual meshes); for a robot built from a
 *    kinematic tree (Webots / URDF) a link without one does not collide (Webots: no boundingObject = no collision;
 *    the transparent UTP-G "ghost" arm);
 *  - MobileRobot.colliders (the base's boundingObject) and LinkDef.collisions are serialized.
 */
import { describe, it, expect } from 'vitest';
import { Station } from '../src/core/items/item';
import { Robot } from '../src/core/items/robot';
import { MobileRobot } from '../src/mobile/items';
import { createRobotFromLibrary } from '../src/core/items/library';
import { collidersOf } from '../src/core/collision/collision';
import { importWebotsWorld } from '../src/io/webots';
import { AssetStore } from '../src/scene/assets';
import { saveStation, loadStation } from '../src/io/station-file';
import { HAS_MURKA, murkaSource } from './webots_helpers';

const box = (s: number) => ({ primitive: { kind: 'box' as const, size: [s, s, s] as [number, number, number] }, origin: [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1] });
const roundTrip = (st: Station) => Station.deserialize(JSON.parse(JSON.stringify(st.serialize())));

describe('declared collision geometry', () => {
  it('link collisions replace the visual meshes and survive a snapshot; RoboDK-style robots keep their meshes', () => {
    const st = new Station('s');
    const r = createRobotFromLibrary('UR10e');
    st.addChild(r);
    const meshBased = collidersOf(r).length;
    expect(meshBased).toBeGreaterThan(0);
    r.chain.links[2].collisions = [box(80)];
    const cols = collidersOf(r);
    expect(cols.length).toBe(1);                                    // only the declared box; no visual-mesh colliders
    expect(cols[0].shape.kind).toBe('obb');
    expect(cols[0].linkIndex).toBe(2);
    const r2 = roundTrip(st).itemsOfType<Robot>(2)[0];
    expect(r2.chain.links[2].collisions?.length).toBe(1);
    expect(collidersOf(r2).length).toBe(1);
  });

  it('MobileRobot boundingObject colliders survive a snapshot (undo) instead of falling back to the visual meshes', () => {
    const st = new Station('s');
    const m = new MobileRobot('base');
    st.addChild(m);
    m.geometry = [box(500), box(300), box(200)];
    m.colliders = [box(600)];
    expect(collidersOf(m).length).toBe(1);
    const m2 = roundTrip(st).children.find((c) => c.name === 'base') as MobileRobot;
    expect(m2.colliders?.length).toBe(1);
    expect(collidersOf(m2).length).toBe(1);
  });

  it.skipIf(!HAS_MURKA)('MURKA retail v2 MUROM and orchard UTP-G-01: boundingObjects after import, undo snapshot and .vbstation save / open', () => {
    const cases = [
      { root: 'robot-retailer-murka-v2/ros2_ws/src/murom_webots', world: 'worlds/retail_store.wbt', mobile: 'murom', base: 6, arms: { murom_head: 2, murom_left_arm: 4, murom_right_arm: 4 } as Record<string, number> },
      { root: 'simulation_ws/src/agromarca/murca_garden_ws', world: 'src/murca_webots/worlds/orchard_demo.wbt', mobile: 'UTP-G-01', base: 11, arms: { 'UTP-G-01_arm': 0 } as Record<string, number> },
    ];
    for (const c of cases) {
      const assets = new AssetStore();
      const { station } = importWebotsWorld(c.world, murkaSource(c.root), { assets });
      const check = (st: Station, label: string) => {
        const m = st.find(c.mobile) as MobileRobot;
        expect(collidersOf(m, { assets }).length, `${label} ${c.mobile} base`).toBe(c.base);
        for (const a of [...m.walk()].filter((i): i is Robot => i instanceof Robot)) expect(collidersOf(a, { assets }).length, `${label} ${a.name}`).toBe(c.arms[a.name]);
      };
      check(station, 'import');
      check(roundTrip(station), 'snapshot');
      check(loadStation(JSON.parse(JSON.stringify(saveStation(station, assets))), new AssetStore()), 'file');
    }
  }, 600000);
});
