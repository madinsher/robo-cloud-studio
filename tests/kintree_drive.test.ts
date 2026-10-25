/**
 * Drive kinematics detected by the kinematic-tree decomposition:
 *  - the wheel centre is on the wheel axis at the wheel geometry (a Webots HingeJoint anchor is only a point of the
 *    axis: E-puck has anchor 0 0 0.02 for both wheels and the ±26 mm offset in the endPoint);
 *  - wheel base / track are measured along the axes the wheels drive (the MURKA UTP-G URDF drives along -Y);
 *  - wheels on a mecanum contact material (Webots youBot) make a holonomic base.
 */
import { describe, it, expect } from 'vitest';
import { buildStationItemsFromTree, KinTree, KinJoint, KinLink } from '../src/io/kintree';
import { importWebotsWorld } from '../src/io/webots';
import { AssetStore } from '../src/scene/assets';
import { MobileRobot } from '../src/mobile/items';
import { Mat4, transl, rotx, roty, DEG } from '../src/core/math/pose';
import { dirSource, webotsCacheFetcher, WEBOTS_HOME, HAS_WEBOTS_CACHE } from './webots_helpers';

const link = (name: string, extra: Partial<KinLink> = {}): KinLink => ({ name, visuals: [{ primitive: { kind: 'box', size: [10, 10, 10] } }], ...extra });
const cont = (name: string, parent: string, child: string, origin: Mat4, axis: [number, number, number]): KinJoint => ({ name, type: 'continuous', parent, child, origin, axis, velocity: 600 });
const rev = (name: string, parent: string, child: string, origin: Mat4, axis: [number, number, number], lo: number, hi: number): KinJoint => ({ name, type: 'revolute', parent, child, origin, axis, lower: lo, upper: hi });
/** Wheel link: a cylinder about `axis` (y or x) centred at `at` in the link frame. */
const wheel = (name: string, r: number, axis: 'x' | 'y', at: [number, number, number] = [0, 0, 0]) => link(name, { visuals: [], collisions: [{ primitive: { kind: 'cylinder', radius: r, length: 10 }, origin: Array.from(((m) => { m[12] = at[0]; m[13] = at[1]; m[14] = at[2]; return m; })(axis === 'y' ? rotx(Math.PI / 2) : roty(Math.PI / 2))) }] });

describe('drive kinematics from the kinematic tree', () => {
  it('Webots-style anchors on the robot axis: the track comes from the wheel geometry along the axis (e-puck 52 mm)', () => {
    const tree: KinTree = {
      name: 'epuck', root: 'body', links: [link('body'), wheel('left wheel', 20, 'y', [0, 26, 0]), wheel('right wheel', 20, 'y', [0, -26, 0])],
      joints: [cont('left wheel motor', 'body', 'left wheel', transl(0, 0, 20), [0, 1, 0]), cont('right wheel motor', 'body', 'right wheel', transl(0, 0, 20), [0, 1, 0])],
    };
    const m = buildStationItemsFromTree(tree).mobile!;
    expect(m.kin.drive).toBe('differential'); expect(m.kin.track).toBeCloseTo(52, 6); expect(m.kin.wheelRadius).toBeCloseTo(20, 6);
  });
  it('Ackermann base driving along -Y of its base frame: wheel base and track along the driving axes (UTP-G 2190 / 1840 mm)', () => {
    const steerJ = (side: 'l' | 'r', x: number) => rev(`front_${side}_steering_joint`, 'base', `front_${side}_knuckle`, transl(x, -2540, 0), [0, 0, 1], -17.19, 17.19);
    const tree: KinTree = {
      name: 'utp_g', root: 'base', links: [link('base'), link('front_l_knuckle', { visuals: [] }), link('front_r_knuckle', { visuals: [] }), ...['fl', 'fr', 'bl', 'br'].map((n) => wheel(`${n}_wheel`, 339, 'x'))],
      joints: [steerJ('l', -920), steerJ('r', 920),
        cont('fl_wheel_joint', 'front_l_knuckle', 'fl_wheel', transl(0, 0, 0), [1, 0, 0]), cont('fr_wheel_joint', 'front_r_knuckle', 'fr_wheel', transl(0, 0, 0), [1, 0, 0]),
        cont('bl_wheel_joint', 'base', 'bl_wheel', transl(-920, -350, 0), [1, 0, 0]), cont('br_wheel_joint', 'base', 'br_wheel', transl(920, -350, 0), [1, 0, 0])],
    };
    const r = buildStationItemsFromTree(tree); const m = r.mobile!;
    expect(m.kin.drive).toBe('ackermann');
    expect(m.kin.wheelBase).toBeCloseTo(2190, 6); expect(m.kin.track).toBeCloseTo(1840, 6);
    expect(m.kin.minTurnRadius).toBeCloseTo(2190 / Math.tan(17.19 * DEG), 3);
    expect(m.kin.maxYawRate!).toBeLessThan(400);
    expect(m.params.forwardAxis).toEqual([0, -1, 0]);
    expect(r.report.some((l) => /drive along \(0\.00, -1\.00\)/.test(l))).toBe(true);
  });
  it('a standard +X differential robot is unchanged (no forward-axis note)', () => {
    const tree: KinTree = { name: 'tb', root: 'b', links: [link('b'), wheel('wl', 33, 'y'), wheel('wr', 33, 'y')], joints: [cont('wl_j', 'b', 'wl', transl(0, 144, 33), [0, 1, 0]), cont('wr_j', 'b', 'wr', transl(0, -144, 33), [0, 1, 0])] };
    const r = buildStationItemsFromTree(tree);
    expect(r.mobile!.kin.track).toBeCloseTo(288, 6); expect(r.mobile!.params.forwardAxis).toEqual([1, 0, 0]);
    expect(r.report.some((l) => /drive along/.test(l))).toBe(false);
  });
});

describe.skipIf(!HAS_WEBOTS_CACHE)('Webots sample robots (local Webots cache)', () => {
  const kin = (world: string) => { const r = importWebotsWorld(world, dirSource(`${WEBOTS_HOME}/projects`), { assets: new AssetStore(), remote: webotsCacheFetcher() }); return [...r.station.walk()].filter((i): i is MobileRobot => i instanceof MobileRobot).map((m) => m.kin); };
  it('e-puck: differential, track 52 mm, wheel radius 20 mm', () => {
    const [k] = kin('robots/gctronic/e-puck/worlds/e-puck.wbt');
    expect(k.drive).toBe('differential'); expect(k.track).toBeCloseTo(52, 1); expect(k.wheelRadius).toBeCloseTo(20, 1);
  }, 120000);
  it('KUKA youBot: mecanum wheels (anisotropic contact materials) -> holonomic base', () => {
    const [k] = kin('robots/kuka/youbot/worlds/youbot.wbt');
    expect(k.drive).toBe('omni'); expect(k.minTurnRadius).toBe(0); expect(k.maxLateralSpeed).toBeGreaterThan(0);
  }, 120000);
});
