/**
 * MURKA retail — the single tasks of /murom/task/mission (undock, goto, home, dock, scan, depalletize, order) and the
 * cancel semantics of cancel_test.py: a cancel during the approach of a pick leaves the hand empty and the arm back in
 * the travel pose; a cancel during a place keeps the product in the hand; the next task that needs that hand first puts
 * it into the basket (RELEASE_HELD) and then completes.
 */
import { describe, it, expect, beforeAll } from 'vitest';
import { buildMurkaRetailStation, runMurkaRetailMission, createRetailSession, RetailStationResult } from '../src/murka/retail';
import type { RetailSession } from '../src/murka/retail';
import { parkingErrors } from '../src/murka/retail/nav';
import { AssetStore } from '../src/scene/assets';
import { HAS_MURKA } from './murka_retail_helpers';

describe.skipIf(!HAS_MURKA)('MURKA retail single tasks (v2)', () => {
  let b: RetailStationResult; let s: RetailSession;
  beforeAll(async () => { const assets = new AssetStore(); b = await buildMurkaRetailStation({ variant: 'v2', assets, loadMeshes: true }); s = createRetailSession(b, { assets }); }, 300000);
  const run = (mission: string, args: string[] = [], extra: Record<string, unknown> = {}) => runMurkaRetailMission(b, { mission, args, session: s, ...extra }).done;

  it('undock: reverse out of the charger onto the dock standoff line, no fine alignment (pose_coarse)', async () => {
    const r = await run('undock');
    expect(r.outcome).toBe('done');
    const ev = s.nav.events.slice(-2).map((e) => e.kind);
    expect(ev[0]).toBe('undock');
    const [eLong, eLat] = parkingErrors(s.sim.pose(), [-6.449, 4.5, Math.PI]);
    expect(Math.abs(eLat)).toBeLessThan(0.06); expect(eLong).toBeLessThanOrEqual(0.1);
    expect(s.sim.battery.docked).toBe(false);
  }, 300000);

  it('goto <slot>: Nav2 to the standoff then glide-docking to the approach of the slot arm, ≤ 2 cm / 0.03 rad', async () => {
    const r = await run('goto', ['shelf_A2_L0_S1']);
    expect(r.outcome).toBe('done');
    const a = b.layout.slots.shelf_A2_L0_S1.approach!;
    const [eLong, eLat, eYaw] = parkingErrors(s.sim.pose(), [a.x, a.y, a.yaw]);
    expect(Math.abs(eLong)).toBeLessThanOrEqual(0.021); expect(Math.abs(eLat)).toBeLessThanOrEqual(0.031); expect(Math.abs(eYaw)).toBeLessThanOrEqual(0.03);
    expect(s.nav.events.slice(-2).map((e) => e.kind)).toEqual(['coarse', 'glide']);
  }, 300000);

  it('home: leaves the tight parking backwards first, then drives home', async () => {
    const n0 = s.nav.events.length;
    const r = await run('home');
    expect(r.outcome).toBe('done');
    expect(s.nav.events.slice(n0).map((e) => e.kind)).toEqual(['undock', 'coarse', 'glide']);
    const [x, y] = s.sim.pose(); expect(Math.hypot(x + 5.5, y - 3)).toBeLessThan(0.03);
  }, 300000);

  it('scan shelf_A2: inventory of its visible slots only, all correct', async () => {
    const r = await run('scan', ['shelf_A2']);
    expect(r.outcome).toBe('done');
    const a2 = Object.keys(r.inventory).filter((k) => k.startsWith('shelf_A2'));
    expect(a2).toHaveLength(8);
    expect(r.checks.scan!.correct).toBe(r.checks.scan!.visible);
  }, 300000);

  it('order tea_box + coffee_can (e2e_test.py): both on the packing table, robot back home', async () => {
    const r = await run('order', ['tea_box', 'coffee_can']);
    expect(r.outcome, r.report.join('\n')).toBe('done');
    expect(r.checks.order!.ok).toBe(true);
    const [x, y] = s.sim.pose(); expect(Math.hypot(x + 5.5, y - 3)).toBeLessThan(0.03); // NAV_HOME of the single task
    expect(Object.values(r.registry).filter((o) => o.location.startsWith('carried'))).toEqual([]);
  }, 600000);

  it('dock: glide-dock to the charger (≤ 6 cm, ≤ 3°) and charge', async () => {
    const r = await run('dock');
    expect(r.outcome).toBe('done');
    expect(r.checks.docked!.ok).toBe(true);
    expect(s.sim.battery.docked).toBe(true);
  }, 300000);
});

describe.skipIf(!HAS_MURKA)('MURKA retail cancel semantics (cancel_test.py, fresh store, v2)', () => {
  let b: RetailStationResult; let s: RetailSession;
  beforeAll(async () => { const assets = new AssetStore(); b = await buildMurkaRetailStation({ variant: 'v2', assets, loadMeshes: true }); s = createRetailSession(b, { assets }); }, 300000);
  const run = (mission: string, args: string[] = [], extra: Record<string, unknown> = {}) => runMurkaRetailMission(b, { mission, args, session: s, ...extra }).done;

  it('cancel during the pick approach: canceled, hand empty, arm retreats to the travel pose and stops; product untouched', async () => {
    let tPick = -1, cancel: () => void = () => undefined;
    const h = runMurkaRetailMission(b, { mission: 'order', args: ['tea_box'], session: s, onStep: (t, st) => { if (st === 'PICK_ITEM' && tPick < 0) tPick = t; if (tPick >= 0 && t > tPick + 2 && tPick !== -2) { tPick = -2; cancel(); } } });
    cancel = h.cancel;
    const r = await h.done;
    expect(r.outcome).toBe('canceled');
    expect(Object.values(r.registry).filter((o) => o.location.startsWith('carried'))).toEqual([]);
    // at the result the arm is back in the travel pose (arms_for_view tolerance 0.1 rad) and settles there
    for (const [i, v] of s.sim.armState('left')[0].entries()) expect(Math.abs(v - [0.3, 0, 0, -2.2][i])).toBeLessThan(0.1);
    for (let k = 0; k < 60; k++) s.sim.step(0.05); // the last travel-pose trajectory (2 s) has run out
    expect(s.sim.trajActive()).toBe(false);
    for (const [i, v] of s.sim.armState('left')[0].entries()) expect(Math.abs(v - [0.3, 0, 0, -2.2][i])).toBeLessThan(1e-6);
    expect(s.manip.log.some((l) => /PICK отменён/.test(l))).toBe(true);
    const tea = r.registry[Object.keys(r.registry).find((n) => n.startsWith('tea_box') && r.registry[n].location.startsWith('shelf_'))!];
    expect(tea.location).toMatch(/^shelf_/);
  }, 600000);

  it('cancel during PLACE_BASKET keeps the product in the hand; the next order puts it into the basket first (RELEASE_HELD) and completes', async () => {
    let tPlace = -1, cancel: () => void = () => undefined;
    const h = runMurkaRetailMission(b, { mission: 'order', args: ['juice_pack'], session: s, onStep: (t, st) => { if (st === 'PLACE_BASKET' && tPlace < 0) tPlace = t; if (tPlace >= 0 && t > tPlace + 1 && tPlace !== -2) { tPlace = -2; cancel(); } } });
    cancel = h.cancel;
    const r = await h.done;
    expect(r.outcome).toBe('canceled');
    const carried = Object.entries(r.registry).filter(([, o]) => o.location === 'carried_left').map(([n]) => n);
    expect(carried).toHaveLength(1); expect(carried[0]).toMatch(/^juice_pack/);
    expect(r.detail).toMatch(/в руках остался товар: рука left/);
    for (let k = 0; k < 60; k++) s.sim.step(0.05); // the last travel-pose trajectory (2 s) has run out
    expect(s.sim.trajActive()).toBe(false);
    const r2 = await run('order', ['tea_box']);
    expect(r2.outcome, r2.report.join('\n')).toBe('done');
    expect(r2.progress.map((e) => e.state)).toContain('RELEASE_HELD');
    expect(Object.values(r2.registry).filter((o) => o.location.startsWith('carried'))).toEqual([]);
    expect(r2.registry[carried[0]].location).toMatch(/^packing_S/); // released into the basket, then unloaded with the order
  }, 900000);

});

describe.skipIf(!HAS_MURKA)('MURKA retail depalletize task (v2)', () => {
  it('depalletize pallet_1 box_red bulk_B1 2: two boxes, top layer west first, bulk L0 slots in order; with the carry fix no contact at all', async () => {
    const assets = new AssetStore();
    const b = await buildMurkaRetailStation({ variant: 'v2', assets, loadMeshes: true });
    const r = await runMurkaRetailMission(b, { mission: 'depalletize', args: ['pallet_1', 'box_red', 'bulk_B1', '2'], assets, fixes: { bothCarryZ: 1.46 } }).done;
    expect(r.outcome, r.report.join('\n')).toBe('done');
    expect(r.checks.depal!.moved).toEqual(['box_red_7 -> bulk_B1_L0_S0', 'box_red_8 -> bulk_B1_L0_S1']);
    expect(r.checks.collisions).toEqual({ armLinks: 0, hands: 0, loads: 0, base: 0, ok: true });
    const [x, y] = r.registry.box_red_7.pos; expect(Math.hypot(x - b.layout.slots.bulk_B1_L0_S0.pose.x, y - b.layout.slots.bulk_B1_L0_S0.pose.y)).toBeLessThan(0.02);
  }, 900000);
});

describe.skipIf(!HAS_MURKA)('MURKA mission FSM as a station control model (Control tab path)', () => {
  it('the station carries the mission fsm model; an edited document (stage parameters) runs through stationFsmExecutor on the station session', async () => {
    const { stationFsmExecutor } = await import('../src/ctl/world');
    const { ControlModelItem } = await import('../src/ctl/model');
    const { ItemType } = await import('../src/core/items/item');
    const b = await buildMurkaRetailStation({ variant: 'v2', assets: new AssetStore(), loadMeshes: false });
    const cm = b.station.itemsOfType<InstanceType<typeof ControlModelItem>>(ItemType.CONTROL_MODEL).find((m) => m.name === 'MUROM mission (full)')!;
    expect(cm.kind).toBe('fsm');
    expect(cm.source).toMatch(/do skill=murom\.phase phase=SCAN scan=shelf_A1,shelf_A2,shelf_A3 outcomes=next/);
    const edited = cm.source.replace('scan=shelf_A1,shelf_A2,shelf_A3', 'scan=shelf_A3').replace('depal=pallet_2,box_blue,bulk_B3,1', 'depal=pallet_3,box_green,bulk_B2,1').replace('order=tea_box,juice_pack', 'order=snack_box');
    const own = stationFsmExecutor(b.station, edited, () => undefined)!;
    expect(own).not.toBeNull();
    const ex = own.exec;
    for (let i = 0; i < 40000 && ex.status === 'running'; i++) ex.tick(0.05); // the app's world clock does this
    expect(ex.outcome).toBe('done');
    const s = (await import('../src/murka/retail/mission')).retailSessionOf(b);
    expect([...s.orch.inventory.keys()].every((k) => k.startsWith('shelf_A3_'))).toBe(true);
    const reg = s.sim.registry();
    expect(Object.entries(reg).filter(([, o]) => o.sku === 'box_green' && o.location.startsWith('bulk_B2_L0')).length).toBe(1);
    expect(Object.values(reg).filter((o) => o.sku === 'snack_box' && o.location.startsWith('packing_')).length).toBe(1);
    expect(s.sim.battery.docked).toBe(true);
    // a generic fsm (no murom.* skills) is not taken over
    expect(stationFsmExecutor(b.station, 'fsm X\nstate A\n  do wait=1\n  ok -> A', () => undefined)).toBeNull();
  }, 900000);
});
