/**
 * Control-model `goto` (stationBindings, used by BT / automaton / Petri / FSM executors) plans with planAndFollow (I8):
 * the robot drives around obstacles of the station map with its footprint (AT-W12 of the Webots report: "goto never
 * enters occupied map cells"), with the drive model's timing, and a missing path is a failure, not a straight line.
 */
import { describe, it, expect } from 'vitest';
import { Station } from '../src/core/items/item';
import { MobileRobot, MapItem, ZoneItem } from '../src/mobile/items';
import { stationBindings } from '../src/ctl/runtime';
import { planAndFollow } from '../src/mobile/navigate';
import type { BTNode } from '../src/ctl/bt';

function world(gap: boolean) {
  const st = new Station('goto');
  const map = new MapItem('map'); map.resize(120, 80, 50, -3000, -2000); map.inflation = 0;
  map.fillRect(-100, -2000, 100, gap ? 600 : 2000); // a wall across the room, open above y = 600 when `gap`
  map.fillRect(-3000, -2000, 3000, -1950); map.fillRect(-3000, 1950, 3000, 2000); map.fillRect(-3000, -2000, -2950, 2000); map.fillRect(2950, -2000, 3000, 2000);
  st.addChild(map);
  const robot = new MobileRobot('amr');
  robot.kin = { ...robot.kin, drive: 'differential', footprint: [600, 500, 400], maxSpeed: 800, maxReverseSpeed: 300, maxAccel: 600, maxYawRate: 90, track: 450, wheelBase: 0 };
  st.addChild(robot); robot.state.x = -2000; robot.state.y = -1000; robot.state.theta = 0; robot.syncPoseFromState();
  const zone = new ZoneItem('goal'); zone.polygon = [[1700, -1300], [2300, -1300], [2300, -700], [1700, -700]]; st.addChild(zone);
  return { st, map, robot, zone };
}

describe('ctl goto via planAndFollow (I8)', () => {
  it('drives around the wall through the gap: footprint centre never in an occupied cell, ends in the zone, duration of the simulated drive', () => {
    const { st, map, robot, zone } = world(true);
    const plan = planAndFollow(st, robot, { x: 2000, y: -1000 });
    expect(plan.ok).toBe(true);
    const b = stationBindings({ robot, station: st });
    const node = { id: 'n1', status: 'idle' } as unknown as BTNode;
    let t = 0, status = 'running', maxY = -Infinity;
    while (status === 'running' && t < 300) {
      status = b.actions.goto({ time: t, dt: 0.05, bb: {} }, { zone: 'goal' }, node) ?? 'success';
      (node as unknown as { status: string }).status = status;
      b.stepRobot(0.05); t += 0.05;
      expect(map.isFree(robot.state.x, robot.state.y), `${robot.state.x},${robot.state.y}`).toBe(true);
      maxY = Math.max(maxY, robot.state.y);
    }
    expect(status).toBe('success');
    expect(zone.contains(robot.state.x, robot.state.y)).toBe(true);
    expect(maxY).toBeGreaterThan(600); // went through the gap, not through the wall
    expect(t).toBeGreaterThan(plan.duration - 0.2); expect(t).toBeLessThan(plan.duration + 0.5);
  });

  it('no path (closed wall): goto fails with a reason instead of driving through the wall', () => {
    const { st, robot } = world(false);
    const b = stationBindings({ robot, station: st });
    const bb: Record<string, unknown> = {};
    const s = b.actions.goto({ time: 0, dt: 0.05, bb }, { zone: 'goal' }, { id: 'n2', status: 'idle' } as unknown as BTNode);
    expect(s).toBe('failure');
    expect(String(bb.error)).toMatch(/no collision-free path to goal/);
    expect(robot.state.x).toBe(-2000);
  });
});
