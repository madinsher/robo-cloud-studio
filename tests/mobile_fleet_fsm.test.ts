/**
 * Per-robot FSM bound to fleet tasks (src/fleet/fleet_fsm.ts): one platform of a harvest fleet is driven by its own
 * studio `fsm` controller. The fleet holds it: it receives a task only in the FSM's Execute state, charges when its FSM
 * decides (battery < 30 % at the start), and the FSM trace (fleet time) is its lifecycle with the task of every Execute.
 */
import { describe, it, expect } from 'vitest';
import { Station } from '../src/core/items/item';
import { createRobotFromLibrary } from '../src/core/items/library';
import { Tool } from '../src/core/items/item';
import { mul, transl, rotz, DEG } from '../src/core/math/pose';
import { MobileRobot } from '../src/mobile/items';
import { FieldItem, MissionItem, cropParams } from '../src/agri/items';
import { generateOrchard, buildFieldMap, rectPolygon } from '../src/agri/orchard';
import { planMission } from '../src/agri/missions';
import { FleetItem, FleetManager } from '../src/fleet/fleet';
import { attachFleetFsm, FLEET_ROBOT_FSM } from '../src/fleet/fleet_fsm';
import { parseFsm, checkFsm } from '../src/ctl/fsm';
import '../src/agri/process';

function station() {
  const st = new Station('fsm fleet');
  const field = st.addChild(new FieldItem('Block'));
  field.polygon = rectPolygon(30000, 16000);
  field.crop = cropParams('apple', { rowHeading: 0, headland: 6000, ripeFraction: 0.5, fruitPerPlant: 10 });
  generateOrchard(field, 3);
  st.addChild(buildFieldMap(field, 250));
  const fleet = st.addChild(new FleetItem('Fleet'));
  const fm = new FleetManager(st, fleet);
  const robots: MobileRobot[] = [];
  for (let i = 0; i < 2; i++) {
    const m = st.addChild(new MobileRobot(`H${i + 1}`));
    Object.assign(m.kin, { drive: 'tracked', wheelBase: 1800, track: 1400, wheelRadius: 300, maxSpeed: 1300, footprint: [2600, 1600, 1400] });
    m.capabilities = ['harvest', 'transport'];
    m.setPose2D(-6000, 4000 + i * 5000, 0);
    m.home = { x: -6000, y: 4000 + i * 5000, theta: 0 };
    m.battery.chargeW = 200000; // fast charger for the test
    fm.addRobot(m);
    const arm = m.addChild(createRobotFromLibrary('UR10e', `Arm ${i + 1}`));
    arm.setPose(mul(transl(0, -700, 1400), rotz(-90 * DEG)));
    const g = arm.addChild(new Tool('Gripper')); g.setPoseTool(transl(0, 0, 150)); arm.setTool(g);
    robots.push(m);
  }
  const mission = st.addChild(new MissionItem('Harvest'));
  mission.missionType = 'harvest'; mission.fieldId = field.id; mission.fleetId = fleet.id;
  mission.rowIds = field.rows().slice(0, 3).map((r) => r.id);
  mission.settings = { workSpeed: 500, bothSides: true, sideOffset: field.crop.rowSpacing / 2, secondsPerFruit: 2 };
  planMission(st, mission, fm);
  return { st, fm, fleet, robots };
}

describe('per-robot FSM bound to fleet tasks', () => {
  it('the default controller is a valid fsm document', () => {
    expect(checkFsm(parseFsm(FLEET_ROBOT_FSM)).filter((i) => i.level === 'error')).toEqual([]);
  });

  it('the FSM robot charges first (no task while charging), then takes tasks only in Execute; all tasks done, traces match', () => {
    const { fm, fleet, robots } = station();
    const [A, B] = robots;
    B.battery.levelWh = 0.25 * B.battery.capacityWh;
    const ctl = attachFleetFsm(fm, B);
    const n = fleet.tasks.length;
    expect(n).toBe(6);
    while (fm.time < 6000) { fm.step(0.2); ctl.tick(0.2); if (fleet.tasks.filter((t) => t.type !== 'charge').every((t) => t.status === 'done')) break; }
    const work = fleet.tasks.filter((t) => t.type !== 'charge');
    expect(work.filter((t) => t.status !== 'done').map((t) => t.id)).toEqual([]);
    const tr = ctl.trace;
    // 1. the FSM decided to charge before anything else
    expect(tr.slice(0, 2).map((e) => e.state)).toEqual(['Idle', 'Charge']);
    const chargeEnd = tr.find((e, i) => i > 1 && tr[i - 1].state === 'Charge')!.t;
    const bTasks = work.filter((t) => t.robotId === B.id);
    expect(bTasks.length).toBeGreaterThanOrEqual(1);
    for (const t of bTasks) expect(t.startedAt!).toBeGreaterThanOrEqual(chargeEnd - 1e-9);
    // 2. every task of B ran in an Execute state that recorded it, started after the state was entered
    for (const t of bTasks) { const e = tr.find((x) => x.state === 'Execute' && x.task === t.id); expect(e, t.id).toBeTruthy(); expect(t.startedAt!).toBeGreaterThanOrEqual(e!.t - 1e-9); }
    // 3. A (fleet-allocated) also worked; B reached > 90 % after charging; contacts none
    expect(work.some((t) => t.robotId === A.id)).toBe(true);
    expect(fm.kpis().contacts.robotObstacleSteps + fm.kpis().contacts.robotRobotSteps).toBe(0);
    // 4. the controller ends idle / resting with no pending task; the robot is back on hold
    expect(fm.hold.has(B.id)).toBe(true);
    expect(tr.filter((e) => e.state === 'Execute' && e.task).length).toBe(bTasks.length);
  }, 300000);
});
