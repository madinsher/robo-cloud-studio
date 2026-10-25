// P1 (MURKA agromarca): modular equipment, recipes, depot reconfiguration during a mission, configuration-dependent
// energy, aerial platforms flying over ground traffic.
import { describe, it, expect } from 'vitest';
import { Station, ItemType } from '../src/core/items/item';
import { MobileRobot, ZoneItem } from '../src/mobile/items';
import { FieldItem, MissionItem, cropParams } from '../src/agri/items';
import { generateOrchard, buildFieldMap, rectPolygon } from '../src/agri/orchard';
import { planMission } from '../src/agri/missions';
import { infectPlants } from '../src/agri/process';
import { applyEquipment, recipeFor, RECIPES, EQUIPMENT, actionEnergy } from '../src/agri/equipment';
import { powerDraw } from '../src/mobile/controller';
import { FleetItem, FleetManager } from '../src/fleet/fleet';

function ground(st: Station, name: string, x: number, y: number) {
  const m = st.addChild(new MobileRobot(name));
  Object.assign(m.kin, { drive: 'tracked', wheelBase: 1600, track: 1300, maxSpeed: 1300, footprint: [2400, 1500, 1400] });
  m.capabilities = ['transport'];
  m.payloadKg = 300;
  m.setPose2D(x, y, 0);
  m.home = { x, y, theta: 0 };
  return m;
}

describe('modular equipment (P1)', () => {
  it('mounting a recipe sets capabilities, resources, module power and the MURKA drive-cost factor; invalid sets are rejected', () => {
    const st = new Station('eq');
    const r = ground(st, 'UTP-G-01', 0, 0);
    applyEquipment(r, RECIPES.sprayer);
    expect(r.capabilities.sort()).toEqual(['spray', 'transport']);
    expect(r.resources.tank).toEqual({ capacity: 400, level: 400, unit: 'L' });
    expect(r.battery.equipmentW).toBe(EQUIPMENT.sprayer.powerW);
    expect(r.battery.driveFactor).toBeCloseTo(1 + 120 / 300, 9);
    // the same platform drives more expensively with the heavy sprayer than with the scouting mast
    r.state.v = 1000;
    const pSpray = powerDraw(r);
    applyEquipment(r, RECIPES.scout);
    const pScout = powerDraw(r);
    expect(r.capabilities.sort()).toEqual(['scout', 'transport']);
    expect(r.resources.tank).toBeUndefined();
    expect(pSpray - pScout).toBeCloseTo((350 - 60) + r.battery.drivePerMps * ((1 + 120 / 300) - (1 + 12 / 300)), 6);
    expect(() => applyEquipment(r, ['sprayer', 'scout_mast'])).toThrow(/top slot/);
    const uav = st.addChild(new MobileRobot('UTP-A-01')); uav.kin.drive = 'aerial';
    expect(() => applyEquipment(uav, ['sprayer'])).toThrow(/does not fit/);
    applyEquipment(uav, RECIPES.uav_sprayer);
    expect(uav.capabilities).toContain('spray');
    expect(recipeFor(['spray'], false)).toBe('sprayer');
    expect(recipeFor(['spray'], true)).toBe('uav_sprayer');
    expect(recipeFor(['harvest'], false)).toBe('harvester');
    // action energy: 10 litres with the UAV mini sprayer
    const e0 = uav.battery.levelWh;
    expect(actionEnergy(null, uav, 'litre', 10)).toBeCloseTo(0.5, 9);
    expect(e0 - uav.battery.levelWh).toBeCloseTo(0.5, 9);
  });

  it('scouting finds a disease, no robot can spray: an idle robot is reconfigured at the depot (180 s, 2 slots) and sprays', () => {
    const st = new Station('garden');
    const field = st.addChild(new FieldItem('Sector 1'));
    field.polygon = rectPolygon(30000, 11500);
    field.crop = cropParams('apple', { rowHeading: 0, headland: 4000 });
    generateOrchard(field, 9);
    st.addChild(buildFieldMap(field, 250));
    const depot = st.addChild(new ZoneItem('Equipment depot')); depot.kind = 'depot'; depot.capacity = 2; depot.polygon = rectPolygon(3000, 3000, -8000, 1000);
    const fleet = st.addChild(new FleetItem('MURKA fleet'));
    const fm = new FleetManager(st, fleet);
    const scout = ground(st, 'UTP-G-01', -3000, 2000); applyEquipment(scout, RECIPES.scout); fm.addRobot(scout);
    const util = ground(st, 'UTP-G-02', -3000, 6000); applyEquipment(util, RECIPES.harvester); fm.addRobot(util);
    expect(fm.robots().some((r) => r.capabilities.includes('spray'))).toBe(false);
    infectPlants(st, [15000, 4000], 2500, 0.6);
    const m = st.addChild(new MissionItem('Scout sector 1'));
    m.missionType = 'scout'; m.fieldId = field.id; m.fleetId = fleet.id; m.rowIds = field.rows().map((r) => r.id);
    m.settings = { workSpeed: 800, followUp: 'spray', diseaseThreshold: 0.6 };
    planMission(st, m, fm);
    const e0 = { scout: scout.battery.levelWh, util: util.battery.levelWh };
    for (let i = 0; fm.time < 6000; i++) {
      fm.step(0.1);
      if (i % 50 === 0 && fleet.tasks.length > 2 && fleet.tasks.every((t) => t.status === 'done')) break;
    }
    const follow = fleet.tasks.filter((t) => t.meta?.followUpOf);
    expect(follow.length).toBeGreaterThan(0);
    expect(follow.every((t) => t.status === 'done')).toBe(true);
    const k = fm.kpis();
    expect(k.process.equipment.reconfigurations).toBe(1);
    // the idle harvester became the sprayer (the scout keeps scouting)
    const sprayer = st.findById(follow[0].robotId!) as MobileRobot;
    expect(sprayer).toBe(util);
    expect(util.equipment).toEqual(RECIPES.sprayer);
    expect(util.capabilities).toContain('spray');
    expect(util.capabilities).not.toContain('harvest');
    expect(follow[0].finishedAt! - follow[0].startedAt!).toBeGreaterThan(180);
    expect(k.process.spray.plantsTreated).toBeGreaterThan(0);
    expect(k.process.energy.imageWh).toBeGreaterThan(0);
    expect(k.process.energy.litreWh).toBeGreaterThan(0);
    expect(e0.util - util.battery.levelWh).toBeGreaterThan(0);
    expect(fm.getLog().some((l) => /reconfigured to sprayer/.test(l.text))).toBe(true);
  }, 180000);

  it('an aerial platform at cruise altitude crosses ground traffic without waiting or contact', () => {
    const st = new Station('air');
    const fleet = st.addChild(new FleetItem('fl'));
    const fm = new FleetManager(st, fleet);
    const g = ground(st, 'Ground', 0, 0); fm.addRobot(g);
    const uav = st.addChild(new MobileRobot('UAV'));
    Object.assign(uav.kin, { drive: 'aerial', maxSpeed: 3000, maxAccel: 1500, maxYawRate: 120, footprint: [900, 900, 400], cruiseAltitude: 4000, maxClimbRate: 2000 });
    uav.setPose2D(20000, 0, 180); uav.state.z = 4000; uav.state.zTarget = 4000; fm.addRobot(uav);
    const tg = fm.addTask({ type: 'goto', location: [20000, 0], duration: 1 });
    const tu = fm.addTask({ type: 'goto', location: [0, 0], duration: 1 });
    fm.startTask(g, tg); fm.startTask(uav, tu);
    let uavWait = 0;
    for (let i = 0; i < 3000 && !(tg.status === 'done' && tu.status === 'done'); i++) { fm.step(0.1); if (uav.state.status === 'waiting') uavWait++; }
    expect(tg.status).toBe('done');
    expect(tu.status).toBe('done');
    expect(uavWait).toBe(0);
    expect(fm.kpis().contacts.robotRobot).toBe(0);
    expect(uav.state.z!).toBeGreaterThan(3000);
  });
});

void ItemType;
