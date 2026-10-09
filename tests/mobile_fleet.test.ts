import { describe, it, expect } from 'vitest';
import { Station, ItemType } from '../src/core/items/item';
import { MobileRobot, MapItem, ZoneItem } from '../src/mobile/items';
import { planPath, coveragePath, rowTraversalPath, pathLength } from '../src/mobile/planner';
import { followPath, stepMobile } from '../src/mobile/controller';
import { FleetItem, FleetManager } from '../src/fleet/fleet';
import { FieldItem, MissionItem, cropParams } from '../src/agri/items';
import { generateOrchard, buildFieldMap, rectPolygon, fruitWorldPositions } from '../src/agri/orchard';
import { planMission, updateMissionProgress, generateHarvestProgram } from '../src/agri/missions';
import { createRobotFromLibrary } from '../src/core/items/library';
import { transl, mul, rotz, DEG } from '../src/core/math/pose';
import { ProcessSimulator, makeConveyor, makeFeeder, makeProcess, makeSink } from '../src/vc/component';
import { fieldsFromGeoJSON } from '../src/agri/geo';

describe('mobile planning & control', () => {
  it('A* finds a path around an obstacle and pure pursuit follows it', () => {
    const st = new Station();
    const map = st.addChild(new MapItem());
    map.resize(100, 100, 200, -10000, -10000);
    map.inflation = 300;
    map.fillRect(-500, -6000, 500, 6000);
    const res = planPath(map, [-5000, 0], [5000, 0]);
    expect(res.ok).toBe(true);
    expect(res.length).toBeGreaterThan(10000);
    expect(res.path.every((p) => map.isFree(p[0], p[1]))).toBe(true);
    const r = st.addChild(new MobileRobot('AMR'));
    r.setPose2D(-5000, 0, 0);
    followPath(r, res.path);
    let steps = 0;
    while (stepMobile(r, 0.05) && steps++ < 20000) {}
    expect(Math.hypot(r.state.x - 5000, r.state.y)).toBeLessThan(250);
    expect(r.battery.levelWh).toBeLessThan(r.battery.capacityWh);
  });

  it('coverage and row traversal paths are produced', () => {
    const cov = coveragePath(rectPolygon(20000, 10000), 2000, 0);
    expect(cov.length).toBeGreaterThan(6);
    const rows = [0, 1, 2].map((i) => ({ start: [0, i * 3000], end: [20000, i * 3000] }));
    const p = rowTraversalPath(rows);
    expect(pathLength(p)).toBeGreaterThan(60000);
  });
});

describe('orchard, fleet and missions', () => {
  it('generates an orchard, a map and executes a harvest mission with a fleet', () => {
    const st = new Station('Farm');
    const field = st.addChild(new FieldItem('Block A'));
    field.polygon = rectPolygon(60000, 30000);
    field.crop = cropParams('apple', { rowHeading: 0, headland: 5000 });
    const rows = generateOrchard(field, 3);
    expect(rows.length).toBeGreaterThan(3);
    expect(rows[0].plants.length).toBeGreaterThan(10);
    expect(fruitWorldPositions(rows[0]).length).toBeGreaterThan(50);
    const map = st.addChild(buildFieldMap(field, 250));
    expect(map.width).toBeGreaterThan(100);
    // robots start at headland
    const fleetItem = st.addChild(new FleetItem('Fleet'));
    const fleet = new FleetManager(st, fleetItem);
    for (let i = 0; i < 2; i++) {
      const r = st.addChild(new MobileRobot(`Picker ${i + 1}`));
      r.capabilities = ['harvest', 'transport'];
      r.kin.maxSpeed = 1500;
      r.setPose2D(2000, 2000 + i * 3000, 0);
      fleet.addRobot(r);
    }
    const charge = st.addChild(new ZoneItem('Charger'));
    charge.kind = 'charging';
    charge.polygon = rectPolygon(2000, 2000, 1000, 1000);
    const mission = st.addChild(new MissionItem('Harvest A'));
    mission.missionType = 'harvest';
    mission.fieldId = field.id;
    mission.rowIds = rows.slice(0, 2).map((r) => r.id);
    mission.settings.secondsPerFruit = 0.01;
    const plan = planMission(st, mission, fleet);
    expect(plan.tasks.length).toBe(4);
    expect(plan.fruitTargets).toBeGreaterThan(0);
    let t = 0;
    while (t < 3600 && fleet.fleet.tasks.some((k) => k.status !== 'done')) { fleet.step(0.2); t += 0.2; }
    updateMissionProgress(mission, fleet);
    expect(mission.progress).toBe(1);
    const kpi = fleet.kpis();
    expect(kpi.tasksDone).toBe(4);
    expect(kpi.distanceTravelled).toBeGreaterThan(50000);
    expect(kpi.utilization).toBeGreaterThan(0.1);
  });

  it('generates an arm harvesting program from fruit positions', () => {
    const st = new Station('Farm');
    const field = st.addChild(new FieldItem('Block'));
    field.polygon = rectPolygon(30000, 12000);
    field.crop = cropParams('apple', { headland: 3000 });
    const rows = generateOrchard(field, 7);
    const row = rows[0];
    const arm = st.addChild(createRobotFromLibrary('UR10e', 'Picker arm'));
    // place the arm beside the row, 1.4 m from the row axis, at canopy height
    const [px, py] = row.pointAt(row.length() / 2, -1400);
    arm.setPose(mul(transl(px, py, 1200), rotz(90 * DEG)));
    const { program, picked } = generateHarvestProgram(st, arm, [row], { limit: 10 });
    expect(picked).toBeGreaterThan(0);
    expect(program.instructions().length).toBeGreaterThan(picked * 4);
  });

  it('imports fields from GeoJSON', () => {
    const gj = { type: 'FeatureCollection', features: [{ type: 'Feature', properties: { name: 'Orchard' }, geometry: { type: 'Polygon', coordinates: [[[30.0, 50.0], [30.001, 50.0], [30.001, 50.0005], [30.0, 50.0005], [30.0, 50.0]]] } }] };
    const { fields } = fieldsFromGeoJSON(gj);
    expect(fields.length).toBe(1);
    expect(fields[0].polygon.length).toBe(4);
    const xs = fields[0].polygon.map((p) => p[0]);
    expect(Math.max(...xs) - Math.min(...xs)).toBeGreaterThan(60000);
  });
});

describe('process components (Visual Components style)', () => {
  it('feeder -> conveyor -> process -> sink flow accumulates throughput', () => {
    const st = new Station();
    const feeder = st.addChild(makeFeeder('Feeder', 5, { name: 'Crate', geometry: { primitive: { kind: 'box', size: [400, 300, 250] }, color: '#c88' } }));
    const conv = st.addChild(makeConveyor('Conveyor', 6000, 500));
    const proc = st.addChild(makeProcess('Packer', 3));
    const sink = st.addChild(makeSink('Truck'));
    conv.setPose(transl(0, 0, 800));
    proc.setPose(transl(6500, 0, 0));
    (feeder.behaviour as any).next = conv.id;
    (conv.behaviour as any).next = proc.id;
    (proc.behaviour as any).next = sink.id;
    const sim = new ProcessSimulator(st);
    for (let i = 0; i < 1200; i++) sim.step(0.1);
    const stats = sim.statistics();
    const sinkStat = stats.find((s) => s.id === sink.id)!;
    expect(sinkStat.exited).toBeGreaterThanOrEqual(15);
    expect(stats.find((s) => s.id === proc.id)!.utilization).toBeGreaterThan(0.3);
    expect(st.itemsOfType(ItemType.OBJECT).length).toBeLessThan(10);
  });
});
