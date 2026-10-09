/** Built-in demo stations. */
import { Station, Frame, Target, Tool, SceneObject } from './core/items/item';
import { Program } from './core/items/program';
import { createRobotFromLibrary } from './core/items/library';
import { transl, mul, rotx, rotz, DEG, rotationAngle } from './core/math/pose';
import { MobileRobot, ZoneItem } from './mobile/items';
import { FleetItem, FleetManager } from './fleet/fleet';
import { FieldItem, MissionItem, cropParams } from './agri/items';
import { generateOrchard, buildFieldMap, rectPolygon, makeHeadlandZones, makeCanopyZones } from './agri/orchard';
import { planMission } from './agri/missions';
import { makeConveyor, makeFeeder, makeProcess, makeSink, makeBuffer } from './vc/component';

export interface Demo { id: string; name: string; description: string; build: () => Station }

function pickPlaceCell(): Station {
  const st = new Station('Pick & place cell');
  const robot = st.addChild(createRobotFromLibrary('UR10e', 'UR10e'));
  robot.setPose(transl(0, 0, 700));
  const tool = robot.addChild(new Tool('Vacuum gripper'));
  tool.toolKind = 'vacuum';
  tool.setPoseTool(transl(0, 0, 160));
  robot.setTool(tool);
  const pedestal = st.addChild(new SceneObject('Pedestal'));
  pedestal.geometry = [{ primitive: { kind: 'cylinder', radius: 180, length: 700 }, origin: Array.from(transl(0, 0, 350)), color: '#495057' }];
  const table = st.addChild(new Frame('Table'));
  table.setPose(mul(transl(700, -300, 0), rotz(0)));
  const tableObj = table.addChild(new SceneObject('Table top'));
  tableObj.geometry = [{ primitive: { kind: 'box', size: [800, 1200, 40] }, origin: Array.from(transl(200, 400, 480)), color: '#868e96' }, { primitive: { kind: 'box', size: [60, 60, 460] }, origin: Array.from(transl(-150, 0, 230)), color: '#495057' }, { primitive: { kind: 'box', size: [60, 60, 460] }, origin: Array.from(transl(550, 800, 230)), color: '#495057' }];
  const conv = st.addChild(makeConveyor('Out conveyor', 3000, 250));
  conv.setPose(mul(transl(-600, 900, 700), rotz(90 * DEG)));
  robot.setFrame(table);
  const prog = st.addChild(new Program('PickPlace'));
  prog.setRobot(robot);
  prog.setSpeed(600, 120);
  prog.setRounding(5);
  const home = table.addChild(new Target('Home'));
  home.setJoints([0, -100, 110, -100, -90, 0]);
  home.setAsJointTarget();
  prog.addMoveJ(home);
  const boxes: SceneObject[] = [];
  for (let i = 0; i < 3; i++) {
    const box = table.addChild(new SceneObject(`Box ${i + 1}`));
    box.geometry = [{ primitive: { kind: 'box', size: [150, 150, 120] }, origin: Array.from(transl(0, 0, 60)), color: ['#e8590c', '#2f9e44', '#1971c2'][i] }];
    box.setPose(transl(100 + i * 220, 150, 500));
    box.bbox = { min: [-75, -75, 0], max: [75, 75, 120] };
    boxes.push(box);
    const above = table.addChild(new Target(`Above ${i + 1}`));
    above.setPose(mul(transl(100 + i * 220, 150, 800), rotx(180 * DEG)));
    const pick = table.addChild(new Target(`Pick ${i + 1}`));
    pick.setPose(mul(transl(100 + i * 220, 150, 622), rotx(180 * DEG)));
    const placeAbove = table.addChild(new Target(`Place above ${i + 1}`));
    placeAbove.setPose(mul(transl(-450, 700, 950), rotz(90 * DEG), rotx(180 * DEG)));
    const place = table.addChild(new Target(`Place ${i + 1}`));
    place.setPose(mul(transl(-450, 700, 700 + 120 * 0 + 1), rotz(90 * DEG), rotx(180 * DEG)));
    prog.addMoveJ(above);
    prog.addMoveL(pick);
    prog.event('attach', box.id);
    prog.setDO('Vacuum', true);
    prog.pause(200);
    prog.addMoveL(above);
    prog.addMoveJ(placeAbove);
    prog.addMoveL(place);
    prog.event('detach', box.id);
    prog.setDO('Vacuum', false);
    prog.addMoveL(placeAbove);
  }
  prog.addMoveJ(home);
  (conv.behaviour as any).next = null;
  void rotationAngle;
  return st;
}

function packingLine(): Station {
  const st = new Station('Packing line (process flow)');
  const feeder = st.addChild(makeFeeder('Crate feeder', 6, { name: 'Crate', geometry: { primitive: { kind: 'box', size: [400, 300, 250] }, origin: Array.from(transl(0, 0, 125)), color: '#d9a066' }, massKg: 12 }));
  const c1 = st.addChild(makeConveyor('Infeed conveyor', 5000, 400));
  c1.setPose(transl(-6000, 0, 800));
  const grader = st.addChild(makeProcess('Optical grader', 4, 1));
  grader.setPose(transl(-800, 0, 0));
  (grader.behaviour as any).mtbf = 600; (grader.behaviour as any).mttr = 40;
  const c2 = st.addChild(makeConveyor('Transfer conveyor', 3000, 400));
  c2.setPose(transl(-400, 0, 800));
  const buffer = st.addChild(makeBuffer('Pallet', 12, { cols: 3, rows: 2, layers: 2, pitch: [420, 320, 260] }));
  buffer.setPose(transl(3200, -900, 150));
  const sink = st.addChild(makeSink('Truck'));
  sink.setPose(transl(6000, -900, 0));
  (feeder.behaviour as any).next = c1.id;
  (c1.behaviour as any).next = grader.id;
  (grader.behaviour as any).next = c2.id;
  (c2.behaviour as any).next = buffer.id;
  (buffer.behaviour as any).next = sink.id;
  // palletizing robot next to the pallet
  const robot = st.addChild(createRobotFromLibrary('KUKA_KR16_R2010', 'Palletizer'));
  robot.setPose(transl(3000, 900, 0));
  const tool = robot.addChild(new Tool('Crate gripper'));
  tool.setPoseTool(transl(0, 0, 250));
  robot.setTool(tool);
  const floor = st.addChild(new SceneObject('Floor marking'));
  floor.geometry = [{ primitive: { kind: 'box', size: [16000, 6000, 10] }, origin: Array.from(transl(0, 0, -5)), color: '#343a40' }];
  return st;
}

function orchard(): Station {
  const st = new Station('Apple orchard — harvesting fleet');
  const field = st.addChild(new FieldItem('Block A (Gala)'));
  field.polygon = rectPolygon(90000, 42000);
  field.crop = cropParams('apple', { rowHeading: 0, headland: 7000, ripeFraction: 0.55 });
  field.geoOrigin = { lat: 46.87, lon: 35.35 };
  generateOrchard(field, 11);
  st.addChild(buildFieldMap(field, 250));
  makeHeadlandZones(field);
  makeCanopyZones(field);
  const fleet = st.addChild(new FleetItem('Harvest fleet'));
  const fm = new FleetManager(st, fleet);
  const charger = st.addChild(new ZoneItem('Charging & unloading'));
  charger.kind = 'charging';
  charger.polygon = rectPolygon(6000, 14000, -9000, 12000);
  fleet.chargingZoneIds.push(charger.id);
  for (let i = 0; i < 3; i++) {
    const m = st.addChild(new MobileRobot(`Harvest platform ${i + 1}`));
    Object.assign(m.kin, { drive: 'tracked', wheelBase: 1800, track: 1400, wheelRadius: 300, maxSpeed: 1300, footprint: [2600, 1600, 1400] });
    m.capabilities = ['harvest', 'transport'];
    m.color = '#c92a2a';
    m.setPose2D(-6000, 14000 + i * 4000, 0);
    m.home = { x: -6000, y: 14000 + i * 4000, theta: 0 };
    fm.addRobot(m);
    // a picking arm on each platform
    const arm = m.addChild(createRobotFromLibrary('UR10e', `Arm ${i + 1}`));
    arm.setPose(mul(transl(0, -900, 1400), rotz(-90 * DEG)));
    const g = arm.addChild(new Tool('Soft gripper'));
    g.toolKind = 'gripper';
    g.setPoseTool(transl(0, 0, 180));
    arm.setTool(g);
    arm.setJoints([0, -60, 80, -110, -90, 0]);
  }
  const sprayer = st.addChild(new MobileRobot('Sprayer'));
  Object.assign(sprayer.kin, { drive: 'ackermann', wheelBase: 1800, track: 1300, wheelRadius: 400, maxSpeed: 2500, footprint: [3000, 1500, 1500], minTurnRadius: 3000 });
  sprayer.capabilities = ['spray', 'transport'];
  sprayer.color = '#1971c2';
  sprayer.setPose2D(-6000, 8000, 0);
  sprayer.home = { x: -6000, y: 8000, theta: 0 };
  fm.addRobot(sprayer);
  const mission = st.addChild(new MissionItem('Harvest rows 1-4'));
  mission.missionType = 'harvest';
  mission.fieldId = field.id;
  mission.fleetId = fleet.id;
  mission.rowIds = field.rows().slice(0, 4).map((r) => r.id);
  mission.settings = { workSpeed: 400, bothSides: true, sideOffset: field.crop.rowSpacing / 2, secondsPerFruit: 4 };
  try { planMission(st, mission, fm); } catch { /* fleet may be empty */ }
  const scoutMission = st.addChild(new MissionItem('Spray rows 5-8'));
  scoutMission.missionType = 'spray';
  scoutMission.fieldId = field.id;
  scoutMission.fleetId = fleet.id;
  scoutMission.rowIds = field.rows().slice(4, 8).map((r) => r.id);
  scoutMission.settings = { workSpeed: 1500, bothSides: false, sideOffset: field.crop.rowSpacing / 2 };
  return st;
}

function greenhouse(): Station {
  const st = new Station('Greenhouse — tomato gutters');
  const field = st.addChild(new FieldItem('House 3'));
  field.indoor = true;
  field.polygon = rectPolygon(60000, 24000);
  field.crop = cropParams('tomato', { rowHeading: 0, headland: 4000, ripeFraction: 0.35 });
  generateOrchard(field, 5);
  st.addChild(buildFieldMap(field, 200));
  const fleet = st.addChild(new FleetItem('Pipe-rail trolleys'));
  const fm = new FleetManager(st, fleet);
  for (let i = 0; i < 2; i++) {
    const m = st.addChild(new MobileRobot(`Rail trolley ${i + 1}`));
    Object.assign(m.kin, { drive: 'differential', wheelBase: 900, track: 550, wheelRadius: 120, maxSpeed: 800, footprint: [1400, 700, 900] });
    m.capabilities = ['harvest', 'prune', 'scout'];
    m.color = '#f08c00';
    m.setPose2D(-3000, 3000 + i * 3000, 0);
    m.home = { x: -3000, y: 3000 + i * 3000, theta: 0 };
    fm.addRobot(m);
    const arm = m.addChild(createRobotFromLibrary('UR5e', `Picker ${i + 1}`));
    arm.setPose(mul(transl(0, 0, 900), rotz(-90 * DEG)));
    const g = arm.addChild(new Tool('Cutter gripper'));
    g.toolKind = 'shears';
    g.setPoseTool(transl(0, 0, 150));
    arm.setTool(g);
    arm.setJoints([0, -70, 90, -110, -90, 0]);
  }
  const packing = st.addChild(makeSink('Packing hall'));
  packing.setPose(transl(-6000, 12000, 0));
  const mission = st.addChild(new MissionItem('Harvest house 3'));
  mission.missionType = 'harvest';
  mission.fieldId = field.id;
  mission.fleetId = fleet.id;
  mission.settings = { workSpeed: 300, bothSides: true, sideOffset: field.crop.rowSpacing / 2, secondsPerFruit: 5 };
  try { planMission(st, mission, fm); } catch { /* ignore */ }
  return st;
}

function weldingCell(): Station {
  const st = new Station('Multi-robot welding cell');
  const r1 = st.addChild(createRobotFromLibrary('ABB_IRB2600', 'IRB 2600 L'));
  r1.setPose(transl(-1200, 0, 500));
  const r2 = st.addChild(createRobotFromLibrary('FANUC_M20iA', 'M-20iA R'));
  r2.setPose(mul(transl(1200, 0, 500), rotz(180 * DEG)));
  for (const r of [r1, r2]) { const t = r.addChild(new Tool('Torch')); t.toolKind = 'welding'; t.setPoseTool(mul(transl(0, 0, 350), rotx(-30 * DEG))); r.setTool(t); }
  const fix = st.addChild(new Frame('Fixture'));
  fix.setPose(transl(0, 600, 800));
  const part = fix.addChild(new SceneObject('Part'));
  part.geometry = [{ primitive: { kind: 'box', size: [1200, 300, 20] }, origin: Array.from(transl(0, 0, 10)), color: '#adb5bd' }, { primitive: { kind: 'box', size: [1200, 20, 200] }, origin: Array.from(transl(0, 0, 110)), color: '#adb5bd' }];
  part.curves = [{ name: 'weld seam', points: Array.from({ length: 25 }, (_, i) => [-600 + i * 50, 10, 20]) }];
  const positioner = st.addChild(createRobotFromLibrary('GENERIC_SCARA', 'Positioner'));
  positioner.setPose(transl(0, 2500, 0));
  for (const [r, sign] of [[r1, -1], [r2, 1]] as const) {
    r.setFrame(fix);
    const p = st.addChild(new Program(`Weld ${sign < 0 ? 'left' : 'right'}`));
    p.setRobot(r);
    p.setSpeed(15, 60);
    const home = fix.addChild(new Target(`Home ${r.name}`));
    home.setJoints(r.jointsHome());
    home.setAsJointTarget();
    p.addMoveJ(home);
    p.setDO('Arc', true);
    for (let i = 0; i < 6; i++) {
      const t = fix.addChild(new Target(`Seam ${r.name} ${i + 1}`));
      // torch travel angle (-15° about the tool X) keeps the wrist away from its singularity along the seam
      t.setPose(mul(transl(sign * (50 + i * 100), 10, 25), rotx(180 * DEG), rotz(sign > 0 ? 180 * DEG : 0), rotx(-15 * DEG)));
      if (i === 0) p.addMoveJ(t); else p.addMoveL(t);
    }
    p.setDO('Arc', false);
    p.addMoveJ(home);
  }
  return st;
}

function verticalBot(): Station {
  const st = new Station('VerticalBot palletizer (ROS 2)');
  const base = st.addChild(new MobileRobot('Vertical platform'));
  Object.assign(base.kin, { drive: 'omni', wheelBase: 600, track: 500, wheelRadius: 100, maxSpeed: 800, footprint: [900, 700, 400] });
  base.capabilities = ['transport', 'palletize'];
  base.rosNamespace = '/vertical_robot';
  const arm = base.addChild(createRobotFromLibrary('GENERIC_PALLETIZER_4', 'Palletizer'));
  arm.setPose(transl(0, 0, 400));
  arm.brand = 'ROS2';
  arm.postProcessor = 'ROS2';
  arm.connection = { rosNamespace: '/palletizer' };
  const tool = arm.addChild(new Tool('Suction cup'));
  tool.toolKind = 'vacuum';
  tool.setPoseTool(transl(0, 0, 120));
  arm.setTool(tool);
  const pallet = st.addChild(new Frame('Pallet'));
  pallet.setPose(transl(1500, 0, 150));
  const palletObj = pallet.addChild(new SceneObject('Euro pallet'));
  palletObj.geometry = [{ primitive: { kind: 'box', size: [1200, 800, 144] }, origin: Array.from(transl(0, 0, -72)), color: '#b08968' }];
  const prog = st.addChild(new Program('Palletize layer'));
  prog.setRobot(arm);
  arm.setFrame(pallet);
  prog.setSpeed(800, 100);
  const home = pallet.addChild(new Target('Home'));
  home.setJoints(arm.jointsHome());
  home.setAsJointTarget();
  prog.addMoveJ(home);
  for (let i = 0; i < 4; i++) {
    const t = pallet.addChild(new Target(`Place ${i + 1}`));
    t.setPose(mul(transl(-450 + (i % 2) * 400, -250 + Math.floor(i / 2) * 400, 350), rotx(180 * DEG)));
    prog.addMoveJ(t);
    prog.event('gripper_open');
    prog.pause(300);
  }
  prog.addMoveJ(home);
  st.addChild(new SceneObject('Note: import vertical_robot_model/urdf + meshes via File > Open to replace the placeholder kinematics'));
  return st;
}


/** Tutorial station: the result of the "Your first station" guide in the documentation (docs/getting-started/tutorial). */
function tutorialCell(): Station {
  const st = new Station('Tutorial — first station');
  const robot = st.addChild(createRobotFromLibrary('UR5e', 'UR5e'));
  robot.setPose(transl(0, 0, 500));
  const pedestal = st.addChild(new SceneObject('Pedestal'));
  pedestal.geometry = [{ primitive: { kind: 'cylinder', radius: 150, length: 500 }, origin: Array.from(transl(0, 0, 250)), color: '#495057' }];
  const gripper = robot.addChild(new Tool('Gripper'));
  gripper.toolKind = 'gripper';
  gripper.setPoseTool(transl(0, 0, 150));
  robot.setTool(gripper);
  const table = st.addChild(new Frame('Table'));
  table.setPose(transl(450, -250, 0));
  const top = table.addChild(new SceneObject('Table top'));
  top.geometry = [{ primitive: { kind: 'box', size: [500, 700, 30] }, origin: Array.from(transl(200, 300, 385)), color: '#868e96' }];
  const part = table.addChild(new SceneObject('Part'));
  part.geometry = [{ primitive: { kind: 'box', size: [100, 100, 80] }, origin: Array.from(transl(0, 0, 40)), color: '#e8590c' }];
  part.setPose(transl(150, 200, 400));
  part.bbox = { min: [-50, -50, 0], max: [50, 50, 80] };
  const bin = table.addChild(new SceneObject('Bin'));
  bin.geometry = [{ primitive: { kind: 'box', size: [200, 200, 20] }, origin: Array.from(transl(0, 0, 10)), color: '#1971c2' }];
  bin.setPose(transl(100, 500, 400));
  robot.setFrame(table);
  const prog = st.addChild(new Program('PickPart'));
  prog.setRobot(robot);
  prog.setSpeed(400, 90);
  prog.setRounding(3);
  const home = table.addChild(new Target('Home'));
  home.setJoints([0, -90, 90, -90, -90, 0]);
  home.setAsJointTarget();
  const approach = table.addChild(new Target('Approach'));
  approach.setPose(mul(transl(150, 200, 620), rotx(180 * DEG)));
  const pick = table.addChild(new Target('Pick'));
  pick.setPose(mul(transl(150, 200, 482), rotx(180 * DEG)));
  const placeAbove = table.addChild(new Target('Place above'));
  placeAbove.setPose(mul(transl(100, 500, 650), rotx(180 * DEG)));
  const place = table.addChild(new Target('Place'));
  place.setPose(mul(transl(100, 500, 502), rotx(180 * DEG)));
  prog.addMoveJ(home);
  prog.addMoveJ(approach);
  prog.addMoveL(pick);
  prog.event('gripper_close');
  prog.event('attach', part.id);
  prog.pause(300);
  prog.addMoveL(approach);
  prog.addMoveJ(placeAbove);
  prog.addMoveL(place);
  prog.event('gripper_open');
  prog.event('detach', part.id);
  prog.pause(300);
  prog.addMoveL(placeAbove);
  prog.addMoveJ(home);
  return st;
}

export const demos: Demo[] = [
  { id: 'pickplace', name: 'Pick & place cell (UR10e)', description: 'Robot arm, tool, targets, program with attach/detach. Run it and export with any post processor.', build: pickPlaceCell },
  { id: 'tutorial', name: 'Tutorial — first station (UR5e)', description: 'The station built in the documentation tutorial: UR5e on a pedestal, table frame, part and bin, gripper, pick-and-place program. Follow docs › Getting started › Your first station.', build: tutorialCell },
  { id: 'welding', name: 'Multi-robot welding cell', description: 'Two robots with torches following a seam on a fixture; positioner as external axis.', build: weldingCell },
  { id: 'packing', name: 'Packing line (process flow)', description: 'Visual-Components-style feeder → conveyor → grader (with failures) → conveyor → pallet buffer → truck. Start the world clock.', build: packingLine },
  { id: 'orchard', name: 'Apple orchard with harvesting fleet', description: 'Field with rows/trees/fruit, occupancy map, 3 tracked platforms with UR10e arms and a sprayer; harvest + spray missions. Start the world clock and watch the fleet.', build: orchard },
  { id: 'greenhouse', name: 'Greenhouse tomato gutters', description: 'Indoor gutters with pipe-rail trolleys carrying UR5e pickers.', build: greenhouse },
  { id: 'verticalbot', name: 'VerticalBot palletizer (this repo)', description: 'Placeholder for the vertical_bot_ros palletizer on an omni base. Import the URDF + meshes for exact geometry; export with the ROS 2 post.', build: verticalBot },
];
