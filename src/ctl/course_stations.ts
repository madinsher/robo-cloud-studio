/**
 * Course examples A and B as runnable stations: library robots, zones, parts, robot programs, and the control models
 * with every state / transition bound to a station action (so "Run on station" drives real robots).
 *
 *  - A (youBot mobile manipulator): a differential base with a UR3e arm and a gripper, zones home / table / bin / dock,
 *    arm programs Reach / Stow / Grasp / Release (Grasp and Release write the IO `Holding` / `Delivered`). The DES is the
 *    course plant made cyclic (`o_next`) under the supervisor E1 E2 E3 E5; vision events come from `sensor` lines on the
 *    base zone, mission decisions from `guard` lines over the signals. The BT mission uses the same supervisor.
 *  - B (production cell): M1 (UR10e) serves the machine-loading process A and unloads the AMR of process B through the
 *    shared zone Z — the course net with its siphon deadlock; M2 (IRB 1200) unloads the machine concurrently. The GMEC
 *    monitor a1 + b1 ≤ 1 (graphdoc `applyDeadlockMonitors`) makes it live.
 */
import { Station, Frame, Target, Tool, SceneObject, ItemType } from '../core/items/item';
import { Program } from '../core/items/program';
import { Robot } from '../core/items/robot';
import { createRobotFromLibrary } from '../core/items/library';
import { transl, mul } from '../core/math/pose';
import { MobileRobot, ZoneItem } from '../mobile/items';
import { youbotPlant, youbotSpecs, YOUBOT_UC, YOUBOT_UO } from './examples';
import { specsToDsl } from './analysis';

const rect = (x0: number, y0: number, x1: number, y1: number) => [[x0, y0], [x1, y0], [x1, y1], [x0, y1]];
function zone(st: Station, name: string, poly: number[][]): ZoneItem { const z = st.addChild(new ZoneItem(name)); z.polygon = poly; return z; }
function box(parent: Station | Frame, name: string, size: [number, number, number], at: [number, number, number], color: string): SceneObject {
  const o = parent.addChild(new SceneObject(name)); o.geometry = [{ primitive: { kind: 'box', size }, origin: Array.from(transl(0, 0, size[2] / 2)), color }]; o.setPose(transl(...at)); return o;
}
function jointTarget(robot: Robot, name: string, q: number[]): Target { const t = robot.addChild(new Target(name)); t.setJoints(q); t.setAsJointTarget(); t.robotId = robot.id; return t; }
/** A program of joint moves through named joint targets (targets are created under the robot, linked to it). */
function jointProgram(st: Station, robot: Robot, name: string, poses: Array<[string, number[]]>, speed = 120): Program {
  const p = st.addChild(new Program(name)); p.setRobot(robot); p.setSpeed(500, speed);
  for (const [tn, q] of poses) { const t = robot.children.find((c) => c instanceof Target && c.name === tn) as Target | undefined ?? jointTarget(robot, tn, q); p.addMoveJ(t); }
  return p;
}

// ---------------------------------------------------------------------------------------------
// Example A — youBot mobile manipulator
// ---------------------------------------------------------------------------------------------

export const YOUBOT_STATION_ZONES: Record<string, [number, number]> = { home: [0, 0], table: [4000, 0], bin: [4000, 3000], dock: [0, 3000] };

export function buildCourseStationA(st: Station = new Station('Course example A — youBot cell')): { station: Station; base: MobileRobot; arm: Robot; part: SceneObject } {
  for (const [n, [x, y]] of Object.entries(YOUBOT_STATION_ZONES)) zone(st, n, rect(x - 700, y - 700, x + 700, y + 700));
  const base = st.addChild(new MobileRobot('youBot'));
  Object.assign(base.kin, { drive: 'differential', maxSpeed: 800, maxReverseSpeed: 300, maxAccel: 600, maxYawRate: 90, footprint: [580, 380, 140] });
  base.setPose2D(0, 0, 0); base.home = { x: 0, y: 0, theta: 0 };
  const arm = base.addChild(createRobotFromLibrary('UR3e', 'youBot arm')); arm.setPose(transl(150, 0, 140));
  const grip = arm.addChild(new Tool('Gripper')); grip.toolKind = 'gripper'; grip.setPoseTool(transl(0, 0, 120)); arm.setTool(grip);
  box(st, 'Table', [800, 800, 700], [4900, 0, 0], '#868e96');
  const part = box(st, 'Part', [60, 60, 60], [4550, 0, 700], '#e8590c');
  box(st, 'Bin', [600, 600, 400], [4900, 3000, 0], '#2f9e44');
  const stow = [0, -90, 0, -90, 0, 0], reach = [0, -40, -80, -60, 90, 0];
  arm.setJoints(stow);
  jointProgram(st, arm, 'Reach', [['A stow', stow], ['A reach', reach]]);
  jointProgram(st, arm, 'Stow', [['A reach', reach], ['A stow', stow]]);
  const grasp = st.addChild(new Program('Grasp')); grasp.setRobot(arm); grasp.pause(500); grasp.event('attach', part.id); grasp.setDO('Holding', 1); grasp.pause(300);
  const release = st.addChild(new Program('Release')); release.setRobot(arm); release.pause(300); release.event('detach', part.id); release.setDO('Holding', 0); release.setDO('Delivered', 1);
  return { station: st, base, arm, part };
}

const S = youbotSpecs();
/** The course plant, cyclic, under E1 E2 E3 E5, with every activity bound to the station of buildCourseStationA. */
export const YOUBOT_STATION_DES = `des youBot cell on the station (example A)
${specsToDsl(youbotPlant(true, true).map((d) => d.toSpec()), [S.E1, S.E2, S.E3, S.E5].map((d) => d.toSpec()), YOUBOT_UC, YOUBOT_UO)}
blackboard Holding=0 Delivered=0
action Base.mT goto=table robot=youBot done=b_arrive timeout=60
action Base.mB goto=bin robot=youBot done=b_arrive timeout=60
action Base.mH goto=home robot=youBot done=b_arrive timeout=60
action Arm.R program=Reach done=a_reached timeout=30
action Arm.W program=Stow done=a_stowed timeout=30
action Grip.C program=Grasp done=g_ok fail=g_miss timeout=20
action Object.Bx program=Release timeout=20
sensor v_detect "zone == 'table' && Holding == 0"
sensor v_lost "zone != 'table'"
guard b_go_table "Delivered == 0 && Holding == 0"
guard b_go_box "Holding == 1"
guard b_go_home "Delivered == 1"
guard a_reach "(zone == 'table' && Holding == 0 && Delivered == 0) || (zone == 'bin' && Holding == 1)"
guard g_close "zone == 'table'"
guard g_put "zone == 'bin'"
guard o_next "false"
guard o_giveup "false"`;

/** BT mission of example A for a cyclic plant: the vision loses the object when the base leaves the table, `o_next` starts the next object. */
export const YOUBOT_STATION_BT = `bt youBot mission on the station (example A)
fallback root
  sequence emergency
    condition estop
    action halt timeout=1
  fallback mission
    sequence collect
      condition "targets > 0 || held || arm_extended"
      sequence "collect one" memory
        action select_target
        action goto zone=table event=b_go_table arrive=b_arrive timeout=120
        action event emit=v_detect
        action reach event=a_reach done=a_reached timeout=30
        retry 2
          sequence grasp memory
            action refine_pose timeout=5
            action grasp event=g_close ok=g_ok miss=g_miss timeout=20 post="held"
        action stow event=a_stow done=a_stowed timeout=30
        action event emit=v_lost
        action goto zone=bin event=b_go_box arrive=b_arrive timeout=120
        action reach event=a_reach done=a_reached timeout=30
        action place put=g_put timeout=20
        action stow event=a_stow done=a_stowed timeout=30
        action set next=1 event=o_next
    sequence finish memory
      action goto zone=home event=b_go_home arrive=b_arrive timeout=120
      action report
supervisor youBot_cell`;

// ---------------------------------------------------------------------------------------------
// Example B — production cell
// ---------------------------------------------------------------------------------------------

export function buildCourseStationB(st: Station = new Station('Course example B — production cell')): { station: Station; m1: Robot; m2: Robot; amr: MobileRobot } {
  const m1 = st.addChild(createRobotFromLibrary('UR10e', 'M1')); m1.setPose(transl(0, 0, 0));
  const m2 = st.addChild(createRobotFromLibrary('ABB_IRB1200', 'M2')); m2.setPose(transl(2200, 600, 0));
  box(st, 'Buffer B', [500, 500, 600], [-900, 500, 0], '#495057');
  box(st, 'Machine S', [900, 700, 900], [1100, 900, 0], '#1c7ed6');
  zone(st, 'Z', rect(400, -1600, 1800, -500));
  zone(st, 'Dock', rect(-4000, -1700, -2600, -300));
  const amr = st.addChild(new MobileRobot('AMR'));
  Object.assign(amr.kin, { drive: 'differential', maxSpeed: 1000, maxAccel: 800, maxYawRate: 120, footprint: [900, 600, 400] });
  amr.setPose2D(-3300, -1000, 0); amr.home = { x: -3300, y: -1000, theta: 0 };
  const home1 = [0, -90, 90, -90, -90, 0];
  m1.setJoints(home1); m2.setJoints([0, 0, 0, 0, 30, 0]);
  jointProgram(st, m1, 'A1 Pick from buffer', [['M1 home', home1], ['M1 buffer', [150, -60, 100, -130, -90, 0]], ['M1 home', home1]]);
  jointProgram(st, m1, 'A2 Load machine', [['M1 machine', [50, -50, 80, -120, -90, 0]], ['M1 zone Z', [-40, -45, 75, -120, -90, 0]], ['M1 machine', [50, -50, 80, -120, -90, 0]]]);
  jointProgram(st, m1, 'A3 Retract', [['M1 home', home1]]);
  jointProgram(st, m1, 'B2 Unload AMR', [['M1 zone Z', [-40, -45, 75, -120, -90, 0]], ['M1 buffer', [150, -60, 100, -130, -90, 0]], ['M1 home', home1]]);
  jointProgram(st, m2, 'C1 Unload machine', [['M2 rest', [0, 0, 0, 0, 30, 0]], ['M2 machine', [120, 30, 10, 0, 50, 0]], ['M2 out', [-60, 20, 20, 0, 50, 0]], ['M2 rest', [0, 0, 0, 0, 30, 0]]]);
  void ItemType; void mul;
  return { station: st, m1, m2, amr };
}

/** The course cell net (§4.6.4) with its operations bound to the station of buildCourseStationB, plus M2's independent unloading cycle. */
export const CELL_STATION_PETRI = `petri Production cell on the station (example B)
place p1 tokens=1 kind=idle "process A ready"
place a1 kind=activity "M1 holds a part"
place a2 kind=activity "M1 loads the machine in Z"
place p2 tokens=1 kind=idle "AMR ready"
place b1 kind=activity "AMR in zone Z"
place b2 kind=activity "M1 unloads the AMR"
place rM tokens=1 kind=resource "robot M1"
place rZ tokens=1 kind=resource "zone Z"
place p3 tokens=1 kind=idle "M2 ready"
transition t1a
transition t2a
transition t3a
transition t1b
transition t2b
transition t3b
transition t1c delay=2
arc p1, rM -> t1a -> a1
arc a1, rZ -> t2a -> a2
arc a2 -> t3a -> rM, rZ, p1
arc p2, rZ -> t1b -> b1
arc b1, rM -> t2b -> b2
arc b2 -> t3b -> rZ, rM, p2
arc p3 -> t1c -> p3
action t1a program="A1 Pick from buffer" robot=M1 timeout=60
action t2a program="A2 Load machine" robot=M1 timeout=60
action t3a program="A3 Retract" robot=M1 timeout=60
action t1b goto=Z robot=AMR timeout=60
action t2b program="B2 Unload AMR" robot=M1 timeout=60
action t3b goto=Dock robot=AMR timeout=60
action t1c program="C1 Unload machine" robot=M2 timeout=60
check ctl AG !deadlock`;
