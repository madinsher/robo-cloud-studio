/**
 * Built-in robot library. DH parameters (standard DH, mm/deg).
 * UR series values are from the official UR DH tables. Other robots are
 * geometrically representative models (documented as approximate) — load a URDF for exact geometry.
 */
import { chainFromDH, ChainDef, DHParams } from '../kinematics/chain';
import { transl, identity, mul, rotx, roty, DEG } from '../math/pose';
import { Robot, RobotBrand } from './robot';

export interface RobotLibraryEntry {
  id: string;
  name: string;
  brand: RobotBrand;
  dof: number;
  reach: number;
  payload: number;
  approximate: boolean;
  category: 'industrial' | 'collaborative' | 'palletizer' | 'scara' | 'delta' | 'agri' | 'gantry';
  build: () => ChainDef;
  postProcessor: string;
  linkColors?: string[];
}

const ur = (d1: number, a2: number, a3: number, d4: number, d5: number, d6: number, vmax = 180): DHParams[] => [
  { theta: 0, d: d1, a: 0, alpha: 90, lower: -360, upper: 360, maxVelocity: vmax },
  { theta: 0, d: 0, a: a2, alpha: 0, lower: -360, upper: 360, maxVelocity: vmax, home: -90 },
  { theta: 0, d: 0, a: a3, alpha: 0, lower: -360, upper: 360, maxVelocity: vmax, home: 90 },
  { theta: 0, d: d4, a: 0, alpha: 90, lower: -360, upper: 360, maxVelocity: vmax, home: -90 },
  { theta: 0, d: d5, a: 0, alpha: -90, lower: -360, upper: 360, maxVelocity: vmax, home: -90 },
  { theta: 0, d: d6, a: 0, alpha: 0, lower: -360, upper: 360, maxVelocity: vmax },
];

/**
 * Generic 6R industrial arm (KUKA/ABB/Fanuc-like): shoulder offset, upper arm, forearm, wrist.
 * Joint 2 = 0 means the upper arm is vertical; vendor limits are given in the KUKA A2 convention
 * (A2 = -90 vertical) and shifted by +90 here.
 */
const sixR = (h: number, a1: number, a2: number, d4: number, d6: number, a3 = 0, limits?: number[][], vmax = 200): DHParams[] => [
  { theta: 0, d: h, a: a1, alpha: -90, lower: limits?.[0]?.[0] ?? -170, upper: limits?.[0]?.[1] ?? 170, maxVelocity: vmax },
  { theta: -90, d: 0, a: a2, alpha: 0, lower: (limits?.[1]?.[0] ?? -190) + 90, upper: (limits?.[1]?.[1] ?? 45) + 90, maxVelocity: vmax, home: 0 },
  { theta: 0, d: 0, a: a3, alpha: -90, lower: limits?.[2]?.[0] ?? -120, upper: limits?.[2]?.[1] ?? 155, maxVelocity: vmax, home: 0 },
  { theta: 0, d: d4, a: 0, alpha: 90, lower: limits?.[3]?.[0] ?? -185, upper: limits?.[3]?.[1] ?? 185, maxVelocity: vmax * 2 },
  { theta: 0, d: 0, a: 0, alpha: -90, lower: limits?.[4]?.[0] ?? -120, upper: limits?.[4]?.[1] ?? 120, maxVelocity: vmax * 2 },
  { theta: 0, d: d6, a: 0, alpha: 0, lower: limits?.[5]?.[0] ?? -350, upper: limits?.[5]?.[1] ?? 350, maxVelocity: vmax * 2 },
];

export const ROBOT_LIBRARY: RobotLibraryEntry[] = [
  { id: 'UR3e', name: 'Universal Robots UR3e', brand: 'UR', dof: 6, reach: 500, payload: 3, approximate: false, category: 'collaborative', postProcessor: 'Universal_Robots', build: () => chainFromDH('UR3e', ur(151.9, -243.65, -213.25, 131.05, 85.35, 92.1)) },
  { id: 'UR5e', name: 'Universal Robots UR5e', brand: 'UR', dof: 6, reach: 850, payload: 5, approximate: false, category: 'collaborative', postProcessor: 'Universal_Robots', build: () => chainFromDH('UR5e', ur(162.5, -425, -392.2, 133.3, 99.7, 99.6)) },
  { id: 'UR10e', name: 'Universal Robots UR10e', brand: 'UR', dof: 6, reach: 1300, payload: 12.5, approximate: false, category: 'collaborative', postProcessor: 'Universal_Robots', build: () => chainFromDH('UR10e', ur(180.7, -612.7, -571.55, 174.15, 119.85, 116.55, 120)) },
  { id: 'UR16e', name: 'Universal Robots UR16e', brand: 'UR', dof: 6, reach: 900, payload: 16, approximate: false, category: 'collaborative', postProcessor: 'Universal_Robots', build: () => chainFromDH('UR16e', ur(180.7, -478.4, -360, 174.15, 119.85, 116.55, 120)) },
  { id: 'UR20', name: 'Universal Robots UR20', brand: 'UR', dof: 6, reach: 1750, payload: 20, approximate: false, category: 'collaborative', postProcessor: 'Universal_Robots', build: () => chainFromDH('UR20', ur(236.3, -862, -728.7, 201, 159.3, 154.3, 120)) },
  { id: 'KUKA_KR6_R900', name: 'KUKA KR 6 R900 sixx (Agilus)', brand: 'KUKA', dof: 6, reach: 901, payload: 6, approximate: true, category: 'industrial', postProcessor: 'KUKA_KRC4', build: () => chainFromDH('KR6 R900', sixR(400, 25, 455, 420, 80, 35, [[-170, 170], [-190, 45], [-120, 156], [-185, 185], [-120, 120], [-350, 350]], 360)) },
  { id: 'KUKA_KR16_R2010', name: 'KUKA KR 16 R2010 (Cybertech)', brand: 'KUKA', dof: 6, reach: 2013, payload: 16, approximate: true, category: 'industrial', postProcessor: 'KUKA_KRC4', build: () => chainFromDH('KR16 R2010', sixR(520, 160, 980, 860, 153, 0, [[-185, 185], [-185, 65], [-138, 175], [-350, 350], [-130, 130], [-350, 350]], 200)) },
  { id: 'KUKA_KR210_R2700', name: 'KUKA KR 210 R2700 (Quantec)', brand: 'KUKA', dof: 6, reach: 2696, payload: 210, approximate: true, category: 'industrial', postProcessor: 'KUKA_KRC4', build: () => chainFromDH('KR210 R2700', sixR(675, 350, 1150, 1200, 215, 41, [[-185, 185], [-140, -5], [-120, 155], [-350, 350], [-125, 125], [-350, 350]], 100)) },
  { id: 'ABB_IRB120', name: 'ABB IRB 120-3/0.6', brand: 'ABB', dof: 6, reach: 580, payload: 3, approximate: true, category: 'industrial', postProcessor: 'ABB_RAPID_IRC5', build: () => chainFromDH('IRB 120', sixR(290, 0, 270, 302, 72, 70, [[-165, 165], [-110, 110], [-110, 70], [-160, 160], [-120, 120], [-400, 400]], 250)) },
  { id: 'ABB_IRB1200', name: 'ABB IRB 1200-7/0.7', brand: 'ABB', dof: 6, reach: 703, payload: 7, approximate: true, category: 'industrial', postProcessor: 'ABB_RAPID_IRC5', build: () => chainFromDH('IRB 1200', sixR(399.1, 0, 350, 351, 82, 42, [[-170, 170], [-100, 135], [-200, 70], [-270, 270], [-130, 130], [-400, 400]], 288)) },
  { id: 'ABB_IRB2600', name: 'ABB IRB 2600-20/1.65', brand: 'ABB', dof: 6, reach: 1650, payload: 20, approximate: true, category: 'industrial', postProcessor: 'ABB_RAPID_IRC5', build: () => chainFromDH('IRB 2600', sixR(445, 150, 700, 795, 85, 115, [[-180, 180], [-95, 155], [-180, 75], [-400, 400], [-120, 120], [-400, 400]], 175)) },
  { id: 'ABB_IRB6700', name: 'ABB IRB 6700-150/3.20', brand: 'ABB', dof: 6, reach: 3200, payload: 150, approximate: true, category: 'industrial', postProcessor: 'ABB_RAPID_IRC5', build: () => chainFromDH('IRB 6700', sixR(780, 320, 1280, 1592.5, 200, 200, [[-170, 170], [-155, -5], [-180, 70], [-300, 300], [-130, 130], [-360, 360]], 100)) },
  { id: 'FANUC_LRMATE200iD', name: 'Fanuc LR Mate 200iD', brand: 'Fanuc', dof: 6, reach: 717, payload: 7, approximate: true, category: 'industrial', postProcessor: 'Fanuc_R30iA', build: () => chainFromDH('LR Mate 200iD', sixR(330, 50, 330, 335, 80, 35, [[-170, 170], [-100, 145], [-140, 200], [-190, 190], [-125, 125], [-360, 360]], 370)) },
  { id: 'FANUC_M20iA', name: 'Fanuc M-20iA', brand: 'Fanuc', dof: 6, reach: 1811, payload: 20, approximate: true, category: 'industrial', postProcessor: 'Fanuc_R30iA', build: () => chainFromDH('M-20iA', sixR(525, 150, 790, 860, 100, 150, [[-185, 185], [-100, 160], [-185, 273], [-200, 200], [-140, 140], [-450, 450]], 195)) },
  { id: 'FANUC_M410iC', name: 'Fanuc M-410iC/185 (palletizer)', brand: 'Fanuc', dof: 4, reach: 3143, payload: 185, approximate: true, category: 'palletizer', postProcessor: 'Fanuc_R30iA', build: () => palletizer4('M-410iC/185', 720, 1130, 1600, 600, 400) },
  { id: 'YASKAWA_GP12', name: 'Yaskawa Motoman GP12', brand: 'Yaskawa', dof: 6, reach: 1440, payload: 12, approximate: true, category: 'industrial', postProcessor: 'Motoman', build: () => chainFromDH('GP12', sixR(450, 155, 614, 640, 100, 200, [[-170, 170], [-90, 155], [-85, 150], [-200, 200], [-150, 150], [-455, 455]], 260)) },
  { id: 'STAUBLI_TX2_60', name: 'Stäubli TX2-60', brand: 'Staubli', dof: 6, reach: 670, payload: 4.5, approximate: true, category: 'industrial', postProcessor: 'Staubli_VAL3', build: () => chainFromDH('TX2-60', sixR(375, 0, 290, 310, 70, 20, [[-180, 180], [-127.5, 127.5], [-152.5, 152.5], [-270, 270], [-121, 133.5], [-270, 270]], 435)) },
  { id: 'DOOSAN_M1013', name: 'Doosan M1013', brand: 'Doosan', dof: 6, reach: 1300, payload: 10, approximate: true, category: 'collaborative', postProcessor: 'Doosan_Robotics', build: () => chainFromDH('M1013', ur(135.2, -700, -556, 152, 122, 135, 120)) },
  { id: 'MECA500', name: 'Mecademic Meca500', brand: 'Mecademic', dof: 6, reach: 330, payload: 0.5, approximate: true, category: 'industrial', postProcessor: 'Mecademic', build: () => chainFromDH('Meca500', sixR(135, 0, 135, 120, 70, 38, [[-175, 175], [-160, 0], [-135, 70], [-170, 170], [-115, 115], [-360, 360]], 150)) },
  { id: 'GENERIC_PALLETIZER_4', name: 'Generic 4-axis palletizer', brand: 'Generic', dof: 4, reach: 2000, payload: 50, approximate: true, category: 'palletizer', postProcessor: 'Generic', build: () => palletizer4('Palletizer 4', 600, 900, 1100, 300, 250) },
  { id: 'GENERIC_SCARA', name: 'Generic SCARA 600', brand: 'Generic', dof: 4, reach: 600, payload: 6, approximate: true, category: 'scara', postProcessor: 'Generic', build: () => scara('SCARA 600', 300, 300, 200, 400) },
  { id: 'GANTRY_XYZ', name: 'Cartesian gantry XYZ (3 m x 2 m x 1 m)', brand: 'Generic', dof: 3, reach: 3000, payload: 100, approximate: false, category: 'gantry', postProcessor: 'Generic', build: () => gantry('Gantry XYZ', 3000, 2000, 1000) },
  { id: 'AGRI_PICKER_7', name: 'Agri harvester arm 7-DOF (telescopic)', brand: 'Custom', dof: 7, reach: 1800, payload: 3, approximate: true, category: 'agri', postProcessor: 'ROS2', build: () => agriPicker('Harvester 7') },
];

/** 4-axis palletizer: base yaw, shoulder, elbow (parallelogram keeps the wrist vertical), wrist yaw. Tool Z points down. */
function palletizer4(name: string, h: number, a2: number, a3: number, d4: number, a1: number): ChainDef {
  const j = (nm: string, origin: ReturnType<typeof transl>, axis: [number, number, number], lower: number, upper: number, v: number, home = 0, mimic?: { joint: string; multiplier: number; offset: number }) =>
    ({ name: nm, type: 'revolute' as const, origin, axis, lower, upper, maxVelocity: v, home, mimic });
  return {
    name,
    joints: [
      j('J1', transl(0, 0, h), [0, 0, 1], -170, 170, 120),
      j('J2', transl(a1, 0, 0), [0, 1, 0], -40, 90, 110, 20),
      j('J3', transl(0, 0, a2), [0, 1, 0], -30, 120, 110, 70),
      j('J3_comp', transl(a3, 0, 0), [0, 1, 0], -360, 360, 1000, 0, { joint: 'J2', multiplier: -1, offset: 0 }),
      j('J3_comp2', identity(), [0, 1, 0], -360, 360, 1000, 0, { joint: 'J3', multiplier: -1, offset: 0 }),
      j('J4', identity(), [0, 0, 1], -180, 180, 200),
    ],
    links: [{ name: 'base', visuals: [] }, { name: 'column', visuals: [] }, { name: 'upper_arm', visuals: [] }, { name: 'forearm', visuals: [] }, { name: 'wrist_a', visuals: [] }, { name: 'wrist_b', visuals: [] }, { name: 'wrist', visuals: [] }],
    flange: mul(transl(0, 0, -d4), rotx(180 * DEG)),
    dof: 4,
  };
}

function scara(name: string, a1: number, a2: number, zStroke: number, h: number): ChainDef {
  const dh: DHParams[] = [
    { theta: 0, d: h, a: a1, alpha: 0, lower: -130, upper: 130, maxVelocity: 400 },
    { theta: 0, d: 0, a: a2, alpha: 180, lower: -145, upper: 145, maxVelocity: 600 },
    { theta: 0, d: 0, a: 0, alpha: 0, type: 'prismatic', lower: 0, upper: zStroke, maxVelocity: 1000 },
    { theta: 0, d: 0, a: 0, alpha: 0, lower: -360, upper: 360, maxVelocity: 1000 },
  ];
  return chainFromDH(name, dh);
}

function gantry(name: string, x: number, y: number, z: number): ChainDef {
  return {
    name,
    joints: [
      { name: 'X', type: 'prismatic', origin: identity(), axis: [1, 0, 0], lower: 0, upper: x, maxVelocity: 1500, home: 0 },
      { name: 'Y', type: 'prismatic', origin: transl(0, 0, z + 200), axis: [0, 1, 0], lower: 0, upper: y, maxVelocity: 1500, home: 0 },
      { name: 'Z', type: 'prismatic', origin: identity(), axis: [0, 0, -1], lower: 0, upper: z, maxVelocity: 800, home: 0 },
    ],
    links: [{ name: 'base', visuals: [] }, { name: 'x_carriage', visuals: [] }, { name: 'y_carriage', visuals: [] }, { name: 'z_axis', visuals: [] }],
    flange: mul(transl(0, 0, -200), rotx(180 * DEG)),
    dof: 3,
  };
}

/** 7-DOF harvesting arm: yaw, pitch, telescopic extension, pitch, yaw, roll + gripper wrist pitch. */
function agriPicker(name: string): ChainDef {
  const dh: DHParams[] = [
    { theta: 0, d: 350, a: 0, alpha: -90, lower: -170, upper: 170, maxVelocity: 90 },
    { theta: -90, d: 0, a: 300, alpha: 0, lower: -60, upper: 120, maxVelocity: 80, home: 30 },
    { theta: 0, d: 0, a: 100, alpha: -90, lower: -120, upper: 60, maxVelocity: 80, home: -30 },
    { theta: 0, d: 500, a: 0, alpha: 0, type: 'prismatic', lower: 0, upper: 700, maxVelocity: 500 },
    { theta: 0, d: 0, a: 0, alpha: 90, lower: -180, upper: 180, maxVelocity: 180 },
    { theta: 0, d: 0, a: 0, alpha: -90, lower: -110, upper: 110, maxVelocity: 180 },
    { theta: 0, d: 120, a: 0, alpha: 0, lower: -360, upper: 360, maxVelocity: 300 },
  ];
  return chainFromDH(name, dh);
}

export function createRobotFromLibrary(id: string, name?: string): Robot {
  const e = ROBOT_LIBRARY.find((r) => r.id === id);
  if (!e) throw new Error(`Unknown robot ${id}`);
  const r = new Robot(name ?? e.name, e.build());
  r.brand = e.brand;
  r.model = e.name;
  r.postProcessor = e.postProcessor;
  r.params.payload = e.payload;
  r.params.libraryId = e.id;
  r.params.approximate = e.approximate;
  return r;
}

export { roty };
