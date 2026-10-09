/**
 * Online robot library.
 *
 * RoboDK ships a proprietary online library of `.robot` files. The same industrial robots
 * (Fanuc, ABB, KUKA, Yaskawa/Motoman, Stäubli, Universal Robots, Franka, Kinova, Doosan, …) are
 * published openly as ROS URDF/xacro *description packages* by their vendors and by the
 * ROS-Industrial consortium (BSD / Apache licences). This module turns those packages into an
 * online library: it fetches the xacro straight from GitHub, expands it with the built-in xacro
 * processor, builds an exact kinematic chain and downloads the visual meshes (STL / COLLADA)
 * into the asset store. No server is needed — everything runs in the browser (or in Node).
 *
 * Any `package://` URI whose package belongs to a known repository can also be resolved through
 * `resolvePackageUri`, which lets the plain URDF importer fetch meshes it is missing.
 */
import { parseURDF, chainFromURDF, URDFModel } from '../urdf/urdf';
import { Robot, RobotBrand } from '../../core/items/robot';
import { AssetStore } from '../../scene/assets';
import { parseDAE } from '../mesh/dae';
import { parseOBJ } from '../mesh/obj';

export interface OnlineRepo {
  id: string;
  owner: string;
  repo: string;
  ref: string;
  /** Package name -> directory inside the repository (defaults to the package name). */
  packages?: Record<string, string>;
  /** Packages whose names start with one of these prefixes live in this repository. */
  prefixes?: string[];
  licence: string;
}

export interface OnlineRobotEntry {
  id: string;
  brand: RobotBrand;
  name: string;
  /** Repository id (see ONLINE_REPOS). */
  repo: string;
  /** Path of the top-level xacro/URDF inside the repository. */
  file: string;
  dof: number;
  payload: number;
  /** Nominal reach, mm. */
  reach: number;
  category: 'industrial' | 'collaborative' | 'palletizing' | 'welding' | 'painting' | 'scara' | 'delta' | 'research';
  postProcessor: string;
  /** xacro args to pass. */
  args?: Record<string, string>;
  /** Tip link overriding the automatic choice (tool0 / flange / tool_frame). */
  tipLink?: string;
  /** Free-text notes shown in the UI. */
  notes?: string;
}

export const ONLINE_REPOS: OnlineRepo[] = [
  { id: 'fanuc', owner: 'ros-industrial', repo: 'fanuc', ref: 'melodic-devel', prefixes: ['fanuc_'], licence: 'BSD-3' },
  { id: 'abb', owner: 'ros-industrial', repo: 'abb', ref: 'kinetic-devel', prefixes: ['abb_'], licence: 'BSD-3' },
  { id: 'kuka', owner: 'ros-industrial', repo: 'kuka_experimental', ref: 'melodic-devel', prefixes: ['kuka_'], licence: 'BSD-3' },
  { id: 'motoman', owner: 'ros-industrial', repo: 'motoman', ref: 'kinetic-devel', prefixes: ['motoman_'], licence: 'BSD-3' },
  { id: 'staubli', owner: 'ros-industrial', repo: 'staubli_experimental', ref: 'kinetic-devel', prefixes: ['staubli_'], licence: 'BSD-3' },
  { id: 'staubli_main', owner: 'ros-industrial', repo: 'staubli', ref: 'melodic-devel', packages: { staubli_resources: 'staubli_resources', staubli_rx160_support: 'staubli_rx160_support' }, licence: 'BSD-3' },
  { id: 'ur', owner: 'ros-industrial', repo: 'universal_robot', ref: 'kinetic-devel', prefixes: ['ur_'], licence: 'BSD-3' },
  { id: 'franka', owner: 'frankaemika', repo: 'franka_ros', ref: '0.7.1', prefixes: ['franka_'], licence: 'Apache-2.0' },
  { id: 'kortex', owner: 'Kinovarobotics', repo: 'ros_kortex', ref: 'melodic-devel', prefixes: ['kortex_'], licence: 'BSD-3' },
  { id: 'doosan', owner: 'doosan-robotics', repo: 'doosan-robot', ref: 'master', prefixes: ['dsr_'], licence: 'BSD-3' },
  { id: 'robotiq', owner: 'ros-industrial', repo: 'robotiq', ref: 'kinetic-devel', prefixes: ['robotiq_'], licence: 'BSD-2' },
];

const F = 'Fanuc_R30iA', A = 'ABB_RAPID_IRC5', K = 'KUKA_KRC4', M = 'Motoman', S = 'Staubli_VAL3', U = 'Universal_Robots', D = 'Doosan_Robotics', G = 'Generic';

export const ONLINE_ROBOT_LIBRARY: OnlineRobotEntry[] = [
  // ---- Fanuc (ros-industrial/fanuc) -------------------------------------------------------
  { id: 'fanuc_lrmate200i', brand: 'Fanuc', name: 'Fanuc LR Mate 200i', repo: 'fanuc', file: 'fanuc_lrmate200i_support/urdf/lrmate200i.xacro', dof: 6, payload: 3, reach: 700, category: 'industrial', postProcessor: F },
  { id: 'fanuc_lrmate200ib', brand: 'Fanuc', name: 'Fanuc LR Mate 200iB', repo: 'fanuc', file: 'fanuc_lrmate200ib_support/urdf/lrmate200ib.xacro', dof: 6, payload: 5, reach: 704, category: 'industrial', postProcessor: F },
  { id: 'fanuc_lrmate200ic', brand: 'Fanuc', name: 'Fanuc LR Mate 200iC', repo: 'fanuc', file: 'fanuc_lrmate200ic_support/urdf/lrmate200ic.xacro', dof: 6, payload: 5, reach: 704, category: 'industrial', postProcessor: F },
  { id: 'fanuc_lrmate200id', brand: 'Fanuc', name: 'Fanuc LR Mate 200iD', repo: 'fanuc', file: 'fanuc_lrmate200id_support/urdf/lrmate200id.xacro', dof: 6, payload: 7, reach: 717, category: 'industrial', postProcessor: F },
  { id: 'fanuc_lrmate200id7l', brand: 'Fanuc', name: 'Fanuc LR Mate 200iD/7L', repo: 'fanuc', file: 'fanuc_lrmate200id_support/urdf/lrmate200id7l.xacro', dof: 6, payload: 7, reach: 911, category: 'industrial', postProcessor: F },
  { id: 'fanuc_lrmate200id7lc', brand: 'Fanuc', name: 'Fanuc LR Mate 200iD/7LC (clean room)', repo: 'fanuc', file: 'fanuc_lrmate200id_support/urdf/lrmate200id7lc.xacro', dof: 6, payload: 7, reach: 911, category: 'industrial', postProcessor: F },
  { id: 'fanuc_lrmate200id7h', brand: 'Fanuc', name: 'Fanuc LR Mate 200iD/7H (5 axes)', repo: 'fanuc', file: 'fanuc_lrmate200id_support/urdf/lrmate200id7h.xacro', dof: 5, payload: 7, reach: 717, category: 'industrial', postProcessor: F },
  { id: 'fanuc_m6ib', brand: 'Fanuc', name: 'Fanuc M-6iB', repo: 'fanuc', file: 'fanuc_m6ib_support/urdf/m6ib.xacro', dof: 6, payload: 6, reach: 1373, category: 'industrial', postProcessor: F },
  { id: 'fanuc_m10ia', brand: 'Fanuc', name: 'Fanuc M-10iA', repo: 'fanuc', file: 'fanuc_m10ia_support/urdf/m10ia.xacro', dof: 6, payload: 10, reach: 1420, category: 'industrial', postProcessor: F },
  { id: 'fanuc_m10ia7l', brand: 'Fanuc', name: 'Fanuc M-10iA/7L', repo: 'fanuc', file: 'fanuc_m10ia_support/urdf/m10ia7l.xacro', dof: 6, payload: 7, reach: 1632, category: 'industrial', postProcessor: F },
  { id: 'fanuc_m16ib20', brand: 'Fanuc', name: 'Fanuc M-16iB/20', repo: 'fanuc', file: 'fanuc_m16ib_support/urdf/m16ib20.xacro', dof: 6, payload: 20, reach: 1667, category: 'industrial', postProcessor: F },
  { id: 'fanuc_m20ia', brand: 'Fanuc', name: 'Fanuc M-20iA', repo: 'fanuc', file: 'fanuc_m20ia_support/urdf/m20ia.xacro', dof: 6, payload: 20, reach: 1811, category: 'industrial', postProcessor: F },
  { id: 'fanuc_m20ia10l', brand: 'Fanuc', name: 'Fanuc M-20iA/10L', repo: 'fanuc', file: 'fanuc_m20ia_support/urdf/m20ia10l.xacro', dof: 6, payload: 10, reach: 2009, category: 'industrial', postProcessor: F },
  { id: 'fanuc_m20ib25', brand: 'Fanuc', name: 'Fanuc M-20iB/25', repo: 'fanuc', file: 'fanuc_m20ib_support/urdf/m20ib25.xacro', dof: 6, payload: 25, reach: 1853, category: 'industrial', postProcessor: F },
  { id: 'fanuc_m430ia2f', brand: 'Fanuc', name: 'Fanuc M-430iA/2F (picking, 5 axes)', repo: 'fanuc', file: 'fanuc_m430ia_support/urdf/m430ia2f.xacro', dof: 5, payload: 2, reach: 900, category: 'industrial', postProcessor: F },
  { id: 'fanuc_cr7ia', brand: 'Fanuc', name: 'Fanuc CR-7iA (collaborative)', repo: 'fanuc', file: 'fanuc_cr7ia_support/urdf/cr7ia.xacro', dof: 6, payload: 7, reach: 717, category: 'collaborative', postProcessor: F },
  { id: 'fanuc_cr7ial', brand: 'Fanuc', name: 'Fanuc CR-7iA/L (collaborative)', repo: 'fanuc', file: 'fanuc_cr7ia_support/urdf/cr7ial.xacro', dof: 6, payload: 7, reach: 911, category: 'collaborative', postProcessor: F },
  { id: 'fanuc_cr35ia', brand: 'Fanuc', name: 'Fanuc CR-35iA (collaborative)', repo: 'fanuc', file: 'fanuc_cr35ia_support/urdf/cr35ia.xacro', dof: 6, payload: 35, reach: 1813, category: 'collaborative', postProcessor: F },
  { id: 'fanuc_m710ic50', brand: 'Fanuc', name: 'Fanuc M-710iC/50', repo: 'fanuc', file: 'fanuc_m710ic_support/urdf/m710ic50.xacro', dof: 6, payload: 50, reach: 2050, category: 'industrial', postProcessor: F },
  { id: 'fanuc_m900ia260l', brand: 'Fanuc', name: 'Fanuc M-900iA/260L', repo: 'fanuc', file: 'fanuc_m900ia_support/urdf/m900ia260l.xacro', dof: 6, payload: 260, reach: 3100, category: 'industrial', postProcessor: F },
  { id: 'fanuc_m900ib700', brand: 'Fanuc', name: 'Fanuc M-900iB/700', repo: 'fanuc', file: 'fanuc_m900ib_support/urdf/m900ib700.xacro', dof: 6, payload: 700, reach: 2832, category: 'industrial', postProcessor: F },
  { id: 'fanuc_r1000ia80f', brand: 'Fanuc', name: 'Fanuc R-1000iA/80F', repo: 'fanuc', file: 'fanuc_r1000ia_support/urdf/r1000ia80f.xacro', dof: 6, payload: 80, reach: 2230, category: 'industrial', postProcessor: F },
  { id: 'fanuc_r2000ic165f', brand: 'Fanuc', name: 'Fanuc R-2000iC/165F', repo: 'fanuc', file: 'fanuc_r2000ic_support/urdf/r2000ic165f.xacro', dof: 6, payload: 165, reach: 2655, category: 'industrial', postProcessor: F },
  { id: 'fanuc_r2000ic210f', brand: 'Fanuc', name: 'Fanuc R-2000iC/210F', repo: 'fanuc', file: 'fanuc_r2000ic_support/urdf/r2000ic210f.xacro', dof: 6, payload: 210, reach: 2655, category: 'industrial', postProcessor: F },
  // ---- ABB (ros-industrial/abb) -----------------------------------------------------------
  { id: 'abb_irb120', brand: 'ABB', name: 'ABB IRB 120-3/0.58', repo: 'abb', file: 'abb_irb120_support/urdf/irb120_3_58.xacro', dof: 6, payload: 3, reach: 580, category: 'industrial', postProcessor: A },
  { id: 'abb_irb2400', brand: 'ABB', name: 'ABB IRB 2400', repo: 'abb', file: 'abb_irb2400_support/urdf/irb2400.xacro', dof: 6, payload: 12, reach: 1550, category: 'industrial', postProcessor: A },
  { id: 'abb_irb4400l', brand: 'ABB', name: 'ABB IRB 4400L-30/2.43', repo: 'abb', file: 'abb_irb4400_support/urdf/irb4400l_30_243.xacro', dof: 6, payload: 30, reach: 2430, category: 'industrial', postProcessor: A },
  { id: 'abb_irb5400', brand: 'ABB', name: 'ABB IRB 5400 (painting)', repo: 'abb', file: 'abb_irb5400_support/urdf/irb5400.xacro', dof: 6, payload: 25, reach: 3100, category: 'painting', postProcessor: A },
  { id: 'abb_irb6600', brand: 'ABB', name: 'ABB IRB 6600-225/2.55', repo: 'abb', file: 'abb_irb6600_support/urdf/irb6600_225_255.xacro', dof: 6, payload: 225, reach: 2550, category: 'industrial', postProcessor: A },
  { id: 'abb_irb6640', brand: 'ABB', name: 'ABB IRB 6640-185/2.8', repo: 'abb', file: 'abb_irb6640_support/urdf/irb6640_185_280.xacro', dof: 6, payload: 185, reach: 2800, category: 'industrial', postProcessor: A },
  // ---- KUKA (ros-industrial/kuka_experimental) ---------------------------------------------
  { id: 'kuka_kr3r540', brand: 'KUKA', name: 'KUKA KR 3 R540 (Agilus)', repo: 'kuka', file: 'kuka_kr3_support/urdf/kr3r540.xacro', dof: 6, payload: 3, reach: 541, category: 'industrial', postProcessor: K },
  { id: 'kuka_kr5_arc', brand: 'KUKA', name: 'KUKA KR 5 arc', repo: 'kuka', file: 'kuka_kr5_support/urdf/kr5_arc.xacro', dof: 6, payload: 5, reach: 1412, category: 'welding', postProcessor: K },
  { id: 'kuka_kr6r700sixx', brand: 'KUKA', name: 'KUKA KR 6 R700 sixx (Agilus)', repo: 'kuka', file: 'kuka_kr6_support/urdf/kr6r700sixx.xacro', dof: 6, payload: 6, reach: 706, category: 'industrial', postProcessor: K },
  { id: 'kuka_kr6r900sixx', brand: 'KUKA', name: 'KUKA KR 6 R900 sixx (Agilus)', repo: 'kuka', file: 'kuka_kr6_support/urdf/kr6r900sixx.xacro', dof: 6, payload: 6, reach: 901, category: 'industrial', postProcessor: K },
  { id: 'kuka_kr10r900_2', brand: 'KUKA', name: 'KUKA KR 10 R900-2 (Agilus)', repo: 'kuka', file: 'kuka_kr10_support/urdf/kr10r900_2.xacro', dof: 6, payload: 10, reach: 901, category: 'industrial', postProcessor: K },
  { id: 'kuka_kr10r1100sixx', brand: 'KUKA', name: 'KUKA KR 10 R1100 sixx (Agilus)', repo: 'kuka', file: 'kuka_kr10_support/urdf/kr10r1100sixx.xacro', dof: 6, payload: 10, reach: 1101, category: 'industrial', postProcessor: K },
  { id: 'kuka_kr16_2', brand: 'KUKA', name: 'KUKA KR 16-2', repo: 'kuka', file: 'kuka_kr16_support/urdf/kr16_2.xacro', dof: 6, payload: 16, reach: 1611, category: 'industrial', postProcessor: K },
  { id: 'kuka_kr120r2500pro', brand: 'KUKA', name: 'KUKA KR 120 R2500 pro (Quantec)', repo: 'kuka', file: 'kuka_kr120_support/urdf/kr120r2500pro.xacro', dof: 6, payload: 120, reach: 2500, category: 'industrial', postProcessor: K },
  { id: 'kuka_kr150_2', brand: 'KUKA', name: 'KUKA KR 150-2', repo: 'kuka', file: 'kuka_kr150_support/urdf/kr150_2.xacro', dof: 6, payload: 150, reach: 2700, category: 'industrial', postProcessor: K },
  { id: 'kuka_kr150r3100_2', brand: 'KUKA', name: 'KUKA KR 150 R3100-2 (Quantec)', repo: 'kuka', file: 'kuka_kr150_support/urdf/kr150r3100_2.xacro', dof: 6, payload: 150, reach: 3100, category: 'industrial', postProcessor: K },
  { id: 'kuka_kr210l150', brand: 'KUKA', name: 'KUKA KR 210 L150', repo: 'kuka', file: 'kuka_kr210_support/urdf/kr210l150.xacro', dof: 6, payload: 150, reach: 3100, category: 'industrial', postProcessor: K },
  { id: 'kuka_lbr_iiwa_14_r820', brand: 'KUKA', name: 'KUKA LBR iiwa 14 R820 (7 axes)', repo: 'kuka', file: 'kuka_lbr_iiwa_support/urdf/lbr_iiwa_14_r820.xacro', dof: 7, payload: 14, reach: 820, category: 'collaborative', postProcessor: K },
  // ---- Yaskawa Motoman (ros-industrial/motoman) --------------------------------------------
  { id: 'motoman_gp4', brand: 'Yaskawa', name: 'Yaskawa Motoman GP4', repo: 'motoman', file: 'motoman_gp4_support/urdf/gp4.xacro', dof: 6, payload: 4, reach: 550, category: 'industrial', postProcessor: M },
  { id: 'motoman_gp7', brand: 'Yaskawa', name: 'Yaskawa Motoman GP7', repo: 'motoman', file: 'motoman_gp7_support/urdf/gp7.xacro', dof: 6, payload: 7, reach: 927, category: 'industrial', postProcessor: M },
  { id: 'motoman_gp8', brand: 'Yaskawa', name: 'Yaskawa Motoman GP8', repo: 'motoman', file: 'motoman_gp8_support/urdf/gp8.xacro', dof: 6, payload: 8, reach: 727, category: 'industrial', postProcessor: M },
  { id: 'motoman_gp12', brand: 'Yaskawa', name: 'Yaskawa Motoman GP12', repo: 'motoman', file: 'motoman_gp12_support/urdf/gp12.xacro', dof: 6, payload: 12, reach: 1440, category: 'industrial', postProcessor: M },
  { id: 'motoman_gp20hl', brand: 'Yaskawa', name: 'Yaskawa Motoman GP20HL', repo: 'motoman', file: 'motoman_gp20hl_support/urdf/gp20hl.xacro', dof: 6, payload: 20, reach: 3124, category: 'industrial', postProcessor: M },
  { id: 'motoman_gp25', brand: 'Yaskawa', name: 'Yaskawa Motoman GP25', repo: 'motoman', file: 'motoman_gp25_support/urdf/gp25.xacro', dof: 6, payload: 25, reach: 1730, category: 'industrial', postProcessor: M },
  { id: 'motoman_gp35l', brand: 'Yaskawa', name: 'Yaskawa Motoman GP35L', repo: 'motoman', file: 'motoman_gp35l_support/urdf/gp35l.xacro', dof: 6, payload: 35, reach: 2538, category: 'industrial', postProcessor: M },
  { id: 'motoman_gp50', brand: 'Yaskawa', name: 'Yaskawa Motoman GP50', repo: 'motoman', file: 'motoman_gp50_support/urdf/gp50.xacro', dof: 6, payload: 50, reach: 2061, category: 'industrial', postProcessor: M },
  { id: 'motoman_gp70l', brand: 'Yaskawa', name: 'Yaskawa Motoman GP70L', repo: 'motoman', file: 'motoman_gp70l_support/urdf/gp70l.xacro', dof: 6, payload: 70, reach: 2733, category: 'industrial', postProcessor: M },
  { id: 'motoman_gp88', brand: 'Yaskawa', name: 'Yaskawa Motoman GP88', repo: 'motoman', file: 'motoman_gp88_support/urdf/gp88.xacro', dof: 6, payload: 88, reach: 2236, category: 'industrial', postProcessor: M },
  { id: 'motoman_gp110', brand: 'Yaskawa', name: 'Yaskawa Motoman GP110', repo: 'motoman', file: 'motoman_gp110_support/urdf/gp110.xacro', dof: 6, payload: 110, reach: 2236, category: 'industrial', postProcessor: M },
  { id: 'motoman_gp180', brand: 'Yaskawa', name: 'Yaskawa Motoman GP180', repo: 'motoman', file: 'motoman_gp180_support/urdf/gp180.xacro', dof: 6, payload: 180, reach: 2702, category: 'industrial', postProcessor: M },
  { id: 'motoman_gp200r', brand: 'Yaskawa', name: 'Yaskawa Motoman GP200R (shelf)', repo: 'motoman', file: 'motoman_gp200r_support/urdf/gp200r.xacro', dof: 6, payload: 200, reach: 3140, category: 'industrial', postProcessor: M },
  { id: 'motoman_hc10', brand: 'Yaskawa', name: 'Yaskawa Motoman HC10 (collaborative)', repo: 'motoman', file: 'motoman_hc10_support/urdf/hc10.xacro', dof: 6, payload: 10, reach: 1200, category: 'collaborative', postProcessor: M },
  { id: 'motoman_hc10dt', brand: 'Yaskawa', name: 'Yaskawa Motoman HC10DT (collaborative)', repo: 'motoman', file: 'motoman_hc10_support/urdf/hc10dt.xacro', dof: 6, payload: 10, reach: 1200, category: 'collaborative', postProcessor: M },
  { id: 'motoman_hc20dtp', brand: 'Yaskawa', name: 'Yaskawa Motoman HC20DTP (collaborative)', repo: 'motoman', file: 'motoman_hc20_support/urdf/hc20dtp.xacro', dof: 6, payload: 20, reach: 1700, category: 'collaborative', postProcessor: M },
  { id: 'motoman_mh5', brand: 'Yaskawa', name: 'Yaskawa Motoman MH5', repo: 'motoman', file: 'motoman_mh5_support/urdf/mh5.xacro', dof: 6, payload: 5, reach: 706, category: 'industrial', postProcessor: M },
  { id: 'motoman_mh5l', brand: 'Yaskawa', name: 'Yaskawa Motoman MH5L', repo: 'motoman', file: 'motoman_mh5_support/urdf/mh5l.xacro', dof: 6, payload: 5, reach: 895, category: 'industrial', postProcessor: M },
  { id: 'motoman_mh12', brand: 'Yaskawa', name: 'Yaskawa Motoman MH12', repo: 'motoman', file: 'motoman_mh12_support/urdf/mh12.xacro', dof: 6, payload: 12, reach: 1440, category: 'industrial', postProcessor: M },
  { id: 'motoman_mh50', brand: 'Yaskawa', name: 'Yaskawa Motoman MH50', repo: 'motoman', file: 'motoman_mh50_support/urdf/mh50.xacro', dof: 6, payload: 50, reach: 2061, category: 'industrial', postProcessor: M },
  { id: 'motoman_mh110', brand: 'Yaskawa', name: 'Yaskawa Motoman MH110', repo: 'motoman', file: 'motoman_mh110_support/urdf/mh110.xacro', dof: 6, payload: 110, reach: 2236, category: 'industrial', postProcessor: M },
  { id: 'motoman_ma2010', brand: 'Yaskawa', name: 'Yaskawa Motoman MA2010 (welding)', repo: 'motoman', file: 'motoman_ma2010_support/urdf/ma2010.xacro', dof: 6, payload: 10, reach: 2010, category: 'welding', postProcessor: M },
  { id: 'motoman_sia5d', brand: 'Yaskawa', name: 'Yaskawa Motoman SIA5D (7 axes)', repo: 'motoman', file: 'motoman_sia5d_support/urdf/sia5d.xacro', dof: 7, payload: 5, reach: 559, category: 'industrial', postProcessor: M },
  { id: 'motoman_sia10d', brand: 'Yaskawa', name: 'Yaskawa Motoman SIA10D (7 axes)', repo: 'motoman', file: 'motoman_sia10d_support/urdf/sia10d.xacro', dof: 7, payload: 10, reach: 720, category: 'industrial', postProcessor: M },
  { id: 'motoman_sia20d', brand: 'Yaskawa', name: 'Yaskawa Motoman SIA20D (7 axes)', repo: 'motoman', file: 'motoman_sia20d_support/urdf/sia20d.xacro', dof: 7, payload: 20, reach: 910, category: 'industrial', postProcessor: M },
  // ---- Stäubli (ros-industrial/staubli_experimental, staubli) -------------------------------
  { id: 'staubli_tx60', brand: 'Staubli', name: 'Stäubli TX60', repo: 'staubli', file: 'staubli_tx60_support/urdf/tx60.xacro', dof: 6, payload: 3.5, reach: 670, category: 'industrial', postProcessor: S },
  { id: 'staubli_tx90', brand: 'Staubli', name: 'Stäubli TX90', repo: 'staubli', file: 'staubli_tx90_support/urdf/tx90.xacro', dof: 6, payload: 7, reach: 1000, category: 'industrial', postProcessor: S },
  { id: 'staubli_tx2_60', brand: 'Staubli', name: 'Stäubli TX2-60', repo: 'staubli', file: 'staubli_tx2_60_support/urdf/tx2_60.xacro', dof: 6, payload: 4.5, reach: 670, category: 'industrial', postProcessor: S },
  { id: 'staubli_tx2_90', brand: 'Staubli', name: 'Stäubli TX2-90', repo: 'staubli', file: 'staubli_tx2_90_support/urdf/tx2_90.xacro', dof: 6, payload: 7, reach: 1000, category: 'industrial', postProcessor: S },
  { id: 'staubli_tx2_90l', brand: 'Staubli', name: 'Stäubli TX2-90L', repo: 'staubli', file: 'staubli_tx2_90_support/urdf/tx2_90l.xacro', dof: 6, payload: 5, reach: 1200, category: 'industrial', postProcessor: S },
  { id: 'staubli_rx160', brand: 'Staubli', name: 'Stäubli RX160', repo: 'staubli_main', file: 'staubli_rx160_support/urdf/rx160.xacro', dof: 6, payload: 20, reach: 1710, category: 'industrial', postProcessor: S },
  // ---- Universal Robots (ros-industrial/universal_robot) ----------------------------------
  { id: 'ur3', brand: 'UR', name: 'Universal Robots UR3', repo: 'ur', file: 'ur_description/urdf/ur3_robot.urdf.xacro', dof: 6, payload: 3, reach: 500, category: 'collaborative', postProcessor: U },
  { id: 'ur5', brand: 'UR', name: 'Universal Robots UR5', repo: 'ur', file: 'ur_description/urdf/ur5_robot.urdf.xacro', dof: 6, payload: 5, reach: 850, category: 'collaborative', postProcessor: U },
  { id: 'ur10', brand: 'UR', name: 'Universal Robots UR10', repo: 'ur', file: 'ur_description/urdf/ur10_robot.urdf.xacro', dof: 6, payload: 10, reach: 1300, category: 'collaborative', postProcessor: U },
  { id: 'ur3e', brand: 'UR', name: 'Universal Robots UR3e', repo: 'ur', file: 'ur_e_description/urdf/ur3e_robot.urdf.xacro', dof: 6, payload: 3, reach: 500, category: 'collaborative', postProcessor: U },
  { id: 'ur5e', brand: 'UR', name: 'Universal Robots UR5e', repo: 'ur', file: 'ur_e_description/urdf/ur5e_robot.urdf.xacro', dof: 6, payload: 5, reach: 850, category: 'collaborative', postProcessor: U },
  { id: 'ur10e', brand: 'UR', name: 'Universal Robots UR10e', repo: 'ur', file: 'ur_e_description/urdf/ur10e_robot.urdf.xacro', dof: 6, payload: 10, reach: 1300, category: 'collaborative', postProcessor: U },
  // ---- Franka Emika -------------------------------------------------------------------------
  { id: 'franka_panda', brand: 'Generic', name: 'Franka Emika Panda (7 axes, with hand)', repo: 'franka', file: 'franka_description/robots/panda_arm_hand.urdf.xacro', dof: 7, payload: 3, reach: 855, category: 'research', postProcessor: 'ROS2', tipLink: 'panda_link8' },
  // ---- Kinova -------------------------------------------------------------------------------
  { id: 'kinova_gen3_7dof', brand: 'Generic', name: 'Kinova Gen3 (7 axes)', repo: 'kortex', file: 'kortex_description/robots/gen3.xacro', dof: 7, payload: 4, reach: 902, category: 'collaborative', postProcessor: 'ROS2', args: { arm: 'gen3', dof: '7', vision: 'true', gripper: '', sim: 'false', prefix: '' }, tipLink: 'tool_frame' },
  { id: 'kinova_gen3_6dof', brand: 'Generic', name: 'Kinova Gen3 (6 axes)', repo: 'kortex', file: 'kortex_description/robots/gen3.xacro', dof: 6, payload: 4, reach: 891, category: 'collaborative', postProcessor: 'ROS2', args: { arm: 'gen3', dof: '6', vision: 'true', gripper: '', sim: 'false', prefix: '' }, tipLink: 'tool_frame' },
  { id: 'kinova_gen3_lite', brand: 'Generic', name: 'Kinova Gen3 lite (6 axes)', repo: 'kortex', file: 'kortex_description/robots/gen3.xacro', dof: 6, payload: 0.5, reach: 760, category: 'collaborative', postProcessor: 'ROS2', args: { arm: 'gen3_lite', dof: '6', vision: 'false', gripper: '', sim: 'false', prefix: '' }, tipLink: 'tool_frame' },
  // ---- Doosan -------------------------------------------------------------------------------
  { id: 'doosan_m0609', brand: 'Doosan', name: 'Doosan M0609', repo: 'doosan', file: 'dsr_description/xacro/m0609.urdf.xacro', dof: 6, payload: 6, reach: 900, category: 'collaborative', postProcessor: D },
  { id: 'doosan_m0617', brand: 'Doosan', name: 'Doosan M0617', repo: 'doosan', file: 'dsr_description/xacro/m0617.urdf.xacro', dof: 6, payload: 6, reach: 1700, category: 'collaborative', postProcessor: D },
  { id: 'doosan_m1013', brand: 'Doosan', name: 'Doosan M1013', repo: 'doosan', file: 'dsr_description/xacro/m1013.urdf.xacro', dof: 6, payload: 10, reach: 1300, category: 'collaborative', postProcessor: D },
  { id: 'doosan_m1509', brand: 'Doosan', name: 'Doosan M1509', repo: 'doosan', file: 'dsr_description/xacro/m1509.urdf.xacro', dof: 6, payload: 15, reach: 900, category: 'collaborative', postProcessor: D },
  { id: 'doosan_a0509', brand: 'Doosan', name: 'Doosan A0509', repo: 'doosan', file: 'dsr_description/xacro/a0509.urdf.xacro', dof: 6, payload: 5, reach: 900, category: 'collaborative', postProcessor: D },
  { id: 'doosan_a0912', brand: 'Doosan', name: 'Doosan A0912', repo: 'doosan', file: 'dsr_description/xacro/a0912.urdf.xacro', dof: 6, payload: 9, reach: 1200, category: 'collaborative', postProcessor: D },
  { id: 'doosan_h2017', brand: 'Doosan', name: 'Doosan H2017', repo: 'doosan', file: 'dsr_description/xacro/h2017.urdf.xacro', dof: 6, payload: 20, reach: 1700, category: 'collaborative', postProcessor: D },
  { id: 'doosan_h2515', brand: 'Doosan', name: 'Doosan H2515', repo: 'doosan', file: 'dsr_description/xacro/h2515.urdf.xacro', dof: 6, payload: 25, reach: 1500, category: 'collaborative', postProcessor: D },
];
void G;

// ------------------------------------------------------------------------------------------
// URL resolution
// ------------------------------------------------------------------------------------------

export function rawBase(repo: OnlineRepo): string {
  return `https://raw.githubusercontent.com/${repo.owner}/${repo.repo}/${repo.ref}/`;
}

/** Find the repository that hosts a ROS package. */
export function repoForPackage(pkg: string): OnlineRepo | null {
  for (const r of ONLINE_REPOS) if (r.packages && pkg in r.packages) return r;
  for (const r of ONLINE_REPOS) if (r.prefixes?.some((p) => pkg.startsWith(p))) return r;
  return null;
}

/** Resolve `package://pkg/path` (or `$(find pkg)/path`) to a raw GitHub URL, or null if unknown. */
export function resolvePackageUri(uri: string, hint?: OnlineRepo): string | null {
  let m = /^package:\/\/([^/]+)\/(.*)$/.exec(uri.trim());
  if (!m) {
    const f = /^\$\(find\s+([^)]+)\)\/?(.*)$/.exec(uri.trim());
    if (f) m = [f[0], f[1].trim(), f[2]] as any;
  }
  if (!m) return null;
  const [, pkg, rest] = m;
  const repo = (hint?.prefixes?.some((p) => pkg.startsWith(p)) || (hint?.packages && pkg in hint.packages)) ? hint : repoForPackage(pkg);
  if (!repo) return null;
  const dir = repo.packages?.[pkg] ?? pkg;
  return `${rawBase(repo)}${dir}/${rest}`;
}

function extOf(path: string): string {
  return (path.split('?')[0].split('#')[0].split('.').pop() ?? '').toLowerCase();
}

// ------------------------------------------------------------------------------------------
// Fetching
// ------------------------------------------------------------------------------------------

export interface OnlineFetchOptions {
  fetchText?: (url: string) => Promise<string | null>;
  fetchBytes?: (url: string) => Promise<Uint8Array | null>;
  /** Where visual meshes are registered (ids = package:// URIs). Omit to skip meshes. */
  assets?: AssetStore;
  /** Download meshes (default true when `assets` is given). */
  meshes?: boolean;
  onProgress?: (message: string, done?: number, total?: number) => void;
  /** Extra xacro args (merged over the catalogue entry's). */
  args?: Record<string, string>;
  /** Max parallel downloads. */
  concurrency?: number;
}

export interface OnlineRobotResult {
  robot: Robot;
  model: URDFModel;
  entry: OnlineRobotEntry;
  url: string;
  /** All xacro/URDF files fetched. */
  files: string[];
  meshes: { loaded: string[]; failed: string[] };
  warnings: string[];
}

async function defaultFetchText(url: string): Promise<string | null> {
  const r = await fetch(url);
  if (!r.ok) return null;
  return r.text();
}
async function defaultFetchBytes(url: string): Promise<Uint8Array | null> {
  const r = await fetch(url);
  if (!r.ok) return null;
  return new Uint8Array(await r.arrayBuffer());
}

async function mapLimit<T, R>(items: T[], limit: number, fn: (t: T) => Promise<R>): Promise<R[]> {
  const out: R[] = new Array(items.length);
  let i = 0;
  const worker = async () => {
    while (i < items.length) {
      const k = i++;
      out[k] = await fn(items[k]);
    }
  };
  await Promise.all(Array.from({ length: Math.min(limit, items.length) }, worker));
  return out;
}

/** Static include targets in a xacro text (those not depending on macro parameters). */
function staticIncludes(text: string): string[] {
  const out: string[] = [];
  const re = /<xacro:include\s+[^>]*filename\s*=\s*"([^"]+)"/g;
  let m: RegExpExecArray | null;
  while ((m = re.exec(text))) {
    const f = m[1];
    if (f.includes('${') || f.includes('$(arg')) continue;
    out.push(f);
  }
  return out;
}

function resolveRelative(file: string, baseUrl: string): string {
  try {
    return new URL(file, baseUrl).toString();
  } catch {
    return file;
  }
}

/**
 * Fetch a catalogue robot (or any xacro/URDF reachable through the known repositories) and build
 * a Robot with exact kinematics and meshes.
 */
export async function fetchOnlineRobot(entryOrId: OnlineRobotEntry | string, opts: OnlineFetchOptions = {}): Promise<OnlineRobotResult> {
  const entry = typeof entryOrId === 'string' ? ONLINE_ROBOT_LIBRARY.find((e) => e.id === entryOrId) : entryOrId;
  if (!entry) throw new Error(`Unknown online robot ${entryOrId}`);
  const repo = ONLINE_REPOS.find((r) => r.id === entry.repo);
  if (!repo) throw new Error(`Unknown repository ${entry.repo}`);
  const url = `${rawBase(repo)}${entry.file}`;
  return fetchRobotFromUrl(url, { ...opts, args: { ...(entry.args ?? {}), ...(opts.args ?? {}) } }, entry, repo);
}

/** Fetch a xacro/URDF by URL. `package://` references are resolved through the known repositories. */
export async function fetchRobotFromUrl(url: string, opts: OnlineFetchOptions = {}, entry?: OnlineRobotEntry, repoHint?: OnlineRepo): Promise<OnlineRobotResult> {
  const fetchText = opts.fetchText ?? defaultFetchText;
  const fetchBytes = opts.fetchBytes ?? defaultFetchBytes;
  const progress = opts.onProgress ?? (() => {});
  const warnings: string[] = [];
  const cache = new Map<string, string | null>(); // url -> text (null = failed)
  const byBase = new Map<string, string>(); // basename -> url
  const baseDir = url.slice(0, url.lastIndexOf('/') + 1);

  const toUrl = (file: string): string | null => {
    if (/^https?:\/\//.test(file)) return file;
    if (file.startsWith('package://') || file.startsWith('$(find')) return resolvePackageUri(file, repoHint);
    if (file.startsWith('file://')) return null;
    return resolveRelative(file, baseDir);
  };

  const load = async (u: string): Promise<string | null> => {
    if (cache.has(u)) return cache.get(u)!;
    progress(`Fetching ${u.split('/').slice(-2).join('/')}`);
    let text: string | null = null;
    try {
      text = await fetchText(u);
    } catch (e) {
      warnings.push(`Fetch failed: ${u} (${(e as Error).message})`);
    }
    cache.set(u, text);
    if (text) {
      byBase.set(u.split('/').pop()!.toLowerCase(), u);
      // prefetch static includes in parallel (one level; deeper levels are picked up on later rounds)
      const inc = staticIncludes(text).map(toUrl).filter((x): x is string => !!x && !cache.has(x));
      await Promise.all(inc.map((x) => load(x)));
    }
    return text;
  };

  const main = await load(url);
  if (!main) throw new Error(`Could not download ${url}`);

  // Expand with the xacro processor; every include miss is fetched and the expansion re-run.
  let model: URDFModel | null = null;
  let lastError: Error | null = null;
  const missing = new Set<string>();
  const resolveInclude = (file: string): string | null => {
    const u = toUrl(file);
    if (u && cache.get(u)) return cache.get(u)!;
    const b = byBase.get(file.split('/').pop()!.toLowerCase());
    if (b && cache.get(b)) return cache.get(b)!;
    if (u && !cache.has(u)) missing.add(u);
    else if (u) warnings.push(`Include not found: ${file}`);
    return null;
  };
  for (let round = 0; round < 10; round++) {
    missing.clear();
    lastError = null;
    try {
      model = parseURDF(main, { resolveInclude, args: opts.args });
    } catch (e) {
      lastError = e as Error;
      model = null;
    }
    if (missing.size === 0) break;
    await Promise.all([...missing].map((u) => load(u)));
  }
  if (!model) throw lastError ?? new Error('URDF expansion failed');
  if (model.joints.length === 0) throw new Error(`No joints found in ${url} — xacro macro may not have expanded`);

  // Build the chain; prefer the ROS-Industrial tool0 frame as flange.
  const prefix = opts.args?.prefix ?? '';
  const tipCandidates = [entry?.tipLink, `${prefix}tool0`, 'tool0', `${prefix}flange`, 'flange', `${prefix}tool_frame`, 'tool_frame'].filter((x): x is string => !!x);
  const tipLink = tipCandidates.find((n) => model!.links.has(n));
  const { chain } = chainFromURDF(model, { tipLink });
  const robot = new Robot(entry?.name ?? model.name, chain);
  robot.brand = entry?.brand ?? 'ROS2';
  robot.model = entry?.name ?? model.name;
  robot.postProcessor = entry?.postProcessor ?? 'ROS2';
  robot.params.source = 'online-library';
  robot.params.sourceUrl = url;
  if (entry) robot.params.libraryId = entry.id;
  if (entry) robot.params.payload = entry.payload;
  if (repoHint) robot.params.licence = repoHint.licence;
  robot.params.meshes = model.meshes;

  // Meshes
  const meshes = { loaded: [] as string[], failed: [] as string[] };
  const wantMeshes = opts.assets && (opts.meshes ?? true);
  if (wantMeshes) {
    const uris = new Set<string>();
    for (const l of chain.links) for (const v of l.visuals) if (v.mesh) uris.add(v.mesh);
    const list = [...uris].filter((u) => !opts.assets!.has(u));
    let done = 0;
    progress(`Downloading ${list.length} meshes`, 0, list.length);
    await mapLimit(list, opts.concurrency ?? 6, async (uri) => {
      const u = toUrl(uri);
      const name = uri.split('/').pop()!;
      try {
        if (!u) throw new Error('unresolvable');
        const ext = extOf(u);
        if (ext === 'stl') {
          const bytes = await fetchBytes(u);
          if (!bytes) throw new Error('404');
          opts.assets!.registerRaw(uri, 'stl', bytes, name);
        } else if (ext === 'dae') {
          const bytes = await fetchBytes(u);
          if (!bytes) throw new Error('404');
          opts.assets!.registerRaw(uri, 'dae', bytes, name, 1, 'm');
        } else if (ext === 'obj') {
          const bytes = await fetchBytes(u);
          if (!bytes) throw new Error('404');
          opts.assets!.registerMesh(uri, parseOBJ(new TextDecoder().decode(bytes), 1), name);
        } else throw new Error(`unsupported mesh format .${ext}`);
        meshes.loaded.push(uri);
      } catch (e) {
        meshes.failed.push(uri);
        warnings.push(`Mesh ${name}: ${(e as Error).message}`);
      }
      progress(`Downloaded ${name}`, ++done, list.length);
    });
  }
  const files = [...cache.entries()].filter(([, t]) => t).map(([u]) => u);
  return { robot, model, entry: entry ?? { id: model.name, brand: 'ROS2', name: model.name, repo: repoHint?.id ?? '', file: url, dof: robot.dof, payload: 0, reach: robot.reach, category: 'industrial', postProcessor: 'ROS2' }, url, files, meshes, warnings };
}

/**
 * Fetch meshes referenced by an already imported URDF (`package://` URIs) into the asset store.
 * Returns the URIs that could be resolved and downloaded.
 */
export async function fetchPackageMeshes(uris: string[], assets: AssetStore, opts: Pick<OnlineFetchOptions, 'fetchBytes' | 'onProgress' | 'concurrency'> = {}): Promise<{ loaded: string[]; failed: string[] }> {
  const fetchBytes = opts.fetchBytes ?? defaultFetchBytes;
  const loaded: string[] = [], failed: string[] = [];
  const list = uris.filter((u) => !assets.has(u));
  let done = 0;
  await mapLimit(list, opts.concurrency ?? 6, async (uri) => {
    const u = resolvePackageUri(uri);
    const name = uri.split('/').pop()!;
    try {
      if (!u) throw new Error('unknown package');
      const bytes = await fetchBytes(u);
      if (!bytes) throw new Error('404');
      const ext = extOf(u);
      if (ext === 'stl') assets.registerRaw(uri, 'stl', bytes, name);
      else if (ext === 'dae') assets.registerRaw(uri, 'dae', bytes, name, 1, 'm');
      else if (ext === 'obj') assets.registerMesh(uri, parseOBJ(new TextDecoder().decode(bytes), 1), name);
      else throw new Error('unsupported');
      loaded.push(uri);
    } catch {
      failed.push(uri);
    }
    opts.onProgress?.(name, ++done, list.length);
  });
  return { loaded, failed };
}

export { parseDAE };
