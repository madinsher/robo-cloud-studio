/**
 * URDF importer. `parseURDF` reads a URDF (xacro expanded with ROS 2 semantics, see ./xacro) into a validated model
 * (urdfdom rules: unique links / joints, existing parent / child links, one root, one parent per link) keeping every
 * link, joint (fixed / revolute / continuous / prismatic / floating / planar, mimic, limits, dynamics), inertial,
 * collision, named material (also when defined inside a visual) and the ros2_control / gazebo / transmission / webots
 * blocks. Units become the studio's mm / deg.
 *
 * Two ways into the station:
 *  - `importURDF` (trees, the default for ROS robots): URDF -> KinTree (interface I3) -> `buildStationItemsFromTree`:
 *    a MobileRobot when drive wheels or a floating base exist, one Robot per serial chain of actuated joints (mimic
 *    joints follow their master), every movable joint kept under its URDF name, the chain cut at `tool0`-like links.
 *  - `chainFromURDF` / `robotFromURDF` (legacy single serial chain, used by the online library): the longest movable
 *    path (or base..tipLink); side branches are drawn static at their zero pose.
 */
import { parseXML, childElements, childElement, nums, XNode, serializeXML } from './xml';
import { expandXacro, isXacro, XacroOptions } from './xacro';
import { ChainDef, JointDef, LinkDef, LinkVisual } from '../../core/kinematics/chain';
import { urdfToPose, identity, multiply, Mat4, Vec3, RAD, rotAxis, transl } from '../../core/math/pose';
import { Robot } from '../../core/items/robot';
import type { GeometryRef, Item, Camera } from '../../core/items/item';
import type { KinTree, KinLink, KinJoint, KinSensor, KinBuildOptions } from '../kintree';
import { buildStationItemsFromTree } from '../kintree';
import type { MobileRobot } from '../../mobile/items';

export interface URDFJoint {
  name: string;
  type: string;
  parent: string;
  child: string;
  origin: Mat4;
  axis: Vec3;
  lower: number;
  upper: number;
  velocity: number;
  effort: number;
  /** URDF <limit lower/upper> present (continuous / floating joints are unlimited). */
  limited?: boolean;
  /** q_this = multiplier * q_master + offset, in studio units of each joint (deg / mm). */
  mimic?: { joint: string; multiplier: number; offset: number };
  dynamics?: { damping?: number; friction?: number };
}

export interface URDFInertial { mass: number; origin: Mat4; inertia: [number, number, number, number, number, number] }

export interface URDFLink {
  name: string;
  visuals: LinkVisual[];
  collisions: LinkVisual[];
  mass: number;
  inertial?: URDFInertial;
}

/** Sensor declared for a link (gazebo <sensor>, webots <device>). Pose in the link frame, mm, studio optical frame for cameras. */
export interface URDFSensor extends KinSensor { link: string }

export interface URDFModel {
  name: string;
  links: Map<string, URDFLink>;
  joints: URDFJoint[];
  rootLink: string;
  /** All mesh URIs referenced. */
  meshes: string[];
  materials: Map<string, string>;
  /** Top-level elements other than link / joint / material (ros2_control, gazebo, transmission, webots ...). */
  extras: XNode[];
  sensors: URDFSensor[];
  /** Non-fatal problems (missing limits, unknown geometry ...). */
  warnings: string[];
  /** The expanded URDF text. */
  xml: string;
}

export class URDFError extends Error { constructor(m: string) { super(m); this.name = 'URDFError'; } }

export interface ParseURDFOptions extends XacroOptions {
  /** urdfdom validation (unique names, existing links, single root). Default true. */
  validate?: boolean;
}

const M2MM = 1000;
const conv = (type: string) => (type === 'prismatic' ? M2MM : RAD);

function parseGeometry(g: XNode | undefined, origin: Mat4, color: string | undefined, meshes: string[] | null, warn: (m: string) => void): LinkVisual | null {
  if (!g) return null;
  const mesh = childElement(g, 'mesh');
  if (mesh) {
    const scale = nums(mesh.attrs.scale, [1, 1, 1]) as Vec3;
    const uri = mesh.attrs.filename ?? '';
    if (meshes && !meshes.includes(uri)) meshes.push(uri);
    // URDF meshes are in metres; our scene is mm. Bake unit conversion into scale.
    return { mesh: uri, origin, color, scale: [scale[0] * M2MM, scale[1] * M2MM, scale[2] * M2MM] };
  }
  const box = childElement(g, 'box');
  if (box) {
    const s = nums(box.attrs.size, [0.1, 0.1, 0.1]);
    return { primitive: { kind: 'box', size: [s[0] * M2MM, s[1] * M2MM, s[2] * M2MM] }, origin, color };
  }
  const cyl = childElement(g, 'cylinder');
  if (cyl) return { primitive: { kind: 'cylinder', radius: +(cyl.attrs.radius ?? 0.05) * M2MM, length: +(cyl.attrs.length ?? 0.1) * M2MM }, origin, color };
  const sph = childElement(g, 'sphere');
  if (sph) return { primitive: { kind: 'sphere', radius: +(sph.attrs.radius ?? 0.05) * M2MM }, origin, color };
  const cap = childElement(g, 'capsule');
  if (cap) warn(`capsule geometry is not a URDF standard type — skipped`);
  return null;
}

function originOf(n: XNode | undefined): Mat4 {
  const o = n ? childElement(n, 'origin') : undefined;
  if (!o) return identity();
  const xyz = nums(o.attrs.xyz, [0, 0, 0]);
  const rpy = nums(o.attrs.rpy, [0, 0, 0]);
  return urdfToPose([xyz[0] * M2MM, xyz[1] * M2MM, xyz[2] * M2MM], rpy as Vec3);
}

function rgbaToCss(v: number[]): string {
  const h = (x: number) => Math.round(Math.max(0, Math.min(1, x)) * 255).toString(16).padStart(2, '0');
  return `#${h(v[0])}${h(v[1])}${h(v[2])}`;
}

/** Expand xacro (if any) with ROS 2 semantics and return the URDF text. */
export function expandURDFText(text: string, opts: XacroOptions = {}): string {
  return isXacro(text) ? expandXacro(text, opts).xml : text;
}

export function parseURDF(text: string, opts: ParseURDFOptions = {}): URDFModel {
  const xml = expandURDFText(text, opts);
  const root = parseXML(xml);
  if (root.name !== 'robot') throw new Error('Not a URDF file (missing <robot> root)');
  const warnings: string[] = [];
  const validate = opts.validate !== false;
  const fail = (m: string) => { if (validate) throw new URDFError(m); warnings.push(m); };
  const materials = new Map<string, string>();
  const defMaterial = (m: XNode) => {
    const c = childElement(m, 'color');
    if (c && m.attrs.name && !materials.has(m.attrs.name)) materials.set(m.attrs.name, rgbaToCss(nums(c.attrs.rgba, [0.7, 0.7, 0.7, 1])));
  };
  const topMaterials = new Set<string>();
  for (const m of childElements(root, 'material')) {
    if (m.attrs.name && topMaterials.has(m.attrs.name)) fail(`material '${m.attrs.name}' is not unique`);
    if (m.attrs.name) topMaterials.add(m.attrs.name);
    defMaterial(m);
  }
  // urdfdom: a named material defined inside a visual is added to the global map and can be reused elsewhere
  for (const l of childElements(root, 'link')) for (const v of childElements(l, 'visual')) { const m = childElement(v, 'material'); if (m) defMaterial(m); }
  const meshes: string[] = [];
  const links = new Map<string, URDFLink>();
  for (const l of childElements(root, 'link')) {
    const name = l.attrs.name;
    if (!name) { fail('link without a name'); continue; }
    if (links.has(name)) { fail(`link '${name}' is not unique`); continue; }
    const visuals: LinkVisual[] = [];
    const warn = (m: string) => warnings.push(`${name}: ${m}`);
    for (const v of childElements(l, 'visual')) {
      const mat = childElement(v, 'material');
      let color: string | undefined;
      if (mat) {
        const c = childElement(mat, 'color');
        color = c ? rgbaToCss(nums(c.attrs.rgba, [0.7, 0.7, 0.7, 1])) : materials.get(mat.attrs.name ?? '');
      }
      const vis = parseGeometry(childElement(v, 'geometry'), originOf(v), color, meshes, warn);
      if (vis) visuals.push(vis);
    }
    const collisions: LinkVisual[] = [];
    for (const c of childElements(l, 'collision')) {
      const vis = parseGeometry(childElement(c, 'geometry'), originOf(c), undefined, null, warn);
      if (vis) collisions.push(vis);
    }
    const inEl = childElement(l, 'inertial');
    let inertial: URDFInertial | undefined;
    if (inEl) {
      const it = childElement(inEl, 'inertia')?.attrs ?? {};
      inertial = { mass: +(childElement(inEl, 'mass')?.attrs.value ?? 0), origin: originOf(inEl), inertia: [+(it.ixx ?? 0), +(it.ixy ?? 0), +(it.ixz ?? 0), +(it.iyy ?? 0), +(it.iyz ?? 0), +(it.izz ?? 0)] };
    }
    links.set(name, { name, visuals, collisions, mass: inertial?.mass ?? 0, inertial });
  }
  const joints: URDFJoint[] = [];
  const jointNames = new Set<string>();
  const parentOf = new Map<string, string>();
  const rawJoints = childElements(root, 'joint');
  for (const j of rawJoints) {
    const name = j.attrs.name ?? '';
    if (jointNames.has(name)) { fail(`joint '${name}' is not unique`); continue; }
    jointNames.add(name);
    const limit = childElement(j, 'limit');
    const type = j.attrs.type ?? 'fixed';
    if (!['fixed', 'revolute', 'continuous', 'prismatic', 'floating', 'planar'].includes(type)) { fail(`joint '${name}' has unknown type '${type}'`); continue; }
    const isPrism = type === 'prismatic';
    const c = conv(type);
    const parent = childElement(j, 'parent')?.attrs.link ?? '';
    const child = childElement(j, 'child')?.attrs.link ?? '';
    if (!links.has(parent)) { fail(`parent link '${parent}' of joint '${name}' not found`); if (validate) continue; }
    if (!links.has(child)) { fail(`child link '${child}' of joint '${name}' not found`); if (validate) continue; }
    if (parentOf.has(child)) { fail(`link '${child}' has two parent joints ('${parentOf.get(child)}', '${name}')`); if (validate) continue; }
    parentOf.set(child, name);
    if ((type === 'revolute' || isPrism) && (!limit || limit.attrs.lower === undefined && limit.attrs.upper === undefined && limit.attrs.velocity === undefined)) warnings.push(`joint '${name}' is of type ${type} but does not specify limits`);
    const limited = (type === 'revolute' || isPrism) && !!limit && (limit.attrs.lower !== undefined || limit.attrs.upper !== undefined);
    let axis = nums(childElement(j, 'axis')?.attrs.xyz, [1, 0, 0]) as Vec3;
    const n = Math.hypot(axis[0], axis[1], axis[2]);
    if (type !== 'fixed' && type !== 'floating' && n < 1e-12) { fail(`joint '${name}' has a zero axis`); axis = [1, 0, 0]; }
    else if (n > 0) axis = [axis[0] / n, axis[1] / n, axis[2] / n];
    const dyn = childElement(j, 'dynamics');
    joints.push({
      name,
      type,
      parent,
      child,
      origin: originOf(j),
      axis,
      lower: limit?.attrs.lower !== undefined ? +limit.attrs.lower * c : type === 'continuous' ? -360 : isPrism ? 0 : -180,
      upper: limit?.attrs.upper !== undefined ? +limit.attrs.upper * c : type === 'continuous' ? 360 : isPrism ? 1000 : 180,
      velocity: limit?.attrs.velocity !== undefined ? +limit.attrs.velocity * c : isPrism ? 1000 : 180,
      effort: +(limit?.attrs.effort ?? 0),
      limited,
      dynamics: dyn ? { damping: dyn.attrs.damping !== undefined ? +dyn.attrs.damping : undefined, friction: dyn.attrs.friction !== undefined ? +dyn.attrs.friction : undefined } : undefined,
    });
  }
  // mimic (second pass: the multiplier converts between the master's and this joint's units)
  const byName = new Map(joints.map((x) => [x.name, x]));
  for (const j of rawJoints) {
    const jj = byName.get(j.attrs.name ?? '');
    const mimic = childElement(j, 'mimic');
    if (!jj || !mimic) continue;
    const master = byName.get(mimic.attrs.joint ?? '');
    if (!master) { fail(`joint '${jj.name}' mimics unknown joint '${mimic.attrs.joint}'`); continue; }
    jj.mimic = { joint: master.name, multiplier: +(mimic.attrs.multiplier ?? 1) * (conv(jj.type) / conv(master.type)), offset: +(mimic.attrs.offset ?? 0) * conv(jj.type) };
  }
  const children = new Set(joints.map((j) => j.child));
  const roots = [...links.keys()].filter((l) => !children.has(l));
  if (links.size && roots.length !== 1) fail(roots.length ? `Two root links found: [${roots.join(', ')}]` : 'No root link found (kinematic loop)');
  const rootLink = roots[0] ?? [...links.keys()][0] ?? '';
  const extras = root.children.filter((c) => c.type === 'element' && !['link', 'joint', 'material'].includes(c.name));
  const model: URDFModel = { name: root.attrs.name ?? 'robot', links, joints, rootLink, meshes, materials, extras, sensors: [], warnings, xml };
  model.sensors = parseSensors(model);
  return model;
}

/** gazebo <sensor> (camera / depth / ray / gpu_ray / imu / gps) and webots <device> declarations. */
function parseSensors(model: URDFModel): URDFSensor[] {
  const out: URDFSensor[] = [];
  // ROS link convention for cameras: +X forward, +Y left, +Z up. Studio optical: +Z view, +X right, +Y down.
  const toOptical = multiply(rotAxis([0, 0, 1], -Math.PI / 2), rotAxis([1, 0, 0], -Math.PI / 2));
  for (const g of model.extras.filter((e) => e.name === 'gazebo' && e.attrs.reference)) {
    const link = g.attrs.reference;
    if (!model.links.has(link)) continue;
    for (const s of childElements(g, 'sensor')) {
      const type = s.attrs.type ?? '';
      const pose = nums(childElement(s, 'pose')?.children.find((c) => c.type === 'text')?.text, [0, 0, 0, 0, 0, 0]);
      const T = urdfToPose([pose[0] * M2MM, pose[1] * M2MM, pose[2] * M2MM], [pose[3] ?? 0, pose[4] ?? 0, pose[5] ?? 0]);
      const name = s.attrs.name ?? `${link}_${type}`;
      const cam = childElement(s, 'camera');
      const num = (n: XNode | undefined, tag: string) => { const e = n ? childElement(n, tag) : undefined; const t = e?.children.find((c) => c.type === 'text')?.text; return t !== undefined ? +t : undefined; };
      if ((type === 'camera' || type === 'depth' || type === 'depth_camera' || type === 'rgbd_camera' || type === 'multicamera') && cam) {
        const img = childElement(cam, 'image'), clip = childElement(cam, 'clip');
        const hfov = num(cam, 'horizontal_fov');
        out.push({ link, kind: type === 'camera' || type === 'multicamera' ? 'camera' : 'depth', name, pose: multiply(T, toOptical), fov: hfov !== undefined ? hfov * RAD : undefined, fovAxis: 'horizontal', width: num(img, 'width'), height: num(img, 'height'), near: num(clip, 'near') !== undefined ? num(clip, 'near')! * M2MM : undefined, far: num(clip, 'far') !== undefined ? num(clip, 'far')! * M2MM : undefined, params: { sensorType: type } }); // depth / depth_camera: depth only; rgbd_camera: colour + depth (Gazebo)
      } else if (type === 'ray' || type === 'gpu_ray' || type === 'lidar' || type === 'gpu_lidar') {
        const ray = childElement(s, 'ray') ?? childElement(s, 'lidar');
        const scan = ray ? childElement(ray, 'scan') : undefined;
        const hz = scan ? childElement(scan, 'horizontal') : undefined, vt = scan ? childElement(scan, 'vertical') : undefined;
        const range = ray ? childElement(ray, 'range') : undefined;
        const minA = num(hz, 'min_angle') ?? -Math.PI, maxA = num(hz, 'max_angle') ?? Math.PI;
        const layers = num(vt, 'samples') ?? 1;
        out.push({ link, kind: layers > 1 ? 'lidar3d' : 'lidar2d', name, pose: multiply(T, toOptical), fov: (maxA - minA) * RAD, fovAxis: 'horizontal', width: num(hz, 'samples'), layers, near: num(range, 'min') !== undefined ? num(range, 'min')! * M2MM : undefined, far: num(range, 'max') !== undefined ? num(range, 'max')! * M2MM : undefined });
      } else if (type === 'imu') out.push({ link, kind: 'imu', name, pose: T });
      else if (type === 'gps' || type === 'navsat') out.push({ link, kind: 'gnss', name, pose: T });
    }
  }
  return out;
}

/** Tree FK of every link (mm), joint values by name in deg / mm; mimic joints follow their master. */
export function urdfLinkPoses(model: URDFModel, q: Record<string, number> = {}, base: Mat4 = identity()): Map<string, Mat4> {
  const value = (j: URDFJoint): number => (j.mimic ? j.mimic.multiplier * (q[j.mimic.joint] ?? 0) + j.mimic.offset : q[j.name] ?? 0);
  const byParent = new Map<string, URDFJoint[]>();
  for (const j of model.joints) { if (!byParent.has(j.parent)) byParent.set(j.parent, []); byParent.get(j.parent)!.push(j); }
  const out = new Map<string, Mat4>([[model.rootLink, base]]);
  const stack = [model.rootLink];
  while (stack.length) {
    const l = stack.pop()!;
    for (const j of byParent.get(l) ?? []) {
      if (out.has(j.child)) continue;
      let T = multiply(out.get(l)!, j.origin);
      const v = value(j);
      if (j.type === 'revolute' || j.type === 'continuous') T = multiply(T, rotAxis(j.axis, (v * Math.PI) / 180));
      else if (j.type === 'prismatic') T = multiply(T, transl(j.axis[0] * v, j.axis[1] * v, j.axis[2] * v));
      out.set(j.child, T);
      stack.push(j.child);
    }
  }
  return out;
}

function toGeometryRef(v: LinkVisual): GeometryRef {
  const g: GeometryRef = { origin: Array.from(v.origin) };
  if (v.mesh) g.mesh = v.mesh;
  if (v.primitive) g.primitive = v.primitive as GeometryRef['primitive'];
  if (v.color) g.color = v.color;
  if (v.scale) g.scale = [...v.scale] as [number, number, number];
  return g;
}

/** URDF model -> neutral kinematic tree (I3). Joint limits in deg / mm (none for continuous / floating / planar). */
export function urdfToKinTree(model: URDFModel): KinTree {
  const sensorsBy = new Map<string, URDFSensor[]>();
  for (const s of model.sensors) { if (!sensorsBy.has(s.link)) sensorsBy.set(s.link, []); sensorsBy.get(s.link)!.push(s); }
  const links: KinLink[] = [...model.links.values()].map((l) => {
    const kl: KinLink = { name: l.name, visuals: l.visuals.map(toGeometryRef) };
    if (l.collisions.length) kl.collisions = l.collisions.map(toGeometryRef);
    if (l.inertial) kl.inertial = { mass: l.inertial.mass, com: [l.inertial.origin[12], l.inertial.origin[13], l.inertial.origin[14]], inertia: [...l.inertial.inertia] };
    const ss = sensorsBy.get(l.name);
    if (ss) kl.sensors = ss.map(({ link: _l, ...s }) => s);
    return kl;
  });
  const joints: KinJoint[] = model.joints.map((j) => {
    const kj: KinJoint = { name: j.name, type: j.type as KinJoint['type'], parent: j.parent, child: j.child, origin: j.origin, axis: j.axis };
    if (j.type === 'revolute' || j.type === 'prismatic') { kj.lower = j.lower; kj.upper = j.upper; }
    if (j.type === 'revolute' || j.type === 'prismatic' || j.type === 'continuous') kj.velocity = j.velocity;
    if (j.mimic) kj.mimic = { ...j.mimic };
    return kj;
  });
  return { name: model.name, root: model.rootLink, links, joints, params: { source: 'urdf' } };
}

const TIP_RE = /(^|[_/])(tool0|flange|tool_frame|tcp|ee_link|end_effector|tool_link)$/;
/** Links a tree import should use as chain flanges: ROS-Industrial `tool0` & co, only where a chain ends there. */
export function defaultTipLinks(model: URDFModel): string[] {
  return [...model.links.keys()].filter((l) => TIP_RE.test(l));
}

export interface ImportURDFOptions extends ParseURDFOptions {
  /** Item name (defaults to the URDF robot name). */
  name?: string;
  /** Chain flanges (default: tool0 / flange / tcp ... links). */
  tipLinks?: string[];
  drive?: KinBuildOptions['drive'];
  assets?: KinBuildOptions['assets'];
  /** Rewrite mesh URIs (e.g. to asset ids registered by the package importer). */
  meshId?: (uri: string) => string;
}

export interface ImportURDFResult {
  root: Item;
  robots: Robot[];
  mobile?: MobileRobot;
  cameras: Camera[];
  model: URDFModel;
  tree: KinTree;
  report: string[];
}

/**
 * Import a URDF as station items through the kinematic tree (every movable joint kept, named as in the URDF).
 * The root item carries `params.robotDescription` (the expanded URDF) for /robot_description and lossless export.
 */
export function importURDF(text: string, opts: ImportURDFOptions = {}): ImportURDFResult {
  const model = parseURDF(text, opts);
  if (opts.meshId) {
    const map = opts.meshId;
    for (const l of model.links.values()) for (const v of [...l.visuals, ...l.collisions]) if (v.mesh) v.mesh = map(v.mesh);
    model.meshes = model.meshes.map(map);
  }
  const tree = urdfToKinTree(model);
  const res = buildStationItemsFromTree(tree, { name: opts.name ?? model.name, tipLinks: opts.tipLinks ?? defaultTipLinks(model), drive: opts.drive, assets: opts.assets });
  for (const r of res.robots) {
    r.brand = 'ROS2';
    r.postProcessor = 'ROS2';
    r.params.source = 'urdf';
    r.params.meshes = model.meshes;
  }
  res.root.params.source = 'urdf';
  res.root.params.urdfName = model.name;
  res.root.params.robotDescription = model.xml;
  res.root.params.urdfJoints = model.joints.filter((j) => j.type !== 'fixed').map((j) => j.name);
  const report = [...model.warnings, ...res.report];
  return { root: res.root, robots: res.robots, mobile: res.mobile, cameras: res.cameras, model, tree, report };
}

export interface ChainBuildOptions {
  /** Tip link name; defaults to the deepest link along the longest path of movable joints. */
  tipLink?: string;
  /** Base link name; defaults to the URDF root. */
  baseLink?: string;
}

const movable = (j: URDFJoint) => j.type === 'revolute' || j.type === 'continuous' || j.type === 'prismatic';

/**
 * Convert a URDF model into a serial ChainDef from base to tip. Side branches (e.g. the second gripper finger, a
 * mimic parallelogram, a second arm) are folded into their parent link as static visuals at their zero pose.
 * Floating / planar joints are not actuated (identity motion).
 */
export function chainFromURDF(model: URDFModel, opts: ChainBuildOptions = {}): { chain: ChainDef; sideBranches: URDFJoint[] } {
  const byParent = new Map<string, URDFJoint[]>();
  for (const j of model.joints) {
    if (!byParent.has(j.parent)) byParent.set(j.parent, []);
    byParent.get(j.parent)!.push(j);
  }
  const base = opts.baseLink ?? model.rootLink;
  const pathTo = new Map<string, URDFJoint[]>();
  const dfs = (link: string, path: URDFJoint[]) => {
    pathTo.set(link, path);
    for (const j of byParent.get(link) ?? []) dfs(j.child, [...path, j]);
  };
  dfs(base, []);
  let tip = opts.tipLink;
  if (!tip) {
    let best = -1;
    for (const [link, path] of pathTo) {
      const score = path.filter((j) => movable(j) && !j.mimic).length * 1000 + path.length;
      if (score > best) { best = score; tip = link; }
    }
  }
  const mainPath = pathTo.get(tip!) ?? [];
  const mainLinks = new Set([base, ...mainPath.map((j) => j.child)]);

  const joints: JointDef[] = [];
  const links: LinkDef[] = [];
  const sideBranches: URDFJoint[] = [];

  const collectStatic = (linkName: string, T: Mat4, into: LinkVisual[]) => {
    const l = model.links.get(linkName);
    if (l) for (const v of l.visuals) into.push({ ...v, origin: multiply(T, v.origin) });
    for (const j of byParent.get(linkName) ?? []) {
      if (mainLinks.has(j.child)) continue;
      if (movable(j) && !j.mimic) sideBranches.push(j);
      collectStatic(j.child, multiply(T, j.origin), into);
    }
  };

  const linkDef = (name: string): LinkDef => {
    const visuals: LinkVisual[] = [];
    collectStatic(name, identity(), visuals);
    return { name, visuals };
  };
  links.push(linkDef(base));
  const onChain = new Set(mainPath.map((j) => j.name));
  for (const j of mainPath) {
    if (!movable(j)) {
      // fixed / floating / planar: zero-motion joint so link frames stay correct
      joints.push({ name: j.name, type: 'fixed', origin: j.origin, axis: [0, 0, 1], lower: 0, upper: 0 });
    } else {
      const mimic = j.mimic && onChain.has(j.mimic.joint) ? j.mimic : undefined;
      joints.push({
        name: j.name,
        type: j.type as JointDef['type'],
        origin: j.origin,
        axis: j.axis,
        lower: j.lower,
        upper: j.upper,
        maxVelocity: j.velocity,
        mimic,
        home: Math.max(j.lower, Math.min(j.upper, 0)),
      });
    }
    links.push(linkDef(j.child));
  }
  const chain: ChainDef = { name: model.name, joints, links, flange: identity(), dof: joints.filter((j) => j.type !== 'fixed' && !j.mimic).length };
  return { chain, sideBranches };
}

export function robotFromURDF(text: string, xacro: XacroOptions = {}, opts: ChainBuildOptions = {}): Robot {
  const model = parseURDF(text, xacro);
  const { chain } = chainFromURDF(model, opts);
  const r = new Robot(model.name, chain);
  r.brand = 'ROS2';
  r.postProcessor = 'ROS2';
  r.params.source = 'urdf';
  r.params.meshes = model.meshes;
  return r;
}

export { serializeXML };
