/**
 * Kinematic tree (interface I3): a neutral description of a branched robot (Webots Robot node, URDF/SDF model) and its
 * decomposition into studio items — a MobileRobot when drive wheels (or a floating base) are detected, one Robot per
 * maximal serial chain of actuated joints (mimic joints follow their master), fixed links folded into the geometry of the
 * item they move with, cameras/sensors attached to the owning item. Joint names are kept (Webots motor / URDF joint names).
 *
 * Conventions: lengths in mm, joint limits in deg (revolute) or mm (prismatic), velocities in deg/s or mm/s.
 * KinJoint.origin is the transform from the parent link frame to the joint frame at zero joint value; the child link
 * frame is joint frame * motion(axis, q) (URDF semantics). Sensor poses of cameras/depth/lidar follow the studio optical
 * convention (+Z view axis, +X image right, +Y image down); importers convert their own convention.
 */
import { GeometryRef, Item, SceneObject, Camera, ParamValue } from '../core/items/item';
import { Robot } from '../core/items/robot';
import { ChainDef, JointDef, LinkDef, LinkVisual } from '../core/kinematics/chain';
import { MobileRobot, MobileKinematics } from '../mobile/items';
import type { AssetStore } from '../scene/assets';
import { Mat4, Vec3, identity, multiply, invert, transformPoint, transformDir, fromArray, rotAxis, transl, normalize, DEG, RAD } from '../core/math/pose';

export type KinJointType = 'revolute' | 'continuous' | 'prismatic' | 'fixed' | 'floating' | 'planar';

export interface KinSensor {
  kind: 'camera' | 'depth' | 'lidar2d' | 'lidar3d' | 'gnss' | 'imu' | 'gyro' | 'accelerometer' | 'compass' | 'distance' | 'touch' | 'other';
  name: string;
  /** Pose in the link frame (mm). Camera/depth/lidar: studio optical convention (+Z view, +X right, +Y down). */
  pose: Mat4;
  /** Field of view (deg) about `fovAxis` (cameras) or horizontal sweep (lidar). */
  fov?: number;
  fovAxis?: 'vertical' | 'horizontal';
  width?: number;
  height?: number;
  /** Near / far clipping or min / max range (mm). */
  near?: number;
  far?: number;
  /** Lidar layers. */
  layers?: number;
  params?: Record<string, ParamValue>;
}

export interface KinInertial { mass: number; com?: Vec3; inertia?: number[] }

export interface KinLink {
  name: string;
  /** Visual geometry in the link frame (mm). */
  visuals: GeometryRef[];
  /** Collision geometry in the link frame (Webots boundingObject, URDF collision). */
  collisions?: GeometryRef[];
  inertial?: KinInertial;
  sensors?: KinSensor[];
  params?: Record<string, ParamValue>;
}

export interface KinJoint {
  name: string;
  type: KinJointType;
  parent: string;
  child: string;
  /** Parent link frame -> joint frame at zero (mm). */
  origin: Mat4;
  /** Motion axis in the joint frame. */
  axis: Vec3;
  lower?: number;
  upper?: number;
  velocity?: number;
  mimic?: { joint: string; multiplier: number; offset: number };
  /** Initial joint value (deg|mm), e.g. Webots HingeJointParameters.position. */
  position?: number;
  params?: Record<string, ParamValue>;
}

export interface KinTree {
  name: string;
  root: string;
  links: KinLink[];
  joints: KinJoint[];
  params?: Record<string, ParamValue>;
}

export interface KinDriveOverride extends Partial<MobileKinematics> {
  /** Explicit wheel / steering joint names (skip the geometric detection). */
  wheels?: string[];
  steer?: string[];
}

export interface KinBuildOptions {
  /** Item name (defaults to tree.name). */
  name?: string;
  /** Used to measure mesh extents (wheel radius from a mesh, footprint). */
  assets?: AssetStore;
  /** Links that must be the flange of their chain (chains are cut there). */
  tipLinks?: string[];
  /**
   * 'auto' (default): MobileRobot when >= 2 drive wheels or a floating/planar base is found; 'none': never;
   * 'kinematic': always a MobileRobot (pose-played platforms without wheel joints); an object forces a MobileRobot and
   * overrides the detected kinematics.
   */
  drive?: 'auto' | 'none' | 'kinematic' | KinDriveOverride;
}

export interface KinBuildResult {
  root: Item;
  mobile?: MobileRobot;
  robots: Robot[];
  cameras: Camera[];
  report: string[];
}

const isMovable = (j: KinJoint) => j.type === 'revolute' || j.type === 'continuous' || j.type === 'prismatic';
const unlimited = (j: KinJoint) => j.type === 'continuous' || j.lower === undefined || j.upper === undefined || j.upper - j.lower >= 359.9;

function motion(j: KinJoint, v: number): Mat4 {
  if (!isMovable(j) || !v) return identity();
  return j.type === 'prismatic' ? transl(j.axis[0] * v, j.axis[1] * v, j.axis[2] * v) : rotAxis(j.axis, v * DEG);
}

function geomOrigin(g: GeometryRef): Mat4 {
  return g.origin ? fromArray(g.origin) : identity();
}

/** Re-express a geometry in another frame: T maps the geometry's current frame into the new one. */
export function moveGeometry(g: GeometryRef, T: Mat4): GeometryRef {
  return { ...g, origin: Array.from(multiply(T, geomOrigin(g))) };
}

/** Axis-aligned bounds (mm) of geometry refs in their common frame; meshes need the asset store. */
export function geometryBounds(refs: GeometryRef[], assets?: AssetStore): { min: Vec3; max: Vec3 } | null {
  const min: Vec3 = [Infinity, Infinity, Infinity], max: Vec3 = [-Infinity, -Infinity, -Infinity];
  const add = (T: Mat4, lo: Vec3, hi: Vec3) => {
    for (let c = 0; c < 8; c++) {
      const p = transformPoint(T, [c & 1 ? hi[0] : lo[0], c & 2 ? hi[1] : lo[1], c & 4 ? hi[2] : lo[2]]);
      for (let a = 0; a < 3; a++) { if (p[a] < min[a]) min[a] = p[a]; if (p[a] > max[a]) max[a] = p[a]; }
    }
  };
  for (const g of refs) {
    const T = geomOrigin(g);
    const s = g.scale ?? [1, 1, 1];
    if (g.primitive) {
      const p = g.primitive;
      const h: Vec3 = p.kind === 'box' ? [p.size[0] / 2, p.size[1] / 2, p.size[2] / 2] : p.kind === 'sphere' ? [p.radius, p.radius, p.radius]
        : p.kind === 'plane' ? [p.size[0] / 2, p.size[1] / 2, 0] : [p.radius, p.radius, p.length / 2];
      add(T, [-h[0] * s[0], -h[1] * s[1], -h[2] * s[2]], [h[0] * s[0], h[1] * s[1], h[2] * s[2]]);
    } else if (g.mesh && assets) {
      const m = assets.resolve ? assets.resolve(g.mesh)?.mesh : assets.get(g.mesh)?.mesh;
      if (!m || !isFinite(m.min[0])) continue;
      add(T, [m.min[0] * s[0], m.min[1] * s[1], m.min[2] * s[2]], [m.max[0] * s[0], m.max[1] * s[1], m.max[2] * s[2]]);
    }
  }
  return isFinite(min[0]) ? { min, max } : null;
}

function toLinkVisual(g: GeometryRef): LinkVisual {
  const v: LinkVisual & { opacity?: number; pbr?: GeometryRef['pbr'] } = { origin: geomOrigin(g) };
  if (g.mesh) v.mesh = g.mesh;
  if (g.primitive) v.primitive = g.primitive as LinkVisual['primitive'];
  if (g.color) v.color = g.color;
  if (g.scale) v.scale = [...g.scale] as Vec3;
  if (g.opacity !== undefined && g.opacity < 1) v.opacity = g.opacity;
  if (g.pbr) v.pbr = { ...g.pbr };
  return v;
}

function setColliders(item: Item, refs: GeometryRef[]): void {
  (item as unknown as { colliders?: GeometryRef[] }).colliders = refs;
}

interface Collected { visuals: GeometryRef[]; collisions: GeometryRef[]; sensors: KinSensor[]; mass: number; hasCollisions: boolean }
interface ChainBuild {
  joints: KinJoint[]; parent: ChainBuild | null; flange: Mat4; name: string; links: Collected[]; robot?: Robot; sideMimics: KinJoint[];
  /** A whole mechanism serialised as one chain (cross-chain mimics): its joint definitions and link names, links aligned with `links`. */
  tour?: { defs: JointDef[]; linkNames: string[]; flangeLink?: number };
}

/** Wheel radius (mm) of a wheel link about its joint axis (link frame = joint frame): collision, primitive or mesh extent. */
function wheelRadius(link: Collected, axis: Vec3, assets?: AssetStore): number | undefined {
  const ax = normalize(axis);
  for (const src of [link.collisions, link.visuals]) {
    for (const g of src) {
      const p = g.primitive;
      if (!p) continue;
      const T = geomOrigin(g);
      if (p.kind === 'sphere') return p.radius * (g.scale?.[0] ?? 1);
      if (p.kind === 'cylinder') {
        const z = transformDir(T, [0, 0, 1]);
        if (Math.abs(z[0] * ax[0] + z[1] * ax[1] + z[2] * ax[2]) > 0.9) return p.radius * (g.scale?.[0] ?? 1);
      }
    }
  }
  const b = geometryBounds(link.visuals.filter((g) => (g.opacity ?? 1) > 0.01), assets) ?? geometryBounds(link.collisions, assets);
  if (!b) return undefined;
  const ext: number[] = [0, 1, 2].map((a) => Math.max(Math.abs(b.min[a]), Math.abs(b.max[a])));
  const k = [0, 1, 2].sort((a, c) => Math.abs(ax[c]) - Math.abs(ax[a]))[0];
  const perp = [0, 1, 2].filter((a) => a !== k);
  return Math.max(ext[perp[0]], ext[perp[1]]);
}

function commonPrefix(names: string[]): string {
  if (!names.length) return '';
  let p = names[0];
  for (const n of names) while (!n.startsWith(p)) p = p.slice(0, -1);
  return p;
}

function chainSuffix(joints: KinJoint[], index: number): string {
  let p = commonPrefix(joints.map((j) => j.name)).replace(/(_joint|joint|_)+$/i, '').replace(/[_\-\s]+$/, '');
  if (joints.length === 1) p = joints[0].name.replace(/_joint$/i, '');
  if (/^(l|left)$/i.test(p)) return 'left_arm';
  if (/^(r|right)$/i.test(p)) return 'right_arm';
  if (p.length >= 2) return p;
  const last = joints[joints.length - 1].child.replace(/[^\w-]+/g, '_');
  return last || `chain${index + 1}`;
}

/**
 * Decompose a kinematic tree into station items (see the module comment). Never throws on structural problems: issues
 * (dangling joints, cross-chain mimics, sensors on intermediate links) are listed in `report`.
 */
export function buildStationItemsFromTree(tree: KinTree, opts: KinBuildOptions = {}): KinBuildResult {
  const report: string[] = [];
  const name = opts.name ?? tree.name;
  const links = new Map<string, KinLink>();
  for (const l of tree.links) links.set(l.name, l);
  const childJoints = new Map<string, KinJoint[]>();
  const parentJoint = new Map<string, KinJoint>();
  for (const j of tree.joints) {
    if (parentJoint.has(j.child)) { report.push(`${name}: link ${j.child} has two parent joints; ${j.name} ignored`); continue; }
    if (!links.has(j.parent)) { links.set(j.parent, { name: j.parent, visuals: [] }); report.push(`${name}: joint ${j.name} references missing link ${j.parent}`); }
    if (!links.has(j.child)) { links.set(j.child, { name: j.child, visuals: [] }); report.push(`${name}: joint ${j.name} references missing link ${j.child}`); }
    parentJoint.set(j.child, j);
    if (!childJoints.has(j.parent)) childJoints.set(j.parent, []);
    childJoints.get(j.parent)!.push(j);
  }
  if (!links.has(tree.root)) links.set(tree.root, { name: tree.root, visuals: [] });
  const jointByName = new Map(tree.joints.map((j) => [j.name, j]));

  // Floating / planar base (URDF world -> base_link floating, SDF free model): the child becomes the mobile base.
  let base = tree.root;
  let floating: KinJoint | null = null;
  let baseOffset = identity();
  for (;;) {
    const cj = childJoints.get(base) ?? [];
    const lk = links.get(base)!;
    if (cj.length === 1 && (cj[0].type === 'floating' || cj[0].type === 'planar') && !lk.visuals.length) { floating = cj[0]; baseOffset = multiply(baseOffset, cj[0].origin); base = cj[0].child; continue; }
    break;
  }
  for (const j of tree.joints) if ((j.type === 'floating' || j.type === 'planar') && j !== floating) report.push(`${name}: ${j.type} joint ${j.name} inside the tree is treated as fixed`);
  const initValue = (j: KinJoint) => (isMovable(j) ? j.position ?? 0 : 0);

  // Zero/initial configuration pose of each link relative to the base link.
  const T0 = new Map<string, Mat4>();
  const walk0 = (link: string, T: Mat4, seen: Set<string>) => {
    if (seen.has(link)) { report.push(`${name}: kinematic loop at ${link}`); return; }
    seen.add(link);
    T0.set(link, T);
    for (const j of childJoints.get(link) ?? []) walk0(j.child, multiply(multiply(T, j.origin), motion(j, initValue(j))), seen);
  };
  walk0(base, identity(), new Set());
  for (const l of links.keys()) if (!T0.has(l) && l !== tree.root && !(floating && l === tree.root)) report.push(`${name}: link ${l} is not connected to ${base}`);
  const fixedPath = (a: string, b: string) => multiply(invert(T0.get(a) ?? identity()), T0.get(b) ?? identity());
  const movableBelow = (link: string): KinJoint[] => {
    const out: KinJoint[] = [];
    const rec = (l: string) => { for (const j of childJoints.get(l) ?? []) { if (isMovable(j)) out.push(j); rec(j.child); } };
    rec(link);
    return out;
  };
  const movableAbove = (link: string): KinJoint[] => {
    const out: KinJoint[] = [];
    let l = link;
    while (l !== base) { const pj = parentJoint.get(l); if (!pj) break; if (isMovable(pj)) out.push(pj); l = pj.parent; }
    return out;
  };

  // ---- drive wheels / steering ------------------------------------------------------------------------------------
  const driveOpt = opts.drive ?? 'auto';
  const override: KinDriveOverride | null = typeof driveOpt === 'object' ? driveOpt : null;
  const collectStatic = (link: string, T: Mat4, into: Collected, folded: Set<KinJoint>) => {
    const lk = links.get(link);
    if (lk) {
      for (const v of lk.visuals) into.visuals.push(moveGeometry(v, T));
      if (lk.collisions) { into.hasCollisions = true; for (const c of lk.collisions) into.collisions.push(moveGeometry(c, T)); }
      for (const s of lk.sensors ?? []) into.sensors.push({ ...s, pose: multiply(T, s.pose) });
      into.mass += lk.inertial?.mass ?? 0;
    }
    for (const j of childJoints.get(link) ?? []) if (!isMovable(j) || folded.has(j)) collectStatic(j.child, multiply(multiply(T, j.origin), motion(j, initValue(j))), into, folded);
  };
  const newCollected = (): Collected => ({ visuals: [], collisions: [], sensors: [], mass: 0, hasCollisions: false });

  let wheels: KinJoint[] = [];
  let steers: KinJoint[] = [];
  const radius = new Map<KinJoint, number>();
  /**
   * Wheel centre in the base frame: the point of the wheel axis closest to the centre of the wheel geometry. The joint
   * origin is only a point of the axis - a Webots HingeJoint anchor (E-puck: anchor 0 0 0.02 for both wheels, the
   * offset ±26 mm lies in the endPoint translation), a URDF joint usually at the wheel centre.
   */
  const wheelCenter = (w: KinJoint) => {
    const Tj = multiply(T0.get(w.parent) ?? identity(), w.origin);
    const c = newCollected(); collectStatic(w.child, identity(), c, new Set());
    const b = geometryBounds(c.visuals.filter((g) => (g.opacity ?? 1) > 0.01), opts.assets) ?? geometryBounds(c.collisions, opts.assets);
    if (!b) return transformPoint(Tj, [0, 0, 0]);
    const a = normalize(w.axis); const mid: Vec3 = [(b.min[0] + b.max[0]) / 2, (b.min[1] + b.max[1]) / 2, (b.min[2] + b.max[2]) / 2];
    const s = mid[0] * a[0] + mid[1] * a[1] + mid[2] * a[2];
    return transformPoint(Tj, [a[0] * s, a[1] * s, a[2] * s]);
  };
  if (driveOpt !== 'none') {
    if (override?.wheels?.length) {
      wheels = override.wheels.map((n) => jointByName.get(n)).filter((j): j is KinJoint => !!j);
      steers = (override.steer ?? []).map((n) => jointByName.get(n)).filter((j): j is KinJoint => !!j);
      for (const w of wheels) { const c = newCollected(); collectStatic(w.child, identity(), c, new Set()); const r = wheelRadius(c, w.axis, opts.assets); if (r) radius.set(w, r); }
    } else {
      const cands: Array<{ w: KinJoint; s?: KinJoint; r: number; bottom: number }> = [];
      for (const w of tree.joints) {
        if (!isMovable(w) || w.type === 'prismatic' || w.mimic || !unlimited(w) || movableBelow(w.child).length) continue;
        const TB = multiply(T0.get(w.parent) ?? identity(), w.origin);
        if (!T0.has(w.parent)) continue;
        const axB = normalize(transformDir(TB, w.axis));
        if (Math.abs(axB[2]) > 0.35) continue;
        const above = movableAbove(w.parent);
        let s: KinJoint | undefined;
        if (above.length > 1) continue;
        if (above.length === 1) {
          s = above[0];
          const sAx = normalize(transformDir(multiply(T0.get(s.parent) ?? identity(), s.origin), s.axis));
          if (s.type === 'prismatic' || s.mimic || Math.abs(sAx[2]) < 0.9 || movableBelow(s.child).length !== 1) continue;
        }
        const c = newCollected();
        collectStatic(w.child, identity(), c, new Set());
        const r = wheelRadius(c, w.axis, opts.assets);
        if (!r) continue;
        cands.push({ w, s, r, bottom: TB[14] - r });
      }
      const ground = Math.min(...cands.map((c) => c.bottom));
      for (const c of cands) {
        if (c.bottom - ground > Math.max(0.35 * c.r, 25)) continue;
        if (c.s && unlimited(c.s)) { report.push(`${name}: ${c.w.name} under the free swivel ${c.s.name} is a caster, not a drive wheel`); continue; }
        wheels.push(c.w); radius.set(c.w, c.r);
        if (c.s && !steers.includes(c.s)) steers.push(c.s);
      }
      if (wheels.length < 2 && !override) { wheels = []; steers = []; }
    }
  }
  const makeMobile = driveOpt !== 'none' && (wheels.length >= 2 || !!floating || driveOpt === 'kinematic' || !!override);
  const folded = new Set<KinJoint>(makeMobile ? [...wheels, ...steers] : []);

  // ---- serial chains ---------------------------------------------------------------------------------------------
  const tips = new Set(opts.tipLinks ?? []);
  const nextMovables = (link: string): KinJoint[] => {
    const out: KinJoint[] = [];
    const rec = (l: string) => { for (const j of childJoints.get(l) ?? []) { if (folded.has(j)) continue; if (isMovable(j)) out.push(j); else rec(j.child); } };
    rec(link);
    return out;
  };
  const fixedReach = (link: string): string[] => {
    const out = [link];
    for (const j of childJoints.get(link) ?? []) if (!isMovable(j)) out.push(...fixedReach(j.child));
    return out;
  };
  const chains: ChainBuild[] = [];
  const startChain = (j0: KinJoint, parent: ChainBuild | null) => {
    const ch: ChainBuild = { joints: [j0], parent, flange: identity(), name: '', links: [], sideMimics: [] };
    chains.push(ch);
    let cur = j0;
    let rest: KinJoint[] = [];
    for (;;) {
      const tip = fixedReach(cur.child).find((l) => tips.has(l));
      const cands = nextMovables(cur.child);
      if (tip) { ch.flange = fixedPath(cur.child, tip); rest = cands; break; }
      if (cands.length === 1) { ch.joints.push(cands[0]); cur = cands[0]; continue; }
      const indep = cands.filter((c) => !c.mimic);
      if (indep.length === 1 && cands.every((c) => c === indep[0] || !movableBelow(c.child).some((d) => !d.mimic))) {
        for (const c of cands) if (c !== indep[0]) { ch.sideMimics.push(c); for (const d of [c, ...movableBelow(c.child)]) folded.add(d); }
        ch.joints.push(indep[0]); cur = indep[0];
        continue;
      }
      rest = cands;
      break;
    }
    for (const c of rest) startChain(c, ch);
  };
  for (const j of nextMovables(base)) startChain(j, null);

  // ---- mechanisms with cross-chain mimics: one serial chain (Euler tour) ---------------------------------------
  // A linkage whose mimic joints follow masters in other branches (palletizer parallelograms: URDF trees with mimics
  // that close the loops) cannot be split into independent serial chains. The whole group (a top-level chain and all
  // chains below it) becomes one Robot whose chain visits every side branch and comes back from it with compensating
  // joints (the joint's motion undone by a mimic with the opposite multiplier, then the inverse origin), before it
  // continues along the main path to the tip. Every link keeps its tree pose for any joint values, the mimics follow
  // their masters, and the DOF are the independent joints of the mechanism.
  const chainOf = new Map<KinJoint, ChainBuild>();
  for (const ch of chains) { for (const j of ch.joints) chainOf.set(j, ch); for (const m of ch.sideMimics) for (const d of [m, ...movableBelow(m.child)]) chainOf.set(d, ch); }
  const rootOf = (ch: ChainBuild): ChainBuild => (ch.parent ? rootOf(ch.parent) : ch);
  for (const top of chains.filter((c) => !c.parent)) {
    const group = chains.filter((c) => rootOf(c) === top);
    const inGroup = new Set(group.flatMap((c) => [...c.joints, ...c.sideMimics.flatMap((m) => [m, ...movableBelow(m.child)])]));
    const cross = [...inGroup].some((j) => { if (!j.mimic) return false; const m = jointByName.get(j.mimic.joint); return !!m && inGroup.has(m) && chainOf.get(m) !== chainOf.get(j); });
    if (!cross) continue;
    for (const j of inGroup) folded.delete(j); // side mimics are toured as moving joints
    const j0 = top.joints[0];
    // main path: to a tip link, else the path with the most independent joints
    const best = (j: KinJoint): KinJoint[] => {
      if (fixedReach(j.child).some((l) => tips.has(l))) return [j];
      let pick: KinJoint[] = [];
      for (const k of nextMovables(j.child)) {
        const p = best(k); const score = (x: KinJoint[]) => (x.some((y) => fixedReach(y.child).some((l) => tips.has(l))) ? 1e6 : 0) + x.filter((y) => !y.mimic).length * 1000 + x.length;
        if (!pick.length || score(p) > score(pick)) pick = p;
      }
      return [j, ...pick];
    };
    const path = best(j0);
    // a mimic resolved to the independent joint that drives it (mimic of a mimic composed)
    const resolveMimic = (j: KinJoint, depth = 0): { joint: string; multiplier: number; offset: number } | null => {
      if (!j.mimic) return null;
      const m = jointByName.get(j.mimic.joint);
      if (!m || !inGroup.has(m) || depth > 16) return null;
      const up = resolveMimic(m, depth + 1);
      return up ? { joint: up.joint, multiplier: j.mimic.multiplier * up.multiplier, offset: j.mimic.multiplier * up.offset + j.mimic.offset } : { ...j.mimic };
    };
    const defs: JointDef[] = []; const links: Collected[] = []; const linkNames: string[] = [];
    const jointDef = (j: KinJoint, origin: Mat4): JointDef => {
      const prism = j.type === 'prismatic';
      const cont = j.type === 'continuous' || (!prism && (j.lower === undefined || j.upper === undefined));
      const lower = j.lower ?? (prism ? -1000 : cont ? -360 : -180), upper = j.upper ?? (prism ? 1000 : cont ? 360 : 180);
      const p0 = j.position ?? 0, home = cont && !prism ? p0 - 360 * Math.round(p0 / 360) : Math.max(lower, Math.min(upper, p0));
      const def: JointDef = { name: j.name, type: prism ? 'prismatic' : cont ? 'continuous' : 'revolute', origin, axis: normalize(j.axis), lower, upper, home };
      if (j.velocity !== undefined) def.maxVelocity = j.velocity;
      if (j.mimic) { const r = resolveMimic(j); if (r) def.mimic = r; else report.push(`${name}: mimic joint ${j.name} follows ${j.mimic.joint} outside its mechanism; imported as independent`); }
      return def;
    };
    const enter = (cur: string, j: KinJoint): JointDef => {
      const def = jointDef(j, multiply(fixedPath(cur, j.parent), j.origin));
      const c = newCollected(); collectStatic(j.child, identity(), c, folded);
      defs.push(def); links.push(c); linkNames.push(j.child);
      return def;
    };
    const back = (j: KinJoint, def: JointDef) => {
      const undo = def.mimic ? { joint: def.mimic.joint, multiplier: -def.mimic.multiplier, offset: -def.mimic.offset } : { joint: def.name, multiplier: -1, offset: 0 };
      defs.push({ name: `${j.name}__return`, type: def.type === 'prismatic' ? 'prismatic' : 'continuous', origin: identity(), axis: def.axis, post: invert(def.origin), lower: -1e6, upper: 1e6, mimic: undo });
      links.push(newCollected()); linkNames.push(`${j.child}__return`);
    };
    const side = (cur: string, j: KinJoint) => { const def = enter(cur, j); for (const k of nextMovables(j.child)) side(j.child, k); back(j, def); };
    path.forEach((j, k) => {
      const cur = k === 0 ? j.parent : path[k - 1].child;
      if (k > 0) for (const s of nextMovables(cur)) if (s !== j) side(cur, s);
      enter(cur, j);
    });
    const last = path[path.length - 1];
    const endLink = defs.length; // chain link of the main path's end (links[0] is the base)
    for (const s of nextMovables(last.child)) side(last.child, s);
    const tip = fixedReach(last.child).find((l) => tips.has(l));
    const merged: ChainBuild = { joints: path, parent: null, flange: tip ? fixedPath(last.child, tip) : identity(), name: '', links, sideMimics: [], tour: { defs, linkNames, ...(defs.length > endLink ? { flangeLink: endLink } : {}) } };
    const at = chains.indexOf(top);
    for (const c of group) chains.splice(chains.indexOf(c), 1);
    chains.splice(Math.min(at, chains.length), 0, merged);
    report.push(`${name}: mechanism ${j0.name}… with cross-chain mimics imported as one chain of ${defs.filter((d) => !d.mimic).length} DOF (${defs.filter((d) => d.mimic && !d.name.endsWith('__return')).length} mimic joints follow their masters)`);
  }
  for (const ch of chains) for (const m of ch.sideMimics) report.push(`${name}: mimic side branch ${m.name} (follows ${m.mimic?.joint}) is drawn static at its initial value`);

  // ---- items -----------------------------------------------------------------------------------------------------
  const baseC = newCollected();
  collectStatic(base, identity(), baseC, folded);
  const cameras: Camera[] = [];
  const robots: Robot[] = [];
  const allVisuals: GeometryRef[] = [...baseC.visuals];

  chains.forEach((ch, idx) => {
    ch.name = chains.length === 1 && !makeMobile ? name : `${name}_${chainSuffix(ch.joints, idx)}`;
    if (ch.tour) {
      const { defs, linkNames } = ch.tour;
      const linkDefs: LinkDef[] = [{ name: ch.joints[0].parent, visuals: [] }, ...defs.map((_, k) => {
        const ld: LinkDef & { collisions?: GeometryRef[] } = { name: linkNames[k], visuals: ch.links[k].visuals.map(toLinkVisual) };
        if (ch.links[k].hasCollisions) ld.collisions = ch.links[k].collisions;
        return ld;
      })];
      const chain: ChainDef = { name: ch.name, joints: defs, links: linkDefs, flange: ch.flange, dof: defs.filter((d) => !d.mimic).length, ...(ch.tour.flangeLink !== undefined ? { flangeLink: ch.tour.flangeLink } : {}) };
      const r = new Robot(ch.name, chain);
      r.params.source = 'kintree';
      r.params.kinTree = tree.name;
      r.setJoints(defs.filter((d) => !d.mimic).map((d) => d.home ?? 0));
      ch.robot = r;
      robots.push(r);
      const Tb = T0.get(ch.joints[0].parent) ?? identity();
      const fk = r.fk(r.joints());
      ch.links.forEach((c, k) => { for (const v of c.visuals) allVisuals.push(moveGeometry(v, multiply(Tb, fk.linkPoses[k + 1]))); });
      return;
    }
    const jset = new Set(ch.joints.map((j) => j.name));
    const jdefs: JointDef[] = ch.joints.map((j, k) => {
      const origin = k === 0 ? j.origin : multiply(fixedPath(ch.joints[k - 1].child, j.parent), j.origin);
      const prism = j.type === 'prismatic';
      const cont = j.type === 'continuous' || (!prism && (j.lower === undefined || j.upper === undefined));
      const lower = j.lower ?? (prism ? -1000 : cont ? -360 : -180), upper = j.upper ?? (prism ? 1000 : cont ? 360 : 180);
      // continuous joints keep the equivalent angle in (-180, 180] (e.g. a valve stored at -15.706 rad), others are clamped
      const p0 = j.position ?? 0, home = cont && !prism ? p0 - 360 * Math.round(p0 / 360) : Math.max(lower, Math.min(upper, p0));
      const def: JointDef = { name: j.name, type: prism ? 'prismatic' : cont ? 'continuous' : 'revolute', origin, axis: normalize(j.axis), lower, upper, home };
      if (j.velocity !== undefined) def.maxVelocity = j.velocity;
      if (j.mimic) {
        if (jset.has(j.mimic.joint)) def.mimic = { ...j.mimic };
        else report.push(`${name}: mimic joint ${j.name} follows ${j.mimic.joint} of another chain; imported as independent`);
      }
      return def;
    });
    ch.links = ch.joints.map((j) => { const c = newCollected(); collectStatic(j.child, identity(), c, folded); return c; });
    const linkDefs: LinkDef[] = [{ name: ch.joints[0].parent, visuals: [] }, ...ch.joints.map((j, k) => {
      const ld: LinkDef & { collisions?: GeometryRef[] } = { name: j.child, visuals: ch.links[k].visuals.map(toLinkVisual) };
      if (ch.links[k].hasCollisions) ld.collisions = ch.links[k].collisions;
      return ld;
    })];
    const chain: ChainDef = { name: ch.name, joints: jdefs, links: linkDefs, flange: ch.flange, dof: jdefs.filter((d) => !d.mimic).length };
    const r = new Robot(ch.name, chain);
    r.params.source = 'kintree';
    r.params.kinTree = tree.name;
    r.setJoints(jdefs.filter((d) => !d.mimic).map((d) => d.home ?? 0));
    ch.robot = r;
    robots.push(r);
    // visuals at the initial configuration for the footprint
    let T = T0.get(ch.joints[0].parent) ?? identity();
    ch.joints.forEach((j, k) => {
      T = multiply(multiply(k === 0 ? T : multiply(T, fixedPath(ch.joints[k - 1].child, j.parent)), j.origin), motion(j, initValue(j)));
      for (const v of ch.links[k].visuals) allVisuals.push(moveGeometry(v, T));
    });
  });

  // root item
  let mobile: MobileRobot | undefined;
  let root: Item;
  const rootRobotCase = !makeMobile && chains.length === 1 && !baseC.sensors.some((s) => s.kind === 'camera' || s.kind === 'depth' || s.kind.startsWith('lidar'));
  if (makeMobile) {
    mobile = new MobileRobot(name);
    mobile.geometry = baseC.visuals;
    if (baseC.hasCollisions) setColliders(mobile, baseC.collisions);
    const kin: Partial<MobileKinematics> = {};
    const centers = new Map(wheels.map((w) => [w, wheelCenter(w)]));
    const mean = (a: number[]) => a.reduce((s, v) => s + v, 0) / Math.max(1, a.length);
    const rs = wheels.map((w) => radius.get(w)).filter((v): v is number => v !== undefined);
    if (rs.length) kin.wheelRadius = mean(rs);
    const steered = new Set(steers.flatMap((s) => movableBelow(s.child)));
    // driving axes from the wheels: lateral = the wheel axes (in the ground plane), forward = lateral x up; for a
    // steered base forward points from the rear to the steered axle (a URDF may drive along -Y of its base frame)
    let lat: Vec3 = [0, 0, 0];
    for (const w of wheels) { const a = normalize(transformDir(multiply(T0.get(w.parent) ?? identity(), w.origin), w.axis)); const sg = lat[0] * a[0] + lat[1] * a[1] < 0 ? -1 : 1; lat = [lat[0] + sg * a[0], lat[1] + sg * a[1], 0]; }
    if (Math.hypot(lat[0], lat[1]) < 1e-6) lat = [0, 1, 0];
    lat = normalize(lat);
    let fwd: Vec3 = [lat[1], -lat[0], 0];
    const front = wheels.filter((w) => steered.has(w)), rear = wheels.filter((w) => !steered.has(w));
    const along = (w: KinJoint, d: Vec3) => { const c = centers.get(w)!; return c[0] * d[0] + c[1] * d[1]; };
    if (steers.length && rear.length ? mean(front.map((w) => along(w, fwd))) < mean(rear.map((w) => along(w, fwd))) : fwd[0] < -1e-9 || (Math.abs(fwd[0]) <= 1e-9 && fwd[1] < 0)) { fwd = [-fwd[0], -fwd[1], 0]; lat = [-lat[0], -lat[1], 0]; }
    if (steers.length) {
      kin.drive = 'ackermann';
      kin.wheelBase = Math.abs(mean(front.map((w) => along(w, fwd))) - (rear.length ? mean(rear.map((w) => along(w, fwd))) : 0));
      // rear axle (reference point of the bicycle model, MobileKinematics.rearAxle) when the base drives along +X:
      // the UTP-G proto origin is the wheelbase midpoint (−wheelBase/2), a Webots Car proto's is the rear axle (0)
      if (rear.length && Math.abs(fwd[0] - 1) < 1e-6) kin.rearAxle = mean(rear.map((w) => along(w, fwd)));
      const axle = rear.length >= 2 ? rear : front;
      const ys = axle.map((w) => along(w, lat));
      kin.track = Math.max(...ys) - Math.min(...ys);
      const lim = steers.map((s) => (unlimited(s) ? 30 : Math.max(Math.abs(s.lower!), Math.abs(s.upper!))));
      kin.maxSteer = Math.min(...lim);
      kin.minTurnRadius = kin.wheelBase / Math.tan(kin.maxSteer * DEG);
    } else if (wheels.length >= 2) {
      kin.drive = 'differential';
      const ls = wheels.map((w) => along(w, lat)); const m = mean(ls);
      const left = ls.filter((v) => v > m + 1e-6), right = ls.filter((v) => v <= m + 1e-6);
      kin.track = left.length && right.length ? mean(left) - mean(right) : 2 * Math.abs(m);
      const xs = wheels.map((w) => along(w, fwd));
      kin.wheelBase = Math.max(...xs) - Math.min(...xs);
      kin.minTurnRadius = 0;
    } else {
      kin.drive = 'omni';
      kin.minTurnRadius = 0;
    }
    const vel = wheels.map((w) => w.velocity).filter((v): v is number => v !== undefined && v > 0);
    if (vel.length && kin.wheelRadius) {
      kin.maxSpeed = Math.min(...vel) * DEG * kin.wheelRadius;
      kin.maxReverseSpeed = kin.maxSpeed;
      if (kin.drive === 'differential' && kin.track) kin.maxYawRate = (2 * kin.maxSpeed / kin.track) * RAD;
      else if (kin.drive === 'ackermann' && kin.minTurnRadius) kin.maxYawRate = (kin.maxSpeed / kin.minTurnRadius) * RAD;
    }
    const fb = geometryBounds(allVisuals.filter((g) => (g.opacity ?? 1) > 0.01), opts.assets) ?? geometryBounds([...baseC.collisions], opts.assets);
    if (fb) kin.footprint = [fb.max[0] - fb.min[0], fb.max[1] - fb.min[1], fb.max[2] - fb.min[2]];
    const { wheels: _w, steer: _s, ...kinOverride } = override ?? {};
    void _w; void _s;
    mobile.kin = { ...mobile.kin, ...kin, ...kinOverride };
    mobile.params.wheelJoints = wheels.map((w) => w.name);
    mobile.params.steerJoints = steers.map((s) => s.name);
    if (wheels.length >= 2) {
      mobile.params.forwardAxis = [Number(fwd[0].toFixed(6)), Number(fwd[1].toFixed(6)), 0];
      if (fwd[0] < 0.99) report.push(`${name}: the wheels drive along (${fwd[0].toFixed(2)}, ${fwd[1].toFixed(2)}) of the base frame, not its +X (wheel base / track measured along that axis)`);
    }
    if (floating) mobile.params.floatingBase = floating.type;
    root = mobile;
    report.push(`${name}: MobileRobot ${mobile.kin.drive}${wheels.length ? ` (${wheels.length} wheels, radius ${(mobile.kin.wheelRadius).toFixed(1)} mm, track ${mobile.kin.track.toFixed(1)} mm${mobile.kin.drive === 'ackermann' ? `, wheel base ${mobile.kin.wheelBase.toFixed(1)} mm, max steer ${mobile.kin.maxSteer.toFixed(2)} deg` : ''})` : floating ? ' (floating base)' : ' (kinematic platform)'}`);
  } else if (rootRobotCase) {
    const ch = chains[0];
    const Tb = T0.get(ch.joints[0].parent) ?? identity();
    const inv = invert(Tb);
    ch.robot!.chain.links[0].visuals = baseC.visuals.map((v) => toLinkVisual(moveGeometry(v, inv)));
    if (baseC.hasCollisions) (ch.robot!.chain.links[0] as LinkDef & { collisions?: GeometryRef[] }).collisions = baseC.collisions.map((c) => moveGeometry(c, inv));
    ch.robot!.setPose(Tb);
    root = ch.robot!;
  } else {
    const so = new SceneObject(name);
    so.geometry = baseC.visuals;
    setColliders(so, baseC.collisions);
    so.mass = baseC.mass;
    so.params.kinTree = tree.name;
    root = so;
  }
  if (floating) root.setPose(baseOffset);
  if (tree.params) for (const [k, v] of Object.entries(tree.params)) root.params[k] = v;

  // mount chains
  for (const ch of chains) {
    const r = ch.robot!;
    if (r === root) continue;
    if (!ch.parent) {
      (root as Item).addChild(r);
      r.setPose(T0.get(ch.joints[0].parent) ?? identity());
    } else {
      const p = ch.parent;
      p.robot!.addChild(r);
      r.setPose(multiply(invert(p.flange), fixedPath(p.joints[p.joints.length - 1].child, ch.joints[0].parent)));
    }
    report.push(`${name}: Robot ${r.name} (${r.dof} DOF: ${r.jointNames().join(', ')})`);
  }

  // sensors
  const addSensor = (s: KinSensor, owner: Item, pose: Mat4) => {
    if (s.kind === 'camera' || s.kind === 'depth' || s.kind === 'lidar2d' || s.kind === 'lidar3d') {
      const cam = new Camera(s.name);
      cam.kind = s.kind === 'camera' ? 'rgb' : s.kind;
      if (s.fov !== undefined) cam.fov = s.fov;
      cam.fovAxis = s.fovAxis ?? 'vertical';
      if (s.width) cam.width = s.width;
      if (s.height) cam.height = s.height;
      if (s.near !== undefined) cam.near = s.near;
      if (s.far !== undefined) cam.far = s.far;
      if (s.params) Object.assign(cam.params, s.params);
      if (s.layers !== undefined) cam.params.layers = s.layers;
      owner.addChild(cam);
      cam.setPose(pose);
      cameras.push(cam);
    }
    if (mobile && owner === mobile) {
      const kind = s.kind === 'lidar2d' || s.kind === 'lidar3d' ? s.kind : s.kind === 'gnss' ? 'gnss' : s.kind === 'imu' || s.kind === 'gyro' || s.kind === 'accelerometer' ? 'imu' : s.kind === 'camera' || s.kind === 'depth' ? 'camera' : null;
      if (kind) mobile.sensors.push({ kind, pose: Array.from(pose), ...(s.far !== undefined ? { range: s.far } : {}), ...(s.fov !== undefined ? { fov: s.fov } : {}) });
    } else if (s.kind !== 'camera' && s.kind !== 'depth' && !s.kind.startsWith('lidar')) {
      const list = (owner.params.sensors as ParamValue[] | undefined) ?? [];
      list.push({ kind: s.kind, name: s.name, pose: Array.from(pose) });
      owner.params.sensors = list;
    }
  };
  const baseOwner = root instanceof Robot && root === chains[0]?.robot ? null : root;
  for (const s of baseC.sensors) {
    if (baseOwner) addSensor(s, baseOwner, s.pose);
    else report.push(`${name}: sensor ${s.name} on the base of a fixed robot is not attached`);
  }
  for (const ch of chains) {
    ch.links.forEach((c, k) => {
      for (const s of c.sensors) {
        if (k !== ch.links.length - 1) report.push(`${name}: sensor ${s.name} sits on intermediate link ${ch.tour ? ch.tour.linkNames[k] : ch.joints[k].child} of ${ch.name}; it is attached to the flange at the initial pose`);
        let pose = multiply(invert(ch.flange), s.pose);
        if (k !== ch.links.length - 1) {
          const r = ch.robot!;
          const fk = r.fk(r.joints());
          pose = multiply(invert(multiply(fk.linkPoses[ch.links.length], ch.flange)), multiply(fk.linkPoses[k + 1], s.pose));
        }
        addSensor(s, ch.robot!, pose);
      }
    });
  }
  return { root, mobile, robots, cameras, report };
}
