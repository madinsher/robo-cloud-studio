/**
 * URDF export: robots (serial chains with meshes) and whole stations as a ROS description package.
 *
 * Output is a set of files (path -> bytes/text) that can be zipped (`packageZip`) and dropped into a
 * ROS workspace (`colcon build`, `ros2 launch <pkg> display.launch.py`), imported into Blender with Phobos, opened in
 * RViz / Gazebo / MoveIt Setup Assistant (a basic SRDF is included) or re-imported here. Units are converted from the
 * studio's mm/deg to URDF metres/radians; meshes are written as binary STL in metres next to the URDF.
 * Robots imported from URDF / xacro (a kinematic tree, possibly several robots + a mobile base) are exported from their
 * kept robot description: every link / joint name (TF frames such as `utp_g_01/base_link`), inertials, mimic and
 * off-chain joints, ros2_control / gazebo / transmission blocks survive the round trip; only mesh URIs are rewritten to
 * the exported package (original mesh files copied when available).
 */
import { ChainDef, JointDef, LinkVisual } from '../../core/kinematics/chain';
import { Robot } from '../../core/items/robot';
import { Item, ItemType, SceneObject, Station, Tool, GeometryRef } from '../../core/items/item';
import { Mat4, poseToXyzrpw, DEG, multiply, identity, fromArray, invert } from '../../core/math/pose';
import { AssetStore } from '../../scene/assets';
import { writeSTL, MeshData } from '../mesh/stl';
import { zipSync, strToU8 } from 'fflate';
import { parseXML, serializeXML, XNode, childElements } from './xml';
import { exportSRDF } from './srdf';

export interface URDFExportOptions {
  /** ROS package name (used in package:// URIs). Defaults to a sanitized robot/station name. */
  packageName?: string;
  /** Include visual meshes (default true). */
  meshes?: boolean;
  /** Also emit <collision> elements mirroring the visuals (default true). */
  collisions?: boolean;
  /** Add a `launch/display.launch.py` (robot_state_publisher + joint_state_publisher) and a basic SRDF (default true). */
  extras?: boolean;
  /**
   * Station export: how mobile robots hang from `world`. 'fixed' (default) freezes them at their current pose;
   * 'floating' (live digital twin) uses a floating joint that robot_state_publisher ignores, the odom -> base TF comes
   * from the twin. Robots carried by a mobile robot are attached to its base link either way.
   */
  mobileBases?: 'fixed' | 'floating';
}

export interface URDFExportResult {
  /** File path inside the package -> content. */
  files: Record<string, Uint8Array | string>;
  urdf: string;
  packageName: string;
  warnings: string[];
  /** URDF joint names of every exported robot (all chain joints, chain order). */
  jointNames?: Map<Robot, string[]>;
  /** URDF base link of every exported mobile robot / URDF import root. */
  baseLinks?: Map<Item, string>;
}

const M = 1 / 1000;

export function sanitizeName(s: string): string {
  return s.replace(/[^A-Za-z0-9_]+/g, '_').replace(/^_+|_+$/g, '').replace(/^(\d)/, '_$1') || 'item';
}

function originXml(m: Mat4 | number[] | undefined): string {
  if (!m) return '';
  const p = poseToXyzrpw(m instanceof Float64Array ? m : fromArray(m));
  const f = (v: number) => (Math.abs(v) < 1e-9 ? 0 : +v.toFixed(6));
  return `<origin xyz="${f(p[0] * M)} ${f(p[1] * M)} ${f(p[2] * M)}" rpy="${f(p[3] * DEG)} ${f(p[4] * DEG)} ${f(p[5] * DEG)}"/>`;
}

function colorXml(color?: string, opacity?: number): string {
  if (!color) return '';
  const c = color.replace('#', '');
  if (c.length < 6) return '';
  const r = parseInt(c.slice(0, 2), 16) / 255, g = parseInt(c.slice(2, 4), 16) / 255, b = parseInt(c.slice(4, 6), 16) / 255;
  return `<material name="c_${c}"><color rgba="${r.toFixed(3)} ${g.toFixed(3)} ${b.toFixed(3)} ${(opacity ?? 1).toFixed(3)}"/></material>`;
}

class MeshWriter {
  files: Record<string, Uint8Array | string> = {};
  private written = new Map<string, string>();
  warnings: string[] = [];
  constructor(private assets: AssetStore | undefined, private pkg: string) {}

  /** Write a mesh (scene units = raw asset units × scale) as STL in metres; returns the package URI. */
  write(meshId: string, scale: [number, number, number] | undefined, baseName: string): string | null {
    const key = `${meshId}|${(scale ?? [1, 1, 1]).join(',')}`;
    const hit = this.written.get(key);
    if (hit) return hit;
    const a = this.assets?.get(meshId) ?? (this.assets ? [...this.assets.entries()].find(([id]) => id.split(/[\\/]/).pop()?.toLowerCase() === meshId.split(/[\\/]/).pop()?.toLowerCase())?.[1] : undefined);
    const mesh: MeshData | undefined = a?.mesh;
    if (!mesh) { this.warnings.push(`Mesh ${meshId} not loaded — visual skipped`); return null; }
    const s = scale ?? [1, 1, 1];
    // asset in raw units; scene mm = raw × scale; metres = mm / 1000
    const unit = a?.units === 'm' ? 1 : 1; // asset units already encoded in `scale` by the visual
    const bytes = writeSTL(mesh, (p) => [p[0] * s[0] * M * unit, p[1] * s[1] * M * unit, p[2] * s[2] * M * unit]);
    let name = sanitizeName(baseName);
    let file = `meshes/${name}.stl`;
    let k = 1;
    while (this.files[file]) file = `meshes/${name}_${++k}.stl`;
    this.files[file] = bytes;
    const uri = `package://${this.pkg}/${file}`;
    this.written.set(key, uri);
    return uri;
  }

  /** Copy the mesh of a URDF `<mesh filename>` into the package unchanged (same units, the URDF keeps its `scale`). */
  copy(uri: string): string | null {
    const hit = this.written.get(`copy|${uri}`);
    if (hit) return hit;
    const base = (u: string) => u.split(/[\\/]/).pop()!.toLowerCase();
    const a = this.assets?.get(uri) ?? (this.assets ? [...this.assets.entries()].find(([id]) => base(id) === base(uri))?.[1] : undefined);
    if (!a?.source && !a?.mesh) { this.warnings.push(`Mesh ${uri} not loaded — kept as an external reference`); return null; }
    const stem = sanitizeName(uri.split(/[\\/]/).pop()!.replace(/\.[^.]+$/, ''));
    const ext = a.source ? a.type : 'stl';
    let file = `meshes/${stem}.${ext}`;
    for (let k = 2; this.files[file]; k++) file = `meshes/${stem}_${k}.${ext}`;
    // without source bytes: raw geometry (STL / OBJ in the file's units = metres for URDF, COLLADA parsed to mm unless kept in m)
    const f = a.type === 'dae' && a.units !== 'm' ? M : 1;
    this.files[file] = a.source ?? writeSTL(a.mesh!, (p) => [p[0] * f, p[1] * f, p[2] * f]);
    const out = `package://${this.pkg}/${file}`;
    this.written.set(`copy|${uri}`, out);
    return out;
  }
}

/** The item holding the kept robot description of a URDF / xacro import that `it` belongs to. */
export function urdfTreeRoot(it: Item | null): Item | null {
  for (let p = it; p; p = p.parent) if (p.params.source === 'urdf' && typeof p.params.robotDescription === 'string') return p;
  return null;
}

/** Link / joint names of a URDF are TF frame names: keep letters, digits, `_` and `/` (sanitizeName for other names). */
export function frameName(s: string): string {
  return s.replace(/[^A-Za-z0-9_/]+/g, '_').replace(/^[_/]+|[_/]+$/g, '').replace(/^(\d)/, '_$1') || 'link';
}

/** Elements of a kept robot description with mesh URIs rewritten into the package (meshes / collisions per options). */
function descriptionElements(desc: string, mw: MeshWriter, opts: URDFExportOptions): { name: string; elements: XNode[]; rootLink: string } {
  const robot = parseXML(desc);
  const hasMesh = (n: XNode): boolean => n.name === 'mesh' || n.children.some(hasMesh);
  const prune = (n: XNode): XNode | null => {
    if (n.type !== 'element') return n.text && n.text.trim() ? n : null;
    if (n.name === 'collision' && opts.collisions === false) return null;
    if ((n.name === 'visual' || n.name === 'collision') && opts.meshes === false && hasMesh(n)) return null;
    const c: XNode = { ...n, attrs: { ...n.attrs }, children: n.children.map(prune).filter((x): x is XNode => !!x) };
    if (n.name === 'mesh' && n.attrs.filename && opts.meshes !== false) { const u = mw.copy(n.attrs.filename); if (u) c.attrs.filename = u; }
    return c;
  };
  const elements = robot.children.map(prune).filter((x): x is XNode => !!x && x.type === 'element');
  const children = new Set(elements.filter((e) => e.name === 'joint').map((j) => childElements(j, 'child')[0]?.attrs.link));
  const rootLink = elements.find((e) => e.name === 'link' && !children.has(e.attrs.name))?.attrs.name ?? 'base_link';
  return { name: robot.attrs.name ?? 'robot', elements, rootLink };
}

/** Adjacent link pairs (parent / child of every joint) of a URDF text. */
function adjacentPairs(urdf: string): [string, string][] {
  return childElements(parseXML(urdf), 'joint').map((j) => [childElements(j, 'parent')[0]?.attrs.link, childElements(j, 'child')[0]?.attrs.link] as [string, string]).filter(([a, b]) => a && b);
}

/** Basic SRDF: one chain group per robot (base -> last link), `home` + imported named poses, adjacent pairs disabled. */
function srdfFor(robotName: string, urdf: string, robots: { robot: Robot; base: string; tip: string }[], extra: { link1: string; link2: string; reason: string }[] = []): string {
  const groups: { name: string; base: string; tip: string }[] = [];
  const states: { name: string; group: string; joints: Record<string, number> }[] = [];
  for (const { robot, base, tip } of robots) {
    let g = ((robot.params.planningGroups as string[] | undefined)?.[0]) ?? (robots.length === 1 ? 'manipulator' : sanitizeName(robot.name).toLowerCase());
    while (groups.some((x) => x.name === g)) g += '_';
    groups.push({ name: g, base, tip });
    const indep = robot.chain.joints.filter((j) => !j.mimic);
    const names = indep.map((j) => frameName(j.name));
    const conv = (i: number, v: number) => (indep[i]?.type === 'prismatic' ? v * M : v * (Math.PI / 180));
    const poses: Record<string, number[]> = { home: robot.jointsHome(), ...((robot.params.namedPoses as Record<string, number[]> | undefined) ?? {}) };
    for (const [pn, q] of Object.entries(poses)) states.push({ name: pn, group: g, joints: Object.fromEntries(names.map((n, i) => [n, conv(i, q[i] ?? 0)])) });
  }
  const seen = new Set<string>();
  const adj = adjacentPairs(urdf).filter(([a, b]) => { const k = [a, b].sort().join('|'); if (seen.has(k)) return false; seen.add(k); return true; });
  return exportSRDF(robotName, groups, states, adj, extra.filter((d) => !seen.has([d.link1, d.link2].sort().join('|'))));
}

function geometryXml(v: LinkVisual | GeometryRef, mw: MeshWriter, baseName: string, opts: URDFExportOptions): string {
  let geom = '';
  if (v.mesh) {
    if (opts.meshes === false) return '';
    const uri = mw.write(v.mesh, v.scale as [number, number, number] | undefined, baseName);
    if (!uri) return '';
    geom = `<mesh filename="${uri}"/>`;
  } else if (v.primitive) {
    const p = v.primitive as any;
    if (p.kind === 'box') geom = `<box size="${(p.size[0] * M).toFixed(5)} ${(p.size[1] * M).toFixed(5)} ${(p.size[2] * M).toFixed(5)}"/>`;
    else if (p.kind === 'cylinder' || p.kind === 'cone') geom = `<cylinder radius="${(p.radius * M).toFixed(5)} " length="${(p.length * M).toFixed(5)}"/>`.replace(' "', '"');
    else if (p.kind === 'sphere') geom = `<sphere radius="${(p.radius * M).toFixed(5)}"/>`;
    else if (p.kind === 'plane') geom = `<box size="${(p.size[0] * M).toFixed(5)} ${(p.size[1] * M).toFixed(5)} 0.001"/>`;
  }
  if (!geom) return '';
  const origin = originXml(v.origin as Mat4 | number[] | undefined);
  const vis = `    <visual>${origin}<geometry>${geom}</geometry>${colorXml(v.color, (v as GeometryRef).opacity)}</visual>\n`;
  const col = opts.collisions === false ? '' : `    <collision>${origin}<geometry>${geom}</geometry></collision>\n`;
  return vis + col;
}

function jointXml(j: JointDef, parent: string, child: string): string {
  const type = j.type === 'fixed' ? 'fixed' : j.type === 'prismatic' ? 'prismatic' : (j.upper - j.lower) >= 720 ? 'continuous' : 'revolute';
  const lim = j.type === 'prismatic' ? { lo: j.lower * M, hi: j.upper * M, v: (j.maxVelocity ?? 1000) * M } : { lo: j.lower * DEG, hi: j.upper * DEG, v: (j.maxVelocity ?? 180) * DEG };
  let s = `  <joint name="${frameName(j.name)}" type="${type}">\n    <parent link="${parent}"/>\n    <child link="${child}"/>\n    ${originXml(j.origin)}\n`;
  if (type !== 'fixed') {
    s += `    <axis xyz="${j.axis.map((a) => +a.toFixed(6)).join(' ')}"/>\n`;
    if (type !== 'continuous') s += `    <limit lower="${lim.lo.toFixed(6)}" upper="${lim.hi.toFixed(6)}" effort="1000" velocity="${lim.v.toFixed(4)}"/>\n`;
    else s += `    <limit effort="1000" velocity="${lim.v.toFixed(4)}"/>\n`;
    if (j.mimic) s += `    <mimic joint="${frameName(j.mimic.joint)}" multiplier="${j.mimic.multiplier}" offset="${j.type === 'prismatic' ? j.mimic.offset * M : j.mimic.offset * DEG}"/>\n`;
  }
  return s + '  </joint>\n';
}

/** URDF for one kinematic chain (robot). Link names are prefixed so several robots can share a package. */
export function chainToURDF(chain: ChainDef, name: string, mw: MeshWriter, opts: URDFExportOptions, prefix = ''): { xml: string; rootLink: string; tipLink: string } {
  const rn = sanitizeName(name);
  const lname = (i: number) => frameName(`${prefix}${chain.links[i]?.name ?? (i === 0 ? 'base_link' : `link_${i}`)}`);
  // Our chains may carry a static `post` transform after each joint motion (DH robots). URDF has no such
  // thing, so the post of joint i is folded into joint i+1's origin and into link i+1's visual origins.
  const postOf = (i: number): Mat4 | undefined => (i >= 0 && i < chain.joints.length ? chain.joints[i].post : undefined);
  let body = '';
  for (let i = 0; i < chain.links.length; i++) {
    const l = chain.links[i];
    const post = postOf(i - 1);
    body += `  <link name="${lname(i)}">\n`;
    for (const v of l.visuals) body += geometryXml(post ? { ...v, origin: multiply(post, v.origin) } : v, mw, `${rn}_${l.name ?? i}`, opts);
    body += '  </link>\n';
  }
  for (let i = 0; i < chain.joints.length; i++) {
    const src = chain.joints[i];
    const post = postOf(i - 1);
    const j: JointDef = { ...src, name: `${prefix}${src.name}`, origin: post ? multiply(post, src.origin) : src.origin };
    if (j.mimic) j.mimic = { ...j.mimic, joint: `${prefix}${j.mimic.joint}` };
    body += jointXml(j, lname(i), lname(i + 1));
  }
  // flange frame (tool0): last post × chain flange
  const tip = lname(chain.links.length - 1);
  const lastPost = postOf(chain.joints.length - 1);
  const flangeT = lastPost ? multiply(lastPost, chain.flange ?? identity()) : chain.flange;
  const flange = flangeT && !isIdentity(flangeT) ? `${prefix}tool0` : '';
  if (flange) body += `  <link name="${sanitizeName(flange)}"/>\n  <joint name="${sanitizeName(flange)}_joint" type="fixed">\n    <parent link="${tip}"/>\n    <child link="${sanitizeName(flange)}"/>\n    ${originXml(flangeT)}\n  </joint>\n`;
  return { xml: body, rootLink: lname(0), tipLink: flange ? sanitizeName(flange) : tip };
}

function isIdentity(m: Mat4): boolean {
  const I = identity();
  for (let i = 0; i < 16; i++) if (Math.abs(m[i] - I[i]) > 1e-9) return false;
  return true;
}

/**
 * Export a single robot (with its tools as fixed links on the flange) as a URDF package. A robot imported from URDF /
 * xacro exports the whole kept description of its import (all chains, mobile base, sensors, ros2_control).
 */
export function exportRobotURDF(robot: Robot, assets?: AssetStore, opts: URDFExportOptions = {}): URDFExportResult {
  const tree = urdfTreeRoot(robot);
  if (tree) return exportDescriptionURDF(tree, assets, opts);
  const pkg = sanitizeName(opts.packageName ?? `${robot.name}_description`).toLowerCase();
  const mw = new MeshWriter(assets, pkg);
  const { xml, rootLink, tipLink } = chainToURDF(robot.chain, robot.name, mw, opts);
  let body = xml;
  for (const t of robot.tools()) {
    const tn = sanitizeName(t.name);
    body += `  <link name="${tn}">\n`;
    for (const g of t.geometry) body += geometryXml(g, mw, tn, opts);
    body += `  </link>\n  <joint name="${tn}_joint" type="fixed">\n    <parent link="${tipLink}"/>\n    <child link="${tn}"/>\n    <origin xyz="0 0 0" rpy="0 0 0"/>\n  </joint>\n`;
    body += `  <link name="${tn}_tcp"/>\n  <joint name="${tn}_tcp_joint" type="fixed">\n    <parent link="${tn}"/>\n    <child link="${tn}_tcp"/>\n    ${originXml(t.poseTool())}\n  </joint>\n`;
  }
  const rn = sanitizeName(robot.name);
  const urdf = `<?xml version="1.0"?>\n<!-- Generated by VerticalBot Studio: ${robot.name} (${robot.brand}${robot.model ? ' ' + robot.model : ''}) -->\n<robot name="${rn}">\n${body}</robot>\n`;
  const lastLink = frameName(`${robot.chain.links[robot.chain.links.length - 1]?.name ?? `link_${robot.chain.links.length - 1}`}`);
  const files: Record<string, Uint8Array | string> = { ...mw.files, [`urdf/${rn}.urdf`]: urdf };
  if (opts.extras !== false) Object.assign(files, displayFiles(pkg, rn, srdfFor(rn, urdf, [{ robot, base: rootLink, tip: lastLink }])));
  Object.assign(files, packageFiles(pkg, robot.name, files, [robot.name]));
  return { files, urdf, packageName: pkg, warnings: mw.warnings };
}

/** Export the kept robot description of a URDF / xacro import (names, inertials, every joint, ros2_control kept). */
export function exportDescriptionURDF(tree: Item, assets?: AssetStore, opts: URDFExportOptions = {}): URDFExportResult {
  const desc = tree.params.robotDescription as string;
  const d0 = parseXML(desc).attrs.name ?? tree.name;
  const pkg = sanitizeName(opts.packageName ?? `${d0}_description`).toLowerCase();
  const mw = new MeshWriter(assets, pkg);
  const { name, elements } = descriptionElements(desc, mw, opts);
  const urdf = `<?xml version="1.0"?>\n<!-- Exported by VerticalBot Studio from the imported description of ${tree.name} -->\n${serializeXML({ type: 'element', name: 'robot', attrs: { name }, children: elements })}`;
  const fn = sanitizeName(name);
  const robots = [...tree.walk()].filter((x): x is Robot => x instanceof Robot && x.chain.joints.length > 0);
  const files: Record<string, Uint8Array | string> = { ...mw.files, [`urdf/${fn}.urdf`]: urdf };
  const disabled = ((tree.params.disabledCollisions as string[][] | undefined) ?? []).map(([link1, link2, reason]) => ({ link1, link2, reason }));
  if (opts.extras !== false) Object.assign(files, displayFiles(pkg, fn, srdfFor(name, urdf, robots.map((r) => ({ robot: r, base: r.chain.links[0].name, tip: r.chain.links[r.chain.links.length - 1].name })), disabled)));
  Object.assign(files, packageFiles(pkg, name, files, robots.map((r) => r.name)));
  return { files, urdf, packageName: pkg, warnings: mw.warnings };
}

/**
 * Export the whole station: every robot becomes a chain hanging from `world` through a fixed joint at its
 * base pose; objects, frames and mobile robots become fixed links with their meshes. One URDF per station.
 */
export function exportStationURDF(station: Station, assets?: AssetStore, opts: URDFExportOptions = {}): URDFExportResult {
  const pkg = sanitizeName(opts.packageName ?? `${station.name}_description`).toLowerCase();
  const mw = new MeshWriter(assets, pkg);
  let body = '  <link name="world"/>\n';
  const used = new Set<string>(['world']);
  const uniq = (n: string) => { let k = sanitizeName(n), i = 1; while (used.has(k)) k = `${sanitizeName(n)}_${++i}`; used.add(k); return k; };
  const robots = station.itemsOfType<Robot>(ItemType.ROBOT);
  const jointNames = new Map<Robot, string[]>();
  const baseLinks = new Map<Item, string>();
  const floating = opts.mobileBases === 'floating';
  const isMobile = (it: Item) => it.type === ItemType.MOBILE_ROBOT;
  /** Parent link + pose (relative to it) for an item: the base link of a carrying mobile robot, else world. */
  const mount = (it: Item): { link: string; pose: Mat4 } => {
    for (let p = it.parent; p; p = p.parent) {
      const bl = baseLinks.get(p);
      if (bl) { const inv = fromArray(Array.from(p.poseAbs())); return { link: bl, pose: multiply(invert(inv), it.poseAbs()) }; }
    }
    return { link: 'world', pose: it.poseAbs() };
  };
  // native mobile robots: a base link each (their children are mounted on it)
  for (const m of station.itemsOfType<Item>(ItemType.MOBILE_ROBOT)) {
    if (urdfTreeRoot(m)) continue;
    const ln = uniq(`${m.name}_base_link`);
    baseLinks.set(m, ln);
    const mt = mount(m);
    body += `  <link name="${ln}"/>\n  <joint name="${ln}_joint" type="${floating ? 'floating' : 'fixed'}">\n    <parent link="${mt.link}"/>\n    <child link="${ln}"/>\n    ${originXml(mt.pose)}\n  </joint>\n`;
  }
  // URDF / xacro imports: their kept descriptions, names unchanged unless they clash (then prefixed, with a warning)
  const trees = [...station.walk()].filter((it) => it.params.source === 'urdf' && typeof it.params.robotDescription === 'string' && urdfTreeRoot(it.parent) === null);
  const treeWarnings: string[] = [];
  for (const t of trees) {
    const { elements, rootLink } = descriptionElements(t.params.robotDescription as string, mw, opts);
    const clash = elements.some((e) => e.name === 'link' && used.has(e.attrs.name));
    const pfx = clash ? `${sanitizeName(t.name)}_` : '';
    if (clash) treeWarnings.push(`${t.name}: link names clash with other items — prefixed with ${pfx}`);
    const renamed = (n: XNode, k: string) => !!pfx && ((n.name === 'link' && k === 'name') || ((n.name === 'parent' || n.name === 'child') && k === 'link') || (n.name === 'joint' && k === 'name') || (n.name === 'mimic' && k === 'joint'));
    const rename = (n: XNode): XNode => ({ ...n, attrs: Object.fromEntries(Object.entries(n.attrs).map(([k, v]) => [k, renamed(n, k) ? `${pfx}${v}` : v])), children: n.children.map(rename) });
    for (const e of elements) { const r = rename(e); if (r.name === 'link') used.add(r.attrs.name); body += serializeXML(r, '  '); }
    const mt = mount(t);
    const tj = floating && isMobile(t) ? 'floating' : 'fixed';
    body += `  <joint name="${uniq(`${pfx}${t.name}_base_joint`)}" type="${tj}">\n    <parent link="${mt.link}"/>\n    <child link="${pfx}${rootLink}"/>\n    ${originXml(mt.pose)}\n  </joint>\n`;
    baseLinks.set(t, `${pfx}${rootLink}`);
    for (const r of [...t.walk()]) if (r instanceof Robot) jointNames.set(r, r.chain.joints.map((j) => `${pfx}${j.name}`));
  }
  for (const r of robots) {
    if (urdfTreeRoot(r)) continue;
    const prefix = `${uniq(r.name)}_`;
    const { xml, rootLink, tipLink } = chainToURDF(r.chain, r.name, mw, opts, prefix);
    jointNames.set(r, r.chain.joints.map((j) => frameName(`${prefix}${j.name}`)));
    body += xml;
    const mt = mount(r);
    body += `  <joint name="${prefix}base_joint" type="fixed">\n    <parent link="${mt.link}"/>\n    <child link="${rootLink}"/>\n    ${originXml(mt.pose)}\n  </joint>\n`;
    for (const t of r.tools()) {
      const tn = uniq(`${prefix}${t.name}`);
      body += `  <link name="${tn}">\n`;
      for (const g of t.geometry) body += geometryXml(g, mw, tn, opts);
      body += `  </link>\n  <joint name="${tn}_joint" type="fixed">\n    <parent link="${tipLink}"/>\n    <child link="${tn}"/>\n    <origin xyz="0 0 0" rpy="0 0 0"/>\n  </joint>\n`;
    }
  }
  const visit = (it: Item) => {
    for (const c of it.children) {
      if (c instanceof Robot || urdfTreeRoot(c)) continue; // robots (and their tools) and URDF imports handled above
      if (c instanceof SceneObject && !(c instanceof Tool) && c.geometry.length) {
        const ln = uniq(c.name);
        const mt = mount(c);
        body += `  <link name="${ln}">\n`;
        for (const g of c.geometry) body += geometryXml(g, mw, ln, opts);
        body += `  </link>\n  <joint name="${ln}_joint" type="fixed">\n    <parent link="${mt.link}"/>\n    <child link="${ln}"/>\n    ${originXml(mt.pose)}\n  </joint>\n`;
      } else if (c.type === ItemType.FRAME) {
        const ln = uniq(c.name);
        const mt = mount(c);
        body += `  <link name="${ln}"/>\n  <joint name="${ln}_joint" type="fixed">\n    <parent link="${mt.link}"/>\n    <child link="${ln}"/>\n    ${originXml(mt.pose)}\n  </joint>\n`;
      }
      visit(c);
    }
  };
  visit(station);
  const sn = sanitizeName(station.name);
  const urdf = `<?xml version="1.0"?>\n<!-- Generated by VerticalBot Studio: station "${station.name}" -->\n<robot name="${sn}">\n${body}</robot>\n`;
  const files: Record<string, Uint8Array | string> = { ...mw.files, [`urdf/${sn}.urdf`]: urdf };
  if (opts.extras !== false) Object.assign(files, displayFiles(pkg, sn, null));
  Object.assign(files, packageFiles(pkg, station.name, files, robots.map((r) => r.name)));
  return { files, urdf, packageName: pkg, warnings: [...treeWarnings, ...mw.warnings], jointNames, baseLinks };
}

/** `launch/display.launch.py` (robot_state_publisher with the URDF, joint_state_publisher when installed) + SRDF. */
function displayFiles(pkg: string, urdfName: string, srdf: string | null): Record<string, string> {
  const out: Record<string, string> = {
    'launch/display.launch.py': [
      `"""robot_state_publisher for ${pkg} (exported by VerticalBot Studio). /joint_states from joint_state_publisher or a driver."""`,
      'import os', 'from ament_index_python.packages import get_package_share_directory, PackageNotFoundError', 'from launch import LaunchDescription',
      'from launch.actions import DeclareLaunchArgument, LogInfo', 'from launch.conditions import IfCondition', 'from launch.substitutions import LaunchConfiguration', 'from launch_ros.actions import Node', '', '',
      'def generate_launch_description():',
      `    urdf = os.path.join(get_package_share_directory('${pkg}'), 'urdf', '${urdfName}.urdf')`,
      "    with open(urdf, encoding='utf-8') as f:", '        description = f.read()',
      "    actions = [DeclareLaunchArgument('use_sim_time', default_value='false'), DeclareLaunchArgument('joint_state_publisher', default_value='true'),",
      "               Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',",
      "                    parameters=[{'robot_description': description, 'use_sim_time': LaunchConfiguration('use_sim_time')}])]",
      '    try:', "        get_package_share_directory('joint_state_publisher')",
      "        actions.append(Node(package='joint_state_publisher', executable='joint_state_publisher', condition=IfCondition(LaunchConfiguration('joint_state_publisher'))))",
      '    except PackageNotFoundError:', "        actions.append(LogInfo(msg='joint_state_publisher not installed: publish /joint_states yourself'))",
      '    return LaunchDescription(actions)', '',
    ].join('\n'),
  };
  if (srdf) out[`config/${urdfName}.srdf`] = srdf;
  return out;
}

/** package.xml / CMakeLists.txt (installing only the folders that exist) / README. */
function packageFiles(pkg: string, title: string, files: Record<string, unknown>, robots: string[]): Record<string, string> {
  const dirs = ['urdf', 'meshes', 'launch', 'config'].filter((d) => Object.keys(files).some((f) => f.startsWith(`${d}/`)));
  const launch = Object.keys(files).includes('launch/display.launch.py');
  const esc = (v: string) => v.replace(/&/g, '&amp;').replace(/</g, '&lt;');
  return {
    'package.xml': `<?xml version="1.0"?>\n<package format="3">\n  <name>${pkg}</name>\n  <version>0.1.0</version>\n  <description>${esc(title)} — URDF description exported from VerticalBot Studio</description>\n  <maintainer email="user@example.com">VerticalBot Studio</maintainer>\n  <license>Apache-2.0</license>\n  <buildtool_depend>ament_cmake</buildtool_depend>\n  <exec_depend>robot_state_publisher</exec_depend>\n${launch ? '  <exec_depend>joint_state_publisher</exec_depend>\n  <exec_depend>launch_ros</exec_depend>\n' : ''}  <export><build_type>ament_cmake</build_type></export>\n</package>\n`,
    'CMakeLists.txt': `cmake_minimum_required(VERSION 3.8)\nproject(${pkg})\nfind_package(ament_cmake REQUIRED)\ninstall(DIRECTORY ${dirs.join(' ')} DESTINATION share/\${PROJECT_NAME})\nament_package()\n`,
    'README.md': `# ${pkg}\n\nExported from VerticalBot Studio. Robots: ${robots.join(', ') || '—'}.\n\n- ROS 2: copy into your workspace \`src/\`, \`colcon build\`, then ${launch ? `\`ros2 launch ${pkg} display.launch.py\` (robot_state_publisher + joint_state_publisher)` : '`ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat urdf/*.urdf)"`'}; MoveIt Setup Assistant can start from \`config/*.srdf\`.\n- Blender: install the Phobos add-on and use *Import > URDF*; meshes are binary STL in metres.\n- Back to the studio: drop the package folder (or zip) onto the 3D view.\n`,
  };
}

/** Zip the exported files under a top-level folder named after the package. */
export function packageZip(res: URDFExportResult): Uint8Array {
  const entries: Record<string, Uint8Array> = {};
  for (const [p, c] of Object.entries(res.files)) entries[`${res.packageName}/${p}`] = typeof c === 'string' ? strToU8(c) : c;
  return zipSync(entries, { level: 6 });
}

export { multiply };
