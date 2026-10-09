/**
 * URDF export: robots (serial chains with meshes) and whole stations as a ROS description package.
 *
 * Output is a set of files (path -> bytes/text) that can be zipped (`packageZip`) and dropped into a
 * ROS workspace, imported into Blender with Phobos, opened in RViz / Gazebo / MoveIt Setup Assistant
 * or re-imported here. Units are converted from the studio's mm/deg to URDF metres/radians; meshes
 * are written as binary STL in metres next to the URDF.
 */
import { ChainDef, JointDef, LinkVisual } from '../../core/kinematics/chain';
import { Robot } from '../../core/items/robot';
import { Item, ItemType, SceneObject, Station, Tool, GeometryRef } from '../../core/items/item';
import { Mat4, poseToXyzrpw, DEG, multiply, identity, fromArray } from '../../core/math/pose';
import { AssetStore } from '../../scene/assets';
import { writeSTL, MeshData } from '../mesh/stl';
import { zipSync, strToU8 } from 'fflate';

export interface URDFExportOptions {
  /** ROS package name (used in package:// URIs). Defaults to a sanitized robot/station name. */
  packageName?: string;
  /** Include visual meshes (default true). */
  meshes?: boolean;
  /** Also emit <collision> elements mirroring the visuals (default true). */
  collisions?: boolean;
}

export interface URDFExportResult {
  /** File path inside the package -> content. */
  files: Record<string, Uint8Array | string>;
  urdf: string;
  packageName: string;
  warnings: string[];
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
  let s = `  <joint name="${sanitizeName(j.name)}" type="${type}">\n    <parent link="${parent}"/>\n    <child link="${child}"/>\n    ${originXml(j.origin)}\n`;
  if (type !== 'fixed') {
    s += `    <axis xyz="${j.axis.map((a) => +a.toFixed(6)).join(' ')}"/>\n`;
    if (type !== 'continuous') s += `    <limit lower="${lim.lo.toFixed(6)}" upper="${lim.hi.toFixed(6)}" effort="1000" velocity="${lim.v.toFixed(4)}"/>\n`;
    else s += `    <limit effort="1000" velocity="${lim.v.toFixed(4)}"/>\n`;
    if (j.mimic) s += `    <mimic joint="${sanitizeName(j.mimic.joint)}" multiplier="${j.mimic.multiplier}" offset="${j.type === 'prismatic' ? j.mimic.offset * M : j.mimic.offset * DEG}"/>\n`;
  }
  return s + '  </joint>\n';
}

/** URDF for one kinematic chain (robot). Link names are prefixed so several robots can share a package. */
export function chainToURDF(chain: ChainDef, name: string, mw: MeshWriter, opts: URDFExportOptions, prefix = ''): { xml: string; rootLink: string; tipLink: string } {
  const rn = sanitizeName(name);
  const lname = (i: number) => sanitizeName(`${prefix}${chain.links[i]?.name ?? (i === 0 ? 'base_link' : `link_${i}`)}`);
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

/** Export a single robot (with its tools as fixed links on the flange) as a URDF package. */
export function exportRobotURDF(robot: Robot, assets?: AssetStore, opts: URDFExportOptions = {}): URDFExportResult {
  const pkg = sanitizeName(opts.packageName ?? `${robot.name}_description`).toLowerCase();
  const mw = new MeshWriter(assets, pkg);
  const { xml, tipLink } = chainToURDF(robot.chain, robot.name, mw, opts);
  let body = xml;
  for (const t of robot.tools()) {
    const tn = sanitizeName(t.name);
    body += `  <link name="${tn}">\n`;
    for (const g of t.geometry) body += geometryXml(g, mw, tn, opts);
    body += `  </link>\n  <joint name="${tn}_joint" type="fixed">\n    <parent link="${tipLink}"/>\n    <child link="${tn}"/>\n    <origin xyz="0 0 0" rpy="0 0 0"/>\n  </joint>\n`;
    body += `  <link name="${tn}_tcp"/>\n  <joint name="${tn}_tcp_joint" type="fixed">\n    <parent link="${tn}"/>\n    <child link="${tn}_tcp"/>\n    ${originXml(t.poseTool())}\n  </joint>\n`;
  }
  const urdf = `<?xml version="1.0"?>\n<!-- Generated by VerticalBot Studio: ${robot.name} (${robot.brand}${robot.model ? ' ' + robot.model : ''}) -->\n<robot name="${sanitizeName(robot.name)}">\n${body}</robot>\n`;
  const files = { ...mw.files, [`urdf/${sanitizeName(robot.name)}.urdf`]: urdf, 'package.xml': packageXml(pkg, robot.name), 'CMakeLists.txt': cmake(pkg), 'README.md': readme(pkg, [robot.name]) };
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
  for (const r of robots) {
    const prefix = `${uniq(r.name)}_`;
    const { xml, rootLink, tipLink } = chainToURDF(r.chain, r.name, mw, opts, prefix);
    body += xml;
    body += `  <joint name="${prefix}base_joint" type="fixed">\n    <parent link="world"/>\n    <child link="${rootLink}"/>\n    ${originXml(r.poseAbs())}\n  </joint>\n`;
    for (const t of r.tools()) {
      const tn = uniq(`${prefix}${t.name}`);
      body += `  <link name="${tn}">\n`;
      for (const g of t.geometry) body += geometryXml(g, mw, tn, opts);
      body += `  </link>\n  <joint name="${tn}_joint" type="fixed">\n    <parent link="${tipLink}"/>\n    <child link="${tn}"/>\n    <origin xyz="0 0 0" rpy="0 0 0"/>\n  </joint>\n`;
    }
  }
  const visit = (it: Item) => {
    for (const c of it.children) {
      if (c instanceof Robot) continue; // robots (and their tools) handled above
      if (c instanceof SceneObject && !(c instanceof Tool) && c.geometry.length) {
        const ln = uniq(c.name);
        body += `  <link name="${ln}">\n`;
        for (const g of c.geometry) body += geometryXml(g, mw, ln, opts);
        body += `  </link>\n  <joint name="${ln}_joint" type="fixed">\n    <parent link="world"/>\n    <child link="${ln}"/>\n    ${originXml(c.poseAbs())}\n  </joint>\n`;
      } else if (c.type === ItemType.FRAME) {
        const ln = uniq(c.name);
        body += `  <link name="${ln}"/>\n  <joint name="${ln}_joint" type="fixed">\n    <parent link="world"/>\n    <child link="${ln}"/>\n    ${originXml(c.poseAbs())}\n  </joint>\n`;
      }
      visit(c);
    }
  };
  visit(station);
  const urdf = `<?xml version="1.0"?>\n<!-- Generated by VerticalBot Studio: station "${station.name}" -->\n<robot name="${sanitizeName(station.name)}">\n${body}</robot>\n`;
  const files = { ...mw.files, [`urdf/${sanitizeName(station.name)}.urdf`]: urdf, 'package.xml': packageXml(pkg, station.name), 'CMakeLists.txt': cmake(pkg), 'README.md': readme(pkg, robots.map((r) => r.name)) };
  return { files, urdf, packageName: pkg, warnings: mw.warnings };
}

function packageXml(pkg: string, title: string): string {
  return `<?xml version="1.0"?>\n<package format="3">\n  <name>${pkg}</name>\n  <version>0.1.0</version>\n  <description>${title} — URDF description exported from VerticalBot Studio</description>\n  <maintainer email="user@example.com">VerticalBot Studio</maintainer>\n  <license>Apache-2.0</license>\n  <buildtool_depend>ament_cmake</buildtool_depend>\n  <exec_depend>robot_state_publisher</exec_depend>\n  <export><build_type>ament_cmake</build_type></export>\n</package>\n`;
}
function cmake(pkg: string): string {
  return `cmake_minimum_required(VERSION 3.8)\nproject(${pkg})\nfind_package(ament_cmake REQUIRED)\ninstall(DIRECTORY urdf meshes DESTINATION share/\${PROJECT_NAME})\nament_package()\n`;
}
function readme(pkg: string, robots: string[]): string {
  return `# ${pkg}\n\nExported from VerticalBot Studio. Robots: ${robots.join(', ') || '—'}.\n\n- ROS 2: copy into your workspace \`src/\`, \`colcon build\`, then \`ros2 launch robot_state_publisher ...\` or open in RViz.\n- Blender: install the Phobos add-on and use *Import > URDF*; meshes are binary STL in metres.\n- Back to the studio: drop the \`.urdf\` together with the \`meshes/*.stl\` files onto the 3D view.\n`;
}

/** Zip the exported files under a top-level folder named after the package. */
export function packageZip(res: URDFExportResult): Uint8Array {
  const entries: Record<string, Uint8Array> = {};
  for (const [p, c] of Object.entries(res.files)) entries[`${res.packageName}/${p}`] = typeof c === 'string' ? strToU8(c) : c;
  return zipSync(entries, { level: 6 });
}

export { multiply };
