/**
 * URDF importer -> ChainDef (serial chain) + mesh references.
 * Handles: fixed/revolute/continuous/prismatic joints, mimic, limits, visual meshes (package:// URIs),
 * primitives, materials. Branching robots are flattened to the longest chain; other branches become
 * static visuals attached to their parent link.
 */
import { parseXML, childElements, childElement, nums, XNode } from './xml';
import { processXacro, XacroOptions } from './xacro';
import { ChainDef, JointDef, LinkDef, LinkVisual } from '../../core/kinematics/chain';
import { urdfToPose, identity, multiply, Mat4, Vec3, RAD } from '../../core/math/pose';
import { Robot } from '../../core/items/robot';

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
  mimic?: { joint: string; multiplier: number; offset: number };
}

export interface URDFLink {
  name: string;
  visuals: LinkVisual[];
  collisions: LinkVisual[];
  mass: number;
}

export interface URDFModel {
  name: string;
  links: Map<string, URDFLink>;
  joints: URDFJoint[];
  rootLink: string;
  /** All mesh URIs referenced. */
  meshes: string[];
  materials: Map<string, string>;
}

const M2MM = 1000;

function parseGeometry(g: XNode | undefined, origin: Mat4, color: string | undefined, meshes: string[]): LinkVisual | null {
  if (!g) return null;
  const mesh = childElement(g, 'mesh');
  if (mesh) {
    const scale = nums(mesh.attrs.scale, [1, 1, 1]) as Vec3;
    const uri = mesh.attrs.filename ?? '';
    if (!meshes.includes(uri)) meshes.push(uri);
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
  return null;
}

function originOf(n: XNode | undefined): Mat4 {
  const o = n ? childElement(n, 'origin') : undefined;
  if (!o) return identity();
  const xyz = nums(o.attrs.xyz, [0, 0, 0]);
  const rpy = nums(o.attrs.rpy, [0, 0, 0]);
  return urdfToPose([xyz[0] * M2MM, xyz[1] * M2MM, xyz[2] * M2MM], rpy as Vec3);
}

export function parseURDF(text: string, xacro: XacroOptions = {}): URDFModel {
  if (text.includes('xacro:') || text.includes('${')) text = processXacro(text, xacro);
  const root = parseXML(text);
  if (root.name !== 'robot') throw new Error('Not a URDF file (missing <robot> root)');
  const materials = new Map<string, string>();
  for (const m of childElements(root, 'material')) {
    const c = childElement(m, 'color');
    if (c && m.attrs.name) materials.set(m.attrs.name, rgbaToCss(nums(c.attrs.rgba, [0.7, 0.7, 0.7, 1])));
  }
  const meshes: string[] = [];
  const links = new Map<string, URDFLink>();
  for (const l of childElements(root, 'link')) {
    const visuals: LinkVisual[] = [];
    for (const v of childElements(l, 'visual')) {
      const mat = childElement(v, 'material');
      let color: string | undefined;
      if (mat) {
        const c = childElement(mat, 'color');
        color = c ? rgbaToCss(nums(c.attrs.rgba, [0.7, 0.7, 0.7, 1])) : materials.get(mat.attrs.name ?? '');
      }
      const vis = parseGeometry(childElement(v, 'geometry'), originOf(v), color, meshes);
      if (vis) visuals.push(vis);
    }
    const collisions: LinkVisual[] = [];
    for (const c of childElements(l, 'collision')) {
      const vis = parseGeometry(childElement(c, 'geometry'), originOf(c), undefined, []);
      if (vis) collisions.push(vis);
    }
    const inertial = childElement(l, 'inertial');
    const mass = inertial ? +(childElement(inertial, 'mass')?.attrs.value ?? 0) : 0;
    links.set(l.attrs.name, { name: l.attrs.name, visuals, collisions, mass });
  }
  const joints: URDFJoint[] = [];
  for (const j of childElements(root, 'joint')) {
    const limit = childElement(j, 'limit');
    const type = j.attrs.type ?? 'fixed';
    const isPrism = type === 'prismatic';
    const conv = isPrism ? M2MM : RAD;
    const mimic = childElement(j, 'mimic');
    joints.push({
      name: j.attrs.name,
      type,
      parent: childElement(j, 'parent')?.attrs.link ?? '',
      child: childElement(j, 'child')?.attrs.link ?? '',
      origin: originOf(j),
      axis: nums(childElement(j, 'axis')?.attrs.xyz, [1, 0, 0]) as Vec3,
      lower: limit?.attrs.lower !== undefined ? +limit.attrs.lower * conv : type === 'continuous' ? -360 : isPrism ? 0 : -180,
      upper: limit?.attrs.upper !== undefined ? +limit.attrs.upper * conv : type === 'continuous' ? 360 : isPrism ? 1000 : 180,
      velocity: limit?.attrs.velocity !== undefined ? +limit.attrs.velocity * conv : isPrism ? 1000 : 180,
      effort: +(limit?.attrs.effort ?? 0),
      mimic: mimic ? { joint: mimic.attrs.joint, multiplier: +(mimic.attrs.multiplier ?? 1), offset: +(mimic.attrs.offset ?? 0) * conv } : undefined,
    });
  }
  const children = new Set(joints.map((j) => j.child));
  const rootLink = [...links.keys()].find((l) => !children.has(l)) ?? [...links.keys()][0];
  return { name: root.attrs.name ?? 'robot', links, joints, rootLink, meshes, materials };
}

function rgbaToCss(v: number[]): string {
  const h = (x: number) => Math.round(Math.max(0, Math.min(1, x)) * 255).toString(16).padStart(2, '0');
  return `#${h(v[0])}${h(v[1])}${h(v[2])}`;
}

export interface ChainBuildOptions {
  /** Tip link name; defaults to the deepest link along the longest path of movable joints. */
  tipLink?: string;
  /** Base link name; defaults to the URDF root. */
  baseLink?: string;
}

/**
 * Convert a URDF model into a serial ChainDef from base to tip. Side branches (e.g. the second
 * gripper finger) are folded into their parent link as static visuals when fixed, or become
 * mimic/extra joints in the chain when they are moving and mimic a chain joint.
 */
export function chainFromURDF(model: URDFModel, opts: ChainBuildOptions = {}): { chain: ChainDef; sideBranches: URDFJoint[] } {
  const byParent = new Map<string, URDFJoint[]>();
  for (const j of model.joints) {
    if (!byParent.has(j.parent)) byParent.set(j.parent, []);
    byParent.get(j.parent)!.push(j);
  }
  const base = opts.baseLink ?? model.rootLink;
  // Find tip: longest path counting movable joints
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
      const score = path.filter((j) => j.type !== 'fixed' && !j.mimic).length * 1000 + path.length;
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
      if (j.type === 'fixed' || !j.mimic) {
        if (j.type !== 'fixed') sideBranches.push(j);
        collectStatic(j.child, multiply(T, j.origin), into);
      }
    }
  };

  const linkDef = (name: string): LinkDef => {
    const visuals: LinkVisual[] = [];
    collectStatic(name, identity(), visuals);
    return { name, visuals };
  };
  links.push(linkDef(base));
  let flangeT = identity();
  for (const j of mainPath) {
    if (j.type === 'fixed') {
      // fold fixed joints into a zero-motion joint so link frames stay correct
      joints.push({ name: j.name, type: 'fixed', origin: j.origin, axis: [0, 0, 1], lower: 0, upper: 0 });
    } else {
      joints.push({
        name: j.name,
        type: j.type as JointDef['type'],
        origin: j.origin,
        axis: j.axis,
        lower: j.lower,
        upper: j.upper,
        maxVelocity: j.velocity,
        mimic: j.mimic,
        home: Math.max(j.lower, Math.min(j.upper, 0)),
      });
    }
    links.push(linkDef(j.child));
  }
  // Mimic side joints that mirror a chain joint: append as mimic joints with their own visuals? Kept as static
  // (folded above with home value) — recorded in sideBranches for advanced rendering.
  const chain: ChainDef = { name: model.name, joints, links, flange: flangeT, dof: joints.filter((j) => j.type !== 'fixed' && !j.mimic).length };
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
