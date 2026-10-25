/**
 * SRDF (MoveIt semantic robot description): planning groups (chains, joints, links, subgroups), named group states,
 * end effectors, virtual / passive joints and the disabled self-collision pairs. Import applies them to the station
 * items of a URDF import (named poses per robot, chain flanges from group chains, collision filter), export writes a
 * basic SRDF for each studio robot (one chain group, `home` state, adjacent-link collision filter).
 */
import { parseXML, childElements, XNode } from './xml';
import type { Robot } from '../../core/items/robot';
import type { Item } from '../../core/items/item';
import type { URDFModel } from './urdf';
import { RAD } from '../../core/math/pose';

export interface SRDFGroup { name: string; chains: { base: string; tip: string }[]; joints: string[]; links: string[]; subgroups: string[] }
export interface SRDFModel {
  name: string;
  groups: SRDFGroup[];
  /** group state name -> joint values in URDF units (rad / m) */
  states: { name: string; group: string; joints: Record<string, number> }[];
  endEffectors: { name: string; group: string; parentLink: string; parentGroup?: string }[];
  virtualJoints: { name: string; type: string; parent: string; child: string }[];
  passiveJoints: string[];
  disabledCollisions: { link1: string; link2: string; reason: string }[];
}

export function parseSRDF(text: string): SRDFModel {
  const root = parseXML(text);
  if (root.name !== 'robot') throw new Error('Not an SRDF file (missing <robot> root)');
  const groups: SRDFGroup[] = childElements(root, 'group').map((g) => ({
    name: g.attrs.name,
    chains: childElements(g, 'chain').map((c) => ({ base: c.attrs.base_link, tip: c.attrs.tip_link })),
    joints: childElements(g, 'joint').map((j) => j.attrs.name),
    links: childElements(g, 'link').map((l) => l.attrs.name),
    subgroups: childElements(g, 'group').map((s) => s.attrs.name),
  }));
  const states = childElements(root, 'group_state').map((s) => ({ name: s.attrs.name, group: s.attrs.group, joints: Object.fromEntries(childElements(s, 'joint').map((j) => [j.attrs.name, +(j.attrs.value ?? 0)])) }));
  return {
    name: root.attrs.name ?? '',
    groups,
    states,
    endEffectors: childElements(root, 'end_effector').map((e) => ({ name: e.attrs.name, group: e.attrs.group, parentLink: e.attrs.parent_link, parentGroup: e.attrs.parent_group })),
    virtualJoints: childElements(root, 'virtual_joint').map((v) => ({ name: v.attrs.name, type: v.attrs.type, parent: v.attrs.parent_frame, child: v.attrs.child_link })),
    passiveJoints: childElements(root, 'passive_joint').map((p) => p.attrs.name),
    disabledCollisions: childElements(root, 'disable_collisions').map((d) => ({ link1: d.attrs.link1, link2: d.attrs.link2, reason: d.attrs.reason ?? '' })),
  };
}

/** Tip links of the SRDF chain groups (used as chain flanges by the tree import). */
export function srdfTipLinks(s: SRDFModel): string[] {
  return [...new Set(s.groups.flatMap((g) => g.chains.map((c) => c.tip)))];
}

/** Joints of a group (chains resolved through the URDF, subgroups recursively). */
export function srdfGroupJoints(s: SRDFModel, model: URDFModel, group: string, seen = new Set<string>()): string[] {
  const g = s.groups.find((x) => x.name === group);
  if (!g || seen.has(group)) return [];
  seen.add(group);
  const parentJoint = new Map(model.joints.map((j) => [j.child, j]));
  const out: string[] = [...g.joints];
  for (const c of g.chains) {
    const path: string[] = [];
    for (let l = c.tip; l && l !== c.base; ) { const pj = parentJoint.get(l); if (!pj) break; path.unshift(pj.name); l = pj.parent; }
    out.push(...path);
  }
  for (const l of g.links) { const pj = parentJoint.get(l); if (pj) out.push(pj.name); }
  for (const sg of g.subgroups) out.push(...srdfGroupJoints(s, model, sg, seen));
  const movable = new Set(model.joints.filter((j) => j.type !== 'fixed').map((j) => j.name));
  return [...new Set(out)].filter((j) => movable.has(j));
}

/**
 * Apply an SRDF to the items of a URDF import: each robot gets `params.planningGroups` (groups whose joints it owns)
 * and `params.namedPoses` (group states, deg / mm in its joint order, missing joints at home); the root item gets the
 * disabled collision pairs (`params.disabledCollisions`) and end effectors.
 */
export function applySRDF(s: SRDFModel, model: URDFModel, root: Item, robots: Robot[]): string[] {
  const report: string[] = [];
  const typeOf = new Map(model.joints.map((j) => [j.name, j.type]));
  for (const r of robots) {
    const names = r.jointNames();
    const groups = s.groups.filter((g) => srdfGroupJoints(s, model, g.name).some((j) => names.includes(j))).map((g) => g.name);
    if (groups.length) r.params.planningGroups = groups;
    const poses: Record<string, number[]> = {};
    for (const st of s.states) {
      if (!groups.includes(st.group) || !Object.keys(st.joints).some((j) => names.includes(j))) continue;
      const home = r.jointsHome();
      poses[st.name] = names.map((n, i) => (n in st.joints ? st.joints[n] * (typeOf.get(n) === 'prismatic' ? 1000 : RAD) : home[i]));
    }
    if (Object.keys(poses).length) { r.params.namedPoses = poses; report.push(`${r.name}: named poses ${Object.keys(poses).join(', ')} (SRDF ${s.name})`); }
  }
  if (s.disabledCollisions.length) root.params.disabledCollisions = s.disabledCollisions.map((d) => [d.link1, d.link2, d.reason]);
  if (s.endEffectors.length) root.params.endEffectors = s.endEffectors.map((e) => ({ name: e.name, group: e.group, parentLink: e.parentLink }));
  root.params.srdf = s.name;
  report.push(`SRDF ${s.name}: ${s.groups.length} groups, ${s.states.length} states, ${s.disabledCollisions.length} disabled collision pairs`);
  return report;
}

const esc = (v: string) => v.replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/"/g, '&quot;');

/** Basic SRDF for one robot as exported to URDF (link names = URDF link names of the export). */
export function exportSRDF(robotName: string, groups: { name: string; base: string; tip: string }[], states: { name: string; group: string; joints: Record<string, number> }[], adjacent: [string, string][], extra: { link1: string; link2: string; reason: string }[] = []): string {
  let s = `<?xml version="1.0" encoding="UTF-8"?>\n<!-- Generated by VerticalBot Studio -->\n<robot name="${esc(robotName)}">\n`;
  for (const g of groups) s += `  <group name="${esc(g.name)}">\n    <chain base_link="${esc(g.base)}" tip_link="${esc(g.tip)}"/>\n  </group>\n`;
  for (const st of states) {
    s += `  <group_state name="${esc(st.name)}" group="${esc(st.group)}">\n`;
    for (const [j, v] of Object.entries(st.joints)) s += `    <joint name="${esc(j)}" value="${+v.toFixed(9)}"/>\n`;
    s += '  </group_state>\n';
  }
  for (const [a, b] of adjacent) s += `  <disable_collisions link1="${esc(a)}" link2="${esc(b)}" reason="Adjacent"/>\n`;
  for (const d of extra) s += `  <disable_collisions link1="${esc(d.link1)}" link2="${esc(d.link2)}" reason="${esc(d.reason)}"/>\n`;
  return `${s}</robot>\n`;
}

export type { XNode };
