/**
 * ROS package / workspace import (interface I7): a dropped folder or zip of one or more ROS packages becomes station
 * items. Packages are discovered from `package.xml`; `package://pkg/...` and `$(find pkg)` resolve inside the drop;
 * robot description entry points come from launch files (`xacro` commands / `os.path.join(...)` / PathJoinSubstitution /
 * pathlib, mappings and `DeclareLaunchArgument` defaults; ROS 1 `<param command="xacro ...">` too) or, without launch
 * files, from the top-level URDF/xacro files (macro-only xacros, files included by others and webots_ros2 plugin
 * manifests are not robots). Every entry is expanded with the ROS 2 xacro semantics (errors are reported, never a
 * 0-DOF robot) and imported as a kinematic tree (./urdf importURDF). Only meshes referenced by an imported robot are
 * registered — under their `package://` URI, STL/OBJ in metres, COLLADA honouring its <unit> — never as stray objects.
 * An SRDF of the same robot (MoveIt config) adds planning groups, named poses, chain tips and the collision filter.
 */
import { unzipSync, strFromU8 } from 'fflate';
import type { Item } from '../../core/items/item';
import type { Robot } from '../../core/items/robot';
import type { AssetStore } from '../../scene/assets';
import { expandXacro, isXacro, joinPath, dirnameOf, XacroError } from './xacro';
import { parseURDF, importURDF, URDFModel, defaultTipLinks } from './urdf';
import { parseSRDF, applySRDF, srdfTipLinks, SRDFModel } from './srdf';
import { parseXML, childElement } from './xml';
import { transl } from '../../core/math/pose';

export interface RosEntry {
  /** Path of the URDF / xacro file inside the drop. */
  file: string;
  package?: string;
  args: Record<string, string>;
  /** Where the entry came from (launch file path, or 'top-level'). */
  source: string;
}

export interface RosPackageImportOptions {
  /** xacro args for every entry (override launch defaults). */
  args?: Record<string, string>;
  /** Only import these entry files (paths as listed in `entries` of a previous result). */
  entries?: string[];
  /** Import every top-level robot file even when launch files name only some of them. */
  all?: boolean;
}

export interface RosPackageImportResult {
  items: Item[];
  robots: Robot[];
  report: string[];
  errors: string[];
  /** All robot description entry points found (the imported ones and the alternatives). */
  entries: RosEntry[];
  packages: Record<string, string>;
  models: URDFModel[];
}

const norm = (p: string) => p.replace(/\\/g, '/').replace(/^\.\//, '').replace(/\/{2,}/g, '/');
const text = (b: Uint8Array) => strFromU8(b).replace(/^﻿/, '');
const MESH_RE = /\.(stl|obj|dae)$/i;

/** Normalize a drop: unify separators, expand nested zips (entries prefixed with the zip name). */
export function normalizeDrop(files: Map<string, Uint8Array>): Map<string, Uint8Array> {
  const out = new Map<string, Uint8Array>();
  for (const [k, v] of files) {
    const p = norm(k);
    if (/\.zip$/i.test(p)) {
      const base = p.replace(/\.zip$/i, '');
      for (const [zk, zv] of Object.entries(unzipSync(v))) if (!zk.endsWith('/')) out.set(norm(`${base}/${zk}`), zv);
    } else out.set(p, v);
  }
  return out;
}

/** package name -> root directory (from package.xml). */
export function discoverPackages(files: Map<string, Uint8Array>): Record<string, string> {
  const pkgs: Record<string, string> = {};
  for (const [p, b] of files) {
    if (!/(^|\/)package\.xml$/.test(p)) continue;
    try {
      const name = childElement(parseXML(text(b)), 'name')?.children.find((c) => c.type === 'text')?.text?.trim();
      if (name && !(name in pkgs)) pkgs[name] = dirnameOf(p) === '.' ? '' : dirnameOf(p);
    } catch { /* not a package manifest */ }
  }
  return pkgs;
}

/** Parse launch files (Python / XML) for robot description entry points and their xacro args. */
export function launchEntries(files: Map<string, Uint8Array>, pkgs: Record<string, string>, exists: (p: string) => string | null): RosEntry[] {
  const out: RosEntry[] = [];
  for (const [p, b] of files) {
    if (!/(\.launch\.py|_launch\.py|\.launch\.xml|\.launch)$/.test(p)) continue;
    const src = text(b);
    const own = Object.entries(pkgs).find(([, root]) => p.startsWith(root ? `${root}/` : ''))?.[0];
    if (p.endsWith('.py')) {
      const defaults: Record<string, string> = {};
      for (const m of src.matchAll(/DeclareLaunchArgument\(\s*(?:name\s*=\s*)?['"]([\w/]+)['"]([^)]*)\)/g)) {
        const d = /default_value\s*=\s*['"]([^'"]*)['"]/.exec(m[2]);
        defaults[m[1]] = d ? d[1] : '';
      }
      const lcVar: Record<string, string> = {};
      for (const m of src.matchAll(/(\w+)\s*=\s*LaunchConfiguration\(\s*['"](\w+)['"]/g)) lcVar[m[1]] = m[2];
      const referenced = [...src.matchAll(/(?:get_package_share_directory|FindPackageShare|get_package_share_path)\(\s*(?:package\s*=\s*)?['"]([\w]+)['"]/g)].map((m) => m[1]);
      for (const m of src.matchAll(/['"]([^'"\n]+\.(?:urdf|xacro))['"]/g)) {
        // quoted path segments right before the file literal: os.path.join(pkg, 'urdf', 'x'), [pkg, 'urdf', 'x'], pkg / 'urdf' / 'x'
        const before = src.slice(Math.max(0, m.index! - 300), m.index!);
        const segs: string[] = [];
        const tail = /((?:['"][^'"\n]+['"]\s*(?:,|\/)\s*)*)$/.exec(before)?.[1] ?? '';
        for (const s of tail.matchAll(/['"]([^'"\n]+)['"]/g)) segs.push(s[1]);
        segs.push(m[1]);
        const rel = segs.join('/');
        const cands = [...new Set([...referenced, ...(own ? [own] : [])])].map((k) => ({ pkg: k, path: joinPath(pkgs[k] ?? '', rel) }));
        let hit = cands.find((c) => exists(c.path));
        if (!hit) { const f = [...files.keys()].find((k) => k === rel || k.endsWith(`/${rel}`)); if (f) hit = { pkg: Object.entries(pkgs).find(([, r]) => f.startsWith(r ? `${r}/` : ''))?.[0] ?? '', path: f }; }
        if (!hit) continue;
        const args: Record<string, string> = {};
        for (const a of src.matchAll(/['"]\s*([\w]+):=\s*['"]\s*,\s*(?:LaunchConfiguration\(\s*['"](\w+)['"]\s*\)|(\w+))/g)) {
          const lc = a[2] ?? lcVar[a[3]];
          if (lc !== undefined) args[a[1]] = defaults[lc] ?? '';
        }
        for (const a of src.matchAll(/['"]\s*([\w]+):=([^'"\s]+)\s*['"]/g)) args[a[1]] = a[2];
        for (const a of src.matchAll(/mappings\s*=\s*\{([^}]*)\}/g)) for (const kv of a[1].matchAll(/['"](\w+)['"]\s*:\s*['"]([^'"]*)['"]/g)) args[kv[1]] = kv[2];
        if (!out.some((e) => e.file === hit!.path && JSON.stringify(e.args) === JSON.stringify(args))) out.push({ file: hit.path, package: hit.pkg || undefined, args, source: p });
      }
    } else {
      const defaults: Record<string, string> = {};
      for (const m of src.matchAll(/<arg\s+[^>]*name="([\w/]+)"[^>]*?(?:default|value)="([^"]*)"/g)) defaults[m[1]] = m[2];
      const sub = (s: string) => s.replace(/\$\(arg ([\w/]+)\)/g, (_x, n) => defaults[n] ?? '');
      for (const m of sub(src).matchAll(/\$\(find(?:-pkg-share)? (\w+)\)\/([^\s'"]+\.(?:urdf|xacro))((?:\s+[\w]+:=[^\s'"]*)*)/g)) {
        const path = joinPath(pkgs[m[1]] ?? '', m[2]);
        if (!exists(path)) continue;
        const args: Record<string, string> = {};
        for (const a of m[3].matchAll(/([\w]+):=([^\s'"]*)/g)) args[a[1]] = a[2];
        if (!out.some((e) => e.file === path && JSON.stringify(e.args) === JSON.stringify(args))) out.push({ file: path, package: m[1], args, source: p });
      }
    }
  }
  return out;
}

export async function importRosPackageFiles(filesIn: Map<string, Uint8Array>, assets?: AssetStore, opts: RosPackageImportOptions = {}): Promise<RosPackageImportResult> {
  const files = normalizeDrop(filesIn);
  const packages = discoverPackages(files);
  const report: string[] = [];
  const errors: string[] = [];
  const textCache = new Map<string, string>();
  const fileText = (p: string): string | null => {
    if (textCache.has(p)) return textCache.get(p)!;
    const b = files.get(p);
    if (!b) return null;
    const t = text(b);
    textCache.set(p, t);
    return t;
  };
  const pkgOf = (path: string) => Object.entries(packages).filter(([, r]) => r === '' || path === r || path.startsWith(`${r}/`)).sort((a, b) => b[1].length - a[1].length)[0];
  /** URI / path -> file in the drop (package map first, then a unique path-suffix match for absolute / unknown roots). */
  const locate = (uri: string, baseDir = ''): string | null => {
    let p = uri.replace(/^file:\/\//, '');
    const m = /^package:\/\/([^/]+)\/(.*)$/.exec(p);
    let rel = '';
    if (m) {
      if (m[1] in packages) { const q = joinPath(packages[m[1]], m[2]); if (files.has(q)) return q; }
      rel = `${m[1]}/${m[2]}`;
      p = m[2];
    } else if (!/^(\/|[A-Za-z]:\/)/.test(p)) { const q = joinPath(baseDir, p); if (files.has(q)) return q; rel = q; }
    else rel = p.replace(/^.*\/share\//, '');
    const tails = [rel, p].filter(Boolean).map((t) => norm(t).replace(/^\/+/, ''));
    for (const t of tails) {
      const hits = [...files.keys()].filter((k) => k === t || k.endsWith(`/${t}`));
      if (hits.length === 1) return hits[0];
    }
    return null;
  };
  const exists = (p: string) => (files.has(p) ? p : null);
  const resolveInclude = (path: string) => { const f = locate(path); return f ? fileText(f) : null; };
  const canonical = (fsPath: string) => { const pk = pkgOf(fsPath); return pk ? `package://${pk[0]}/${pk[1] ? fsPath.slice(pk[1].length + 1) : fsPath}` : fsPath; };

  // ---- entry points
  const launch = launchEntries(files, packages, exists);
  const candidates: RosEntry[] = [];
  for (const [p] of files) {
    if (!/\.(urdf|xacro)$/i.test(p)) continue;
    const t = fileText(p) ?? '';
    if (!/<robot[\s>]/.test(t)) continue;
    candidates.push({ file: p, package: pkgOf(p)?.[0], args: {}, source: 'top-level' });
  }
  type Expanded = { entry: RosEntry; xml: string; includes: string[]; links: number; robot: boolean; signature: string };
  const expanded: Expanded[] = [];
  const failed: Array<{ entry: RosEntry; msg: string }> = [];
  const expand = (e: RosEntry): Expanded | null => {
    const t = fileText(e.file);
    if (t === null) { failed.push({ entry: e, msg: `${e.file}: file not found` }); return null; }
    try {
      const pk = e.package ?? pkgOf(e.file)?.[0];
      const filename = pk ? canonical(e.file) : e.file;
      const res = isXacro(t) ? expandXacro(t, { filename, packages, resolveInclude, args: { ...e.args, ...(opts.args ?? {}) }, env: {} }) : { xml: t, includes: [] as string[], warnings: [] as string[] };
      for (const w of res.warnings) if (!/redefining global symbol|deprecated/.test(w)) report.push(`${e.file}: ${w}`);
      const links = (res.xml.match(/<link[\s>]/g) ?? []).length;
      // the expanded document must be a URDF <robot> (a launch file may also expand an SDF / world xacro for Gazebo)
      const root = rootElement(res.xml);
      const robot = root === 'robot';
      if (!robot && links > 0 && !report.some((r) => r.startsWith(`${e.file}: <${root}>`))) report.push(`${e.file}: <${root}> document (not a URDF robot), skipped`);
      return { entry: e, xml: res.xml, includes: res.includes.map((i) => locate(i) ?? i), links, robot, signature: robot ? robotSignature(res.xml) : '' };
    } catch (err) {
      failed.push({ entry: e, msg: `${e.file}${Object.keys(e.args).length ? ` (${Object.entries(e.args).map(([k, v]) => `${k}:=${v}`).join(' ')})` : ''}: ${(err as Error).message}` });
      return null;
    }
  };
  for (const e of [...launch, ...candidates]) { const x = expand(e); if (x) expanded.push(x); }
  const included = new Set(expanded.flatMap((x) => x.includes));
  const robotsFound = expanded.filter((x) => x.robot && x.links > 0 && (x.entry.source !== 'top-level' || !included.has(x.entry.file)));
  // one result per file: a launch entry wins over the bare top-level expansion of the same file
  const byFile = new Map<string, Expanded[]>();
  for (const x of robotsFound) { const l = byFile.get(x.entry.file) ?? []; l.push(x); byFile.set(x.entry.file, l); }
  const entries: RosEntry[] = [];
  for (const [, list] of byFile) { const fromLaunch = list.filter((x) => x.entry.source !== 'top-level'); for (const x of fromLaunch.length ? fromLaunch : list.slice(0, 1)) entries.push(x.entry); }
  const exp = (e: RosEntry) => expanded.find((y) => y.entry === e)!;
  // the same robot reached twice (its xacro from one launch file and the pre-expanded URDF from another, or two launch
  // files with the same arguments): one copy; the other is listed as an alternative
  const sameAs = new Map<RosEntry, RosEntry>();
  entries.forEach((e, k) => { const first = entries.slice(0, k).find((f) => !sameAs.has(f) && exp(f).signature === exp(e).signature); if (first) sameAs.set(e, first); });
  const robotEntries = entries.filter((e) => !sameAs.has(e));
  let chosen = robotEntries;
  if (opts.entries?.length) chosen = entries.filter((e) => opts.entries!.includes(e.file));
  else if (!opts.all && launch.length && robotEntries.some((e) => e.source !== 'top-level')) chosen = robotEntries.filter((e) => e.source !== 'top-level');
  for (const e of entries) {
    if (chosen.includes(e)) continue;
    const twin = sameAs.get(e);
    report.push(twin && chosen.includes(twin) ? `Same robot as ${twin.file} (${twin.source}): ${e.file} (${e.source}) is not imported twice` : `Also available: ${e.file} (${e.source})`);
  }
  // errors: only for files that would have been robots (top-level and not included elsewhere), launch entries always —
  // unless the same file expanded fine for another entry (the bare top-level expansion of a xacro that needs the
  // arguments its launch file passes)
  const okFiles = new Set(robotsFound.map((x) => x.entry.file));
  const reported = new Set<string>();
  for (const { entry: e, msg } of failed) {
    const f = e.file;
    if (included.has(f) || okFiles.has(f) || reported.has(msg)) continue;
    if (e.source !== 'top-level' || !entries.some((x) => x.file === f)) { errors.push(msg); reported.add(msg); }
  }

  // ---- SRDFs
  const srdfs: SRDFModel[] = [];
  for (const [p] of files) if (/\.srdf$/i.test(p)) { try { srdfs.push(parseSRDF(fileText(p)!)); } catch (err) { errors.push(`${p}: ${(err as Error).message}`); } }

  // ---- import
  const items: Item[] = [];
  const robots: Robot[] = [];
  const models: URDFModel[] = [];
  const missing = new Set<string>();
  let offset = 0;
  const importOne = (e: RosEntry): boolean => {
    const x = exp(e);
    try {
      const model = parseURDF(x.xml);
      const baseDir = dirnameOf(e.file);
      const ids = new Map<string, string>();
      for (const uri of model.meshes) {
        const f = locate(uri, baseDir);
        if (!f) { missing.add(uri); continue; }
        const id = canonical(f);
        ids.set(uri, id);
        if (assets && !assets.has(id) && MESH_RE.test(f)) {
          const ext = f.split('.').pop()!.toLowerCase() as 'stl' | 'obj' | 'dae';
          try { assets.registerRaw(id, ext, files.get(f)!, f.split('/').pop(), 1, ext === 'dae' ? 'm' : undefined); } catch (err) { report.push(`mesh ${f}: ${(err as Error).message}`); }
        }
      }
      const srdf = srdfs.find((s) => s.name === model.name);
      const tips = [...new Set([...(srdf ? srdfTipLinks(srdf) : []), ...defaultTipLinks(model)])].filter((l) => model.links.has(l));
      const res = importURDF(x.xml, { meshId: (u) => ids.get(u) ?? u, assets, tipLinks: tips });
      if (srdf) report.push(...applySRDF(srdf, res.model, res.root, res.robots));
      res.root.params.rosPackage = e.package ?? '';
      res.root.params.urdfFile = e.file;
      if (Object.keys(e.args).length) res.root.params.xacroArgs = { ...e.args };
      if (offset) res.root.setPose(transl(0, offset, 0));
      offset += 1500;
      items.push(res.root);
      robots.push(...res.robots);
      models.push(res.model);
      const dof = res.robots.reduce((n, r) => n + r.dof, 0);
      report.push(`Imported ${model.name} from ${e.file}${Object.keys(e.args).length ? ` [${Object.entries(e.args).map(([k, v]) => `${k}:=${v}`).join(' ')}]` : ''}: ${res.robots.length} robot(s), ${dof} DOF${res.mobile ? `, mobile base (${res.mobile.kin.drive})` : ''}`);
      report.push(...res.report);
      return true;
    } catch (err) {
      errors.push(`${e.file}: ${(err as Error).message}`);
      return false;
    }
  };
  let imported = 0;
  for (const e of chosen) if (importOne(e)) imported++;
  // no robot of the launch files could be imported: the package's other robot descriptions are what the user dropped
  if (!imported && !opts.entries?.length) {
    for (const e of robotEntries.filter((x) => !chosen.includes(x))) if (importOne(e)) { imported++; report.push(`Imported ${e.file} (${e.source}) instead: no robot of the launch files could be imported`); }
  }
  if (missing.size) report.push(`${missing.size} mesh file(s) not in the drop: ${[...missing].slice(0, 8).join(', ')}${missing.size > 8 ? ' …' : ''}`);
  if (!chosen.length && !errors.length) errors.push('No robot description (URDF / xacro with links) found in the dropped files');
  return { items, robots, report, errors, entries, packages, models };
}

/** Name of the root element of an XML document (after the declaration, comments and a DOCTYPE). */
export function rootElement(xml: string): string {
  const body = xml.replace(/<\?[\s\S]*?\?>/g, '').replace(/<!--[\s\S]*?-->/g, '').replace(/<!DOCTYPE[^>]*>/gi, '');
  return /<([A-Za-z_][\w:.-]*)/.exec(body)?.[1] ?? '';
}

/** Identity of an expanded URDF: robot name, links and joints (name, type, parent, child). Two files describing the same
 *  robot (a xacro and its pre-expanded URDF) give the same signature. */
export function robotSignature(xml: string): string {
  try {
    const m = parseURDF(xml);
    const joints = [...m.joints.values()].map((j) => `${j.name}:${j.type}:${j.parent}>${j.child}`).sort();
    return `${m.name}|${[...m.links.keys()].sort().join(',')}|${joints.join(',')}`;
  } catch { return `unparsed:${xml.length}:${xml.slice(0, 200)}`; }
}

export { XacroError };
