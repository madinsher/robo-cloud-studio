/**
 * Helpers for the ROS ground-truth tests (T5): canonical XML tree comparison against real `xacro` output, a resolver over
 * a directory / package map, and access to the MURKA / WSL inputs (absent -> tests skip visibly).
 */
import { readFileSync, existsSync, readdirSync, statSync } from 'node:fs';
import { join, dirname, resolve } from 'node:path';
import { parseXML, XNode } from '../src/io/urdf/xml';

export const FIX = resolve(__dirname, 'fixtures/ros_gt');
export const MURKA = 'C:/Users/TRITON 300 SE/YandexDisk/Работа/КБНЦ РАН/MURKA_Simulation';
export const HAS_MURKA = existsSync(MURKA);
export const VBOT = 'C:/vertical_bot_ros';
export const HAS_VBOT = existsSync(VBOT);

/** Absolute GT paths of `$(find pkg)` (`.../share/<pkg>`, `file://...`) become `package://<pkg>` like the studio emits. */
export function normalizeFind(s: string): string {
  return s.replace(/(?:file:\/\/)?\/[^\s"'<>]*?\/share\/([A-Za-z0-9_]+)(?=[/"'\s<]|$)/g, 'package://$1');
}

export interface Diff { path: string; msg: string }
/** Compare two element trees: names, attribute maps (exact strings), element children in order, trimmed text. */
export function diffTrees(a: XNode, b: XNode, path = '', out: Diff[] = [], norm = (s: string) => s): Diff[] {
  const here = `${path}/${a.name}${a.attrs.name ? `[@name=${a.attrs.name}]` : ''}`;
  if (a.name !== b.name) { out.push({ path: here, msg: `tag ${a.name} != ${b.name}` }); return out; }
  const ka = Object.keys(a.attrs).filter((k) => !k.startsWith('xmlns')).sort(), kb = Object.keys(b.attrs).filter((k) => !k.startsWith('xmlns')).sort();
  if (ka.join() !== kb.join()) out.push({ path: here, msg: `attrs {${ka}} != {${kb}}` });
  for (const k of ka) if (b.attrs[k] !== undefined && norm(a.attrs[k]) !== norm(b.attrs[k])) out.push({ path: here, msg: `@${k}: '${a.attrs[k]}' != '${b.attrs[k]}'` });
  const ta = a.children.filter((c) => c.type === 'text').map((c) => c.text ?? '').join(' ').replace(/\s+/g, ' ').trim();
  const tb = b.children.filter((c) => c.type === 'text').map((c) => c.text ?? '').join(' ').replace(/\s+/g, ' ').trim();
  if (norm(ta) !== norm(tb)) out.push({ path: here, msg: `text '${ta}' != '${tb}'` });
  const ea = a.children.filter((c) => c.type === 'element'), eb = b.children.filter((c) => c.type === 'element');
  if (ea.length !== eb.length) out.push({ path: here, msg: `${ea.length} child elements != ${eb.length}: [${ea.map((e) => e.attrs.name ?? e.name).slice(0, 12)}] vs [${eb.map((e) => e.attrs.name ?? e.name).slice(0, 12)}]` });
  for (let i = 0; i < Math.min(ea.length, eb.length); i++) diffTrees(ea[i], eb[i], here, out, norm);
  return out;
}
export function diffXml(ours: string, gt: string): Diff[] {
  return diffTrees(parseXML(ours), parseXML(gt), '', [], normalizeFind);
}

/** Resolver over files on disk: `package://pkg/...` through `pkgs`, other paths relative to `base`. */
export function diskResolver(base: string, pkgs: Record<string, string> = {}) {
  return (path: string): string | null => {
    let p = path;
    const m = /^package:\/\/([^/]+)\/?(.*)$/.exec(p);
    if (m) { if (!pkgs[m[1]]) return null; p = join(pkgs[m[1]], m[2]); }
    else if (!/^([A-Za-z]:|\/)/.test(p)) p = join(base, p);
    return existsSync(p) && statSync(p).isFile() ? readFileSync(p, 'utf8') : null;
  };
}

export function listFiles(dir: string, re: RegExp): string[] {
  const out: string[] = [];
  const walk = (d: string) => { for (const f of readdirSync(d)) { const p = join(d, f); if (statSync(p).isDirectory()) walk(p); else if (re.test(f)) out.push(p); } };
  if (existsSync(dir)) walk(dir);
  return out;
}

export const read = (p: string) => readFileSync(p, 'utf8');
export { join, dirname, existsSync };
