/**
 * PROTO resolution and instantiation: EXTERNPROTO URLs resolved relative to the declaring file, webots:// and
 * raw.githubusercontent.com/cyberbotics URLs mapped to the clean-room built-in equivalents (builtins.ts), by-name
 * fallback for worlds without EXTERNPROTO lines, field defaults / IS substitution (SF/MF values and nodes), JavaScript
 * template expansion (template.ts), recursive PROTOs with a cycle guard and an expansion cache per instance node.
 */
import { WNode, WValue, WExtern, ProtoFile, ProtoFieldDecl, parseProtoFile, parseNodes, newScope, WebotsSyntaxError } from './vrml';
import { expandTemplate, TemplateContext, TemplateError } from './template';
import { BUILTIN_PROTOS, BuiltinProto } from './builtins';

/** Read-only file source (a dropped folder, a zip, a directory on the server). Paths use '/' separators. */
export interface WebotsSource {
  read(path: string): Uint8Array | undefined;
  list(): string[];
}

export const BASE_NODES = new Set([
  'Accelerometer', 'Altimeter', 'Appearance', 'Background', 'BallJoint', 'BallJointParameters', 'Billboard', 'Box', 'Brake', 'CadShape', 'Camera', 'Capsule',
  'Charger', 'Color', 'Compass', 'Cone', 'Connector', 'ContactProperties', 'Coordinate', 'Cylinder', 'Damping', 'DirectionalLight', 'Display', 'DistanceSensor',
  'ElevationGrid', 'Emitter', 'Fluid', 'Focus', 'Fog', 'GPS', 'Group', 'Gyro', 'Hinge2Joint', 'Hinge2JointParameters', 'HingeJoint', 'HingeJointParameters',
  'ImageTexture', 'ImmersionProperties', 'IndexedFaceSet', 'IndexedLineSet', 'InertialUnit', 'JointParameters', 'LED', 'Lens', 'LensFlare', 'Lidar',
  'LightSensor', 'LinearMotor', 'Material', 'Mesh', 'Microphone', 'Muscle', 'Normal', 'PBRAppearance', 'Pen', 'Physics', 'Plane', 'PointLight', 'PointSet',
  'Pose', 'PositionSensor', 'Propeller', 'Radar', 'Radio', 'Receiver', 'Recognition', 'RangeFinder', 'Robot', 'RotationalMotor', 'Shape', 'Skin', 'SliderJoint',
  'Slot', 'Solid', 'SolidReference', 'Speaker', 'Sphere', 'SpotLight', 'TextureCoordinate', 'TextureTransform', 'TouchSensor', 'Track', 'TrackWheel',
  'Transform', 'VacuumGripper', 'Viewpoint', 'WorldInfo', 'Zoom',
]);

export function normPath(p: string): string {
  const parts: string[] = [];
  const abs = /^[A-Za-z]:/.test(p) ? '' : p.startsWith('/') ? '/' : '';
  for (const s of p.replace(/\\/g, '/').split('/')) {
    if (!s || s === '.') continue;
    if (s === '..' && parts.length && parts[parts.length - 1] !== '..') parts.pop();
    else parts.push(s);
  }
  return abs + parts.join('/');
}
export const dirOf = (p: string) => { const i = p.replace(/\\/g, '/').lastIndexOf('/'); return i >= 0 ? p.slice(0, i) : ''; };
export const baseName = (p: string) => p.replace(/\\/g, '/').split('/').pop() ?? p;
export const isRemote = (u: string) => /^(https?|webots):/i.test(u);
export const isStandardLibraryUrl = (u: string) => /^webots:\/\//i.test(u) || /raw\.githubusercontent\.com\/cyberbotics\/webots\//i.test(u) || /cyberbotics\.com/i.test(u);

/** Resolve a URL relative to the file that declares it. */
export function resolveUrl(url: string, fromFile: string): string {
  if (isRemote(url)) return url;
  if (isRemote(fromFile)) { try { return new URL(url.replace(/\\/g, '/'), fromFile).href; } catch { return url; } } // relative to a remote PROTO
  if (/^[A-Za-z]:[\\/]/.test(url) || url.startsWith('/')) return normPath(url);
  return normPath(`${dirOf(fromFile)}/${url}`);
}

export interface ProtoDecl {
  name: string;
  path: string;
  file?: ProtoFile;
  builtin?: BuiltinProto;
  fields: ProtoFieldDecl[];
}

export interface Scope {
  file: string;
  externs: Map<string, string>;
  /** PROTO names declared IMPORTABLE (spawned at run time by a supervisor, e.g. RetailProduct). */
  importable?: Set<string>;
}

export interface ProtoRegistryOptions {
  /** Optional fetcher for remote PROTO URLs (e.g. the server reading the local Webots cache). */
  remote?: (url: string) => Uint8Array | undefined;
  context?: Partial<TemplateContext>;
}

/** Webots value shapes for templates. */
export function jsFieldValue(type: string, v: WValue | undefined, reg?: ProtoRegistry, scope?: Scope): unknown {
  const nums = v && v.k === 'num' ? v.v : [];
  const group = <T>(n: number, mk: (a: number[]) => T): T[] => { const out: T[] = []; for (let i = 0; i + n <= nums.length; i += n) out.push(mk(nums.slice(i, i + n))); return out; };
  switch (type) {
    case 'SFBool': return v && v.k === 'bool' ? v.v[0] : false;
    case 'SFInt32': case 'SFFloat': return nums[0] ?? 0;
    case 'SFString': return v && v.k === 'str' ? v.v[0] ?? '' : '';
    case 'SFVec2f': return { x: nums[0] ?? 0, y: nums[1] ?? 0 };
    case 'SFVec3f': return { x: nums[0] ?? 0, y: nums[1] ?? 0, z: nums[2] ?? 0 };
    case 'SFColor': return { r: nums[0] ?? 0, g: nums[1] ?? 0, b: nums[2] ?? 0 };
    case 'SFRotation': return { x: nums[0] ?? 0, y: nums[1] ?? 0, z: nums[2] ?? 1, a: nums[3] ?? 0 };
    case 'MFBool': return v && v.k === 'bool' ? [...v.v] : [];
    case 'MFInt32': case 'MFFloat': return [...nums];
    case 'MFString': return v && v.k === 'str' ? [...v.v] : [];
    case 'MFVec2f': return group(2, (a) => ({ x: a[0], y: a[1] }));
    case 'MFVec3f': return group(3, (a) => ({ x: a[0], y: a[1], z: a[2] }));
    case 'MFColor': return group(3, (a) => ({ r: a[0], g: a[1], b: a[2] }));
    case 'MFRotation': return group(4, (a) => ({ x: a[0], y: a[1], z: a[2], a: a[3] }));
    case 'SFNode': { const n = v && v.k === 'node' ? v.v : v && v.k === 'nodes' ? v.v[0] ?? null : null; return n ? jsNode(n, reg, scope) : null; }
    case 'MFNode': return v && (v.k === 'nodes' || v.k === 'node') ? (v.k === 'nodes' ? v.v : v.v ? [v.v] : []).map((n) => jsNode(n, reg, scope)) : [];
    default: return undefined;
  }
}

function jsNode(n: WNode, reg?: ProtoRegistry, scope?: Scope): unknown {
  const decl = reg && scope ? reg.lookup(n.type, { file: n.base, externs: reg.scopeOf(n.base)?.externs ?? scope.externs }, true) : undefined;
  const fields: Record<string, { value: unknown; defaultValue: unknown }> = {};
  if (decl) for (const f of decl.fields) fields[f.name] = { value: jsFieldValue(f.type, n.fields[f.name] ?? f.def, reg, scope), defaultValue: jsFieldValue(f.type, f.def, reg, scope) };
  else for (const [k, v] of Object.entries(n.fields)) fields[k] = { value: v.k === 'num' ? (v.v.length === 1 ? v.v[0] : [...v.v]) : v.k === 'str' ? v.v[0] : v.k === 'bool' ? v.v[0] : null, defaultValue: null };
  return { node_name: n.type, fields };
}

/** Deep copy of a parsed PROTO body with IS references replaced by the instance field values. */
function substituteIS(n: WNode, values: Record<string, WValue>, memo: Map<WNode, WNode>, warn: (m: string) => void): WNode {
  const seen = memo.get(n);
  if (seen) return seen;
  const out: WNode = { ...n, fields: {} };
  memo.set(n, out);
  for (const [k, v] of Object.entries(n.fields)) {
    if (v.k === 'is') {
      const val = values[v.v];
      if (val === undefined) { warn(`IS ${v.v}: no such PROTO field`); continue; }
      out.fields[k] = val;
    } else if (v.k === 'node') out.fields[k] = { k: 'node', v: v.v ? substituteIS(v.v, values, memo, warn) : null };
    else if (v.k === 'nodes') out.fields[k] = { k: 'nodes', v: v.v.map((c) => substituteIS(c, values, memo, warn)) };
    else out.fields[k] = v;
  }
  return out;
}

let instanceCounter = 0;

export class ProtoRegistry {
  private files = new Map<string, ProtoDecl | null>();
  private scopes = new Map<string, Scope>();
  private expanded = new WeakMap<WNode, WNode | null>();
  private byBase = new Map<string, string[]>();
  private stack: string[] = [];
  readonly report: string[] = [];
  private warned = new Set<string>();

  constructor(public src: WebotsSource, public opts: ProtoRegistryOptions = {}) {
    for (const p of src.list()) {
      const b = baseName(p).toLowerCase();
      if (!this.byBase.has(b)) this.byBase.set(b, []);
      this.byBase.get(b)!.push(p);
    }
  }

  warnOnce(msg: string): void {
    if (this.warned.has(msg)) return;
    this.warned.add(msg);
    this.report.push(msg);
  }

  /** Register the EXTERNPROTO list of a file (world or PROTO). */
  registerScope(file: string, externs: WExtern[]): Scope {
    const m = new Map<string, string>(), imp = new Set<string>();
    for (const e of externs) {
      const name = baseName(e.url).replace(/\.proto$/i, '');
      m.set(name, isRemote(e.url) ? e.url : resolveUrl(e.url, file));
      if (e.importable) imp.add(name);
    }
    const s = { file, externs: m, importable: imp };
    this.scopes.set(file, s);
    return s;
  }
  scopeOf(file: string): Scope | undefined {
    return this.scopes.get(file);
  }

  /** `webots://` -> raw GitHub URL of the world's Webots version (the Webots cache key), other URLs unchanged. */
  canonicalUrl(url: string): string {
    if (!/^webots:\/\//i.test(url)) return url;
    const v = this.opts.context?.webots_version ?? { major: 'R2025', revision: 'a' };
    return `https://raw.githubusercontent.com/cyberbotics/webots/${v.major}${v.revision}/${url.replace(/^webots:\/\//i, '')}`;
  }
  /** Bytes of a remote PROTO / mesh / texture through `opts.remote` (e.g. the local Webots cache); undefined offline. */
  fetchRemote(url: string): Uint8Array | undefined {
    return this.opts.remote ? this.opts.remote(this.canonicalUrl(url)) : undefined;
  }

  /** Find a file in the source: exact path, case-insensitive path, then unique basename. */
  findFile(path: string, byNameFallback = true): string | undefined {
    const n = normPath(path);
    if (this.src.read(n)) return n;
    const lower = n.toLowerCase();
    for (const p of this.src.list()) if (normPath(p).toLowerCase() === lower) return p;
    if (!byNameFallback) return undefined;
    const cands = this.byBase.get(baseName(n).toLowerCase()) ?? [];
    if (cands.length >= 1) {
      // prefer the candidate sharing the longest path suffix
      const segs = lower.split('/');
      let best = cands[0], bestScore = -1;
      for (const c of cands) {
        const cs = normPath(c).toLowerCase().split('/');
        let k = 0;
        while (k < cs.length && k < segs.length && cs[cs.length - 1 - k] === segs[segs.length - 1 - k]) k++;
        if (k > bestScore) { bestScore = k; best = c; }
      }
      return best;
    }
    return undefined;
  }

  private loadFile(path: string): ProtoDecl | null {
    if (this.files.has(path)) return this.files.get(path)!;
    this.files.set(path, null);
    let bytes = this.src.read(path);
    if (!bytes && isRemote(path)) bytes = this.fetchRemote(path);
    if (!bytes) return null;
    try {
      const pf = parseProtoFile(new TextDecoder().decode(bytes), path);
      this.registerScope(path, pf.externs);
      const decl: ProtoDecl = { name: pf.name, path, file: pf, fields: pf.fields };
      this.files.set(path, decl);
      return decl;
    } catch (e) {
      this.report.push(`PROTO ${path}: ${(e as Error).message}`);
      return null;
    }
  }

  /** Resolve a node type to a PROTO declaration (undefined: base node or unknown). */
  lookup(type: string, scope: Scope, quiet = false): ProtoDecl | undefined {
    if (BASE_NODES.has(type)) return undefined;
    const url = scope.externs.get(type);
    const builtin = BUILTIN_PROTOS[type];
    if (builtin && scope.file.startsWith('builtin:')) return { name: type, path: `builtin:${type}`, builtin, fields: builtin.fields };
    if (url) {
      if (isRemote(url)) {
        if (builtin && isStandardLibraryUrl(url)) return { name: type, path: url, builtin, fields: builtin.fields };
        const local = this.findFile(`${type}.proto`);
        if (local) { if (!quiet) this.warnOnce(`${type}: ${url} resolved to the local file ${local}`); const d = this.loadFile(local); if (d) return d; }
        if (this.opts.remote) { const d = this.loadFile(url); if (d) return d; }
        if (builtin) { if (!quiet) this.warnOnce(`${type}: ${url} replaced by the built-in equivalent`); return { name: type, path: url, builtin, fields: builtin.fields }; }
        if (!quiet) this.warnOnce(`${type}: remote PROTO ${url} is not available offline`);
        return undefined;
      }
      const found = this.findFile(url, false);
      if (found) { const d = this.loadFile(found); if (d) return d; }
      const byName = this.findFile(url, true);
      if (byName) { if (!quiet) this.warnOnce(`${type}: ${url} not found, using ${byName}`); const d = this.loadFile(byName); if (d) return d; }
      if (builtin) { if (!quiet) this.warnOnce(`${type}: ${url} not found, using the built-in equivalent`); return { name: type, path: `builtin:${type}`, builtin, fields: builtin.fields }; }
      if (!quiet) this.warnOnce(`${type}: EXTERNPROTO ${url} not found`);
      return undefined;
    }
    // no EXTERNPROTO declaration: local file by name (same folder, ../protos, anywhere), then built-in
    const dir = dirOf(scope.file);
    for (const cand of [`${dir}/${type}.proto`, `${dir}/../protos/${type}.proto`, `${dir}/protos/${type}.proto`]) {
      const f = this.findFile(cand, false);
      if (f) { if (!quiet) this.warnOnce(`${type}: no EXTERNPROTO in ${scope.file}, resolved by name to ${f}`); const d = this.loadFile(f); if (d) return d; }
    }
    if (builtin) { if (!quiet) this.warnOnce(`${type}: no EXTERNPROTO in ${scope.file}, resolved by name to the built-in equivalent`); return { name: type, path: `builtin:${type}`, builtin, fields: builtin.fields }; }
    const any = this.findFile(`${type}.proto`);
    if (any) { if (!quiet) this.warnOnce(`${type}: no EXTERNPROTO in ${scope.file}, resolved by name to ${any}`); const d = this.loadFile(any); if (d) return d; }
    if (!quiet) this.warnOnce(`${type}: unknown node type (no PROTO found)`);
    return undefined;
  }

  /** Is `type` a PROTO (resolvable or not) rather than a base node? */
  isProto(type: string): boolean {
    return !BASE_NODES.has(type);
  }

  /**
   * Expand a node until it is a base node. Returns null when the PROTO cannot be resolved / expanded (reported).
   * The returned node carries `proto` (outermost instance) for item parameters.
   */
  resolve(node: WNode): WNode | null {
    let n: WNode | null = node;
    let outer: WNode['proto'] | undefined;
    let guard = 0;
    while (n && !BASE_NODES.has(n.type)) {
      if (++guard > 32) { this.report.push(`${node.type}: recursive PROTO (nesting too deep)`); return null; }
      const inst: WNode = n;
      n = this.expandOne(inst);
      if (n && !outer) outer = n.proto;
    }
    if (n && outer && n.proto !== outer) n = { ...n, proto: outer };
    return n;
  }

  /** Body text of a PROTO for resolved field values (JavaScript template applied). */
  private bodyText(decl: ProtoDecl, values: Record<string, WValue>, scope: Scope): string {
    const pf = decl.file!;
    if (pf.template !== 'javascript') return pf.body;
    const fields: Record<string, { value: unknown; defaultValue: unknown }> = {};
    for (const f of decl.fields) fields[f.name] = { value: jsFieldValue(f.type, values[f.name], this, scope), defaultValue: jsFieldValue(f.type, f.def, this, scope) };
    const ctx: TemplateContext = {
      world: this.opts.context?.world ?? '', proto: pf.path, id: ++instanceCounter, coordinate_system: this.opts.context?.coordinate_system ?? 'ENU',
      webots_version: this.opts.context?.webots_version ?? { major: 'R2025', revision: 'a' }, os: this.opts.context?.os ?? 'linux', webots_home: '', temporary_files_path: '/tmp/',
    };
    const messages: string[] = [];
    try { return expandTemplate(pf.body, fields, ctx, { messages }); } finally { for (const m of messages) this.warnOnce(`${decl.name}: ${m}`); }
  }

  /** Expanded body text of a PROTO instance (template applied, IS not substituted); null if not a file PROTO. */
  expandedText(node: WNode): string | null {
    const scope = this.scopes.get(node.base) ?? this.registerScope(node.base, []);
    const decl = this.lookup(node.type, scope, true);
    if (!decl?.file || decl.file.template === 'lua') return null;
    const values: Record<string, WValue> = {};
    for (const f of decl.fields) values[f.name] = node.fields[f.name] ?? f.def;
    return this.bodyText(decl, values, scope);
  }

  private expandOne(node: WNode): WNode | null {
    if (this.expanded.has(node)) return this.expanded.get(node)!;
    const scope = this.scopes.get(node.base) ?? this.registerScope(node.base, []);
    const decl = this.lookup(node.type, scope);
    if (!decl) { this.expanded.set(node, null); return null; }
    const key = decl.path;
    // nodes made by a built-in carry base 'builtin:<type>' (e.g. the default BrushedAluminium wall appearance): not a cycle
    if (this.stack.includes(key) || (key === node.base && !decl.builtin)) { this.report.push(`${node.type}: recursive PROTO ${key}`); this.expanded.set(node, null); return null; }
    this.stack.push(key);
    try {
      const values: Record<string, WValue> = {};
      for (const f of decl.fields) values[f.name] = node.fields[f.name] ?? f.def;
      if (!decl.builtin?.lenient) for (const k of Object.keys(node.fields)) if (!decl.fields.some((f) => f.name === k)) this.warnOnce(`${node.type}: unknown field '${k}' ignored`);
      let root: WNode | null = null;
      if (decl.builtin) root = decl.builtin.build(values, node, (m) => this.warnOnce(m));
      else if (decl.file) {
        const pf = decl.file;
        if (pf.template === 'lua') { this.report.push(`${node.type}: Lua template PROTOs are not supported`); this.expanded.set(node, null); return null; }
        const body = this.bodyText(decl, values, scope);
        const ps = newScope();
        const nodes = parseNodes(body, pf.path, ps);
        for (const w of ps.warnings) this.warnOnce(w);
        if (!nodes.length) { this.report.push(`${node.type}: empty PROTO body`); this.expanded.set(node, null); return null; }
        if (nodes.length > 1) this.warnOnce(`${node.type}: PROTO body has ${nodes.length} root nodes, only the first is used`);
        root = substituteIS(nodes[0], values, new Map(), (m) => this.warnOnce(`${node.type}: ${m}`));
      }
      if (!root) { this.expanded.set(node, null); return null; }
      const out: WNode = { ...root, proto: root.proto ?? { name: decl.name, url: decl.path, def: node.def, fields: values } };
      if (node.def && !out.def) out.def = node.def;
      this.expanded.set(node, out);
      return out;
    } catch (e) {
      const msg = e instanceof TemplateError || e instanceof WebotsSyntaxError ? e.message : String((e as Error).message ?? e);
      this.report.push(`${node.type} (${decl.path}): ${msg}`);
      this.expanded.set(node, null);
      return null;
    } finally {
      this.stack.pop();
    }
  }
}
