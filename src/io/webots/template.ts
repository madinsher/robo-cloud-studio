/**
 * JavaScript template engine for Webots PROTOs (`# template language: javascript`). The body is compiled into a function:
 * text chunks are appended verbatim, `%<= expr >%` appends String(expr), `%< statements >%` are inlined.
 * `import ... from 'wbxxx.js'` statements map to clean-room shims of the Webots template modules.
 * Field values follow the Webots shapes: SFVec3f {x,y,z}, SFVec2f {x,y}, SFColor {r,g,b}, SFRotation {x,y,z,a},
 * scalars, MF* arrays, SFNode {node_name, fields} | null.
 *
 * Isolation: a template is untrusted code (it comes with the opened .wbt / .proto files). It runs in a separate
 * JavaScript realm, never in the studio's: a `node:vm` context in Node (tests, server, CLI), a detached same-origin
 * iframe in the browser. The engine itself is evaluated inside that realm (`TEMPLATE_ENGINE_SOURCE`) and, before any
 * template runs, disables code generation there (Function / generator / async function constructors and eval throw),
 * so a template cannot reach the realm's global object; every global name of the realm except the pure built-ins
 * (Math, JSON, Number, String, Array, Object, ...) is shadowed by an undefined parameter; dynamic `import()` is
 * rejected. Only strings cross the realm boundary (fields and context as JSON in, the output as JSON out). Limits: a
 * loop guard in every loop condition (iterations and time), an output size limit, and in Node the vm timeout.
 */

export interface TemplateContext {
  world: string;
  proto: string;
  /** Unique node id (a number in Webots). */
  id: number | string;
  coordinate_system: 'ENU' | 'NUE';
  webots_version: { major: string; revision: string };
  os: string;
  webots_home: string;
  temporary_files_path: string;
}

export class TemplateError extends Error {}

/** Transform ES module imports of template modules into lookups in `__imports`. */
function rewriteImports(code: string): string {
  return code
    .replace(/import\s+\*\s+as\s+([A-Za-z_$][\w$]*)\s+from\s+['"]([^'"]+)['"]\s*;?/g, (_m, id, mod) => `const ${id} = __imports(${JSON.stringify(mod)});`)
    .replace(/import\s+\{([^}]*)\}\s+from\s+['"]([^'"]+)['"]\s*;?/g, (_m, list: string, mod) => `const {${list.replace(/\s+as\s+/g, ': ')}} = __imports(${JSON.stringify(mod)});`)
    .replace(/import\s+([A-Za-z_$][\w$]*)\s+from\s+['"]([^'"]+)['"]\s*;?/g, (_m, id, mod) => `const ${id} = __imports(${JSON.stringify(mod)});`);
}

/**
 * Guard every loop: `while (c)` (also the tail of do-while) becomes `while (G() && (c))`, `for (a; c; b)` becomes
 * `for (a; G() && (c); b)`; for-in / for-of iterate finite collections (a generator's own loop is guarded).
 */
function guardLoops(code: string, g = '__g'): string {
  let out = '';
  let i = 0;
  const n = code.length;
  const re = /\b(for|while)\s*\(/g;
  let m: RegExpExecArray | null;
  while ((m = re.exec(code))) {
    const open = m.index + m[0].length; // after '('
    // the matching ')' and the top-level ';' of a for header
    let j = open, depth = 1; const semis: number[] = [];
    while (j < n && depth > 0) {
      const c = code[j];
      if (c === '"' || c === "'" || c === '`') { const q = c; j++; while (j < n && code[j] !== q) { if (code[j] === '\\') j++; j++; } }
      else if (c === '(' || c === '[' || c === '{') depth++;
      else if (c === ')' || c === ']' || c === '}') depth--;
      else if (c === ';' && depth === 1) semis.push(j);
      j++;
    }
    const close = j - 1;
    if (m[1] === 'while') { out += code.slice(i, open) + `${g}() && (` + code.slice(open, close) + ')'; i = close; re.lastIndex = close; continue; }
    if (semis.length === 2) { const cond = code.slice(semis[0] + 1, semis[1]); out += code.slice(i, semis[0] + 1) + ` ${g}() && (${cond.trim() || 'true'})` + code.slice(semis[1], close); i = close; re.lastIndex = close; continue; }
    re.lastIndex = open;
  }
  return out + code.slice(i);
}

/** Compile a template body to JavaScript source (exported for tests / debugging); `g` is the loop-guard function name. */
export function compileTemplate(body: string, g = '__g'): string {
  let js = '';
  const re = /%<(=?)([\s\S]*?)>%/g;
  let last = 0;
  let m: RegExpExecArray | null;
  while ((m = re.exec(body))) {
    js += `__w(${JSON.stringify(body.slice(last, m.index))});\n`;
    const code = m[1] ? m[2] : rewriteImports(m[2]);
    // module imports were rewritten to __imports(); any other `import` (dynamic import(), import.meta) would load
    // code outside the isolated realm's control
    if (/\bimport\b/.test(code)) throw new Error(`template code may not use import (${code.trim().slice(0, 60)})`);
    js += m[1] ? `__w(${guardLoops(code, g)});\n` : `${guardLoops(code, g)}\n`;
    last = re.lastIndex;
  }
  js += `__w(${JSON.stringify(body.slice(last))});\n`;
  return js;
}

export interface TemplateOptions {
  /** Max loop iterations / output characters / milliseconds. */
  maxIterations?: number;
  maxOutput?: number;
  maxMs?: number;
  /** Receives the template's console.error / console.warn / wbutility.error lines (Webots prints them in its console). */
  messages?: string[];
  /** The isolated realm to evaluate in (one per import, see createTemplateRealm); a fresh one when omitted. */
  realm?: TemplateRealm;
}

/**
 * The engine as it runs inside the isolated realm (its source is `TEMPLATE_ENGINE_SOURCE`): disables code
 * generation in the realm, then expands templates from JSON arguments to a JSON result. Self-contained: it uses only
 * compileTemplate / guardLoops / rewriteImports / templateModules (shipped with it) and the realm's built-ins.
 */
function vbsTemplateEngine(): { expand(args: string): string } {
  const RealFunction = Function;
  const G = globalThis as unknown as Record<string, unknown>;
  const deny = function () { throw new TypeError('code generation is disabled in PROTO templates'); };
  for (const f of [function () { /* plain */ }, function* () { /* generator */ }, async function () { /* async */ }, async function* () { /* async generator */ }]) {
    const p = Object.getPrototypeOf(f);
    try { Object.defineProperty(p, 'constructor', { value: deny, writable: false, configurable: false }); } catch { /* already frozen */ }
  }
  for (const k of ['eval', 'Function']) { try { Object.defineProperty(G, k, { value: deny, writable: false, configurable: false }); } catch { try { G[k] = deny; } catch { /* keep */ } } }
  // a browser realm (the detached iframe): drop what reaches storage, the network or other windows (defence in depth -
  // template code cannot name or reach the realm's global object)
  for (let o: object | null = G; o; o = Object.getPrototypeOf(o)) for (const k of ['fetch', 'XMLHttpRequest', 'WebSocket', 'EventSource', 'Worker', 'SharedWorker', 'BroadcastChannel', 'importScripts', 'localStorage', 'sessionStorage', 'indexedDB', 'caches', 'cookieStore', 'open', 'opener', 'parent', 'frames', 'postMessage', 'navigator', 'setTimeout', 'setInterval', 'queueMicrotask', 'requestAnimationFrame']) { try { delete (o as Record<string, unknown>)[k]; } catch { /* unforgeable */ } }
  const KEEP = new Set(['Math', 'JSON', 'Number', 'String', 'Boolean', 'Array', 'Object', 'Symbol', 'BigInt', 'Map', 'Set', 'WeakMap', 'WeakSet', 'Date', 'RegExp', 'Error', 'TypeError', 'RangeError', 'SyntaxError', 'ReferenceError', 'EvalError', 'URIError', 'parseInt', 'parseFloat', 'isNaN', 'isFinite', 'Infinity', 'NaN', 'undefined', 'encodeURIComponent', 'decodeURIComponent', 'encodeURI', 'decodeURI', 'Int8Array', 'Uint8Array', 'Uint8ClampedArray', 'Int16Array', 'Uint16Array', 'Int32Array', 'Uint32Array', 'Float32Array', 'Float64Array', 'BigInt64Array', 'BigUint64Array', 'ArrayBuffer', 'DataView', 'Intl']);
  const RESERVED = new Set(['eval', 'arguments', 'implements', 'interface', 'let', 'package', 'private', 'protected', 'public', 'static', 'yield', 'await', 'enum', 'fields', 'context', '__w', '__imports', 'console', 'break', 'case', 'catch', 'class', 'const', 'continue', 'debugger', 'default', 'delete', 'do', 'else', 'export', 'extends', 'false', 'finally', 'for', 'function', 'if', 'import', 'in', 'instanceof', 'new', 'null', 'return', 'super', 'switch', 'this', 'throw', 'true', 'try', 'typeof', 'var', 'void', 'while', 'with']);
  const names = new Set<string>(['globalThis', 'Function', 'Reflect', 'Proxy', 'Promise', 'WebAssembly', 'Atomics', 'SharedArrayBuffer', 'FinalizationRegistry', 'WeakRef', '__engine', '__args', 'window', 'self', 'top', 'parent', 'document', 'location', 'process', 'require', 'module', 'exports', 'global', 'fetch', 'XMLHttpRequest', 'localStorage', 'indexedDB', 'navigator', 'setTimeout', 'setInterval', 'queueMicrotask', 'postMessage', 'importScripts', 'Deno', 'Bun']);
  for (let o: object | null = G; o; o = Object.getPrototypeOf(o)) {
    let own: string[] = [];
    try { own = Object.getOwnPropertyNames(o); } catch { own = []; }
    for (const k of own) if (/^[A-Za-z_$][\w$]*$/.test(k)) names.add(k);
  }
  const SHADOW = [...names].filter((k) => !KEEP.has(k) && !RESERVED.has(k));
  // built-ins the engine needs, taken before any template runs (a template may overwrite the realm's own)
  const Str = String, parse = JSON.parse, stringify = JSON.stringify, now = Date.now, apply = Reflect.apply, join = Array.prototype.join, map = Array.prototype.map;
  const text = (v: unknown) => { try { return Str(v); } catch { return '[unprintable]'; } };
  const add = (list: string[], v: string) => { list[list.length] = v; };
  const expand = (argsJson: string): string => {
    const a = parse(argsJson) as { body: string; fields: unknown; context: unknown; maxIt: number; maxOut: number; maxMs: number; guard: string };
    const parts: string[] = [], stdout: string[] = [], stderr: string[] = [];
    let size = 0, iters = 0;
    const t0 = now();
    const result = (r: Record<string, unknown>) => stringify({ ...r, stdout, stderr });
    class Limit extends Error {}
    const w = (x: unknown) => { const s = text(x); size += s.length; if (size > a.maxOut) throw new Limit('template output limit exceeded'); add(parts, s); };
    const guard = () => { if (++iters > a.maxIt || (iters % 4096 === 0 && now() - t0 > a.maxMs)) throw new Limit('template loop limit exceeded'); return true; };
    const say = (into: string[]) => (...xs: unknown[]) => { add(into, apply(join, apply(map, xs, [text]), [' ']) as string); };
    const consoleShim = { log: say(stdout), info: say(stdout), debug: say(stdout), warn: say(stderr), error: say(stderr) };
    const modules = templateModules(stderr, stdout);
    const imports = (mod: string) => {
      const key = String(mod).replace(/^.*[\\/]/, '').replace(/\.js$/, '');
      const shim = modules[key];
      if (!shim) throw new Limit(`unsupported template module '${mod}'`);
      return shim;
    };
    let fn: (...args: unknown[]) => void;
    try {
      fn = new RealFunction('fields', 'context', '__w', a.guard, '__imports', 'console', ...SHADOW, `"use strict";\n${compileTemplate(a.body, a.guard)}`) as (...args: unknown[]) => void;
    } catch (e) { return result({ error: `template compile error: ${text((e as Error)?.message ?? e)}` }); }
    try {
      fn(a.fields, a.context, w, guard, imports, consoleShim, ...SHADOW.map(() => undefined));
    } catch (e) {
      if (e instanceof Limit) return result({ error: e.message });
      return result({ error: `template error: ${text((e as Error)?.message ?? e)}` });
    }
    return result({ out: apply(join, parts, ['']) as string });
  };
  return { expand };
}

/** Source of the engine evaluated inside the isolated realm (a function body returning the engine). */
export const TEMPLATE_ENGINE_SOURCE = (): string => '"use strict";\n' + ([rewriteImports, guardLoops, compileTemplate, templateModules] as Array<(...a: never[]) => unknown>).map((f) => `const ${f.name} = (${f.toString()});`).join('\n') + `\nreturn (${vbsTemplateEngine.toString()})();`;

/** An isolated JavaScript realm running the template engine. */
export interface TemplateRealm { kind: 'vm' | 'iframe'; expand(argsJson: string, timeoutMs: number): string }

/** A fresh realm for one import: a node:vm context (Node) or a detached iframe (browser); null when neither exists. */
export function createTemplateRealm(prefer: 'vm' | 'iframe' = 'vm'): TemplateRealm | null {
  const proc = (globalThis as { process?: { getBuiltinModule?: (id: string) => unknown; versions?: { node?: string } } }).process;
  const vm = prefer === 'vm' && proc?.versions?.node && typeof proc.getBuiltinModule === 'function' ? proc.getBuiltinModule('node:vm') as typeof import('node:vm') | undefined : undefined;
  if (vm) {
    // a fresh context for every expansion (< 1 ms): one template cannot change what the next one sees
    const src = `globalThis.__engine = (function () {\n${TEMPLATE_ENGINE_SOURCE()}\n})();`;
    return {
      kind: 'vm',
      expand: (argsJson, timeoutMs) => {
        const sandbox = vm.createContext(Object.create(null) as object, { codeGeneration: { strings: true, wasm: false } });
        vm.runInContext(src, sandbox, { timeout: 5000 });
        (sandbox as Record<string, unknown>).__args = argsJson;
        const r = vm.runInContext('__engine.expand(__args)', sandbox, { timeout: Math.max(100, timeoutMs) });
        return typeof r === 'string' ? r : '{"error":"template engine returned a non-string"}';
      },
    };
  }
  const doc = (globalThis as { document?: Document }).document;
  if (doc?.createElement && doc.body) {
    const frame = doc.createElement('iframe');
    frame.style.display = 'none';
    doc.body.appendChild(frame);
    const win = frame.contentWindow as (Window & { Function: FunctionConstructor }) | null;
    if (!win) { frame.remove(); return null; }
    const engine = new win.Function(TEMPLATE_ENGINE_SOURCE())() as { expand(a: string): unknown };
    frame.remove(); // detached: the realm keeps working, its window has no parent / top any more
    return { kind: 'iframe', expand: (argsJson) => { const r = engine.expand(argsJson); return typeof r === 'string' ? r : '{"error":"template engine returned a non-string"}'; } };
  }
  return null;
}

/** Expand a template body in an isolated realm. Throws TemplateError (with the JS error message) on failure. */
export function expandTemplate(body: string, fields: Record<string, { value: unknown; defaultValue: unknown }>, context: TemplateContext, opts: TemplateOptions = {}): string {
  const maxIt = opts.maxIterations ?? 5e6, maxOut = opts.maxOutput ?? 64e6, maxMs = opts.maxMs ?? 5000;
  const realm = opts.realm ?? createTemplateRealm();
  if (!realm) throw new TemplateError('JavaScript PROTO templates need an isolated realm (node:vm or an iframe); none is available here');
  const rnd = Array.from({ length: 4 }, () => Math.floor(Math.random() * 0x10000).toString(16)).join('');
  let args: string;
  try { args = JSON.stringify({ body, fields, context, maxIt, maxOut, maxMs, guard: `__g_${rnd}` }); } catch (e) { throw new TemplateError(`template fields cannot be passed: ${(e as Error).message}`); }
  let raw: string;
  try { raw = realm.expand(args, maxMs + 1000); } catch (e) { throw new TemplateError(/timed out/i.test(String((e as Error)?.message)) ? 'template time limit exceeded' : `template error: ${String((e as Error)?.message ?? e)}`); }
  let r: { out?: unknown; error?: unknown; stderr?: unknown; stdout?: unknown };
  try { r = JSON.parse(raw); } catch { throw new TemplateError('template engine returned an invalid result'); }
  const msgs = Array.isArray(r.stderr) ? r.stderr.map((m) => String(m)) : [];
  opts.messages?.push(...msgs);
  if (typeof r.error === 'string') throw new TemplateError(r.error);
  if (typeof r.out !== 'string') throw new TemplateError('template engine returned no output');
  return r.out;
}

// ---- template modules: same API and results as Webots' wbvector2/3, wbrotation, wbgeometry, wbutility, wbrandom -------
// (resources/web/wwi/protoVisualizer/templating/modules/webots in the Webots distribution, Apache-2.0); reimplemented
// here with identical formulas so generated geometry (circles, B-splines, LCG random placement) equals Webots'.

type V2 = { x: number; y: number };
type V3 = { x: number; y: number; z: number };
type Rot = { x: number; y: number; z: number; a: number };
type Quat = { w: number; x: number; y: number; z: number };
type M3 = Record<number, number>;

/** Fresh module instances for one template evaluation (wbrandom's seed starts at 0 like a new Webots evaluation). */
export function templateModules(stderr: string[] = [], stdout: string[] = []): Record<string, unknown> {
  const has = (o: unknown, ...k: string[]) => !!o && typeof o === 'object' && k.every((key) => Object.prototype.hasOwnProperty.call(o, key));
  const wbutility = {
    assert: (ok: unknown, msg: string) => { if (!ok) stderr.push(msg); },
    error: (msg: string) => { stderr.push(msg); },
    info: (msg: string) => { stdout.push(msg); },
    deepCopy: <T>(o: T): T => JSON.parse(JSON.stringify(o)),
    degreesToRadians: (a: number) => (a * Math.PI) / 180.0,
    radiansToDegrees: (a: number) => (a * 180.0) / Math.PI,
    isScalar: (v: unknown) => typeof v === 'number',
    isVector2: (v: unknown) => has(v, 'x', 'y'),
    isVector3: (v: unknown) => has(v, 'x', 'y', 'z'),
    isAxisAngle: (v: unknown) => has(v, 'x', 'y', 'z', 'a'),
    isQuaternion: (v: unknown) => has(v, 'w', 'x', 'y', 'z'),
    isMatrix3: (m: unknown) => has(m, '0', '1', '2', '3', '4', '5', '6', '7', '8'),
    isArrayOfPoints: (a: unknown, dim: number) => {
      if (!Array.isArray(a)) return false;
      if (dim !== 2 && dim !== 3) { stderr.push('The second paramter of wbutility.isArrayOfPoints can only be 2 or 3.'); return false; }
      return a.every((p) => (dim === 2 ? has(p, 'x', 'y') : has(p, 'x', 'y', 'z')));
    },
  };
  const wbvector2 = {
    equal: (a: V2, b: V2) => a.x === b.x && a.y === b.y,
    add: (a: V2, b: V2): V2 => ({ x: a.x + b.x, y: a.y + b.y }),
    minus: (a: V2, b: V2): V2 => ({ x: a.x - b.x, y: a.y - b.y }),
    multiply: (v: V2, s: number): V2 => ({ x: s * v.x, y: s * v.y }),
    norm: (v: V2) => Math.sqrt(v.x * v.x + v.y * v.y),
    distance: (a: V2, b: V2) => Math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2),
    angle: (a: V2, b: V2) => Math.atan2(a.y - b.y, a.x - b.x),
    cross: (a: V2, b: V2) => a.x * b.y - a.y * b.x,
    dot: (a: V2, b: V2) => a.x * b.x + a.y * b.y,
    normalize: (v: V2): V2 | null => { const n = Math.sqrt(v.x * v.x + v.y * v.y); if (n === 0) { stderr.push('The norm cannot be zero when normalizing.'); return null; } return { x: v.x / n, y: v.y / n }; },
    /** Intersection of segments p1-p2 and p3-p4, null when none. */
    intersection: (p1: V2, p2: V2, p3: V2, p4: V2): V2 | null => {
      if (Math.max(p1.x, p2.x) < Math.min(p3.x, p4.x)) return null;
      if ((p1.x === p2.x && p1.y === p2.y) || (p3.x === p4.x && p3.y === p4.y)) return null;
      if ((p1.x === p2.x && p3.x === p4.x) || (p1.y === p2.y && p3.y === p4.y)) return null;
      let xa: number, ya: number;
      if (p1.x !== p2.x && p3.x !== p4.x) {
        const a1 = (p1.y - p2.y) / (p1.x - p2.x), a2 = (p3.y - p4.y) / (p3.x - p4.x), b1 = p1.y - a1 * p1.x, b2 = p3.y - a2 * p3.x;
        if (a1 === a2) return null;
        xa = (b2 - b1) / (a1 - a2); ya = a2 * xa + b2;
      } else {
        const a1 = (p1.x - p2.x) / (p1.y - p2.y), a2 = (p3.x - p4.x) / (p3.y - p4.y), b1 = p1.x - a1 * p1.y, b2 = p3.x - a2 * p3.y;
        if (a1 === a2) return null;
        ya = (b2 - b1) / (a1 - a2); xa = a2 * ya + b2;
      }
      if (xa < Math.max(Math.min(p1.x, p2.x), Math.min(p3.x, p4.x)) || xa > Math.min(Math.max(p1.x, p2.x), Math.max(p3.x, p4.x))) return null;
      return { x: xa, y: ya };
    },
  };
  const wbvector3 = {
    equal: (a: V3, b: V3) => a.x === b.x && a.y === b.y && a.z === b.z,
    add: (a: V3, b: V3): V3 => ({ x: a.x + b.x, y: a.y + b.y, z: a.z + b.z }),
    minus: (a: V3, b: V3): V3 => ({ x: a.x - b.x, y: a.y - b.y, z: a.z - b.z }),
    multiply: (v: V3, s: number): V3 => ({ x: s * v.x, y: s * v.y, z: s * v.z }),
    norm: (v: V3) => Math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z),
    distance: (a: V3, b: V3) => Math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2),
    cross: (a: V3, b: V3): V3 => ({ x: a.y * b.z - a.z * b.y, y: a.z * b.x - a.x * b.z, z: a.x * b.y - a.y * b.x }),
    dot: (a: V3, b: V3) => a.x * b.x + a.y * b.y + a.z * b.z,
    normalize: (v: V3): V3 | null => { const n = Math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z); if (n === 0) { stderr.push('The norm cannot be zero when normalizing.'); return null; } return { x: v.x / n, y: v.y / n, z: v.z / n }; },
  };
  const normalizeRot = (r: Rot): Rot => {
    if (r.x === 0 && r.y === 0 && r.z === 0) stderr.push('The parameter in wbrotation.normalize is not a valid axis (norm is zero).');
    else { const inv = 1 / Math.sqrt(r.x * r.x + r.y * r.y + r.z * r.z); r.x *= inv; r.y *= inv; r.z *= inv; }
    while (r.a < -Math.PI) r.a += 2 * Math.PI;
    while (r.a > Math.PI) r.a -= 2 * Math.PI;
    return r;
  };
  const fromQuaternion = (q: Quat): Rot => {
    const a = 2 * Math.acos(q.w);
    if (a < 0.0001) return { x: 0, y: 1, z: 0, a: 0 };
    const n = Math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z);
    return { a, x: q.x / n, y: q.y / n, z: q.z / n };
  };
  const toQuaternion = (r: Rot): Quat => { normalizeRot(r); const h = r.a * 0.5, s = Math.sin(h); return { w: Math.cos(h), x: r.x * s, y: r.y * s, z: r.z * s }; };
  const toMatrix3 = (r: Rot): M3 => {
    const c = Math.cos(r.a), s = Math.sin(r.a), t1 = 1 - c, t2 = r.x * r.z * t1, t3 = r.x * r.y * t1, t4 = r.y * r.z * t1;
    return { 0: r.x * r.x * t1 + c, 1: t3 - r.z * s, 2: t2 + r.y * s, 3: t3 + r.z * s, 4: r.y * r.y * t1 + c, 5: t4 - r.x * s, 6: t2 - r.y * s, 7: t4 + r.x * s, 8: r.z * r.z * t1 + c };
  };
  const byMatrix3 = (m: M3, v: V3): V3 => ({ x: m[0] * v.x + m[1] * v.y + m[2] * v.z, y: m[3] * v.x + m[4] * v.y + m[5] * v.z, z: m[6] * v.x + m[7] * v.y + m[8] * v.z });
  const wbrotation = {
    equal: (a: Rot, b: Rot) => a.x === b.x && a.y === b.y && a.z === b.z && a.a === b.a,
    fromQuaternion,
    fromMatrix3: (m: M3): Rot => { const c = 0.5 * (m[0] + m[4] + m[8] - 1); return Math.abs(c) > 1 ? { x: 1, y: 0, z: 0, a: 0 } : { x: m[7] - m[5], y: m[2] - m[6], z: m[3] - m[1], a: Math.acos(c) }; },
    toQuaternion,
    toMatrix3,
    isIdentity: (r: Rot) => r.a === 0.0,
    normalize: normalizeRot,
    combine: (ra: Rot, rb: Rot): Rot => {
      const a = toQuaternion(ra), b = toQuaternion(rb);
      return fromQuaternion({ w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z, x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y, y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x, z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w });
    },
    rotateVector3ByMatrix3: byMatrix3,
    rotateVector3ByRotation: (r: Rot, v: V3): V3 => byMatrix3(toMatrix3(r), v),
    rotateVector3ByQuaternion: (q: Quat, v: V3): V3 => byMatrix3(toMatrix3(fromQuaternion(q)), v),
  };
  const spline = <P extends V2>(pointsIn: P[], subdivision: number, dims: Array<'x' | 'y' | 'z'>, first: number): P[] => {
    const pts = pointsIn as unknown as Array<Record<string, number>>;
    const n = pts.length;
    const ext = (a: Record<string, number>, b: Record<string, number>) => Object.fromEntries(dims.map((k) => [k, a[k] + (a[k] - b[k])]));
    pts.push(ext(pts[n - 1], pts[n - 2]));
    pts.push(ext(pts[n], pts[n - 1]));
    pts.unshift(ext(pts[0], pts[1]));
    pts.unshift(ext(pts[0], pts[1]));
    const out: Array<Record<string, number>> = [pts[2]];
    for (let i = first; i < n + 1; ++i) {
      const c = dims.map((k) => [(-pts[i - 1][k] + 3 * pts[i][k] - 3 * pts[i + 1][k] + pts[i + 2][k]) / 6.0, (3 * pts[i - 1][k] - 6 * pts[i][k] + 3 * pts[i + 1][k]) / 6.0, (-3 * pts[i - 1][k] + 3 * pts[i + 1][k]) / 6.0, (pts[i - 1][k] + 4 * pts[i][k] + pts[i + 1][k]) / 6.0]);
      for (let j = 1; j <= subdivision; j++) { const t = j / subdivision; out.push(Object.fromEntries(dims.map((k, d) => [k, (c[d][2] + t * (c[d][1] + t * c[d][0])) * t + c[d][3]]))); }
    }
    return out as unknown as P[];
  };
  const wbgeometry = {
    /** div + 1 points (closed) on a circle of `radius` around `center`, starting at angle `shift`. */
    circle: (radius: number, div: number, center: V2 = { x: 0, y: 0 }, shift = 0): V2[] => {
      wbutility.assert(typeof radius === 'number' && radius > 0, 'Expected radius to be a positive number in wbgeometry.circle.');
      wbutility.assert(typeof div === 'number' && div > 0, 'Expected div to be a positive number in wbgeometry.circle.');
      const q = (2 * Math.PI) / div, out: V2[] = [];
      for (let i = 0; i <= div; ++i) out.push({ x: radius * Math.cos(i * q + shift) + center.x, y: radius * Math.sin(i * q + shift) + center.y });
      return out;
    },
    isPoint2InPolygon: (p: V2, poly: V2[]) => {
      const n = poly.length;
      if (n < 3) return false;
      let inside = false, p1x = poly[0].x, p1y = poly[0].y, xin = 0;
      for (let i = 1; i <= n; ++i) {
        const p2x = poly[i % n].x, p2y = poly[i % n].y;
        if (p.y > Math.min(p1y, p2y) && p.y <= Math.max(p1y, p2y) && p.x <= Math.max(p1x, p2x)) {
          if (p1y !== p2y) xin = ((p.y - p1y) * (p2x - p1x)) / (p2y - p1y) + p1x;
          if (p1x === p2x || p.x <= xin) inside = !inside;
        }
        p1x = p2x; p1y = p2y;
      }
      return inside;
    },
    findClosestPoint2InArray: (ref: V2, pts: V2[]): V2 => { let best = ref, d = Infinity; for (const p of pts) { const c = wbvector2.distance(ref, p); if (c < d) { d = c; best = p; } } return best; },
    isPoint2ArrayClockwise: (pts: V2[]) => {
      const n = pts.length;
      let total = 0;
      for (let i = 0; i < n - 1; ++i) total += (pts[i + 1].x - pts[i].x) * (pts[i + 1].y + pts[i].y);
      total += (pts[0].x - pts[n - 1].x) * (pts[0].y + pts[n - 1].y);
      return !(total >= 0);
    },
    /** Third-order B-spline (mutates `points` like Webots: the array is extended by two points at each end). */
    bSpline2: (points: V2[], subdivision: number): V2[] => spline(points, subdivision, ['x', 'y'], 1),
    bSpline3: (points: V3[], subdivision: number): V3[] => spline(points, subdivision, ['x', 'y', 'z'], 2),
  };
  // Linear congruential generator with the Microsoft Visual Basic coefficients, 24-bit state, seed 0 by default
  let seedValue = 0;
  const update = () => { seedValue = (1140671485 * seedValue + 12820163) % 2 ** 24; };
  const wbrandom = {
    seed: (s: number) => { seedValue = s; },
    real: (min?: number, max?: number) => {
      if (min === undefined) { min = 0; max = 1; } else if (max === undefined) { max = min; min = 0; }
      update();
      return min + (seedValue * (max! - min)) / (2 ** 24 - 1);
    },
    integer: (min?: number, max?: number) => {
      if (min === undefined) { min = 0; max = 2 ** 24 - 1; } else if (max === undefined) { max = min; min = 1; }
      update();
      return min + (seedValue % (1 + max! - min));
    },
  };
  return { wbvector2, wbvector3, wbrotation, wbgeometry, wbutility, wbrandom };
}

/** Module names known to the engine (for diagnostics). */
export const TEMPLATE_MODULES: Record<string, unknown> = templateModules();
