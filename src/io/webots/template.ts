/**
 * JavaScript template engine for Webots PROTOs (`# template language: javascript`). The body is compiled into a function:
 * text chunks are appended verbatim, `%<= expr >%` appends String(expr), `%< statements >%` are inlined. The function runs
 * in strict mode with the studio globals shadowed (window, document, fetch, ...), a loop/time guard and an output size
 * limit. `import ... from 'wbxxx.js'` statements map to clean-room shims of the Webots template modules.
 * Field values follow the Webots shapes: SFVec3f {x,y,z}, SFVec2f {x,y}, SFColor {r,g,b}, SFRotation {x,y,z,a},
 * scalars, MF* arrays, SFNode {node_name, fields} | null.
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

const SHADOWED = ['window', 'document', 'globalThis', 'self', 'global', 'process', 'require', 'module', 'exports', 'fetch', 'XMLHttpRequest', 'WebSocket', 'Worker', 'SharedWorker', 'importScripts', 'localStorage', 'sessionStorage', 'indexedDB', 'navigator', 'location', 'setTimeout', 'setInterval', 'queueMicrotask', 'postMessage', 'Deno', 'Bun'];

/** Transform ES module imports of template modules into lookups in `__imports`. */
function rewriteImports(code: string): string {
  return code
    .replace(/import\s+\*\s+as\s+([A-Za-z_$][\w$]*)\s+from\s+['"]([^'"]+)['"]\s*;?/g, (_m, id, mod) => `const ${id} = __imports(${JSON.stringify(mod)});`)
    .replace(/import\s+\{([^}]*)\}\s+from\s+['"]([^'"]+)['"]\s*;?/g, (_m, list: string, mod) => `const {${list.replace(/\s+as\s+/g, ': ')}} = __imports(${JSON.stringify(mod)});`)
    .replace(/import\s+([A-Za-z_$][\w$]*)\s+from\s+['"]([^'"]+)['"]\s*;?/g, (_m, id, mod) => `const ${id} = __imports(${JSON.stringify(mod)});`);
}

/** Insert a guard call at the start of every braced loop body (for / while / do). */
function guardLoops(code: string): string {
  let out = '';
  let i = 0;
  const n = code.length;
  const re = /\b(for|while)\s*\(|\bdo\s*\{/g;
  let m: RegExpExecArray | null;
  while ((m = re.exec(code))) {
    if (m[0].startsWith('do')) { out += code.slice(i, m.index + m[0].length) + '__g();'; i = m.index + m[0].length; continue; }
    // find the matching ')'
    let j = m.index + m[0].length, depth = 1;
    while (j < n && depth > 0) {
      const c = code[j];
      if (c === '"' || c === "'" || c === '`') { const q = c; j++; while (j < n && code[j] !== q) { if (code[j] === '\\') j++; j++; } }
      else if (c === '(') depth++;
      else if (c === ')') depth--;
      j++;
    }
    let k = j;
    while (k < n && /\s/.test(code[k])) k++;
    if (code[k] === '{') { out += code.slice(i, k + 1) + '__g();'; i = k + 1; re.lastIndex = k + 1; }
    else re.lastIndex = j;
  }
  return out + code.slice(i);
}

/** Compile a template body to JavaScript source (exported for tests / debugging). */
export function compileTemplate(body: string): string {
  let js = '';
  const re = /%<(=?)([\s\S]*?)>%/g;
  let last = 0;
  let m: RegExpExecArray | null;
  while ((m = re.exec(body))) {
    js += `__w(${JSON.stringify(body.slice(last, m.index))});\n`;
    js += m[1] ? `__w(${m[2]});\n` : `${guardLoops(rewriteImports(m[2]))}\n`;
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
}

/** Expand a template body. Throws TemplateError (with the JS error message) on failure. */
export function expandTemplate(body: string, fields: Record<string, { value: unknown; defaultValue: unknown }>, context: TemplateContext, opts: TemplateOptions = {}): string {
  const maxIt = opts.maxIterations ?? 5e6, maxOut = opts.maxOutput ?? 64e6, maxMs = opts.maxMs ?? 5000;
  const parts: string[] = [];
  let size = 0, iters = 0;
  const t0 = Date.now();
  const __w = (x: unknown) => {
    const s = String(x);
    size += s.length;
    if (size > maxOut) throw new TemplateError('template output limit exceeded');
    parts.push(s);
  };
  const __g = () => {
    if (++iters > maxIt || (iters % 4096 === 0 && Date.now() - t0 > maxMs)) throw new TemplateError('template loop limit exceeded');
  };
  const stdout: string[] = [], stderr: string[] = [];
  const consoleShim = { log: (...a: unknown[]) => stdout.push(a.join(' ')), info: (...a: unknown[]) => stdout.push(a.join(' ')), debug: (...a: unknown[]) => stdout.push(a.join(' ')), warn: (...a: unknown[]) => stderr.push(a.join(' ')), error: (...a: unknown[]) => stderr.push(a.join(' ')) };
  const modules = templateModules(stderr, stdout);
  const __imports = (mod: string) => {
    const key = mod.replace(/^.*[\\/]/, '').replace(/\.js$/, '');
    const shim = modules[key];
    if (!shim) throw new TemplateError(`unsupported template module '${mod}'`);
    return shim;
  };
  let fn: (...args: unknown[]) => void;
  try {
    // eslint-disable-next-line no-new-func
    fn = new Function('fields', 'context', '__w', '__g', '__imports', 'console', ...SHADOWED, `"use strict";\n${compileTemplate(body)}`) as (...args: unknown[]) => void;
  } catch (e) {
    throw new TemplateError(`template compile error: ${(e as Error).message}`);
  }
  try {
    fn(fields, context, __w, __g, __imports, consoleShim, ...SHADOWED.map(() => undefined));
  } catch (e) {
    if (e instanceof TemplateError) throw e;
    throw new TemplateError(`template error: ${(e as Error).message}`);
  } finally {
    opts.messages?.push(...stderr);
  }
  return parts.join('');
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
