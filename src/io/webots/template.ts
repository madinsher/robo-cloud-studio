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
  id: string;
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
  const logs: string[] = [];
  const consoleShim = { log: (...a: unknown[]) => logs.push(a.join(' ')), warn: (...a: unknown[]) => logs.push(a.join(' ')), error: (...a: unknown[]) => logs.push(a.join(' ')), info: (...a: unknown[]) => logs.push(a.join(' ')) };
  const __imports = (mod: string) => {
    const key = mod.replace(/^.*[\\/]/, '').replace(/\.js$/, '');
    const shim = TEMPLATE_MODULES[key];
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
  }
  return parts.join('');
}

// ---- clean-room shims of the Webots template modules (wbvector2/3, wbrotation, wbgeometry, wbutility, wbrandom) -------

type V2 = { x: number; y: number };
type V3 = { x: number; y: number; z: number };
type Rot = { x: number; y: number; z: number; a: number };
type Quat = { w: number; x: number; y: number; z: number };

const wbvector2 = {
  add: (a: V2, b: V2): V2 => ({ x: a.x + b.x, y: a.y + b.y }),
  minus: (a: V2, b: V2): V2 => ({ x: a.x - b.x, y: a.y - b.y }),
  multiply: (a: V2, s: number): V2 => ({ x: a.x * s, y: a.y * s }),
  dot: (a: V2, b: V2) => a.x * b.x + a.y * b.y,
  norm: (a: V2) => Math.hypot(a.x, a.y),
  distance: (a: V2, b: V2) => Math.hypot(a.x - b.x, a.y - b.y),
  normalize: (a: V2): V2 => { const l = Math.hypot(a.x, a.y) || 1; return { x: a.x / l, y: a.y / l }; },
  angle: (a: V2, b: V2) => Math.atan2(b.y - a.y, b.x - a.x),
  equal: (a: V2, b: V2) => a.x === b.x && a.y === b.y,
};

const wbvector3 = {
  add: (a: V3, b: V3): V3 => ({ x: a.x + b.x, y: a.y + b.y, z: a.z + b.z }),
  minus: (a: V3, b: V3): V3 => ({ x: a.x - b.x, y: a.y - b.y, z: a.z - b.z }),
  multiply: (a: V3, s: number): V3 => ({ x: a.x * s, y: a.y * s, z: a.z * s }),
  dot: (a: V3, b: V3) => a.x * b.x + a.y * b.y + a.z * b.z,
  cross: (a: V3, b: V3): V3 => ({ x: a.y * b.z - a.z * b.y, y: a.z * b.x - a.x * b.z, z: a.x * b.y - a.y * b.x }),
  norm: (a: V3) => Math.hypot(a.x, a.y, a.z),
  distance: (a: V3, b: V3) => Math.hypot(a.x - b.x, a.y - b.y, a.z - b.z),
  normalize: (a: V3): V3 => { const l = Math.hypot(a.x, a.y, a.z) || 1; return { x: a.x / l, y: a.y / l, z: a.z / l }; },
  equal: (a: V3, b: V3) => a.x === b.x && a.y === b.y && a.z === b.z,
};

function quatFromRot(r: Rot): Quat {
  const l = Math.hypot(r.x, r.y, r.z) || 1, h = r.a / 2, s = Math.sin(h);
  return { w: Math.cos(h), x: (r.x / l) * s, y: (r.y / l) * s, z: (r.z / l) * s };
}
function rotFromQuat(q: Quat): Rot {
  const w = Math.max(-1, Math.min(1, q.w));
  const a = 2 * Math.acos(w), s = Math.sqrt(Math.max(0, 1 - w * w));
  return s < 1e-9 ? { x: 0, y: 0, z: 1, a: 0 } : { x: q.x / s, y: q.y / s, z: q.z / s, a };
}
function quatMul(a: Quat, b: Quat): Quat {
  return { w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z, x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y, y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x, z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w };
}
function matFromRot(r: Rot): number[] {
  const l = Math.hypot(r.x, r.y, r.z) || 1, x = r.x / l, y = r.y / l, z = r.z / l, c = Math.cos(r.a), s = Math.sin(r.a), t = 1 - c;
  return [t * x * x + c, t * x * y - s * z, t * x * z + s * y, t * x * y + s * z, t * y * y + c, t * y * z - s * x, t * x * z - s * y, t * y * z + s * x, t * z * z + c];
}
function rotFromMat(m: number[]): Rot {
  const tr = m[0] + m[4] + m[8];
  let q: Quat;
  if (tr > 0) { const s = Math.sqrt(tr + 1) * 2; q = { w: s / 4, x: (m[7] - m[5]) / s, y: (m[2] - m[6]) / s, z: (m[3] - m[1]) / s }; }
  else if (m[0] > m[4] && m[0] > m[8]) { const s = Math.sqrt(1 + m[0] - m[4] - m[8]) * 2; q = { w: (m[7] - m[5]) / s, x: s / 4, y: (m[1] + m[3]) / s, z: (m[2] + m[6]) / s }; }
  else if (m[4] > m[8]) { const s = Math.sqrt(1 + m[4] - m[0] - m[8]) * 2; q = { w: (m[2] - m[6]) / s, x: (m[1] + m[3]) / s, y: s / 4, z: (m[5] + m[7]) / s }; }
  else { const s = Math.sqrt(1 + m[8] - m[0] - m[4]) * 2; q = { w: (m[3] - m[1]) / s, x: (m[2] + m[6]) / s, y: (m[5] + m[7]) / s, z: s / 4 }; }
  return rotFromQuat(q);
}

const wbrotation = {
  fromAxisAngle: (x: number, y: number, z: number, a: number): Rot => ({ x, y, z, a }),
  toQuaternion: (r: Rot): Quat => quatFromRot(r),
  fromQuaternion: (q: Quat): Rot => rotFromQuat(q),
  toMatrix3: (r: Rot): number[] => matFromRot(r),
  fromMatrix3: (m: number[]): Rot => rotFromMat(m),
  combine: (a: Rot, b: Rot): Rot => rotFromQuat(quatMul(quatFromRot(a), quatFromRot(b))),
  multiply: (a: Rot, b: Rot): Rot => rotFromQuat(quatMul(quatFromRot(a), quatFromRot(b))),
  rotateVector3ByMatrix3: (m: number[], v: V3): V3 => ({ x: m[0] * v.x + m[1] * v.y + m[2] * v.z, y: m[3] * v.x + m[4] * v.y + m[5] * v.z, z: m[6] * v.x + m[7] * v.y + m[8] * v.z }),
  rotateVector3ByRotation: (r: Rot, v: V3): V3 => { const m = matFromRot(r); return { x: m[0] * v.x + m[1] * v.y + m[2] * v.z, y: m[3] * v.x + m[4] * v.y + m[5] * v.z, z: m[6] * v.x + m[7] * v.y + m[8] * v.z }; },
  equal: (a: Rot, b: Rot) => a.x === b.x && a.y === b.y && a.z === b.z && a.a === b.a,
  isIdentity: (r: Rot) => r.a === 0,
  normalize: (r: Rot): Rot => { const l = Math.hypot(r.x, r.y, r.z) || 1; return { x: r.x / l, y: r.y / l, z: r.z / l, a: r.a }; },
};

const wbgeometry = {
  /** Points on a circle in the XY plane (div segments, optional start angle). */
  circle: (radius: number, div: number, cx = 0, cy = 0, shift = 0): V2[] => Array.from({ length: div }, (_, i) => ({ x: cx + radius * Math.cos(shift + (2 * Math.PI * i) / div), y: cy + radius * Math.sin(shift + (2 * Math.PI * i) / div) })),
  isPoint2InTriangle: (p: V2, a: V2, b: V2, c: V2) => {
    const s = (p1: V2, p2: V2, p3: V2) => (p1.x - p3.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p3.y);
    const d1 = s(p, a, b), d2 = s(p, b, c), d3 = s(p, c, a);
    return !((d1 < 0 || d2 < 0 || d3 < 0) && (d1 > 0 || d2 > 0 || d3 > 0));
  },
  /** Uniform cubic B-spline through control points (open), `subdivision` samples per span. */
  bspline: (pts: V3[], subdivision: number): V3[] => {
    if (pts.length < 3) return pts.slice();
    const out: V3[] = [];
    const p = [pts[0], ...pts, pts[pts.length - 1]];
    for (let i = 0; i + 3 < p.length; i++) for (let k = 0; k < subdivision; k++) {
      const t = k / subdivision, it = 1 - t;
      const b0 = (it * it * it) / 6, b1 = (3 * t * t * t - 6 * t * t + 4) / 6, b2 = (-3 * t * t * t + 3 * t * t + 3 * t + 1) / 6, b3 = (t * t * t) / 6;
      out.push({ x: b0 * p[i].x + b1 * p[i + 1].x + b2 * p[i + 2].x + b3 * p[i + 3].x, y: b0 * p[i].y + b1 * p[i + 1].y + b2 * p[i + 2].y + b3 * p[i + 3].y, z: b0 * (p[i].z ?? 0) + b1 * (p[i + 1].z ?? 0) + b2 * (p[i + 2].z ?? 0) + b3 * (p[i + 3].z ?? 0) });
    }
    out.push(pts[pts.length - 1]);
    return out;
  },
};

const wbutility = {
  deepCopy: <T>(o: T): T => JSON.parse(JSON.stringify(o)),
  isScalar: (v: unknown) => typeof v === 'number',
  isVector2: (v: unknown) => !!v && typeof v === 'object' && 'x' in (v as object) && 'y' in (v as object) && !('z' in (v as object)),
  isVector3: (v: unknown) => !!v && typeof v === 'object' && 'x' in (v as object) && 'y' in (v as object) && 'z' in (v as object) && !('a' in (v as object)),
  isRotation: (v: unknown) => !!v && typeof v === 'object' && 'a' in (v as object),
  isColor: (v: unknown) => !!v && typeof v === 'object' && 'r' in (v as object) && 'g' in (v as object) && 'b' in (v as object),
  isArray: (v: unknown) => Array.isArray(v),
};

/** Deterministic PRNG (mulberry32), seeded like Webots templates with a numeric seed. */
let rngState = 1;
const wbrandom = {
  seed: (s: number) => { rngState = (Math.floor(s) >>> 0) || 1; },
  real: (min = 0, max = 1) => {
    rngState = (rngState + 0x6d2b79f5) >>> 0;
    let t = rngState;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    const u = ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    return min + (max - min) * u;
  },
  integer: (min = 0, max = 100) => Math.floor(min + (max - min + 1) * wbrandom.real()),
};

export const TEMPLATE_MODULES: Record<string, unknown> = { wbvector2, wbvector3, wbrotation, wbgeometry, wbutility, wbrandom };
