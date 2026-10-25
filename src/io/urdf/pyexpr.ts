/**
 * Sandboxed evaluator for the Python expression subset used by ROS xacro (`${...}`, `$(eval ...)`, yaml unit tags).
 * No JavaScript `eval`/`Function`: expressions are tokenized, parsed to an AST and interpreted with Python 3 semantics
 * (int vs float typing, true division, floor division/modulo signs, banker's rounding, `str()`/`repr()` formatting,
 * `%`-formatting, chained comparisons, comprehensions, lambdas, slicing). Attribute access only reaches whitelisted
 * methods of the value kinds below, never JS object properties.
 *
 * Value model: None=null, bool=boolean, int=bigint, float=number, str=string, list=Array, tuple=PyTuple, dict=PyDict,
 * range=PyRange, callables=PyFunc, namespaces = objects implementing PyObj.getAttr.
 */

export class PyError extends Error {
  constructor(readonly pyType: string, message: string) { super(message); this.name = pyType; }
  toString(): string { return `${this.pyType}: ${this.message}`; }
}
const err = (t: string, m: string) => new PyError(t, m);

export interface PyObj { getAttr?(name: string): any; getItem?(key: any): any; has?(key: any): boolean; typeName?: string }
export class PyTuple { constructor(readonly items: any[]) {} }
export class PyRange { constructor(readonly start: bigint, readonly stop: bigint, readonly step: bigint) {}
  items(): bigint[] { const r: bigint[] = []; if (this.step > 0n) for (let i = this.start; i < this.stop; i += this.step) r.push(i); else for (let i = this.start; i > this.stop; i += this.step) r.push(i); return r; }
}
export type PyCall = (args: any[], kw: Map<string, any>) => any;
export class PyFunc { constructor(readonly name: string, readonly fn: PyCall) {} }
export const pyfn = (name: string, fn: PyCall) => new PyFunc(name, fn);

/** Ordered dict with Python key equality (1 == 1.0 == True). `dotted` enables attribute access (xacro.dotify / load_yaml). */
export class PyDict implements PyObj {
  private m = new Map<string, [any, any]>();
  dotted = false;
  constructor(entries?: Iterable<[any, any]>, dotted = false) { if (entries) for (const [k, v] of entries) this.set(k, v); this.dotted = dotted; }
  get typeName() { return 'dict'; }
  set(k: any, v: any) { this.m.set(hashKey(k), [k, v]); }
  get(k: any): any { const e = this.m.get(hashKey(k)); if (!e) throw err('KeyError', pyRepr(k)); return this.wrap(e[1]); }
  tryGet(k: any): any { const e = this.m.get(hashKey(k)); return e ? this.wrap(e[1]) : undefined; }
  has(k: any) { return this.m.has(hashKey(k)); }
  keys(): any[] { return [...this.m.values()].map((e) => e[0]); }
  values(): any[] { return [...this.m.values()].map((e) => this.wrap(e[1])); }
  entries(): [any, any][] { return [...this.m.values()].map((e) => [e[0], this.wrap(e[1])]); }
  get size() { return this.m.size; }
  getItem(k: any) { return this.get(k); }
  getAttr(name: string) {
    if (this.dotted && this.has(name)) return this.get(name);
    const meth = dictMethod(this, name);
    if (meth) return meth;
    if (this.dotted) throw err('AttributeError', `The yaml dictionary has no key '${name}'`);
    throw err('AttributeError', `'dict' object has no attribute '${name}'`);
  }
  private wrap(v: any) { return this.dotted && v instanceof PyDict && !v.dotted ? new PyDict(v.entries(), true) : v; }
}

function hashKey(k: any): string {
  if (k === null) return 'N';
  if (typeof k === 'boolean') return `n:${k ? 1 : 0}`;
  if (typeof k === 'bigint') return `n:${k}`;
  if (typeof k === 'number') return Number.isInteger(k) && Math.abs(k) < 2 ** 53 ? `n:${BigInt(k)}` : `f:${k}`;
  if (typeof k === 'string') return `s:${k}`;
  if (k instanceof PyTuple) return `t:(${k.items.map(hashKey).join(',')})`;
  throw err('TypeError', `unhashable type: '${pyType(k)}'`);
}

// ---------------------------------------------------------------------------------------------
// type helpers
// ---------------------------------------------------------------------------------------------
export function pyType(v: any): string {
  if (v === null || v === undefined) return 'NoneType';
  if (typeof v === 'boolean') return 'bool';
  if (typeof v === 'bigint') return 'int';
  if (typeof v === 'number') return 'float';
  if (typeof v === 'string') return 'str';
  if (Array.isArray(v)) return 'list';
  if (v instanceof PyTuple) return 'tuple';
  if (v instanceof PyDict) return 'dict';
  if (v instanceof PyRange) return 'range';
  if (v instanceof PyFunc) return 'builtin_function_or_method';
  return v.typeName ?? 'object';
}
const isIntLike = (v: any): v is bigint | boolean => typeof v === 'bigint' || typeof v === 'boolean';
const isNum = (v: any) => typeof v === 'bigint' || typeof v === 'number' || typeof v === 'boolean';
const big = (v: bigint | boolean): bigint => (typeof v === 'boolean' ? (v ? 1n : 0n) : v);
const flt = (v: any): number => (typeof v === 'number' ? v : typeof v === 'boolean' ? +v : Number(v));

export function truthy(v: any): boolean {
  if (v === null || v === undefined) return false;
  if (typeof v === 'boolean') return v;
  if (typeof v === 'bigint') return v !== 0n;
  if (typeof v === 'number') return v !== 0;
  if (typeof v === 'string') return v.length > 0;
  if (Array.isArray(v)) return v.length > 0;
  if (v instanceof PyTuple) return v.items.length > 0;
  if (v instanceof PyDict) return v.size > 0;
  if (v instanceof PyRange) return v.items().length > 0;
  return true;
}

/** Python repr() of a float (shortest round-trip digits, exponent outside 1e-4 <= |x| < 1e16). */
export function floatRepr(x: number): string {
  if (Number.isNaN(x)) return 'nan';
  if (!Number.isFinite(x)) return x > 0 ? 'inf' : '-inf';
  if (x === 0) return Object.is(x, -0) ? '-0.0' : '0.0';
  const [mant, e] = Math.abs(x).toExponential().split('e');
  const digits = mant.replace('.', '');
  const exp = parseInt(e, 10);
  const sign = x < 0 ? '-' : '';
  if (exp < -4 || exp >= 16) {
    const m = digits.length > 1 ? `${digits[0]}.${digits.slice(1)}` : digits;
    return `${sign}${m}e${exp < 0 ? '-' : '+'}${String(Math.abs(exp)).padStart(2, '0')}`;
  }
  if (exp < 0) return `${sign}0.${'0'.repeat(-exp - 1)}${digits}`;
  if (digits.length <= exp + 1) return `${sign}${digits}${'0'.repeat(exp + 1 - digits.length)}.0`;
  return `${sign}${digits.slice(0, exp + 1)}.${digits.slice(exp + 1)}`;
}

function strRepr(s: string): string {
  const q = s.includes("'") && !s.includes('"') ? '"' : "'";
  let out = '';
  for (const ch of s) {
    if (ch === '\\') out += '\\\\';
    else if (ch === q) out += `\\${q}`;
    else if (ch === '\n') out += '\\n';
    else if (ch === '\r') out += '\\r';
    else if (ch === '\t') out += '\\t';
    else if (ch.charCodeAt(0) < 0x20 || ch.charCodeAt(0) === 0x7f) out += `\\x${ch.charCodeAt(0).toString(16).padStart(2, '0')}`;
    else out += ch;
  }
  return q + out + q;
}

export function pyRepr(v: any): string {
  if (typeof v === 'string') return strRepr(v);
  if (Array.isArray(v)) return `[${v.map(pyRepr).join(', ')}]`;
  if (v instanceof PyTuple) return v.items.length === 1 ? `(${pyRepr(v.items[0])},)` : `(${v.items.map(pyRepr).join(', ')})`;
  if (v instanceof PyDict) return `{${v.entries().map(([k, x]) => `${pyRepr(k)}: ${pyRepr(x)}`).join(', ')}}`;
  return pyStr(v);
}

/** Python str(). */
export function pyStr(v: any): string {
  if (v === null || v === undefined) return 'None';
  if (typeof v === 'boolean') return v ? 'True' : 'False';
  if (typeof v === 'bigint') return v.toString();
  if (typeof v === 'number') return floatRepr(v);
  if (typeof v === 'string') return v;
  if (v instanceof PyRange) return v.step === 1n ? `range(${v.start}, ${v.stop})` : `range(${v.start}, ${v.stop}, ${v.step})`;
  if (v instanceof PyFunc) return `<built-in function ${v.name}>`;
  if (Array.isArray(v) || v instanceof PyTuple || v instanceof PyDict) return pyRepr(v);
  if (typeof v.pyStr === 'function') return v.pyStr();
  return `<${pyType(v)} object>`;
}

export function iterate(v: any): any[] {
  if (typeof v === 'string') return [...v];
  if (Array.isArray(v)) return v;
  if (v instanceof PyTuple) return v.items;
  if (v instanceof PyDict) return v.keys();
  if (v instanceof PyRange) return v.items();
  throw err('TypeError', `'${pyType(v)}' object is not iterable`);
}

// ---------------------------------------------------------------------------------------------
// equality / ordering / arithmetic
// ---------------------------------------------------------------------------------------------
export function pyEq(a: any, b: any): boolean {
  if (isNum(a) && isNum(b)) return isIntLike(a) && isIntLike(b) ? big(a) === big(b) : flt(a) === flt(b);
  if (a === null || b === null || a === undefined || b === undefined) return (a ?? null) === (b ?? null);
  if (typeof a === 'string' || typeof b === 'string') return a === b;
  if (Array.isArray(a) && Array.isArray(b)) return a.length === b.length && a.every((x, i) => pyEq(x, b[i]));
  if (a instanceof PyTuple && b instanceof PyTuple) return a.items.length === b.items.length && a.items.every((x, i) => pyEq(x, b.items[i]));
  if (a instanceof PyDict && b instanceof PyDict) return a.size === b.size && a.entries().every(([k, v]) => b.has(k) && pyEq(v, b.get(k)));
  if (a instanceof PyRange && b instanceof PyRange) return pyEq(a.items(), b.items());
  return a === b;
}

function pyCmp(a: any, b: any, op: string): number {
  if (isNum(a) && isNum(b)) {
    if (isIntLike(a) && isIntLike(b)) { const x = big(a), y = big(b); return x < y ? -1 : x > y ? 1 : 0; }
    const x = flt(a), y = flt(b);
    if (Number.isNaN(x) || Number.isNaN(y)) return NaN;
    return x < y ? -1 : x > y ? 1 : 0;
  }
  if (typeof a === 'string' && typeof b === 'string') return a < b ? -1 : a > b ? 1 : 0;
  const seq = (v: any) => (Array.isArray(v) ? v : v instanceof PyTuple ? v.items : null);
  const sa = seq(a), sb = seq(b);
  if (sa && sb && Array.isArray(a) === Array.isArray(b)) {
    for (let i = 0; i < Math.min(sa.length, sb.length); i++) if (!pyEq(sa[i], sb[i])) return pyCmp(sa[i], sb[i], op);
    return sa.length - sb.length;
  }
  throw err('TypeError', `'${op}' not supported between instances of '${pyType(a)}' and '${pyType(b)}'`);
}

function opErr(op: string, a: any, b: any): PyError { return err('TypeError', `unsupported operand type(s) for ${op}: '${pyType(a)}' and '${pyType(b)}'`); }

function floorDivBig(a: bigint, b: bigint): bigint {
  if (b === 0n) throw err('ZeroDivisionError', 'integer division or modulo by zero');
  const q = a / b;
  return (a % b !== 0n && (a < 0n) !== (b < 0n)) ? q - 1n : q;
}
function modBig(a: bigint, b: bigint): bigint {
  if (b === 0n) throw err('ZeroDivisionError', 'integer division or modulo by zero');
  const r = a % b;
  return r !== 0n && (r < 0n) !== (b < 0n) ? r + b : r;
}
function modFloat(a: number, b: number): number {
  if (b === 0) throw err('ZeroDivisionError', 'float modulo');
  let r = a % b;
  if (r !== 0 && (r < 0) !== (b < 0)) r += b;
  else if (r === 0) r = b < 0 ? -0 : 0;
  return r;
}

export function binop(op: string, a: any, b: any): any {
  switch (op) {
    case '+':
      if (isIntLike(a) && isIntLike(b)) return big(a) + big(b);
      if (isNum(a) && isNum(b)) return flt(a) + flt(b);
      if (typeof a === 'string' && typeof b === 'string') return a + b;
      if (Array.isArray(a) && Array.isArray(b)) return [...a, ...b];
      if (a instanceof PyTuple && b instanceof PyTuple) return new PyTuple([...a.items, ...b.items]);
      throw opErr(op, a, b);
    case '-':
      if (isIntLike(a) && isIntLike(b)) return big(a) - big(b);
      if (isNum(a) && isNum(b)) return flt(a) - flt(b);
      throw opErr(op, a, b);
    case '*': {
      if (isIntLike(a) && isIntLike(b)) return big(a) * big(b);
      if (isNum(a) && isNum(b)) return flt(a) * flt(b);
      const rep = (s: any, n: any) => {
        const k = Math.max(0, Number(big(n)));
        if (typeof s === 'string') return s.repeat(k);
        if (Array.isArray(s)) return Array.from({ length: k }, () => s).flat();
        return new PyTuple(Array.from({ length: k }, () => (s as PyTuple).items).flat());
      };
      if ((typeof a === 'string' || Array.isArray(a) || a instanceof PyTuple) && isIntLike(b)) return rep(a, b);
      if ((typeof b === 'string' || Array.isArray(b) || b instanceof PyTuple) && isIntLike(a)) return rep(b, a);
      throw opErr(op, a, b);
    }
    case '/':
      if (isNum(a) && isNum(b)) { const y = flt(b); if (y === 0) throw err('ZeroDivisionError', 'division by zero'); return flt(a) / y; }
      throw opErr(op, a, b);
    case '//':
      if (isIntLike(a) && isIntLike(b)) return floorDivBig(big(a), big(b));
      if (isNum(a) && isNum(b)) { const y = flt(b); if (y === 0) throw err('ZeroDivisionError', 'float floor division by zero'); return Math.floor(flt(a) / y); }
      throw opErr(op, a, b);
    case '%':
      if (typeof a === 'string') return percentFormat(a, b);
      if (isIntLike(a) && isIntLike(b)) return modBig(big(a), big(b));
      if (isNum(a) && isNum(b)) return modFloat(flt(a), flt(b));
      throw opErr(op, a, b);
    case '**':
      if (isIntLike(a) && isIntLike(b)) {
        const x = big(a), y = big(b);
        if (y >= 0n) return x ** y;
        if (x === 0n) throw err('ZeroDivisionError', '0.0 cannot be raised to a negative power');
        return Number(x) ** Number(y);
      }
      if (isNum(a) && isNum(b)) {
        const x = flt(a), y = flt(b);
        if (x === 0 && y < 0) throw err('ZeroDivisionError', '0.0 cannot be raised to a negative power');
        if (x < 0 && !Number.isInteger(y)) throw err('ValueError', 'complex results are not supported');
        return x ** y;
      }
      throw opErr(op, a, b);
    case '<<': case '>>': case '&': case '|': case '^': {
      if (!(isIntLike(a) && isIntLike(b))) throw opErr(op, a, b);
      if (typeof a === 'boolean' && typeof b === 'boolean' && op !== '<<' && op !== '>>') return op === '&' ? a && b : op === '|' ? a || b : a !== b;
      const x = big(a), y = big(b);
      return op === '<<' ? x << y : op === '>>' ? x >> y : op === '&' ? x & y : op === '|' ? x | y : x ^ y;
    }
    case '==': return pyEq(a, b);
    case '!=': return !pyEq(a, b);
    case '<': return pyCmp(a, b, op) < 0;
    case '<=': return pyCmp(a, b, op) <= 0;
    case '>': return pyCmp(a, b, op) > 0;
    case '>=': return pyCmp(a, b, op) >= 0;
    case 'in': return contains(b, a);
    case 'not in': return !contains(b, a);
    case 'is': return a === b || (a == null && b == null) || (isIntLike(a) && isIntLike(b) && typeof a === typeof b && big(a) === big(b));
    case 'is not': return !binop('is', a, b);
  }
  throw err('SyntaxError', `unsupported operator ${op}`);
}

function contains(container: any, x: any): boolean {
  if (typeof container === 'string') { if (typeof x !== 'string') throw err('TypeError', `'in <string>' requires string as left operand, not ${pyType(x)}`); return container.includes(x); }
  if (container instanceof PyDict) return container.has(x);
  if (container && typeof container.has === 'function' && !(container instanceof Map)) return container.has(x);
  return iterate(container).some((y) => pyEq(y, x));
}

// ---------------------------------------------------------------------------------------------
// formatting: % operator, str.format
// ---------------------------------------------------------------------------------------------
function toIntForFormat(v: any, spec: string): bigint {
  if (isIntLike(v)) return big(v);
  if (typeof v === 'number') { if (!Number.isFinite(v)) throw err('OverflowError', 'cannot convert float infinity to integer'); return BigInt(Math.trunc(v)); }
  throw err('TypeError', `%${spec} format: a real number is required, not ${pyType(v)}`);
}
function fmtExp(x: number, prec: number, upper: boolean): string {
  let s = x.toExponential(prec);
  s = s.replace(/e([+-])(\d)$/, 'e$10$2');
  return upper ? s.toUpperCase() : s;
}
function fmtGeneral(x: number, prec: number, alt: boolean, upper: boolean): string {
  if (!Number.isFinite(x)) return Number.isNaN(x) ? (upper ? 'NAN' : 'nan') : (x > 0 ? (upper ? 'INF' : 'inf') : (upper ? '-INF' : '-inf'));
  const p = prec === 0 ? 1 : prec;
  if (x === 0) return alt ? (0).toFixed(p - 1) : (Object.is(x, -0) ? '-0' : '0');
  const exp = parseInt(x.toExponential(p - 1).split('e')[1], 10);
  let s: string;
  if (exp >= -4 && exp < p) s = x.toFixed(Math.max(0, p - 1 - exp));
  else s = fmtExp(x, p - 1, upper);
  if (!alt) {
    if (s.includes('e') || s.includes('E')) s = s.replace(/\.?0+(?=[eE])/, '');
    else if (s.includes('.')) s = s.replace(/\.?0+$/, '');
  }
  return s;
}
function fmtFloatSpec(x: number, type: string, prec: number | undefined, alt: boolean): string {
  if (!Number.isFinite(x) && type !== 'r' && type !== 's') return Number.isNaN(x) ? 'nan' : x > 0 ? 'inf' : '-inf';
  switch (type) {
    case 'f': case 'F': return x.toFixed(prec ?? 6);
    case 'e': case 'E': return fmtExp(x, prec ?? 6, type === 'E');
    case 'g': case 'G': return fmtGeneral(x, prec ?? 6, alt, type === 'G');
    case '%': return `${(x * 100).toFixed(prec ?? 6)}%`;
  }
  return floatRepr(x);
}
function pad(s: string, width: number, left: boolean, zero: boolean, align?: string, fill = ' '): string {
  if (s.length >= width) return s;
  const n = width - s.length;
  if (zero) { const m = /^[+\- ]/.exec(s); return m ? m[0] + '0'.repeat(n) + s.slice(1) : '0'.repeat(n) + s; }
  if (align === '^') return fill.repeat(Math.floor(n / 2)) + s + fill.repeat(n - Math.floor(n / 2));
  if (align === '=') { const m = /^[+\- ]/.exec(s); return m ? m[0] + fill.repeat(n) + s.slice(1) : fill.repeat(n) + s; }
  return left ? s + fill.repeat(n) : fill.repeat(n) + s;
}

export function percentFormat(fmt: string, arg: any): string {
  const args = arg instanceof PyTuple ? arg.items : [arg];
  let ai = 0;
  const next = () => { if (ai >= args.length) throw err('TypeError', 'not enough arguments for format string'); return args[ai++]; };
  let usedMap = false;
  const out = fmt.replace(/%(\([^)]*\))?([-+ #0]*)(\*|\d+)?(?:\.(\*|\d+))?[hlL]?([diouxXeEfFgGcrsa%])/g, (_m, key, flags: string, w, p, conv: string) => {
    if (conv === '%') return '%';
    let width = w === '*' ? Number(next()) : w ? parseInt(w, 10) : 0;
    const prec = p === '*' ? Number(next()) : p !== undefined ? parseInt(p, 10) : undefined;
    let v: any;
    if (key) { usedMap = true; if (!(arg instanceof PyDict)) throw err('TypeError', 'format requires a mapping'); v = arg.get(key.slice(1, -1)); } else v = next();
    const left = flags.includes('-'), zero = flags.includes('0') && !left, plus = flags.includes('+'), space = flags.includes(' '), alt = flags.includes('#');
    const sign = (neg: boolean, s: string) => (neg ? '-' : plus ? '+' : space ? ' ' : '') + s;
    let s: string;
    switch (conv) {
      case 's': s = pyStr(v); if (prec !== undefined) s = s.slice(0, prec); break;
      case 'r': case 'a': s = pyRepr(v); if (prec !== undefined) s = s.slice(0, prec); break;
      case 'c': s = typeof v === 'string' ? v : String.fromCodePoint(Number(big(v as any))); break;
      case 'd': case 'i': case 'u': { const n = toIntForFormat(v, conv); s = sign(n < 0n, (n < 0n ? -n : n).toString()); break; }
      case 'o': case 'x': case 'X': {
        const n = toIntForFormat(v, conv); const base = conv === 'o' ? 8 : 16;
        let d = (n < 0n ? -n : n).toString(base); if (conv === 'X') d = d.toUpperCase();
        if (alt) d = (conv === 'o' ? '0o' : conv === 'x' ? '0x' : '0X') + d;
        s = sign(n < 0n, d); break;
      }
      default: {
        if (!isNum(v)) throw err('TypeError', `must be real number, not ${pyType(v)}`);
        const x = flt(v);
        const body = fmtFloatSpec(Math.abs(x), conv, prec, alt);
        s = sign(x < 0 || Object.is(x, -0), body);
      }
    }
    if (width < 0) { width = -width; return pad(s, width, true, false); }
    return pad(s, width, left, zero && conv !== 's' && conv !== 'r');
  });
  if (!usedMap && ai < args.length && !(arg instanceof PyDict)) throw err('TypeError', 'not all arguments converted during string formatting');
  return out;
}

function formatSpec(v: any, spec: string): string {
  const m = /^(?:(.)?([<>=^]))?([-+ ])?(#)?(0)?(\d+)?([,_])?(?:\.(\d+))?([bcdeEfFgGnosxX%])?$/.exec(spec);
  if (!m) throw err('ValueError', `Invalid format specifier '${spec}'`);
  const [, fill, align, signc, alt, zero, w, group, p, typ] = m;
  const prec = p !== undefined ? parseInt(p, 10) : undefined;
  let s: string;
  const signed = (neg: boolean, body: string) => (neg ? '-' : signc === '+' ? '+' : signc === ' ' ? ' ' : '') + body;
  if (typ === 's' || (!typ && typeof v === 'string')) { s = pyStr(v); if (prec !== undefined) s = s.slice(0, prec); }
  else if (typ && 'bdoxXcn'.includes(typ) || (!typ && isIntLike(v) && prec === undefined)) {
    const n = toIntForFormat(v, typ ?? 'd');
    if (typ === 'c') s = String.fromCodePoint(Number(n));
    else {
      const base = typ === 'b' ? 2 : typ === 'o' ? 8 : typ === 'x' || typ === 'X' ? 16 : 10;
      let d = (n < 0n ? -n : n).toString(base);
      if (typ === 'X') d = d.toUpperCase();
      if (group) d = d.replace(/\B(?=(\d{3})+(?!\d))/g, group);
      if (alt && base !== 10) d = (base === 2 ? '0b' : base === 8 ? '0o' : '0x') + d;
      s = signed(n < 0n, d);
    }
  } else if (isNum(v)) {
    const x = flt(v);
    let body = typ ? fmtFloatSpec(Math.abs(x), typ, prec, !!alt) : prec !== undefined ? fmtGeneral(Math.abs(x), prec, !!alt, false) : floatRepr(Math.abs(x));
    if (group) { const [i, f] = body.split('.'); body = i.replace(/\B(?=(\d{3})+(?!\d))/g, group) + (f !== undefined ? `.${f}` : ''); }
    s = signed(x < 0 || Object.is(x, -0), body);
  } else s = pyStr(v);
  const width = w ? parseInt(w, 10) : 0;
  const defLeft = !(isNum(v) && typeof v !== 'string');
  if (zero && !align) return pad(s, width, false, true);
  return pad(s, width, align ? align === '<' : defLeft, false, align, fill ?? ' ');
}

export function strFormat(fmt: string, args: any[], kw: Map<string, any>): string {
  let auto = 0;
  return fmt.replace(/\{\{|\}\}|\{([^{}!:]*)(?:!([rsa]))?(?::([^{}]*))?\}/g, (m, field: string | undefined, conv, spec) => {
    if (m === '{{') return '{';
    if (m === '}}') return '}';
    let v: any;
    const path = (field ?? '').match(/^([^.[]*)(.*)$/)!;
    const head = path[1];
    if (head === '') v = args[auto++];
    else if (/^\d+$/.test(head)) v = args[parseInt(head, 10)];
    else { if (!kw.has(head)) throw err('KeyError', `'${head}'`); v = kw.get(head); }
    if (v === undefined) throw err('IndexError', 'Replacement index out of range for positional args tuple');
    for (const part of path[2].matchAll(/\.([A-Za-z_]\w*)|\[([^\]]+)\]/g)) v = part[1] ? getattr(v, part[1]) : getitem(v, /^\d+$/.test(part[2]) ? BigInt(part[2]) : part[2]);
    if (conv === 'r' || conv === 'a') v = pyRepr(v);
    else if (conv === 's') v = pyStr(v);
    return formatSpec(v, spec ?? '');
  });
}

// ---------------------------------------------------------------------------------------------
// builtin conversions
// ---------------------------------------------------------------------------------------------
export function pyFloat(v: any): number {
  if (typeof v === 'number') return v;
  if (isIntLike(v)) return Number(big(v));
  if (typeof v === 'string') {
    const s = v.trim().toLowerCase();
    if (/^[+-]?(inf|infinity)$/.test(s)) return s.startsWith('-') ? -Infinity : Infinity;
    if (/^[+-]?nan$/.test(s)) return NaN;
    if (/^[+-]?(\d(_?\d)*)?(\.(\d(_?\d)*)?)?([e][+-]?\d(_?\d)*)?$/.test(s) && /\d/.test(s.split('e')[0])) return parseFloat(s.replace(/_/g, ''));
    throw err('ValueError', `could not convert string to float: ${strRepr(v)}`);
  }
  throw err('TypeError', `float() argument must be a string or a real number, not '${pyType(v)}'`);
}
export function pyInt(v: any, base?: any): bigint {
  if (base !== undefined && base !== null) {
    if (typeof v !== 'string') throw err('TypeError', "int() can't convert non-string with explicit base");
    const b = Number(big(base));
    let s = v.trim().toLowerCase().replace(/_/g, '');
    let neg = false;
    if (s.startsWith('-') || s.startsWith('+')) { neg = s[0] === '-'; s = s.slice(1); }
    const pref = { 16: '0x', 8: '0o', 2: '0b' } as Record<number, string>;
    if (b === 0) { const k = Object.entries(pref).find(([, p]) => s.startsWith(p)); return pyInt((neg ? '-' : '') + (k ? s.slice(2) : s), k ? BigInt(k[0]) : 10n); }
    if (pref[b] && s.startsWith(pref[b])) s = s.slice(2);
    const digits = '0123456789abcdefghijklmnopqrstuvwxyz'.slice(0, b);
    if (!s || [...s].some((c) => !digits.includes(c))) throw err('ValueError', `invalid literal for int() with base ${b}: ${strRepr(v)}`);
    let r = 0n;
    for (const c of s) r = r * BigInt(b) + BigInt(digits.indexOf(c));
    return neg ? -r : r;
  }
  if (isIntLike(v)) return big(v);
  if (typeof v === 'number') {
    if (Number.isNaN(v)) throw err('ValueError', 'cannot convert float NaN to integer');
    if (!Number.isFinite(v)) throw err('OverflowError', 'cannot convert float infinity to integer');
    return BigInt(Math.trunc(v));
  }
  if (typeof v === 'string') {
    const s = v.trim();
    if (!/^[+-]?\d(_?\d)*$/.test(s)) throw err('ValueError', `invalid literal for int() with base 10: ${strRepr(v)}`);
    return BigInt(s.replace(/_/g, ''));
  }
  throw err('TypeError', `int() argument must be a string, a bytes-like object or a real number, not '${pyType(v)}'`);
}

/** Python round(): banker's rounding; with ndigits returns the same kind as the input. */
export function pyRound(x: any, nd?: any): any {
  if (nd === undefined || nd === null) {
    if (isIntLike(x)) return big(x);
    const v = flt(x);
    if (!Number.isFinite(v)) throw err(Number.isNaN(v) ? 'ValueError' : 'OverflowError', 'cannot convert float to integer');
    const f = Math.floor(v), d = v - f;
    const r = d > 0.5 ? f + 1 : d < 0.5 ? f : (f % 2 === 0 ? f : f + 1);
    return BigInt(r);
  }
  const n = Number(big(nd));
  if (isIntLike(x)) {
    if (n >= 0) return big(x);
    const p = 10n ** BigInt(-n), v = big(x), q = floorDivBig(v, p), r = v - q * p, h = p / 2n;
    return (r > h || (r === h && q % 2n !== 0n) ? q + 1n : q) * p;
  }
  const v = flt(x);
  if (!Number.isFinite(v) || v === 0) return v;
  if (n > 20 || Math.abs(v) >= 1e21) return v;
  if (n < 0) { const p = 10 ** -n; return Number(pyRound(v / p)) * p; }
  const exact = Math.abs(v).toFixed(Math.min(100, n + 40));
  const [ip, fp = ''] = exact.split('.');
  const kept = fp.slice(0, n), rest = fp.slice(n);
  let digits = BigInt(ip + kept);
  if (rest[0] > '5' || (rest[0] === '5' && (/[1-9]/.test(rest.slice(1)) || digits % 2n === 1n))) digits += 1n;
  const res = Number(digits) / 10 ** n;
  return v < 0 ? -res : res;
}

// ---------------------------------------------------------------------------------------------
// attribute and item access
// ---------------------------------------------------------------------------------------------
function argN(args: any[], i: number, def?: any) { return args.length > i ? args[i] : def; }

function strMethod(s: string, name: string): PyFunc | null {
  const f = (fn: PyCall) => pyfn(`str.${name}`, fn);
  const ws = (chars: any) => (chars === undefined || chars === null ? null : String(chars));
  const esc = (c: string) => c.replace(/[.*+?^${}()|[\]\\]/g, '\\$&');
  switch (name) {
    case 'upper': return f(() => s.toUpperCase());
    case 'lower': return f(() => s.toLowerCase());
    case 'capitalize': return f(() => s.charAt(0).toUpperCase() + s.slice(1).toLowerCase());
    case 'title': return f(() => s.replace(/[A-Za-z]+/g, (w) => w[0].toUpperCase() + w.slice(1).toLowerCase()));
    case 'swapcase': return f(() => [...s].map((c) => (c === c.toUpperCase() ? c.toLowerCase() : c.toUpperCase())).join(''));
    case 'strip': case 'lstrip': case 'rstrip': return f((a) => {
      const c = ws(a[0]); const cls = c === null ? '\\s' : `[${esc(c)}]`;
      let r = s; if (name !== 'rstrip') r = r.replace(new RegExp(`^${c === null ? '\\s' : cls}+`), ''); if (name !== 'lstrip') r = r.replace(new RegExp(`${c === null ? '\\s' : cls}+$`), ''); return r;
    });
    case 'split': case 'rsplit': return f((a, kw) => {
      const sep = argN(a, 0, kw.get('sep') ?? null); const max = Number(big(argN(a, 1, kw.get('maxsplit') ?? -1n)));
      if (sep === null) {
        const parts = s.trim().split(/\s+/).filter(Boolean);
        if (max < 0 || parts.length <= max + 1) return parts;
        return name === 'split' ? [...parts.slice(0, max), s.trim().split(/\s+/).slice(max).join(' ')] : [s.trim().split(/\s+/).slice(0, parts.length - max).join(' '), ...parts.slice(parts.length - max)];
      }
      if (sep === '') throw err('ValueError', 'empty separator');
      const parts = s.split(sep);
      if (max < 0 || parts.length <= max + 1) return parts;
      return name === 'split' ? [...parts.slice(0, max), parts.slice(max).join(sep)] : [parts.slice(0, parts.length - max).join(sep), ...parts.slice(parts.length - max)];
    });
    case 'splitlines': return f(() => s.split(/\r\n|\r|\n/).filter((x, i, arr) => i < arr.length - 1 || x !== ''));
    case 'join': return f((a) => iterate(a[0]).map((x) => { if (typeof x !== 'string') throw err('TypeError', `sequence item: expected str instance, ${pyType(x)} found`); return x; }).join(s));
    case 'replace': return f((a) => { const c = a[2] !== undefined ? Number(big(a[2])) : -1; let r = s, k = 0, out = ''; const o = String(a[0]), n = String(a[1]); if (c < 0) return s.split(o).join(n); while (k < c) { const i = r.indexOf(o); if (i < 0) break; out += r.slice(0, i) + n; r = r.slice(i + o.length); k++; } return out + r; });
    case 'startswith': case 'endswith': return f((a) => { const pre = a[0] instanceof PyTuple ? a[0].items : [a[0]]; return pre.some((p: string) => (name === 'startswith' ? s.startsWith(p) : s.endsWith(p))); });
    case 'find': case 'index': case 'rfind': case 'rindex': return f((a) => { const i = name.startsWith('r') ? s.lastIndexOf(String(a[0])) : s.indexOf(String(a[0])); if (i < 0 && name.endsWith('index')) throw err('ValueError', 'substring not found'); return BigInt(i); });
    case 'count': return f((a) => BigInt(String(a[0]) === '' ? s.length + 1 : s.split(String(a[0])).length - 1));
    case 'format': return f((a, kw) => strFormat(s, a, kw));
    case 'zfill': return f((a) => pad(s, Number(big(a[0])), false, true));
    case 'ljust': case 'rjust': case 'center': return f((a) => pad(s, Number(big(a[0])), name === 'ljust', false, name === 'center' ? '^' : undefined, a[1] ?? ' '));
    case 'isdigit': case 'isnumeric': case 'isdecimal': return f(() => /^\d+$/.test(s));
    case 'isalpha': return f(() => /^[A-Za-z]+$/.test(s));
    case 'isalnum': return f(() => /^[A-Za-z0-9]+$/.test(s));
    case 'isspace': return f(() => /^\s+$/.test(s));
    case 'isupper': return f(() => /[A-Z]/.test(s) && s === s.toUpperCase());
    case 'islower': return f(() => /[a-z]/.test(s) && s === s.toLowerCase());
    case 'partition': case 'rpartition': return f((a) => { const sep = String(a[0]); const i = name === 'partition' ? s.indexOf(sep) : s.lastIndexOf(sep); if (i < 0) return name === 'partition' ? new PyTuple([s, '', '']) : new PyTuple(['', '', s]); return new PyTuple([s.slice(0, i), sep, s.slice(i + sep.length)]); });
  }
  return null;
}
function listMethod(l: any[], name: string): PyFunc | null {
  switch (name) {
    case 'index': return pyfn('list.index', (a) => { const i = l.findIndex((x) => pyEq(x, a[0])); if (i < 0) throw err('ValueError', `${pyRepr(a[0])} is not in list`); return BigInt(i); });
    case 'count': return pyfn('list.count', (a) => BigInt(l.filter((x) => pyEq(x, a[0])).length));
    case 'copy': return pyfn('list.copy', () => [...l]);
    case 'append': return pyfn('list.append', (a) => { l.push(a[0]); return null; });
    case 'extend': return pyfn('list.extend', (a) => { l.push(...iterate(a[0])); return null; });
    case 'pop': return pyfn('list.pop', (a) => { const i = a.length ? Number(big(a[0])) : l.length - 1; const k = i < 0 ? l.length + i : i; if (k < 0 || k >= l.length) throw err('IndexError', 'pop index out of range'); return l.splice(k, 1)[0]; });
  }
  return null;
}
function dictMethod(d: PyDict, name: string): PyFunc | null {
  switch (name) {
    case 'get': return pyfn('dict.get', (a) => { const v = d.tryGet(a[0]); return v === undefined ? (a[1] ?? null) : v; });
    case 'keys': return pyfn('dict.keys', () => d.keys());
    case 'values': return pyfn('dict.values', () => d.values());
    case 'items': return pyfn('dict.items', () => d.entries().map(([k, v]) => new PyTuple([k, v])));
    case 'copy': return pyfn('dict.copy', () => new PyDict(d.entries(), d.dotted));
  }
  return null;
}

export function getattr(obj: any, name: string): any {
  if (name.startsWith('__')) throw err('NameError', `Use of invalid name(s): ${name}`);
  if (typeof obj === 'string') { const m = strMethod(obj, name); if (m) return m; }
  else if (Array.isArray(obj)) { const m = listMethod(obj, name); if (m) return m; }
  else if (obj instanceof PyTuple) {
    if (name === 'index' || name === 'count') return listMethod(obj.items, name)!;
  } else if (typeof obj === 'number' || typeof obj === 'bigint' || typeof obj === 'boolean') {
    if (name === 'real') return obj;
    if (name === 'imag') return typeof obj === 'number' ? 0 : 0n;
    if (name === 'is_integer' && typeof obj === 'number') return pyfn('float.is_integer', () => Number.isInteger(obj));
    if (name === 'conjugate') return pyfn('conjugate', () => obj);
  } else if (obj && typeof obj.getAttr === 'function') return obj.getAttr(name);
  throw err('AttributeError', `'${pyType(obj)}' object has no attribute '${name}'`);
}

function normIndex(i: any, n: number): number {
  if (!isIntLike(i)) throw err('TypeError', `indices must be integers, not ${pyType(i)}`);
  let k = Number(big(i));
  if (k < 0) k += n;
  if (k < 0 || k >= n) throw err('IndexError', 'index out of range');
  return k;
}
export class PySlice { constructor(readonly start: any, readonly stop: any, readonly step: any) {} }
function sliceSeq<T>(seq: T[], s: PySlice): T[] {
  const n = seq.length;
  const step = s.step === null ? 1 : Number(big(s.step));
  if (step === 0) throw err('ValueError', 'slice step cannot be zero');
  const clamp = (v: any, def: number, lo: number, hi: number) => { if (v === null) return def; let k = Number(big(v)); if (k < 0) k += n; return Math.max(lo, Math.min(hi, k)); };
  const start = clamp(s.start, step > 0 ? 0 : n - 1, step > 0 ? 0 : -1, step > 0 ? n : n - 1);
  const stop = clamp(s.stop, step > 0 ? n : -1, step > 0 ? 0 : -1, step > 0 ? n : n - 1);
  const r: T[] = [];
  if (step > 0) for (let i = start; i < stop; i += step) r.push(seq[i]);
  else for (let i = start; i > stop; i += step) r.push(seq[i]);
  return r;
}
export function getitem(obj: any, key: any): any {
  if (typeof obj === 'string') { const cs = [...obj]; return key instanceof PySlice ? sliceSeq(cs, key).join('') : cs[normIndex(key, cs.length)]; }
  if (Array.isArray(obj)) return key instanceof PySlice ? sliceSeq(obj, key) : obj[normIndex(key, obj.length)];
  if (obj instanceof PyTuple) return key instanceof PySlice ? new PyTuple(sliceSeq(obj.items, key)) : obj.items[normIndex(key, obj.items.length)];
  if (obj instanceof PyRange) { const it = obj.items(); return key instanceof PySlice ? sliceSeq(it, key) : it[normIndex(key, it.length)]; }
  if (obj instanceof PyDict) return obj.get(key);
  if (obj && typeof obj.getItem === 'function') return obj.getItem(key);
  throw err('TypeError', `'${pyType(obj)}' object is not subscriptable`);
}

// ---------------------------------------------------------------------------------------------
// builtins
// ---------------------------------------------------------------------------------------------
function minmax(name: 'min' | 'max', args: any[], kw: Map<string, any>) {
  const items = args.length === 1 ? iterate(args[0]) : args;
  const key = kw.get('key');
  if (!items.length) { if (kw.has('default')) return kw.get('default'); throw err('ValueError', `${name}() arg is an empty sequence`); }
  let best = items[0], bk = key ? callValue(key, [best]) : best;
  for (const x of items.slice(1)) { const k = key ? callValue(key, [x]) : x; if (name === 'min' ? pyCmp(k, bk, '<') < 0 : pyCmp(k, bk, '>') > 0) { best = x; bk = k; } }
  return best;
}
function sortedImpl(args: any[], kw: Map<string, any>) {
  const items = [...iterate(args[0])];
  const key = kw.get('key');
  const keyed = items.map((x) => [key && key !== null ? callValue(key, [x]) : x, x] as [any, any]);
  keyed.sort((a, b) => pyCmp(a[0], b[0], '<'));
  if (truthy(kw.get('reverse') ?? false)) keyed.reverse();
  return keyed.map((k) => k[1]);
}
function rangeImpl(args: any[]) {
  const n = args.map((a) => { if (!isIntLike(a)) throw err('TypeError', `'${pyType(a)}' object cannot be interpreted as an integer`); return big(a); });
  if (n.length === 1) return new PyRange(0n, n[0], 1n);
  if (n[2] === 0n) throw err('ValueError', 'range() arg 3 must not be zero');
  return new PyRange(n[0], n[1], n[2] ?? 1n);
}
function dictImpl(args: any[], kw: Map<string, any>) {
  const d = new PyDict();
  if (args.length) {
    const a = args[0];
    if (a instanceof PyDict) for (const [k, v] of a.entries()) d.set(k, v);
    else for (const p of iterate(a)) { const kv = iterate(p); if (kv.length !== 2) throw err('ValueError', 'dictionary update sequence element has wrong length'); d.set(kv[0], kv[1]); }
  }
  for (const [k, v] of kw) d.set(k, v);
  return d;
}
export function callValue(f: any, args: any[], kw: Map<string, any> = new Map()): any {
  if (f instanceof PyFunc) return f.fn(args, kw);
  throw err('TypeError', `'${pyType(f)}' object is not callable`);
}

const B: Record<string, PyCall> = {
  list: (a) => (a.length ? [...iterate(a[0])] : []),
  tuple: (a) => new PyTuple(a.length ? [...iterate(a[0])] : []),
  dict: dictImpl,
  map: (a) => { const f = a[0]; const its = a.slice(1).map(iterate); const n = Math.min(...its.map((x) => x.length)); return Array.from({ length: n }, (_, i) => callValue(f, its.map((x) => x[i]))); },
  filter: (a) => iterate(a[1]).filter((x) => (a[0] === null ? truthy(x) : truthy(callValue(a[0], [x])))),
  len: (a) => { const v = a[0]; if (typeof v === 'string') return BigInt([...v].length); if (v instanceof PyDict) return BigInt(v.size); if (v && typeof v === 'object' && typeof v.size === 'number') return BigInt(v.size); return BigInt(iterate(v).length); },
  str: (a) => (a.length ? pyStr(a[0]) : ''),
  float: (a) => (a.length ? pyFloat(a[0]) : 0),
  int: (a, kw) => (a.length ? pyInt(a[0], a[1] ?? kw.get('base')) : 0n),
  bool: (a) => (a.length ? truthy(a[0]) : false),
  min: (a, kw) => minmax('min', a, kw),
  max: (a, kw) => minmax('max', a, kw),
  round: (a, kw) => pyRound(a[0], a[1] ?? kw.get('ndigits')),
  sorted: sortedImpl,
  range: rangeImpl,
  abs: (a) => { const v = a[0]; if (isIntLike(v)) { const b = big(v); return b < 0n ? -b : b; } if (typeof v === 'number') return Math.abs(v); throw err('TypeError', `bad operand type for abs(): '${pyType(v)}'`); },
  all: (a) => iterate(a[0]).every(truthy),
  any: (a) => iterate(a[0]).some(truthy),
  divmod: (a) => new PyTuple([binop('//', a[0], a[1]), binop('%', a[0], a[1])]),
  enumerate: (a, kw) => { const s = big(a[1] ?? kw.get('start') ?? 0n); return iterate(a[0]).map((x, i) => new PyTuple([s + BigInt(i), x])); },
  ord: (a) => BigInt(String(a[0]).codePointAt(0) ?? 0),
  chr: (a) => String.fromCodePoint(Number(big(a[0]))),
  repr: (a) => pyRepr(a[0]),
  reversed: (a) => [...iterate(a[0])].reverse(),
  sum: (a, kw) => iterate(a[0]).reduce((acc, x) => binop('+', acc, x), a[1] ?? kw.get('start') ?? 0n),
  zip: (a) => { const its = a.map(iterate); const n = its.length ? Math.min(...its.map((x) => x.length)) : 0; return Array.from({ length: n }, (_, i) => new PyTuple(its.map((x) => x[i]))); },
  set: (a) => { const out: any[] = []; for (const x of a.length ? iterate(a[0]) : []) if (!out.some((y) => pyEq(x, y))) out.push(x); return out; },
  isinstance: (a) => { const t = a[1] instanceof PyTuple ? a[1].items : [a[1]]; return t.some((f: any) => f instanceof PyFunc && (pyType(a[0]) === f.name || (f.name === 'int' && pyType(a[0]) === 'bool'))); },
  type: (a) => pyfn(pyType(a[0]), (b) => b[0]),
};
export const PY_BUILTINS: Record<string, PyFunc> = Object.fromEntries(Object.entries(B).map(([k, f]) => [k, pyfn(k, f)]));

const num1 = (name: string, f: (x: number) => number) => pyfn(name, (a) => { const v = a[0]; if (!isNum(v)) throw err('TypeError', `must be real number, not ${pyType(v)}`); const r = f(flt(v)); if (Number.isNaN(r) && !Number.isNaN(flt(v))) throw err('ValueError', 'math domain error'); return r; });
const toIntChecked = (x: number) => { if (!Number.isFinite(x)) throw err(Number.isNaN(x) ? 'ValueError' : 'OverflowError', 'cannot convert float to integer'); return BigInt(x); };
const intRes = (name: string, f: (x: number) => number) => pyfn(name, (a) => (isIntLike(a[0]) ? big(a[0]) : toIntChecked(f(flt(a[0])))));
export const PY_MATH: Record<string, any> = {
  pi: Math.PI, e: Math.E, tau: 2 * Math.PI, inf: Infinity, nan: NaN,
  sin: num1('sin', Math.sin), cos: num1('cos', Math.cos), tan: num1('tan', Math.tan),
  asin: num1('asin', Math.asin), acos: num1('acos', Math.acos), atan: num1('atan', Math.atan),
  sinh: num1('sinh', Math.sinh), cosh: num1('cosh', Math.cosh), tanh: num1('tanh', Math.tanh),
  asinh: num1('asinh', Math.asinh), acosh: num1('acosh', Math.acosh), atanh: num1('atanh', Math.atanh),
  sqrt: num1('sqrt', Math.sqrt), exp: num1('exp', Math.exp), expm1: num1('expm1', Math.expm1),
  log10: num1('log10', Math.log10), log2: num1('log2', Math.log2), log1p: num1('log1p', Math.log1p),
  fabs: num1('fabs', Math.abs), degrees: num1('degrees', (x) => (x * 180) / Math.PI), radians: num1('radians', (x) => (x * Math.PI) / 180),
  floor: intRes('floor', Math.floor), ceil: intRes('ceil', Math.ceil), trunc: intRes('trunc', Math.trunc),
  log: pyfn('log', (a) => { const x = flt(a[0]); if (x <= 0) throw err('ValueError', 'math domain error'); return a.length > 1 ? Math.log(x) / Math.log(flt(a[1])) : Math.log(x); }),
  pow: pyfn('pow', (a) => flt(a[0]) ** flt(a[1])),
  atan2: pyfn('atan2', (a) => Math.atan2(flt(a[0]), flt(a[1]))),
  hypot: pyfn('hypot', (a) => Math.hypot(...a.map(flt))),
  copysign: pyfn('copysign', (a) => { const x = Math.abs(flt(a[0])), y = flt(a[1]); return y < 0 || Object.is(y, -0) ? -x : x; }),
  fmod: pyfn('fmod', (a) => { const y = flt(a[1]); if (y === 0) throw err('ValueError', 'math domain error'); return flt(a[0]) % y; }),
  isclose: pyfn('isclose', (a, kw) => { const x = flt(a[0]), y = flt(a[1]); const rt = flt(kw.get('rel_tol') ?? 1e-9), at = flt(kw.get('abs_tol') ?? 0); return x === y || Math.abs(x - y) <= Math.max(rt * Math.max(Math.abs(x), Math.abs(y)), at); }),
  isfinite: pyfn('isfinite', (a) => Number.isFinite(flt(a[0]))), isinf: pyfn('isinf', (a) => { const x = flt(a[0]); return !Number.isFinite(x) && !Number.isNaN(x); }), isnan: pyfn('isnan', (a) => Number.isNaN(flt(a[0]))),
  factorial: pyfn('factorial', (a) => { const n = pyInt(a[0]); if (n < 0n) throw err('ValueError', 'factorial() not defined for negative values'); let r = 1n; for (let i = 2n; i <= n; i++) r *= i; return r; }),
  gcd: pyfn('gcd', (a) => a.map((x) => pyInt(x)).reduce((x, y) => { x = x < 0n ? -x : x; y = y < 0n ? -y : y; while (y) [x, y] = [y, x % y]; return x; }, 0n)),
  prod: pyfn('prod', (a) => iterate(a[0]).reduce((acc, x) => binop('*', acc, x), 1n)),
  fsum: pyfn('fsum', (a) => iterate(a[0]).reduce((acc: number, x) => acc + flt(x), 0)),
  modf: pyfn('modf', (a) => { const x = flt(a[0]); const i = Math.trunc(x); return new PyTuple([x - i, i]); }),
  ldexp: pyfn('ldexp', (a) => flt(a[0]) * 2 ** Number(big(a[1]))),
  dist: pyfn('dist', (a) => Math.hypot(...iterate(a[0]).map((x, i) => flt(x) - flt(iterate(a[1])[i])))),
};

/** Namespace object (python., math., xacro. ...) giving dotted access to a fixed symbol set. */
export class PyNamespace implements PyObj {
  constructor(readonly ns: string, readonly symbols: Record<string, any>) {}
  get typeName() { return 'NameSpace'; }
  getAttr(name: string) { if (Object.prototype.hasOwnProperty.call(this.symbols, name)) return this.symbols[name]; throw err('NameError', `name '${name}' is not defined`); }
  getItem(key: any) { return this.getAttr(String(key)); }
  has(key: any) { return Object.prototype.hasOwnProperty.call(this.symbols, String(key)); }
}

// ---------------------------------------------------------------------------------------------
// tokenizer
// ---------------------------------------------------------------------------------------------
type Tok = { t: 'num' | 'str' | 'name' | 'op' | 'end'; v: any; pos: number };
const KEYWORDS = new Set(['and', 'or', 'not', 'in', 'is', 'if', 'else', 'lambda', 'for', 'None', 'True', 'False']);
const OPS3 = ['**=', '//=', '...', '<<=', '>>='];
const OPS2 = ['**', '//', '==', '!=', '<=', '>=', '<<', '>>', '->', ':=', '+=', '-=', '*=', '/=', '<>'];

function tokenize(src: string): Tok[] {
  const toks: Tok[] = [];
  let i = 0;
  const n = src.length;
  while (i < n) {
    const c = src[i];
    if (/\s/.test(c)) { i++; continue; }
    if (c === '#') break;
    const strStart = /^([rRbBuUfF]{0,2})(['"])/.exec(src.slice(i, i + 4));
    if (strStart && (strStart[1] === '' || /^[A-Za-z]+$/.test(strStart[1])) && (strStart[1] === '' || !/[A-Za-z0-9_]/.test(src[i - 1] ?? ''))) {
      const prefix = strStart[1].toLowerCase();
      if (prefix.includes('f')) throw err('SyntaxError', 'f-strings are not supported');
      if (prefix.includes('b')) throw err('SyntaxError', 'bytes literals are not supported');
      const raw = prefix.includes('r');
      let j = i + strStart[1].length;
      const q = src[j];
      const triple = src.startsWith(q.repeat(3), j);
      const close = triple ? q.repeat(3) : q;
      j += close.length;
      let out = '';
      for (;;) {
        if (j >= n) throw err('SyntaxError', 'unterminated string literal');
        if (src.startsWith(close, j)) { j += close.length; break; }
        const ch = src[j];
        if (!triple && ch === '\n') throw err('SyntaxError', 'unterminated string literal');
        if (ch === '\\' && j + 1 < n) {
          const e = src[j + 1];
          if (raw) { out += ch + e; j += 2; continue; }
          const simple: Record<string, string> = { n: '\n', t: '\t', r: '\r', '\\': '\\', "'": "'", '"': '"', a: '\x07', b: '\b', f: '\f', v: '\v', '0': '\0', '\n': '' };
          if (e === 'x') { out += String.fromCharCode(parseInt(src.slice(j + 2, j + 4), 16)); j += 4; continue; }
          if (e === 'u') { out += String.fromCharCode(parseInt(src.slice(j + 2, j + 6), 16)); j += 6; continue; }
          if (e === 'U') { out += String.fromCodePoint(parseInt(src.slice(j + 2, j + 10), 16)); j += 10; continue; }
          if (/[0-7]/.test(e)) { const m = /^[0-7]{1,3}/.exec(src.slice(j + 1))![0]; out += String.fromCharCode(parseInt(m, 8)); j += 1 + m.length; continue; }
          if (e in simple) { out += simple[e]; j += 2; continue; }
          out += ch + e; j += 2; continue;
        }
        out += ch; j++;
      }
      toks.push({ t: 'str', v: out, pos: i });
      i = j;
      continue;
    }
    if (/[0-9]/.test(c) || (c === '.' && /[0-9]/.test(src[i + 1] ?? ''))) {
      const m = /^(0[xX](_?[0-9a-fA-F])+|0[oO](_?[0-7])+|0[bB](_?[01])+|((\d(_?\d)*)?\.(\d(_?\d)*)?([eE][+-]?\d(_?\d)*)?|\d(_?\d)*[eE][+-]?\d(_?\d)*|\d(_?\d)*))([jJ])?/.exec(src.slice(i))!;
      const text = m[0];
      if (m[15]) throw err('SyntaxError', 'complex literals are not supported');
      const clean = text.replace(/_/g, '');
      let v: any;
      if (/^0[xob]/i.test(clean)) v = BigInt(clean.toLowerCase());
      else if (/[.eE]/.test(clean)) v = parseFloat(clean);
      else {
        if (/^0\d/.test(clean) && /[1-9]/.test(clean)) throw err('SyntaxError', 'leading zeros in decimal integer literals are not permitted');
        v = BigInt(clean);
      }
      toks.push({ t: 'num', v, pos: i });
      i += text.length;
      if (/[A-Za-z_]/.test(src[i] ?? '')) throw err('SyntaxError', 'invalid decimal literal');
      continue;
    }
    if (/[A-Za-z_À-￿]/.test(c)) {
      let j = i + 1;
      while (j < n && /[A-Za-z0-9_À-￿]/.test(src[j])) j++;
      toks.push({ t: 'name', v: src.slice(i, j), pos: i });
      i = j;
      continue;
    }
    const three = src.slice(i, i + 3), two = src.slice(i, i + 2);
    if (OPS3.includes(three)) { toks.push({ t: 'op', v: three, pos: i }); i += 3; continue; }
    if (OPS2.includes(two)) { toks.push({ t: 'op', v: two, pos: i }); i += 2; continue; }
    if ('+-*/%<>()[]{},:.~&|^=@;!'.includes(c)) { toks.push({ t: 'op', v: c, pos: i }); i++; continue; }
    throw err('SyntaxError', `invalid character '${c}'`);
  }
  toks.push({ t: 'end', v: null, pos: n });
  return toks;
}

// ---------------------------------------------------------------------------------------------
// parser -> AST
// ---------------------------------------------------------------------------------------------
type Node =
  | { k: 'const'; v: any } | { k: 'name'; id: string } | { k: 'bin'; op: string; a: Node; b: Node }
  | { k: 'cmp'; first: Node; ops: string[]; rest: Node[] } | { k: 'bool'; op: 'and' | 'or'; a: Node; b: Node }
  | { k: 'not'; a: Node } | { k: 'unary'; op: string; a: Node } | { k: 'ifexp'; c: Node; a: Node; b: Node }
  | { k: 'attr'; o: Node; name: string } | { k: 'sub'; o: Node; key: Node } | { k: 'slice'; a: Node | null; b: Node | null; c: Node | null }
  | { k: 'call'; f: Node; args: Arg[] } | { k: 'list'; items: Node[] } | { k: 'tuple'; items: Node[] } | { k: 'dict'; items: [Node, Node][] }
  | { k: 'comp'; kind: 'list' | 'gen' | 'dict' | 'set'; elt: Node; val?: Node; gens: { target: Target; iter: Node; ifs: Node[] }[] }
  | { k: 'lambda'; params: string[]; defaults: (Node | null)[]; body: Node } | { k: 'starred'; a: Node };
type Arg = { kind: 'pos' | 'kw' | 'star' | 'dstar'; name?: string; v: Node };
type Target = string | Target[];

class Parser {
  i = 0;
  constructor(readonly toks: Tok[]) {}
  peek(o = 0) { return this.toks[this.i + o]; }
  isOp(v: string, o = 0) { const t = this.peek(o); return t.t === 'op' && t.v === v; }
  isKw(v: string, o = 0) { const t = this.peek(o); return t.t === 'name' && t.v === v; }
  eat(v: string) { if (!this.isOp(v)) throw err('SyntaxError', `expected '${v}'${this.peek().t === 'end' ? ' (unexpected EOF)' : ` near '${this.peek().v}'`}`); this.i++; }
  eatKw(v: string) { if (!this.isKw(v)) throw err('SyntaxError', `expected '${v}'`); this.i++; }

  parseTop(): Node {
    const e = this.testlist();
    if (this.peek().t !== 'end') throw err('SyntaxError', `invalid syntax near '${this.peek().v}'`);
    return e;
  }
  testlist(): Node {
    const first = this.testOrStar();
    if (!this.isOp(',')) return first;
    const items = [first];
    while (this.isOp(',')) { this.i++; if (this.atExprEnd()) break; items.push(this.testOrStar()); }
    return { k: 'tuple', items };
  }
  atExprEnd() { const t = this.peek(); return t.t === 'end' || (t.t === 'op' && [')', ']', '}', ':', '='].includes(t.v)) || (t.t === 'name' && (t.v === 'for' || t.v === 'in' || t.v === 'if')); }
  testOrStar(): Node { if (this.isOp('*')) { this.i++; return { k: 'starred', a: this.orTest() }; } return this.test(); }
  test(): Node {
    if (this.isKw('lambda')) return this.lambda();
    const a = this.orTest();
    if (this.isKw('if') ) {
      this.i++;
      const c = this.orTest();
      this.eatKw('else');
      const b = this.test();
      return { k: 'ifexp', c, a, b };
    }
    return a;
  }
  testNoCond(): Node { return this.isKw('lambda') ? this.lambda() : this.orTest(); }
  lambda(): Node {
    this.eatKw('lambda');
    const params: string[] = [], defaults: (Node | null)[] = [];
    while (!this.isOp(':')) {
      const t = this.peek();
      if (t.t !== 'name' || KEYWORDS.has(t.v)) throw err('SyntaxError', 'invalid lambda parameter');
      this.i++;
      params.push(t.v);
      if (this.isOp('=')) { this.i++; defaults.push(this.test()); } else defaults.push(null);
      if (this.isOp(',')) this.i++; else break;
    }
    this.eat(':');
    return { k: 'lambda', params, defaults, body: this.test() };
  }
  orTest(): Node { let a = this.andTest(); while (this.isKw('or')) { this.i++; a = { k: 'bool', op: 'or', a, b: this.andTest() }; } return a; }
  andTest(): Node { let a = this.notTest(); while (this.isKw('and')) { this.i++; a = { k: 'bool', op: 'and', a, b: this.notTest() }; } return a; }
  notTest(): Node { if (this.isKw('not')) { this.i++; return { k: 'not', a: this.notTest() }; } return this.comparison(); }
  comparison(): Node {
    const first = this.bitor();
    const ops: string[] = [], rest: Node[] = [];
    for (;;) {
      const t = this.peek();
      let op: string | null = null;
      if (t.t === 'op' && ['<', '>', '==', '>=', '<=', '!=', '<>'].includes(t.v)) { op = t.v === '<>' ? '!=' : t.v; this.i++; }
      else if (this.isKw('in')) { op = 'in'; this.i++; }
      else if (this.isKw('not') && this.isKw('in', 1)) { op = 'not in'; this.i += 2; }
      else if (this.isKw('is')) { this.i++; if (this.isKw('not')) { this.i++; op = 'is not'; } else op = 'is'; }
      if (!op) break;
      ops.push(op);
      rest.push(this.bitor());
    }
    return ops.length ? { k: 'cmp', first, ops, rest } : first;
  }
  binLevel(next: () => Node, ops: string[]): Node {
    let a = next();
    while (this.peek().t === 'op' && ops.includes(this.peek().v)) { const op = this.peek().v; this.i++; a = { k: 'bin', op, a, b: next() }; }
    return a;
  }
  bitor(): Node { return this.binLevel(() => this.bitxor(), ['|']); }
  bitxor(): Node { return this.binLevel(() => this.bitand(), ['^']); }
  bitand(): Node { return this.binLevel(() => this.shift(), ['&']); }
  shift(): Node { return this.binLevel(() => this.arith(), ['<<', '>>']); }
  arith(): Node { return this.binLevel(() => this.term(), ['+', '-']); }
  term(): Node { return this.binLevel(() => this.factor(), ['*', '/', '//', '%', '@']); }
  factor(): Node {
    const t = this.peek();
    if (t.t === 'op' && (t.v === '-' || t.v === '+' || t.v === '~')) { this.i++; return { k: 'unary', op: t.v, a: this.factor() }; }
    return this.power();
  }
  power(): Node {
    const a = this.primary();
    if (this.isOp('**')) { this.i++; return { k: 'bin', op: '**', a, b: this.factor() }; }
    return a;
  }
  primary(): Node {
    let a = this.atom();
    for (;;) {
      if (this.isOp('.')) {
        this.i++;
        const t = this.peek();
        if (t.t !== 'name') throw err('SyntaxError', 'invalid attribute');
        this.i++;
        a = { k: 'attr', o: a, name: t.v };
      } else if (this.isOp('(')) {
        this.i++;
        a = { k: 'call', f: a, args: this.callArgs() };
      } else if (this.isOp('[')) {
        this.i++;
        const subs: Node[] = [this.subscript()];
        while (this.isOp(',')) { this.i++; if (this.isOp(']')) break; subs.push(this.subscript()); }
        this.eat(']');
        a = { k: 'sub', o: a, key: subs.length === 1 ? subs[0] : { k: 'tuple', items: subs } };
      } else break;
    }
    return a;
  }
  subscript(): Node {
    let lo: Node | null = null, hi: Node | null = null, st: Node | null = null;
    if (!this.isOp(':')) { lo = this.test(); if (!this.isOp(':')) return lo; }
    this.eat(':');
    if (!this.isOp(']') && !this.isOp(':') && !this.isOp(',')) hi = this.test();
    if (this.isOp(':')) { this.i++; if (!this.isOp(']') && !this.isOp(',')) st = this.test(); }
    return { k: 'slice', a: lo, b: hi, c: st };
  }
  callArgs(): Arg[] {
    const args: Arg[] = [];
    while (!this.isOp(')')) {
      if (this.isOp('*')) { this.i++; args.push({ kind: 'star', v: this.test() }); }
      else if (this.isOp('**')) { this.i++; args.push({ kind: 'dstar', v: this.test() }); }
      else if (this.peek().t === 'name' && this.isOp('=', 1)) { const name = this.peek().v; this.i += 2; args.push({ kind: 'kw', name, v: this.test() }); }
      else {
        const v = this.test();
        if (this.isKw('for')) { args.push({ kind: 'pos', v: this.comprehension('gen', v) }); break; }
        args.push({ kind: 'pos', v });
      }
      if (this.isOp(',')) this.i++; else break;
    }
    this.eat(')');
    return args;
  }
  target(): Target {
    const one = (): Target => {
      if (this.isOp('(')) { this.i++; const t = this.targetList(); this.eat(')'); return t; }
      const t = this.peek();
      if (t.t !== 'name' || KEYWORDS.has(t.v)) throw err('SyntaxError', 'invalid comprehension target');
      this.i++;
      return t.v;
    };
    const first = one();
    if (!this.isOp(',')) return first;
    const items = [first];
    while (this.isOp(',')) { this.i++; if (this.isKw('in')) break; items.push(one()); }
    return items;
  }
  targetList(): Target { return this.target(); }
  comprehension(kind: 'list' | 'gen' | 'dict' | 'set', elt: Node, val?: Node): Node {
    const gens: { target: Target; iter: Node; ifs: Node[] }[] = [];
    while (this.isKw('for')) {
      this.i++;
      const target = this.target();
      this.eatKw('in');
      const iter = this.orTest();
      const ifs: Node[] = [];
      while (this.isKw('if')) { this.i++; ifs.push(this.testNoCond()); }
      gens.push({ target, iter, ifs });
    }
    return { k: 'comp', kind, elt, val, gens };
  }
  atom(): Node {
    const t = this.peek();
    if (t.t === 'num') { this.i++; return { k: 'const', v: t.v }; }
    if (t.t === 'str') {
      let s = '';
      while (this.peek().t === 'str') { s += this.peek().v; this.i++; }
      return { k: 'const', v: s };
    }
    if (t.t === 'name') {
      if (t.v === 'None') { this.i++; return { k: 'const', v: null }; }
      if (t.v === 'True') { this.i++; return { k: 'const', v: true }; }
      if (t.v === 'False') { this.i++; return { k: 'const', v: false }; }
      if (KEYWORDS.has(t.v)) throw err('SyntaxError', `invalid syntax near '${t.v}'`);
      this.i++;
      return { k: 'name', id: t.v };
    }
    if (t.t === 'op') {
      if (t.v === '(') {
        this.i++;
        if (this.isOp(')')) { this.i++; return { k: 'tuple', items: [] }; }
        const first = this.testOrStar();
        if (this.isKw('for')) { const c = this.comprehension('gen', first); this.eat(')'); return c; }
        if (this.isOp(')')) { this.i++; return first; }
        const items = [first];
        while (this.isOp(',')) { this.i++; if (this.isOp(')')) break; items.push(this.testOrStar()); }
        this.eat(')');
        return { k: 'tuple', items };
      }
      if (t.v === '[') {
        this.i++;
        if (this.isOp(']')) { this.i++; return { k: 'list', items: [] }; }
        const first = this.testOrStar();
        if (this.isKw('for')) { const c = this.comprehension('list', first); this.eat(']'); return c; }
        const items = [first];
        while (this.isOp(',')) { this.i++; if (this.isOp(']')) break; items.push(this.testOrStar()); }
        this.eat(']');
        return { k: 'list', items };
      }
      if (t.v === '{') {
        this.i++;
        if (this.isOp('}')) { this.i++; return { k: 'dict', items: [] }; }
        const k0 = this.test();
        if (this.isOp(':')) {
          this.i++;
          const v0 = this.test();
          if (this.isKw('for')) { const c = this.comprehension('dict', k0, v0); this.eat('}'); return c; }
          const items: [Node, Node][] = [[k0, v0]];
          while (this.isOp(',')) { this.i++; if (this.isOp('}')) break; const k = this.test(); this.eat(':'); items.push([k, this.test()]); }
          this.eat('}');
          return { k: 'dict', items };
        }
        if (this.isKw('for')) { const c = this.comprehension('set', k0); this.eat('}'); return c; }
        const items = [k0];
        while (this.isOp(',')) { this.i++; if (this.isOp('}')) break; items.push(this.test()); }
        this.eat('}');
        return { k: 'call', f: { k: 'const', v: PY_BUILTINS.set }, args: [{ kind: 'pos', v: { k: 'list', items } }] };
      }
    }
    if (t.t === 'end') throw err('SyntaxError', 'unexpected EOF while parsing');
    throw err('SyntaxError', `invalid syntax near '${t.v}'`);
  }
}

const astCache = new Map<string, Node>();
export function parseExpr(src: string): Node {
  const key = src.trim();
  let n = astCache.get(key);
  if (!n) {
    n = new Parser(tokenize(key)).parseTop();
    if (astCache.size > 5000) astCache.clear();
    astCache.set(key, n);
  }
  return n;
}

// ---------------------------------------------------------------------------------------------
// evaluator
// ---------------------------------------------------------------------------------------------
export type Lookup = (name: string) => any;

function bindTarget(t: Target, v: any, scope: Map<string, any>) {
  if (typeof t === 'string') { scope.set(t, v); return; }
  const items = iterate(v);
  if (items.length !== t.length) throw err('ValueError', `cannot unpack ${items.length} values into ${t.length} targets`);
  t.forEach((s, i) => bindTarget(s, items[i], scope));
}

function evalNode(n: Node, look: Lookup): any {
  switch (n.k) {
    case 'const': return n.v;
    case 'name':
      if (n.id.startsWith('__')) throw err('NameError', `Use of invalid name(s): ${n.id}`);
      return look(n.id);
    case 'bin': return binop(n.op, evalNode(n.a, look), evalNode(n.b, look));
    case 'cmp': {
      let left = evalNode(n.first, look);
      for (let i = 0; i < n.ops.length; i++) {
        const right = evalNode(n.rest[i], look);
        if (!binop(n.ops[i], left, right)) return false;
        left = right;
      }
      return true;
    }
    case 'bool': { const a = evalNode(n.a, look); if (n.op === 'and' ? !truthy(a) : truthy(a)) return a; return evalNode(n.b, look); }
    case 'not': return !truthy(evalNode(n.a, look));
    case 'unary': {
      const a = evalNode(n.a, look);
      if (n.op === '-') { if (isIntLike(a)) return -big(a); if (typeof a === 'number') return -a; throw err('TypeError', `bad operand type for unary -: '${pyType(a)}'`); }
      if (n.op === '+') { if (isIntLike(a)) return big(a); if (typeof a === 'number') return a; throw err('TypeError', `bad operand type for unary +: '${pyType(a)}'`); }
      if (isIntLike(a)) return -(big(a) + 1n);
      throw err('TypeError', `bad operand type for unary ~: '${pyType(a)}'`);
    }
    case 'ifexp': return truthy(evalNode(n.c, look)) ? evalNode(n.a, look) : evalNode(n.b, look);
    case 'attr': return getattr(evalNode(n.o, look), n.name);
    case 'sub': return getitem(evalNode(n.o, look), evalNode(n.key, look));
    case 'slice': return new PySlice(n.a ? evalNode(n.a, look) : null, n.b ? evalNode(n.b, look) : null, n.c ? evalNode(n.c, look) : null);
    case 'call': {
      const f = evalNode(n.f, look);
      const args: any[] = [];
      const kw = new Map<string, any>();
      for (const a of n.args) {
        if (a.kind === 'pos') args.push(evalNode(a.v, look));
        else if (a.kind === 'star') args.push(...iterate(evalNode(a.v, look)));
        else if (a.kind === 'kw') kw.set(a.name!, evalNode(a.v, look));
        else { const d = evalNode(a.v, look); if (!(d instanceof PyDict)) throw err('TypeError', 'argument after ** must be a mapping'); for (const [k, v] of d.entries()) kw.set(String(k), v); }
      }
      return callValue(f, args, kw);
    }
    case 'list': return n.items.flatMap((x) => (x.k === 'starred' ? iterate(evalNode(x.a, look)) : [evalNode(x, look)]));
    case 'tuple': return new PyTuple(n.items.flatMap((x) => (x.k === 'starred' ? iterate(evalNode(x.a, look)) : [evalNode(x, look)])));
    case 'starred': throw err('SyntaxError', "can't use starred expression here");
    case 'dict': { const d = new PyDict(); for (const [k, v] of n.items) d.set(evalNode(k, look), evalNode(v, look)); return d; }
    case 'comp': {
      const results: any[] = [];
      const dict = new PyDict();
      const run = (gi: number, scope: Map<string, any>) => {
        const lk: Lookup = (name) => (scope.has(name) ? scope.get(name) : look(name));
        if (gi === n.gens.length) {
          if (n.kind === 'dict') dict.set(evalNode(n.elt, lk), evalNode(n.val!, lk));
          else results.push(evalNode(n.elt, lk));
          return;
        }
        const g = n.gens[gi];
        for (const v of iterate(evalNode(g.iter, lk))) {
          const s = new Map(scope);
          bindTarget(g.target, v, s);
          const lk2: Lookup = (name) => (s.has(name) ? s.get(name) : look(name));
          if (g.ifs.every((c) => truthy(evalNode(c, lk2)))) run(gi + 1, s);
        }
      };
      run(0, new Map());
      if (n.kind === 'dict') return dict;
      if (n.kind === 'set') return PY_BUILTINS.set.fn([results], new Map());
      return results;
    }
    case 'lambda': {
      const defaults = n.defaults.map((d) => (d ? evalNode(d, look) : undefined));
      return pyfn('<lambda>', (args, kw) => {
        const s = new Map<string, any>();
        n.params.forEach((p, i) => {
          const v = i < args.length ? args[i] : kw.has(p) ? kw.get(p) : defaults[i];
          if (v === undefined) throw err('TypeError', `<lambda>() missing required argument: '${p}'`);
          s.set(p, v);
        });
        return evalNode(n.body, (name) => (s.has(name) ? s.get(name) : look(name)));
      });
    }
  }
}

/** Evaluate a Python expression. `lookup` resolves free names (throw PyError NameError when unknown). */
export function pyEval(src: string, lookup: Lookup): any {
  return evalNode(parseExpr(src), lookup);
}

/** Lookup over a plain symbol record (for `$(eval)` and yaml unit tags). */
export function recordLookup(...recs: Record<string, any>[]): Lookup {
  return (name) => {
    for (const r of recs) if (Object.prototype.hasOwnProperty.call(r, name)) return r[name];
    throw err('NameError', `name '${name}' is not defined`);
  };
}
