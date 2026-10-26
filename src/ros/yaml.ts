/**
 * YAML for ROS: a reader for the YAML 1.1 subset PyYAML's SafeLoader accepts in ROS packages (block / flow collections,
 * quoted / plain / block scalars, comments, anchors + aliases + `<<` merges, tags with custom constructors such as
 * xacro's `!degrees`), resolving plain scalars exactly like PyYAML (yes/no/on/off booleans, `1e3` stays a string,
 * octal `010`), and a typed writer for ROS 2 parameter files: doubles always carry a decimal point, arrays are
 * homogeneous, strings that YAML would re-type are quoted. rcl rejects `20` for a double parameter and
 * `[1.5, 0, 1.57]` as a mixed sequence — the writer never produces either.
 */

export type YamlValue = null | boolean | number | bigint | string | YamlValue[] | Map<string | number | boolean | null, YamlValue> | { tag: string; value: unknown };

export interface YamlReadOptions {
  /** Integers as bigint (Python int typing for xacro); default: JS numbers. */
  bigint?: boolean;
  /** Constructors for local tags (`!degrees 90` -> fn('90')). Unknown local tags throw like SafeLoader. */
  tags?: Record<string, (scalar: string) => unknown>;
}

export class YamlError extends Error {}

interface Line { indent: number; text: string; no: number }

const NULL_RE = /^(?:~|null|Null|NULL|)$/;
const BOOL_TRUE = /^(?:yes|Yes|YES|true|True|TRUE|on|On|ON)$/;
const BOOL_FALSE = /^(?:no|No|NO|false|False|FALSE|off|Off|OFF)$/;
const INT_RE = /^(?:[-+]?0b[0-1_]+|[-+]?0[0-7_]+|[-+]?(?:0|[1-9][0-9_]*)|[-+]?0x[0-9a-fA-F_]+|[-+]?[1-9][0-9_]*(?::[0-5]?[0-9])+)$/;
const FLOAT_RE = /^(?:[-+]?(?:[0-9][0-9_]*)\.[0-9_]*(?:[eE][-+][0-9]+)?|\.[0-9_]+(?:[eE][-+][0-9]+)?|[-+]?[0-9][0-9_]*(?::[0-5]?[0-9])+\.[0-9_]*|[-+]?\.(?:inf|Inf|INF)|\.(?:nan|NaN|NAN))$/;

/** PyYAML implicit resolution of a plain scalar. */
export function resolvePlain(s: string, bigint = false): YamlValue {
  if (NULL_RE.test(s)) return null;
  if (BOOL_TRUE.test(s)) return true;
  if (BOOL_FALSE.test(s)) return false;
  if (INT_RE.test(s)) {
    let t = s.replace(/_/g, ''), sign = 1n;
    if (t[0] === '-' || t[0] === '+') { if (t[0] === '-') sign = -1n; t = t.slice(1); }
    let v: bigint;
    if (t.includes(':')) v = t.split(':').reduce((a, d) => a * 60n + BigInt(d), 0n);
    else if (t.startsWith('0b')) v = BigInt(t);
    else if (t.startsWith('0x')) v = BigInt(t);
    else if (t.length > 1 && t[0] === '0') v = BigInt(`0o${t.slice(1)}`);
    else v = BigInt(t);
    v *= sign;
    return bigint ? v : Number(v);
  }
  if (FLOAT_RE.test(s)) {
    const t = s.replace(/_/g, '').toLowerCase();
    if (t.endsWith('.inf')) return t.startsWith('-') ? -Infinity : Infinity;
    if (t.endsWith('.nan')) return NaN;
    if (t.includes(':')) { const sign = t.startsWith('-') ? -1 : 1; return sign * t.replace(/^[-+]/, '').split(':').reduce((a, d) => a * 60 + parseFloat(d), 0); }
    return parseFloat(t);
  }
  return s;
}

function stripComment(s: string): string {
  let q: string | null = null;
  for (let i = 0; i < s.length; i++) {
    const c = s[i];
    if (q) { if (c === q) { if (q === "'" && s[i + 1] === "'") { i++; continue; } q = null; } else if (c === '\\' && q === '"') i++; continue; }
    if ((c === '"' || c === "'") && (i === 0 || /[\s\[{,:-]/.test(s[i - 1]))) { q = c; continue; }
    if (c === '#' && (i === 0 || /\s/.test(s[i - 1]))) return s.slice(0, i).replace(/\s+$/, '');
  }
  return s.replace(/\s+$/, '');
}

/** Index of the `: ` (or trailing `:`) separating a block-mapping key, outside quotes / flow brackets; -1 when none. */
function keySep(s: string): number {
  let q: string | null = null, depth = 0;
  for (let i = 0; i < s.length; i++) {
    const c = s[i];
    if (q) { if (c === q) { if (q === "'" && s[i + 1] === "'") { i++; continue; } q = null; } else if (c === '\\' && q === '"') i++; continue; }
    if (c === '"' || c === "'") { if (i === 0 || /[\s\[{,]/.test(s[i - 1])) q = c; continue; }
    if (c === '[' || c === '{') depth++;
    else if (c === ']' || c === '}') depth--;
    else if (c === ':' && depth === 0 && (i + 1 === s.length || /\s/.test(s[i + 1]))) return i;
  }
  return -1;
}

class Reader {
  lines: Line[] = [];
  i = 0;
  anchors = new Map<string, YamlValue>();
  constructor(src: string, readonly opts: YamlReadOptions) {
    const raw = src.replace(/\r\n?/g, '\n').split('\n');
    let started = false;
    for (let k = 0; k < raw.length; k++) {
      const l = raw[k];
      if (/^(---|\.\.\.)(\s|$)/.test(l)) {
        if (l.startsWith('...')) break;
        if (started) break; // only the first document (safe_load of single docs)
        started = true;
        const rest = l.slice(3).trim();
        if (rest && !rest.startsWith('#')) this.lines.push({ indent: 0, text: rest, no: k + 1 });
        continue;
      }
      if (/^%/.test(l) && !started) continue;
      if (l.includes('\t') && /^\t/.test(l)) throw new YamlError(`line ${k + 1}: tabs are not allowed for indentation`);
      this.lines.push({ indent: l.length - l.trimStart().length, text: l.trimStart(), no: k + 1 });
      if (l.trim()) started = true;
    }
  }
  /** next non-blank, non-comment line index */
  skip(): void { while (this.i < this.lines.length && (!this.lines[this.i].text.trim() || this.lines[this.i].text.trimStart().startsWith('#'))) this.i++; }
  fail(msg: string, line?: Line): never { throw new YamlError(`line ${line?.no ?? this.lines[this.i]?.no ?? '?'}: ${msg}`); }

  parseDocument(): YamlValue {
    this.skip();
    if (this.i >= this.lines.length) return null;
    const v = this.parseBlock(this.lines[this.i].indent);
    this.skip();
    if (this.i < this.lines.length) this.fail(`unexpected content '${this.lines[this.i].text}'`);
    return v;
  }

  parseBlock(indent: number): YamlValue {
    this.skip();
    const l = this.lines[this.i];
    if (!l || l.indent < indent) return null;
    const t = stripComment(l.text);
    if (t === '-' || t.startsWith('- ')) return this.parseSeq(l.indent);
    if (keySep(t) >= 0 || /^\?\s/.test(t)) return this.parseMap(l.indent);
    // a scalar / flow node possibly spanning lines
    this.i++;
    return this.parseInline(this.collectContinuation(t, l.indent), l.indent);
  }

  /** Join continuation lines of a multi-line plain/flow/quoted scalar (deeper-indented, not a new key/item). */
  collectContinuation(t: string, indent: number): string {
    let text = t;
    const open = () => { let d = 0, q: string | null = null; for (let k = 0; k < text.length; k++) { const c = text[k]; if (q) { if (c === q) q = null; else if (c === '\\' && q === '"') k++; continue; } if (c === '"' || c === "'") q = c; else if (c === '[' || c === '{') d++; else if (c === ']' || c === '}') d--; } return d > 0 || q !== null; };
    while (this.i < this.lines.length) {
      const n = this.lines[this.i];
      if (!n.text.trim()) { if (open()) { this.i++; continue; } break; }
      if (open() || (n.indent > indent && !/^[\[{"'|>]/.test(text.trim()) && !text.trim().startsWith('&') && !text.trim().startsWith('*'))) {
        if (!open() && (keySep(stripComment(n.text)) >= 0 || n.text.startsWith('- '))) break;
        text += (open() ? '\n' : ' ') + stripComment(n.text);
        this.i++;
      } else break;
    }
    return text;
  }

  parseSeq(indent: number): YamlValue[] {
    const out: YamlValue[] = [];
    for (;;) {
      this.skip();
      const l = this.lines[this.i];
      if (!l || l.indent !== indent) { if (l && l.indent > indent) this.fail('bad indentation of a sequence entry', l); break; }
      const t = stripComment(l.text);
      if (!(t === '-' || t.startsWith('- '))) break;
      const rest = t === '-' ? '' : t.slice(2);
      const restTrim = rest.trimStart();
      if (!restTrim) { this.i++; out.push(this.parseBlock(indent + 1)); continue; }
      // "- key: value" opens a mapping whose indent is the column of the key
      const col = indent + 2 + (rest.length - restTrim.length);
      if (restTrim === '-' || restTrim.startsWith('- ') || keySep(restTrim) >= 0) {
        this.lines[this.i] = { indent: col, text: restTrim, no: l.no };
        out.push(this.parseBlock(col));
        continue;
      }
      this.i++;
      out.push(this.parseNodeValue(restTrim, indent));
    }
    return out;
  }

  parseMap(indent: number): Map<any, YamlValue> {
    const out = new Map<any, YamlValue>();
    for (;;) {
      this.skip();
      const l = this.lines[this.i];
      if (!l || l.indent !== indent) { if (l && l.indent > indent) this.fail(`bad indentation of a mapping entry '${l.text}'`, l); break; }
      const t = stripComment(l.text);
      if (t === '-' || t.startsWith('- ')) break;
      const sep = keySep(t);
      if (sep < 0) this.fail(`expected a mapping key in '${t}'`, l);
      const keyText = t.slice(0, sep).trim();
      const key = this.scalarKey(keyText);
      const rest = t.slice(sep + 1).trim();
      this.i++;
      let val: YamlValue;
      if (!rest) {
        this.skip();
        const n = this.lines[this.i];
        // a sequence may sit at the same indent as its key
        if (n && (n.indent > indent || (n.indent === indent && (stripComment(n.text) === '-' || n.text.startsWith('- '))))) val = this.parseBlock(n.indent);
        else val = null;
      } else val = this.parseNodeValue(rest, indent);
      if (key === '<<' ) {
        const merge = (m: YamlValue) => { if (m instanceof Map) for (const [k, v] of m) if (!out.has(k)) out.set(k, v); };
        if (Array.isArray(val)) val.forEach(merge); else merge(val);
      } else out.set(key, val);
    }
    return out;
  }

  scalarKey(s: string): any {
    const v = this.parseInline(s, 0);
    if (v instanceof Map || Array.isArray(v)) return JSON.stringify(v);
    return typeof v === 'bigint' ? Number(v) : (v as any);
  }

  /** Value after `key:` or `- `: tags, anchors, block scalars, nested block on following lines, or inline. */
  parseNodeValue(rest: string, parentIndent: number): YamlValue {
    let tag: string | null = null, anchor: string | null = null;
    let r = rest;
    for (;;) {
      const m = /^(![^\s]*|&[^\s]+)(\s+|$)/.exec(r);
      if (!m) break;
      if (m[1].startsWith('&')) anchor = m[1].slice(1); else tag = m[1];
      r = r.slice(m[0].length);
    }
    let v: YamlValue;
    if (/^[|>][-+0-9]*$/.test(r)) v = this.blockScalar(r, parentIndent);
    else if (!r) {
      this.skip();
      const n = this.lines[this.i];
      v = n && n.indent > parentIndent ? this.parseBlock(n.indent) : null;
    } else v = this.parseInline(this.collectContinuation(r, parentIndent), parentIndent, true);
    if (tag) v = this.applyTag(tag, v);
    if (anchor) this.anchors.set(anchor, v);
    return v;
  }

  applyTag(tag: string, v: YamlValue): YamlValue {
    const scalar = typeof v === 'string' ? v : v === null ? '' : Array.isArray(v) || v instanceof Map ? null : String(v);
    switch (tag) {
      case '!!str': case '!': return scalar ?? v;
      case '!!int': return resolvePlain(String(scalar), this.opts.bigint);
      case '!!float': return parseFloat(String(scalar));
      case '!!bool': return BOOL_TRUE.test(String(scalar));
      case '!!null': return null;
      case '!!seq': case '!!map': return v;
    }
    const ctor = this.opts.tags?.[tag];
    if (!ctor) throw new YamlError(`could not determine a constructor for the tag '${tag}'`);
    return ctor(scalar ?? '') as YamlValue;
  }

  blockScalar(head: string, parentIndent: number): string {
    const literal = head[0] === '|';
    const chomp = head.includes('-') ? 'strip' : head.includes('+') ? 'keep' : 'clip';
    const explicit = /\d/.exec(head)?.[0];
    const lines: string[] = [];
    let ind = explicit ? parentIndent + parseInt(explicit, 10) : -1;
    while (this.i < this.lines.length) {
      const l = this.lines[this.i];
      if (l.text.trim() === '') { lines.push(''); this.i++; continue; }
      if (ind < 0) ind = l.indent;
      if (l.indent < ind || l.indent <= parentIndent) break;
      lines.push(' '.repeat(l.indent - ind) + l.text);
      this.i++;
    }
    let trailing = 0;
    while (lines.length && lines[lines.length - 1] === '') { lines.pop(); trailing++; }
    let s: string;
    if (literal) s = lines.join('\n');
    else {
      s = '';
      for (let k = 0; k < lines.length; k++) {
        const ln = lines[k];
        if (k === 0) { s = ln; continue; }
        if (ln === '' ) s += '\n';
        else if (lines[k - 1] === '' || /^\s/.test(ln) || /^\s/.test(lines[k - 1])) s += (s.endsWith('\n') ? '' : '\n') + ln;
        else s += ' ' + ln;
      }
    }
    if (chomp === 'strip') return s;
    if (chomp === 'keep') return s + '\n'.repeat(trailing + (lines.length ? 1 : 0));
    return lines.length ? s + '\n' : s;
  }

  /** Inline node: flow collection, quoted scalar, alias, plain scalar (with optional tag/anchor prefix). */
  parseInline(src: string, indent: number, tagsDone = false): YamlValue {
    const p = new FlowParser(src.trim(), this, indent);
    const v = tagsDone ? p.node() : p.node();
    p.ws();
    if (p.i < p.s.length) this.fail(`unexpected characters '${p.s.slice(p.i)}'`);
    return v;
  }
}

class FlowParser {
  i = 0;
  constructor(readonly s: string, readonly r: Reader, readonly indent: number) {}
  ws() { while (this.i < this.s.length && /\s/.test(this.s[this.i])) this.i++; }
  node(inFlow = false): YamlValue {
    this.ws();
    let tag: string | null = null, anchor: string | null = null;
    for (;;) {
      const m = /^(![^\s,\]}]*|&[^\s,\]}]+)(\s+|(?=[,\]}])|$)/.exec(this.s.slice(this.i));
      if (!m) break;
      if (m[1].startsWith('&')) anchor = m[1].slice(1); else tag = m[1];
      this.i += m[0].length;
    }
    this.ws();
    let v: YamlValue;
    const c = this.s[this.i];
    if (c === '[') v = this.seq();
    else if (c === '{') v = this.map();
    else if (c === '"' || c === "'") v = this.quoted();
    else if (c === '*') { const m = /^\*([^\s,\]}]+)/.exec(this.s.slice(this.i))!; this.i += m[0].length; if (!this.r.anchors.has(m[1])) this.r.fail(`found undefined alias '${m[1]}'`); v = this.r.anchors.get(m[1])!; }
    else v = this.plain(inFlow);
    if (tag) v = this.r.applyTag(tag, typeof v === 'string' || v === null || typeof v !== 'object' ? (v === null && tag !== '!!null' ? '' : v) : v);
    if (anchor) this.r.anchors.set(anchor, v);
    return v;
  }
  quoted(): string {
    const q = this.s[this.i++];
    let out = '';
    while (this.i < this.s.length) {
      const c = this.s[this.i];
      if (c === q) {
        if (q === "'" && this.s[this.i + 1] === "'") { out += "'"; this.i += 2; continue; }
        this.i++;
        return q === '"' ? out : out.replace(/[ \t]*\n[ \t]*/g, (m) => (m.split('\n').length > 2 ? '\n' : ' '));
      }
      if (q === '"' && c === '\\') {
        const e = this.s[this.i + 1];
        const map: Record<string, string> = { n: '\n', t: '\t', r: '\r', '"': '"', '\\': '\\', '/': '/', '0': '\0', a: '\x07', b: '\b', e: '\x1b', f: '\f', v: '\v', ' ': ' ', N: '\x85', _: '\xa0' };
        if (e === 'x') { out += String.fromCharCode(parseInt(this.s.substr(this.i + 2, 2), 16)); this.i += 4; continue; }
        if (e === 'u') { out += String.fromCharCode(parseInt(this.s.substr(this.i + 2, 4), 16)); this.i += 6; continue; }
        if (e === 'U') { out += String.fromCodePoint(parseInt(this.s.substr(this.i + 2, 8), 16)); this.i += 10; continue; }
        if (e === '\n') { this.i += 2; while (/[ \t]/.test(this.s[this.i] ?? '')) this.i++; continue; }
        if (e in map) { out += map[e]; this.i += 2; continue; }
        this.r.fail(`unknown escape \\${e}`);
      }
      if (q === '"' && c === '\n') { out = out.replace(/[ \t]+$/, ''); let nl = 0; while (/[\s]/.test(this.s[this.i] ?? '')) { if (this.s[this.i] === '\n') nl++; this.i++; } out += nl > 1 ? '\n'.repeat(nl - 1) : ' '; continue; }
      out += c;
      this.i++;
    }
    this.r.fail('unterminated quoted scalar');
  }
  plain(inFlow: boolean): YamlValue {
    const start = this.i;
    while (this.i < this.s.length) {
      const c = this.s[this.i];
      if (inFlow && (c === ',' || c === ']' || c === '}')) break;
      if (inFlow && c === ':' && (/[\s,\]}]/.test(this.s[this.i + 1] ?? ' '))) break;
      if (c === '#' && /\s/.test(this.s[this.i - 1] ?? '')) break;
      this.i++;
    }
    const text = this.s.slice(start, this.i).trim().replace(/\s*\n\s*/g, ' ');
    return resolvePlain(text, this.r.opts.bigint);
  }
  seq(): YamlValue[] {
    this.i++;
    const out: YamlValue[] = [];
    for (;;) {
      this.ws();
      if (this.s[this.i] === ']') { this.i++; return out; }
      const save = this.i;
      const k = this.node(true);
      this.ws();
      if (this.s[this.i] === ':') { this.i++; const v = this.node(true); out.push(new Map([[k as any, v]])); } else { out.push(k); void save; }
      this.ws();
      if (this.s[this.i] === ',') { this.i++; continue; }
      if (this.s[this.i] === ']') { this.i++; return out; }
      this.r.fail(`expected ',' or ']' in flow sequence`);
    }
  }
  map(): Map<any, YamlValue> {
    this.i++;
    const out = new Map<any, YamlValue>();
    for (;;) {
      this.ws();
      if (this.s[this.i] === '}') { this.i++; return out; }
      const k = this.node(true);
      this.ws();
      let v: YamlValue = null;
      if (this.s[this.i] === ':') { this.i++; v = this.node(true); }
      out.set(typeof k === 'bigint' ? Number(k) : k, v);
      this.ws();
      if (this.s[this.i] === ',') { this.i++; continue; }
      if (this.s[this.i] === '}') { this.i++; return out; }
      this.r.fail(`expected ',' or '}' in flow mapping`);
    }
  }
}

/** Parse YAML into Maps / arrays / scalars (PyYAML SafeLoader typing). */
export function parseYamlRaw(src: string, opts: YamlReadOptions = {}): YamlValue {
  return new Reader(src, opts).parseDocument();
}

/** Parse YAML into plain JS objects (mapping keys become strings). */
export function parseYaml(src: string, opts: YamlReadOptions = {}): any {
  const conv = (v: any): any => {
    if (v instanceof Map) { const o: Record<string, any> = {}; for (const [k, x] of v) o[String(k)] = conv(x); return o; }
    if (Array.isArray(v)) return v.map(conv);
    return v;
  };
  return conv(parseYamlRaw(src, opts));
}

// ---------------------------------------------------------------------------------------------
// typed writer for ROS 2 parameter files
// ---------------------------------------------------------------------------------------------

/** Force a value to be written as a double / integer regardless of the writer's numeric default. */
export class YDouble { constructor(readonly v: number) {} }
export class YInt { constructor(readonly v: number) {} }
export const ydouble = (v: number) => new YDouble(v);
export const yint = (v: number) => new YInt(Math.round(v));

export interface RosYamlOptions {
  /** How untyped JS numbers are written: 'double' (default — ROS 2 parameters are mostly doubles) or 'auto'. */
  numbers?: 'double' | 'auto';
  /** Parameter names whose numeric values are integers (e.g. Nav2 `max_iterations`). */
  intKeys?: Iterable<string>;
}

/** A double literal YAML/rcl both read as a double: always has a '.' (or inf/nan). */
export function formatDouble(x: number): string {
  if (Number.isNaN(x)) return '.nan';
  if (!Number.isFinite(x)) return x > 0 ? '.inf' : '-.inf';
  if (x === 0) return Object.is(x, -0) ? '-0.0' : '0.0';
  let s = String(x);
  if (/e/i.test(s)) { const [m, e] = s.split(/e/i); s = `${m.includes('.') ? m : `${m}.0`}e${e.startsWith('-') ? '-' : '+'}${e.replace(/^[+-]/, '').padStart(2, '0')}`; }
  else if (!s.includes('.')) s += '.0';
  return s;
}

const YAML_RESERVED = /^(?:~|null|Null|NULL|y|Y|yes|Yes|YES|n|N|no|No|NO|true|True|TRUE|false|False|FALSE|on|On|ON|off|Off|OFF|=|<<|[-+]?(?:inf|infinity|nan|\.inf|\.nan))$/i;
export function formatYamlString(s: string): string {
  const needs = s === '' || YAML_RESERVED.test(s) || resolvePlain(s) !== s || /^[\s\-?:,\[\]{}#&*!|>'"%@`]/.test(s) || /[:#]\s|\s#|:$|^\s|\s$|[\n\t\x00-\x1f]/.test(s) || /^[-+]?(\d|\.\d)/.test(s);
  return needs ? JSON.stringify(s) : s;
}

/** Write a ROS 2 parameter file (or any YAML document) with explicit numeric typing. */
export function toRosYaml(obj: any, opts: RosYamlOptions = {}): string {
  const intKeys = new Set(opts.intKeys ?? []);
  const asDouble = (opts.numbers ?? 'double') === 'double';
  const scalar = (v: any, key: string, forceDouble = false): string => {
    if (v instanceof YDouble) return formatDouble(v.v);
    if (v instanceof YInt) return String(v.v);
    if (typeof v === 'bigint') return v.toString();
    if (typeof v === 'number') {
      if (intKeys.has(key) && Number.isInteger(v) && !forceDouble) return String(v);
      if (forceDouble || asDouble || !Number.isInteger(v)) return formatDouble(v);
      return String(v);
    }
    if (typeof v === 'boolean') return v ? 'true' : 'false';
    if (v === null || v === undefined) return 'null';
    return formatYamlString(String(v));
  };
  const isScalar = (v: any) => v === null || typeof v !== 'object' || v instanceof YDouble || v instanceof YInt;
  const seq = (arr: any[], key: string): string => {
    const nums = arr.filter((x) => typeof x === 'number' || x instanceof YDouble || x instanceof YInt);
    // homogeneous: a sequence with any double (or in double mode) is written entirely as doubles
    const dbl = nums.length === arr.length && arr.length > 0 && !intKeys.has(key) && (asDouble || arr.some((x) => x instanceof YDouble || (typeof x === 'number' && !Number.isInteger(x))));
    if (arr.some((x) => typeof x === 'string') && arr.some((x) => typeof x !== 'string')) throw new Error(`YAML parameter '${key}': mixed-type sequence`);
    return `[${arr.map((x) => scalar(x instanceof YInt && dbl ? x.v : x, key, dbl)).join(', ')}]`;
  };
  const emit = (v: any, indent: number, key: string): string => {
    const pad = '  '.repeat(indent);
    if (Array.isArray(v)) {
      if (v.every(isScalar)) return ` ${seq(v, key)}\n`;
      if (!v.length) return ' []\n';
      return '\n' + v.map((x) => {
        if (isScalar(x)) return `${pad}- ${scalar(x, key)}\n`;
        if (Array.isArray(x)) return `${pad}- ${seq(x, key)}\n`;
        const body = emit(x, indent + 1, key);
        return `${pad}-${body.replace(/^\n\s*/, ' ')}`;
      }).join('');
    }
    if (v && typeof v === 'object' && !(v instanceof YDouble) && !(v instanceof YInt)) {
      const entries = v instanceof Map ? [...v.entries()] : Object.entries(v);
      if (!entries.length) return ' {}\n';
      return '\n' + entries.filter(([, x]) => x !== undefined).map(([k, x]) => `${pad}${formatYamlString(String(k))}:${emit(x, indent + 1, String(k))}`).join('');
    }
    return ` ${scalar(v, key)}\n`;
  };
  const out = emit(obj, 0, '');
  return out.replace(/^\n/, '');
}
