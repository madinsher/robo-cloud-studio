/**
 * Webots VRML_SIM (.wbt / .proto) lexer and parser. Produces a light AST of nodes with untyped field values; field
 * semantics (SF/MF types) are applied by the consumers (PROTO interfaces, scene builder). Handles UTF-8 text, `#`
 * comments outside strings, commas as whitespace, MF values without brackets, DEF/USE (per parse scope), IS, NULL,
 * EXTERNPROTO / IMPORTABLE EXTERNPROTO statements and PROTO interfaces (enumerated value sets, node defaults).
 * Template islands (%< >%) must be expanded before a PROTO body is parsed (see template.ts).
 */

export interface WNode {
  type: string;
  def?: string;
  fields: Record<string, WValue>;
  /** Path of the file (or PROTO) the node text comes from: relative URLs (meshes, textures, PROTOs) resolve against it. */
  base: string;
  line?: number;
  /** Set on nodes produced by a PROTO expansion: instance information. */
  proto?: { name: string; url: string; def?: string; fields: Record<string, WValue> };
  /** Built-in equivalents tag their nodes (e.g. 'ground' floors, 'background'). */
  meta?: Record<string, unknown>;
}

export type WValue =
  | { k: 'num'; v: number[] }
  | { k: 'str'; v: string[] }
  | { k: 'bool'; v: boolean[] }
  | { k: 'node'; v: WNode | null }
  | { k: 'nodes'; v: WNode[] }
  | { k: 'is'; v: string };

export interface WExtern { url: string; importable: boolean }

export interface WFile {
  path: string;
  /** e.g. 'R2025a' (empty for unknown headers). */
  version: string;
  header: string;
  externs: WExtern[];
  nodes: WNode[];
  /** Header comment metadata (`# template language: javascript`, `# tags: ...`). */
  meta: Record<string, string>;
}

export class WebotsSyntaxError extends Error {
  constructor(msg: string, public file: string, public line: number) {
    super(`${file}:${line}: ${msg}`);
  }
}

type TokT = 'num' | 'str' | 'id' | '{' | '}' | '[' | ']' | 'eof';
export interface Tok { t: TokT; v: string; n?: number; line: number; pos: number; end: number }

const NUM_RE = /^[+-]?(?:0[xX][0-9a-fA-F]+|(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?)$/;
const isSpace = (c: string) => c === ' ' || c === '\t' || c === '\n' || c === '\r' || c === ',' || c === '\f' || c === '\v' || c === '﻿';

/** Incremental lexer: PROTO interfaces are lexed up to the body start only (the body may hold template code). */
export class Lexer {
  pos = 0;
  line = 1;
  private peeked: Tok | null = null;
  constructor(public src: string, public file: string) {}

  private scan(): Tok {
    const s = this.src;
    const n = s.length;
    for (;;) {
      while (this.pos < n && isSpace(s[this.pos])) { if (s[this.pos] === '\n') this.line++; this.pos++; }
      if (this.pos < n && s[this.pos] === '#') { while (this.pos < n && s[this.pos] !== '\n') this.pos++; continue; }
      break;
    }
    const start = this.pos;
    if (this.pos >= n) return { t: 'eof', v: '', line: this.line, pos: start, end: start };
    const c = s[this.pos];
    if (c === '{' || c === '}' || c === '[' || c === ']') { this.pos++; return { t: c, v: c, line: this.line, pos: start, end: this.pos }; }
    if (c === '"') {
      let out = '';
      let j = this.pos + 1;
      const line = this.line;
      while (j < n && s[j] !== '"') {
        if (s[j] === '\\' && j + 1 < n) { out += s[j + 1]; j += 2; continue; }
        if (s[j] === '\n') this.line++;
        out += s[j++];
      }
      if (j >= n) throw new WebotsSyntaxError('unterminated string', this.file, line);
      this.pos = j + 1;
      return { t: 'str', v: out, line, pos: start, end: this.pos };
    }
    let j = this.pos;
    while (j < n && !isSpace(s[j]) && s[j] !== '{' && s[j] !== '}' && s[j] !== '[' && s[j] !== ']' && s[j] !== '"' && s[j] !== '#') j++;
    const w = s.slice(this.pos, j);
    this.pos = j;
    if (NUM_RE.test(w)) return { t: 'num', v: w, n: /^[+-]?0[xX]/.test(w) ? parseInt(w, 16) : +w, line: this.line, pos: start, end: j };
    return { t: 'id', v: w, line: this.line, pos: start, end: j };
  }
  peek(): Tok {
    if (!this.peeked) this.peeked = this.scan();
    return this.peeked;
  }
  next(): Tok {
    const t = this.peek();
    this.peeked = null;
    return t;
  }
  expect(t: TokT, v?: string): Tok {
    const k = this.next();
    if (k.t !== t || (v !== undefined && k.v !== v)) throw new WebotsSyntaxError(`expected ${v ?? t}, got ${k.t === 'eof' ? 'end of file' : `'${k.v}'`}`, this.file, k.line);
    return k;
  }
  /** Offset just after the last consumed token. */
  offset(): number {
    return this.peeked ? this.peeked.pos : this.pos;
  }
}

const KEYWORDS = new Set(['TRUE', 'FALSE', 'NULL', 'IS', 'DEF', 'USE', 'true', 'false']);

export interface ParseScope {
  defs: Map<string, WNode>;
  warnings: string[];
}

export function newScope(): ParseScope {
  return { defs: new Map(), warnings: [] };
}

/** Parse one field value (type-agnostic). */
export function parseValue(lx: Lexer, base: string, scope: ParseScope): WValue {
  const t = lx.peek();
  if (t.t === '[') {
    lx.next();
    const nums: number[] = [], strs: string[] = [], bools: boolean[] = [], nodes: WNode[] = [];
    for (;;) {
      const k = lx.peek();
      if (k.t === ']') { lx.next(); break; }
      if (k.t === 'eof') throw new WebotsSyntaxError("missing ']'", lx.file, k.line);
      if (k.t === 'num') { lx.next(); nums.push(k.n!); }
      else if (k.t === 'str') { lx.next(); strs.push(k.v); }
      else if (k.t === 'id' && (k.v === 'TRUE' || k.v === 'true')) { lx.next(); bools.push(true); }
      else if (k.t === 'id' && (k.v === 'FALSE' || k.v === 'false')) { lx.next(); bools.push(false); }
      else if (k.t === 'id' && k.v === 'NULL') { lx.next(); }
      else if (k.t === 'id') { const nd = parseNode(lx, base, scope); if (nd) nodes.push(nd); }
      else throw new WebotsSyntaxError(`unexpected '${k.v}' in list`, lx.file, k.line);
    }
    if (nums.length) return { k: 'num', v: nums };
    if (strs.length) return { k: 'str', v: strs };
    if (bools.length) return { k: 'bool', v: bools };
    return { k: 'nodes', v: nodes };
  }
  if (t.t === 'str') { lx.next(); return { k: 'str', v: [t.v] }; }
  if (t.t === 'num') {
    const v: number[] = [];
    while (lx.peek().t === 'num') v.push(lx.next().n!);
    return { k: 'num', v };
  }
  if (t.t === 'id') {
    if (t.v === 'TRUE' || t.v === 'true') { lx.next(); return { k: 'bool', v: [true] }; }
    if (t.v === 'FALSE' || t.v === 'false') { lx.next(); return { k: 'bool', v: [false] }; }
    if (t.v === 'NULL') { lx.next(); return { k: 'node', v: null }; }
    if (t.v === 'IS') { lx.next(); return { k: 'is', v: lx.expect('id').v }; }
    return { k: 'node', v: parseNode(lx, base, scope) };
  }
  throw new WebotsSyntaxError(`unexpected '${t.v || t.t}'`, lx.file, t.line);
}

/** Parse `[DEF name] Type { fields }` or `USE name`. Returns null for an unresolved USE (warned). */
export function parseNode(lx: Lexer, base: string, scope: ParseScope): WNode | null {
  let t = lx.next();
  if (t.t === 'id' && t.v === 'USE') {
    const name = lx.expect('id').v;
    const n = scope.defs.get(name);
    if (!n) scope.warnings.push(`${lx.file}:${t.line}: USE ${name} before DEF`);
    return n ?? null;
  }
  let def: string | undefined;
  if (t.t === 'id' && t.v === 'DEF') { def = lx.expect('id').v; t = lx.next(); }
  if (t.t !== 'id' || KEYWORDS.has(t.v)) throw new WebotsSyntaxError(`expected a node type, got '${t.v || t.t}'`, lx.file, t.line);
  const node: WNode = { type: t.v, fields: {}, base, line: t.line };
  if (def) node.def = def;
  lx.expect('{');
  for (;;) {
    const f = lx.next();
    if (f.t === '}') break;
    if (f.t !== 'id') throw new WebotsSyntaxError(`expected a field name in ${node.type}, got '${f.v || f.t}'`, lx.file, f.line);
    node.fields[f.v] = parseValue(lx, base, scope);
  }
  if (def) scope.defs.set(def, node);
  return node;
}

/** Header comment metadata from the leading comment block. */
export function headerMeta(text: string): { header: string; version: string; meta: Record<string, string> } {
  const lines = text.replace(/^﻿/, '').split(/\r?\n/);
  const header = lines[0] ?? '';
  const m = /^#VRML_SIM\s+(R\d{4}[a-z]?)/.exec(header);
  const meta: Record<string, string> = {};
  for (const l of lines.slice(1)) {
    const s = l.trim();
    if (s && !s.startsWith('#')) break;
    const mm = /^#\s*(template language|tags|license|license url|documentation url|keywords)\s*:\s*(.*)$/i.exec(s);
    if (mm) meta[mm[1].toLowerCase()] = mm[2].trim();
  }
  return { header, version: m ? m[1] : /^#VRML V2\.0/.test(header) ? 'VRML2' : '', meta };
}

/** Parse EXTERNPROTO statements at the current position. */
function parseExterns(lx: Lexer, out: WExtern[]): void {
  for (;;) {
    const t = lx.peek();
    if (t.t === 'id' && t.v === 'EXTERNPROTO') { lx.next(); out.push({ url: lx.expect('str').v, importable: false }); continue; }
    if (t.t === 'id' && t.v === 'IMPORTABLE') { lx.next(); lx.expect('id', 'EXTERNPROTO'); out.push({ url: lx.expect('str').v, importable: true }); continue; }
    break;
  }
}

/** Parse a world (.wbt) file. */
export function parseWorld(text: string, path: string, scope: ParseScope = newScope()): WFile {
  const { header, version, meta } = headerMeta(text);
  const lx = new Lexer(text, path);
  const externs: WExtern[] = [];
  const nodes: WNode[] = [];
  for (;;) {
    parseExterns(lx, externs);
    const t = lx.peek();
    if (t.t === 'eof') break;
    const n = parseNode(lx, path, scope);
    if (n) nodes.push(n);
  }
  return { path, version, header, externs, nodes, meta };
}

/** Parse a text holding only nodes (expanded PROTO body). */
export function parseNodes(text: string, base: string, scope: ParseScope = newScope()): WNode[] {
  const lx = new Lexer(text, base);
  const nodes: WNode[] = [];
  while (lx.peek().t !== 'eof') { const n = parseNode(lx, base, scope); if (n) nodes.push(n); }
  return nodes;
}

export interface ProtoFieldDecl {
  kind: string;
  type: string;
  name: string;
  def: WValue;
  /** Raw text of an enumerated value set `{...}` if any. */
  enumText?: string;
}

export interface ProtoFile {
  path: string;
  name: string;
  version: string;
  meta: Record<string, string>;
  externs: WExtern[];
  fields: ProtoFieldDecl[];
  /** Body text between the outer braces (may contain template islands). */
  body: string;
  bodyLine: number;
  template: 'javascript' | 'lua' | null;
}

const FIELD_KINDS = new Set(['field', 'vrmlField', 'hiddenField', 'deprecatedField', 'unconnectedField', 'exposedField']);

/** Parse a .proto file: EXTERNPROTO statements, interface and the raw body text. */
export function parseProtoFile(text: string, path: string): ProtoFile {
  const { version, meta } = headerMeta(text);
  const lx = new Lexer(text, path);
  const externs: WExtern[] = [];
  parseExterns(lx, externs);
  lx.expect('id', 'PROTO');
  const name = lx.expect('id').v;
  lx.expect('[');
  const fields: ProtoFieldDecl[] = [];
  const scope = newScope();
  for (;;) {
    const t = lx.next();
    if (t.t === ']') break;
    if (t.t !== 'id' || !FIELD_KINDS.has(t.v)) throw new WebotsSyntaxError(`expected a field declaration, got '${t.v || t.t}'`, path, t.line);
    const type = lx.expect('id').v;
    let enumText: string | undefined;
    if (lx.peek().t === '{') {
      const open = lx.next();
      let depth = 1;
      while (depth > 0) { const k = lx.next(); if (k.t === 'eof') throw new WebotsSyntaxError('unterminated value set', path, open.line); if (k.t === '{') depth++; else if (k.t === '}') depth--; }
      enumText = text.slice(open.pos, lx.offset());
    }
    const fname = lx.expect('id').v;
    const def = parseValue(lx, path, scope);
    fields.push({ kind: t.v, type, name: fname, def, ...(enumText ? { enumText } : {}) });
  }
  const open = lx.expect('{');
  // body ends at the last '}' that is not inside a trailing comment line
  let end = text.length;
  const tail = text.slice(open.end);
  const lines = tail.split('\n');
  let off = text.length;
  for (let i = lines.length - 1; i >= 0; i--) {
    off -= lines[i].length + (i === lines.length - 1 ? 0 : 1);
    const l = lines[i];
    const hash = l.indexOf('#');
    const code = hash >= 0 ? l.slice(0, hash) : l;
    const b = code.lastIndexOf('}');
    if (b >= 0) { end = off + b; break; }
  }
  const tl = (meta['template language'] ?? '').toLowerCase();
  const template = tl === 'javascript' ? 'javascript' : tl === 'lua' ? 'lua' : /%<[\s\S]*?>%/.test(tail) ? 'javascript' : /%\{[\s\S]*?\}%/.test(tail) ? 'lua' : null;
  return { path, name, version, meta, externs, fields, body: text.slice(open.end, end), bodyLine: open.line, template };
}

// ---- value accessors -----------------------------------------------------------------------------------------------

export function fNum(n: WNode | null | undefined, f: string, def: number[]): number[] {
  const v = n?.fields[f];
  return v && v.k === 'num' && v.v.length ? v.v : def;
}
export function fNum1(n: WNode | null | undefined, f: string, def: number): number {
  const v = n?.fields[f];
  return v && v.k === 'num' && v.v.length ? v.v[0] : def;
}
export function fStr(n: WNode | null | undefined, f: string, def: string): string {
  const v = n?.fields[f];
  return v && v.k === 'str' && v.v.length ? v.v[0] : def;
}
export function fStrs(n: WNode | null | undefined, f: string): string[] {
  const v = n?.fields[f];
  return v && v.k === 'str' ? v.v : [];
}
export function fBool(n: WNode | null | undefined, f: string, def: boolean): boolean {
  const v = n?.fields[f];
  return v && v.k === 'bool' && v.v.length ? v.v[0] : def;
}
export function fNode(n: WNode | null | undefined, f: string): WNode | null {
  const v = n?.fields[f];
  if (!v) return null;
  if (v.k === 'node') return v.v;
  if (v.k === 'nodes') return v.v[0] ?? null;
  return null;
}
export function fNodes(n: WNode | null | undefined, f: string): WNode[] {
  const v = n?.fields[f];
  if (!v) return [];
  if (v.k === 'nodes') return v.v;
  if (v.k === 'node') return v.v ? [v.v] : [];
  return [];
}

/**
 * Count node types textually (each `Type {` occurrence; USE references are not counted). Template islands are replaced
 * like the investigator's inventory did (expressions by 0, statements removed: both branches of an if are counted).
 */
export function countTypesInText(text: string, file = 'text'): Record<string, number> {
  const src = text.replace(/%<=([\s\S]*?)>%/g, ' 0 ').replace(/%<([\s\S]*?)>%/g, ' ');
  const lx = new Lexer(src, file);
  const acc: Record<string, number> = {};
  let prev: Tok | null = null;
  for (;;) {
    const t = lx.next();
    if (t.t === 'eof') break;
    if (t.t === '{' && prev && prev.t === 'id' && !FIELD_TYPE_RE.test(prev.v) && prev.v !== 'PROTO') acc[prev.v] = (acc[prev.v] ?? 0) + 1;
    prev = t;
  }
  return acc;
}
const FIELD_TYPE_RE = /^(SF|MF)(Bool|Int32|Float|String|Vec2f|Vec3f|Rotation|Color|Node)$/;

/** Count node types in a parsed tree (USE'd subtrees are counted again for each reference). */
export function countNodeTypes(nodes: WNode[], acc: Record<string, number> = {}): Record<string, number> {
  const walk = (n: WNode) => {
    acc[n.type] = (acc[n.type] ?? 0) + 1;
    for (const v of Object.values(n.fields)) {
      if (v.k === 'node' && v.v) walk(v.v);
      else if (v.k === 'nodes') v.v.forEach(walk);
    }
  };
  nodes.forEach(walk);
  return acc;
}
