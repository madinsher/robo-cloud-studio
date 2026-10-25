/**
 * xacro preprocessor — a port of ROS 2 xacro 2.x semantics (checked against `xacro` of ROS 2 Jazzy, see
 * tests/ros_xacro*.test.ts and tests/fixtures/ros_gt): in-order processing, `${}` evaluated by a sandboxed Python
 * expression interpreter (./pyexpr, no JS eval), `_eval_literal` typing (int / float / bool / 'quoted'), lazy
 * properties with scopes (local / parent / global) and `lazy_eval`, `remove` / `default`, block properties, macros with
 * defaults (`:=`, `'quoted defaults'`, `^` / `^|` forwarding) and `*` / `**` block parameters, dynamic scoping,
 * local macros, `xacro:call`, namespaced includes (`ns=`), includes relative to the including file, `optional`
 * includes, `xacro:element` / `xacro:attribute`, `$(arg|find|env|optenv|eval)` substitution args, `$${` escapes,
 * `xacro.load_yaml` (with `!degrees` / `!radians` / length unit tags), `python.*` / `math.*` / `xacro.*` symbols.
 *
 * Errors are hard like in ROS (unknown macro, missing include, undefined property / arg, non-boolean condition, bad
 * parameters); `lenient: true` turns them into warnings for best-effort previews.
 */
import { XNode, parseXML, serializeXML } from './xml';
import {
  PyError, PyDict, PyFunc, PyNamespace, PyTuple, PY_BUILTINS, PY_MATH, pyEval, pyStr, pyfn, pyFloat, pyInt, iterate, recordLookup, truthy,
} from './pyexpr';
import { parseYamlRaw, YamlValue } from '../../ros/yaml';

export interface XacroOptions {
  /**
   * Text of an include / yaml file. `path` is resolved like ROS does: relative names against the directory of the
   * including file, `$(find pkg)` as `package://pkg`. `raw` is the filename as written (after substitution).
   */
  resolveInclude?: (path: string, raw?: string) => string | null;
  /** Command-line style args (`name:=value`). */
  args?: Record<string, string>;
  /** Path of the processed file (base of relative includes). */
  filename?: string;
  /** Environment for `$(env)` / `$(optenv)`. */
  env?: Record<string, string>;
  /** Known packages (name -> root path). When given, `$(find unknown)` is an error like in ROS. */
  packages?: Record<string, string> | Map<string, string>;
  /** Glob expansion for include patterns (`*`, `?`, `[`). */
  glob?: (pattern: string) => string[];
  /** Best effort: report errors as warnings and keep going (old behaviour, previews of incomplete packages). */
  lenient?: boolean;
  /** Receives warnings (also returned by expandXacro). */
  warnings?: string[];
}

export interface XacroResult {
  xml: string;
  root: XNode;
  /** Every included / loaded file (resolved paths), like `xacro --deps`. */
  includes: string[];
  warnings: string[];
}

export class XacroError extends Error {
  constructor(message: string, readonly file?: string, readonly cause?: unknown) { super(message); this.name = 'XacroError'; }
}

const PY_KEYWORDS = new Set(['False', 'None', 'True', 'and', 'as', 'assert', 'async', 'await', 'break', 'class', 'continue', 'def', 'del', 'elif', 'else', 'except', 'finally', 'for', 'from', 'global', 'if', 'import', 'in', 'is', 'lambda', 'nonlocal', 'not', 'or', 'pass', 'raise', 'return', 'try', 'while', 'with', 'yield']);
const isValidName = (s: string) => /^[A-Za-z_À-￿][A-Za-z0-9_À-￿]*$/.test(s) && !PY_KEYWORDS.has(s);

class KeyMiss extends Error {}

/** xacro symbol table: lazily evaluated properties, parent chain, global symbols at the root. */
class Table {
  readonly own = new Map<string, any>();
  readonly unevaluated = new Set<string>();
  readonly recursive: string[] = [];
  constructor(readonly ctx: Ctx, readonly parent: Table | null, readonly root: Map<string, any>) {}
  get typeName() { return 'Table'; }
  static evalLiteral(v: any): any {
    if (typeof v !== 'string') return v;
    if (v.length >= 2 && v[0] === "'" && v[v.length - 1] === "'") return v.slice(1, -1);
    if (v.includes('_')) return v;
    const s = v.trim();
    if (/^[+-]?\d+$/.test(s)) return BigInt(s);
    if (/^[+-]?(inf|infinity|nan)$/i.test(s)) return s.startsWith('-') ? -Infinity : /nan/i.test(s) ? NaN : Infinity;
    if (/^[+-]?(\d+\.?\d*|\.\d+)(e[+-]?\d+)?$/i.test(s)) return parseFloat(s);
    if (v === 'true' || v === 'True') return true;
    if (v === 'false' || v === 'False') return false;
    return v;
  }
  hasOwn(k: string) { return this.own.has(k); }
  get(key: string): any {
    if (this.own.has(key)) return this.resolve(key);
    if (this.parent) return this.parent.get(key);
    if (this.root.has(key)) return this.root.get(key);
    throw new KeyMiss(key);
  }
  private resolve(key: string): any {
    if (this.unevaluated.has(key)) {
      if (this.recursive.includes(key)) throw new XacroError(`circular variable definition: ${[...this.recursive, key].join(' -> ')}\nConsider disabling lazy evaluation via lazy_eval="false"`);
      this.recursive.push(key);
      try { this.own.set(key, Table.evalLiteral(this.ctx.evalText(this.own.get(key), this))); }
      finally { this.recursive.splice(this.recursive.indexOf(key), 1); }
      this.unevaluated.delete(key);
    }
    return this.own.get(key);
  }
  setItem(key: string, value: any, unevaluated: boolean) {
    if (this.root.has(key)) this.ctx.warn(`redefining global symbol: ${key}`);
    value = Table.evalLiteral(value);
    this.own.set(key, value);
    if (unevaluated && typeof value === 'string') this.unevaluated.add(key);
    else this.unevaluated.delete(key);
  }
  has(key: string): boolean { return this.own.has(key) || (this.parent ? this.parent.has(key) : this.root.has(key)); }
  del(key: string) {
    for (let p: Table | null = this; p; p = p.parent) { p.own.delete(key); p.unevaluated.delete(key); }
    if (this.root.has(key)) this.ctx.warn(`Cannot remove global symbol: ${key}`);
  }
  top(): Table { let p: Table = this; while (p.parent) p = p.parent; return p; }
}

/** Namespace of an `ns=` include: dotted access forwards to the table (with fallback to the parent chain). */
class NameSpaceTable extends Table {
  get typeName() { return 'NameSpace'; }
  getAttr(name: string) {
    try { return this.get(name); } catch (e) { if (e instanceof KeyMiss) throw new PyError('NameError', `name '${name}' is not defined`); throw e; }
  }
  getItem(key: any) { try { return this.get(String(key)); } catch (e) { if (e instanceof KeyMiss) throw new PyError('KeyError', String(key)); throw e; } }
}

interface Macro { name: string; params: string[]; defaults: Map<string, [string | null, string | null]>; body: XNode }

const DEFAULT_VALUE = String.raw`\$\{.*?\}|\$\(.*?\)|(?:'.*?'|".*?"|[^\s'"]+)+|`;
const RE_MACRO_ARG = new RegExp(String.raw`^\s*([^\s:=]+?)\s*:?=\s*(\^\|?)?(${DEFAULT_VALUE})(?:\s+|$)(.*)$`);

function parseMacroArg(s: string): [string, [string | null, string | null] | null, string] {
  const m = RE_MACRO_ARG.exec(s);
  if (m) {
    const [, param, forward, def, rest] = m;
    return [param, [forward ? param : null, def ? def : null], rest];
  }
  const t = s.replace(/^\s+/, '');
  const i = t.search(/\s/);
  return i < 0 ? [t, null, ''] : [t.slice(0, i), null, t.slice(i).replace(/^\s+/, '')];
}

const isAbsPath = (p: string) => p.startsWith('/') || /^[A-Za-z][A-Za-z0-9+.-]*:\/\//.test(p) || /^[A-Za-z]:[\\/]/.test(p);
function normPath(p: string): string {
  const m = /^([A-Za-z][A-Za-z0-9+.-]*:\/\/[^/]*)(.*)$/.exec(p);
  const head = m ? m[1] : p.startsWith('/') ? '/' : '';
  const body = m ? m[2] : p.startsWith('/') ? p.slice(1) : p;
  const out: string[] = [];
  for (const seg of body.replace(/\\/g, '/').split('/')) {
    if (seg === '' || seg === '.') continue;
    if (seg === '..' && out.length && out[out.length - 1] !== '..') out.pop();
    else out.push(seg);
  }
  const joined = out.join('/');
  return m ? `${head}/${joined}` : head + joined;
}
export function dirnameOf(p: string): string { const n = p.replace(/\\/g, '/'); const i = n.lastIndexOf('/'); if (i < 0) return '.'; const d = n.slice(0, i); return /^[A-Za-z][A-Za-z0-9+.-]*:\/\/[^/]*$/.test(d) ? `${d}/` : d || '/'; }
export function joinPath(dir: string, rel: string): string { return isAbsPath(rel) ? normPath(rel) : normPath(!dir || dir === '.' ? rel : `${dir.replace(/\/$/, '')}/${rel}`); }

const LEX: [string, RegExp][] = [
  ['DDB', /^\$\$+(\{|\()/],
  ['EXPR', /^\$\{[^}]*\}/],
  ['EXT', /^\$\([^)]*\)/],
  ['TEXT', /^(?:[^$]+|\$[^{($]+|\$$)/],
];

const UNITS: Record<string, number> = { '!radians': 1, '!degrees': Math.PI / 180, '!meters': 1, '!millimeters': 0.001, '!foot': 0.3048, '!inches': 0.0254 };

class Ctx {
  args: Record<string, string>;
  filestack: (string | null)[] = [];
  includes: string[] = [];
  warnings: string[];
  globals: Map<string, any>;
  constructor(readonly opts: XacroOptions) {
    // Legacy callers that only pass a basename resolver (no file / package context: the single-file drop and the
    // online library's multi-round fetch) keep the old best-effort behaviour; everything with a package context is
    // strict like ROS. An explicit `lenient` always wins.
    if (opts.lenient === undefined && opts.resolveInclude && opts.filename === undefined && opts.packages === undefined) this.opts = { ...opts, lenient: true };
    this.args = { ...(opts.args ?? {}) };
    this.warnings = opts.warnings ?? [];
    this.filestack = [opts.filename ?? null];
    this.globals = this.createGlobals();
  }
  warn(msg: string) { const f = this.filestack[this.filestack.length - 1]; this.warnings.push(f ? `${msg} [${f}]` : msg); }
  fail(msg: string, cause?: unknown): never {
    const files = this.filestack.filter((f): f is string => !!f);
    throw new XacroError(files.length ? `${msg}\nwhen processing file: ${files.reverse().join(' included from: ')}` : msg, files[0], cause);
  }
  /** Run `fn`; in lenient mode an error becomes a warning and `fallback()` is used. */
  soft<T>(fn: () => T, fallback: (e: Error) => T): T {
    if (!this.opts.lenient) return fn();
    try { return fn(); } catch (e) { this.warn((e as Error).message.split('\n')[0]); return fallback(e as Error); }
  }

  absFilename(spec: string): string {
    if (isAbsPath(spec)) return normPath(spec);
    const parent = this.filestack[this.filestack.length - 1];
    return joinPath(parent ? dirnameOf(parent) : '.', spec);
  }
  readFile(path: string, raw: string): string | null {
    const r = this.opts.resolveInclude?.(path, raw);
    return r === undefined ? null : r;
  }
  findPackage(pkg: string): string {
    const known = this.opts.packages;
    if (known) {
      const has = known instanceof Map ? known.has(pkg) : Object.prototype.hasOwnProperty.call(known, pkg);
      if (!has) {
        if (this.opts.lenient) { this.warn(`package '${pkg}' not found`); return `package://${pkg}`; }
        throw new PyError('PackageNotFoundError', `"package '${pkg}' not found, searching: ${[...(known instanceof Map ? known.keys() : Object.keys(known))].join(', ')}"`);
      }
    }
    return `package://${pkg}`;
  }

  loadYaml(filename: string): any {
    const path = this.absFilename(filename);
    const text = this.readFile(path, filename);
    if (text === null) throw new PyError('FileNotFoundError', `[Errno 2] No such file or directory: '${path}'`);
    this.includes.push(path);
    const tags: Record<string, (s: string) => unknown> = {};
    for (const [tag, c] of Object.entries(UNITS)) tags[tag] = (s: string) => {
      const v = pyEval(s, (n) => { if (this.globals.has(n)) return this.globals.get(n); throw new PyError('NameError', `name '${n}' is not defined`); });
      return toFloat(v) * c;
    };
    this.filestack.push(path);
    try { return yamlToPy(parseYamlRaw(text, { bigint: true, tags })); } finally { this.filestack.pop(); }
  }

  createGlobals(): Map<string, any> {
    const g = new Map<string, any>();
    const deprecated = (name: string, ns: string, f: any) => (f instanceof PyFunc ? pyfn(name, (a, kw) => { this.warn(`Using ${name}() directly is deprecated. Use ${ns}.${name}() instead.`); return f.fn(a, kw); }) : f);
    for (const k of ['list', 'dict', 'map', 'len', 'str', 'float', 'int', 'bool', 'min', 'max', 'round']) g.set(k, PY_BUILTINS[k]);
    g.set('True', true); g.set('False', false);
    const pyNs: Record<string, any> = { True: true, False: false };
    for (const k of ['list', 'dict', 'map', 'len', 'str', 'float', 'int', 'bool', 'min', 'max', 'round', 'abs', 'all', 'any', 'divmod', 'enumerate', 'filter', 'ord', 'repr', 'reversed', 'set', 'sum', 'tuple', 'type', 'zip', 'sorted', 'range', 'isinstance']) pyNs[k] = PY_BUILTINS[k];
    g.set('python', new PyNamespace('python', pyNs));
    g.set('sorted', deprecated('sorted', 'python', PY_BUILTINS.sorted));
    g.set('range', deprecated('range', 'python', PY_BUILTINS.range));
    for (const [k, v] of Object.entries(PY_MATH)) g.set(k, v);
    g.set('math', new PyNamespace('math', PY_MATH));
    const loadYaml = pyfn('load_yaml', (a) => this.loadYaml(pyStr(a[0])));
    const absFilename = pyfn('abs_filename', (a) => this.absFilename(pyStr(a[0])));
    const dotify = pyfn('dotify', (a) => new PyDict(a[0] instanceof PyDict ? a[0].entries() : [], true));
    const msg = (name: string) => pyfn(name, (a) => { const t = a.map(pyStr).join(' '); if (name !== 'print_location') this.warnings.push(`${name}: ${t}`); return ''; });
    const xacroNs: Record<string, any> = {
      load_yaml: loadYaml, abs_filename: absFilename, dotify,
      arg: pyfn('arg', (a) => { const n = pyStr(a[0]); if (!(n in this.args)) throw new PyError('KeyError', `'${n}'`); return this.args[n]; }),
      message: msg('message'), warning: msg('warning'), error: msg('error'), print_location: msg('print_location'),
      fatal: pyfn('fatal', (a) => { throw new XacroError(a.map(pyStr).join(' ')); }),
      tokenize: pyfn('tokenize', (a, kw) => {
        const s = pyStr(a[0]); const sep = a[1] !== undefined ? pyStr(a[1]) : kw.has('sep') ? pyStr(kw.get('sep')) : ',; ';
        const skip = a[2] !== undefined ? truthy(a[2]) : kw.has('skip_empty') ? truthy(kw.get('skip_empty')) : true;
        const parts = s.split(new RegExp(`[${sep.replace(/[\]\\^-]/g, '\\$&')}]`));
        return skip ? parts.filter((x) => x) : parts;
      }),
    };
    g.set('xacro', new PyNamespace('xacro', xacroNs));
    g.set('load_yaml', deprecated('load_yaml', 'xacro', loadYaml));
    g.set('abs_filename', deprecated('abs_filename', 'xacro', absFilename));
    g.set('dotify', deprecated('dotify', 'xacro', dotify));
    return g;
  }

  // ---------------------------------------------------------------- substitution args ($(...))
  collectArgs(s: string): string[] {
    const args: string[] = [];
    let state = 0, buf = '';
    for (const c of s) {
      if (c === '$') { if (state === 0) state = 1; else if (state === 1) { /* keep */ } else throw new PyError('SubstitutionException', `Dollar signs "$" cannot be inside of substitution args [${s}]`); }
      else if (c === '(') { if (state === 1) state = 2; else if (state !== 0) throw new PyError('SubstitutionException', `Invalid left parenthesis "(" in substitution args [${s}]`); }
      else if (c === ')') { if (state === 3) { args.push(buf); buf = ''; } state = 0; }
      else if (state === 1) state = 0;
      else if (state === 2) state = 3;
      if (state === 3) buf += c;
    }
    return args;
  }
  resolveArgs(s: string): string {
    if (!s) return s;
    if (s.startsWith('$(eval ') && s.endsWith(')')) return this.evalSubst(s.slice(7, -1));
    const valid = ['find', 'env', 'optenv', 'dirname', 'arg'];
    const pass = (str: string, cmds: string[]) => {
      let res = str;
      for (const a of this.collectArgs(str)) {
        const sp = a.split(' ').filter(Boolean);
        if (!valid.includes(sp[0])) throw new PyError('SubstitutionException', `Unknown substitution command [${a}]. Valid commands are ${JSON.stringify(valid).replace(/"/g, "'").replace(/,/g, ', ')}`);
        if (!cmds.includes(sp[0])) continue;
        const args = sp.slice(1);
        let val: string;
        switch (sp[0]) {
          case 'env':
            if (args.length !== 1) throw new PyError('SubstitutionException', `$(env var) command only accepts one argument [${a}]`);
            if (!this.opts.env || !(args[0] in this.opts.env)) throw new PyError('SubstitutionException', `environment variable ${args[0]} is not set`);
            val = this.opts.env[args[0]]; break;
          case 'optenv':
            if (!args.length) throw new PyError('SubstitutionException', `$(optenv var) must specify an environment variable [${a}]`);
            val = this.opts.env && args[0] in this.opts.env ? this.opts.env[args[0]] : args.slice(1).join(' '); break;
          case 'dirname':
            throw new PyError('SubstitutionException', 'Cannot substitute $(dirname),no file/directory information available.');
          case 'arg':
            if (!args.length) throw new PyError('SubstitutionException', `$(arg var) must specify a variable name [${a}]`);
            if (args.length > 1) throw new PyError('SubstitutionException', `$(arg var) may only specify one arg [${a}]`);
            if (!(args[0] in this.args)) {
              if (this.opts.lenient) { this.warn(`Undefined substitution argument ${args[0]}`); val = ''; break; }
              throw new XacroError(`Undefined substitution argument ${args[0]}`);
            }
            val = this.args[args[0]]; break;
          default: // find
            if (args.length !== 1) throw new PyError('SubstitutionException', `$(find pkg) accepts exactly one argument [${a}]`);
            val = this.findPackage(args[0]);
        }
        res = res.split(`$(${a})`).join(val);
      }
      return res;
    };
    return pass(pass(s, ['env', 'optenv', 'dirname', 'arg']), ['find']);
  }
  evalSubst(expr: string): string {
    if (expr.includes('__')) throw new PyError('SubstitutionException', '$(eval ...) may not contain double underscore expressions');
    const auto = (v: string): any => {
      try { return v.includes('.') ? pyFloat(v) : pyInt(v); } catch { /* not a number */ }
      const l = v.toLowerCase();
      if (l === 'true') return true;
      if (l === 'false') return false;
      return v;
    };
    const fns: Record<string, any> = {
      true: true, false: false, True: true, False: false,
      list: PY_BUILTINS.list, dict: PY_BUILTINS.dict, map: PY_BUILTINS.map, str: PY_BUILTINS.str, float: PY_BUILTINS.float, int: PY_BUILTINS.int,
      arg: pyfn('arg', (a) => { const n = pyStr(a[0]); if (!(n in this.args)) throw new XacroError(`Undefined substitution argument ${n}`); return auto(this.args[n]); }),
      env: pyfn('env', (a) => { const n = pyStr(a[0]); if (!this.opts.env || !(n in this.opts.env)) throw new PyError('SubstitutionException', `environment variable ${n} is not set`); return this.opts.env[n]; }),
      optenv: pyfn('optenv', (a) => { const n = pyStr(a[0]); return this.opts.env && n in this.opts.env ? this.opts.env[n] : a.length > 1 ? pyStr(a[1]) : ''; }),
      find: pyfn('find', (a) => this.findPackage(pyStr(a[0]))),
      dirname: pyfn('dirname', () => { throw new PyError('SubstitutionException', 'Cannot substitute $(dirname),no file/directory information available.'); }),
      ...PY_MATH,
    };
    const look = (n: string) => {
      if (Object.prototype.hasOwnProperty.call(fns, n)) return fns[n];
      if (n in this.args) return auto(this.args[n]);
      throw new PyError('NameError', `name '${n}' is not defined`);
    };
    return pyStr(pyEval(expr, look));
  }

  // ---------------------------------------------------------------- eval_text
  evalText(text: string, symbols: Table): any {
    const results: any[] = [];
    let s = text;
    while (s) {
      let hit: [string, string] | null = null;
      for (const [id, re] of LEX) { const m = re.exec(s); if (m) { hit = [id, m[0]]; break; } }
      if (!hit) this.fail(`invalid expression: ${s}`);
      s = s.slice(hit[1].length);
      const [id, tok] = hit;
      if (id === 'EXPR') results.push(this.handleExpr(tok.slice(2, -1), symbols, tok));
      else if (id === 'EXT') results.push(this.handleExt(tok.slice(2, -1), symbols, tok));
      else if (id === 'TEXT') results.push(tok);
      else results.push(tok.slice(1));
    }
    if (results.length === 1) return results[0];
    return results.map(pyStr).join('');
  }
  handleExpr(expr: string, symbols: Table, raw: string): any {
    return this.soft(() => {
      try {
        const src = pyStr(this.evalText(expr, symbols));
        return pyEval(src, (name) => {
          try { return symbols.get(name); } catch (e) { if (e instanceof KeyMiss) throw new PyError('NameError', `name '${name}' is not defined`); throw e; }
        });
      } catch (e) {
        if (e instanceof XacroError && !(e.cause instanceof PyError)) throw e;
        const m = e instanceof PyError ? (e.pyType === 'NameError' || e.pyType === 'KeyError' || e.pyType === 'SyntaxError' || e.pyType === 'TypeError' || e.pyType === 'ZeroDivisionError' ? e.message : `${e.pyType}: ${e.message}`) : (e as Error).message;
        this.fail(`${m} \nwhen evaluating expression '${expr}'`, e);
      }
    }, () => raw);
  }
  handleExt(inner: string, symbols: Table, raw: string): any {
    return this.soft(() => {
      const s = `$(${pyStr(this.evalText(inner, symbols))})`;
      try { return this.resolveArgs(s); } catch (e) {
        if (e instanceof XacroError) this.fail(e.message, e);
        this.fail(`${e instanceof PyError ? `${e.pyType}: ` : ''}${(e as Error).message}`, e);
      }
    }, () => raw);
  }

  boolValue(v: any, cond: string | undefined | null): boolean {
    if (typeof v === 'string') {
      if (v === 'true' || v === 'True') return true;
      if (v === 'false' || v === 'False') return false;
      if (/^\s*[+-]?\d(_?\d)*\s*$/.test(v)) return BigInt(v.trim().replace(/_/g, '')) !== 0n;
      return this.soft<boolean>(() => this.fail(`Xacro conditional "${cond}" evaluated to "${v}", which is not a boolean expression.`), () => v.trim() !== '' && v.trim() !== '0');
    }
    if (v === undefined || v === null) return false;
    return truthy(v);
  }

  // ---------------------------------------------------------------- eval_all
  evalAll(node: XNode, macros: Table, symbols: Table): void {
    for (const name of Object.keys(node.attrs)) {
      if (name.startsWith('xacro:')) delete node.attrs[name];
      else node.attrs[name] = pyStr(this.evalText(node.attrs[name], symbols));
    }
    delete node.attrs['xmlns:xacro'];
    const kids = node.children;
    let i = 0;
    while (i < kids.length) {
      const c = kids[i];
      if (c.type === 'text') { c.text = pyStr(this.evalText(c.text ?? '', symbols)); i++; continue; }
      const tag = c.name;
      if (tag === 'xacro:insert_block') {
        const name = c.attrs.name ?? '';
        let block: XNode | null = null, contentOnly = false;
        if (symbols.has(`**${name}`)) { block = symbols.get(`**${name}`); contentOnly = true; }
        else if (symbols.has(`*${name}`)) block = symbols.get(`*${name}`);
        else { this.soft(() => this.fail(`Undefined block "${name}"`), () => null); kids.splice(i, 1); continue; }
        const b = cloneNode(block!);
        this.evalAll(b, macros, symbols);
        const repl = contentOnly ? b.children : [b];
        kids.splice(i, 1, ...repl);
        i += repl.length;
      } else if (tag === 'xacro:include') {
        const repl = this.processInclude(c, macros, symbols);
        kids.splice(i, 1, ...repl);
        i += repl.length;
      } else if (tag === 'xacro:property') {
        kids.splice(i, 1);
        this.grabProperty(c, symbols);
      } else if (tag === 'xacro:macro') {
        kids.splice(i, 1);
        this.grabMacro(c, macros);
      } else if (tag === 'xacro:arg') {
        kids.splice(i, 1);
        const name = c.attrs.name;
        if (name === undefined || c.attrs.default === undefined) this.fail(`${tag}: missing attribute(s): ${['name', 'default'].filter((a) => c.attrs[a] === undefined).join(', ')}`);
        if (!(name in this.args)) this.args[name] = pyStr(this.evalText(c.attrs.default, symbols));
      } else if (tag === 'xacro:element') {
        const nm = c.attrs['xacro:name'];
        if (nm === undefined) this.fail('xacro:element: missing attribute(s): xacro:name');
        const name = pyStr(this.evalText(nm, symbols));
        if (!name) this.fail('xacro:element: empty name');
        delete c.attrs['xacro:name'];
        c.name = name;
        continue; // re-process with the new tag
      } else if (tag === 'xacro:attribute') {
        if (c.attrs.name === undefined || c.attrs.value === undefined) this.fail('xacro:attribute: missing attribute(s): name, value');
        const name = pyStr(this.evalText(c.attrs.name, symbols));
        const value = pyStr(this.evalText(c.attrs.value, symbols));
        if (!name) this.fail('xacro:attribute: empty name');
        node.attrs[name] = value;
        kids.splice(i, 1);
      } else if (tag === 'xacro:if' || tag === 'xacro:unless') {
        const cond = c.attrs.value;
        if (cond === undefined) this.fail(`${tag}: missing attribute(s): value`);
        let keep = this.boolValue(this.evalText(cond, symbols), cond);
        if (tag === 'xacro:unless') keep = !keep;
        if (keep) {
          this.evalAll(c, macros, symbols);
          kids.splice(i, 1, ...c.children);
          i += c.children.length;
        } else kids.splice(i, 1);
      } else if (tag === 'xacro:call' || tag.startsWith('xacro:')) {
        const repl = this.handleMacroCall(c, macros, symbols);
        if (repl === null) { kids.splice(i, 1); continue; }
        kids.splice(i, 1, ...repl);
        i += repl.length;
      } else {
        this.evalAll(c, macros, symbols);
        i++;
      }
    }
  }

  processInclude(elt: XNode, macros: Table, symbols: Table): XNode[] {
    const spec = elt.attrs.filename;
    if (spec === undefined) this.fail('xacro:include: missing attribute(s): filename');
    let nsMacros = macros, nsSymbols = symbols;
    if (elt.attrs.ns) {
      const ns = pyStr(this.evalText(elt.attrs.ns, symbols));
      nsMacros = new NameSpaceTable(this, null, new Map());
      macros.setItem(ns, nsMacros, true);
      nsSymbols = new NameSpaceTable(this, symbols, symbols.root);
      symbols.setItem(ns, nsSymbols, true);
    }
    const optional = elt.attrs.optional !== undefined ? this.boolValue(elt.attrs.optional, elt.attrs.optional) : false;
    if (elt.children.some((k) => k.type === 'element')) this.warn('Child elements of a <xacro:include> tag are ignored');
    const raw = pyStr(this.evalText(spec, symbols));
    const path = this.absFilename(raw);
    let files = [path];
    if (/[*[?]+/.test(path)) {
      files = this.opts.glob ? [...this.opts.glob(path)].sort() : [];
      if (!files.length) this.warn(`Include tag's filename spec "${path}" matched no files.`);
    }
    const out: XNode[] = [];
    for (const f of files) {
      this.includes.push(f);
      const text = this.readFile(f, raw);
      if (text === null) {
        if (optional) continue;
        if (this.opts.lenient) { this.warn(`No such file or directory: ${f}`); continue; }
        this.fail(`No such file or directory: ${f}`);
      }
      this.filestack.push(f);
      const root = parseXML(text);
      this.evalAll(root, nsMacros, nsSymbols);
      this.filestack.pop();
      out.push(...root.children);
    }
    return out;
  }

  grabProperty(elt: XNode, table: Table): void {
    const a = elt.attrs;
    if (a.name === undefined) this.fail('xacro:property: missing attribute(s): name');
    let name = pyStr(this.evalText(a.name, table));
    if (!isValidName(name)) this.fail(`Property names must be valid python identifiers: ${name}`);
    if (name.startsWith('__')) this.fail(`Property names must not start with double underscore:${name}`);
    const remove = this.boolValue(this.evalText(a.remove ?? 'false', table), a.remove);
    let value: any = a.value;
    if ([value !== undefined, a.default !== undefined, remove].filter(Boolean).length > 1) this.fail(`Property attributes default, value, and remove are mutually exclusive: ${name}`);
    if (remove && table.has(name)) { table.del(name); return; }
    if (a.default !== undefined) {
      if (a.scope !== undefined) this.warn(`${name}: default property value can only be defined on local scope`);
      if (!table.has(name)) value = a.default;
      else return;
    }
    if (value === undefined) { name = `**${name}`; value = elt; }
    let lazy = this.boolValue(this.evalText(a.lazy_eval ?? 'true', table), a.lazy_eval);
    let target: Table = table;
    if (a.scope === 'global') { target = table.top(); lazy = false; }
    else if (a.scope === 'parent') {
      if (!table.parent) { this.warn(`${name}: no parent scope at global scope `); return; }
      target = table.parent;
      lazy = false;
      if (!(table instanceof NameSpaceTable)) while (target instanceof NameSpaceTable && target.parent) target = target.parent;
    }
    if (!lazy && typeof value === 'string') value = this.evalText(value, table);
    target.setItem(name, value, lazy);
  }

  grabMacro(elt: XNode, macros: Table): void {
    let name = elt.attrs.name;
    if (name === undefined) this.fail('xacro:macro: missing attribute(s): name');
    if (name === 'call') this.fail("Invalid use of macro name 'call'");
    if (name.includes('.')) this.fail(`macro names must not contain '.' (reserved for namespaces): ${name}`);
    if (name.startsWith('xacro:')) { this.warn(`macro names must not contain prefix 'xacro:': ${name}`); name = name.slice(6); }
    const m: Macro = { name, params: [], defaults: new Map(), body: elt };
    let params = elt.attrs.params ?? '';
    while (params) {
      const [p, v, rest] = parseMacroArg(params);
      m.params.push(p);
      if (v) m.defaults.set(p, v);
      params = rest;
    }
    macros.setItem(name, m, true);
  }

  resolveMacro(fullname: string, macros: Table, symbols: Table): [Table, Table, Macro] {
    try { return [macros, symbols, macros.get(fullname)]; } catch (e) {
      if (!(e instanceof KeyMiss)) throw e;
      const nss = fullname.split('.');
      const name = nss.pop()!;
      if (!nss.length) throw e;
      let ms: any = macros, ss: any = symbols;
      for (const ns of nss) { ms = ms.get(ns); ss = ss.get(ns); if (!(ms instanceof Table) || !(ss instanceof Table)) throw new KeyMiss(ns); }
      return [ms, ss, ms.get(name)];
    }
  }

  handleMacroCall(node: XNode, macros: Table, symbols: Table): XNode[] | null {
    if (node.name === 'xacro:call') {
      const mn = node.attrs.macro;
      if (!mn) this.fail("xacro:call is missing the 'macro' attribute");
      const name = pyStr(this.evalText(mn, symbols));
      delete node.attrs.macro;
      node.name = `xacro:${name}`;
    }
    const name = node.name.slice(6);
    let resolved: [Table, Table, Macro];
    try { resolved = this.resolveMacro(name, macros, symbols); } catch (e) {
      if (!(e instanceof KeyMiss)) throw e;
      return this.soft<XNode[] | null>(() => this.fail(`unknown macro name: ${node.name}`), () => null);
    }
    const [sm, ss, m] = resolved;
    const body = cloneNode(m.body);
    const scopedSymbols = new Table(this, ss, ss.root);
    const scopedMacros = new Table(this, sm, sm.root);
    const params = [...m.params];
    const where = `\nwhen instantiating macro: ${m.name}`;
    for (const an of Object.keys(node.attrs)) {
      const k = params.indexOf(an);
      if (k < 0) this.fail(`Invalid parameter "${an}"${where}`);
      params.splice(k, 1);
      scopedSymbols.setItem(an, this.evalText(node.attrs[an], symbols), false);
      node.attrs[an] = '';
    }
    this.evalAll(node, macros, symbols);
    const elems = node.children.filter((x) => x.type === 'element');
    let bi = 0;
    for (const p of [...params]) {
      if (p[0] !== '*') continue;
      if (bi >= elems.length) this.fail(`Not enough blocks${where}`);
      params.splice(params.indexOf(p), 1);
      scopedSymbols.setItem(p, elems[bi++], true);
    }
    if (bi < elems.length) this.fail(`Unused block "${elems[bi].name}"${where}`);
    for (const p of [...params]) {
      if (p[0] === '*') continue;
      const [fwd, def] = m.defaults.get(p) ?? [null, null];
      if (fwd !== null || def !== null) {
        let v: any;
        if (fwd === null) v = this.evalText(def!, symbols);
        else {
          try { v = symbols.get(fwd); } catch (e) {
            if (!(e instanceof KeyMiss)) throw e;
            if (def !== null) v = this.evalText(def, symbols);
            else this.fail(`Undefined property to forward: ${fwd}${where}`);
          }
        }
        scopedSymbols.setItem(p, v, false);
        params.splice(params.indexOf(p), 1);
      }
    }
    if (params.length) this.fail(`Undefined parameters [${params.join(',')}]${where}`);
    this.evalAll(body, scopedMacros, scopedSymbols);
    return body.children;
  }
}

function toFloat(v: any): number {
  if (typeof v === 'number') return v;
  if (typeof v === 'bigint') return Number(v);
  if (typeof v === 'boolean') return +v;
  const n = Number(v);
  if (Number.isNaN(n)) throw new PyError('ValueError', `could not convert string to float: '${v}'`);
  return n;
}

function yamlToPy(v: YamlValue | unknown): any {
  if (v instanceof Map) return new PyDict([...v.entries()].map(([k, x]) => [typeof k === 'number' && Number.isInteger(k) ? BigInt(k) : k, yamlToPy(x)] as [any, any]), true);
  if (Array.isArray(v)) return v.map(yamlToPy);
  if (v === undefined) return null;
  return v;
}

export function cloneNode(n: XNode): XNode {
  return { ...n, attrs: { ...n.attrs }, children: (n.children ?? []).map(cloneNode) };
}

/** Expand a xacro document (ROS 2 semantics). Throws XacroError unless `lenient`. */
export function expandXacro(xml: string, opts: XacroOptions = {}): XacroResult {
  const ctx = new Ctx(opts);
  const root = parseXML(xml);
  const symbols = new Table(ctx, null, ctx.globals);
  const macros = new Table(ctx, null, new Map());
  const target = root.attrs['xacro:targetNamespace'];
  if (target) { delete root.attrs['xacro:targetNamespace']; root.attrs.xmlns = target; }
  try {
    ctx.evalAll(root, macros, symbols);
  } catch (e) {
    if (e instanceof XacroError) throw e;
    if (e instanceof KeyMiss) throw new XacroError(`name '${e.message}' is not defined`, opts.filename, e);
    throw new XacroError((e as Error).message, opts.filename, e);
  }
  return { xml: serializeXML(root), root, includes: ctx.includes, warnings: ctx.warnings };
}

/** Expand a xacro document and return the URDF/XML text. */
export function processXacro(xml: string, opts: XacroOptions = {}): string {
  return expandXacro(xml, opts).xml;
}

/** Does this text use xacro (namespace, xacro: tags or ${} / $() substitutions)? */
export function isXacro(text: string): boolean {
  return /xmlns:xacro|<xacro:|\$\{|\$\((arg|find|eval|env|optenv)\s/.test(text);
}

export { PyTuple, iterate, recordLookup };
