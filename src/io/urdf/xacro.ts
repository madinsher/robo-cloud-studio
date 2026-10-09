/**
 * Minimal xacro preprocessor: properties, ${} expressions, macros (with params and blocks),
 * xacro:if / xacro:unless, xacro:include (via resolver). Works on DOM in browser and via a
 * tiny XML parser fallback for Node (see xml.ts).
 */
import { XNode, parseXML, serializeXML } from './xml';

export interface XacroOptions {
  /** Resolve an include filename to text. */
  resolveInclude?: (filename: string) => string | null;
  /** Command-line style args. */
  args?: Record<string, string>;
}

type Scope = Map<string, any>;

/** xacro strips matching quotes from string literals on lookup (`arm_id:='panda'` -> panda). */
function literal(v: any): any {
  if (typeof v === 'string') {
    const m = /^\s*(['"])(.*)\1\s*$/s.exec(v);
    if (m) return m[2];
  }
  return v;
}

function evalExpr(expr: string, scope: Scope): any {
  // Replace known identifiers with scope values; support pi, radians(), degrees(), math functions.
  const names: string[] = [];
  const vals: any[] = [];
  for (const [k, v] of scope) {
    if (/^[A-Za-z_][A-Za-z0-9_]*$/.test(k)) {
      names.push(k);
      vals.push(literal(v));
    }
  }
  const helpers = {
    pi: Math.PI,
    radians: (d: number) => (d * Math.PI) / 180,
    degrees: (r: number) => (r * 180) / Math.PI,
    sin: Math.sin, cos: Math.cos, tan: Math.tan, atan2: Math.atan2, sqrt: Math.sqrt, abs: Math.abs, pow: Math.pow, floor: Math.floor, ceil: Math.ceil, min: Math.min, max: Math.max,
    True: true, False: false,
  };
  const hNames = Object.keys(helpers).filter((n) => !names.includes(n));
  const hVals = hNames.map((n) => (helpers as any)[n]);
  // python-ish operators
  let js = expr.replace(/\band\b/g, '&&').replace(/\bor\b/g, '||').replace(/\bnot\b/g, '!').replace(/\*\*/g, '**');
  try {
    // eslint-disable-next-line no-new-func
    const fn = new Function(...names, ...hNames, `"use strict"; return (${js});`);
    return fn(...vals, ...hVals);
  } catch {
    return expr;
  }
}

function substitute(text: string, scope: Scope): string {
  if (!text.includes('${') && !text.includes('$(')) return text;
  let out = text.replace(/\$\(arg\s+([^)]+)\)/g, (_m, a) => String(scope.get(`arg:${a.trim()}`) ?? ''));
  out = out.replace(/\$\(find\s+([^)]+)\)/g, (_m, p) => `package://${p.trim()}`);
  let guard = 0;
  while (out.includes('${') && guard++ < 20) {
    out = out.replace(/\$\{([^{}]*)\}/g, (_m, e) => {
      const v = evalExpr(e.trim(), scope);
      return typeof v === 'number' ? String(v) : String(v);
    });
  }
  return out;
}

function truthy(v: any): boolean {
  if (typeof v === 'string') {
    const s = v.trim().toLowerCase();
    if (s === 'false' || s === '0' || s === '') return false;
    if (s === 'true' || s === '1') return true;
    const n = Number(s);
    return isNaN(n) ? true : n !== 0;
  }
  return !!v;
}

interface Macro {
  params: string[];
  defaults: Map<string, string>;
  body: XNode[];
}

export function processXacro(xml: string, opts: XacroOptions = {}): string {
  const root = parseXML(xml);
  const scope: Scope = new Map();
  for (const [k, v] of Object.entries(opts.args ?? {})) scope.set(`arg:${k}`, v);
  const macros = new Map<string, Macro>();
  const isX = (n: XNode, local: string) => n.name === `xacro:${local}` || n.name === local;

  const expand = (nodes: XNode[], scope: Scope): XNode[] => {
    const out: XNode[] = [];
    for (const n of nodes) {
      if (n.type === 'text') {
        out.push({ ...n, text: substitute(n.text ?? '', scope) });
        continue;
      }
      if (isX(n, 'arg')) {
        const name = n.attrs.name;
        if (!scope.has(`arg:${name}`)) scope.set(`arg:${name}`, substitute(n.attrs.default ?? '', scope));
        continue;
      }
      if (isX(n, 'property')) {
        const name = n.attrs.name;
        if (n.attrs.value !== undefined) {
          const raw = substitute(n.attrs.value, scope);
          const num = Number(raw);
          scope.set(name, raw.trim() !== '' && !isNaN(num) ? num : raw);
        } else scope.set(name, n.children);
        continue;
      }
      if (isX(n, 'include')) {
        const file = substitute(n.attrs.filename ?? '', scope);
        const text = opts.resolveInclude?.(file);
        if (text) {
          const inc = parseXML(text);
          out.push(...expand(inc.children, scope));
        }
        continue;
      }
      if (isX(n, 'macro')) {
        const params = (n.attrs.params ?? '').split(/\s+/).filter(Boolean);
        const defaults = new Map<string, string>();
        const names = params.map((p) => {
          const [nm, def] = p.split(':=');
          if (def !== undefined) defaults.set(nm, def);
          return nm;
        });
        macros.set(n.attrs.name, { params: names, defaults, body: n.children });
        continue;
      }
      if (isX(n, 'if') || isX(n, 'unless')) {
        const v = truthy(evalExpr(substitute(n.attrs.value ?? '', scope), scope));
        if (v === isX(n, 'if')) out.push(...expand(n.children, scope));
        continue;
      }
      const macroName = n.name.startsWith('xacro:') ? n.name.slice(6) : n.name;
      const macro = macros.get(macroName);
      if (macro) {
        const local: Scope = new Map(scope);
        for (const p of macro.params) {
          const isBlock = p.startsWith('*');
          const pname = isBlock ? p.slice(1) : p;
          if (isBlock) {
            const blk = n.children.find((c) => c.type === 'element');
            local.set(pname, blk ? [blk] : []);
            if (blk) n.children.splice(n.children.indexOf(blk), 1);
          } else if (n.attrs[pname] !== undefined) local.set(pname, coerce(substitute(n.attrs[pname], scope)));
          else if (macro.defaults.has(pname)) local.set(pname, coerce(substitute(macro.defaults.get(pname)!, scope)));
        }
        out.push(...expand(cloneNodes(macro.body), local));
        continue;
      }
      if (isX(n, 'insert_block')) {
        const blk = scope.get(n.attrs.name);
        if (Array.isArray(blk)) out.push(...expand(cloneNodes(blk), scope));
        continue;
      }
      const attrs: Record<string, string> = {};
      for (const [k, v] of Object.entries(n.attrs)) if (!k.startsWith('xmlns:xacro')) attrs[k] = substitute(v, scope);
      out.push({ type: 'element', name: n.name, attrs, children: expand(n.children, scope) });
    }
    return out;
  };

  const result: XNode = { type: 'element', name: root.name, attrs: {}, children: [] };
  const expanded = expand([root], scope);
  return serializeXML(expanded[0] ?? result);
}

function coerce(s: string): any {
  const n = Number(s);
  return s.trim() !== '' && !isNaN(n) ? n : s;
}

function cloneNodes(nodes: XNode[]): XNode[] {
  return nodes.map((n) => ({ ...n, attrs: { ...n.attrs }, children: cloneNodes(n.children ?? []) }));
}
