/**
 * Shared helpers of the control-code generators: expression translation (the control DSL expression language →
 * Python / IEC 61131-3 ST / NuSMV / SCXML ECMAScript), identifiers, XML writing and a small XML reader (PNML import,
 * well-formedness checks of the generated documents).
 */
export interface GenFile { name: string; content: string; mime?: string }

export const pyStr = (s: string) => JSON.stringify(s);
/** Identifier for C / Python / ST / SMV: letters, digits, underscore, not starting with a digit. */
export const ident = (s: string, prefix = 'x') => { const r = s.replace(/[^A-Za-z0-9_]/g, '_'); return /^[A-Za-z_]/.test(r) ? r : `${prefix}_${r}`; };
export const xmlEsc = (s: string) => s.replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;').replace(/"/g, '&quot;');

type Tok = { k: 'num' | 'str' | 'id' | 'op' | 'lp' | 'rp' | 'comma'; v: string };
export function exprTokens(src: string): Tok[] {
  const out: Tok[] = []; let i = 0;
  while (i < src.length) {
    const c = src[i];
    if (/\s/.test(c)) { i++; continue; }
    if (/[0-9.]/.test(c) && /[0-9]/.test(src[i + 1] ?? c)) { let j = i; while (j < src.length && /[0-9.eE]/.test(src[j])) j++; out.push({ k: 'num', v: src.slice(i, j) }); i = j; continue; }
    if (c === "'" || c === '"') { const j = src.indexOf(c, i + 1); if (j < 0) throw new Error(`unterminated string in ${src}`); out.push({ k: 'str', v: src.slice(i + 1, j) }); i = j + 1; continue; }
    if (/[A-Za-z_]/.test(c)) { let j = i; while (j < src.length && /[A-Za-z0-9_.]/.test(src[j])) j++; out.push({ k: 'id', v: src.slice(i, j) }); i = j; continue; }
    const two = src.slice(i, i + 2);
    if (['&&', '||', '==', '!=', '<=', '>=', '->'].includes(two)) { out.push({ k: 'op', v: two }); i += 2; continue; }
    if ('+-*/%<>!=?:'.includes(c)) { out.push({ k: 'op', v: c }); i++; continue; }
    if (c === '(') { out.push({ k: 'lp', v: c }); i++; continue; }
    if (c === ')') { out.push({ k: 'rp', v: c }); i++; continue; }
    if (c === ',') { out.push({ k: 'comma', v: c }); i++; continue; }
    throw new Error(`unexpected '${c}' in expression ${src}`);
  }
  return out;
}
const FUNCS = ['abs', 'min', 'max', 'sqrt', 'hypot', 'sin', 'cos', 'exp', 'log', 'floor', 'ceil', 'sign'];

export interface ExprDialect { id(name: string): string; str(s: string): string; ops: Record<string, string>; fn(name: string): string; bool: [string, string] }
export const PY_DIALECT: ExprDialect = { id: (n) => `V(${pyStr(n)})`, str: (s) => pyStr(s), ops: { '&&': ' and ', '||': ' or ', '!': ' not ', '=': ' == ', '==': ' == ', '!=': ' != ', '->': ' <= ' }, fn: (n) => (['abs', 'min', 'max'].includes(n) ? n : n === 'sign' ? '_sign' : `math.${n}`), bool: ['True', 'False'] };
export const ST_DIALECT = (idOf: (n: string) => string): ExprDialect => ({ id: idOf, str: (s) => `'${s}'`, ops: { '&&': ' AND ', '||': ' OR ', '!': 'NOT ', '=': ' = ', '==': ' = ', '!=': ' <> ', '%': ' MOD ' }, fn: (n) => n.toUpperCase(), bool: ['TRUE', 'FALSE'] });
export const SMV_DIALECT = (idOf: (n: string) => string): ExprDialect => ({ id: idOf, str: (s) => ident(s), ops: { '&&': ' & ', '||': ' | ', '!': '!', '=': ' = ', '==': ' = ', '!=': ' != ', '->': ' -> ', '%': ' mod ' }, fn: (n) => n, bool: ['TRUE', 'FALSE'] });
export const ECMA_DIALECT: ExprDialect = { id: (n) => n.replace(/\./g, '_'), str: (s) => `'${s}'`, ops: { '=': ' == ', '==': ' == ', '->': ' <= ' }, fn: (n) => `Math.${n}`, bool: ['true', 'false'] };

/** Translate a DSL expression to a target dialect (the ternary operator is not supported by the generators). */
export function translateExpr(src: string, d: ExprDialect): string {
  const t = exprTokens(src); let out = '';
  for (let i = 0; i < t.length; i++) {
    const x = t[i];
    if (x.k === 'num') out += x.v;
    else if (x.k === 'str') out += d.str(x.v);
    else if (x.k === 'id') {
      const low = x.v.toLowerCase();
      if (low === 'true') out += d.bool[0]; else if (low === 'false') out += d.bool[1];
      else if (low === 'and') out += d.ops['&&'] ?? ' && '; else if (low === 'or') out += d.ops['||'] ?? ' || '; else if (low === 'not') out += d.ops['!'] ?? '!';
      else if (FUNCS.includes(x.v) && t[i + 1]?.k === 'lp') out += d.fn(x.v);
      else out += d.id(x.v);
    } else if (x.k === 'op') { if (x.v === '?' || x.v === ':') throw new Error(`the ternary operator is not supported in generated code: ${src}`); out += d.ops[x.v] ?? ` ${x.v} `; }
    else out += x.v === ',' ? ', ' : x.v;
  }
  return out.replace(/\s+/g, ' ').trim();
}
/** Identifiers read by an expression. */
export function exprIds(src: string): string[] { try { return [...new Set(exprTokens(src).filter((x, i, a) => x.k === 'id' && !['true', 'false', 'and', 'or', 'not'].includes(x.v.toLowerCase()) && !(FUNCS.includes(x.v) && a[i + 1]?.k === 'lp')).map((x) => x.v))]; } catch { return []; } }

// ---------------------------------------------------------------------------------------------
// Minimal XML reader (well-formed documents: elements, attributes, text, CDATA, comments, PI, DOCTYPE)
// ---------------------------------------------------------------------------------------------

export interface XmlNode { name: string; attrs: Record<string, string>; children: XmlNode[]; text: string }
const unesc = (s: string) => s.replace(/&lt;/g, '<').replace(/&gt;/g, '>').replace(/&quot;/g, '"').replace(/&apos;/g, "'").replace(/&#(\d+);/g, (_, n) => String.fromCharCode(Number(n))).replace(/&#x([0-9a-f]+);/gi, (_, n) => String.fromCharCode(parseInt(n, 16))).replace(/&amp;/g, '&');
/** Parse an XML document; throws on malformed input (unbalanced tags, bad attributes, junk after the root). */
export function parseXml(src: string): XmlNode {
  let i = 0; const root: XmlNode = { name: '#document', attrs: {}, children: [], text: '' }; const stack: XmlNode[] = [root];
  const err = (m: string) => { const line = src.slice(0, i).split('\n').length; throw new Error(`XML line ${line}: ${m}`); };
  while (i < src.length) {
    if (src.startsWith('<?', i)) { const j = src.indexOf('?>', i); if (j < 0) err('unterminated processing instruction'); i = j + 2; continue; }
    if (src.startsWith('<!--', i)) { const j = src.indexOf('-->', i); if (j < 0) err('unterminated comment'); i = j + 3; continue; }
    if (src.startsWith('<![CDATA[', i)) { const j = src.indexOf(']]>', i); if (j < 0) err('unterminated CDATA'); stack[stack.length - 1].text += src.slice(i + 9, j); i = j + 3; continue; }
    if (src.startsWith('<!', i)) { let depth = 0, j = i; for (; j < src.length; j++) { if (src[j] === '[') depth++; else if (src[j] === ']') depth--; else if (src[j] === '>' && depth <= 0) break; } i = j + 1; continue; }
    if (src.startsWith('</', i)) {
      const j = src.indexOf('>', i); if (j < 0) err('unterminated end tag'); const name = src.slice(i + 2, j).trim();
      const top = stack.pop(); if (!top || top.name !== name) err(`end tag </${name}> does not match <${top?.name}>`); i = j + 1; continue;
    }
    if (src[i] === '<') {
      const m = /^<([A-Za-z_][\w.:-]*)/.exec(src.slice(i, i + 200)); if (!m) err('bad tag name');
      const node: XmlNode = { name: m![1], attrs: {}, children: [], text: '' }; i += m![0].length;
      for (;;) {
        while (/\s/.test(src[i])) i++;
        if (src.startsWith('/>', i)) { i += 2; stack[stack.length - 1].children.push(node); break; }
        if (src[i] === '>') { i++; stack[stack.length - 1].children.push(node); stack.push(node); break; }
        const a = /^([A-Za-z_][\w.:-]*)\s*=\s*("([^"]*)"|'([^']*)')/.exec(src.slice(i, i + 4000)); if (!a) err(`bad attribute in <${node.name}>`);
        if (a![1] in node.attrs) err(`duplicate attribute ${a![1]}`);
        node.attrs[a![1]] = unesc(a![3] ?? a![4]); i += a![0].length;
      }
      continue;
    }
    const j = src.indexOf('<', i); const txt = src.slice(i, j < 0 ? src.length : j);
    if (stack.length === 1 && txt.trim()) err('text outside the root element');
    if (/&(?!(lt|gt|amp|quot|apos|#\d+|#x[0-9a-fA-F]+);)/.test(txt)) err('unescaped &');
    stack[stack.length - 1].text += unesc(txt); i = j < 0 ? src.length : j;
  }
  if (stack.length !== 1) err(`unclosed <${stack[stack.length - 1].name}>`);
  if (root.children.length !== 1) err(`${root.children.length} root elements`);
  return root.children[0];
}
export const xmlFind = (n: XmlNode, name: string): XmlNode | undefined => n.children.find((c) => c.name === name || c.name.endsWith(`:${name}`));
export const xmlAll = (n: XmlNode, name: string): XmlNode[] => { const out: XmlNode[] = []; const walk = (x: XmlNode) => { for (const c of x.children) { if (c.name === name || c.name.endsWith(`:${name}`)) out.push(c); walk(c); } }; walk(n); return out; };
