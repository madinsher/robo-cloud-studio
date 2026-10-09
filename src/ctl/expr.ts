/**
 * Tiny expression language shared by guards, invariants, STL predicates and GR(1) atoms.
 *
 *   numbers, identifiers (dots allowed: robot.v), strings 'idle', + - * / %, comparisons (< <= > >= == !=, and
 *   `=` as equality), boolean ! && || (also `and`, `or`, `not`), ternary a ? b : c, functions abs min max sqrt hypot
 *   sin cos exp log floor ceil, and a prime suffix (`x'`) which reads the variable from `next` when provided.
 */
export type Value = number | boolean | string;
export type Env = Record<string, Value | undefined>;

type Node =
  | { k: 'num'; v: number } | { k: 'str'; v: string } | { k: 'var'; name: string; next: boolean }
  | { k: 'un'; op: string; a: Node } | { k: 'bin'; op: string; a: Node; b: Node } | { k: 'cond'; c: Node; a: Node; b: Node }
  | { k: 'call'; fn: string; args: Node[] };

const FUNCS: Record<string, (...a: number[]) => number> = { abs: Math.abs, min: Math.min, max: Math.max, sqrt: Math.sqrt, hypot: Math.hypot, sin: Math.sin, cos: Math.cos, exp: Math.exp, log: Math.log, floor: Math.floor, ceil: Math.ceil, sign: Math.sign };

export class Expr {
  private constructor(private readonly root: Node, readonly source: string, readonly vars: string[]) {}
  static parse(src: string): Expr { const p = new Parser(src); const root = p.parseExpr(); p.expectEnd(); return new Expr(root, src, [...p.vars]); }
  eval(env: Env, next?: Env): Value { return evalNode(this.root, env, next); }
  bool(env: Env, next?: Env): boolean { return truthy(this.eval(env, next)); }
  num(env: Env, next?: Env): number { const v = this.eval(env, next); return typeof v === 'number' ? v : v === true ? 1 : v === false ? 0 : Number(v); }
  /** Signed margin of a comparison (robustness): a >= b → a − b; a <= b → b − a; boolean combos via min/max. */
  robustness(env: Env, next?: Env): number { return robust(this.root, env, next); }
}

const cache = new Map<string, Expr>();
export function expr(src: string): Expr { let e = cache.get(src); if (!e) { e = Expr.parse(src); cache.set(src, e); } return e; }
export function evalExpr(src: string, env: Env, next?: Env): Value { return expr(src).eval(env, next); }
export function truthy(v: Value | undefined): boolean { return v === true || (typeof v === 'number' && v !== 0) || (typeof v === 'string' && v !== '' && v !== 'false'); }

function evalNode(n: Node, env: Env, next?: Env): Value {
  switch (n.k) {
    case 'num': return n.v;
    case 'str': return n.v;
    case 'var': { const src = n.next ? next ?? env : env; const v = src[n.name]; if (v === undefined) { if (n.name === 'true') return true; if (n.name === 'false') return false; return 0; } return v; }
    case 'un': { const a = evalNode(n.a, env, next); return n.op === '!' ? !truthy(a) : -(a as number); }
    case 'cond': return truthy(evalNode(n.c, env, next)) ? evalNode(n.a, env, next) : evalNode(n.b, env, next);
    case 'call': { const f = FUNCS[n.fn]; if (!f) throw new Error(`unknown function ${n.fn}`); return f(...n.args.map((a) => Number(evalNode(a, env, next)))); }
    case 'bin': {
      if (n.op === '&&') return truthy(evalNode(n.a, env, next)) && truthy(evalNode(n.b, env, next));
      if (n.op === '||') return truthy(evalNode(n.a, env, next)) || truthy(evalNode(n.b, env, next));
      if (n.op === '->') return !truthy(evalNode(n.a, env, next)) || truthy(evalNode(n.b, env, next));
      const a = evalNode(n.a, env, next), b = evalNode(n.b, env, next);
      switch (n.op) {
        case '+': return (a as number) + (b as number); case '-': return (a as number) - (b as number); case '*': return (a as number) * (b as number); case '/': return (a as number) / (b as number); case '%': return (a as number) % (b as number);
        case '<': return a < b; case '<=': return a <= b; case '>': return a > b; case '>=': return a >= b;
        case '==': return a === b || String(a) === String(b); case '!=': return !(a === b || String(a) === String(b));
      }
      throw new Error(`bad operator ${n.op}`);
    }
  }
}

function robust(n: Node, env: Env, next?: Env): number {
  if (n.k === 'bin') {
    const cmp = n.op;
    if (cmp === '>=' || cmp === '>') return Number(evalNode(n.a, env, next)) - Number(evalNode(n.b, env, next));
    if (cmp === '<=' || cmp === '<') return Number(evalNode(n.b, env, next)) - Number(evalNode(n.a, env, next));
    if (cmp === '&&') return Math.min(robust(n.a, env, next), robust(n.b, env, next));
    if (cmp === '||') return Math.max(robust(n.a, env, next), robust(n.b, env, next));
    if (cmp === '->') return Math.max(-robust(n.a, env, next), robust(n.b, env, next));
    if (cmp === '==') return truthy(evalNode(n, env, next)) ? 1 : -1;
    if (cmp === '!=') return truthy(evalNode(n, env, next)) ? 1 : -1;
  }
  if (n.k === 'un' && n.op === '!') return -robust(n.a, env, next);
  const v = evalNode(n, env, next);
  return typeof v === 'boolean' ? (v ? 1 : -1) : Number(v);
}

class Parser {
  private i = 0; readonly vars = new Set<string>();
  constructor(private readonly s: string) {}
  private ws() { while (this.i < this.s.length && /\s/.test(this.s[this.i])) this.i++; }
  private peek(re: RegExp): string | null { this.ws(); const m = re.exec(this.s.slice(this.i)); return m && m.index === 0 ? m[0] : null; }
  private take(re: RegExp): string | null { const t = this.peek(re); if (t !== null) this.i += t.length; return t; }
  expectEnd(): void { this.ws(); if (this.i < this.s.length) throw new Error(`unexpected '${this.s.slice(this.i)}' in expression "${this.s}"`); }
  parseExpr(): Node { return this.parseCond(); }
  private parseCond(): Node { const c = this.parseImpl(); if (this.take(/^\?/)) { const a = this.parseCond(); if (!this.take(/^:/)) throw new Error("expected ':'"); const b = this.parseCond(); return { k: 'cond', c, a, b }; } return c; }
  private parseImpl(): Node { let a = this.parseOr(); while (this.take(/^(->|=>)/)) { const b = this.parseOr(); a = { k: 'bin', op: '->', a, b }; } return a; }
  private parseOr(): Node { let a = this.parseAnd(); while (this.take(/^(\|\||\bor\b|\|)/)) { const b = this.parseAnd(); a = { k: 'bin', op: '||', a, b }; } return a; }
  private parseAnd(): Node { let a = this.parseNot(); while (this.take(/^(&&|\band\b|&)/)) { const b = this.parseNot(); a = { k: 'bin', op: '&&', a, b }; } return a; }
  private parseNot(): Node { if (this.take(/^(!(?!=)|\bnot\b)/)) return { k: 'un', op: '!', a: this.parseNot() }; return this.parseCmp(); }
  private parseCmp(): Node { let a = this.parseAdd(); const op = this.take(/^(<=|>=|==|!=|<|>|=(?!>))/); if (op) { const b = this.parseAdd(); a = { k: 'bin', op: op === '=' ? '==' : op, a, b }; } return a; }
  private parseAdd(): Node { let a = this.parseMul(); let op: string | null; while ((op = this.take(/^(\+|-(?!>))/))) { const b = this.parseMul(); a = { k: 'bin', op, a, b }; } return a; }
  private parseMul(): Node { let a = this.parseUnary(); let op: string | null; while ((op = this.take(/^[*/%]/))) { const b = this.parseUnary(); a = { k: 'bin', op, a, b }; } return a; }
  private parseUnary(): Node { if (this.take(/^-/)) return { k: 'un', op: '-', a: this.parseUnary() }; return this.parsePrimary(); }
  private parsePrimary(): Node {
    const num = this.take(/^\d+(\.\d+)?([eE][+-]?\d+)?/); if (num) return { k: 'num', v: parseFloat(num) };
    const str = this.take(/^'[^']*'|^"[^"]*"/); if (str) return { k: 'str', v: str.slice(1, -1) };
    if (this.take(/^\(/)) { const e = this.parseExpr(); if (!this.take(/^\)/)) throw new Error("expected ')'"); return e; }
    const id = this.take(/^[A-Za-z_][\w.]*'?/);
    if (id) {
      if (this.peek(/^\(/) && FUNCS[id]) { this.take(/^\(/); const args: Node[] = []; if (!this.peek(/^\)/)) { do args.push(this.parseExpr()); while (this.take(/^,/)); } if (!this.take(/^\)/)) throw new Error("expected ')'"); return { k: 'call', fn: id, args }; }
      if (id === 'true') return { k: 'num', v: 1 }; if (id === 'false') return { k: 'num', v: 0 };
      const next = id.endsWith("'"); const name = next ? id.slice(0, -1) : id; this.vars.add(name);
      return { k: 'var', name, next };
    }
    throw new Error(`unexpected '${this.s.slice(this.i, this.i + 10)}' in expression "${this.s}"`);
  }
}
