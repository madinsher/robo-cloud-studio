/**
 * Shared text-DSL helpers (tokenizer, line splitter, error type) used by the control-design DSL (ctl/dsl.ts) and the
 * group-control DSL (mrs/dsl.ts).
 */
export type Opts = Record<string, string | number | boolean>;

/** Split a line into tokens honouring quotes; returns words and key=value options. */
export function tokenize(line: string): { words: string[]; opts: Opts; quoted: string[] } {
  const words: string[] = []; const opts: Opts = {}; const quoted: string[] = [];
  // key="quoted value" | "quoted" | 'quoted' | bare word (bare key=value pairs are split below)
  const re = /([A-Za-z_][\w.]*)=(?:"([^"]*)"|'([^']*)')|"([^"]*)"|'([^']*)'|(\S+)/g; let m: RegExpExecArray | null;
  const parts: Array<{ v: string; q: boolean; key?: string }> = [];
  while ((m = re.exec(line))) {
    if (m[1] !== undefined) parts.push({ v: m[2] ?? m[3], q: true, key: m[1] });
    else if (m[4] !== undefined) parts.push({ v: m[4], q: true }); else if (m[5] !== undefined) parts.push({ v: m[5], q: true }); else parts.push({ v: m[6], q: false });
  }
  for (let i = 0; i < parts.length; i++) {
    const p = parts[i];
    if (p.key) { opts[p.key] = p.v; continue; }
    const kv = !p.q && /^([A-Za-z_][\w.]*)=(.*)$/.exec(p.v);
    if (kv) { let val: string = kv[2]; if (val === '' && parts[i + 1]) val = parts[++i].v; opts[kv[1]] = coerce(val, parts[i]?.q); }
    else if (p.q) { quoted.push(p.v); words.push(p.v); } else words.push(p.v);
  }
  return { words, opts, quoted };
}
function coerce(v: string, quoted = false): string | number | boolean { if (quoted) return v; const q = /^"(.*)"$|^'(.*)'$/.exec(v); if (q) return q[1] ?? q[2]; if (v === 'true') return true; if (v === 'false') return false; if (/^-?\d+(\.\d+)?([eE][+-]?\d+)?$/.test(v)) return Number(v); return v; }

export function lines(src: string): Array<{ n: number; text: string; indent: number }> {
  return src.split(/\r?\n/).map((raw, i) => ({ n: i + 1, text: raw.replace(/(^|\s)(#|\/\/).*$/, '').replace(/\s+$/, ''), indent: raw.search(/\S|$/) })).filter((l) => l.text.trim().length > 0);
}

export class DslError extends Error { constructor(msg: string, readonly line?: number) { super(line ? `line ${line}: ${msg}` : msg); } }

