/**
 * Small YAML subset reader for robot configuration files (ROS parameter / layout files such as MURKA products.yaml and
 * locations.yaml): block mappings and sequences by indentation, flow mappings / sequences, plain / quoted scalars
 * (numbers, booleans, null), comments. Anchors, tags and multi-line scalars are not supported (reported as errors).
 */

export type YamlValue = string | number | boolean | null | YamlValue[] | { [k: string]: YamlValue };

function stripComment(line: string): string {
  let q: string | null = null;
  for (let i = 0; i < line.length; i++) {
    const c = line[i];
    if (q) { if (c === q && line[i - 1] !== '\\') q = null; continue; }
    if (c === '"' || c === "'") q = c;
    else if (c === '#' && (i === 0 || /\s/.test(line[i - 1]))) return line.slice(0, i);
  }
  return line;
}

function scalar(s: string): YamlValue {
  const t = s.trim();
  if (t === '' || t === '~' || t === 'null' || t === 'Null' || t === 'NULL') return null;
  if (/^(true|True|TRUE)$/.test(t)) return true;
  if (/^(false|False|FALSE)$/.test(t)) return false;
  if (/^[-+]?(\d+\.?\d*|\.\d+)([eE][-+]?\d+)?$/.test(t)) return Number(t);
  if (/^0x[0-9a-fA-F]+$/.test(t)) return parseInt(t, 16);
  if ((t.startsWith('"') && t.endsWith('"')) || (t.startsWith("'") && t.endsWith("'"))) return t.slice(1, -1).replace(/\\"/g, '"').replace(/''/g, "'");
  return t;
}

/** Parse a flow value: {a: 1, b: [1, 2]} / [x, y] / scalar. */
function flow(src: string): YamlValue {
  let i = 0;
  const ws = () => { while (i < src.length && /\s/.test(src[i])) i++; };
  const value = (): YamlValue => {
    ws();
    if (src[i] === '{') {
      i++;
      const o: { [k: string]: YamlValue } = {};
      for (;;) {
        ws();
        if (src[i] === '}') { i++; return o; }
        let key = '';
        if (src[i] === '"' || src[i] === "'") { const q = src[i++]; while (i < src.length && src[i] !== q) key += src[i++]; i++; }
        else while (i < src.length && src[i] !== ':' && src[i] !== ',' && src[i] !== '}') key += src[i++];
        ws();
        if (src[i] === ':') { i++; o[key.trim()] = value(); } else o[key.trim()] = null;
        ws();
        if (src[i] === ',') i++;
        else if (src[i] !== '}') throw new Error(`YAML flow mapping: unexpected '${src[i] ?? 'end'}'`);
      }
    }
    if (src[i] === '[') {
      i++;
      const a: YamlValue[] = [];
      for (;;) {
        ws();
        if (src[i] === ']') { i++; return a; }
        a.push(value());
        ws();
        if (src[i] === ',') i++;
        else if (src[i] !== ']') throw new Error(`YAML flow sequence: unexpected '${src[i] ?? 'end'}'`);
      }
    }
    let s = '';
    if (src[i] === '"' || src[i] === "'") { const q = src[i++]; s = q; while (i < src.length && src[i] !== q) s += src[i++]; s += q; i++; return scalar(s); }
    while (i < src.length && !',]}'.includes(src[i])) s += src[i++];
    return scalar(s);
  };
  const v = value();
  ws();
  if (i < src.length) throw new Error(`YAML: trailing text '${src.slice(i)}'`);
  return v;
}

interface Line { indent: number; text: string; no: number }

/** Parse a YAML document (subset). */
export function parseYaml(text: string): YamlValue {
  const lines: Line[] = [];
  text.replace(/^﻿/, '').split(/\r?\n/).forEach((raw, no) => {
    const s = stripComment(raw);
    if (!s.trim() || s.trim() === '---') return;
    if (/^\s*(&|\*|!|\||>)/.test(s.trim().replace(/^- /, '').replace(/^[^:]+:\s*/, ''))) throw new Error(`YAML line ${no + 1}: anchors, tags and block scalars are not supported`);
    lines.push({ indent: s.length - s.trimStart().length, text: s.trim(), no: no + 1 });
  });
  let k = 0;
  const block = (indent: number): YamlValue => {
    if (k >= lines.length) return null;
    if (lines[k].text.startsWith('- ') || lines[k].text === '-') {
      const arr: YamlValue[] = [];
      while (k < lines.length && lines[k].indent === indent && (lines[k].text.startsWith('- ') || lines[k].text === '-')) {
        const item = lines[k].text.slice(1).trim();
        k++;
        if (!item) arr.push(k < lines.length && lines[k].indent > indent ? block(lines[k].indent) : null);
        else if (/^[^{["'][^:]*:\s/.test(item) || /^[^{["'][^:]*:$/.test(item)) {
          // "- key: value" starts a mapping whose further keys are indented deeper
          lines.splice(k, 0, { indent: indent + 2, text: item, no: lines[k - 1].no });
          arr.push(block(indent + 2));
        } else arr.push(item.startsWith('{') || item.startsWith('[') ? flow(item) : scalar(item));
      }
      return arr;
    }
    const obj: { [k: string]: YamlValue } = {};
    while (k < lines.length && lines[k].indent === indent) {
      const l = lines[k];
      const m = /^("[^"]*"|'[^']*'|[^:]+?)\s*:(?:\s+(.*))?$/.exec(l.text);
      if (!m) throw new Error(`YAML line ${l.no}: expected 'key: value'`);
      const key = String(scalar(m[1]));
      const rest = (m[2] ?? '').trim();
      k++;
      if (rest) obj[key] = rest.startsWith('{') || rest.startsWith('[') ? flow(rest) : scalar(rest);
      else obj[key] = k < lines.length && lines[k].indent > indent ? block(lines[k].indent) : k < lines.length && lines[k].indent === indent && lines[k].text.startsWith('- ') ? block(indent) : null;
    }
    return obj;
  };
  const v = block(lines.length ? lines[0].indent : 0);
  if (k < lines.length) throw new Error(`YAML line ${lines[k].no}: unexpected indentation`);
  return v;
}
