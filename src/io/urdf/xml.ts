/** Tiny dependency-free XML parser (elements, attributes, text, comments, CDATA). Enough for URDF/xacro/SDF. */
export interface XNode {
  type: 'element' | 'text';
  name: string;
  attrs: Record<string, string>;
  children: XNode[];
  text?: string;
}

export function parseXML(src: string): XNode {
  let i = 0;
  const n = src.length;
  const root: XNode = { type: 'element', name: '#document', attrs: {}, children: [] };
  const stack: XNode[] = [root];
  const decode = (s: string) => s.replace(/&lt;/g, '<').replace(/&gt;/g, '>').replace(/&quot;/g, '"').replace(/&apos;/g, "'").replace(/&#(\d+);/g, (_m, d) => String.fromCharCode(+d)).replace(/&amp;/g, '&');
  while (i < n) {
    if (src[i] === '<') {
      if (src.startsWith('<!--', i)) {
        const e = src.indexOf('-->', i);
        i = e < 0 ? n : e + 3;
        continue;
      }
      if (src.startsWith('<![CDATA[', i)) {
        const e = src.indexOf(']]>', i);
        const txt = src.slice(i + 9, e < 0 ? n : e);
        stack[stack.length - 1].children.push({ type: 'text', name: '#text', attrs: {}, children: [], text: txt });
        i = e < 0 ? n : e + 3;
        continue;
      }
      if (src.startsWith('<?', i) || src.startsWith('<!', i)) {
        const e = src.indexOf('>', i);
        i = e < 0 ? n : e + 1;
        continue;
      }
      if (src[i + 1] === '/') {
        const e = src.indexOf('>', i);
        i = e < 0 ? n : e + 1;
        if (stack.length > 1) stack.pop();
        continue;
      }
      // start tag
      let j = i + 1;
      while (j < n && !/[\s/>]/.test(src[j])) j++;
      const name = src.slice(i + 1, j);
      const attrs: Record<string, string> = {};
      let selfClose = false;
      while (j < n) {
        while (j < n && /\s/.test(src[j])) j++;
        if (src[j] === '/') { selfClose = true; j++; continue; }
        if (src[j] === '>') { j++; break; }
        let k = j;
        while (k < n && !/[\s=/>]/.test(src[k])) k++;
        const an = src.slice(j, k);
        j = k;
        while (j < n && /\s/.test(src[j])) j++;
        let av = '';
        if (src[j] === '=') {
          j++;
          while (j < n && /\s/.test(src[j])) j++;
          const q = src[j];
          if (q === '"' || q === "'") {
            const e = src.indexOf(q, j + 1);
            av = src.slice(j + 1, e < 0 ? n : e);
            j = e < 0 ? n : e + 1;
          } else {
            let e = j;
            while (e < n && !/[\s/>]/.test(src[e])) e++;
            av = src.slice(j, e);
            j = e;
          }
        }
        if (an) attrs[an] = decode(av);
      }
      const el: XNode = { type: 'element', name, attrs, children: [] };
      stack[stack.length - 1].children.push(el);
      if (!selfClose) stack.push(el);
      i = j;
    } else {
      const e = src.indexOf('<', i);
      const txt = src.slice(i, e < 0 ? n : e);
      if (txt.trim()) stack[stack.length - 1].children.push({ type: 'text', name: '#text', attrs: {}, children: [], text: decode(txt) });
      i = e < 0 ? n : e;
    }
  }
  const first = root.children.find((c) => c.type === 'element');
  return first ?? root;
}

export function serializeXML(node: XNode, indent = ''): string {
  if (node.type === 'text') return escape(node.text ?? '');
  const attrs = Object.entries(node.attrs).map(([k, v]) => ` ${k}="${escape(v)}"`).join('');
  if (!node.children.length) return `${indent}<${node.name}${attrs}/>\n`;
  const inner = node.children.map((c) => (c.type === 'text' ? escape(c.text ?? '') : serializeXML(c, indent + '  '))).join('');
  return `${indent}<${node.name}${attrs}>\n${inner}${indent}</${node.name}>\n`;
}

function escape(s: string): string {
  return s.replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;').replace(/"/g, '&quot;');
}

export function childElements(n: XNode, name?: string): XNode[] {
  return n.children.filter((c) => c.type === 'element' && (!name || c.name === name));
}
export function childElement(n: XNode, name: string): XNode | undefined {
  return n.children.find((c) => c.type === 'element' && c.name === name);
}
export function nums(s: string | undefined, def: number[]): number[] {
  if (!s) return def;
  const v = s.trim().split(/\s+/).map(Number);
  return v.some(isNaN) ? def : v;
}
