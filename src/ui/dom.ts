/** Tiny DOM helpers (no framework). */
import { t } from './i18n';
export type Child = Node | string | number | null | undefined | false | Child[];

export function h<K extends keyof HTMLElementTagNameMap>(tag: K, attrs: Record<string, any> | null = null, ...children: Child[]): HTMLElementTagNameMap[K] {
  const el = document.createElement(tag);
  if (attrs) {
    for (const [k, v] of Object.entries(attrs)) {
      if (v === undefined || v === null || v === false) continue;
      if (k === 'class' || k === 'className') el.className = String(v);
      else if (k === 'style' && typeof v === 'object') Object.assign(el.style, v);
      else if (k.startsWith('on') && typeof v === 'function') el.addEventListener(k.slice(2).toLowerCase(), v);
      else if (k === 'dataset') Object.assign(el.dataset, v);
      else if (k in el && k !== 'list' && k !== 'form' && k !== 'type') { try { (el as any)[k] = v; } catch { el.setAttribute(k, String(v)); } }
      else el.setAttribute(k, String(v));
    }
  }
  append(el, children);
  return el;
}

export function append(el: Node, children: Child[]): void {
  for (const c of children) {
    if (c === null || c === undefined || c === false) continue;
    if (Array.isArray(c)) append(el, c);
    else if (c instanceof Node) el.appendChild(c);
    else el.appendChild(document.createTextNode(String(c)));
  }
}

export function clear(el: Element): void {
  while (el.firstChild) el.removeChild(el.firstChild);
}

export function icon(name: string, title?: string): HTMLSpanElement {
  const s = h('span', { class: `ico ico-${name}`, title });
  s.textContent = ICONS[name] ?? '•';
  return s;
}

const ICONS: Record<string, string> = {
  station: '🏭', robot: '🦾', frame: '⌖', tool: '🔧', object: '📦', target: '🎯', program: '📜', instruction: '›', folder: '📁', camera: '📷',
  mobile: '🚜', component: '⚙️', fleet: '🚚', map: '🗺️', field: '🌳', mission: '🧭', sensor: '📡', zone: '⬠', path: '〰️', notes: '📝', row: '🌿', control: '🎛',
  play: '▶', pause: '⏸', stop: '⏹', step: '⏭', add: '＋', remove: '✕', up: '↑', down: '↓', eye: '👁', eyeoff: '🚫', gear: '⚙', save: '💾', open: '📂', new: '🗋',
  fit: '⤢', translate: '✥', rotate: '⟳', undo: '↶', redo: '↷', export: '⇩', import: '⇧', run: '⚡', tree: '🌲', warn: '⚠', ok: '✓', link: '🔗', info: 'ⓘ',
};

export interface FieldSpec {
  key: string;
  label: string;
  type: 'text' | 'number' | 'select' | 'checkbox' | 'textarea' | 'range' | 'color' | 'file';
  value?: any;
  options?: Array<{ value: string; label: string }>;
  min?: number;
  max?: number;
  step?: number;
  hint?: string;
  accept?: string;
  multiple?: boolean;
}

export function formField(spec: FieldSpec, onChange?: (v: any) => void): { el: HTMLElement; get: () => any } {
  let input: HTMLInputElement | HTMLSelectElement | HTMLTextAreaElement;
  if (spec.type === 'select') {
    input = h('select', null, ...(spec.options ?? []).map((o) => h('option', { value: o.value, selected: o.value === String(spec.value) }, t(o.label))));
  } else if (spec.type === 'textarea') {
    input = h('textarea', { rows: 4 });
    input.value = spec.value ?? '';
  } else {
    input = h('input', { type: spec.type === 'range' ? 'range' : spec.type, min: spec.min, max: spec.max, step: spec.step ?? (spec.type === 'number' ? 'any' : undefined), accept: spec.accept, multiple: spec.multiple }) as HTMLInputElement;
    if (spec.type === 'checkbox') (input as HTMLInputElement).checked = !!spec.value;
    else if (spec.type !== 'file') input.value = spec.value ?? '';
  }
  const get = () => {
    if (spec.type === 'checkbox') return (input as HTMLInputElement).checked;
    if (spec.type === 'number' || spec.type === 'range') return parseFloat(input.value);
    if (spec.type === 'file') return (input as HTMLInputElement).files;
    return input.value;
  };
  if (onChange) input.addEventListener(spec.type === 'range' ? 'input' : 'change', () => onChange(get()));
  const el = h('label', { class: `field field-${spec.type}` }, h('span', { class: 'field-label' }, t(spec.label)), input, spec.hint ? h('small', { class: 'hint' }, t(spec.hint)) : null);
  return { el, get };
}

/** Modal dialog with a form; resolves with values or null when cancelled. */
export function dialog<T extends Record<string, any>>(title: string, fields: FieldSpec[], opts: { okLabel?: string; width?: number; body?: HTMLElement; onChange?: (values: Record<string, any>) => void } = {}): Promise<T | null> {
  return new Promise((resolve) => {
    const getters: Record<string, () => any> = {};
    const values = () => Object.fromEntries(Object.entries(getters).map(([k, g]) => [k, g()]));
    const form = h('div', { class: 'dialog-form' });
    for (const f of fields) {
      const { el, get } = formField(f, () => opts.onChange?.(values()));
      getters[f.key] = get;
      form.appendChild(el);
    }
    if (opts.body) form.appendChild(opts.body);
    const close = (v: T | null) => { overlay.remove(); resolve(v); };
    const overlay = h('div', { class: 'overlay', onClick: (e: MouseEvent) => { if (e.target === overlay) close(null); } },
      h('div', { class: 'dialog', style: { width: `${opts.width ?? 460}px` } },
        h('div', { class: 'dialog-title' }, t(title), h('button', { class: 'btn-icon', onClick: () => close(null) }, '✕')),
        form,
        h('div', { class: 'dialog-actions' }, h('button', { class: 'btn', onClick: () => close(null) }, t('Cancel')), h('button', { class: 'btn primary', onClick: () => close(values() as T) }, t(opts.okLabel ?? 'OK')))));
    document.body.appendChild(overlay);
    overlay.addEventListener('keydown', (e) => { if (e.key === 'Escape') close(null); if (e.key === 'Enter' && (e.target as HTMLElement).tagName !== 'TEXTAREA') close(values() as T); });
    (form.querySelector('input,select,textarea') as HTMLElement | null)?.focus();
  });
}

export function confirmDialog(title: string, message: string): Promise<boolean> {
  return dialog(title, [], { body: h('p', null, message), okLabel: 'Yes' }).then((r) => r !== null);
}

export function toast(message: string, level: 'info' | 'warn' | 'error' | 'ok' = 'info', ms = 3500): void {
  let host = document.getElementById('toasts');
  if (!host) { host = h('div', { id: 'toasts' }); document.body.appendChild(host); }
  const t = h('div', { class: `toast toast-${level}` }, message);
  host.appendChild(t);
  setTimeout(() => { t.classList.add('hide'); setTimeout(() => t.remove(), 400); }, ms);
}

export interface MenuEntry { label?: string; action?: () => void; disabled?: boolean; separator?: boolean; children?: MenuEntry[]; shortcut?: string; checked?: boolean }

export function contextMenu(x: number, y: number, entries: MenuEntry[]): void {
  document.querySelectorAll('.ctx-menu').forEach((m) => m.remove());
  const menu = h('div', { class: 'ctx-menu', style: { left: `${x}px`, top: `${y}px` } });
  const close = () => { menu.remove(); document.removeEventListener('pointerdown', onDoc, true); };
  const onDoc = (e: PointerEvent) => { if (!menu.contains(e.target as Node)) close(); };
  for (const e of entries) {
    if (e.separator) { menu.appendChild(h('div', { class: 'ctx-sep' })); continue; }
    const row = h('div', { class: `ctx-item ${e.disabled ? 'disabled' : ''}`, onClick: () => { if (!e.disabled && e.action) { close(); e.action(); } } }, e.checked ? '✓ ' : '', t(e.label ?? ''), e.shortcut ? h('span', { class: 'shortcut' }, e.shortcut) : null, e.children ? h('span', { class: 'shortcut' }, '▸') : null);
    if (e.children) {
      row.addEventListener('pointerenter', () => {
        const r = row.getBoundingClientRect();
        menu.querySelectorAll('.ctx-menu').forEach((m) => m.remove());
        const sub = h('div', { class: 'ctx-menu', style: { left: `${r.width - 4}px`, top: `${row.offsetTop}px` } });
        for (const c of e.children!) {
          if (c.separator) { sub.appendChild(h('div', { class: 'ctx-sep' })); continue; }
          sub.appendChild(h('div', { class: `ctx-item ${c.disabled ? 'disabled' : ''}`, onClick: () => { if (!c.disabled && c.action) { close(); c.action(); } } }, t(c.label ?? '')));
        }
        menu.appendChild(sub);
      });
    }
    menu.appendChild(row);
  }
  document.body.appendChild(menu);
  const r = menu.getBoundingClientRect();
  if (r.right > window.innerWidth) menu.style.left = `${Math.max(0, x - r.width)}px`;
  if (r.bottom > window.innerHeight) menu.style.top = `${Math.max(0, y - r.height)}px`;
  setTimeout(() => document.addEventListener('pointerdown', onDoc, true), 0);
}

export function downloadText(name: string, content: string, mime = 'text/plain'): void {
  const blob = new Blob([content], { type: mime });
  const a = h('a', { href: URL.createObjectURL(blob), download: name });
  document.body.appendChild(a);
  a.click();
  setTimeout(() => { URL.revokeObjectURL(a.href); a.remove(); }, 100);
}

export function downloadBlob(name: string, blob: Blob): void {
  const a = h('a', { href: URL.createObjectURL(blob), download: name });
  document.body.appendChild(a);
  a.click();
  setTimeout(() => { URL.revokeObjectURL(a.href); a.remove(); }, 100);
}

export function pickFiles(accept: string, multiple = false): Promise<File[]> {
  return new Promise((resolve) => {
    const input = h('input', { type: 'file', accept, multiple, style: { display: 'none' } }) as HTMLInputElement;
    input.addEventListener('change', () => { resolve([...(input.files ?? [])]); input.remove(); });
    document.body.appendChild(input);
    input.click();
  });
}

export function fmt(v: number, d = 2): string {
  return Number.isFinite(v) ? v.toFixed(d) : '-';
}
