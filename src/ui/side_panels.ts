import { h } from './dom';
import { t } from './i18n';

/**
 * Side dock around the Station tree or the Properties panel: a splitter on the inner edge resizes the panel (drag;
 * double-click restores the default width), the button on the splitter collapses it to a thin rail and back.
 * Width and collapsed state are remembered per side in localStorage.
 */
export interface SideDock {
  el: HTMLElement;
  setCollapsed(collapsed: boolean): void;
  isCollapsed(): boolean;
  setWidth(px: number): void;
  width(): number;
}

const MIN = 160;
const store = {
  get(key: string): string | null { try { return localStorage.getItem(key); } catch { return null; } },
  set(key: string, v: string): void { try { localStorage.setItem(key, v); } catch { /* private mode */ } },
};

export function sideDock(panel: HTMLElement, opts: { side: 'left' | 'right'; title: string; defaultWidth: number; key: string }): SideDock {
  const { side, key } = opts;
  const maxWidth = () => Math.max(MIN, Math.round((typeof window !== 'undefined' ? window.innerWidth : 1600) * 0.6));
  const clamp = (w: number) => Math.min(maxWidth(), Math.max(MIN, Math.round(w)));
  let width = clamp(Number(store.get(`${key}.width`)) || opts.defaultWidth);
  let collapsed = store.get(`${key}.collapsed`) === '1';

  const toggle = h('button', {
    class: 'dock-toggle', type: 'button',
    onClick: (e: Event) => { e.stopPropagation(); dock.setCollapsed(!collapsed); },
  });
  const splitter = h('div', { class: 'dock-splitter', title: t('Drag to resize, double-click for the default width') }, toggle);
  const rail = h('div', { class: 'dock-rail', title: t('Expand'), onClick: () => dock.setCollapsed(false) },
    h('span', { class: 'dock-rail-arrow' }, side === 'left' ? '›' : '‹'), h('span', { class: 'dock-rail-label' }, t(opts.title)));
  const el = h('div', { class: `side-dock ${side}` });
  if (side === 'left') el.append(rail, panel, splitter); else el.append(splitter, panel, rail);

  const apply = () => {
    el.classList.toggle('collapsed', collapsed);
    panel.style.width = `${width}px`;
    panel.style.flex = '0 0 auto';
    // the arrow points to where the panel goes
    toggle.textContent = (side === 'left') !== collapsed ? '‹' : '›';
    toggle.title = collapsed ? t('Expand') : t('Collapse');
  };

  // drag the splitter: width follows the pointer (inner edge), captured so the 3D view does not get the drag
  splitter.addEventListener('pointerdown', (e: PointerEvent) => {
    if (e.target === toggle || collapsed || e.button !== 0) return;
    e.preventDefault();
    const x0 = e.clientX, w0 = width;
    splitter.setPointerCapture?.(e.pointerId);
    el.classList.add('resizing');
    const move = (ev: PointerEvent) => { dock.setWidth(w0 + (side === 'left' ? ev.clientX - x0 : x0 - ev.clientX)); };
    const up = (ev: PointerEvent) => {
      splitter.releasePointerCapture?.(ev.pointerId);
      splitter.removeEventListener('pointermove', move);
      splitter.removeEventListener('pointerup', up);
      el.classList.remove('resizing');
      store.set(`${key}.width`, String(width));
    };
    splitter.addEventListener('pointermove', move);
    splitter.addEventListener('pointerup', up);
  });
  splitter.addEventListener('dblclick', (e) => { if (e.target === toggle) return; dock.setWidth(opts.defaultWidth); store.set(`${key}.width`, String(width)); });

  const dock: SideDock = {
    el,
    setCollapsed(c: boolean) { collapsed = c; store.set(`${key}.collapsed`, c ? '1' : '0'); apply(); },
    isCollapsed: () => collapsed,
    setWidth(px: number) { width = clamp(px); apply(); },
    width: () => width,
  };
  apply();
  return dock;
}
