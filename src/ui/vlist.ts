/**
 * Virtualised list: only the rows inside the scroll viewport (plus an overscan) exist in the DOM, so trees and
 * instruction lists of 10k+ entries render in constant time. Rows are keyed; a row whose signature is unchanged
 * keeps its DOM element across repaints (clicks that land between mousedown and mouseup are not lost), and the
 * DOM is only touched when the visible sequence actually changes. Events are expected to be delegated to `el`.
 */
export class VirtualList<T> {
  private items: T[] = [];
  private host: HTMLElement;
  private top: HTMLElement;
  private bottom: HTMLElement;
  private cache = new Map<string, { sig: string; el: HTMLElement }>();
  private raf = 0;
  /** Rows rendered when the container has no layout yet (hidden tab, jsdom). */
  fallbackRows = 400;
  overscan = 12;

  constructor(readonly el: HTMLElement, readonly rowHeight: number, private readonly opts: {
    key: (item: T, index: number) => string;
    sig: (item: T, index: number) => string;
    render: (item: T, index: number) => HTMLElement;
  }) {
    this.top = document.createElement('div');
    this.bottom = document.createElement('div');
    this.host = document.createElement('div');
    this.top.className = 'vlist-pad'; this.bottom.className = 'vlist-pad';
    el.append(this.top, this.host, this.bottom);
    el.addEventListener('scroll', () => this.schedule());
  }

  get length(): number { return this.items.length; }
  at(i: number): T | undefined { return this.items[i]; }

  setItems(items: T[]): void {
    this.items = items;
    this.repaint();
  }

  /** Re-render on the next animation frame (scroll), or now when there is no rAF. */
  schedule(): void {
    if (this.raf) return;
    if (typeof requestAnimationFrame !== 'function') { this.repaint(); return; }
    this.raf = requestAnimationFrame(() => { this.raf = 0; this.repaint(); });
  }

  /** Index range currently materialised. */
  window(): [number, number] {
    const n = this.items.length;
    const h = this.el.clientHeight;
    if (!h) return [0, Math.min(n, this.fallbackRows)];
    const first = Math.max(0, Math.floor(this.el.scrollTop / this.rowHeight) - this.overscan);
    const last = Math.min(n, Math.ceil((this.el.scrollTop + h) / this.rowHeight) + this.overscan);
    return [first, last];
  }

  repaint(): void {
    const [a, b] = this.window();
    const els: HTMLElement[] = [];
    const live = new Set<string>();
    for (let i = a; i < b; i++) {
      const it = this.items[i];
      const key = this.opts.key(it, i);
      const sig = this.opts.sig(it, i);
      live.add(key);
      let c = this.cache.get(key);
      if (!c || c.sig !== sig) { c = { sig, el: this.opts.render(it, i) }; this.cache.set(key, c); }
      els.push(c.el);
    }
    for (const k of [...this.cache.keys()]) if (!live.has(k)) this.cache.delete(k);
    this.top.style.height = `${a * this.rowHeight}px`;
    this.bottom.style.height = `${Math.max(0, this.items.length - b) * this.rowHeight}px`;
    const cur = this.host.children;
    let same = cur.length === els.length;
    for (let i = 0; same && i < els.length; i++) if (cur[i] !== els[i]) same = false;
    if (!same) this.host.replaceChildren(...els);
  }

  /** Scroll so that row `i` is visible. */
  scrollTo(i: number): void {
    const h = this.el.clientHeight;
    if (!h) return;
    const y = i * this.rowHeight;
    if (y < this.el.scrollTop) this.el.scrollTop = y;
    else if (y + this.rowHeight > this.el.scrollTop + h) this.el.scrollTop = y + this.rowHeight - h;
  }

  /** Drop cached rows (forces a re-render of every visible row). */
  invalidate(): void { this.cache.clear(); }
}
