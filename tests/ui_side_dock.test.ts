// @vitest-environment jsdom
/** Station tree / Properties docks: drag to resize, collapse to a rail and back, both remembered per side. */
import { describe, it, expect, beforeEach } from 'vitest';
import { sideDock } from '../src/ui/side_panels';

function pointer(type: string, clientX: number): PointerEvent {
  const e = new MouseEvent(type, { clientX, button: 0, bubbles: true }) as unknown as PointerEvent;
  Object.defineProperty(e, 'pointerId', { value: 1 });
  return e;
}

describe('side docks', () => {
  beforeEach(() => { localStorage.clear(); document.body.innerHTML = ''; });

  it('resizes the left panel by dragging its splitter and keeps the width within limits', () => {
    const panel = document.createElement('div');
    const dock = sideDock(panel, { side: 'left', title: 'Station', defaultWidth: 290, key: 't.tree' });
    document.body.append(dock.el);
    expect(panel.style.width).toBe('290px');
    const splitter = dock.el.querySelector<HTMLElement>('.dock-splitter')!;
    splitter.dispatchEvent(pointer('pointerdown', 300));
    splitter.dispatchEvent(pointer('pointermove', 420));
    splitter.dispatchEvent(pointer('pointerup', 420));
    expect(dock.width()).toBe(410);
    expect(panel.style.width).toBe('410px');
    expect(localStorage.getItem('t.tree.width')).toBe('410');
    dock.setWidth(10);
    expect(dock.width()).toBe(160);
    splitter.dispatchEvent(new MouseEvent('dblclick', { bubbles: true }));
    expect(dock.width()).toBe(290);
  });

  it('the right panel grows when its splitter is dragged to the left', () => {
    const panel = document.createElement('div');
    const dock = sideDock(panel, { side: 'right', title: 'Properties', defaultWidth: 330, key: 't.props' });
    document.body.append(dock.el);
    const splitter = dock.el.querySelector<HTMLElement>('.dock-splitter')!;
    splitter.dispatchEvent(pointer('pointerdown', 800));
    splitter.dispatchEvent(pointer('pointermove', 700));
    splitter.dispatchEvent(pointer('pointerup', 700));
    expect(dock.width()).toBe(430);
  });

  it('collapses to a rail with the toggle and expands from the rail; the state survives a reload', () => {
    const panel = document.createElement('div');
    const dock = sideDock(panel, { side: 'left', title: 'Station', defaultWidth: 290, key: 't.tree' });
    document.body.append(dock.el);
    dock.el.querySelector<HTMLButtonElement>('.dock-toggle')!.click();
    expect(dock.isCollapsed()).toBe(true);
    expect(dock.el.classList.contains('collapsed')).toBe(true);
    expect(dock.el.querySelector('.dock-rail')!.textContent).toContain('Station');
    // a new page builds the dock collapsed with the same width
    const again = sideDock(document.createElement('div'), { side: 'left', title: 'Station', defaultWidth: 290, key: 't.tree' });
    expect(again.isCollapsed()).toBe(true);
    dock.el.querySelector<HTMLElement>('.dock-rail')!.click();
    expect(dock.isCollapsed()).toBe(false);
    expect(dock.el.classList.contains('collapsed')).toBe(false);
    expect(localStorage.getItem('t.tree.collapsed')).toBe('0');
  });
});
