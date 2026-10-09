// @vitest-environment jsdom
import { describe, it, expect } from 'vitest';
import { renderPlot } from '../src/ui/plots';
import { analyse } from '../src/ctl/analysis';
import { MRS_TEMPLATES } from '../src/mrs/dsl';
import { MRS_KINDS } from '../src/mrs/model';

describe('report charts (SVG renderer used by the Control tab)', () => {
  it('renders lines, paths, grids and bars with axes, legends and markers', () => {
    const l = renderPlot({ kind: 'lines', title: 'x', xlabel: 't', ylabel: 'v', series: [{ name: 'a', points: [[0, 0], [1, 1], [2, 0.5]] }, { name: 'b', points: [[0, 1], [2, 2]], dashed: true }] });
    expect(l.tagName.toLowerCase()).toBe('svg'); expect(l.querySelectorAll('path.plot-line').length).toBe(2); expect(l.querySelector('.plot-title')!.textContent).toBe('x'); expect(l.querySelectorAll('.plot-tick').length).toBeGreaterThan(4); expect(l.querySelector('[stroke-dasharray="5 4"]')).not.toBeNull();
    const p = renderPlot({ kind: 'paths', series: [{ name: 'r1', points: [[0, 0], [1, 1]] }], markers: [{ x: 1, y: 1, kind: 'goal', label: 'g' }, { x: 0, y: 0, kind: 'source' }, { x: 0.5, y: 0.5, kind: 'station' }], obstacles: [{ x: 0.2, y: 0.2, w: 0.1, h: 0.1 }], equal: true });
    expect(p.querySelector('.plot-marker.goal')).not.toBeNull(); expect(p.querySelector('.plot-marker.source')).not.toBeNull(); expect(p.querySelector('.plot-obstacle')).not.toBeNull(); expect(p.textContent).toContain('g');
    const g = renderPlot({ kind: 'grid', cells: [[1, 0], [0, 1]], cellStyle: 'binary', title: 'cells' }); expect(g.querySelectorAll('rect.plot-cell').length).toBe(4);
    const lab = renderPlot({ kind: 'grid', cells: [[1, 2, 3]], cellStyle: 'labels' }); expect(lab.querySelectorAll('rect.plot-cell').length).toBe(3);
    const b = renderPlot({ kind: 'bars', bars: [{ label: 'A', value: 2 }, { label: 'B', value: -1 }], title: 'bars' }); expect(b.querySelectorAll('rect').length).toBe(2); expect(b.textContent).toContain('A');
    const log = renderPlot({ kind: 'lines', series: [{ name: 'e', points: [[0, 1], [1, 0.01], [2, 1e-4]] }], ylog: true }); expect(log.querySelectorAll('path.plot-line').length).toBe(1);
    const empty = renderPlot({ kind: 'lines', series: [] }); expect(empty.tagName.toLowerCase()).toBe('svg');
  });
  it('every chart of every template renders', () => {
    let charts = 0;
    for (const k of MRS_KINDS) { const r = analyse(k.kind, MRS_TEMPLATES[k.kind]); for (const s of r.sections) if (s.plot) { const svg = renderPlot(s.plot); expect(svg.getAttribute('viewBox')).toMatch(/^0 0 \d/); charts++; } }
    expect(charts).toBeGreaterThan(20);
  }, 60000);
});
