// @vitest-environment jsdom
/**
 * Graph editor: two opposite edges between the same nodes (automaton template I ⇄ W) are two separate arcs. Each edge
 * group carries a 12 px transparent hit stroke (styles.css .ge-edge-hit); before, the first edge was a straight line
 * and the second bowed by 6.5 px, so the second edge's hit stroke covered the first along its whole length and a click
 * on the first edge (or its label) selected the second (V4 F13). The hit paths must keep more than one stroke width
 * apart everywhere, also after auto layout and after dragging a node away, and every label must lie outside the other
 * edge's hit stroke.
 */
import { describe, it, expect, beforeAll } from 'vitest';
import { TEMPLATES } from '../src/ctl/dsl';
import { buildGraphEditor } from '../src/ui/graph_editor';
import { installDomStubs } from './ui_harness';

type P = [number, number];
let editor: ReturnType<typeof buildGraphEditor>;
beforeAll(() => { installDomStubs(); editor = buildGraphEditor({ onChange: () => undefined }); document.body.appendChild(editor.el); });

/** Points along an SVG path made of M + L or M + Q (what the editor draws for edges between two nodes). */
function samples(d: string, n = 400): P[] {
  const v = d.replace(/[MLQ,]/g, ' ').trim().split(/\s+/).map(Number);
  if (/Q/.test(d)) { const [x0, y0, cx, cy, x1, y1] = v; return Array.from({ length: n + 1 }, (_, i) => { const t = i / n, a = (1 - t) ** 2, b = 2 * t * (1 - t), c = t * t; return [a * x0 + b * cx + c * x1, a * y0 + b * cy + c * y1] as P; }); }
  const [x0, y0, x1, y1] = v; return Array.from({ length: n + 1 }, (_, i) => { const t = i / n; return [x0 + (x1 - x0) * t, y0 + (y1 - y0) * t] as P; });
}
const minDist = (a: P[], b: P[]) => { let m = Infinity; for (const p of a) for (const q of b) m = Math.min(m, Math.hypot(p[0] - q[0], p[1] - q[1])); return m; };
const HIT = 12; // .ge-edge-hit stroke-width

function edges(): Array<{ from: string; to: string; hit: P[]; label: P | null; group: Element }> {
  return [...editor.el.querySelectorAll('g.ge-edge')].map((g) => {
    const hit = g.querySelector('path.ge-edge-hit')!.getAttribute('d')!;
    const t = g.querySelector('text.ctl-elabel');
    return { from: '', to: '', hit: samples(hit), label: t ? [Number(t.getAttribute('x')), Number(t.getAttribute('y'))] : null, group: g };
  });
}
function checkApart(where: string) {
  const es = edges();
  expect(es.length, where).toBe(2);
  const [a, b] = es;
  // the two hit strokes (12 px wide) do not overlap anywhere: a click on one edge cannot land on the other one
  expect(minDist(a.hit, b.hit), where).toBeGreaterThan(HIT + 2);
  // each label is outside the other edge's hit stroke and nearer to its own edge
  for (const [own, other] of [[a, b], [b, a]]) {
    expect(minDist([own.label!], other.hit), where).toBeGreaterThan(HIT / 2 + 4);
    expect(minDist([own.label!], own.hit), where).toBeLessThan(minDist([own.label!], other.hit));
  }
}

describe('graph editor: opposite edges are separately selectable', () => {
  it('automaton template I ⇄ W: separate arcs at the template layout, after auto layout and after dragging W 200 px away', () => {
    editor.load('des', TEMPLATES.des);
    checkApart('template layout');
    editor.autoLayout();
    checkApart('auto layout');
    const W = editor.state().des!.layout![editor.state().des!.plant[0].name].W;
    editor.moveNode('W', W[0] + 200, W[1]);
    checkApart('W dragged');
    // a pointerdown on an edge group (line, hit stroke or label) selects that edge: the two groups select both directions
    const picked: string[] = [];
    for (let i = 0; i < 2; i++) { edges()[i].group.dispatchEvent(new Event('pointerdown', { bubbles: true })); const s = editor.state().selection; picked.push(s?.type === 'edge' ? `${s.from}->${s.to}` : 'none'); }
    expect(picked.sort()).toEqual(['I->W', 'W->I']);
  });
});
