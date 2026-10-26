// @vitest-environment jsdom
/**
 * The diagram inspectors express what the text view can: Petri transition guards and input events, final markings and
 * sensors; automaton edge guards, sensors and `terminate marked`. Edits write the DSL; the written model runs as typed.
 */
import { describe, it, expect, beforeAll } from 'vitest';
import { parsePetri, parseDes, TEMPLATES } from '../src/ctl/dsl';
import { buildGraphEditor } from '../src/ui/graph_editor';
import { installDomStubs, change } from './ui_harness';

let src = '';
let editor: ReturnType<typeof buildGraphEditor>;
const fieldIn = (label: string): HTMLInputElement => {
  const f = [...editor.el.querySelectorAll<HTMLElement>('.ge-inspector label.ge-field, .ge-inspector label.ge-check')].find((l) => (l.querySelector('span')?.textContent ?? l.textContent!.trim()) === label || l.textContent!.trim() === label);
  if (!f) throw new Error(`field "${label}" not found among: ${[...editor.el.querySelectorAll('.ge-inspector label')].map((l) => l.querySelector('span')?.textContent ?? l.textContent).join(' | ')}`);
  return f.querySelector('input,select,textarea') as HTMLInputElement;
};
beforeAll(() => { installDomStubs(); editor = buildGraphEditor({ onChange: (s) => { src = s; } }); document.body.appendChild(editor.el); });

describe('diagram inspectors: guards, input events, sensors, final markings', () => {
  it('Petri net: transition guard and input event, final markings and sensors in the document inspector', () => {
    editor.load('petri', TEMPLATES.petri);
    editor.select({ type: 'node', id: 't2a' });
    change(fieldIn('Guard (expression)'), 'Ready == 1'); change(fieldIn('Input event (on=)'), 'part_in');
    let p = parsePetri(src); const t2a = p.spec.transitions.find((t) => t.id === 't2a')!;
    expect(t2a.guard).toBe('Ready == 1'); expect(t2a.on).toBe('part_in');
    editor.select(null);
    change(fieldIn('Final markings'), 'p1 p2 rM rZ; a2=1 b2'); change(fieldIn('Sensors'), 'part_in "count > 0"');
    p = parsePetri(src);
    expect(p.finals).toEqual([{ p1: 1, p2: 1, rM: 1, rZ: 1 }, { a2: 1, b2: 1 }]);
    expect(p.sensors).toEqual([{ event: 'part_in', expr: 'count > 0' }]);
    // a later edit keeps them; clearing the guard removes it
    editor.select({ type: 'node', id: 't2a' }); change(fieldIn('Guard (expression)'), '');
    p = parsePetri(src); expect(p.spec.transitions.find((t) => t.id === 't2a')!.guard).toBeUndefined(); expect(p.finals?.length).toBe(2);
  });
  it('automata: a guard per controllable event of an edge, sensors and terminate marked', () => {
    editor.load('des', TEMPLATES.des);
    editor.select({ type: 'edge', from: 'I', to: 'W' });
    change(fieldIn('Guard of start1'), "zone == 'table'");
    let d = parseDes(src); expect(d.guards).toEqual({ start1: "zone == 'table'" });
    editor.select(null);
    change(fieldIn('Sensors'), 'finish1 "done1 == 1"'); change(fieldIn('Terminate when marked'), 'on');
    const box = fieldIn('Terminate when marked'); if (box.type === 'checkbox' && !box.checked) { box.checked = true; box.dispatchEvent(new Event('change')); }
    d = parseDes(src);
    expect(d.sensors).toEqual([{ event: 'finish1', expr: 'done1 == 1' }]); expect(d.terminate).toBe('marked');
    // an uncontrollable event has no guard field
    editor.select({ type: 'edge', from: 'W', to: 'I' });
    expect(() => fieldIn('Guard of finish1')).toThrow(/not found/);
  });
});
