// @vitest-environment jsdom
/**
 * The FSM diagram editor (graph editor, kind `fsm`): one diagram per machine with the machine's outcomes as final nodes,
 * outcome edges, state inspector (kind, initial, timeout, `do` binding, decisions, entry / exit), nested machines. Every
 * edit writes the DSL back; the edited document parses, checks and runs with the FSM executor.
 */
import { describe, it, expect, beforeAll } from 'vitest';
import { TEMPLATES } from '../src/ctl/dsl';
import { parseFsm, checkFsm, FsmExecutor } from '../src/ctl/fsm';
import { buildGraphEditor } from '../src/ui/graph_editor';
import { simulatedHost } from '../src/ctl/runtime';
import { installDomStubs, change, pointer, setField, clickOk, tickMs, dialogTitle } from './ui_harness';

let src = '';
const ed = () => editor;
let editor: ReturnType<typeof buildGraphEditor>;
const nodeIds = () => [...editor.el.querySelectorAll('g.ge-node title')].map((t) => t.textContent!.split(' — ')[0]);
const nodeGroup = (id: string) => [...editor.el.querySelectorAll<SVGGElement>('g.ge-node')].find((n) => n.querySelector('title')!.textContent!.split(' — ')[0] === id)!;
const fieldIn = (label: string): HTMLInputElement => { const f = [...editor.el.querySelectorAll<HTMLElement>('.ge-inspector label.ge-field, .ge-inspector label.ge-check')].find((l) => (l.querySelector('span')?.textContent ?? l.textContent!.trim()) === label || l.textContent!.trim() === label); if (!f) throw new Error(`field ${label} not found`); return f.querySelector('input,select,textarea') as HTMLInputElement; };
const button = (root: ParentNode, label: string) => [...root.querySelectorAll<HTMLButtonElement>('button')].find((b) => b.textContent === label)!;

beforeAll(() => { installDomStubs(); editor = buildGraphEditor({ onChange: (s) => { src = s; }, catalog: () => ({ robots: [{ name: 'UR5e', mobile: false }], programs: [{ name: 'Home' }, { name: 'Pick' }, { name: 'Place' }, { name: 'Scan' }], targets: [], zones: [], signals: [] }) }); document.body.appendChild(editor.el); });

describe('FSM diagram editor', () => {
  it('draws one machine per diagram with outcome nodes and outcome edges; the machine selector opens nested machines', () => {
    ed().load('fsm', TEMPLATES.fsm);
    expect(nodeIds().sort()).toEqual(['Cycle', 'Home', 'Park', 'aborted', 'done'].sort());
    const edges = [...editor.el.querySelectorAll('g.ge-edge text')].map((t) => t.textContent); expect(edges).toEqual(expect.arrayContaining(['ok', 'fail', 'empty', 'failed']));
    const sel = editor.el.querySelector<HTMLSelectElement>('select.ge-block')!; expect([...sel.options].map((o) => o.textContent)).toEqual(['machine Pick_all_boxes', 'machine Pick_all_boxes/Cycle']);
    change(sel, '1'); expect(nodeIds().sort()).toEqual(['Pick', 'Place', 'Plan', 'empty', 'failed'].sort());
    expect(nodeGroup('Plan').querySelector('path')).not.toBeNull(); // decision drawn as a diamond
  });
  it('adds a skill state with a binding, connects outcomes, renames, and the written DSL runs', async () => {
    ed().load('fsm', TEMPLATES.fsm); ed().setBlock(0);
    // a new state before Park: Scan (skill: program Scan), Cycle's `empty` now goes to Scan, Scan ok -> Park
    const id = ed().addNodeAt(300, 300)!; expect(parseFsm(src).root.states.find((x) => x.name === id)?.kind).toBe('pass');
    change(fieldIn('Name'), 'Scan'); expect(parseFsm(src).root.states.map((s) => s.name)).toContain('Scan');
    change(fieldIn('Kind'), 'skill'); change(fieldIn('Do (binding)'), 'program=Scan timeout=10');
    let doc = parseFsm(src); const scan = doc.root.states.find((s) => s.name === 'Scan')!; expect(scan.kind).toBe('skill'); expect(scan.action).toMatchObject({ kind: 'program', value: 'Scan', timeout: 10 });
    ed().setMode('edge'); nodeGroup('Scan').dispatchEvent(pointer('pointerdown')); nodeGroup('Park').dispatchEvent(pointer('pointerdown'));
    expect(dialogTitle()).toBe('New outcome edge'); setField('Outcomes of Scan → Park (comma separated)', 'ok'); clickOk(); await tickMs(1);
    ed().connect('Scan', '⊙aborted', ['fail']);
    ed().select({ type: 'edge', from: 'Cycle', to: 'Park' }); change(fieldIn('Outcomes'), ''); // remove Cycle -empty-> Park
    ed().connect('Cycle', 'Scan', ['empty']);
    doc = parseFsm(src);
    expect(doc.root.states.find((s) => s.name === 'Scan')!.transitions).toEqual({ ok: 'Park', fail: 'aborted' });
    expect(doc.root.states.find((s) => s.name === 'Cycle')!.transitions.empty).toBe('Scan');
    expect(checkFsm(doc).filter((i) => i.level === 'error')).toEqual([]);
    // the edited machine runs: Home, 3 × (Pick, Place), Scan, Park → done
    const host = simulatedHost({ programs: { Home: 1, Pick: 1, Place: 1, Scan: 2 } });
    const x = new FsmExecutor({ doc, world: { host, forRobot: () => null, step: (dt) => host.step?.(dt) } }); x.run(60, 0.1);
    expect(x.outcome).toBe('done'); expect(host.runs).toEqual(['Home', 'Pick', 'Place', 'Pick', 'Place', 'Pick', 'Place', 'Scan', 'Home']);
  });
  it('decision, entry / exit, timeout and initial edit through the inspector; a new nested machine gets its own diagram', () => {
    ed().load('fsm', TEMPLATES.fsm); ed().setBlock(1); ed().select({ type: 'node', id: 'Plan' });
    change(fieldIn('When (expr -> outcome, one per line)'), 'remaining > 1 -> next\nremaining == 1 -> next'); change(fieldIn('Otherwise'), 'empty');
    let plan = parseFsm(src).root.states.find((s) => s.name === 'Cycle')!.machine!.states.find((s) => s.name === 'Plan')!;
    expect(plan.when).toEqual([{ expr: 'remaining > 1', outcome: 'next' }, { expr: 'remaining == 1', outcome: 'next' }]); expect(plan.otherwise).toBe('empty');
    ed().select({ type: 'node', id: 'Pick' }); change(fieldIn('Entry actions (one per line)'), 'set=phase value=pick'); change(fieldIn('Timeout (s, → timeout outcome)'), '90');
    const pick = parseFsm(src).root.states.find((s) => s.name === 'Cycle')!.machine!.states.find((s) => s.name === 'Pick')!;
    expect(pick.entry[0]).toMatchObject({ kind: 'set', value: 'phase', args: { value: 'pick' } }); expect(pick.timeout).toBe(90); expect(pick.exit[0]).toMatchObject({ kind: 'set', value: 'remaining' });
    ed().setBlock(0); button(editor.el, '+ Machine').click();
    const doc = parseFsm(src); const m = doc.root.states.find((s) => s.kind === 'machine' && s.name !== 'Cycle')!; expect(m.machine!.outcomes).toEqual(['done']); expect(m.machine!.states[0].transitions).toEqual({ ok: 'done' });
    expect([...editor.el.querySelectorAll<HTMLSelectElement>('select.ge-block')[0].options].map((o) => o.textContent)).toContain(`machine Pick_all_boxes/${m.name}`);
    plan = doc.root.states.find((s) => s.name === 'Cycle')!.machine!.states.find((s) => s.name === 'Plan')!; expect(plan.otherwise).toBe('empty'); // other machines untouched
  });
});

describe('Petri diagram write-back keeps the run-time lines', () => {
  it('final markings, sensors, blackboard, guards and servers survive an edit in the diagram', () => {
    const net = 'petri p\nplace a tokens=1\nplace b\ntransition t guard="part_present" servers=2\narc a -> t -> b\nfinal b\nsensor arrived "x > 1"\nblackboard count=0';
    ed().load('petri', net); ed().moveNode('a', 100, 120);
    expect(src).toMatch(/^final b$/m); expect(src).toMatch(/^sensor arrived "x > 1"$/m); expect(src).toMatch(/^blackboard count=0$/m); expect(src).toMatch(/transition t servers=2 guard="part_present"/);
  });
});
