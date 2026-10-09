import { describe, it, expect } from 'vitest';
import { parseDes, parsePetri, TEMPLATES } from '../src/ctl/dsl';
import { desToDsl, petriToDsl, addState, removeState, renameState, setEdgeEvents, edgeEvents, toggleMarked, setInitial, newAutomaton, addPlace, addPetriTransition, setArc, removeArc, removePetriNode, renamePetriNode, uniqueName, automatonEdges, setAlphabet } from '../src/ctl/graphdoc';
import { analyse } from '../src/ctl/analysis';
import { COURSE_EXAMPLES, YOUBOT_DES_DSL, CELL_PETRI_DSL } from '../src/ctl/examples_dsl';
import { layoutGraph } from '../src/ui/graph_layout';
import { DES, parallel } from '../src/ctl/des';
import type { PetriNetSpec } from '../src/ctl/petri';

const norm = (a: ReturnType<typeof parseDes>) => ({ name: a.name, plant: a.plant.map((x) => ({ name: x.name, initial: x.initial, marked: [...x.marked].sort(), events: [...x.events].sort(), tr: x.transitions.map((t) => `${t.from} ${t.event} ${t.to}`).sort() })), specs: a.specs.map((x) => ({ name: x.name, initial: x.initial, marked: [...x.marked].sort(), events: [...x.events].sort(), tr: x.transitions.map((t) => `${t.from} ${t.event} ${t.to}`).sort() })), uc: [...a.uncontrollable].sort(), uo: a.unobservable, faults: a.faults, checks: a.checks });

describe('Graph document model (editors on top of the DSL)', () => {
  it('DES documents round-trip through desToDsl with identical structure, analysis and layout', () => {
    for (const ex of COURSE_EXAMPLES.filter((e) => e.kind === 'des')) {
      const doc = parseDes(ex.source);
      doc.layout = { Base: { H: [100, 80], mT: [220.4, 80.6] } };
      const text = desToDsl(doc);
      const again = parseDes(text);
      expect(norm(again)).toEqual(norm(doc));
      expect(again.layout).toEqual({ Base: { H: [100, 80], mT: [220.4, 80.6] } });
      expect(analyse('des', text).metrics['supervisor states']).toBe(analyse('des', ex.source).metrics['supervisor states']);
    }
    // a spec that only declares an alphabet (E4) survives: `events` line without transitions
    const e4 = parseDes(COURSE_EXAMPLES.find((e) => e.id === 'A-des-e4')!.source);
    const spec = e4.specs.find((s) => s.name === 'E4')!; expect(spec.events).toEqual(['g_slip']); expect(spec.transitions.length).toBe(0);
    expect(desToDsl(e4)).toContain('spec E4\n  events g_slip');
  });
  it('Petri documents round-trip (grouped arcs, weights, inhibitors, layout) with the same analysis', () => {
    const p = parsePetri(CELL_PETRI_DSL);
    const doc = { spec: p.spec, checks: p.checks, horizon: p.horizon, layout: { p1: [50, 60] as [number, number], t1a: [150, 60] as [number, number] }, actions: p.actions };
    const text = petriToDsl(doc); const again = parsePetri(text);
    expect(again.spec.places.map((x) => x.id)).toEqual(p.spec.places.map((x) => x.id)); expect(again.spec.arcs.length).toBe(p.spec.arcs.length);
    expect(again.layout).toEqual(doc.layout); expect(again.checks).toEqual(p.checks);
    const a = analyse('petri', text), b = analyse('petri', CELL_PETRI_DSL); expect(a.metrics).toEqual(b.metrics);
    // special arcs are written individually
    setArc(p.spec, 'a1', 't2a', { weight: 2 }); setArc(p.spec, 'rZ', 't1a', { inhibitor: true });
    const t2 = petriToDsl(doc); expect(t2).toContain('arc a1 -> t2a weight=2'); expect(t2).toContain('arc rZ -o t1a');
    const p2 = parsePetri(t2); expect(p2.spec.arcs.find((x) => x.from === 'a1' && x.to === 't2a')!.weight).toBe(2); expect(p2.spec.arcs.find((x) => x.from === 'rZ' && x.to === 't1a')!.inhibitor).toBe(true);
    expect(parsePetri(TEMPLATES.petri.replace('petri Two processes sharing a robot and a zone', 'petri Two processes horizon=120')).horizon).toBe(120);
  });
  it('automaton edits: states, edges, marking, renaming, alphabet', () => {
    const a = newAutomaton('M');
    expect(a.initial).toBe('q0'); const s1 = addState(a); expect(s1).toBe('q'); const s2 = addState(a, 'W');
    setEdgeEvents(a, 'q0', 'W', ['start', 'start']); setEdgeEvents(a, 'W', 'q0', ['finish']); setEdgeEvents(a, 'q', 'q', ['idle']);
    expect(edgeEvents(a, 'q0', 'W')).toEqual(['start']); expect(a.events.sort()).toEqual(['finish', 'idle', 'start']);
    expect(automatonEdges(a).length).toBe(3);
    expect(() => addState(a, 'W')).toThrow(/exists/); expect(() => renameState(a, 'q', 'W')).toThrow(/exists/); expect(() => renameState(a, 'q', 'a b')).toThrow();
    renameState(a, 'q0', 'I'); expect(a.initial).toBe('I'); expect(a.marked).toEqual(['I']); expect(edgeEvents(a, 'I', 'W')).toEqual(['start']);
    toggleMarked(a, 'W'); expect(a.marked).toEqual(['I', 'W']); toggleMarked(a, 'W', false); expect(a.marked).toEqual(['I']);
    setInitial(a, 'W'); setEdgeEvents(a, 'W', 'I', []); expect(edgeEvents(a, 'W', 'I')).toEqual([]);
    setAlphabet(a, ['extra']); expect(a.events.sort()).toEqual(['extra', 'idle', 'start']);
    removeState(a, s2); expect(a.transitions.every((t) => t.from !== 'W' && t.to !== 'W')).toBe(true); expect(a.initial).toBe('I');
    const text = desToDsl({ name: 'x', plant: [a], specs: [], uncontrollable: ['idle'], unobservable: [], faults: [], checks: [{ kind: 'ltl', formula: 'G F M=\'I\'', name: 'live' }], actions: [] });
    const back = parseDes(text); expect(back.plant[0].initial).toBe('I'); expect(back.uncontrollable).toEqual(['idle']); expect(back.checks[0]).toEqual({ kind: 'ltl', formula: "G F M='I'", name: 'live' });
    expect(uniqueName(['p', 'p1'], 'p')).toBe('p2');
    // the composition of the edited plant is still analysable
    expect(parallel(...back.plant.map((x) => DES.fromSpec(x))).X.size).toBeGreaterThan(0);
  });
  it('Petri edits: places, transitions, arcs (bipartite only), renaming, removal', () => {
    const spec: PetriNetSpec = { name: 'n', places: [], transitions: [], arcs: [] };
    const p = addPlace(spec, undefined, 'resource'); const t = addPetriTransition(spec); const p2 = addPlace(spec);
    expect([p, t, p2]).toEqual(['p', 't', 'p1']);
    setArc(spec, p, t); setArc(spec, t, p2, { weight: 3 });
    expect(() => setArc(spec, p, p2)).toThrow(/place and a transition/); expect(() => setArc(spec, t, p2, { inhibitor: true })).toThrow(/inhibitor/);
    renamePetriNode(spec, t, 'fire'); expect(spec.arcs.map((a) => `${a.from}>${a.to}`)).toEqual(['p>fire', 'fire>p1']);
    expect(() => renamePetriNode(spec, 'p', 'p1')).toThrow(/exists/); expect(() => renamePetriNode(spec, 'p', '1x')).toThrow();
    removeArc(spec, 'fire', 'p1'); expect(spec.arcs.length).toBe(1); setArc(spec, 'fire', 'p1');
    removePetriNode(spec, 'p'); expect(spec.arcs.map((a) => `${a.from}>${a.to}`)).toEqual(['fire>p1']); expect(spec.places.map((x) => x.id)).toEqual(['p1']);
    const text = petriToDsl({ spec, checks: [], horizon: 300, layout: {}, actions: [] }); expect(text).toContain('transition fire');
    const r = analyse('petri', text); expect(r.error).toBeUndefined(); expect(r.sections[0].title).toBe('Structure'); // a source transition: unbounded, reported, not a crash
  });
  it('layout: pinned nodes stay, the others are placed around them; trees go left to right', () => {
    const g = { kind: 'automaton' as const, nodes: ['a', 'b', 'c', 'd'].map((id) => ({ id, label: id, initial: id === 'a' })), edges: [{ from: 'a', to: 'b' }, { from: 'b', to: 'c' }, { from: 'c', to: 'd' }, { from: 'd', to: 'a' }] };
    const l1 = layoutGraph(g); expect(l1.pos.size).toBe(4); expect(l1.width).toBeGreaterThanOrEqual(320);
    const l2 = layoutGraph(g, 320, new Map([['a', [100, 100]], ['b', [300, 100]]])); expect(l2.pos.get('a')).toEqual([100, 100]); expect(l2.pos.get('b')).toEqual([300, 100]);
    for (const [, p] of l2.pos) { expect(p[0]).toBeGreaterThanOrEqual(40); expect(p[1]).toBeGreaterThanOrEqual(40); }
    const tree = layoutGraph({ kind: 'tree', nodes: ['r', 'x', 'y'].map((id) => ({ id, label: id })), edges: [{ from: 'r', to: 'x' }, { from: 'r', to: 'y' }] });
    expect(tree.tree).toBe(true); expect(tree.pos.get('r')![0]).toBeLessThan(tree.pos.get('x')![0]); expect(tree.pos.get('x')![1]).toBeLessThan(tree.pos.get('y')![1]);
  });
  it('the course DES still composes to 648 states after a write-back', () => {
    const doc = parseDes(YOUBOT_DES_DSL); const back = parseDes(desToDsl(doc));
    expect(parallel(...back.plant.map((x) => DES.fromSpec(x))).X.size).toBe(648);
  });
});
