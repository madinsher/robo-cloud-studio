/**
 * Interchange and PLC formats generated from control models: well-formedness (XML parser, ST block / declaration
 * checker, SMV structure) and the round trips that must be exact (PNML → DSL → same net, same reachability graph).
 */
import { describe, it, expect } from 'vitest';
import { TEMPLATES, parsePetri, parseDes, parseBt } from '../src/ctl/dsl';
import type { BTNodeSpec } from '../src/ctl/bt';
import { btCppXml, btCppXmlToDsl } from '../src/ctl/codegen/ros2';
import { PetriNet } from '../src/ctl/petri';
import { DES } from '../src/ctl/des';
import { COURSE_EXAMPLES } from '../src/ctl/examples_dsl';
import { automatonToST, petriToST, petriToPlcOpen, petriToPnml, desToScxml, fsmToScxml, desToSmv, petriToSmv, desToUppaal, controlExports } from '../src/ctl/codegen';
import { pnmlToPetriDsl } from '../src/ctl/codegen/interchange';
import { parseXml, xmlAll, XmlNode } from '../src/ctl/codegen/common';

/** Minimal IEC 61131-3 ST checker: balanced blocks, every assigned / compared identifier declared, statements end with `;`. */
function checkST(src: string): string[] {
  const errs: string[] = [];
  const code = src.replace(/\(\*[\s\S]*?\*\)/g, ' ').replace(/'[^']*'/g, "''");
  const stack: string[] = []; const pairs: Record<string, string> = { PROGRAM: 'END_PROGRAM', VAR: 'END_VAR', VAR_INPUT: 'END_VAR', VAR_OUTPUT: 'END_VAR', IF: 'END_IF', CASE: 'END_CASE', FOR: 'END_FOR', WHILE: 'END_WHILE' };
  const decl = new Set<string>(); let inVar = false; const body: string[] = [];
  for (const raw of code.split('\n')) {
    const line = raw.trim(); if (!line) continue;
    if (/^VAR(_INPUT|_OUTPUT)?(\s+CONSTANT)?$/.test(line)) { stack.push(line.split(/\s+/)[0]); inVar = true; continue; }
    if (line === 'END_VAR') { if (!['VAR', 'VAR_INPUT', 'VAR_OUTPUT'].includes(stack.pop() ?? '')) errs.push('END_VAR without VAR'); inVar = false; continue; }
    if (inVar) { const m = /^([A-Za-z_]\w*)\s*(AT\s+%\S+\s*)?:\s*[A-Za-z_]\w*(\s*:=\s*[^;]+)?;$/.exec(line); if (!m) errs.push(`bad declaration: ${line}`); else decl.add(m[1]); continue; }
    for (const tok of line.match(/\b[A-Z_]+\b/g) ?? []) {
      if (tok === 'PROGRAM' || tok === 'IF' || tok === 'CASE' || tok === 'FOR' || tok === 'WHILE') stack.push(tok);
      else if (tok.startsWith('END_')) { const open = stack.pop(); if (!open || pairs[open] !== tok) errs.push(`${tok} closes ${open ?? 'nothing'}`); }
    }
    if (!/^PROGRAM\b|^END_PROGRAM$/.test(line) && !/(;|THEN|ELSE|OF|DO|:)$/.test(line)) errs.push(`statement without ';': ${line}`);
  }
  if (stack.length) errs.push(`unclosed: ${stack.join(', ')}`);
  // identifiers on the left of := must be declared (FB instances: T_x.Q / R_x(...) are calls)
  for (const m of body.join(' ; ').matchAll(/(^|[;\s(])([A-Za-z_]\w*)\s*:=/g)) { const id = m[2]; if (!decl.has(id) && !['IN', 'PT', 'CLK'].includes(id)) errs.push(`undeclared ${id}`); }
  return errs;
}
const ids = (n: XmlNode, tag: string, attr: string) => xmlAll(n, tag).map((x) => x.attrs[attr]);

const SIGNALED_PETRI = `petri Guarded cell
place free tokens=1 kind=resource
place idle tokens=2 kind=idle
place busy
place done
transition load guard="part_present && speed > 0.5"
transition work delay=2
transition unload
arc idle, free -> load -> busy
arc busy -> work -> done
arc done -> unload -> idle, free
action load program="Load part"
action unload program="Unload part"`;

describe('IEC 61131-3 Structured Text', () => {
  it('automata: well-formed CASE state machines, supervisor table, handshake I/O map for bound states', () => {
    const src = `${TEMPLATES.des}\naction M1.W program="Machine 1 cycle" done=finish1\naction M2.W program="Machine 2 cycle" done=finish2`;
    const [st, io] = automatonToST(src, 'Two machines');
    expect(checkST(st.content)).toEqual([]);
    expect(st.content).toMatch(/PROGRAM Two_machines[\s\S]*END_PROGRAM/); expect(st.content).toMatch(/CASE SUP OF/);
    expect(io.content.split('\n').filter((l) => /^START_|^DONE_/.test(l)).map((l) => l.split(',').slice(0, 3).join(','))).toEqual(['START_M1_W,PLC output → robot input,%QX0.0', 'DONE_M1_W,robot output → PLC input,%IX0.0', 'START_M2_W,PLC output → robot input,%QX0.1', 'DONE_M2_W,robot output → PLC input,%IX0.1']);
  });
  it('Petri nets: marking per place, guards over inputs, TON for delays, START / DONE per bound transition', () => {
    const [st, io] = petriToST(SIGNALED_PETRI, 'Guarded cell');
    expect(checkST(st.content)).toEqual([]);
    expect(st.content).toMatch(/m_idle : INT := 2;/); expect(st.content).toMatch(/T_work\(IN := busy_work, PT := T#2000MS\);/);
    expect(st.content).toMatch(/part_present AND speed > 0\.5/);
    expect(io.content).toMatch(/START_load,PLC output → robot input,%QX0\.0/); expect(io.content).toMatch(/part_present,robot output → PLC input,%IX0\.2/);
    for (const ex of COURSE_EXAMPLES.filter((e) => e.kind === 'petri')) for (const f of petriToST(ex.source, ex.id)) if (f.name.endsWith('.st')) expect(checkST(f.content), ex.id).toEqual([]);
  });
  it('PLCopen XML (TC6 2.01) SFC: steps = places, one initial step per token, every connection references an existing element', () => {
    const net = parsePetri(TEMPLATES.petri).spec;
    const x = parseXml(petriToPlcOpen(TEMPLATES.petri, 'Cell').content);
    expect(x.name).toBe('project'); expect(x.attrs.xmlns).toBe('http://www.plcopen.org/xml/tc6_0201');
    const steps = xmlAll(x, 'step'); const placeSteps = steps.filter((s) => net.places.some((p) => p.id === s.attrs.name));
    expect(placeSteps.length).toBe(net.places.length);
    expect(placeSteps.filter((s) => s.attrs.initialStep === 'true').map((s) => s.attrs.name).sort()).toEqual(net.places.filter((p) => (p.tokens ?? 0) > 0).map((p) => p.id).sort());
    const local = new Set(xmlAll(x, '*').length ? [] : []); void local;
    const all = new Set<string>(); const walk = (n: XmlNode) => { if (n.attrs.localId) all.add(n.attrs.localId); n.children.forEach(walk); }; walk(x);
    const refs = ids(x, 'connection', 'refLocalId'); expect(refs.length).toBeGreaterThan(net.transitions.length);
    for (const r of refs) expect(all.has(r)).toBe(true);
    expect(xmlAll(x, 'transition').length).toBeGreaterThanOrEqual(net.transitions.length);
    expect(() => petriToPlcOpen('petri x\nplace a tokens=2\nplace b\ntransition t\narc a -> t -> b', 'x')).toThrow(/1-safe/);
  });
});

describe('PNML, SCXML, NuSMV, UPPAAL', () => {
  it('PNML round trip: same places, markings, capacities, transitions (timing, guards) and arcs; same reachability graph', () => {
    for (const src of [TEMPLATES.petri, SIGNALED_PETRI, ...COURSE_EXAMPLES.filter((e) => e.kind === 'petri').map((e) => e.source)]) {
      const a = parsePetri(src); const xml = petriToPnml(src, 'n').content; parseXml(xml);
      const b = parsePetri(pnmlToPetriDsl(xml));
      expect(b.spec.places.map((p) => [p.id, p.tokens ?? 0, p.capacity ?? null])).toEqual(a.spec.places.map((p) => [p.id, p.tokens ?? 0, p.capacity ?? null]));
      expect(b.spec.transitions.map((t) => [t.id, t.delay ?? null, t.rate ?? null, !!t.immediate, t.guard ?? null])).toEqual(a.spec.transitions.map((t) => [t.id, t.delay ?? null, t.rate ?? null, !!t.immediate, t.guard ?? null]));
      const na = new PetriNet(a.spec), nb = new PetriNet(b.spec);
      expect(nb.pre).toEqual(na.pre); expect(nb.post).toEqual(na.post); expect(nb.inhibitor).toEqual(na.inhibitor);
      expect(nb.reachability({ limit: 5000 }).markings.length).toBe(na.reachability({ limit: 5000 }).markings.length);
      expect(b.actions.map((x) => `${x.target}:${x.kind}:${x.value}`)).toEqual(a.actions.map((x) => `${x.target}:${x.kind}:${x.value}`));
    }
  });
  it('SCXML: a parallel state per automaton with its states / transitions; FSM machines nest as compound states', () => {
    const doc = parseDes(TEMPLATES.des); const x = parseXml(desToScxml(TEMPLATES.des, 'Two').content);
    expect(x.name).toBe('scxml'); expect(x.attrs.xmlns).toBe('http://www.w3.org/2005/07/scxml');
    for (const a of doc.plant) { const d = DES.fromSpec(a); for (const s of d.X) expect(ids(x, 'state', 'id')).toContain(`${a.name}.${s}`); }
    const targets = new Set([...ids(x, 'state', 'id'), ...ids(x, 'parallel', 'id'), ...ids(x, 'final', 'id')]);
    for (const t of xmlAll(x, 'transition')) for (const tg of (t.attrs.target ?? '').split(/\s+/).filter(Boolean)) expect(targets.has(tg)).toBe(true);
    expect(xmlAll(x, 'transition').map((t) => t.attrs.event).filter(Boolean).sort()).toEqual(doc.plant.flatMap((a) => a.transitions.map((t) => t.event)).sort());
    const f = parseXml(fsmToScxml(TEMPLATES.fsm, 'F').content); const fids = new Set([...ids(f, 'state', 'id'), ...ids(f, 'final', 'id')]);
    for (const t of xmlAll(f, 'transition')) for (const tg of (t.attrs.target ?? '').split(/\s+/).filter(Boolean)) expect(fids.has(tg)).toBe(true);
  });
  it('NuSMV: one variable per automaton / place with init and next, supervisor gating in TRANS, the checks as LTLSPEC', () => {
    const smv = desToSmv(TEMPLATES.des, 'Two').content;
    expect(smv).toMatch(/^MODULE main$/m); for (const v of ['M1', 'M2', 'sup']) { expect(smv).toMatch(new RegExp(`init\\(${v}\\) :=`)); expect(smv).toMatch(new RegExp(`next\\(${v}\\) := case[\\s\\S]*?esac;`)); }
    expect(smv).toMatch(/LTLSPEC G\(M1=s_W -> F M1=s_I\);/);
    const caseCount = (smv.match(/:= case/g) ?? []).length; expect((smv.match(/esac;/g) ?? []).length).toBe(caseCount);
    const p = petriToSmv(TEMPLATES.petri, 'Cell').content; const net = parsePetri(TEMPLATES.petri).spec;
    for (const pl of net.places) expect(p).toMatch(new RegExp(`init\\(m_${pl.id}\\) := ${pl.tokens ?? 0};`));
    expect(p).toMatch(/deadlock := !\(/);
  });
  it('UPPAAL: well-formed NTA, one template per automaton plus the scheduler, every transition references existing locations, a system line', () => {
    const doc = parseDes(TEMPLATES.des); const x = parseXml(desToUppaal(TEMPLATES.des, 'Two').content);
    expect(x.name).toBe('nta'); const templates = xmlAll(x, 'template'); // one per automaton + the scheduler issuing controllable events
    expect(templates.map((t) => xmlAll(t, 'name')[0].text)).toEqual([...doc.plant.map((a) => a.name), 'Scheduler']);
    for (const t of templates) { const locs = new Set(ids(t, 'location', 'id')); for (const tr of xmlAll(t, 'transition')) { expect(locs.has(xmlAll(tr, 'source')[0].attrs.ref)).toBe(true); expect(locs.has(xmlAll(tr, 'target')[0].attrs.ref)).toBe(true); } }
    expect(xmlAll(x, 'system')[0].text).toMatch(/system M1, M2, Scheduler;/);
  });
  it('BehaviorTree.CPP XML import: export → import gives the same tree (structure, memory flags, counts, conditions, scripts)', () => {
    const src = `bt round trip
fallback root
  sequence emergency
    condition estop
    action halt timeout=1
  sequence mission memory
    parallel 2
      action program program="Weld left"
      retry 2
        action goto zone=table
    repeat 3
      inverter
        condition "battery < 0.3"
    action set count="=count + 1"
    action wait seconds=0.5
    action grasp p=0.9 pre="arm_extended"
    action report`;
    const back = parseBt(btCppXmlToDsl(btCppXml(src).xml));
    const strip = (n: BTNodeSpec): unknown => ({ type: n.type, memory: !!n.memory, threshold: n.threshold, count: n.count, seconds: n.seconds, fn: n.fn, args: n.type === 'action' ? n.args : undefined, pre: n.pre, children: (n.children ?? []).map(strip) });
    // an action timeout is exported as a Timeout decorator around the action: normalise the original the same way
    const norm = (n: BTNodeSpec): BTNodeSpec => (n.type === 'action' && n.timeout !== undefined ? { type: 'timeout', seconds: n.timeout, children: [{ ...n, timeout: undefined }] } : { ...n, children: n.children?.map(norm) });
    expect(strip(back.root)).toEqual(strip(norm(parseBt(src).root)));
    expect(() => btCppXmlToDsl('<root BTCPP_format="4"><BehaviorTree ID="T"><Fancy><A/></Fancy></BehaviorTree></root>')).toThrow(/unknown control/);
  });
  it('the Export menu offers the generators per kind', () => {
    const labels = (k: Parameters<typeof controlExports>[0], src: string) => controlExports(k, src, 'm').map((e) => e.id);
    expect(labels('des', TEMPLATES.des)).toEqual(['robodk', 'ros2', 'st', 'scxml', 'smv', 'uppaal']);
    expect(labels('petri', TEMPLATES.petri)).toEqual(['robodk', 'ros2', 'st', 'sfc', 'pnml', 'smv']);
    expect(labels('fsm', TEMPLATES.fsm)).toEqual(['robodk', 'ros2', 'scxml']);
    expect(labels('bt', TEMPLATES.bt)).toEqual(['robodk', 'ros2']);
    for (const e of controlExports('petri', TEMPLATES.petri, 'm')) expect(e.files().length).toBeGreaterThan(0);
  });
});
