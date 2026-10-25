/**
 * `check ltl|ctl` lines through diagram edits and analysis:
 *  - a quoted atom inside a formula is not the check's name (BT template: G("ok:estop" -> "tick:halt") survived no edit);
 *  - an atom over an automaton / place / state the model does not have is an error, not a vacuous ✅;
 *  - renaming an automaton or a state in the diagram updates the checks (and the bindings) that use the name.
 */
import { describe, it, expect } from 'vitest';
import { TEMPLATES, parseBt, parseDes, parsePetri, parseCheckLine } from '../src/ctl/dsl';
import { btToDsl, desToDsl, renameAutomatonRefs, renameStateRefs, renameState } from '../src/ctl/graphdoc';
import { analyse } from '../src/ctl/analysis';
import { desToSmv } from '../src/ctl/codegen/interchange';

const sectionsOf = (kind: Parameters<typeof analyse>[0], src: string) => analyse(kind, src).sections as Array<{ title: string; level: string; lines?: string[] }>;

describe('check lines', () => {
  it('a quoted atom in the formula is not taken as the name; named checks keep their name', () => {
    expect(parseCheckLine('check ltl G("ok:estop" -> "tick:halt")')).toEqual({ kind: 'ltl', formula: 'G("ok:estop" -> "tick:halt")' });
    expect(parseCheckLine('check ltl "safety" G(!a)')).toEqual({ kind: 'ltl', formula: 'G(!a)', name: 'safety' });
    expect(parseCheckLine('check ctl "a" -> AF "b"')).toEqual({ kind: 'ctl', formula: '"a" -> AF "b"' });
  });
  it('BT template: a diagram edit (rename the root) keeps the LTL check; the analysis is unchanged', () => {
    const doc = parseBt(TEMPLATES.bt); const before = doc.checks.map((c) => ({ ...c }));
    doc.root.name = 'mission';
    const src = btToDsl(doc);
    expect(src).toContain('check ltl G("ok:estop" -> "tick:halt")');
    expect(parseBt(src).checks).toEqual(before);
    const ltl = sectionsOf('bt', src).filter((s) => /^LTL/.test(s.title));
    expect(ltl.length).toBeGreaterThan(0);
    for (const s of ltl) expect(s.title).not.toMatch(/->\s*\)/);
    expect(sectionsOf('bt', src).filter((s) => /^LTL/.test(s.title)).map((s) => s.level)).toEqual(sectionsOf('bt', TEMPLATES.bt).filter((s) => /^LTL/.test(s.title)).map((s) => s.level));
  });
  it('LTL over an automaton or a state the model does not have is reported as an error', () => {
    const bad = `${TEMPLATES.des}\ncheck ltl G(NoSuchAutomaton='Q' -> F NoSuchAutomaton='Z')`;
    const s = sectionsOf('des', bad).find((x) => x.title.includes('NoSuchAutomaton'))!;
    expect(s.level).toBe('error'); expect(s.lines!.join(' ')).toMatch(/unknown name 'NoSuchAutomaton'/);
    const badState = `${TEMPLATES.des}\ncheck ltl G(M1='Nowhere' -> F M1='I')`;
    expect(sectionsOf('des', badState).find((x) => x.title.includes('Nowhere'))!.level).toBe('error');
    // the template's own check still holds
    expect(sectionsOf('des', TEMPLATES.des).find((x) => x.title.includes("M1='W'"))!.level).toBe('ok');
    const petri = `${TEMPLATES.petri}\ncheck ctl AG !(nosuchplace >= 1)`;
    expect(sectionsOf('petri', petri).find((x) => x.title.includes('nosuchplace'))!.level).toBe('error');
  });
  it('renaming an automaton / a state in the diagram updates checks and bindings; the SMV export stays consistent', () => {
    const doc = parseDes(TEMPLATES.des); const a = doc.plant.find((x) => x.name === 'M1')!;
    doc.actions.push({ target: 'M1.W', kind: 'program', value: 'Work', args: {} } as never);
    renameAutomatonRefs(doc, 'M1', 'RA'); a.name = 'RA';
    expect(doc.checks.map((c) => c.formula)).toContain("G(RA='W' -> F RA='I')");
    expect(doc.actions.some((x) => x.target === 'RA.W')).toBe(true);
    renameState(a, 'W', 'Work'); renameStateRefs(doc, 'RA', 'W', 'Work');
    expect(doc.checks.map((c) => c.formula)).toContain("G(RA='Work' -> F RA='I')");
    const src = desToDsl(doc);
    expect(sectionsOf('des', src).find((x) => x.title.includes("RA='Work'"))!.level).toBe('ok');
    const smv = desToSmv(src, 'm').content;
    expect(smv).not.toMatch(/\bM1\b/); expect(smv).toMatch(/LTLSPEC/);
  });
  it('a Petri net check is parsed by the shared parser too', () => {
    expect(parsePetri(`${TEMPLATES.petri}\ncheck ctl "no overflow" AG !(a1 >= 2)`).checks.at(-1)).toEqual({ kind: 'ctl', formula: 'AG !(a1 >= 2)', name: 'no overflow' });
  });
});
