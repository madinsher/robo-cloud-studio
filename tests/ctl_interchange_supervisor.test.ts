/**
 * Model-checker exports keep what the studio run enforces: the UPPAAL model of an automaton with specifications
 * carries the synthesised supervisor (B may not start before A finished, as in the studio, the SMV and the SCXML
 * exports); the Petri SMV keeps transition guards as environment inputs instead of dropping them.
 */
import { describe, it, expect } from 'vitest';
import { desToUppaal, petriToSmv } from '../src/ctl/codegen/interchange';
import { synthesizeSupervisor } from '../src/ctl/supervisor';
import { parseXml, xmlAll } from '../src/ctl/codegen/common';

const DES = `des Two machines and a buffer
automaton RA
  initial I
  marked I F
  I -sA-> P
  P -fA-> D
  D -sC-> Q
  Q -fC-> F
automaton RB
  initial I
  marked I F
  I -sB-> W
  W -fB-> F
spec Buffer
  events fA sB
  initial E
  marked E G
  E -fA-> A
  A -sB-> G
uncontrollable fA fB fC`;

/** Explore the UPPAAL model's discrete part (ints, en_e() guards, assignments) by interpreting its declarations. */
function explore(xml: string): { seq: (events: string[]) => boolean } {
  const root = parseXml(xml);
  const decl = xmlAll(root, 'declaration')[0].text ?? '';
  const ints = new Map([...decl.matchAll(/^int (\w+) = (\d+);/gm)].map((m) => [m[1], Number(m[2])]));
  const fn = (src: string) => new Function(...[...ints.keys()], `return (${src});`) as (...a: number[]) => boolean;
  const en = new Map([...decl.matchAll(/^bool en_(\w+)\(\) \{ return (.*); \}/gm)].map((m) => [m[1], fn(m[2].replace(/\|\|/g, '||').replace(/&&/g, '&&'))]));
  const supFns = new Map([...decl.matchAll(/^int sup_(\w+)\(int s\) \{ (.*) return s; \}/gm)].map((m) => [m[1], new Function('s', `${m[2]} return s;`) as (s: number) => number]));
  const templates = xmlAll(root, 'template');
  return {
    seq: (events) => {
      const st = new Map(ints);
      for (const e of events) {
        if (!en.get(e)!(...[...st.values()])) return false;
        for (const t of templates) {
          const name = t.children.find((c) => c.name === 'name')?.text;
          for (const tr of t.children.filter((c) => c.name === 'transition')) {
            const sync = tr.children.find((c) => c.attrs.kind === 'synchronisation')?.text;
            if (sync !== `e_${e}?`) continue;
            const src = tr.children.find((c) => c.name === 'source')!.attrs.ref; const loc = t.children.find((c) => c.name === 'location' && c.attrs.id === src)!.children.find((c) => c.name === 'name')!.text;
            const cur = [...st.entries()].find(([k]) => k === `s_${name}`)![1];
            const codes = new Map([...decl.matchAll(new RegExp(`^int s_${name} = \\d+; // (.*)$`, 'gm'))][0][1].split(' ').map((p) => { const [i, x] = p.split('='); return [x, Number(i)] as const; }));
            if (codes.get(loc!) !== cur) continue;
            const asg = tr.children.find((c) => c.attrs.kind === 'assignment')!.text!; const m = /s_\w+ = (\d+)/.exec(asg)!; st.set(`s_${name}`, Number(m[1])); break;
          }
        }
        if (supFns.has(e)) st.set('sup', supFns.get(e)!(st.get('sup')!));
      }
      return true;
    },
  };
}

describe('UPPAAL / SMV exports', () => {
  it('UPPAAL: the supervisor forbids sB before fA (studio semantics); the allowed order is accepted', () => {
    const xml = desToUppaal(DES, 'cell').content;
    expect(xml).toMatch(/int sup = \d+;/); expect(xml).toMatch(/sup = sup_sB\(sup\)/);
    const sup = synthesizeSupervisor(DES)!; expect(sup.table.states.length).toBeGreaterThan(1);
    const m = explore(xml);
    expect(m.seq(['sB'])).toBe(false); // B may not start first
    expect(m.seq(['sA', 'sB'])).toBe(false); // nor before A finished
    expect(m.seq(['sA', 'fA', 'sB', 'sC', 'fB', 'fC'])).toBe(true);
  });
  it('Petri SMV: a guarded transition is enabled only with its environment input', () => {
    const net = `petri g\nplace a2 tokens=1\nplace done\ntransition tC guard="Ready == 1"\narc a2 -> tC -> done`;
    const smv = petriToSmv(net, 'g').content;
    expect(smv).toMatch(/-- tC: guard "Ready == 1" -> environment input g_t_tC/);
    expect(smv).toMatch(/IVAR[\s\S]*g_t_tC : boolean;/);
    expect(smv).toMatch(/en_t_tC := m_a2 >= 1 & g_t_tC;/);
  });
});
