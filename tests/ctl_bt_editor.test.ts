import { describe, it, expect } from 'vitest';
import { parseBt, TEMPLATES } from '../src/ctl/dsl';
import { btToDsl, btWalk, btAddChild, btRemove, btMove, btReparent, btOutdent, btSetType, btLabel, newBtNode } from '../src/ctl/graphdoc';
import { analyse } from '../src/ctl/analysis';
import { YOUBOT_BT_DSL } from '../src/ctl/examples_dsl';
import type { BTNodeSpec } from '../src/ctl/bt';

const shape = (n: BTNodeSpec): unknown => ({ type: n.type, name: n.name, fn: n.fn, args: n.args ?? {}, timeout: n.timeout, pre: n.pre, post: n.post, memory: !!n.memory, threshold: n.threshold, count: n.count, seconds: n.seconds, children: (n.children ?? []).map(shape) });

describe('Behavior-tree document model (graphical editor on top of the DSL)', () => {
  it('the course tree and the template round-trip through btToDsl with the same structure and analysis', () => {
    for (const src of [YOUBOT_BT_DSL, TEMPLATES.bt]) {
      const doc = parseBt(src); const text = btToDsl(doc); const again = parseBt(text);
      expect(shape(again.root)).toEqual(shape(doc.root));
      expect(again.outcomes).toEqual(doc.outcomes); expect(again.leafModels).toEqual(doc.leafModels); expect(again.monitors).toEqual(doc.monitors); expect(again.checks).toEqual(doc.checks); expect(again.supervisor).toEqual(doc.supervisor); expect(again.name).toBe(doc.name);
      const a = analyse('bt', text), b = analyse('bt', src); expect(a.error).toBeUndefined(); expect(a.metrics['nodes']).toBe(b.metrics['nodes']); expect(a.metrics['abstract states']).toBe(b.metrics['abstract states']); expect(a.ok).toBe(b.ok);
    }
    expect(btToDsl(parseBt(YOUBOT_BT_DSL))).toContain('    condition "battery < 0.2"');
    expect(btToDsl(parseBt(YOUBOT_BT_DSL))).toContain('        action grasp event=g_close ok=g_ok miss=g_miss timeout=12 post="held"');
  }, 60000);
  it('node operations: add, reorder, reparent, outdent, retype, remove', () => {
    const doc = parseBt(TEMPLATES.bt); const root = doc.root;
    const mission = root.children!.find((c) => c.name === 'mission')!;
    const seq = btAddChild(mission, 'sequence'); expect(seq.name).toBe('sequence'); expect(mission.children!.at(-1)).toBe(seq);
    const act = btAddChild(seq, 'action'); expect(act.fn).toBe('wait'); expect(() => btAddChild(act, 'action')).toThrow(/no children/);
    const dec = btAddChild(seq, 'retry'); btAddChild(dec, 'condition'); expect(() => btAddChild(dec, 'action')).toThrow(/exactly one/);
    btMove(root, seq, -1); expect(mission.children!.indexOf(seq)).toBe(mission.children!.length - 2);
    const emergency = root.children![0];
    btReparent(root, seq, emergency); expect(emergency.children!.at(-1)).toBe(seq); expect(mission.children!.includes(seq)).toBe(false);
    expect(() => btReparent(root, emergency, seq)).toThrow(/own subtree/);
    const halt = emergency.children!.find((c) => c.fn === 'halt')!; btReparent(root, act, halt); expect(emergency.children!.indexOf(act)).toBe(emergency.children!.indexOf(halt) + 1); // dropped on a leaf → placed after it
    btOutdent(root, act); expect(root.children!.includes(act)).toBe(true); expect(() => btOutdent(root, root.children![0])).toThrow(/top level/);
    btSetType(act, 'condition'); expect(act.type).toBe('condition'); expect(act.args).toBeUndefined();
    btSetType(act, 'sequence'); expect(act.children).toEqual([]); expect(act.name).toBe('sequence');
    expect(() => btSetType(emergency, 'action')).toThrow(/children first/);
    expect(() => btRemove(root, root)).toThrow(/root/); btRemove(root, act); expect(root.children!.includes(act)).toBe(false);
    expect(btLabel(newBtNode('retry'))).toBe('retry ×2'); expect(btLabel({ type: 'action', fn: 'goto', args: { zone: 'table' } })).toBe('goto table'); expect(btLabel({ type: 'sequence', name: 'collect one', memory: true, children: [] })).toBe('collect one*');
    const text = btToDsl(doc); const again = parseBt(text); expect(btWalk(again.root).length).toBe(btWalk(root).length); expect(analyse('bt', text).error).toBeUndefined();
  });
});
