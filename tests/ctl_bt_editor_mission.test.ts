// @vitest-environment jsdom
/**
 * A behavior tree built with the editor's node factory (newBtNode / btAddChild) and run as a mission on the station:
 * new actions carry no default timeout (a 5 s default failed every longer program and the mission looped forever with
 * an empty log); a failing tree ends the run with a logged reason instead of restarting silently.
 */
import { describe, it, expect, beforeAll } from 'vitest';
import { App, ItemType, Program } from '../src/app';
import { demos } from '../src/demos';
import { addControlModel } from '../src/ctl/model';
import { newBtNode, btAddChild, btToDsl } from '../src/ctl/graphdoc';
import { parseBt } from '../src/ctl/dsl';
import { startMission } from '../src/ui/control_ui';
import { ProgramSimulator } from '../src/core/motion/simulator';
import { installDomStubs, makeApp, toasts, clearToasts } from './ui_harness';

let app: App;
beforeAll(() => { installDomStubs(); app = makeApp('welding'); });

const duration = (name: string) => { const s = new ProgramSimulator(app.station); s.collisionOptions = { enabled: false }; s.compile(app.station.itemsOfType<Program>(ItemType.PROGRAM).find((p) => p.name === name)!); return s.duration; };

/** sequence(memory)[ parallel(2)[ L, R ], L ] built like the BT editor does (Add child → Binding program). */
function editorTree(progL: string, progR: string, timeout?: number) {
  const doc = parseBt('bt Mission\nsequence root memory');
  const par = btAddChild(doc.root, 'parallel'); par.threshold = 2;
  const add = (parent: typeof doc.root, prog: string) => { const n = btAddChild(parent, 'action'); n.fn = 'program'; n.args = { program: prog }; if (timeout !== undefined) n.timeout = timeout; return n; };
  add(par, progL); add(par, progR); add(doc.root, progL);
  return btToDsl(doc);
}

describe('BT editor tree as a mission', () => {
  it('a new action has no default timeout', () => {
    expect(newBtNode('action').timeout).toBeUndefined();
  });
  it('built with the editor defaults, the mission runs every program once and ends with success', () => {
    app.setStation(demos.find((d) => d.id === 'welding')!.build()); clearToasts();
    const progs = app.station.itemsOfType<Program>(ItemType.PROGRAM).map((p) => p.name).filter((n) => /weld/i.test(n));
    expect(progs.length).toBeGreaterThanOrEqual(2);
    const [L, R] = progs; const longest = Math.max(duration(L), duration(R));
    const src = editorTree(L, R); expect(src).not.toMatch(/timeout=/);
    const m = app.cmd(() => addControlModel(app.station, 'bt', 'Mission', src));
    const run = startMission(app, m, null);
    for (let i = 0; i < 4000 && !run.done; i++) (app as any).tick(0.05);
    expect(run.done).toBe(true); expect(run.rt.status).toBe('success');
    expect(run.rt.time).toBeGreaterThan(longest); expect(run.rt.time).toBeLessThan(longest + duration(L) + 5);
    expect(toasts().some((t) => /Mission: success/.test(t))).toBe(true);
    // the mission log of an arm-only tree: every program start and end with its time, then the tree result
    const log = run.rt.log; const tOf = (re: RegExp) => Number(/^\[([\d.]+)\]/.exec(log.find((l) => re.test(l)) ?? '')?.[1] ?? NaN);
    for (const p of [L, R]) { expect(log.some((l) => l.includes(`program ${p} started`)), log.join('\n')).toBe(true); expect(log.some((l) => l.includes(`program ${p} success`)), log.join('\n')).toBe(true); }
    expect(log.filter((l) => l.includes(`program ${L} started`))).toHaveLength(2);
    expect(tOf(new RegExp(`program ${R} success`))).toBeGreaterThan(tOf(new RegExp(`program ${R} started`)));
    expect(log[log.length - 1]).toMatch(/tree success$/);
  });
  it('a timeout shorter than its program ends the mission with a logged failure (no silent endless restarts)', () => {
    app.setStation(demos.find((d) => d.id === 'welding')!.build()); clearToasts();
    const progs = app.station.itemsOfType<Program>(ItemType.PROGRAM).map((p) => p.name).filter((n) => /weld/i.test(n));
    const [L, R] = progs; const short = Math.min(duration(L), duration(R)) / 2;
    const m = app.cmd(() => addControlModel(app.station, 'bt', 'Mission', editorTree(L, R, short)));
    const run = startMission(app, m, null);
    for (let i = 0; i < 4000 && !run.done; i++) (app as any).tick(0.05);
    expect(run.done).toBe(true); expect(run.rt.status).toBe('failure');
    expect(run.rt.time).toBeLessThan(short + 1);
    expect(run.rt.log.some((l) => /timed out after/.test(l))).toBe(true);
    expect(toasts().some((t) => /Mission: failure/.test(t))).toBe(true);
  });
});
