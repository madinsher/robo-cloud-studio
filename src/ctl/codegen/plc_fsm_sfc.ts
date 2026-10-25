/**
 * An `fsm` model as a PLCopen XML (TC6 v2.01) SFC: the hierarchy is flattened — every leaf state (skill / decision /
 * pass) is a step, a nested machine is entered at its initial leaf, an outcome of a nested machine continues with the
 * transition of its parent state, the root outcomes are final steps `END_<outcome>`. Transition conditions use the same
 * handshake as the ST program of plc_fsm.ts (DONE_x and RES_x = k, FAIL_x), decision expressions (first true wins: the
 * earlier conditions are negated), `<step>.T` for waits and state timeouts; operation steps carry START_x (qualifier N),
 * `set=` / `signal=` steps an inline P1 assignment. Machine-level timeouts and machine entry / exit actions have no SFC
 * counterpart in this flattening (the ST program implements them).
 */
import { parseFsm, FsmMachine, FsmState, stateOutcomes, initialState, fsmMachines } from '../fsm';
import type { ActionBinding } from '../dsl';
import { GenFile, ident, translateExpr, ST_DIALECT, xmlEsc } from './common';
import { fsmToST } from './plc_fsm';

const isOp = (b?: ActionBinding) => !!b && ['program', 'target', 'goto', 'skill'].includes(b.kind);
const stTime = (s: number) => `T#${Math.round(s * 1000)}MS`;
const stVal = (v: unknown) => (typeof v === 'boolean' ? (v ? 'TRUE' : 'FALSE') : typeof v === 'number' ? String(v) : v === 'true' ? 'TRUE' : v === 'false' ? 'FALSE' : Number.isFinite(Number(v)) && String(v).trim() !== '' ? String(Number(v)) : `'${String(v)}'`);

/** Flattened chart: steps (leaf states + END_<outcome>) and transitions in priority order per step. */
export interface FlatSfc { initial: string; steps: Array<{ name: string; action?: string; p1?: string }>; transitions: Array<{ from: string; to: string; outcome: string; cond: string }> }

export function flattenFsm(source: string): FlatSfc {
  const doc = parseFsm(source);
  const st = (e: string) => translateExpr(e, ST_DIALECT((n) => ident(n)));
  type Ctx = { m: FsmMachine; path: string; parent: { ctx: Ctx; state: FsmState } | null };
  const leaves: Array<{ ctx: Ctx; s: FsmState; step: string }> = [];
  const stepName = (ctx: Ctx, s: FsmState) => ident(`${ctx.path.split('/').slice(1).join('_')}_${s.name}`.replace(/^_/, ''));
  const childCtx = (ctx: Ctx, s: FsmState): Ctx => ({ m: s.machine!, path: `${ctx.path}/${s.name}`, parent: { ctx, state: s } });
  const walk = (ctx: Ctx) => { for (const s of ctx.m.states) { if (s.kind === 'machine' && s.machine) walk(childCtx(ctx, s)); else leaves.push({ ctx, s, step: stepName(ctx, s) }); } };
  const root: Ctx = { m: doc.root, path: fsmMachines(doc)[0].path, parent: null };
  walk(root);
  const leafOf = (ctx: Ctx, s: FsmState): string => (s.kind === 'machine' && s.machine ? leafOf(childCtx(ctx, s), initialState(s.machine)!) : stepName(ctx, s));
  const target = (ctx: Ctx, s: FsmState, o: string): string | null => {
    let to = s.transitions[o];
    if (to === undefined && o === 'timeout') to = s.transitions[s.action?.fail ?? 'fail'];
    if (to === undefined && ctx.m.outcomes.includes(o)) to = o;
    if (to === undefined) return null;
    const sib = ctx.m.states.find((x) => x.name === to);
    if (sib) return leafOf(ctx, sib);
    if (!ctx.m.outcomes.includes(to)) return null;
    return ctx.parent ? target(ctx.parent.ctx, ctx.parent.state, to) : `END_${ident(to)}`;
  };
  const steps: FlatSfc['steps'] = []; const transitions: FlatSfc['transitions'] = [];
  for (const l of leaves) {
    const b = l.s.action; const step: FlatSfc['steps'][number] = { name: l.step };
    if (b && isOp(b)) step.action = `START_${l.step}`;
    if (b && b.kind === 'set') step.p1 = `${ident(b.value)} := ${typeof b.args.value === 'string' && b.args.value.startsWith('=') ? st(b.args.value.slice(1)) : stVal(b.args.value ?? true)};`;
    if (b && b.kind === 'signal') step.p1 = `${ident(b.value)} := ${stVal(b.args.value ?? true)};`;
    steps.push(step);
  }
  for (const o of doc.root.outcomes) steps.push({ name: `END_${ident(o)}` });
  for (const l of leaves) {
    const s = l.s; const b = s.action; const outs = stateOutcomes(s);
    const named = typeof b?.args.outcomes === 'string' ? String(b.args.outcomes).split(',').map((x) => x.trim()).filter(Boolean) : null;
    const okO = named ? named[0] : (b?.done ?? 'ok'), failO = named ? named[named.length - 1] : (b?.fail ?? 'fail');
    const conds: Array<[string, string]> = [];
    if (s.kind === 'pass') conds.push(['ok', 'TRUE']);
    else if (s.kind === 'decision') {
      const prev: string[] = [];
      for (const w of s.when) { conds.push([w.outcome, [...prev.map((p) => `NOT (${p})`), `(${st(w.expr)})`].join(' AND ')]); prev.push(st(w.expr)); }
      if (s.otherwise) conds.push([s.otherwise, prev.length ? prev.map((p) => `NOT (${p})`).join(' AND ') : 'TRUE']);
    } else if (b && isOp(b)) {
      const id = l.step;
      if (named) named.forEach((o, k) => conds.push([o, `DONE_${id} AND (RES_${id} = ${k + 1}${k === 0 ? ` OR RES_${id} = 0` : ''})`]));
      else conds.push([okO, `DONE_${id}`]);
      conds.push([failO, `FAIL_${id}`]);
    } else if (b?.kind === 'wait') conds.push([okO, `${l.step}.T >= ${stTime(Number(b.value))}`]);
    else if (b?.kind === 'await') conds.push([okO, st(b.value)]);
    else conds.push([okO, 'TRUE']);
    const tmo = s.timeout ?? b?.timeout;
    if (tmo !== undefined) conds.unshift([outs.includes('timeout') ? 'timeout' : failO, `${l.step}.T >= ${stTime(tmo)}`]);
    for (const [o, c] of conds) { const to = target(l.ctx, s, o); if (to) transitions.push({ from: l.step, to, outcome: o, cond: c }); }
  }
  return { initial: leafOf(root, initialState(doc.root)!), steps, transitions };
}

export function fsmToPlcOpen(source: string, name: string): GenFile {
  const doc = parseFsm(source); const prog = ident(name);
  const flat = flattenFsm(source);
  interface S { id: number; name: string; initial: boolean; action?: string; p1?: string; ins: number[]; outs: number[] }
  interface T { id: number; label: string; cond: string; from: number; to: number }
  let lid = 0; const nid = () => ++lid;
  const steps: S[] = flat.steps.map((s) => ({ id: nid(), name: s.name, initial: s.name === flat.initial, action: s.action, p1: s.p1, ins: [], outs: [] }));
  const byName = new Map(steps.map((s) => [s.name, s]));
  const trs: T[] = flat.transitions.map((t) => { const tr: T = { id: nid(), label: `${t.from}_${t.outcome}`, cond: t.cond, from: byName.get(t.from)!.id, to: byName.get(t.to)!.id }; byName.get(t.from)!.outs.push(tr.id); byName.get(t.to)!.ins.push(tr.id); return tr; });
  const selDiv = new Map<number, number>(), selConv = new Map<number, number>();
  for (const s of steps) { if (s.outs.length > 1) selDiv.set(s.id, nid()); if (s.ins.length > 1) selConv.set(s.id, nid()); }
  const ref = (id: number, fp?: string) => `<connection refLocalId="${id}"${fp ? ` formalParameter="${fp}"` : ''}/>`;
  let row = 0; const pos = () => `<position x="${40 + (row % 6) * 180}" y="${40 + 60 * Math.floor(row++ / 6)}"/>`;
  const out: string[] = [];
  for (const s of steps) {
    const inXml = selConv.has(s.id) ? `<connectionPointIn>${ref(selConv.get(s.id)!)}</connectionPointIn>` : s.ins.length ? `<connectionPointIn>${ref(s.ins[0])}</connectionPointIn>` : '';
    out.push(`<step localId="${s.id}" name="${xmlEsc(s.name)}" initialStep="${s.initial}">${pos()}${inXml}<connectionPointOut formalParameter="sfc"/></step>`);
    if (selConv.has(s.id)) out.push(`<selectionConvergence localId="${selConv.get(s.id)}">${pos()}${s.ins.map((t) => `<connectionPointIn>${ref(t)}</connectionPointIn>`).join('')}<connectionPointOut/></selectionConvergence>`);
    if (selDiv.has(s.id)) out.push(`<selectionDivergence localId="${selDiv.get(s.id)}">${pos()}<connectionPointIn>${ref(s.id)}</connectionPointIn>${s.outs.map((_, k) => `<connectionPointOut formalParameter="o${k}"/>`).join('')}</selectionDivergence>`);
    const acts = [...(s.action ? [`<action localId="${nid()}" qualifier="N"><reference name="${s.action}"/></action>`] : []), ...(s.p1 ? [`<action localId="${nid()}" qualifier="P1"><inline><ST><xhtml:p>${xmlEsc(s.p1)}</xhtml:p></ST></inline></action>`] : [])];
    if (acts.length) out.push(`<actionBlock localId="${nid()}">${pos()}<connectionPointIn>${ref(s.id)}</connectionPointIn>${acts.join('')}</actionBlock>`);
  }
  for (const t of trs) {
    const from = steps.find((x) => x.id === t.from)!;
    const inXml = selDiv.has(from.id) ? ref(selDiv.get(from.id)!, `o${from.outs.indexOf(t.id)}`) : ref(from.id);
    out.push(`<transition localId="${t.id}">${pos()}<connectionPointIn>${inXml}</connectionPointIn><condition><inline name="${xmlEsc(t.label)}"><ST><xhtml:p>${xmlEsc(t.cond)}</xhtml:p></ST></inline></condition><connectionPointOut/></transition>`);
  }
  // interface: the handshake signals and inputs of the ST program (same names and addresses)
  const [stFile] = fsmToST(source, name);
  const decl = (sec: string) => { const m = new RegExp(`${sec}\\n([\\s\\S]*?)END_VAR`).exec(stFile.content); return (m?.[1] ?? '').split('\n').map((l) => /^\s*(\w+) AT (%\S+) : (\w+);/.exec(l)).filter((x): x is RegExpExecArray => !!x).map((x) => ({ n: x[1], a: x[2], t: x[3] })); };
  const vars = (list: Array<{ n: string; t: string; a?: string }>) => list.map((v) => `<variable name="${xmlEsc(v.n)}"${v.a ? ` address="${v.a}"` : ''}><type><${v.t}/></type></variable>`).join('');
  const locals = Object.entries(doc.blackboard ?? {}).map(([k, v]) => ({ n: ident(k), t: typeof v === 'boolean' ? 'BOOL' : 'LREAL' }));
  const now = new Date().toISOString().replace(/\.\d+Z$/, '');
  const xml = `<?xml version="1.0" encoding="UTF-8"?>
<project xmlns="http://www.plcopen.org/xml/tc6_0201" xmlns:xhtml="http://www.w3.org/1999/xhtml">
  <fileHeader companyName="VerticalBot Studio" productName="VerticalBot Studio" productVersion="1" creationDateTime="${now}"/>
  <contentHeader name="${xmlEsc(name)}" modificationDateTime="${now}"><coordinateInfo><fbd><scaling x="1" y="1"/></fbd><ld><scaling x="1" y="1"/></ld><sfc><scaling x="1" y="1"/></sfc></coordinateInfo></contentHeader>
  <types>
    <dataTypes/>
    <pous>
      <pou name="${prog}" pouType="program">
        <interface><inputVars>${vars(decl('VAR_INPUT'))}</inputVars><outputVars>${vars(decl('VAR_OUTPUT').filter((v) => !/^FSM_/.test(v.n)))}</outputVars><localVars>${vars(locals)}</localVars></interface>
        <body><SFC>
${out.map((l) => `          ${l}`).join('\n')}
        </SFC></body>
      </pou>
    </pous>
  </types>
  <instances><configurations><configuration name="Config0"><resource name="Res0"><task name="MainTask" priority="1" interval="T#20ms"><pouInstance name="${prog}_1" typeName="${prog}"/></task></resource></configuration></configurations></instances>
</project>
`;
  return { name: `${prog}.plcopen.xml`, content: xml, mime: 'application/xml' };
}
