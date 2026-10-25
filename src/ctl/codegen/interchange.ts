/**
 * Interchange formats for the control models:
 *  - PNML (ISO/IEC 15909-2, P/T net grammar) export and import, with the studio's extras (delays, rates, priorities,
 *    capacities, guards, bound actions, inhibitor arcs) as `toolspecific` elements so a round trip is lossless;
 *  - SCXML (W3C) from an FSM (nested states, outcome finals, eventless decision transitions, invoked skills, timeouts)
 *    and from an automaton document (one parallel region per component);
 *  - NuSMV / nuXmv `.smv` from automata (components + event input variable, the supervisor as a variable when there are
 *    specifications), Petri nets (bounded markings) and the synchronous `smv` documents;
 *  - UPPAAL XML (timed automata) from automata: one template per component, a broadcast channel per event issued by a
 *    scheduler when the event is enabled everywhere, `wait` bindings as clock invariants / guards.
 */
import { parseDes, parsePetri, parseSmv, ActionBinding } from '../dsl';
import { DES } from '../des';
import { PetriNet, PetriNetSpec } from '../petri';
import { parseFsm, FsmMachine, FsmState, initialState, stateOutcomes } from '../fsm';
import { petriToDsl } from '../graphdoc';
import { synthesizeSupervisor } from '../supervisor';
import { bindingText } from '../graphdoc';
import { GenFile, ident, xmlEsc, translateExpr, ECMA_DIALECT, SMV_DIALECT, parseXml, xmlFind, xmlAll, XmlNode } from './common';

const TOOL = 'VerticalBot Studio';

// ---------------------------------------------------------------------------------------------
// PNML
// ---------------------------------------------------------------------------------------------

export function petriToPnml(source: string, name: string): GenFile {
  const doc = parsePetri(source); const s = doc.spec;
  const ts = (inner: string) => (inner ? `<toolspecific tool="${TOOL}" version="1.0">${inner}</toolspecific>` : '');
  const el = (tag: string, v: unknown) => (v === undefined || v === null || v === false ? '' : `<${tag}>${xmlEsc(String(v))}</${tag}>`);
  const pos = (id: string) => (doc.layout[id] ? `<graphics><position x="${doc.layout[id][0]}" y="${doc.layout[id][1]}"/></graphics>` : '');
  const L: string[] = ['<?xml version="1.0" encoding="UTF-8"?>', '<pnml xmlns="http://www.pnml.org/version-2009/grammar/pnml">', `  <net id="${ident(name)}" type="http://www.pnml.org/version-2009/grammar/ptnet">`, `    <name><text>${xmlEsc(s.name)}</text></name>`, '    <page id="page0">'];
  for (const p of s.places) L.push(`      <place id="${xmlEsc(p.id)}"><name><text>${xmlEsc(p.label ?? p.id)}</text></name>${p.tokens ? `<initialMarking><text>${p.tokens}</text></initialMarking>` : ''}${pos(p.id)}${ts(el('kind', p.kind) + el('capacity', p.capacity) + el('label', p.label))}</place>`);
  for (const t of s.transitions) { const b = doc.actions.find((a) => a.target === t.id); L.push(`      <transition id="${xmlEsc(t.id)}"><name><text>${xmlEsc(t.label ?? t.id)}</text></name>${pos(t.id)}${ts(el('delay', t.delay) + el('rate', t.rate) + (t.immediate ? '<immediate/>' : '') + el('weight', t.weight) + el('priority', t.priority) + el('servers', t.servers) + el('guard', t.guard) + el('on', t.on) + el('label', t.label) + (b ? `<action>${xmlEsc(bindingText(b))}</action>` : ''))}</transition>`); }
  s.arcs.forEach((a, k) => L.push(`      <arc id="a${k + 1}" source="${xmlEsc(a.from)}" target="${xmlEsc(a.to)}">${a.weight && a.weight !== 1 ? `<inscription><text>${a.weight}</text></inscription>` : ''}${a.inhibitor ? ts('<inhibitor/>') : ''}</arc>`));
  L.push('    </page>');
  if (doc.finals?.length) L.push(`    ${ts(doc.finals.map((f) => `<final>${xmlEsc(Object.entries(f).map(([p, n]) => (n === 1 ? p : `${p}=${n}`)).join(' '))}</final>`).join(''))}`);
  L.push('  </net>', '</pnml>', '');
  return { name: `${ident(name)}.pnml`, content: L.join('\n'), mime: 'application/xml' };
}

/** PNML (P/T net) → Petri DSL source. Ids that are not DSL identifiers are sanitised; tool-specific extras of the studio are restored. */
export function pnmlToPetriDsl(xml: string): string {
  const root = parseXml(xml); const net = xmlFind(root, 'net'); if (!net) throw new Error('PNML: no <net>');
  const text = (n: XmlNode | undefined) => (n ? (xmlFind(n, 'text')?.text ?? n.text).trim() : '');
  const tool = (n: XmlNode) => n.children.find((c) => c.name.endsWith('toolspecific') && c.attrs.tool === TOOL);
  const tv = (n: XmlNode, tag: string) => { const t = tool(n); const x = t ? xmlFind(t, tag) : undefined; return x ? x.text.trim() : undefined; };
  const ids = new Map<string, string>(); const used = new Set<string>();
  const sid = (raw: string) => { let id = /^[A-Za-z_][\w.-]*$/.test(raw) ? raw : ident(raw, 'n'); while (used.has(id)) id += '_'; used.add(id); ids.set(raw, id); return id; };
  const spec: PetriNetSpec = { name: text(xmlFind(net, 'name')) || net.attrs.id || 'Imported net', places: [], transitions: [], arcs: [] };
  const layout: Record<string, [number, number]> = {}; const actions: ActionBinding[] = [];
  const posOf = (n: XmlNode, id: string) => { const g = xmlFind(n, 'graphics'); const p = g ? xmlFind(g, 'position') : undefined; if (p) layout[id] = [Number(p.attrs.x), Number(p.attrs.y)]; };
  const num = (v: string | undefined) => (v === undefined ? undefined : Number(v));
  for (const p of xmlAll(net, 'place')) { const id = sid(p.attrs.id); const lbl = tv(p, 'label'); spec.places.push({ id, tokens: Number(text(xmlFind(p, 'initialMarking')) || 0), kind: tv(p, 'kind') as PetriNetSpec['places'][number]['kind'], capacity: num(tv(p, 'capacity')), label: lbl ?? (text(xmlFind(p, 'name')) !== id && text(xmlFind(p, 'name')) ? text(xmlFind(p, 'name')) : undefined) }); posOf(p, id); }
  for (const t of xmlAll(net, 'transition')) {
    const id = sid(t.attrs.id); const tl = tool(t);
    spec.transitions.push({ id, delay: num(tv(t, 'delay')), rate: num(tv(t, 'rate')), immediate: tl ? !!xmlFind(tl, 'immediate') || undefined : undefined, weight: num(tv(t, 'weight')), priority: num(tv(t, 'priority')), servers: num(tv(t, 'servers')), guard: tv(t, 'guard'), on: tv(t, 'on'), label: tv(t, 'label') ?? (text(xmlFind(t, 'name')) !== id && text(xmlFind(t, 'name')) ? text(xmlFind(t, 'name')) : undefined) });
    posOf(t, id); const act = tv(t, 'action'); if (act) actions.push(parsePetri(`petri x\ntransition ${id}\naction ${id} ${act}`).actions[0]);
  }
  for (const a of xmlAll(net, 'arc')) { const from = ids.get(a.attrs.source), to = ids.get(a.attrs.target); if (!from || !to) throw new Error(`PNML arc ${a.attrs.id}: unknown node ${a.attrs.source} / ${a.attrs.target}`); const w = Number(text(xmlFind(a, 'inscription')) || 1); spec.arcs.push({ from, to, weight: w !== 1 ? w : undefined, inhibitor: tool(a) && xmlFind(tool(a)!, 'inhibitor') ? true : undefined }); }
  for (const p of spec.places) for (const k of Object.keys(p) as Array<keyof typeof p>) if (p[k] === undefined) delete p[k];
  for (const t of spec.transitions) for (const k of Object.keys(t) as Array<keyof typeof t>) if (t[k] === undefined) delete t[k];
  const nt = tool(net); const finals = nt ? nt.children.filter((c) => c.name === 'final').map((f) => { const m: Record<string, number> = {}; for (const w of f.text.trim().split(/\s+/)) { const [p, n] = w.split('='); m[ids.get(p) ?? p] = n ? Number(n) : 1; } return m; }) : [];
  return petriToDsl({ spec, checks: [], horizon: 300, layout, actions, ...(finals.length ? { finals } : {}) });
}

// ---------------------------------------------------------------------------------------------
// SCXML
// ---------------------------------------------------------------------------------------------

const ecma = (e: string) => xmlEsc(translateExpr(e, ECMA_DIALECT));
function scxmlAction(b: ActionBinding | Omit<ActionBinding, 'target'>): string {
  if (b.kind === 'set') { const v = b.args.value; return `<assign location="${ident(b.value)}" expr="${typeof v === 'string' && v.startsWith('=') ? ecma(v.slice(1)) : xmlEsc(typeof v === 'string' ? `'${v}'` : String(v ?? true))}"/>`; }
  if (b.kind === 'event') return `<raise event="${xmlEsc(b.value)}"/>`;
  return `<send event="vbs.${b.kind}" target="#_vbs_station"><param name="value" expr="'${xmlEsc(b.value.replace(/'/g, ''))}'"/>${b.robot ? `<param name="robot" expr="'${xmlEsc(b.robot)}'"/>` : ''}${b.kind === 'signal' ? `<param name="state" expr="${xmlEsc(String(b.args.value ?? true))}"/>` : ''}</send>`;
}
export function fsmToScxml(source: string, name: string): GenFile {
  const doc = parseFsm(source);
  const sid = (path: string[]) => path.map((p) => ident(p)).join('.');
  const machine = (m: FsmMachine, path: string[], ind: string): string[] => {
    const out: string[] = [];
    for (const s of m.states) out.push(...state(m, s, [...path, s.name], ind));
    for (const o of m.outcomes) out.push(`${ind}<final id="${sid([...path, '_' + o])}"><onentry><raise event="${xmlEsc(path.length ? `${ident(path[path.length - 1])}.${o}` : o)}"/></onentry></final>`);
    return out;
  };
  const target = (m: FsmMachine, path: string[], to: string) => (m.states.some((x) => x.name === to) ? sid([...path, to]) : sid([...path, '_' + to]));
  const state = (m: FsmMachine, s: FsmState, path: string[], ind: string): string[] => {
    const parent = path.slice(0, -1); const id = sid(path);
    const out: string[] = [`${ind}<state id="${id}"${s.machine ? ` initial="${sid([...path, initialState(s.machine)!.name])}"` : ''}>`];
    const onentry = [...s.entry.map(scxmlAction), ...(s.timeout !== undefined ? [`<send event="timeout" delay="${s.timeout}s" id="${id}_timer"/>`] : [])];
    if (onentry.length) out.push(`${ind}  <onentry>${onentry.join('')}</onentry>`);
    const onexit = [...s.exit.map(scxmlAction), ...(s.timeout !== undefined ? [`<cancel sendid="${id}_timer"/>`] : [])];
    if (onexit.length) out.push(`${ind}  <onexit>${onexit.join('')}</onexit>`);
    if (s.action) out.push(`${ind}  <invoke id="${id}_skill" type="http://verticalbot.studio/skill/${s.action.kind}"><param name="value" expr="'${xmlEsc(s.action.value.replace(/'/g, ''))}'"/>${s.action.robot ? `<param name="robot" expr="'${xmlEsc(s.action.robot)}'"/>` : ''}<finalize/></invoke>`);
    if (s.machine) out.push(...machine(s.machine, path, ind + '  '));
    const outcomes = stateOutcomes(s);
    if (s.kind === 'decision') { for (const w of s.when) out.push(`${ind}  <transition cond="${ecma(w.expr)}" target="${target(m, parent, s.transitions[w.outcome] ?? w.outcome)}"/>`); if (s.otherwise) out.push(`${ind}  <transition target="${target(m, parent, s.transitions[s.otherwise] ?? s.otherwise)}"/>`); }
    else if (s.kind === 'pass') out.push(`${ind}  <transition target="${target(m, parent, s.transitions.ok ?? 'ok')}"/>`);
    else for (const o of [...new Set([...outcomes, ...Object.keys(s.transitions)])]) { const to = s.transitions[o] ?? (o === 'timeout' ? s.transitions[s.action?.fail ?? 'fail'] : undefined) ?? (m.outcomes.includes(o) ? o : undefined); if (to !== undefined) out.push(`${ind}  <transition event="${xmlEsc(s.machine ? `${ident(s.name)}.${o}` : o)}" target="${target(m, parent, to)}"/>`); }
    out.push(`${ind}</state>`);
    return out;
  };
  const init = initialState(doc.root)!;
  const L = ['<?xml version="1.0" encoding="UTF-8"?>', `<!-- FSM "${xmlEsc(doc.name)}" generated by ${TOOL}: skills are <invoke>d services of the station (done.invoke → outcome events ok / fail), decisions are eventless transitions, machine outcomes are finals raising <machine>.<outcome>. -->`,
    `<scxml xmlns="http://www.w3.org/2005/07/scxml" version="1.0" datamodel="ecmascript" name="${xmlEsc(ident(doc.name))}" initial="${sid([init.name])}">`];
  if (Object.keys(doc.blackboard).length) L.push(`  <datamodel>${Object.entries(doc.blackboard).map(([k, v]) => `<data id="${ident(k)}" expr="${xmlEsc(typeof v === 'string' ? `'${v}'` : String(v))}"/>`).join('')}</datamodel>`);
  L.push(...machine(doc.root, [], '  '), '</scxml>', '');
  return { name: `${ident(name)}.scxml`, content: L.join('\n'), mime: 'application/xml' };
}
export function desToScxml(source: string, name: string): GenFile {
  const doc = parseDes(source); const uc = new Set(doc.uncontrollable);
  const L = ['<?xml version="1.0" encoding="UTF-8"?>', `<!-- automata "${xmlEsc(doc.name)}" generated by ${TOOL}: one parallel region per component; events synchronise the regions (controllable: ${xmlEsc(doc.plant.flatMap((a) => a.events).filter((e, i, a) => !uc.has(e) && a.indexOf(e) === i).join(' '))}). -->`,
    `<scxml xmlns="http://www.w3.org/2005/07/scxml" version="1.0" datamodel="ecmascript" name="${xmlEsc(ident(doc.name))}" initial="plant">`, '  <parallel id="plant">'];
  for (const a of doc.plant) {
    const d = DES.fromSpec(a); const id = (x: string) => `${ident(a.name)}.${ident(x)}`;
    L.push(`    <state id="${ident(a.name)}" initial="${id(d.x0)}">`);
    for (const x of d.X) { const b = doc.actions.find((ac) => ac.target === `${a.name}.${x}`); L.push(`      <state id="${id(x)}">${b ? `<onentry>${scxmlAction(b)}</onentry>` : ''}${d.active(x).map((e) => `<transition event="${xmlEsc(e)}" target="${id(d.next(x, e)!)}"${doc.guards?.[e] ? ` cond="${ecma(doc.guards[e])}"` : ''}/>`).join('')}</state>`); }
    L.push('    </state>');
  }
  L.push('  </parallel>', '</scxml>', '');
  return { name: `${ident(name)}.scxml`, content: L.join('\n'), mime: 'application/xml' };
}

// ---------------------------------------------------------------------------------------------
// NuSMV
// ---------------------------------------------------------------------------------------------

const val = (s: string) => `s_${ident(s)}`;
export function desToSmv(source: string, name: string): GenFile {
  const doc = parseDes(source); const comps = doc.plant.map((a) => DES.fromSpec(a));
  const events = [...new Set(comps.flatMap((c) => [...c.sigma]))];
  const sup = doc.specs.length ? synthesizeSupervisor(doc) : null;
  const cv = (c: DES) => ident(c.name);
  const L = [`-- automata "${doc.name}" generated by ${'VerticalBot Studio'}: synchronous product over the input event ev${sup ? '; supervisor table as the variable sup (controllable events only where it enables them)' : ''}`, 'MODULE main', 'VAR'];
  for (const c of comps) L.push(`  ${cv(c)} : {${[...c.X].map(val).join(', ')}};`);
  if (sup) L.push(`  sup : 0..${sup.table.states.length - 1};`);
  L.push('IVAR', `  ev : {${events.map((e) => `e_${ident(e)}`).join(', ')}};`, 'ASSIGN');
  for (const c of comps) { L.push(`  init(${cv(c)}) := ${val(c.x0)};`, `  next(${cv(c)}) := case`); for (const x of c.X) for (const e of c.active(x)) L.push(`    ${cv(c)} = ${val(x)} & ev = e_${ident(e)} : ${val(c.next(x, e)!)};`); L.push(`    TRUE : ${cv(c)};`, '  esac;'); }
  if (sup) { L.push(`  init(sup) := ${sup.table.initial};`, '  next(sup) := case'); for (const s of sup.table.states) for (const [e, n] of Object.entries(s.next)) if (events.includes(e)) L.push(`    sup = ${s.id} & ev = e_${ident(e)} : ${n};`); L.push('    TRUE : sup;', '  esac;'); }
  L.push('TRANS');
  const tr: string[] = [];
  for (const e of events) {
    const parts = comps.filter((c) => c.sigma.has(e)).map((c) => `(${[...c.X].filter((x) => c.next(x, e) !== undefined).map((x) => `${cv(c)} = ${val(x)}`).join(' | ') || 'FALSE'})`);
    if (sup && !doc.uncontrollable.includes(e)) { const ss = sup.table.states.filter((s) => s.enabled_controllable.includes(e)).map((s) => s.id); parts.push(ss.length ? `(${ss.map((v) => `sup = ${v}`).join(' | ')})` : 'FALSE'); }
    tr.push(`  (ev = e_${ident(e)} -> ${parts.join(' & ') || 'TRUE'})`);
  }
  L.push(tr.join(' &\n') + ';');
  const fm = (f: string) => translateExpr(f.replace(/"([^"]+)"/g, '$1'), SMV_DIALECT((n) => ident(n))).replace(/\b([A-Za-z_]\w*)\s*=\s*s_/g, '$1 = s_');
  for (const c of doc.checks) { try { L.push(`${c.kind === 'ltl' ? 'LTLSPEC' : 'CTLSPEC'} ${c.name ? `NAME ${ident(c.name)} := ` : ''}${smvTemporal(c.formula, fm)};`); } catch (e) { L.push(`-- not translated: ${c.formula} (${(e as Error).message})`); } }
  return { name: `${ident(name)}.smv`, content: L.join('\n') + '\n', mime: 'text/plain' };
}
/** Temporal formula in NuSMV syntax: operators kept, quoted state values → s_x, identifiers mapped (dots → _). */
function smvTemporal(f: string, idMap: (id: string) => string = (n) => ident(n)): string {
  const KW = /^(G|F|X|U|R|V|W|Y|Z|H|O|S|T|A|E|AG|AF|AX|EG|EF|EX|AU|EU|TRUE|FALSE|mod|deadlock)$/;
  return f.split(/('[^']*')/).map((p) => (p.startsWith("'") ? val(p.slice(1, -1)) : p.replace(/[A-Za-z_][\w.]*/g, (m) => (KW.test(m) ? m : m === 'true' ? 'TRUE' : m === 'false' ? 'FALSE' : idMap(m))))).join('').replace(/==/g, '=').replace(/&&/g, '&').replace(/\|\|/g, '|');
}
export function petriToSmv(source: string, name: string, bound = 8): GenFile {
  const doc = parsePetri(source); const net = new PetriNet(doc.spec);
  let maxTokens = net.places.map((p) => p.capacity ?? bound);
  try { const rg = net.reachability({ limit: 20000 }); if (!rg.truncated) maxTokens = net.places.map((_, i) => Math.max(1, ...rg.markings.map((m) => m[i]))); } catch { /* keep the bound */ }
  const mv = (i: number) => `m_${ident(net.places[i].id)}`; const tv = (j: number) => `t_${ident(net.transitions[j].id)}`;
  const en = (j: number) => net.places.map((p, i) => [net.pre[j][i] ? `${mv(i)} >= ${net.pre[j][i]}` : '', net.inhibitor[j][i] ? `${mv(i)} < ${net.inhibitor[j][i]}` : '', p.capacity !== undefined && net.post[j][i] ? `${mv(i)} - ${net.pre[j][i]} + ${net.post[j][i]} <= ${p.capacity}` : ''].filter(Boolean).join(' & ')).filter(Boolean).join(' & ') || 'TRUE';
  // a guard over station signals (guard="Ready == 1") or an input event (on=...) is not part of the net: it is an
  // environment input that may hold or not at every step (IVAR g_t), so the model over-approximates the controller
  const gated = net.transitions.map((t, j) => ({ j, t })).filter(({ t }) => t.guard || t.on);
  const L = [`-- Petri net "${doc.spec.name}" generated by VerticalBot Studio (interleaving semantics, markings bounded by the reachability graph)`,
    ...gated.map(({ t }) => `-- ${t.id}: ${t.guard ? `guard "${t.guard}"` : ''}${t.guard && t.on ? ', ' : ''}${t.on ? `input event ${t.on}` : ''} -> environment input g_${tv(net.transitions.indexOf(t))}`), 'MODULE main', 'VAR'];
  net.places.forEach((_, i) => L.push(`  ${mv(i)} : 0..${maxTokens[i]};`));
  L.push('IVAR', `  t : {none, ${net.transitions.map((_, j) => tv(j)).join(', ')}};`, ...gated.map(({ j }) => `  g_${tv(j)} : boolean;`), 'DEFINE');
  net.transitions.forEach((t, j) => L.push(`  en_${tv(j)} := ${en(j)}${t.guard || t.on ? ` & g_${tv(j)}` : ''};`));
  // deadlock: nothing can fire whatever the environment inputs (a transition waiting for its guard is not dead)
  const enStruct = (j: number) => en(j);
  L.push(`  deadlock := !(${net.transitions.map((_, j) => `(${enStruct(j)})`).join(' | ') || 'FALSE'});`, 'ASSIGN');
  net.places.forEach((p, i) => { L.push(`  init(${mv(i)}) := ${p.tokens ?? 0};`, `  next(${mv(i)}) := case`); net.transitions.forEach((_, j) => { const d = net.post[j][i] - net.pre[j][i]; if (d) L.push(`    t = ${tv(j)} : ${mv(i)} ${d > 0 ? '+' : '-'} ${Math.abs(d)};`); }); L.push(`    TRUE : ${mv(i)};`, '  esac;'); });
  L.push('TRANS', `  ${[...net.transitions.map((_, j) => `(t = ${tv(j)} -> en_${tv(j)})`), `(t = none -> (deadlock${gated.length ? ` | !(${net.transitions.map((_, j) => `en_${tv(j)}`).join(' | ')})` : ''}))`].join(' & ')};`);
  const fm = (s: string) => { const p = net.places.find((x) => x.id === s); return p ? mv(net.places.indexOf(p)) : ident(s); };
  for (const c of doc.checks) L.push(`${c.kind === 'ltl' ? 'LTLSPEC' : 'CTLSPEC'} ${smvTemporal(c.formula, fm)};`);
  return { name: `${ident(name)}.smv`, content: L.join('\n') + '\n', mime: 'text/plain' };
}
export function smvDocToNuSmv(source: string, name: string): GenFile {
  const spec = parseSmv(source); const id = (n: string) => ident(n);
  const ex = (e: string) => e.replace(/[A-Za-z_][\w]*(\.[A-Za-z_]\w*)+/g, (m) => id(m));
  const L = [`-- "${spec.name}" generated by VerticalBot Studio`, 'MODULE main', 'VAR'];
  for (const v of spec.vars) L.push(`  ${id(v.name)} : ${v.domain.every((d) => typeof d === 'boolean') ? 'boolean' : v.domain.every((d) => typeof d === 'number') ? `${Math.min(...(v.domain as number[]))}..${Math.max(...(v.domain as number[]))}` : `{${v.domain.join(', ')}}`};`);
  if (Object.keys(spec.defines ?? {}).length) { L.push('DEFINE'); for (const [k, v] of Object.entries(spec.defines!)) L.push(`  ${id(k)} := ${ex(v)};`); }
  L.push('ASSIGN');
  for (const [k, v] of Object.entries(spec.init ?? {})) L.push(`  init(${id(k)}) := ${ex(v)};`);
  for (const [k, cases] of Object.entries(spec.next ?? {})) { L.push(`  next(${id(k)}) := case`); for (const c of cases) L.push(`    ${ex(c.cond)} : ${ex(c.value)};`); if (!cases.some((c) => /^TRUE$/i.test(c.cond.trim()))) L.push(`    TRUE : ${id(k)};`); L.push('  esac;'); }
  for (const f of spec.fairness ?? []) L.push(`FAIRNESS ${ex(f)};`);
  for (const s of spec.specs ?? []) L.push(`${s.kind === 'LTL' ? 'LTLSPEC' : 'CTLSPEC'} ${s.name ? `NAME ${ident(s.name)} := ` : ''}${ex(s.formula)};`);
  return { name: `${ident(name)}.smv`, content: L.join('\n') + '\n', mime: 'text/plain' };
}

// ---------------------------------------------------------------------------------------------
// UPPAAL
// ---------------------------------------------------------------------------------------------

export function desToUppaal(source: string, name: string): GenFile {
  const doc = parseDes(source); const comps = doc.plant.map((a) => DES.fromSpec(a));
  // the synthesised supervisor (as in the studio run, the SMV and the SCXML exports): `sup` is its state, a
  // controllable event is enabled only where the table enables it, every event moves it
  const sup = doc.specs.length ? synthesizeSupervisor(doc) : null;
  const supT = sup?.table ?? null;
  const events = [...new Set(comps.flatMap((c) => [...c.sigma]))]; const uc = new Set(doc.uncontrollable);
  const sv = (c: DES) => `s_${ident(c.name)}`; const code = (c: DES) => new Map([...c.X].map((x, i) => [x, i]));
  const waitOf = (c: DES, x: string) => { const b = doc.actions.find((a) => a.target === `${c.name}.${x}`); return b?.kind === 'wait' ? { secs: Number(b.value), done: b.done } : null; };
  const decl: string[] = [`// automata "${doc.name}" generated by VerticalBot Studio`, ...comps.map((c) => `int ${sv(c)} = ${code(c).get(c.x0)}; // ${[...code(c)].map(([x, i]) => `${i}=${x}`).join(' ')}`), ...comps.map((c) => `clock x_${ident(c.name)};`), ...events.map((e) => `broadcast chan e_${ident(e)};`)];
  if (supT) {
    decl.push(`int sup = ${supT.initial}; // supervisor state (${supT.states.length} states${supT.observer ? ', observer-based' : ''})`);
    for (const e of events) {
      const moves = supT.states.filter((s) => s.next[e] !== undefined && s.next[e] !== s.id).map((s) => `if (s == ${s.id}) return ${s.next[e]};`);
      decl.push(`int sup_${ident(e)}(int s) { ${moves.join(' ')} return s; }`);
    }
  }
  for (const e of events) {
    const conds = comps.filter((c) => c.sigma.has(e)).map((c) => { const xs = [...c.X].filter((x) => c.next(x, e) !== undefined); return `(${xs.map((x) => { const w = waitOf(c, x); return `(${sv(c)} == ${code(c).get(x)}${w && w.done === e ? ` && x_${ident(c.name)} >= ${w.secs}` : ''})`; }).join(' || ') || 'false'})`; });
    if (supT && !(uc.has(e) && (doc.unobservable ?? []).includes(e))) { const ok = supT.states.filter((s) => (uc.has(e) ? s.next[e] !== undefined : s.enabled_controllable.includes(e))).map((s) => `sup == ${s.id}`); conds.push(`(${ok.join(' || ') || 'false'})`); }
    decl.push(`bool en_${ident(e)}() { return ${conds.join(' && ') || 'true'}; }${uc.has(e) ? ' // uncontrollable' : ''}`);
  }
  if (Object.keys(doc.guards ?? {}).length) decl.push(`// guards over station signals are not modelled (the events are enabled whatever the signals): ${Object.entries(doc.guards ?? {}).map(([e, g]) => `${e}: ${g}`).join('; ')}`);
  let lid = 0; const templates: string[] = [];
  for (const c of comps) {
    const ids = new Map([...c.X].map((x) => [x, `id${lid++}`]));
    const locs = [...c.X].map((x, k) => { const w = waitOf(c, x); return `<location id="${ids.get(x)}" x="${k * 150}" y="0"><name>${xmlEsc(ident(x))}</name>${w ? `<label kind="invariant">x_${ident(c.name)} &lt;= ${w.secs}</label>` : ''}</location>`; });
    const edges = [...c.X].flatMap((x) => c.active(x).map((e) => { const to = c.next(x, e)!; return `<transition><source ref="${ids.get(x)}"/><target ref="${ids.get(to)}"/><label kind="synchronisation">e_${ident(e)}?</label><label kind="assignment">${sv(c)} = ${code(c).get(to)}, x_${ident(c.name)} = 0</label></transition>`; }));
    templates.push(`<template><name>${xmlEsc(ident(c.name))}</name>${locs.join('')}<init ref="${ids.get(c.x0)}"/>${edges.join('')}</template>`);
  }
  const sched = `id${lid++}`;
  templates.push(`<template><name>Scheduler</name><location id="${sched}" x="0" y="0"><name>run</name></location><init ref="${sched}"/>${events.map((e) => `<transition><source ref="${sched}"/><target ref="${sched}"/><label kind="guard">en_${ident(e)}()</label><label kind="synchronisation">e_${ident(e)}!</label>${supT ? `<label kind="assignment">sup = sup_${ident(e)}(sup)</label>` : ''}</transition>`).join('')}</template>`);
  const sys = `system ${[...comps.map((c) => ident(c.name)), 'Scheduler'].join(', ')};`;
  const queries = doc.checks.filter((c) => c.kind === 'ctl').map((c) => `<query><formula>${xmlEsc(c.formula.replace(/\bAG\b/g, 'A[]').replace(/\bEF\b/g, 'E<>').replace(/\bAF\b/g, 'A<>').replace(/\bEG\b/g, 'E[]'))}</formula><comment>${xmlEsc(c.name ?? '')}</comment></query>`);
  const xml = `<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE nta PUBLIC '-//Uppaal Team//DTD Flat System 1.1//EN' 'http://www.it.uu.se/research/group/darts/uppaal/flat-1_2.dtd'>
<nta>
<declaration>${xmlEsc(decl.join('\n'))}</declaration>
${templates.join('\n')}
<system>${xmlEsc(sys)}</system>
<queries>${queries.join('')}<query><formula>A[] not deadlock</formula><comment>no deadlock</comment></query></queries>
</nta>
`;
  return { name: `${ident(name)}.uppaal.xml`, content: xml, mime: 'application/xml' };
}
