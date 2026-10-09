// @vitest-environment jsdom
import { describe, it, expect, beforeAll, afterEach, vi } from 'vitest';
import { App, ItemType, MobileRobot, Program } from '../src/app';
import { demos } from '../src/demos';
import { ControlModelItem, controlModels, addControlModel, CONTROL_KINDS } from '../src/ctl/model';
import { TEMPLATES, parseDes, parsePetri, parseBt } from '../src/ctl/dsl';
import { analyse } from '../src/ctl/analysis';
import { COURSE_EXAMPLES } from '../src/ctl/examples_dsl';
import { MRS_EXAMPLES } from '../src/mrs/examples';
import { buildControlPanel, newControlModel, newModelDialog, courseExamplesDialog, groupExamplesDialog, analyseAll, analyseModel, controlSection, renderGraph, renderReport, standaloneSvg, supervisorFromDes, startMission, stopMission, traceCsv, actionCatalog, stationWorld, startFleetRun, startModelRun, buildWarehouseSceneFromModel, openControl } from '../src/ui/control_ui';
import { buildGraphEditor } from '../src/ui/graph_editor';
import { buildBtEditor } from '../src/ui/bt_editor';
import { installDomStubs, makeApp, downloads, tickMs, waitFor, toasts, lastToast, clearToasts, ctxItem, ctxItems, closeMenus, currentDialog, dialogs, dialogTitle, setField, fieldInput, clickOk, clickCancel, dialogButton, closeAllDialogs, pointer, key, change, input, buttonByText } from './ui_harness';

let app: App; let shown: string[] = []; let panel: ReturnType<typeof buildControlPanel>; let cp: any;
const pickplace = () => app.setStation(demos.find((d) => d.id === 'pickplace')!.build());
const fieldIn = (root: ParentNode, label: string): HTMLInputElement => { const f = [...root.querySelectorAll<HTMLElement>('label.ge-field, label.ge-check')].find((l) => (l.querySelector('span')?.textContent ?? l.textContent!.trim()) === label || l.textContent!.trim() === label); if (!f) throw new Error(`field "${label}" not found among: ${[...root.querySelectorAll('label.ge-field, label.ge-check')].map((l) => l.querySelector('span')?.textContent ?? l.textContent).join(' | ')}`); return f.querySelector('input,select,textarea') as HTMLInputElement; };
const nodeGroup = (svg: SVGSVGElement, id: string): SVGGElement => { const g = [...svg.querySelectorAll<SVGGElement>('g.ge-node')].find((n) => n.querySelector('title')!.textContent!.split(' — ')[0] === id); if (!g) throw new Error(`node ${id} not drawn among ${[...svg.querySelectorAll('g.ge-node title')].map((t) => t.textContent).join(', ')}`); return g; };

beforeAll(() => {
  installDomStubs(); app = makeApp('pickplace'); (app as any).bottom = { show: (id: string) => shown.push(id) };
  panel = buildControlPanel(app); cp = (app as any).controlPanel; document.body.appendChild(panel.el);
});
afterEach(() => { closeAllDialogs(); clearToasts(); closeMenus(); shown = []; });

describe('control panel', () => {
  it('starts empty, creates models from the New menu and lists them', () => {
    pickplace(); panel.render();
    expect(panel.el.querySelector('.ctl-list')!.textContent).toMatch(/^No control models/);
    buttonByText(panel.el, 'New ▾').click();
    const items = ctxItems(); expect(items.length).toBe(CONTROL_KINDS.length + 7); expect(items[0].classList.contains('disabled')).toBe(true);
    ctxItem('Automata & supervisory control').click();
    expect(controlModels(app.station).length).toBe(1); const m = controlModels(app.station)[0]; expect(m.kind).toBe('des'); expect(m.source).toBe(TEMPLATES.des);
    expect(shown).toEqual(['control']); expect(cp.current()).toBe(m); expect(app.station.selection[0]).toBe(m);
    expect(panel.el.querySelectorAll('.ctl-item').length).toBe(1); expect(panel.el.querySelector('.ctl-item.active .ctl-item-name')!.textContent).toBe('Automata & supervisory control');
    expect((panel.el.querySelector('input.ctl-name') as HTMLInputElement).value).toBe(m.name); expect((panel.el.querySelector('select.ctl-kind') as HTMLSelectElement).value).toBe('des'); expect((panel.el.querySelector('textarea.ctl-editor') as HTMLTextAreaElement).value).toBe(TEMPLATES.des);
    expect(panel.el.querySelector('.ctl-status')!.textContent).toMatch(/^Run on station executes/);
    expect(buttonByText(panel.el, '▶ Run on station')).toBeTruthy();
  });
  it('analyses, renames, retypes, detects the kind, edits the text and deletes', async () => {
    const m = cp.current() as ControlModelItem;
    buttonByText(panel.el, 'Analyse').click();
    const report = panel.el.querySelector<HTMLElement>('.ctl-report-wrap')!;
    expect(report.querySelector('.ctl-verdict')!.textContent).toMatch(/✅|❌/); expect(report.querySelectorAll('details.ctl-section').length).toBeGreaterThan(2); expect(report.querySelector('svg.ctl-graph')).not.toBeNull(); expect(m.lastOk).not.toBeNull();
    expect(panel.el.querySelector('.ctl-item .ctl-dot')!.textContent).toBe('●'); expect(app.logs.at(-1)!.text).toMatch(/^\[control\] /);
    const nameIn = panel.el.querySelector<HTMLInputElement>('input.ctl-name')!; change(nameIn, 'Two machines'); expect(m.name).toBe('Two machines'); expect(panel.el.querySelector('.ctl-item-name')!.textContent).toBe('Two machines');
    const editor = panel.el.querySelector<HTMLTextAreaElement>('textarea.ctl-editor')!;
    input(editor, TEMPLATES.petri); expect(m.source).toBe(TEMPLATES.petri);
    buttonByText(panel.el, 'Detect kind').click(); expect(m.kind).toBe('petri'); expect(lastToast()).toBe('Kind: Petri net'); expect((panel.el.querySelector('select.ctl-kind') as HTMLSelectElement).value).toBe('petri');
    input(editor, 'gibberish first line'); buttonByText(panel.el, 'Detect kind').click(); expect(lastToast()).toBe('Unknown document kind');
    editor.value = TEMPLATES.petri; change(editor); // a committed change enters the model history
    expect((buttonByText(panel.el, '↶') as HTMLButtonElement).disabled).toBe(false);
    buttonByText(panel.el, '↶').click(); expect(m.source).toBe('gibberish first line'); buttonByText(panel.el, '↷').click(); expect(m.source).toBe(TEMPLATES.petri);
    editor.value = 'a'; editor.setSelectionRange(1, 1); editor.dispatchEvent(key('Tab')); expect(editor.value).toBe('a  ');
    editor.value = TEMPLATES.petri; editor.dispatchEvent(key('Enter', { ctrlKey: true })); expect(report.querySelector('.ctl-verdict')!.textContent).toContain('Two processes');
    const kindSel = panel.el.querySelector<HTMLSelectElement>('select.ctl-kind')!; change(kindSel, 'mdp'); expect(m.kind).toBe('mdp'); expect((buttonByText(panel.el, 'Diagram') as HTMLButtonElement).disabled).toBe(true);
    cp.setView('diagram'); expect(lastToast()).toMatch(/^The diagram editor is available/); // the button itself is disabled for this kind
    change(kindSel, 'petri'); expect((buttonByText(panel.el, 'Diagram') as HTMLButtonElement).disabled).toBe(false);
    buttonByText(panel.el, 'Delete').click(); expect(controlModels(app.station).length).toBe(0); expect(cp.current()).toBeNull(); expect(panel.el.querySelector('.ctl-list')!.textContent).toMatch(/^No control models/);
    buttonByText(panel.el, 'Analyse').click(); expect(lastToast()).toBe('Select or create a model first');
    buttonByText(panel.el, 'Diagram').click(); expect(lastToast()).toBe('Select a model first');
    buttonByText(panel.el, '▶ Run mission').click(); expect(lastToast()).toBe('Select a model first');
    buttonByText(panel.el, 'Export ▾').click(); expect(ctxItems()[0].textContent).toBe('No model selected'); closeMenus();
  });
  it('course and group example menus, dialogs, analyse all and the export menu', async () => {
    pickplace(); panel.render();
    buttonByText(panel.el, 'Course examples ▾').click(); expect(ctxItems().length).toBe(COURSE_EXAMPLES.length + 1);
    ctxItem(COURSE_EXAMPLES.find((e) => e.id === 'A-des')!.name).click(); expect(controlModels(app.station).length).toBe(1); expect(cp.current().kind).toBe('des'); expect(cp.current().lastOk).not.toBeNull();
    buttonByText(panel.el, 'Group examples ▾').click(); expect(ctxItems().length).toBe(MRS_EXAMPLES.length + 1 + 14);
    ctxItem(MRS_EXAMPLES.find((e) => e.id === 'pr1-chain')!.name).click(); expect(controlModels(app.station).length).toBe(2); expect(cp.current().kind).toBe('consensus');
    expect(panel.el.querySelector('.ctl-status')!.textContent).toMatch(/^Run on fleet drives/); expect(buttonByText(panel.el, '▶ Run on fleet')).toBeTruthy();
    let p = newModelDialog(app); expect(dialogTitle()).toBe('New control model'); setField('Kind', 'bt'); setField('Name', 'Tree'); clickOk(); await p;
    expect(cp.current().kind).toBe('bt'); expect(cp.current().name).toBe('Tree'); expect(panel.el.querySelector('.ctl-verdict')).not.toBeNull(); // trees are analysed on open
    expect(panel.el.querySelector('.ctl-status')!.textContent).toMatch(/^Zones named as in the tree/);
    p = newModelDialog(app, 'swarm'); expect(dialogTitle()).toBe('New group-control model'); clickCancel(); await p;
    p = courseExamplesDialog(app); setField('Example', 'B-perf'); clickOk(); await p; expect(cp.current().kind).toBe('perf'); expect(lastToast()).toBe('1 model(s) added');
    p = courseExamplesDialog(app); clickCancel(); await p;
    p = groupExamplesDialog(app); setField('Example', 'part:ПР3'); clickOk(); await p; expect(lastToast()).toBe(`${MRS_EXAMPLES.filter((e) => e.part === 'ПР3').length} model(s) added`); expect(cp.current().kind).toBe('allocation');
    expect(panel.el.querySelector('.ctl-status')!.textContent).toMatch(/analysed only/);
    p = groupExamplesDialog(app); clickCancel(); await p;
    const n = controlModels(app.station).length;
    analyseAll(app); expect(lastToast()).toMatch(new RegExp(`(${n} models OK|of ${n} models report issues)`)); expect(controlModels(app.station).every((m) => m.lastOk !== null)).toBe(true);
    // export menu for a DES model
    const des = newControlModel(app, 'des'); buttonByText(panel.el, 'Analyse').click(); expect(panel.el.querySelector('.ctl-report-wrap svg.ctl-graph')).not.toBeNull();
    const d0 = downloads.length;
    buttonByText(panel.el, 'Export ▾').click(); expect(ctxItems().map((e) => e.textContent)).toEqual(['Report (Markdown)', 'Document (.ctl.txt)', 'Graph view (SVG)', 'Supervisor table (JSON)', 'Supervisor runtime (Python)']);
    ctxItem('Report (Markdown)').click(); expect(downloads.at(-1)!.name).toBe(`${des.name}.md`);
    buttonByText(panel.el, 'Export ▾').click(); ctxItem('Document (.ctl.txt)').click(); expect(downloads.at(-1)!.name).toBe(`${des.name}.ctl.txt`);
    buttonByText(panel.el, 'Export ▾').click(); ctxItem('Graph view (SVG)').click(); expect(downloads.at(-1)!.name).toBe(`${des.name}.svg`);
    buttonByText(panel.el, 'Export ▾').click(); ctxItem('Supervisor table (JSON)').click(); expect(downloads.at(-1)!.name).toBe(`${des.name}.supervisor.json`);
    buttonByText(panel.el, 'Export ▾').click(); ctxItem('Supervisor runtime (Python)').click(); expect(downloads.at(-1)!.name).toMatch(/_supervisor\.py$/);
    expect(downloads.length).toBe(d0 + 5); expect(cp.graphSvgs().length).toBeGreaterThan(0); expect(cp.graphSvgs()[0]).toContain('<svg');
    const e4 = app.cmd(() => addControlModel(app.station, 'des', 'E4', COURSE_EXAMPLES.find((e) => e.id === 'A-des-e4')!.source)); cp.open(e4);
    buttonByText(panel.el, 'Export ▾').click(); ctxItem('Supervisor table (JSON)').click(); expect(lastToast()).toBe('Specification is unrealisable — no supervisor');
    buttonByText(panel.el, 'Export ▾').click(); ctxItem('Supervisor runtime (Python)').click(); expect(lastToast()).toBe('Specification is unrealisable — no supervisor');
    expect(supervisorFromDes(e4.source)).toBeNull(); expect(supervisorFromDes('des empty')).toBeNull();
    // Graph view export needs a report with a graph
    cp.open(controlModels(app.station).find((m) => m.kind === 'perf')!); buttonByText(panel.el, 'Analyse').click(); buttonByText(panel.el, 'Export ▾').click(); ctxItem('Graph view (SVG)').click(); expect(lastToast()).toBe('Analyse a model that has a graph view first');
    // properties section
    const sec = controlSection(app, des); expect(sec.textContent).toContain('Automata & supervisory control'); expect(sec.textContent).toContain('✅ OK');
    buttonByText(sec, 'Open in Control tab').click(); expect(shown.at(-1)).toBe('control'); expect(cp.current()).toBe(des);
    buttonByText(sec, 'Analyse').click(); buttonByText(sec, 'Report (Markdown)').click(); expect(downloads.at(-1)!.name).toBe(`${des.name}.md`);
    openControl(app); expect(shown.at(-1)).toBe('control');
  });
  it('runs automata and Petri nets on the station, missions on a mobile robot and group laws on the fleet', async () => {
    pickplace(); panel.render();
    const des = newControlModel(app, 'des'); expect(cp.current()).toBe(des);
    buttonByText(panel.el, '▶ Run on station').click(); const run = cp.run(); expect(run).not.toBeNull(); expect(run.exec).toBeTruthy(); expect(app.worldRunning).toBe(true); expect(app.logs.some((l) => /automaton .* started on the station/.test(l.text))).toBe(true);
    for (let i = 0; i < 5; i++) (app as any).tick(0.5);
    expect(panel.el.querySelector('.ctl-status')!.textContent).toMatch(/firings/); expect(run.status()).toMatch(/^[▶⏹] /);
    buttonByText(panel.el, 'Export ▾').click(); expect(ctxItems().map((e) => e.textContent)).toContain('Mission log'); ctxItem('Mission log').click(); expect(downloads.at(-1)!.name).toBe(`${des.name}.log.txt`);
    buttonByText(panel.el, '⏹ Stop').click(); expect(run.done).toBe(true); expect(app.logs.at(-1)!.text).toBe('[control] run stopped'); expect(panel.el.querySelectorAll('.ctl-runlog div').length).toBe(Math.min(6, run.logLines().length));
    const petri = newControlModel(app, 'petri'); buttonByText(panel.el, '▶ Run on station').click(); expect(cp.run().exec).toBeTruthy(); for (let i = 0; i < 5; i++) (app as any).tick(0.5); expect(cp.run().status()).toMatch(/firings/); cp.stop(); expect(cp.run().done).toBe(true);
    // a mission needs a mobile robot
    const bt = newControlModel(app, 'bt'); buttonByText(panel.el, '▶ Run mission').click(); expect(lastToast()).toMatch(/^Add a mobile robot first/);
    const m = app.addMobileRobot(); panel.render(); expect((panel.el.querySelector('select.ctl-robot') as HTMLSelectElement).options.length).toBe(1);
    (panel.el.querySelector('input.ctl-targets') as HTMLInputElement).value = '2';
    buttonByText(panel.el, '▶ Run mission').click(); const mission = cp.run(); expect(mission.rt).toBeTruthy(); expect(mission.rt.bb.targets).toBe(2); expect(app.worldHooks.length).toBe(1);
    for (let i = 0; i < 10; i++) (app as any).tick(0.1);
    expect(panel.el.querySelector('.ctl-status')!.textContent).toMatch(/ticks/);
    buttonByText(panel.el, 'Export ▾').click(); ctxItem('Mission trace (CSV)').click(); expect(downloads.at(-1)!.name).toBe(`${bt.name}.trace.csv`); expect(traceCsv(mission.rt).split('\n')[0]).toMatch(/^t/);
    cp.stop(); expect(mission.done).toBe(true); expect(app.worldHooks.length).toBe(0); expect(m.state.status).toBe('idle'); if (mission.logLines().length) expect(panel.el.querySelectorAll('.ctl-runlog div').length).toBeGreaterThan(0);
    // supervisor + modes resolved by name; unrealisable supervisor warns
    app.deleteItems(controlModels(app.station).filter((x) => x.kind === 'des')); app.cmd(() => addControlModel(app.station, 'des', 'Sup', COURSE_EXAMPLES.find((e) => e.id === 'A-des-e4')!.source)); app.cmd(() => addControlModel(app.station, 'hybrid', 'Modes', TEMPLATES.hybrid));
    const run2 = startMission(app, bt, m, { maxSeconds: 1 }); expect(toasts().some((x) => /unrealisable/.test(x))).toBe(true); for (let i = 0; i < 12; i++) (app as any).tick(0.1); expect(run2.done).toBe(true); stopMission(app, run2);
    // group-control laws on the fleet
    app.deleteItems([m]);
    const cons = newControlModel(app, 'consensus'); buttonByText(panel.el, '▶ Run on fleet').click(); expect(lastToast()).toMatch(/^Cannot start: the station has no mobile robots/);
    for (let i = 0; i < 4; i++) app.addMobileRobot();
    buttonByText(panel.el, '▶ Run on fleet').click(); const fr = cp.run(); expect(fr.fleet).toBeTruthy(); expect(fr.model).toBe(cons); for (let i = 0; i < 5; i++) (app as any).tick(0.1); expect(fr.status()).toMatch(/t=/);
    buttonByText(panel.el, 'Export ▾').click(); ctxItem('Fleet log').click(); expect(downloads.at(-1)!.name).toBe(`${cons.name}.log.txt`);
    cp.stop(); expect(fr.done).toBe(true);
    const alloc = newControlModel(app, 'allocation'); buttonByText(panel.el, '▶ Run on fleet').click(); expect(lastToast()).toMatch(/^Cannot start: .* analysed, not executed/);
    expect(() => startFleetRun(app, des)).toThrow(/not a group-control model/); expect(() => startModelRun(app, alloc)).toThrow(/only automata/);
    const mdp = newControlModel(app, 'mdp'); buttonByText(panel.el, '▶ Run mission').click(); expect(lastToast()).toMatch(/^Run needs a behavior tree/);
    // warehouse scene
    buildWarehouseSceneFromModel(app, mdp); expect(lastToast()).toBe('Select a warehouse model (group control) first');
    const wh = newControlModel(app, 'warehouse'); expect(buttonByText(panel.el, 'Build scene').style.display).toBe('');
    const nr = app.station.itemsOfType(ItemType.MOBILE_ROBOT).length; buttonByText(panel.el, 'Build scene').click(); expect(lastToast()).toMatch(/^Warehouse scene: \d+ stations, \d+ robots/); expect(app.station.itemsOfType(ItemType.MOBILE_ROBOT).length).toBeGreaterThan(nr); expect(app.station.itemsOfType(ItemType.MAP).length).toBe(1);
    cp.buildScene(); expect(lastToast()).toMatch(/already exist/);
    buttonByText(panel.el, '▶ Run on fleet').click(); expect(cp.run().model).toBe(wh); for (let i = 0; i < 3; i++) (app as any).tick(0.2); cp.stop();
    const cat = actionCatalog(app); expect(cat.robots.some((r) => r.mobile)).toBe(true); expect(cat.robots.some((r) => !r.mobile)).toBe(true); expect(cat.programs[0].name).toBe('PickPlace'); expect(cat.targets).toContain('Home'); expect(cat.desModels).toContain('Sup'); expect(cat.hybridModels).toContain('Modes');
    const m2 = app.station.itemsOfType<MobileRobot>(ItemType.MOBILE_ROBOT)[0]; const world = stationWorld(app); expect(world.forRobot(m2.name)).toBeTruthy(); expect(world.forRobot('nobody')).toBeNull(); world.step!(0.1); expect(Array.isArray(world.events!())).toBe(true); world.stop(); expect(m2.state.status).toBe('idle');
    app.pauseWorld(); app.worldHooks.length = 0;
  });
});

describe('graph views and reports', () => {
  it('renders every kind of graph and report section, and serialises standalone SVG', () => {
    const rep = analyse('des', TEMPLATES.des); const el = renderReport(rep);
    expect(el.querySelector('.ctl-verdict')!.textContent).toContain(rep.title); expect(el.querySelectorAll('svg.ctl-graph').length).toBeGreaterThan(0); expect(el.querySelector('table.grid')).not.toBeNull(); expect(el.textContent).toContain('Metrics');
    const svg = el.querySelector<SVGSVGElement>('svg.ctl-graph')!; const s = standaloneSvg(svg); expect(s).toContain('xmlns="http://www.w3.org/2000/svg"'); expect(s).toContain('<style'); expect(s).toContain('.ctl-node');
    const petri = renderReport(analyse('petri', TEMPLATES.petri)); expect(petri.querySelector('g.ctl-node.transition rect')).not.toBeNull(); expect(petri.querySelector('text.tok')).not.toBeNull();
    const bt = renderReport(analyse('bt', TEMPLATES.bt)); expect(bt.querySelector('g.ctl-node.action rect')).not.toBeNull();
    const cons = renderReport(analyse('consensus', TEMPLATES.consensus)); expect(cons.querySelectorAll('svg').length).toBeGreaterThan(0);
    const g = renderGraph({ kind: 'network', undirected: true, nodes: ['a', 'b', 'c', 'd'].map((id) => ({ id, label: `robot ${id}` })), edges: [{ from: 'a', to: 'b', label: 'link' }, { from: 'b', to: 'c' }, { from: 'c', to: 'a' }, { from: 'a', to: 'c', label: 'a very long edge label that gets cut' }, { from: 'd', to: 'd', label: 'self' }] });
    expect(g.querySelectorAll('g.ctl-node').length).toBe(4); expect(g.querySelectorAll('path.ctl-edge').length).toBe(5); expect(g.textContent).toContain('…'); expect(g.querySelectorAll('[marker-end]').length).toBe(1); // undirected: no arrow heads except on the self loop
    const pos = renderGraph({ kind: 'network', positions: { a: [0, 0], b: [10, 0], c: [5, 8] }, nodes: [{ id: 'a', label: 'a', initial: true, marked: true }, { id: 'b', label: 'b' }, { id: 'c', label: 'c-long-label' }], edges: [{ from: 'a', to: 'b' }, { from: 'b', to: 'a' }] });
    expect(pos.querySelector('g.ctl-node.initial.marked circle.inner')).not.toBeNull(); expect(pos.querySelectorAll('path.ctl-edge').length).toBe(2); expect(pos.textContent).toContain('c-long-label');
    const tree = renderGraph({ kind: 'tree', nodes: [{ id: 'r', label: 'root', kind: 'sequence' }, { id: 'x', label: 'go', kind: 'action' }, { id: 'y', label: 'a condition with a rather long name', kind: 'condition' }], edges: [{ from: 'r', to: 'x' }, { from: 'r', to: 'y' }] });
    expect(tree.querySelectorAll('rect').length).toBe(3); expect(tree.querySelector('g.ctl-node.condition rect')!.getAttribute('rx')).toBe('10');
  });
});

describe('graph editor (automata and Petri nets)', () => {
  const editor = () => cp.graph as ReturnType<typeof buildGraphEditor>;
  const svg = () => editor().el.querySelector<SVGSVGElement>('svg.ge-svg')!;
  const inspector = () => editor().el.querySelector<HTMLElement>('.ge-inspector')!;
  const bar = () => editor().el.querySelector<HTMLElement>('.ge-bar')!;
  it('draws the DES template, adds states and edges, edits the inspector and keeps the text in sync', async () => {
    pickplace(); panel.render();
    const m = newControlModel(app, 'des');
    expect(editor().el.style.display).toBe('none'); expect(editor().kind()).toBeNull(); editor().render(); expect(svg().textContent).toContain('No document loaded');
    buttonByText(panel.el, 'Diagram').click();
    expect(editor().el.style.display).toBe(''); expect(editor().kind()).toBe('des'); expect((panel.el.querySelector('textarea.ctl-editor') as HTMLElement).style.display).toBe('none');
    expect(svg().querySelectorAll('g.ge-node').length).toBeGreaterThan(1); expect(svg().querySelectorAll('g.ge-edge').length).toBeGreaterThan(0); expect(svg().querySelector('g.ge-node.initial')).not.toBeNull();
    expect(bar().textContent).toContain('+ State'); expect(bar().querySelector('select.ge-block')).not.toBeNull();
    // document inspector
    const dI = inspector(); expect(dI.textContent).toContain('Document'); change(fieldIn(dI, 'Uncontrollable'), 'b2, b1'); expect(editor().state().des!.uncontrollable).toEqual(['b2', 'b1']); expect(m.source).toContain('uncontrollable');
    change(fieldIn(inspector(), 'Unobservable'), 'x'); expect(editor().state().des!.unobservable).toEqual(['x']); change(fieldIn(inspector(), 'Faults'), 'f'); expect(editor().state().des!.faults).toEqual(['f']);
    change(fieldIn(inspector(), 'Alphabet'), 'extra'); expect(editor().state().des!.plant[0].events).toContain('extra');
    change(inspector().querySelectorAll<HTMLInputElement>('input.ge-text')[4], 'M1x'); expect(editor().state().des!.plant[0].name).toBe('M1x');
    change(inspector().querySelectorAll<HTMLInputElement>('input.ge-text')[4], 'has space'); expect(lastToast()).toBe('a name without spaces');
    change(inspector().querySelectorAll<HTMLInputElement>('input.ge-text')[0], 'Renamed doc'); expect(editor().state().des!.name).toBe('Renamed doc'); expect(m.source.split('\n')[0]).toBe('des Renamed doc');
    // add a state programmatically and interactively
    const id = editor().addNodeAt(300, 200)!; expect(id).toBeTruthy(); expect(editor().state().selection).toEqual({ type: 'node', id }); expect(editor().state().des!.layout![editor().state().des!.plant[0].name][id]).toEqual([300, 200]);
    expect(inspector().textContent).toContain('State'); change(fieldIn(inspector(), 'Name'), 'S9'); expect(editor().state().selection).toEqual({ type: 'node', id: 'S9' }); expect(m.source).toContain('S9');
    change(fieldIn(inspector(), 'Marked'), 'true'); expect(editor().state().des!.plant[0].marked).toContain('S9'); change(fieldIn(inspector(), 'Initial'), 'true'); expect(editor().state().des!.plant[0].initial).toBe('S9');
    editor().connect('S9', 'S9', ['loop']); expect(editor().state().selection).toEqual({ type: 'edge', from: 'S9', to: 'S9' }); expect(inspector().textContent).toContain('Edge'); expect(svg().querySelectorAll('g.ge-edge.ge-selected').length).toBe(1);
    change(fieldIn(inspector(), 'Events'), 'loop, again'); expect(editor().state().des!.plant[0].transitions.filter((t) => t.from === 'S9' && t.to === 'S9').length).toBe(2);
    change(fieldIn(inspector(), 'Events'), ''); expect(editor().state().selection).toBeNull();
    editor().moveNode('S9', 111, 222); expect(editor().state().des!.layout![editor().state().des!.plant[0].name].S9).toEqual([111, 222]);
    // toolbar modes and canvas interaction
    buttonByText(bar(), '+ State').click(); expect(editor().state().mode).toBe('node'); expect(editor().el.querySelector('.ge-canvas')!.classList.contains('mode-add')).toBe(true);
    const before = svg().querySelectorAll('g.ge-node').length; svg().dispatchEvent(pointer('pointerdown', { clientX: 400, clientY: 300 })); expect(svg().querySelectorAll('g.ge-node').length).toBe(before + 1);
    buttonByText(bar(), '+ Edge').click(); expect(editor().state().mode).toBe('edge'); expect(bar().textContent).toContain('Click the source node');
    const newId = editor().state().selection!.type === 'node' ? (editor().state().selection as any).id : 'q';
    nodeGroup(svg(), 'S9').dispatchEvent(pointer('pointerdown')); expect(bar().textContent).toContain('From S9'); expect(svg().querySelector('g.ge-arc-src')).not.toBeNull();
    nodeGroup(svg(), newId).dispatchEvent(pointer('pointerdown')); expect(dialogTitle()).toBe('New edge'); setField(`Events S9 → ${newId} (comma separated)`, 'go'); clickOk(); await tickMs(1);
    expect(editor().state().des!.plant[0].transitions.some((t) => t.from === 'S9' && t.to === newId && t.event === 'go')).toBe(true); expect(m.source).toContain('go');
    nodeGroup(svg(), 'S9').dispatchEvent(pointer('pointerdown')); nodeGroup(svg(), newId).dispatchEvent(pointer('pointerdown')); expect(dialogTitle()).toBe('Edit edge events'); clickCancel(); await tickMs(1);
    nodeGroup(svg(), 'S9').dispatchEvent(pointer('pointerdown')); svg().dispatchEvent(pointer('pointerdown')); expect(svg().querySelector('g.ge-arc-src')).toBeNull(); // click on the canvas cancels the source
    svg().dispatchEvent(key('Escape')); expect(editor().state().mode).toBe('select');
    // drag a node in select mode
    const g = nodeGroup(svg(), 'S9'); g.dispatchEvent(pointer('pointerdown', { clientX: 111, clientY: 222 })); svg().dispatchEvent(pointer('pointermove', { clientX: 161, clientY: 262 })); svg().dispatchEvent(pointer('pointerup'));
    expect(editor().state().des!.layout![editor().state().des!.plant[0].name].S9).toEqual([161, 262]);
    svg().dispatchEvent(pointer('dblclick')); // not on the background: ignored
    const bg = svg().querySelector('rect.ge-bg')!; const b2 = svg().querySelectorAll('g.ge-node').length; bg.dispatchEvent(new MouseEvent('dblclick', { bubbles: true, clientX: 50, clientY: 50 })); expect(svg().querySelectorAll('g.ge-node').length).toBe(b2 + 1);
    nodeGroup(svg(), 'S9').dispatchEvent(new MouseEvent('dblclick', { bubbles: true })); expect(editor().state().selection).toEqual({ type: 'node', id: 'S9' });
    // delete mode, Delete key, action bindings
    editor().select({ type: 'edge', from: 'S9', to: newId }); svg().dispatchEvent(key('Delete')); expect(editor().state().des!.plant[0].transitions.some((t) => t.from === 'S9' && t.to === newId)).toBe(false);
    editor().select({ type: 'node', id: newId }); buttonByText(inspector(), 'Delete').click(); expect(editor().state().des!.plant[0].states ?? []).not.toContain(newId);
    editor().select({ type: 'node', id: 'S9' });
    const act = inspector().querySelector<HTMLElement>('.ge-action')!; expect(act.textContent).toContain('Entry action (robot)');
    change(act.querySelector('select')!, 'program'); expect(m.source).toMatch(/action .*S9.* program/); expect(inspector().querySelector('.ge-action')!.textContent).toContain('Program'); expect(svg().querySelector('g.ge-node.has-action')).not.toBeNull();
    const ev0 = editor().state().des!.plant[0].events[0]; change(fieldIn(inspector().querySelector('.ge-action')!, 'Done event (fired when finished)'), ev0); expect(editor().state().des!.actions[0].done).toBe(ev0);
    change(inspector().querySelector<HTMLSelectElement>('.ge-action select')!, 'target'); change(fieldIn(inspector().querySelector('.ge-action')!, 'Linear (MoveL)'), 'true'); expect(editor().state().des!.actions[0].args.linear).toBe(true);
    for (const k of ['goto', 'signal', 'wait', 'event', 'set']) { change(inspector().querySelector<HTMLSelectElement>('.ge-action select')!, k); expect(editor().state().des!.actions[0].kind).toBe(k); }
    change(fieldIn(inspector().querySelector('.ge-action')!, 'Value'), '5'); expect(editor().state().des!.actions[0].args.value).toBe('5');
    change(inspector().querySelector<HTMLSelectElement>('.ge-action select')!, 'signal'); change(fieldIn(inspector().querySelector('.ge-action')!, 'Value'), '7'); expect(editor().state().des!.actions[0].args.value).toBe(7);
    change(inspector().querySelector<HTMLSelectElement>('.ge-action select')!, 'wait'); change(fieldIn(inspector().querySelector('.ge-action')!, 'Seconds'), '3'); expect(editor().state().des!.actions[0].value).toBe('3');
    change(inspector().querySelector<HTMLSelectElement>('.ge-action select')!, ''); expect(editor().state().des!.actions.length).toBe(0);
    buttonByText(bar(), 'Delete').click(); expect(editor().state().mode).toBe('delete'); nodeGroup(svg(), 'S9').dispatchEvent(pointer('pointerdown')); expect(editor().state().des!.layout![editor().state().des!.plant[0].name].S9).toBeUndefined();
    const anyEdge = svg().querySelector<SVGGElement>('g.ge-edge')!; const edges = editor().state().des!.plant[0].transitions.length; anyEdge.dispatchEvent(pointer('pointerdown')); expect(editor().state().des!.plant[0].transitions.length).toBeLessThan(edges);
    buttonByText(bar(), 'Select').click();
    // blocks: add automaton / spec, switch, remove
    const blocks = () => editor().state().des!.plant.length + editor().state().des!.specs.length; const nb = blocks();
    buttonByText(bar(), '+ Automaton').click(); expect(blocks()).toBe(nb + 1); expect(editor().state().block).toBe(editor().state().des!.plant.length - 1);
    buttonByText(bar(), '+ Spec').click(); expect(blocks()).toBe(nb + 2); expect(inspector().textContent).toContain('Specification');
    change(bar().querySelector('select.ge-block')!, '0'); expect(editor().state().block).toBe(0); editor().setBlock(99); expect(editor().state().block).toBe(blocks() - 1);
    buttonByText(bar(), 'Remove').click(); expect(blocks()).toBe(nb + 1);
    buttonByText(bar(), 'Auto layout').click(); expect(Object.keys(editor().state().des!.layout![editor().state().des!.plant[0].name] ?? {}).length).toBe(0 + Object.keys(editor().state().des!.layout![editor().state().des!.plant[0].name] ?? {}).length);
    // undo / redo of the model history through the panel buttons
    const src = m.source; buttonByText(panel.el, '↶').click(); expect(m.source).not.toBe(src); buttonByText(panel.el, '↷').click(); expect(m.source).toBe(src);
    cp.undo(); cp.redo(); expect(m.source).toBe(src); expect(cp.history().undo.length).toBeGreaterThan(5);
    // text round trip: what the diagram wrote parses back
    expect(() => parseDes(m.source)).not.toThrow();
    buttonByText(panel.el, '⛶').click(); expect(panel.el.classList.contains('ge-expanded')).toBe(true);
    buttonByText(panel.el, 'Text').click(); expect(editor().el.style.display).toBe('none'); expect(panel.el.classList.contains('ge-expanded')).toBe(false);
    input(panel.el.querySelector('textarea.ctl-editor')!, 'des broken\nautomaton A\n  q0 -- nope'); buttonByText(panel.el, 'Diagram').click(); expect(lastToast()).toMatch(/^Cannot draw the document/);
    input(panel.el.querySelector('textarea.ctl-editor')!, 'des Empty'); buttonByText(panel.el, 'Diagram').click(); expect(editor().state().des!.plant.length).toBe(1); expect(editor().state().des!.plant[0].name).toBe('G');
  });
  it('edits Petri nets: places, transitions, arcs (weights, inhibitors, reverse) and operations', async () => {
    const m = newControlModel(app, 'petri'); buttonByText(panel.el, 'Text').click(); buttonByText(panel.el, 'Diagram').click(); expect(editor().kind()).toBe('petri');
    const spec = () => editor().state().petri!.spec;
    expect(bar().textContent).toContain('+ Place'); expect(bar().textContent).toContain('+ Transition'); expect(bar().textContent).toContain('+ Arc');
    expect(inspector().textContent).toContain('Petri net'); change(fieldIn(inspector(), 'Horizon (s)'), '77'); expect(editor().state().petri!.horizon).toBe(77); change(fieldIn(inspector(), 'Name'), 'Net X'); expect(spec().name).toBe('Net X');
    const p = editor().addNodeAt(50, 50)!; const t = editor().addNodeAt(150, 50, true)!; expect(spec().places.some((x) => x.id === p)).toBe(true); expect(spec().transitions.some((x) => x.id === t)).toBe(true);
    editor().select({ type: 'node', id: p }); expect(inspector().textContent).toContain('Place');
    change(fieldIn(inspector(), 'Tokens'), '3'); expect(spec().places.find((x) => x.id === p)!.tokens).toBe(3); expect(svg().querySelector('text.tok')!.textContent!.length).toBeGreaterThan(0);
    change(fieldIn(inspector(), 'Kind'), 'resource'); expect(spec().places.find((x) => x.id === p)!.kind).toBe('resource'); change(fieldIn(inspector(), 'Capacity'), '5'); change(fieldIn(inspector(), 'Label'), 'Robot free');
    change(fieldIn(inspector(), 'Name'), 'pR'); expect(editor().state().selection).toEqual({ type: 'node', id: 'pR' }); expect(m.source).toContain('pR');
    change(fieldIn(inspector(), 'Name'), '1bad'); expect(toasts().length).toBeGreaterThan(0);
    editor().select({ type: 'node', id: t }); expect(inspector().textContent).toContain('Transition'); change(fieldIn(inspector(), 'Delay (s)'), '2.5'); change(fieldIn(inspector(), 'Rate (1/s)'), '0.1'); change(fieldIn(inspector(), 'Immediate'), 'true'); change(fieldIn(inspector(), 'Weight'), '2'); change(fieldIn(inspector(), 'Priority'), '1'); change(fieldIn(inspector(), 'Label'), 'fire');
    const tr = spec().transitions.find((x) => x.id === t)!; expect(tr.delay).toBe(2.5); expect(tr.immediate).toBe(true); expect(tr.label).toBe('fire');
    expect(inspector().querySelector('.ge-action')!.textContent).toContain('Operation (robot)'); change(inspector().querySelector<HTMLSelectElement>('.ge-action select')!, 'signal'); expect(editor().state().petri!.actions[0].target).toBe(t);
    editor().connect('pR', t); expect(spec().arcs.some((a) => a.from === 'pR' && a.to === t)).toBe(true); expect(inspector().textContent).toContain('Arc');
    change(fieldIn(inspector(), 'Inhibitor'), 'true'); expect(spec().arcs.find((a) => a.from === 'pR' && a.to === t)!.inhibitor).toBe(true); expect(svg().querySelector('g.ge-edge circle')).not.toBeNull();
    change(fieldIn(inspector(), 'Inhibitor'), 'false'); change(fieldIn(inspector(), 'Weight'), '2'); expect(spec().arcs.find((a) => a.from === 'pR' && a.to === t)!.weight).toBe(2); expect(svg().textContent).toContain('2');
    buttonByText(inspector(), 'Reverse').click(); expect(spec().arcs.some((a) => a.from === t && a.to === 'pR')).toBe(true); expect(editor().state().selection).toEqual({ type: 'edge', from: t, to: 'pR' });
    expect(inspector().querySelector('label.ge-check')).toBeNull(); // arcs from a transition cannot be inhibitors
    editor().connect('pR', 'pR'); expect(lastToast()).toMatch(/place and a transition/);
    buttonByText(inspector(), 'Delete').click(); expect(spec().arcs.some((a) => a.from === t && a.to === 'pR')).toBe(false);
    buttonByText(bar(), '+ Transition').click(); const nt = spec().transitions.length; svg().dispatchEvent(pointer('pointerdown', { clientX: 90, clientY: 90 })); expect(spec().transitions.length).toBe(nt + 1);
    buttonByText(bar(), '+ Arc').click(); nodeGroup(svg(), 'pR').dispatchEvent(pointer('pointerdown')); nodeGroup(svg(), t).dispatchEvent(pointer('pointerdown')); expect(spec().arcs.some((a) => a.from === 'pR' && a.to === t)).toBe(true);
    buttonByText(bar(), 'Select').click(); editor().select({ type: 'node', id: 'pR' }); svg().dispatchEvent(key('Backspace')); expect(spec().places.some((x) => x.id === 'pR')).toBe(false); expect(editor().state().petri!.layout.pR).toBeUndefined();
    buttonByText(bar(), 'Auto layout').click(); expect(Object.keys(editor().state().petri!.layout).length).toBe(spec().places.length + spec().transitions.length);
    expect(() => parsePetri(m.source)).not.toThrow(); expect(parsePetri(m.source).horizon).toBe(77);
    buttonByText(panel.el, 'Text').click();
  });
});

describe('behavior-tree editor', () => {
  const bt = () => cp.btGraph as ReturnType<typeof buildBtEditor>;
  const svg = () => bt().el.querySelector<SVGSVGElement>('svg.bt-svg')!;
  const inspector = () => bt().el.querySelector<HTMLElement>('.ge-inspector')!;
  const bar = () => bt().el.querySelector<HTMLElement>('.ge-bar')!;
  const nodes = () => [...svg().querySelectorAll<SVGGElement>('g.bt-node')];
  it('draws the template, adds / moves / reparents / removes nodes and edits every inspector field', async () => {
    pickplace(); panel.render();
    const m = newControlModel(app, 'bt'); buttonByText(panel.el, 'Diagram').click();
    expect(bt().el.style.display).toBe(''); expect(cp.graph.el.style.display).toBe('none'); const doc = () => bt().state().doc!;
    const n0 = nodes().length; expect(n0).toBeGreaterThan(5); expect(svg().querySelectorAll('path.ctl-edge').length).toBe(n0 - 1);
    // document inspector
    expect(inspector().textContent).toContain('Behavior tree'); change(fieldIn(inspector(), 'Name'), 'Tree X'); expect(doc().name).toBe('Tree X'); expect(m.source.split('\n')[0]).toBe('bt Tree X');
    change(fieldIn(inspector(), 'Supervisor (des model)'), 'Sup'); expect(doc().supervisor).toBe('Sup'); change(fieldIn(inspector(), 'Mode automaton (hybrid model)'), ''); expect(doc().modes).toBeUndefined();
    change(fieldIn(inspector(), 'Monitors (LTL, one per line)'), 'G !estop\n\nF done'); expect(doc().monitors).toEqual(['G !estop', 'F done']);
    change(fieldIn(inspector(), 'Leaf models (leaf p= ticks=)'), 'grasp p=0.8 ticks=3\ndock ticks=2'); expect(doc().leafModels.grasp).toEqual({ p: 0.8, ticks: 3 });
    change(fieldIn(inspector(), 'Outcomes (leaf=success,running)'), 'grasp=success,failure'); expect(doc().outcomes.grasp).toEqual(['success', 'failure']);
    // root: + Child of every type
    const rootG = nodes()[0]; rootG.dispatchEvent(pointer('pointerdown')); svg().dispatchEvent(pointer('pointerup')); expect(bt().state().selection).toBe(doc().root); expect(inspector().textContent).toContain('Root node');
    buttonByText(bar(), '+ Child ▾').click(); const types = ctxItems().map((e) => e.textContent!); expect(types.length).toBeGreaterThan(8); ctxItem(types.find((t) => /sequence/.test(t))!.trim()).click();
    const seq = bt().state().selection!; expect(seq.type).toBe('sequence'); expect(doc().root.children!.at(-1)).toBe(seq);
    const act = bt().addChild('action')!; expect(act.type).toBe('action'); expect(seq.children).toContain(act);
    expect((buttonByText(bar(), '+ Child ▾') as HTMLButtonElement).disabled).toBe(true); // leaves take no children
    buttonByText(bar(), '+ Sibling ▾').click(); ctxItem(types.find((t) => /condition/.test(t))!.trim()).click(); const cond = bt().state().selection!; expect(cond.type).toBe('condition'); expect(seq.children!.indexOf(cond)).toBe(seq.children!.indexOf(act) + 1);
    buttonByText(bar(), '↑').click(); expect(seq.children!.indexOf(cond)).toBe(0); buttonByText(bar(), '↓').click(); expect(seq.children!.indexOf(cond)).toBe(1);
    svg().dispatchEvent(key('ArrowUp', { altKey: true })); expect(seq.children!.indexOf(cond)).toBe(0); svg().dispatchEvent(key('ArrowDown', { altKey: true })); expect(seq.children!.indexOf(cond)).toBe(1);
    buttonByText(bar(), '⇤').click(); expect(doc().root.children).toContain(cond); expect(seq.children).not.toContain(cond);
    // condition & action inspectors
    expect(inspector().textContent).toContain('Node'); change(fieldIn(inspector(), 'Expression or blackboard key'), 'battery < 0.3'); expect(cond.fn).toBe('battery < 0.3'); change(fieldIn(inspector(), 'Name'), 'low battery'); expect(cond.name).toBe('low battery');
    bt().select(act); const a = act.args!;
    change(fieldIn(inspector(), 'Binding'), 'goto'); expect(act.fn).toBe('goto'); change(fieldIn(inspector(), 'Zone'), 'home'); expect(a.zone).toBe('home');
    change(fieldIn(inspector(), 'Binding'), 'program'); change(fieldIn(inspector(), 'Program'), 'PickPlace'); expect(a.name).toBe('PickPlace'); change(fieldIn(inspector(), 'Robot'), 'UR10e'); expect(a.robot).toBe('UR10e');
    change(fieldIn(inspector(), 'Binding'), 'move'); change(fieldIn(inspector(), 'Target'), 'Home'); expect(a.target).toBe('Home');
    change(fieldIn(inspector(), 'Binding'), 'signal'); change(fieldIn(inspector(), 'Signal'), 'start'); change(fieldIn(inspector(), 'Value'), '12'); expect(a.value).toBe(12); change(fieldIn(inspector(), 'Value'), 'false'); expect(a.value).toBe(false);
    change(fieldIn(inspector(), 'Binding'), 'wait'); change(fieldIn(inspector(), 'Seconds'), '4'); expect(a.seconds).toBe(4);
    change(fieldIn(inspector(), 'Binding'), 'event'); change(fieldIn(inspector(), 'Emit event'), 'done1'); expect(a.emit).toBe('done1');
    change(fieldIn(inspector(), 'Binding'), 'grasp'); change(fieldIn(inspector(), 'Success probability p'), '0.7'); expect(a.p).toBe(0.7); change(fieldIn(inspector(), 'Success probability p'), ''); expect(a.p).toBeUndefined();
    change(fieldIn(inspector(), 'event= (controllable, asked before start)'), 'g_close'); change(fieldIn(inspector(), 'done= (observed on success)'), 'g_ok'); change(fieldIn(inspector(), 'arrive= / ok= / miss= / put= (world events)'), 'ok=g_ok miss=g_miss');
    expect(a.event).toBe('g_close'); expect(a.done).toBe('g_ok'); expect(a.miss).toBe('g_miss');
    change(fieldIn(inspector(), 'Timeout (s)'), '12'); change(fieldIn(inspector(), 'Precondition'), 'near'); change(fieldIn(inspector(), 'Postcondition'), 'held'); change(fieldIn(inspector(), 'Other arguments (key=value …)'), 'force=5 mode="soft grip"'); change(fieldIn(inspector(), 'Name (optional)'), 'grab');
    expect(act.timeout).toBe(12); expect(act.pre).toBe('near'); expect(act.post).toBe('held'); expect(a.force).toBe(5); expect(a.mode).toBe('soft grip'); expect(act.name).toBe('grab');
    expect(m.source).toContain('action grasp'); expect(m.source).toContain('timeout=12');
    change(fieldIn(inspector(), 'event= (controllable, asked before start)'), ''); expect(a.event).toBeUndefined();
    // composite / decorator inspectors
    bt().select(seq); change(fieldIn(inspector(), 'Name'), 'collect'); expect(seq.name).toBe('collect'); change(fieldIn(inspector(), 'Memory (resume the running child)'), 'true'); expect(seq.memory).toBe(true);
    change(fieldIn(inspector(), 'Type'), 'parallel'); expect(seq.type).toBe('parallel'); change(fieldIn(inspector(), 'Success threshold'), '2'); expect(seq.threshold).toBe(2);
    const retry = bt().addChild('retry')!; expect(retry.type).toBe('retry'); change(fieldIn(inspector(), 'Count'), '4'); expect(retry.count).toBe(4);
    change(fieldIn(inspector(), 'Type'), 'timeout'); change(fieldIn(inspector(), 'Seconds'), '9'); expect(retry.seconds).toBe(9);
    bt().addChild('action'); expect((buttonByText(bar(), '+ Child ▾') as HTMLButtonElement).disabled).toBe(true); // decorators hold one child
    bt().select(retry); change(fieldIn(inspector(), 'Type'), 'sequence'); expect(retry.type).toBe('sequence'); // a decorator with a child becomes a composite
    bt().select(retry.children![0]); change(fieldIn(inspector(), 'Type'), 'fallback'); expect(retry.children![0].type).toBe('fallback'); bt().select(retry); change(fieldIn(inspector(), 'Type'), 'action'); expect(lastToast()).toMatch(/children first/);
    // drag & drop reparent through pointer events
    bt().select(null);
    const titleOf = (g: SVGGElement) => g.querySelector('title')!.textContent!; const condG = nodes().find((g) => titleOf(g).startsWith('condition') && /low battery|0\.3/.test(titleOf(g)))!; const seqG = nodes().find((g) => titleOf(g).startsWith('parallel: collect'))!; expect(condG && seqG).toBeTruthy();
    const seqNode = seq; (document as any).elementFromPoint = () => seqG;
    condG.dispatchEvent(pointer('pointerdown', { clientX: 10, clientY: 10 })); svg().dispatchEvent(pointer('pointermove', { clientX: 12, clientY: 12 })); svg().dispatchEvent(pointer('pointermove', { clientX: 60, clientY: 60 }));
    expect(svg().querySelector('g.bt-drop')).not.toBeNull(); svg().dispatchEvent(pointer('pointerup'));
    expect(seqNode.children).toContain(cond); expect(doc().root.children).not.toContain(cond);
    (document as any).elementFromPoint = () => null;
    const g2 = nodes().find((g) => titleOf(g).startsWith('condition') && /low battery|0\.3/.test(titleOf(g)))!; g2.dispatchEvent(pointer('pointerdown', { clientX: 0, clientY: 0 })); svg().dispatchEvent(pointer('pointermove', { clientX: 90, clientY: 90 })); svg().dispatchEvent(pointer('pointerup')); expect(seqNode.children).toContain(cond);
    // remove
    bt().select(cond); svg().dispatchEvent(key('Delete')); expect(seqNode.children).not.toContain(cond);
    bt().select(doc().root); bt().remove(); expect(doc().root).toBeTruthy(); buttonByText(bar(), 'Delete').click();
    bt().select(act); buttonByText(inspector(), 'Delete').click(); expect(seqNode.children).not.toContain(act);
    bt().select(act); bt().move(1); expect(toasts().length).toBeGreaterThan(0);
    expect(() => parseBt(m.source)).not.toThrow(); expect(analyse('bt', m.source).error).toBeUndefined();
    const kindSel = panel.el.querySelector<HTMLSelectElement>('select.ctl-kind')!; change(kindSel, 'des'); expect(lastToast()).toMatch(/^Cannot draw the document/); expect(bt().el.style.display).toBe(''); // the text is still a tree: the automaton editor cannot take over
    change(kindSel, 'bt'); expect(bt().el.style.display).toBe('');
    cp.open(newControlModel(app, 'des')); expect(cp.graph.el.style.display).toBe(''); cp.open(m); expect(bt().el.style.display).toBe('');
    input(panel.el.querySelector('textarea.ctl-editor')!, 'bt Broken\n  sequence\n    ???'); cp.open(m); // reopening a broken tree falls back to the text view
    buttonByText(panel.el, 'Text').click();
  });
});

export { Program, MobileRobot };
