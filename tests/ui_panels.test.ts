// @vitest-environment jsdom
import { describe, it, expect, beforeAll, afterEach } from 'vitest';
import { App, ItemType, Frame, Target, Program, Instruction, Robot, MobileRobot, FleetItem, MissionItem, SceneObject, Folder, Camera, Component } from '../src/app';
import { Tool } from '../src/core/items/item';
import { demos } from '../src/demos';
import { transl } from '../src/core/math/pose';
import { getLang, setLang } from '../src/ui/i18n';
import { MenuBar } from '../src/ui/menu';
import { BottomPanel } from '../src/ui/bottom';
import { TreePanel } from '../src/ui/tree';
import { PropertiesPanel } from '../src/ui/properties';
import { ProgramEditor } from '../src/ui/program-editor';
import { addControlModel, controlModels } from '../src/ctl/model';
import { TEMPLATES } from '../src/ctl/dsl';
import { addCamera } from '../src/ui/vision_ui';
import { installDomStubs, makeApp, downloads, rendererCalls, tickMs, waitFor, toasts, lastToast, clearToasts, openMenu, clickMenu, ctxItem, ctxItems, closeMenus, currentDialog, dialogs, dialogTitle, setField, fieldInput, clickOk, clickCancel, dialogButton, closeAllDialogs, resolvePickFiles, mouse, key, change, input, buttonByText } from './ui_harness';

let app: App; let bottom: BottomPanel; let menu: MenuBar; let tree: TreePanel; let props: PropertiesPanel; let toggled = 0;
const pickplace = () => app.setStation(demos.find((d) => d.id === 'pickplace')!.build());

beforeAll(() => {
  installDomStubs();
  app = makeApp('pickplace');
  bottom = new BottomPanel(app); (app as any).bottom = bottom;
  menu = new MenuBar(app, () => { toggled++; });
  tree = new TreePanel(app);
  props = new PropertiesPanel(app);
  document.body.append(menu.el, tree.el, bottom.el, props.el);
});
afterEach(() => { closeAllDialogs(); clearToasts(); closeMenus(); });

describe('MenuBar', () => {
  it('has 13 menus with the expected entries', () => {
    expect(menu.el.querySelectorAll('.menu-btn').length).toBe(13);
    expect(menu.el.querySelector('.menu-title')!.textContent).toBe('VerticalBot Studio');
    const expected: Record<string, [number, number]> = { File: [10, 2], Edit: [4, 1], Add: [12, 2], Program: [9, 2], Robot: [9, 2], 'Mobile & Fleet': [9, 2], Agriculture: [5, 1], Control: [7, 2], Group: [7, 2], Tools: [12, 2], Connect: [5, 2], View: [12, 2], Help: [4, 0] };
    for (const [name, [items, seps]] of Object.entries(expected)) {
      const m = openMenu(name);
      expect(m.querySelectorAll('.ctx-item').length, name).toBe(items);
      expect(m.querySelectorAll('.ctx-sep').length, name).toBe(seps);
    }
    openMenu('File'); expect(ctxItem('Demo stations').textContent).toContain('▸'); expect(ctxItem('New station').querySelector('.shortcut')!.textContent).toBe('Ctrl+N');
    openMenu('Robot'); expect(ctxItem('UR10e').textContent).toContain('✓'); closeMenus();
  });
  it('File: new station, demo submenu, save and exports', async () => {
    clickMenu('File', 'New station'); expect(app.station.name).toBe('New station');
    openMenu('File'); ctxItem('Demo stations').dispatchEvent(new Event('pointerenter'));
    const sub = document.querySelector<HTMLElement>('.ctx-menu .ctx-menu')!; expect(sub.querySelectorAll('.ctx-item').length).toBe(demos.length);
    [...sub.querySelectorAll<HTMLElement>('.ctx-item')].find((e) => e.textContent!.startsWith('Pick & place'))!.click();
    expect(app.station.name).toBe('Pick & place cell'); expect(lastToast()).toBe(demos[0].description);
    const n0 = downloads.length;
    clickMenu('File', 'Save station'); expect(downloads.at(-1)!.name).toBe('Pick_place_cell.vbstation'); expect(lastToast()).toBe('Station saved');
    clickMenu('File', 'Export station as RoboDK API script'); expect(downloads.at(-1)!.name).toBe('Pick_place_cell_robodk.py');
    clickMenu('File', 'Export station JSON'); expect(downloads.at(-1)!.name).toBe('Pick_place_cell.vbstation');
    clickMenu('File', 'Export screenshot'); expect(downloads.at(-1)!.name).toBe('station.png'); expect(downloads.at(-1)!.href).toMatch(/^data:image\/png/);
    clickMenu('File', 'Save for Blender'); await waitFor(() => downloads.at(-1)!.name === 'Pick_place_cell_blender.vbstation');
    expect(downloads.length - n0).toBe(5);
    clickMenu('File', 'Export program'); expect(dialogTitle()).toBe('Export PickPlace'); clickCancel();
    clickMenu('File', 'Export URDF package'); expect(dialogTitle()).toBe('Export URDF package'); clickCancel();
    clickMenu('File', 'Open / import'); resolvePickFiles([]); await tickMs(5); expect(dialogs().length).toBe(0);
    menu.save(); expect(downloads.at(-1)!.name).toBe('Pick_place_cell.vbstation');
  });
  it('Edit: undo / redo / delete / rename', async () => {
    pickplace();
    const n = app.station.itemsOfType(ItemType.FRAME).length;
    clickMenu('Add', 'Reference frame'); expect(app.station.itemsOfType(ItemType.FRAME).length).toBe(n + 1);
    clickMenu('Edit', 'Undo'); expect(app.station.itemsOfType(ItemType.FRAME).length).toBe(n);
    clickMenu('Edit', 'Redo'); expect(app.station.itemsOfType(ItemType.FRAME).length).toBe(n + 1);
    app.select(app.station.itemsOfType(ItemType.FRAME).at(-1)!);
    clickMenu('Edit', 'Delete selection'); expect(app.station.itemsOfType(ItemType.FRAME).length).toBe(n);
    clickMenu('Edit', 'Rename station'); expect(dialogTitle()).toBe('Station name'); setField('Name', 'Renamed cell'); clickOk(); await tickMs(1);
    expect(app.station.name).toBe('Renamed cell');
    clickMenu('Edit', 'Rename station'); setField('Name', ''); clickOk(); await tickMs(1); expect(app.station.name).toBe('Renamed cell');
  });
  it('Add: items, primitives, camera, process component and the library dialogs', async () => {
    pickplace();
    const count = (t: ItemType) => app.station.itemsOfType(t).length;
    const f0 = count(ItemType.FRAME), t0 = count(ItemType.TARGET), p0 = count(ItemType.PROGRAM), o0 = count(ItemType.OBJECT), fo0 = count(ItemType.FOLDER), c0 = count(ItemType.COMPONENT);
    clickMenu('Add', 'Reference frame'); clickMenu('Add', 'Target (teach)'); clickMenu('Add', 'Program'); clickMenu('Add', 'Folder'); clickMenu('Add', 'Box'); clickMenu('Add', 'Cylinder'); clickMenu('Add', 'Sphere');
    expect([count(ItemType.FRAME) - f0, count(ItemType.TARGET) - t0, count(ItemType.PROGRAM) - p0, count(ItemType.FOLDER) - fo0, count(ItemType.OBJECT) - o0]).toEqual([1, 1, 1, 1, 3]);
    clickMenu('Add', 'Camera / vision sensor'); await waitFor(() => count(ItemType.CAMERA) === 1); expect(app.station.selection[0]).toBeInstanceOf(Camera);
    clickMenu('Add', 'Process component'); expect(dialogTitle()).toBe('Add process component'); setField('Type', 'process'); setField('Name', 'Grader'); clickOk(); await tickMs(1);
    expect(count(ItemType.COMPONENT)).toBe(c0 + 1); expect((app.station.find('Grader') as Component).behaviour.type).toBe('process');
    clickMenu('Add', 'Robot from library'); expect(dialogTitle()).toBe('Add robot from library'); expect((fieldInput('Robot') as HTMLSelectElement).options.length).toBe(app.library.length); clickCancel();
    clickMenu('Add', 'Robot from online library'); expect(dialogTitle()).toBe('Add robot from online library'); clickCancel();
    clickMenu('Add', 'Import URDF'); resolvePickFiles([]); await tickMs(1);
  });
  it('Program: new / teach / run / pause / stop / validate / export', async () => {
    pickplace();
    clickMenu('Program', 'New program'); const p = app.activeProgram!; expect(p.name).toBe('Prog 2'); expect(p.robot()).toBe(app.activeRobot);
    clickMenu('Program', 'Teach MoveJ'); clickMenu('Program', 'Teach MoveL'); expect(p.instructions().length).toBe(2);
    clickMenu('Program', 'Run'); expect(app.sim.playing).toBe(true);
    clickMenu('Program', 'Pause'); expect(app.sim.playing).toBe(false);
    clickMenu('Program', 'Stop'); expect(app.sim.time).toBe(0);
    clickMenu('Program', 'Validate (compile)'); expect(lastToast()).toMatch(/^OK · [\d.]+ s$/);
    clickMenu('Program', 'Export with post processor'); expect(dialogTitle()).toBe('Export Prog 2'); clickCancel();
    clickMenu('Program', 'Import RoboDK post processors'); resolvePickFiles([]); await tickMs(1);
  });
  it('Robot: home, tool, dialogs guards, collisions and the active robot list', async () => {
    pickplace();
    const r = app.activeRobot!; r.setJoints([10, -80, 90, -100, -90, 10]);
    clickMenu('Robot', 'Home active robot'); expect(r.joints()).toEqual(r.jointsHome());
    clickMenu('Robot', 'Add tool to active robot'); expect(r.tools().length).toBe(2); expect(r.activeTool()!.name).toBe('Tool 2'); expect(app.station.selection[0]).toBe(r.activeTool());
    clickMenu('Robot', 'Fruit picking program'); expect(lastToast()).toBe('No crop rows in the station');
    clickMenu('Robot', 'Follow curve'); expect(lastToast()).toMatch(/^No object with curves\/points/);
    clickMenu('Robot', 'Robot machining project'); expect(lastToast()).toMatch(/^Import an NC/);
    clickMenu('Robot', 'Move with external axes'); await waitFor(() => /not mounted/.test(lastToast()));
    clickMenu('Robot', 'Check collisions now'); expect(app.logs.at(-1)!.text).toMatch(/collision/i);
    expect(app.checkCollisions).toBe(false); clickMenu('Robot', 'Check collisions during'); expect(app.checkCollisions).toBe(true);
    openMenu('Robot'); expect(ctxItem('Check collisions during').textContent).toContain('✓'); ctxItem('Check collisions during').click(); expect(app.checkCollisions).toBe(false);
    app.setActiveRobot(null); clickMenu('Robot', 'Home active robot'); clickMenu('Robot', 'Fruit picking program'); clickMenu('Robot', 'Follow curve'); clickMenu('Robot', 'Move with external axes'); // no active robot: guards
    clickMenu('Robot', 'Add tool to active robot'); expect(lastToast()).toBe('Select a robot');
    clickMenu('Robot', 'Robot machining project'); expect(lastToast()).toBe('Select a robot');
    clickMenu('Robot', 'UR10e'); expect(app.activeRobot).toBe(r); expect(app.station.selection[0]).toBe(r);
  });
  it('Mobile & Fleet: dialogs, navigation stack guards and the world clock', async () => {
    pickplace();
    clickMenu('Mobile & Fleet', 'Navigation & SLAM stack'); await waitFor(() => lastToast() === 'Add a mobile robot first');
    clearToasts(); clickMenu('Mobile & Fleet', 'Export ROS 2 navigation package'); await waitFor(() => lastToast() === 'Add a mobile robot first');
    clickMenu('Mobile & Fleet', 'Add mobile robot'); expect(dialogTitle()).toBe('Add mobile robot'); setField('Type', 'scout'); setField('Name', 'Rover'); clickOk(); await tickMs(1);
    const m = app.station.itemsOfType<MobileRobot>(ItemType.MOBILE_ROBOT)[0]; expect(m.name).toBe('Rover'); expect(m.capabilities).toContain('scout');
    clickMenu('Mobile & Fleet', 'Navigation & SLAM stack'); await waitFor(() => dialogs().length === 1); expect(dialogTitle()).toBe('Navigation & SLAM stack'); clickCancel();
    clearToasts(); clickMenu('Mobile & Fleet', 'Export ROS 2 navigation package'); await waitFor(() => lastToast() === 'Select a navigation stack first');
    clickMenu('Mobile & Fleet', 'Create fleet'); expect(dialogTitle()).toBe('Create fleet'); setField('Number of robots', 1); setField('Robot type', 'amr'); clickOk(); await tickMs(1);
    const fleet = app.station.itemsOfType<FleetItem>(ItemType.FLEET)[0]; expect(fleet.name).toBe('Orchard fleet'); expect(fleet.robotIds.length).toBe(1); expect(app.station.itemsOfType(ItemType.ZONE).length).toBe(1);
    clickMenu('Mobile & Fleet', 'Occupancy map'); setField('Width (m)', 10); setField('Height (m)', 10); clickOk(); await tickMs(1); expect(app.station.itemsOfType(ItemType.MAP).length).toBe(1);
    clickMenu('Mobile & Fleet', 'Zone (charging'); setField('Name', 'No-go'); setField('Kind', 'nogo'); clickOk(); await tickMs(1); expect(app.station.itemsOfType(ItemType.ZONE).length).toBe(2);
    clickMenu('Mobile & Fleet', 'Start world simulation'); expect(app.worldRunning).toBe(true);
    (app as any).tick(0.2); expect(app.worldTime).toBeGreaterThan(0);
    clickMenu('Mobile & Fleet', 'Pause world'); expect(app.worldRunning).toBe(false);
    clickMenu('Mobile & Fleet', 'Reset world'); expect(app.worldTime).toBe(0);
  });
  it('Agriculture: field, mission, GeoJSON import and the demos', async () => {
    pickplace();
    clickMenu('Agriculture', 'Create field / orchard'); expect(dialogTitle()).toBe('Create field / orchard');
    setField('Field width (m)', 30); setField('Field length (m)', 20); setField('Headland (m)', 2); setField('Ripe fraction', 0.8); setField('Build navigation map + headland zones', false); clickOk(); await tickMs(1);
    const field = app.station.itemsOfType(ItemType.FIELD)[0] as any; expect(field.name).toBe('Orchard block A'); expect(field.rows().length).toBeGreaterThan(1); expect(lastToast()).toMatch(/rows, \d+ plants$/); expect(app.station.itemsOfType(ItemType.MAP).length).toBe(0);
    clickMenu('Agriculture', 'Create mission'); expect(dialogTitle()).toBe('Create mission'); setField('Type', 'scout'); setField('Rows (e.g. 1-5,8; empty = all)', '1-2'); setField('Plan now (create fleet tasks)', false); clickOk(); await tickMs(1);
    const mi = app.station.itemsOfType<MissionItem>(ItemType.MISSION)[0]; expect(mi.missionType).toBe('scout'); expect(mi.rowIds.length).toBe(2); expect(mi.fleetId).toBeNull();
    clickMenu('Agriculture', 'Import field from GeoJSON'); resolvePickFiles([]); await tickMs(1);
    clickMenu('Agriculture', 'Demo: apple orchard'); expect(app.station.name).toMatch(/^Apple orchard/);
    clickMenu('Agriculture', 'Demo: greenhouse'); expect(app.station.name).toMatch(/^Greenhouse/);
  });
  it('Control / Group: models, examples, analysis, runs and the scenario dialogs', async () => {
    pickplace();
    clickMenu('Control', 'New control model'); await waitFor(() => dialogs().length === 1); expect(dialogTitle()).toBe('New control model'); setField('Name', 'Cell automaton'); clickOk(); await tickMs(1);
    expect(controlModels(app.station).length).toBe(1); expect(controlModels(app.station)[0].kind).toBe('des'); expect(controlModels(app.station)[0].name).toBe('Cell automaton');
    expect(bottom.el.querySelector('.tab.active')!.textContent).toBe('Control');
    clickMenu('Control', 'Course examples'); await waitFor(() => dialogs().length === 1); setField('Example', 'A-des'); clickOk(); await tickMs(1);
    expect(controlModels(app.station).length).toBe(2); expect(lastToast()).toBe('1 model(s) added');
    bottom.show('program'); clickMenu('Control', 'Open Control tab'); expect(bottom.el.querySelector('.tab.active')!.textContent).toBe('Control');
    clickMenu('Control', 'Analyse all models'); await waitFor(() => /models OK|report issues/.test(lastToast())); expect(controlModels(app.station).every((m) => m.lastOk !== null)).toBe(true);
    clickMenu('Control', 'Run mission'); // the selected model is an automaton: it runs on the station
    const panel = (app as any).controlPanel; expect(panel.run()).not.toBeNull(); expect(app.worldRunning).toBe(true);
    clickMenu('Control', 'Stop mission'); expect(panel.run().done).toBe(true);
    clickMenu('Group', 'New group-control model'); await waitFor(() => dialogs().length === 1); expect(dialogTitle()).toBe('New group-control model'); expect(fieldInput('Kind').value).toBe('consensus'); clickCancel();
    clickMenu('Group', 'Course examples: group control'); await waitFor(() => dialogs().length === 1); setField('Example', 'pr1-chain'); clickOk(); await tickMs(1);
    expect(controlModels(app.station).length).toBe(3); expect(controlModels(app.station).at(-1)!.kind).toBe('consensus');
    for (let i = 0; i < 4; i++) app.addMobileRobot();
    clickMenu('Group', 'Run selected model on the fleet'); expect(panel.run().fleet, lastToast()).toBeTruthy(); expect(app.station.itemsOfType(ItemType.MOBILE_ROBOT).length).toBeGreaterThan(0);
    (app as any).tick(0.1);
    clickMenu('Group', 'Stop fleet run'); expect(panel.run().done).toBe(true);
    clickMenu('Group', 'Build warehouse scene'); expect(lastToast()).toBe('Select a warehouse model (group control) first');
    clickMenu('Group', 'Open Control tab');
    clickMenu('Group', 'Demo scenarios: group control'); await waitFor(() => dialogs().length === 1); expect((currentDialog().querySelector('select') as HTMLSelectElement).value).toBe('group'); dialogButton('Close').click();
    clickMenu('Control', 'Demo scenarios: control design'); await waitFor(() => dialogs().length === 1); expect(currentDialog().querySelectorAll('.nav-rec').length).toBeGreaterThan(5); dialogButton('Close').click();
    app.pauseWorld();
  });
  it('Tools: collision map, measure, camera, vision, exports and station tabs', async () => {
    pickplace();
    clickMenu('Tools', 'Collision map'); expect(dialogTitle()).toBe('Collision map'); dialogButton('Close').click(); await tickMs(1);
    app.select(null); clickMenu('Tools', 'Measure'); expect(lastToast()).toBe('Select two items (Shift+click) to measure');
    clickMenu('Tools', 'Camera parameters'); expect(lastToast()).toBe('Select a camera item');
    const cam = addCamera(app); app.select(cam);
    clickMenu('Tools', 'Camera parameters'); expect(dialogTitle()).toBe('Camera Camera 1'); clickCancel();
    clickMenu('Tools', 'Machine vision stack'); await waitFor(() => dialogs().length === 1); expect(dialogTitle()).toBe('Machine vision stack — Camera 1'); clickCancel();
    bottom.show('program'); clickMenu('Tools', 'Run vision pipeline'); expect(bottom.el.querySelector('.tab.active')!.textContent).toBe('Vision');
    clearToasts(); clickMenu('Tools', 'Export ROS 2 perception package'); await waitFor(() => lastToast() === 'Select a vision stack first');
    app.select(null); app.deleteItems([cam]); clearToasts(); clickMenu('Tools', 'Export ROS 2 perception package'); await waitFor(() => lastToast() === 'Add a camera first');
    clickMenu('Tools', 'Record video'); expect(lastToast()).toBe('Video capture is not supported in this browser');
    const n0 = downloads.length;
    clickMenu('Tools', 'Export 3D HTML'); await waitFor(() => downloads.length === n0 + 1); expect(downloads.at(-1)!.name).toMatch(/_3d\.html$/);
    clickMenu('Tools', 'Export scene as glTF'); await waitFor(() => downloads.length === n0 + 2); expect(downloads.at(-1)!.name).toMatch(/\.glb$/);
    clickMenu('Tools', 'Export animation as glTF'); expect(dialogTitle()).toBe('Export animation (glTF for Blender)'); clickCancel();
    clickMenu('Tools', 'New station tab'); expect(app.stations.length).toBe(2); expect(app.station.name).toBe('Station 2');
    clickMenu('Tools', 'Close station tab'); expect(app.stations.length).toBe(1); expect(app.station.name).toBe('Pick & place cell');
  });
  it('Connect: rosbridge, server info and VDA 5050 dialogs', async () => {
    clickMenu('Connect', 'ROS 2 via rosbridge'); expect(dialogTitle()).toBe('Connect to ROS 2 (rosbridge_server)'); expect(fieldInput('rosbridge websocket URL').value).toBe('ws://localhost:9090'); clickCancel(); await tickMs(1);
    clickMenu('Connect', 'Disconnect rosbridge'); expect((app as any).ros).toBeNull();
    clickMenu('Connect', 'Studio server'); expect(dialogTitle()).toBe('Studio server'); expect(currentDialog().textContent).toContain('20500'); clickOk();
    clickMenu('Connect', 'VDA 5050 fleet interface'); expect(lastToast()).toMatch(/^Start the studio server/);
    clickMenu('Connect', 'Disconnect VDA 5050'); await tickMs(1); expect(toasts().some((x) => x === 'VDA 5050 disconnected')).toBe(false);
  });
  it('View: renderer commands, toggles and the language switch', () => {
    const n = rendererCalls.length;
    for (const item of ['Fit all', 'Top', 'Front', 'Side', 'Isometric', 'Gizmo: translate', 'Gizmo: rotate']) clickMenu('View', item);
    expect(rendererCalls.slice(n)).toEqual(['fitAll', 'setView', 'setView', 'setView', 'setView', 'setGizmoMode', 'setGizmoMode']);
    openMenu('View'); expect(ctxItem('Show reference frames').textContent).toContain('✓'); ctxItem('Show reference frames').click(); expect(app.renderer.showFrames).toBe(false);
    clickMenu('View', 'Show targets'); expect(app.renderer.showTargets).toBe(false);
    clickMenu('View', 'Show robot reach'); expect(app.renderer.showReach).toBe(true); expect(rendererCalls.at(-1)).toBe('rebuildAll');
    clickMenu('View', 'Show reference frames'); clickMenu('View', 'Show targets'); clickMenu('View', 'Show robot reach');
    const t0 = toggled; clickMenu('View', 'Toggle bottom panel'); expect(toggled).toBe(t0 + 1);
    clickMenu('View', 'Language'); expect(getLang()).toBe('ru'); setLang('en');
  });
  it('Help: scenarios, quick start, notes and about', async () => {
    clickMenu('Help', 'Demo scenarios'); await waitFor(() => dialogs().length === 1); expect(dialogTitle()).toMatch(/^Demo scenarios/); dialogButton('Close').click();
    clickMenu('Help', 'Quick start'); expect(dialogTitle()).toBe('Help'); expect(currentDialog().querySelectorAll('li').length).toBe(5); clickOk();
    clickMenu('Help', 'RoboDK API compatibility notes');
    clickMenu('Help', 'About'); expect(lastToast()).toMatch(/^VerticalBot Studio/);
  });
});

describe('BottomPanel', () => {
  it('switches between the ten tabs', () => {
    const ids = ['program', 'sim', 'fleet', 'process', 'nav', 'vision', 'control', 'camera', 'console', 'log'];
    const pages = [...bottom.el.querySelectorAll<HTMLElement>('.tab-page')]; expect(pages.map((p) => p.dataset.tab)).toEqual(ids);
    const tabs = [...bottom.el.querySelectorAll<HTMLButtonElement>('.tab-bar .tab')]; expect(tabs.map((t) => t.textContent)).toEqual(['Program', 'Simulation', 'Fleet', 'Process', 'Navigation', 'Vision', 'Control', 'Camera', 'Console (RoboDK API)', 'Log']);
    for (const [i, id] of ids.entries()) {
      tabs[i].click();
      expect(tabs[i].classList.contains('active')).toBe(true); expect(tabs.filter((t) => t.classList.contains('active')).length).toBe(1);
      expect(pages.filter((p) => p.style.display !== 'none').map((p) => p.dataset.tab)).toEqual([id]);
    }
    bottom.show('program');
  });
  it('Simulation tab controls the program playback and the world clock', () => {
    pickplace();
    bottom.show('sim');
    const page = bottom.el.querySelector<HTMLElement>('.sim-panel')!;
    buttonByText(page, '▶ Run').click(); expect(app.sim.playing).toBe(true);
    buttonByText(page, '⏸ Pause').click(); expect(app.sim.playing).toBe(false);
    buttonByText(page, '⏭ End').click(); expect(app.sim.time).toBeCloseTo(app.sim.duration, 5); expect(page.querySelector('.time-label')!.textContent).toMatch(/^[\d.]+ \/ [\d.]+ s$/);
    buttonByText(page, '⏹ Stop').click(); expect(app.sim.time).toBe(0);
    const speed = page.querySelector('select')!; change(speed, '2'); expect(app.simSpeed).toBe(2); change(speed, '1');
    const range = page.querySelector<HTMLInputElement>('input.timeline')!; expect(Number(range.max)).toBeCloseTo(app.sim.duration, 5);
    app.runProgram(); input(range, '1'); expect(app.sim.playing).toBe(false); expect(app.sim.time).toBeCloseTo(1, 5);
    buttonByText(page, '▶ Start').click(); expect(app.worldRunning).toBe(true);
    buttonByText(page, '⏸ Pause', ).click(); // first "⏸ Pause" is the program one; the world pause is the second
    const pauses = [...page.querySelectorAll('button')].filter((b) => b.textContent === '⏸ Pause'); pauses[1].click(); expect(app.worldRunning).toBe(false);
    buttonByText(page, '↺ Reset').click(); expect(app.worldTime).toBe(0);
    app.stopProgram();
  });
  it('Fleet tab lists fleets, robots, tasks and missions', () => {
    pickplace(); bottom.show('fleet');
    const body = bottom.el.querySelector<HTMLElement>('.tab-page[data-tab=fleet]')!;
    expect(body.textContent).toContain('No fleet');
    app.setStation(demos.find((d) => d.id === 'orchard')!.build()); app.startWorld(); for (let i = 0; i < 5; i++) (app as any).tick(0.2); app.pauseWorld();
    bottom.show('fleet');
    expect(body.querySelector('.fleet-head')!.textContent).toContain('Harvest fleet'); expect(body.querySelectorAll('table.grid')[0].querySelectorAll('tbody tr').length).toBe(4);
    expect(body.querySelectorAll('table.grid.small tbody tr').length).toBeGreaterThan(0); expect(body.querySelectorAll('.mission-row').length).toBe(2);
    (body.querySelector('table.grid tbody tr') as HTMLElement).click(); expect(app.station.selection[0]).toBeInstanceOf(MobileRobot);
    expect(body.querySelector('.bar-fill')).not.toBeNull();
  });
  it('Process tab shows statistics, the chart and the signals', () => {
    app.setStation(demos.find((d) => d.id === 'packing')!.build()); bottom.show('process');
    const body = bottom.el.querySelector<HTMLElement>('.tab-page[data-tab=process]')!;
    expect(body.querySelector('canvas.mini-chart')).not.toBeNull(); expect(body.querySelectorAll('table.grid tbody tr').length).toBe(app.station.itemsOfType(ItemType.COMPONENT).length);
    app.startWorld(); for (let i = 0; i < 12; i++) { (app as any).tick(1); bottom.show('process'); } app.pauseWorld();
    expect(body.textContent).toMatch(/Process clock [\d.]+ s/); expect(body.textContent).toMatch(/output \d+/);
    app.processSim.setSignal('door', true); bottom.show('process'); expect(body.textContent).toContain('Signals: door=true');
    (body.querySelector('table.grid tbody tr') as HTMLElement).click(); expect(app.station.selection[0]).toBeInstanceOf(Component);
    app.newStation(); bottom.show('process'); expect(body.textContent).toContain('No process components');
    app.resetWorld();
  });
  it('Camera tab renders the view from an item or a camera', () => {
    pickplace(); bottom.show('camera');
    const page = bottom.el.querySelector<HTMLElement>('.cam-panel')!; const info = page.querySelector('.hint')!;
    app.select(null); buttonByText(page, 'Render view').click(); expect(info.textContent).toBe('No camera / item selected.');
    app.select(app.activeRobot); buttonByText(page, 'Render view').click(); expect(info.textContent).toBe('View from UR10e'); expect(rendererCalls.at(-1)).toBe('renderFromItem');
    const cam = addCamera(app); app.select(cam); buttonByText(page, 'Render view').click(); expect(info.textContent).toMatch(/^View from Camera 1 — \d+ fruit detections/);
    app.select(null); buttonByText(page, 'Render view').click(); expect(info.textContent).toMatch(/^View from Camera 1/); // falls back to the first camera
    const n = downloads.length; buttonByText(page, 'Save PNG').click(); expect(downloads.length).toBe(n + 1); expect(downloads.at(-1)!.name).toBe('camera.png');
  });
  it('Console runs scripts against the RoboDK-compatible API', async () => {
    pickplace(); bottom.show('console');
    const page = bottom.el.querySelector<HTMLElement>('.console')!; const ta = page.querySelector('textarea')!; const out = page.querySelector('.console-out')!;
    const run = buttonByText(page, 'Run (Ctrl+Enter)');
    ta.value = ''; run.click(); expect(out.children.length).toBe(0);
    ta.value = '1 + 1'; run.click(); await waitFor(() => out.querySelector('.c-out'), 3000, 'first output'); expect(out.querySelector('.c-out')!.textContent).toBe('2');
    ta.value = 'print(RDK.Item("", ITEM_TYPE_ROBOT).Name(), RDK.ItemList().length);'; run.click(); await waitFor(() => out.querySelectorAll('.c-out').length === 2, 3000, 'print: ' + out.innerHTML); expect(out.querySelectorAll('.c-out')[1].textContent).toMatch(/^UR10e \d+$/);
    ta.value = 'const robot = RDK.Item("", ITEM_TYPE_ROBOT); return robot.Pose()'; run.click(); await waitFor(() => out.querySelectorAll('.c-out').length === 3); expect(out.querySelectorAll('.c-out')[2].textContent!.split('\n').length).toBe(4);
    ta.value = 'return RDK.Item("", ITEM_TYPE_ROBOT)'; run.click(); await waitFor(() => out.querySelectorAll('.c-out').length === 4); expect(out.querySelectorAll('.c-out')[3].textContent).toBe('Item(UR10e)');
    ta.value = 'return { a: 1, q: new Float64Array([1, 2]) }'; run.click(); await waitFor(() => out.querySelectorAll('.c-out').length === 5); expect(out.querySelectorAll('.c-out')[4].textContent).toContain('"a": 1');
    ta.value = 'nope()'; ta.dispatchEvent(key('Enter', { ctrlKey: true })); await waitFor(() => out.querySelector('.c-err')); expect(out.querySelector('.c-err')!.textContent).toMatch(/nope is not defined/);
    const n = app.station.itemsOfType(ItemType.FRAME).length;
    ta.value = 'RDK.AddFrame("Console frame")'; run.click(); await waitFor(() => app.station.itemsOfType(ItemType.FRAME).length === n + 1);
    const ex = page.querySelector('select')!; change(ex, 'List items'); expect(ta.value).toMatch(/^for \(const it of RDK.ItemList\(\)\)/); expect(ex.value).toBe('');
    ta.value = ''; ta.dispatchEvent(key('ArrowUp', { altKey: true })); expect(ta.value).toBe('RDK.AddFrame("Console frame")');
    ta.dispatchEvent(key('ArrowUp', { altKey: true })); expect(ta.value).toBe('nope()');
    ta.dispatchEvent(key('ArrowDown', { altKey: true })); ta.dispatchEvent(key('ArrowDown', { altKey: true })); expect(ta.value).toBe('');
    buttonByText(page, 'Clear').click(); expect(out.children.length).toBe(0);
  });
  it('Log tab receives the application log', () => {
    bottom.show('log');
    const out = bottom.el.querySelector<HTMLElement>('.log-out')!; const n = out.children.length;
    app.log('hello log'); app.log('careful', 'warn');
    expect(out.children.length).toBe(n + 2); expect(out.lastElementChild!.className).toBe('log-warn'); expect(out.lastElementChild!.textContent).toMatch(/careful$/);
    bottom.show('program');
  });
});

describe('TreePanel', () => {
  const rows = () => [...tree.el.querySelectorAll<HTMLElement>('.tree-row')];
  const rowOf = (name: string) => rows().find((r) => r.querySelector('.tree-name')!.childNodes[0].textContent === name)!;
  it('renders the station with icons, badges and selection', () => {
    pickplace(); tree.render();
    expect(rows().length).toBeGreaterThan(20); expect(rows()[0].hasAttribute('draggable')).toBe(false); expect(rows()[1].getAttribute('draggable')).toBe('true');
    expect(rowOf('UR10e').querySelector('.badge')!.textContent).toBe('6 DOF'); expect(rowOf('UR10e').classList.contains('active')).toBe(true);
    expect(rowOf('PickPlace').querySelector('.badge')!.textContent).toMatch(/^\d+ ins/); expect(rowOf('Home').querySelector('.badge')!.textContent).toBe('J');
    expect(rowOf('Vacuum gripper').querySelector('.badge')!.textContent).toBe('active'); expect(rowOf('Table').querySelector('.badge')!.textContent).toBe('ref');
    expect(rows().some((r) => r.querySelector('.tree-name')!.textContent!.startsWith('MoveJ'))).toBe(false); // instructions stay out of the tree
    rowOf('Home').click(); expect(app.station.selection[0].name).toBe('Home'); tree.render(); expect(rowOf('Home').classList.contains('selected')).toBe(true);
    rowOf('Pick 1').dispatchEvent(mouse('click', { ctrlKey: true })); expect(app.station.selection.length).toBe(2);
    rowOf('Pedestal').dispatchEvent(mouse('click', { shiftKey: true })); expect(app.station.selection.length).toBe(3);
    app.select(null);
  });
  it('double-click moves the robot / runs the program / focuses, twisty and eye toggle', async () => {
    pickplace(); tree.render();
    const r = app.activeRobot!; r.setJoints([30, -80, 90, -100, -90, 10]);
    rowOf('Home').dispatchEvent(mouse('dblclick')); expect(r.joints().map(Math.round)).toEqual([0, -100, 110, -100, -90, 0]);
    rowOf('PickPlace').dispatchEvent(mouse('dblclick')); expect(app.sim.playing).toBe(true); app.stopProgram();
    rowOf('Pedestal').dispatchEvent(mouse('dblclick')); expect(rendererCalls.at(-1)).toBe('focusItem');
    const n = rows().length; rowOf('Table').querySelector<HTMLElement>('.twisty')!.click(); expect(rows().length).toBeLessThan(n); expect(rowOf('Table').querySelector('.twisty')!.textContent).toBe('▸');
    rowOf('Table').querySelector<HTMLElement>('.twisty')!.click(); expect(rows().length).toBe(n);
    expect(rowOf('Home').querySelector('.twisty')!.classList.contains('empty')).toBe(true); expect(rows()[0].querySelector('.tree-eye')).toBeNull();
    rowOf('Pedestal').querySelector<HTMLElement>('.tree-eye')!.click(); expect(app.station.find('Pedestal')!.visible).toBe(false); await tickMs(40); expect(rowOf('Pedestal').classList.contains('hidden-item')).toBe(true); expect(rowOf('Pedestal').querySelector('.tree-eye')!.textContent).toBe('◌');
    rowOf('Pedestal').querySelector<HTMLElement>('.tree-eye')!.click(); expect(app.station.find('Pedestal')!.visible).toBe(true);
  });
  it('filters, renames, opens context menus and drags items onto new parents', async () => {
    pickplace(); tree.render();
    const filter = tree.el.querySelector<HTMLInputElement>('.tree-filter')!; input(filter, 'pick 1'); expect(rows().map((r) => r.querySelector('.tree-name')!.childNodes[0].textContent)).toEqual(['Pick & place cell', 'Table', 'Pick 1']);
    input(filter, ''); expect(rows().length).toBeGreaterThan(20);
    const p = tree.rename(app.station.find('Pedestal')!); expect(dialogTitle()).toBe('Rename'); setField('Name', '  Base  '); clickOk(); await p; expect(app.station.find('Base')).not.toBeNull(); tree.render();
    rowOf('Base').querySelector<HTMLElement>('.tree-name')!.dispatchEvent(mouse('dblclick')); expect(dialogTitle()).toBe('Rename'); setField('Name', '   '); clickOk(); await tickMs(1); expect(app.station.find('Base')).not.toBeNull();
    rowOf('Home').dispatchEvent(mouse('contextmenu', { clientX: 5, clientY: 5 })); expect(app.station.selection[0].name).toBe('Home'); expect(ctxItems().map((e) => e.textContent)).toContain('Move robot here (MoveJ)'); closeMenus();
    const list = tree.el.querySelector<HTMLElement>('.tree-list')!; list.dispatchEvent(mouse('contextmenu')); expect(ctxItems().some((e) => e.textContent === 'Focus')).toBe(true); expect(ctxItems().some((e) => e.textContent!.startsWith('Delete'))).toBe(false); closeMenus();
    // drag Box 1 onto the Base object (static reparent), then with Shift (keeps the local pose)
    const box = app.station.find('Box 1')!; const abs = box.poseAbs();
    rowOf('Box 1').dispatchEvent(new Event('dragstart', { bubbles: true }));
    const over = new Event('dragover', { bubbles: true, cancelable: true }); rowOf('Base').dispatchEvent(over); expect(over.defaultPrevented).toBe(true); expect(rowOf('Base').classList.contains('drop')).toBe(true);
    rowOf('Base').dispatchEvent(new Event('dragleave', { bubbles: true })); expect(rowOf('Base').classList.contains('drop')).toBe(false);
    rowOf('Base').dispatchEvent(new Event('drop', { bubbles: true, cancelable: true }));
    expect(box.parent!.name).toBe('Base'); tree.render(); for (let i = 12; i < 15; i++) expect(box.poseAbs()[i]).toBeCloseTo(abs[i], 6);
    tree.render();
    rowOf('Box 1').dispatchEvent(new Event('dragstart', { bubbles: true })); rowOf('Box 1').dispatchEvent(new Event('drop', { bubbles: true })); expect(box.parent!.name).toBe('Base'); // onto itself: ignored
    rowOf('Table').dispatchEvent(new Event('dragstart', { bubbles: true })); const bad = new Event('dragover', { bubbles: true, cancelable: true }); rowOf('Table top').dispatchEvent(bad); expect(bad.defaultPrevented).toBe(false); // ancestor onto descendant
    tree.render(); rowOf('Base').dispatchEvent(new Event('dragstart', { bubbles: true })); const shiftDrop = Object.assign(new Event('drop', { bubbles: true }), { shiftKey: true }); rowOf('Table').dispatchEvent(shiftDrop); expect(app.station.find('Base')!.parent!.name).toBe('Table');
  });
});

describe('PropertiesPanel', () => {
  const body = () => props.el.querySelector<HTMLElement>('.panel-body')!;
  const sectionTitles = () => [...props.el.querySelectorAll('details.section > summary')].map((s) => s.textContent);
  it('shows the hint without a selection and the station / object / folder editors', () => {
    pickplace(); app.select(null); expect(props.el.querySelector('.panel-body')!.textContent).toBe('Select an item in the tree or the 3D view.');
    app.select(app.station); expect(app.station.selection).toEqual([]); // the station root is never part of the selection
    const ped = app.station.find('Pedestal') as SceneObject; ped.setParam('note', 'x'); ped.setParam('obj', { a: 1 }); app.select(ped);
    expect(sectionTitles()).toEqual(['Pose relative to Station (mm / deg)', 'Object', 'Parameters']); expect(body().textContent).toContain('cylinder');
    const col = props.el.querySelector<HTMLInputElement>('input[type=color]')!; change(col, '#112233'); expect(ped.color).toBe('#112233'); expect(ped.geometry[0].color).toBe('#112233');
    app.select(app.station.addChild(new Folder('F'))); expect(sectionTitles()).toEqual(['Pose relative to Station (mm / deg)']);
  });
  it('pose editor: all Euler conventions edit the pose and follow live changes', async () => {
    pickplace();
    const box = app.station.find('Box 2')!; app.select(box);
    expect(sectionTitles()[0]).toBe('Pose relative to Table (mm / deg)');
    const cells = () => [...props.el.querySelectorAll<HTMLInputElement>('.pose-grid input')];
    expect(cells().length).toBe(6); expect(Number(cells()[0].value)).toBeCloseTo(320, 3);
    change(cells()[0], '400'); expect(box.pose()[12]).toBeCloseTo(400, 6);
    const mode = props.el.querySelector<HTMLSelectElement>('.field-select select')!;
    change(mode, 'kuka'); expect(cells().length).toBe(6); change(cells()[3], '90'); expect(box.pose()[0]).toBeCloseTo(0, 6);
    change(mode, 'ur'); change(cells()[5], '0'); change(cells()[3], '0'); change(cells()[4], '0'); expect(box.pose()[0]).toBeCloseTo(1, 6);
    change(mode, 'abb'); expect(cells().length).toBe(7); change(cells()[2], '600'); expect(box.pose()[14]).toBeCloseTo(600, 6);
    change(mode, 'xyzrpw'); change(cells()[1], '10');
    box.setPose(transl(1, 2, 3)); await tickMs(60); expect(cells().map((c) => Number(c.value)).slice(0, 3)).toEqual([1, 2, 3]); expect(body().textContent).toContain('abs: ');
  });
  it('robot editor: joints, jog, home, teach, parameters, frame and tool selectors', async () => {
    pickplace();
    const r = app.activeRobot!; app.select(r);
    expect(sectionTitles().slice(0, 5)).toEqual(['Pose relative to Station (mm / deg)', 'Joints (deg / mm)', 'Cartesian jog (tool frame)', 'Status', 'Frames']);
    const sliders = [...props.el.querySelectorAll<HTMLInputElement>('.joint-row input[type=range]')], nums = [...props.el.querySelectorAll<HTMLInputElement>('.joint-row input.joint-num')];
    expect(sliders.length).toBe(6);
    input(sliders[0], '25'); expect(r.joints()[0]).toBeCloseTo(25, 6); expect(nums[0].value).toBe('25.00'); change(sliders[0]);
    change(nums[1], '-45'); expect(r.joints()[1]).toBeCloseTo(-45, 6); expect(sliders[1].value).toBe('-45');
    change(nums[2], '9999'); expect(r.joints()[2]).toBeLessThan(9999); // clamped to the joint limits
    r.setJoints([5, -90, 90, -90, -90, 0]); await tickMs(60); expect(props.el.querySelector<HTMLInputElement>('.joint-row input.joint-num')!.value).toBe('5.00'); expect(body().textContent).toContain('TCP in frame');
    buttonByText(body(), 'Home').click(); expect(r.joints()).toEqual(r.jointsHome());
    const t0 = app.station.itemsOfType(ItemType.TARGET).length; buttonByText(body(), 'Teach target').click(); expect(app.station.itemsOfType(ItemType.TARGET).length).toBe(t0 + 1);
    app.select(r);
    const n0 = app.activeProgram!.instructions().length; buttonByText(body(), '+ MoveJ').click(); app.select(r); buttonByText(body(), '+ MoveL').click(); expect(app.activeProgram!.instructions().length).toBe(n0 + 2);
    app.select(r); const q = r.joints().slice();
    buttonByText(body(), '+X').click(); expect(r.joints()).not.toEqual(q);
    buttonByText(body(), '-Rz').click(); buttonByText(body(), '+Ry').click(); buttonByText(body(), '-Y').click(); buttonByText(body(), '+Z').click();
    const stepIn = [...props.el.querySelectorAll<HTMLInputElement>('.btn-row input.joint-num')][0]; stepIn.value = '100000'; buttonByText(body(), '+X').click(); expect(app.logs.at(-1)!.text).toBe('Jog target unreachable');
    buttonByText(body(), 'Parameters…').click(); expect(dialogTitle()).toBe('UR10e parameters'); setField('Linear speed (mm/s)', 333); setField('Controller IP / ROS namespace', '10.0.0.5'); clickOk(); await tickMs(1); expect(r.motion.speedLinear).toBe(333); expect(r.connection.ip).toBe('10.0.0.5');
    app.setActiveRobot(null); app.select(r); buttonByText(body(), 'Set active').click(); expect(app.activeRobot).toBe(r);
    const fieldIn = (label: string) => [...props.el.querySelectorAll<HTMLElement>('label.field')].find((l) => l.querySelector('.field-label')!.textContent === label)!.querySelector('select')!;
    change(fieldIn('Active reference frame'), ''); expect(r.activeFrame()).not.toBe(app.station.find('Table')); app.select(r); change(fieldIn('Active reference frame'), app.station.find('Table')!.id); expect(r.activeFrame()!.name).toBe('Table');
    app.select(r); change(fieldIn('Active tool'), ''); expect(r.activeToolId).toBeNull(); app.select(r); change(fieldIn('Active tool'), r.tools()[0].id); expect(r.activeTool()!.name).toBe('Vacuum gripper');
  });
  it('target, tool, program and instruction editors', async () => {
    pickplace();
    const t = app.station.find('Home') as Target; app.select(t);
    expect(sectionTitles()).toContain('Target'); expect(body().textContent).toContain('Recorded joints'); expect(body().textContent).toContain('yes');
    const cb = props.el.querySelector<HTMLInputElement>('.field-checkbox input')!; change(cb, false); expect(t.isJointTarget).toBe(false); app.select(t); change(props.el.querySelector<HTMLInputElement>('.field-checkbox input')!, true); expect(t.isJointTarget).toBe(true);
    const r = app.activeRobot!; r.setJoints([40, -80, 90, -100, -90, 10]);
    buttonByText(body(), 'MoveJ here').click(); expect(r.joints().map(Math.round)).toEqual([0, -100, 110, -100, -90, 0]);
    r.setJoints([40, -80, 90, -100, -90, 10]); buttonByText(body(), 'MoveL here').click(); expect(r.joints().map(Math.round)).toEqual([0, -100, 110, -100, -90, 0]);
    r.setJoints([40, -80, 90, -100, -90, 10]); buttonByText(body(), 'Teach').click(); expect(t.joints!.map(Math.round)).toEqual([40, -80, 90, -100, -90, 10]);
    const tool = r.tools()[0]; app.select(tool); expect(sectionTitles()).toContain('Tool'); expect(body().textContent).toContain('vacuum');
    const kindSel = [...props.el.querySelectorAll<HTMLSelectElement>('select')].find((s) => [...s.options].some((o) => o.value === 'welding'))!; change(kindSel, 'welding'); expect(tool.toolKind).toBe('welding');
    r.setTool(null); app.select(tool); buttonByText(body(), 'Set as active tool').click(); expect(r.activeTool()).toBe(tool);
    const p = app.activeProgram!; app.select(p);
    expect(sectionTitles()).toEqual(['Program']); expect(body().textContent).toContain('Instructions');
    buttonByText(body(), 'Validate').click(); expect(body().textContent).toContain('OK'); expect(body().textContent).toMatch(/Cycle time[\d.]+ s/);
    buttonByText(body(), '▶ Run').click(); expect(app.sim.playing).toBe(true); app.stopProgram();
    buttonByText(body(), 'Export…').click(); expect(dialogTitle()).toBe('Export PickPlace'); clickCancel();
    change(props.el.querySelector<HTMLSelectElement>('select')!, ''); expect(p.robotId).toBeNull(); app.select(p); change(props.el.querySelector<HTMLSelectElement>('select')!, r.id); expect(p.robot()).toBe(r);
    p.setRobot(r);
    // instructions of every kind, on a small program so that every edit re-compiles quickly
    const q = app.addProgram(r, 'Kinds'); q.addMoveJ(t); q.setSpeed(500, 90); q.setRounding(5); q.pause(100); q.setDO('DO_1', true); q.event('attach', null); q.runInstruction('MyRoutine'); q.showMessage('hi'); q.navigateTo(100, 200); q.addInstruction({ kind: 'signal', signal: 'spray', value: true, wait: false }); q.addInstruction({ kind: 'call', programId: null } as any);
    app.previewProgram();
    const kinds = new Map(q.instructions().map((i) => [i.data.kind + ((i.data as any).moveType ?? ''), i]));
    const open = (i: Instruction) => { app.select(i); expect(sectionTitles()[0]).toBe(`Instruction: ${i.name}`); return body(); };
    const setSel = (label: string, v: string) => { const f = [...props.el.querySelectorAll<HTMLElement>('label.field')].find((l) => l.querySelector('.field-label')!.textContent === label)!; change(f.querySelector('input,select,textarea')!, v); };
    let b = open(kinds.get('moveMoveJ')!); expect(b.querySelectorAll('label.field').length).toBe(4); setSel('Speed override (mm/s, empty = program)', '250'); expect((kinds.get('moveMoveJ')!.data as any).speed).toBe(250);
    app.select(kinds.get('moveMoveJ')!); setSel('Move type', 'MoveC'); expect((kinds.get('moveMoveJ')!.data as any).moveType).toBe('MoveC'); app.select(kinds.get('moveMoveJ')!); expect(body().querySelectorAll('label.field').length).toBe(5); setSel('Via target', app.station.find('Pick 1')!.id); app.select(kinds.get('moveMoveJ')!); setSel('Target', ''); expect((kinds.get('moveMoveJ')!.data as any).targetId).toBeNull();
    b = open(kinds.get('speed')!); setSel('Linear speed (mm/s)', '700'); expect((kinds.get('speed')!.data as any).speedLinear).toBe(700); app.select(kinds.get('speed')!); setSel('Joint speed (deg/s)', ''); expect((kinds.get('speed')!.data as any).speedJoints).toBeUndefined();
    b = open(kinds.get('pause')!); setSel('Pause (ms, -1 = wait user)', '500'); expect((kinds.get('pause')!.data as any).timeMs).toBe(500);
    b = open(kinds.get('rounding')!); setSel('Rounding radius (mm, -1 = fine)', '2'); expect((kinds.get('rounding')!.data as any).radius).toBe(2);
    b = open(kinds.get('io')!); setSel('IO name', 'DO_9'); app.select(kinds.get('io')!); setSel('Wait for input (instead of set output)', 'true'); expect((kinds.get('io')!.data as any).io).toBe('DO_9'); expect((kinds.get('io')!.data as any).wait).toBe(true);
    b = open(kinds.get('code')!); setSel('Code / program name', 'Other'); app.select(kinds.get('code')!); setSel('Call as function', 'false'); expect((kinds.get('code')!.data as any).code).toBe('Other');
    b = open(kinds.get('print')!); setSel('Message', 'msg2'); app.select(kinds.get('print')!); setSel('Comment only', 'true'); expect((kinds.get('print')!.data as any).isComment).toBe(true);
    b = open(kinds.get('event')!); setSel('Action', 'gripper_close'); app.select(kinds.get('event')!); setSel('Object', ''); expect((kinds.get('event')!.data as any).action).toBe('gripper_close');
    b = open(kinds.get('mobile_move')!); setSel('X (mm)', '5'); app.select(kinds.get('mobile_move')!); setSel('Y (mm)', '6'); app.select(kinds.get('mobile_move')!); setSel('Speed (mm/s)', ''); expect((kinds.get('mobile_move')!.data as any).x).toBe(5);
    b = open(kinds.get('signal')!); setSel('Signal', 'pump'); app.select(kinds.get('signal')!); setSel('Value', '12'); expect((kinds.get('signal')!.data as any).value).toBe(12); app.select(kinds.get('signal')!); setSel('Value', 'false'); expect((kinds.get('signal')!.data as any).value).toBe(false); app.select(kinds.get('signal')!); setSel('Value', 'txt'); expect((kinds.get('signal')!.data as any).value).toBe('txt');
    b = open(kinds.get('call')!); expect(b.querySelector('pre.code-preview')).not.toBeNull(); setSel('Enabled', 'false'); expect(kinds.get('call')!.enabled).toBe(false);
  });
  it('mobile robot, field, crop row, mission, fleet, map and zone editors', async () => {
    app.setStation(demos.find((d) => d.id === 'orchard')!.build());
    const m = app.station.itemsOfType<MobileRobot>(ItemType.MOBILE_ROBOT)[0]; app.select(m);
    expect(sectionTitles()).toEqual(['Pose relative to Station (mm / deg)', 'State', 'Kinematics', 'Battery & capabilities']); expect(body().textContent).toContain('Navigation & localization');
    const fieldIn = (label: string) => [...props.el.querySelectorAll<HTMLElement>('label.field')].find((l) => l.querySelector('.field-label')!.textContent === label)!.querySelector('input,select')!;
    change(fieldIn('Drive'), 'omni'); expect(m.kin.drive).toBe('omni'); app.select(m);
    change(fieldIn('Max speed (mm/s)'), '900'); expect(m.kin.maxSpeed).toBe(900); app.select(m);
    change(fieldIn('Capacity (Wh)'), '3000'); expect(m.battery.capacityWh).toBe(3000); app.select(m);
    change(fieldIn('Capabilities (comma separated)'), 'spray, scout'); expect(m.capabilities).toEqual(['spray', 'scout']); app.select(m);
    change(fieldIn('ROS 2 namespace'), 'r1'); expect(m.rosNamespace).toBe('r1'); app.select(m);
    m.state.v = 500; m.notify('state'); await tickMs(60); expect(body().textContent).toContain('0.50 m/s');
    const field = app.station.itemsOfType(ItemType.FIELD)[0] as any; app.select(field); expect(sectionTitles()).toContain('Field'); expect(body().textContent).toContain('apple'); expect(body().textContent).toContain('46.870000');
    app.select(field.rows()[0]); expect(sectionTitles()).toEqual(['Crop row']); expect(body().textContent).toContain('Plants');
    const mi = app.station.itemsOfType<MissionItem>(ItemType.MISSION)[1]; app.select(mi); expect(sectionTitles()).toContain('Mission'); expect(body().textContent).toContain('spray');
    buttonByText(body(), 'Plan').click(); expect(lastToast()).toMatch(/tasks over \d+ rows/); expect(mi.taskIds.length).toBeGreaterThan(0);
    buttonByText(body(), '▶ Run world').click(); expect(app.worldRunning).toBe(true); app.pauseWorld();
    const fleet = app.station.itemsOfType<FleetItem>(ItemType.FLEET)[0]; app.select(fleet); expect(sectionTitles()).toContain('Fleet'); expect(body().textContent).toContain('Tasks done / pending');
    change(fieldIn('Allocation'), 'nearest'); expect(fleet.allocation).toBe('nearest'); app.select(fleet);
    change(fieldIn('Robots per row segment'), '2'); expect(fleet.segmentCapacity).toBe(2); app.select(fleet);
    app.addMobileRobot('scout'); app.select(fleet); buttonByText(body(), 'Add all mobile robots').click(); expect(fleet.robotIds.length).toBe(5);
    app.select(app.station.itemsOfType(ItemType.MAP)[0]); expect(sectionTitles()).toContain('Map'); expect(body().textContent).toMatch(/\d+ × \d+ cells/);
    const z = app.station.itemsOfType(ItemType.ZONE)[0] as any; app.select(z); expect(sectionTitles()).toContain('Zone'); change([...props.el.querySelectorAll<HTMLSelectElement>('select')].find((s) => [...s.options].some((o) => o.value === 'parking'))!, 'parking'); expect(z.kind).toBe('parking');
    app.resetWorld();
  });
  it('component, camera and control-model editors', async () => {
    app.setStation(demos.find((d) => d.id === 'packing')!.build());
    const fieldIn = (label: string) => [...props.el.querySelectorAll<HTMLElement>('label.field')].find((l) => l.querySelector('.field-label')!.textContent === label)!.querySelector('input,select')!;
    const comps = app.station.itemsOfType<Component>(ItemType.COMPONENT);
    const types = new Set<string>();
    for (const c of comps) {
      app.select(c); const b = (c.behaviour as any); types.add(b.type);
      expect(sectionTitles()).toContain('Component'); expect(body().textContent).toContain(b.type);
      if (b.type === 'conveyor') { change(fieldIn('Speed (mm/s)'), '123'); expect(b.speed).toBe(123); }
      if (b.type === 'feeder') { change(fieldIn('Interval (s)'), '2.5'); expect(b.interval).toBe(2.5); }
      if (b.type === 'process') { change(fieldIn('Cycle time (s)'), '7'); expect(b.cycleTime).toBe(7); app.select(c); change(fieldIn('MTBF (s, 0 = none)'), '0'); }
      if (b.type === 'buffer') { change(fieldIn('Capacity'), '9'); expect(b.capacity).toBe(9); }
      app.select(c); change(fieldIn('Enabled'), 'false'); expect(b.enabled).toBe(false); b.enabled = true;
      if ('next' in b) { app.select(c); const other = comps.find((x) => x !== c)!; change(fieldIn('Output to'), other.id); expect(b.next).toBe(other.id); }
    }
    expect([...types].sort()).toEqual(['buffer', 'conveyor', 'feeder', 'process', 'sink']);
    const cam = addCamera(app); app.select(cam);
    expect(sectionTitles()).toContain('Camera / sensor'); expect(body().textContent).toContain('Machine vision'); expect(body().textContent).toContain('not configured');
    change(fieldIn('Kind'), 'lidar2d'); expect(cam.kind).toBe('lidar2d'); app.select(cam); change(fieldIn('Field of view (deg)'), '120'); expect(cam.fov).toBe(120);
    const m = app.cmd(() => addControlModel(app.station, 'petri', 'Net', TEMPLATES.petri)); app.select(m);
    expect(sectionTitles()).toEqual(['Control model']); expect(body().textContent).toContain('Petri net'); expect(body().textContent).toContain('—');
    const n = downloads.length; buttonByText(body(), 'Report (Markdown)').click(); expect(downloads.at(-1)!.name).toBe('Net.md'); expect(m.lastOk).not.toBeNull();
    buttonByText(body(), 'Analyse').click(); expect(bottom.el.querySelector('.tab.active')!.textContent).toBe('Control'); bottom.show('program');
    buttonByText(body(), 'Open in Control tab').click(); expect(bottom.el.querySelector('.tab.active')!.textContent).toBe('Control'); bottom.show('program');
    expect(downloads.length).toBe(n + 1);
  });
});

describe('ProgramEditor', () => {
  let ed: ProgramEditor;
  const rows = () => [...ed.el.querySelectorAll<HTMLElement>('.ins-row')];
  beforeAll(() => { ed = new ProgramEditor(app); document.body.appendChild(ed.el); });
  it('adds every instruction kind, reorders, deletes and follows the selection', () => {
    app.newStation();
    expect(ed.el.textContent).toContain('Create a program');
    const r = app.addRobotFromLibrary('UR5e');
    buttonByText(ed.el, '+ New').click(); const p = app.activeProgram!; expect(p.robot()).toBe(r);
    const head = ed.el.querySelector<HTMLElement>('.pe-head')!;
    buttonByText(head, 'MoveJ').click(); r.setJoints([20, -80, 90, -100, -90, 0]); buttonByText(head, 'MoveL').click();
    expect(p.instructions().length).toBe(2); expect(app.station.itemsOfType(ItemType.TARGET).length).toBe(2); ed.render(); expect(rows().length).toBe(2); expect(rows()[0].querySelector('.ins-kind')!.textContent).toBe('MoveJ'); expect(rows()[0].querySelector('.ins-name')!.textContent).toBe('Target 1');
    for (const label of ['MoveC', 'Speed', 'Round', 'Frame', 'Tool', 'Pause', 'Set DO', 'Wait DI', 'Grip', 'Release', 'Code', 'Comment', 'Call', 'Navigate', 'Signal', 'Thread', 'Wait']) buttonByText(head, label).click();
    expect(p.instructions().length).toBe(19); ed.render();
    expect(rows().map((x) => x.querySelector('.ins-kind')!.textContent)).toEqual(['MoveJ', 'MoveL', 'MoveC', 'Speed', 'Round', 'Frame', 'Tool', 'Pause', 'SetDO', 'WaitDI', 'gripper close', 'gripper open', 'Call', '//', 'Call', 'Navigate', 'SetSig', 'Thread', 'Wait']);
    expect(rows().some((x) => x.querySelector('.ins-time'))).toBe(true); expect(ed.el.querySelector('.pe-foot')!.textContent).toMatch(/^Cycle time [\d.]+ s/);
    // insert after the selected instruction
    ed.render(); rows()[0].click(); expect(app.station.selection[0]).toBe(p.instructions()[0]);
    buttonByText(head, 'Pause').click(); expect(p.instructions()[1].data.kind).toBe('pause');
    buttonByText(head, 'MoveJ').click(); expect(p.instructions()[1].data.kind).toBe('move'); expect(p.instructions()[2].data.kind).toBe('pause');
    // move / delete buttons
    const first = p.instructions()[0]; ed.render();
    const second = p.instructions()[1], third = p.instructions()[2];
    rows()[0].querySelector<HTMLButtonElement>('button[title="Move down"]')!.click(); expect(p.instructions().slice(0, 3)).toEqual([second, first, third]); ed.render();
    rows()[1].querySelector<HTMLButtonElement>('button[title="Move down"]')!.click(); expect(p.instructions().slice(0, 3)).toEqual([second, third, first]); ed.render();
    rows()[2].querySelector<HTMLButtonElement>('button[title="Move up"]')!.click(); expect(p.instructions().slice(0, 3)).toEqual([second, first, third]); ed.render();
    rows()[1].querySelector<HTMLButtonElement>('button[title="Move up"]')!.click(); expect(p.instructions()[0]).toBe(first); ed.render();
    rows()[0].querySelector<HTMLButtonElement>('button[title="Move up"]')!.click(); expect(p.instructions()[0]).toBe(first); ed.render();
    const n = p.instructions().length; rows()[1].querySelector<HTMLButtonElement>('button[title="Delete"]')!.click(); expect(p.instructions().length).toBe(n - 1);
    // drag & drop reorder
    ed.render(); const src = p.instructions()[3]; const dst = p.instructions()[0];
    const drop = Object.assign(new Event('drop', { bubbles: true, cancelable: true }), { dataTransfer: { getData: () => src.id } });
    rows()[3].dispatchEvent(new Event('dragstart', { bubbles: true })); const over = new Event('dragover', { bubbles: true, cancelable: true }); rows()[0].dispatchEvent(over); expect(rows()[0].classList.contains('drop')).toBe(true); rows()[0].dispatchEvent(new Event('dragleave', { bubbles: true })); rows()[0].dispatchEvent(drop);
    expect(p.instructions()[0]).toBe(src); expect(p.instructions()[1]).toBe(dst);
  });
  it('double-click, context menu, running highlight and the program selector', async () => {
    const p = app.activeProgram!; const r = app.activeRobot!;
    const target = app.station.find('Target 1') as Target; r.setJoints([50, -60, 60, -90, -90, 0]);
    ed.render(); const moveRow = rows().find((x) => x.querySelector('.ins-name')!.textContent === 'Target 1')!;
    moveRow.dispatchEvent(mouse('dblclick')); expect(r.joints().map(Math.round)).toEqual(target.joints!.map(Math.round));
    const speedRow = rows().find((x) => x.querySelector('.ins-kind')!.textContent === 'Speed')!; speedRow.dispatchEvent(mouse('dblclick')); // seeks to the step end
    speedRow.dispatchEvent(mouse('contextmenu'));
    expect(ctxItems().map((e) => e.textContent)).toEqual(['Run from here', 'Disable', 'Insert MoveJ (teach) after', 'Insert MoveL (teach) after', 'Delete']);
    const speedIns = p.instructions().find((i) => i.data.kind === 'speed')!;
    ctxItem('Disable').click(); expect(speedIns.enabled).toBe(false); ed.render();
    rows().find((x) => x.querySelector('.ins-kind')!.textContent === 'Speed')!.dispatchEvent(mouse('contextmenu')); expect(ctxItem('Enable')).toBeTruthy(); ctxItem('Enable').click(); expect(speedIns.enabled).toBe(true); ed.render();
    const n = p.instructions().length; const idx = p.instructions().indexOf(speedIns);
    rows()[idx].dispatchEvent(mouse('contextmenu')); ctxItem('Insert MoveL (teach) after').click(); expect(p.instructions().length).toBe(n + 1); expect((p.instructions()[idx + 1].data as any).moveType).toBe('MoveL'); ed.render();
    rows()[idx].dispatchEvent(mouse('contextmenu')); ctxItem('Insert MoveJ (teach) after').click(); expect((p.instructions()[idx + 1].data as any).moveType).toBe('MoveJ'); ed.render();
    rows()[idx].dispatchEvent(mouse('contextmenu')); ctxItem('Run from here').click(); expect(app.sim.playing).toBe(true); app.stopProgram(); ed.render();
    rows()[idx + 1].dispatchEvent(mouse('contextmenu')); ctxItem('Delete').click(); expect(p.instructions().length).toBe(n + 1); ed.render();
    app.runProgram(); ed.render(); (app as any).tick(0.05); expect(rows().some((x) => x.classList.contains('running'))).toBe(true); app.seekProgram(app.sim.duration - 0.01); app.stopProgram();
    const p2 = app.addProgram(null, 'Second'); ed.render();
    const sel = ed.el.querySelector<HTMLSelectElement>('select.prog-select')!; expect(sel.options.length).toBe(3); expect(sel.value).toBe(p2.id);
    change(sel, p.id); expect(app.activeProgram).toBe(p); expect(app.station.selection[0]).toBe(p);
    change(sel, ''); expect(app.activeProgram).toBeNull(); expect(ed.el.textContent).toContain('Create a program');
    change(sel, p2.id); app.deleteItems([app.activeRobot!]); app.setActiveRobot(null); buttonByText(ed.el.querySelector<HTMLElement>('.pe-head')!, 'MoveJ').click(); expect(app.logs.at(-1)!.text).toBe('Program has no robot');
    buttonByText(ed.el.querySelector<HTMLElement>('.pe-head')!, 'Validate').click(); buttonByText(ed.el.querySelector<HTMLElement>('.pe-head')!, 'Export…').click(); expect(dialogTitle()).toBe('Export Second'); clickCancel();
    buttonByText(ed.el.querySelector<HTMLElement>('.pe-head')!, '▶ Run').click(); app.stopProgram();
  });
});

export { Frame, Robot, Program, Tool };
