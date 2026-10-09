// @vitest-environment jsdom
import { describe, it, expect, beforeAll, afterEach, vi } from 'vitest';
import { App, ItemType, Frame, Target, Program, Robot, MobileRobot, FleetItem, MissionItem, SceneObject, Folder, Camera, Component, MapItem, ZoneItem, FieldItem } from '../src/app';
import { Tool } from '../src/core/items/item';
import { demos } from '../src/demos';
import { transl, mul, rotx, DEG } from '../src/core/math/pose';
import { createRobotFromLibrary } from '../src/core/items/library';
import { parseGcode, gcodeToMachiningCurves } from '../src/io/programs/gcode';
import { itemContextMenu, robotLibraryDialog, onlineLibraryDialog, vdaDialog, mobileRobotDialog, orchardDialog, fleetDialog, missionDialog, runMissionPlan, mapDialog, zoneDialog, componentDialog, navigateDialog, harvestArmDialog, machiningDialog, curveFollowDialog, railIKDialog, exportDialog, importDialog, robotParametersDialog } from '../src/ui/dialogs';
import { navStackSection, navStackDialog, exportNavPackage, buildNavPanel } from '../src/ui/navstack_ui';
import { getNavStack, setNavStack, defaultNavStack } from '../src/mobile/navstack';
import { camerasOf, addCamera, visionSection, visionStackDialog, exportVisionPackage, importPointCloud, buildVisionPanel } from '../src/ui/vision_ui';
import { getVisionStack } from '../src/vision/stack';
import { scenariosDialog } from '../src/ui/scenarios_ui';
import { ALL_SCENARIOS } from '../src/scenarios';
import { contextMenu } from '../src/ui/dom';
import { installDomStubs, makeApp, downloads, rendererCalls, tickMs, waitFor, toasts, lastToast, clearToasts, ctxItem, ctxItems, closeMenus, currentDialog, dialogs, dialogTitle, setField, fieldInput, dialogFields, clickOk, clickCancel, dialogButton, closeAllDialogs, resolvePickFiles, textFile, change, buttonByText } from './ui_harness';

let app: App; let shown: string[] = [];
const pickplace = () => app.setStation(demos.find((d) => d.id === 'pickplace')!.build());
const orchard = () => app.setStation(demos.find((d) => d.id === 'orchard')!.build());
const rowLabel = (e: HTMLElement) => [...e.childNodes].filter((n) => n.nodeType === Node.TEXT_NODE).map((n) => n.textContent).join('').replace(/^✓ /, '');
const menuFor = (item: any) => { closeMenus(); contextMenu(0, 0, itemContextMenu(app, item)); return ctxItems().map(rowLabel); };
const submenu = (label: string) => { ctxItem(label).dispatchEvent(new Event('pointerenter')); return [...document.querySelectorAll<HTMLElement>('.ctx-menu .ctx-menu .ctx-item')]; };
const subItem = (label: string) => { const it = submenu('Add').find((e) => e.textContent!.startsWith(label)); if (!it) throw new Error(`no sub item ${label}`); return it; };

beforeAll(() => { installDomStubs(); app = makeApp('pickplace'); (app as any).bottom = { show: (id: string) => shown.push(id) }; });
afterEach(() => { closeAllDialogs(); clearToasts(); closeMenus(); shown = []; });

describe('itemContextMenu', () => {
  it('offers the Add submenu on every item and runs its entries', async () => {
    pickplace();
    const table = app.station.find('Table') as Frame;
    expect(menuFor(table)).toEqual(['Add', 'Set as active reference', 'Focus', 'Hide', 'Rename…', 'Delete']);
    expect(submenu('Add').length).toBe(18);
    const count = (t: ItemType) => app.station.itemsOfType(t).length;
    subItem('Reference frame').click(); expect(app.station.selection[0].parent).toBe(table);
    menuFor(app.activeRobot); subItem('Reference frame').click(); expect(app.station.selection[0].parent).toBe(app.station); // robots host no frames
    menuFor(table); subItem('Target (at robot TCP)').click(); expect(app.station.selection[0].parent).toBe(table);
    const p0 = count(ItemType.PROGRAM); menuFor(table); subItem('Program').click(); expect(count(ItemType.PROGRAM)).toBe(p0 + 1);
    menuFor(table); subItem('Robot from library').click(); expect(dialogTitle()).toBe('Add robot from library'); clickCancel(); await tickMs(1);
    menuFor(table); subItem('Robot from online library').click(); expect(dialogTitle()).toBe('Add robot from online library'); clickCancel(); await tickMs(1);
    menuFor(table); subItem('Mobile robot').click(); setField('Type', 'tractor'); clickOk(); await tickMs(1); expect(count(ItemType.MOBILE_ROBOT)).toBe(1);
    const r = app.activeRobot!; const nt = r.tools().length; menuFor(table); subItem('Tool (on active robot)').click(); expect(r.tools().length).toBe(nt + 1); expect(r.activeTool()!.name).toBe(`Tool ${nt + 1}`);
    app.setActiveRobot(null); menuFor(table); subItem('Tool (on active robot)').click(); expect(lastToast()).toBe('Select a robot'); app.setActiveRobot(r);
    const o0 = count(ItemType.OBJECT); menuFor(table); subItem('Box').click(); menuFor(table); subItem('Cylinder').click(); menuFor(table); subItem('Sphere').click(); expect(count(ItemType.OBJECT)).toBe(o0 + 3);
    menuFor(table); subItem('Folder').click(); expect(table.children.some((c) => c instanceof Folder)).toBe(true);
    menuFor(table); subItem('Camera (on selected item)').click(); expect(app.station.selection[0]).toBeInstanceOf(Camera); expect(app.station.selection[0].parent).toBe(table);
    menuFor(table); subItem('Field / orchard').click(); expect(dialogTitle()).toBe('Create field / orchard'); clickCancel(); await tickMs(1);
    menuFor(table); subItem('Fleet').click(); expect(dialogTitle()).toBe('Create fleet'); clickCancel(); await tickMs(1);
    menuFor(table); subItem('Mission').click(); expect(lastToast()).toBe('Create a field first (Add > Field / orchard)');
    menuFor(table); subItem('Map (occupancy grid)').click(); expect(dialogTitle()).toBe('Create occupancy map'); clickCancel(); await tickMs(1);
    menuFor(table); subItem('Zone (charging / no-go)').click(); expect(dialogTitle()).toBe('Create zone'); clickCancel(); await tickMs(1);
    menuFor(table); subItem('Process component').click(); expect(dialogTitle()).toBe('Add process component'); clickCancel(); await tickMs(1);
  });
  it('target, robot, frame, tool and program entries', async () => {
    pickplace();
    const r = app.activeRobot!; const home = app.station.find('Home') as Target; const pick = app.station.find('Pick 2') as Target;
    expect(menuFor(home)).toEqual(['Add', 'Move robot here (MoveJ)', 'Move robot here (MoveL)', 'Teach current position', 'Set as Cartesian target', 'Add MoveJ to program', 'Add MoveL to program', 'Focus', 'Hide', 'Rename…', 'Delete']);
    r.setJoints([30, -80, 90, -100, -90, 0]); ctxItem('Move robot here (MoveJ)').click(); expect(r.joints().map(Math.round)).toEqual([0, -100, 110, -100, -90, 0]);
    r.setJoints([30, -80, 90, -100, -90, 0]); menuFor(home); ctxItem('Move robot here (MoveL)').click(); expect(r.joints().map(Math.round)).toEqual([0, -100, 110, -100, -90, 0]);
    r.setJoints([30, -80, 90, -100, -90, 0]); menuFor(pick); ctxItem('Teach current position').click(); expect(pick.joints!.map(Math.round)).toEqual([30, -80, 90, -100, -90, 0]);
    menuFor(home); ctxItem('Set as Cartesian target').click(); expect(home.isJointTarget).toBe(false); menuFor(home); ctxItem('Set as Joint target').click(); expect(home.isJointTarget).toBe(true);
    app.setActiveProgram(null); const np = app.station.itemsOfType(ItemType.PROGRAM).length;
    menuFor(home); ctxItem('Add MoveJ to program').click(); expect(app.station.itemsOfType(ItemType.PROGRAM).length).toBe(np + 1); expect(app.activeProgram!.instructions().length).toBe(1);
    menuFor(home); ctxItem('Add MoveL to program').click(); expect(app.activeProgram!.instructions().length).toBe(2); expect((app.activeProgram!.instructions()[1].data as any).moveType).toBe('MoveL');
    expect(menuFor(r)).toEqual(['Add', 'Set as active robot', 'Home', 'Generate picking program for nearby fruit…', 'Follow curve / points of an object…', 'Move with external axes (rail/gantry)…', 'Focus', 'Hide', 'Rename…', 'Delete']);
    app.setActiveRobot(null); ctxItem('Set as active robot').click(); expect(app.activeRobot).toBe(r);
    r.setJoints([30, -80, 90, -100, -90, 0]); menuFor(r); ctxItem('Home').click(); expect(r.joints()).toEqual(r.jointsHome());
    menuFor(r); ctxItem('Generate picking program').click(); expect(lastToast()).toBe('No crop rows in the station');
    menuFor(r); ctxItem('Follow curve').click(); expect(lastToast()).toMatch(/^No object with curves/);
    menuFor(r); ctxItem('Move with external axes').click(); await waitFor(() => /not mounted/.test(lastToast()));
    const table = app.station.find('Table') as Frame; r.setFrame(null); menuFor(table); ctxItem('Set as active reference').click(); expect(r.activeFrame()).toBe(table);
    const tool = r.tools()[0]; r.setTool(null); expect(menuFor(tool)).toContain('Set as active tool'); ctxItem('Set as active tool').click(); expect(r.activeToolId).toBe(tool.id);
    const prog = app.station.find('PickPlace') as Program;
    expect(menuFor(prog)).toEqual(['Add', 'Run', 'Validate', 'Export (post processor)…', 'Focus', 'Hide', 'Rename…', 'Delete']);
    ctxItem('Run').click(); expect(app.sim.playing).toBe(true); app.stopProgram();
    menuFor(prog); ctxItem('Validate').click(); expect(lastToast()).toMatch(/^OK: [\d.]+ s, [\d.]+ m$/);
    menuFor(prog); ctxItem('Export (post processor)').click(); expect(dialogTitle()).toBe('Export PickPlace'); clickCancel();
  });
  it('field, mission, mobile robot and fleet entries, plus focus / hide / rename / delete', async () => {
    orchard();
    const field = app.station.itemsOfType<FieldItem>(ItemType.FIELD)[0];
    expect(menuFor(field)).toContain('Regenerate rows…'); ctxItem('Regenerate rows').click(); expect(dialogTitle()).toBe(`Regenerate ${field.name}`); clickCancel(); await tickMs(1);
    const nm = app.station.itemsOfType(ItemType.MAP).length, nz = app.station.itemsOfType(ItemType.ZONE).length;
    menuFor(field); ctxItem('Build navigation map').click(); expect(app.station.itemsOfType(ItemType.MAP).length).toBe(nm + 1); expect(app.station.itemsOfType(ItemType.ZONE).length).toBeGreaterThan(nz);
    const mission = app.station.itemsOfType<MissionItem>(ItemType.MISSION)[1]; expect(mission.taskIds.length).toBe(0);
    expect(menuFor(mission)).toContain('Plan mission (create fleet tasks)'); ctxItem('Plan mission').click(); expect(mission.taskIds.length).toBeGreaterThan(0); expect(lastToast()).toMatch(/tasks over/);
    const m = app.station.itemsOfType<MobileRobot>(ItemType.MOBILE_ROBOT)[0];
    m.setPose2D(123, 456, 7); expect(menuFor(m)).toContain('Set home here'); ctxItem('Set home here').click(); expect(m.home).toEqual({ x: 123, y: 456, theta: 7 });
    menuFor(m); ctxItem('Navigate to point').click(); expect(dialogTitle()).toBe(`Navigate ${m.name}`); clickCancel(); await tickMs(1);
    const fleet = app.station.itemsOfType<FleetItem>(ItemType.FLEET)[0]; const extra = app.addMobileRobot('scout');
    expect(menuFor(fleet)).toContain('Add all mobile robots to fleet'); ctxItem('Add all mobile robots to fleet').click(); expect(fleet.robotIds).toContain(extra.id);
    const n = rendererCalls.length; menuFor(m); ctxItem('Focus').click(); expect(rendererCalls.slice(n)).toContain('focusItem');
    menuFor(m); ctxItem('Hide').click(); expect(m.visible).toBe(false); expect(menuFor(m)).toContain('Show'); ctxItem('Show').click(); expect(m.visible).toBe(true);
    menuFor(m); ctxItem('Rename').click(); expect(dialogTitle()).toBe('Rename'); setField('Name', 'Platform X'); clickOk(); await tickMs(1); expect(m.name).toBe('Platform X');
    menuFor(m); ctxItem('Rename').click(); setField('Name', ''); clickOk(); await tickMs(1); expect(m.name).toBe('Platform X');
    app.select(m); app.select(extra, true); expect(menuFor(m)).toContain('Delete (2)'); ctxItem('Delete (2)').click(); expect(app.station.findById(m.id)).toBeNull(); expect(app.station.findById(extra.id)).toBeNull();
    expect(menuFor(app.station)).not.toContain('Delete');
  });
});

describe('creation dialogs', () => {
  it('robot library and online library', async () => {
    pickplace();
    let p = robotLibraryDialog(app); setField('Robot', 'ABB_IRB120'); clickOk(); await p;
    expect(app.activeRobot!.name).toMatch(/IRB 120/); expect(app.station.itemsOfType(ItemType.ROBOT).length).toBe(2);
    p = robotLibraryDialog(app); clickCancel(); await p; expect(app.station.itemsOfType(ItemType.ROBOT).length).toBe(2);
    p = onlineLibraryDialog(app); expect(fieldInput('Download 3D meshes')).toBeTruthy(); expect(currentDialog().textContent).toMatch(/\d+ robots with exact kinematics/); clickCancel(); await p;
    const fetchSpy = vi.spyOn(globalThis, 'fetch').mockRejectedValue(new Error('offline'));
    p = onlineLibraryDialog(app); clickOk(); await p;
    expect(toasts().some((x) => x.startsWith('Downloading '))).toBe(true); expect(toasts().some((x) => x.startsWith('Download failed: '))).toBe(true); expect(app.logs.at(-1)!.level).toBe('error');
    fetchSpy.mockRestore();
  });
  it('VDA 5050 needs the studio server; the form connects through it', async () => {
    await vdaDialog(app); expect(lastToast()).toMatch(/^Start the studio server/);
    window.history.pushState({}, '', '/?server=ws://h:1');
    const fetchSpy = vi.spyOn(globalThis, 'fetch').mockRejectedValue(new Error('no server'));
    try {
      let p = vdaDialog(app); expect(dialogTitle()).toBe('VDA 5050 fleet interface (AGV / AMR)'); expect(dialogFields().length).toBe(9); expect(currentDialog().textContent).toContain('not connected'); clickCancel(); await p;
      p = vdaDialog(app); setField('Role', 'both'); setField('Username (optional)', 'u'); clickOk(); await p;
      expect((app as any).vda).toBeTruthy(); expect(lastToast()).toMatch(/^VDA 5050: /);
      p = vdaDialog(app); expect(dialogButton('Reconnect')).toBeTruthy(); clickCancel(); await p;
    } finally { fetchSpy.mockRestore(); window.history.pushState({}, '', '/'); (app as any).vda = null; }
  });
  it('mobile robot, orchard (create + regenerate), fleet, mission and plan', async () => {
    app.newStation();
    let p = mobileRobotDialog(app); setField('Type', 'sprayer'); setField('Name', 'S1'); clickOk(); await p;
    expect(app.station.itemsOfType<MobileRobot>(ItemType.MOBILE_ROBOT)[0].name).toBe('S1');
    p = mobileRobotDialog(app); clickCancel(); await p;
    p = orchardDialog(app); setField('Name', 'Block'); setField('Crop', 'pear'); setField('Field width (m)', 30); setField('Field length (m)', 20); setField('Headland (m)', 2); setField('Greenhouse (indoor)', true); clickOk(); await p;
    const field = app.station.itemsOfType<FieldItem>(ItemType.FIELD)[0]; expect(field.name).toBe('Block'); expect(field.crop.crop).toBe('pear'); expect(field.indoor).toBe(true); expect(field.rows().length).toBeGreaterThan(1);
    expect(app.station.itemsOfType(ItemType.MAP).length).toBe(1); expect(app.station.itemsOfType(ItemType.ZONE).length).toBeGreaterThan(0);
    const rows0 = field.rows().length;
    p = orchardDialog(app, field); expect(dialogTitle()).toBe('Regenerate Block'); expect(Number(fieldInput('Field width (m)').value)).toBe(30); setField('Row spacing (m)', 5); clickOk(); await p;
    expect(field.rows().length).toBeLessThan(rows0); expect(app.station.itemsOfType(ItemType.MAP).length).toBe(1);
    p = orchardDialog(app); clickCancel(); await p;
    await missionDialog(app.newStation() as any ?? app); // no field
    expect(lastToast()).toBe('Create a field first (Add > Field / orchard)');
    app.station.addChild(field); // reuse the field in the fresh station
    p = fleetDialog(app); setField('Fleet name', 'F1'); setField('Number of robots', 2); setField('Robot type', 'scout'); setField('Allocation', 'nearest'); clickOk(); await p;
    const fleet = app.station.itemsOfType<FleetItem>(ItemType.FLEET)[0]; expect(fleet.name).toBe('F1'); expect(fleet.allocation).toBe('nearest'); expect(fleet.robotIds.length).toBe(2); expect(fleet.chargingZoneIds.length).toBe(1);
    const robots = app.station.itemsOfType<MobileRobot>(ItemType.MOBILE_ROBOT); expect(robots.map((r) => r.name)).toEqual(['Scout rover 1', 'Scout rover 2']); expect(robots[0].home).toBeTruthy();
    p = fleetDialog(app); setField('Number of robots', 0); setField('Add charging zone at start', false); clickOk(); await p; expect(app.station.itemsOfType(ItemType.FLEET).length).toBe(2);
    p = missionDialog(app); expect(dialogTitle()).toBe('Create mission'); setField('Name', 'Scout it'); setField('Type', 'scout'); setField('Rows (e.g. 1-5,8; empty = all)', '1-2, 9-x, 4'); setField('Work speed (m/s)', 1); clickOk(); await p;
    const mi = app.station.itemsOfType<MissionItem>(ItemType.MISSION)[0]; expect(mi.name).toBe('Scout it'); expect(mi.rowIds.length).toBe(3); expect(mi.fleetId).toBe(fleet.id); expect(mi.settings.workSpeed).toBe(1000); expect(mi.taskIds.length).toBeGreaterThan(0); expect(app.station.selection[0]).toBe(mi);
    p = missionDialog(app); clickCancel(); await p;
    const orphan = app.station.addChild(new MissionItem('No fleet')); orphan.fleetId = 'missing'; runMissionPlan(app, orphan); expect(lastToast()).toBe('Mission has no fleet');
    orphan.fleetId = null; orphan.fieldId = 'missing'; runMissionPlan(app, orphan); expect(lastToast()).toBe('Mission has no field');
  });
  it('map, zone, component and navigate dialogs', async () => {
    pickplace();
    let p = mapDialog(app); setField('Width (m)', 8); setField('Height (m)', 6); setField('Resolution (m/cell)', 0.5); clickOk(); await p;
    const map = app.station.itemsOfType<MapItem>(ItemType.MAP)[0]; expect(map.width).toBe(16); expect(map.height).toBe(12); expect(map.resolution).toBe(500); expect(map.isFree(800, -150)).toBe(false); expect(map.isFree(-3500, 2500)).toBe(true); // the boxes' footprints are rasterised
    p = mapDialog(app); setField('Rasterise objects footprints', false); clickOk(); await p; expect(app.station.itemsOfType(ItemType.MAP).length).toBe(2);
    p = zoneDialog(app); setField('Name', 'Dock'); setField('Kind', 'gnss_denied'); setField('Width (m)', 2); setField('Length (m)', 3); clickOk(); await p;
    const z = app.station.itemsOfType<ZoneItem>(ItemType.ZONE)[0]; expect(z.name).toBe('Dock'); expect(z.kind).toBe('gnss_denied'); expect(z.polygon.length).toBe(4);
    p = zoneDialog(app); clickCancel(); await p;
    const c0 = app.station.itemsOfType(ItemType.COMPONENT).length;
    for (const [type, name] of [['feeder', 'F'], ['conveyor', 'C'], ['process', 'P'], ['buffer', 'B'], ['sink', '']] as const) { p = componentDialog(app); setField('Type', type); setField('Name', name); if (type === 'sink') setField('Connect output to', app.station.itemsOfType(ItemType.COMPONENT).at(-1)!.id); clickOk(); await p; }
    const comps = app.station.itemsOfType<Component>(ItemType.COMPONENT).slice(c0);
    expect(comps.map((c) => c.behaviour.type)).toEqual(['feeder', 'conveyor', 'process', 'buffer', 'sink']); expect(comps[4].name).toBe('sink'); expect((comps[4].behaviour as any).next).toBe(comps[3].id); expect(comps[1].pose()[14]).toBe(800);
    p = componentDialog(app); clickCancel(); await p;
    const m = app.addMobileRobot('amr');
    p = navigateDialog(app, m); expect(Number(fieldInput('X (m)').value)).toBeCloseTo(m.state.x / 1000 + 10, 1); setField('X (m)', 2); setField('Y (m)', -3); clickOk(); await p;
    expect(m.state.path).toBeTruthy(); expect(app.worldRunning).toBe(true); app.pauseWorld();
    app.deleteItems(app.station.itemsOfType(ItemType.MAP)); p = navigateDialog(app, m); setField('X (m)', 1); clickOk(); await p; expect(m.state.path!.length).toBe(2); app.pauseWorld();
    p = navigateDialog(app, m); clickCancel(); await p;
  });
  it('harvest, machining, curve follow and rail IK generators', async () => {
    pickplace();
    const r = app.activeRobot!;
    await harvestArmDialog(app, r); expect(lastToast()).toBe('No crop rows in the station');
    orchard();
    const arm = app.station.find('Arm 1') as Robot; const platform = arm.parent as MobileRobot;
    let p = harvestArmDialog(app, arm); expect(dialogTitle()).toBe('Generate fruit picking program'); clickOk(); await p; expect(lastToast()).toMatch(/^No rows within reach/);
    const field = app.station.itemsOfType<FieldItem>(ItemType.FIELD)[0]; const row = field.rows()[0]; const fp = field.poseAbs(); const a = row.pointAt(row.length() / 2);
    platform.setPose2D(fp[12] + a[0], fp[13] + a[1] - 1200, 0);
    p = harvestArmDialog(app, arm); setField('Max fruit', 3); setField('Approach', 'radial'); clickOk(); await p;
    expect(lastToast()).toMatch(/^Program with \d+ picks/); expect(app.activeProgram!.name).toMatch(/harvest/i);
    p = harvestArmDialog(app, arm); clickCancel(); await p;
    pickplace();
    const r2 = app.activeRobot!;
    await machiningDialog(app, r2); expect(lastToast()).toMatch(/^Import an NC/);
    await curveFollowDialog(app, r2); expect(lastToast()).toMatch(/^No object with curves/);
    const g = parseGcode('G21\nG0 X0 Y0 Z5\nG1 X20 Y0 Z0 F300\nG1 X20 Y20\nG1 X0 Y20\nG0 Z5\n');
    const part = app.station.addChild(new SceneObject('part')); part.curves = gcodeToMachiningCurves(g); part.setPose(mul(transl(500, -300, 700), rotx(180 * DEG)));
    p = machiningDialog(app, r2); expect(dialogTitle()).toBe('Robot machining project (NC / G-code)'); expect(fieldInput('Part / NC program object').value).toBe(part.id); setField('Extruder digital output (3D printing, blank = none)', 'Ext'); setField('Resample cuts every (mm, 0 = keep NC points)', 5); clickOk(); await p;
    expect(lastToast()).toMatch(/points, \d+ segments/); expect(app.activeProgram!.name).not.toBe('PickPlace');
    p = machiningDialog(app, r2); clickCancel(); await p;
    p = curveFollowDialog(app, r2); expect(dialogTitle()).toBe(`Follow curve / points with ${r2.name}`); setField('Point spacing along curve (mm, 0 = vertices)', 5); clickOk(); await p;
    expect(lastToast()).toMatch(/^\d+ points programmed, \d+ unreachable$/);
    const pts = app.station.addChild(new SceneObject('pts')); pts.points = [{ name: 'a', point: [0, 0, 0] }, { name: 'b', point: [30, 0, 0] }]; pts.setPose(mul(transl(500, -200, 700), rotx(180 * DEG)));
    p = curveFollowDialog(app, r2); setField('Object', pts.id); setField('Follow', 'points'); setField('Digital output while following (empty = none)', ''); clickOk(); await p; expect(lastToast()).toMatch(/points programmed/);
    p = curveFollowDialog(app, r2); clickCancel(); await p;
    // rail / gantry
    await railIKDialog(app, r2); await waitFor(() => /not mounted/.test(lastToast()));
    const gantry = app.station.addChild(createRobotFromLibrary('GANTRY_XYZ', 'Gantry')); const ur = gantry.addChild(createRobotFromLibrary('UR5e', 'Rail UR'));
    p = railIKDialog(app, ur); await waitFor(() => dialogs().length === 1); expect(dialogTitle()).toMatch(/^Move Rail UR \+ Gantry/); setField('Target X (mm, carrier base frame)', 800); setField('Target Y', 100); setField('Target Z', 600); clickOk(); await p;
    expect(lastToast()).toMatch(/^Carrier: \[|^Not reachable/);
    const tgt = app.station.addChild(new Target('Rail target')); tgt.setPose(mul(transl(900, 0, 700), rotx(180 * DEG))); app.select(tgt);
    p = railIKDialog(app, ur); await waitFor(() => dialogs().length === 1); clickOk(); await p; expect(lastToast()).toMatch(/^Carrier: \[|^Not reachable/);
    p = railIKDialog(app, ur); await waitFor(() => dialogs().length === 1); clickCancel(); await p;
  });
  it('export, import and robot parameter dialogs', async () => {
    pickplace();
    await exportDialog(app, null); expect(lastToast()).toBe('Select a program');
    const n0 = downloads.length;
    let p = exportDialog(app); const d = currentDialog(); expect(dialogTitle()).toBe('Export PickPlace');
    const pre = d.querySelector('pre.code-preview')!; expect(pre.textContent!.length).toBeGreaterThan(50); expect(fieldInput('Post processor').value).toBe('Universal_Robots');
    setField('Post processor', 'KUKA_KRC4'); expect(pre.textContent).toMatch(/DEF |PTP|LIN/);
    dialogButton('Run a RoboDK Python post (.py)…').click(); resolvePickFiles([]); await tickMs(1);
    clickOk(); await p; expect(downloads.length).toBeGreaterThan(n0); expect(downloads.at(-1)!.name).toMatch(/\.(src|dat)$/);
    const mp = app.addProgram(app.addMobileRobot(), 'Mobile'); p = exportDialog(app, mp); expect(fieldInput('Post processor').value).toBe('JSON'); clickCancel(); await p;
    p = importDialog(app); resolvePickFiles([textFile('pts.csv', '100,0,300,180,0,0\n')]); await p; expect(app.logs.at(-1)!.text).toBe('Imported 1 targets from pts.csv');
    const r = app.station.itemsOfType<Robot>(ItemType.ROBOT)[0];
    p = robotParametersDialog(app, r); expect(dialogTitle()).toBe('UR10e parameters'); setField('Name', 'Arm'); setField('Post processor', 'KUKA_KRC4'); setField('Joint speed (deg/s)', 45); clickOk(); await p;
    expect(r.name).toBe('Arm'); expect(r.postProcessor).toBe('KUKA_KRC4'); expect(r.motion.speedJoints).toBe(45);
    p = robotParametersDialog(app, r); clickCancel(); await p; expect(r.name).toBe('Arm');
  });
});

describe('navigation stack UI', () => {
  it('section, wizard, export and the Navigation tab', async () => {
    orchard();
    const m = app.station.itemsOfType<MobileRobot>(ItemType.MOBILE_ROBOT)[0]; const live: Array<() => void> = [];
    let sec = navStackSection(app, m, live); expect(sec.textContent).toContain('not configured'); expect(buttonByText(sec, 'Select stack…')).toBeTruthy(); expect(sec.querySelector('.field-checkbox')).toBeNull();
    exportNavPackage(app, m); expect(lastToast()).toBe('Select a navigation stack first');
    let p = navStackDialog(app, m); const d = currentDialog(); expect(dialogTitle()).toBe('Navigation & SLAM stack'); expect(fieldInput('Platform').value).toBe('tracked'); expect(fieldInput('Environment').value).toBe('orchard');
    expect(d.querySelectorAll('.nav-rec').length).toBeGreaterThan(2);
    setField('Platform', 'amr'); expect((fieldInput('2D LiDAR') as HTMLInputElement).checked).toBe(true);
    setField('Environment', 'warehouse'); setField('GNSS sky view (0–1, −1 = environment default)', 0.5); setField('Max hardware cost (1–5)', 3); setField('Compute budget', 'low'); setField('Night operation', true); setField('Global repeatable frame needed (row entries, docking, fleet)', false);
    setField('3D LiDAR', true); setField('3D LiDAR', false);
    (d.querySelectorAll<HTMLElement>('.nav-rec')[1]).click(); expect(d.querySelectorAll('.nav-rec')[1].classList.contains('active')).toBe(true);
    clickOk(); await p;
    const cfg = getNavStack(m)!; expect(cfg).toBeTruthy(); expect(cfg.platform).toBe('amr'); expect(cfg.environment).toBe('warehouse'); expect(cfg.simulate).toBe(true); expect(cfg.gnssAvailability).toBe(0.5); expect(lastToast()).toMatch(/applied to /); expect(app.station.selection[0]).toBe(m);
    p = navStackDialog(app, m); expect(fieldInput('Platform').value).toBe('amr'); clickCancel(); await p;
    sec = navStackSection(app, m, live); expect(sec.textContent).toContain('Platform'); expect(buttonByText(sec, 'Change stack…')).toBeTruthy();
    change(sec.querySelector('.field-checkbox input')!, false); expect(getNavStack(m)!.simulate).toBe(false); change(sec.querySelector('.field-checkbox input')!, true); expect(getNavStack(m)!.simulate).toBe(true);
    buttonByText(sec, 'Navigation tab').click(); expect(shown).toEqual(['nav']);
    const n = downloads.length; buttonByText(sec, 'Export ROS 2 package').click(); expect(downloads.length).toBe(n + 1); expect(downloads.at(-1)!.name).toMatch(/\.zip$/); expect(app.logs.some((l) => /^ROS 2 navigation package/.test(l.text))).toBe(true);
    // Navigation tab
    const panel = buildNavPanel(app); document.body.appendChild(panel.el);
    app.newStation(); panel.render(); expect(panel.el.querySelector('.hint')!.textContent).toMatch(/^Add a mobile robot/);
    orchard(); const m2 = app.station.itemsOfType<MobileRobot>(ItemType.MOBILE_ROBOT)[0];
    panel.render(); const sel = panel.el.querySelector('select')!; expect(sel.options.length).toBe(4); expect(panel.el.querySelector('.hint')!.textContent).toMatch(/^No navigation stack/);
    setNavStack(m2, { ...defaultNavStack('tracked', 'orchard'), simulate: false }); panel.render(); expect(panel.el.querySelector('.hint')!.textContent).toMatch(/localization simulation is off/);
    setNavStack(m2, { ...defaultNavStack('amr', 'warehouse'), sensors: ['wheel_odom', 'imu', 'lidar2d'], localization: 'slam_toolbox_2d', simulate: true }); panel.render(); expect(panel.el.querySelector('.hint')!.textContent).toMatch(/waiting for the world simulation/);
    app.startWorld(); for (let i = 0; i < 6; i++) (app as any).tick(0.2); app.pauseWorld();
    expect((m2 as any)._nav).toBeTruthy(); panel.render(); expect(panel.el.querySelector('.hint')!.textContent).toMatch(/error [\d.]+ m · RMSE/);
    sec = navStackSection(app, m2, live); expect(sec.textContent).toContain('Estimate error'); expect(sec.textContent).toContain('SLAM map');
    buttonByText(panel.el, 'Reset estimate').click(); expect((m2 as any)._nav).toBeUndefined();
    buttonByText(panel.el, 'Export ROS 2').click(); expect(downloads.at(-1)!.name).toMatch(/\.zip$/);
    buttonByText(panel.el, 'Stack…').click(); expect(dialogTitle()).toBe('Navigation & SLAM stack'); clickCancel(); await tickMs(1);
    change(sel, app.station.itemsOfType<MobileRobot>(ItemType.MOBILE_ROBOT)[3].id); panel.render(); expect(sel.value).toBe(app.station.itemsOfType<MobileRobot>(ItemType.MOBILE_ROBOT)[3].id);
    panel.el.remove();
  });
});

describe('machine vision UI', () => {
  const pcd = () => { const pts: string[] = []; for (let x = 0; x < 10; x++) for (let y = 0; y < 10; y++) pts.push(`${x} ${y} 0.01`); for (const [cx, cy] of [[2, 2], [5, 5], [8, 3]]) for (let k = 0; k < 15; k++) pts.push(`${cx + (k % 3) * 0.05} ${cy + Math.floor(k / 3) * 0.05} ${0.3 + k * 0.08}`); return `# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH ${pts.length}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS ${pts.length}\nDATA ascii\n${pts.join('\n')}\n`; };
  it('adds cameras with sensible default poses', () => {
    pickplace();
    app.select(null); const c1 = addCamera(app); expect(c1.name).toBe('Camera 1'); expect(c1.parent).toBe(app.station); expect(c1.pose()[13]).toBeCloseTo(-1500, 6); expect(c1.kind).toBe('rgb');
    const tool = app.activeRobot!.tools()[0]; app.select(tool); const c2 = addCamera(app); expect(c2.parent).toBe(tool); expect(c2.name).toBe('Camera 2');
    const m = app.addMobileRobot(); const c3 = addCamera(app, m, 'lidar3d'); expect(c3.parent).toBe(m); expect(c3.kind).toBe('lidar3d'); expect(c3.fov).toBe(360); expect(c3.pose()[14]).toBeCloseTo(1200, 6);
    const c4 = addCamera(app, app.station, 'rgbd'); expect(c4.kind).toBe('depth');
    expect(camerasOf(app).length).toBe(4);
  });
  it('section, wizard, export and point clouds', async () => {
    orchard();
    const platform = app.station.itemsOfType<MobileRobot>(ItemType.MOBILE_ROBOT)[0]; const cam = addCamera(app, platform); const live: Array<() => void> = [];
    let sec = visionSection(app, cam, live); expect(sec.textContent).toContain('not configured'); expect(buttonByText(sec, 'Select vision stack…')).toBeTruthy();
    exportVisionPackage(app, cam); expect(lastToast()).toBe('Select a vision stack first');
    let p = visionStackDialog(app, cam); const d = currentDialog(); expect(dialogTitle()).toBe(`Machine vision stack — ${cam.name}`);
    expect(fieldInput('Environment').value).toBe('orchard'); expect((fieldInput('Sensor on a moving vehicle') as HTMLInputElement).checked).toBe(true); expect(fieldInput('Classes (comma separated; prompt for open-vocabulary models)').value).toMatch(/^apple/);
    expect(d.querySelectorAll('.nav-rec').length).toBeGreaterThan(2);
    setField('Segmentation', true); setField('Environment', 'greenhouse'); setField('Sensor modality', 'stereo'); setField('Compute', 'edge_mid'); setField('Working distance (m)', 2); setField('Required rate (Hz)', 5); setField('Max hardware cost (1–5)', 4);
    setField('3D positions needed (picking, distances)', true); setField('Night / dark operation', true); setField('Open vocabulary (text-prompted classes, no training)', true); setField('Connectivity available (cloud / hosted models allowed)', true); setField('Sensor on a moving vehicle', false);
    setField('VLM query', true); setField('VLA policy', true);
    setField('Runtime (where models execute)', 'simulated'); setField('ONNX model URL / path (Ultralytics export)', '/m.onnx'); setField('Input size (px)', 320); setField('Server model (name in STUDIO_VISION_MODELS or path)', 'best.pt'); setField('ROS 2 detections topic', '/det'); setField('VLM endpoint (OpenAI-compatible base URL)', 'http://vlm'); setField('VLM model', 'vlm-1'); setField('VLM API key (optional)', 'k'); setField('VLA policy server URL', 'http://vla'); setField('VLA model / checkpoint', 'pi'); setField('VLA server format', 'openvla'); setField('Default instruction', 'pick'); setField('Object size prior for mono (mm)', 80); setField('Publish results to URL (webhook, POST JSON after every run)', ''); setField('Publish to ROS 2 through rosbridge (Connect › ROS 2): vision_msgs, PoseArray, PointCloud2, CompressedImage', true); setField('Camera mounting', 'eye_to_hand');
    setField('Sensor modality', 'any'); setField('Compute', 'any'); expect(d.querySelectorAll('.nav-rec').length).toBeGreaterThan(0);
    (d.querySelectorAll<HTMLElement>('.nav-rec')[1] ?? d.querySelector<HTMLElement>('.nav-rec')!).click();
    for (const t of ['Detection', 'Tracking', 'Classification', 'Segmentation', 'VLM query', 'VLA policy', '6D pose']) { const f = dialogFields(d).find((l) => (l.querySelector('.field-label')?.textContent ?? '').startsWith(t)); if (f) { const i = f.querySelector('input') as HTMLInputElement; i.checked = false; i.dispatchEvent(new Event('change')); } }
    expect(d.querySelector('.hint.warn')).not.toBeNull(); // no tasks → no stack
    setField('Detection', true); setField('Tracking', true); setField('6D pose', true); setField('VLM query', true); setField('VLA policy', true);
    clickOk(); await p;
    const cfg = getVisionStack(cam)!; expect(cfg).toBeTruthy(); expect(cfg.tasks).toContain('detect'); expect(cfg.vlm!.model).toBe('vlm-1'); expect(cfg.vla!.format).toBe('openvla'); expect(cfg.inputSize).toBe(320); expect(cfg.publishRos).toBe(true); expect(cfg.handEyeMode).toBe('eye_to_hand'); expect(cfg.environment).toBe('greenhouse');
    expect(lastToast()).toMatch(/applied to Camera/); expect(shown).toEqual(['vision']);
    sec = visionSection(app, cam, live); expect(sec.textContent).toContain('Error budget'); expect(buttonByText(sec, 'Change vision stack…')).toBeTruthy();
    buttonByText(sec, 'Vision tab').click(); expect(shown).toEqual(['vision', 'vision']);
    const n = downloads.length; buttonByText(sec, 'Export ROS 2 package').click(); expect(downloads.length).toBe(n + 1); expect(downloads.at(-1)!.name).toMatch(/\.zip$/); expect(lastToast()).toMatch(/^ROS 2 package .*\.zip exported$/);
    p = visionStackDialog(app, cam); expect(currentDialog().querySelector('details')!.open).toBe(false); clickCancel(); await p; expect(getVisionStack(cam)).toBeTruthy();
    buttonByText(sec, 'Remove').click(); expect(getVisionStack(cam)).toBeNull();
    const obj = importPointCloud(app, 'scan.pcd', new TextEncoder().encode(pcd()).buffer as ArrayBuffer);
    expect(obj.name).toBe('scan'); expect(obj.points.length).toBe(145); expect((app as any).pointClouds.get(obj.id).xyz.length).toBe(145 * 3); expect(app.logs.at(-1)!.text).toMatch(/^scan.pcd: 145 points, extent/); expect(app.station.selection[0]).toBe(obj);
    expect(() => importPointCloud(app, 'bad.pcd', new TextEncoder().encode('VERSION 0.7\nFIELDS a b\nPOINTS 0\nDATA ascii\n').buffer as ArrayBuffer)).toThrow(/x\/y\/z/);
  });
  it('Vision tab runs the pipeline, draws, creates targets, follows and analyses clouds', async () => {
    orchard();
    const panel = buildVisionPanel(app); document.body.appendChild(panel.el);
    const info = panel.el.querySelector<HTMLElement>('.hint')!;
    app.newStation(); panel.render(); expect(info.textContent).toMatch(/^Add a camera/);
    buttonByText(panel.el, 'Stack…').click(); expect(camerasOf(app).length).toBe(1); // no camera yet: adds one
    buttonByText(panel.el, 'Add camera').click(); expect(camerasOf(app).length).toBe(2);
    panel.render(); await waitFor(() => /No vision stack/.test(info.textContent ?? ''));
    orchard(); const platform = app.station.itemsOfType<MobileRobot>(ItemType.MOBILE_ROBOT)[0]; const cam = addCamera(app, platform);
    let p = visionStackDialog(app, cam); clickOk(); await p; expect(getVisionStack(cam)).toBeTruthy();
    panel.render(); const sel = panel.el.querySelector('select')!; expect(sel.options.length).toBe(1); expect(sel.options[0].textContent).toContain(' — ');
    const stats = panel.el.querySelectorAll<HTMLElement>('.hint')[1];
    await waitFor(() => /^1 frames/.test(stats.textContent ?? ''), 5000, 'first run'); expect(info.textContent).toMatch(/^[\w+ -]+: /); expect(stats.textContent).toMatch(/^\d+ frames · precision/);
    buttonByText(panel.el, 'Run').click(); await waitFor(() => /^2 frames/.test(stats.textContent ?? ''), 5000, 'second run');
    (panel.el.querySelectorAll<HTMLInputElement>('input[type=checkbox]')[1]).checked = true; // truth boxes
    buttonByText(panel.el, 'Run').click(); await waitFor(() => /^3 frames/.test(stats.textContent ?? ''), 5000, 'third run');
    buttonByText(panel.el, 'Targets → station').click(); expect(lastToast()).toMatch(/targets created|No 3D positions/);
    if (/targets created/.test(lastToast())) expect(app.station.find('Vision Camera 1')).not.toBeNull();
    const robotSel = panel.el.querySelectorAll('select')[1]; expect(robotSel.options.length).toBe(4);
    (panel.el.querySelectorAll<HTMLInputElement>('input[type=checkbox]')[2]).checked = true; // follow
    buttonByText(panel.el, 'Run').click(); await waitFor(() => /^4 frames/.test(stats.textContent ?? ''), 5000, 'follow run');
    const follower = app.station.findById(robotSel.value) as MobileRobot; expect(follower.params.follow).toBeTruthy();
    const prompt = panel.el.querySelector<HTMLInputElement>('input.vision-prompt')!; prompt.value = 'how many apples?';
    buttonByText(panel.el, 'Ask / step').click(); await waitFor(() => /^5 frames/.test(stats.textContent ?? ''), 5000, 'ask run');
    const n = downloads.length; buttonByText(panel.el, 'Export ROS 2').click(); expect(downloads.length).toBe(n + 1);
    buttonByText(panel.el, 'Export cloud (.pcd)').click(); expect(lastToast()).toBe('No point cloud in the last run');
    delete (app as any).pointClouds; buttonByText(panel.el, 'Analyse imported cloud').click(); expect(lastToast()).toMatch(/^Drop a .pcd/);
    importPointCloud(app, 'scan.pcd', new TextEncoder().encode(pcd()).buffer as ArrayBuffer);
    buttonByText(panel.el, 'Analyse imported cloud').click(); expect(info.textContent).toMatch(/^145 points → \d+ after voxel · ground \d+ · objects \d+ · \d+ clusters/); expect(panel.el.querySelector<HTMLCanvasElement>('canvas.vision-bev')!.style.display).toBe('');
    buttonByText(panel.el, 'Export cloud (.pcd)').click(); expect(downloads.at(-1)!.name).toBe('Camera 1.pcd');
    buttonByText(panel.el, 'Reset stats').click(); panel.render(); await waitFor(() => /^1 frames/.test(stats.textContent ?? ''), 5000, 'after reset');
    buttonByText(panel.el, 'Stack…').click(); expect(dialogTitle()).toMatch(/^Machine vision stack/); clickCancel(); await tickMs(1);
    (panel.el.querySelectorAll<HTMLInputElement>('input[type=checkbox]')[2]).checked = false;
    panel.stop(); panel.el.remove();
  });
});

describe('scenarios dialog', () => {
  it('lists, filters, loads and runs scenarios and downloads the report', async () => {
    pickplace();
    let p = scenariosDialog(app); const d = currentDialog(); expect(dialogTitle()).toMatch(/^Demo scenarios/);
    expect(d.querySelectorAll('.nav-rec').length).toBe(ALL_SCENARIOS.length);
    const sel = d.querySelector('select')!; expect(sel.value).toBe('all');
    change(sel, 'vision'); expect(d.querySelectorAll('.nav-rec').length).toBe(ALL_SCENARIOS.filter((s) => s.group === 'vision').length);
    dialogButton('Download report (.md)', d).click(); expect(lastToast()).toBe('Run the scenarios first');
    change(sel, 'control');
    const cheap = ALL_SCENARIOS.find((s) => s.id === 'ctl_decisions')!; const idx = ALL_SCENARIOS.filter((s) => s.group === 'control').indexOf(cheap);
    const row = d.querySelectorAll<HTMLElement>('.nav-rec')[idx]; expect(row.textContent).toContain(cheap.title); expect(row.textContent).toContain(cheap.method);
    buttonByText(row, 'Run headless').click(); await waitFor(() => d.querySelectorAll<HTMLElement>('.nav-rec')[idx].querySelector('.badge.ok, .badge.warn'));
    expect(d.querySelectorAll<HTMLElement>('.nav-rec')[idx].querySelector('.badge')!.textContent).toMatch(/pass|FAIL/); expect(app.logs.at(-1)!.text).toMatch(/^(PASS|FAIL) /);
    const n = downloads.length; dialogButton('Download report (.md)', d).click(); expect(downloads.length).toBe(n + 1); expect(downloads.at(-1)!.name).toBe('scenario-results.md');
    dialogButton('Run all (headless)', d).click(); await waitFor(() => /scenarios pass/.test(d.querySelectorAll('.hint')[0]?.textContent ?? '') || /scenarios pass/.test(d.textContent ?? ''), 15000, 'run all');
    expect(d.textContent).toMatch(/\d+\/\d+ scenarios pass/);
    buttonByText(d.querySelectorAll<HTMLElement>('.nav-rec')[0], 'Load into the studio').click(); await p;
    expect(dialogs().length).toBe(0); expect(shown).toEqual(['control']); expect(lastToast()).toMatch(/loaded$/); expect(app.station.name).not.toBe('Pick & place cell');
    p = scenariosDialog(app, 'navigation'); expect((currentDialog().querySelector('select') as HTMLSelectElement).value).toBe('navigation');
    buttonByText(currentDialog().querySelector<HTMLElement>('.nav-rec')!, 'Load into the studio').click(); await p; expect(shown).toEqual(['control', 'nav']); expect(app.worldRunning).toBe(true); app.pauseWorld();
    p = scenariosDialog(app, 'group'); const render = vi.fn(); (app as any).controlPanel = { render }; buttonByText(currentDialog().querySelector<HTMLElement>('.nav-rec')!, 'Load into the studio').click(); await p; expect(render).toHaveBeenCalled(); expect(shown.at(-1)).toBe('control'); app.pauseWorld();
    p = scenariosDialog(app, 'vision'); buttonByText(currentDialog().querySelector<HTMLElement>('.nav-rec')!, 'Load into the studio').click(); await p; expect(shown.at(-1)).toBe('vision');
    p = scenariosDialog(app); dialogButton('Close').click(); await p;
  });
});

export { Program, Tool };
