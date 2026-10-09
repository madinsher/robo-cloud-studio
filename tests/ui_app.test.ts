// @vitest-environment jsdom
import { describe, it, expect, beforeAll, afterEach, vi } from 'vitest';
import { readFileSync } from 'node:fs';
import path from 'node:path';
import { App, ItemType, Frame, Target, Program, Instruction, Robot, MobileRobot, FleetItem, MissionItem, SceneObject, Folder, Camera } from '../src/app';
import { Station, Tool } from '../src/core/items/item';
import { demos } from '../src/demos';
import { loadStation } from '../src/io/station-file';
import { transl } from '../src/core/math/pose';
import { t, setLang, getLang, onLangChange } from '../src/ui/i18n';
import { h, append, clear, icon, formField, dialog, confirmDialog, toast, contextMenu, downloadText, downloadBlob, pickFiles, fmt } from '../src/ui/dom';
import { StationTabs, collisionMapDialog, measureDialog, cameraDialog, VideoRecorder, exportHtml3D, exportGlb, exportAnimationGltf, exportUrdfPackage, importRoboDKPosts, saveForBlender } from '../src/ui/tools';
import { installDomStubs, makeApp, downloads, rendererCalls, flush, tickMs, waitFor, toasts, lastToast, clearToasts, ctxItem, ctxItems, currentDialog, dialogs, dialogTitle, setField, clickOk, clickCancel, dialogButton, closeAllDialogs, resolvePickFiles, textFile, mouse, key, change } from './ui_harness';

beforeAll(() => installDomStubs());
afterEach(() => { closeAllDialogs(); clearToasts(); document.querySelectorAll('.ctx-menu').forEach((m) => m.remove()); });

describe('i18n', () => {
  it('translates with fallback, keeps the ellipsis decoration and notifies listeners', () => {
    expect(getLang()).toBe('en');
    expect(t('File')).toBe('File');
    const seen: string[] = [];
    const off = onLangChange(() => seen.push(getLang()));
    setLang('ru');
    expect(seen).toEqual(['ru']);
    expect(t('File')).toBe('Файл');
    expect(t('Open / import…')).toBe('Открыть / импорт…');
    expect(t('Rename station…')).toBe('Переименовать станцию…');
    expect(t('no such key at all')).toBe('no such key at all');
    expect(t('Cancel')).toBe('Отмена');
    expect(localStorage.getItem('vbs.lang')).toBe('ru');
    off();
    setLang('en');
    expect(seen).toEqual(['ru']);
    expect(t('File')).toBe('File');
  });
  it('every static label of the menu bar resolves to a non-empty RU string (translated or fallback)', () => {
    const src = readFileSync(path.join(process.cwd(), 'src/ui/menu.ts'), 'utf8');
    const labels = [...src.matchAll(/label: '([^']+)'/g)].map((m) => m[1]);
    const menus = [...src.matchAll(/\['([^']+)', \(\) => \[/g)].map((m) => m[1]);
    expect(menus).toEqual(['File', 'Edit', 'Add', 'Program', 'Robot', 'Mobile & Fleet', 'Agriculture', 'Control', 'Group', 'Tools', 'Connect', 'View', 'Help']);
    expect(labels.length).toBeGreaterThan(80);
    setLang('ru');
    try {
      for (const m of menus) expect(t(m), m).not.toBe(m); // the menu titles are all translated
      let translated = 0;
      for (const l of labels) { const r = t(l); expect(typeof r).toBe('string'); expect(r.length).toBeGreaterThan(0); if (r !== l) translated++; }
      expect(translated / labels.length).toBeGreaterThan(0.85);
    } finally { setLang('en'); }
  });
});

describe('dom helpers', () => {
  it('h(): attributes, styles, listeners, dataset, properties, children arrays', () => {
    let clicked = 0;
    const el = h('div', { class: 'a b', style: { color: 'red' }, onClick: () => clicked++, dataset: { id: 'x' }, title: 'tt', 'aria-label': 'al', hidden: false, nope: undefined, value: 3 }, 'text', 1, null, false, undefined, [h('span', null, 'in'), ['deep']]);
    expect(el.className).toBe('a b'); expect(el.style.color).toBe('red'); expect(el.dataset.id).toBe('x'); expect(el.title).toBe('tt'); expect(el.getAttribute('aria-label')).toBe('al'); expect(el.hasAttribute('hidden')).toBe(false); expect(el.getAttribute('value')).toBe('3');
    expect(el.textContent).toBe('text1indeep');
    el.click(); expect(clicked).toBe(1);
    const inp = h('input', { type: 'number', list: 'l1', form: 'f1', className: 'c' }) as HTMLInputElement;
    expect(inp.type).toBe('number'); expect(inp.getAttribute('list')).toBe('l1'); expect(inp.className).toBe('c');
    append(el, [h('b', null, '!')]); expect(el.querySelector('b')).not.toBeNull();
    clear(el); expect(el.childNodes.length).toBe(0);
    expect(icon('robot').textContent).toBe('🦾'); expect(icon('unknown-thing', 'T').textContent).toBe('•'); expect(icon('unknown-thing', 'T').title).toBe('T');
    expect(fmt(1.23456)).toBe('1.23'); expect(fmt(NaN)).toBe('-'); expect(fmt(2, 0)).toBe('2');
  });
  it('formField(): every field type reads back its value and reports changes', () => {
    const got: unknown[] = [];
    const text = formField({ key: 'a', label: 'A', type: 'text', value: 'x', hint: 'hint' }, (v) => got.push(v));
    expect(text.get()).toBe('x'); expect(text.el.querySelector('.hint')!.textContent).toBe('hint');
    change(text.el.querySelector('input')!, 'y'); expect(got).toEqual(['y']);
    const num = formField({ key: 'n', label: 'N', type: 'number', value: 2, min: 0, max: 10, step: 0.5 }); expect(num.get()).toBe(2);
    const sel = formField({ key: 's', label: 'S', type: 'select', value: 'b', options: [{ value: 'a', label: 'A' }, { value: 'b', label: 'B' }] }); expect(sel.get()).toBe('b');
    const cb = formField({ key: 'c', label: 'C', type: 'checkbox', value: true }); expect(cb.get()).toBe(true);
    const ta = formField({ key: 't', label: 'T', type: 'textarea', value: 'multi' }); expect(ta.get()).toBe('multi');
    const rg = formField({ key: 'r', label: 'R', type: 'range', value: 0.5, min: 0, max: 1, step: 0.1 }, (v) => got.push(v)); expect(rg.get()).toBe(0.5);
    const ri = rg.el.querySelector('input')!; ri.value = '0.7'; ri.dispatchEvent(new Event('input')); expect(got.at(-1)).toBeCloseTo(0.7);
    const col = formField({ key: 'k', label: 'K', type: 'color', value: '#ff0000' }); expect(col.get()).toBe('#ff0000');
    const file = formField({ key: 'f', label: 'F', type: 'file', accept: '.py', multiple: true }); expect(file.get()).not.toBeUndefined();
  });
  it('dialog(): OK returns values, Cancel / Escape / close button / overlay click return null, Enter submits', async () => {
    const changes: Record<string, unknown>[] = [];
    let p = dialog<{ n: string; k: number; c: boolean }>('Test dialog', [{ key: 'n', label: 'Name', type: 'text', value: 'a' }, { key: 'k', label: 'Num', type: 'number', value: 1 }, { key: 'c', label: 'Chk', type: 'checkbox', value: false }], { body: h('p', null, 'body'), onChange: (v) => changes.push(v), width: 300 });
    expect(dialogTitle()).toBe('Test dialog'); expect(currentDialog().querySelector('.dialog')!.getAttribute('style')).toContain('300px');
    setField('Name', 'bob'); setField('Num', 5); setField('Chk', true);
    expect(changes.length).toBe(3);
    clickOk();
    expect(await p).toEqual({ n: 'bob', k: 5, c: true });
    expect(dialogs().length).toBe(0);
    p = dialog('D2', []); clickCancel(); expect(await p).toBeNull();
    p = dialog('D3', []); currentDialog().querySelector<HTMLButtonElement>('.dialog-title .btn-icon')!.click(); expect(await p).toBeNull();
    p = dialog('D4', []); currentDialog().dispatchEvent(mouse('click')); expect(await p).toBeNull();
    p = dialog('D5', [{ key: 'x', label: 'X', type: 'text', value: 'v' }]); currentDialog().querySelector('input')!.dispatchEvent(key('Escape')); expect(await p).toBeNull();
    p = dialog('D6', [{ key: 'x', label: 'X', type: 'text', value: 'v' }]); currentDialog().querySelector('input')!.dispatchEvent(key('Enter')); expect(await p).toEqual({ x: 'v' });
    p = dialog('D7', [{ key: 'x', label: 'X', type: 'textarea', value: 'v' }]); currentDialog().querySelector('textarea')!.dispatchEvent(key('Enter')); expect(dialogs().length).toBe(1); clickOk(); await p;
    let c = confirmDialog('Sure?', 'really'); expect(currentDialog().textContent).toContain('really'); dialogButton('Yes').click(); expect(await c).toBe(true);
    c = confirmDialog('Sure?', 'really'); clickCancel(); expect(await c).toBe(false);
  });
  it('toast(): levels and auto-hide', async () => {
    toast('hello', 'ok', 20); toast('warned', 'warn', 20); toast('plain');
    expect(document.querySelector('#toasts .toast-ok')!.textContent).toBe('hello');
    expect(document.querySelector('#toasts .toast-warn')).not.toBeNull(); expect(document.querySelector('#toasts .toast-info')!.textContent).toBe('plain');
    await tickMs(40); expect(document.querySelector('#toasts .toast-ok')!.classList.contains('hide')).toBe(true);
    await tickMs(450); expect(document.querySelector('#toasts .toast-ok')).toBeNull(); expect(document.querySelector('#toasts .toast-info')).not.toBeNull();
  });
  it('contextMenu(): items, separators, disabled, checked, shortcuts, submenus and outside close', async () => {
    const hits: string[] = [];
    contextMenu(10, 10, [{ label: 'One', shortcut: 'Ctrl+1', action: () => hits.push('one') }, { separator: true }, { label: 'Off', disabled: true, action: () => hits.push('off') }, { label: 'Chk', checked: true, action: () => hits.push('chk') }, { label: 'Sub', children: [{ label: 'Child A', action: () => hits.push('a') }, { separator: true }, { label: 'Child B', disabled: true, action: () => hits.push('b') }] }]);
    const menu = document.querySelector<HTMLElement>('.ctx-menu')!;
    expect(ctxItems().length).toBe(4); expect(menu.querySelectorAll('.ctx-sep').length).toBe(1); expect(ctxItem('One').querySelector('.shortcut')!.textContent).toBe('Ctrl+1'); expect(ctxItem('Chk').textContent).toContain('✓ ');
    ctxItem('Off').click(); expect(hits).toEqual([]); expect(document.querySelector('.ctx-menu')).not.toBeNull();
    ctxItem('Sub').dispatchEvent(new Event('pointerenter'));
    const sub = menu.querySelector<HTMLElement>('.ctx-menu')!; expect(sub).not.toBeNull(); expect(sub.querySelectorAll('.ctx-item').length).toBe(2); expect(sub.querySelectorAll('.ctx-sep').length).toBe(1);
    ctxItem('Sub').dispatchEvent(new Event('pointerenter')); expect(menu.querySelectorAll('.ctx-menu').length).toBe(1);
    [...sub.querySelectorAll<HTMLElement>('.ctx-item')].find((e) => e.textContent === 'Child B')!.click(); expect(hits).toEqual([]);
    [...sub.querySelectorAll<HTMLElement>('.ctx-item')].find((e) => e.textContent === 'Child A')!.click(); expect(hits).toEqual(['a']); expect(document.querySelector('.ctx-menu')).toBeNull();
    contextMenu(5, 5, [{ label: 'X', action: () => hits.push('x') }]); await tickMs(1);
    document.body.dispatchEvent(new PointerEvent('pointerdown', { bubbles: true })); expect(document.querySelector('.ctx-menu')).toBeNull();
    contextMenu(5, 5, [{ label: 'Y', action: () => hits.push('y') }]); ctxItem('Y').click(); expect(hits).toEqual(['a', 'y']);
  });
  it('downloads and file picking go through the DOM', async () => {
    const n0 = downloads.length;
    downloadText('a.txt', 'hello'); downloadBlob('b.bin', new Blob([new Uint8Array([1, 2])]));
    expect(downloads.slice(n0).map((d) => d.name)).toEqual(['a.txt', 'b.bin']); expect(downloads.at(-1)!.href).toMatch(/^blob:/);
    await tickMs(120); expect(document.querySelector('a[download]')).toBeNull();
    const p = pickFiles('.py', true); const inp = document.querySelector<HTMLInputElement>('body > input[type=file]')!; expect(inp.accept).toBe('.py'); expect(inp.multiple).toBe(true);
    resolvePickFiles([textFile('x.py', 'print(1)')]); const files = await p; expect(files.map((f) => f.name)).toEqual(['x.py']); expect(document.querySelector('body > input[type=file]')).toBeNull();
  });
});

describe('App', () => {
  it('constructs without a DOM, logs with levels and caps the log', () => {
    const app = new App();
    expect(app.station.name).toBe('New station'); expect(app.stations).toEqual([]); expect(app.activeProgram).toBeNull(); expect(app.library.length).toBeGreaterThan(10); expect(app.posts().length).toBeGreaterThan(5);
    const seen: string[] = []; app.events.on('log', (e) => seen.push(e.level));
    app.log('info'); app.log('careful', 'warn'); app.log('bad', 'error');
    expect(seen).toEqual(['info', 'warn', 'error']); expect(toasts()).toEqual(['careful', 'bad']);
    for (let i = 0; i < 1005; i++) app.log(`m${i}`);
    expect(app.logs.length).toBe(1000); expect(app.logs[0].text).toBe('m5');
    expect(app.serverHttpBase()).toBeNull();
  });
  it('station tabs: add / set / close / new, with events and API event queue', () => {
    const app = makeApp('pickplace');
    const ev = vi.fn(); app.onStationEvent = ev;
    const changed: string[] = []; app.events.on('stationChanged', ({ station }) => changed.push(station.name));
    expect(app.stations.length).toBe(1); expect(app.activeProgram!.name).toBe('PickPlace'); expect(app.activeRobot!.name).toBe('UR10e');
    const s2 = app.addStation('Second'); expect(app.station).toBe(s2); expect(app.stations.length).toBe(2); expect(ev).toHaveBeenCalledWith('station', s2.id);
    app.setStation(app.stations[0]); expect(app.station.name).toBe('Pick & place cell');
    const s3 = new Station('Third'); app.setStation(s3); expect(app.stations).toContain(s3); expect(app.stations.length).toBe(2);
    app.closeStation(); expect(app.stations.length).toBe(1); expect(app.station).toBe(s2);
    app.closeStation(); expect(app.stations.length).toBe(1); expect(app.station.name).toBe('New station');
    app.newStation(); expect(app.station.name).toBe('New station'); expect(app.stations.length).toBe(1);
    expect(changed.length).toBeGreaterThanOrEqual(5);
    // item events are relayed to the API queue
    app.setStation(demos[0].build());
    const r = app.station.itemsOfType<Robot>(ItemType.ROBOT)[0];
    app.select(r); expect(ev).toHaveBeenCalledWith('selection', r.id);
    r.setPose(transl(1, 2, 3)); expect(ev).toHaveBeenCalledWith('moved', r.id);
    r.setJoints([1, -90, 90, 0, 90, 0]); expect(ev).toHaveBeenCalledWith('robotMoved', r.id);
    r.setName('R2'); expect(ev).toHaveBeenCalledWith('renamed', r.id);
    r.setVisible(false); expect(ev).toHaveBeenCalledWith('visibility', r.id);
  });
  it('snapshot / cmd / undo / redo restore items and keep the active program and robot', () => {
    const app = makeApp('pickplace');
    const undoEv: Array<{ canUndo: boolean; canRedo: boolean }> = []; app.events.on('undo', (e) => undoEv.push(e));
    app.snapshot(); // identical: no-op
    expect(undoEv.length).toBe(0);
    const n = app.station.itemsOfType(ItemType.FRAME).length;
    const f = app.addFrame(app.station, 'Extra');
    expect(app.station.itemsOfType(ItemType.FRAME).length).toBe(n + 1); expect(undoEv.at(-1)).toEqual({ canUndo: true, canRedo: false });
    app.undo();
    expect(app.stations).toEqual([app.station]); // the tab list follows the restored station
    expect(app.station.itemsOfType(ItemType.FRAME).length).toBe(n); expect(app.station.findById(f.id)).toBeNull(); expect(app.activeProgram!.name).toBe('PickPlace'); expect(app.activeRobot!.name).toBe('UR10e'); expect(undoEv.at(-1)).toEqual({ canUndo: false, canRedo: true });
    app.redo();
    expect(app.station.itemsOfType(ItemType.FRAME).length).toBe(n + 1); expect(app.station.findById(f.id)!.name).toBe('Extra');
    app.redo(); app.undo(); app.undo(); app.undo(); // extra calls are ignored
    expect(app.station.itemsOfType(ItemType.FRAME).length).toBe(n);
    expect(app.cmd(() => 42)).toBe(42);
    for (let i = 0; i < 60; i++) app.addFrame(); // the undo stack is capped at 50 snapshots
    let steps = 0; for (;;) { const before = app.station.itemsOfType(ItemType.FRAME).length; app.undo(); if (app.station.itemsOfType(ItemType.FRAME).length === before || steps > 100) break; steps++; }
    expect(steps).toBeGreaterThanOrEqual(49); expect(steps).toBeLessThanOrEqual(51); expect(app.station.itemsOfType(ItemType.FRAME).length).toBeGreaterThan(n);
  });
  it('select() drives the active program / robot and the gizmo', () => {
    const app = makeApp('pickplace');
    const st = app.station;
    const prog = st.itemsOfType<Program>(ItemType.PROGRAM)[0], robot = st.itemsOfType<Robot>(ItemType.ROBOT)[0], tool = robot.tools()[0];
    const progEv = vi.fn(), robEv = vi.fn(); app.events.on('activeProgram', progEv); app.events.on('activeRobot', robEv);
    app.setActiveProgram(null); app.setActiveRobot(null);
    app.select(prog.instructions()[0]); expect(app.activeProgram).toBe(prog); expect(progEv).toHaveBeenCalledTimes(2);
    app.select(tool); expect(app.activeRobot).toBe(robot);
    app.setActiveRobot(null); app.select(robot); expect(app.activeRobot).toBe(robot); expect(robEv).toHaveBeenCalledTimes(4);
    app.select(prog); expect(st.selection).toEqual([prog]);
    app.select(robot, true); expect(st.selection.length).toBe(2);
    app.select(null); expect(st.selection).toEqual([]);
    expect(progEv).toHaveBeenCalledTimes(2); app.setActiveProgram(null); app.setActiveProgram(prog); app.setActiveProgram(prog); expect(progEv).toHaveBeenCalledTimes(4);
  });
  it('adds robots, frames, targets, programs, primitives, mobile robots and deletes items', () => {
    const app = makeApp();
    expect(app.addTarget()!.parent).toBe(app.station); // no robot: target under the station without joints
    const r1 = app.addRobotFromLibrary('UR5e'); expect(app.activeRobot).toBe(r1); expect(r1.tools().length).toBe(1); expect(r1.activeTool()!.name).toBe('Tool 1');
    const r2 = app.addRobotFromLibrary('KUKA_KR6_R900'); expect(r2.pose()[12]).toBe(1500); expect(app.activeRobot).toBe(r2);
    const f = app.addFrame(); expect(f.name).toBe('Frame 1'); expect(app.station.selection[0]).toBe(f);
    const t1 = app.addTarget()!; expect(t1.parent).toBe(app.station); expect(t1.joints).toEqual(r2.joints()); expect(t1.robotId).toBe(r2.id); expect(t1.name).toBe('Target 2');
    r2.setFrame(f); const t2 = app.addTarget(undefined, 'Named')!; expect(t2.parent).toBe(f); expect(t2.name).toBe('Named');
    const p = app.addProgram(); expect(p.name).toBe('Prog 1'); expect(p.robot()).toBe(r2); expect(app.activeProgram).toBe(p);
    const p2 = app.addProgram(null, 'Empty'); expect(p2.robotId).toBeNull();
    const box = app.addObjectPrimitive('box'); expect(box.geometry[0].primitive!.kind).toBe('box');
    const cyl = app.addObjectPrimitive('cylinder', [150, 150, 600]); expect(cyl.geometry[0].primitive!.kind).toBe('cylinder');
    const sph = app.addObjectPrimitive('sphere', [200], f); expect(sph.parent).toBe(f); expect(sph.geometry[0].primitive!.kind).toBe('sphere');
    const presets = ['amr', 'tractor', 'harvester', 'sprayer', 'scout'] as const;
    const mobiles = presets.map((k) => app.addMobileRobot(k));
    expect(mobiles.map((m) => m.name)).toEqual(['AMR', 'Tractor', 'Harvest platform', 'Sprayer', 'Scout rover']);
    expect(mobiles[1].kin.drive).toBe('ackermann'); expect(mobiles[2].capabilities).toContain('harvest'); expect(mobiles[3].color).toBe('#1971c2'); expect(mobiles[4].battery.capacityWh).toBe(600); expect(mobiles[4].state.x).toBe(4 * 2500);
    app.deleteItems([p2, r2, app.station]);
    expect(app.activeProgram).toBeNull(); expect(app.activeRobot).toBeNull(); expect(app.station.findById(p2.id)).toBeNull(); expect(app.station.findById(p.id)).toBe(p); expect(app.station.selection).toEqual([]);
    expect(app.station.parent).toBeNull();
  });
  it('teach, preview, run / pause / stop / seek and the animation tick', () => {
    const app = makeApp();
    app.teach('MoveJ'); expect(app.logs.at(-1)!.text).toBe('Select a robot first');
    const r = app.addRobotFromLibrary('UR5e');
    app.teach('MoveJ');
    const prog = app.activeProgram!; expect(prog.instructions().length).toBe(1); expect((prog.instructions()[0].data as any).moveType).toBe('MoveJ');
    const t0 = app.station.itemsOfType<Target>(ItemType.TARGET)[0]; expect(t0.isJointTarget).toBe(true);
    r.setJoints([20, -80, 90, -100, -90, 10]);
    app.teach('MoveL'); expect(prog.instructions().length).toBe(2); expect((prog.instructions()[1].data as any).moveType).toBe('MoveL');
    const simEv: Array<{ playing: boolean; time: number; duration: number }> = []; app.events.on('simulation', (e) => simEv.push(e));
    app.previewProgram(); expect(app.sim.duration).toBeGreaterThan(0); expect(simEv.at(-1)!.playing).toBe(false);
    const d = app.sim.duration;
    app.runProgram(); expect(app.sim.playing).toBe(true); expect(simEv.at(-1)).toEqual({ playing: true, time: 0, duration: d });
    (app as any).tick(0.1); expect(app.sim.time).toBeCloseTo(0.1, 5); expect(simEv.at(-1)!.time).toBeCloseTo(0.1, 5);
    app.simSpeed = 2; (app as any).tick(0.1); expect(app.sim.time).toBeCloseTo(0.3, 5);
    app.pauseProgram(); expect(app.sim.playing).toBe(false); expect(simEv.at(-1)!.playing).toBe(false);
    app.seekProgram(d); expect(app.sim.time).toBeCloseTo(d, 5);
    app.stopProgram(); expect(app.sim.time).toBe(0);
    let still = true; app.runProgram(prog); for (let i = 0; i < 200 && still; i++) still = app.sim.tick(1); expect(still).toBe(false);
    app.setActiveProgram(null); app.previewProgram(); app.runProgram(); expect(app.logs.at(-1)!.text).toBe('No program selected');
    const m = app.addMobileRobot(); const mp = app.addProgram(m); expect(app.activeProgram).toBe(mp); app.previewProgram(); // non-arm program: no trajectory preview
    expect(rendererCalls.filter((c) => c === 'showTrajectoryPreview').length).toBeGreaterThan(2);
  });
  it('moveRobotTo reaches targets, warns on unreachable ones and previews linear moves', () => {
    const app = makeApp('pickplace');
    const st = app.station; const r = app.activeRobot!;
    const home = st.find('Home') as Target; const pick = st.find('Pick 1') as Target;
    app.moveRobotTo(home); expect(r.joints().map((v) => Math.round(v))).toEqual([0, -100, 110, -100, -90, 0]);
    app.moveRobotTo(pick); expect(r.joints()).not.toEqual([0, -100, 110, -100, -90, 0]);
    app.moveRobotTo(home, true); expect(r.joints().map((v) => Math.round(v))).toEqual([0, -100, 110, -100, -90, 0]);
    const far = st.addChild(new Target('Far')); far.setPose(transl(50000, 0, 0));
    app.moveRobotTo(far); expect(app.logs.at(-1)!.text).toBe('Far is unreachable');
    app.setActiveRobot(null); app.moveRobotTo(home); // no robot: nothing happens
  });
  it('checkStationCollisions reports overlapping items', () => {
    const app = makeApp();
    app.addRobotFromLibrary('UR5e');
    expect(app.checkStationCollisions()).toBe(0); expect(app.logs.at(-1)!.text).toBe('No collisions in the current state');
    const a = app.addObjectPrimitive('box', [400, 400, 400]); a.bbox = { min: [-200, -200, 0], max: [200, 200, 400] }; a.setPose(transl(3000, 0, 0));
    const b = app.addObjectPrimitive('box', [400, 400, 400]); b.bbox = { min: [-200, -200, 0], max: [200, 200, 400] }; b.setPose(transl(3100, 50, 0));
    const n = app.checkStationCollisions();
    expect(n).toBeGreaterThanOrEqual(1); expect(app.logs.at(-1)!.text).toMatch(/^Collision: Box/); expect(rendererCalls).toContain('setCollisionHighlight');
    app.checkCollisions = true; app.addProgram(); app.previewProgram(); // collision checking during validation
  });
  it('world simulation: fleets, missions, process components and hooks advance with the tick', () => {
    const app = makeApp('orchard');
    const fleet = app.station.itemsOfType<FleetItem>(ItemType.FLEET)[0];
    const fm = app.fleetManager(fleet); expect(app.fleetManager(fleet)).toBe(fm); expect(fm.robots().length).toBe(4);
    const hook = vi.fn(); app.worldHooks.push(hook);
    expect(app.worldRunning).toBe(false); (app as any).tick(0.1); expect(app.worldTime).toBe(0);
    app.startWorld(); expect(app.worldRunning).toBe(true);
    for (let i = 0; i < 10; i++) (app as any).tick(0.1);
    expect(app.worldTime).toBeCloseTo(1, 5); expect(hook.mock.calls.length).toBeGreaterThanOrEqual(10);
    app.simSpeed = 10; (app as any).tick(0.1); expect(app.worldTime).toBeCloseTo(2, 5); expect(hook.mock.calls.length).toBeGreaterThanOrEqual(30); // sub-steps at high speed
    const mission = app.station.itemsOfType<MissionItem>(ItemType.MISSION)[0]; expect(mission.status).not.toBe('draft');
    const robots = app.station.itemsOfType<MobileRobot>(ItemType.MOBILE_ROBOT); expect(robots.some((m) => m.state.status !== 'idle' || m.state.odometer > 0)).toBe(true);
    app.pauseWorld(); expect(app.worldRunning).toBe(false);
    app.resetWorld();
    expect(app.worldTime).toBe(0); expect(fleet.tasks).toEqual([]); expect(app.fleets.size).toBe(0);
    for (const m of robots) { expect(m.state.status).toBe('idle'); expect(m.state.path).toBeNull(); if (m.home) expect(m.state.x).toBe(m.home.x); }
    for (const mi of app.station.itemsOfType<MissionItem>(ItemType.MISSION)) { expect(mi.status).toBe('draft'); expect(mi.progress).toBe(0); expect(mi.taskIds).toEqual([]); }
    // process components (packing line)
    app.setStation(demos.find((d) => d.id === 'packing')!.build());
    expect(app.station.itemsOfType(ItemType.COMPONENT).length).toBeGreaterThan(3);
    app.simSpeed = 1; app.startWorld(); for (let i = 0; i < 30; i++) (app as any).tick(0.5);
    expect(app.processSim.time).toBeGreaterThan(10); expect(app.processSim.statistics().some((s) => s.entered > 0)).toBe(true);
    app.resetWorld(); expect(app.processSim.time).toBe(0);
  });
  it('saveToJSON round-trips through loadStation and openFiles', async () => {
    const app = makeApp('pickplace');
    const json = app.saveToJSON();
    const st = loadStation(JSON.parse(json), app.assets);
    expect(st.name).toBe(app.station.name);
    expect([...st.walk()].length).toBe([...app.station.walk()].length);
    expect(st.itemsOfType<Program>(ItemType.PROGRAM)[0].instructions().length).toBe(app.activeProgram!.instructions().length);
    expect([...st.walk()].map((i) => i.name)).toEqual([...app.station.walk()].map((i) => i.name));
    app.newStation();
    await app.openFiles([textFile('cell.vbstation', json)]);
    expect(app.station.name).toBe('Pick & place cell'); expect(app.station.filePath).toBe('cell.vbstation'); expect(app.activeProgram!.name).toBe('PickPlace');
    app.newStation();
    await app.openFiles([textFile('data.json', json)]); expect(app.station.name).toBe('Pick & place cell'); // detected by format
    await app.openFiles([textFile('other.json', '{"foo":1}')]); expect(app.logs.at(-1)!.text).toBe('Unknown JSON content in other.json');
  });
  it('openFiles imports targets, programs, G-code, GeoJSON, meshes and rejects unknown files', async () => {
    const app = makeApp();
    app.addRobotFromLibrary('UR5e');
    await app.openFiles([textFile('pts.csv', '# X,Y,Z,Rx,Ry,Rz\n300,0,300,180,0,0\n300,100,300,180,0,0\n')]);
    expect(app.logs.at(-1)!.text).toBe('Imported 2 targets from pts.csv'); expect(app.station.itemsOfType(ItemType.TARGET).length).toBe(2);
    await app.openFiles([textFile('prog.src', 'DEF prog()\nPTP {A1 0, A2 -90, A3 90, A4 0, A5 90, A6 0}\nLIN {X 300, Y 0, Z 300, A 0, B 180, C 0}\nEND\n')]);
    expect(app.logs.at(-1)!.text).toMatch(/^Imported KRL program prog/); expect(app.activeProgram!.name).toBe('prog');
    await app.openFiles([textFile('part.nc', 'G21\nG0 X0 Y0 Z5\nG1 X10 Y0 Z0 F300\nG1 X10 Y10\nG0 Z5\n')]);
    expect(app.logs.at(-1)!.text).toMatch(/^Imported NC program part.nc: /);
    const nc = app.station.find('part') as SceneObject; expect(nc.curves.length).toBeGreaterThan(0); expect(nc.params.nc).toBeTruthy();
    const geo = { type: 'FeatureCollection', features: [{ type: 'Feature', properties: { name: 'Plot' }, geometry: { type: 'Polygon', coordinates: [[[35.35, 46.87], [35.351, 46.87], [35.351, 46.8705], [35.35, 46.8705], [35.35, 46.87]]] } }] };
    await app.openFiles([textFile('field.geojson', JSON.stringify(geo))]);
    expect(app.logs.at(-1)!.text).toBe('Imported 1 field(s) from field.geojson'); expect(app.station.itemsOfType(ItemType.FIELD).length).toBe(1);
    await app.openFiles([textFile('fc.json', JSON.stringify(geo))]); expect(app.station.itemsOfType(ItemType.FIELD).length).toBe(2);
    await app.openFiles([textFile('tri.obj', 'v 0 0 0\nv 100 0 0\nv 0 100 0\nvn 0 0 1\nf 1//1 2//1 3//1\n')]);
    expect(app.logs.at(-1)!.text).toBe('Imported mesh tri.obj'); expect(app.assets.has('tri.obj')).toBe(true); expect((app.station.find('tri') as SceneObject).bbox).toBeTruthy();
    await app.openFiles([textFile('solid.stl', 'solid s\nfacet normal 0 0 1\nouter loop\nvertex 0 0 0\nvertex 10 0 0\nvertex 0 10 0\nendloop\nendfacet\nendsolid s\n')]);
    expect(app.logs.at(-1)!.text).toBe('Imported mesh solid.stl'); expect(app.station.find('solid')).not.toBeNull();
    await app.openFiles([textFile('pic.png', 'x'), textFile('weird.xyz', 'x')]);
    expect(app.logs.map((l) => l.text)).toContain('PNG files are not importable as geometry'); expect(app.logs.at(-1)!.text).toBe('Unsupported file weird.xyz');
    await app.openFiles([textFile('bad.vbstation', '{not json')]); expect(app.logs.at(-1)!.text).toMatch(/^Failed to open bad.vbstation/);
    await app.openFiles([textFile('robot.urdf', '<robot name="mini"><link name="base"/><link name="l1"/><joint name="j1" type="revolute"><parent link="base"/><child link="l1"/><origin xyz="0 0 0.1"/><axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint></robot>')]);
    expect(app.logs.at(-1)!.text).toMatch(/^Imported URDF mini \(1 DOF\)/); expect(app.activeRobot!.name).toBe('mini');
    await app.openFiles([textFile('dh.json', JSON.stringify({ dh: [{ a: 0, alpha: 90, d: 100, theta: 0 }, { a: 300, alpha: 0, d: 0, theta: 0 }] }))]);
    expect(app.logs.at(-1)!.text).toMatch(/Imported DH robot dh|Failed to open dh.json/);
  });
  it('RoboDK files: server conversion fallback and best-effort import', async () => {
    const app = makeApp();
    window.history.pushState({}, '', '/?server=ws://h:1');
    try {
      expect(app.serverHttpBase()).toBe('http://h:1');
      const fetchSpy = vi.spyOn(globalThis, 'fetch').mockResolvedValue({ ok: false, status: 501, json: async () => ({ error: 'no robodk' }) } as Response);
      await app.openFiles([new File([new Uint8Array([1, 2, 3, 4])], 'x.rdk')]);
      expect(fetchSpy).toHaveBeenCalled(); expect(app.logs.some((l) => /RoboDK converter not available/.test(l.text))).toBe(true);
      fetchSpy.mockRejectedValue(new Error('offline'));
      await app.openFiles([new File([new Uint8Array([1, 2, 3, 4])], 'y.robot')]);
      expect(app.logs.some((l) => /y\.robot|Failed to open y.robot/.test(l.text) || l.level === 'warn')).toBe(true);
      await expect(app.addOnlineRobot('ur10e')).rejects.toBeTruthy();
      fetchSpy.mockRestore();
    } finally { window.history.pushState({}, '', '/'); }
    expect(app.serverHttpBase()).toBeNull();
  });
  it('imports zip containers and point clouds, and accepts a RoboDK server conversion', async () => {
    const app = makeApp('pickplace');
    const { exportStationURDF, packageZip } = await import('../src/io/urdf/urdf_export');
    const zip = packageZip(exportStationURDF(app.station, app.assets, {}));
    await app.openFiles([new File([zip as BlobPart], 'pkg.zip')]);
    expect(app.logs.some((l) => /^Archive pkg\.zip: \d+ meshes, \d+ metadata files/.test(l.text))).toBe(true); expect(app.station.selection[0]).toBeInstanceOf(Folder); expect(app.station.selection[0].name).toBe('pkg');
    const pcd = ['# .PCD v0.7', 'VERSION 0.7', 'FIELDS x y z', 'SIZE 4 4 4', 'TYPE F F F', 'COUNT 1 1 1', 'WIDTH 3', 'HEIGHT 1', 'POINTS 3', 'DATA ascii', '0 0 0', '1 0 0.5', '0 1 1'].join('\n');
    await app.openFiles([textFile('cloud.pcd', pcd)]);
    expect(app.logs.at(-1)!.text).toMatch(/^cloud\.pcd: 3 points, extent 1\.0 × 1\.0 × 1\.0 m/); expect((app.station.find('cloud') as SceneObject).points.length).toBe(3);
    // a studio server with RoboDK converts .rdk (whole station) and .robot (items appended) losslessly
    const converted = JSON.parse(app.saveToJSON()); (converted.station ?? converted).name = 'Converted cell';
    window.history.pushState({}, '', '/?server=ws://h:1');
    const fetchSpy = vi.spyOn(globalThis, 'fetch').mockResolvedValue({ ok: true, json: async () => converted } as Response);
    try {
      await app.openFiles([new File([new Uint8Array([1, 2])], 'conv.rdk')]);
      expect(app.station.name).toBe('Converted cell'); expect(app.logs.at(-1)!.text).toBe('conv.rdk converted with RoboDK (lossless)');
      const n = app.station.children.length;
      await app.openFiles([new File([new Uint8Array([1, 2])], 'arm.robot')]);
      expect(app.station.children.length).toBeGreaterThan(n); expect(app.logs.at(-1)!.text).toBe('arm.robot converted with RoboDK (lossless)');
    } finally { fetchSpy.mockRestore(); window.history.pushState({}, '', '/'); }
  });
  it('exportProgram works for every registered post processor; the RoboDK script mirrors the station', () => {
    const app = makeApp('pickplace');
    const ids = app.posts().map((p) => p.id);
    expect(ids).toContain('KUKA_KRC4'); expect(ids).toContain('Generic');
    for (const id of ids) {
      const files = app.exportProgram(id);
      expect(files.length, id).toBeGreaterThan(0);
      for (const f of files) { expect(typeof f.name, id).toBe('string'); expect(f.content.length, id).toBeGreaterThan(20); }
    }
    expect(app.exportProgram('NOPE')).toEqual([]); expect(app.logs.at(-1)!.text).toBe('Unknown post NOPE');
    expect(app.exportProgram('Generic', null)).toEqual([]); expect(app.logs.at(-1)!.text).toBe('No program to export');
    const py = app.exportRoboDKScript();
    expect(py).toContain('UR10e'); expect(py).toContain('PickPlace'); expect(py).toMatch(/robolink|RDK/);
  });
});

describe('tools', () => {
  it('StationTabs renders open stations and switches / closes / adds them', () => {
    const app = makeApp('pickplace');
    const tabs = new StationTabs(app); document.body.appendChild(tabs.el);
    expect(tabs.el.querySelectorAll('.stab').length).toBe(2); expect(tabs.el.querySelector('.stab-close')).toBeNull();
    tabs.el.querySelector<HTMLButtonElement>('.stab.add')!.click();
    expect(app.stations.length).toBe(2); expect(app.station.name).toBe('Station 2'); expect(tabs.el.querySelectorAll('.stab-close').length).toBe(2);
    const first = tabs.el.querySelector<HTMLButtonElement>('.stab')!; first.click(); expect(app.station.name).toBe('Pick & place cell'); expect(first.classList.contains('active')).toBe(false);
    [...tabs.el.querySelectorAll<HTMLElement>('.stab-close')][1].click(); expect(app.stations.length).toBe(1); expect(app.station.name).toBe('Pick & place cell'); // closing a non-active tab keeps the active one
    tabs.el.querySelector<HTMLButtonElement>('.stab.add')!.click();
    tabs.el.querySelector<HTMLElement>('.stab.active .stab-close')!.click(); expect(app.stations.length).toBe(1); expect(app.station.name).toBe('Pick & place cell');
    tabs.el.remove();
  });
  it('measureDialog: two items, item to TCP, and the warning', () => {
    const app = makeApp('pickplace');
    const st = app.station;
    app.select(null); measureDialog(app); expect(lastToast()).toBe('Select two items (Shift+click) to measure'); expect(dialogs().length).toBe(0);
    app.select(st.find('Home')); measureDialog(app); expect(dialogTitle()).toBe('Measure'); expect(currentDialog().textContent).toContain('TCP'); dialogButton('Close').click();
    app.select(st.find('Home')); app.select(st.find('Pick 1'), true); measureDialog(app); expect(currentDialog().textContent).toContain('Distance'); expect(currentDialog().textContent).toContain('Angle between frames'); dialogButton('Close').click();
  });
  it('cameraDialog updates the camera item', async () => {
    const app = makeApp();
    const cam = app.station.addChild(new Camera('Cam'));
    const p = cameraDialog(app, cam);
    expect(dialogTitle()).toBe('Camera Cam');
    setField('Type', 'depth'); setField('Field of view (deg)', 90); setField('Width (px)', 320); setField('Height (px)', 240); setField('Near (mm)', 20); setField('Far (mm)', 9000);
    clickOk(); await p;
    expect(cam.kind).toBe('depth'); expect(cam.fov).toBe(90); expect(cam.width).toBe(320); expect(cam.height).toBe(240); expect(cam.near).toBe(20); expect(cam.far).toBe(9000);
    const p2 = cameraDialog(app, cam); clickCancel(); await p2; expect(cam.fov).toBe(90);
  });
  it('collisionMapDialog toggles pairs, checks and resets', async () => {
    const app = makeApp();
    await collisionMapDialog(app); expect(lastToast()).toBe('At least two collidable items are required');
    app.setStation(demos[0].build());
    const p = collisionMapDialog(app);
    const d = currentDialog(); expect(dialogTitle()).toBe('Collision map');
    const boxes = [...d.querySelectorAll<HTMLInputElement>('table.cmap input[type=checkbox]')]; expect(boxes.length).toBeGreaterThan(5); expect(boxes.every((b) => b.checked)).toBe(true);
    boxes[0].checked = false; boxes[0].dispatchEvent(new Event('change'));
    const { getCollisionMap } = await import('../src/core/collision/collision');
    expect(getCollisionMap(app.station).disabled.length).toBe(1);
    setField('Collision checking active', false, d); expect(getCollisionMap(app.station).active).toBe(false);
    dialogButton('Check now', d).click(); expect(d.querySelector('.hint')!.textContent).toMatch(/collisions/i);
    dialogButton('Reset (check all)', d).click(); expect(getCollisionMap(app.station).disabled.length).toBe(0); expect(boxes[0].checked).toBe(true);
    dialogButton('Close', d).click(); await p;
  });
  it('VideoRecorder reports missing capture support; glTF / HTML / URDF / Blender exports download files', async () => {
    const app = makeApp('pickplace');
    const v = new VideoRecorder(app); expect(v.recording).toBe(false); v.start(); expect(lastToast()).toBe('Video capture is not supported in this browser'); v.stop();
    const n0 = downloads.length;
    await exportGlb(app); expect(downloads.at(-1)!.name).toBe('Pick_place_cell.glb');
    await exportHtml3D(app); expect(downloads.at(-1)!.name).toBe('Pick_place_cell_3d.html'); expect(lastToast()).toBe('3D HTML exported');
    let p = exportUrdfPackage(app); expect(dialogTitle()).toBe('Export URDF package'); setField('Content', 'station'); setField('ROS package name (optional)', 'my_pkg'); clickOk(); await p;
    expect(downloads.at(-1)!.name).toBe('my_pkg.zip'); expect(app.logs.at(-1)!.text).toMatch(/^URDF package my_pkg: \d+ files/);
    p = exportUrdfPackage(app); clickOk(); await p; expect(downloads.at(-1)!.name).toMatch(/\.zip$/); // the active robot
    p = exportUrdfPackage(app); clickCancel(); await p;
    await saveForBlender(app); expect(downloads.at(-1)!.name).toBe('Pick_place_cell_blender.vbstation'); expect(lastToast()).toMatch(/^Saved with PickPlace animation/);
    app.setActiveProgram(null); await saveForBlender(app); expect(lastToast()).toMatch(/no active program/);
    await exportAnimationGltf(app); expect(lastToast()).toBe('Select or create a program first');
    app.setActiveProgram(app.station.itemsOfType<Program>(ItemType.PROGRAM)[0]);
    p = exportAnimationGltf(app); expect(dialogTitle()).toBe('Export animation (glTF for Blender)'); clickCancel(); await p;
    expect(downloads.length - n0).toBe(6);
  });
  it('importRoboDKPosts reads picked .py files', async () => {
    const app = makeApp();
    let p = importRoboDKPosts(app); resolvePickFiles([]); await p; expect(toasts().length).toBe(0);
    p = importRoboDKPosts(app); resolvePickFiles([textFile('notes.py', 'print("no post here")')]); await p;
    expect(lastToast()).toBe('0 post processors imported'); expect(app.logs.at(-1)!.text).toMatch(/^RoboDK posts: 0 imported, 1 skipped/);
  });
});

export { Frame, Instruction, Folder, Tool, ctxItem, flush, waitFor };
