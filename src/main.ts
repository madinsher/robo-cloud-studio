import './ui/styles.css';
import { App } from './app';
import { TreePanel } from './ui/tree';
import { PropertiesPanel } from './ui/properties';
import { BottomPanel } from './ui/bottom';
import { MenuBar } from './ui/menu';
import { h, toast, contextMenu } from './ui/dom';
import { itemContextMenu } from './ui/dialogs';
import { demos } from './demos';
import { Robolink, robomath } from './api/robolink';
import { connectToStudioServer } from './api/ws-client';
import { StationTabs, measureDialog } from './ui/tools';

const app = new App();
const viewport = h('div', { class: 'viewport' });
const bottom = new BottomPanel(app);
(app as any).bottom = bottom;
const menu = new MenuBar(app, () => { bottom.el.classList.toggle('collapsed'); setTimeout(() => app.renderer.resize(), 50); });
const tree = new TreePanel(app);
const tabs = new StationTabs(app);
const props = new PropertiesPanel(app);

const toolbar = h('div', { class: 'toolbar' },
  h('button', { class: 'tb', title: 'Fit all (F)', onClick: () => app.renderer.fitAll() }, '⤢ Fit'),
  h('button', { class: 'tb', title: 'Translate gizmo (T)', onClick: () => app.renderer.setGizmoMode('translate') }, '✥'),
  h('button', { class: 'tb', title: 'Rotate gizmo (R)', onClick: () => app.renderer.setGizmoMode('rotate') }, '⟳'),
  h('span', { class: 'tb-sep' }),
  h('button', { class: 'tb', title: 'Teach MoveJ (J)', onClick: () => app.teach('MoveJ') }, '+J'),
  h('button', { class: 'tb', title: 'Teach MoveL (L)', onClick: () => app.teach('MoveL') }, '+L'),
  h('button', { class: 'tb primary', title: 'Run program (F5)', onClick: () => app.runProgram() }, '▶'),
  h('button', { class: 'tb', title: 'Stop', onClick: () => app.stopProgram() }, '⏹'),
  h('span', { class: 'tb-sep' }),
  h('button', { class: 'tb', title: 'Start world (fleet/process)', onClick: () => app.startWorld() }, '🌍 ▶'),
  h('button', { class: 'tb', title: 'Pause world', onClick: () => app.pauseWorld() }, '🌍 ⏸'),
  h('span', { class: 'tb-sep' }),
  h('button', { class: 'tb', title: 'Undo (Ctrl+Z)', onClick: () => app.undo() }, '↶'),
  h('button', { class: 'tb', title: 'Redo (Ctrl+Y)', onClick: () => app.redo() }, '↷'),
);
const status = h('div', { class: 'statusbar' }, 'Ready');

const layout = h('div', { class: 'layout' },
  menu.el,
  h('div', { class: 'main' },
    tree.el,
    h('div', { class: 'center' }, tabs.el, toolbar, viewport, bottom.el),
    props.el),
  status);
document.body.appendChild(layout);
app.mount(viewport);

// Drag & drop files
viewport.addEventListener('dragover', (e) => { e.preventDefault(); viewport.classList.add('drop'); });
viewport.addEventListener('dragleave', () => viewport.classList.remove('drop'));
viewport.addEventListener('drop', async (e) => { e.preventDefault(); viewport.classList.remove('drop'); const files = [...(e.dataTransfer?.files ?? [])]; if (files.length) await app.openFiles(files); });
viewport.addEventListener('contextmenu', (e) => { e.preventDefault(); const it = app.renderer.pick(e.clientX, e.clientY) ?? app.station.selection[0] ?? app.station; if (it !== app.station.selection[0]) app.select(it); contextMenu(e.clientX, e.clientY, itemContextMenu(app, it)); });

// Keyboard shortcuts
window.addEventListener('keydown', (e) => {
  const tag = (e.target as HTMLElement).tagName;
  if (tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT') return;
  const k = e.key.toLowerCase();
  if ((e.ctrlKey || e.metaKey) && k === 'z') { e.preventDefault(); app.undo(); }
  else if ((e.ctrlKey || e.metaKey) && (k === 'y' || (k === 'z' && e.shiftKey))) { e.preventDefault(); app.redo(); }
  else if ((e.ctrlKey || e.metaKey) && k === 's') { e.preventDefault(); menu.save(); }
  else if ((e.ctrlKey || e.metaKey) && k === 'o') { e.preventDefault(); import('./ui/dialogs').then((d) => d.importDialog(app)); }
  else if ((e.ctrlKey || e.metaKey) && k === 'n') { e.preventDefault(); app.newStation(); }
  else if (k === 'delete' || k === 'backspace') { if (app.station.selection.length) app.deleteItems(app.station.selection); }
  else if (k === 'f') app.renderer.fitAll();
  else if (k === 't') app.renderer.setGizmoMode('translate');
  else if (k === 'r') app.renderer.setGizmoMode('rotate');
  else if (k === 'j') app.teach('MoveJ');
  else if (k === 'l') app.teach('MoveL');
  else if (k === 'f5') { e.preventDefault(); app.runProgram(); }
  else if (k === ' ') { e.preventDefault(); if (app.sim.playing) app.pauseProgram(); else app.runProgram(); }
  else if (k === 'escape') app.select(null);
  else if (k === 'm') measureDialog(app);
});

// Status bar
setInterval(() => {
  const r = app.activeRobot;
  const sel = app.station.selection[0];
  const parts = [`${app.station.name}`, sel ? `sel: ${sel.name}` : '', r ? `robot: ${r.name} [${r.joints().map((v) => v.toFixed(1)).join(', ')}]` : '', app.activeProgram ? `program: ${app.activeProgram.name}` : '', app.worldRunning ? `world ${app.worldTime.toFixed(0)} s` : ''];
  status.textContent = parts.filter(Boolean).join('   ·   ');
}, 250);

// Expose the API for scripting from the browser devtools and for the server bridge
const RDK = new Robolink(app.station);
app.events.on('stationChanged', () => { RDK.station = app.station; });
RDK.onRunProgram = (p) => app.runProgram(p);
RDK.onRender = () => {};
RDK.onMessage = (m, popup) => (popup ? toast(m, 'info', 6000) : app.log(m));
RDK.assets = app.assets;
RDK.onSnapshot = (cam, w, h) => { const c = document.createElement('canvas'); c.width = w; c.height = h; app.renderer.renderFromItem(cam, c, (cam as any).fov ?? 60); return c.toDataURL('image/png'); };
RDK.onViewPose = { get: () => app.renderer.getViewPose(), set: (m) => app.renderer.setViewPose(m) };
RDK.onStations = { list: () => app.stations, setActive: (s) => app.setStation(s), add: (n) => app.addStation(n), close: () => app.closeStation() };
RDK.onWindow = (cmd, value) => { if (cmd === 'state' && value === -1) bottom.el.classList.add('collapsed'); if (cmd === 'state' && value === 2) bottom.el.classList.remove('collapsed'); if (cmd === 'close') app.newStation(); };
RDK.onShowSequence = (r, rows) => app.renderer.showSequence(r, rows);
RDK.onInteractiveMode = (mode) => { if (mode === 2 || mode === 5) app.renderer.setGizmoMode(mode === 2 ? 'rotate' : 'translate'); };
RDK.simTime = { get: () => app.sim.time, set: (t) => app.seekProgram(t) };
RDK.onPluginLoad = (name) => { import(/* @vite-ignore */ name).then((mod) => { const plugin = mod.default ?? mod; RDK.registerPlugin(name, plugin); plugin.onLoad?.(app, RDK); app.log(`Plugin ${name} loaded`); }).catch((e) => app.log(`Plugin ${name} failed: ${e.message}`, 'error')); return true; };
app.onStationEvent = (type, itemId, data) => RDK.events.push({ type, itemId, data });
app.renderer.onAnimate((dt) => { if (app.sim.playing || app.worldRunning) RDK.spray.step(dt * app.simSpeed); });
(window as any).app = app;
(window as any).RDK = RDK;
(window as any).robomath = robomath;

// Optional WebSocket bridge to the studio server (python clients)
const params = new URLSearchParams(location.search);
const wsUrl = params.get('server') ?? (location.hostname ? `ws://${location.hostname}:20500` : '');
if (wsUrl && params.get('server') !== 'off') connectToStudioServer(app, RDK, wsUrl).catch(() => {});

// Initial station
const demoId = params.get('demo') ?? 'pickplace';
const demo = demos.find((d) => d.id === demoId) ?? demos[0];
app.setStation(demo.build());
app.log(`Loaded demo: ${demo.name}`);
