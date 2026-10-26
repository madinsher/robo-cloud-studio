import { App } from '../app';
import { MobileRobot } from '../mobile/items';
import { h, contextMenu, MenuEntry, downloadText, toast, dialog, pickFolder } from './dom';
import { robotLibraryDialog, onlineLibraryDialog, mobileRobotDialog, orchardDialog, fleetDialog, missionDialog, mapDialog, zoneDialog, componentDialog, exportDialog, importDialog, harvestArmDialog, curveFollowDialog, machiningDialog, railIKDialog, vdaDialog, importFolderDialog } from './dialogs';
import { ItemType, Folder } from '../core/items/item';
import { demos } from '../demos';
import { t, getLang, setLang } from './i18n';
import { collisionMapDialog, measureDialog, cameraDialog, VideoRecorder, exportHtml3D, exportGlb, exportAnimationGltf, exportUrdfPackage, exportMoveItPackage, importRoboDKPosts, saveForBlender } from './tools';
import { Camera as CameraItem } from '../core/items/item';
import { Robot } from '../core/items/robot';
import { murkaMenu } from './murka_ui';
import { needsRetailLayout, loadRetailLayoutFiles } from '../ros/murka_layout';

export class MenuBar {
  el: HTMLElement;
  video: VideoRecorder;
  constructor(readonly app: App, readonly onToggleBottom: () => void) {
    this.video = new VideoRecorder(app);
    this.el = h('div', { class: 'menubar' });
    const menus: Array<[string, () => MenuEntry[]]> = [
      ['File', () => [
        { label: 'New station', shortcut: 'Ctrl+N', action: () => app.newStation() },
        { label: 'Open / import…', shortcut: 'Ctrl+O', action: () => importDialog(app) },
        { label: 'Open folder (ROS package / Webots project)…', action: () => importFolderDialog(app) },
        { label: 'Save station (.vbstation)', shortcut: 'Ctrl+S', action: () => this.save() },
        { separator: true },
        { label: 'Demo stations', children: demos.map((d) => ({ label: d.name, action: () => { app.setStation(d.build()); toast(d.description, 'info', 6000); } })) },
        { label: 'MURKA examples', children: murkaMenu(app) },
        { separator: true },
        { label: 'Export program (post processor)…', action: () => exportDialog(app) },
        { label: 'Export station as RoboDK API script (.py)', action: () => downloadText(`${app.station.name.replace(/\W+/g, '_')}_robodk.py`, app.exportRoboDKScript()) },
        { label: 'Export URDF package (robot / station, zip)…', action: () => exportUrdfPackage(app) },
        { label: 'Export MoveIt 2 config (robot, zip)…', action: () => exportMoveItPackage(app) },
        { label: 'Save for Blender add-on (.vbstation + animation)', action: () => saveForBlender(app) },
        { label: 'Export station JSON (for rdk_export.py / server)', action: () => downloadText(`${app.station.name.replace(/\W+/g, '_')}.vbstation`, app.saveToJSON(), 'application/json') },
        { label: 'Export screenshot (PNG)', action: () => { const a = document.createElement('a'); a.href = app.renderer.screenshot(); a.download = 'station.png'; a.click(); } },
        { label: 'Files saved by scripts…', action: () => this.savedFilesDialog() },
      ]],
      ['Edit', () => [
        { label: 'Undo', shortcut: 'Ctrl+Z', action: () => app.undo() },
        { label: 'Redo', shortcut: 'Ctrl+Y', action: () => app.redo() },
        { separator: true },
        { label: 'Delete selection', shortcut: 'Del', action: () => app.deleteItems(app.station.selection) },
        { label: 'Rename station…', action: async () => { const r = await dialog<{ n: string }>('Station name', [{ key: 'n', label: 'Name', type: 'text', value: app.station.name }]); if (r?.n) app.cmd(() => app.station.setName(r.n)); } },
      ]],
      ['Add', () => [
        { label: 'Robot from library…', action: () => robotLibraryDialog(app) },
        { label: 'Robot from online library (ROS-Industrial)…', action: () => onlineLibraryDialog(app) },
        { label: 'Reference frame', action: () => app.addFrame() },
        { label: 'Target (teach)', action: () => app.addTarget() },
        { label: 'Program', action: () => app.addProgram() },
        { label: 'Folder', action: () => app.cmd(() => app.station.addChild(new Folder('Folder'))) },
        { separator: true },
        { label: 'Box', action: () => app.addObjectPrimitive('box') },
        { label: 'Cylinder', action: () => app.addObjectPrimitive('cylinder', [150, 150, 600]) },
        { label: 'Sphere', action: () => app.addObjectPrimitive('sphere', [200]) },
        { separator: true },
        { label: 'Camera / vision sensor', action: async () => { const { addCamera } = await import('./vision_ui'); addCamera(app); } },
        { label: 'Process component (VC-style)…', action: () => componentDialog(app) },
        { label: 'Import URDF / STL / program / .rdk…', action: () => importDialog(app) },
      ]],
      ['Program', () => [
        { label: 'New program', action: () => app.addProgram() },
        { label: 'Teach MoveJ', shortcut: 'J', action: () => app.teach('MoveJ') },
        { label: 'Teach MoveL', shortcut: 'L', action: () => app.teach('MoveL') },
        { separator: true },
        { label: 'Run', shortcut: 'F5', action: () => app.runProgram() },
        { label: 'Pause', action: () => app.pauseProgram() },
        { label: 'Resume', action: () => app.resumeProgram() },
        { label: 'Stop', action: () => app.stopProgram() },
        { label: 'Reset simulation (restore home)', action: () => app.resetSimulation() },
        { label: 'Validate (compile)', action: () => { const r = app.validateProgram(); if (r) toast(r.ok ? `OK · ${r.duration.toFixed(2)} s` : r.problems.map((p) => p.message).join('; '), r.ok ? 'ok' : 'error', 6000); } },
        { separator: true },
        { label: 'Export with post processor…', action: () => exportDialog(app) },
        { label: 'Import RoboDK post processors (.py)…', action: () => importRoboDKPosts(app) },
      ]],
      ['Robot', () => [
        { label: 'Home active robot', action: () => { const r = app.activeRobot; if (r) app.cmd(() => r.setJoints(r.jointsHome())); } },
        { label: 'Add tool to active robot', action: () => { const r = app.activeRobot; if (!r) return toast('Select a robot', 'warn'); app.cmd(() => { const { Tool } = require_items(); const t = new Tool(`Tool ${r.tools().length + 1}`); r.addChild(t); r.setTool(t); app.select(t); }); } },
        { label: 'Fruit picking program for active robot…', action: () => { const r = app.activeRobot; if (r) harvestArmDialog(app, r); } },
        { label: 'Follow curve / points of an object…', action: () => { const r = app.activeRobot; if (r) curveFollowDialog(app, r); } },
        { label: 'Robot machining project (NC / G-code / 3D print)…', action: () => { const r = app.activeRobot; if (r) machiningDialog(app, r); else toast('Select a robot', 'warn'); } },
        { label: 'Move with external axes (rail / gantry)…', action: () => { const r = app.activeRobot; if (r) railIKDialog(app, r); } },
        { separator: true },
        { label: 'Check collisions now (static)', action: () => app.checkStationCollisions() },
        { label: 'Check collisions during program validation', checked: app.checkCollisions, action: () => { app.checkCollisions = !app.checkCollisions; app.validateProgram(); } },
        { separator: true },
        ...app.station.itemsOfType<Robot>(ItemType.ROBOT).map((r) => ({ label: r.name, checked: r === app.activeRobot, action: () => { app.setActiveRobot(r); app.select(r); } })),
      ]],
      ['Mobile & Fleet', () => [
        { label: 'Add mobile robot…', action: () => mobileRobotDialog(app) },
        { label: 'Create fleet…', action: () => fleetDialog(app) },
        { label: 'Occupancy map…', action: () => mapDialog(app) },
        { label: 'Zone (charging / no-go)…', action: () => zoneDialog(app) },
        { separator: true },
        { label: 'Navigation & SLAM stack…', action: async () => { const m = app.station.selection.find((i) => i instanceof MobileRobot) as MobileRobot | undefined ?? app.station.itemsOfType<MobileRobot>(ItemType.MOBILE_ROBOT)[0]; if (!m) return toast('Add a mobile robot first', 'warn'); const { navStackDialog } = await import('./navstack_ui'); navStackDialog(app, m); } },
        { label: 'Export ROS 2 navigation package (Nav2 + SLAM)…', action: async () => { const m = app.station.selection.find((i) => i instanceof MobileRobot) as MobileRobot | undefined ?? app.station.itemsOfType<MobileRobot>(ItemType.MOBILE_ROBOT)[0]; if (!m) return toast('Add a mobile robot first', 'warn'); const { exportNavPackage } = await import('./navstack_ui'); exportNavPackage(app, m); } },
        { label: 'Play scripted fleet (Webots supervisor script, MURKA Mission B)…', action: () => this.scriptedFleetDialog() },
        { separator: true },
        { label: 'Start world simulation', action: () => app.startWorld() },
        { label: 'Pause world', action: () => app.pauseWorld() },
        { label: 'Reset world', action: () => app.resetWorld() },
      ]],
      ['Agriculture', () => [
        { label: 'Create field / orchard…', action: () => orchardDialog(app) },
        { label: 'Create mission…', action: () => missionDialog(app) },
        { label: 'Import field from GeoJSON…', action: () => importDialog(app) },
        { separator: true },
        { label: 'Demo: apple orchard with harvesting fleet', action: () => { const d = demos.find((x) => x.id === 'orchard'); if (d) app.setStation(d.build()); } },
        { label: 'Demo: greenhouse tomato with rail robots', action: () => { const d = demos.find((x) => x.id === 'greenhouse'); if (d) app.setStation(d.build()); } },
      ]],
      ['Control', () => [
        { label: 'New control model…', action: async () => { const { newModelDialog } = await import('./control_ui'); newModelDialog(app); } },
        { label: 'Course examples (A: mobile manipulator, B: production cell)…', action: async () => { const { courseExamplesDialog } = await import('./control_ui'); courseExamplesDialog(app); } },
        { label: 'Open Control tab', action: () => { (app as any).bottom?.show?.('control'); } },
        { separator: true },
        { label: 'Analyse all models (deadlocks, livelocks, specifications, synthesis)', action: async () => { const { analyseAll } = await import('./control_ui'); analyseAll(app); } },
        { label: 'Run mission (selected behavior tree on the selected robot)', action: () => { (app as any).bottom?.show?.('control'); (app as any).controlPanel?.start?.(); } },
        { label: 'Stop mission', action: () => { (app as any).controlPanel?.stop?.(); } },
        { separator: true },
        { label: 'Demo scenarios: control design…', action: async () => { const { scenariosDialog } = await import('./scenarios_ui'); scenariosDialog(app); } },
      ]],
      ['Group', () => [
        { label: 'New group-control model…', action: async () => { const { newModelDialog } = await import('./control_ui'); newModelDialog(app, 'consensus'); } },
        { label: 'Course examples: group control (ПР1–ПР6, warehouse homework, chapters)…', action: async () => { const { groupExamplesDialog } = await import('./control_ui'); groupExamplesDialog(app); } },
        { label: 'Open Control tab', action: () => { (app as any).bottom?.show?.('control'); } },
        { separator: true },
        { label: 'Run selected model on the fleet (consensus, swarm, coverage, safety, MAPF, warehouse)', action: () => { (app as any).bottom?.show?.('control'); (app as any).controlPanel?.start?.(); } },
        { label: 'Stop fleet run', action: () => { (app as any).controlPanel?.stop?.(); } },
        { label: 'Build scene from the selected group model', action: () => { (app as any).controlPanel?.buildScene?.(); } },
        { separator: true },
        { label: 'Demo scenarios: group control…', action: async () => { const { scenariosDialog } = await import('./scenarios_ui'); scenariosDialog(app, 'group'); } },
      ]],
      ['Tools', () => [
        { label: 'Collision map…', action: () => collisionMapDialog(app) },
        { label: 'Measure (two selected items / item to TCP)', shortcut: 'M', action: () => measureDialog(app) },
        { label: 'Camera parameters…', action: () => { const c = app.station.selection.find((i) => i instanceof CameraItem) as CameraItem | undefined; if (c) cameraDialog(app, c); else toast('Select a camera item', 'warn'); } },
        { label: 'Machine vision stack (camera)…', action: async () => { const { visionStackDialog, camerasOf, addCamera } = await import('./vision_ui'); const c = app.station.selection.find((i) => i instanceof CameraItem) as CameraItem | undefined ?? camerasOf(app)[0] ?? addCamera(app); visionStackDialog(app, c); } },
        { label: 'Run vision pipeline (Vision tab)', action: () => { (app as any).bottom?.show?.('vision'); } },
        { label: 'Export ROS 2 perception package (YOLO / tracker / VLM / VLA)…', action: async () => { const { exportVisionPackage, camerasOf } = await import('./vision_ui'); const c = app.station.selection.find((i) => i instanceof CameraItem) as CameraItem | undefined ?? camerasOf(app)[0]; if (!c) return toast('Add a camera first', 'warn'); exportVisionPackage(app, c); } },
        { separator: true },
        { label: this.video.recording ? 'Stop video recording' : 'Record video of the 3D view (WebM)…', action: () => (this.video.recording ? this.video.stop() : this.video.start()) },
        { label: 'Export 3D HTML (self-contained viewer)', action: () => exportHtml3D(app) },
        { label: 'Export scene as glTF (.glb)', action: () => exportGlb(app) },
        { label: 'Export animation as glTF (Blender)…', action: () => exportAnimationGltf(app) },
        { separator: true },
        { label: 'New station tab', action: () => app.addStation(`Station ${app.stations.length + 1}`) },
        { label: 'Close station tab', action: () => app.closeStation() },
      ]],
      ['Connect', () => [
        { label: 'ROS 2 via rosbridge…', action: () => this.rosDialog() },
        { label: 'Disconnect rosbridge', action: () => { (app as any).ros?.disconnect(); (app as any).ros = null; } },
        { separator: true },
        { label: 'Studio server (Python RoboDK API clients)…', action: () => this.serverInfo() },
        { separator: true },
        { label: 'VDA 5050 fleet interface (AGV / AMR, KUKA Fleet, MiR)…', action: () => vdaDialog(app) },
        { label: 'Disconnect VDA 5050', action: async () => { const c = (app as any).vda; if (c) { await c.disconnect(); (app as any).vda = null; toast('VDA 5050 disconnected', 'info'); } } },
      ]],
      ['View', () => [
        { label: 'Fit all', shortcut: 'F', action: () => app.renderer.fitAll() },
        { label: 'Top', action: () => app.renderer.setView('top') },
        { label: 'Front', action: () => app.renderer.setView('front') },
        { label: 'Side', action: () => app.renderer.setView('side') },
        { label: 'Isometric', action: () => app.renderer.setView('iso') },
        { separator: true },
        { label: 'Show reference frames', checked: app.renderer.showFrames, action: () => { app.renderer.showFrames = !app.renderer.showFrames; } },
        { label: 'Show targets', checked: app.renderer.showTargets, action: () => { app.renderer.showTargets = !app.renderer.showTargets; } },
        { label: 'Gizmo: translate', shortcut: 'T', action: () => app.renderer.setGizmoMode('translate') },
        { label: 'Gizmo: rotate', shortcut: 'R', action: () => app.renderer.setGizmoMode('rotate') },
        { label: 'Toggle bottom panel', action: () => onToggleBottom() },
        { label: 'Show robot reach', checked: app.renderer.showReach, action: () => { app.renderer.showReach = !app.renderer.showReach; app.renderer.rebuildAll(); } },
        { separator: true },
        { label: 'Language: Русский', action: () => { setLang(getLang() === 'ru' ? 'en' : 'ru'); location.reload(); } },
      ]],
      ['Help', () => [
        { label: 'Demo scenarios (navigation, vision, control design)…', action: async () => { const { scenariosDialog } = await import('./scenarios_ui'); scenariosDialog(app); } },
        { label: 'Quick start', action: () => this.help() },
        { label: 'RoboDK API compatibility notes', action: () => window.open('docs/robodk-compatibility.md', '_blank') },
        { label: 'About', action: () => toast('VerticalBot Studio — browser-native robot simulation & offline programming (RoboDK / Visual Components class) for industrial, mobile and agricultural robotics. MIT.', 'info', 8000) },
      ]],
    ];
    for (const [label, build] of menus) {
      const b = h('button', { class: 'menu-btn' }, t(label));
      b.addEventListener('click', () => { const r = b.getBoundingClientRect(); contextMenu(r.left, r.bottom, build()); });
      this.el.appendChild(b);
    }
    this.el.appendChild(h('span', { class: 'menu-title' }, 'VerticalBot Studio'));
  }

  async rosDialog(): Promise<void> {
    // a MURKA world opened without murom_world/config (the Webots supervisor reads products.yaml + locations.yaml from
    // $MUROM_CONFIG_DIR): offer to load the layout, otherwise /murom/sim/objects would be empty
    let layoutBox: HTMLElement | null = null;
    if (needsRetailLayout(this.app.station)) {
      const status = h('p', { class: 'hint' }, 'This MURKA world has no products: the Webots supervisor spawns them from murom_world/config (products.yaml + locations.yaml), which was not in the opened folder. Load that folder before connecting as the simulator backend, or /murom/sim/objects stays empty.');
      const btn = h('button', { class: 'btn', onClick: async () => {
        const files = (await pickFolder()).filter((f) => /^(products|locations)\.ya?ml$/i.test(f.name));
        const map = new Map<string, string>();
        for (const f of files) map.set(((f as any).webkitRelativePath || f.name) as string, await f.text());
        const res = await loadRetailLayoutFiles(this.app.station, map);
        this.app.log(`Retail layout ${res.pair ? `(${res.pair.products}, ${res.pair.locations})` : ''}: ${res.report.join('; ')}`, res.products ? 'info' : 'warn');
        status.textContent = res.products ? `Retail layout loaded: ${res.products} products (${res.pair!.products.replace(/\/[^/]*$/, '')}).` : `No layout loaded: ${res.report.join('; ')}`;
        if (res.products) btn.setAttribute('disabled', '');
      } }, 'Load retail layout (murom_world/config)…') as HTMLButtonElement;
      layoutBox = h('div', null, status, btn);
    }
    const r = await dialog<{ url: string; mode: any; rate: number }>('Connect to ROS 2 (rosbridge_server)', [
      { key: 'url', label: 'rosbridge websocket URL', type: 'text', value: 'ws://localhost:9090' },
      { key: 'mode', label: 'Mode', type: 'select', value: 'both', options: [{ value: 'both', label: 'Publish sim state + follow /joint_states' }, { value: 'publish', label: 'Publish only (drive real robots)' }, { value: 'follow', label: 'Follow only (digital twin of real robots)' }, { value: 'sim', label: 'Simulator backend (the studio replaces Webots / Gazebo: /clock, /odom, /scan, cameras, /cmd_vel)' }] },
      { key: 'rate', label: 'Publish rate (Hz)', type: 'number', value: 10 },
    ], { body: h('div', null,
      h('p', { class: 'hint' }, 'Publishes /cmd_joint_state, /cmd_point, /tcp_pose per arm and /cmd_vel, /robot_pose, /battery_state per mobile robot (namespaced). Subscribes to /joint_states and /odom. Run: ros2 launch rosbridge_server rosbridge_websocket_launch.xml'),
      h('p', { class: 'hint' }, 'Simulator backend: the studio plays the role of the robot simulator for ROS 2 nodes (MURKA Webots interface) — /clock, /odom + TF, /joint_states, /scan, /imu, camera images + camera_info, depth; it follows /cmd_vel and joint trajectories. On a MURKA station it runs in a background worker at real time (software-rendered camera images), independent of the 3D view.'),
      layoutBox) });
    if (!r) return;
    const { RosBridge } = await import('../ros/rosbridge');
    try {
      const rb = new RosBridge(this.app, { url: r.url, mode: r.mode, rate: r.rate });
      await rb.connect();
      (this.app as any).ros = rb;
    } catch (e: any) { toast(e.message, 'error'); }
  }

  /** Play a scripted fleet (agromarca scripted_fleet JSON) on the world clock; default: MURKA Mission B. */
  async scriptedFleetDialog(): Promise<void> { const { scriptedFleetDialog } = await import('./murka_ui'); return scriptedFleetDialog(this.app); }

  /** Files written by API calls / station scripts in this browser session (Cam2D_Snapshot, MakeProgram, Save, CSV). */
  savedFilesDialog(): void {
    const fs = (window as any).savedFiles as { list(): Array<{ path: string; data: Uint8Array; time: number }> } | undefined;
    const files = fs?.list() ?? [];
    const save = (f: { path: string; data: Uint8Array }) => {
      const url = URL.createObjectURL(new Blob([f.data as BlobPart]));
      const a = document.createElement('a'); a.href = url; a.download = f.path.replace(/^.*\//, ''); a.click();
      setTimeout(() => URL.revokeObjectURL(url), 1000);
    };
    const body = h('div', { class: 'help' },
      h('p', null, files.length ? `${files.length} file(s) written by API calls and station scripts in this session (they are kept in memory, like the files RoboDK writes to disk).` : 'No file was written by API calls or station scripts in this session.'),
      ...files.slice(-200).reverse().map((f) => h('div', { class: 'btn-row' }, h('button', { class: 'btn', onClick: () => save(f) }, 'Download'), h('span', null, ` ${f.path} (${(f.data.length / 1024).toFixed(1)} KB, ${new Date(f.time).toLocaleTimeString()})`))));
    dialog('Files saved by scripts', [], { body, width: 720 });
  }

  serverInfo(): void {
    const body = h('div', { class: 'help' },
      h('p', null, 'The studio server exposes the RoboDK-compatible API over WebSocket (port 20500) and relays calls to this browser tab.'),
      h('pre', { class: 'code-preview' }, `cd studio\nnpm run server              # ws://localhost:20500\n# open the studio with ?server=ws://localhost:20500\npython python/examples/hello_studio.py`),
      h('p', null, 'Python: put studio/python on PYTHONPATH — `from robodk.robolink import *` then works without RoboDK installed. Existing RoboDK scripts run unchanged for the supported API subset (see docs/robodk-compatibility.md).'),
      h('p', null, 'The page connects only when asked: open it with ?server=ws://host:port (or ?server=auto), or connect now:'),
      h('div', { class: 'btn-row' }, h('button', { class: 'btn primary', onClick: () => { const c = (window as any).connectStudioServer as ((u?: string) => Promise<unknown>) | undefined; c?.().then(() => toast('Connected to the studio server', 'ok')).catch((e: any) => toast(`Studio server not reachable: ${e?.message ?? e}`, 'warn')); } }, `Connect to ws://${location.hostname || 'localhost'}:20500`)));
    dialog('Studio server', [], { body, width: 640 });
  }

  save(): void {
    downloadText(`${this.app.station.name.replace(/\W+/g, '_') || 'station'}.vbstation`, this.app.saveToJSON(), 'application/json');
    toast('Station saved', 'ok');
  }

  help(): void {
    const body = h('div', { class: 'help' },
      h('p', null, h('b', null, 'Quick start'), ' — full manual: ', h('a', { href: 'https://vertical-bot-ros.readthedocs.io', target: '_blank' }, 'vertical-bot-ros.readthedocs.io'), '. Load File › Demo stations › Tutorial for the guided example.'),
      h('ol', null,
        h('li', null, 'Add > Robot from library, or Add > Robot from online library (Fanuc/ABB/KUKA/Motoman/Stäubli/UR/… with meshes), or drag a URDF + STL files onto the 3D view.'),
        h('li', null, 'Move the robot with the joint sliders or the cartesian jog in Properties; press J / L to teach MoveJ / MoveL into the active program.'),
        h('li', null, 'Run the program (F5) and export it with a post processor (KUKA, ABB, Fanuc, UR, Motoman, Stäubli, Doosan, Mecademic, ROS 2, RoboDK API script).'),
        h('li', null, 'Agriculture > Create field / orchard, Mobile & Fleet > Create fleet, then Agriculture > Create mission and start the world simulation.'),
        h('li', null, 'Console tab: script the station with the RoboDK-compatible API (RDK.Item, AddTarget, MoveJ, MakeProgram…). The same API is served to Python over WebSocket by the studio server (see server/ and python/).')),
      h('p', null, h('b', null, 'Mouse: '), 'left drag orbit · middle/right pan · wheel zoom · click select · double-click focus / move robot to target · Shift+click multi-select · gizmo moves the selected item (T translate, R rotate).'));
    dialog('Help', [], { body, width: 640 });
  }
}
import * as items from '../core/items/item';
function require_items() { return items; }
