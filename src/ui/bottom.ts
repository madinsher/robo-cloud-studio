import { App } from '../app';
import { ProgramEditor } from './program-editor';
import { h, clear, fmt, icon } from './dom';
import { ItemType } from '../core/items/item';
import { FleetItem } from '../fleet/fleet';
import { MobileRobot } from '../mobile/items';
import { MissionItem } from '../agri/items';
import { Camera as CameraItem, Item } from '../core/items/item';
import { detectFruit } from '../agri/vision';
import { Robolink, robomath, RobolinkItem, Mat } from '../api/robolink';
import * as RobolinkConsts from '../api/robolink';
import { t } from './i18n';
import { buildNavPanel } from './navstack_ui';
import { buildVisionPanel } from './vision_ui';
import { buildControlPanel } from './control_ui';

/** Bottom dock: Program | Simulation | Fleet | Process | Navigation | Vision | Control | Camera | Console | Log. */
export class BottomPanel {
  el: HTMLElement;
  private tabs: Record<string, HTMLElement> = {};
  private navPanel!: { el: HTMLElement; render: () => void };
  private visionPanel!: { el: HTMLElement; render: () => void };
  private controlPanel!: { el: HTMLElement; render: () => void };
  private active = 'program';
  private programEditor: ProgramEditor;
  private timelineRange!: HTMLInputElement;
  private timeLabel!: HTMLElement;
  private consoleOut!: HTMLElement;
  private logOut!: HTMLElement;
  private fleetBody!: HTMLElement;
  private processBody!: HTMLElement;
  private history: string[] = [];
  private histIdx = -1;

  constructor(readonly app: App) {
    this.programEditor = new ProgramEditor(app);
    this.el = h('div', { class: 'bottom-dock' });
    const tabBar = h('div', { class: 'tab-bar' });
    const content = h('div', { class: 'tab-content' });
    const defs: Array<[string, string, HTMLElement]> = [
      ['program', 'Program', this.programEditor.el],
      ['sim', 'Simulation', this.buildSim()],
      ['fleet', 'Fleet', (this.fleetBody = h('div', { class: 'pad' }))],
      ['process', 'Process', (this.processBody = h('div', { class: 'pad' }))],
      ['nav', 'Navigation', (this.navPanel = buildNavPanel(app)).el],
      ['vision', 'Vision', (this.visionPanel = buildVisionPanel(app)).el],
      ['control', 'Control', (this.controlPanel = buildControlPanel(app)).el],
      ['camera', 'Camera', this.buildCamera()],
      ['console', 'Console (RoboDK API)', this.buildConsole()],
      ['log', 'Log', (this.logOut = h('div', { class: 'log-out' }))],
    ];
    for (const [id, label, el] of defs) {
      const tab = h('button', { class: `tab ${id === this.active ? 'active' : ''}`, onClick: () => this.show(id) }, t(label));
      this.tabs[id] = tab;
      tabBar.appendChild(tab);
      el.classList.add('tab-page');
      el.dataset.tab = id;
      el.style.display = id === this.active ? '' : 'none';
      content.appendChild(el);
    }
    this.el.append(tabBar, content);
    app.events.on('simulation', ({ time, duration }) => { this.timelineRange.max = String(duration); this.timelineRange.value = String(time); this.timeLabel.textContent = `${fmt(time, 2)} / ${fmt(duration, 2)} s`; });
    app.events.on('log', ({ text, level }) => { this.logOut.appendChild(h('div', { class: `log-${level}` }, `${new Date().toLocaleTimeString()}  ${text}`)); this.logOut.scrollTop = this.logOut.scrollHeight; });
    setInterval(() => { if (this.active === 'fleet') this.renderFleet(); if (this.active === 'process') this.renderProcess(); if (this.active === 'nav') this.navPanel.render(); }, 500);
  }

  show(id: string): void {
    this.active = id;
    for (const [k, tab] of Object.entries(this.tabs)) tab.classList.toggle('active', k === id);
    this.el.querySelectorAll<HTMLElement>('.tab-page').forEach((p) => { p.style.display = p.dataset.tab === id ? '' : 'none'; });
    if (id === 'fleet') this.renderFleet();
    if (id === 'process') this.renderProcess();
    if (id === 'nav') this.navPanel.render();
    if (id === 'vision') this.visionPanel.render();
    if (id === 'control') this.controlPanel.render();
  }

  private buildSim(): HTMLElement {
    this.timelineRange = h('input', { type: 'range', min: 0, max: 1, step: 0.01, value: 0, class: 'timeline' }) as HTMLInputElement;
    this.timelineRange.addEventListener('input', () => { const t = parseFloat(this.timelineRange.value); this.app.pauseProgram(); this.app.seekProgram(t); });
    this.timeLabel = h('span', { class: 'time-label' }, '0.00 / 0.00 s');
    const speed = h('select', { onChange: (e: Event) => { this.app.simSpeed = parseFloat((e.target as HTMLSelectElement).value); } }, ...[0.25, 0.5, 1, 2, 5, 10, 50, 100].map((s) => h('option', { value: s, selected: s === 1 }, `${s}×`)));
    const worldState = h('span', { class: 'hint' });
    setInterval(() => { worldState.textContent = `World clock ${fmt(this.app.worldTime, 1)} s ${this.app.worldRunning ? '(running)' : '(paused)'}`; }, 300);
    return h('div', { class: 'pad sim-panel' },
      h('div', { class: 'btn-row' },
        h('b', null, 'Program: '),
        h('button', { class: 'btn primary', onClick: () => this.app.runProgram() }, '▶ Run'),
        h('button', { class: 'btn', onClick: () => this.app.pauseProgram() }, '⏸ Pause'),
        h('button', { class: 'btn', onClick: () => this.app.stopProgram() }, '⏹ Stop'),
        h('button', { class: 'btn', onClick: () => this.app.seekProgram(this.app.sim.duration) }, '⏭ End'),
        h('label', null, ' Speed ', speed), this.timeLabel),
      this.timelineRange,
      h('div', { class: 'btn-row' },
        h('b', null, 'World (fleet & process): '),
        h('button', { class: 'btn primary', onClick: () => this.app.startWorld() }, '▶ Start'),
        h('button', { class: 'btn', onClick: () => this.app.pauseWorld() }, '⏸ Pause'),
        h('button', { class: 'btn', onClick: () => this.app.resetWorld() }, '↺ Reset'), worldState),
      h('div', { class: 'hint' }, 'Program playback drives robot arms along compiled trajectories. The world clock runs fleets (mobile robots, tasks, charging) and process components (feeders, conveyors, machines).'));
  }

  private renderFleet(): void {
    const body = this.fleetBody;
    clear(body);
    const fleets = this.app.station.itemsOfType<FleetItem>(ItemType.FLEET);
    if (!fleets.length) { body.appendChild(h('div', { class: 'hint' }, 'No fleet. Use Mobile > Create fleet…')); return; }
    for (const f of fleets) {
      const fm = this.app.fleetManager(f);
      const k = fm.kpis();
      const robots = fm.robots();
      body.appendChild(h('div', { class: 'fleet-head' }, h('b', null, f.name), ` — ${robots.length} robots · done ${k.tasksDone} · pending ${k.tasksPending} · ${k.throughputPerHour.toFixed(1)} tasks/h · utilisation ${(k.utilization * 100).toFixed(0)} % · ${(k.distanceTravelled / 1000).toFixed(0)} m · ${k.energyUsedWh.toFixed(0)} Wh${k.fruitPicked ? ` · harvested ${k.fruitPicked} fruit (${k.yieldKg.toFixed(0)} kg)` : ''}${k.areaWorkedM2 ? ` · ${(k.areaWorkedM2 / 10000).toFixed(2)} ha worked` : ''}`));
      const table = h('table', { class: 'grid' }, h('thead', null, h('tr', null, ...['Robot', 'Status', 'Battery', 'Task', 'Speed', 'Odometer', 'Busy'].map((c) => h('th', null, c)))),
        h('tbody', null, ...robots.map((r: MobileRobot) => { const pr = k.perRobot[r.id]; return h('tr', { onClick: () => this.app.select(r) }, h('td', null, r.name), h('td', { class: `st-${r.state.status}` }, r.state.status), h('td', null, this.bar(r.batteryLevel())), h('td', null, r.state.taskId ?? '—'), h('td', null, `${fmt(r.state.v / 1000, 2)} m/s`), h('td', null, `${fmt(r.state.odometer / 1000, 0)} m`), h('td', null, `${fmt((pr?.busyTime ?? 0) / Math.max(1, fm.time) * 100, 0)} %`)); })));
      body.appendChild(table);
      const tasks = f.tasks.slice(-40).reverse();
      body.appendChild(h('table', { class: 'grid small' }, h('thead', null, h('tr', null, ...['Task', 'Type', 'Status', 'Robot', 'Row', 'Wait', 'Duration'].map((c) => h('th', null, c)))),
        h('tbody', null, ...tasks.map((t) => { const rb = t.robotId ? this.app.station.findById(t.robotId) : null; return h('tr', { class: `task-${t.status}` }, h('td', null, t.id), h('td', null, t.type), h('td', null, t.status), h('td', null, rb?.name ?? '—'), h('td', null, t.meta?.rowId ? (this.app.station.findById(t.meta.rowId)?.name ?? '') + (t.meta.side ? (t.meta.side > 0 ? ' R' : ' L') : '') : '—'), h('td', null, t.startedAt !== undefined ? `${fmt(t.startedAt - t.createdAt, 0)} s` : '—'), h('td', null, t.finishedAt !== undefined ? `${fmt(t.finishedAt - (t.startedAt ?? t.createdAt), 0)} s` : '—')); }))));
      const missions = this.app.station.itemsOfType<MissionItem>(ItemType.MISSION).filter((m) => !m.fleetId || m.fleetId === f.id);
      if (missions.length) body.appendChild(h('div', null, ...missions.map((m) => h('div', { class: 'mission-row' }, h('span', null, `${m.name} (${m.missionType}) — ${m.status}`), h('progress', { value: m.progress, max: 1 })))));
    }
  }

  /** Small canvas chart: cumulative output (green) and WIP (blue) over process time. */
  private chart(): HTMLElement {
    const c = h('canvas', { width: 600, height: 90, class: 'mini-chart' }) as HTMLCanvasElement;
    const ctx = c.getContext('2d')!;
    const hist = this.procHistory;
    if (hist.length < 2) return c;
    const t0 = hist[0].t, t1 = hist[hist.length - 1].t || 1;
    const maxOut = Math.max(1, ...hist.map((p) => p.out)), maxWip = Math.max(1, ...hist.map((p) => p.wip));
    ctx.fillStyle = '#1b1f26'; ctx.fillRect(0, 0, c.width, c.height);
    const line = (key: 'out' | 'wip', max: number, color: string) => {
      ctx.strokeStyle = color; ctx.lineWidth = 1.5; ctx.beginPath();
      hist.forEach((p, i) => { const x = ((p.t - t0) / (t1 - t0 || 1)) * (c.width - 10) + 5; const y = c.height - 5 - (p[key] / max) * (c.height - 10); if (i) ctx.lineTo(x, y); else ctx.moveTo(x, y); });
      ctx.stroke();
    };
    line('out', maxOut, '#51cf66');
    line('wip', maxWip, '#4dabf7');
    ctx.fillStyle = '#9aa4b2'; ctx.font = '11px sans-serif';
    ctx.fillText(`output (max ${maxOut})`, 8, 12); ctx.fillStyle = '#4dabf7'; ctx.fillText(`WIP (max ${maxWip})`, 120, 12);
    return c;
  }

  private bar(v: number): HTMLElement {
    return h('div', { class: 'bar' }, h('div', { class: `bar-fill ${v < 0.25 ? 'low' : ''}`, style: { width: `${Math.round(v * 100)}%` } }), h('span', null, `${Math.round(v * 100)}%`));
  }

  private procHistory: Array<{ t: number; out: number; wip: number }> = [];
  private renderProcess(): void {
    const body = this.processBody;
    clear(body);
    const stats = this.app.processSim.statistics();
    if (!stats.length) { body.appendChild(h('div', { class: 'hint' }, 'No process components. Use Add > Process component… (feeder → conveyor → machine → sink).')); return; }
    const totalOut = stats.filter((s) => s.type === 'sink').reduce((a, s) => a + s.exited, 0);
    const tNow = this.app.processSim.time;
    if (!this.procHistory.length || tNow < this.procHistory[this.procHistory.length - 1].t) this.procHistory = [];
    if (!this.procHistory.length || tNow - this.procHistory[this.procHistory.length - 1].t >= 5) this.procHistory.push({ t: tNow, out: totalOut, wip: this.app.processSim.products.size });
    if (this.procHistory.length > 400) this.procHistory.shift();
    body.appendChild(h('div', null, `Process clock ${fmt(tNow, 1)} s · products in system ${this.app.processSim.products.size} · output ${totalOut} (${tNow > 0 ? fmt((totalOut * 3600) / tNow, 0) : '0'} /h)`));
    body.appendChild(this.chart());
    body.appendChild(h('table', { class: 'grid' }, h('thead', null, h('tr', null, ...['Component', 'Type', 'In', 'Out', 'WIP', 'Utilisation', 'Blocked', 'Failures'].map((c) => h('th', null, c)))),
      h('tbody', null, ...stats.map((s) => h('tr', { onClick: () => { const it = this.app.station.findById(s.id); if (it) this.app.select(it); } }, h('td', null, s.name), h('td', null, s.type), h('td', null, String(s.entered)), h('td', null, String(s.exited)), h('td', null, String(s.wip)), h('td', null, this.bar(s.utilization)), h('td', null, `${fmt(s.blocked * 100, 0)} %`), h('td', null, String(s.failures)))))));
    const signals = [...this.app.processSim.signals.entries()];
    if (signals.length) body.appendChild(h('div', { class: 'hint' }, 'Signals: ', signals.map(([k, v]) => `${k}=${v}`).join(', ')));
  }

  private camCanvas!: HTMLCanvasElement;
  private camInfo!: HTMLElement;
  private buildCamera(): HTMLElement {
    this.camCanvas = h('canvas', { width: 640, height: 400, class: 'cam-canvas' }) as HTMLCanvasElement;
    this.camInfo = h('div', { class: 'hint' }, 'Select a camera item (or any item: the view looks along its +Z axis). Add > Camera on a tool to simulate an eye-in-hand camera.');
    const render = () => {
      const sel = this.app.station.selection[0] as Item | undefined;
      const cam = sel instanceof CameraItem ? sel : sel ?? this.app.station.itemsOfType<CameraItem>(ItemType.CAMERA)[0];
      if (!cam) { this.camInfo.textContent = 'No camera / item selected.'; return; }
      const fov = cam instanceof CameraItem ? cam.fov : 60;
      this.app.renderer.renderFromItem(cam, this.camCanvas, fov, cam instanceof CameraItem ? cam.near : 10, cam instanceof CameraItem ? cam.far : 50000, cam instanceof CameraItem && cam.kind === 'depth');
      let txt = `View from ${cam.name}`;
      if (cam instanceof CameraItem) {
        const det = detectFruit(this.app.station, cam, { onlyRipe: false, maxRange: 6000 });
        const ctx = this.camCanvas.getContext('2d')!;
        const sx = this.camCanvas.width / cam.width, sy = this.camCanvas.height / cam.height;
        ctx.lineWidth = 2;
        for (const d of det) { ctx.strokeStyle = d.ripe >= 0.5 ? '#ff4d4d' : '#ffd43b'; ctx.strokeRect((d.u - d.w / 2) * sx, (d.v - d.h / 2) * sy, Math.max(4, d.w * sx), Math.max(4, d.h * sy)); }
        txt += ` — ${det.length} fruit detections (${det.filter((d) => d.ripe >= 0.5).length} ripe)`;
      }
      this.camInfo.textContent = txt;
    };
    const live = h('input', { type: 'checkbox' }) as HTMLInputElement;
    setInterval(() => { if (this.active === 'camera' && live.checked) render(); }, 500);
    return h('div', { class: 'pad cam-panel' }, h('div', { class: 'btn-row' }, h('button', { class: 'btn primary', onClick: render }, 'Render view'), h('label', null, live, ' live'), h('button', { class: 'btn', onClick: () => { const a = document.createElement('a'); a.href = this.camCanvas.toDataURL('image/png'); a.download = 'camera.png'; a.click(); } }, 'Save PNG')), this.camCanvas, this.camInfo);
  }

  private buildConsole(): HTMLElement {
    this.consoleOut = h('div', { class: 'console-out' });
    const input = h('textarea', { class: 'console-in', rows: 3, placeholder: 'JavaScript with the RoboDK-compatible API. Example:\nconst robot = RDK.Item("", ITEM_TYPE_ROBOT); robot.MoveJ([0,-90,90,0,90,0]); print(robot.Pose())' }) as HTMLTextAreaElement;
    const run = async () => {
      const code = input.value.trim();
      if (!code) return;
      this.history.push(code); this.histIdx = this.history.length;
      this.consoleOut.appendChild(h('div', { class: 'c-in' }, '› ' + code));
      try {
        const result = await this.evalScript(code);
        if (result !== undefined) this.consoleOut.appendChild(h('div', { class: 'c-out' }, formatValue(result)));
      } catch (e: any) {
        this.consoleOut.appendChild(h('div', { class: 'c-err' }, String(e?.stack ?? e)));
      }
      this.app.snapshot();
      this.app.previewProgram();
      this.consoleOut.scrollTop = this.consoleOut.scrollHeight;
    };
    input.addEventListener('keydown', (e) => {
      if (e.key === 'Enter' && (e.ctrlKey || e.metaKey)) { e.preventDefault(); run(); }
      if (e.key === 'ArrowUp' && e.altKey && this.history.length) { this.histIdx = Math.max(0, this.histIdx - 1); input.value = this.history[this.histIdx]; }
      if (e.key === 'ArrowDown' && e.altKey && this.history.length) { this.histIdx = Math.min(this.history.length, this.histIdx + 1); input.value = this.history[this.histIdx] ?? ''; }
    });
    const examples: Record<string, string> = {
      'List items': `for (const it of RDK.ItemList()) print(it.Name(), '(' + it.Type() + ')')`,
      'Move robot & teach': `const robot = RDK.Item('', ITEM_TYPE_ROBOT);\nconst frame = RDK.AddFrame('Frame API');\nrobot.setPoseFrame(frame);\nconst prog = RDK.AddProgram('API program', robot);\nfor (let i = 0; i < 5; i++) {\n  const q = robot.Joints(); q[0] += 15; robot.setJoints(q);\n  const t = RDK.AddTarget('T' + i, frame, robot);\n  prog.MoveJ(t);\n}\nprint(prog.Update())`,
      'Square with MoveL': `const robot = RDK.Item('', ITEM_TYPE_ROBOT);\nconst p0 = robot.Pose();\nconst prog = RDK.AddProgram('Square', robot);\nprog.MoveJ(robot.Joints());\nfor (const [dx, dy] of [[100,0],[100,100],[0,100],[0,0]]) prog.MoveL(p0.mul(transl(dx, dy, 0)));\nprint(prog.Update())`,
      'Generate KRL': `const prog = RDK.Item('', ITEM_TYPE_PROGRAM);\nconst [ok, code] = prog.MakeProgram('', 'KUKA_KRC4');\nprint(code)`,
      'Fleet KPIs': `for (const f of app.station.itemsOfType(102)) print(app.fleetManager(f).kpis())`,
    };
    const ex = h('select', { onChange: (e: Event) => { const k = (e.target as HTMLSelectElement).value; if (examples[k]) input.value = examples[k]; (e.target as HTMLSelectElement).value = ''; } }, h('option', { value: '' }, 'examples…'), ...Object.keys(examples).map((k) => h('option', { value: k }, k)));
    return h('div', { class: 'console' }, this.consoleOut, h('div', { class: 'console-bar' }, input, h('div', { class: 'console-btns' }, h('button', { class: 'btn primary', onClick: run }, 'Run (Ctrl+Enter)'), ex, h('button', { class: 'btn', onClick: () => clear(this.consoleOut) }, 'Clear'))));
  }

  /** Evaluate user script with RDK globals. */
  private async evalScript(code: string): Promise<any> {
    const RDK = new Robolink(this.app.station);
    RDK.onRender = () => {};
    RDK.onMessage = (m) => this.consoleOut.appendChild(h('div', { class: 'c-out' }, m));
    RDK.onRunProgram = (p) => this.app.runProgram(p);
    RDK.assetsRegister = (id, positions) => this.app.assets.registerMesh(id, { positions, normals: new Float32Array(0), min: [0, 0, 0], max: [0, 0, 0], triangles: positions.length / 9 });
    const print = (...args: any[]) => this.consoleOut.appendChild(h('div', { class: 'c-out' }, args.map(formatValue).join(' ')));
    const scope: Record<string, any> = { RDK, print, app: this.app, RobolinkItem, Robolink, ...robomath };
    for (const k of Object.keys(RobolinkConsts)) if (/^[A-Z_0-9]+$/.test(k)) scope[k] = (RobolinkConsts as any)[k];
    const names = Object.keys(scope);
    const vals = names.map((k) => scope[k]);
    const body = /\breturn\b|;|\n/.test(code) ? code : `return (${code})`;
    // eslint-disable-next-line no-new-func
    const fn = new Function(...names, `"use strict"; return (async () => { ${body} })();`);
    return fn(...vals);
  }
}

function formatValue(v: any): string {
  if (v instanceof Mat) return v.toString();
  if (v instanceof RobolinkItem) return `Item(${v.Name()})`;
  if (typeof v === 'object') { try { return JSON.stringify(v, (_k, x) => (x instanceof Float64Array ? Array.from(x) : x), 1); } catch { return String(v); } }
  return String(v);
}
export { icon };
