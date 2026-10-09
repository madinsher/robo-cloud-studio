/**
 * Screenshots for the control-design documentation (Control tab, course examples, mission runtime).
 * Usage: node scripts/docs-screenshots-control.mjs [outDir]   (default ../docs/_static/screens); preview build on :4173.
 */
import { chromium } from 'playwright';
import fs from 'node:fs';
import path from 'node:path';
const OUT = process.argv[2] ?? path.resolve('../docs/_static/screens');
fs.mkdirSync(OUT, { recursive: true });
const BASE = 'http://127.0.0.1:4173/';
const browser = await chromium.launch({ executablePath: '/opt/pw-browsers/chromium-1194/chrome-linux/chrome', args: ['--use-gl=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'] });
const page = await browser.newPage({ viewport: { width: 1440, height: 900 } });
const errors = [];
const ev = (fn, arg) => page.evaluate(fn, arg);
const saveSvg = async (name) => {
  const svgs = await ev(() => window.app.controlPanel.graphSvgs()); if (!svgs.length) { console.log('no svg for', name); return; }
  fs.writeFileSync(path.join(OUT, `${name}.svg`), svgs[0]); console.log('svg', name);
  const p2 = await browser.newPage({ viewport: { width: 1200, height: 800 }, deviceScaleFactor: 2 });
  await p2.setContent(`<html><body style="margin:0;background:#2c323d">${svgs[0]}</body></html>`);
  const b = await (await p2.waitForSelector('svg')).boundingBox();
  await p2.screenshot({ path: path.join(OUT, `${name}.png`), clip: { x: 0, y: 0, width: Math.min(1200, Math.ceil(b.width)), height: Math.min(800, Math.ceil(b.height)) } });
  await p2.close();
};
page.on('pageerror', (e) => errors.push(String(e).slice(0, 160)));
const shot = async (name, opts = {}) => { await page.waitForTimeout(opts.wait ?? 500); await ev(() => document.querySelectorAll('.toast').forEach((x) => x.remove())); await page.screenshot({ path: path.join(OUT, `${name}.png`), ...(opts.clip ? { clip: opts.clip } : {}) }); console.log('shot', name); };
const clipOf = async (sel, pad = 8) => { const b = await (await page.waitForSelector(sel)).boundingBox(); return { x: Math.max(0, b.x - pad), y: Math.max(0, b.y - pad), width: Math.min(b.width + 2 * pad, 1440 - Math.max(0, b.x - pad)), height: Math.min(b.height + 2 * pad, 900 - Math.max(0, b.y - pad)) }; };
const openMenu = async (top) => { await page.click(`.menu-btn:has-text("${top}")`); await page.waitForTimeout(200); };
const menu = async (top, item) => { await openMenu(top); const ok = await page.evaluate((txt) => { const el = [...document.querySelectorAll('.ctx-item')].find((e) => e.textContent.includes(txt)); if (el) { el.click(); return true; } return false; }, item); if (!ok) { console.log('menu item not found', top, item); await page.keyboard.press('Escape'); } await page.waitForTimeout(400); };
const dialogShot = async (name) => { await shot(name, { clip: await clipOf('.dialog', 12) }); };
const load = async (q) => { await page.goto(`${BASE}?${q}&server=off`, { waitUntil: 'load' }); await page.waitForTimeout(1500); };
const tallDock = async (px = 560) => { await ev((h) => { const d = document.querySelector('.bottom-dock'); d.style.height = `${h}px`; const e = document.querySelector('.ctl-editor'); if (e) e.style.flex = '0 0 36%'; window.dispatchEvent(new Event('resize')); }, px); await page.waitForTimeout(300); };
const pickModel = async (text) => { await page.click(`.ctl-item:has-text("${text}")`); await page.waitForTimeout(300); };
const analyse = async (wait = 1500) => { await page.click('.ctl-right button:has-text("Analyse"):not(:has-text("all"))'); await page.waitForTimeout(wait); };
const openAll = async () => { await ev(() => document.querySelectorAll('.ctl-report details').forEach((d) => { d.open = true; })); };

await load('demo=none');
await ev(() => window.app.setStation(new (window.app.station.constructor)('Control design')));
await page.waitForTimeout(300);
// 40 Control menu
await openMenu('Control'); await shot('40-control-menu'); await page.keyboard.press('Escape');
// 41 course examples dialog
await menu('Control', 'Course examples'); await dialogShot('41-course-examples-dialog');
await page.selectOption('.dialog select', '*'); await page.click('.dialog button.primary'); await page.waitForTimeout(800);
await tallDock(600);
// 42 DES report (example A)
await pickModel('A · DES plant'); await analyse(2500); await openAll(); await shot('42-control-tab-des');
await ev(() => { const m = window.app.station.itemsOfType(111).find((x) => x.kind === 'des'); const src = m.source.replace(/\n(automaton|spec)[\s\S]*$/, ''); return src; });
// a small automaton for the graph view: the template
await ev(() => { const { app } = window; const it = app.station.itemsOfType(111).find((x) => x.name === 'A · DES plant + specifications E1 E2 E3 E5'); void it; });
// 43 E4 unrealisable
await pickModel('never drop'); await analyse(2500); await openAll(); await shot('43-des-e4-unrealisable', { clip: await clipOf('.ctl-right') });
// 44 Petri net with graph
await pickModel('B · Cell Petri net'); await analyse(1500); await openAll(); await shot('44-petri-report', { clip: await clipOf('.ctl-right') }); await saveSvg('ctl-graph-petri');
await pickModel('B · Cell as an S'); await analyse(1500); await openAll(); await shot('44b-s3pr-report', { clip: await clipOf('.ctl-right') });
// 45 behavior tree
await pickModel('A · Mission behavior tree'); await page.waitForTimeout(3500); await openAll(); await shot('45-bt-report', { clip: await clipOf('.ctl-right') }); await saveSvg('ctl-graph-bt');
// 46 SMV counterexample
await pickModel('B · Shared zone: r2 starves'); await analyse(2500); await openAll(); await shot('46-smv-counterexample', { clip: await clipOf('.ctl-right') });
// 47 GR(1)
await pickModel('A · GR(1)'); await analyse(3000); await openAll(); await shot('47-gr1-synthesis', { clip: await clipOf('.ctl-right') });
await pickModel('A · Mode automaton'); await analyse(1500); await openAll(); await shot('47b-modes', { clip: await clipOf('.ctl-right') }); await saveSvg('ctl-graph-modes');
await pickModel('A · Classification POMDP'); await analyse(1500); await openAll(); await shot('47c-pomdp', { clip: await clipOf('.ctl-right') });
await pickModel('A · Onboard task set'); await analyse(1500); await openAll(); await shot('47d-realtime', { clip: await clipOf('.ctl-right') });
await pickModel('A · Fault tree'); await analyse(1500); await openAll(); await shot('47e-fta', { clip: await clipOf('.ctl-right') });
await pickModel('B · Bottleneck'); await analyse(1500); await openAll(); await shot('47f-perf', { clip: await clipOf('.ctl-right') });
await pickModel('B · Corridor'); await analyse(1500); await openAll(); await shot('47g-mapf', { clip: await clipOf('.ctl-right') });
await pickModel('B · Acceptance'); await analyse(1500); await openAll(); await shot('47h-acceptance', { clip: await clipOf('.ctl-right') });
// 48 PDDL
await pickModel('A · PDDL'); await analyse(3000); await openAll(); await shot('48-pddl-plan', { clip: await clipOf('.ctl-right') });
// 49 job shop
await pickModel('B · Job shop'); await analyse(3000); await openAll(); await shot('49-jobshop', { clip: await clipOf('.ctl-right') });
// 50 reliability / safety
await pickModel('B · Reliability'); await analyse(2000); await openAll(); await shot('50-reliability-safety', { clip: await clipOf('.ctl-right') });
// 51 MDP
await pickModel('A · Grasp strategy MDP'); await analyse(1500); await openAll(); await shot('51-mdp', { clip: await clipOf('.ctl-right') });
// 52 New model dialog
await menu('Control', 'New control model'); await dialogShot('52-new-model-dialog'); await page.keyboard.press('Escape');
// 53 tree + properties of a control model
await ev(() => { const m = window.app.station.itemsOfType(111)[0]; window.app.select(m); }); await page.waitForTimeout(400);
await shot('53-control-model-properties', { clip: await clipOf('.props-panel') });

// 57-59 graphical editors
await tallDock(620);
await pickModel('A · DES plant'); await page.click('.ctl-view button:has-text("Diagram")'); await page.waitForTimeout(300); await page.click('.ctl-view button:has-text("⛶")'); await page.waitForTimeout(600);
await ev(() => window.app.controlPanel.graph.select({ type: 'node', id: 'mT' })); await page.waitForTimeout(300);
await shot('57-editor-automaton');
await page.click('.ctl-view button:has-text("⛶")'); await page.waitForTimeout(200);
await pickModel('B · Cell Petri net'); await page.waitForTimeout(300); await page.click('.ctl-view button:has-text("⛶")'); await page.waitForTimeout(600);
await ev(() => window.app.controlPanel.graph.select({ type: 'node', id: 't1a' })); await page.waitForTimeout(300);
await shot('58-editor-petri', { clip: await clipOf('.ctl-right') });
await ev(() => { const g = window.app.controlPanel.graph; const st = g.state(); st.petri.actions.push({ target: 't1a', kind: 'program', value: 'Load M1', robot: 'UR10e', args: {} }); g.select({ type: 'node', id: 't1a' }); }); await page.waitForTimeout(300);
await shot('58b-editor-petri-action', { clip: await clipOf('.ctl-right') });
await ev(() => document.querySelector('.ge-action')?.scrollIntoView()); await page.waitForTimeout(200); await shot('59-action-panel', { clip: await clipOf('.ge-action', 10) });
await page.click('.ctl-view button:has-text("⛶")'); await page.waitForTimeout(200);
await pickModel('A · Mission behavior tree'); await page.waitForTimeout(800); await page.click('.ctl-view button:has-text("⛶")'); await page.waitForTimeout(600);
await ev(() => { const g = window.app.controlPanel.btGraph; const st = g.state(); const leaves = []; const rec = (n) => { if (n.type === 'action' && n.fn === 'grasp') leaves.push(n); (n.children ?? []).forEach(rec); }; rec(st.doc.root); g.select(leaves[0] ?? null); }); await page.waitForTimeout(300);
await shot('60-editor-bt');
await page.click('.ctl-view button:has-text("⛶")'); await page.waitForTimeout(200);
await page.click('.ctl-view button:has-text("Text")'); await page.waitForTimeout(300);
await tallDock(600);
// 54-55 mission runtime scenario
await menu('Help', 'Demo scenarios');
await page.selectOption('.dialog select', 'control'); await page.waitForTimeout(300);
await dialogShot('54-control-scenarios');
await ev(() => { const card = [...document.querySelectorAll('.dialog .btn.primary')].find((b) => b.closest('div')?.parentElement?.textContent.includes('Mission runtime')); card?.click(); });
await page.waitForTimeout(800); await page.keyboard.press('Escape');
await tallDock(300);
await ev(() => { window.app.renderer.fitAll(); });
await pickModel('behavior tree');
await page.click('button:has-text("Run mission")');
await page.waitForTimeout(9000);
await shot('55-mission-runtime');
for (let i = 0; i < 60; i++) { await page.waitForTimeout(1000); const st = await ev(() => document.querySelector('.ctl-status')?.textContent); if (st?.startsWith('⏹')) break; }
await shot('56-mission-done', { clip: await clipOf('.ctl-right') });
console.log('errors', errors);
await browser.close();
