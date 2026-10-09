/**
 * Screenshots for the group-control documentation (Group menu, examples dialog, reports with charts, fleet runtime,
 * warehouse scene). Usage: node scripts/docs-screenshots-group.mjs [outDir] (default ../docs/_static/screens); preview build on :4173.
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
page.on('pageerror', (e) => errors.push(String(e).slice(0, 200)));
const ev = (fn, arg) => page.evaluate(fn, arg);
const shot = async (name, opts = {}) => { await page.waitForTimeout(opts.wait ?? 500); await ev(() => document.querySelectorAll('.toast').forEach((x) => x.remove())); await page.screenshot({ path: path.join(OUT, `${name}.png`), clip: opts.clip }); console.log('shot', name); };
const clipOf = async (sel, pad = 8) => { const b = await (await page.waitForSelector(sel)).boundingBox(); return { x: Math.max(0, b.x - pad), y: Math.max(0, b.y - pad), width: Math.min(b.width + 2 * pad, 1440 - Math.max(0, b.x - pad)), height: Math.min(b.height + 2 * pad, 900 - Math.max(0, b.y - pad)) }; };
const openMenu = async (top) => { await page.click(`.menu-btn:has-text("${top}")`); await page.waitForTimeout(200); };
const menu = async (top, item) => { await openMenu(top); const ok = await page.evaluate((txt) => { const el = [...document.querySelectorAll('.ctx-item')].find((e) => e.textContent.includes(txt)); if (el) { el.click(); return true; } return false; }, item); if (!ok) throw new Error(`menu item ${item} not found`); await page.waitForTimeout(400); };
const load = async (q) => { await page.goto(`${BASE}?${q}&server=off`, { waitUntil: 'load' }); await page.waitForTimeout(1500); };
const tallDock = async (px = 600) => { await ev((h) => { const d = document.querySelector('.bottom-dock'); d.style.height = `${h}px`; }, px); await page.waitForTimeout(200); };
const pickModel = async (text) => { await page.click(`.ctl-item:has-text("${text}")`); await page.waitForTimeout(300); };
const analyse = async (wait = 1500) => { await page.click('.ctl-right button:has-text("Analyse"):not(:has-text("all"))'); await page.waitForTimeout(wait); };
const openAll = async () => { await ev(() => document.querySelectorAll('.ctl-report details').forEach((d) => { d.open = true; })); };
const scrollReportTo = async (text) => { await ev((t) => { const el = [...document.querySelectorAll('.ctl-report details summary')].find((s) => s.textContent.includes(t)); if (el) el.parentElement.scrollIntoView({ block: 'start' }); }, text); await page.waitForTimeout(200); };

await load('demo=none');
await ev(() => window.app.setStation(new (window.app.station.constructor)('Group control')));
await page.waitForTimeout(300);
// 62 Group menu
await openMenu('Group'); await shot('62-group-menu'); await page.keyboard.press('Escape');
// 66 examples dialog
await menu('Group', 'Course examples: group control'); await shot('66-group-examples-dialog', { clip: await clipOf('.dialog', 12) });
await page.selectOption('.dialog select', '*'); await page.click('.dialog button.primary'); await page.waitForTimeout(1500);
await tallDock(620);
// 61 consensus report (chain P4)
await pickModel('ПР1 · Chain'); await analyse(1500); await openAll(); await scrollReportTo('Communication graph'); await shot('61-group-tab-consensus');
// 63 formation
await pickModel('ПР1 · Formation'); await analyse(2500); await openAll(); await scrollReportTo('Formation "circle"'); await shot('63-pr1-formation', { clip: await clipOf('.ctl-right') });
// 64 swarm boids
await pickModel('ПР2 · Reynolds'); await analyse(2500); await openAll(); await scrollReportTo('Reynolds flock'); await shot('64-pr2-swarm', { clip: await clipOf('.ctl-right') });
// 65 allocation
await pickModel('ПР3 · Nine tasks'); await analyse(2000); await openAll(); await scrollReportTo('Sequential single-item'); await shot('65-pr3-allocation', { clip: await clipOf('.ctl-right') });
// 67 mapf
await pickModel('ПР4 · Four robots'); await analyse(2000); await openAll(); await scrollReportTo('Conflict-based search'); await shot('67-pr4-mapf', { clip: await clipOf('.ctl-right') });
// 68 coverage
await pickModel('ПР5 · Lloyd'); await analyse(2500); await openAll(); await scrollReportTo('Lloyd iteration'); await shot('68-pr5-coverage', { clip: await clipOf('.ctl-right') });
// 69 safety antipodal
await pickModel('ПР6 · Antipodal'); await analyse(4000); await openAll(); await scrollReportTo('Barrier-function filter'); await shot('69-pr6-safety', { clip: await clipOf('.ctl-right') });
// 71 warehouse report
await pickModel('ДЗ · Warehouse fleet, baseline'); await analyse(3500); await openAll(); await scrollReportTo('Fleet simulation'); await shot('71-warehouse-report', { clip: await clipOf('.ctl-right') });
// 72 warehouse scene + fleet run in 3D
await page.click('.ctl-run button:has-text("Build scene")'); await page.waitForTimeout(800);
await ev(() => { window.app.fitAll?.(); }); await page.waitForTimeout(300);
await page.click('.ctl-run button:has-text("Run on fleet")'); await page.waitForTimeout(9000);
await tallDock(360); await page.waitForTimeout(500); await shot('72-warehouse-fleet-run');
await page.click('.ctl-run button:has-text("Stop")'); await page.waitForTimeout(300);
// 73 games / 74 q-learning / 75 fuzzy / 76 resilience / 77 ca
await tallDock(620);
await pickModel('Chapter 9 · Task allocation game'); await analyse(1500); await openAll(); await shot('73-ch9-game', { clip: await clipOf('.ctl-right') });
await pickModel('Chapter 7 · Q-learning in the corridor'); await analyse(1500); await openAll(); await shot('74-ch7-qlearning', { clip: await clipOf('.ctl-right') });
await pickModel('Chapter 13 · Fuzzy'); await analyse(1500); await openAll(); await shot('75-ch13-fuzzy', { clip: await clipOf('.ctl-right') });
await pickModel('Chapter 16 · Four agents'); await analyse(1500); await openAll(); await scrollReportTo('Switched linear'); await shot('76-ch16-resilience', { clip: await clipOf('.ctl-right') });
await pickModel('Chapter 4 · Rule 90'); await analyse(1500); await openAll(); await shot('77-ch4-ca', { clip: await clipOf('.ctl-right') });
await pickModel('Chapter 12 · GA'); await analyse(2500); await openAll(); await shot('78-ch12-evo', { clip: await clipOf('.ctl-right') });
// 79 architectures report
await pickModel('Architectures · Form, move'); await analyse(3500); await openAll(); await scrollReportTo('Architectures compared'); await shot('79-architectures-report', { clip: await clipOf('.ctl-right') });
// 80 station mission on configured robots: load the Architectures scenario (robots, zones, supervisor, modes) and run
await menu('Group', 'Demo scenarios: group control'); await page.waitForTimeout(500);
await page.evaluate(() => { const rec = [...document.querySelectorAll('.nav-rec')].find((r) => /Architectures/.test(r.textContent)); rec.querySelector('button.primary').click(); }); await page.waitForTimeout(1200);
await tallDock(360); await pickModel('Architectures · Station mission'); await page.waitForTimeout(300);
await page.click('.ctl-run button:has-text("Run on fleet")'); await page.waitForTimeout(12000); await shot('80-architectures-station');
await page.click('.ctl-run button:has-text("Stop")'); await page.waitForTimeout(300); await tallDock(620);
// 70 scenarios dialog (group)
await menu('Group', 'Demo scenarios: group control'); await page.waitForTimeout(500); await shot('70-group-scenarios', { clip: await clipOf('.dialog', 12) });
await page.keyboard.press('Escape');
console.log('errors:', errors.length ? errors : 'none');
await browser.close();
