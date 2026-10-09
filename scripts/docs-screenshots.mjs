/**
 * Generates the documentation screenshots by driving the real application (preview build on :4173).
 * Usage: node scripts/docs-screenshots.mjs [outDir]   (default ../docs/_static/screens)
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
page.on('pageerror', (e) => errors.push(String(e).slice(0, 160)));
const shot = async (name, opts = {}) => { await page.waitForTimeout(opts.wait ?? 500); await page.screenshot({ path: path.join(OUT, `${name}.png`), ...(opts.clip ? { clip: opts.clip } : {}) }); console.log('shot', name); };
const openMenu = async (top) => { await page.click(`.menu-btn:has-text("${top}")`); await page.waitForTimeout(200); };
const menu = async (top, item) => { try { await openMenu(top); const ok = await page.evaluate((txt) => { const el = [...document.querySelectorAll('.ctx-item')].find((e) => e.textContent.includes(txt)); if (el) { el.click(); return true; } return false; }, item); if (!ok) throw new Error('menu item not found'); } catch (e) { console.log('menu failed', top, item, e.message.split('\n')[0]); await page.keyboard.press('Escape'); } await page.waitForTimeout(350); };
const closeDialog = async () => { await page.keyboard.press('Escape'); await page.waitForTimeout(200); };
const dialogShot = async (name) => { try { const d = await page.waitForSelector('.dialog', { timeout: 8000, state: 'attached' }); const b = await d.boundingBox(); await shot(name, { clip: { x: Math.max(0, b.x - 12), y: Math.max(0, b.y - 12), width: Math.min(b.width + 24, 1440), height: Math.min(b.height + 24, 900 - Math.max(0, b.y - 12)) } }); } catch (e) { console.log('dialogShot failed', name, e.message.split('\n')[0]); } };
const tab = async (label) => { await page.click(`.tab:has-text("${label}")`); await page.waitForTimeout(250); };
const ev = (fn, arg) => page.evaluate(fn, arg);
const fit = async () => { await page.click('text=Fit'); await page.waitForTimeout(400); };
const selectTree = async (name) => { await page.click(`.tree-name:text-is("${name}")`).catch(() => page.click(`.tree-name:has-text("${name}")`)); await page.waitForTimeout(300); };
const load = async (q) => { await page.goto(`${BASE}?${q}&server=off`, { waitUntil: 'load' }); await page.waitForTimeout(1500); };

// 1. empty station, interface, Add menu
await load('demo=none');
await ev(() => window.app.setStation(new (window.app.station.constructor)('My first station')));
await page.waitForTimeout(300);
await shot('01-empty-station');
await openMenu('Add'); await shot('02-add-menu'); await page.keyboard.press('Escape');

// 2. robot from library
await menu('Add', 'Robot from library');
await page.selectOption('.dialog select', 'UR5e');
await dialogShot('03-robot-library-dialog');
await page.click('.dialog button.primary');
await page.waitForTimeout(500);
await ev(() => { const app = window.app; const r = app.station.itemsOfType(2)[0]; const m = r.pose(); m[14] = 500; app.cmd(() => r.setPose(m)); app.select(r); });
await fit();
await shot('04-robot-added');
await menu('Add', 'Robot from online library');
await dialogShot('05-online-library-dialog');
await closeDialog();
await menu('Add', 'Reference frame');
await page.waitForTimeout(300);
await ev(() => { const app = window.app; const f = app.station.itemsOfType(3).slice(-1)[0]; const m = f.pose(); m[12] = 450; m[13] = -250; f.setPose(m); f.setName('Table'); app.select(f); });
await shot('06-frame-properties');

// 3. the finished tutorial station
await load('demo=tutorial');
fs.writeFileSync(path.join(OUT, '..', 'examples', 'first_station.vbstation'), await ev(() => window.app.saveToJSON()));
await selectTree('Part');
await fit();
await shot('07-objects-added');
await selectTree('Gripper');
await shot('08-tool-properties');
await ev(() => { const app = window.app; const t = app.station.find('Pick', 6); app.setActiveRobot(app.station.itemsOfType(2)[0]); app.moveRobotTo(t, true); app.select(t); });
await page.waitForTimeout(400);
await shot('09-targets-taught');
await tab('Program');
await shot('10-program-editor');
await menu('Program', 'Validate (compile)');
await page.waitForTimeout(700);
await shot('11-validate');
await tab('Simulation');
await ev(() => { window.app.previewProgram(); window.app.seekProgram(window.app.sim.duration * 0.42); });
await page.waitForTimeout(500);
await shot('12-simulation-timeline');
await menu('Program', 'Export with post processor');
await page.waitForTimeout(900);
await dialogShot('13-export-post-dialog');
await closeDialog();
await menu('Robot', 'Check collisions now');
await page.waitForTimeout(500);
await tab('Log');
await shot('14-collision-check-log');
await menu('Tools', 'Collision map');
await dialogShot('15-collision-map');
await closeDialog();

// 4. machining on the tutorial table
const nc = `%\nG21 G90 G17\nT1 M6\nS12000 M3\nG0 X0 Y0 Z10\nG1 Z-2 F300\nG1 X60 F800\nG1 Y40\nG2 X50 Y50 I-10 J0\nG1 X0\nG1 Y0\nG0 Z10\nM5\nM30\n%`;
await ev(async (nc) => { const app = window.app; await app.openFiles([new File([nc], 'pocket.nc')]); const o = app.station.itemsOfType(5).find((x) => x.name === 'pocket'); const m = o.poseAbs(); m[12] = 620; m[13] = -100; m[14] = 430; o.setPoseAbs(m); app.select(o); }, nc);
await page.waitForTimeout(400);
await menu('Robot', 'Robot machining project');
await dialogShot('16-machining-dialog');
await page.click('.dialog button.primary');
await page.waitForTimeout(2500);
await tab('Program');
await ev(() => { window.app.seekProgram(window.app.sim.duration * 0.5); });
await fit();
await shot('17-machining-program');
await menu('Robot', 'Follow curve');
await dialogShot('18-curve-follow-dialog').catch(() => console.log('no curve dialog'));
await closeDialog();
await tab('Console');
await page.fill('.console-in', "const r = RDK.Item('', ITEM_TYPE_ROBOT); print(r.Name(), r.Joints()); RDK.ItemList(ITEM_TYPE_TARGET).length");
await page.keyboard.press('Enter');
await page.waitForTimeout(400);
await shot('19-console');
await menu('Tools', 'Export animation as glTF');
await dialogShot('20-export-animation-dialog');
await closeDialog();
await menu('File', 'Export URDF package');
await dialogShot('21-export-urdf-dialog');
await closeDialog();
await menu('Robot', 'Move with external axes').catch(() => {});
await page.keyboard.press('Escape');

// 5. demos
await load('demo=welding'); await tab('Program'); await shot('22-welding-cell');
await load('demo=packing'); await ev(() => window.app.startWorld()); await page.waitForTimeout(6000); await tab('Process'); await shot('23-packing-line');
await load('demo=orchard'); await ev(() => window.app.startWorld()); await page.waitForTimeout(8000); await tab('Fleet'); await shot('24-orchard-fleet');
await ev(() => window.app.pauseWorld()); await page.waitForTimeout(500);
await menu('Agriculture', 'Create field / orchard'); await dialogShot('25-field-dialog'); await closeDialog();
await menu('Agriculture', 'Create mission'); await dialogShot('26-mission-dialog').catch(() => {}); await closeDialog();
await menu('Mobile & Fleet', 'Add mobile robot'); await dialogShot('27-mobile-robot-dialog'); await closeDialog();
await menu('Mobile & Fleet', 'Create fleet'); await dialogShot('28-fleet-dialog').catch(() => {}); await closeDialog();
await load('demo=greenhouse'); await ev(() => window.app.startWorld()); await page.waitForTimeout(5000); await shot('29-greenhouse');
await load('demo=pickplace');
await tab('Camera');
await ev(() => { try { const app = window.app; const RDK = app.bottom?.rdk; } catch {} });
await shot('30-pickplace-camera');
// machine vision stack: camera on a mobile robot in the orchard, wizard, Vision tab
await load('demo=orchard'); await ev(() => window.app.pauseWorld());
await ev(() => { const app = window.app; app.select(app.station.itemsOfType(100)[0]); });
await menu('Add', 'Camera / vision sensor'); await page.waitForTimeout(800);
// park the platform in the alley beside row 2 and turn the camera 55° towards the canopy
await ev(() => {
  const app = window.app; const r = app.station.itemsOfType(100)[0]; const row = app.station.itemsOfType(108)[1]; const f = row.parent; const abs = f.poseAbs();
  const [lx, ly] = row.pointAt(row.length() / 2, -1750); const [dx, dy] = row.direction();
  r.setPose2D(abs[0] * lx + abs[4] * ly + abs[12], abs[1] * lx + abs[5] * ly + abs[13], (Math.atan2(dy, dx) * 180) / Math.PI);
  const cam = app.station.itemsOfType(19)[0]; const m = cam.pose(); const a = (55 * Math.PI) / 180, c = Math.cos(a), sn = Math.sin(a);
  const out = new Float64Array(16); for (let col = 0; col < 4; col++) { const x = m[col * 4], y = m[col * 4 + 1]; out[col * 4] = c * x - sn * y; out[col * 4 + 1] = sn * x + c * y; out[col * 4 + 2] = m[col * 4 + 2]; out[col * 4 + 3] = m[col * 4 + 3]; }
  cam.setPose(out); app.select(cam);
});
await page.waitForTimeout(500);
await menu('Tools', 'Machine vision stack'); await page.waitForTimeout(600); await dialogShot('36-vision-wizard');
await page.click('.dialog button:has-text("Apply stack")').catch(() => {}); await page.waitForTimeout(800);
await ev(() => window.app.bottom.show('vision')); await page.waitForTimeout(300);
await page.click('.vision-panel button:has-text("Run")').catch(() => {}); await page.waitForTimeout(2500);
await shot('37-vision-tab');
await menu('Connect', 'ROS 2 via rosbridge'); await dialogShot('31-ros2-dialog'); await closeDialog();
await menu('Connect', 'VDA 5050'); await page.waitForTimeout(400); if (await page.$('.dialog')) { await dialogShot('32-vda5050-dialog'); await closeDialog(); } else console.log('vda dialog needs server (toast shown)');
await menu('Help', 'Demo scenarios'); await page.waitForSelector('.dialog', { timeout: 40000, state: 'attached' }).catch(() => {}); await dialogShot('38-scenarios-dialog'); await closeDialog();
await menu('View', 'Language'); await page.waitForTimeout(700); await shot('33-russian-ui');
console.log('errors:', errors.slice(0, 8));
await browser.close();
