import { chromium } from 'playwright';
const shots = '/tmp/claude-0/-home-user-vertical-bot-ros/25e24200-66a7-5233-b5cf-23e3c701b805/scratchpad/shots';
const browser = await chromium.launch({ executablePath: '/opt/pw-browsers/chromium-1194/chrome-linux/chrome', args: ['--use-gl=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'] });
const page = await browser.newPage({ viewport: { width: 1500, height: 900 } });
const errors = [];
page.on('pageerror', (e) => errors.push(`[pageerror] ${e.message}`));
page.on('console', (m) => { if (m.type() === 'error') errors.push(`[error] ${m.text()}`); });
await page.goto('http://127.0.0.1:4173/?demo=pickplace&server=off', { waitUntil: 'load' });
await page.waitForTimeout(1000);
// static collision check via menu, then collision-checked validation
await page.click('text=Robot');
await page.click('text=Check collisions now (static)');
await page.waitForTimeout(300);
const r1 = await page.evaluate(() => { const app = window.app; app.checkCollisions = true; app.previewProgram(); return { problems: app.activeProgram.lastResult.problems.map(p => p.message), collisions: app.sim.collisions.length }; });
console.log('collision validation:', JSON.stringify(r1));
// move a box into the robot path and re-validate
const r2 = await page.evaluate(() => { const app = window.app; const box = app.station.find('Box 2'); const r = app.activeRobot; const fk = r.fk(); const p = r.poseAbs(); box.setPoseAbs(window.robomath.transl(p[12] + fk.linkPoses[3][12], p[13] + fk.linkPoses[3][13], p[14] + fk.linkPoses[3][14] - 100).m); app.previewProgram(); return { problems: app.activeProgram.lastResult.problems.map(p => p.message).slice(0, 3) }; });
console.log('with obstacle:', JSON.stringify(r2));
await page.screenshot({ path: `${shots}/collision.png` });
// orchard: run world for a while at high speed, check fleet dashboard
await page.goto('http://127.0.0.1:4173/?demo=orchard&server=off', { waitUntil: 'load' });
await page.waitForTimeout(1200);
await page.click('.tab:has-text("Fleet")');
const r3 = await page.evaluate(async () => { const app = window.app; app.simSpeed = 50; app.startWorld(); await new Promise(r => setTimeout(r, 6000)); app.pauseWorld(); const f = app.station.itemsOfType(102)[0]; const k = app.fleetManager(f).kpis(); return { worldTime: app.worldTime, done: k.tasksDone, pending: k.tasksPending, fruit: k.fruitPicked, kg: Math.round(k.yieldKg), robots: Object.values(k.perRobot).map(r => r.status) }; });
console.log('fleet:', JSON.stringify(r3));
await page.waitForTimeout(500);
await page.screenshot({ path: `${shots}/fleet.png` });
// packing line process stats
await page.goto('http://127.0.0.1:4173/?demo=packing&server=off', { waitUntil: 'load' });
await page.waitForTimeout(1000);
await page.click('.tab:has-text("Process")');
const r4 = await page.evaluate(async () => { const app = window.app; app.simSpeed = 100; app.startWorld(); await new Promise(r => setTimeout(r, 4000)); app.pauseWorld(); return { t: app.processSim.time, stats: app.processSim.statistics().map(s => `${s.name}:${s.exited}`) }; });
console.log('process:', JSON.stringify(r4));
await page.screenshot({ path: `${shots}/process.png` });
console.log('ERRORS:', errors.length ? errors.join('\n') : 'none');
await browser.close();
