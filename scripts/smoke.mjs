import { chromium } from 'playwright';
const shots = '/tmp/claude-0/-home-user-vertical-bot-ros/25e24200-66a7-5233-b5cf-23e3c701b805/scratchpad/shots';
const browser = await chromium.launch({ executablePath: '/opt/pw-browsers/chromium-1194/chrome-linux/chrome', args: ['--use-gl=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'] });
const page = await browser.newPage({ viewport: { width: 1500, height: 900 } });
const errors = [];
page.on('console', (m) => { if (m.type() === 'error' || m.type() === 'warning') errors.push(`[${m.type()}] ${m.text()}`); });
page.on('pageerror', (e) => errors.push(`[pageerror] ${e.message}`));
for (const demo of ['pickplace', 'orchard', 'packing', 'welding', 'greenhouse', 'verticalbot']) {
  await page.goto(`http://127.0.0.1:4173/?demo=${demo}&server=off`, { waitUntil: 'load' });
  await page.waitForTimeout(1500);
  const info = await page.evaluate(() => {
    const app = window.app;
    const out = { station: app.station.name, items: [...app.station.walk()].length, program: app.activeProgram?.name ?? null, robots: app.station.itemsOfType(2).length };
    if (app.activeProgram) { app.runProgram(); app.sim.runToEnd(); out.duration = app.sim.duration; out.problems = app.sim.result.problems.map(p => p.message); }
    if (demo => true) { app.startWorld(); }
    return out;
  });
  await page.waitForTimeout(2500);
  const world = await page.evaluate(() => ({ worldTime: window.app.worldTime, fleets: window.app.station.itemsOfType(102).map(f => window.app.fleetManager(f).kpis()) }));
  await page.screenshot({ path: `${shots}/${demo}.png` });
  console.log(demo, JSON.stringify(info), JSON.stringify(world).slice(0, 300));
}
// UI interactions: open Add > Robot dialog, teach, run
await page.goto('http://127.0.0.1:4173/?demo=pickplace&server=off', { waitUntil: 'load' });
await page.waitForTimeout(1000);
await page.click('text=Add');
await page.click('text=Robot from library…');
await page.waitForTimeout(300);
await page.selectOption('.dialog select', 'KUKA_KR6_R900');
await page.click('.dialog .btn.primary');
await page.waitForTimeout(500);
await page.keyboard.press('j');
await page.keyboard.press('j');
await page.click('.tab:has-text("Console")');
await page.fill('.console-in', 'const r = RDK.Item("", ITEM_TYPE_ROBOT); const p = RDK.AddProgram("API", r); p.MoveJ(r.Joints()); print(p.Update()); RDK.ItemList(ITEM_TYPE_ROBOT, true)');
await page.click('text=Run (Ctrl+Enter)');
await page.waitForTimeout(500);
const consoleText = await page.textContent('.console-out');
console.log('console:', consoleText.replace(/\s+/g, ' ').slice(0, 300));
await page.click('.tab:has-text("Program")');
await page.click('.tab:has-text("Fleet")');
await page.click('text=Agriculture');
await page.click('text=Create field / orchard…');
await page.waitForTimeout(300);
await page.click('.dialog .btn.primary');
await page.waitForTimeout(800);
const after = await page.evaluate(() => ({ fields: window.app.station.itemsOfType(104).length, rows: window.app.station.itemsOfType(108).length, maps: window.app.station.itemsOfType(103).length }));
console.log('orchard dialog:', JSON.stringify(after));
await page.screenshot({ path: `${shots}/ui.png` });
console.log('ERRORS:', errors.length ? errors.slice(0, 20).join('\n') : 'none');
await browser.close();
