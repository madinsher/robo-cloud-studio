// @vitest-environment jsdom
/**
 * G3: Connect › ROS 2 via rosbridge on a MURKA world opened without murom_world/config offers "Load retail layout
 * (murom_world/config)…"; the folder pick spawns the 50 products into the open station (what the Webots supervisor
 * does from $MUROM_CONFIG_DIR). A station that has its products, or no MUROM robot, gets no such button.
 */
import { describe, it, expect, beforeAll, afterEach } from 'vitest';
import fs from 'node:fs';
import path from 'node:path';
import { installDomStubs, makeApp, currentDialog, clickCancel, closeAllDialogs, waitFor } from './ui_harness';
import { importWebotsWorld } from '../src/io/webots';
import { HAS_MURKA, RETAIL_V2, murkaSource } from './webots_helpers';
import { buildRetailStation, MURKA_CONFIG } from './ros_murka_helpers';
import { retailProductCount } from '../src/ros/murka_layout';

beforeAll(() => installDomStubs());
afterEach(() => closeAllDialogs());

describe('Connect › ROS 2: retail layout of a MURKA world', () => {
  it.skipIf(!HAS_MURKA)('a MUROM world without products: the button loads murom_world/config into the station', async () => {
    const app = makeApp();
    app.setStation(importWebotsWorld('worlds/retail_store.wbt', murkaSource(RETAIL_V2), { assets: app.assets }).station);
    expect(retailProductCount(app.station)).toBe(0);
    const { MenuBar } = await import('../src/ui/menu');
    const menu = new MenuBar(app, () => {});
    const p = menu.rosDialog();
    const dlg = currentDialog();
    const btn = [...dlg.querySelectorAll('button')].find((b) => /Load retail layout/.test(b.textContent ?? '')) as HTMLButtonElement;
    expect(btn).toBeTruthy();
    expect(dlg.textContent).toMatch(/has no products/);
    // the folder picker (webkitdirectory input) answers with the two YAML files of murom_world/config
    const files = ['products.yaml', 'locations.yaml'].map((n) => {
      const f = new File([fs.readFileSync(path.join(MURKA_CONFIG, n), 'utf8')], n);
      Object.defineProperty(f, 'webkitRelativePath', { value: `config/${n}` });
      return f;
    });
    const click = HTMLInputElement.prototype.click;
    HTMLInputElement.prototype.click = function (this: HTMLInputElement) {
      if (this.type !== 'file') return click.call(this);
      Object.defineProperty(this, 'files', { value: files });
      this.dispatchEvent(new Event('change'));
    };
    try { btn.click(); await waitFor(() => /Retail layout loaded: 50 products/.test(dlg.textContent ?? ''), 20000); } finally { HTMLInputElement.prototype.click = click; }
    expect(retailProductCount(app.station)).toBe(50);
    expect(btn.disabled).toBe(true);
    expect(app.logs.some((l) => /Retail layout .*50 products, 93 slots/.test(l.text))).toBe(true);
    clickCancel(); await p;
  }, 60000);

  it.skipIf(!HAS_MURKA)('a station with its products (or without MUROM) has no layout button', async () => {
    const app = makeApp();
    app.setStation(buildRetailStation().station);
    const { MenuBar } = await import('../src/ui/menu');
    const menu = new MenuBar(app, () => {});
    let p = menu.rosDialog();
    expect([...currentDialog().querySelectorAll('button')].some((b) => /Load retail layout/.test(b.textContent ?? ''))).toBe(false);
    clickCancel(); await p;
    const app2 = makeApp('pickplace');
    p = new MenuBar(app2, () => {}).rosDialog();
    expect([...currentDialog().querySelectorAll('button')].some((b) => /Load retail layout/.test(b.textContent ?? ''))).toBe(false);
    clickCancel(); await p;
  }, 60000);
});
