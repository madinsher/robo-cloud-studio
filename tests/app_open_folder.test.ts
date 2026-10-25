// @vitest-environment jsdom
/**
 * File › Open folder findings (V5 / V7):
 *  - Webots standard robots (remote PROTOs on raw.githubusercontent.com) were dropped in the browser: the import now
 *    downloads the PROTOs / meshes it asks for, in rounds (src/io/library/webots_remote.ts);
 *  - skipped robots / missing PROTOs were logged at info level only: they are warnings with one summary toast;
 *  - a folder with several worlds silently opened the shallowest one: the user picks (App.importUi.chooseWorld);
 *  - a ROS package shipping Webots worlds came in as the world only: the user chooses world or description;
 *  - a description folder without package.xml was imported file by file (0-DOF robots, stray meshes);
 *  - xacro args / entry points could not be passed: App.importUi.rosOptions re-imports with them;
 *  - the Webots Viewpoint `follow` was ignored: the 3D view follows that robot.
 */
import { describe, it, expect, beforeAll } from 'vitest';
import { existsSync, readFileSync, readdirSync, statSync } from 'node:fs';
import path from 'node:path';
import { makeApp, installDomStubs, toasts, clearToasts } from './ui_harness';
import { followTarget, App } from '../src/app';
import { Station, Frame } from '../src/core/items/item';
import { Robot } from '../src/core/items/robot';
import { withRemoteWebotsFiles, webotsRemoteCache } from '../src/io/library/webots_remote';
import { importWebotsFiles } from '../src/io/webots/index';
import { AssetStore } from '../src/scene/assets';
import { parseXacroArgs } from '../src/ui/dialogs';
import { webotsCacheFetcher, WEBOTS_HOME, HAS_WEBOTS_CACHE } from './webots_helpers';

beforeAll(() => installDomStubs());

/** File objects of a folder as the browser's folder picker gives them (webkitRelativePath = <folder>/<path>). */
function folderFiles(root: string, files: Record<string, string>): File[] {
  return Object.entries(files).map(([p, text]) => { const f = new File([text], p.split('/').pop()!); Object.defineProperty(f, 'webkitRelativePath', { value: `${root}/${p}` }); return f; });
}
function diskFolder(dir: string, max = 400): File[] {
  const out: File[] = [];
  const top = path.basename(dir);
  const walk = (d: string, rel: string) => {
    for (const n of readdirSync(d)) {
      if (out.length >= max) return;
      const p = path.join(d, n);
      if (statSync(p).isDirectory()) walk(p, `${rel}${n}/`);
      else { const f = new File([readFileSync(p)], n); Object.defineProperty(f, 'webkitRelativePath', { value: `${top}/${rel}${n}` }); out.push(f); }
    }
  };
  walk(dir, '');
  return out;
}
const WORLD = (name: string, box = 0.2) => `#VRML_SIM R2025a utf8\nWorldInfo { title "${name}" }\nViewpoint { position 0 -3 1 orientation 0 0 1 1.57 follow "rover" }\nSolid { name "rover" children [ Shape { geometry Box { size ${box} ${box} ${box} } } ] }\n`;

describe('Webots import in the browser', () => {
  it.skipIf(!HAS_WEBOTS_CACHE || !existsSync(`${WEBOTS_HOME}/projects/robots/abb/irb/worlds`))('the ABB IRB 4600 world brings its robot: remote PROTOs are fetched in rounds (Webots cache as the network)', async () => {
    webotsRemoteCache().clear();
    const cache = webotsCacheFetcher();
    const dir = `${WEBOTS_HOME}/projects/robots/abb/irb`;
    const map = new Map<string, Uint8Array>();
    for (const f of diskFolder(dir)) map.set((f as any).webkitRelativePath, new Uint8Array(await f.arrayBuffer()));
    // offline (no fetch): the robot is missing, as the verifier saw in the browser
    const offline = await importWebotsFiles(map, new AssetStore());
    expect([...offline.station.walk()].some((i) => i instanceof Robot)).toBe(false);
    expect(offline.report.some((l) => /not available offline/.test(l))).toBe(true);
    const { result, stats } = await withRemoteWebotsFiles((remote) => importWebotsFiles(map, new AssetStore(), { remote }), { fetchFn: async (u) => cache(u) ?? null });
    const robots = [...result.station.walk()].filter((i): i is Robot => i instanceof Robot);
    expect(stats.fetched.length).toBeGreaterThan(0);
    expect(robots.some((r) => r.dof === 6)).toBe(true);
    expect(result.report.some((l) => /Irb4600-40: remote PROTO .* not available offline/.test(l))).toBe(false);
  }, 240000);

  it('import problems are warnings in the Log with one summary toast', () => {
    const app = makeApp();
    clearToasts();
    const n = app.logImportReport('Webots world X', ['Imported robot A', 'Irb4600-40: remote PROTO https://x/Irb4600-40.proto is not available offline', 'E-puck (DEF E_PUCK): skipped (cannot be resolved)']);
    expect(n).toBe(2);
    expect(app.logs.filter((l) => l.level === 'warn').length).toBe(2);
    expect(toasts().filter((t) => /could not be imported/.test(t)).length).toBe(1);
  });

  it('a folder with several worlds asks which one to open; the Viewpoint follow target is followed', async () => {
    const app = makeApp();
    const asked: string[][] = [];
    app.importUi = { chooseWorld: async (worlds) => { asked.push(worlds); return worlds.find((w) => w.endsWith('retail_store.wbt'))!; } };
    let followed: unknown = 'unset';
    (app.renderer as any).setFollow = (it: unknown) => { followed = it; };
    await app.openFiles(folderFiles('murka_simulation', { 'worlds/retail_store_classic.wbt': WORLD('Classic', 0.1), 'worlds/retail_store.wbt': WORLD('MUROM v2', 0.3) }));
    expect([...asked[0]].sort()).toEqual(['murka_simulation/worlds/retail_store.wbt', 'murka_simulation/worlds/retail_store_classic.wbt']);
    expect(app.station.name).toBe('MUROM v2');
    app.applyInitialView();
    expect((followed as any)?.name).toBe('rover');
  });
});

describe('ROS packages from Open folder', () => {
  const URDF = (name: string) => `<robot name="${name}"><link name="base"/><link name="l1"/><joint name="j1" type="revolute"><parent link="base"/><child link="l1"/><axis xyz="0 0 1"/><limit lower="-3" upper="3" effort="1" velocity="1"/></joint></robot>`;
  it('a description folder without package.xml is imported as a package (no file-by-file robots / stray meshes)', async () => {
    const app = makeApp();
    const before = [...app.station.walk()].length;
    await app.openFiles(folderFiles('ur_description', { 'urdf/arm.urdf': URDF('arm'), 'meshes/visual/base.stl': 'solid x\nendsolid x\n' }));
    const robots = [...app.station.walk()].filter((i): i is Robot => i instanceof Robot);
    expect(robots.map((r) => r.name)).toContain('arm');
    expect(robots.every((r) => r.dof > 0)).toBe(true);
    expect(app.logs.some((l) => /no package\.xml/.test(l.text))).toBe(true);
    expect([...app.station.walk()].length - before).toBeLessThan(6);
  });

  it('xacro args and the entry point chosen in the dialog are used for the import', async () => {
    const app = makeApp();
    const xacro = `<?xml version="1.0"?><robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="$(arg name)"><xacro:arg name="name" default=""/><link name="base"/><link name="tip"/><joint name="j" type="revolute"><parent link="base"/><child link="tip"/><axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint></robot>`;
    let shown: any = null;
    app.importUi = { rosOptions: async (info) => { shown = info; return { entries: [info.entries.find((e) => e.file.endsWith('.xacro'))!.file], args: parseXacroArgs('name:=ur5e') }; } };
    await app.openFiles(folderFiles('ur_pkg', { 'package.xml': '<package><name>ur_pkg</name></package>', 'urdf/ur.urdf.xacro': xacro, 'urdf/other.urdf': URDF('other') }));
    expect(shown?.entries.map((e: any) => e.file), JSON.stringify(shown)).toContain('ur_pkg/urdf/ur.urdf.xacro');
    expect([...app.station.walk()].filter((i) => i instanceof Robot).map((r) => r.name)).toContain('ur5e');
  });

  it('parseXacroArgs reads name:=value lists', () => {
    expect(parseXacroArgs('robot_name:=utp_a_02 slot_front:=lidar2d use_manipulator:=false  slot_top:=none')).toEqual({ robot_name: 'utp_a_02', slot_front: 'lidar2d', use_manipulator: 'false', slot_top: 'none' });
  });
});

describe('Viewpoint follow', () => {
  it('finds the robot named by follow (name, Webots DEF / name params, case-insensitive fallback)', () => {
    const st = new Station('s');
    const a = st.addChild(new Frame('MUROM'));
    const b = st.addChild(new Frame('base')); b.params.webotsName = 'murom';
    expect(followTarget(st, 'murom')).toBe(b);
    expect(followTarget(st, 'MUROM')).toBe(a);
    expect(followTarget(st, 'murom', 'None')).toBeNull();
    expect(followTarget(st, '')).toBeNull();
    void App;
  });
});
