// @vitest-environment jsdom
/**
 * T11 (I7 wiring): a Webots project or a ROS package dropped / picked as a folder goes through App.openFiles to
 * importWebotsFiles / importRosPackageFiles with the paths relative to the folder (File.webkitRelativePath from the
 * folder picker, File.relativePath from a drag-and-drop walk). The project's meshes are assets of the import, never
 * stray objects; a ROS package is merged into the open station as one undo step.
 */
import { describe, it, expect, beforeAll } from 'vitest';
import { readFileSync, readdirSync, statSync, existsSync } from 'node:fs';
import { join, relative } from 'node:path';
import { installDomStubs, makeApp } from './ui_harness';
import { ItemType, SceneObject, Robot } from '../src/app';
import { MobileRobot } from '../src/mobile/items';
import { importWebotsFiles } from '../src/io/webots/index';
import { zipSync } from 'fflate';
import { AssetStore } from '../src/scene/assets';

beforeAll(() => installDomStubs());
const WEBOTS = join(__dirname, 'fixtures/webots/mini');
const TB4 = join(__dirname, 'fixtures/ros_gt/pkgs/nav2_minimal_tb4_description');

/** Files of a folder as the browser gives them: name = basename, the folder-relative path on webkitRelativePath / relativePath. */
function folderFiles(dir: string, how: 'picker' | 'drop'): File[] {
  const out: File[] = [];
  const root = join(dir, '..');
  const walk = (d: string) => {
    for (const n of readdirSync(d)) {
      const p = join(d, n);
      if (statSync(p).isDirectory()) { walk(p); continue; }
      const f = new File([readFileSync(p)], n);
      Object.defineProperty(f, how === 'picker' ? 'webkitRelativePath' : 'relativePath', { value: relative(root, p).replace(/\\/g, '/') });
      out.push(f);
    }
  };
  walk(dir);
  return out;
}
const names = (items: Iterable<{ name: string }>) => [...items].map((i) => i.name).sort();

describe('opening projects (I7)', () => {
  it.skipIf(!existsSync(WEBOTS))('a Webots project folder (picker or drop) opens as the station importWebotsFiles builds; meshes are not added as objects', async () => {
    // reference: the importer called directly on the same files
    const ref = await importWebotsFiles(new Map(folderFiles(WEBOTS, 'picker').map((f) => [(f as any).webkitRelativePath, new Uint8Array(readFileSync(join(WEBOTS, '..', (f as any).webkitRelativePath)))])), new AssetStore());
    for (const how of ['picker', 'drop'] as const) {
      const app = makeApp('pickplace');
      await app.openFiles(folderFiles(WEBOTS, how));
      expect(app.station.name, how).toBe(ref.station.name);
      expect(names(app.station.walk()), how).toEqual(names(ref.station.walk()));
      expect(app.station.find('bot'), how).toBeInstanceOf(MobileRobot);
      expect(app.station.find('box_a'), how).toBeInstanceOf(SceneObject);
      // project meshes (tri.stl, two.obj, tex.dae) stay assets of the world: no stray "tri"/"two"/"tex" objects
      for (const n of ['tri', 'two', 'tex']) expect(app.station.itemsOfType(ItemType.OBJECT).some((o) => o.name === n), `${how}: stray ${n}`).toBe(false);
      // every mesh the world references resolves in the app's asset store
      for (const it of app.station.walk()) for (const g of ((it as any).geometry ?? []) as Array<{ mesh?: string }>) if (g.mesh) expect(app.assets.resolve(g.mesh), `${how}: ${it.name} → ${g.mesh}`).toBeTruthy();
      expect(app.stations.includes(app.station)).toBe(true);
    }
  });

  it.skipIf(!existsSync(TB4))('a ROS package folder is merged into the open station (one undo step) as the robot importRosPackageFiles builds', async () => {
    for (const how of ['picker', 'drop'] as const) {
      const app = makeApp('pickplace');
      const before = names(app.station.children);
      const undoBefore = (app as any).hist().undo.length;
      await app.openFiles(folderFiles(TB4, how));
      const added = app.station.children.filter((c) => !before.includes(c.name));
      expect(added.length, how).toBe(1);
      expect(added[0], how).toBeInstanceOf(MobileRobot); // TurtleBot4: differential mobile base
      // the arm-less base keeps the joint tree: its wheels are robots / joints of the mobile item, nothing at station level
      expect(app.station.itemsOfType(ItemType.ROBOT).filter((r) => r.parent === app.station).length).toBe(1); // only the pick & place arm
      expect((app as any).hist().undo.length, how).toBe(undoBefore + 1);
      app.undo();
      expect(names(app.station.children), how).toEqual(before);
    }
  });

  it.skipIf(!existsSync(WEBOTS) || !existsSync(TB4))('a zipped Webots project opens as its world; a zipped ROS package is merged as its robot', async () => {
    const zipOf = (dir: string) => new File([zipSync(Object.fromEntries(folderFiles(dir, 'picker').map((f) => [(f as any).webkitRelativePath, new Uint8Array(readFileSync(join(dir, '..', (f as any).webkitRelativePath)))])))], `${dir.split(/[\\/]/).pop()}.zip`);
    const w = makeApp('pickplace');
    await w.openFiles([zipOf(WEBOTS)]);
    expect(w.station.find('bot')).toBeInstanceOf(MobileRobot);
    expect(w.station.find('box_a')).toBeInstanceOf(SceneObject);
    expect(w.station.itemsOfType(ItemType.FOLDER).some((f) => /mini/.test(f.name))).toBe(false); // not the generic container import
    const r = makeApp('pickplace');
    const n0 = r.station.children.length;
    await r.openFiles([zipOf(TB4)]);
    expect(r.station.children.length).toBe(n0 + 1);
    expect(r.station.children.at(-1)).toBeInstanceOf(MobileRobot);
  });

  it('a single .wbt file opens alone (no folder): its inline nodes are built', async () => {
    const app = makeApp();
    const wbt = '#VRML_SIM R2025a utf8\nWorldInfo { title "Single" }\nSolid { translation 1 2 0.5 name "crate" children [ Shape { geometry Box { size 0.2 0.2 0.2 } } ] }\n';
    await app.openFiles([new File([wbt], 'single.wbt')]);
    const crate = app.station.find('crate') as SceneObject;
    expect(crate).toBeInstanceOf(SceneObject);
    const p = crate.poseAbs();
    expect([p[12], p[13], p[14]]).toEqual([1000, 2000, 500]); // Webots metres → mm
    expect(app.station.itemsOfType(ItemType.ROBOT).length).toBe(0);
    void Robot;
  });
});
