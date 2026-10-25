/**
 * F1 (V1 finding): RoboDK hides only the item whose visibility flag is off. A hidden reference frame keeps the robot /
 * objects under it drawn (Library 03.b, 02.b, 06.d Pick and stamp, 06.f CNC store their robots under hidden base
 * frames), and a hidden robot or tool keeps its tools / attached objects drawn. Checked against RoboDK 5.9.4:
 *  - the ground truth dumps (tests/fixtures/robodk_gt: Visible() per item) — what RoboDK reports visible must be drawn
 *    by the studio's 3D view (SceneRenderer.syncTransforms) and its simulated cameras (sceneMeshes), what it reports
 *    hidden must not;
 *  - a RoboDK experiment (hidden frame / robot / tool with children, snapshots of the main view): children stay drawn.
 */
import { describe, it, expect } from 'vitest';
import { existsSync, readFileSync } from 'node:fs';
import { gunzipSync } from 'node:zlib';
import path from 'node:path';
import * as THREE from 'three';
import { importRdkBestEffort } from '../src/io/robodk/rdk_container';
import { AssetStore } from '../src/scene/assets';
import { SceneRenderer } from '../src/scene/renderer';
import { Station, Item, ItemType, Frame, SceneObject, Tool, Folder, isShown } from '../src/core/items/item';
import { Robot } from '../src/core/items/robot';
import { createRobotFromLibrary } from '../src/core/items/library';

const LIB = 'C:/RoboDK/Library';
const GT = path.resolve(__dirname, 'fixtures/robodk_gt');
const gtOf = (file: string): any => { const p = path.join(GT, file.replace(/ /g, '_').replace(/[()]/g, '_') + '_.json.gz'); return existsSync(p) ? JSON.parse(gunzipSync(readFileSync(p)).toString().replace(/\bNaN\b/g, 'null')) : null; };

/** Drawn state of every item after the 3D view's per-frame sync (a SceneRenderer without WebGL: only syncTransforms). */
function drawn(station: Station): Map<Item, boolean> {
  const entries = new Map<string, any>();
  for (const it of station.walk()) if (it !== station) entries.set(it.id, { item: it, root: new THREE.Group() });
  const fake: any = { station, entries, collisionIds: new Set(), showFrames: true, showTargets: true, scene: new THREE.Scene(), gizmoTarget: null, transform: { dragging: false } };
  (SceneRenderer.prototype as any).syncTransforms.call(fake);
  return new Map([...entries.values()].map((e) => [e.item, e.root.visible]));
}

describe('visibility: a hidden item hides only itself (RoboDK)', () => {
  it('hidden frame / robot / tool keep their children drawn; folders still hide their contents', () => {
    const st = new Station('s');
    const fh = st.addChild(new Frame('F_hidden'));
    const under = fh.addChild(new SceneObject('ObjUnderHiddenFrame'));
    const robot = createRobotFromLibrary('UR10e');
    st.addChild(robot);
    const tool = robot.addChild(new Tool('T')) as Tool;
    const onTool = tool.addChild(new SceneObject('ObjOnTool'));
    const folder = st.addChild(new Folder('Folder'));
    const inFolder = folder.addChild(new SceneObject('InFolder'));
    fh.setVisible(false); robot.setVisible(false); tool.setVisible(false); folder.setVisible(false);
    const d = drawn(st);
    expect(d.get(fh)).toBe(false);
    expect(d.get(under)).toBe(true);
    expect(d.get(robot)).toBe(false);
    expect(d.get(tool)).toBe(false);
    expect(d.get(onTool)).toBe(true);
    expect(d.get(inFolder)).toBe(false);
    expect([isShown(under), isShown(onTool), isShown(inFolder)]).toEqual([true, true, false]);
  });

  const FILES = ['Example-03.b-Welding Station - Fanuc.rdk', 'Example-02.b-Robot Machining Mold - Motoman.rdk', 'Example-06.d-Pick and stamp.rdk', 'Example-06.f-Pick and Place CNC and Dual RobotiQ Gripper-UR10.rdk', 'Example-06.e-Conveyor with 2 UR robots.rdk'];
  for (const file of FILES) {
    it.skipIf(!existsSync(`${LIB}/${file}`) || !gtOf(file))(`${file}: robots, tools and objects are drawn exactly when RoboDK shows them`, async () => {
      const gt = gtOf(file);
      const b = readFileSync(`${LIB}/${file}`);
      const { station } = await importRdkBestEffort(b.buffer.slice(b.byteOffset, b.byteOffset + b.byteLength) as ArrayBuffer, file, new AssetStore());
      const d = drawn(station);
      const byName = new Map<string, Item[]>();
      for (const it of station.walk()) byName.set(it.name, [...(byName.get(it.name) ?? []), it]);
      const gtCount = new Map<string, number>();
      for (const g of gt.items) gtCount.set(`${g.name}`, (gtCount.get(g.name) ?? 0) + 1);
      let checked = 0, robotsShown = 0;
      const wrong: string[] = [];
      for (const g of gt.items) {
        if (![2, 4, 5].includes(g.type) || gtCount.get(g.name) !== 1) continue;
        const its = (byName.get(g.name) ?? []).filter((x) => x.type === g.type);
        if (its.length !== 1) continue;
        checked++;
        const want = g.visible === 1;
        if (d.get(its[0]) !== want) wrong.push(`${g.name}: RoboDK ${want ? 'shows' : 'hides'} it, studio ${d.get(its[0]) ? 'draws' : 'hides'} it`);
        if (g.type === 2 && want && d.get(its[0])) robotsShown++;
      }
      expect(wrong, wrong.join('\n')).toEqual([]);
      expect(checked).toBeGreaterThan(2);
      expect(robotsShown).toBeGreaterThan(0);
      // the reference frames RoboDK reports hidden stay hidden (only their triads)
      for (const g of gt.items) {
        if (g.type !== 3 || gtCount.get(g.name) !== 1 || g.visible !== 0) continue;
        const f = (byName.get(g.name) ?? []).filter((x) => x.type === ItemType.FRAME);
        if (f.length === 1) expect(d.get(f[0]), g.name).toBe(false);
      }
      // every robot of these stations is drawn although its base frame is hidden (the V1 report: 0 visible meshes)
      for (const r of station.itemsOfType<Robot>(ItemType.ROBOT)) if (r.visible) expect(isShown(r), r.name).toBe(true);
    }, 120000);
  }
});
