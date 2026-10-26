// @vitest-environment jsdom
/**
 * F1 (V1 finding): a RoboDK robot whose file stores no post processor (Item.getParam('PostProcessor') == '' in the
 * ground truth) is exported like RoboDK 5.9.4 does it, with RoboDK's built-in generator of the brand (G4: the Library
 * code ground truth was written by those generators, src/posts/rdk_builtin.ts)
 * (tests/fixtures/robodk_gt/code: .LS for the Fanuc of 06.d Pick and stamp, .mod for the ABB of 06.b, .src for the
 * KUKAs of 08.a, .pdl for the Comau of 05.d). The studio imported them with the Generic post, so the Export dialog
 * offered the CSV post and the user downloaded .csv files.
 */
import { describe, it, expect, beforeAll, afterEach } from 'vitest';
import path from 'node:path';
import { installDomStubs, makeApp, currentDialog, fieldInput, clickCancel, closeAllDialogs } from './ui_harness';
import { haveLibrary, loadLibraryStation, gtCode, gtStationDir } from './posts_library_util';
import { exportDialog } from '../src/ui/dialogs';
import { getPost } from '../src/posts/index';
import { defaultPostOf } from '../src/io/robodk/rdk_robot';
import { ItemType } from '../src/core/items/item';
import { Robot } from '../src/core/items/robot';
import { Program } from '../src/core/items/program';

beforeAll(() => installDomStubs());
afterEach(() => closeAllDialogs());

const CASES: Array<[string, string]> = [
  ['Example-06.d-Pick and stamp.rdk', 'Fanuc'],
  ['Example-06.b-Pick and place 2 tables.rdk', 'ABB'],
  ['Example-08.a-Synchronize 3 Robots with a Screen (GUI).rdk', 'KUKA'],
  ['Example-05.d-H bot gantry - Comau.rdk', 'Comau'],
];

describe('default post processor of RoboDK robots without one', () => {
  it("RoboDK's built-in generators for ABB, Fanuc, KUKA, Comau, UR (tests/posts_builtin.test.ts); the placeholder for mechanisms", () => {
    expect(defaultPostOf('ABB IRB 1600-6/1.45', 1, 6)).toBe('RoboDK_Builtin_ABB');
    expect(defaultPostOf('Fanuc LR Mate 200iD/7L', 2, 6)).toBe('RoboDK_Builtin_Fanuc');
    expect(defaultPostOf('KUKA KR 6 R900', 5, 6)).toBe('RoboDK_Builtin_KUKA');
    expect(defaultPostOf('Comau Smart5 NJ 165-3.0', 11, 6)).toBe('RoboDK_Builtin_Comau');
    expect(defaultPostOf('UR10e', 0, 6)).toBe('RoboDK_Builtin_UR');
    // no post-less Motoman station exists to check RoboDK's choice: the brand's post
    expect(defaultPostOf('Motoman GP180', 3, 6)).toBe('Motoman');
    expect(defaultPostOf('Turntable', 0, 1)).toBe('RoboDK_Builtin_Generic');
    expect(defaultPostOf('Gudel 3-axis gantry', 0, 3)).toBe('RoboDK_Builtin_Generic');
    for (const id of ['RoboDK_Builtin_ABB', 'RoboDK_Builtin_Fanuc', 'RoboDK_Builtin_KUKA', 'RoboDK_Builtin_Comau', 'RoboDK_Builtin_UR', 'RoboDK_Builtin_Generic']) expect(getPost(id), id).toBeTruthy();
  });

  for (const [file, brand] of CASES) {
    it.skipIf(!haveLibrary(file))(`${file}: the ${brand} robot exports with the post whose files RoboDK wrote, and the Export dialog selects it`, async () => {
      const st = await loadLibraryStation(file);
      const gt = gtCode();
      const dir = gtStationDir(file);
      // programs RoboDK generated -> extension it wrote
      const extOf = new Map<string, string>();
      for (const k of gt.keys()) if (k.startsWith(dir + '/')) { const [, prog, f] = k.split('/'); if (prog && f && !/\.(txt|urp)$/i.test(f)) extOf.set(prog, path.extname(f).toLowerCase()); }
      const norm = (s: string) => s.replace(/[^A-Za-z0-9]/g, '').toLowerCase();
      let checked = 0;
      const app = makeApp();
      app.setStation(st);
      for (const [prog, ext] of extOf) {
        const p = st.itemsOfType<Program>(ItemType.PROGRAM).find((x) => x.name === prog || norm(x.name) === norm(prog));
        const r = p?.robot();
        if (!(r instanceof Robot) || r.dof < 4) continue;
        if (gt.get(`${dir}/${prog}/${[...gt.keys()].find((k) => k.startsWith(`${dir}/${prog}/`))!.split('/')[2]}`)!.toString('latin1').includes('Invalid license')) continue;
        expect(r.params.rdkPostProcessor, `${r.name}: RoboDK stores no post`).toBe('');
        const post = getPost(r.postProcessor);
        expect(post, `${r.name}: post ${r.postProcessor}`).toBeTruthy();
        expect(`.${post!.extension.toLowerCase()}`, `${prog} (${r.name}, ${post!.id})`).toBe(ext);
        const dlg = exportDialog(app, p!);
        expect((fieldInput('Post processor', currentDialog()) as HTMLSelectElement).value, prog).toBe(post!.id);
        clickCancel(); await dlg;
        checked++;
      }
      expect(checked).toBeGreaterThan(0);
    }, 120000);
  }
});
