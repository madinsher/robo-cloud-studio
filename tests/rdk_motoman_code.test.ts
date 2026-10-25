/**
 * F1 (V1 finding): a Motoman program exported from a RoboDK station uses the robot's pulse ratios and RoboDK's tool
 * number. RoboDK 5.9.4 (tests/fixtures/robodk_gt/code, Library 02.i, Motoman ES200D, Motoman_Cartesian post) writes
 * PULSE positions as joints x pulses per degree from the robot parameters (imported as params.motomanPulses) and takes
 * the tool number from the tool name ("USER - CONE - 10MM  X  45 DEG" -> ///TOOL 45). The studio wrote joint degrees
 * as pulses (1 pulse/deg: a YRC controller would go to ~0.01 deg instead of -27 deg) and ///TOOL 9.
 */
import { describe, it, expect } from 'vitest';
import { compareStationCode, haveLibrary, gtCode, gtStationDir, loadLibraryStation } from './posts_library_util';
import { compileForPost, getPost } from '../src/posts/index';
import { nameIndex } from '../src/posts/base';
import { ItemType } from '../src/core/items/item';
import { Program } from '../src/core/items/program';
import { Robot } from '../src/core/items/robot';

const FILE = 'Example-02.i-Wood engraving - Remote TCP - Motoman.rdk';
/** //POS section of a JBI (position lines and the ///TOOL / ///POSTYPE headers). */
const posSection = (s: string) => { const L = s.replace(/\r/g, '').split('\n'); return L.slice(L.indexOf('//POS') + 1, L.indexOf('//INST')); };

describe('RoboDK tool / frame numbers from item names', () => {
  it('the last number written as a word, else trailing digits (RoboDK 02.i: 45, 30; none for "10MM")', () => {
    expect(nameIndex('USER - CONE - 10MM  X  45 DEG')).toBe(45);
    expect(nameIndex('USER - ENGRAVE - 30 DEG POINT END')).toBe(30);
    expect(nameIndex('BALL END - 10MM')).toBe(-1);
    expect(nameIndex('Wood_Holder')).toBe(-1);
    expect(nameIndex('Frame 3')).toBe(3);
    expect(nameIndex('Tool2')).toBe(2);
  });
});

describe.skipIf(!haveLibrary(FILE))('02.i Motoman ES200D: JBI positions in pulses like RoboDK', () => {
  it('the imported robot carries the ES200D pulse ratios', async () => {
    const st = await loadLibraryStation(FILE);
    const r = st.itemsOfType<Robot>(ItemType.ROBOT)[0];
    expect((r.params.motomanPulses as number[]).map((v) => +v.toFixed(2))).toEqual([2389.33, 2933.67, 2447.56, 2474.67, 2265.44, 1248.06]);
  });

  it('Approach: //POS equals RoboDK (C00001=0,-78222,-152582,0,103479,0 and ///TOOL 45)', async () => {
    const st = await loadLibraryStation(FILE);
    const p = st.itemsOfType<Program>(ItemType.PROGRAM).find((x) => x.name === 'Approach')!;
    const post = getPost((p.robot() as Robot).postProcessor)!;
    expect(post.id).toBe('Motoman_Cartesian');
    const files = post.generate(compileForPost(st, p));
    const mine = files.find((f) => f.name === 'APPROAC.JBI')!;
    const gt = gtCode().get(`${gtStationDir(FILE)}/Approach/APPROAC.JBI`)!.toString('latin1');
    expect(posSection(mine.content)).toEqual(posSection(gt));
  });

  it('every 02.i program RoboDK generated: the first PULSE position equals the RoboDK one within 1 pulse', async () => {
    // (the other //POS lines of the machining programs differ for reasons outside the pulse conversion: RoboDK
    // regenerated those programs before writing them, and writes linear moves as RECTAN positions)
    const cases = await compareStationCode(FILE);
    const st = await loadLibraryStation(FILE);
    const gt = gtCode();
    const norm = (t: string) => t.replace(/[^A-Za-z0-9]/g, '').toLowerCase();
    const firstPulse = (s: string) => { const L = posSection(s); const k = L.indexOf('///PULSE'); return k >= 0 ? L[k + 1].split('=')[1].split(',').map(Number) : null; };
    const bad: string[] = [];
    let n = 0;
    for (const c of cases.filter((x) => x.status === 'equal' || x.status === 'diff')) {
      const p = st.itemsOfType<Program>(ItemType.PROGRAM).find((x) => x.name === c.program || norm(x.name) === norm(c.program))!;
      const mine = getPost((p.robot() as Robot).postProcessor)!.generate(compileForPost(st, p)).find((f) => f.name.toLowerCase() === c.file.toLowerCase());
      const want = firstPulse(gt.get(`${gtStationDir(FILE)}/${c.program}/${c.file}`)!.toString('latin1'));
      if (!want || !mine) continue;
      const got = firstPulse(mine.content);
      n++;
      if (!got || got.length !== want.length || got.some((v, i) => Math.abs(v - want[i]) > 1)) bad.push(`${c.program}/${c.file}: RoboDK ${want.join(',')} studio ${got?.join(',')}`);
    }
    expect(n).toBeGreaterThan(8);
    expect(bad, bad.join(' | ')).toEqual([]);
  }, 120000);
});
