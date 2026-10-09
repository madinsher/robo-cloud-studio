/** The Blender add-on's pure-python core is checked against the TypeScript kinematics on a demo station. */
import { describe, it, expect } from 'vitest';
import { execFileSync } from 'node:child_process';
import { mkdtempSync, writeFileSync } from 'node:fs';
import { tmpdir } from 'node:os';
import path from 'node:path';
import { demos } from '../src/demos';
import { saveStation } from '../src/io/station-file';
import { AssetStore } from '../src/scene/assets';
import { ProgramSimulator } from '../src/core/motion/simulator';
import { Program } from '../src/core/items/program';
import { Robot } from '../src/core/items/robot';
import { ItemType } from '../src/core/items/item';
import { multiply } from '../src/core/math/pose';

function hasPython(): boolean { try { execFileSync('python3', ['--version'], { stdio: 'ignore' }); return true; } catch { return false; } }

describe.skipIf(!hasPython())('Blender add-on core (python3)', () => {
  it('reproduces the studio forward kinematics, attachments and geometry', () => {
    const d = demos.find((x) => /pick/i.test(x.name))!;
    const st = d.build();
    const file: any = saveStation(st, new AssetStore());
    const prog = st.itemsOfType<Program>(ItemType.PROGRAM)[0];
    const sim = new ProgramSimulator(st);
    sim.compile(prog);
    file.animation = { program: prog.name, duration: sim.duration, dt: 0.1, robots: Object.fromEntries(st.itemsOfType<Robot>(ItemType.ROBOT).map((r) => [r.id, sim.jointsList(r.id, 0.1)])) };
    const expected = st.itemsOfType<Robot>(ItemType.ROBOT).flatMap((r) => [r.jointsHome(), [30, -60, 45, -100, 60, 10].slice(0, r.dof)].map((q) => { const fk = r.fk(q); return { robot: r.id, q, linkPoses: fk.linkPoses.map((m) => Array.from(m)), flange: Array.from(fk.flange), flangeAbs: Array.from(multiply(r.poseAbs(), r.solveFKFlange(q))) }; }));
    const dir = mkdtempSync(path.join(tmpdir(), 'vbs-blender-'));
    writeFileSync(path.join(dir, 'station.vbstation'), JSON.stringify(file));
    writeFileSync(path.join(dir, 'fk.json'), JSON.stringify(expected));
    const out = execFileSync('python3', [path.join(__dirname, '..', 'blender', 'test_core.py'), path.join(dir, 'station.vbstation'), path.join(dir, 'fk.json')], { encoding: 'utf8' });
    expect(out).toContain('blender core ok');
  });
});
