// Library sweep (P0c): every program of every C:/RoboDK/Library station compiles for its robot's post in < 30 s,
// keeps every move instruction (no dropped MoveJ / MoveL / MoveC), and the post RoboDK assigns to the robot writes
// valid-looking files (no NaN / undefined, every move of the program present in the output).
import { describe, it, expect } from 'vitest';
import { existsSync, readdirSync, readFileSync } from 'node:fs';
import path from 'node:path';
import { importRdkBestEffort } from '../src/io/robodk/rdk_container';
import { AssetStore } from '../src/scene/assets';
import { Robot } from '../src/core/items/robot';
import { Program } from '../src/core/items/program';
import { ItemType, Target } from '../src/core/items/item';
import { compileForPost, getPost } from '../src/posts/index';

const LIB = 'C:/RoboDK/Library';
const HAS_LIB = existsSync(LIB);
const STATIONS = HAS_LIB ? readdirSync(LIB).filter((f) => f.endsWith('.rdk')).sort() : [];

/** Move instructions that carry a target, a pose or joints (the ones a post can express). */
function expressibleMoves(st: any, p: Program): number {
  return p.instructions().filter((i) => {
    const d: any = i.data;
    if (!i.enabled || d.kind !== 'move') return false;
    if (d.moveType === 'MoveC' && !(d.viaTargetId || d.viaPose)) return false;
    const t = d.targetId ? st.findById(d.targetId) : null;
    return t instanceof Target || !!d.pose || !!d.joints;
  }).length;
}

describe('Library programs post without losing moves', () => {
  it('finds the RoboDK Library', () => { expect(HAS_LIB, `${LIB} missing: the sweep below is skipped`).toBe(true); });
  for (const file of STATIONS) {
    it.skipIf(!HAS_LIB)(file, async () => {
      const buf = readFileSync(path.join(LIB, file));
      const st = (await importRdkBestEffort(buf.buffer.slice(buf.byteOffset, buf.byteOffset + buf.byteLength), file, new AssetStore())).station;
      for (const p of st.itemsOfType<Program>(ItemType.PROGRAM)) {
        const robot = p.robot();
        const t0 = performance.now();
        const c = compileForPost(st, p);
        const dt = (performance.now() - t0) / 1000;
        expect(dt, `${p.name}: compile time`).toBeLessThan(30);
        const moveEvents = c.events.filter((e) => e.kind === 'moveJ' || e.kind === 'moveL' || e.kind === 'moveC').length;
        // every move kept, except the linear moves to the point the robot is already at (RoboDK skips them too)
        expect(moveEvents + (c.skippedMoves ?? 0), `${p.name}: moves kept`).toBe(expressibleMoves(st, p));
        const post = (robot instanceof Robot ? getPost(robot.postProcessor) : undefined) ?? getPost('Generic')!;
        const t1 = performance.now();
        const files = post.generate(c, { date: new Date(2026, 9, 25, 12) });
        expect((performance.now() - t1) / 1000, `${p.name}: post time`).toBeLessThan(30);
        expect(files.length, `${p.name} / ${post.id}`).toBeGreaterThan(0);
        const text = files.filter((f) => f.encoding !== 'base64').map((f) => f.content).join('\n');
        expect(text.replace(/undefined post processor/g, ''), `${p.name} / ${post.id}`).not.toMatch(/NaN|undefined/);
        // every joint target of the main program appears in the output with its joint values (first joint, 3 decimals)
        const firstJ = c.events.find((e) => e.kind === 'moveJ') as any;
        // (RoboDK's built-in generators write a MoveJ to a Cartesian target as a pose: ABB MoveJ robtarget, KUKA PTP POS)
        if (firstJ && post.id !== 'Generic' && !/Motoman/.test(post.id) && !(post.id.startsWith('RoboDK_Builtin_') && !firstJ.jointTarget)) {
          const j1 = Number(firstJ.joints[0]);
          const nums = (text.match(/-?\d+\.\d+/g) ?? []).map(Number);
          const shown = nums.some((v) => Math.abs(v - j1) < 0.006 || Math.abs(v - (j1 * Math.PI) / 180) < 1e-5);
          expect(shown, `${p.name} / ${post.id}: first MoveJ value ${j1} written`).toBe(true);
        }
      }
    }, 120_000);
  }
});
