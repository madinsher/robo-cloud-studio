/**
 * T5 xacro feature parity: every case under tests/fixtures/ros_gt/xacro_cases was expanded by the real ROS 2 Jazzy
 * `xacro` (WSL); `gt/<case>.urdf` is its output, a case with only `gt/<case>.err` is rejected by ROS. The studio must
 * produce the same element tree (tags, every attribute string, text) or throw.
 */
import { describe, it, expect } from 'vitest';
import { readdirSync, existsSync } from 'node:fs';
import { expandXacro, processXacro, XacroError } from '../src/io/urdf/xacro';
import { FIX, diffXml, diskResolver, read, join } from './ros_gt_util';

const SETS: Record<string, { pkg: string; args?: Record<string, Record<string, string>> }> = {
  cal: { pkg: 'xc_cal' },
  xc: { pkg: 'xc_probe' },
};

for (const [set, cfg] of Object.entries(SETS)) {
  const dir = join(FIX, 'xacro_cases', set);
  const cases = readdirSync(dir).filter((f) => f.endsWith('.xacro')).sort();
  describe(`xacro feature cases (${set}) vs ROS 2 xacro`, () => {
    it('has ground truth for every case', () => {
      expect(cases.length).toBeGreaterThanOrEqual(set === 'cal' ? 40 : 18);
      for (const c of cases) expect(existsSync(join(dir, 'gt', c.replace(/\.xacro$/, '.urdf'))) || existsSync(join(dir, 'gt', c.replace(/\.xacro$/, '.err')))).toBe(true);
    });
    for (const c of cases) {
      const base = c.replace(/\.xacro$/, '');
      const gtUrdf = join(dir, 'gt', `${base}.urdf`);
      const ok = existsSync(gtUrdf);
      it(`${base}: ${ok ? 'same tree as ROS' : 'rejected like ROS'}`, () => {
        const run = () => expandXacro(read(join(dir, c)), { filename: c, resolveInclude: diskResolver(dir, { [cfg.pkg]: dir }), packages: { [cfg.pkg]: dir }, env: {} });
        if (ok) {
          const res = run();
          expect(diffXml(res.xml, read(gtUrdf))).toEqual([]);
        } else {
          const gtErr = read(join(dir, 'gt', `${base}.err`));
          const key = /error: (.*)/.exec(gtErr)?.[1].replace(/<class '[^']*'>: /, '').slice(0, 30) ?? '';
          let thrown: unknown = null;
          try { run(); } catch (e) { thrown = e; }
          expect(thrown, `ROS rejects ${base}: ${gtErr.split('\n')[0]}`).toBeInstanceOf(XacroError);
          expect(String((thrown as Error).message)).toContain(key.split(' ').slice(0, 3).join(' '));
        }
      });
    }
  });
}

describe('xacro sandbox and error surfacing', () => {
  it('never executes JavaScript from expressions', () => {
    (globalThis as any).__t5_pwned = undefined;
    const src = '<robot xmlns:xacro="http://www.ros.org/wiki/xacro"><link name="${(globalThis.__t5_pwned = 1, \'base\')}"/></robot>';
    expect(() => processXacro(src)).toThrow(XacroError);
    expect(() => processXacro('<robot xmlns:xacro="x"><v a="${constructor.constructor(\'return 1\')()}"/></robot>')).toThrow(/not defined|invalid/);
    expect(() => processXacro('<robot xmlns:xacro="x"><v a="${\'\'.__class__}"/></robot>')).toThrow(/invalid name/);
    expect((globalThis as any).__t5_pwned).toBeUndefined();
  });
  it('lenient mode reports problems as warnings and keeps going', () => {
    const warnings: string[] = [];
    const out = processXacro('<robot xmlns:xacro="x"><xacro:include filename="nope.xacro"/><xacro:unknown/><link name="${undefined_name}"/><link name="ok"/></robot>', { lenient: true, warnings });
    expect(out).toContain('<link name="ok"/>');
    expect(out).toContain('${undefined_name}');
    expect(warnings.some((w) => /nope\.xacro/.test(w))).toBe(true);
    expect(warnings.some((w) => /unknown macro name: xacro:unknown/.test(w))).toBe(true);
    expect(warnings.some((w) => /undefined_name/.test(w))).toBe(true);
  });
  it('reports includes like xacro --deps and resolves them relative to the including file', () => {
    const files: Record<string, string> = {
      'pkg/urdf/top.xacro': '<robot xmlns:xacro="x"><xacro:include filename="parts/a.xacro"/><xacro:include filename="$(find pkg)/urdf/b.xacro"/></robot>',
      'pkg/urdf/parts/a.xacro': '<robot xmlns:xacro="x"><xacro:include filename="../c.xacro"/><link name="a"/></robot>',
      'pkg/urdf/c.xacro': '<robot><link name="c"/></robot>',
      'pkg/urdf/b.xacro': '<robot><link name="b"/></robot>',
    };
    const res = expandXacro(files['pkg/urdf/top.xacro'], { filename: 'pkg/urdf/top.xacro', packages: { pkg: 'pkg' }, resolveInclude: (p) => files[p.replace(/^package:\/\/pkg\//, 'pkg/')] ?? null });
    expect(res.includes).toEqual(['pkg/urdf/parts/a.xacro', 'pkg/urdf/c.xacro', 'package://pkg/urdf/b.xacro']);
    expect([...res.xml.matchAll(/link name="(\w)"/g)].map((m) => m[1])).toEqual(['c', 'a', 'b']);
    expect(() => expandXacro('<robot xmlns:xacro="x"><xacro:include filename="$(find other)/x.xacro"/></robot>', { packages: { pkg: 'pkg' } })).toThrow(/package 'other' not found/);
  });
});
