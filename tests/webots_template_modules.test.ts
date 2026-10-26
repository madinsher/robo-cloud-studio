/**
 * The template modules (wbgeometry, wbrandom, wbrotation, wbvector2/3, wbutility) give the same results as Webots' own
 * JavaScript modules: the reference is the installed Webots (resources/web/wwi/protoVisualizer/templating/modules/webots),
 * imported and called with the same arguments. Skipped (visibly) without a Webots installation.
 */
import { describe, it, expect, beforeAll } from 'vitest';
import fs from 'node:fs';
import path from 'node:path';
import { pathToFileURL } from 'node:url';
import { templateModules, expandTemplate } from '../src/io/webots/template';
import { WEBOTS_HOME } from './webots_helpers';

const MOD_DIR = path.join(WEBOTS_HOME, 'resources/web/wwi/protoVisualizer/templating/modules/webots');
const HAS_MODULES = fs.existsSync(path.join(MOD_DIR, 'wbrandom.js'));
type Mod = Record<string, (...a: unknown[]) => unknown>;

describe.skipIf(!HAS_MODULES)('template modules equal the Webots JavaScript modules', () => {
  const ref: Record<string, Mod> = {};
  beforeAll(async () => {
    const g = globalThis as Record<string, unknown>;
    g.stderr = g.stderr ?? []; g.stdout = g.stdout ?? []; // Webots' wbutility pushes into these globals
    for (const m of ['wbrandom', 'wbgeometry', 'wbrotation', 'wbvector2', 'wbvector3', 'wbutility']) ref[m] = (await import(pathToFileURL(path.join(MOD_DIR, `${m}.js`)).href)) as Mod;
  });
  const ours = () => templateModules() as Record<string, Mod>;

  it('wbrandom: the same LCG sequence for real / integer with seeds, ranges and defaults', () => {
    const a = ours().wbrandom, b = ref.wbrandom;
    for (const s of [0, 1, 42, 123456, 16777215]) {
      a.seed(s); b.seed(s);
      const seqA = [a.real(), a.real(2, 5), a.real(3), a.integer(), a.integer(10), a.integer(-4, 4), a.real(-1, 1)];
      const seqB = [b.real(), b.real(2, 5), b.real(3), b.integer(), b.integer(10), b.integer(-4, 4), b.real(-1, 1)];
      expect(seqA).toEqual(seqB);
    }
  });

  it('wbgeometry: circle, polygon tests, closest point, orientation and B-splines', () => {
    const a = ours().wbgeometry, b = ref.wbgeometry;
    expect(a.circle(1.5, 24, { x: 0.5, y: -2 }, -Math.PI / 2)).toEqual(b.circle(1.5, 24, { x: 0.5, y: -2 }, -Math.PI / 2));
    const poly = [{ x: 0, y: 0 }, { x: 4, y: 0 }, { x: 4, y: 3 }, { x: 2, y: 5 }, { x: 0, y: 3 }];
    for (const p of [{ x: 1, y: 1 }, { x: 3.9, y: 3.1 }, { x: 2, y: 5 }, { x: -1, y: 1 }, { x: 4, y: 1.5 }]) expect(a.isPoint2InPolygon(p, poly)).toBe(b.isPoint2InPolygon(p, poly));
    expect(a.findClosestPoint2InArray({ x: 3, y: 4 }, poly)).toEqual(b.findClosestPoint2InArray({ x: 3, y: 4 }, poly));
    expect(a.isPoint2ArrayClockwise(poly)).toBe(b.isPoint2ArrayClockwise(poly));
    expect(a.isPoint2ArrayClockwise([...poly].reverse())).toBe(b.isPoint2ArrayClockwise([...poly].reverse()));
    const p2 = () => [{ x: 0, y: 0 }, { x: 1, y: 2 }, { x: 3, y: 2.5 }, { x: 4, y: 0 }];
    expect(a.bSpline2(p2(), 5)).toEqual(b.bSpline2(p2(), 5));
    const p3 = () => [{ x: 0, y: 0, z: 0 }, { x: 1, y: 2, z: 1 }, { x: 3, y: 2.5, z: -1 }, { x: 4, y: 0, z: 0.5 }, { x: 6, y: 1, z: 0 }];
    expect(a.bSpline3(p3(), 4)).toEqual(b.bSpline3(p3(), 4));
  });

  it('wbrotation and wbvector2/3: conversions, combination, rotation of vectors, segment intersection', () => {
    const A = ours(), B = ref;
    const r = () => ({ x: 0.3, y: -0.5, z: 0.8, a: 2.2 });
    for (const f of ['toQuaternion', 'toMatrix3', 'normalize']) expect(A.wbrotation[f](r())).toEqual(B.wbrotation[f](r()));
    expect(A.wbrotation.fromQuaternion({ w: 0.6, x: 0.48, y: 0, z: 0.64 })).toEqual(B.wbrotation.fromQuaternion({ w: 0.6, x: 0.48, y: 0, z: 0.64 }));
    const m = B.wbrotation.toMatrix3(B.wbrotation.normalize(r()));
    expect(A.wbrotation.fromMatrix3(m)).toEqual(B.wbrotation.fromMatrix3(m));
    expect(A.wbrotation.combine(r(), { x: 0, y: 0, z: 1, a: -0.7 })).toEqual(B.wbrotation.combine(r(), { x: 0, y: 0, z: 1, a: -0.7 }));
    expect(A.wbrotation.rotateVector3ByRotation(B.wbrotation.normalize(r()), { x: 1, y: 2, z: 3 })).toEqual(B.wbrotation.rotateVector3ByRotation(B.wbrotation.normalize(r()), { x: 1, y: 2, z: 3 }));
    expect(A.wbrotation.rotateVector3ByQuaternion({ w: 0.6, x: 0.48, y: 0, z: 0.64 }, { x: 1, y: 0, z: 0 })).toEqual(B.wbrotation.rotateVector3ByQuaternion({ w: 0.6, x: 0.48, y: 0, z: 0.64 }, { x: 1, y: 0, z: 0 }));
    const s = [[{ x: 0, y: 0 }, { x: 4, y: 4 }, { x: 0, y: 4 }, { x: 4, y: 0 }], [{ x: 0, y: 0 }, { x: 0, y: 4 }, { x: -1, y: 1 }, { x: 3, y: 2 }], [{ x: 0, y: 0 }, { x: 1, y: 1 }, { x: 2, y: 0 }, { x: 3, y: 1 }]];
    for (const [p1, p2, p3, p4] of s) expect(A.wbvector2.intersection(p1, p2, p3, p4)).toEqual(B.wbvector2.intersection(p1, p2, p3, p4));
    for (const f of ['angle', 'cross', 'dot', 'distance']) expect(A.wbvector2[f]({ x: 1, y: 2 }, { x: -3, y: 0.5 })).toBe(B.wbvector2[f]({ x: 1, y: 2 }, { x: -3, y: 0.5 }));
    expect(A.wbvector3.cross({ x: 1, y: 2, z: 3 }, { x: -1, y: 0.5, z: 2 })).toEqual(B.wbvector3.cross({ x: 1, y: 2, z: 3 }, { x: -1, y: 0.5, z: 2 }));
    expect(A.wbvector3.normalize({ x: 3, y: 0, z: 4 })).toEqual(B.wbvector3.normalize({ x: 3, y: 0, z: 4 }));
  });
});

describe('template engine diagnostics', () => {
  it('console.error / wbutility.error lines reach the caller; every evaluation starts with seed 0', () => {
    const ctx = { world: 'w.wbt', proto: 'p.proto', id: 7, coordinate_system: 'ENU' as const, webots_version: { major: 'R2025', revision: 'a' }, os: 'linux', webots_home: '', temporary_files_path: '/tmp/' };
    const messages: string[] = [];
    const body = `%< import * as wbrandom from 'wbrandom.js'; import * as wbutility from 'wbutility.js'; console.error('size must be positive'); wbutility.error('bad'); >%%<= wbrandom.integer(1000) >%`;
    const a = expandTemplate(body, {}, ctx, { messages });
    expect(messages).toEqual(['size must be positive', 'bad']);
    expect(expandTemplate(body, {}, ctx)).toBe(a); // module state is per evaluation
    expect(Number(a)).toBe(1 + (12820163 % 1000)); // first LCG step from seed 0
  });
});
