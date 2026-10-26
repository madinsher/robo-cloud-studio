// @vitest-environment jsdom
/**
 * Webots PROTO JavaScript templates are untrusted code: they run in an isolated realm (node:vm context; in the browser
 * a detached iframe) with code generation disabled and the realm's globals shadowed. The verifier's escape
 * (`({}).constructor.constructor("return process")()`, which returned the Node pid) and its variants must fail, and
 * nothing a template does may reach or change the studio's own globals.
 */
import { describe, it, expect } from 'vitest';
import { expandTemplate, createTemplateRealm, TemplateError, TemplateContext } from '../src/io/webots/template';

const ctx: TemplateContext = { world: 'w', proto: 'p', id: 1, coordinate_system: 'ENU', webots_version: { major: 'R2025', revision: 'a' }, os: 'linux', webots_home: '', temporary_files_path: '/tmp/' };
const ESCAPES: Array<[string, string]> = [
  ['Function constructor via an object', `%<= ({}).constructor.constructor("return typeof process === 'object' ? 'PID=' + process.pid : 'no-process'")() >%`],
  ['Function constructor via a function', `%<= (function () {}).constructor("return this")() >%`],
  ['generator function constructor', `%<= Object.getPrototypeOf(function* () {}).constructor("yield this")().next().value >%`],
  ['async function constructor', `%<= typeof Object.getPrototypeOf(async function () {}).constructor("return 1") >%`],
  ['eval', `%<= eval("1 + 1") >%`],
  ['indirect eval', `%<= (0, [eval][0])("this") >%`],
  ['the template engine itself', `%<= __engine.expand >%`],
  ['dynamic import', `%< import("node:fs").then((fs) => fs.writeFileSync("pwned", "x")); >%`],
];
const realms: Array<['vm' | 'iframe', () => ReturnType<typeof createTemplateRealm>]> = [['vm', () => createTemplateRealm('vm')], ['iframe', () => createTemplateRealm('iframe')]];

describe.each(realms)('PROTO template sandbox (%s realm)', (kind, make) => {
  const realm = make();
  it('the realm exists and expands a normal template', () => {
    expect(realm?.kind).toBe(kind);
    const out = expandTemplate('%< import * as wbgeometry from "wbgeometry.js"; const c = wbgeometry.circle(1, 4); >%n=%<= c.length >% %<= fields.size.value.x * 2 >%', { size: { value: { x: 1.5, y: 0, z: 0 }, defaultValue: { x: 1, y: 0, z: 0 } } }, ctx, { realm: realm! });
    expect(out).toBe('n=5 3');
  });
  for (const [label, body] of ESCAPES) {
    it(`blocks: ${label}`, () => {
      let out: string | null = null;
      try { out = expandTemplate(body, {}, ctx, { realm: realm! }); } catch (e) { expect(e).toBeInstanceOf(TemplateError); }
      if (out !== null) expect(out).toMatch(/^(undefined|)$/); // a name the template cannot resolve shadows to undefined
      expect(out ?? '').not.toMatch(/PID=|\[object (global|Window)/);
    });
  }
  it('host globals are not reachable by name and stack-trace hooks see no receiver / function', () => {
    expect(expandTemplate('%<= [typeof process, typeof require, typeof globalThis, typeof window, typeof document, typeof fetch, typeof localStorage, typeof Function, typeof this].join() >%', {}, ctx, { realm: realm! })).toBe('undefined,undefined,undefined,undefined,undefined,undefined,undefined,undefined,undefined');
    const r = expandTemplate('%< Error.prepareStackTrace = (e, cs) => cs.map((c) => typeof c.getThis() + ":" + typeof c.getFunction()).join(" "); const s = String(new Error("x").stack); Error.prepareStackTrace = undefined; >%%<= s >%', {}, ctx, { realm: realm! });
    expect(r).not.toMatch(/object:|function/);
  });
  it('changes to built-ins stay inside the realm (no pollution of the studio)', () => {
    expandTemplate('%< Array.prototype.vbsPolluted = 1; Object.prototype.then = function () {}; JSON.parse = null; >%', {}, ctx, { realm: realm! });
    expect(([] as unknown as { vbsPolluted?: number }).vbsPolluted).toBeUndefined();
    expect(({} as { then?: unknown }).then).toBeUndefined();
    expect(JSON.parse('1')).toBe(1);
  });
  it('loops without braces are guarded too', () => {
    expect(() => expandTemplate('%< while (1); >%', {}, ctx, { realm: realm!, maxIterations: 1000 })).toThrow(/loop limit/);
    expect(() => expandTemplate('%< for (;;) ; >%', {}, ctx, { realm: realm!, maxIterations: 1000 })).toThrow(/loop limit/);
    expect(() => expandTemplate('%< let i = 0; do i++; while (true); >%', {}, ctx, { realm: realm!, maxIterations: 1000 })).toThrow(/loop limit/);
  });
});
