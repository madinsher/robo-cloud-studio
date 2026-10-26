/**
 * Webots VRML_SIM lexer / parser, PROTO interface parsing, the JavaScript template engine and PROTO resolution
 * (EXTERNPROTO relative to the declaring file, webots:// -> built-ins, by-name fallback, IS, cycles, Lua).
 */
import { describe, it, expect, afterEach } from 'vitest';
import { parseWorld, parseProtoFile, parseNodes, countTypesInText, Lexer, WebotsSyntaxError, fNode, fNodes, fNum, fStrs } from '../src/io/webots/vrml';
import { expandTemplate, compileTemplate, TemplateError } from '../src/io/webots/template';
import { ProtoRegistry, resolveUrl } from '../src/io/webots/proto';
import { mapSource } from '../src/io/webots';

const enc = (s: string) => new TextEncoder().encode(s);
const ctx = { world: 'w.wbt', proto: 'p.proto', id: 'n1', coordinate_system: 'ENU' as const, webots_version: { major: 'R2025', revision: 'a' }, os: 'linux', webots_home: '', temporary_files_path: '/tmp/' };

describe('lexer and parser', () => {
  it('tokenises comments, strings with escapes and #, commas, hex and exponent numbers', () => {
    const lx = new Lexer('a 0x1F, -1.5e-3 .5 +2 "q \\"x\\" # y" # comment\n b', 't');
    const toks = [];
    for (let t = lx.next(); t.t !== 'eof'; t = lx.next()) toks.push(t.t === 'num' ? t.n : t.v);
    expect(toks).toEqual(['a', 31, -0.0015, 0.5, 2, 'q "x" # y', 'b']);
  });

  it('parses a world: header, (IMPORTABLE) EXTERNPROTO, DEF/USE sharing, NULL, MF values with and without brackets', () => {
    const w = parseWorld(`#VRML_SIM R2023b utf8
# комментарий
EXTERNPROTO "../protos/A.proto"
IMPORTABLE EXTERNPROTO "B.proto"
DEF S Solid { name "s" children [ DEF SH Shape { geometry Box { size 1, 2, 3 } } ] physics NULL }
Solid { children [ USE SH ] boundingObject USE SH }
Shape { geometry Mesh { url "m.obj" } }
Group { children [ ] }
`, 'w/w.wbt');
    expect(w.version).toBe('R2023b');
    expect(w.externs).toEqual([{ url: '../protos/A.proto', importable: false }, { url: 'B.proto', importable: true }]);
    expect(w.nodes.map((n) => n.type)).toEqual(['Solid', 'Solid', 'Shape', 'Group']);
    const sh = fNodes(w.nodes[0], 'children')[0];
    expect(fNodes(w.nodes[1], 'children')[0]).toBe(sh); // USE shares the DEF'd node
    expect(fNode(w.nodes[1], 'boundingObject')).toBe(sh);
    expect(fNum(fNode(sh, 'geometry'), 'size', [])).toEqual([1, 2, 3]);
    expect(w.nodes[0].fields.physics).toEqual({ k: 'node', v: null });
    expect(fStrs(fNode(w.nodes[2], 'geometry'), 'url')).toEqual(['m.obj']);
    expect(w.nodes[3].fields.children).toEqual({ k: 'nodes', v: [] });
    expect(w.nodes[0].base).toBe('w/w.wbt');
  });

  it('reports syntax errors with the line number and warns on USE before DEF', () => {
    expect(() => parseWorld('#VRML_SIM R2025a utf8\nSolid {\n name "x\n', 'bad.wbt')).toThrow(WebotsSyntaxError);
    expect(() => parseWorld('#VRML_SIM R2025a utf8\nSolid {\n\n  children [ Shape { ] }', 'bad.wbt')).toThrow(/bad\.wbt:4/);
    const scope = { defs: new Map(), warnings: [] as string[] };
    parseNodes('Solid { children [ USE NOPE ] }', 'x', scope);
    expect(scope.warnings[0]).toMatch(/USE NOPE before DEF/);
  });

  it('parses a PROTO interface: multi-line enumerations, node defaults, MF defaults without brackets, body end', () => {
    const p = parseProtoFile(`#VRML_SIM R2025a utf8
# template language: javascript
# tags: nonDeterministic
EXTERNPROTO "webots://projects/appearances/protos/Parquetry.proto"
PROTO Thing [
  field SFString{"a b",
                 "c"}    mode "c"   # comment with ]
  field SFNode     appearance Parquetry { type "chequered" }
  hiddenField MFString texture "webots://x.jpg"
  field MFVec3f    pts [ 0 0 0, 1 1 1 ]
  unconnectedField SFBool flag FALSE
]
{
  Solid { name "%<= fields.mode.value >%" }
}
# } trailing
`, 'protos/Thing.proto');
    expect(p.name).toBe('Thing');
    expect(p.template).toBe('javascript');
    expect(p.meta.tags).toBe('nonDeterministic');
    expect(p.externs[0].url).toMatch(/Parquetry/);
    expect(p.fields.map((f) => `${f.kind}:${f.type}:${f.name}`)).toEqual(['field:SFString:mode', 'field:SFNode:appearance', 'hiddenField:MFString:texture', 'field:MFVec3f:pts', 'unconnectedField:SFBool:flag']);
    expect(p.fields[0].enumText).toContain('"a b"');
    expect(p.fields[1].def.k).toBe('node');
    expect(p.fields[2].def).toEqual({ k: 'str', v: ['webots://x.jpg'] });
    expect(p.fields[3].def).toEqual({ k: 'num', v: [0, 0, 0, 1, 1, 1] });
    expect(p.body.trim()).toBe('Solid { name "%<= fields.mode.value >%" }');
  });

  it('counts node types textually (USE not counted), like the investigator inventory', () => {
    expect(countTypesInText('DEF A Shape { geometry Box {} } Solid { children [ USE A Shape { } ] }')).toEqual({ Shape: 2, Box: 1, Solid: 1 });
  });

  it('resolves URLs relative to the declaring file', () => {
    expect(resolveUrl('../../../assets/m.stl', 'ws/src/pkg/protos/UTP_G.proto')).toBe('ws/assets/m.stl');
    expect(resolveUrl('webots://projects/x.proto', 'a/b.wbt')).toBe('webots://projects/x.proto');
  });
});

describe('JavaScript template engine', () => {
  afterEach(() => { delete (globalThis as Record<string, unknown>).window; });
  const F = (v: Record<string, unknown>) => Object.fromEntries(Object.entries(v).map(([k, x]) => [k, { value: x, defaultValue: x }]));

  it('expands loops, conditions and expressions with Webots value shapes', () => {
    const body = `%< for (let i = 0; i < fields.levels.value.length; ++i) { >%B{z %<= fields.levels.value[i] * 2 >%}%< } >%%< if (fields.on.value) { >%ON%< } else { >%OFF%< } >% %<= fields.size.value.x + fields.size.value.z >% %<= fields.c.value.g >% %<= fields.r.value.a >%`;
    const out = expandTemplate(body, F({ levels: [1, 2.5], on: false, size: { x: 1, y: 2, z: 3 }, c: { r: 0, g: 0.5, b: 1 }, r: { x: 0, y: 0, z: 1, a: 1.5 } }), ctx);
    expect(out).toBe('B{z 2}B{z 5}OFF 4 0.5 1.5');
  });

  it('gives access to context and the template module shims', () => {
    const body = `%< import * as wbvector3 from 'wbvector3.js'; import { circle } from 'wbgeometry.js'; >%%<= context.coordinate_system >% %<= wbvector3.norm(wbvector3.add({x:1,y:2,z:0},{x:2,y:2,z:0})) >% %<= circle(1, 6, {x: 0, y: 0}, 0).length >%`;
    expect(expandTemplate(body, {}, ctx)).toBe('ENU 5 7'); // Webots' circle is closed: div + 1 points
    expect(compileTemplate('%< import * as R from "wbrotation.js"; >%')).toContain('__imports("wbrotation.js")');
  });

  it('isolates studio globals and stops runaway templates', () => {
    (globalThis as Record<string, unknown>).window = { secret: 42 };
    expect(expandTemplate('%<= typeof window >% %<= typeof fetch >% %<= typeof document >%', {}, ctx)).toBe('undefined undefined undefined');
    expect(() => expandTemplate('%< while (true) { } >%', {}, ctx, { maxIterations: 1000 })).toThrow(TemplateError);
    expect(() => expandTemplate('%< for (;;) { >%x%< } >%', {}, ctx, { maxIterations: 1e9, maxOutput: 1000 })).toThrow(/limit/);
    expect(() => expandTemplate('%<= fields.nope.value.x >%', {}, ctx)).toThrow(/template error/);
    expect(() => expandTemplate('%< import * as M from "fs"; >%', {}, ctx)).toThrow(/unsupported template module/);
  });
});

describe('PROTO resolution', () => {
  it('expands a template PROTO with instance fields and IS, keeps defaults, maps webots:// to built-ins', () => {
    const src = mapSource(new Map([
      ['p/protos/Bar.proto', enc(`#VRML_SIM R2025a utf8
# template language: javascript
EXTERNPROTO "webots://projects/objects/floors/protos/Floor.proto"
PROTO Bar [ field SFFloat len 1 field SFString name "bar" field SFColor color 1 0 0 field MFNode extra [] ]
{ Solid { name IS name children [ Shape { appearance PBRAppearance { baseColor IS color } geometry Box { size %<= fields.len.value >% 0.1 0.1 } } Floor { size 3 2 } ] boundingObject Group { children IS extra } } }`)],
    ]));
    const reg = new ProtoRegistry(src);
    reg.registerScope('p/worlds/w.wbt', [{ url: '../protos/Bar.proto', importable: false }]);
    const [inst] = parseNodes('Bar { len 2.5 extra [ Box { size 1 1 1 } ] }', 'p/worlds/w.wbt');
    const n = reg.resolve(inst)!;
    expect(n.type).toBe('Solid');
    expect(n.proto!.name).toBe('Bar');
    expect(n.fields.name).toEqual({ k: 'str', v: ['bar'] }); // default
    const kids = fNodes(n, 'children');
    expect(fNum(fNode(kids[0], 'geometry'), 'size', [])).toEqual([2.5, 0.1, 0.1]);
    expect(fNum(fNode(kids[0], 'appearance'), 'baseColor', [])).toEqual([1, 0, 0]);
    expect(fNodes(fNode(n, 'boundingObject'), 'children')[0].type).toBe('Box'); // MFNode IS from the instance
    const floor = reg.resolve(kids[1])!;
    expect(floor.type).toBe('Solid');
    expect(floor.meta?.builtin).toBe('Floor');
    expect(reg.report).toEqual([]);
  });

  it('built-ins reset non-positive sizes like Webots (RectangleArena wallHeight 0 -> 0.1 m walls) and resolve their own default appearances', () => {
    const reg = new ProtoRegistry(mapSource(new Map()));
    reg.registerScope('w.wbt', [{ url: 'webots://projects/objects/floors/protos/RectangleArena.proto', importable: false }]);
    const [inst] = parseNodes('RectangleArena { floorSize 4 2 wallHeight 0.0 wallThickness -1 }', 'w.wbt');
    const n = reg.resolve(inst)!;
    const walls = fNodes(n, 'children').filter((c) => c.type === 'Pose');
    expect(walls.map((w) => [...fNum(w, 'translation', []), ...fNum(fNode(fNodes(w, 'children')[0], 'geometry'), 'size', [])].map((v) => Math.round(v * 1e4) / 1e4))).toEqual([
      [2.005, 0, 0.05, 0.01, 2, 0.1], [-2.005, 0, 0.05, 0.01, 2, 0.1], [0, 1.005, 0.05, 4.02, 0.01, 0.1], [0, -1.005, 0.05, 4.02, 0.01, 0.1],
    ]);
    // the default wall appearance (BrushedAluminium, created by the built-in itself) resolves instead of being reported as a cycle
    const app = reg.resolve(fNode(fNodes(walls[0], 'children')[0], 'appearance')!)!;
    expect(app.type).toBe('PBRAppearance');
    expect(reg.report).toEqual([
      "RectangleArena: 'wallThickness' must be strictly positive, reset to 0.01 (as Webots does)",
      "RectangleArena: 'wallHeight' must be strictly positive, reset to 0.1 (as Webots does)",
    ]);
  });

  it('reports Lua templates, recursive PROTOs and unknown fields without throwing', () => {
    const src = mapSource(new Map([
      ['L.proto', enc('#VRML_SIM R2022a utf8\n# template language: lua\nPROTO L [ ]\n{ Solid { name "%{= 1 }%" } }')],
      ['R.proto', enc('#VRML_SIM R2025a utf8\nEXTERNPROTO "R.proto"\nPROTO R [ ]\n{ R { } }')],
    ]));
    const reg = new ProtoRegistry(src);
    reg.registerScope('w.wbt', [{ url: 'L.proto', importable: false }, { url: 'R.proto', importable: false }]);
    const nodes = parseNodes('L { } R { } L { bogus 1 }', 'w.wbt');
    expect(nodes.map((n) => reg.resolve(n))).toEqual([null, null, null]);
    expect(reg.report.some((l) => l.includes('Lua template PROTOs are not supported'))).toBe(true);
    expect(reg.report.some((l) => l.includes('recursive PROTO'))).toBe(true);
    expect(reg.report.some((l) => l.includes("unknown field 'bogus'"))).toBe(true);
  });
});
