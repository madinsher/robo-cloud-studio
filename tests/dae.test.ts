import { it, expect } from 'vitest';
import { parseDAE } from '../src/io/mesh/dae';

it('parses a COLLADA polylist with a node transform and unit scale', () => {
  const dae = `<?xml version="1.0"?><COLLADA><asset><unit meter="0.001"/><up_axis>Z_UP</up_axis></asset>
  <library_geometries><geometry id="g"><mesh>
    <source id="pos"><float_array id="pa" count="12">0 0 0 1 0 0 1 1 0 0 1 0</float_array><technique_common><accessor source="#pa" count="4" stride="3"/></technique_common></source>
    <source id="nrm"><float_array id="na" count="3">0 0 1</float_array><technique_common><accessor source="#na" count="1" stride="3"/></technique_common></source>
    <vertices id="v"><input semantic="POSITION" source="#pos"/></vertices>
    <polylist count="1"><input semantic="VERTEX" source="#v" offset="0"/><input semantic="NORMAL" source="#nrm" offset="1"/><vcount>4</vcount><p>0 0 1 0 2 0 3 0</p></polylist>
  </mesh></geometry></library_geometries>
  <library_visual_scenes><visual_scene id="s"><node><translate>10 0 0</translate><instance_geometry url="#g"/></node></visual_scene></library_visual_scenes></COLLADA>`;
  const m = parseDAE(dae);
  expect(m.triangles).toBe(2);
  // unit 0.001 m -> mm scale 1, translate 10
  expect(m.min[0]).toBeCloseTo(10);
  expect(m.max[0]).toBeCloseTo(11);
  expect(m.normals[2]).toBeCloseTo(1);
});
