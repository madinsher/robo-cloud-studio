import { describe, it, expect } from 'vitest';
import { unzipSync, strFromU8 } from 'fflate';
import { parseXML, serializeXML, childElements, childElement, nums } from '../src/io/urdf/xml';
import { processXacro } from '../src/io/urdf/xacro';
import { parseURDF, chainFromURDF, robotFromURDF } from '../src/io/urdf/urdf';
import { exportRobotURDF, exportStationURDF, packageZip, sanitizeName, chainToURDF } from '../src/io/urdf/urdf_export';
import { AssetStore } from '../src/scene/assets';
import { writeBinarySTL, parseSTL } from '../src/io/mesh/stl';
import { Station, SceneObject, Frame, Tool } from '../src/core/items/item';
import { Robot } from '../src/core/items/robot';
import { createRobotFromLibrary } from '../src/core/items/library';
import { chainFromDH } from '../src/core/kinematics/chain';
import { transl, mul, rotz, DEG, getPos, distance } from '../src/core/math/pose';

describe('tiny XML parser', () => {
  it('parses CDATA, comments, entities, unquoted attributes and processing instructions', () => {
    const doc = parseXML(`<?xml version="1.0"?><!DOCTYPE x><root a=1 b='two' c="a &amp; b &lt;c&gt; &#65; &quot;q&quot; &apos;s&apos;" flag>
      <!-- ignored --><child><![CDATA[raw <text> & stuff]]></child><empty/>text &amp; more</root>`);
    expect(doc.name).toBe('root');
    expect(doc.attrs).toEqual({ a: '1', b: 'two', c: 'a & b <c> A "q" \'s\'', flag: '' });
    const child = childElement(doc, 'child')!;
    expect(child.children[0].text).toBe('raw <text> & stuff');
    expect(childElements(doc).map((c) => c.name)).toEqual(['child', 'empty']);
    expect(doc.children.find((c) => c.type === 'text')?.text).toBe('text & more');
    expect(parseXML('just text').name).toBe('#document');
  });
  it('serializes back with escaping and indentation', () => {
    const doc = parseXML('<a x="1&amp;2"><b>t&lt;</b><c/></a>');
    const out = serializeXML(doc);
    expect(out).toBe('<a x="1&amp;2">\n  <b>\nt&lt;  </b>\n  <c/>\n</a>\n');
    expect(serializeXML({ type: 'text', name: '#text', attrs: {}, children: [], text: '<' })).toBe('&lt;');
  });
  it('nums() falls back on malformed input', () => {
    expect(nums('1 2 3', [0])).toEqual([1, 2, 3]);
    expect(nums('1 x', [9])).toEqual([9]);
    expect(nums(undefined, [7])).toEqual([7]);
  });
});

describe('xacro preprocessor', () => {
  it('expands properties, expressions, args, if/unless, macros with defaults and blocks, includes', () => {
    const inc = new Map<string, string>([['common.xacro', '<robot><xacro:property name="from_include" value="3"/></robot>']]);
    const out = processXacro(`<robot name="r" xmlns:xacro="http://wiki.ros.org/xacro">
      <xacro:arg name="prefix" default="arm_"/>
      <xacro:arg name="use_gripper" default="true"/>
      <xacro:include filename="$(find my_pkg)/common.xacro"/>
      <xacro:include filename="common.xacro"/>
      <xacro:include filename="missing.xacro"/>
      <xacro:property name="len" value="\${2 * 0.5 + from_include}"/>
      <xacro:property name="quoted" value="'hello'"/>
      <xacro:property name="origin_block"><origin xyz="1 2 3"/></xacro:property>
      <xacro:macro name="seg" params="name mass:=1.5 *shape">
        <link name="$(arg prefix)\${name}" mass="\${mass}" pi="\${pi}" deg="\${degrees(pi/2)}" rad="\${radians(180)}" q="\${quoted}">
          <xacro:insert_block name="shape"/>
          <xacro:insert_block name="origin_block"/>
        </link>
      </xacro:macro>
      <xacro:seg name="a"><box size="1 1 \${len}"/></xacro:seg>
      <seg name="b" mass="2"><sphere/></seg>
      <xacro:if value="$(arg use_gripper)"><gripper/></xacro:if>
      <xacro:if value="\${len > 100 and True}"><never/></xacro:if>
      <xacro:unless value="0"><always/></xacro:unless>
      <xacro:unless value="\${not False}"><never2/></xacro:unless>
      <text>\${len} \${unknown_symbol}</text>
    </robot>`, { resolveInclude: (f) => inc.get(f.replace(/^package:\/\/my_pkg\//, '')) ?? null, args: { use_gripper: 'false' } });
    expect(out).toContain('<link name="arm_a" mass="1.5" pi="3.141592653589793" deg="90" rad="3.141592653589793" q="hello">');
    expect(out).toContain('<box size="1 1 4"/>');
    expect(out).toContain('<origin xyz="1 2 3"/>');
    expect(out).toContain('<link name="arm_b" mass="2"');
    expect(out).toContain('<sphere/>');
    expect(out).not.toContain('<gripper/>'); // command-line arg overrides the default
    expect(out).not.toContain('<never');
    expect(out).toContain('<always/>');
    expect(out).toContain('4 unknown_symbol'); // unknown identifiers are left as text
    expect(out).not.toContain('xmlns:xacro');
  });
  it('supports python-style operators and ** in expressions', () => {
    const out = processXacro('<r><xacro:property name="a" value="2"/><v x="${a ** 3}" y="${a == 2 or False}" z="${max(a, 5)}" w="${abs(-1)}"/></r>');
    expect(out).toContain('x="8"');
    expect(out).toContain('y="true"');
    expect(out).toContain('z="5"');
    expect(out).toContain('w="1"');
  });
});

describe('URDF import', () => {
  const urdf = `<?xml version="1.0"?>
<robot name="prims">
  <material name="red"><color rgba="1 0 0 1"/></material>
  <material name="nocolor"/>
  <link name="base"><visual><origin xyz="0 0 0.1"/><geometry><box size="0.2 0.3 0.4"/></geometry><material name="red"/></visual>
    <collision><geometry><cylinder radius="0.05" length="0.2"/></geometry></collision>
    <inertial><mass value="4"/></inertial></link>
  <link name="l1"><visual><geometry><cylinder radius="0.1" length="0.5"/></geometry><material name="inline"><color rgba="0 1 0 1"/></material></visual></link>
  <link name="l2"><visual><geometry><sphere radius="0.05"/></geometry><material name="nocolor"/></visual><visual><geometry><capsule/></geometry></visual></link>
  <link name="slider"/>
  <link name="finger_l"/><link name="finger_r"/><link name="spinner"/>
  <joint name="j1" type="revolute"><parent link="base"/><child link="l1"/><origin xyz="0 0 0.5" rpy="0 0 1.5707963"/><axis xyz="0 0 1"/><limit lower="-1" upper="1" velocity="2" effort="10"/></joint>
  <joint name="j2" type="continuous"><parent link="l1"/><child link="l2"/><origin xyz="0.3 0 0"/><axis xyz="0 1 0"/></joint>
  <joint name="j3" type="prismatic"><parent link="l2"/><child link="slider"/><axis xyz="1 0 0"/><limit lower="0" upper="0.1" velocity="0.5"/></joint>
  <joint name="fl" type="prismatic"><parent link="slider"/><child link="finger_l"/><axis xyz="0 1 0"/><limit lower="0" upper="0.04"/></joint>
  <joint name="fr" type="prismatic"><parent link="slider"/><child link="finger_r"/><axis xyz="0 -1 0"/><limit lower="0" upper="0.04"/><mimic joint="fl" multiplier="-1" offset="0.01"/></joint>
  <joint name="spin" type="revolute"><parent link="base"/><child link="spinner"/><axis xyz="0 0 1"/></joint>
</robot>`;
  it('parses primitives, materials, limits with unit conversion and mimic joints', () => {
    const m = parseURDF(urdf);
    expect(m.name).toBe('prims');
    expect(m.rootLink).toBe('base');
    expect(m.materials.get('red')).toBe('#ff0000');
    const base = m.links.get('base')!;
    expect(base.mass).toBe(4);
    expect(base.visuals[0].primitive).toEqual({ kind: 'box', size: [200, 300, 400] });
    expect(base.visuals[0].color).toBe('#ff0000');
    expect(getPos(base.visuals[0].origin as any)[2]).toBeCloseTo(100, 9);
    expect(base.collisions[0].primitive).toEqual({ kind: 'cylinder', radius: 50, length: 200 });
    expect(m.links.get('l1')!.visuals[0].color).toBe('#00ff00');
    const l2 = m.links.get('l2')!;
    expect(l2.visuals.length).toBe(1); // unknown <capsule> skipped
    expect(l2.visuals[0].primitive).toEqual({ kind: 'sphere', radius: 50 });
    expect(l2.visuals[0].color).toBeUndefined();
    const j = Object.fromEntries(m.joints.map((x) => [x.name, x]));
    expect(j.j1.lower).toBeCloseTo(-57.2958, 3);
    expect(j.j1.velocity).toBeCloseTo(114.59, 1);
    expect(j.j1.effort).toBe(10);
    expect([j.j2.lower, j.j2.upper]).toEqual([-360, 360]);
    expect([j.j3.lower, j.j3.upper, j.j3.velocity]).toEqual([0, 100, 500]);
    expect([j.fl.velocity, j.spin.lower, j.spin.upper]).toEqual([1000, -180, 180]);
    expect(j.fr.mimic).toEqual({ joint: 'fl', multiplier: -1, offset: 10 });
    expect(() => parseURDF('<model/>')).toThrow(/Not a URDF/);
  });
  it('builds a serial chain along the longest movable path with side branches folded', () => {
    const m = parseURDF(urdf);
    const { chain, sideBranches } = chainFromURDF(m);
    // base -> j1 -> j2 -> j3 -> fl (fl is movable & not mimic so counts); tip = finger_l
    expect(chain.joints.map((x) => x.name)).toEqual(['j1', 'j2', 'j3', 'fl']);
    expect(chain.dof).toBe(4);
    expect(chain.links.map((l) => l.name)).toEqual(['base', 'l1', 'l2', 'slider', 'finger_l']);
    // the non-mimic side joint "spin" is reported, the mimic finger is folded silently
    expect(sideBranches.map((s) => s.name)).toEqual(['spin']);
    const custom = chainFromURDF(m, { tipLink: 'l2', baseLink: 'base' });
    expect(custom.chain.joints.map((x) => x.name)).toEqual(['j1', 'j2']);
    const r = robotFromURDF(urdf, {}, { tipLink: 'l2' });
    expect(r).toBeInstanceOf(Robot);
    expect(r.brand).toBe('ROS2');
    expect(r.params.source).toBe('urdf');
    expect(r.dof).toBe(2);
    r.setJoints([90, 0]);
    // l1 origin is 0.5 m up, l2 is 0.3 m along l1's X (rotated by j1 origin yaw 90° and joint 90°): ends at (-300, 0, 500)
    const p = getPos(r.solveFKFlange());
    expect(distance(p, [-300, 0, 500])).toBeLessThan(1e-3); // rpy given with 7 digits of pi/2
  });
  it('folds fixed joints on the main path and keeps mesh scale in mm', () => {
    const r = robotFromURDF(`<robot name="m"><link name="a"><visual><geometry><mesh filename="package://p/a.dae" scale="2 2 2"/></geometry></visual></link><link name="b"/><link name="c"/>
      <joint name="f" type="fixed"><parent link="a"/><child link="b"/><origin xyz="0 0 1"/></joint>
      <joint name="r" type="revolute"><parent link="b"/><child link="c"/><axis xyz="0 0 1"/><limit lower="-3.14" upper="3.14"/></joint></robot>`);
    expect(r.chain.joints[0].type).toBe('fixed');
    expect(r.chain.links[0].visuals[0].scale).toEqual([2000, 2000, 2000]);
    expect(r.params.meshes).toEqual(['package://p/a.dae']);
    expect(getPos(r.solveFKFlange())[2]).toBeCloseTo(1000, 6);
  });
});

describe('URDF export', () => {
  const tri = new Float32Array([0, 0, 0, 100, 0, 0, 0, 100, 0]);
  function cell() {
    const st = new Station('My Cell #1');
    const robot = st.addChild(createRobotFromLibrary('KUKA_KR6_R900', 'KR 6'));
    robot.setPose(mul(transl(1000, 0, 500), rotz(90 * DEG)));
    robot.chain.links[1].visuals.push({ mesh: 'link1.stl', origin: Array.from(transl(0, 0, 10)) as any, color: '#ff8800', scale: [1, 1, 1] } as any);
    robot.chain.links[2].visuals.push({ mesh: 'missing.stl', origin: Array.from(transl(0, 0, 0)) as any } as any);
    const tool = robot.addChild(new Tool('Gripper 2000'));
    tool.setPoseTool(transl(0, 0, 150));
    tool.geometry = [{ primitive: { kind: 'cylinder', radius: 30, length: 150 }, origin: Array.from(transl(0, 0, 75)), color: '#00ff00', opacity: 0.5 }];
    robot.setTool(tool);
    const table = st.addChild(new Frame('Table'));
    table.setPose(transl(500, 0, 0));
    const part = table.addChild(new SceneObject('Part'));
    part.geometry = [{ mesh: 'assets/part.STL', origin: Array.from(transl(0, 0, 20)), color: '#0000ff' }, { primitive: { kind: 'box', size: [10, 20, 30] }, color: '#abc' }, { primitive: { kind: 'sphere', radius: 5 } }, { primitive: { kind: 'plane', size: [100, 100] } }, { primitive: { kind: 'cone', radius: 5, length: 10 } }];
    const dup = table.addChild(new SceneObject('Part'));
    dup.geometry = [{ mesh: 'assets/part.STL', origin: Array.from(transl(0, 0, 20)) }];
    st.addChild(new SceneObject('empty'));
    const assets = new AssetStore();
    assets.registerRaw('link1.stl', 'stl', new Uint8Array(writeBinarySTL(tri)), 'link1');
    assets.registerRaw('PART.stl', 'stl', new Uint8Array(writeBinarySTL(tri)), 'part'); // matched by base name, case-insensitive
    return { st, robot, tool, assets };
  }
  it('exports a robot package with meshes in metres, tools and tcp frames', () => {
    const { robot, assets } = cell();
    const res = exportRobotURDF(robot, assets, { packageName: 'kr6 pkg' });
    expect(res.packageName).toBe('kr6_pkg');
    expect(Object.keys(res.files).sort()).toEqual(['CMakeLists.txt', 'README.md', 'meshes/KR_6_link_1.stl', 'package.xml', 'urdf/KR_6.urdf']);
    expect(res.warnings).toEqual(['Mesh missing.stl not loaded — visual skipped']);
    const stl = parseSTL((res.files['meshes/KR_6_link_1.stl'] as Uint8Array).buffer as ArrayBuffer);
    expect(stl.max[0]).toBeCloseTo(0.1, 6); // 100 mm -> 0.1 m (float32)
    expect(res.urdf).toContain('<robot name="KR_6">');
    expect(res.urdf).toContain('<mesh filename="package://kr6_pkg/meshes/KR_6_link_1.stl"/>');
    expect(res.urdf).toContain('<material name="c_ff8800"><color rgba="1.000 0.533 0.000 1.000"/></material>');
    expect(res.urdf).toMatch(/<joint name="Gripper_2000_tcp_joint" type="fixed">[\s\S]*<origin xyz="0 0 0.15" rpy="0 0 0"\/>/);
    expect(res.urdf).toContain('<cylinder radius="0.03000" length="0.15000"/>');
    expect(res.urdf).toContain('rgba="0.000 1.000 0.000 0.500"');
    expect(res.urdf).toContain('<link name="tool0"/>');
    expect((res.urdf.match(/<joint /g) ?? []).length).toBeGreaterThanOrEqual(6 + 2); // 6 axes (+ tool0) + gripper + tcp
    expect(res.urdf).toMatch(/<joint name="Gripper_2000_joint" type="fixed">\n\s*<parent link="(KR_6_link_6|tool0)"\/>/);
    expect(res.urdf).not.toContain('type="continuous"'); // KUKA A6 ±350° spans 700° < 720°
    expect(res.files['package.xml']).toContain('<name>kr6_pkg</name>');
    expect(res.files['CMakeLists.txt']).toContain('project(kr6_pkg)');
    expect(res.files['README.md']).toContain('KR 6');
  });
  it('re-imports its own export with matching kinematics', () => {
    const robot = createRobotFromLibrary('UR5e');
    const res = exportRobotURDF(robot);
    const back = robotFromURDF(res.urdf);
    expect(back.dof).toBe(6);
    const q = [10, -70, 50, -60, 80, 20];
    expect(distance(getPos(robot.solveFK(q)), getPos(back.solveFKFlange(q)))).toBeLessThan(1e-3);
    // continuous joints have no lower/upper
    const c = exportRobotURDF(new Robot('c', chainFromDH('c', [{ theta: 0, d: 100, a: 0, alpha: 0, lower: -360, upper: 360 }])));
    expect(c.urdf).toContain('type="continuous"');
    expect(c.urdf).toContain('<limit effort="1000" velocity=');
  });
  it('exports a whole station (objects, frames, duplicates) and zips it', () => {
    const { st, assets } = cell();
    const res = exportStationURDF(st, assets);
    expect(res.packageName).toBe('my_cell_1_description');
    expect(res.urdf).toContain('<link name="world"/>');
    expect(res.urdf).toContain('<joint name="KR_6_base_joint" type="fixed">');
    expect(res.urdf).toMatch(/<parent link="world"\/>\n\s*<child link="KR_6_base(_link)?"\/>\n\s*<origin xyz="1 0 0.5" rpy="0 0 1.570796"\/>/);
    expect(res.urdf).toContain('<link name="Table"/>');
    expect(res.urdf).toContain('<link name="Part">');
    expect(res.urdf).toContain('<link name="Part_2">');
    expect(res.urdf).not.toContain('<link name="empty">');
    expect(res.urdf).toContain('<box size="0.01000 0.02000 0.03000"/>');
    expect(res.urdf).toContain('<sphere radius="0.00500"/>');
    expect(res.urdf).toContain('<box size="0.10000 0.10000 0.001"/>');
    expect(res.urdf).toContain('<cylinder radius="0.00500" length="0.01000"/>');
    expect(res.urdf).not.toContain('c_abc'); // short colour ignored
    expect(res.urdf).toContain('<mesh filename="package://my_cell_1_description/meshes/Part.stl"/>');
    // the same mesh (same scale) is written once and shared by both parts
    expect(Object.keys(res.files).filter((f) => f.startsWith('meshes/')).sort()).toEqual(['meshes/KR_6_link_1.stl', 'meshes/Part.stl']);
    expect(res.urdf).toContain('<link name="KR_6_Gripper_2000">');
    const zip = unzipSync(packageZip(res));
    expect(Object.keys(zip)).toContain('my_cell_1_description/urdf/My_Cell_1.urdf');
    expect(strFromU8(zip['my_cell_1_description/package.xml'])).toContain('My Cell #1');
    // options: no meshes / no collisions
    const bare = exportStationURDF(st, assets, { meshes: false, collisions: false });
    expect(bare.urdf).not.toContain('<collision>');
    expect(bare.urdf).not.toContain('<mesh ');
    expect(Object.keys(bare.files).some((f) => f.startsWith('meshes/'))).toBe(false);
  });
  it('folds DH post transforms into the next joint and prefixes names; sanitizes names', () => {
    const robot = createRobotFromLibrary('UR10e');
    const mw = { files: {}, warnings: [] } as any;
    const { xml, rootLink, tipLink } = chainToURDF(robot.chain, 'ur', { write: () => null, files: {}, warnings: [] } as any, {}, 'r1_');
    expect(rootLink).toBe('r1_base');
    expect(tipLink).toMatch(/^r1_/);
    expect(xml).toContain('<joint name="r1_');
    void mw;
    expect(sanitizeName('  9 lives!! ')).toBe('_9_lives');
    expect(sanitizeName('***')).toBe('item');
  });
});
