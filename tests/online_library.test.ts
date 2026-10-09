import { describe, it, expect } from 'vitest';
import { ONLINE_ROBOT_LIBRARY, ONLINE_REPOS, resolvePackageUri, repoForPackage, fetchOnlineRobot, fetchRobotFromUrl } from '../src/io/library/online_library';
import { AssetStore } from '../src/scene/assets';
import { flangePose } from '../src/core/kinematics/chain';
import { getPos } from '../src/core/math/pose';

describe('online library catalogue', () => {
  it('has unique ids and valid repositories', () => {
    const ids = new Set<string>();
    for (const e of ONLINE_ROBOT_LIBRARY) {
      expect(ids.has(e.id), e.id).toBe(false);
      ids.add(e.id);
      expect(ONLINE_REPOS.some((r) => r.id === e.repo), `${e.id} repo ${e.repo}`).toBe(true);
      expect(e.dof).toBeGreaterThanOrEqual(4);
    }
    expect(ONLINE_ROBOT_LIBRARY.length).toBeGreaterThan(80);
  });
  it('resolves package:// URIs to raw GitHub URLs', () => {
    expect(resolvePackageUri('package://fanuc_lrmate200id_support/meshes/lrmate200id/visual/base_link.stl')).toBe('https://raw.githubusercontent.com/ros-industrial/fanuc/melodic-devel/fanuc_lrmate200id_support/meshes/lrmate200id/visual/base_link.stl');
    expect(resolvePackageUri('$(find abb_resources)/urdf/common_materials.xacro')).toContain('/ros-industrial/abb/kinetic-devel/abb_resources/urdf/common_materials.xacro');
    expect(resolvePackageUri('package://staubli_resources/urdf/common_materials.xacro')).toContain('/ros-industrial/staubli/melodic-devel/');
    expect(resolvePackageUri('package://staubli_tx2_60_support/x.stl')).toContain('/staubli_experimental/');
    expect(resolvePackageUri('package://unknown_pkg/x.stl')).toBeNull();
    expect(repoForPackage('ur_e_description')?.id).toBe('ur');
  });
  it('builds a robot from an in-memory xacro through the fetch pipeline (no network)', async () => {
    const files: Record<string, string> = {
      'https://example.test/pkg/urdf/robot.xacro': `<?xml version="1.0"?><robot name="mini" xmlns:xacro="http://wiki.ros.org/xacro">
        <xacro:include filename="$(find fanuc_resources)/urdf/common_materials.xacro"/>
        <xacro:include filename="robot_macro.xacro"/>
        <xacro:mini prefix=""/></robot>`,
      'https://example.test/pkg/urdf/robot_macro.xacro': `<robot xmlns:xacro="http://wiki.ros.org/xacro"><xacro:macro name="mini" params="prefix">
        <link name="\${prefix}base_link"><visual><geometry><mesh filename="package://fanuc_lrmate200id_support/meshes/base.stl"/></geometry></visual></link>
        <link name="\${prefix}link_1"/><link name="\${prefix}link_2"/><link name="\${prefix}flange"/><link name="\${prefix}tool0"/>
        <joint name="j1" type="revolute"><parent link="\${prefix}base_link"/><child link="\${prefix}link_1"/><origin xyz="0 0 0.3"/><axis xyz="0 0 1"/><limit lower="-3" upper="3" effort="1" velocity="1"/></joint>
        <joint name="j2" type="revolute"><parent link="\${prefix}link_1"/><child link="\${prefix}link_2"/><origin xyz="0.1 0 0"/><axis xyz="0 1 0"/><limit lower="-3" upper="3" effort="1" velocity="1"/></joint>
        <joint name="j2-flange" type="fixed"><parent link="\${prefix}link_2"/><child link="\${prefix}flange"/><origin xyz="0.4 0 0"/></joint>
        <joint name="flange-tool0" type="fixed"><parent link="\${prefix}flange"/><child link="\${prefix}tool0"/><origin xyz="0 0 0" rpy="0 1.5707963 0"/></joint>
        </xacro:macro></robot>`,
      'https://raw.githubusercontent.com/ros-industrial/fanuc/melodic-devel/fanuc_resources/urdf/common_materials.xacro': `<robot xmlns:xacro="http://wiki.ros.org/xacro"><xacro:macro name="material_fanuc_yellow"><material name="y"><color rgba="1 1 0 1"/></material></xacro:macro></robot>`,
    };
    const stl = new Uint8Array(84 + 50);
    new DataView(stl.buffer).setUint32(80, 1, true);
    const dv = new DataView(stl.buffer, 84);
    const tri = [0, 0, 1, 0, 0, 0, 0.1, 0, 0, 0, 0.1, 0];
    tri.forEach((v, i) => dv.setFloat32(i * 4, v, true));
    const assets = new AssetStore();
    const res = await fetchRobotFromUrl('https://example.test/pkg/urdf/robot.xacro', {
      assets,
      fetchText: async (u) => files[u] ?? null,
      fetchBytes: async (u) => (u.endsWith('base.stl') ? stl : null),
    });
    expect(res.robot.dof).toBe(2);
    expect(res.files.length).toBe(3);
    expect(res.meshes.loaded).toEqual(['package://fanuc_lrmate200id_support/meshes/base.stl']);
    expect(assets.get('package://fanuc_lrmate200id_support/meshes/base.stl')?.mesh?.triangles).toBe(1);
    // flange = tool0 frame (Z along +X after the fixed rotation), at x = 100 + 400 mm, z = 300 mm
    const p = getPos(flangePose(res.robot.chain, [0, 0]));
    expect(p[0]).toBeCloseTo(500, 3);
    expect(p[2]).toBeCloseTo(300, 3);
  });
});

// Real downloads from GitHub; run with STUDIO_NET_TESTS=1 (skipped in CI without network).
const net = !!process.env.STUDIO_NET_TESTS;
describe.skipIf(!net)('online library (network)', () => {
  const cases: [string, [number, number, number]][] = [
    ['fanuc_lrmate200id', [465, 0, 695]],
    ['abb_irb2400', [940, 0, 1455]],
    ['kuka_kr6r900sixx', [980, 0, 435]],
    ['motoman_gp8', [460, 0, 715]],
    ['ur5e', [817, 234, 63]],
    ['franka_panda', [107, 0, 925]],
    ['doosan_m1013', [0, 35, 1452]],
  ];
  for (const [id, at0] of cases) {
    it(`downloads ${id} with meshes`, async () => {
      const assets = new AssetStore();
      const r = await fetchOnlineRobot(id, { assets });
      const e = ONLINE_ROBOT_LIBRARY.find((x) => x.id === id)!;
      expect(r.robot.dof).toBe(e.dof);
      expect(r.meshes.failed).toEqual([]);
      expect(r.meshes.loaded.length).toBeGreaterThan(5);
      const p = getPos(flangePose(r.robot.chain, r.robot.jointsHome()));
      for (let i = 0; i < 3; i++) expect(Math.abs(p[i] - at0[i])).toBeLessThan(3);
    }, 120000);
  }
});
