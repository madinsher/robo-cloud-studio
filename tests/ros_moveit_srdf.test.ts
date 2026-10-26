/**
 * V5 finding: the MoveIt 2 export of a robot imported from a ROS package wrote an empty SRDF (it looked up
 * config/<robot>.srdf while the description names it after its URDF, config/murom.srdf) and put every joint of the
 * URDF (wheels, head, both arms) in the planning group / controller, so move_group did not start. The export now takes
 * the description's SRDF and the group of the robot's own joints; File › Export MoveIt 2 config offers it in the UI.
 */
import { describe, it, expect } from 'vitest';
import fs from 'node:fs';
import path from 'node:path';
import { importRosPackageFiles } from '../src/io/urdf/package';
import { exportMoveItConfig } from '../src/io/urdf/moveit_export';
import { AssetStore } from '../src/scene/assets';
import { MURKA, HAS_MURKA } from './webots_helpers';

const DESC = `${MURKA}/robot-retailer-murka-v2/ros2_ws/src/murom_description`;
function readDir(root: string, prefix: string): Map<string, Uint8Array> {
  const m = new Map<string, Uint8Array>();
  const walk = (d: string, rel: string) => { for (const e of fs.readdirSync(d, { withFileTypes: true })) { const p = path.join(d, e.name), r = `${rel}/${e.name}`; if (e.isDirectory()) walk(p, r); else m.set(r, new Uint8Array(fs.readFileSync(p))); } };
  walk(root, prefix);
  return m;
}

describe('MoveIt 2 export of a robot imported from a ROS package', () => {
  it.skipIf(!HAS_MURKA || !fs.existsSync(DESC))('MUROM left arm: non-empty SRDF, the left-arm group and controller with its 4 joints', async () => {
    const assets = new AssetStore();
    const res = await importRosPackageFiles(readDir(DESC, 'murom_description'), assets);
    const robot = res.robots.find((r) => r.name === 'murom_left_arm')!;
    expect(robot).toBeTruthy();
    const mv = exportMoveItConfig(robot, assets);
    const srdf = Object.entries(mv.files).find(([p]) => p.endsWith('.srdf') && p.includes('moveit_config'));
    expect(srdf, Object.keys(mv.files).join(', ')).toBeTruthy();
    expect(String(srdf![1]).length).toBeGreaterThan(500);
    expect(mv.group).toBe('murom_left_arm');
    expect(mv.joints).toEqual(['l_shoulder_pitch_joint', 'l_shoulder_roll_joint', 'l_shoulder_yaw_joint', 'l_elbow_joint']);
    const ctl = String(Object.entries(mv.files).find(([p]) => p.endsWith('moveit_controllers.yaml'))![1]);
    expect(ctl).toContain('murom_left_arm_controller');
    expect(ctl).not.toMatch(/wheel|head_pan/);
  }, 120000);
});
