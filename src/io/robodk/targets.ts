/**
 * RoboDK-compatible target import/export.
 * Formats:
 *  - CSV/TXT with 6 columns: X,Y,Z,Rx,Ry,Rz (mm/deg, XYZRPW = RoboDK default Pose_2_TxyzRxyz)
 *  - 7 columns: name + 6, or 12 columns (pose + joints), or 3 columns (points -> Z-down approach).
 *  - Lines starting with # or % are comments; separators: comma, semicolon, tab, space.
 */
import { Frame, Target, Item } from '../../core/items/item';
import { xyzrpwToPose, poseToXyzrpw, poseToKuka, poseToFanuc, kukaToPose, Mat4, transl, rotx, mul, DEG } from '../../core/math/pose';

export type EulerFormat = 'xyzrpw' | 'kuka' | 'fanuc' | 'xyz';

export interface TargetImportOptions {
  format?: EulerFormat;
  /** Frame to attach the targets to. */
  parent: Item;
  /** Prefix for generated names. */
  namePrefix?: string;
  /** Joints for 3-column points (approach pointing -Z). */
  pointApproachPose?: Mat4;
}

export function parseTargetsText(text: string): Array<{ name?: string; values: number[] }> {
  const rows: Array<{ name?: string; values: number[] }> = [];
  for (const raw of text.split(/\r?\n/)) {
    const line = raw.trim();
    if (!line || line.startsWith('#') || line.startsWith('%') || line.startsWith('//')) continue;
    const parts = line.split(/[,;\t ]+/).filter(Boolean);
    let name: string | undefined;
    if (isNaN(Number(parts[0]))) name = parts.shift();
    const values = parts.map(Number).filter((v) => !isNaN(v));
    if (values.length >= 3) rows.push({ name, values });
  }
  return rows;
}

export function importTargets(text: string, opts: TargetImportOptions): Target[] {
  const rows = parseTargetsText(text);
  const targets: Target[] = [];
  const fmt = opts.format ?? 'xyzrpw';
  rows.forEach((r, i) => {
    const t = new Target(r.name ?? `${opts.namePrefix ?? 'Target'} ${i + 1}`);
    const v = r.values;
    let pose: Mat4;
    if (v.length >= 6) {
      pose = fmt === 'kuka' ? kukaToPose(v[0], v[1], v[2], v[3], v[4], v[5]) : fmt === 'fanuc' ? kukaToPose(v[0], v[1], v[2], v[5], v[4], v[3]) : xyzrpwToPose(v[0], v[1], v[2], v[3], v[4], v[5]);
      if (v.length >= 12) t.joints = v.slice(6, 12);
    } else {
      pose = opts.pointApproachPose ? mul(transl(v[0], v[1], v[2]), opts.pointApproachPose) : mul(transl(v[0], v[1], v[2]), rotx(180 * DEG));
    }
    t.setPose(pose);
    opts.parent.addChild(t);
    targets.push(t);
  });
  return targets;
}

export function exportTargets(targets: Target[], fmt: EulerFormat = 'xyzrpw', withJoints = true): string {
  const lines = [`# name,x,y,z,rx,ry,rz${withJoints ? ',j1,j2,j3,j4,j5,j6' : ''} (${fmt}, mm/deg) — VerticalBot Studio`];
  for (const t of targets) {
    const m = t.pose();
    const v = fmt === 'kuka' ? poseToKuka(m) : fmt === 'fanuc' ? poseToFanuc(m) : poseToXyzrpw(m);
    const row = [t.name.replace(/[, ]/g, '_'), ...v.map((x) => x.toFixed(3))];
    if (withJoints && t.joints) row.push(...t.joints.map((x) => x.toFixed(3)));
    lines.push(row.join(','));
  }
  return lines.join('\n') + '\n';
}
export { Frame };
