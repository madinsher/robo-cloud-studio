/**
 * Robot definition from DH tables — the format used by RoboDK's "Robot Parameters" dialog and
 * robot builder: one joint per line "theta, d, a, alpha [, lower, upper]" (mm / deg), or a JSON
 * object { name, dh: [...], flange?, brand?, post? }. Lines starting with # are comments.
 */
import { chainFromDH, DHParams } from '../../core/kinematics/chain';
import { Robot, RobotBrand } from '../../core/items/robot';
import { xyzrpwToPose, identity } from '../../core/math/pose';

export interface DHRobotSpec {
  name?: string;
  brand?: RobotBrand;
  post?: string;
  /** Standard DH rows */
  dh: Array<number[] | DHParams>;
  /** Flange pose as [x,y,z,rx,ry,rz] (mm/deg) */
  flange?: number[];
}

export function parseDHText(text: string): DHRobotSpec {
  const t = text.trim();
  if (t.startsWith('{')) return JSON.parse(t);
  const rows: number[][] = [];
  let name: string | undefined;
  for (const raw of t.split(/\r?\n/)) {
    const line = raw.trim();
    if (!line) continue;
    if (line.startsWith('#')) { const m = line.match(/#\s*name\s*[:=]\s*(.+)/i); if (m) name = m[1].trim(); continue; }
    const vals = line.split(/[,;\t ]+/).map(Number).filter((v) => !isNaN(v));
    if (vals.length >= 4) rows.push(vals);
  }
  return { name, dh: rows };
}

export function robotFromDH(spec: DHRobotSpec, fallbackName = 'DH robot'): Robot {
  const dh: DHParams[] = spec.dh.map((r, i) => Array.isArray(r) ? ({ theta: r[0], d: r[1], a: r[2], alpha: r[3], lower: r[4] ?? -180, upper: r[5] ?? 180, type: r[6] === 1 ? 'prismatic' : 'revolute', home: r[7] ?? 0 } as DHParams) : { lower: -180, upper: 180, ...r, home: (r as DHParams).home ?? 0 });
  void dh.forEach((d, i) => void i);
  const flange = spec.flange ? xyzrpwToPose(spec.flange[0], spec.flange[1], spec.flange[2], spec.flange[3] ?? 0, spec.flange[4] ?? 0, spec.flange[5] ?? 0) : identity();
  const chain = chainFromDH(spec.name ?? fallbackName, dh, flange);
  const robot = new Robot(spec.name ?? fallbackName, chain);
  if (spec.brand) robot.brand = spec.brand;
  if (spec.post) robot.postProcessor = spec.post;
  robot.params.source = 'dh';
  return robot;
}

/** Export a robot's DH table (if it was built from DH). */
export function robotToDHText(robot: Robot): string | null {
  if (!robot.chain.dh) return null;
  return [`# name: ${robot.name}`, `# theta, d, a, alpha, lower, upper, prismatic(0/1), home  (mm/deg, standard DH)`, ...robot.chain.dh.map((d) => [d.theta, d.d, d.a, d.alpha, d.lower ?? -180, d.upper ?? 180, d.type === 'prismatic' ? 1 : 0, d.home ?? 0].join(', '))].join('\n') + '\n';
}
