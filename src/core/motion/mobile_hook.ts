/**
 * Mobile-base motion hook (contract I2). The program runtime asks the registered planner how a mobile robot executes
 * a Navigate (mobile_move) or Follow path (mobile_follow) instruction. Without a registered planner the base drives in
 * a straight line (or along the path polyline) at the instruction speed.
 * Poses are in the mobile robot's parent frame: x, y in mm, heading in degrees.
 */
import type { MobileMoveInstruction, MobileFollowInstruction } from '../items/program';
import type { Station, Item } from '../items/item';

export interface Pose2D { x: number; y: number; heading: number }
export interface MobileMotionPlan { ok: boolean; error?: string; duration: number; sample: (t: number) => Pose2D }
export type MobileMotionPlanner = (station: Station, mobile: Item, ins: MobileMoveInstruction | MobileFollowInstruction, start: Pose2D) => MobileMotionPlan;

let registered: MobileMotionPlanner | null = null;

/** Register the planner used for mobile_move / mobile_follow (null restores the straight-line default). */
export function setMobileMotionPlanner(fn: MobileMotionPlanner | null): void {
  registered = fn;
}
export function getMobileMotionPlanner(): MobileMotionPlanner {
  return registered ?? straightLinePlanner;
}

const angDiff = (a: number, b: number) => ((((b - a) % 360) + 540) % 360) - 180;

/** Polyline motion at constant speed; heading follows the travel direction, then turns to the final heading. */
function polylinePlan(start: Pose2D, pts: number[][], speed: number, yawRate: number, finalHeading?: number): MobileMotionPlan {
  const legs: Array<{ x0: number; y0: number; x1: number; y1: number; h: number; t0: number; tTurn: number; tDrive: number; hFrom: number }> = [];
  let x = start.x, y = start.y, h = start.heading, t = 0;
  for (const p of pts) {
    const dx = p[0] - x, dy = p[1] - y, len = Math.hypot(dx, dy);
    if (len < 1e-6) continue;
    const hd = (Math.atan2(dy, dx) * 180) / Math.PI;
    const tTurn = Math.abs(angDiff(h, hd)) / yawRate, tDrive = len / speed;
    legs.push({ x0: x, y0: y, x1: p[0], y1: p[1], h: hd, t0: t, tTurn, tDrive, hFrom: h });
    t += tTurn + tDrive; x = p[0]; y = p[1]; h = hd;
  }
  const hEnd = finalHeading ?? h, tFinal = Math.abs(angDiff(h, hEnd)) / yawRate;
  const tLast = t;
  const duration = Math.max(1e-3, t + tFinal);
  return {
    ok: true,
    duration,
    sample: (tt: number) => {
      if (tt >= tLast) { const f = tFinal > 0 ? Math.min(1, (tt - tLast) / tFinal) : 1; return { x, y, heading: h + angDiff(h, hEnd) * f }; }
      for (const l of legs) {
        if (tt > l.t0 + l.tTurn + l.tDrive) continue;
        const u = tt - l.t0;
        if (u < l.tTurn) return { x: l.x0, y: l.y0, heading: l.hFrom + angDiff(l.hFrom, l.h) * (u / l.tTurn) };
        const f = l.tDrive > 0 ? (u - l.tTurn) / l.tDrive : 1;
        return { x: l.x0 + (l.x1 - l.x0) * f, y: l.y0 + (l.y1 - l.y0) * f, heading: l.h };
      }
      return { x: start.x, y: start.y, heading: start.heading };
    },
  };
}

/** Default: straight line (Navigate) or the path polyline (Follow) at the instruction speed, never 0 s. */
export const straightLinePlanner: MobileMotionPlanner = (station, mobile, ins, start) => {
  const kin = (mobile as any).kin ?? {};
  const yawRate = Math.max(1, kin.maxYawRate ?? 90);
  if (ins.kind === 'mobile_move') {
    const speed = Math.max(1, ins.speed ?? kin.maxSpeed ?? 1000);
    return polylinePlan(start, [[ins.x, ins.y]], speed, yawRate, ins.heading);
  }
  const path = ins.pathId ? (station.findById(ins.pathId) as any) : null;
  if (!path || !Array.isArray(path.points) || !path.points.length) return { ok: false, error: 'Path not found', duration: 0, sample: () => start };
  const speed = Math.max(1, ins.speed ?? path.speed ?? kin.maxSpeed ?? 1000);
  // path points are relative to the path item; express them in the mobile robot's parent frame
  const pa = path.poseAbs(), par = mobile.parent ? mobile.parent.poseAbs() : null;
  const toParent = (p: number[]): number[] => {
    const wx = pa[0] * p[0] + pa[4] * p[1] + pa[8] * (p[2] ?? 0) + pa[12], wy = pa[1] * p[0] + pa[5] * p[1] + pa[9] * (p[2] ?? 0) + pa[13], wz = pa[2] * p[0] + pa[6] * p[1] + pa[10] * (p[2] ?? 0) + pa[14];
    if (!par || mobile.parent!.type === 1) return [wx, wy];
    const dx = wx - par[12], dy = wy - par[13], dz = wz - par[14];
    return [par[0] * dx + par[1] * dy + par[2] * dz, par[4] * dx + par[5] * dy + par[6] * dz];
  };
  let pts: number[][] = path.points.map(toParent);
  if (path.closed && pts.length > 2) pts.push(pts[0]);
  if (ins.reverse) pts = pts.reverse();
  return polylinePlan(start, pts, speed, yawRate);
};
