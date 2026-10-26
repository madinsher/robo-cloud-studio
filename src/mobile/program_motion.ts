/**
 * Mobile program motion (contract I2): registers the planner the program runtime uses for Navigate (mobile_move) and
 * Follow path (mobile_follow) instructions, so the base really drives — footprint- and drive-aware planning on the
 * station's collision world, then the drive model + tracker simulated for the trajectory and its duration.
 * Imported from ./items so every station with mobile robots gets it. Items that are not MobileRobots keep the
 * straight-line default.
 */
import { setMobileMotionPlanner, straightLinePlanner, MobileMotionPlanner } from '../core/motion/mobile_hook';
import { MobileRobot } from './items';
import { planAndFollow, followPolyline } from './navigate';

export const mobileProgramPlanner: MobileMotionPlanner = (station, mobile, ins, start) => {
  if (!(mobile instanceof MobileRobot)) return straightLinePlanner(station, mobile, ins, start);
  if (ins.kind === 'mobile_move') {
    const r = planAndFollow(station, mobile, { x: ins.x, y: ins.y, heading: ins.heading, tolerance: ins.tolerance, speed: ins.speed }, { start });
    return { ok: r.ok, error: r.error, duration: r.duration, sample: r.sample };
  }
  const path = ins.pathId ? (station.findById(ins.pathId) as any) : null;
  if (!path || !Array.isArray(path.points) || !path.points.length) return { ok: false, error: 'Path not found', duration: 0, sample: () => start };
  // path points are relative to the path item: express them in the robot's parent frame
  const pa = path.poseAbs(), par = mobile.parent && mobile.parent.parent ? mobile.parent.poseAbs() : null;
  let pts: number[][] = path.points.map((p: number[]) => {
    const wx = pa[0] * p[0] + pa[4] * p[1] + pa[8] * (p[2] ?? 0) + pa[12], wy = pa[1] * p[0] + pa[5] * p[1] + pa[9] * (p[2] ?? 0) + pa[13];
    if (!par) return [wx, wy];
    const dx = wx - par[12], dy = wy - par[13];
    return [par[0] * dx + par[1] * dy, par[4] * dx + par[5] * dy];
  });
  if (path.closed && pts.length > 2) pts.push(pts[0]);
  if (ins.reverse) pts = pts.reverse();
  const r = followPolyline(station, mobile, pts, { start, speed: ins.speed ?? path.speed });
  return { ok: r.ok, error: r.error, duration: r.duration, sample: r.sample };
};

setMobileMotionPlanner(mobileProgramPlanner);
