/**
 * NavigateToTarget of murom_navigation (navigate_server.py + parking.py), on the studio's own drive model:
 *  1. resolve the goal: named target (home), dock (standoff -> glide-dock to the dock approach), slot (approach /
 *     approach_alt chosen for the arm, standoff as the coarse goal) or an explicit pose;
 *  0. leave a tight parking first: reverse glide to the standoff of the parking the robot stands in (corridor test of
 *     parking.py — the mission starts on the charger), or only re-glide when the goal is the same parking;
 *  2. coarse navigation (the Nav2 stand-in): footprint-aware path on the occupancy map (planForRobot, I8 planner) tracked
 *     by the studio path follower at the Nav2 velocity limits, stopping within xy_goal_tolerance 0.30 m;
 *  3. fine alignment: the glide-docking controller of navigate_server (P-control on the ground-truth pose, 0.12 m/s,
 *     0.4 rad/s, entry angle ≤ 20°, retreat-and-retry, 2 cm / 0.03 rad), unless skip_fine_align.
 */
import { planForRobot } from '../../mobile/planner';
import { stationWorld } from '../../mobile/world';
import { toPosePath, isPosePath } from '../../mobile/controller';
import type { Pose3, RetailLayout } from './layout';
import type { MuromSim } from './sim';
import { Task } from './tasks';

const norm = (a: number) => Math.atan2(Math.sin(a), Math.cos(a));
export type P3 = [number, number, number];
const p3 = (p: Pose3): P3 => [p.x, p.y, p.yaw];

/** navigate_server parameters as configured in murom_navigation/config/nav2_params.yaml (not the code defaults). */
export const NAV_PARAMS = {
  navTimeout: 240, fineXy: 0.02, fineYaw: 0.03, fineTimeout: 60, fineV: 0.12, fineW: 0.4,
  /** Nav2 RPP desired_linear_vel and xy_goal_tolerance */
  nav2V: 0.4, nav2Tol: 0.30,
};
// parking.py
export const CORRIDOR_BACK = 0.7, CORRIDOR_AHEAD = 0.15, CORRIDOR_HALF_WIDTH = 0.15, CORRIDOR_YAW = 0.4;
export const UNDOCK_STRAIGHT = 0.25, UNDOCK_DONE_LONG = 0.10, UNDOCK_LAT_TOL = 0.06, UNDOCK_OVERRUN = 0.40;

export function parkingErrors(pose: P3, fine: P3): P3 {
  const ux = Math.cos(fine[2]), uy = Math.sin(fine[2]); const dx = pose[0] - fine[0], dy = pose[1] - fine[1];
  return [dx * ux + dy * uy, -dx * uy + dy * ux, norm(pose[2] - fine[2])];
}
export function inParking(pose: P3, fine: P3): boolean {
  const [el, ea, ey] = parkingErrors(pose, fine);
  return -CORRIDOR_BACK < el && el < CORRIDOR_AHEAD && Math.abs(ea) < CORRIDOR_HALF_WIDTH && Math.abs(ey) < CORRIDOR_YAW;
}
export type Parking = [P3, P3];
export function parkings(L: RetailLayout): Parking[] {
  const out: Parking[] = [];
  if (L.dock) out.push([p3(L.dock.approach), p3(L.dock.standoff)]);
  for (const s of Object.values(L.slots)) {
    if (s.standoff && s.approach) out.push([p3(s.approach), p3(s.standoff)]);
    if (s.standoffAlt && s.approachAlt) out.push([p3(s.approachAlt), p3(s.standoffAlt)]);
  }
  return out;
}
export function parkingNear(cands: Parking[], pose: P3): Parking | null {
  let best: [number, Parking] | null = null;
  for (const c of cands) { if (!inParking(pose, c[0])) continue; const d = Math.hypot(pose[0] - c[0][0], pose[1] - c[0][1]); if (!best || d < best[0]) best = [d, c]; }
  return best ? best[1] : null;
}
export function currentParking(pose: P3, last: Parking | null, cands: Parking[]): Parking | null {
  if (last && inParking(pose, last[0])) return last;
  return parkingNear(cands, pose);
}
export function undockStep(eLong: number, eLat: number): [boolean, number] {
  if (eLong <= UNDOCK_DONE_LONG && (Math.abs(eLat) <= UNDOCK_LAT_TOL || eLong <= -UNDOCK_OVERRUN)) return [true, 0];
  if (eLong > UNDOCK_STRAIGHT) return [false, 0];
  if (eLong > UNDOCK_DONE_LONG) return [false, Math.max(-0.35, Math.min(0.35, 1.5 * eLat))];
  return [false, Math.max(-0.45, Math.min(0.45, 2.5 * eLat))];
}

/** nav2 RegulatedPurePursuitController with the murom_navigation parameters (nav2_params.yaml FollowPath). */
export const RPP = { desiredV: 0.4, lookaheadTime: 1.5, minLookahead: 0.45, maxLookahead: 0.9, approachDist: 0.6, minApproachV: 0.08, minRadius: 0.9, minRegulatedV: 0.25, rotateMinAngle: 1.0, rotateW: 0.7 };
/** One RPP control step on a densified path (m): carrot at the velocity-scaled lookahead, rotate-to-heading above
 *  1 rad, curvature- and approach-regulated linear speed. Returns the command and the pruned path index. */
export function rppCommand(pts: Array<[number, number]>, start: number, pose: P3, vNow: number): { v: number; w: number; index: number } {
  const [rx, ry, th] = pose;
  let best = start, bd = Infinity;
  for (let i = start; i < pts.length; i++) { const d = Math.hypot(pts[i][0] - rx, pts[i][1] - ry); if (d < bd) { bd = d; best = i; } if (d > bd + 1.0) break; }
  const L = Math.max(RPP.minLookahead, Math.min(RPP.maxLookahead, vNow * RPP.lookaheadTime));
  let carrot = pts[pts.length - 1];
  for (let i = best; i < pts.length; i++) if (Math.hypot(pts[i][0] - rx, pts[i][1] - ry) >= L) { carrot = pts[i]; break; }
  let remaining = 0; for (let i = best; i + 1 < pts.length; i++) remaining += Math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1]);
  remaining += bd;
  const dx = carrot[0] - rx, dy = carrot[1] - ry;
  const cx = Math.cos(th) * dx + Math.sin(th) * dy, cy = -Math.sin(th) * dx + Math.cos(th) * dy;
  const ang = Math.atan2(cy, cx);
  if (Math.abs(ang) > RPP.rotateMinAngle) return { v: 0, w: Math.sign(ang) * RPP.rotateW, index: best };
  const d2 = cx * cx + cy * cy; const curv = d2 > 1e-9 ? (2 * cy) / d2 : 0;
  let v = RPP.desiredV;
  const radius = Math.abs(curv) > 1e-9 ? 1 / Math.abs(curv) : Infinity;
  if (radius < RPP.minRadius) v *= 1 - Math.abs(radius - RPP.minRadius) / RPP.minRadius;
  v = Math.max(v, RPP.minRegulatedV);
  if (remaining < RPP.approachDist) v = Math.min(v, Math.max(v * (remaining / RPP.approachDist), RPP.minApproachV));
  return { v, w: v * curv, index: best };
}

export interface NavGoal { targetId?: string; arm?: string; pose?: P3; skipFineAlign?: boolean }
export interface NavOutcome { success: boolean; message: string; time: number; coarse?: P3; fine?: P3 | null; finalError?: [number, number, number] }

/** Navigation server state that persists between goals (last tight parking). */
export class Navigator {
  lastParked: Parking | null = null;
  readonly cands: Parking[];
  /** Planned coarse paths (mm) of every goal, for reports / the 3D view. */
  readonly paths: number[][][] = [];
  /** Phases of every goal (undock / coarse / glide) with their sim times and results. */
  readonly events: Array<{ t0: number; t1: number; kind: 'undock' | 'coarse' | 'glide'; ok: boolean; message: string }> = [];
  private *timed(kind: 'undock' | 'coarse' | 'glide', task: Task<[boolean, string]>): Task<[boolean, string]> { const t0 = this.sim.time; const r = yield* task; this.events.push({ t0, t1: this.sim.time, kind, ok: r[0], message: r[1] }); return r; }
  constructor(readonly sim: MuromSim, readonly L: RetailLayout, readonly cancelled: () => boolean = () => false) { this.cands = parkings(L); }

  resolve(goal: NavGoal): [P3, P3 | null] | null {
    if (goal.pose) return [goal.pose, null];
    const tid = goal.targetId ?? '';
    if (tid === 'dock' && this.L.dock) return [p3(this.L.dock.standoff), p3(this.L.dock.approach)];
    const t = this.L.targets[tid]; if (t) return [p3(t), null];
    const s = this.L.slots[tid];
    if (s && s.approach) {
      let approach = s.approach, standoff = s.standoff;
      if (goal.arm && s.arm !== goal.arm && s.armAlt === goal.arm && s.approachAlt) { approach = s.approachAlt; standoff = s.standoffAlt; }
      const fine = p3(approach);
      return [standoff ? p3(standoff) : fine, fine];
    }
    return null;
  }

  *execute(goal: NavGoal): Task<NavOutcome> {
    const sim = this.sim; const t0 = sim.time;
    const resolved = this.resolve(goal);
    if (!resolved) return { success: false, message: `Неизвестная цель: ${goal.targetId}`, time: 0 };
    const [coarse, fine] = resolved;
    const out = (success: boolean, message: string): NavOutcome => { sim.stop(); const p = sim.pose(); const f = fine ?? coarse; const e = parkingErrors(p, f); return { success, message, time: sim.time - t0, coarse, fine, finalError: e }; };
    // phase 0: leave the tight parking
    const parked = currentParking(sim.pose(), this.lastParked, this.cands);
    if (parked) {
      const [lastFine, lastStandoff] = parked;
      if (fine && Math.hypot(fine[0] - lastFine[0], fine[1] - lastFine[1]) < 0.05) {
        this.lastParked = [fine, coarse];
        const [ok, msg] = yield* this.timed('glide', this.glideTo(fine));
        return out(ok, ok ? 'OK (уже на месте)' : `fine_align: ${msg}`);
      }
      const [ok, msg] = yield* this.timed('undock', this.undockTo(lastStandoff));
      if (!ok) { this.lastParked = parked; return out(false, `undock: ${msg}`); }
    }
    this.lastParked = null;
    // phase 1: coarse navigation (Nav2 stand-in), up to 3 attempts
    let [ok, msg] = yield* this.timed('coarse', this.coarse(coarse));
    for (let a = 1; !ok && a < 3 && !this.cancelled(); a++) { yield 3.0; [ok, msg] = yield* this.timed('coarse', this.coarse(coarse)); }
    if (!ok) return out(false, msg);
    // phase 2: fine alignment
    if (!goal.skipFineAlign) {
      const target = fine ?? coarse;
      const [gok, gmsg] = yield* this.timed('glide', this.glideTo(target));
      sim.stop();
      if (fine) this.lastParked = [fine, coarse];
      if (!gok) return out(false, `fine_align: ${gmsg}`);
    }
    return out(true, 'OK');
  }

  /** Nav2 stand-in: plan on the occupancy map (footprint-aware), track at 0.4 m/s, stop within 0.30 m. */
  *coarse(goal: P3): Task<[boolean, string]> {
    const sim = this.sim; const robot = sim.base;
    const [x, y] = sim.pose();
    if (Math.hypot(goal[0] - x, goal[1] - y) <= NAV_PARAMS.nav2Tol) return [true, 'OK'];
    const world = stationWorld(sim.station);
    let path: number[][] | null = null; let err = '';
    for (const margin of [120, 60, 20]) {
      const r = planForRobot(robot, { x: robot.state.x, y: robot.state.y, theta: robot.state.theta }, { x: goal[0] * 1000, y: goal[1] * 1000 }, { world, margin });
      if (r.ok) { path = r.path; break; }
      err = r.error ?? 'no path';
    }
    if (!path) return [false, `Nav2: путь не найден (${err})`];
    if (!isPosePath(path)) path = toPosePath(robot, path, {});
    this.paths.push(path);
    // densified polyline (m) for the RPP controller
    const pts: Array<[number, number]> = [];
    for (const p of path) { const q: [number, number] = [p[0] / 1000, p[1] / 1000]; const last = pts[pts.length - 1]; if (last) { const d = Math.hypot(q[0] - last[0], q[1] - last[1]); const n = Math.ceil(d / 0.05); for (let k = 1; k < n; k++) pts.push([last[0] + ((q[0] - last[0]) * k) / n, last[1] + ((q[1] - last[1]) * k) / n]); if (d < 1e-6) continue; } pts.push(q); }
    const t1 = sim.time + NAV_PARAMS.navTimeout;
    let idx = 0;
    for (;;) {
      if (this.cancelled()) { sim.stop(); return [false, 'Отменено']; }
      if (sim.time > t1) { sim.stop(); return [false, 'Nav2 timeout']; }
      const [rx, ry, th] = sim.pose();
      if (Math.hypot(goal[0] - rx, goal[1] - ry) <= NAV_PARAMS.nav2Tol) break; // SimpleGoalChecker (stateful, yaw free)
      const cmd = rppCommand(pts, idx, [rx, ry, th], Math.abs(sim.base.state.v) / 1000);
      idx = cmd.index;
      sim.cmdVel(cmd.v, cmd.w);
      yield;
    }
    sim.stop();
    return [true, 'OK'];
  }

  /** navigate_server._undock_to: reverse glide out of a tight parking onto the standoff line. */
  *undockTo(standoff: P3): Task<[boolean, string]> {
    const sim = this.sim; const P = NAV_PARAMS;
    const [xt, yt, yawt] = standoff; const ux = Math.cos(yawt), uy = Math.sin(yawt);
    const t0 = sim.time; let last: [number | null, number] = [null, t0]; let wiggles = 0;
    for (;;) {
      if (this.cancelled()) { sim.stop(); return [false, 'Отменено']; }
      if (sim.time - t0 > 40) { sim.stop(); return [false, 'таймаут']; }
      const [x, y, th] = sim.pose();
      const dx = x - xt, dy = y - yt; const eLong = dx * ux + dy * uy, eLat = -dx * uy + dy * ux;
      const [done, latCorr] = undockStep(eLong, eLat);
      if (done) { sim.stop(); return [true, 'OK']; }
      const [ref, tRef] = last;
      if (ref === null || ref - eLong > 0.04) last = [eLong, sim.time];
      else if (sim.time - tRef > 5) {
        if (wiggles >= 3) { sim.stop(); return [false, 'заклинило (3 расклинивания не помогли)']; }
        wiggles++;
        const tEnd = sim.time + 1.5;
        while (sim.time < tEnd && !this.cancelled()) { sim.cmdVel(P.fineV, 0); yield; }
        sim.stop(); last = [null, sim.time];
        continue;
      }
      const glideHeading = yawt + latCorr;
      sim.cmdVel(-Math.min(P.fineV, 0.4 * Math.max(eLong, 0) + 0.05), Math.max(-P.fineW, Math.min(P.fineW, 1.8 * norm(glideHeading - th))));
      yield;
    }
  }

  /** navigate_server._glide_to: precise parking on the target line (2 cm / 0.03 rad). */
  *glideTo(target: P3): Task<[boolean, string]> {
    const sim = this.sim; const P = NAV_PARAMS;
    const [xt, yt, yawt] = target; const ux = Math.cos(yawt), uy = Math.sin(yawt);
    const t0 = sim.time; let retreats = 0; let mode: 'glide' | 'retreat' = 'glide';
    for (;;) {
      if (this.cancelled()) { sim.stop(); return [false, 'Отменено']; }
      if (sim.time - t0 > P.fineTimeout) { sim.stop(); return [false, `Таймаут fine_align (${P.fineTimeout} с)`]; }
      const [x, y, th] = sim.pose();
      const dx = x - xt, dy = y - yt; const eLong = dx * ux + dy * uy, eLat = -dx * uy + dy * ux, eYaw = norm(th - yawt);
      if (mode === 'glide' && Math.abs(eYaw) > 0.5 && Math.abs(eLong) > P.fineXy) { sim.cmdVel(0, Math.max(-P.fineW, Math.min(P.fineW, -1.5 * eYaw))); yield; continue; }
      if (mode === 'retreat') {
        if (eLong <= -0.45) { mode = 'glide'; retreats++; if (retreats > 5) { sim.stop(); return [false, 'fine_align: не сошёлся за 5 заходов']; } }
        else { const gh = yawt + Math.max(-0.35, Math.min(0.35, 1.5 * eLat)); sim.cmdVel(-P.fineV, Math.max(-P.fineW, Math.min(P.fineW, 1.8 * norm(gh - th)))); }
        yield; continue;
      }
      if (eLong < -P.fineXy) {
        const gh = yawt - Math.max(-0.35, Math.min(0.35, 1.5 * eLat));
        sim.cmdVel(Math.min(P.fineV, 0.5 * -eLong + 0.04), Math.max(-P.fineW, Math.min(P.fineW, 1.8 * norm(gh - th))));
      } else if (Math.abs(eLat) > P.fineXy * 1.5 || eLong > 0.06) { mode = 'retreat'; continue; }
      else if (Math.abs(eYaw) > P.fineYaw) sim.cmdVel(0, Math.max(-P.fineW, Math.min(P.fineW, -1.5 * eYaw)));
      else { sim.stop(); return [true, 'OK']; }
      yield;
    }
  }
}
