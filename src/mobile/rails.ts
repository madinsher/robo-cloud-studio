/**
 * Rail networks (greenhouse pipe-rail trolleys, navigation method `rail_1d`): straight rails (heating-pipe pairs in
 * the paths between the gutters) attached to a concrete corridor. A trolley moves along a rail only (1-D), changes
 * rails on the corridor, where it keeps right of the corridor axis (two lanes, so trolleys pass each other), and turns
 * in place only on the corridor. `railRoute` gives the polyline; the planner turns it into a pose path for the drive.
 */

export interface RailNetwork {
  /** Corridor axis: two points (mm); the corridor is `corridorWidth` wide. */
  corridor: [number[], number[]];
  corridorWidth: number;
  /** Rails: [corridor end, far end] (mm); the corridor end lies on the corridor axis. */
  rails: Array<[number[], number[]]>;
}

const sub = (a: number[], b: number[]) => [a[0] - b[0], a[1] - b[1]];
const dot = (a: number[], b: number[]) => a[0] * b[0] + a[1] * b[1];
const len = (a: number[]) => Math.hypot(a[0], a[1]);

/** Projection of p on segment [a, b]: parameter (0..1), point and distance. */
export function projectOnSegment(p: number[], a: number[], b: number[]): { t: number; q: number[]; d: number } {
  const ab = sub(b, a), l2 = dot(ab, ab);
  const t = l2 > 0 ? Math.max(0, Math.min(1, dot(sub(p, a), ab) / l2)) : 0;
  const q = [a[0] + ab[0] * t, a[1] + ab[1] * t];
  return { t, q, d: len(sub(p, q)) };
}

/** Where a point lies on the network: the nearest rail (index) or the corridor (-1), the projection and distance. */
export function locateOnRails(net: RailNetwork, p: number[]): { rail: number; q: number[]; d: number } {
  const c = projectOnSegment(p, net.corridor[0], net.corridor[1]);
  let best = { rail: -1, q: c.q, d: Math.max(0, c.d - net.corridorWidth / 2) };
  net.rails.forEach((r, i) => { const pr = projectOnSegment(p, r[0], r[1]); if (pr.d < best.d - 1e-6 || (pr.d < 1 && pr.t > 1e-6 && best.rail < 0)) best = { rail: i, q: pr.q, d: pr.d }; });
  return best;
}

/** Distance (mm) of a point to the network: rail axes and the corridor band. */
export function distanceToRails(net: RailNetwork, p: number[]): number {
  return locateOnRails(net, p).d;
}

/**
 * Route between two points on the network (polyline, mm): along the start rail to the corridor, along the corridor in
 * the lane on the right of the travel direction, onto the goal rail. Points off the network are joined by a straight
 * segment to their projection.
 */
export function railRoute(net: RailNetwork, start: number[], goal: number[]): number[][] {
  const s = locateOnRails(net, start), g = locateOnRails(net, goal);
  const out: number[][] = [[start[0], start[1]]];
  const push = (p: number[]) => { const l = out[out.length - 1]; if (Math.hypot(p[0] - l[0], p[1] - l[1]) > 1) out.push([p[0], p[1]]); };
  push(s.q);
  if (s.rail >= 0 && s.rail === g.rail) { push(g.q); push(goal); return out; }
  const axis = sub(net.corridor[1], net.corridor[0]), al = len(axis) || 1;
  const u = [axis[0] / al, axis[1] / al];
  const along = (p: number[]) => dot(sub(p, net.corridor[0]), u);
  const sJ = s.rail >= 0 ? net.rails[s.rail][0] : s.q, gJ = g.rail >= 0 ? net.rails[g.rail][0] : g.q;
  const dirSign = Math.sign(along(gJ) - along(sJ)) || 1;
  // right-hand lane: offset to the right of the travel direction along the corridor
  const right = [u[1] * dirSign, -u[0] * dirSign];
  const off = net.corridorWidth / 4;
  const lane = (p: number[]) => { const a = along(p); return [net.corridor[0][0] + u[0] * a + right[0] * off, net.corridor[0][1] + u[1] * a + right[1] * off]; };
  if (Math.abs(along(gJ) - along(sJ)) > 1) { push(lane(sJ)); push(lane(gJ)); }
  else push(lane(sJ));
  if (g.rail >= 0) push(gJ);
  push(g.q); push(goal);
  return out;
}
