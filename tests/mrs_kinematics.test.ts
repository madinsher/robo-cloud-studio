/**
 * Group control on the fleet is kinematically compliant by default (B22): for every runnable course example the robots
 * are driven through their own controller — measured speed ≤ 1.05 × kin.maxSpeed, turn rate ≤ kin.maxYawRate, no
 * teleport (per-tick displacement bounded by the speed limit). `drive: 'pose'` remains the ideal-model visualisation.
 */
import { describe, it, expect } from 'vitest';
import { Station } from '../src/core/items/item';
import { MRS_EXAMPLES } from '../src/mrs/examples';
import { buildGroupScene } from '../src/mrs/scene';
import { FleetRuntime, RUNNABLE_KINDS } from '../src/mrs/runtime';
import { parseSwarm } from '../src/mrs/dsl';

const runnable = MRS_EXAMPLES.filter((e) => RUNNABLE_KINDS.includes(e.kind) && (e.kind !== 'swarm' || ['boids', 'vicsek', 'pso'].includes(parseSwarm(e.source).model)));

function measure(rt: FleetRuntime, seconds: number, dt = 0.05) {
  const prev = rt.robots.map((r) => [r.state.x, r.state.y, r.state.theta]); let vOver = 0, wMax = 0, jump = 0;
  for (let i = 0; i < seconds / dt && !rt.done; i++) {
    rt.tick(dt);
    rt.robots.forEach((r, k) => {
      const [x, y, th] = prev[k]; const d = Math.hypot(r.state.x - x, r.state.y - y); let dth = Math.abs(r.state.theta - th) % 360; if (dth > 180) dth = 360 - dth;
      vOver = Math.max(vOver, d / dt / r.kin.maxSpeed); wMax = Math.max(wMax, dth / dt / r.kin.maxYawRate); jump = Math.max(jump, d / (r.kin.maxSpeed * dt)); prev[k] = [r.state.x, r.state.y, r.state.theta];
    });
  }
  return { vOver, wMax, jump };
}

describe('fleet runtime: kinematic compliance of every runnable example (B22)', () => {
  it(`covers the runnable examples (${runnable.length})`, () => { expect(runnable.length).toBeGreaterThanOrEqual(20); });
  for (const ex of runnable) {
    it(`${ex.id}: speed ≤ 1.05·vmax, turn rate ≤ yaw-rate limit, no teleport`, () => {
      const st = new Station(); const robots = buildGroupScene(st, ex.kind, ex.source);
      const rt = new FleetRuntime({ kind: ex.kind, source: ex.source, robots, station: st, maxSeconds: 120 });
      const m = measure(rt, 60);
      expect(m.vOver).toBeLessThanOrEqual(1.05); expect(m.wMax).toBeLessThanOrEqual(1.0 + 1e-6); expect(m.jump).toBeLessThanOrEqual(1.05);
    });
  }
  it('pose mode stays available as the ideal model and is not compliant (Vicsek torus wrap teleports)', () => {
    const ex = runnable.find((e) => e.kind === 'swarm' && parseSwarm(e.source).model === 'vicsek')!;
    const st = new Station(); const robots = buildGroupScene(st, ex.kind, ex.source);
    const m = measure(new FleetRuntime({ kind: ex.kind, source: ex.source, robots, station: st, maxSeconds: 120, drive: 'pose' }), 60);
    expect(m.jump).toBeGreaterThan(1.05);
  });
});
