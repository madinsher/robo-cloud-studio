/**
 * Petri executor: a race whose loser stays marked is not a partial deadlock when a declared final marking is still
 * reachable, and a real partial deadlock lets the firings in progress complete (no robot stopped in mid-motion).
 */
import { describe, it, expect } from 'vitest';
import { Station } from '../src/core/items/item';
import { Program } from '../src/core/items/program';
import { createRobotFromLibrary } from '../src/core/items/library';
import { parsePetri } from '../src/ctl/dsl';
import { PetriExecutor } from '../src/ctl/exec';
import { simulatedBackend } from '../src/ctl/host';
import { makeStationWorld } from '../src/ctl/world';

const PROGRAMS: Record<string, { robot: string; duration: number }> = { A_Pick: { robot: 'R1', duration: 2.3 }, A_Place: { robot: 'R1', duration: 2.6 }, B_Work: { robot: 'R2', duration: 6.6 } };
function world() {
  const st = new Station('s'); const robots = new Map<string, ReturnType<typeof createRobotFromLibrary>>();
  for (const [name, p] of Object.entries(PROGRAMS)) { let r = robots.get(p.robot); if (!r) { r = st.addChild(createRobotFromLibrary('UR5e', p.robot)); robots.set(p.robot, r); } st.addChild(new Program(name)).setRobot(r); }
  const be = simulatedBackend(Object.fromEntries(Object.entries(PROGRAMS).map(([n, p]) => [n, p.duration])));
  return { world: makeStationWorld({ station: st, programs: be }), be };
}
const RACE = (finals: boolean) => `petri Race between a robot program and a timer
place p0 tokens=1
place a1
place d1
place tok tokens=1
place a2
place d2
place xA
place xB
transition tF
transition tA
transition tD delay=2
transition tX
transition tY
arc p0 -> tF -> a1, d1
arc a1 -> tA -> a2
arc d1 -> tD -> d2
arc a2, tok -> tX -> xA
arc d2, tok -> tY -> xB
${finals ? 'final xA d2\nfinal xB a2\n' : ''}action tA program=A_Pick robot=R1
action tX program=A_Place robot=R1
action tY program=B_Work robot=R2`;

describe('Petri executor: final markings and partial deadlocks', () => {
  it('race with declared finals: the timer wins, B_Work runs to its end and the run ends done at {a2 xB}', () => {
    const { world: w, be } = world();
    const x = new PetriExecutor({ doc: parsePetri(RACE(true)), world: w }); x.run(30, 0.05);
    expect(x.log.some((l) => /DEADLOCK/.test(l)), x.log.join('\n')).toBe(false);
    expect(x.verdict).toBe('done'); expect(x.where()).toBe('a2 xB');
    expect(be.log.filter((e) => e.program === 'B_Work').map((e) => e.event)).toEqual(['start', 'end']);
    expect(be.log.some((e) => e.event === 'stop')).toBe(false);
    expect(x.time).toBeGreaterThan(8.5); expect(x.time).toBeLessThan(9);
  });
  it('the same race without finals is a partial deadlock (a2 is never left), but B_Work completes before the run stops', () => {
    const { world: w, be } = world();
    const x = new PetriExecutor({ doc: parsePetri(RACE(false)), world: w }); x.run(30, 0.05);
    const dl = x.log.findIndex((l) => /DEADLOCK \(partial\)/.test(l)); expect(dl).toBeGreaterThanOrEqual(0);
    expect(x.log[dl]).toMatch(/^\[2\.[0-9]\]/); // reported when A_Pick has put its token into a2
    expect(x.verdict).toBe('deadlock'); expect(x.status).toBe('idle');
    expect(be.log.filter((e) => e.program === 'B_Work').map((e) => e.event)).toEqual(['start', 'end']);
    expect(be.log.some((e) => e.event === 'stop')).toBe(false);
    expect(x.where()).toBe('a2 xB');
  });
});
