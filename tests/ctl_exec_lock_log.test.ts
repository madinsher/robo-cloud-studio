/**
 * Control executors: the log says when a robot program really begins. Two transitions of the same robot enabled
 * together both fire at 0 s, but the second program waits for the robot (one action per robot, FIFO) — the log used to
 * say only "tC started: program A_Place on R1" at 0.0 s although A_Place ran from 2.3 s (V4 F15). Now the wait and the
 * real start are logged with their times, on the station world (Run on station) and on the simulated host.
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
const SAME_ROBOT = `petri Same robot twice
place a1 tokens=1
place c1 tokens=1
place b1 tokens=1
place a2
place c2
place b2
transition tA
transition tC
transition tB
arc a1 -> tA -> a2
arc c1 -> tC -> c2
arc b1 -> tB -> b2
action tA program=A_Pick robot=R1
action tC program=A_Place robot=R1
action tB program=B_Work robot=R2`;

function stationWorld() {
  const st = new Station('s'); const robots = new Map<string, ReturnType<typeof createRobotFromLibrary>>();
  for (const [name, p] of Object.entries(PROGRAMS)) { let r = robots.get(p.robot); if (!r) { r = st.addChild(createRobotFromLibrary('UR5e', p.robot)); robots.set(p.robot, r); } st.addChild(new Program(name)).setRobot(r); }
  return makeStationWorld({ station: st, programs: simulatedBackend(Object.fromEntries(Object.entries(PROGRAMS).map(([n, p]) => [n, p.duration]))) });
}
const at = (log: string[], re: RegExp) => { const l = log.find((x) => re.test(x)); return l ? Number(/^\[([\d.]+)\]/.exec(l)?.[1] ?? NaN) : null; };

describe('control executors: a program waiting for its robot is logged as waiting, then as started', () => {
  it('Petri net on the station world: tC fires at 0 s, A_Place waits for R1 and starts after A_Pick (2.3 s)', () => {
    const ex = new PetriExecutor({ doc: parsePetri(SAME_ROBOT), world: stationWorld() });
    for (let i = 0; i < 400 && ex.status === 'running'; i++) ex.tick(0.05);
    expect(ex.verdict, ex.log.join('\n')).toBe('done');
    const log = ex.log;
    expect(at(log, /tC started: program A_Place on R1/)).toBeCloseTo(0, 6);
    expect(at(log, /program A_Place waits for R1 \(busy\)/), log.join('\n')).toBeCloseTo(0, 6);
    const begin = at(log, /program A_Place starts on R1 after waiting 2\.\d\d s/);
    expect(begin, log.join('\n')).not.toBeNull();
    expect(begin!).toBeGreaterThanOrEqual(2.3 - 1e-6); expect(begin!).toBeLessThan(2.5);
    // the programs that got their robot at once are not reported as waiting
    expect(log.filter((l) => /A_Pick waits|B_Work waits/.test(l))).toEqual([]);
  });
});
