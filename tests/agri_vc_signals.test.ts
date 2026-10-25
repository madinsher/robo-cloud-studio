// B19: process components and robot programs share one signal table (StationRuntime.signals, contract I1): a robot
// program waits on a conveyor sensor and stops the conveyor; the process simulator reads what the program wrote.
import { describe, it, expect } from 'vitest';
import { Station } from '../src/core/items/item';
import { Program } from '../src/core/items/program';
import { StationRuntime } from '../src/core/motion/runtime';
import { Component, ProcessSimulator, makeConveyor, makeFeeder, makeSink } from '../src/vc/component';
import { transl } from '../src/core/math/pose';

describe('process components <-> robot programs (B19)', () => {
  it('a program waits for the end-of-conveyor sensor, then stops the conveyor through a signal', () => {
    const st = new Station('line');
    const feeder = st.addChild(makeFeeder('Feeder', 2, { name: 'Box', geometry: { primitive: { kind: 'box', size: [300, 200, 200] }, origin: Array.from(transl(0, 0, 100)), color: '#d9a066' } }));
    const conv = st.addChild(makeConveyor('Conveyor', 4000, 400, 400, 0));
    (conv.behaviour as any).stopSignal = 'stop_conveyor';
    const sink = st.addChild(makeSink('Sink'));
    const sensor = st.addChild(new Component('End sensor', { type: 'sensor', enabled: true, signal: 'part_at_end', range: 250 } as any));
    sensor.setPose(transl(3800, 0, 0));
    (feeder.behaviour as any).next = conv.id;
    (conv.behaviour as any).next = sink.id;
    const prog = st.addChild(new Program('Stop when a part arrives'));
    prog.addInstruction({ kind: 'signal', signal: 'part_at_end', value: true, wait: true });
    prog.addInstruction({ kind: 'signal', signal: 'stop_conveyor', value: true, wait: false });
    const sim = new ProcessSimulator(st);
    const rt = StationRuntime.of(st);
    const run = rt.start(prog);
    let tSensor = -1, tDone = -1;
    for (let i = 0; i < 600; i++) {
      sim.step(0.05);
      rt.tick(0.05);
      if (tSensor < 0 && rt.signals.get('part_at_end') === true) tSensor = sim.time;
      if (tDone < 0 && run.status === 'done') tDone = sim.time;
    }
    // the first box is fed at t = 0 and needs (3800 - 250) / 400 = 8.9 s to enter the sensor range: the program waited
    expect(tSensor).toBeGreaterThan(8.8);
    expect(tSensor).toBeLessThan(9.1);
    expect(tDone).toBeGreaterThanOrEqual(tSensor);
    expect(tDone - tSensor).toBeLessThan(0.2);
    // the conveyor read the program's signal: it stopped, nothing reached the sink
    expect(sim.signals.get('stop_conveyor')).toBe(true);
    expect((sink.behaviour as any).count).toBe(0);
    const held = conv.products.length;
    for (let i = 0; i < 100; i++) sim.step(0.05);
    expect(conv.products.length).toBeGreaterThanOrEqual(held);
    expect((sink.behaviour as any).count).toBe(0);
    // and the component signal is visible to everything reading the station table
    expect(rt.signals.snapshot().part_at_end).toBe(true);
  });
});
