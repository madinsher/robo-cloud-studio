/**
 * Packing line demo acts: the KR16 palletizer is bound to the process flow (robot-driven transfer component,
 * src/vc/component.ts `motion`): it picks every crate at the conveyor end, the crate rides on the gripper, and it
 * stacks it on the pallet slot of the buffer grid; the equivalent robot program "Palletize" compiles (all targets
 * reachable). VerticalBot demo: the 4-axis palletizer program picks the 4 boxes and stacks them on the pallet.
 */
import { describe, it, expect } from 'vitest';
import { demos } from '../src/demos';
import { ItemType, SceneObject } from '../src/core/items/item';
import { Robot } from '../src/core/items/robot';
import { Program } from '../src/core/items/program';
import { Component, ProcessSimulator } from '../src/vc/component';
import { ProgramSimulator } from '../src/core/motion/simulator';
import { getPos, multiply, transl } from '../src/core/math/pose';

describe('packing line: palletizer bound to the process flow', () => {
  it('picks each crate at the conveyor end, carries it on the gripper and stacks it on its pallet slot', () => {
    const st = demos.find((d) => d.id === 'packing')!.build();
    const comps = st.itemsOfType<Component>(ItemType.COMPONENT);
    const cell = comps.find((c) => c.name === 'Palletizer cell')!;
    const pallet = comps.find((c) => c.name === 'Pallet')!;
    const robot = st.itemsOfType<Robot>(ItemType.ROBOT).find((r) => r.name === 'Palletizer')!;
    const sim = new ProcessSimulator(st);
    const g = (pallet.behaviour as any).grid;
    let carried = 0, offsetDrift = 0, maxJointStep = 0, prevQ = robot.joints(), released = 0, releaseErr = 0;
    const seenOnPallet = new Set<string>();
    let gripped: { id: string; d: number } | null = null;
    for (let i = 0; i < 4000 && pallet.products.length < 6; i++) {
      sim.step(0.1);
      const q = robot.joints(); maxJointStep = Math.max(maxJointStep, ...q.map((v, k) => Math.abs(v - prevQ[k]))); prevQ = q;
      const tcp = getPos(robot.poseTCPAbs());
      if (cell.runtime.grip && cell.products[0]) {
        const p = getPos((st.findById(cell.products[0]) as SceneObject).poseAbs());
        const d = Math.hypot(p[0] - tcp[0], p[1] - tcp[1], p[2] - tcp[2]);
        if (!gripped || gripped.id !== cell.products[0]) gripped = { id: cell.products[0], d };
        offsetDrift = Math.max(offsetDrift, Math.abs(d - gripped.d));
        carried++;
      }
      for (const [k, pid] of pallet.products.entries()) {
        if (seenOnPallet.has(pid)) continue;
        seenOnPallet.add(pid);
        released++;
        // the crate sits on its grid slot and the gripper (crate top) was there when it let go
        const slot = getPos(multiply(pallet.poseAbs(), transl((k % g.cols) * g.pitch[0], (Math.floor(k / g.cols) % g.rows) * g.pitch[1], Math.floor(k / (g.cols * g.rows)) * g.pitch[2])));
        const crate = getPos((st.findById(pid) as SceneObject).poseAbs());
        expect(Math.hypot(crate[0] - slot[0], crate[1] - slot[1], crate[2] - slot[2])).toBeLessThan(1);
        releaseErr = Math.max(releaseErr, Math.hypot(tcp[0] - slot[0], tcp[1] - slot[1], tcp[2] - (slot[2] + 250)));
      }
    }
    expect(pallet.products.length).toBe(6);
    expect(released).toBe(6);
    expect(carried).toBeGreaterThan(100);
    expect(offsetDrift).toBeLessThan(1); // rigidly on the gripper
    expect(releaseErr).toBeLessThan(5); // the robot put it there
    expect(maxJointStep).toBeLessThan(15); // smooth joint motion (deg per 0.1 s)
    expect(sim.signals.get('part_at_pick')).not.toBeUndefined();
  }, 120000);

  it('the equivalent robot program compiles: every pick / slot target is reachable', () => {
    const st = demos.find((d) => d.id === 'packing')!.build();
    const prog = st.itemsOfType<Program>(ItemType.PROGRAM).find((p) => p.name === 'Palletize')!;
    expect(prog.instructions().filter((i) => i.data.kind === 'move').length).toBe(1 + 12 * 6);
    const ps = new ProgramSimulator(st);
    const r = ps.compile(prog);
    expect(r.ok, JSON.stringify(r.problems.filter((p) => p.severity === 'error')).slice(0, 300)).toBe(true);
    expect(r.duration).toBeGreaterThan(12 * 2);
  }, 120000);
});

describe('VerticalBot demo acts', () => {
  it('the 4-axis palletizer program picks the 4 cartons from the table and stacks them on the pallet slots', () => {
    const st = demos.find((d) => d.id === 'verticalbot')!.build();
    const prog = st.itemsOfType<Program>(ItemType.PROGRAM).find((p) => p.name === 'Palletize layer')!;
    const boxes = st.itemsOfType<SceneObject>(ItemType.OBJECT).filter((o) => /^Box \d$/.test(o.name));
    expect(boxes).toHaveLength(4);
    const pallet = st.find('Pallet')!;
    const sim = new ProgramSimulator(st);
    const r = sim.compile(prog);
    expect(r.problems).toEqual([]);
    sim.runToEnd();
    boxes.forEach((b, i) => {
      const place = st.find(`Place ${i + 1}`)!;
      const bp = getPos(b.poseAbs()), tp = getPos(place.poseAbs());
      // carton base right under the place target (target = carton top), on the pallet surface
      expect(Math.hypot(bp[0] - tp[0], bp[1] - tp[1])).toBeLessThan(1);
      expect(bp[2]).toBeCloseTo(pallet.poseAbs()[14], 0);
    });
    expect(sim.attachments.size).toBe(0);
  }, 120000);
});
