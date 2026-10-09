import { it, expect } from 'vitest';
import { demos } from '../src/demos';
import { cloneItem, clipboard, uniqueName } from '../src/core/items/clone';
import { Program, Instruction } from '../src/core/items/program';
import { Robot } from '../src/core/items/robot';
import { ProgramSimulator } from '../src/core/motion/simulator';

it('clones a robot with its tool and a program with remapped target references', () => {
  const st = demos[0].build();
  const robot = st.find('UR10e') as Robot;
  const copy = cloneItem(robot) as Robot;
  expect(copy.id).not.toBe(robot.id);
  expect(copy.name).toBe('UR10e 2');
  expect(copy.tools().length).toBe(1);
  expect(copy.activeTool()?.id).toBe(copy.tools()[0].id); // active tool id remapped
  const prog = st.find('PickPlace') as Program;
  clipboard.copy(prog);
  const pasted = clipboard.paste(st, st) as Program;
  expect(pasted.name).toBe('PickPlace 2');
  expect(pasted.instructions().length).toBe(prog.instructions().length);
  // targets referenced by the pasted program still resolve (they were not copied, ids point to originals)
  const res = new ProgramSimulator(st).compile(pasted);
  expect(res.problems.filter((p) => p.severity === 'error')).toEqual([]);
  expect(uniqueName(st, 'Box 1')).toBe('Box 4');
  void Instruction;
});
