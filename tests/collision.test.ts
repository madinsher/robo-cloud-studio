import { describe, it, expect } from 'vitest';
import { Station, SceneObject, Tool } from '../src/core/items/item';
import { createRobotFromLibrary } from '../src/core/items/library';
import { checkCollisions, shapeDistance, checkRobotCollisions } from '../src/core/collision/collision';
import { transl, mul, rotz, DEG } from '../src/core/math/pose';

describe('collision detection', () => {
  it('primitive distances', () => {
    expect(shapeDistance({ kind: 'sphere', c: [0, 0, 0], r: 10 }, { kind: 'sphere', c: [15, 0, 0], r: 10 })).toBeCloseTo(5);
    expect(shapeDistance({ kind: 'capsule', a: [0, 0, 0], b: [100, 0, 0], r: 10 }, { kind: 'sphere', c: [50, 25, 0], r: 10 })).toBeCloseTo(-5);
    const boxA = { kind: 'obb' as const, center: [0, 0, 0] as [number, number, number], axes: [[1, 0, 0], [0, 1, 0], [0, 0, 1]] as any, half: [50, 50, 50] as [number, number, number] };
    const boxB = { ...boxA, center: [90, 0, 0] as [number, number, number] };
    expect(shapeDistance(boxA, boxB)).toBeCloseTo(10);
    const boxC = { ...boxA, center: [120, 0, 0] as [number, number, number] };
    expect(shapeDistance(boxA, boxC)).toBeLessThan(0);
  });

  it('robot at home does not self-collide, but collides with a box placed in its arm', () => {
    const st = new Station();
    const r = st.addChild(createRobotFromLibrary('UR10e'));
    const tool = r.addChild(new Tool('T'));
    tool.setPoseTool(transl(0, 0, 100));
    r.setTool(tool);
    expect(checkCollisions(st)).toEqual([]);
    const box = st.addChild(new SceneObject('Box'));
    box.geometry = [{ primitive: { kind: 'box', size: [300, 300, 300] }, origin: Array.from(transl(0, 0, 150)), color: '#888' }];
    // place the box far away: no collision
    box.setPose(transl(3000, 0, 0));
    expect(checkCollisions(st)).toEqual([]);
    // place the box at the elbow
    const fk = r.fk();
    const elbow = fk.linkPoses[3];
    box.setPose(transl(elbow[12], elbow[13], elbow[14] - 150));
    const hits = checkRobotCollisions(st, r);
    expect(hits.length).toBeGreaterThan(0);
    expect(hits.some((h) => h.a.item === box || h.b.item === box)).toBe(true);
  });

  it('two robots close together collide, far apart do not', () => {
    const st = new Station();
    const a = st.addChild(createRobotFromLibrary('KUKA_KR6_R900'));
    const b = st.addChild(createRobotFromLibrary('KUKA_KR6_R900'));
    b.setPose(mul(transl(4000, 0, 0), rotz(180 * DEG)));
    expect(checkCollisions(st)).toEqual([]);
    b.setPose(transl(60, 0, 0));
    expect(checkCollisions(st).length).toBeGreaterThan(0);
  });
});
