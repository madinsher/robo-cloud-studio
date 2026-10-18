import { describe, it, expect } from 'vitest';
import { meshDataToGeometry, smoothTriangleSoup } from '../src/scene/three-geometry';

function quad(a: number[], b: number[], c: number[], d: number[]): number[] {
  return [...a, ...b, ...c, ...a, ...c, ...d];
}

describe('smooth triangle soup', () => {
  it('welds a cube and keeps the 90 degree corners hard', () => {
    const p = [
      [0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0],
      [0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1],
    ];
    const pos = new Float32Array([
      ...quad(p[0], p[1], p[2], p[3]),
      ...quad(p[4], p[5], p[6], p[7]),
      ...quad(p[0], p[1], p[5], p[4]),
      ...quad(p[3], p[2], p[6], p[7]),
      ...quad(p[0], p[3], p[7], p[4]),
      ...quad(p[1], p[2], p[6], p[5]),
    ]);
    const g = smoothTriangleSoup(pos);
    expect(g.getAttribute('position').count).toBe(24);
    expect(g.getIndex()!.count).toBe(36);
  });

  it('averages normals across a shallow bend and keeps a cube corner split', () => {
    const pos = new Float32Array([
      0, -1, 0.3, 1, -1, 0.3, 1, 0, 0,
      0, -1, 0.3, 1, 0, 0, 0, 0, 0,
      0, 0, 0, 1, 0, 0, 1, 1, 0.3,
      0, 0, 0, 1, 1, 0.3, 0, 1, 0.3,
    ]);
    const g = meshDataToGeometry({ positions: pos, normals: new Float32Array(0), min: [0, -1, 0], max: [1, 1, 0.3], triangles: 4 });
    const p = g.getAttribute('position');
    const n = g.getAttribute('normal');
    let ridge = false;
    for (let i = 0; i < p.count; i++) {
      if (Math.abs(p.getY(i)) < 1e-4 && Math.abs(p.getZ(i)) < 1e-4) {
        ridge = true;
        expect(Math.abs(n.getY(i))).toBeLessThan(0.2);
        expect(n.getZ(i)).toBeGreaterThan(0.9);
      }
    }
    expect(ridge).toBe(true);
  });
});
