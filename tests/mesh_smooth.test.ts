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

/** Box soup with outward winding (counter-clockwise seen from outside), corners at `lo` / `hi`. */
function boxSoup(lo: number[], hi: number[], subdiv = 1): Float32Array {
  const out: number[] = [];
  const face = (o: number[], u: number[], v: number[]) => {
    for (let i = 0; i < subdiv; i++) for (let j = 0; j < subdiv; j++) {
      const P = (a: number, b: number) => [0, 1, 2].map((k) => o[k] + u[k] * a / subdiv + v[k] * b / subdiv);
      out.push(...P(i, j), ...P(i + 1, j), ...P(i + 1, j + 1), ...P(i, j), ...P(i + 1, j + 1), ...P(i, j + 1));
    }
  };
  const [x0, y0, z0] = lo, [x1, y1, z1] = hi, dx = x1 - x0, dy = y1 - y0, dz = z1 - z0;
  face([x0, y0, z0], [0, dy, 0], [dx, 0, 0]);   // bottom (-z)
  face([x0, y0, z1], [dx, 0, 0], [0, dy, 0]);   // top (+z)
  face([x0, y0, z0], [dx, 0, 0], [0, 0, dz]);   // front (-y)
  face([x0, y1, z0], [0, 0, dz], [dx, 0, 0]);   // back (+y)
  face([x0, y0, z0], [0, 0, dz], [0, dy, 0]);   // left (-x)
  face([x1, y0, z0], [0, dy, 0], [0, 0, dz]);   // right (+x)
  return new Float32Array(out);
}
/** Largest distance between a soup corner and the vertex it was welded to. */
function weldError(pos: Float32Array, g: ReturnType<typeof smoothTriangleSoup>): number {
  const p = g.getAttribute('position'), idx = g.getIndex()!;
  let worst = 0;
  for (let i = 0; i < idx.count; i++) { const k = idx.getX(i); worst = Math.max(worst, Math.hypot(p.getX(k) - pos[i * 3], p.getY(k) - pos[i * 3 + 1], p.getZ(k) - pos[i * 3 + 2])); }
  return worst;
}
const signedVolume = (g: ReturnType<typeof smoothTriangleSoup>) => {
  const p = g.getAttribute('position'), idx = g.getIndex()!;
  let v = 0;
  for (let t = 0; t < idx.count; t += 3) {
    const a = [p.getX(idx.getX(t)), p.getY(idx.getX(t)), p.getZ(idx.getX(t))], b = [p.getX(idx.getX(t + 1)), p.getY(idx.getX(t + 1)), p.getZ(idx.getX(t + 1))], c = [p.getX(idx.getX(t + 2)), p.getY(idx.getX(t + 2)), p.getZ(idx.getX(t + 2))];
    v += a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) + a[2] * (b[0] * c[1] - b[1] * c[0]);
  }
  return v / 6;
};

describe('mesh units and winding (V5 / V7 findings)', () => {
  it('a ROS mesh in metres keeps its shape: corners are welded on a grid relative to the mesh size, not 0.05 units', () => {
    // a 0.12 x 0.134 x 0.135 m UR5e-shoulder-sized box with 2 mm facets (fixed 0.05-unit welding collapsed it)
    const pos = boxSoup([-0.06, -0.067, 0], [0.06, 0.067, 0.135], 60);
    const g = smoothTriangleSoup(pos);
    expect(weldError(pos, g)).toBeLessThan(1e-6);
    g.computeBoundingBox();
    const s = new (g.boundingBox!.max.constructor as any)().subVectors(g.boundingBox!.max, g.boundingBox!.min);
    expect([s.x, s.y, s.z].map((v: number) => +v.toFixed(4))).toEqual([0.12, 0.134, 0.135]);
    // the same mesh in millimetres welds exactly as before
    const mm = boxSoup([-60, -67, 0], [60, 67, 135], 60);
    expect(weldError(mm, smoothTriangleSoup(mm))).toBeLessThan(1e-3);
  });

  it('a mirrored (inward-wound) closed STL is turned outwards so its faces are not culled', () => {
    const out = boxSoup([0, 0, 0], [100, 50, 20]);
    const mirrored = new Float32Array(out.length);
    for (let i = 0; i < out.length; i += 3) { mirrored[i] = -out[i]; mirrored[i + 1] = out[i + 1]; mirrored[i + 2] = out[i + 2]; }   // x -> -x flips the winding
    const mesh = (positions: Float32Array) => ({ positions, normals: new Float32Array(0), min: [0, 0, 0] as [number, number, number], max: [0, 0, 0] as [number, number, number], triangles: positions.length / 9 });
    const gOut = meshDataToGeometry(mesh(out)), gMir = meshDataToGeometry(mesh(mirrored));
    expect(signedVolume(gOut)).toBeCloseTo(100 * 50 * 20, 0);
    expect(signedVolume(gMir)).toBeCloseTo(100 * 50 * 20, 0);   // was -100000: every face back-facing, the forearm invisible
    expect(gMir.userData.doubleSided).toBe(false);
    // an open surface has no inside: drawn from both sides
    const open = new Float32Array(out.slice(0, 18));
    expect(meshDataToGeometry(mesh(open)).userData.doubleSided).toBe(true);
  });
});
