/**
 * Spray / deposition simulation — RoboDK `Spray_Add`, `Spray_SetState`, `Spray_GetStats`, `Spray_Clear`.
 * A spray gun is attached to a tool; when ON, each simulation sample projects a cone (or a particle set) from
 * the TCP along its Z axis onto the target object's surface samples; hits accumulate per surface point so that
 * coverage, overspray and thickness statistics can be reported.
 */
import { Station, SceneObject, Tool, Item } from '../items/item';
import { Robot } from '../items/robot';
import { Mat4, multiply, invert, transformPoint, getPos } from '../math/pose';
import { AssetStore } from '../../scene/assets';

export interface SprayParams {
  /** Particle emission: cone half-angle (deg) and max range (mm). */
  angleDeg: number;
  range: number;
  /** Nominal deposition rate (thickness units per second at 1 m). */
  rate: number;
  /** Surface sample spacing for the target (mm). */
  sampleSpacing: number;
}

export interface SprayGun {
  id: number;
  toolId: string;
  objectId: string;
  params: SprayParams;
  on: boolean;
  /** Surface sample points in the object frame and accumulated deposition. */
  samples: Array<{ p: [number, number, number]; n: [number, number, number]; dep: number }>;
  /** Simulated particles for rendering (world positions of the last sample). */
  particles: [number, number, number][];
  overspray: number;
  totalTime: number;
}

export class SpraySimulator {
  guns = new Map<number, SprayGun>();
  private nextId = 1;
  constructor(readonly station: Station, readonly assets?: AssetStore) {}

  add(tool: Tool, object: SceneObject, params: Partial<SprayParams> = {}): SprayGun {
    const p: SprayParams = { angleDeg: 15, range: 400, rate: 1, sampleSpacing: 20, ...params };
    const gun: SprayGun = { id: this.nextId++, toolId: tool.id, objectId: object.id, params: p, on: false, samples: this.sampleSurface(object, p.sampleSpacing), particles: [], overspray: 0, totalTime: 0 };
    this.guns.set(gun.id, gun);
    return gun;
  }

  /** Sample the object's surface: mesh triangles (if available) or the faces of its bounding box / primitives. */
  private sampleSurface(obj: SceneObject, spacing: number): SprayGun['samples'] {
    const out: SprayGun['samples'] = [];
    for (const g of obj.geometry) {
      const origin = g.origin ? Float64Array.from(g.origin) : undefined;
      const tf = (p: [number, number, number]) => (origin ? transformPoint(origin, p) : p);
      const asset = g.mesh && this.assets ? this.assets.get(g.mesh) : undefined;
      if (asset?.mesh) {
        const m = asset.mesh;
        const s = g.scale ?? [1, 1, 1];
        for (let i = 0; i < m.triangles; i++) {
          const k = i * 9;
          const a: [number, number, number] = [m.positions[k] * s[0], m.positions[k + 1] * s[1], m.positions[k + 2] * s[2]];
          const b: [number, number, number] = [m.positions[k + 3] * s[0], m.positions[k + 4] * s[1], m.positions[k + 5] * s[2]];
          const c: [number, number, number] = [m.positions[k + 6] * s[0], m.positions[k + 7] * s[1], m.positions[k + 8] * s[2]];
          const nrm: [number, number, number] = [m.normals[k], m.normals[k + 1], m.normals[k + 2]];
          const area = triArea(a, b, c);
          const n = Math.max(1, Math.round(area / (spacing * spacing)));
          for (let q = 0; q < n; q++) {
            // R2 low-discrepancy pair (1/g, 1/g² with g the plastic constant): the two multipliers must not sum to 1,
            // otherwise u + v is always ~1 and every sample lands on the b-c edge of the triangle
            let u = ((0.5 + q * 0.7548777) % 1), v = ((0.5 + q * 0.5698403) % 1);
            if (u + v > 1) { u = 1 - u; v = 1 - v; }
            out.push({ p: tf([a[0] + (b[0] - a[0]) * u + (c[0] - a[0]) * v, a[1] + (b[1] - a[1]) * u + (c[1] - a[1]) * v, a[2] + (b[2] - a[2]) * u + (c[2] - a[2]) * v]), n: nrm, dep: 0 });
          }
        }
      } else if (g.primitive?.kind === 'box') {
        const [sx, sy, sz] = g.primitive.size;
        const faces: Array<[number, [number, number, number]]> = [[0, [1, 0, 0]], [0, [-1, 0, 0]], [1, [0, 1, 0]], [1, [0, -1, 0]], [2, [0, 0, 1]], [2, [0, 0, -1]]];
        const half = [sx / 2, sy / 2, sz / 2];
        for (const [axis, nrm] of faces) {
          const u = (axis + 1) % 3, v = (axis + 2) % 3;
          const nu = Math.max(1, Math.round((2 * half[u]) / spacing)), nv = Math.max(1, Math.round((2 * half[v]) / spacing));
          for (let i = 0; i < nu; i++) for (let j = 0; j < nv; j++) {
            const p: [number, number, number] = [0, 0, 0];
            p[axis] = nrm[axis] * half[axis];
            p[u] = -half[u] + ((i + 0.5) * 2 * half[u]) / nu;
            p[v] = -half[v] + ((j + 0.5) * 2 * half[v]) / nv;
            out.push({ p: tf(p), n: nrm, dep: 0 });
          }
        }
      } else if (g.primitive?.kind === 'cylinder' || g.primitive?.kind === 'sphere') {
        const r = g.primitive.radius;
        const L = g.primitive.kind === 'cylinder' ? g.primitive.length : 2 * r;
        const na = Math.max(8, Math.round((2 * Math.PI * r) / spacing)), nl = Math.max(1, Math.round(L / spacing));
        for (let i = 0; i < na; i++) for (let j = 0; j < nl; j++) {
          const a = (2 * Math.PI * i) / na;
          const z = -L / 2 + ((j + 0.5) * L) / nl;
          if (g.primitive.kind === 'sphere') { const ph = Math.acos(Math.max(-1, Math.min(1, z / r))); const rr = r * Math.sin(ph); out.push({ p: tf([rr * Math.cos(a), rr * Math.sin(a), z]), n: [Math.cos(a) * Math.sin(ph), Math.sin(a) * Math.sin(ph), z / r], dep: 0 }); }
          else out.push({ p: tf([r * Math.cos(a), r * Math.sin(a), z]), n: [Math.cos(a), Math.sin(a), 0], dep: 0 });
        }
      }
    }
    if (!out.length && obj.bbox) {
      const b = obj.bbox;
      const spacingN = spacing;
      for (let x = b.min[0]; x <= b.max[0]; x += spacingN) for (let y = b.min[1]; y <= b.max[1]; y += spacingN) out.push({ p: [x, y, b.max[2]], n: [0, 0, 1], dep: 0 });
    }
    return out;
  }

  setState(id: number | 'all', on: boolean): void {
    for (const g of this.guns.values()) if (id === 'all' || g.id === id) g.on = on;
  }
  clear(id: number | 'all' = 'all'): void {
    for (const g of [...this.guns.values()]) if (id === 'all' || g.id === id) { for (const s of g.samples) s.dep = 0; g.overspray = 0; g.totalTime = 0; g.particles = []; }
  }
  remove(id: number | 'all' = 'all'): void {
    if (id === 'all') this.guns.clear(); else this.guns.delete(id);
  }

  /** Advance deposition for dt seconds at the current robot state. */
  step(dt: number): void {
    for (const g of this.guns.values()) {
      if (!g.on) continue;
      const tool = this.station.findById(g.toolId) as Tool | null;
      const obj = this.station.findById(g.objectId) as SceneObject | null;
      if (!tool || !obj) continue;
      const tcpAbs = tool.poseAbs();
      const objInv = invert(obj.poseAbs());
      const tcpInObj = multiply(objInv, tcpAbs);
      const o = getPos(tcpInObj);
      const dir: [number, number, number] = [tcpInObj[8], tcpInObj[9], tcpInObj[10]];
      const cosA = Math.cos((g.params.angleDeg * Math.PI) / 180);
      let hits = 0;
      g.particles = [];
      for (const s of g.samples) {
        const v: [number, number, number] = [s.p[0] - o[0], s.p[1] - o[1], s.p[2] - o[2]];
        const dist = Math.hypot(v[0], v[1], v[2]);
        if (dist < 1e-6 || dist > g.params.range) continue;
        const c = (v[0] * dir[0] + v[1] * dir[1] + v[2] * dir[2]) / dist;
        if (c < cosA) continue;
        // facing check: surface normal against spray direction
        const facing = -(s.n[0] * dir[0] + s.n[1] * dir[1] + s.n[2] * dir[2]);
        if (facing <= 0.05) continue;
        // deposition ~ rate * facing / (dist/1000)² * gaussian across the cone
        const falloff = Math.exp(-(((1 - c) / (1 - cosA)) ** 2) * 2);
        s.dep += (g.params.rate * facing * falloff * dt) / Math.max(0.05, (dist / 1000) ** 2);
        hits++;
        if (g.particles.length < 200) g.particles.push(transformPoint(obj.poseAbs(), s.p));
      }
      g.totalTime += dt;
      if (!hits) g.overspray += dt;
    }
  }

  /** Stats like RoboDK: [coverage %, mean thickness, min, max, overspray time]. */
  stats(id: number | 'all' = 'all'): { coverage: number; mean: number; min: number; max: number; oversprayTime: number; samples: number } {
    const guns = id === 'all' ? [...this.guns.values()] : [this.guns.get(id)].filter(Boolean) as SprayGun[];
    let n = 0, covered = 0, sum = 0, min = Infinity, max = 0, over = 0;
    for (const g of guns) {
      over += g.overspray;
      for (const s of g.samples) { n++; if (s.dep > 0) covered++; sum += s.dep; min = Math.min(min, s.dep); max = Math.max(max, s.dep); }
    }
    return { coverage: n ? (100 * covered) / n : 0, mean: n ? sum / n : 0, min: n ? min : 0, max, oversprayTime: over, samples: n };
  }
}

function triArea(a: number[], b: number[], c: number[]): number {
  const ux = b[0] - a[0], uy = b[1] - a[1], uz = b[2] - a[2], vx = c[0] - a[0], vy = c[1] - a[1], vz = c[2] - a[2];
  return 0.5 * Math.hypot(uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx);
}

export { Robot };
