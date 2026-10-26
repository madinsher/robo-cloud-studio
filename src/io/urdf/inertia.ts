/**
 * Mass properties of studio-native robot links estimated from their geometry, for the URDF export (`<inertial>`):
 * every visual is a uniform-density solid — closed meshes by exact tetrahedral integration (signed volumes from the
 * link origin: volume, centre of mass and the second moments ∫x xᵀ dV = V/20 (Σ v vᵀ + s sᵀ) per tetrahedron),
 * primitives analytically — combined with the parallel-axis theorem. Open or inside-out meshes (|V| < 2 % of their
 * bounding box) fall back to their bounding box. Density: `density` kg/m³ (default 1200, a cast-aluminium arm with
 * its cavities), or scaled so the links sum to the robot mass when it is known.
 */
import type { Mat4 } from '../../core/math/pose';
import type { LinkVisual } from '../../core/kinematics/chain';
import type { MeshData } from '../mesh/stl';

export type Vec3 = [number, number, number];
/** Mass (kg), centre of mass (m, link frame), inertia about the centre of mass (kg·m², link axes): ixx ixy ixz iyy iyz izz. */
export interface MassProps { mass: number; com: Vec3; I: [number, number, number, number, number, number] }

/** Volume (m³), first moment (m⁴) and second moments ∫x_i x_j dV (m⁵) of a solid. */
interface Moments { V: number; S: Vec3; C: number[] /* 3x3 row-major */ }

const zero = (): Moments => ({ V: 0, S: [0, 0, 0], C: [0, 0, 0, 0, 0, 0, 0, 0, 0] });
function add(a: Moments, b: Moments): Moments { return { V: a.V + b.V, S: [a.S[0] + b.S[0], a.S[1] + b.S[1], a.S[2] + b.S[2]], C: a.C.map((v, i) => v + b.C[i]) }; }

/** Moments of a closed triangle soup (9 numbers per triangle, metres). */
export function meshMoments(tri: ArrayLike<number>): Moments {
  const m = zero();
  for (let k = 0; k + 8 < tri.length; k += 9) {
    const a = [tri[k], tri[k + 1], tri[k + 2]], b = [tri[k + 3], tri[k + 4], tri[k + 5]], c = [tri[k + 6], tri[k + 7], tri[k + 8]];
    const det = a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) + a[2] * (b[0] * c[1] - b[1] * c[0]);
    const V = det / 6;
    const s = [a[0] + b[0] + c[0], a[1] + b[1] + c[1], a[2] + b[2] + c[2]];
    m.V += V;
    for (let i = 0; i < 3; i++) m.S[i] += (V * s[i]) / 4;
    for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) m.C[i * 3 + j] += (V / 20) * (a[i] * a[j] + b[i] * b[j] + c[i] * c[j] + s[i] * s[j]);
  }
  return m;
}

/** Moments of a box of size `size` (m) centred at the origin of `T` (rotation + translation in metres). */
function boxMoments(size: Vec3, T: number[]): Moments {
  const V = size[0] * size[1] * size[2];
  const local = [size[0] ** 2 / 12, size[1] ** 2 / 12, size[2] ** 2 / 12].map((v) => v * V); // ∫x² dV about the centre
  return transformMoments({ V, S: [0, 0, 0], C: [local[0], 0, 0, 0, local[1], 0, 0, 0, local[2]] }, T);
}
function cylinderMoments(r: number, h: number, T: number[]): Moments {
  const V = Math.PI * r * r * h;
  const cxx = (V * r * r) / 4, czz = (V * h * h) / 12;
  return transformMoments({ V, S: [0, 0, 0], C: [cxx, 0, 0, 0, cxx, 0, 0, 0, czz] }, T);
}
function sphereMoments(r: number, T: number[]): Moments {
  const V = (4 / 3) * Math.PI * r ** 3, c = (V * r * r) / 5;
  return transformMoments({ V, S: [0, 0, 0], C: [c, 0, 0, 0, c, 0, 0, 0, c] }, T);
}
/** Moments of a solid given about its own frame, expressed in the frame where T (column-major, metres) places it. */
function transformMoments(m: Moments, T: number[]): Moments {
  const R = [T[0], T[4], T[8], T[1], T[5], T[9], T[2], T[6], T[10]]; // row-major rotation
  const t = [T[12], T[13], T[14]];
  // C' = R C Rᵀ + R S tᵀ + t (R S)ᵀ + V t tᵀ
  const RS = [0, 1, 2].map((i) => R[i * 3] * m.S[0] + R[i * 3 + 1] * m.S[1] + R[i * 3 + 2] * m.S[2]);
  const C: number[] = new Array(9).fill(0);
  for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) {
    let v = 0;
    for (let k = 0; k < 3; k++) for (let l = 0; l < 3; l++) v += R[i * 3 + k] * m.C[k * 3 + l] * R[j * 3 + l];
    C[i * 3 + j] = v + RS[i] * t[j] + t[i] * RS[j] + m.V * t[i] * t[j];
  }
  return { V: m.V, S: [RS[0] + m.V * t[0], RS[1] + m.V * t[1], RS[2] + m.V * t[2]], C };
}

/** Mass properties from moments at density rho. */
export function massProps(m: Moments, rho: number): MassProps | null {
  if (!(m.V > 0)) return null;
  const mass = rho * m.V;
  const com: Vec3 = [m.S[0] / m.V, m.S[1] / m.V, m.S[2] / m.V];
  // inertia about the origin: I = tr(C) 1 − C, then about the centre of mass (parallel axis)
  const C = m.C.map((v) => v * rho), tr = C[0] + C[4] + C[8];
  const Io = [tr - C[0], -C[1], -C[2], tr - C[4], -C[5], tr - C[8]];
  const c2 = com[0] ** 2 + com[1] ** 2 + com[2] ** 2;
  const I: MassProps['I'] = [Io[0] - mass * (c2 - com[0] ** 2), Io[1] + mass * com[0] * com[1], Io[2] + mass * com[0] * com[2], Io[3] - mass * (c2 - com[1] ** 2), Io[4] + mass * com[1] * com[2], Io[5] - mass * (c2 - com[2] ** 2)];
  return { mass, com, I };
}

const mmToM = (T: ArrayLike<number> | undefined): number[] => { const a = T ? Array.from(T) : [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]; a[12] /= 1000; a[13] /= 1000; a[14] /= 1000; return a; };

/** Solid moments of one visual (mm geometry, `origin` in the link frame) in metres. */
export function visualMoments(v: Pick<LinkVisual, 'mesh' | 'primitive' | 'origin' | 'scale'>, mesh?: MeshData | null): Moments | null {
  const T = mmToM(v.origin as ArrayLike<number> | undefined);
  const p = v.primitive as { kind: string; size?: number[]; radius?: number; length?: number } | undefined;
  if (p?.kind === 'box' && p.size) return boxMoments([p.size[0] / 1000, p.size[1] / 1000, p.size[2] / 1000], T);
  if ((p?.kind === 'cylinder' || p?.kind === 'cone') && p.radius && p.length) { const m = cylinderMoments(p.radius / 1000, p.length / 1000, T); if (p.kind === 'cone') { m.V /= 3; m.S = m.S.map((x) => x / 3) as Vec3; m.C = m.C.map((x) => x / 3); } return m; }
  if (p?.kind === 'sphere' && p.radius) return sphereMoments(p.radius / 1000, T);
  if (!mesh || !v.mesh) return null;
  const s = (v.scale as number[] | undefined) ?? [1, 1, 1];
  const pos = mesh.positions, n = Math.floor(pos.length / 9) * 9;
  const tri = new Float64Array(n);
  for (let i = 0; i < n; i += 3) { tri[i] = (pos[i] * s[0]) / 1000; tri[i + 1] = (pos[i + 1] * s[1]) / 1000; tri[i + 2] = (pos[i + 2] * s[2]) / 1000; }
  let m = meshMoments(tri);
  const bb = [(mesh.max[0] - mesh.min[0]) * Math.abs(s[0]) / 1000, (mesh.max[1] - mesh.min[1]) * Math.abs(s[1]) / 1000, (mesh.max[2] - mesh.min[2]) * Math.abs(s[2]) / 1000] as Vec3;
  const bbV = bb[0] * bb[1] * bb[2];
  if (m.V < 0) m = { V: -m.V, S: m.S.map((x) => -x) as Vec3, C: m.C.map((x) => -x) }; // inside-out winding
  if (!(m.V > 0.02 * bbV)) {
    const c = [((mesh.max[0] + mesh.min[0]) / 2) * s[0] / 1000, ((mesh.max[1] + mesh.min[1]) / 2) * s[1] / 1000, ((mesh.max[2] + mesh.min[2]) / 2) * s[2] / 1000];
    m = boxMoments(bb, [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, c[0], c[1], c[2], 1]);
  }
  return transformMoments(m, T);
}

/** Moments of a closed triangle soup given in mm in a frame placed by `local` (mm) in the link frame. */
export function soupMoments(tris: ArrayLike<number>, local: ArrayLike<number>): Moments {
  const t = new Float64Array(tris.length);
  for (let i = 0; i < t.length; i++) t[i] = tris[i] / 1000;
  let m = meshMoments(t);
  if (m.V < 0) m = { V: -m.V, S: m.S.map((x) => -x) as Vec3, C: m.C.map((x) => -x) };
  return transformMoments(m, mmToM(local));
}

/** Mass properties of a link from its visuals (null when it has no solid geometry); `solids`: extra closed soups
 *  (mm, placed by `local`), e.g. the procedural shapes drawn for links without meshes. */
export function linkMassProps(visuals: Array<Pick<LinkVisual, 'mesh' | 'primitive' | 'origin' | 'scale'>>, meshOf: (id: string) => MeshData | null | undefined, density = 1200, solids: Array<{ tris: ArrayLike<number>; local: ArrayLike<number> }> = []): MassProps | null {
  let acc: Moments | null = null;
  for (const v of visuals) { const m = visualMoments(v, v.mesh ? meshOf(v.mesh) : null); if (m) acc = acc ? add(acc, m) : m; }
  for (const s of solids) { const m = soupMoments(s.tris, s.local); acc = acc ? add(acc, m) : m; }
  return acc ? massProps(acc, density) : null;
}

/** `<inertial>` element (rotation-free: the tensor is given in the link axes about the centre of mass). */
export function inertialXml(p: MassProps, indent = '    '): string {
  const f = (v: number) => (Math.abs(v) < 1e-12 ? '0' : Number(v.toPrecision(7)).toString());
  return `${indent}<inertial>\n${indent}  <origin xyz="${p.com.map(f).join(' ')}" rpy="0 0 0"/>\n${indent}  <mass value="${f(p.mass)}"/>\n${indent}  <inertia ixx="${f(p.I[0])}" ixy="${f(p.I[1])}" ixz="${f(p.I[2])}" iyy="${f(p.I[3])}" iyz="${f(p.I[4])}" izz="${f(p.I[5])}"/>\n${indent}</inertial>\n`;
}

/** Shift helper for tests / callers that place a solid elsewhere. */
export function momentsOfBox(size: Vec3, T: Mat4 | number[]): Moments { return boxMoments(size, Array.from(T)); }
