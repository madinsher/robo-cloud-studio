import * as THREE from 'three';
import { MeshData } from '../io/mesh/stl';
import { GeometryRef } from '../core/items/item';
import { LinkVisual } from '../core/kinematics/chain';
import { AssetStore } from './assets';
import { Mat4 } from '../core/math/pose';

/** Faces meeting more steeply than this keep a hard edge. Shallower joins (cylinder sides) share a normal. */
const CREASE_DEG = 50;

function authoredSmoothNormals(m: MeshData): boolean {
  if (m.normals.length !== m.positions.length || m.positions.length < 9) return false;
  const tris = m.positions.length / 9;
  const sample = Math.min(tris, 64);
  for (let t = 0; t < sample; t++) {
    const o = t * 9;
    const ax = m.normals[o], ay = m.normals[o + 1], az = m.normals[o + 2];
    if (ax === 0 && ay === 0 && az === 0) return false;
    for (let k = 1; k < 3; k++) {
      const p = o + k * 3;
      if (Math.abs(m.normals[p] - ax) > 1e-4 || Math.abs(m.normals[p + 1] - ay) > 1e-4 || Math.abs(m.normals[p + 2] - az) > 1e-4) return true;
    }
  }
  return false;
}

/**
 * Triangle soups store three private corners per face, so a plain normal pass leaves every face flat.
 * Weld corners that share a position, then average normals inside a crease angle.
 */
export function smoothTriangleSoup(positions: Float32Array, creaseDeg = CREASE_DEG): THREE.BufferGeometry {
  const corners = positions.length / 3;
  const tris = corners / 3;
  const faceN = new Float32Array(tris * 3);
  const faceA = new Float32Array(tris);
  for (let t = 0; t < tris; t++) {
    const o = t * 9;
    const ax = positions[o], ay = positions[o + 1], az = positions[o + 2];
    const bx = positions[o + 3] - ax, by = positions[o + 4] - ay, bz = positions[o + 5] - az;
    const cx = positions[o + 6] - ax, cy = positions[o + 7] - ay, cz = positions[o + 8] - az;
    const nx = by * cz - bz * cy, ny = bz * cx - bx * cz, nz = bx * cy - by * cx;
    const area = Math.hypot(nx, ny, nz);
    faceA[t] = area;
    if (area > 1e-12) { faceN[t * 3] = nx / area; faceN[t * 3 + 1] = ny / area; faceN[t * 3 + 2] = nz / area; }
  }
  const groups = new Map<string, number[]>();
  for (let i = 0; i < corners; i++) {
    const key = `${Math.round(positions[i * 3] * 20)},${Math.round(positions[i * 3 + 1] * 20)},${Math.round(positions[i * 3 + 2] * 20)}`;
    const list = groups.get(key);
    if (list) list.push(i);
    else groups.set(key, [i]);
  }
  const cos = Math.cos(creaseDeg * Math.PI / 180);
  const outPos: number[] = [];
  const outNor: number[] = [];
  const index = new Uint32Array(corners);
  for (const group of groups.values()) {
    const clusters: { rx: number; ry: number; rz: number; nx: number; ny: number; nz: number; oi: number }[] = [];
    for (const vi of group) {
      const t = Math.floor(vi / 3);
      const nx = faceN[t * 3], ny = faceN[t * 3 + 1], nz = faceN[t * 3 + 2];
      const w = faceA[t] || 1;
      let placed = false;
      if (faceA[t] > 1e-12) {
        for (const c of clusters) {
          if (nx * c.rx + ny * c.ry + nz * c.rz >= cos) {
            c.nx += nx * w; c.ny += ny * w; c.nz += nz * w;
            index[vi] = c.oi;
            placed = true;
            break;
          }
        }
      } else if (clusters.length) {
        index[vi] = clusters[0].oi;
        placed = true;
      }
      if (!placed) {
        const oi = outPos.length / 3;
        outPos.push(positions[vi * 3], positions[vi * 3 + 1], positions[vi * 3 + 2]);
        outNor.push(nx, ny, nz);
        clusters.push({ rx: nx, ry: ny, rz: nz, nx: nx * w, ny: ny * w, nz: nz * w, oi });
        index[vi] = oi;
      }
    }
    for (const c of clusters) {
      const len = Math.hypot(c.nx, c.ny, c.nz) || 1;
      outNor[c.oi * 3] = c.nx / len;
      outNor[c.oi * 3 + 1] = c.ny / len;
      outNor[c.oi * 3 + 2] = c.nz / len;
    }
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.BufferAttribute(new Float32Array(outPos), 3));
  g.setAttribute('normal', new THREE.BufferAttribute(new Float32Array(outNor), 3));
  g.setIndex(new THREE.BufferAttribute(index, 1));
  g.computeBoundingBox();
  g.computeBoundingSphere();
  return g;
}

export function meshDataToGeometry(m: MeshData): THREE.BufferGeometry {
  if (!authoredSmoothNormals(m)) return smoothTriangleSoup(m.positions);
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.BufferAttribute(m.positions, 3));
  g.setAttribute('normal', new THREE.BufferAttribute(m.normals, 3));
  g.computeBoundingBox();
  g.computeBoundingSphere();
  return g;
}

export function mat4ToThree(m: Mat4 | number[] | undefined): THREE.Matrix4 {
  const t = new THREE.Matrix4();
  if (!m) return t;
  t.fromArray(Array.from(m));
  return t;
}

const geomCache = new Map<string, THREE.BufferGeometry>();

export function primitiveGeometry(p: NonNullable<GeometryRef['primitive']> | NonNullable<LinkVisual['primitive']>): THREE.BufferGeometry {
  const key = JSON.stringify(p);
  let g = geomCache.get(key);
  if (g) return g;
  switch (p.kind) {
    case 'box': g = new THREE.BoxGeometry(p.size[0], p.size[1], p.size[2]); break;
    case 'cylinder': g = new THREE.CylinderGeometry(p.radius, p.radius, p.length, 24).rotateX(Math.PI / 2); break;
    case 'sphere': g = new THREE.SphereGeometry(p.radius, 20, 14); break;
    case 'plane': g = new THREE.PlaneGeometry((p as any).size[0], (p as any).size[1]); break;
    case 'cone': g = new THREE.ConeGeometry((p as any).radius, (p as any).length, 20).rotateX(Math.PI / 2); break;
    default: g = new THREE.BoxGeometry(100, 100, 100);
  }
  geomCache.set(key, g);
  return g;
}

export function materialFor(color: string | undefined, opacity = 1, fallback = '#9aa3ad'): THREE.MeshStandardMaterial {
  const m = new THREE.MeshStandardMaterial({ color: new THREE.Color(color ?? fallback), metalness: 0.22, roughness: 0.42, flatShading: false });
  if (opacity < 1) { m.transparent = true; m.opacity = opacity; }
  return m;
}

/** Build a three Object3D for a list of GeometryRefs (objects, tools, components). */
export function buildGeometry(refs: GeometryRef[], assets: AssetStore, defaultColor?: string): THREE.Group {
  const group = new THREE.Group();
  for (const r of refs) {
    let geometry: THREE.BufferGeometry | null = null;
    if (r.mesh) {
      const a = assets.get(r.mesh) ?? findByBasename(assets, r.mesh);
      if (a?.mesh) {
        if (!a.gpu) a.gpu = meshDataToGeometry(a.mesh);
        geometry = a.gpu as THREE.BufferGeometry;
      } else {
        // placeholder for a missing mesh
        const ph = new THREE.Mesh(new THREE.BoxGeometry(100, 100, 100), new THREE.MeshStandardMaterial({ color: 0xff00ff, wireframe: true }));
        ph.matrixAutoUpdate = false;
        ph.matrix.copy(mat4ToThree(r.origin));
        ph.userData.missingMesh = r.mesh;
        group.add(ph);
        continue;
      }
    } else if (r.primitive) geometry = primitiveGeometry(r.primitive);
    if (!geometry) continue;
    const mesh = new THREE.Mesh(geometry, materialFor(r.color ?? defaultColor, r.opacity ?? 1));
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    mesh.matrixAutoUpdate = false;
    const m = mat4ToThree(r.origin);
    if (r.scale) m.multiply(new THREE.Matrix4().makeScale(r.scale[0], r.scale[1], r.scale[2]));
    mesh.matrix.copy(m);
    group.add(mesh);
  }
  return group;
}

export function buildLinkVisuals(visuals: LinkVisual[], assets: AssetStore, defaultColor: string): THREE.Group {
  return buildGeometry(visuals.map((v) => ({ mesh: v.mesh, primitive: v.primitive as any, origin: Array.from(v.origin), color: v.color, scale: v.scale })), assets, defaultColor);
}

export function findByBasename(assets: AssetStore, uri: string) {
  const base = uri.split(/[\\/]/).pop()?.toLowerCase();
  if (!base) return undefined;
  for (const [id, a] of assets.entries()) {
    if (id.split(/[\\/]/).pop()?.toLowerCase() === base || a.name?.toLowerCase() === base) return a;
  }
  return undefined;
}

/** Axis triad (X red, Y green, Z blue). */
export function makeTriad(size = 100, lineWidth = 1): THREE.Group {
  const g = new THREE.Group();
  const axes: Array<[number, number, number, number]> = [[1, 0, 0, 0xff4d4d], [0, 1, 0, 0x4dff4d], [0, 0, 1, 0x4d8dff]];
  for (const [x, y, z, c] of axes) {
    const geo = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, 0, 0), new THREE.Vector3(x * size, y * size, z * size)]);
    const line = new THREE.Line(geo, new THREE.LineBasicMaterial({ color: c, linewidth: lineWidth, depthTest: false }));
    line.renderOrder = 10;
    g.add(line);
  }
  return g;
}

/** Small target glyph: triad + point. */
export function makeTargetGlyph(size = 60, color = 0xffaa00): THREE.Group {
  const g = makeTriad(size);
  const s = new THREE.Mesh(new THREE.SphereGeometry(size * 0.12, 10, 8), new THREE.MeshBasicMaterial({ color }));
  g.add(s);
  return g;
}

export function polylineObject(points: number[][], color = 0x33aaff, closed = false, z = 0): THREE.Line {
  const pts = points.map((p) => new THREE.Vector3(p[0], p[1], (p[2] ?? 0) + z));
  if (closed && pts.length) pts.push(pts[0].clone());
  const geo = new THREE.BufferGeometry().setFromPoints(pts);
  return new THREE.Line(geo, new THREE.LineBasicMaterial({ color }));
}

export function polygonMesh(points: number[][], color = 0x2f9e44, opacity = 0.25, z = 1): THREE.Mesh {
  const shape = new THREE.Shape(points.map((p) => new THREE.Vector2(p[0], p[1])));
  const geo = new THREE.ShapeGeometry(shape);
  const mesh = new THREE.Mesh(geo, new THREE.MeshBasicMaterial({ color, transparent: true, opacity, side: THREE.DoubleSide, depthWrite: false }));
  mesh.position.z = z;
  return mesh;
}
