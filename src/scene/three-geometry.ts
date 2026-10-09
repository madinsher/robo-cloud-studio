import * as THREE from 'three';
import { MeshData } from '../io/mesh/stl';
import { GeometryRef } from '../core/items/item';
import { LinkVisual } from '../core/kinematics/chain';
import { AssetStore } from './assets';
import { Mat4 } from '../core/math/pose';

export function meshDataToGeometry(m: MeshData): THREE.BufferGeometry {
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.BufferAttribute(m.positions, 3));
  if (m.normals.length === m.positions.length && m.normals.some((v) => v !== 0)) g.setAttribute('normal', new THREE.BufferAttribute(m.normals, 3));
  else g.computeVertexNormals();
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
  const m = new THREE.MeshStandardMaterial({ color: new THREE.Color(color ?? fallback), metalness: 0.2, roughness: 0.6 });
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
