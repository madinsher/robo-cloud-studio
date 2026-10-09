/**
 * glTF / GLB mesh import (Blender, CAD exporters, Sketchfab…): all meshes of the file are flattened
 * into one MeshData in scene units (mm). Uses three's GLTFLoader, so Draco/KTX-free files only.
 */
import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { MeshData } from './stl';

export interface GltfImportOptions {
  /** Multiply coordinates by this factor (glTF is metres; the studio is mm). Default 1000. */
  scale?: number;
  /** Convert glTF Y-up to Z-up (default true). */
  yUpToZUp?: boolean;
}

export async function parseGLTF(data: ArrayBuffer | string, opts: GltfImportOptions = {}): Promise<MeshData & { nodes: number }> {
  const loader = new GLTFLoader();
  const gltf = await new Promise<any>((resolve, reject) => loader.parse(data, '', resolve, reject));
  const scale = opts.scale ?? 1000;
  const root: THREE.Object3D = gltf.scene;
  // files exported by the studio itself are already Z-up (the root node carries the mm->m scale)
  let ownExport = false;
  root.traverse((o) => { if (o.name.startsWith('VerticalBotStudio')) ownExport = true; });
  const yToZ = opts.yUpToZUp ?? !ownExport;
  root.updateMatrixWorld(true);
  const pos: number[] = [], nor: number[] = [];
  let nodes = 0;
  const v = new THREE.Vector3(), n = new THREE.Vector3();
  const nm = new THREE.Matrix3();
  root.traverse((o) => {
    const m = o as THREE.Mesh;
    if (!m.isMesh || !m.geometry) return;
    nodes++;
    const g = (m.geometry.index ? m.geometry.toNonIndexed() : m.geometry) as THREE.BufferGeometry;
    const p = g.getAttribute('position');
    if (!g.getAttribute('normal')) g.computeVertexNormals();
    const na = g.getAttribute('normal');
    nm.getNormalMatrix(m.matrixWorld);
    for (let i = 0; i < p.count; i++) {
      v.fromBufferAttribute(p, i).applyMatrix4(m.matrixWorld).multiplyScalar(scale);
      n.fromBufferAttribute(na, i).applyMatrix3(nm).normalize();
      if (yToZ) { pos.push(v.x, -v.z, v.y); nor.push(n.x, -n.z, n.y); } else { pos.push(v.x, v.y, v.z); nor.push(n.x, n.y, n.z); }
    }
  });
  const positions = new Float32Array(pos), normals = new Float32Array(nor);
  const min: [number, number, number] = [Infinity, Infinity, Infinity], max: [number, number, number] = [-Infinity, -Infinity, -Infinity];
  for (let i = 0; i < positions.length; i += 3) for (let k = 0; k < 3; k++) { min[k] = Math.min(min[k], positions[i + k]); max[k] = Math.max(max[k], positions[i + k]); }
  if (!positions.length) { min.fill(0); max.fill(0); }
  return { positions, normals, min, max, triangles: positions.length / 9, nodes };
}
