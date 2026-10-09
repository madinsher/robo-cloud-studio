/**
 * Animated glTF export: the whole scene plus the motion of a program simulation as glTF 2.0
 * animation tracks. Blender (File › Import › glTF 2.0), Unity, Unreal, three.js and Omniverse import
 * the result with keyframes on every robot link, tool and moved object. Units are converted to metres
 * through a root node scaled by 0.001.
 */
import * as THREE from 'three';
import type { SceneRenderer } from '../../scene/renderer';
import type { ProgramSimulator } from '../../core/motion/simulator';
import { Robot } from '../../core/items/robot';

export interface GltfAnimOptions {
  /** Keyframe rate (default 30). */
  fps?: number;
  /** Clip name (default: program name). */
  name?: string;
  /** Hide reference frames / targets helpers in the export (default true). */
  hideHelpers?: boolean;
  /** Scale factor applied at the root (default 0.001 = mm -> m). */
  scale?: number;
  onProgress?: (done: number, total: number) => void;
}

interface Tracked { obj: THREE.Object3D; times: number[]; pos: number[]; quat: number[]; scl: number[]; wasAuto: boolean; }

/** Samples the simulator from 0 to its duration and returns a GLB (ArrayBuffer) with one animation clip. */
export async function exportAnimatedGLB(renderer: SceneRenderer, sim: ProgramSimulator, opts: GltfAnimOptions = {}): Promise<{ glb: ArrayBuffer; frames: number; duration: number; tracks: number }> {
  const { GLTFExporter } = await import('three/addons/exporters/GLTFExporter.js');
  const fps = opts.fps ?? 30;
  const duration = Math.max(0, sim.duration);
  const frames = Math.max(2, Math.ceil(duration * fps) + 1);
  const dt = frames > 1 ? duration / (frames - 1) : 0;

  // objects to track: item roots (poses can change during simulation: attachments, mobile robots) + robot links/flanges
  const tracked: Tracked[] = [];
  const track = (obj: THREE.Object3D) => tracked.push({ obj, times: [], pos: [], quat: [], scl: [], wasAuto: obj.matrixAutoUpdate });
  for (const t of renderer.animationTargets()) {
    track(t.root);
    if (t.item instanceof Robot) { for (const l of t.links ?? []) track(l); if (t.flangeGroup) track(t.flangeGroup); }
  }
  const savedTime = sim.time;
  const p = new THREE.Vector3(), q = new THREE.Quaternion(), s = new THREE.Vector3();
  for (let f = 0; f < frames; f++) {
    const t = f * dt;
    sim.seek(t);
    renderer.syncNow();
    for (const tr of tracked) {
      tr.obj.matrix.decompose(p, q, s);
      tr.times.push(t);
      tr.pos.push(p.x, p.y, p.z);
      tr.quat.push(q.x, q.y, q.z, q.w);
      tr.scl.push(s.x, s.y, s.z);
    }
    opts.onProgress?.(f + 1, frames);
  }
  sim.seek(savedTime);
  renderer.syncNow();

  // keep only nodes that actually move; glTF animated nodes must use TRS (not matrix)
  const tracks: THREE.KeyframeTrack[] = [];
  const restore: Array<() => void> = [];
  const moving = (a: number[], stride: number) => { for (let i = stride; i < a.length; i++) if (Math.abs(a[i] - a[i % stride]) > 1e-6) return true; return false; };
  for (const tr of tracked) {
    const o = tr.obj;
    if (!o.name) o.name = `node_${o.id}`;
    const movesP = moving(tr.pos, 3), movesQ = moving(tr.quat, 4);
    if (!movesP && !movesQ) continue;
    const auto = o.matrixAutoUpdate;
    o.matrix.decompose(o.position, o.quaternion, o.scale);
    o.matrixAutoUpdate = true;
    restore.push(() => { o.matrixAutoUpdate = auto; o.matrix.compose(o.position, o.quaternion, o.scale); });
    const times = new Float32Array(tr.times);
    if (movesP) tracks.push(new THREE.VectorKeyframeTrack(`${o.uuid}.position`, times, new Float32Array(tr.pos)));
    if (movesQ) tracks.push(new THREE.QuaternionKeyframeTrack(`${o.uuid}.quaternion`, times, new Float32Array(tr.quat)));
  }
  const clip = new THREE.AnimationClip(opts.name ?? 'Simulation', duration, tracks);

  // wrap the scene in a metre-scaled root; hide helpers
  const scene = renderer.scene;
  const wrap = new THREE.Group();
  wrap.name = 'VerticalBotStudio_mm_to_m';
  wrap.scale.setScalar(opts.scale ?? 0.001);
  const children = [...scene.children];
  for (const c of children) wrap.add(c);
  scene.add(wrap);
  const hidden: THREE.Object3D[] = [];
  // helpers, grid/ground and lights-only nodes are not part of the exported model
  scene.traverse((o) => {
    const helper = /Helper$/.test(o.type) || o.userData.isGround || o.name === 'ground' || o.name === 'grid';
    const frames = opts.hideHelpers !== false && (o.userData.isFrame || o.userData.isTarget || o.name === 'path');
    if ((helper || frames) && o.visible) { o.visible = false; hidden.push(o); }
  });
  const gizmo = renderer.transform.getHelper();
  const gizmoVisible = gizmo.visible;
  gizmo.visible = false;
  try {
    const exporter = new GLTFExporter();
    const glb = await new Promise<ArrayBuffer>((resolve, reject) => exporter.parse(wrap, (r) => resolve(r as ArrayBuffer), (e) => reject(e), { binary: true, onlyVisible: true, animations: tracks.length ? [clip] : [] }));
    return { glb, frames, duration, tracks: tracks.length };
  } finally {
    gizmo.visible = gizmoVisible;
    for (const o of hidden) o.visible = true;
    for (const c of children) scene.add(c);
    scene.remove(wrap);
    for (const r of restore) r();
  }
}
