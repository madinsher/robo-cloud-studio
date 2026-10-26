/**
 * State mirror between a simulator backend that runs on a copy of the station (the browser runs the MURKA backend in
 * a Web Worker, ./sim_worker.ts) and the studio's own station, which the 3D view draws. A simulator interface moves
 * three kinds of state: mobile bases (2D state), robot joints and item poses (carried / placed products). The backend
 * side (`StateMirror`) sends what changed since the previous snapshot; the studio side (`applySnapshot`) applies it to
 * the items with the same ids (a serialised station keeps its item ids).
 */
import { Item, Station, type SerializedItem } from '../core/items/item';
import { Robot } from '../core/items/robot';
import { MobileRobot } from '../mobile/items';
import type { Mat4 } from '../core/math/pose';
import type { AssetStore } from '../scene/assets';
import type { MeshData } from '../io/mesh/stl';
import { referencedMeshIds } from '../io/station-file';

/** A station as a worker gets it (structured clone): the item tree and the parsed meshes it references. */
export interface StationTransfer { tree: SerializedItem; meshes: Array<{ id: string; mesh: MeshData; name?: string; units?: 'mm' | 'm' }> }

/** The station and the meshes it references (by the ids the tree uses), for postMessage to a worker. */
export function stationTransfer(station: Station, assets?: AssetStore | null): StationTransfer {
  const tree = station.serialize();
  const meshes: StationTransfer['meshes'] = [];
  if (assets) for (const ref of referencedMeshIds(tree)) { const a = assets.resolve(ref); if (a?.mesh) meshes.push({ id: ref, mesh: a.mesh, name: a.name, units: a.units }); }
  return { tree, meshes };
}

/** Base state: [x mm, y mm, theta deg, v mm/s, omega deg/s, odometer mm]. */
export type MirrorBase = [number, number, number, number, number, number];

export interface MirrorSnapshot {
  /** Sim time (s). */
  t: number;
  bases: Array<[string, MirrorBase]>;
  /** Robot joints (studio units: deg / mm). */
  joints: Array<[string, number[]]>;
  /** Local poses (column-major 4x4) of other items. */
  poses: Array<[string, number[]]>;
}

const baseOf = (m: MobileRobot): MirrorBase => [m.state.x, m.state.y, m.state.theta, m.state.v, m.state.omega, m.state.odometer ?? 0];
const same = (a: ArrayLike<number> | undefined, b: ArrayLike<number>) => {
  if (!a || a.length !== b.length) return false;
  for (let i = 0; i < b.length; i++) if (a[i] !== b[i]) return false;
  return true;
};

/** Backend side: diffs of the station state since the previous snapshot (the first one: since construction). */
export class StateMirror {
  private bases = new Map<string, MirrorBase>();
  private joints = new Map<string, number[]>();
  private poses = new Map<string, number[]>();
  private items: Item[];

  constructor(readonly station: Station) {
    this.items = [...station.walk()].filter((it) => it !== station);
    this.collect(() => {}, () => {}, () => {});
  }

  private collect(onBase: (id: string, b: MirrorBase) => void, onJoints: (id: string, q: number[]) => void, onPose: (id: string, p: number[]) => void): void {
    for (const it of this.items) {
      if (it instanceof MobileRobot) {
        const b = baseOf(it);
        if (!same(this.bases.get(it.id), b)) { this.bases.set(it.id, b); onBase(it.id, b); }
        continue; // its pose follows the state
      }
      if (it instanceof Robot) {
        const q = it.joints();
        if (!same(this.joints.get(it.id), q)) { this.joints.set(it.id, q); onJoints(it.id, q); }
      }
      const p = Array.from(it.pose());
      if (!same(this.poses.get(it.id), p)) { this.poses.set(it.id, p); onPose(it.id, p); }
    }
  }

  /** What changed since the last call. */
  snapshot(t: number): MirrorSnapshot {
    const s: MirrorSnapshot = { t, bases: [], joints: [], poses: [] };
    this.collect((id, b) => s.bases.push([id, b]), (id, q) => s.joints.push([id, q]), (id, p) => s.poses.push([id, p]));
    return s;
  }
}

/** Studio side: apply a snapshot to the station (items found by id; unknown ids are counted, not fatal). */
export function applySnapshot(station: Station, s: MirrorSnapshot): { applied: number; missing: number } {
  let applied = 0, missing = 0;
  for (const [id, b] of s.bases) {
    const m = station.findById(id);
    if (!(m instanceof MobileRobot)) { missing++; continue; }
    const st = m.state;
    st.x = b[0]; st.y = b[1]; st.theta = b[2]; st.v = b[3]; st.omega = b[4]; st.odometer = b[5];
    m.syncPoseFromState();
    applied++;
  }
  for (const [id, q] of s.joints) {
    const r = station.findById(id);
    if (!(r instanceof Robot)) { missing++; continue; }
    r.setJoints(q);
    applied++;
  }
  for (const [id, p] of s.poses) {
    const it = station.findById(id);
    if (!it) { missing++; continue; }
    it.setPose(Float64Array.from(p) as Mat4);
    applied++;
  }
  station.simTime = s.t;
  return { applied, missing };
}
