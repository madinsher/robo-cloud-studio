/**
 * Studio worker: heavy jobs off the UI thread. `importRdk` parses a RoboDK station and returns the serialized
 * item tree plus mesh data (typed arrays transferred, not copied); `compile` compiles a program of a serialized
 * station (cycle time, per-instruction step times, problems, a subsampled TCP path for the preview line).
 */
import { importRdkBestEffort } from '../io/robodk/rdk_container';
import { AssetStore } from './assets';
import { loadStation, StationFile } from '../io/station-file';
import { ProgramSimulator } from '../core/motion/simulator';
import { Program } from '../core/items/program';
import { Robot } from '../core/items/robot';
import { multiply } from '../core/math/pose';

const ctx = self as unknown as { postMessage(m: unknown, transfer?: Transferable[]): void; onmessage: ((e: MessageEvent) => void) | null };
const progress = (id: number, detail: string) => ctx.postMessage({ id, progress: { detail } });

ctx.onmessage = async (e: MessageEvent) => {
  const { id, kind } = e.data as { id: number; kind: string };
  try {
    if (kind === 'importRdk') {
      const { bytes, name } = e.data as { bytes: ArrayBuffer; name: string };
      const assets = new AssetStore();
      progress(id, 'parsing station');
      const { station, report } = await importRdkBestEffort(bytes, name, assets);
      progress(id, 'serialising items');
      const tree = station.serialize();
      let itemCount = 0; for (const _ of station.walk()) itemCount++;
      const meshes: unknown[] = [];
      const transfer = new Set<ArrayBuffer>();
      const grab = (v: unknown) => { if (ArrayBuffer.isView(v) && v.buffer instanceof ArrayBuffer) transfer.add(v.buffer); else if (v && typeof v === 'object') for (const x of Object.values(v)) grab(x); };
      for (const [aid, a] of assets.entries()) {
        if (!a.mesh) continue;
        meshes.push({ id: aid, name: a.name, units: a.units, type: a.type, mesh: a.mesh, source: a.source });
        grab(a.mesh); if (a.source) transfer.add(a.source.buffer as ArrayBuffer);
      }
      progress(id, `transferring ${meshes.length} meshes`);
      ctx.postMessage({ id, ok: true, result: { tree, report: JSON.parse(JSON.stringify(report)), meshes, itemCount } }, [...transfer]);
    } else if (kind === 'compile') {
      const { file, programId, collisions, selfOnly } = e.data as { file: StationFile; programId: string; collisions: boolean; selfOnly: boolean };
      const assets = new AssetStore();
      const st = loadStation(file, assets);
      const prog = st.findById(programId);
      if (!(prog instanceof Program)) throw new Error('program not found');
      const sim = new ProgramSimulator(st);
      sim.collisionOptions = { enabled: collisions, selfOnly, assets, sampleStep: 0.05 };
      progress(id, `compiling ${prog.instructions().length} instructions`);
      const result = sim.compile(prog);
      const steps = sim.steps.map((s) => [s.instruction.id, s.t0, s.t1, s.trajectory ? s.trajectory.duration : -1]);
      // TCP path, subsampled to at most 60k points
      const robot = prog.robot();
      let n = 0; for (const s of sim.steps) n += s.trajectory?.samples.length ?? 0;
      const stride = Math.max(1, Math.ceil(n / 60000));
      const pts: number[] = [];
      if (robot instanceof Robot) {
        const base = robot.poseAbs();
        let k = 0;
        for (const s of sim.steps) for (const smp of s.trajectory?.samples ?? []) { if (k++ % stride) continue; const m = multiply(base, smp.pose); pts.push(m[12], m[13], m[14]); }
      }
      const points = new Float32Array(pts);
      ctx.postMessage({ id, ok: true, result: { result, steps, points } }, [points.buffer]);
    } else throw new Error(`unknown job ${kind}`);
  } catch (err) {
    ctx.postMessage({ id, ok: false, error: (err as Error)?.message ?? String(err) });
  }
};
