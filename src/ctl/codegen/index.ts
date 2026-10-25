/**
 * Code generators from control models (automata, Petri nets, FSMs, behavior trees, SMV modules): the entries of the
 * Control tab's Export menu. Each entry returns the files to download (several files → one zip in the UI).
 *  - robodk.ts     RoboDK station script (Python robolink; runs against RoboDK and the studio API server)
 *  - vendor.ts     per-robot programs with IO handshakes through the post processors (+ the robot I/O map)
 *  - plc.ts        IEC 61131-3 Structured Text (CASE state machines / Petri marking logic) and PLCopen XML SFC + I/O map
 *  - ros2.ts       ROS 2 packages: rclpy supervisor node, yasmin FSM, BehaviorTree.CPP v4 XML with skill stubs, launch
 *  - interchange.ts PNML, SCXML, NuSMV .smv, UPPAAL XML
 */
import type { Station } from '../../core/items/item';
import type { ControlKind } from '../model';
import { robodkStationScript } from './robodk';
import { vendorPrograms } from './vendor';
import { automatonToST, petriToST, petriToPlcOpen } from './plc';
import { ros2Packages, Ros2Options } from './ros2';
import { petriToPnml, fsmToScxml, desToScxml, desToSmv, petriToSmv, smvDocToNuSmv, desToUppaal } from './interchange';
import { ident } from './common';
import type { GenFile } from './common';

export type { GenFile } from './common';
export interface ExportEntry { id: string; label: string; files(): GenFile[] }
export { robodkStationScript, vendorPrograms, automatonToST, petriToST, petriToPlcOpen, ros2Packages, petriToPnml, fsmToScxml, desToScxml, desToSmv, petriToSmv, smvDocToNuSmv, desToUppaal };

/** Export entries available for a model kind (`station` enables the robot programs and the station's durations / robots for ROS). */
export function controlExports(kind: ControlKind, source: string, name: string, station?: Station, opts: { supervisorSource?: string; ros?: Ros2Options } = {}): ExportEntry[] {
  const out: ExportEntry[] = []; const n = ident(name).toLowerCase();
  const add = (id: string, label: string, files: () => GenFile[]) => out.push({ id, label, files });
  if (['des', 'petri', 'fsm', 'bt'].includes(kind)) {
    add('robodk', 'RoboDK station script (Python)', () => [robodkStationScript(kind, source, name, { supervisorSource: opts.supervisorSource })]);
    if (station) add('vendor', 'Robot programs with IO handshake (vendor posts)', () => vendorPrograms(station, kind, source, name));
    add('ros2', kind === 'fsm' ? 'ROS 2 package (rclpy node + yasmin FSM)' : kind === 'bt' ? 'ROS 2 packages (rclpy node + BehaviorTree.CPP v4)' : 'ROS 2 package (rclpy supervisor node)', () => ros2Packages(kind, source, name, { supervisorSource: opts.supervisorSource, ...(opts.ros ?? {}) }).map((f) => ({ ...f, name: `src/${f.name}` })));
  }
  if (kind === 'des') {
    add('st', 'IEC 61131-3 Structured Text + I/O map', () => automatonToST(source, name));
    add('scxml', 'SCXML state chart', () => [desToScxml(source, name)]);
    add('smv', 'NuSMV / nuXmv model (.smv)', () => [desToSmv(source, name)]);
    add('uppaal', 'UPPAAL timed automata (XML)', () => [desToUppaal(source, name)]);
  }
  if (kind === 'petri') {
    add('st', 'IEC 61131-3 Structured Text + I/O map', () => petriToST(source, name));
    add('sfc', 'PLCopen XML (SFC / Grafcet)', () => [petriToPlcOpen(source, name)]);
    add('pnml', 'PNML (ISO/IEC 15909-2)', () => [petriToPnml(source, name)]);
    add('smv', 'NuSMV / nuXmv model (.smv)', () => [petriToSmv(source, name)]);
  }
  if (kind === 'fsm') add('scxml', 'SCXML state chart', () => [fsmToScxml(source, name)]);
  if (kind === 'smv') add('smv', 'NuSMV / nuXmv model (.smv)', () => [smvDocToNuSmv(source, name)]);
  void n;
  return out;
}
