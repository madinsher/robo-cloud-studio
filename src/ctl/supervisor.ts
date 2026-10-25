/**
 * Supervisor of a DES document as deployed: Ramadge–Wonham synthesis (supcon) followed by the observer-based table
 * when events are unobservable (des.ts `observedSupervisor`). The executors, the BT mission runtime, the Python export
 * and the code generators all use this one function, so what the analysis reports is what runs.
 */
import { DES, parallel, supcon, observedSupervisor, ObservedSupervisor, SupervisorTable } from './des';
import { parseDes, DesDoc } from './dsl';

export interface DeployedSupervisor { table: SupervisorTable; observed: ObservedSupervisor; implementable: boolean; states: number; doc: DesDoc }

/** null when there are no specifications or the specification is unrealisable. */
export function synthesizeSupervisor(source: string | DesDoc): DeployedSupervisor | null {
  const doc = typeof source === 'string' ? parseDes(source) : source;
  if (!doc.plant.length || !doc.specs.length) return null;
  const comps = doc.plant.map((a) => DES.fromSpec(a));
  const g = comps.length === 1 ? comps[0] : parallel(...comps);
  if (comps.length === 1) { g.components = [g.name]; for (const x of g.X) g.parts.set(x, [x]); }
  const r = supcon(g, doc.specs.map((a) => DES.fromSpec(a)), doc.uncontrollable);
  if (!r.realizable) return null;
  const observed = observedSupervisor(g, r.supervisor, doc.uncontrollable, doc.unobservable, doc.plant.length);
  return { table: observed.table, observed, implementable: observed.nonblocking, states: r.states.size, doc };
}
