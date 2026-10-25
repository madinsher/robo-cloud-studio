/**
 * Supervisor of a DES document as deployed: Ramadge–Wonham synthesis (supcon) followed by the observer-based table
 * when events are unobservable (des.ts `observedSupervisor`). The executors, the BT mission runtime, the Python export
 * and the code generators all use this one function, so what the analysis reports is what runs.
 */
import { DES, parallel, supcon, observedSupervisor, ObservedSupervisor, SupervisorTable } from './des';
import { parseDes, parsePetri, DesDoc } from './dsl';
import { PetriNet, preventDeadlocks } from './petri';
import { petriToDsl } from './graphdoc';

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

/**
 * Apply the GMEC deadlock-prevention monitors of the analysis (petri.ts `preventDeadlocks`, Theorem 4.9) to a Petri
 * document: the monitor places and their arcs are added, everything else (bindings, timing, finals, layout) is kept.
 */
export function applyDeadlockMonitors(source: string): { source: string; monitors: Array<{ id: string; constraint: string; siphon: string[] }>; live: boolean } {
  const doc = parsePetri(source); const r = preventDeadlocks(new PetriNet(doc.spec));
  if (!r.monitors.length) return { source, monitors: [], live: r.live };
  const spec = r.net.toSpec();
  // keep the document's own transitions (timing, guards, servers) and only add the monitor places / arcs
  spec.transitions = doc.spec.transitions;
  return { source: petriToDsl({ ...doc, spec }), monitors: r.monitors, live: r.live };
}
