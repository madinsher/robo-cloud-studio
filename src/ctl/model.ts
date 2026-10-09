/**
 * Control-design model documents stored in the station tree.
 *
 * A `ControlModelItem` holds one text document in the control DSL (see dsl.ts): a discrete-event plant with
 * specifications, a Petri net, a behavior tree, a mode automaton, a GR(1) specification, a PDDL domain… The
 * analysis (analysis.ts) turns the document into a report; the runtime (runtime.ts) executes behavior trees,
 * supervisors, monitors and mode machines against the simulated station.
 */
import { Item, ItemType, Station, Folder, registerItemType, SerializedItem, DeserializeContext } from '../core/items/item';
import { MRS_KINDS, MrsKind } from '../mrs/model';

export type CtlKind = 'des' | 'petri' | 's3pr' | 'smv' | 'bt' | 'statechart' | 'gr1' | 'hybrid' | 'pddl' | 'stn' | 'mdp' | 'pomdp' | 'jobshop' | 'realtime' | 'fta' | 'reliability' | 'mrta' | 'mapf' | 'stl' | 'fmea' | 'acceptance' | 'perf';
/** All document kinds: the control-design kinds (course *control of robotic complexes*) and the group-control kinds of the multi-robot module (course *control of distributed robotic systems*, src/mrs). */
export type ControlKind = CtlKind | MrsKind;
export type ControlGroup = 'discrete' | 'executive' | 'verification' | 'planning' | 'coordination' | 'engineering' | 'multi-robot';

export const CONTROL_KINDS: Array<{ kind: ControlKind; label: string; chapter: string; group: ControlGroup; /** practicum of the group-control course (multi-robot kinds) */ practicum?: string }> = [
  { kind: 'des', label: 'Automata & supervisory control', chapter: '2–3', group: 'discrete' },
  { kind: 'petri', label: 'Petri net', chapter: '4–5', group: 'discrete' },
  { kind: 's3pr', label: 'Resource-allocation system (S³PR)', chapter: '4', group: 'discrete' },
  { kind: 'perf', label: 'Performance (bottleneck, cycle time, Little)', chapter: '5', group: 'discrete' },
  { kind: 'bt', label: 'Behavior tree (mission logic)', chapter: '6', group: 'executive' },
  { kind: 'statechart', label: 'Statechart (hierarchical FSM)', chapter: '6', group: 'executive' },
  { kind: 'hybrid', label: 'Mode automaton + CBF safety filter', chapter: '9', group: 'executive' },
  { kind: 'smv', label: 'Synchronous model + LTL/CTL specs', chapter: '7', group: 'verification' },
  { kind: 'gr1', label: 'GR(1) reactive synthesis', chapter: '8', group: 'verification' },
  { kind: 'stl', label: 'STL monitor / falsification', chapter: '9, 17', group: 'verification' },
  { kind: 'pddl', label: 'PDDL planning (+ HTN)', chapter: '10–11', group: 'planning' },
  { kind: 'stn', label: 'Temporal network (STN / STNU)', chapter: '10', group: 'planning' },
  { kind: 'mdp', label: 'MDP decision model', chapter: '12', group: 'planning' },
  { kind: 'pomdp', label: 'POMDP (partial observation)', chapter: '12', group: 'planning' },
  { kind: 'mrta', label: 'Task allocation (assignment / auction)', chapter: '13', group: 'coordination' },
  { kind: 'mapf', label: 'Multi-agent path finding', chapter: '13', group: 'coordination' },
  { kind: 'jobshop', label: 'Job-shop schedule', chapter: '14', group: 'coordination' },
  { kind: 'realtime', label: 'Real-time task set & latency', chapter: '15', group: 'engineering' },
  { kind: 'reliability', label: 'Reliability budget & safety', chapter: '16', group: 'engineering' },
  { kind: 'fta', label: 'Fault tree', chapter: '16', group: 'engineering' },
  { kind: 'fmea', label: 'FMEA', chapter: '16', group: 'engineering' },
  { kind: 'acceptance', label: 'Acceptance statistics & traceability', chapter: '17', group: 'engineering' },
  // group control (multi-robot systems) — chapters of the second course, see src/mrs
  ...MRS_KINDS.map((k) => ({ kind: k.kind as ControlKind, label: k.label, chapter: `MRS ${k.chapter}`, group: 'multi-robot' as ControlGroup, practicum: k.practicum })),
];
export const GROUP_LABELS: Record<ControlGroup, string> = { discrete: 'Discrete-event models', executive: 'Executive layer', verification: 'Verification & synthesis', planning: 'Planning & decisions', coordination: 'Coordination & scheduling', engineering: 'Engineering: real-time, reliability, V&V', 'multi-robot': 'Group control (multi-robot systems)' };

export class ControlModelItem extends Item {
  kind: ControlKind = 'des';
  source = '';
  /** Last analysis report (markdown) and its verdict. */
  lastReport: string | null = null;
  lastOk: boolean | null = null;
  /** Runtime bindings: which station items the model drives (ids). */
  robotIds: string[] = [];
  enabled = true;
  constructor(name = 'Control model', id?: string) { super(ItemType.CONTROL_MODEL, name, id); }
  protected override serializeExtra() { return { kind: this.kind, source: this.source, lastReport: this.lastReport, lastOk: this.lastOk, robotIds: this.robotIds, enabled: this.enabled }; }
  override deserializeExtra(d: SerializedItem, _ctx?: DeserializeContext) {
    const x = d as unknown as Record<string, unknown>;
    this.kind = (x.kind as ControlKind) ?? 'des'; this.source = (x.source as string) ?? ''; this.lastReport = (x.lastReport as string) ?? null; this.lastOk = (x.lastOk as boolean) ?? null; this.robotIds = (x.robotIds as string[]) ?? []; this.enabled = (x.enabled as boolean) ?? true;
  }
}
registerItemType(ItemType.CONTROL_MODEL, (n, id) => new ControlModelItem(n, id));

export function controlModels(station: Station): ControlModelItem[] { return station.itemsOfType<ControlModelItem>(ItemType.CONTROL_MODEL); }

/** Folder "Control design" under the station root (created on demand). */
export function controlFolder(station: Station): Folder {
  const existing = station.children.find((c) => c instanceof Folder && c.name === 'Control design') as Folder | undefined;
  if (existing) return existing;
  return station.addChild(new Folder('Control design'));
}

export function addControlModel(station: Station, kind: ControlKind, name: string, source: string): ControlModelItem {
  const m = new ControlModelItem(name); m.kind = kind; m.source = source;
  controlFolder(station).addChild(m);
  return m;
}
