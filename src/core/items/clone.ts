/**
 * Deep-copy items (Copy/Paste, duplicate). Serializes the subtree, assigns fresh ids and remaps every
 * reference to an old id (targets in instructions, active tool/frame, robot links, fleet members).
 */
import { Item, Station, SerializedItem, itemFactories, DeserializeContext } from './item';
import { uid } from '../units';

export function serializeSubtree(item: Item): SerializedItem {
  return item.serialize();
}

/** Materialise a serialized subtree under `parent` with new ids. Returns the new root item. */
export function instantiateSubtree(data: SerializedItem, parent: Item, station: Station): Item {
  const idMap = new Map<string, string>();
  const collect = (d: SerializedItem) => { idMap.set(d.id, uid('copy')); for (const c of d.children ?? []) collect(c); };
  collect(data);
  let text = JSON.stringify(data);
  // replace ids inside quotes only (ids are unique tokens, so a global replace is safe)
  for (const [oldId, newId] of idMap) text = text.split(`"${oldId}"`).join(`"${newId}"`);
  const copy = JSON.parse(text) as SerializedItem;
  const ctx: DeserializeContext = { station, deferred: [], byId: new Map() };
  const build = (d: SerializedItem, p: Item): Item => {
    const factory = itemFactories.get(d.type);
    const item = factory ? factory(d.name, d.id) : new Item(d.type, d.name, d.id);
    item.applyBase(d);
    p.children.push(item);
    item.parent = p;
    ctx.byId.set(item.id, item);
    item.deserializeExtra(d, ctx);
    for (const c of d.children ?? []) build(c, item);
    return item;
  };
  const root = build(copy, parent);
  for (const fn of ctx.deferred) fn();
  station.notifyTree('childAdded', { parent, child: root });
  return root;
}

export function cloneItem(item: Item, parent: Item = item.parent ?? item.station!, rename = true): Item {
  const st = item.station!;
  const copy = instantiateSubtree(serializeSubtree(item), parent, st);
  if (rename) copy.setName(uniqueName(st, item.name));
  return copy;
}

export function uniqueName(station: Station, base: string): string {
  const names = new Set([...station.walk()].map((i) => i.name));
  if (!names.has(base)) return base;
  const m = base.match(/^(.*?)(?: (\d+))?$/);
  const stem = m?.[1] ?? base;
  let n = m?.[2] ? +m[2] + 1 : 2;
  while (names.has(`${stem} ${n}`)) n++;
  return `${stem} ${n}`;
}

/** Simple clipboard shared by UI and API. */
export class Clipboard {
  private data: SerializedItem | null = null;
  copy(item: Item): void { this.data = serializeSubtree(item); }
  hasContent(): boolean { return !!this.data; }
  paste(parent: Item, station: Station): Item | null {
    if (!this.data) return null;
    const it = instantiateSubtree(this.data, parent, station);
    it.setName(uniqueName(station, it.name));
    return it;
  }
}

export const clipboard = new Clipboard();
