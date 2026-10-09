/** Iterative Tarjan SCC (no recursion: reachability graphs and supervisors can be tens of thousands of states deep). */
export function stronglyConnected<T>(nodes: Iterable<T>, succ: (v: T) => Iterable<T>): T[][] {
  let index = 0; const idx = new Map<T, number>(), low = new Map<T, number>(); const stack: T[] = []; const on = new Set<T>(); const out: T[][] = [];
  for (const root of nodes) {
    if (idx.has(root)) continue;
    const work: Array<{ v: T; it: Iterator<T> }> = [{ v: root, it: succ(root)[Symbol.iterator]() }];
    idx.set(root, index); low.set(root, index); index++; stack.push(root); on.add(root);
    while (work.length) {
      const top = work[work.length - 1]; const nx = top.it.next();
      if (!nx.done) {
        const w = nx.value;
        if (!idx.has(w)) { idx.set(w, index); low.set(w, index); index++; stack.push(w); on.add(w); work.push({ v: w, it: succ(w)[Symbol.iterator]() }); }
        else if (on.has(w)) low.set(top.v, Math.min(low.get(top.v)!, idx.get(w)!));
        continue;
      }
      work.pop();
      if (work.length) { const p = work[work.length - 1].v; low.set(p, Math.min(low.get(p)!, low.get(top.v)!)); }
      if (low.get(top.v) === idx.get(top.v)) { const comp: T[] = []; let w: T; do { w = stack.pop()!; on.delete(w); comp.push(w); } while (w !== top.v); out.push(comp); }
    }
  }
  return out;
}
