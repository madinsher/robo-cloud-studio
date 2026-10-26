/**
 * Linear sum assignment — a line-by-line port of SciPy's `scipy.optimize.linear_sum_assignment`
 * (scipy/optimize/rectangular_lsap/rectangular_lsap.cpp: shortest augmenting path, Crouse 2016, with SciPy's tie rules:
 * columns scanned in reverse order, a tie on the lowest reduced cost prefers an unassigned column). Same arithmetic in
 * the same order, so it returns the same assignment as SciPy also when several optima exist — the MURKA experiments and
 * the S1 dispatcher (Hungarian baseline) depend on it.
 */

export interface LsapResult { rows: number[]; cols: number[] }

function augmentingPath(nc: number, cost: Float64Array, u: Float64Array, v: Float64Array, path: Int32Array, row4col: Int32Array,
  spc: Float64Array, i0: number, SR: Uint8Array, SC: Uint8Array, remaining: Int32Array): { sink: number; minVal: number } {
  let minVal = 0;
  let numRemaining = nc;
  for (let it = 0; it < nc; it++) remaining[it] = nc - it - 1;
  SR.fill(0); SC.fill(0); spc.fill(Infinity);
  let sink = -1, i = i0;
  while (sink === -1) {
    let index = -1, lowest = Infinity;
    SR[i] = 1;
    for (let it = 0; it < numRemaining; it++) {
      const j = remaining[it];
      const r = minVal + cost[i * nc + j] - u[i] - v[j];
      if (r < spc[j]) { path[j] = i; spc[j] = r; }
      if (spc[j] < lowest || (spc[j] === lowest && row4col[j] === -1)) { lowest = spc[j]; index = it; }
    }
    minVal = lowest;
    if (minVal === Infinity) return { sink: -1, minVal };
    const j = remaining[index];
    if (row4col[j] === -1) sink = j; else i = row4col[j];
    SC[j] = 1;
    remaining[index] = remaining[--numRemaining];
  }
  return { sink, minVal };
}

/** `linear_sum_assignment(cost, maximize=False)` for a dense matrix (rows × cols). Throws when infeasible / NaN. */
export function linearSumAssignment(matrix: readonly (readonly number[])[], maximize = false): LsapResult {
  let nr = matrix.length, nc = nr ? matrix[0].length : 0;
  if (nr === 0 || nc === 0) return { rows: [], cols: [] };
  const transpose = nc < nr;
  let cost = new Float64Array(nr * nc);
  for (let i = 0; i < nr; i++) for (let j = 0; j < nc; j++) cost[i * nc + j] = matrix[i][j];
  if (transpose) {
    const t = new Float64Array(nr * nc);
    for (let i = 0; i < nr; i++) for (let j = 0; j < nc; j++) t[j * nr + i] = cost[i * nc + j];
    cost = t; [nr, nc] = [nc, nr];
  }
  if (maximize) for (let k = 0; k < cost.length; k++) cost[k] = -cost[k];
  for (let k = 0; k < cost.length; k++) if (Number.isNaN(cost[k]) || cost[k] === -Infinity) throw new Error('matrix contains invalid numeric entries');
  const u = new Float64Array(nr), v = new Float64Array(nc), spc = new Float64Array(nc);
  const path = new Int32Array(nc).fill(-1), col4row = new Int32Array(nr).fill(-1), row4col = new Int32Array(nc).fill(-1);
  const SR = new Uint8Array(nr), SC = new Uint8Array(nc), remaining = new Int32Array(nc);
  for (let curRow = 0; curRow < nr; curRow++) {
    const { sink, minVal } = augmentingPath(nc, cost, u, v, path, row4col, spc, curRow, SR, SC, remaining);
    if (sink < 0) throw new Error('cost matrix is infeasible');
    u[curRow] += minVal;
    for (let i = 0; i < nr; i++) if (SR[i] && i !== curRow) u[i] += minVal - spc[col4row[i]];
    for (let j = 0; j < nc; j++) if (SC[j]) v[j] -= minVal - spc[j];
    let j = sink;
    for (;;) {
      const i = path[j];
      row4col[j] = i;
      const tmp = col4row[i]; col4row[i] = j; j = tmp;
      if (i === curRow) break;
    }
  }
  if (transpose) {
    const order = [...col4row.keys()].sort((a, b) => col4row[a] - col4row[b]);
    return { rows: order.map((k) => col4row[k]), cols: order };
  }
  return { rows: [...Array(nr).keys()], cols: [...col4row] };
}
