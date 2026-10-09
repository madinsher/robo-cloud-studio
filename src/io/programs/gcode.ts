/**
 * G-code / NC import (RoboDK "robot machining from NC file"): G0/G1/G2/G3 with X/Y/Z (mm or inch),
 * feed (F), spindle/tool on-off (M3/M5, M7/M8/M9) into a polyline path with segment metadata.
 * Arcs (G2/G3) are tessellated. The result becomes object curves (rapid vs. cutting) that the
 * curve-follow generator turns into a robot program.
 */
export interface GcodeSegment {
  kind: 'rapid' | 'cut';
  points: number[][];
  /** Feed rate in mm/min (G-code F word). */
  feed?: number;
  spindle?: boolean;
  /** Filament / material is being extruded along this segment (E axis increasing, 3D printing). */
  extrude?: boolean;
  /** Active tool number (T word / M6). */
  tool?: number;
}

export interface GcodeResult {
  segments: GcodeSegment[];
  bounds: { min: number[]; max: number[] };
  length: number;
  units: 'mm' | 'inch';
}

export function parseGcode(text: string, opts: { arcStep?: number } = {}): GcodeResult {
  const arcStep = opts.arcStep ?? 2; // mm
  let units: 'mm' | 'inch' = 'mm';
  let absolute = true;
  let pos = [0, 0, 0];
  let feed: number | undefined;
  let spindle = false;
  let extrude = false;
  let tool: number | undefined;
  let e = 0, eAbsolute = true;
  let plane: 17 | 18 | 19 = 17;
  const segments: GcodeSegment[] = [];
  let cur: GcodeSegment | null = null;
  const min = [Infinity, Infinity, Infinity], max = [-Infinity, -Infinity, -Infinity];
  const push = (kind: 'rapid' | 'cut', p: number[]) => {
    if (!cur || cur.kind !== kind || cur.spindle !== spindle || cur.extrude !== extrude || cur.tool !== tool || (kind === 'cut' && feed !== undefined && cur.feed !== feed)) { cur = { kind, points: [pos.slice()], feed, spindle, extrude, tool }; segments.push(cur); }
    cur.points.push(p.slice());
    for (let i = 0; i < 3; i++) { min[i] = Math.min(min[i], p[i]); max[i] = Math.max(max[i], p[i]); }
  };
  for (let i = 0; i < 3; i++) { min[i] = Math.min(min[i], pos[i]); max[i] = Math.max(max[i], pos[i]); }
  let modal = 0;
  for (const raw of text.split(/\r?\n/)) {
    let line = raw.replace(/\(.*?\)/g, '').replace(/;.*$/, '').trim().toUpperCase();
    if (!line || line.startsWith('%') || line.startsWith('O')) continue;
    line = line.replace(/^N\d+\s*/, '');
    const words = [...line.matchAll(/([A-Z])\s*([-+]?\d*\.?\d+)/g)].map((m) => [m[1], parseFloat(m[2])] as [string, number]);
    if (!words.length) continue;
    const w: Record<string, number> = {};
    const gs: number[] = [], ms: number[] = [];
    for (const [k, v] of words) { if (k === 'G') gs.push(v); else if (k === 'M') ms.push(v); else w[k] = v; }
    for (const g of gs) {
      if (g === 20) units = 'inch'; else if (g === 21) units = 'mm';
      else if (g === 90) absolute = true; else if (g === 91) absolute = false;
      else if (g === 17 || g === 18 || g === 19) plane = g as 17 | 18 | 19;
      else if (g === 92 && w.E !== undefined) e = w.E; // reset extruder position
      else if (g === 28) { /* homing: ignore */ }
      else if (g === 0 || g === 1 || g === 2 || g === 3) modal = g;
    }
    for (const m of ms) { if (m === 3 || m === 4) spindle = true; else if (m === 5) spindle = false; else if (m === 82) eAbsolute = true; else if (m === 83) eAbsolute = false; }
    if (w.T !== undefined) tool = w.T;
    if (w.F !== undefined) feed = w.F;
    // extrusion (3D printing): a move with increasing E extrudes; retractions (E decreasing without XYZ) do not create moves
    if (w.E !== undefined) {
      const eNew = eAbsolute ? w.E : e + w.E;
      extrude = eNew > e + 1e-9;
      e = eNew;
    } else extrude = false;
    const has = w.X !== undefined || w.Y !== undefined || w.Z !== undefined;
    if (!has) continue;
    const scale = units === 'inch' ? 25.4 : 1;
    const target = pos.slice();
    (['X', 'Y', 'Z'] as const).forEach((k, i) => { if (w[k] !== undefined) target[i] = absolute ? w[k] * scale : pos[i] + w[k] * scale; });
    if (modal === 2 || modal === 3) {
      // arc in the active plane with I/J/K centre offsets (or R)
      const [a, b] = plane === 17 ? [0, 1] : plane === 18 ? [2, 0] : [1, 2];
      const cx = pos[a] + ((plane === 17 ? w.I : plane === 18 ? w.K : w.J) ?? 0) * scale;
      const cy = pos[b] + ((plane === 17 ? w.J : plane === 18 ? w.I : w.K) ?? 0) * scale;
      let r = Math.hypot(pos[a] - cx, pos[b] - cy);
      if (w.R !== undefined) r = Math.abs(w.R) * scale;
      const a0 = Math.atan2(pos[b] - cy, pos[a] - cx);
      let a1 = Math.atan2(target[b] - cy, target[a] - cx);
      if (modal === 2) { if (a1 >= a0 - 1e-9) a1 -= 2 * Math.PI; } else { if (a1 <= a0 + 1e-9) a1 += 2 * Math.PI; }
      const n = Math.max(2, Math.ceil((Math.abs(a1 - a0) * r) / arcStep));
      const other = 3 - a - b;
      for (let k = 1; k <= n; k++) {
        const t = k / n;
        const ang = a0 + (a1 - a0) * t;
        const p = [0, 0, 0];
        p[a] = cx + r * Math.cos(ang);
        p[b] = cy + r * Math.sin(ang);
        p[other] = pos[other] + (target[other] - pos[other]) * t;
        push('cut', p);
        pos = p;
      }
      pos = target;
      continue;
    }
    push(modal === 0 ? 'rapid' : 'cut', target);
    pos = target;
  }
  let length = 0;
  for (const s of segments) for (let i = 1; i < s.points.length; i++) length += Math.hypot(s.points[i][0] - s.points[i - 1][0], s.points[i][1] - s.points[i - 1][1], s.points[i][2] - s.points[i - 1][2]);
  return { segments, bounds: { min, max }, length, units };
}

/** Convert parsed G-code into object curves: cutting moves only (rapids are skipped; the follow generator adds approach/retract). */
export function gcodeToCurves(g: GcodeResult): Array<{ name: string; points: number[][] }> {
  const curves: Array<{ name: string; points: number[][] }> = [];
  let n = 0;
  let prevCut = false;
  for (const s of g.segments) {
    if (s.kind !== 'cut' || s.points.length < 2) { prevCut = false; continue; }
    // consecutive cuts (feed / spindle changes only) form one continuous curve
    if (prevCut && curves.length) curves[curves.length - 1].points.push(...s.points.slice(1));
    else curves.push({ name: `cut ${++n}${s.feed ? ` F${s.feed}` : ''}`, points: s.points.slice() });
    prevCut = true;
  }
  return curves;
}

/** All segments (rapids and cuts) as curves with machining metadata, for `generateMachining`. */
export function gcodeToMachiningCurves(g: GcodeResult): Array<{ name: string; points: number[][]; kind: 'cut' | 'rapid'; feed?: number; spindle?: boolean; extrude?: boolean; tool?: number }> {
  const out: Array<{ name: string; points: number[][]; kind: 'cut' | 'rapid'; feed?: number; spindle?: boolean; extrude?: boolean; tool?: number }> = [];
  let nc = 0, nr = 0;
  for (const s of g.segments) {
    if (s.points.length < 2) continue;
    const name = s.kind === 'cut' ? `cut ${++nc}${s.feed ? ` F${s.feed}` : ''}${s.extrude ? ' E' : ''}` : `rapid ${++nr}`;
    out.push({ name, points: s.points, kind: s.kind, feed: s.feed, spindle: s.spindle, extrude: s.extrude, tool: s.tool });
  }
  return out;
}

/** Summary statistics of a parsed NC program. */
export function gcodeStats(g: GcodeResult): { cutLength: number; rapidLength: number; cuts: number; rapids: number; extruding: number; tools: number[]; feeds: number[] } {
  const len = (pts: number[][]) => { let l = 0; for (let i = 1; i < pts.length; i++) l += Math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1], pts[i][2] - pts[i - 1][2]); return l; };
  const st = { cutLength: 0, rapidLength: 0, cuts: 0, rapids: 0, extruding: 0, tools: [] as number[], feeds: [] as number[] };
  for (const s of g.segments) {
    const l = len(s.points);
    if (s.kind === 'cut') { st.cuts++; st.cutLength += l; if (s.extrude) st.extruding += l; } else { st.rapids++; st.rapidLength += l; }
    if (s.tool !== undefined && !st.tools.includes(s.tool)) st.tools.push(s.tool);
    if (s.feed !== undefined && !st.feeds.includes(s.feed)) st.feeds.push(s.feed);
  }
  return st;
}
