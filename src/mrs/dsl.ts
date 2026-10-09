/**
 * Text DSL of the group-control documents (one document = one kind; the first keyword names the kind):
 *
 *   consensus   graph <path|ring|star|complete|edges|disk> n= …; edge a b [w]; robot r at=x,y; x0 …; eps; steps;
 *               formation <circle|line|wedge|grid> r= gain=; rendezvous radius= gain=; wmsr F= malicious=; event sigma=;
 *               transport target=x,y kp= kd=; compare path ring …; comm radius= drop=
 *   swarm       model boids|vicsek|pso|aco|firefly|gwo|bee with its parameters (one model per document)
 *   allocation  robot r at=x,y; task t at=x,y [reward=]; random robots= tasks= area= seed=; methods …; cbba capacity= lambda= graph=
 *   gridmapf    map … end; agent a start=r,c goal=r,c; random agents= seed=; methods prioritized cbs; execute delay= runs=
 *   coverage    area xmin xmax ymin ymax res=; density gaussian center= sigma= base=; robots n seed= spawn=; iterations; range
 *   estimation  target x,y; robots n seed= radius=; noise vmin vmax; iterations; ci a= A= b= B=
 *   safety      scenario antipodal n= radius= | crossing | custom; robot r at= goal=; params d_safe= gamma= vmax= dt= steps= sense=; unstuck; stale lag=
 *   warehouse   cell; map … end; station NAME r,c; robots names homes= speed= handle= radio= drop= latency=; orders rate= reward= first= pickups= drops=;
 *               order id P D [reward= at=]; traffic settle= stale= wait= lookahead= protective=; cbba capacity= discount= period= stale= commit= lost=;
 *               fault robot at= kind=stop|mute|recover; duration s dt= seed=
 *   game        players A B; strategies r1 r2 ; c1 c2; payoff ri cj uA uB; coalition S=v …; start ri cj; rounds n
 *   marl        map … end (S start, G goal, # wall); params alpha= gamma= epsilon= episodes= step= goal= seed= [path=right];
 *               agents 2 starts=r,c;r,c goals=r,c;r,c penalty= episodes=
 *   evo         method ga bits= pop= generations= pc= pm= elitism= seed= initial= fitness=<expr in x>; method de|es benchmark= dim= …; objective consensus graph= n=
 *   ca          wolfram rule= width= steps=; life steps= size= … end; pheromone …
 *   fuzzy       input dL= dF= dR=; obstacles x,y …; goal x,y; rule dL dF dR -> v omega
 *   resilience  agent NAME value [lies A=v B=v]; trust graph= n= x0= liar= eps= steps=; degrade graph= n= order=; switched A1=a,b,c,d A2=… tau=min,max
 */
import { tokenize, lines, DslError, Opts } from '../ctl/dsl_core';
import { MrsKind, MRS_KIND_SET } from './model';
import { Vec2, Rng } from './rng';
import { FleetConfig, HOMEWORK_FLEET, HOMEWORK_WAREHOUSE } from './warehouse';
import { Bimatrix, CharFn } from './games';
import { FuzzyRule } from './fuzzy';
import { MissionSpec, MissionPhase, Architecture, PhaseKind } from './mission';

type L = { n: number; text: string; indent: number };
const num = (o: Opts, k: string, d: number): number => (o[k] === undefined ? d : Number(o[k]));
const str = (o: Opts, k: string, d: string): string => (o[k] === undefined ? d : String(o[k]));
const bool = (o: Opts, k: string, d: boolean): boolean => (o[k] === undefined ? d : o[k] === true || o[k] === 'true');
const pair = (v: unknown, n: number): Vec2 => { const m = /^\s*(-?[\d.eE+-]+)\s*[,;x ]\s*(-?[\d.eE+-]+)\s*$/.exec(String(v)); if (!m) throw new DslError(`expected x,y but got "${v}"`, n); return [Number(m[1]), Number(m[2])]; };
const list = (v: unknown): string[] => String(v).split(/[,;]/).map((s) => s.trim()).filter(Boolean);
const nums = (v: unknown): number[] => list(v).map(Number);
const pairs = (v: unknown, n: number): Vec2[] => String(v).split(';').map((s) => s.trim()).filter(Boolean).map((s) => pair(s, n));
/** `map` … `end` block → the RAW rows (comment stripping would eat rows that start with "#"). Returns the rows and the index of the next parsed line. */
function block(ls: L[], i: number, raw: string[]): { rows: string[]; next: number } {
  const rows: string[] = []; let j = ls[i].n; // raw index of the line after the header (n is 1-based)
  while (j < raw.length) { const t = raw[j].trim(); if (t === 'end') { j++; break; } if (t.length && !raw[j].startsWith(' ') && !raw[j].startsWith('\t')) break; if (t.length) rows.push(t); j++; }
  let next = i + 1; while (next < ls.length && ls[next].n <= j) next++;
  return { rows, next };
}
export function detectMrsKind(src: string): MrsKind | null { const first = lines(src)[0]?.text.trim().split(/\s+/)[0]?.toLowerCase(); return first && MRS_KIND_SET.has(first) ? (first as MrsKind) : null; }
function header(src: string, kind: MrsKind): { name: string; ls: L[]; raw: string[] } {
  const ls = lines(src); if (!ls.length) throw new DslError('empty document');
  const first = ls[0].text.trim(); const w = first.split(/\s+/)[0].toLowerCase(); if (w !== kind) throw new DslError(`a ${kind} document must start with "${kind} <name>" (found "${w}")`, ls[0].n);
  return { name: first.slice(w.length).trim() || kind, ls: ls.slice(1), raw: src.split(/\r?\n/) };
}

// --- consensus -------------------------------------------------------------------------------------------------------
export interface GraphSpec { type: 'path' | 'ring' | 'star' | 'complete' | 'edges' | 'disk'; n: number; edges: Array<[number, number, number?]>; radius: number }
export interface ConsensusDoc { name: string; graph: GraphSpec; robots: Array<{ name: string; at?: Vec2 }>; x0?: number[]; eps?: number; steps: number; seed: number; box: number; continuous: boolean; formation?: { shape: string; r: number; gain: number; steps: number; dt: number }; rendezvous?: { radius: number; gain: number; dt: number; steps: number; vmax: number }; wmsr?: { F: number; eps: number; steps: number; malicious: number[]; value: number; pattern: 'alternate' | 'constant' | 'ramp' }; event?: { sigma: number; abs: number }; transport?: { target: Vec2; m: number; kp: number; kd: number }; compare: string[]; comm?: { radius: number; drop: number }; failures: Array<{ robot: number; at: number }> }
export function parseConsensus(src: string): ConsensusDoc {
  const { name, ls } = header(src, 'consensus'); const d: ConsensusDoc = { name, graph: { type: 'complete', n: 0, edges: [], radius: 1 }, robots: [], steps: 30, seed: 1, box: 1, continuous: false, compare: [], failures: [] };
  for (const l of ls) {
    const { words, opts } = tokenize(l.text); const k = words[0].toLowerCase();
    if (k === 'graph') { const t = (words[1] ?? 'complete').toLowerCase() as GraphSpec['type']; if (!['path', 'ring', 'star', 'complete', 'edges', 'disk'].includes(t)) throw new DslError(`unknown graph type ${t}`, l.n); d.graph.type = t; d.graph.n = num(opts, 'n', d.graph.n); d.graph.radius = num(opts, 'radius', d.graph.radius); }
    else if (k === 'edge') { const a = Number(words[1]), b = Number(words[2]); if (!Number.isInteger(a) || !Number.isInteger(b)) throw new DslError('edge <i> <j> [weight] with 1-based robot numbers', l.n); d.graph.edges.push([a - 1, b - 1, words[3] !== undefined ? Number(words[3]) : num(opts, 'w', 1)]); if (d.graph.type !== 'disk') d.graph.type = 'edges'; d.graph.n = Math.max(d.graph.n, a, b); }
    else if (k === 'robot') { d.robots.push({ name: words[1] ?? `r${d.robots.length + 1}`, at: opts.at !== undefined ? pair(opts.at, l.n) : undefined }); }
    else if (k === 'robots') { const n = Number(words[1] ?? opts.n); if (!(n > 0)) throw new DslError('robots <n> [seed= box=]', l.n); d.graph.n = Math.max(d.graph.n, n); d.seed = num(opts, 'seed', d.seed); d.box = num(opts, 'box', d.box); for (let i = d.robots.length; i < n; i++) d.robots.push({ name: `r${i + 1}` }); }
    else if (k === 'x0') d.x0 = words.slice(1).map(Number);
    else if (k === 'eps' || k === 'epsilon') d.eps = Number(words[1]);
    else if (k === 'steps') d.steps = Number(words[1]);
    else if (k === 'continuous') d.continuous = words[1] !== 'false';
    else if (k === 'seed') d.seed = Number(words[1]);
    else if (k === 'formation') d.formation = { shape: (words[1] ?? 'circle').toLowerCase(), r: num(opts, 'r', 0.3), gain: num(opts, 'gain', 1), steps: num(opts, 'steps', 600), dt: num(opts, 'dt', 0.05) };
    else if (k === 'rendezvous') d.rendezvous = { radius: num(opts, 'radius', 1), gain: num(opts, 'gain', 0.5), dt: num(opts, 'dt', 0.002), steps: num(opts, 'steps', 5000), vmax: num(opts, 'vmax', 0.5) };
    else if (k === 'wmsr') d.wmsr = { F: num(opts, 'F', 1), eps: num(opts, 'eps', 0.1), steps: num(opts, 'steps', 300), malicious: opts.malicious !== undefined ? nums(opts.malicious).map((x) => x - 1) : [], value: num(opts, 'value', 100), pattern: (str(opts, 'pattern', 'alternate') as 'alternate' | 'constant' | 'ramp') };
    else if (k === 'event') d.event = { sigma: num(opts, 'sigma', 0.1), abs: num(opts, 'abs', 0) };
    else if (k === 'transport') d.transport = { target: pair(opts.target ?? '5,2', l.n), m: num(opts, 'm', 1), kp: num(opts, 'kp', 4), kd: num(opts, 'kd', 2) };
    else if (k === 'compare') d.compare = words.slice(1).map((w) => w.toLowerCase());
    else if (k === 'comm') d.comm = { radius: num(opts, 'radius', Infinity), drop: num(opts, 'drop', 0) };
    else if (k === 'fail' || k === 'failure') d.failures.push({ robot: Number(words[1]) - 1, at: num(opts, 'at', 0) });
    else throw new DslError(`unknown consensus line "${k}"`, l.n);
  }
  if (!d.graph.n) d.graph.n = d.robots.length || d.x0?.length || 4;
  if (d.x0 && d.x0.length !== d.graph.n) throw new DslError(`x0 has ${d.x0.length} values for ${d.graph.n} robots`);
  for (let i = d.robots.length; i < d.graph.n; i++) d.robots.push({ name: `r${i + 1}` });
  return d;
}

// --- swarm -----------------------------------------------------------------------------------------------------------
export interface SwarmDoc { name: string; model: 'boids' | 'vicsek' | 'pso' | 'aco' | 'firefly' | 'gwo' | 'bee'; n: number; steps: number; seed: number; box: number; dt: number; vmin: number; vmax: number; rSep: number; rView: number; wSep: number; wAli: number; wCoh: number; radius: number; speed: number; etas: number[]; source: Vec2; sigma: number; distractors: Array<[Vec2, number, number]>; range: [number, number]; robots: boolean; dMin: number; noise: number; forget: number; k: number; points: Vec2[]; ants: number; iterations: number; alpha: number; beta: number; rho: number; robotNames: string[] }
export function parseSwarm(src: string): SwarmDoc {
  const { name, ls } = header(src, 'swarm'); const d: SwarmDoc = { name, model: 'boids', n: 20, steps: 800, seed: 4, box: 1, dt: 0.05, vmin: 0.2, vmax: 0.3, rSep: 0.15, rView: 0.6, wSep: 0.05, wAli: 5, wCoh: 0.1, radius: 1, speed: 0.03, etas: [0.3, 1, 2, 3, 4, 5, 6], source: [0.7, -0.4], sigma: 0.5, distractors: [], range: [-2, 2], robots: false, dMin: 0.1, noise: 0, forget: 0, k: 1, points: [], ants: 10, iterations: 60, alpha: 1, beta: 3, rho: 0.3, robotNames: [] };
  let seen = false;
  for (const l of ls) {
    const { words, opts } = tokenize(l.text); const k = words[0].toLowerCase();
    if (k === 'model') {
      if (seen) throw new DslError('one model per swarm document', l.n); seen = true; const m = (words[1] ?? '').toLowerCase(); if (!['boids', 'vicsek', 'pso', 'aco', 'firefly', 'gwo', 'bee'].includes(m)) throw new DslError(`unknown swarm model "${m}" (boids | vicsek | pso | aco | firefly | gwo | bee)`, l.n); d.model = m as SwarmDoc['model'];
      if (m === 'vicsek') { d.n = 100; d.box = 5; d.steps = 300; d.seed = 5; } if (m === 'pso' || m === 'firefly' || m === 'gwo' || m === 'bee') { d.n = 12; d.steps = 150; d.seed = 8; } if (m === 'aco') { d.n = 10; d.seed = 1; }
      d.n = num(opts, 'n', d.n); d.steps = num(opts, 'steps', d.steps); d.seed = num(opts, 'seed', d.seed); d.box = num(opts, 'box', d.box); d.dt = num(opts, 'dt', d.dt); d.vmin = num(opts, 'vmin', d.vmin); d.vmax = num(opts, 'vmax', m === 'pso' ? 0.05 : d.vmax); d.rSep = num(opts, 'r_sep', d.rSep); d.rView = num(opts, 'r_view', d.rView); d.wSep = num(opts, 'w_sep', d.wSep); d.wAli = num(opts, 'w_ali', d.wAli); d.wCoh = num(opts, 'w_coh', d.wCoh);
      d.radius = num(opts, 'radius', d.radius); d.speed = num(opts, 'speed', d.speed); if (opts.eta !== undefined) d.etas = nums(opts.eta); if (opts.source !== undefined) d.source = pair(opts.source, l.n); d.sigma = num(opts, 'sigma', d.sigma); if (opts.range !== undefined) { const r = nums(opts.range); d.range = [r[0], r[1] ?? -r[0]]; } d.robots = bool(opts, 'robots', false); d.dMin = num(opts, 'd_min', d.dMin); d.noise = num(opts, 'noise', d.noise); d.forget = num(opts, 'forget', d.forget); d.k = num(opts, 'k', d.k);
      d.ants = num(opts, 'ants', d.n); d.iterations = num(opts, 'iterations', d.iterations); d.alpha = num(opts, 'alpha', d.alpha); d.beta = num(opts, 'beta', d.beta); d.rho = num(opts, 'rho', d.rho);
    } else if (k === 'distractor') d.distractors.push([pair(opts.at ?? words[1], l.n), num(opts, 'sigma', 0.3), num(opts, 'amp', 0.5)]);
    else if (k === 'point' || k === 'city') d.points.push(pair(opts.at ?? words[1], l.n));
    else if (k === 'robot') d.robotNames.push(words[1] ?? `r${d.robotNames.length + 1}`);
    else throw new DslError(`unknown swarm line "${k}"`, l.n);
  }
  return d;
}

// --- allocation --------------------------------------------------------------------------------------------------------
export interface AllocationDoc { name: string; robots: Array<{ name: string; at: Vec2 }>; tasks: Array<{ name: string; at: Vec2; reward: number; value: number }>; random?: { robots: number; tasks: number; area: number; seed: number }; methods: string[]; cbba: { capacity: number; lam: number; graph: 'complete' | 'path' | 'ring' | 'disk'; radius: number; speed: number } }
export function parseAllocation(src: string): AllocationDoc {
  const { name, ls } = header(src, 'allocation'); const d: AllocationDoc = { name, robots: [], tasks: [], methods: ['greedy', 'hungarian', 'ssi', 'cbba'], cbba: { capacity: 3, lam: 0.95, graph: 'complete', radius: 5, speed: 1 } };
  for (const l of ls) {
    const { words, opts } = tokenize(l.text); const k = words[0].toLowerCase();
    if (k === 'robot') d.robots.push({ name: words[1] ?? `r${d.robots.length + 1}`, at: pair(opts.at, l.n) });
    else if (k === 'task') d.tasks.push({ name: words[1] ?? `t${d.tasks.length + 1}`, at: pair(opts.at, l.n), reward: num(opts, 'reward', 10), value: num(opts, 'value', 20) });
    else if (k === 'random') d.random = { robots: num(opts, 'robots', 3), tasks: num(opts, 'tasks', 9), area: num(opts, 'area', 10), seed: num(opts, 'seed', 2) };
    else if (k === 'methods') d.methods = words.slice(1).map((w) => w.toLowerCase());
    else if (k === 'cbba') { d.cbba.capacity = num(opts, 'capacity', 3); d.cbba.lam = num(opts, 'lambda', num(opts, 'lam', 0.95)); d.cbba.graph = str(opts, 'graph', 'complete') as AllocationDoc['cbba']['graph']; d.cbba.radius = num(opts, 'radius', 5); d.cbba.speed = num(opts, 'speed', 1); }
    else throw new DslError(`unknown allocation line "${k}"`, l.n);
  }
  if (!d.random && (!d.robots.length || !d.tasks.length)) throw new DslError('give robot / task lines or a random instance');
  return d;
}

// --- grid MAPF -----------------------------------------------------------------------------------------------------------
export interface GridMapfDoc { name: string; map: string[]; agents: Array<{ name: string; start: [number, number]; goal: [number, number] }>; random?: { agents: number; seed: number }; methods: string[]; execute?: { delay: number; runs: number; seed: number }; cellSize: number }
export function parseGridMapf(src: string): GridMapfDoc {
  const { name, ls, raw } = header(src, 'gridmapf'); const d: GridMapfDoc = { name, map: [], agents: [], methods: ['prioritized', 'cbs'], cellSize: 1 };
  for (let i = 0; i < ls.length; i++) {
    const l = ls[i]; const { words, opts } = tokenize(l.text); const k = words[0].toLowerCase();
    if (k === 'map') { const b = block(ls, i, raw); d.map = b.rows; i = b.next - 1; d.cellSize = num(opts, 'cell', 1); }
    else if (k === 'agent') { const p = (v: unknown) => pair(v, l.n).map(Math.round) as [number, number]; d.agents.push({ name: words[1] ?? `a${d.agents.length + 1}`, start: p(opts.start), goal: p(opts.goal) }); }
    else if (k === 'random') d.random = { agents: num(opts, 'agents', 4), seed: num(opts, 'seed', 11) };
    else if (k === 'methods') d.methods = words.slice(1).map((w) => w.toLowerCase());
    else if (k === 'execute') d.execute = { delay: num(opts, 'delay', 0.3), runs: num(opts, 'runs', 5), seed: num(opts, 'seed', 12) };
    else throw new DslError(`unknown gridmapf line "${k}"`, l.n);
  }
  if (!d.map.length) throw new DslError('a map block is required (map … end, "#" shelves, "." free)');
  const w = d.map[0].length; if (d.map.some((r) => r.length !== w)) throw new DslError('map rows must have the same length');
  if (!d.random && !d.agents.length) throw new DslError('give agent lines or a random instance');
  return d;
}

// --- coverage / estimation -----------------------------------------------------------------------------------------------
export interface CoverageDoc { name: string; area: [number, number, number, number]; res: number; density: { kind: 'uniform' | 'gaussian'; center: Vec2; sigma: number; base: number }; robots: Array<{ name: string; at?: Vec2 }>; n: number; seed: number; spawn: [number, number]; iterations: number; range?: number; gain: number }
export function parseCoverage(src: string): CoverageDoc {
  const { name, ls } = header(src, 'coverage'); const d: CoverageDoc = { name, area: [0, 4, 0, 4], res: 0.1, density: { kind: 'gaussian', center: [3, 1], sigma: 0.8, base: 0.05 }, robots: [], n: 6, seed: 1, spawn: [0, 0.8], iterations: 60, gain: 1 };
  for (const l of ls) {
    const { words, opts } = tokenize(l.text); const k = words[0].toLowerCase();
    if (k === 'area') { const v = words.slice(1, 5).map(Number); if (v.length < 4 || v.some(Number.isNaN)) throw new DslError('area xmin xmax ymin ymax [res=]', l.n); d.area = v as [number, number, number, number]; d.res = num(opts, 'res', d.res); }
    else if (k === 'density') { const t = (words[1] ?? 'gaussian').toLowerCase(); d.density = { kind: t === 'uniform' ? 'uniform' : 'gaussian', center: opts.center !== undefined ? pair(opts.center, l.n) : d.density.center, sigma: num(opts, 'sigma', d.density.sigma), base: num(opts, 'base', d.density.base) }; }
    else if (k === 'robots') { d.n = Number(words[1] ?? opts.n ?? d.n); d.seed = num(opts, 'seed', d.seed); if (opts.spawn !== undefined) d.spawn = nums(opts.spawn) as [number, number]; }
    else if (k === 'robot') d.robots.push({ name: words[1] ?? `r${d.robots.length + 1}`, at: opts.at !== undefined ? pair(opts.at, l.n) : undefined });
    else if (k === 'iterations') d.iterations = Number(words[1]);
    else if (k === 'range') d.range = Number(words[1] ?? opts.r);
    else if (k === 'gain') d.gain = Number(words[1]);
    else throw new DslError(`unknown coverage line "${k}"`, l.n);
  }
  if (d.robots.length) d.n = Math.max(d.n, d.robots.length); if (d.robots.length && d.robots.length < d.n) for (let i = d.robots.length; i < d.n; i++) d.robots.push({ name: `r${i + 1}` });
  return d;
}
export interface EstimationDoc { name: string; target: Vec2; n: number; seed: number; radius: number; graph: 'disk' | 'ring' | 'path' | 'complete'; noise: [number, number]; iterations: number; ci?: { a: Vec2; A: [number, number]; b: Vec2; B: [number, number] } }
export function parseEstimation(src: string): EstimationDoc {
  const { name, ls } = header(src, 'estimation'); const d: EstimationDoc = { name, target: [2, -1], n: 8, seed: 3, radius: 0.55, graph: 'disk', noise: [0.01, 1], iterations: 300 };
  for (const l of ls) {
    const { words, opts } = tokenize(l.text); const k = words[0].toLowerCase();
    if (k === 'target') d.target = pair(words[1] ?? opts.at, l.n);
    else if (k === 'robots') { d.n = Number(words[1] ?? opts.n ?? d.n); d.seed = num(opts, 'seed', d.seed); d.radius = num(opts, 'radius', d.radius); d.graph = str(opts, 'graph', 'disk') as EstimationDoc['graph']; }
    else if (k === 'noise') d.noise = [Number(words[1]), Number(words[2] ?? words[1])];
    else if (k === 'iterations') d.iterations = Number(words[1]);
    else if (k === 'ci') d.ci = { a: pair(opts.a ?? '0,0', l.n), A: nums(opts.A ?? '1,4') as [number, number], b: pair(opts.b ?? '1,1', l.n), B: nums(opts.B ?? '4,1') as [number, number] };
    else throw new DslError(`unknown estimation line "${k}"`, l.n);
  }
  return d;
}

// --- safety -----------------------------------------------------------------------------------------------------------
export interface SafetyDoc { name: string; scenario: 'antipodal' | 'crossing' | 'custom'; n: number; radius: number; robots: Array<{ name: string; at: Vec2; goal: Vec2; /** own speed limit (uncooperative intruders can be slower or faster than the fleet) */ speed?: number }>; dSafe: number; gamma: number; vmax: number; dt: number; steps: number; sense: number; gain: number; unstuck?: { angle: number; hold: number }; stale?: { lag: number; vmaxJ: number }; uncooperative: number[]; compareNominal: boolean }
export function parseSafety(src: string): SafetyDoc {
  const { name, ls } = header(src, 'safety'); const d: SafetyDoc = { name, scenario: 'antipodal', n: 8, radius: 1, robots: [], dSafe: 0.2, gamma: 2, vmax: 0.3, dt: 0.02, steps: 2500, sense: 1, gain: 1, uncooperative: [], compareNominal: true };
  for (const l of ls) {
    const { words, opts } = tokenize(l.text); const k = words[0].toLowerCase();
    if (k === 'scenario') { const s = (words[1] ?? 'antipodal').toLowerCase(); if (!['antipodal', 'crossing', 'custom'].includes(s)) throw new DslError(`unknown scenario ${s}`, l.n); d.scenario = s as SafetyDoc['scenario']; d.n = num(opts, 'n', d.n); d.radius = num(opts, 'radius', d.radius); }
    else if (k === 'robot') { d.robots.push({ name: words[1] ?? `r${d.robots.length + 1}`, at: pair(opts.at, l.n), goal: pair(opts.goal, l.n), speed: opts.speed !== undefined ? Number(opts.speed) : undefined }); d.scenario = 'custom'; }
    else if (k === 'params') { d.dSafe = num(opts, 'd_safe', d.dSafe); d.gamma = num(opts, 'gamma', d.gamma); d.vmax = num(opts, 'vmax', d.vmax); d.dt = num(opts, 'dt', d.dt); d.steps = num(opts, 'steps', d.steps); d.sense = num(opts, 'sense', d.sense); d.gain = num(opts, 'gain', d.gain); d.compareNominal = bool(opts, 'compare', true); }
    else if (k === 'unstuck') d.unstuck = { angle: (num(opts, 'angle', -45) * Math.PI) / 180, hold: num(opts, 'hold', 50) };
    else if (k === 'stale') d.stale = { lag: num(opts, 'lag', 25), vmaxJ: num(opts, 'vmax_j', 0.1) };
    else if (k === 'uncooperative') d.uncooperative = words.slice(1).map((w) => Number(w) - 1);
    else throw new DslError(`unknown safety line "${k}"`, l.n);
  }
  return d;
}

// --- warehouse ----------------------------------------------------------------------------------------------------------
export interface WarehouseDoc { name: string; cfg: FleetConfig; stationKinds: Record<string, string> }
export function parseWarehouse(src: string): WarehouseDoc {
  const { name, ls, raw } = header(src, 'warehouse'); const cfg: FleetConfig = JSON.parse(JSON.stringify(HOMEWORK_FLEET)); cfg.faults = []; const stationKinds: Record<string, string> = {}; let mapGiven = false, stationsGiven = false;
  for (let i = 0; i < ls.length; i++) {
    const l = ls[i]; const { words, opts } = tokenize(l.text); const k = words[0].toLowerCase();
    if (k === 'cell') cfg.warehouse.cell = Number(words[1]);
    else if (k === 'map') { const b = block(ls, i, raw); cfg.warehouse.map = b.rows.join('\n'); i = b.next - 1; mapGiven = true; }
    else if (k === 'station') { if (!stationsGiven) { cfg.warehouse.stations = {}; stationsGiven = true; } const c = pair(words[2] ?? opts.at, l.n); cfg.warehouse.stations[words[1]] = [Math.round(c[0]), Math.round(c[1])]; stationKinds[words[1]] = str(opts, 'kind', words[1][0] === 'P' ? 'pickup' : words[1][0] === 'D' ? 'drop' : 'home'); }
    else if (k === 'robots') { const names = list(words[1] ?? opts.names ?? cfg.robots.names.join(',')); cfg.robots.names = names; if (opts.homes !== undefined) cfg.robots.homes = list(opts.homes); cfg.robots.speed = num(opts, 'speed', cfg.robots.speed); cfg.robots.handleTime = num(opts, 'handle', cfg.robots.handleTime); cfg.robots.radioRange = num(opts, 'radio', cfg.robots.radioRange); cfg.robots.radioDrop = num(opts, 'drop', cfg.robots.radioDrop); cfg.robots.radioLatency = num(opts, 'latency', cfg.robots.radioLatency); }
    else if (k === 'orders') { cfg.orders.rate = num(opts, 'rate', cfg.orders.rate); cfg.orders.reward = num(opts, 'reward', cfg.orders.reward); cfg.orders.firstBatch = num(opts, 'first', cfg.orders.firstBatch); if (opts.pickups !== undefined) cfg.orders.pickups = list(opts.pickups); if (opts.drops !== undefined) cfg.orders.drops = list(opts.drops); }
    else if (k === 'order') { (cfg.orders.fixed ??= []).push({ id: words[1], pickup: words[2], drop: words[3], reward: num(opts, 'reward', cfg.orders.reward), at: num(opts, 'at', 0) }); }
    else if (k === 'traffic') { cfg.traffic.settle = num(opts, 'settle', cfg.traffic.settle); cfg.traffic.staleAfter = num(opts, 'stale', cfg.traffic.staleAfter); cfg.traffic.waitLimit = num(opts, 'wait', cfg.traffic.waitLimit); cfg.traffic.lookahead = num(opts, 'lookahead', cfg.traffic.lookahead); cfg.traffic.protective = bool(opts, 'protective', true); }
    else if (k === 'cbba') { cfg.cbba.capacity = num(opts, 'capacity', cfg.cbba.capacity); cfg.cbba.discount = num(opts, 'discount', cfg.cbba.discount); cfg.cbba.period = num(opts, 'period', cfg.cbba.period); cfg.cbba.staleAfter = num(opts, 'stale', cfg.cbba.staleAfter); cfg.cbba.commitAfter = num(opts, 'commit', cfg.cbba.commitAfter); cfg.cbba.lostAfter = num(opts, 'lost', cfg.cbba.lostAfter); }
    else if (k === 'fault') { const kind = str(opts, 'kind', 'stop'); if (!['stop', 'mute', 'recover'].includes(kind)) throw new DslError('fault <robot> at= kind=stop|mute|recover', l.n); cfg.faults.push({ robot: words[1], at: num(opts, 'at', 0), kind: kind as 'stop' | 'mute' | 'recover' }); }
    else if (k === 'duration') { cfg.duration = Number(words[1]); cfg.dt = num(opts, 'dt', cfg.dt); cfg.seed = num(opts, 'seed', cfg.seed); cfg.idleToHome = num(opts, 'idle_home', cfg.idleToHome); }
    else throw new DslError(`unknown warehouse line "${k}"`, l.n);
  }
  if (!stationsGiven) for (const s of Object.keys(HOMEWORK_WAREHOUSE.stations)) stationKinds[s] = s[0] === 'P' ? 'pickup' : s[0] === 'D' ? 'drop' : 'home';
  if (mapGiven && !stationsGiven) throw new DslError('a custom map needs station lines');
  if (cfg.orders.rate <= 0 && !cfg.orders.firstBatch && !cfg.orders.fixed?.length) throw new DslError('no orders: set orders rate= / first= or order lines');
  for (const h of cfg.robots.homes) if (!cfg.warehouse.stations[h]) throw new DslError(`home station ${h} is not defined`);
  return { name, cfg, stationKinds };
}

// --- game ---------------------------------------------------------------------------------------------------------------
export interface GameDoc { name: string; players: [string, string]; game?: Bimatrix; coalition?: { v: CharFn; n: number; names: string[] }; start?: [number, number]; rounds: number; replicator?: number[] }
export function parseGame(src: string): GameDoc {
  const { name, ls } = header(src, 'game'); const d: GameDoc = { name, players: ['Robot 1', 'Robot 2'], rounds: 500 }; let rows: string[] = [], cols: string[] = []; const cells = new Map<string, [number, number]>();
  for (const l of ls) {
    const { words, opts } = tokenize(l.text); const k = words[0].toLowerCase();
    if (k === 'players') d.players = [words[1] ?? 'A', words[2] ?? 'B'];
    else if (k === 'strategies') { const s = words.slice(1).join(' ').split(';'); rows = s[0].trim().split(/\s+/).filter(Boolean); cols = (s[1] ?? s[0]).trim().split(/\s+/).filter(Boolean); }
    else if (k === 'payoff') { if (words.length < 5) throw new DslError('payoff <row> <col> <u1> <u2>', l.n); cells.set(`${words[1]}|${words[2]}`, [Number(words[3]), Number(words[4])]); if (!rows.includes(words[1])) rows.push(words[1]); if (!cols.includes(words[2])) cols.push(words[2]); }
    else if (k === 'coalition') { // coalition A=10 B=20 AB=60 … (single-letter players; the tokenizer delivers them as key=value options)
      const entries = Object.entries(opts).filter(([kk]) => /^[A-Za-z0-9]+$/.test(kk)); if (!entries.length || words.length > 1) throw new DslError('coalition A=10 B=20 AB=60 … (single-letter players)', l.n);
      const names = new Set<string>(); for (const [kk] of entries) for (const ch of kk) names.add(ch); const nm = [...names].sort(); const v: CharFn = {};
      for (const [kk, val] of entries) { const x = Number(val); if (!Number.isFinite(x)) throw new DslError(`coalition ${kk}: a number is required`, l.n); v[[...kk].map((c) => nm.indexOf(c)).sort((a, b) => a - b).join(',')] = x; }
      d.coalition = { v, n: nm.length, names: nm };
    }
    else if (k === 'start') d.start = [rows.indexOf(words[1]), cols.indexOf(words[2])];
    else if (k === 'rounds') d.rounds = Number(words[1]);
    else if (k === 'replicator') d.replicator = words.slice(1).map(Number);
    else throw new DslError(`unknown game line "${k}"`, l.n);
  }
  if (cells.size) { const u = rows.map((r) => cols.map((c) => { const v = cells.get(`${r}|${c}`); if (!v) throw new DslError(`missing payoff for ${r} ${c}`); return v; })); d.game = { rows, cols, u }; }
  if (!d.game && !d.coalition) throw new DslError('give payoff lines (normal-form game) or a coalition line (cooperative game)');
  return d;
}

// --- marl -----------------------------------------------------------------------------------------------------------------
export interface MarlDoc { name: string; map: string[]; alpha: number; gamma: number; epsilon: number; episodes: number; stepReward: number; goalReward: number; seed: number; forceRight: boolean; sarsa: boolean; agents?: { starts: [Vec2, Vec2]; goals: [Vec2, Vec2]; penalty: number; episodes: number } }
export function parseMarl(src: string): MarlDoc {
  const { name, ls, raw } = header(src, 'marl'); const d: MarlDoc = { name, map: [], alpha: 0.5, gamma: 0.9, epsilon: 0.1, episodes: 300, stepReward: -1, goalReward: 10, seed: 1, forceRight: false, sarsa: false };
  for (let i = 0; i < ls.length; i++) {
    const l = ls[i]; const { words, opts } = tokenize(l.text); const k = words[0].toLowerCase();
    if (k === 'map') { const b = block(ls, i, raw); d.map = b.rows; i = b.next - 1; }
    else if (k === 'params') { d.alpha = num(opts, 'alpha', d.alpha); d.gamma = num(opts, 'gamma', d.gamma); d.epsilon = num(opts, 'epsilon', d.epsilon); d.episodes = num(opts, 'episodes', d.episodes); d.stepReward = num(opts, 'step', d.stepReward); d.goalReward = num(opts, 'goal', d.goalReward); d.seed = num(opts, 'seed', d.seed); d.forceRight = str(opts, 'path', '') === 'right'; d.sarsa = str(opts, 'method', 'q') === 'sarsa'; }
    else if (k === 'agents') { const s = pairs(opts.starts ?? '0,0;2,3', l.n), g = pairs(opts.goals ?? '2,3;0,0', l.n); d.agents = { starts: [s[0], s[1] ?? s[0]], goals: [g[0], g[1] ?? g[0]], penalty: num(opts, 'penalty', -5), episodes: num(opts, 'episodes', 600) }; }
    else throw new DslError(`unknown marl line "${k}"`, l.n);
  }
  if (!d.map.length) throw new DslError('a map block is required (S start, G goal, # walls)');
  return d;
}

// --- evo ------------------------------------------------------------------------------------------------------------------
export interface EvoDoc { name: string; methods: Array<{ method: 'ga' | 'de' | 'es'; bits: number; pop: number; generations: number; pc: number; pm: number; elitism: boolean; seed: number; initial?: string[]; fitness: string; benchmark: string; dim: number; x0?: number[]; objective?: 'consensus'; graph: string; n: number }> }
export function parseEvo(src: string): EvoDoc {
  const { name, ls } = header(src, 'evo'); const d: EvoDoc = { name, methods: [] };
  for (const l of ls) {
    const { words, opts } = tokenize(l.text); const k = words[0].toLowerCase();
    if (k === 'method') { const m = (words[1] ?? 'ga').toLowerCase(); if (!['ga', 'de', 'es'].includes(m)) throw new DslError('method ga | de | es', l.n); d.methods.push({ method: m as 'ga' | 'de' | 'es', bits: num(opts, 'bits', 5), pop: num(opts, 'pop', m === 'ga' ? 4 : 20), generations: num(opts, 'generations', m === 'ga' ? 30 : m === 'de' ? 150 : 80), pc: num(opts, 'pc', 0.9), pm: num(opts, 'pm', 0.05), elitism: bool(opts, 'elitism', true), seed: num(opts, 'seed', 4), initial: opts.initial !== undefined ? list(opts.initial) : undefined, fitness: str(opts, 'fitness', 'x*x'), benchmark: str(opts, 'benchmark', 'sphere'), dim: num(opts, 'dim', 2), x0: opts.x0 !== undefined ? nums(opts.x0) : undefined, objective: opts.objective === 'consensus' ? 'consensus' : undefined, graph: str(opts, 'graph', 'path'), n: num(opts, 'n', 6) }); }
    else throw new DslError(`unknown evo line "${k}"`, l.n);
  }
  if (!d.methods.length) throw new DslError('give at least one method line');
  return d;
}

// --- ca --------------------------------------------------------------------------------------------------------------------
export interface CaDoc { name: string; wolfram?: { rule: number; width: number; steps: number }; life?: { steps: number; size: number; pattern: string[] }; pheromone?: { size: number; steps: number; rho: number; kappa: number; path: Vec2[] } }
export function parseCa(src: string): CaDoc {
  const { name, ls, raw } = header(src, 'ca'); const d: CaDoc = { name };
  for (let i = 0; i < ls.length; i++) {
    const l = ls[i]; const { words, opts } = tokenize(l.text); const k = words[0].toLowerCase();
    if (k === 'wolfram') d.wolfram = { rule: num(opts, 'rule', 90), width: num(opts, 'width', 31), steps: num(opts, 'steps', 15) };
    else if (k === 'life') { const b = block(ls, i, raw); i = b.next - 1; d.life = { steps: num(opts, 'steps', 12), size: num(opts, 'size', 10), pattern: b.rows.length ? b.rows : ['.#.', '..#', '###'] }; }
    else if (k === 'pheromone') d.pheromone = { size: num(opts, 'size', 12), steps: num(opts, 'steps', 40), rho: num(opts, 'rho', 0.05), kappa: num(opts, 'kappa', 0.1), path: opts.path !== undefined ? pairs(opts.path, l.n) : [[2, 2], [2, 3], [2, 4], [3, 4], [4, 4], [5, 4], [6, 4]] };
    else throw new DslError(`unknown ca line "${k}"`, l.n);
  }
  if (!d.wolfram && !d.life && !d.pheromone) throw new DslError('give a wolfram, life or pheromone line');
  return d;
}

// --- fuzzy -----------------------------------------------------------------------------------------------------------------
export interface FuzzyDoc { name: string; input?: { dL: number; dF: number; dR: number }; obstacles: Vec2[]; goal?: Vec2; rules: FuzzyRule[]; start: Vec2 }
export function parseFuzzy(src: string): FuzzyDoc {
  const { name, ls } = header(src, 'fuzzy'); const d: FuzzyDoc = { name, obstacles: [], rules: [], start: [0, 0] };
  for (const l of ls) {
    const { words, opts } = tokenize(l.text); const k = words[0].toLowerCase();
    if (k === 'input') d.input = { dL: num(opts, 'dL', 2), dF: num(opts, 'dF', 0.7), dR: num(opts, 'dR', 1.6) };
    else if (k === 'obstacles' || k === 'obstacle') { for (const w of words.slice(1)) d.obstacles.push(pair(w, l.n)); if (opts.goal !== undefined) d.goal = pair(opts.goal, l.n); }
    else if (k === 'goal') d.goal = pair(words[1] ?? opts.at, l.n);
    else if (k === 'start') d.start = pair(words[1] ?? opts.at, l.n);
    else if (k === 'rule') { const arrow = words.indexOf('->'); if (arrow !== 4 || words.length < 7) throw new DslError('rule <dL> <dF> <dR> -> <v> <omega>', l.n); d.rules.push({ if: { dL: words[1], dF: words[2], dR: words[3] }, then: { v: words[5], omega: words[6] } }); }
    else throw new DslError(`unknown fuzzy line "${k}"`, l.n);
  }
  return d;
}

// --- resilience ----------------------------------------------------------------------------------------------------------------
export interface ResilienceDoc { name: string; agents: Array<{ name: string; value: string; lies?: Record<string, string> }>; trust?: { graph: string; n: number; x0?: number[]; liars: number[]; eps: number; steps: number; value: number }; degrade?: { graph: string; n: number; order: number[] }; switched?: { modes: Array<[[number, number], [number, number]]>; tauMin: number; tauMax: number } }
export function parseResilience(src: string): ResilienceDoc {
  const { name, ls } = header(src, 'resilience'); const d: ResilienceDoc = { name, agents: [] };
  for (const l of ls) {
    const { words, opts } = tokenize(l.text); const k = words[0].toLowerCase();
    if (k === 'agent') { const lies: Record<string, string> = {}; let liar = false; for (const [kk, v] of Object.entries(opts)) { lies[kk] = String(v); liar = true; } const li = words.indexOf('lies'); d.agents.push({ name: words[1], value: words[2] ?? 'continue', lies: liar || li > 0 ? lies : undefined }); }
    else if (k === 'trust') d.trust = { graph: str(opts, 'graph', 'complete'), n: num(opts, 'n', 6), x0: opts.x0 !== undefined ? nums(opts.x0) : undefined, liars: opts.liar !== undefined ? nums(opts.liar).map((x) => x - 1) : [], eps: num(opts, 'eps', 0.1), steps: num(opts, 'steps', 200), value: num(opts, 'value', 100) };
    else if (k === 'degrade') d.degrade = { graph: str(opts, 'graph', 'star'), n: num(opts, 'n', 5), order: opts.order !== undefined ? nums(opts.order).map((x) => x - 1) : [0] };
    else if (k === 'switched') { const modes: Array<[[number, number], [number, number]]> = []; for (const [kk, v] of Object.entries(opts)) if (/^A\d+$/.test(kk)) { const m = nums(v); if (m.length !== 4) throw new DslError(`${kk}=a,b,c,d (row-major 2×2)`, l.n); modes.push([[m[0], m[1]], [m[2], m[3]]]); } const tau = opts.tau !== undefined ? nums(opts.tau) : [0.05, 3]; if (modes.length < 2) throw new DslError('switched A1=… A2=… [tau=min,max]', l.n); d.switched = { modes, tauMin: tau[0], tauMax: tau[1] ?? 3 }; }
    else throw new DslError(`unknown resilience line "${k}"`, l.n);
  }
  if (!d.agents.length && !d.trust && !d.degrade && !d.switched) throw new DslError('give agent lines (Byzantine agreement), trust, degrade or switched lines');
  return d;
}


// --- mission (architectures) ----------------------------------------------------------------------------------------
export function parseMission(src: string): MissionSpec {
  const { name, ls } = header(src, 'mission');
  const d: MissionSpec = { name, robots: [], architecture: 'compare', phases: [], comm: { radius: 4, drop: 0, period: 0.5, lost: 3, settle: 1 }, coordinator: { at: [0, 0], range: Infinity }, safety: { dSafe: 0.4, gamma: 2, sense: 1.5 }, obstacles: [], failures: [], duration: 300, dt: 0.1, seed: 1, vmax: 0.5, drive: 'unicycle' };
  let box = 3, seed = 1;
  for (const l of ls) {
    const { words, opts } = tokenize(l.text); const k = words[0].toLowerCase();
    if (k === 'robot') d.robots.push({ name: words[1] ?? `r${d.robots.length + 1}`, at: opts.at !== undefined ? pair(opts.at, l.n) : [NaN, NaN], home: opts.home !== undefined ? pair(opts.home, l.n) : undefined, speed: opts.speed !== undefined ? Number(opts.speed) : undefined });
    else if (k === 'robots') { const names = words.slice(1); const n = names.length ? names.length : num(opts, 'n', 6); box = num(opts, 'box', box); seed = num(opts, 'seed', seed); for (let i = 0; i < n; i++) d.robots.push({ name: names[i] ?? `r${d.robots.length + 1}`, at: [NaN, NaN] }); }
    else if (k === 'architecture') { const a = (words[1] ?? 'compare').toLowerCase(); if (!['centralized', 'centralised', 'decentralized', 'decentralised', 'hybrid', 'compare'].includes(a)) throw new DslError('architecture centralized | decentralized | hybrid | compare', l.n); d.architecture = (a.startsWith('cen') ? 'centralized' : a.startsWith('dec') ? 'decentralized' : a) as Architecture | 'compare'; }
    else if (k === 'comm') { d.comm.radius = num(opts, 'radius', d.comm.radius); d.comm.drop = num(opts, 'drop', d.comm.drop); d.comm.period = num(opts, 'period', d.comm.period); d.comm.lost = num(opts, 'lost', d.comm.lost); d.comm.settle = num(opts, 'settle', d.comm.settle); }
    else if (k === 'coordinator') { d.coordinator = { at: opts.at !== undefined ? pair(opts.at, l.n) : d.coordinator.at, range: num(opts, 'range', Infinity), fail: opts.fail !== undefined ? Number(opts.fail) : undefined, recover: opts.recover !== undefined ? Number(opts.recover) : undefined }; }
    else if (k === 'phase') {
      const kind = (words[1] ?? '').toLowerCase() as PhaseKind; if (!['form', 'goto', 'allocate', 'gather', 'cover', 'home', 'hold'].includes(kind)) throw new DslError('phase form | goto | allocate | gather | cover | home | hold', l.n);
      const ph: MissionPhase = { kind, name: str(opts, 'name', `${kind}${d.phases.filter((x) => x.kind === kind).length ? ' ' + (d.phases.filter((x) => x.kind === kind).length + 1) : ''}`), shape: (words[2] && !words[2].includes('=') ? words[2] : str(opts, 'shape', 'circle')).toLowerCase(), r: num(opts, 'r', 1), gain: num(opts, 'gain', 1), at: opts.at !== undefined ? pair(opts.at, l.n) : undefined, zone: opts.zone !== undefined ? String(opts.zone) : undefined, targets: opts.targets !== undefined ? pairs(opts.targets, l.n) : [], targetNames: [], area: opts.area !== undefined ? (nums(opts.area) as [number, number, number, number]) : undefined, seconds: num(opts, 'seconds', 5), speed: num(opts, 'speed', 0.3), tol: num(opts, 'tol', 0.08) };
      if (kind === 'goto' && !ph.at && !ph.zone) throw new DslError('phase goto needs at=x,y or zone=<name>', l.n);
      if (kind === 'allocate' && !ph.targets.length && opts.zones === undefined) throw new DslError('phase allocate needs targets=x,y;x,y;… (or zones=Z1,Z2 on the station)', l.n);
      if (kind === 'cover' && !ph.area) throw new DslError('phase cover needs area=xmin,xmax,ymin,ymax', l.n);
      if (opts.zones !== undefined) ph.targetNames = list(opts.zones); if (!ph.targetNames.length) ph.targetNames = ph.targets.map((_, j) => `t${j + 1}`);
      d.phases.push(ph);
    }
    else if (k === 'safety') { d.safety.dSafe = num(opts, 'd_safe', d.safety.dSafe); d.safety.gamma = num(opts, 'gamma', d.safety.gamma); d.safety.sense = num(opts, 'sense', d.safety.sense); }
    else if (k === 'obstacle') d.obstacles.push({ at: pair(opts.at ?? words[1], l.n), r: num(opts, 'r', 0.5), name: opts.name !== undefined ? String(opts.name) : undefined });
    else if (k === 'fail') { const idx = d.robots.findIndex((r) => r.name === words[1]); d.failures.push({ robot: idx >= 0 ? idx : Number(words[1]) - 1, at: num(opts, 'at', 0) }); }
    else if (k === 'duration') { d.duration = Number(words[1]); d.dt = num(opts, 'dt', d.dt); d.seed = num(opts, 'seed', d.seed); d.vmax = num(opts, 'vmax', d.vmax); }
    else if (k === 'drive') d.drive = (words[1] ?? 'unicycle') === 'pose' ? 'pose' : 'unicycle';
    else if (k === 'supervisor') d.supervisor = words.slice(1).join(' ');
    else if (k === 'modes') d.modes = words.slice(1).join(' ');
    else throw new DslError(`unknown mission line "${k}"`, l.n);
  }
  if (!d.robots.length) for (let i = 0; i < 6; i++) d.robots.push({ name: `r${i + 1}`, at: [NaN, NaN] });
  const rng = new Rng(seed); for (const r of d.robots) if (Number.isNaN(r.at[0])) r.at = [rng.uniform(-box, box), rng.uniform(-box, box)];
  for (const f of d.failures) if (!(f.robot >= 0 && f.robot < d.robots.length)) throw new DslError(`fail: unknown robot`);
  if (!d.phases.length) throw new DslError('give at least one phase line');
  return d;
}

export type MrsDoc = MissionSpec | ConsensusDoc | SwarmDoc | AllocationDoc | GridMapfDoc | CoverageDoc | EstimationDoc | SafetyDoc | WarehouseDoc | GameDoc | MarlDoc | EvoDoc | CaDoc | FuzzyDoc | ResilienceDoc;
export const MRS_PARSERS: Record<MrsKind, (src: string) => MrsDoc> = { mission: parseMission, consensus: parseConsensus, swarm: parseSwarm, allocation: parseAllocation, gridmapf: parseGridMapf, coverage: parseCoverage, estimation: parseEstimation, safety: parseSafety, warehouse: parseWarehouse, game: parseGame, marl: parseMarl, evo: parseEvo, ca: parseCa, fuzzy: parseFuzzy, resilience: parseResilience };

// --- templates ------------------------------------------------------------------------------------------------------------------
export const MRS_TEMPLATES: Record<MrsKind, string> = {
  mission: `mission Form, move, allocate, return — three architectures
robots r1 r2 r3 r4 r5 r6 box=3 seed=1      # names of station robots (positions from the station in Run on fleet)
architecture compare                        # centralized | decentralized | hybrid | compare
comm radius=4 drop=0.05 period=0.5 lost=3 settle=1
coordinator at=0,0 range=12 fail=60 recover=110   # the fleet manager: radio range, outage window
phase form circle r=1.2
phase goto at=6,0 speed=0.3
phase allocate targets=8,2;9,-1;7,-3;10,1;8,-2;9,3
phase gather
phase home
safety d_safe=0.4 gamma=2 sense=1.5
obstacle at=3,0.5 r=0.6
fail r4 at=40
duration 300 dt=0.1 seed=1 vmax=0.5`,
  consensus: `consensus Chain of four robots (§4.3.4)
graph path n=4        # path | ring | star | complete | edges (edge i j) | disk radius=
x0 0 4 8 12
eps 0.25
steps 30
compare path ring star complete   # table 4.1 for this n
formation circle r=0.3 gain=1 steps=600 dt=0.05
event sigma=0.15 abs=0.01          # event-triggered variant: broadcast only on a relative change`,
  swarm: `swarm Flock of twenty robots (ПР2)
model boids n=20 steps=800 dt=0.05 vmin=0.2 vmax=0.3 r_sep=0.15 r_view=0.6 w_sep=0.05 w_ali=5 w_coh=0.1 seed=4
# model vicsek n=100 box=5 radius=1 speed=0.03 steps=300 eta=0.3,1,2,3,4,5,6
# model pso source=0.7,-0.4 sigma=0.5 n=12 steps=150 seed=8 range=-2,2
# model pso robots=true source=2,1.5 sigma=0.8 n=8 steps=400 vmax=0.03 d_min=0.1 noise=0.005 range=0,0.5
# model aco n=10 ants=10 iterations=60 alpha=1 beta=3 rho=0.3 seed=1`,
  allocation: `allocation Nine tasks for three robots (ПР3)
random robots=3 tasks=9 area=10 seed=2
methods greedy hungarian ssi cbba vickrey
cbba capacity=3 lambda=0.95 graph=complete`,
  gridmapf: `gridmapf Warehouse crossing (ПР4)
map
  ..........
  .##.##.##.
  ..........
  .##.##.##.
  ..........
  .##.##.##.
  ..........
end
agent a start=0,0 goal=6,9
agent b start=6,9 goal=0,0
agent c start=0,9 goal=6,0
agent d start=6,0 goal=0,9
methods prioritized cbs
execute delay=0.3 runs=5 seed=12`,
  coverage: `coverage Hot spot at (3, 1) (ПР5)
area 0 4 0 4 res=0.1
density gaussian center=3,1 sigma=0.8 base=0.05
robots 6 seed=1 spawn=0,0.8
iterations 60
# range 0.8    # limited sensing radius (decentralised Lloyd)`,
  estimation: `estimation Target position by eight robots (ПР5)
target 2,-1
robots 8 seed=3 radius=0.55
noise 0.01 1.0
iterations 300
ci a=0,0 A=1,4 b=1,1 B=4,1`,
  safety: `safety Antipodal swap of eight robots (ПР6)
scenario antipodal n=8 radius=1
params d_safe=0.2 gamma=2 vmax=0.3 dt=0.02 steps=2500 sense=1
unstuck angle=-45 hold=50
# stale lag=25 vmax_j=0.1`,
  warehouse: `warehouse Homework fleet: four robots, six stations
# map / stations of the homework (config/warehouse.yaml) are the defaults; override with map … end and station lines
robots r1,r2,r3,r4 homes=H1,H2,H3,H4 speed=0.3 handle=3 radio=4 drop=0 latency=0.05
orders rate=0.05 reward=10 first=6 pickups=P1,P2,P3 drops=D1,D2,D3
traffic settle=0.3 stale=2 wait=6 lookahead=2 protective=true
cbba capacity=3 discount=0.98 period=0.5 stale=3 commit=2 lost=6
# fault r2 at=120 kind=stop
duration 600 dt=0.1 seed=1`,
  game: `game Task allocation between two robots (§9.3.4)
players R1 R2
strategies Z1 Z2 ; Z1 Z2
payoff Z1 Z1 4 4
payoff Z1 Z2 10 6
payoff Z2 Z1 6 10
payoff Z2 Z2 2 2
start Z1 Z1
rounds 500
coalition A=10 B=20 C=30 AB=60 AC=70 BC=90 ABC=120`,
  marl: `marl Corridor Q-learning (§7.2.4)
map
  S..G
end
params alpha=0.5 gamma=0.9 epsilon=0.1 episodes=300 step=-1 goal=10 seed=1
agents 2 starts=0,0;0,3 goals=0,3;0,0 penalty=-5 episodes=600`,
  evo: `evo Genetic algorithm for x² (§12.2.6) and continuous tuning
method ga bits=5 pop=4 generations=30 pc=0.9 pm=0.05 elitism=true seed=4 initial=01101,11000,01000,10011 fitness="x*x"
method de benchmark=rastrigin dim=2 pop=20 generations=150 seed=5
method es objective=consensus graph=path n=6 generations=40`,
  ca: `ca Rule 90, a glider and a pheromone trail
wolfram rule=90 width=31 steps=15
life steps=12 size=10
  .#.
  ..#
  ###
end
pheromone size=12 steps=40 rho=0.05 kappa=0.1`,
  fuzzy: `fuzzy Obstacle avoidance (§13.4.2)
input dL=2.0 dF=0.7 dR=1.6
obstacles 3,0 3,0.4 3,-0.4 goal=6,0`,
  resilience: `resilience Byzantine agents, trust, degradation, switching
agent A continue
agent B continue
agent C continue
agent D continue lies A=continue B=evacuate C=evacuate
trust graph=complete n=6 x0=0.1,0.4,0.2,0.9,0.5,0 liar=6 eps=0.1 steps=200
degrade graph=star n=5 order=2,1
switched A1=-0.1,1,-10,-0.1 A2=-0.1,10,-1,-0.1 tau=0.05,3`,
};
