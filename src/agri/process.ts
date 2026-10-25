/**
 * Agricultural process models executed by the FleetManager while robots work their rows (registered at import, so
 * every FleetManager — including the one the app creates for a loaded station — runs them):
 * - harvest / thin: arm-side work-path orientation, base stops placed so the fruit are within the arm's real reach,
 *   picking time per fruit (the base stands still), fruit removed one by one at their pick time, bin capacity with
 *   unload trips; thinning removes unripe fruit above the target crop load and books no yield;
 * - spray / irrigate: dose per metre from the canopy volume (or litres per plant), tank / water capacity with refill
 *   trips, plants record the applied dose, spraying cures diseased plants;
 * - mow / weed: swath coverage (area worked, coverage of the field's inter-rows);
 * - prune / pollinate: workload per plant (time from canopy volume), plants stamped as done;
 * - scout: observations of plant health (noisy) stored on the mission, disease detection, optional follow-up tasks;
 * - transport: load at the location, deliver to `meta.dropoff`, unload.
 * Also provides the orchard alleys as traffic segments.
 */
import { Station, ItemType } from '../core/items/item';
import { MobileRobot, ZoneItem } from '../mobile/items';
import { FleetManager, FleetTask, registerTaskProcess, registerSegmentProvider, TrafficSegment, WorkDirective, fleetPlugins } from '../fleet/fleet';
import { FieldItem, CropRow, MissionItem } from './items';
import { fruitWorldPositions } from './orchard';
import { armOf, armMount, armReach } from './harvest';
import { plantWorld } from './collision_world';
import { Robot } from '../core/items/robot';
import { poseFromZ, multiply, invert, getPos, normalize } from '../core/math/pose';

// -- helpers -------------------------------------------------------------------------------------------------------

function rowOf(fm: FleetManager, task: FleetTask): CropRow | null {
  const r = task.meta?.rowId ? fm.station.findById(task.meta.rowId) : null;
  return r instanceof CropRow ? r : null;
}
function missionOf(fm: FleetManager, task: FleetTask): MissionItem | null {
  const m = task.meta?.missionId ? fm.station.findById(task.meta.missionId) : null;
  return m instanceof MissionItem ? m : null;
}
/** Row direction in the station frame. */
function rowDirWorld(row: CropRow): [number, number] {
  const [dx, dy] = row.direction();
  const a = row.parent ? row.parent.poseAbs() : null;
  return a ? [a[0] * dx + a[4] * dy, a[1] * dx + a[5] * dy] : [dx, dy];
}
function workDir(task: FleetTask): [number, number] {
  const w = task.workPath!;
  const dx = w[w.length - 1][0] - w[0][0], dy = w[w.length - 1][1] - w[0][1], l = Math.hypot(dx, dy) || 1;
  return [dx / l, dy / l];
}
function settingsOf(fm: FleetManager, task: FleetTask): Record<string, any> { return missionOf(fm, task)?.settings ?? task.meta ?? {}; }
function rt(task: FleetTask): Record<string, any> { return (task.meta._proc ??= {}); }
/** Unloading / refill location: zone of the kind, else charging zone, else the robot's home. */
function serviceLocation(fm: FleetManager, robot: MobileRobot, kinds: string[]): [number, number] | null {
  const zones = fm.station.itemsOfType<ZoneItem>(ItemType.ZONE);
  for (const k of kinds) {
    const z = zones.find((q) => q.kind === k);
    if (z) { const [x, y] = z.centroid(); const a = z.poseAbs(); return [a[0] * x + a[4] * y + a[12], a[1] * x + a[5] * y + a[13]]; }
  }
  return robot.home ? [robot.home.x, robot.home.y] : null;
}
function resource(robot: MobileRobot, name: string, capacity: number, unit: string, full = true) {
  return (robot.resources[name] ??= { capacity, level: full ? capacity : 0, unit });
}

/** Orient a row work path so that the carrier's arm faces the row (fixes the arm facing the neighbour row). */
export function orientForArm(fm: FleetManager, task: FleetTask, robot: MobileRobot): void {
  const row = rowOf(fm, task);
  const arm = armOf(robot);
  if (!row || !arm || !task.workPath || task.workPath.length < 2 || !task.meta.side) return;
  const sA = armMount(arm).side;
  const d = rowDirWorld(row);
  const w = workDir(task);
  // travelling along +d with the line on the row's side s, the row is on the robot's right for s = +1
  const want = -task.meta.side * sA; // +1: along the row direction
  if ((w[0] * d[0] + w[1] * d[1]) * want < 0) {
    task.workPath = [...task.workPath].reverse();
    task.location = [task.workPath[0][0], task.workPath[0][1]];
    task.goalHeading = Math.atan2(task.workPath[1][1] - task.workPath[0][1], task.workPath[1][0] - task.workPath[0][0]) * 180 / Math.PI;
  }
}

// -- harvest / thin --------------------------------------------------------------------------------------------------

interface Stop { s: number; fruit: number[][] }

/** Place base stops along the work path so that the fruit of the row side are inside the arm's reach. */
export function planStops(fm: FleetManager, task: FleetTask, robot: MobileRobot, select: (f: { ripe: number; picked: boolean }, plantIdx: number, fruitIdx: number) => boolean): { stops: Stop[]; unreachable: number } {
  const row = rowOf(fm, task)!;
  const arm = armOf(robot);
  const w = task.workPath!;
  const a = w[0], dir = workDir(task);
  const nrm = [-dir[1], dir[0]]; // robot left
  const mount = arm ? armMount(arm) : { x: 0, y: 0, z: robot.kin.footprint[2], side: 1 as const };
  const R = (settingsOf(fm, task).reach as number | undefined) ?? (arm ? armReach(arm) * 0.9 : 1200);
  const ground = row.parent ? row.parent.poseAbs()[14] : 0;
  const zBase = robot.poseAbs()[14] - ground + mount.z;
  const items: Array<{ s: number; half: number; ref: number[] }> = [];
  let unreachable = 0;
  const all = fruitWorldPositions(row, false);
  let k = 0;
  row.plants.forEach((pl, pi) => pl.fruit.forEach((f, fi) => {
    const p = all[k++];
    if (!select(f, pi, fi)) return;
    if (task.meta.side && p.side !== task.meta.side) return;
    const s = (p.p[0] - a[0]) * dir[0] + (p.p[1] - a[1]) * dir[1];
    const lat = (p.p[0] - a[0]) * nrm[0] + (p.p[1] - a[1]) * nrm[1];
    const dz = p.p[2] - ground - zBase;
    const h2 = R * R - (lat - mount.y) * (lat - mount.y) - dz * dz;
    if (h2 <= 0 || Math.sign(lat - 0) !== Math.sign(mount.y || lat)) { unreachable++; return; }
    items.push({ s, half: Math.sqrt(h2), ref: [pi, fi] });
  }));
  items.sort((x, y) => x.s - y.s);
  const len = Math.hypot(w[w.length - 1][0] - a[0], w[w.length - 1][1] - a[1]);
  const stops: Stop[] = [];
  const done = new Uint8Array(items.length);
  for (let i = 0; i < items.length; i++) {
    if (done[i]) continue;
    const sArm = items[i].s + items[i].half * 0.9;
    const s = Math.max(0, Math.min(len, sArm - mount.x));
    const armAt = s + mount.x;
    const st: Stop = { s, fruit: [] };
    for (let j = i; j < items.length; j++) {
      if (items[j].s - armAt > R) break;
      if (!done[j] && Math.abs(items[j].s - armAt) <= items[j].half) { done[j] = 1; st.fruit.push(items[j].ref); }
    }
    if (!done[i]) { done[i] = 1; unreachable++; }
    if (st.fruit.length) stops.push(st);
  }
  return { stops, unreachable };
}

/** Optional arm animation: point the arm at a fruit (one IK solve per pick). */
export const harvestAnimation = { enabled: false };

function pointArm(arm: Robot, p: [number, number, number]) {
  const base = arm.poseAbs();
  const bp = getPos(base);
  const dir = normalize([p[0] - bp[0], p[1] - bp[1], 0]);
  const r = arm.solveIK(multiply(invert(base), poseFromZ(p, dir, [0, 0, -1])), { seed: arm.joints(), restarts: 0, maxIterations: 40 });
  if (r.ok) arm.setJoints(r.joints);
}

function pickProcess(kind: 'harvest' | 'thin') {
  return {
    onAssign: (fm: FleetManager, task: FleetTask, robot: MobileRobot) => orientForArm(fm, task, robot),
    onWorkStart: (fm: FleetManager, task: FleetTask, robot: MobileRobot) => {
      const row = rowOf(fm, task);
      if (!row) return;
      const st = settingsOf(fm, task);
      const p = rt(task);
      if (kind === 'harvest') {
        const r = planStops(fm, task, robot, (f) => f.ripe >= 0.5 && !f.picked);
        p.stops = r.stops; p.unreachable = r.unreachable;
      } else {
        // thinning: unripe fruit above the target load per plant
        const keep = st.keepFruitPerPlant ?? Math.round((row.parent as FieldItem | null)?.crop.fruitPerPlant ?? 50) * 0.6;
        const excess = new Set<string>();
        row.plants.forEach((pl, pi) => {
          const live = pl.fruit.map((f, fi) => ({ f, fi })).filter((x) => !x.f.picked);
          const n = live.length - keep;
          if (n <= 0) return;
          live.filter((x) => x.f.ripe < 0.5).sort((x, y) => x.f.ripe - y.f.ripe).slice(0, n).forEach((x) => excess.add(`${pi}:${x.fi}`));
        });
        const r = planStops(fm, task, robot, (_f, pi, fi) => excess.has(`${pi}:${fi}`));
        p.stops = r.stops; p.unreachable = r.unreachable;
      }
      p.stopIdx = 0; p.picks = [];
      fm.bookStat(kind, 'unreachable', p.unreachable);
    },
    onWork: (fm: FleetManager, task: FleetTask, robot: MobileRobot, s: number, _dt: number, holding: boolean): WorkDirective | void => {
      const row = rowOf(fm, task);
      if (!row) return;
      const p = rt(task);
      const st = settingsOf(fm, task);
      const spf = st.secondsPerFruit ?? (kind === 'thin' ? 2 : 6);
      const fruitKg = task.meta.fruitKg ?? 0.18;
      // fruit removed at their pick time
      const arm = armOf(robot);
      while (p.picks?.length && p.picks[0].t <= fm.time) {
        const pk = p.picks.shift();
        const f = row.plants[pk.ref[0]]?.fruit[pk.ref[1]];
        if (!f || f.picked) continue;
        f.picked = true;
        if (kind === 'thin') (f as any).thinned = true;
        if (kind === 'harvest') {
          fm.bookYield({ fruit: 1, kg: fruitKg });
          task.meta.harvestedKg = (task.meta.harvestedKg ?? 0) + fruitKg;
          task.meta.harvested = (task.meta.harvested ?? 0) + 1;
          const bin = resource(robot, 'bin', st.binCapacityKg ?? 400, 'kg', false);
          bin.level += fruitKg;
        } else fm.bookStat('thin', 'fruitRemoved', 1);
        if (harvestAnimation.enabled && arm) { const w = fruitWorldPositions(row, false); let k = 0; for (let i = 0; i < pk.ref[0]; i++) k += row.plants[i].fruit.length; const fw = w[k + pk.ref[1]]; if (fw) pointArm(arm, fw.p); }
        row.notify('plants');
      }
      if (holding) return;
      if (kind === 'harvest') {
        const bin = robot.resources.bin;
        if (bin && bin.level >= bin.capacity) {
          const loc = serviceLocation(fm, robot, ['unloading', 'loading', 'charging']);
          if (loc) return { service: { kind: 'unload', location: loc, duration: st.unloadSeconds ?? 90 } };
        }
      }
      const stops: Stop[] = p.stops ?? [];
      if (p.stopIdx < stops.length && s >= stops[p.stopIdx].s - 25) {
        const stop = stops[p.stopIdx++];
        const settle = st.settleSeconds ?? 2;
        stop.fruit.forEach((ref, i) => p.picks.push({ t: fm.time + settle + (i + 1) * spf, ref }));
        return { hold: settle + stop.fruit.length * spf + 0.05 };
      }
    },
    onService: (fm: FleetManager, _task: FleetTask, robot: MobileRobot, k: string) => { if (k === 'unload' && robot.resources.bin) { fm.bookStat('harvest', 'unloadedKg', robot.resources.bin.level); robot.resources.bin.level = 0; fm.bookStat('harvest', 'unloads', 1); } },
    onDone: (fm: FleetManager, task: FleetTask) => { const row = rowOf(fm, task); row?.notify('plants'); (row?.parent as FieldItem | null)?.notify('rows'); return {}; },
  };
}
registerTaskProcess('harvest', pickProcess('harvest'));
registerTaskProcess('thin', pickProcess('thin'));

// -- spray / irrigate ------------------------------------------------------------------------------------------------

function plantsAlong(fm: FleetManager, task: FleetTask, s0: number, s1: number): Array<{ row: CropRow; idx: number }> {
  const row = rowOf(fm, task);
  if (!row || !task.workPath) return [];
  const a = task.workPath[0], dir = workDir(task);
  const out: Array<{ row: CropRow; idx: number }> = [];
  row.plants.forEach((pl, i) => { const [x, y] = plantWorld(row, pl); const s = (x - a[0]) * dir[0] + (y - a[1]) * dir[1]; if (s >= s0 && s < s1) out.push({ row, idx: i }); });
  return out;
}

function liquidProcess(kind: 'spray' | 'irrigate') {
  const res = kind === 'spray' ? 'tank' : 'water';
  return {
    onWorkStart: (_fm: FleetManager, task: FleetTask) => { rt(task).s = 0; },
    onWork: (fm: FleetManager, task: FleetTask, robot: MobileRobot, s: number): WorkDirective | void => {
      const row = rowOf(fm, task);
      const st = settingsOf(fm, task);
      const p = rt(task);
      const tank = resource(robot, res, st.tankL ?? (kind === 'spray' ? 600 : 1000), 'L');
      const s0 = p.s ?? 0;
      if (s <= s0) return;
      const crop = (row?.parent as FieldItem | null)?.crop;
      let litres = 0;
      const plants = plantsAlong(fm, task, s0, s);
      if (kind === 'spray') {
        // tree-row-volume dosing: litres per metre of row side = canopy height x width x dose per m³
        const lpm = st.doseLPerM ?? ((crop ? (crop.canopyHeight / 1000) * (crop.canopyWidth / 1000) : 2) * (st.doseLPerM3 ?? 0.07));
        litres = lpm * ((s - s0) / 1000);
        for (const { row: r, idx } of plants) { const pl = r.plants[idx] as any; pl.sprayedL = (pl.sprayedL ?? 0) + lpm * (crop ? crop.plantSpacing / 1000 : 1); if (pl.health < 0.7) { pl.health = Math.min(1, pl.health + 0.3); fm.bookStat('spray', 'plantsTreated', 1); } }
      } else {
        const lpp = st.litresPerPlant ?? 4;
        litres = lpp * plants.length;
        for (const { row: r, idx } of plants) { const pl = r.plants[idx] as any; pl.waterL = (pl.waterL ?? 0) + lpp; }
      }
      if (litres > tank.level) {
        // tank empty part-way: apply what is left, go refill, come back here
        fm.bookStat(kind, 'litres', tank.level);
        tank.level = 0;
        p.s = s;
        const loc = serviceLocation(fm, robot, ['loading', 'charging']);
        if (loc) return { service: { kind: 'refill', location: loc, duration: tank.capacity / (st.refillLps ?? 5) } };
        return;
      }
      tank.level -= litres;
      fm.bookStat(kind, 'litres', litres);
      p.s = s;
    },
    onService: (fm: FleetManager, _task: FleetTask, robot: MobileRobot) => { const t = robot.resources[res]; if (t) t.level = t.capacity; fm.bookStat(kind, 'refills', 1); },
    onDone: (_fm: FleetManager, task: FleetTask) => ({ areaM2: task.workPath ? (pathLength2(task.workPath) / 1000) * (task.meta.swathM ?? 2) : 0 }),
  };
}
registerTaskProcess('spray', liquidProcess('spray'));
registerTaskProcess('irrigate', liquidProcess('irrigate'));

function pathLength2(w: number[][]) { let l = 0; for (let i = 1; i < w.length; i++) l += Math.hypot(w[i][0] - w[i - 1][0], w[i][1] - w[i - 1][1]); return l; }

// -- mow / weed ------------------------------------------------------------------------------------------------------

function coverageProcess(kind: 'mow' | 'weed') {
  return {
    onWorkStart: (_fm: FleetManager, task: FleetTask) => { rt(task).s = 0; },
    onWork: (fm: FleetManager, task: FleetTask, _robot: MobileRobot, s: number): WorkDirective | void => {
      const p = rt(task);
      if (s > (p.s ?? 0)) { const a = ((s - p.s) / 1000) * (task.meta.swathM ?? 1.5); fm.bookStat(kind, 'areaM2', a); fm.bookYield({ areaM2: a }); p.s = s; }
      if (kind === 'weed') {
        // weeding stops at each plant position for the tool cycle
        const st = settingsOf(fm, task);
        const spp = st.secondsPerPlant ?? 0;
        if (spp > 0) {
          const pl = plantsAlong(fm, task, p.last ?? -1, s);
          if (pl.length) { p.last = s; fm.bookStat('weed', 'plantsWeeded', pl.length); return { hold: spp * pl.length }; }
        }
      }
    },
    onDone: (fm: FleetManager, task: FleetTask) => {
      const row = rowOf(fm, task);
      const field = row?.parent as FieldItem | null;
      if (field) { const inter = field.rows().reduce((a, r) => a + (r.length() / 1000) * (field.crop.rowSpacing / 1000), 0); fm.bookStat(kind, 'fieldInterRowM2', 0); if (inter > 0) fm.processStats[kind].coverage = (fm.processStats[kind].areaM2 ?? 0) / inter; }
      return {};
    },
  };
}
registerTaskProcess('mow', coverageProcess('mow'));
registerTaskProcess('weed', coverageProcess('weed'));

// -- prune / pollinate -------------------------------------------------------------------------------------------------

function perPlantProcess(kind: 'prune' | 'pollinate') {
  return {
    onAssign: (fm: FleetManager, task: FleetTask, robot: MobileRobot) => orientForArm(fm, task, robot),
    onWorkStart: (_fm: FleetManager, task: FleetTask) => { rt(task).last = -1e9; rt(task).done = 0; },
    onWork: (fm: FleetManager, task: FleetTask, _robot: MobileRobot, s: number, _dt: number, holding: boolean): WorkDirective | void => {
      if (holding) return;
      const p = rt(task);
      const row = rowOf(fm, task);
      const crop = (row?.parent as FieldItem | null)?.crop;
      const st = settingsOf(fm, task);
      const plants = plantsAlong(fm, task, p.last, s + 1);
      if (!plants.length) return;
      p.last = s + 1;
      let t = 0;
      for (const { row: r, idx } of plants) {
        const pl = r.plants[idx] as any;
        if (kind === 'prune' && pl.prunedAt !== undefined) continue;
        // workload grows with the canopy volume (m³): ~25 s/m³ pruning, ~4 s/m³ pollination
        const vol = (pl.height / 1000) * Math.PI * (pl.width / 2000) ** 2;
        t += st.secondsPerPlant ?? vol * (kind === 'prune' ? 25 : 4);
        if (kind === 'prune') pl.prunedAt = fm.time; else pl.pollinatedAt = fm.time;
        fm.bookStat(kind, 'plants', 1);
      }
      void crop;
      if (t > 0) { fm.bookStat(kind, 'seconds', t); return { hold: t }; }
    },
  };
}
registerTaskProcess('prune', perPlantProcess('prune'));
registerTaskProcess('pollinate', perPlantProcess('pollinate'));

// -- scout ----------------------------------------------------------------------------------------------------------------

export interface ScoutObservation { t: number; rowId: string; plant: number; health: number; diseased: boolean; robot: string }

let scoutSeed = 99;
const scoutRnd = () => { scoutSeed = (scoutSeed * 16807) % 2147483647; return scoutSeed / 2147483647; };

registerTaskProcess('scout', {
  onWorkStart: (_fm, task) => { rt(task).last = -1e9; },
  onWork: (fm, task, robot, s) => {
    const p = rt(task);
    const range = 1500;
    const plants = plantsAlong(fm, task, p.last, s + range);
    if (!plants.length) return;
    p.last = s + range;
    const mission = missionOf(fm, task);
    const st = settingsOf(fm, task);
    const thr = st.diseaseThreshold ?? 0.6;
    for (const { row, idx } of plants) {
      const pl = row.plants[idx];
      const noise = (scoutRnd() + scoutRnd() + scoutRnd() - 1.5) * 0.04;
      const h = Math.max(0, Math.min(1, pl.health + noise));
      const obs: ScoutObservation = { t: fm.time, rowId: row.id, plant: idx, health: +h.toFixed(3), diseased: h < thr, robot: robot.name };
      if (mission) mission.observations.push(obs);
      fm.bookStat('scout', 'observations', 1);
      if (obs.diseased) fm.bookStat('scout', 'diseased', 1);
    }
  },
  onDone: (fm, task) => {
    const mission = missionOf(fm, task);
    const st = settingsOf(fm, task);
    if (!mission || !st.followUp) return {};
    // follow-up tasks: one per row with diseased plants (e.g. spray), created once
    const row = rowOf(fm, task);
    if (!row || !task.workPath) return {};
    const sick = mission.observations.filter((o) => o.rowId === row.id && o.diseased);
    const key = `${row.id}:${st.followUp}`;
    if (!sick.length || mission.followUps.includes(key)) return {};
    mission.followUps.push(key);
    fm.addTask({ type: st.followUp, location: [task.workPath[0][0], task.workPath[0][1]], workPath: task.workPath.map((q) => [...q]), workSpeed: st.followUpSpeed ?? 800, requiredCapabilities: [st.followUp], segments: task.segments, meta: { missionId: mission.id, rowId: row.id, side: task.meta.side, swathM: task.meta.swathM, followUpOf: task.id, diseasedPlants: sick.length } });
    return {};
  },
});

// -- transport -------------------------------------------------------------------------------------------------------------

registerTaskProcess('transport', {
  onWork: (fm, task, robot, _s, _dt, holding) => {
    if (holding || !task.meta.dropoff) return;
    const cargo = resource(robot, 'cargo', robot.payloadKg, 'kg', false);
    cargo.level = Math.min(cargo.capacity, task.payloadKg ?? 0);
    return { service: { kind: 'deliver', location: task.meta.dropoff as [number, number], duration: task.meta.unloadSeconds ?? 30 } };
  },
  onService: (fm, task, robot, k) => { if (k === 'deliver') { fm.bookStat('transport', 'deliveredKg', robot.resources.cargo?.level ?? task.payloadKg ?? 0); fm.bookStat('transport', 'deliveries', 1); if (robot.resources.cargo) robot.resources.cargo.level = 0; } },
});

// -- traffic segments: orchard alleys --------------------------------------------------------------------------------------

/** Alley polygons (station frame) between consecutive row lines, plus the outer alleys. Ids match mission task segments. */
export function alleySegments(station: Station): TrafficSegment[] {
  const out: TrafficSegment[] = [];
  for (const field of station.itemsOfType<FieldItem>(ItemType.FIELD)) {
    const rows = field.rows();
    if (!rows.length) continue;
    const [dx, dy] = rows[0].direction();
    const nx = -dy, ny = dx;
    const a = field.poseAbs();
    const toW = (s: number, o: number) => { const x = s * dx + o * nx, y = s * dy + o * ny; return [a[0] * x + a[4] * y + a[12], a[1] * x + a[5] * y + a[13]]; };
    const byIdx = new Map<number, { o: number; s0: number; s1: number }>();
    for (const r of rows) {
      const o = r.start[0] * nx + r.start[1] * ny;
      const s0 = Math.min(r.start[0] * dx + r.start[1] * dy, r.end[0] * dx + r.end[1] * dy), s1 = Math.max(r.start[0] * dx + r.start[1] * dy, r.end[0] * dx + r.end[1] * dy);
      const e = byIdx.get(r.index);
      byIdx.set(r.index, e ? { o, s0: Math.min(e.s0, s0), s1: Math.max(e.s1, s1) } : { o, s0, s1 });
    }
    const idx = [...byIdx.keys()].sort((p, q) => p - q);
    const sp = field.crop.rowSpacing;
    for (let i = -1; i < idx.length; i++) {
      const lo = i >= 0 ? byIdx.get(idx[i])! : null, hi = i + 1 < idx.length ? byIdx.get(idx[i + 1])! : null;
      const o0 = lo ? lo.o : hi!.o - sp, o1 = hi ? hi.o : lo!.o + sp;
      const s0 = Math.min(lo?.s0 ?? Infinity, hi?.s0 ?? Infinity), s1 = Math.max(lo?.s1 ?? -Infinity, hi?.s1 ?? -Infinity);
      const id = `${field.id}:alley${i >= 0 ? idx[i] : (hi ? idx[0] - 1 : -1)}`;
      out.push({ id, polygon: [toW(s0, o0), toW(s1, o0), toW(s1, o1), toW(s0, o1)], capacity: 1 });
    }
  }
  return out;
}
registerSegmentProvider('agri.alleys', alleySegments);

/** Lower the health of plants within `radius` of a point (disease outbreak that scouting can discover). */
export function infectPlants(station: Station, center: [number, number], radius: number, severity = 0.5): number {
  let n = 0;
  for (const row of station.itemsOfType<CropRow>(ItemType.CROP_ROW)) row.plants.forEach((pl) => {
    const [x, y] = plantWorld(row, pl);
    if (Math.hypot(x - center[0], y - center[1]) <= radius) { pl.health = Math.max(0, pl.health - severity); n++; }
  });
  return n;
}

fleetPlugins.push((fm) => { fm.logMsg(`agri process models active (${['harvest', 'thin', 'spray', 'irrigate', 'mow', 'weed', 'prune', 'pollinate', 'scout', 'transport'].join(', ')})`); });
