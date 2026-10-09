/** Build a runnable group model's scene using the same initial conditions as its analysis. */
import { Station, ItemType } from '../core/items/item';
import { MobileRobot, MapItem, ZoneItem } from '../mobile/items';
import { MrsKind } from './model';
import { parseConsensus, parseSwarm, parseCoverage, parseSafety, parseGridMapf, parseWarehouse } from './dsl';
import { parseMission } from './dsl_bridge';
import { coverageSetup, safetyInstance, gridInstance } from './an_practicum';
import { buildWarehouseScene } from './runtime';
import { Rng, Vec2 } from './rng';
import { MRS_EXAMPLES, FLEET_SUPERVISOR, FLEET_MODES } from './examples';
import { addControlModel, controlModels } from '../ctl/model';

export function buildGroupScene(station: Station, kind: MrsKind, source: string): MobileRobot[] {
  const existing = station.itemsOfType<MobileRobot>(ItemType.MOBILE_ROBOT);
  let names: string[] = [], points: Vec2[] = [], size = 100, speed = 500;
  const add = (name: string, at: Vec2): MobileRobot => {
    const old = existing.find((r) => r.name === name);
    if (old) return old;
    const r = station.addChild(new MobileRobot(name));
    r.kin.footprint = [size, size * 0.8, size * 0.6];
    r.kin.wheelBase = size * 0.6; r.kin.track = size * 0.6; r.kin.wheelRadius = size * 0.15;
    r.kin.maxSpeed = speed;
    r.setPose2D(at[0] * 1000, at[1] * 1000, 0);
    r.home = { x: r.state.x, y: r.state.y, theta: 0 };
    return r;
  };
  switch (kind) {
    case 'consensus': {
      const d = parseConsensus(source), rng = new Rng(d.seed);
      names = d.robots.map((r) => r.name);
      points = d.robots.map((r, i) => r.at ?? (d.x0 ? [d.x0[i], 0] : [rng.uniform(-d.box / 2, d.box / 2), rng.uniform(-d.box / 2, d.box / 2)]));
      break;
    }
    case 'swarm': {
      const d = parseSwarm(source), rng = new Rng(d.seed);
      if (!['boids', 'vicsek', 'pso'].includes(d.model)) throw new Error(`Swarm model ${d.model} is analysis-only`);
      names = Array.from({ length: d.n }, (_, i) => d.robotNames[i] ?? `r${i + 1}`);
      points = names.map(() => d.model === 'pso' ? [rng.uniform(...d.range), rng.uniform(...d.range)] : [rng.uniform(-d.box / 2, d.box / 2), rng.uniform(-d.box / 2, d.box / 2)]);
      size = Math.max(20, Math.min(d.rSep, d.dMin) * 400); speed = d.vmax * 1000;
      break;
    }
    case 'coverage': {
      const setup = coverageSetup(parseCoverage(source)); names = setup.names; points = setup.p0; break;
    }
    case 'safety': {
      const d = parseSafety(source), setup = safetyInstance(d); names = setup.names; points = setup.p;
      size = d.dSafe * 500; speed = d.vmax * 1000; break;
    }
    case 'gridmapf': {
      const d = parseGridMapf(source), setup = gridInstance(d); names = setup.names;
      points = setup.starts.map(([row, col]) => [(col + 0.5) * d.cellSize, -(row + 0.5) * d.cellSize]);
      size = d.cellSize * 600;
      if (!station.find('Group grid map', ItemType.MAP)) {
        const map = station.addChild(new MapItem('Group grid map')), res = d.cellSize * 1000;
        map.resize(setup.grid[0].length, setup.grid.length, res, 0, -setup.grid.length * res);
        setup.grid.forEach((row, i) => row.forEach((blocked, j) => { if (blocked) map.fillRect(j * res, -(i + 1) * res, (j + 1) * res - 1, -i * res - 1); }));
      }
      break;
    }
    case 'mission': {
      const d = parseMission(source); names = d.robots.map((r) => r.name); points = d.robots.map((r) => r.at);
      size = d.safety.dSafe * 500; speed = d.vmax * 1000;
      const zone = (name: string, at: Vec2) => {
        if (station.itemsOfType<ZoneItem>(ItemType.ZONE).some((z) => z.name === name)) return;
        const z = station.addChild(new ZoneItem(name)), x = at[0] * 1000, y = at[1] * 1000;
        z.polygon = [[x - 300, y - 300], [x + 300, y - 300], [x + 300, y + 300], [x - 300, y + 300]];
      };
      for (const phase of d.phases) {
        if (phase.zone && phase.at) zone(phase.zone, phase.at);
        if (phase.kind === 'allocate') phase.targetNames.forEach((name, i) => { if (phase.targets[i]) zone(name, phase.targets[i]); });
      }
      if (MRS_EXAMPLES.some((e) => e.id === 'arch-station' && e.source === source)) {
        if (!controlModels(station).some((m) => m.name === d.supervisor)) addControlModel(station, 'des', d.supervisor!, FLEET_SUPERVISOR);
        if (!controlModels(station).some((m) => m.name === d.modes)) addControlModel(station, 'hybrid', d.modes!, FLEET_MODES);
      }
      // Custom supervisor references remain requirements; only the supplied demo's dependencies are added.
      break;
    }
    case 'warehouse': {
      const d = parseWarehouse(source);
      if (!station.find('Warehouse map', ItemType.MAP)) return buildWarehouseScene(station, d.cfg, d.stationKinds).robots;
      names = d.cfg.robots.names;
      // Existing warehouse scenes can be extended with the missing named robots by the runtime.
      if (!names.every((name) => existing.some((r) => r.name === name))) throw new Error('Warehouse scene is incomplete; rebuild it before running');
      return names.map((name) => existing.find((r) => r.name === name)!);
    }
    default: throw new Error(`Model ${kind} is analysis-only`);
  }
  return names.map((name, i) => add(name, points[i]));
}
