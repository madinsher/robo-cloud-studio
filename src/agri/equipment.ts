/**
 * Modular equipment for agricultural platforms (MURKA agromarca model): equipment modules with a mount slot, mass,
 * continuous power ("entropy" ε of the module), capabilities, resources (spray tank, fruit crate) and per-action energy;
 * recipes = equipment sets per role; depot zones (kind 'depot', capacity = slots, param `reconfigSeconds`, default
 * 180 s) where the fleet manager swaps equipment when no robot has the capabilities a task needs.
 *
 * Energy (MURKA): E(t+Δt) = E(t) − ε_total·Δt − c_action, ε_total = ε_base + Σ ε_module, c_drive = c_base·(1 + W_eq/W_max):
 * the platform's idle power + Σ module power (battery.equipmentW), traction cost scaled by 1 + equipment mass / payload
 * (battery.driveFactor), action costs debited per pick / litre / cut / image.
 */
import { ItemType } from '../core/items/item';
import { MobileRobot, ZoneItem } from '../mobile/items';
import { FleetManager, registerReconfigurer } from '../fleet/fleet';
import '../fleet/murka_alloc'; // registers the MuRKA contract-net allocator `murka`

export type MountSlot = 'top' | 'front' | 'rear' | 'cargo' | 'gimbal';

export interface EquipmentSpec {
  id: string;
  name: string;
  slot: MountSlot;
  massKg: number;
  /** Continuous power draw while mounted (W) — the module's entropy ε. */
  powerW: number;
  capabilities: string[];
  /** Resources the module brings (tank litres, crate kg); tanks start full, crates empty. */
  resources?: Record<string, { capacity: number; unit: string; startFull?: boolean }>;
  /** Energy per action (Wh): pick, litre, cut, image, plant. */
  actionWh?: Record<string, number>;
  /** Platform kind the module fits. */
  platform: 'ground' | 'aerial' | 'any';
}

export const EQUIPMENT: Record<string, EquipmentSpec> = {
  harvester_arm: { id: 'harvester_arm', name: 'Harvesting manipulator (4-DOF + soft gripper)', slot: 'top', massKg: 45, powerW: 180, capabilities: ['harvest'], actionWh: { pick: 0.05 }, platform: 'ground' },
  fruit_crate: { id: 'fruit_crate', name: 'Fruit crate', slot: 'cargo', massKg: 15, powerW: 0, capabilities: [], resources: { bin: { capacity: 300, unit: 'kg' } }, platform: 'ground' },
  sprayer: { id: 'sprayer', name: 'Air-blast sprayer (400 L)', slot: 'top', massKg: 120, powerW: 350, capabilities: ['spray'], resources: { tank: { capacity: 400, unit: 'L', startFull: true } }, actionWh: { litre: 0.02 }, platform: 'ground' },
  scout_mast: { id: 'scout_mast', name: 'Scouting mast (RGB-D + multispectral)', slot: 'top', massKg: 12, powerW: 60, capabilities: ['scout'], actionWh: { image: 0.002 }, platform: 'ground' },
  pruner: { id: 'pruner', name: 'Pruning manipulator with shears', slot: 'top', massKg: 40, powerW: 200, capabilities: ['prune', 'thin'], actionWh: { cut: 0.03 }, platform: 'ground' },
  mower_deck: { id: 'mower_deck', name: 'Inter-row mower deck', slot: 'front', massKg: 80, powerW: 900, capabilities: ['mow', 'weed'], platform: 'ground' },
  cargo_bed: { id: 'cargo_bed', name: 'Cargo bed', slot: 'cargo', massKg: 25, powerW: 0, capabilities: ['transport'], resources: { cargo: { capacity: 500, unit: 'kg' } }, platform: 'ground' },
  gimbal_camera: { id: 'gimbal_camera', name: 'Gimbal camera', slot: 'gimbal', massKg: 1.2, powerW: 15, capabilities: ['scout'], actionWh: { image: 0.001 }, platform: 'aerial' },
  mini_sprayer: { id: 'mini_sprayer', name: 'UAV mini sprayer (10 L)', slot: 'gimbal', massKg: 12, powerW: 120, capabilities: ['spray'], resources: { tank: { capacity: 10, unit: 'L', startFull: true } }, actionWh: { litre: 0.05 }, platform: 'aerial' },
};

/** Roles: equipment sets (ground platform UTP-G, aerial platform UTP-A). */
export const RECIPES: Record<string, string[]> = {
  harvester: ['harvester_arm', 'fruit_crate'],
  sprayer: ['sprayer'],
  scout: ['scout_mast'],
  pruner: ['pruner'],
  mower: ['mower_deck'],
  transporter: ['cargo_bed'],
  uav_scout: ['gimbal_camera'],
  uav_sprayer: ['mini_sprayer'],
};

const isAerial = (r: MobileRobot) => r.kin.drive === 'aerial';

/** Mount an equipment set on a robot: capabilities, resources, continuous power and drive-cost factor are recomputed. */
export function applyEquipment(robot: MobileRobot, ids: string[]): void {
  const specs = ids.map((id) => { const e = EQUIPMENT[id]; if (!e) throw new Error(`Unknown equipment ${id}`); return e; });
  const slots = new Set<string>();
  for (const e of specs) {
    if (slots.has(e.slot)) throw new Error(`Two modules on the ${e.slot} slot of ${robot.name}`);
    slots.add(e.slot);
    if (e.platform !== 'any' && (e.platform === 'aerial') !== isAerial(robot)) throw new Error(`${e.name} does not fit the ${robot.kin.drive} platform ${robot.name}`);
  }
  if (robot.baseCapabilities === null) {
    const old = new Set(robot.equipment.flatMap((id) => EQUIPMENT[id]?.capabilities ?? []));
    robot.baseCapabilities = robot.capabilities.filter((c) => !old.has(c));
  }
  // resources of removed modules disappear, new ones start full (tanks) or empty (crates); kept ones keep their level
  const keep = new Set(specs.flatMap((e) => Object.keys(e.resources ?? {})));
  for (const id of robot.equipment) for (const k of Object.keys(EQUIPMENT[id]?.resources ?? {})) if (!keep.has(k)) delete robot.resources[k];
  for (const e of specs) for (const [k, r] of Object.entries(e.resources ?? {})) {
    const cur = robot.resources[k];
    robot.resources[k] = { capacity: r.capacity, level: cur && robot.equipment.some((id) => EQUIPMENT[id]?.resources?.[k]) ? Math.min(cur.level, r.capacity) : r.startFull ? r.capacity : 0, unit: r.unit };
  }
  robot.equipment = [...ids];
  robot.capabilities = [...new Set([...robot.baseCapabilities, ...specs.flatMap((e) => e.capabilities)])];
  const mass = specs.reduce((a, e) => a + e.massKg, 0);
  robot.battery.equipmentW = specs.reduce((a, e) => a + e.powerW, 0);
  robot.battery.driveFactor = 1 + mass / Math.max(1, robot.payloadKg);
  robot.setParam('equipmentMassKg', mass);
}

/** Total mass of the mounted equipment (kg). */
export function equipmentMass(robot: MobileRobot): number {
  return robot.equipment.reduce((a, id) => a + (EQUIPMENT[id]?.massKg ?? 0), 0);
}

/** Debit the energy of `n` actions (pick, litre, cut, image) from the robot's battery; booked as process statistics. */
export function actionEnergy(fm: FleetManager | null, robot: MobileRobot, action: string, n = 1): number {
  const wh = robot.equipment.reduce((a, id) => a + (EQUIPMENT[id]?.actionWh?.[action] ?? 0), 0) * n;
  if (wh <= 0) return 0;
  robot.battery.levelWh = Math.max(0, robot.battery.levelWh - wh);
  fm?.bookStat('energy', `${action}Wh`, wh);
  return wh;
}

/** Smallest recipe (fewest modules, lightest) giving all `caps` on this platform kind, or null. */
export function recipeFor(caps: string[], aerial: boolean): string | null {
  let best: string | null = null, bestKey = Infinity;
  for (const [name, ids] of Object.entries(RECIPES)) {
    const specs = ids.map((id) => EQUIPMENT[id]);
    if (specs.some((e) => e.platform !== 'any' && (e.platform === 'aerial') !== aerial)) continue;
    const have = new Set(specs.flatMap((e) => e.capabilities));
    if (!caps.every((c) => have.has(c))) continue;
    const key = ids.length * 1000 + specs.reduce((a, e) => a + e.massKg, 0);
    if (key < bestKey) { bestKey = key; best = name; }
  }
  return best;
}

/** Depots of a station (zones of kind 'depot'): centre, slots and swap duration. */
export function depots(fm: FleetManager): Array<{ zone: ZoneItem; location: [number, number]; slots: number; seconds: number }> {
  return fm.station.itemsOfType<ZoneItem>(ItemType.ZONE).filter((z) => z.kind === 'depot' && z.polygon.length > 2).map((z) => {
    const p = z.worldPolygon();
    const c: [number, number] = [p.reduce((a, q) => a + q[0], 0) / p.length, p.reduce((a, q) => a + q[1], 0) / p.length];
    return { zone: z, location: c, slots: Math.max(1, z.capacity || 1), seconds: Number(z.getParam('reconfigSeconds') ?? 180) || 180 };
  });
}

// The fleet manager asks this when a robot lacks a task's capabilities: reconfigure at the nearest depot.
registerReconfigurer({
  plan(fm, robot, caps) {
    const d = depots(fm);
    if (!d.length) return null;
    const base = new Set(robot.baseCapabilities ?? robot.capabilities.filter((c) => !robot.equipment.some((id) => EQUIPMENT[id]?.capabilities.includes(c))));
    const need = caps.filter((c) => !base.has(c));
    const recipe = recipeFor(need, isAerial(robot));
    if (!recipe) return null;
    const near = d.reduce((a, b) => (Math.hypot(b.location[0] - robot.state.x, b.location[1] - robot.state.y) < Math.hypot(a.location[0] - robot.state.x, a.location[1] - robot.state.y) ? b : a));
    return { location: near.location, seconds: near.seconds, recipe, depotId: near.zone.id, capacity: near.slots };
  },
  apply(_fm, robot, recipe) { applyEquipment(robot, RECIPES[recipe] ?? []); },
});

// ---------------------------------------------------------------------------------------------------------------
// MURKA configuration-dependent energy (abstract units of agromarca article_experiments: u/s and u/m)
// ---------------------------------------------------------------------------------------------------------------

/** Energy profile of a configuration: platform entropy ε_base, Σ module entropy, drive cost per metre, mass. */
export interface EnergyProfile { baseEntropy: number; equipmentOverhead: number; driveCostPerM: number; massKg: number }

/** ε_total = ε_base + Σ ε_module. */
export function entropyRate(p: EnergyProfile): number { return p.baseEntropy + p.equipmentOverhead; }

/** c_drive = c_base·(1 + W_eq / W_max). */
export function driveCost(cBase: number, equipmentMassKg: number, maxPayloadKg: number): number { return cBase * (1 + equipmentMassKg / maxPayloadKg); }

/** One step of E(t+Δt) = E(t) − ε_total·Δt − c_action, the drive action costing c_drive per metre driven (ds). */
export function energyStep(E: number, p: EnergyProfile, dt: number, ds: number, cAction = 0): number {
  return E - entropyRate(p) * dt - p.driveCostPerM * ds - cAction;
}

/** Closed form of `energyStep` over a route of `lengthM` at constant `speed` (m/s): ε_total·L/v + c_drive·L. */
export function routeEnergy(p: EnergyProfile, lengthM: number, speed: number): { energy: number; duration: number } {
  const duration = lengthM / speed;
  return { energy: entropyRate(p) * duration + p.driveCostPerM * lengthM, duration };
}

/** Energy profile of a studio equipment set: ε from module powers (W → u/s by `wPerUnit`), c_drive from the mass ratio. */
export function profileOfEquipment(ids: string[], base: { entropy: number; cBase: number; maxPayloadKg: number; platformMassKg: number; wPerUnit?: number }): EnergyProfile {
  const specs = ids.map((id) => { const e = EQUIPMENT[id]; if (!e) throw new Error(`Unknown equipment ${id}`); return e; });
  const mass = specs.reduce((a, e) => a + e.massKg, 0);
  return { baseEntropy: base.entropy, equipmentOverhead: specs.reduce((a, e) => a + e.powerW, 0) / (base.wPerUnit ?? 10000), driveCostPerM: driveCost(base.cBase, mass, base.maxPayloadKg), massKg: base.platformMassKg + mass };
}
