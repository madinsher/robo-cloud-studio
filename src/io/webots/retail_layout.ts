/**
 * Retail layout loader (MURKA murom_world/config): products.yaml (catalogue + instances in slots) and locations.yaml
 * (slots with surface pose, approach / standoff poses, arm, ceiling; dock; home; basket slots) -> station items:
 * products placed exactly like the Webots supervisor plugin spawns them (RetailProduct PROTO when the world declares it,
 * translation = slot pose + size.z / 2 + 2 mm), Targets for slots / approach / standoff / dock / home, a charging Zone
 * around the dock pose (radius from the file), a packing Zone around the packing slots, basket Targets on the robot.
 */
import { Item, Station, SceneObject, Target, Folder, Frame, GeometryRef } from '../../core/items/item';
import { MobileRobot, ZoneItem } from '../../mobile/items';
import { Mat4, multiply, transl, rotz, identity } from '../../core/math/pose';
import { parseYaml, YamlValue } from './yaml';
import { parseNodes } from './vrml';
import type { WebotsSceneBuilder } from './scene';
import { rgbHex } from './geometry';

type Obj = { [k: string]: YamlValue };
const num = (v: YamlValue | undefined, d = 0) => (typeof v === 'number' ? v : d);
const asObj = (v: YamlValue | undefined): Obj => (v && typeof v === 'object' && !Array.isArray(v) ? (v as Obj) : {});

export interface RetailLayoutOptions {
  /** Builder of the imported world (instantiates the RetailProduct PROTO like the supervisor). */
  builder?: WebotsSceneBuilder;
  /** World file the PROTO declarations come from (scope of the IMPORTABLE EXTERNPROTO). */
  worldPath?: string;
  /** Robot receiving the basket slot Targets (default: the first MobileRobot). */
  robot?: MobileRobot | null;
}

export interface RetailLayoutResult {
  products: SceneObject[];
  slots: Target[];
  targets: Target[];
  zones: ZoneItem[];
  report: string[];
}

const pose2D = (p: Obj): Mat4 => multiply(transl(num(p.x) * 1000, num(p.y) * 1000, num(p.z) * 1000), rotz(num(p.yaw)));

/** Load products.yaml + locations.yaml (texts) into the station. */
export function loadRetailLayout(station: Station, productsYaml: string, locationsYaml: string, opts: RetailLayoutOptions = {}): RetailLayoutResult {
  const report: string[] = [];
  const prod = asObj(parseYaml(productsYaml));
  const loc = asObj(parseYaml(locationsYaml));
  const catalog = asObj(prod.catalog);
  const slots = asObj(loc.slots);
  const res: RetailLayoutResult = { products: [], slots: [], targets: [], zones: [], report };

  // locations: slots and their approach / standoff poses
  const locFolder = new Folder('locations');
  locFolder.params.frame = String(loc.frame ?? 'map');
  station.addChild(locFolder);
  const slotFrame = new Frame('slots');
  const navFrame = new Frame('approach_standoff');
  locFolder.addChild(slotFrame);
  locFolder.addChild(navFrame);
  for (const [id, raw] of Object.entries(slots)) {
    const s = asObj(raw);
    const p = asObj(s.pose);
    const t = new Target(id);
    t.params.kind = String(s.kind ?? '');
    for (const k of ['arm', 'arm_alt', 'ceiling', 'depth_in']) if (s[k] !== undefined) t.params[k] = s[k] as string | number;
    slotFrame.addChild(t);
    t.setPose(transl(num(p.x) * 1000, num(p.y) * 1000, num(p.z) * 1000));
    res.slots.push(t);
    for (const k of ['approach', 'standoff', 'approach_alt', 'standoff_alt']) {
      if (!s[k]) continue;
      const a = new Target(`${id}_${k}`);
      a.params.slot = id;
      a.params.role = k;
      navFrame.addChild(a);
      a.setPose(pose2D(asObj(s[k])));
      res.targets.push(a);
    }
  }
  for (const [name, raw] of Object.entries(asObj(loc.targets))) {
    const t = new Target(name);
    locFolder.addChild(t);
    t.setPose(pose2D(asObj(raw)));
    res.targets.push(t);
  }
  const dock = asObj(loc.dock);
  if (dock.approach) {
    for (const k of ['approach', 'standoff']) {
      if (!dock[k]) continue;
      const t = new Target(`dock_${k}`);
      locFolder.addChild(t);
      t.setPose(pose2D(asObj(dock[k])));
      res.targets.push(t);
    }
    const a = asObj(dock.approach);
    const r = num(dock.radius, 0.06) * 1000;
    const z = new ZoneItem('dock');
    z.kind = 'charging';
    z.polygon = Array.from({ length: 24 }, (_, i) => [num(a.x) * 1000 + r * Math.cos((2 * Math.PI * i) / 24), num(a.y) * 1000 + r * Math.sin((2 * Math.PI * i) / 24)]);
    z.params.radius = r;
    z.params.dockYaw = num(a.yaw);
    locFolder.addChild(z);
    res.zones.push(z);
  }
  const packing = res.slots.filter((s) => s.params.kind === 'packing');
  if (packing.length) {
    const xs = packing.map((s) => s.pose()[12]), ys = packing.map((s) => s.pose()[13]);
    const m = 250;
    const z = new ZoneItem('packing');
    z.kind = 'unloading';
    z.polygon = [[Math.min(...xs) - m, Math.min(...ys) - m], [Math.max(...xs) + m, Math.min(...ys) - m], [Math.max(...xs) + m, Math.max(...ys) + m], [Math.min(...xs) - m, Math.max(...ys) + m]];
    locFolder.addChild(z);
    res.zones.push(z);
  }
  const robot = opts.robot === undefined ? station.itemsOfType<MobileRobot>(100)[0] ?? null : opts.robot;
  const basket = asObj(loc.basket_slots);
  if (robot) for (const [name, raw] of Object.entries(basket)) {
    const b = asObj(raw);
    const t = new Target(name);
    t.params.arm = String(b.arm ?? '');
    t.params.role = 'basket';
    robot.addChild(t);
    t.setPose(transl(num(b.dx) * 1000, num(b.dy) * 1000, num(b.dz) * 1000));
    res.targets.push(t);
  } else if (Object.keys(basket).length) report.push('basket_slots: no MobileRobot to attach them to');

  // products, spawned like supervisor_plugin.__spawn_products
  const folder = new Folder('products');
  station.addChild(folder);
  const instances = Array.isArray(prod.instances) ? prod.instances : [];
  const builder = opts.builder;
  const base = opts.worldPath ?? builder?.reg.scopeOf(station.filePath ?? '')?.file ?? station.filePath ?? '';
  const viaProto = !!builder && !!builder.reg.scopeOf(base)?.externs.has('RetailProduct');
  if (!viaProto) report.push('RetailProduct PROTO not declared by the world: products built as plain boxes');
  for (const raw of instances) {
    const inst = asObj(raw);
    const name = String(inst.name), sku = String(inst.sku), slot = String(inst.slot);
    const cat = asObj(catalog[sku]);
    const size = (Array.isArray(cat.size) ? cat.size : [0.1, 0.1, 0.1]).map((v) => num(v));
    const color = (Array.isArray(cat.color) ? cat.color : [0.7, 0.7, 0.7]).map((v) => num(v));
    const sp = asObj(asObj(slots[slot]).pose);
    if (!slots[slot]) { report.push(`${name}: slot ${slot} not in locations.yaml`); continue; }
    const x = num(sp.x), y = num(sp.y), z = num(sp.z) + size[2] / 2 + 0.002;
    let item: Item | null = null;
    if (viaProto) {
      const text = `DEF ${name} RetailProduct { translation ${x.toFixed(4)} ${y.toFixed(4)} ${z.toFixed(4)} name "${name}" size ${size.map((v) => v.toFixed(3)).join(' ')} color ${color.map((v) => v.toFixed(3)).join(' ')} }`;
      const [n] = parseNodes(text, base);
      const resolved = builder!.reg.resolve(n);
      if (resolved) item = builder!.buildNode(resolved, folder, identity());
    }
    if (!item) {
      const so = new SceneObject(name);
      so.geometry = [{ primitive: { kind: 'box', size: [size[0] * 1000, size[1] * 1000, size[2] * 1000] }, color: rgbHex(color) }];
      (so as unknown as { colliders: GeometryRef[] }).colliders = [];
      folder.addChild(so);
      so.setPose(transl(x * 1000, y * 1000, z * 1000));
      item = so;
    }
    item.params.sku = sku;
    item.params.slot = slot;
    item.params.size = size.map((v) => v * 1000);
    if (cat.two_hand) item.params.twoHand = true;
    item.params.visionClass = sku;
    if (item instanceof SceneObject) res.products.push(item);
  }
  report.push(`retail layout: ${res.products.length} products, ${res.slots.length} slots, ${res.targets.length} targets, ${res.zones.length} zones`);
  return res;
}
