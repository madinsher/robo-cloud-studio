/**
 * Clean-room equivalents of the Webots standard PROTOs used by worlds (the Webots asset PROTOs are licensed for use with
 * Webots only and are not bundled). Each equivalent has the documented interface (field names and defaults) and builds
 * base nodes with the same geometry (floor plane at z = 0, arena walls outside the floor edges); textures are replaced
 * by representative colours.
 */
import type { WNode, WValue, ProtoFieldDecl } from './vrml';

export interface BuiltinProto {
  fields: ProtoFieldDecl[];
  /** Build the base-node tree for the resolved field values (`warn`: report line, e.g. a field reset like Webots does). */
  build(values: Record<string, WValue>, instance: WNode, warn?: (msg: string) => void): WNode;
  /** Appearance-like PROTOs accept any field without a warning. */
  lenient?: boolean;
}

const num = (...v: number[]): WValue => ({ k: 'num', v });
const str = (s: string): WValue => ({ k: 'str', v: [s] });
const bool = (b: boolean): WValue => ({ k: 'bool', v: [b] });
const node = (n: WNode | null): WValue => ({ k: 'node', v: n });
const nodes = (ns: WNode[]): WValue => ({ k: 'nodes', v: ns });
const N = (type: string, fields: Record<string, WValue>, meta?: Record<string, unknown>): WNode => ({ type, fields, base: `builtin:${type}`, ...(meta ? { meta } : {}) });
const fd = (type: string, name: string, def: WValue): ProtoFieldDecl => ({ kind: 'field', type, name, def });
const nums = (v: WValue | undefined, d: number[]) => (v && v.k === 'num' && v.v.length ? v.v : d);
/** Webots resets non-positive sizes of the standard PROTOs to the field default (with a console error); same here. */
const positive = (proto: string, field: string, v: number[], d: number[], warn?: (msg: string) => void): number[] => {
  if (v.every((x) => x > 0)) return v;
  warn?.(`${proto}: '${field}' must be strictly positive, reset to ${d.join(' ')} (as Webots does)`);
  return d;
};
const nodeOf = (v: WValue | undefined): WNode | null => (v ? (v.k === 'node' ? v.v : v.k === 'nodes' ? v.v[0] ?? null : null) : null);

/** Representative colours of the standard appearance PROTOs (texture-free approximation). */
const APPEARANCES: Record<string, { color: [number, number, number]; roughness?: number; metalness?: number; transparency?: number }> = {
  Parquetry: { color: [0.72, 0.53, 0.34], roughness: 0.6, metalness: 0 },
  BrushedAluminium: { color: [0.78, 0.79, 0.81], roughness: 0.3, metalness: 1 },
  BrushedSteel: { color: [0.68, 0.69, 0.7], roughness: 0.3, metalness: 1 },
  GalvanizedMetal: { color: [0.64, 0.66, 0.67], roughness: 0.4, metalness: 1 },
  MetalPipePaint: { color: [0.75, 0.2, 0.15], roughness: 0.5, metalness: 0.3 },
  PaintedWood: { color: [0.8, 0.6, 0.4], roughness: 0.6, metalness: 0 },
  MattePaint: { color: [0.8, 0.8, 0.8], roughness: 0.9, metalness: 0 },
  GlossyPaint: { color: [0.8, 0.8, 0.8], roughness: 0.2, metalness: 0 },
  RoughPolymer: { color: [0.6, 0.6, 0.6], roughness: 0.8, metalness: 0 },
  Plastic: { color: [0.85, 0.85, 0.85], roughness: 0.4, metalness: 0 },
  Rubber: { color: [0.1, 0.1, 0.1], roughness: 0.9, metalness: 0 },
  Asphalt: { color: [0.22, 0.22, 0.23], roughness: 0.95, metalness: 0 },
  Pavement: { color: [0.55, 0.54, 0.52], roughness: 0.9, metalness: 0 },
  Grass: { color: [0.3, 0.5, 0.2], roughness: 1, metalness: 0 },
  Sand: { color: [0.86, 0.76, 0.56], roughness: 1, metalness: 0 },
  DryMud: { color: [0.52, 0.42, 0.32], roughness: 1, metalness: 0 },
  Soil: { color: [0.4, 0.3, 0.2], roughness: 1, metalness: 0 },
  CorrodedMetal: { color: [0.5, 0.36, 0.26], roughness: 0.8, metalness: 0.6 },
  OldSteel: { color: [0.42, 0.42, 0.44], roughness: 0.6, metalness: 0.9 },
  ThreadMetalPlate: { color: [0.6, 0.6, 0.62], roughness: 0.4, metalness: 1 },
  Copper: { color: [0.86, 0.52, 0.32], roughness: 0.3, metalness: 1 },
  Glass: { color: [0.92, 0.96, 1], roughness: 0, metalness: 0, transparency: 0.7 },
  MarbleTiles: { color: [0.9, 0.9, 0.88], roughness: 0.3, metalness: 0 },
  CementTiles: { color: [0.7, 0.7, 0.68], roughness: 0.8, metalness: 0 },
  CeramicTiles: { color: [0.88, 0.88, 0.86], roughness: 0.3, metalness: 0 },
  Roughcast: { color: [0.85, 0.82, 0.75], roughness: 1, metalness: 0 },
  RedBricks: { color: [0.6, 0.27, 0.2], roughness: 0.9, metalness: 0 },
  Leather: { color: [0.35, 0.22, 0.14], roughness: 0.7, metalness: 0 },
  OldPlywood: { color: [0.7, 0.58, 0.42], roughness: 0.8, metalness: 0 },
  DamascusSteel: { color: [0.55, 0.55, 0.57], roughness: 0.3, metalness: 1 },
  SandyGround: { color: [0.75, 0.66, 0.5], roughness: 1, metalness: 0 },
  WireFence: { color: [0.6, 0.6, 0.6], roughness: 0.5, metalness: 1 },
  ChequeredParquetry: { color: [0.72, 0.53, 0.34], roughness: 0.6, metalness: 0 },
};

function appearanceProto(name: string): BuiltinProto {
  const a = APPEARANCES[name];
  return {
    lenient: true,
    fields: [fd('SFColor', 'colorOverride', num(1, 1, 1)), fd('SFColor', 'baseColor', num(...a.color)), fd('SFString', 'type', str('')), fd('SFNode', 'textureTransform', node(null)), fd('SFFloat', 'IBLStrength', num(1))],
    build(v, inst) {
      const co = nums(v.colorOverride, [1, 1, 1]);
      const explicitBase = inst.fields.baseColor && inst.fields.baseColor.k === 'num' ? inst.fields.baseColor.v : null;
      const c = explicitBase ?? a.color.map((x, i) => x * (co[i] ?? 1));
      const f: Record<string, WValue> = { baseColor: num(c[0], c[1], c[2]), roughness: num(a.roughness ?? 0.5), metalness: num(a.metalness ?? 0) };
      if (a.transparency) f.transparency = num(a.transparency);
      return N('PBRAppearance', f, { builtin: name });
    },
  };
}

const floorAppearanceDefault = () => node(N('Parquetry', { type: str('chequered') }));

const Floor: BuiltinProto = {
  fields: [fd('SFVec3f', 'translation', num(0, 0, 0)), fd('SFRotation', 'rotation', num(0, 0, 1, 0)), fd('SFString', 'name', str('floor')), fd('SFString', 'contactMaterial', str('default')), fd('SFVec2f', 'size', num(10, 10)), fd('SFVec2f', 'tileSize', num(0.5, 0.5)), fd('SFNode', 'appearance', floorAppearanceDefault())],
  build(v, _i, warn) {
    const size = positive('Floor', 'size', nums(v.size, [10, 10]), [10, 10], warn);
    const app = nodeOf(v.appearance);
    return N('Solid', {
      translation: v.translation, rotation: v.rotation, name: v.name, contactMaterial: v.contactMaterial, locked: bool(true),
      children: nodes([N('Shape', { appearance: node(app), geometry: node(N('Plane', { size: num(size[0], size[1]) })) })]),
      boundingObject: node(N('Plane', { size: num(size[0], size[1]) })),
    }, { builtin: 'Floor', ground: true });
  },
};

const RectangleArena: BuiltinProto = {
  fields: [fd('SFVec3f', 'translation', num(0, 0, 0)), fd('SFRotation', 'rotation', num(0, 0, 1, 0)), fd('SFString', 'name', str('rectangle arena')), fd('SFString', 'contactMaterial', str('default')), fd('SFVec2f', 'floorSize', num(1, 1)), fd('SFVec2f', 'floorTileSize', num(0.5, 0.5)), fd('SFNode', 'floorAppearance', floorAppearanceDefault()), fd('SFFloat', 'wallThickness', num(0.01)), fd('SFFloat', 'wallHeight', num(0.1)), fd('SFNode', 'wallAppearance', node(N('BrushedAluminium', {})))],
  build(v, _i, warn) {
    const [fx, fy] = positive('RectangleArena', 'floorSize', nums(v.floorSize, [1, 1]), [1, 1], warn);
    const t = positive('RectangleArena', 'wallThickness', nums(v.wallThickness, [0.01]), [0.01], warn)[0], h = positive('RectangleArena', 'wallHeight', nums(v.wallHeight, [0.1]), [0.1], warn)[0];
    const floorApp = nodeOf(v.floorAppearance), wallApp = nodeOf(v.wallAppearance);
    const children: WNode[] = [N('Shape', { appearance: node(floorApp), geometry: node(N('Plane', { size: num(fx, fy) })) })];
    const bounds: WNode[] = [N('Plane', { size: num(fx, fy) })];
    {
      const walls: Array<[number, number, number, number]> = [[fx / 2 + t / 2, 0, t, fy], [-(fx / 2 + t / 2), 0, t, fy], [0, fy / 2 + t / 2, fx + 2 * t, t], [0, -(fy / 2 + t / 2), fx + 2 * t, t]];
      for (const [x, y, sx, sy] of walls) {
        children.push(N('Pose', { translation: num(x, y, h / 2), children: nodes([N('Shape', { appearance: node(wallApp), geometry: node(N('Box', { size: num(sx, sy, h) })) })]) }));
        bounds.push(N('Pose', { translation: num(x, y, h / 2), children: nodes([N('Box', { size: num(sx, sy, h) })]) }));
      }
    }
    return N('Solid', { translation: v.translation, rotation: v.rotation, name: v.name, contactMaterial: v.contactMaterial, locked: bool(true), children: nodes(children), boundingObject: node(N('Group', { children: nodes(bounds) })) }, { builtin: 'RectangleArena', ground: true });
  },
};

/** Approximate sky colours of the background textures (zenith-ish average). */
const SKY: Record<string, [number, number, number]> = {
  mountains: [0.55, 0.68, 0.85], noon_cloudy_countryside: [0.68, 0.75, 0.84], noon_sunny_empty: [0.5, 0.7, 0.95], noon_sunny_garden: [0.5, 0.7, 0.95], noon_cloudy_empty: [0.7, 0.75, 0.8],
  noon_cloudy_mountains: [0.65, 0.72, 0.82], noon_park_empty: [0.6, 0.72, 0.88], noon_stormy_empty: [0.45, 0.48, 0.52], noon_building_overcast: [0.7, 0.72, 0.75], dawn_cloudy_empty: [0.7, 0.62, 0.6],
  dusk: [0.45, 0.4, 0.5], twilight_cloudy_empty: [0.35, 0.35, 0.45], morning_cloudy_empty: [0.7, 0.72, 0.78], factory: [0.5, 0.5, 0.5], empty_office: [0.8, 0.8, 0.8], entrance_hall: [0.75, 0.72, 0.68],
  music_hall: [0.4, 0.35, 0.3], stadium: [0.55, 0.65, 0.8], stadium_dry: [0.6, 0.65, 0.75], mars: [0.75, 0.55, 0.4],
};

const TexturedBackground: BuiltinProto = {
  fields: [fd('SFString', 'texture', str('mountains')), fd('SFFloat', 'luminosity', num(1)), fd('SFBool', 'skybox', bool(true)), fd('MFColor', 'skyColor', num(0, 0, 0))],
  build(v) {
    const tex = v.texture && v.texture.k === 'str' ? v.texture.v[0] : 'mountains';
    const skybox = !(v.skybox && v.skybox.k === 'bool' && !v.skybox.v[0]);
    const c = skybox ? SKY[tex] ?? [0.6, 0.7, 0.85] : nums(v.skyColor, [0, 0, 0]);
    return N('Background', { skyColor: num(c[0], c[1], c[2]), luminosity: v.luminosity }, { builtin: 'TexturedBackground', texture: tex });
  },
};

const TexturedBackgroundLight: BuiltinProto = {
  fields: [fd('SFString', 'texture', str('mountains')), fd('SFFloat', 'luminosity', num(1)), fd('SFBool', 'castShadows', bool(true))],
  build(v) {
    const lum = Math.max(0, nums(v.luminosity, [1])[0]);
    return N('DirectionalLight', { direction: num(0.4, -0.35, -1), intensity: num(lum), castShadows: v.castShadows, ambientIntensity: num(0.3) }, { builtin: 'TexturedBackgroundLight', texture: v.texture && v.texture.k === 'str' ? v.texture.v[0] : 'mountains' });
  },
};

const SolidBox: BuiltinProto = {
  fields: [fd('SFVec3f', 'translation', num(0, 0, 0)), fd('SFRotation', 'rotation', num(0, 0, 1, 0)), fd('SFString', 'name', str('box')), fd('SFVec3f', 'size', num(2, 2, 2)), fd('SFString', 'contactMaterial', str('default')), fd('SFNode', 'appearance', node(N('PBRAppearance', { baseColor: num(1, 1, 1), roughness: num(0.5), metalness: num(0) }))), fd('SFNode', 'physics', node(null)), fd('SFBool', 'enableBoundingObject', bool(true)), fd('SFBool', 'castShadows', bool(true))],
  build(v) {
    const s = nums(v.size, [2, 2, 2]);
    const f: Record<string, WValue> = { translation: v.translation, rotation: v.rotation, name: v.name, contactMaterial: v.contactMaterial, physics: v.physics, children: nodes([N('Shape', { appearance: node(nodeOf(v.appearance)), geometry: node(N('Box', { size: num(s[0], s[1], s[2]) })), castShadows: v.castShadows })]) };
    if (!(v.enableBoundingObject && v.enableBoundingObject.k === 'bool' && !v.enableBoundingObject.v[0])) f.boundingObject = node(N('Box', { size: num(s[0], s[1], s[2]) }));
    return N('Solid', f, { builtin: 'SolidBox' });
  },
};

const WoodenBox: BuiltinProto = {
  fields: [fd('SFVec3f', 'translation', num(0, 0, 0.3)), fd('SFRotation', 'rotation', num(0, 0, 1, 0)), fd('SFString', 'name', str('wooden box')), fd('SFVec3f', 'size', num(0.6, 0.6, 0.6)), fd('SFFloat', 'mass', num(0)), fd('SFBool', 'locked', bool(false))],
  build(v) {
    const s = nums(v.size, [0.6, 0.6, 0.6]);
    const m = nums(v.mass, [0])[0];
    return N('Solid', {
      translation: v.translation, rotation: v.rotation, name: v.name, locked: v.locked,
      children: nodes([N('Shape', { appearance: node(N('PBRAppearance', { baseColor: num(0.7, 0.53, 0.33), roughness: num(0.8), metalness: num(0) })), geometry: node(N('Box', { size: num(s[0], s[1], s[2]) })) })]),
      boundingObject: node(N('Box', { size: num(s[0], s[1], s[2]) })),
      ...(m > 0 ? { physics: node(N('Physics', { mass: num(m), density: num(-1) })) } : {}),
    }, { builtin: 'WoodenBox' });
  },
};

export const BUILTIN_PROTOS: Record<string, BuiltinProto> = {
  Floor, RectangleArena, TexturedBackground, TexturedBackgroundLight, SolidBox, WoodenBox,
  ...Object.fromEntries(Object.keys(APPEARANCES).map((k) => [k, appearanceProto(k)])),
};
