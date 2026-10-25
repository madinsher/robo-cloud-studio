/**
 * Webots world -> studio Station. WorldInfo / Viewpoint / Background / lights -> station settings; Solid -> SceneObject
 * (visual geometry flattened, boundingObject -> colliders, nested Solids -> child objects); Pose / Transform / Group ->
 * Frame; Robot -> KinTree -> MobileRobot + Robots + cameras (src/io/kintree.ts). PROTO instances are expanded through
 * the registry; DEF names, PROTO names and controllers are kept in item params.
 */
import { Item, Station, SceneObject, Frame, Notes, GeometryRef, ParamValue } from '../../core/items/item';
import { Mat4, identity, multiply, transformDir, transformPoint, normalize } from '../../core/math/pose';
import { buildStationItemsFromTree, KinDriveOverride, KinBuildResult } from '../kintree';
import { MobileRobot } from '../../mobile/items';
import { WNode, WFile, parseWorld, fNum, fNum1, fStr, fStrs, fNode, fNodes, fBool, newScope } from './vrml';
import { ProtoRegistry, WebotsSource, baseName } from './proto';
import { BuildCtx, poseOf, rigidOf, walkShapes, boundingRefs, warnOnce, M2MM, decompose } from './geometry';
import { robotToKinTree, visibleVisuals, RobotInfo } from './robot';
import type { AssetStore } from '../../scene/assets';

export interface WebotsBuildOptions {
  assets?: AssetStore;
  /** Parse and register mesh files (default true). */
  loadMeshes?: boolean;
  /** Optional fetcher for remote PROTO URLs. */
  remote?: (url: string) => Uint8Array | undefined;
}

export interface WebotsBuildResult {
  station: Station;
  report: string[];
  world: WFile;
  /** Per-robot import summary (name -> items). */
  robots: Array<{ name: string; info: RobotInfo; result?: KinBuildResult }>;
  /** The builder (PROTO registry + scene builder) for later additions such as the retail layout. */
  builder: WebotsSceneBuilder;
}

function setColliders(item: Item, refs: GeometryRef[]): void {
  (item as unknown as { colliders?: GeometryRef[] }).colliders = refs;
}
/** Solid-derived nodes built like a Solid outside robots (devices keep their geometry and boundingObject; Fluid volume). */
const SOLID_LIKE = new Set(['Charger', 'Connector', 'Display', 'DistanceSensor', 'Emitter', 'Receiver', 'LED', 'LightSensor', 'TouchSensor', 'Pen', 'Speaker', 'Camera', 'RangeFinder', 'Lidar', 'GPS', 'Compass', 'InertialUnit', 'Gyro', 'Accelerometer', 'Altimeter', 'Radar', 'VacuumGripper', 'Fluid']);
const LIGHTS = new Set(['PointLight', 'SpotLight', 'DirectionalLight']);
const colliders = (item: Item): GeometryRef[] => (item as unknown as { colliders?: GeometryRef[] }).colliders ?? [];

function protoParams(n: WNode, out: Record<string, ParamValue>): void {
  if (n.def) out.def = n.def;
  if (n.proto) { out.proto = n.proto.name; if (n.proto.def && !out.def) out.def = n.proto.def; if (!n.proto.url.startsWith('builtin:')) out.protoUrl = n.proto.url; }
  if (n.meta?.builtin) out.webotsBuiltin = String(n.meta.builtin);
}

export class WebotsSceneBuilder {
  readonly ctx: BuildCtx;
  readonly robots: WebotsBuildResult['robots'] = [];
  constructor(public reg: ProtoRegistry, src: WebotsSource, opts: WebotsBuildOptions) {
    this.ctx = { reg, src, assets: opts.assets, report: [], loadMeshes: opts.loadMeshes ?? true, meshes: new Map(), warned: new Set() };
  }

  /** Build the station of a parsed world. */
  buildWorld(world: WFile): Station {
    const station = new Station(baseName(world.path).replace(/\.wbt$/i, ''));
    station.filePath = world.path;
    const webots: Record<string, ParamValue> = { version: world.version, file: world.path, externprotos: world.externs.map((e) => e.url) };
    station.settings.webots = webots;
    station.settings.units = 'mm';
    station.settings.upAxis = 'z';
    const lights: ParamValue[] = (this.lights = []);
    for (const raw of world.nodes) {
      const n = this.reg.resolve(raw);
      if (!n) { this.ctx.report.push(`${raw.type}${raw.def ? ` (DEF ${raw.def})` : ''}: skipped (cannot be resolved)`); continue; }
      switch (n.type) {
        case 'WorldInfo': {
          const title = fStr(n, 'title', '');
          if (title) station.name = title;
          webots.title = title;
          webots.basicTimeStep = fNum1(n, 'basicTimeStep', 32);
          webots.gravity = fNum1(n, 'gravity', 9.81);
          webots.coordinateSystem = fStr(n, 'coordinateSystem', 'ENU');
          webots.FPS = fNum1(n, 'FPS', 60);
          webots.contactProperties = fNodes(n, 'contactProperties').map((c) => ({ material1: fStr(c, 'material1', 'default'), material2: fStr(c, 'material2', 'default'), coulombFriction: fNum(c, 'coulombFriction', [1]), bounce: fNum1(c, 'bounce', 0.5), softCFM: fNum1(c, 'softCFM', 0.001) }));
          station.settings.gravity = -fNum1(n, 'gravity', 9.81);
          const info = fStrs(n, 'info');
          if (info.length) { const notes = new Notes('World info'); notes.text = info.join('\n'); station.addChild(notes); }
          if (webots.coordinateSystem !== 'ENU') this.ctx.report.push(`WorldInfo: coordinateSystem ${webots.coordinateSystem} is not converted (ENU assumed)`);
          break;
        }
        case 'Viewpoint': {
          const p = fNum(n, 'position', [0, 0, 10]);
          station.settings.viewpoint = {
            position: [p[0] * M2MM, p[1] * M2MM, p[2] * M2MM], orientation: fNum(n, 'orientation', [0, 0, 1, 0]), fieldOfView: fNum1(n, 'fieldOfView', 0.785398),
            near: fNum1(n, 'near', 0.05) * M2MM, follow: fStr(n, 'follow', ''), followType: fStr(n, 'followType', ''), ...(n.def ? { def: n.def } : {}),
          };
          break;
        }
        case 'Background': {
          station.settings.background = { skyColor: fNum(n, 'skyColor', [0, 0, 0]).slice(0, 3), luminosity: fNum1(n, 'luminosity', 1), ...(n.meta?.texture ? { texture: String(n.meta.texture) } : {}) };
          break;
        }
        case 'DirectionalLight': case 'PointLight': case 'SpotLight': this.addLight(n, identity()); break;
        case 'Fog': station.settings.fog = { color: fNum(n, 'color', [1, 1, 1]), visibilityRange: fNum1(n, 'visibilityRange', 0) * M2MM }; break;
        default: this.buildNode(n, station, identity());
      }
    }
    station.settings.lights = lights;
    return station;
  }

  private lights: ParamValue[] = [];
  /** A light (world level or inside a Solid / Robot) -> station.settings.lights, location / direction in station coordinates. */
  addLight(n: WNode, abs: Mat4): void {
    const l: Record<string, ParamValue> = { type: n.type, intensity: fNum1(n, 'intensity', 1), color: fNum(n, 'color', [1, 1, 1]), ambientIntensity: fNum1(n, 'ambientIntensity', 0), castShadows: fBool(n, 'castShadows', false) };
    if (n.type !== 'PointLight') l.direction = normalize(transformDir(abs, fNum(n, 'direction', [0, 0, -1]) as [number, number, number]));
    if (n.type !== 'DirectionalLight') l.location = transformPoint(abs, fNum(n, 'location', [0, 0, 0]).map((v) => v * M2MM) as [number, number, number]);
    if (n.meta?.texture) l.texture = String(n.meta.texture);
    if (abs[12] || abs[13] || abs[14] || abs[0] !== 1 || abs[5] !== 1) l.attached = true;
    this.lights.push(l);
  }

  /** Build a non-world node under `parent` (M: rigid transform from parent frame to the node's containing frame). */
  buildNode(n: WNode, parent: Item, M: Mat4): Item | null {
    if (SOLID_LIKE.has(n.type)) return this.buildSolid(n, parent, M); // Solid-derived devices (Charger, Connector, ...) and Fluid
    switch (n.type) {
      case 'Solid': return this.buildSolid(n, parent, M);
      case 'Robot': return this.buildRobot(n, parent, M);
      case 'Slot': { const ep = fNode(n, 'endPoint'); const e = ep ? this.reg.resolve(ep) : null; return e ? this.buildNode(e, parent, M) : null; }
      case 'PointLight': case 'SpotLight': case 'DirectionalLight': this.addLight(n, multiply(parent.poseAbs(), M)); return null;
      case 'Pose': case 'Transform': case 'Group': return this.buildGroup(n, parent, M);
      case 'Shape': case 'CadShape': {
        const so = new SceneObject(n.def ?? (n.type === 'CadShape' ? baseName(fStrs(n, 'url')[0] ?? 'cad') : 'shape'));
        walkShapes([n], identity(), this.ctx, so.geometry);
        setColliders(so, []);
        so.params.webotsType = n.type;
        protoParams(n, so.params);
        parent.addChild(so);
        so.setPose(M);
        return so;
      }
      default:
        if (['HingeJoint', 'SliderJoint', 'Hinge2Joint', 'BallJoint'].includes(n.type)) {
          const ep = fNode(n, 'endPoint');
          const e = ep ? this.reg.resolve(ep) : null;
          warnOnce(this.ctx, `${n.type} outside a Robot: endPoint ${e ? fStr(e, 'name', '') : ''} imported as a static object`);
          return e ? this.buildNode(e, parent, M) : null;
        }
        warnOnce(this.ctx, `${n.type} at this level is not supported (${n.base})`);
        return null;
    }
  }

  private buildGroup(n: WNode, parent: Item, M: Mat4): Item {
    const fr = new Frame(n.def ?? n.proto?.name ?? n.type);
    fr.params.webotsType = n.type;
    protoParams(n, fr.params);
    parent.addChild(fr);
    const P = n.type === 'Group' ? identity() : poseOf(n);
    fr.setPose(multiply(M, P));
    const sc = n.type === 'Transform' ? fNum(n, 'scale', [1, 1, 1]) : [1, 1, 1];
    const S = identity(); S[0] = sc[0]; S[5] = sc[1]; S[10] = sc[2];
    const shapes: GeometryRef[] = [];
    walkShapes(fNodes(n, 'children'), S, this.ctx, shapes, {
      other: (c, Mc) => {
        if (c.type === 'Solid' || c.type === 'Robot' || c.type === 'Pose' || c.type === 'Transform' || c.type === 'Group' || SOLID_LIKE.has(c.type) || LIGHTS.has(c.type)) {
          if (decompose(Mc).scale.some((v) => Math.abs(Math.abs(v) - 1) > 1e-6)) warnOnce(this.ctx, `${c.type} under a scaled Transform: scale ignored for the Solid`);
          this.buildNode(c, fr, rigidOf(Mc));
          return true;
        }
        return false;
      },
    });
    if (shapes.length) {
      const so = new SceneObject(`${fr.name}_shapes`);
      so.geometry = shapes;
      setColliders(so, []);
      fr.addChild(so);
    }
    return fr;
  }

  buildSolid(n: WNode, parent: Item, M: Mat4): SceneObject {
    const name = fStr(n, 'name', n.def ?? (n.type === 'Solid' ? 'solid' : n.type.toLowerCase()));
    const so = new SceneObject(name);
    so.params.webotsType = n.type;
    protoParams(n, so.params);
    if (fBool(n, 'locked', false)) so.params.locked = true;
    const cm = fStr(n, 'contactMaterial', 'default');
    if (cm !== 'default') so.params.contactMaterial = cm;
    const ph = fNode(n, 'physics');
    const p = ph ? this.reg.resolve(ph) : null;
    if (p) { so.mass = Math.max(0, fNum1(p, 'mass', -1)); so.params.physics = true; }
    parent.addChild(so);
    so.setPose(multiply(M, poseOf(n)));
    walkShapes(fNodes(n, 'children'), identity(), this.ctx, so.geometry, {
      other: (c, Mc) => {
        if (c.type === 'Solid' || c.type === 'Robot' || SOLID_LIKE.has(c.type) || LIGHTS.has(c.type) || ['HingeJoint', 'SliderJoint', 'Hinge2Joint', 'BallJoint'].includes(c.type)) { this.buildNode(c, so, rigidOf(Mc)); return true; }
        return false;
      },
    });
    const ground = { plane: false };
    const bo = n.type === 'Fluid' ? null : fNode(n, 'boundingObject'); // a Fluid's boundingObject is its immersion volume, not a collider
    if (n.type === 'Fluid') { so.params.fluid = true; so.params.density = fNum1(n, 'density', 1000); }
    setColliders(so, bo ? boundingRefs(bo, identity(), this.ctx, ground) : []);
    if (ground.plane || n.meta?.ground) so.params.ground = true;
    else {
      // floor-like Solid (one thin box collider whose top is at the station ground, >= 4 m2): tag as ground
      const c = colliders(so);
      const P = so.poseAbs();
      if (c.length === 1 && c[0].primitive?.kind === 'box' && Math.abs(transformDir(P, [0, 0, 1])[2] - 1) < 1e-6) {
        const s = c[0].primitive.size;
        const top = P[14] + (c[0].origin?.[14] ?? 0) + s[2] / 2;
        if (s[2] <= 50 && Math.abs(top) <= 1 && s[0] * s[1] >= 4e6) { so.params.ground = true; so.params.groundReason = 'floor-like'; }
      }
    }
    if (!bo) so.params.noCollider = true;
    return so;
  }

  buildRobot(n: WNode, parent: Item, M: Mat4): Item {
    const { tree, info } = robotToKinTree(n, this.ctx);
    const pose = multiply(M, poseOf(n));
    const visible = visibleVisuals(tree).length;
    const hasDevices = info.devices > 0;
    if (!info.joints && !visible && !hasDevices) {
      const fr = new Frame(info.name);
      Object.assign(fr.params, info.params);
      protoParams(n, fr.params);
      parent.addChild(fr);
      fr.setPose(pose);
      this.robots.push({ name: info.name, info });
      this.ctx.report.push(`${info.name}: ${info.supervisor ? 'supervisor ' : ''}Robot without geometry (controller ${info.controller}) imported as a frame`);
      return fr;
    }
    let drive: 'auto' | 'kinematic' | KinDriveOverride = 'auto';
    if (!info.joints) {
      // kinematic platform (pose written by a supervisor): take the drive kinematics from the PROTO's default variant
      drive = 'kinematic';
      if (n.proto && !n.proto.url.startsWith('builtin:') && !n.proto.url.includes('://')) {
        const twinScope = `${n.proto.url}#default-variant`;
        if (!this.reg.scopeOf(twinScope)) this.reg.registerScope(twinScope, [{ url: baseName(n.proto.url), importable: false }]);
        const twin = this.reg.resolve({ type: n.proto.name, fields: {}, base: twinScope, line: 0 });
        if (twin && twin.type === 'Robot') {
          const t = robotToKinTree(twin, { ...this.ctx, report: [], warned: new Set(this.ctx.warned) });
          if (t.info.joints) {
            const tr = buildStationItemsFromTree(t.tree, { name: info.name, assets: this.ctx.assets });
            if (tr.mobile) {
              const { footprint: _f, ...kin } = tr.mobile.kin;
              void _f;
              drive = kin;
              this.ctx.report.push(`${info.name}: no joints in this variant; drive kinematics taken from the default ${n.proto.name} variant (${kin.drive})`);
            }
          }
          if (t.info.propellers) { drive = typeof drive === 'object' ? drive : {}; (drive as KinDriveOverride).drive = 'omni'; }
        }
      }
    }
    const res = buildStationItemsFromTree(tree, { name: info.name, assets: this.ctx.assets, drive });
    for (const l of res.report) this.ctx.report.push(l);
    const root = res.root;
    for (const [k, v] of Object.entries(info.params)) root.params[k] = v;
    protoParams(n, root.params);
    if (info.propellers) root.params.aerial = true;
    if (!info.joints) root.params.kinematic = true;
    parent.addChild(root);
    root.setPose(multiply(pose, root.pose()));
    if (root instanceof MobileRobot) {
      root.home = { x: root.state.x, y: root.state.y, theta: root.state.theta };
      if (info.propellers || n.proto?.name && /UTP_A|drone|quad/i.test(n.proto.name)) root.params.aerial = true;
    }
    this.robots.push({ name: info.name, info, result: res });
    return root;
  }
}

/** Parse + build a world file from a source. */
export function buildWebotsWorld(worldPath: string, src: WebotsSource, opts: WebotsBuildOptions = {}): WebotsBuildResult {
  const bytes = src.read(worldPath);
  if (!bytes) throw new Error(`world ${worldPath} not found`);
  const scope = newScope();
  const world = parseWorld(new TextDecoder().decode(bytes), worldPath, scope);
  const m = /^R(\d{4})([a-z]?)$/.exec(world.version);
  const reg = new ProtoRegistry(src, { remote: opts.remote, context: { world: worldPath, ...(m ? { webots_version: { major: `R${m[1]}`, revision: m[2] } } : {}) } });
  for (const w of scope.warnings) reg.report.push(w);
  reg.registerScope(worldPath, world.externs);
  if (!world.externs.length) reg.report.push(`${worldPath}: no EXTERNPROTO declarations (standard PROTOs resolved by name)`);
  if (world.version && /^R20(1\d|2[01])/.test(world.version)) reg.report.push(`${worldPath}: ${world.version} world (pre-R2022a NUE axes are not converted)`);
  const b = new WebotsSceneBuilder(reg, src, opts);
  const station = b.buildWorld(world);
  return { station, report: [...reg.report, ...b.ctx.report], world, robots: b.robots, builder: b };
}

/** Expand a single PROTO with its default fields into items (preview of a robot / object PROTO without a world). */
export function buildWebotsProto(protoPath: string, src: WebotsSource, opts: WebotsBuildOptions = {}, fields: Record<string, import('./vrml').WValue> = {}): WebotsBuildResult {
  const reg = new ProtoRegistry(src, { remote: opts.remote });
  const name = baseName(protoPath).replace(/\.proto$/i, '');
  reg.registerScope(`${protoPath}#preview`, [{ url: baseName(protoPath), importable: false }]);
  const b = new WebotsSceneBuilder(reg, src, opts);
  const station = new Station(name);
  station.filePath = protoPath;
  const inst: WNode = { type: name, fields, base: `${protoPath}#preview` };
  const n = reg.resolve(inst);
  if (n) b.buildNode(n, station, identity());
  else b.ctx.report.push(`${name}: PROTO could not be expanded`);
  return { station, report: [...reg.report, ...b.ctx.report], world: { path: protoPath, version: '', header: '', externs: [], nodes: [], meta: {} }, robots: b.robots, builder: b };
}

export { transformDir };
