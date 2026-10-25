/**
 * Webots Robot node -> KinTree (src/io/kintree.ts). Links are Solids (the Robot itself, joint endPoints and nested fixed
 * Solids); HingeJoint / SliderJoint / Hinge2Joint / BallJoint become revolute / continuous / prismatic joints named
 * after their motor (Webots + webots_ros2 convention), limits from the motor min/maxPosition or the joint stops (none ->
 * continuous), velocities from the motor; endPoints stored at a non-zero `position` are rotated back to the zero pose.
 * Joints without a motor are passive and imported as fixed at their current value. Devices become link sensors:
 * Camera / RangeFinder / Lidar with the +X optical axis converted to the studio camera frame and horizontal FOV, GPS,
 * InertialUnit, Gyro, Accelerometer, Compass, DistanceSensor, TouchSensor.
 */
import { GeometryRef, ParamValue } from '../../core/items/item';
import { Mat4, Vec3, identity, multiply, invert, transl, rotAxis, transformPoint, transformDir, fromArray, normalize, RAD } from '../../core/math/pose';
import { KinTree, KinLink, KinJoint, KinSensor } from '../kintree';
import { WNode, fNum, fNum1, fStr, fStrs, fNode, fNodes, fBool } from './vrml';
import { BuildCtx, poseOf, rigidOf, walkShapes, boundingRefs, warnOnce, M2MM } from './geometry';

/** Studio camera axes (X right, Y down, Z view) expressed in a Webots device frame (X forward, Y left, Z up). */
export const WEBOTS_DEVICE_TO_STUDIO_CAMERA: Mat4 = fromArray([0, -1, 0, 0, 0, 0, -1, 0, 1, 0, 0, 0, 0, 0, 0, 1]);

export const DEVICE_TYPES = new Set(['Camera', 'RangeFinder', 'Lidar', 'GPS', 'InertialUnit', 'Gyro', 'Accelerometer', 'Compass', 'DistanceSensor', 'TouchSensor', 'LightSensor', 'Radar', 'Receiver', 'Emitter', 'Display', 'LED', 'Speaker', 'Pen', 'Connector', 'VacuumGripper', 'Altimeter', 'Microphone', 'Radio', 'Charger']);
const DEFAULT_NAMES: Record<string, string> = { Camera: 'camera', RangeFinder: 'range-finder', Lidar: 'lidar', GPS: 'gps', InertialUnit: 'inertial unit', Gyro: 'gyro', Accelerometer: 'accelerometer', Compass: 'compass', DistanceSensor: 'distance sensor', TouchSensor: 'touch sensor' };

export interface RobotInfo {
  name: string;
  controller: string;
  supervisor: boolean;
  /** Number of actuated joints, devices and visuals found (used to tell supervisor-only robots and kinematic platforms). */
  joints: number;
  devices: number;
  visuals: number;
  propellers: number;
  params: Record<string, ParamValue>;
}

/** Convert a resolved Webots Robot node into a kinematic tree (frame = the Robot node frame). */
export function robotToKinTree(robot: WNode, ctx: BuildCtx): { tree: KinTree; info: RobotInfo } {
  const name = fStr(robot, 'name', 'robot');
  const links: KinLink[] = [];
  const joints: KinJoint[] = [];
  const used = new Set<string>();
  const uniq = (base: string) => { let n = base || 'link', k = 1; while (used.has(n)) n = `${base}_${k++}`; used.add(n); return n; };
  const jointNames = new Set<string>();
  const uniqJ = (base: string) => { let n = base, k = 1; while (jointNames.has(n)) n = `${base}_${k++}`; jointNames.add(n); return n; };
  const info: RobotInfo = { name, controller: fStr(robot, 'controller', '<generic>'), supervisor: fBool(robot, 'supervisor', false), joints: 0, devices: 0, visuals: 0, propellers: 0, params: {} };

  const newLink = (n: string, solid: WNode | null): KinLink => {
    const l: KinLink = { name: uniq(n), visuals: [] };
    if (solid) {
      const ph = fNode(solid, 'physics');
      const p = ph ? ctx.reg.resolve(ph) : null;
      if (p) { const m = fNum1(p, 'mass', -1); if (m > 0) l.inertial = { mass: m, ...(fNum(p, 'centerOfMass', []).length === 3 ? { com: fNum(p, 'centerOfMass', []).map((v) => v * M2MM) as Vec3 } : {}) }; }
      const params: Record<string, ParamValue> = { webotsName: fStr(solid, 'name', '') };
      if (solid.def) params.def = solid.def;
      const cm = fStr(solid, 'contactMaterial', '');
      if (cm && cm !== 'default') params.contactMaterial = cm;
      l.params = params;
    }
    links.push(l);
    return l;
  };

  const addBounding = (link: KinLink, solid: WNode, T: Mat4) => {
    const bo = fNode(solid, 'boundingObject');
    if (!bo) return;
    const refs = boundingRefs(bo, T, ctx);
    if (refs.length) (link.collisions ??= []).push(...refs);
  };

  const sensorOf = (dev: WNode, T: Mat4): KinSensor | null => {
    const P = multiply(rigidOf(T), poseOf(dev));
    const nm = fStr(dev, 'name', DEFAULT_NAMES[dev.type] ?? dev.type.toLowerCase());
    const cam = () => multiply(P, WEBOTS_DEVICE_TO_STUDIO_CAMERA);
    switch (dev.type) {
      case 'Camera': {
        const far = fNum1(dev, 'far', 0);
        return { kind: 'camera', name: nm, pose: cam(), fov: fNum1(dev, 'fieldOfView', 0.7854) * RAD, fovAxis: 'horizontal', width: fNum1(dev, 'width', 64), height: fNum1(dev, 'height', 64), near: fNum1(dev, 'near', 0.01) * M2MM, ...(far > 0 ? { far: far * M2MM } : {}), params: { webotsDevice: 'Camera' } };
      }
      case 'RangeFinder':
        return { kind: 'depth', name: nm, pose: cam(), fov: fNum1(dev, 'fieldOfView', 0.7854) * RAD, fovAxis: 'horizontal', width: fNum1(dev, 'width', 64), height: fNum1(dev, 'height', 64), near: fNum1(dev, 'minRange', 0.01) * M2MM, far: fNum1(dev, 'maxRange', 1) * M2MM, params: { webotsDevice: 'RangeFinder' } };
      case 'Lidar': {
        const layers = fNum1(dev, 'numberOfLayers', 4);
        return {
          kind: layers > 1 ? 'lidar3d' : 'lidar2d', name: nm, pose: cam(), fov: fNum1(dev, 'fieldOfView', Math.PI / 2) * RAD, fovAxis: 'horizontal', width: fNum1(dev, 'horizontalResolution', 512), height: layers, layers,
          near: fNum1(dev, 'minRange', 0.01) * M2MM, far: fNum1(dev, 'maxRange', 1) * M2MM,
          params: { webotsDevice: 'Lidar', verticalFieldOfView: fNum1(dev, 'verticalFieldOfView', 0.2) * RAD, noise: fNum1(dev, 'noise', 0), defaultFrequency: fNum1(dev, 'defaultFrequency', 10), type: fStr(dev, 'type', 'fixed') },
        };
      }
      case 'GPS': return { kind: 'gnss', name: nm, pose: P, params: { webotsDevice: 'GPS' } };
      case 'InertialUnit': return { kind: 'imu', name: nm, pose: P, params: { webotsDevice: 'InertialUnit' } };
      case 'Gyro': return { kind: 'gyro', name: nm, pose: P, params: { webotsDevice: 'Gyro' } };
      case 'Accelerometer': return { kind: 'accelerometer', name: nm, pose: P, params: { webotsDevice: 'Accelerometer' } };
      case 'Compass': return { kind: 'compass', name: nm, pose: P, params: { webotsDevice: 'Compass' } };
      case 'DistanceSensor': return { kind: 'distance', name: nm, pose: P, params: { webotsDevice: 'DistanceSensor', type: fStr(dev, 'type', 'generic') } };
      case 'TouchSensor': return { kind: 'touch', name: nm, pose: P, params: { webotsDevice: 'TouchSensor' } };
      default: return { kind: 'other', name: nm, pose: P, params: { webotsDevice: dev.type } };
    }
  };

  const motorOf = (devices: WNode[]) => {
    let motor: WNode | null = null, sensor: WNode | null = null;
    for (const d of devices) { const r = ctx.reg.resolve(d); if (!r) continue; if (r.type === 'RotationalMotor' || r.type === 'LinearMotor') motor = r; else if (r.type === 'PositionSensor') sensor = r; }
    return { motor, sensor };
  };

  const jointName = (motor: WNode | null, sensor: WNode | null, ep: WNode, fallback: string) =>
    uniqJ(motor ? fStr(motor, 'name', motor.type === 'LinearMotor' ? 'linear motor' : 'rotational motor') : sensor ? fStr(sensor, 'name', 'position sensor').replace(/_sensor$/, '') : `${fStr(ep, 'name', fallback)}_joint`);

  const limits = (motor: WNode | null, jp: WNode | null, conv: number): { lower?: number; upper?: number } => {
    const mn = motor ? fNum1(motor, 'minPosition', 0) : 0, mx = motor ? fNum1(motor, 'maxPosition', 0) : 0;
    if (mn !== 0 || mx !== 0) return { lower: mn * conv, upper: mx * conv };
    const s0 = jp ? fNum1(jp, 'minStop', 0) : 0, s1 = jp ? fNum1(jp, 'maxStop', 0) : 0;
    if (s0 !== 0 || s1 !== 0) return { lower: s0 * conv, upper: s1 * conv };
    return {};
  };

  /** Add the content of a Solid's `children` to `link`; T maps node coordinates into the link frame. */
  const addContent = (link: KinLink, children: WNode[], T: Mat4) => {
    walkShapes(children, T, ctx, link.visuals, {
      other: (n, M) => {
        if (n.type === 'Solid' || n.type === 'Robot') {
          if (n.type === 'Robot') warnOnce(ctx, `${name}: nested Robot ${fStr(n, 'name', '')} treated as a fixed link`);
          const P = multiply(rigidOf(M), poseOf(n));
          const child = newLink(fStr(n, 'name', 'solid'), n);
          joints.push({ name: uniqJ(`${child.name}_fixed`), type: 'fixed', parent: link.name, child: child.name, origin: P, axis: [0, 0, 1] });
          addContent(child, fNodes(n, 'children'), identity());
          addBounding(child, n, identity());
          return true;
        }
        if (DEVICE_TYPES.has(n.type)) {
          info.devices++;
          const s = sensorOf(n, M);
          if (s && s.kind !== 'other') (link.sensors ??= []).push(s);
          else if (s) { (link.params ??= {}); const list = (link.params.devices as ParamValue[] | undefined) ?? []; list.push({ type: n.type, name: s.name, pose: Array.from(s.pose) }); link.params.devices = list; }
          const P = multiply(rigidOf(M), poseOf(n));
          walkShapes(fNodes(n, 'children'), P, ctx, link.visuals, { other: (c) => { warnOnce(ctx, `${name}: ${c.type} inside device ${n.type} ignored`); return true; } });
          const bo = fNode(n, 'boundingObject');
          if (bo) { const refs = boundingRefs(bo, P, ctx); if (refs.length) (link.collisions ??= []).push(...refs); }
          return true;
        }
        if (n.type === 'HingeJoint' || n.type === 'SliderJoint' || n.type === 'Hinge2Joint' || n.type === 'BallJoint') { addJoint(link, n, rigidOf(M)); return true; }
        if (n.type === 'Propeller') {
          info.propellers++;
          for (const f of ['slowHelix', 'fastHelix']) {
            const h = fNode(n, f);
            const hs = h ? ctx.reg.resolve(h) : null;
            if (hs && f === 'slowHelix') walkShapes(fNodes(hs, 'children'), multiply(rigidOf(M), poseOf(hs)), ctx, link.visuals, { other: () => true });
          }
          const ax = fNum(n, 'shaftAxis', [1, 0, 0]);
          const c = fNum(n, 'centerOfThrust', [0, 0, 0]);
          (link.params ??= {});
          const list = (link.params.propellers as ParamValue[] | undefined) ?? [];
          list.push({ shaftAxis: ax, centerOfThrust: c.map((v) => v * M2MM) });
          link.params.propellers = list;
          return true;
        }
        if (n.type === 'Track') { warnOnce(ctx, `${name}: Track is imported as static geometry (tracked drive not detected)`); addContent(link, fNodes(n, 'children'), multiply(rigidOf(M), poseOf(n))); return true; }
        if (n.type === 'Slot') { const ep = fNode(n, 'endPoint'); if (ep) addContent(link, [ep], M); return true; }
        return false;
      },
    });
  };

  const addJoint = (parent: KinLink, j: WNode, T: Mat4) => {
    const epRaw = fNode(j, 'endPoint');
    let ep = epRaw ? ctx.reg.resolve(epRaw) : null;
    for (let k = 0; ep?.type === 'Slot' && k < 8; k++) { const e2 = fNode(ep, 'endPoint'); ep = e2 ? ctx.reg.resolve(e2) : null; } // a Slot plugs the real endPoint
    if (!ep) return;
    if (ep.type === 'SolidReference') { warnOnce(ctx, `${name}: closed kinematic loop (SolidReference) ignored`); return; }
    const Te = multiply(T, poseOf(ep));
    if (j.type === 'HingeJoint' || j.type === 'SliderJoint') {
      const jpRaw = fNode(j, 'jointParameters');
      const jp = jpRaw ? ctx.reg.resolve(jpRaw) : null;
      const prism = j.type === 'SliderJoint';
      const axisL = normalize(transformDir(T, (fNum(jp, 'axis', prism ? [0, 0, 1] : [1, 0, 0]) as Vec3)));
      const anchorL = transformPoint(T, fNum(jp, 'anchor', [0, 0, 0]).map((v) => v * M2MM) as Vec3);
      const pos = fNum1(jp, 'position', 0);
      const { motor, sensor } = motorOf(fNodes(j, 'device'));
      const child = newLink(fStr(ep, 'name', 'solid'), ep);
      // zero pose of the endPoint: undo the stored joint position
      const undo = prism ? transl(-axisL[0] * pos * M2MM, -axisL[1] * pos * M2MM, -axisL[2] * pos * M2MM) : multiply(multiply(transl(anchorL[0], anchorL[1], anchorL[2]), rotAxis(axisL, -pos)), transl(-anchorL[0], -anchorL[1], -anchorL[2]));
      const Te0 = multiply(undo, Te);
      const Re = identity();
      for (let i = 0; i < 12; i++) if (i % 4 !== 3) Re[i] = Te0[i];
      const origin = prism ? Te0 : multiply(transl(anchorL[0], anchorL[1], anchorL[2]), Re);
      const axisJ = normalize(transformDir(invert(Re), axisL));
      const content = multiply(invert(origin), Te0);
      if (!motor) {
        // passive joint: fixed at its current value
        joints.push({ name: jointName(null, sensor, ep, child.name), type: 'fixed', parent: parent.name, child: child.name, origin: Te, axis: axisJ, params: { passive: true, webotsJoint: j.type } });
        warnOnce(ctx, `${name}: ${j.type} without motor (${child.name}) is imported as fixed`);
        addContent(child, fNodes(ep, 'children'), identity());
        addBounding(child, ep, identity());
        return;
      }
      info.joints++;
      const conv = prism ? M2MM : RAD;
      const lim = limits(motor, jp, conv);
      const kj: KinJoint = {
        name: jointName(motor, sensor, ep, child.name), type: prism ? 'prismatic' : lim.lower === undefined ? 'continuous' : 'revolute', parent: parent.name, child: child.name,
        origin, axis: axisJ, ...lim, velocity: fNum1(motor, 'maxVelocity', 10) * conv, position: pos * conv,
        params: { webotsJoint: j.type, ...(sensor ? { positionSensor: fStr(sensor, 'name', '') } : {}) },
      };
      joints.push(kj);
      addContent(child, fNodes(ep, 'children'), content);
      addBounding(child, ep, content);
      return;
    }
    // Hinge2Joint / BallJoint: chain of revolute joints through intermediate links at the anchor
    const jpRaw = fNode(j, 'jointParameters');
    const jp = jpRaw ? ctx.reg.resolve(jpRaw) : null;
    const anchorL = transformPoint(T, fNum(jp, 'anchor', [0, 0, 0]).map((v) => v * M2MM) as Vec3);
    const axes: Vec3[] = [];
    const devs: string[] = j.type === 'BallJoint' ? ['device', 'device2', 'device3'] : ['device', 'device2'];
    const jps: Array<WNode | null> = [jp];
    if (j.type === 'Hinge2Joint') {
      const jp2Raw = fNode(j, 'jointParameters2');
      const jp2 = jp2Raw ? ctx.reg.resolve(jp2Raw) : null;
      jps.push(jp2);
      axes.push(normalize(transformDir(T, fNum(jp, 'axis', [1, 0, 0]) as Vec3)), normalize(transformDir(T, fNum(jp2, 'axis', [0, 0, 1]) as Vec3)));
    } else {
      const jp2Raw = fNode(j, 'jointParameters2'), jp3Raw = fNode(j, 'jointParameters3');
      const jp2 = jp2Raw ? ctx.reg.resolve(jp2Raw) : null, jp3 = jp3Raw ? ctx.reg.resolve(jp3Raw) : null;
      jps.push(jp2, jp3);
      axes.push(normalize(transformDir(T, fNum(jp, 'axis', [1, 0, 0]) as Vec3)), normalize(transformDir(T, fNum(jp2, 'axis', [0, 1, 0]) as Vec3)), normalize(transformDir(T, fNum(jp3, 'axis', [0, 0, 1]) as Vec3)));
    }
    const final = newLink(fStr(ep, 'name', 'solid'), ep);
    let prev = parent.name;
    axes.forEach((ax, k) => {
      const { motor, sensor } = motorOf(fNodes(j, devs[k]));
      const childName = k === axes.length - 1 ? final.name : uniq(`${final.name}_${j.type === 'BallJoint' ? 'ball' : 'hinge2'}${k + 1}`);
      if (k < axes.length - 1) links.push({ name: childName, visuals: [] });
      const lim = limits(motor, jps[k], RAD);
      if (motor) info.joints++;
      joints.push({
        name: jointName(motor, sensor, ep, `${final.name}_${k + 1}`), type: motor ? (lim.lower === undefined ? 'continuous' : 'revolute') : 'fixed', parent: prev, child: childName,
        origin: k === 0 ? transl(anchorL[0], anchorL[1], anchorL[2]) : identity(), axis: ax, ...lim, ...(motor ? { velocity: fNum1(motor, 'maxVelocity', 10) * RAD } : {}), position: fNum1(jps[k], 'position', 0) * RAD,
        params: { webotsJoint: j.type },
      });
      prev = childName;
    });
    const content = multiply(transl(-anchorL[0], -anchorL[1], -anchorL[2]), Te);
    addContent(final, fNodes(ep, 'children'), content);
    addBounding(final, ep, content);
  };

  const root = newLink(name, robot);
  addContent(root, fNodes(robot, 'children'), identity());
  addBounding(root, robot, identity());
  info.visuals = links.reduce((s, l) => s + l.visuals.length, 0);
  info.params = {
    webotsType: 'Robot', controller: info.controller, supervisor: info.supervisor, synchronization: fBool(robot, 'synchronization', true),
    controllerArgs: fStrs(robot, 'controllerArgs'), customData: fStr(robot, 'customData', ''),
  };
  if (robot.def) info.params.def = robot.def;
  return { tree: { name, root: root.name, links, joints, params: {} }, info };
}

/** Number of visuals of a tree that are not fully transparent. */
export function visibleVisuals(tree: KinTree): GeometryRef[] {
  return tree.links.flatMap((l) => l.visuals.filter((v) => (v.opacity ?? 1) > 0.01));
}
