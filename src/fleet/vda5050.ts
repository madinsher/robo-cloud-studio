/**
 * VDA 5050 (v2.x) — the standard interface between AGV/AMR fleets and a master control system used by
 * KUKA Fleet / KMP platforms, MiR, SEW, Jungheinrich, Linde, STILL, Omron, ANT (BlueBotics), Open-RMF adapters…
 *
 * This module is transport-neutral (no MQTT here): it builds and parses the JSON messages, converts between
 * the studio's fleet model (mm, degrees, station frame) and VDA 5050 (metres, radians, map frame), and keeps
 * per-AGV order bookkeeping (orderId / orderUpdateId, base/horizon, action states).
 *
 *   master control (studio fleet manager)  --order / instantActions-->  AGV
 *   AGV  --state / connection / visualization / factsheet-->  master control
 *
 * Two roles are supported:
 *   - Master: the studio dispatches its fleet tasks as VDA 5050 orders to real AGVs and mirrors their state.
 *   - AGV bridge: simulated studio robots answer as VDA 5050 AGVs (digital twins for testing a master such as
 *     KUKA Fleet or your own dispatcher).
 */
import type { FleetTask } from './fleet';
import type { MobileRobot } from '../mobile/items';

export const VDA_VERSION = '2.0.0';
export const VDA_PROTOCOL = 'v2';

// ---------------------------------------------------------------------------------------------
// Message types (subset of the VDA 5050 JSON schemas that matters for dispatch and monitoring)
// ---------------------------------------------------------------------------------------------

export interface VdaHeader {
  headerId: number;
  /** ISO 8601 UTC. */
  timestamp: string;
  version: string;
  manufacturer: string;
  serialNumber: string;
}

export interface VdaActionParameter { key: string; value: string | number | boolean | unknown[] | Record<string, unknown> }
export interface VdaAction {
  actionType: string;
  actionId: string;
  actionDescription?: string;
  blockingType: 'NONE' | 'SOFT' | 'HARD';
  actionParameters?: VdaActionParameter[];
}

export interface VdaNodePosition { x: number; y: number; theta?: number; allowedDeviationXY?: number; allowedDeviationTheta?: number; mapId: string; mapDescription?: string }
export interface VdaNode { nodeId: string; sequenceId: number; nodeDescription?: string; released: boolean; nodePosition?: VdaNodePosition; actions: VdaAction[] }
export interface VdaEdge { edgeId: string; sequenceId: number; edgeDescription?: string; released: boolean; startNodeId: string; endNodeId: string; maxSpeed?: number; maxHeight?: number; minHeight?: number; orientation?: number; direction?: string; rotationAllowed?: boolean; maxRotationSpeed?: number; length?: number; actions: VdaAction[] }

export interface VdaOrder extends VdaHeader { orderId: string; orderUpdateId: number; zoneSetId?: string; nodes: VdaNode[]; edges: VdaEdge[] }
export interface VdaInstantActions extends VdaHeader { actions: VdaAction[] }

export interface VdaNodeState { nodeId: string; sequenceId: number; nodeDescription?: string; nodePosition?: VdaNodePosition; released: boolean }
export interface VdaEdgeState { edgeId: string; sequenceId: number; edgeDescription?: string; released: boolean; trajectory?: unknown }
export interface VdaActionState { actionId: string; actionType?: string; actionDescription?: string; actionStatus: 'WAITING' | 'INITIALIZING' | 'RUNNING' | 'PAUSED' | 'FINISHED' | 'FAILED'; resultDescription?: string }
export interface VdaError { errorType: string; errorLevel: 'WARNING' | 'FATAL'; errorDescription?: string; errorReferences?: Array<{ referenceKey: string; referenceValue: string }> }
export interface VdaAgvPosition { x: number; y: number; theta: number; mapId: string; mapDescription?: string; positionInitialized: boolean; localizationScore?: number; deviationRange?: number }
export interface VdaBatteryState { batteryCharge: number; batteryVoltage?: number; batteryHealth?: number; charging: boolean; reach?: number }

export interface VdaState extends VdaHeader {
  orderId: string;
  orderUpdateId: number;
  zoneSetId?: string;
  lastNodeId: string;
  lastNodeSequenceId: number;
  nodeStates: VdaNodeState[];
  edgeStates: VdaEdgeState[];
  agvPosition?: VdaAgvPosition;
  velocity?: { vx?: number; vy?: number; omega?: number };
  loads?: unknown[];
  driving: boolean;
  paused?: boolean;
  newBaseRequest?: boolean;
  distanceSinceLastNode?: number;
  actionStates: VdaActionState[];
  batteryState: VdaBatteryState;
  operatingMode: 'AUTOMATIC' | 'SEMIAUTOMATIC' | 'MANUAL' | 'SERVICE' | 'TEACHIN';
  errors: VdaError[];
  information?: Array<{ infoType: string; infoLevel: 'INFO' | 'DEBUG'; infoDescription?: string }>;
  safetyState: { eStop: 'AUTOACK' | 'MANUAL' | 'REMOTE' | 'NONE'; fieldViolation: boolean };
}

export interface VdaConnection extends VdaHeader { connectionState: 'ONLINE' | 'OFFLINE' | 'CONNECTIONBROKEN' }
export interface VdaVisualization extends VdaHeader { agvPosition?: VdaAgvPosition; velocity?: { vx?: number; vy?: number; omega?: number } }
export interface VdaFactsheet extends VdaHeader {
  typeSpecification: { seriesName: string; seriesDescription?: string; agvKinematic: 'DIFF' | 'OMNI' | 'THREEWHEEL'; agvClass: 'FORKLIFT' | 'CONVEYOR' | 'TUGGER' | 'CARRIER'; maxLoadMass: number; localizationTypes: string[]; navigationTypes: string[] };
  physicalParameters: { speedMin: number; speedMax: number; accelerationMax: number; decelerationMax: number; heightMin?: number; heightMax?: number; width: number; length: number };
  protocolLimits?: Record<string, unknown>;
  protocolFeatures?: { optionalParameters?: unknown[]; agvActions?: Array<{ actionType: string; actionDescription?: string; actionScopes: string[]; actionParameters?: unknown[] }> };
  agvGeometry?: Record<string, unknown>;
  loadSpecification?: Record<string, unknown>;
}

export type VdaTopicType = 'order' | 'instantActions' | 'state' | 'connection' | 'visualization' | 'factsheet';

// ---------------------------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------------------------

export function vdaTopic(prefix: string, manufacturer: string, serial: string, type: VdaTopicType, protocol = VDA_PROTOCOL): string {
  return `${prefix.replace(/\/+$/, '')}/${protocol}/${manufacturer}/${serial}/${type}`;
}

/** Parse `<prefix>/<version>/<manufacturer>/<serial>/<type>`; returns null for other topics. */
export function parseVdaTopic(topic: string): { prefix: string; protocol: string; manufacturer: string; serial: string; type: VdaTopicType } | null {
  const parts = topic.split('/');
  if (parts.length < 5) return null;
  const type = parts[parts.length - 1] as VdaTopicType;
  if (!['order', 'instantActions', 'state', 'connection', 'visualization', 'factsheet'].includes(type)) return null;
  return { prefix: parts.slice(0, -4).join('/'), protocol: parts[parts.length - 4], manufacturer: parts[parts.length - 3], serial: parts[parts.length - 2], type };
}

export function nowIso(): string { return new Date().toISOString(); }

/** VDA 5050 works in metres/radians; the studio in mm/degrees. */
export const MM = 1000;
export const toRad = (deg: number) => (deg * Math.PI) / 180;
export const toDeg = (rad: number) => (rad * 180) / Math.PI;
/** Serial numbers must be MQTT-topic safe. */
export function serialFor(name: string): string { return name.replace(/[^A-Za-z0-9_-]+/g, '_'); }

// ---------------------------------------------------------------------------------------------
// Studio task -> VDA order
// ---------------------------------------------------------------------------------------------

export interface OrderOptions {
  mapId?: string;
  /** Spacing between generated nodes along the path (mm). Default 2000. */
  nodeSpacing?: number;
  /** Number of released nodes (base); the rest form the horizon. Default: all released. */
  baseNodes?: number;
  /** Max edge speed (m/s). */
  maxSpeed?: number;
  allowedDeviationXY?: number;
  /** Extra actions for the last node (e.g. pick / drop / startCharging). Auto-derived from the task type when omitted. */
  finalActions?: VdaAction[];
  orderId?: string;
  orderUpdateId?: number;
}

let actionSeq = 0;
export function makeAction(actionType: string, blockingType: VdaAction['blockingType'] = 'HARD', params: Record<string, string | number | boolean> = {}, description?: string): VdaAction {
  const a: VdaAction = { actionType, actionId: `${actionType}_${Date.now().toString(36)}_${(actionSeq++).toString(36)}`, blockingType };
  if (description) a.actionDescription = description;
  const keys = Object.keys(params);
  if (keys.length) a.actionParameters = keys.map((k) => ({ key: k, value: params[k] }));
  return a;
}

/** Actions a fleet task implies at its destination (VDA standard actions where they exist, custom otherwise). */
export function actionsForTask(task: Pick<FleetTask, 'type' | 'meta' | 'duration' | 'payloadKg'>): VdaAction[] {
  switch (task.type) {
    case 'charge': return [makeAction('startCharging', 'HARD')];
    case 'transport': return [makeAction(task.meta?.drop ? 'drop' : 'pick', 'HARD', { loadType: String(task.meta?.loadType ?? 'bin'), ...(task.payloadKg ? { loadMass: task.payloadKg } : {}) })];
    case 'goto': return [];
    default: return [makeAction(task.type, 'HARD', { duration: task.duration ?? 0, ...(task.meta?.rowId ? { rowId: String(task.meta.rowId) } : {}) }, `${task.type} task ${task.meta?.name ?? ''}`.trim())];
  }
}

function subsample(path: number[][], spacing: number): number[][] {
  if (path.length <= 2 || spacing <= 0) return path;
  const out = [path[0]];
  let acc = 0;
  for (let i = 1; i < path.length; i++) {
    acc += Math.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1]);
    if (acc >= spacing || i === path.length - 1) { out.push(path[i]); acc = 0; }
  }
  return out;
}

/**
 * Build a VDA 5050 order from a planned path (mm, station frame). The path should start at the AGV's current
 * position (first node) — VDA requires the first node to be reachable trivially.
 */
export function pathToOrder(header: Omit<VdaHeader, 'headerId' | 'timestamp' | 'version'> & { headerId: number }, path: number[][], opts: OrderOptions = {}, workPath?: number[][]): VdaOrder {
  const mapId = opts.mapId ?? 'station';
  const pts = subsample(path, opts.nodeSpacing ?? 2000);
  if (workPath && workPath.length > 1) pts.push(...subsample(workPath, opts.nodeSpacing ?? 2000).slice(0, undefined));
  const orderId = opts.orderId ?? `order_${Date.now().toString(36)}_${Math.random().toString(36).slice(2, 6)}`;
  const base = opts.baseNodes ?? pts.length;
  const nodes: VdaNode[] = pts.map((p, i) => {
    const nxt = pts[Math.min(i + 1, pts.length - 1)], prv = pts[Math.max(i - 1, 0)];
    const theta = Math.atan2(nxt[1] - prv[1], nxt[0] - prv[0]);
    return { nodeId: `n${i}`, sequenceId: i * 2, released: i < base, nodePosition: { x: p[0] / MM, y: p[1] / MM, theta: pts.length > 1 ? theta : undefined, allowedDeviationXY: (opts.allowedDeviationXY ?? 300) / MM, mapId }, actions: [] };
  });
  if (nodes.length) nodes[nodes.length - 1].actions = opts.finalActions ?? [];
  const edges: VdaEdge[] = [];
  for (let i = 0; i + 1 < pts.length; i++) {
    const len = Math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1]) / MM;
    edges.push({ edgeId: `e${i}`, sequenceId: i * 2 + 1, released: i + 1 < base, startNodeId: `n${i}`, endNodeId: `n${i + 1}`, maxSpeed: opts.maxSpeed, length: len, actions: [] });
  }
  return { headerId: header.headerId, timestamp: nowIso(), version: VDA_VERSION, manufacturer: header.manufacturer, serialNumber: header.serialNumber, orderId, orderUpdateId: opts.orderUpdateId ?? 0, nodes, edges };
}

export function taskToOrder(header: Omit<VdaHeader, 'headerId' | 'timestamp' | 'version'> & { headerId: number }, task: FleetTask, path: number[][], opts: OrderOptions = {}): VdaOrder {
  const o = pathToOrder(header, path, { ...opts, finalActions: opts.finalActions ?? actionsForTask(task) }, task.workPath);
  o.nodes.forEach((n) => { n.nodeDescription = `${task.type} ${task.id}`; });
  return o;
}

// ---------------------------------------------------------------------------------------------
// VDA state <-> studio mobile robot
// ---------------------------------------------------------------------------------------------

/** Apply a received AGV state to a studio mobile robot (digital shadow). Returns what changed. */
export function applyStateToRobot(state: VdaState, robot: MobileRobot): { moved: boolean; status: MobileRobot['state']['status'] } {
  let moved = false;
  if (state.agvPosition && state.agvPosition.positionInitialized !== false) {
    const x = state.agvPosition.x * MM, y = state.agvPosition.y * MM, th = toDeg(state.agvPosition.theta);
    moved = Math.abs(x - robot.state.x) > 1 || Math.abs(y - robot.state.y) > 1 || Math.abs(th - robot.state.theta) > 0.1;
    robot.state.x = x; robot.state.y = y; robot.state.theta = th;
    robot.syncPoseFromState();
  }
  if (state.velocity) { robot.state.v = Math.hypot(state.velocity.vx ?? 0, state.velocity.vy ?? 0) * MM; robot.state.omega = toDeg(state.velocity.omega ?? 0); }
  if (state.batteryState) robot.battery.levelWh = robot.battery.capacityWh * Math.max(0, Math.min(1, state.batteryState.batteryCharge / 100));
  const fatal = state.errors?.some((e) => e.errorLevel === 'FATAL') || state.safetyState?.eStop !== 'NONE' && !!state.safetyState;
  const status: MobileRobot['state']['status'] = fatal && state.errors?.length ? 'error' : state.batteryState?.charging ? 'charging' : state.paused ? 'waiting' : state.driving ? 'moving' : state.actionStates?.some((a) => a.actionStatus === 'RUNNING') ? 'working' : 'idle';
  robot.state.status = status;
  robot.setParam('vda5050', { orderId: state.orderId, orderUpdateId: state.orderUpdateId, lastNodeId: state.lastNodeId, remainingNodes: state.nodeStates?.length ?? 0, errors: (state.errors ?? []).map((e) => `${e.errorLevel}: ${e.errorType}${e.errorDescription ? ' — ' + e.errorDescription : ''}`), operatingMode: state.operatingMode, headerId: state.headerId, timestamp: state.timestamp });
  return { moved, status };
}

/** Build a VDA 5050 state message for a simulated studio robot (AGV bridge). */
export function robotToState(robot: MobileRobot, ctx: { header: Omit<VdaHeader, 'timestamp' | 'version'>; order?: VdaOrder | null; lastNodeSequenceId?: number; actionStates?: VdaActionState[]; mapId?: string; errors?: VdaError[] }): VdaState {
  const st = robot.state;
  const order = ctx.order ?? null;
  const lastSeq = ctx.lastNodeSequenceId ?? -1;
  const remaining = order ? order.nodes.filter((n) => n.sequenceId > lastSeq) : [];
  const remainingEdges = order ? order.edges.filter((e) => e.sequenceId > lastSeq) : [];
  const last = order ? order.nodes.filter((n) => n.sequenceId <= lastSeq).pop() : undefined;
  return {
    headerId: ctx.header.headerId, timestamp: nowIso(), version: VDA_VERSION, manufacturer: ctx.header.manufacturer, serialNumber: ctx.header.serialNumber,
    orderId: order?.orderId ?? '', orderUpdateId: order?.orderUpdateId ?? 0,
    lastNodeId: last?.nodeId ?? '', lastNodeSequenceId: last?.sequenceId ?? 0,
    nodeStates: remaining.map((n) => ({ nodeId: n.nodeId, sequenceId: n.sequenceId, released: n.released, nodePosition: n.nodePosition })),
    edgeStates: remainingEdges.map((e) => ({ edgeId: e.edgeId, sequenceId: e.sequenceId, released: e.released })),
    agvPosition: { x: st.x / MM, y: st.y / MM, theta: toRad(st.theta), mapId: ctx.mapId ?? 'station', positionInitialized: true, localizationScore: 1 },
    velocity: { vx: (st.v / MM) * Math.cos(toRad(st.theta)), vy: (st.v / MM) * Math.sin(toRad(st.theta)), omega: toRad(st.omega) },
    driving: st.status === 'moving' || st.path !== null,
    paused: st.status === 'waiting',
    newBaseRequest: order ? remaining.length > 0 && remaining.every((n) => !n.released) : false,
    actionStates: ctx.actionStates ?? [],
    batteryState: { batteryCharge: Math.round(robot.batteryLevel() * 1000) / 10, charging: st.status === 'charging', reach: Math.round(robot.batteryLevel() * 20000) },
    operatingMode: 'AUTOMATIC',
    errors: ctx.errors ?? (st.status === 'error' ? [{ errorType: 'robotError', errorLevel: 'FATAL', errorDescription: 'robot reported an error' }] : []),
    safetyState: { eStop: 'NONE', fieldViolation: false },
  };
}

export function factsheetFor(robot: MobileRobot, header: Omit<VdaHeader, 'timestamp' | 'version'>): VdaFactsheet {
  const kin = robot.kin;
  const k: VdaFactsheet['typeSpecification']['agvKinematic'] = kin.drive === 'omni' ? 'OMNI' : kin.drive === 'ackermann' ? 'THREEWHEEL' : 'DIFF';
  return {
    headerId: header.headerId, timestamp: nowIso(), version: VDA_VERSION, manufacturer: header.manufacturer, serialNumber: header.serialNumber,
    typeSpecification: { seriesName: robot.name, agvKinematic: k, agvClass: 'CARRIER', maxLoadMass: robot.payloadKg ?? 0, localizationTypes: ['NATURAL'], navigationTypes: ['AUTONOMOUS'] },
    physicalParameters: { speedMin: 0.05, speedMax: kin.maxSpeed / MM, accelerationMax: kin.maxAccel / MM, decelerationMax: kin.maxAccel / MM, width: kin.footprint[1] / MM, length: kin.footprint[0] / MM, heightMax: kin.footprint[2] / MM },
    protocolFeatures: { agvActions: ['startCharging', 'stopCharging', 'pick', 'drop', 'startPause', 'stopPause', 'cancelOrder', 'initPosition', 'stateRequest', 'factsheetRequest', 'harvest', 'spray', 'mow', 'prune', 'scout', 'pollinate', 'weed'].map((a) => ({ actionType: a, actionScopes: ['INSTANT', 'NODE'] })) },
  };
}

// ---------------------------------------------------------------------------------------------
// Per-AGV bookkeeping shared by master and bridge
// ---------------------------------------------------------------------------------------------

export interface AgvRecord {
  manufacturer: string;
  serial: string;
  connection: VdaConnection['connectionState'] | 'UNKNOWN';
  lastState?: VdaState;
  lastSeen?: number;
  factsheet?: VdaFactsheet;
  /** Order last sent by the master. */
  order?: VdaOrder;
  headerIds: Record<string, number>;
  /** Studio robot linked to this AGV (by id). */
  robotId?: string;
  taskId?: string | null;
}

export class Vda5050Master {
  agvs = new Map<string, AgvRecord>();
  /** Outgoing publisher (topic, JSON payload). */
  constructor(public send: (topic: string, payload: string, retain?: boolean) => void, public prefix = 'uagv', public mapId = 'station') {}

  key(manufacturer: string, serial: string): string { return `${manufacturer}/${serial}`; }
  agv(manufacturer: string, serial: string): AgvRecord {
    const k = this.key(manufacturer, serial);
    let a = this.agvs.get(k);
    if (!a) { a = { manufacturer, serial, connection: 'UNKNOWN', headerIds: {} }; this.agvs.set(k, a); }
    return a;
  }
  private nextHeader(a: AgvRecord, type: string): number { a.headerIds[type] = (a.headerIds[type] ?? 0) + 1; return a.headerIds[type]; }

  /** Feed any received VDA message (state/connection/visualization/factsheet). Returns the record or null. */
  receive(topic: string, payload: string | Record<string, unknown>): AgvRecord | null {
    const t = parseVdaTopic(topic);
    if (!t) return null;
    let msg: any;
    try { msg = typeof payload === 'string' ? JSON.parse(payload) : payload; } catch { return null; }
    const a = this.agv(t.manufacturer, t.serial);
    a.lastSeen = Date.now();
    if (t.type === 'state') a.lastState = msg as VdaState;
    else if (t.type === 'connection') a.connection = (msg as VdaConnection).connectionState;
    else if (t.type === 'factsheet') a.factsheet = msg as VdaFactsheet;
    else if (t.type === 'visualization' && a.lastState && msg.agvPosition) a.lastState = { ...a.lastState, agvPosition: msg.agvPosition, velocity: msg.velocity ?? a.lastState.velocity };
    return a;
  }

  /** Send an order along a path (mm). Re-sending to the same AGV with a running order creates an order update. */
  sendOrder(manufacturer: string, serial: string, path: number[][], opts: OrderOptions = {}, task?: FleetTask): VdaOrder {
    const a = this.agv(manufacturer, serial);
    const header = { headerId: this.nextHeader(a, 'order'), manufacturer, serialNumber: serial };
    const running = a.order && a.lastState && a.lastState.orderId === a.order.orderId && a.lastState.nodeStates.length > 0;
    const o = task
      ? taskToOrder(header, task, path, { mapId: this.mapId, ...opts, orderId: running ? a.order!.orderId : opts.orderId, orderUpdateId: running ? a.order!.orderUpdateId + 1 : 0 })
      : pathToOrder(header, path, { mapId: this.mapId, ...opts, orderId: running ? a.order!.orderId : opts.orderId, orderUpdateId: running ? a.order!.orderUpdateId + 1 : 0 });
    a.order = o;
    a.taskId = task?.id ?? null;
    this.send(vdaTopic(this.prefix, manufacturer, serial, 'order'), JSON.stringify(o));
    return o;
  }

  instantAction(manufacturer: string, serial: string, actionType: string, params: Record<string, string | number | boolean> = {}, blockingType: VdaAction['blockingType'] = 'HARD'): VdaInstantActions {
    const a = this.agv(manufacturer, serial);
    const msg: VdaInstantActions = { headerId: this.nextHeader(a, 'instantActions'), timestamp: nowIso(), version: VDA_VERSION, manufacturer, serialNumber: serial, actions: [makeAction(actionType, blockingType, params)] };
    this.send(vdaTopic(this.prefix, manufacturer, serial, 'instantActions'), JSON.stringify(msg));
    return msg;
  }
  cancelOrder(m: string, s: string) { return this.instantAction(m, s, 'cancelOrder'); }
  pause(m: string, s: string) { return this.instantAction(m, s, 'startPause'); }
  resume(m: string, s: string) { return this.instantAction(m, s, 'stopPause'); }
  initPosition(m: string, s: string, xMm: number, yMm: number, thetaDeg: number, mapId = this.mapId) { return this.instantAction(m, s, 'initPosition', { x: xMm / MM, y: yMm / MM, theta: toRad(thetaDeg), mapId, lastNodeId: '' }); }

  /** True when the AGV finished the order we sent (no remaining node states, all actions finished). */
  orderDone(a: AgvRecord): boolean {
    if (!a.order || !a.lastState) return false;
    if (a.lastState.orderId !== a.order.orderId) return false;
    const nodesLeft = a.lastState.nodeStates.length;
    const actionsRunning = a.lastState.actionStates.some((s) => s.actionStatus !== 'FINISHED' && s.actionStatus !== 'FAILED');
    return nodesLeft === 0 && !actionsRunning;
  }
  orderFailed(a: AgvRecord): boolean {
    return !!a.lastState && (a.lastState.errors.some((e) => e.errorLevel === 'FATAL') || a.lastState.actionStates.some((s) => s.actionStatus === 'FAILED'));
  }

  summary() {
    return [...this.agvs.values()].map((a) => ({ manufacturer: a.manufacturer, serial: a.serial, connection: a.connection, lastSeen: a.lastSeen, orderId: a.lastState?.orderId ?? a.order?.orderId ?? '', remainingNodes: a.lastState?.nodeStates.length ?? null, driving: a.lastState?.driving ?? null, battery: a.lastState?.batteryState?.batteryCharge ?? null, errors: a.lastState?.errors?.length ?? 0, position: a.lastState?.agvPosition ? { x: a.lastState.agvPosition.x * MM, y: a.lastState.agvPosition.y * MM, theta: toDeg(a.lastState.agvPosition.theta) } : null, robotId: a.robotId, taskId: a.taskId }));
  }
}

/**
 * AGV side: executes VDA orders on a simulated studio robot (digital twin). The caller drives the robot
 * with the fleet/mobile simulation and calls `tick()` to advance node bookkeeping and produce state messages.
 */
export class Vda5050Agv {
  order: VdaOrder | null = null;
  lastNodeSequenceId = -1;
  actionStates: VdaActionState[] = [];
  paused = false;
  headerIds: Record<string, number> = {};
  errors: VdaError[] = [];
  constructor(public robot: MobileRobot, public manufacturer: string, public serial: string, public drive: (robot: MobileRobot, pathMm: number[][]) => void, public stop: (robot: MobileRobot) => void, public mapId = 'station') {}

  header(type: string) { this.headerIds[type] = (this.headerIds[type] ?? 0) + 1; return { headerId: this.headerIds[type], manufacturer: this.manufacturer, serialNumber: this.serial }; }

  /** Accept an order (or order update). Returns an error description when rejected (VDA validation rules). */
  acceptOrder(o: VdaOrder): string | null {
    if (this.order && o.orderId === this.order.orderId && o.orderUpdateId <= this.order.orderUpdateId) return `orderUpdateId ${o.orderUpdateId} not newer than ${this.order.orderUpdateId}`;
    const released = o.nodes.filter((n) => n.released && n.nodePosition);
    if (!released.length) return 'order has no released nodes';
    if (this.order && o.orderId !== this.order.orderId && this.order.nodes.some((n) => n.sequenceId > this.lastNodeSequenceId && n.released) && this.robot.state.path) {
      return 'AGV still has released nodes of a running order; cancel it first';
    }
    this.errors = [];
    // order update: keep progress if the new order continues from our last node
    if (!(this.order && o.orderId === this.order.orderId)) this.lastNodeSequenceId = -1;
    this.order = o;
    this.actionStates = o.nodes.flatMap((n) => n.actions.map((a) => ({ actionId: a.actionId, actionType: a.actionType, actionStatus: 'WAITING' as const })));
    const path = released.filter((n) => n.sequenceId > this.lastNodeSequenceId).map((n) => [n.nodePosition!.x * MM, n.nodePosition!.y * MM]);
    if (path.length) { this.drive(this.robot, [[this.robot.state.x, this.robot.state.y], ...path]); this.robot.state.status = 'moving'; }
    return null;
  }

  instantActions(msg: VdaInstantActions | { actions?: VdaAction[]; instantActions?: VdaAction[] }): VdaActionState[] {
    const acts = (msg as any).actions ?? (msg as any).instantActions ?? [];
    const out: VdaActionState[] = [];
    for (const a of acts as VdaAction[]) {
      let status: VdaActionState['actionStatus'] = 'FINISHED';
      switch (a.actionType) {
        case 'cancelOrder': this.stop(this.robot); this.order = null; this.actionStates = []; this.lastNodeSequenceId = -1; this.robot.state.status = 'idle'; break;
        case 'startPause': this.paused = true; this.stop(this.robot); this.robot.state.status = 'waiting'; break;
        case 'stopPause': this.paused = false; if (this.order) this.acceptOrder({ ...this.order, orderUpdateId: this.order.orderUpdateId + 1 }); break;
        case 'initPosition': {
          const p = Object.fromEntries((a.actionParameters ?? []).map((q) => [q.key, q.value])) as any;
          this.robot.state.x = Number(p.x) * MM; this.robot.state.y = Number(p.y) * MM; this.robot.state.theta = toDeg(Number(p.theta)); this.robot.syncPoseFromState();
          break;
        }
        case 'startCharging': this.robot.state.status = 'charging'; break;
        case 'stopCharging': this.robot.state.status = 'idle'; break;
        case 'stateRequest': case 'factsheetRequest': break;
        default: status = 'FAILED';
      }
      out.push({ actionId: a.actionId, actionType: a.actionType, actionStatus: status });
    }
    this.actionStates.push(...out);
    return out;
  }

  /** Advance node bookkeeping from the robot's path progress; run node actions when the destination is reached. */
  tick(): void {
    if (!this.order) return;
    const released = this.order.nodes.filter((n) => n.released && n.nodePosition && n.sequenceId > this.lastNodeSequenceId);
    for (const n of released) {
      const d = Math.hypot(n.nodePosition!.x * MM - this.robot.state.x, n.nodePosition!.y * MM - this.robot.state.y);
      if (d <= (n.nodePosition!.allowedDeviationXY ?? 0.3) * MM + 50) {
        this.lastNodeSequenceId = n.sequenceId;
        for (const a of n.actions) { const s = this.actionStates.find((x) => x.actionId === a.actionId); if (s) s.actionStatus = a.actionType === 'startCharging' ? 'RUNNING' : 'FINISHED'; if (a.actionType === 'startCharging') this.robot.state.status = 'charging'; }
      } else break;
    }
    const allDone = this.order.nodes.every((n) => !n.released || n.sequenceId <= this.lastNodeSequenceId);
    if (allDone && this.robot.state.path === null && this.robot.state.status === 'moving') this.robot.state.status = 'idle';
  }

  state(): VdaState { return robotToState(this.robot, { header: this.header('state'), order: this.order, lastNodeSequenceId: this.lastNodeSequenceId, actionStates: this.actionStates, mapId: this.mapId, errors: this.errors }); }
  connection(state: VdaConnection['connectionState']): VdaConnection { return { ...this.header('connection'), timestamp: nowIso(), version: VDA_VERSION, connectionState: state }; }
  visualization(): VdaVisualization { const s = this.state(); return { ...this.header('visualization'), timestamp: s.timestamp, version: VDA_VERSION, agvPosition: s.agvPosition, velocity: s.velocity }; }
  factsheet(): VdaFactsheet { return factsheetFor(this.robot, this.header('factsheet')); }
}
