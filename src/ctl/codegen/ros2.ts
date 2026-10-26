/**
 * ROS 2 packages generated from control models (built with colcon on ROS 2 Jazzy):
 *
 *  - `<pkg>_control` (ament_python): `supervisor` node — the model (automaton + its supervisor, Petri net, FSM or
 *    behavior tree) run by the same Python runtime as the RoboDK station script (py_runtime.ts) over a ROS link:
 *    robot programs / skills / goto / moves are `std_srvs/Trigger` services (`/<ns>/program/<name>`, `/<ns>/skill/<name>`,
 *    `/<ns>/goto/<zone>`, `/<ns>/move/<target>`), signals arrive as JSON on `/<ns>/signals` and are written to
 *    `/<ns>/signals/set`; the active state is published on `/<ns>/fsm/state` (telemetry, like /murom/fsm/state).
 *    `skill_server` is a simulated cell answering those services with the station's program durations (sim:=true in the
 *    launch file). For an FSM model the package also has `yasmin_fsm`: the same machine as a yasmin StateMachine
 *    (nested machines, outcome maps, blackboard decisions, entry / exit, timeouts, cancel) over the same link.
 *  - `<pkg>_bt` (ament_cmake, for behavior trees): the tree as BehaviorTree.CPP v4 XML (py_trees-style reactive
 *    sequences → ReactiveSequence, memory sequences → Sequence, parallel thresholds, Retry / Repeat / Timeout, conditions
 *    → ScriptCondition) with C++ skill stubs (StatefulActionNode calling the Trigger services) and a `bt_runner`.
 */
import type { ControlKind } from '../model';
import { parseBt } from '../dsl';
import { parseFsm, fsmMachines, stateOutcomes, FsmMachine, FsmState } from '../fsm';
import type { BTNodeSpec } from '../bt';
import { synthesizeSupervisor } from '../supervisor';
import { runtimeModel } from './robodk';
import { PY_RUNTIME, PY_ROBODK_IMPORT } from './py_runtime';
import { GenFile, ident, xmlEsc, translateExpr, ECMA_DIALECT, exprIds, parseXml, XmlNode } from './common';

export interface Ros2Options { /** topic / service namespace (default: the package name) */ namespace?: string; supervisorSource?: string; /** simulated skill durations (s) by program / skill name */ durations?: Record<string, number>; /** program → robot (lanes of the runtime: one action per robot) */ robots?: Record<string, string>; /** signals a program sets (SetDO / setParam instructions of the station program), applied by the simulated skill server when the program ends */ outputs?: Record<string, Array<[string, number | boolean | string]>>; maintainer?: string }

const pkgName = (name: string) => (ident(name).toLowerCase().replace(/_+/g, '_').replace(/^_|_$/g, '') || 'control').replace(/^(\d)/, 'm$1');

/** The runtime core for ROS: robolink import replaced by item-type constants (the link is ros_link.RosLink). */
const PY_CORE = PY_RUNTIME.replace(PY_ROBODK_IMPORT, '\nITEM_TYPE_STATION, ITEM_TYPE_ROBOT, ITEM_TYPE_TARGET, ITEM_TYPE_PROGRAM = 1, 2, 6, 8\n');

const ROS_LINK = String.raw`"""robolink-like API over ROS 2: programs, skills, goto and moves are std_srvs/Trigger services, signals are JSON topics."""
import json

from std_msgs.msg import String
from std_srvs.srv import Trigger

ITEM_TYPE_STATION, ITEM_TYPE_ROBOT, ITEM_TYPE_TARGET, ITEM_TYPE_PROGRAM = 1, 2, 6, 8


def srv_name(name):
    return ''.join(c if c.isalnum() or c == '_' else '_' for c in str(name))


class RosItem(object):
    def __init__(self, link, name, typ):
        self.link, self.name, self.typ = link, name, typ
        self.future = None
        self.failed = False

    def service(self):
        if self.name.startswith('goto_'):
            return '%s/goto/%s' % (self.link.ns, srv_name(self.name[5:]))
        kind = 'program' if self.name in self.link.programs else 'skill'
        return '%s/%s/%s' % (self.link.ns, kind, srv_name(self.name))

    def Valid(self):
        if self.typ == ITEM_TYPE_ROBOT:
            return bool(self.name)
        if self.typ == ITEM_TYPE_TARGET:
            return self.link.client('%s/move/%s' % (self.link.ns, srv_name(self.name))).service_is_ready()
        return self.link.client(self.service()).service_is_ready()

    def Name(self):
        return self.name

    def getLink(self, typ):
        return RosItem(self.link, self.link.robots.get(self.name, 'prog:' + self.name), ITEM_TYPE_ROBOT)

    def _call(self, srv):
        self.failed = False
        self.future = self.link.client(srv).call_async(Trigger.Request())

    def RunProgram(self):
        self._call(self.service())

    def MoveJ(self, target, blocking=True):
        self._call('%s/move/%s' % (self.link.ns, srv_name(target.name)))

    def MoveL(self, target, blocking=True):
        self._call('%s/move_linear/%s' % (self.link.ns, srv_name(target.name)))

    def Busy(self):
        f = self.future
        if f is None:
            return False
        if not f.done():
            return True
        r = f.result()
        self.failed = r is None or not r.success
        if r is not None:
            self.link.apply_signals(r.message)
        if self.failed:
            self.link.node.get_logger().warn('%s failed: %s' % (self.name, r.message if r is not None else 'no response'))
        self.future = None
        return False

    def Failed(self):
        return self.failed

    def Stop(self):
        if self.future is not None:
            self.future.cancel()
            self.future = None
        self.link.halt_pub.publish(String(data=self.name))


class RosLink(object):
    """What the runtime calls on robolink.Robolink: Item(), getParam(), setParam()."""

    def __init__(self, node, ns, programs, robots):
        self.node, self.ns, self.programs, self.robots = node, ns, set(programs), dict(robots)
        self.clients, self.values = {}, {}
        self.set_pub = node.create_publisher(String, ns + '/signals/set', 10)
        self.halt_pub = node.create_publisher(String, ns + '/halt', 10)
        node.create_subscription(String, ns + '/signals', self._on_signals, 10)

    def _on_signals(self, msg):
        try:
            self.values.update(json.loads(msg.data))
        except ValueError:
            self.node.get_logger().warn('bad signals message: %s' % msg.data)

    def client(self, name):
        c = self.clients.get(name)
        if c is None:
            c = self.clients[name] = self.node.create_client(Trigger, name)
        return c

    def Item(self, name, typ=-1):
        return RosItem(self, name, typ)

    def getParam(self, name):
        v = self.values.get(name)
        return None if v is None else str(v)

    def apply_signals(self, message):
        """Signals a skill reports with its response (a line 'SIGNALS name=value;...' of the Trigger message): they
        are known before the runtime sees the skill finished, as the program outputs in the studio / RoboDK."""
        for line in str(message or '').splitlines():
            if not line.startswith('SIGNALS '):
                continue
            for kv in line[8:].split(';'):
                if '=' in kv:
                    k, v = kv.split('=', 1)
                    try:
                        self.values[k.strip()] = json.loads(v)
                    except ValueError:
                        self.values[k.strip()] = v

    def setParam(self, name, value):
        self.values[name] = value
        self.set_pub.publish(String(data=json.dumps({'name': name, 'value': value})))
`;

const SUPERVISOR_NODE = String.raw`"""Control model node: runs the generated model with the studio's semantics over the ROS link."""
import json
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from . import vbs_runtime as rt
from .model import MODEL, PROGRAMS, ROBOTS, SERVICES
from .ros_link import RosLink


class SupervisorNode(Node):
    def __init__(self):
        super().__init__('supervisor')
        self.declare_parameter('namespace', MODEL.get('namespace', '/vbs'))
        self.declare_parameter('limit', 3600.0)
        self.declare_parameter('discovery', 20.0)
        ns = self.get_parameter('namespace').value
        self.limit = float(self.get_parameter('limit').value)
        self.link = RosLink(self, ns, PROGRAMS, ROBOTS)
        self.state_pub = self.create_publisher(String, ns + '/fsm/state', 10)
        self.result_pub = self.create_publisher(String, ns + '/result', 10)
        deadline = time.time() + float(self.get_parameter('discovery').value)
        missing = [s for s in SERVICES if not self.link.client(ns + s).wait_for_service(timeout_sec=max(0.0, deadline - time.time()))]
        if missing:
            self.get_logger().warn('services not available: %s' % ', '.join(missing))
        if SERVICES:
            time.sleep(1.5)  # let the servers match our clients: a request sent before that is lost
        self.world = rt.World(self.link, MODEL.get('signals', []))
        rt.start_clock(self.link)
        self.run = {'des': rt.AutomatonRun, 'petri': rt.PetriRun, 'fsm': rt.FsmRun, 'bt': rt.BtRun}[MODEL['kind']](MODEL['model'], self.world)
        self.done = False
        self.timer = self.create_timer(rt.DT, self.step)

    def step(self):
        if self.done:
            return
        self.run.tick(rt.read_clock())
        self.state_pub.publish(String(data=str(self.run.where())))
        if self.run.status != 'running' or rt.now() >= self.limit:
            verdict = self.run.verdict if self.run.status != 'running' else 'timeout'
            outcome = getattr(self.run, 'outcome', None)
            line = 'RESULT\t%s\t%.2f\t%s%s' % (verdict, rt.now(), self.run.where(), ('\t' + str(outcome)) if outcome else '')
            print(line, flush=True)
            self.result_pub.publish(String(data=json.dumps({'verdict': verdict, 'time': rt.now(), 'where': str(self.run.where()), 'outcome': outcome})))
            self.done = True


def main(args=None):
    rclpy.init(args=args)
    node = SupervisorNode()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
`;

const SKILL_SERVER = String.raw`"""Simulated cell: answers the Trigger services of the model after the station's program durations, publishes the signals."""
import json
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .model import DURATIONS, MODEL, OUTPUTS, SERVICES


class SkillServer(Node):
    def __init__(self):
        super().__init__('skill_server')
        self.declare_parameter('namespace', MODEL.get('namespace', '/vbs'))
        self.declare_parameter('speed', 1.0)
        ns = self.get_parameter('namespace').value
        self.speed = float(self.get_parameter('speed').value)
        self.values = {}
        self.pub = self.create_publisher(String, ns + '/signals', 10)
        self.updates = self.create_publisher(String, ns + '/signal_updates', 10)
        self.create_subscription(String, ns + '/signals/set', self.on_set, 10)
        group = ReentrantCallbackGroup()
        for s in SERVICES:
            self.create_service(Trigger, ns + s, self.handler(s), callback_group=group)
        self.create_timer(0.2, self.publish)

    def handler(self, srv):
        key = srv.split('/', 2)[-1]
        seconds = float(DURATIONS.get(key, DURATIONS.get(srv.rsplit('/', 1)[-1], 1.0)))
        outputs = OUTPUTS.get(key, OUTPUTS.get(srv.rsplit('/', 1)[-1], []))

        def run(req, res):
            self.get_logger().info('serving %s (%.2f s)' % (srv, seconds))
            time.sleep(seconds / max(self.speed, 1e-3))
            # the station program's outputs (SetDO Ready, 1 ...): published and reported with the response
            for name, value in outputs:
                self.values[name] = value
                self.updates.publish(String(data='%s=%s' % (name, json.dumps(value))))
            if outputs:
                self.publish()
            res.success = True
            res.message = '%s done in %.2f s' % (srv, seconds) + (('\nSIGNALS ' + ';'.join('%s=%s' % (n, json.dumps(v)) for n, v in outputs)) if outputs else '')
            self.get_logger().info('done %s' % srv)
            return res
        return run

    def on_set(self, msg):
        d = json.loads(msg.data)
        self.values[d['name']] = d['value']
        self.updates.publish(String(data='%s=%s' % (d['name'], json.dumps(d['value']))))
        self.publish()

    def publish(self):
        self.pub.publish(String(data=json.dumps(self.values)))


def main(args=None):
    rclpy.init(args=args)
    node = SkillServer()
    ex = MultiThreadedExecutor(num_threads=8)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
`;

/** The services a model calls (relative to the namespace). */
function servicesOf(m: ReturnType<typeof runtimeModel>): string[] {
  const out = new Set<string>(); const visit = (b: Record<string, unknown> | null | undefined) => {
    if (!b) return; const v = String(b.value ?? ''); const s = (x: string) => x.replace(/[^A-Za-z0-9_]/g, '_');
    if (b.kind === 'program') out.add(`/program/${s(v)}`); else if (b.kind === 'skill') out.add(`/skill/${s(v)}`); else if (b.kind === 'goto') out.add(`/goto/${s(v)}`);
    else if (b.kind === 'target') out.add(`/${(b.args as Record<string, unknown> | undefined)?.linear === true || (b.args as Record<string, unknown> | undefined)?.linear === 'true' ? 'move_linear' : 'move'}/${s(v)}`);
  };
  const walk = (x: unknown): void => { if (Array.isArray(x)) { x.forEach(walk); return; } if (!x || typeof x !== 'object') return; const o = x as Record<string, unknown>; if (typeof o.kind === 'string' && 'value' in o && 'args' in o) visit(o); for (const v of Object.values(o)) walk(v); };
  walk(m.model);
  return [...out].sort();
}

// ---------------------------------------------------------------------------------------------
// yasmin FSM
// ---------------------------------------------------------------------------------------------

const YASMIN_HEAD = String.raw`"""yasmin state machine generated by VerticalBot Studio (same structure and semantics as the studio FSM executor)."""
import time

from yasmin import State, StateMachine, Blackboard

from . import vbs_runtime as rt

DT = 0.05


class Ctx(object):
    """World (robolink-like link + blackboard), sensors and the cancel flag shared by the states."""

    def __init__(self, world, sensors):
        self.w, self.sensors, self.t0 = world, rt.Sensors(sensors), time.time()
        self.events = []

    def now(self):
        return time.time() - self.t0

    def sense(self):
        self.w.bb.update(self.w.sense())
        evs = list(self.w.events) + self.sensors.poll(self.w.bb)
        del self.w.events[:]
        self.w.bb['event'] = evs[-1] if evs else ''
        return evs

    def instant(self, b):
        a = rt.Action(self.w, b, self.now())
        if a.tick(self.now()) == 'running':
            a.halt()


def mapped(trans, outcome, fail, outcomes):
    if outcome in trans:
        return trans[outcome]
    if outcome == 'timeout' and fail in trans:
        return trans[fail]
    return outcome


class VState(State):
    """Skill, decision or pass state with entry / exit actions and a timeout."""

    def __init__(self, ctx, path, s):
        self.ctx, self.path, self.s = ctx, path, s
        super().__init__(s['outcomes'])

    def outcome(self):
        s, ctx = self.s, self.ctx
        t0 = ctx.now()
        a = rt.Action(ctx.w, s['action'], t0) if s['kind'] == 'skill' else None
        while True:
            ctx.sense()
            if self.is_canceled():
                if a:
                    a.halt()
                return 'canceled'
            if s['timeout'] is not None and ctx.now() - t0 >= s['timeout'] - 1e-9:
                if a:
                    a.halt()
                return 'timeout'
            if s['kind'] == 'pass':
                return 'ok'
            if s['kind'] == 'decision':
                for expr, o in s['when']:
                    if rt.evaluate(expr, ctx.w.bb):
                        return o
                if s['otherwise']:
                    return s['otherwise']
            else:
                st = a.tick(ctx.now())
                named = rt.named_outcomes(s)
                if st == 'success':
                    return a.outcome or (named[0] if named else (s['action'].get('done') or 'ok'))
                if st == 'failure':
                    return 'timeout' if a.timed_out else (a.outcome or (named[-1] if named else (s['action'].get('fail') or 'fail')))
            time.sleep(DT)

    def execute(self, blackboard):
        self.ctx.w.bb['fsm_state'] = self.path + '/' + self.s['name']
        for b in self.s['entry']:
            self.ctx.instant(b)
        o = self.outcome()
        for b in self.s['exit']:
            self.ctx.instant(b)
        rt.trace('outcome', '%s/%s:%s' % (self.path, self.s['name'], o))
        blackboard['fsm_state'] = self.path + '/' + self.s['name']
        return o


class VMachine(State):
    """A nested machine as a state: entry, the inner StateMachine, exit."""

    def __init__(self, ctx, path, s, sm):
        self.ctx, self.path, self.s, self.sm = ctx, path, s, sm
        super().__init__(s['outcomes'])

    def execute(self, blackboard):
        for b in self.s['entry']:
            self.ctx.instant(b)
        o = self.sm(blackboard)
        for b in self.s['exit']:
            self.ctx.instant(b)
        rt.trace('outcome', '%s/%s:%s' % (self.path, self.s['name'], o))
        return o

    def cancel_state(self):
        self.sm.cancel_state()
        super().cancel_state()


def build(ctx, mach, path):
    sm = StateMachine(outcomes=mach['outcomes'])
    for s in mach['states']:
        if s['kind'] == 'machine':
            st = VMachine(ctx, path, s, build(ctx, s['machine'], path + '/' + s['name']))
        else:
            st = VState(ctx, path, s)
        sm.add_state(s['name'], st, transitions=s['map'])
    sm.set_start_state(mach['initial'])
    return sm


def build_fsm(link, model):
    """The root StateMachine over a robolink-like link (ros_link.RosLink, robolink.Robolink or a stand-in)."""
    world = rt.World(link, model.get('signals', []))
    world.bb.update(model['model'].get('bb', {}))
    ctx = Ctx(world, model['model'].get('sensors', []))
    rt.T0 = time.time()
    return build(ctx, model['yasmin'], model['yasmin']['path']), ctx
`;

const YASMIN_MAIN = String.raw`

def main(args=None):
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    import yasmin_ros
    from .model import MODEL, PROGRAMS, ROBOTS
    from .ros_link import RosLink
    import threading
    rclpy.init(args=args)
    node = Node('yasmin_fsm')
    yasmin_ros.set_ros_loggers()
    ns = node.declare_parameter('namespace', MODEL.get('namespace', '/vbs')).value
    link = RosLink(node, ns, PROGRAMS, ROBOTS)
    from .model import SERVICES
    deadline = time.time() + float(node.declare_parameter('discovery', 20.0).value)
    missing = [x for x in SERVICES if not link.client(ns + x).wait_for_service(timeout_sec=max(0.0, deadline - time.time()))]
    if missing:
        node.get_logger().warn('services not available: %s' % ', '.join(missing))
    if SERVICES:
        time.sleep(1.5)  # let the servers match our clients
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()
    sm, ctx = build_fsm(link, MODEL)
    bb = Blackboard()
    outcome = sm(bb)
    print('RESULT\tdone\t%.2f\t%s' % (ctx.now(), outcome), flush=True)
    node.create_publisher(String, ns + '/result', 10).publish(String(data=outcome))
    rclpy.shutdown()


if __name__ == '__main__':
    main()
`;

/** The FSM as the yasmin builder's data: per state its full outcome list and the complete outcome → target map. */
export function yasminModel(source: string): Record<string, unknown> {
  const doc = parseFsm(source); const cancel = 'canceled';
  const needsTimeout = new Set<FsmMachine>();
  const conv = (m: FsmMachine, path: string): Record<string, unknown> => {
    const outs = new Set([...m.outcomes, cancel]);
    const states = m.states.map((s: FsmState) => {
      const outcomes = [...new Set([...stateOutcomes(s), cancel])];
      const map: Record<string, string> = {};
      for (const o of outcomes) {
        let to = s.transitions[o]; if (to === undefined && o === 'timeout') to = s.transitions[s.action?.fail ?? 'fail'];
        if (to === undefined && m.outcomes.includes(o)) to = o;
        if (to === undefined) { to = o; outs.add(o); if (o === 'timeout') needsTimeout.add(m); }
        map[o] = to;
      }
      const base = { name: s.name, kind: s.kind, entry: s.entry, exit: s.exit, timeout: s.timeout ?? null, outcomes, map };
      if (s.kind === 'machine' && s.machine) return { ...base, timeout: null, machine: conv(s.machine, `${path}/${s.name}`) };
      return { ...base };
    });
    return { name: m.name, path, outcomes: [...outs], initial: (m.states.find((s) => s.initial) ?? m.states[0]).name, states };
  };
  const root = conv(doc.root, fsmMachines(doc)[0].path);
  // a nested machine's outcome list must contain what its children propagate (unmapped timeout / canceled)
  const fix = (mm: Record<string, unknown>) => { for (const s of mm.states as Array<Record<string, unknown>>) if (s.machine) { fix(s.machine as Record<string, unknown>); const inner = (s.machine as { outcomes: string[] }).outcomes; const outs = s.outcomes as string[]; for (const o of inner) if (!outs.includes(o)) { outs.push(o); (s.map as Record<string, string>)[o] = (s.map as Record<string, string>)[o] ?? o; if (!(mm.outcomes as string[]).includes(o)) (mm.outcomes as string[]).push(o); } } };
  fix(root); void needsTimeout;
  return root;
}

/** Translate the bindings / expressions of the yasmin data like the runtime model does (Python expressions). */
function yasminData(source: string): Record<string, unknown> {
  const rm = runtimeModel('fsm', source); const y = yasminModel(source);
  // reuse the runtime model's translated bindings and decisions (same tree shape: machines → states)
  const merge = (ym: Record<string, unknown>, rmach: Record<string, unknown>) => {
    const rs = rmach.states as Array<Record<string, unknown>>;
    (ym.states as Array<Record<string, unknown>>).forEach((s, i) => { const r = rs[i]; s.action = r.action; s.entry = r.entry; s.exit = r.exit; s.when = r.when; s.otherwise = r.otherwise; if (s.machine) merge(s.machine as Record<string, unknown>, r.machine as Record<string, unknown>); });
  };
  merge(y, (rm.model as { root: Record<string, unknown> }).root);
  return y;
}

// ---------------------------------------------------------------------------------------------
// BehaviorTree.CPP v4
// ---------------------------------------------------------------------------------------------

const pascal = (s: string) => s.split(/[^A-Za-z0-9]+/).filter(Boolean).map((w) => w[0].toUpperCase() + w.slice(1)).join('') || 'Skill';
const SKILL_KIND: Record<string, { kind: string; key: string }> = { program: { kind: 'program', key: 'program' }, goto: { kind: 'goto', key: 'zone' }, move: { kind: 'move', key: 'target' } };
const scriptExpr = (e: string) => translateExpr(e, ECMA_DIALECT);
const scriptVal = (v: unknown) => (typeof v === 'number' || typeof v === 'boolean' ? String(v) : `'${String(v).replace(/'/g, '')}'`);

/** The tree as BT.CPP v4 XML plus the custom action node types it uses (ID → ports). */
export function btCppXml(source: string): { xml: string; skills: Map<string, { fn: string; ports: Set<string> }>; vars: Map<string, 'bool' | 'double'> } {
  const doc = parseBt(source); const skills = new Map<string, { fn: string; ports: Set<string> }>(); const vars = new Map<string, 'bool' | 'double'>();
  const noteVars = (e: string) => { for (const id of exprIds(e)) { const numeric = new RegExp(`\\b${id}\\s*(<|>|<=|>=|\\+|-|\\*|/)|(<|>|<=|>=|\\+|-|\\*|/)\\s*${id}\\b`).test(e); if (numeric || vars.get(id) === 'double') vars.set(id, 'double'); else if (!vars.has(id)) vars.set(id, 'bool'); } };
  const nm = (n: BTNodeSpec) => (n.name && n.name !== n.type && n.name !== n.fn ? ` name="${xmlEsc(n.name)}"` : '');
  const out: string[] = [];
  const emit = (n: BTNodeSpec, ind: string): void => {
    const kids = (i2: string) => (n.children ?? []).forEach((c) => emit(c, i2));
    const wrap = (open: string, close: string) => { out.push(`${ind}${open}`); kids(ind + '  '); out.push(`${ind}${close}`); };
    switch (n.type) {
      case 'sequence': { const t = n.memory ? 'Sequence' : 'ReactiveSequence'; return wrap(`<${t}${nm(n)}>`, `</${t}>`); }
      case 'fallback': { const t = n.memory ? 'Fallback' : 'ReactiveFallback'; return wrap(`<${t}${nm(n)}>`, `</${t}>`); }
      case 'parallel': { const k = n.children?.length ?? 0; const M = n.threshold ?? k; return wrap(`<Parallel${nm(n)} success_count="${M}" failure_count="${Math.max(1, k - M + 1)}">`, '</Parallel>'); }
      case 'inverter': return wrap(`<Inverter${nm(n)}>`, '</Inverter>');
      case 'force_success': return wrap(`<ForceSuccess${nm(n)}>`, '</ForceSuccess>');
      case 'force_failure': return wrap(`<ForceFailure${nm(n)}>`, '</ForceFailure>');
      case 'retry': return wrap(`<RetryUntilSuccessful${nm(n)} num_attempts="${(n.count ?? 1) + 1}">`, '</RetryUntilSuccessful>');
      case 'repeat': return wrap(`<Repeat${nm(n)} num_cycles="${n.count ?? 1}">`, '</Repeat>');
      case 'timeout': return wrap(`<Timeout${nm(n)} msec="${Math.round((n.seconds ?? 1) * 1000)}">`, '</Timeout>');
      case 'condition': noteVars(n.fn ?? 'false'); out.push(`${ind}<ScriptCondition${nm(n)} code="${xmlEsc(scriptExpr(n.fn ?? 'false'))}"/>`); return;
      case 'action': {
        const a = (n.args ?? {}) as Record<string, unknown>; const fn = n.fn ?? 'skill';
        const pre = n.pre ? (noteVars(n.pre), ` _failureIf="${xmlEsc(`!(${scriptExpr(n.pre)})`)}"`) : '';
        let el: string;
        if (fn === 'wait') el = `<Sleep${nm(n)} msec="${Math.round(Number(a.seconds ?? 1) * 1000)}"${pre}/>`;
        else if (fn === 'set') { const parts = Object.entries(a).filter(([k]) => !['event', 'done', 'robot'].includes(k)).map(([k, v]) => `${k.replace(/\./g, '_')}:=${typeof v === 'string' && v.startsWith('=') ? scriptExpr(v.slice(1)) : scriptVal(v)}`); parts.forEach((p) => vars.has(p.split(':=')[0]) || vars.set(p.split(':=')[0], 'double')); el = `<Script${nm(n)} code="${xmlEsc(parts.join('; ') || 'flag:=true')}"${pre}/>`; }
        else if (fn === 'report') { vars.set('reported', 'bool'); el = `<Script${nm(n)} code="reported:=true"${pre}/>`; }
        else if (fn === 'select_target') { vars.set('targets', 'double'); el = `<ScriptCondition${nm(n)} code="targets &gt; 0"/>`; }
        else if (fn === 'await') { const e = String(a.expr ?? a.await ?? 'true'); noteVars(e); el = `<WaitCondition${nm(n)} code="${xmlEsc(scriptExpr(e))}"${pre}/>`; registerSkill('WaitCondition', 'await', ['code']); }
        else {
          const id = pascal(fn); const sk = registerSkill(id, fn, Object.keys(a).filter((k) => !['event', 'done', 'ok', 'miss', 'put', 'arrive'].includes(k)));
          const attrs = [...sk.ports].filter((p) => a[p] !== undefined).map((p) => ` ${p}="${xmlEsc(String(a[p]))}"`).join('');
          el = `<${id}${nm(n)}${attrs}${pre}/>`;
        }
        if (n.timeout !== undefined) { out.push(`${ind}<Timeout msec="${Math.round(n.timeout * 1000)}">`); out.push(`${ind}  ${el}`); out.push(`${ind}</Timeout>`); } else out.push(`${ind}${el}`);
      }
    }
  };
  function registerSkill(id: string, fn: string, ports: string[]) { let s = skills.get(id); if (!s) { s = { fn, ports: new Set() }; skills.set(id, s); } for (const p of ports) s.ports.add(p); const k = SKILL_KIND[fn]; if (k) s.ports.add(k.key); return s; }
  emit(doc.root, '    ');
  const model = [...skills].map(([id, s]) => `    <Action ID="${id}">${[...s.ports].map((p) => `<input_port name="${p}"/>`).join('')}</Action>`);
  const xml = [`<?xml version="1.0" encoding="UTF-8"?>`, `<!-- ${xmlEsc(doc.name ?? 'tree')}: generated by VerticalBot Studio. py_trees-style sequences / fallbacks without memory are ReactiveSequence / ReactiveFallback. -->`, `<root BTCPP_format="4" main_tree_to_execute="MainTree">`, `  <BehaviorTree ID="MainTree">`, ...out, `  </BehaviorTree>`, `  <TreeNodesModel>`, ...model, `  </TreeNodesModel>`, `</root>`, ''].join('\n');
  return { xml, skills, vars };
}

/**
 * BehaviorTree.CPP v4 XML → BT document (import / round trip): ReactiveSequence / ReactiveFallback → non-memory
 * sequence / fallback, Sequence / Fallback / SequenceWithMemory → memory, Parallel success_count, RetryUntilSuccessful
 * (n attempts = retry n − 1), Repeat, Timeout, Inverter, ForceSuccess / ForceFailure, ScriptCondition → condition,
 * Script `k:=v` → set, Sleep → wait, custom action nodes → actions named in snake_case with their attributes as
 * arguments, `_failureIf="!(e)"` → pre="e". SubTree references are expanded.
 */
export function btCppXmlToDsl(xml: string): string {
  const root = parseXml(xml); if (root.name !== 'root') throw new Error('BT.CPP XML: <root> expected');
  const trees = root.children.filter((c) => c.name === 'BehaviorTree'); if (!trees.length) throw new Error('BT.CPP XML: no <BehaviorTree>');
  const main = trees.find((t) => t.attrs.ID === root.attrs.main_tree_to_execute) ?? trees[0];
  const snake = (id: string) => id.replace(/([a-z0-9])([A-Z])/g, '$1_$2').toLowerCase();
  const back = (code: string) => code.replace(/\s*==\s*/g, ' == ').replace(/!\s+/g, '!').trim();
  const q = (v: string) => (/^[\w.-]+$/.test(v) ? v : `"${v.replace(/"/g, "'")}"`);
  const out: string[] = [`bt ${main.attrs.ID === 'MainTree' ? 'imported tree' : main.attrs.ID}`];
  const nm = (n: XmlNode) => (n.attrs.name ? ` name=${q(n.attrs.name)}` : '');
  const emit = (n: XmlNode, d: number, depth = 0): void => {
    if (depth > 32) throw new Error('BT.CPP XML: SubTree recursion');
    const ind = '  '.repeat(d); const kids = () => n.children.forEach((c) => emit(c, d + 1, depth));
    const pre = n.attrs._failureIf ? (/^!\((.*)\)$/.exec(n.attrs._failureIf)?.[1] ?? `!(${n.attrs._failureIf})`) : undefined; const preA = pre ? ` pre=${q(back(pre))}` : '';
    switch (n.name) {
      case 'ReactiveSequence': out.push(`${ind}sequence${nm(n)}`); return kids();
      case 'Sequence': case 'SequenceWithMemory': case 'SequenceStar': out.push(`${ind}sequence${nm(n)} memory`); return kids();
      case 'ReactiveFallback': out.push(`${ind}fallback${nm(n)}`); return kids();
      case 'Fallback': out.push(`${ind}fallback${nm(n)} memory`); return kids();
      case 'Parallel': out.push(`${ind}parallel ${n.attrs.success_count ?? n.children.length}${nm(n)}`); return kids();
      case 'Inverter': out.push(`${ind}inverter${nm(n)}`); return kids();
      case 'ForceSuccess': out.push(`${ind}force_success${nm(n)}`); return kids();
      case 'ForceFailure': out.push(`${ind}force_failure${nm(n)}`); return kids();
      case 'RetryUntilSuccessful': out.push(`${ind}retry ${Math.max(0, Number(n.attrs.num_attempts ?? 1) - 1)}${nm(n)}`); return kids();
      case 'Repeat': out.push(`${ind}repeat ${n.attrs.num_cycles ?? 1}${nm(n)}`); return kids();
      case 'Timeout': out.push(`${ind}timeout ${Number(n.attrs.msec ?? 1000) / 1000}${nm(n)}`); return kids();
      case 'ScriptCondition': out.push(`${ind}condition "${back(n.attrs.code ?? 'false').replace(/"/g, "'")}"${nm(n)}`); return;
      case 'Sleep': out.push(`${ind}action wait seconds=${Number(n.attrs.msec ?? 0) / 1000}${nm(n)}${preA}`); return;
      case 'Script': {
        const parts = (n.attrs.code ?? '').split(';').map((x) => x.trim()).filter(Boolean).map((x) => { const m = /^([\w.]+)\s*:=\s*(.+)$/.exec(x); if (!m) throw new Error(`BT.CPP Script '${x}': only assignments are imported`); const v = m[2].trim(); return [m[1], /^'.*'$/.test(v) ? v.slice(1, -1) : /^(true|false|-?\d+(\.\d+)?)$/.test(v) ? v : `=${back(v)}`] as const; });
        if (parts.length === 1 && parts[0][0] === 'reported' && parts[0][1] === 'true') out.push(`${ind}action report${nm(n)}${preA}`);
        else out.push(`${ind}action set ${parts.map(([k, v]) => `${k}=${q(v)}`).join(' ')}${nm(n)}${preA}`);
        return;
      }
      case 'SubTree': { const t = trees.find((x) => x.attrs.ID === n.attrs.ID); if (!t) throw new Error(`BT.CPP XML: SubTree ${n.attrs.ID} not found`); t.children.forEach((c) => emit(c, d, depth + 1)); return; }
      default: {
        if (n.children.length) throw new Error(`BT.CPP XML: unknown control / decorator node <${n.name}>`);
        const args = Object.entries(n.attrs).filter(([k]) => k !== 'name' && !k.startsWith('_')).map(([k, v]) => ` ${k}=${q(v)}`).join('');
        out.push(`${ind}action ${n.name === 'WaitCondition' ? 'await' : snake(n.name)}${args.replace(' code=', ' expr=')}${nm(n)}${preA}`);
      }
    }
  };
  main.children.forEach((c) => emit(c, 0));
  return out.join('\n');
}

function btCppPackage(source: string, name: string, pkg: string, ns: string): GenFile[] {
  const { xml, skills, vars } = btCppXml(source); const bp = `${pkg}_bt`;
  const classes = [...skills].map(([id, s]) => {
    const k = s.fn === 'await' ? null : SKILL_KIND[s.fn] ?? { kind: 'skill', key: '' };
    const ports = [...s.ports].map((p) => `BT::InputPort<std::string>("${p}")`).join(', ');
    if (!k) return `/** Wait until a script condition over the blackboard holds. */
class ${id} : public BT::StatefulActionNode {
 public:
  ${id}(const std::string& n, const BT::NodeConfig& c) : BT::StatefulActionNode(n, c) {}
  static BT::PortsList providedPorts() { return { ${ports} }; }
  BT::NodeStatus onStart() override { return check(); }
  BT::NodeStatus onRunning() override { return check(); }
  void onHalted() override {}
 private:
  BT::NodeStatus check() {
    auto code = getInput<std::string>("code"); if (!code) return BT::NodeStatus::FAILURE;
    if (!exec_) exec_ = BT::ParseScript(code.value()).value();
    BT::Ast::Environment env{config().blackboard, config().enums};
    return exec_(env).cast<bool>() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
  }
  BT::ScriptFunction exec_;
};`;
    return `/** ${s.fn}: Trigger service ${ns}/${k.kind}/<${k.key || s.fn}> (stub: replace by your skill's action client). */
class ${id} : public TriggerSkill {
 public:
  ${id}(const std::string& n, const BT::NodeConfig& c, rclcpp::Node::SharedPtr node) : TriggerSkill(n, c, node, "${k.kind}", "${k.key}", "${s.fn}") {}
  static BT::PortsList providedPorts() { return { ${ports} }; }
};`;
  });
  const reg = [...skills].map(([id, s]) => (s.fn === 'await' ? `  factory.registerNodeType<${id}>("${id}");` : `  factory.registerNodeType<${id}>("${id}", node);`));
  const init = [...vars].map(([v, t]) => `  bb->set<${t === 'bool' ? 'bool' : 'double'}>("${v}", ${t === 'bool' ? 'false' : v === 'targets' ? '1.0' : '0.0'});`);
  const cpp = `// Skill stubs and runner for the behavior tree "${name}" (BehaviorTree.CPP v4), generated by VerticalBot Studio.
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/scripting/script_parser.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <chrono>
#include <string>
#include <thread>

static std::string g_ns = "${ns}";
static double g_discovery = 10.0;  // seconds a skill waits for its service (startup of the skill servers)
static std::string srvName(std::string s) { for (auto& c : s) if (!isalnum(static_cast<unsigned char>(c)) && c != '_') c = '_'; return s; }

/** Calls a std_srvs/Trigger service when started; SUCCESS / FAILURE from the response. */
class TriggerSkill : public BT::StatefulActionNode {
 public:
  TriggerSkill(const std::string& n, const BT::NodeConfig& c, rclcpp::Node::SharedPtr node, std::string kind, std::string key, std::string fn)
      : BT::StatefulActionNode(n, c), node_(std::move(node)), kind_(std::move(kind)), key_(std::move(key)), fn_(std::move(fn)) {}
  /** The service of this node (empty when a port is missing). */
  std::string service() {
    std::string arg = fn_;
    if (!key_.empty()) { auto v = getInput<std::string>(key_); if (!v) return ""; arg = v.value(); }
    return g_ns + "/" + kind_ + "/" + srvName(arg);
  }
  /** Create the client and wait for the server (called for every skill before the first tick). */
  bool prepare() {
    const std::string srv = service(); if (srv.empty()) return false;
    if (client_ && srv == srv_) return true;
    srv_ = srv; client_ = node_->create_client<std_srvs::srv::Trigger>(srv);
    if (!client_->wait_for_service(std::chrono::duration<double>(g_discovery))) { RCLCPP_ERROR(node_->get_logger(), "service %s not available", srv.c_str()); client_.reset(); return false; }
    // the client sees the server first; give the server time to match this client (an earlier request is dropped)
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    return true;
  }
  BT::NodeStatus onStart() override {
    if (service().empty()) { RCLCPP_ERROR(node_->get_logger(), "%s: missing port %s", name().c_str(), key_.c_str()); return BT::NodeStatus::FAILURE; }
    if (!prepare()) return BT::NodeStatus::FAILURE;
    const std::string srv = srv_;
    future_ = client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>()).future.share();
    RCLCPP_INFO(node_->get_logger(), "%s: called %s", name().c_str(), srv.c_str());
    return BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus onRunning() override {
    if (future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) return BT::NodeStatus::RUNNING;
    auto res = future_.get();
    // "SIGNALS name=value;..." in the response: the program's outputs, on the blackboard before the skill reports done
    const auto at = res->message.find("SIGNALS ");
    if (at != std::string::npos) {
      std::string list = res->message.substr(at + 8); list = list.substr(0, list.find('\\n'));
      size_t p = 0;
      while (p <= list.size()) {
        const size_t q = std::min(list.find(';', p), list.size()); const std::string kv = list.substr(p, q - p); p = q + 1;
        const auto eq = kv.find('='); if (eq == std::string::npos) continue;
        const std::string k = kv.substr(0, eq); std::string v = kv.substr(eq + 1);
        if (v == "true" || v == "false") config().blackboard->set<bool>(k, v == "true");
        else { try { config().blackboard->set<double>(k, std::stod(v)); } catch (...) { if (v.size() >= 2 && v.front() == '"') v = v.substr(1, v.size() - 2); config().blackboard->set<std::string>(k, v); } }
      }
    }
    return res->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
  void onHalted() override {}
 protected:
  rclcpp::Node::SharedPtr node_;
  std::string kind_, key_, fn_, srv_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
  std::shared_future<std_srvs::srv::Trigger::Response::SharedPtr> future_;
};

${classes.join('\n\n')}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("${bp}");
  g_ns = node->declare_parameter<std::string>("namespace", g_ns);
  g_discovery = node->declare_parameter<double>("discovery", g_discovery);
  const std::string file = node->declare_parameter<std::string>("tree", ament_index_cpp::get_package_share_directory("${bp}") + "/behavior_trees/${pkg}.xml");
  BT::BehaviorTreeFactory factory;
${reg.join('\n')}
  auto bb = BT::Blackboard::create();
${init.join('\n')}
  auto tree = factory.createTreeFromFile(file, bb);
  // discover every skill service before the first tick, then let the servers match this client:
  // a request sent before the server has matched the client is lost (the skill would never answer)
  int prepared = 0;
  tree.applyVisitor([&prepared](BT::TreeNode* n) { if (auto* sk = dynamic_cast<TriggerSkill*>(n)) prepared += sk->prepare() ? 1 : 0; });
  RCLCPP_INFO(node->get_logger(), "%d skill service(s) ready", prepared);
  // station signals ("name=value" lines) → blackboard
  auto sub = node->create_subscription<std_msgs::msg::String>(g_ns + "/signal_updates", 10, [bb](const std_msgs::msg::String& m) {
    const auto eq = m.data.find('='); if (eq == std::string::npos) return;
    const std::string k = m.data.substr(0, eq), v = m.data.substr(eq + 1);
    if (v == "true" || v == "false") bb->set<bool>(k, v == "true"); else { try { bb->set<double>(k, std::stod(v)); } catch (...) { bb->set<std::string>(k, v); } }
  });
  const double limit = node->declare_parameter<double>("limit", 3600.0);
  const auto t0 = node->now();
  BT::NodeStatus st = BT::NodeStatus::RUNNING;
  // the node spins in its own thread (service responses, signal updates); the tree is ticked here at 20 Hz
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  std::thread spinner([&exec]() { exec.spin(); });
  rclcpp::WallRate rate(20);
  while (rclcpp::ok() && (st == BT::NodeStatus::RUNNING || st == BT::NodeStatus::IDLE) && (node->now() - t0).seconds() < limit) {
    st = tree.tickOnce();
    rate.sleep();
  }
  RCLCPP_INFO(node->get_logger(), "RESULT %s after %.2f s", BT::toStr(st).c_str(), (node->now() - t0).seconds());
  std::printf("RESULT\\t%s\\t%.2f\\n", BT::toStr(st).c_str(), (node->now() - t0).seconds());
  tree.haltTree();
  exec.cancel();
  spinner.join();
  rclcpp::shutdown();
  return st == BT::NodeStatus::SUCCESS ? 0 : 1;
}
`;
  const cmake = `cmake_minimum_required(VERSION 3.8)
project(${bp})
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()
set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
add_executable(bt_runner src/bt_runner.cpp)
ament_target_dependencies(bt_runner rclcpp std_msgs std_srvs behaviortree_cpp ament_index_cpp)
install(TARGETS bt_runner DESTINATION lib/\${PROJECT_NAME})
install(DIRECTORY behavior_trees DESTINATION share/\${PROJECT_NAME})
ament_package()
`;
  const pkgXml = `<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>${bp}</name>
  <version>0.1.0</version>
  <description>BehaviorTree.CPP v4 tree "${xmlEsc(name)}" with skill stubs, generated by VerticalBot Studio</description>
  <maintainer email="user@example.com">VerticalBot Studio user</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>std_srvs</depend>
  <depend>behaviortree_cpp</depend>
  <depend>ament_index_cpp</depend>
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
`;
  return [
    { name: `${bp}/package.xml`, content: pkgXml }, { name: `${bp}/CMakeLists.txt`, content: cmake },
    { name: `${bp}/behavior_trees/${pkg}.xml`, content: xml, mime: 'application/xml' }, { name: `${bp}/src/bt_runner.cpp`, content: cpp },
  ];
}

// ---------------------------------------------------------------------------------------------
// The package
// ---------------------------------------------------------------------------------------------

/** All files of the ROS 2 workspace `src/` for a control model (paths relative to `src/`). */
export function ros2Packages(kind: ControlKind, source: string, name: string, opts: Ros2Options = {}): GenFile[] {
  if (!['des', 'petri', 'fsm', 'bt'].includes(kind)) throw new Error(`no ROS 2 package for ${kind} models (des, petri, fsm, bt)`);
  const base = pkgName(name); const pkg = `${base}_control`; const ns = opts.namespace ?? `/${base}`;
  const m = runtimeModel(kind, source);
  if (kind === 'bt' && opts.supervisorSource) m.model.supervisor = synthesizeSupervisor(opts.supervisorSource)?.table ?? null;
  const services = servicesOf(m);
  const programs = services.filter((s) => s.startsWith('/program/')).map((s) => s.slice(9));
  const model: Record<string, unknown> = { kind: m.kind, model: m.model, signals: m.signals, namespace: ns };
  if (kind === 'fsm') model.yasmin = yasminData(source);
  const durations: Record<string, number> = {}; for (const s of services) { const key = s.split('/').slice(2).join('/'); const d = opts.durations?.[key] ?? Object.entries(opts.durations ?? {}).find(([k]) => k.replace(/[^A-Za-z0-9_]/g, '_') === key)?.[1]; durations[key] = d ?? 1; }
  const outputs: Record<string, Array<[string, number | boolean | string]>> = {}; for (const s of services) { const key = s.split('/').slice(2).join('/'); const o = opts.outputs?.[key] ?? Object.entries(opts.outputs ?? {}).find(([k]) => k.replace(/[^A-Za-z0-9_]/g, '_') === key)?.[1]; if (o?.length) outputs[key] = o; }
  const robots: Record<string, string> = {}; for (const [p, r] of Object.entries(opts.robots ?? {})) robots[p] = r;
  const py = (o: unknown) => JSON.stringify(o);
  const modelPy = `"""Model data generated by VerticalBot Studio from "${name.replace(/"/g, "'")}" (${kind})."""
import json

MODEL = json.loads(r'''${py(model)}''')
SERVICES = json.loads(r'''${py(services)}''')
PROGRAMS = json.loads(r'''${py(Object.keys(opts.durations ?? {}).length ? [...new Set([...programs, ...Object.keys(opts.durations ?? {})])] : programs)}''')
ROBOTS = json.loads(r'''${py(robots)}''')
DURATIONS = json.loads(r'''${py(durations)}''')
OUTPUTS = json.loads(r'''${py(outputs)}''')
`;
  const entry = [`supervisor = ${pkg}.supervisor_node:main`, `skill_server = ${pkg}.skill_server:main`, ...(kind === 'fsm' ? [`yasmin_fsm = ${pkg}.yasmin_fsm:main`] : [])];
  const setupPy = `import os
from glob import glob

from setuptools import find_packages, setup

package_name = '${pkg}'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='VerticalBot Studio user',
    maintainer_email='user@example.com',
    description='Control model "${name.replace(/'/g, '')}" (${kind}) generated by VerticalBot Studio',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
${entry.map((e) => `            '${e}',`).join('\n')}
        ],
    },
)
`;
  const pkgXml = `<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>${pkg}</name>
  <version>0.1.0</version>
  <description>Control model "${xmlEsc(name)}" (${kind}) generated by VerticalBot Studio: supervisor node${kind === 'fsm' ? ', yasmin FSM' : ''} and a simulated skill server</description>
  <maintainer email="user@example.com">VerticalBot Studio user</maintainer>
  <license>Apache-2.0</license>
  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>std_srvs</exec_depend>
  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>
${kind === 'fsm' ? '  <exec_depend>yasmin</exec_depend>\n  <exec_depend>yasmin_ros</exec_depend>\n' : ''}${kind === 'bt' ? `  <exec_depend>${base}_bt</exec_depend>\n` : ''}  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
`;
  const engines = kind === 'fsm' ? `'runtime' or 'yasmin'` : kind === 'bt' ? `'runtime' or 'btcpp'` : `'runtime'`;
  const launch = `"""Launch the control model "${name.replace(/"/g, "'")}": the supervisor (engine:=${engines}) and, with sim:=true, the simulated skill server."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import EqualsSubstitution, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ns = LaunchConfiguration('namespace')
    params = [{'namespace': ns}]
    engines = [
        Node(package='${pkg}', executable='supervisor', name='supervisor', output='screen', parameters=params, condition=IfCondition(EqualsSubstitution(LaunchConfiguration('engine'), 'runtime'))),
${kind === 'fsm' ? `        Node(package='${pkg}', executable='yasmin_fsm', name='yasmin_fsm', output='screen', parameters=params, condition=IfCondition(EqualsSubstitution(LaunchConfiguration('engine'), 'yasmin'))),\n` : ''}${kind === 'bt' ? `        Node(package='${base}_bt', executable='bt_runner', name='bt_runner', output='screen', parameters=params, condition=IfCondition(EqualsSubstitution(LaunchConfiguration('engine'), 'btcpp'))),\n` : ''}    ]
    # the launch ends with the control program (the simulated skill server would run forever)
    stop = [RegisterEventHandler(OnProcessExit(target_action=e, on_exit=[EmitEvent(event=Shutdown(reason='control program finished'))])) for e in engines]
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='${ns}'),
        DeclareLaunchArgument('sim', default_value='true', description='start the simulated skill server'),
        DeclareLaunchArgument('engine', default_value='runtime', description=${py(`${engines}`)}),
        Node(package='${pkg}', executable='skill_server', name='skill_server', output='screen', parameters=params, condition=IfCondition(LaunchConfiguration('sim'))),
        *engines, *stop,
    ])
`;
  const readme = `# ${pkg}\n\nGenerated by VerticalBot Studio from the ${kind} model "${name}".\n\n\`\`\`bash\ncd ~/ws && colcon build && source install/setup.bash\nros2 launch ${pkg} ${pkg}.launch.py            # supervisor + simulated skill server\nros2 launch ${pkg} ${pkg}.launch.py sim:=false  # against your skill servers\n${kind === 'fsm' ? `ros2 launch ${pkg} ${pkg}.launch.py engine:=yasmin\n` : ''}${kind === 'bt' ? `ros2 launch ${pkg} ${pkg}.launch.py engine:=btcpp\n` : ''}\`\`\`\n\nInterfaces (namespace \`${ns}\`): ${services.map((s) => `\`${ns}${s}\``).join(', ') || 'none'} (std_srvs/Trigger); signals: \`${ns}/signals\` (std_msgs/String JSON, in), \`${ns}/signals/set\` (out); state: \`${ns}/fsm/state\`; result: \`${ns}/result\`.\n\nOn WSL2, when the C++ bt_runner does not reach the Python skill servers (UDP loopback), run with \`export FASTDDS_BUILTIN_TRANSPORTS=SHM\`.\n`;
  const files: GenFile[] = [
    { name: `${pkg}/package.xml`, content: pkgXml }, { name: `${pkg}/setup.py`, content: setupPy },
    { name: `${pkg}/setup.cfg`, content: `[develop]\nscript_dir=$base/lib/${pkg}\n[install]\ninstall_scripts=$base/lib/${pkg}\n` },
    { name: `${pkg}/resource/${pkg}`, content: '' }, { name: `${pkg}/README.md`, content: readme },
    { name: `${pkg}/${pkg}/__init__.py`, content: '' },
    { name: `${pkg}/${pkg}/vbs_runtime.py`, content: `# -*- coding: utf-8 -*-\n"""Control runtime of VerticalBot Studio (same semantics as the studio executors)."""\n${PY_CORE}` },
    { name: `${pkg}/${pkg}/ros_link.py`, content: ROS_LINK }, { name: `${pkg}/${pkg}/model.py`, content: modelPy },
    { name: `${pkg}/${pkg}/supervisor_node.py`, content: SUPERVISOR_NODE }, { name: `${pkg}/${pkg}/skill_server.py`, content: SKILL_SERVER },
    { name: `${pkg}/launch/${pkg}.launch.py`, content: launch },
  ];
  if (kind === 'fsm') files.push({ name: `${pkg}/${pkg}/yasmin_fsm.py`, content: YASMIN_HEAD + YASMIN_MAIN });
  if (kind === 'bt') files.push(...btCppPackage(source, name, base, ns));
  return files;
}
