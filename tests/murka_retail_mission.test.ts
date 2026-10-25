/**
 * MURKA retail — the `full` mission of robot-retailer-murka-v2 (scenario of docs/scenario.md) run headless by the
 * studio: orchestrator FSM (same yasmin structure as murom_fsm/orchestrator.py, checked against the Python source) on
 * the studio FsmExecutor, Nav2 RPP stand-in + glide docking, DLS palm IK with furniture-aware selection, magnetic
 * attach, classical perception on studio renders, battery model. Success criteria = mission_test.py + webots.md §6.1.
 */
import { describe, it, expect, beforeAll } from 'vitest';
import { buildMurkaRetailStation, runMurkaRetailMission, RetailMissionResult, RetailStationResult, missionFsmDsl, parseMission } from '../src/murka/retail';
import type { RetailSession } from '../src/murka/retail';
import { parseFsm, fsmMachines, stateOutcomes, checkFsm, FsmMachine } from '../src/ctl/fsm';
import { AssetStore } from '../src/scene/assets';
import { StationRuntime } from '../src/core/motion/runtime';
import { HAS_MURKA, HAS_PY, runPython, V2_SRC } from './murka_retail_helpers';

let built: RetailStationResult; let res: RetailMissionResult; let session: RetailSession;
const truth: Record<string, string> = {};

describe.skipIf(!HAS_MURKA)('MURKA retail v2: full mission headless', () => {
  beforeAll(async () => {
    const assets = new AssetStore();
    built = await buildMurkaRetailStation({ variant: 'v2', assets, loadMeshes: true });
    for (const i of built.layout.instances) truth[i.slot] = i.sku;
    const run = runMurkaRetailMission(built, { assets });
    session = run.session;
    res = await run.done;
  }, 900000);

  it('FSM result done; stages SCAN -> DEPAL -> ORDER -> DOCK all done, in order, no error continuation', () => {
    expect(res.outcome, res.report.join('\n')).toBe('done');
    expect(res.stages.map((s) => [s.name, s.outcome])).toEqual([['SCAN', 'done'], ['DEPAL', 'done'], ['ORDER', 'done'], ['DOCK', 'done']]);
    for (let i = 1; i < res.stages.length; i++) expect(res.stages[i].t0).toBeCloseTo(res.stages[i - 1].t1, 6);
    const paths = res.trace.map((e) => e.path.split('/').slice(1).join('/'));
    expect(paths.filter((p) => !p.includes('/'))).toEqual(['PH_SCAN', 'SCAN', 'PH_DEPAL', 'DEPAL', 'PH_ORDER', 'ORDER', 'PH_DOCK', 'DOCK', 'FINISH']);
    expect(res.checks.fsmDone).toBe(true);
  });

  it('SCAN: 24/24 visible gondola slots correct (= Webots and MuJoCo), 6 views, undock from the charger by reverse glide first', () => {
    expect(res.checks.scan).toEqual({ visible: 24, correct: 24, wrong: [], webotsVisible: 24 });
    for (const [slot, [sku]] of Object.entries(res.inventory)) expect(sku, slot).toBe(truth[slot] ?? '');
    expect(session.orch.scans.filter((s) => s.target.startsWith('shelf_A') && !s.target.includes('_L'))).toHaveLength(6);
    expect(session.orch.navLog[0].state).toBe('NAV_VIEW');
    // the mission starts on the charger: the first goal begins with the reverse glide onto the dock standoff line
    const ev = session.nav.events;
    expect(ev[0].kind).toBe('undock'); expect(ev[0].ok).toBe(true);
    expect(ev[0].t0).toBeGreaterThanOrEqual(1.9); expect(ev[0].t0).toBeLessThan(2.5); // after arms_for_view(True): 2 s trajectory
    expect(ev.filter((e) => e.kind === 'undock').length).toBeGreaterThanOrEqual(1);
    expect(session.orch.states[0].state).toBe('SCAN');
  });

  it('DEPAL: one box_blue from the top layer of pallet_2 (west column first) on the lowest reachable bulk_B3 slot', () => {
    expect(res.checks.depal!.ok).toBe(true);
    expect(res.checks.depal!.moved).toEqual(['box_blue_7 -> bulk_B3_L0_S0']);
    const b = res.registry.box_blue_7;
    expect(b.pos[0]).toBeCloseTo(1.4, 2); expect(b.pos[1]).toBeCloseTo(-1.39, 2);
    expect(b.pos[2]).toBeCloseTo(1.15 + 0.14 + 0.002, 3); // standing on the board: centre = board + h/2 + 2 mm
    // the two-hand place on bulk L1 is out of IK reach (the slot is skipped by /murom/check_place, like the original)
    expect(session.manip.checkPlace('box_blue_8', 'bulk_B3_L1_S0', 'both')[0]).toBe(false);
  });

  it('ORDER: tea_box and juice_pack chosen from the camera inventory, verified by the camera, basket, packing table', () => {
    expect(res.checks.order!.ok).toBe(true);
    const onPack = Object.entries(res.registry).filter(([, o]) => o.location.startsWith('packing_')).map(([n, o]) => [n, o.location]).sort();
    expect(onPack).toEqual([['juice_pack_1', 'packing_S0'], ['tea_box_1', 'packing_S1']]);
    for (const [n] of onPack) expect(res.registry[n].pos[2]).toBeGreaterThan(1.0); // on the table top (1.0 m)
    const verify = session.orch.log.filter((l) => l.includes('VERIFY_ITEM'));
    expect(verify).toHaveLength(2);
    expect(verify.every((l) => /уверенность 1\.00/.test(l))).toBe(true);
    // picked by the left arm from the gondola, carried in the basket, unloaded to packing (two trips of the arm)
    const ops = session.orch.opLog.map((o) => `${o.state}:${o.kind}:${o.result.success}`);
    expect(ops).toEqual(['PICK:pick:true', 'PLACE:place:true', 'PICK_ITEM:pick:true', 'PLACE_BASKET:place:true', 'PICK_ITEM:pick:true', 'PLACE_BASKET:place:true', 'PICK_BASKET:pick:true', 'PLACE_PACK:place:true', 'PICK_BASKET:pick:true', 'PLACE_PACK:place:true']);
  });

  it('DOCK: on the charger within 6 cm / 3°, battery_node charges 0.8 %/s there', () => {
    expect(res.checks.docked!.ok).toBe(true);
    expect(res.checks.docked!.distance).toBeLessThan(0.06); expect(res.checks.docked!.yawDeg).toBeLessThan(3);
    expect(res.battery.docked).toBe(true);
    expect(res.battery.pct).toBeLessThan(100); // the mission drained it (idle + distance + turns)
    const p0 = session.sim.battery.pct; for (let i = 0; i < 50; i++) session.sim.step(0.05);
    expect(session.sim.battery.pct - p0).toBeCloseTo(Math.min(2.0, 100 - p0), 6);
  });

  it('collisions: no arm link, carried load or robot body ever enters the furniture colliders (mesh hand model, 5 mm); the only hand contact is the MURKA BOTH_CARRY fingertip at the bulk L1 board', () => {
    expect(session.sim.armGeom.left.source).toBe('mesh');
    expect(res.checks.collisions.armLinks).toBe(0);
    expect(res.checks.collisions.loads).toBe(0);
    expect(res.checks.collisions.base).toBe(0);
    // reproduced defect of the original (documented in docs/murka-retail.md): two-hand carry at z = 1.50 m, the
    // fingertips of the inward palms touch the underside of the bulk_B3 L1 board (1.62 m) at the end of the glide
    for (const c of res.collisions) {
      expect(c.part).toMatch(/_hand$/); expect(c.item).toBe('bulk_B3'); expect(c.depth).toBeLessThan(0.02);
      const st = session.orch.states.filter((s) => s.t <= c.t).pop()!.state;
      expect(st).toBe('NAV_PLACE');
    }
  });

  it('mission time: simulated time reported per stage and compared with the MURKA reference (Webots 1069 s wall at RTF 0.84, MuJoCo 990 s)', () => {
    expect(res.reference.webotsWall).toBe(1069); expect(res.reference.webotsRtf).toBe(0.84); expect(res.reference.mujoco).toBe(990);
    expect(res.reference.webotsSim).toBeCloseTo(1069 * 0.84, 6);
    expect(res.simTime).toBeCloseTo(res.stages[res.stages.length - 1].t1, 0);
    // same order of magnitude as the reference: the same servers with the same speeds and durations, minus Nav2 / ROS overheads
    expect(res.reference.ratio!).toBeGreaterThan(0.5); expect(res.reference.ratio!).toBeLessThan(1.2);
    // the shared station clock advanced with the mission
    expect(StationRuntime.of(built.station).time).toBeGreaterThanOrEqual(res.simTime - 1e-6);
    expect(res.report[0]).toMatch(/Webots 1069 s wall at RTF 0\.84/);
  });

  it('progress telemetry like /murom/fsm/progress: phase percentages SCAN 0–15, DEPAL 15–50, ORDER 50–90, DOCK 90–100', () => {
    const byPhase: Record<string, number[]> = {};
    for (const e of res.progress) (byPhase[e.phase] ??= []).push(e.percent);
    expect(Math.min(...byPhase.SCAN)).toBeGreaterThanOrEqual(0); expect(Math.max(...byPhase.SCAN)).toBeLessThanOrEqual(15);
    expect(Math.min(...byPhase.DEPAL)).toBeGreaterThanOrEqual(15); expect(Math.max(...byPhase.DEPAL)).toBeLessThanOrEqual(50);
    expect(Math.min(...byPhase.ORDER)).toBeGreaterThanOrEqual(50); expect(Math.max(...byPhase.ORDER)).toBeLessThanOrEqual(90);
    expect(Math.min(...byPhase.DOCK)).toBeGreaterThanOrEqual(90);
    expect(res.progress.map((e) => e.state)).toContain('RECOGNIZE');
  });
});

describe('MURKA orchestrator as an fsm document', () => {
  const L = { targets: { home: { x: -5.5, y: 3, yaw: 0 } }, dock: { approach: { x: -7.199, y: 4.5, yaw: 3.1416 }, standoff: { x: -6.449, y: 4.5, yaw: 3.1416 }, radius: 0.06 }, slots: { shelf_A1_L0_S0: { id: 'shelf_A1_L0_S0', kind: 'gondola', pose: { x: 0, y: 0, z: 0 }, arm: 'left' } }, basket: {}, catalog: {}, instances: [], frame: 'map' } as never;
  it('every task document parses, checks clean (all outcomes mapped, no unreachable state) and keeps the yasmin outcomes', () => {
    for (const [name, args] of [['full', []], ['full', ['scan=', 'order=tea_box']], ['depalletize', []], ['order', ['tea_box']], ['scan', []], ['goto', ['home']], ['dock', []], ['undock', []], ['home', []]] as Array<[string, string[]]>) {
      const doc = parseFsm(missionFsmDsl(parseMission(name, args, L)));
      expect(checkFsm(doc).filter((i) => i.level === 'error'), name).toEqual([]);
      expect(doc.root.outcomes).toEqual(['done', 'error']);
    }
    const noScan = parseFsm(missionFsmDsl(parseMission('full', ['scan=', 'order=tea_box'], L)));
    expect(noScan.root.states.map((s) => s.name)).toEqual(['PH_DEPAL', 'DEPAL', 'ERR_DEPAL', 'PH_ORDER', 'ORDER', 'ERR_ORDER', 'PH_DOCK', 'DOCK', 'ERR_DOCK', 'FINISH']);
  });
});

const PY_STRUCT = String.raw`
import json, sys, types
inp = json.load(sys.stdin)
class _Any:
    def __init__(self, *a, **k): pass
    def __getattr__(self, n): return _Any()
    def __call__(self, *a, **k): return _Any()
def mod(name, **attrs):
    m = types.ModuleType(name); m.__dict__.update(attrs); sys.modules[name] = m; return m
class Node:
    def __init__(self, *a, **k): pass
class State:
    def __init__(self, outcomes): self._outcomes = list(outcomes)
class StateMachine:
    def __init__(self, outcomes): self._outcomes = list(outcomes); self.states = []
    def add_state(self, name, state, transitions=None): self.states.append((name, state, dict(transitions or {})))
    def __call__(self, bb): return self
class Blackboard(dict): pass
for n in ['builtin_interfaces', 'geometry_msgs', 'sensor_msgs', 'std_msgs', 'std_srvs', 'trajectory_msgs', 'murom_msgs']: mod(n)
mod('rclpy', ok=lambda: False); mod('rclpy.action', ActionClient=_Any); mod('rclpy.callback_groups', ReentrantCallbackGroup=_Any)
mod('rclpy.executors', ExternalShutdownException=Exception, MultiThreadedExecutor=_Any); mod('rclpy.node', Node=Node)
mod('builtin_interfaces.msg', Duration=_Any); mod('geometry_msgs.msg', PoseStamped=_Any); mod('sensor_msgs.msg', JointState=_Any)
mod('std_msgs.msg', String=_Any); mod('std_srvs.srv', Trigger=_Any); mod('trajectory_msgs.msg', JointTrajectory=_Any, JointTrajectoryPoint=_Any)
mod('yasmin', Blackboard=Blackboard, State=State, StateMachine=StateMachine)
mod('murom_msgs.action', NavigateToTarget=_Any, PickProduct=_Any, PlaceProduct=_Any); mod('murom_msgs.msg', ProductArray=_Any, TaskProgress=_Any)
mod('murom_msgs.srv', CheckPlace=_Any, ScanShelf=_Any, StartDepalletize=_Any, StartMission=_Any, StartOrder=_Any)
sys.path.insert(0, inp['pkg'])
from murom_fsm import orchestrator as O
orch = O.Orchestrator.__new__(O.Orchestrator)
orch._locations = inp['locations']
def ser(sm):
    out = {'outcomes': sm._outcomes, 'states': []}
    for name, st, tr in sm.states:
        d = {'name': name, 'transitions': tr}
        if isinstance(st, StateMachine): d['machine'] = ser(st); d['outcomes'] = st._outcomes
        else: d['outcomes'] = st._outcomes
        out['states'].append(d)
    return out
res = []
for name, args in inp['tasks']:
    task = orch._parse_mission(name, args)
    task['task_id'] = 't'
    sm, bb = orch._run_task(task)
    res.append(ser(sm))
print(json.dumps(res))
`;

describe.skipIf(!HAS_PY)('MURKA orchestrator structure: studio fsm document = yasmin machines of orchestrator.py', () => {
  it('full / full with skipped stages / depalletize / order / scan / goto / dock / undock / home: same states, outcomes, transitions', () => {
    const tasks: Array<[string, string[]]> = [['full', []], ['full', ['scan=', 'depal=pallet_1,box_red,bulk_B1,2']], ['full', ['order=']], ['depalletize', []], ['order', ['tea_box', 'juice_pack']], ['scan', []], ['goto', ['home']], ['goto', ['shelf_A2_L0_S1']], ['dock', []], ['undock', []], ['home', []]];
    const locations = { targets: { home: { x: -5.5, y: 3, yaw: 0 } }, dock: { approach: { x: -7.199, y: 4.5, yaw: 3.1416 }, standoff: { x: -6.449, y: 4.5, yaw: 3.1416 } }, slots: { shelf_A2_L0_S1: { arm: 'left' } } };
    const py = runPython(PY_STRUCT, { pkg: `${V2_SRC}/murom_fsm`, tasks, locations }) as Array<{ outcomes: string[]; states: Array<{ name: string; outcomes: string[]; transitions: Record<string, string>; machine?: unknown }> }>;
    const L = { targets: locations.targets, dock: { ...locations.dock, radius: 0.06 }, slots: { shelf_A2_L0_S1: { id: 'shelf_A2_L0_S1', kind: 'gondola', pose: { x: 0, y: 0, z: 0 }, arm: 'left' } }, basket: {}, catalog: {}, instances: [], frame: 'map' } as never;
    type PyM = { outcomes: string[]; states: Array<{ name: string; outcomes: string[]; transitions: Record<string, string>; machine?: PyM }> };
    const cmp = (m: FsmMachine, p: PyM, where: string) => {
      expect([...m.outcomes].sort(), where).toEqual([...p.outcomes].sort());
      expect(m.states.map((s) => s.name), where).toEqual(p.states.map((s) => s.name));
      m.states.forEach((s, i) => {
        const q = p.states[i];
        expect([...stateOutcomes(s)].sort(), `${where}/${s.name}`).toEqual([...q.outcomes].sort());
        expect(s.transitions, `${where}/${s.name}`).toEqual(q.transitions);
        expect(!!s.machine, `${where}/${s.name}`).toBe(!!q.machine);
        if (s.machine) cmp(s.machine, q.machine!, `${where}/${s.name}`);
      });
    };
    tasks.forEach(([name, args], i) => {
      const doc = parseFsm(missionFsmDsl(parseMission(name, args, L)));
      cmp(doc.root, py[i] as PyM, `${name} ${args.join(' ')}`);
      expect(fsmMachines(doc).length).toBeGreaterThan(0);
    });
  }, 300000);
});

const PY_YASMIN = String.raw`
import json, os, sys, types
inp = json.load(sys.stdin)
class _Any:
    def __init__(self, *a, **k): pass
    def __getattr__(self, n): return _Any()
    def __call__(self, *a, **k): return _Any()
def mod(name, **attrs):
    m = types.ModuleType(name); m.__dict__.update(attrs); sys.modules[name] = m; return m
class Node:
    def __init__(self, *a, **k): pass
class State:
    def __init__(self, outcomes): self._outcomes = list(outcomes)
    def cancel_state(self): pass
class StateMachine(State):
    def __init__(self, outcomes): State.__init__(self, outcomes); self.states = []; self.start = None
    def add_state(self, name, state, transitions=None): self.states.append((name, state, dict(transitions or {})))
    def set_start_state(self, n): self.start = n
    def __call__(self, bb): return self
class Blackboard(dict): pass
for n in ['builtin_interfaces', 'geometry_msgs', 'sensor_msgs', 'std_msgs', 'std_srvs', 'trajectory_msgs', 'murom_msgs']: mod(n)
mod('rclpy', ok=lambda: False); mod('rclpy.action', ActionClient=_Any); mod('rclpy.callback_groups', ReentrantCallbackGroup=_Any)
mod('rclpy.executors', ExternalShutdownException=Exception, MultiThreadedExecutor=_Any); mod('rclpy.node', Node=Node)
mod('builtin_interfaces.msg', Duration=_Any); mod('geometry_msgs.msg', PoseStamped=_Any); mod('sensor_msgs.msg', JointState=_Any)
mod('std_msgs.msg', String=_Any); mod('std_srvs.srv', Trigger=_Any); mod('trajectory_msgs.msg', JointTrajectory=_Any, JointTrajectoryPoint=_Any)
mod('yasmin', Blackboard=Blackboard, State=State, StateMachine=StateMachine)
mod('murom_msgs.action', NavigateToTarget=_Any, PickProduct=_Any, PlaceProduct=_Any); mod('murom_msgs.msg', ProductArray=_Any, TaskProgress=_Any)
mod('murom_msgs.srv', CheckPlace=_Any, ScanShelf=_Any, StartDepalletize=_Any, StartMission=_Any, StartOrder=_Any)
# the generated ROS 2 package (studio export of the retail FSM)
pkg = os.path.join(inp['dir'], 'gen'); os.makedirs(os.path.join(pkg, inp['pkg']), exist_ok=True)
for name, content in inp['files'].items():
    open(os.path.join(pkg, inp['pkg'], name), 'w', encoding='utf-8').write(content)
sys.path.insert(0, pkg)
gen = __import__(inp['pkg'] + '.yasmin_fsm', fromlist=['build'])
model = __import__(inp['pkg'] + '.model', fromlist=['MODEL']).MODEL
# the MURKA orchestrator
sys.path.insert(0, inp['murka'])
from murom_fsm import orchestrator as O
orch = O.Orchestrator.__new__(O.Orchestrator); orch._locations = {}
def ser(sm, drop):
    out = {'outcomes': sorted(o for o in sm._outcomes if o not in drop), 'states': []}
    for name, st, tr in sm.states:
        d = {'name': name, 'transitions': {k: v for k, v in tr.items() if k not in drop}, 'outcomes': sorted(o for o in st._outcomes if o not in drop)}
        if isinstance(st, StateMachine): d['machine'] = ser(st, drop)
        elif hasattr(st, 'sm'): d['machine'] = ser(st.sm, drop)
        out['states'].append(d)
    return out
task = orch._parse_mission('full', []); task['task_id'] = 't'
ours = ser(gen.build(None, model['yasmin'], model['yasmin']['path']), {'canceled'})
theirs = ser(orch._run_task(task)[0], set())
print(json.dumps({'ours': ours, 'theirs': theirs}))
`;

describe.skipIf(!HAS_PY)('yasmin export of the retail FSM (ROS 2 package of the Control tab) = orchestrator.py', () => {
  it('the generated yasmin_fsm.build() makes the same machines as _build_mission_sm (cancel outcomes of the export aside)', async () => {
    const { ros2Packages } = await import('../src/ctl/codegen/ros2');
    const L = { targets: {}, dock: null, slots: {}, basket: {}, catalog: {}, instances: [], frame: 'map' } as never;
    const files = ros2Packages('fsm', missionFsmDsl(parseMission('full', [], L)), 'murom');
    const pkg = 'murom_control';
    const want = ['__init__.py', 'vbs_runtime.py', 'model.py', 'yasmin_fsm.py'];
    const sel = Object.fromEntries(files.filter((f) => f.name.startsWith(`${pkg}/${pkg}/`) && want.includes(f.name.split('/').pop()!)).map((f) => [f.name.split('/').pop()!, f.content]));
    expect(Object.keys(sel).sort()).toEqual([...want].sort());
    const r = runPython(PY_YASMIN, { files: sel, pkg, murka: `${V2_SRC}/murom_fsm` });
    expect(r.ours).toEqual(r.theirs);
    expect(r.ours.states.map((s: { name: string }) => s.name)).toEqual(['PH_SCAN', 'SCAN', 'ERR_SCAN', 'PH_DEPAL', 'DEPAL', 'ERR_DEPAL', 'PH_ORDER', 'ORDER', 'ERR_ORDER', 'PH_DOCK', 'DOCK', 'ERR_DOCK', 'FINISH']);
  }, 300000);
});
