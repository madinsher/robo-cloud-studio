/**
 * MURKA retail — the MURKA_Simulation copy (simulation_ws/src/MURKA_Simulation: Murka v2 robot = variant 'murka',
 * MurkaClassic = 'murka_classic') runs that project's OWN task FSMs (murka_fsm/orchestrator.py: depalletize and
 * collect_order only — no scan / verify / dock / mission) and passes that project's own acceptance test
 * scripts/e2e_test.py (depalletize pallet_1/box_red -> bulk_B1, then order [tea_box, coffee_can] -> packing_S*).
 * References: the structure of the real orchestrator.py (Python, stubbed ROS) and the Webots runs stored in that
 * project (article/data = robot_model v2, article/data_classic = classic: /murka/fsm/progress state sequences).
 * The robot stands on its wheels like in Webots (base_link z from the wheel / caster boundingObjects, not the 1 cm of the
 * world translation): from 1 cm the palm IK of the shelf L0 pre-grasp has no solution in the original arm_kinematics.py.
 */
import { describe, it, expect } from 'vitest';
import fs from 'node:fs';
import path from 'node:path';
import { buildMurkaRetailStation, runMurkaRetailMission, retailSessionOf, parseMission, missionFsmDsl, runMurkaE2E } from '../src/murka/retail';
import type { RetailVariant } from '../src/murka/retail';
import { parseFsm, FsmMachine, stateOutcomes } from '../src/ctl/fsm';
import { AssetStore } from '../src/scene/assets';
import { ItemType, Item } from '../src/core/items/item';
import { HAS_MURKA, MURKA, HAS_PY, runPython } from './murka_retail_helpers';

const COPY = path.join(MURKA, 'simulation_ws/src/MURKA_Simulation');
const COPY_SRC = path.join(COPY, 'ros2_ws/src');
const HAS_COPY = HAS_MURKA && fs.existsSync(path.join(COPY_SRC, 'murka_fsm/murka_fsm/orchestrator.py'));

/** FSM state sequences per task of the Webots runs stored in the copy (article/<dir>/run_norviz_*.json). */
function webotsRuns(dir: string): Array<{ file: string; depal: string[]; order: string[]; final: Record<string, string> }> {
  const d = path.join(COPY, 'article', dir);
  if (!fs.existsSync(d)) return [];
  return fs.readdirSync(d).filter((f) => /^run_norviz_\d+\.json$/.test(f)).sort().map((f) => {
    const j = JSON.parse(fs.readFileSync(path.join(d, f), 'utf8')) as { events: Array<{ task_type: string; state: string }>; objects_final: Record<string, string> };
    const seq = (t: string) => j.events.filter((e) => e.task_type === t).map((e) => e.state).filter((s) => s !== 'done' && s !== 'error');
    return { file: f, depal: seq('depalletize'), order: seq('collect_order'), final: j.objects_final };
  });
}

describe.skipIf(!HAS_MURKA)('MURKA retail: the robot stands on its wheels (Webots settles it under gravity)', () => {
  it('every variant: base_link z = floor top - lowest point of the wheel / caster boundingObjects (anchor 0.181, radius 0.177 -> -4 mm), not the world translation (+10 mm)', async () => {
    for (const variant of ['v2', 'v1', 'murka', 'murka_classic'] as RetailVariant[]) {
      const b = await buildMurkaRetailStation({ variant, loadMeshes: false });
      const z = b.robot.base.poseAbs()[14];
      expect(Math.abs(z + 4), variant).toBeLessThan(0.01);
      expect(b.report.some((l) => /rests on its wheels: base_link z 10\.0 -> -4\.0 mm/.test(l)), b.report.join('\n')).toBe(true);
      // the 2D state and the home pose stay where the world puts the robot
      expect(b.robot.base.state.x).toBe(variant.startsWith('murka') ? -5500 : -7199);
    }
  }, 300000);
});

const PY_COPY_STRUCT = String.raw`
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
for n in ['geometry_msgs', 'std_msgs', 'murka_msgs']: mod(n)
mod('rclpy', ok=lambda: False); mod('rclpy.action', ActionClient=_Any); mod('rclpy.callback_groups', ReentrantCallbackGroup=_Any)
mod('rclpy.executors', MultiThreadedExecutor=_Any); mod('rclpy.node', Node=Node)
mod('geometry_msgs.msg', PoseStamped=_Any); mod('std_msgs.msg', String=_Any)
mod('yasmin', Blackboard=Blackboard, State=State, StateMachine=StateMachine)
mod('murka_msgs.action', NavigateToTarget=_Any, PickProduct=_Any, PlaceProduct=_Any); mod('murka_msgs.msg', ProductArray=_Any, TaskProgress=_Any)
mod('murka_msgs.srv', CheckPlace=_Any, StartDepalletize=_Any, StartOrder=_Any)
sys.path.insert(0, inp['pkg'])
from murka_fsm import orchestrator as O
orch = O.Orchestrator.__new__(O.Orchestrator)
orch._locations = {}
def ser(sm):
    return {'outcomes': sm._outcomes, 'states': [{'name': n, 'transitions': tr, 'outcomes': st._outcomes} for n, st, tr in sm.states]}
res = []
for t in inp['tasks']:
    sm, bb = orch._run_task(t)
    res.append(ser(sm))
print(json.dumps(res))
`;

describe.skipIf(!HAS_COPY || !HAS_PY)('MURKA_Simulation copy: studio fsm documents = yasmin machines of murka_fsm/orchestrator.py', () => {
  it('depalletize / collect_order: same states, outcomes and transitions; the copy has no other task', () => {
    const tasks = [{ type: 'depalletize', pallet_id: 'pallet_1', sku: 'box_red', target_shelf: 'bulk_B1', count: 1 }, { type: 'collect_order', skus: ['tea_box', 'coffee_can'] }];
    const py = runPython(PY_COPY_STRUCT, { pkg: path.join(COPY_SRC, 'murka_fsm'), tasks }) as Array<{ outcomes: string[]; states: Array<{ name: string; outcomes: string[]; transitions: Record<string, string> }> }>;
    const L = { targets: { home: { x: -5.5, y: 3, yaw: 0 } }, dock: null, slots: {}, basket: {}, catalog: {}, instances: [], frame: 'map' } as never;
    const ours = [parseMission('depalletize', ['pallet_1', 'box_red', 'bulk_B1', '1'], L, 'murka'), parseMission('order', ['tea_box', 'coffee_can'], L, 'murka')].map((t) => parseFsm(missionFsmDsl(t, 'murka')).root);
    ours.forEach((m: FsmMachine, i) => {
      expect([...m.outcomes].sort()).toEqual([...py[i].outcomes].sort());
      expect(m.states.map((s) => s.name)).toEqual(py[i].states.map((s) => s.name));
      m.states.forEach((s, k) => { expect([...stateOutcomes(s)].sort(), s.name).toEqual([...py[i].states[k].outcomes].sort()); expect(s.transitions, s.name).toEqual(py[i].states[k].transitions); });
    });
    for (const name of ['full', 'scan', 'dock', 'undock', 'home', 'goto']) expect(() => parseMission(name, [], L, 'murka'), name).toThrow(/depalletize, order/);
  }, 120000);
});

describe.skipIf(!HAS_COPY)('MURKA_Simulation copy: its own acceptance test scripts/e2e_test.py on the studio simulator', () => {
  for (const [variant, dir] of [['murka', 'data'], ['murka_classic', 'data_classic']] as Array<[RetailVariant, string]>) {
    it(`${variant}: depalletize pallet_1/box_red -> bulk_B1, order [tea_box, coffee_can] -> packing; FSM sequences = the Webots runs of that project (article/${dir})`, async () => {
      const b = await buildMurkaRetailStation({ variant, assets: new AssetStore(), loadMeshes: true });
      const ref = webotsRuns(dir);
      expect(ref.length).toBe(5);
      const s = retailSessionOf(b);
      const e2e = await runMurkaE2E(b).done;
      const log = [...e2e.lines, ...s.orch.log.slice(-30), ...s.manip.log.slice(-30)].join('\n');
      // e2e_test.py criteria: both tasks end 'done', a box_red on bulk_B1_*, a tea_box and a coffee_can on packing_S*
      expect(e2e.failures, log).toEqual([]);
      expect(e2e.pass).toBe(true);
      expect(e2e.lines[e2e.lines.length - 1]).toBe('E2E PASS: оба сценария выполнены');
      const [depal, order] = e2e.tasks;
      expect(depal.outcome).toBe('done'); expect(order.outcome).toBe('done');
      // the state sequences of the copy's orchestrator (no NAV_VERIFY / VERIFY_ITEM / RELEASE_HELD), as published in Webots
      const webDepal = new Set(ref.map((r) => r.depal.join(' '))), webOrder = new Set(ref.map((r) => r.order.join(' ')));
      expect(webDepal.has(depal.states.join(' ')), depal.states.join(' ')).toBe(true);
      expect(webOrder.has(order.states.join(' ')), order.states.join(' ')).toBe(true);
      // the same products end where Webots put them in the runs that delivered both SKUs
      const full = ref.filter((r) => r.final.tea_box_1 === 'packing_S1');
      expect(full.length).toBeGreaterThanOrEqual(2);
      for (const name of ['box_red_7', 'tea_box_1', 'coffee_can_1']) expect(s.sim.objects.get(name)!.location, name).toBe(full[0].final[name]);
      // no task of the v2 orchestrator ran
      expect([...depal.states, ...order.states].filter((x) => /VERIFY|RELEASE|SCAN|DOCK|PH_/.test(x))).toEqual([]);
      // the robot body never enters the furniture
      expect(depal.checks.collisions.base + order.checks.collisions.base).toBe(0);
    }, 900000);
  }

  it('murka: a single order task through runMurkaRetailMission uses the copy FSM and its defaults (README / e2e_test.py)', async () => {
    const b = await buildMurkaRetailStation({ variant: 'murka', loadMeshes: false });
    // the station carries the copy's two task machines as control models (Control tab: view, export, Run on station)
    const models = [...b.station.walk()].filter((i) => i.type === ItemType.CONTROL_MODEL) as Array<Item & { source: string }>;
    expect(models.map((m) => m.name)).toEqual(['MURKA depalletize', 'MURKA collect_order']);
    const r = await runMurkaRetailMission(b, { mission: 'order' }).done;
    expect(r.outcome, r.report.join('\n')).toBe('done');
    expect(r.dsl).toBe(models[1].source);
    expect(r.dsl.split('\n')[0]).toBe('fsm MURKA collect_order');
    expect(r.checks.order!.onPacking).toEqual({ tea_box: ['tea_box_1 @ packing_S1'], coffee_can: ['coffee_can_1 @ packing_S0'] });
    await expect(async () => runMurkaRetailMission(b, { mission: 'full' })).rejects.toThrow(/depalletize, order/);
  }, 600000);
});
