/**
 * Course examples used throughout the control-design modules, docs and demo scenarios.
 *
 *  - Example A — youBot mobile manipulator collecting parts from tables into bins (DES model + specifications
 *    E1…E5, supervisor synthesis, observer), planning domain, MDP grasp strategy, POMDP classification.
 *  - Example B — production cell: two manipulators M1/M2, machine S, buffer B, shared zone Z, AMR (Petri net with
 *    siphon-induced deadlock and its GMEC monitor, GSPN throughput, job-shop schedule 54 s, reliability budget).
 */
import { DES, Transition } from './des';

// ---------------------------------------------------------------------------------------------
// Example A — DES model (identical to the course reference implementation des_youbot.py)
// ---------------------------------------------------------------------------------------------

export const YOUBOT_UC = ['b_arrive', 'b_block', 'a_reached', 'a_stowed', 'g_ok', 'g_miss', 'g_slip', 'v_detect', 'v_lost'];
export const YOUBOT_UO = ['g_slip'];
const GO = ['b_go_table', 'b_go_box', 'b_go_home'];

const T = (list: Array<[string, string, string]>): Transition[] => list.map(([from, event, to]) => ({ from, event, to }));

export function youbotPlant(withGiveup = true): DES[] {
  // Base: H home, T table, B bin; m* moving, s* stopped by an obstacle
  const base = new DES('Base', ['b_go_table', 'b_go_box', 'b_go_home', 'b_arrive', 'b_block', 'b_replan', 'g_ok', 'g_put'], T([
    ['H', 'b_go_table', 'mT'], ['B', 'b_go_table', 'mT'], ['T', 'b_go_box', 'mB'], ['T', 'b_go_home', 'mH'], ['B', 'b_go_home', 'mH'], ['T', 'b_go_table', 'mT'],
    ['mT', 'b_arrive', 'T'], ['mB', 'b_arrive', 'B'], ['mH', 'b_arrive', 'H'],
    ['mT', 'b_block', 'sT'], ['mB', 'b_block', 'sB'], ['mH', 'b_block', 'sH'],
    ['sT', 'b_replan', 'mT'], ['sB', 'b_replan', 'mB'], ['sH', 'b_replan', 'mH'],
    ['T', 'g_ok', 'T'], ['B', 'g_put', 'B'],
  ]), 'H', ['H']);
  // Arm: S stowed, R reaching, X extended, W stowing
  const arm = new DES('Arm', ['a_reach', 'a_reached', 'a_stow', 'a_stowed', 'g_ok', 'g_put'], T([
    ['S', 'a_reach', 'R'], ['R', 'a_reached', 'X'], ['X', 'a_stow', 'W'], ['W', 'a_stowed', 'S'], ['X', 'g_ok', 'X'], ['X', 'g_put', 'X'],
  ]), 'S', ['S']);
  // Gripper: O open, C closing, D holding
  const grip = new DES('Grip', ['g_close', 'g_ok', 'g_miss', 'g_put', 'g_slip'], T([
    ['O', 'g_close', 'C'], ['C', 'g_ok', 'D'], ['C', 'g_miss', 'O'], ['D', 'g_put', 'O'], ['D', 'g_slip', 'O'],
  ]), 'O', ['O']);
  // Vision: Sr searching, F object in view
  const vis = new DES('Vision', ['v_detect', 'v_lost'], T([['Sr', 'v_detect', 'F'], ['F', 'v_lost', 'Sr']]), 'Sr', ['Sr', 'F']);
  // Object: Tb on table, Hn held, Fl on floor, Bx in bin, Un marked unreachable
  const objT: Array<[string, string, string]> = [['Tb', 'g_ok', 'Hn'], ['Hn', 'g_slip', 'Fl'], ['Hn', 'g_put', 'Bx']];
  const objSigma = ['g_ok', 'g_slip', 'g_put'];
  if (withGiveup) { objT.push(['Fl', 'o_giveup', 'Un']); objSigma.push('o_giveup'); }
  const obj = new DES('Object', objSigma, T(objT), 'Tb', ['Bx', 'Un']);
  return [base, arm, grip, vis, obj];
}

export function youbotSpecs(): Record<'E1' | 'E2' | 'E3' | 'E4' | 'E4m' | 'E5', DES> {
  // E1: the arm does not move while the base moves and vice versa
  const e1: Transition[] = [];
  for (const e of GO) e1.push({ from: 'Free', event: e, to: 'Base' });
  e1.push({ from: 'Base', event: 'b_arrive', to: 'Free' }, { from: 'Base', event: 'b_block', to: 'Base' }, { from: 'Base', event: 'b_replan', to: 'Base' });
  for (const e of ['a_reach', 'a_stow']) e1.push({ from: 'Free', event: e, to: 'Arm' });
  for (const e of ['a_reached', 'a_stowed']) e1.push({ from: 'Arm', event: e, to: 'Free' });
  const E1 = new DES('E1', [...GO, 'b_arrive', 'b_block', 'b_replan', 'a_reach', 'a_reached', 'a_stow', 'a_stowed'], e1, 'Free', ['Free', 'Base', 'Arm']);
  // E2: the base starts only with the arm stowed
  const e2: Transition[] = [{ from: 'In', event: 'a_reach', to: 'Out' }, { from: 'Out', event: 'a_stowed', to: 'In' }];
  for (const e of GO) e2.push({ from: 'In', event: e, to: 'In' });
  const E2 = new DES('E2', [...GO, 'a_reach', 'a_stowed'], e2, 'In', ['In', 'Out']);
  // E3: the gripper closes only on a visible object
  const E3 = new DES('E3', ['v_detect', 'v_lost', 'g_close'], T([['N', 'v_detect', 'Y'], ['Y', 'v_lost', 'N'], ['Y', 'g_close', 'Y']]), 'N', ['N', 'Y']);
  // E4: the object never drops (uncontrollable g_slip forbidden → unrealisable)
  const E4 = new DES('E4', ['g_slip'], [], 'Ok', ['Ok']);
  // E4': the object does not drop while the base moves
  const e4m: Transition[] = [];
  for (const e of GO) e4m.push({ from: 'Still', event: e, to: 'Move' });
  e4m.push({ from: 'Move', event: 'b_arrive', to: 'Still' }, { from: 'Move', event: 'b_block', to: 'Still' }, { from: 'Still', event: 'b_replan', to: 'Move' }, { from: 'Still', event: 'g_slip', to: 'Still' });
  const E4m = new DES("E4'", [...GO, 'b_arrive', 'b_block', 'b_replan', 'g_slip'], e4m, 'Still', ['Still', 'Move']);
  // E5: the gripper closes only with the arm extended
  const E5 = new DES('E5', ['a_reached', 'a_stow', 'g_close'], T([['Nx', 'a_reached', 'Ex'], ['Ex', 'a_stow', 'Nx'], ['Ex', 'g_close', 'Ex']]), 'Nx', ['Nx', 'Ex']);
  return { E1, E2, E3, E4, E4m, E5 };
}

// ---------------------------------------------------------------------------------------------
// Example B — production cell (Petri nets, performance)
// ---------------------------------------------------------------------------------------------
import { PetriNetSpec, S3PRSpec } from './petri';
import { ResourceLoad, CycleOperation } from './perf';

/** §4.6.4: two processes acquiring robot M1 and zone Z in opposite orders → siphon {rM, rZ, a2, b2} → deadlock. */
export const CELL_DEADLOCK_NET: PetriNetSpec = {
  name: 'Cell fragment: M1 and zone Z in opposite order',
  places: [
    { id: 'p1_idle', tokens: 1, kind: 'idle' }, { id: 'a1', kind: 'activity' }, { id: 'a2', kind: 'activity' },
    { id: 'p2_idle', tokens: 1, kind: 'idle' }, { id: 'b1', kind: 'activity' }, { id: 'b2', kind: 'activity' },
    { id: 'rM', tokens: 1, kind: 'resource', label: 'robot M1' }, { id: 'rZ', tokens: 1, kind: 'resource', label: 'zone Z' },
  ],
  transitions: [
    { id: 't1a', label: 'P1 takes M1', delay: 4 }, { id: 't2a', label: 'P1 enters Z', delay: 5 }, { id: 't3a', label: 'P1 releases M1, Z', delay: 1 },
    { id: 't1b', label: 'P2 enters Z', delay: 5 }, { id: 't2b', label: 'P2 takes M1', delay: 4 }, { id: 't3b', label: 'P2 releases Z, M1', delay: 1 },
  ],
  arcs: [
    { from: 'p1_idle', to: 't1a' }, { from: 'rM', to: 't1a' }, { from: 't1a', to: 'a1' },
    { from: 'a1', to: 't2a' }, { from: 'rZ', to: 't2a' }, { from: 't2a', to: 'a2' },
    { from: 'a2', to: 't3a' }, { from: 't3a', to: 'rM' }, { from: 't3a', to: 'rZ' }, { from: 't3a', to: 'p1_idle' },
    { from: 'p2_idle', to: 't1b' }, { from: 'rZ', to: 't1b' }, { from: 't1b', to: 'b1' },
    { from: 'b1', to: 't2b' }, { from: 'rM', to: 't2b' }, { from: 't2b', to: 'b2' },
    { from: 'b2', to: 't3b' }, { from: 't3b', to: 'rZ' }, { from: 't3b', to: 'rM' }, { from: 't3b', to: 'p2_idle' },
  ],
};

/** The same cell as an S³PR description (used by the builder / DSL). */
export const CELL_S3PR: S3PRSpec = {
  name: 'Cell S3PR', resources: { M1: 1, Z: 1 },
  processes: [
    { id: 'A', jobs: 1, steps: [{ id: 'take', resources: ['M1'], duration: 4 }, { id: 'zone', resources: ['M1', 'Z'], duration: 5 }] },
    { id: 'B', jobs: 1, steps: [{ id: 'zone', resources: ['Z'], duration: 5 }, { id: 'robot', resources: ['Z', 'M1'], duration: 4 }] },
  ],
};

/** §5.3.3: resource loads per part → bottleneck = machine S (22 s, 163.6 parts/h). */
export const CELL_LOADS: ResourceLoad[] = [
  { resource: 'M1', timePerCycle: 9, capacity: 1 }, { resource: 'S', timePerCycle: 22, capacity: 1 }, { resource: 'M2', timePerCycle: 9, capacity: 1 },
  { resource: 'Z', timePerCycle: 10, capacity: 1 }, { resource: 'AMR', timePerCycle: 6, capacity: 1 },
];

/** §5.6.6: operations with resource holding → max-plus cycle 32 s (machine busy during load/unload). */
export const CELL_OPERATIONS: CycleOperation[] = [
  { id: 'u1', duration: 4, resources: ['M1'] },
  { id: 'u2', duration: 5, resources: ['M1', 'S', 'Z'], after: ['u1'] },
  { id: 'u3', duration: 22, resources: ['S'], after: ['u2'] },
  { id: 'u4', duration: 5, resources: ['S', 'M2', 'Z'], after: ['u3'] },
  { id: 'u5', duration: 4, resources: ['M2'], after: ['u4'] },
];

/** §5.4.4: machine with failures as a GSPN (timed transitions only). */
export const MACHINE_GSPN: PetriNetSpec = {
  name: 'Machine with failures',
  places: [{ id: 'idle', tokens: 1 }, { id: 'work' }, { id: 'fail' }],
  transitions: [{ id: 'start', rate: 0.2 }, { id: 'finish', rate: 1 / 22 }, { id: 'breakdown', rate: 1 / 3600 }, { id: 'repair', rate: 1 / 900 }],
  arcs: [{ from: 'idle', to: 'start' }, { from: 'start', to: 'work' }, { from: 'work', to: 'finish' }, { from: 'finish', to: 'idle' }, { from: 'work', to: 'breakdown' }, { from: 'breakdown', to: 'fail' }, { from: 'fail', to: 'repair' }, { from: 'repair', to: 'idle' }],
};

// ---------------------------------------------------------------------------------------------
// Example A — GR(1) mission specification (§8.5)
// ---------------------------------------------------------------------------------------------
import { GR1Spec } from './gr1';

export function youbotGR1(opts: { humanLeaves?: boolean } = {}): GR1Spec {
  const adj: Record<string, string[]> = { base: ['base', 'corridor'], corridor: ['corridor', 'base', 'table1', 'table2', 'bins'], table1: ['table1', 'corridor'], table2: ['table2', 'corridor'], bins: ['bins', 'corridor'] };
  const move = Object.entries(adj).map(([from, tos]) => `loc = '${from}' -> (${tos.map((t) => `loc' = '${t}'`).join(' || ')})`);
  return {
    name: 'youBot mission (GR(1))',
    vars: [
      { name: 'loc', owner: 'sys', domain: ['base', 'corridor', 'table1', 'table2', 'bins'] },
      { name: 'holding', owner: 'sys', domain: ['none', 'obj'] },
      { name: 'obj_at_t1', owner: 'env', domain: [false, true] }, { name: 'obj_at_t2', owner: 'env', domain: [false, true] },
      { name: 'human_present', owner: 'env', domain: [false, true] }, { name: 'battery_low', owner: 'env', domain: [false, true] },
    ],
    envInit: ['!human_present', '!battery_low'],
    envTrans: [
      "obj_at_t1 && !(loc = 'table1') -> obj_at_t1'", "obj_at_t2 && !(loc = 'table2') -> obj_at_t2'", // objects do not vanish by themselves
    ],
    envLive: [...(opts.humanLeaves === false ? [] : ['!human_present']), '!battery_low'],
    sysInit: ["loc = 'base'", "holding = 'none'"],
    sysTrans: [
      ...move,
      "human_present -> loc' = loc",
      "!(holding' = 'obj' && loc' = 'base')",
      "holding' = 'obj' -> (holding = 'obj' || (loc = 'table1' && obj_at_t1) || (loc = 'table2' && obj_at_t2))",
      "holding' = 'none' -> (holding = 'none' || loc = 'bins')",
    ],
    sysLive: ["!obj_at_t1 || (loc = 'table1' && holding = 'obj')", "!obj_at_t2 || (loc = 'table2' && holding = 'obj')", "holding = 'none' || loc = 'bins'", "loc = 'base'", "!battery_low || loc = 'base'"],
  };
}

// ---------------------------------------------------------------------------------------------
// Example A — PDDL domain / problem (§10.3.1) and HTN methods (§10.5.3); Example B — durative cell domain (§10.9)
// ---------------------------------------------------------------------------------------------
import type { HTNDomain } from './planning';

export const YOUBOT_PDDL_DOMAIN = `(define (domain mobile-manipulator)
  (:requirements :strips :typing :negative-preconditions :action-costs :universal-preconditions)
  (:types location object bin - locatable part tool - object)
  (:predicates (at-robot ?l - location) (connected ?l1 ?l2 - location) (on ?o - object ?l - location) (holding ?o - object)
               (gripper-empty) (detected ?o - object) (in-bin ?o - object ?b - bin) (bin-at ?b - bin ?l - location)
               (accepts ?b - bin ?o - object) (reachable ?o - object ?l - location) (blocked ?o - object ?o2 - object))
  (:functions (travel-time ?l1 ?l2 - location) (total-cost))
  (:action move :parameters (?from ?to - location)
    :precondition (and (at-robot ?from) (connected ?from ?to))
    :effect (and (at-robot ?to) (not (at-robot ?from)) (increase (total-cost) (travel-time ?from ?to))))
  (:action pick :parameters (?o - object ?l - location)
    :precondition (and (at-robot ?l) (on ?o ?l) (gripper-empty) (detected ?o) (reachable ?o ?l) (forall (?o2 - object) (not (blocked ?o ?o2))))
    :effect (and (holding ?o) (not (on ?o ?l)) (not (gripper-empty)) (forall (?o3 - object) (not (blocked ?o3 ?o))) (increase (total-cost) 8)))
  (:action place :parameters (?o - object ?b - bin ?l - location)
    :precondition (and (at-robot ?l) (bin-at ?b ?l) (holding ?o) (accepts ?b ?o))
    :effect (and (in-bin ?o ?b) (gripper-empty) (not (holding ?o)) (increase (total-cost) 6)))
  (:action put-aside :parameters (?o - object ?l - location)
    :precondition (and (at-robot ?l) (holding ?o))
    :effect (and (on ?o ?l) (gripper-empty) (not (holding ?o)) (forall (?o3 - object) (not (blocked ?o3 ?o))) (increase (total-cost) 5)))
  (:action observe :parameters (?o - object ?l - location)
    :precondition (and (at-robot ?l) (on ?o ?l))
    :effect (and (detected ?o) (increase (total-cost) 3))))`;

export const YOUBOT_PDDL_PROBLEM = `(define (problem collect-2024-11)
  (:domain mobile-manipulator)
  (:objects base corridor table1 table2 binzone - location bolt1 bolt2 nut1 - part bin-bolts bin-nuts - bin)
  (:init (at-robot base) (gripper-empty)
    (connected base corridor) (connected corridor base) (connected corridor table1) (connected table1 corridor)
    (connected corridor table2) (connected table2 corridor) (connected corridor binzone) (connected binzone corridor)
    (on bolt1 table1) (on bolt2 table1) (on nut1 table2)
    (reachable bolt1 table1) (reachable bolt2 table1) (reachable nut1 table2)
    (bin-at bin-bolts binzone) (bin-at bin-nuts binzone)
    (accepts bin-bolts bolt1) (accepts bin-bolts bolt2) (accepts bin-nuts nut1)
    (= (travel-time base corridor) 12) (= (travel-time corridor base) 12) (= (travel-time corridor table1) 9) (= (travel-time table1 corridor) 9)
    (= (travel-time corridor table2) 11) (= (travel-time table2 corridor) 11) (= (travel-time corridor binzone) 7) (= (travel-time binzone corridor) 7) (= (total-cost) 0))
  (:goal (and (in-bin bolt1 bin-bolts) (in-bin bolt2 bin-bolts) (in-bin nut1 bin-nuts) (at-robot base)))
  (:metric minimize (total-cost)))`;

/** HTN methods for deliver(?o ?b): M1 already holding, M2 object on a known table, M3 not detected → search then deliver. */
export const YOUBOT_HTN: HTNDomain = {
  methods: [
    { task: 'deliver', name: 'M1 holding', params: ['?o', '?b'], pre: { k: 'atom', name: 'holding', args: ['?o'] }, subtasks: [{ name: 'goto', args: ['?bl'] }, { name: 'place', args: ['?o', '?b', '?bl'] }] },
    { task: 'deliver', name: 'M2 on table', params: ['?o', '?b'], pre: { k: 'and', items: [{ k: 'atom', name: 'on', args: ['?o', '?t'] }, { k: 'atom', name: 'detected', args: ['?o'] }, { k: 'atom', name: 'bin-at', args: ['?b', '?bl'] }] }, subtasks: [{ name: 'goto', args: ['?t'] }, { name: 'pick', args: ['?o', '?t'] }, { name: 'goto', args: ['?bl'] }, { name: 'place', args: ['?o', '?b', '?bl'] }] },
    { task: 'deliver', name: 'M3 not detected', params: ['?o', '?b'], pre: { k: 'and', items: [{ k: 'not', a: { k: 'atom', name: 'detected', args: ['?o'] } }, { k: 'atom', name: 'on', args: ['?o', '?t'] }] }, subtasks: [{ name: 'goto', args: ['?t'] }, { name: 'observe', args: ['?o', '?t'] }, { name: 'deliver', args: ['?o', '?b'] }] },
    { task: 'goto', name: 'already there', params: ['?l'], pre: { k: 'atom', name: 'at-robot', args: ['?l'] }, subtasks: [] },
    { task: 'goto', name: 'one hop', params: ['?l'], pre: { k: 'and', items: [{ k: 'atom', name: 'at-robot', args: ['?f'] }, { k: 'atom', name: 'connected', args: ['?f', '?l'] }] }, subtasks: [{ name: 'move', args: ['?f', '?l'] }] },
    { task: 'goto', name: 'via corridor', params: ['?l'], pre: { k: 'and', items: [{ k: 'atom', name: 'at-robot', args: ['?f'] }, { k: 'not', a: { k: 'atom', name: 'connected', args: ['?f', '?l'] } }, { k: 'atom', name: 'connected', args: ['?f', 'corridor'] }, { k: 'atom', name: 'connected', args: ['corridor', '?l'] }] }, subtasks: [{ name: 'move', args: ['?f', 'corridor'] }, { name: 'move', args: ['corridor', '?l'] }] },
  ],
};

export const CELL_PDDL_DOMAIN = `(define (domain cell)
  (:requirements :typing :durative-actions :fluents)
  (:types robot machine part location - object)
  (:predicates (free ?r - robot) (holds ?r - robot ?p - part) (at-part ?p - part ?l - location) (machine-free ?m - machine) (in-machine ?p - part ?m - machine) (processed ?p - part) (zone-free) (loader ?r - robot) (unloader ?r - robot) (delivered ?p - part))
  (:durative-action load :parameters (?r - robot ?p - part ?m - machine)
    :duration (= ?duration 5)
    :condition (and (at start (holds ?r ?p)) (at start (machine-free ?m)) (at start (zone-free)) (over all (holds ?r ?p)))
    :effect (and (at start (not (zone-free))) (at start (not (machine-free ?m))) (at end (in-machine ?p ?m)) (at end (not (holds ?r ?p))) (at end (free ?r)) (at end (zone-free))))
  (:durative-action process :parameters (?p - part ?m - machine)
    :duration (= ?duration 22)
    :condition (and (at start (in-machine ?p ?m)) (over all (in-machine ?p ?m)))
    :effect (at end (processed ?p)))
  (:durative-action unload :parameters (?r - robot ?p - part ?m - machine)
    :duration (= ?duration 5)
    :condition (and (at start (free ?r)) (at start (unloader ?r)) (at start (processed ?p)) (at start (in-machine ?p ?m)) (at start (zone-free)))
    :effect (and (at start (not (zone-free))) (at start (not (free ?r))) (at end (holds ?r ?p)) (at end (not (in-machine ?p ?m))) (at end (machine-free ?m)) (at end (zone-free))))
  (:durative-action take :parameters (?r - robot ?p - part ?l - location)
    :duration (= ?duration 4)
    :condition (and (at start (free ?r)) (at start (loader ?r)) (at start (at-part ?p ?l)))
    :effect (and (at start (not (free ?r))) (at end (holds ?r ?p)) (at end (not (at-part ?p ?l)))))
  (:durative-action put :parameters (?r - robot ?p - part)
    :duration (= ?duration 4)
    :condition (and (at start (holds ?r ?p)) (at start (processed ?p)))
    :effect (and (at end (delivered ?p)) (at end (not (holds ?r ?p))) (at end (free ?r)))))`;

export const CELL_PDDL_PROBLEM = (parts = 3) => `(define (problem cell-${parts})
  (:domain cell)
  (:objects m1 m2 - robot s - machine ${Array.from({ length: parts }, (_, i) => `p${i + 1}`).join(' ')} - part conv - location)
  (:init (free m1) (free m2) (loader m1) (unloader m2) (machine-free s) (zone-free) ${Array.from({ length: parts }, (_, i) => `(at-part p${i + 1} conv)`).join(' ')})
  (:goal (and ${Array.from({ length: parts }, (_, i) => `(delivered p${i + 1})`).join(' ')})))`;
