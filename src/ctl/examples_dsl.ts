/**
 * Course examples as control-DSL documents (loadable into the studio, used by the demo scenarios and the docs).
 */
import { youbotPlant, youbotSpecs, YOUBOT_UC, YOUBOT_UO, YOUBOT_PDDL_DOMAIN, YOUBOT_PDDL_PROBLEM } from './examples';
import { specsToDsl } from './analysis';
import { TEMPLATES } from './dsl';
import { ControlKind } from './model';

export interface ExampleDoc { id: string; kind: ControlKind; name: string; example: 'A' | 'B'; chapter: string; source: string; summary: string }

const S = youbotSpecs();
export const YOUBOT_DES_DSL = `des youBot mobile manipulator (example A)\n${specsToDsl(youbotPlant(true).map((d) => d.toSpec()), [S.E1, S.E2, S.E3, S.E5].map((d) => d.toSpec()), YOUBOT_UC, YOUBOT_UO)}\nfaults g_slip\ncheck ltl "R1: arm still while the base moves" G !((Base = 'mT' | Base = 'mB' | Base = 'mH') & (Arm = 'R' | Arm = 'W'))\ncheck ctl "an object can always still be delivered or given up" AG EF (Object = 'Bx' | Object = 'Un')`;

export const YOUBOT_DES_E4_DSL = `des youBot with E4 "never drop" (unrealisable)\n${specsToDsl(youbotPlant(true).map((d) => d.toSpec()), [S.E1, S.E2, S.E3, S.E5, S.E4].map((d) => d.toSpec()), YOUBOT_UC, YOUBOT_UO)}`;

/** Mission tree of example A with supervisor events on the actions (§6.6.2 + §3 supervisor as the protective filter). */
export const YOUBOT_BT_DSL = `bt youBot mission (example A)
fallback root
  sequence emergency
    condition estop
    action halt timeout=1
  sequence battery
    condition "battery < 0.2"
    sequence "to dock" memory
      action stow event=a_stow done=a_stowed timeout=10
      action goto zone=dock event=b_go_home arrive=b_arrive timeout=120
      action dock timeout=30
  fallback mission
    sequence collect
      condition "targets > 0 || held || arm_extended"   # keep the cycle running until the arm is stowed again
      sequence "collect one" memory
        action select_target
        action goto zone=table event=b_go_table arrive=b_arrive timeout=120
        action event emit=v_detect
        action reach event=a_reach done=a_reached timeout=10
        retry 2
          sequence grasp memory
            action refine_pose timeout=5
            action grasp event=g_close ok=g_ok miss=g_miss timeout=12 post="held"
        action stow event=a_stow done=a_stowed timeout=10
        action goto zone=bin event=b_go_box arrive=b_arrive timeout=120
        action reach event=a_reach done=a_reached timeout=10
        action place put=g_put timeout=10
        action stow event=a_stow done=a_stowed timeout=10
    sequence finish memory
      action goto zone=home event=b_go_home arrive=b_arrive timeout=120
      action report
outcomes halt=success,running
model grasp p=0.8 ticks=3
model estop p=0
check ltl "halt follows e-stop in the same tick" G("ok:estop" -> "tick:halt")
monitor G(held -> (held U placed))
monitor G(estop -> X halted)`;

export const YOUBOT_GR1_DSL = `gr1 youBot mission (example A, §8.5)
sys loc : base corridor table1 table2 bins
sys holding : none obj
env obj_at_t1 : bool
env obj_at_t2 : bool
env human_present : bool
env battery_low : bool
env_init !human_present
env_init !battery_low
env_trans obj_at_t1 && !(loc = 'table1') -> obj_at_t1'
env_trans obj_at_t2 && !(loc = 'table2') -> obj_at_t2'
env_live !human_present
env_live !battery_low
sys_init loc = 'base'
sys_init holding = 'none'
sys_trans loc = 'base' -> (loc' = 'base' || loc' = 'corridor')
sys_trans loc = 'corridor' -> (loc' = 'corridor' || loc' = 'base' || loc' = 'table1' || loc' = 'table2' || loc' = 'bins')
sys_trans loc = 'table1' -> (loc' = 'table1' || loc' = 'corridor')
sys_trans loc = 'table2' -> (loc' = 'table2' || loc' = 'corridor')
sys_trans loc = 'bins' -> (loc' = 'bins' || loc' = 'corridor')
sys_trans human_present -> loc' = loc
sys_trans !(holding' = 'obj' && loc' = 'base')
sys_trans holding' = 'obj' -> (holding = 'obj' || (loc = 'table1' && obj_at_t1) || (loc = 'table2' && obj_at_t2))
sys_trans holding' = 'none' -> (holding = 'none' || loc = 'bins')
sys_live !obj_at_t1 || (loc = 'table1' && holding = 'obj')
sys_live !obj_at_t2 || (loc = 'table2' && holding = 'obj')
sys_live holding = 'none' || loc = 'bins'
sys_live loc = 'base'
sys_live !battery_low || loc = 'base'`;

export const YOUBOT_PDDL_DSL = `pddl search=gbfs heuristic=hff
${YOUBOT_PDDL_DOMAIN}
${YOUBOT_PDDL_PROBLEM}
method deliver(?o ?b) "M1 holding" : (holding ?o) => goto(?bl) place(?o ?b ?bl)
method deliver(?o ?b) "M2 on a table" : (and (on ?o ?t) (detected ?o) (bin-at ?b ?bl)) => goto(?t) pick(?o ?t) goto(?bl) place(?o ?b ?bl)
method deliver(?o ?b) "M3 not detected" : (and (not (detected ?o)) (on ?o ?t)) => goto(?t) observe(?o ?t) deliver(?o ?b)
method goto(?l) "already there" : (at-robot ?l) =>
method goto(?l) "one hop" : (and (at-robot ?f) (connected ?f ?l)) => move(?f ?l)
method goto(?l) "via corridor" : (and (at-robot ?f) (not (connected ?f ?l)) (connected ?f corridor) (connected corridor ?l)) => move(?f corridor) move(corridor ?l)
task deliver(bolt1 bin-bolts)`;

export const CELL_PETRI_DSL = TEMPLATES.petri.replace('petri Two processes sharing a robot and a zone', 'petri Production cell fragment (example B, §4.6.4)');
export const CELL_SMV_DSL = TEMPLATES.smv.replace('smv Shared zone with priority', 'smv Shared zone Z with static priority (example B, §7.9)');
export const CELL_SMV_FAIR_DSL = `smv Shared zone Z with alternation (example B, §7.9 fix)
var r1.state : {idle, waiting, in_zone, leaving}
var r2.state : {idle, waiting, in_zone, leaving}
var owner : {none, one, two}
var turn : {one, two}
define g1 := (owner = none) & (turn = one | r2.state != waiting)
define g2 := (owner = none) & (turn = two | r1.state != waiting)
init r1.state := idle
init r2.state := idle
init owner := none
init turn := one
next r1.state := case
  r1.state = idle : {idle, waiting};
  r1.state = waiting & g1 : in_zone;
  r1.state = waiting & !g1 : waiting;
  r1.state = in_zone : {in_zone, leaving};
  r1.state = leaving : {idle, waiting};
esac
next r2.state := case
  r2.state = idle : {idle, waiting};
  r2.state = waiting & g2 : in_zone;
  r2.state = waiting & !g2 : waiting;
  r2.state = in_zone : {in_zone, leaving};
  r2.state = leaving : {idle, waiting};
esac
next owner := case
  r1.state = waiting & g1 : one;
  r2.state = waiting & g2 : two;
  r1.state = leaving & owner = one : none;
  r2.state = leaving & owner = two : none;
  TRUE : owner;
esac
next turn := case
  r1.state = leaving : two;
  r2.state = leaving : one;
  TRUE : turn;
esac
fairness !(r1.state = in_zone)
fairness !(r2.state = in_zone)
ltlspec "mutual exclusion" G !(r1.state = in_zone & r2.state = in_zone)
ltlspec "no starvation r1" G (r1.state = waiting -> F r1.state = in_zone)
ltlspec "no starvation r2" G (r2.state = waiting -> F r2.state = in_zone)
ctlspec "recoverable" AG EF owner = none
ctlspec "premise reachable" EF r2.state = waiting`;

export const COURSE_EXAMPLES: ExampleDoc[] = [
  { id: 'A-des', kind: 'des', name: 'A · DES plant + specifications E1 E2 E3 E5', example: 'A', chapter: '2–3', source: YOUBOT_DES_DSL, summary: '5 components (648 states), supervisor of 324 states, observer under unobservable g_slip, LTL/CTL checks.' },
  { id: 'A-des-e4', kind: 'des', name: 'A · E4 "never drop" is unrealisable', example: 'A', chapter: '3', source: YOUBOT_DES_E4_DSL, summary: 'Adding E4 makes the supremal controllable sublanguage empty: g_slip is uncontrollable.' },
  { id: 'A-bt', kind: 'bt', name: 'A · Mission behavior tree + supervisor + monitors', example: 'A', chapter: '6, 9', source: YOUBOT_BT_DSL, summary: 'Priority layers (e-stop, battery, mission), skill contracts, supervisor-gated events, R2 monitor.' },
  { id: 'A-gr1', kind: 'gr1', name: 'A · GR(1) mission synthesis', example: 'A', chapter: '8', source: YOUBOT_GR1_DSL, summary: '160-state game; realisable with GF ¬human_present, unrealisable without it.' },
  { id: 'A-pddl', kind: 'pddl', name: 'A · PDDL domain, planning and HTN', example: 'A', chapter: '10–11', source: YOUBOT_PDDL_DSL, summary: 'GBFS/h_FF plan for three parts; HTN methods with recursion (search before deliver).' },
  { id: 'A-mdp', kind: 'mdp', name: 'A · Grasp strategy MDP', example: 'A', chapter: '12', source: TEMPLATES.mdp.replace('mdp Grasp strategy', 'mdp Grasp strategy (example A, §12.2.5)'), summary: 'Refine first: 12.94 s vs 15.29 s; switch point at p ≈ 0.70.' },
  { id: 'A-pomdp', kind: 'pomdp', name: 'A · Classification POMDP', example: 'A', chapter: '12', source: TEMPLATES.pomdp.replace('pomdp Classification before placing', 'pomdp Classification before placing (example A, §12.4)'), summary: 'Belief 0.72 is not enough: look closer (11.07) beats placing (0.26); threshold 0.943.' },
  { id: 'A-modes', kind: 'hybrid', name: 'A · Mode automaton with hysteresis', example: 'A', chapter: '9', source: TEMPLATES.hybrid.replace('hybrid Speed modes', 'hybrid Speed modes near a human (example A, §9.2)'), summary: 'SLOW / NORMAL hysteresis 2.0 / 2.5 m and 0.4 s dwell time.' },
  { id: 'A-realtime', kind: 'realtime', name: 'A · Onboard task set and latency budget', example: 'A', chapter: '15', source: TEMPLATES.realtime.replace('realtime Onboard computer', 'realtime Onboard computer (example A, §15.3–15.7)'), summary: 'U = 0.80 above the Liu–Layland bound but schedulable; 313 ms latency → 47 mm error.' },
  { id: 'A-fta', kind: 'fta', name: 'A · Fault tree: contact with a human', example: 'A', chapter: '16', source: TEMPLATES.fta.replace('fta Contact with a human', 'fta Contact with a human (example A, §16.5)'), summary: 'Three cut sets of order 4, P ≈ 1.1e-12; B dominates; without the hardware chain: order 3.' },
  { id: 'A-stl', kind: 'stl', name: 'A · STL robustness of the distance requirement', example: 'A', chapter: '17', source: TEMPLATES.stl.replace('stl Distance to a human', 'stl Distance to a human (example A, R6)'), summary: 'ρ > 0 means margin, ρ < 0 a violation found by falsification.' },
  { id: 'B-petri', kind: 'petri', name: 'B · Cell Petri net: siphon, deadlock, monitor', example: 'B', chapter: '4', source: CELL_PETRI_DSL, summary: 'Siphon {rM, rZ, a2, b2} empties → deadlock; GMEC monitor a1 + b1 ≤ 1 makes the net live.' },
  { id: 'B-s3pr', kind: 's3pr', name: 'B · Cell as an S³PR', example: 'B', chapter: '4', source: TEMPLATES.s3pr.replace('s3pr Cell', 's3pr Cell (example B)'), summary: 'Same deadlock generated from process routes.' },
  { id: 'B-perf', kind: 'perf', name: 'B · Bottleneck, saturation and cycle time', example: 'B', chapter: '5', source: TEMPLATES.perf.replace('perf Production cell', 'perf Production cell (example B, §5.3, §5.6)'), summary: 'Machine S bounds 22 s (163.6/h); max-plus cycle 32 s; 2 machines → 16 s, 3 → 10.67 s.' },
  { id: 'B-smv', kind: 'smv', name: 'B · Shared zone: r2 starves under static priority', example: 'B', chapter: '7', source: CELL_SMV_DSL, summary: 'Mutual exclusion holds; no-starvation r2 fails with a cyclic counterexample.' },
  { id: 'B-smv-fair', kind: 'smv', name: 'B · Shared zone with alternation (fixed)', example: 'B', chapter: '7', source: CELL_SMV_FAIR_DSL, summary: 'Alternation restores liveness for both robots.' },
  { id: 'B-jobshop', kind: 'jobshop', name: 'B · Job shop: 54 s vs LB 44 s', example: 'B', chapter: '14', source: TEMPLATES.jobshop.replace('jobshop Cell with two machines', 'jobshop Cell with two machines (example B, §14.11)'), summary: 'Dispatch rules and branch & bound reach 54 s; the gap to 44 s is structural.' },
  { id: 'B-mrta', kind: 'mrta', name: 'B · Three AMRs, three pallets', example: 'B', chapter: '13', source: TEMPLATES.mrta.replace('mrta Three AMRs and three pallets', 'mrta Three AMRs and three pallets (example B, §13.7)'), summary: 'Hungarian 81 s vs greedy 140 s; minimax 55 s.' },
  { id: 'B-mapf', kind: 'mapf', name: 'B · Corridor: priority planning fails, CBS succeeds', example: 'B', chapter: '13', source: TEMPLATES.mapf.replace('mapf Corridor with a pocket', 'mapf Corridor with a pocket (example B, §13.5)'), summary: 'CBS plan, temporal plan graph robust to delays, corridor is a critical section.' },
  { id: 'B-reliability', kind: 'reliability', name: 'B · Reliability budget, SSM, PL', example: 'B', chapter: '16', source: TEMPLATES.reliability.replace('reliability Production cell', 'reliability Production cell (example B, §16.3, §16.8)'), summary: 'MTTF 633 h, A = 0.9937; S_p = 1.54 m, 0.31 m/s in a 1 m aisle; PL d with category 3.' },
  { id: 'B-stn', kind: 'stn', name: 'B · Delivery to a machine (STNU)', example: 'B', chapter: '10', source: TEMPLATES.stn.replace('stnu Delivery to a machine', 'stnu Delivery to a machine (example B, §10.7)'), summary: 'Dynamically controllable with travel [8,14]; not with [8,16].' },
  { id: 'B-acceptance', kind: 'acceptance', name: 'B · Acceptance statistics and traceability', example: 'B', chapter: '17', source: TEMPLATES.acceptance.replace('acceptance Mission time', 'acceptance Mission time (example, §17.7)'), summary: '97/100 does not demonstrate 0.95; upper bound of the mission time; sim-real gap.' },
];
