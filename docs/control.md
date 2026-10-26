# Control models: execution on the station and code generation

The Control tab turns control-design models (automata with supervisors, Petri nets, executable FSMs, behavior trees)
into running cell controllers and into deployable code. Sources: `src/ctl/` (executors, DSL, analysis),
`src/ctl/codegen/` (generators), `src/ui/control_ui.ts`, `src/ui/graph_editor.ts`, `src/ui/bt_editor.ts`.

**Diagram editors** (Text / Diagram views of the same DSL): automata (one diagram per automaton), Petri nets, behavior
trees and FSMs — one diagram per machine with the machine's outcomes as final nodes (⊙), outcome edges, and an inspector
for the state kind (pass / skill / decision / nested machine), initial state, timeout, the `do` binding (DSL text or a
program pick list), decisions (`expr -> outcome`), entry / exit actions, machine outcomes and the blackboard. The
Petri inspector edits transition guards (`guard=`) and input events (`on=`), and in the document the final markings
(`xA d2; xB a2`) and sensors (`event "expr"; …`); the automaton inspector edits a guard per controllable event of an edge,
the sensors and `terminate marked`. Renaming an automaton or a state updates the bindings and the `check` atoms that
use the name. New behavior-tree actions carry no default timeout (set one for the bound program in the inspector).
Edges between the same two nodes fan out symmetrically: two opposite edges (`I ⇄ W`) are two arcs bowed to either side,
leaving the node circles apart, with their labels outside, so each one (line or label) is selected by a click
(`tests/ui_graph_edges.test.ts`: the 12 px hit strokes never overlap, at the template layout, after Auto layout and
after a drag; checked in Chrome: every click at the label and at 25 / 50 / 75 % of either arc selects that arc).

## Run on station

All executors use the same world (`src/ctl/world.ts`, `src/ctl/runtime.ts` `hostBindings`) over a **program backend**
(`src/ctl/host.ts`):

- In the app, programs start on the station runtime (`app.runtime`, shared clock): programs of different robots run
  concurrently, a second program for a busy robot waits (FIFO). Headless code can use `perRobotBackend` (one
  ProgramSimulator per robot) or `simulatedBackend` (durations by name).
- A program that does not compile or stops at an error **fails** the action (the message is logged).
- One action per robot at a time (`RobotLocks`); halting an action stops only its own program / move. The log tells
  when a contested robot really starts: the transition / state that fired is logged at once (`tC started: program
  A_Place on R1`), then `program A_Place waits for R1 (busy)` and `program A_Place starts on R1 after waiting 2.30 s`
  (`tests/ctl_exec_lock_log.test.ts`).
- Behavior-tree missions write the Mission log as the other executors do: every action that runs over several ticks
  when it starts and ends (`pickA: program A_Pick on R1 started` / `… success`), every failure, and the tree result
  (`tree success`) — an arm-only tree of programs used to leave it empty (`tests/ctl_bt_editor_mission.test.ts`).
- `move target=` moves the target's own robot (explicit `robot=`, the target's robot link, its robot parent, the frame
  used by exactly one robot, or the only robot); `linear=true` plays the planned MoveL (straight TCP path).
- The blackboard holds the merged signal table (runtime signals: robot DI/DO, program signals; process-component
  signals), `program_playing`, per mobile robot its state (`youBot.zone`, `battery`, `held`…) and model variables.

| Kind | Semantics |
|---|---|
| `des` automata | every automaton is a state machine; entering a state starts its `action`; `done=` / `fail=` fire events; controllable events fire when enabled in all sharing components, allowed by the supervisor and by their `guard "expr"`; `sensor ev "expr"` raises an event on each rising edge; unobservable events move the plant only (observer-based supervisor table, also used by the Python export). Ends when nothing can happen any more: `done` in a marked configuration, `deadlock` otherwise, `failed` after a failed action without `fail=`; with `terminate marked` the run also ends `done` as soon as every automaton and the supervisor are marked and idle after at least one event (an event-driven cell otherwise keeps waiting for its next part). |
| `petri` nets | single-server transitions by default (`servers=`), `delay=` / sampled `rate=`, `immediate` pre-empts timed ones, `priority=`, `capacity=`, `guard="expr"`, input transitions `on=event`; a failed action returns its tokens. Ends `done` in a declared `final` marking (or tokens only in resource / idle / sink places), `deadlock` otherwise — including a **partial deadlock** (some activity places can never be left while other transitions still cycle; exact, from the reachability set of the current marking; not reported while a declared `final` marking is still reachable, e.g. the loser of a race). After a partial deadlock no new firing starts and the run ends when the firings in progress have completed (robots are not stopped in mid-motion). `failed` after failed actions. `applyDeadlockMonitors` (supervisor.ts) adds the GMEC monitors of the analysis. |
| `fsm` | yasmin-style hierarchical machines: states return named outcomes (`ok` / `fail` / `timeout` / `canceled`, `done=` / `fail=` rename them; a skill with several outcomes lists them, `do skill=plan outcomes=found,none_left,error`, and reports the one it ended with in `node.outcome` — the MURKA retail orchestrator runs this way, docs/murka-retail.md), outcome → sibling state or → outcome of the enclosing machine, nested `machine` blocks, decision states (`when "expr" -> outcome`, `otherwise`), `entry` / `exit` actions, state timeouts (a machine timeout pre-empts its children), `cancel()`. |
| `bt` | py_trees semantics (non-memory sequences are reactive), parallel with completed-child memory, retry / repeat / timeout decorators, unknown actions fail (an action timeout is logged); runs on arm-only stations (program / move / signal / wait / set / await) and with a mobile robot (goto, dock, reach / stow / grasp / place run by the arm programs of a mobile manipulator; a grasp by an arm program succeeds only when the gripper holds an object afterwards; `parts_here` counts the loose parts in the robot's zone). *Run mission* ends at the first success or failure of the root for a tree without `report` (as the generated runtimes); a reporting tree ends when it reports, on a failure persisting 0.5 s, after 3 failed cycles in a row or on a supervisor denial. |

Action bindings (`action <state|transition> …`, `do …`): `program=` `target=` (`linear=true`) `goto=` `signal=`
`wait=` `event=` `set=` (`value="=expr"`) `await=` `skill=`, with `robot=`, `timeout=`, `done=`, `fail=`.

`check ltl|ctl ["name"] <formula>`: the name is a quoted string right after the kind (a quoted atom inside the formula,
`G("ok:estop" -> "tick:halt")`, is not one). In automata and Petri nets an atom over a name the model does not have
(an automaton, a state of an automaton, a place) is an error, not a vacuous verdict.

`goto zone=` of a mobile robot plans with `planAndFollow` (src/mobile/navigate.ts, I8: footprint- and drive-aware path
on the station map and the crop obstacles, simulated with the drive model) and plays that trajectory; without a path the
action fails ("no collision-free path"), it never drives through obstacles (tests/ctl_goto_nav.test.ts). A planner
registered with `registerGotoPlanner` replaces it.

**Course examples as runnable stations** (*Course examples ▾ → A/B — runnable station*, `src/ctl/course_stations.ts`):
A — youBot base with a UR3e arm, zones home / table / bin / dock, arm programs Reach / Stow / Grasp / Release,
automaton under the course supervisor and the BT mission (both detect the part with `parts_here > 0` at the table: run
after the part was delivered, they do not grasp and the mission does not report); B — production cell with UR10e (M1), IRB 1200 (M2) and an AMR,
the course Petri net (deadlocks at {a1 b1}) and the same net with its GMEC monitor (live).

## Code generation (Export ▾)

| Export | Files | Validation (tests) |
|---|---|---|
| RoboDK station script | `<model>_robodk.py`: the model as JSON + a Python port of the executors over robolink (RunProgram / Busy, MoveJ / MoveL, setParam / getParams signals). `python script.py [seconds] [speed]`: the run sets RoboDK's simulation speed (default 1, as the studio runs; 0 keeps RoboDK's, which starts at 5) and restores it; delays / waits / timeouts run on the station's simulation clock (`RDK.SimulationTime()`), and a timer that ends while a program is still seen running is applied first, so a race does not depend on the speed or on the robolink round trips (`Busy()` costs 0.1–0.2 s while a program runs); guards / conditions / decisions read the signals again after a program ended (its `SetDO` outputs); signals are read with one `getParams()` per read, so a signal no program has set yet prints no `WARNING: Invalid/unknown parameter` line per tick (RoboDK 5.9.4, verifier station: 0 warning lines for the guard net and the BT; `ctl_codegen_robodk_timing.test.ts` with a stand-in that prints RoboDK's warning) | `tests/ctl_codegen_robodk.test.ts`: py_compile and run under RoboDK's Python 3.7 against a virtual-clock robolink stand-in — same firing / outcome trace and times as the studio executors; `tests/ctl_codegen_robodk_timing.test.ts`: a RoboDK-like stand-in (speed 5, slow Busy(), program outputs) — the BT condition after a parallel and the timer / program race end as in the studio. Checked against RoboDK 5.9.4 (two-robot station): BT success 9.7 s, race → B_Work on R2 at 2.0 s at speed 1 and 5, guard / FSM / DES as in the studio |
| Robot programs with IO handshake | per robot (its own post, `Robot.postProcessor`): server = reset DONE, endless loop over `IF DI[START_k]` (one operation: `WAIT DI`), operation wrapper = call the bound program / move, `DO[DONE_k]`, wait START low, DONE low; robot I/O map CSV. In the studio the same programs (named START_x / DONE_x signals) plus `Main <model>` (Start thread per server) run on the station runtime | `tests/ctl_deploy.test.ts`: the executor as cell controller (`handshakeBackend`) drives the generated programs on the station runtime — same cycle as direct execution, START once per operation, robots concurrent; ABB RAPID / Fanuc LS structure |
| IEC 61131-3 ST | automata: CASE state machines + supervisor table, one event per scan with the DONE / sensor edges latched until their event is taken (two robots finishing in the same scan are both seen); Petri nets: marking logic, TON timers, START / DONE handshakes; `io_map.csv`. Addresses (all ST exports): bits from %IX0.0 / %QX0.0, INT words after the bit bytes, REAL (numeric signals) double words after the words - no overlap in the CODESYS (word / dword index) or the Siemens (byte offset) address model | `tests/ctl_codegen_formats.test.ts`: block / declaration checker; `tests/ctl_fsm_st.test.ts`: address overlaps in both models, same-scan DONE edges |
| PLCopen XML (TC6 2.01) SFC | 1-safe Petri nets as Grafcet | XML parse, steps = places, initial steps, connection references |
| IEC 61131-3 ST for `fsm` | one CASE state machine per machine, nested machines inline in their parent's branch, outcome registers; START_x / DONE_x / FAIL_x handshake, RES_x = index of a named skill outcome; TON for waits and state timeouts; decisions in order; FSM_DONE / FSM_OUTCOME; `io_map.csv` | `tests/ctl_fsm_st.test.ts`: the program is executed by an ST interpreter against a scripted skill server — same state sequence and outcome as FsmExecutor (MURKA retail mission with loops, retries, error continuation; decisions, waits, timeout) |
| PLCopen XML SFC for `fsm` | flattened hierarchy: leaf states = steps, nested machines entered at their initial leaf, root outcomes = END steps, ordered transition conditions on the same handshake / `<step>.T` | same test: references, step / transition counts, token game = FsmExecutor state sequence |
| ROS 2 packages | `<name>_control` (ament_python): `supervisor` node (same runtime over std_srvs/Trigger services `/<ns>/program/<p>`, `/<ns>/goto/<zone>`, `/<ns>/move/<target>`, `/<ns>/skill/<s>`, JSON signals `/<ns>/signals`, state `/<ns>/fsm/state`), simulated `skill_server` (program durations and the signals each station program sets, e.g. `SetDO Ready, 1`: published on `/<ns>/signals` / `/<ns>/signal_updates` and reported with the Trigger response, so a guard / condition / decision sees them when the program is done), launch (`sim:=`, `engine:=runtime|yasmin|btcpp`, ends with the control program); `yasmin_fsm` for FSMs; `<name>_bt` (ament_cmake): BehaviorTree.CPP v4 XML (reactive / memory sequences, Parallel thresholds, RetryUntilSuccessful, Repeat, Timeout, ScriptCondition, Sleep) + C++ skill stubs + `bt_runner` | `tests/ctl_codegen_ros.test.ts` (WSL Ubuntu-22.04, ROS 2 Jazzy): colcon build of all packages, py_compile, end-to-end runs over DDS (Petri / automaton traces = studio, BT.CPP SUCCESS, yasmin outcome), yasmin on a robolink stand-in = FSM executor trace |
| PNML / SCXML / NuSMV / UPPAAL | interchange; PNML import (`pnmlToPetriDsl`), BehaviorTree.CPP XML import (`btCppXmlToDsl`). UPPAAL carries the synthesised supervisor (`sup`, enabled-event guards, moves on every event); the Petri SMV keeps transition guards / input events as environment inputs (`IVAR g_t`) | PNML exact round trip (net, timing, guards, bindings, reachability), XML structure, SMV structure; `tests/ctl_interchange_supervisor.test.ts` |

## Group FSMs (agromarca-style)

An `fsm` with `group all | robots=a,b | capability=c` is a per-robot template (`src/ctl/group_fsm.ts`, Run on fleet):
each member gets its own instance with its own bindings (goto / dock act on its robot) and blackboard (`robot`, `caps`,
`cap_<c>`, own zone / battery), messages (`skill=send msg= to=<robot|all|cap:c>` → `ev_<m>` / `msg_<m>`), and the contract
net (`skill=announce task= cap= zone=` → bids = distance + battery → ACCEPTED / REJECTED → `start_task` EXECUTING →
`complete` COMPLETED, no bidder → CANCELLED). `$name` in bindings reads the blackboard (`goto=$task_zone`). Capabilities
come from `capabilities r=c1,c2` lines or a YAML / JSON ontology (robot types + equipment + robots; *Group examples →
Import a capability ontology…*). *Group examples → Garden robots* opens a demo station. Tests: `tests/ctl_group_fsm.test.ts`.

## Group control on the fleet

`src/mrs/runtime.ts`: every runnable group-control kind drives the robots through their own kinematic controller by
default (a point ahead of the axle tracks the reference; (v, ω) saturated by the robot's speed, yaw-rate and
acceleration limits). Consensus, coverage and safety run in closed loop on the measured robots (safety keeps
d_safe + 2·lookahead between tracked points); swarm / MAPF / warehouse plans are tracked references. `drive pose`
shows the ideal model (not kinematically compliant). `tests/mrs_kinematics.test.ts` checks every runnable example.

## Running the acceptance tests

```bash
npx vitest run tests/ctl_ tests/mrs_ --pool=threads          # all control / group-control tests
npx vitest run tests/ctl_codegen_ros.test.ts --pool=threads   # needs WSL Ubuntu-22.04 with ROS 2 Jazzy (~5 min, skipped otherwise)
# note: in this WSL2 the UDP loopback drops rclcpp ↔ rclpy service traffic; the test (and the generated README) use
# FASTDDS_BUILTIN_TRANSPORTS=SHM. Python ↔ Python works over the default transports.
npx vitest run tests/ctl_codegen_robodk.test.ts               # needs C:/RoboDK/Python37 (skipped otherwise)
```
