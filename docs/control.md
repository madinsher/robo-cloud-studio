# Control models: execution on the station and code generation

The Control tab turns control-design models (automata with supervisors, Petri nets, executable FSMs, behavior trees)
into running cell controllers and into deployable code. Sources: `src/ctl/` (executors, DSL, analysis),
`src/ctl/codegen/` (generators), `src/ui/control_ui.ts`, `src/ui/graph_editor.ts`, `src/ui/bt_editor.ts`.

**Diagram editors** (Text / Diagram views of the same DSL): automata (one diagram per automaton), Petri nets, behavior
trees and FSMs — one diagram per machine with the machine's outcomes as final nodes (⊙), outcome edges, and an inspector
for the state kind (pass / skill / decision / nested machine), initial state, timeout, the `do` binding (DSL text or a
program pick list), decisions (`expr -> outcome`), entry / exit actions, machine outcomes and the blackboard.

## Run on station

All executors use the same world (`src/ctl/world.ts`, `src/ctl/runtime.ts` `hostBindings`) over a **program backend**
(`src/ctl/host.ts`):

- In the app, programs start on the station runtime (`app.runtime`, shared clock): programs of different robots run
  concurrently, a second program for a busy robot waits (FIFO). Headless code can use `perRobotBackend` (one
  ProgramSimulator per robot) or `simulatedBackend` (durations by name).
- A program that does not compile or stops at an error **fails** the action (the message is logged).
- One action per robot at a time (`RobotLocks`); halting an action stops only its own program / move.
- `move target=` moves the target's own robot (explicit `robot=`, the target's robot link, its robot parent, the frame
  used by exactly one robot, or the only robot); `linear=true` plays the planned MoveL (straight TCP path).
- The blackboard holds the merged signal table (runtime signals: robot DI/DO, program signals; process-component
  signals), `program_playing`, per mobile robot its state (`youBot.zone`, `battery`, `held`…) and model variables.

| Kind | Semantics |
|---|---|
| `des` automata | every automaton is a state machine; entering a state starts its `action`; `done=` / `fail=` fire events; controllable events fire when enabled in all sharing components, allowed by the supervisor and by their `guard "expr"`; `sensor ev "expr"` raises an event on each rising edge; unobservable events move the plant only (observer-based supervisor table, also used by the Python export). Ends when nothing can happen any more: `done` in a marked configuration, `deadlock` otherwise, `failed` after a failed action without `fail=`; with `terminate marked` the run also ends `done` as soon as every automaton and the supervisor are marked and idle after at least one event (an event-driven cell otherwise keeps waiting for its next part). |
| `petri` nets | single-server transitions by default (`servers=`), `delay=` / sampled `rate=`, `immediate` pre-empts timed ones, `priority=`, `capacity=`, `guard="expr"`, input transitions `on=event`; a failed action returns its tokens. Ends `done` in a declared `final` marking (or tokens only in resource / idle / sink places), `deadlock` otherwise — including a **partial deadlock** (some activity places can never be left while other transitions still cycle; exact, from the reachability set of the current marking), `failed` after failed actions. `applyDeadlockMonitors` (supervisor.ts) adds the GMEC monitors of the analysis. |
| `fsm` | yasmin-style hierarchical machines: states return named outcomes (`ok` / `fail` / `timeout` / `canceled`, `done=` / `fail=` rename them; a skill with several outcomes lists them, `do skill=plan outcomes=found,none_left,error`, and reports the one it ended with in `node.outcome` — the MURKA retail orchestrator runs this way, docs/murka-retail.md), outcome → sibling state or → outcome of the enclosing machine, nested `machine` blocks, decision states (`when "expr" -> outcome`, `otherwise`), `entry` / `exit` actions, state timeouts (a machine timeout pre-empts its children), `cancel()`. |
| `bt` | py_trees semantics (non-memory sequences are reactive), parallel with completed-child memory, retry / repeat / timeout decorators, unknown actions fail; runs on arm-only stations (program / move / signal / wait / set / await) and with a mobile robot (goto, dock, reach / stow / grasp / place run by the arm programs of a mobile manipulator). |

Action bindings (`action <state|transition> …`, `do …`): `program=` `target=` (`linear=true`) `goto=` `signal=`
`wait=` `event=` `set=` (`value="=expr"`) `await=` `skill=`, with `robot=`, `timeout=`, `done=`, `fail=`.

`goto zone=` of a mobile robot plans with `planAndFollow` (src/mobile/navigate.ts, I8: footprint- and drive-aware path
on the station map and the crop obstacles, simulated with the drive model) and plays that trajectory; without a path the
action fails ("no collision-free path"), it never drives through obstacles (tests/ctl_goto_nav.test.ts). A planner
registered with `registerGotoPlanner` replaces it.

**Course examples as runnable stations** (*Course examples ▾ → A/B — runnable station*, `src/ctl/course_stations.ts`):
A — youBot base with a UR3e arm, zones home / table / bin / dock, arm programs Reach / Stow / Grasp / Release,
automaton under the course supervisor and the BT mission; B — production cell with UR10e (M1), IRB 1200 (M2) and an AMR,
the course Petri net (deadlocks at {a1 b1}) and the same net with its GMEC monitor (live).

## Code generation (Export ▾)

| Export | Files | Validation (tests) |
|---|---|---|
| RoboDK station script | `<model>_robodk.py`: the model as JSON + a Python port of the executors over robolink (RunProgram / Busy, MoveJ / MoveL, setParam / getParam signals) | `tests/ctl_codegen_robodk.test.ts`: py_compile and run under RoboDK's Python 3.7 against a virtual-clock robolink stand-in — same firing / outcome trace and times as the studio executors |
| Robot programs with IO handshake | per robot (its own post, `Robot.postProcessor`): server = reset DONE, endless loop over `IF DI[START_k]` (one operation: `WAIT DI`), operation wrapper = call the bound program / move, `DO[DONE_k]`, wait START low, DONE low; robot I/O map CSV. In the studio the same programs (named START_x / DONE_x signals) plus `Main <model>` (Start thread per server) run on the station runtime | `tests/ctl_deploy.test.ts`: the executor as cell controller (`handshakeBackend`) drives the generated programs on the station runtime — same cycle as direct execution, START once per operation, robots concurrent; ABB RAPID / Fanuc LS structure |
| IEC 61131-3 ST | automata: CASE state machines + supervisor table; Petri nets: marking logic, TON timers, START / DONE handshakes; `io_map.csv` (%QX / %IX) | `tests/ctl_codegen_formats.test.ts`: block / declaration checker |
| PLCopen XML (TC6 2.01) SFC | 1-safe Petri nets as Grafcet | XML parse, steps = places, initial steps, connection references |
| IEC 61131-3 ST for `fsm` | one CASE state machine per machine, nested machines inline in their parent's branch, outcome registers; START_x / DONE_x / FAIL_x handshake, RES_x = index of a named skill outcome; TON for waits and state timeouts; decisions in order; FSM_DONE / FSM_OUTCOME; `io_map.csv` | `tests/ctl_fsm_st.test.ts`: the program is executed by an ST interpreter against a scripted skill server — same state sequence and outcome as FsmExecutor (MURKA retail mission with loops, retries, error continuation; decisions, waits, timeout) |
| PLCopen XML SFC for `fsm` | flattened hierarchy: leaf states = steps, nested machines entered at their initial leaf, root outcomes = END steps, ordered transition conditions on the same handshake / `<step>.T` | same test: references, step / transition counts, token game = FsmExecutor state sequence |
| ROS 2 packages | `<name>_control` (ament_python): `supervisor` node (same runtime over std_srvs/Trigger services `/<ns>/program/<p>`, `/<ns>/goto/<zone>`, `/<ns>/move/<target>`, `/<ns>/skill/<s>`, JSON signals `/<ns>/signals`, state `/<ns>/fsm/state`), simulated `skill_server`, launch (`sim:=`, `engine:=runtime|yasmin|btcpp`, ends with the control program); `yasmin_fsm` for FSMs; `<name>_bt` (ament_cmake): BehaviorTree.CPP v4 XML (reactive / memory sequences, Parallel thresholds, RetryUntilSuccessful, Repeat, Timeout, ScriptCondition, Sleep) + C++ skill stubs + `bt_runner` | `tests/ctl_codegen_ros.test.ts` (WSL Ubuntu-22.04, ROS 2 Jazzy): colcon build of all packages, py_compile, end-to-end runs over DDS (Petri / automaton traces = studio, BT.CPP SUCCESS, yasmin outcome), yasmin on a robolink stand-in = FSM executor trace |
| PNML / SCXML / NuSMV / UPPAAL | interchange; PNML import (`pnmlToPetriDsl`), BehaviorTree.CPP XML import (`btCppXmlToDsl`) | PNML exact round trip (net, timing, guards, bindings, reachability), XML structure, SMV structure |

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
