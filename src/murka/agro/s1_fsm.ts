/**
 * Benchmark S1 automata (agromarca article/BENCHMARK_SCENARIO.md §3–§4) as studio `fsm` documents (src/ctl/fsm.ts):
 * the same DSL the Control tab edits, checks (checkFsm), draws and exports (yasmin, SCXML…). Every `skill=s1_*` is
 * bound by the S1 runner (s1.ts) to the role's world actions; signals of the 2023 article are kept in the comments
 * (i_rob_S, i_rob_GW, i_rob_Work, i_rob_End, o_rob_Ready, o_rob_Arrived, o_rob_WD).
 *
 * Interpretation notes: WaitSwap → PickTree [trees left] of §4.2 goes through AdvanceTree (the harvester stands at the
 * tree it just picked); the drone's Start → Takeoff waits for i_rob_GW (the mission's survey command); End states wait
 * for i_rob_End (return_home) where §4 has them. Level D (§7): a harvester without rows may be released for a RECONFIGURE
 * contract (Released → Reconfigure, depot 180 s) and ends its harvester automaton with outcome `reconfigured`; the runner
 * then starts the Logistics automaton on the same platform.
 */

export const S1_MISSION_FSM = `fsm S1 Mission
outcomes done aborted
state IDLE initial
  do skill=s1_mission_start
  fail -> aborted
  ok -> SURVEY
state SURVEY
  do skill=s1_survey fail=timeout_reached
  ok -> PLAN
  timeout_reached -> aborted
state PLAN
  do skill=s1_plan fail=timeout_reached
  ok -> EXECUTE
  timeout_reached -> aborted
state EXECUTE
  do skill=s1_execute fail=timeout_reached
  ok -> RETURN
  timeout_reached -> aborted
state RETURN
  do skill=s1_return fail=timeout_reached
  ok -> done
  timeout_reached -> aborted`;

export const S1_DRONE_FSM = `fsm S1 Drone
blackboard leg=0 legs=0
outcomes end failed
state Wait initial
  do skill=s1_wait_start
  fail -> failed
  ok -> Start
state Start
  do skill=s1_await_survey
  fail -> failed
  ok -> Takeoff
state Takeoff
  do skill=s1_takeoff
  fail -> failed
  ok -> SurveyLeg
machine SurveyLeg
  outcomes all_legs failed
  state FlyLeg initial
    do skill=s1_fly_leg
    fail -> failed
    ok -> MoreLegs
  state MoreLegs decision
    when "leg < legs" -> more
    otherwise -> all_legs
    more -> TurnNextLeg
  state TurnNextLeg
    do skill=s1_turn_leg
    fail -> failed
    ok -> FlyLeg
  all_legs -> ReportMap
  failed -> failed
state ReportMap
  do skill=s1_report_map
  fail -> failed
  ok -> ReturnHome
state ReturnHome
  do skill=s1_return_home
  fail -> failed
  ok -> Land
state Land
  do skill=s1_land
  fail -> failed
  ok -> End
state End
  do skill=s1_end
  fail -> failed
  ok -> end`;

export const S1_HARVESTER_FSM = `fsm S1 Harvester
blackboard box_full=false trees_left=0 reconfigure=false
outcomes end failed reconfigured
state Wait initial
  do skill=s1_wait_start
  fail -> failed
  ok -> Start
state Start
  do skill=s1_ready
  fail -> failed
  ok -> AwaitRow
state AwaitRow
  do skill=s1_await_row done=assigned fail=released
  assigned -> GotoWaypoints
  released -> Released
state Released decision
  when "reconfigure" -> reconf
  otherwise -> home
  reconf -> Reconfigure
  home -> ReturnHome
state Reconfigure
  do skill=s1_reconfigure
  fail -> failed
  ok -> reconfigured
machine GotoWaypoints
  outcomes ready failed
  state FollowWaypoints initial
    do skill=s1_goto_row
    fail -> failed
    ok -> ReachedGoal
  state ReachedGoal
    do skill=s1_arrived
    fail -> failed
    ok -> ready
  ready -> AlignRow
  failed -> failed
state AlignRow
  do skill=s1_align_row
  fail -> failed
  ok -> HarvestLoop
machine HarvestLoop
  outcomes row_done failed
  state PickTree initial
    do skill=s1_pick_tree
    fail -> failed
    ok -> Check
  state Check decision
    when "box_full" -> full
    when "trees_left > 0" -> next
    otherwise -> row_done
    full -> WaitSwap
    next -> AdvanceTree
  state WaitSwap
    do skill=s1_wait_swap
    fail -> failed
    ok -> AfterSwap
  state AfterSwap decision
    when "trees_left > 0" -> next
    otherwise -> row_done
    next -> AdvanceTree
  state AdvanceTree
    do skill=s1_advance_tree
    fail -> failed
    ok -> PickTree
  row_done -> RowDone
  failed -> failed
state RowDone
  do skill=s1_row_done
  fail -> failed
  ok -> AwaitRow
state ReturnHome
  do skill=s1_return_home
  fail -> failed
  ok -> End
state End
  do skill=s1_end
  fail -> failed
  ok -> end`;

export const S1_LOGISTICS_FSM = `fsm S1 Logistics
outcomes end failed
state Wait initial
  do skill=s1_wait_start
  fail -> failed
  ok -> Start
state Start
  do skill=s1_ready
  fail -> failed
  ok -> Idle
state Idle
  do skill=s1_await_request done=request fail=home
  request -> GotoHarvester
  home -> ReturnHome
state GotoHarvester
  do skill=s1_goto_harvester
  fail -> failed
  ok -> SwapBox
state SwapBox
  do skill=s1_swap_box
  fail -> failed
  ok -> GotoCP
state GotoCP
  do skill=s1_goto_cp
  fail -> failed
  ok -> Unload
state Unload
  do skill=s1_unload
  fail -> failed
  ok -> Idle
state ReturnHome
  do skill=s1_return_home
  fail -> failed
  ok -> End
state End
  do skill=s1_end
  fail -> failed
  ok -> end`;

export const S1_FSMS = { mission: S1_MISSION_FSM, drone: S1_DRONE_FSM, harvester: S1_HARVESTER_FSM, logistics: S1_LOGISTICS_FSM };
