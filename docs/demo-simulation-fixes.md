# Demo simulation fixes

The previous demo smoke test checked whether the first program compiled. The regression suite in
`tests/demo_simulation.test.ts` checks every industrial demo program against self-collisions and verifies
its actual world TCP at every Cartesian endpoint (position error below 0.1 mm; full-pose orientation
error below 0.001 rad for six-axis robots).

## Changes

- Program trajectories use the tool selected by the program for FK and IK, including linear and circular moves.
- Seeking forward applies each event at its own robot configuration and updates held objects before release.
  Seeking backward restores initial object poses, tool attachments, robot state, IO and signals before replay.
  Small playback ticks, a single seek to the end, and replay now give the same final object poses.
- The UI checks self-collisions before playback even when environment collision checking is disabled.
  Invalid programs do not play. Link adjacency uses the intervening physical chain length for approximate
  procedural geometry instead of excluding every pair within three link indices. Initial contacts are
  limited to their specific parts and original penetration; they never disable a robot's self-collisions.
- The UR10e pick-and-place demo starts in a home configuration whose outbound and return paths do not
  fold the arm through itself. Three discharge points are on the conveyor at distinct positions, with
  box bottoms 1 mm above the belt. Demo/UI conveyors have optional floor supports. Packing-line products
  remain at the grader's configured support height rather than dropping to its origin on the floor.
- Runnable group examples can build their robots, names, initial coordinates and maps from the model.
  Scene construction reuses named robots and is repeatable without duplicates. The supplied station
  mission also adds its provided supervisor and mode-machine dependencies. The scene button works for
  every executable group kind, and an empty station can prepare its scene on Run. Warehouse construction
  can reuse previously configured robots. Completion stops the robots' motion status.
- Loading, restoring or resetting a station clears the old world-loop callbacks and clock state.

## Reproduce

1. Load `?demo=pickplace&server=off`, run PickPlace, and seek to the end. All three boxes should be on the belt.
   Stop and repeat; stop should restore the original box locations.
2. Load `?demo=tutorial&server=off` and run PickPart. The part should be transferred to the bin after release.
3. Load `?demo=packing&server=off` and start the world clock. Products should traverse the grader at 800 mm.
4. On a new station, add Group → Course examples → ПР1 chain, formation, or a warehouse example.
   The scene is prepared automatically. Run on fleet should move the required robots. Build scene twice
   should not duplicate them. Analysis-only kinds remain reports rather than executable robot programs.

## Validation

```bash
npm run build
npm test
E2E_CHROME=/usr/bin/chromium npm run test:e2e
```

The new suite has 36 regressions: seven demo stations, direct/incremental/repeated object transfers,
program tool changes, initial self-collision rejection, grader support height, and 25 executable group
examples built from empty stations. The short group tests validate setup and finite execution, not
completion of every long mission. Existing scenario tests exercise the algorithms separately.

Collision geometry for procedural library robots remains approximate. Exact imported vendor geometry
is still required for model-specific clearance assessment; these tests validate the built-in model.
