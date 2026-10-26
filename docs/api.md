# RoboDK-compatible API: transports, Python runtime, compatibility

The studio exposes the RoboDK API (`Robolink` / `Item`, src/api/robolink.ts) to scripts in four ways. Every request
ends up in the same JavaScript implementation, so behaviour does not depend on the transport.

| Transport | Endpoint | Clients |
|---|---|---|
| JSON-RPC over WebSocket | `ws://host:20500` (`STUDIO_PORT`) | Python drop-in `python/robodk` (+ legacy `python/robolink`), C# `clients/csharp` |
| JSON lines over TCP | `host:20501` (`STUDIO_TCP_PORT`) | MATLAB `clients/matlab`, C++ `clients/cpp`, PLCs |
| RoboDK binary API protocol | `host:$STUDIO_RDK_PORT` (opt-in) | the **official** `robodk` pip package / RoboDK's embedded Python, RoboDK's own C#, C++ and MATLAB APIs, unmodified |
| In-process | — | browser console (`window.RDK`), Pyodide station scripts (src/api/runner_pyodide.ts) |

When a browser tab is connected to the server as host (`?server=ws://host:20500`), WebSocket, TCP and binary-protocol
requests are all relayed to it and run against the live 3D station; otherwise the server runs a headless station with
a realtime clock (`STUDIO_SIM_SPEED`). Per-method timeouts (src/api/rpc.ts `methodTimeoutMs`) replace the old 30 s cap
(`WaitForEvent(t)` waits t + 30 s, `WaitFinished`/blocking moves have none). The browser bridge takes one undo snapshot
per burst of mutating calls (debounced, none for read-only calls) and no longer recompiles the program preview after
each call (that cancelled `RunProgram`).

The page connects to the server only when asked: `?server=ws://host:port` (`?server=auto` = port 20500 of the page's
host) or Connect › Studio server › Connect. It no longer probes `ws://<host>:20500` on every load.

## Access control (src/api/security.ts)

The API runs code (station Python programs on CPython through `POST /python/run`, RoboDK posts and user post sources
through `POST /posts/native/run`, `RunProgram` of Python items), so the server serves this machine only by default:

| Setting | Default | Effect |
|---|---|---|
| `STUDIO_HOST` | `127.0.0.1` | bind address of the HTTP / WebSocket, TCP JSON-lines and binary-protocol sockets; `0.0.0.0` serves other hosts (WSL, LAN) — every host that reaches it can run code |
| `STUDIO_ALLOWED_ORIGINS` | — | extra browser origins (comma separated, `*` = any). A request with an `Origin` header (every request a web page makes, WebSocket upgrades included) is refused with 403 unless the origin is local (`http(s)://localhost`, `127.0.0.1`, `[::1]`, `*.localhost`, any port) or listed |
| `STUDIO_ALLOWED_HOSTS` | — | DNS names the server may be reached by; other `Host` headers are refused (DNS rebinding). Loopback names and IP literals always pass |
| `STUDIO_TOKEN` | — | shared secret: HTTP requests need `x-studio-token` (or `Authorization: Bearer`, or `?token=`), WebSocket upgrades `?token=` |

CORS answers echo the allowed origin (never `*`), and the code-running endpoints accept `content-type:
application/json` only, so a cross-site page always needs a preflight. Python / MATLAB / C# / C++ clients send no
`Origin` header (or a local one) and are not affected.

In the browser the API's file writes (`Cam2D_Snapshot(file)`, `MakeProgram`, `Save`, CSV exports of scripts) go to the
session's in-memory files (src/api/memory_fs.ts): the calls succeed like in RoboDK (`Cam2D_Snapshot(file)` returns 1,
a script can `AddFile` what it saved) and File › Files saved by scripts downloads them.

## Python drop-in

`python/robodk/robolink.py` subclasses `RobolinkAPI` / `ItemAPI` from `_official.py`, which
`python/robodk/_generate_api.py` generates from RoboDK's own `robolink.py`: all 187 constants with RoboDK's values,
the exception classes (`TargetReachError`, `StoppedError`, `InputError`, `LicenseError`), the helpers and one wrapper
per official method with the official signature. The subclass converts what RoboDK returns as `robomath.Mat`
(Joints, SolveIK, InstructionList, InstructionListJoints, Spray_GetStats, ...), tuples and files. Poses travel by rows,
joint vectors as flat lists; lists/dicts of items and poses are encoded recursively; calls on an invalid item raise
`InputError` instead of running on the station. `from robolink import *` and `from robodk import *` work as in RoboDK.

Semantics that follow RoboDK: `SolveFK/SolveIK` on the flange with optional tool/reference, robot `PoseAbs` = base,
program `setPoseFrame/setPoseTool` insert instructions, `MoveC` with poses or joints, timed robot moves through the
station runtime (`Busy`, `WaitMove`, non-blocking moves; unreachable targets raise `TargetReachError`),
`RUNMODE_MAKE_ROBOTPROG` + `ProgramStart` + `Finish()` record robot moves and post them to files, `MakeProgram(folder)`
writes `<name>.<ext>` and returns `(ok, log, transfer_ok)` (unknown post = error), `AddFile` uses the importers,
`Save(path)` writes the station, `getParam('PATH_OPENSTATION' | 'FILE_OPENSTATION' | 'PATH_LIBRARY' | 'PATH_DESKTOP')`,
`Item.setParam` as command channel (`PostProcessor`, instruction dicts, `Display`), station `Command` queries
(`TrajectoryTime`, `SimulationTime`, ...), `ProjectPoints` (object coordinates, nearest surface hit along the point
normal, recalculated normals — equal to RoboDK's own projections to 0.01 mm, tests/api_project_points.test.ts),
`AttachClosest` on the object surface (mesh distance, 100 mm default tolerance), machining / curve-follow projects
(also the ones imported from `.rdk`; `status == 0` when the whole path is reachable).

## Station Python programs (contract I5)

RoboDK stations embed Python programs (`ITEM_TYPE_PROGRAM_PYTHON`, source in `params.pythonCode` from the importer).
They run when double-clicked (`RunCode`/`RunProgram` on the item) or when a program calls them
(`SprayOn(1)` → `sys.argv = ['SprayOn.py', '1']`; the calling program waits for the script, like RoboDK).

- **CPython runner** (server, headless, tests: src/api/runner_cpython.ts): a pool of persistent interpreters
  (`python/robodk/_script_worker.py`) runs one script each in a fresh `__main__` with the drop-in on the path and
  `STUDIO_URL` pointing at the server; concurrent scripts get separate workers (threaded multi-robot scripts work);
  `quit()`/`sys.exit(n)` are honoured and the script's threads are joined. `headlessGui` (`STUDIO_HEADLESS_GUI=1`)
  replaces tkinter by `python/robodk/_headless_gui` for unattended runs: `mainloop()` presses the main button with
  the default values, message boxes / file dialogs return their defaults, matplotlib uses Agg.
- **Pyodide runner** (browser without server: src/api/runner_pyodide.ts, `installPyodideRunner(RDK)`, installed by
  `src/main.ts` at start-up; Pyodide is downloaded when a station with Python programs is shown, and a run waiting for
  a script while it loads has its clock held, so the download is not simulation time). The drop-in is written into
  Pyodide's file system and talks to the page's `Robolink` in process. Two modes:
  - **cooperative** (browsers with WebAssembly JSPI: Chrome / Edge 137+): every script is a coroutine
    (`pyodide.ffi.run_sync`); `pause()` / `time.sleep()` wait in real time scaled by the simulation speed, blocking
    moves / `WaitMove` / `WaitFinished` wait for the realtime clock (`rdk.exec.asyncDepth`), and scripts run
    concurrently like RoboDK's script processes — RoboDK "background" scripts (`while True:` conveyor, laser-sensor
    and gripper simulations of Library 06.c, 06.d, 06.e, 06.f) run beside the programs without blocking the page.
    `print()` output and `tic()` / `toc()` are per script. A script whose calling run stops (Stop, Reset, another
    station) gets `StudioScriptStopped` at its next API call or pause and ends (history entry `stopped`).
  - **synchronous** (no JSPI): waits, blocking moves and `time.sleep()` advance the station clock (virtual time), one
    script at a time; a script that keeps the page thread busy longer than 30 s (an endless background loop) is
    stopped with a message instead of freezing the tab.
  The tree's "Run Python program" (RoboDK double-click) and `App.runProgram(pythonItem)` start a script item.
  `tests/e2e/pyodide.e2e.test.ts` runs real Pyodide in Chrome on Library stations 06.a (Gripper(1)/Gripper(0): fingers
  at ±6·arg), 01.a (SprayOn(1)/SprayOn(0): gun on, paint deposited, gun off at the end) and 07.a (DoWeld: the TCP passes
  the 6 hexagon vertices < 1 mm, ends at Home); `tests/e2e/pyodide_background.e2e.test.ts` plays 06.e StartSimulation
  (both parts carried over the conveyor and stored on table B), 06.d laser sensor (18 parts on PalletB), 06.c 2D camera
  (12 parts on PalletB) and 06.f delta with the page answering every second, then Stop ends the background loops.
- `App.runToEnd` ("⏭ End") waits for script runners: the clock stands still while a script answers (a declining runner
  resumes the run at the call), so the end state equals a run without the browser's runners (Library sweep).
- While the page is connected to a server, station scripts run on the server's CPython (POST /python/run).
- A call naming no station Python program (e.g. `SetRPM(70)`, meant for the controller) is ignored in simulation.
- A program blocked in a script call does not hold its robot: the script may move that robot (RoboDK behaviour).

## RoboDK binary protocol (official clients unmodified)

`src/api/rdk_protocol.ts` implements RoboDK's TCP protocol (handshake `RDK_API` or `CMD_START`, all 215 command strings of RoboDK 5.9.4's robolink.py: items, poses,
joints, FK/IK, programs and instructions, moves with the two-status blocking reply, collisions, parameters and
commands, spray, cameras, machining, calibration). Each command becomes a JSON-RPC request handled by the relay-aware
server, so the official clients see the same station as the drop-in:

```bash
STUDIO_RDK_PORT=20500 STUDIO_PORT=20600 npm run server
python -c "from robodk.robolink import *; RDK = Robolink(robodk_ip='127.0.0.1', port=20500); print(RDK.Item('', ITEM_TYPE_ROBOT).Joints())"
```

Use `robodk_ip='127.0.0.1'` (not the default `'localhost'`): when the official client cannot connect to `localhost` it
starts RoboDK itself.

Desktop-only commands (cursor projection, window docking, hover selection) answer with an explicit RoboDK error.

`Item.Update()` reports RoboDK's estimate: Set speed instructions are not applied there (RoboDK's own Update ignores
them; Library GT: 124 of 168 programs within 15 % in this mode, 99 with speeds applied), while `RunProgram` playback
applies them. Studio extensions: `RDK.Cam2D_Detect(cam, robot)` / its alias `RDK.Vision_Run(cam, robot)` (also in the
Python drop-in) run a camera's vision stack and refresh its pick targets; `camera.setParam('Settings')` returns
RoboDK's camera settings string (`FOV=.. FAR_LENGTH=.. NEAR_LENGTH=.. SIZE=WxH`) and applies one given as value. The
server installs the vision script runner (`VisionDetect("<camera>")` in programs) with the headless station's meshes.

`tests/e2e/server.e2e.test.ts` (built server, official package, `robodk_ip='127.0.0.1'`): `/health` reports the RoboDK
converter with RoboDK's own `robodk` package, `AddFile` of Library Example 01.b, `Update()` = RoboDK's
`[22, 7.4786 s, 1283.61 mm, 1]`, `RunProgram` until `Busy() == 0` ends at RoboDK's last joint row (< 0.01°), and
`MakeProgram` with RoboDK's native KUKA_KRC2 post equals the program RoboDK generated itself; a page opened with
`?server=` lists the server's RoboDK posts and its Export through RoboDK's Universal_Robots post equals the built-in one.

## Acceptance tests

`npx vitest run tests/api` runs everything below (CPython and C:/RoboDK are used when present; otherwise those blocks
are reported as skipped):

| Test file | What it proves |
|---|---|
| tests/api_python.test.ts | drop-in end to end over WebSocket: constants (AST diff with RoboDK's file empty), poses, Mat returns, FK/IK, PoseAbs, frames/tools, lists, invalid items, timed moves, offline programming, MakeProgram files, AddFile/Save/PATH_*, instruction formats, RunInstruction call, RunProgram/Busy, machining |
| tests/api_rdk_protocol.test.ts | the official `robodk` package (run isolated) drives the studio over the binary protocol |
| tests/api_station_scripts.test.ts | Library station Python programs reproduce their effects (spray, gripper, hexagon/polygon paths, pallets, sync of 3 robots, DH mirror, curve projection = RoboDK) |
| tests/api_pyodide.test.ts | browser runner bootstrap + drop-in (run by CPython in place of Pyodide), in-process transport, program call through StationRuntime |
| tests/api_server.test.ts | WS/TCP relay, per-method timeouts, host disconnect, MATLAB wire format, bridge latency / undo snapshots |
| tests/api_project_points.test.ts | ProjectPoints vs RoboDK's saved projections |
| tests/api_library_sweep.test.ts | opt-in (`T10_SWEEP=1`, ~40 min): all 170 Library Scripts/Macros headless; the 77 that pass must keep passing |

Current results: of the 54 Python programs embedded in the Library stations, 43 reproduce their RoboDK effect in the
tests, 6 more run (effect not asserted or a RoboDK plug-in missing), 5 are blocked (the runtime's robot queue for
Example-06.e, a delta-robot import issue for 06.f, a real UR controller for UR_ActivateMonitoring). Of the 170 Library
Scripts/Macros, 77 exit 0 headless, 39 are skipped on purpose (network, controllers, installers, processes), and most
of the rest need another station (calibration / machining projects, named items) or an input file, wait for a user or
sensor forever by design, or need third-party modules.
