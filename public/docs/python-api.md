# Python API (RoboDK drop-in)

`studio/python/robodk` is a `robodk` package with `robolink` and `robomath` modules that talk to a running
VerticalBot Studio server over WebSocket. Put `studio/python` first on `PYTHONPATH` and existing RoboDK scripts
run against the studio (in the browser when a tab is connected, headless otherwise).

```bash
cd studio && npm run server                         # ws://localhost:20500
# open http://localhost:5173/?server=ws://localhost:20500 to see API calls live (optional)
PYTHONPATH=studio/python python my_robodk_script.py
```

Environment: `STUDIO_URL=ws://host:20500` overrides the default (`Robolink(robodk_ip, port)` arguments are honoured).
No third-party dependency is required (`websocket-client` is used when installed).

## Example

```python
from robodk.robolink import *
from robodk.robomath import *

RDK = Robolink()
robot = RDK.Item('', ITEM_TYPE_ROBOT) or RDK.AddRobot('UR10e')
frame = RDK.AddFrame('Work frame')
frame.setPose(transl(600, 0, 300))
robot.setPoseFrame(frame)
tool = robot.AddTool(transl(0, 0, 120), 'Gripper')
robot.setPoseTool(tool)

prog = RDK.AddProgram('Pick', robot)
prog.setSpeed(300, 60)
home = RDK.AddTarget('Home', frame, robot); home.setAsJointTarget()
prog.MoveJ(home)
p = robot.Pose()
for i, (dx, dy) in enumerate([(0, 0), (150, 0), (150, 150), (0, 150)]):
    t = RDK.AddTarget('P%d' % i, frame, robot)
    t.setPose(p * transl(dx, dy, 0))
    prog.MoveL(t)
prog.setDO('Gripper', 1)
prog.RunInstruction('Done', INSTRUCTION_SHOW_MESSAGE)

valid, seconds, mm, ratio, msg = prog.Update()
ok, code, filename = prog.MakeProgram('', post='ABB_RAPID_IRC5')
print(code)
```

## Studio extensions

- `RDK.AddRobot(name_or_library_id)` — add a robot from the built-in library (`UR5e`, `KUKA_KR6_R900`, …).
- `RDK.AddMobileRobot(name)` — add a mobile robot item.
- `RDK.App(path, *args)` — call an application function, e.g. `RDK.App('startWorld')`, `RDK.App('pauseWorld')`,
  `RDK.App('fleetManager', fleet_item)` (returns the manager; use `RDK.App('fleets.get', id)`), when a browser
  session is attached.
- Item type constants above 100 identify studio items: `ITEM_TYPE_MOBILE_ROBOT = 100`, components 101,
  fleets 102, maps 103, fields 104, missions 105.

## Exporting a real RoboDK station

Run `python/rdk_export.py` from RoboDK (Tools › Run Script). It writes `<station>.vbstation` plus STL meshes;
open the file in the studio. Robots are reconstructed from DH tables when available, otherwise matched to the
library by name; programs are rebuilt from the instruction list.

## Protocol

JSON-RPC over WebSocket. Request: `{"id": 1, "method": "MoveJ", "target": "<item id>", "params": [...]}`.
Items serialise as `{"$item": id, "name": ..., "type": ...}`, poses as `{"$pose": [[r0],[r1],[r2],[r3]]}` (row-major).
Responses: `{"id": 1, "result": ...}` or `{"id": 1, "error": "message"}`. See `src/api/rpc.ts`.

## Running on the real robot (drivers)

The studio server hosts robot drivers (like RoboDK's drivers): `UR` (URScript over TCP 30002 + real-time joint
feedback on 30003), `ABB_RWS` (Robot Web Services over HTTP), `KUKA_KVP` (KUKAVARPROXY variables), `ROS2` (rosbridge + ros2_control joint trajectory controller, for any robot with a ROS 2 driver). From Python:

```python
RDK.setRunMode(RUNMODE_RUN_ROBOT)
robot.Connect('192.168.1.10', driver='UR')   # driver inferred from the robot name when omitted
robot.MoveJ(home)                             # simulated first, then sent to the controller and awaited
robot.setDO('1', True)
print(RDK.Driver('state', robot=robot.Name()))
```

Other languages: the server also listens on `port + 1` (20501) with newline-delimited JSON, used by the C#
(`clients/csharp`), C++ (`clients/cpp`) and MATLAB (`clients/matlab`) clients.
