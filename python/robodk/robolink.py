"""robolink drop-in for VerticalBot Studio (RoboDK API compatible subset).

    from robodk.robolink import *
    from robodk.robomath import *
    RDK = Robolink()                       # connects to ws://localhost:20500
    robot = RDK.Item('', ITEM_TYPE_ROBOT)
    robot.MoveJ([0, -90, 90, -90, -90, 0])
    prog = RDK.AddProgram('Hello', robot)
    prog.MoveJ(RDK.AddTarget('T1', robot.Parent(), robot))

Transport: WebSocket JSON-RPC. Uses the `websocket-client` package if installed, otherwise a
tiny built-in RFC 6455 client (no dependencies).
"""
import json
import os
import socket
import struct
import threading
import base64
import itertools

from .robomath import Mat

# ---- constants (RoboDK values) -----------------------------------------------------------
ITEM_TYPE_ANY = -1
ITEM_TYPE_STATION = 1
ITEM_TYPE_ROBOT = 2
ITEM_TYPE_FRAME = 3
ITEM_TYPE_TOOL = 4
ITEM_TYPE_OBJECT = 5
ITEM_TYPE_TARGET = 6
ITEM_TYPE_CURVE = 7
ITEM_TYPE_PROGRAM = 8
ITEM_TYPE_INSTRUCTION = 9
ITEM_TYPE_PROGRAM_PYTHON = 10
ITEM_TYPE_MACHINING = 11
ITEM_TYPE_FOLDER = 17
ITEM_TYPE_ROBOT_ARM = 18
ITEM_TYPE_CAMERA = 19
ITEM_TYPE_MOBILE_ROBOT = 100

INSTRUCTION_CALL_PROGRAM = 0
INSTRUCTION_INSERT_CODE = 1
INSTRUCTION_START_THREAD = 2
INSTRUCTION_COMMENT = 3
INSTRUCTION_SHOW_MESSAGE = 4

RUNMODE_SIMULATE = 1
RUNMODE_QUICKVALIDATE = 2
RUNMODE_MAKE_ROBOTPROG = 3
RUNMODE_RUN_ROBOT = 6

ROBOTCOM_READY = 2
ROBOTCOM_DISCONNECTED = 0
ROBOTCOM_CONNECTING = 1
ROBOTCOM_UNKNOWN = -1
ROBOTCOM_PROBLEMS = -2

ITEM_TYPE_MACHINING = 11
COLLISION_OFF = 0
COLLISION_ON = 1
PROGRAM_RUN_ON_SIMULATOR = 1
PROGRAM_RUN_ON_ROBOT = 2
CALIBRATE_TCP_BY_POINT = 0
CALIBRATE_TCP_BY_PLANE = 1
CALIBRATE_TCP_BY_LINE = 2
CALIBRATE_FRAME_3P_P1_ON_X = 0
CALIBRATE_FRAME_3P_P1_ORIGIN = 1
CALIBRATE_FRAME_6P = 2
CALIBRATE_TURNTABLE = 3
CALIBRATE_TURNTABLE_2X = 4
MAKE_ROBOT_1R, MAKE_ROBOT_1T, MAKE_ROBOT_2R, MAKE_ROBOT_2T, MAKE_ROBOT_3R, MAKE_ROBOT_3T, MAKE_ROBOT_4R, MAKE_ROBOT_4T, MAKE_ROBOT_6DOF, MAKE_ROBOT_7DOF, MAKE_ROBOT_SCARA, MAKE_ROBOT_1R1T = 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12
WINDOWSTATE_HIDDEN, WINDOWSTATE_SHOW, WINDOWSTATE_MINIMIZED, WINDOWSTATE_NORMAL, WINDOWSTATE_MAXIMIZED, WINDOWSTATE_FULLSCREEN, WINDOWSTATE_CINEMA, WINDOWSTATE_FULLSCREEN_CINEMA = -1, 0, 1, 2, 3, 4, 5, 6
FLAG_ROBODK_NONE, FLAG_ROBODK_ALL = 0, 0xFFFF
FLAG_ITEM_SELECTABLE, FLAG_ITEM_EDITABLE, FLAG_ITEM_DRAGALLOWED, FLAG_ITEM_DROPALLOWED, FLAG_ITEM_ENABLED, FLAG_ITEM_NONE, FLAG_ITEM_ALL = 1, 2, 4, 8, 32, 0, 111
SELECT_RESET, SELECT_NONE, SELECT_RECTANGLE, SELECT_ROTATE, SELECT_ZOOM, SELECT_PAN, SELECT_MOVE, SELECT_MOVE_SHIFT, SELECT_MOVE_CLEAR = -1, 0, 1, 2, 3, 4, 5, 6, 7
EVENT_SELECTION_TREE_CHANGED, EVENT_ITEM_MOVED, EVENT_REFERENCE_PICKED, EVENT_REFERENCE_RELEASED, EVENT_TOOL_MODIFIED, EVENT_CREATED_ISOCUBE, EVENT_SELECTION_3D_CHANGED, EVENT_3DVIEW_MOVED, EVENT_ROBOT_MOVED, EVENT_KEY = 1, 2, 3, 4, 5, 6, 7, 8, 9, 10
EVENT_ITEM_MOVED_POSE, EVENT_COLLISIONMAP_RESET, EVENT_COLLISIONMAP_TOO_LARGE, EVENT_CALIB_MEASUREMENT, EVENT_SELECTION3D_CLICK, EVENT_ITEM_CHANGED, EVENT_ITEM_RENAMED, EVENT_ITEM_VISIBILITY, EVENT_STATION_CHANGED = 11, 12, 13, 14, 15, 16, 17, 18, 19
INS_TYPE_INVALID, INS_TYPE_MOVE, INS_TYPE_MOVEC, INS_TYPE_CHANGESPEED, INS_TYPE_CHANGEFRAME, INS_TYPE_CHANGETOOL, INS_TYPE_CHANGEROBOT, INS_TYPE_PAUSE, INS_TYPE_EVENT, INS_TYPE_CODE, INS_TYPE_PRINT = -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
MOVE_TYPE_INVALID, MOVE_TYPE_JOINT, MOVE_TYPE_LINEAR, MOVE_TYPE_CIRCULAR = -1, 1, 2, 3
PROJECTION_NONE, PROJECTION_CLOSEST, PROJECTION_ALONG_NORMAL, PROJECTION_ALONG_NORMAL_RECALC, PROJECTION_CLOSEST_RECALC, PROJECTION_RECALC = 0, 1, 2, 3, 4, 5
JOINT_FORMAT = -1
TRACKER_TYPE_LASER = 0
RUNMODE_MAKE_ROBOTPROG_AND_UPLOAD, RUNMODE_MAKE_ROBOTPROG_AND_START, RUNMODE_TEACH = 4, 5, 7


class _MiniWebSocket:
    """Minimal RFC 6455 client (text frames only) used when websocket-client is missing."""

    def __init__(self, url, timeout=30):
        assert url.startswith("ws://"), "only ws:// is supported by the built-in client"
        rest = url[5:]
        hostport, _, path = rest.partition("/")
        host, _, port = hostport.partition(":")
        self.sock = socket.create_connection((host, int(port or 80)), timeout=timeout)
        key = base64.b64encode(os.urandom(16)).decode()
        req = (f"GET /{path} HTTP/1.1\r\nHost: {hostport}\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
               f"Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n")
        self.sock.sendall(req.encode())
        resp = b""
        while b"\r\n\r\n" not in resp:
            chunk = self.sock.recv(4096)
            if not chunk:
                raise ConnectionError("websocket handshake failed")
            resp += chunk
        if b" 101 " not in resp.split(b"\r\n")[0]:
            raise ConnectionError("websocket handshake rejected: " + resp.decode(errors="ignore")[:200])
        self.buf = resp.split(b"\r\n\r\n", 1)[1]

    def send(self, text):
        data = text.encode()
        header = bytearray([0x81])
        n = len(data)
        if n < 126:
            header.append(0x80 | n)
        elif n < 65536:
            header.append(0x80 | 126)
            header += struct.pack(">H", n)
        else:
            header.append(0x80 | 127)
            header += struct.pack(">Q", n)
        mask = os.urandom(4)
        header += mask
        masked = bytes(b ^ mask[i % 4] for i, b in enumerate(data))
        self.sock.sendall(bytes(header) + masked)

    def _read(self, n):
        while len(self.buf) < n:
            chunk = self.sock.recv(65536)
            if not chunk:
                raise ConnectionError("websocket closed")
            self.buf += chunk
        out, self.buf = self.buf[:n], self.buf[n:]
        return out

    def recv(self):
        while True:
            b1, b2 = self._read(2)
            opcode = b1 & 0x0F
            n = b2 & 0x7F
            if n == 126:
                n = struct.unpack(">H", self._read(2))[0]
            elif n == 127:
                n = struct.unpack(">Q", self._read(8))[0]
            mask = self._read(4) if b2 & 0x80 else None
            payload = self._read(n)
            if mask:
                payload = bytes(b ^ mask[i % 4] for i, b in enumerate(payload))
            if opcode == 0x1:
                return payload.decode()
            if opcode == 0x9:  # ping -> pong
                self.sock.sendall(bytes([0x8A, 0x80]) + os.urandom(4))
            if opcode == 0x8:
                raise ConnectionError("websocket closed by server")

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass


def _connect(url):
    try:
        import websocket  # type: ignore

        ws = websocket.create_connection(url, timeout=60)
        return ws
    except ImportError:
        return _MiniWebSocket(url)


class Robolink:
    """Connection to VerticalBot Studio (browser via relay, or headless server)."""

    def __init__(self, robodk_ip="localhost", port=20500, args=None, robodk_path=None, close_std_out=False, quit_on_close=False, com_object=None, skipstatus=False):
        url = os.environ.get("STUDIO_URL") or robodk_ip
        if not url.startswith("ws"):
            url = f"ws://{robodk_ip}:{port}"
        self.url = url
        self._ws = _connect(url)
        self._lock = threading.Lock()
        self._ids = itertools.count(1)
        self.run_mode = RUNMODE_SIMULATE

    # -- transport -------------------------------------------------------------
    def _call(self, method, params=None, target=None):
        req = {"id": next(self._ids), "method": method, "params": [_encode(p) for p in (params or [])], "target": target}
        with self._lock:
            self._ws.send(json.dumps(req))
            while True:
                res = json.loads(self._ws.recv())
                if res.get("id") == req["id"]:
                    break
        if res.get("error"):
            raise Exception(res["error"])
        return _decode(self, res.get("result"))

    def Disconnect(self):
        self._ws.close()

    Finish = Disconnect

    # -- station -----------------------------------------------------------------
    def Item(self, name, itemtype=ITEM_TYPE_ANY):
        return self._call("Item", [name, itemtype])

    def ItemList(self, filter=ITEM_TYPE_ANY, list_names=False):
        return self._call("ItemList", [filter, list_names])

    def ItemUserPick(self, message="Pick one item", itemtype=ITEM_TYPE_ANY):
        return self._call("ItemUserPick", [message, itemtype])

    def ActiveStation(self):
        return self._call("ActiveStation")

    def AddStation(self, name="New Station"):
        return self._call("AddStation", [name])

    def AddFrame(self, name, itemparent=None):
        return self._call("AddFrame", [name, itemparent])

    def AddFolder(self, name, itemparent=None):
        return self._call("AddFolder", [name, itemparent])

    def AddTarget(self, name, itemparent=None, itemrobot=None):
        return self._call("AddTarget", [name, itemparent, itemrobot])

    def AddProgram(self, name, itemrobot=None):
        return self._call("AddProgram", [name, itemrobot])

    def AddRobot(self, library_name, itemparent=None):
        return self._call("AddRobot", [library_name, itemparent])

    def AddMobileRobot(self, name):
        return self._call("AddMobileRobot", [name])

    def AddFile(self, filename, parent=None):
        return self._call("AddFile", [filename, parent])

    def AddShape(self, triangle_points, add_to=None, override_shapes=False, name="Shape"):
        return self._call("AddShape", [triangle_points, add_to, name])

    def AddCurve(self, curve_points, reference_object=None, add_to_ref=False, projection_type=0):
        return self._call("AddCurve", [curve_points, reference_object, add_to_ref])

    def AddPoints(self, points, reference_object=None, add_to_ref=False, projection_type=0):
        return self._call("AddPoints", [points, reference_object])

    def ShowMessage(self, message, popup=True):
        return self._call("ShowMessage", [message, popup])

    def Render(self, always_render=True):
        return self._call("Render", [always_render])

    def Update(self):
        return self._call("Update")

    def setRunMode(self, run_mode=RUNMODE_SIMULATE):
        self.run_mode = run_mode
        return self._call("setRunMode", [run_mode])

    # -- live robot drivers (server side): UR, ABB_RWS, KUKA_KVP --
    def Driver(self, action, **kwargs):
        """Talk to the studio server robot drivers (RoboDK "drivers"). action: list|connect|disconnect|state|moveJ|moveL|setDO|getDI|runScript|stop."""
        params = dict(kwargs)
        params["action"] = action
        return self._call("__driver__", [params])

    def RunMode(self):
        return self._call("RunMode")

    def setSimulationSpeed(self, speed):
        return self._call("setSimulationSpeed", [speed])

    def SimulationSpeed(self):
        return self._call("SimulationSpeed")

    def Save(self, filename, itemsave=None):
        return self._call("Save", [filename, itemsave])

    def getParam(self, param="PATH_OPENSTATION"):
        return self._call("getParam", [param])

    def setParam(self, param, value):
        return self._call("setParam", [param, value])

    def Command(self, cmd, value=""):
        return self._call("Command", [cmd, value])

    def Version(self):
        return self._call("Version")

    def License(self):
        return self._call("License")

    def Selection(self):
        return self._call("Selection")

    def setSelection(self, list_items):
        return self._call("setSelection", [list_items])

    def Collisions(self):
        return self._call("Collisions")

    def Delete(self, item_list):
        return self._call("Delete", [item_list])

    def Cam2D_Snapshot(self, file_save_img="", cam_handle=None):
        return self._call("Cam2D_Snapshot", [file_save_img, cam_handle])


    # -- extended surface (RoboDK parity) --
    def AddTargetJ(self, name, joints, itemparent=None, itemrobot=None):
        return self._call("AddTargetJ", [name, list(joints), itemparent, itemrobot])

    def BuildMechanism(self, type, list_obj=None, parameters=None, joints_build=None, joints_home=None, joints_senses=None, joints_lim_low=None, joints_lim_high=None, base=None, tool=None, name="New robot", robot=None):
        return self._call("BuildMechanism", [type, list_obj or [], list(parameters or []), list(joints_build or []), list(joints_home or []), list(joints_senses or []), list(joints_lim_low or []), list(joints_lim_high or []), base, tool, name, robot])

    def setRobotParams(self, robot, dhm, poseBase=None, poseTool=None):
        return self._call("setRobotParams", [robot, [list(r) for r in dhm], poseBase, poseTool])

    def AddMachiningProject(self, name="Curve follow settings", itemrobot=None):
        return self._call("AddMachiningProject", [name, itemrobot])

    def AddMillingProject(self, name="Milling settings", itemrobot=None):
        return self._call("AddMillingProject", [name, itemrobot])

    def Cam2D_Add(self, item_object, cam_params=""):
        return self._call("Cam2D_Add", [item_object, cam_params])

    def Cam2D_SetParams(self, params, cam_handle=None):
        return self._call("Cam2D_SetParams", [params, cam_handle])

    def Cam2D_Close(self, cam_handle=None):
        return self._call("Cam2D_Close", [cam_handle])

    def Calibrate_Reference(self, joints_points, method=CALIBRATE_FRAME_3P_P1_ORIGIN, use_joints=False, robot=None):
        return self._call("Calibrate_Reference", [[list(p) for p in joints_points], method, use_joints, robot])

    def CalibrateTool(self, poses_xyzwpr, input_format=JOINT_FORMAT, algorithm=CALIBRATE_TCP_BY_POINT, robot=None, tool=None):
        return tuple(self._call("CalibrateTool", [[list(p) for p in poses_xyzwpr], input_format, algorithm, robot, tool]))

    def Calibrate_Robot(self, measurements, robot=None, options=None):
        return self._call("Calibrate_Robot", [[list(m) for m in measurements], robot, options or {}])

    def LaserTracker_Measure(self, estimate=(0, 0, 0), search=False):
        return self._call("LaserTracker_Measure", [list(estimate), search])

    def MeasurePose(self, target=-1, time_avg=0, tip_xyz=(0, 0, 0)):
        return tuple(self._call("MeasurePose", [target, time_avg, list(tip_xyz)]))

    def StereoCamera_Measure(self):
        return tuple(self._call("StereoCamera_Measure"))

    def Popup_ISO9283_CubeProgram(self, robot=None, center=(1000, 0, 800), side=400, blocking=True):
        return self._call("Popup_ISO9283_CubeProgram", [robot, list(center), side, blocking])

    def BallbarProgram(self, robot=None, center=(1000, 0, 800), radius=150):
        return self._call("BallbarProgram", [robot, list(center), radius])

    def Collision_Line(self, p1, p2, ref=None):
        return tuple(self._call("Collision_Line", [list(p1), list(p2), ref]))

    def Collision_SetPair(self, item1, item2, id1=-1, id2=-1, collision_check=True):
        return self._call("Collision_SetPair", [item1, item2, id1, id2, collision_check])

    def Collision_SetPairList(self, list_items1, list_items2, list_id1=None, list_id2=None, list_check_state=True):
        return self._call("Collision_SetPairList", [list_items1, list_items2, list_id1 or [], list_id2 or [], list_check_state])

    def setCollisionActive(self, check_state=COLLISION_ON):
        return self._call("setCollisionActive", [check_state])

    def setCollisionActivePair(self, check_state, item1, item2, id1=-1, id2=-1):
        return self._call("setCollisionActivePair", [check_state, item1, item2, id1, id2])

    def CollisionItems(self):
        return self._call("CollisionItems")

    def CollisionPairs(self):
        return self._call("CollisionPairs")

    def Copy(self, item, copy_children=True):
        return self._call("Copy", [item, copy_children])

    def Paste(self, paste_to=None, paste_times=1):
        return self._call("Paste", [paste_to, paste_times])

    def Duplicate(self, item):
        return self._call("Duplicate", [item])

    def getOpenStations(self):
        return self._call("getOpenStations")

    def setActiveStation(self, station):
        return self._call("setActiveStation", [station])

    def CloseStation(self):
        return self._call("CloseStation")

    def CloseRoboDK(self):
        return self._call("CloseRoboDK")

    def getFlagsItem(self, item):
        return self._call("getFlagsItem", [item])

    def setFlagsItem(self, item, flags=FLAG_ITEM_ALL):
        return self._call("setFlagsItem", [item, flags])

    def getFlagsRoboDK(self):
        return self._call("getFlagsRoboDK")

    def setFlagsRoboDK(self, flags=FLAG_ROBODK_ALL):
        return self._call("setFlagsRoboDK", [flags])

    def HideRoboDK(self):
        return self._call("HideRoboDK")

    def ShowRoboDK(self):
        return self._call("ShowRoboDK")

    def setWindowState(self, windowstate=WINDOWSTATE_NORMAL):
        return self._call("setWindowState", [windowstate])

    def setInteractiveMode(self, mode_type=SELECT_MOVE, default_ref_flags=0, custom_objects=None, custom_ref_flags=None):
        return self._call("setInteractiveMode", [mode_type, default_ref_flags, custom_objects, custom_ref_flags])

    def setViewPose(self, pose):
        return self._call("setViewPose", [pose])

    def ViewPose(self, preset=-1):
        return self._call("ViewPose", [preset])

    def Joints(self, robot_item_list=None):
        return self._call("Joints", [robot_item_list])

    def setJoints(self, robot_item_list, joints_list):
        return self._call("setJoints", [robot_item_list, [list(j) for j in joints_list]])

    def setPoses(self, items, poses):
        return self._call("setPoses", [items, poses])

    def MergeItems(self, list_items):
        return self._call("MergeItems", [list_items])

    def RunCode(self, code, code_is_fcn_call=False):
        return self._call("RunCode", [code, code_is_fcn_call])

    def RunMessage(self, message, message_is_comment=False):
        return self._call("RunMessage", [message, message_is_comment])

    def RunProgram(self, fcn_param, wait_for_finished=False):
        return self._call("RunProgram", [fcn_param, wait_for_finished])

    def ShowSequence(self, matrix, display_type=0, timeout=-1):
        return self._call("ShowSequence", [[list(r) for r in matrix], display_type, timeout])

    def FilterTarget(self, pose, joints_approx=None, robot=None):
        return tuple(self._call("FilterTarget", [pose, joints_approx, robot]))

    def getParams(self):
        return self._call("getParams")

    def SimulationTime(self):
        return self._call("SimulationTime")

    def setSimulationTime(self, t):
        return self._call("setSimulationTime", [t])

    def Spray_Add(self, item_tool=None, item_object=None, params="", points=None, geometry=None):
        return self._call("Spray_Add", [item_tool, item_object, params, points, geometry])

    def Spray_SetState(self, state=1, id_spray=-1):
        return self._call("Spray_SetState", [state, id_spray])

    def Spray_GetStats(self, id_spray=-1):
        return tuple(self._call("Spray_GetStats", [id_spray]))

    def Spray_Clear(self, id_spray=-1):
        return self._call("Spray_Clear", [id_spray])

    def EventsListen(self):
        return self._call("EventsListen")

    def WaitForEvent(self, timeout=3600):
        return self._call("WaitForEvent", [timeout])

    def PluginLoad(self, plugin_name="", load=1):
        return self._call("PluginLoad", [plugin_name, load])

    def PluginCommand(self, plugin_name, plugin_command="", value=""):
        return self._call("PluginCommand", [plugin_name, plugin_command, value])

    def ProjectPoints(self, points, object_project=None, projection_type=PROJECTION_ALONG_NORMAL):
        return self._call("ProjectPoints", [[list(p) for p in points], object_project, projection_type])

    def IsInside(self, object_inside, object_parent):
        return self._call("IsInside", [object_inside, object_parent])

    def AddTool(self, tool_pose, tool_name="New TCP"):
        return self.Item("", ITEM_TYPE_ROBOT).AddTool(tool_pose, tool_name)

    def App(self, path, *args):
        """VerticalBot extension: call an application function (e.g. 'startWorld')."""
        return self._call("__app__", [path, *args])


class Item:
    """Proxy to a station item (robot, frame, target, program, tool, object...)."""

    def __init__(self, link, item_id, name="", itemtype=-1):
        self.link = link
        self.item = item_id
        self._name = name
        self._type = itemtype

    def __repr__(self):
        return f"Item({self._name!r}, {self._type})"

    def __eq__(self, other):
        return isinstance(other, Item) and other.item == self.item

    def __hash__(self):
        return hash(self.item)

    def _c(self, method, *params):
        return self.link._call(method, list(params), target=self.item)

    def RDK(self):
        return self.link

    def Valid(self, check_deleted=False):
        return self.item is not None and self._c("Valid")

    def Name(self):
        return self._c("Name")

    def setName(self, name):
        self._name = name
        return self._c("setName", name)

    def Type(self):
        return self._c("Type")

    def Parent(self):
        return self._c("Parent")

    def Childs(self):
        return self._c("Childs")

    def Delete(self):
        return self._c("Delete")

    def Visible(self):
        return self._c("Visible")

    def setVisible(self, visible, visible_frame=None):
        return self._c("setVisible", visible)

    def setParent(self, parent):
        return self._c("setParent", parent)

    def setParentStatic(self, parent):
        return self._c("setParentStatic", parent)

    def Pose(self):
        return self._c("Pose")

    def setPose(self, pose):
        return self._c("setPose", pose)

    def PoseAbs(self):
        return self._c("PoseAbs")

    def setPoseAbs(self, pose):
        return self._c("setPoseAbs", pose)

    def PoseTool(self):
        return self._c("PoseTool")

    def setPoseTool(self, tool):
        return self._c("setPoseTool", tool)

    def PoseFrame(self):
        return self._c("PoseFrame")

    def setPoseFrame(self, frame):
        return self._c("setPoseFrame", frame)

    def Joints(self):
        return self._c("Joints")

    def setJoints(self, joints):
        return self._c("setJoints", list(joints))

    def JointsHome(self):
        return self._c("JointsHome")

    def JointLimits(self):
        return self._c("JointLimits")

    def setJointLimits(self, lower, upper):
        return self._c("setJointLimits", list(lower), list(upper))

    def SolveFK(self, joints, tool=None, reference=None):
        return self._c("SolveFK", list(joints))

    def SolveIK(self, pose, joints_approx=None, tool=None, reference=None):
        return self._c("SolveIK", pose, list(joints_approx) if joints_approx else None)

    def SolveIK_All(self, pose, tool=None, reference=None):
        return self._c("SolveIK_All", pose)

    def Connect(self, robot_ip="", driver=None):
        """Connect the robot to a live controller through the server driver (UR / ABB_RWS / KUKA_KVP).
        The driver is inferred from the robot name/brand when not given."""
        name = self.Name()
        drv = driver or self.getParam("driver") or ("UR" if "UR" in name.upper() else "ABB_RWS" if "ABB" in name.upper() or "IRB" in name.upper() else "KUKA_KVP" if "KUKA" in name.upper() or "KR" in name.upper() else "UR")
        if not robot_ip:
            robot_ip = self.ConnectionParams()[0]
        try:
            self.link.Driver("connect", robot=name, driver=drv, ip=robot_ip)
            self._driver = drv
            return True
        except Exception as e:  # noqa
            self._last_error = str(e)
            return False

    def _live(self):
        return self.link.run_mode == RUNMODE_RUN_ROBOT and getattr(self, "_driver", None)

    def ConnectedState(self):
        if getattr(self, "_driver", None):
            try:
                st = self.link.Driver("state", robot=self.Name())
                return ROBOTCOM_READY if st.get("connected") else ROBOTCOM_DISCONNECTED
            except Exception:
                return ROBOTCOM_PROBLEMS
        return self._c("ConnectedState")

    def setSpeed(self, speed_linear, speed_joints=-1, accel_linear=-1, accel_joints=-1):
        return self._c("setSpeed", speed_linear, speed_joints, accel_linear, accel_joints)

    def setRounding(self, rounding_mm):
        return self._c("setRounding", rounding_mm)

    setZoneData = setRounding

    def MoveJ(self, target, blocking=True):
        res = self._c("MoveJ", target, blocking)
        if self._live() and self.Type() == ITEM_TYPE_ROBOT:
            self.link.Driver("moveJ", robot=self.Name(), joints=self.Joints())
        return res

    def MoveL(self, target, blocking=True):
        res = self._c("MoveL", target, blocking)
        if self._live() and self.Type() == ITEM_TYPE_ROBOT:
            pose = self._c("PoseAbs")
            rows = pose.rows
            cols = [rows[r][c] for c in range(4) for r in range(4)]
            self.link.Driver("moveL", robot=self.Name(), pose=cols)
        return res

    def MoveC(self, target1, target2, blocking=True):
        return self._c("MoveC", target1, target2)

    def Pause(self, time_ms=-1):
        return self._c("Pause", time_ms)

    def setDO(self, io_var, io_value):
        res = self._c("setDO", str(io_var), io_value)
        if self._live() and self.Type() == ITEM_TYPE_ROBOT:
            self.link.Driver("setDO", robot=self.Name(), io=str(io_var), value=io_value)
        return res

    def waitDI(self, io_var, io_value, timeout_ms=-1):
        return self._c("waitDI", str(io_var), io_value, timeout_ms)

    def RunInstruction(self, code, run_type=INSTRUCTION_CALL_PROGRAM):
        return self._c("RunInstruction", code, run_type)

    RunCodeCustom = RunInstruction

    def ShowInstructions(self, show=True):
        return self._c("ShowInstructions", show)

    def ShowTargets(self, show=True):
        return self._c("ShowTargets", show)

    def InstructionCount(self):
        return self._c("InstructionCount")

    def Instruction(self, ins_id=-1):
        r = self._c("Instruction", ins_id)
        return r["name"], r["type"], r["moveType"], r["isJointTarget"], r["pose"], r["joints"]

    def InstructionList(self):
        return self._c("InstructionList")

    def InstructionDelete(self, ins_id=0):
        return self._c("InstructionDelete", ins_id)

    def setRobot(self, robot):
        return self._c("setRobot", robot)

    def getLink(self, type_linked=ITEM_TYPE_ROBOT):
        return self._c("getLink", type_linked)

    def AddTool(self, tool_pose, tool_name="New TCP"):
        return self._c("AddTool", tool_pose, tool_name)

    def AddFrame(self, name):
        return self._c("AddFrame", name)

    def AddTarget(self, name):
        return self._c("AddTarget", name)

    def setAsCartesianTarget(self):
        return self._c("setAsCartesianTarget")

    def setAsJointTarget(self):
        return self._c("setAsJointTarget")

    def isJointTarget(self):
        return self._c("isJointTarget")

    def Update(self, check_collisions=0, timeout_sec=3600, mm_step=-1, deg_step=-1):
        return tuple(self._c("Update"))

    def RunProgram(self, prog_parameters=None):
        return self._c("RunProgram")

    RunCode = RunProgram

    def MakeProgram(self, folder_path="", run_mode=RUNMODE_MAKE_ROBOTPROG, post=None):
        return tuple(self._c("MakeProgram", folder_path, post))

    def setParam(self, param, value=""):
        return self._c("setParam", param, value)

    def getParam(self, param):
        return self._c("getParam", param)

    def setColor(self, tocolor, fromcolor=None, tolerance=0.1):
        return self._c("setColor", tocolor)

    Recolor = setColor

    def setGeometryPose(self, pose):
        return self._c("setGeometryPose", pose)

    def Busy(self):
        return self._c("Busy")

    def Stop(self):
        return self._c("Stop")

    def WaitMove(self, timeout=360000):
        return self._c("WaitMove")

    def DOF(self):
        return self._c("DOF")


    # -- extended surface (RoboDK parity) --
    def AttachClosest(self, keyword="", tolerance_mm=500, list_objects=None):
        return self._c("AttachClosest", tolerance_mm)

    def DetachClosest(self, parent=None):
        return self._c("DetachClosest", parent)

    def DetachAll(self, parent=None):
        return self._c("DetachAll", parent)

    def Collision(self, item2):
        return self._c("Collision", item2)

    def Copy(self, copy_children=True):
        return self._c("Copy")

    def Paste(self):
        return self._c("Paste")

    def GeometryPose(self):
        return self._c("GeometryPose")

    def InstructionListJoints(self, mm_step=10, deg_step=5, save_to_file=None, collision_check=0, flags=0, time_step=0.1):
        return tuple(self._c("InstructionListJoints", mm_step, deg_step, save_to_file or "", collision_check, flags, time_step))

    def InstructionSelect(self, ins_id=-1):
        return self._c("InstructionSelect", ins_id)

    def setInstruction(self, ins_id, name, instype, movetype, isjointtarget, target, joints):
        return self._c("setInstruction", ins_id, name, instype, movetype, isjointtarget, target, list(joints) if joints else None)

    def JointsConfig(self, joints):
        return self._c("JointsConfig", list(joints))

    def setJointsHome(self, joints):
        return self._c("setJointsHome", list(joints))

    def setAccuracyActive(self, accurate=1):
        return self._c("setAccuracyActive", bool(accurate))

    def AccuracyActive(self):
        return self._c("AccuracyActive")

    def setAcceleration(self, accel_linear):
        return self._c("setAcceleration", accel_linear)

    def setAccelerationJoints(self, accel_joints):
        return self._c("setAccelerationJoints", accel_joints)

    def setSpeedJoints(self, speed_joints):
        return self._c("setSpeedJoints", speed_joints)

    def setLink(self, item):
        return self._c("setLink", item)

    def ObjectLink(self, link_id=0):
        return self._c("ObjectLink", link_id)

    def Save(self, filename):
        data = self._c("Save", filename)
        if filename:
            with open(filename, "w", encoding="utf-8") as f:
                f.write(data)
        return data

    def setMachiningParameters(self, ncfile="", part=None, params=""):
        return tuple(self._c("setMachiningParameters", ncfile, part, params))

    def MachiningParameters(self):
        return self._c("MachiningParameters")

    def FilterTarget(self, pose, joints_approx=None):
        return tuple(self._c("FilterTarget", pose, joints_approx))

    def FilterProgram(self, filestr=""):
        return tuple(self._c("FilterProgram", filestr))

    def setRunType(self, program_run_type):
        return self._c("setRunType", program_run_type)

    def RunType(self):
        return self._c("RunType")

    def WaitFinished(self, timeout=3600):
        return self._c("WaitFinished", timeout)

    def MoveJ_Test(self, j1, j2, minstep_deg=-1):
        return self._c("MoveJ_Test", list(j1), list(j2), minstep_deg if minstep_deg > 0 else 1)

    def MoveL_Test(self, j1, pose, minstep_mm=-1):
        return self._c("MoveL_Test", list(j1), pose, minstep_mm if minstep_mm > 0 else 1)

    def SearchL(self, target, blocking=True):
        return self._c("SearchL", target, blocking)

    def Scale(self, scale):
        return self._c("Scale", scale)

    def setColorShape(self, color, shape_id=0):
        return self._c("setColorShape", color, shape_id)

    def setColorCurve(self, color, curve_id=-1):
        return self._c("setColorCurve", color, curve_id)

    def Color(self):
        return self._c("Color")

    def setValue(self, varname, value=""):
        return self._c("setValue", varname, value)

    def Value(self, varname=""):
        return self._c("Value", varname)

    def setAO(self, io_var, io_value):
        return self._c("setAO", str(io_var), io_value)

    def getDI(self, io_var):
        return self._c("getDI", str(io_var))

    def getAI(self, io_var):
        return self._c("getAI", str(io_var))

    def customInstruction(self, name, path_run, path_icon="", blocking=1, cmd_run_on_robot=""):
        return self._c("customInstruction", name, path_run, path_icon, blocking, cmd_run_on_robot)

    def addMoveJ(self, itemtarget):
        return self._c("addMoveJ", itemtarget)

    def addMoveL(self, itemtarget):
        return self._c("addMoveL", itemtarget)

    def ConnectSafe(self, robot_ip="", max_attempts=5, wait_connection=4, callback_abort=None):
        return self._c("ConnectSafe", robot_ip, max_attempts, wait_connection)

    def ConnectionParams(self):
        return tuple(self._c("ConnectionParams"))

    def setConnectionParams(self, robot_ip, port=0, remote_path="", ftp_user="", ftp_pass=""):
        return self._c("setConnectionParams", robot_ip, port, remote_path, ftp_user, ftp_pass)

    def Disconnect(self):
        return self._c("Disconnect")

    def JointPoses(self, joints=None):
        return self._c("JointPoses", list(joints) if joints else None)

    def setRobotParams(self, dhm, poseBase=None, poseTool=None):
        return self._c("setRobotParams", [list(r) for r in dhm], poseBase, poseTool)

    def RobotParams(self):
        return self._c("RobotParams")


# ---- (de)serialisation ------------------------------------------------------------
def _encode(v):
    if isinstance(v, Item):
        return {"$item": v.item}
    if isinstance(v, Mat):
        return {"$pose": v.tolist()}
    if isinstance(v, (list, tuple)):
        return [_encode(x) for x in v]
    if isinstance(v, dict):
        return {k: _encode(x) for k, x in v.items()}
    return v


def _decode(link, v):
    if isinstance(v, dict):
        if "$item" in v:
            return Item(link, v["$item"], v.get("name", ""), v.get("type", -1))
        if "$pose" in v:
            return Mat(v["$pose"])
        return {k: _decode(link, x) for k, x in v.items()}
    if isinstance(v, list):
        return [_decode(link, x) for x in v]
    return v
