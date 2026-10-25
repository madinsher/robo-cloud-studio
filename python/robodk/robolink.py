"""robolink drop-in for VerticalBot Studio (RoboDK API compatible).

    from robodk.robolink import *          # or the legacy: from robolink import *
    from robodk.robomath import *          # or the legacy: from robodk import *
    RDK = Robolink()                       # connects to ws://localhost:20500 (STUDIO_URL overrides)
    robot = RDK.Item('', ITEM_TYPE_ROBOT)
    robot.MoveJ([0, -90, 90, -90, -90, 0])

Every official constant, exception and method signature comes from _official.py, generated from RoboDK's own
robolink.py by _generate_api.py, so values (ROBOTCOM_*, MAKE_ROBOT_*, SPRAY_*, ...) match RoboDK exactly.
The methods below override the generated JSON-RPC wrappers where RoboDK returns robomath.Mat, tuples or files.

Transport: JSON-RPC over WebSocket (websocket-client if installed, else a built-in RFC 6455 client), or an
in-process transport (set robodk.robolink._TRANSPORT = callable(request_json) -> response_json, used by the
browser's Pyodide runner). Poses travel as {"$pose": rows}, joint vectors as flat lists, items as {"$item": id}.
"""
import json
import os
import sys
import time
import socket
import struct
import threading
import base64
import itertools
import math

from . import robomath
from .robomath import Mat
from ._official import *  # noqa: F401,F403  (official constants, enums, exceptions, helpers)
from ._official import RobolinkAPI, ItemAPI, OFFICIAL_CONSTANTS, OFFICIAL_ROBOLINK_METHODS, OFFICIAL_ITEM_METHODS  # noqa: F401
from ._official import TargetReachError, StoppedError, InputError, LicenseError  # noqa: F401

# ---- studio extensions (not in RoboDK) -------------------------------------------------------
ITEM_TYPE_ANY = -1
ITEM_TYPE_MOBILE_ROBOT = 100
TRACKER_TYPE_LASER = 0

#: in-process transport hook (Pyodide): callable(str request JSON) -> str response JSON
_TRANSPORT = None
#: default URL when Robolink() is called without arguments
DEFAULT_URL = os.environ.get('STUDIO_URL', '')

_NOTARGET = object()
_LONG = 3600 * 24 * 7
# per-method client timeouts (s); everything else uses Robolink.TIMEOUT
_TIMEOUTS = {
    'Update': 3600, 'InstructionListJoints': 3600, 'MakeProgram': 600, 'setMachiningParameters': 3600, 'setMillingParameters': 3600,
    'AddFile': 600, 'Save': 600, 'ProjectPoints': 600, 'Popup_ISO9283_CubeProgram': 3600, 'CalibrateTool': 3600, 'MoveJ_Test': 3600,
    'MoveL_Test': 3600, 'MoveJ_Test_Blend': 3600, 'Collisions': 600, 'BuildMechanism': 600, 'Command': 3600, 'PluginCommand': 3600,
    'Cam2D_Snapshot': 600, 'Finish': 600, 'RunCode': 60, 'App': 3600,
}


# ---- transport --------------------------------------------------------------------------------
class _MiniWebSocket:
    """Minimal RFC 6455 client (text frames only) used when websocket-client is missing."""

    def __init__(self, url, timeout=30):
        assert url.startswith("ws://"), "only ws:// is supported by the built-in client"
        rest = url[5:]
        hostport, _, path = rest.partition("/")
        host, _, port = hostport.partition(":")
        self.sock = socket.create_connection((host, int(port or 80)), timeout=timeout)
        try:
            self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except OSError:
            pass
        key = base64.b64encode(os.urandom(16)).decode()
        req = ("GET /%s HTTP/1.1\r\nHost: %s\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
               "Sec-WebSocket-Key: %s\r\nSec-WebSocket-Version: 13\r\n\r\n") % (path, hostport, key)
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

    def settimeout(self, t):
        self.sock.settimeout(t)

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
        if n > 4096:
            m = (mask * (n // 4 + 1))[:n]
            masked = (int.from_bytes(data, 'big') ^ int.from_bytes(m, 'big')).to_bytes(n, 'big')
        else:
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
        parts = []
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
            if opcode in (0x1, 0x0):
                parts.append(payload)
                if b1 & 0x80:
                    return b"".join(parts).decode()
            elif opcode == 0x9:  # ping -> pong
                self.sock.sendall(bytes([0x8A, 0x80]) + os.urandom(4))
            elif opcode == 0x8:
                raise ConnectionError("websocket closed by server")

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass


class _InProcess:
    """Transport through _TRANSPORT (Pyodide in the browser: synchronous calls into the studio)."""

    def __init__(self, fn):
        self.fn = fn
        self.pending = None

    def settimeout(self, t):
        pass

    def send(self, text):
        self.pending = self.fn(text)

    def recv(self):
        out, self.pending = self.pending, None
        return out

    def close(self):
        pass


def _connect(url, timeout=10):
    if _TRANSPORT is not None:
        return _InProcess(_TRANSPORT)
    try:
        import websocket  # type: ignore
        ws = websocket.create_connection(url, timeout=timeout)
        if not hasattr(ws, 'settimeout'):
            ws.settimeout = lambda t, _ws=ws: _ws.sock.settimeout(t)
        return ws
    except ImportError:
        return _MiniWebSocket(url, timeout)


def _raise(msg):
    """Map server errors to the official exception classes."""
    text = str(msg)
    for prefix, cls in (('TargetReachError', TargetReachError), ('StoppedError', StoppedError), ('InputError', InputError), ('LicenseError', LicenseError)):
        if text.startswith(prefix):
            raise cls(text[len(prefix):].lstrip(': '))
    if 'not reachable' in text or 'unreachable' in text.lower():
        raise TargetReachError(text)
    raise Exception(text)


# ---- (de)serialisation ------------------------------------------------------------------------
def _num(x):
    if isinstance(x, float) and (math.isnan(x) or math.isinf(x)):
        return None
    return x


def _encode(v):
    if isinstance(v, Item):
        return {"$item": v.item if v.item else None}
    if isinstance(v, Mat):
        r, c = v.size()
        if r == 4 and c == 4:
            return {"$pose": [[_num(float(x)) for x in row] for row in v.rows]}
        if c == 1:
            return [_num(float(row[0])) for row in v.rows]
        if r == 1:
            return [_num(float(x)) for x in v.rows[0]]
        if r == 0 or c == 0:
            return []
        return {"$mat": [[_num(float(x)) for x in row] for row in v.rows]}
    if isinstance(v, bool) or v is None or isinstance(v, str):
        return v
    if isinstance(v, (bytes, bytearray)):
        return {"$bytes": base64.b64encode(bytes(v)).decode()}
    if isinstance(v, (list, tuple)):
        return [_encode(x) for x in v]
    if isinstance(v, dict):
        return {str(k): _encode(x) for k, x in v.items()}
    if isinstance(v, float):
        return _num(v)
    if isinstance(v, int):
        return v
    if hasattr(v, 'tolist'):  # numpy arrays / scalars
        return _encode(v.tolist())
    if hasattr(v, '__float__'):
        return _num(float(v))
    return v


def _decode(link, v):
    if isinstance(v, dict):
        if "$item" in v:
            return Item(link, v["$item"] or 0, v.get("type", -1), v.get("name", ""))
        if "$pose" in v:
            return Mat([list(map(float, r)) for r in v["$pose"]])
        if "$mat" in v:
            rows = v["$mat"]
            return Mat([list(map(float, r)) for r in rows]) if rows and rows[0] else Mat(0, 0)
        if "$bytes" in v:
            return base64.b64decode(v["$bytes"])
        return {k: _decode(link, x) for k, x in v.items()}
    if isinstance(v, list):
        return [_decode(link, x) for x in v]
    return v


def _jmat(v):
    """Joint vector / list -> robomath.Mat column (RoboDK returns Mat for Joints, SolveIK, ...)."""
    if isinstance(v, Mat):
        return v
    if v is None:
        return Mat(0, 0)
    v = list(v)
    if not v:
        return Mat([])
    return Mat([float(x) for x in v])


def _cols(rows):
    """list of row vectors -> Mat whose columns are the vectors (RoboDK matrix convention)."""
    if not rows:
        return Mat(0, 0)
    return Mat([[float(x) for x in r] for r in rows]).tr()


def _points(points):
    """3xN / 6xN Mat (columns) or list of points -> list of point lists."""
    if isinstance(points, Mat):
        r, c = points.size()
        if r in (3, 6) or (r < c and r <= 6):
            return [list(col) for col in points.tr().rows]
        return [list(row) for row in points.rows]
    out = []
    for p in points:
        out.append(list(p.list()) if isinstance(p, Mat) else list(p))
    return out


# ---- Robolink ---------------------------------------------------------------------------------
class Robolink(RobolinkAPI):
    """Connection to VerticalBot Studio (browser via relay, or headless server). Same surface as RoboDK's Robolink."""

    IP = 'localhost'
    PORT_START = 20500
    PORT_END = 20500
    PORT = -1
    TIMEOUT = 120
    BUILD = 30000
    ARGUMENTS = []
    APPLICATION_DIR = ''
    NEW_INSTANCE = None
    DEBUG = False
    CAMERA_AS_ITEM = True
    LAST_STATUS_MESSAGE = ''

    def __init__(self, robodk_ip='localhost', port=None, args=[], robodk_path=None, close_std_out=False, quit_on_close=False, com_object=None, skipstatus=False):
        if type(args) is str:
            args = [args] if args else []
        self.ARGUMENTS = list(args)
        self.CLOSE_STD_OUT = close_std_out
        self.QUIT_ON_CLOSE = quit_on_close
        self.APPLICATION_DIR = robodk_path or ''
        self._lock = threading.RLock()
        self._ids = itertools.count(1)
        self.run_mode = RUNMODE_SIMULATE
        env_url = os.environ.get('STUDIO_URL', '') or DEFAULT_URL
        if robodk_ip and str(robodk_ip).startswith('ws'):
            url = str(robodk_ip)
        elif port is None and robodk_ip in ('localhost', '127.0.0.1', '', None) and env_url:
            url = env_url
        else:
            p = port if port is not None else int(os.environ.get('ROBODK_API_PORT', os.environ.get('STUDIO_PORT', 20500)))
            url = 'ws://%s:%i' % (robodk_ip or 'localhost', p)
        self.url = url
        self.IP = robodk_ip
        try:
            self.PORT = int(url.rsplit(':', 1)[1].split('/')[0])
        except Exception:
            self.PORT = -1
        self.COM = None
        self.Connect()

    # -- transport --------------------------------------------------------------------------------
    def _call(self, method, params=None, target=_NOTARGET, timeout=None):
        req = {"id": next(self._ids), "method": method, "params": [_encode(p) for p in (params or [])]}
        if target is not _NOTARGET:
            req["target"] = target if target else None
        with self._lock:
            if self.COM is None:
                raise Exception('RoboDK API is not connected (call Connect or NewLink)')
            t = timeout if timeout is not None else _TIMEOUTS.get(method, self.TIMEOUT)
            try:
                self.COM.settimeout(t)
            except Exception:
                pass
            self.COM.send(json.dumps(req, allow_nan=False))
            while True:
                raw = self.COM.recv()
                res = json.loads(raw)
                if res.get("id") == req["id"]:
                    break
        if res.get("error") is not None:
            self.LAST_STATUS_MESSAGE = str(res["error"])
            _raise(res["error"])
        return _decode(self, res.get("result"))

    def _rpc(self, method, params):
        return self._call(method, params)

    def Connect(self):
        """Connect (or reconnect) to the studio. Returns 1 on success."""
        try:
            self.COM = _connect(self.url, 10)
        except Exception as e:
            raise Exception('Unable to connect to VerticalBot Studio at %s (%s). Start the studio server (npm run server) or open the studio with ?server=...' % (self.url, e))
        return 1

    def NewLink(self):
        """Open a new independent connection (RoboDK: needed for each thread that drives robots)."""
        try:
            self.Disconnect()
        except Exception:
            pass
        self.Connect()

    def isNewInstance(self):
        return False

    def Disconnect(self):
        if self.COM is not None:
            self.COM.close()
        self.COM = None

    def Finish(self):
        """Generates pending offline programs (RUNMODE_MAKE_ROBOTPROG) and disconnects."""
        try:
            if self.COM is not None:
                self._call('Finish')
        finally:
            self.Disconnect()

    def __del__(self):
        try:
            self.Disconnect()
        except Exception:
            pass

    # -- station ----------------------------------------------------------------------------------
    def Item(self, name, itemtype=None):
        if type(name) is not str:
            raise Exception('Invalid name: provide a name as a string. Item names are visible in the RoboDK tree.')
        return self._call("Item", [name, ITEM_TYPE_ANY if itemtype is None else itemtype])

    def ItemList(self, filter=None, list_names=False):
        return self._call("ItemList", [ITEM_TYPE_ANY if filter is None else filter, list_names])

    def ItemUserPick(self, message='Pick one item', itemtype_or_list=None):
        return self._call("ItemUserPick", [message, ITEM_TYPE_ANY if itemtype_or_list is None else itemtype_or_list])

    def getParam(self, param='PATH_OPENSTATION', str_type=True):
        value = self._call("getParam", [param, str_type])
        if not str_type:
            if value is None:
                return b''
            return value if isinstance(value, bytes) else str(value).encode()
        if value is None:
            return None
        if isinstance(value, bool):
            return float(value)
        if isinstance(value, (int, float)):
            return float(value)
        if isinstance(value, (list, dict)):
            return value
        value = str(value)
        if value.replace('.', '', 1).isnumeric():
            return float(value)
        return value

    def getParams(self):
        return [list(p) for p in self._call("getParams")]

    def setParam(self, param, value):
        # RoboDK stores station parameters as text (str(value)); bytes as binary data
        if isinstance(value, dict):
            value = json.dumps(value)
        elif isinstance(value, Item):
            value = str(value.item)
        elif not isinstance(value, (str, bytes, bytearray)):
            value = str(value).replace(chr(10), ' ')
        return self._call("setParam", [param, value])

    def Command(self, cmd, value='', skip_result=False):
        if isinstance(value, dict):
            value = json.dumps(value)
        elif isinstance(value, Item):
            value = str(value.item)
        elif not isinstance(value, Mat):
            value = str(value)
        return self._call("Command", [cmd, value, skip_result])

    def setRunMode(self, run_mode=1):
        self.run_mode = run_mode
        return self._call("setRunMode", [run_mode])

    def Joints(self, robot_item_list):
        return [_jmat(j) for j in self._call("Joints", [robot_item_list])]

    def setJoints(self, robot_item_list, joints_list):
        return self._call("setJoints", [robot_item_list, [list(j.list()) if isinstance(j, Mat) else list(j) for j in joints_list]])

    def setPoses(self, items, poses):
        if len(items) != len(poses):
            raise Exception('The number of items must match the number of poses')
        return self._call("setPoses", [items, poses])

    def setPosesAbs(self, items, poses):
        if len(items) != len(poses):
            raise Exception('The number of items must match the number of poses')
        return self._call("setPosesAbs", [items, poses])

    def AddShape(self, triangle_points, add_to=0, override_shapes=False):
        if isinstance(triangle_points, list) and triangle_points and isinstance(triangle_points[0], Mat):
            shapes = [_points(m) for m in triangle_points if isinstance(m, Mat)]
            return self._call("AddShape", [shapes, add_to, override_shapes])
        return self._call("AddShape", [_points(triangle_points), add_to, override_shapes])

    def AddCurve(self, curve_points, reference_object=0, add_to_ref=False, projection_type=PROJECTION_ALONG_NORMAL_RECALC):
        if isinstance(curve_points, list) and curve_points and (isinstance(curve_points[0], Mat) or (isinstance(curve_points[0], list) and curve_points[0] and isinstance(curve_points[0][0], list))):
            return self._call("AddCurve", [[_points(c) for c in curve_points], reference_object, add_to_ref, projection_type, True])
        return self._call("AddCurve", [_points(curve_points), reference_object, add_to_ref, projection_type])

    def AddPoints(self, points, reference_object=0, add_to_ref=False, projection_type=PROJECTION_ALONG_NORMAL_RECALC):
        return self._call("AddPoints", [_points(points), reference_object, add_to_ref, projection_type])

    def ProjectPoints(self, points, object_project, projection_type=PROJECTION_ALONG_NORMAL_RECALC, timeout=30):
        islist = not isinstance(points, Mat)
        res = self._call("ProjectPoints", [_points(points), object_project, projection_type], timeout=max(timeout, 60))
        return res if islist else _cols(res)

    def Save(self, filename, itemsave=0):
        self._call("Save", [filename, itemsave])

    def AddFile(self, filename, parent=0):
        return self._call("AddFile", [os.path.abspath(filename) if filename and not filename.startswith(('http:', 'https:')) else filename, parent])

    def ShowSequence(self, matrix, display_type=SEQUENCE_DISPLAY_DEFAULT, timeout=-1):
        Item(self, 0).ShowSequence(matrix, display_type, timeout)

    def Spray_GetStats(self, id_spray=-1):
        info, data = self._call("Spray_GetStats", [id_spray])
        return str(info).replace('<br>', '\t'), _cols(data) if data and isinstance(data[0], list) else _jmat(data)

    def Collision_Line(self, p1, p2, ref=None):
        res = self._call("Collision_Line", [_points([p1])[0] if not isinstance(p1[0], (list, Mat)) else _points(p1), _points([p2])[0] if not isinstance(p2[0], (list, Mat)) else _points(p2), ref])
        return tuple(res)

    def MeasurePose(self, target=-1, time_avg_ms=0, tip_xyz=None):
        pose, data = self._call("MeasurePose", [target, time_avg_ms, list(tip_xyz) if tip_xyz is not None else [0, 0, 0]])
        return pose, list(data) if isinstance(data, list) else [data]

    def CalibrateTool(self, poses_xyzwpr, input_format=EULER_RX_RY_RZ, algorithm=CALIBRATE_TCP_BY_POINT, robot=None, tool=None):
        if isinstance(poses_xyzwpr, Mat):
            poses_xyzwpr = [list(c) for c in poses_xyzwpr.tr().rows] if poses_xyzwpr.size(0) <= 7 else poses_xyzwpr.rows
        elif poses_xyzwpr and isinstance(poses_xyzwpr[0], Mat):
            if poses_xyzwpr[0].size() == (4, 4):
                poses_xyzwpr = [robomath.Pose_2_Staubli(p) for p in poses_xyzwpr]
                input_format = EULER_RX_RYp_RZpp
            else:
                poses_xyzwpr = [list(p.list()) for p in poses_xyzwpr]
        tcp, stats, errors = self._call("CalibrateTool", [[list(p) for p in poses_xyzwpr], input_format, algorithm, robot, tool])
        return _jmat(tcp).list(), list(stats), _jmat(errors)

    def CalibrateReference(self, joints_points, method=CALIBRATE_FRAME_3P_P1_ON_X, use_joints=False, robot=None):
        pts = _points(joints_points) if isinstance(joints_points, Mat) else [list(p) for p in joints_points]
        pose, stats = self._call("CalibrateReference", [pts, method, use_joints, robot])
        return (pose, list(stats)) if stats and len(stats) > 3 else pose

    def Cam2D_Snapshot(self, file_save_img='', cam_handle=0, params=''):
        if type(file_save_img) is not str:
            raise Exception('The first argument must be a valid file name (str)')
        res = self._call("Cam2D_Snapshot", [file_save_img, cam_handle, params])
        data = None
        if isinstance(res, str) and res.startswith('data:'):
            data = base64.b64decode(res.split(',', 1)[1])
        elif isinstance(res, bytes):
            data = res
        if not file_save_img:
            return data if data is not None else b''
        if data is not None:
            with open(file_save_img, 'wb') as f:
                f.write(data)
            return 1
        return 1 if res in (1, True) else 0

    def Paste(self, paste_to=0, paste_times=1):
        return self._call("Paste", [paste_to, paste_times])

    def RunProgram(self, fcn_param, wait_for_finished=False):
        return self._call("RunProgram", [fcn_param, wait_for_finished], timeout=_LONG if wait_for_finished else None)

    def MoveC(self, target1, target2, itemrobot, blocking=True):
        return itemrobot.MoveC(target1, target2, blocking)

    def CursorXYZ(self, x_coord=-1, y_coord=-1):
        return tuple(self._call("CursorXYZ", [x_coord, y_coord]))

    def GetPoints(self, feature_type=FEATURE_HOVER_OBJECT_MESH):
        if feature_type < FEATURE_HOVER_OBJECT_MESH:
            raise Exception('Invalid feature type, use FEATURE_HOVER_OBJECT_MESH, FEATURE_HOVER_OBJECT or equivalent')
        return tuple(self._call("GetPoints", [feature_type]))

    def Collision(self, item1, item2):
        return self._call("Collision", [item1, item2])

    def ViewPose(self):
        return self._call("ViewPose", [])

    def EmbedWindow(self, window_name, docked_name=None, size_w=-1, size_h=-1, pid=0, area_add=1, area_allowed=15, timeout=500):
        return self._call("EmbedWindow", [window_name, docked_name, size_w, size_h, pid, area_add, area_allowed, timeout])

    # -- studio extensions ------------------------------------------------------------------------
    def AddTool(self, tool_pose, tool_name="New TCP"):
        return self.Item("", ITEM_TYPE_ROBOT).AddTool(tool_pose, tool_name)

    def AddFolder(self, name, itemparent=0):
        return self._call("AddFolder", [name, itemparent])

    def AddRobot(self, library_name, itemparent=0):
        return self._call("AddRobot", [library_name, itemparent])

    def AddMobileRobot(self, name):
        return self._call("AddMobileRobot", [name])

    def AddTargetJ(self, name, joints, itemparent=0, itemrobot=0):
        return self._call("AddTargetJ", [name, list(joints), itemparent, itemrobot])

    def Duplicate(self, item):
        return self._call("Duplicate", [item])

    def setRobotParams(self, robot, dhm, poseBase=None, poseTool=None):
        return self._call("setRobotParams", [robot, [list(r) for r in dhm], poseBase, poseTool])

    def Calibrate_Reference(self, joints_points, method=CALIBRATE_FRAME_3P_P1_ORIGIN, use_joints=False, robot=None):
        return self._call("Calibrate_Reference", [[list(p) for p in joints_points], method, use_joints, robot])

    def Calibrate_Robot(self, measurements, robot=None, options=None):
        return self._call("Calibrate_Robot", [[list(m) for m in measurements], robot, options or {}])

    def StereoCamera_Measure(self):
        return tuple(self._call("StereoCamera_Measure"))

    def BallbarProgram(self, robot=None, center=(1000, 0, 800), radius=150):
        return self._call("BallbarProgram", [robot, list(center), radius])

    def Collision_SetPair(self, item1, item2, id1=-1, id2=-1, collision_check=True):
        return self._call("Collision_SetPair", [item1, item2, id1, id2, collision_check])

    def Collision_SetPairList(self, list_items1, list_items2, list_id1=None, list_id2=None, list_check_state=True):
        return self._call("Collision_SetPairList", [list_items1, list_items2, list_id1 or [], list_id2 or [], list_check_state])

    def getFlagsRoboDK(self):
        return self._call("getFlagsRoboDK")

    def FilterTarget(self, pose, joints_approx=None, robot=None):
        return tuple(self._call("FilterTarget", [pose, joints_approx, robot]))

    def setSimulationTime(self, t):
        return self._call("setSimulationTime", [t])

    def EventsListen(self):
        return self._call("EventsListen")

    def WaitForEvent(self, timeout=3600):
        return self._call("WaitForEvent", [timeout], timeout=timeout + 30)

    def Driver(self, action, **kwargs):
        """Talk to the studio server robot drivers. action: list|connect|disconnect|state|moveJ|moveL|setDO|getDI|runScript|stop."""
        params = dict(kwargs)
        params["action"] = action
        return self._call("__driver__", [params], timeout=600)

    def App(self, path, *args):
        """VerticalBot extension: call an application function (e.g. 'startWorld')."""
        return self._call("__app__", [path] + list(args))


# ---- Item -------------------------------------------------------------------------------------
class Item(ItemAPI):
    """Proxy to a station item (robot, frame, target, program, tool, object...). Same surface as RoboDK's Item."""

    def __init__(self, link, ptr_item=0, itemtype=-1, name=""):
        self.link = link
        self.item = ptr_item if ptr_item not in (None, '') else 0
        self.type = itemtype
        self._name = name

    def __repr__(self):
        if self.Valid():
            return 'RoboDK item (%s) of type %i' % (self.item, int(self.type))
        return 'RoboDK item (INVALID)'

    def __eq__(self, other):
        return isinstance(other, Item) and other.item == self.item

    def __ne__(self, other):
        return not self.__eq__(other)

    def __hash__(self):
        return hash(self.item)

    def _rpc_item(self, method, params, timeout=None):
        return self.link._call(method, list(params), target=self.item or None, timeout=timeout)

    def _c(self, method, *params):
        return self._rpc_item(method, params)

    def equals(self, item2):
        return self.item == item2.item

    def RDK(self):
        return self.link

    def Valid(self, check_deleted=False):
        if not self.item:
            return False
        if check_deleted:
            return self.Type() >= 0
        return True

    def Type(self):
        if not self.item:
            return -1
        t = self._c("Type")
        self.type = t
        return t

    def Delete(self):
        if not self.item:
            raise InputError('Item is not valid or was already deleted')
        self._c("Delete")
        self.item = 0

    def setName(self, name):
        self._name = name
        self._c("setName", name)
        return self

    def AddFile(self, filename):
        return self.link.AddFile(filename, self)

    def Save(self, filename):
        self.link.Save(filename, self)

    def IsInside(self, object):
        return self.link.IsInside(self, object)

    def PoseWrt(self, item):
        from . import robolinkutils
        return robolinkutils.getPoseWrt(self, item)

    def getLinks(self, type_linked=ITEM_TYPE_ROBOT):
        from . import robolinkutils
        return robolinkutils.getLinks(self, type_linked)

    def setValue(self, varname, value=None):
        if value is None or isinstance(value, Mat):
            return self._c("setValue", varname, value)
        if not isinstance(value, str):
            raise Exception('Unsupported value type')
        self._c("setValue", varname, value)

    def AddShape(self, triangle_points):
        return self.link.AddShape(triangle_points, self, False)

    def AddCurve(self, curve_points, add_to_ref=False, projection_type=PROJECTION_ALONG_NORMAL_RECALC):
        return self.link.AddCurve(curve_points, self, add_to_ref, projection_type)

    def AddPoints(self, points, add_to_ref=False, projection_type=PROJECTION_ALONG_NORMAL_RECALC):
        return self.link.AddPoints(points, self, add_to_ref, projection_type)

    def ProjectPoints(self, points, projection_type=PROJECTION_ALONG_NORMAL_RECALC):
        return self.link.ProjectPoints(points, self, projection_type)

    def SelectedFeature(self):
        return tuple(self._c("SelectedFeature"))

    def GetPoints(self, feature_type=FEATURE_SURFACE, feature_id=0):
        if feature_type >= FEATURE_HOVER_OBJECT_MESH:
            raise Exception('Invalid feature type. Use FEATURE_SURFACE, FEATURE_MESH or equivalent.')
        points, name = self._c("GetPoints", feature_type, feature_id)
        return [list(p) for p in points], name

    def setMachiningParameters(self, ncfile='', part=0, params=''):
        prog, status = self._rpc_item("setMachiningParameters", [ncfile, part, params], timeout=3600)
        return prog, status

    def setMillingParameters(self, ncfile='', part=0, params=''):
        prog, status = self._rpc_item("setMillingParameters", [ncfile, part, params], timeout=3600)
        return prog, status

    # -- kinematics -------------------------------------------------------------------------------
    def Joints(self):
        return _jmat(self._c("Joints"))

    def SimulatorJoints(self):
        return list(self._c("SimulatorJoints"))

    def JointsHome(self):
        return _jmat(self._c("JointsHome"))

    def setJointsHome(self, joints):
        self._c("setJointsHome", joints)
        return self

    def setJoints(self, joints):
        self._c("setJoints", joints)
        return self

    def JointLimits(self):
        lo, hi, jtype = self._c("JointLimits")
        return _jmat(lo), _jmat(hi), jtype

    def setJointLimits(self, lower_limit, upper_limit):
        self._c("setJointLimits", lower_limit, upper_limit)

    def JointPoses(self, joints=None):
        return self._c("JointPoses", joints)

    def SolveFK(self, joints, tool=None, reference=None):
        pose = self._c("SolveFK", joints)
        if tool is not None:
            pose = pose * tool
        if reference is not None:
            pose = robomath.invH(reference) * pose
        return pose

    def SolveIK(self, pose, joints_approx=None, tool=None, reference=None):
        if tool is not None:
            pose = pose * robomath.invH(tool)
        if reference is not None:
            pose = reference * pose
        return _jmat(self._c("SolveIK", pose, joints_approx))

    def SolveIK_All(self, pose, tool=None, reference=None):
        if tool is not None:
            pose = pose * robomath.invH(tool)
        if reference is not None:
            pose = reference * pose
        return _cols(self._c("SolveIK_All", pose))

    def JointsConfig(self, joints):
        return _jmat(self._c("JointsConfig", joints))

    def FilterTarget(self, pose, joints_approx=None):
        p, j = self._c("FilterTarget", pose, joints_approx if joints_approx is not None else [0, 0, 0, 0, 0, 0])
        return p, _jmat(j)

    def setPoseFrame(self, frame):
        self._c("setPoseFrame", frame)
        return self

    def setPoseTool(self, tool):
        self._c("setPoseTool", tool)
        return self

    def Htool(self):
        return self.PoseTool()

    def Tool(self):
        return self.PoseTool()

    def Frame(self):
        return self.PoseFrame()

    def setHtool(self, tool):
        self.setPoseTool(tool)

    def setTool(self, tool):
        self.setPoseTool(tool)

    def setFrame(self, frame):
        self.setPoseFrame(frame)

    def setRobot(self, robot=None):
        self._c("setRobot", robot)
        return self

    # -- connection (live drivers through the studio server) ---------------------------------------
    def Connect(self, robot_ip='', blocking=True, driver=None):
        """Connect the robot to a live controller through the server driver (UR / ABB_RWS / KUKA_KVP)."""
        name = self.Name()
        drv = driver or self.getParam("driver") or ("UR" if "UR" in name.upper() else "ABB_RWS" if "ABB" in name.upper() or "IRB" in name.upper() else "KUKA_KVP" if "KUKA" in name.upper() or "KR" in name.upper() else "UR")
        if isinstance(drv, bytes):
            drv = drv.decode(errors='ignore') or 'UR'
        if not robot_ip:
            robot_ip = self.ConnectionParams()[0]
        try:
            self.link.Driver("connect", robot=name, driver=drv, ip=robot_ip)
            self._driver = drv
            return ROBOTCOM_READY
        except Exception as e:  # noqa
            self._last_error = str(e)
            return ROBOTCOM_PROBLEMS

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

    def ConnectionParams(self):
        return tuple(self._c("ConnectionParams"))

    # -- motion -----------------------------------------------------------------------------------
    def _move(self, method, args, blocking):
        res = self._rpc_item(method, list(args) + [blocking], timeout=_LONG if blocking else None)
        return res

    def MoveJ(self, target, blocking=True):
        if self.type == ITEM_TYPE_PROGRAM:
            blocking = False
        self._move("MoveJ", [target], blocking)
        if self._live() and self.Type() == ITEM_TYPE_ROBOT:
            self.link.Driver("moveJ", robot=self.Name(), joints=list(self.Joints().list()))

    def MoveL(self, target, blocking=True):
        if self.type == ITEM_TYPE_PROGRAM:
            blocking = False
        self._move("MoveL", [target], blocking)
        if self._live() and self.Type() == ITEM_TYPE_ROBOT:
            pose = self._c("PoseAbs")
            cols = [pose.rows[r][c] for c in range(4) for r in range(4)]
            self.link.Driver("moveL", robot=self.Name(), pose=cols)

    def MoveC(self, target1, target2, blocking=True):
        if self.type == ITEM_TYPE_PROGRAM:
            blocking = False
        self._move("MoveC", [target1, target2], blocking)

    def SearchL(self, target, blocking=True):
        res = self._move("SearchL", [target], blocking)
        if self.type == ITEM_TYPE_PROGRAM:
            return None
        return _jmat(res)

    def MoveJ_Test(self, j1, j2, minstep_deg=-1):
        return self._rpc_item("MoveJ_Test", [j1, j2, minstep_deg], timeout=3600)

    def MoveJ_Test_Blend(self, j1, j2, j3, blend_deg=5, minstep_deg=-1):
        return self._rpc_item("MoveJ_Test_Blend", [j1, j2, j3, blend_deg, minstep_deg], timeout=3600)

    def MoveL_Test(self, j1, pose, minstep_mm=-1):
        return self._rpc_item("MoveL_Test", [j1, pose, minstep_mm], timeout=3600)

    def setRounding(self, rounding_mm):
        self._c("setRounding", rounding_mm)
        return self

    def setZoneData(self, zonedata):
        return self.setRounding(zonedata)

    def setSpeed(self, speed_linear, speed_joints=-1, accel_linear=-1, accel_joints=-1):
        self._c("setSpeed", speed_linear, speed_joints, accel_linear, accel_joints)
        return self

    def setAcceleration(self, accel_linear):
        self._c("setAcceleration", accel_linear)
        return self

    def setSpeedJoints(self, speed_joints):
        self._c("setSpeedJoints", speed_joints)
        return self

    def setAccelerationJoints(self, accel_joints):
        self._c("setAccelerationJoints", accel_joints)
        return self

    def ShowSequence(self, matrix, display_type=SEQUENCE_DISPLAY_DEFAULT, timeout=-1):
        if isinstance(matrix, Mat):
            data = {"$mat": [list(c) for c in matrix.tr().rows]}
        elif isinstance(matrix, list) and matrix and isinstance(matrix[0], Mat) and matrix[0].size() == (4, 4):
            data = matrix
        else:
            data = [list(r.list()) if isinstance(r, Mat) else list(r) for r in matrix]
        self.link._call("ShowSequence", [data, display_type, timeout], target=self.item or None)

    def Busy(self):
        return int(self._c("Busy"))

    def Stop(self):
        self._c("Stop")

    def WaitMove(self, timeout=360000):
        self._rpc_item("WaitMove", [timeout], timeout=max(timeout, self.link.TIMEOUT) + 30)

    def WaitFinished(self):
        self._rpc_item("WaitFinished", [], timeout=_LONG)

    def ProgramStart(self, programname, folder='', postprocessor=''):
        return self.link.ProgramStart(programname, folder, postprocessor, self)

    def setDO(self, io_var, io_value):
        res = self._c("setDO", str(io_var), io_value if isinstance(io_value, (int, float)) and not isinstance(io_value, bool) else str(int(io_value)) if isinstance(io_value, bool) else str(io_value))
        if self._live() and self.Type() == ITEM_TYPE_ROBOT:
            self.link.Driver("setDO", robot=self.Name(), io=str(io_var), value=io_value)
        return res

    def setAO(self, io_var, io_value):
        return self._c("setAO", str(io_var), io_value if isinstance(io_value, (int, float)) else str(io_value))

    def getDI(self, io_var):
        return str(self._c("getDI", str(io_var)))

    def getAI(self, io_var):
        return str(self._c("getAI", str(io_var)))

    def waitDI(self, io_var, io_value, timeout_ms=-1):
        self._rpc_item("waitDI", [str(io_var), io_value, timeout_ms], timeout=_LONG if timeout_ms < 0 else timeout_ms / 1000.0 + 60)

    def Pause(self, time_ms=-1):
        self._rpc_item("Pause", [time_ms], timeout=_LONG if time_ms < 0 else time_ms / 1000.0 + 60)

    # -- programs ---------------------------------------------------------------------------------
    def RunProgram(self, prog_parameters=None):
        return self.RunCode(prog_parameters)

    def RunCode(self, prog_parameters=None):
        params = [str(p) for p in prog_parameters] if isinstance(prog_parameters, (list, tuple)) else ([] if prog_parameters is None else [str(prog_parameters)])
        return self._c("RunProgram", params)

    def Instruction(self, ins_id=-1):
        r = self._c("Instruction", ins_id)
        if isinstance(r, dict):
            j = r.get("joints")
            return r["name"], r["type"], r.get("moveType"), r.get("isJointTarget"), r.get("pose"), _jmat(j) if j is not None else None
        return tuple(r)

    def InstructionList(self):
        rows, errors = self._c("InstructionList")
        return _cols(rows), errors

    def InstructionListJoints(self, mm_step=10, deg_step=5, save_to_file=None, collision_check=COLLISION_OFF, flags=0, time_step=0.1):
        msg, rows, code = self._rpc_item("InstructionListJoints", [mm_step, deg_step, save_to_file or '', collision_check, flags, time_step], timeout=3600)
        if save_to_file is not None:
            return msg, save_to_file, code
        return msg, _cols(rows), code

    def Update(self, check_collisions=COLLISION_OFF, timeout_sec=3600, mm_step=-1, deg_step=-1):
        return tuple(self._rpc_item("Update", [check_collisions, timeout_sec, mm_step, deg_step], timeout=timeout_sec + 30))

    def MakeProgram(self, folder_path='', run_mode=RUNMODE_MAKE_ROBOTPROG, post=None):
        if len(folder_path) > 0 and not (folder_path.endswith('/') or folder_path.endswith('\\')):
            folder_path = folder_path + '/'
        return tuple(self._rpc_item("MakeProgram", [folder_path, run_mode, post], timeout=600))

    def FilterProgram(self, filestr):
        return tuple(self._c("FilterProgram", filestr))

    def getParam(self, param):
        v = self._c("getParam", param)
        return v

    def setParam(self, param, value='', skip_result=False):
        if isinstance(value, dict):
            value = json.dumps(value)
        elif isinstance(value, Item):
            value = str(value.item)
        elif isinstance(value, bool):
            value = '1' if value else '0'
        elif isinstance(value, (bytes, bytearray, Mat)):
            pass
        else:
            value = str(value)
        res = self._rpc_item("setParam", [param, value, skip_result], timeout=3600)
        if skip_result:
            return None
        # official client: a query (empty value) answered with a JSON object is returned decoded (instruction dicts, ...)
        if isinstance(value, str) and len(value) == 0 and isinstance(res, str) and res.startswith('{'):
            return json.loads(res)
        return res

    # -- extensions kept from earlier studio releases ----------------------------------------------
    def DOF(self):
        return self._c("DOF")

    def MachiningParameters(self):
        return self._c("MachiningParameters")

    def setRobotParams(self, dhm, poseBase=None, poseTool=None):
        return self._c("setRobotParams", [list(r) for r in dhm], poseBase, poseTool)

    def RobotParams(self):
        return self._c("RobotParams")


__all__ = [n for n in dir() if not n.startswith('_')]
