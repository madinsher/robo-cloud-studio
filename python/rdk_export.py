"""Run this script INSIDE RoboDK (Tools > Run Script, or `python rdk_export.py` with RoboDK open).

It walks the open RoboDK station through the official RoboDK API and writes a VerticalBot Studio
station file (.vbstation JSON) next to the .rdk, plus STL meshes for objects/tools. Open the
result in the studio (File > Open) — this is the lossless path for RoboDK projects, since the
.rdk container itself is proprietary.

Exported: frames, robots (name, pose, joints, limits — kinematics matched from the studio library
by name, or DH parameters if available via robot.setParam('JointsDH')), tools (TCP pose, mesh),
targets (pose, joints, cartesian/joint), programs (moves, speeds, frames, tools, pauses, IO, code,
plus the exact joint path from InstructionListJoints for replay), objects (pose + STL + curves and
points used by machining projects), machining projects (curve/point follow, 3D printing, milling:
robot, part, generated program, update status), Python programs (name; source when RoboDK exposes it).
"""
import json
import os
import sys

try:
    from robodk.robolink import *  # noqa
    from robodk.robomath import *  # noqa
except ImportError:  # RoboDK < 5.4
    from robolink import *  # type: ignore # noqa
    from robodk import *  # type: ignore # noqa

RDK = Robolink()
station = RDK.ActiveStation()
out_dir = os.path.dirname(RDK.getParam("PATH_OPENSTATION") or ".") or "."
base_name = os.path.splitext(os.path.basename(RDK.getParam("PATH_OPENSTATION") or "station.rdk"))[0]
mesh_dir = os.path.join(out_dir, base_name + "_meshes")
os.makedirs(mesh_dir, exist_ok=True)

TYPE_MAP = {ITEM_TYPE_FRAME: 3, ITEM_TYPE_ROBOT: 2, ITEM_TYPE_TOOL: 4, ITEM_TYPE_OBJECT: 5, ITEM_TYPE_TARGET: 6, ITEM_TYPE_PROGRAM: 8, ITEM_TYPE_FOLDER: 17,
            ITEM_TYPE_PROGRAM_PYTHON: 10, ITEM_TYPE_MACHINING: 11, ITEM_TYPE_NOTES: 22}
JOINT_PATH_STEP_MM = float(os.environ.get("VBS_JOINTS_STEP_MM", "10"))
JOINT_PATH_MAX_ROWS = int(os.environ.get("VBS_JOINTS_MAX_ROWS", "20000"))


def mat_rows(m):
    """Mat -> list of rows (python lists)."""
    try:
        return [list(r) for r in m.rows]
    except Exception:
        try:
            return m.tolist()
        except Exception:
            return []


def object_features(item):
    """Curves and points stored on an object (used by curve/point follow and 3D printing projects)."""
    curves, points = [], []
    for i in range(256):
        try:
            pts, name = item.GetPoints(FEATURE_CURVE, i)
        except Exception:
            break
        if not pts:
            break
        curves.append({"name": name or ("Curve %d" % (i + 1)), "points": [[float(v) for v in p[:6]] for p in pts]})
    for i in range(1024):
        try:
            pts, name = item.GetPoints(FEATURE_POINT, i)
        except Exception:
            break
        if not pts:
            break
        for p in pts:
            points.append({"name": name or ("Point %d" % (len(points) + 1)), "point": [float(v) for v in p[:6]]})
    return curves, points


def link_name(item, t):
    try:
        l = item.getLink(t)
        return l.Name() if l.Valid() else None
    except Exception:
        return None
counter = [0]


def pose_cols(m):
    """RoboDK Mat (rows) -> column-major 16 list."""
    rows = m.rows if hasattr(m, "rows") else m
    return [rows[r][c] for c in range(4) for r in range(4)]


def new_id(prefix):
    counter[0] += 1
    return "%s_%d" % (prefix, counter[0])


def export_mesh(item, name):
    try:
        path = os.path.join(mesh_dir, name + ".stl")
        RDK.Save(path, item)
        if os.path.exists(path):
            return os.path.relpath(path, out_dir)
    except Exception:
        pass
    return None


def export_item(item, parent_frame=None):
    t = item.Type()
    node = {"id": new_id("rdk"), "type": TYPE_MAP.get(t, 20), "name": item.Name(), "pose": pose_cols(item.Pose() if t != ITEM_TYPE_ROBOT else item.PoseAbs()), "visible": bool(item.Visible()), "params": {"rdk_type": t}, "children": []}
    if t == ITEM_TYPE_ROBOT:
        node["pose"] = pose_cols(item.Pose()) if item.Parent().Type() != ITEM_TYPE_STATION else pose_cols(item.PoseAbs())
        lower, upper = item.JointLimits()
        node["params"].update({"joints": list(item.Joints().list()) if hasattr(item.Joints(), "list") else list(item.Joints()), "lower": list(lower.list()) if hasattr(lower, "list") else list(lower), "upper": list(upper.list()) if hasattr(upper, "list") else list(upper), "library_hint": item.Name()})
        try:
            node["params"]["dh"] = item.setParam("JointsDH")  # available on recent RoboDK versions
        except Exception:
            pass
        node["params"]["activeFrame"] = item.getLink(ITEM_TYPE_FRAME).Name() if item.getLink(ITEM_TYPE_FRAME).Valid() else None
        node["params"]["activeTool"] = item.getLink(ITEM_TYPE_TOOL).Name() if item.getLink(ITEM_TYPE_TOOL).Valid() else None
    elif t == ITEM_TYPE_TOOL:
        node["pose"] = pose_cols(item.PoseTool())
        mesh = export_mesh(item, node["id"])
        if mesh:
            node["geometry"] = [{"mesh": mesh, "origin": pose_cols(eye()), "color": "#8ca0b3"}]
    elif t == ITEM_TYPE_OBJECT:
        mesh = export_mesh(item, node["id"])
        if mesh:
            node["geometry"] = [{"mesh": mesh, "origin": pose_cols(eye()), "color": "#9aa3ad"}]
        curves, points = object_features(item)
        if curves:
            node["curves"] = [{"name": c["name"], "points": [p[:3] for p in c["points"]], "normals": [p[3:6] for p in c["points"] if len(p) >= 6]} for c in curves]
        if points:
            node["points"] = [{"name": p["name"], "point": p["point"][:3], "normal": p["point"][3:6]} for p in points]
    elif t == ITEM_TYPE_TARGET:
        j = item.Joints()
        node["joints"] = list(j.list()) if hasattr(j, "list") else list(j)
        node["isJointTarget"] = bool(item.isJointTarget())
    elif t == ITEM_TYPE_PROGRAM:
        robot = item.getLink(ITEM_TYPE_ROBOT)
        node["params"]["robotName"] = robot.Name() if robot.Valid() else None
        ins = []
        n = item.InstructionCount()
        for i in range(n):
            name, ins_type, move_type, isjoint, pose, joints = item.Instruction(i)
            entry = {"name": name, "type": ins_type, "moveType": move_type, "isJointTarget": bool(isjoint)}
            if ins_type in (INS_TYPE_MOVE, INS_TYPE_MOVEC):
                entry["pose"] = pose_cols(pose)
                entry["joints"] = list(joints.list()) if hasattr(joints, "list") else list(joints)
            ins.append(entry)
        node["params"]["instructions"] = ins
        node["params"]["frameName"] = link_name(item, ITEM_TYPE_FRAME)
        node["params"]["toolName"] = link_name(item, ITEM_TYPE_TOOL)
        # exact joint path as simulated by RoboDK (works for any robot, even ones the studio cannot solve)
        try:
            msg, joints_mat, status = item.InstructionListJoints(JOINT_PATH_STEP_MM, 5, None, COLLISION_OFF, 0, 0.05)
            rows = mat_rows(joints_mat.tr()) if hasattr(joints_mat, "tr") else []
            if rows:
                nj = robot.Joints().list().__len__() if robot.Valid() else len(rows[0])
                node["params"]["jointsList"] = [[float(v) for v in r[:nj]] for r in rows[:JOINT_PATH_MAX_ROWS]]
                node["params"]["jointsListStatus"] = int(status)
                node["params"]["jointsListStepMm"] = JOINT_PATH_STEP_MM
        except Exception as e:  # older RoboDK builds / invalid programs
            node["params"]["jointsListError"] = str(e)
    elif t == ITEM_TYPE_MACHINING:
        node["params"]["robotName"] = link_name(item, ITEM_TYPE_ROBOT)
        node["params"]["partName"] = link_name(item, ITEM_TYPE_OBJECT)
        node["params"]["programName"] = link_name(item, ITEM_TYPE_PROGRAM)
        node["params"]["toolName"] = link_name(item, ITEM_TYPE_TOOL)
        node["params"]["frameName"] = link_name(item, ITEM_TYPE_FRAME)
        for key in ("Machining", "MachiningSettings", "Settings", "NCFile", "PathInput"):
            try:
                val = item.setParam(key)
                if val and isinstance(val, (str, bytes)) and len(val) < 200000:
                    node["params"]["rdk_" + key] = val.decode("utf-8", "ignore") if isinstance(val, bytes) else val
            except Exception:
                pass
        try:
            res = item.Update()
            node["params"]["updateValidRatio"] = float(res[0]) if isinstance(res, (list, tuple)) else float(res)
        except Exception:
            pass
    elif t == ITEM_TYPE_PROGRAM_PYTHON:
        for key in ("Script", "ScriptSource", "Source"):
            try:
                val = item.setParam(key)
                if val and isinstance(val, (str, bytes)) and "def " in (val if isinstance(val, str) else val.decode("utf-8", "ignore")):
                    node["params"]["source"] = val if isinstance(val, str) else val.decode("utf-8", "ignore")
                    break
            except Exception:
                pass
        if "source" not in node["params"]:
            # RoboDK keeps macros as files next to the station or embedded; try the station folder
            for cand in (os.path.join(out_dir, item.Name() + ".py"), os.path.join(out_dir, "Macros", item.Name() + ".py")):
                if os.path.exists(cand):
                    with open(cand, "r", encoding="utf-8", errors="ignore") as f:
                        node["params"]["source"] = f.read()
                    break
    for child in item.Childs():
        node["children"].append(export_item(child))
    return node


root = {"id": "station", "type": 1, "name": station.Name(), "pose": pose_cols(eye()), "visible": True, "params": {"source": "rdk_export.py"}, "children": [export_item(c) for c in station.Childs()], "settings": {"units": "mm"}, "format": "vbs-station", "version": 1}
out = {"format": "vbstation", "version": 1, "savedAt": "", "app": "rdk_export.py", "station": root, "assets": {}}
out_path = os.path.join(out_dir, base_name + ".vbstation")
with open(out_path, "w", encoding="utf-8") as f:
    json.dump(out, f, indent=1)
print("Exported %s (%d top-level items). Meshes in %s" % (out_path, len(root["children"]), mesh_dir))
sys.exit(0)
