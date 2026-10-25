"""Export the open RoboDK station to a VerticalBot Studio station file (.vbstation JSON) through the official API.

Run it INSIDE RoboDK (Tools > Run Script, or `python rdk_export.py` while RoboDK is open), or let the studio
server call it through rdk2vbs.py (POST /convert/rdk, a hidden RoboDK on a free port).

Lossless part: RoboDK saves the station again with its own writer and the file is embedded (base64) as
`station.params.rdkFile`; the studio decodes it with its binary reader (src/io/robodk), which reproduces RoboDK's
tree, robot kinematics and meshes, targets, programs (by item reference), Python programs, curves and points.
API walk (always written, used for the cross-check and by studios without the binary reader): frames, robots (base
pose relative to the parent - not the TCP that Robot.Pose() returns -, exact name, joints, home, limits, active
frame / tool), tools (TCP), objects, targets (pose, joints, joint flag), programs (robot, raw Instruction() tuples
with the official INS_TYPE numbering in `insType`), machining projects (links), Python programs (names).
VBS_EXPORT_STL=1 additionally embeds object / tool meshes as STL assets; VBS_JOINT_PATH=1 adds RoboDK's simulated
joint path (InstructionListJoints) per program.
"""
import base64
import json
import os
import shutil
import sys
import tempfile

try:
    from robodk.robolink import *  # noqa
    from robodk.robomath import *  # noqa
except ImportError:  # RoboDK < 5.4
    from robolink import *  # type: ignore # noqa
    from robodk import *  # type: ignore # noqa

TYPE_MAP = {ITEM_TYPE_FRAME: 3, ITEM_TYPE_ROBOT: 2, ITEM_TYPE_TOOL: 4, ITEM_TYPE_OBJECT: 5, ITEM_TYPE_TARGET: 6, ITEM_TYPE_PROGRAM: 8, ITEM_TYPE_FOLDER: 17,
            ITEM_TYPE_PROGRAM_PYTHON: 10, ITEM_TYPE_MACHINING: 11, ITEM_TYPE_NOTES: 22}


def pose_cols(m):
    """RoboDK Mat (rows) -> column-major list of 16 (the studio's Mat4 layout)."""
    rows = m.rows if hasattr(m, 'rows') else m
    return [float(rows[r][c]) for c in range(4) for r in range(4)]


def as_list(v):
    try:
        return [float(x) for x in (v.list() if hasattr(v, 'list') else v)]
    except Exception:
        return []


def link_name(item, t):
    try:
        l = item.getLink(t)
        return l.Name() if l.Valid() else None
    except Exception:
        return None


def rel_pose(item):
    """Pose of an item in its parent's frame. Robots: the base (Robot.Pose() would be the TCP)."""
    t = item.Type()
    if t == ITEM_TYPE_TOOL:
        return item.PoseTool()
    if t != ITEM_TYPE_ROBOT:
        return item.Pose()
    par = item.Parent()
    if not par.Valid() or par.Type() == ITEM_TYPE_STATION:
        ref = eye()
    elif par.Type() == ITEM_TYPE_ROBOT:  # carried on the flange of another robot
        ref = par.PoseAbs() * par.SolveFK(par.Joints())
    else:
        ref = par.PoseAbs()
    return invH(ref) * item.PoseAbs()


class Exporter:
    def __init__(self, rdk, out_dir, stl=False, joint_path=False):
        self.rdk, self.out_dir, self.stl, self.joint_path = rdk, out_dir, stl, joint_path
        self.assets, self.n = {}, 0
        self.tmp = tempfile.mkdtemp(prefix='vbs-rdk-')

    def new_id(self, prefix):
        self.n += 1
        return '%s_%d' % (prefix, self.n)

    def mesh(self, item, node_id):
        if not self.stl:
            return None
        path = os.path.join(self.tmp, node_id + '.stl')
        try:
            self.rdk.Save(path, item)
            with open(path, 'rb') as f:
                self.assets[node_id] = {'type': 'stl', 'data': base64.b64encode(f.read()).decode('ascii'), 'name': item.Name()}
            return node_id
        except Exception:
            return None

    def item(self, item):
        t = item.Type()
        node = {'id': self.new_id('rdk'), 'type': TYPE_MAP.get(t, 20), 'name': item.Name(), 'pose': pose_cols(rel_pose(item)),
                'visible': bool(item.Visible()), 'params': {'rdk_type': t}, 'children': []}
        p = node['params']
        if t == ITEM_TYPE_ROBOT:
            lower, upper, jtype = (list(item.JointLimits()) + [None])[:3]
            p.update({'joints': as_list(item.Joints()), 'home': as_list(item.JointsHome()), 'lower': as_list(lower), 'upper': as_list(upper),
                      'jointsType': jtype, 'library_hint': item.Name(), 'activeFrame': link_name(item, ITEM_TYPE_FRAME), 'activeTool': link_name(item, ITEM_TYPE_TOOL)})
        elif t in (ITEM_TYPE_TOOL, ITEM_TYPE_OBJECT):
            m = self.mesh(item, node['id'])
            if m:
                node['geometry'] = [{'mesh': m, 'origin': pose_cols(eye()), 'color': '#9aa3ad'}]
        elif t == ITEM_TYPE_TARGET:
            node['joints'] = as_list(item.Joints())
            node['isJointTarget'] = bool(item.isJointTarget())
            p['robotName'] = link_name(item, ITEM_TYPE_ROBOT)
        elif t == ITEM_TYPE_PROGRAM:
            p['robotName'] = link_name(item, ITEM_TYPE_ROBOT)
            ins = []
            for i in range(item.InstructionCount()):
                name, ins_type, move_type, isjoint, pose, joints = item.Instruction(i)
                e = {'name': name, 'insType': ins_type, 'moveType': move_type, 'isJointTarget': bool(isjoint)}
                if pose is not None:
                    e['pose'] = pose_cols(pose)
                if joints is not None:
                    e['joints'] = as_list(joints)
                ins.append(e)
            p['instructions'] = ins
            if self.joint_path:
                try:
                    msg, mat, status = item.InstructionListJoints(10, 5, None, COLLISION_OFF, 0, 0.05)
                    rows = [list(r) for r in mat.tr().rows] if hasattr(mat, 'tr') else []
                    robot = item.getLink(ITEM_TYPE_ROBOT)
                    nj = len(as_list(robot.Joints())) if robot.Valid() else (len(rows[0]) if rows else 0)
                    p['jointsList'] = [[float(v) for v in r[:nj]] for r in rows[:20000]]
                    p['jointsListStatus'] = int(status)
                    p['jointsListStepMm'] = 10
                except Exception as e:
                    p['jointsListError'] = str(e)
        elif t == ITEM_TYPE_MACHINING:
            for k, lt in (('robotName', ITEM_TYPE_ROBOT), ('partName', ITEM_TYPE_OBJECT), ('programName', ITEM_TYPE_PROGRAM), ('toolName', ITEM_TYPE_TOOL), ('frameName', ITEM_TYPE_FRAME)):
                p[k] = link_name(item, lt)
        for child in item.Childs():
            node['children'].append(self.item(child))
        return node


def export_station(rdk, station, out_dir, embed_rdk=True, source_name=None):
    ex = Exporter(rdk, out_dir, stl=os.environ.get('VBS_EXPORT_STL') == '1' or not embed_rdk, joint_path=os.environ.get('VBS_JOINT_PATH') == '1')
    root = {'id': 'station', 'type': 1, 'name': station.Name(), 'pose': pose_cols(eye()), 'visible': True, 'params': {'source': 'rdk_export.py'},
            'children': [ex.item(c) for c in station.Childs()], 'settings': {'units': 'mm'}, 'format': 'vbs-station', 'version': 1}
    if embed_rdk:
        path = os.path.join(ex.tmp, 'station.rdk')
        rdk.Save(path, station)
        with open(path, 'rb') as f:
            raw = f.read()
        name = source_name if source_name and source_name.lower().endswith('.rdk') else station.Name() + '.rdk'
        root['params']['rdkFile'] = {'name': name, 'bytes': len(raw), 'data': base64.b64encode(raw).decode('ascii')}
    shutil.rmtree(ex.tmp, ignore_errors=True)
    return {'format': 'vbstation', 'version': 1, 'savedAt': '', 'app': 'rdk_export.py', 'station': root, 'assets': ex.assets}


def write_json(data, out_path):
    with open(out_path, 'w', encoding='utf-8') as f:
        json.dump(data, f)


if __name__ == '__main__':
    RDK = Robolink()
    station = RDK.ActiveStation()
    file = RDK.getParam('FILE_OPENSTATION') or ''
    folder = RDK.getParam('PATH_OPENSTATION') or (os.path.dirname(file) if file else '.')
    out = os.environ.get('VBS_EXPORT_OUT') or os.path.join(folder, (os.path.splitext(os.path.basename(file))[0] if file else station.Name()) + '.vbstation')
    write_json(export_station(RDK, station, folder, embed_rdk=True, source_name=os.path.basename(file) if file else None), out)
    print('Exported %s' % out)
