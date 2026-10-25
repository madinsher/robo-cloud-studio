# Ground-truth tree FK with urdf_parser_py (ROS jazzy). q: 0.3 rad for revolute/continuous, 0.05 m prismatic (mimic honoured)
import sys, json, math
import numpy as np
from urdf_parser_py.urdf import URDF

def rpy_mat(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr], [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr], [-sp, cp*sr, cp*cr]])

def axis_angle(a, t):
    a = np.array(a, float); a = a / np.linalg.norm(a); x, y, z = a; c, s = math.cos(t), math.sin(t); C = 1 - c
    return np.array([[c + x*x*C, x*y*C - z*s, x*z*C + y*s], [y*x*C + z*s, c + y*y*C, y*z*C - x*s], [z*x*C - y*s, z*y*C + x*s, c + z*z*C]])

def T(R, p):
    M = np.eye(4); M[:3, :3] = R; M[:3, 3] = p; return M

def fk(robot, qmode):
    q = {}
    for j in robot.joints:
        if j.type in ('revolute', 'continuous'): q[j.name] = 0.3 if qmode else 0.0
        elif j.type == 'prismatic': q[j.name] = 0.05 if qmode else 0.0
    for j in robot.joints:
        if j.mimic is not None:
            q[j.name] = q.get(j.mimic.joint, 0) * (j.mimic.multiplier if j.mimic.multiplier is not None else 1) + (j.mimic.offset or 0)
    root = robot.get_root()
    poses = {root: np.eye(4)}
    children = {}
    for j in robot.joints: children.setdefault(j.parent, []).append(j)
    stack = [root]
    while stack:
        l = stack.pop()
        for j in children.get(l, []):
            o = j.origin
            xyz = o.xyz if o is not None and o.xyz is not None else [0, 0, 0]
            rpy = o.rpy if o is not None and o.rpy is not None else [0, 0, 0]
            M = poses[l] @ T(rpy_mat(*rpy), xyz)
            ax = j.axis if j.axis is not None else [1, 0, 0]
            v = q.get(j.name, 0)
            if j.type in ('revolute', 'continuous'): M = M @ T(axis_angle(ax, v), [0, 0, 0])
            elif j.type == 'prismatic': M = M @ T(np.eye(3), np.array(ax, float) / np.linalg.norm(ax) * v)
            poses[j.child] = M
            stack.append(j.child)
    return {k: v.tolist() for k, v in poses.items()}

out = {}
for f in sys.argv[2:]:
  try:
    r = URDF.from_xml_string(open(f, encoding='utf-8').read().encode() if False else open(f, encoding='utf-8').read())
    out[f.split('/')[-1]] = {'zero': fk(r, False), 'q': fk(r, True), 'joints': [{'name': j.name, 'type': j.type, 'xyz': (j.origin.xyz if j.origin else None), 'rpy': (j.origin.rpy if j.origin else None), 'axis': j.axis, 'lower': (j.limit.lower if j.limit else None), 'upper': (j.limit.upper if j.limit else None)} for j in r.joints]}
  except Exception as e:
    print('skip', f, e)
json.dump(out, open(sys.argv[1], "w"), separators=(",", ":"))
print('wrote', sys.argv[1], list(out.keys()))
