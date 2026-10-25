"""Absolute poses of every robot link item (passive counterbalance links included) at several joint vectors.

    python -I dump_linkposes.py <file> <port> <robot name> <out.json>

RoboDK clamps setJoints to the joint limits: the joints recorded are the ones RoboDK applied (robot.Joints()).
"""
import sys, json
sys.path.insert(0, r'C:\RoboDK\Python')
from robodk.robolink import *
from robodk.robomath import *
src, port, rname, out = sys.argv[1], int(sys.argv[2]), sys.argv[3], sys.argv[4]
RDK = Robolink(args=['-NOSPLASH', '-HIDDEN', '-NEWINSTANCE', '-NOUI'], port=port)
res = {'samples': []}
try:
    RDK.AddFile(src)
    r = RDK.Item(rname, ITEM_TYPE_ROBOT)
    q0 = r.Joints().list()
    res['base'] = [v for row in r.PoseAbs().rows for v in row]
    for dq in [[0, 0, 0, 0, 0, 0], [0, 20, 0, 0, 0, 0], [0, -20, 0, 0, 0, 0], [0, 0, 30, 0, 0, 0], [30, 10, -20, 0, 0, 0], [0, 35, -35, 0, 0, 0]]:
        q = [q0[i] + dq[i] if i < len(dq) else q0[i] for i in range(len(q0))]
        r.setJoints(q)
        q = r.Joints().list()
        links = []
        for k in range(0, 12):
            it = r.ObjectLink(k)
            if not it.Valid(): break
            links.append([v for row in it.PoseAbs().rows for v in row])
        res['samples'].append({'q': q, 'links': links})
    print('links per sample', len(res['samples'][0]['links']))
finally:
    json.dump(res, open(out, 'w'))
    RDK.CloseRoboDK()
