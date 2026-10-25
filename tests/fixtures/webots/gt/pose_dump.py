"""Webots supervisor: dump the absolute pose of every positioned node (PROTO internals included) and every Shape
geometry of the world after the first step, as JSON, then quit. Used to build pose ground truth for the studio's
Webots importer (tests/fixtures/webots/gt/*.json.gz). Output path = first controller argument."""
import json
import os
import sys

WEBOTS_HOME = os.environ.get('WEBOTS_HOME', r'C:\Program Files\Webots')
sys.path.insert(0, os.path.join(WEBOTS_HOME, 'lib', 'controller', 'python'))
from controller import Supervisor, Field  # noqa: E402

sup = Supervisor()
step = int(sup.getBasicTimeStep())
# poses are read before the first physics step (initial state; no settling of contacts or motors)
out = sup.getWorldPath() + '.json'

GEOM_FIELDS = {'Box': ['size'], 'Cylinder': ['radius', 'height'], 'Sphere': ['radius'], 'Capsule': ['radius', 'height'],
               'Cone': ['bottomRadius', 'height'], 'Plane': ['size'], 'Mesh': ['url'], 'ElevationGrid': ['xSpacing', 'ySpacing'],
               'IndexedFaceSet': []}
CHILD_FIELDS = ['children', 'endPoint']
POSED = {'Pose', 'Transform', 'Solid', 'Robot', 'Accelerometer', 'Altimeter', 'Camera', 'Compass', 'Connector', 'Display',
         'DistanceSensor', 'Emitter', 'GPS', 'Gyro', 'InertialUnit', 'LED', 'Lidar', 'LightSensor', 'Pen', 'Radar',
         'RangeFinder', 'Receiver', 'Speaker', 'TouchSensor', 'Track', 'Charger', 'VacuumGripper', 'Fluid'}


def bfield(node, name):
    try:
        return node.getBaseNodeField(name) if node.isProto() else node.getField(name)
    except Exception:
        return None


def value(f):
    if f is None:
        return None
    t = f.getTypeName()
    try:
        if t == 'SFVec3f' or t == 'SFVec2f' or t == 'SFRotation' or t == 'SFColor':
            return list(getattr(f, 'get' + t)())
        if t in ('SFFloat', 'SFInt32', 'SFBool', 'SFString'):
            return getattr(f, 'get' + t)()
        if t == 'MFString':
            return [f.getMFString(i) for i in range(f.getCount())]
    except Exception:
        return None
    return None


nodes = []
counter = [0]


def geom(gn):
    gt = gn.getBaseTypeName()
    g = {'type': gt}
    for k in GEOM_FIELDS.get(gt, []):
        g[k] = value(bfield(gn, k))
    return g


def walk_bo(n, apose, out, depth):
    """boundingObject subtree -> [{geometry, pose}] (geometry nodes directly or in Shape / Pose / Transform / Group)."""
    if n is None or depth > 20:
        return
    b = n.getBaseTypeName()
    if b in ('Pose', 'Transform'):
        p = n.getPose()
        if p and len(p) == 16:
            apose = [round(v, 7) for v in p]
    if b in GEOM_FIELDS:
        out.append({'geometry': geom(n), 'pose': apose, **({'scale': value(bfield(n, 'scale'))} if b == 'Transform' else {})})
        return
    if b == 'Shape':
        g = bfield(n, 'geometry')
        gn = g.getSFNode() if g else None
        if gn:
            out.append({'geometry': geom(gn), 'pose': apose})
        return
    f = bfield(n, 'children')
    if f is not None:
        if f.getTypeName() == 'MFNode':
            for i in range(f.getCount()):
                walk_bo(f.getMFNode(i), apose, out, depth + 1)
        elif f.getTypeName() == 'SFNode':
            walk_bo(f.getSFNode(), apose, out, depth + 1)


def visit(node, parent, depth, apose):
    if node is None or depth > 60:
        return
    counter[0] += 1
    rec = {'id': counter[0], 'parent': parent, 'type': node.getTypeName(), 'base': node.getBaseTypeName()}
    if parent is None:
        rec['wid'] = node.getId()  # Webots unique node id (context.id of a PROTO template); internal nodes refuse getId
    d = node.getDef()
    if d:
        rec['def'] = d
    nm = value(bfield(node, 'name'))
    if isinstance(nm, str):
        rec['name'] = nm
    if rec['base'] in POSED:
        p = node.getPose()
        if p and len(p) == 16:
            apose = [round(v, 7) for v in p]
            rec['pose'] = apose
    elif rec['base'] in ('Shape', 'CadShape') and apose:
        rec['pose'] = apose
    if rec['base'] == 'CadShape':
        rec['url'] = value(bfield(node, 'url'))
    if rec['base'] == 'Transform':
        rec['scale'] = value(bfield(node, 'scale'))
    if rec['base'] in ('HingeJoint', 'SliderJoint', 'Hinge2Joint', 'BallJoint'):
        jp = bfield(node, 'jointParameters')
        jpn = jp.getSFNode() if jp else None
        if jpn:
            rec['position'] = value(bfield(jpn, 'position'))
            rec['axis'] = value(bfield(jpn, 'axis'))
            rec['anchor'] = value(bfield(jpn, 'anchor'))
        names = []
        for df in ('device', 'device2', 'device3'):  # Hinge2Joint / BallJoint have one device list per axis
            dev = bfield(node, df)
            if dev:
                for i in range(dev.getCount()):
                    dn = dev.getMFNode(i)
                    if dn:
                        names.append({'type': dn.getBaseTypeName(), 'name': value(bfield(dn, 'name')), 'axis': df})
        rec['devices'] = names
    if rec['base'] == 'Shape':
        g = bfield(node, 'geometry')
        gn = g.getSFNode() if g else None
        if gn:
            gt = gn.getBaseTypeName()
            rec['geometry'] = {'type': gt}
            for k in GEOM_FIELDS.get(gt, []):
                rec['geometry'][k] = value(bfield(gn, k))
    if rec['base'] in POSED and rec['base'] not in ('Pose', 'Transform'):
        bo = bfield(node, 'boundingObject')
        bon = bo.getSFNode() if bo else None
        if bon:
            rec['bounding'] = []
            walk_bo(bon, apose, rec['bounding'], 0)
    nodes.append(rec)
    for cf in CHILD_FIELDS:
        f = bfield(node, cf)
        if f is None:
            continue
        if f.getTypeName() == 'MFNode':
            for i in range(f.getCount()):
                visit(f.getMFNode(i), rec['id'], depth + 1, apose)
        elif f.getTypeName() == 'SFNode':
            visit(f.getSFNode(), rec['id'], depth + 1, apose)


root = sup.getRoot()
rc = root.getField('children')
for i in range(rc.getCount()):
    visit(rc.getMFNode(i), None, 0, None)

with open(out, 'w', encoding='utf-8') as fh:
    json.dump({'webots': sup.getWorldPath(), 'time': sup.getTime(), 'nodes': nodes}, fh, ensure_ascii=False)
print('pose_dump: %d nodes -> %s' % (len(nodes), out))
sys.stdout.flush()
sup.simulationQuit(0)
sup.step(step)
