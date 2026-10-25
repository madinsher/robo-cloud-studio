"""Simulation ground truth of a RoboDK station, every program run from the station AS LOADED.

    python -I dump_sim.py <file.rdk> <out.json> <port>

dump_gt.py updates every machining project before it lists the programs (RoboDK then regenerates the linked programs)
and updates the programs one after the other (each Update(COLLISION_ON) leaves the robot where the simulation stopped,
so every program starts where the previous one ended). This dump reloads the station before every program so that
each record is the program exactly as stored in the file, started from the stored robot joints:

  programs[k] = {name, robot, count, start: {robot: joints}, update: Item.Update(COLLISION_OFF) ->
                 [valid instructions, time s, distance mm, valid ratio, message],
                 jl: InstructionListJoints(5 mm, 5 deg, flags=1) rows [J.., ERROR, MM_STEP, DEG_STEP, MOVE_ID, TIME, X, Y, Z]
                     (the last row of every run of equal MOVE_ID = every move end, also inside calls; every 20th row),
                 jlMsg, jlStatus, jlSteps,
                 updateCollision: Item.Update(COLLISION_ON) from the same start,
                 collision: {joints: {robot: joints where RoboDK stopped}, pairs: [[a, b]...]} with a / b = item name or
                            "<robot>#<link>" for robot links (RDK.CollisionItems() at the stopped state)}
Env: DUMP_COLL_MAX=<n> skips the collision-checked update of programs with more than n instructions (default 4000).
"""
import sys, os, json, time
sys.path.insert(0, r'C:\RoboDK\Python')
from robodk.robolink import *
from robodk.robomath import *

src, out, port = sys.argv[1], sys.argv[2], int(sys.argv[3])
COLL_MAX = int(os.environ.get('DUMP_COLL_MAX', '4000'))
T0 = time.time()
res = {'file': src, 'errors': [], 'programs': []}
try:
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')  # program names such as 'Sıde0' (05.b); -I ignores PYTHONIOENCODING
except Exception:
    pass


def safe(fn, default=None, tag=''):
    try:
        return fn()
    except Exception as e:
        if tag:
            res['errors'].append('%s: %s' % (tag, str(e)[:200]))
        return default


def jlist(m):
    try:
        return [round(float(v), 6) for v in m.list()]
    except Exception:
        try:
            return [round(float(v), 6) for v in m]
        except Exception:
            return None


RDK = Robolink(args=['-NOSPLASH', '-HIDDEN', '-NEWINSTANCE', '-NOUI'], port=port)
res['version'] = safe(RDK.Version, '')


def load():
    st = RDK.AddFile(src)
    if st.Valid() and safe(st.Type) == ITEM_TYPE_STATION:
        safe(lambda: RDK.setActiveStation(st))
    return st


def robots_joints():
    return {safe(r.Name, ''): jlist(safe(r.Joints)) for r in RDK.ItemList(ITEM_TYPE_ROBOT)}


def link_names():
    """item pointer -> "<robot>#<link>" for every robot link."""
    names = {}
    for r in RDK.ItemList(ITEM_TYPE_ROBOT):
        rn = safe(r.Name, '')
        for k in range(0, 12):
            it = safe(lambda: r.ObjectLink(k))
            if it is None or not safe(it.Valid, False):
                break
            names[it.item] = '%s#%d' % (rn, k)
    return names


try:
    station = load()
    res['station'] = safe(station.Name, '')
    res['load_time'] = round(time.time() - T0, 2)
    progs0 = RDK.ItemList(ITEM_TYPE_PROGRAM)
    res['robots'] = robots_joints()
    for k in range(len(progs0)):
        if k > 0:
            safe(RDK.CloseStation, None, 'close')
            load()
        p = RDK.ItemList(ITEM_TYPE_PROGRAM)[k]
        name = safe(p.Name, '')
        robot = safe(lambda: p.getLink(ITEM_TYPE_ROBOT), None)
        rname = safe(robot.Name, '') if robot is not None and safe(robot.Valid, False) else ''
        pd = {'name': name, 'robot': rname, 'count': safe(p.InstructionCount, 0) or 0, 'start': robots_joints()}
        t1 = time.time()
        pd['update'] = safe(lambda: list(p.Update(COLLISION_OFF, 600)), None, 'update ' + name)
        mc = safe(lambda: p.InstructionListJoints(5, 5, None, COLLISION_OFF, 1, 0.1), None, 'joints ' + name)
        if mc:
            msg, mat, status = mc
            pd['jlMsg'], pd['jlStatus'] = msg, status
            rows = []
            try:
                rws = getattr(mat, 'rows', None)
                steps = [list(c) for c in zip(*rws)] if rws and isinstance(rws[0], list) else []
                pd['jlSteps'] = len(steps)
                keep = set(range(0, len(steps), 20))
                for i, s in enumerate(steps):
                    dof = len(s) - 8
                    if i == len(steps) - 1 or steps[i + 1][dof + 3] != s[dof + 3]:
                        keep.add(i)
                for i in sorted(keep):
                    rows.append([round(float(v), 4) for v in steps[i]])
            except Exception as e:
                res['errors'].append('jl parse %s: %s' % (name, e))
            pd['jl'] = rows
        if pd['count'] <= COLL_MAX:
            pd['updateCollision'] = safe(lambda: list(p.Update(COLLISION_ON, 600)), None, 'updateC ' + name)
            uc = pd['updateCollision']
            if uc and uc[3] < 0.999 and 'ollision' in str(uc[4]):
                links = link_names()
                n = safe(RDK.Collisions, 0)
                items = safe(RDK.CollisionItems, []) or []
                lab = [links.get(it.item, safe(it.Name, '?')) for it in items]
                pairs = []
                # pair the colliding items: RDK.Collision(a, b) for every combination of the listed items
                for i in range(len(items)):
                    for j in range(i + 1, len(items)):
                        if safe(lambda: RDK.Collision(items[i], items[j]), 0):
                            pairs.append([lab[i], lab[j]])
                pd['collision'] = {'joints': robots_joints(), 'count': n, 'items': lab, 'pairs': pairs}
        else:
            pd['updateCollision'] = None
            pd['collisionSkipped'] = 'more than %d instructions' % COLL_MAX
        pd['simTime'] = round(time.time() - t1, 2)
        res['programs'].append(pd)
        print('  ', name, pd['update'], (pd.get('updateCollision') or [None] * 5)[3], pd.get('collision', {}).get('pairs'), flush=True)
except Exception as e:
    res['errors'].append('fatal: %s' % e)
res['elapsed'] = round(time.time() - T0, 2)
with open(out, 'w', encoding='utf-8') as f:
    json.dump(res, f, default=lambda o: o.decode('latin1') if isinstance(o, bytes) else str(o))
print('OK', src, 'programs', len(res['programs']), 'errors', len(res['errors']), res['elapsed'], 's', flush=True)
try:
    RDK.CloseRoboDK()
except Exception:
    pass
