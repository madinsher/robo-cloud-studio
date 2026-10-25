"""RoboDK Item.Update() of every program with a positioner set beforehand to given angles (05.b evidence).

    python -I dump_turntable.py <file.rdk> <out.json> <port> <positioner name> <angle> [<angle> ...]

RoboDK's Update() keeps other mechanisms where they are: a program that turns a turntable through a called program
(05.b TopSurf calls TT0deg) is evaluated with the turntable at its current angle. This dump records Update(COLLISION_OFF)
of every program with the positioner set (setJoints) to each angle, from the station as loaded (reloaded per angle).
"""
import sys, json
sys.path.insert(0, r'C:\RoboDK\Python')
from robodk.robolink import *
src, out, port, mech = sys.argv[1], sys.argv[2], int(sys.argv[3]), sys.argv[4]
angles = [float(a) for a in sys.argv[5:]]
try:
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
except Exception:
    pass
RDK = Robolink(args=['-NOSPLASH', '-HIDDEN', '-NEWINSTANCE', '-NOUI'], port=port)
res = {'file': src, 'positioner': mech, 'angles': {}}
try:
    for a in angles:
        RDK.CloseStation()
        RDK.AddFile(src)
        m = RDK.Item(mech, ITEM_TYPE_ROBOT)
        m.setJoints([a])
        rec = {}
        for p in RDK.ItemList(ITEM_TYPE_PROGRAM):
            rec[p.Name()] = list(p.Update(COLLISION_OFF, 300))
        res['angles'][str(a)] = rec
        print(a, {k: [round(v[1], 3), v[3]] for k, v in rec.items()}, flush=True)
finally:
    with open(out, 'w', encoding='utf-8') as f:
        json.dump(res, f, ensure_ascii=False)
    RDK.CloseRoboDK()
