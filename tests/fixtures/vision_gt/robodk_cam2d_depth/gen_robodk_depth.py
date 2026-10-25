"""G3: where does RoboDK 5.9.4 sample its Cam2D DEPTH pixels? Tilted planes (ramps) give the exact sample position per
pixel: with a plane tilted about the camera y axis the depth depends on u only, about x on v only.

    python rdk_depth_probe.py robodk <port> <outdir>     # new hidden RoboDK instance, closed at the end
    python rdk_depth_probe.py studio <ws_url> <outdir>   # the studio Python drop-in (same script)
"""
import sys, os, json, struct, time, math
mode, conn, outdir = sys.argv[1], sys.argv[2], sys.argv[3]
os.makedirs(outdir, exist_ok=True)
if mode == 'studio':
    sys.path.insert(0, r'C:\robo-cloud-studio\python')
    os.environ['STUDIO_URL'] = conn
else:
    sys.path.insert(0, r'C:\RoboDK\Python')
from robodk.robolink import *
from robodk.robomath import *


def stl_box(path, sx, sy, sz):
    """Binary STL of a box centred at the origin."""
    x0, x1, y0, y1, z0, z1 = -sx / 2, sx / 2, -sy / 2, sy / 2, -sz / 2, sz / 2
    v = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0), (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
    f = [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7), (0, 1, 5), (0, 5, 4), (1, 2, 6), (1, 6, 5), (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7)]
    with open(path, 'wb') as fh:
        fh.write(b'\0' * 80 + struct.pack('<I', len(f)))
        for a, b, c in f:
            fh.write(struct.pack('<3f', 0, 0, 0))
            for k in (a, b, c):
                fh.write(struct.pack('<3f', *v[k]))
            fh.write(b'\0\0')


res = {'mode': mode}
if mode == 'robodk':
    RDK = Robolink(args=['-NOSPLASH', '-NEWINSTANCE'], port=int(conn))
else:
    RDK = Robolink()
    RDK.AddStation('G3 depth probe')
res['version'] = RDK.Version()
ref = RDK.AddFrame('Camera Reference')
ref.setPose(transl(0, 0, 1000) * rotx(pi))
p = os.path.join(outdir, 'ramp.stl')
stl_box(p, 3000, 3000, 2)
rampA = RDK.AddFile(p); rampA.setName('rampA'); rampA.setPose(roty(20 * pi / 180))
rampB = RDK.AddFile(p); rampB.setName('rampB'); rampB.setPose(rotx(20 * pi / 180))
# a small plate at a known place for the silhouette rule (as in V6): 60x60x2 at (-50,-200), top at z=100
q = os.path.join(outdir, 'plate.stl'); stl_box(q, 60, 60, 2)
plate = RDK.AddFile(q); plate.setName('plate'); plate.setPose(transl(-50, -200, 99))
RDK.Render(True)
cam = RDK.Cam2D_Add(ref, 'FOV=32 FAR_LENGTH=3000 SIZE=640x480 DEPTH')
time.sleep(1.0)
shots = {}
for name, vis in [('rampA', (1, 0, 0)), ('rampB', (0, 1, 0)), ('plate', (0, 0, 1))]:
    rampA.setVisible(bool(vis[0])); rampB.setVisible(bool(vis[1])); plate.setVisible(bool(vis[2]))
    RDK.Render(True); time.sleep(0.5)
    f = os.path.join(outdir, f'depth_{name}.grey32')
    shots[name] = RDK.Cam2D_Snapshot(f, cam)
cam2 = RDK.Cam2D_Add(ref, 'FOV=50 FAR_LENGTH=3000 SIZE=320x240 DEPTH')
time.sleep(1.0)
for name, vis in [('rampA', (1, 0, 0)), ('rampB', (0, 1, 0))]:
    rampA.setVisible(bool(vis[0])); rampB.setVisible(bool(vis[1])); plate.setVisible(bool(vis[2]))
    RDK.Render(True); time.sleep(0.5)
    shots[name + '_small'] = RDK.Cam2D_Snapshot(os.path.join(outdir, f'depth_{name}_small.grey32'), cam2)
res['shots'] = shots
try:
    RDK.Cam2D_Close()
except Exception:
    pass
json.dump(res, open(os.path.join(outdir, 'result.json'), 'w'), indent=1, default=str)
print(json.dumps(res, default=str))
if mode == 'robodk':
    try:
        RDK.CloseRoboDK()
    except Exception:
        pass
