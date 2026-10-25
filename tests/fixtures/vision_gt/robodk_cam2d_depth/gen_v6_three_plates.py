"""V6: the same Cam2D script against RoboDK 5.9.4 (ground truth) and against the studio (Python drop-in / binary protocol).

    python cam2d_gt.py robodk <port> <outdir>       # starts a new hidden RoboDK instance on <port>, closes it at the end
    python cam2d_gt.py studio <ws_url> <outdir>     # studio Python drop-in (python/robodk, WebSocket JSON-RPC)
    python cam2d_gt.py studio_bin <port> <outdir>   # official robodk package over the RoboDK binary protocol to the studio

Scene: camera frame 1 m above the origin looking down (rotx(pi)); three thin plates (STL written here):
  red   100x100x2 at (120, 80) top z=100   (900 mm in front of the camera)
  green  60x60x2  at (-50, -200) top z=100 (near the image bottom edge; discriminates full vs half FOV angle)
  blue  100x100x2 at (-150, 100) top z=-100 (1100 mm: beyond FAR_LENGTH=1000)
"""
import sys, os, json, struct, time, traceback
mode, conn, outdir = sys.argv[1], sys.argv[2], sys.argv[3]
os.makedirs(outdir, exist_ok=True)
if mode == 'studio':
    sys.path.insert(0, r'C:\robo-cloud-studio\python')
    os.environ['STUDIO_URL'] = conn
else:
    sys.path.insert(0, r'C:\RoboDK\Python')
from robodk.robolink import *
from robodk.robomath import *
res = {'mode': mode, 'errors': []}


def stl_box(path, sx, sy, sz):
    """Binary STL of a box centred in x/y with z in [0, sz]."""
    x0, x1, y0, y1, z0, z1 = -sx / 2, sx / 2, -sy / 2, sy / 2, 0, sz
    v = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0), (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
    f = [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7), (0, 1, 5), (0, 5, 4), (1, 2, 6), (1, 6, 5), (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7)]
    with open(path, 'wb') as fh:
        fh.write(b'\0' * 80 + struct.pack('<I', len(f)))
        for a, b, c in f:
            fh.write(struct.pack('<3f', 0, 0, 0))
            for k in (a, b, c):
                fh.write(struct.pack('<3f', *v[k]))
            fh.write(b'\0\0')


def step(name, fn):
    try:
        r = fn()
        return r
    except Exception as e:
        res['errors'].append('%s: %s' % (name, str(e)[:300]))
        return None


if mode == 'robodk':
    RDK = Robolink(args=['-NOSPLASH', '-NEWINSTANCE'], port=int(conn))
elif mode == 'studio_bin':
    RDK = Robolink(port=int(conn))
else:
    RDK = Robolink()
res['version'] = step('Version', RDK.Version)
if mode != 'robodk':
    step('new station', lambda: RDK.AddStation('V6 camera probe'))
ref = RDK.AddFrame('Camera Reference')
ref.setPose(transl(0, 0, 1000) * rotx(pi))
plates = {}
for name, (sx, sy, sz), pos, col in [('red', (100, 100, 2), (120, 80, 98), [1, 0, 0, 1]), ('green', (60, 60, 2), (-50, -200, 98), [0, 1, 0, 1]), ('blue', (100, 100, 2), (-150, 100, -102), [0, 0, 1, 1])]:
    p = os.path.join(outdir, 'plate_%s.stl' % name)
    stl_box(p, sx, sy, sz)
    it = RDK.AddFile(p)
    step('setPose ' + name, lambda: it.setPose(transl(*pos)))
    step('setColor ' + name, lambda: it.setColor(col))
    step('setName ' + name, lambda: it.setName('plate_' + name))
    plates[name] = it
RDK.Render(True)
step('Cam2D_Close', lambda: RDK.Cam2D_Close())
cam = RDK.Cam2D_Add(ref, 'FOCAL_LENGHT=6 FOV=32 FAR_LENGHT=1000 SIZE=640x480 BG_COLOR=black')
res['cam_valid'] = step('cam.Valid', lambda: bool(cam.Valid()))
res['cam_type'] = step('cam.Type', lambda: cam.Type())
res['cam_name'] = step('cam.Name', lambda: cam.Name())
res['settings'] = step('Settings', lambda: cam.setParam('Settings'))
time.sleep(1.0)
png = os.path.join(outdir, 'snap_rgb.png')
res['snapshot_ret'] = step('Cam2D_Snapshot(file)', lambda: RDK.Cam2D_Snapshot(png, cam))
res['snapshot_file'] = os.path.getsize(png) if os.path.exists(png) else None
b = step('Cam2D_Snapshot("")', lambda: RDK.Cam2D_Snapshot('', cam))
res['snapshot_bytes'] = {'type': type(b).__name__, 'len': (len(b) if b is not None and hasattr(b, '__len__') else None), 'head': (bytes(b[:8]).hex() if isinstance(b, (bytes, bytearray)) else str(b)[:40])}
if isinstance(b, (bytes, bytearray)):
    open(os.path.join(outdir, 'snap_rgb_bytes.png'), 'wb').write(b)
# the 06.c SimulateCamera string (lights)
cam2 = RDK.Cam2D_Add(ref, 'FOCAL_LENGHT=6 FOV=32 FAR_LENGHT=1000 SIZE=640x480 BG_COLOR=black LIGHT_AMBIENT=red LIGHT_DIFFUSE=black LIGHT_SPECULAR=white')
time.sleep(1.0)
res['snapshot06c_ret'] = step('Cam2D_Snapshot 06c', lambda: RDK.Cam2D_Snapshot(os.path.join(outdir, 'snap_06c.png'), cam2))
# depth camera
camd = RDK.Cam2D_Add(ref, 'FOV=32 FAR_LENGTH=1000 SIZE=640x480 DEPTH')
time.sleep(1.0)
gd = os.path.join(outdir, 'snap_depth.grey32')
res['depth_ret'] = step('Cam2D_Snapshot grey32', lambda: RDK.Cam2D_Snapshot(gd, camd))
res['depth_file'] = os.path.getsize(gd) if os.path.exists(gd) else None
bd = step('Cam2D_Snapshot("", DEPTH)', lambda: RDK.Cam2D_Snapshot('', camd, 'DEPTH'))
res['depth_bytes_len'] = len(bd) if isinstance(bd, (bytes, bytearray)) else str(bd)[:60]
res['depth_settings'] = step('depth Settings', lambda: camd.setParam('Settings'))
res['cams_in_station'] = step('ItemList cams', lambda: RDK.ItemList(ITEM_TYPE_CAMERA, True))
step('Cam2D_Close end', lambda: RDK.Cam2D_Close())
json.dump(res, open(os.path.join(outdir, 'result.json'), 'w'), indent=1, default=str)
print(json.dumps(res, indent=1, default=str))
if mode == 'robodk':
    try:
        RDK.CloseRoboDK()
    except Exception:
        pass
