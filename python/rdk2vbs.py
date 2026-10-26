"""Convert a RoboDK file with RoboDK itself (the lossless path behind the studio server's POST /convert/rdk).

    python -I rdk2vbs.py <file.rdk|.robot|.tool|.sld> <out.vbstation>

RoboDK (ROBODK_PATH = RoboDK.exe, default C:/RoboDK/bin/RoboDK.exe) is started as a NEW hidden instance on a free API
port (21500-21999, or ROBODK_API_PORT) so it never talks to another RoboDK or to the studio server (port 20500).
The official API is imported from RoboDK's own folder with an isolated sys.path (python/robodk, the studio's
drop-in client, can never shadow it). RoboDK opens the file and saves the station again with its own writer; the
.vbstation carries that re-saved .rdk (base64, `station.params.rdkFile`) for the studio's binary decoder, which
reproduces RoboDK's item tree, kinematics, meshes and programs (tests/robodk_gt.test.ts), plus an API walk of the
station (rdk_export.export_station: base poses, exact names, raw instructions) used for a cross-check.
Exit codes: 0 ok, 2 RoboDK API not found, 3 RoboDK could not start, 4 the file did not load.
"""
import os
import socket
import sys

HERE = os.path.dirname(os.path.abspath(__file__))


def robodk_api_dir():
    exe = os.environ.get('ROBODK_PATH') or r'C:\RoboDK\bin\RoboDK.exe'
    root = os.path.dirname(os.path.dirname(exe)) if exe.lower().endswith('.exe') else exe
    return os.path.join(root, 'Python'), exe


def free_port(lo=21500, hi=21999):
    forced = os.environ.get('ROBODK_API_PORT')
    if forced:
        return int(forced)
    for port in range(lo, hi + 1):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            s.bind(('127.0.0.1', port))
            return port
        except OSError:
            continue
        finally:
            s.close()
    raise RuntimeError('no free port in %d-%d' % (lo, hi))


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return 1
    src = os.path.abspath(sys.argv[1])
    out = os.path.abspath(sys.argv[2])
    api, exe = robodk_api_dir()
    # isolated import path: RoboDK's API first, never this folder (python/robodk is the studio's drop-in client)
    sys.path[:] = [api] + [p for p in sys.path if p and os.path.abspath(p) != HERE]
    try:
        from robodk import robolink
    except ImportError as e:
        print('RoboDK API not found in %s (%s)' % (api, e))
        return 2
    if os.path.abspath(robolink.__file__).startswith(HERE):
        print('refusing the studio drop-in robodk package: %s' % robolink.__file__)
        return 2
    sys.path.append(HERE)  # rdk_export (plain module, no package clash)
    import rdk_export

    port = free_port()
    rdk = robolink.Robolink(robodk_path=exe if os.path.exists(exe) else None, args=['-NOSPLASH', '-HIDDEN', '-NEWINSTANCE', '-NOUI'], port=port)
    try:
        version = rdk.Version()
    except Exception as e:
        print('Could not start/connect to RoboDK on port %d: %s' % (port, e))
        return 3
    try:
        ext = os.path.splitext(src)[1].lower()
        if ext == '.rdk':
            item = rdk.AddFile(src)
            station = item if item.Valid() and item.Type() == robolink.ITEM_TYPE_STATION else rdk.ActiveStation()
        else:
            station = rdk.AddStation(os.path.splitext(os.path.basename(src))[0])
            item = rdk.AddFile(src, station)
        if not item.Valid():
            print('RoboDK could not load %s' % src)
            return 4
        data = rdk_export.export_station(rdk, station, os.path.dirname(out), embed_rdk=True, source_name=os.path.basename(src))
        data['station']['params']['robodkVersion'] = version
        rdk_export.write_json(data, out)
        print('Written', out, 'port', port, 'RoboDK', version)
        return 0
    finally:
        try:
            rdk.CloseRoboDK()
        except Exception:
            pass


if __name__ == '__main__':
    sys.exit(main())
