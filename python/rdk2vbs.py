"""Convert RoboDK files with RoboDK itself (works with the free version, headless):

    python rdk2vbs.py station.rdk [output.vbstation]
    python rdk2vbs.py robot.robot [output.vbstation]      # library robot -> station with that robot

RoboDK is started hidden (or an existing instance is reused), the file is loaded through the API,
and the station is exported with rdk_export.py's walker (frames, robots, tools, targets, programs, STL meshes).
Set ROBODK_PATH if RoboDK is not in the default location. Requires `pip install robodk` (the official API).
"""
import os
import sys
import json

HERE = os.path.dirname(os.path.abspath(__file__))


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    src = os.path.abspath(sys.argv[1])
    out = os.path.abspath(sys.argv[2]) if len(sys.argv) > 2 else os.path.splitext(src)[0] + ".vbstation"
    try:
        from robodk.robolink import Robolink, ITEM_TYPE_STATION  # official RoboDK API package
    except ImportError:
        print("The official 'robodk' package is required: pip install robodk (and RoboDK installed)")
        return 2
    args = ["-NOSPLASH", "-HIDDEN", "-EXIT_LAST_COM"]
    rdk = Robolink(robodk_path=os.environ.get("ROBODK_PATH") or None, args=args)
    if not rdk.Connect():
        print("Could not start/connect to RoboDK")
        return 3
    if src.lower().endswith(".rdk"):
        rdk.AddFile(src)
        station = rdk.ActiveStation()
    else:
        station = rdk.AddStation(os.path.splitext(os.path.basename(src))[0])
        item = rdk.AddFile(src)
        if not item.Valid():
            print("RoboDK could not load %s" % src)
            return 4
    # reuse the exporter (it reads PATH_OPENSTATION for output placement; we override)
    os.environ["VBS_EXPORT_OUT"] = out
    sys.argv = [sys.argv[0]]
    import importlib.util
    spec = importlib.util.spec_from_file_location("rdk_export", os.path.join(HERE, "rdk_export.py"))
    mod = importlib.util.module_from_spec(spec)
    # rdk_export runs at import: it connects with Robolink() (same instance) and writes next to the .rdk
    try:
        spec.loader.exec_module(mod)
    except SystemExit:
        pass
    produced = os.path.join(os.path.dirname(rdk.getParam("PATH_OPENSTATION") or src) or ".", os.path.splitext(os.path.basename(rdk.getParam("PATH_OPENSTATION") or src))[0] + ".vbstation")
    if os.path.exists(produced) and os.path.abspath(produced) != out:
        os.replace(produced, out)
    print("Written", out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
