"""Unit tests for the pure (non-bpy) part of the Blender add-on.

Usage: python3 blender/test_core.py <station.vbstation> <fk_expected.json>
The studio's vitest suite generates both files from a demo station and runs this script (tests/blender_addon.test.ts).
"""
import base64
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import vertical_bot_studio as vbs  # noqa: E402


def close(a, b, tol=1e-6):
    return all(abs(x - y) <= tol for x, y in zip(a, b))


def main(station_path, fk_path):
    data, station, items = vbs.load_station(station_path)
    robots = [r for r in items if r["type"] == vbs.ITEM_ROBOT]
    assert robots, "no robots parsed"
    expected = json.load(open(fk_path))
    checked = 0
    for e in expected:
        rec = next(r for r in robots if r["id"] == e["robot"])
        lp, fl = vbs.forward_kinematics(rec["node"]["chain"], e["q"])
        assert len(lp) == len(e["linkPoses"]), "link count %d != %d" % (len(lp), len(e["linkPoses"]))
        for i, m in enumerate(e["linkPoses"]):
            assert close(lp[i], m, 1e-6), "link %d pose mismatch for q=%s" % (i, e["q"])
        assert close(fl, e["flange"], 1e-6), "flange mismatch"
        # absolute flange = robot abs pose * flange
        assert close(vbs.mul(rec["abs"], fl), e["flangeAbs"], 1e-6), "absolute flange mismatch"
        checked += 1
    assert checked == len(expected)
    # tools hang on the flange: the tool record's absolute pose equals robot flange_abs
    tools = [t for t in items if t["type"] == vbs.ITEM_TOOL]
    for t in tools:
        assert any(close(t["abs"], r["flange_abs"], 1e-6) for r in robots), "tool not attached to a flange"
    # primitives produce closed meshes
    for kind, p in (("box", {"kind": "box", "size": [100, 200, 300]}), ("cylinder", {"kind": "cylinder", "radius": 50, "length": 200}), ("sphere", {"kind": "sphere", "radius": 30}), ("cone", {"kind": "cone", "radius": 40, "length": 90})):
        v, f = vbs.primitive_mesh(p)
        assert v and f, kind
        assert max(max(face) for face in f) < len(v), kind
    # geometry of objects resolves (primitives and any STL assets)
    objs = [o for o in items if o["type"] in (vbs.ITEM_OBJECT, vbs.ITEM_TOOL)]
    n_meshes = sum(len(vbs.geometry_meshes(o["node"].get("geometry"), data.get("assets", {}))) for o in objs)
    assert n_meshes > 0, "no meshes resolved"
    # STL round trip through the exporter and parser
    verts = [(0, 0, 0), (100, 0, 0), (0, 100, 0), (0, 0, 100)]
    tris = [(0, 1, 2), (0, 1, 3), (1, 2, 3), (0, 2, 3)]
    stl = vbs.write_stl(verts, tris)
    v2, f2 = vbs.parse_stl(stl)
    assert len(f2) == 4 and len(v2) == 4, (len(v2), len(f2))
    exported = vbs.station_from_meshes("from blender", [("Cube", [(x * 0.001, y * 0.001, z * 0.001) for x, y, z in verts], tris)])
    a = list(exported["assets"].values())[0]
    v3, f3 = vbs.parse_stl(base64.b64decode(a["data"]))
    assert close(v3[1], (100, 0, 0), 1e-3), v3[1]
    assert exported["station"]["children"][0]["geometry"][0]["mesh"] == "Cube.stl"
    # animation block (when present) has rows per robot matching the chain dof
    anim = data.get("animation")
    if anim:
        for rid, rows in anim.get("robots", {}).items():
            rec = next(r for r in robots if r["id"] == rid)
            dof = len(vbs.actuated_indices(rec["node"]["chain"]))
            assert all(len(row) >= dof for row in rows), "animation rows shorter than dof"
            vbs.forward_kinematics(rec["node"]["chain"], rows[-1])
    print("blender core ok: %d robots, %d fk checks, %d meshes, animation=%s" % (len(robots), checked, n_meshes, bool(anim)))


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
