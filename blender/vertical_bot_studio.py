"""VerticalBot Studio <-> Blender add-on.

Imports `.vbstation` files (the studio's native station format) directly into Blender: objects with their
meshes and primitives, robots as link hierarchies with exact forward kinematics, tools on the flange, frames and
targets as empties, and — when the file was saved with *File › Save for Blender* — the sampled program
animation as keyframes on every robot link and moved object. Exports selected Blender meshes back to a
`.vbstation` the studio opens (binary STL assets in mm) and object animations as CSV pose logs.

Install: Blender › Edit › Preferences › Add-ons › Install… › pick this file. Blender 3.x / 4.x.
The module also runs without Blender (`bpy` missing) so the pure parts are unit-tested by the studio's test suite.
"""
import base64
import json
import math
import os
import struct

bl_info = {
    "name": "VerticalBot Studio (.vbstation)",
    "author": "VerticalBot Studio",
    "version": (0, 2, 0),
    "blender": (3, 0, 0),
    "location": "File > Import/Export > VerticalBot Studio",
    "description": "Import stations, robots and program animations from VerticalBot Studio; export meshes back",
    "category": "Import-Export",
}

try:  # pragma: no cover - Blender only
    import bpy
    from bpy_extras.io_utils import ImportHelper, ExportHelper
    from mathutils import Matrix
except Exception:  # running outside Blender (tests)
    bpy = None
    ImportHelper = ExportHelper = object
    Matrix = None

MM = 0.001  # studio mm -> Blender m

# ------------------------------------------------------------------------------------------------
# Pure-python math on 4x4 matrices stored column-major as 16 floats (studio convention)
# ------------------------------------------------------------------------------------------------


def identity():
    return [1.0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 1.0]


def mul(a, b):
    """a * b for column-major 16-element matrices."""
    out = [0.0] * 16
    for c in range(4):
        for r in range(4):
            out[c * 4 + r] = a[r] * b[c * 4] + a[4 + r] * b[c * 4 + 1] + a[8 + r] * b[c * 4 + 2] + a[12 + r] * b[c * 4 + 3]
    return out


def transl(x, y, z):
    m = identity()
    m[12], m[13], m[14] = x, y, z
    return m


def rot_axis(axis, ang):
    x, y, z = axis
    n = math.sqrt(x * x + y * y + z * z) or 1.0
    x, y, z = x / n, y / n, z / n
    c, s, t = math.cos(ang), math.sin(ang), 1 - math.cos(ang)
    return [t * x * x + c, t * x * y + s * z, t * x * z - s * y, 0,
            t * x * y - s * z, t * y * y + c, t * y * z + s * x, 0,
            t * x * z + s * y, t * y * z - s * x, t * z * z + c, 0,
            0, 0, 0, 1]


def to_rows(m, scale=1.0):
    """Column-major 16 -> row-major 4x4 nested list with translation scaled (mm -> m)."""
    return [[m[0], m[4], m[8], m[12] * scale], [m[1], m[5], m[9], m[13] * scale], [m[2], m[6], m[10], m[14] * scale], [0, 0, 0, 1]]


def from_rows(rows, scale=1.0):
    return [rows[0][0], rows[1][0], rows[2][0], 0, rows[0][1], rows[1][1], rows[2][1], 0, rows[0][2], rows[1][2], rows[2][2], 0,
            rows[0][3] * scale, rows[1][3] * scale, rows[2][3] * scale, 1]


# ------------------------------------------------------------------------------------------------
# Kinematics (mirrors src/core/kinematics/chain.ts)
# ------------------------------------------------------------------------------------------------


def joint_motion(j, value):
    if j.get("type") == "fixed":
        return identity()
    if j.get("type") == "prismatic":
        ax = j["axis"]
        return transl(ax[0] * value, ax[1] * value, ax[2] * value)
    return rot_axis(j["axis"], math.radians(value))


def actuated_indices(chain):
    return [i for i, j in enumerate(chain["joints"]) if j.get("type") != "fixed" and not j.get("mimic")]


def expand_joints(chain, q):
    """Actuated joint values (deg/mm) -> full per-joint values incl. fixed and mimic joints."""
    joints = chain["joints"]
    full = [0.0] * len(joints)
    act = actuated_indices(chain)
    for k, i in enumerate(act):
        full[i] = q[k] if k < len(q) else joints[i].get("home", 0.0)
    by_name = {j["name"]: i for i, j in enumerate(joints)}
    for i, j in enumerate(joints):
        m = j.get("mimic")
        if m:
            src = by_name.get(m["joint"])
            full[i] = (full[src] if src is not None else 0.0) * m.get("multiplier", 1.0) + m.get("offset", 0.0)
    return full


def forward_kinematics(chain, q):
    """Returns (link_poses[0..n], flange) as column-major matrices relative to the robot base."""
    full = expand_joints(chain, q)
    T = identity()
    link_poses = [T[:]]
    for i, j in enumerate(chain["joints"]):
        T = mul(T, j["origin"])
        T = mul(T, joint_motion(j, full[i]))
        if j.get("post"):
            T = mul(T, j["post"])
        link_poses.append(T[:])
    flange = mul(T, chain.get("flange") or identity())
    return link_poses, flange


def home_joints(chain):
    return [chain["joints"][i].get("home", 0.0) for i in actuated_indices(chain)]


# ------------------------------------------------------------------------------------------------
# Station parsing
# ------------------------------------------------------------------------------------------------

ITEM_STATION, ITEM_ROBOT, ITEM_FRAME, ITEM_TOOL, ITEM_OBJECT, ITEM_TARGET, ITEM_PROGRAM, ITEM_FOLDER = 1, 2, 3, 4, 5, 6, 8, 17
ITEM_MOBILE = 100


def walk(node, parent_abs, out, parent_kind=None, robot=None):
    """Flatten the item tree into records with absolute poses (mm)."""
    pose = node.get("pose") or identity()
    t = node.get("type")
    if parent_kind == ITEM_ROBOT and t in (ITEM_TOOL, ITEM_FRAME, ITEM_OBJECT, ITEM_TARGET):
        # children of a robot hang on its flange
        parent_abs = robot["flange_abs"]
    if t == ITEM_TOOL:
        # tool item pose is the TCP relative to the flange; geometry is defined in the flange frame
        abs_pose = parent_abs
    else:
        abs_pose = mul(parent_abs, pose)
    rec = {"node": node, "abs": abs_pose, "type": t, "id": node.get("id"), "name": node.get("name", "item")}
    if t == ITEM_ROBOT and node.get("chain"):
        q = node.get("joints") or home_joints(node["chain"])
        link_poses, flange = forward_kinematics(node["chain"], q)
        rec["link_poses"] = link_poses
        rec["flange_abs"] = mul(abs_pose, flange)
        robot = rec
    out.append(rec)
    for c in node.get("children", []):
        walk(c, abs_pose, out, t, robot if t == ITEM_ROBOT else (robot if parent_kind == ITEM_ROBOT else None))
    return out


def load_station(path_or_dict):
    data = path_or_dict if isinstance(path_or_dict, dict) else json.load(open(path_or_dict, "r", encoding="utf-8"))
    station = data.get("station", data)
    items = walk(station, identity(), [])
    return data, station, items


# ------------------------------------------------------------------------------------------------
# Geometry: primitives and STL assets -> (vertices, faces) in mm
# ------------------------------------------------------------------------------------------------


def primitive_mesh(p, segments=32):
    k = p.get("kind")
    if k == "box":
        sx, sy, sz = [v / 2.0 for v in p["size"]]
        v = [(-sx, -sy, -sz), (sx, -sy, -sz), (sx, sy, -sz), (-sx, sy, -sz), (-sx, -sy, sz), (sx, -sy, sz), (sx, sy, sz), (-sx, sy, sz)]
        f = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
        return v, f
    if k == "plane":
        sx, sy = p["size"][0] / 2.0, p["size"][1] / 2.0
        return [(-sx, -sy, 0), (sx, -sy, 0), (sx, sy, 0), (-sx, sy, 0)], [(0, 1, 2, 3)]
    if k in ("cylinder", "cone"):
        r, h = p["radius"], p["length"]
        r_top = 0.0 if k == "cone" else r
        v, f = [], []
        for i in range(segments):
            a = 2 * math.pi * i / segments
            v.append((r * math.cos(a), r * math.sin(a), -h / 2.0))
        for i in range(segments):
            a = 2 * math.pi * i / segments
            v.append((r_top * math.cos(a), r_top * math.sin(a), h / 2.0))
        for i in range(segments):
            j = (i + 1) % segments
            f.append((i, j, segments + j, segments + i))
        f.append(tuple(reversed(range(segments))))
        f.append(tuple(range(segments, 2 * segments)))
        return v, f
    if k == "sphere":
        r = p["radius"]
        rings, segs = max(4, segments // 2), segments
        v, f = [], []
        for i in range(rings + 1):
            phi = math.pi * i / rings
            for j in range(segs):
                th = 2 * math.pi * j / segs
                v.append((r * math.sin(phi) * math.cos(th), r * math.sin(phi) * math.sin(th), r * math.cos(phi)))
        for i in range(rings):
            for j in range(segs):
                a, b = i * segs + j, i * segs + (j + 1) % segs
                c, d = (i + 1) * segs + (j + 1) % segs, (i + 1) * segs + j
                f.append((a, b, c, d))
        return v, f
    return [], []


def parse_stl(data):
    """Binary or ASCII STL bytes -> (vertices, triangles) with shared vertices."""
    verts, faces, index = [], [], {}

    def vid(p):
        key = (round(p[0], 4), round(p[1], 4), round(p[2], 4))
        i = index.get(key)
        if i is None:
            i = len(verts)
            index[key] = i
            verts.append(key)
        return i

    is_ascii = data[:5] == b"solid" and b"facet" in data[:400]
    if is_ascii:
        nums = []
        for line in data.decode("ascii", "ignore").splitlines():
            line = line.strip()
            if line.startswith("vertex"):
                nums.append(tuple(float(x) for x in line.split()[1:4]))
        for i in range(0, len(nums) - 2, 3):
            faces.append((vid(nums[i]), vid(nums[i + 1]), vid(nums[i + 2])))
        return verts, faces
    n = struct.unpack_from("<I", data, 80)[0]
    off = 84
    for _ in range(n):
        if off + 50 > len(data):
            break
        vals = struct.unpack_from("<12fH", data, off)
        faces.append((vid(vals[3:6]), vid(vals[6:9]), vid(vals[9:12])))
        off += 50
    return verts, faces


def parse_obj(text):
    verts, faces = [], []
    for line in text.splitlines():
        if line.startswith("v "):
            verts.append(tuple(float(x) for x in line.split()[1:4]))
        elif line.startswith("f "):
            idx = [int(p.split("/")[0]) for p in line.split()[1:]]
            faces.append(tuple(i - 1 if i > 0 else len(verts) + i for i in idx))
    return verts, faces


def asset_mesh(assets, mesh_id):
    """Look up an asset by id (or basename) and decode it. Returns (verts, faces, units_scale_to_mm) or None."""
    a = assets.get(mesh_id)
    if a is None:
        base = os.path.basename(mesh_id).lower()
        for k, v in assets.items():
            if os.path.basename(k).lower() == base:
                a = v
                break
    if a is None:
        return None
    raw = base64.b64decode(a["data"])
    t = a.get("type")
    if t == "stl":
        v, f = parse_stl(raw)
    elif t == "obj":
        v, f = parse_obj(raw.decode("utf-8", "ignore"))
    else:
        return None  # dae / glb: export those objects from the studio as glTF instead
    return v, f


def geometry_meshes(geometry, assets):
    """Geometry refs of an object/link -> list of (name, verts_mm, faces, origin16, color)."""
    out = []
    for i, g in enumerate(geometry or []):
        origin = g.get("origin") or identity()
        scale = g.get("scale") or [1, 1, 1]
        if g.get("primitive"):
            v, f = primitive_mesh(g["primitive"])
            out.append(("%s_%d" % (g["primitive"]["kind"], i), v, f, origin, g.get("color")))
        elif g.get("mesh"):
            m = asset_mesh(assets, g["mesh"])
            if m:
                v = [(p[0] * scale[0], p[1] * scale[1], p[2] * scale[2]) for p in m[0]]
                out.append((os.path.basename(g["mesh"]), v, m[1], origin, g.get("color")))
    return out


def hex_color(c):
    try:
        c = (c or "#a5b1c2").lstrip("#")
        return (int(c[0:2], 16) / 255.0, int(c[2:4], 16) / 255.0, int(c[4:6], 16) / 255.0, 1.0)
    except Exception:
        return (0.65, 0.69, 0.76, 1.0)


# ------------------------------------------------------------------------------------------------
# Export helpers (pure)
# ------------------------------------------------------------------------------------------------


def write_stl(verts_mm, tris):
    out = bytearray(b"VerticalBot Studio Blender export".ljust(80, b"\0"))
    out += struct.pack("<I", len(tris))
    for t in tris:
        a, b, c = verts_mm[t[0]], verts_mm[t[1]], verts_mm[t[2]]
        u = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
        w = (c[0] - a[0], c[1] - a[1], c[2] - a[2])
        n = (u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0])
        l = math.sqrt(n[0] ** 2 + n[1] ** 2 + n[2] ** 2) or 1.0
        out += struct.pack("<12fH", n[0] / l, n[1] / l, n[2] / l, *a, *b, *c, 0)
    return bytes(out)


def station_from_meshes(name, meshes):
    """meshes: list of (name, verts_m (world), tris). Returns a .vbstation dict with STL assets in mm."""
    children, assets = [], {}
    for i, (nm, verts, tris) in enumerate(meshes):
        mm = [(v[0] / MM, v[1] / MM, v[2] / MM) for v in verts]
        asset_id = "%s.stl" % nm.replace(" ", "_")
        assets[asset_id] = {"type": "stl", "data": base64.b64encode(write_stl(mm, tris)).decode("ascii"), "name": asset_id}
        children.append({"id": "object_blender_%d" % i, "type": ITEM_OBJECT, "name": nm, "pose": identity(), "visible": True, "params": {"source": "blender"},
                         "children": [], "geometry": [{"mesh": asset_id, "origin": identity(), "color": "#a5b1c2"}], "curves": [], "points": [], "mass": 0})
    station = {"id": "station", "type": ITEM_STATION, "name": name, "pose": identity(), "visible": True, "params": {"source": "blender"}, "children": children, "settings": {"units": "mm"}, "format": "vbs-station", "version": 1}
    return {"format": "vbstation", "version": 1, "savedAt": "", "app": "blender-addon", "station": station, "assets": assets}


# ------------------------------------------------------------------------------------------------
# Blender operators
# ------------------------------------------------------------------------------------------------

if bpy is not None:  # pragma: no cover - Blender only

    def _mat(m16, scale=MM):
        return Matrix(to_rows(m16, scale))

    def _new_mesh_object(name, verts_mm, faces, color, collection):
        me = bpy.data.meshes.new(name)
        me.from_pydata([(v[0] * MM, v[1] * MM, v[2] * MM) for v in verts_mm], [], [tuple(f) for f in faces if len(f) >= 3])
        me.update()
        ob = bpy.data.objects.new(name, me)
        mat = bpy.data.materials.new(name + "_mat")
        mat.diffuse_color = hex_color(color)
        me.materials.append(mat)
        collection.objects.link(ob)
        return ob

    def _new_empty(name, collection, kind="ARROWS", size=0.1):
        ob = bpy.data.objects.new(name, None)
        ob.empty_display_type = kind
        ob.empty_display_size = size
        collection.objects.link(ob)
        return ob

    def import_station(filepath, with_animation=True, fps=None):
        data, station, items = load_station(filepath)
        assets = data.get("assets", {})
        coll = bpy.data.collections.new(station.get("name", "Station"))
        bpy.context.scene.collection.children.link(coll)
        objects_by_id = {}
        link_empties = {}  # robot id -> [empties]
        for rec in items:
            node, t = rec["node"], rec["type"]
            if t == ITEM_ROBOT and node.get("chain"):
                chain = node["chain"]
                root = _new_empty(rec["name"], coll, "PLAIN_AXES", 0.2)
                root.matrix_world = _mat(rec["abs"])
                objects_by_id[rec["id"]] = root
                empties = []
                for i, link in enumerate(chain["links"]):
                    e = _new_empty("%s.%s" % (rec["name"], link.get("name") or ("link_%d" % i)), coll, "PLAIN_AXES", 0.05)
                    e.parent = root
                    e.matrix_basis = _mat(rec["link_poses"][i])
                    empties.append(e)
                    for (gname, v, f, origin, color) in geometry_meshes(link.get("visuals"), assets):
                        mo = _new_mesh_object("%s.%s" % (e.name, gname), v, f, color, coll)
                        mo.parent = e
                        mo.matrix_basis = _mat(origin)
                flange = _new_empty("%s.flange" % rec["name"], coll, "ARROWS", 0.08)
                flange.parent = root
                flange.matrix_basis = _mat(mul(rec["link_poses"][-1] if False else identity(), identity()))
                # flange = last link pose * chain flange
                lp, fl = forward_kinematics(chain, node.get("joints") or home_joints(chain))
                flange.matrix_basis = _mat(fl)
                empties.append(flange)
                link_empties[rec["id"]] = (empties, chain, root)
            elif t in (ITEM_OBJECT, ITEM_TOOL, ITEM_MOBILE):
                meshes = geometry_meshes(node.get("geometry"), assets)
                holder = _new_empty(rec["name"], coll, "PLAIN_AXES", 0.05)
                holder.matrix_world = _mat(rec["abs"])
                objects_by_id[rec["id"]] = holder
                for (gname, v, f, origin, color) in meshes:
                    mo = _new_mesh_object("%s.%s" % (rec["name"], gname), v, f, color, coll)
                    mo.parent = holder
                    mo.matrix_basis = _mat(origin)
            elif t in (ITEM_FRAME, ITEM_TARGET):
                e = _new_empty(rec["name"], coll, "ARROWS" if t == ITEM_FRAME else "SPHERE", 0.1 if t == ITEM_FRAME else 0.02)
                e.matrix_world = _mat(rec["abs"])
                objects_by_id[rec["id"]] = e
        # animation block written by File > Save for Blender
        anim = data.get("animation") if with_animation else None
        if anim:
            scene = bpy.context.scene
            dt = float(anim.get("dt", 0.05))
            scene.render.fps = fps or int(round(1.0 / dt))
            n = 0
            for rid, rows in (anim.get("robots") or {}).items():
                if rid not in link_empties:
                    continue
                empties, chain, root = link_empties[rid]
                for k, q in enumerate(rows):
                    lp, fl = forward_kinematics(chain, q)
                    frame = k + 1
                    for i, e in enumerate(empties[:-1]):
                        e.matrix_basis = _mat(lp[i])
                        e.keyframe_insert("location", frame=frame)
                        e.keyframe_insert("rotation_euler", frame=frame)
                    empties[-1].matrix_basis = _mat(fl)
                    empties[-1].keyframe_insert("location", frame=frame)
                    empties[-1].keyframe_insert("rotation_euler", frame=frame)
                n = max(n, len(rows))
            for iid, poses in (anim.get("items") or {}).items():
                ob = objects_by_id.get(iid)
                if not ob:
                    continue
                for k, p in enumerate(poses):
                    if p is None:
                        continue
                    ob.matrix_world = _mat(p)
                    ob.keyframe_insert("location", frame=k + 1)
                    ob.keyframe_insert("rotation_euler", frame=k + 1)
                n = max(n, len(poses))
            scene.frame_start, scene.frame_end = 1, max(1, n)
        return coll

    class VBS_OT_import(bpy.types.Operator, ImportHelper):
        bl_idname = "import_scene.vbstation"
        bl_label = "VerticalBot Studio station (.vbstation)"
        filename_ext = ".vbstation"
        filter_glob: bpy.props.StringProperty(default="*.vbstation;*.json", options={"HIDDEN"})
        with_animation: bpy.props.BoolProperty(name="Import program animation", default=True)

        def execute(self, context):
            import_station(self.filepath, self.with_animation)
            return {"FINISHED"}

    class VBS_OT_export(bpy.types.Operator, ExportHelper):
        bl_idname = "export_scene.vbstation"
        bl_label = "VerticalBot Studio station (.vbstation)"
        filename_ext = ".vbstation"
        selected_only: bpy.props.BoolProperty(name="Selected objects only", default=True)

        def execute(self, context):
            objs = context.selected_objects if self.selected_only else context.scene.objects
            meshes = []
            depsgraph = context.evaluated_depsgraph_get()
            for ob in objs:
                if ob.type != "MESH":
                    continue
                ev = ob.evaluated_get(depsgraph)
                me = ev.to_mesh()
                me.calc_loop_triangles()
                verts = [ob.matrix_world @ v.co for v in me.vertices]
                tris = [tuple(t.vertices) for t in me.loop_triangles]
                meshes.append((ob.name, [(v.x, v.y, v.z) for v in verts], tris))
                ev.to_mesh_clear()
            data = station_from_meshes(os.path.splitext(os.path.basename(self.filepath))[0], meshes)
            with open(self.filepath, "w", encoding="utf-8") as f:
                json.dump(data, f)
            self.report({"INFO"}, "Exported %d meshes" % len(meshes))
            return {"FINISHED"}

    class VBS_OT_export_motion(bpy.types.Operator, ExportHelper):
        """Selected objects' world poses per frame as CSV (mm, RoboDK XYZ + Rx Ry Rz deg): a pose log the studio imports as targets."""
        bl_idname = "export_scene.vbstation_motion"
        bl_label = "VerticalBot Studio pose log (.csv)"
        filename_ext = ".csv"

        def execute(self, context):
            scene = context.scene
            lines = ["object,frame,t,x,y,z,rx,ry,rz"]
            for fr in range(scene.frame_start, scene.frame_end + 1):
                scene.frame_set(fr)
                for ob in context.selected_objects:
                    m = ob.matrix_world
                    e = m.to_euler("XYZ")
                    lines.append("%s,%d,%.4f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f" % (ob.name, fr, (fr - scene.frame_start) / scene.render.fps, m.translation.x / MM, m.translation.y / MM, m.translation.z / MM, math.degrees(e.x), math.degrees(e.y), math.degrees(e.z)))
            with open(self.filepath, "w", encoding="utf-8") as f:
                f.write("\n".join(lines))
            return {"FINISHED"}

    def _menu_import(self, context):
        self.layout.operator(VBS_OT_import.bl_idname)

    def _menu_export(self, context):
        self.layout.operator(VBS_OT_export.bl_idname)
        self.layout.operator(VBS_OT_export_motion.bl_idname)

    classes = (VBS_OT_import, VBS_OT_export, VBS_OT_export_motion)

    def register():
        for c in classes:
            bpy.utils.register_class(c)
        bpy.types.TOPBAR_MT_file_import.append(_menu_import)
        bpy.types.TOPBAR_MT_file_export.append(_menu_export)

    def unregister():
        bpy.types.TOPBAR_MT_file_import.remove(_menu_import)
        bpy.types.TOPBAR_MT_file_export.remove(_menu_export)
        for c in reversed(classes):
            bpy.utils.unregister_class(c)

    if __name__ == "__main__":
        register()
