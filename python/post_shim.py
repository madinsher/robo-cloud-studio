"""Runs a RoboDK-style Python post processor (a module defining class RobotPost) against a
JSON list of post events produced by VerticalBot Studio, and returns the generated program text.

Used in the browser through Pyodide (src/posts/python_post.ts) and testable with CPython:
    python post_shim.py my_post.py events.json > program.txt

Supports the standard RobotPost interface used by RoboDK posts:
  ProgStart, ProgFinish, ProgSave, ProgSendRobot, MoveJ(pose, joints, conf_RLF), MoveL(pose, joints, conf_RLF),
  MoveC(pose1, joints1, pose2, joints2, conf1, conf2), setFrame(pose, frame_id, frame_name), setTool(pose, tool_id, tool_name),
  Pause(time_ms), setSpeed(speed_mms), setAcceleration, setSpeedJoints, setAccelerationJoints, setZoneData(zone_mm),
  setDO(io_var, io_value), setAO, waitDI(io_var, io_value, timeout_ms), RunCode(code, is_function_call), RunMessage(message, iscomment),
  addline, addlog and the PROG/LOG attributes.
"""
import json
import sys
import types
import importlib.util
import os


def _mat_class():
    try:
        from robodk.robomath import Mat  # our shim or the real robodk package
        return Mat
    except Exception:
        from robomath import Mat  # type: ignore
        return Mat


def load_post(path_or_source, name="robodk_post"):
    """Load a post module from a file path or from source text."""
    Mat = _mat_class()
    module = types.ModuleType(name)
    module.__dict__["Mat"] = Mat
    if os.path.exists(str(path_or_source)):
        with open(path_or_source, "r", encoding="utf-8", errors="ignore") as f:
            source = f.read()
    else:
        source = path_or_source
    # RoboDK posts import "from robodk import *" / "from robolink import *" — provide both namespaces
    try:
        import robodk.robomath as rm
        module.__dict__.update({k: getattr(rm, k) for k in dir(rm) if not k.startswith("_")})
    except Exception:
        pass
    exec(compile(source, "robodk_post.py", "exec"), module.__dict__)
    if not hasattr(module, "RobotPost"):
        raise RuntimeError("Post processor has no RobotPost class")
    return module


def run_post(module, events, robot_name="Robot", program_name="Program", dof=6, robot_post="", post_kwargs=None):
    Mat = _mat_class()
    RobotPost = module.RobotPost
    kwargs = dict(post_kwargs or {})
    try:
        post = RobotPost(robotpost=robot_post or module.__name__, robotname=robot_name, robot_axes=dof, **kwargs)
    except TypeError:
        try:
            post = RobotPost(robot_post or module.__name__, robot_name, dof)
        except TypeError:
            post = RobotPost()

    def mat(rows):
        return Mat(rows) if rows is not None else None

    def conf_of(joints):
        if not joints:
            return [0, 0, 0]
        return [0 if joints[0] < 180 else 1, 0, 0 if len(joints) < 5 or joints[4] >= 0 else 1]

    post.ProgStart(program_name)
    frame_id = 1
    tool_id = 1
    for ev in events:
        k = ev.get("kind")
        if k == "moveJ":
            post.MoveJ(mat(ev.get("pose")), ev.get("joints") or [], conf_of(ev.get("joints")))
        elif k == "moveL":
            post.MoveL(mat(ev.get("pose")), ev.get("joints") or [], conf_of(ev.get("joints")))
        elif k == "moveC":
            post.MoveC(mat(ev.get("via")), [], mat(ev.get("pose")), ev.get("joints") or [], [0, 0, 0], conf_of(ev.get("joints")))
        elif k == "setFrame":
            frame_id += 1
            post.setFrame(mat(ev.get("pose")), frame_id, ev.get("name", "Frame"))
        elif k == "setTool":
            tool_id += 1
            post.setTool(mat(ev.get("pose")), tool_id, ev.get("name", "Tool"))
        elif k == "setSpeed":
            if ev.get("speedLinear") is not None:
                post.setSpeed(ev["speedLinear"])
            if ev.get("speedJoints") is not None and hasattr(post, "setSpeedJoints"):
                post.setSpeedJoints(ev["speedJoints"])
            if ev.get("accelLinear") is not None and hasattr(post, "setAcceleration"):
                post.setAcceleration(ev["accelLinear"])
            if ev.get("accelJoints") is not None and hasattr(post, "setAccelerationJoints"):
                post.setAccelerationJoints(ev["accelJoints"])
        elif k == "setRounding":
            post.setZoneData(ev.get("radius", -1))
        elif k == "pause":
            post.Pause(ev.get("timeMs", 0))
        elif k == "setDO":
            post.setDO(ev.get("io"), ev.get("value"))
        elif k == "waitDI":
            post.waitDI(ev.get("io"), ev.get("value"), ev.get("timeoutMs", -1))
        elif k == "runCode":
            post.RunCode(ev.get("code", ""), bool(ev.get("isCall")))
        elif k == "comment":
            post.RunMessage(ev.get("text", ""), True)
        elif k == "message":
            post.RunMessage(ev.get("text", ""), False)
        elif k == "callProgram":
            post.RunCode(ev.get("name", ""), True)
        elif k == "gripper":
            post.RunCode("Attach" if ev.get("close") else "Detach", True)
        else:
            post.RunMessage("%s %s" % (k, json.dumps({kk: v for kk, v in ev.items() if kk != "kind"})), True)
    post.ProgFinish(program_name)
    prog = post.PROG
    if isinstance(prog, list):
        prog = "\n".join(prog)
    log = getattr(post, "LOG", "")
    if isinstance(log, list):
        log = "\n".join(log)
    return {"program": prog, "log": log, "extension": getattr(post, "PROG_EXT", "txt")}


def _prepare_events(raw):
    """Convert the studio JSON event dump (poses as 16 column-major floats) into row-major 4x4 lists."""
    out = []
    for ev in raw:
        e = dict(ev)
        for key in ("pose", "via"):
            v = e.get(key)
            if isinstance(v, list) and len(v) == 16:
                e[key] = [[v[c * 4 + r] for c in range(4)] for r in range(4)]
            elif isinstance(v, dict) and "$pose" in v:
                e[key] = v["$pose"]
        out.append(e)
    return out


def run_from_json(post_source, events_json, robot_name="Robot", program_name="Program", dof=6):
    module = load_post(post_source)
    events = _prepare_events(json.loads(events_json) if isinstance(events_json, str) else events_json)
    return run_post(module, events, robot_name, program_name, dof)


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)
    with open(sys.argv[2], "r", encoding="utf-8") as f:
        events = json.load(f)
    res = run_from_json(sys.argv[1], events, program_name=os.path.splitext(os.path.basename(sys.argv[2]))[0])
    sys.stdout.write(res["program"])
