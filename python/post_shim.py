"""Runs a RoboDK post processor (a module defining class RobotPost) on a "post job" produced by VerticalBot Studio
and returns every file the post writes.

Two execution modes, same event mapping:
  * native (server / desktop / fixture generator): the post is imported as a real module (RoboDK's own compiled
    libspp/vXY posts work) with __file__ set and its folder on sys.path, using RoboDK's `robodk` package
    (run it with RoboDK's embedded Python, e.g. C:/RoboDK/Python-Embedded/python.exe):
        python post_shim.py C:/RoboDK/Posts/KUKA_KRC4.py job.json out_dir   -> prints a JSON result
        python post_shim.py KUKA_KRC4 job.json out_dir --posts C:/RoboDK/Posts
  * Pyodide (browser, src/posts/python_post.ts): source posts (and v312 bytecode) are loaded from text with
    run_job_source(); files are written to a virtual folder and returned.

Post job (JSON): {program, robot, dof, axes_type, frame:{name,pose}, tool:{name,pose}, events:[...],
subprograms:[{name, events}], header_comments:[...]}; poses are 16 column-major numbers (studio Mat4) or
4x4 row lists. Event kinds are the studio PostEvent kinds (src/posts/base.ts).
RoboDK's Robolink is blocked: posts that need the RoboDK API at generation time raise instead of starting RoboDK.
"""
import base64
import contextlib
import importlib
import importlib.util
import io
import json
import os
import shutil
import subprocess
import sys
import tempfile
import traceback
import types


# ---- robodk package / Robolink guard -------------------------------------------------------------------------------
class _BlockedRobolink(object):
    def __init__(self, *a, **k):
        raise RuntimeError('robodk.robolink.Robolink is not available while generating code from VerticalBot Studio')


def _guard_robolink():
    try:
        import robodk.robolink as rl
        rl.Robolink = _BlockedRobolink
        sys.modules.setdefault('robolink', rl)
    except Exception:
        pass


def _mat_class():
    try:
        from robodk.robomath import Mat
        return Mat
    except Exception:
        from robomath import Mat  # type: ignore
        return Mat


# ---- loading ---------------------------------------------------------------------------------------------------------
def load_post_native(post, posts_dir=None):
    """Import a post module natively: `post` is a path to a .py/.pyc file or a module name inside posts_dir."""
    _guard_robolink()
    if os.path.isfile(str(post)):
        path = os.path.abspath(post)
        folder = os.path.dirname(path)
        name = os.path.splitext(os.path.basename(path))[0]
    else:
        folder = posts_dir or 'C:/RoboDK/Posts'
        name = post
        path = os.path.join(folder, name + '.py')
    if folder not in sys.path:
        sys.path.insert(0, folder)
    if name in sys.modules:
        return sys.modules[name]
    if path.endswith('.py') and os.path.isfile(path):
        return importlib.import_module(name)
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def load_post_source(source, name='robodk_post', folder='/studio/posts'):
    """Load a post from source text (Pyodide). __file__ points into `folder` so posts that read their own path work."""
    _guard_robolink()
    Mat = _mat_class()
    module = types.ModuleType(name)
    module.__dict__['Mat'] = Mat
    module.__dict__['__file__'] = folder.rstrip('/') + '/' + name + '.py'
    module.__dict__['__name__'] = name
    if folder not in sys.path:
        sys.path.insert(0, folder)
    try:
        import robodk.robomath as rm
        module.__dict__.update({k: getattr(rm, k) for k in dir(rm) if not k.startswith('_')})
    except Exception:
        pass
    sys.modules[name] = module
    exec(compile(source, module.__dict__['__file__'], 'exec'), module.__dict__)
    if not hasattr(module, 'RobotPost'):
        raise RuntimeError('Post processor has no RobotPost class')
    return module


def load_post(path_or_source, name='robodk_post'):
    """Backwards compatible loader: a file path is imported natively, anything else is treated as source."""
    if os.path.exists(str(path_or_source)):
        return load_post_native(path_or_source)
    return load_post_source(path_or_source, name)


# ---- event mapping ---------------------------------------------------------------------------------------------------
def _rows(v):
    if v is None:
        return None
    if isinstance(v, dict) and '$pose' in v:
        return v['$pose']
    if isinstance(v, list) and len(v) == 16 and not isinstance(v[0], list):
        return [[v[c * 4 + r] for c in range(4)] for r in range(4)]
    return v


def _io(v):
    """RoboDK passes numeric IO names as numbers (posts then write $OUT[3], DO[3], ...) and names as strings."""
    if isinstance(v, str) and v.strip().lstrip('-').isdigit():
        return int(v)
    return v


def _value(v):
    if isinstance(v, bool):
        return 1 if v else 0
    return v


def _make_post(module, job):
    RobotPost = module.RobotPost
    dof = int(job.get('dof') or 6)
    axes_type = job.get('axes_type') or ['R'] * dof
    robot = job.get('robot') or 'Robot'
    kw = dict(axes_type=axes_type, native_name=job.get('native_name') or robot, ip_com='127.0.0.1', api_port=20500, prog_ptr=0, robot_ptr=0)
    for k in ('pose_turntable', 'pose_rail', 'lines_x_prog'):
        if job.get(k) is not None:
            kw[k] = job[k]
    for args, kwargs in (((job.get('post_name') or module.__name__, robot, dof), kw), ((job.get('post_name') or module.__name__, robot, dof), {'axes_type': axes_type}),
                         ((job.get('post_name') or module.__name__, robot, dof), {}), ((), {})):
        try:
            return RobotPost(*args, **kwargs)
        except TypeError:
            continue
    return RobotPost()


def _replay(post, prog, Mat):
    def mat(v):
        r = _rows(v)
        return Mat(r) if r is not None else None

    def conf(ev, key='conf'):
        c = ev.get(key)
        return list(c) if c else [0, 0, 0]

    post.ProgStart(prog['name'])
    for c in prog.get('header_comments') or []:
        post.RunMessage(c, True)
    for ev in prog['events']:
        k = ev.get('kind')
        if k == 'moveJ':
            post.MoveJ(mat(ev.get('pose')), ev.get('joints') or [], conf(ev))
        elif k == 'moveL':
            post.MoveL(mat(ev.get('pose')), ev.get('joints') or [], conf(ev))
        elif k == 'moveC':
            post.MoveC(mat(ev.get('via')), ev.get('viaJoints') or ev.get('joints') or [], mat(ev.get('pose')), ev.get('joints') or [], conf(ev, 'viaConf') if ev.get('viaConf') else conf(ev), conf(ev))
        elif k == 'setFrame':
            fid = ev.get('frameId')
            post.setFrame(mat(ev.get('pose')), -1 if fid is None else fid, ev.get('name', 'Frame'))
        elif k == 'setTool':
            tid = ev.get('toolNum')
            post.setTool(mat(ev.get('pose')), -1 if tid is None else tid, ev.get('name', 'Tool'))
        elif k == 'setSpeed':
            if ev.get('speedLinear') is not None:
                post.setSpeed(ev['speedLinear'])
            if ev.get('speedJoints') is not None and hasattr(post, 'setSpeedJoints'):
                post.setSpeedJoints(ev['speedJoints'])
            if ev.get('accelLinear') is not None and hasattr(post, 'setAcceleration'):
                post.setAcceleration(ev['accelLinear'])
            if ev.get('accelJoints') is not None and hasattr(post, 'setAccelerationJoints'):
                post.setAccelerationJoints(ev['accelJoints'])
        elif k == 'setRounding':
            post.setZoneData(ev.get('radius', -1))
        elif k == 'pause':
            post.Pause(ev.get('timeMs', 0))
        elif k == 'setDO':
            post.setDO(_io(ev.get('io')), _value(ev.get('value')))
        elif k == 'setAO' and hasattr(post, 'setAO'):
            post.setAO(_io(ev.get('io')), ev.get('value'))
        elif k == 'waitDI':
            post.waitDI(_io(ev.get('io')), _value(ev.get('value')), ev.get('timeoutMs', -1))
        elif k == 'runCode':
            post.RunCode(ev.get('code', ''), bool(ev.get('isCall')))
        elif k == 'comment':
            post.RunMessage(ev.get('text', ''), True)
        elif k == 'message':
            post.RunMessage(ev.get('text', ''), False)
        elif k == 'callProgram':
            post.RunCode(ev.get('name', ''), True)
        elif k == 'thread':
            post.RunCode(ev.get('name', ''), True)
        elif k == 'loop':
            post.RunMessage('loop %s: %s' % (ev.get('count'), ev.get('body')), True)
            post.RunCode(ev.get('body', ''), True)
        elif k == 'if':
            post.RunMessage('if %s: %s else %s' % (ev.get('condition'), ev.get('then'), ev.get('else')), True)
        elif k == 'gripper':
            # attach/detach are simulation events: RoboDK writes nothing for them; keep a trace as a comment
            post.RunMessage(ev.get('text') or ('gripper ' + ('close' if ev.get('close') else 'open')), True)
        elif k == 'navigate':
            post.RunMessage('NAVIGATE X %.3f Y %.3f' % (ev.get('x', 0), ev.get('y', 0)), True)
        elif k == 'task':
            post.RunMessage('TASK %s' % ev.get('task', ''), True)
        else:
            post.RunMessage('%s %s' % (k, json.dumps({kk: v for kk, v in ev.items() if kk != 'kind'})), True)
    post.ProgFinish(prog['name'])


def _collect(folder):
    files = []
    for root, _dirs, names in os.walk(folder):
        for n in sorted(names):
            p = os.path.join(root, n)
            rel = os.path.relpath(p, folder).replace(os.sep, '/')
            data = open(p, 'rb').read()
            try:
                files.append({'name': rel, 'content': data.decode('utf-8'), 'encoding': 'utf-8'})
            except UnicodeDecodeError:
                try:
                    txt = data.decode('cp1252')
                    if any(ord(ch) < 9 for ch in txt):
                        raise ValueError
                    files.append({'name': rel, 'content': txt, 'encoding': 'cp1252'})
                except Exception:
                    files.append({'name': rel, 'content': base64.b64encode(data).decode('ascii'), 'encoding': 'base64'})
    return files


def _merge(src_dir, out_dir):
    for root, _dirs, names in os.walk(src_dir):
        for n in names:
            src = os.path.join(root, n)
            dst = os.path.join(out_dir, os.path.relpath(src, src_dir))
            os.makedirs(os.path.dirname(dst), exist_ok=True)
            shutil.copyfile(src, dst)


def include_sub_programs(module, job):
    inc = job.get('include_sub_programs')
    return getattr(module.RobotPost, 'INCLUDE_SUB_PROGRAMS', True) if inc is None else bool(inc)


def _run_group(module, job, group, folder, Mat, logs):
    post = _make_post(module, job)
    for k, v in (job.get('post_attrs') or {}).items():
        setattr(post, k, v)
    for p in group:
        _replay(post, p, Mat)
    try:
        post.ProgSave(folder, group[0]['name'], False, False)
    except TypeError:
        post.ProgSave(folder, group[0]['name'])
    plog = getattr(post, 'LOG', '')
    if isinstance(plog, list):
        plog = '\n'.join(plog)
    if plog:
        logs.append(plog)
    if not _collect(folder):
        prog = getattr(post, 'PROG', '')
        if isinstance(prog, list):
            prog = '\n'.join(prog)
        if prog:
            open(os.path.join(folder, '%s.%s' % (group[0]['name'], getattr(post, 'PROG_EXT', 'txt'))), 'w', encoding='utf-8').write(prog)


def run_job(module, job, out_dir, spawn=None):
    """Replay the main program and its sub-programs as RoboDK does and ProgSave into out_dir. Posts with
    INCLUDE_SUB_PROGRAMS = True receive every program in one instance (one ProgSave); otherwise every program is
    generated on its own in a clean folder, in a fresh process when `spawn(sub_job, folder)` is given (RoboDK runs
    each generation in a new Python process and some posts keep state in class attributes). Returns {files, log, stdout}."""
    Mat = _mat_class()
    os.makedirs(out_dir, exist_ok=True)
    log = io.StringIO()
    progs = [{'name': job['program'], 'events': job.get('events') or [], 'header_comments': job.get('header_comments')}]
    progs += [{'name': s['name'], 'events': s.get('events') or [], 'header_comments': job.get('header_comments')} for s in (job.get('subprograms') or [])]
    logs = []
    include = include_sub_programs(module, job)
    with contextlib.redirect_stdout(log):
        if include or len(progs) == 1:
            _run_group(module, job, progs, out_dir.replace(os.sep, '/'), Mat, logs)
        else:
            for p in progs:
                tmp = tempfile.mkdtemp(prefix='vbs_post_')
                sub_job = dict(job, program=p['name'], name=p['name'], events=p['events'], subprograms=[], include_sub_programs=True)
                if spawn:
                    res = spawn(sub_job, tmp)
                    if res.get('log'):
                        logs.append(res['log'])
                    if not res.get('ok'):
                        raise RuntimeError(res.get('error'))
                else:
                    _run_group(module, sub_job, [p], tmp.replace(os.sep, '/'), Mat, logs)
                _merge(tmp, out_dir)
                shutil.rmtree(tmp, ignore_errors=True)
    return {'ok': True, 'files': _collect(out_dir), 'log': '\n'.join(logs), 'stdout': log.getvalue(), 'include_sub_programs': include}


def run_job_source(post_source, job_json, out_dir='/studio/out', name='robodk_post'):
    """Pyodide entry point: returns a JSON string."""
    import shutil
    try:
        shutil.rmtree(out_dir)
    except Exception:
        pass
    job = json.loads(job_json) if isinstance(job_json, str) else job_json
    try:
        module = load_post_source(post_source, name)
        return json.dumps(run_job(module, job, out_dir))
    except Exception as e:
        return json.dumps({'ok': False, 'error': '%s: %s' % (type(e).__name__, e), 'trace': traceback.format_exc(), 'files': []})


# ---- legacy single-text API (kept for older callers) -------------------------------------------------------------------
def run_from_json(post_source, events_json, robot_name='Robot', program_name='Program', dof=6):
    events = json.loads(events_json) if isinstance(events_json, str) else events_json
    job = {'program': program_name, 'robot': robot_name, 'dof': dof, 'events': events}
    res = json.loads(run_job_source(post_source, job, '/tmp/vbs_post_out' if os.name != 'nt' else os.path.join(os.environ.get('TEMP', '.'), 'vbs_post_out')))
    if not res.get('ok'):
        raise RuntimeError(res.get('error'))
    main = res['files'][0] if res['files'] else {'name': 'program.txt', 'content': ''}
    return {'program': main['content'], 'log': res.get('log', ''), 'extension': main['name'].rsplit('.', 1)[-1], 'files': res['files']}


def main(argv):
    if len(argv) < 4:
        print(__doc__)
        return 1
    post, job_path, out_dir = argv[1], argv[2], argv[3]
    posts_dir = None
    if '--posts' in argv:
        posts_dir = argv[argv.index('--posts') + 1]
    with open(job_path, 'r', encoding='utf-8') as f:
        job = json.load(f)

    def spawn(sub_job, folder):
        jp = os.path.normpath(folder) + '.job.json'
        with open(jp, 'w', encoding='utf-8') as fj:
            json.dump(sub_job, fj)
        cmd = [sys.executable, os.path.abspath(__file__), post, jp, folder] + (['--posts', posts_dir] if posts_dir else [])
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=600)
        os.remove(jp)
        try:
            return json.loads(r.stdout[r.stdout.index('{'):])
        except Exception:
            return {'ok': False, 'error': (r.stdout + r.stderr)[-2000:]}

    try:
        with contextlib.redirect_stdout(io.StringIO()):
            module = load_post_native(post, posts_dir)
        res = run_job(module, job, out_dir, spawn)
    except Exception as e:
        res = {'ok': False, 'error': '%s: %s' % (type(e).__name__, e), 'trace': traceback.format_exc(), 'files': []}
    sys.stdout.write(json.dumps(res))
    sys.stdout.flush()
    return 0 if res.get('ok') else 2


if __name__ == '__main__':
    sys.exit(main(sys.argv))
