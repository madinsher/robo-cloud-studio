"""Persistent CPython worker for station Python programs (driven by src/api/runner_cpython.ts).

RoboDK runs a station Python program (a PROGRAM_PYTHON item, or a program call such as ``SprayOn(1)``) as a new
Python process with the arguments in sys.argv. Starting CPython and importing the API costs about a second per call,
which is too slow for programs that call a script on every instruction (3D printing, dispensing). This worker keeps
the interpreter (and the imported drop-in) alive and runs one script at a time in a fresh ``__main__`` namespace:

    stdin  (JSON lines): {"id", "file", "code", "args", "url", "cwd"}
    stdout (JSON lines): {"t": "out", "id", "stream", "s"}   script output
                         {"t": "done", "id", "code", "error"} exit code (SystemExit honoured), traceback on error
                         {"t": "ready"}                       after start-up

Scripts behave as in their own process: sys.argv, __name__ == '__main__', quit()/exit()/sys.exit(code), the threads
they start are joined before the run is reported done, and the Robolink connections they leave open are closed.
"""
import sys
import os
import io
import json
import gc
import threading
import traceback
import builtins

# run as a file, sys.path[0] is python/robodk: `import robolink` would then load robodk/robolink.py as a top-level
# module. Put the drop-in root (python/) there instead.
_here = os.path.dirname(os.path.abspath(__file__))
sys.path = [p for p in sys.path if os.path.abspath(p or '.') != _here]
if os.path.dirname(_here) not in sys.path:
    sys.path.insert(0, os.path.dirname(_here))

_real_out = sys.stdout
_real_err = sys.stderr
_lock = threading.Lock()


def _send(msg):
    with _lock:
        _real_out.write(json.dumps(msg) + '\n')
        _real_out.flush()


class _Pipe(io.TextIOBase):
    def __init__(self, rid, stream):
        self.rid, self.stream = rid, stream

    def write(self, s):
        if s:
            _send({'t': 'out', 'id': self.rid, 'stream': self.stream, 's': s})
        return len(s)

    def flush(self):
        pass

    def isatty(self):
        return False

    @property
    def encoding(self):
        return 'utf-8'


def _quit(code=None):
    # site.Quitter closes sys.stdin (our command channel): raise SystemExit only
    raise SystemExit(code)


builtins.quit = _quit
builtins.exit = _quit

try:  # warm up: the API import is what makes a fresh process slow
    import robodk.robolink as _rl  # noqa: F401
    import robodk.robomath  # noqa: F401
    import robolink  # noqa: F401  (legacy alias module)
except Exception:  # pragma: no cover - reported by the first script instead
    _rl = None


def _close_links(g):
    for v in list(g.values()):
        try:
            if _rl is not None and isinstance(v, _rl.Robolink):
                v.Disconnect()
        except Exception:
            pass


def run(req):
    rid = req['id']
    sys.argv = [req.get('file', 'script.py')] + [str(a) for a in req.get('args', [])]
    url = req.get('url')
    if url:
        os.environ['STUDIO_URL'] = url
        if _rl is not None:
            _rl.DEFAULT_URL = url
    cwd = req.get('cwd')
    if cwd and os.path.isdir(cwd):
        os.chdir(cwd)
        if cwd not in sys.path:
            sys.path.insert(0, cwd)
    before = set(threading.enumerate())
    sys.stdout = _Pipe(rid, 'stdout')
    sys.stderr = _Pipe(rid, 'stderr')
    code, err = 0, ''
    g = {'__name__': '__main__', '__file__': req.get('file', 'script.py'), '__builtins__': builtins}
    try:
        exec(compile(req['code'], req.get('file', 'script.py'), 'exec'), g)
        # a process exits after its non-daemon threads: wait for the ones the script started
        for t in list(threading.enumerate()):
            if t not in before and not t.daemon and t is not threading.current_thread():
                t.join()
    except SystemExit as e:
        c = e.code
        if c is None:
            code = 0
        elif isinstance(c, int):
            code = c
        else:
            code, err = 1, str(c)
            sys.stderr.write(str(c) + '\n')
    except BaseException:
        code = 1
        err = traceback.format_exc()
        sys.stderr.write(err)
    finally:
        sys.stdout, sys.stderr = _real_out, _real_err
        _close_links(g)
        g.clear()
        # the headless tkinter keeps per-script state (buttons, mainloop done): fresh module for the next script
        for m in [m for m in sys.modules if m == 'tkinter' or m.startswith('tkinter.')]:
            if '_headless_gui' in (getattr(sys.modules[m], '__file__', '') or ''):
                del sys.modules[m]
        gc.collect()
    _send({'t': 'done', 'id': rid, 'code': code, 'error': err[-4000:]})


def main():
    _send({'t': 'ready'})
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        try:
            req = json.loads(line)
        except ValueError:
            continue
        if req.get('quit'):
            break
        run(req)


if __name__ == '__main__':
    main()
