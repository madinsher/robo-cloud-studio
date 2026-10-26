"""Threads for station Python programs in the browser (Pyodide).

Pyodide has no OS threads: ``threading.Thread.start()`` raised "can't start new thread", so RoboDK scripts that drive
several robots from threads (Library 08.a "Synchronize 3 Robots (Script matching timings)": one ``MoveRobot`` thread
per robot, synchronised with a lock and a busy-wait on a shared counter) could not run. With WebAssembly JSPI
(cooperative runner of src/api/runner_pyodide.ts) a thread is a coroutine of the one interpreter:

* ``Thread.start()`` runs ``run()`` on its own suspendable stack (a promising call started on the next macrotask);
  every API call that waits and every ``time.sleep()`` / ``pause()`` lets the other threads, the other scripts and the
  page run;
* ``join(timeout)``, ``is_alive()``, ``daemon`` behave like CPython's: the script ends after its non-daemon threads,
  daemon threads stop at their next API call or pause once the script has ended;
* ``threading.Lock`` / ``Condition`` / ``Event`` / ``Queue`` wait cooperatively when contended.

Without JSPI (synchronous runner) ``start()`` still refuses, with a message that names the alternatives.
``install(globals())`` is called by the runner's bootstrap after it defined ``__studio_run`` / ``_studio_rid``.
"""
import sys
import time
import threading
import traceback
import builtins


def install(g):
    try:
        from pyodide.ffi import run_sync, can_run_sync, create_proxy, to_js
        import js
    except Exception:
        return False
    spawn_js = js.Function.new('f', 'return new Promise((res) => setTimeout(() => { let p; try { p = f.callPromising(); } catch (e) { res(null); return; } Promise.resolve(p).then(() => res(null), () => res(null)); }, 0));')
    sleep_js = js.Function.new('s', 'return new Promise((res) => setTimeout(res, Math.max(0, s * 1000)));')

    def coop_wait(seconds):
        run_sync(sleep_js(seconds))

    # -- cooperative locks: contention waits for the holder (another coroutine) instead of blocking the page
    class CoopLock(object):
        def __init__(self):
            self._locked = False

        def acquire(self, blocking=True, timeout=-1):
            if not self._locked:
                self._locked = True
                return True
            if not blocking:
                return False
            if not can_run_sync():
                raise RuntimeError('lock held by another thread: this browser cannot switch threads (no WebAssembly JSPI)')
            end = None if timeout is None or timeout < 0 else time.monotonic() + timeout
            while self._locked:
                if end is not None and time.monotonic() >= end:
                    return False
                coop_wait(0.002)
            self._locked = True
            return True

        def release(self):
            if not self._locked:
                raise RuntimeError('release unlocked lock')
            self._locked = False

        def locked(self):
            return self._locked

        def __enter__(self):
            self.acquire()
            return self

        def __exit__(self, *a):
            self.release()

        def _at_fork_reinit(self):
            self._locked = False

    threading.Lock = CoopLock
    threading._allocate_lock = CoopLock
    threading.RLock = threading._PyRLock

    orig_run = g['__studio_run']
    rid_of = g['_studio_rid']
    threads_of = {}   # run id -> green threads started by that script

    def __studio_run(code, file, argv, run_id=0, thread=None):
        # the script (thread None) or one of its threads; _studio_rid() finds the run id in these frames
        try:
            return orig_run(code, file, argv, run_id)
        finally:
            if thread is None:
                # like a CPython process: the script ends after its non-daemon threads; daemon threads are stopped
                for t in list(threads_of.get(run_id, [])):
                    if not t.daemon and t.is_alive():
                        try:
                            run_sync(t._studio_promise)
                        except BaseException:
                            pass
                for t in threads_of.pop(run_id, []):
                    t._studio_kill = True
    g['__studio_run'] = __studio_run

    def current_thread_obj():
        f = sys._getframe(1)
        code = __studio_run.__code__
        while f is not None:
            if f.f_code is code:
                return f.f_locals.get('thread')
            f = f.f_back
        return None

    def check_killed():
        t = current_thread_obj()
        if t is not None and getattr(t, '_studio_kill', False):
            raise SystemExit()

    rl = sys.modules.get('robodk.robolink')
    if rl is not None:
        orig_transport = rl._TRANSPORT

        def transport(text):
            check_killed()
            return orig_transport(text)
        rl._TRANSPORT = transport
    orig_sleep = time.sleep

    def sleep(seconds):
        check_killed()
        return orig_sleep(seconds)
    time.sleep = sleep

    seq = [0]

    def start(self):
        if getattr(self, '_studio_started', False):
            raise RuntimeError('threads can only be started once')
        if not can_run_sync():
            raise RuntimeError("can't start new thread: station scripts with threads need a browser with WebAssembly JSPI (Chrome / Edge 137+) or the studio server (CPython)")
        rid = rid_of()
        self._studio_started = True
        self._studio_done = False
        self._studio_kill = False
        seq[0] += 1
        key = '__studio_thread_%d' % seq[0]
        me = self

        def body():
            try:
                me.run()
            except SystemExit:
                pass
            except BaseException:
                sys.stderr.write('Exception in thread %s:\n%s' % (me.name, traceback.format_exc()))
            finally:
                me._studio_done = True
        setattr(builtins, key, body)
        code = 'import builtins as _b\n_b.__dict__.pop(%r)()\n' % key
        argv = list(sys.argv[1:])
        file = sys.argv[0] if sys.argv else 'thread.py'

        def runner():
            __studio_run(code, file, argv, rid, me)
        self._studio_proxy = create_proxy(runner)
        self._studio_promise = spawn_js(self._studio_proxy)
        threads_of.setdefault(rid, []).append(self)
        try:
            self._started.set()
        except Exception:
            pass

    def join(self, timeout=None):
        if not getattr(self, '_studio_started', False):
            raise RuntimeError('cannot join thread before it is started')
        if self._studio_done:
            return
        p = self._studio_promise
        if timeout is not None:
            p = js.Promise.race(to_js([p, sleep_js(float(timeout))]))
        run_sync(p)

    def is_alive(self):
        return bool(getattr(self, '_studio_started', False)) and not getattr(self, '_studio_done', True)

    threading.Thread.start = start
    threading.Thread.join = join
    threading.Thread.is_alive = is_alive
    return True
