/**
 * Python control runtime embedded in the generated RoboDK station scripts: a port of the studio executors
 * (ctl/exec.ts AutomatonExecutor / PetriExecutor, ctl/fsm.ts FsmExecutor, ctl/bt.ts + runtime.ts ControlRuntime) over
 * the robolink API, so the script reproduces the studio run against RoboDK or the studio's RoboDK-compatible server:
 * programs start with RunProgram (non-blocking; robots run concurrently), one action per robot at a time, moves with
 * MoveJ / MoveL(blocking=False), signals are station parameters (setParam / getParam), events and the blackboard are local.
 * Every firing / outcome is printed as `TRACE<TAB>time<TAB>kind<TAB>label` so runs can be compared.
 * Python 3.6+ (RoboDK ships 3.7), no dependency besides robodk.
 */
/** robolink import of the RoboDK station script (the ROS 2 package replaces it by its own link, see ros2.ts). */
export const PY_ROBODK_IMPORT = String.raw`
try:
    from robodk.robolink import Robolink, ITEM_TYPE_PROGRAM, ITEM_TYPE_ROBOT, ITEM_TYPE_TARGET
except ImportError:  # RoboDK < 5.4
    from robolink import Robolink, ITEM_TYPE_PROGRAM, ITEM_TYPE_ROBOT, ITEM_TYPE_TARGET
`;
export const PY_RUNTIME = String.raw`
import json, math, sys, time
` + PY_ROBODK_IMPORT + String.raw`
DT = 0.05
T0 = None
CLOCK = [None]


class Clock(object):
    """The run clock = the station's simulation time: RoboDK's SimulationTime() follows the simulation speed, so
    delay= / wait= / timeout= keep their ratio to the program durations at any speed and however long one tick (one
    robolink round trip per Busy() call) takes. A link without that call (stand-ins, the ROS link) runs on wall time
    times its SimulationSpeed() (1 when it has none)."""

    def __init__(self, rdk):
        self.rdk, self.sim, self.speed = rdk, False, 1.0
        f = getattr(rdk, 'SimulationTime', None)
        if callable(f):
            try:
                float(f())
                self.sim = True
            except Exception:
                self.sim = False
        s = getattr(rdk, 'SimulationSpeed', None)
        try:
            self.speed = float(s()) if callable(s) else 1.0
        except Exception:
            self.speed = 1.0
        if not self.speed > 0:
            self.speed = 1.0
        self.t0 = self.raw()
        self.last = (time.time(), 0.0)

    def raw(self):
        if self.sim:
            return float(self.rdk.SimulationTime())
        return time.time() * self.speed

    def __call__(self):
        """Read the clock (one robolink round trip on RoboDK)."""
        t = self.raw() - self.t0
        self.last = (time.time(), t)
        return t

    def est(self):
        """The last reading advanced by the wall time since, at the simulation speed (no round trip)."""
        w, t = self.last
        return t + (time.time() - w) * self.speed


def start_clock(rdk):
    global T0
    T0 = time.time()
    CLOCK[0] = Clock(rdk)
    return CLOCK[0]


def now():
    """Run time (simulation seconds) without a robolink round trip: traces, Busy() observation times."""
    return CLOCK[0].est() if CLOCK[0] is not None else time.time() - T0


def read_clock():
    """Run time read from the station (the time of a control step)."""
    return CLOCK[0]() if CLOCK[0] is not None else time.time() - T0


def trace(kind, label):
    print('TRACE\t%.2f\t%s\t%s' % (now(), kind, label))
    sys.stdout.flush()


def _sign(x):
    return (x > 0) - (x < 0)


def _num(v):
    if isinstance(v, (int, float, bool)):
        return v
    s = str(v).strip()
    if s.lower() in ('true', 'false'):
        return s.lower() == 'true'
    try:
        return float(s)
    except ValueError:
        return s


def evaluate(expr, bb):
    if not expr:
        return True
    return eval(expr, {'V': lambda k: bb.get(k, 0), 'math': math, '_sign': _sign, '__builtins__': {'abs': abs, 'min': min, 'max': max}})


class World(object):
    """robolink actions, robot locks (one action per robot, FIFO), signals = station parameters, local events."""

    def __init__(self, rdk, signals):
        self.rdk = rdk
        self.signals = signals
        self.bb = {}
        self.events = []
        self.owner = {}
        self.waiting = {}
        self.stale = False

    def sense(self):
        allp = getattr(self.rdk, 'getParams', None)
        if self.signals and callable(allp):
            # one round trip for every station parameter (and no 'unknown parameter' warning per tick for a
            # signal no program has set yet, as getParam prints)
            try:
                vals = dict((str(k), v) for k, v in allp())
            except Exception:
                vals = None
            if vals is not None:
                for n in self.signals:
                    v = vals.get(n)
                    if v is not None and v != '':
                        self.bb[n] = _num(v)
                self.stale = False
                return self.bb
        for n in self.signals:
            v = self.rdk.getParam(n)
            if v is not None and v != '':
                self.bb[n] = _num(v)
        self.stale = False
        return self.bb

    def fresh(self):
        """The blackboard for a guard / condition / decision: signals are read again when a program or move ended
        since the last read (its outputs, e.g. SetDO Ready, are set by the time it reports done - as in the studio,
        where the program step runs before the control tick)."""
        if self.stale:
            self.sense()
        return self.bb

    def acquire(self, robot, key):
        o = self.owner.get(robot)
        if o == key:
            return True
        q = self.waiting.setdefault(robot, [])
        if o is None and (not q or q[0] == key):
            if q and q[0] == key:
                q.pop(0)
            self.owner[robot] = key
            return True
        if key not in q:
            q.append(key)
        return False

    def release(self, key):
        for r, o in list(self.owner.items()):
            if o == key:
                del self.owner[r]
        for q in self.waiting.values():
            if key in q:
                q.remove(key)

    def set_signal(self, name, value):
        self.rdk.setParam(name, value)
        self.bb[name] = value


_seq = [0]


def _sub(v, bb):
    return str(bb[v[1:]]) if isinstance(v, str) and v.startswith('$') and v[1:] in bb else v


def named_outcomes(s):
    """Outcome list of a skill state with several outcomes (do skill=... outcomes=a,b,c), else None."""
    o = ((s.get('action') or {}).get('args') or {}).get('outcomes')
    return [x.strip() for x in str(o).split(',') if x.strip()] if o else None


class Action(object):
    def __init__(self, world, b, t):
        if str(b.get('value', '')).startswith('$') or any(isinstance(v, str) and v.startswith('$') for v in b.get('args', {}).values()):
            b = dict(b, value=_sub(b['value'], world.bb), args=dict((k, _sub(v, world.bb)) for k, v in b.get('args', {}).items()))  # $name = blackboard value
        _seq[0] += 1
        self.w, self.b, self.key, self.t0, self.state = world, b, 'a%d' % _seq[0], t, 'init'
        self.result = None
        self.outcome = None   # named outcome reported by the skill (item.Outcome()), if any
        self.timed_out = False
        self.busy_at = t      # run time of the last Busy() call that saw the program / move running (it ends later)
        self.done_at = t      # run time after the Busy() call that saw it finished (it ended before)

    def tick(self, t):
        b, w = self.b, self.w
        if b.get('timeout') is not None and t - self.t0 >= b['timeout'] - 1e-9:
            self.halt()
            self.timed_out = True
            return 'failure'
        k = b['kind']
        if k == 'program' or k == 'skill' or k == 'goto':
            name = b['value'] if k == 'program' else ('goto_' + b['value'] if k == 'goto' else b['value'])
            if self.state == 'init':
                self.item = w.rdk.Item(name, ITEM_TYPE_PROGRAM)
                if not self.item.Valid():
                    print('program %s not found' % name)
                    return 'failure'
                r = self.item.getLink(ITEM_TYPE_ROBOT)
                self.robot = r.Name() if r.Valid() else 'prog:' + name
                self.state = 'wait'
            if self.state == 'wait':
                if not w.acquire(self.robot, self.key):
                    return 'running'
                self.item.RunProgram()
                trace('program', name)
                self.state = 'run'
                return 'running'
            t_call = now()
            if self.item.Busy():
                self.busy_at = t_call
                return 'running'
            self.done_at = now()
            w.release(self.key)
            w.stale = True
            oc = getattr(self.item, 'Outcome', None)
            self.outcome = oc() if callable(oc) else None
            failed = getattr(self.item, 'Failed', None)
            return 'failure' if failed is not None and failed() else 'success'
        if k == 'target':
            if self.state == 'init':
                self.target = w.rdk.Item(b['value'], ITEM_TYPE_TARGET)
                if not self.target.Valid():
                    print('target %s not found' % b['value'])
                    return 'failure'
                self.robot = w.rdk.Item(b['robot'], ITEM_TYPE_ROBOT) if b.get('robot') else self.target.getLink(ITEM_TYPE_ROBOT)
                if not self.robot.Valid():
                    print('no robot for target %s' % b['value'])
                    return 'failure'
                self.state = 'wait'
            if self.state == 'wait':
                if not w.acquire(self.robot.Name(), self.key):
                    return 'running'
                (self.robot.MoveL if b['args'].get('linear') in (True, 'true') else self.robot.MoveJ)(self.target, False)
                trace('move', b['value'])
                self.state = 'run'
                return 'running'
            t_call = now()
            if self.robot.Busy():
                self.busy_at = t_call
                return 'running'
            self.done_at = now()
            w.release(self.key)
            w.stale = True
            failed = getattr(self.robot, 'Failed', None)
            return 'failure' if failed is not None and failed() else 'success'
        if k == 'signal':
            v = b['args'].get('value', True)
            w.set_signal(b['value'], v)
            return 'success'
        if k == 'wait':
            return 'success' if t - self.t0 >= float(b['value']) - 1e-9 else 'running'
        if k == 'set':
            v = b['args'].get('value', True)
            w.bb[b['value']] = evaluate(b['py'], w.bb) if b.get('py') else v
            return 'success'
        if k == 'await':
            return 'success' if evaluate(b['py'], w.fresh()) else 'running'
        if k == 'check':
            return 'success' if evaluate(b['py'], w.fresh()) else 'failure'
        if k == 'event':
            w.events.append(b['value'])
            return 'success'
        print('unsupported action %s' % k)
        return 'failure'

    def halt(self):
        if self.state == 'run':
            try:
                (self.item if hasattr(self, 'item') else self.robot).Stop()
            except Exception:
                pass
        self.w.release(self.key)


class Supervisor(object):
    def __init__(self, table):
        self.states = dict((s['id'], s) for s in table['states'])
        self.uc = set(table['uncontrollable'])
        self.uo = set(table['unobservable'])
        self.state = table['initial']

    def allowed(self, e):
        s = self.states[self.state]
        return e in s['enabled_controllable'] or (e in self.uc and e in s['next'])

    def observe(self, e):
        if e in self.uo:
            return False
        nxt = self.states[self.state]['next'].get(e)
        if nxt is None:
            raise ValueError('event %s impossible in supervisor state %s' % (e, self.state))
        self.state = nxt
        return True

    def marked(self):
        return self.states[self.state]['marked']


class Sensors(object):
    def __init__(self, specs):
        self.specs, self.prev = specs, {}

    def poll(self, bb):
        out = []
        for ev, expr in self.specs:
            v = bool(evaluate(expr, bb))
            p = self.prev.get(ev + '|' + expr)
            if v and p is False:
                out.append(ev)
            self.prev[ev + '|' + expr] = v
        return out


class AutomatonRun(object):
    def __init__(self, m, w):
        self.m, self.w = m, w
        self.state = dict((c['name'], c['x0']) for c in m['comps'])
        self.settled, self.blocked, self.running, self.last = set(), set(), {}, {}
        self.uc = set(m['uc'])
        self.sup = Supervisor(m['supervisor']) if m.get('supervisor') else None
        self.sensors = Sensors(m.get('sensors', []))
        self.status, self.verdict, self.time, self.ticks = 'running', 'running', 0.0, 0
        w.bb.update(w.sense())
        w.bb.update(m.get('bb', {}))
        for c in m['comps']:
            self.enter(c)

    def comp(self, name):
        for c in self.m['comps']:
            if c['name'] == name:
                return c

    def nxt(self, c, x, e):
        for ev, to in c['delta'].get(x, []):
            if ev == e:
                return to
        return None

    def action_for(self, c, x):
        return self.m['actions'].get(c['name'] + '.' + x)

    def enter(self, c):
        x = self.state[c['name']]
        b = self.action_for(c, x)
        self.settled.discard(c['name'])
        self.blocked.discard(c['name'])
        if not b:
            self.settled.add(c['name'])
            return
        self.running[c['name']] = Action(self.w, b, self.time)

    def fire(self, e, src):
        parts = [c for c in self.m['comps'] if e in c['sigma']]
        if not parts:
            print('event %s belongs to no component' % e)
            return False
        for c in parts:
            if self.nxt(c, self.state[c['name']], e) is None:
                print('model mismatch: %s (%s) not enabled in %s=%s' % (e, src, c['name'], self.state[c['name']]))
                return False
        if self.sup:
            try:
                self.sup.observe(e)
            except ValueError as err:
                print('supervisor: %s' % err)
        for c in parts:
            a = self.running.pop(c['name'], None)
            if a:
                a.halt()
            self.state[c['name']] = self.nxt(c, self.state[c['name']], e)
        self.last[e] = self.ticks
        trace('event', e)
        for c in parts:
            self.enter(c)
        return True

    def enabled_auto(self):
        out = []
        for c in self.m['comps']:
            if c['name'] not in self.settled:
                continue
            for e, _to in c['delta'].get(self.state[c['name']], []):
                if e in self.uc or e in out:
                    continue
                if not all(e not in d['sigma'] or (d['name'] in self.settled and self.nxt(d, self.state[d['name']], e) is not None) for d in self.m['comps']):
                    continue
                g = self.m['guards'].get(e)
                if g and not evaluate(g, self.w.fresh()):
                    continue
                if self.sup and not self.sup.allowed(e):
                    continue
                out.append(e)
        return out

    def waiting_for_world(self):
        return any(c['name'] not in self.blocked and any(e in self.uc for e, _ in c['delta'].get(self.state[c['name']], [])) for c in self.m['comps'])

    def tick(self, t):
        """One control step at run-clock time t (simulation seconds since the start)."""
        w = self.w
        self.time = t
        w.bb.update(w.sense())
        evs = w.events[:]
        del w.events[:]
        for e in evs + self.sensors.poll(w.bb):
            self.fire(e, 'world')
        for name, a in list(self.running.items()):
            if self.running.get(name) is not a:
                continue
            st = a.tick(self.time)
            if st == 'running':
                continue
            del self.running[name]
            if st == 'failure':
                print('%s: action %s %s failed' % (name, a.b['kind'], a.b['value']))
                if a.b.get('fail'):
                    self.settled.add(name)
                    self.fire(a.b['fail'], 'fail')
                else:
                    self.blocked.add(name)
                continue
            self.settled.add(name)
            if a.b.get('done'):
                self.fire(a.b['done'], 'done')
        evs = self.enabled_auto()
        if evs:
            evs.sort(key=lambda e: self.last.get(e, -1))
            self.fire(evs[0], 'auto')
        self.ticks += 1
        marked = all(self.state[c['name']] in c['marked'] for c in self.m['comps']) and (not self.sup or self.sup.marked())
        if not self.running and not self.enabled_auto() and (not self.waiting_for_world() or (self.m.get('terminate') == 'marked' and marked and self.last and not self.blocked)):
            self.status = 'idle'
            self.verdict = 'failed' if self.blocked else ('done' if marked else 'deadlock')

    def where(self):
        return ' '.join('%s=%s' % (c['name'], self.state[c['name']]) for c in self.m['comps'])


class PetriRun(object):
    def __init__(self, m, w):
        self.m, self.w = m, w
        self.places = m['places']
        self.mk = dict((p['id'], p['tokens']) for p in self.places)
        self.active, self.faulted, self.last = [], set(), {}
        self.sensors = Sensors(m.get('sensors', []))
        self.status, self.verdict, self.time, self.ticks, self.n = 'running', 'running', 0.0, 0, 0
        w.bb.update(w.sense())
        w.bb.update(m.get('bb', {}))

    def enabled(self, t):
        for p, wt in t['pre'].items():
            if self.mk[p] < wt:
                return False
        for p, wt in t['inh'].items():
            if self.mk[p] >= wt:
                return False
        for p in self.places:
            cap = p.get('capacity')
            if cap is not None and t['post'].get(p['id'], 0) > 0 and self.mk[p['id']] - t['pre'].get(p['id'], 0) + t['post'][p['id']] > cap:
                return False
        return True

    def in_progress(self, t):
        return len([a for a in self.active if a[0] is t])

    def candidates(self, evs):
        out = []
        for j, t in enumerate(self.m['transitions']):
            if j in self.faulted or not self.enabled(t) or self.in_progress(t) >= t.get('servers', 1):
                continue
            if t.get('on') and t['on'] not in evs:
                continue
            if t.get('guard') and not evaluate(t['guard'], self.w.fresh()):
                continue
            out.append(j)
        imm = [j for j in out if self.m['transitions'][j].get('immediate')]
        pool = imm or out
        if not pool:
            return []
        top = max(self.m['transitions'][j].get('priority', 0) for j in pool)
        pool = [j for j in pool if self.m['transitions'][j].get('priority', 0) == top]
        pool.sort(key=lambda j: (self.last.get(j, -1), j))
        return pool

    def duration(self, t):
        if t.get('immediate'):
            return 0.0
        if t.get('delay') is not None:
            return float(t['delay'])
        if t.get('rate'):
            import random
            return random.expovariate(t['rate'])
        return 0.0

    def tick(self, now_t):
        """One control step at run-clock time now_t. Firings that end in this step are applied in time order: a
        timer (delay=) that ended while a program was still seen running ends first and its successors fire
        before the program's end is applied - one robolink round trip per Busy() makes a step longer than DT, and
        the race between a timer and a program must not depend on that."""
        w = self.w
        self.time = now_t
        w.bb.update(w.sense())
        evs = w.events[:]
        del w.events[:]
        evs += self.sensors.poll(w.bb)
        ended, progs = [], []
        for a in list(self.active):
            t, act, end = a
            if act is not None:
                st = act.result or act.tick(self.time)
                if st == 'failure':
                    for p, wt in t['pre'].items():
                        self.mk[p] += wt
                    self.active.remove(a)
                    self.faulted.add(self.m['transitions'].index(t))
                    trace('fail', t['id'])
                    continue
                if st == 'success' and self.time >= end - 1e-9:
                    ended.append(a)
                    # the program ended after the last Busy() call that saw it running and before the call that
                    # saw it finished; when slow round trips make that window much longer than DT, its middle is
                    # the estimate (with steps of DT it is the last running observation, as in the studio)
                    seen, gone = act.busy_at, max(act.done_at, act.busy_at)
                    progs.append(max(seen + (gone - seen) / 2.0 if gone - seen > 1.5 * DT else seen, end))
            elif self.time >= end - 1e-9:
                ended.append(a)
        early = min(progs) if progs else None
        first = [a for a in ended if a[1] is None and early is not None and a[2] <= early + 1e-9]
        count = [0]
        pending = list(evs)
        if first:
            for a in first:
                self.finish(a)
            self.fire_enabled(pending, count)
        for a in ended:
            if a not in first:
                self.finish(a)
        self.fire_enabled(pending, count)
        self.ticks += 1
        waits = any(j not in self.faulted and (t.get('on') or t.get('guard')) and self.enabled(t) and self.in_progress(t) < t.get('servers', 1) for j, t in enumerate(self.m['transitions']))
        if not self.active and not self.candidates([]) and not waits:
            self.status = 'idle'
            self.verdict = ('failed' if self.faulted else 'done') if self.is_final() else ('failed' if self.faulted else 'deadlock')

    def finish(self, a):
        t = a[0]
        for p, wt in t['post'].items():
            self.mk[p] += wt
        self.active.remove(a)
        trace('end', t['id'])

    def fire_enabled(self, pending, count):
        w = self.w
        while True:
            en = self.candidates(pending)
            if not en or count[0] >= 20:
                break
            count[0] += 1
            n = count[0]
            j = en[0]
            t = self.m['transitions'][j]
            b = self.m['actions'].get(t['id'])
            act = Action(w, b, self.time) if b else None
            if t.get('on'):
                pending.remove(t['on'])
            for p, wt in t['pre'].items():
                self.mk[p] -= wt
            d = self.duration(t)
            self.n += 1
            self.last[j] = self.ticks * 1000000 + n
            trace('start', t['id'])
            if act is None and d == 0:
                for p, wt in t['post'].items():
                    self.mk[p] += wt
                trace('end', t['id'])
                continue
            self.active.append((t, act, self.time + d))
            if act is not None:
                st = act.tick(self.time)
                if st != 'running':
                    act.result = st

    def is_final(self):
        finals = self.m.get('finals') or []
        if finals:
            return any(all(self.mk[p] == n for p, n in f.items()) for f in finals)
        return all(not self.mk[p['id']] or not p['out'] or p.get('kind') in ('resource', 'idle', 'monitor') for p in self.places)

    def where(self):
        return ' '.join((p['id'] + ('x%d' % self.mk[p['id']] if self.mk[p['id']] > 1 else '')) for p in self.places if self.mk[p['id']])


class FsmRun(object):
    def __init__(self, m, w):
        self.m, self.w = m, w
        self.frames = []
        self.sensors = Sensors(m.get('sensors', []))
        self.status, self.verdict, self.time, self.outcome = 'running', 'running', 0.0, None
        w.bb.update(w.sense())
        w.bb.update(m.get('bb', {}))
        root = m['root']
        self.enter(root, root['name'], self.initial(root))

    def initial(self, mach):
        for s in mach['states']:
            if s.get('initial'):
                return s
        return mach['states'][0]

    def instant(self, b):
        a = Action(self.w, b, self.time)
        if a.tick(self.time) == 'running':
            a.halt()

    def enter(self, mach, path, s):
        f = {'m': mach, 'path': path, 's': s, 't': self.time, 'run': None}
        self.frames.append(f)
        for b in s['entry']:
            self.instant(b)
        if s['kind'] == 'machine':
            self.enter(s['machine'], path + '/' + s['name'], self.initial(s['machine']))
            return
        if s['kind'] == 'skill':
            f['run'] = Action(self.w, s['action'], self.time)

    def resolve(self, i, o):
        while True:
            f = self.frames[i]
            s = f['s']
            for k in range(len(self.frames) - 1, i, -1):
                g = self.frames[k]
                if g['run']:
                    g['run'].halt()
                for b in g['s']['exit']:
                    self.instant(b)
            del self.frames[i + 1:]
            if f['run']:
                f['run'].halt()
            for b in s['exit']:
                self.instant(b)
            to = s['transitions'].get(o)
            if to is None and o == 'timeout':
                to = s['transitions'].get((s.get('action') or {}).get('fail') or 'fail')
            if to is None and o in f['m']['outcomes']:
                to = o
            self.frames.pop()
            if to is None:
                if o in ('canceled', 'timeout'):
                    to = o
                else:
                    trace('outcome', f['path'] + '/' + s['name'] + ':' + o)
                    self.finish(o, 'failed')
                    return
            trace('outcome', f['path'] + '/' + s['name'] + ':' + o)
            nxt = [x for x in f['m']['states'] if x['name'] == to]
            if nxt:
                self.enter(f['m'], f['path'], nxt[0])
                return
            if i == 0:
                self.finish(to, 'stopped' if to == 'canceled' else 'done')
                return
            i -= 1
            o = to

    def finish(self, outcome, verdict):
        self.outcome, self.status, self.verdict = outcome, 'idle', verdict
        del self.frames[:]

    def tick(self, t):
        w = self.w
        self.time = t
        w.bb.update(w.sense())
        evs = w.events[:]
        del w.events[:]
        evs += self.sensors.poll(w.bb)
        w.bb['event'] = evs[-1] if evs else ''
        for n in range(50):
            if self.status != 'running' or not self.frames:
                break
            ti = [k for k, f in enumerate(self.frames) if f['s'].get('timeout') is not None and self.time - f['t'] >= f['s']['timeout'] - 1e-9]
            if ti:
                self.resolve(ti[0], 'timeout')
                continue
            i = len(self.frames) - 1
            f = self.frames[i]
            s = f['s']
            o = None
            if s['kind'] == 'pass':
                o = 'ok'
            elif s['kind'] == 'decision':
                for expr, oc in s['when']:
                    if evaluate(expr, w.fresh()):
                        o = oc
                        break
                if o is None:
                    o = s.get('otherwise')
            elif s['kind'] == 'skill':
                st = f['run'].tick(self.time)
                named = named_outcomes(s)
                if st == 'success':
                    o = f['run'].outcome or (named[0] if named else (s['action'].get('done') or 'ok'))
                elif st == 'failure':
                    o = 'timeout' if f['run'].timed_out else (f['run'].outcome or (named[-1] if named else (s['action'].get('fail') or 'fail')))
            if o is None:
                break
            self.resolve(i, o)

    def where(self):
        return (self.frames[-1]['path'] + '/' + self.frames[-1]['s']['name']) if self.frames else ('= ' + str(self.outcome))


class BtRun(object):
    """Behavior tree with the studio semantics (memory / reactive sequences and fallbacks, parallel with completed-child
    memory, decorators, actions under supervisor event gating)."""

    def __init__(self, m, w):
        self.m, self.w = m, w
        self.root = self.build(m['root'])
        self.sup = Supervisor(m['supervisor']) if m.get('supervisor') else None
        self.status, self.verdict, self.time = 'running', 'running', 0.0
        self.running, self.prev = set(), set()
        w.bb.update(w.sense())
        w.bb.update(m.get('bb', {}))

    def build(self, spec):
        n = dict(spec)
        n['children'] = [self.build(c) for c in spec.get('children', [])]
        n['status'], n['cursor'], n['attempts'], n['start'], n['done'], n['act'] = 'idle', 0, 0, None, {}, None
        return n

    def reset(self, n):
        n['status'], n['cursor'], n['attempts'], n['start'], n['done'] = 'idle', 0, 0, None, {}
        for c in n['children']:
            self.reset(c)

    def halt(self, n):
        if n.get('act'):
            n['act'].halt()
            n['act'] = None
        n['status'], n['start'] = 'idle', None

    def walk(self, n):
        yield n
        for c in n['children']:
            for d in self.walk(c):
                yield d

    def observe(self, e):
        if self.sup:
            try:
                self.sup.observe(e)
            except ValueError as err:
                print('model mismatch: %s' % err)

    def tick_node(self, n):
        t, bb = n['type'], self.w.bb
        if t == 'condition':
            st = 'success' if evaluate(n.get('py'), self.w.fresh()) else 'failure'
        elif t == 'action':
            self.ticked.add(id(n))
            args = n.get('args', {})
            if n['status'] != 'running':
                if n.get('pre_py') and not evaluate(n['pre_py'], self.w.fresh()):
                    n['status'] = 'failure'
                    return 'failure'
                ev = args.get('event')
                if ev and self.sup:
                    if not self.sup.allowed(ev):
                        print('supervisor denied %s' % ev)
                        n['status'] = 'failure'
                        return 'failure'
                    self.observe(ev)
                n['act'] = Action(self.w, n['binding'], self.time) if n.get('binding') else None
                n['start'] = self.time
            if n['act'] is None:
                print('action %s has no binding' % n.get('fn'))
                st = 'failure'
            else:
                st = n['act'].tick(self.time)
            if st == 'running' and n.get('timeout') is not None and self.time - n['start'] >= n['timeout']:
                self.halt(n)
                st = 'failure'
            if st == 'success' and n.get('post_py') and not evaluate(n['post_py'], self.w.fresh()):
                st = 'failure'
            if st == 'success':
                for key in ('ok', 'put', 'arrive'):
                    if args.get(key):
                        self.w.events.append(args[key])
            if st == 'failure' and args.get('miss'):
                self.w.events.append(args['miss'])
            if st == 'success' and args.get('done'):
                self.observe(args['done'])
            if st == 'running':
                self.running.add(id(n))
                self.nodes[id(n)] = n
            else:
                n['act'] = None
        elif t in ('sequence', 'fallback'):
            stop = 'failure' if t == 'sequence' else 'success'
            st = None
            for i in range(n['cursor'] if n.get('memory') else 0, len(n['children'])):
                s = self.tick_node(n['children'][i])
                if s == 'running':
                    if n.get('memory'):
                        n['cursor'] = i
                    st = 'running'
                    break
                if s == stop:
                    n['cursor'] = 0
                    st = stop
                    break
            if st is None:
                n['cursor'] = 0
                st = 'success' if t == 'sequence' else 'failure'
        elif t == 'parallel':
            M = n.get('threshold') or len(n['children'])
            succ = fail = 0
            for i, c in enumerate(n['children']):
                s = n['done'].get(i) or self.tick_node(c)
                if s != 'running':
                    n['done'][i] = s
                if s == 'success':
                    succ += 1
                elif s == 'failure':
                    fail += 1
            st = 'success' if succ >= M else ('failure' if fail > len(n['children']) - M else 'running')
            if st != 'running':
                for c in n['children']:
                    for d in self.walk(c):
                        if id(d) in self.running:
                            self.running.discard(id(d))
                            self.halt(d)
                    self.reset(c)
                n['done'] = {}
        elif t == 'inverter':
            s = self.tick_node(n['children'][0])
            st = 'failure' if s == 'success' else ('success' if s == 'failure' else 'running')
        elif t == 'force_success':
            s = self.tick_node(n['children'][0])
            st = 'running' if s == 'running' else 'success'
        elif t == 'force_failure':
            s = self.tick_node(n['children'][0])
            st = 'running' if s == 'running' else 'failure'
        elif t == 'retry':
            s = self.tick_node(n['children'][0])
            if s == 'failure' and n['attempts'] < (n.get('count') or 1):
                n['attempts'] += 1
                self.reset(n['children'][0])
                st = 'running'
            else:
                if s != 'running':
                    n['attempts'] = 0
                st = s
        elif t == 'repeat':
            s = self.tick_node(n['children'][0])
            if s == 'success' and n['attempts'] < (n.get('count') or 1) - 1:
                n['attempts'] += 1
                self.reset(n['children'][0])
                st = 'running'
            else:
                if s != 'running':
                    n['attempts'] = 0
                st = s
        elif t == 'timeout':
            if n['status'] != 'running':
                n['start'] = self.time
            s = self.tick_node(n['children'][0])
            if s == 'running' and self.time - n['start'] >= (n.get('seconds') or 1):
                for d in self.walk(n['children'][0]):
                    if id(d) in self.running:
                        self.running.discard(id(d))
                        self.halt(d)
                self.reset(n['children'][0])
                st = 'failure'
            else:
                st = s
        else:
            st = 'failure'
        n['status'] = st
        return st

    def tick(self, t):
        w = self.w
        self.time = t
        w.bb.update(w.sense())
        for e in w.events:
            self.observe(e)
        del w.events[:]
        self.prev, self.running, self.ticked, self.nodes = self.running, set(), set(), getattr(self, 'nodes', {})
        st = self.tick_node(self.root)
        for k in self.prev:
            if k not in self.ticked and k in self.nodes:
                self.halt(self.nodes[k])
        if st != 'running' and not (self.m.get('loop') and st == 'success' and not w.bb.get('reported')):
            self.status, self.verdict = 'idle', 'done' if st == 'success' else 'failed'
            trace('tree', st)

    def where(self):
        return self.status


def set_speed(rdk, speed):
    """Simulation speed for the run (the studio runs a model in real time: 1); returns the previous speed, or None
    when the link has no speed call or speed <= 0 (RoboDK's own speed is kept)."""
    get, put = getattr(rdk, 'SimulationSpeed', None), getattr(rdk, 'setSimulationSpeed', None)
    if not (speed > 0) or not callable(put):
        return None
    try:
        old = float(get()) if callable(get) else None
        put(speed)
        return old
    except Exception:
        return None


def main(model):
    """python script.py [SECONDS] [SPEED]: run for at most SECONDS of simulation time at simulation speed SPEED
    (default 1 = real time, as the studio runs it; 0 keeps RoboDK's speed - its start-up value is 5). Delays, waits
    and timeouts run on the station's simulation clock either way."""
    rdk = Robolink()
    world = World(rdk, model.get('signals', []))
    limit = float(sys.argv[1]) if len(sys.argv) > 1 else model.get('limit', 3600)
    speed = float(sys.argv[2]) if len(sys.argv) > 2 else model.get('speed', 1.0)
    old = set_speed(rdk, speed)
    start_clock(rdk)
    run = {'des': AutomatonRun, 'petri': PetriRun, 'fsm': FsmRun, 'bt': BtRun}[model['kind']](model['model'], world)
    try:
        while run.status == 'running' and now() < limit:
            t_tick = time.time()
            run.tick(read_clock())
            time.sleep(max(0.0, DT - (time.time() - t_tick)))
    finally:
        if old is not None and old != speed:
            set_speed(rdk, old)
    print('RESULT\t%s\t%.2f\t%s%s' % (run.verdict if run.status != 'running' else 'timeout', now(), run.where(), ('\t' + str(run.outcome)) if getattr(run, 'outcome', None) else ''))
    return run
`;
