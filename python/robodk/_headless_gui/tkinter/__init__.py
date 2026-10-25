"""Headless tkinter for scripted (unattended) runs of RoboDK station scripts and Library macros.

Only on sys.path when the studio runs scripts with a headless GUI (runner option headlessGui / STUDIO_HEADLESS_GUI=1),
never in a desktop session. Widgets are inert, Tk variables keep their values, and ``mainloop()`` "clicks" one button
(the first one, or the first whose text matches the regular expression in STUDIO_TK_BUTTON), runs the callbacks
scheduled with ``after()``, waits for the threads the click started and returns, like a user who accepts the defaults
and presses the main action button. Dialog helpers (messagebox, filedialog, simpledialog) return their defaults.
"""
import os
import re
import sys
import threading

TkVersion = 8.6
TclVersion = 8.6
_buttons = []
_after = []
_roots = []


class TclError(Exception):
    pass


class Variable(object):
    _default = ''

    def __init__(self, master=None, value=None, name=None):
        self._value = self._default if value is None else value
        self._traces = []

    def get(self):
        return self._value

    def set(self, value):
        self._value = value
        for cb in list(self._traces):
            try:
                cb('', '', 'write')
            except Exception:
                pass

    def trace_add(self, mode, callback):
        self._traces.append(callback)
        return str(len(self._traces))

    def trace(self, mode, callback):
        return self.trace_add(mode, callback)

    trace_variable = trace

    def trace_remove(self, *a):
        pass


class StringVar(Variable):
    _default = ''

    def get(self):
        return str(self._value)


class IntVar(Variable):
    _default = 0

    def get(self):
        try:
            return int(float(self._value))
        except (TypeError, ValueError):
            return 0


class DoubleVar(Variable):
    _default = 0.0

    def get(self):
        try:
            return float(self._value)
        except (TypeError, ValueError):
            return 0.0


class BooleanVar(Variable):
    _default = False

    def get(self):
        return bool(self._value)


class Misc(object):
    def __init__(self, master=None, cnf=None, **kw):
        self.master = master
        self.children = {}
        self._kw = dict(cnf or {}, **kw)
        self._text = ''
        if 'command' in kw and isinstance(self, Button):
            _buttons.append(self)

    # geometry / configuration: inert
    def pack(self, *a, **k): return None
    def grid(self, *a, **k): return None
    def place(self, *a, **k): return None
    pack_configure = grid_configure = place_configure = pack
    def pack_forget(self): pass
    def grid_forget(self): pass
    def configure(self, cnf=None, **kw):
        self._kw.update(cnf or {}, **kw)
        if isinstance(self, Button) and 'command' in kw and self not in _buttons:
            _buttons.append(self)
    config = configure
    def cget(self, key): return self._kw.get(key, '')
    def __getitem__(self, key): return self.cget(key)
    def __setitem__(self, key, value): self._kw[key] = value
    def bind(self, *a, **k): return ''
    def bind_all(self, *a, **k): return ''
    def unbind(self, *a, **k): pass
    def focus_set(self): pass
    focus = focus_set
    def update(self): _run_after()
    def update_idletasks(self): pass
    def after(self, ms, func=None, *args):
        if func is not None:
            _after.append((func, args))
        return 'after#%d' % len(_after)
    def after_idle(self, func, *args): return self.after(0, func, *args)
    def after_cancel(self, ident): pass
    def destroy(self): pass
    def quit(self): pass
    def winfo_width(self): return 400
    def winfo_height(self): return 300
    def winfo_screenwidth(self): return 1920
    def winfo_screenheight(self): return 1080
    def winfo_exists(self): return 1
    def winfo_children(self): return []
    def mainloop(self, n=0): _mainloop()
    def wait_window(self, w=None): _mainloop()
    def option_add(self, *a, **k): pass
    def columnconfigure(self, *a, **k): pass
    def rowconfigure(self, *a, **k): pass
    grid_columnconfigure = columnconfigure
    grid_rowconfigure = rowconfigure
    def lift(self, *a): pass
    def lower(self, *a): pass
    def event_generate(self, *a, **k): pass
    def clipboard_clear(self): pass
    def clipboard_append(self, *a, **k): pass


class Wm(object):
    def title(self, s=None): return 'tk'
    wm_title = title
    def geometry(self, s=None): return '400x300+0+0'
    wm_geometry = geometry
    def resizable(self, *a): pass
    def protocol(self, *a, **k): pass
    wm_protocol = protocol
    def iconbitmap(self, *a, **k): pass
    def iconphoto(self, *a, **k): pass
    def attributes(self, *a, **k): pass
    wm_attributes = attributes
    def overrideredirect(self, *a): pass
    def minsize(self, *a): pass
    def maxsize(self, *a): pass
    def withdraw(self): pass
    def deiconify(self): pass
    def iconify(self): pass
    def state(self, *a): return 'normal'
    def transient(self, *a): pass
    def grab_set(self): pass


class Tk(Misc, Wm):
    def __init__(self, *a, **k):
        Misc.__init__(self)
        self.tk = self
        _roots.append(self)

    def call(self, *a): return ''
    def eval(self, *a): return ''


class Toplevel(Misc, Wm):
    pass


class Widget(Misc):
    pass


class Frame(Widget): pass
class LabelFrame(Widget): pass
class Label(Widget): pass
class Message(Widget): pass
class Canvas(Widget):
    def create_line(self, *a, **k): return 1
    create_rectangle = create_oval = create_text = create_polygon = create_image = create_arc = create_window = create_line
    def delete(self, *a): pass
    def coords(self, *a): return []
    def itemconfig(self, *a, **k): pass
    itemconfigure = itemconfig
    def move(self, *a): pass
class Scrollbar(Widget):
    def set(self, *a): pass
class PanedWindow(Widget):
    def add(self, *a, **k): pass
class Menu(Widget):
    def add_command(self, *a, **k): pass
    add_cascade = add_separator = add_checkbutton = add_radiobutton = add_command
class Menubutton(Widget): pass


class Button(Widget):
    def invoke(self):
        cmd = self._kw.get('command')
        return cmd() if cmd else None

    def flash(self): pass


class _VarWidget(Widget):
    def __init__(self, master=None, cnf=None, **kw):
        Widget.__init__(self, master, cnf, **kw)
        self.var = kw.get('variable') or kw.get('textvariable')

    def get(self):
        return self.var.get() if self.var is not None else self._kw.get('value', 0)

    def set(self, v):
        if self.var is not None:
            self.var.set(v)
        else:
            self._kw['value'] = v


class Checkbutton(_VarWidget):
    def select(self): self.set(self._kw.get('onvalue', 1))
    def deselect(self): self.set(self._kw.get('offvalue', 0))
    def invoke(self):
        cmd = self._kw.get('command')
        return cmd() if cmd else None


class Radiobutton(Checkbutton):
    def select(self): self.set(self._kw.get('value', 1))


class Scale(_VarWidget): pass


class Entry(_VarWidget):
    def get(self):
        return str(self.var.get()) if self.var is not None else self._text

    def insert(self, index, s):
        if self.var is not None:
            self.var.set(str(self.var.get()) + str(s))
        else:
            self._text += str(s)

    def delete(self, first, last=None):
        if self.var is not None:
            self.var.set('')
        else:
            self._text = ''


class Spinbox(Entry): pass


class Text(Widget):
    def insert(self, index, s, *tags): self._text += str(s)
    def get(self, a='1.0', b=None): return self._text
    def delete(self, *a): self._text = ''
    def see(self, *a): pass
    def tag_configure(self, *a, **k): pass
    tag_config = tag_configure


class Listbox(Widget):
    def __init__(self, master=None, cnf=None, **kw):
        Widget.__init__(self, master, cnf, **kw)
        self.items = []

    def insert(self, index, *items): self.items.extend(items)
    def delete(self, *a): self.items = []
    def get(self, a, b=None): return self.items[a] if b is None and isinstance(a, int) and a < len(self.items) else tuple(self.items)
    def curselection(self): return (0,) if self.items else ()
    def size(self): return len(self.items)
    def selection_set(self, *a): pass
    select_set = selection_set


class OptionMenu(Widget):
    def __init__(self, master, variable, value, *values, **kw):
        Widget.__init__(self, master)
        self.var = variable


class PhotoImage(object):
    def __init__(self, *a, **k): pass
    def width(self): return 1
    def height(self): return 1


def _run_after():
    while _after:
        func, args = _after.pop(0)
        try:
            func(*args)
        except SystemExit:
            raise
        except Exception as e:  # a GUI callback error does not stop Tk
            sys.stderr.write('tkinter callback error: %s\n' % e)


_looped = [False]


def _mainloop():
    """Headless 'user': press one button (STUDIO_TK_BUTTON regex, else the first), then wait for its threads."""
    if _looped[0]:
        return
    _looped[0] = True
    _run_after()
    pattern = os.environ.get('STUDIO_TK_BUTTON', '')
    btn = None
    for b in _buttons:
        text = str(b._kw.get('text', ''))
        if not pattern or re.search(pattern, text, re.I):
            btn = b
            break
    if btn is not None:
        before = set(threading.enumerate())
        sys.stderr.write('[headless tkinter] pressing "%s"\n' % btn._kw.get('text', ''))
        btn.invoke()
        _run_after()
        for t in list(threading.enumerate()):
            if t not in before and not t.daemon and t is not threading.current_thread():
                t.join()


def mainloop(n=0):
    _mainloop()


def __getattr__(name):  # Tk constants (LEFT, W, END, BOTH, ...): their Tk string value
    if name.isupper():
        return name.lower()
    raise AttributeError(name)


# common constants explicitly (Python < 3.7 has no module __getattr__)
for _c in ('LEFT', 'RIGHT', 'TOP', 'BOTTOM', 'BOTH', 'X', 'Y', 'N', 'S', 'E', 'W', 'NE', 'NW', 'SE', 'SW', 'NS', 'EW', 'NSEW',
           'CENTER', 'END', 'INSERT', 'HORIZONTAL', 'VERTICAL', 'NORMAL', 'DISABLED', 'ACTIVE', 'HIDDEN', 'RAISED', 'SUNKEN',
           'FLAT', 'RIDGE', 'GROOVE', 'SOLID', 'WORD', 'CHAR', 'NONE', 'SINGLE', 'BROWSE', 'MULTIPLE', 'EXTENDED', 'YES', 'NO'):
    globals()[_c] = _c.lower()
TRUE, FALSE = 1, 0
