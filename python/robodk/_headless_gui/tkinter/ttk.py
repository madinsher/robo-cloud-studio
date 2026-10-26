"""Headless tkinter.ttk: the same inert widgets as tkinter."""
from tkinter import *  # noqa: F401,F403
from tkinter import Frame, Label, Button, Entry, Checkbutton, Radiobutton, Scale, Scrollbar, Spinbox, Widget, LabelFrame  # noqa: F401


class Combobox(Entry):
    def current(self, i=None): return 0


class Progressbar(Widget):
    def start(self, *a): pass
    def stop(self): pass
    def step(self, *a): pass


class Notebook(Widget):
    def add(self, *a, **k): pass


class Separator(Widget): pass
class Treeview(Widget):
    def insert(self, *a, **k): return 'I001'
    def heading(self, *a, **k): pass
    def column(self, *a, **k): pass
    def delete(self, *a): pass
    def get_children(self, *a): return ()


class Style(object):
    def __init__(self, *a, **k): pass
    def configure(self, *a, **k): pass
    def theme_use(self, *a): return 'default'
    def map(self, *a, **k): pass
