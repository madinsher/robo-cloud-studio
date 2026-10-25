"""Headless tkinter.messagebox: informational boxes print, questions answer yes/ok (the default button)."""
import sys


def _say(kind, title, message):
    sys.stderr.write('[headless %s] %s: %s\n' % (kind, title, message))


def showinfo(title=None, message=None, **k): _say('info', title, message); return 'ok'
def showwarning(title=None, message=None, **k): _say('warning', title, message); return 'ok'
def showerror(title=None, message=None, **k): _say('error', title, message); return 'ok'
def askquestion(title=None, message=None, **k): _say('question', title, message); return 'yes'
def askokcancel(title=None, message=None, **k): _say('question', title, message); return True
def askyesno(title=None, message=None, **k): _say('question', title, message); return True
def askyesnocancel(title=None, message=None, **k): _say('question', title, message); return True
def askretrycancel(title=None, message=None, **k): _say('question', title, message); return True
