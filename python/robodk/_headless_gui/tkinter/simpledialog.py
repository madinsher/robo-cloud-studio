"""Headless tkinter.simpledialog: returns initialvalue when given, else None (Cancel)."""


def askstring(title, prompt, **k): return k.get('initialvalue')
def askinteger(title, prompt, **k): return k.get('initialvalue')
def askfloat(title, prompt, **k): return k.get('initialvalue')
