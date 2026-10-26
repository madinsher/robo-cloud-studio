"""Headless tkinter.font (see tkinter/__init__.py)."""
NORMAL, BOLD, ITALIC, ROMAN = 'normal', 'bold', 'italic', 'roman'


class Font(object):
    def __init__(self, root=None, font=None, name=None, exists=False, **options):
        self.options = dict(options)

    def configure(self, **options):
        self.options.update(options)
    config = configure

    def cget(self, key):
        return self.options.get(key)

    def measure(self, text, *a):
        return 7 * len(str(text))

    def metrics(self, *a, **k):
        return 12

    def actual(self, *a, **k):
        return dict(self.options)


def families(*a, **k):
    return ('Helvetica',)


def nametofont(name, root=None):
    return Font()
