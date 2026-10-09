"""Headless stand-in for robodk.robodialogs.

RoboDK post processors and scripts call these helpers to show message boxes or ask for files.
Inside VerticalBot Studio (Pyodide in the browser, or the headless server) there is no Tk/Qt, so
every dialog degrades to a log line and a sensible default so posts keep running unattended.
The function names and signatures follow the official Apache-2.0 robodk package.
"""
import sys

FILE_TYPES_ALL = ('All Files', '.*')
FILE_TYPES_ROBODK = ('RoboDK Files', '.sld .rdk .robot .tool .rdka .rdkbak .rdkp .py')
FILE_TYPES_3D_OBJECT = ('3D Object Files', '.sld .stl .iges .igs .step .stp .obj .slp .3ds .dae .blend .wrl .wrml')
FILE_TYPES_TEXT = ('Text Files', '.txt .csv')
FILE_TYPES_IMG = ('Image Files', '.png .jpg')

ENABLE_QT = False
ENABLE_TK = False

LOG = []  #: every message that would have been shown to the user


def _log(kind, msg):
    line = "[%s] %s" % (kind, msg)
    LOG.append(line)
    try:
        sys.stderr.write(line + "\n")
    except Exception:
        pass


def getOpenFile(path_preference="C:/RoboDK/Library/", strfile='', strtitle='Open File', defaultextension='.txt', filetypes=None):
    _log("dialog", "getOpenFile(%s) -> '' (headless)" % strtitle)
    return ''


def getSaveFile(path_preference="C:/RoboDK/Library/", strfile='file.txt', strtitle='Save File As', defaultextension='.txt', filetypes=None):
    _log("dialog", "getSaveFile(%s) -> %s (headless)" % (strtitle, strfile))
    return strfile


def getOpenFileName(path_preference="C:/RoboDK/Library/", strfile='', strtitle='Open File', defaultextension='.txt', filetypes=None):
    return getOpenFile(path_preference, strfile, strtitle, defaultextension, filetypes)


def getSaveFileName(path_preference="C:/RoboDK/Library/", strfile='file.txt', strtitle='Save File As', defaultextension='.txt', filetypes=None):
    return getSaveFile(path_preference, strfile, strtitle, defaultextension, filetypes)


def getOpenFileNames(path_preference="C:/RoboDK/Library/", strfile='', strtitle='Open File(s)', defaultextension='.txt', filetypes=None):
    return []


def getOpenFolder(path_preference="C:/RoboDK/Library/", strtitle='Open Folder'):
    _log("dialog", "getOpenFolder(%s) -> '' (headless)" % strtitle)
    return ''


def getSaveFolder(path_preference="C:/RoboDK/Library/", strtitle='Save to Folder', **kwargs):
    return path_preference


def ShowMessage(msg, title=None):
    _log("message", "%s%s" % ((title + ": ") if title else "", msg))
    return True


def ShowMessageYesNo(msg, title=None):
    _log("question", "%s -> Yes (headless default)" % msg)
    return True


def ShowMessageYesNoCancel(msg, title=None):
    _log("question", "%s -> Yes (headless default)" % msg)
    return True


def ShowMessageOkCancel(msg, title=None):
    _log("question", "%s -> OK (headless default)" % msg)
    return True


def mbox(msg, b1='OK', b2='Cancel', frame=True, t=False, entry=None):
    """Message box: returns True for the first button, or the default entry text when `entry` is given."""
    if entry is not None:
        _log("input", "%s -> %r (headless default)" % (msg, entry))
        return entry
    _log("question", "%s [%s/%s] -> %s (headless default)" % (msg, b1, b2, b1))
    return True


def InputDialog(msg, value, title=None, default_button=False, default_value=None, embed=False, actions=None):
    _log("input", "%s -> %r (headless default)" % (msg, value))
    return value


def getString(title, default=""):
    return default


def getInt(title, default=0):
    return default


def getFloat(title, default=0.0):
    return default


class MessageBox(object):
    def __init__(self, msg, b1, b2, frame, t, entry):
        self.returning = True
