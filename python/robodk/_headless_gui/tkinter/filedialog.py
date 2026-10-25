"""Headless tkinter.filedialog for unattended runs.

Open dialogs return '' (nothing chosen, like Cancel). Save dialogs accept the proposed file name like a user pressing
Save, but always inside a scratch folder (tempdir/vbs_headless_files) so unattended scripts never write next to the
RoboDK installation or the user's files.
"""
import os
import tempfile


def _save_path(initialfile=None, defaultextension=None, **k):
    folder = os.path.join(tempfile.gettempdir(), 'vbs_headless_files')
    os.makedirs(folder, exist_ok=True)
    name = os.path.basename(str(initialfile or 'file'))
    ext = defaultextension if isinstance(defaultextension, str) else ''
    if ext and not os.path.splitext(name)[1]:
        name += ext if ext.startswith('.') else '.' + ext
    return os.path.join(folder, name).replace(os.sep, '/')


def askopenfilename(**k): return ''
def askdirectory(**k): return ''
def askopenfilenames(**k): return ()
def askopenfile(**k): return None
def asksaveasfilename(**k): return _save_path(**k)


def asksaveasfile(mode='w', **k):
    return open(_save_path(**k), mode)
