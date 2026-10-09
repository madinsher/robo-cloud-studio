/**
 * Runs genuine RoboDK Python post processors (files defining `class RobotPost`) in the browser with Pyodide.
 * The studio's robomath shim and post_shim.py are written into the Pyodide file system; the compiled
 * PostProgram events are passed as JSON. Pyodide (≈10 MB) is loaded on first use from jsDelivr.
 */
import { PostProgram, PostFile, PostProcessor, registerPost } from './base';
import robomathSrc from '../../python/robodk/robomath.py?raw';
import robodialogsSrc from '../../python/robodk/robodialogs.py?raw';
import robofileioSrc from '../../python/robodk/robofileio.py?raw';
import shimSrc from '../../python/post_shim.py?raw';

/** Minimal robodk.robolink for posts running inside the browser: constants + helpers posts commonly touch. */
const ROBOLINK_STUB = `
ITEM_TYPE_STATION=1; ITEM_TYPE_ROBOT=2; ITEM_TYPE_FRAME=3; ITEM_TYPE_TOOL=4; ITEM_TYPE_OBJECT=5; ITEM_TYPE_TARGET=6
ITEM_TYPE_PROGRAM=8; ITEM_TYPE_INSTRUCTION=9; ITEM_TYPE_PROGRAM_PYTHON=10; ITEM_TYPE_MACHINING=11; ITEM_TYPE_FOLDER=17
INS_TYPE_MOVE=1; INS_TYPE_MOVEC=2; INS_TYPE_CHANGESPEED=3; INS_TYPE_CHANGEFRAME=4; INS_TYPE_CHANGETOOL=5; INS_TYPE_PAUSE=7; INS_TYPE_CODE=9; INS_TYPE_PRINT=10
MOVE_TYPE_JOINT=1; MOVE_TYPE_LINEAR=2; MOVE_TYPE_CIRCULAR=3
RUNMODE_SIMULATE=1; RUNMODE_MAKE_ROBOTPROG=3
def import_install(module_name, pip_name=None, rdk=None, upgrade_pip=False):
    import importlib
    return importlib.import_module(module_name)
def getPathRoboDK():
    return '/studio'
def getPathIcon():
    return '/studio/icon.png'
class Robolink(object):
    def __init__(self, *a, **k):
        raise RuntimeError('robodk.robolink.Robolink is not available inside a post processor running in the browser')
class Item(object):
    pass
`;

const PYODIDE_URL = 'https://cdn.jsdelivr.net/pyodide/v0.26.4/full/pyodide.js';
let pyodidePromise: Promise<any> | null = null;

async function loadPyodide(): Promise<any> {
  if (pyodidePromise) return pyodidePromise;
  pyodidePromise = (async () => {
    if (!(window as any).loadPyodide) {
      await new Promise<void>((resolve, reject) => { const s = document.createElement('script'); s.src = PYODIDE_URL; s.onload = () => resolve(); s.onerror = () => reject(new Error('Failed to load Pyodide')); document.head.appendChild(s); });
    }
    const py = await (window as any).loadPyodide({ indexURL: PYODIDE_URL.replace(/pyodide\.js$/, '') });
    py.FS.mkdirTree('/studio/robodk');
    // The official Apache-2.0 robodk.robomath / robofileio plus headless robodialogs, so unmodified RoboDK posts import cleanly
    py.FS.writeFile('/studio/robodk/__init__.py', 'from . import robomath, robodialogs, robofileio\nfrom .robomath import *\nfrom .robodialogs import *\nfrom .robofileio import *\n');
    py.FS.writeFile('/studio/robodk/robomath.py', robomathSrc);
    py.FS.writeFile('/studio/robodk/robodialogs.py', robodialogsSrc);
    py.FS.writeFile('/studio/robodk/robofileio.py', robofileioSrc);
    py.FS.writeFile('/studio/robodk/robolink.py', ROBOLINK_STUB);
    // legacy top-level modules used by older posts: `from robolink import *`, `from robodk import *`
    py.FS.writeFile('/studio/robolink.py', 'from robodk.robolink import *\n');
    py.FS.writeFile('/studio/post_shim.py', shimSrc);
    await py.runPythonAsync('import sys\nsys.path.insert(0, "/studio")\nimport post_shim');
    return py;
  })();
  return pyodidePromise;
}

/** Serialize PostEvents for the Python side (poses as column-major 16 arrays). */
export function eventsToJSON(p: PostProgram): string {
  return JSON.stringify(p.events.map((e) => { const o: any = { ...e }; for (const k of ['pose', 'via']) if (o[k] instanceof Float64Array) o[k] = Array.from(o[k]); return o; }));
}

/** Run a Python post source against a compiled program. */
export async function runPythonPost(postSource: string, program: PostProgram): Promise<PostFile[]> {
  const py = await loadPyodide();
  py.globals.set('__post_src', postSource);
  py.globals.set('__events', eventsToJSON(program));
  py.globals.set('__robot', program.robotName);
  py.globals.set('__prog', program.name);
  py.globals.set('__dof', program.dof);
  const result = await py.runPythonAsync('import json, post_shim\nres = post_shim.run_from_json(__post_src, __events, __robot, __prog, __dof)\njson.dumps(res)');
  const r = JSON.parse(result);
  const files: PostFile[] = [{ name: `${program.name.replace(/\W+/g, '_')}.${r.extension || 'txt'}`, content: r.program }];
  if (r.log) files.push({ name: `${program.name.replace(/\W+/g, '_')}.log`, content: r.log });
  for (const sub of program.subprograms) files.push(...(await runPythonPost(postSource, sub)));
  return files;
}

/** Register a user-provided Python post as a selectable post processor (async generation via `generateAsync`). */
export function registerPythonPost(id: string, name: string, source: string): PostProcessor & { generateAsync: (p: PostProgram) => Promise<PostFile[]> } {
  const post = {
    id, name: `${name} (Python post)`, brand: 'Custom', extension: 'txt',
    generate: (_p: PostProgram): PostFile[] => [{ name: 'README.txt', content: 'This is a Python post processor: use generateAsync / the Export dialog (runs in Pyodide).' }],
    generateAsync: (p: PostProgram) => runPythonPost(source, p),
  };
  registerPost(post);
  return post;
}
