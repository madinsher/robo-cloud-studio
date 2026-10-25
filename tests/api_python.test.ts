/**
 * T10 acceptance: the Python drop-in (python/robodk, legacy python/robolink) against a headless studio API server.
 * Every block runs real CPython through the WebSocket transport and checks RoboDK semantics (values from RoboDK's own
 * robolink.py / docstrings) or physical invariants.
 */
import { describe, it, expect, beforeAll, afterAll } from 'vitest';
import { spawn } from 'node:child_process';
import { mkdtempSync, writeFileSync, readFileSync, existsSync, readdirSync } from 'node:fs';
import { tmpdir } from 'node:os';
import path from 'node:path';
import { Station, ItemType } from '../src/core/items/item';
import { Robot } from '../src/core/items/robot';
import { createNodeHost, NodeHost } from '../src/api/node_host';
import { findPython } from '../src/api/runner_cpython';
import { createRobotFromLibrary } from '../src/core/items/library';

let py: { cmd: string; args: string[] } | null = null;
try { py = findPython(); } catch { py = null; }
const DROPIN = path.join(process.cwd(), 'python');
const OFFICIAL = 'C:/RoboDK/Python/robodk/robolink.py';

/** Run a Python snippet asynchronously (the API server lives in this process: never block the event loop). */
function runPy(host: NodeHost, code: string, args: string[] = [], timeout = 120000): Promise<{ out: string; err: string; status: number | null; json: any }> {
  const dir = mkdtempSync(path.join(tmpdir(), 'vbs-pytest-'));
  const file = path.join(dir, 'case.py');
  writeFileSync(file, code);
  return new Promise((resolve) => {
    const child = spawn(py!.cmd, [...py!.args, '-u', file, ...args], { env: { ...process.env, STUDIO_URL: host.url, PYTHONPATH: DROPIN, PYTHONIOENCODING: 'utf-8' }, cwd: dir, windowsHide: true });
    let out = '', err = '';
    child.stdout.on('data', (d) => (out += d));
    child.stderr.on('data', (d) => (err += d));
    const timer = setTimeout(() => { err += '[timeout]'; child.kill(); }, timeout);
    child.on('close', (status) => {
      clearTimeout(timer);
      const lines = out.trim().split(/\r?\n/);
      let json: any = null;
      for (let i = lines.length - 1; i >= 0; i--) { if (lines[i].startsWith('JSON:')) { json = JSON.parse(lines[i].slice(5)); break; } }
      resolve({ out, err, status, json });
    });
  });
}
const HEAD = `import json, sys\nfrom robolink import *\nfrom robodk import *\nRDK = Robolink()\ndef out(**k):\n    print('JSON:' + json.dumps(k))\n`;

function ur(st: Station, name = 'UR10e', x = 0): Robot {
  const r = createRobotFromLibrary('UR10e');
  r.setName(name);
  st.addChild(r);
  if (x) r.setPose(Float64Array.from([1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, x, 0, 0, 1]));
  r.setJoints([10, -80, 100, -110, -90, 20]);
  return r;
}

describe.skipIf(!py)('Python drop-in against the headless studio (CPython, WebSocket)', { timeout: 120000 }, () => {
  let host: NodeHost;
  beforeAll(async () => { host = await createNodeHost({ station: new Station('T10'), port: 0, tcpPort: 0, speed: 50, log: () => {} }); });
  afterAll(async () => { await host?.close(); });
  const fresh = (build?: (st: Station) => void) => { const st = new Station('T10'); build?.(st); host.setStation(st); return st; };

  it('legacy imports alias the drop-in and every official constant has RoboDK\'s value', async () => {
    fresh();
    const r = await runPy(host, `${HEAD}
import robolink as legacy, robodk.robolink as modern
from robodk import robolink as rl2
out(same=legacy is modern and legacy.Robolink is modern.Robolink, ready=ROBOTCOM_READY, disc=ROBOTCOM_DISCONNECTED, r3=MAKE_ROBOT_3R, dof6=MAKE_ROBOT_6DOF,
    spray=[SPRAY_OFF, SPRAY_ON], path=PATH_OPENSTATION, notes=ITEM_TYPE_NOTES, exc=[TargetReachError.__name__, StoppedError.__name__, InputError.__name__, LicenseError.__name__],
    missing_r=[m for m in OFFICIAL_ROBOLINK_METHODS if not hasattr(Robolink, m)], missing_i=[m for m in OFFICIAL_ITEM_METHODS if not hasattr(Item, m)],
    n_const=len(OFFICIAL_CONSTANTS), time_exported=hasattr(sys.modules['__main__'], 'time'))
`);
    expect(r.status, r.err).toBe(0);
    expect(r.json.same).toBe(true);
    // values from C:/RoboDK/Python/robodk/robolink.py (RoboDK 5.9.4)
    expect([r.json.ready, r.json.disc, r.json.r3, r.json.dof6]).toEqual([0, -2, 3, 7]);
    expect(r.json.spray).toEqual([0, 1]);
    expect(r.json.path).toBe('PATH_OPENSTATION');
    expect(r.json.notes).toBe(22);
    expect(r.json.exc).toEqual(['TargetReachError', 'StoppedError', 'InputError', 'LicenseError']);
    expect(r.json.missing_r).toEqual([]);
    expect(r.json.missing_i).toEqual([]);
    expect(r.json.n_const).toBeGreaterThanOrEqual(200);
    expect(r.json.time_exported).toBe(true);
  });

  it.skipIf(!existsSync(OFFICIAL))('every constant of the official robolink.py exists in the drop-in with the same value (AST diff empty)', async () => {
    fresh();
    const r = await runPy(host, `${HEAD}
import ast
tree = ast.parse(open(r'${OFFICIAL}', encoding='utf-8').read())
diff = []
for n in tree.body:
    if isinstance(n, ast.Assign) and all(isinstance(t, ast.Name) for t in n.targets):
        try:
            val = eval(compile(ast.Expression(n.value), 'x', 'eval'), {}, {})
        except Exception:
            continue
        for t in n.targets:
            if globals().get(t.id, '<missing>') != val:
                diff.append(t.id)
out(diff=diff)
`);
    expect(r.status, r.err).toBe(0);
    expect(r.json.diff).toEqual([]);
  });

  it('poses travel by rows: setPose/Pose round trip, doc example, Mat return types', async () => {
    fresh((st) => ur(st));
    const r = await runPy(host, `${HEAD}
f = RDK.AddFrame('F')
f.setPose(transl(600, 0, 300) * rotz(30 * pi / 180))
p = f.Pose()
robot = RDK.Item('', ITEM_TYPE_ROBOT)
j = robot.Joints()
out(pos=p.Pos(), rz=p.rows[1][0], joints_type=type(j).__name__, jl=j.list(), jt=robot.JointsHome().tolist(), size=list(robot.Joints().size()),
    tr=robot.Joints().tr().size(), lims=[type(x).__name__ for x in robot.JointLimits()[:2]])
`);
    expect(r.status, r.err).toBe(0);
    expect(r.json.pos).toEqual([600, 0, 300]);
    expect(r.json.rz).toBeCloseTo(Math.sin(Math.PI / 6), 9);
    expect(r.json.joints_type).toBe('Mat');
    expect(r.json.jl).toEqual([10, -80, 100, -110, -90, 20]);
    expect(r.json.size).toEqual([6, 1]);
    expect(r.json.tr).toEqual([1, 6]);
    expect(r.json.lims).toEqual(['Mat', 'Mat']);
  });

  it('SolveFK/SolveIK are flange based with optional tool/reference; PoseAbs of a robot is its base', async () => {
    fresh((st) => ur(st, 'UR10e', 1000));
    const r = await runPy(host, `${HEAD}
robot = RDK.Item('', ITEM_TYPE_ROBOT)
j0 = [10, -80, 100, -110, -90, 20]
fk_no_tool = robot.SolveFK(j0)
tool = robot.AddTool(transl(0, 0, 200), 'T200')
fk = robot.SolveFK(j0)
fk_tool = robot.SolveFK(j0, transl(0, 0, 200))
ik = robot.SolveIK(fk, j0)
ik_tool = robot.SolveIK(fk_tool, j0, transl(0, 0, 200))
ref = transl(100, 50, 0)
fk_ref = robot.SolveFK(j0, None, ref)
ik_ref = robot.SolveIK(fk_ref, j0, None, ref)
out(d_tool=(fk.Pos()[0]-fk_no_tool.Pos()[0])**2 + (fk.Pos()[1]-fk_no_tool.Pos()[1])**2 + (fk.Pos()[2]-fk_no_tool.Pos()[2])**2,
    d_flange_tcp=norm(subs3(fk_tool.Pos(), fk.Pos())), ik=ik.list(), ik_tool=ik_tool.list(), ik_ref=ik_ref.list(),
    base=robot.PoseAbs().Pos(), unreachable=robot.SolveIK(transl(9000, 0, 0)).list(), all_cols=robot.SolveIK_All(fk).size(0))
`);
    expect(r.status, r.err).toBe(0);
    expect(r.json.d_tool).toBeLessThan(1e-9);               // the active 200 mm tool does not change SolveFK
    expect(r.json.d_flange_tcp).toBeCloseTo(200, 6);
    for (const k of ['ik', 'ik_tool', 'ik_ref']) r.json[k].forEach((v: number, i: number) => expect(v).toBeCloseTo([10, -80, 100, -110, -90, 20][i], 3));
    expect(r.json.base).toEqual([1000, 0, 0]);
    expect(r.json.unreachable).toEqual([]);
    expect(r.json.all_cols).toBe(6);
  });

  it('setPoseFrame/setPoseTool with matrices reuse items; program frame/tool changes are instructions', async () => {
    const st = fresh((s) => ur(s));
    const r = await runPy(host, `${HEAD}
robot = RDK.Item('', ITEM_TYPE_ROBOT)
n0 = len(RDK.ItemList(ITEM_TYPE_FRAME))
for k in range(3):
    robot.setPoseFrame(transl(100 * k, 0, 0))
robot.setPoseTool(transl(0, 0, 100))
F1 = RDK.AddFrame('F1'); F2 = RDK.AddFrame('F2'); F2.setPose(transl(0, 300, 0))
T1 = RDK.AddTarget('T1', F1, robot); T2 = RDK.AddTarget('T2', F2, robot)
prog = RDK.AddProgram('P', robot)
prog.setPoseFrame(F1); prog.MoveJ(T1); prog.setPoseFrame(F2); prog.MoveJ(T2)
types = [prog.Instruction(i)[1] for i in range(prog.InstructionCount())]
out(frames=len(RDK.ItemList(ITEM_TYPE_FRAME)) - n0, frame_x=robot.PoseFrame().Pos()[0], tool_z=robot.PoseTool().Pos()[2], types=types, last_pose=[round(v, 6) for v in prog.Instruction(-1)[4].Pos()], t2_pose=[round(v, 6) for v in T2.Pose().Pos()])
`);
    expect(r.status, r.err).toBe(0);
    expect(r.json.frames).toBe(1 + 2);   // one reused user frame + F1 + F2
    expect(r.json.frame_x).toBeCloseTo(200, 6);
    expect(r.json.tool_z).toBeCloseTo(100, 6);
    expect(r.json.types).toEqual([3, 0, 3, 0]);
    expect(r.json.last_pose).toEqual(r.json.t2_pose); // Instruction(-1) is the last instruction (MoveJ T2)
    expect(st.itemsOfType(ItemType.TOOL).length).toBe(1);
  });

  it('lists of items/poses are decoded recursively; calls on invalid items raise', async () => {
    fresh((st) => { ur(st, 'R1'); ur(st, 'R2', 1500); });
    const r = await runPy(host, `${HEAD}
a = RDK.AddFrame('a'); b = RDK.AddFrame('b')
RDK.setPoses([a, b], [transl(1, 2, 3), transl(4, 5, 6)])
pa, pb = a.Pose().Pos(), b.Pose().Pos()
r1, r2 = RDK.Item('R1'), RDK.Item('R2')
jj = RDK.Joints([r1, r2])
RDK.setSelection([a]); sel = [s.Name() for s in RDK.Selection()]
RDK.Delete([a, b])
bad = RDK.Item('does not exist')
errs = []
for call in (lambda: bad.Joints(), lambda: bad.setParam('X', 5), lambda: bad.Name()):
    try:
        call(); errs.append('no error')
    except Exception as e:
        errs.append(type(e).__name__ + ':' + str(e)[:40])
out(pa=pa, pb=pb, njj=len(jj), jj0=jj[0].list(), sel=sel, left=RDK.ItemList(ITEM_TYPE_FRAME, True), valid=bad.Valid(), errs=errs, stationX=RDK.getParam('X'))
`);
    expect(r.status, r.err).toBe(0);
    expect(r.json.pa).toEqual([1, 2, 3]);
    expect(r.json.pb).toEqual([4, 5, 6]);
    expect(r.json.njj).toBe(2);
    expect(r.json.jj0).toEqual([10, -80, 100, -110, -90, 20]);
    expect(r.json.sel).toEqual(['a']);
    expect(r.json.left).toEqual([]);
    expect(r.json.valid).toBe(false);
    expect(r.json.errs.every((e: string) => e.startsWith('InputError'))).toBe(true);
    expect(r.json.stationX).toBeNull();
  });

  it('robot moves are timed motions: Busy/WaitMove with non-blocking moves, TargetReachError, MoveC with poses', async () => {
    fresh((st) => ur(st));
    const r = await runPy(host, `${HEAD}
import time
robot = RDK.Item('', ITEM_TYPE_ROBOT)
robot.setSpeed(200, 60)
home = [0, -90, 90, -90, -90, 0]
t0 = time.time()
robot.MoveJ(home, False)
busy = robot.Busy()
robot.WaitMove()
done = robot.Joints().list()
p = robot.Pose()
robot.MoveL(p * transl(0, 0, 100))
dz = robot.Pose().Pos()[2] - p.Pos()[2]
robot.MoveC(p * transl(50, 50, 100), p * transl(100, 0, 100))
dx = norm(subs3(robot.Pose().Pos(), (p * transl(100, 0, 100)).Pos()))
try:
    robot.MoveJ([0, -90, 90, -90, -90, 9999]); reach = 'moved'
except TargetReachError as e:
    reach = 'TargetReachError'
try:
    robot.MoveL(transl(9000, 0, 0)); reach2 = 'moved'
except TargetReachError as e:
    reach2 = 'TargetReachError'
out(busy=busy, done=done, dz=dz, dx=dx, reach=reach, reach2=reach2, t=time.time() - t0, simt=RDK.SimulationTime())
`);
    expect(r.status, r.err).toBe(0);
    expect(r.json.busy).toBe(1);
    r.json.done.forEach((v: number, i: number) => expect(v).toBeCloseTo([0, -90, 90, -90, -90, 0][i], 6));
    expect(Math.abs(r.json.dz)).toBeCloseTo(100, 1); // 100 mm along the tool Z axis
    expect(r.json.dx).toBeLessThan(0.01); // MoveC ends on the end pose
    expect(r.json.reach).toBe('TargetReachError');
    expect(r.json.reach2).toBe('TargetReachError');
    expect(r.json.simt).toBeGreaterThan(0.5);   // simulated time advanced by the motions
  });
});
