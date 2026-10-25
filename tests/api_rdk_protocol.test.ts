/**
 * T10 P1: the RoboDK binary API protocol (src/api/rdk_protocol.ts). RoboDK's OFFICIAL Python package
 * (C:/RoboDK/Python/robodk, run isolated so the studio drop-in cannot shadow it) connects to a headless studio and
 * drives it unmodified: items, poses (column-major on the wire), joints, FK/IK, programs, blocking moves with
 * TargetReachError, lists, parameters, invalid items. Values are checked against the studio's own model.
 */
import { describe, it, expect, beforeAll, afterAll } from 'vitest';
import { spawn } from 'node:child_process';
import { mkdtempSync, writeFileSync, existsSync } from 'node:fs';
import { tmpdir } from 'node:os';
import path from 'node:path';
import { Station, ItemType, Frame } from '../src/core/items/item';
import { Robot } from '../src/core/items/robot';
import { Program } from '../src/core/items/program';
import { createRobotFromLibrary } from '../src/core/items/library';
import { createNodeHost, NodeHost } from '../src/api/node_host';
import { startRdkProtocolServer, RdkProtocolServer } from '../src/api/rdk_protocol';
import { findPython } from '../src/api/runner_cpython';
import { getPos } from '../src/core/math/pose';

const OFFICIAL = 'C:/RoboDK/Python';
let py: { cmd: string; args: string[] } | null = null;
try { py = findPython(); } catch { py = null; }
const ready = !!py && existsSync(`${OFFICIAL}/robodk/robolink.py`);

function runOfficial(port: number, body: string, timeout = 120000): Promise<{ status: number | null; out: string; err: string; json: any }> {
  const dir = mkdtempSync(path.join(tmpdir(), 'vbs-rdkproto-'));
  const file = path.join(dir, 'case.py');
  writeFileSync(file, `import sys, json\nsys.path.insert(0, ${JSON.stringify(OFFICIAL)})\nfrom robodk.robolink import *\nfrom robodk.robomath import *\nimport robodk.robolink as _rl\nassert _rl.__file__.replace('\\\\', '/').startswith(${JSON.stringify(OFFICIAL)}), _rl.__file__\nRDK = Robolink(robodk_ip="127.0.0.1", port=${port})   # not "localhost": the official client would start RoboDK when it cannot connect\ndef out(**k):\n    print('JSON:' + json.dumps(k))\n${body}\n`);
  return new Promise((resolve) => {
    // -I: isolated mode (no PYTHONPATH / user site / script dir): only the official package is importable;
    // -B: no .pyc written next to C:/RoboDK/Python (read-only)
    const child = spawn(py!.cmd, [...py!.args, '-I', '-B', '-u', file], { cwd: dir, windowsHide: true, env: { ...process.env, PYTHONIOENCODING: 'utf-8' } });
    let out = '', err = '';
    child.stdout.on('data', (d) => (out += d));
    child.stderr.on('data', (d) => (err += d));
    const timer = setTimeout(() => { err += '[timeout]'; child.kill(); }, timeout);
    child.on('close', (status) => {
      clearTimeout(timer);
      const line = out.split(/\r?\n/).reverse().find((l) => l.startsWith('JSON:'));
      resolve({ status, out, err, json: line ? JSON.parse(line.slice(5)) : null });
    });
  });
}

describe.skipIf(!ready)('official robodk package over the RoboDK binary protocol', { timeout: 180000 }, () => {
  let host: NodeHost;
  let proto: RdkProtocolServer;
  let robot: Robot;
  beforeAll(async () => {
    const st = new Station('Proto');
    robot = createRobotFromLibrary('UR10e');
    st.addChild(robot);
    robot.setJoints([10, -80, 100, -110, -90, 20]);
    host = await createNodeHost({ station: st, port: 0, tcpPort: -1, speed: 50, log: () => {} });
    proto = await startRdkProtocolServer({ port: 0, handle: (req) => host.api.handle(req), log: () => {} });
  });
  afterAll(async () => { await proto?.close(); await host?.close(); });

  it('items, poses, joints, FK/IK, lists, parameters and invalid items', async () => {
    const r = await runOfficial(proto.port, `
robot = RDK.Item('', ITEM_TYPE_ROBOT)
frame = RDK.AddFrame('Fp')
frame.setPose(transl(100, 200, 300) * rotz(0.5))
p = frame.Pose()
j = robot.Joints().list()
fk = robot.SolveFK(j)
ik = robot.SolveIK(fk, j).list()
robot.setJoints([0, -90, 90, -90, -90, 0])
names = RDK.ItemList(ITEM_TYPE_FRAME, True)
RDK.setParam('ProtoKey', 'abc')
bad = RDK.Item('does not exist')
try:
    bad.Name(); badname = 'no error'
except Exception as e:
    badname = type(e).__name__
out(valid=robot.Valid(), name=robot.Name(), type=robot.Type(), pos=p.Pos(), rz=p[0, 1], joints=j, fk=fk.Pos(), ik=ik,
    after=robot.Joints().list(), names=names, param=RDK.getParam('ProtoKey'), unknown=RDK.getParam('NoSuchKey'),
    badvalid=bad.Valid(), badname=badname, ver=RDK.Version(), childs=[c.Name() for c in RDK.ActiveStation().Childs()])
`);
    expect(r.status, r.err + r.out).toBe(0);
    const j = r.json;
    expect(j.valid).toBe(true);
    expect(j.name).toBe(robot.name);
    expect(j.type).toBe(ItemType.ROBOT);
    j.pos.forEach((v: number, i: number) => expect(v).toBeCloseTo([100, 200, 300][i], 9));
    expect(j.rz).toBeCloseTo(-Math.sin(0.5), 9);                     // row 0 col 1 of rotz(0.5): poses travel column-major
    j.joints.forEach((v: number, i: number) => expect(v).toBeCloseTo([10, -80, 100, -110, -90, 20][i], 9));
    const fl = getPos(robot.solveFKFlange([10, -80, 100, -110, -90, 20]));
    j.fk.forEach((v: number, i: number) => expect(v).toBeCloseTo(fl[i], 6));   // SolveFK = flange w.r.t. the robot base
    j.ik.forEach((v: number, i: number) => expect(v).toBeCloseTo(j.joints[i], 4));
    j.after.forEach((v: number, i: number) => expect(v).toBeCloseTo([0, -90, 90, -90, -90, 0][i], 9));
    expect(robot.joints()[1]).toBeCloseTo(-90, 9);
    expect(j.names).toContain('Fp');
    expect(j.param).toBe('abc');
    expect(j.unknown).toBeNull();
    expect(j.badvalid).toBe(false);
    expect(j.childs).toContain(robot.name);
    expect(host.station().find('Fp', ItemType.FRAME)).toBeInstanceOf(Frame);
  });

  it('programs, targets, blocking robot moves and TargetReachError', async () => {
    const r = await runOfficial(proto.port, `
robot = RDK.Item('', ITEM_TYPE_ROBOT)
robot.setSpeed(500, 180)
home = [0, -90, 90, -90, -90, 0]
robot.MoveJ(home)
t = RDK.AddTarget('Tp', 0, robot)
prog = RDK.AddProgram('Pp', robot)
prog.MoveJ(t)
prog.MoveL(robot.Pose() * transl(0, 0, 50))
n = prog.InstructionCount()
ins = prog.Instruction(0)
dest = [20, -90, 90, -90, -90, 0]
robot.MoveJ(dest)
reached = robot.Joints().list()
p0 = robot.Pose()
robot.MoveL(p0 * transl(0, 0, 80))
dz = robot.Pose().Pos()[2] - p0.Pos()[2]
try:
    robot.MoveL(transl(9000, 0, 0)); err = 'moved'
except TargetReachError as e:
    err = 'TargetReachError'
upd = prog.Update()
out(n=n, insname=ins[0], instype=ins[1], reached=reached, dz=dz, err=err, valid=upd[0], busy=robot.Busy())
`);
    expect(r.status, r.err + r.out).toBe(0);
    expect(r.json.n).toBe(2);
    expect(r.json.instype).toBe(0);                                   // INS_TYPE_MOVE
    r.json.reached.forEach((v: number, i: number) => expect(v).toBeCloseTo([20, -90, 90, -90, -90, 0][i], 6));
    expect(Math.abs(r.json.dz)).toBeCloseTo(80, 3);
    expect(r.json.err).toBe('TargetReachError');
    expect(r.json.valid).toBe(2);
    expect(r.json.busy).toBe(0);
    const prog = host.station().find('Pp', ItemType.PROGRAM) as Program;
    expect(prog.instructions().length).toBe(2);
  });
});
