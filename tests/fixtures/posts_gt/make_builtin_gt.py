"""Ground truth of RoboDK's BUILT-IN post processors (compiled into RoboDK.exe), for tests/posts_builtin.test.ts.

RoboDK 5.9.4 generates a robot's programs with a built-in generator when the robot has no post processor stored in
the station (Library stations 02.d, 03.e, 05.c, 05.d, 06.b, 06.d stamp, 07.e, 08.a, 10.a; mechanisms get the
"undefined post processor" C-like placeholder). A robot loaded from a .robot file always has a post (setParam
('PostProcessor', '') only reads it back), so this script opens Library stations whose robot has none, adds feature
programs (every instruction kind) through the official API and saves what MakeProgram writes:

    C:/RoboDK/Python37/python -I make_builtin_gt.py <port> [brand-filter]

Output: builtin/<brand>/<program>/<files> and builtin/<brand>/station.json (each program's instructions with the
poses and joints RoboDK holds, frames, tools), from which tests/posts_builtin.test.ts rebuilds the same programs
without RoboDK.
"""
import sys, os, json, shutil
sys.path.insert(0, r'C:\RoboDK\Python')
from robodk.robolink import *
from robodk.robomath import *

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, 'builtin')
LIB = r'C:\RoboDK\Library'
port = int(sys.argv[1])
flt = sys.argv[2] if len(sys.argv) > 2 else ''
STATIONS = [
    ('ABB', 'Example-06.b-Pick and place 2 tables.rdk', 'ABB IRB 1600-8/1.45'),
    ('KUKA', 'Example-08.a-Synchronize 3 Robots with a Screen (GUI).rdk', 'R1'),
    ('Fanuc', 'Example-06.d-Pick and stamp.rdk', 'Fanuc LR Mate 200iD/7L'),
    ('Comau', 'Example-05.d-H bot gantry - Comau.rdk', 'Comau Smart NM 45-2.0'),
    ('UR', 'Example-07.e-Path Move_Test - Move Linear or Joints.rdk', 'UR10'),
    ('Mechanism', 'Example-06.d-Pick and stamp.rdk', 'Gripper'),
]

RDK = Robolink(args=['-NOSPLASH', '-HIDDEN', '-NEWINSTANCE', '-NOUI'], port=port)
RDK.setRunMode(RUNMODE_SIMULATE)
if os.environ.get('BUILTIN_TIMEOUT'):
    RDK.COM.settimeout(float(os.environ['BUILTIN_TIMEOUT']))


def m16(p):
    return [p[i, j] for i in range(4) for j in range(4)]


def lst(j):
    return j.list() if hasattr(j, 'list') else list(j)


def dump_prog(p, pdir):
    ins = []
    for i in range(p.InstructionCount()):
        nm, it, mt, isj, pose, joints = p.Instruction(i)
        ins.append({'name': nm, 'type': it, 'move': mt, 'isJoint': isj, 'pose': m16(pose) if pose is not None else None,
                    'joints': lst(joints) if joints is not None else None})
    upd = p.Update()
    os.makedirs(pdir, exist_ok=True)
    print(p.Name(), 'update', upd, flush=True)
    mk = p.MakeProgram(pdir)
    return {'instructions': ins, 'update': list(upd), 'make': list(mk) if mk else None, 'files': sorted(os.listdir(pdir))}


try:
    for brand, station, rname in STATIONS:
        if flt and flt != brand:
            continue
        RDK.CloseStation()
        RDK.AddFile(os.path.join(LIB, station))
        robot = RDK.Item(rname, ITEM_TYPE_ROBOT)
        dof = len(lst(robot.Joints()))
        rdir = os.path.join(OUT, brand)
        shutil.rmtree(rdir, ignore_errors=True)
        os.makedirs(rdir)
        j0 = lst(robot.Joints())
        if brand == 'Comau':
            j0 = lst(RDK.Item('Approach1', ITEM_TYPE_TARGET).Joints())   # a configuration the station reaches
        elif dof >= 6 and abs(j0[4]) < 20:
            j0[4] += 35                                    # keep the wrist away from its singularity
            j0[2] += 10
            lo, hi = [lst(x) for x in robot.JointLimits()[:2]]
            for i in range(6):                             # and every arm joint 15 deg inside its limits
                j0[i] = min(max(j0[i], lo[i] + 15), hi[i] - 15)
        info = {'brand': brand, 'station': station, 'robot': rname, 'dof': dof, 'joints': j0, 'programs': {}}
        if brand == 'Mechanism':
            prog = RDK.AddProgram('MechProg', robot)
            t1 = RDK.AddTarget('Mech Open', robot.Parent(), robot)
            t1.setAsJointTarget()
            t1.setJoints([0] * dof)
            t2 = RDK.AddTarget('Mech Close', robot.Parent(), robot)
            t2.setAsJointTarget()
            t2.setJoints([12.5] + [0] * (dof - 1))
            prog.MoveJ(t1)
            prog.Pause(500)
            prog.RunInstruction('Mech comment', INSTRUCTION_COMMENT)
            prog.setDO('7', 1)
            prog.MoveJ(t2)
            progs = [prog]
        else:
            if brand == 'Comau':
                # gantry-mounted arm: reuse a point of the station's own path (Path1, frame Part1 reference, Spindle)
                moves = []
                p1 = RDK.Item('Path1', ITEM_TYPE_PROGRAM)
                for i in range(p1.InstructionCount()):
                    nm, it, mt, isj, pose, joints = p1.Instruction(i)
                    if it == INS_TYPE_MOVE and pose is not None and not isj:
                        moves.append((pose, lst(joints)))
                tcp, j0 = moves[3]
                frame = RDK.Item('Part1 reference', ITEM_TYPE_FRAME)
                frame2 = RDK.Item('BaseStation', ITEM_TYPE_FRAME)
                tool = RDK.Item('Spindle', ITEM_TYPE_TOOL)
                tool2 = robot.AddTool(tool.PoseTool() * transl(0, 0, 10), 'Gun')
                tcp2 = frame.Pose() * transl(0, 0, 0) * tcp
                robot.setPoseTool(tool)
                robot.setPoseFrame(frame)
                robot.setJoints(j0)
                info['joints'] = j0
            else:
                tool = robot.AddTool(transl(0, 0, 120) * roty(pi / 12), 'Tool 1')
                tool2 = robot.AddTool(transl(10, 0, 150), 'Gun')
                robot.setPoseTool(tool)
                robot.setJoints(j0)
                frame = RDK.AddFrame('Work Frame')            # station root: frame pose = absolute pose
                frame2 = RDK.AddFrame('Frame 2')
                tcp_abs = robot.PoseAbs() * robot.SolveFK(j0) * tool.PoseTool()   # TCP at j0 (PoseAbs: the robot base)
                frame.setPose(transl(tcp_abs.Pos()) * rotz(pi / 6))
                frame2.setPose(transl(tcp_abs.Pos()) * transl(40, -30, 20))
                tcp = frame.Pose().inv() * tcp_abs
                tcp2 = frame2.Pose().inv() * tcp_abs
                robot.setPoseFrame(frame)
            s = 40.0

            RDK.setSimulationSpeed(500)

            def ctarget(name, fr, pose):
                # a taught target: its stored joints are the IK solution RoboDK reaches it with from j0
                t = RDK.AddTarget(name, fr, robot)
                t.setAsCartesianTarget()
                t.setPose(pose)
                t.setJoints(j0)
                robot.setJoints(j0)
                try:
                    robot.MoveJ(t)
                    t.setJoints(lst(robot.Joints()))
                except Exception as e:
                    print('teach failed', name, e, flush=True)   # gantry-mounted arm: keep the joints of the path point
                robot.setJoints(j0)
                return t
            jt = RDK.AddTarget('Start Joints', frame, robot)
            jt.setAsJointTarget()
            jt.setJoints(j0)
            t1 = ctarget('Target 1', frame, tcp)
            t2 = ctarget('Target 2', frame, transl(s, 0, 0) * tcp)
            t3 = ctarget('Target 3', frame, transl(s, s, 0) * tcp)
            t4 = ctarget('Target 4', frame, transl(0, s, 10) * tcp)
            t5 = ctarget('Target 5', frame2, transl(0, 0, -20) * tcp2)
            tj = ctarget('tmp joint line', frame, transl(s / 2, -s / 2, 5) * tcp)
            robot.setJoints(lst(tj.Joints()))             # joints of a point no other move goes to
            tj.Delete()
            jl = RDK.AddTarget('Joint Line', frame, robot)
            jl.setAsJointTarget()
            jl.setJoints(lst(robot.Joints()))
            robot.setJoints(j0)

            sub = RDK.AddProgram('SubProg', robot)
            sub.setPoseFrame(frame)
            sub.setPoseTool(tool)
            sub.MoveL(t1)
            sub.RunInstruction('Inside sub', INSTRUCTION_COMMENT)

            thr = RDK.AddProgram('ThreadProg', robot)
            thr.RunInstruction('Thread body', INSTRUCTION_COMMENT)

            spaced = RDK.AddProgram('Spaced Name', robot)
            spaced.RunInstruction('Spaced body', INSTRUCTION_COMMENT)

            longp = RDK.AddProgram('Long_Program Name12', robot)
            longp.RunInstruction('Long body', INSTRUCTION_COMMENT)
            longp.setPoseTool(tool2)
            longp.MoveL(t3)

            prog = RDK.AddProgram('Feature', robot)
            steps = [
                ('frame', lambda: prog.setPoseFrame(frame)),
                ('tool', lambda: prog.setPoseTool(tool)),
                ('movej_joint', lambda: prog.MoveJ(jt)),
                ('speed', lambda: prog.setSpeed(250)),
                ('rounding', lambda: prog.setRounding(5)),
                ('movej_cart', lambda: prog.MoveJ(t1)),
                ('movel_cart', lambda: prog.MoveL(t2)),
                ('movel_joint', lambda: prog.MoveL(jl)),
                ('movec', lambda: prog.MoveC(t3, t4)),
                ('fine', lambda: prog.setRounding(-1)),
                ('speed_joints', lambda: prog.setSpeed(-1, 45)),
                ('speed_accel', lambda: prog.setSpeed(120, -1, 800)),
                ('movel_back', lambda: prog.MoveL(t1)),
                ('movel_same', lambda: prog.MoveL(t1)),
                ('movel_rot', lambda: prog.MoveL(t1.Pose() * rotz(pi / 9))),
                ('movej_same', lambda: prog.MoveJ(t1)),
                ('speed_all', lambda: (prog.setSpeed(300, 60, 1500, 200), prog.setSpeed(-1, -1, -1, 150), prog.setSpeed(-1, -1, 50))),
                ('do', lambda: (prog.setDO('5', 1), prog.setDO('3', 0))),
                ('wait', lambda: prog.waitDI('2', 1)),
                ('wait_timeout', lambda: prog.waitDI('4', 0, 2500)),
                ('pause', lambda: prog.Pause(1500)),
                ('stop', lambda: prog.Pause()),
                ('comment', lambda: prog.RunInstruction('A comment line', INSTRUCTION_COMMENT)),
                ('message', lambda: prog.RunInstruction('A message', INSTRUCTION_SHOW_MESSAGE)),
                ('call', lambda: (prog.RunInstruction('SubProg', INSTRUCTION_CALL_PROGRAM), prog.RunInstruction('Spaced Name', INSTRUCTION_CALL_PROGRAM))),
                ('call_external', lambda: prog.RunInstruction('ExternalProg', INSTRUCTION_CALL_PROGRAM)),
                ('call_more', lambda: (prog.RunInstruction('Long_Program Name12', INSTRUCTION_CALL_PROGRAM),
                                       prog.RunInstruction('Another Long_External Name', INSTRUCTION_CALL_PROGRAM),
                                       prog.RunInstruction('SetRPM(700)', INSTRUCTION_CALL_PROGRAM),
                                       prog.RunInstruction('Ab_c', INSTRUCTION_CALL_PROGRAM))),
                ('code', lambda: prog.RunInstruction('custom_code(1)', INSTRUCTION_INSERT_CODE)),
                ('thread', lambda: prog.RunInstruction('ThreadProg', INSTRUCTION_START_THREAD)),
                ('ao', lambda: prog.setAO('2', 3.5)),
                ('frame2', lambda: (prog.setPoseFrame(frame2), prog.setPoseTool(tool2))),
                ('movej_t5', lambda: prog.MoveJ(t5)),
                ('movej_inline', lambda: prog.MoveJ(j0)),
                ('movel_inline', lambda: prog.MoveL(t5.Pose() * transl(0, 0, -20))),
                ('back', lambda: (prog.setPoseTool(tool), prog.setPoseFrame(frame), prog.MoveJ(jt))),
            ]
            skip = [x for x in os.environ.get('BUILTIN_SKIP', '').split(',') if x]
            for key, fn in steps:
                if key not in skip:
                    fn()
            info['skipped'] = skip
            progs = [sub, spaced, thr, longp, prog]
            info['targets'] = {}
            for t in [jt, t1, t2, t3, t4, t5, jl]:
                info['targets'][t.Name()] = {'pose': m16(t.Pose()), 'joints': lst(t.Joints()), 'isJoint': t.isJointTarget(), 'frame': t.Parent().Name()}
            info['frames'] = {f.Name(): {'pose': m16(f.Pose()), 'parent': f.Parent().Name()} for f in [frame, frame2]}
            info['tools'] = {t.Name(): m16(t.PoseTool()) for t in [tool, tool2]}
        for p in progs:
            pdir = os.path.join(rdir, ''.join(ch if ch.isalnum() or ch in '-_' else '_' for ch in p.Name()))
            info['programs'][p.Name()] = dump_prog(p, pdir)
        json.dump(info, open(os.path.join(rdir, 'station.json'), 'w', encoding='utf-8'), indent=1, default=str)
        print(brand, '->', {k: v['files'] for k, v in info['programs'].items()}, flush=True)
finally:
    try:
        RDK.CloseRoboDK()
    except Exception:
        # a modal popup blocked the API: kill only the instance listening on this script's port
        import subprocess
        ps = ("Get-CimInstance Win32_Process -Filter \"Name='RoboDK.exe'\" | Where-Object { $_.CommandLine -like '*-PORT=%d*' } "
              "| ForEach-Object { Stop-Process -Id $_.ProcessId -Force }" % port)
        subprocess.run(['powershell', '-NoProfile', '-Command', ps])
