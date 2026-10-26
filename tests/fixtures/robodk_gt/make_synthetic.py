"""Build the synthetic RoboDK station used by tests/robodk_gt.test.ts (fixture synthetic/T1-instructions.rdk).

    C:/RoboDK/Python37/python -I make_synthetic.py <out.rdk> <port>

The station is made by RoboDK 5.9.4 itself through its official API and holds what the Library examples do not:
digital/analog IO and waits, pause until resume, rounding, joint speed/acceleration, every RunInstruction variant,
inline joint and pose moves, MoveC, duplicate target names in two frames, a station Python program, a station
parameter and Cyrillic item names. Dump the ground truth afterwards with dump_gt.py (DUMP_NO_MAKE=1).
"""
import sys, os, tempfile
sys.path.insert(0, r'C:\RoboDK\Python')
from robodk.robolink import *
from robodk.robomath import *

out, port = sys.argv[1], int(sys.argv[2])
RDK = Robolink(args=['-NOSPLASH', '-HIDDEN', '-NEWINSTANCE', '-NOUI'], port=port)
RDK.AddStation('T1 Синтетика')
robot = RDK.AddFile(r'C:\RoboDK\Library\ABB-IRB-1600-6-1-45.robot')
tool = robot.AddTool(transl(0, 0, 150) * roty(pi / 6), 'Инструмент 1')
frame = RDK.AddFrame('Рамка 1')
frame.setPose(transl(700, -200, 300))
frame2 = RDK.AddFrame('Frame B')
frame2.setPose(transl(600, 300, 200) * rotz(pi / 2))
robot.setPoseFrame(frame)
robot.setPoseTool(tool)
home = RDK.AddTarget('Дом', frame, robot)
home.setAsJointTarget()
home.setJoints([0, 10, 10, 0, 40, 0])
t1 = RDK.AddTarget('Цель 1', frame, robot)
t1.setPose(transl(0, 0, 0) * rotx(pi))
t2 = RDK.AddTarget('Target 2', frame, robot)
t2.setPose(transl(100, 0, 0) * rotx(pi))
t3 = RDK.AddTarget('Target 3', frame, robot)
t3.setPose(transl(100, 100, 0) * rotx(pi))
t2b = RDK.AddTarget('Target 2', frame2, robot)  # same name, other frame
t2b.setPose(transl(50, 0, 0) * rotx(pi))

py = os.path.join(tempfile.mkdtemp(), 'ПитонПрог.py')
with open(py, 'w', encoding='utf-8') as f:
    f.write('# Скрипт станции\nfrom robodk.robolink import *\nRDK = Robolink()\nprint("hello")\n')
RDK.AddFile(py)
RDK.setParam('T1_PARAM', '42')

sub = RDK.AddProgram('Подпрограмма', robot)
sub.MoveL(t2)
prog = RDK.AddProgram('Программа', robot)
prog.setPoseFrame(frame)
prog.setPoseTool(tool)
prog.setSpeed(150, 90, 500, 300)
prog.setRounding(5)
prog.MoveJ(home)
prog.MoveL(t1)
prog.MoveC(t2, t3)
prog.Pause(500)
prog.Pause()
prog.setDO('DO_5', 1)
prog.setAO('AO_1', 2.5)
prog.waitDI('DI_3', 1, 2000)
prog.waitDI('DI_4', 0)
prog.RunInstruction('SetRPM(70)', INSTRUCTION_CALL_PROGRAM)
prog.RunInstruction('Подпрограмма', INSTRUCTION_CALL_PROGRAM)
prog.RunInstruction('Подпрограмма', INSTRUCTION_START_THREAD)
prog.RunInstruction('ПитонПрог', INSTRUCTION_CALL_PROGRAM)
prog.RunInstruction('LaserOn', INSTRUCTION_INSERT_CODE)
prog.RunInstruction('Комментарий', INSTRUCTION_COMMENT)
prog.RunInstruction('Hello operator', INSTRUCTION_SHOW_MESSAGE)
prog.setPoseFrame(frame2)
prog.MoveL(t2b)
prog.MoveJ([10, -10, 20, 0, 40, 0])
prog.MoveL(transl(40, 20, 0) * rotx(pi))
prog.setRounding(-1)
prog.setSpeedJoints(45)
prog.setAccelerationJoints(200)
prog.setAcceleration(800)
prog.MoveJ(home)
RDK.Save(out)
print('saved', out, prog.InstructionCount(), flush=True)

# T1-collision.rdk: robot + box with the robot-box collision pairs of links 0..3 switched off (collision map layout)
RDK.AddStation('C2')
robot2 = RDK.AddFile(r'C:\RoboDK\Library\ABB-IRB-1600-6-1-45.robot')
box = RDK.AddFile(r'C:\RoboDK\Library\box.sld')
box.setName('B')
for j in range(4):
    RDK.setCollisionActivePair(COLLISION_OFF, robot2, box, j, 0)
col = os.path.join(os.path.dirname(out), 'T1-collision.rdk')
RDK.Save(col)
print('saved', col, flush=True)
RDK.CloseRoboDK()
