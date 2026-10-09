"""Same code you would write for RoboDK — runs against VerticalBot Studio.
Start the server (npm run server) and open the studio with ?server=ws://localhost:20500 to see it live."""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from robodk.robolink import *
from robodk.robomath import *

RDK = Robolink()
print(RDK.Version())
robot = RDK.Item("", ITEM_TYPE_ROBOT)
if not robot.Valid():
    robot = RDK.AddRobot("UR10e")
print("Robot:", robot.Name(), "joints:", robot.Joints())

frame = RDK.AddFrame("Frame from Python")
frame.setPose(transl(600, 0, 300))
robot.setPoseFrame(frame)
prog = RDK.AddProgram("Python program", robot)
prog.setSpeed(300, 60)
home = RDK.AddTarget("Home", frame, robot)
home.setAsJointTarget()
prog.MoveJ(home)
pose = robot.Pose()
for i, (dx, dy) in enumerate([(0, 0), (150, 0), (150, 150), (0, 150), (0, 0)]):
    t = RDK.AddTarget("Square %d" % i, frame, robot)
    t.setPose(pose * transl(dx, dy, 0))
    prog.MoveL(t)
prog.RunInstruction("Square done", INSTRUCTION_SHOW_MESSAGE)
valid, time_s, dist_mm, ratio, msg = prog.Update()
print("Program valid instructions: %s, cycle time: %.2f s, distance: %.0f mm (%s)" % (valid, time_s, dist_mm, msg))
ok, code, name = prog.MakeProgram("", post="KUKA_KRC4")
print(code[:400])
