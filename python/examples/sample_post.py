"""Minimal RoboDK-style post processor (same structure as the posts shipped with RoboDK)."""
from robodk.robomath import *


def pose_2_str(pose):
    x, y, z, r, p, w = Pose_2_KUKA(pose)
    return 'X %.3f,Y %.3f,Z %.3f,A %.3f,B %.3f,C %.3f' % (x, y, z, r, p, w)


class RobotPost(object):
    PROG_EXT = 'src'

    def __init__(self, robotpost=None, robotname=None, robot_axes=6, **kwargs):
        self.ROBOT_POST = robotpost
        self.ROBOT_NAME = robotname
        self.PROG = []
        self.LOG = ''
        self.nAxes = robot_axes
        self.SPEED = 0.25

    def ProgStart(self, progname):
        self.addline('DEF %s ( )' % progname)
        self.addline('$VEL.CP = %.3f' % self.SPEED)

    def ProgFinish(self, progname):
        self.addline('END')

    def ProgSave(self, folder, progname, ask_user=False, show_result=False):
        pass

    def ProgSendRobot(self, robot_ip, remote_path, ftp_user, ftp_pass):
        pass

    def MoveJ(self, pose, joints, conf_RLF=None):
        self.addline('PTP {A1 %.3f,A2 %.3f,A3 %.3f,A4 %.3f,A5 %.3f,A6 %.3f}' % tuple(joints[:6]))

    def MoveL(self, pose, joints, conf_RLF=None):
        self.addline('LIN {%s}' % pose_2_str(pose))

    def MoveC(self, pose1, joints1, pose2, joints2, conf_RLF_1=None, conf_RLF_2=None):
        self.addline('CIRC {%s},{%s}' % (pose_2_str(pose1), pose_2_str(pose2)))

    def setFrame(self, pose, frame_id=None, frame_name=None):
        self.addline('$BASE = {FRAME: %s}' % pose_2_str(pose))

    def setTool(self, pose, tool_id=None, tool_name=None):
        self.addline('$TOOL = {FRAME: %s}' % pose_2_str(pose))

    def Pause(self, time_ms):
        if time_ms < 0:
            self.addline('HALT')
        else:
            self.addline('WAIT SEC %.3f' % (time_ms * 0.001))

    def setSpeed(self, speed_mms):
        self.SPEED = speed_mms / 1000.0
        self.addline('$VEL.CP = %.3f' % self.SPEED)

    def setAcceleration(self, accel_mmss):
        self.addline('$ACC.CP = %.3f' % (accel_mmss / 1000.0))

    def setSpeedJoints(self, speed_degs):
        self.addline('$VEL_AXIS[1] = %.0f' % min(100, speed_degs / 3.6))

    def setAccelerationJoints(self, accel_degss):
        pass

    def setZoneData(self, zone_mm):
        self.addline('$APO.CDIS = %.1f' % max(0, zone_mm))

    def setDO(self, io_var, io_value):
        self.addline('$OUT[%s] = %s' % (io_var, 'TRUE' if io_value else 'FALSE'))

    def setAO(self, io_var, io_value):
        self.addline('$ANOUT[%s] = %s' % (io_var, io_value))

    def waitDI(self, io_var, io_value, timeout_ms=-1):
        self.addline('WAIT FOR $IN[%s] == %s' % (io_var, 'TRUE' if io_value else 'FALSE'))

    def RunCode(self, code, is_function_call=False):
        self.addline(code + '()' if is_function_call else code)

    def RunMessage(self, message, iscomment=False):
        self.addline('; ' + message if iscomment else 'MsgNotify("%s")' % message)

    def addline(self, newline):
        self.PROG.append(newline)

    def addlog(self, newline):
        self.LOG = self.LOG + newline + '\n'
