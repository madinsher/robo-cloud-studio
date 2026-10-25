import sys

try:
    import kortex_api
except Exception as e:
    try:
        from robodk import robolink
        from pathlib import Path
        sys.path.append((Path(robolink.getPathRoboDK()) / "../../Python/").resolve().as_posix())
        import kortex_api
    except Exception as e:
        print(e)
        raise ImportError('Unable to find the kortex_api module. Are you using the right Python version? Make sure the library is installed in C:/RoboDK/Python and/or in PYTHONPATH. Contact us at info@robodk.com if the issue persist.')

def print_message(arg):
    print(arg)
def UpdateStatus(arg):
    pass
ROBOTCOM_UNKNOWN = -1000
ROBOTCOM_CONNECTION_PROBLEMS = -3
ROBOTCOM_DISCONNECTED = -2
ROBOTCOM_NOT_CONNECTED = -1
ROBOTCOM_READY = 0
ROBOTCOM_WORKING = 1
ROBOTCOM_WAITING = 2
nDOFs_MIN = 6


class ComRobot:
    """Robot class for programming Kinova robots"""
    LAST_MSG = None  # Keep a copy of the last message received
    CONNECTED = False  # Connection status is known at all times
    KINOVAAPI = None
    BUFFER_SIZE = None
    #TIMEOUT = None
    #Speed and accelerations
    LINEARSPEED = 100
    LINEARACEL = 30
    JOINTSPEED  = 100
    JOINTACEL  = 80
    LAST_TARGET_JOINTS = []

    def check_for_end_or_abort(self,e):
        """Return a closure checking for END or ABORT notifications

        Arguments:
        e -- event to signal when the action is completed
            (will be set when an END or ABORT occurs)
        """
        from kortex_api.autogen.messages import Base_pb2

        def check(notification, e=e):
            print("EVENT : " + \
                Base_pb2.ActionEvent.Name(notification.action_event))
            if notification.action_event == Base_pb2.ACTION_END \
                    or notification.action_event == Base_pb2.ACTION_ABORT:
                e.set()

        return check


    def joints_to_kinova(self,jnts):
        jnts2 = []
        for j in jnts:
            jnts2.append(j+360)
        return jnts2

    def joints_from_kinova(self,jnts):
        jnts2 = []
        for j in jnts:
            jnts2.append(j-360)
        return jnts2

    # This is executed when the object is created
    def __init__(self):
        self.BUFFER_SIZE = 512  # bytes
        #self.TIMEOUT = 5 * 60  # seconds: it must be enough time for a movement to complete
        # self.TIMEOUT = 10 # seconds
        self.CONNECTED = False
        self.router = None
        self.base = None
        self.base_cyclic = None

    def __del__(self):
        self.disconnect()

    # Disconnect from robot
    def disconnect(self):
        if self.router is not None:
            try:
                self.router.finish()
            except:
                print("Failed to properly disconnect")

        self.CONNECTED = False
        self.router = None
        self.base = None
        self.base_cyclic = None
        return True

    # Connect to robot
    def connect(self, ip, port=-1):
        self.disconnect()
        print_message('Connecting to robot %s:%i' % (ip, port))
        # Create new socket connection
        UpdateStatus(ROBOTCOM_WORKING)
        #try:
        if True:
            from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
            from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient

            #self.router = DeviceConnection(ip, port=port, credentials=('admin', "admin"))
            self.router = DeviceConnection.createTcpConnection(ip)

            #with self.router:
            #    self.base = BaseClient(self.router)
            #    self.base_cyclic = BaseCyclicClient(self.router)

            # Start communication
            self.router.start()

            # Create required services
            self.base = BaseClient(self.router.router)
            self.base_cyclic = BaseCyclicClient(self.router.router)


        #except Exception as e:
        #    print_message(str(e))
        #    return False

        self.CONNECTED = True
        sys.stdout.flush()
        return True

    def MoveJ(self,coordArray):
        # Joint move to the provided angles
        from kortex_api.autogen.messages import Base_pb2
        print("Starting angular action movement ...")
        action = Base_pb2.Action()
        action.name = "Angular action movement"
        action.application_data = ""

        actuator_count = self.base.GetActuatorCount()

        # Set the joint angles
        for i, joint_id in enumerate(range(actuator_count.count)):
            joint_angle = action.reach_joint_angles.joint_angles.joint_angles.add()
            joint_angle.joint_identifier = joint_id
            offset = 0.0
            scale = 1.0
            #if joint_id == 3:
            #    # Requires a 90 degree offset
            #    offset = 90

            #if joint_id == 0 or joint_id == 5:
            #    # Reverse rotation of the first joint
            #    scale *= -1

            #if joint_id == 5:
            #    # Kinova robot must operate from -149 to 149
            #    if coordArray[i] < -149 or coordArray[i] > 149:
            #        offset -= 180

            #if scale * coordArray[i] + offset < 0:
            #    offset += 360

            joint_angle.value = scale * coordArray[i] + offset
            #if joint_angle.value > 360:
            #    joint_angle.value -= 360
            #if joint_angle.value < 0:
            #    joint_angle.value += 360

        import threading

        e = threading.Event()
        notification_handle = self.base.OnNotificationActionTopic(
            self.check_for_end_or_abort(e),
            Base_pb2.NotificationOptions()
        )

        print("Executing action")
        self.base.ExecuteAction(action)

        #print("Waiting for movement to finish ...")
        while True:
            finished = e.wait(0.02)
            if finished:
                #print("Angular movement completed")
                self.base.Unsubscribe(notification_handle)
                return True

            # Monitor joints
            if MONITOR_MOVE:
                jnts = self.getJoints()
                print_joints(jnts, True)
                pass

        self.base.Unsubscribe(notification_handle)
        return False

    def MoveL(self,xyzabc):
        import threading
        """Linear move (joint values provided in degrees)"""
        # Create new action
        from kortex_api.autogen.messages import Base_pb2
        action = Base_pb2.Action()
        action.name = "Linear move from RoboDK"
        action.application_data = ""

        x,y,z,rx,ry,rz = xyzabc[:6]
        cartesian_pose = action.reach_pose.target_pose
        cartesian_pose.x = x
        cartesian_pose.y = y
        cartesian_pose.z = z
        cartesian_pose.theta_x = rx
        cartesian_pose.theta_y = ry
        cartesian_pose.theta_z = rz

        e = threading.Event()
        notification_handle = self.base.OnNotificationActionTopic(
            self.check_for_end_or_abort(e),
            Base_pb2.NotificationOptions()
        )

        #print("Executing action")
        self.base.ExecuteAction(action)

        #print("Waiting for movement to finish ...")
        while True:
            finished = e.wait(0.02)
            if finished:
                #print("Angular movement completed")
                self.base.Unsubscribe(notification_handle)
                return True

            # Monitor joints
            if MONITOR_MOVE:
                jnts = self.getJoints()
                print_joints(jnts, True)
                pass

        self.base.Unsubscribe(notification_handle)
        return False

    def MoveC(self,joints):
        raise Exception("Circular move not implemented")
        #try:
        #    self.UARMAPI.set_mode(mode=1) #Mode 1 corresponds to moving
        #    self.UARMAPI.motion_enable(True)
        #    self.UARMAPI.move_circle(pose1=joints[0:6], pose2=joints[7:], percent=50, speed=self.JOINTSPEED, mvacc=self.JOINTACELL, #wait=True)
            #self.LAST_TARGET_JOINTS = joints
        #except Exception as e:
        #    print_message(str(e))
        #    return False
        #return True

    def GripperPosition(self, gripperPosition):
        # Open the gripper the indicated amount
        # Passed:  Integer position between 0 (open) and 1 (closed)

        # Create the GripperCommand we will send
        gripper_command = Base_pb2.GripperCommand()
        finger = gripper_command.gripper.finger.add()

        # Move the gripper
        gripper_command.mode = Base_pb2.GRIPPER_POSITION
        finger.value = gripperPosition
        finger.finger_identifier = 1

        print("Actuating gripper to {:0.2f}...".format(finger.value))
        self.base.SendGripperCommand(gripper_command)

        # Wait for reported position to be reached or speed reaches zero
        gripper_request1 = Base_pb2.GripperRequest()
        gripper_request1.mode = Base_pb2.GRIPPER_POSITION
        gripper_request2 = Base_pb2.GripperRequest()
        gripper_request2.mode = Base_pb2.GRIPPER_SPEED
        while True:
            gripper_measure = self.base.GetMeasuredGripperMovement(gripper_request1)
            gripper_speed = self.base.GetMeasuredGripperMovement(gripper_request2)
            if len(gripper_measure.finger):
                print("Current position is : {0}".format(gripper_measure.finger[0].value))
                if abs(gripper_measure.finger[0].value - gripperPosition) < 0.01:
                    break
                if gripper_speed == 0.0:
                    break
            else: # Else, no finger present in answer, end loop
                break


    def getJoints(self):
        actuator_count = self.base.GetActuatorCount()
        # TODO: Debug data:
        print(actuator_count)
        print("TODO: DEBUG DATA")
        joints = actuator_count.values
        return joints_from_kinova(joints)

    def setSpeed(self, speed_values):
        # speed_values[0] = speed_values[0] # linear speed in mm/s
        # speed_values[1] = speed_values[1] # joint speed in mm/s
        # speed_values[2] = speed_values[2] # linear acceleration in mm/s2
        # speed_values[3] = speed_values[3] # joint acceleration in deg/s2
        if (speed_values[0] != -1):
            self.LINEARSPEED = speed_values[0]

        if (speed_values[1] != -1):
            self.JOINTSPEED = speed_values[1]

        if (speed_values[2] != -1):
            self.LINEARACEL = speed_values[2]

        if (speed_values[3] != -1):
            self.JOINTACEL = speed_values[3]

        return True

    def setTool(self,tool_pose):
        #self.UARMAPI.set_tcp_offset(tool_pose)
        return True

    def Pause(self,timeMS):
        import time
        time.sleep(timeMS/1000)
        return True

    def setRounding(self,rounding):
        return True

    def setDO(self,digital_IO_State):
        return False

    def WaitDI(self,digital_IO_Num):
        return False

        #import time
        #start = time.time()
        #ioNumber = digital_IO_Num[0]
        #ioState = self.UARMAPI.get_tgpio_digital(ioNumber)
        #desiredState = digital_IO_Num[1]
        #try:
        #    timeout = digital_IO_Num[2]
        #except Exception as e:
        #    e = e
        #    timeout = 0
class DeviceConnection:
    TCP_PORT = 10000
    UDP_PORT = 10001

    @staticmethod
    def createTcpConnection(ip, username="admin", password="admin"):
        """
        returns RouterClient required to create services and send requests to device or sub-devices,
        """
        print("Using TCP connection")
        return DeviceConnection(ip, port=DeviceConnection.TCP_PORT, credentials=(username, password))

    @staticmethod
    def createUdpConnection(ip, username="admin", password="admin"):
        """
        returns RouterClient that allows to create services and send requests to a device or its sub-devices @ 1khz.
        """
        print("Using UDP connection")
        return DeviceConnection(ip, port=DeviceConnection.UDP_PORT, credentials=(username, password))

    def __init__(self, ipAddress, port=TCP_PORT, credentials = ("","")):
        from kortex_api.TCPTransport import TCPTransport
        from kortex_api.UDPTransport import UDPTransport
        from kortex_api.RouterClient import RouterClient, RouterClientSendOptions

        self.ipAddress = ipAddress
        self.port = port
        self.credentials = credentials

        self.sessionManager = None

        # Setup API
        self.transport = TCPTransport() if port == DeviceConnection.TCP_PORT else UDPTransport()
        self.router = RouterClient(self.transport, RouterClient.basicErrorCallback)

    def start(self):
        self.transport.connect(self.ipAddress, self.port)

        if (self.credentials[0] != ""):
            from kortex_api.SessionManager import SessionManager
            from kortex_api.autogen.messages import Session_pb2

            session_info = Session_pb2.CreateSessionInfo()
            session_info.username = self.credentials[0]
            session_info.password = self.credentials[1]
            session_info.session_inactivity_timeout = 10000   # (milliseconds)
            session_info.connection_inactivity_timeout = 2000 # (milliseconds)

            self.sessionManager = SessionManager(self.router)
            print("Logging as", self.credentials[0], "on device", self.ipAddress)
            self.sessionManager.CreateSession(session_info)

        return self.router

    def finish(self):
        if self.sessionManager != None:
            from kortex_api.RouterClient import RouterClient, RouterClientSendOptions

            router_options = RouterClientSendOptions()
            router_options.timeout_ms = 1000

            self.sessionManager.CloseSession(router_options)

        self.transport.disconnect()

    # Called when entering 'with' statement
    def __enter__(self):
        return self.start()

    # Called when exiting 'with' statement
    def __exit__(self, exc_type, exc_value, traceback):
        self.finish()


DensoCobotta = ComRobot()
DensoCobotta.connect('127.0.0.1', 12345)



# Program Start,JointTargets
def JointTargets():
    global DensoCobotta
    # Generating program: JointTargets

    #Program generated by VerticalBot Studio for Denso Cobotta on 25/10/2026 12:00:00
    #Using nominal kinematics.
    #Denso Cobotta ref frame set to 100.000, -50.000, 20.000,  0.000, 0.000, 30.000
    DensoCobotta.setTool([10.000, 0.000, 120.000,  0.000, 15.000, 0.000])

    DensoCobotta.MoveJ([10.000000,20.000000,90.000000,30.000000,45.000000,60.000000])
    #Linear movement using joint targets is not supported. Change the target type to cartesian or use a joint movement.
    DensoCobotta.MoveJ([18.000000,24.000000,90.000000,30.000000,45.000000,60.000000])
    DensoCobotta.MoveL([10.000000,20.000000,90.000000,30.000000,45.000000,60.000000,250.703, 15.135, 51.473,  174.608, 31.200, 102.556])
    #Linear movement using joint targets is not supported. Change the target type to cartesian or use a joint movement.
    DensoCobotta.setRounding(5.000)
    DensoCobotta.MoveJ([10.000000,20.000000,90.000000,30.000000,45.000000,60.000000])
    #Linear movement using joint targets is not supported. Change the target type to cartesian or use a joint movement.
    DensoCobotta.MoveJ([18.000000,24.000000,90.000000,30.000000,45.000000,60.000000])
    DensoCobotta.MoveL([10.000000,20.000000,90.000000,30.000000,45.000000,60.000000,250.703, 15.135, 51.473,  174.608, 31.200, 102.556])
    #Linear movement using joint targets is not supported. Change the target type to cartesian or use a joint movement.
    DensoCobotta.setRounding(-1.000)
    DensoCobotta.MoveJ([10.000000,20.000000,90.000000,30.000000,45.000000,60.000000])
    return

#Call main
JointTargets()
