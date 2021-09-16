#!/usr/bin/env python

import rospy
import sys
import time
import numpy as np
import pygame
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
import copy
import pickle

from std_msgs.msg import Float64MultiArray, String

from sensor_msgs.msg import (
    JointState
)
from geometry_msgs.msg import(
    TwistStamped,
    Twist
)

from ur_dashboard_msgs.srv import(
    IsProgramRunning,
    Load,
    GetProgramState
)

HOME = [-1.45, -1.88, -1.80,-0.97, 1.54, -0.02]

STEP_SIZE_L = 0.15
STEP_SIZE_A = 0.2 * np.pi / 4
STEP_TIME = 0.01
DEADBAND = 0.1

class JoystickControl(object):

    def __init__(self):
        pygame.init()
        self.gamepad = pygame.joystick.Joystick(0)
        self.gamepad.init()
        self.toggle = False
        self.action = None

    def getInput(self):
        pygame.event.get()
        toggle_angular = self.gamepad.get_button(4)
        toggle_linear = self.gamepad.get_button(5)
        if not self.toggle and toggle_angular:
            self.toggle = True
        elif self.toggle and toggle_linear:
            self.toggle = False
        return self.getEvent()

    def getEvent(self):
        z1 = self.gamepad.get_axis(0)
        z2 = self.gamepad.get_axis(1)
        z3 = self.gamepad.get_axis(4)
        z = [z1, z2, z3]
        for idx in range(len(z)):
            if abs(z[idx]) < DEADBAND:
                z[idx] = 0.0
        stop = self.gamepad.get_button(7)
        gripper_open = self.gamepad.get_button(1)
        gripper_close = self.gamepad.get_button(0)
        return tuple(z), (gripper_open, gripper_close), stop

    def getAction(self, z):
        if self.toggle:
            self.action = (0, 0, 0, STEP_SIZE_A * -z[1], STEP_SIZE_A * -z[0], STEP_SIZE_A * -z[2])
        else:
            self.action = (STEP_SIZE_L * -z[1], STEP_SIZE_L * -z[0], STEP_SIZE_L * -z[2], 0, 0, 0)

class RecordClient(object):

    def __init__(self):
        # Subscribers to update joint state
        self.joint_sub = rospy.Subscriber('/joint_states', JointState, self.joint_states_cb)
        self.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",\
                            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        self.base_link = "base_link"
        self.end_link = "wrist_3_link"
        self.joint_states = None
        self.robot_urdf = URDF.from_parameter_server()
        self.kdl_kin = KDLKinematics(self.robot_urdf, self.base_link, self.end_link)
        self.script_pub = rospy.Publisher('/ur_hardware_interface/script_command', \
                                            String, queue_size=2)
        # self.load_program_cli = rospy.ServiceProxy('/ur_hardware_interface/dashboard/load_program', \
                                                    # Load)

    def joint_states_cb(self, msg):
        states = list(msg.position)
        states[2], states[0] = states[0], states[2]
        self.joint_states = tuple(states) 
    
    def xdot2qdot(self, xdot):
        J = self.kdl_kin.jacobian(self.joint_states)
        J_inv = np.linalg.pinv(J)
        return J_inv.dot(xdot)

    def send_cmd(self, cmd):
        self.script_pub.publish(cmd)

def main():
    filename = "data/" + sys.argv[1] + ".pkl"
    data = []
    rospy.init_node("recorder")
    rospy.sleep(2)
    recorder = RecordClient()
    joystick = JoystickControl()

    while not recorder.joint_states:
        pass

    record = False
    start_time = time.time()
    step_time = 0.1
    rate = rospy.Rate(1000)

    homing = False
    i = 0
    while not rospy.is_shutdown():
        if not homing:
            while i < 2:
                recorder.send_cmd('movel([-1.45, -1.88, -1.80,-0.97, 1.54, -0.02])')
                rospy.sleep(2)
                i += 1
            homing = True
            rospy.sleep(4)
            i = 0
            print("[*] Press Start")
        axes, buttons, start = joystick.getInput()
        if record and start:
            pickle.dump(data, open(filename, "wb"))
            print(data)
            return True
        elif not record and start:
            record = True
            curr_time = time.time()
            while start:
                axes, buttons, start = joystick.getInput()
            # while i < 2:
            #     recorder.send_cmd('free_mode()')
            #     rospy.sleep(2)
            #     i += 1
            print("[*] Ready")

        s = list(recorder.joint_states)
        curr_time = time.time()
        if record and curr_time - start_time >= step_time:
            data.append(s)
            start_time = curr_time

        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass


