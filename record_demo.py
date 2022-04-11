import rospy
import actionlib
import sys
import time
import numpy as np
import pygame
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
import copy
import pickle
import torch
import argparse
from positions import HOME, RETURN, Goals

from std_msgs.msg import Float64MultiArray, String

from robotiq_2f_gripper_msgs.msg import (
    CommandRobotiqGripperFeedback, 
    CommandRobotiqGripperResult, 
    CommandRobotiqGripperAction, 
    CommandRobotiqGripperGoal
)

from robotiq_2f_gripper_control.robotiq_2f_gripper_driver import (
    Robotiq2FingerGripperDriver as Robotiq
)

from control_msgs.msg import (
    GripperCommandAction,
    GripperCommandGoal,
    GripperCommand
)

from sensor_msgs.msg import (
    JointState
)
from geometry_msgs.msg import(
    TwistStamped,
    Twist
)


class JoystickControl(object):

    def __init__(self):
        pygame.init()
        self.gamepad = pygame.joystick.Joystick(0)
        self.gamepad.init()
        self.toggle = False
        self.action = None

    def getInput(self):
        pygame.event.get()
        START = self.gamepad.get_button(7)
        A = self.gamepad.get_button(0)
        B = self.gamepad.get_button(1)
        X = self.gamepad.get_button(2)
        Y = self.gamepad.get_button(3)
        return A, B, X, Y, START


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
                                            String, queue_size=100)

        # Gripper action and client
        action_name = rospy.get_param('~action_name', 'command_robotiq_action')
        self.robotiq_client = actionlib.SimpleActionClient(action_name, \
                                CommandRobotiqGripperAction)
        self.robotiq_client.wait_for_server()
        # Initialize gripper
        goal = CommandRobotiqGripperGoal()
        goal.emergency_release = False
        goal.stop = False
        goal.position = 1.00
        goal.speed = 0.1
        goal.force = 5.0
        # Sends the goal to the gripper.
        self.robotiq_client.send_goal(goal)

    def joint_states_cb(self, msg):
        try:
            states = list(msg.position)
            states[2], states[0] = states[0], states[2]
            self.joint_states = tuple(states)
        except:
            pass

    def send_cmd(self, cmd):
        self.script_pub.publish(cmd)

    def actuate_gripper(self, pos, speed, force):
        Robotiq.goto(self.robotiq_client, pos=pos, speed=speed, force=force, block=True)
        return self.robotiq_client.get_result()

    def forward_kinematics(self, s, end_link="wrist_3_link", base_link="base_link"):
        return self.kdl_kin.forward(s)



def main():

    parser = argparse.ArgumentParser(description='Collecting offline demonstrations')
    parser.add_argument('--trial', type=str, default='0')
    parser.add_argument('--set', help='XY, Z, ROT', type=str, default=None)
    args = parser.parse_args()

    filename = "demos/" + args.set + "/trail_" + args.trial + ".pkl"
    data = []
    rospy.init_node("recorder")
    rate = rospy.Rate(100)    
    recorder = RecordClient()
    joystick = JoystickControl()

    while not recorder.joint_states:
        pass

    rospy.sleep(1)

    # for waypoint in RETURN:
    #     recorder.send_cmd('movel(' + str(waypoint) + ')')
    #     rospy.sleep(1)

    recorder.send_cmd('movel(' + str(HOME) + ')')
    rospy.sleep(2)
    recorder.actuate_gripper(1, 0.1, 1)
    gripper_open = True
    rospy.sleep(0.5)    
    print("[*] Press A to START Recording")
    print("[*] Press B to STOP Recording")

    record = False
    segment = 0
    step_time = 0.1
    gripper_open = True


    goal = Goals['Goal4']
    while not rospy.is_shutdown():


        s = list(recorder.joint_states)
        # print(recorder.forward_kinematics(s))
        
        
        A, B, X, Y, start = joystick.getInput()
        if X and gripper_open:
            recorder.actuate_gripper(0.05, 0.1, 1)
            gripper_open = False
        if Y and not gripper_open:
            recorder.actuate_gripper(1, 0.1, 1)
            gripper_open = True
        if record and B:
            pickle.dump(data, open(filename, "wb"))
            return True
        elif not record and A:
            record = True
            last_time = time.time()
            start_time = time.time()
            time_last_segment = time.time()
            print("[*] Recording...")
        curr_time = time.time()
        if start and record and curr_time - time_last_segment > 0.5:
            segment += 1
            time_last_segment = time.time()
        if record and curr_time - last_time > step_time:
            data.append([s, goal])
            last_time = curr_time

        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass


