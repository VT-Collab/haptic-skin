import rospy
import actionlib
import sys
import time
import numpy as np
import pygame
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
import pickle
import argparse
# import os
from positions import HOME
from utils import JoystickControl, TrajectoryClient, go2home

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


parser = argparse.ArgumentParser(description='Collecting offline demonstrations')
parser.add_argument('--who', help='expert vs. user(i)', type=str)
parser.add_argument('--feature', help='XY, Z, ROT', type=str)
parser.add_argument('--trial', help='demonstration index', type=str, default='0')
args = parser.parse_args()



# class RecordClient(object):

#     def __init__(self):
#         # Subscribers to update joint state
#         self.joint_sub = rospy.Subscriber('/joint_states', JointState, self.joint_states_cb)
#         self.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",\
#                             "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
#         self.base_link = "base_link"
#         self.end_link = "wrist_3_link"
#         self.joint_states = None
#         self.robot_urdf = URDF.from_parameter_server()
#         self.kdl_kin = KDLKinematics(self.robot_urdf, self.base_link, self.end_link)
#         self.script_pub = rospy.Publisher('/ur_hardware_interface/script_command', \
#                                             String, queue_size=100)

#         # Gripper action and client
#         action_name = rospy.get_param('~action_name', 'command_robotiq_action')
#         self.robotiq_client = actionlib.SimpleActionClient(action_name, \
#                                 CommandRobotiqGripperAction)
#         self.robotiq_client.wait_for_server()
     
#         # Initialize gripper
#         goal = CommandRobotiqGripperGoal()
#         goal.emergency_release = False
#         goal.stop = False
#         goal.position = 1.00
#         goal.speed = 0.1
#         goal.force = 5.0
#         # Sends the goal to the gripper.
#         self.robotiq_client.send_goal(goal)

    # def joint_states_cb(self, msg):
    #     try:
    #         states = list(msg.position)
    #         states[2], states[0] = states[0], states[2]
    #         self.joint_states = tuple(states)
    #     except:
    #         pass

    # def send_cmd(self, cmd):
    #     self.script_pub.publish(cmd)

    # def actuate_gripper(self, pos, speed, force):
    #     Robotiq.goto(self.robotiq_client, pos=pos, speed=speed, force=force, block=True)
    #     return self.robotiq_client.get_result()

    # def forward_kinematics(self, s, end_link="wrist_3_link", base_link="base_link"):
    #     return self.kdl_kin.forward(s)



def main():

    if args.who == "expert":
        filename = "data/demos/" + args.feature + "/" + args.who + "_" + args.trial + ".pkl"
    elif args.who[0:4] == "user":
        filename = "data/demos/" + args.feature + "/" + args.who + ".pkl"


    rospy.init_node("UR10")
    rate = rospy.Rate(100)    
    UR10 = TrajectoryClient()
    joystick = JoystickControl()

    while not UR10.joint_states:
        pass

    rospy.sleep(1)
    go2home(HOME)
    rospy.sleep(2)

    UR10.actuate_gripper(1, 0.1, 1)
    gripper_open = True
    rospy.sleep(0.5)    
    print("[*] Press A to START Recording")
    print("[*] Press B to STOP Recording")

    record = False
    segment = 0
    step_time = 0.1
    gripper_open = True

    data = []
    while not rospy.is_shutdown():

        s = list(UR10.joint_states)
        
        A, B, X, Y, start = joystick.getInput()
        if X and gripper_open:
            UR10.actuate_gripper(0.05, 0.1, 1)
            gripper_open = False
        if Y and not gripper_open:
            UR10.actuate_gripper(1, 0.1, 1)
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
            data.append(s)
            last_time = curr_time

        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass


