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
from positions import HOME
from utils import JoystickControl, TrajectoryClient, go2home

parser = argparse.ArgumentParser(description='Collecting offline demonstrations')
parser.add_argument('--who', help='expert vs. user(i)', type=str)
parser.add_argument('--feature', help='XY, Z, ROT', type=str)
parser.add_argument('--trial', help='demonstration index', type=str, default='0')
args = parser.parse_args()

if args.who == "expert":
    filename = "data/demos/" + args.feature + "/" + args.who + "_" + args.trial + ".pkl"
elif args.who[0:4] == "user":
    filename = "data/demos/" + args.feature + "/" + args.who + ".pkl"

def main():

    rospy.init_node("UR10")
    rate = rospy.Rate(100)    
    UR10 = TrajectoryClient()
    joystick = JoystickControl()

    while not UR10.joint_states:
        pass

    # send robot to home 
    rospy.sleep(1)
    go2home(HOME)
    rospy.sleep(2)

    # open gripper
    UR10.actuate_gripper(1, 0.1, 1)
    gripper_open = True
    rospy.sleep(0.5)    

    print("[*] Press A to START Recording")
    print("[*] Press B to STOP Recording")

    data = []
    record = False
    step_time = 0.1

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
            print("[*] Saved Recording")
            return True
        elif not record and A:
            record = True
            last_time = time.time()
            start_time = time.time()
            time_last_segment = time.time()
            print("[*] Recording...")
        curr_time = time.time()
        if record and curr_time - last_time > step_time:
            data.append(s)
            last_time = curr_time

        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass


