from __future__ import division
import rospy
import actionlib
import sys
import time
import numpy as np
import serial
from ur_msgs.srv import SetIO
import copy
import pickle
import argparse
from utils import TrajectoryClient as TC
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

# ser = serial.Serial('/dev/ttyACM0', 9600)


def main():
	rospy.init_node("pressure_calibration")
	mover = TC()
	rate = rospy.Rate(100)


	while not rospy.is_shutdown():
		
		P = input("Pressures: ")

		signal_P1 = (8*P/15 + 4)/1000
		signal_P2 = (8*P/15 + 4)/1000

		# print(signal_P1, signal_P2)
		
		mover.analog_io(0, 0, signal_P1)
		mover.analog_io(0, 1, signal_P2)
		rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass


