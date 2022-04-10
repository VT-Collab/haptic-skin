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

def analog_IO(fun, pin, state):
    rospy.wait_for_service('/ur_hardware_interface/set_io')
    try:
        set_io = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)
        set_io(fun = fun,pin = pin,state = state)
        print(state)
        print("Sending analog "+ str(pin)+ " pressure...")

    except rospy.ServiceException, e:
        print "Unable to send pressure command: %s"%e

def send_arduino(user_input):
	print("Sending arduino pressure...")
	num = str(user_input)
	b = num.encode('utf-8')
	ser.write(b)

def send_pressures(user_input):
	p1 = user_input[0]
	p2 = user_input[1]
	p3 = user_input[2]
	if ((p1 >= 0.) and(p2 >= 0.) and(p3 >= 0.) and(p1 <= 3.) and(p2 <= 3.) and(p3 <= 3.)):
		send_arduino(p1)
		state1 = p2/30.0
		state2 = p3/30.0
		analog_IO(3, 0, state1)
		analog_IO(3, 1, state2)

def pressure2current(p):
	return (8/15)*p + 4

# pressure calculations are wrong!
# this is what it should be: p = 15/8 I - 7.5 or I = 8/15 P + 4

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


