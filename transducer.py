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

comm_arduino = serial.Serial('/dev/ttyACM0', baudrate=9600)


def analog_IO(fun, pin, state):
    rospy.wait_for_service('/ur_hardware_interface/set_io')
    try:
        set_io = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)
        set_io(fun = fun,pin = pin,state = state)
        print("Sending analog "+ str(pin)+ " pressure...")

    except rospy.ServiceException, e:
        print "Unable to send pressure command: %s"%e

def send_arduino(user_input):
	string = '<' + str(user_input) + '>'
	comm_arduino.write(string)

def send_pressures(user_input):
	p1 = user_input[0]
	p2 = user_input[1]
	p3 = user_input[2]
	if ((p1 >= 0.) and(p2 >= 0.) and(p3 >= 0.) and(p1 <= 10.) and(p2 <= 10.) and(p3 <= 10.)):
		send_arduino(p1)
		state1 = p2/30.0
		state2 = p3/30.0
		analog_IO(3, 0, state1)
		analog_IO(3, 1, state2)




def main():
	rospy.init_node("pressure_calibration")

	while not rospy.is_shutdown():
		user_input = input("Pressure: ")
		send_arduino(user_input)

		# send_send_arduino(user_input)

		analog_IO(3, 1, user_input/30.0)
		analog_IO(3, 0, user_input/30.0)



if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass


