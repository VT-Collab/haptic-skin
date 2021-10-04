import rospy
import actionlib
import sys
import time
import numpy as np
import pygame
from urdf_parser_py.urdf import URDF
from ur_msgs.srv import SetIO
from pykdl_utils.kdl_kinematics import KDLKinematics
import copy
import pickle
import torch
from train_model import MLP
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


def analog_IO(fun, pin, state):
    rospy.wait_for_service('/ur_hardware_interface/set_io')
    try:
        set_io = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)
        set_io(fun = fun,pin = pin,state = state)
    except rospy.ServiceException, e:
        print "Unable to send pressure command: %s"%e


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


class Model(object):
    def __init__(self, task, segment):
        self.model = MLP()
        model_dict = torch.load("models/MLP_model_task" + str(task) + "_segment" + str(segment), map_location='cpu')
        self.model.load_state_dict(model_dict)
        self.model.eval

    def decoder(self, state):
        s_tensor = torch.FloatTensor(state)
        action = self.model.decoder(s_tensor).detach().numpy()
        return action


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


def scale_uncertainty(uncertainty, task):
    if task == 1:
        min_uncertainty = 0.005
        max_uncertainty = 0.0185
    if task == 2:
        min_uncertainty = 0.005
        max_uncertainty = 0.0175
    if task == 3:
        min_uncertainty = 0.005
        max_uncertainty = 0.020
    uncertainty = (uncertainty - min_uncertainty) / (max_uncertainty - min_uncertainty)
    if uncertainty > 1.0:
        uncertainty = 1.0
    elif uncertainty < 0.0:
        uncertainty = 0.0
    return uncertainty


def pressure_feedback(uncertainty):

    min_pressure = 1
    max_pressure = 3

    pressure = min_pressure + (max_pressure - min_pressure) * uncertainty
    if pressure > max_pressure:
        pressure = max_pressure
    if pressure < min_pressure:
        pressure = min_pressure

    state_a = pressure/30.0

    print(uncertainty, pressure)
    analog_IO(3, 0, state_a)


def main():

    parser = argparse.ArgumentParser(description='haptic pressure feedback on the flat table')
    parser.add_argument('--user', type=int, default=0)
    parser.add_argument('--task', type=int, default=0)
    parser.add_argument('--segment', type=int, default=0)
    parser.add_argument('--trial', type=int, default=0)
    args = parser.parse_args()

    filename = "demos/user" + str(args.user) + "/task/table" + str(args.task) + "_trial" + str(args.trial) + ".pkl"

    if args.trial == 1:
        HOME = [-1.45, -1.88, -1.80,-0.97, 1.54, -0.02]
    else:
        HOME = pickle.load(open("home.pkl", "rb"))

    data = []
    rospy.init_node("recorder")
    rate = rospy.Rate(100)
    recorder = RecordClient()
    joystick = JoystickControl()
    model = Model(args.task, args.segment)
    analog_IO(3, 0, 0.0)

    while not recorder.joint_states:
        pass

    rospy.sleep(1)
    recorder.send_cmd('movel(' + str(HOME) + ')')
    rospy.sleep(2)
    recorder.actuate_gripper(1, 0.1, 1)
    gripper_open = True
    rospy.sleep(0.5)
    print("[*] Press A to START Recording")
    print("[*] Press B to STOP Recording")

    record = False
    step_time = 0.05
    n_samples = 100
    start_time = time.time()

    while not rospy.is_shutdown():

        curr_time = time.time() - start_time

        A, B, X, Y, start = joystick.getInput()
        if start:
            pickle.dump(s, open("home.pkl", "wb"))
        if X and gripper_open:
            recorder.actuate_gripper(0.05, 0.1, 1)
            gripper_open = False
        if Y and not gripper_open:
            recorder.actuate_gripper(1, 0.1, 1)
            gripper_open = True
        if record and B:
            recorder.actuate_gripper(1, 0.1, 1)
            pickle.dump(data, open(filename, "wb"))
            print("I recorded this many data points:", len(data))
            analog_IO(3, 0, 0.0)
            return True
        elif not record and A:
            record = True
            last_time = time.time()
            start_time = time.time()
            print("[*] Recording...")
        s = list(recorder.joint_states)

        actions = []
        for idx in range(n_samples):
            actions.append(model.decoder(s))
        actions = np.asarray(actions)

        curr_time = time.time()
        if record and curr_time - last_time > step_time:
            data.append(s)
            last_time = curr_time

        # Here is where the haptic feedback commands go

        uncertainty = sum(np.std(actions, axis=0))
        scaled_uncertainty = scale_uncertainty(uncertainty, args.task)
        pressure_feedback(scaled_uncertainty)

        # end of haptic feedback commands

        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
