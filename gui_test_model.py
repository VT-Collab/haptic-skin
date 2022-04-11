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
from train_model import BC
import argparse
import serial
from pyquaternion import Quaternion
import itertools
from positions import HOME, Goals
import argparse
from Tkinter import *

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


class Model(object):
    def __init__(self, name):
        self.model = BC(32)
        model_dict = torch.load("models/" + name, map_location='cpu')
        self.model.load_state_dict(model_dict)
        self.model.eval

    def policy(self, state, goal):
        s_tensor = torch.FloatTensor(state)
        goal_tensor = torch.FloatTensor(goal)
        action = self.model.encoder(torch.cat((s_tensor, goal_tensor))).detach().numpy()
        return action


class Robot(object):

    def __init__(self):
        self.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",\
                            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        self.base_link = "base_link"
        self.end_link = "wrist_3_link"
        self.robot_urdf = URDF.from_parameter_server()
        self.kdl_kin = KDLKinematics(self.robot_urdf, self.base_link, self.end_link)


    def forward_kinematics(self, s, end_link= "wrist_3_link", base_link= "base_link"):
        ee = self.kdl_kin.forward(s)
        xyz = np.array([ee[0, 3], ee[1, 3], ee[2, 3]])
        quat = self.rot2quat(ee[:3, :3])
        return xyz, quat, ee

    def rot2quat(self, R):
        return Quaternion(matrix=R)

    def rot2eul(self, R):
        beta = -np.arcsin(R[2,0])
        alpha = np.arctan2(R[2,1]/np.cos(beta),R[2,2]/np.cos(beta))
        gamma = np.arctan2(R[1,0]/np.cos(beta),R[0,0]/np.cos(beta))
        return np.array([alpha, beta, gamma])


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


def joint2pose(joint_states):
        state = kdl_kin.forward(joint_states)
        xyz_lin = np.array(state[:,3][:3]).T
        xyz_lin = xyz_lin.tolist()
        R = state[:,:3][:3]
        beta = -np.arcsin(R[2,0])
        alpha = np.arctan2(R[2,1]/np.cos(beta),R[2,2]/np.cos(beta))
        gamma = np.arctan2(R[1,0]/np.cos(beta),R[0,0]/np.cos(beta))
        xyz_ang = [alpha, beta, gamma]
        xyz = np.asarray(xyz_lin[-1]).tolist() + np.asarray(xyz_ang).tolist()
        return xyz



def main():
    close_z_pos = 0.3266
    parser = argparse.ArgumentParser(description='playing rolled-out policy')
    parser.add_argument('--goal', type=str, default=0)
    parser.add_argument('--set', type=int, default=0)
    args = parser.parse_args()
 
    data = []
    rospy.init_node("recorder")
    rate = rospy.Rate(100)
    recorder = RecordClient()
    joystick = JoystickControl()

    model1 = Model("MLP_model_1")
    model2 = Model("MLP_model_2")
    model3 = Model("MLP_model_3")
    model4 = Model("MLP_model_4")
    model5 = Model("MLP_model_5")
    
    ur10 = Robot()

    goal = Goals['Goal' + args.goal]
    while not recorder.joint_states:
        pass


    root = Tk()
    root.title("Uncertainity Output")
    myLabel1 = Label(root, text = "X-Y Uncertainty", font=("Arial", 40))
    myLabel1.grid(row = 0, column = 0, pady = 100, padx = 100)
    textbox1 = Entry(root, width = 10, bg = "white", fg = "#676767", borderwidth = 3, font=("Arial", 40))
    textbox1.grid(row = 0, column = 1,  pady = 10, padx = 20)
    textbox1.insert(0,0)
    

    myLabel2 = Label(root, text = "Z Uncertainty", font=("Arial", 40))
    myLabel2.grid(row = 1, column = 0, pady = 100, padx = 100)
    textbox2 = Entry(root, width = 10, bg = "white", fg = "#676767", borderwidth = 3, font=("Arial", 40))
    textbox2.grid(row = 1, column = 1,  pady = 10, padx = 20)
    textbox2.insert(0,0)
    

    myLabel3 = Label(root, text = "ROT Uncertainty", font=("Arial", 40))
    myLabel3.grid(row = 2, column = 0, pady = 100, padx = 100)
    textbox3 = Entry(root, width = 10, bg = "white", fg = "#676767", borderwidth = 3, font=("Arial", 40))
    textbox3.grid(row = 2, column = 1,  pady = 10, padx = 20)
    textbox3.insert(0,0)
    update_time = 0.5

    start_time = time.time()
    last_update = time.time()

    rospy.sleep(1)
    recorder.send_cmd('movel(' + str(HOME) + ')')
    rospy.sleep(2)
    recorder.actuate_gripper(1, 0.1, 1)
    gripper_open = True
    rospy.sleep(0.5)
    print("[*] Press A to START Recording")
    print("[*] Press B to STOP Recording")
    print("[*] Press BACK to STOP Program")

    shutdown = False
    while not rospy.is_shutdown():

        A, B, X, Y, start = joystick.getInput()
        if X and gripper_open:
            recorder.actuate_gripper(0.05, 0.1, 1)
            gripper_open = False
        if Y and not gripper_open:
            recorder.actuate_gripper(1, 0.1, 1)
            gripper_open = True
        if B:
            shutdown = True
            time_stop = time.time()
        if shutdown and time.time() - time_stop > 2.0:
            recorder.actuate_gripper(1, 0.1, 1)
            return True

        s = list(recorder.joint_states)

        actions = []
        a1 = model1.policy(s, goal)
        a2 = model2.policy(s, goal)
        a3 = model3.policy(s, goal)
        a4 = model4.policy(s, goal)
        a5 = model5.policy(s, goal)
        actions = np.array([a1, a2, a3, a4, a5])


        # option 2: get forward kinematics, then do xy, z, roll-pitch-yaw
        s_array = np.array(s)
        xyz1, quat1, _ = ur10.forward_kinematics(s_array + a1)
        xyz2, quat2, _ = ur10.forward_kinematics(s_array + a2)
        xyz3, quat3, _ = ur10.forward_kinematics(s_array + a3)
        xyz4, quat4, _ = ur10.forward_kinematics(s_array + a4)
        xyz5, quat5, _ = ur10.forward_kinematics(s_array + a5)
        actions_xyz = np.array([xyz1, xyz2, xyz3, xyz4, xyz5])
        # print('actions_xyz = ', actions_xyz)

        action_xyz_std = np.std(actions_xyz, axis=0)
        actions_quat = [quat1, quat2, quat3, quat4, quat5]

        action_quat_d = 0.0
        quat_pairs = list(itertools.combinations(actions_quat, 2))
        for item in quat_pairs:
            action_quat_d += Quaternion.absolute_distance(item[0], item[1]) / len(quat_pairs)

        
        # reducing_multiplier
        alpha = 0.3

        if args.set == 1:
            uncertainty1 = (action_xyz_std[0] + action_xyz_std[1]) / 2.0 * 7
            uncertainty2 = action_xyz_std[2] * 4 * alpha
            uncertainty3 = action_quat_d * 0.7 * alpha
        elif args.set == 2:
            uncertainty1 = (action_xyz_std[0] + action_xyz_std[1]) / 2.0 * 7* alpha
            uncertainty2 = action_xyz_std[2] * 4 
            uncertainty3 = action_quat_d * 0.7 * alpha
        elif args.set == 3:
            uncertainty1 = (action_xyz_std[0] + action_xyz_std[1]) / 2.0 * 7 * alpha
            uncertainty2 = action_xyz_std[2] * 4
            uncertainty3 = action_quat_d * 0.7* alpha
        else:
            uncertainty1 = (action_xyz_std[0] + action_xyz_std[1]) / 2.0 * 7
            uncertainty2 = action_xyz_std[2] * 4
            uncertainty3 = action_quat_d * 0.7
       

        uncertainty1 = round(uncertainty1 * 100, 2)
        uncertainty2 = round(uncertainty2 * 100, 2)
        uncertainty3 = round(uncertainty3 * 100, 2)
        uncertainty = np.array([uncertainty1, uncertainty2, uncertainty3])
        most_uncertain = np.argmax(uncertainty)
   

        if most_uncertain == 0:# and uncertainty1 >= 0.5:
            uncertain_name = "X-Y"
        elif most_uncertain == 1:# and uncertainty2 >= 0.5:
            uncertain_name = "-Z-"
        elif most_uncertain == 2:# and uncertainty3 >= 0.5:
            uncertain_name = "ROT"
        else:
            uncertain_name = " "    

        print(uncertain_name, uncertainty1, uncertainty2, uncertainty3)

        uncertainty1 = np.clip(uncertainty1, 0.0, 3.0)
        uncertainty2 = np.clip(uncertainty2, 0.0, 3.0)
        uncertainty3 = np.clip(uncertainty3, 0.0, 3.0)

        gui_number1 = uncertainty1 * 33.33333
        gui_number2 = uncertainty2 * 33.33333
        gui_number3 = uncertainty3 * 33.33333


        curr_time = time.time()

        if curr_time - last_update > update_time:
            textbox1.delete(0, END)
            textbox1.insert(0, round(gui_number1,1))
            textbox2.delete(0, END)
            textbox2.insert(0, round(gui_number2,1))
            textbox3.delete(0, END)
            textbox3.insert(0, round(gui_number3,1))
            root.update()
            last_update = time.time()


        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        
        pass
