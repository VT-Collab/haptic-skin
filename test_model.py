from __future__ import division
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
from positions import HOME
from utils import TrajectoryClient as TR
import argparse
from Tkinter import *
from utils import interface_GUI
from utils import JoystickControl
from ur_msgs.srv import SetIO

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


parser = argparse.ArgumentParser(description='Preparing state-action pair dataset')
parser.add_argument('--feature', help='XY, Z, ROT', type=str)
args = parser.parse_args()


def analog_IO(fun, pin, state):
    rospy.wait_for_service('/ur_hardware_interface/set_io')
    try:
        set_io = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)
        set_io(fun = fun,pin = pin,state = state)
        # print("Sending analog "+ str(pin)+ " pressure...")
    except rospy.ServiceException, e:
        print "Unable to send pressure command: %s"%e



class Model(object):
    def __init__(self, name):
        self.model = BC(32)
        model_dict = torch.load("data/models/" + name, map_location='cpu')
        self.model.load_state_dict(model_dict)
        self.model.eval

    def policy(self, state):#, goal):
        s_tensor = torch.FloatTensor(state)
        # goal_tensor = torch.FloatTensor(goal)
        action = self.model.encoder(s_tensor).detach().numpy()
        # action = self.model.encoder(torch.cat((s_tensor, goal_tensor))).detach().numpy()
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
        # self.robotiq_client.send_goal(goal)

    
   
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
    start_time = time.time()
    last_time = time.time()
    

    data = []
    rospy.init_node("recorder")
    rate = rospy.Rate(100)


    ur10 = Robot()
    recorder = RecordClient()
    joystick = JoystickControl()
    GUI = interface_GUI()
    comm_arduino = serial.Serial('/dev/ttyACM0', baudrate=9600)

    
    model1 = Model(args.feature + "/" + "expert_model_1")
    model2 = Model(args.feature + "/" + "expert_model_2")
    model3 = Model(args.feature + "/" + "expert_model_3")
    model4 = Model(args.feature + "/" + "expert_model_4")
    model5 = Model(args.feature + "/" + "expert_model_5")


    while not recorder.joint_states:
        pass

    rospy.sleep(1)
    recorder.send_cmd('movel(' + str(HOME) + ')')
    rospy.sleep(2)
    # recorder.actuate_gripper(1, 0.1, 1)
    gripper_open = True
    # rospy.sleep(0.5)

    print("[*] Press B to STOP")

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
            print("----pressed B")
            shutdown = True
            time_stop = time.time()
      
        if shutdown and time.time() - time_stop > 2.0:
            # recorder.actuate_gripper(1, 0.1, 1)
            return True

        s = list(recorder.joint_states)
        cur_xyz, _, _ = ur10.forward_kinematics(s)

        actions = []
        a1 = model1.policy(s)
        a2 = model2.policy(s)
        a3 = model3.policy(s)
        a4 = model4.policy(s)
        a5 = model5.policy(s)
        actions = np.array([a1, a2, a3, a4, a5])


  
        # forward kinematics, then do xy, z, roll-pitch-yaw
        s_array = np.array(s)
        xyz1, quat1, _ = ur10.forward_kinematics(s_array + a1)
        xyz2, quat2, _ = ur10.forward_kinematics(s_array + a2)
        xyz3, quat3, _ = ur10.forward_kinematics(s_array + a3)
        xyz4, quat4, _ = ur10.forward_kinematics(s_array + a4)
        xyz5, quat5, _ = ur10.forward_kinematics(s_array + a5)
        actions_xyz = np.array([xyz1, xyz2, xyz3, xyz4, xyz5])
        action_xyz_std = np.std(actions_xyz, axis=0)
        actions_quat = [quat1, quat2, quat3, quat4, quat5]
        action_quat_d = 0.0
        quat_pairs = list(itertools.combinations(actions_quat, 2))
        for item in quat_pairs:
            action_quat_d += Quaternion.absolute_distance(item[0], item[1]) / len(quat_pairs)
        uncertainty1 = (action_xyz_std[0] + action_xyz_std[1]) / 2.0
        uncertainty2 = action_xyz_std[2]
        uncertainty3 = action_quat_d
        

        # hyperparameters
        if not args.feature:
            hyp_xy = 1.0
            hyp_z = 0.5
            hyp_orien = 0.1
        elif args.feature == 'XY':
            hyp_xy = 1.5
            hyp_z = 0.7
            hyp_orien = 0.1
        elif args.feature == 'Z':
            hyp_xy = 1.0
            hyp_z = 1.5
            hyp_orien = 0.2
        elif args.feature == 'ROT':
            hyp_xy = 1.0
            hyp_z = 1.0
            hyp_orien = 1.0

        uncertainty = np.array([uncertainty1 * hyp_xy, uncertainty2 * hyp_z, uncertainty3 * hyp_orien])
        most_uncertain = np.argmax(uncertainty)
        
        if most_uncertain == 0:
            uncertain_name = "XY"
        elif most_uncertain == 1:
            uncertain_name = "Z"
        elif most_uncertain == 2:
            uncertain_name = "ROT"
        else:
            uncertain_name = " "    

    
        # normalize and map uncertainty to 0-3 [psi]
        if cur_xyz[1] < 0.55:
            signal_P = np.array([0.9, 0.9, 0.9])
        else:
            print("--here", cur_xyz[1])
            signal_P = np.round(3*uncertainty/np.linalg.norm(uncertainty), 2)

        if shutdown:
            print("[*] Shutting down...")
            signal_P = 0*signal_P
        # else:
        # #     # print(uncertain_name, uncertainty[0], uncertainty[1], uncertainty[2])
        #     print(uncertain_name, signal_P[0], signal_P[1], signal_P[2])


        # # send signal to UR10
        analog_IO(3, 0, signal_P[0]/30.0)    # XY
        analog_IO(3, 1, signal_P[1]/30.0)    # Z

        # send signal to Arduino       
        comm_arduino.write('<' + str(signal_P[2]) + '>')
        rospy.sleep(0.01)


        rate.sleep()




if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        
        pass
