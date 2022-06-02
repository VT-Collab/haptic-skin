from __future__ import division
import sys
import time
import numpy as np
import pygame
import copy
import pickle
import torch
from train_model import BC
import argparse
import serial
from pyquaternion import Quaternion
import itertools
from utils import GUI_Interface, JoystickControl, TrajectoryClient, HOME
from Tkinter import *


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

    def policy(self, state):
        s_tensor = torch.FloatTensor(state)
        action = self.model.encoder(s_tensor).detach().numpy()
        return action


start_time = time.time()
last_time = time.time()



# instantiate the robot, joystick, and model
Panda = TrajectoryClient()
joystick = JoystickControl()
GUI = GUI_Interface()
model = Model(model_name)
comm_arduino = serial.Serial('/dev/ttyACM0', baudrate=9600)

ur10 = Robot()
recorder = RecordClient()


# establish socket connection with panda
print('[*] Connecting to Panda...')
PORT_robot = 8080
conn = Panda.connect2robot(PORT_robot)

# send robot to home
print('[*] Sending Panda to home...')
Panda.go2home(conn, HOME)

print("[*] Press B to STOP")

model1 = Model(args.feature + "/" + "expert_model_1")
model2 = Model(args.feature + "/" + "expert_model_2")
model3 = Model(args.feature + "/" + "expert_model_3")
model4 = Model(args.feature + "/" + "expert_model_4")
model5 = Model(args.feature + "/" + "expert_model_5")


run = False
shutdown = False

while not shutdown:


    A, B, _, _, _ = joystick.getInput()
    if A:
        run = True
        print("[*] Robot is moving")
    if B:
        print('[*] Robot stopped!')
        run = False
        shutdown = True

    state = Panda.readState(conn)
    joint_pos = state["q"].tolist()

    cur_xyz = Panda.joint2pose(joint_pos)


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
