import sys
import time
import numpy as np
import pygame
import copy
import pickle
import torch
from collections import deque
import argparse
from train_model import BC
import argparse
from utils import JoystickControl, TrajectoryClient, HOME



parser = argparse.ArgumentParser(description='Preparing state-action pair dataset')
parser.add_argument('--who', help='expert vs. user(i)', type=str, default="expert")
args = parser.parse_args()


if args.who == "expert":
    model_name = "expert_model_1"
elif args.who[0:4] == "user":
    model_name = args.who + "_model_1"

ACTION_SCALE = 0.15


class Model(object):
    def __init__(self, model_name):
        self.model = BC(32)
        model_dict = torch.load("data/models/" + model_name, map_location='cpu')
        self.model.load_state_dict(model_dict)
        self.model.eval

    def policy(self, state):
        s_tensor = torch.FloatTensor(state)
        action = self.model.encoder(s_tensor).detach().numpy()
        return action


# instantiate the robot, joystick, and model
Panda = TrajectoryClient()
joystick = JoystickControl()
model = Model(model_name)

# establish socket connection with panda
print('[*] Connecting to Panda...')
PORT_robot = 8080
conn = Panda.connect2robot(PORT_robot)

# send robot to home
print('[*] Sending Panda to home...')
Panda.go2home(conn, HOME)

print("[*] Press A to start robot")

run = False
shutdown = False
n_samples = 100

while not shutdown:
    state = Panda.readState(conn)
    joint_pos = state["q"].tolist()

    action = model.policy(joint_pos) * 50.0
    qdot = action - joint_pos

    if np.linalg.norm(qdot) > ACTION_SCALE:
        qdot = qdot / np.linalg.norm(qdot) * ACTION_SCALE

    A, B, _, _, _ = joystick.getInput()
    if A:
        run = True
        print("[*] Robot is moving")
    if B:
        print('[*] Robot stopped!')
        run = False
        shutdown = True
    if not run:
        qdot = np.asarray([0.0] * 7)
    # print("qdot: ", qdot)
    Panda.send2robot(conn, qdot, "v")
