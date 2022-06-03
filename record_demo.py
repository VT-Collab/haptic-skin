import time
import numpy as np
import pickle
import argparse
from utils import GUI_Interface, JoystickControl, TrajectoryClient, HOME, R_desire
from tkinter import *
from pyquaternion import Quaternion
import serial


parser = argparse.ArgumentParser(description='Collecting offline demonstrations')
parser.add_argument('--method', help='GUI, local, global', type=str)
parser.add_argument('--who', help='expert vs. user(i)', type=str, default="expert")
parser.add_argument('--trial', help='demonstration index', type=str, default="0")
args = parser.parse_args()

if args.who == "expert":
    filename = "data/demos/" + args.who + "_" + args.trial + ".pkl"
elif args.who[0:4] == "user":
    filename = "data/demos/" + args.who + "_" + args.method + ".pkl"

# instantiate the robot and joystick
Panda = TrajectoryClient()
joystick = JoystickControl()

# establish socket connection with panda
print('[*] Connecting to Panda...')
PORT_robot = 8080
conn = Panda.connect2robot(PORT_robot)

# send robot to home
print('[*] Sending Panda to home...')
Panda.go2home(conn, HOME)

print("[*] Press A to START Recording")
print("[*] Press B to STOP Recording")

# initilize GUI
GUI = GUI_Interface()
# serial communication
comm_arduino = serial.Serial('/dev/ttyACM0', baudrate=9600)

data = {}
q = []
ee = []
R = []
t = []

mode = "k"
record = False
shutdown = False
refresh_time = 0.5
step_time = 0.05
last_update = time.time()
signal_1, signal_2, signal_3 = 0, 0, 0

while not shutdown:
    # read robot states
    state = Panda.readState(conn)
    joint_pos = state["q"].tolist()
    curr_xyz, curr_quat, rot_mat = Panda.joint2pose(joint_pos)

    # joystick commands
    A, B, _, _, START = joystick.getInput()
    if record and B:
        mode = "v"
        shutdown = True
        data["joint positions"] = q
        data["ee positions"] = ee
        data["rotation matrix"] = R
        data["time"] = t
        pickle.dump(data, open(filename, "wb"))
        print("[*] Saved Demonstration")
    elif not record and A:
        mode = "k"
        record = True
        last_time = time.time()
        start_time = time.time()
        print("[*] Started Recording")
    if record and time.time() - last_time > step_time:
        q.append(joint_pos)
        ee.append(curr_xyz.tolist())
        R.append(rot_mat)
        t.append(time.time() - start_time)
        last_time = time.time()
    qdot = [0]*7
    Panda.send2robot(conn, qdot, mode)


    alpha = 0.5
    beta = 0.8
    signal = [3.0 * abs(-0.4 - curr_xyz[1]) / alpha,
              3.0 * abs(0.0 - curr_xyz[2]) / alpha,
              3.0 * Quaternion.absolute_distance(Panda.rot2quat(R_desire), curr_quat) / beta]


    # methods: GUI, local, global, none
    if args.method:
        if args.method == "GUI":
            # segment assignment
            if curr_xyz[0] < 0.2:
                signal_xy, signal_z, signal_orien = 0, signal[1], 0
            elif 0.2 <= curr_xyz[0] <= 0.4:
                signal_xy, signal_z, signal_orien = signal[0], 0, 0
            elif curr_xyz[0] > 0.4 and curr_xyz[1] > 0.2:
                signal_xy, signal_z, signal_orien = 0, 0, 0
            elif curr_xyz[0] > 0.4:
                signal_xy, signal_z, signal_orien = 0, 0, signal[2]
            # update GUI
            passed_time = time.time() - last_update
            if passed_time > refresh_time:
                GUI.textbox1.delete(0, END)
                GUI.textbox1.insert(0, signal_xy)
                GUI.textbox2.delete(0, END)
                GUI.textbox2.insert(0, signal_z)
                GUI.textbox3.delete(0, END)
                GUI.textbox3.insert(0, signal_orien)
                GUI.root.update()
                last_update = time.time()

        elif args.method == "local":
            # segment assignment
            if curr_xyz[0] < 0.2:
                signal_xy, signal_z, signal_orien = 0, 0, signal[2]
            elif 0.2 <= curr_xyz[0] <= 0.4:
                signal_xy, signal_z, signal_orien = signal[0], 0, 0
            elif curr_xyz[0] > 0.4 and curr_xyz[1] > 0.2:
                signal_xy, signal_z, signal_orien = 0, 0, 0
            elif curr_xyz[0] > 0.4:
                signal_xy, signal_z, signal_orien = 0, signal[1], 0

            # send pressure signal to Arduino
            send_arduino(comm_arduino, str(signal_xy) + str(signal_z) + str(signal_orien))

        elif args.method == "global":
            # segment assignment
            if curr_xyz[0] < 0.2:
                signal_xy, signal_z, signal_orien = 0, signal[1], 0
            elif 0.2 <= curr_xyz[0] <= 0.4:
                signal_xy, signal_z, signal_orien = 0, 0, signal[2]
            elif curr_xyz[0] > 0.4 and curr_xyz[1] > 0.2:
                signal_xy, signal_z, signal_orien = 0, 0, 0
            elif curr_xyz[0] > 0.4:
                signal_xy, signal_z, signal_orien = signal[0], 0, 0
