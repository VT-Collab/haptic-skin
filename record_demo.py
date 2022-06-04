import time
import numpy as np
import pickle
import argparse
from utils import *
from tkinter import *
from pyquaternion import Quaternion
import serial


parser = argparse.ArgumentParser(description='Collecting offline demonstrations')
parser.add_argument('--method', help='GUI, local, global', type=str, default="None")
parser.add_argument('--who', help='expert vs. user(i)', type=str, default="expert")
args = parser.parse_args()

if args.who == "expert":
    filename = "data/demos/" + args.who + ".pkl"
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
p = []

mode = "k"
record = False
shutdown = False
refresh_time = 0.02
step_time = 0.02
last_update = time.time()
signal = np.zeros(3)
qdot = [0]*7

# hyperparameters
alpha = 0.4
beta = 0.8
max_press = 3.5

# margins in robot workspace
x_margin_1 = 0.1
x_margin_2 = 0.4
y_margin = 0.2

while not shutdown:
    # read robot states
    state = Panda.readState(conn)
    joint_pos = state["q"].tolist()
    curr_xyz, curr_quat, rot_mat = Panda.joint2pose(joint_pos)

    # joystick commands
    A, B, _, _, START = joystick.getInput()
    if record and B:
        shutdown = True
        mode = "v"
        send_serial(comm_arduino, "0.0;0.0;0.0")
        data["joint positions"] = q
        data["ee positions"] = ee
        data["rotation matrix"] = R
        data["time"] = t
        data["pressure"] = p
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
        p.append(new_signal.tolist())
        t.append(time.time() - start_time)
        last_time = time.time()

    # send zero joint velocity
    Panda.send2robot(conn, qdot, mode)

    # compute distance from optimal trajectory
    dist = np.array([max_press * abs(-0.45 - curr_xyz[1]) / alpha,
              max_press * abs(0.05 - curr_xyz[2]) / alpha,
              max_press * Quaternion.absolute_distance(Panda.rot2quat(R_desire), curr_quat) / beta])

    if args.method:
        if args.method == "GUI":
            # segment assignment
            if curr_xyz[0] < 0.2:
                signal_xy, signal_z, signal_orien = 0.0, dist[1], 0.0
            elif 0.2 <= curr_xyz[0] <= 0.4:
                signal_xy, signal_z, signal_orien = dist[0], 0.0, 0.0
            elif curr_xyz[0] > 0.4 and curr_xyz[1] > 0.2:
                signal_xy, signal_z, signal_orien = 0.0, 0.0, 0.0
            elif curr_xyz[0] > 0.4:
                signal_xy, signal_z, signal_orien = 0.0, 0.0, dist[2]

        if args.method == "local":
            # segment assignment
            if curr_xyz[0] < x_margin_1:
                signal_xy, signal_z, signal_orien = 0.0, 0.0, dist[2]
            elif x_margin_1 <= curr_xyz[0] <= x_margin_2:
                signal_xy, signal_z, signal_orien = dist[0], 0.0, 0.0
            elif curr_xyz[0] > x_margin_2 and curr_xyz[1] > y_margin:
                signal_xy, signal_z, signal_orien = 0.0, 0.0, 0.0
            elif curr_xyz[0] > x_margin_2:
                signal_xy, signal_z, signal_orien = 0.0, dist[1], 0.0

        elif args.method == "global":
            # segment assignment
            if curr_xyz[0] < 0.2:
                signal_xy, signal_z, signal_orien = 0.0, dist[1], 0.0
            elif 0.2 <= curr_xyz[0] <= 0.4:
                signal_xy, signal_z, signal_orien = 0.0, 0.0, dist[2]
            elif curr_xyz[0] > 0.4 and curr_xyz[1] > 0.2:
                signal_xy, signal_z, signal_orien = 0.0, 0.0, 0.0
            elif curr_xyz[0] > 0.4:
                signal_xy, signal_z, signal_orien = dist[0], 0.0, 0.0

    # compute signal change
    new_signal = np.array([signal_xy, signal_z, signal_orien])
    if curr_xyz[0] > 0.4 and curr_xyz[1] > 0.2:
        diff = 1.0
    else:
        diff = abs(np.linalg.norm(new_signal) - np.linalg.norm(signal))

    # render feedback signals
    if args.method:
        passed_time = time.time() - last_update
        if passed_time > refresh_time and diff > 0.3:
            # update GUI
            if args.method == "GUI":
                GUI.textbox1.delete(0, END)
                GUI.textbox1.insert(0, signal_xy)
                GUI.textbox2.delete(0, END)
                GUI.textbox2.insert(0, signal_z)
                GUI.textbox3.delete(0, END)
                GUI.textbox3.insert(0, signal_orien)
                GUI.root.update()
                last_update = time.time()
            # inflate bags locally or globally
            elif args.method == "local" or "global":
                pressure = np.clip(np.round(new_signal, 1), 0, max_press)
                send_serial(comm_arduino, str(pressure[0]) + ";" + str(pressure[1]) + ";" + str(pressure[2]))
                signal = new_signal
                if shutdown:
                    print("[*] Shutting down...")
                    signal_xy, signal_z, signal_orien = 0.0, 0.0, 0.0
