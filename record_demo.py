import time
import numpy as np
import pickle
import argparse
from utils import GUI_Interface, JoystickControl, TrajectoryClient, HOME
from tkinter import *

parser = argparse.ArgumentParser(description='Collecting offline demonstrations')
parser.add_argument('--who', help='expert vs. user(i)', type=str, default="expert")
parser.add_argument('--trial', help='demonstration index', type=str, default="0")
args = parser.parse_args()

if args.who == "expert":
    filename = "data/demos/" + args.who + "_" + args.trial + ".pkl"
elif args.who[0:4] == "user":
    filename = "data/demos/" + args.who + ".pkl"

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
# time.sleep(3)

print("[*] Press A to START Recording")
print("[*] Press B to STOP Recording")


# initilize GUI and serial communication
GUI = GUI_Interface()
# comm_arduino = serial.Serial('/dev/ttyACM0', baudrate=9600)

data = []
record = False
shutdown = False

last_time = time.time()
refresh_time = 0.5
step_time = 0.05

mode = "k"

while not shutdown:
    # read robot states
    state = Panda.readState(conn)
    joint_pos = state["q"].tolist()
    cur_xyz,_,_ = Panda.joint2pose(joint_pos)

    # joystick commands
    A, B, _, _, START = joystick.getInput()
    if record and B:
        mode = "v"
        pickle.dump(data, open(filename, "wb"))
        print("[*] Saved Recording")
        # print(data)
        shutdown = True

    elif not record and A:
        mode = "k"
        record = True
        last_time = time.time()
        start_time = time.time()
        time_last_segment = time.time()
        print("[*] Started Recording")

    if record and cur_time - last_time > step_time:
        data.append(joint_pos)
        last_time = curr_time

    qdot = [0]*7
    Panda.send2robot(conn, qdot, mode)



    # segment assignment
    if cur_xyz[0] < 0.2:
        print("[*] Segment 1: ", cur_xyz)
        signal_xy = 0.0
        signal_z = 3.0
        signal_orien = 0.0
    elif 0.2 <= cur_xyz[0] <= 0.4:
        print("[*] Segment 2: ", cur_xyz)
        signal_xy = 3.0
        signal_z = 0.0
        signal_orien = 0.0
    elif cur_xyz[0] > 0.4 and cur_xyz[1] > 0.2:
        print("[*] Segment 4: ", cur_xyz)
        signal_xy = 0.0
        signal_z = 0.0
        signal_orien = 0.0
    elif cur_xyz[0] > 0.4:
        print("[*] Segment 3: ", cur_xyz)
        signal_xy = .0
        signal_z = 0.0
        signal_orien = 3.0

    # update GUI
    passed_time = time.time() - last_time
    if passed_time > refresh_time:
        GUI.textbox1.delete(0, END)
        GUI.textbox1.insert(0, signal_xy)
        GUI.textbox2.delete(0, END)
        GUI.textbox2.insert(0, signal_z)
        GUI.textbox3.delete(0, END)
        GUI.textbox3.insert(0, signal_orien)
        GUI.root.update()
        last_time = time.time()
