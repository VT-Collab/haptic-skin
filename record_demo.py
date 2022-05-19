import time
import numpy as np
import pickle
import argparse

from utils import JoystickControl, TrajectoryClient, HOME


parser = argparse.ArgumentParser(description='Collecting offline demonstrations')
parser.add_argument('--who', help='expert vs. user(i)', type=str, default="expert")
parser.add_argument('--feature', help='XY, Z, ROT', type=str, default="XY")
parser.add_argument('--trial', help='demonstration index', type=str, default="0")
args = parser.parse_args()

if args.who == "expert":
    filename = "data/demos/" + args.feature + "/" + args.who + "_" + args.trial + ".pkl"
elif args.who[0:4] == "user":
    filename = "data/demos/" + args.feature + "/" + args.who + ".pkl"

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

data = []
record = False
shutdown = False
segment = 0
step_time = 0.05

while not shutdown:
    # read robot states
    state = Panda.readState(conn)
    joint_pos = state["q"].tolist()

    # joystick commands
    A, B, _, _, START = joystick.getInput()
    if record and B:
        pickle.dump(data, open(filename, "wb"))
        print("[*] Saved Recording")
        print(data)
        shutdown = True

    elif not record and A:
        record = True
        last_time = time.time()
        start_time = time.time()
        time_last_segment = time.time()
        print("[*] Recording segment ", segment)

    curr_time = time.time()
    if START and record and curr_time - time_last_segment > 0.5:
        segment += 1
        print("[*] Next segment: ", segment)
        time_last_segment = time.time()
    if record and curr_time - last_time > step_time:
        data.append(joint_pos + [segment])
        last_time = curr_time
