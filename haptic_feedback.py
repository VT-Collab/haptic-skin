
import rospy
import sys
import time
import numpy as np
import pygame
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
import copy
import pickle
import torch
from train_model import MLP

from std_msgs.msg import Float64MultiArray, String

from sensor_msgs.msg import (
    JointState
)
from geometry_msgs.msg import(
    TwistStamped,
    Twist
)

# from ur_dashboard_msgs.srv import(
#     IsProgramRunning,
#     Load,
#     GetProgramState
# )


HOME = [-1.45, -1.88, -1.80,-0.97, 1.54, -0.02]


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
        return A, B, START


class Model(object):
    def __init__(self):
        self.model = MLP()
        model_dict = torch.load("models/MLP_model", map_location='cpu')
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

    def joint_states_cb(self, msg):
        states = list(msg.position)
        states[2], states[0] = states[0], states[2]
        self.joint_states = tuple(states)

    def send_cmd(self, cmd):
        self.script_pub.publish(cmd)


def main():
    filename = "demos/" + sys.argv[1] + ".pkl"
    data = []
    rospy.init_node("recorder")
    rate = rospy.Rate(100)
    recorder = RecordClient()
    joystick = JoystickControl()
    model = Model()

    while not recorder.joint_states:
        pass

    rospy.sleep(1)
    recorder.send_cmd('movel(' + str(HOME) + ')')
    rospy.sleep(2)
    print("[*] Press A to START Recording")
    print("[*] Press B to STOP Recording")

    record = False
    step_time = 0.1
    n_samples = 100

    while not rospy.is_shutdown():

        A, B, start = joystick.getInput()
        if record and B:
            pickle.dump(data, open(filename, "wb"))
            print("I recorded this many data points:", len(data))
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
        print(uncertainty)

        # end of haptic feedback commands

        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
