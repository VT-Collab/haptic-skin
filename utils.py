import numpy as np
import time
import pickle
import socket
import sys
import pygame
import copy
from pyquaternion import Quaternion
from tkinter import *
import serial


########## User number list ##########
USERS = [1,2,3,4,5,7,8,9,10]

########## robot home joint positions ##########
HOME = [-1.45, -0.42, -0.88, -2.46, -0.8, 1.72, 0.8]

########## ee desired orientation ##########
R_desire = np.array([[ 7.44356863e-01,  6.66865793e-01,  3.49696414e-02],
                    [ 6.66166262e-01, -7.45177629e-01,  3.05419708e-02],
                    [ 4.64259900e-02,  5.61469737e-04, -9.98921575e-01]])

########## margins in workspace ##########
x_margin_1 = 0.15
x_margin_2 = 0.45
x_margin_3 = 0.75
y_margin = 0.15

########## Serial Comm. with Arduino ##########
def send_serial(comm, output):
    string = '<' + output + '>'
    comm.write(str.encode(string))

########## GUI design ##########
class GUI_Interface(object):
    def __init__(self):
        self.root = Tk()
        self.root.geometry("+1000+100")
        self.root.title("Uncertainity Output")
        self.update_time = 0.02
        font = "Palatino Linotype"

        # X_Y Uncertainty
        myLabel1 = Label(self.root, text = "Distance From Edge", font=(font, 40))
        myLabel1.grid(row = 0, column = 0, pady = 50, padx = 50)
        self.textbox1 = Entry(self.root, width = 5, bg = "white", fg = "#676767", borderwidth = 3, font=(font, 40))
        self.textbox1.grid(row = 0, column = 1,  pady = 10, padx = 20)
        self.textbox1.insert(0,0)

        # Z Uncertainty
        myLabel2 = Label(self.root, text = "Height from Table", font=("Palatino Linotype", 40))
        myLabel2.grid(row = 1, column = 0, pady = 50, padx = 50)
        self.textbox2 = Entry(self.root, width = 5, bg = "white", fg = "#676767", borderwidth = 3, font=(font, 40))
        self.textbox2.grid(row = 1, column = 1,  pady = 10, padx = 20)
        self.textbox2.insert(0,0)

        # ROT Uncertainty
        myLabel3 = Label(self.root, text = "Orientation", font=("Palatino Linotype", 40))
        myLabel3.grid(row = 2, column = 0, pady = 50, padx = 50)
        self.textbox3 = Entry(self.root, width = 5, bg = "white", fg = "#676767", borderwidth = 3, font=(font, 40))
        self.textbox3.grid(row = 2, column = 1,  pady = 10, padx = 20)
        self.textbox3.insert(0,0)


########## Logitech joystick ##########
class JoystickControl(object):

	def __init__(self):
		pygame.init()
		self.gamepad = pygame.joystick.Joystick(0)
		self.gamepad.init()
		self.deadband = 0.1
		self.timeband = 0.5
		self.lastpress = time.time()

	def getInput(self):
		pygame.event.get()
		curr_time = time.time()
		A_pressed = self.gamepad.get_button(0) and (curr_time - self.lastpress > self.timeband)
		B_pressed = self.gamepad.get_button(1) and (curr_time - self.lastpress > self.timeband)
		X_pressed = self.gamepad.get_button(2) and (curr_time - self.lastpress > self.timeband)
		Y_pressed = self.gamepad.get_button(3) and (curr_time - self.lastpress > self.timeband)
		START_pressed = self.gamepad.get_button(7) and (curr_time - self.lastpress > self.timeband)
		if A_pressed or START_pressed or B_pressed:
			self.lastpress = curr_time
		return A_pressed, B_pressed, X_pressed, Y_pressed, START_pressed


########## Panda ##########
class TrajectoryClient(object):

    def __init__(self):
    	pass

    def connect2robot(self, PORT):
    	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    	s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    	s.bind(('172.16.0.3', PORT))
    	s.listen()
    	conn, addr = s.accept()
    	return conn

    def connect2gripper(PORT):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(('172.16.0.3', PORT))
        s.listen(10)
        conn, addr = s.accept()
        return conn

    def send2gripper(self, conn):
        send_msg = "o"
        conn.send(send_msg.encode())

    def send2robot(self, conn, qdot, mode, limit=1.0):
    	qdot = np.asarray(qdot)
    	scale = np.linalg.norm(qdot)
    	if scale > limit:
    		qdot *= limit/scale
    	send_msg = np.array2string(qdot, precision=5, separator=',',suppress_small=True)[1:-1]
    	send_msg = "s," + send_msg + "," + mode + ","
    	conn.send(send_msg.encode())

    def listen2robot(self, conn):
    	state_length = 7 + 7 + 7 + 42
    	message = str(conn.recv(2048))[2:-2]
    	state_str = list(message.split(","))

    	for idx in range(len(state_str)):
    		if state_str[idx] == "s":
    			state_str = state_str[idx+1:idx+1+state_length]
    			break
    	try:
    		state_vector = [float(item) for item in state_str]
    	except ValueError:
    		return None

    	if len(state_vector) is not state_length:
    		return None

    	state_vector = np.asarray(state_vector)
    	state = {}
    	state["q"] = state_vector[0:7]
    	state["dq"] = state_vector[7:14]
    	state["tau"] = state_vector[14:21]
    	state["J"] = state_vector[21:].reshape((7,6)).T

    	# get cartesian pose
    	xyz_lin, _, R = self.joint2pose(state_vector[0:7])
    	beta = -np.arcsin(R[2,0])
    	alpha = np.arctan2(R[2,1]/np.cos(beta),R[2,2]/np.cos(beta))
    	gamma = np.arctan2(R[1,0]/np.cos(beta),R[0,0]/np.cos(beta))
    	xyz_ang = [alpha, beta, gamma]
    	xyz = np.asarray(xyz_lin).tolist() + np.asarray(xyz_ang).tolist()
    	state["x"] = np.array(xyz)
    	return state

    def readState(self, conn):
    	while True:
    		state = self.listen2robot(conn)
    		if state is not None:
    			break
    	return state

    def xdot2qdot(self, xdot, state):
    	J_pinv = np.linalg.pinv(state["J"])
    	return J_pinv @ np.asarray(xdot)

    def joint2pose(self, q):
        def RotX(q):
            return np.array([[1, 0, 0, 0], [0, np.cos(q), -np.sin(q), 0], [0, np.sin(q), np.cos(q), 0], [0, 0, 0, 1]])
        def RotZ(q):
            return np.array([[np.cos(q), -np.sin(q), 0, 0], [np.sin(q), np.cos(q), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        def TransX(q, x, y, z):
            return np.array([[1, 0, 0, x], [0, np.cos(q), -np.sin(q), y], [0, np.sin(q), np.cos(q), z], [0, 0, 0, 1]])
        def TransZ(q, x, y, z):
            return np.array([[np.cos(q), -np.sin(q), 0, x], [np.sin(q), np.cos(q), 0, y], [0, 0, 1, z], [0, 0, 0, 1]])
        H1 = TransZ(q[0], 0, 0, 0.333)
        H2 = np.dot(RotX(-np.pi/2), RotZ(q[1]))
        H3 = np.dot(TransX(np.pi/2, 0, -0.316, 0), RotZ(q[2]))
        H4 = np.dot(TransX(np.pi/2, 0.0825, 0, 0), RotZ(q[3]))
        H5 = np.dot(TransX(-np.pi/2, -0.0825, 0.384, 0), RotZ(q[4]))
        H6 = np.dot(RotX(np.pi/2), RotZ(q[5]))
        H7 = np.dot(TransX(np.pi/2, 0.088, 0, 0), RotZ(q[6]))
        H_panda_hand = TransZ(-np.pi/4, 0, 0, 0.2105)
        H = np.linalg.multi_dot([H1, H2, H3, H4, H5, H6, H7, H_panda_hand])
        xyz = H[:,3][:3]
        rot_matrix = H[:,:3][:3]
        quaternion = self.rot2quat(rot_matrix)
        return xyz, quaternion, rot_matrix

    def rot2quat(self, R):
        return Quaternion(matrix=R)

    def go2home(self, conn, HOME):
        total_time = 35.0
        # distance to home
        start_time = time.time()
        state = self.readState(conn)
        joint_pos = np.asarray(state["q"].tolist())
        dist = np.linalg.norm(joint_pos - HOME)
        curr_time = time.time()
        action_time = time.time()
        elapsed_time = curr_time - start_time

        while dist > 0.02 and elapsed_time < total_time:
            joint_pos = np.asarray(state["q"].tolist())
            action_interval = curr_time - action_time

            if action_interval > 0.005:
                qdot = HOME - joint_pos
                self.send2robot(conn, qdot, "v")
                action_time = time.time()

            state = self.readState(conn)
            dist = np.linalg.norm(joint_pos - HOME)
            curr_time = time.time()
            elapsed_time = curr_time - start_time

        # Send completion status
        if dist <= 0.02:
            return True
        elif elapsed_time >= total_time:
            return False

        # def wrap_angles(self, theta):
        # if theta < -np.pi:
        # 	theta += 2*np.pi
        # elif theta > np.pi:
        # 	theta -= 2*np.pi
        # else:
        # 	theta = theta
        # return theta
