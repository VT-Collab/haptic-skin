import numpy as np
import time
import pickle
import socket
import sys
# from scipy.interpolate import interp1d
import pygame
# import torch
import copy
# from torch.optim import Adam
# from torch.nn.utils.convert_parameters import parameters_to_vector


########## robot home joint positions ##########
HOME = [-0.03, -0.95, 0.036, -2.23, 0.02, 1.3, -0.92]

########## GUI design ##########
class GUI_Interface(object):
    def __init__(self):
        self.root = Tk()
        self.root.title("Uncertainity Output")
        self.update_time = 0.02
        font = "Palatino Linotype"

        # X_Y Uncertainty
        myLabel1 = Label(self.root, text = "X-Y", font=(font, 40))
        myLabel1.grid(row = 0, column = 0, pady = 50, padx = 50)
        self.textbox1 = Entry(self.root, width = 5, bg = "white", fg = "#676767", borderwidth = 3, font=(font, 40))
        self.textbox1.grid(row = 0, column = 1,  pady = 10, padx = 20)
        self.textbox1.insert(0,0)

        # Z Uncertainty
        myLabel2 = Label(self.root, text = "Z", font=("Palatino Linotype", 40))
        myLabel2.grid(row = 1, column = 0, pady = 50, padx = 50)
        self.textbox2 = Entry(self.root, width = 5, bg = "white", fg = "#676767", borderwidth = 3, font=(font, 40))
        self.textbox2.grid(row = 1, column = 1,  pady = 10, padx = 20)
        self.textbox2.insert(0,0)

        # ROT Uncertainty
        myLabel3 = Label(self.root, text = "ROT", font=("Palatino Linotype", 40))
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

    def send2robot(self, conn, qdot, mode, traj_name=None, limit=0.5):
    	if traj_name is not None:
    		if traj_name[0] == 'q':
    			limit = 1.0
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
    	xyz_lin, R = self.joint2pose(state_vector[0:7])
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
    	return H[:,3][:3], H[:,:3][:3]


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



"""Collect Physical Human Demonstrations"""
def collect_demos(conn, args):

	print("RETURNING HOME")
	interface = Joystick()
	# go2home(conn)
	print("PRESS START WHEN READY")

	state = readState(conn)
	print(state['x'])
	qdot = [0.0]*7
	demonstration = []
	record = False
	steptime = 0.1
	XI = []
	scale = 1.0
	mode = "k"
	while True:

		state = readState(conn)
		# print(state['x'])

		A, B, stop, start = interface.input()

		if A:
			record = False
			print("Are you satisfied with the demonstration?")
			print("Enter [yes] to proceed any ANY KEY to scrap it")
			ans = input()
			if ans == 'yes':
				for idx in range (len(demonstration)):
					demonstration[idx] = demonstration[idx] + demonstration[-1]
				XI.append(demonstration)
				print("[*] Done!")
				print("[*] I recorded this many datapoints: ", len(demonstration))
			demonstration = []
			print("Please release the E-Stop")
			time.sleep(5)
			go2home(conn)
			print("Press START for another demonstration or X to save the dataset")


		if stop:
			pickle.dump(XI, open('../demos/run_' + args.run_name + '/demo_expert.pkl', "wb"))

			demos = pickle.load(open('../demos/run_' + args.run_name + '/demo_expert.pkl', "rb"))
			XI = []
			print(len(demos))
			for idx in range(len(demos)):
				demo = []
				demo1 = np.array(demos[idx])
				for d_idx in range (len(demo1)-1):
					if np.linalg.norm(demo1[d_idx,:6] - demo1[d_idx+1, :6]) > 0.001:
						demo.append(demo1[d_idx,:])

				XI.append(demo)
				print(len(XI))

			pickle.dump(XI, open('../demos/run_' + args.run_name + '/demo_train.pkl', "wb"))
			return True


		if start and not record:
			record = True
			start_time = time.time()
			print('[*] Recording the demonstration...')

		curr_time = time.time()
		if record and curr_time - start_time >= steptime:
			demonstration.append(state["x"].tolist())
			start_time = curr_time

		send2robot(conn, qdot, mode)


# get_target()
