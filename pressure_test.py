import sys
import time
import numpy as np
import serial
import copy
import pickle
import time



comm_arduino = serial.Serial('/dev/ttyACM0', baudrate=9600)


def send_arduino(user_input):
	string = '<' + user_input + '>'
	comm_arduino.write(str.encode(string))
last_update = time.time()
while True:
	# user_input = input("Pressure: ")
	# send_arduino(user_input)

	# send_arduino("0.00.01.0")
	passed_time = time.time() - last_update
	if passed_time > 0.1:
		send_arduino("0.0;0.0;0.0")
