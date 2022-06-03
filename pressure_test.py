import sys
import time
import numpy as np
import serial
import copy
import pickle
import argparse


comm_arduino = serial.Serial('/dev/ttyACM0', baudrate=9600)


def send_arduino(user_input):
	string = '<' + user_input + '>'
	comm_arduino.write(str.encode(string))



while True:
	user_input = input("Pressure: ")
	if len(user_input) == 3:
		send_arduino(user_input)
	else:
		pass
