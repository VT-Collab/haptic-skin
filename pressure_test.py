import sys
import time
import numpy as np
import serial
import copy
import pickle



comm_arduino = serial.Serial('/dev/ttyACM0', baudrate=9600)


def send_arduino(user_input):
	string = '<' + user_input + '>'
	comm_arduino.write(str.encode(string))

while True:
	user_input = input("Pressure: ")
	send_arduino(user_input)
