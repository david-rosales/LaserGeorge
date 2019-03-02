#!/usr/bin/env python

"""
Servo Control class for Arduino interface
"""

import serial # Serial communication with Arduino 

class LaserControl:

	def __init__(self, offset_theta=0, offset_phi=0, address='/dev/serial/'):
		# Define serial 
		self.ser = serial.Serial('/dev/ttyACM0', 19200)
		self.offset_theta = offset_theta
		self.offset_phi = offset_phi

	def move_theta(self, theta):
		theta += self.offset_theta
		theta = min(max(0, theta), 180)
		self.ser.write(str(theta) + 't')
		return True

	def move_phi(self, phi):
		phi += self.offset_phi
		phi = min(max(0, phi), 180)
		self.ser.write(str(phi) + 'p')
		return True

	def turn_on(self):
		self.ser.write('y')
		return True 

	def turn_off(self):
		self.ser.write('n')
		return True

	def calibrate(self):
		ok = False
		theta = 90
		phi = 90
		while not ok:
			string = raw_input('angles as "theta-phi", input "on", "off", or enter "x" if done: ')
			if string == "x":
				ok = True
				self.offset_theta = theta - 90
				self.offset_phi = phi - 90
				print("offset_theta: ", theta - 90)
				print("offset_phi: ", phi - 90)

			elif string == "on":
				self.turn_on()

			elif string == "off":
				self.turn_off()

			else:
				values = string.split("-")
				theta = int(values[0])
				phi = int(values[1])
				self.move_theta(theta)
				self.move_phi(phi)

		return True

if __name__ == "__main__":
	laser_control = LaserControl(offset_theta=6, offset_phi=5)
	laser_control.calibrate()
