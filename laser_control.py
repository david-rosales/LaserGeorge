#!/usr/bin/env python

"""
Servo Control class for Arduino interface
"""

import serial # Serial communication with Arduino
import numpy as np 

class LaserControl:

	def __init__(self, theta_min=60, theta_max=120, 
					   phi_min=60, phi_max=120,
					   offset_delta=-3.5, offset_eps=3.7,
					   screen_distance=100,
					   address='/dev/ttyACM0'):

		# all calib parameter in cm or degrees
		# Define serial 
		self.ser = serial.Serial(address, 19200)

		### values based on physical system measurements 
		self.offset_delta = offset_delta
		self.offset_eps = offset_eps
		
		### calibration values 
		self.screen_distance = screen_distance
		self.min_phi_raw = phi_min
		self.max_phi_raw = phi_max
		self.min_theta = theta_min # this is after accountinig for offset delta
		self.max_theta = theta_max 

	def move_theta(self, theta):
		# direct movement to laser system 
		# theta is the raw theta (after conversion)
		theta = min(max(0, theta), 180)
		self.ser.write(str(theta) + 't')
		return True

	def move_phi(self, phi):
		# direct movement to laser system 
		# phi is the raw phi (after conversion)
		# bound as to not break system
		phi = min(max(60, phi), 140)
		self.ser.write(str(phi) + 'p')
		return True

	def turn_on(self):
		self.ser.write('y')
		return True 

	def turn_off(self):
		self.ser.write('n')
		return True

	# def screen2angles(self, x, y):
	# 	# get angle from screen coordinates (range 0~1 in x y)
	# 	x_ = x - 0.5
	# 	y_ = y - 0.5
	# 	thet = 


	def three_point_calibrate(self):

		ok = False

		theta_tl_raw = 90
		phi_tl_raw = 90

		while not ok:
			string = raw_input('Move pointer to top left corner of frame: "wasd", press "x" when done: ')
			if string == "x":
				ok = True

			elif string == "on":
				self.turn_on()

			elif string == "off":
				self.turn_off()

			else:

				if string == "w":
					phi_tl_raw += 1

				elif string == "a":
					theta_tl_raw += 1

				elif string == "s":
					phi_tl_raw -= 1

				elif string == "d":
					theta_tl_raw -= 1
					
				self.move_theta(theta_tl_raw)
				self.move_phi(phi_tl_raw)

		theta_br_raw = 90
		phi_br_raw = 90
		ok = False

		while not ok:
			string = raw_input('Move pointer to bottom right corner of frame: "wasd", press "x" when done: ')
			if string == "x":
				ok = True

			elif string == "on":
				self.turn_on()

			elif string == "off":
				self.turn_off()

			else:

				if string == "w":
					phi_br_raw += 1

				elif string == "a":
					theta_br_raw += 1

				elif string == "s":
					phi_br_raw -= 1

				elif string == "d":
					theta_br_raw -= 1
					
				self.move_theta(theta_br_raw)
				self.move_phi(phi_br_raw)

		theta_bl_raw = 90
		phi_bl_raw = 90
		ok = False
		
		while not ok:
			string = raw_input('Move pointer to bottom left corner of frame: "wasd", press "x" when done: ')
			if string == "x":
				ok = True

			elif string == "on":
				self.turn_on()

			elif string == "off":
				self.turn_off()

			else:

				if string == "w":
					phi_bl_raw += 1

				elif string == "a":
					theta_bl_raw += 1

				elif string == "s":
					phi_bl_raw -= 1

				elif string == "d":
					theta_bl_raw -= 1
					
				self.move_theta(theta_bl_raw)
				self.move_phi(phi_bl_raw)

		### fill in calibration value 
		self.min_phi_raw = phi_br_raw
		self.max_phi_raw = phi_tl_raw

		# assume pointer is centered to screen 
		d_theta_max = (theta_bl_raw + theta_br_raw)/2.0 - 90
		self.max_theta_raw = (theta_bl_raw - theta_br_raw)/2.0
		self.min_theta_raw = -self.max_theta_raw

		# calculate screen distance 
		d_theta_max = np.deg2rad(d_theta_max)
		max_theta_raw = np.deg2rad(self.max_theta_raw)
		d = np.sqrt(self.offset_delta**2/(np.tan(d_theta_max)**2) - np.tan(max_theta_raw)**2)

		# project 
		self.screen_distance = d * np.sin(np.deg2rad(phi_br_raw))

		print("phi_min: ", self.min_phi_raw, " phi_max: ", self.max_phi_raw)
		print("theta_min: ", self.min_theta_raw, " theta_max: ", self.max_theta_raw)
		print("screen_distance: ", self.screen_distance)


if __name__ == "__main__":
	laser_control = LaserControl()
	laser_control.three_point_calibrate()



""" Unused stuff (In case need it later) 

	
	def body2raw(self, theta_b, phi_b):
		# convert theta and phi in body frame 
		# to raw input angle 

		# convert to radians 
		theta_b = np.deg2rad(theta_b)
		phi_b = np.deg2rad(phi_b)

		# calculate new theta based on offset 
		d_theta = np.arctan2(-self.offset_delta, np.sqrt(np.tan(theta_b)**2 + self.screen_distance**2))
		theta = theta_b + d_theta
		phi = np.arctan2(np.tan(phi_b), (1 - self.offset_eps*np.tan(phi_b)))

		print(theta)
		# back to degrees and add offset 
		theta = np.rad2deg(theta) + self.offset_theta
		phi = np.rad2deg(phi) + self.offset_phi


		print(int(theta), int(phi))

		return int(theta), int(phi)

	def calibrate_theta_phi_offset(self):
		# find self.offset_theta and self.offset_phi
		# set screen distance to huge for calibration 
		# should be able to point laser delta from center of screen
		self.screen_distance = 999999
		ok = False

		while not ok:
			string = raw_input('Calibrate phi, theta: control centering as "w", "a", "s", "d", switch "on", "off", or enter "x" if done: ')
			if string == "x":
				ok = True
				print("offset_theta: ", self.offset_theta)
				print("offset_phi: ", self.offset_phi)

			elif string == "on":
				self.turn_on()

			elif string == "off":
				self.turn_off()

			else:

				if string == "w":
					self.offset_phi += 1

				elif string == "a":
					self.offset_theta += 1

				elif string == "s":
					self.offset_phi -= 1

				elif string == "d":
					self.offset_theta -= 1
					
				theta, phi = self.body2raw(0,0)
				self.move_theta(theta)
				self.move_phi(phi)

		return True

	def calibrate_screen_dist(self):
		ok = False

		while not ok:
			string = raw_input('Enter screen distance in centimeters, laser on/off, enter "x" if done: ')
			if string == "x":
				ok = True
				print("screen_distance: ", self.screen_distance)

			elif string == "on":
				self.turn_on()

			elif string == "off":
				self.turn_off()

			else:
				self.screen_distance = float(string)
				theta, phi = self.body2raw(0,0)
				self.move_theta(theta)
				self.move_phi(phi)

		return True

"""
