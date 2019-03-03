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
		self.screen_min_y = np.tan(np.deg2rad(self.min_phi_raw - 90)) * self.screen_distance
		self.screen_max_y = np.tan(np.deg2rad(self.max_phi_raw - 90)) * self.screen_distance
		self.screen_min_x = np.tan(np.deg2rad(self.min_theta)) * self.screen_distance
		self.screen_max_x = np.tan(np.deg2rad(self.max_theta)) * self.screen_distance

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

	def move2xy(self, x, y):
		# move to xy coord on screen

		theta, phi = self.screen2angles(x, y)
		self.move_theta(round(theta))
		self.move_phi(round(phi))
		return True

	def screen2angles(self, x, y):
		# get angle from screen coordinates (range 0~1 in x y)
		w = self.screen_max_x - self.screen_min_x
		h = self.screen_max_y - self.screen_min_y
		x_ = -(x - 0.5)*w
		y_ = y * h + self.screen_min_y

		phi_ = np.arctan2(y_, self.screen_distance)
		theta_ = np.arctan2(x_, self.screen_distance)

		phi_raw = np.rad2deg(phi_) + 90

		# adjust theta_raw from screen distance 
		d_theta_raw = np.arctan2(self.offset_delta, np.sqrt(np.tan(theta_)**2 + self.screen_distance**2))

		theta_raw = np.rad2deg(theta_ + d_theta_raw) + 90

		return theta_raw, phi_raw

	def test(self):
		ok = False

		while not ok:
			string = raw_input("Enter 'x-y' between 0 and 1, or 'x' if done: ")
			if string == 'x':
				ok = True
			else:
				coords = string.split('-')
				self.move2xy(float(coords[0]), float(coords[1]))
		return True


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

		self.screen_min_y = np.tan(np.deg2rad(self.min_phi_raw - 90)) * self.screen_distance
		self.screen_max_y = np.tan(np.deg2rad(self.max_phi_raw - 90)) * self.screen_distance
		self.screen_min_x = np.tan(np.deg2rad(self.min_theta_raw)) * self.screen_distance
		self.screen_max_x = np.tan(np.deg2rad(self.max_theta_raw)) * self.screen_distance

		print("phi_min: ", self.min_phi_raw, " phi_max: ", self.max_phi_raw)
		print("theta_min: ", self.min_theta_raw, " theta_max: ", self.max_theta_raw)
		print("screen_distance: ", self.screen_distance)


if __name__ == "__main__":
	laser_control = LaserControl(phi_min=81, phi_max=106, theta_min=-11.5, theta_max=11.5, screen_distance=18.65)
	# laser_control.three_point_calibrate()
	laser_control.test()



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
