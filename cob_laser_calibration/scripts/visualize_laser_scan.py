#!/usr/bin/env python

### DETAILS ###
# Script: Create average laser scan from multiple laser scan detection and visualize the laser scan image
# Author: Daniel Maeki (ipa-fxm-dm)
# Supervisor: Felix Messmer (ipa-fxm)

### IMPORTS ###
import roslib
import rospy
import numpy as np
import cv
import cv2
from math import pi, sin, cos, hypot, atan2
from sensor_msgs.msg import LaserScan

### GLOBAL VARIABLES ###
# ---

### POSSIBLE_IMPROVEMENTS ###
# Instead of plainly connecting each point, create smoother lines. This might improve the circle detection

### TODO ###
# Convert all python list calculation into numpy calculations, speeding up the overall process
# The yaw value for visualization is not correct, this should be corrected but it has no influence on the calibration end results, it is purely for visualization.


### SCRIPT ###
class Get_laserscan():
	
	# 4.1 When initialized, purge the data needed for new calculations
	def __init__(self):
		self.clear()
	
	def clear(self):
		self.result = LaserScan()
		self.counter = 0
	
	# 4.2 Each time this function is run, a new average using the new laser scan is calculated
	def append_laserscan(self, laserscan):
		# Check if the laser scan message is indeed the same message
		if self.result.header.frame_id is not "":
			try:
				assert self.result.header.frame_id == laserscan.header.frame_id
			except(AssertionError):
				print "\n\n\n--> The 'LaserScan' message used for calculating the average laser scan, is not the same"
				print "--> Check if both scripts have specified the correct path to this 'LaserScan' message\n\n\n"
				exit()
			# Calculate average
			self.result.ranges = list((np.array(self.result.ranges) * self.counter + np.array(laserscan.ranges)) / (self.counter + 1))
		else:
			self.result = laserscan
		self.counter += 1
	
	def get_count(self):
		return self.counter
	
	def get_result(self):
		return self.result


class Visualize_laserscan():
	
	# 5.0 Setup variables and purge the data needed for new calculations
	def __init__(self, resolution, border, max_laser_point_dist, laserscan_pose, cylinder, line_color):
		self.res = resolution
		self.border = border
		self.max_laser_point_dist = max_laser_point_dist
		self.base_pose = abs(laserscan_pose[0][0]) * self.res, abs(laserscan_pose[0][1]) * self.res
		self.base_size = max(self.base_pose[0], self.base_pose[1])
		self.cylinder_radii = cylinder["radii"]
		self.cylinder_rad_diff = self.cylinder_radii[0]/3
		self.min_rad = self.cylinder_radii[0] - self.cylinder_rad_diff
		self.max_rad = self.cylinder_radii[2] + self.cylinder_rad_diff
		self.cyl_angles = cylinder["angles"]
		self.cyl_dist = (cylinder["dist"]/2) / cos(pi/6)
		self.min_dist = cylinder["dist"] - (self.cylinder_radii[0]/2)
		self.max_dist = cylinder["dist"] + (self.cylinder_radii[0]/2)
		self.line_color = line_color
		self.clear()
	
	def clear(self):
		self.raw_image = None
		self.points = []
		self.x = []
		self.y = []
	
	# 5.2 Using the average laser scan measurements we can calculate the x and y coordinates for all laser scan points
	def calculate_image_points(self, scan):
		self.angles = [scan.angle_min]
		# Increment the angle until the maximum angle is reach
		while self.angles[-1] <= scan.angle_max:
			self.angles.append(self.angles[-1] + scan.angle_increment)
		# Calculate the x and y coordinates for each point using the distance and angle
		for distance, angle in zip(scan.ranges, self.angles):
			# Skip all empty detections
			if distance != 0.0:
				# The distance may not exceed the max_laser_point_dist
				if distance > self.max_laser_point_dist:
					distance = self.max_laser_point_dist
				self.x.append(distance * cos(angle) * self.res)
				self.y.append(distance * sin(angle) * self.res)
				self.points.append([self.x[-1], self.y[-1]])
	
	# 5.3 Create image and set origin as the laser scanner in the image
	def create_image(self):
		# Here we first calculate what the maximum range is for both x and y values
		self.x_range = max(abs(min(self.x)), max(self.x)) + self.border
		self.y_range = max(abs(min(self.y)), max(self.y)) + self.border
		# Set origin, this is also the position of the laser scanner
		self.origin = (self.border + self.base_size * 2, self.y_range)
		# Here we add the origin values to all x and y values so that the origin will indeed show as the origin point in the image
		for i in range(0,len(self.points)):
			self.points[i][0] += self.origin[0]
			self.points[i][1] += self.origin[1]
		# Create image
		self.raw_image = np.zeros((int(self.x_range + self.border + self.base_size * 2), int(self.y_range * 2), 3), np.uint8)
	
	# 5.4 Interpolate laser scan points, here we draw a line between each 2 points so that we can later on detect the circles of the cylinders.
	# POSSIBLE_IMPROVEMENT: Instead of plainly connecting each point, create smoother lines. This might improve the circle detection
	def interpolate_points(self):
		for i in range(0,len(self.points)-1):
			# Before colouring the pixels between each two points, we calculate the distance and angle between every two points.
			dist = hypot(self.points[i+1][0]-self.points[i][0], self.points[i+1][1]-self.points[i][1])
			angle = atan2((self.points[i+1][1]-self.points[i][1]), (self.points[i+1][0]-self.points[i][0]))
			next_x = self.points[i][0]
			next_y = self.points[i][1]
			next_dist = 0
			# A pixel has a value of one so each round we add cos(angle) to the x-axis and sin(angle) to the y-axis until we reach the next point.
			while next_dist <= dist:
				self.raw_image[int(round(next_x))][int(round(next_y))] = self.line_color
				next_x += cos(angle)
				next_y += sin(angle)
				next_dist = hypot(self.points[i][0]-next_x, self.points[i][1]-next_y)
	
	# 5.5 Draw the fixed objects in the image, the laser scanner and the base
	def draw_fixed_objects(self):
		# Draw laser scanner
		cv2.circle(self.raw_image, (int(self.origin[1]), int(self.origin[0])), 4, (255, 255, 0), 4)
		# Set base coordinates and draw the base center and box
		self.base_pose = self.origin[0] - self.base_pose[0], self.origin[1] - self.base_pose[1]
		cv2.circle(self.raw_image, (int(self.base_pose[1]), int(self.base_pose[0])), 2, (255, 255, 0), 1)
		cv2.rectangle(self.raw_image, (int(self.base_pose[1]-self.base_size), int(self.base_pose[0]-self.base_size)),
			(int(self.base_pose[1]+self.base_size), int(self.base_pose[0]+self.base_size)), (127, 127, 127), 1)
	
	# 5.1 This is the main function of this class
	def convert_to_image(self, laserscan):
		self.calculate_image_points(laserscan)
		self.create_image()
		self.interpolate_points()
		self.draw_fixed_objects()
		return self.raw_image, self.origin
	
	# 10.1 Draw calibration object from detected average calibration object pose
	# TODO: The yaw value for visualization is not correct, this should be corrected but it has no influence on the calibration end results, it is purely for visualization.
	def draw_calibration_object(self, image, cal_obj_pose):
		# Set and draw x and y coordinates in image of the detected calibration object pose.
		cal_obj_pose_in_image = self.base_pose[1] - cal_obj_pose[1] * self.res, self.base_pose[0] - cal_obj_pose[0] * self.res, cal_obj_pose[2]
		cv2.circle(image, (int(cal_obj_pose_in_image[0]), int(cal_obj_pose_in_image[1])), 2, (0, 0, 255), 1)
		# Set and draw each cylinder with respect to the calibration object center
		for i in range(0,len(self.cyl_angles)):
			cyl_x = self.cyl_dist * cos(self.cyl_angles[i]+cal_obj_pose_in_image[2]) + cal_obj_pose_in_image[0]
			cyl_y = self.cyl_dist * sin(self.cyl_angles[i]+cal_obj_pose_in_image[2]) + cal_obj_pose_in_image[1]
			cv2.circle(image, (int(cyl_x), int(cyl_y)), int(self.cylinder_radii[i]), (0, 0, 255), 1)
		# Draw the minimun and maximum distance of the cylinders with respect to the center
		cv2.circle(image, (int(cal_obj_pose_in_image[0]), int(cal_obj_pose_in_image[1])), int(self.cyl_dist-self.cylinder_radii[0]), (255, 255, 0), 1)
		cv2.circle(image, (int(cal_obj_pose_in_image[0]), int(cal_obj_pose_in_image[1])), int(self.cyl_dist+self.cylinder_radii[0]), (255, 255, 0), 1)
		# Draw line between calibration object pose and base pose
		cv2.line(image, (int(cal_obj_pose_in_image[0]), int(cal_obj_pose_in_image[1])),  (int(self.base_pose[1]), int(self.base_pose[0])), (255, 0, 0), 1)
		return image
	
	# 10.2 If the resolution is 200 or smaller, we visualize the end result containing the; laser scan, fixed objects and calibration object
	def show_image(self, image):
		cv.ShowImage("laser_scan", cv.fromarray(image))
		# Wait until a key is pressed before closing the image and continuing the script
		cv.WaitKey()

