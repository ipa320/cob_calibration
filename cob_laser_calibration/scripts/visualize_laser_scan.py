#!/usr/bin/env python

### Details ###
# Script: 
# Author: Daniel Maeki (ipa-fxm-dm)
# Supervisor: Felix Messmer (ipa-fxm)

### Imports ###
import roslib
import rospy
import numpy as np
import cv
import cv2
from math import sqrt, sin, cos, tanh
from sensor_msgs.msg import LaserScan


class Get_laserscan():
	def __init__(self):
		self.clear()

	def clear(self):
		self.result = LaserScan()
		self.counter = 0

	def append_laserscan(self, laserscan):
		if self.result.header.frame_id is not "":
			assert self.result.header.frame_id == laserscan.header.frame_id
			self.result.ranges = list((np.array(self.result.ranges) * self.counter + np.array(laserscan.ranges)) / (self.counter + 1))
		else:
			self.result = laserscan
		self.counter += 1

	def get_count(self):
		return self.counter

	def get_result(self):
		return self.result


class Visualize_laserscan():
	def __init__(self, resolution, border, max_laser_point_dist, laserscan_pose):
		self.res = resolution
		self.border = border
		self.max_laser_point_dist = max_laser_point_dist
		self.base_pose = abs(laserscan_pose[0][0]) * self.res, abs(laserscan_pose[0][1]) * self.res #####################################
		self.base_size = max(self.base_pose[0], self.base_pose[1])
		self.clear()

	def clear(self):
		self.raw_image = None
		self.points = []
		self.x = []
		self.y = []

	def calculate_image_points(self, scan):
		self.angles = [scan.angle_min]
		while self.angles[-1] <= scan.angle_max:
			self.angles.append(self.angles[-1] + scan.angle_increment)
		for distance, angle in zip(scan.ranges, self.angles):
			if distance != 0.0:
				if distance > self.max_laser_point_dist:
					distance = self.max_laser_point_dist
				self.x.append(distance * cos(angle) * self.res)
				self.y.append(distance * sin(angle) * self.res)
				self.points.append([self.x[-1], self.y[-1]])

	def create_image(self):
		self.x_range = max(abs(min(self.x)), max(self.x)) + self.border
		self.y_range = max(abs(min(self.y)), max(self.y)) + self.border
		self.origin = (self.border + self.base_size * 2, self.y_range)
		for i in range(0,len(self.points)):
			self.points[i][0] += self.origin[0]
			self.points[i][1] += self.origin[1]
		self.raw_image = np.zeros((int(self.x_range + self.border + self.base_size * 2), int(self.y_range * 2), 3), np.uint8)

	def interpolate_points(self):
		for i in range(0,len(self.points)-1):
			cv2.line(self.raw_image, (int(self.points[i][1]),int(self.points[i][0])), (int(self.points[i+1][1]),int(self.points[i+1][0])), (0, 127, 127), 1)

	def draw_fixed_objects(self):
		cv2.circle(self.raw_image, (int(self.origin[1]), int(self.origin[0])), 4, (255, 255, 0), 4) # the position of the laser scanner
		self.base_pose = self.origin[0] - (self.base_pose[0]), self.origin[1] - (self.base_pose[1])
		cv2.circle(self.raw_image, (int(self.base_pose[1]), int(self.base_pose[0])), 2, (255, 255, 0), 1)
		cv2.line(self.raw_image, (int(self.origin[1]), int(self.origin[0])),  (int(self.base_pose[1]), int(self.base_pose[0])), (255, 0, 0), 1)
		cv2.rectangle(self.raw_image, (int(self.base_pose[1]-self.base_size), int(self.base_pose[0]-self.base_size)), (int(self.base_pose[1]+self.base_size), int(self.base_pose[0]+self.base_size)), (127, 127, 127), 1)

	def convert_to_image(self, laserscan):
		self.calculate_image_points(laserscan)
		self.create_image()
		self.interpolate_points()
		self.draw_fixed_objects()
		return self.raw_image, self.origin

	def show_image(self, image):
		cv.ShowImage("laser_scan", cv.fromarray(image))
		cv.WaitKey()


class Detect_calibration_object():
	def __init__(self, resolution, origin, frustum):
		self.res = resolution
		self.origin = origin
		self.frustum = frustum
		self.clear()

	def clear(self):
		self.cal_obj_pose = None

	def detect_circles(self):
		self.gray = cv2.equalizeHist(cv2.cvtColor(self.image, cv.CV_BGR2GRAY))
		self.circles = cv2.HoughCircles(self.gray, cv.CV_HOUGH_GRADIENT, 2, int(self.frustum["min_dist"]), param1=180, param2=24, minRadius=int(self.frustum["dim"][1]), maxRadius=int(self.frustum["dim"][0]))
		if self.circles is not None:
			temp_circles = []
			for i in range(0,len(self.circles[0])):
				temp_circles.append((self.circles[0][i][1], self.circles[0][i][0], self.circles[0][i][2]))
			check_for_false_detections = True
			while check_for_false_detections:
				check_for_false_detections = False
				for circle_1 in temp_circles:
					frustums_detected = 0
					for circle_2 in temp_circles:
						dist = sqrt((circle_1[0]-circle_2[0])**2+(circle_1[1]-circle_2[1])**2)
						if self.frustum["min_dist"] <= dist <= self.frustum["max_dist"] and dist != 0.0:
							frustums_detected += 1
					if frustums_detected < self.frustum["amount"]-1:
						temp_circles.remove(circle_1)
						check_for_false_detections = True
			self.circles = temp_circles
			if len(self.circles) == self.frustum["amount"]:
				for circle in self.circles:
					cv2.circle(self.image, (circle[1], circle[0]), int(self.frustum["dim"][1]), (127, 0, 127), 1)
					cv2.circle(self.image, (circle[1], circle[0]), int(self.frustum["dim"][0]), (127, 0, 127), 1)
					cv2.circle(self.image, (circle[1], circle[0]), int(self.frustum["min_dist"]), (127, 127, 0), 1)
					cv2.circle(self.image, (circle[1], circle[0]), int(self.frustum["max_dist"]), (127, 127, 0), 1)
					cv2.circle(self.image, (circle[1], circle[0]), int(circle[2]), (0, 0, 255), 1)
				return self.circles
			else:
				print "Calibration object has exactly %i frustums but %i frustums were detected" %(self.frustum["amount"], len(self.circles))
		self.circles = None
		return self.circles

	def determine_cal_obj_pose(self):
		self.frustum_distances = []
		for frustum_1 in self.frustums:
			for frustum_2 in self.frustums:
				dist = sqrt((frustum_2[0]-frustum_1[0])**2+(frustum_2[1]-frustum_1[1])**2)/self.res*100
				if dist != 0.0 and dist not in self.frustum_distances:
					self.frustum_distances.append(dist)
				cv2.line(self.image, (int(frustum_1[1]),int(frustum_1[0])), (int(frustum_2[1]),int(frustum_2[0])), (127, 0, 127), 1)
		# create another calibration object validation test using frustum distances
		cal_obj_center_x = (self.frustums[0][0] + self.frustums[1][0] + self.frustums[2][0]) / 3
		cal_obj_center_y = (self.frustums[0][1] + self.frustums[1][1] + self.frustums[2][1]) / 3
		cv2.circle(self.image, (int(cal_obj_center_y), int(cal_obj_center_x)), 2, (255, 255, 0), 1)
		self.cal_obj_pose = cal_obj_center_x, cal_obj_center_y
		cv2.line(self.image, (int(self.cal_obj_pose[1]),int(self.cal_obj_pose[0])), (int(self.origin[1]),int(self.origin[0])), (255, 0, 0), 1)
		self.cal_obj_pose = -1 * (self.origin[0]-self.cal_obj_pose[0]) / self.res, (self.origin[1]-self.cal_obj_pose[1]) / self.res
		return self.cal_obj_pose

	def detect_cal_object(self, image):
		self.image = image
		self.frustums = self.detect_circles()
		if self.frustums is not None:
			self.determine_cal_obj_pose()
			self.cal_obj_pose = ((self.cal_obj_pose[0], self.cal_obj_pose[1], 0), (0, 0, 0))
		else:
			print "Could not detect calibration object..."
		return self.image, self.cal_obj_pose
