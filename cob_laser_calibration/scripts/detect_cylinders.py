#!/usr/bin/env python

### Details ###
# Script: Detect cylinders of calibration object
# Author: Daniel Maeki (ipa-fxm-dm)
# Supervisor: Felix Messmer (ipa-fxm)

### Imports ###
import roslib
import rospy
import cv
import cv2
from math import pi, sin, cos, hypot, atan2


class Detect_calibration_object():
	
	# 6.1 Setup variables and reset all data for new calculations
	def __init__(self, resolution, origin, cylinder, line_color):
		self.res = resolution
		self.origin = origin
		self.cylinder_radii = cylinder["radii"]
		self.cylinder_rad_diff = self.cylinder_radii[0]/2
		self.min_rad = self.cylinder_radii[0] - self.cylinder_rad_diff
		self.max_rad = self.cylinder_radii[2] + self.cylinder_rad_diff
		self.cyl_angles = cylinder["angles"]
		self.cyl_dist = cylinder["dist"]
		self.min_dist = self.cyl_dist - (self.cylinder_radii[0])
		self.max_dist = self.cyl_dist + (self.cylinder_radii[0])
		self.line_color = line_color
		self.clear()
	
	def clear(self):
		self.cal_obj_pose = None
		self.cylinders = None
		self.circles = None
	
	# 6.4 BLABLABLA
	def filter_circles(self, circles):
		circles_to_filter = circles
		filtered_circles = []
		try:
			for circle in circles_to_filter:
				angle = atan2((circle[1]-self.origin[1]), (circle[0]-self.origin[0]))
				next_x = circle[0]
				next_y = circle[1]
				next_color = (self.image[next_x][next_y][0], self.image[next_x][next_y][1], self.image[next_x][next_y][2])
				next_dist = 0
				while next_color != self.line_color and next_dist <= self.max_rad+self.max_dist:
					next_x -= cos(angle)
					next_y -= sin(angle)
					next_color = (self.image[next_x][next_y][0], self.image[next_x][next_y][1], self.image[next_x][next_y][2])
					next_dist = hypot(circle[0]-next_x, circle[1]-next_y)
				if next_dist > self.max_rad+self.max_dist-self.cylinder_rad_diff:
					filtered_circles.append(circle)
		except(Exception), e:
			print "Error: ", e
			print "next_x = ", next_x
			print "next_y = ", next_y
		for circle in filtered_circles:
			circles_to_filter.remove(circle)
		return circles_to_filter
	
	# 6.3 BLABLABLA
	def detect_cal_obj_cylinders(self):
		# Detect circles from image
		if self.res <= 200:
			parameter2 = 18
		else:
			parameter2 = 34
		self.gray = cv2.equalizeHist(cv2.cvtColor(self.image, cv.CV_BGR2GRAY))
		self.circles = cv2.HoughCircles(self.gray, cv.CV_HOUGH_GRADIENT, 2,
						int(self.min_dist), param1=180, param2=parameter2,
						minRadius=int(self.min_rad), maxRadius=int(self.max_rad))
		
		# Remove all circles that are not part of the calibration object
		if self.circles is not None:
			temp_circles = []
			for i in range(0,len(self.circles[0])):
				temp_circles.append((self.circles[0][i][1], self.circles[0][i][0], self.circles[0][i][2]))
			
			print "Amount of plausible cylinders = ", len(temp_circles)
			check_change = 0
			while True:
				third_circles = []
				check_for_false_detections = True
				while check_for_false_detections:
					check_for_false_detections = False
					for circle_1 in temp_circles:
						remove_circle = True
						rad_min_diff = circle_1[2] - self.cylinder_rad_diff
						rad_max_diff = circle_1[2] + self.cylinder_rad_diff
						for circle_2 in temp_circles:
							dist = hypot(circle_1[0]-circle_2[0], circle_1[1]-circle_2[1])
							# if circle_2 is not in the correct distance of circle_1, continue
							if not self.min_dist < dist < self.max_dist or dist == 0:
								continue
							# if circle_2 has same radius, continue
							elif rad_min_diff < circle_2[2] < rad_max_diff:
								continue
							else:
								# if circle pair is found, create third circle
								# 2=s,m,l and 1=s,m,l
								if circle_2[2] < circle_1[2]:
									# 2=s,m and 1=m,l
									if circle_2[2] < self.cylinder_radii[0]+self.cylinder_rad_diff:
										# 2=s and 1=m,l
										if circle_1[2] > self.cylinder_radii[2]-self.cylinder_rad_diff:
											# 2=s and 1=l
											third_circles.append([circle_1[0]+dist*sin(pi/3), circle_1[1]+dist*cos(-pi/3), self.cylinder_radii[1]])
										else:
											# 2=s and 1=m
											third_circles.append([circle_1[0]+dist*sin(pi/3), circle_1[1]+dist*cos(pi/3), self.cylinder_radii[2]])
									else:
										# 2=m and 1=l
										third_circles.append([circle_1[0]+dist*sin(pi/3), circle_1[1]+dist*cos(pi/3), self.cylinder_radii[0]])
								# 2=s,m,l and 1=s,m,l
								elif circle_2[2] > circle_1[2]:
									# 2=m,l and 1=s,m
									if circle_2[2] > self.cylinder_radii[2]-self.cylinder_rad_diff:
										# 2=l and 1=s,m
										if circle_1[2] < self.cylinder_radii[0]+self.cylinder_rad_diff:
											# 2=l and 1=s
											third_circles.append([circle_1[0]+dist*sin(pi/3), circle_1[1]+dist*cos(pi/3), self.cylinder_radii[1]])
										else:
											# 2=l and 1=m
											third_circles.append([circle_1[0]+dist*sin(pi/3), circle_1[1]+dist*cos(-pi/3), self.cylinder_radii[0]])
									else:
										# 2=m and 1=s
										third_circles.append([circle_1[0]+dist*sin(pi/3), circle_1[1]+dist*cos(-pi/3), self.cylinder_radii[2]])
								
								# don't remove circle_1
								remove_circle = False
						if remove_circle:
							temp_circles.remove(circle_1)
							check_for_false_detections = True
				
				invisble_circles = []
				for circle_3 in third_circles:
					add_circle = True
					rad_min_diff = circle_3[2] - self.cylinder_rad_diff
					rad_max_diff = circle_3[2] + self.cylinder_rad_diff
					for circle in temp_circles:
						dist = hypot(circle[0]-circle_3[0], circle[1]-circle_3[1])
						if dist < self.min_dist:
							add_circle = False
						elif rad_min_diff < circle[2] < rad_max_diff:
							add_circle = False
					if add_circle:
						invisble_circles.append(circle_3)
				
				for circle in invisble_circles:
					temp_circles.append(circle)
				
				# If circle is in visible area, remove
				temp_circles = self.filter_circles(temp_circles)
				
				print "Amount of plausible cylinders = ", len(temp_circles)
				
				if check_change == len(temp_circles):
					break
				check_change = len(temp_circles)
			
			large = False
			medium = False
			small = False
			for cylinder in temp_circles:
				if self.cylinder_radii[2]-self.cylinder_rad_diff < cylinder[2] < self.cylinder_radii[2]+self.cylinder_rad_diff:
					if large:
						large = False
					else:
						large = True
				if self.cylinder_radii[0]-self.cylinder_rad_diff < cylinder[2] < self.cylinder_radii[0]+self.cylinder_rad_diff:
					if small:
						small = False
					else:
						small = True
				if self.cylinder_radii[1]-self.cylinder_rad_diff < cylinder[2] < self.cylinder_radii[1]+self.cylinder_rad_diff:
					if medium:
						medium = False
					else:
						medium = True
			
			self.circles = temp_circles
			print large, medium, small
		if self.circles is not None:
			if len(self.circles) != 3 or not large or not medium or not small:
				#print "Calibration object has exactly 3 cylinders but %i cylinders were detected" %len(self.circles)
				self.circles = None
		
		return self.circles
	
	# 6.2 This is the main function of this class
	def detect_cal_object(self, image):
		self.image = image
		self.cylinders = self.detect_cal_obj_cylinders()
		if self.cylinders is not None:
			
			# 6.2.1 Added all three cylinders and deviding them by three will give us the center coordinates of the calibration object
			cal_obj_pose_x = (self.cylinders[0][0] + self.cylinders[1][0] + self.cylinders[2][0]) / 3
			cal_obj_pose_y = (self.cylinders[0][1] + self.cylinders[1][1] + self.cylinders[2][1]) / 3
			
			# 6.2.2 Here we estimate an average yaw by getting the arctangent of the cylinder and calibration object center
			yaws = []
			cal_obj_pose_yaw = 0
			counter = 0
			for cylinder in self.cylinders:
				yaw = 0
				if self.max_rad-self.cylinder_radii[0] <= cylinder[2] <= self.max_rad:
					yaw = atan2((cal_obj_pose_y - cylinder[1]), (cal_obj_pose_x - cylinder[0])) + self.cyl_angles[2] # large
				elif self.min_rad <= cylinder[2] <= self.min_rad+self.cylinder_radii[0]:
					yaw = atan2((cal_obj_pose_y - cylinder[1]), (cal_obj_pose_x - cylinder[0])) + self.cyl_angles[0] # small
				elif self.min_rad+self.cylinder_radii[0] < cylinder[2] < self.max_rad-self.cylinder_radii[0]:
					yaw = atan2((cal_obj_pose_y - cylinder[1]), (cal_obj_pose_x - cylinder[0])) + self.cyl_angles[1] # medium
				if yaw == 0:
					self.cal_obj_pose = None
					return self.image, self.cal_obj_pose
				elif yaw < 0:
					yaw += 2*pi
				elif yaw >= 2*pi:
					yaw -= 2*pi
				yaws.append(yaw)
			
			for i in range(0,len(yaws)):
				cal_obj_pose_yaw = (cal_obj_pose_yaw * counter + yaws[i]) / (counter + 1)
				counter += 1
			print yaws
			
			self.cal_obj_pose = -(self.origin[0]-cal_obj_pose_x)/self.res, -(self.origin[1]-cal_obj_pose_y)/self.res, cal_obj_pose_yaw+pi/2
			
			# filter bad detections
			for i in range(0,len(yaws)):
				for j in range(0,len(yaws)):
					if yaws[i]-yaws[j] < -0.2 or yaws[i]-yaws[j] > 0.2:
						self.cal_obj_pose = None
		
		return self.image, self.cal_obj_pose





























'''
	# 6.3 BLABLABLA
	def detect_cal_obj_cylinders(self):
		# Detect circles from image
		if self.res <= 200:
			parameter2 = 18
		else:
			parameter2 = 34
		self.gray = cv2.equalizeHist(cv2.cvtColor(self.image, cv.CV_BGR2GRAY))
		self.circles = cv2.HoughCircles(self.gray, cv.CV_HOUGH_GRADIENT, 2,
						int(self.min_dist), param1=180, param2=parameter2,
						minRadius=int(self.min_rad), maxRadius=int(self.max_rad))
		
		# Remove all circles that are not part of the calibration object
		if self.circles is not None:
			temp_circles = []
			for i in range(0,len(self.circles[0])):
				temp_circles.append((self.circles[0][i][1], self.circles[0][i][0], self.circles[0][i][2]))
			
			self.circles = temp_circles
			print "Amount of plausible cylinders = ", len(self.circles)
			
			# If circle is within the distance between cylinders and has not the same radius, add to temp_circles
			check_change = 0
			while True:
				temp_circles = []
				for circle_1 in self.circles:
					rad_min_diff = circle_1[2] - self.cylinder_rad_diff
					rad_max_diff = circle_1[2] + self.cylinder_rad_diff
					for circle_2 in self.circles:
						dist = hypot(circle_1[0]-circle_2[0], circle_1[1]-circle_2[1])
						if (self.min_dist < dist < self.max_dist) and not (rad_min_diff < circle_2[2] < rad_max_diff):
							if circle_1 not in temp_circles:
								temp_circles.append(circle_1)
				
				# If circle is not in invisible area, remove
				temp_circles = self.filter_circles(temp_circles)
				
				self.circles = temp_circles
				if self.circles is not None:
					print "Amount of plausible cylinders = ", len(self.circles)
					if check_change == len(self.circles):
						break
					check_change = len(self.circles)
				else:
					print "Amount of plausible cylinders = 0"
					break
			
			if len(self.circles) != 2 and len(self.circles) != 3:
				self.circles = None
		
		return self.circles


	# 6.2 This is the main function of this class
	def detect_cal_object(self, image):
		self.image = image
		self.cylinders = self.detect_cal_obj_cylinders()
		if self.cylinders is not None:
			
			invalid_result = False
			rad_min_diff = self.cylinders[0][2] - self.cylinder_rad_diff
			rad_max_diff = self.cylinders[0][2] + self.cylinder_rad_diff
			if self.cylinders[1][2] < rad_min_diff:
				if self.cylinder_radii[2] < rad_max_diff+self.cylinder_radii[0]:
					if self.cylinders[1][2] < rad_min_diff-self.cylinder_radii[0]:
						# self.cylinders[0] = large, self.cylinders[1] = small, yaw = medium
						angle_xy = -60
						angle_yaw = self.cyl_angles[1]
					else:
						# self.cylinders[0] = large, self.cylinders[1] = medium, yaw = small
						angle_xy = 60
						angle_yaw = self.cyl_angles[0]
				else:
					# self.cylinders[0] = medium, self.cylinders[1] = small, yaw = large
						angle_xy = 60
						angle_yaw = self.cyl_angles[2]
			elif self.cylinders[1][2] > rad_max_diff:
				if self.cylinder_radii[0] > rad_min_diff-self.cylinder_radii[0]:
					if self.cylinders[1][2] > rad_max_diff+self.cylinder_radii[0]:
						# self.cylinders[0] = small, self.cylinders[1] = large, yaw = medium
						angle_xy = 60
						angle_yaw = self.cyl_angles[1]
					else:
						# self.cylinders[0] = small, self.cylinders[1] = medium, yaw = large
						angle_xy = -60
						angle_yaw = self.cyl_angles[2]
				else:
					# self.cylinders[0] = medium, self.cylinders[1] = large, yaw = small
						angle_xy = -60
						angle_yaw = self.cyl_angles[0]
			else:
				invalid_result = True
				self.cylinders = []
				
			if not invalid_result:
				dist = (self.cyl_dist/2) / cos(60/180*pi)
				cal_obj_pose_x = self.cylinders[0][0]+dist*sin(angle_xy/180*pi)
				cal_obj_pose_y = self.cylinders[0][1]+dist*cos(angle_xy/180*pi)
				cal_obj_pose_yaw = atan2(cal_obj_pose_y-self.cylinders[0][1], cal_obj_pose_x-self.cylinders[0][0]) + angle_yaw
				self.cal_obj_pose = (self.origin[0]-cal_obj_pose_x)/self.res, (self.origin[1]-cal_obj_pose_y)/self.res, cal_obj_pose_yaw# + pi
			

'''
