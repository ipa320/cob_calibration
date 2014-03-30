#!/usr/bin/env python

### DETAILS ###
# Script: Detect cylinders of calibration object
# Author: Daniel Maeki (ipa-fxm-dm)
# Supervisor: Felix Messmer (ipa-fxm)

### IMPORTS ###
import roslib
import rospy
import cv
import cv2
from math import pi, sin, cos, hypot, atan2

### GLOBAL VARIABLES ###
# ---

### POSSIBLE_IMPROVEMENTS ###
# ---

### TODO ###
# Convert all python list calculation into numpy calculations, speeding up the overall process (optional)
# There is a flaw in function 'filter_circles', it is going outside the laser scan image resulting in an error, this has to be fixed (may not be true anymore)
# The function 'detect_cal_obj_cylinders' is working properly but should still be cleaned up


### SCRIPT ###
class Detect_calibration_object():
	
	# 6.0 Setup variables and purge the data needed for new calculations
	def __init__(self, resolution, origin, cylinder, line_color):
		self.res = resolution
		self.origin = origin
		self.cylinder_radii = cylinder["radii"]
		self.cylinder_rad_diff = self.cylinder_radii[0]/2
		self.min_rad = self.cylinder_radii[0] - self.cylinder_rad_diff
		self.max_rad = self.cylinder_radii[2] + self.cylinder_rad_diff
		self.cyl_angles = cylinder["angles"]
		self.cyl_dist = cylinder["dist"]
		self.min_dist = self.cyl_dist - self.cylinder_radii[0]
		self.max_dist = self.cyl_dist + self.cylinder_radii[0]
		self.line_color = line_color
		self.clear()
	
	def clear(self):
		self.cal_obj_pose = None
		self.cylinders = None
		self.circles = None
	
	# 6.3 This function filters all circles that are in the visible area of the laser scanner because these circles can not be part of the calibration object
	# TODO: There is a flaw in this function where it is going outside the laser scan image resulting in an error, this has to be fixed
	def filter_circles(self, circles):
		circles_to_filter = circles
		filtered_circles = []
		try:
			# Here we go over every pixel along the line between the circle center and the laser scanner, checking if we cross the laser scan's border between the visible and invisible area
			for circle in circles_to_filter:
				# Calculate the angle between the circle center and the laser scanner
				angle = atan2((circle[1]-self.origin[1]), (circle[0]-self.origin[0]))
				# Set the starting position
				next_x = circle[0]
				next_y = circle[1]
				# Get the color of the current pixel
				next_color = (self.image[next_x][next_y][0], self.image[next_x][next_y][1], self.image[next_x][next_y][2])
				next_dist = 0
				# Loop until border is crossed or until the distance between the current position and the staring position is too large
				add = True
				while next_dist <= self.cyl_dist*2:
					# Calculate the next pixel's x and y value
					next_x -= cos(angle)
					next_y -= sin(angle)
					# Get the color of the next pixel
					next_color = (self.image[next_x][next_y][0], self.image[next_x][next_y][1], self.image[next_x][next_y][2])
					if next_color == self.line_color:
						add = False
						break
					# Get the current distance between the current position and the starting position
					next_dist = hypot(circle[0]-next_x, circle[1]-next_y)
				if add:
					filtered_circles.append(circle)
		except(Exception), e:
			print "Error: ", e
			print "next_x = ", next_x
			print "next_y = ", next_y
			print "origin = ", self.origin
		
		# Remove all circles which are no plausible cylinder of the calibration object
		for circle in filtered_circles:
			circles_to_filter.remove(circle)
		
		return circles_to_filter
	
	# 6.2 Detect all cylinders of the calibration object by using an opencv function called HoughCircles that detects all circular objects in the laser scan image
	# TODO: This function is working properly but should still be cleaned up
	def detect_cal_obj_cylinders(self):
		# Set parameter depending on resolution
		if self.res <= 200:
			parameter2 = 18
		else:
			parameter2 = 34
		# Detect circles from image using opencv
		self.gray = cv2.equalizeHist(cv2.cvtColor(self.image, cv.CV_BGR2GRAY))
		self.circles = cv2.HoughCircles(self.gray, cv.CV_HOUGH_GRADIENT, 2,
						int(self.min_dist), param1=180, param2=parameter2,
						minRadius=int(self.min_rad), maxRadius=int(self.max_rad))
		
		# Eliminate all circles that are can not be part of the calibration object
		if self.circles is not None:
			temp_circles = []
			for i in range(0,len(self.circles[0])):
				temp_circles.append((self.circles[0][i][1], self.circles[0][i][0], self.circles[0][i][2]))
			
			print "Amount of plausible cylinders = ", len(temp_circles)
			no_change = 1
			prev_len = 0
			# Loop until no change for 3 rounds is observed
			while True:
				check_for_false_detections = True
				# Loop until no falsely detected circle is observed anymore
				while check_for_false_detections:
					check_for_false_detections = False
					# Check for each pair of cylinders if they could be part of the calibration object
					for circle_1 in temp_circles:
						remove_circle = True
						rad_min_diff = circle_1[2] - self.cylinder_rad_diff
						rad_max_diff = circle_1[2] + self.cylinder_rad_diff
						for circle_2 in temp_circles:
							dist = hypot(circle_1[0]-circle_2[0], circle_1[1]-circle_2[1])
							# If circle_2 is not in the correct distance of circle_1, continue
							if not self.min_dist < dist < self.max_dist or dist == 0:
								continue
							# If circle_2 has a radius which is too similar to circle_1, continue
							elif rad_min_diff < circle_2[2] < rad_max_diff:
								continue
							else:
								# If plausible pair is found, don't remove circle_1
								remove_circle = False
						if remove_circle:
							# If no plausible cylinder pair is found, remove circle_1
							temp_circles.remove(circle_1)
							check_for_false_detections = True
				
				# Plausible circle pair is found, now determine position of third circle
				# Depending on the radii of circle_1 and circle_2, we can determine the position of the third cylinder
				third_circles = []
				for circle_1 in temp_circles:
					for circle_2 in temp_circles:
						if circle_1 == circle_2:
							continue
						
						theta = atan2((circle_2[1] - circle_1[1]), (circle_2[0] - circle_1[0]))
						
						if circle_1[2] > circle_2[2]:
							if circle_1[2] > self.cylinder_radii[2]-self.cylinder_rad_diff:
								if circle_2[2] < self.cylinder_radii[0]+self.cylinder_rad_diff:
									# circle_1 = large, circle_2 = small
									third_circles.append([((-self.cyl_dist)*cos((-pi/3)+theta)+circle_1[0]), ((-self.cyl_dist)*sin((-pi/3)+theta)+circle_1[1]), self.cylinder_radii[1]])
								elif circle_2[2] < self.cylinder_radii[1]+self.cylinder_rad_diff:
									# circle_1 = large, circle_2 = medium
									third_circles.append([(self.cyl_dist*cos(pi/3+theta)+circle_1[0]), (self.cyl_dist*sin(pi/3+theta)+circle_1[1]), self.cylinder_radii[0]])
							elif circle_1[2] > self.cylinder_radii[1]-self.cylinder_rad_diff:
								if circle_2[2] < self.cylinder_radii[0]+self.cylinder_rad_diff:
									# circle_1 = medium, circle_2 = small
									third_circles.append([(self.cyl_dist*cos(pi/3+theta)+circle_1[0]), (self.cyl_dist*sin(pi/3+theta)+circle_1[1]), self.cylinder_radii[2]])
						elif circle_1[2] < circle_2[2]:
							if circle_1[2] < self.cylinder_radii[0]+self.cylinder_rad_diff:
								if circle_2[2] > self.cylinder_radii[2]-self.cylinder_rad_diff:
									# circle_1 = small, circle_2 = large
									third_circles.append([(self.cyl_dist*cos(pi/3+theta)+circle_1[0]), (self.cyl_dist*sin(pi/3+theta)+circle_1[1]), self.cylinder_radii[1]])
								elif circle_2[2] > self.cylinder_radii[1]-self.cylinder_rad_diff:
									# circle_1 = small, circle_2 = medium
									third_circles.append([((-self.cyl_dist)*cos((-pi/3)+theta)+circle_1[0]), ((-self.cyl_dist)*sin((-pi/3)+theta)+circle_1[1]), self.cylinder_radii[2]])
							elif circle_1[2] < self.cylinder_radii[1]+self.cylinder_rad_diff:
								if circle_2[2] > self.cylinder_radii[2]-self.cylinder_rad_diff:
									# circle_1 = medium, circle_2 = large
									third_circles.append([((-self.cyl_dist)*cos((-pi/3)+theta)+circle_1[0]), ((-self.cyl_dist)*sin((-pi/3)+theta)+circle_1[1]), self.cylinder_radii[0]])
				
				#print "third_circle_amount = ", len(third_circles)
				
				# Check for each third cylinder if they could be part of the calibration object
				undetected_circles = []
				for circle_3 in third_circles:
					add_circle = True
					rad_min_diff = circle_3[2] - self.cylinder_rad_diff
					rad_max_diff = circle_3[2] + self.cylinder_rad_diff
					for circle in temp_circles:
						dist = hypot(circle[0]-circle_3[0], circle[1]-circle_3[1])
						# If circle is not in the correct distance of circle_3, don't add the third circle
						if not self.min_dist < dist:
							add_circle = False
						# If circle has a radius which is too similar to circle_3, don't add the third circle
						elif rad_min_diff < circle[2] < rad_max_diff:
							add_circle = False
					if add_circle:
						# Append the thrid cylinder to the undetected circles because it was not detected by the opencv function
						undetected_circles.append(circle_3)
				
				#print "undetected_circle_amount = ", len(undetected_circles)
				
				# Append each third cylinder to temp.circles that could be part of the calibration object
				for circle in undetected_circles:
					temp_circles.append(circle)
				
				# Each circle that is not in the invisible area of the laser scan should be removed
				temp_circles = self.filter_circles(temp_circles)
				
				print "Amount of plausible cylinders = ", len(temp_circles)
				
				# If the amount of the plausible cylinders does not change for 3 rounds, end the loop
				if prev_len == len(temp_circles):
					if no_change == 2:
						break
					no_change += 1
				prev_len = len(temp_circles)
				
			# If no changes to the amount of plausible cylinders are found, set True or False for each cylinder size
			large = False
			medium = False
			small = False
			for cylinder in temp_circles:
				if self.cylinder_radii[2]-self.cylinder_rad_diff < cylinder[2] < self.cylinder_radii[2]+self.cylinder_rad_diff:
					large = True
				if self.cylinder_radii[0]-self.cylinder_rad_diff < cylinder[2] < self.cylinder_radii[0]+self.cylinder_rad_diff:
					small = True
				if self.cylinder_radii[1]-self.cylinder_rad_diff < cylinder[2] < self.cylinder_radii[1]+self.cylinder_rad_diff:
					medium = True
			
			# Set self.cylinders to temp_circles, now there is no going back
			self.circles = temp_circles
			print large, medium, small
		
		if self.circles is not None:
			# Check if the amount of detected circles is three and each cylinder size is existent
			if len(self.circles) != 3 or not large or not medium or not small:
				self.circles = None
		
		return self.circles
	
	# 6.1 This is the main function of this class, here we extract the positions of the cylinders and convert this into the calibration object pose
	def detect_cal_object(self, image):
		self.image = image
		# Detect the positions of the cylinders
		self.cylinders = self.detect_cal_obj_cylinders()
		if self.cylinders is not None:
			
			# Adding all three cylinders and deviding them by three will give us the center coordinates of the calibration object
			cal_obj_pose_x = (self.cylinders[0][0] + self.cylinders[1][0] + self.cylinders[2][0]) / 3
			cal_obj_pose_y = (self.cylinders[0][1] + self.cylinders[1][1] + self.cylinders[2][1]) / 3
			
			# Here we estimate an average yaw by getting the arctangent of the cylinder and calibration object center
			yaws = []
			cal_obj_pose_yaw = 0
			counter = 0
			for cylinder in self.cylinders:
				yaw = 0
				if self.max_rad-self.cylinder_radii[0] <= cylinder[2] <= self.max_rad:
				# If current cylinder is the large cylinder then:
					yaw = atan2((cal_obj_pose_y - cylinder[1]), (cal_obj_pose_x - cylinder[0])) + self.cyl_angles[2]
				elif self.min_rad <= cylinder[2] <= self.min_rad+self.cylinder_radii[0]:
				# If current cylinder is the small cylinder then:
					yaw = atan2((cal_obj_pose_y - cylinder[1]), (cal_obj_pose_x - cylinder[0])) + self.cyl_angles[0]
				elif self.min_rad+self.cylinder_radii[0] < cylinder[2] < self.max_rad-self.cylinder_radii[0]:
				# If current cylinder is the medium cylinder then:
					yaw = atan2((cal_obj_pose_y - cylinder[1]), (cal_obj_pose_x - cylinder[0])) + self.cyl_angles[1]
				# This will set all yaw to the same Pi range so that it will be possible to calculate an average
				if yaw == 0:
					self.cal_obj_pose = None
					return self.image, self.cal_obj_pose
				elif yaw < 0:
					yaw += 2*pi
				elif yaw >= 2*pi:
					yaw -= 2*pi
				yaws.append(yaw)
			 
			# Calculate average of yaws
			for i in range(0,len(yaws)):
				cal_obj_pose_yaw = (cal_obj_pose_yaw * counter + yaws[i]) / (counter + 1)
				counter += 1
			print yaws
			
			# Set calibration object pose
			self.cal_obj_pose = -(self.origin[0]-cal_obj_pose_x)/self.res, -(self.origin[1]-cal_obj_pose_y)/self.res, cal_obj_pose_yaw+pi/2
			
			# Filter bad detections
			for i in range(0,len(yaws)):
				for j in range(0,len(yaws)):
					if yaws[i]-yaws[j] < -0.2 or yaws[i]-yaws[j] > 0.2:
						self.cal_obj_pose = None
		
		return self.image, self.cal_obj_pose

