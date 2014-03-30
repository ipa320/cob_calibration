#!/usr/bin/env python

### DETAILS ###
# Script: Convert the calibration object pose with respect to the base into x, y and z coordinates of the checkerboard points
# Author: Daniel Maeki (ipa-fxm-dm)
# Supervisor: Felix Messmer (ipa-fxm)

### IMPORTS ###
import roslib
import rospy
from math import sin, cos

### POSSIBLE_IMPROVEMENTS ###
# ---

### TODO ###
# Check if the positions of each checkerboard_point is correct
# Check if order of checkerboard_points is correct


### SCRIPT ###
class Convert_cal_obj_pose():
	
	def __init__(self, cal_obj_pose):
		self.cal_obj_pose = cal_obj_pose
	
	def pose_to_points(self):
		checkerboard_dims = rospy.get_param('checkerboard', None)
		
		square_size = checkerboard_dims['square_size']
		y_points_amount = checkerboard_dims['checkerboards']['cb_9x6']['corners_y']
		x_points_amount = checkerboard_dims['checkerboards']['cb_9x6']['corners_x']
		checkerboard_center = [self.cal_obj_pose[0][0], self.cal_obj_pose[0][1]]
		first_checkerboard_point = checkerboard_center
		first_checkerboard_point[0] -= cos(self.cal_obj_pose[1][2])*x_points_amount/2*square_size
		first_checkerboard_point[1] -= cos(self.cal_obj_pose[1][2])*y_points_amount/2*square_size
		
		checkerboard_points = []
		for i in range(0,y_points_amount):
			for j in range(0,x_points_amount):
				checkerboard_point_x = first_checkerboard_point[0]+cos(self.cal_obj_pose[1][2])*j*square_size
				checkerboard_point_y = first_checkerboard_point[1]+cos(self.cal_obj_pose[1][2])*i*square_size
				checkerboard_points.append([checkerboard_point_x, checkerboard_point_y, self.cal_obj_pose[0][2]])
		
		return checkerboard_points
