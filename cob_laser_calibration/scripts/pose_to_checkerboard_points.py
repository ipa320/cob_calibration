#!/usr/bin/env python

### DETAILS ###
# Script: Convert the calibration object pose with respect to the base into x, y and z coordinates of the checkerboard points
# Author: Daniel Maeki (ipa-fxm-dm)
# Supervisor: Felix Messmer (ipa-fxm)

### IMPORTS ###
import roslib
import rospy

### POSSIBLE_IMPROVEMENTS ###
# ---

### TODO ###
# ---


### SCRIPT ###
class Convert_cal_obj_pose():
	
	def __init__(self, cal_obj_pose):
		self.cal_obj_pose = cal_obj_pose
	
	def pose_to_points(self):
		checkerboard_dims = rospy.get_param('checkerboard', None)
		# from center of checkerboard, get the upper left point and start point_list by first going to the right and then one down and going to the right again and so on..
		print checkerboard_dims
		checkerboard_points = self.cal_obj_pose
		return checkerboard_points
