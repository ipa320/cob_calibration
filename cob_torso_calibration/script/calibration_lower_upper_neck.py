#!/usr/bin/env python
#################################################################
##\file
#
# \note
#   Copyright (c) 2011-2012 \n
#   Fraunhofer Institute for Manufacturing Engineering
#   and Automation (IPA) \n\n
#
#################################################################
#
# \note
#   Project name: care-o-bot
# \note
#   ROS stack name: cob_calibration
# \note
#   ROS package name: cob_torso_calibration
#
# \author
#   Author: Jannik Abbenseth, email:jannik.abbenseth@gmail.com
# \author
#   Supervised by: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
#
# \date Date of creation: September 2012
#
#################################################################
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     - Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer. \n
#     - Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution. \n
#     - Neither the name of the Fraunhofer Institute for Manufacturing
#       Engineering and Automation (IPA) nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission. \n
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License LGPL as
# published by the Free Software Foundation, either version 3 of the
# License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Lesser General Public License LGPL for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License LGPL along with this program.
# If not, see <http://www.gnu.org/licenses/>.
#
#################################################################
PKG= 'cob_torso_calibration'
NODE= 'torso_pitch_calibration_node'
import roslib; roslib.load_manifest(PKG)
import rospy

import math

import numpy as np
import cv2

from os import system

from sensor_msgs.msg import CameraInfo, Image
from cob_camera_calibration import Checkerboard, CheckerboardDetector, cv2util
from cv_bridge import CvBridge, CvBridgeError
from simple_script_server import simple_script_server

from torso_state_calculation import TorsoState
from update_cob_torso_calibration_urdf import UpdateCobTorsoCalibrationUrdf




def prettyprint(array):
	print '*'*20
	for row in array:
		print row

class TorsoCalibration():
	
	
	def __init__(self):
		'''
		Configures the calibration node
		Reads configuration from parameter server or uses default values
		'''
		rospy.init_node(NODE)
		print "==> started " + NODE
		
		# get parameter from parameter server or set defaults
		self.pattern_size  = rospy.get_param('~pattern_size',       "9x6")
		self.square_size   = rospy.get_param('~square_size',        0.03)
        
        
		self.alpha         = rospy.get_param('~alpha',              0.0)
		self.verbose       = rospy.get_param('~verbose',            True)
		
		self.save_result  = rospy.get_param('~save_result',        False)
        
		# split pattern_size string into tuple, e.g '9x6' -> tuple(9,6)
		self.pattern_size = tuple((int(self.pattern_size.split("x")[0]), int(self.pattern_size.split("x")[1])))
		
		self.camera_info = CameraInfo()
		self.camera_info_received=False
		self.latest_image=Image()
		self.bridge = CvBridge() 
		
		# set up Checkerboard and CheckerboardDetector
		self.board       = Checkerboard(self.pattern_size, self.square_size)
		self.detector    = CheckerboardDetector(self.board)
		
		self.sss=simple_script_server()
		self.ts=TorsoState()
		
	def __camera_info_callback__(self,data):
		'''
		executed on new message in /cam3d/rgb/camera_info
		stores received data
		'''
		self.camera_info = data
		self.camera_info_received=True
		
	def __image_raw_callback__(self,data):
		'''
		executed on new message in /cam3d/rgb/image_raw
		stores newest image
		'''
		self.latest_image=data
        
	def run(self):
		'''
		Runs the calibration
		'''
		print "==> starting torso pitch/height calibration"
        
        
		# subscribe to /cam3d/rgb/camera_info for camera_matrix and distortion coefficients
		rospy.Subscriber('/cam3d/rgb/camera_info',CameraInfo,self.__camera_info_callback__)
		# subscribe to /cam3d/rgb/image_raw for image data
		rospy.Subscriber('/cam3d/rgb/image_raw',Image,self.__image_raw_callback__)
		
		
		# wait until camera informations are recieved. 
		start_time=rospy.Time.now()
		while not (self.camera_info_received or rospy.is_shutdown()):
			rospy.sleep(0.05)
			if start_time + rospy.Duration(2.0) < rospy.Time.now():
				# print warning every 2 seconds if the message is stil missing
				print "--> still waiting for /cam3d/rgb/camera_info"
				start_time=rospy.Time.now()
		
		# convert camera matrix an distortion coefficients and store them
		camera_matrix=self.camera_info.K
		cm=np.asarray(camera_matrix)
		self.cm=np.reshape(cm,(3,3))
		dist_coeffs=self.camera_info.D
		self.dc=np.asarray(dist_coeffs)
		
		# initialize torso for movement
		self.sss.init("torso")
		
		# get initial state of Torso (Upper Tilt Joint and Head Axis Joint horizontal)
		state=list(self.ts.calc_references())
		print state
		
		# start minimization for getting the upright position of the torso
		start_time=rospy.Time.now()
		self.minimize_yvalue(state)
		end_time=rospy.Time.now()
		if self.verbose:
			print 'Elapsed time: ',(rospy.Time.now()-start_time).to_sec()
		if self.save_result:
			system('roslaunch cob_torso_calibration update_torso_calibration_urdf.launch')
			

		#
	def get_position(self): 
		'''
		returns the Y value in subpixel accuracy for the center of the recognized checkerboard
		'''
		cvImage = self.bridge.imgmsg_to_cv(self.latest_image, "mono8")
		image_raw = cv2util.cvmat2np(cvImage)
		image_processed=cv2.undistort(image_raw,self.cm,self.dc)
		
		#points=self.detector.detect_image_points(image_processed,True)
		(image_points,rmat,tvec)= self.detector.calculate_object_pose_ransac(image_raw,self.cm,self.dc,True)
		#print tvec[1]
		

		return tvec[0][0],tvec[1][0],tvec[2][0]
		
	
		
	def get_average_position(self,nsec,frequency=15):
		'''
		returns the average Y value for the last nsec seconds
		
		@param nsec: time for samples to be taken
		@type  nsec: integer
		
		@param frequency: update frequency for image topic
		@type  frequency: integer
		'''
		R=rospy.Rate(frequency)
		position_list=[]
		for i in range(frequency*nsec):
			position_list.append(list(self.get_position()))
			R.sleep()
			
		#if self.verbose:
			#prettyprint(position_list)
			#print np.average(position_list,0)
		return np.average(position_list,0)
		
	
			
	def minimize_yvalue(self,initial_state,eps=0.0005):
		'''
		Executive function. Calculates and applies forward rotation.
		Moves torso until the upright position is found
		
		@param initial_state: initial joint state of torso (usually "Home" position) with leveled head joint
		@type  initial_state: list [joint state lower neck, joint state tilt, joint state upper neck]

				
		@param eps: minimum accuracy for upright position
		@type  eps: float
		
		'''		
		initial_state=np.asarray(initial_state)
		states=[]
		step=0.1
		step_factor=[1,-1]	
		state=initial_state		
		self.sss.move("torso",[state.tolist()])
		rospy.sleep(3)
		
		# calculate rotational reference and rotate to center
		
		
		states.append([self.get_average_position(10)[1],state])
		while step>eps:
		
			
			for factor in step_factor:
				state=initial_state+np.asarray([factor*step,0,factor*-step])		
				self.sss.move("torso",[state.tolist()])
				rospy.sleep(2.5)
				states.append([self.get_average_position(10)[1],state])
			if self.verbose:	
				prettyprint(states)
				states=sorted(states)
				prettyprint(states)
			step/=2
			states=[states[0]]			
			initial_state=states[0][1]	#initial state for next iteration
			
			if self.verbose:
				print '*'*20
				print(states)
				print initial_state
		self.sss.move("torso",[state.tolist()])
		rospy.sleep(2)
		position=self.get_average_position(10)
		rotational_error=math.atan(position[0]/position[2])
		state-=np.asarray([0,rotational_error,0])
		self.sss.move("torso",[state.tolist()])
		return states
		
			
if __name__=='__main__':
	node=TorsoCalibration()
	node.run();
	print "==> done, exiting"
