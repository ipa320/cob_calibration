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

import numpy as np
import cv2

from sensor_msgs.msg import CameraInfo, Image
from cob_camera_calibration import Checkerboard, CheckerboardDetector, cv2util
from cv_bridge import CvBridge, CvBridgeError
from simple_script_server import simple_script_server

def prettyprint(array):
	print '*'*20
	for row in array:
		print row

class TorsoPitchCalibration():
	
	
	def __init__(self):
		'''
		Configures the calibration node
		Reads configuration from parameter server or uses default values
		'''
		rospy.init_node(NODE)
		print "==> started " + NODE
		
		# get parameter from parameter server or set defaults
		self.folder        = rospy.get_param('~folder',             ".")
		self.pattern_size  = rospy.get_param('~pattern_size',       "9x6")
		self.square_size   = rospy.get_param('~square_size',        0.03)
        
		self.image_prefix  = rospy.get_param('~image_prefix',       "camera")
		self.camera_name   = rospy.get_param('~camera_name',        "camera")
		self.frame_id      = rospy.get_param('~frame_id',           "/camera")
		self.output_file   = rospy.get_param('~output_file',        self.camera_name+".yaml")
        
		self.alpha         = rospy.get_param('~alpha',              0.0)
		self.verbose       = rospy.get_param('~verbose',            True)
        
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
		
		# start minimization for getting the upright position of the torso
		self.minimize_yvalue_der(0)

		#
	def get_height(self): 
		'''
		returns the Y value in subpixel accuracy for the center of the recognized checkerboard
		'''
		cvImage = self.bridge.imgmsg_to_cv(self.latest_image, "mono8")
		image_raw = cv2util.cvmat2np(cvImage)
		image_processed=cv2.undistort(image_raw,self.cm,self.dc)
		
		points=self.detector.detect_image_points(image_processed,True)
		(image_points,rmat,tvec)= self.detector.calculate_object_pose(image_raw,self.cm,self.dc,True)
		#print tvec[1]
		tvec=tvec.tolist()
		return tvec[1]
		
		points=points.reshape((54,2))
		points_mean=points.mean(0)
		#if self.verbose:
			#print "Y value: ", points_mean[1]
		return points_mean[1]
		
	def get_average_height(self,nsec,frequency=15):
		'''
		returns the average Y value for the last nsec seconds
		
		@param nsec: time for samples to be taken
		@type  nsec: integer
		
		@param frequency: update frequency for image topic
		@type  frequency: integer
		'''
		R=rospy.Rate(frequency)
		height_list=[]
		for i in range(frequency*nsec):
			height_list.append(self.get_height())
		return np.average(height_list)
		
	def minimize_yvalue(self,delta_angle,eps=0.0005,initial_state=[0,0,0]):
		'''
		Executive function. Moves torso until the upright position is found
		
		@param delta_angle: tilt angle for vertical position of head joint
		@type  delta_angle: float
		
		@param eps: minimum accuracy for upright position
		@type  eps: float
		
		@param initial_state: initial joint state of torso (usually "Home" position)
		@type  initial_state: list [joint state lower neck, joint state tilt, joint state upper neck]
		'''		
		startTime=rospy.Time.now()
		angular_error=np.asarray([0,0,delta_angle])
		initial_state=np.asarray(initial_state)
		initial_state-=angular_error
		step=0.1
		self.sss.move("torso",[initial_state.tolist()])
		initial_height=self.get_average_height(1.0/15)
		home_height=initial_height
		stepcounter=0
		while np.abs(step)>eps:
			
			
			new_state=initial_state-np.asarray([step,0,-step])
			self.sss.move("torso",[new_state.tolist()])
			new_height=self.get_average_height(1.0/15)
			if self.verbose:
				rospy.logdebug("*****new iteration step*****")
				rospy.logdebug("step = %s",step)
				rospy.logdebug("initial_height = %s",initial_height)
				rospy.logdebug("new_height     = %s",new_height)
				print("*****new iteration step*****")
				print("step = %s",step)
				print("initial_height = %s",initial_height)
				print("new_height     = %s",new_height)
			if initial_height<new_height:
				step/=-2
			initial_height=new_height
			initial_state=new_state
			if self.verbose:
				rospy.logdebug("new_step = %s", step)
			stepcounter+=1
		if self.verbose:
			print "new height:  ", new_height
			print "home_height: ", home_height
			print "Solution found at ",new_state," after ", stepcounter," steps."
			print "elapsed time: ",(rospy.Time.now()-startTime).to_sec(), " seconds." 
			
			
	def minimize_yvalue_der(self,delta_angle,eps=0.0005,initial_state=[0,0,0]):
		'''
		Executive function. Moves torso until the upright position is found
		
		@param delta_angle: tilt angle for vertical position of head joint
		@type  delta_angle: float
		
		@param eps: minimum accuracy for upright position
		@type  eps: float
		
		@param initial_state: initial joint state of torso (usually "Home" position)
		@type  initial_state: list [joint state lower neck, joint state tilt, joint state upper neck]
		'''		
		startTime=rospy.Time.now()
		angular_error=np.asarray([0,0,delta_angle])
		initial_state=np.asarray(initial_state)
		initial_state-=angular_error
		states=[]
		step=0.1
		step_factor=[1,-1]
		state=initial_state		
		self.sss.move("torso",[state.tolist()])
		states.append([self.get_average_height(10),state])
		while step>eps:
		
			
			for factor in step_factor:
				state=initial_state+np.asarray([factor*step,0,factor*-step])		
				self.sss.move("torso",[state.tolist()])
				rospy.sleep(1.5)
				states.append([self.get_average_height(10),state])
				
			prettyprint(states)
			states=sorted(states)
			prettyprint(states)
			step/=2
			states=[states[0]]
			initial_state=states[0][1]
			
			print '*'*20
			print(states)
			print initial_state
		
		'''
		home_height=initial_height
		stepcounter=0
		while np.abs(step)>eps:
			
			
			new_state=initial_state-np.asarray([step,0,-step])
			self.sss.move("torso",[new_state.tolist()])
			new_height=self.get_average_height(1.0/15)
			if self.verbose:
				rospy.logdebug("*****new iteration step*****")
				rospy.logdebug("step = %s",step)
				rospy.logdebug("initial_height = %s",initial_height)
				rospy.logdebug("new_height     = %s",new_height)
				print("*****new iteration step*****")
				print("step = %s",step)
				print("initial_height = %s",initial_height)
				print("new_height     = %s",new_height)
			if initial_height<new_height:
				step/=-2
			initial_height=new_height
			initial_state=new_state
			if self.verbose:
				rospy.logdebug("new_step = %s", step)
			stepcounter+=1
		if self.verbose:
			print "new height:  ", new_height
			print "home_height: ", home_height
			print "Solution found at ",new_state," after ", stepcounter," steps."
			print "elapsed time: ",(rospy.Time.now()-startTime).to_sec(), " seconds." 
			'''	
			
if __name__=='__main__':
	node=TorsoPitchCalibration()
	node.run();
	print "==> done, exiting"
