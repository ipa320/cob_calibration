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
#   ROS package name: cob_robot_calibration
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
PKG  = 'cob_torso_calibration'
NODE = 'torso_state_calculation'
import roslib; roslib.load_manifest(PKG)
import rospy
import tf
import numpy

import sys
from math import pi



from pr2_controllers_msgs.msg import JointTrajectoryControllerState


class TorsoState():
	'''
	@summary: Updates calibration urdf file with calibration results
    
	The results of the cob calibration process are read from the calibrated_system yaml
	file and propagated to the urdf robot calibration file.
	'''
	maxcounter=100
	counter=maxcounter
	
    
	def __init__(self):
		'''
		Get file locations from parameter server (or use defaults) and setup dictionary
		which specifies which values are updated. 
		'''
        
		self.tlntr=[] 	#Values for torso lower neck tilt
		self.tpr=[]		#Values for torso pan
		self.tuntr=[]	#Values for torso upper neck tilt
		
		rospy.Subscriber("/torso_controller/state",JointTrajectoryControllerState, self.torso_callback)
		
    
	
	
		

	def torso_callback(self,data):
		'''
		called when message in topic /torso_controller/state is received
		'''
				
		if self.counter<self.maxcounter:
		
			torso_positions=data.actual.positions
			self.tlntr.append(torso_positions[0])
			self.tpr.append(torso_positions[1])
			self.tuntr.append(torso_positions[2])
			
			
			self.counter+=1

	
	def calc_references(self,debug=False):
		'''
		Wait for enough samples and calculate mean
		'''
		self.counter=0
		self.torso_positions=[]
		while self.counter<self.maxcounter:
			rospy.sleep(0.1)
			
		if debug:
			print "Std Deviation for lower_neck_tilt is ",numpy.std(self.tlntr)
			print "Std Deviation for pan is " ,numpy.std(self.tpr)
			print "Std Deviation for upper_neck_tilt is ",numpy.std(self.tuntr)
		return numpy.average(self.tlntr), numpy.average(self.tpr), numpy.average(self.tuntr)

