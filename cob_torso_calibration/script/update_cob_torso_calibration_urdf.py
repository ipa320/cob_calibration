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
NODE = 'update_cob_torso_calibration_urdf'
import roslib; 
roslib.load_manifest(PKG)
import rospy
import tf
import numpy



import sys
import yaml
from math import pi
from xml.dom import minidom, Node


from cob_robot_calibration import calibration_urdf_updater
from torso_state_calculation import TorsoState


 # Default values for files and debug output
DEFAULT_CALIB_URDF_XACRO_IN  = "/tmp/cal/calibration.urdf.xacro"
DEFAULT_CALIB_URDF_XACRO_OUT = "/tmp/cal/calibration.urdf.xacro_updated" 
ENABLE_DEBUG_OUTPUT = True

class UpdateCobTorsoCalibrationUrdf():
	'''
	@summary: Updates calibration urdf file with calibration results
    
	The results of the cob calibration process are read from the calibrated_system yaml
	file and propagated to the urdf robot calibration file.
	'''

	
	

	
    
	def __init__(self):
		'''
		Get file locations from parameter server (or use defaults) and setup dictionary
		which specifies which values are updated. 
		'''
		rospy.init_node(NODE)
		print "==> started " + NODE
        
        # get file names from parameter server
		self.file_urdf_in =             rospy.get_param('~urdf_in',        DEFAULT_CALIB_URDF_XACRO_IN)
		self.file_urdf_out =            rospy.get_param('~urdf_out',       DEFAULT_CALIB_URDF_XACRO_OUT)
		self.debug =                    rospy.get_param('~debug',          ENABLE_DEBUG_OUTPUT)
		
		self.ts=TorsoState()
	

	
		
	def run(self):	
		'''
		Start the update process. Values are calculated from torso joint state. Therefor the actual torso jointstate copied to the output file
		'''
		
		attributes2update = {}
		X,Y,Z=self.ts.calc_references(self.debug)
		attributes2update["torso_lower_neck_tilt_ref"]= X
		attributes2update["torso_pan_ref"]= Y
		attributes2update["torso_upper_neck_tilt_ref"]= Z
		if self.debug:
			print attributes2update
		# update calibration xml based on attributes2update dict
		urdf_updater = calibration_urdf_updater.CalibrationUrdfUpdater(self.file_urdf_in, self.file_urdf_out, self.debug)
		urdf_updater.update(attributes2update)

	
			
		
		

        
  

if __name__ == "__main__":
    # start main
    print 'Saving results'
    updateUrdf = UpdateCobTorsoCalibrationUrdf()
    updateUrdf.run()
    
    # shutdown
    rospy.signal_shutdown(rospy.Time.now())
    print "==> done! exiting..."
    
