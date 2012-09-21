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
#   ROS package name: cob_camera_calibration
#
# \author
#   Author: Sebastian Haug, email:sebhaug@gmail.com
# \author
#   Supervised by: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
#
# \date Date of creation: November 2011
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

import yaml
import numpy as np
import rospy

class CalibrationData:
    '''
    Calibration data class stores intrinsic camera parameters 
    and some additional parameters (name and frame id)
    
    Provides methods to save and read parameters from yaml file.
    '''
    
    def __init__(self, camera_name, frame_id, image_width, image_height):
        '''
        Initialize object
        '''
        self.camera_name    = camera_name
        self.frame_id       = frame_id
        self.image_width    = image_width
        self.image_height   = image_height
        
        # initialize matrices
        self.camera_matrix              = np.identity(3)
        self.rectification_matrix       = np.identity(3)
        self.projection_matrix          = np.hstack((np.identity(3), np.zeros((3,1))))
        self.distortion_model           = "plumb_bob"
        self.distortion_coefficients    = np.zeros((1,5))
 
#        # DEBUG       
#        print self.camera_matrix              
#        print self.rectification_matrix       
#        print self.projection_matrix          
#        print self.distortion_model           
#        print self.distortion_coefficients    

    
    def read_camera_yaml_file(self, filename):
        '''
        Load camera calibration info from the yaml file located at 'filename'.
        '''
        calib = yaml.load(file(filename))
        
        # checks
        assert calib['camera_matrix']['rows'] == 3
        assert calib['camera_matrix']['cols'] == 3
        assert calib['rectification_matrix']['rows'] == 3
        assert calib['rectification_matrix']['cols'] == 3
        assert calib['projection_matrix']['rows'] == 3
        assert calib['projection_matrix']['cols'] == 4

        assert calib['distortion_coefficients']['rows'] == 1
        assert calib['distortion_coefficients']['cols'] == 5
        
      
        
        # import
        self.camera_name             = calib['camera_name']
        
        self.camera_matrix           = np.matrix(calib['camera_matrix']['data']).reshape((3,3))
        self.rectification_matrix    = np.matrix(calib['rectification_matrix']['data']).reshape((3,3))
        self.projection_matrix       = np.matrix(calib['projection_matrix']['data']).reshape((3,4))
        
        self.distortion_coefficients = np.matrix(calib['distortion_coefficients']['data']).reshape((1,5))
        self.image_width             = calib['image_width']
        self.image_height            = calib['image_height']   
        
        
        try:
            self.frame_id                = calib['frame_id']  
            self.distortion_model        = calib['distortion_model']   
        except:
            pass
        pass
    
    def save_camera_yaml_file(self, filename):
        '''
        Save camera calibration info to the yaml file located at 'filename'.
        '''
        open(filename, "w").writelines(self._as_yaml_string())
    
    def _as_yaml_string(self):
        '''
        Convert calibration data to yaml string.
        '''
        return yaml_template % (self.camera_name, 
                                self.frame_id, 
                                self.image_width, 
                                self.image_height, 
                                np.array(self.camera_matrix).flatten().tolist(), 
                                np.array(self.distortion_coefficients).flatten().tolist(), 
                                np.array(self.rectification_matrix).flatten().tolist(), 
                                np.array(self.projection_matrix).flatten().tolist())

yaml_template='''camera_name: %s
frame_id: %s
image_width: %d
image_height: %d
camera_matrix:
  rows: 3
  cols: 3
  data: %s
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: %s
rectification_matrix:
  rows: 3
  cols: 3
  data: %s
projection_matrix:
  rows: 3
  cols: 4
  data: %s
'''
