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
#   ROS package name: cob_calibration_executive
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

import numpy as np
from cob_camera_calibration import CalibrationData
class ViewfieldChecker():
    '''
    functions to check if one point (translation from camera frame) 
    is visible by the specified cameras
    '''

    def __init__(self,cameras,path):
        '''
        @param cameras: selected cameras for check
        @type  cameras: list of strings
        
        @param path: path to calibrationfiles for selected cameras
        @type  path: dict with key==cameras and value==string to calibrationfile
        '''
    
        self.cameras=cameras
        self.calibration={}
        for camera in cameras:

            self.calibration[camera] =CalibrationData(camera,0,0,0)
            self.calibration[camera].read_camera_yaml_file(path[camera])
            
    
        self.inverter=np.matrix([   [1,1,-1,1],
                                    [1,1,-1,1],
                                    [1,1,1,1]])
        print self.calibration
    
    def invert_h0_coordinates(self,projection_matrix):
        '''
        inverts the translation parameter of the projection matrix
        
        @param projection_matrix: projection matrix of camera
        @type  projection_matrix: numpy matrix (3x4)
        
        '''
        return np.multiply(projection_matrix,self.inverter)
        
    def cb_in_modeled_viewfields(self,translations):
        '''
        computes if the specified point is visible in all cameramodels
        
        @param translations: point
        @type  translations: dict with key==cameras and value==translation tuple
        '''
        visible=[]
        for camera in self.cameras:
            visible.append(self.in_viewfield(camera,translations[camera]))
        print visible
        return all(visible)
        
    def in_viewfield(self,camera,translation):
        '''
        computes if the specified point is visible in the selected 
        cameramodel. 
        
        Calculating the x and y coordinate of projection. 
        If values are >=0 point is neither to far on the left nor below
        
        Then alculating the x and y coordinate for a projection 
        to the upper right corner. 
        If values are >=0 point is neither to far on the right nor up
        
        @param camera: selected camera
        @type  camera: string
        
        @param translation: point
        @type  translation: tuple

        '''
        print translation
        t=[translation[2],translation[1],translation[0],1]
        p1=self.calibration[camera].projection_matrix*np.matrix(t).T
        
        criteria=[]
        criteria.append( p1[0]/p1[2]>=0)
        criteria.append( p1[1]/p1[2]>=0)
        
        p2=self.invert_h0_coordinates(self.calibration[camera].projection_matrix)*np.matrix(t).T

        
        criteria.append( p2[0]/p2[2]<=0)
        criteria.append( p2[1]/p2[2]<=0)
        return all(criteria)
