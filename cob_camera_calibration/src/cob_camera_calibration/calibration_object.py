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

import numpy as np

class CalibrationObject:
    '''
    Base Class for calibration objects
    
    A calibration object is defined by its pattern_size and square_size.
    '''
    
    def __init__(self, (rows, cols), square_size):
        self.pattern_size = (rows, cols)
        self.square_size = square_size

    def get_pattern_points(self):
        raise NotImplementedError()
    
class Checkerboard(CalibrationObject):
    '''
    Checkerboard calibration object
    
    Pattern size is number of inner checkerboard corners in horizontal and
    vertical direction. Square size is the size of a square (im m)
    '''
    def get_pattern_points(self):
        '''
        Returns the location of checkerboard corners in object centered
        coordinate system. z coordinate is always 0.
        Origin is in the lower left corner
        '''
        # implementation adapted from opencv examples
        pattern_points = np.zeros((np.prod(self.pattern_size), 3), np.float32)
        pattern_points[:,:2] = np.indices(self.pattern_size).T.reshape(-1, 2)
        pattern_points *= self.square_size
        return pattern_points
        
    def get_pattern_points_center(self):
        '''
        Returns the location of checkerboard corners in object centered
        coordinate system. z coordinate is always 0.
        origin is in the center of the chessboard
        '''
        # implementation adapted from opencv examples
        pattern_points = np.zeros((np.prod(self.pattern_size), 3), np.float32)
        pattern_points[:,:2] = np.indices(self.pattern_size).T.reshape(-1, 2)
        pattern_points[:,:1] = pattern_points[:,:1]-(max(pattern_points[:,:1])/2)
        pattern_points[:,1] = pattern_points[:,1]-(max(pattern_points[:,1])/2)
        
        
        pattern_points *= self.square_size
        
        return pattern_points

class SymmerticCirclegrid(CalibrationObject):
    '''
    Symmetric Circlegrid calibration object (as defined by opencv)
    '''
    def get_pattern_points(self):
        '''
        Returns the location of circle centers in object centered
        coordinate system. z coordinate is always 0.
        '''
        # implementation adapted from opencv examples
        pattern_points = np.zeros((np.prod(self.pattern_size), 3), np.float32)
        pattern_points[:,:2] = np.indices(self.pattern_size).T.reshape(-1, 2)
        pattern_points *= self.square_size
        return pattern_points
