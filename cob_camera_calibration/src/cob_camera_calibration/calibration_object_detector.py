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

import cv2

DEBUG_OUTPUT = True

class CalibrationObjectDetector:
    '''
    Base Class for calibration object detector
    
    A specific calibration object detector implementation can detect the specific 
    calibration object in images and calculate its 3D pose.
    '''
    
    def __init__(self, calibration_object):
        '''
        @param calibration_object: Calibration object to work with. Must be compatible to CalibrationObjectDetector
        @type  calibration_object: CalibrationObject
        '''
        self.calibration_object = calibration_object
        
    def detect_image_points(self, image, is_grayscale):
        raise NotImplementedError()
    
    def calculate_object_pose(self, image_raw, camera_matrix, dist_coeffs, is_grayscale):
        '''
        Calculate 3D pose of calibration object in image given the camera's
        camera matrix and distortion coefficients. 
        
        Returns rotation matrix and translation vector.
        
        @param image_raw: input image, not distortion corrected
        @type  image_raw: cv2 compatible numpy image
        
        @param camera_matrix: camera matrix of camera
        @type  camera_matrix: numpy matrix
        
        @param dist_coeffs: distortion coefficients of camera
        @type  dist_coeffs: numpy matrix
        
        @param is_grayscale: set to true if image is grayscale
        @type  is_grayscale: bool
        '''
        # get image and object points
        image_points = self.detect_image_points(image_raw, is_grayscale)
        object_points = self.calibration_object.get_pattern_points()
        
        # get object pose in raw image (not yet distortion corrected)
        (retval, rvec, tvec) = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coeffs)
        
        # convert rvec to rotation matrix
        rmat = cv2.Rodrigues(rvec)[0]
        return (rmat, tvec)

class CheckerboardDetector(CalibrationObjectDetector):
    '''
    Detects a checkerboard calibration object
    '''
    def detect_image_points(self, image, is_grayscale):
        '''
        Detect the pixels at which the checkerboard's corners are. Returns
        list of (x, y) coordinates.
        
        @param image: input image with checkerboard
        @type  image: cv2 compatible numpy image
        
        @param is_grayscale: set to true if image is grayscale
        @type  is_grayscale: bool
        '''
        # detect checkerboard
        found, corners = cv2.findChessboardCorners(image, self.calibration_object.pattern_size)
        if found:
            # create gray image if image is not grayscale
            if not is_grayscale:
                gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            else:
                gray_image = image
            
            # refine checkerboard corners to subpixel accuracy
            term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.1)
            cv2.cornerSubPix(gray_image, corners, (5, 5), (-1, -1), term)
        else:
            # could not find checkerboard
            if DEBUG_OUTPUT:
                print 'checkerboard not found'
            return None
        return corners
