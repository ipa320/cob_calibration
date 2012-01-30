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
PKG = 'cob_camera_calibration'
import roslib; roslib.load_manifest(PKG)
import unittest

import numpy as np
import cv2
from cob_camera_calibration import Checkerboard

class TestCalibrationObject(unittest.TestCase):
    def test_checkerboard_object_points_2x2(self):
        c = Checkerboard((2,2), 1)
        points = c.get_pattern_points()
        points_ = [[0, 0, 0], [1, 0, 0], [0, 1, 0], [1, 1, 0]]
        self.assertTrue(np.allclose(points, points_))
    
    def test_checkerboard_object_points_2x2_scale(self):
        c = Checkerboard((2,2), 1.5)
        points = c.get_pattern_points()
        points_ = [[0, 0, 0], [1.5, 0, 0], [0, 1.5, 0], [1.5, 1.5, 0]]
        self.assertTrue(np.allclose(points, points_)) 
        
    def test_checkerboard_object_points_4x4(self):
        c = Checkerboard((4,4), 1)
        points = c.get_pattern_points()
        points_ = [[0, 0, 0], [1, 0, 0], [2, 0, 0], [3, 0, 0],
                   [0, 1, 0], [1, 1, 0], [2, 1, 0], [3, 1, 0],
                   [0, 2, 0], [1, 2, 0], [2, 2, 0], [3, 2, 0],
                   [0, 3, 0], [1, 3, 0], [2, 3, 0], [3, 3, 0]]
        self.assertTrue(np.allclose(points, points_)) 
                
if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'TestCalibrationObject', TestCalibrationObject)
