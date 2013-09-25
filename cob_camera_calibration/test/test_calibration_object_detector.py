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
import yaml

from cob_camera_calibration import CheckerboardDetector, Checkerboard

CHECKERBOARD_IMAGE="./data/cb1.jpg"
CHECKERBOARD_IMAGE_EXPECTED_POINTS_YAML = "./data/cb1_points.yaml"

class TestCalibrationObjectDetector(unittest.TestCase):
    def test_detect_image_points_gray_image(self):
        # create Checkerboard instance and read image
        c = Checkerboard((9, 6), 0.03)
        image = cv2.imread(CHECKERBOARD_IMAGE, 1)

        # create CheckerboardDetector instance and detect image points
        cd = CheckerboardDetector(c)
        points = cd.detect_image_points(image, False)

        #dump = {'points': points.flatten().tolist()}
        #yaml.dump(dump, open(CHECKERBOARD_IMAGE_EXPECTED_POINTS_YAML, "w"))

        # load expected points
        points_expected = yaml.load(file(CHECKERBOARD_IMAGE_EXPECTED_POINTS_YAML))['points']
        self.assertTrue(np.allclose(points.flatten().tolist(), points_expected))

    def test_detect_image_points_color_image(self):
        # create Checkerboard instance and read image
        c = Checkerboard((9, 6), 0.03)
        image = cv2.imread(CHECKERBOARD_IMAGE, 0)

        # create CheckerboardDetector instance and detect image points
        cd = CheckerboardDetector(c)
        points = cd.detect_image_points(image, True)

        #dump = {'points': points.flatten().tolist()}
        #yaml.dump(dump, open(CHECKERBOARD_IMAGE_EXPECTED_POINTS_YAML, "w"))

        # load expected points
        points_expected = yaml.load(file(CHECKERBOARD_IMAGE_EXPECTED_POINTS_YAML))['points']
        self.assertTrue(np.allclose(points.flatten().tolist(), points_expected))

    def test_calculate_object_pose(self):
        # create Checkerboard instance and read image
        c = Checkerboard((9, 6), 0.03)
        image = cv2.imread(CHECKERBOARD_IMAGE, 0)

        # create CheckerboardDetector instance and detect object pose
        # (only dummy matrices, does not correspond to real camera used)
        cd = CheckerboardDetector(c)
        camera_matrix   = np.matrix([1200, 0, 600, 0, 1200, 600, 0, 0, 1], dtype=np.float32).reshape((3,3))
        dist_coeffs     = np.matrix([0, 0, 0, 0, 0], dtype=np.float32).reshape((1,5))
        (rvec, tvec) = cd.calculate_object_pose(image, camera_matrix, dist_coeffs, True)

        rvec_expected = np.matrix([[0.99905897,  0.035207,   -0.02533079],
                                   [-0.0208742,   0.90224111,  0.43072642],
                                   [ 0.03801906, -0.42979234,  0.90212699]])
        tvec_expected = np.matrix([[-0.03181351], [-0.13593217], [ 0.7021291]])

        self.assertTrue(np.allclose(rvec_expected, rvec))
        self.assertTrue(np.allclose(tvec_expected, tvec))

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'TestCalibrationObjectDetector', TestCalibrationObjectDetector)
