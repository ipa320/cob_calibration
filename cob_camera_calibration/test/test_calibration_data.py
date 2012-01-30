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

from cob_camera_calibration import CalibrationData

TEST_FILE = "test/data/calibration_data.yaml"
TEST_OUT = "/tmp/calibration_data.yaml"

class TestCalibrationData(unittest.TestCase):
    def test_read(self):
        data = CalibrationData("camera", "frame", "image_width", "image_height")
        data.read_camera_yaml_file(TEST_FILE)
        
    def test_read_write(self):
        data = CalibrationData("camera", "frame", "image_width", "image_height")
        data.read_camera_yaml_file(TEST_FILE)
        data.save_camera_yaml_file(TEST_OUT)
        
        # compare with correct results
        self.assertTrue(self._cmp_files(TEST_FILE, TEST_OUT))
        
    def _cmp_files(self, f1, f2):
        return open(f1).read() == open(f2).read()
                
if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'TestCalibrationData', TestCalibrationData)
