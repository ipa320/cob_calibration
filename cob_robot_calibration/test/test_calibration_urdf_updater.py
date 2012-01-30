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
#   Author: Sebastian Haug, email:sebhaug@gmail.com
# \author
#   Supervised by: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
#
# \date Date of creation: January 2012
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
PKG = 'cob_robot_calibration'
import roslib; roslib.load_manifest(PKG)
import unittest

from  cob_robot_calibration import calibration_urdf_updater 

FILE_IN = "test/data/test_calibration.urdf.xacro"
FILE_OUT1 = "/tmp/test_calibration.urdf.xacro_out1"
FILE_OUT2 = "/tmp/test_calibration.urdf.xacro_out2"
FILE_OUT_RES1 = "test/data/test_calibration.urdf.xacro_corr-result1"
FILE_OUT_RES2 = "test/data/test_calibration.urdf.xacro_corr-result2"

class TestCalibrationUrdfUpdater(unittest.TestCase):
    def test_update_one_param(self):
        # define params to update
        attributes2update = {'a': 1.0}
        
        # do update
        updater = calibration_urdf_updater.CalibrationUrdfUpdater(FILE_IN, FILE_OUT1)
        updater.update(attributes2update)
        
        # compare with correct results
        self.assertTrue(self._cmp_files(FILE_OUT1, FILE_OUT_RES1))
        
    def test_update_one_param_string(self):
        # define params to update
        attributes2update = {'a': "1.0"}
        
        # do update
        updater = calibration_urdf_updater.CalibrationUrdfUpdater(FILE_IN, FILE_OUT1)
        updater.update(attributes2update)
        
        # compare with correct results
        self.assertTrue(self._cmp_files(FILE_OUT1, FILE_OUT_RES1))
        
    def test_update_all_params(self):
        # define params to update
        attributes2update = {'a': 1.0}
        attributes2update = {'b': 2.0}
        attributes2update = {'c': 3.0}
        
        # do update
        updater = calibration_urdf_updater.CalibrationUrdfUpdater(FILE_IN, FILE_OUT2)
        updater.update(attributes2update)
        
        # compare with correct results
        self.assertTrue(self._cmp_files(FILE_OUT2, FILE_OUT_RES2))
        
    def test_update_all_params_string(self):
        # define params to update
        attributes2update = {'a': "1.0"}
        attributes2update = {'b': "2.0"}
        attributes2update = {'c': "3.0"}
        
        # do update
        updater = calibration_urdf_updater.CalibrationUrdfUpdater(FILE_IN, FILE_OUT2)
        updater.update(attributes2update)
        
        # compare with correct results
        self.assertTrue(self._cmp_files(FILE_OUT2, FILE_OUT_RES2))

    def _cmp_files(self, f1, f2):
        return open(f1).read() == open(f2).read()

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'TestCalibrationUrdfUpdater', TestCalibrationUrdfUpdater)
