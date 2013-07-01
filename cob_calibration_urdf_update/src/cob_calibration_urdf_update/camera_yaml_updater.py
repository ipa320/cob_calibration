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

import yaml

class CameraYamlUpdater():
    '''
    @summary: Parses a camera calibration yaml file and provides a method to update it
    '''
    
    def __init__(self, yaml_in, yaml_out, debug=False):
        '''
        Init object with paths to input and output yaml
        '''
        self.file_yaml_in =  yaml_in
        self.file_yaml_out = yaml_out
        self.debug = debug
    
    def update_baseline(self, baseline_offset_x):
        '''
        Opens yaml file (self.file_yaml_in) and adjusts stereo baseline in x direction 
        by baseline_offset and saves result to output yaml file (self.file_yaml_out)
        
        @param baseline_offset: baseline offset
        @type  baseline_offset: float
        '''
        # open yaml
        print "--> loading camera yaml file from '%s'" % self.file_yaml_in
        camera_calibration = yaml.load(file(self.file_yaml_in))
        
        # adjust baseline
        camera_calibration['projection_matrix']['data'][3] += baseline_offset_x
        
        # save yaml
        print "--> saving results to camera yaml file '%s'" % self.file_yaml_out
        yaml.dump(camera_calibration, open(self.file_yaml_out, "w")) 
