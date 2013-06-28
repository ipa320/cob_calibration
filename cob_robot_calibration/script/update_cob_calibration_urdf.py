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
PKG  = 'cob_robot_calibration'
NODE = 'update_cob_calibration_urdf'
import roslib; roslib.load_manifest(PKG)
import rospy
import tf

import sys
import yaml
from math import pi
from xml.dom import minidom, Node

from cob_robot_calibration_est import single_transform
from cob_calibration_urdf_update import calibration_urdf_updater

 # Default values for files and debug output
DEFAULT_CALIB_URDF_XACRO_IN  = "/tmp/cal/calibration.urdf.xacro"
DEFAULT_CALIB_URDF_XACRO_OUT = "/tmp/cal/calibration.urdf.xacro_updated"
DEFAULT_YAML_CALIB_SYSTEM    = "/tmp/cal/result_step_3.yaml"
#DEFAULT_YAML_INITIAL_SYSTEM  = "/home/fmw-ja/ros_workspace/01_Calibration/cob_calibration/cob_robot_calibration/config/cob3-3/system.yaml"
ENABLE_DEBUG_OUTPUT = False

class UpdateCobCalibrationUrdf():
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
        self.file_urdf_default=		rospy.get_param('~urdf_default')
        self.file_urdf_in =             rospy.get_param('~urdf_in',        DEFAULT_CALIB_URDF_XACRO_IN)
        self.file_urdf_out =            rospy.get_param('~urdf_out',       DEFAULT_CALIB_URDF_XACRO_OUT)
        self.file_yaml_calib_system =   rospy.get_param('~calib_system',   DEFAULT_YAML_CALIB_SYSTEM)
        #self.file_yaml_init_system =    rospy.get_param('~initial_system', DEFAULT_YAML_INITIAL_SYSTEM)
        self.debug =                    rospy.get_param('~debug',          ENABLE_DEBUG_OUTPUT)

        # tfs2update stores the transform names [which need to be converted to (x, y, z, roll, pitch, yaw)
        # and updated in the urdf] and their corresponding property name prefixes as used in calibration.urdf.xarco
        self.tfs2update = {'arm_0_link':                        'offset_arm_',
                           'torso_base_link':                      'offset_torso_',
                           'cam_reference_link':          'offset_cam_l_'
                           #'head_color_camera_r_joint':          'cam_r_',
                            #'head_cam3d_link': 'offset_cam3d_'
        }

        # chains2process stores the dh chain names of chains which need to be updated and their corresponding
        # property names/segments as used in calibration.urdf.xarco
        self.chains2update = {'arm_chain':   ['offset_arm_1_ref',
                                              'offset_arm_2_ref',
                                              'offset_arm_3_ref',
                                              'offset_arm_4_ref',
                                              'offset_arm_5_ref',
                                              'offset_arm_6_ref',
                                              'offset_arm_7_ref'],
                              'torso_chain': ['offset_torso_lower_neck_tilt_cal_offset',
                                              'offset_torso_pan_cal_offset',
                                              'offset_torso_upper_neck_tilt_cal_offset']}

    def run(self):
        '''
        Start the update process. Values are read from yaml files and are preprocessed for
        writing to xml
        '''
        # load yaml files
        print "--> loading calibrated system from '%s'" % self.file_yaml_calib_system
        calib_system   = yaml.load(file(self.file_yaml_calib_system))
        #print "--> loading initial system from '%s'" % self.file_yaml_init_system
        #initial_system = yaml.load(file(self.file_yaml_init_system))

        attributes2update = {}

        # process transforms
        for tf_name in self.tfs2update.keys():
            prefix = self.tfs2update[tf_name]
            (x, y, z, roll, pitch, yaw) = self._convert_transform(calib_system['transforms'][tf_name])

            # add to attributes2update dict as "attribute name -> new_value" entries
            attributes2update[prefix+"x"]       = round(x, 10)
            attributes2update[prefix+"y"]       = round(y, 10)
            attributes2update[prefix+"z"]       = round(z, 10)
            attributes2update[prefix+"roll"]    = round(roll, 10)
            attributes2update[prefix+"pitch"]   = round(pitch, 10)
            attributes2update[prefix+"yaw"]     = round(yaw, 10)

        # process dh chains
        '''for chain in self.chains2update:
            segments = self.chains2update[chain]
            for id in range(len(segments)):
                # process segment with id
                segment = segments[id]

                # process segment of chain
                initial_value = initial_system["dh_chains"][chain]["dh"][id][0] # get initial theta value of segment for current chain
                calib_value   =   calib_system["dh_chains"][chain]["dh"][id][0] # get calibr. theta value of segment for current chain
                new_value = float(calib_value) - float(eval(str(initial_value)))
                new_value = round(new_value, 10)

                # add to attributes2update dict as "attribute name -> new_value" entries
                attributes2update[segment] = new_value
           '''
        # update calibration xml based on attributes2update dict
        urdf_updater = calibration_urdf_updater.CalibrationUrdfUpdater(self.file_urdf_in, self.file_urdf_out, self.debug,self.file_urdf_default)
        urdf_updater.update(attributes2update)

    def _convert_transform(self, t):
        '''
        Convert transform notation from (x, y, z, rotation_vector) as used in calibrated_system
        and initial_system to (x, y, z, roll, pitch, yaw) notation used in calibration.urdf.xacro


        @param t: (x, y, z, rotation_vector)
        @type  t: tuple

        @return: (x, y, z, roll, pitch, yaw)
        @rtype:  tuple
        '''
        matrix = single_transform.SingleTransform(t).transform # convert to 4x4 transformation matrix
        roll, pitch, yaw = tf.transformations.euler_from_matrix(matrix)
        x, y, z = matrix[0,3], matrix[1,3], matrix[2,3]
        return (x, y, z, roll, pitch, yaw)

if __name__ == "__main__":
    # start main
    updateUrdf = UpdateCobCalibrationUrdf()
    updateUrdf.run()

    # shutdown
    rospy.signal_shutdown(rospy.Time.now())
    print "==> done! exiting..."
