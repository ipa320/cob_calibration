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
PKG = 'cob_torso_calibration'
NODE = 'torso_state_calculation'
import roslib
roslib.load_manifest(PKG)
import rospy
import numpy as np

from control_msgs.msg import JointTrajectoryControllerState


class TorsoState():
    '''
    @summary: Updates calibration urdf file with calibration results

    The results of the cob calibration process are read from the calibrated_system yaml
    file and propagated to the urdf robot calibration file.
    '''
    maxcounter = 100
    counter = maxcounter
    armcounter = maxcounter

    def __init__(self):
        '''
        Get file locations from parameter server (or use defaults) and setup dictionary
        which specifies which values are updated.
        '''
        self.torsoPositions = None
        rospy.Subscriber("/torso_controller/state",
                         JointTrajectoryControllerState, self.torso_callback)

        self.armPositions = None
        rospy.Subscriber("/arm_controller/state",
                         JointTrajectoryControllerState, self.arm_callback)

    def arm_callback(self, data):
        '''
        called when message in topic /torso_controller/state is received
        '''
        if self.armPositions is None:
            self.armPositions = np.array([list(data.actual.positions)])
            return

        if self.armcounter < self.maxcounter:

            self.armPositions = np.append(
                self.armPositions, [list(data.actual.positions)], axis=0)

            self.armcounter += 1

    def torso_callback(self, data):
        '''
        called when message in topic /torso_controller/state is received
        '''
        if self.torsoPositions is None:
            self.torsoPositions = np.array([list(data.actual.positions)])
            return

        if self.counter < self.maxcounter:
            self.torsoPositions = np.append(
                self.torsoPositions, [list(data.actual.positions)], axis=0)

            self.counter += 1

    def calc_references(self, debug=False):
        '''
        Wait for enough samples and calculate mean
        '''
        self.counter = 0
        self.armcounter = 0
        while self.counter < self.maxcounter:#or self.armcounter < self.maxcounter:
            rospy.sleep(0.1)
            print "waiting for samples", self.armcounter, self.counter

        #if debug:
            #print "Std Deviation for lower_neck_tilt is ", numpy.std(self.tlntr)
            #print "Std Deviation for pan is ", numpy.std(self.tpr)
            #print "Std Deviation for upper_neck_tilt is ", numpy.std(self.tuntr)

        try:
            arm=np.average(self.armPositions,0)
        except:
            arm=None
        try:
            torso=np.average(self.torsoPositions,0)
        except:
            torso=None
        print torso, arm

        return torso, arm
