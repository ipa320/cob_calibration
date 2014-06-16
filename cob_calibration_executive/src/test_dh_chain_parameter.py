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
PKG = 'cob_calibration_executive'
NODE = 'test_dh_parameters'
import roslib
roslib.load_manifest(PKG)
import rospy

from simple_script_server import simple_script_server
import yaml
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState
import tf
import tf.transformations as tft
from cob_robot_calibration_est.dh_chain import DhChain
import numpy as np

class DhHardwareTest:
    def __init__(self):

        self.arm_js =JointState()
        self.torso_js = JointState()
        self.arm_received = False
        self.torso_received = False
        rospy.Subscriber("arm_controller/state", JointTrajectoryControllerState, self.callback_arm)
        rospy.Subscriber("torso_controller/state", JointTrajectoryControllerState, self.callback_torso)
        print "::::::::::initialized::::::::::"
    def callback_arm(self, data):
        self.arm_js = data.actual
        self.arm_received = True
        #print "arm_received"
    def callback_torso(self, data):
        self.torso_js = data.actual
        self.torso_received = True
        #print "torso_received"

    def capture_loop(self, positions, system, sss):
        '''
        Moves arm to all positions using script server instance sss
        and compares fk transformation to computed model.
        '''


        arm_chain = DhChain(system["dh_chains"]["arm_chain"])
        torso_chain = DhChain(system["dh_chains"]["torso_chain"])

        listener = tf.TransformListener();
        rospy.sleep(2)


        for index in range(len(positions)):
            if positions[index]['torso_position'] == [0, 0, 0]:
                continue
            print "--> moving arm to sample #%s" % index
            pos = positions[index]
            joint_pos = [[a for a in positions[index]['joint_position']]]
            #print pos
            nh = sss.move("arm", joint_pos)
            while nh.get_state() == 0:
                rospy.sleep(0.2)
            #if nh.get_state() != 3:
                #sss.move("torso", "home")
                #nh = sss.move("arm", joint_pos)
                #rospy.sleep(2)
                #if nh.get_state() != 3:
                    #continue

            sss.move("torso", [positions[index]['torso_position']])


            # compare torso state
            while not rospy.is_shutdown():
                try:
                    (trans_torso,rot_torso) = listener.lookupTransform('/torso_base_link', '/torso_upper_neck_tilt_link', rospy.Time(0))
                    (trans_arm,rot_arm) = listener.lookupTransform('/arm_0_link', '/arm_7_link', rospy.Time(0))
                    break
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                    print e
                    print "waiting for transform"
                    rospy.sleep(0.1)
                    continue
            print self.arm_received
            print self.torso_received

            rot_torso = tft.euler_from_quaternion(rot_torso)
            tf_state_torso = tft.compose_matrix(angles = rot_torso, translate = trans_torso);
            dh_state_torso = torso_chain.fk(self.torso_js);

            tf_state_torso = np.matrix(tf_state_torso)
            dh_state_torso = np.matrix(dh_state_torso)


            print ":::::::::::::::::::::::::::::"
            print tf_state_torso
            print dh_state_torso

            assert np.allclose(tf_state_torso, dh_state_torso, atol = 10**-3)
            print "Success"

            # compare arm transformation

            rot_arm = tft.euler_from_quaternion(rot_arm)
            tf_state_arm = tft.compose_matrix(angles = rot_arm, translate = trans_arm);
            dh_state_arm = arm_chain.fk(self.arm_js);

            tf_state_arm = np.matrix(tf_state_arm)
            dh_state_arm = np.matrix(dh_state_arm)
            print tf_state_arm
            print dh_state_arm
            assert np.allclose(tf_state_arm, dh_state_arm, atol = 10**-3)
            print "arm: Success"
            print


def main():
    rospy.init_node(NODE)
    print "==> %s started " % NODE


    # init
    print "--> initializing sss"
    sss = simple_script_server()
    print "--> setup care-o-bot for capture"


    # get position from parameter server
    position_path = rospy.get_param('position_path', None)
    if position_path is None:
        print "[ERROR]: no path for positions set"
        return
    with open(position_path, 'r') as f:
        positions = yaml.load(f)

    system_path = rospy.get_param('system_path', None)
    if system_path is None:
        print "[ERROR]: no path for system configuration set"
        return
    with open(system_path, 'r') as f:
        system = yaml.load(f);
    print "==> capturing samples"
    start = rospy.Time.now()
    t = DhHardwareTest()
    t.capture_loop(positions, system, sss)
    print "finished after %s seconds" % (rospy.Time.now() - start).to_sec()

if __name__ == '__main__':
    main()
    print "==> done exiting"
