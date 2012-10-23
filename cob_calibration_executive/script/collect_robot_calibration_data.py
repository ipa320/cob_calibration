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
PKG  = 'cob_calibration_executive'
NODE = 'collect_robot_calibration_data_node'
import roslib; roslib.load_manifest(PKG)
import rospy

from simple_script_server import simple_script_server
from cob_calibration_srvs.srv import Capture

def capture_loop_arm(positions, sss, capture):
    '''
    Moves arm to all positions using script server instance sss 
    and calls capture() to capture samples
    '''
    for pos in positions:
        print "--> moving arm to pos %s sample" % pos
        sss.move("arm", pos)
        sss.sleep(1.5)
        capture()

def capture_loop_torso(positions, sss, capture):
    '''
    Moves torso to all positions using script server instance sss 
    and calls capture() to capture samples
    '''
    for pos in positions:
        print "--> moving torso to pos %s sample" % pos
        sss.move("torso", pos)
        sss.sleep(1.5)
        capture()

def main():
    rospy.init_node(NODE)
    print "==> %s started " % NODE
    
    # service client
    image_capture_service_name = "/collect_data/capture"
    capture = rospy.ServiceProxy(image_capture_service_name, Capture)
    rospy.wait_for_service(image_capture_service_name, 1)
    print "--> service client for capture images initialized"

    # init
    print "--> initializing sss"
    sss = simple_script_server()
    sss.init("base")
    sss.init("torso")
    sss.init("head")
    sss.recover("base")
    sss.recover("torso")
    sss.recover("head")
    
    print "--> setup care-o-bot for capture"
    sss.move("head", "back")

    # get position from parameter server
    positions_back = rospy.get_param("/script_server/arm/all_robot_back")
    positions_center = rospy.get_param("/script_server/arm/all_robot_center")
    positions_right = rospy.get_param("/script_server/arm/all_robot_right")
    positions_left = rospy.get_param("/script_server/arm/all_robot_left")
    torso_positions = ["calib_front_left2", "calib_front_right2", "calib_right2", "calib_left2", "calib_back_left2", "calib_back_right2", "home"]

    print "==> capturing images BACK"
    sss.move("torso", "back")
    capture_loop_arm(positions_back, sss, capture)
    
    print "==> capturing images LEFT"
    sss.move("torso", "left")
    capture_loop_arm(positions_left, sss, capture)
    
    print "==> capturing images RIGHT"
    sss.move("torso", "right")
    capture_loop_arm(positions_right, sss, capture)
    
    print "==> capturing images CENTER"
    sss.move("torso", "home")
    capture_loop_arm(positions_center, sss, capture)
    
    print "==> capturing TORSO samples"
    sss.move("arm", "calibration")
    capture_loop_torso(torso_positions, sss, capture)

if __name__ == '__main__':
    main()
    print "==> done exiting"
