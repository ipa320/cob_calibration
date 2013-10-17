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
NODE = 'joint_state_listener_arm'
import roslib; roslib.load_manifest(PKG)
import rospy
import sensor_msgs.msg

'''
Print joint states for cob arm.

Listens to /joints_states topic for joint state messages from arm controller
and prints them.
'''
def main():
    rospy.init_node(NODE)
    print "==> %s started " % NODE
    
    # get joint names for arm from parameter server
    joint_names = None
    try: joint_names = rospy.get_param("arm_controller/joint_names") # real hardware
    except KeyError: pass
    try: joint_names = rospy.get_param("arm_controller/joints")      # simulation
    except KeyError: pass
    if joint_names == None:
        print "Could not get joint names from parameter server. exiting..."
        exit(-1)
    print joint_names

    while not rospy.is_shutdown():
        if rospy.is_shutdown(): exit(0)
        
        # try getting /joint_states message
        try:
            msg = rospy.wait_for_message("/joint_states", sensor_msgs.msg.JointState)
        except rospy.exceptions.ROSInterruptException:
            exit(0)
        if joint_names[0] in msg.name:
            # message is from arm
            angles = []
            for name in joint_names:
                angles.append(msg.position[msg.name.index(name)])
            # nicely print joint angles with 8 digits
            print "[" + ", ".join(["%0.8f" % i for i in angles]) + "]"

if __name__ == '__main__':
    main()
