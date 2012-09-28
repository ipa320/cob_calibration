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
NODE = 'arm_ik_node'
import roslib; roslib.load_manifest(PKG)
import rospy

import numpy as np
import yaml
from math import pi, sqrt

from simple_script_server import simple_script_server
from kinematics_msgs.srv import GetPositionIK, GetPositionIKRequest
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import tf

def getIk(arm_ik, (t, q), link, seed=None):
    '''
    query arm_ik server for joint_position which put arm_7_link to pose (t, q)
    
    @param arm_ik: arm_ik service proxy
    @param t: translation
    @param q: rotation as quaternion
    @param link: frame in which pose (t, q) is defined
    @param seed: initial joint positions for ik calculation, in None current joint pos of arm is used.
    
    @return: tuple of joint_positions or None if ik was not found
    '''
    # get joint names for arm from parameter server
    joint_names = None
    try: joint_names = rospy.get_param("arm_controller/joint_names") # real hardware
    except KeyError: pass
    try: joint_names = rospy.get_param("arm_controller/joints")      # simulation
    except KeyError: pass
    if joint_names == None:
        print "Could not get arm joint_names from parameter server."
        return None
        
    # if seed == None get current joint angles from arm as seed position
    if seed == None:
        seed = []
        for tries in range(20):
            try: msg = rospy.wait_for_message("/joint_states", JointState)
            except rospy.exceptions.ROSInterruptException: pass
            if joint_names[0] in msg.name:
                # message is from arm, save current angles
                for name in joint_names: seed.append(msg.position[msg.name.index(name)])
                break
        if seed == []:
            print "Could not get /joint_states message from arm controller. "
            return None
    assert len(seed) == len(joint_names)
    
    # create and send ik request
    req = GetPositionIKRequest()
    req.timeout = rospy.Duration(1.0)
    req.ik_request.ik_link_name = "arm_7_link"
    req.ik_request.ik_seed_state.joint_state.position = seed
    req.ik_request.ik_seed_state.joint_state.name = joint_names
    req.ik_request.pose_stamped.header.frame_id = link
    req.ik_request.pose_stamped.pose.position.x = t[0]
    req.ik_request.pose_stamped.pose.position.y = t[1]
    req.ik_request.pose_stamped.pose.position.z = t[2]
    req.ik_request.pose_stamped.pose.orientation.x = q[0]
    req.ik_request.pose_stamped.pose.orientation.y = q[1]
    req.ik_request.pose_stamped.pose.orientation.z = q[2]
    req.ik_request.pose_stamped.pose.orientation.w = q[3]
    
    # try to get inverse kinecmatics for at least 3 times
    for i in range(3):
        resp = arm_ik(req)
        if resp.error_code.val == resp.error_code.SUCCESS:
            break
    
    # report sucess or return None on error
    if resp.error_code.val == resp.error_code.SUCCESS:
        result = list(resp.solution.joint_state.position)
        return result
    else:
        print "Inverse kinematics request failed with error code", resp.error_code.val, ", seed was", seed
        return None

def tadd(t1, t2):
    '''
    Shortcut function to add two translations t1 and t2
    '''
    return map(lambda (t1x, t2x): t1x+t2x, zip(t1, t2))

def qmult(q1, q2):
    '''
    Shortcut function to multiply two quaternions q1 and q2
    '''
    return tuple(tf.transformations.quaternion_multiply(q1, q2))
    
def rpy2q(r, p, y, axes=None):
    '''
    Shortcut function to convert rpy to quaternion
    '''
    if axes == None:
        return tuple(tf.transformations.quaternion_from_euler(r, p, y))
    else:
        return tuple(tf.transformations.quaternion_from_euler(r, p, y, axes))

# main
def main():
    rospy.init_node(NODE)
    print "==> started " + NODE
    
    # init
    arm_ik = rospy.ServiceProxy('/arm_kinematics/get_ik', GetPositionIK)
    
    # translation and rotation for main calibration position
    # ------------------------------------------------------
    t_calib         = (-0.670, 0.00, 1.08) ###
    #t_calib         = (-0.65, 0.0, 1.078) ###
    #t_calib_handeye = (-0.777, -0.016, 1.006)
    q_calib         = (0.061, 0.972, 0.166, 0.154) ###
    
    # define translations
    # -------------------

    f=0.5
    t_c  = tadd(t_calib, ( 0.12*f,  0.00,  0.00)) # closer
    t_cr = tadd(t_calib, ( 0.10*f, -0.05*f,  0.00)) # closer right
    t_f  = tadd(t_calib, (-0.05*f,  0.00,  0.00)) # further
    t_f1 = tadd(t_calib, (-0.10*f,  0.00,  0.00)) # further more
    t_r  = tadd(t_calib, ( 0.00, -0.05*f,  0.00)) # right
    t_l  = tadd(t_calib, ( 0.00,  0.05*f,  0.00)) # left
    t_t  = tadd(t_calib, ( 0.00,  0.00,  0.15*f)) # top
    t_b  = tadd(t_calib, ( 0.00,  0.00, -0.15*f)) # bottom
    t_tr = tadd(t_calib, ( 0.00, -0.15*f,  0.15*f)) # top right
    t_tl = tadd(t_calib, ( 0.00,  0.10*f,  0.15*f)) # top left
    t_br = tadd(t_calib, ( 0.00, -0.15*f, -0.15*f)) # bottom right
    t_bl = tadd(t_calib, ( 0.00,  0.10*f, -0.13*f)) # bottom left
    
    # define translations (robot calibration)
    # ---------------------------------------
    t_rob_tr = tadd(t_calib, ( 0.00, -0.10*f,  0.12*f)) # top right
    t_rob_tl = tadd(t_calib, ( 0.00,  0.05*f,  0.12*f)) # top left
    t_rob_br = tadd(t_calib, ( 0.00, -0.10*f, -0.12*f)) # bottom right
    t_rob_bl = tadd(t_calib, ( 0.00,  0.05*f, -0.12*f)) # bottom left
    
    t_rob_rr  = tadd(t_calib, ( 0.05*f, -0.175*f, -0.02*f)) # right right
    t_rob_ll  = tadd(t_calib, ( 0.05*f,  0.175*f, -0.02*f)) # left left
    t_rob_brr = tadd(t_calib, ( 0.05*f, -0.125*f, -0.12*f)) # bottom right right
    t_rob_bll = tadd(t_calib, ( 0.05*f,  0.125*f, -0.12*f)) # bottom left left
    
    t_rob_bb  = tadd(t_calib, ( 0.00,  0.0, -0.25*f)) # bottom bottom
    
    # define quaternions
    # ------------------
    q_a    = qmult(q_calib, rpy2q( pi/6,  0,     0)) # tilt away
    q_as1  = qmult(q_calib, rpy2q( pi/12, 0, 0)) # tilt away side 1
    q_as2  = qmult(q_calib, rpy2q( pi/8, -pi/8,  0)) # tilt away side 2
    q_as2m = qmult(q_calib, rpy2q( pi/8, -pi/4,  0)) # tilt away side 2 more
    q_n    = qmult(q_calib, rpy2q(-pi/6,  0,     0)) # tilt near
    q_ns1  = qmult(q_calib, rpy2q(-pi/8,  0, 0)) # tilt near side 1
    q_ns2  = qmult(q_calib, rpy2q(-pi/8, -pi/8,  0)) # tilt near side 2
    q_ns2m = qmult(q_calib, rpy2q(-pi/6, -pi/8,  0)) # tilt near side 2 more
    
    # generate poses from defined translations and positions
    # ------------------------------------------------------
    poses = {}
    
    # hand eye calibration pose
    poses["calibration"] = (t_calib, q_calib)
    
    # stereo camera intrinsic calibration poses
    poses["intrinsic_00"]  = (t_calib, q_calib)
    poses["intrinsic_01"]  = (t_r, q_as1)
    poses["intrinsic_02"]  = (t_l, q_as2)
    poses["intrinsic_03"]  = (t_calib, q_ns1)
    poses["intrinsic_04"]  = (t_calib, q_ns2)
    poses["intrinsic_05"]  = (t_c, q_a)
    poses["intrinsic_06"]  = (t_cr, q_n)
    poses["intrinsic_07"]  = (t_f1, q_as2m)
    poses["intrinsic_08"]  = (t_f, q_ns2m)
    poses["intrinsic_09"]  = (t_tr, q_as1)
    poses["intrinsic_10"]  = (t_tl, q_as2)
    poses["intrinsic_11"]  = (t_bl, q_as2)
    poses["intrinsic_12"]  = (t_br, q_as1)
    
    # robot calibration poses
    poses["robot_center_00"]  = (t_calib, q_calib)
    poses["robot_center_01"]  = (t_r, q_as1)
    poses["robot_center_02"]  = (t_l, q_as2)
    poses["robot_center_03"]  = (t_calib, q_ns1)
    poses["robot_center_04"]  = (t_calib, q_ns2)
    poses["robot_center_05"]  = (t_c, q_a)
    poses["robot_center_06"]  = (t_c, q_n)
    poses["robot_center_07"]  = (t_f1, q_as2m)
    poses["robot_center_08"]  = (t_f, q_ns2m)
    poses["robot_center_09"]  = (t_rob_tr, q_as1)
    poses["robot_center_10"]  = (t_rob_tl, q_as2)
    poses["robot_center_11"]  = (t_rob_bl, q_as2)
    poses["robot_center_12"]  = (t_rob_br, q_as1)
    
    # NOTE: in keys left and right are defined like the torso
    poses["robot_right_00"]  = (t_rob_ll, q_calib)
    poses["robot_right_01"]  = (t_rob_ll, q_as1)
    poses["robot_right_02"]  = (t_rob_bll, q_as2)

    poses["robot_left_00"]  = (t_rob_rr, q_calib)
    poses["robot_left_01"]  = (t_rob_rr, q_as2)
    poses["robot_left_02"]  = (t_rob_brr, q_as1)
    
    poses["robot_back_00"]  = (t_rob_bb, q_a)
    poses["robot_back_01"]  = (t_rob_bb, q_as2)
    poses["robot_back_02"]  = (t_rob_bb, q_as1)
    
    
    # converting to joint_positions
    # -----------------------------
    print "==> converting poses to joint_states"
    # stable seed for center position
    # IMPORTANT: adjust this to something reasonable if you change the main t_calib, q_calib position
    #prev_state = [0.13771, -1.61107, 1.60103, -0.90346, 2.30279, -1.28408, -0.93369]    #kuka_seed
    prev_state = [1.0936758098900126, -1.4298150472476228, 0.85311792602466141, -1.0861945216896403, 1.3819114386425193, -1.6894261396652632, -0.91464684174966893]
     
    arm_states = {}
    for key in sorted(poses.keys()):
        print "--> calling getIk for '%s'" % key
        
        # query ik server for ik solution:
        # use previous joint angles as inital seed, if this fails 
        # use current arm position and finally try zero position
        for seed in [prev_state, None, [0,0,0,0,0,0,0]]:
            joint_positions = getIk(arm_ik, poses[key], "base_link", seed)
            if joint_positions != None:
                arm_states[key] = [joint_positions]
                # remember current position as prev positions for next ik call
                prev_state = joint_positions
                break
        else: 
            print "--> ERROR no IK solution was found..."
    
    # convert to yaml_string manually (easier to achieve compact notation)
    # --------------------------------------------------------------------
    yaml_string = ""
    for key in sorted(arm_states.keys()):
        # set prcision to 5 digits
        tmp = map(lambda x: "%.5f"%x, arm_states[key][0])
        yaml_string += "%s: [[%s]]\n" % (key, ', '.join(tmp))
    
    # manually add group trajectories
    yaml_string += '''all_intrinsic: ["intrinsic_00", "intrinsic_01", "intrinsic_02", "intrinsic_03", "intrinsic_04", "intrinsic_05", "intrinsic_06", "intrinsic_07", "intrinsic_08", "intrinsic_09", "intrinsic_10", "intrinsic_11", "intrinsic_12"]\n'''
    yaml_string += '''all_robot_center: ["robot_center_00", "robot_center_01", "robot_center_02", "robot_center_03", "robot_center_04", "robot_center_05", "robot_center_06", "robot_center_07", "robot_center_08", "robot_center_09", "robot_center_10", "robot_center_11", "robot_center_12"]\n'''
    yaml_string += '''all_robot_left: ["robot_left_00", "robot_left_01", "robot_left_02"]\n'''
    yaml_string += '''all_robot_right: ["robot_right_00", "robot_right_01", "robot_right_02"]\n'''
    yaml_string += '''all_robot_back: ["robot_back_00", "robot_back_01", "robot_back_02"]\n'''

    # print joint angles
    print "==> RESULT: joint_positions, please add to config/ROBOT/arm_joint_configurations.yaml"
    print yaml_string

#    # DEBUG move arm
#    # --------------
#    sss = simple_script_server()
#    print "==> moving arm"
#    for key in sorted(arm_states.keys()):
#        print "--> moving to '%s'" % key
#        sss.move("arm", arm_states[key])
#        sss.wait_for_input()
   
if __name__ == '__main__':
    main()
    rospy.signal_shutdown(rospy.Time.now())
    print "==> done exiting"
