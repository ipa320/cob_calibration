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
PKG = 'cob_calibration_executive'
NODE = 'collect_robot_calibration_data_node'
import roslib
roslib.load_manifest(PKG)
import rospy


from kinematics_msgs.srv import GetPositionIK, GetPositionIKRequest
from geometry_msgs.msg import PoseStamped
from control_msgs.msg import JointTrajectoryControllerState
from moveit_msgs.msg import PositionIKRequest
from moveit_msgs.srv import GetPositionIK as GetPositionIKMoveit

import tf
import numpy as np
import yaml
import os
import math
from cob_calibration_executive.torso_ik import TorsoIK

from simple_script_server import simple_script_server

from random import shuffle  #shuffle links to look at

#board       = Checkerboard(self.pattern_size, self.square_size)
#checkerboard_detector=CheckerboardDetector()
#latest_image=Image()
def getIk(arm_ik, (t, q), link, seed=None):
    '''
    query arm_ik server for joint_position which put arm_7_link to pose
    @param arm_ik: arm_ik service proxy
    @param t: translation
    @param q: rotation as quaternion
    @param link: frame in which pose (t, q) is defined
    @param seed: initial joint positions for ik calculation, in None current joint pos of arm is used.
    @return: tuple of joint_positions or None if ik was not found
    '''
    # get joint names for arm from parameter server
    joint_names = None
    try:
        joint_names = rospy.get_param(
            "arm_controller/joint_names")  # real hardware
    except KeyError:
        pass
    try:
        joint_names = rospy.get_param(
            "arm_controller/joints")      # simulation
    except KeyError:
        pass
    if joint_names is None:
        print "Could not get arm joint_names from parameter server."
        return None

    msg = rospy.wait_for_message(
        "/arm_controller/state", JointTrajectoryControllerState)

    if seed is None:
        seed = msg.actual.positions

    # create and send ik request
    req = PositionIKRequest()
    req.group_name = "arm"
    req.timeout = rospy.Duration(1.0)
    req.ik_link_name = link
    #req.ik_link_names = msg.joint_names
    req.robot_state.joint_state.position = seed
    req.robot_state.joint_state.name = msg.joint_names
    req.pose_stamped.header.frame_id = 'base_link'
    req.pose_stamped.pose.position.x = t[0]
    req.pose_stamped.pose.position.y = t[1]
    req.pose_stamped.pose.position.z = t[2]
    req.pose_stamped.pose.orientation.x = q[0]
    req.pose_stamped.pose.orientation.y = q[1]
    req.pose_stamped.pose.orientation.z = q[2]
    req.pose_stamped.pose.orientation.w = q[3]
    req.attempts = 3

    resp = arm_ik(req)

    # report sucess or return None on error
    result = []
    if resp.error_code.val == resp.error_code.SUCCESS:
        for name, position in zip(list(resp.solution.joint_state.name),
                                  list(resp.solution.joint_state.position)):
            if name in msg.joint_names:
                result.append(position)
        return result


def calculate_ik(pose, arm_ik, seed=[0.7, -1.6, 4.4, 0.5, 1.2, 1.5, 3.0]):
    via_home = False
    for seed in [seed]:
        joint_positions = getIk(arm_ik, pose, "sdh_palm_link", seed)
        if joint_positions is not None:
            if seed is [0, 0, 0, 0, 0, 0, 0]:
                via_home = True
            #print 'Found IK @ %s' % joint_positions
            break

    return joint_positions, via_home

def lookat(pose, torso_ik):
    '''
    query arm_ik server for joint_position which put arm_7_link to pose

    @param arm_ik: arm_ik service proxy
    @param t: translation
    @param q: rotation as quaternion
    @param link: frame in which pose (t, q) is defined
    @param seed: initial joint positions for ik calculation, in None current joint pos of arm is used.

    @return: tuple of joint_positions or None if ik was not found
    '''

    # get joint names for arm from parameter server
    t=pose[0]
    q = pose[1]

    # create and send ik request
    req = GetPositionIKRequest()
    req.timeout = rospy.Duration(1.0)
    req.ik_request.ik_link_name = "lookat_focus_frame"
    req.ik_request.pose_stamped.header.frame_id = 'base_link'
    req.ik_request.pose_stamped.pose.position.x = t[0]
    req.ik_request.pose_stamped.pose.position.y = t[1]
    req.ik_request.pose_stamped.pose.position.z = t[2]
    req.ik_request.pose_stamped.pose.orientation.x = q[0]
    req.ik_request.pose_stamped.pose.orientation.y = q[1]
    req.ik_request.pose_stamped.pose.orientation.z = q[2]
    req.ik_request.pose_stamped.pose.orientation.w = q[3]

    # try to get inverse kinecmatics for at least 3 times
    for i in range(3):
        resp = torso_ik(req)
        if resp.error_code.val == resp.error_code.SUCCESS:
            break

    # report sucess or return None on error
    if resp.error_code.val == resp.error_code.SUCCESS:
        result = list(resp.solution.joint_state.position)
        return result


def get_cb_pose_center(listener, base_frame):
    return listener.lookupTransform(base_frame, '/chessboard_center', rospy.Time(0))


def get_cb_pose(listener, base_frame):
    return listener.lookupTransform(base_frame, '/chessboard_position_link', rospy.Time(0))

def get_position(listener, tip):
    return listener.lookupTransform('/base_link', tip, rospy.Time(0))

def main():
    rospy.init_node(NODE)
    print "==> %s started " % NODE

    chessboard_pose = rospy.Publisher(
        '/cob_calibration/chessboard_pose', PoseStamped)
    print 'chessboard_pose publisher activated'
    listener = tf.TransformListener()
    rospy.sleep(1.5)
    arm_ik = rospy.ServiceProxy('/compute_ik', GetPositionIKMoveit)
    torso_ik = rospy.ServiceProxy('/lookat_get_ik', GetPositionIK)
    '''
    (t,r)=listener.lookupTransform('/arm_0_link','arm_7_link',rospy.Time(0))

    a=calculate_ik((t,r), arm_ik)
    print a[0]
    '''

    # init
    print "--> initializing sss"
    sss = simple_script_server()
    sss.init("base")
    sss.init("torso")
    sss.init("head")
    sss.recover("base")
    sss.recover("torso")
    sss.recover("head")
    camera_link = "/cam_reference_link"

    [xhead, yhead, zhead] = get_position(listener, camera_link)[0]
    print xhead, yhead, zhead

    print "--> setup care-o-bot for capture"
    sss.move("head", "back")

    calibration_seed = rospy.get_param("/script_server/arm/calibration")

    #sss.move("arm",[a[0]])
    nextPose = PoseStamped()

    torso = TorsoIK()

    # define cuboid for positions
    # limits from base_link frame
    limits = {'x': (-0.4, -1.0),
              'y': (-0.3, 0.3),
              'z': (0.5, 1.5)}

    sample_density = {'x': 6,
                      'y': 6,
                      'z': 6}

    sample_positions = {'x': [],
                        'y': [],
                        'z': []}
    for key in limits.keys():
        limits[key] = sorted(limits[key])
        sample_positions[key].append(limits[key][0])
        diff = limits[key][1] - limits[key][0]
        step = 1.0 * diff / (sample_density[key] - 1)
     #   print key, ' ',diff,' ',step

        while sample_positions[key][-1] + step <= (limits[key][1] + 0.01):
            sample_positions[key].append(sample_positions[key][-1] + step)

    joint_states = []
    torso.get_torso_limits()

    cb_links = ["/chessboard_center","/chessboard_lu_corner",
                "/chessboard_ru_corner", "/chessboard_ll_corner",
                "/chessboard_rl_corner"]

    for x in sample_positions['x']:
        for y in sample_positions['y']:
            for z in sample_positions['z']:
                #for q in quaternion:
                for cb_link in cb_links:
                    print "\033[1;34mNew Position\033[1;m"
                    nextPose.header.frame_id = '/base_link'
                    nextPose.pose.position.x = x
                    nextPose.pose.position.y = y
                    nextPose.pose.position.z = z

                    # (0,0,0,1) for cob3-6
                    nextPose.pose.orientation.x = 0
                    nextPose.pose.orientation.y = 0
                    nextPose.pose.orientation.z = 0
                    nextPose.pose.orientation.w = 1

                    chessboard_pose.publish(nextPose)
                    rospy.sleep(0.2)
                    transformation_base_cb = get_position(
                        listener, '/chessboard_center')
                    [x1, y1, z1] = transformation_base_cb[0]
                    (dx, dy, dz) = (x1 - xhead, y1 - yhead, z1 - zhead)
                    roll = 0
                    pitch = math.atan2(dz, -dx)
                    yaw = -math.atan2(dy, -dx)
                    '''
                    Add noise to roll pitch and yaw values of cb
                    '''
                    std_dev = 0.3
                    roll = np.random.normal(roll,std_dev,1)[0]
                    pitch = np.random.normal(pitch,std_dev,1)[0]
                    yaw = np.random.normal(yaw,std_dev,1)[0]

                    q=tf.transformations.quaternion_from_euler(roll, pitch, yaw)
                    #print q
                    nextPose.pose.orientation.x = q[0]
                    nextPose.pose.orientation.y = q[1]
                    nextPose.pose.orientation.z = q[2]
                    nextPose.pose.orientation.w = q[3]
                    chessboard_pose.publish(nextPose)
                    rospy.sleep(0.2)
                    transformation_base_cb = get_position(
                        listener, cb_link)


                    #if not torso.in_range(angles):
                        #continue
                    #print t
                    try:
                        torso_js = lookat(transformation_base_cb ,torso_ik)[:len(torso.limits)]
                    except:
                        break

                    if torso_js[0] is None:
                        break
                    if not torso.valid_ik(torso_js):
                        break
                    print '\033[1;33mTorso solution found\033[1;m'


                    #(t, r) = get_cb_pose(listener, '/head_cam3d_link')

                    (t, r) = get_cb_pose(listener, '/base_link')
                    js = calculate_ik((
                        t, r), arm_ik, calibration_seed[0])
                    if js[0] is not None:
                        joint_states.append({'joint_position': js[0]
                                            , 'torso_position': list(torso_js)})
                        print joint_states[-1]

                        print '\033[1;32mIK solution found\033[1;m'
                    else:
                        print '\033[1;31mNo solution found\033[1;m'

    path = rospy.get_param('~output_path', None)
    directory = os.path.dirname(path)

    if path is not None:
        if not os.path.exists(directory):
            os.makedirs(directory)
        with open(path, 'w') as f:
            f.write('# autogenerated: Do not edit #\n')
            f.write(yaml.dump(joint_states))
    else:
        print yaml.dump(joint_states)
    print '%s ik solutions found' % len(joint_states)


def calculate_angles(t):
    '''
    computes pan and tilt angles for camera like translations
    z-axis: optical axis
    y-axis: vertical
    x-axis: horizontal
    '''
    angles = {}
    angles['p'] = np.arctan2(t[0], t[2])
    angles['t'] = np.arctan2(t[1], t[2])
    return angles

if __name__ == '__main__':
    main()
    print "==> done exiting"
