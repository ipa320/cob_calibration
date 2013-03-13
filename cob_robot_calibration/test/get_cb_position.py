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
#   ROS package name: cob_torso_calibration
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
PKG = 'cob_robot_calibration'
NODE = 'test_calibration_point_node'
import roslib
roslib.load_manifest(PKG)
import rospy

import tf
import numpy as np

from sensor_msgs.msg import CameraInfo, Image
from cob_camera_calibration import Checkerboard, CheckerboardDetector, cv2util
from cv_bridge import CvBridge
from kinematics_msgs.srv import *
from arm_navigation_msgs.srv import *

from simple_script_server import simple_script_server


def prettyprint(array):
    print '*' * 20
    for row in array:
        print row


class TransformationTransform():
    def __init__():
        self.bc = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()

    def sendTransform(transform, frame):
        rot = tf.transformations.quaternion_from_matrix(transform[0])
        self.bc.sendTransform(transform[1], rot, "detected", frame)
        self.bc.sendTransform((0, 0, 0.1), (0, 0, 0, 1), "target", "/detected")

    def getTransform(transform, frame):
        self.sendTransform(transform, frame)

        trans, rot = self.listener.waitForTransform("/target", "/arm_0_link",
                                                    rospy.Time.now(), rospy.Duration(4))

        return trans, rot


class IK():
    def __init__():
        SetPlanningSceneDiffService = rospy.ServiceProxy('/environment_server/set_planning_scene_diff', SetPlanningSceneDiff)

        #sending empty request for triggering planning scene
        planning_scene_request = SetPlanningSceneDiffRequest()
        planning_scene_response = SetPlanningSceneDiffService(
            planning_scene_request)
        #print planning_scene_response
        if not planning_scene_response:
            print "Can't get planning scene!"

    def callIK(pose_stamped, link):
        req = GetPositionIKRequest()
        req.timeout = rospy.Duration(5)
        req.ik_request.ik_link_name = link
        req.ik_request.ik_seed_state.joint_state.name = [
            'arm_%d_joint' % (d + 1) for d in range(7)]
        req.ik_request.ik_seed_state.joint_state.position = [2.4660451412200928, -1.6569322347640991, 1.8512247800827026, 1.3687270879745483,
                                                             -1.1863553524017334, 1.029731035232544, 0.6861133575439453]
        req.ik_request.pose_stamped = pose_stamped
        iks = rospy.ServiceProxy('/cob_ik_wrapper/get_ik', GetPositionIK)
        res = iks(req)
        return res.solution.joint_state.position if res.error_code.val == res.error_code.SUCCESS else None, res.error_code


class Detection():

    def __init__(self):
        '''
        Configures the calibration node
        Reads configuration from parameter server or uses default values
        '''

        # get parameter from parameter server or set defaults
        self.pattern_size = rospy.get_param('~pattern_size', "9x6")
        self.square_size = rospy.get_param('~square_size', 0.03)

        self.alpha = rospy.get_param('~alpha', 0.0)
        self.verbose = rospy.get_param('~verbose', True)

        self.save_result = rospy.get_param('~save_result', False)

        # split pattern_size string into tuple, e.g '9x6' -> tuple(9,6)
        self.pattern_size = tuple((int(self.pattern_size.split(
            "x")[0]), int(self.pattern_size.split("x")[1])))

        self.camera_info = CameraInfo()
        self.camera_info_received = False
        self.latest_image = Image()
        self.bridge = CvBridge()

        # set up Checkerboard and CheckerboardDetector
        self.board = Checkerboard(self.pattern_size, self.square_size)
        self.detector = CheckerboardDetector(self.board)

        self.sss = simple_script_server()

        # subscribe to /cam3d/rgb/camera_info for camera_matrix and distortion coefficients
        rospy.Subscriber('/cam3d/rgb/camera_info', CameraInfo,
                         self.__camera_info_callback__)
        # subscribe to /cam3d/rgb/image_raw for image data
        rospy.Subscriber(
            '/cam3d/rgb/image_color', Image, self.__image_raw_callback__)

        # wait until camera informations are recieved.
        start_time = rospy.Time.now()
        while not (self.camera_info_received or rospy.is_shutdown()):
            rospy.sleep(0.05)
            if start_time + rospy.Duration(2.0) < rospy.Time.now():
                # print warning every 2 seconds if the message is stil missing
                print "--> still waiting for /cam3d/rgb/camera_info"
                start_time = rospy.Time.now()

        # convert camera matrix an distortion coefficients and store them
        camera_matrix = self.camera_info.K
        cm = np.asarray(camera_matrix)
        self.cm = np.reshape(cm, (3, 3))
        dist_coeffs = self.camera_info.D
        self.dc = np.asarray(dist_coeffs)
        self.frame = self.camera_info.frame

        # initialize torso for movement
    def __camera_info_callback__(self, data):
        '''
        executed on new message in /cam3d/rgb/camera_info
        stores received data
        '''
        self.camera_info = data
        self.camera_info_received = True

    def __image_raw_callback__(self, data):
        '''
        executed on new message in /cam3d/rgb/image_raw
        stores newest image
        '''
        self.latest_image = data

    def get_position(self):
        '''
        returns the Y value in subpixel accuracy for the center of the recognized checkerboard
        '''
        r = rospy.Rate(10)  # Hz
        while rospy.Time.now() - self.latest_image.header.stamp > rospy.Duration(1) and not rospy.is_shutdown():
            print 'no image received'
            print rospy.Time.now(), self.latest_image.header.stamp
            r.sleep()
        cvImage = self.bridge.imgmsg_to_cv(self.latest_image, "mono8")
        image_raw = cv2util.cvmat2np(cvImage)
        #image_processed = cv2.undistort(image_raw, self.cm, self.dc)

        #points=self.detector.detect_image_points(image_processed,True)
        (rmat, tvec, image_points) = self.detector.calculate_object_pose_ransac(image_raw, self.cm, self.dc, True)
        #print tvec[1]

        return (rmat, tvec), self.frame


if __name__ == '__main__':

    rospy.init_node(NODE)
    print "==> started " + NODE

    # Detect chessboard
    detect = Detection()
    p, frame = detect.get_position()
    print p, frame

    # calculate pose (10cm in z-direction ahead)
    tft = TransformationTransform()
    quat, trans = tft.getTransform(p, frame)

    # prepare for ik calculation
    ik = IK()
    point = Point(trans)
    orientation = Quaternion(quat)
    pose = Pose(point, orientation)
    h = Header()
    h.frame_id = "/arm_0_link"
    pose_stamped = PoseStamped(h, pose)

    # solve ik
    sol, ec = ik.callIK(pose_stamped, "sdh_tip_link")
    print sol, ec

    # tbd: move arm to target

    print "==> done, exiting"
