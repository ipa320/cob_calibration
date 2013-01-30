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
NODE = 'collect_data_node'
import roslib
roslib.load_manifest(PKG)
import rospy

import yaml
from sensor_msgs.msg import Image
from cob_calibration_msgs.msg import RobotMeasurement, ChainMeasurement, CameraMeasurement, ImagePoint
from cv_bridge import CvBridge

from cob_calibration_srvs.srv import Capture, CaptureResponse
from cob_camera_calibration import Checkerboard, CheckerboardDetector, cv2util

import tf
CHECKERBOARD_NAME = "cb_9x6"
CHECKERBOARD_CHAIN = "arm_chain"


class DataCollector():
    '''
    @summary: Collects data for robot calibration.

    Subscribes to various topics needed (e.g. images, camera infos, joint angles) and
    provides a service. When service is called, a set of samples is recorded,
    processed (e.g. checkerboards are detected) and combined to a RobotMeasurement message
    which is published as /robot_measurement.
    '''

    def __init__(self):
        '''
        Set up subscribers, publishers and local storage
        '''
        rospy.init_node(NODE)
        print "==> %s started " % NODE

        checkerboard = rospy.get_param("~calibration_pattern")
        self.checkerboard_square_size = checkerboard["square_size"]
        self.checkerboard_pattern_size = (
            int(checkerboard["pattern_size"].split("x")[0]),
            int(checkerboard["pattern_size"].split("x")[1]))
        with open(rospy.get_param("~sensors_yaml"), 'r') as a:
            sensors_yaml = yaml.load(a.read())
        self._get_transformation_links(sensors_yaml)
        self.listener = tf.TransformListener()

       # CvBridge
        self.bridge = CvBridge()

        # initialize private storage
        self._left = {}
        self._left_received = False
        self.counter = 1

        #  init publisher / subscriber
        self._robot_measurement_pub = rospy.Publisher(
            "/robot_measurement", RobotMeasurement)
        self._image_pub_left = rospy.Publisher(
            "/robot_measurement_image_left", Image)  # DEBUG

        # left camera
        rospy.Subscriber(
            rospy.get_param("~cameras")["reference"]["topic"], Image, self._callback_left)

        print "==> done with initialization"

    def _callback_left(self, image_raw):
        '''
        Callback function for left camera message filter
        '''
        #print "DEBUG: callback left"
        self._left["image"] = image_raw
#        if self._left_received == False:
#            print "--> left sample received (this only prints once!)"
        self._left_received = True

    def _get_transformation_links(self, sensors_yaml):
        ## camera_chains
        self.transformations = {}
        for _chain in sensors_yaml["chains"]:
                self.transformations[_chain["chain_id"]
                                     ] = _chain["links"]

    def run(self):
        '''
        Main method, starts service to provide capture functionality
        '''
        rospy.sleep(1)
        # Start service
        rospy.Service('/collect_data/capture', Capture, self._collect)
        rospy.loginfo("service '/collect_data/capture' started, waiting for requests...")
        rospy.spin()

    def _collect(self, data):
        '''
        Executed on service call. Logs and calls _capture_and_pub
        '''
        rospy.loginfo("capturing sample %.2i" % self.counter)
        res = self._capture_and_pub(
            "sample%.2i" % self.counter, CHECKERBOARD_NAME,
            CHECKERBOARD_CHAIN,
            self.checkerboard_pattern_size,
            self.checkerboard_square_size)
        self.counter += 1
        return CaptureResponse(res)

    def _capture_and_pub(self, sample_id, target_id, chain_id, pattern_size, square_size):
        '''
        Main capturing function. Gets a set of recent messages for all needed topics.
        Processes messages and creates RobotMeasuerment message which is published.

        @param sample_id: Sample identifier (e.g. sample01)
        @type  sample_id: string

        @param target_id: Name of checkerboard (e.g. cb_9x6)
        @type  target_id: string

        @param chain_id: Name of dh chain to which checkerboard is attached (e.g. arm_chain)
        @type  chain_id: string

        @param pattern_size: Size of checkerboard pattern as defined by opencv (e.g. (9, 6))
        @type  pattern_size: tuple(x, y)

        @param square_size: Size of checkerboard sqaures (im m)
        @type  square_size: float
        '''
        # capture measurements
        # --------------------
        self._left_received = False
        self._right_received = False
        self._kinect_rgb_received = False
        start_time = rospy.Time.now()
        while (not self._left_received):
            rospy.sleep(0.005)
            # print warning every 2 seconds if one of the messages is still missing...
            if start_time + rospy.Duration(2.0) < rospy.Time.now():
                if not self._left_received:
                    print "--> still waiting for sample from left"
                start_time = rospy.Time.now()
        latest_left = self._left

        self._torso_joint_msg_received = False
        self._arm_joint_msg_received = False
        # set up checkerboard and checkerboard detector
        # ---------------------------------------------
        checkerboard = Checkerboard(pattern_size, square_size)
        checkerboard_detector = CheckerboardDetector(checkerboard)

        # detect cb left
        # --------------
        cvImage = self.bridge.imgmsg_to_cv(latest_left["image"], "mono8")
        image = cv2util.cvmat2np(cvImage)

        corners = checkerboard_detector.detect_image_points(
            image, is_grayscale=True)
        if corners is not None:
            print "cb found: left"
            img_points_left = []
            for (x, y) in corners.reshape(-1, 2):
                img_points_left.append(ImagePoint(x, y))
        else:
            # cb not found
            return False

        # create camera msg left
        # ----------------------
        cam_msg_left = CameraMeasurement()
        cam_msg_left.camera_id = "left"
        cam_msg_left.header.stamp = latest_left["image"].header.stamp
        #cam_msg_left.cam_info = latest_left["camera_info"]
        cam_msg_left.image_points = img_points_left
        cam_msg_left.verbose = False
        #cam_ms_leftg.image        = latest_left["image_color"]
        #cam_msg_left.image_rect   = latest_left["image_rect"]
        #cam_msg_left.features    = # Not implemented here

        # create chain msgs
        # ----------------------
        transformations = []
        print self.transformations
        for (key, links) in self.transformations.iteritems():
            chain_msg = ChainMeasurement()
            chain_msg.chain_id = key
            while not rospy.is_shutdown():
                try:
                    (
                        trans, rot) = self.listener.lookupTransform(links[0], links[1],
                                                                    rospy.Time(0))
                    break
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationExcept):
                    rospy.sleep(0.5)
            chain_msg.translation = trans
            chain_msg.rotation = rot
            transformations.append(chain_msg)
        # DEBUG publish pic
        # -----------------
        self._image_pub_left.publish(latest_left["image"])

        # create robot measurement msg and publish
        # -----------------
        robot_msg = RobotMeasurement()
        robot_msg.sample_id = sample_id
        robot_msg.target_id = target_id
        robot_msg.chain_id = chain_id
        robot_msg.M_cam = [cam_msg_left]
        robot_msg.M_chain = transformations
        self._robot_measurement_pub.publish(robot_msg)

        return True


if __name__ == "__main__":
    collector = DataCollector()
    collector.run()
