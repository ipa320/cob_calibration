#!/usr/bin/env python
#
# \file
#
# \note
#   Copyright (c) 2011-2012 \n
#   Fraunhofer Institute for Manufacturing Engineering
#   and Automation (IPA) \n\n
#
#
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
#
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
#
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

from pr2_controllers_msgs.msg import JointTrajectoryControllerState

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
        # self._get_transformation_links(sensors_yaml)
        self._create_transformation_callbacks(sensors_yaml)
        #self.listener = tf.TransformListener()

       # CvBridge
        self.bridge = CvBridge()

        # initialize private storage
        self._images = {}
        self._images_received = {}

        self.counter = 1

        #  init publisher / subscriber
        self._robot_measurement_pub = rospy.Publisher(
            "/robot_measurement", RobotMeasurement)
 

        # left camera
        rospy.Subscriber(
            rospy.get_param("~cameras")["reference"]["topic"],
            Image,
            self._callback_image,
            rospy.get_param("~cameras")["reference"]["name"])
        self._images[rospy.get_param("~cameras")["reference"]["name"]] = {}
        self._images_received[rospy.get_param(
            "~cameras")["reference"]["name"]] = False
        for camera in rospy.get_param("~cameras")["further"]:
            rospy.Subscriber(
                camera["topic"],
                Image,
                self._callback_image,
                camera["name"])
            self._images[camera["name"]] = {}
            self._images_received[camera["name"]] = True
        print "==> done with initialization"

    def _callback_image(self, image_raw, id):
        '''
        Callback function for left camera message filter
        '''
        # print "DEBUG: callback left"
        self._images[id]["image"] = image_raw
        # if self._left_received == False:
            # print "--> left sample received (this only prints once!)"
        self._images_received[id] = True

    def _create_transformation_callbacks(self, sensors_yaml):
        # kinematic chains

        self.transformations = {}
        self.transformations_received = {}
        for _chain in sensors_yaml["chains"]:
            rospy.Subscriber(_chain[
                             "topic"], JointTrajectoryControllerState, self._callback_jointstate, _chain["chain_id"])
	    self.transformations_received[_chain["chain_id"]]=False

    def _callback_jointstate(self, data, id):
        self.transformations[id] = data
        self.transformations[id].header.frame_id = id
	self.transformations_received[id] = True


    def run(self):
        '''
        Main method, starts service to provide capture functionality
        '''
        rospy.sleep(1)
        # Start service
        rospy.Service('/collect_data/capture', Capture, self._collect)
        rospy.loginfo(
            "service '/collect_data/capture' started, waiting for requests...")
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
        # create robot measurement msg and publish
        # -----------------
        robot_msg = RobotMeasurement()
        robot_msg.sample_id = sample_id
        robot_msg.target_id = target_id
        robot_msg.chain_id = chain_id


        # --------------------
        # receive images
        # -----------------
        for v in self._images_received:
            self._images_received[v] = False
        for v in self.transformations_received:
            self.transformations_received[v] = False
        print self._images_received



        start_time = rospy.Time.now()
        while (not all(self._images_received.values()) or not all(self.transformations.values())):
            rospy.sleep(0.005)
            # print warning every 2 seconds if one of the messages is still
            # missing...
            if start_time + rospy.Duration(2.0) < rospy.Time.now():
                for name, v in self._images_received.iteritems():
                    if not v:
                        print "--> still waiting for sample from %s"%name
                for name, v in self._transformations_received.iteritems():
                    if not v:
                        print "--> still waiting for sample from %s"%name
            start_time = rospy.Time.now()
        print "got sample"
        #latest_left = self._left

        # set up checkerboard and checkerboard detector
        # ---------------------------------------------
        checkerboard = Checkerboard(pattern_size, square_size)
        checkerboard_detector = CheckerboardDetector(checkerboard)

        # detect cb
        # --------------
        for name, image in self._images.iteritems():
            image = image["image"]
	    #print image.header
            cvImage = self.bridge.imgmsg_to_cv(image, "mono8")
            imagecv = cv2util.cvmat2np(cvImage)

            try:
                corners = checkerboard_detector.detect_image_points(
                    imagecv, is_grayscale=True)
            except:
                # cb not found
                rospy.logwarn("No calibration pattern found for: '%s'"%name)
                return False
            else:
                print "cb found: %s"%name
                img_points = []
                for (x, y) in corners.reshape(-1, 2):
                    img_points.append(ImagePoint(x, y))

            # create camera msg left
            # ----------------------
            cam_msg = CameraMeasurement()
            cam_msg.camera_id = name
	    #print "crash before"
            cam_msg.header.stamp = image.header.stamp
	    #print "crash after"
            #cam_msg.cam_info = image["camera_info"]
            cam_msg.image_points = img_points
            cam_msg.verbose = False
            robot_msg.M_cam.append(cam_msg)
            cam_msg.image        = image
	    # print cam_msg.camera_id
	    #print cam_msg.header
            # cam_msg.image_rect   = latest_left["image_rect"]
            # cam_msg.features    = # Not implemented here

        #----------------------
        #DEBUG publish pic
        #-----------------
        #self._image_pub_left.publish(latest_left["image"])

        #----------------------
        # Fill robot_msg
        #----------------------
        robot_msg.M_chain = self.transformations.values()

        self._robot_measurement_pub.publish(robot_msg)

        return True


if __name__ == "__main__":
    collector = DataCollector()
    collector.run()
