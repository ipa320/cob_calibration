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
#   ROS package name: cob_image_capture
#
# \author
#   Author: Sebastian Haug, email:sebhaug@gmail.com
# \author
#   Supervised by: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
#
# \date Date of creation: December 2011
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
PKG = 'cob_image_capture'
NODE = 'image_capture'
import roslib
roslib.load_manifest(PKG)
import rospy

import cv
from sensor_msgs.msg import Image
from cob_calibration_srvs.srv import Capture, CaptureResponse
from cv_bridge import CvBridge, CvBridgeError


class ImageCaptureNode():
    '''
    @summary: Captures Images from one or more cameras (Image message topics) to files.

    Number of cameras, output folder and file names are configurable via ROS parameters.
    After startup call "~capture_images" ROS service to save images of all
    cameras to output folder.
    '''

    def __init__(self):
        '''
        Initializes storage, gets parameters from parameter server and logs to rosinfo
        '''
        rospy.init_node(NODE)
        self.bridge = CvBridge()
        self.counter = 0

        # Get params from ros parameter server or use default
        self.cams = rospy.get_param("~cameras")
        self.numCams = int(rospy.get_param("~number_of_cameras", "1"))
        self.output_folder = rospy.get_param("~output_folder", "/tmp")
        self.save_header_stamp = rospy.get_param("~save_header_stamp", "False")
        self.camera = [self.cams['reference']['topic']]

        self.file_prefix = [self.cams['reference']['file_prefix']]
        for cam in self.cams["further"]:
            self.camera.append(cam["topic"])
            self.file_prefix.append(cam["file_prefix"])

        self.numCams = len(self.camera)
        # Init images
        self.image = []
        for id in range(self.numCams):
            self.image.append(Image())

        # Subscribe to images
        self.imageSub = []
        for id in range(self.numCams):
            self.imageSub.append(rospy.Subscriber(
                self.camera[id], Image, self._imageCallback, id))

        # Wait for image messages
        for id in range(self.numCams):
            rospy.wait_for_message(self.camera[id], Image, 5)

        # Report
        rospy.loginfo("started capture process...")
        rospy.loginfo("capturing images from")
        for id in range(self.numCams):
            rospy.loginfo(
                " %s -> files %s*" % (self.camera[id], self.file_prefix[id]))
        rospy.loginfo("to output folder %s" % self.output_folder)

    def _imageCallback(self, data, id):
        '''
        Copy image message to local storage

        @param data: Currently received image message
        @type  data: ROS Image() message
        @param id: Id of camera from which the image was received
        @type  id: integer
        '''
        #print "cb executed"
        self.image[id] = data

    def _convertAndSaveImage(self, rosImage, filenamePrefix, counter):
        '''
        Convert image to cvImage and store to file as jpg image.

        @param rosImage: Image
        @type  rosImage: ROS Image() message
        @param filenamePrefix: Prefix to be prepended to image filename
        @type  filenamePrefix: string
        @param counter: Number to be appended to image filename
        @type  counter: integer
        '''
        # save image
        cvImage = cv.CreateImage((1, 1), 1, 3)
        try:
            cvImage = self.bridge.imgmsg_to_cv(rosImage, "bgr8")
        except CvBridgeError, e:
            print e
        cv.SaveImage(self.output_folder + '/' + filenamePrefix +
                     '%05d.jpg' % counter, cvImage)

        # save header
        if self.save_header_stamp:
            f = open(self.output_folder + '/' +
                     filenamePrefix + '%05d_header.txt' % counter, "w")
            f.writelines(str(rosImage.header))
            f.close()

    def _captureHandle(self, req):
        '''
        Service handle for "Capture" Service
        Grabs images, converts them and saves them to files

        @param req: service request
        @type  req: CaptureRequest() message

        @return: CaptureResponse() message
        '''
        # grab image messages
        localImages = []
        for id in range(self.numCams):
            if self.image[id].header.stamp > rospy.Time(0):
                localImages.append(self.image[id])
                rospy.loginfo("   header.stamp of cam %d: %s" %
                              (id, str(self.image[id].header.stamp)))

        if len(localImages) == 0:
            rospy.loginfo("No sample captured, no image in queue.")
            return CaptureResponse(False)

        # convert and save
        for id in range(self.numCams):
            self._convertAndSaveImage(
                localImages[id], self.file_prefix[id], self.counter)
        self.counter = self.counter + 1

        # log infos
        rospy.loginfo("-> %s image(s) captured | captured %d set(s) of images in total" % (self.numCams, self.counter))

        # return service response
        return CaptureResponse(True)

    def run(self):
        # Start service
        srv = rospy.Service(
            '/image_capture/capture', Capture, self._captureHandle)
        rospy.loginfo("service '/image_capture/capture' started, waiting for requests...")
        rospy.spin()


if __name__ == '__main__':
    node = ImageCaptureNode()
    node.run()
