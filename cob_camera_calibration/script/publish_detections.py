#!/usr/bin/env python

NODE = "publish_cb_detections"
PKG = "cob_camera_calibration"
import roslib
roslib.load_manifest(PKG)
import rospy
from cob_camera_calibration import Checkerboard, CheckerboardDetector, cv2util
import tf
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from std_msgs.msg import ColorRGBA


class Cb_marker_publish():

    def __init__(self, cb):
        self.cb = cb
        self.detector = CheckerboardDetector(cb)

        self.bridge = CvBridge

        self.pub = rospy.Publisher("cb_detections", Marker)
        self.topics = ["/stereo/left/image_raw",
                       "/stereo/right/image_raw", "/cam3d/rgb/image_raw"]

        self.info_topics = ["/stereo/left/camera_info",
                            "/stereo/right/camera_info",
                            "/cam3d/rgb/camera_info"]

        self.images = {}
        self.camera_infos = {}
        self.colors = [ColorRGBA()] * 3

        self.colors[0].r = 1
        self.colors[1].g = 1
        self.colors[2].b = 1

        self.points = []
        for point in self.cb.get_pattern_points():
            self.points.append()  # TODO

        for topic, info_topic in zip(self.topics, self.info_topics):
            rospy.Subscriber(topic, Image, self.callback_image, topic)
            rospy.Subscriber(topic, CameraInfo, self.callback_info, topic)

    def callback_image(self, data, topic):
        self.images[topic] = data

    def callback_info(self, data, topic):
        self.camera_infos[topic] = data

    def get_pose(self, topic):
        image = self.images(topic)
        cvImage = self.bridge.imgmsg_to_cv(image, 'rgb8')
        img_raw = cv2util.cvmat2np(cvImage)

        camera_matrix = np.matrix(np.reshape(
            self.camera_infos[topic].K, (3, 3)))
        dist_coeffs = np.matrix(self.camera_infos[topic].D)
        return self.detector.calculate_object_pose(img_raw, camera_matrix, dist_coeffs, False)

    def run(self):
        rate = rospy.Rate(10)

        while not rospy.isShutdown():
            # put code here
            msgs = []
            for topic in self.topics:
                msg = Marker()
                msg.type = Marker.SPHERE_LIST
                msg.header.frame_id = self.camera_infos[topic].frame_id
                msg.header.stamp = rospy.Time()
                msg.id = self.topics.index(topic)

                p = self.get_pose(topic)
                msg.pose.position.x = p[1][0]
                msg.pose.position.y = p[1][1]
                msg.pose.position.z = p[1][2]
                rmat = cv2util.cvmat2np(p[0])
                q = tf.transformations.quaternion_from_matrix(rmat)
                msg.pose.orientation.x = q[0]
                msg.pose.orientation.y = q[1]
                msg.pose.orientation.z = q[2]
                msg.pose.orientation.w = q[3]
                c = self.colors(self.topics.index(topic))
                msg.scale.x = 1
                msg.scale.y = 1
                msg.scale.z = 1
                msg.color = c
                msgs.append(msg)

            for msg in msgs:
                self.pub.publish(msg)

            rate.sleep()

if __name__ == "__main__":
    node = Cb_marker_publish(Checkerboard((9, 6), 0.03))
    node.run()
