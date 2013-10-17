#!/usr/bin/env python

NODE = "publish_cb_detections"
PKG = "cob_camera_calibration"
import roslib
roslib.load_manifest(PKG)
import rospy
from cob_camera_calibration import Checkerboard, CheckerboardDetector, cv2util
import tf
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from colorsys import hsv_to_rgb

class Cb_marker_publish():

    def __init__(self, cb):
        rospy.init_node(NODE)
        self.cb = cb
        self.detector = CheckerboardDetector(cb)

        self.bridge = CvBridge()

        self.pub = rospy.Publisher("cb_detections", MarkerArray)
        self.topics = ["/stereo/left/image_raw",
                       "/stereo/right/image_raw", "/cam3d/rgb/image_color"]

        self.info_topics = [t.replace("image_raw","camera_info") for t in self.topics]
        self.info_topics = [t.replace("image_color","camera_info") for t in self.info_topics]
        print self.info_topics

        self.images = dict.fromkeys(self.topics)
        self.camera_infos = dict.fromkeys(self.topics)
        self.colors = [ColorRGBA() for i in range(len(self.topics))]
        hue_values = np.linspace(0, 1, len(self.topics)+1)
        for c, hue in zip(self.colors, hue_values):
            c.a = 1
            (c.r, c.g, c.b) = hsv_to_rgb(hue, 1, 1)

        self.points = []
        for point in self.cb.get_pattern_points():
            p = Point()
            p.x = point[0]
            p.y = point[1]
            p.z = point[2]
            self.points.append(p)

        for topic, info_topic in zip(self.topics, self.info_topics):
            rospy.Subscriber(topic, Image, self.callback_image, topic)
            rospy.Subscriber(info_topic, CameraInfo, self.callback_info, topic)
        rospy.sleep(3)

    def callback_image(self, data, topic):
        self.images[topic] = data

    def callback_info(self, data, topic):
        self.camera_infos[topic] = data

    def get_pose(self, topic):
        image = self.images[topic]
        cvImage = self.bridge.imgmsg_to_cv(image, 'rgb8')
        img_raw = cv2util.cvmat2np(cvImage)

        camera_matrix = np.matrix(np.reshape(
            self.camera_infos[topic].K, (3, 3)))
        dist_coeffs = np.matrix(self.camera_infos[topic].D)
        return self.detector.calculate_object_pose(img_raw, camera_matrix, dist_coeffs, False)

    def run(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            # put code here
            msgs = MarkerArray()
            for topic in self.topics:
                # pick color as defined before
                c = self.colors[self.topics.index(topic)]
                if self.images[topic] is None:
                    continue

                try:
                    # compute pose of detected pattern

                    p = self.get_pose(topic)
                except CheckerboardDetector.NoPatternFoundException:
                    rospy.logwarn("No pattern found on topic: '%s'"%topic)
                    continue
                else:
                    rmat = np.array(p[0])
                    rmat = np.append(rmat, [[0,0,0]],0)
                    rmat = np.append(rmat, [[0],[0],[0],[1]],1)
                    q = tf.transformations.quaternion_from_matrix(rmat)
                    msg = Marker()
                    msg.type = Marker.SPHERE_LIST
                    msg.header.frame_id = self.camera_infos[topic].header.frame_id
                    msg.header.stamp = rospy.Time()
                    msg.id = self.topics.index(topic)


                    msg.pose.position.x = p[1][0]
                    msg.pose.position.y = p[1][1]
                    msg.pose.position.z = p[1][2]
                    msg.pose.orientation.x = q[0]
                    msg.pose.orientation.y = q[1]
                    msg.pose.orientation.z = q[2]
                    msg.pose.orientation.w = q[3]
                    msg.scale.x = 0.01
                    msg.scale.y = 0.01
                    msg.scale.z = 0.01
                    msg.color = c
                    msg.points = self.points
                    msgs.markers.append(msg)
            self.pub.publish(msgs)

            rate.sleep()

if __name__ == "__main__":
    node = Cb_marker_publish(Checkerboard((9, 6), 0.03))
    node.run()
