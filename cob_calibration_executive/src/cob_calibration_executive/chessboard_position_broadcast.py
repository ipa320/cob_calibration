#!/usr/bin/env python
PKG = 'cob_calibration_executive'
NODE = 'chessboard_position_broadcast'
import roslib
roslib.load_manifest(PKG)

import rospy
import tf
from geometry_msgs.msg import PoseStamped


class ChessboardPositionBroadcast():

    def __init__(self):
        self.translation = (0, 0, 0)
        self.rotation = (0, 0, 0, 1)
        self.base = "/base_link"

    def pose_callback(self, data):
        print 'received next pose'
        self.base = data.header.frame_id
        self.translation = (
            data.pose.position.x, data.pose.position.y, data.pose.position.z)
        self.rotation = (data.pose.orientation.x, data.pose.orientation.y,
                         data.pose.orientation.z, data.pose.orientation.w)

    def run(self):
        rospy.init_node(NODE)

        self.pattern_size = rospy.get_param(
            '~pattern_size', "9x6")
        self.square_size = rospy.get_param(
            '~square_size', 0.03)
        self.pattern_size = tuple((int(self.pattern_size.split(
            "x")[0]), int(self.pattern_size.split("x")[1])))

        rospy.Subscriber('/cob_calibration/chessboard_pose',
                         PoseStamped, self.pose_callback)

        br = tf.TransformBroadcaster()
        rate = rospy.Rate(20.0)
        while not rospy.is_shutdown():
            #print self.translation
            br.sendTransform(self.translation,
                             self.rotation,
                             rospy.Time.now(),
                             "/chessboard_position_link",
                             self.base)
            rospy.sleep(0.05)

            br.sendTransform(
                (0, 0.5 * self.pattern_size[0] *
                 self.square_size, self.pattern_size[1] * self.square_size + 0.15),
                (0, 0, 0, 1),
                rospy.Time.now(),
                "/chessboard_ru_corner",
                "/chessboard_position_link")  # right upper corner
            br.sendTransform(
                (0, -0.5 * self.pattern_size[0] *
                 self.square_size, self.pattern_size[1] * self.square_size + 0.15),
                (0, 0, 0, 1),
                rospy.Time.now(),
                "/chessboard_lu_corner",
                "/chessboard_position_link")  # left upper corner

            br.sendTransform(
                (0, -0.5 * self.pattern_size[0] * self.square_size, 0.15),
                (0, 0, 0, 1),
                rospy.Time.now(),
                "/chessboard_ll_corner",
                "/chessboard_position_link")  # left lower corner
            br.sendTransform(
                (0, 0.5 * self.pattern_size[0] * self.square_size, 0.15),
                (0, 0, 0, 1),
                rospy.Time.now(),
                "/chessboard_rl_corner",
                "/chessboard_position_link")  # right lower corner
            br.sendTransform((0, 0, 0.15 + (self.pattern_size[1] * self.square_size) / 2),
                             (0, 0, 0, 1),
                             rospy.Time.now(),
                             "/chessboard_center",
                             "/chessboard_position_link")  # right upper corner
            rate.sleep()


if __name__ == '__main__':
    node = ChessboardPositionBroadcast()
    node.run()
    print "==> Done, exiting"
