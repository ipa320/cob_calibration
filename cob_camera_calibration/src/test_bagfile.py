#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       untitled.py
#
#       Copyright 2012 Jannik Abbenseth <fmw-ja@cob-stud-103>
#
#       This program is free software; you can redistribute it and/or modify
#       it under the terms of the GNU General Public License as published by
#       the Free Software Foundation; either version 2 of the License, or
#       (at your option) any later version.
#
#       This program is distributed in the hope that it will be useful,
#       but WITHOUT ANY WARRANTY; without even the implied warranty of
#       MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#       GNU General Public License for more details.
#
#       You should have received a copy of the GNU General Public License
#       along with this program; if not, write to the Free Software
#       Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#       MA 02110-1301, USA.


import roslib
roslib.load_manifest("cob_camera_calibration")
import cv
import rosbag
import rospy
import numpy as np
from cv_bridge import CvBridge,CvBridgeError
from cob_calibration_msgs.msg import RobotMeasurement
import sys


def main(argv):
    try:
        bag = rosbag.Bag(argv)
    except IOError:
        print "Bagfile not found ==> exiting"
        return
    points=[]
    cv_image=None
    bridge=CvBridge()
    cv.NamedWindow("L",1)
    cv.NamedWindow("R",1)
    cv.NamedWindow("3D",1)
    prefix = ["left","right","kinect_"]
    index = 0

    for topic, msg, t in bag.read_messages(['/robot_measurement']):
        print topic

        #if topic=='/robot_measurement_image_left':
            #cv_image_l=bridge.imgmsg_to_cv(msg,'bgr8')
        #if topic=='/robot_measurement_image_right':
            #cv_image_r=bridge.imgmsg_to_cv(msg,'bgr8')
        #if topic=='/robot_measurement_image_kinect_rgb':
            #cv_image_3d=bridge.imgmsg_to_cv(msg,'bgr8')
        cv_image_l=cv.LoadImageM("/tmp/cal/intrinsic/left_%05d.jpg"%index)
        cv_image_r=cv.LoadImageM("/tmp/cal/intrinsic/right_%05d.jpg"%index)
        cv_image_3d=cv.LoadImageM("/tmp/cal/intrinsic/kinect_%05d.jpg"%index)

        if topic=='/robot_measurement':
            points={'l':[],'r':[],'3d':[]}
            for cam in msg.M_cam:
                if cam.camera_id == "cam3d":
                    i = "3d"
                elif cam.camera_id == "left":
                    i = "l"
                elif cam.camera_id == "right":
                    i = "r"
                for point in cam.image_points:
                    points[i].append((point.x, point.y))
            #for point in msg.M_cam[0].image_points:
                #points['l'].append((point.x,point.y))
            #for point in msg.M_cam[1].image_points:
                #points['r'].append((point.x,point.y))
            #for point in msg.M_cam[2].image_points:
                #points['3d'].append((point.x,point.y))


        #found,points=cv.FindChessboardCorners(cv_image,(9,6),cv.CV_CALIB_CB_ADAPTIVE_THRESH)
            found=9*6
            print found
            print points
            cv.DrawChessboardCorners(cv_image_l,(9,6),points['l'],found)
            cv.DrawChessboardCorners(cv_image_r,(9,6),points['r'],found)
            cv.DrawChessboardCorners(cv_image_3d,(9,6),points['3d'],found)
            cv.ShowImage("L",cv_image_l)
            cv.ShowImage("R",cv_image_r)
            cv.ShowImage("3D",cv_image_3d)
            cv.WaitKey(0)
            index+=1


    bag.close();

    return 0
def usage():
    print "Not enough arguments passed in. Specify Bagfile in Paramter"
if __name__ == '__main__':
    if len(sys.argv)!=2: usage()
    else: main("/tmp/cal/cal_measurements.bag")

