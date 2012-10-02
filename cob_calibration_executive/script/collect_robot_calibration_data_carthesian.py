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
NODE = 'collect_robot_calibration_data_node'
import roslib; roslib.load_manifest(PKG)
import rospy

from simple_script_server import simple_script_server
from cob_calibration_srvs.srv import Capture
from cob_camera_calibration import ViewfieldChecker,Checkerboard,CheckerboardDetector, cv2util

from sensor_msgs.msg import Image
from kinematics_msgs.srv import GetPositionIK, GetPositionIKRequest
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from math import pi, sqrt
from pr2_controllers_msgs.msg import JointTrajectoryControllerState
import tf
import math
import numpy as np


from cv_bridge import CvBridge, CvBridgeError


#board       = Checkerboard(self.pattern_size, self.square_size)
#checkerboard_detector=CheckerboardDetector()
#latest_image=Image()
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
    #print link
    
    # get joint names for arm from parameter server
    joint_names = None
    try: joint_names = rospy.get_param("arm_controller/joint_names") # real hardware
    except KeyError: pass
    try: joint_names = rospy.get_param("arm_controller/joints")      # simulation
    except KeyError: pass
    if joint_names == None:
        print "Could not get arm joint_names from parameter server."
        return None
        
   
    msg = rospy.wait_for_message("/arm_controller/state",JointTrajectoryControllerState)

    if seed is None: seed=msg.actual.positions

    
    # create and send ik request
    req = GetPositionIKRequest()
    req.timeout = rospy.Duration(1.0)
    req.ik_request.ik_link_name = link
    req.ik_request.ik_seed_state.joint_state.position = seed
    req.ik_request.ik_seed_state.joint_state.name = msg.joint_names
    req.ik_request.pose_stamped.header.frame_id = 'arm_0_link'
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

def __image_raw_callback__(data):
    image_raw=data
    
def calibration_object_visible():
    cvImage = self.bridge.imgmsg_to_cv(latest_image, "mono8")
    image_raw = cv2util.cvmat2np(cvImage)
    
    return checkerboard_detector.detect_image_points( image_raw, is_greyscale=True, quick_check=True) != None

        
def calculate_ik(pose,arm_ik):
    via_home=False
    for seed in [None, [0,0,0,0,0,0,0]]:
        joint_positions = getIk(arm_ik, pose, "sdh_palm_link", seed)
        if joint_positions != None:
            if seed is [0,0,0,0,0,0,0]:
                via_home=True
            #print 'Found IK @ %s' % joint_positions
            break
            
    else: 
        print "--> ERROR no IK solution was found..."
    return joint_positions, via_home

def get_cb_pose(listener,base_frame):
    return listener.lookupTransform(base_frame,'/chessboard_position_link',rospy.Time(0))
    
def is_positive(a):
    return True if a>=0 else False

    
    
            
def main():
    rospy.init_node(NODE)
    print "==> %s started " % NODE
    path={}
    #cameras=['kinect_rgb','left','right']
    #frames={'kinect_rgb':'/head_cam3d_frame','left':'/head_color_camera_l_frame','right':'/head_color_camera_r_frame'}
    #for camera in cameras:
    #    path[camera] = rospy.get_param('~camera_%s'%camera,  '$(find cob_calibration_data)/$(env ROBOT)/calibration/cameras/%s.yaml'%camera)
        
    chessboard_pose=rospy.Publisher('/cob_calibration/chessboard_pose',PoseStamped)
    print 'chessboard_pose publisher activated'
    rospy.Subscriber('/cam3d/rgb/image_raw',Image,__image_raw_callback__)
    
    translations={}
    # service client
    image_capture_service_name = "/collect_data/capture"
    capture = rospy.ServiceProxy(image_capture_service_name, Capture)
    rospy.wait_for_service(image_capture_service_name, 1)
    
    print "--> service client for capture images initialized"
    listener=tf.TransformListener()
    #viewfieldCheck=ViewfieldChecker(cameras,path)
    rospy.sleep(1.5)
    arm_ik = rospy.ServiceProxy('/cob_ik_wrapper/arm/get_ik', GetPositionIK)
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
    
    print "--> setup care-o-bot for capture"
    sss.move("head", "back")
    
    #sss.move("arm",[a[0]])
    
    nextPose=PoseStamped()

    
    
    # lissajous like figure for rotation
    cb_tip_offset=3
    cb_tip_positions=[(0,0),(1,0),(1,1),
                      (0,1),(-1,1),(-1,0),
                      (0,0),(1,0),(1,-1),
                      (0,-1),(-1,-1),(0,-1)]
    quaternion=[]
    for cb_tip_p in cb_tip_positions:
        temp=list(cb_tip_p)
        temp.append(cb_tip_offset)
        vector_to=np.matrix(temp)
        vector_from=np.matrix([0,0,1])
        a=vector_to+vector_from
        a=a/np.linalg.norm(a)
        #print a
        '''
        print '*'*20
        print type(a)
        print a.T.shape
        print type(vector_from)
        print vector_from.T.shape
        '''
        w=np.dot(vector_from, a.T)
        vector_from=vector_from.tolist()[0]
        a=a.tolist()[0]
        print vector_from
        print a
        x=vector_from[1]*a[2]-vector_from[2]*a[1]
        y=vector_from[2]*a[0]-vector_from[0]*a[2]
        z=vector_from[0]*a[1]-vector_from[1]*a[0]
        
        quaternion.append(tuple(tf.transformations.quaternion_multiply([0,1,0,0],[x,y,z,w]).reshape(1,4).tolist()[0])) 

    print quaternion
    # movement as spiral around center of cam3d

    
    torso_position_offset={'front': 0.7,'back': 0.7, 'right':0.6,'left':0.6,'home':0.7}
    for key,value in torso_position_offset.items():
        sss.move("torso",key)
        
        nextPose.header.frame_id='/head_cam3d_frame'
        nextPose.pose.position.x=value
        nextPose.pose.position.y=0
        nextPose.pose.position.z=0
        
        # (0,0,0,1) for cob3-6
        nextPose.pose.orientation.x=0
        nextPose.pose.orientation.y=0
        nextPose.pose.orientation.z=0
        nextPose.pose.orientation.w=1
        
        
        segmentation=1
        angle_step=2*math.pi/segmentation
        angle=0
        a=0.01
        
        
        no_valid_solution=[]
        y_dimension=True
        R=rospy.Rate(0.5)
        nsamples=0
        counter=-1
        rounds=0
        steps=0
        last_valid_round=1
        reconfiguration=False
        while not rospy.is_shutdown():
            counter+=1
            
            radius=0.02*angle
            if steps==0:
                rounds+=1
                segmentation*=2
                if segmentation>10:
                    segmentation=10
                steps=segmentation
                angle_step=2*math.pi/segmentation
            
            
            x=math.cos(angle)*radius
            y=0.75*math.sin(angle)*radius
            print '========================'
            print steps
            print 'round: ',rounds
            print 'lastvalidround: ',last_valid_round
           
            #print 'x: ',x
            #print 'y: ',y
            
            nextPose.pose.position.y=x
            nextPose.pose.position.z=y
            
            nextPose.pose.orientation.x=quaternion[counter%len(quaternion)][0]
            nextPose.pose.orientation.y=quaternion[counter%len(quaternion)][1]
            nextPose.pose.orientation.z=quaternion[counter%len(quaternion)][2]
            nextPose.pose.orientation.w=quaternion[counter%len(quaternion)][3]
           
            chessboard_pose.publish(nextPose)
            rospy.sleep(0.5)
            
            #for camera in cameras:            
                #(t,r)=get_cb_pose(listener,frames[camera])
                #translations[camera]=t
                
            #if viewfieldCheck.cb_in_modeled_viewfields(translations):

            (t,r)=get_cb_pose(listener,'/arm_0_link')
            jp=calculate_ik((t,r),arm_ik)
            #print jp
            if jp[0] is not None:
                
                
                # safe reconfiguration of the arm
                try:
                    reconfiguration=(abs(jp[0][0]-old_jp[0])>math.pi/2) or (abs(jp[0][2]-old_jp[2])>math.pi) or (abs(jp[0][4]-old_jp[4])>math.pi)
                except:
                    pass
                
                if jp[1] or reconfiguration: # jp[1] means solution was found with seed "home"
                    # set all tilt joints to 0
                    p=[]
                    for i in range(len(jp[0])):
                        if i%2!=0:
                            p.append(0)
                        else:
                            p.append(old_jp[i])
                    print p
                    sss.move("arm",[p])
                    # apply all new pan joint values
                    p=[]
                    for i in range(len(jp[0])):
                        if i%2!=0:
                            p.append(0)
                        else:
                            p.append(jp[0][i])
                    print p
                    sss.move("arm",[p])
                    rospy.sleep(5)
                    
                    
                # move arm to position
                sss.move("arm",[jp[0]])
                rospy.sleep(3)
                capture()
                
                
                last_valid_round=rounds
                nsamples+=1
                old_jp=jp[0]
                    


            if rounds-2==last_valid_round:
                break
            angle+=angle_step
            
            
            steps-=1
            R.sleep()
    print "Finished after %s samples"%nsamples

    
   
                
    
    
    
   
    
    #print "==> capturing TORSO samples"
    #sss.move("arm", "calibration")
    #capture_loop_torso(torso_positions, sss, capture)

if __name__ == '__main__':
    main()
    print "==> done exiting"
