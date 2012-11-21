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
import yaml
import os


from cv_bridge import CvBridge, CvBridgeError


#board       = Checkerboard(self.pattern_size, self.square_size)
#checkerboard_detector=CheckerboardDetector()
#latest_image=Image()
def getIk(arm_ik, (t,q), link, seed=None):
    '''
    query arm_ik server for joint_position which put arm_7_link to pose 
    
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

        
def calculate_ik(pose,arm_ik,seed=None):
    via_home=False
    for seed in [seed,None, [0,0,0,0,0,0,0]]:
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
   # path={}
    #cameras=['kinect_rgb','left','right']
    #frames={'kinect_rgb':'/head_cam3d_frame','left':'/head_color_camera_l_frame','right':'/head_color_camera_r_frame'}
    #for camera in cameras:
    #    path[camera] = rospy.get_param('~camera_%s'%camera,  '$(find cob_calibration_data)/$(env ROBOT)/calibration/cameras/%s.yaml'%camera)
        
    chessboard_pose=rospy.Publisher('/cob_calibration/chessboard_pose',PoseStamped)
    print 'chessboard_pose publisher activated'
    #rospy.Subscriber('/cam3d/rgb/image_raw',Image,__image_raw_callback__)
    
    #translations={}
    # service client
    #image_capture_service_name = "/collect_data/capture"
    #capture = rospy.ServiceProxy(image_capture_service_name, Capture)
    #rospy.wait_for_service(image_capture_service_name, 1)
    
    #print "--> service client for capture images initialized"
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
    
    
    
    '''
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
    '''
    nextPose=PoseStamped()

    
    
    # lissajous like figure for rotation
    cb_tip_offset=3
    cb_tip_positions=[(0,0),(1,0),(0,1),
                      (-1,0),(0,-1)]
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
        
        quaternion.append(tuple(tf.transformations.quaternion_multiply([0,0,0,1],[x,y,z,w]).reshape(1,4).tolist()[0])) 


    # define cuboid for positions
    # limits from base_link frame
    limits={'x':(-0.5,-1.2),
            'y':(-0.3,0.3),
            'z':(0.4,1.0)}

    
    sample_density={'x':4,
                    'y':4,
                    'z':4}
                    
                    
                    
    sample_positions={  'x':[],
                        'y':[],
                        'z':[]}
    for key in limits.keys():
        limits[key]=sorted(limits[key])
        sample_positions[key].append(limits[key][0])
        diff=limits[key][1]-limits[key][0]
        step=1.0*diff/(sample_density[key]-1)
     #   print key, ' ',diff,' ',step
        
        while sample_positions[key][-1]+step <=(limits[key][1]+0.01):
            sample_positions[key].append(sample_positions[key][-1]+step)
        
    #print sample_positions
    #return
    joint_states=[]
    
    for x in sample_positions['x']:
               
        for y in sample_positions['y']:
            ll=1.0*limits['y'][0]
            diff=limits['y'][1]-ll
            y_sector=4

            if y<(ll+diff/4):
                y_sector=1
            elif y<(ll+diff/2):
                y_sector=2
            elif y<(ll+diff*3/4):
                y_sector=3

                
                
            for z in sample_positions['z']:
                ll=1.0*limits['z'][0]
                diff=limits['z'][1]-ll
                z_sector=4
                if z<(ll+diff/4):
                    z_sector=1
                elif z<(ll+diff/2):
                    z_sector=2
                elif z<(ll+diff*3/4):
                    z_sector=3
                
                for q in quaternion:
                    nextPose.header.frame_id='/base_link'
                    nextPose.pose.position.x=x
                    nextPose.pose.position.y=y
                    nextPose.pose.position.z=z
                    
                    # (0,0,0,1) for cob3-6
                    nextPose.pose.orientation.x=q[0]
                    nextPose.pose.orientation.y=q[1]
                    nextPose.pose.orientation.z=q[2]
                    nextPose.pose.orientation.w=q[3]
                    
                    chessboard_pose.publish(nextPose)
                    rospy.sleep(0.01)
                    (t,r)=get_cb_pose(listener,'/arm_0_link')
                    try:
                        js=calculate_ik((t,r),arm_ik,joint_states[-1]['joint_position'])
                    except IndexError:
                        js=calculate_ik((t,r),arm_ik)
                    if js[0] is not None:
                        print 'IK solution found'
                        torso_sector_y=[]
                        if y_sector in [3,4]: torso_sector_y.append('right') 
                        if y_sector in [2,3]: torso_sector_y.append('center')
                        if y_sector in [1,2]: torso_sector_y.append('left')
                        
                        
                        torso_sector_z=[]
                        if z_sector in [3,4]: torso_sector_z.append('top') 
                        if z_sector in [2,3]: torso_sector_z.append('center')
                        if z_sector in [1,2]: torso_sector_z.append('low')
                        
                        joint_states.append({'joint_position':js[0],'y_sector':torso_sector_y,'z_sector':torso_sector_z} )
    path=rospy.get_param('output_path',None)
    directory=os.path.dirname(path)
    
    
    if path is not None:
        if not os.path.exists(directory):
            os.makedirs(directory)
        with open(path,'w') as f:
            f.write(yaml.dump(joint_states))
    else:
        print yaml.dump(joint_states)
    print '%s ik solutions found'%len(joint_states)

  

if __name__ == '__main__':
    main()
    print "==> done exiting"
