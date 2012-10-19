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
PKG  = 'cob_calibration_executive'
NODE = 'collect_robot_calibration_data_node'
import roslib; roslib.load_manifest(PKG)
import rospy

from simple_script_server import simple_script_server
from cob_calibration_srvs.srv import *
import numpy as np
import yaml

#TODO: implement visible service 
def capture_loop(positions, sss,visible, capture_kinematics, capture_image):
    '''
    Moves arm to all positions using script server instance sss 
    and calls capture() to capture samples
    '''
    
    counter_camera=0
    counter_kinematics=0
    for index in range(len(positions)):
        print "--> moving arm to sample #%s" % index
        pos=positions[index]
        print pos
        sss.move_planned("arm",  [pos['joint_position']])

        #TODO: get limits from robot_description
        left=np.matrix([0,-0.125,0])

        right=left*-1

        front=np.matrix([-0.075,0,-0.1])
        back=front*-1

        for lr in pos['y_sector']:
            
            torso_joint=np.matrix([0,0,0])
            if lr=='right': 
                torso_joint=torso_joint+right
                
            elif lr=='left':  
                torso_joint=torso_joint+left
                
            
            for fb in pos['z_sector']:
                
                if fb=='top': 
                    torso_joint_final=torso_joint+front
                elif fb=='low': 
                    torso_joint_final=torso_joint+back
                elif fb=='center':
                    torso_joint_final=torso_joint
                
                
                print '*'*20
                print lr
                print fb
                print torso_joint_final
                

                sss.move("torso",torso_joint_final.tolist())
        
                sss.sleep(2)
                print visible(3).visible
                
                if visible(3).visible:
                    print "3 Checkerboards found"
                    capture_kinematics()
                    capture_image()
                    print "--> captured 1 sample for camera calibration"
                    print "--> captured 1 sample for kinematics calibration"
                    counter_camera+=1
                    counter_kinematics +=1
                elif visible(2).visible:
                    print "2 Checkerboards found"
                    capture_kinematics()
                    print "--> captured 1 sample for kinematics calibration"
                    counter_kinematics +=1
                    
                #capture()

def main():
    rospy.init_node(NODE)
    print "==> %s started " % NODE
    
    
    
    
    # service client
    
    checkerboard_checker_name  = "/image_capture/visibility_check"
    visible = rospy.ServiceProxy(checkerboard_checker_name, Visible)
    rospy.wait_for_service(checkerboard_checker_name, 1)
    print "--> service client for for checking for chessboards initialized"
    
    kinematics_capture_service_name = "/collect_data/capture"
    capture_kinematics = rospy.ServiceProxy(kinematics_capture_service_name, Capture)
    rospy.wait_for_service(kinematics_capture_service_name, 1)
    print "--> service client for capture robot_states initialized"
    
    
    image_capture_service_name = "/image_capture/capture"
    capture_image = rospy.ServiceProxy(image_capture_service_name, Capture)
    rospy.wait_for_service(image_capture_service_name, 1)
    print "--> service client for capture images initialized"

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

    # get position from parameter server
    position_path=rospy.get_param('position_path',None)
    if position_path==None:
        print "[ERROR]: no path for positions set"
        return
    with open(position_path,'r') as f:
        positions=yaml.load(f)
    print "==> capturing samples"
    start=rospy.Time.now()
    capture_loop(positions,sss,visible, capture_kinematics,capture_image)
    print "finished after %s seconds"%(rospy.Time.now()-start).to_sec()
    

if __name__ == '__main__':
    main()
    print "==> done exiting"
