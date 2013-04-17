#!/usr/bin/env python
PKG='cob_calibration_executive'
NODE='kinematics_test_node'
import roslib; roslib.load_manifest(PKG)


import sys

import rospy
from kinematics_msgs.srv import GetPositionFK
from arm_navigation_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

import tf

from pr2_controllers_msgs.msg import JointTrajectoryControllerState



class getFK_Kinematics():
    def __init__(self):
        rospy.init_node(NODE, anonymous=True)
        print "initialized"
        self.js=JointTrajectoryControllerState()
        self.js_received=False
    
    
    def callback(self,data):
        self.js=data
        self.js_received=True
    
    
    def getFK(self,link, service_name='/cob_ik_wrapper/arm/get_fk'):
        
        
        header=Header()
        rs=RobotState()
        
        
        
        
        print "- Try to get JointStates"
    
       
        rospy.Subscriber("/arm_controller/state",JointTrajectoryControllerState,self.callback)
        i=0
        while not self.js_received:
            i+=1
            rospy.sleep(0.1)
            if i==10: 
                i=0
                print "[DEBUG] still waiting for message"

        
        
        joint_states=JointState()
        joint_states.header=self.js.header
        joint_states.name=self.js.joint_names
        joint_states.position=self.js.actual.positions
        
        #print joint_states

        print "- wait for service %s" % service_name
        rospy.wait_for_service(service_name)
        
        print '- generate header'
        header=joint_states.header
        header.frame_id='arm_0_link'
        #print header
        
        print '- generate fk_link_names'
        fk_link_name=['arm_7_link']
        #print fk_link_names
        
        
        print 'generate RobotState message'
        rs.joint_state=joint_states
        #print rs
        
        
        
        arm_fk= rospy.ServiceProxy(service_name, GetPositionFK)
        print '*'*10,' Solution ','*'*10
        try:
            print arm_fk(header,fk_link_name,rs)
            return arm_fk(header,fk_link_name,rs)
        except e:
            print "Service call failed due to error %s" % e
        
def getFK_tf():


    listener = tf.TransformListener()
    rospy.sleep(1.5)
    (trans,rot) = listener.lookupTransform('/arm_0_link', '/arm_7_link', rospy.Time(0))
    print '='*25
    print trans
    print '+'*10
    print rot
    


if __name__ == "__main__":

    print "Requesting FK"
    getFK_Kinematics().getFK("arm_7_link")
    getFK_tf()

