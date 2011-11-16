#!/usr/bin/env python
PKG  = 'cob_calibration_capture'
NODE = 'move_arm'
import roslib; roslib.load_manifest(PKG)
import rospy

from simple_script_server import simple_script_server
from kinematics_msgs.srv import *
from geometry_msgs.msg import PoseStamped
import tf

def getIk(arm_ik, tf_listener, goal_pose, link):
    # get seed joint state (pregrasp assumend here)
    arm_pre_grasp = rospy.get_param("/script_server/arm/pregrasp")
    joint_names = rospy.get_param("arm_controller/joint_names");
    print arm_pre_grasp[0]
    
    # create ik request
    req = GetPositionIKRequest()
    req.ik_request.ik_link_name = link
    req.ik_request.ik_seed_state.joint_state.position = arm_pre_grasp[0]
    req.ik_request.ik_seed_state.joint_state.name = joint_names
    req.ik_request.pose_stamped = goal_pose
    resp = arm_ik(req)
    result = []
    for o in resp.solution.joint_state.position:
        result.append(o)
    return (result, resp.error_code)

# main
def main():
    rospy.init_node(NODE)
    print "==> started " + NODE
    
    # init
    sss = simple_script_server()
    arm_ik = rospy.ServiceProxy('/arm_controller/get_ik', GetPositionIK)
    tf_listener = tf.TransformListener()

    # get current pose of arm
    tf_listener.waitForTransform("base_link", "arm_7_link", rospy.Time(), rospy.Duration(1))
    (t, q) = tf_listener.lookupTransform("base_link", "arm_7_link", rospy.Time())
    print (t, q)
    
    # create goal pose and query ik server
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = "base_link"
    goal_pose.pose.position.x = t[0]
    goal_pose.pose.position.y = t[1]
    goal_pose.pose.position.z = t[2]
    goal_pose.pose.orientation.x = q[0]
    goal_pose.pose.orientation.y = q[1]
    goal_pose.pose.orientation.z = q[2]
    goal_pose.pose.orientation.w = q[3]
    print goal_pose
    print getIk(arm_ik, tf_listener, goal_pose ,"base_link")

if __name__ == '__main__':
    main()
    print "==> done exiting"