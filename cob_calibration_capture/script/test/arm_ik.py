#!/usr/bin/env python
PKG  = 'cob_calibration_capture'
NODE = 'arm_ik_node'
import roslib; roslib.load_manifest(PKG)
import rospy

from simple_script_server import simple_script_server
from kinematics_msgs.srv import GetPositionIK, GetPositionIKRequest
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import tf

def getIk(arm_ik, (t, q), link):
    # get joint names for arm from parameter server
    joint_names = None
    try: joint_names = rospy.get_param("arm_controller/joint_names") # real hardware
    except KeyError: pass
    try: joint_names = rospy.get_param("arm_controller/joints")      # simulation
    except KeyError: pass
    if joint_names == None:
        print "Could not get arm joint_names from parameter server. exiting..."
        exit(-1)
        
    # get current joint angles from arm as seed position
    current_joint_angles = []
    for tries in range(10):
        try: msg = rospy.wait_for_message("/joint_states", JointState)
        except rospy.exceptions.ROSInterruptException: pass
        if joint_names[0] in msg.name:
            # message is from arm, save current angles
            for name in joint_names: current_joint_angles.append(msg.position[msg.name.index(name)])
            break
    if current_joint_angles == []:
        print "Could not get /joint_states message from arm controller. Assuming zero seed state..."
        for name in joint_names: current_joint_angles.append(0.0)
    
    # create and send ik request
    req = GetPositionIKRequest()
    req.timeout = rospy.Duration(0.5)
    req.ik_request.ik_link_name = "arm_7_link"
    req.ik_request.ik_seed_state.joint_state.position = current_joint_angles
    req.ik_request.ik_seed_state.joint_state.name = joint_names
    req.ik_request.pose_stamped.header.frame_id = link
    req.ik_request.pose_stamped.pose.position.x = t[0]
    req.ik_request.pose_stamped.pose.position.y = t[1]
    req.ik_request.pose_stamped.pose.position.z = t[2]
    req.ik_request.pose_stamped.pose.orientation.x = q[0]
    req.ik_request.pose_stamped.pose.orientation.y = q[1]
    req.ik_request.pose_stamped.pose.orientation.z = q[2]
    req.ik_request.pose_stamped.pose.orientation.w = q[3]
    resp = arm_ik(req)
    
    if resp.error_code.val == resp.error_code.SUCCESS:
        result = resp.solution.joint_state.position
    else:
        print "Inverse kinematics request failed"
        result = None
    return result

# main
def main():
    rospy.init_node(NODE)
    print "==> started " + NODE
    
    # init
    sss = simple_script_server()
    #arm_ik = rospy.ServiceProxy('/arm_controller/get_ik', GetPositionIK)
    arm_ik = rospy.ServiceProxy('/arm_kinematics/get_ik', GetPositionIK)
    tf_listener = tf.TransformListener()

    # get current pose of arm
    tf_listener.waitForTransform("base_link", "arm_7_link", rospy.Time(), rospy.Duration(1))
    (t, q) = tf_listener.lookupTransform("base_link", "arm_7_link", rospy.Time())
    
    print "--> calling getIk"
    print getIk(arm_ik, (t, q) ,"base_link")
    print getIk(arm_ik, (t+(0.1, 0.5, -0.2), q) ,"base_link")

if __name__ == '__main__':
    main()
    print "==> done exiting"