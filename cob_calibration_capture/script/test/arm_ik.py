#!/usr/bin/env python
PKG  = 'cob_calibration_capture'
NODE = 'arm_ik_node'
import roslib; roslib.load_manifest(PKG)
import rospy
from math import pi, sqrt

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
        print "Could not get arm joint_names from parameter server."
        return None
        
    # get current joint angles from arm as seed position
    current_joint_angles = []
    for tries in range(20):
        try: msg = rospy.wait_for_message("/joint_states", JointState)
        except rospy.exceptions.ROSInterruptException: pass
        if joint_names[0] in msg.name:
            # message is from arm, save current angles
            for name in joint_names: current_joint_angles.append(msg.position[msg.name.index(name)])
            break
    if current_joint_angles == []:
        print "Could not get /joint_states message from arm controller. "
        return None
    
    # create and send ik request
    req = GetPositionIKRequest()
    req.timeout = rospy.Duration(1.0)
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
        result = list(resp.solution.joint_state.position)
        return result
    else:
        print "Inverse kinematics request failed with error code", resp.error_code.val
        return None

def tadd(t1, t2):
    '''
    Shortcut functionto add two translations t1 and t2
    '''
    return map(lambda (t1x, t2x): t1x+t2x, zip(t1, t2))
    
def rpy2q(r, p, y, axes=None):
    '''
    Shortcut function to convert rpy to quaternion
    '''
    if axes == None:
        return tuple(tf.transformations.quaternion_from_euler(r, p, y))
    else:
        return tuple(tf.transformations.quaternion_from_euler(r, p, y, axes))

# main
def main():
    rospy.init_node(NODE)
    print "==> started " + NODE
    
    # init
    sss = simple_script_server()
    arm_ik = rospy.ServiceProxy('/arm_kinematics/get_ik', GetPositionIK)

#    # get current pose of arm
#    tf_listener = tf.TransformListener()
#    tf_listener.waitForTransform("base_link", "arm_7_link", rospy.Time(), rospy.Duration(1))
#    (t, q) = tf_listener.lookupTransform("base_link", "arm_7_link", rospy.Time())
    
    # translation and rotation for main calibration position
    t_calib = (-0.73, 0.0, 1.05)
    q_calib = (0, 0, sqrt(2), -sqrt(2))
    
    # list of poses
    poses = {}
    poses["right"]        = (tadd(t_calib, (0.00,  0.16,  0.00)), q_calib)
    poses["left"]         = (tadd(t_calib, (0.00, -0.16,  0.00)), q_calib)
    poses["top"]          = (tadd(t_calib, (0.00,  0.00,  0.16)), q_calib)
    poses["bottom"]       = (tadd(t_calib, (0.00,  0.00, -0.16)), q_calib)
    poses["top_right"]    = (tadd(t_calib, (0.00,  0.15,  0.15)), q_calib)
    poses["top_left"]     = (tadd(t_calib, (0.00, -0.15,  0.15)), q_calib)
    poses["bottom_left"]  = (tadd(t_calib, (0.00,  0.15, -0.15)), q_calib)
    poses["bottom_right"] = (tadd(t_calib, (0.00, -0.15, -0.15)), q_calib)
    poses["center"]       = (tadd(t_calib, (0.00,  0.00,  0.00)), q_calib)
    
    # converting to joint_positions
    print "==> converting poses to joint_states" 
    arm_states = {}
    for key in sorted(poses.keys()):
        print "--> calling getIk for '%s'" % key
        joint_positions = getIk(arm_ik, poses[key], "base_link")
        #print ["%.3f" %s for s in joint_positions]
        if joint_positions != None:
            arm_states[key] = [joint_positions]
        else: print "--> ERROR no IK solution was found..."

#    # move arm
#    print "==> moving arm" 
#    sss.move("arm", arm_states["top"])
#    sss.move("arm", arm_states["tilt_center"])
    
if __name__ == '__main__':
    main()
    print "==> done exiting"
