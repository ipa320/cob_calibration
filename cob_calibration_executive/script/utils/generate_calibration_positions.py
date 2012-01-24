#!/usr/bin/env python
PKG  = 'cob_calibration_executive'
NODE = 'arm_ik_node'
import roslib; roslib.load_manifest(PKG)
import rospy
import numpy as np
import yaml
from math import pi, sqrt

from simple_script_server import simple_script_server
from kinematics_msgs.srv import GetPositionIK, GetPositionIKRequest
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import tf

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
    # get joint names for arm from parameter server
    joint_names = None
    try: joint_names = rospy.get_param("arm_controller/joint_names") # real hardware
    except KeyError: pass
    try: joint_names = rospy.get_param("arm_controller/joints")      # simulation
    except KeyError: pass
    if joint_names == None:
        print "Could not get arm joint_names from parameter server."
        return None
        
    # if seed == None get current joint angles from arm as seed position
    if seed == None:
        seed = []
        for tries in range(20):
            try: msg = rospy.wait_for_message("/joint_states", JointState)
            except rospy.exceptions.ROSInterruptException: pass
            if joint_names[0] in msg.name:
                # message is from arm, save current angles
                for name in joint_names: seed.append(msg.position[msg.name.index(name)])
                break
        if seed == []:
            print "Could not get /joint_states message from arm controller. "
            return None
    assert len(seed) == len(joint_names)
    
    # create and send ik request
    req = GetPositionIKRequest()
    req.timeout = rospy.Duration(1.0)
    req.ik_request.ik_link_name = "arm_7_link"
    req.ik_request.ik_seed_state.joint_state.position = seed
    req.ik_request.ik_seed_state.joint_state.name = joint_names
    req.ik_request.pose_stamped.header.frame_id = link
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

def tadd(t1, t2):
    '''
    Shortcut function to add two translations t1 and t2
    '''
    return map(lambda (t1x, t2x): t1x+t2x, zip(t1, t2))

def qmult(q1, q2):
    '''
    Shortcut function to multiply two quaternions q1 and q2
    '''
    return tuple(tf.transformations.quaternion_multiply(q1, q2))
    
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
    arm_ik = rospy.ServiceProxy('/arm_kinematics/get_ik', GetPositionIK)

#    # get current pose of arm
#    tf_listener = tf.TransformListener()
#    tf_listener.waitForTransform("base_link", "arm_7_link", rospy.Time(), rospy.Duration(1))
#    (t, q) = tf_listener.lookupTransform("base_link", "arm_7_link", rospy.Time())
    
    # translation and rotation for main calibration position
    t_calib         = (-0.73, 0.0, 1.05)
    t_calib_handeye = (-0.777, -0.016, 1.006)
    q_calib         = (0, 0, sqrt(2), -sqrt(2))
    
    # define translations
    t_c  = tadd(t_calib, ( 0.12,  0.00,  0.00)) # closer
    t_cr = tadd(t_calib, ( 0.10, -0.05,  0.00)) # closer right
    t_f  = tadd(t_calib, (-0.05,  0.00,  0.00)) # further
    t_f1 = tadd(t_calib, (-0.10,  0.00,  0.00)) # further more
    t_r  = tadd(t_calib, ( 0.00, -0.05,  0.00)) # right
    t_l  = tadd(t_calib, ( 0.00,  0.05,  0.00)) # left
    t_t  = tadd(t_calib, ( 0.00,  0.00,  0.15)) # top
    t_b  = tadd(t_calib, ( 0.00,  0.00, -0.15)) # bottom
    t_tr = tadd(t_calib, ( 0.00, -0.15,  0.15)) # top right
    t_tl = tadd(t_calib, ( 0.00,  0.10,  0.15)) # top left
    t_br = tadd(t_calib, ( 0.00, -0.15, -0.15)) # bottom right
    t_bl = tadd(t_calib, ( 0.00,  0.10, -0.13)) # bottom left
    
    # define quaternions
    q_a    = qmult(q_calib, rpy2q( pi/6,  0,     0)) # tilt away
    q_as1  = qmult(q_calib, rpy2q( pi/12, pi/10, 0)) # tilt away side 1
    q_as2  = qmult(q_calib, rpy2q( pi/8, -pi/8,  0)) # tilt away side 2
    q_as2m = qmult(q_calib, rpy2q( pi/8, -pi/4,  0)) # tilt away side 2 more
    q_n    = qmult(q_calib, rpy2q(-pi/6,  0,     0)) # tilt near
    q_ns1  = qmult(q_calib, rpy2q(-pi/8,  pi/10, 0)) # tilt near side 1
    q_ns2  = qmult(q_calib, rpy2q(-pi/8, -pi/8,  0)) # tilt near side 2
    q_ns2m = qmult(q_calib, rpy2q(-pi/6, -pi/8,  0)) # tilt near side 2 more
    
    # generate poses from defined translations and positions
    poses = {}
    # stable seed for center position
    # IMPORTANT: adjust this to something reasonable if you change calib position
    prev_state = [0.13771, -1.61107, 1.60103, -0.90346, 2.30279, -1.28408, -0.93369]
    poses["calib"] = (t_calib, q_calib)
    
    # hand eye calibration pose
    poses["hand_eye"] = (t_calib_handeye, q_calib)
    
    # stereo camera calibration poses
    poses["stereo_00"]  = poses["calib"]
    poses["stereo_01"]  = (t_r, q_as1)
    poses["stereo_02"]  = (t_l, q_as2)
    poses["stereo_03"]  = (t_calib, q_ns1)
    poses["stereo_04"]  = (t_calib, q_ns2)
    poses["stereo_05"]  = (t_c, q_a)
    poses["stereo_06"]  = (t_cr, q_n)
    poses["stereo_07"]  = (t_f1, q_as2m)
    poses["stereo_08"]  = (t_f, q_ns2m)
    poses["stereo_09"]  = (t_tr, q_as1)
    poses["stereo_10"]  = (t_tl, q_as2)
    poses["stereo_11"]  = (t_bl, q_as2)
    poses["stereo_12"]  = (t_br, q_as1)
    
    # converting to joint_positions
    print "==> converting poses to joint_states" 
    arm_states = {}
    for key in sorted(poses.keys()):
        print "--> calling getIk for '%s'" % key
        
        # query ik server for ik solution:
        # use previous joint angles as inital seed, if this fails 
        # use current arm position and finally try zero position
        for seed in [prev_state, None, [0,0,0,0,0,0,0]]:
            joint_positions = getIk(arm_ik, poses[key], "base_link", seed)
            if joint_positions != None:
                arm_states[key] = [joint_positions]
                # remember current position as prev positions for next ik call
                prev_state = joint_positions
                break
        else: 
            print "--> ERROR no IK solution was found..."
    
    # convert to yaml_string manually (easier to achieve compact notation)
    yaml_string = ""
    for key in sorted(arm_states.keys()):
        # set prcision to 5 digits
        tmp = map(lambda x: "%.5f"%x, arm_states[key][0])
        yaml_string += "%s: [[%s]]\n" % (key, ', '.join(tmp))
    yaml_string += '''stereo: ["stereo_00", "stereo_01", "stereo_02", "stereo_03", "stereo_04", "stereo_05", "stereo_06", "stereo_07", "stereo_08", "stereo_09", "stereo_10", "stereo_11", "stereo_12"]'''
    
    # print joint angles
    print "==> RESULT: joint_positions, please add to config/arm_joint_configurations.yaml"
    print yaml_string

#    # DEBUG move arm
#    sss = simple_script_server()
#    print "==> moving arm"
#    for key in sorted(arm_states.keys()):
#        print "--> moving to '%s'" % key
#        sss.move("arm", arm_states[key])
#        sss.sleep(5)
   
if __name__ == '__main__':
    main()
    rospy.signal_shutdown(rospy.Time.now())
    print "==> done exiting"
