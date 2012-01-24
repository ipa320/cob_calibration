#!/usr/bin/env python
PKG  = 'cob_calibration_executive'
NODE = 'joint_state_listener_torso'
import roslib; roslib.load_manifest(PKG)
import rospy
import sensor_msgs.msg

'''
Print joint states for cob torso.

Listens to /joints_states topic for joint state messages from torso controller
and prints them.
'''
def main():
    rospy.init_node(NODE)
    print "==> %s started " % NODE
    
    # get joint names for arm from parameter server
    joint_names = None
    try: joint_names = rospy.get_param("torso_controller/joint_names") # real hardware
    except KeyError: pass
    try: joint_names = rospy.get_param("torso_controller/joints")      # simulation
    except KeyError: pass
    if joint_names == None:
        print "Could not get joint names from parameter server. exiting..."
        exit(-1)
    print joint_names

    while not rospy.is_shutdown():
        if rospy.is_shutdown(): exit(0)
        
        # try getting /joint_states message
        try:
            msg = rospy.wait_for_message("/joint_states", sensor_msgs.msg.JointState)
        except rospy.exceptions.ROSInterruptException:
            exit(0)
        if joint_names[0] in msg.name:
            # message is from arm
            angles = []
            for name in joint_names:
                angles.append(msg.position[msg.name.index(name)])
            # nicely print joint angles with 8 digits
            print "[" + ", ".join(["%0.8f" % i for i in angles]) + "]"

if __name__ == '__main__':
    main()
