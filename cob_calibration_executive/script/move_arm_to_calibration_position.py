#!/usr/bin/env python
PKG  = 'cob_calibration_executive'
NODE = 'move_arm'
import roslib; roslib.load_manifest(PKG)
import rospy

from simple_script_server import simple_script_server

# main
def main():
    rospy.init_node(NODE)
    print "==> started " + NODE
    sss = simple_script_server()

    # movements
    print "--> moving arm to 'pregrasp'"
    sss.move("arm", "pregrasp")
    
    print "--> moving arm to 'calibration'"
    sss.move("arm", "calibration")
    
if __name__ == '__main__':
    main()
    print "==> done exiting"
