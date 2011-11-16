#!/usr/bin/env python
PKG  = 'cob_calibration_capture'
NODE = 'move_arm'
import roslib; roslib.load_manifest(PKG)
import rospy

from simple_script_server import simple_script_server

# main
def main():
    rospy.init_node(NODE)
    print "==> started " + NODE
    
    # init
    sss = simple_script_server()

    # movements
    print "==> starting movements"
    
    print "--> moving arm to pregrasp"
    sss.move("arm", "pregrasp")
    
    print "--> moving arm to calib"
    sss.move("arm", "calib")
    
    print "--> moving arm to 'calib' via 'pregrasp'"
    sss.move("arm", ["pregrasp", "calib"])
    
if __name__ == '__main__':
    main()
    print "==> done exiting"
