#!/usr/bin/env python
PKG  = 'cob_calibration_capture'
NODE = 'move_sdh'
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
    print "--> moving sdh home"
    sss.move("sdh", "home")
    
    print "--> moving sdh calib"
    sss.move("sdh", "calib")
    
if __name__ == '__main__':
    main()
    print "==> done exiting"
